#include "livo_backend/livo_backend.h"

/************************************************************************/
/*  LIOBackend 实现                                                      */
/************************************************************************/

LIOBackend::LIOBackend(ros::NodeHandle &nh)
    : nh_(nh), keyframe_counter_(0), isam2_initialized_(false), first_livo_received_(false) {
    readParameters(nh);
    initializePublishers(nh);
    initializeISAM2();
    initializeFactorModules(nh);

    run_output_dir_ = createTimestampedDir(std::string(ROOT_DIR) + "Log");
    ROS_INFO_STREAM("[Backend] Run output directory: " << run_output_dir_);

    optimized_path_.header.frame_id = "camera_init";

    ROS_INFO("[Backend] Initialization complete. Waiting for data...");
}

LIOBackend::~LIOBackend() {
    ROS_INFO_STREAM("[Backend] Shutting down. Total keyframes: " << keyframes_.size()
                      << ", raw poses: " << raw_poses_.size());
    if (!run_output_dir_.empty()) {
        saveTrajectory(run_output_dir_ + "/backend_optimized_traj.txt");
        saveFullTrajectory(run_output_dir_ + "/backend_full_traj.txt");
    }
}

void LIOBackend::readParameters(ros::NodeHandle &nh) {
    std::string livo_topic, gnss_topic, wheel_topic;
    nh.param<std::string>("livo_odom_topic", livo_topic, std::string("/aft_mapped_to_init"));
    nh.param<std::string>("gnss_topic", gnss_topic, std::string("/gnss_data"));
    nh.param<std::string>("wheel_topic", wheel_topic, std::string("/wheel_odometry"));

    nh.param<int>("keyframe_skip", keyframe_skip_, 5);
    nh.param<double>("keyframe_distance_threshold", keyframe_distance_threshold_, 0.5);
    nh.param<double>("keyframe_angle_threshold", keyframe_angle_threshold_, 0.3);

    nh.param<double>("noise/livox_pos_cov", livox_pos_cov_, 0.1);
    nh.param<double>("noise/livox_rot_cov", livox_rot_cov_, 0.05);

    nh.param<bool>("enable_gnss", enable_gnss_, true);
    nh.param<bool>("enable_wheel", enable_wheel_, true);
    nh.param<bool>("enable_loop_distance", enable_loop_distance_, false);

    ROS_INFO("[Backend] Parameters loaded:");
    ROS_INFO_STREAM("  livo_odom_topic     : " << livo_topic);
    ROS_INFO_STREAM("  gnss_topic          : " << gnss_topic << (enable_gnss_ ? " (enabled)" : " (disabled)"));
    ROS_INFO_STREAM("  wheel_topic         : " << wheel_topic << (enable_wheel_ ? " (enabled)" : " (disabled)"));
    ROS_INFO_STREAM("  loop_distance       : " << (enable_loop_distance_ ? "enabled" : "disabled"));
    ROS_INFO_STREAM("  keyframe_skip       : " << keyframe_skip_);
    ROS_INFO_STREAM("  keyframe_dist_thresh: " << keyframe_distance_threshold_);
    ROS_INFO_STREAM("  keyframe_angle_thresh: " << keyframe_angle_threshold_);
    ROS_INFO_STREAM("  livox_pos_cov       : " << livox_pos_cov_);
    ROS_INFO_STREAM("  livox_rot_cov       : " << livox_rot_cov_);
}

void LIOBackend::initializePublishers(ros::NodeHandle &nh) {
    std::string livo_topic, gnss_topic, wheel_topic;
    nh.param<std::string>("livo_odom_topic", livo_topic, std::string("/aft_mapped_to_init"));
    nh.param<std::string>("gnss_topic", gnss_topic, std::string("/gnss_data"));
    nh.param<std::string>("wheel_topic", wheel_topic, std::string("/wheel_odometry"));

    sub_livo_odom_ = nh_.subscribe<nav_msgs::Odometry>(livo_topic, 2000,
                                                       &LIOBackend::livoOdomCallback, this);
    ROS_INFO_STREAM("[Backend] Subscribed to LIVO odom: " << livo_topic);

    if (enable_gnss_) {
        sub_gnss_ = nh_.subscribe<sensor_msgs::NavSatFix>(gnss_topic, 2000,
                                                          &LIOBackend::gnssCallback, this);
        ROS_INFO_STREAM("[Backend] Subscribed to GNSS: " << gnss_topic);
    }

    if (enable_wheel_) {
        sub_wheel_ = nh_.subscribe<nav_msgs::Odometry>(wheel_topic, 2000,
                                                       &LIOBackend::wheelOdomCallback, this);
        ROS_INFO_STREAM("[Backend] Subscribed to Wheel: " << wheel_topic);
    }

    pub_optimized_path_ = nh.advertise<nav_msgs::Path>("/backend/optimized_path", 10);
    pub_optimized_odom_ = nh.advertise<nav_msgs::Odometry>("/backend/optimized_odom", 10);
    ROS_INFO("[Backend] Publishers: /backend/optimized_path, /backend/optimized_odom");
}

void LIOBackend::initializeISAM2() {
    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    parameters.factorization = ISAM2Params::CHOLESKY;
    isam2_ = std::shared_ptr<gtsam::ISAM2>(new gtsam::ISAM2(parameters));
}

void LIOBackend::initializeFactorModules(ros::NodeHandle &nh) {
    if (enable_gnss_) {
        gnss_module_.reset(new GNSSFactorModule());
        gnss_module_->loadParameters(nh, "gnss_module");
        factor_modules_.push_back(gnss_module_);
        ROS_INFO("[Backend] GNSS factor module loaded");
    }

    if (enable_wheel_) {
        wheel_module_.reset(new WheelFactorModule());
        wheel_module_->loadParameters(nh, "wheel_module");
        factor_modules_.push_back(wheel_module_);
        ROS_INFO("[Backend] Wheel odometry factor module loaded");
    }

    if (enable_loop_distance_) {
        loop_distance_module_.reset(new LoopDistanceModule());
        loop_distance_module_->loadParameters(nh, "loop_distance_module");
        factor_modules_.push_back(loop_distance_module_);
        ROS_INFO("[Backend] Distance-based loop closure module loaded");
    }
}

/* -----------------------------------------------------------------------
 * 回调函数 —— 接收传感器数据并递给相应模块
 * ----------------------------------------------------------------------- */

void LIOBackend::livoOdomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(mtx_);

    static size_t msg_count = 0;
    msg_count++;

    if (!first_livo_received_) {
        first_livo_received_ = true;
        ROS_INFO_STREAM("[Backend] First LIVO odometry received (t=" << std::fixed << std::setprecision(3) << msg->header.stamp.toSec() << ")");
    }

    Eigen::Quaterniond q(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z);
    if (q.norm() > 1e-6) {
        q.normalize();
        Eigen::Vector3d t(
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z);
        if (std::isfinite(t.x()) && std::isfinite(t.y()) && std::isfinite(t.z())) {
            RawPose rp;
            rp.timestamp = msg->header.stamp.toSec();
            rp.raw_pose = gtsam::Pose3(gtsam::Rot3(q.matrix()), gtsam::Point3(t.x(), t.y(), t.z()));
            raw_poses_.push_back(rp);
        }
    }

    if (msg_count % 100 == 0) {
        ROS_INFO_STREAM("[Backend] LIVO odom messages received: " << msg_count
                          << ", keyframes: " << keyframes_.size()
                          << ", raw_poses: " << raw_poses_.size());
    }

    static size_t skip_counter = 0;
    skip_counter++;

    if (skip_counter % keyframe_skip_ != 0) {
        return;
    }

    addKeyframe(msg);
}

void LIOBackend::gnssCallback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
    if (!gnss_module_)
        return;

    std::lock_guard<std::mutex> lock(mtx_);

    static size_t gnss_count = 0;
    gnss_count++;

    GNSSData data;
    data.timestamp = msg->header.stamp.toSec();
    data.latitude = msg->latitude;
    data.longitude = msg->longitude;
    data.altitude = msg->altitude;

    if (msg->position_covariance_type == 0) {
        data.cov_enu_x = -1;
        data.cov_enu_y = -1;
        data.cov_enu_z = -1;
    } else {
        data.cov_enu_x = msg->position_covariance[0];
        data.cov_enu_y = msg->position_covariance[4];
        data.cov_enu_z = msg->position_covariance[8];
    }

    gnss_module_->pushData(data);

    if (gnss_count == 1) {
        ROS_INFO_STREAM("[Backend] First GNSS data received (lat=" << std::fixed << std::setprecision(6)
                          << data.latitude << ", lon=" << data.longitude << ")");
    }
    if (gnss_count % 100 == 0) {
        ROS_INFO_STREAM("[Backend] GNSS messages received: " << gnss_count);
    }
}

void LIOBackend::wheelOdomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    if (!wheel_module_)
        return;
    std::lock_guard<std::mutex> lock(mtx_);

    static size_t wheel_count = 0;
    wheel_count++;

    WheelData data;
    data.timestamp = msg->header.stamp.toSec();
    data.position = Eigen::Vector3d(
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z);
    data.orientation = Eigen::Quaterniond(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z);
    if (data.orientation.norm() > 1e-6)
        data.orientation.normalize();
    data.valid = true;

    wheel_module_->pushData(data);

    if (wheel_count == 1) {
        ROS_INFO_STREAM("[Backend] First wheel odom received (pos="
                          << std::fixed << std::setprecision(2)
                          << data.position.x() << ", " << data.position.y() << ", " << data.position.z() << ")");
    }
    if (wheel_count % 100 == 0) {
        ROS_INFO_STREAM("[Backend] Wheel odom messages received: " << wheel_count);
    }
}

/* -----------------------------------------------------------------------
 * 核心：添加关键帧 + 构建因子图
 * ----------------------------------------------------------------------- */

void LIOBackend::addKeyframe(const nav_msgs::Odometry::ConstPtr &msg) {
    KeyFrame kf;
    kf.timestamp = msg->header.stamp.toSec();
    kf.id = keyframe_counter_;

    Eigen::Quaterniond q(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z);
    if (q.norm() < 1e-6) {
        ROS_WARN_STREAM("[Backend] Invalid quaternion at t=" << kf.timestamp);
        return;
    }
    q.normalize();
    Eigen::Vector3d t(
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z);
    if (!std::isfinite(t.x()) || !std::isfinite(t.y()) || !std::isfinite(t.z())) {
        ROS_WARN_STREAM("[Backend] NaN/Inf position at t=" << kf.timestamp);
        return;
    }
    kf.livo_pose = gtsam::Pose3(gtsam::Rot3(q.matrix()), gtsam::Point3(t.x(), t.y(), t.z()));

    /* 关键帧去重：与上一帧距离/角度太近则跳过 */
    if (!keyframes_.empty()) {
        const KeyFrame &last_kf = keyframes_.back();
        double dist = (t - Eigen::Vector3d(
                               last_kf.livo_pose.translation().x(),
                               last_kf.livo_pose.translation().y(),
                               last_kf.livo_pose.translation().z()))
                          .norm();

        Eigen::AngleAxisd aa(q * last_kf.livo_pose.rotation().matrix().transpose());
        double angle = aa.angle();

        if (dist < keyframe_distance_threshold_ && angle < keyframe_angle_threshold_) {
            ROS_DEBUG_STREAM("[Backend] KF skipped (dist=" << std::fixed << std::setprecision(3)
                            << dist << "m, angle=" << angle << "rad)");
            return;
        }
    }

    keyframes_.push_back(kf);
    size_t current_id = keyframes_.size() - 1;
    keyframe_counter_++;

    /* ---- 第一帧：添加先验因子 ---- */
    if (current_id == 0) {
        gtsam::Vector6 prior_noise;
        prior_noise << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6;
        auto prior_noise_model = noiseModel::Diagonal::Variances(prior_noise);

        initial_estimate_.insert(X(0), kf.livo_pose);
        graph_.add(gtsam::NonlinearFactor::shared_ptr(
            new gtsam::PriorFactor<gtsam::Pose3>(X(0), kf.livo_pose, prior_noise_model)));

        isam2_->update(graph_, initial_estimate_);
        isam2_initialized_ = true;
        graph_.resize(0);
        initial_estimate_.clear();

        keyframes_[0].optimized_pose = kf.livo_pose;
        ROS_INFO_STREAM("[Backend] KF 0 initialized at ("
                          << std::fixed << std::setprecision(3)
                          << t.x() << ", " << t.y() << ", " << t.z() << ")");
        return;
    }

    /* ---- 后续帧：添加 LIVO 里程计因子 ---- */
    const KeyFrame &prev_kf = keyframes_[current_id - 1];

    gtsam::Pose3 prev_optimized;
    if (current_estimate_.exists(X(current_id - 1)))
        prev_optimized = current_estimate_.at<gtsam::Pose3>(X(current_id - 1));
    else
        prev_optimized = prev_kf.livo_pose;

    gtsam::Pose3 delta_from_raw = prev_kf.livo_pose.between(kf.livo_pose);
    gtsam::Pose3 est_current = prev_optimized.compose(delta_from_raw);

    initial_estimate_.insert(X(current_id), est_current);

    gtsam::Vector6 odom_noise_vec;
    odom_noise_vec << livox_rot_cov_, livox_rot_cov_, livox_rot_cov_,
        livox_pos_cov_, livox_pos_cov_, livox_pos_cov_;
    auto odom_noise = noiseModel::Diagonal::Variances(odom_noise_vec);

    graph_.add(gtsam::NonlinearFactor::shared_ptr(
        new gtsam::BetweenFactor<gtsam::Pose3>(X(current_id - 1), X(current_id), delta_from_raw, odom_noise)));

    /* ---- 遍历所有因子模块，各自添加因子 ---- */
    for (auto &module : factor_modules_) {
        if (!module->isEnabled())
            continue;

        auto factors = module->evaluate(current_id, keyframes_, current_estimate_);
        for (auto &factor : factors) {
            graph_.add(factor);
        }

        if (!factors.empty()) {
            ROS_INFO_STREAM("[Backend] Module '" << module->name()
                              << "' added " << factors.size() << " factor(s) at KF " << current_id);
        }
    }

    /* ---- iSAM2 增量更新 ---- */
    try {
        isam2_->update(graph_, initial_estimate_);
        current_estimate_ = isam2_->calculateEstimate();
    } catch (const std::exception &e) {
        ROS_ERROR_STREAM("[Backend] iSAM2 update failed: " << e.what());
        graph_.resize(0);
        initial_estimate_.clear();
        return;
    }
    graph_.resize(0);
    initial_estimate_.clear();

    gtsam::Pose3 cur_est = current_estimate_.at<gtsam::Pose3>(X(current_id));
    if (!std::isfinite(cur_est.translation().x()) ||
        !std::isfinite(cur_est.translation().y()) ||
        !std::isfinite(cur_est.translation().z())) {
        ROS_ERROR_STREAM("[Backend] NaN/Inf in ISAM2 estimate at KF " << current_id
                                                                      << ", skipping update");
        return;
    }

    /* ---- 更新所有关键帧的优化后位姿 ---- */
    for (size_t i = 0; i < keyframes_.size(); ++i) {
        if (current_estimate_.exists(X(i))) {
            keyframes_[i].optimized_pose = current_estimate_.at<gtsam::Pose3>(X(i));
        }
    }

    /* ---- 发布 ---- */
    gtsam::Pose3 optimized_pose = current_estimate_.at<gtsam::Pose3>(X(current_id));
    publishOptimizedOdometry(optimized_pose, kf.timestamp);
    publishTF(optimized_pose, kf.timestamp);
    publishOptimizedPath();

    if (current_id % 10 == 0) {
        gtsam::Point3 opt_t = optimized_pose.translation();
        ROS_INFO_STREAM("[Backend] KF " << current_id << " | keyframes: " << keyframes_.size()
                          << " | optimized: (" << std::fixed << std::setprecision(2)
                          << opt_t.x() << ", " << opt_t.y() << ", " << opt_t.z() << ")");
    }
}

/* -----------------------------------------------------------------------
 * 发布 / 保存
 * ----------------------------------------------------------------------- */

void LIOBackend::publishOptimizedPath() {
    if (!isam2_initialized_)
        return;

    optimized_path_.header.stamp = ros::Time::now();
    optimized_path_.poses.clear();

    for (size_t i = 0; i < keyframes_.size(); ++i) {
        if (!current_estimate_.exists(X(i)))
            continue;

        gtsam::Pose3 pose = current_estimate_.at<gtsam::Pose3>(X(i));
        geometry_msgs::PoseStamped ps;
        ps.header.stamp = ros::Time(keyframes_[i].timestamp);
        ps.header.frame_id = "camera_init";

        ps.pose.position.x = pose.translation().x();
        ps.pose.position.y = pose.translation().y();
        ps.pose.position.z = pose.translation().z();

        Eigen::Quaterniond q(pose.rotation().matrix());
        ps.pose.orientation.w = q.w();
        ps.pose.orientation.x = q.x();
        ps.pose.orientation.y = q.y();
        ps.pose.orientation.z = q.z();

        optimized_path_.poses.push_back(ps);
    }

    pub_optimized_path_.publish(optimized_path_);
}

void LIOBackend::publishOptimizedOdometry(const gtsam::Pose3 &pose, double timestamp) {
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time(timestamp);
    odom.header.frame_id = "camera_init";
    odom.child_frame_id = "backend_optimized";

    odom.pose.pose.position.x = pose.translation().x();
    odom.pose.pose.position.y = pose.translation().y();
    odom.pose.pose.position.z = pose.translation().z();

    Eigen::Quaterniond q(pose.rotation().matrix());
    odom.pose.pose.orientation.w = q.w();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();

    pub_optimized_odom_.publish(odom);
}

void LIOBackend::publishTF(const gtsam::Pose3 &pose, double timestamp) {
    static tf::TransformBroadcaster br;

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(
        pose.translation().x(), pose.translation().y(), pose.translation().z()));

    Eigen::Quaterniond q(pose.rotation().matrix());
    tf::Quaternion tf_q(q.x(), q.y(), q.z(), q.w());
    transform.setRotation(tf_q);

    br.sendTransform(tf::StampedTransform(transform, ros::Time(timestamp), "camera_init", "backend_optimized"));
}

void LIOBackend::saveTrajectory(const std::string &filename) {
    if (filename.empty() || !isam2_initialized_)
        return;

    std::ofstream fout(filename.c_str());
    if (!fout.is_open()) {
        ROS_WARN_STREAM("[Backend] Cannot open file: " << filename);
        return;
    }

    fout << "# timestamp tx ty tz qx qy qz qw" << std::endl;
    for (size_t i = 0; i < keyframes_.size(); ++i) {
        if (!current_estimate_.exists(X(i)))
            continue;

        gtsam::Pose3 pose = current_estimate_.at<gtsam::Pose3>(X(i));
        Eigen::Quaterniond q(pose.rotation().matrix());

        fout << std::fixed << std::setprecision(9)
             << keyframes_[i].timestamp << " "
             << pose.translation().x() << " "
             << pose.translation().y() << " "
             << pose.translation().z() << " "
             << q.x() << " " << q.y() << " " << q.z() << " " << q.w()
             << std::endl;
    }

    fout.close();
    ROS_INFO_STREAM("[Backend] Trajectory saved: " << filename << " (" << keyframes_.size() << " keyframes)");
}

void LIOBackend::saveFullTrajectory(const std::string &filename) {
    if (filename.empty() || !isam2_initialized_ || keyframes_.empty())
        return;

    std::ofstream fout(filename.c_str());
    if (!fout.is_open()) {
        ROS_WARN_STREAM("[Backend] Cannot open file: " << filename);
        return;
    }

    fout << "# timestamp tx ty tz qx qy qz qw" << std::endl;

    size_t kf_idx = 0;

    for (size_t i = 0; i < raw_poses_.size(); ++i) {
        const auto &rp = raw_poses_[i];

        while (kf_idx + 1 < keyframes_.size() &&
               keyframes_[kf_idx + 1].timestamp <= rp.timestamp) {
            kf_idx++;
        }

        if (!current_estimate_.exists(X(kf_idx)))
            continue;

        gtsam::Pose3 kf_optimized = current_estimate_.at<gtsam::Pose3>(X(kf_idx));
        const KeyFrame &kf = keyframes_[kf_idx];

        gtsam::Pose3 delta = kf.livo_pose.between(rp.raw_pose);
        gtsam::Pose3 full_pose = kf_optimized.compose(delta);

        Eigen::Quaterniond q(full_pose.rotation().matrix());

        fout << std::fixed << std::setprecision(9)
             << rp.timestamp << " "
             << full_pose.translation().x() << " "
             << full_pose.translation().y() << " "
             << full_pose.translation().z() << " "
             << q.x() << " " << q.y() << " " << q.z() << " " << q.w()
             << std::endl;
    }

    fout.close();
    ROS_INFO_STREAM("[Backend] Full trajectory saved: " << filename << " (" << raw_poses_.size() << " frames)");
}

void LIOBackend::run() {
    ros::spin();
}

/************************************************************************/
/*  GNSS 因子模块                                                        */
/************************************************************************/

void GNSSFactorModule::loadParameters(ros::NodeHandle &nh, const std::string &ns) {
    nh.param<double>(ns + "/gnss_pos_cov", gnss_pos_cov_, 25.0);
    nh.param<double>(ns + "/time_tolerance", time_tolerance_, 0.5);
    nh.param<int>(ns + "/transform_update_interval", transform_update_interval_, 50);
    nh.param<int>(ns + "/min_correspondences", min_correspondences_, 20);
    origin_set_ = false;
    last_transform_kf_ = 0;
}

void GNSSFactorModule::setOrigin(double lat, double lon, double alt) {
    origin_lat_ = lat;
    origin_lon_ = lon;
    origin_alt_ = alt;
    origin_set_ = true;
    ROS_INFO_STREAM("[GNSSModule] Origin set: lat=" << lat << ", lon=" << lon << ", alt=" << alt);
}

void GNSSFactorModule::pushData(const GNSSData &data) {
    if (!origin_set_) {
        setOrigin(data.latitude, data.longitude, data.altitude);
    }

    GNSSData d = data;
    gnssWgs842Enu(d.latitude, d.longitude, d.altitude, d.enu_pos);
    d.enu_initialized = true;

    buffer_.push_back(d);
    if (buffer_.size() > 1000)
        buffer_.pop_front();
}

std::vector<gtsam::NonlinearFactor::shared_ptr> GNSSFactorModule::evaluate(
    size_t keyframe_id,
    const KeyFrameVector &keyframes,
    const gtsam::Values &current_estimate) {
    std::vector<gtsam::NonlinearFactor::shared_ptr> factors;

    if (!enabled_ || !origin_set_ || keyframe_id >= keyframes.size()) {
        return factors;
    }

    GNSSData closest;
    findClosest(keyframe_id, keyframes, closest);

    if (!closest.enu_initialized)
        return factors;

    Eigen::Vector3d enu_pt(closest.enu_pos.x(), closest.enu_pos.y(), closest.enu_pos.z());

    gnss_keyframe_matches_[keyframe_id] = enu_pt;

    if (!transform_estimated_) {
        Eigen::Vector3d livo_pt = getLivoPosition(keyframe_id, keyframes, current_estimate);
        collectCorrespondence(enu_pt, livo_pt);

        if (static_cast<int>(enu_points_.size()) >= min_correspondences_ && !transform_estimated_) {
            if (estimateTransform()) {
                ROS_INFO("[GNSSModule] ENU->LIVO transform estimated from %zu correspondences",
                         enu_points_.size());
                enu_points_.clear();
                livo_points_.clear();
            }
        }
        return factors;
    }

    maybeReestimateTransform(keyframe_id, keyframes, current_estimate);

    Eigen::Vector4d enu_h(enu_pt.x(), enu_pt.y(), enu_pt.z(), 1.0);
    Eigen::Vector4d livo_h = enu_to_livo_ * enu_h;
    gtsam::Point3 gnss_position(livo_h(0), livo_h(1), livo_h(2));

    gtsam::Vector3 gps_cov;
    if (closest.cov_enu_x > 0 && closest.cov_enu_y > 0 && closest.cov_enu_z > 0) {
        gps_cov << closest.cov_enu_x, closest.cov_enu_y, closest.cov_enu_z * 2.0;
    } else {
        gps_cov << gnss_pos_cov_, gnss_pos_cov_, gnss_pos_cov_ * 2.0;
    }
    auto gps_noise = noiseModel::Diagonal::Variances(gps_cov);

    factors.push_back(gtsam::make_shared<gtsam::GPSFactor>(
        X(keyframe_id), gnss_position, gps_noise));

    return factors;
}

Eigen::Vector3d GNSSFactorModule::getLivoPosition(
    size_t keyframe_id,
    const KeyFrameVector &keyframes,
    const gtsam::Values &current_estimate) const {
    if (current_estimate.exists(X(keyframe_id))) {
        gtsam::Pose3 opt_pose = current_estimate.at<gtsam::Pose3>(X(keyframe_id));
        return Eigen::Vector3d(opt_pose.translation().x(),
                               opt_pose.translation().y(),
                               opt_pose.translation().z());
    }
    gtsam::Pose3 raw_pose = keyframes[keyframe_id].livo_pose;
    return Eigen::Vector3d(raw_pose.translation().x(),
                           raw_pose.translation().y(),
                           raw_pose.translation().z());
}

void GNSSFactorModule::maybeReestimateTransform(
    size_t keyframe_id,
    const KeyFrameVector &keyframes,
    const gtsam::Values &current_estimate) {
    if (static_cast<int>(keyframe_id - last_transform_kf_) < transform_update_interval_)
        return;

    if (gnss_keyframe_matches_.size() < static_cast<size_t>(min_correspondences_))
        return;

    Eigen::Matrix3Xd enu_pts(3, gnss_keyframe_matches_.size());
    Eigen::Matrix3Xd livo_pts(3, gnss_keyframe_matches_.size());
    int valid_count = 0;

    for (const auto &kv : gnss_keyframe_matches_) {
        if (current_estimate.exists(X(kv.first))) {
            Eigen::Vector3d livo_pt = getLivoPosition(kv.first, keyframes, current_estimate);
            enu_pts.col(valid_count) = kv.second;
            livo_pts.col(valid_count) = livo_pt;
            valid_count++;
        }
    }

    if (valid_count < min_correspondences_)
        return;

    enu_pts.conservativeResize(3, valid_count);
    livo_pts.conservativeResize(3, valid_count);

    Eigen::Vector3d enu_mean = enu_pts.rowwise().mean();
    Eigen::Vector3d livo_mean = livo_pts.rowwise().mean();

    Eigen::Matrix3Xd enu_c = enu_pts.colwise() - enu_mean;
    Eigen::Matrix3Xd livo_c = livo_pts.colwise() - livo_mean;

    Eigen::Matrix3d H = livo_c * enu_c.transpose();
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d R = svd.matrixU() * svd.matrixV().transpose();

    if (R.determinant() < 0) {
        Eigen::Matrix3d V = svd.matrixV();
        V.col(2) *= -1;
        R = svd.matrixU() * V.transpose();
    }

    Eigen::Vector3d t = livo_mean - R * enu_mean;

    enu_to_livo_ = Eigen::Matrix4d::Identity();
    enu_to_livo_.block<3, 3>(0, 0) = R;
    enu_to_livo_.block<3, 1>(0, 3) = t;
    last_transform_kf_ = keyframe_id;

    double residuals = 0;
    for (int i = 0; i < valid_count; i++) {
        Eigen::Vector3d transformed = R * enu_pts.col(i) + t;
        residuals += (transformed - livo_pts.col(i)).squaredNorm();
    }
    ROS_INFO_STREAM("[GNSSModule] Transform re-estimated at KF " << keyframe_id
                      << " with " << valid_count << " correspondences"
                      << ", RMSE: " << std::sqrt(residuals / valid_count) << " m");
}

void GNSSFactorModule::collectCorrespondence(const Eigen::Vector3d &enu_pos, const Eigen::Vector3d &livo_pos) {
    enu_points_.push_back(enu_pos);
    livo_points_.push_back(livo_pos);
}

bool GNSSFactorModule::estimateTransform() {
    int n = static_cast<int>(enu_points_.size());
    if (n < min_correspondences_)
        return false;

    Eigen::Map<Eigen::Matrix3Xd> enu_mat(enu_points_[0].data(), 3, n);
    Eigen::Matrix3Xd enu_pts(3, n);
    Eigen::Matrix3Xd livo_pts(3, n);
    for (int i = 0; i < n; i++) {
        enu_pts.col(i) = enu_points_[i];
        livo_pts.col(i) = livo_points_[i];
    }

    Eigen::Vector3d enu_mean = enu_pts.rowwise().mean();
    Eigen::Vector3d livo_mean = livo_pts.rowwise().mean();

    Eigen::Matrix3Xd enu_c = enu_pts.colwise() - enu_mean;
    Eigen::Matrix3Xd livo_c = livo_pts.colwise() - livo_mean;

    Eigen::Matrix3d H = livo_c * enu_c.transpose();
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d R = svd.matrixU() * svd.matrixV().transpose();

    if (R.determinant() < 0) {
        Eigen::Matrix3d V = svd.matrixV();
        V.col(2) *= -1;
        R = svd.matrixU() * V.transpose();
    }

    Eigen::Vector3d t = livo_mean - R * enu_mean;

    enu_to_livo_ = Eigen::Matrix4d::Identity();
    enu_to_livo_.block<3, 3>(0, 0) = R;
    enu_to_livo_.block<3, 1>(0, 3) = t;
    transform_estimated_ = true;

    ROS_INFO_STREAM("[GNSSModule] Transform ENU->LIVO:");
    ROS_INFO_STREAM("  R: [" << R.row(0) << "; " << R.row(1) << "; " << R.row(2) << "]");
    ROS_INFO_STREAM("  t: [" << t.transpose() << "]");

    double residuals = 0;
    for (int i = 0; i < n; i++) {
        Eigen::Vector3d transformed = R * enu_points_[i] + t;
        residuals += (transformed - livo_points_[i]).squaredNorm();
    }
    ROS_INFO_STREAM("[GNSSModule] Transform fit RMSE: " << std::sqrt(residuals / n) << " m");

    return true;
}

void GNSSFactorModule::findClosest(size_t keyframe_id, const KeyFrameVector &keyframes, GNSSData &closest) const {
    closest.enu_initialized = false;
    if (keyframe_id >= keyframes.size() || buffer_.empty())
        return;

    double kf_time = keyframes[keyframe_id].timestamp;
    double min_dt = std::numeric_limits<double>::max();

    for (const auto &gnss : buffer_) {
        double dt = fabs(gnss.timestamp - kf_time);
        if (dt < min_dt && dt < time_tolerance_) {
            min_dt = dt;
            closest = gnss;
        }
    }

    if (min_dt < time_tolerance_) {
        closest.enu_initialized = true;
    }
}

void GNSSFactorModule::gnssWgs842Enu(double lat, double lon, double alt, Eigen::Vector3d &enu) const {
    double a = 6378137.0;
    double f = 1.0 / 298.257223563;
    double e2 = 2.0 * f - f * f;

    double deg2rad = M_PI / 180.0;

    double lat_rad = lat * deg2rad;
    double lon_rad = lon * deg2rad;
    double lat0_rad = origin_lat_ * deg2rad;
    double lon0_rad = origin_lon_ * deg2rad;

    double N = a / sqrt(1.0 - e2 * sin(lat_rad) * sin(lat_rad));
    double x = (N + alt) * cos(lat_rad) * cos(lon_rad);
    double y = (N + alt) * cos(lat_rad) * sin(lon_rad);
    double z = (N * (1.0 - e2) + alt) * sin(lat_rad);

    double N0 = a / sqrt(1.0 - e2 * sin(lat0_rad) * sin(lat0_rad));
    double x0 = (N0 + origin_alt_) * cos(lat0_rad) * cos(lon0_rad);
    double y0 = (N0 + origin_alt_) * cos(lat0_rad) * sin(lon0_rad);
    double z0 = (N0 * (1.0 - e2) + origin_alt_) * sin(lat0_rad);

    double dx = x - x0;
    double dy = y - y0;
    double dz = z - z0;

    double e = -sin(lon0_rad) * dx + cos(lon0_rad) * dy;
    double n = -sin(lat0_rad) * cos(lon0_rad) * dx - sin(lat0_rad) * sin(lon0_rad) * dy + cos(lat0_rad) * dz;
    double u = cos(lat0_rad) * cos(lon0_rad) * dx + cos(lat0_rad) * sin(lon0_rad) * dy + sin(lat0_rad) * dz;

    enu(0) = e;
    enu(1) = n;
    enu(2) = u;
}

/************************************************************************/
/*  轮速计因子模块                                                        */
/************************************************************************/

void WheelFactorModule::loadParameters(ros::NodeHandle &nh, const std::string &ns) {
    nh.param<double>(ns + "/wheel_pos_cov", wheel_pos_cov_, 0.5);
    nh.param<double>(ns + "/wheel_rot_cov", wheel_rot_cov_, 0.1);
    nh.param<double>(ns + "/time_tolerance", time_tolerance_, 0.2);
}

void WheelFactorModule::pushData(const WheelData &data) {
    buffer_.push_back(data);
    if (buffer_.size() > 2000)
        buffer_.pop_front();
}

std::vector<gtsam::NonlinearFactor::shared_ptr> WheelFactorModule::evaluate(
    size_t keyframe_id,
    const KeyFrameVector &keyframes,
    const gtsam::Values &current_estimate) {
    std::vector<gtsam::NonlinearFactor::shared_ptr> factors;

    if (!enabled_ || keyframe_id < 1 || keyframe_id >= keyframes.size()) {
        return factors;
    }

    WheelData wd_curr, wd_prev;
    findClosest(keyframe_id, keyframes, wd_curr);
    findClosest(keyframe_id - 1, keyframes, wd_prev);

    if (!wd_curr.valid || !wd_prev.valid)
        return factors;

    Eigen::Isometry3d T_prev = Eigen::Isometry3d::Identity();
    T_prev.linear() = wd_prev.orientation.toRotationMatrix();
    T_prev.translation() = wd_prev.position;

    Eigen::Isometry3d T_curr = Eigen::Isometry3d::Identity();
    T_curr.linear() = wd_curr.orientation.toRotationMatrix();
    T_curr.translation() = wd_curr.position;

    Eigen::Isometry3d T_delta = T_prev.inverse() * T_curr;

    double delta_trans = T_delta.translation().norm();
    if (delta_trans < 0.001)
        return factors;

    gtsam::Pose3 wheel_delta(
        gtsam::Rot3(T_delta.rotation()),
        gtsam::Point3(T_delta.translation()));

    gtsam::Vector6 noise_vec;
    noise_vec << wheel_rot_cov_, wheel_rot_cov_, wheel_rot_cov_ * 10.0,
        wheel_pos_cov_, wheel_pos_cov_, wheel_pos_cov_ * 10.0;
    auto noise = noiseModel::Diagonal::Variances(noise_vec);

    factors.push_back(gtsam::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
        X(keyframe_id - 1), X(keyframe_id), wheel_delta, noise));

    return factors;
}

void WheelFactorModule::findClosest(size_t keyframe_id, const KeyFrameVector &keyframes, WheelData &closest) const {
    closest.valid = false;
    if (keyframe_id >= keyframes.size() || buffer_.empty())
        return;

    double kf_time = keyframes[keyframe_id].timestamp;
    double min_dt = std::numeric_limits<double>::max();

    for (const auto &w : buffer_) {
        double dt = fabs(w.timestamp - kf_time);
        if (dt < min_dt && dt < time_tolerance_) {
            min_dt = dt;
            closest = w;
        }
    }

    if (min_dt < time_tolerance_)
        closest.valid = true;
}

/************************************************************************/
/*  距离回环因子模块 (方案一)                                              */
/************************************************************************/

void LoopDistanceModule::loadParameters(ros::NodeHandle &nh, const std::string &ns) {
    nh.param<double>(ns + "/search_radius", search_radius_, 5.0);
    nh.param<double>(ns + "/loop_pos_cov", loop_pos_cov_, 0.5);
    nh.param<double>(ns + "/loop_rot_cov", loop_rot_cov_, 0.1);
    nh.param<int>(ns + "/min_keyframe_gap", min_keyframe_gap_, 50);
    nh.param<int>(ns + "/max_keyframe_gap", max_keyframe_gap_, 500);
}

std::vector<gtsam::NonlinearFactor::shared_ptr> LoopDistanceModule::evaluate(
    size_t keyframe_id,
    const KeyFrameVector &keyframes,
    const gtsam::Values &current_estimate) {
    std::vector<gtsam::NonlinearFactor::shared_ptr> factors;

    if (!enabled_ || keyframe_id < (size_t)min_keyframe_gap_ ||
        keyframe_id >= keyframes.size()) {
        return factors;
    }

    if (!current_estimate.exists(X(keyframe_id))) {
        return factors;
    }

    gtsam::Pose3 cur_pose = current_estimate.at<gtsam::Pose3>(X(keyframe_id));
    gtsam::Point3 cur_pos = cur_pose.translation();

    size_t search_start = 0;
    size_t search_end = keyframe_id - min_keyframe_gap_;

    for (size_t i = search_start; i < search_end; ++i) {
        if (loop_history_.count(i))
            continue;

        if (!current_estimate.exists(X(i)))
            continue;

        gtsam::Pose3 cand_pose = current_estimate.at<gtsam::Pose3>(X(i));
        double dist = (cur_pos - cand_pose.translation()).norm();

        if (dist < search_radius_) {
            gtsam::Pose3 relative = cand_pose.between(cur_pose);

            gtsam::Vector6 loop_noise_vec;
            loop_noise_vec << loop_rot_cov_, loop_rot_cov_, loop_rot_cov_,
                loop_pos_cov_, loop_pos_cov_, loop_pos_cov_;
            auto loop_noise = noiseModel::Diagonal::Variances(loop_noise_vec);

            factors.push_back(gtsam::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
                X(i), X(keyframe_id), relative, loop_noise));

            loop_history_[i] = keyframe_id;

            ROS_INFO_STREAM("[LoopDistance] Loop closed: KF " << i
                                                              << " <-> KF " << keyframe_id << " (dist=" << dist << "m)");

            break;
        }
    }

    return factors;
}

/************************************************************************/
/*  main                                                                */
/************************************************************************/

/* -----------------------------------------------------------------------
 *  工具函数：自动创建目录 & 带时间戳子文件夹
 * ----------------------------------------------------------------------- */

bool LIOBackend::createDirectory(const std::string &path) {
    struct stat st;
    if (stat(path.c_str(), &st) == 0) {
        if (S_ISDIR(st.st_mode))
            return true;
        ROS_WARN_STREAM("[Backend] " << path << " exists but is not a directory");
        return false;
    }

    std::string cmd = "mkdir -p " + path;
    int ret = system(cmd.c_str());
    if (ret != 0) {
        ROS_WARN_STREAM("[Backend] Failed to create directory: " << path);
        return false;
    }
    return true;
}

std::string LIOBackend::createTimestampedDir(const std::string &base_dir) {
    std::string dir = base_dir;
    if (dir.empty())
        dir = std::string(ROOT_DIR) + "Log";

    createDirectory(dir);

    std::time_t now = std::time(nullptr);
    std::tm *t = std::localtime(&now);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", t);
    std::string timestamp(buf);

    std::string run_dir = dir + "/" + timestamp;
    createDirectory(run_dir);

    return run_dir;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "livo_backend");
    ros::NodeHandle nh("~");

    LIOBackend backend(nh);
    backend.run();

    return 0;
}