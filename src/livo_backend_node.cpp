#include "livo_backend/livo_backend.h"

#include <pcl/registration/gicp.h>
#include <pcl/common/point_tests.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/image_encodings.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <sstream>

namespace {

gtsam::Pose3 poseFromMatrix4f(const Eigen::Matrix4f &transform) {
    Eigen::Matrix3d rotation = transform.block<3, 3>(0, 0).cast<double>();
    Eigen::Vector3d translation = transform.block<3, 1>(0, 3).cast<double>();
    return gtsam::Pose3(gtsam::Rot3(rotation),
                        gtsam::Point3(translation.x(), translation.y(), translation.z()));
}

Eigen::Matrix4f poseToMatrix4f(const gtsam::Pose3 &pose) {
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3, 3>(0, 0) = pose.rotation().matrix().cast<float>();
    transform(0, 3) = static_cast<float>(pose.translation().x());
    transform(1, 3) = static_cast<float>(pose.translation().y());
    transform(2, 3) = static_cast<float>(pose.translation().z());
    return transform;
}

} // namespace

/************************************************************************/
/*  LIOBackend 实现                                                      */
/************************************************************************/

LIOBackend::LIOBackend(ros::NodeHandle &nh)
    : nh_(nh), keyframe_counter_(0), isam2_initialized_(false), first_livo_received_(false) {
    readParameters(nh);
    initializeFactorModules(nh);
    initializePublishers(nh);
    initializeISAM2();

    run_output_dir_ = createTimestampedDir(std::string(ROOT_DIR) + "Log");
    initializeDebugLog();
    if (loop_closure_module_) {
        loop_closure_module_->setDebugLogger([this](const std::string &message) {
            logDebugEvent(std::string("[loop_closure] ") + message);
        });
    }
    ROS_INFO_STREAM("[Backend] Run output directory: " << run_output_dir_);
    // #region debug-point backend-constructor
    logDebugEvent(std::string("constructor: run_output_dir=") + run_output_dir_);
    // #endregion

    optimized_path_.header.frame_id = "camera_init";

    ROS_INFO("[Backend] Initialization complete. Waiting for data...");
    // #region debug-point backend-init-complete
    logDebugEvent("initialization complete, waiting for data");
    // #endregion
}

LIOBackend::~LIOBackend() {
    ROS_INFO_STREAM("[Backend] Shutting down. Total keyframes: " << keyframes_.size()
                      << ", raw poses: " << raw_poses_.size());
    // #region debug-point backend-destructor
    {
        std::ostringstream oss;
        oss << "destructor: keyframes=" << keyframes_.size()
            << ", raw_poses=" << raw_poses_.size()
            << ", isam2_initialized=" << (isam2_initialized_ ? "true" : "false");
        logDebugEvent(oss.str());
    }
    // #endregion
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
    nh.param<std::string>("loop_closure/image_topic", loop_image_topic_, std::string("/rgb_img"));
    nh.param<std::string>("loop_closure/cloud_topic", loop_cloud_topic_, std::string("/cloud_registered"));

    nh.param<int>("keyframe_skip", keyframe_skip_, 5);
    nh.param<double>("keyframe_distance_threshold", keyframe_distance_threshold_, 0.5);
    nh.param<double>("keyframe_angle_threshold", keyframe_angle_threshold_, 0.3);

    nh.param<double>("noise/livox_pos_cov", livox_pos_cov_, 0.1);
    nh.param<double>("noise/livox_rot_cov", livox_rot_cov_, 0.05);

    nh.param<bool>("enable_gnss", enable_gnss_, true);
    nh.param<bool>("enable_wheel", enable_wheel_, true);
    nh.param<bool>("enable_loop_distance", enable_loop_distance_, false);
    nh.param<bool>("enable_loop_closure", enable_loop_closure_, false);
    nh.param<std::string>("loop_closure/mode", loop_mode_, std::string("hybrid_a"));

    ROS_INFO("[Backend] Parameters loaded:");
    ROS_INFO_STREAM("  livo_odom_topic     : " << livo_topic);
    ROS_INFO_STREAM("  gnss_topic          : " << gnss_topic << (enable_gnss_ ? " (enabled)" : " (disabled)"));
    ROS_INFO_STREAM("  wheel_topic         : " << wheel_topic << (enable_wheel_ ? " (enabled)" : " (disabled)"));
    ROS_INFO_STREAM("  loop_distance       : " << (enable_loop_distance_ ? "enabled" : "disabled"));
    ROS_INFO_STREAM("  loop_closure        : " << (enable_loop_closure_ ? "enabled" : "disabled")
                                                 << " (mode=" << loop_mode_ << ")");
    if (enable_loop_closure_) {
        ROS_INFO_STREAM("  loop_image_topic    : " << loop_image_topic_);
        ROS_INFO_STREAM("  loop_cloud_topic    : " << loop_cloud_topic_);
    }
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

    if (enable_loop_closure_) {
        sub_loop_image_ = nh_.subscribe<sensor_msgs::Image>(loop_image_topic_, 200,
                                                            &LIOBackend::loopImageCallback, this);
        sub_loop_cloud_ = nh_.subscribe<sensor_msgs::PointCloud2>(loop_cloud_topic_, 50,
                                                                  &LIOBackend::loopCloudCallback, this);
        ROS_INFO_STREAM("[Backend] Subscribed to loop image: " << loop_image_topic_);
        ROS_INFO_STREAM("[Backend] Subscribed to loop cloud: " << loop_cloud_topic_);
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

    if (enable_loop_closure_) {
        loop_closure_module_.reset(new LoopClosureModule());
        loop_closure_module_->loadParameters(nh, "loop_closure");
        factor_modules_.push_back(loop_closure_module_);
        ROS_INFO_STREAM("[Backend] Unified loop closure module loaded (mode=" << loop_mode_ << ")");
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
        // #region debug-point backend-first-livo
        {
            std::ostringstream oss;
            oss << "first_livo_odom: t=" << std::fixed << std::setprecision(6) << msg->header.stamp.toSec();
            logDebugEvent(oss.str());
        }
        // #endregion
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
        // #region debug-point backend-first-wheel
        {
            std::ostringstream oss;
            oss << "first_wheel_odom: t=" << std::fixed << std::setprecision(6) << data.timestamp
                << ", pos=" << std::setprecision(3)
                << data.position.x() << "," << data.position.y() << "," << data.position.z();
            logDebugEvent(oss.str());
        }
        // #endregion
    }
    if (wheel_count % 100 == 0) {
        ROS_INFO_STREAM("[Backend] Wheel odom messages received: " << wheel_count);
    }
}

void LIOBackend::loopImageCallback(const sensor_msgs::Image::ConstPtr &msg) {
    if (!loop_closure_module_)
        return;

    std::lock_guard<std::mutex> lock(mtx_);
    // #region debug-point backend-first-loop-image
    static bool first_loop_image_logged = false;
    if (!first_loop_image_logged) {
        std::ostringstream oss;
        oss << "first_loop_image: t=" << std::fixed << std::setprecision(6) << msg->header.stamp.toSec();
        logDebugEvent(oss.str());
        first_loop_image_logged = true;
    }
    // #endregion
    loop_closure_module_->onImage(msg);
}

void LIOBackend::loopCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    if (!loop_closure_module_)
        return;

    std::lock_guard<std::mutex> lock(mtx_);
    // #region debug-point backend-first-loop-cloud
    static bool first_loop_cloud_logged = false;
    if (!first_loop_cloud_logged) {
        std::ostringstream oss;
        oss << "first_loop_cloud: t=" << std::fixed << std::setprecision(6) << msg->header.stamp.toSec();
        logDebugEvent(oss.str());
        first_loop_cloud_logged = true;
    }
    // #endregion
    loop_closure_module_->onCloud(msg);
}

/* -----------------------------------------------------------------------
 * 核心：添加关键帧 + 构建因子图
 * ----------------------------------------------------------------------- */

void LIOBackend::addKeyframe(const nav_msgs::Odometry::ConstPtr &msg) {
    KeyFrame kf;
    kf.timestamp = msg->header.stamp.toSec();
    kf.id = keyframe_counter_;
    // #region debug-point backend-add-keyframe-start
    {
        std::ostringstream oss;
        oss << "addKeyframe start: pending_id=" << kf.id
            << ", timestamp=" << std::fixed << std::setprecision(6) << kf.timestamp;
        logDebugEvent(oss.str());
    }
    // #endregion

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

    for (auto &module : factor_modules_) {
        if (!module->isEnabled())
            continue;
        // #region debug-point backend-prepare-module
        {
            std::ostringstream oss;
            oss << "prepareKeyframe begin: module=" << module->name()
                << ", pending_id=" << kf.id;
            logDebugEvent(oss.str());
        }
        // #endregion
        module->prepareKeyframe(kf);
        // #region debug-point backend-prepare-module-done
        {
            std::ostringstream oss;
            oss << "prepareKeyframe done: module=" << module->name()
                << ", pending_id=" << kf.id
                << ", has_loop_image=" << (kf.has_loop_image ? "true" : "false")
                << ", has_loop_cloud=" << (kf.has_loop_cloud ? "true" : "false");
            logDebugEvent(oss.str());
        }
        // #endregion
    }

    keyframes_.push_back(kf);
    size_t current_id = keyframes_.size() - 1;
    keyframe_counter_++;
    // #region debug-point backend-keyframe
    if (current_id == 0 || current_id % 20 == 0) {
        std::ostringstream oss;
        oss << "keyframe accepted: id=" << current_id
            << ", timestamp=" << std::fixed << std::setprecision(6) << kf.timestamp
            << ", total_keyframes=" << keyframes_.size();
        logDebugEvent(oss.str());
    }
    // #endregion

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
        // #region debug-point backend-evaluate-module
        {
            std::ostringstream oss;
            oss << "evaluate begin: module=" << module->name()
                << ", current_id=" << current_id;
            logDebugEvent(oss.str());
        }
        // #endregion

        auto factors = module->evaluate(current_id, keyframes_, current_estimate_);
        // #region debug-point backend-evaluate-module-done
        {
            std::ostringstream oss;
            oss << "evaluate done: module=" << module->name()
                << ", current_id=" << current_id
                << ", factors=" << factors.size();
            logDebugEvent(oss.str());
        }
        // #endregion
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
        logDebugEvent(std::string("isam2 update begin: current_id=") + std::to_string(current_id));
        isam2_->update(graph_, initial_estimate_);
        current_estimate_ = isam2_->calculateEstimate();
        logDebugEvent(std::string("isam2 update done: current_id=") + std::to_string(current_id));
    } catch (const std::exception &e) {
        ROS_ERROR_STREAM("[Backend] iSAM2 update failed: " << e.what());
        logDebugEvent(std::string("isam2 update exception: ") + e.what());
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
    logDebugEvent(std::string("publish begin: current_id=") + std::to_string(current_id));
    publishOptimizedOdometry(optimized_pose, kf.timestamp);
    publishTF(optimized_pose, kf.timestamp);
    publishOptimizedPath();
    logDebugEvent(std::string("publish done: current_id=") + std::to_string(current_id));

    if (!run_output_dir_.empty() && current_id > 0 && current_id % 20 == 0) {
        saveTrajectory(run_output_dir_ + "/backend_optimized_traj.txt");
        saveFullTrajectory(run_output_dir_ + "/backend_full_traj.txt");
        logDebugEvent("periodic trajectory snapshot saved");
    }

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
    // #region debug-point backend-save-traj-entry
    {
        std::ostringstream oss;
        oss << "saveTrajectory called: filename=" << filename
            << ", isam2_initialized=" << (isam2_initialized_ ? "true" : "false")
            << ", keyframes=" << keyframes_.size();
        logDebugEvent(oss.str());
    }
    // #endregion
    if (filename.empty() || !isam2_initialized_)
        return;

    std::ofstream fout(filename.c_str());
    if (!fout.is_open()) {
        ROS_WARN_STREAM("[Backend] Cannot open file: " << filename);
        logDebugEvent(std::string("saveTrajectory open failed: ") + filename);
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
    // #region debug-point backend-save-traj-done
    logDebugEvent(std::string("saveTrajectory finished: ") + filename);
    // #endregion
}

void LIOBackend::saveFullTrajectory(const std::string &filename) {
    // #region debug-point backend-save-full-traj-entry
    {
        std::ostringstream oss;
        oss << "saveFullTrajectory called: filename=" << filename
            << ", isam2_initialized=" << (isam2_initialized_ ? "true" : "false")
            << ", keyframes=" << keyframes_.size()
            << ", raw_poses=" << raw_poses_.size();
        logDebugEvent(oss.str());
    }
    // #endregion
    if (filename.empty() || !isam2_initialized_ || keyframes_.empty())
        return;

    std::ofstream fout(filename.c_str());
    if (!fout.is_open()) {
        ROS_WARN_STREAM("[Backend] Cannot open file: " << filename);
        logDebugEvent(std::string("saveFullTrajectory open failed: ") + filename);
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
    // #region debug-point backend-save-full-traj-done
    logDebugEvent(std::string("saveFullTrajectory finished: ") + filename);
    // #endregion
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
/*  统一回环模块                                                          */
/************************************************************************/

void LoopClosureModule::loadParameters(ros::NodeHandle &nh, const std::string &ns) {
    std::string mode = "hybrid_a";
    nh.param<std::string>(ns + "/mode", mode, mode);
    mode_ = parseMode(mode);
    enabled_ = (mode_ != LoopClosureMode::DISABLED);

    nh.param<int>(ns + "/image_buffer_size", image_buffer_size_, 30);
    nh.param<int>(ns + "/cloud_buffer_size", cloud_buffer_size_, 30);
    nh.param<double>(ns + "/image_time_tolerance", image_time_tolerance_, 0.15);
    nh.param<double>(ns + "/cloud_time_tolerance", cloud_time_tolerance_, 0.15);
    nh.param<double>(ns + "/image_resize_scale", image_resize_scale_, 0.5);
    nh.param<int>(ns + "/orb_features", orb_features_, 600);
    nh.param<int>(ns + "/detection_interval", detection_interval_, 5);
    nh.param<int>(ns + "/top_k_candidates", top_k_candidates_, 5);
    nh.param<int>(ns + "/min_keyframe_gap", min_keyframe_gap_, 50);
    nh.param<int>(ns + "/max_keyframe_gap", max_keyframe_gap_, 500);
    nh.param<int>(ns + "/min_orb_matches", min_orb_matches_, 30);
    nh.param<int>(ns + "/min_orb_inliers", min_orb_inliers_, 20);
    nh.param<double>(ns + "/min_visual_score", min_visual_score_, 0.08);
    nh.param<bool>(ns + "/require_both_modalities", require_both_modalities_, false);
    nh.param<double>(ns + "/min_fused_score", min_fused_score_, 0.75);
    nh.param<int>(ns + "/scan_context_rings", scan_context_rings_, 20);
    nh.param<int>(ns + "/scan_context_sectors", scan_context_sectors_, 60);
    nh.param<double>(ns + "/scan_context_max_radius", scan_context_max_radius_, 40.0);
    nh.param<double>(ns + "/scan_context_distance_threshold", scan_context_distance_threshold_, 0.35);
    nh.param<double>(ns + "/cloud_voxel_size", cloud_voxel_size_, 0.4);
    nh.param<double>(ns + "/icp_max_corr_distance", icp_max_corr_distance_, 2.0);
    nh.param<double>(ns + "/icp_fitness_threshold", icp_fitness_threshold_, 0.6);
    nh.param<int>(ns + "/min_icp_cloud_points", min_icp_cloud_points_, 30);
    nh.param<double>(ns + "/max_livo_loop_distance", max_livo_loop_distance_, 2.0);
    nh.param<double>(ns + "/max_gicp_translation_delta", max_gicp_translation_delta_, 1.0);
    nh.param<double>(ns + "/max_gicp_rotation_delta_deg", max_gicp_rotation_delta_deg_, 15.0);
    nh.param<double>(ns + "/loop_pos_cov", loop_pos_cov_, 0.3);
    nh.param<double>(ns + "/loop_rot_cov", loop_rot_cov_, 0.05);

    orb_detector_ = cv::ORB::create(std::max(orb_features_, 100));
    warn_unimplemented_mode_ = false;

    ROS_INFO_STREAM("[LoopClosure] mode=" << modeName()
                                          << ", detect_interval=" << detection_interval_
                                          << ", top_k=" << top_k_candidates_
                                          << ", require_both_modalities=" << (require_both_modalities_ ? "true" : "false")
                                          << ", min_fused_score=" << min_fused_score_
                                          << ", min_icp_cloud_points=" << min_icp_cloud_points_
                                          << ", max_livo_loop_distance=" << max_livo_loop_distance_
                                          << ", max_gicp_translation_delta=" << max_gicp_translation_delta_
                                          << ", max_gicp_rotation_delta_deg=" << max_gicp_rotation_delta_deg_);
}

LoopClosureMode LoopClosureModule::parseMode(const std::string &mode) const {
    if (mode == "hybrid_a" || mode == "A" || mode == "a")
        return LoopClosureMode::HYBRID_A;
    if (mode == "visual_b" || mode == "B" || mode == "b")
        return LoopClosureMode::VISUAL_B;
    if (mode == "frontend_c" || mode == "C" || mode == "c")
        return LoopClosureMode::FRONTEND_C;
    return LoopClosureMode::DISABLED;
}

std::string LoopClosureModule::modeName() const {
    switch (mode_) {
    case LoopClosureMode::HYBRID_A:
        return "hybrid_a";
    case LoopClosureMode::VISUAL_B:
        return "visual_b";
    case LoopClosureMode::FRONTEND_C:
        return "frontend_c";
    default:
        return "disabled";
    }
}

void LoopClosureModule::onImage(const sensor_msgs::Image::ConstPtr &msg) {
    if (!enabled_)
        return;

    BufferedImage data;
    data.timestamp = msg->header.stamp.toSec();

    try {
        cv_bridge::CvImageConstPtr cv_ptr =
            cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        if (image_resize_scale_ > 0.0 && std::abs(image_resize_scale_ - 1.0) > 1e-3) {
            cv::resize(cv_ptr->image, data.gray_image, cv::Size(), image_resize_scale_, image_resize_scale_);
        } else {
            data.gray_image = cv_ptr->image.clone();
        }
    } catch (const cv_bridge::Exception &e) {
        ROS_WARN_STREAM_THROTTLE(2.0, "[LoopClosure] Failed to convert image: " << e.what());
        return;
    }

    image_buffer_.push_back(data);
    while ((int)image_buffer_.size() > image_buffer_size_)
        image_buffer_.pop_front();
}

void LoopClosureModule::onCloud(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    if (!enabled_)
        return;

    BufferedCloud data;
    data.timestamp = msg->header.stamp.toSec();
    pcl::fromROSMsg(*msg, *data.cloud);
    if (data.cloud->empty())
        return;

    cloud_buffer_.push_back(data);
    while ((int)cloud_buffer_.size() > cloud_buffer_size_)
        cloud_buffer_.pop_front();
}

void LoopClosureModule::prepareKeyframe(KeyFrame &keyframe) {
    if (!enabled_)
        return;

    attachNearestImage(keyframe);
    attachNearestCloud(keyframe);
}

bool LoopClosureModule::attachNearestImage(KeyFrame &keyframe) {
    if (image_buffer_.empty())
        return false;

    auto best_it = image_buffer_.end();
    double min_dt = std::numeric_limits<double>::max();
    for (auto it = image_buffer_.begin(); it != image_buffer_.end(); ++it) {
        double dt = std::abs(it->timestamp - keyframe.timestamp);
        if (dt < min_dt && dt <= image_time_tolerance_) {
            min_dt = dt;
            best_it = it;
        }
    }

    if (best_it == image_buffer_.end())
        return false;

    keyframe.loop_image_gray = best_it->gray_image.clone();
    keyframe.has_loop_image = !keyframe.loop_image_gray.empty();
    extractImageFeatures(keyframe);
    return keyframe.has_loop_image;
}

bool LoopClosureModule::attachNearestCloud(KeyFrame &keyframe) {
    if (cloud_buffer_.empty())
        return false;

    auto best_it = cloud_buffer_.end();
    double min_dt = std::numeric_limits<double>::max();
    for (auto it = cloud_buffer_.begin(); it != cloud_buffer_.end(); ++it) {
        double dt = std::abs(it->timestamp - keyframe.timestamp);
        if (dt < min_dt && dt <= cloud_time_tolerance_) {
            min_dt = dt;
            best_it = it;
        }
    }

    if (best_it == cloud_buffer_.end() || !best_it->cloud || best_it->cloud->empty())
        return false;

    LoopCloud::Ptr local_cloud(new LoopCloud());
    Eigen::Matrix4d T_local_world = keyframe.livo_pose.inverse().matrix();
    local_cloud->reserve(best_it->cloud->size());

    for (const auto &pt : best_it->cloud->points) {
        if (!pcl::isFinite(pt))
            continue;
        Eigen::Vector4d pw(pt.x, pt.y, pt.z, 1.0);
        Eigen::Vector4d pl = T_local_world * pw;
        LoopPointType local_pt;
        local_pt.x = static_cast<float>(pl.x());
        local_pt.y = static_cast<float>(pl.y());
        local_pt.z = static_cast<float>(pl.z());
        local_pt.intensity = pt.intensity;
        local_cloud->push_back(local_pt);
    }

    pcl::VoxelGrid<LoopPointType> voxel;
    voxel.setLeafSize(cloud_voxel_size_, cloud_voxel_size_, cloud_voxel_size_);
    voxel.setInputCloud(local_cloud);
    keyframe.local_cloud.reset(new LoopCloud());
    voxel.filter(*keyframe.local_cloud);

    keyframe.has_loop_cloud = keyframe.local_cloud && keyframe.local_cloud->size() >= 50;
    extractCloudFeatures(keyframe);
    return keyframe.has_loop_cloud;
}

void LoopClosureModule::extractImageFeatures(KeyFrame &keyframe) {
    if (!orb_detector_ || keyframe.loop_image_gray.empty()) {
        keyframe.has_loop_image = false;
        return;
    }

    orb_detector_->detectAndCompute(
        keyframe.loop_image_gray, cv::noArray(), keyframe.loop_keypoints, keyframe.loop_descriptors);
    keyframe.has_loop_image = !keyframe.loop_descriptors.empty();
}

void LoopClosureModule::extractCloudFeatures(KeyFrame &keyframe) {
    if (!keyframe.local_cloud || keyframe.local_cloud->empty()) {
        keyframe.has_loop_cloud = false;
        return;
    }

    keyframe.scan_context = buildScanContext(keyframe.local_cloud);
    keyframe.has_loop_cloud = !keyframe.scan_context.empty();
}

cv::Mat LoopClosureModule::buildScanContext(const LoopCloud::Ptr &cloud) const {
    if (!cloud || cloud->empty())
        return cv::Mat();

    cv::Mat descriptor = cv::Mat::zeros(scan_context_rings_, scan_context_sectors_, CV_32F);
    const float sector_step = static_cast<float>(2.0 * M_PI / std::max(scan_context_sectors_, 1));

    for (const auto &pt : cloud->points) {
        const float radius = std::sqrt(pt.x * pt.x + pt.y * pt.y);
        if (radius < 1e-3f || radius > scan_context_max_radius_)
            continue;

        float theta = std::atan2(pt.y, pt.x);
        if (theta < 0.0f)
            theta += static_cast<float>(2.0 * M_PI);

        int ring = std::min(scan_context_rings_ - 1,
                            static_cast<int>((radius / scan_context_max_radius_) * scan_context_rings_));
        int sector = std::min(scan_context_sectors_ - 1,
                              static_cast<int>(theta / std::max(sector_step, 1e-6f)));
        float value = std::max(0.0f, pt.z + 2.0f);
        descriptor.at<float>(ring, sector) = std::max(descriptor.at<float>(ring, sector), value);
    }

    return descriptor;
}

double LoopClosureModule::computeScanContextDistance(const cv::Mat &lhs, const cv::Mat &rhs) const {
    if (lhs.empty() || rhs.empty() || lhs.size() != rhs.size())
        return std::numeric_limits<double>::infinity();

    double best_similarity = -1.0;
    for (int shift = 0; shift < scan_context_sectors_; ++shift) {
        double dot = 0.0;
        double lhs_norm = 0.0;
        double rhs_norm = 0.0;

        for (int r = 0; r < scan_context_rings_; ++r) {
            for (int c = 0; c < scan_context_sectors_; ++c) {
                float a = lhs.at<float>(r, c);
                float b = rhs.at<float>(r, (c + shift) % scan_context_sectors_);
                dot += static_cast<double>(a) * static_cast<double>(b);
                lhs_norm += static_cast<double>(a) * static_cast<double>(a);
                rhs_norm += static_cast<double>(b) * static_cast<double>(b);
            }
        }

        if (lhs_norm < 1e-9 || rhs_norm < 1e-9)
            continue;

        double similarity = dot / (std::sqrt(lhs_norm) * std::sqrt(rhs_norm));
        best_similarity = std::max(best_similarity, similarity);
    }

    if (best_similarity < 0.0)
        return std::numeric_limits<double>::infinity();
    return 1.0 - best_similarity;
}

LoopClosureModule::VisualMatchSummary LoopClosureModule::computeVisualMatch(
    const KeyFrame &current,
    const KeyFrame &candidate) const {
    VisualMatchSummary summary;
    if (current.loop_descriptors.empty() || candidate.loop_descriptors.empty())
        return summary;

    cv::BFMatcher matcher(cv::NORM_HAMMING, false);
    std::vector<std::vector<cv::DMatch>> knn_matches;
    matcher.knnMatch(current.loop_descriptors, candidate.loop_descriptors, knn_matches, 2);

    std::vector<cv::Point2f> cur_pts;
    std::vector<cv::Point2f> cand_pts;
    for (const auto &pair : knn_matches) {
        if (pair.size() < 2)
            continue;
        if (pair[0].distance >= 0.75f * pair[1].distance)
            continue;

        cur_pts.push_back(current.loop_keypoints[pair[0].queryIdx].pt);
        cand_pts.push_back(candidate.loop_keypoints[pair[0].trainIdx].pt);
    }

    summary.good_matches = static_cast<int>(cur_pts.size());
    if (summary.good_matches < min_orb_matches_)
        return summary;

    std::vector<uchar> inlier_mask;
    if (cur_pts.size() >= 8) {
        cv::findFundamentalMat(cur_pts, cand_pts, cv::FM_RANSAC, 3.0, 0.99, inlier_mask);
    }

    summary.inliers = 0;
    for (uchar is_inlier : inlier_mask)
        summary.inliers += static_cast<int>(is_inlier != 0);

    if (inlier_mask.empty())
        summary.inliers = summary.good_matches;

    int denom = std::max(1, std::min(current.loop_descriptors.rows, candidate.loop_descriptors.rows));
    summary.score = static_cast<double>(summary.inliers) / static_cast<double>(denom);
    summary.valid = (summary.good_matches >= min_orb_matches_ &&
                     summary.inliers >= min_orb_inliers_ &&
                     summary.score >= min_visual_score_);
    return summary;
}

std::vector<LoopClosureModule::CandidateScore> LoopClosureModule::generateHybridCandidates(
    size_t keyframe_id,
    const KeyFrameVector &keyframes) const {
    std::vector<CandidateScore> candidates;
    if (keyframe_id >= keyframes.size())
        return candidates;

    const KeyFrame &current = keyframes[keyframe_id];
    size_t search_begin = (keyframe_id > static_cast<size_t>(max_keyframe_gap_))
                              ? keyframe_id - static_cast<size_t>(max_keyframe_gap_)
                              : 0;
    size_t search_end = keyframe_id - static_cast<size_t>(min_keyframe_gap_);

    if (current.has_loop_cloud) {
        for (size_t i = search_begin; i < search_end; ++i) {
            if (loop_history_.count(i))
                continue;

            const KeyFrame &candidate = keyframes[i];
            if (!candidate.has_loop_cloud)
                continue;

            CandidateScore score;
            score.id = i;
            score.scan_distance = computeScanContextDistance(current.scan_context, candidate.scan_context);
            if (!std::isfinite(score.scan_distance) ||
                score.scan_distance > scan_context_distance_threshold_) {
                continue;
            }
            score.has_cloud_support = true;

            if (current.has_loop_image && candidate.has_loop_image) {
                VisualMatchSummary visual = computeVisualMatch(current, candidate);
                score.visual_score = visual.score;
                score.visual_good_matches = visual.good_matches;
                score.visual_inliers = visual.inliers;
                score.has_visual_support = visual.valid;
                if (require_both_modalities_ && !visual.valid)
                    continue;
            }

            score.fused_score = (1.0 - score.scan_distance) + score.visual_score;
            if (score.fused_score < min_fused_score_)
                continue;
            candidates.push_back(score);
        }

        std::sort(candidates.begin(), candidates.end(),
                  [](const CandidateScore &lhs, const CandidateScore &rhs) {
                      return lhs.fused_score > rhs.fused_score;
                  });
        if ((int)candidates.size() > top_k_candidates_)
            candidates.resize(top_k_candidates_);
        if (debug_logger_ && !candidates.empty()) {
            std::ostringstream oss;
            oss << "generateHybridCandidates: keyframe_id=" << keyframe_id
                << ", cloud_candidates=" << candidates.size();
            for (const auto &candidate : candidates) {
                oss << " [id=" << candidate.id
                    << ", scan=" << std::fixed << std::setprecision(6) << candidate.scan_distance
                    << ", visual=" << candidate.visual_score << "]";
            }
            debug_logger_(oss.str());
        }
        return candidates;
    }

    if (!current.has_loop_image)
        return candidates;

    for (size_t i = search_begin; i < search_end; ++i) {
        if (loop_history_.count(i))
            continue;

        const KeyFrame &candidate = keyframes[i];
        if (!candidate.has_loop_image)
            continue;

        VisualMatchSummary visual = computeVisualMatch(current, candidate);
        if (!visual.valid)
            continue;

        CandidateScore score;
        score.id = i;
        score.visual_score = visual.score;
        score.visual_good_matches = visual.good_matches;
        score.visual_inliers = visual.inliers;
        score.has_visual_support = true;
        score.fused_score = visual.score;
        candidates.push_back(score);
    }

    std::sort(candidates.begin(), candidates.end(),
              [](const CandidateScore &lhs, const CandidateScore &rhs) {
                  return lhs.fused_score > rhs.fused_score;
              });
    if ((int)candidates.size() > top_k_candidates_)
        candidates.resize(top_k_candidates_);
    if (debug_logger_ && !candidates.empty()) {
        std::ostringstream oss;
        oss << "generateHybridCandidates: keyframe_id=" << keyframe_id
            << ", image_candidates=" << candidates.size();
        for (const auto &candidate : candidates) {
            oss << " [id=" << candidate.id
                << ", visual=" << std::fixed << std::setprecision(6) << candidate.visual_score << "]";
        }
        debug_logger_(oss.str());
    }
    return candidates;
}

bool LoopClosureModule::estimateLoopConstraintGICP(
    const KeyFrame &current,
    const KeyFrame &candidate,
    gtsam::Pose3 &relative_pose,
    double &fitness_score) const {
    if (!current.local_cloud || !candidate.local_cloud ||
        current.local_cloud->empty() || candidate.local_cloud->empty()) {
        return false;
    }
    if (static_cast<int>(current.local_cloud->size()) < min_icp_cloud_points_ ||
        static_cast<int>(candidate.local_cloud->size()) < min_icp_cloud_points_) {
        if (debug_logger_) {
            std::ostringstream oss;
            oss << "gicp skipped: cloud too small"
                << ", current_cloud_size=" << current.local_cloud->size()
                << ", candidate_cloud_size=" << candidate.local_cloud->size()
                << ", min_required=" << min_icp_cloud_points_;
            debug_logger_(oss.str());
        }
        return false;
    }

    pcl::GeneralizedIterativeClosestPoint<LoopPointType, LoopPointType> gicp;
    gicp.setMaximumIterations(64);
    gicp.setMaxCorrespondenceDistance(icp_max_corr_distance_);
    gicp.setTransformationEpsilon(1e-4);
    gicp.setEuclideanFitnessEpsilon(1e-3);
    gicp.setInputSource(current.local_cloud);
    gicp.setInputTarget(candidate.local_cloud);

    LoopCloud aligned;
    Eigen::Matrix4f initial_guess = poseToMatrix4f(candidate.livo_pose.between(current.livo_pose));
    gicp.align(aligned, initial_guess);
    fitness_score = gicp.getFitnessScore();

    if (!gicp.hasConverged() || !std::isfinite(fitness_score) ||
        fitness_score > icp_fitness_threshold_) {
        return false;
    }

    Eigen::Matrix4f transform = gicp.getFinalTransformation();
    if (!transform.allFinite())
        return false;

    relative_pose = poseFromMatrix4f(transform);
    return true;
}

std::vector<gtsam::NonlinearFactor::shared_ptr> LoopClosureModule::evaluate(
    size_t keyframe_id,
    const KeyFrameVector &keyframes,
    const gtsam::Values &current_estimate) {
    std::vector<gtsam::NonlinearFactor::shared_ptr> factors;
    (void)current_estimate;

    if (!enabled_ || keyframe_id >= keyframes.size() ||
        keyframe_id < static_cast<size_t>(min_keyframe_gap_)) {
        return factors;
    }

    if (detection_interval_ > 1 &&
        (keyframe_id % static_cast<size_t>(detection_interval_)) != 0) {
        return factors;
    }

    if (mode_ == LoopClosureMode::DISABLED)
        return factors;

    if ((mode_ == LoopClosureMode::VISUAL_B || mode_ == LoopClosureMode::FRONTEND_C) &&
        !warn_unimplemented_mode_) {
        warn_unimplemented_mode_ = true;
        ROS_WARN_STREAM("[LoopClosure] Mode '" << modeName()
                        << "' is using the current hybrid_a pipeline as a placeholder."
                        << " You can later specialize it without changing backend wiring.");
    }

    const KeyFrame &current = keyframes[keyframe_id];
    std::vector<CandidateScore> candidates = generateHybridCandidates(keyframe_id, keyframes);
    if (debug_logger_) {
        std::ostringstream oss;
        oss << "evaluate: keyframe_id=" << keyframe_id
            << ", has_loop_image=" << (current.has_loop_image ? "true" : "false")
            << ", has_loop_cloud=" << (current.has_loop_cloud ? "true" : "false")
            << ", candidates=" << candidates.size();
        debug_logger_(oss.str());
    }
    if (candidates.empty())
        return factors;

    for (const auto &candidate_score : candidates) {
        const KeyFrame &candidate = keyframes[candidate_score.id];
        gtsam::Pose3 prior_relative_pose = candidate.livo_pose.between(current.livo_pose);
        const double prior_translation = prior_relative_pose.translation().norm();
        if (debug_logger_) {
            std::ostringstream oss;
            oss << "candidate begin: current_id=" << keyframe_id
                << ", candidate_id=" << candidate_score.id
                << ", cloud_support=" << (candidate_score.has_cloud_support ? "true" : "false")
                << ", visual_support=" << (candidate_score.has_visual_support ? "true" : "false")
                << ", scan_distance=" << std::fixed << std::setprecision(6) << candidate_score.scan_distance
                << ", visual_score=" << candidate_score.visual_score
                << ", fused_score=" << candidate_score.fused_score
                << ", visual_matches=" << candidate_score.visual_good_matches
                << ", visual_inliers=" << candidate_score.visual_inliers
                << ", prior_translation=" << prior_translation;
            debug_logger_(oss.str());
        }
        if (require_both_modalities_ &&
            !(candidate_score.has_cloud_support && candidate_score.has_visual_support)) {
            if (debug_logger_) {
                std::ostringstream oss;
                oss << "candidate rejected: current_id=" << keyframe_id
                    << ", candidate_id=" << candidate_score.id
                    << ", reason=require_both_modalities";
                debug_logger_(oss.str());
            }
            continue;
        }
        if (candidate_score.has_cloud_support && prior_translation > max_livo_loop_distance_) {
            if (debug_logger_) {
                std::ostringstream oss;
                oss << "candidate rejected: current_id=" << keyframe_id
                    << ", candidate_id=" << candidate_score.id
                    << ", reason=prior_translation_too_large"
                    << ", prior_translation=" << std::fixed << std::setprecision(6) << prior_translation
                    << ", limit=" << max_livo_loop_distance_;
                debug_logger_(oss.str());
            }
            continue;
        }
        if (debug_logger_) {
            std::ostringstream oss;
            oss << "gicp begin: current_id=" << keyframe_id
                << ", candidate_id=" << candidate_score.id
                << ", current_cloud_size=" << (current.local_cloud ? current.local_cloud->size() : 0)
                << ", candidate_cloud_size=" << (candidate.local_cloud ? candidate.local_cloud->size() : 0)
                << ", scan_distance=" << std::fixed << std::setprecision(6) << candidate_score.scan_distance;
            debug_logger_(oss.str());
        }

        gtsam::Pose3 relative_pose;
        double fitness_score = std::numeric_limits<double>::infinity();
        if (!estimateLoopConstraintGICP(current, candidate, relative_pose, fitness_score))
        {
            if (debug_logger_) {
                std::ostringstream oss;
                oss << "gicp rejected: current_id=" << keyframe_id
                    << ", candidate_id=" << candidate_score.id;
                debug_logger_(oss.str());
            }
            continue;
        }
        gtsam::Pose3 correction_pose = prior_relative_pose.between(relative_pose);
        const double correction_translation = correction_pose.translation().norm();
        Eigen::AngleAxisd correction_axis_angle(correction_pose.rotation().matrix());
        const double correction_rotation_deg = std::abs(correction_axis_angle.angle()) * 180.0 / M_PI;
        if (correction_translation > max_gicp_translation_delta_ ||
            correction_rotation_deg > max_gicp_rotation_delta_deg_) {
            if (debug_logger_) {
                std::ostringstream oss;
                oss << "candidate rejected: current_id=" << keyframe_id
                    << ", candidate_id=" << candidate_score.id
                    << ", reason=gicp_delta_too_large"
                    << ", correction_translation=" << std::fixed << std::setprecision(6) << correction_translation
                    << ", correction_rotation_deg=" << correction_rotation_deg
                    << ", translation_limit=" << max_gicp_translation_delta_
                    << ", rotation_limit=" << max_gicp_rotation_delta_deg_;
                debug_logger_(oss.str());
            }
            continue;
        }

        gtsam::Vector6 noise_vec;
        noise_vec << loop_rot_cov_, loop_rot_cov_, loop_rot_cov_,
            loop_pos_cov_, loop_pos_cov_, loop_pos_cov_;
        auto base_noise = noiseModel::Diagonal::Variances(noise_vec);
        auto robust_noise = noiseModel::Robust::Create(
            noiseModel::mEstimator::Huber::Create(1.345), base_noise);

        factors.push_back(gtsam::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
            X(candidate_score.id), X(keyframe_id), relative_pose, robust_noise));

        loop_history_[candidate_score.id] = keyframe_id;
        ROS_INFO_STREAM("[LoopClosure] Loop accepted: KF " << candidate_score.id
                          << " <-> KF " << keyframe_id
                          << " | mode=" << modeName()
                          << " | scan_dist=" << candidate_score.scan_distance
                          << " | visual_score=" << candidate_score.visual_score
                          << " | icp_fitness=" << fitness_score);
        if (debug_logger_) {
            std::ostringstream oss;
            oss << "gicp accepted: current_id=" << keyframe_id
                << ", candidate_id=" << candidate_score.id
                << ", fitness=" << std::fixed << std::setprecision(6) << fitness_score
                << ", correction_translation=" << correction_translation
                << ", correction_rotation_deg=" << correction_rotation_deg
                << ", visual_support=" << (candidate_score.has_visual_support ? "true" : "false")
                << ", cloud_support=" << (candidate_score.has_cloud_support ? "true" : "false");
            debug_logger_(oss.str());
        }
        break;
    }

    return factors;
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

void LIOBackend::initializeDebugLog() {
    if (run_output_dir_.empty()) {
        return;
    }
    debug_log_.open((run_output_dir_ + "/runtime_backend.log").c_str(), std::ios::out);
    if (!debug_log_.is_open()) {
        ROS_WARN_STREAM("[Backend] Failed to open debug log: " << run_output_dir_ << "/runtime_backend.log");
        return;
    }
    logDebugEvent(std::string("debug log initialized at ") + run_output_dir_ + "/runtime_backend.log");
}

void LIOBackend::logDebugEvent(const std::string &message) {
    if (!debug_log_.is_open()) {
        return;
    }
    std::time_t now = std::time(nullptr);
    std::tm *tm_now = std::localtime(&now);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", tm_now);
    debug_log_ << "[" << buf << "] " << message << std::endl;
    debug_log_.flush();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "livo_backend");
    ros::NodeHandle nh("~");

    LIOBackend backend(nh);
    backend.run();

    return 0;
}
