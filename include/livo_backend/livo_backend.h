#ifndef LIVO_BACKEND_H
#define LIVO_BACKEND_H

#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cmath>
#include <ctime>
#include <deque>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <mutex>
#include <sys/stat.h>
#include <vector>

using namespace gtsam;

using gtsam::symbol_shorthand::X;

using LoopPointType = pcl::PointXYZI;
using LoopCloud = pcl::PointCloud<LoopPointType>;

/* ========================================================================
 * 数据结构定义
 * ======================================================================== */

struct KeyFrame {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // 添加此宏以保证内存对齐
    KeyFrame()
        : timestamp(0.0), id(0), local_cloud(new LoopCloud()),
          has_loop_image(false), has_loop_cloud(false), loop_visual_score(0.0) {}

    double timestamp;
    gtsam::Pose3 livo_pose;
    gtsam::Pose3 optimized_pose;
    size_t id;
    cv::Mat loop_image_gray;
    std::vector<cv::KeyPoint> loop_keypoints;
    cv::Mat loop_descriptors;
    LoopCloud::Ptr local_cloud;
    cv::Mat scan_context;
    bool has_loop_image;
    bool has_loop_cloud;
    double loop_visual_score;
};
using KeyFrameVector = std::vector<KeyFrame, Eigen::aligned_allocator<KeyFrame>>;
struct GNSSData {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // 添加此宏以保证内存对齐
    double timestamp;
    double latitude;
    double longitude;
    double altitude;
    double cov_enu_x;
    double cov_enu_y;
    double cov_enu_z;
    Eigen::Vector3d enu_pos;
    bool enu_initialized;
};

struct WheelData {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double timestamp;
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    bool valid;
};

/* ========================================================================
 * 回环候选结构
 * ======================================================================== */

struct LoopCandidate {
    size_t current_id;
    size_t candidate_id;
    gtsam::Pose3 relative_pose;
    double distance;
    bool valid;
};

/* ========================================================================
 * 因子模块基类 —— 所有传感器因子 + 回环因子都继承自此接口
 * 每个模块负责：
 *   1. 判断是否需要在该关键帧添加因子
 *   2. 构造并返回因子列表
 *   3. 从配置参数初始化
 * ======================================================================== */

class FactorModule {
public:
    FactorModule() : enabled_(true), name_("unnamed") {}
    virtual ~FactorModule() {}

    virtual void loadParameters(ros::NodeHandle &nh, const std::string &ns) = 0;
    virtual std::vector<gtsam::NonlinearFactor::shared_ptr> evaluate(
        size_t keyframe_id,
        const KeyFrameVector &keyframes,
        const gtsam::Values &current_estimate) = 0;
    virtual void onImage(const sensor_msgs::Image::ConstPtr &msg) {}
    virtual void onCloud(const sensor_msgs::PointCloud2::ConstPtr &msg) {}
    virtual void prepareKeyframe(KeyFrame &keyframe) {}

    void setEnabled(bool en) { enabled_ = en; }
    bool isEnabled() const { return enabled_; }
    const std::string &name() const { return name_; }

protected:
    bool enabled_;
    std::string name_;
};

/* ========================================================================
 * GNSS 因子模块
 * ======================================================================== */

class GNSSFactorModule : public FactorModule {
public:
    GNSSFactorModule() { name_ = "gnss"; }

    void loadParameters(ros::NodeHandle &nh, const std::string &ns) override;
    std::vector<gtsam::NonlinearFactor::shared_ptr> evaluate(
        size_t keyframe_id,
        const KeyFrameVector &keyframes,
        const gtsam::Values &current_estimate) override;

    void pushData(const GNSSData &data);
    void setOrigin(double lat, double lon, double alt);
    void setTransform(const Eigen::Matrix4d &T) { enu_to_livo_ = T; transform_estimated_ = true; }
    bool isTransformEstimated() const { return transform_estimated_; }

    void collectCorrespondence(const Eigen::Vector3d &enu_pos, const Eigen::Vector3d &livo_pos);
    bool estimateTransform();
    void maybeReestimateTransform(size_t keyframe_id,
                                  const KeyFrameVector &keyframes,
                                  const gtsam::Values &current_estimate);
    Eigen::Vector3d getLivoPosition(size_t keyframe_id,
                                    const KeyFrameVector &keyframes,
                                    const gtsam::Values &current_estimate) const;

private:
    void findClosest(size_t keyframe_id, const KeyFrameVector &keyframes, GNSSData &closest) const;

    std::deque<GNSSData, Eigen::aligned_allocator<GNSSData>> buffer_;
    double gnss_pos_cov_;
    double time_tolerance_;
    bool origin_set_;
    double origin_lat_, origin_lon_, origin_alt_;

    void gnssWgs842Enu(double lat, double lon, double alt, Eigen::Vector3d &enu) const;

    Eigen::Matrix4d enu_to_livo_;
    bool transform_estimated_ = false;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> enu_points_;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> livo_points_;
    int min_correspondences_ = 20;
    std::map<size_t, Eigen::Vector3d> gnss_keyframe_matches_;
    int transform_update_interval_ = 50;
    size_t last_transform_kf_ = 0;
};

/* ========================================================================
 * 轮速计因子模块
 * ======================================================================== */

class WheelFactorModule : public FactorModule {
public:
    WheelFactorModule() { name_ = "wheel"; }

    void loadParameters(ros::NodeHandle &nh, const std::string &ns) override;
    std::vector<gtsam::NonlinearFactor::shared_ptr> evaluate(
        size_t keyframe_id,
        const KeyFrameVector &keyframes,
        const gtsam::Values &current_estimate) override;

    void pushData(const WheelData &data);

private:
    void findClosest(size_t keyframe_id, const KeyFrameVector &keyframes, WheelData &closest) const;

    std::deque<WheelData, Eigen::aligned_allocator<WheelData>> buffer_;
    double wheel_pos_cov_;
    double wheel_rot_cov_;
    double time_tolerance_;
};

/* ========================================================================
 * 距离回环因子模块 (方案一)
 * ======================================================================== */

class LoopDistanceModule : public FactorModule {
public:
    LoopDistanceModule() { name_ = "loop_distance"; }

    void loadParameters(ros::NodeHandle &nh, const std::string &ns) override;
    std::vector<gtsam::NonlinearFactor::shared_ptr> evaluate(
        size_t keyframe_id,
        const KeyFrameVector &keyframes,
        const gtsam::Values &current_estimate) override;

private:
    double search_radius_;
    double loop_pos_cov_;
    double loop_rot_cov_;
    int min_keyframe_gap_;
    int max_keyframe_gap_;

    std::map<size_t, size_t> loop_history_;
};

enum class LoopClosureMode {
    DISABLED = 0,
    HYBRID_A = 1,
    VISUAL_B = 2,
    FRONTEND_C = 3,
};

class LoopClosureModule : public FactorModule {
public:
    LoopClosureModule() { name_ = "loop_closure"; }

    void loadParameters(ros::NodeHandle &nh, const std::string &ns) override;
    std::vector<gtsam::NonlinearFactor::shared_ptr> evaluate(
        size_t keyframe_id,
        const KeyFrameVector &keyframes,
        const gtsam::Values &current_estimate) override;
    void onImage(const sensor_msgs::Image::ConstPtr &msg) override;
    void onCloud(const sensor_msgs::PointCloud2::ConstPtr &msg) override;
    void prepareKeyframe(KeyFrame &keyframe) override;
    void setDebugLogger(const std::function<void(const std::string &)> &logger) { debug_logger_ = logger; }

private:
    struct BufferedImage {
        double timestamp = 0.0;
        cv::Mat gray_image;
    };

    struct BufferedCloud {
        double timestamp = 0.0;
        LoopCloud::Ptr cloud;

        BufferedCloud() : cloud(new LoopCloud()) {}
    };

    struct CandidateScore {
        size_t id = 0;
        double scan_distance = std::numeric_limits<double>::infinity();
        double visual_score = 0.0;
        double fused_score = -std::numeric_limits<double>::infinity();
        int visual_good_matches = 0;
        int visual_inliers = 0;
        bool has_cloud_support = false;
        bool has_visual_support = false;
    };

    struct VisualMatchSummary {
        int good_matches = 0;
        int inliers = 0;
        double score = 0.0;
        bool valid = false;
    };

    LoopClosureMode parseMode(const std::string &mode) const;
    std::string modeName() const;
    bool attachNearestImage(KeyFrame &keyframe);
    bool attachNearestCloud(KeyFrame &keyframe);
    void extractImageFeatures(KeyFrame &keyframe);
    void extractCloudFeatures(KeyFrame &keyframe);
    cv::Mat buildScanContext(const LoopCloud::Ptr &cloud) const;
    double computeScanContextDistance(const cv::Mat &lhs, const cv::Mat &rhs) const;
    VisualMatchSummary computeVisualMatch(const KeyFrame &current, const KeyFrame &candidate) const;
    std::vector<CandidateScore> generateHybridCandidates(size_t keyframe_id, const KeyFrameVector &keyframes) const;
    bool estimateLoopConstraintGICP(
        const KeyFrame &current,
        const KeyFrame &candidate,
        gtsam::Pose3 &relative_pose,
        double &fitness_score) const;

    LoopClosureMode mode_ = LoopClosureMode::DISABLED;
    bool warn_unimplemented_mode_ = false;
    int image_buffer_size_ = 30;
    int cloud_buffer_size_ = 30;
    double image_time_tolerance_ = 0.15;
    double cloud_time_tolerance_ = 0.15;
    double image_resize_scale_ = 0.5;
    int orb_features_ = 600;
    int detection_interval_ = 5;
    int top_k_candidates_ = 5;
    int min_keyframe_gap_ = 50;
    int max_keyframe_gap_ = 500;
    int min_orb_matches_ = 30;
    int min_orb_inliers_ = 20;
    double min_visual_score_ = 0.08;
    bool require_both_modalities_ = false;
    double min_fused_score_ = 0.75;
    int scan_context_rings_ = 20;
    int scan_context_sectors_ = 60;
    double scan_context_max_radius_ = 40.0;
    double scan_context_distance_threshold_ = 0.35;
    double cloud_voxel_size_ = 0.4;
    double icp_max_corr_distance_ = 2.0;
    double icp_fitness_threshold_ = 0.6;
    int min_icp_cloud_points_ = 30;
    double max_livo_loop_distance_ = 2.0;
    double max_gicp_translation_delta_ = 1.0;
    double max_gicp_rotation_delta_deg_ = 15.0;
    double loop_pos_cov_ = 0.3;
    double loop_rot_cov_ = 0.05;
    cv::Ptr<cv::ORB> orb_detector_;
    std::deque<BufferedImage> image_buffer_;
    std::deque<BufferedCloud> cloud_buffer_;
    std::map<size_t, size_t> loop_history_;
    std::function<void(const std::string &)> debug_logger_;
};

/* ========================================================================
 * 主后端类
 * ======================================================================== */

class LIOBackend {
public:
    LIOBackend(ros::NodeHandle &nh);
    ~LIOBackend();

    void run();

private:
    void livoOdomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void gnssCallback(const sensor_msgs::NavSatFix::ConstPtr &msg);
    void wheelOdomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void loopImageCallback(const sensor_msgs::Image::ConstPtr &msg);
    void loopCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);

    void readParameters(ros::NodeHandle &nh);
    void initializePublishers(ros::NodeHandle &nh);
    void initializeISAM2();
    void initializeFactorModules(ros::NodeHandle &nh);

    void addKeyframe(const nav_msgs::Odometry::ConstPtr &msg);

    void publishOptimizedPath();
    void publishOptimizedOdometry(const gtsam::Pose3 &pose, double timestamp);
    void publishTF(const gtsam::Pose3 &pose, double timestamp);
    void saveTrajectory(const std::string &filename);
    void saveFullTrajectory(const std::string &filename);
    std::string createTimestampedDir(const std::string &base_dir);
    bool createDirectory(const std::string &path);
    void initializeDebugLog();
    void logDebugEvent(const std::string &message);

    std::mutex mtx_;

    ros::NodeHandle nh_;
    ros::Subscriber sub_livo_odom_;
    ros::Subscriber sub_gnss_;
    ros::Subscriber sub_wheel_;
    ros::Subscriber sub_loop_image_;
    ros::Subscriber sub_loop_cloud_;

    ros::Publisher pub_optimized_path_;
    ros::Publisher pub_optimized_odom_;

    std::vector<KeyFrame, Eigen::aligned_allocator<KeyFrame>> keyframes_;
    gtsam::NonlinearFactorGraph graph_;
    gtsam::Values initial_estimate_;
    gtsam::Values current_estimate_;
    std::shared_ptr<gtsam::ISAM2> isam2_;

    struct RawPose {
        double timestamp;
        gtsam::Pose3 raw_pose;
    };
    std::vector<RawPose, Eigen::aligned_allocator<RawPose>> raw_poses_;

    size_t keyframe_counter_;
    bool isam2_initialized_;
    bool first_livo_received_;

    /* ---- 可插拔因子模块列表 ---- */
    std::vector<std::shared_ptr<FactorModule>> factor_modules_;

    std::shared_ptr<GNSSFactorModule> gnss_module_;
    std::shared_ptr<WheelFactorModule> wheel_module_;
    std::shared_ptr<LoopDistanceModule> loop_distance_module_;
    std::shared_ptr<LoopClosureModule> loop_closure_module_;

    /* ---- 参数 ---- */
    int keyframe_skip_;
    double keyframe_distance_threshold_;
    double keyframe_angle_threshold_;
    double livox_pos_cov_;
    double livox_rot_cov_;
    bool enable_gnss_;
    bool enable_wheel_;
    bool enable_loop_distance_;
    bool enable_loop_closure_;
    std::string loop_mode_;
    std::string loop_image_topic_;
    std::string loop_cloud_topic_;
    std::string run_output_dir_;
    std::ofstream debug_log_;

    nav_msgs::Path optimized_path_;
};

#endif
