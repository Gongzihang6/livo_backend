#ifndef LIVO_BACKEND_H
#define LIVO_BACKEND_H

#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

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

#include <cmath>
#include <ctime>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <mutex>
#include <sys/stat.h>
#include <vector>

using namespace gtsam;

using gtsam::symbol_shorthand::X;

/* ========================================================================
 * 数据结构定义
 * ======================================================================== */

struct KeyFrame {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // 添加此宏以保证内存对齐
    double timestamp;
    gtsam::Pose3 livo_pose;
    gtsam::Pose3 optimized_pose;
    size_t id;
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

    std::mutex mtx_;

    ros::NodeHandle nh_;
    ros::Subscriber sub_livo_odom_;
    ros::Subscriber sub_gnss_;
    ros::Subscriber sub_wheel_;

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

    /* ---- 参数 ---- */
    int keyframe_skip_;
    double keyframe_distance_threshold_;
    double keyframe_angle_threshold_;
    double livox_pos_cov_;
    double livox_rot_cov_;
    bool enable_gnss_;
    bool enable_wheel_;
    bool enable_loop_distance_;
    std::string run_output_dir_;

    nav_msgs::Path optimized_path_;
};

#endif