# livo_backend

Loosely-Coupled Multi-Sensor Fusion SLAM Backend for FAST-LIVO2.

将 FAST-LIVO2 作为"黑盒前端（高频里程计）"，在其外部套一层基于 **GTSAM iSAM2 因子图** 的后端，松耦合地融合 GNSS、轮速计、回环检测等多种传感器信息，输出全局一致的优化轨迹。

---

## Table of Contents

- [Why Loosely-Coupled?](#why-loosely-coupled)
- [Architecture Overview](#architecture-overview)
- [Factor Modules](#factor-modules)
  - [LIVO Odometry Factor（里程计因子）](#1-livo-odometry-factor)
  - [GNSS Factor (GPSFactor)](#2-gnss-factor)
  - [Wheel Odometry Factor（轮速计因子）](#3-wheel-odometry-factor)
  - [Loop Distance Factor（距离回环因子）](#4-loop-distance-factor)
- [Quick Start](#quick-start)
  - [Prerequisites](#prerequisites)
  - [Build](#build)
  - [Run with M3DGR Dataset](#run-with-m3dgr-dataset)
  - [Run with Your Own Data](#run-with-your-own-data)
- [Configuration Guide](#configuration-guide)
  - [Enabling / Disabling Modules](#enabling--disabling-modules)
  - [Parameter Reference](#parameter-reference)
- [Module Development Guide](#module-development-guide)
  - [Adding a New Factor Module](#adding-a-new-factor-module)
  - [Example: LoopVisualModule](#example-loopvisualmodule-stub)
- [Outputs](#outputs)
- [ROS Topic Interface](#ros-topic-interface)
- [Algorithm Details](#algorithm-details)
  - [iSAM2 Incremental Optimization](#isam2-incremental-optimization)
  - [WGS84 to ENU Coordinate Conversion](#wgs84-to-enu-conversion)
  - [Wheel Odometry Bicycle Model](#wheel-odometry-bicycle-model)
  - [Distance-Based Loop Closure](#distance-based-loop-closure)

---

## Why Loosely-Coupled?

FAST-LIVO2 的前端（IESKF）本身已经很重且容易发散，强行塞入低频的 GNSS 会破坏原有的代码结构。采用松耦合方案：

| Aspect | Tightly-Coupled（紧耦合） | Loosely-Coupled（松耦合） |
|--------|-------------------------|--------------------------|
| **架构** | GNSS/轮速计入 IESKF 状态向量 | FAST-LIVO2 作为黑盒，后端独立优化 |
| **代码侵入** | 需要修改 FAST-LIVO2 核心滤波逻辑 | 无需改动前端一行代码 |
| **可替换性** | 耦合紧密，替换前端困难 | 可通过 topic 替换任意里程计前端 |
| **故障隔离** | 前端发散导致整个系统崩溃 | 后端独立进程，前端崩溃可重启 |
| **扩展性** | 新增传感器需修改 IESKF | 新增传感器只需添加一个 FactorModule |
| **调试** | 需要理解整个前端代码 | 可单独调试后端因子图 |

---

## Architecture Overview

```
                          FAST-LIVO2 Frontend (black box)
                          ┌─────────────────────────────────────┐
                          │  LiDAR + IMU + Camera               │
                          │  IESKF Tightly-coupled Frontend     │
                          │  ~10-50 Hz odometry output          │
                          └──────────────┬──────────────────────┘
                                         │ /aft_mapped_to_init
                                         │ (nav_msgs::Odometry)
                                         ▼
 ┌─────────────────────────────────────────────────────────────────┐
 │                    livo_backend (factor graph)                   │
 │                                                                 │
 │  ┌─────────────────────────────────────────────────────────┐    │
 │  │  Factor Graph                                            │    │
 │  │                                                          │    │
 │  │  X0 ──Between── X1 ──Between── X2 ──Between── ... ──Xn  │    │
 │  │   │              │              │                        │    │
 │  │   │ GPSFactor    │ GPSFactor    │ GPSFactor              │    │
 │  │   │              │              │                        │    │
 │  │   │ WheelBetween│ WheelBetween│ WheelBetween            │    │
 │  │   │              │              │                        │    │
 │  │   │              │      ←── LoopBetween ──Xk            │    │
 │  │   │              │              │                        │    │
 │  └─────────────────────────────────────────────────────────┘    │
 │                                                                 │
 │  ┌────────────┬──────────────┬──────────────────┐               │
 │  │GNSSModule  │WheelModule   │LoopDistanceModule│   ← plugins  │
 │  └────────────┴──────────────┴──────────────────┘               │
 │                                                                 │
 │  Solver: GTSAM iSAM2 (incremental smoothing and mapping)        │
 └──────────────────────┬──────────────────────────────────────────┘
                        │
                        ▼
            /backend/optimized_odom  (优化后里程计)
            /backend/optimized_path  (全局优化轨迹)
            camera_init → backend_optimized (TF)
```

### Data Flow

1. **FAST-LIVO2** 以 10-50Hz 发布 `/aft_mapped_to_init` 里程计
2. **livo_backend** 按关键帧策略（`keyframe_skip`）采样，构建 BetweenFactor
3. 每个关键帧上，各 **FactorModule** 插件判断是否需要添加额外因子
4. 所有因子加入因子图后，**iSAM2** 进行增量式优化
5. 优化后的位姿通过 topic 和 TF 发布

---

## Factor Modules

系统采用可插拔的模块化架构。所有因子模块继承自 `FactorModule` 抽象基类：

```cpp
class FactorModule
{
public:
  virtual void loadParameters(ros::NodeHandle &nh, const std::string &ns) = 0;
  virtual std::vector<gtsam::NonlinearFactor::shared_ptr> evaluate(
      size_t keyframe_id,
      const std::vector<KeyFrame> &keyframes,
      const gtsam::Values &current_estimate) = 0;

  void setEnabled(bool en);
  bool isEnabled() const;
  const std::string &name() const;
};
```

### 1. LIVO Odometry Factor

**类型**: `BetweenFactor<Pose3>`

每一对相邻关键帧之间都会添加，约束两帧之间的相对位姿变化：

```
factor: X_{i} -- BetweenFactor(Pose3) --> X_{i+1}
measurement: delta = (Pose_i)^{-1} * Pose_{i+1}  (from FAST-LIVO2)
noise: diag(rot_cov, rot_cov, rot_cov, pos_cov, pos_cov, pos_cov)
```

这是因子图的**主干约束**，确保了局部轨迹的光滑性。

### 2. GNSS Factor

**类型**: `GPSFactor`

| 输入 | ROS Topic | Message Type |
|------|-----------|-------------|
| GNSS | `/gnss_data` (可配置) | `sensor_msgs/NavSatFix` |

**处理流程**:

1. 收到第一个 GNSS 消息时，将其设置为 ENU 坐标系原点
2. 后续所有 GNSS 数据通过 WGS84 椭球模型转换为 ENU 坐标
3. 在每个关键帧上，查找时间上最近的 GNSS 观测（默认 0.5s 窗口）
4. 若找到，添加 GPSFactor 约束该关键帧的全局位置

```
factor: GPSFactor(X_i, enu_position)
noise: diag(pos_cov, pos_cov, pos_cov * 2)
```

### 3. Wheel Odometry Factor

**类型**: `BetweenFactor<Pose3>`

| 输入 | ROS Topic | Message Type |
|------|-----------|-------------|
| 轮速计 | `/wheel_odometry` (可配置) | `geometry_msgs/TwistStamped` |

**处理流程**:

1. 接收轮速数据，使用自行车模型（bicycle model）计算两帧间的运动增量
2. 在每个关键帧上，查找时间上最近的轮速观测（默认 0.2s 窗口）
3. 维护上一帧的轮速状态，计算 `delta = (dx, dy, dyaw)`
4. 添加 BetweenFactor 约束相邻两帧的相对运动

```
factor: X_{i} -- BetweenFactor(Pose3) --> X_{i+1}
measurement: wheel_delta (from bicycle model)
noise: diag(rot_cov, rot_cov, rot_cov*0.1, pos_cov, pos_cov, pos_cov*10)
```

> **注意**: 轮速计的 Z 轴噪声较大（`pos_cov * 10`），因为轮速计通常无法观测垂直运动。

### 4. Loop Distance Factor

**类型**: `BetweenFactor<Pose3>`

基于空间距离的回环检测，无需额外的传感器数据。

**检测策略**:

1. 当新的关键帧加入因子图并完成优化后，检查其优化位姿
2. 遍历所有历史关键帧（跳过最近 `min_keyframe_gap` 帧）
3. 若空间距离 < `search_radius`，则认为是一个回环候选
4. 确保每个历史帧只触发一次回环（`loop_history_` 去重）

```
for each candidate i in [0, current_id - min_keyframe_gap]:
    if distance(pose_i, pose_current) < search_radius:
        add BetweenFactor(X_i, X_current, relative_pose)
```

---

## Quick Start

### Prerequisites

| Dependency | Version | Purpose |
|-----------|---------|---------|
| Ubuntu + ROS | Noetic | 运行环境 |
| FAST-LIVO2 | latest | 前端里程计 |
| GTSAM | >= 4.1.1 | 因子图优化 |
| Eigen3 | >= 3.3 | 线性代数 |
| PCL | >= 1.9 | 点云（CMake依赖） |

### Build

```bash
cd FAST-LIVO2_ws
catkin_make
source devel/setup.bash
```

### Run with M3DGR Dataset

**方法一：一键启动（前端 + 后端）**:

```bash
roslaunch livo_backend livo_loosely_coupled.launch
```

**方法二：分别启动（调试用）**:

```bash
# 终端 1：启动前端
roslaunch fast_livo mapping_m3dgr_mid360.launch

# 终端 2：启动后端
roslaunch livo_backend livo_backend_only.launch

# 终端 3：播放数据集
rosbag play /path/to/M3DGR/Varying-illu05.bag --clock
```

> **注意**: 根据数据集中实际的 topic 名称修改 `config/backend_m3dgr.yaml` 中的 `gnss_topic` 和 `wheel_topic`。可以先运行 `rosbag info your.bag` 查看所有 topic。

### Run with Your Own Data

1. 复制并修改配置文件：
   ```bash
   cp config/backend_m3dgr.yaml config/my_robot.yaml
   ```

2. 根据实际情况修改 topic 名称和参数：
   ```yaml
   livo_odom_topic: "/my_vo/odom"
   gnss_topic: "/gps/fix"
   wheel_topic: "/vesc/odom"
   ```

3. 创建自定义 launch 文件或直接启动：
   ```bash
   rosrun livo_backend livo_backend_node \
     _livo_odom_topic:=/my_vo/odom \
     _gnss_topic:=/gps/fix \
     _wheel_topic:=/vesc/odom
   ```

---

## Configuration Guide

完整的配置文件示例在 `config/backend_m3dgr.yaml`。

### Enabling / Disabling Modules

每个模块可以通过一个 `bool` 参数独立开关：

```yaml
enable_gnss: true            # GNSS 因子模块
enable_wheel: true           # 轮速计因子模块
enable_loop_distance: true   # 距离回环因子模块（方案一）
```

**测试场景示例**:

```yaml
# 场景 1: 纯 LIVO + GNSS（户外大场景）
enable_gnss: true
enable_wheel: false
enable_loop_distance: false

# 场景 2: LIVO + 轮速计 + 回环（隧道/GNSS失效）
enable_gnss: false
enable_wheel: true
enable_loop_distance: true

# 场景 3: 全部开启（最完整的多传感器融合）
enable_gnss: true
enable_wheel: true
enable_loop_distance: true

# 场景 4: 纯 LIVO（退化测试，对比后端效果）
enable_gnss: false
enable_wheel: false
enable_loop_distance: false
```

### Parameter Reference

#### Topic Names

```yaml
livo_odom_topic: "/aft_mapped_to_init"   # FAST-LIVO2 输出的里程计
gnss_topic: "/gnss_data"                 # GNSS 原始数据
wheel_topic: "/wheel_odometry"           # 轮速计数据
```

#### Keyframe Selection

```yaml
keyframe_skip: 5                    # 每 N 帧 LIVO 里程计取一帧关键帧
keyframe_distance_threshold: 0.5    # 与上一帧距离 < 此值则跳过（米）
keyframe_angle_threshold: 0.3       # 与上一帧角度 < 此值则跳过（弧度）
```

> 调大 `keyframe_skip` → 关键帧更稀疏 → 因子图优化更快，但精度可能下降。
> 调小 `keyframe_distance_threshold` → 更多关键帧 → 精度更高，但计算更慢。

#### Noise Covariances（共享层级）

```yaml
noise:
  livox_pos_cov: 0.01    # LIVO 里程计平移噪声方差
  livox_rot_cov: 0.001   # LIVO 里程计旋转噪声方差
  gnss_pos_cov: 9.0      # GNSS 位置噪声方差（默认）
  wheel_pos_cov: 0.5     # 轮速计平移噪声方差
  wheel_rot_cov: 0.1     # 轮速计旋转噪声方差
```

> **调参建议**:
> - `livox_pos_cov`: FAST-LIVO2 精度高时设小（如 0.01），振动大时设大
> - `gnss_pos_cov`: RTK-GNSS 设小（如 0.01），普通 GPS 设大（如 9.0-25.0）
> - `wheel_pos_cov`: 轮速计可信时设小（如 0.1），打滑时设大

#### Per-Module Parameters

```yaml
gnss_module:
  gnss_pos_cov: 9.0         # 覆盖共享层级的值
  time_tolerance: 0.5       # GNSS 与关键帧的时间匹配窗口（秒）

wheel_module:
  wheel_pos_cov: 0.5
  wheel_rot_cov: 0.1
  wheel_base: 1.5           # 车辆轴距（米），用于自行车模型
  time_tolerance: 0.2       # 轮速与关键帧的时间匹配窗口（秒）

loop_distance_module:
  search_radius: 5.0        # 回环搜索半径（米）
  loop_pos_cov: 0.5         # 回环因子的平移噪声
  loop_rot_cov: 0.1         # 回环因子的旋转噪声
  min_keyframe_gap: 50      # 最小帧间隔，避免相邻帧误检
  max_keyframe_gap: 500     # 最大帧间隔
```

---

## Module Development Guide

### Adding a New Factor Module

系统设计为易于扩展。要添加一个新的因子模块（例如视觉词袋回环），只需以下 5 步：

#### Step 1: 创建模块类

在 `livo_backend.h` 中添加：

```cpp
class LoopVisualModule : public FactorModule
{
public:
  LoopVisualModule() { name_ = "loop_visual"; }

  void loadParameters(ros::NodeHandle &nh, const std::string &ns) override;
  std::vector<gtsam::NonlinearFactor::shared_ptr> evaluate(
      size_t keyframe_id,
      const std::vector<KeyFrame> &keyframes,
      const gtsam::Values &current_estimate) override;

private:
  double visual_score_threshold_;
  double loop_pos_cov_;
  double loop_rot_cov_;
  int min_keyframe_gap_;
  std::map<size_t, size_t> loop_history_;
  // ... DBoW2 相关成员
};
```

#### Step 2: 实现接口

在 `livo_backend_node.cpp` 中添加实现：

```cpp
void LoopVisualModule::loadParameters(ros::NodeHandle &nh, const std::string &ns)
{
  nh.param<double>(ns + "/visual_score_threshold", visual_score_threshold_, 0.6);
  nh.param<double>(ns + "/loop_pos_cov", loop_pos_cov_, 0.3);
  nh.param<double>(ns + "/loop_rot_cov", loop_rot_cov_, 0.05);
  nh.param<int>(ns + "/min_keyframe_gap", min_keyframe_gap_, 30);
}

std::vector<gtsam::NonlinearFactor::shared_ptr> LoopVisualModule::evaluate(
    size_t keyframe_id,
    const std::vector<KeyFrame> &keyframes,
    const gtsam::Values &current_estimate)
{
  std::vector<gtsam::NonlinearFactor::shared_ptr> factors;
  // ... 视觉回环检测逻辑
  return factors;
}
```

#### Step 3: 声明成员

在 `LIOBackend` 类中添加：

```cpp
std::shared_ptr<LoopVisualModule> loop_visual_module_;
bool enable_loop_visual_;
```

#### Step 4: 注册模块

在 `initializeFactorModules()` 中添加：

```cpp
if (enable_loop_visual_)
{
  loop_visual_module_.reset(new LoopVisualModule());
  loop_visual_module_->loadParameters(nh, "loop_visual_module");
  factor_modules_.push_back(loop_visual_module_);
  ROS_INFO("[Backend] Visual loop closure module loaded");
}
```

#### Step 5: 添加配置

在 `backend_m3dgr.yaml` 中添加：

```yaml
enable_loop_visual: true     # 视觉词袋回环
# enable_loop_scan: false    # Scan Context 回环（同样方式添加）

loop_visual_module:
  visual_score_threshold: 0.6
  loop_pos_cov: 0.3
  loop_rot_cov: 0.05
  min_keyframe_gap: 30
```

**不需要修改后端主流程的任何代码**——这就是多态架构带来的扩展性。

### Example: LoopVisualModule (Stub)

如果你想快速有一个视觉回环检测的框架，可以参考以下伪代码思路：

```cpp
// livo_backend 订阅 /left_camera/image (或从前端获取特征)
// 在 evaluate() 中：
// 1. 提取当前帧的图像特征（ORB / SuperPoint等）
// 2. 查询 DBoW2 词袋，得到最相似的历史帧
// 3. 若相似度 > threshold，几何验证（PnP / Fundamental matrix）
// 4. 计算相对位姿
// 5. 构造 BetweenFactor 并返回
```

---

## Outputs

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/backend/optimized_odom` | `nav_msgs/Odometry` | 优化后的里程计（最新关键帧） |
| `/backend/optimized_path` | `nav_msgs/Path` | 全部关键帧的优化轨迹 |
| `camera_init → backend_optimized` | `tf` | 优化后位姿的 TF 变换 |

### Saved Files

```yaml
# 在 config/backend_m3dgr.yaml 中配置
output:
  trajectory_file: "Log/backend_optimized_traj.txt"
  optimized_path: "Log/backend_optimized_path.txt"
```

输出的轨迹文件格式（TUM 格式）：

```
# timestamp tx ty tz qx qy qz qw
1623456789.012 1.234 5.678 0.123 0.001 0.002 0.003 0.999
1623456789.112 1.334 5.778 0.124 0.001 0.002 0.003 0.999
...
```

可以使用 EVO 工具评估：

```bash
evo_ape tum ground_truth.txt Log/backend_optimized_traj.txt -a
evo_rpe tum ground_truth.txt Log/backend_optimized_traj.txt -a
```

---

## ROS Topic Interface

### Subscribed Topics

| Topic | Type | Default Name | Notes |
|-------|------|-------------|-------|
| LIVO Odometry | `nav_msgs/Odometry` | `/aft_mapped_to_init` | 由 FAST-LIVO2 发布 |
| GNSS Fix | `sensor_msgs/NavSatFix` | `/gnss_data` | WGS84 经纬高 |
| Wheel Odometry | `geometry_msgs/TwistStamped` | `/wheel_odometry` | linear.x = 速度, angular.z = 转向角 |

> **关于 GNSS topic**: M3DGR 数据集中的 GNSS topic 名称可能不是 `/gnss_data`。可以先运行 `rosbag info your.bag` 查看实际名称，然后修改配置文件中的 `gnss_topic`。

### Published Topics

| Topic | Type | Frame ID | Frequency |
|-------|------|----------|-----------|
| `/backend/optimized_odom` | `nav_msgs/Odometry` | `camera_init` | 与关键帧同频 |
| `/backend/optimized_path` | `nav_msgs/Path` | `camera_init` | 与关键帧同频 |

### TF

| From | To | Notes |
|------|-----|-------|
| `camera_init` | `backend_optimized` | 优化后的定位 TF |

---

## Algorithm Details

### iSAM2 Incremental Optimization

使用 GTSAM 的 iSAM2（Incremental Smoothing and Mapping）求解器进行增量式因子图优化。

**配置参数**:

```cpp
ISAM2Params parameters;
parameters.relinearizeThreshold = 0.01;  // 重线性化阈值
parameters.relinearizeSkip = 1;          // 每次更新都检查重线性化
parameters.factorization = ISAM2Params::CHOLESKY;  // 使用 Cholesky 分解
```

**优化流程**:

```
每添加一个关键帧:
  1. 添加 LIVO BetweenFactor
  2. 遍历所有 FactorModule，调用 evaluate() 获取额外因子
  3. 所有因子加入因子图
  4. isam2->update() 增量更新贝叶斯树
  5. isam2->calculateEstimate() 获取所有变量估计值
  6. 更新关键帧的优化后位姿
  7. 发布结果
```

iSAM2 的优势：
- **增量式**：每次只更新受影响的变量，而非全局重优化
- **贝叶斯树**：将因子图转换为贝叶斯树，利用稀疏性加速
- **实时性**：支持实时 SLAM 应用

### WGS84 to ENU Conversion

GNSS 使用 WGS84 坐标系（经纬高），需要转换为局部 ENU（东-北-天）坐标系才能与因子图结合。

**转换步骤**:

1. **设置原点**: 以第一个有效的 GNSS 观测点为 ENU 原点 `(lat0, lon0, alt0)`

2. **ECEF 中间转换**: 将 WGS84 坐标转换为地心地固坐标系（ECEF）:
   ```
   N = a / sqrt(1 - e^2 * sin(lat)^2)
   x = (N + alt) * cos(lat) * cos(lon)
   y = (N + alt) * cos(lat) * sin(lon)
   z = (N * (1 - e^2) + alt) * sin(lat)
   ```
   其中 `a = 6378137.0`（长半轴），`f = 1/298.257223563`（扁率）

3. **ENU 转换**: 将 ECEF 差值旋转到局部 ENU 坐标系:
   ```
   e = -sin(lon0) * dx + cos(lon0) * dy
   n = -sin(lat0)*cos(lon0) * dx - sin(lat0)*sin(lon0) * dy + cos(lat0) * dz
   u = cos(lat0)*cos(lon0) * dx + cos(lat0)*sin(lon0) * dy + sin(lat0) * dz
   ```

### Wheel Odometry Bicycle Model

轮速计因子使用自行车模型（Bicycle Model）计算两帧间的位姿增量：

**模型假设**:
- 车辆前后轮简化为一个前轮转向 + 后轮驱动的自行车
- 轮速计提供 `velocity`（线速度，来自 `linear.x`）和 `steering_angle`（前轮转向角，来自 `angular.z`）
- 轴距 `wheel_base` 可从配置文件设置，默认 1.5m

**运动模型**:

```
dt = t2 - t1
v = (v1 + v2) / 2                     # 平均速度
delta = (steer1 + steer2) / 2         # 平均转向角

if |delta| < epsilon:                 # 直线行驶
    dx = v * dt
    dy = 0
    dyaw = 0
else:                                 # 转弯
    R = wheel_base / tan(delta)       # 转弯半径
    dyaw = v * dt / R                 # 航向角变化
    dx = R * sin(dyaw)                # x 方向位移
    dy = R * (1 - cos(dyaw))          # y 方向位移
```

注意：自行车模型假设平面运动，因此 Z 轴和 roll/pitch 的方向噪声被设置得很大（`pos_cov * 10`），使得因子图更倾向于相信 LIVO 在这几个自由度的估计。

### Distance-Based Loop Closure

距离回环（方案一）是一种轻量级的回环检测方案，不需要额外的传感器数据。

**算法**:

```
输入: 当前关键帧 ID (current_id), 当前 iSAM2 估计值
输出: 回环因子列表

if current_id < min_keyframe_gap:
    return 空

cur_pose = estimate.at(X(current_id))
cur_pos = cur_pose.translation()

for i in [0, current_id - min_keyframe_gap]:
    if i 已经在 loop_history_ 中:      # 每个历史帧只触发一次回环
        continue
    if estimate 中没有 X(i):
        continue

    cand_pose = estimate.at(X(i))
    dist = |cur_pos - cand_pose.translation()|

    if dist < search_radius:
        relative = cand_pose.between(cur_pose)
        添加 BetweenFactor(X(i), X(current_id), relative)
        loop_history_[i] = current_id
        break   # 每个当前帧只匹配一个历史帧

return factors
```

**为什么要用优化后的位姿判断回环**:

使用 iSAM2 优化后的位姿（而非 LIVO 原始位姿）来判断回环，原因是：
- 优化后的位姿已经融合了 GNSS/轮速等信息，更加准确
- 在小范围漂移累积后，优化位姿可能比原始位姿更接近真实位置
- 在第一次回环纠正后，后续的位姿会进一步收敛，更容易检测到更多回环

**回环因子的协方差设置**:

距离回环的协方差应该比 LIVO 里程计因子大（更不确定），原因：
- 没有经过特征匹配验证，纯粹基于距离的假设
- 优化后的位姿本身存在误差，基于它计算的相对位姿不可完全信任
- 较大的协方差让 iSAM2 在优化时不过度依赖回环约束，避免错误回环破坏轨迹

---

## License

This project is provided under the BSD License. See `package.xml` for details.

## References

- [FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry](https://github.com/hku-mar/FAST-LIVO2)
- [GTSAM: Georgia Tech Smoothing and Mapping library](https://github.com/borglab/gtsam)
- [M3DGR Dataset](https://github.com/Blurryface0814/M3DGR)
- [iSAM2: Incremental Smoothing and Mapping with Fluid Relinearization](https://ieeexplore.ieee.org/document/6121345)