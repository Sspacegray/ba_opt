### indoor_location说明：

1 cloud_process：揽沃雷达mid_360以及rs16数据处理
发布的话题：/deskew_cloud，数据类型 sensor_msgs::msg::PointCloud2
2 nano_flann：用来存储全局pcd地图的kdtree树数据结构，提高匹配效率
3 ndt：三维点云匹配库
4 p2pl_icp：三维点面匹配算法
5 utility：定义基础函数类型
6 location_livox_p2pl_icp.cpp：揽沃mid_360定位总程序,使用点面匹配算法p2pl_icp，location_livox_ndt.cpp 揽沃mid_360定位总程序,使用点面匹配算法ndt

2 ###功能说明

1 支持手动重定位功能，rviz2中点2D Pose Estimate就可以给定重定位位姿

2 支持雷达角度距离数据裁剪、以及机械雷达rs16去畸变(慎重使用，特别是大场景点云多的话容易影响定位效率)

2 发布/agv_pose位姿话题以及map-baselink的tf变换

3 需要更高精度的话可以调整参数体素地图、点云范围、点云体素、匹配迭代次数

2 ####运行说明：
ros2 launch indoor_location run_rs16.launch.py  #速腾雷达rs16，搭载wit的imu；
ros2 launch indoor_location run_livox_mid360.launch.py #ea100搭载livox_mid360雷达；
ros2 launch indoor_location run_qr100_livox_mid360.launch.py #qr100搭载livox_mid360雷达
对于大场景使用ndt做前端，只需要在CMakeLists.txt修改location_ndt.cpp，location_p2pl_icp.cpp程序，启动不同雷达的yaml文件；

总结：对于30米左右的距离雷达，小范围使用的话可以使用p2picp；对于大环境得使用距离更远80米以上雷达，使用ndt;当然后续也可以使用ndt做粗匹配，p2picp做细匹配来增加鲁棒性，但是有带来一个问题eskf位姿出来得的耗时会更大，为了更好的兼容性会持续更新

## 版本优化更新 2025-10-16

### 时间戳同步优化，全部使用雷达时间戳进行处理数据

**问题描述：**
在rviz2中观察到点云和定位位姿存在滞后现象，主要原因是数据处理链中时间戳不一致。

**修改内容：**

#### 1. 点云处理节点 (`cloud_process_livox_node.cpp`)
```cpp
// 修改前：使用当前系统时间
tempCloud.header.stamp = this->get_clock()->now();

// 修改后：保持雷达原始时间戳
tempCloud.header.stamp = cloud_msg->header.stamp;  // 保持原始时间戳，避免滞后
```

#### 2. RS16点云处理节点 (`cloud_process_rs_node.cpp`)
```cpp
// 修改前：使用当前系统时间
tempCloud.header.stamp = this->get_clock()->now();

// 修改后：保持雷达原始时间戳
tempCloud.header.stamp = cloud_msg->header.stamp;  // 保持原始时间戳，避免滞后
```

#### 3. 定位节点 (`location_p2pl_icp.cpp` 和 `location_ndt.cpp`)
```cpp
// 修改 publish_pose 函数，接受点云时间戳参数
void publish_pose(pose_type pose, rclcpp::Time cloud_timestamp)

// 位姿发布使用点云时间戳
pose_stamped.header.stamp = cloud_timestamp;  // 使用点云时间戳保持同步

// TF变换也使用点云时间戳
transform_base_stamped.header.stamp = cloud_timestamp;  // 使用点云时间戳保持TF同步

// 位姿时间戳记录
act_pose.timestamp = cloud_timestamp.seconds();  // 使用点云时间戳
```

**涉及文件：**
- `cloud_process_livox_node.cpp` - Livox雷达点云处理
- `cloud_process_rs_node.cpp` - RS16雷达点云处理  
- `location_p2pl_icp.cpp` - P2P-ICP定位算法
- `location_ndt.cpp` - NDT定位算法

**优化效果：**
- ✅ 解决了rviz2中点云和位姿显示的时间滞后问题
- ✅ 确保整个数据处理链时间戳一致性
- ✅ 提高了TF变换的时间同步精度
- ✅ 改善了rviz2中的可视化体验
- ✅ 适用于所有雷达类型和定位算法

**使用方法：**
无需额外配置，重新编译后直接使用原有启动命令即可：
```bash
# RS16雷达
ros2 launch indoor_location run_rs16.launch.py

# Livox Mid360雷达 
ros2 launch indoor_location run_cr100_livox_mid360.launch.py
ros2 launch indoor_location run_qr100_livox_mid360.launch.py
```
## 版本优化更新 2025-10-18

### 处理日志文件，使用RCLCPP_DEBUG；提高tf位姿输出频率，使用时间imu预计分的位姿发布
RCLCPP_DEBUG(nh_->get_logger(), "(p2picp_pose).seconds(): time %f", (nh_->get_clock()->now() - store_pose_timestamp_).seconds());
// 高频发布IMU积分结果
rclcpp::Time imu_timestamp_ros((*imu_iter)->header.stamp);
 publish_pose(act_pose, imu_timestamp_ros);
RCLCPP_DEBUG(nh_->get_logger(), "Published high-freq IMU pose at time: %f", imu_timestamp);

优点：位姿频率提升，能最高达到130hz,cpu的能耗更小，不会超过两个核；
缺点：单个先验位姿的话可能不收敛，发散，导致位姿可能巨变，跳动导致导航运动不平顺；

# 定位性能统计功能 - 更新总结 2025-10-21

## 更新内容

根据您的需求，我已经完成了定位系统的性能统计功能，特别添加了**最终定位位姿发布的时间统计**。

## 新增的关键指标

### 1. 位姿发布耗时 (pose_publish_time_ms)
- 测量位姿消息发布和TF广播的处理时间
- 包括ROS2消息序列化和发送的开销

### 2. 总延迟 (total_latency_ms) ⭐ **重点指标**
- **测量范围**: 从点云数据接收到最终位姿发布完成的完整时间
- **包含内容**:
  - 所有处理模块的时间（IMU预测、点云配准、ESKF校正、位姿发布）
  - ROS2消息传输时间
  - 系统调度延迟
  - 其他系统开销

### 3. 时间戳记录
- `cloud_receive_time`: 点云接收的系统时间
- `final_publish_time`: 最终位姿发布的系统时间
- 用于精确计算端到端延迟

## 完整的性能指标列表

现在系统统计以下所有指标：

| 指标 | 说明 | 典型值 |
|------|------|--------|
| 定位发布频率 | 位姿发布的实际频率 | ~10 Hz |
| IMU预测耗时 | IMU预积分处理时间 | < 10 ms |
| 点云配准耗时 | P2P ICP配准时间 | 30-100 ms |
| ESKF校正耗时 | 卡尔曼滤波校正时间 | < 5 ms |
| 位姿发布耗时 | 消息发布和TF广播时间 | < 2 ms |
| 总处理时间 | 所有CPU处理时间总和 | < 100 ms |
| **总延迟** | **端到端完整延迟** | **< 120 ms** |
| IMU发布次数 | 每帧高频IMU输出次数 | 5-20 次 |

## 输出示例

```
========== 定位性能统计 (最近50帧) ==========
定位发布频率: 10.23 Hz (总共发布 512 次)
--- 各模块平均耗时 ---
  IMU预测:       5.32 ms
  点云配准:       45.67 ms
  ESKF校正:      2.14 ms
  位姿发布:       0.85 ms
  总处理时间:     54.63 ms
  总延迟(含网络): 58.42 ms  ← 这是您关心的端到端时间
  平均IMU发布次数: 8.5 次/帧
--- 最新一帧耗时 ---
  IMU预测:       5.12 ms
  点云配准:       46.23 ms
  ESKF校正:      2.08 ms
  位姿发布:       0.91 ms
  总处理时间:     54.98 ms
  总延迟(含网络): 59.15 ms  ← 实时监控这个值
  IMU发布次数:   9 次
==========================================
```

## 定位算法框架 2025-10-31

1. 数据同步 ✓
   └─ 等待IMU覆盖点云时间范围

2. IMU预积分 ✓ (这是ESKF的预测步骤)
   └─ 对每个IMU调用 eskf->predict()
   └─ 得到预测位姿

3. 点云配准 ✓
   └─ 使用预测位姿作为初值
   └─ ICP配准得到观测位姿

4. ESKF校正 ✓ (这是ESKF的更新步骤)
   └─ 融合预测和观测
   └─ 得到最优位姿

5. 发布位姿 ✓

## 添加点云去畸变功能(基于IMU的运动补偿)2025-11-4

主要改动:
1. 在location_p2pl_icp.cpp中实现去畸变功能:
   - 添加Sophus库支持(SO3/SE3旋转表示)
   - 实现IMU插值函数interpolateImu()
   - 实现旋转积分函数integrateRotation()
   - 实现点云去畸变函数undistortPointCloud()
   - 在timerCB中集成去畸变处理流程

2. 添加配置参数enable_undistort:
   - 支持通过YAML配置文件控制是否启用去畸变
   - 默认值为true(启用)
   - 所有配置文件已添加该参数

3. 更新CMakeLists.txt:
   - 添加Sophus库依赖
   - 添加fmt库依赖

4. 新增配置文件:
   - config/qr100_sucai_config_livox_mid360.yaml
   - launch/run_qr100_sucai_livox_mid360.launch.py

技术特性:
- 使用IMU角速度进行点云运动补偿
- 支持RS16和Livox等不同雷达类型
- 去畸变耗时约4-5ms,性能开销小
- 帧计数器与处理流程同步
- 详细的日志输出便于调试"