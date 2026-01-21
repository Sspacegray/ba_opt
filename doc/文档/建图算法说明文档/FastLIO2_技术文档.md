# FastLIO2 ROS2版本技术文档

## 项目概述

FastLIO2是一个高性能的激光雷达惯性里程计（LiDAR-Inertial Odometry）系统，基于迭代扩展卡尔曼滤波（iterated Extended Kalman Filter, iEKF）实现实时的机器人定位与建图。本文档详细分析了FastLIO2 ROS2版本的代码实现。

## 1. 系统架构

### 1.1 总体架构设计

```
┌─────────────────────────────────────────────────────────────┐
│                        LIONode (ROS2节点)                   │
├─────────────────────────────────────────────────────────────┤
│  传感器数据接收层                                             │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐         │
│  │  IMU订阅器   │  │ LiDAR订阅器  │  │  数据同步器  │         │
│  └─────────────┘  └─────────────┘  └─────────────┘         │
├─────────────────────────────────────────────────────────────┤
│  数据处理层                                                   │
│  ┌─────────────────────────────────────────────────────────┐ │
│  │                MapBuilder                              │ │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐    │ │
│  │  │IMUProcessor │  │LidarProcessor│  │    IESKF     │    │ │
│  │  │             │  │              │  │             │    │ │
│  │  │ - IMU初始化  │  │ - 点云配准    │  │ - 状态估计   │    │ │
│  │  │ - 去畸变     │  │ - 地图管理    │  │ - 协方差更新 │    │ │
│  │  │ - 预积分     │  │ - 特征提取    │  │ - 迭代优化   │    │ │
│  │  └─────────────┘  └─────────────┘  └─────────────┘    │ │
│  └─────────────────────────────────────────────────────────┘ │
├─────────────────────────────────────────────────────────────┤
│  输出发布层                                                   │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐         │
│  │  TF广播器    │  │  路径发布    │  │  点云发布    │         │
│  └─────────────┘  └─────────────┘  └─────────────┘         │
└─────────────────────────────────────────────────────────────┘
```

### 1.2 核心模块关系

```
┌──────────────┐    同步包    ┌──────────────┐
│              │  ────────→  │              │
│  IMUProcessor│              │LidarProcessor│
│              │  ←────────  │              │
└──────────────┘    状态      └──────────────┘
       │                             │
       │                             │
       ▼                             ▼
┌─────────────────────────────────────────────┐
│                  IESKF                      │
│  - 21维状态向量（姿态、位置、速度、偏置等）   │
│  - 迭代扩展卡尔曼滤波                       │
│  - 点到面约束优化                           │
└─────────────────────────────────────────────┘
```

## 2. 数据流程分析

### 2.1 系统工作流程

```
开始
 │
 ▼
┌─────────────────┐
│   传感器数据输入  │
│ • IMU数据流      │
│ • LiDAR点云流    │
└─────────────────┘
 │
 ▼
┌─────────────────┐
│   数据同步       │
│ • 时间戳对齐     │
│ • 数据包封装     │
└─────────────────┘
 │
 ▼
┌─────────────────┐      否     ┌─────────────────┐
│  IMU初始化？    │ ─────────→ │ IMU数据积累      │
│                 │            │ 估计重力方向     │
│                 │            │ 初始化偏置      │
└─────────────────┘            └─────────────────┘
 │ 是                                   │
 ▼                                     │
┌─────────────────┐                    │
│   点云去畸变     │ ←──────────────────┘
│ • IMU预积分      │
│ • 运动补偿       │
└─────────────────┘
 │
 ▼
┌─────────────────┐      否     ┌─────────────────┐
│  地图初始化？   │ ─────────→ │ 初始化局部地图   │
│                 │            │ 添加首帧点云     │
└─────────────────┘            └─────────────────┘
 │ 是                                   │
 ▼                                     │
┌─────────────────┐                    │
│   点云配准       │ ←──────────────────┘
│ • 特征点提取     │
│ • 最近邻搜索     │
│ • 平面拟合       │
└─────────────────┘
 │
 ▼
┌─────────────────┐
│   状态估计       │
│ • iEKF迭代优化   │
│ • 协方差更新     │
└─────────────────┘
 │
 ▼
┌─────────────────┐
│   地图更新       │
│ • 动态裁剪       │
│ • 增量建图       │
└─────────────────┘
 │
 ▼
┌─────────────────┐
│   结果输出       │
│ • TF变换        │
│ • 轨迹路径      │
│ • 点云地图      │
└─────────────────┘
```

### 2.2 数据类型定义

系统中的核心数据结构：

```cpp
// 21维状态向量
struct State {
    M3D r_wi;    // 世界到IMU旋转矩阵 (3x3)
    V3D t_wi;    // IMU在世界系中的位置 (3x1)
    M3D r_il;    // IMU到LiDAR旋转矩阵 (3x3)
    V3D t_il;    // IMU到LiDAR平移向量 (3x1)
    V3D v;       // IMU速度 (3x1)
    V3D bg;      // 陀螺仪偏置 (3x1)
    V3D ba;      // 加速度计偏置 (3x1)
    V3D g;       // 重力向量 (3x1)
};

// 同步数据包
struct SyncPackage {
    Vec<IMUData> imus;           // IMU数据序列
    CloudType::Ptr cloud;        // 点云数据
    double cloud_start_time;     // 点云开始时间
    double cloud_end_time;       // 点云结束时间
};
```

## 3. 核心算法详解

### 3.1 迭代扩展卡尔曼滤波器 (IESKF)

IESKF是系统的核心，实现了21维状态的估计：

#### 状态向量定义
$\mathbf{x} = [r_{wi}^T, t_{wi}^T, r_{il}^T, t_{il}^T, v^T, b_g^T, b_a^T]^T \in \mathbb{R}^{21}$

#### 预测步骤
```cpp
void IESKF::predict(const Input &inp, double dt, const M12D &Q) {
    // 状态传播
    r_wi *= exp(ω̂ * dt);           // 姿态更新
    t_wi += v * dt;                // 位置更新  
    v += (r_wi * (a - ba) + g) * dt; // 速度更新
    
    // 协方差传播
    P = F * P * F^T + G * Q * G^T;
}
```

#### 更新步骤
基于点到面距离约束的迭代优化：
```cpp
void IESKF::update() {
    for (int iter = 0; iter < max_iter; iter++) {
        // 计算观测残差和雅可比
        updateLossFunc(state, shared_data);
        
        // 构建增量方程 H*δx = b
        H = J^T * P^(-1) * J + H_obs;
        b = J^T * P^(-1) * δx + b_obs;
        
        // 求解状态增量
        δx = -H^(-1) * b;
        
        // 更新状态
        state += δx;
        
        // 收敛判断
        if (convergence_check(δx)) break;
    }
}
```

### 3.2 IMU预处理与点云去畸变

#### IMU初始化
```cpp
bool IMUProcessor::initialize(SyncPackage &package) {
    // 收集足够的IMU数据
    if (imu_cache.size() < imu_init_num) return false;
    
    // 计算静止状态下的平均值
    V3D acc_mean = compute_mean(acc_measurements);
    V3D gyro_mean = compute_mean(gyro_measurements);
    
    // 初始化状态
    kf->x().bg = gyro_mean;              // 陀螺仪偏置
    kf->x().g = -acc_mean.normalized() * 9.81; // 重力方向
    
    // 重力对齐（可选）
    if (gravity_align) {
        kf->x().r_wi = align_gravity_to_z_axis(acc_mean);
    }
    
    return true;
}
```

#### 运动补偿
对点云中的每个点根据其时间戳进行运动补偿：

```cpp
void IMUProcessor::undistort(SyncPackage &package) {
    // IMU预积分，生成轨迹
    for (auto& imu : imu_cache) {
        kf->predict(imu, dt, Q);
        poses_cache.push_back(current_pose);
    }
    
    // 对每个点进行去畸变
    for (auto& point : package.cloud->points) {
        double t = point.curvature / 1000.0;  // 相对时间
        
        // 插值获得该时刻的位姿
        Pose pose_t = interpolate_pose(poses_cache, t);
        
        // 将点变换到扫描结束时刻
        point = transform_to_end_time(point, pose_t);
    }
}
```

### 3.3 LiDAR处理与地图管理

#### 点云配准
```cpp
void LidarProcessor::process(SyncPackage &package) {
    // 点云降采样
    downsample_cloud(package.cloud);
    
    // 特征点选择与最近邻搜索
    select_features_and_find_neighbors();
    
    // 平面拟合与残差计算
    fit_planes_and_compute_residuals();
    
    // IESKF更新
    kf->update();
    
    // 地图管理
    trimCloudMap();   // 裁剪
    incrCloudMap();   // 增量更新
}
```

#### 动态地图管理
系统维护一个以当前位置为中心的立方体局部地图：

```cpp
void LidarProcessor::trimCloudMap() {
    Vector3d lidar_pos = current_lidar_position();
    
    // 检查是否需要移动地图边界
    if (distance_to_boundary(lidar_pos) < threshold) {
        // 计算新的地图边界
        BoxPointType new_boundary = compute_new_boundary(lidar_pos);
        
        // 删除地图外的点
        ikd_tree->Delete_Point_Boxes(boxes_to_remove);
        
        // 更新地图边界
        local_map_corner = new_boundary;
    }
}
```

## 4. 性能特点

### 4.1 算法优势
1. **实时性能**: 通过增量式KD树和局部地图管理实现高效处理
2. **鲁棒性**: 迭代EKF提供更好的非线性优化能力
3. **精度**: 点到面约束比点到点约束更稳定
4. **适应性**: 支持多种激光雷达类型（Livox, Velodyne等）

### 4.2 关键技术
1. **增量式KD树**: 高效的最近邻搜索和动态更新
2. **运动补偿**: 基于IMU预积分的精确去畸变
3. **局部地图**: 动态维护固定大小的地图窗口
4. **外参标定**: 在线估计IMU-LiDAR外参

## 5. 配置参数

### 5.1 主要参数说明
```yaml
# IMU相关
na: 0.01                    # 加速度计噪声
ng: 0.0005                  # 陀螺仪噪声  
nba: 0.0005                 # 加速度偏置噪声
nbg: 0.00002                # 陀螺仪偏置噪声
imu_init_num: 20            # IMU初始化帧数

# LiDAR相关
lidar_filter_num: 3         # 点云滤波参数
scan_down_sampling_rate: 0.1 # 扫描降采样率
map_resolution: 0.1         # 地图分辨率
cube_len: 30                # 局部地图边长
move_thresh: 0.8            # 地图移动阈值

# 优化相关
ieskf_max_iter: 15          # 最大迭代次数
near_search_num: 5          # 近邻点数量
plane_fitting_tolerance: 0.2 # 平面拟合容差
```

## 6. 工程实现特点

### 6.1 ROS2适配
- 使用ROS2的Publisher/Subscriber机制
- 支持多线程执行器提高并发性能
- 完整的TF变换发布

### 6.2 模块化设计
- 清晰的接口分离
- 易于扩展和维护
- 支持不同传感器配置

### 6.3 数据管理
- 高效的内存管理
- 智能指针使用
- 线程安全的数据访问

这个FastLIO2实现展现了现代SLAM系统的设计理念，通过合理的模块分工和高效的算法实现，在保证实时性的同时提供了高精度的定位与建图能力。