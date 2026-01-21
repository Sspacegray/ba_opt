# FastLIO2 程序数据流程图和架构图

## 1. 系统整体架构图

```mermaid
graph TD
    A[传感器输入层] --> B[数据同步层]
    B --> C[算法处理层]
    C --> D[结果输出层]
    
    subgraph "传感器输入层"
        A1[IMU传感器<br/>• 角速度<br/>• 线加速度<br/>• 时间戳]
        A2[LiDAR传感器<br/>• 点云数据<br/>• 强度信息<br/>• 时间戳]
    end
    
    subgraph "数据同步层"
        B1[IMU回调函数<br/>imuCB()]
        B2[LiDAR回调函数<br/>livoxCB()/lidarCB()]
        B3[数据同步器<br/>syncPackage()]
        B1 --> B3
        B2 --> B3
    end
    
    subgraph "算法处理层"
        C1[MapBuilder<br/>总控制器]
        C2[IMUProcessor<br/>IMU处理]
        C3[LidarProcessor<br/>点云处理]
        C4[IESKF<br/>状态估计器]
        C1 --> C2
        C1 --> C3
        C2 --> C4
        C3 --> C4
    end
    
    subgraph "结果输出层"
        D1[TF变换发布<br/>坐标系关系]
        D2[路径发布<br/>机器人轨迹]
        D3[点云发布<br/>实时地图]
        D4[里程计发布<br/>位姿速度]
    end
    
    A1 --> B1
    A2 --> B2
    C --> D1
    C --> D2
    C --> D3
    C --> D4
```

## 2. 主程序执行流程

```mermaid
flowchart TD
    Start([程序启动]) --> Init[节点初始化]
    Init --> Config[加载配置参数]
    Config --> CreateSub[创建订阅器]
    CreateSub --> CreatePub[创建发布器]
    CreatePub --> Timer[启动定时器]
    Timer --> Wait[等待数据]
    
    Wait --> CheckSync{数据同步成功?}
    CheckSync -->|否| Wait
    CheckSync -->|是| Process[处理数据包]
    
    Process --> CheckIMU{IMU已初始化?}
    CheckIMU -->|否| IMUInit[IMU初始化]
    IMUInit --> InitSuccess{初始化成功?}
    InitSuccess -->|否| Wait
    InitSuccess -->|是| StateInit[状态设为MAP_INIT]
    StateInit --> Wait
    
    CheckIMU -->|是| Undistort[点云去畸变]
    Undistort --> CheckMap{地图已初始化?}
    CheckMap -->|否| MapInit[地图初始化]
    MapInit --> StateMapping[状态设为MAPPING]
    StateMapping --> Wait
    
    CheckMap -->|是| LidarProcess[LiDAR处理]
    LidarProcess --> Publish[发布结果]
    Publish --> Wait
```

## 3. IESKF状态估计详细流程

```mermaid
flowchart TD
    Start([IESKF开始]) --> Predict[预测步骤]
    
    subgraph "预测步骤 predict()"
        Predict --> P1[状态传播<br/>姿态: r_wi *= exp(ω*dt)<br/>位置: t_wi += v*dt<br/>速度: v += (R*(a-ba)+g)*dt]
        P1 --> P2[雅可比计算<br/>状态转移矩阵F<br/>噪声矩阵G]
        P2 --> P3[协方差传播<br/>P = F*P*F^T + G*Q*G^T]
    end
    
    P3 --> Update[更新步骤]
    
    subgraph "更新步骤 update()"
        Update --> U1[保存预测状态<br/>predict_x = x]
        U1 --> IterStart[开始迭代 i=0]
        IterStart --> U2[计算观测<br/>调用损失函数]
        U2 --> U3{观测有效?}
        U3 -->|否| IterEnd
        U3 -->|是| U4[构建信息矩阵<br/>H = J^T*P^-1*J + H_obs<br/>b = J^T*P^-1*δx + b_obs]
        U4 --> U5[求解增量<br/>δx = -H^-1*b]
        U5 --> U6[状态更新<br/>x += δx]
        U6 --> U7[迭代计数<br/>i++]
        U7 --> U8{收敛检查}
        U8 -->|未收敛| U2
        U8 -->|收敛| U9[更新协方差<br/>P = L*H^-1*L^T]
    end
    
    U9 --> IterEnd([IESKF结束])
```

## 4. IMU处理流程

```mermaid
flowchart TD
    IMUStart([IMU处理开始]) --> InitCheck{需要初始化?}
    
    subgraph "IMU初始化 initialize()"
        InitCheck -->|是| I1[积累IMU数据]
        I1 --> I2{数据量足够?}
        I2 -->|否| I3[等待更多数据]
        I3 --> I1
        I2 -->|是| I4[计算平均值<br/>acc_mean, gyro_mean]
        I4 --> I5[设置初始偏置<br/>bg = gyro_mean]
        I5 --> I6[重力对齐处理<br/>估计重力方向]
        I6 --> I7[初始化协方差矩阵]
        I7 --> InitDone[初始化完成]
    end
    
    InitCheck -->|否| Undist[点云去畸变]
    InitDone --> ProcessEnd
    
    subgraph "点云去畸变 undistort()"
        Undist --> U1[准备IMU缓存<br/>添加前一帧最后IMU]
        U1 --> U2[初始化姿态缓存<br/>记录t=0时刻状态]
        U2 --> U3[IMU预积分循环]
        
        subgraph "预积分处理"
            U3 --> U4[计算中值<br/>gyro = 0.5*(head+tail)<br/>acc = 0.5*(head+tail)]
            U4 --> U5[调用IESKF预测<br/>更新状态和协方差]
            U5 --> U6[记录当前姿态<br/>存储到poses_cache]
            U6 --> U7{还有IMU数据?}
            U7 -->|是| U4
        end
        
        U7 -->|否| U8[外推到扫描结束]
        U8 --> U9[点云运动补偿]
        
        subgraph "运动补偿"
            U9 --> U10[遍历所有点]
            U10 --> U11[根据时间戳插值姿态]
            U11 --> U12[计算补偿变换<br/>将点变换到扫描结束时刻]
            U12 --> U13{处理完所有点?}
            U13 -->|否| U10
        end
    end
    
    U13 -->|是| ProcessEnd([IMU处理结束])
```

## 5. LiDAR处理流程

```mermaid
flowchart TD
    LStart([LiDAR处理开始]) --> L1[点云降采样<br/>体素滤波]
    L1 --> L2[特征点选择]
    L2 --> L3[最近邻搜索<br/>使用ikd-tree]
    L3 --> L4[平面拟合]
    L4 --> L5[残差计算]
    L5 --> L6[调用IESKF更新]
    L6 --> L7[地图管理]
    
    subgraph "地图管理"
        L7 --> M1{需要裁剪地图?}
        M1 -->|是| M2[计算新边界]
        M2 --> M3[删除超出范围点]
        M3 --> M4[更新地图边界]
        M1 -->|否| M5[增量更新地图]
        M4 --> M5
        M5 --> M6[添加新扫描点]
    end
    
    M6 --> LEnd([LiDAR处理结束])
    
    subgraph "残差计算详情"
        L5 --> R1[遍历有效特征点]
        R1 --> R2[搜索k近邻点]
        R2 --> R3[拟合平面方程<br/>ax+by+cz+d=0]
        R3 --> R4[计算点到平面距离]
        R4 --> R5[计算雅可比矩阵<br/>∂r/∂x]
        R5 --> R6[填充H矩阵和b向量]
    end
```

## 6. 数据结构关系图

```mermaid
classDiagram
    class State {
        +M3D r_wi : 世界到IMU旋转
        +V3D t_wi : IMU位置
        +M3D r_il : IMU到LiDAR旋转  
        +V3D t_il : IMU到LiDAR平移
        +V3D v : 速度
        +V3D bg : 陀螺仪偏置
        +V3D ba : 加速度偏置
        +V3D g : 重力向量
        +operator+=() : 状态更新
        +operator-() : 状态差分
    }
    
    class IESKF {
        -State m_x : 当前状态
        -M21D m_P : 协方差矩阵
        -loss_func m_loss_func : 损失函数
        -stop_func m_stop_func : 停止条件
        +predict() : 预测步骤
        +update() : 更新步骤
    }
    
    class SyncPackage {
        +Vec~IMUData~ imus : IMU数据序列
        +CloudType::Ptr cloud : 点云
        +double cloud_start_time : 开始时间
        +double cloud_end_time : 结束时间
    }
    
    class MapBuilder {
        -IMUProcessor m_imu_processor
        -LidarProcessor m_lidar_processor
        -IESKF m_kf
        -BuilderStatus m_status
        +process() : 主处理函数
    }
    
    class IMUProcessor {
        -IESKF m_kf
        -Vec~IMUData~ m_imu_cache
        -Vec~Pose~ m_poses_cache
        +initialize() : IMU初始化
        +undistort() : 去畸变
    }
    
    class LidarProcessor {
        -IESKF m_kf
        -KD_TREE m_ikdtree
        -LocalMap m_local_map
        +process() : 点云处理
        +updateLossFunc() : 更新损失函数
        +trimCloudMap() : 地图裁剪
        +incrCloudMap() : 增量建图
    }
    
    MapBuilder --> IMUProcessor
    MapBuilder --> LidarProcessor  
    MapBuilder --> IESKF
    IMUProcessor --> IESKF
    LidarProcessor --> IESKF
    IESKF --> State
    IMUProcessor --> SyncPackage
    LidarProcessor --> SyncPackage
```

## 7. 时间序列处理流程

```mermaid
sequenceDiagram
    participant S as 传感器
    participant N as ROS2节点
    participant M as MapBuilder
    participant I as IMUProcessor
    participant L as LidarProcessor
    participant K as IESKF
    
    loop 实时处理循环
        S->>N: IMU数据流
        S->>N: LiDAR数据流
        
        Note over N: 数据缓冲和同步
        N->>N: syncPackage()
        
        alt IMU未初始化
            N->>M: process(package)
            M->>I: initialize(package)
            I->>K: 设置初始状态
            K-->>I: 初始化结果
            I-->>M: 初始化状态
        else 正常处理
            N->>M: process(package)
            M->>I: undistort(package)
            I->>K: predict(多次调用)
            K-->>I: 预测状态
            I-->>M: 去畸变完成
            
            alt 地图未初始化
                M->>L: initCloudMap()
            else 正常建图
                M->>L: process(package)
                L->>L: 特征提取和匹配
                L->>K: update()(通过loss函数)
                K-->>L: 优化后状态
                L->>L: 更新地图
            end
        end
        
        M-->>N: 处理完成
        N->>N: 发布结果(TF,路径,点云等)
    end
```

这些图表详细展示了FastLIO2系统的完整架构和数据流程，从传感器数据输入到最终结果输出的每个步骤都有清晰的说明。