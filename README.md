# UAV-Delta 无人机协同捕获系统

基于 STM32MP257F-DK（Cortex-A35 + Cortex-M33 + NPU）的无人机与 Delta 机械臂协同捕获系统。

## 系统架构

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         Cortex-A35 (Linux + ROS 2 Humble)               │
│                                                                         │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐ │
│  │  uwb_driver   │  │uwb_navigation│  │  fcu_bridge   │  │vision_bridge │ │
│  │  UWB 数据采集 │  │ UWB 导航控制 │  │  飞控通信桥接 │  │  视觉桥接    │ │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘ │
│         │                 │                 │                 │          │
│  ┌──────┴───────┐  ┌──────┴───────┐  ┌──────┴───────┐                │
│  │   safety      │  │delta_kinemat.│  │ uav_delta_msgs│                │
│  │  安全管理     │  │  臂运动学    │  │  自定义消息   │                │
│  └──────────────┘  └──────────────┘  └──────────────┘                │
│                                                                         │
│  ┌──────────────────────────────────────────────────────────────────┐  │
│  │                     Docker (ros2humble)                          │  │
│  │  ros-humble-desktop-full + mavros + vision_opencv               │  │
│  └──────────────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────────┘
        │ UART            │ UART/USB          │ Ethernet
   ┌────┴────┐       ┌────┴────┐         ┌────┴────┐
   │ UWB模块 │       │ 飞控    │         │ Jetson  │
   │ ALX-AOA │       │ PX4/ArduPilot│    │ 视觉处理│
   └─────────┘       └─────────┘         └─────────┘
```

## 包结构

| 包名 | 功能 | 关键节点 |
|------|------|----------|
| **uwb_driver** | UWB AOA 数据采集与滤波 | `uwb_aoa_driver_node` |
| **uwb_navigation** | UWB 导航控制与任务规划 | `uwb_mission_planner_node` |
| **fcu_bridge** | 飞控通信桥接 | `fcu_state_node`, `attitude_publisher_node`, `flight_commander_node` |
| **vision_bridge** | 视觉坐标变换桥接 | `vision_transform_node`, `perception_node` |
| **safety** | 安全管理与故障处理 | `failsafe_manager_node` |
| **uav_delta_msgs** | 自定义消息/服务/动作定义 | - |
| **delta_kinematics** | Delta 机械臂运动学 | `delta_kinematics_node` |
| **vision_test** | 视觉测试工具 | `vision_test_node` |

## 快速开始

### 环境要求

- STM32MP257F-DK 开发板
- Docker 环境（镜像: `my_ros2_humble:latest`）
- 飞控（PX4/ArduPilot）通过 MAVROS 连接
- UWB ALX-AOA-FIT 模块（UART 115200）

### 启动 Docker 环境

```bash
# 启动 Docker 守护进程（如果未运行）
nohup dockerd --data-root /usr/local/docker > /tmp/dockerd.log 2>&1 &

# 进入 ROS 2 容器
docker exec -it ros2humble bash
```

### 编译工作空间

```bash
cd /usr/local/Uav_Delta_capture/uav_delta_capture
colcon build --symlink-install
source install/setup.bash
```

### 运行示例

```bash
# 启动完整的 UWB 导航系统
ros2 launch uwb_navigation uwb_navigation_system.launch.py

# 或单独启动各模块
ros2 launch uwb_driver uwb_aoa_driver.launch.py
ros2 launch uwb_navigation uwb_mission_planner.launch.py
ros2 launch fcu_bridge attitude_publisher.launch.py
ros2 launch vision_bridge vision_transform.launch.py
ros2 launch safety failsafe_manager.launch.py
```

## 核心节点说明

### uwb_aoa_driver_node（UWB 数据采集）

从 UWB 模块读取 AOA 数据，解析 37 字节帧，应用卡尔曼滤波。

- **订阅**: 无（直接读取串口）
- **发布**: `uwb_aoa/data`（`UavDeltaMsgs/UwbAoa`）
- **参数**: `serial_port`, `baud_rate`, `kalman_q`, `kalman_r`

### uwb_mission_planner_node（任务规划器）

UWB 导航状态机，控制无人机按预设路径飞行。

- **订阅**: `uwb_aoa/data`, `mavros/state`, `mavros/local_position/pose`
- **发布**: `mavros/setpoint_velocity/cmd_vel_unstamped`, `uwb_mission/state`, `uwb_mission/event`
- **状态**: IDLE → ARMING → TAKEOFF → HOVER_TAKEOFF → MOVE_ABOVE → HOVER_ABOVE → DESCEND → HOVER_FINAL → DONE

### attitude_publisher_node（姿态发布）

从 MAVROS 提取姿态角并发布。

- **订阅**: `mavros/local_position/pose`, `mavros/imu/data`
- **发布**: `fcu/local_attitude`, `fcu/imu_attitude`

### vision_transform_node（视觉变换）

使用 TF2 将视觉目标从相机坐标系变换到机械臂坐标系。

- **订阅**: `vision/target_offset`
- **发布**: `target_point`
- **TF**: `camera_optical_frame` → `delta_base_link`

### failsafe_manager_node（安全管理）

监控系统状态，在异常时触发保护动作。

- **订阅**: `mavros/state`, `mavros/battery`, `uwb_mission/state`
- **发布**: `mavros/setpoint_velocity/cmd_vel_unstamped`, `failsafe/status`

## 关键话题

| 话题 | 消息类型 | 说明 |
|------|----------|------|
| `uwb_aoa/data` | `UavDeltaMsgs/UwbAoa` | UWB 测距/角度数据 |
| `uwb_mission/state` | `std_msgs/String` | 任务状态 |
| `uwb_mission/event` | `std_msgs/String` | 任务事件 |
| `fcu/local_attitude` | `geometry_msgs/Vector3Stamped` | 飞控姿态角 |
| `target_point` | `geometry_msgs/PointStamped` | 视觉目标点（臂坐标系） |
| `cmd_vel` | `geometry_msgs/Twist` | 速度控制指令 |

## UWB 协议说明

ALX-AOA-FIT 模块使用 UART 115200 通信，帧格式：

```
帧头(2B) + 长度(2B) + 命令(2B) + 数据(28B) + 校验(1B) + 帧尾(2B) = 37 字节
```

数据字段：
- `Distance`: uint32, 距离（cm）
- `Azimuth`: int16, 水平角度（°, ±90°）
- `Elevation`: int16, 垂直角度（°, ±30°）

精度：距离 ±10cm，角度 ±2°

## 配置文件

各节点配置文件位于对应包的 `config/` 目录：

```
uwb_driver/config/uwb_aoa_driver.yaml
uwb_navigation/config/uwb_mission_planner.yaml
fcu_bridge/config/attitude_publisher.yaml
vision_bridge/config/vision_transform.yaml
```

## 项目结构

```
uav_delta_capture/
├── src/
│   ├── uav_delta_msgs/          # 自定义消息
│   ├── delta_kinematics/        # 机械臂运动学
│   ├── uwb_driver/              # UWB 数据采集
│   │   ├── src/
│   │   ├── config/
│   │   └── launch/
│   ├── uwb_navigation/          # UWB 导航控制
│   │   ├── src/
│   │   ├── config/
│   │   └── launch/
│   ├── fcu_bridge/              # 飞控通信
│   │   ├── src/
│   │   ├── config/
│   │   └── launch/
│   ├── vision_bridge/           # 视觉桥接
│   │   ├── src/
│   │   ├── config/
│   │   └── launch/
│   ├── safety/                  # 安全管理
│   │   ├── src/
│   │   └── launch/
│   └── vision_test/             # 视觉测试
```

## 开发说明

### 添加新节点

1. 在对应包的 `src/` 下创建源文件
2. 更新 `CMakeLists.txt` 添加可执行文件
3. 更新 `package.xml` 添加依赖
4. 创建配置文件（如需要）
5. 创建启动文件

### 消息定义

自定义消息在 `uav_delta_msgs/msg/` 下定义，编译后自动生成头文件。

## 许可证

[待定]
