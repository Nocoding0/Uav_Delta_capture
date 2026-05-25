# uav_delta_capture

基于 ROS 2 Humble 的“无人机挂载 Delta 机械臂协同抓取”项目框架，面向 STM32MP257F-DK（A35 + NPU）平台。

## 工作空间结构

```text
uav_delta_capture/
└── src/
    ├── uav_delta_msgs/      # 自定义消息与服务
    ├── delta_kinematics/    # Delta 正/逆运动学节点（C++）
    ├── perception_logic/    # 视觉原型节点（Python，预留 NPU 推理接口）
    └── uav_bridge/          # MAVROS/TF2 桥接节点（C++）
```

## 快速开始

```bash
cd /home/ros2_ws/MyDrone/uav_delta_capture
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 运行

```bash
ros2 run delta_kinematics delta_kinematics_node
ros2 run perception_logic perception_node
ros2 run uav_bridge uav_bridge_node
```

或使用统一启动：

```bash
ros2 launch uav_bridge uav_delta_system.launch.py
```

统一启动支持参数切换：

```bash
# Mock 飞控输入
ros2 launch uav_bridge uav_delta_system.launch.py use_mock_fcu:=true start_mavros:=false

# 真实飞控输入（需已安装 mavros）
ros2 launch uav_bridge uav_delta_system.launch.py use_mock_fcu:=false start_mavros:=true
```

使用 Mock 飞控位姿（无数传联调）：

```bash
ros2 launch uav_bridge uav_delta_with_mock_fcu.launch.py
```

使用真实飞控通信链路（含健康监控/状态机/failsafe 基础模块）：

```bash
ros2 launch uav_bridge uav_delta_with_fcu.launch.py
```

联调检查建议：

```bash
ros2 topic hz /mavros/local_position/pose
ros2 topic echo fcu_link/status
ros2 topic echo uav_bridge/flight_state
ros2 topic echo target_point
ros2 topic echo target_point_safe
ros2 topic echo joint_angles
```

## 关键参数

- `delta_kinematics`：`L1`, `L2`, `R`, `r`
- `uav_bridge`：`serial_port`, `baudrate`, `target_frame`, `camera_frame`
- `mock_mavros_pose_node`：`publish_rate_hz`, `trajectory_mode`, `origin_*`, `amplitude_*`
- `fcu_link_monitor_node`：`timeout_sec`, `check_rate_hz`
- `failsafe_manager_node`：`safe_x`, `safe_y`, `safe_z`
- `perception_logic`：`image_topic`, `publish_topic`, `camera_frame`

参数模板位置：

- `delta_kinematics`：[src/delta_kinematics/config/delta_kinematics.yaml](src/delta_kinematics/config/delta_kinematics.yaml)
- `perception_logic`：[src/perception_logic/config/perception_logic.yaml](src/perception_logic/config/perception_logic.yaml)
- `uav_bridge`：[src/uav_bridge/config/uav_bridge.yaml](src/uav_bridge/config/uav_bridge.yaml)
- `mock_mavros_pose`：[src/uav_bridge/config/mock_mavros_pose.yaml](src/uav_bridge/config/mock_mavros_pose.yaml)
- `fcu_health`：[src/uav_bridge/config/fcu_health.yaml](src/uav_bridge/config/fcu_health.yaml)
- `failsafe`：[src/uav_bridge/config/failsafe.yaml](src/uav_bridge/config/failsafe.yaml)

## 约束落实

- C++ 节点使用 `rclcpp::Timer`，无 `while` 主循环。
- 高频回调避免大内存分配，复用消息缓存。
- 串口路径与波特率参数化，未硬编码。
- 使用 `RCLCPP_INFO` / `RCLCPP_DEBUG` 记录关键步骤。
