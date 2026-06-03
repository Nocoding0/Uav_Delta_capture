# fcu_bridge - 飞控通信桥接包

MAVROS 飞控通信桥接与状态发布。

## 功能

- MAVROS 连接状态监控
- 飞行模式与解锁状态发布
- 姿态角提取与发布
- 飞行指令发送
- MAVROS 连接健康检查

## 节点

### fcu_state_node

飞控状态发布节点。

**订阅：**
- `mavros/state`（`mavros_msgs/State`）
- `mavros/local_position/pose`（`geometry_msgs/PoseStamped`）

**发布：**
- `fcu/state`（`mavros_msgs/State`）- 飞控状态
- `fcu/armed`（`std_msgs/Bool`）- 解锁状态
- `fcu/flight_mode`（`std_msgs/String`）- 飞行模式

### attitude_publisher_node

姿态角发布节点。

**订阅：**
- `mavros/local_position/pose`（`geometry_msgs/PoseStamped`）
- `mavros/imu/data`（`sensor_msgs/Imu`）

**发布：**
- `fcu/local_attitude`（`geometry_msgs/Vector3Stamped`）- roll/pitch/yaw
- `fcu/imu_attitude`（`geometry_msgs/Vector3Stamped`）- IMU 姿态

**参数：**
- `pose_topic`（string）- 位姿话题，默认 `mavros/local_position/pose`
- `imu_topic`（string）- IMU 话题，默认 `mavros/imu/data`
- `local_attitude_topic`（string）- 发布话题，默认 `fcu/local_attitude`
- `imu_attitude_topic`（string）- IMU 话题，默认 `fcu/imu_attitude`

### flight_commander_node

飞行指令发送节点。

**服务：**
- `fcu/arm`（`std_srvs/Trigger`）- 解锁
- `fcu/takeoff`（`std_srvs/Trigger`）- 起飞
- `fcu/land`（`std_srvs/Trigger`）- 降落
- `fcu/set_mode`（`mavros_msgs/SetMode`）- 设置模式

### fcu_link_monitor_node

MAVROS 连接监控节点。

**发布：**
- `fcu/link_status`（`std_msgs/Bool`）- 连接状态
- `fcu/heartbeat`（`std_msgs/Header`）- 心跳

### mock_mavros_pose_node

模拟 MAVROS 位姿（测试用）。

**发布：**
- `mavros/local_position/pose`（`geometry_msgs/PoseStamped`）
- `mavros/imu/data`（`sensor_msgs/Imu`）

## 使用

```bash
# 启动姿态发布
ros2 launch fcu_bridge attitude_publisher.launch.py

# 测试模式（无需飞控）
ros2 run fcu_bridge mock_mavros_pose_node
```

## 配置

配置文件：`config/attitude_publisher.yaml`
