# uwb_navigation - UWB 导航控制包

基于 UWB AOA 数据的无人机自主导航控制。

## 功能

- UWB 数据驱动的 3D 坐标计算
- 起飞、悬停、水平移动、降落状态机
- PID 速度控制
- 超时与故障保护

## 节点

### uwb_mission_planner_node

导航任务规划主节点。

**订阅话题：**
- `uwb_aoa/data`（`UavDeltaMsgs/UwbAoa`）- UWB 数据
- `mavros/state`（`mavros_msgs/State`）- 飞控状态
- `mavros/local_position/pose`（`geometry_msgs/PoseStamped`）- 当前位置

**发布话题：**
- `mavros/setpoint_velocity/cmd_vel_unstamped`（`geometry_msgs/Twist`）- 速度指令
- `uwb_mission/state`（`std_msgs/String`）- 当前状态
- `uwb_mission/event`（`std_msgs/String`）- 事件通知

**状态机：**
```
IDLE → ARMING → TAKEOFF → HOVER_TAKEOFF → MOVE_ABOVE → HOVER_ABOVE → DESCEND → HOVER_FINAL → DONE
                                                                                    ↓
                                                                              FAILSAFE
```

**参数：**
- `takeoff_altitude`（double）- 起飞高度，默认 `1.5` m
- `descend_altitude`（double）- 降落高度，默认 `0.5` m
- `kp_horizontal`（double）- 水平 PID 系数，默认 `0.4`
- `kp_vertical`（double）- 垂直 PID 系数，默认 `0.3`
- `max_vel_xy`（double）- 最大水平速度，默认 `0.5` m/s
- `max_vel_z`（double）- 最大垂直速度，默认 `0.3` m/s
- `horizontal_deadband`（double）- 水平死区，默认 `0.15` m
- `hover_stable_time`（double）- 悬停稳定时间，默认 `2.0` s

## 使用

```bash
# 单独启动
ros2 launch uwb_navigation uwb_mission_planner.launch.py

# 启动完整导航系统
ros2 launch uwb_navigation uwb_navigation_system.launch.py
```

## 配置

配置文件：`config/uwb_mission_planner.yaml`

## 飞行路径

1. **起飞** - 解锁并爬升到 `takeoff_altitude`
2. **悬停** - 在起飞点悬停 `hover_stable_time` 秒
3. **水平移动** - 飞到目标正上方
4. **悬停** - 在目标上方悬停稳定
5. **降落** - 下降到 `descend_altitude`
6. **最终悬停** - 稳定后交给机械臂
