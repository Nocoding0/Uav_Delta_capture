# safety - 安全管理包

系统安全监控与故障处理。

## 功能

- 飞控状态监控
- 电池电量监测
- UWB 数据有效性检查
- 故障自动处理（悬停/降落）
- 紧急停止

## 节点

### failsafe_manager_node

故障安全管理节点。

**订阅：**
- `mavros/state`（`mavros_msgs/State`）- 飞控状态
- `mavros/battery`（`sensor_msgs/BatteryState`）- 电池状态
- `uwb_mission/state`（`std_msgs/String`）- 任务状态
- `fcu/link_status`（`std_msgs/Bool`）- 连接状态

**发布：**
- `mavros/setpoint_velocity/cmd_vel_unstamped`（`geometry_msgs/Twist`）- 速度指令
- `failsafe/status`（`std_msgs/String`）- 故障状态
- `failsafe/triggered`（`std_msgs/Bool`）- 故障触发标志

**检查项目：**
- 飞控连接状态
- 电池电压/电量
- UWB 数据超时
- 任务状态异常

**响应动作：**
- 警告 - 发布警告信息
- 悬停 - 保持当前位置
- 降落 - 自动降落
- 紧急停止 - 立即停止电机

## 使用

```bash
ros2 launch safety failsafe_manager.launch.py
```

## 参数

- `battery_warn_voltage`（double）- 电池警告电压，默认 `11.0` V
- `battery_critical_voltage`（double）- 电池临界电压，默认 `10.5` V
- `uwb_timeout`（double）- UWB 数据超时，默认 `1.0` s
- `fcu_timeout`（double）- 飞控连接超时，默认 `2.0` s
