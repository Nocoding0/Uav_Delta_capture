# test_mission_node — UWB 导航全流程测试

## 代码结构

```
src/uwb_navigation/
├── src/
│   ├── test_mission_node.py         ← 测试节点, 14 阶段状态机 + PID 控制器
│   └── uwb_mission_planner_node.cpp ← 正式任务规划器 (C++, PID)
├── config/
│   ├── test_mission.yaml            ← 测试节点参数
│   └── uwb_mission_planner.yaml     ← 正式任务规划器参数
├── launch/
│   └── test_mission.launch.py       ← 一键启动
└── README_TEST.md

src/fcu_bridge/
└── src/
    └── flight_state_machine_node.cpp ← 飞行状态机 (5 状态, 恢复窗口 + 锁存)
```

**核心类：** `TestMissionNode(Node)`

- `_control_loop()` — 20Hz 主循环, 按 Phase 枚举分发到对应 `_tick_*()` 函数
- `_call_flight_cmd()` — 异步调 `flight_command` 服务
- `_check_stable_and_transition()` — 稳定计时器, 到时自动切阶段
- `_tick_recovering()` — 链路恢复悬停 (3s 容忍期)
- 各 `_tick_*()` — 每阶段的控制逻辑

**14 阶段状态机：**

```
INIT → ARM → TAKEOFF → HOVER_TAKEOFF → MOVE_ABOVE → HOVER_ABOVE
  → DESCEND → HOVER_FINAL → CLIMB → RETURN → HOVER_RETURN → LAND → DONE
                                                          ↑
  任何飞行阶段 ──LOST──→ RECOVERING ──超时──→ FAILSAFE
                          │                      │
                        恢复 OK                  RESET
                          ↓                      ↓
                      继续飞行                WAIT_FCU
```

**导航策略：**

| 阶段 | 策略 |
|------|------|
| MOVE_ABOVE / DESCEND | UWB 反应式 PID: `azimuth→0` 控左右, `horizontal_dist→0` 控前后 |
| RETURN | FCU NED 坐标返航: `/mavros/local_position/pose` → 飞回起飞原点 |
| RECOVERING | 零速悬停等待链路恢复 |
| 其余 | PID 高度保持 / 零速悬停 |

**PID 控制器 (uwb_mission_planner_node.cpp)：**

```
error = target - actual
filtered_error = α × error + (1-α) × prev_filtered    (低通滤波)
integral += error × dt, clamp(±integral_limit)         (积分限幅)
derivative = (filtered_error - prev_filtered) / dt      (微分, 用滤波后误差)

output = kp × error + ki × integral + kd × derivative
```

- ki=0, kd=0 时退化为原 P 控制 (向后兼容)
- 阶段切换时自动清零 integral / filtered_error

**飞行状态机 (flight_state_machine_node.cpp)：**

```
INIT → WAIT_FCU → TRACKING ⇄ RECOVERING → FAILSAFE
```

| 状态 | 条件 | 说明 |
|------|------|------|
| TRACKING | 链路 OK | 正常工作 |
| RECOVERING | 链路 LOST | 3s 容忍期, 零速悬停 |
| FAILSAFE | 容忍期超时 | 锁存, 只能 RESET 解锁 |

**数据链路：**

```
输入:  /uwb_aoa/data /fcu_state /mavros/local_position/pose /fcu_link/status
输出:  /cmd_vel (TwistStamped) + test_mission/state + test_mission/event
服务:  flight_command (ARM / TAKEOFF / LAND)
重置:  /uav_bridge/flight_reset (String "RESET" → 解锁 FAILSAFE)
```

---

## 使用步骤

### 前置条件

- Docker 容器 `ros2humble` 已启动
- 已构建: `colcon build --packages-select uwb_navigation fcu_bridge`

### Mock 模式测试

用模拟飞控 + 计时器替代传感器检查，验证整个任务流程的链路连通性。

**Step 1: 启动 Mock FCU 桥接**

```bash
# 每个节点用 docker exec -d 启动 (独立后台运行)
docker exec -d ros2humble bash -c 'source /opt/ros/humble/setup.bash && source /workspace/uav_delta_capture/install/setup.bash && ros2 run fcu_bridge mock_mavros_pose_node --ros-args -p mock_altitude:=0.0'
docker exec -d ros2humble bash -c 'source /opt/ros/humble/setup.bash && source /workspace/uav_delta_capture/install/setup.bash && ros2 run fcu_bridge fcu_state_node --ros-args -p use_mock:=true -p mock_armed:=true -p mock_altitude:=0.0'
docker exec -d ros2humble bash -c 'source /opt/ros/humble/setup.bash && source /workspace/uav_delta_capture/install/setup.bash && ros2 run fcu_bridge flight_commander_node --ros-args -p use_mock:=true'
docker exec -d ros2humble bash -c 'source /opt/ros/humble/setup.bash && source /workspace/uav_delta_capture/install/setup.bash && ros2 run fcu_bridge flight_state_machine_node'
```

**Step 2: 确认服务就绪**

```bash
docker exec ros2humble bash -c "
  source /opt/ros/humble/setup.bash
  source /workspace/uav_delta_capture/install/setup.bash
  ros2 service list | grep flight_command
"
# 应输出: /flight_command
```

**Step 3: 启动测试节点**

```bash
docker exec -it ros2humble bash -c "
  source /opt/ros/humble/setup.bash
  source /workspace/uav_delta_capture/install/setup.bash
  python3 /workspace/uav_delta_capture/install/uwb_navigation/lib/uwb_navigation/test_mission_node.py \
    --ros-args --params-file /workspace/uav_delta_capture/install/uwb_navigation/share/uwb_navigation/test_mission.yaml
"
```

预期: 约 16 秒内打印 `MISSION COMPLETE`, 12 阶段全部通过。

### RECOVERING 测试

验证链路丢失后的恢复逻辑。

**Step 1: 按上面 Mock 模式启动节点 (但不要启动 fcu_link_monitor_node)**

```bash
# 如果已启动 fcu_link_monitor, 先杀掉
docker exec ros2humble pkill -f fcu_link_monitor
```

**Step 2: 运行恢复测试脚本**

```bash
docker exec ros2humble bash -c "
  source /opt/ros/humble/setup.bash
  source /workspace/uav_delta_capture/install/setup.bash
  python3 /tmp/recovery_test.py
"
```

预期输出:
- Test 1: LOST → RECOVERING → 2s 后 OK → TRACKING ✅
- Test 2: LOST → RECOVERING → 4s 超时 → FAILSAFE → RESET → WAIT_FCU ✅

### 真实 FCU 模式

**Step 1: 确认硬件连接**

```bash
ls /dev/ttyACM0        # FCU (CUAVv5-bdshot) — 应存在
ls /dev/ttySTM1        # UWB (CN5 串口) — 应存在
```

**Step 2: 改配置**

修改 `src/uwb_navigation/config/test_mission.yaml`:

```yaml
test_mission_node:
  ros__parameters:
    use_mock: false      # 切到真实模式
```

**Step 3: 启 MAVROS + FCU 桥接**

```bash
# 启动 MAVROS (连 /dev/ttyACM0)
ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM0:921600

# 启动 FCU 桥接 (非 mock)
ros2 run fcu_bridge fcu_state_node        # 默认 use_mock:=false
ros2 run fcu_bridge flight_commander_node  # 默认 use_mock:=false
ros2 run fcu_bridge fcu_link_monitor_node
ros2 run fcu_bridge flight_state_machine_node
```

**Step 4: 确认全部数据就绪**

```bash
ros2 topic echo /fcu_state --once       # 应看到 connected: true
ros2 topic echo /uwb_aoa/data --once    # 应看到 signal_valid: true
```

**Step 5: 启动测试节点**（同 Mock Step 3）

---

## 关键参数

全部在 `config/test_mission.yaml` 中修改，运行时通过 `--params-file` 加载。

### 飞行参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `use_mock` | false | Mock 模式开关 (true=跳过传感器检查, 计时器推进) |
| `takeoff_altitude` | 1.5 | 起飞/巡航高度 (m) |
| `descend_altitude` | 0.5 | 抓取下降高度 (m) |
| `max_vel_xy` | 0.5 | 最大水平速度 (m/s) |
| `max_vel_z` | 0.3 | 最大垂直速度 (m/s) |
| `azimuth_deadband` | 3.0 | 方位角死区 (°) |
| `horizontal_deadband` | 0.15 | 水平距死区 (m) |
| `altitude_tolerance` | 0.15 | 高度容差 (m) |
| `return_xy_tolerance` | 0.3 | 返航到达半径 (m) |
| `hover_stable_time` | 2.0 | 阶段切换稳定计时 (s) |
| `grasp_duration_sec` | 5.0 | 假抓取等待时长 (s) |
| `control_rate_hz` | 20 | 控制循环频率 (Hz) |

### PID 控制参数 (ki/kd=0 时退化为纯 P 控制)

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `kp_horizontal` | 0.4 | 水平比例增益 |
| `kp_vertical` | 0.3 | 垂直比例增益 |
| `kp_return` | 0.5 | 返航比例增益 |
| `ki_horizontal` | 0.0 | 水平积分增益 (0=禁用) |
| `ki_vertical` | 0.0 | 垂直积分增益 (0=禁用) |
| `kd_horizontal` | 0.0 | 水平微分增益 (0=禁用, 预留) |
| `kd_vertical` | 0.0 | 垂直微分增益 (0=禁用, 预留) |
| `integral_limit` | 1.0 | 积分限幅 (m·s) |
| `lp_filter_alpha` | 0.3 | 低通滤波系数 (0.01~0.99, 越小越平滑) |

### 安全参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `uwb_signal_timeout` | 3.0 | UWB 信号丢失超时 (s) |
| `low_battery_pct` | 20 | 低电量触发阈值 (%) |
| `recovery_timeout` | 3.0 | 链路丢失恢复容忍期 (s) |

---

## 异常处理

| 级别 | 条件 | 动作 | Mock 模式 |
|------|------|------|-----------|
| 🟡 链路丢失 | `fcu_link/status == "LOST"` | 进入 RECOVERING, 零速悬停 3s | 不触发 |
| 🟡 链路恢复 | RECOVERING 期间收到 OK | 回到飞行阶段继续 | 不触发 |
| 🔴 恢复超时 | RECOVERING 超过 3s | 转 FAILSAFE → LAND | 不触发 |
| 🔴 FCU 断连 | `fcu_state.connected == false` | 立即 FAILSAFE → LAND | 不触发 |
| 🔴 低电量 | `remaining < low_battery_pct%` | 立即 FAILSAFE → LAND | 不触发 |
| 🔴 起飞失败 | `flight_command` 返回失败 | 转 FAILSAFE → LAND | 同左 |
| 🔴 ARM 超限 | ARM 重试超过 20 次 | 转 FAILSAFE → LAND | 同左 |
| 🟡 UWB 丢失 | `signal_valid==false` 或超时 >3s | 零速悬停等待 | 不触发 |

**FAILSAFE 锁存**: 一旦进入 FAILSAFE, 只能通过 `/uav_bridge/flight_reset` 发送 `"RESET"` 消息解锁。

---

## 数据监控

```bash
# 阶段状态
ros2 topic echo /test_mission/state

# 事件流
ros2 topic echo /test_mission/event

# 飞行状态机状态
ros2 topic echo /uav_bridge/flight_state

# 速度指令
ros2 topic echo /cmd_vel

# FCU 状态
ros2 topic echo /fcu_state

# UWB 数据
ros2 topic echo /uwb_aoa/data

# 解锁 FAILSAFE
ros2 topic pub --once /uav_bridge/flight_reset std_msgs/msg/String "{data: RESET}"
```

---

## 已验证

| 日期 | 模式 | 测试内容 | 结果 |
|------|------|----------|------|
| 2026-06-06 | Mock | 12 阶段全流程 | 全部通过 ✅ |
| 2026-06-07 | Mock | PID 升级后全流程 (ki=kd=0) | 全部通过 ✅ |
| 2026-06-07 | Mock | RECOVERING 恢复 (2s 内 OK) | TRACKING → RECOVERING → TRACKING ✅ |
| 2026-06-07 | Mock | RECOVERING 超时 → FAILSAFE | RECOVERING → FAILSAFE ✅ |
| 2026-06-07 | Mock | FAILSAFE 解锁 (RESET) | FAILSAFE → WAIT_FCU ✅ |

---

## 待测试

| 模式 | 内容 | 说明 |
|------|------|------|
| 真实 FCU | PID 参数调试 | 建议先设 ki=0.05 小量启用积分项 |
| 真实 FCU | MAVROS 连通 | /dev/ttyACM0, 921600 baud |
| 真实 FCU | 短距离起飞悬停 | 验证 PID 实际效果 |
