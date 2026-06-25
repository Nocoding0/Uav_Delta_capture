# uwb_navigation 常用命令

## 0. Start Docker/MAVROS

Use this first. It starts/fixes Docker, starts the ros2humble container, starts MAVROS, requests local_position, then runs read-only checks.

```bash
cd /usr/local/Uav_Delta_capture
./start_ready.sh
```

Success marker:

```text
READY: docker, container, MAVROS checked. Log: /tmp/mlog
```

If it fails, check logs:

```bash
tail -80 /tmp/dockerd.manual.log
docker exec ros2humble tail -100 /tmp/mlog
```

Manual read-only checks:

```bash
# FCU state
docker exec ros2humble bash -lc "source /opt/ros/humble/setup.bash && ros2 topic echo /mavros/state --once"

# local_position rate
docker exec ros2humble bash -lc "source /opt/ros/humble/setup.bash && timeout 10 ros2 topic hz /mavros/local_position/pose"

# rangefinder
docker exec ros2humble bash -lc "source /opt/ros/humble/setup.bash && timeout 5 ros2 topic echo /mavros/rangefinder_pub sensor_msgs/msg/Range --qos-profile sensor_data --once"

# optical flow
docker exec ros2humble bash -lc "source /opt/ros/humble/setup.bash && timeout 10 ros2 topic hz /mavros/optical_flow/raw/optical_flow"
```

## 1. 一键预检

用途：只读检查，不 ARM，不起飞。

```bash
# 只连 UWB、没连 FCU 时使用。
docker exec ros2humble bash -lc "/workspace/uav_delta_capture/scripts/preflight_check.sh uwb_only"

# FCU/MAVROS/UWB 都连接后使用。
docker exec ros2humble bash -lc "/workspace/uav_delta_capture/scripts/preflight_check.sh full"
```

通过标志：

```text
RESULT: READY
```

## 2. 纯 mock 测试

用途：不连真实 FCU，不上桨，只测 ROS 状态机。

```bash
docker exec -it ros2humble bash -lc "
  source /opt/ros/humble/setup.bash
  source /workspace/uav_delta_capture/install/setup.bash
  ros2 launch uwb_navigation test_mission.launch.py
"
```

## 3. 桌面 bench 测试

用途：去桨，连真实 FCU，测试 ARM、速度指令、DISARM 和传感器链路。

```bash
docker exec -it ros2humble bash -lc "
  source /opt/ros/humble/setup.bash
  source /workspace/uav_delta_capture/install/setup.bash
  ros2 launch uwb_navigation test_mission_bench.launch.py
"
```

通过标志：

```text
BENCH RESULT: PASS
```

## 4. 上桨简单起降测试

用途：上桨后首次自主短测，只做起飞、悬停、降落。当前版本使用 MAVROS takeoff 起飞；为避免飞控拒绝低目标高度，takeoff 服务目标约为 `current_local_z + 0.2m`，任务达高仍按 rangefinder 相对高度 `0.6m` 判断，并在降落后等待落地/解锁确认。MAVROS takeoff 尚未达高时，任务节点只观察高度，不向 `/mavros/setpoint_velocity/cmd_vel` 连续发送零速度。

前置：bench 已 PASS，且已完成手动 `ALT_HOLD/LOITER` 短悬停。

遥控接管：

- 空中异常：模式开关最终切到 `ALT_HOLD` 接管；如果开关已经在 `ALT_HOLD` 位，先拨到 `LOITER` 再拨回 `ALT_HOLD`。
- 只拨右手摇杆不等于取消自主任务。
- 已经落地且桨还在转：油门最低，ARM 开关先拨到解锁位，再拨回未解锁位停桨。

```bash
docker exec -it ros2humble bash -lc "
  source /opt/ros/humble/setup.bash
  source /workspace/uav_delta_capture/install/setup.bash
  ros2 launch uwb_navigation test_mission_takeoff_land.launch.py
"
```

通过标志：

```text
TAKEOFF_LAND RESULT: PASS
service_target
Landing complete
```

### 4.1 上桨 LOITER 悬停对比测试

用途：在简单起降基础上，对比飞控 `LOITER` 定点悬停效果。流程是 `GUIDED` takeoff 起飞，到高度后先在 `GUIDED` 低高度稳定约 1.5 秒，再自动切 `LOITER` 悬停 5 秒，最后切回 `GUIDED` 并执行 `LAND`。如果 `LOITER` 悬停期间高度掉到近地阈值以下，会判失败并进入安全处理，不再继续计时判 PASS。

```bash
docker exec -it ros2humble bash -lc "
  source /opt/ros/humble/setup.bash
  source /workspace/uav_delta_capture/install/setup.bash
  ros2 launch uwb_navigation test_mission_takeoff_loiter_land.launch.py
"
```

关键通过标志：

```text
Takeoff OK
service_target
FCU mode confirmed: LOITER
FCU mode confirmed: GUIDED
Landing complete
TAKEOFF_LOITER_LAND RESULT: PASS
```

## 5. 上桨 UWB 接近降落精简测试

用途：在简单 GUIDED 起降通过后，先验证“起飞、UWB 接近 tag 正上方、悬停、原地降落”。这个模式不做抓取、复飞、返航、投放，是完整任务前的上桨精简版本。

前置：简单起降已 PASS，UWB tag 放在地上，场地只做低高度、短距离测试。不要使用 LOITER 版本作为这个测试的前置。

```bash
docker exec -it ros2humble bash -lc "
  source /opt/ros/humble/setup.bash
  source /workspace/uav_delta_capture/install/setup.bash
  ros2 launch uwb_navigation test_mission_uwb_approach_land.launch.py
"
```

关键日志：

```text
UWB approach-land preflight
Takeoff OK
Phase: HOVER_TAKEOFF -> MOVE_ABOVE
UWB approach:
Above target
Phase: HOVER_ABOVE -> LAND
UWB_APPROACH_LAND RESULT: PASS
```

通过后再考虑恢复 `test_mission_real_full.launch.py` 的完整抓取、返航、投放流程。

## 6. 日志查看

```bash
# MAVROS 日志
docker exec ros2humble bash -lc "tail -160 /tmp/mavros.log"

# bench 后台日志
docker exec ros2humble bash -lc "grep -E 'Bench preflight|BENCH RESULT|Core links|Sensor links|Bench warnings|Phase|ERROR|WARN' /tmp/mission_bench.log | tail -160"

# 简单起降后台日志
docker exec ros2humble bash -lc "grep -E 'Takeoff-land preflight|TAKEOFF_LAND RESULT|Core links|Sensor links|Phase|Takeoff OK|Landing complete|LAND_WAIT|Takeoff|Land|FAILSAFE|ERROR|WARN' /tmp/mission_takeoff_land.log | tail -160"

# UWB 接近降落精简任务后台日志
docker exec ros2humble bash -lc "grep -E 'UWB approach-land preflight|UWB_APPROACH_LAND RESULT|Core links|Sensor links|Phase|Takeoff OK|UWB approach|Above target|Landing complete|LAND_WAIT|FAILSAFE|ERROR|WARN' /tmp/mission_uwb_approach_land.log | tail -180"
```
