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

用途：上桨后首次自主短测，只做起飞、悬停、降落。

前置：bench 已 PASS，且已完成手动 `ALT_HOLD/LOITER` 短悬停。

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
```

## 5. 上桨完整任务测试

用途：上桨后完整任务，包含起飞、UWB 接近、下降、抓取占位、返航、投放占位、降落。

前置：简单起降已 PASS，再做低高度、短距离测试。

```bash
docker exec -it ros2humble bash -lc "
  source /opt/ros/humble/setup.bash
  source /workspace/uav_delta_capture/install/setup.bash
  ros2 launch uwb_navigation test_mission_real_full.launch.py
"
```

关键日志：

```text
Real preflight
Phase: ...
```

## 6. 日志查看

```bash
# MAVROS 日志
docker exec ros2humble bash -lc "tail -160 /tmp/mavros.log"

# bench 后台日志
docker exec ros2humble bash -lc "grep -E 'Bench preflight|BENCH RESULT|Core links|Sensor links|Bench warnings|Phase|ERROR|WARN' /tmp/mission_bench.log | tail -160"

# 简单起降后台日志
docker exec ros2humble bash -lc "grep -E 'Takeoff-land preflight|TAKEOFF_LAND RESULT|Core links|Sensor links|Phase|Takeoff|Land|FAILSAFE|ERROR|WARN' /tmp/mission_takeoff_land.log | tail -160"

# 完整任务后台日志
docker exec ros2humble bash -lc "grep -E 'Real preflight|Phase|UWB|local_pose|rangefinder|optical_flow|ARM|GUIDED|LAND|DONE|FAILSAFE|ERROR|WARN' /tmp/mission_real_full.log | tail -160"
```