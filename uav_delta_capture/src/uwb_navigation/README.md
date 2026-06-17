# uwb_navigation

UWB navigation and mission sequencing for the UAV capture flow.

## Modes

- `mock_full`: pure software flow test.
- `bench_velocity`: real FCU/UWB/local-pose preflight, ARM, short vertical velocity profile, DISARM.
- `real_full`: real mission flow with fake or topic-driven grasp/drop completion.

## Launch

```bash
# Pure mock full flow
ros2 launch uwb_navigation test_mission.launch.py

# Bench test. Start MAVROS first. No propellers.
ros2 launch uwb_navigation test_mission_bench.launch.py

# Backward-compatible bench entry
ros2 launch uwb_navigation test_mission_real.launch.py

# Real full mission. Start MAVROS first.
ros2 launch uwb_navigation test_mission_real_full.launch.py
```

## Mission Flow

```text
INIT -> ARM -> TAKEOFF -> HOVER_TAKEOFF -> MOVE_ABOVE -> HOVER_ABOVE
-> DESCEND -> HOVER_FINAL -> WAIT_GRASP -> CLIMB -> HOVER_CLIMB
-> RETURN -> HOVER_RETURN -> WAIT_DROP -> LAND -> DONE
```

`BENCH_VELOCITY` is used only by `bench_velocity`.

## Navigation Policy

- UWB guides the aircraft from takeoff hover to above the tag.
- Return uses `/mavros/local_position/pose` and the recorded takeoff origin.
- The node does not run SLAM and does not read UTF01 directly. UTF01/optical flow must be fused by the FCU/MAVROS local position estimate.
- RC intervention is handled by mode takeover: if FCU mode leaves `auto_modes`, the mission publishes zero velocity and enters `PAUSED_MANUAL`.

## Placeholder Interfaces

- `grasp_done` (`std_msgs/String`): values `true`, `ok`, `done`, `complete`, `success`, or `1` complete grasp.
- `drop_done` (`std_msgs/String`): same convention for drop completion.
- `fake_grasp` / `fake_drop` can keep these stages timer-driven until the real modules are ready.

## Key Parameters

- `mission_mode`
- `use_mock`
- `desktop_test`
- `require_uwb_ready`
- `require_local_pose_ready`
- `takeoff_altitude`
- `descend_altitude`
- `max_vel_xy`
- `max_vel_z`
- `velocity_slew_rate`
- `auto_modes`
