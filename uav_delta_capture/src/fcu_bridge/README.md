# fcu_bridge

Bridge package between project nodes and MAVROS.

## Nodes

- `fcu_state_node`: publishes `fcu_state` from MAVROS state, battery, estimator, and local pose.
- `flight_commander_node`: exposes `flight_command` and forwards project `cmd_vel` setpoints to MAVROS.
- `fcu_link_monitor_node`: monitors `/mavros/local_position/pose` freshness and publishes `fcu_link/status`.
- `mock_mavros_pose_node`: publishes synthetic local pose for mock tests.

## Velocity Safety

`flight_commander_node` clamps velocity setpoints and publishes a zero setpoint if the latest project `cmd_vel` is older than `vel_timeout_sec`.

Default topics:

- input: `cmd_vel`
- output: `/mavros/setpoint_velocity/cmd_vel`

## Manual Takeover

Manual intervention is expected to happen at the FCU mode/RC layer. Mission code should stop publishing motion commands when FCU mode leaves the configured autonomous mode list.
