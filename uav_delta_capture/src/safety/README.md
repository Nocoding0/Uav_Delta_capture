# safety

Safety utilities package.

## Current Role

The active UWB mission path currently handles critical checks in:

- `uwb_navigation/test_mission_node.py`
- `fcu_bridge/flight_commander_node.cpp`

`failsafe_manager_node` remains available for future target-point safety gating, but it is not the primary controller for the current UWB mission launch files.
