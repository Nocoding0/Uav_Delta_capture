#!/usr/bin/env python3
"""UWB mission node.

Mission modes:
  - mock_full: pure software flow test.
  - bench_velocity: real hardware preflight + ARM + short Z velocity profile + DISARM.
  - real_full: take off, UWB approach, descend, fake/real grasp, climb, return, drop, land.
"""

import math
import threading
from enum import Enum

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.srv import SetMode
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from uav_delta_msgs.msg import FcuState, UwbAoa
from uav_delta_msgs.srv import FlightCommand


SENSOR_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


class Phase(Enum):
    INIT = 0
    ARM = 1
    BENCH_VELOCITY = 2
    TAKEOFF = 3
    HOVER_TAKEOFF = 4
    MOVE_ABOVE = 5
    HOVER_ABOVE = 6
    DESCEND = 7
    HOVER_FINAL = 8
    WAIT_GRASP = 9
    CLIMB = 10
    HOVER_CLIMB = 11
    RETURN = 12
    HOVER_RETURN = 13
    WAIT_DROP = 14
    LAND = 15
    DONE = 16
    PAUSED_MANUAL = 17
    RECOVERING = 18
    FAILSAFE = 19


PHASE_NAMES = {phase: phase.name for phase in Phase}


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def string_is_done(value: str) -> bool:
    return value.strip().upper() in {"1", "TRUE", "OK", "DONE", "COMPLETE", "SUCCESS"}


class TestMissionNode(Node):
    def __init__(self):
        super().__init__("test_mission_node")

        self.uwb_aoa_topic = self.declare_parameter("uwb_aoa_topic", "uwb_aoa/data").value
        self.fcu_state_topic = self.declare_parameter("fcu_state_topic", "fcu_state").value
        self.local_pose_topic = self.declare_parameter(
            "local_pose_topic", "/mavros/local_position/pose"
        ).value
        self.fcu_link_topic = self.declare_parameter("fcu_link_topic", "fcu_link/status").value
        self.cmd_vel_topic = self.declare_parameter("cmd_vel_topic", "cmd_vel").value
        self.mission_state_topic = self.declare_parameter(
            "mission_state_topic", "test_mission/state"
        ).value
        self.mission_event_topic = self.declare_parameter(
            "mission_event_topic", "test_mission/event"
        ).value
        self.flight_command_service = self.declare_parameter(
            "flight_command_service", "flight_command"
        ).value

        self.mission_mode = self.declare_parameter("mission_mode", "").value
        self.use_mock = self.declare_parameter("use_mock", False).value
        self.desktop_test = self.declare_parameter("desktop_test", False).value
        if not self.mission_mode:
            if self.use_mock:
                self.mission_mode = "mock_full"
            elif self.desktop_test:
                self.mission_mode = "bench_velocity"
            else:
                self.mission_mode = "real_full"

        self.fake_grasp = self.declare_parameter("fake_grasp", True).value
        self.fake_grasp_delay_sec = self.declare_parameter("fake_grasp_delay_sec", 10.0).value
        self.fake_drop = self.declare_parameter("fake_drop", True).value
        self.fake_drop_delay_sec = self.declare_parameter("fake_drop_delay_sec", 2.0).value
        self.grasp_done_topic = self.declare_parameter("grasp_done_topic", "grasp_done").value
        self.drop_done_topic = self.declare_parameter("drop_done_topic", "drop_done").value

        self.takeoff_altitude = self.declare_parameter("takeoff_altitude", 1.5).value
        self.descend_altitude = self.declare_parameter("descend_altitude", 0.5).value

        self.kp_horizontal = self.declare_parameter("kp_horizontal", 0.4).value
        self.kp_vertical = self.declare_parameter("kp_vertical", 0.3).value
        self.kp_return = self.declare_parameter("kp_return", 0.5).value
        self.max_vel_xy = self.declare_parameter("max_vel_xy", 0.5).value
        self.max_vel_z = self.declare_parameter("max_vel_z", 0.3).value
        self.velocity_slew_rate = self.declare_parameter("velocity_slew_rate", 0.4).value

        self.azimuth_deadband = self.declare_parameter("azimuth_deadband", 3.0).value
        self.horizontal_deadband = self.declare_parameter("horizontal_deadband", 0.15).value
        self.altitude_tolerance = self.declare_parameter("altitude_tolerance", 0.15).value
        self.return_xy_tolerance = self.declare_parameter("return_xy_tolerance", 0.3).value

        self.hover_stable_time = self.declare_parameter("hover_stable_time", 2.0).value
        self.control_rate_hz = self.declare_parameter("control_rate_hz", 20.0).value

        self.require_uwb_ready = self.declare_parameter("require_uwb_ready", True).value
        self.require_local_pose_ready = self.declare_parameter("require_local_pose_ready", True).value
        self.uwb_signal_timeout = self.declare_parameter("uwb_signal_timeout", 3.0).value
        self.local_pose_timeout = self.declare_parameter("local_pose_timeout", 1.0).value
        self.low_battery_pct = self.declare_parameter("low_battery_pct", 20.0).value
        self.recovery_timeout = self.declare_parameter("recovery_timeout", 3.0).value
        self.auto_modes = self._parse_modes(
            self.declare_parameter("auto_modes", "GUIDED").value
        )
        self.arm_mode = self.declare_parameter("arm_mode", "ALT_HOLD").value

        self.bench_velocity_z = self.declare_parameter("bench_velocity_z", 0.15).value
        self.bench_climb_sec = self.declare_parameter("bench_climb_sec", 2.0).value
        self.bench_hold_sec = self.declare_parameter("bench_hold_sec", 2.0).value
        self.bench_descend_sec = self.declare_parameter("bench_descend_sec", 2.0).value
        self.bench_zero_sec = self.declare_parameter("bench_zero_sec", 1.0).value

        self.takeoff_altitude = max(0.5, self.takeoff_altitude)
        self.descend_altitude = max(0.2, self.descend_altitude)
        self.kp_horizontal = max(0.01, self.kp_horizontal)
        self.kp_vertical = max(0.01, self.kp_vertical)
        self.kp_return = max(0.01, self.kp_return)
        self.max_vel_xy = max(0.1, self.max_vel_xy)
        self.max_vel_z = max(0.05, self.max_vel_z)
        self.velocity_slew_rate = max(0.01, self.velocity_slew_rate)
        self.bench_velocity_z = clamp(abs(self.bench_velocity_z), 0.02, self.max_vel_z)

        self.phase = Phase.INIT
        self.previous_flight_phase = Phase.INIT
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.origin_recorded = False

        self.hover_start_time = None
        self.grasp_start_time = None
        self.drop_start_time = None
        self.bench_start_time = None
        self._pending_command = False
        self._command_retry_count = 0
        self._arm_mode_set = False
        self._desktop_disarm_done = False
        self._last_velocity = (0.0, 0.0, 0.0)
        self._last_velocity_time = self.get_clock().now()
        self._recovery_start_time = None

        self._data_lock = threading.Lock()
        self._last_uwb = None
        self._last_fcu_state = None
        self._last_local_pose = None
        self._last_link_status = "OK"
        self._has_uwb = False
        self._has_fcu = False
        self._has_pose = False
        self._last_uwb_time = self.get_clock().now()
        self._last_pose_time = self.get_clock().now()
        self._grasp_done = False
        self._drop_done = False

        cb_group = ReentrantCallbackGroup()

        self.uwb_sub = self.create_subscription(UwbAoa, self.uwb_aoa_topic, self._uwb_callback, 10)
        self.fcu_sub = self.create_subscription(FcuState, self.fcu_state_topic, self._fcu_callback, 10)
        self.pose_sub = self.create_subscription(
            PoseStamped, self.local_pose_topic, self._pose_callback, SENSOR_QOS
        )
        self.link_sub = self.create_subscription(String, self.fcu_link_topic, self._link_callback, 10)
        self.grasp_sub = self.create_subscription(String, self.grasp_done_topic, self._grasp_callback, 10)
        self.drop_sub = self.create_subscription(String, self.drop_done_topic, self._drop_callback, 10)

        self.vel_pub = self.create_publisher(TwistStamped, self.cmd_vel_topic, 20)
        self.state_pub = self.create_publisher(String, self.mission_state_topic, 10)
        self.event_pub = self.create_publisher(String, self.mission_event_topic, 10)
        self.flight_reset_pub = self.create_publisher(String, "uav_bridge/flight_reset", 10)

        self.flight_cmd_client = self.create_client(
            FlightCommand, self.flight_command_service, callback_group=cb_group
        )
        self.mavros_set_mode_client = self.create_client(
            SetMode, "/mavros/set_mode", callback_group=cb_group
        )

        ctrl_period = 1.0 / max(1.0, self.control_rate_hz)
        self.ctrl_timer = self.create_timer(ctrl_period, self._control_loop)

        self.get_logger().info(
            f"test_mission_node started: mode={self.mission_mode} "
            f"mock={str(self.use_mock).lower()} "
            f"takeoff={self.takeoff_altitude:.1f}m descend={self.descend_altitude:.1f}m"
        )

    def _parse_modes(self, modes_text):
        return {mode.strip().upper() for mode in modes_text.split(",") if mode.strip()}

    def _uwb_callback(self, msg: UwbAoa):
        with self._data_lock:
            self._last_uwb = msg
            self._has_uwb = True
            self._last_uwb_time = self.get_clock().now()

    def _fcu_callback(self, msg: FcuState):
        with self._data_lock:
            self._last_fcu_state = msg
            self._has_fcu = True

    def _pose_callback(self, msg: PoseStamped):
        with self._data_lock:
            self._last_local_pose = msg
            self._has_pose = True
            self._last_pose_time = self.get_clock().now()

    def _link_callback(self, msg: String):
        with self._data_lock:
            self._last_link_status = msg.data

    def _grasp_callback(self, msg: String):
        if string_is_done(msg.data):
            self._grasp_done = True

    def _drop_callback(self, msg: String):
        if string_is_done(msg.data):
            self._drop_done = True

    def _control_loop(self):
        if self._check_critical():
            self._publish_state()
            return

        tick_map = {
            Phase.INIT: self._tick_init,
            Phase.ARM: self._tick_arm,
            Phase.BENCH_VELOCITY: self._tick_bench_velocity,
            Phase.TAKEOFF: self._tick_takeoff,
            Phase.HOVER_TAKEOFF: self._tick_hover_takeoff,
            Phase.MOVE_ABOVE: self._tick_move_above,
            Phase.HOVER_ABOVE: self._tick_hover_above,
            Phase.DESCEND: self._tick_descend,
            Phase.HOVER_FINAL: self._tick_hover_final,
            Phase.WAIT_GRASP: self._tick_wait_grasp,
            Phase.CLIMB: self._tick_climb,
            Phase.HOVER_CLIMB: self._tick_hover_climb,
            Phase.RETURN: self._tick_return,
            Phase.HOVER_RETURN: self._tick_hover_return,
            Phase.WAIT_DROP: self._tick_wait_drop,
            Phase.LAND: self._tick_land,
            Phase.DONE: self._tick_done,
            Phase.PAUSED_MANUAL: self._tick_paused_manual,
            Phase.RECOVERING: self._tick_recovering,
            Phase.FAILSAFE: self._tick_failsafe,
        }
        tick_map[self.phase]()
        self._publish_state()

    def _check_critical(self) -> bool:
        if self.phase in (Phase.INIT, Phase.ARM, Phase.DONE, Phase.LAND, Phase.FAILSAFE):
            return False

        link = self._last_link_status
        if link == "LOST":
            if self.phase != Phase.RECOVERING:
                self._publish_event("link_recovering")
                self._transition(Phase.RECOVERING)
                return True
            return False

        if self.use_mock:
            return False

        fcu = self._last_fcu_state
        if not fcu or not fcu.connected:
            self._publish_event("failsafe_fcu_disconnected")
            self._transition(Phase.FAILSAFE)
            return True

        if fcu.voltage > 0.1 and fcu.remaining < self.low_battery_pct / 100.0:
            self._publish_event("failsafe_low_battery")
            self._transition(Phase.FAILSAFE)
            return True

        if self.phase not in (Phase.PAUSED_MANUAL, Phase.BENCH_VELOCITY):
            mode = (fcu.mode or "").upper()
            if mode and mode not in self.auto_modes:
                self.previous_flight_phase = self.phase
                self._publish_event(f"manual_takeover:{mode}")
                self._transition(Phase.PAUSED_MANUAL)
                return True

        return False

    def _uwb_valid_and_fresh(self) -> bool:
        if self.use_mock:
            return True
        with self._data_lock:
            if not self._has_uwb or self._last_uwb is None:
                return False
            if not self._last_uwb.signal_valid:
                return False
            age = (self.get_clock().now() - self._last_uwb_time).nanoseconds / 1e9
            return age <= self.uwb_signal_timeout

    def _local_pose_ready(self) -> bool:
        if self.use_mock:
            return True
        with self._data_lock:
            if not self._has_pose or self._last_local_pose is None:
                return False
            age = (self.get_clock().now() - self._last_pose_time).nanoseconds / 1e9
            return age <= self.local_pose_timeout

    def _fcu_connected(self) -> bool:
        if self.use_mock:
            return True
        with self._data_lock:
            return self._has_fcu and self._last_fcu_state and self._last_fcu_state.connected

    def _get_uwb(self):
        with self._data_lock:
            uwb = self._last_uwb
            if uwb is None:
                return None
            return (uwb.azimuth_deg, uwb.distance_m, uwb.elevation_deg)

    def _get_fcu_altitude(self) -> float:
        with self._data_lock:
            if self._last_fcu_state is None:
                return 0.0
            return abs(self._last_fcu_state.local_z)

    def _get_local_xy(self):
        with self._data_lock:
            if self._last_local_pose is None:
                return None
            p = self._last_local_pose.pose.position
            return (p.x, p.y)

    def _get_fcu_mode(self) -> str:
        with self._data_lock:
            if self._last_fcu_state is None:
                return ""
            return self._last_fcu_state.mode or ""

    def _tick_init(self):
        fcu_ok = self._fcu_connected()
        uwb_ok = (not self.require_uwb_ready) or self._uwb_valid_and_fresh()
        pose_ok = (not self.require_local_pose_ready) or self._local_pose_ready()

        if fcu_ok and uwb_ok and pose_ok:
            self._record_origin()
            self.get_logger().info("Preflight links ready, starting mission")
            self._transition(Phase.ARM)
            return

        if not fcu_ok:
            self.get_logger().warn("Waiting for FCU...", throttle_duration_sec=5.0)
        if not uwb_ok:
            self.get_logger().warn("Waiting for UWB...", throttle_duration_sec=5.0)
        if not pose_ok:
            self.get_logger().warn("Waiting for local pose...", throttle_duration_sec=5.0)

    def _tick_arm(self):
        if self._pending_command:
            return

        if not self._arm_mode_set:
            cur_mode = self._get_fcu_mode()
            if cur_mode == "LAND":
                self.get_logger().info(f"FCU in LAND, switching to {self.arm_mode} before ARM")
                self._call_mavros_set_mode(self.arm_mode)
                self._arm_mode_set = True
                return
            self._arm_mode_set = True

        if self._command_retry_count > 20:
            self.get_logger().error("ARM retry limit exceeded")
            self._transition(Phase.FAILSAFE)
            return

        self._pending_command = True
        self._call_flight_cmd(FlightCommand.Request.CMD_ARM, 0.0, self._on_arm_result)

    def _on_arm_result(self, success: bool, msg: str):
        self._pending_command = False
        if not success:
            self._command_retry_count += 1
            self.get_logger().warn(f"Arm failed ({self._command_retry_count}): {msg}")
            return

        self.get_logger().info(f"Arm OK: {msg}")
        self._publish_event("armed")
        self._command_retry_count = 0

        if self.mission_mode == "bench_velocity":
            self._call_mavros_set_mode("GUIDED")
            self.bench_start_time = self.get_clock().now()
            self._transition(Phase.BENCH_VELOCITY)
            return

        self._transition(Phase.TAKEOFF)

    def _tick_bench_velocity(self):
        if self.bench_start_time is None:
            self.bench_start_time = self.get_clock().now()

        elapsed = (self.get_clock().now() - self.bench_start_time).nanoseconds / 1e9
        climb_end = self.bench_climb_sec
        hold_end = climb_end + self.bench_hold_sec
        descend_end = hold_end + self.bench_descend_sec
        zero_end = descend_end + self.bench_zero_sec

        if elapsed < climb_end:
            vz = self.bench_velocity_z
            label = "bench_climb"
        elif elapsed < hold_end:
            vz = 0.0
            label = "bench_hold"
        elif elapsed < descend_end:
            vz = -self.bench_velocity_z
            label = "bench_descend"
        elif elapsed < zero_end:
            vz = 0.0
            label = "bench_zero"
        else:
            self._publish_velocity(0.0, 0.0, 0.0)
            if not self._desktop_disarm_done and not self._pending_command:
                self._pending_command = True
                self.get_logger().info("Bench velocity profile complete, disarming")
                self._call_flight_cmd(FlightCommand.Request.CMD_DISARM, 0.0, self._on_bench_disarm)
            return

        self.get_logger().info(f"{label}: vz={vz:.2f} m/s", throttle_duration_sec=1.0)
        self._publish_velocity(0.0, 0.0, vz)

    def _on_bench_disarm(self, success: bool, msg: str):
        self._pending_command = False
        self._desktop_disarm_done = True
        self.get_logger().info(f"Bench disarm: {'OK' if success else 'FAIL'} - {msg}")
        self._publish_event("bench_velocity_done")
        self._transition(Phase.DONE)

    def _tick_takeoff(self):
        if self._pending_command:
            return

        self._record_origin()
        self._pending_command = True
        self._call_flight_cmd(
            FlightCommand.Request.CMD_TAKEOFF,
            float(self.takeoff_altitude),
            self._on_takeoff_result,
        )

    def _on_takeoff_result(self, success: bool, msg: str):
        self._pending_command = False
        if success:
            self.get_logger().info(f"Takeoff OK: {msg}")
            self._publish_event("takeoff_accepted")
            self._transition(Phase.HOVER_TAKEOFF)
        else:
            self.get_logger().error(f"Takeoff FAILED: {msg}")
            self._transition(Phase.FAILSAFE)

    def _tick_hover_takeoff(self):
        self._publish_velocity(0.0, 0.0, 0.0)
        if self.use_mock:
            self._check_stable_and_transition(
                Phase.MOVE_ABOVE, "Mock takeoff hover stable", "hover_takeoff_stable"
            )
            return

        alt = self._get_fcu_altitude()
        if abs(alt - self.takeoff_altitude) <= self.altitude_tolerance:
            self._check_stable_and_transition(
                Phase.MOVE_ABOVE, f"Takeoff altitude stable at {alt:.2f}m", "hover_takeoff_stable"
            )
        else:
            self.hover_start_time = None

    def _tick_move_above(self):
        if self.use_mock:
            self.get_logger().info("Mock MOVE_ABOVE complete")
            self._transition(Phase.HOVER_ABOVE)
            return

        uwb = self._get_uwb()
        if uwb is None or not self._uwb_valid_and_fresh():
            self.get_logger().warn("No fresh UWB data, hovering", throttle_duration_sec=2.0)
            self._publish_velocity(0.0, 0.0, 0.0)
            return

        azimuth, distance, _ = uwb
        az_rad = azimuth * math.pi / 180.0
        horizontal_dist = distance * math.cos(az_rad)

        vx = 0.0
        vy = 0.0
        if abs(azimuth) > self.azimuth_deadband:
            vx = clamp(-self.kp_horizontal * az_rad * self.max_vel_xy, -self.max_vel_xy, self.max_vel_xy)
        if horizontal_dist > self.horizontal_deadband:
            vy = clamp(-self.kp_horizontal * horizontal_dist, -self.max_vel_xy, self.max_vel_xy)

        alt_err = self.takeoff_altitude - self._get_fcu_altitude()
        vz = clamp(self.kp_vertical * alt_err, -self.max_vel_z, self.max_vel_z)

        self._publish_velocity(vx, vy, vz)

        if abs(azimuth) < self.azimuth_deadband and horizontal_dist < self.horizontal_deadband:
            self._check_stable_and_transition(
                Phase.HOVER_ABOVE,
                f"Above target: az={azimuth:.1f}deg hdist={horizontal_dist:.2f}m",
                "above_target_reached",
            )
        else:
            self.hover_start_time = None

    def _tick_hover_above(self):
        self._publish_velocity(0.0, 0.0, 0.0)
        self._check_stable_and_transition(
            Phase.DESCEND, "Hover above target stable", "hover_above_done"
        )

    def _tick_descend(self):
        if self.use_mock:
            self._check_stable_and_transition(
                Phase.HOVER_FINAL, "Mock descent complete", "final_altitude_reached"
            )
            return

        vx = 0.0
        vy = 0.0
        uwb = self._get_uwb()
        if uwb is not None and self._uwb_valid_and_fresh():
            azimuth, distance, _ = uwb
            az_rad = azimuth * math.pi / 180.0
            horizontal_dist = distance * math.cos(az_rad)
            if abs(azimuth) > self.azimuth_deadband:
                vx = clamp(
                    -self.kp_horizontal * az_rad * self.max_vel_xy * 0.5,
                    -self.max_vel_xy * 0.5,
                    self.max_vel_xy * 0.5,
                )
            if horizontal_dist > self.horizontal_deadband:
                vy = clamp(
                    -self.kp_horizontal * horizontal_dist * 0.5,
                    -self.max_vel_xy * 0.5,
                    self.max_vel_xy * 0.5,
                )

        alt = self._get_fcu_altitude()
        alt_err = self.descend_altitude - alt
        vz = 0.0
        if abs(alt_err) > self.altitude_tolerance:
            vz = clamp(-self.kp_vertical * abs(alt_err), -self.max_vel_z, 0.0)

        self._publish_velocity(vx, vy, vz)

        if abs(alt - self.descend_altitude) <= self.altitude_tolerance:
            self._check_stable_and_transition(
                Phase.HOVER_FINAL, f"Final altitude stable at {alt:.2f}m", "final_altitude_reached"
            )
        else:
            self.hover_start_time = None

    def _tick_hover_final(self):
        self._publish_velocity(0.0, 0.0, 0.0)
        self._check_stable_and_transition(
            Phase.WAIT_GRASP, "Final hover stable, waiting for grasp", "final_hover_reached"
        )

    def _tick_wait_grasp(self):
        self._publish_velocity(0.0, 0.0, 0.0)
        if self.fake_grasp:
            if self.grasp_start_time is None:
                self.grasp_start_time = self.get_clock().now()
                self.get_logger().info(f"Waiting for fake grasp ({self.fake_grasp_delay_sec:.1f}s)")
            elapsed = (self.get_clock().now() - self.grasp_start_time).nanoseconds / 1e9
            if elapsed >= self.fake_grasp_delay_sec:
                self._publish_event("grasp_complete")
                self._transition(Phase.CLIMB)
            return

        if self._grasp_done:
            self._publish_event("grasp_complete")
            self._transition(Phase.CLIMB)
            return

        self.get_logger().info("Waiting for grasp_done signal...", throttle_duration_sec=2.0)

    def _tick_climb(self):
        if self.use_mock:
            self._check_stable_and_transition(Phase.HOVER_CLIMB, "Mock climb complete", "climb_done")
            return

        alt = self._get_fcu_altitude()
        alt_err = self.takeoff_altitude - alt
        vz = 0.0
        if abs(alt_err) > self.altitude_tolerance:
            vz = clamp(self.kp_vertical * alt_err, -self.max_vel_z, self.max_vel_z)

        self._publish_velocity(0.0, 0.0, vz)

        if abs(alt - self.takeoff_altitude) <= self.altitude_tolerance:
            self._check_stable_and_transition(Phase.HOVER_CLIMB, f"Climbed to {alt:.2f}m", "climb_done")
        else:
            self.hover_start_time = None

    def _tick_hover_climb(self):
        self._publish_velocity(0.0, 0.0, 0.0)
        self._check_stable_and_transition(
            Phase.RETURN, "Climb hover stable, returning", "hover_climb_done"
        )

    def _tick_return(self):
        if self.use_mock:
            self.get_logger().info("Mock RETURN complete")
            self._transition(Phase.HOVER_RETURN)
            return

        pos = self._get_local_xy()
        if pos is None:
            self._publish_velocity(0.0, 0.0, 0.0)
            self.get_logger().warn("No local pose for return, hovering", throttle_duration_sec=2.0)
            return

        dx = pos[0] - self.origin_x
        dy = pos[1] - self.origin_y
        dist = math.sqrt(dx * dx + dy * dy)

        vx = 0.0
        vy = 0.0
        if dist > self.return_xy_tolerance:
            vx = -self.kp_return * dx
            vy = -self.kp_return * dy
            speed = math.sqrt(vx * vx + vy * vy)
            if speed > self.max_vel_xy:
                scale = self.max_vel_xy / speed
                vx *= scale
                vy *= scale

        alt_err = self.takeoff_altitude - self._get_fcu_altitude()
        vz = clamp(self.kp_vertical * alt_err, -self.max_vel_z, self.max_vel_z)
        self._publish_velocity(vx, vy, vz)

        if dist <= self.return_xy_tolerance:
            self._check_stable_and_transition(
                Phase.HOVER_RETURN, f"Returned to origin: d={dist:.2f}m", "return_arrived"
            )
        else:
            self.hover_start_time = None

    def _tick_hover_return(self):
        self._publish_velocity(0.0, 0.0, 0.0)
        self._check_stable_and_transition(
            Phase.WAIT_DROP, "Hover above origin stable, waiting for drop", "hover_return_done"
        )

    def _tick_wait_drop(self):
        self._publish_velocity(0.0, 0.0, 0.0)
        if self.fake_drop:
            if self.drop_start_time is None:
                self.drop_start_time = self.get_clock().now()
                self.get_logger().info(f"Waiting for fake drop ({self.fake_drop_delay_sec:.1f}s)")
            elapsed = (self.get_clock().now() - self.drop_start_time).nanoseconds / 1e9
            if elapsed >= self.fake_drop_delay_sec:
                self._publish_event("drop_complete")
                self._transition(Phase.LAND)
            return

        if self._drop_done:
            self._publish_event("drop_complete")
            self._transition(Phase.LAND)
            return

        self.get_logger().info("Waiting for drop_done signal...", throttle_duration_sec=2.0)

    def _tick_land(self):
        if self._pending_command:
            return
        self._publish_velocity(0.0, 0.0, 0.0)
        self._pending_command = True
        self._call_flight_cmd(FlightCommand.Request.CMD_MODE_LAND, 0.0, self._on_land_result)

    def _on_land_result(self, success: bool, msg: str):
        self._pending_command = False
        if success:
            self.get_logger().info(f"Land OK: {msg}")
            self._publish_event("landing")
            self._transition(Phase.DONE)
            reset_msg = String()
            reset_msg.data = "RESET"
            self.flight_reset_pub.publish(reset_msg)
        else:
            self.get_logger().error(f"Land FAILED: {msg}, retrying")

    def _tick_done(self):
        self._publish_velocity(0.0, 0.0, 0.0)

    def _tick_paused_manual(self):
        self._publish_velocity(0.0, 0.0, 0.0)
        self.get_logger().warn(
            "Mission paused by RC/mode takeover. Restart mission node to resume autonomy.",
            throttle_duration_sec=5.0,
        )

    def _tick_recovering(self):
        self._publish_velocity(0.0, 0.0, 0.0)

        if self._last_link_status == "OK":
            self.get_logger().info("Link recovered, resuming from hover-takeoff stage")
            self._publish_event("link_recovered")
            self._recovery_start_time = None
            self._transition(Phase.HOVER_TAKEOFF)
            return

        if self._recovery_start_time is None:
            self._recovery_start_time = self.get_clock().now()

        elapsed = (self.get_clock().now() - self._recovery_start_time).nanoseconds / 1e9
        if elapsed >= self.recovery_timeout:
            self.get_logger().error(f"Recovery timeout ({elapsed:.1f}s), entering FAILSAFE")
            self._publish_event("recovery_timeout")
            self._transition(Phase.FAILSAFE)

    def _tick_failsafe(self):
        self._publish_velocity(0.0, 0.0, 0.0)
        if self._pending_command:
            return
        self._pending_command = True
        self._call_flight_cmd(
            FlightCommand.Request.CMD_MODE_LAND,
            0.0,
            lambda ok, msg: self.get_logger().info(
                f"Failsafe LAND: {'OK' if ok else 'FAILED'} - {msg}"
            ),
        )

    def _record_origin(self):
        if self.origin_recorded:
            return
        pos = self._get_local_xy()
        if pos is not None:
            self.origin_x = pos[0]
            self.origin_y = pos[1]
        self.origin_recorded = True
        self.get_logger().info(f"Origin recorded: ({self.origin_x:.2f}, {self.origin_y:.2f})")

    def _check_stable_and_transition(self, next_phase: Phase, log_msg: str, event: str):
        now = self.get_clock().now()
        if self.hover_start_time is None:
            self.hover_start_time = now
            self.get_logger().info(f"{log_msg}, stabilizing")
            return

        elapsed = (now - self.hover_start_time).nanoseconds / 1e9
        if elapsed >= self.hover_stable_time:
            self.get_logger().info(log_msg)
            self._publish_event(event)
            self._transition(next_phase)

    def _transition(self, new_phase: Phase):
        old = PHASE_NAMES[self.phase]
        new = PHASE_NAMES[new_phase]
        self.get_logger().info(f"Phase: {old} -> {new}")
        self.phase = new_phase
        self.hover_start_time = None
        if new_phase != Phase.WAIT_GRASP:
            self.grasp_start_time = None
        if new_phase != Phase.WAIT_DROP:
            self.drop_start_time = None

    def _publish_velocity(self, vx: float, vy: float, vz: float):
        now = self.get_clock().now()
        dt = max((now - self._last_velocity_time).nanoseconds / 1e9, 1.0 / max(self.control_rate_hz, 1.0))
        max_delta = self.velocity_slew_rate * dt
        last_vx, last_vy, last_vz = self._last_velocity
        vx = clamp(vx, last_vx - max_delta, last_vx + max_delta)
        vy = clamp(vy, last_vy - max_delta, last_vy + max_delta)
        vz = clamp(vz, last_vz - max_delta, last_vz + max_delta)

        msg = TwistStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "body"
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.linear.z = vz
        self.vel_pub.publish(msg)

        self._last_velocity = (vx, vy, vz)
        self._last_velocity_time = now

    def _publish_state(self):
        msg = String()
        msg.data = PHASE_NAMES[self.phase]
        self.state_pub.publish(msg)

    def _publish_event(self, event: str):
        msg = String()
        msg.data = event
        self.event_pub.publish(msg)

    def _call_flight_cmd(self, command: int, param: float, callback):
        if not self.flight_cmd_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Flight command service not ready")
            callback(False, "service not ready")
            return

        req = FlightCommand.Request()
        req.command = command
        req.param = param
        future = self.flight_cmd_client.call_async(req)

        def _done(fut):
            try:
                result = fut.result()
                callback(result.success, result.message)
            except Exception as exc:
                self.get_logger().error(f"Service call exception: {exc}")
                callback(False, str(exc))

        future.add_done_callback(_done)

    def _call_mavros_set_mode(self, mode_str: str):
        if self.use_mock:
            return
        if not self.mavros_set_mode_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("MAVROS set_mode service not available")
            return
        req = SetMode.Request()
        req.custom_mode = mode_str
        future = self.mavros_set_mode_client.call_async(req)

        def done(fut):
            try:
                result = fut.result()
                self.get_logger().info(f"MAVROS SetMode({mode_str}): mode_sent={result.mode_sent}")
            except Exception as exc:
                self.get_logger().error(f"SetMode failed: {exc}")

        future.add_done_callback(done)


def main(args=None):
    rclpy.init(args=args)
    node = TestMissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
