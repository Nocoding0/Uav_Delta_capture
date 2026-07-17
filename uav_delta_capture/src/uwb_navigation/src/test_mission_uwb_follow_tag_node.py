#!/usr/bin/env python3
"""Real UWB tag-follow test that finishes with a low hover and LAND."""

import math

import rclpy

from test_mission_node import Phase, TestMissionNode, clamp


class UwbFollowTagNode(TestMissionNode):
    FOLLOW_MODE = "uwb_follow_tag_land"

    def __init__(self):
        super().__init__()
        self.follow_timeout_sec = max(5.0, float(self.declare_parameter("follow_timeout_sec", 60.0).value))
        self.follow_center_radius_m = clamp(float(self.declare_parameter("follow_center_radius_m", 0.10).value), 0.05, 0.50)
        self.follow_quiet_speed_mps = clamp(abs(float(self.declare_parameter("follow_quiet_speed_mps", 0.02).value)), 0.005, self.max_vel_xy)
        self.follow_stationary_sec = max(1.0, float(self.declare_parameter("follow_stationary_sec", 5.0).value))
        self.follow_front_loss_sec = max(0.0, float(self.declare_parameter("follow_front_loss_sec", 2.0).value))
        self._follow_started_time = None
        self._follow_missing_start_time = None
        self._follow_front_loss_start_time = None
        self._follow_stationary_start_time = None
        self._follow_acquire_ok = False
        self._follow_stationary_ok = False
        self._follow_descend_ok = False
        self.get_logger().info(
            "test_mission_uwb_follow_tag_node started: "
            f"takeoff={self.takeoff_altitude:.2f}m descend={self.descend_altitude:.2f}m "
            f"timeout={self.follow_timeout_sec:.1f}s center={self.follow_center_radius_m:.2f}m "
            f"quiet={self.follow_quiet_speed_mps:.3f}m/s stationary={self.follow_stationary_sec:.1f}s"
        )

    def _is_takeoff_land_mode(self):
        return self.mission_mode == self.FOLLOW_MODE or super()._is_takeoff_land_mode()

    def _is_uwb_staged_mode(self):
        return self.mission_mode == self.FOLLOW_MODE or super()._is_uwb_staged_mode()

    def _takeoff_land_label(self):
        return "UWB_FOLLOW_TAG_LAND" if self.mission_mode == self.FOLLOW_MODE else super()._takeoff_land_label()

    def _takeoff_land_text(self):
        return "UWB follow-tag-land" if self.mission_mode == self.FOLLOW_MODE else super()._takeoff_land_text()

    def _transition(self, new_phase):
        super()._transition(new_phase)
        if new_phase != Phase.HOVER_ABOVE:
            self._follow_stationary_start_time = None
        if new_phase != Phase.MOVE_ABOVE:
            self._follow_missing_start_time = None
            self._follow_front_loss_start_time = None

    def _follow_altitude(self):
        rel_alt, source = self._get_takeoff_land_relative_altitude()
        vz = 0.0 if rel_alt is None else clamp(
            self.kp_vertical * (self.takeoff_altitude - rel_alt), -self.max_vel_z, self.max_vel_z
        )
        return rel_alt, source, vz

    def _follow_geometry(self, now):
        uwb = self._get_uwb()
        if uwb is None or not self._uwb_valid_and_fresh():
            return None
        raw_azimuth, distance, elevation = uwb
        return self._uwb_smoothed_geometry(now, raw_azimuth, distance, elevation), raw_azimuth, elevation

    def _follow_command(self, geom, vz):
        limit = self.max_vel_xy
        if geom["horizontal_dist"] <= self.uwb_slow_radius_m:
            limit = min(limit, self.uwb_slow_max_vel_xy)
        vx = self.kp_horizontal * geom["forward_dist"]
        vy = self.kp_horizontal * geom["lateral_dist"]
        required = math.hypot(vx, vy)
        if required > limit and required > 0.0:
            scale = limit / required
            vx *= scale
            vy *= scale
        return vx, vy, vz, limit, required

    def _start_follow_scan(self, reason, azimuth=None):
        self._publish_velocity(0.0, 0.0, 0.0, frame_id="body", immediate=True)
        if azimuth is not None:
            self._uwb_scan_direction = -1.0 if azimuth < 0.0 else 1.0
        self._publish_event("uwb_follow_scan")
        self.get_logger().warn(f"{reason}; stopping XY and starting yaw scan")
        self._transition(Phase.UWB_SCAN_YAW)

    def _follow_timeout(self, now):
        if self._follow_started_time is None:
            self._follow_started_time = now
            return False
        elapsed = (now - self._follow_started_time).nanoseconds / 1e9
        if elapsed < self.follow_timeout_sec:
            return False
        self._takeoff_land_abort_reason = f"UWB follow timeout after {elapsed:.1f}s"
        self.get_logger().warn(f"{self._takeoff_land_abort_reason}; landing")
        self._publish_event("uwb_follow_timeout_land")
        self._transition(Phase.LAND)
        return True

    def _acquire_front_lock(self, now, geom, raw_azimuth, elevation, vz):
        azimuth = geom["body_azimuth"]
        aligned = geom["forward_dist"] > 0.0 and abs(azimuth) <= self.uwb_front_line_lock_deg
        if not aligned:
            self._start_follow_scan(
                "UWB follow acquire target not front-line aligned "
                f"(az={azimuth:.1f}/{self.uwb_front_line_lock_deg:.1f}deg)", azimuth
            )
            return
        if self._uwb_front_start_time is None:
            self._uwb_front_start_time = now
        elapsed = (now - self._uwb_front_start_time).nanoseconds / 1e9
        self._publish_velocity(0.0, 0.0, vz, frame_id="body", immediate=True)
        if elapsed < self.uwb_front_stable_sec:
            self.get_logger().info(
                "UWB follow front candidate: "
                f"az={azimuth:.1f}deg raw_az={raw_azimuth:.1f}deg raw_el={elevation:.1f}deg "
                f"hdist={geom['horizontal_dist']:.2f}m stable={elapsed:.1f}/{self.uwb_front_stable_sec:.1f}s",
                throttle_duration_sec=0.2,
            )
            return
        self._uwb_front_start_time = None
        self._uwb_front_stable_once = True
        self._uwb_front_line_locked = True
        self._follow_acquire_ok = True
        self._publish_event("uwb_follow_acquired")
        self.get_logger().info(
            "UWB follow target locked: "
            f"az={azimuth:.1f}deg raw_az={raw_azimuth:.1f}deg raw_el={elevation:.1f}deg "
            f"hdist={geom['horizontal_dist']:.2f}m; starting 2D follow"
        )

    def _tick_move_above(self):
        if self.mission_mode != self.FOLLOW_MODE:
            super()._tick_move_above()
            return
        now = self.get_clock().now()
        if self._follow_timeout(now):
            return
        rel_alt, altitude_source, vz = self._follow_altitude()
        follow = self._follow_geometry(now)
        if follow is None:
            self._publish_velocity(0.0, 0.0, vz, frame_id="body", immediate=True)
            if self._follow_missing_start_time is None:
                self._follow_missing_start_time = now
            elapsed = (now - self._follow_missing_start_time).nanoseconds / 1e9
            if elapsed >= self.uwb_missing_timeout_sec:
                self._start_follow_scan(f"UWB follow data missing for {elapsed:.1f}s")
                return
            self.get_logger().warn(
                f"UWB follow waiting for fresh data {elapsed:.1f}/{self.uwb_missing_timeout_sec:.1f}s",
                throttle_duration_sec=0.5,
            )
            return
        self._follow_missing_start_time = None
        geom, raw_azimuth, elevation = follow
        azimuth = geom["body_azimuth"]
        if geom["body_elevation"] < self.uwb_min_body_elevation_deg:
            self._start_follow_scan(
                "UWB follow geometry invalid "
                f"(body_el={geom['body_elevation']:.1f}/{self.uwb_min_body_elevation_deg:.1f}deg)", azimuth
            )
            return
        if not self._uwb_front_line_locked:
            self._acquire_front_lock(now, geom, raw_azimuth, elevation, vz)
            return
        front_ok = geom["forward_dist"] > 0.0 and abs(azimuth) <= self.uwb_approach_front_sector_deg
        if not front_ok:
            if self._follow_front_loss_start_time is None:
                self._follow_front_loss_start_time = now
            elapsed = (now - self._follow_front_loss_start_time).nanoseconds / 1e9
            self._publish_velocity(0.0, 0.0, vz, frame_id="body", immediate=True)
            if elapsed >= self.follow_front_loss_sec:
                self._start_follow_scan(
                    f"UWB follow target left front sector (az={azimuth:.1f}deg) after {elapsed:.1f}s", azimuth
                )
                return
            self.get_logger().warn(
                f"UWB follow target outside front sector, holding {elapsed:.1f}/{self.follow_front_loss_sec:.1f}s "
                f"az={azimuth:.1f}deg", throttle_duration_sec=0.5
            )
            return
        self._follow_front_loss_start_time = None
        radius = self._mission_xy_distance_from_origin()
        if radius is not None and radius >= self.mission_soft_radius_m:
            self._publish_velocity(0.0, 0.0, vz, frame_id="body", immediate=True)
            self.get_logger().warn(
                f"UWB follow soft radius reached: r={radius:.2f}/{self.mission_soft_radius_m:.2f}m, holding position",
                throttle_duration_sec=0.5,
            )
            return
        vx, vy, vz, speed_limit, required = self._follow_command(geom, vz)
        if geom["horizontal_dist"] <= self.follow_center_radius_m and required <= self.follow_quiet_speed_mps:
            self._publish_velocity(0.0, 0.0, vz, frame_id="body", immediate=True)
            self.get_logger().info(
                "UWB follow near-center candidate: "
                f"hdist={geom['horizontal_dist']:.2f}/{self.follow_center_radius_m:.2f}m "
                f"required_xy={required:.3f}/{self.follow_quiet_speed_mps:.3f}m/s",
                throttle_duration_sec=0.2,
            )
            self._transition(Phase.HOVER_ABOVE)
            return
        self._publish_velocity(vx, vy, vz, frame_id="body")
        alt_text = "missing" if rel_alt is None else f"{rel_alt:.2f}m"
        self.get_logger().info(
            "UWB follow BODY_NED: "
            f"az={azimuth:.1f}deg raw_az={raw_azimuth:.1f}deg raw_el={elevation:.1f}deg "
            f"body_el={geom['body_elevation']:.1f}deg hdist={geom['horizontal_dist']:.2f}m "
            f"body_dist=({geom['forward_dist']:.2f},{geom['lateral_dist']:.2f},{geom['vertical_dist']:.2f}) "
            f"rel_alt={alt_text} ({altitude_source}) cmd_body=({vx:.2f},{vy:.2f},{vz:.2f}) "
            f"required_xy={required:.3f}m/s speed_limit={speed_limit:.2f}",
            throttle_duration_sec=0.5,
        )

    def _tick_hover_above(self):
        if self.mission_mode != self.FOLLOW_MODE:
            super()._tick_hover_above()
            return
        now = self.get_clock().now()
        if self._follow_timeout(now):
            return
        rel_alt, altitude_source, vz = self._follow_altitude()
        follow = self._follow_geometry(now)
        if follow is None:
            self._start_follow_scan("UWB follow stationary candidate lost data")
            return
        geom, raw_azimuth, elevation = follow
        azimuth = geom["body_azimuth"]
        front_ok = (
            geom["forward_dist"] > 0.0
            and abs(azimuth) <= self.uwb_approach_front_sector_deg
            and geom["body_elevation"] >= self.uwb_min_body_elevation_deg
        )
        if not front_ok:
            self._start_follow_scan(
                f"UWB follow stationary candidate left front sector (az={azimuth:.1f}deg)", azimuth
            )
            return
        _, _, vz, _, required = self._follow_command(geom, vz)
        centered = geom["horizontal_dist"] <= self.follow_center_radius_m
        quiet = required <= self.follow_quiet_speed_mps
        if not centered or not quiet:
            self.get_logger().info(
                "UWB follow stationary reset: "
                f"hdist={geom['horizontal_dist']:.2f}/{self.follow_center_radius_m:.2f}m "
                f"required_xy={required:.3f}/{self.follow_quiet_speed_mps:.3f}m/s; resuming follow",
                throttle_duration_sec=0.2,
            )
            self._transition(Phase.MOVE_ABOVE)
            return
        self._publish_velocity(0.0, 0.0, vz, frame_id="body", immediate=True)
        if self._follow_stationary_start_time is None:
            self._follow_stationary_start_time = now
        elapsed = (now - self._follow_stationary_start_time).nanoseconds / 1e9
        if elapsed >= self.follow_stationary_sec:
            self._follow_stationary_ok = True
            self._uwb_approach_ok = True
            self._publish_event("uwb_follow_tag_stationary")
            self.get_logger().info(
                "UWB follow tag stationary confirmed: "
                f"stable={elapsed:.1f}/{self.follow_stationary_sec:.1f}s hdist={geom['horizontal_dist']:.2f}m "
                f"body_dist=({geom['forward_dist']:.2f},{geom['lateral_dist']:.2f},{geom['vertical_dist']:.2f}); "
                "descending before LAND"
            )
            self._transition(Phase.DESCEND)
            return
        alt_text = "missing" if rel_alt is None else f"{rel_alt:.2f}m"
        self.get_logger().info(
            "UWB follow stationary candidate: "
            f"stable={elapsed:.1f}/{self.follow_stationary_sec:.1f}s hdist={geom['horizontal_dist']:.2f}m "
            f"required_xy={required:.3f}m/s rel_alt={alt_text} ({altitude_source}) "
            f"raw_az={raw_azimuth:.1f}deg raw_el={elevation:.1f}deg",
            throttle_duration_sec=0.5,
        )

    def _tick_descend(self):
        if self.mission_mode != self.FOLLOW_MODE:
            super()._tick_descend()
            return
        rel_alt, source = self._get_takeoff_land_relative_altitude()
        if rel_alt is None:
            self._takeoff_land_abort_reason = "UWB follow descend waiting for relative altitude source"
            self.get_logger().error(self._takeoff_land_abort_reason)
            self._transition(Phase.FAILSAFE)
            return
        if rel_alt > self.descend_altitude + self.altitude_tolerance:
            vz = clamp(self.kp_vertical * (self.descend_altitude - rel_alt), -self.max_vel_z, 0.0)
            self.hover_start_time = None
            self._publish_velocity(0.0, 0.0, vz, frame_id="body")
            self.get_logger().info(
                "UWB follow descending: "
                f"rel_alt={rel_alt:.2f}/{self.descend_altitude:.2f}m ({source}) "
                f"cmd_body=(0.00,0.00,{vz:.2f}); tag motion ignored after descent",
                throttle_duration_sec=0.5,
            )
            return
        self._publish_velocity(0.0, 0.0, 0.0, frame_id="body", immediate=True)
        if self.hover_start_time is None:
            self.hover_start_time = self.get_clock().now()
            self.get_logger().info(
                f"UWB follow descend altitude reached at rel_alt={rel_alt:.2f}m ({source}), stabilizing before LAND"
            )
            return
        elapsed = (self.get_clock().now() - self.hover_start_time).nanoseconds / 1e9
        if elapsed >= self.hover_stable_time:
            self._follow_descend_ok = True
            self._publish_event("uwb_follow_descend_complete")
            self.get_logger().info("UWB follow low hover stable; tag motion is ignored, landing")
            self._transition(Phase.LAND)
            return
        self.get_logger().info(
            f"UWB follow low hover stabilizing {elapsed:.1f}/{self.hover_stable_time:.1f}s "
            f"at rel_alt={rel_alt:.2f}m ({source})",
            throttle_duration_sec=0.5,
        )

    def _report_takeoff_land_result(self, force_fail_reason=None):
        if self.mission_mode != self.FOLLOW_MODE:
            super()._report_takeoff_land_result(force_fail_reason)
            return
        if self._takeoff_land_result_reported:
            return
        self._takeoff_land_result_reported = True
        snapshot = self._bench_snapshot()
        core_ok = (
            self._bench_arm_ok and self._takeoff_land_takeoff_ok and self._takeoff_land_hover_ok
            and self._follow_acquire_ok and self._follow_stationary_ok
            and self._follow_descend_ok and self._takeoff_land_land_ok
        )
        sensor_ok = (
            snapshot["fcu_ok"] and snapshot["rc_ok"] and snapshot["uwb_ok"]
            and snapshot["pose_ok"] and snapshot["range_ok"] and snapshot["flow_ok"]
            and snapshot["set_mode_ok"]
        )
        result = "PASS" if core_ok and sensor_ok and not force_fail_reason else "FAIL"
        self.get_logger().info("========== UWB_FOLLOW_TAG_LAND RESULT ==========")
        self.get_logger().info(f"UWB_FOLLOW_TAG_LAND RESULT: {result}")
        self.get_logger().info(
            "Core links: "
            f"ARM={'OK' if self._bench_arm_ok else 'FAIL'} "
            f"TAKEOFF={'OK' if self._takeoff_land_takeoff_ok else 'FAIL'} "
            f"HOVER={'OK' if self._takeoff_land_hover_ok else 'FAIL'} "
            f"ACQUIRE={'OK' if self._follow_acquire_ok else 'FAIL'} "
            f"TAG_STILL={'OK' if self._follow_stationary_ok else 'FAIL'} "
            f"DESCEND={'OK' if self._follow_descend_ok else 'FAIL'} "
            f"LAND={'OK' if self._takeoff_land_land_ok else 'FAIL'}"
        )
        self.get_logger().info(f"Sensor links: {self._format_bench_snapshot(snapshot)}")
        if force_fail_reason:
            self.get_logger().warn(f"UWB follow-tag-land warnings: {force_fail_reason}")
        self.get_logger().info("===============================================")
        self._shutdown_requested = True


def main(args=None):
    rclpy.init(args=args)
    node = UwbFollowTagNode()
    try:
        while rclpy.ok() and not node.should_exit():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
