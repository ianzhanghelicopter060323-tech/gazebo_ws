"""Closed-loop ``cmd_vel`` alignment owned by the navigation package."""

import math
import threading
import time

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import rospy

from smart_factory_navigation import error_codes, states
from smart_factory_navigation.navigation_stage import NavigationOutcome


class BaseAlignmentFailure(RuntimeError):
    def __init__(self, error_code, message):
        super().__init__(message)
        self.error_code = error_code


class BaseAlignmentPreempted(RuntimeError):
    pass


class BaseAlignmentController:
    """Sole publisher for navigation-owned direct base velocity commands."""

    def __init__(self, localization, publish_state, preempt_requested):
        self._localization = localization
        self._publish_state = publish_state
        self._preempt_requested = preempt_requested
        cmd_vel_topic = rospy.get_param(
            "~navigation/cmd_vel_topic", "/cmd_vel"
        )
        self._escape_enabled = bool(
            rospy.get_param("~navigation/recovery/enabled", True)
        )
        self._escape_speed = float(
            rospy.get_param("~navigation/recovery/speed", 0.05)
        )
        self._escape_max_distance = float(
            rospy.get_param("~navigation/recovery/max_distance", 0.08)
        )
        self._escape_wall_timeout = float(
            rospy.get_param("~navigation/recovery/wall_timeout", 2.0)
        )
        self._escape_scan_sector = float(
            rospy.get_param("~navigation/recovery/scan_sector_half_angle", 0.52)
        )
        # Scale the bounded escape distance by the chosen direction's sector
        # clearance so tight gaps get a short nudge while open space stays
        # capped by recovery/max_distance. 0.7 leaves a 30% margin.
        self._escape_distance_scale = float(
            rospy.get_param("~navigation/recovery/distance_scale_factor", 0.7)
        )
        # Lateral (strafe) escape.  When BOTH the front and rear corridors are
        # too tight for a forward/back nudge (the robot is jammed in a fan
        # dead-end) but one side opens up, strafe along the base_link y-axis
        # toward the larger side gap so the next channel selection sees open
        # space.  The chassis is omnidirectional (planar_move), so linear.y is
        # a real, bounded motion primitive shared with the baseline escape.
        self._escape_strafe_enabled = bool(
            rospy.get_param("~navigation/recovery/strafe_enabled", True)
        )
        self._escape_front_rear_threshold = float(
            rospy.get_param(
                "~navigation/recovery/strafe_front_rear_threshold", 0.40
            )
        )
        self._escape_side_gap_threshold = float(
            rospy.get_param(
                "~navigation/recovery/strafe_side_gap_threshold", 0.50
            )
        )
        self._escape_side_sector = float(
            rospy.get_param(
                "~navigation/recovery/strafe_side_sector_half_angle", 0.78
            )
        )
        self._escape_strafe_speed = float(
            rospy.get_param("~navigation/recovery/strafe_speed", 0.30)
        )
        self._escape_strafe_distance = float(
            rospy.get_param("~navigation/recovery/strafe_distance", 0.30)
        )
        self._escape_strafe_max_total = float(
            rospy.get_param("~navigation/recovery/strafe_max_total", 0.60)
        )
        self._escape_strafe_wall_timeout = float(
            rospy.get_param("~navigation/recovery/strafe_wall_timeout", 4.0)
        )
        self._scan_lock = threading.Lock()
        self._latest_scan = None

        self._heading_tolerance = float(
            rospy.get_param(
                "~navigation/intermediate_yaw_tolerance", 0.25
            )
        )
        self._heading_timeout = float(
            rospy.get_param(
                "~navigation/heading_alignment_timeout", 8.0
            )
        )
        self._heading_gain = float(
            rospy.get_param("~navigation/heading_alignment_kp", 1.0)
        )
        self._heading_max_speed = float(
            rospy.get_param(
                "~navigation/heading_alignment_max_angular_speed", 0.45
            )
        )
        self._heading_min_speed = float(
            rospy.get_param(
                "~navigation/heading_alignment_min_angular_speed", 0.40
            )
        )
        if not (
            math.isfinite(self._heading_tolerance)
            and 0.0 < self._heading_tolerance <= math.pi
        ):
            raise ValueError(
                "navigation/intermediate_yaw_tolerance must be in (0, pi]"
            )
        for name, value in (
            ("heading_alignment_timeout", self._heading_timeout),
            ("heading_alignment_kp", self._heading_gain),
            (
                "heading_alignment_max_angular_speed",
                self._heading_max_speed,
            ),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(
                    "navigation/{} must be positive".format(name)
                )
        if not (
            math.isfinite(self._heading_min_speed)
            and self._heading_min_speed > 0.0
            and self._heading_min_speed <= self._heading_max_speed
        ):
            raise ValueError(
                "navigation/heading_alignment_min_angular_speed must be "
                "positive and no greater than the maximum"
            )

        self._tolerance = float(
            rospy.get_param("~pickup/direct_alignment_tolerance", 0.008)
        )
        self._gain = float(
            rospy.get_param("~pickup/direct_alignment_gain", 1.0)
        )
        self._max_speed = float(
            rospy.get_param("~pickup/direct_alignment_max_speed", 0.08)
        )
        self._min_speed = float(
            rospy.get_param("~pickup/direct_alignment_min_speed", 0.05)
        )
        self._timeout = float(
            rospy.get_param("~pickup/direct_alignment_timeout", 8.0)
        )
        self._wall_timeout = float(
            rospy.get_param("~pickup/direct_alignment_wall_timeout", 45.0)
        )
        self._alignment_clock_stall_timeout = float(
            rospy.get_param("~pickup/direct_alignment_clock_stall_timeout", 10.0)
        )
        for name, value in (
            ("direct_alignment_tolerance", self._tolerance),
            ("direct_alignment_gain", self._gain),
            ("direct_alignment_max_speed", self._max_speed),
            ("direct_alignment_min_speed", self._min_speed),
            ("direct_alignment_timeout", self._timeout),
            ("direct_alignment_wall_timeout", self._wall_timeout),
            (
                "direct_alignment_clock_stall_timeout",
                self._alignment_clock_stall_timeout,
            ),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError("pickup/{} must be positive".format(name))
        if self._min_speed > self._max_speed:
            raise ValueError(
                "pickup/direct_alignment_min_speed must not exceed max speed"
            )
        self._velocity_publisher = rospy.Publisher(
            cmd_vel_topic, Twist, queue_size=1
        )
        self._scan_subscriber = rospy.Subscriber(
            rospy.get_param("~navigation/recovery/scan_topic", "/scan"),
            LaserScan,
            self._scan_callback,
            queue_size=1,
        )
        for name, value in (
            ("recovery/speed", self._escape_speed),
            ("recovery/max_distance", self._escape_max_distance),
            ("recovery/wall_timeout", self._escape_wall_timeout),
            ("recovery/scan_sector_half_angle", self._escape_scan_sector),
            ("recovery/distance_scale_factor", self._escape_distance_scale),
            ("recovery/strafe_front_rear_threshold", self._escape_front_rear_threshold),
            ("recovery/strafe_side_gap_threshold", self._escape_side_gap_threshold),
            ("recovery/strafe_side_sector_half_angle", self._escape_side_sector),
            ("recovery/strafe_speed", self._escape_strafe_speed),
            ("recovery/strafe_distance", self._escape_strafe_distance),
            ("recovery/strafe_max_total", self._escape_strafe_max_total),
            ("recovery/strafe_wall_timeout", self._escape_strafe_wall_timeout),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError("navigation/{} must be positive".format(name))
        if not math.isfinite(self._escape_distance_scale) or (
            self._escape_distance_scale < 0.0 or self._escape_distance_scale > 1.0
        ):
            raise ValueError(
                "navigation/recovery/distance_scale_factor must be in [0, 1]"
            )

    @property
    def heading_tolerance(self):
        return self._heading_tolerance

    def stop(self):
        """Publish an explicit zero velocity command."""
        self._velocity_publisher.publish(Twist())

    def _scan_callback(self, message):
        with self._scan_lock:
            self._latest_scan = message

    def _escape_max_distance_for(self, clearance):
        """Bound one escape nudge by 70% of the direction's clearance."""
        if math.isfinite(clearance):
            return min(
                self._escape_distance_scale * clearance,
                self._escape_max_distance,
            )
        return self._escape_max_distance

    @staticmethod
    def _sector_clearance(scan, center, half_angle):
        values = []
        for index, value in enumerate(scan.ranges):
            angle = scan.angle_min + index * scan.angle_increment
            delta = math.atan2(math.sin(angle - center), math.cos(angle - center))
            if (
                abs(delta) <= half_angle
                and math.isfinite(value)
                and scan.range_min <= value <= scan.range_max
            ):
                values.append(value)
        if not values:
            return math.nan
        values.sort()
        # A low percentile is robust to one bad beam while still representing
        # the closest obstacle across the escape corridor.
        return values[max(0, int(0.1 * (len(values) - 1)))]

    def _strafe_gate_open(self, scan, front, rear):
        """True when forward/back are both too tight but a side gap is large.

        This is the "前后均距离不足、左/右距离充足，使用横移" trigger: front
        and rear 10th-percentile clearances must both sit under the configured
        threshold (the robot is jammed nose-to-tail in a cone dead-end) while
        at least one lateral sector offers a gap large enough to strafe into.
        """
        if (
            not math.isfinite(front)
            or not math.isfinite(rear)
            or front >= self._escape_front_rear_threshold
            or rear >= self._escape_front_rear_threshold
        ):
            return False
        left = self._sector_clearance(
            scan, math.pi / 2.0, self._escape_side_sector
        )
        right = self._sector_clearance(
            scan, -math.pi / 2.0, self._escape_side_sector
        )
        return (
            math.isfinite(left)
            and math.isfinite(right)
            and max(left, right) >= self._escape_side_gap_threshold
        )

    def _strafe_probe(self, direction, frame_id):
        """Publish one bounded lateral nudge and return the lateral travel.

        ``direction`` is +1 toward the robot's left (+y in base_link) and -1
        toward the right.  Lateral travel is the map-frame displacement
        projected onto the robot's lateral axis at the start yaw; a pure
        strafe keeps the yaw constant so the projection is exact.  The probe
        stops on target travel, the wall timeout, or a stall (pressing into an
        obstacle the scan cannot see).
        """
        start_pose = self._localization.localized_pose(frame_id)
        if start_pose is None:
            return 0.0
        yaw = start_pose[2]
        lat_x = -math.sin(yaw)
        lat_y = math.cos(yaw)
        stall_timeout = min(0.5, self._escape_strafe_wall_timeout / 4.0)
        deadline = time.monotonic() + self._escape_strafe_wall_timeout
        last_moved = 0.0
        last_progress_at = time.monotonic()
        moved = 0.0
        command = Twist()
        command.linear.y = direction * self._escape_strafe_speed
        while not rospy.is_shutdown():
            if self._preempt_requested():
                raise BaseAlignmentPreempted(
                    "task preempted during bounded lateral recovery"
                )
            current = self._localization.localized_pose(frame_id)
            if current is not None:
                signed = (
                    (current[0] - start_pose[0]) * lat_x
                    + (current[1] - start_pose[1]) * lat_y
                )
                moved = abs(signed)
            now = time.monotonic()
            if moved > last_moved:
                last_moved = moved
                last_progress_at = now
            if (
                moved >= self._escape_strafe_distance
                or now >= deadline
                or now - last_progress_at >= stall_timeout
            ):
                break
            self._velocity_publisher.publish(command)
            time.sleep(0.05)
        return moved

    def _strafe(self, scan, frame_id):
        """Strafe toward the larger side gap, bounded by strafe_max_total.

        Re-checks the gate between steps so a robot that has already opened
        up a forward/back corridor stops instead of drifting past the gap.
        Returns True when any meaningful lateral travel was achieved.
        """
        left = self._sector_clearance(
            scan, math.pi / 2.0, self._escape_side_sector
        )
        right = self._sector_clearance(
            scan, -math.pi / 2.0, self._escape_side_sector
        )
        direction = 1.0 if left >= right else -1.0
        attempts = []
        succeeded = False
        try:
            total_moved = 0.0
            while (
                total_moved < self._escape_strafe_max_total
                and total_moved < 2.0 * self._escape_strafe_distance
            ):
                moved = self._strafe_probe(direction, frame_id)
                attempts.append(moved)
                total_moved += moved
                if moved >= min(0.02, self._escape_strafe_distance):
                    succeeded = True
                if moved < min(0.02, self._escape_strafe_distance):
                    break
                with self._scan_lock:
                    fresh = self._latest_scan
                if fresh is None:
                    break
                fresh_front = self._sector_clearance(
                    fresh, 0.0, self._escape_scan_sector
                )
                fresh_rear = self._sector_clearance(
                    fresh, math.pi, self._escape_scan_sector
                )
                if not self._strafe_gate_open(fresh, fresh_front, fresh_rear):
                    break
        finally:
            self.stop()
        rospy.logwarn(
            "bounded lateral recovery completed: strafe=%s "
            "left_clearance=%s right_clearance=%s scale=%.1f wall=%.3fs",
            "|".join(
                "{:.3f}m".format(move) for move in attempts
            ),
            "unavailable" if not math.isfinite(left) else "{:.3f}".format(left),
            "unavailable" if not math.isfinite(right) else "{:.3f}".format(right),
            self._escape_distance_scale,
            self._escape_strafe_wall_timeout,
        )
        return succeeded

    def escape(self, frame_id):
        """Execute one bounded recovery (strafe or forward/back) under sole ownership.

        When both the front and rear corridors are tight but a side gap is
        large, the escape strafes along the base_link y-axis toward the larger
        gap (``recovery/strafe_*`` params).  Otherwise the preferred direction
        is the one with the larger measured sector clearance.  The escape
        probes that direction first; if the robot stops making progress for a
        short stall window (e.g. it is pressing into an obstacle the laser
        cannot see, like a jam cube below the scan plane), it switches to the
        opposite direction for the remaining wall budget.  Total motion stays
        bounded by ``recovery/max_distance`` and the whole escape by
        ``recovery/wall_timeout``.
        """
        if not self._escape_enabled:
            return False
        with self._scan_lock:
            scan = self._latest_scan
        front = rear = math.nan
        if scan is not None:
            front = self._sector_clearance(scan, 0.0, self._escape_scan_sector)
            rear = self._sector_clearance(scan, math.pi, self._escape_scan_sector)
        # When the robot is jammed nose-to-tail but a side opens up, a forward
        # or backward nudge can only press deeper into the dead-end.  Strafe
        # laterally instead so the fan sees a fresh, clear corridor.
        if (
            self._escape_strafe_enabled
            and scan is not None
            and self._strafe_gate_open(scan, front, rear)
        ):
            return self._strafe(scan, frame_id)
        direction = -1.0
        if math.isfinite(front) and math.isfinite(rear):
            direction = 1.0 if front > rear else -1.0
        elif math.isfinite(front):
            direction = 1.0

        # 70% of the chosen direction's clearance bounds each nudge; when the
        # clearance is unknown, fall back to the configured maximum so a missing
        # scan never produces an unbounded or zero-distance nudge.
        stall_timeout = min(0.5, self._escape_wall_timeout / 4.0)
        deadline = time.monotonic() + self._escape_wall_timeout
        attempts = []
        succeeded = False
        try:
            for probe in (direction, -direction):
                chosen_clearance = front if probe > 0.0 else rear
                max_distance = self._escape_max_distance_for(chosen_clearance)
                start_pose = self._localization.localized_pose(frame_id)
                if start_pose is None:
                    return False
                last_moved = 0.0
                last_progress_at = time.monotonic()
                moved = 0.0
                command = Twist()
                command.linear.x = probe * self._escape_speed
                while not rospy.is_shutdown():
                    if self._preempt_requested():
                        raise BaseAlignmentPreempted(
                            "task preempted during bounded navigation recovery"
                        )
                    current = self._localization.localized_pose(frame_id)
                    if current is not None:
                        moved = math.hypot(
                            current[0] - start_pose[0],
                            current[1] - start_pose[1],
                        )
                    now = time.monotonic()
                    if moved > last_moved:
                        last_moved = moved
                        last_progress_at = now
                    if (
                        moved >= max_distance
                        or now >= deadline
                        or now - last_progress_at >= stall_timeout
                    ):
                        break
                    self._velocity_publisher.publish(command)
                    time.sleep(0.05)
                attempts.append(
                    {
                        "direction": "forward" if probe > 0.0 else "backward",
                        "moved": moved,
                        "target": max_distance,
                    }
                )
                if moved >= min(0.02, self._escape_max_distance):
                    succeeded = True
                    break
        finally:
            self.stop()
        rospy.logwarn(
            "bounded navigation recovery completed: probes=%s "
            "front_clearance=%s rear_clearance=%s scale=%.1f wall=%.3fs",
            "|".join(
                "{}{:.3f}m".format(attempt["direction"], attempt["moved"])
                for attempt in attempts
            ),
            "unavailable" if not math.isfinite(front) else "{:.3f}".format(front),
            "unavailable" if not math.isfinite(rear) else "{:.3f}".format(rear),
            self._escape_distance_scale,
            time.monotonic() - deadline + self._escape_wall_timeout,
        )
        return succeeded

    @staticmethod
    def _quaternion_yaw(quaternion):
        return math.atan2(
            2.0
            * (
                quaternion.w * quaternion.z
                + quaternion.x * quaternion.y
            ),
            1.0
            - 2.0
            * (
                quaternion.y * quaternion.y
                + quaternion.z * quaternion.z
            ),
        )

    @staticmethod
    def _shortest_angular_distance(first, second):
        return math.atan2(
            math.sin(second - first),
            math.cos(second - first),
        )

    def heading_error(self, waypoint):
        """Return the shortest signed yaw error for ``waypoint``."""
        frame_id = waypoint.header.frame_id or self._localization.map_frame
        current = self._localization.localized_pose(frame_id)
        if current is None:
            return None
        target_yaw = self._quaternion_yaw(waypoint.pose.orientation)
        return self._shortest_angular_distance(current[2], target_yaw)

    def heading_command(self, yaw_error):
        """Convert a yaw error to the configured bounded angular velocity."""
        command = self._heading_gain * yaw_error
        command = max(
            -self._heading_max_speed,
            min(self._heading_max_speed, command),
        )
        if command == 0.0:
            return 0.0
        if abs(command) < self._heading_min_speed:
            return math.copysign(self._heading_min_speed, command)
        return command

    # Compatibility aliases for callers that used the extracted implementation
    # before its public navigation API was named.
    _heading_error = heading_error
    _heading_command = heading_command

    def align_heading(self, waypoint, heartbeat):
        """Rotate to a waypoint heading after RouteExecutor cancels move_base."""
        deadline = rospy.Time.now() + rospy.Duration(self._heading_timeout)
        rate = rospy.Rate(10.0)
        outcome = NavigationOutcome.TIMEOUT
        message = "intermediate heading alignment timed out"
        try:
            while not rospy.is_shutdown():
                if self._preempt_requested():
                    outcome = NavigationOutcome.PREEMPTED
                    message = "task was preempted during heading alignment"
                    break
                yaw_error = self.heading_error(waypoint)
                if yaw_error is not None:
                    if abs(yaw_error) <= self._heading_tolerance:
                        outcome = NavigationOutcome.SUCCEEDED
                        message = "intermediate waypoint heading aligned"
                        break
                    command = Twist()
                    command.angular.z = self.heading_command(yaw_error)
                    self._velocity_publisher.publish(command)
                heartbeat()
                if rospy.Time.now() >= deadline:
                    break
                rate.sleep()
        finally:
            self._velocity_publisher.publish(Twist())
        return outcome, message

    def align(self, context, state_machine, pose, detail):
        """Compatibility adapter for context/state-machine based callers."""
        context.retry_count = 0
        state_machine.transition(
            states.ALIGN_FOR_GRASP,
            detail + "; direct low-speed translation",
        )
        self.align_to_pose(
            pose,
            feedback=lambda residual: self._publish_state(
                context,
                detail + "; direct residual {:.3f}m".format(residual),
            ),
        )

    def align_to_pose(self, pose, feedback=None):
        """Translate without changing heading and always command stop.

        ``feedback`` receives the current residual distance. This is the
        package-neutral API used by new callers; ``align`` remains as a narrow
        adapter while mission integration is migrated.
        """
        frame_id = pose.header.frame_id or self._localization.map_frame
        started_wall = time.monotonic()
        wall_deadline = started_wall + self._wall_timeout
        started_ros = rospy.Time.now()
        sim_deadline = started_ros + rospy.Duration(self._timeout)
        last_clock = started_ros
        last_clock_progress_wall = started_wall
        next_feedback = 0.0
        try:
            while not rospy.is_shutdown():
                if self._preempt_requested():
                    raise BaseAlignmentPreempted(
                        "task preempted during fixed-standoff alignment"
                    )
                current = self._localization.localized_pose(frame_id)
                if current is None:
                    raise BaseAlignmentFailure(
                        error_codes.ALIGNMENT_FAILED,
                        "localized base pose is unavailable during direct alignment",
                    )
                error_x = pose.pose.position.x - current[0]
                error_y = pose.pose.position.y - current[1]
                distance = math.hypot(error_x, error_y)
                if distance <= self._tolerance:
                    rospy.loginfo(
                        "direct grasp alignment reached %.4fm residual",
                        distance,
                    )
                    return
                now_wall = time.monotonic()
                now_ros = rospy.Time.now()
                if now_ros > last_clock:
                    last_clock = now_ros
                    last_clock_progress_wall = now_wall
                timeout_reason = None
                if not started_ros.is_zero() and now_ros >= sim_deadline:
                    timeout_reason = "simulation-time deadline"
                elif now_wall >= wall_deadline:
                    timeout_reason = "wall-clock hard deadline"
                elif (
                    now_wall - last_clock_progress_wall
                    >= self._alignment_clock_stall_timeout
                ):
                    timeout_reason = "simulation clock stalled"
                if timeout_reason is not None:
                    raise BaseAlignmentFailure(
                        error_codes.ALIGNMENT_FAILED,
                        "direct fixed-standoff alignment timed out at {:.3f}m "
                        "({}; sim_elapsed={:.3f}s wall_elapsed={:.3f}s)".format(
                            distance,
                            timeout_reason,
                            max(0.0, (now_ros - started_ros).to_sec()),
                            now_wall - started_wall,
                        ),
                    )

                cosine = math.cos(current[2])
                sine = math.sin(current[2])
                error_forward = cosine * error_x + sine * error_y
                error_lateral = -sine * error_x + cosine * error_y
                speed = min(
                    self._max_speed,
                    max(self._min_speed, self._gain * distance),
                )
                command = Twist()
                command.linear.x = speed * error_forward / distance
                command.linear.y = speed * error_lateral / distance
                self._velocity_publisher.publish(command)

                if feedback is not None and now_wall >= next_feedback:
                    feedback(distance)
                    next_feedback = now_wall + 0.5
                time.sleep(0.05)
        finally:
            self._velocity_publisher.publish(Twist())
