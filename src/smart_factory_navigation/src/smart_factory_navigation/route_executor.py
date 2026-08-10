"""Navigation-owned route execution built on the shared ``move_base`` client."""

import math
import time

from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as NavigationPath
import rospy
from std_msgs.msg import Float64

from smart_factory_navigation import error_codes, states
from smart_factory_navigation.base_alignment_controller import (
    BaseAlignmentController,
)
from smart_factory_navigation.navigation_stage import (
    NavigationOutcome,
    NavigationStage,
)
from smart_factory_navigation.path_tracker import (
    FittedPath,
    PathConfigError,
    PathTracker,
)


class RouteNavigationFailure(RuntimeError):
    def __init__(self, error_code, message):
        super().__init__(message)
        self.error_code = error_code


class RouteNavigationPreempted(RuntimeError):
    pass


class RouteExecutor:
    """Execute legacy waypoint and fitted-path routes for an orchestration client."""

    SERVER_WAIT_POLL_PERIOD = 0.1

    ACTIVE_NAVIGATION_STATES = (
        GoalStatus.PENDING,
        GoalStatus.ACTIVE,
        GoalStatus.PREEMPTING,
        GoalStatus.RECALLING,
    )

    def __init__(
        self,
        localization,
        publish_state,
        abort,
        preempt,
        preempt_requested,
        progress_callback=None,
    ):
        self._localization = localization
        self._publish_state = publish_state
        self._abort = abort
        self._preempt = preempt
        self._preempt_requested = preempt_requested
        self._progress_callback = progress_callback or (lambda _progress: None)

        action_name = rospy.get_param(
            "~navigation/action_name", "/move_base"
        )
        self._server_wait_timeout = rospy.get_param(
            "~navigation/server_wait_timeout", 10.0
        )
        goal_timeout = rospy.get_param(
            "~navigation/goal_timeout", 60.0
        )
        self._max_retries = int(
            rospy.get_param("~navigation/max_retries", 1)
        )

        self._intermediate_pass_radius = float(
            rospy.get_param(
                "~navigation/intermediate_pass_radius", 0.20
            )
        )
        if self._intermediate_pass_radius < 0.0:
            raise ValueError(
                "navigation/intermediate_pass_radius must not be negative"
            )
        constrained_waypoints = rospy.get_param(
            "~navigation/heading_constrained_waypoints", []
        )
        if not isinstance(constrained_waypoints, list):
            raise ValueError(
                "navigation/heading_constrained_waypoints must be a list"
            )
        if any(
            isinstance(number, bool)
            or not isinstance(number, int)
            or number <= 0
            for number in constrained_waypoints
        ):
            raise ValueError(
                "navigation/heading_constrained_waypoints must contain "
                "positive waypoint numbers"
            )
        self._heading_constrained_waypoints = set(constrained_waypoints)
        self._route_execution_mode = rospy.get_param(
            "~navigation/route_execution_mode", "legacy_waypoints"
        )
        if self._route_execution_mode not in (
            "legacy_waypoints",
            "fitted_path_lookahead",
        ):
            raise ValueError(
                "navigation/route_execution_mode must be legacy_waypoints "
                "or fitted_path_lookahead"
            )
        self._fitted_path = None
        if self._route_execution_mode == "fitted_path_lookahead":
            try:
                self._fitted_path = FittedPath.from_config(
                    rospy.get_param("~fitted_path", {})
                )
            except PathConfigError as exc:
                raise ValueError("invalid fitted path: {}".format(exc))

        self._load_path_tracking_parameters()
        self._base_alignment = BaseAlignmentController(
            localization=self._localization,
            publish_state=self._publish_state,
            preempt_requested=self._preempt_requested,
        )
        self._navigation = NavigationStage(action_name, goal_timeout)

        self._reference_path_publisher = rospy.Publisher(
            self._path_topic, NavigationPath, queue_size=1, latch=True
        )
        self._tracking_goal_publisher = rospy.Publisher(
            self._tracking_goal_topic, PoseStamped, queue_size=1
        )
        self._path_progress_publisher = rospy.Publisher(
            self._progress_topic, Float64, queue_size=10
        )

    def wait_for_server(self):
        deadline = time.monotonic() + float(self._server_wait_timeout)
        while not rospy.is_shutdown():
            if self._preempt_requested():
                return False
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return False
            if self._navigation.wait_for_server(
                min(self.SERVER_WAIT_POLL_PERIOD, remaining)
            ):
                return True
        return False

    def cancel(self):
        """Stop any active move_base goal and direct base command."""
        self._navigation.cancel_goal()
        self._base_alignment.stop()

    @property
    def route_execution_mode(self):
        return self._route_execution_mode

    def _load_path_tracking_parameters(self):
        namespace = "~navigation/path_tracking/"
        self._path_topic = rospy.get_param(
            namespace + "path_topic", "/sim_task/reference_path"
        )
        self._tracking_goal_topic = rospy.get_param(
            namespace + "tracking_goal_topic", "/sim_task/tracking_goal"
        )
        self._progress_topic = rospy.get_param(
            namespace + "progress_topic", "/sim_task/path_progress"
        )
        self._path_control_frequency = float(
            rospy.get_param(namespace + "control_frequency", 20.0)
        )
        self._lookahead_min = float(
            rospy.get_param(namespace + "lookahead_min", 0.18)
        )
        self._lookahead_max = float(
            rospy.get_param(namespace + "lookahead_max", 0.55)
        )
        self._lookahead_curvature_gain = float(
            rospy.get_param(namespace + "lookahead_curvature_gain", 0.15)
        )
        self._projection_window = float(
            rospy.get_param(namespace + "projection_window", 1.0)
        )
        self._path_acquire_radius = float(
            rospy.get_param(namespace + "acquire_radius", 0.20)
        )
        self._goal_update_distance = float(
            rospy.get_param(namespace + "goal_update_distance", 0.18)
        )
        self._goal_update_period = float(
            rospy.get_param(namespace + "goal_update_period", 1.0)
        )
        self._cross_track_warn = float(
            rospy.get_param(namespace + "cross_track_warn", 0.12)
        )
        self._cross_track_abort = float(
            rospy.get_param(namespace + "cross_track_abort", 0.25)
        )
        self._progress_timeout = float(
            rospy.get_param(namespace + "progress_timeout", 15.0)
        )
        self._progress_epsilon = float(
            rospy.get_param(namespace + "progress_epsilon", 0.05)
        )
        self._final_phase_distance = float(
            rospy.get_param(namespace + "final_phase_distance", 0.30)
        )
        self._route_timeout = float(
            rospy.get_param(namespace + "route_timeout", 240.0)
        )

        raw_direct_segments = rospy.get_param(
            namespace + "direct_segments", []
        )
        if not isinstance(raw_direct_segments, list):
            raise ValueError(
                "navigation/path_tracking/direct_segments must be a list"
            )
        self._direct_segments = []
        for segment in raw_direct_segments:
            if (
                not isinstance(segment, list)
                or len(segment) != 2
                or any(
                    isinstance(value, bool)
                    or not isinstance(value, int)
                    or value <= 0
                    for value in segment
                )
            ):
                raise ValueError(
                    "each direct segment must contain two positive seq integers"
                )
            self._direct_segments.append(tuple(segment))

        raw_heading_locks = rospy.get_param(namespace + "heading_locks", [])
        if not isinstance(raw_heading_locks, list):
            raise ValueError(
                "navigation/path_tracking/heading_locks must be a list"
            )
        self._heading_locks = []
        lock_keys = (
            "start_seq",
            "full_lock_seq",
            "end_seq",
            "direction_start_seq",
            "direction_end_seq",
        )
        for lock in raw_heading_locks:
            if not isinstance(lock, dict):
                raise ValueError("each heading lock must be a mapping")
            sequences = [lock.get(key) for key in lock_keys]
            if any(
                isinstance(value, bool)
                or not isinstance(value, int)
                or value <= 0
                for value in sequences
            ):
                raise ValueError(
                    "heading lock seq fields must be positive integers"
                )
            release_distance = lock.get("release_distance")
            if isinstance(release_distance, bool) or not isinstance(
                release_distance, (int, float)
            ):
                raise ValueError(
                    "heading lock release_distance must be numeric"
                )
            self._heading_locks.append(
                tuple(sequences) + (float(release_distance),)
            )

        positive = {
            "control_frequency": self._path_control_frequency,
            "lookahead_min": self._lookahead_min,
            "lookahead_max": self._lookahead_max,
            "projection_window": self._projection_window,
            "acquire_radius": self._path_acquire_radius,
            "goal_update_distance": self._goal_update_distance,
            "goal_update_period": self._goal_update_period,
            "cross_track_warn": self._cross_track_warn,
            "cross_track_abort": self._cross_track_abort,
            "progress_timeout": self._progress_timeout,
            "progress_epsilon": self._progress_epsilon,
            "final_phase_distance": self._final_phase_distance,
            "route_timeout": self._route_timeout,
        }
        for name, value in positive.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(
                    "navigation/path_tracking/{} must be positive".format(name)
                )
        if self._lookahead_max < self._lookahead_min:
            raise ValueError(
                "path lookahead_max must be no smaller than lookahead_min"
            )
        if self._cross_track_abort <= self._cross_track_warn:
            raise ValueError(
                "cross_track_abort must be greater than cross_track_warn"
            )
        if (
            not math.isfinite(self._lookahead_curvature_gain)
            or self._lookahead_curvature_gain < 0.0
        ):
            raise ValueError(
                "lookahead_curvature_gain must not be negative"
            )

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

    @staticmethod
    def _make_pose(frame_id, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _publish_fitted_reference_path(self):
        message = NavigationPath()
        message.header.frame_id = self._fitted_path.frame_id
        message.header.stamp = rospy.Time.now()
        for point in self._fitted_path.points:
            pose = self._make_pose(
                self._fitted_path.frame_id,
                point.x,
                point.y,
                point.yaw,
            )
            pose.header.stamp = message.header.stamp
            message.poses.append(pose)
        self._reference_path_publisher.publish(message)

    def waypoint_is_passed(self, waypoint):
        """Return whether localization is inside the configured pass radius."""
        if self._intermediate_pass_radius <= 0.0:
            return False
        return self._localization.pose_is_within_radius(
            waypoint, self._intermediate_pass_radius
        )

    _intermediate_waypoint_is_passed = waypoint_is_passed

    def _align_intermediate_waypoint_heading(self, waypoint, heartbeat):
        self._navigation.cancel_goal()
        rospy.sleep(0.2)
        return self._base_alignment.align_heading(waypoint, heartbeat)

    def _fitted_path_matches_route(self, goals):
        if not goals:
            return False
        final = goals[-1]
        expected_x, expected_y, expected_yaw = self._fitted_path.final_goal
        actual_yaw = self._quaternion_yaw(final.pose.orientation)
        return (
            math.hypot(
                final.pose.position.x - expected_x,
                final.pose.position.y - expected_y,
            )
            <= 1.0e-4
            and abs(
                self._shortest_angular_distance(actual_yaw, expected_yaw)
            )
            <= 1.0e-4
        )

    def _pose_is_within_tolerances(
        self, pose, position_tolerance, yaw_tolerance
    ):
        frame_id = pose.header.frame_id or self._localization.map_frame
        localized = self._localization.localized_pose(frame_id)
        if localized is None:
            return False
        position_error = math.hypot(
            pose.pose.position.x - localized[0],
            pose.pose.position.y - localized[1],
        )
        target_yaw = self._quaternion_yaw(pose.pose.orientation)
        yaw_error = abs(
            self._shortest_angular_distance(localized[2], target_yaw)
        )
        comparison_epsilon = 1.0e-12
        return (
            position_error <= position_tolerance + comparison_epsilon
            and yaw_error <= yaw_tolerance + comparison_epsilon
        )

    def navigate_pose(
        self,
        context,
        state_machine,
        pose,
        stage,
        detail,
        position_tolerance=0.0,
        yaw_tolerance=0.0,
    ):
        """Navigate to one task-selected pose with mission retry semantics."""
        if not all(
            math.isfinite(value) and value >= 0.0
            for value in (position_tolerance, yaw_tolerance)
        ):
            raise ValueError(
                "navigation pose tolerances must be finite and nonnegative"
            )
        tolerances_enabled = position_tolerance > 0.0 and yaw_tolerance > 0.0
        if (position_tolerance > 0.0) != (yaw_tolerance > 0.0):
            raise ValueError(
                "position_tolerance and yaw_tolerance must both be positive "
                "or both be zero"
            )
        pass_condition = (
            lambda: self._pose_is_within_tolerances(
                pose, position_tolerance, yaw_tolerance
            )
            if tolerances_enabled
            else None
        )
        last_outcome = None
        last_message = "navigation was not attempted"
        for retry in range(self._max_retries + 1):
            context.retry_count = retry
            state_machine.transition(
                stage,
                "{} (attempt {}/{})".format(
                    detail, retry + 1, self._max_retries + 1
                ),
            )
            last_outcome, last_message = self._navigation.navigate(
                pose,
                self._preempt_requested,
                lambda: self._publish_state(
                    context, detail + "; move_base active"
                ),
                pass_condition=pass_condition,
            )
            if last_outcome in (
                NavigationOutcome.SUCCEEDED,
                NavigationOutcome.PASSED,
            ):
                if last_outcome == NavigationOutcome.PASSED:
                    self._navigation.cancel_goal()
                context.retry_count = 0
                return
            if last_outcome == NavigationOutcome.PREEMPTED:
                raise RouteNavigationPreempted(
                    detail + ": " + last_message
                )
        error_code = (
            error_codes.NAVIGATION_TIMEOUT
            if last_outcome == NavigationOutcome.TIMEOUT
            else error_codes.NAVIGATION_ABORTED
        )
        raise RouteNavigationFailure(
            error_code,
            "{} failed: {}".format(detail, last_message),
        )

    def align_for_grasp(
        self, context, state_machine, pose, detail
    ):
        """Coordinate move_base handoff before direct grasp alignment."""
        if self._navigation.get_state() in self.ACTIVE_NAVIGATION_STATES:
            self._navigation.cancel_goal()
        self._base_alignment.align(
            context,
            state_machine,
            pose,
            detail,
        )

    def execute_staging_route(self, context, state_machine):
        if not context.pickup_staging_goals:
            raise RouteNavigationFailure(
                error_codes.GOAL_UNAVAILABLE,
                "staging route contains no waypoints",
            )
        if self._route_execution_mode == "fitted_path_lookahead":
            rospy.loginfo(
                "task=%s executing offline fitted path with moving lookahead",
                context.task_id,
            )
            if not self._execute_fitted_path(context, state_machine):
                return None
            return "fitted path completed; arrived at pickup staging area"
        return self._execute_legacy_waypoints(context, state_machine)

    def _execute_legacy_waypoints(self, context, state_machine):
        waypoint_count = len(context.pickup_staging_goals)
        for waypoint_index, waypoint in enumerate(
            context.pickup_staging_goals
        ):
            context.current_waypoint_index = waypoint_index
            context.retry_count = 0
            waypoint_number = waypoint_index + 1
            is_final_waypoint = waypoint_number == waypoint_count
            requires_intermediate_heading = (
                waypoint_number in self._heading_constrained_waypoints
            )
            while context.retry_count <= self._max_retries:
                state_machine.transition(
                    states.NAVIGATE_TO_PICKUP_STAGING,
                    "sending waypoint {}/{} to move_base".format(
                        waypoint_number, waypoint_count
                    ),
                )
                pass_condition = None
                if not is_final_waypoint:
                    pass_condition = (
                        lambda waypoint=waypoint:
                        self.waypoint_is_passed(waypoint)
                    )
                outcome, message = self._navigation.navigate(
                    waypoint,
                    self._preempt_requested,
                    lambda waypoint_number=waypoint_number: self._publish_state(
                        context,
                        "waypoint {}/{} move_base goal is active".format(
                            waypoint_number, waypoint_count
                        ),
                    ),
                    pass_condition=pass_condition,
                )

                if (
                    outcome == NavigationOutcome.PASSED
                    and requires_intermediate_heading
                ):
                    state_machine.transition(
                        states.ALIGN_HEADING,
                        "waypoint {}/{} position reached; aligning heading".format(
                            waypoint_number, waypoint_count
                        ),
                    )
                    self._publish_state(
                        context,
                        "waypoint {}/{} position reached; aligning heading".format(
                            waypoint_number, waypoint_count
                        ),
                    )
                    outcome, message = self._align_intermediate_waypoint_heading(
                        waypoint,
                        lambda waypoint_number=waypoint_number:
                        self._publish_state(
                            context,
                            "waypoint {}/{} aligning heading".format(
                                waypoint_number, waypoint_count
                            ),
                        ),
                    )
                    if outcome == NavigationOutcome.SUCCEEDED:
                        outcome = NavigationOutcome.PASSED

                if outcome in (
                    NavigationOutcome.SUCCEEDED,
                    NavigationOutcome.PASSED,
                ):
                    if outcome == NavigationOutcome.SUCCEEDED:
                        waypoint_status = "goal reached"
                    elif requires_intermediate_heading:
                        waypoint_status = (
                            "passed within {:.2f} m and {:.2f} rad heading "
                            "tolerance; advancing"
                        ).format(
                            self._intermediate_pass_radius,
                            self._base_alignment.heading_tolerance,
                        )
                    else:
                        waypoint_status = (
                            "passed within {:.2f} m; advancing without "
                            "final orientation alignment"
                        ).format(self._intermediate_pass_radius)
                    self._publish_state(
                        context,
                        "waypoint {}/{} {}".format(
                            waypoint_number,
                            waypoint_count,
                            waypoint_status,
                        ),
                    )
                    break

                waypoint_message = "waypoint {}/{}: {}".format(
                    waypoint_number, waypoint_count, message
                )
                if outcome == NavigationOutcome.PREEMPTED:
                    self._preempt(
                        context, state_machine, waypoint_message
                    )
                    return None
                if context.retry_count < self._max_retries:
                    context.retry_count += 1
                    self._publish_state(
                        context, "retrying " + waypoint_message
                    )
                    continue
                error_code = (
                    error_codes.NAVIGATION_TIMEOUT
                    if outcome == NavigationOutcome.TIMEOUT
                    else error_codes.NAVIGATION_ABORTED
                )
                self._abort(
                    context,
                    state_machine,
                    error_code,
                    waypoint_message,
                )
                return None
            else:
                self._abort(
                    context,
                    state_machine,
                    error_codes.INTERNAL_ERROR,
                    "waypoint {}/{} retry loop ended unexpectedly".format(
                        waypoint_number, waypoint_count
                    ),
                )
                return None

        message = "all {} waypoints reached; arrived at pickup staging area".format(
            waypoint_count
        )
        state_machine.transition(states.ARRIVED_PICKUP_STAGING, message)
        return message

    def _execute_fitted_path(self, context, state_machine):
        """Follow the offline spline through low-rate moving move_base goals."""
        if not self._fitted_path_matches_route(context.pickup_staging_goals):
            self._abort(
                context,
                state_machine,
                error_codes.GOAL_UNAVAILABLE,
                "fitted path is stale: regenerate it from the active route",
            )
            return None

        try:
            tracker = PathTracker(
                self._fitted_path,
                self._lookahead_min,
                self._lookahead_max,
                self._lookahead_curvature_gain,
                self._projection_window,
                self._direct_segments,
                self._heading_locks,
            )
        except PathConfigError as exc:
            self._abort(
                context,
                state_machine,
                error_codes.GOAL_UNAVAILABLE,
                "invalid fitted-path tracker configuration: {}".format(exc),
            )
            return None

        self._publish_fitted_reference_path()
        first_point = self._fitted_path.points[0]
        first_pose = self._make_pose(
            self._fitted_path.frame_id,
            first_point.x,
            first_point.y,
            first_point.yaw,
        )

        acquired = False
        acquisition_message = ""
        outcome = NavigationOutcome.ABORTED
        for retry in range(self._max_retries + 1):
            context.retry_count = retry
            state_machine.transition(
                states.ACQUIRE_ROUTE,
                "acquiring fitted path start (attempt {}/{})".format(
                    retry + 1, self._max_retries + 1
                ),
            )
            outcome, acquisition_message = self._navigation.navigate(
                first_pose,
                self._preempt_requested,
                lambda: self._publish_state(
                    context, "move_base is acquiring the fitted path start"
                ),
                pass_condition=lambda: self._localization.pose_is_within_radius(
                    first_pose, self._path_acquire_radius
                ),
            )
            if outcome in (
                NavigationOutcome.SUCCEEDED,
                NavigationOutcome.PASSED,
            ):
                acquired = True
                break
            if outcome == NavigationOutcome.PREEMPTED:
                self._preempt(
                    context,
                    state_machine,
                    "task preempted while acquiring fitted path",
                )
                return None
        if not acquired:
            error_code = (
                error_codes.NAVIGATION_TIMEOUT
                if outcome == NavigationOutcome.TIMEOUT
                else error_codes.NAVIGATION_ABORTED
            )
            self._abort(
                context,
                state_machine,
                error_code,
                "failed to acquire fitted path: {}".format(acquisition_message),
            )
            return None

        context.retry_count = 0
        state_machine.transition(
            states.FOLLOW_ROUTE,
            "fitted path acquired; starting moving-lookahead navigation",
        )

        route_started = time.monotonic()
        progress_checked_at = route_started
        checked_progress = 0.0
        heartbeat_at = 0.0
        last_goal_s = None
        last_goal_at = None
        consecutive_goal_failures = 0
        rate = rospy.Rate(self._path_control_frequency)

        while not rospy.is_shutdown():
            now = time.monotonic()
            if self._preempt_requested():
                self._navigation.cancel_goal()
                self._preempt(
                    context,
                    state_machine,
                    "task preempted during fitted-path navigation",
                )
                return None
            if now - route_started >= self._route_timeout:
                self._navigation.cancel_goal()
                self._abort(
                    context,
                    state_machine,
                    error_codes.NAVIGATION_TIMEOUT,
                    "fitted-path navigation exceeded {:.1f}s".format(
                        self._route_timeout
                    ),
                )
                return None

            localized = self._localization.localized_xy(
                self._fitted_path.frame_id
            )
            if localized is None:
                rate.sleep()
                continue
            tracking = tracker.update(localized[0], localized[1])
            self._path_progress_publisher.publish(
                Float64(data=tracking.progress_s)
            )
            self._progress_callback(tracking.progress_s)

            if tracking.cross_track_error >= self._cross_track_abort:
                self._navigation.cancel_goal()
                self._abort(
                    context,
                    state_machine,
                    error_codes.NAVIGATION_ABORTED,
                    "cross-track error {:.3f}m exceeds {:.3f}m limit at "
                    "s={:.3f}m".format(
                        tracking.cross_track_error,
                        self._cross_track_abort,
                        tracking.progress_s,
                    ),
                )
                return None
            if tracking.cross_track_error >= self._cross_track_warn:
                rospy.logwarn_throttle(
                    1.0,
                    "fitted-path cross-track error is %.3fm at s=%.3fm",
                    tracking.cross_track_error,
                    tracking.progress_s,
                )

            if tracking.progress_s >= checked_progress + self._progress_epsilon:
                checked_progress = tracking.progress_s
                progress_checked_at = now
                consecutive_goal_failures = 0
            elif now - progress_checked_at >= self._progress_timeout:
                self._navigation.cancel_goal()
                self._abort(
                    context,
                    state_machine,
                    error_codes.NAVIGATION_TIMEOUT,
                    "fitted path made less than {:.2f}m progress in {:.1f}s "
                    "at s={:.3f}m".format(
                        self._progress_epsilon,
                        self._progress_timeout,
                        tracking.progress_s,
                    ),
                )
                return None

            remaining = self._fitted_path.total_length - tracking.progress_s
            if remaining <= self._final_phase_distance:
                break

            state = self._navigation.get_state()
            update_due = (
                last_goal_s is None
                or tracking.target.s >= last_goal_s + self._goal_update_distance
                or (
                    last_goal_at is not None
                    and now - last_goal_at >= self._goal_update_period
                )
                or state == GoalStatus.SUCCEEDED
            )
            if update_due:
                target_pose = self._make_pose(
                    self._fitted_path.frame_id,
                    tracking.target.x,
                    tracking.target.y,
                    tracking.target.yaw,
                )
                self._navigation.send_or_replace_goal(target_pose)
                self._tracking_goal_publisher.publish(target_pose)
                last_goal_s = tracking.target.s
                last_goal_at = now
                state = self._navigation.get_state()

            if state in (
                GoalStatus.PREEMPTED,
                GoalStatus.ABORTED,
                GoalStatus.REJECTED,
                GoalStatus.RECALLED,
                GoalStatus.LOST,
            ):
                if consecutive_goal_failures < self._max_retries:
                    consecutive_goal_failures += 1
                    context.retry_count = consecutive_goal_failures
                    retry_pose = self._make_pose(
                        self._fitted_path.frame_id,
                        tracking.target.x,
                        tracking.target.y,
                        tracking.target.yaw,
                    )
                    self._navigation.send_or_replace_goal(retry_pose)
                    self._tracking_goal_publisher.publish(retry_pose)
                    last_goal_s = tracking.target.s
                    last_goal_at = now
                    self._publish_state(
                        context,
                        "retrying moving target after move_base state {}".format(
                            state
                        ),
                    )
                else:
                    self._abort(
                        context,
                        state_machine,
                        error_codes.NAVIGATION_ABORTED,
                        "moving lookahead goal failed with move_base state "
                        "{} at s={:.3f}m".format(state, tracking.progress_s),
                    )
                    return None

            if now - heartbeat_at >= 1.0:
                heartbeat_at = now
                self._publish_state(
                    context,
                    "tracking fitted path s={:.2f}/{:.2f}m, error={:.3f}m, "
                    "lookahead={:.2f}m".format(
                        tracking.progress_s,
                        self._fitted_path.total_length,
                        tracking.cross_track_error,
                        tracking.lookahead,
                    ),
                )
            rate.sleep()

        if rospy.is_shutdown():
            self._navigation.cancel_goal()
            return None

        final_goal = context.pickup_staging_goals[-1]
        final_outcome = None
        final_message = ""
        state_machine.transition(
            states.FINAL_GOAL,
            "entering exact final staging goal phase",
        )
        for retry in range(self._max_retries + 1):
            context.retry_count = retry
            self._publish_state(
                context,
                "sending exact final staging pose (attempt {}/{})".format(
                    retry + 1, self._max_retries + 1
                ),
            )
            final_outcome, final_message = self._navigation.navigate(
                final_goal,
                self._preempt_requested,
                lambda: self._publish_state(
                    context, "exact final staging goal is active"
                ),
            )
            if final_outcome == NavigationOutcome.SUCCEEDED:
                break
            if final_outcome == NavigationOutcome.PREEMPTED:
                self._preempt(
                    context,
                    state_machine,
                    "task preempted during exact final staging goal",
                )
                return None
        if final_outcome != NavigationOutcome.SUCCEEDED:
            error_code = (
                error_codes.NAVIGATION_TIMEOUT
                if final_outcome == NavigationOutcome.TIMEOUT
                else error_codes.NAVIGATION_ABORTED
            )
            self._abort(
                context,
                state_machine,
                error_code,
                "exact final staging goal failed: {}".format(final_message),
            )
            return None

        state_machine.transition(
            states.ARRIVED_PICKUP_STAGING,
            "fitted path completed; arrived at pickup staging area",
        )
        return True
