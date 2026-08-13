"""Navigation-owned route execution built on the shared ``move_base`` client."""

import math
import time

from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as NavigationPath
import rospy

from smart_factory_navigation import error_codes, states
from smart_factory_navigation.base_alignment_controller import (
    BaseAlignmentController,
)
from smart_factory_navigation.navigation_stage import (
    NavigationOutcome,
    NavigationStage,
)
from smart_factory_navigation.fitted_path import FittedPath, PathConfigError


class RouteNavigationFailure(RuntimeError):
    def __init__(self, error_code, message):
        super().__init__(message)
        self.error_code = error_code


class RouteNavigationPreempted(RuntimeError):
    pass


class RouteExecutor:
    """Execute the offline-generated waypoints through one move_base client."""

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
        # Retain compatibility with the current navigation Action server. The
        # sequential executor reports waypoint indices instead of arc progress.
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
        self._final_pass_radius = float(
            rospy.get_param("~navigation/final_pass_radius", 0.0)
        )
        if self._final_pass_radius < 0.0:
            raise ValueError(
                "navigation/final_pass_radius must not be negative"
            )
        self._final_yaw_tolerance = float(
            rospy.get_param("~navigation/final_yaw_tolerance", 0.04)
        )
        if (
            not math.isfinite(self._final_yaw_tolerance)
            or self._final_yaw_tolerance <= 0.0
        ):
            raise ValueError(
                "navigation/final_yaw_tolerance must be positive"
            )
        self._fitted_waypoint_count = rospy.get_param(
            "~navigation/fitted_waypoints/count", 30
        )
        if (
            isinstance(self._fitted_waypoint_count, bool)
            or not isinstance(self._fitted_waypoint_count, int)
            or self._fitted_waypoint_count < 2
        ):
            raise ValueError(
                "navigation/fitted_waypoints/count must be an integer "
                "greater than one"
            )
        try:
            self._fitted_path = FittedPath.from_config(
                rospy.get_param("~fitted_path", {})
            )
        except PathConfigError as exc:
            raise ValueError("invalid fitted path: {}".format(exc))
        if (
            len(self._fitted_path.execution_waypoints)
            != self._fitted_waypoint_count
        ):
            raise ValueError(
                "fitted path contains {} execution waypoints; expected {}".format(
                    len(self._fitted_path.execution_waypoints),
                    self._fitted_waypoint_count,
                )
            )
        orientation_required_sequences = rospy.get_param(
            "~navigation/fitted_waypoints/orientation_required_sequences", []
        )
        if not isinstance(orientation_required_sequences, list) or any(
            isinstance(sequence, bool)
            or not isinstance(sequence, int)
            or sequence <= 0
            for sequence in orientation_required_sequences
        ):
            raise ValueError(
                "navigation/fitted_waypoints/orientation_required_sequences "
                "must contain positive integers"
            )
        if len(set(orientation_required_sequences)) != len(
            orientation_required_sequences
        ):
            raise ValueError(
                "navigation/fitted_waypoints/orientation_required_sequences "
                "must not contain duplicates"
            )
        self._orientation_yaw_tolerance = float(
            rospy.get_param(
                "~navigation/fitted_waypoints/orientation_yaw_tolerance", 0.15
            )
        )
        if not 0.0 < self._orientation_yaw_tolerance <= math.pi:
            raise ValueError(
                "navigation/fitted_waypoints/orientation_yaw_tolerance must "
                "be in (0, pi]"
            )
        source_sequences = [
            waypoint.source_seq
            for waypoint in self._fitted_path.execution_waypoints
        ]
        self._orientation_required_waypoint_indices = set()
        for sequence in orientation_required_sequences:
            matching_indices = [
                index
                for index, source_sequence in enumerate(source_sequences)
                if source_sequence == sequence
            ]
            if len(matching_indices) != 1:
                raise ValueError(
                    "orientation-required seq {} must identify exactly one "
                    "fitted execution waypoint; regenerate the fitted path".format(
                        sequence
                    )
                )
            self._orientation_required_waypoint_indices.add(
                matching_indices[0]
            )

        self._base_alignment = BaseAlignmentController(
            localization=self._localization,
            publish_state=self._publish_state,
            preempt_requested=self._preempt_requested,
        )
        self._navigation = NavigationStage(action_name, goal_timeout)

        self._reference_path_publisher = rospy.Publisher(
            "/sim_task/reference_path",
            NavigationPath,
            queue_size=1,
            latch=True,
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

    def final_waypoint_is_passed(self, waypoint):
        """Return whether seq35 satisfies global position and yaw tolerances."""
        if self._final_pass_radius <= 0.0:
            return False
        return self._pose_is_within_tolerances(
            waypoint,
            self._final_pass_radius,
            self._final_yaw_tolerance,
        )

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

    def _fitted_execution_goals(self, final_goal):
        waypoints = self._fitted_path.execution_waypoints
        if len(waypoints) < 2:
            raise RouteNavigationFailure(
                error_codes.GOAL_UNAVAILABLE,
                "fitted path does not contain sequential execution waypoints",
            )
        goals = [
            self._make_pose(
                self._fitted_path.frame_id,
                waypoint.x,
                waypoint.y,
                waypoint.yaw,
            )
            for waypoint in waypoints[:-1]
        ]
        # Preserve the mission-provided final yaw and exact pose. The fitted
        # intermediate points use path-tangent yaw only to guide each segment.
        goals.append(final_goal)
        return goals

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
        return self._execute_fitted_waypoints(context, state_machine)

    def _execute_fitted_waypoints(self, context, state_machine):
        """Send the fitted execution points one at a time to move_base."""
        if not self._fitted_path_matches_route(context.pickup_staging_goals):
            self._abort(
                context,
                state_machine,
                error_codes.GOAL_UNAVAILABLE,
                "fitted path is stale: regenerate it from the active route",
            )
            return None
        try:
            execution_goals = self._fitted_execution_goals(
                context.pickup_staging_goals[-1]
            )
        except RouteNavigationFailure as exc:
            self._abort(
                context,
                state_machine,
                exc.error_code,
                str(exc),
            )
            return None

        self._publish_fitted_reference_path()
        context.pickup_staging_goals = execution_goals
        rospy.loginfo(
            "task=%s executing %d fitted waypoints sequentially with %.2fm "
            "intermediate radius, orientation-required indices=%s at %.3frad, "
            "and %.2fm/%.3frad final tolerances",
            context.task_id,
            len(execution_goals),
            self._intermediate_pass_radius,
            sorted(
                index + 1
                for index in self._orientation_required_waypoint_indices
            ),
            self._orientation_yaw_tolerance,
            self._final_pass_radius,
            self._final_yaw_tolerance,
        )
        return self._execute_waypoints(context, state_machine)

    def _execute_waypoints(self, context, state_machine):
        waypoint_count = len(context.pickup_staging_goals)
        for waypoint_index, waypoint in enumerate(
            context.pickup_staging_goals
        ):
            context.current_waypoint_index = waypoint_index
            context.retry_count = 0
            waypoint_number = waypoint_index + 1
            is_final_waypoint = waypoint_number == waypoint_count
            orientation_required = (
                waypoint_index in self._orientation_required_waypoint_indices
            )
            while context.retry_count <= self._max_retries:
                state_machine.transition(
                    states.NAVIGATE_TO_PICKUP_STAGING,
                    "sending waypoint {}/{} to move_base".format(
                        waypoint_number, waypoint_count
                    ),
                )
                pass_condition = None
                if is_final_waypoint and self._final_pass_radius > 0.0:
                    pass_condition = (
                        lambda waypoint=waypoint:
                        self.final_waypoint_is_passed(waypoint)
                    )
                elif orientation_required:
                    pass_condition = (
                        lambda waypoint=waypoint: self._pose_is_within_tolerances(
                            waypoint,
                            self._intermediate_pass_radius,
                            self._orientation_yaw_tolerance,
                        )
                    )
                elif not is_final_waypoint:
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

                if outcome in (
                    NavigationOutcome.SUCCEEDED,
                    NavigationOutcome.PASSED,
                ):
                    if outcome == NavigationOutcome.SUCCEEDED:
                        waypoint_status = "goal reached"
                    elif is_final_waypoint:
                        waypoint_status = (
                            "final pose aligned within {:.2f} m and "
                            "{:.3f} rad"
                        ).format(
                            self._final_pass_radius,
                            self._final_yaw_tolerance,
                        )
                        # The final goal has no successor, so cancel it before
                        # the mission proceeds to perception.
                        self._navigation.cancel_goal()
                    elif orientation_required:
                        waypoint_status = (
                            "position and heading aligned within {:.2f} m and "
                            "{:.3f} rad"
                        ).format(
                            self._intermediate_pass_radius,
                            self._orientation_yaw_tolerance,
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
