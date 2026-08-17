"""ROS Action server that owns all navigation-side mutable state."""

import math
import time

import actionlib
from geometry_msgs.msg import PoseStamped
import rospy
from std_srvs.srv import Trigger, TriggerResponse

from smart_factory_navigation import error_codes, states
from smart_factory_navigation.base_alignment_controller import (
    BaseAlignmentFailure,
    BaseAlignmentPreempted,
)
from smart_factory_navigation.localization_monitor import LocalizationMonitor
from smart_factory_navigation.models import (
    NavigationFeedbackEvent,
    NavigationStateRecorder,
    RouteExecutionContext,
)
from smart_factory_navigation.msg import (
    NavigateAction,
    NavigateFeedback,
    NavigateResult,
)
from smart_factory_navigation.route_executor import (
    RouteExecutor,
    RouteNavigationFailure,
    RouteNavigationPreempted,
)


class NavigationActionServer:
    """Expose route, pose, alignment, and localization operations as one Action."""

    def __init__(
        self,
        action_server=None,
        localization=None,
        route_executor=None,
    ):
        self._action_name = rospy.get_param(
            "~action_name", "/smart_factory/navigation"
        )
        self._server = action_server or actionlib.SimpleActionServer(
            self._action_name,
            NavigateAction,
            execute_cb=self.execute,
            auto_start=False,
        )
        self._localization = localization or LocalizationMonitor()
        self._active_context = None
        self._active_state = None
        self._failure = None
        self._last_progress = 0.0
        progress_feedback_frequency = float(
            rospy.get_param(
                "~navigation/path_tracking/progress_feedback_frequency",
                2.0,
            )
        )
        if (
            not math.isfinite(progress_feedback_frequency)
            or progress_feedback_frequency <= 0.0
        ):
            raise ValueError(
                "navigation/path_tracking/progress_feedback_frequency "
                "must be positive"
            )
        self._progress_feedback_period = 1.0 / progress_feedback_frequency
        self._last_progress_feedback_at = None
        self._route_executor = route_executor or RouteExecutor(
            localization=self._localization,
            publish_state=self._publish_route_state,
            abort=self._record_abort,
            preempt=self._record_preempt,
            preempt_requested=self._server.is_preempt_requested,
            progress_callback=self._publish_progress,
        )
        self._server.start()
        # On-demand bounded escape (strafe or front/back), invoked by the
        # mission server when the entry fan cannot select a channel from the
        # current pose (EntrySelectionUnavailable).  The escape repositions
        # the base so the next channel selection sees open space.
        self._recover_service = rospy.Service(
            rospy.get_param("~recover_service_name", "recover"),
            Trigger,
            self._handle_recover,
        )
        rospy.loginfo("smart factory navigation ready on %s", self._action_name)

    def _handle_recover(self, request):
        """Trigger the navigation-side bounded escape; never crash the node."""
        try:
            succeeded = self._route_executor.recover()
        except Exception as exc:
            rospy.logerr("recover service failed: %s", exc)
            return TriggerResponse(success=False, message=str(exc))
        return TriggerResponse(
            success=succeeded,
            message=(
                "bounded escape completed"
                if succeeded
                else "bounded escape failed to move the base"
            ),
        )

    def _new_result(self, success=False, error_code=error_codes.INTERNAL_ERROR, message=""):
        result = NavigateResult()
        result.success = success
        result.error_code = error_code
        result.message = message
        result.completed_waypoints = 0
        result.pose_valid = False
        return result

    def _publish_feedback(self, event):
        feedback = NavigateFeedback()
        feedback.phase = event.phase
        feedback.current_waypoint = max(0, int(event.current_waypoint))
        feedback.waypoint_count = max(0, int(event.waypoint_count))
        feedback.retry_count = max(0, int(event.retry_count))
        feedback.path_progress = float(event.path_progress)
        feedback.detail = event.detail
        self._server.publish_feedback(feedback)

    def _publish_route_state(self, context, detail):
        phase = (
            self._active_state.current_phase
            if self._active_state is not None
            else states.FOLLOW_ROUTE
        )
        self._publish_feedback(
            NavigationFeedbackEvent(
                phase=phase,
                current_waypoint=context.current_waypoint_index + 1,
                waypoint_count=len(context.pickup_staging_goals),
                retry_count=context.retry_count,
                path_progress=self._last_progress,
                detail=detail,
            )
        )

    def _publish_progress(self, progress):
        self._last_progress = float(progress)
        now = time.monotonic()
        feedback_due = (
            self._last_progress_feedback_at is None
            or now - self._last_progress_feedback_at
            >= self._progress_feedback_period
        )
        if self._active_context is not None and feedback_due:
            self._last_progress_feedback_at = now
            self._publish_route_state(
                self._active_context,
                "following fitted path at s={:.3f}m".format(progress),
            )

    def _record_abort(self, context, _state_machine, code, message):
        self._failure = (False, int(code), message)
        self._publish_route_state(context, message)

    def _record_preempt(self, context, _state_machine, message):
        self._failure = (True, error_codes.REQUEST_PREEMPTED, message)
        self._publish_route_state(context, message)

    def _set_terminal(self, result, preempted=False):
        if preempted or result.error_code == error_codes.REQUEST_PREEMPTED:
            self._server.set_preempted(result, result.message)
        elif result.success:
            self._server.set_succeeded(result, result.message)
        else:
            self._server.set_aborted(result, result.message)

    def _require_move_base(self):
        if self._route_executor.wait_for_server():
            return True
        if self._server.is_preempt_requested():
            result = self._new_result(
                error_code=error_codes.REQUEST_PREEMPTED,
                message="navigation request was preempted while waiting for move_base",
            )
            self._set_terminal(result, preempted=True)
            return False
        result = self._new_result(
            error_code=error_codes.MOVE_BASE_UNAVAILABLE,
            message="move_base action server is unavailable",
        )
        self._set_terminal(result)
        return False

    def _execute_route(self, goal):
        if not goal.waypoints:
            result = self._new_result(
                error_code=error_codes.GOAL_UNAVAILABLE,
                message="staging route contains no waypoints",
            )
            self._set_terminal(result)
            return
        if not self._require_move_base():
            return

        context = RouteExecutionContext(
            task_id=goal.request_id or "navigation-action",
            pickup_staging_goals=list(goal.waypoints),
        )
        self._active_context = context
        self._active_state = NavigationStateRecorder(
            context, self._publish_feedback
        )
        self._failure = None
        self._last_progress = 0.0
        self._last_progress_feedback_at = None
        try:
            message = self._route_executor.execute_staging_route(
                context, self._active_state
            )
            if message is None:
                preempted, code, detail = self._failure or (
                    False,
                    error_codes.NAVIGATION_ABORTED,
                    "route execution ended without a result",
                )
                result = self._new_result(error_code=code, message=detail)
                result.completed_waypoints = min(
                    context.current_waypoint_index,
                    len(context.pickup_staging_goals),
                )
                self._set_terminal(result, preempted=preempted)
                return

            detail = (
                message
                if isinstance(message, str)
                else "fitted path completed; arrived at pickup staging area"
            )
            result = self._new_result(
                success=True,
                error_code=error_codes.SUCCESS,
                message=detail,
            )
            result.completed_waypoints = len(context.pickup_staging_goals)
            self._set_terminal(result)
        finally:
            self._active_context = None
            self._active_state = None

    def _execute_pose(self, goal):
        if not self._require_move_base():
            return
        context = RouteExecutionContext(
            task_id=goal.request_id or "navigation-action",
            pickup_staging_goals=[goal.target_pose],
        )
        recorder = NavigationStateRecorder(context, self._publish_feedback)
        self._active_context = context
        self._active_state = recorder
        try:
            try:
                self._route_executor.navigate_pose(
                    context,
                    recorder,
                    goal.target_pose,
                    states.NAVIGATE_POSE,
                    "navigating to requested pose",
                    position_tolerance=goal.position_tolerance,
                    yaw_tolerance=goal.yaw_tolerance,
                )
            except RouteNavigationPreempted as exc:
                result = self._new_result(
                    error_code=error_codes.REQUEST_PREEMPTED,
                    message=str(exc),
                )
                self._set_terminal(result, preempted=True)
                return
            except RouteNavigationFailure as exc:
                result = self._new_result(
                    error_code=exc.error_code, message=str(exc)
                )
                self._set_terminal(result)
                return
            result = self._new_result(
                success=True,
                error_code=error_codes.SUCCESS,
                message="requested pose reached",
            )
            result.completed_waypoints = 1
            self._set_terminal(result)
        finally:
            self._active_context = None
            self._active_state = None

    def _execute_alignment(self, goal):
        context = RouteExecutionContext(
            task_id=goal.request_id or "navigation-action",
            pickup_staging_goals=[],
        )
        recorder = NavigationStateRecorder(context, self._publish_feedback)
        self._active_context = context
        self._active_state = recorder
        try:
            try:
                self._route_executor.align_for_grasp(
                    context,
                    recorder,
                    goal.target_pose,
                    "aligning base for grasp",
                )
            except BaseAlignmentPreempted as exc:
                result = self._new_result(
                    error_code=error_codes.REQUEST_PREEMPTED,
                    message=str(exc),
                )
                self._set_terminal(result, preempted=True)
                return
            except BaseAlignmentFailure as exc:
                result = self._new_result(
                    error_code=exc.error_code, message=str(exc)
                )
                self._set_terminal(result)
                return
            result = self._new_result(
                success=True,
                error_code=error_codes.SUCCESS,
                message="base aligned for grasp",
            )
            self._set_terminal(result)
        finally:
            self._active_context = None
            self._active_state = None

    def _execute_wait_localization(self):
        # Preserve the existing mission startup order: navigation capability is
        # validated before localization readiness or goal lookup.
        if not self._require_move_base():
            return
        self._publish_feedback(
            NavigationFeedbackEvent(
                phase=states.WAIT_LOCALIZATION,
                detail="waiting for localization readiness",
            )
        )
        ready = self._localization.wait_until_ready(
            self._server.is_preempt_requested
        )
        if ready:
            result = self._new_result(
                success=True,
                error_code=error_codes.SUCCESS,
                message="localization is ready",
            )
            self._set_terminal(result)
        elif self._server.is_preempt_requested():
            result = self._new_result(
                error_code=error_codes.REQUEST_PREEMPTED,
                message="localization wait was preempted",
            )
            self._set_terminal(result, preempted=True)
        else:
            result = self._new_result(
                error_code=error_codes.LOCALIZATION_NOT_READY,
                message="localization did not become ready before timeout",
            )
            self._set_terminal(result)

    def _execute_get_pose(self, goal):
        frame_id = goal.frame_id.strip() or self._localization.map_frame
        localized = self._localization.localized_pose(frame_id)
        if localized is None:
            result = self._new_result(
                error_code=error_codes.LOCALIZATION_NOT_READY,
                message="localized pose in {} is unavailable".format(frame_id),
            )
            result.localized_pose.header.frame_id = frame_id
            self._set_terminal(result)
            return

        result = self._new_result(
            success=True,
            error_code=error_codes.SUCCESS,
            message="localized pose in {} is available".format(frame_id),
        )
        result.pose_valid = True
        result.localized_pose = PoseStamped()
        result.localized_pose.header.frame_id = frame_id
        result.localized_pose.header.stamp = rospy.Time.now()
        result.localized_pose.pose.position.x = localized[0]
        result.localized_pose.pose.position.y = localized[1]
        result.localized_pose.pose.orientation.z = math.sin(localized[2] / 2.0)
        result.localized_pose.pose.orientation.w = math.cos(localized[2] / 2.0)
        self._set_terminal(result)

    def execute(self, goal):
        """Handle one generated ``NavigateGoal`` (also used as ROS callback)."""
        try:
            if goal.command == goal.EXECUTE_STAGING_ROUTE:
                self._execute_route(goal)
            elif goal.command == goal.NAVIGATE_POSE:
                self._execute_pose(goal)
            elif goal.command == goal.ALIGN_FOR_GRASP:
                self._execute_alignment(goal)
            elif goal.command == goal.WAIT_FOR_LOCALIZATION:
                self._execute_wait_localization()
            elif goal.command == goal.GET_LOCALIZED_POSE:
                self._execute_get_pose(goal)
            else:
                result = self._new_result(
                    error_code=error_codes.INVALID_COMMAND,
                    message="unsupported navigation command {}".format(goal.command),
                )
                self._set_terminal(result)
        except Exception as exc:  # Action callbacks must always terminate a goal.
            rospy.logerr("navigation action failed unexpectedly: %s", exc)
            try:
                self._route_executor.cancel()
            except Exception:  # Keep original exception as the diagnostic.
                pass
            result = self._new_result(
                error_code=error_codes.INTERNAL_ERROR,
                message="navigation internal error: {}".format(exc),
            )
            self._set_terminal(result)
