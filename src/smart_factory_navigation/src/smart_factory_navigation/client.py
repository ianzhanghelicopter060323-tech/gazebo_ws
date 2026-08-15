"""Mission-facing client for the navigation Action boundary."""

import math
import time

import actionlib
import rospy

from smart_factory_navigation import error_codes
from smart_factory_navigation.models import (
    NavigationFeedbackEvent,
    NavigationResult,
)
from smart_factory_navigation.msg import NavigateAction, NavigateGoal


class NavigationClient:
    """Blocking high-level API backed by ``Navigate.action``.

    The optional ``action_client`` argument is an explicit test seam; callers do
    not need to construct partially initialized production objects.
    """

    def __init__(
        self,
        action_name="/smart_factory/navigation",
        action_client=None,
        poll_period=0.1,
    ):
        self.action_name = action_name
        self._client = action_client or actionlib.SimpleActionClient(
            action_name, NavigateAction
        )
        self._poll_period = float(poll_period)
        if self._poll_period <= 0.0:
            raise ValueError("poll_period must be positive")
        self.last_result = None

    def wait_for_server(self, timeout=10.0):
        return self._client.wait_for_server(rospy.Duration(float(timeout)))

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
    def _feedback_event(message):
        return NavigationFeedbackEvent(
            phase=message.phase,
            current_waypoint=message.current_waypoint,
            waypoint_count=message.waypoint_count,
            retry_count=message.retry_count,
            path_progress=message.path_progress,
            detail=message.detail,
        )

    @classmethod
    def _result_value(cls, message):
        if message is None:
            return NavigationResult(
                success=False,
                error_code=error_codes.INTERNAL_ERROR,
                message="navigation action returned no result",
            )
        localized = None
        frame_id = ""
        if message.pose_valid:
            pose = message.localized_pose
            frame_id = pose.header.frame_id
            localized = (
                pose.pose.position.x,
                pose.pose.position.y,
                cls._quaternion_yaw(pose.pose.orientation),
            )
        return NavigationResult(
            success=message.success,
            error_code=message.error_code,
            message=message.message,
            completed_waypoints=message.completed_waypoints,
            localized_pose=localized,
            frame_id=frame_id,
        )

    def _call(
        self,
        goal,
        feedback_cb=None,
        preempt_requested=None,
        timeout=None,
    ):
        if timeout is not None:
            timeout = float(timeout)
            if not math.isfinite(timeout) or timeout <= 0.0:
                raise ValueError("navigation request timeout must be positive")
        deadline = None if timeout is None else time.monotonic() + timeout
        callback = None
        if feedback_cb is not None:
            callback = lambda message: feedback_cb(
                self._feedback_event(message)
            )
        self._client.send_goal(goal, feedback_cb=callback)

        def cancel_for_preempt():
            self._client.cancel_goal()
            # Give the server a short opportunity to return its more specific
            # preempt message, but never allow a simultaneously completed goal
            # to override the caller's cancellation request.
            self._client.wait_for_result(rospy.Duration(1.0))
            server_result = self._result_value(self._client.get_result())
            message = (
                server_result.message
                if server_result.error_code == error_codes.REQUEST_PREEMPTED
                else "navigation request was cancelled by the client"
            )
            result = NavigationResult(
                success=False,
                error_code=error_codes.REQUEST_PREEMPTED,
                message=message,
            )
            self.last_result = result
            return result

        def cancel_for_timeout():
            self._client.cancel_goal()
            self._client.wait_for_result(rospy.Duration(1.0))
            result = NavigationResult(
                success=False,
                error_code=error_codes.NAVIGATION_TIMEOUT,
                message="navigation request exceeded {:.1f}s".format(timeout),
            )
            self.last_result = result
            return result

        while not rospy.is_shutdown():
            if preempt_requested is not None and preempt_requested():
                return cancel_for_preempt()
            if deadline is not None and time.monotonic() >= deadline:
                return cancel_for_timeout()
            finished = self._client.wait_for_result(
                rospy.Duration(self._poll_period)
            )
            # Cancellation wins when it becomes visible in the same polling
            # interval as a terminal Action result. This matches the former
            # in-process navigation behavior.
            if preempt_requested is not None and preempt_requested():
                return cancel_for_preempt()
            if deadline is not None and time.monotonic() >= deadline:
                return cancel_for_timeout()
            if finished:
                break
        if rospy.is_shutdown():
            self._client.cancel_goal()
        result = self._result_value(self._client.get_result())
        self.last_result = result
        return result

    def execute_staging_route(
        self,
        waypoints,
        request_id="",
        feedback_cb=None,
        preempt_requested=None,
    ):
        goal = NavigateGoal()
        goal.command = NavigateGoal.EXECUTE_STAGING_ROUTE
        goal.request_id = request_id
        goal.waypoints = list(waypoints)
        return self._call(goal, feedback_cb, preempt_requested)

    def navigate_pose(
        self,
        pose,
        request_id="",
        feedback_cb=None,
        preempt_requested=None,
        position_tolerance=0.0,
        yaw_tolerance=0.0,
        timeout=None,
    ):
        goal = NavigateGoal()
        goal.command = NavigateGoal.NAVIGATE_POSE
        goal.request_id = request_id
        goal.target_pose = pose
        goal.position_tolerance = float(position_tolerance)
        goal.yaw_tolerance = float(yaw_tolerance)
        return self._call(
            goal,
            feedback_cb,
            preempt_requested,
            timeout=timeout,
        )

    def align_for_grasp(
        self,
        pose,
        request_id="",
        feedback_cb=None,
        preempt_requested=None,
    ):
        goal = NavigateGoal()
        goal.command = NavigateGoal.ALIGN_FOR_GRASP
        goal.request_id = request_id
        goal.target_pose = pose
        return self._call(goal, feedback_cb, preempt_requested)

    def wait_for_localization(
        self, preempt_requested=lambda: False, feedback_cb=None
    ):
        goal = NavigateGoal()
        goal.command = NavigateGoal.WAIT_FOR_LOCALIZATION
        return self._call(goal, feedback_cb, preempt_requested)

    def wait_until_ready(self, preempt_requested=lambda: False, feedback_cb=None):
        """Compatibility convenience matching ``LocalizationMonitor``."""
        return self.wait_for_localization(
            preempt_requested, feedback_cb
        ).success

    def localized_pose(self, frame_id):
        goal = NavigateGoal()
        goal.command = NavigateGoal.GET_LOCALIZED_POSE
        goal.frame_id = frame_id
        return self._call(goal).localized_pose

    def localized_xy(self, frame_id):
        localized = self.localized_pose(frame_id)
        return None if localized is None else localized[:2]

    def pose_is_within_radius(self, pose, radius):
        frame_id = pose.header.frame_id
        localized = self.localized_xy(frame_id)
        if localized is None:
            return False
        return math.hypot(
            pose.pose.position.x - localized[0],
            pose.pose.position.y - localized[1],
        ) <= radius
