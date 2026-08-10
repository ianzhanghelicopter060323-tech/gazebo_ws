#!/usr/bin/env python3

import math
import unittest
from unittest import mock

import rospy

from smart_factory_navigation import error_codes, states
from smart_factory_navigation.action_server import NavigationActionServer
from smart_factory_navigation.models import RouteExecutionContext
from smart_factory_navigation.msg import NavigateGoal


class FakeActionServer:
    def __init__(self):
        self.started = False
        self.terminal = None
        self.feedback = []
        self.preempt_requested = False

    def start(self):
        self.started = True

    def is_preempt_requested(self):
        return self.preempt_requested

    def publish_feedback(self, feedback):
        self.feedback.append(feedback)

    def set_succeeded(self, result, _message):
        self.terminal = ("succeeded", result)

    def set_aborted(self, result, _message):
        self.terminal = ("aborted", result)

    def set_preempted(self, result, _message):
        self.terminal = ("preempted", result)


class FakeRouteExecutor:
    def __init__(self, events):
        self.events = events
        self.publish_hook = None

    def wait_for_server(self):
        self.events.append("move_base")
        return True

    def cancel(self):
        self.events.append("cancel")

    def navigate_pose(
        self,
        context,
        recorder,
        _pose,
        phase,
        _detail,
        position_tolerance=0.0,
        yaw_tolerance=0.0,
    ):
        self.events.append(
            ("pose_tolerances", position_tolerance, yaw_tolerance)
        )
        recorder.transition(phase, "pose started")
        self.publish_hook(context, "pose heartbeat")

    def align_for_grasp(self, context, recorder, _pose, _detail):
        recorder.transition(states.ALIGN_FOR_GRASP, "alignment started")
        self.publish_hook(context, "alignment heartbeat")


class FakeLocalization:
    map_frame = "map"

    def __init__(self, events):
        self.events = events

    def wait_until_ready(self, _preempt_requested):
        self.events.append("localization")
        return not _preempt_requested()

    def localized_pose(self, frame_id):
        self.events.append(("pose", frame_id))
        return (1.0, -2.0, 0.5)


class NavigationActionServerTest(unittest.TestCase):
    def _server(self):
        events = []
        action = FakeActionServer()
        route_executor = FakeRouteExecutor(events)
        with mock.patch(
            "smart_factory_navigation.action_server.rospy.get_param",
            side_effect=lambda _name, default=None: default,
        ), mock.patch("smart_factory_navigation.action_server.rospy.loginfo"):
            server = NavigationActionServer(
                action_server=action,
                localization=FakeLocalization(events),
                route_executor=route_executor,
            )
        route_executor.publish_hook = server._publish_route_state
        return server, action, events

    def test_localization_wait_preserves_move_base_first_order(self):
        server, action, events = self._server()
        goal = NavigateGoal(command=NavigateGoal.WAIT_FOR_LOCALIZATION)

        server.execute(goal)

        self.assertEqual(["move_base", "localization"], events)
        self.assertEqual("succeeded", action.terminal[0])

    def test_pose_query_returns_valid_pose_and_requested_frame(self):
        server, action, events = self._server()
        goal = NavigateGoal(
            command=NavigateGoal.GET_LOCALIZED_POSE, frame_id="odom"
        )

        with mock.patch.object(
            rospy.Time, "now", return_value=rospy.Time(1.0)
        ):
            server.execute(goal)

        result = action.terminal[1]
        self.assertTrue(result.pose_valid)
        self.assertEqual("odom", result.localized_pose.header.frame_id)
        self.assertEqual(1.0, result.localized_pose.pose.position.x)
        self.assertEqual([("pose", "odom")], events)

    def test_empty_route_is_rejected_before_move_base_wait(self):
        server, action, events = self._server()
        goal = NavigateGoal(command=NavigateGoal.EXECUTE_STAGING_ROUTE)

        server.execute(goal)

        self.assertEqual([], events)
        self.assertEqual("aborted", action.terminal[0])
        self.assertEqual(
            error_codes.GOAL_UNAVAILABLE,
            action.terminal[1].error_code,
        )

    def test_localization_preempt_sets_action_preempted(self):
        server, action, events = self._server()
        action.preempt_requested = True
        goal = NavigateGoal(command=NavigateGoal.WAIT_FOR_LOCALIZATION)

        server.execute(goal)

        self.assertEqual(["move_base", "localization"], events)
        self.assertEqual("preempted", action.terminal[0])
        self.assertEqual(
            error_codes.REQUEST_PREEMPTED,
            action.terminal[1].error_code,
        )

    def test_move_base_wait_failure_honors_concurrent_preempt(self):
        server, action, events = self._server()

        def wait_then_preempt():
            events.append("move_base")
            action.preempt_requested = True
            return False

        server._route_executor.wait_for_server = wait_then_preempt

        server.execute(
            NavigateGoal(command=NavigateGoal.WAIT_FOR_LOCALIZATION)
        )

        self.assertEqual(["move_base"], events)
        self.assertEqual("preempted", action.terminal[0])
        self.assertEqual(
            error_codes.REQUEST_PREEMPTED,
            action.terminal[1].error_code,
        )

    def test_unknown_command_aborts_with_explicit_error(self):
        server, action, events = self._server()

        server.execute(NavigateGoal(command=99))

        self.assertEqual([], events)
        self.assertEqual("aborted", action.terminal[0])
        self.assertEqual(
            error_codes.INVALID_COMMAND,
            action.terminal[1].error_code,
        )

    def test_pose_heartbeat_keeps_pose_phase(self):
        server, action, _events = self._server()

        server.execute(NavigateGoal(command=NavigateGoal.NAVIGATE_POSE))

        self.assertEqual("succeeded", action.terminal[0])
        self.assertEqual(
            [states.NAVIGATE_POSE, states.NAVIGATE_POSE],
            [feedback.phase for feedback in action.feedback],
        )

    def test_pose_request_passes_stage_specific_tolerances(self):
        server, action, events = self._server()
        goal = NavigateGoal(command=NavigateGoal.NAVIGATE_POSE)
        goal.position_tolerance = 0.04
        goal.yaw_tolerance = math.radians(5.0)

        server.execute(goal)

        self.assertEqual("succeeded", action.terminal[0])
        self.assertIn(
            ("pose_tolerances", 0.04, math.radians(5.0)), events
        )

    def test_alignment_heartbeat_keeps_alignment_phase(self):
        server, action, _events = self._server()

        server.execute(NavigateGoal(command=NavigateGoal.ALIGN_FOR_GRASP))

        self.assertEqual("succeeded", action.terminal[0])
        self.assertEqual(
            [states.ALIGN_FOR_GRASP, states.ALIGN_FOR_GRASP],
            [feedback.phase for feedback in action.feedback],
        )

    def test_path_progress_feedback_is_throttled_without_losing_progress(self):
        server, action, _events = self._server()
        server._active_context = RouteExecutionContext(
            task_id="throttle-test",
            pickup_staging_goals=[object()],
        )
        server._active_state = mock.Mock(current_phase=states.FOLLOW_ROUTE)

        with mock.patch(
            "smart_factory_navigation.action_server.time.monotonic",
            side_effect=[10.0, 10.1, 10.5],
        ):
            server._publish_progress(1.0)
            server._publish_progress(2.0)
            server._publish_progress(3.0)

        self.assertEqual(3.0, server._last_progress)
        self.assertEqual(2, len(action.feedback))
        self.assertEqual(
            [1.0, 3.0],
            [feedback.path_progress for feedback in action.feedback],
        )


if __name__ == "__main__":
    unittest.main()
