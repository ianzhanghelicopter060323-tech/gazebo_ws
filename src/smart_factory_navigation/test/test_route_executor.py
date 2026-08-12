#!/usr/bin/env python3

from types import SimpleNamespace
import math
import unittest
from unittest import mock

from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
import rospy

from smart_factory_navigation import error_codes, states
from smart_factory_navigation.models import (
    NavigationStateRecorder,
    RouteExecutionContext,
)
from smart_factory_navigation.navigation_stage import NavigationOutcome
from smart_factory_navigation.route_executor import (
    RouteExecutor,
    RouteNavigationFailure,
    RouteNavigationPreempted,
)


class RecordingPublisher:
    def __init__(self, *args, **kwargs):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeLocalization:
    map_frame = "map"

    def localized_pose(self, _frame_id):
        return (0.0, 0.0, 0.0)

    def localized_xy(self, _frame_id):
        return (0.0, 0.0)

    def pose_is_within_radius(self, _pose, _radius):
        return True


class FakeNavigation:
    def __init__(self, outcomes):
        self.outcomes = iter(outcomes)
        self.navigate_calls = []
        self.cancel_calls = 0
        self.state = GoalStatus.SUCCEEDED

    def navigate(self, pose, preempt_requested, heartbeat, pass_condition=None):
        self.navigate_calls.append(
            SimpleNamespace(
                pose=pose,
                preempt_requested=preempt_requested,
                heartbeat=heartbeat,
                pass_condition=pass_condition,
            )
        )
        return next(self.outcomes)

    def wait_for_server(self, _timeout):
        return True

    def cancel_goal(self):
        self.cancel_calls += 1

    def get_state(self):
        return self.state


class RouteExecutorTest(unittest.TestCase):
    @staticmethod
    def _pose(x=0.0, y=0.0, yaw=0.0):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    @staticmethod
    def _fitted_config():
        points = [
            {"s": 0.0, "x": 0.0, "y": 0.0, "yaw": 0.1},
            {"s": 1.5, "x": 1.0, "y": 1.0, "yaw": 0.2},
            {"s": 3.5, "x": 2.0, "y": 3.0, "yaw": 1.2},
        ]
        return {
            "configured": True,
            "frame_id": "map",
            "points": points,
            "execution_waypoints": list(points),
            "final_goal": {"x": 2.0, "y": 3.0, "yaw": 0.4},
        }

    def _executor(
        self,
        outcomes,
        parameter_overrides=None,
        preempt_requested=None,
    ):
        parameters = {
            "~navigation/fitted_waypoints/count": 3,
            "~fitted_path": self._fitted_config(),
        }
        parameters.update(parameter_overrides or {})
        navigation = FakeNavigation(outcomes)
        alignment = mock.Mock()
        alignment.stop = mock.Mock()
        published = []
        aborted = []
        preempted = []

        def get_param(name, default=None):
            return parameters.get(name, default)

        with mock.patch(
            "smart_factory_navigation.route_executor.rospy.get_param",
            side_effect=get_param,
        ), mock.patch(
            "smart_factory_navigation.route_executor.rospy.Publisher",
            side_effect=RecordingPublisher,
        ), mock.patch(
            "smart_factory_navigation.route_executor.NavigationStage",
            return_value=navigation,
        ), mock.patch(
            "smart_factory_navigation.route_executor.BaseAlignmentController",
            return_value=alignment,
        ):
            executor = RouteExecutor(
                localization=FakeLocalization(),
                publish_state=lambda context, detail: published.append(
                    (context, detail)
                ),
                abort=lambda context, state_machine, code, detail: aborted.append(
                    (code, detail)
                ),
                preempt=lambda context, state_machine, detail: preempted.append(
                    detail
                ),
                preempt_requested=(
                    preempt_requested
                    if preempt_requested is not None
                    else lambda: False
                ),
            )
        return executor, navigation, alignment, published, aborted, preempted

    @staticmethod
    def _context(goals):
        return RouteExecutionContext("route-test", list(goals))

    @staticmethod
    def _state(context):
        return NavigationStateRecorder(context, lambda _event: None)

    def test_fitted_waypoints_are_sent_sequentially(self):
        executor, navigation, _alignment, _published, aborted, _preempted = (
            self._executor(
                [
                    (NavigationOutcome.PASSED, "inside pass radius"),
                    (NavigationOutcome.PASSED, "inside pass radius"),
                    (NavigationOutcome.PASSED, "inside final pass radius"),
                ],
                {
                    "~navigation/intermediate_pass_radius": 0.15,
                    "~navigation/final_pass_radius": 0.15,
                    "~navigation/final_yaw_tolerance": 0.04,
                },
            )
        )
        executor._publish_fitted_reference_path = mock.Mock()
        executor._pose_is_within_tolerances = mock.Mock(return_value=True)
        final_goal = self._pose(2.0, 3.0, yaw=0.4)
        context = self._context([final_goal])

        with mock.patch(
            "smart_factory_navigation.route_executor.rospy.Time.now",
            return_value=rospy.Time(1.0),
        ):
            message = executor.execute_staging_route(
                context, self._state(context)
            )

        self.assertEqual(
            "all 3 waypoints reached; arrived at pickup staging area", message
        )
        self.assertEqual(3, len(navigation.navigate_calls))
        self.assertIsNotNone(navigation.navigate_calls[0].pass_condition)
        self.assertIsNotNone(navigation.navigate_calls[1].pass_condition)
        self.assertIsNotNone(navigation.navigate_calls[2].pass_condition)
        self.assertTrue(navigation.navigate_calls[2].pass_condition())
        executor._pose_is_within_tolerances.assert_called_once_with(
            final_goal, 0.15, 0.04
        )
        self.assertAlmostEqual(
            1.0, navigation.navigate_calls[1].pose.pose.position.x
        )
        self.assertIs(navigation.navigate_calls[2].pose, final_goal)
        self.assertEqual(0.15, executor._intermediate_pass_radius)
        self.assertEqual(0.15, executor._final_pass_radius)
        self.assertEqual(0.04, executor._final_yaw_tolerance)
        self.assertEqual(1, navigation.cancel_calls)
        executor._publish_fitted_reference_path.assert_called_once_with()
        self.assertFalse(aborted)

    def test_zero_final_pass_radius_keeps_exact_move_base_completion(self):
        executor, navigation, _alignment, _published, aborted, _preempted = (
            self._executor(
                [
                    (NavigationOutcome.PASSED, "inside pass radius"),
                    (NavigationOutcome.PASSED, "inside pass radius"),
                    (NavigationOutcome.SUCCEEDED, "goal reached"),
                ],
                {
                    "~navigation/intermediate_pass_radius": 0.15,
                    "~navigation/final_pass_radius": 0.0,
                },
            )
        )
        executor._publish_fitted_reference_path = mock.Mock()
        final_goal = self._pose(2.0, 3.0, yaw=0.4)
        context = self._context([final_goal])

        with mock.patch(
            "smart_factory_navigation.route_executor.rospy.Time.now",
            return_value=rospy.Time(1.0),
        ):
            message = executor.execute_staging_route(
                context, self._state(context)
            )

        self.assertEqual(
            "all 3 waypoints reached; arrived at pickup staging area", message
        )
        self.assertIsNone(navigation.navigate_calls[2].pass_condition)
        self.assertEqual(0, navigation.cancel_calls)
        self.assertFalse(aborted)

    def test_negative_final_pass_radius_is_rejected(self):
        with self.assertRaisesRegex(
            ValueError, "navigation/final_pass_radius must not be negative"
        ):
            self._executor(
                [], {"~navigation/final_pass_radius": -0.01}
            )

    def test_nonpositive_final_yaw_tolerance_is_rejected(self):
        with self.assertRaisesRegex(
            ValueError, "navigation/final_yaw_tolerance must be positive"
        ):
            self._executor(
                [], {"~navigation/final_yaw_tolerance": 0.0}
            )

    def test_final_radius_does_not_bypass_seq35_yaw_alignment(self):
        executor, *_ = self._executor(
            [],
            {
                "~navigation/final_pass_radius": 0.15,
                "~navigation/final_yaw_tolerance": 0.04,
            },
        )

        self.assertFalse(
            executor.final_waypoint_is_passed(self._pose(yaw=0.041))
        )
        self.assertTrue(
            executor.final_waypoint_is_passed(self._pose(yaw=0.039))
        )

    def test_stale_final_goal_is_rejected_before_navigation(self):
        executor, navigation, _alignment, _published, aborted, _preempted = (
            self._executor([])
        )
        context = self._context([self._pose(2.0, 3.0, yaw=0.5)])

        with mock.patch(
            "smart_factory_navigation.route_executor.rospy.Time.now",
            return_value=rospy.Time(1.0),
        ):
            result = executor.execute_staging_route(
                context, self._state(context)
            )

        self.assertIsNone(result)
        self.assertFalse(navigation.navigate_calls)
        self.assertEqual(error_codes.GOAL_UNAVAILABLE, aborted[-1][0])

    def test_move_base_wait_polls_and_stops_promptly_on_preempt(self):
        preempt_requested = mock.Mock(side_effect=(False, True))
        executor, navigation, *_ = self._executor(
            [], preempt_requested=preempt_requested
        )
        navigation.wait_for_server = mock.Mock(return_value=False)

        with mock.patch(
            "smart_factory_navigation.route_executor.time.monotonic",
            return_value=0.0,
        ), mock.patch(
            "smart_factory_navigation.route_executor.rospy.is_shutdown",
            return_value=False,
        ):
            available = executor.wait_for_server()

        self.assertFalse(available)
        navigation.wait_for_server.assert_called_once_with(
            executor.SERVER_WAIT_POLL_PERIOD
        )

    def test_public_route_api_reports_retry_exhaustion(self):
        executor, navigation, _alignment, _published, aborted, _preempted = (
            self._executor(
                [
                    (NavigationOutcome.TIMEOUT, "timed out"),
                    (NavigationOutcome.TIMEOUT, "timed out again"),
                ]
            )
        )
        context = self._context([self._pose(2.0, 3.0, yaw=0.4)])

        with mock.patch(
            "smart_factory_navigation.route_executor.rospy.Time.now",
            return_value=rospy.Time(1.0),
        ):
            result = executor.execute_staging_route(
                context, self._state(context)
            )

        self.assertIsNone(result)
        self.assertEqual(2, len(navigation.navigate_calls))
        self.assertEqual(error_codes.NAVIGATION_TIMEOUT, aborted[-1][0])

    def test_public_route_api_reports_preemption(self):
        executor, _navigation, _alignment, _published, _aborted, preempted = (
            self._executor(
                [(NavigationOutcome.PREEMPTED, "preempted")]
            )
        )
        context = self._context([self._pose(2.0, 3.0, yaw=0.4)])

        with mock.patch(
            "smart_factory_navigation.route_executor.rospy.Time.now",
            return_value=rospy.Time(1.0),
        ):
            result = executor.execute_staging_route(
                context, self._state(context)
            )

        self.assertIsNone(result)
        self.assertEqual(1, len(preempted))

    def test_empty_route_is_rejected_consistently_at_new_boundary(self):
        executor, _navigation, _alignment, _published, _aborted, _preempted = (
            self._executor([])
        )
        context = self._context([])

        with self.assertRaises(RouteNavigationFailure) as raised:
            executor.execute_staging_route(context, self._state(context))

        self.assertEqual(error_codes.GOAL_UNAVAILABLE, raised.exception.error_code)

    def test_single_pose_api_retries_and_returns(self):
        executor, navigation, _alignment, _published, _aborted, _preempted = (
            self._executor(
                [
                    (NavigationOutcome.ABORTED, "failed"),
                    (NavigationOutcome.SUCCEEDED, "reached"),
                ]
            )
        )
        context = self._context([])

        executor.navigate_pose(
            context,
            self._state(context),
            self._pose(),
            states.NAVIGATE_POSE,
            "single pose",
        )

        self.assertEqual(2, len(navigation.navigate_calls))
        self.assertEqual(0, context.retry_count)

    def test_single_pose_accepts_five_degree_entry_tolerance(self):
        executor, navigation, *_ = self._executor(
            [(NavigationOutcome.PASSED, "inside entry tolerances")]
        )
        context = self._context([])
        pose = self._pose(yaw=math.radians(5.0))

        executor.navigate_pose(
            context,
            self._state(context),
            pose,
            states.NAVIGATE_POSE,
            "cone preparation pose",
            position_tolerance=0.04,
            yaw_tolerance=math.radians(5.0),
        )

        self.assertTrue(navigation.navigate_calls[0].pass_condition())
        self.assertEqual(1, navigation.cancel_calls)

    def test_entry_tolerance_rejects_heading_over_five_degrees(self):
        executor, *_ = self._executor([])
        pose = self._pose(yaw=math.radians(5.01))

        self.assertFalse(
            executor._pose_is_within_tolerances(
                pose, 0.04, math.radians(5.0)
            )
        )

    def test_single_pose_maps_terminal_outcomes_to_public_exceptions(self):
        cases = (
            (
                NavigationOutcome.PREEMPTED,
                RouteNavigationPreempted,
                None,
            ),
            (
                NavigationOutcome.TIMEOUT,
                RouteNavigationFailure,
                error_codes.NAVIGATION_TIMEOUT,
            ),
            (
                NavigationOutcome.ABORTED,
                RouteNavigationFailure,
                error_codes.NAVIGATION_ABORTED,
            ),
        )
        for outcome, exception_type, expected_code in cases:
            with self.subTest(outcome=outcome):
                executor, *_ = self._executor(
                    [(outcome, "navigation detail")],
                    {"~navigation/max_retries": 0},
                )
                context = self._context([])
                with self.assertRaises(exception_type) as raised:
                    executor.navigate_pose(
                        context,
                        self._state(context),
                        self._pose(),
                        states.NAVIGATE_POSE,
                        "single pose",
                    )
                self.assertIn("navigation detail", str(raised.exception))
                if expected_code is not None:
                    self.assertEqual(
                        expected_code, raised.exception.error_code
                    )

    def test_grasp_alignment_cancels_only_active_move_base_states(self):
        active = (
            GoalStatus.PENDING,
            GoalStatus.ACTIVE,
            GoalStatus.PREEMPTING,
            GoalStatus.RECALLING,
        )
        terminal = (
            GoalStatus.PREEMPTED,
            GoalStatus.SUCCEEDED,
            GoalStatus.ABORTED,
            GoalStatus.REJECTED,
            GoalStatus.RECALLED,
            GoalStatus.LOST,
        )
        for state in active + terminal:
            with self.subTest(state=state):
                executor, navigation, alignment, *_ = self._executor([])
                navigation.state = state
                context = self._context([])
                recorder = self._state(context)
                pose = self._pose()

                executor.align_for_grasp(
                    context, recorder, pose, "alignment"
                )

                self.assertEqual(int(state in active), navigation.cancel_calls)
                alignment.align.assert_called_once_with(
                    context, recorder, pose, "alignment"
                )

if __name__ == "__main__":
    unittest.main()
