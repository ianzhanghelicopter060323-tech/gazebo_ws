#!/usr/bin/env python3

from types import SimpleNamespace
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
    def _pose(x=0.0, y=0.0):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        return pose

    def _executor(
        self,
        outcomes,
        parameter_overrides=None,
        preempt_requested=None,
    ):
        parameters = parameter_overrides or {}
        navigation = FakeNavigation(outcomes)
        alignment = mock.Mock()
        alignment.heading_tolerance = 0.25
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

    def test_public_route_api_advances_intermediate_and_final_waypoints(self):
        executor, navigation, _alignment, _published, aborted, _preempted = (
            self._executor(
                [
                    (NavigationOutcome.PASSED, "inside pass radius"),
                    (NavigationOutcome.SUCCEEDED, "goal reached"),
                ]
            )
        )
        context = self._context([self._pose(1.0), self._pose(2.0)])

        message = executor.execute_staging_route(
            context, self._state(context)
        )

        self.assertEqual(
            "all 2 waypoints reached; arrived at pickup staging area", message
        )
        self.assertEqual(2, len(navigation.navigate_calls))
        self.assertIsNotNone(navigation.navigate_calls[0].pass_condition)
        self.assertIsNone(navigation.navigate_calls[1].pass_condition)
        self.assertFalse(aborted)

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
        context = self._context([self._pose()])

        result = executor.execute_staging_route(context, self._state(context))

        self.assertIsNone(result)
        self.assertEqual(2, len(navigation.navigate_calls))
        self.assertEqual(error_codes.NAVIGATION_TIMEOUT, aborted[-1][0])

    def test_public_route_api_reports_preemption(self):
        executor, _navigation, _alignment, _published, _aborted, preempted = (
            self._executor(
                [(NavigationOutcome.PREEMPTED, "preempted")]
            )
        )
        context = self._context([self._pose()])

        result = executor.execute_staging_route(context, self._state(context))

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

    def test_constrained_pass_aligns_heading_without_private_setup(self):
        executor, navigation, alignment, published, _aborted, _preempted = (
            self._executor(
                [
                    (NavigationOutcome.PASSED, "inside pass radius"),
                    (NavigationOutcome.SUCCEEDED, "goal reached"),
                ],
                {"~navigation/heading_constrained_waypoints": [1]},
            )
        )
        alignment.align_heading.return_value = (
            NavigationOutcome.SUCCEEDED,
            "heading aligned",
        )
        context = self._context([self._pose(1.0), self._pose(2.0)])

        with mock.patch(
            "smart_factory_navigation.route_executor.rospy.sleep"
        ):
            executor.execute_staging_route(context, self._state(context))

        self.assertEqual(1, navigation.cancel_calls)
        alignment.align_heading.assert_called_once()
        self.assertTrue(
            any("aligning heading" in detail for _context, detail in published)
        )

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

    def _fitted_executor(self, outcomes, tracking):
        executor, navigation, alignment, published, aborted, preempted = (
            self._executor(outcomes)
        )
        executor._route_execution_mode = "fitted_path_lookahead"
        executor._fitted_path = SimpleNamespace(
            frame_id="map",
            final_goal=(2.0, 3.0, 0.0),
            total_length=1.0,
            points=(SimpleNamespace(x=0.0, y=0.0, yaw=0.0),),
        )
        executor._lookahead_min = 0.20
        executor._lookahead_max = 0.50
        executor._lookahead_curvature_gain = 0.45
        executor._projection_window = 1.0
        executor._direct_segments = []
        executor._heading_locks = []
        executor._path_acquire_radius = 0.20
        executor._path_control_frequency = 20.0
        executor._route_timeout = 240.0
        executor._cross_track_warn = 0.08
        executor._cross_track_abort = 0.25
        executor._progress_epsilon = 0.05
        executor._progress_timeout = 15.0
        executor._final_phase_distance = 0.30
        executor._goal_update_distance = 0.12
        executor._goal_update_period = 0.8
        executor._localization = mock.Mock()
        executor._localization.localized_xy.return_value = (0.0, 0.0)
        executor._localization.pose_is_within_radius.return_value = True
        executor._path_progress_publisher = mock.Mock()
        executor._tracking_goal_publisher = mock.Mock()
        executor._publish_fitted_reference_path = mock.Mock()
        tracker = mock.Mock()
        tracker.update.return_value = tracking
        return (
            executor,
            navigation,
            tracker,
            published,
            aborted,
            preempted,
        )

    def test_public_fitted_route_requires_exact_final_goal_success(self):
        tracking = SimpleNamespace(
            progress_s=0.80,
            cross_track_error=0.01,
            lookahead=0.20,
            target=SimpleNamespace(s=0.90, x=1.8, y=2.8, yaw=0.0),
        )
        executor, navigation, tracker, _published, aborted, preempted = (
            self._fitted_executor(
                [
                    (NavigationOutcome.PASSED, "path start acquired"),
                    (NavigationOutcome.SUCCEEDED, "exact goal reached"),
                ],
                tracking,
            )
        )
        context = self._context([self._pose(2.0, 3.0)])

        with mock.patch(
            "smart_factory_navigation.route_executor.PathTracker",
            return_value=tracker,
        ), mock.patch(
            "smart_factory_navigation.route_executor.time.monotonic",
            return_value=0.0,
        ), mock.patch(
            "smart_factory_navigation.route_executor.rospy.is_shutdown",
            return_value=False,
        ), mock.patch(
            "smart_factory_navigation.route_executor.rospy.Time.now",
            return_value=rospy.Time(1.0),
        ), mock.patch("smart_factory_navigation.route_executor.rospy.Rate"):
            message = executor.execute_staging_route(
                context, self._state(context)
            )

        self.assertIn("fitted path completed", message)
        self.assertEqual(2, len(navigation.navigate_calls))
        self.assertIsNotNone(navigation.navigate_calls[0].pass_condition)
        self.assertIsNone(navigation.navigate_calls[1].pass_condition)
        executor._publish_fitted_reference_path.assert_called_once_with()
        executor._path_progress_publisher.publish.assert_called_once()
        self.assertFalse(aborted)
        self.assertFalse(preempted)

    def test_public_fitted_route_cancels_and_aborts_cross_track_limit(self):
        tracking = SimpleNamespace(
            progress_s=0.10,
            cross_track_error=0.25,
            lookahead=0.20,
            target=SimpleNamespace(s=0.30, x=0.3, y=0.0, yaw=0.0),
        )
        executor, navigation, tracker, _published, aborted, _preempted = (
            self._fitted_executor(
                [(NavigationOutcome.PASSED, "path start acquired")],
                tracking,
            )
        )
        context = self._context([self._pose(2.0, 3.0)])

        with mock.patch(
            "smart_factory_navigation.route_executor.PathTracker",
            return_value=tracker,
        ), mock.patch(
            "smart_factory_navigation.route_executor.time.monotonic",
            return_value=0.0,
        ), mock.patch(
            "smart_factory_navigation.route_executor.rospy.is_shutdown",
            return_value=False,
        ), mock.patch(
            "smart_factory_navigation.route_executor.rospy.Time.now",
            return_value=rospy.Time(1.0),
        ), mock.patch("smart_factory_navigation.route_executor.rospy.Rate"):
            result = executor.execute_staging_route(
                context, self._state(context)
            )

        self.assertIsNone(result)
        self.assertEqual(1, navigation.cancel_calls)
        self.assertEqual(error_codes.NAVIGATION_ABORTED, aborted[-1][0])
        self.assertIn("cross-track error 0.250m", aborted[-1][1])

    def test_public_fitted_route_shutdown_cancels_without_terminal_callback(self):
        tracking = SimpleNamespace(
            progress_s=0.0,
            cross_track_error=0.0,
            lookahead=0.20,
            target=SimpleNamespace(s=0.2, x=0.2, y=0.0, yaw=0.0),
        )
        executor, navigation, tracker, _published, aborted, preempted = (
            self._fitted_executor(
                [(NavigationOutcome.PASSED, "path start acquired")],
                tracking,
            )
        )
        context = self._context([self._pose(2.0, 3.0)])

        with mock.patch(
            "smart_factory_navigation.route_executor.PathTracker",
            return_value=tracker,
        ), mock.patch(
            "smart_factory_navigation.route_executor.time.monotonic",
            return_value=0.0,
        ), mock.patch(
            "smart_factory_navigation.route_executor.rospy.is_shutdown",
            return_value=True,
        ), mock.patch(
            "smart_factory_navigation.route_executor.rospy.Time.now",
            return_value=rospy.Time(1.0),
        ), mock.patch("smart_factory_navigation.route_executor.rospy.Rate"):
            result = executor.execute_staging_route(
                context, self._state(context)
            )

        self.assertIsNone(result)
        self.assertEqual(1, navigation.cancel_calls)
        self.assertFalse(aborted)
        self.assertFalse(preempted)


if __name__ == "__main__":
    unittest.main()
