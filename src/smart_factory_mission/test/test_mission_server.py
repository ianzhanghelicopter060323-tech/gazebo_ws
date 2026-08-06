#!/usr/bin/env python3

from types import SimpleNamespace
import unittest
from unittest import mock

from geometry_msgs.msg import PoseStamped

from smart_factory_navigation import error_codes as navigation_error_codes
from smart_factory_navigation.models import NavigationResult
from smart_factory_mission import error_codes, states
from smart_factory_mission.mission_server import MissionServer


class MissionServerTest(unittest.TestCase):
    @staticmethod
    def _server():
        action_server = mock.Mock()
        action_server.is_preempt_requested.return_value = False
        navigation = mock.Mock()
        navigation.wait_for_server.return_value = True
        navigation.wait_for_localization.return_value = (
            NavigationResult(True, navigation_error_codes.SUCCESS, "ready")
        )
        navigation.execute_staging_route.return_value = (
            NavigationResult(
                True,
                navigation_error_codes.SUCCESS,
                "arrived at pickup staging area",
                completed_waypoints=1,
            )
        )
        goal_provider = mock.Mock()
        goal_provider.get_pickup_staging_goals.return_value = [
            PoseStamped()
        ]
        server = MissionServer(
            action_name="/test/execute",
            remember_results=20,
            pipeline_stop_after="ARRIVED_PICKUP_STAGING",
            pickup_pipeline=mock.Mock(),
            goal_provider=goal_provider,
            navigation=navigation,
            navigation_wait_timeout=10.0,
            state_publisher=mock.Mock(),
            action_server=action_server,
            auto_start=False,
        )
        server.published_stages = []
        server._publish_state = mock.Mock(
            side_effect=lambda context, _detail: server.published_stages.append(
                context.current_stage
            )
        )
        server._abort = mock.Mock()
        server._preempt = mock.Mock()
        server._continue_after_arrival = mock.Mock()
        return server

    @staticmethod
    def _goal():
        return SimpleNamespace(task_id="mission-test", target_class=0)

    def test_execute_delegates_route_then_continues_high_level_pipeline(self):
        server = self._server()
        arrival = "arrived at pickup staging area"

        server._execute(self._goal())

        server._navigation.wait_for_server.assert_called_once_with(10.0)
        server._navigation.wait_for_localization.assert_called_once()
        self.assertIs(
            server._server.is_preempt_requested,
            server._navigation.wait_for_localization.call_args.kwargs[
                "preempt_requested"
            ],
        )
        server._goal_provider.get_pickup_staging_goals.assert_called_once()
        route_call = server._navigation.execute_staging_route.call_args
        self.assertEqual(1, len(route_call.args[0]))
        self.assertEqual("mission-test", route_call.kwargs["request_id"])
        context, state_machine, _message = (
            server._continue_after_arrival.call_args.args
        )
        self.assertEqual("mission-test", context.task_id)
        self.assertEqual(0, context.target_class)
        server._continue_after_arrival.assert_called_once_with(
            context, state_machine, arrival
        )
        self.assertEqual(
            [
                states.ACCEPT_TASK,
                states.VALIDATE_TASK,
                states.CHECK_LOCALIZATION,
                states.GET_PICKUP_STAGING_GOAL,
                states.NAVIGATE_TO_PICKUP_STAGING,
                states.ARRIVED_PICKUP_STAGING,
            ],
            server.published_stages,
        )
        server._abort.assert_not_called()
        server._preempt.assert_not_called()

    def test_execute_maps_unavailable_move_base_before_localization(self):
        server = self._server()
        server._navigation.wait_for_server.return_value = False

        server._execute(self._goal())

        self.assertEqual(
            error_codes.MOVE_BASE_UNAVAILABLE,
            server._abort.call_args.args[2],
        )
        server._navigation.wait_for_localization.assert_not_called()
        server._goal_provider.get_pickup_staging_goals.assert_not_called()

    def test_execute_preserves_preempt_precedence_when_localization_wait_fails(self):
        server = self._server()
        server._navigation.wait_for_localization.return_value = (
            NavigationResult(
                False,
                navigation_error_codes.REQUEST_PREEMPTED,
                "localization wait was preempted",
            )
        )
        server._server.is_preempt_requested.return_value = True

        server._execute(self._goal())

        server._preempt.assert_called_once()
        self.assertIn(
            "localization wait was preempted",
            server._preempt.call_args.args[2],
        )
        server._abort.assert_not_called()

    def test_execute_preempts_when_cancel_arrives_with_localization_success(self):
        server = self._server()

        def finish_and_cancel(**_kwargs):
            server._server.is_preempt_requested.return_value = True
            return NavigationResult(
                True, navigation_error_codes.SUCCESS, "ready"
            )

        server._navigation.wait_for_localization.side_effect = (
            finish_and_cancel
        )

        server._execute(self._goal())

        server._preempt.assert_called_once()
        server._goal_provider.get_pickup_staging_goals.assert_not_called()
        server._abort.assert_not_called()

    def test_execute_preempts_when_cancel_arrives_with_route_success(self):
        server = self._server()

        def finish_and_cancel(*_args, **_kwargs):
            server._server.is_preempt_requested.return_value = True
            return NavigationResult(
                True,
                navigation_error_codes.SUCCESS,
                "arrived at pickup staging area",
                completed_waypoints=1,
            )

        server._navigation.execute_staging_route.side_effect = (
            finish_and_cancel
        )

        server._execute(self._goal())

        server._preempt.assert_called_once()
        server._continue_after_arrival.assert_not_called()
        server._abort.assert_not_called()

    def test_execute_maps_route_action_failure_without_entering_pickup(self):
        server = self._server()
        server._navigation.execute_staging_route.return_value = (
            NavigationResult(
                False,
                navigation_error_codes.NAVIGATION_TIMEOUT,
                "route timed out",
            )
        )

        server._execute(self._goal())

        self.assertEqual(
            error_codes.NAVIGATION_TIMEOUT,
            server._abort.call_args.args[2],
        )
        self.assertEqual("route timed out", server._abort.call_args.args[3])
        server._continue_after_arrival.assert_not_called()


if __name__ == "__main__":
    unittest.main()
