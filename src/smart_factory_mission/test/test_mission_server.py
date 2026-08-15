#!/usr/bin/env python3

from types import SimpleNamespace
import math
import unittest
from unittest import mock

from geometry_msgs.msg import PoseStamped
import rospy

from smart_factory_navigation import error_codes as navigation_error_codes
from smart_factory_navigation.models import NavigationResult
from smart_factory_mission import error_codes, states
from smart_factory_mission.goal_provider import DeliveryDestination
from smart_factory_mission.delivery_entry_selector import (
    EntrySelectionUnavailable,
    EntrySelectionPreempted,
)
from smart_factory_mission.mission_server import MissionServer
from smart_factory_mission.state_machine import MissionStateMachine
from smart_factory_mission.task_context import TaskContext


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
        delivery_entry_selector = mock.Mock()
        delivery_entry_selector.resolve.side_effect = (
            lambda pose, **_kwargs: pose
        )
        delivery_entry_selector.requires_approach = False
        delivery_entry_selector.channel_enabled = False
        delivery_entry_selector.channel_waypoint_timeout = 30.0
        delivery_entry_selector.channel_handoff_timeout = 60.0
        delivery_entry_selector.channel_rollback_timeout = 30.0
        delivery_entry_selector.channel_max_reselections = 2
        server = MissionServer(
            action_name="/test/execute",
            remember_results=20,
            pipeline_stop_after="ARRIVED_PICKUP_STAGING",
            pickup_pipeline=mock.Mock(),
            goal_provider=goal_provider,
            delivery_entry_selector=delivery_entry_selector,
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

    def test_grasped_cube_aligns_at_entry_then_navigates_and_releases(self):
        server = self._server()
        server._pipeline_stop_after = "TASK_COMPLETED"
        server._continue_after_arrival = (
            MissionServer._continue_after_arrival.__get__(server)
        )
        destination_pose = PoseStamped()
        destination_pose.header.frame_id = "map"
        destination_pose.pose.position.x = 1.045097827911377
        destination_pose.pose.position.y = -2.963428497314453
        entry_pose = PoseStamped()
        entry_pose.header.frame_id = "map"
        entry_pose.pose.position.x = -0.9398201704025269
        entry_pose.pose.position.y = -1.2094495296478271
        server._goal_provider.get_delivery_destination.return_value = (
            DeliveryDestination(
                "food workshop",
                destination_pose,
                entry_pose,
                entry_position_tolerance=0.04,
                entry_yaw_tolerance=math.radians(10.0),
                position_tolerance=0.04,
                yaw_tolerance=math.radians(5.0),
            )
        )
        server._navigation.navigate_pose.return_value = NavigationResult(
            True,
            navigation_error_codes.SUCCESS,
            "requested pose reached",
            completed_waypoints=1,
        )

        def grasp(**kwargs):
            kwargs["state_machine"].transition(
                states.OBJECT_GRASPED, "target grasped"
            )
            return 35

        server._pickup_pipeline.run.side_effect = grasp
        context = TaskContext("delivery-logic-test", 0)
        state_machine = MissionStateMachine(context, server._publish_state)

        with mock.patch(
            "smart_factory_mission.mission_server.rospy.Time.now",
            return_value=rospy.Time(42, 0),
        ):
            server._continue_after_arrival(
                context, state_machine, "arrived at pickup staging area"
            )

        server._goal_provider.get_delivery_destination.assert_called_once_with(
            context
        )
        server._delivery_entry_selector.resolve.assert_called_once_with(
            entry_pose,
            preempt_requested=server._server.is_preempt_requested,
        )
        navigation_calls = server._navigation.navigate_pose.call_args_list
        self.assertEqual(2, len(navigation_calls))
        self.assertIs(entry_pose, navigation_calls[0].args[0])
        self.assertIs(destination_pose, navigation_calls[1].args[0])
        self.assertAlmostEqual(
            0.04, navigation_calls[0].kwargs["position_tolerance"]
        )
        self.assertAlmostEqual(
            math.radians(10.0),
            navigation_calls[0].kwargs["yaw_tolerance"],
        )
        self.assertAlmostEqual(
            0.04, navigation_calls[1].kwargs["position_tolerance"]
        )
        self.assertAlmostEqual(
            math.radians(5.0),
            navigation_calls[1].kwargs["yaw_tolerance"],
        )
        self.assertTrue(
            all(
                call.kwargs["request_id"] == "delivery-logic-test"
                for call in navigation_calls
            )
        )
        server._pickup_pipeline.release.assert_called_once_with(
            server._server.is_preempt_requested
        )
        self.assertEqual(
            [
                states.OBJECT_GRASPED,
                states.GET_DELIVERY_GOAL,
                states.NAVIGATE_TO_DELIVERY,
                states.NAVIGATE_TO_DELIVERY,
                states.ARRIVED_DELIVERY,
                states.RELEASE_OBJECT,
                states.TASK_COMPLETED,
            ],
            server.published_stages,
        )
        result = server._server.set_succeeded.call_args.args[0]
        self.assertTrue(result.success)
        self.assertEqual(states.TASK_COMPLETED, result.completed_stage)
        self.assertIn("food workshop", result.message)
        server._abort.assert_not_called()
        server._preempt.assert_not_called()

    def test_delivery_navigation_failure_never_releases_cube(self):
        server = self._server()
        server._pipeline_stop_after = "TASK_COMPLETED"
        server._continue_after_arrival = (
            MissionServer._continue_after_arrival.__get__(server)
        )
        destination_pose = PoseStamped()
        entry_pose = PoseStamped()
        server._goal_provider.get_delivery_destination.return_value = (
            DeliveryDestination("daily workshop", destination_pose, entry_pose)
        )
        server._navigation.navigate_pose.return_value = NavigationResult(
            False,
            navigation_error_codes.NAVIGATION_ABORTED,
            "cone-zone goal aborted",
        )

        def grasp(**kwargs):
            kwargs["state_machine"].transition(
                states.OBJECT_GRASPED, "target grasped"
            )
            return 36

        server._pickup_pipeline.run.side_effect = grasp
        context = TaskContext("delivery-failure-test", 1)
        state_machine = MissionStateMachine(context, server._publish_state)

        with mock.patch(
            "smart_factory_mission.mission_server.rospy.Time.now",
            return_value=rospy.Time(42, 0),
        ):
            server._continue_after_arrival(
                context, state_machine, "arrived at pickup staging area"
            )

        server._pickup_pipeline.release.assert_not_called()
        server._navigation.navigate_pose.assert_called_once()
        self.assertIs(
            entry_pose, server._navigation.navigate_pose.call_args.args[0]
        )
        server._abort.assert_called_once_with(
            context,
            state_machine,
            error_codes.NAVIGATION_ABORTED,
            "cone-zone goal aborted",
        )
        server._server.set_succeeded.assert_not_called()

    def test_delivery_entry_selection_honors_task_preempt(self):
        server = self._server()
        server._pipeline_stop_after = "TASK_COMPLETED"
        server._continue_after_arrival = (
            MissionServer._continue_after_arrival.__get__(server)
        )
        server._delivery_entry_selector.resolve.side_effect = (
            EntrySelectionPreempted("entry selection cancelled")
        )

        def grasp(**kwargs):
            kwargs["state_machine"].transition(
                states.OBJECT_GRASPED, "target grasped"
            )
            return 35

        server._pickup_pipeline.run.side_effect = grasp
        context = TaskContext("entry-preempt-test", 0)
        state_machine = MissionStateMachine(context, server._publish_state)

        with mock.patch(
            "smart_factory_mission.mission_server.rospy.Time.now",
            return_value=rospy.Time(42, 0),
        ):
            server._continue_after_arrival(
                context, state_machine, "arrived at pickup staging area"
            )

        server._preempt.assert_called_once_with(
            context, state_machine, "entry selection cancelled"
        )
        server._navigation.navigate_pose.assert_not_called()
        server._abort.assert_not_called()

    def test_delivery_entry_is_projected_after_safe_laser_approach(self):
        server = self._server()
        server._pipeline_stop_after = "TASK_COMPLETED"
        server._continue_after_arrival = (
            MissionServer._continue_after_arrival.__get__(server)
        )
        destination_pose = PoseStamped()
        nominal_entry = PoseStamped()
        safe_entry = PoseStamped()
        safe_entry.pose.position.x = -1.1
        safe_entry.pose.position.y = -1.3
        server._goal_provider.get_delivery_destination.return_value = (
            DeliveryDestination(
                "daily workshop",
                destination_pose,
                nominal_entry,
                entry_position_tolerance=0.04,
                entry_yaw_tolerance=math.radians(10.0),
                position_tolerance=0.04,
                yaw_tolerance=math.radians(5.0),
            )
        )
        server._delivery_entry_selector.requires_approach = True
        server._delivery_entry_selector.approach_position_tolerance = 0.35
        server._delivery_entry_selector.resolve.side_effect = None
        server._delivery_entry_selector.resolve.return_value = safe_entry
        server._navigation.navigate_pose.return_value = NavigationResult(
            True,
            navigation_error_codes.SUCCESS,
            "requested pose reached",
            completed_waypoints=1,
        )

        def grasp(**kwargs):
            kwargs["state_machine"].transition(
                states.OBJECT_GRASPED, "target grasped"
            )
            return 35

        server._pickup_pipeline.run.side_effect = grasp
        context = TaskContext("dynamic-entry-test", 1)
        state_machine = MissionStateMachine(context, server._publish_state)

        with mock.patch(
            "smart_factory_mission.mission_server.rospy.Time.now",
            return_value=rospy.Time(42, 0),
        ):
            server._continue_after_arrival(
                context, state_machine, "arrived at pickup staging area"
            )

        calls = server._navigation.navigate_pose.call_args_list
        self.assertEqual(3, len(calls))
        self.assertIs(nominal_entry, calls[0].args[0])
        self.assertAlmostEqual(
            0.35, calls[0].kwargs["position_tolerance"]
        )
        server._delivery_entry_selector.resolve.assert_called_once_with(
            nominal_entry,
            preempt_requested=server._server.is_preempt_requested,
        )
        selected_pose = calls[1].args[0]
        self.assertAlmostEqual(-1.1, selected_pose.pose.position.x)
        self.assertAlmostEqual(-1.3, selected_pose.pose.position.y)
        self.assertAlmostEqual(
            0.04, calls[1].kwargs["position_tolerance"]
        )
        self.assertIs(destination_pose, calls[2].args[0])

    def test_laser_channel_waypoint_is_inserted_before_destination(self):
        server = self._server()
        server._pipeline_stop_after = "TASK_COMPLETED"
        server._continue_after_arrival = (
            MissionServer._continue_after_arrival.__get__(server)
        )
        destination_pose = PoseStamped()
        nominal_entry = PoseStamped()
        safe_entry = PoseStamped()
        channel_pose = PoseStamped()
        channel_pose.pose.position.x = -0.2
        channel_pose.pose.position.y = -1.0
        server._goal_provider.get_delivery_destination.return_value = (
            DeliveryDestination(
                "daily workshop",
                destination_pose,
                nominal_entry,
                entry_position_tolerance=0.04,
                entry_yaw_tolerance=math.radians(10.0),
                position_tolerance=0.04,
                yaw_tolerance=math.radians(5.0),
            )
        )
        server._delivery_entry_selector.resolve.side_effect = None
        server._delivery_entry_selector.resolve.return_value = safe_entry
        server._delivery_entry_selector.channel_enabled = True
        server._delivery_entry_selector.channel_max_waypoints = 2
        server._delivery_entry_selector.channel_waypoint_position_tolerance = (
            0.12
        )
        server._delivery_entry_selector.channel_waypoint_yaw_tolerance = math.pi
        server._delivery_entry_selector.channel_handoff_position_tolerance = (
            0.30
        )
        server._delivery_entry_selector.channel_handoff_yaw_tolerance = math.pi
        server._delivery_entry_selector.resolve_channel_waypoint.side_effect = [
            channel_pose,
            None,
        ]
        server._navigation.navigate_pose.return_value = NavigationResult(
            True,
            navigation_error_codes.SUCCESS,
            "requested pose reached",
            completed_waypoints=1,
        )

        def grasp(**kwargs):
            kwargs["state_machine"].transition(
                states.OBJECT_GRASPED, "target grasped"
            )
            return 35

        server._pickup_pipeline.run.side_effect = grasp
        context = TaskContext("channel-test", 1)
        state_machine = MissionStateMachine(context, server._publish_state)

        with mock.patch(
            "smart_factory_mission.mission_server.rospy.Time.now",
            return_value=rospy.Time(42, 0),
        ):
            server._continue_after_arrival(
                context, state_machine, "arrived at pickup staging area"
            )

        calls = server._navigation.navigate_pose.call_args_list
        self.assertEqual(4, len(calls))
        self.assertIs(safe_entry, calls[0].args[0])
        self.assertIs(channel_pose, calls[1].args[0])
        self.assertAlmostEqual(0.12, calls[1].kwargs["position_tolerance"])
        self.assertAlmostEqual(math.pi, calls[1].kwargs["yaw_tolerance"])
        self.assertIs(destination_pose, calls[2].args[0])
        self.assertAlmostEqual(0.30, calls[2].kwargs["position_tolerance"])
        self.assertAlmostEqual(math.pi, calls[2].kwargs["yaw_tolerance"])
        self.assertIs(destination_pose, calls[3].args[0])
        self.assertAlmostEqual(0.04, calls[3].kwargs["position_tolerance"])
        self.assertAlmostEqual(
            math.radians(5.0), calls[3].kwargs["yaw_tolerance"]
        )
        self.assertEqual(
            [mock.call(True), mock.call(False)],
            server._delivery_entry_selector
            .set_channel_avoidance_lock.call_args_list,
        )
        self.assertEqual(
            [
                mock.call(
                    destination_pose,
                    safe_entry,
                    0,
                    preempt_requested=server._server.is_preempt_requested,
                    force_selection=False,
                ),
                mock.call(
                    destination_pose,
                    safe_entry,
                    1,
                    preempt_requested=server._server.is_preempt_requested,
                    force_selection=False,
                ),
            ],
            server._delivery_entry_selector
            .resolve_channel_waypoint.call_args_list,
        )

    def test_channel_limit_never_unlocks_into_still_narrow_path(self):
        server = self._server()
        server._pipeline_stop_after = "TASK_COMPLETED"
        server._continue_after_arrival = (
            MissionServer._continue_after_arrival.__get__(server)
        )
        destination_pose = PoseStamped()
        nominal_entry = PoseStamped()
        safe_entry = PoseStamped()
        channel_pose = PoseStamped()
        still_narrow_pose = PoseStamped()
        server._goal_provider.get_delivery_destination.return_value = (
            DeliveryDestination(
                "daily workshop",
                destination_pose,
                nominal_entry,
                entry_position_tolerance=0.04,
                entry_yaw_tolerance=math.radians(10.0),
                position_tolerance=0.04,
                yaw_tolerance=math.radians(5.0),
            )
        )
        server._delivery_entry_selector.resolve.side_effect = None
        server._delivery_entry_selector.resolve.return_value = safe_entry
        server._delivery_entry_selector.channel_enabled = True
        server._delivery_entry_selector.channel_max_waypoints = 1
        server._delivery_entry_selector.channel_waypoint_position_tolerance = (
            0.18
        )
        server._delivery_entry_selector.channel_waypoint_yaw_tolerance = math.pi
        server._delivery_entry_selector.resolve_channel_waypoint.side_effect = [
            channel_pose,
            still_narrow_pose,
        ]
        server._navigation.navigate_pose.return_value = NavigationResult(
            True,
            navigation_error_codes.SUCCESS,
            "requested pose reached",
            completed_waypoints=1,
        )

        def grasp(**kwargs):
            kwargs["state_machine"].transition(
                states.OBJECT_GRASPED, "target grasped"
            )
            return 35

        server._pickup_pipeline.run.side_effect = grasp
        context = TaskContext("channel-limit-test", 1)
        state_machine = MissionStateMachine(context, server._publish_state)

        with mock.patch(
            "smart_factory_mission.mission_server.rospy.Time.now",
            return_value=rospy.Time(42, 0),
        ):
            server._continue_after_arrival(
                context, state_machine, "arrived at pickup staging area"
            )

        navigation_calls = server._navigation.navigate_pose.call_args_list
        self.assertEqual(2, len(navigation_calls))
        self.assertIs(safe_entry, navigation_calls[0].args[0])
        self.assertIs(channel_pose, navigation_calls[1].args[0])
        self.assertFalse(
            any(
                call.args[0] is destination_pose
                for call in navigation_calls
            )
        )
        self.assertEqual(
            [mock.call(True), mock.call(False)],
            server._delivery_entry_selector
            .set_channel_avoidance_lock.call_args_list,
        )
        server._abort.assert_called_once()
        abort_call = server._abort.call_args
        self.assertIn(
            "refusing an unsafe direct-goal handoff", abort_call.args[3]
        )
        server._pickup_pipeline.release.assert_not_called()

    def test_first_channel_timeout_reselects_without_returning_to_preparation(self):
        server = self._server()
        server._pipeline_stop_after = "TASK_COMPLETED"
        server._continue_after_arrival = (
            MissionServer._continue_after_arrival.__get__(server)
        )
        destination_pose = PoseStamped()
        destination_pose.header.frame_id = "map"
        nominal_entry = PoseStamped()
        nominal_entry.header.frame_id = "map"
        safe_entry = PoseStamped()
        safe_entry.header.frame_id = "map"
        timed_out_pose = PoseStamped()
        timed_out_pose.header.frame_id = "map"
        alternate_pose = PoseStamped()
        alternate_pose.header.frame_id = "map"
        server._goal_provider.get_delivery_destination.return_value = (
            DeliveryDestination(
                "daily workshop",
                destination_pose,
                nominal_entry,
                position_tolerance=0.04,
                yaw_tolerance=math.radians(5.0),
            )
        )
        selector = server._delivery_entry_selector
        selector.resolve.side_effect = None
        selector.resolve.return_value = safe_entry
        selector.channel_enabled = True
        selector.channel_max_waypoints = 3
        selector.channel_waypoint_position_tolerance = 0.18
        selector.channel_waypoint_yaw_tolerance = math.pi
        selector.channel_handoff_position_tolerance = 0.15
        selector.channel_handoff_yaw_tolerance = math.pi
        selector.resolve_channel_waypoint.side_effect = [
            timed_out_pose,
            alternate_pose,
            None,
        ]
        succeeded = NavigationResult(
            True,
            navigation_error_codes.SUCCESS,
            "requested pose reached",
            completed_waypoints=1,
        )
        timed_out = NavigationResult(
            False,
            navigation_error_codes.NAVIGATION_TIMEOUT,
            "navigation request exceeded 30.0s",
        )
        server._navigation.navigate_pose.side_effect = [
            succeeded,
            timed_out,
            succeeded,
            succeeded,
            succeeded,
        ]

        def grasp(**kwargs):
            kwargs["state_machine"].transition(
                states.OBJECT_GRASPED, "target grasped"
            )
            return 35

        server._pickup_pipeline.run.side_effect = grasp
        context = TaskContext("first-channel-timeout-test", 1)
        state_machine = MissionStateMachine(context, server._publish_state)

        with mock.patch(
            "smart_factory_mission.mission_server.rospy.Time.now",
            return_value=rospy.Time(42, 0),
        ):
            server._continue_after_arrival(
                context, state_machine, "arrived at pickup staging area"
            )

        selector.reject_channel_waypoint.assert_called_once_with(
            timed_out_pose
        )
        calls = server._navigation.navigate_pose.call_args_list
        self.assertEqual(1, sum(call.args[0] is safe_entry for call in calls))
        self.assertEqual(
            [False, True, False],
            [
                call.kwargs["force_selection"]
                for call in selector.resolve_channel_waypoint.call_args_list
            ],
        )
        server._abort.assert_not_called()

    def test_handoff_timeout_rolls_back_then_forces_another_waypoint(self):
        server = self._server()
        selector = server._delivery_entry_selector
        selector.channel_max_waypoints = 3
        selector.channel_waypoint_position_tolerance = 0.18
        selector.channel_waypoint_yaw_tolerance = math.pi
        selector.channel_handoff_position_tolerance = 0.15
        selector.channel_handoff_yaw_tolerance = math.pi
        first_pose = PoseStamped()
        alternate_pose = PoseStamped()
        destination_pose = PoseStamped()
        destination = SimpleNamespace(pose=destination_pose)
        selector.resolve_channel_waypoint.side_effect = [
            first_pose,
            None,
            alternate_pose,
            None,
        ]
        succeeded = NavigationResult(
            True,
            navigation_error_codes.SUCCESS,
            "requested pose reached",
        )
        timed_out = NavigationResult(
            False,
            navigation_error_codes.NAVIGATION_TIMEOUT,
            "navigation request exceeded 60.0s",
        )
        server._navigation.navigate_pose.side_effect = [
            succeeded,
            timed_out,
            succeeded,
            succeeded,
            succeeded,
        ]
        context = TaskContext("handoff-timeout-test", 1)
        state_machine = MissionStateMachine(context, server._publish_state)

        server._navigate_delivery_channel(
            context, state_machine, destination, PoseStamped()
        )

        navigation_poses = [
            call.args[0]
            for call in server._navigation.navigate_pose.call_args_list
        ]
        self.assertEqual(
            [first_pose, destination_pose, first_pose, alternate_pose, destination_pose],
            navigation_poses,
        )
        self.assertTrue(
            selector.resolve_channel_waypoint.call_args_list[2].kwargs[
                "force_selection"
            ]
        )


if __name__ == "__main__":
    unittest.main()
