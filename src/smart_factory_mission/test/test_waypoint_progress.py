#!/usr/bin/env python3

import math
import unittest
from unittest import mock

from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, TransformStamped
import rospy

from smart_factory_mission.mission_server import MissionServer
from smart_factory_mission.navigation_stage import (
    NavigationOutcome,
    NavigationStage,
)


class FakeActionClient:
    def __init__(self, states):
        self._states = iter(states)
        self.sent_goals = []

    def send_goal(self, goal):
        self.sent_goals.append(goal)

    def get_state(self):
        return next(self._states)

    def cancel_goal(self):
        raise AssertionError("an intermediate pass must not insert a cancel gap")


class FakeTfBuffer:
    def __init__(self, x, y, yaw=0.0):
        self._transform = TransformStamped()
        self._transform.transform.translation.x = x
        self._transform.transform.translation.y = y
        self._transform.transform.rotation.z = math.sin(yaw / 2.0)
        self._transform.transform.rotation.w = math.cos(yaw / 2.0)

    def lookup_transform(self, *_args):
        return self._transform


class WaypointProgressTest(unittest.TestCase):
    def _make_stage(self, states):
        stage = NavigationStage.__new__(NavigationStage)
        stage._client = FakeActionClient(states)
        stage._goal_timeout = 10.0
        return stage

    def test_intermediate_pass_does_not_wait_for_move_base_success(self):
        stage = self._make_stage([GoalStatus.ACTIVE])

        with mock.patch.object(
            rospy.Time, "now", return_value=rospy.Time(1.0)
        ):
            outcome, _message = stage.navigate(
                PoseStamped(),
                lambda: False,
                lambda: None,
                pass_condition=lambda: True,
            )

        self.assertEqual(NavigationOutcome.PASSED, outcome)
        self.assertEqual(1, len(stage._client.sent_goals))

    def test_replacing_a_goal_does_not_cancel_first(self):
        stage = self._make_stage([GoalStatus.PENDING])

        with mock.patch.object(
            rospy.Time, "now", return_value=rospy.Time(1.0)
        ):
            stage.send_or_replace_goal(PoseStamped())
            stage.send_or_replace_goal(PoseStamped())

        self.assertEqual(2, len(stage._client.sent_goals))

    def test_final_goal_still_requires_move_base_success(self):
        stage = self._make_stage(
            [GoalStatus.ACTIVE, GoalStatus.SUCCEEDED]
        )

        with mock.patch.object(
            rospy.Time, "now", return_value=rospy.Time(1.0)
        ):
            outcome, _message = stage.navigate(
                PoseStamped(),
                lambda: False,
                lambda: None,
            )

        self.assertEqual(NavigationOutcome.SUCCEEDED, outcome)

    def test_pass_radius_uses_localized_tf_position_only(self):
        server = MissionServer.__new__(MissionServer)
        server._intermediate_pass_radius = 0.20
        server._intermediate_yaw_tolerance = 0.25
        server._map_frame = "map"
        server._base_frame = "base_footprint"
        server._tf_buffer = FakeTfBuffer(1.12, 2.08)

        waypoint = PoseStamped()
        waypoint.header.frame_id = "map"
        waypoint.pose.position.x = 1.0
        waypoint.pose.position.y = 2.0
        waypoint.pose.orientation.z = 1.0

        self.assertTrue(server._intermediate_waypoint_is_passed(waypoint))

        server._tf_buffer = FakeTfBuffer(1.30, 2.0)
        self.assertFalse(server._intermediate_waypoint_is_passed(waypoint))

    def test_heading_error_distinguishes_wrong_and_aligned_yaw(self):
        server = MissionServer.__new__(MissionServer)
        server._map_frame = "map"
        server._base_frame = "base_footprint"

        waypoint = PoseStamped()
        waypoint.header.frame_id = "map"
        waypoint.pose.position.x = 2.621
        waypoint.pose.position.y = -1.013
        target_yaw = -3.068
        waypoint.pose.orientation.z = math.sin(target_yaw / 2.0)
        waypoint.pose.orientation.w = math.cos(target_yaw / 2.0)

        server._tf_buffer = FakeTfBuffer(2.60, -0.82, yaw=-1.55)
        self.assertGreater(
            abs(server._intermediate_waypoint_heading_error(waypoint)),
            0.25,
        )

        server._tf_buffer = FakeTfBuffer(2.60, -0.98, yaw=-3.00)
        self.assertLessEqual(
            abs(server._intermediate_waypoint_heading_error(waypoint)),
            0.25,
        )

    def test_heading_constraint_uses_shortest_angle_across_pi(self):
        server = MissionServer.__new__(MissionServer)
        server._map_frame = "map"
        server._base_frame = "base_footprint"

        waypoint = PoseStamped()
        waypoint.header.frame_id = "map"
        waypoint.pose.orientation.z = math.sin(3.10 / 2.0)
        waypoint.pose.orientation.w = math.cos(3.10 / 2.0)
        server._tf_buffer = FakeTfBuffer(0.0, 0.0, yaw=-3.10)

        self.assertLessEqual(
            abs(server._intermediate_waypoint_heading_error(waypoint)),
            0.10,
        )

    def test_heading_alignment_command_clamps_shortest_turn(self):
        server = MissionServer.__new__(MissionServer)
        server._heading_alignment_kp = 1.0
        server._heading_alignment_min_angular_speed = 0.40
        server._heading_alignment_max_angular_speed = 0.45

        self.assertAlmostEqual(-0.45, server._heading_alignment_command(-1.5))
        self.assertAlmostEqual(0.40, server._heading_alignment_command(0.26))
        self.assertAlmostEqual(0.0, server._heading_alignment_command(0.0))

    def test_heading_alignment_cancels_move_base_and_stops(self):
        server = MissionServer.__new__(MissionServer)
        server._map_frame = "map"
        server._base_frame = "base_footprint"
        server._intermediate_yaw_tolerance = 0.25
        server._heading_alignment_timeout = 8.0
        server._heading_alignment_kp = 1.0
        server._heading_alignment_min_angular_speed = 0.40
        server._heading_alignment_max_angular_speed = 0.45
        server._tf_buffer = FakeTfBuffer(0.0, 0.0, yaw=-3.00)
        server._navigation = mock.Mock()
        server._cmd_vel_pub = mock.Mock()
        server._server = mock.Mock()
        server._server.is_preempt_requested.return_value = False

        waypoint = PoseStamped()
        target_yaw = -3.068
        waypoint.pose.orientation.z = math.sin(target_yaw / 2.0)
        waypoint.pose.orientation.w = math.cos(target_yaw / 2.0)

        with mock.patch.object(rospy, "sleep"), mock.patch.object(
            rospy, "Rate"
        ), mock.patch.object(
            rospy.Time, "now", return_value=rospy.Time(1.0)
        ):
            outcome, _message = (
                server._align_intermediate_waypoint_heading(
                    waypoint, lambda: None
                )
            )

        self.assertEqual(NavigationOutcome.SUCCEEDED, outcome)
        server._navigation.cancel_goal.assert_called_once_with()
        server._cmd_vel_pub.publish.assert_called_once()


if __name__ == "__main__":
    unittest.main()
