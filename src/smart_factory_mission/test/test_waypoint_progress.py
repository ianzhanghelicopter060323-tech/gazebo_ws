#!/usr/bin/env python3

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
    def __init__(self, x, y):
        self._transform = TransformStamped()
        self._transform.transform.translation.x = x
        self._transform.transform.translation.y = y

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


if __name__ == "__main__":
    unittest.main()
