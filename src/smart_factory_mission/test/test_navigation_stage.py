#!/usr/bin/env python3

import unittest
from unittest import mock

from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
import rospy

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
        raise AssertionError("moving lookahead goals must not insert a cancel gap")


class NavigationStageTest(unittest.TestCase):
    def _make_stage(self, states):
        stage = NavigationStage.__new__(NavigationStage)
        stage._client = FakeActionClient(states)
        stage._goal_timeout = 10.0
        return stage

    def test_path_acquisition_condition_does_not_wait_for_move_base_success(self):
        stage = self._make_stage([GoalStatus.ACTIVE])

        with mock.patch.object(
            rospy.Time, "now", return_value=rospy.Time(1.0)
        ):
            outcome, _message = stage.navigate(
                PoseStamped(),
                lambda: False,
                lambda: None,
                completion_condition=lambda: True,
            )

        self.assertEqual(NavigationOutcome.CONDITION_MET, outcome)
        self.assertEqual(1, len(stage._client.sent_goals))

    def test_replacing_a_goal_does_not_cancel_first(self):
        stage = self._make_stage([GoalStatus.PENDING])

        with mock.patch.object(
            rospy.Time, "now", return_value=rospy.Time(1.0)
        ):
            stage.send_or_replace_goal(PoseStamped())
            stage.send_or_replace_goal(PoseStamped())

        self.assertEqual(2, len(stage._client.sent_goals))

    def test_final_goal_requires_move_base_success(self):
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


if __name__ == "__main__":
    unittest.main()
