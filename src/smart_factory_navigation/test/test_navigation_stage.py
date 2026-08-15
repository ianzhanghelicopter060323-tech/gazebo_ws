#!/usr/bin/env python3

import unittest
from unittest import mock

from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
import rospy

from smart_factory_navigation.navigation_stage import (
    NavigationOutcome,
    NavigationStage,
)


class FakeActionClient:
    def __init__(self, states):
        self.states = iter(states)
        self.goals = []
        self.cancel_calls = 0

    def send_goal(self, goal):
        self.goals.append(goal)

    def get_state(self):
        return next(self.states)

    def cancel_goal(self):
        self.cancel_calls += 1

    def wait_for_server(self, _timeout):
        return True


class NavigationStageTest(unittest.TestCase):
    def _stage(self, states):
        client = FakeActionClient(states)
        with mock.patch(
            "smart_factory_navigation.navigation_stage.actionlib.SimpleActionClient",
            return_value=client,
        ):
            stage = NavigationStage("/move_base", 10.0)
        return stage, client

    def test_intermediate_pass_does_not_cancel_active_goal(self):
        stage, client = self._stage([GoalStatus.ACTIVE])
        with mock.patch.object(
            rospy.Time, "now", return_value=rospy.Time(1.0)
        ):
            outcome, _ = stage.navigate(
                PoseStamped(),
                lambda: False,
                lambda: None,
                pass_condition=lambda: True,
            )
        self.assertEqual(NavigationOutcome.PASSED, outcome)
        self.assertEqual(0, client.cancel_calls)

    def test_replacement_goal_has_no_explicit_cancel_gap(self):
        stage, client = self._stage([])
        with mock.patch.object(
            rospy.Time, "now", return_value=rospy.Time(1.0)
        ):
            stage.send_or_replace_goal(PoseStamped())
            stage.send_or_replace_goal(PoseStamped())
        self.assertEqual(2, len(client.goals))
        self.assertEqual(0, client.cancel_calls)

    def test_final_goal_still_requires_move_base_success(self):
        stage, _client = self._stage(
            [GoalStatus.ACTIVE, GoalStatus.SUCCEEDED]
        )
        with mock.patch.object(
            rospy.Time, "now", return_value=rospy.Time(1.0)
        ), mock.patch(
            "smart_factory_navigation.navigation_stage.time.monotonic",
            return_value=0.0,
        ), mock.patch(
            "smart_factory_navigation.navigation_stage.time.sleep"
        ):
            outcome, _ = stage.navigate(
                PoseStamped(), lambda: False, lambda: None
            )
        self.assertEqual(NavigationOutcome.SUCCEEDED, outcome)

    def test_no_progress_heartbeat_cancels_goal_for_recovery(self):
        stage, client = self._stage([GoalStatus.ACTIVE])
        with mock.patch.object(
            rospy.Time, "now", return_value=rospy.Time(1.0)
        ):
            outcome, message = stage.navigate(
                PoseStamped(), lambda: False, lambda: True
            )

        self.assertEqual(NavigationOutcome.STUCK, outcome)
        self.assertIn("no measurable progress", message)
        self.assertEqual(1, client.cancel_calls)


if __name__ == "__main__":
    unittest.main()
