#!/usr/bin/env python3

from types import SimpleNamespace
import math
import unittest
from unittest import mock

from geometry_msgs.msg import PoseStamped

from smart_factory_navigation import error_codes
from smart_factory_navigation.client import NavigationClient
from smart_factory_navigation.msg import NavigateGoal


class FakeActionClient:
    def __init__(self, result):
        self.result = result
        self.goals = []
        self.cancel_calls = 0

    def send_goal(self, goal, feedback_cb=None):
        self.goals.append(goal)

    def wait_for_result(self, _timeout):
        return True

    def get_result(self):
        return self.result

    def cancel_goal(self):
        self.cancel_calls += 1

    def wait_for_server(self, _timeout):
        return True


def successful_result(pose_valid=False):
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = 1.0
    pose.pose.position.y = -2.0
    yaw = 0.5
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return SimpleNamespace(
        success=True,
        error_code=0,
        message="ok",
        completed_waypoints=2,
        pose_valid=pose_valid,
        localized_pose=pose,
    )


class NavigationClientTest(unittest.TestCase):
    def test_route_method_builds_action_goal_and_returns_value(self):
        action_client = FakeActionClient(successful_result())
        client = NavigationClient(action_client=action_client)
        waypoints = [PoseStamped(), PoseStamped()]

        result = client.execute_staging_route(waypoints, request_id="task-1")

        self.assertTrue(result.success)
        self.assertEqual(2, result.completed_waypoints)
        self.assertEqual(
            NavigateGoal.EXECUTE_STAGING_ROUTE,
            action_client.goals[-1].command,
        )
        self.assertEqual("task-1", action_client.goals[-1].request_id)
        self.assertEqual(2, len(action_client.goals[-1].waypoints))
        self.assertEqual(0, action_client.cancel_calls)

    def test_cancellation_wins_when_result_finishes_in_same_poll(self):
        action_client = FakeActionClient(successful_result())
        client = NavigationClient(action_client=action_client)
        preempt_requested = mock.Mock(side_effect=(False, True))

        result = client.navigate_pose(
            PoseStamped(), preempt_requested=preempt_requested
        )

        self.assertFalse(result.success)
        self.assertEqual(error_codes.REQUEST_PREEMPTED, result.error_code)
        self.assertEqual(1, action_client.cancel_calls)

    def test_pose_method_sends_exactly_one_unmodified_target(self):
        action_client = FakeActionClient(successful_result())
        client = NavigationClient(action_client=action_client)
        target = PoseStamped()
        target.header.frame_id = "map"
        target.pose.position.x = 1.045097827911377
        target.pose.position.y = -2.963428497314453

        result = client.navigate_pose(
            target,
            request_id="delivery-food",
            position_tolerance=0.04,
            yaw_tolerance=math.radians(5.0),
        )

        self.assertTrue(result.success)
        self.assertEqual(1, len(action_client.goals))
        goal = action_client.goals[0]
        self.assertEqual(NavigateGoal.NAVIGATE_POSE, goal.command)
        self.assertEqual("delivery-food", goal.request_id)
        self.assertIs(target, goal.target_pose)
        self.assertAlmostEqual(0.04, goal.position_tolerance)
        self.assertAlmostEqual(math.radians(5.0), goal.yaw_tolerance)

    def test_localized_pose_preserves_frame_and_yaw(self):
        action_client = FakeActionClient(successful_result(pose_valid=True))
        client = NavigationClient(action_client=action_client)

        localized = client.localized_pose("map")

        self.assertEqual(
            NavigateGoal.GET_LOCALIZED_POSE,
            action_client.goals[-1].command,
        )
        self.assertAlmostEqual(1.0, localized[0])
        self.assertAlmostEqual(-2.0, localized[1])
        self.assertAlmostEqual(0.5, localized[2])
        self.assertEqual("map", client.last_result.frame_id)

    def test_wait_for_localization_returns_full_result_and_bool_alias(self):
        action_client = FakeActionClient(successful_result())
        client = NavigationClient(action_client=action_client)

        result = client.wait_for_localization()
        ready = client.wait_until_ready()

        self.assertTrue(result.success)
        self.assertTrue(ready)
        self.assertTrue(
            all(
                goal.command == NavigateGoal.WAIT_FOR_LOCALIZATION
                for goal in action_client.goals
            )
        )


if __name__ == "__main__":
    unittest.main()
