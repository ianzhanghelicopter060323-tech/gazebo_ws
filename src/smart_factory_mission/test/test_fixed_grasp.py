#!/usr/bin/env python3

import math
import unittest

from smart_factory_mission.fixed_grasp import FixedGraspPlanner


class FixedGraspPlannerTest(unittest.TestCase):
    def setUp(self):
        self.planner = FixedGraspPlanner(
            target_forward=0.356,
            target_lateral=0.0,
            depth_to_center_forward=-0.005,
        )

    def test_seq35_keeps_fixed_forward_clearance(self):
        goal = self.planner.goal_from_surface(-0.88, -0.52, 0.0)
        self.assertAlmostEqual(goal.pose.position.x, -1.241)
        self.assertAlmostEqual(goal.pose.position.y, -0.52)

    def test_seq36_rotates_clearance_with_heading(self):
        goal = self.planner.goal_from_surface(-1.40, 0.06, math.pi / 2.0)
        self.assertAlmostEqual(goal.pose.position.x, -1.40)
        self.assertAlmostEqual(goal.pose.position.y, -0.301)

    def test_seq37_keeps_clearance_behind_negative_x_target(self):
        goal = self.planner.goal_from_surface(-1.99, -0.44, math.pi)
        self.assertAlmostEqual(goal.pose.position.x, -1.629)
        self.assertAlmostEqual(goal.pose.position.y, -0.44)

    def test_lateral_tcp_offset_is_rotated(self):
        planner = FixedGraspPlanner(
            0.356, target_lateral=0.01, depth_to_center_forward=0
        )
        goal = planner.goal_from_surface(1.0, 2.0, math.pi / 2.0)
        self.assertAlmostEqual(goal.pose.position.x, 1.01)
        self.assertAlmostEqual(goal.pose.position.y, 1.644)


if __name__ == "__main__":
    unittest.main()
