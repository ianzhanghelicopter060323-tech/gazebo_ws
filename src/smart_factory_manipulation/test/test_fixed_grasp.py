#!/usr/bin/env python3

import math
import unittest

import rospy

from smart_factory_manipulation.fixed_grasp import FixedGraspPlanner


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

    def test_preserves_requested_frame_timestamp_and_heading(self):
        stamp = rospy.Time(12, 34)
        planner = FixedGraspPlanner(0.4, frame_id="odom")

        goal = planner.goal_from_surface(2.0, 3.0, -math.pi / 2.0, stamp)

        self.assertEqual(goal.header.frame_id, "odom")
        self.assertEqual(goal.header.stamp, stamp)
        self.assertAlmostEqual(goal.pose.orientation.z, -math.sqrt(0.5))
        self.assertAlmostEqual(goal.pose.orientation.w, math.sqrt(0.5))

    def test_reports_planar_correction_distance(self):
        goal = self.planner.goal_from_surface(4.0, 6.0, 0.0)
        distance = self.planner.correction_distance(
            goal.pose.position.x - 3.0,
            goal.pose.position.y - 4.0,
            goal,
        )
        self.assertAlmostEqual(distance, 5.0)

    def test_rejects_invalid_calibration_and_measurements(self):
        invalid_calibrations = (
            {"target_forward": 0.0},
            {"target_forward": -0.1},
            {"target_forward": float("nan")},
            {"target_forward": 0.3, "target_lateral": float("inf")},
            {"target_forward": 0.3, "frame_id": "  "},
        )
        for config in invalid_calibrations:
            with self.subTest(config=config):
                with self.assertRaises(ValueError):
                    FixedGraspPlanner(**config)

        with self.assertRaises(ValueError):
            self.planner.goal_from_surface(float("nan"), 0.0, 0.0)


if __name__ == "__main__":
    unittest.main()
