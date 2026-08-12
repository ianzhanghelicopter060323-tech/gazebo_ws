#!/usr/bin/env python3

import math
import unittest

from smart_factory_navigation.fitted_path import FittedPath, PathConfigError


def path_config():
    points = [
        {"s": 0.0, "x": 0.0, "y": 0.0, "yaw": 0.0},
        {"s": 1.0, "x": 1.0, "y": 0.0, "yaw": 0.0},
        {"s": 2.0, "x": 2.0, "y": 0.0, "yaw": math.pi / 2.0},
        {"s": 3.0, "x": 2.0, "y": 1.0, "yaw": math.pi / 2.0},
    ]
    return {
        "configured": True,
        "frame_id": "map",
        "points": points,
        "execution_waypoints": [points[0], points[2], points[3]],
        "final_goal": {"x": 2.0, "y": 1.0, "yaw": 1.0},
    }


class FittedPathTest(unittest.TestCase):
    def test_loads_only_runtime_path_fields(self):
        config = path_config()
        config["points"][0].update(
            curvature=1.5,
            source_seq_start=1,
            source_seq_end=2,
        )

        path = FittedPath.from_config(config)

        self.assertEqual("map", path.frame_id)
        self.assertEqual(4, len(path.points))
        self.assertEqual(3, len(path.execution_waypoints))
        self.assertAlmostEqual(2.0, path.execution_waypoints[1].x)
        self.assertEqual((2.0, 1.0, 1.0), path.final_goal)

    def test_rejects_non_monotonic_reference_arc(self):
        config = path_config()
        config["points"][2]["s"] = 1.0

        with self.assertRaises(PathConfigError):
            FittedPath.from_config(config)

    def test_requires_sequential_execution_waypoints(self):
        config = path_config()
        del config["execution_waypoints"]

        with self.assertRaises(PathConfigError):
            FittedPath.from_config(config)

    def test_rejects_execution_waypoints_that_stop_before_path_end(self):
        config = path_config()
        config["execution_waypoints"] = config["points"][:3]

        with self.assertRaises(PathConfigError):
            FittedPath.from_config(config)

    def test_rejects_final_goal_position_that_differs_from_path_end(self):
        config = path_config()
        config["final_goal"]["x"] += 0.01

        with self.assertRaises(PathConfigError):
            FittedPath.from_config(config)


if __name__ == "__main__":
    unittest.main()
