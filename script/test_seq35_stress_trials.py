#!/usr/bin/env python3

from collections import Counter
import math
import unittest
from unittest import mock

import run_seq35_stress_trials as stress
import yaml


class Seq35StressPlanTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = stress.load_config(stress.DEFAULT_CONFIG)
        cls.plans = stress.build_case_plans(
            cls.config, cls.config["count"], cls.config["seed"]
        )

    def test_ten_cases_are_food_only_and_deterministic(self):
        repeated = stress.build_case_plans(
            self.config, self.config["count"], self.config["seed"]
        )
        self.assertEqual(self.plans, repeated)
        counts = Counter(plan["target_class"] for plan in self.plans)
        self.assertEqual(sum(counts.values()), 10)
        self.assertEqual(counts, {"food": 10})
        self.assertTrue(all(plan["target_model"] == "cube_0" for plan in self.plans))

    def test_all_targets_jitter_around_round_87_problem_spot(self):
        target = self.config["target"]
        self.assertEqual(target["sampling"], "uniform_jitter")
        self.assertAlmostEqual(target["x"], -0.821)
        self.assertAlmostEqual(target["y"], -0.555)
        poses = []
        for plan in self.plans:
            pose = plan["models"][plan["target_model"]]
            self.assertLessEqual(abs(pose["x"] + 0.821), 0.02)
            self.assertLessEqual(abs(pose["y"] + 0.555), 0.02)
            self.assertLessEqual(abs(pose["yaw"] + 1.5708), 0.05236)
            poses.append((pose["x"], pose["y"], pose["yaw"]))
        self.assertGreater(len(set(poses)), 1)

    def test_every_case_places_one_cube_at_each_station(self):
        for plan in self.plans:
            self.assertEqual(set(plan["models"]), {"cube_0", "cube_1", "cube_2"})
            stations = sorted(pose["station"] if "station" in pose else pose["number"] for pose in plan["models"].values())
            self.assertEqual(stations, [35, 36, 37])

    def test_pre_capture_checks_have_independent_deadlines(self):
        with mock.patch.object(stress.time, "monotonic", side_effect=[10.0, 20.0, 30.0]), mock.patch.object(stress, "wait_for_ros") as wait:
            stress.wait_for_pre_capture_health(15.0)

        self.assertEqual(wait.call_count, 3)
        self.assertEqual([call.args[0] for call in wait.call_args_list], [
            "RGB camera frame",
            "AMCL pose",
            "running arm controller",
        ])
        self.assertEqual([call.args[2] for call in wait.call_args_list], [25.0, 35.0, 45.0])
        self.assertNotIn("mission", " ".join(call.args[0] for call in wait.call_args_list).lower())

    def test_seq35_goal_and_scan_pose_match_runtime_configs(self):
        mission = yaml.safe_load(
            (stress.WORKSPACE / "src/smart_factory_mission/config/mission.yaml").read_text()
        )
        route = yaml.safe_load(
            (stress.WORKSPACE / "src/smart_factory_mission/config/pickup_staging_dev.yaml").read_text()
        )
        fitted = yaml.safe_load(
            (
                stress.WORKSPACE
                / "src/smart_factory_navigation/config/pickup_staging_fitted_path.yaml"
            ).read_text()
        )
        station = next(item for item in mission["pickup"]["stations"] if item["number"] == 35)
        stress_goal = self.config["navigation_goal"]
        route_goal = route["pickup_staging"]["waypoints"][-1]
        fitted_goal = fitted["fitted_path"]["final_goal"]

        for key in ("x", "y", "yaw"):
            self.assertAlmostEqual(stress_goal[key], station[key])
            self.assertAlmostEqual(route_goal[key], station[key])
            self.assertAlmostEqual(fitted_goal[key], station[key])
        self.assertEqual(list(self.config["arm_scan_positions"]), station["scan_positions"])
        self.assertEqual(
            station["supplemental_scan_positions"],
            [0.0, 0.3, 0.60, 1.8, 0.0],
        )

    def test_grasp_acceptance_is_tighter_than_dwa_but_looser_than_direct_control(self):
        mission = yaml.safe_load(
            (
                stress.WORKSPACE
                / "src/smart_factory_mission/config/mission.yaml"
            ).read_text()
        )
        navigation = yaml.safe_load(
            (
                stress.WORKSPACE
                / "src/smart_factory_navigation/config/navigation.yaml"
            ).read_text()
        )
        dwa = yaml.safe_load(
            (
                stress.WORKSPACE
                / "src/gazebo_nav/launch/config/move_base/dwa_local_planner_params.yaml"
            ).read_text()
        )
        mission_tolerance = mission["pickup"]["alignment_tolerance"]
        direct_tolerance = navigation["pickup"]["direct_alignment_tolerance"]
        dwa_tolerance = dwa["DWAPlannerROS"]["xy_goal_tolerance"]
        self.assertAlmostEqual(mission_tolerance, 0.010)
        self.assertAlmostEqual(direct_tolerance, 0.008)
        self.assertAlmostEqual(dwa_tolerance, 0.040)
        self.assertGreater(mission_tolerance, direct_tolerance)
        self.assertLess(mission_tolerance, dwa_tolerance)

    def test_target_motion_reports_translation_and_wrapped_yaw(self):
        scene = {
            "measured": {
                "cube_0": {
                    "x": -0.821,
                    "y": -0.555,
                    "z": 0.02,
                    "yaw": -math.pi / 2.0,
                }
            }
        }
        summary = {
            "runtime_state": {
                "cube_0": {
                    "pose": {
                        "position": {"x": -0.801, "y": -0.555, "z": 0.02},
                        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                    }
                }
            }
        }
        motion = stress.target_motion(scene, summary, "cube_0")
        self.assertAlmostEqual(motion["translation_xy_m"], 0.02)
        self.assertAlmostEqual(motion["yaw_change_rad"], math.pi / 2.0)
        self.assertTrue(motion["movement_detected"])


if __name__ == "__main__":
    unittest.main()
