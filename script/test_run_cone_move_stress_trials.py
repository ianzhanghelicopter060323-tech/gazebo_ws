#!/usr/bin/env python3

import json
from pathlib import Path
import tempfile
import unittest

import _run_cone_move_navigation as navigation_helper
import _setup_cone_move_stress_scene as scene_helper
import run_cone_move_stress_trials as stress


class ConeMoveStressRunnerTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        cones = {
            "cone_{}".format(number): {
                "position": {"x": float(number), "y": -2.0, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            }
            for number in range(10, 20)
        }
        templates = []
        for source_round in (2, 9, 12, 17, 27, 37, 38, 40):
            templates.append(
                {
                    "template_id": "collision_round_{:03d}".format(source_round),
                    "source_round": source_round,
                    "source_task_id": "source_{}".format(source_round),
                    "target_class": (
                        "daily" if source_round in (37, 40) else "electronics"
                    ),
                    "cones": cones,
                }
            )
        self.manifest = self.root / "collision_templates.json"
        self.manifest.write_text(
            json.dumps({"templates": templates}), encoding="utf-8"
        )

    def tearDown(self):
        self.temporary.cleanup()

    @staticmethod
    def runtime():
        destination = {
            "frame_id": "map",
            "name": "workshop",
            "x": 1.0,
            "y": -2.0,
            "yaw": 0.0,
            "position_tolerance": 0.04,
            "yaw_tolerance": 0.1,
        }
        return {
            "robot_start": {"x": -0.9, "y": -1.49, "yaw": -1.57, "z": 0.01},
            "switch_distance": 0.5,
            "destinations": {
                "food": dict(destination),
                "daily": dict(destination),
                "electronics": dict(destination),
            },
        }

    def test_defaults_are_headless_five_repetitions_and_requested_root(self):
        args = stress.parse_args([])

        self.assertFalse(args.gui)
        self.assertEqual(5, args.repetitions)
        self.assertEqual(60.0, args.navigation_ready_timeout)
        self.assertEqual(stress.DEFAULT_RECORDING_ROOT, args.recording_root)

    def test_readiness_wait_retries_until_action_handshake_succeeds(self):
        attempts = []
        outcomes = iter((False, False, True))

        ready = navigation_helper.wait_until_ready(
            lambda timeout: attempts.append(timeout) or next(outcomes),
            timeout=60.0,
            poll_timeout=2.0,
        )

        self.assertTrue(ready)
        self.assertEqual(3, len(attempts))
        self.assertTrue(all(0.0 < timeout <= 2.0 for timeout in attempts))

    def test_readiness_wait_obeys_wall_clock_deadline(self):
        clock = iter((10.0, 10.0, 11.0, 12.1))
        attempts = []

        ready = navigation_helper.wait_until_ready(
            lambda timeout: attempts.append(timeout) or False,
            timeout=2.0,
            poll_timeout=1.0,
            monotonic=lambda: next(clock),
            shutdown=lambda: False,
        )

        self.assertFalse(ready)
        self.assertEqual([1.0, 1.0], attempts)

    def test_default_manifest_requires_exact_eight_templates(self):
        templates = stress.load_templates(self.manifest)

        self.assertEqual(8, len(templates))
        self.assertEqual(
            [2, 9, 12, 17, 27, 37, 38, 40],
            [template["source_round"] for template in templates],
        )

    def test_schedule_groups_five_repetitions_per_template(self):
        templates = stress.load_templates(self.manifest)

        plans = stress.build_plans(templates, 5, self.runtime())

        self.assertEqual(40, len(plans))
        self.assertEqual(
            ["collision_round_002"] * 5,
            [plan["scenario_id"] for plan in plans[:5]],
        )
        self.assertEqual([1, 2, 3, 4, 5], [plan["repetition"] for plan in plans[:5]])
        self.assertEqual("daily", plans[-1]["target_class"])

    def test_selected_template_can_be_replayed_independently(self):
        templates = stress.load_templates(
            self.manifest, ["collision_round_037"]
        )

        plans = stress.build_plans(templates, 3, self.runtime())

        self.assertEqual(3, len(plans))
        self.assertTrue(
            all(plan["source_round"] == 37 for plan in plans)
        )

    def test_generated_case_is_accepted_by_both_runtime_helpers(self):
        template = stress.load_templates(
            self.manifest, ["collision_round_002"]
        )[0]
        plan = stress.build_plans([template], 1, self.runtime())[0]
        plan["task_id"] = "stress-test-task"
        case_file = self.root / "case_plan.json"
        case_file.write_text(json.dumps(plan), encoding="utf-8")

        scene = scene_helper.load_case(case_file)
        navigation = navigation_helper.load_case(case_file)

        self.assertEqual(10, len(scene["cones"]))
        self.assertEqual("stress-test-task", navigation["task_id"])
        self.assertAlmostEqual(0.5, navigation["switch_distance"])


if __name__ == "__main__":
    unittest.main()
