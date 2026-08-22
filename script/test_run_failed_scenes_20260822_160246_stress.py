#!/usr/bin/env python3

from collections import Counter
import datetime as dt
from pathlib import Path
import unittest

import run_failed_scenes_20260822_160246_stress as stress
import run_fixed_cone_e2e_stress_trials as harness
from run_cone_move_stress_trials import load_runtime_config


class FailedSceneStressTest(unittest.TestCase):
    def test_defaults_to_two_headless_recorded_rounds_per_scene(self):
        args = stress.parse_args([])

        self.assertFalse(args.gui)
        self.assertFalse(args.disable_gazebo_recording)
        self.assertEqual(stress.REPETITIONS_PER_SCENE, 2)
        self.assertEqual(stress.TOTAL_ROUNDS, 18)
        self.assertEqual(
            stress.SOURCE_ROUNDS, (11, 24, 37, 41, 50, 51, 55, 57, 58)
        )

    def test_gui_flag_enables_gazebo_client(self):
        self.assertTrue(stress.parse_args(["--gui"]).gui)

    def test_command_selects_every_scene_and_exact_total(self):
        args = stress.parse_args([])
        command = stress.build_command(args, Path("/tmp/failed-scenes"))

        selected = [
            int(command[index + 1])
            for index, value in enumerate(command)
            if value == "--source-round"
        ]
        self.assertEqual(selected, list(stress.SOURCE_ROUNDS))
        self.assertEqual(
            command[command.index("--rounds") + 1],
            str(stress.TOTAL_ROUNDS),
        )
        self.assertIn("--headless", command)
        self.assertNotIn("--gui", command)

    def test_manifest_schedule_repeats_each_scene_twice(self):
        templates = harness.load_templates(
            stress.TEMPLATE_MANIFEST, list(stress.SOURCE_ROUNDS)
        )
        plans = harness.build_plans(
            templates, stress.TOTAL_ROUNDS, load_runtime_config()
        )

        self.assertEqual(len(plans), stress.TOTAL_ROUNDS)
        self.assertEqual(
            Counter(plan["source_round"] for plan in plans),
            Counter({source_round: 2 for source_round in stress.SOURCE_ROUNDS}),
        )

    def test_default_log_directory_contains_source_timestamp(self):
        value = stress.default_log_dir(dt.datetime(2026, 8, 22, 23, 45, 6))

        self.assertEqual(
            value.name,
            "failed_scenes_20260822_160246_stress_20260822_234506",
        )


if __name__ == "__main__":
    unittest.main()
