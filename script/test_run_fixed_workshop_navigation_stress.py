#!/usr/bin/env python3

import datetime as dt
from pathlib import Path
import unittest

import run_fixed_workshop_navigation_stress as stress


class FixedWorkshopNavigationStressTest(unittest.TestCase):
    def test_defaults_to_five_fixed_headless_recorded_rounds(self):
        args = stress.parse_args([])

        self.assertFalse(args.gui)
        self.assertEqual(
            ("electronics",) * 5,
            stress.TARGET_SEQUENCE,
        )
        self.assertEqual(stress.DEFAULT_RECORDING_ROOT, args.recording_root)

    def test_gui_flag_enables_gazebo_client(self):
        self.assertTrue(stress.parse_args(["--gui"]).gui)

    def test_command_uses_one_scene_and_exact_target_sequence(self):
        args = stress.parse_args([])
        command = stress.build_command(args, Path("/tmp/workshop-stress"))

        self.assertEqual(
            command[command.index("--template-id") + 1], stress.TEMPLATE_ID
        )
        self.assertEqual(
            command[command.index("--target-sequence") + 1],
            "electronics,electronics,electronics,electronics,electronics",
        )
        self.assertIn("--headless", command)
        self.assertNotIn("--gui", command)

    def test_default_log_directory_is_deterministic(self):
        value = stress.default_log_dir(dt.datetime(2026, 8, 22, 12, 34, 56))

        self.assertEqual(
            value.name, "fixed_workshop_navigation_stress_20260822_123456"
        )


if __name__ == "__main__":
    unittest.main()
