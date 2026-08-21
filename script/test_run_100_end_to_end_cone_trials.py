#!/usr/bin/env python3

import datetime as dt
from pathlib import Path
import unittest

import run_100_end_to_end_cone_trials as trials


class EndToEnd100WrapperTest(unittest.TestCase):
    def test_defaults_to_100_headless_recorded_rounds(self):
        args = trials.parse_args([])

        self.assertEqual(100, trials.ROUNDS)
        self.assertFalse(args.gui)
        self.assertEqual(trials.DEFAULT_RECORDING_ROOT, args.recording_root)

    def test_gui_flag_is_supported(self):
        self.assertTrue(trials.parse_args(["--gui"]).gui)

    def test_identity_uses_explicit_seed_and_cleanup_compatible_names(self):
        args = trials.parse_args(["--seed", "123"])

        seed, log_name, recording_name = trials.build_run_identity(
            args, dt.datetime(2026, 8, 22, 13, 14, 15)
        )

        self.assertEqual(123, seed)
        self.assertEqual("end_to_end_cone_trials_20260822_131415", log_name)
        self.assertEqual(log_name + "_seed123", recording_name)

    def test_command_fixes_round_count_and_defaults_to_headless(self):
        args = trials.parse_args([])
        command = trials.build_command(
            args, 456, "end_to_end_cone_trials_20260822_131415"
        )

        self.assertEqual(command[command.index("--rounds") + 1], "100")
        self.assertEqual(command[command.index("--seed") + 1], "456")
        self.assertIn("--headless", command)
        self.assertNotIn("--gui", command)


if __name__ == "__main__":
    unittest.main()
