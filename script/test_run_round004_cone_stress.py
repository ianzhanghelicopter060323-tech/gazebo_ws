#!/usr/bin/env python3

from pathlib import Path
import unittest

import run_round004_cone_stress as stress


class Round004ConeStressTest(unittest.TestCase):
    def test_defaults_to_five_headless_recorded_rounds(self):
        args = stress.parse_args([])

        self.assertEqual(args.rounds, 5)
        self.assertFalse(args.gui)
        self.assertFalse(args.disable_gazebo_recording)

    def test_gui_flag_enables_gazebo_client(self):
        self.assertTrue(stress.parse_args(["--gui"]).gui)

    def test_command_is_bound_to_source_round_four(self):
        args = stress.parse_args([])
        command = stress.build_command(args, Path("/tmp/round004-test"))

        self.assertIn(str(stress.TEMPLATE), command)
        self.assertEqual(command[command.index("--source-round") + 1], "4")
        self.assertEqual(command[command.index("--rounds") + 1], "5")
        self.assertIn("--headless", command)
        self.assertNotIn("--gui", command)


if __name__ == "__main__":
    unittest.main()
