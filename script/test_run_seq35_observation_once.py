#!/usr/bin/env python3

from pathlib import Path
import tempfile
import unittest

import run_seq35_observation_once as once


class Seq35ObservationOnceTest(unittest.TestCase):
    def test_defaults_are_one_gui_run_held_after_arrival(self):
        args = once.parse_args([])

        self.assertTrue(args.gui)
        self.assertFalse(args.exit_after_arrival)
        self.assertFalse(hasattr(args, "rounds"))

    def test_reads_formal_seq35_pose(self):
        with tempfile.TemporaryDirectory() as temporary:
            mission = Path(temporary) / "mission.yaml"
            mission.write_text(
                "pickup:\n"
                "  stations:\n"
                "    - {number: 35, x: -1.26, y: -0.525, yaw: 0.0}\n",
                encoding="utf-8",
            )

            self.assertEqual(
                once.read_seq35_pose(mission),
                {"x": -1.26, "y": -0.525, "yaw": 0.0},
            )


if __name__ == "__main__":
    unittest.main()
