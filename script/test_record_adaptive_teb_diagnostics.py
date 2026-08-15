#!/usr/bin/env python3

import unittest

import record_adaptive_teb_diagnostics as recorder


class AdaptiveTebDiagnosticsParsingTest(unittest.TestCase):
    def test_mode_message_fields_are_parsed(self):
        fields = recorder.parse_fields(
            "mode=avoidance reason=laser_obstacle_evidence "
            "compact_clusters=2 clearance=0.147"
        )

        self.assertEqual(fields["mode"], "avoidance")
        self.assertEqual(fields["reason"], "laser_obstacle_evidence")
        self.assertEqual(fields["compact_clusters"], "2")
        self.assertAlmostEqual(float(fields["clearance"]), 0.147)

    def test_non_finite_clearance_is_not_accepted(self):
        self.assertIsNone(recorder.finite_float("inf"))
        self.assertIsNone(recorder.finite_float("not-a-number"))
        self.assertAlmostEqual(recorder.finite_float("0.25"), 0.25)


if __name__ == "__main__":
    unittest.main()
