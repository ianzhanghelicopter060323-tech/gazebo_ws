#!/usr/bin/env python3

import math
import threading
import types
import unittest
from unittest import mock

import record_cone_trial as recording


class ContactStreamParserTest(unittest.TestCase):
    def test_extracts_car_to_cone_contact(self):
        pairs = []
        parser = recording.ContactStreamParser(
            lambda first, second: pairs.append((first, second))
        )
        for line in (
            "contact {",
            '  collision1: "car3::wheel_lf_link::collision"',
            '  collision2: "cone_14::link::collision"',
            "}",
        ):
            parser.feed_line(line)

        self.assertEqual(len(pairs), 1)
        self.assertEqual(recording.car_cone_pair(*pairs[0]), "cone_14")

    def test_accepts_reversed_contact_and_rejects_ground(self):
        self.assertEqual(
            recording.car_cone_pair(
                "cone_10::link::collision", "car3::base_link::collision"
            ),
            "cone_10",
        )
        self.assertIsNone(
            recording.car_cone_pair(
                "ground_plane::link::collision", "car3::wheel::collision"
            )
        )


class PoseMathTest(unittest.TestCase):
    @staticmethod
    def orientation(x=0.0, y=0.0, z=0.0, w=1.0):
        return types.SimpleNamespace(x=x, y=y, z=z, w=w)

    def test_tilt_is_zero_for_yaw_only(self):
        yaw = 1.2
        orientation = self.orientation(
            z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0)
        )
        self.assertAlmostEqual(recording.tilt_from_quaternion(orientation), 0.0)
        self.assertAlmostEqual(recording.yaw_from_quaternion(orientation), yaw)

    def test_stage_phase_changes_after_grasp(self):
        self.assertEqual(recording.stage_phase(5), "pre_pickup")
        self.assertEqual(recording.stage_phase(14), "delivery")
        self.assertEqual(recording.stage_phase(250), "task_failed")


class ContactStreamShutdownTest(unittest.TestCase):
    def test_already_stopped_child_converges_manifest_status(self):
        recorder = recording.ConeTrialRecorder.__new__(
            recording.ConeTrialRecorder
        )
        recorder._contact_process = types.SimpleNamespace(poll=lambda: -2)
        recorder._contact_thread = mock.Mock()
        recorder._lock = threading.RLock()
        recorder._shutdown_complete = True
        recorder._contact_stream_status = "running"
        recorder._contact_stream_error = ""

        recorder._stop_contact_stream()

        recorder._contact_thread.join.assert_called_once_with(timeout=1.0)
        self.assertEqual(recorder._contact_stream_status, "stopped")
        self.assertEqual(recorder._contact_stream_error, "")


if __name__ == "__main__":
    unittest.main()
