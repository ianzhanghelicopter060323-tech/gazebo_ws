#!/usr/bin/env python3

import json
from pathlib import Path
import tempfile
import types
import unittest
from unittest import mock

import record_gazebo_world as recording


class ProgressParsingTest(unittest.TestCase):
    def test_parses_action_feedback_progress(self):
        self.assertEqual(
            recording.parse_fitted_progress(
                "following fitted path at s=7.885m"
            ),
            7.885,
        )

    def test_parses_route_tracking_progress(self):
        self.assertEqual(
            recording.parse_fitted_progress(
                "tracking fitted path s=7.78/8.19m, error=0.007m"
            ),
            7.78,
        )

    def test_unrelated_state_has_no_progress(self):
        self.assertIsNone(
            recording.parse_fitted_progress("moving arm to fixed grasp pose")
        )


class RecorderTriggerTest(unittest.TestCase):
    def make_recorder(self, output_dir):
        recorder = recording.GazeboWorldRecorder.__new__(
            recording.GazeboWorldRecorder
        )
        recorder._args = types.SimpleNamespace(
            task_id="trial_001",
            start_progress=7.8585,
            start_stage=None,
            output_dir=Path(output_dir),
            encoding="zlib",
            world_name="default",
        )
        recorder._lock = __import__("threading").Lock()
        recorder._recording = False
        recorder._start_attempted = False
        recorder._shutdown_complete = False
        recorder._manifest_path = Path(output_dir) / "gazebo_world_recording.json"
        recorder._state_log_path = Path(output_dir) / "gazebo_world_state.log"
        recorder._manifest = {
            "status": "waiting_for_seq34",
            "trigger_progress_m": None,
            "started_at": None,
            "started_sim_time": None,
            "error": "",
        }
        recorder._write_manifest = mock.Mock()
        recorder._publish_control = mock.Mock()
        return recorder

    def state(self, task_id, detail, state=5):
        return types.SimpleNamespace(
            task_id=task_id,
            detail=detail,
            state=state,
            header=types.SimpleNamespace(
                stamp=types.SimpleNamespace(secs=123, nsecs=456)
            ),
        )

    @mock.patch.object(recording.rospy, "loginfo")
    def test_starts_once_after_crossing_seq34(self, _loginfo):
        with tempfile.TemporaryDirectory() as output_dir:
            recorder = self.make_recorder(output_dir)
            recorder._state_callback(
                self.state("trial_001", "following fitted path at s=7.723m")
            )
            recorder._publish_control.assert_not_called()

            recorder._state_callback(
                self.state("trial_001", "following fitted path at s=7.885m")
            )
            recorder._state_callback(
                self.state("trial_001", "following fitted path at s=7.950m")
            )

            recorder._publish_control.assert_called_once()
            payload = recorder._publish_control.call_args.args[0]
            self.assertIn("start: true", payload)
            self.assertTrue(recorder._recording)
            self.assertEqual(recorder._manifest["status"], "recording")
            self.assertEqual(recorder._manifest["trigger_progress_m"], 7.885)

    def test_ignores_another_task(self):
        with tempfile.TemporaryDirectory() as output_dir:
            recorder = self.make_recorder(output_dir)
            recorder._state_callback(
                self.state("trial_999", "following fitted path at s=8.000m")
            )
            recorder._publish_control.assert_not_called()

    @mock.patch.object(recording.rospy, "loginfo")
    def test_can_start_on_first_navigation_stage(self, _loginfo):
        with tempfile.TemporaryDirectory() as output_dir:
            recorder = self.make_recorder(output_dir)
            recorder._args.start_progress = None
            recorder._args.start_stage = 5

            recorder._state_callback(self.state("trial_001", "ready", state=3))
            recorder._publish_control.assert_not_called()
            recorder._state_callback(
                self.state("trial_001", "navigating", state=5)
            )

            recorder._publish_control.assert_called_once()
            self.assertEqual(recorder._manifest["trigger_task_state"], 5)
            self.assertIsNone(recorder._manifest["trigger_progress_m"])


class RecordingNormalizationTest(unittest.TestCase):
    def test_moves_gazebo_nested_state_log_into_round_directory(self):
        with tempfile.TemporaryDirectory() as output_dir:
            output_path = Path(output_dir)
            nested = output_path / "timestamp" / "gzserver"
            nested.mkdir(parents=True)
            source = nested / "state.log"
            source.write_bytes(b"gazebo-state")

            recorder = recording.GazeboWorldRecorder.__new__(
                recording.GazeboWorldRecorder
            )
            recorder._args = types.SimpleNamespace(output_dir=output_path)
            recorder._state_log_path = output_path / "gazebo_world_state.log"
            recorder._find_recorded_state_log = mock.Mock(return_value=source)

            size = recorder._normalize_state_log()

            self.assertEqual(size, len(b"gazebo-state"))
            self.assertEqual(
                recorder._state_log_path.read_bytes(), b"gazebo-state"
            )
            self.assertFalse(source.exists())

    def test_waits_for_a_silent_status_interval_after_stop(self):
        recorder = recording.GazeboWorldRecorder.__new__(
            recording.GazeboWorldRecorder
        )
        recorder._args = types.SimpleNamespace(world_name="default")
        status = types.SimpleNamespace(returncode=0, stdout="sim_time { sec: 1 }")
        silent = types.SimpleNamespace(returncode=0, stdout="")
        with mock.patch.object(
            recording.subprocess, "run", side_effect=(status, silent)
        ) as run:
            recorder._wait_until_recording_stops(timeout=5.0)
        self.assertEqual(run.call_count, 2)


if __name__ == "__main__":
    unittest.main()
