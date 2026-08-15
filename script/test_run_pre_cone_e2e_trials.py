#!/usr/bin/env python3

import csv
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

import run_pre_cone_e2e_trials as trials


class PreConeConfigurationTest(unittest.TestCase):
    def test_defaults_match_hundred_round_pre_cone_request(self):
        args = trials.parse_args([])

        self.assertEqual(args.rounds, 100)
        self.assertFalse(args.gui)
        self.assertEqual(args.progress_timeout, 90.0)
        self.assertEqual(args.recording_root, trials.DEFAULT_RECORDING_ROOT)

    def test_generated_mission_stops_after_object_grasped(self):
        with tempfile.TemporaryDirectory() as temporary, mock.patch.object(
            trials, "DEFAULT_MISSION", Path(temporary) / "source.yaml"
        ):
            trials.DEFAULT_MISSION.write_text(
                "pipeline_stop_after: TASK_COMPLETED\npickup:\n  enabled: true\n",
                encoding="utf-8",
            )
            output_dir = Path(temporary) / "output"
            output_dir.mkdir()

            generated = trials.prepare_pre_cone_mission(output_dir)

            self.assertIn(
                "pipeline_stop_after: OBJECT_GRASPED",
                generated.read_text(encoding="utf-8"),
            )
            self.assertIn(
                "pipeline_stop_after: TASK_COMPLETED",
                trials.DEFAULT_MISSION.read_text(encoding="utf-8"),
            )


class PreConeClassificationTest(unittest.TestCase):
    def base_record(self):
        return {
            "recognition_correct": False,
            "recognition_status": "no_target_selected",
            "last_operational_stage": None,
            "completed_stage": None,
            "error_code": None,
            "status": "not_started",
        }

    def test_progress_timeout_marker_and_last_stage_are_parsed(self):
        output = (
            "noise\n"
            "stage=5 retry=0 detail=waypoint active\n"
            'TASK_PROGRESS_TIMEOUT={"inactive_seconds": 90.1}\n'
        )

        self.assertEqual(trials.last_feedback_stage(output), 5)
        self.assertEqual(
            trials.parse_progress_timeout(output)["inactive_seconds"], 90.1
        )

    def test_recognition_failure_requires_an_attempt(self):
        record = self.base_record()
        self.assertIsNone(trials.classify_recognition_failure(record))

        record["last_operational_stage"] = 9
        self.assertIs(trials.classify_recognition_failure(record), True)

    def test_correct_recognition_is_explicitly_clean(self):
        record = self.base_record()
        record["recognition_correct"] = True
        record["recognition_status"] = "correct"

        self.assertIs(trials.classify_recognition_failure(record), False)

    def test_navigation_timeout_is_classified_as_stuck(self):
        record = self.base_record()
        record["status"] = "task_failed"
        record["error_code"] = 6

        self.assertEqual(
            trials.classify_stuck(record), (True, "navigation_timeout")
        )

    def test_terminal_non_navigation_failure_is_not_stuck(self):
        record = self.base_record()
        record["status"] = "task_failed"
        record["error_code"] = 13

        self.assertEqual(trials.classify_stuck(record), (False, ""))


class PreConeReportTest(unittest.TestCase):
    def test_reports_are_mirrored_beside_recordings(self):
        args = trials.parse_args(["--rounds", "1"])
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            logs = root / "logs"
            recordings = root / "recordings"
            logs.mkdir()
            recordings.mkdir()
            metadata = {"task_seed": 123}

            trials.write_reports(logs, recordings, [], args, metadata)

            with (recordings / "trials.csv").open(
                "r", encoding="utf-8", newline=""
            ) as stream:
                self.assertEqual(list(csv.DictReader(stream)), [])
            report = json.loads(
                (recordings / "summary.json").read_text(encoding="utf-8")
            )
            self.assertEqual(report["summary"]["requested_rounds"], 1)


if __name__ == "__main__":
    unittest.main()
