#!/usr/bin/env python3

import csv
import json
from pathlib import Path
import tempfile
import types
import unittest

import prune_successful_gazebo_recordings as cleanup


class RecordingCleanupTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        root = Path(self.temporary.name)
        self.data_root = root / "data"
        self.logs_root = root / "logs"
        self.run_name = "grasp_trials_20260809_120000_seed12345"
        self.log_name = "grasp_trials_20260809_120000"
        self.run_dir = self.data_root / self.run_name
        self.log_dir = self.logs_root / self.log_name
        self.run_dir.mkdir(parents=True)
        self.log_dir.mkdir(parents=True)

        with (self.log_dir / "trials.csv").open(
            "w", encoding="utf-8", newline=""
        ) as csv_file:
            writer = csv.DictWriter(
                csv_file,
                fieldnames=("round", "recognition_correct", "success"),
            )
            writer.writeheader()
            writer.writerow(
                {"round": 1, "recognition_correct": "True", "success": "True"}
            )
            writer.writerow(
                {"round": 2, "recognition_correct": "False", "success": "True"}
            )

        for round_number in (1, 2, 3):
            round_dir = self.run_dir / "round_{:03d}".format(round_number)
            round_dir.mkdir()
            (round_dir / "gazebo_world_state.log").write_bytes(b"recording")
            (round_dir / "gazebo_world_recording.json").write_text("{}")
            (round_dir / "gazebo_world_recorder.log").write_text("helper")
            (round_dir / "photos.json").write_text("{}")
            (round_dir / "seq35.png").write_bytes(b"photo")

    def tearDown(self):
        self.temporary.cleanup()

    def args(self, apply):
        return types.SimpleNamespace(
            data_root=self.data_root,
            logs_root=self.logs_root,
            runs=None,
            apply=apply,
        )

    def test_dry_run_does_not_delete_anything(self):
        _root, results = cleanup.process(self.args(apply=False))

        self.assertTrue(results[0]["eligible_for_recording_deletion"])
        self.assertFalse(results[1]["eligible_for_recording_deletion"])
        self.assertEqual(results[2]["reason"], "trial_result_missing")
        for round_number in (1, 2, 3):
            self.assertTrue(
                (self.run_dir / "round_{:03d}".format(round_number)
                 / "gazebo_world_state.log").is_file()
            )

    def test_apply_only_deletes_successful_round_recordings(self):
        _root, results = cleanup.process(self.args(apply=True))

        successful_round = self.run_dir / "round_001"
        for filename in cleanup.RECORDING_FILES:
            self.assertFalse((successful_round / filename).exists())
        self.assertTrue((successful_round / "photos.json").is_file())
        self.assertTrue((successful_round / "seq35.png").is_file())

        for round_number in (2, 3):
            retained_round = self.run_dir / "round_{:03d}".format(round_number)
            for filename in cleanup.RECORDING_FILES:
                self.assertTrue((retained_round / filename).is_file())
        self.assertEqual(len(results[0]["deleted_files"]), 3)

    def test_strict_boolean_parser_does_not_treat_unknown_as_success(self):
        self.assertIs(cleanup.parse_strict_bool("TRUE"), True)
        self.assertIs(cleanup.parse_strict_bool("false"), False)
        self.assertIsNone(cleanup.parse_strict_bool("correct"))

    def test_watch_signature_changes_when_result_becomes_successful(self):
        missing = cleanup.evaluate_round(
            self.run_dir,
            self.run_dir / "round_003",
            None,
        )
        successful = cleanup.evaluate_round(
            self.run_dir,
            self.run_dir / "round_003",
            {"recognition_correct": "True", "success": "True"},
        )
        self.assertNotEqual(
            cleanup.watch_signature(missing),
            cleanup.watch_signature(successful),
        )


class ConeRecordingCleanupTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        root = Path(self.temporary.name)
        self.legacy_root = root / "missing_legacy"
        self.video_root = root / "cone_videos"
        self.logs_root = root / "logs"
        self.log_name = "end_to_end_cone_trials_20260810_120000"
        self.run_dir = self.video_root / (self.log_name + "_seed24680")
        self.log_dir = self.logs_root / self.log_name
        self.run_dir.mkdir(parents=True)
        self.log_dir.mkdir(parents=True)

        fields = (
            "round", "task_id", "success", "status", "completed_stage",
            "error_code", "preceding_navigation_failure",
            "delivery_navigation_failure", "cone_collision",
            "cone_monitor_status", "contact_stream_status",
            "gazebo_recording_status", "gazebo_recording_path",
            "gazebo_recording_size_bytes",
        )
        rows = []
        for round_number in (1, 2, 3, 4):
            round_dir = self.run_dir / "round_{:03d}".format(round_number)
            round_dir.mkdir()
            recording_path = (round_dir / "gazebo_world_state.log").resolve()
            recording_path.write_bytes(b"gazebo-state")
            manifest = {
                "task_id": "task_{}".format(round_number),
                "status": "complete",
                "recording_file": recording_path.name,
                "size_bytes": recording_path.stat().st_size,
            }
            (round_dir / "gazebo_world_recording.json").write_text(
                json.dumps(manifest), encoding="utf-8"
            )
            (round_dir / "gazebo_world_recorder.log").write_text(
                "recorder", encoding="utf-8"
            )
            rows.append(
                {
                    "round": round_number,
                    "task_id": "task_{}".format(round_number),
                    "success": "True",
                    "status": "task_completed",
                    "completed_stage": 20,
                    "error_code": 0,
                    "preceding_navigation_failure": "False",
                    "delivery_navigation_failure": "False",
                    "cone_collision": "False",
                    "cone_monitor_status": "complete",
                    "contact_stream_status": "stopped",
                    "gazebo_recording_status": "complete",
                    "gazebo_recording_path": str(recording_path),
                    "gazebo_recording_size_bytes": recording_path.stat().st_size,
                }
            )
        rows[1]["cone_collision"] = "True"
        rows[2]["preceding_navigation_failure"] = "True"
        rows[3]["cone_monitor_status"] = ""

        with (self.log_dir / "trials.csv").open(
            "w", encoding="utf-8", newline=""
        ) as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fields)
            writer.writeheader()
            writer.writerows(rows)

    def tearDown(self):
        self.temporary.cleanup()

    def args(self, apply):
        return types.SimpleNamespace(
            data_root=self.legacy_root,
            cone_video_root=self.video_root,
            logs_root=self.logs_root,
            runs=None,
            apply=apply,
        )

    def test_only_a_fully_clean_round_is_eligible(self):
        _root, results = cleanup.process(self.args(apply=False), emit=False)

        self.assertEqual(len(results), 4)
        self.assertTrue(results[0]["eligible_for_recording_deletion"])
        self.assertEqual(results[1]["reason"], "cone_collision_or_unknown")
        self.assertEqual(
            results[2]["reason"], "preceding_navigation_failed_or_unknown"
        )
        self.assertEqual(results[3]["reason"], "cone_monitor_incomplete")

    def test_apply_deletes_only_clean_round_video_artifacts(self):
        _root, results = cleanup.process(self.args(apply=True), emit=False)

        for filename in cleanup.CONE_RECORDING_FILES:
            self.assertFalse((self.run_dir / "round_001" / filename).exists())
        for round_number in (2, 3, 4):
            for filename in cleanup.CONE_RECORDING_FILES:
                self.assertTrue(
                    (self.run_dir / "round_{:03d}".format(round_number)
                     / filename).is_file()
                )
        self.assertEqual(len(results[0]["deleted_files"]), 3)


class FixedConeEndToEndCleanupTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        root = Path(self.temporary.name)
        self.fixed_root = root / "fixed_e2e"
        self.logs_root = root / "logs"
        self.run_name = (
            "fixed_cone_e2e_stress_20260812_120000_global_inf_0p20_cf_0p55"
        )
        self.run_dir = self.fixed_root / self.run_name
        self.log_dir = self.logs_root / self.run_name
        self.round_dir = self.run_dir / "round_001"
        self.round_dir.mkdir(parents=True)
        self.log_dir.mkdir(parents=True)

        recording = (self.round_dir / "gazebo_world_state.log").resolve()
        recording.write_bytes(b"gazebo-state")
        (self.round_dir / "gazebo_world_recording.json").write_text(
            json.dumps(
                {
                    "task_id": "fixed_001",
                    "status": "complete",
                    "recording_file": recording.name,
                    "size_bytes": recording.stat().st_size,
                }
            ),
            encoding="utf-8",
        )
        (self.round_dir / "gazebo_world_recorder.log").write_text(
            "recorder", encoding="utf-8"
        )
        fields = (
            "round", "task_id", "success", "status", "completed_stage",
            "error_code", "preceding_navigation_failure",
            "delivery_navigation_failure", "cone_collision",
            "cone_monitor_status", "contact_stream_status",
            "gazebo_recording_status", "gazebo_recording_path",
            "gazebo_recording_size_bytes",
        )
        with (self.log_dir / "trials.csv").open(
            "w", encoding="utf-8", newline=""
        ) as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fields)
            writer.writeheader()
            writer.writerow(
                {
                    "round": 1,
                    "task_id": "fixed_001",
                    "success": "True",
                    "status": "task_completed",
                    "completed_stage": 20,
                    "error_code": 0,
                    "preceding_navigation_failure": "False",
                    "delivery_navigation_failure": "False",
                    "cone_collision": "False",
                    "cone_monitor_status": "complete",
                    "contact_stream_status": "stopped",
                    "gazebo_recording_status": "complete",
                    "gazebo_recording_path": str(recording),
                    "gazebo_recording_size_bytes": recording.stat().st_size,
                }
            )

    def tearDown(self):
        self.temporary.cleanup()

    def args(self, apply):
        root = Path(self.temporary.name)
        return types.SimpleNamespace(
            data_root=root / "missing_legacy",
            cone_video_root=root / "missing_cone",
            cone_stress_root=root / "missing_stress",
            pre_navigation_root=root / "missing_pre_navigation",
            fixed_cone_e2e_root=self.fixed_root,
            logs_root=self.logs_root,
            runs=None,
            apply=apply,
        )

    def test_labeled_run_is_recognized_and_uses_full_task_checks(self):
        self.assertEqual(
            cleanup.fixed_cone_e2e_log_run_name(self.run_name), self.run_name
        )

        _root, results = cleanup.process(self.args(apply=False), emit=False)

        self.assertEqual(len(results), 1)
        self.assertEqual(results[0]["dataset"], "fixed_cone_e2e_world")
        self.assertTrue(results[0]["eligible_for_recording_deletion"])

    def test_apply_deletes_only_recording_artifacts(self):
        _root, results = cleanup.process(self.args(apply=True), emit=False)

        self.assertEqual(len(results[0]["deleted_files"]), 3)
        for filename in cleanup.CONE_RECORDING_FILES:
            self.assertFalse((self.round_dir / filename).exists())

    def test_targeted_report_root_follows_existing_fixed_run(self):
        args = self.args(apply=False)
        unrelated_root = Path(self.temporary.name) / "existing_cone_stress"
        unrelated_root.mkdir()
        args.cone_stress_root = unrelated_root
        args.runs = [self.run_name]

        report_root, _results = cleanup.process(args, emit=False)

        self.assertEqual(report_root, self.fixed_root.resolve())


class ConeStressRecordingCleanupTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        root = Path(self.temporary.name)
        self.legacy_root = root / "missing_legacy"
        self.cone_root = root / "missing_cone"
        self.stress_root = root / "cone_stress"
        self.logs_root = root / "logs"
        self.run_name = "cone_move_stress_20260811_120000"
        self.run_dir = self.stress_root / self.run_name
        self.log_dir = self.logs_root / self.run_name
        self.run_dir.mkdir(parents=True)
        self.log_dir.mkdir(parents=True)

        fields = (
            "round", "task_id", "success", "status", "completed_stage",
            "error_code", "navigation_failure", "profile_switched",
            "cone_collision", "cone_monitor_status", "contact_stream_status",
            "gazebo_recording_status", "gazebo_recording_path",
            "gazebo_recording_size_bytes",
        )
        rows = []
        for round_number in (1, 2, 3, 4):
            round_dir = self.run_dir / "round_{:03d}".format(round_number)
            round_dir.mkdir()
            recording = (round_dir / "gazebo_world_state.log").resolve()
            recording.write_bytes(b"gazebo-state")
            (round_dir / "gazebo_world_recording.json").write_text(
                json.dumps(
                    {
                        "task_id": "stress_{}".format(round_number),
                        "status": "complete",
                        "recording_file": recording.name,
                        "size_bytes": recording.stat().st_size,
                    }
                ),
                encoding="utf-8",
            )
            (round_dir / "gazebo_world_recorder.log").write_text(
                "recorder", encoding="utf-8"
            )
            rows.append(
                {
                    "round": round_number,
                    "task_id": "stress_{}".format(round_number),
                    "success": "True",
                    "status": "navigation_completed",
                    "completed_stage": 17,
                    "error_code": 0,
                    "navigation_failure": "False",
                    "profile_switched": "True",
                    "cone_collision": "False",
                    "cone_monitor_status": "complete",
                    "contact_stream_status": "stopped",
                    "gazebo_recording_status": "complete",
                    "gazebo_recording_path": str(recording),
                    "gazebo_recording_size_bytes": recording.stat().st_size,
                }
            )
        rows[1]["cone_collision"] = "True"
        rows[2]["navigation_failure"] = "True"
        rows[3]["profile_switched"] = "False"

        with (self.log_dir / "trials.csv").open(
            "w", encoding="utf-8", newline=""
        ) as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fields)
            writer.writeheader()
            writer.writerows(rows)

    def tearDown(self):
        self.temporary.cleanup()

    def args(self, apply):
        return types.SimpleNamespace(
            data_root=self.legacy_root,
            cone_video_root=self.cone_root,
            cone_stress_root=self.stress_root,
            logs_root=self.logs_root,
            runs=None,
            apply=apply,
        )

    def test_only_clean_navigation_is_eligible(self):
        _root, results = cleanup.process(self.args(apply=False), emit=False)

        self.assertEqual(len(results), 4)
        self.assertTrue(results[0]["eligible_for_recording_deletion"])
        self.assertEqual(results[1]["reason"], "cone_collision_or_unknown")
        self.assertEqual(results[2]["reason"], "navigation_failed_or_unknown")
        self.assertEqual(
            results[3]["reason"], "automatic_profile_switch_missing"
        )

    def test_apply_deletes_only_clean_stress_recording(self):
        _root, results = cleanup.process(self.args(apply=True), emit=False)

        for filename in cleanup.CONE_RECORDING_FILES:
            self.assertFalse((self.run_dir / "round_001" / filename).exists())
        for round_number in (2, 3, 4):
            for filename in cleanup.CONE_RECORDING_FILES:
                self.assertTrue(
                    (self.run_dir / "round_{:03d}".format(round_number)
                     / filename).is_file()
                )
        self.assertEqual(len(results[0]["deleted_files"]), 3)


class PreNavigationRecordingCleanupTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        root = Path(self.temporary.name)
        self.legacy_root = root / "missing_legacy"
        self.cone_root = root / "missing_cone"
        self.stress_root = root / "missing_stress"
        self.pre_navigation_root = root / "pre_navigation"
        self.logs_root = root / "logs"
        self.run_name = "pre_navigation_trials_20260812_120000_ros1132"
        self.run_dir = self.pre_navigation_root / self.run_name
        self.log_dir = self.logs_root / self.run_name
        self.run_dir.mkdir(parents=True)
        self.log_dir.mkdir(parents=True)

        fields = (
            "round", "task_id", "success", "status", "error_code",
            "server_error_code",
            "acceptance_mode", "yaw_alignment_required",
            "completed_waypoints", "waypoint_count",
            "final_position_error_m", "final_position_tolerance_m",
            "gazebo_recording_status", "gazebo_recording_path",
            "gazebo_recording_size_bytes",
        )
        rows = []
        for round_number in (1, 2, 3, 4):
            round_dir = self.run_dir / "round_{:03d}".format(round_number)
            round_dir.mkdir()
            recording = (round_dir / "gazebo_world_state.log").resolve()
            recording.write_bytes(b"gazebo-state")
            (round_dir / "gazebo_world_recording.json").write_text(
                json.dumps(
                    {
                        "task_id": "pre_nav_{}".format(round_number),
                        "status": "complete",
                        "start_condition": {"type": "immediate"},
                        "recording_file": recording.name,
                        "size_bytes": recording.stat().st_size,
                    }
                ),
                encoding="utf-8",
            )
            (round_dir / "gazebo_world_recorder.log").write_text(
                "recorder", encoding="utf-8"
            )
            rows.append(
                {
                    "round": round_number,
                    "task_id": "pre_nav_{}".format(round_number),
                    "success": "True",
                    "status": "navigation_completed",
                    "error_code": 0,
                    "server_error_code": 0,
                    "acceptance_mode": "position_tolerance",
                    "yaw_alignment_required": "False",
                    "completed_waypoints": 30,
                    "waypoint_count": 30,
                    "final_position_error_m": 0.149,
                    "final_position_tolerance_m": 0.15,
                    "gazebo_recording_status": "complete",
                    "gazebo_recording_path": str(recording),
                    "gazebo_recording_size_bytes": recording.stat().st_size,
                }
            )
        rows[1]["success"] = "False"
        rows[1]["status"] = "navigation_failed"
        rows[2]["final_position_error_m"] = 0.151
        rows[3]["server_error_code"] = 7

        with (self.log_dir / "trials.csv").open(
            "w", encoding="utf-8", newline=""
        ) as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fields)
            writer.writeheader()
            writer.writerows(rows)

    def tearDown(self):
        self.temporary.cleanup()

    def args(self, apply):
        return types.SimpleNamespace(
            data_root=self.legacy_root,
            cone_video_root=self.cone_root,
            cone_stress_root=self.stress_root,
            pre_navigation_root=self.pre_navigation_root,
            logs_root=self.logs_root,
            runs=None,
            apply=apply,
        )

    def test_only_successful_position_accepted_round_is_eligible(self):
        _root, results = cleanup.process(self.args(apply=False), emit=False)

        self.assertEqual(len(results), 4)
        self.assertTrue(results[0]["eligible_for_recording_deletion"])
        self.assertEqual(results[1]["reason"], "navigation_not_successful")
        self.assertEqual(
            results[2]["reason"], "seq35_position_tolerance_not_met"
        )
        self.assertEqual(
            results[3]["reason"], "navigation_server_error_code_not_zero"
        )

    def test_apply_deletes_only_successful_pre_navigation_recording(self):
        _root, results = cleanup.process(self.args(apply=True), emit=False)

        for filename in cleanup.RECORDING_FILES:
            self.assertFalse((self.run_dir / "round_001" / filename).exists())
        for round_number in (2, 3, 4):
            for filename in cleanup.RECORDING_FILES:
                self.assertTrue(
                    (self.run_dir / "round_{:03d}".format(round_number)
                     / filename).is_file()
                )
        self.assertEqual(len(results[0]["deleted_files"]), 3)

    def test_uses_result_table_mirrored_beside_recordings(self):
        source = self.log_dir / "trials.csv"
        mirrored = self.run_dir / "trials.csv"
        mirrored.write_text(source.read_text(encoding="utf-8"), encoding="utf-8")
        source.unlink()

        _root, results = cleanup.process(self.args(apply=False), emit=False)

        self.assertTrue(results[0]["eligible_for_recording_deletion"])
        self.assertEqual(results[1]["reason"], "navigation_not_successful")


if __name__ == "__main__":
    unittest.main()
