#!/usr/bin/env python3

import csv
import json
from pathlib import Path
import tempfile
import types
import unittest

import prune_successful_gazebo_recordings as cleanup


class TargetedWatcherCompletionTest(unittest.TestCase):
    def test_only_finalized_rows_count_toward_automatic_stop(self):
        results = [
            {
                "run": "selected_run",
                "round": 1,
                "reason": "error_free_end_to_end_round",
            },
            {
                "run": "selected_run",
                "round": 2,
                "reason": "trial_result_missing",
            },
            {
                "run": "another_run",
                "round": 3,
                "reason": "task_not_successful",
            },
        ]

        self.assertEqual(
            cleanup.finalized_target_rounds(results, "selected_run"), {1}
        )

    def test_stop_after_rounds_arguments_are_parsed(self):
        args = cleanup.parse_args(
            [
                "--watch",
                "--run",
                "selected_run",
                "--stop-after-rounds",
                "200",
            ]
        )

        self.assertTrue(args.watch)
        self.assertEqual(args.runs, ["selected_run"])
        self.assertEqual(args.stop_after_rounds, 200)


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
            "gazebo_recording_size_bytes", "log_file",
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
            log_round_dir = self.log_dir / "round_{:03d}".format(round_number)
            log_round_dir.mkdir()
            log_file = log_round_dir / "roslaunch.log"
            log_file.write_text(
                "clean round; no recovery events\n", encoding="utf-8"
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
                    "log_file": str(log_file),
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

    def test_uses_result_table_mirrored_beside_normal_recordings(self):
        (self.log_dir / "trials.csv").replace(self.run_dir / "trials.csv")

        _root, results = cleanup.process(self.args(apply=False), emit=False)

        self.assertEqual(len(results), 4)
        self.assertTrue(results[0]["eligible_for_recording_deletion"])
        self.assertEqual(results[1]["reason"], "cone_collision_or_unknown")

    def test_normal_e2e_root_argument_alias(self):
        args = cleanup.parse_args(
            ["--normal-e2e-root", str(self.video_root)]
        )

        self.assertEqual(args.cone_video_root, self.video_root)

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

    def test_clean_round_with_recovery_marker_is_still_deleted_but_annotated(self):
        log_path = self.log_dir / "round_001" / "roslaunch.log"
        log_path.write_text(
            "AdaptiveTebLocalPlannerROS initialized: ok\n"
            "bounded navigation recovery completed\n"
            "trajectory is not feasible. Resetting planner\n"
            "possible oscillation (of the robot or its local plan) detected\n"
            "oscillation recovery disabled/expired\n",
            encoding="utf-8",
        )
        _root, results = cleanup.process(self.args(apply=True), emit=False)

        self.assertTrue(results[0]["eligible_for_recording_deletion"])
        self.assertTrue(results[0]["deleted_files"])
        self.assertEqual(results[0]["recovery_event_count"], 4)
        self.assertEqual(
            results[0]["recovery_markers"],
            [
                "bounded navigation recovery completed",
                "trajectory is not feasible. Resetting planner",
                "possible oscillation (of the robot or its local plan) detected",
                "oscillation recovery disabled/expired",
            ],
        )

    def test_recovery_annotation_reads_log_file_column(self):
        _root, results = cleanup.process(self.args(apply=False), emit=False)

        self.assertTrue(results[0]["eligible_for_recording_deletion"])
        self.assertEqual(results[0]["recovery_event_count"], 0)
        self.assertEqual(results[0]["recovery_markers"], [])

    def test_missing_log_file_is_non_blocking(self):
        (self.log_dir / "round_001" / "roslaunch.log").unlink()

        _root, results = cleanup.process(self.args(apply=False), emit=False)

        self.assertTrue(results[0]["eligible_for_recording_deletion"])
        self.assertEqual(results[0]["recovery_event_count"], 0)
        self.assertEqual(results[0]["recovery_markers"], [])

    def test_row_without_log_file_column_is_non_blocking(self):
        csv_path = self.log_dir / "trials.csv"
        with csv_path.open("r", encoding="utf-8", newline="") as csv_file:
            reader = csv.DictReader(csv_file)
            fields = [f for f in reader.fieldnames if f != "log_file"]
            rows = [
                {k: v for k, v in dict(row).items() if k != "log_file"}
                for row in reader
            ]
        with csv_path.open("w", encoding="utf-8", newline="") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fields)
            writer.writeheader()
            writer.writerows(rows)

        _root, results = cleanup.process(self.args(apply=False), emit=False)

        self.assertTrue(results[0]["eligible_for_recording_deletion"])
        self.assertEqual(results[0]["recovery_event_count"], 0)
        self.assertEqual(results[0]["recovery_markers"], [])

    def test_recovery_log_memoization_refreshes_when_file_changes(self):
        log_path = self.log_dir / "round_001" / "roslaunch.log"
        log_path.write_text("clean\n", encoding="utf-8")
        self.assertEqual(cleanup.recovery_events_in_log(str(log_path)), (0, []))
        with log_path.open("a", encoding="utf-8") as stream:
            stream.write("bounded navigation recovery completed\n")
        self.assertEqual(
            cleanup.recovery_events_in_log(str(log_path)),
            (1, ["bounded navigation recovery completed"]),
        )


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

    def test_new_adaptive_run_requires_complete_diagnostics(self):
        csv_path = self.log_dir / "trials.csv"
        with csv_path.open("r", encoding="utf-8", newline="") as csv_file:
            reader = csv.DictReader(csv_file)
            fields = list(reader.fieldnames) + ["adaptive_monitor_status"]
            rows = list(reader)
        rows[0]["adaptive_monitor_status"] = "manifest_unavailable"
        with csv_path.open("w", encoding="utf-8", newline="") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fields)
            writer.writeheader()
            writer.writerows(rows)

        _root, results = cleanup.process(self.args(apply=False), emit=False)

        self.assertFalse(results[0]["eligible_for_recording_deletion"])
        self.assertEqual(results[0]["reason"], "adaptive_monitor_incomplete")

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


class PreConeEndToEndRecordingCleanupTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        root = Path(self.temporary.name)
        self.pre_cone_root = root / "teb_pre_cone"
        self.logs_root = root / "logs"
        self.run_name = "pre_cone_e2e_trials_20260815_120000_seed12345"
        self.run_dir = self.pre_cone_root / self.run_name
        self.log_dir = self.logs_root / self.run_name
        self.run_dir.mkdir(parents=True)
        self.log_dir.mkdir(parents=True)

        fields = (
            "round", "task_id", "success", "status", "stuck", "stuck_reason",
            "recognition_failure", "recognition_status", "cube_scene_status",
            "gazebo_recording_status", "gazebo_recording_path",
            "gazebo_recording_size_bytes",
        )
        rows = []
        for round_number in range(1, 6):
            round_dir = self.run_dir / "round_{:03d}".format(round_number)
            round_dir.mkdir()
            recording = (round_dir / "gazebo_world_state.log").resolve()
            recording.write_bytes(b"gazebo-state")
            (round_dir / "gazebo_world_recording.json").write_text(
                json.dumps(
                    {
                        "task_id": "pre_cone_{}".format(round_number),
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
                    "task_id": "pre_cone_{}".format(round_number),
                    "success": "True",
                    "status": "object_grasped",
                    "stuck": "False",
                    "stuck_reason": "",
                    "recognition_failure": "False",
                    "recognition_status": "correct",
                    "cube_scene_status": "complete",
                    "gazebo_recording_status": "complete",
                    "gazebo_recording_path": str(recording),
                    "gazebo_recording_size_bytes": recording.stat().st_size,
                }
            )
        rows[1]["stuck"] = "True"
        rows[1]["stuck_reason"] = "no_task_progress"
        rows[2]["recognition_failure"] = "True"
        rows[2]["recognition_status"] = "incorrect"
        rows[3]["recognition_failure"] = ""
        # A terminal grasp failure is intentionally prunable when the two
        # requested diagnostic classifiers are both explicitly clean.
        rows[4]["success"] = "False"
        rows[4]["status"] = "task_failed"

        with (self.log_dir / "trials.csv").open(
            "w", encoding="utf-8", newline=""
        ) as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows(rows)

    def tearDown(self):
        self.temporary.cleanup()

    def args(self, apply):
        root = Path(self.temporary.name)
        return types.SimpleNamespace(
            data_root=root / "missing_legacy",
            cone_video_root=root / "missing_cone",
            cone_stress_root=root / "missing_stress",
            pre_navigation_root=root / "missing_pre_navigation",
            fixed_cone_e2e_root=root / "missing_fixed_cone",
            pre_cone_root=self.pre_cone_root,
            logs_root=self.logs_root,
            runs=None,
            apply=apply,
        )

    def test_keeps_only_stuck_recognition_failure_or_unknown_rounds(self):
        _root, results = cleanup.process(self.args(apply=False), emit=False)

        self.assertEqual(len(results), 5)
        self.assertTrue(results[0]["eligible_for_recording_deletion"])
        self.assertEqual(results[1]["reason"], "stuck_or_unknown")
        self.assertEqual(results[2]["reason"], "recognition_failure_or_unknown")
        self.assertEqual(results[3]["reason"], "recognition_failure_or_unknown")
        self.assertTrue(results[4]["eligible_for_recording_deletion"])

    def test_apply_deletes_only_explicitly_clean_recordings(self):
        _root, results = cleanup.process(self.args(apply=True), emit=False)

        for round_number in (1, 5):
            for filename in cleanup.RECORDING_FILES:
                self.assertFalse(
                    (self.run_dir / "round_{:03d}".format(round_number) / filename)
                    .exists()
                )
        for round_number in (2, 3, 4):
            for filename in cleanup.RECORDING_FILES:
                self.assertTrue(
                    (self.run_dir / "round_{:03d}".format(round_number) / filename)
                    .is_file()
                )
        self.assertEqual(sum(len(item["deleted_files"]) for item in results), 6)


if __name__ == "__main__":
    unittest.main()
