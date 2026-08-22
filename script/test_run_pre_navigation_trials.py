#!/usr/bin/env python3

import os
from pathlib import Path
import tempfile
import unittest
from unittest import mock

import run_pre_navigation_trials as trials


class PreNavigationTrialConfigurationTest(unittest.TestCase):
    def test_defaults_use_requested_isolated_ports_and_hundred_rounds(self):
        args = trials.parse_args([])

        self.assertEqual(args.rounds, 100)
        self.assertEqual(args.ros_master_port, 1132)
        self.assertEqual(args.gazebo_master_port, 1133)
        self.assertFalse(args.gui)
        self.assertEqual(args.recording_root, trials.DEFAULT_RECORDING_ROOT)
        self.assertIsNone(args.seq35_position_tolerance)

        metadata = trials.validate_args(args)

        self.assertEqual(args.seq35_position_tolerance, 0.15)
        self.assertEqual(metadata["formal_final_pass_radius_m"], 0.15)
        self.assertEqual(args.navigation_config, trials.DEFAULT_NAVIGATION_CONFIG)
        self.assertEqual(
            metadata["execution_sequences"],
            [6, 9, 10, 16, 17, 18, 20, 21, 22, 29, 33, 34, 35],
        )
        self.assertEqual(
            metadata["orientation_required_sequences"], [17, 18, 21]
        )
        self.assertEqual(
            metadata["orientation_position_tolerance_overrides_m"], {21: 0.08}
        )

    def test_master_environment_is_scoped_by_explicit_ports(self):
        args = trials.parse_args(
            ["--ros-master-port", "12001", "--gazebo-master-port", "12002"]
        )
        with tempfile.TemporaryDirectory() as temporary, mock.patch.dict(
            os.environ, {}, clear=True
        ):
            values = trials.configure_master_environment(
                args, Path(temporary) / "ros_logs"
            )

            self.assertEqual(values["ROS_MASTER_URI"], "http://127.0.0.1:12001")
            self.assertEqual(
                values["GAZEBO_MASTER_URI"], "http://127.0.0.1:12002"
            )
            self.assertEqual(os.environ["ROS_MASTER_URI"], values["ROS_MASTER_URI"])
            self.assertEqual(
                os.environ["GAZEBO_MASTER_URI"], values["GAZEBO_MASTER_URI"]
            )
            self.assertTrue(Path(os.environ["ROS_LOG_DIR"]).is_dir())

    def test_client_result_requires_explicit_marker(self):
        payload = trials.parse_client_result(
            'noise\nPRE_NAVIGATION_RESULT={"success": true, "error_code": 0}\n'
        )
        self.assertTrue(payload["success"])

    def test_existing_gazebo_server_is_rejected(self):
        with mock.patch.object(
            trials, "running_gazebo_server_pids", return_value=[123, 456]
        ):
            with self.assertRaisesRegex(
                trials.CleanupError, "another gzserver is already running"
            ):
                trials.ensure_no_concurrent_gazebo()

    def test_reports_are_mirrored_beside_recordings(self):
        args = trials.parse_args(["--rounds", "1"])
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            logs = root / "logs"
            recordings = root / "recordings"
            logs.mkdir()
            recordings.mkdir()
            metadata = {"recording_run_dir": str(recordings)}

            trials.write_reports(logs, [], args, metadata)

            self.assertTrue((logs / "trials.csv").is_file())
            self.assertTrue((recordings / "trials.csv").is_file())
            self.assertTrue((recordings / "summary.json").is_file())


class PreNavigationTrialResultTest(unittest.TestCase):
    def make_record(self):
        args = trials.parse_args([])
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            return trials.new_record(args, 1, "pre_nav_1", root, root)

    def test_position_tolerance_result_is_successful_without_yaw_alignment(self):
        record = self.make_record()
        payload = {
            "success": True,
            "status": "navigation_completed",
            "error_code": 0,
            "server_error_code": 8,
            "message": "position accepted",
            "acceptance_mode": "position_tolerance",
            "yaw_alignment_required": False,
            "completed_waypoints": 30,
            "waypoint_count": 30,
            "final_position_error_m": 0.149,
            "final_position_tolerance_m": 0.15,
            "final_yaw_error_rad": 1.2,
            "final_pose": {"x": -1.2, "y": -0.5, "yaw": 1.2},
        }

        trials.apply_client_result(record, payload, return_code=0)

        self.assertTrue(record["success"])
        self.assertFalse(record["yaw_alignment_required"])
        self.assertEqual(record["acceptance_mode"], "position_tolerance")

    def test_position_outside_tolerance_is_not_successful(self):
        record = self.make_record()
        payload = {
            "success": True,
            "status": "navigation_completed",
            "error_code": 0,
            "yaw_alignment_required": False,
            "completed_waypoints": 30,
            "waypoint_count": 30,
            "final_position_error_m": 0.151,
            "final_position_tolerance_m": 0.15,
        }

        trials.apply_client_result(record, payload, return_code=0)

        self.assertFalse(record["success"])
        self.assertEqual(record["status"], "navigation_client_validation_failed")


if __name__ == "__main__":
    unittest.main()
