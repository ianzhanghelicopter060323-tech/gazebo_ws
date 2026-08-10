#!/usr/bin/env python3

import unittest
from pathlib import Path
import tempfile

import run_end_to_end_cone_trials as trials


def record(**overrides):
    value = {
        "round": 1,
        "task_id": "task_1",
        "target_class": "food",
        "success": False,
        "status": "task_failed",
        "completed_stage": 250,
        "error_code": 7,
        "message": "navigation aborted",
        "cone_monitor_file": "/tmp/cone_trial.json",
        "cone_collision": False,
        "collision_cones": "",
        "direct_contact_cones": "",
        "moved_or_tilted_cones": "",
        "cone_monitor_status": "complete",
        "gazebo_recording_status": "complete",
        "preceding_navigation_failure": False,
        "delivery_navigation_failure": False,
    }
    value.update(overrides)
    return value


class FailureClassificationTest(unittest.TestCase):
    def test_navigation_abort_before_grasp_is_preceding_failure(self):
        self.assertTrue(trials.is_navigation_failure(record(), 5, delivery=False))
        self.assertFalse(trials.is_navigation_failure(record(), 5, delivery=True))

    def test_navigation_abort_after_grasp_is_delivery_failure(self):
        self.assertFalse(trials.is_navigation_failure(record(), 16, delivery=False))
        self.assertTrue(trials.is_navigation_failure(record(), 16, delivery=True))

    def test_perception_failure_is_not_navigation_failure(self):
        failed = record(error_code=10, message="object not found")
        self.assertFalse(trials.is_navigation_failure(failed, 9, delivery=False))

    def test_timeout_while_preceding_navigation_is_classified(self):
        failed = record(status="task_timeout", error_code=None)
        self.assertTrue(trials.is_navigation_failure(failed, 8, delivery=False))


class ArgumentDefaultsTest(unittest.TestCase):
    def test_gazebo_gui_and_hardware_launch_are_enabled_by_default(self):
        args = trials.parse_args([])

        self.assertTrue(args.gui)

    def test_headless_explicitly_disables_gazebo_gui(self):
        args = trials.parse_args(["--headless"])

        self.assertFalse(args.gui)


class StressTemplateTest(unittest.TestCase):
    def test_collision_round_becomes_exact_cone_template(self):
        failed = record(
            cone_collision=True,
            collision_cones="cone_12",
            moved_or_tilted_cones="cone_12",
        )
        pose = {
            "position": {"x": 1.0, "y": -2.0, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            "yaw": 0.0,
            "tilt_rad": 0.0,
        }
        monitor = {
            "initial_scene": {
                "car3": pose,
                "cube_0": pose,
                "cone_12": pose,
            }
        }

        template = trials.build_stress_template(failed, monitor)

        self.assertEqual(set(template["cones"]), {"cone_12"})
        self.assertEqual(template["target_class"], "food")
        self.assertIn("unchanged random spawner", template["replay_policy"])

    def test_noncollision_round_does_not_create_template(self):
        self.assertIsNone(
            trials.build_stress_template(record(), {"initial_scene": {}})
        )


class SummaryTest(unittest.TestCase):
    def test_counts_failures_and_collisions_independently(self):
        results = [
            record(preceding_navigation_failure=True),
            record(
                round=2,
                target_class="daily",
                success=True,
                status="task_completed",
                error_code=0,
                cone_collision=True,
                direct_contact_cones="cone_10",
                moved_or_tilted_cones="cone_10",
            ),
        ]
        summary = trials.make_summary(results, requested_rounds=40, seed=123)
        self.assertEqual(summary["completed_rounds"], 2)
        self.assertEqual(summary["task_successes"], 1)
        self.assertEqual(summary["preceding_navigation_failures"], 1)
        self.assertEqual(summary["cone_collision_rounds"], 1)
        self.assertEqual(summary["gazebo_recording_complete_rounds"], 2)


class GazeboRecordingResultTest(unittest.TestCase):
    def test_invalid_manifest_size_is_reported_without_crashing(self):
        with tempfile.TemporaryDirectory() as temporary:
            manifest = Path(temporary) / "gazebo_world_recording.json"
            manifest.write_text(
                '{"status":"complete","size_bytes":"invalid"}',
                encoding="utf-8",
            )
            value = {
                "gazebo_recording_status": "not_started",
                "gazebo_recording_manifest": str(manifest),
                "gazebo_recording_size_bytes": 0,
                "gazebo_recording_path": str(
                    Path(temporary) / "gazebo_world_state.log"
                ),
            }

            trials.add_gazebo_recording_result(value)

            self.assertEqual(
                value["gazebo_recording_status"], "manifest_invalid"
            )


if __name__ == "__main__":
    unittest.main()
