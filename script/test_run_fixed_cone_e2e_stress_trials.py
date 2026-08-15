#!/usr/bin/env python3

from collections import Counter
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

import _run_cone_move_navigation as navigation_helper
import _setup_cone_move_stress_scene as scene_helper
import capture_pickup_dataset as automation
import run_fixed_cone_e2e_stress_trials as trials


HARD_SCENARIO_MANIFEST = (
    Path(__file__).resolve().parent
    / "config"
    / "fixed_cone_e2e_rounds_007_009_012_032.json"
)
LEGACY_SCENARIO_MANIFEST = (
    Path(__file__).resolve().parent
    / "config"
    / "fixed_cone_e2e_rounds_025_030.json"
)
SEND_TASK_SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "src"
    / "smart_factory_tests"
    / "scripts"
    / "send_navigation_task.py"
)
NAVIGATION_CONFIG = (
    Path(__file__).resolve().parents[1]
    / "src"
    / "smart_factory_navigation"
    / "config"
    / "navigation.yaml"
)


def load_send_task_module():
    spec = importlib.util.spec_from_file_location(
        "send_navigation_task_under_test", SEND_TASK_SCRIPT
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def runtime_config():
    return {
        "switch_distance": 0.2,
        "destinations": {
            target_class: {
                "name": "{} workshop".format(target_class),
                "frame_id": "map",
                "x": float(index),
                "y": -2.0,
                "yaw": 0.0,
                "position_tolerance": 0.04,
                "yaw_tolerance": 0.1,
            }
            for index, target_class in enumerate(
                ("food", "daily", "electronics")
            )
        },
    }


class ArgumentDefaultsTest(unittest.TestCase):
    def test_navigation_snapshot_includes_dynamic_delivery_goal_config(self):
        self.assertIn(
            trials.DELIVERY_GOALS_CONFIG,
            trials.NAVIGATION_CONFIG_FILES,
        )

    def test_default_is_20_headless_hard_scenario_trials(self):
        args = trials.parse_args([])

        self.assertEqual(args.rounds, 20)
        self.assertFalse(args.gui)
        self.assertIsNone(args.source_rounds)
        self.assertEqual(trials.DEFAULT_SOURCE_ROUNDS, (7, 9, 12, 32))
        self.assertEqual(args.templates, trials.DEFAULT_TEMPLATE_MANIFEST)
        self.assertEqual(args.recording_root, trials.DEFAULT_RECORDING_ROOT)
        self.assertEqual(args.startup_retries, 2)
        self.assertEqual(args.startup_timeout, 180.0)
        self.assertEqual(args.experiment_label, "")
        self.assertEqual(args.task_timeout, 480.0)
        self.assertEqual(args.task_progress_timeout, 360.0)
        self.assertEqual(args.status_interval, 30.0)
        self.assertEqual(args.adaptive_monitor_ready_timeout, 15.0)

    def test_navigation_goal_timeout_is_six_minutes(self):
        config_text = NAVIGATION_CONFIG.read_text(encoding="utf-8")

        self.assertRegex(config_text, r"(?m)^\s*goal_timeout:\s*360\.0\s*$")
        self.assertIn(
            "/smart_factory_navigation/navigation/goal_timeout",
            trials.RUNTIME_PARAMETER_NAMES,
        )

    def test_gui_flag_enables_gazebo_client(self):
        self.assertTrue(trials.parse_args(["--gui"]).gui)
        self.assertFalse(trials.parse_args(["--headless"]).gui)

        with self.assertRaises(SystemExit):
            trials.parse_args(["--gui", "--headless"])

    def test_experiment_label_must_be_filesystem_safe(self):
        args = trials.parse_args(["--experiment-label", "bad label"])

        with self.assertRaisesRegex(automation.AutomationError, "experiment-label"):
            trials.validate_args(args)

    def test_source_footprint_label_is_checked_before_launch(self):
        with mock.patch.object(
            trials,
            "configured_avoidance_footprint_half_extent",
            return_value=0.20,
        ):
            trials.validate_source_experiment_label(
                "adaptive_teb_avoidfp_0p20_v1"
            )
            with self.assertRaisesRegex(
                automation.AutomationError, "Gazebo was not started"
            ):
                trials.validate_source_experiment_label(
                    "adaptive_teb_avoidfp_0p15_v1"
                )

    def test_end_to_end_readiness_requires_mission_action(self):
        parsed = navigation_helper.parse_args(
            [
                "--case-file",
                "/tmp/case.json",
                "--readiness-only",
                "--require-mission-action",
                "--skip-profile-services",
            ]
        )

        self.assertTrue(parsed.require_mission_action)
        self.assertTrue(parsed.skip_profile_services)
        self.assertEqual(parsed.mission_action, "/sim_task/execute")

    def test_post_scene_readiness_can_require_localization(self):
        parsed = navigation_helper.parse_args(
            [
                "--case-file",
                "/tmp/case.json",
                "--readiness-only",
                "--require-localization",
            ]
        )

        self.assertTrue(parsed.readiness_only)
        self.assertTrue(parsed.require_localization)


class FixedManifestTest(unittest.TestCase):
    def test_hard_daily_manifest_contains_requested_scenarios(self):
        templates = trials.load_templates(
            HARD_SCENARIO_MANIFEST, [7, 9, 12, 32]
        )

        self.assertEqual(
            [value["source_round"] for value in templates], [7, 9, 12, 32]
        )
        self.assertTrue(
            all(value["target_class"] == "daily" for value in templates)
        )
        self.assertAlmostEqual(
            templates[0]["cones"]["cone_10"]["position"]["x"], -0.982132
        )

    def test_manifest_reproduces_rounds_and_task_types(self):
        templates = trials.load_templates(trials.DEFAULT_TEMPLATE_MANIFEST)

        self.assertEqual(
            [(value["source_round"], value["target_class"]) for value in templates],
            [(7, "daily"), (9, "daily"), (12, "daily"), (32, "daily")],
        )
        self.assertEqual(set(templates[0]["cones"]), set(scene_helper.CONE_NAMES))
        self.assertAlmostEqual(
            templates[0]["cones"]["cone_10"]["position"]["x"], -0.982132
        )

    def test_cones_only_case_does_not_require_robot_start(self):
        templates = trials.load_templates(trials.DEFAULT_TEMPLATE_MANIFEST)
        with tempfile.TemporaryDirectory() as temporary:
            case_file = Path(temporary) / "case.json"
            case_file.write_text(
                json.dumps({"cones": templates[0]["cones"]}), encoding="utf-8"
            )

            parsed = scene_helper.load_case(case_file, require_robot=False)

        self.assertNotIn("robot_start", parsed)
        self.assertEqual(set(parsed["cones"]), set(scene_helper.CONE_NAMES))


class ScheduleTest(unittest.TestCase):
    def test_hard_daily_schedule_runs_each_scenario_five_times(self):
        templates = trials.load_templates(
            HARD_SCENARIO_MANIFEST, [7, 9, 12, 32]
        )
        plans = trials.build_plans(templates, 20, runtime_config())

        self.assertEqual(
            [plan["source_round"] for plan in plans[:8]],
            [7, 9, 12, 32, 7, 9, 12, 32],
        )
        self.assertEqual(
            Counter(plan["source_round"] for plan in plans),
            Counter({7: 5, 9: 5, 12: 5, 32: 5}),
        )

    def test_default_schedule_runs_each_hard_scenario_five_times(self):
        templates = trials.load_templates(
            trials.DEFAULT_TEMPLATE_MANIFEST,
            list(trials.DEFAULT_SOURCE_ROUNDS),
        )
        plans = trials.build_plans(templates, 20, runtime_config())

        self.assertEqual(len(plans), 20)
        self.assertEqual(
            Counter(plan["source_round"] for plan in plans),
            Counter({7: 5, 9: 5, 12: 5, 32: 5}),
        )

    def test_40_round_schedule_is_round_robin_and_balanced(self):
        templates = trials.load_templates(
            LEGACY_SCENARIO_MANIFEST, [25, 26, 27, 28, 29, 30]
        )
        plans = trials.build_plans(templates, 40, runtime_config())

        self.assertEqual(
            [plan["source_round"] for plan in plans[:6]],
            [25, 26, 27, 28, 29, 30],
        )
        counts = Counter(plan["source_round"] for plan in plans)
        self.assertEqual(
            counts, Counter({25: 7, 26: 7, 27: 7, 28: 7, 29: 6, 30: 6})
        )
        self.assertEqual(plans[-1]["source_round"], 28)
        self.assertEqual(plans[-1]["repetition"], 7)

    def test_selected_scenarios_remain_round_robin(self):
        templates = trials.load_templates(
            LEGACY_SCENARIO_MANIFEST, [27, 30]
        )
        plans = trials.build_plans(templates, 5, runtime_config())

        self.assertEqual(
            [plan["source_round"] for plan in plans], [27, 30, 27, 30, 27]
        )
        self.assertEqual(
            [plan["target_class"] for plan in plans],
            ["electronics", "food", "electronics", "food", "electronics"],
        )


class InfrastructureRetryTest(unittest.TestCase):
    def test_action_handshake_failure_is_retryable(self):
        record = {
            "status": "result_unavailable",
            "message": "task client exited with code 2 without a parseable result",
        }

        self.assertTrue(trials.is_retryable_infrastructure_failure(record))

    def test_operational_navigation_failure_is_not_retryable(self):
        record = {"status": "task_failed", "message": "move_base aborted"}

        self.assertFalse(trials.is_retryable_infrastructure_failure(record))

    def test_configuration_mismatch_is_not_retried(self):
        record = {
            "status": "configuration_validation_error",
            "message": "experiment label does not match runtime parameters",
        }

        self.assertFalse(trials.is_retryable_infrastructure_failure(record))
        self.assertTrue(trials.is_fatal_batch_failure(record))

    def test_navigation_failure_is_not_a_fatal_batch_configuration_error(self):
        record = {"status": "task_failed", "message": "move_base aborted"}

        self.assertFalse(trials.is_fatal_batch_failure(record))

    def test_final_startup_failure_exhausts_retries(self):
        record = {"status": "startup_error", "message": "Gazebo models missing"}

        self.assertFalse(
            trials.infrastructure_retries_exhausted(
                record, attempt=2, startup_retries=2
            )
        )
        self.assertTrue(
            trials.infrastructure_retries_exhausted(
                record, attempt=3, startup_retries=2
            )
        )

    def test_operational_failure_does_not_abort_batch_as_infrastructure(self):
        record = {"status": "task_failed", "message": "move_base aborted"}

        self.assertFalse(
            trials.infrastructure_retries_exhausted(
                record, attempt=3, startup_retries=2
            )
        )

    def test_keyboard_interrupt_stops_owned_action_client(self):
        process = mock.Mock()
        process.communicate.side_effect = KeyboardInterrupt
        with mock.patch.object(
            automation.subprocess, "Popen", return_value=process
        ), mock.patch.object(automation, "stop_process_group") as stop:
            with self.assertRaises(KeyboardInterrupt):
                automation.run_owned(["fake-action-client"], timeout=10.0)

        stop.assert_called_once_with(process, interrupt_timeout=2.0)


class RuntimeIdentityTest(unittest.TestCase):
    @staticmethod
    def parameters():
        return {
            "/move_base/global_costmap/inflation_layer/inflation_radius": 0.25,
            "/move_base/global_costmap/inflation_layer/cost_scaling_factor": 2.0,
            "/move_base/GlobalPlanner/cost_factor": 0.55,
            "/move_base/GlobalPlanner/neutral_cost": 40,
            "/move_base/GlobalPlanner/use_dijkstra": False,
            "/move_base/GlobalPlanner/use_grid_path": True,
            "/move_base/base_local_planner": "teb_local_planner/TebLocalPlannerROS",
        }

    def test_astar_label_matches_runtime_parameters(self):
        trials.validate_runtime_experiment_label(
            "astar_ginf_0p25_gscale_2p0_nc40", self.parameters()
        )

    def test_legacy_mismatched_label_is_rejected(self):
        with self.assertRaisesRegex(
            automation.AutomationError, "global inflation"
        ):
            trials.validate_runtime_experiment_label(
                "global_inf_0p20_cf_0p55", self.parameters()
            )

    def test_astar_label_rejects_dijkstra_runtime(self):
        parameters = self.parameters()
        parameters["/move_base/GlobalPlanner/use_dijkstra"] = True

        with self.assertRaisesRegex(automation.AutomationError, "use_dijkstra"):
            trials.validate_runtime_experiment_label(
                "astar_ginf_0p25_gscale_2p0_nc40", parameters
            )

    def test_teb_dijkstra_label_matches_runtime_parameters(self):
        parameters = self.parameters()
        parameters["/move_base/GlobalPlanner/use_dijkstra"] = True

        trials.validate_runtime_experiment_label(
            "teb_dijkstra_ginf_0p25_gscale_2p0_nc40", parameters
        )

    def test_teb_label_rejects_a_non_teb_local_planner(self):
        parameters = self.parameters()
        parameters["/move_base/base_local_planner"] = "dwa_local_planner/DWAPlannerROS"

        with self.assertRaisesRegex(automation.AutomationError, "base_local_planner"):
            trials.validate_runtime_experiment_label("teb", parameters)

    def test_adaptive_teb_label_matches_adaptive_wrapper(self):
        parameters = self.parameters()
        parameters["/move_base/base_local_planner"] = trials.ADAPTIVE_TEB_PLUGIN
        parameters["/move_base/AdaptiveTebLocalPlannerROS"] = {
            "avoidance": {
                "footprint_model": {
                    "vertices": [
                        [0.10, -0.10],
                        [0.10, 0.10],
                        [-0.10, 0.10],
                        [-0.10, -0.10],
                    ]
                }
            }
        }

        trials.validate_runtime_experiment_label(
            "adaptive_teb_avoidfp_0p10", parameters
        )

    def test_avoidance_footprint_label_rejects_wrong_half_extent(self):
        parameters = self.parameters()
        parameters["/move_base/base_local_planner"] = trials.ADAPTIVE_TEB_PLUGIN
        parameters["/move_base/AdaptiveTebLocalPlannerROS"] = {
            "avoidance": {
                "footprint_model": {
                    "vertices": [[0.08, -0.08], [0.08, 0.08], [-0.08, 0.08]]
                }
            }
        }

        with self.assertRaisesRegex(
            automation.AutomationError, "footprint half-extent"
        ):
            trials.validate_runtime_experiment_label(
                "adaptive_teb_avoidfp_0p10", parameters
            )

    def test_adaptive_label_rejects_plain_teb(self):
        with self.assertRaisesRegex(
            automation.AutomationError, "adaptive TEB"
        ):
            trials.validate_runtime_experiment_label(
                "adaptive_teb", self.parameters()
            )

    def test_progress_timeout_marker_is_parsed(self):
        output = (
            'prefix\nTASK_PROGRESS_TIMEOUT={"inactive_seconds": 360.2, '
            '"task_id": "case"}\nsuffix\n'
        )

        payload = trials.progress_timeout_payload(output)

        self.assertEqual(payload["task_id"], "case")
        self.assertAlmostEqual(payload["inactive_seconds"], 360.2)


class TaskProgressTrackerTest(unittest.TestCase):
    def test_only_meaningful_feedback_changes_reset_timeout(self):
        module = load_send_task_module()
        clock = iter((10.0, 11.0, 12.0))
        tracker = module.ProgressTracker(monotonic=lambda: next(clock))
        feedback = mock.Mock(
            current_stage=5,
            retry_count=0,
            detail="waypoint 1/30 move_base goal is active",
        )

        tracker.update(feedback)
        self.assertEqual(tracker.last_progress, 11.0)
        tracker.update(feedback)
        self.assertEqual(tracker.last_progress, 11.0)

        feedback.detail = "waypoint 1/30 passed within 0.15 m"
        tracker.update(feedback)
        self.assertEqual(tracker.last_progress, 12.0)


class AdaptiveDiagnosticsResultTest(unittest.TestCase):
    def test_complete_matching_manifest_populates_trial_fields(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "adaptive.json"
            path.write_text(
                json.dumps(
                    {
                        "status": "complete",
                        "round": 3,
                        "task_id": "task-3",
                        "summary": {
                            "mode_event_count": 9,
                            "diagnostic_sample_count": 42,
                            "avoidance_entries": 2,
                            "avoidance_used": True,
                            "avoidance_command_samples": 7,
                            "avoidance_failure_recovery_samples": 1,
                            "avoidance_infeasible_samples": 2,
                            "baseline_fallback_events": 1,
                            "baseline_fallback_samples": 1,
                            "baseline_infeasible_samples": 0,
                            "both_planners_infeasible_samples": 0,
                            "minimum_plan_clearance_m": 0.147,
                        },
                    }
                ),
                encoding="utf-8",
            )
            record = {
                "round": 3,
                "task_id": "task-3",
                "adaptive_monitor_file": str(path),
                "adaptive_monitor_status": "monitoring",
            }

            trials.add_adaptive_diagnostics_result(record)

        self.assertEqual(record["adaptive_monitor_status"], "complete")
        self.assertEqual(record["adaptive_avoidance_entries"], 2)
        self.assertTrue(record["adaptive_avoidance_used"])
        self.assertEqual(record["adaptive_avoidance_infeasible_samples"], 2)
        self.assertEqual(record["adaptive_baseline_fallback_events"], 1)
        self.assertEqual(record["adaptive_baseline_fallback_samples"], 1)
        self.assertAlmostEqual(
            record["adaptive_minimum_plan_clearance_m"], 0.147
        )

    def test_mismatched_task_manifest_is_rejected(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "adaptive.json"
            path.write_text(
                json.dumps(
                    {
                        "status": "complete",
                        "round": 1,
                        "task_id": "wrong",
                        "summary": {},
                    }
                ),
                encoding="utf-8",
            )
            record = {
                "round": 1,
                "task_id": "expected",
                "adaptive_monitor_file": str(path),
                "adaptive_monitor_status": "monitoring",
            }

            trials.add_adaptive_diagnostics_result(record)

        self.assertEqual(record["adaptive_monitor_status"], "manifest_invalid")


class StartupReadinessTest(unittest.TestCase):
    def test_pre_navigation_readiness_does_not_wait_for_rgb_camera(self):
        calls = []

        def fake_wait(label, arguments, deadline, required_text=None):
            calls.append((label, arguments, required_text))

        with mock.patch.object(trials, "wait_for_ros", side_effect=fake_wait):
            trials.wait_for_trial_simulation(10.0)

        labels = [call[0] for call in calls]
        arguments = [part for call in calls for part in call[1]]
        self.assertIn("AMCL pose", labels)
        self.assertIn("mission server", labels)
        self.assertNotIn("/camera/rgb/image_raw", arguments)

    def test_required_models_use_one_topic_read_instead_of_services(self):
        calls = []

        def fake_wait(label, arguments, deadline, required_text=None):
            calls.append((label, arguments, required_text, deadline))

        with mock.patch.object(
            trials, "wait_for_ros", side_effect=fake_wait
        ), mock.patch.object(
            trials.time,
            "monotonic",
            side_effect=(10.0, 20.0, 30.0, 40.0, 50.0),
        ):
            trials.wait_for_trial_simulation(180.0)

        model_calls = [call for call in calls if call[0] == "Gazebo required models"]
        self.assertEqual(len(model_calls), 1)
        self.assertEqual(model_calls[0][1][0:4], ["rostopic", "echo", "-n", "1"])
        self.assertEqual(model_calls[0][1][4], "/gazebo/model_states/name")
        self.assertNotIn("--noarr", model_calls[0][1])
        self.assertEqual(
            model_calls[0][2], ("car3", "cube_0", "cube_1", "cube_2")
        )
        self.assertFalse(any("rosservice" in call[1] for call in calls))

    def test_each_startup_dependency_gets_a_fresh_deadline(self):
        deadlines = []

        def fake_wait(_label, _arguments, deadline, required_text=None):
            deadlines.append(deadline)

        with mock.patch.object(
            trials, "wait_for_ros", side_effect=fake_wait
        ), mock.patch.object(
            trials.time,
            "monotonic",
            side_effect=(10.0, 20.0, 30.0, 40.0, 50.0),
        ):
            trials.wait_for_trial_simulation(180.0)

        self.assertEqual(deadlines, [190.0, 200.0, 210.0, 220.0, 230.0])

    def test_wait_for_ros_accepts_multiple_required_fragments(self):
        with mock.patch.object(
            automation,
            "run_owned",
            return_value=(0, "name: [ground_plane, car3, cube_0, cube_1, cube_2]"),
        ), mock.patch.object(automation.time, "monotonic", return_value=1.0):
            automation.wait_for_ros(
                "models",
                ["fake-command"],
                deadline=2.0,
                required_text=("car3", "cube_0", "cube_1", "cube_2"),
            )


class AcceptanceRuleTest(unittest.TestCase):
    @staticmethod
    def record(scenario_id, navigation_failure=False, collision=False):
        return {
            "scenario_id": scenario_id,
            "success": not navigation_failure,
            "preceding_navigation_failure": False,
            "delivery_navigation_failure": navigation_failure,
            "cone_collision": collision,
            "gazebo_recording_status": "complete",
            "cone_monitor_status": "complete",
            "status": "task_failed" if navigation_failure else "task_completed",
        }

    def test_one_issue_in_any_scenario_requires_further_tuning(self):
        plans = [
            {"scenario_id": "round_007", "source_round": 7, "target_class": "daily"},
            {"scenario_id": "round_009", "source_round": 9, "target_class": "daily"},
        ]
        results = [
            self.record("round_007"),
            self.record("round_009", navigation_failure=True),
        ]

        summary = trials.make_summary(results, 2, plans)

        self.assertFalse(summary["acceptance_passed"])
        self.assertTrue(summary["requires_further_tuning"])
        self.assertFalse(
            summary["by_scenario"]["round_007"]["requires_further_tuning"]
        )
        self.assertTrue(
            summary["by_scenario"]["round_009"]["requires_further_tuning"]
        )


if __name__ == "__main__":
    unittest.main()
