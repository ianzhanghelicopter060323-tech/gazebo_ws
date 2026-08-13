#!/usr/bin/env python3
"""Replay eight recorded cone layouts five times from preparation pose 2."""

import argparse
import csv
import datetime as dt
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import time

import yaml

from capture_pickup_dataset import (
    AutomationError,
    CleanupError,
    SETUP_FILE,
    WORKSPACE,
    launch_simulation,
    master_is_running,
    read_ros_run_id,
    run_owned,
    stop_owned_launch,
    stop_process_group,
    wait_for_ros,
    wait_for_simulation,
)
from run_end_to_end_cone_trials import (
    add_gazebo_recording_result,
    add_monitor_result,
    append_section,
    load_monitor,
    start_cone_monitor,
    start_gazebo_world_recorder,
)


DEFAULT_TEMPLATE_MANIFEST = (
    WORKSPACE
    / "script"
    / "logs"
    / "end_to_end_cone_trials_20260811_074704"
    / "collision_templates.json"
)
DEFAULT_RECORDING_ROOT = (
    WORKSPACE / "data" / "cone_zone" / "end_to_end_test" / "conse_stress"
)
DELIVERY_CONFIG = (
    WORKSPACE / "src" / "smart_factory_mission" / "config" / "delivery_goals.yaml"
)
MISSION_CONFIG = (
    WORKSPACE / "src" / "smart_factory_mission" / "config" / "mission.yaml"
)
RANDOM_SPAWNER = WORKSPACE / "src" / "car3" / "scripts" / "spawn_cubes.py"
SCENE_HELPER = WORKSPACE / "script" / "_setup_cone_move_stress_scene.py"
NAVIGATION_HELPER = WORKSPACE / "script" / "_run_cone_move_navigation.py"
TARGET_CLASS_IDS = {"food": 0, "daily": 1, "electronics": 2}
EXPECTED_TEMPLATE_IDS = {
    "collision_round_002",
    "collision_round_009",
    "collision_round_012",
    "collision_round_017",
    "collision_round_027",
    "collision_round_037",
    "collision_round_038",
    "collision_round_040",
}
RESULT_MARKER = "CONE_MOVE_STRESS_RESULT="
CSV_FIELDS = (
    "round",
    "scenario_id",
    "source_round",
    "repetition",
    "task_id",
    "target_class",
    "target_name",
    "success",
    "status",
    "completed_stage",
    "error_code",
    "message",
    "navigation_failure",
    "last_operational_stage",
    "last_operational_stage_name",
    "preceding_navigation_failure",
    "delivery_navigation_failure",
    "planner_mode",
    "profile_switching_required",
    "profile_switched",
    "profile_switch_distance_m",
    "minimum_goal_distance_m",
    "cone_collision",
    "collision_cones",
    "direct_contact_cones",
    "moved_or_tilted_cones",
    "cone_monitor_status",
    "contact_stream_status",
    "gazebo_recording_status",
    "gazebo_recording_path",
    "gazebo_recording_size_bytes",
    "gazebo_recording_manifest",
    "move_base_goal_count",
    "duration_seconds",
    "started_at",
    "round_dir",
    "case_file",
    "log_file",
    "cone_monitor_file",
)


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description=(
            "restart an isolated headless simulation for each fixed cone layout, "
            "start at preparation pose 2, and navigate to the template-bound "
            "workshop while recording Gazebo world state"
        )
    )
    parser.add_argument(
        "--templates", type=Path, default=DEFAULT_TEMPLATE_MANIFEST
    )
    parser.add_argument(
        "--template-id",
        action="append",
        dest="template_ids",
        help="run only this template id; may be repeated",
    )
    parser.add_argument(
        "--repetitions", type=int, default=5, help="runs per template (default: 5)"
    )
    display = parser.add_mutually_exclusive_group()
    display.add_argument("--gui", dest="gui", action="store_true")
    display.add_argument("--headless", dest="gui", action="store_false")
    parser.set_defaults(gui=False)
    parser.add_argument("--startup-timeout", type=float, default=90.0)
    parser.add_argument("--startup-settle", type=float, default=5.0)
    parser.add_argument("--scene-timeout", type=float, default=40.0)
    parser.add_argument("--navigation-timeout", type=float, default=180.0)
    parser.add_argument(
        "--navigation-ready-timeout",
        type=float,
        default=60.0,
        help="wall-clock readiness deadline for the TEB navigation Actions",
    )
    parser.add_argument("--monitor-ready-timeout", type=float, default=15.0)
    parser.add_argument(
        "--gazebo-recording-ready-timeout", type=float, default=15.0
    )
    parser.add_argument("--restart-settle", type=float, default=3.0)
    parser.add_argument("--cone-motion-threshold", type=float, default=0.01)
    parser.add_argument("--cone-tilt-threshold-deg", type=float, default=5.0)
    parser.add_argument("--disable-contact-stream", action="store_true")
    parser.add_argument(
        "--recording-root", type=Path, default=DEFAULT_RECORDING_ROOT
    )
    parser.add_argument("--log-dir", type=Path)
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="validate templates/configuration and print all planned rounds",
    )
    return parser.parse_args(argv)


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def finite(value, label, positive=False):
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise AutomationError("{} must be numeric".format(label)) from exc
    if not math.isfinite(result) or (positive and result <= 0.0):
        qualifier = "positive and finite" if positive else "finite"
        raise AutomationError("{} must be {}".format(label, qualifier))
    return result


def load_yaml(path):
    try:
        return yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise AutomationError("cannot read {}: {}".format(path, exc))


def load_templates(path, selected_ids=None):
    path = path.expanduser().resolve()
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        raise AutomationError("cannot read {}: {}".format(path, exc))
    templates = payload.get("templates") if isinstance(payload, dict) else None
    if not isinstance(templates, list):
        raise AutomationError("template manifest must contain a templates list")
    by_id = {}
    expected_cones = {"cone_{}".format(number) for number in range(10, 20)}
    for template in templates:
        if not isinstance(template, dict):
            raise AutomationError("every stress template must be a mapping")
        template_id = str(template.get("template_id", ""))
        if not template_id or template_id in by_id:
            raise AutomationError("stress template ids must be unique and nonempty")
        if template.get("target_class") not in TARGET_CLASS_IDS:
            raise AutomationError("{} has invalid target_class".format(template_id))
        cones = template.get("cones")
        if not isinstance(cones, dict) or set(cones) != expected_cones:
            raise AutomationError(
                "{} must contain cone_10 through cone_19".format(template_id)
            )
        by_id[template_id] = template

    if selected_ids:
        unknown = set(selected_ids) - set(by_id)
        if unknown:
            raise AutomationError(
                "unknown template ids: {}".format(", ".join(sorted(unknown)))
            )
        selected = [by_id[name] for name in selected_ids]
    else:
        if set(by_id) != EXPECTED_TEMPLATE_IDS:
            raise AutomationError(
                "default run requires the eight recorded collision templates"
            )
        selected = list(by_id.values())
    return sorted(selected, key=lambda value: int(value["source_round"]))


def load_runtime_config():
    delivery = load_yaml(DELIVERY_CONFIG)
    mission = load_yaml(MISSION_CONFIG)
    try:
        root = delivery["delivery"]
        frame_id = str(root["frame_id"])
        entry = root["entry_pose"]
        destinations = root["destinations"]
        switch_distance = mission.get("navigation_profiles", {}).get(
            "workshop_approach_distance", 0.22
        )
    except (KeyError, TypeError) as exc:
        raise AutomationError("delivery configuration is incomplete") from exc
    by_class = {}
    for raw in destinations:
        target_id = int(raw.get("target_class", -1))
        matched = [name for name, value in TARGET_CLASS_IDS.items() if value == target_id]
        if len(matched) != 1:
            raise AutomationError("delivery destination has invalid target_class")
        by_class[matched[0]] = {
            "frame_id": frame_id,
            "name": str(raw["name"]),
            "x": finite(raw["x"], "destination.x"),
            "y": finite(raw["y"], "destination.y"),
            "yaw": finite(raw["yaw"], "destination.yaw"),
            "position_tolerance": finite(
                raw.get("position_tolerance", 0.04),
                "destination.position_tolerance",
                positive=True,
            ),
            "yaw_tolerance": math.radians(
                finite(
                    raw.get("yaw_tolerance_deg", 5.0),
                    "destination.yaw_tolerance_deg",
                    positive=True,
                )
            ),
        }
    if set(by_class) != set(TARGET_CLASS_IDS):
        raise AutomationError("delivery goals must define all three target classes")
    return {
        "robot_start": {
            "x": finite(entry["x"], "entry_pose.x"),
            "y": finite(entry["y"], "entry_pose.y"),
            "yaw": finite(entry["yaw"], "entry_pose.yaw"),
            "z": 0.01,
        },
        "switch_distance": finite(
            switch_distance,
            "navigation_profiles.workshop_approach_distance",
            positive=True,
        ),
        "destinations": by_class,
    }


def build_plans(templates, repetitions, runtime):
    plans = []
    round_number = 0
    for template in templates:
        target_class = template["target_class"]
        destination = runtime["destinations"][target_class]
        for repetition in range(1, repetitions + 1):
            round_number += 1
            plans.append(
                {
                    "round": round_number,
                    "scenario_id": template["template_id"],
                    "source_round": int(template["source_round"]),
                    "source_task_id": template.get("source_task_id", ""),
                    "repetition": repetition,
                    "target_class": target_class,
                    "target_name": destination["name"],
                    "robot_start": dict(runtime["robot_start"]),
                    "switch_distance": runtime["switch_distance"],
                    "destination": {
                        key: value for key, value in destination.items() if key != "name"
                    },
                    "cones": template["cones"],
                }
            )
    return plans


def validate_args(args):
    if args.repetitions <= 0:
        raise AutomationError("--repetitions must be positive")
    for name in (
        "startup_timeout",
        "scene_timeout",
        "navigation_timeout",
        "navigation_ready_timeout",
        "monitor_ready_timeout",
        "gazebo_recording_ready_timeout",
        "cone_motion_threshold",
        "cone_tilt_threshold_deg",
    ):
        if getattr(args, name) <= 0.0:
            raise AutomationError("--{} must be positive".format(name.replace("_", "-")))
    for name in ("startup_settle", "restart_settle"):
        if getattr(args, name) < 0.0:
            raise AutomationError(
                "--{} must be non-negative".format(name.replace("_", "-"))
            )
    for path in (
        SETUP_FILE,
        args.templates.expanduser().resolve(),
        DELIVERY_CONFIG,
        MISSION_CONFIG,
        RANDOM_SPAWNER,
        SCENE_HELPER,
        NAVIGATION_HELPER,
    ):
        if not path.is_file():
            raise AutomationError("missing required file {}".format(path))


def parse_marker(output, marker):
    matches = [
        line[len(marker) :]
        for line in output.splitlines()
        if line.startswith(marker)
    ]
    if not matches:
        raise AutomationError("command output did not contain {}".format(marker))
    try:
        return json.loads(matches[-1])
    except ValueError as exc:
        raise AutomationError("{} payload is invalid JSON".format(marker)) from exc


def new_record(plan, round_dir, recording_round_dir):
    task_id = "cone_stress_{:03d}_{}".format(plan["round"], int(time.time()))
    case_file = round_dir / "case_plan.json"
    return {
        "round": plan["round"],
        "scenario_id": plan["scenario_id"],
        "source_round": plan["source_round"],
        "repetition": plan["repetition"],
        "task_id": task_id,
        "target_class": plan["target_class"],
        "target_name": plan["target_name"],
        "success": False,
        "status": "not_started",
        "completed_stage": None,
        "error_code": None,
        "message": "",
        "navigation_failure": False,
        "last_operational_stage": None,
        "last_operational_stage_name": "",
        "preceding_navigation_failure": False,
        "delivery_navigation_failure": False,
        "planner_mode": "fixed_teb",
        "profile_switching_required": False,
        "profile_switched": False,
        "profile_switch_distance_m": None,
        "minimum_goal_distance_m": None,
        "cone_collision": False,
        "collision_cones": "",
        "direct_contact_cones": "",
        "moved_or_tilted_cones": "",
        "cone_monitor_status": "not_started",
        "contact_stream_status": "not_started",
        "gazebo_recording_status": "not_started",
        "gazebo_recording_path": str(recording_round_dir / "gazebo_world_state.log"),
        "gazebo_recording_size_bytes": 0,
        "gazebo_recording_manifest": str(
            recording_round_dir / "gazebo_world_recording.json"
        ),
        "move_base_goal_count": 0,
        "duration_seconds": 0.0,
        "started_at": dt.datetime.now().astimezone().isoformat(timespec="seconds"),
        "round_dir": str(round_dir),
        "case_file": str(case_file),
        "log_file": str(round_dir / "roslaunch.log"),
        "cone_monitor_file": str(round_dir / "cone_trial.json"),
    }


def run_trial(args, plan, output_dir, recording_run_dir):
    round_number = plan["round"]
    round_dir = output_dir / "round_{:03d}".format(round_number)
    round_dir.mkdir(parents=True, exist_ok=False)
    recording_round_dir = recording_run_dir / "round_{:03d}".format(round_number)
    record = new_record(plan, round_dir, recording_round_dir)
    case_payload = dict(plan)
    case_payload["task_id"] = record["task_id"]
    case_file = Path(record["case_file"])
    case_file.write_text(
        json.dumps(case_payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )

    started = time.monotonic()
    launch_process = None
    launch_run_id = None
    monitor_process = None
    monitor_log = None
    monitor_ready = None
    recorder_process = None
    recorder_log = None
    recorder_ready = None
    phase = "startup"

    with Path(record["log_file"]).open("w", encoding="utf-8") as log_file:
        try:
            if master_is_running():
                raise CleanupError(
                    "a ROS master is still running before round {}; refusing to "
                    "reuse a stale simulation".format(round_number)
                )
            launch_process = launch_simulation(
                args.gui, log_file, start_perception=False
            )
            deadline = time.monotonic() + args.startup_timeout
            wait_for_ros("ROS master", ["rosnode", "list"], deadline)
            launch_run_id = read_ros_run_id()
            wait_for_simulation(args.startup_timeout)
            if launch_process.poll() is not None:
                raise AutomationError("roslaunch exited during startup")
            if args.startup_settle:
                time.sleep(args.startup_settle)

            phase = "navigation_readiness"
            return_code, output = run_owned(
                [
                    "python3",
                    str(NAVIGATION_HELPER),
                    "--case-file",
                    str(case_file),
                    "--readiness-only",
                    "--skip-profile-services",
                    "--navigation-wait-timeout",
                    str(args.navigation_ready_timeout),
                ],
                timeout=args.navigation_ready_timeout * 3.0 + 10.0,
            )
            append_section(log_file, "navigation stack readiness", output)
            if return_code != 0:
                raise AutomationError("navigation stack readiness check failed")

            phase = "scene_setup"
            return_code, output = run_owned(
                ["python3", str(SCENE_HELPER), "--case-file", str(case_file)],
                timeout=args.scene_timeout,
            )
            append_section(log_file, "fixed cone scene", output)
            if return_code != 0:
                raise AutomationError("fixed cone scene setup failed")
            measured = parse_marker(output, "CONE_MOVE_STRESS_SCENE=")
            (round_dir / "scene_measured.json").write_text(
                json.dumps(measured, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )

            phase = "gazebo_recording_startup"
            recorder_process, recorder_log, recorder_ready = (
                start_gazebo_world_recorder(
                    args, record, recording_round_dir, start_stage=16
                )
            )
            (recording_round_dir / "case_plan.json").write_text(
                json.dumps(case_payload, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )

            phase = "cone_monitor_startup"
            monitor_process, monitor_log, monitor_ready = start_cone_monitor(
                args, record, round_dir
            )

            phase = "navigation"
            return_code, output = run_owned(
                [
                    "python3",
                    str(NAVIGATION_HELPER),
                    "--case-file",
                    str(case_file),
                    "--skip-profile-services",
                    "--navigation-wait-timeout",
                    str(args.navigation_ready_timeout),
                ],
                timeout=args.navigation_timeout,
            )
            append_section(log_file, "delivery-only navigation", output)
            navigation = parse_marker(output, RESULT_MARKER)
            for key in (
                "success",
                "status",
                "completed_stage",
                "error_code",
                "message",
                "planner_mode",
                "profile_switching_required",
                "profile_switched",
                "profile_switch_distance_m",
                "minimum_goal_distance_m",
            ):
                if key in navigation:
                    record[key] = navigation[key]
            record["success"] = bool(record["success"] and return_code == 0)
            if not record["success"] and record["status"] == "navigation_completed":
                record["status"] = "navigation_client_error"
            record["navigation_failure"] = not record["success"]
        except CleanupError:
            raise
        except AutomationError as exc:
            record["status"] = "{}_error".format(phase)
            record["message"] = str(exc)
            record["navigation_failure"] = phase == "navigation"
            append_section(log_file, "automation error", str(exc))
        finally:
            try:
                stop_process_group(monitor_process, interrupt_timeout=6.0)
                if monitor_log is not None:
                    monitor_log.close()
                if monitor_ready is not None:
                    try:
                        monitor_ready.unlink()
                    except FileNotFoundError:
                        pass
            finally:
                try:
                    stop_process_group(recorder_process, interrupt_timeout=12.0)
                    if recorder_log is not None:
                        recorder_log.close()
                    if recorder_ready is not None:
                        try:
                            recorder_ready.unlink()
                        except FileNotFoundError:
                            pass
                finally:
                    try:
                        stop_owned_launch(launch_process, launch_run_id)
                    finally:
                        record["duration_seconds"] = round(
                            time.monotonic() - started, 3
                        )

    monitor, monitor_error = load_monitor(record)
    add_monitor_result(record, monitor, monitor_error)
    add_gazebo_recording_result(record)
    return record


def make_summary(results, requested_rounds):
    successes = sum(bool(record["success"]) for record in results)
    by_scenario = {}
    for scenario_id in sorted({record["scenario_id"] for record in results}):
        selected = [record for record in results if record["scenario_id"] == scenario_id]
        by_scenario[scenario_id] = {
            "source_round": selected[0]["source_round"],
            "target_class": selected[0]["target_class"],
            "rounds": len(selected),
            "navigation_successes": sum(bool(value["success"]) for value in selected),
            "cone_collision_rounds": sum(
                bool(value["cone_collision"]) for value in selected
            ),
        }
    return {
        "requested_rounds": requested_rounds,
        "completed_rounds": len(results),
        "navigation_successes": successes,
        "navigation_failures": len(results) - successes,
        "success_rate": round(successes / len(results), 4) if results else None,
        "cone_collision_rounds": sum(
            bool(record["cone_collision"]) for record in results
        ),
        "direct_contact_rounds": sum(
            bool(record["direct_contact_cones"]) for record in results
        ),
        "movement_evidence_rounds": sum(
            bool(record["moved_or_tilted_cones"]) for record in results
        ),
        "gazebo_recording_complete_rounds": sum(
            record["gazebo_recording_status"] == "complete" for record in results
        ),
        "infrastructure_errors": sum(
            str(record["status"]).endswith("_error")
            or record["cone_monitor_status"] != "complete"
            or record["gazebo_recording_status"] != "complete"
            for record in results
        ),
        "by_scenario": by_scenario,
    }


def write_reports(output_dir, results, plans, metadata):
    csv_path = output_dir / "trials.csv"
    with csv_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS)
        writer.writeheader()
        writer.writerows(results)
    summary = make_summary(results, len(plans))
    report_path = output_dir / "summary.json"
    report_path.write_text(
        json.dumps(
            {"metadata": metadata, "summary": summary, "trials": results},
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    return csv_path, report_path, summary


def print_summary(summary, csv_path, report_path):
    print("\n===== cone movement stress summary =====")
    print(
        "completed={}/{} navigation_successes={} failures={} collisions={}".format(
            summary["completed_rounds"],
            summary["requested_rounds"],
            summary["navigation_successes"],
            summary["navigation_failures"],
            summary["cone_collision_rounds"],
        )
    )
    print(
        "recordings_complete={} infrastructure_errors={}".format(
            summary["gazebo_recording_complete_rounds"],
            summary["infrastructure_errors"],
        )
    )
    print("csv={}".format(csv_path))
    print("json={}".format(report_path))


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    results = []
    output_dir = None
    metadata = None
    plans = []
    try:
        validate_args(args)
        templates = load_templates(args.templates, args.template_ids)
        runtime = load_runtime_config()
        plans = build_plans(templates, args.repetitions, runtime)
        metadata = {
            "created_at": dt.datetime.now().astimezone().isoformat(timespec="seconds"),
            "template_manifest": str(args.templates.expanduser().resolve()),
            "template_manifest_sha256": sha256(args.templates.expanduser().resolve()),
            "template_ids": [template["template_id"] for template in templates],
            "repetitions_per_template": args.repetitions,
            "schedule": [
                {
                    "round": plan["round"],
                    "scenario_id": plan["scenario_id"],
                    "repetition": plan["repetition"],
                    "target_class": plan["target_class"],
                }
                for plan in plans
            ],
            "delivery_config": str(DELIVERY_CONFIG),
            "mission_config": str(MISSION_CONFIG),
            "planner_mode": "fixed_teb",
            "profile_switching": "disabled",
            "navigation_ready_timeout_seconds": args.navigation_ready_timeout,
            "robot_start": runtime["robot_start"],
            "random_spawner": str(RANDOM_SPAWNER),
            "random_spawner_sha256": sha256(RANDOM_SPAWNER),
            "gazebo_gui_enabled": args.gui,
            "gazebo_recording": {
                "enabled": True,
                "format": "Gazebo Classic replayable state.log (zlib)",
                "start_stage": "NAVIGATE_TO_DELIVERY",
                "root": str(args.recording_root.expanduser().resolve()),
            },
        }
        if args.dry_run:
            print(
                "templates={} repetitions={} rounds={} gazebo_gui={}".format(
                    len(templates),
                    args.repetitions,
                    len(plans),
                    "enabled" if args.gui else "disabled",
                )
            )
            for plan in plans:
                print(
                    "round_{:03d}: {} repetition={} target={}".format(
                        plan["round"],
                        plan["scenario_id"],
                        plan["repetition"],
                        plan["target_class"],
                    )
                )
            print("recording_root={}".format(args.recording_root.expanduser().resolve()))
            return 0

        if master_is_running():
            raise AutomationError(
                "a ROS master is already running; stop the manual test before "
                "starting isolated stress trials"
            )
        timestamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        output_dir = (
            args.log_dir.expanduser().resolve()
            if args.log_dir is not None
            else WORKSPACE / "script" / "logs" / ("cone_move_stress_" + timestamp)
        )
        output_dir.mkdir(parents=True, exist_ok=False)
        recording_run_dir = (
            args.recording_root.expanduser().resolve() / output_dir.name
        )
        recording_run_dir.mkdir(parents=True, exist_ok=False)
        metadata["gazebo_recording"]["run_dir"] = str(recording_run_dir)
        print("workspace={}".format(WORKSPACE))
        print("rounds={} logs={}".format(len(plans), output_dir))
        print("gazebo_gui={}".format("enabled" if args.gui else "disabled"))
        print("gazebo_recording={}".format(recording_run_dir))

        for plan in plans:
            print(
                "[round {}/{}] scenario={} repetition={} target={}".format(
                    plan["round"],
                    len(plans),
                    plan["scenario_id"],
                    plan["repetition"],
                    plan["target_class"],
                ),
                flush=True,
            )
            record = run_trial(args, plan, output_dir, recording_run_dir)
            results.append(record)
            csv_path, report_path, _summary = write_reports(
                output_dir, results, plans, metadata
            )
            print(
                "  status={} success={} collision={} cones={} recording={} "
                "duration={:.1f}s".format(
                    record["status"],
                    record["success"],
                    record["cone_collision"],
                    record["collision_cones"] or "none",
                    record["gazebo_recording_status"],
                    record["duration_seconds"],
                ),
                flush=True,
            )
            if plan["round"] < len(plans) and args.restart_settle:
                time.sleep(args.restart_settle)

        csv_path, report_path, summary = write_reports(
            output_dir, results, plans, metadata
        )
        print_summary(summary, csv_path, report_path)
        return 0
    except KeyboardInterrupt:
        print("interrupted by user", file=sys.stderr)
        if output_dir is not None and metadata is not None:
            paths = write_reports(output_dir, results, plans, metadata)
            print_summary(paths[2], paths[0], paths[1])
        return 130
    except (AutomationError, CleanupError, OSError, subprocess.SubprocessError) as exc:
        print("run_cone_move_stress_trials: {}".format(exc), file=sys.stderr)
        if output_dir is not None and metadata is not None:
            paths = write_reports(output_dir, results, plans, metadata)
            print_summary(paths[2], paths[0], paths[1])
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
