#!/usr/bin/env python3
"""Run 40 isolated end-to-end tasks and retain cone-collision layouts."""

import argparse
import csv
import datetime as dt
import hashlib
import json
from pathlib import Path
import random
import secrets
import shutil
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
    ros_command,
    run_owned,
    stop_owned_launch,
    stop_process_group,
    wait_for_ros,
    wait_for_simulation,
)
from run_grasp_trials import TASK_CLASSES, parse_task_result


MONITOR_SCRIPT = WORKSPACE / "script" / "record_cone_trial.py"
GAZEBO_RECORDER_SCRIPT = WORKSPACE / "script" / "record_gazebo_world.py"
DEFAULT_VIDEO_ROOT = WORKSPACE / "data" / "cone_zone" / "end_to_end_test"
MISSION_CONFIG = WORKSPACE / "src" / "smart_factory_mission" / "config" / "mission.yaml"
RANDOM_SPAWNER = WORKSPACE / "src" / "car3" / "scripts" / "spawn_cubes.py"
NAVIGATION_ERROR_CODES = {3, 4, 5, 6, 7, 11}
PRE_NAVIGATION_STAGES = {3, 5, 8, 10}
DELIVERY_NAVIGATION_STAGES = {14, 15, 16}
CSV_FIELDS = (
    "round",
    "task_id",
    "target_class",
    "success",
    "status",
    "completed_stage",
    "error_code",
    "message",
    "last_operational_stage",
    "last_operational_stage_name",
    "preceding_navigation_failure",
    "delivery_navigation_failure",
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
    "log_file",
    "cone_monitor_file",
    "stress_template_file",
)


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description=(
            "restart the complete simulation for every round, randomly send "
            "a food/daily/electronics task, and retain cone-collision layouts"
        )
    )
    parser.add_argument(
        "--rounds", type=int, default=40, help="number of trials (default: 40)"
    )
    parser.add_argument(
        "--seed", type=int, help="task RNG seed; omitted means generate and record one"
    )
    display = parser.add_mutually_exclusive_group()
    display.add_argument(
        "--gui",
        dest="gui",
        action="store_true",
        help="show the hardware-accelerated Gazebo GUI",
    )
    display.add_argument(
        "--headless",
        dest="gui",
        action="store_false",
        help=(
            "do not start gzclient (default); Gazebo state recording "
            "remains enabled"
        ),
    )
    parser.set_defaults(gui=False)
    parser.add_argument("--startup-timeout", type=float, default=90.0)
    parser.add_argument("--startup-settle", type=float, default=8.0)
    parser.add_argument("--task-timeout", type=float, default=480.0)
    parser.add_argument("--monitor-ready-timeout", type=float, default=15.0)
    parser.add_argument(
        "--gazebo-recording-ready-timeout",
        "--video-ready-timeout",
        dest="gazebo_recording_ready_timeout",
        type=float,
        default=15.0,
    )
    parser.add_argument("--restart-settle", type=float, default=3.0)
    parser.add_argument(
        "--cone-motion-threshold",
        type=float,
        default=0.01,
        help="XY motion treated as collision evidence in metres (default: 0.01)",
    )
    parser.add_argument(
        "--cone-tilt-threshold-deg",
        type=float,
        default=5.0,
        help="tilt change treated as collision evidence (default: 5 degrees)",
    )
    parser.add_argument(
        "--disable-contact-stream",
        action="store_true",
        help=(
            "disable direct Gazebo contact parsing and use cone motion/tilt "
            "evidence only"
        ),
    )
    parser.add_argument(
        "--recording-root",
        "--video-root",
        dest="recording_root",
        type=Path,
        default=DEFAULT_VIDEO_ROOT,
        help=(
            "replayable Gazebo state-log root (default: "
            "/home/ianzhang/gazebo_ws/data/cone_zone/end_to_end_test)"
        ),
    )
    parser.add_argument(
        "--disable-gazebo-recording",
        "--disable-gazebo-video",
        dest="disable_gazebo_recording",
        action="store_true",
        help="disable the default per-round replayable Gazebo state log",
    )
    parser.add_argument(
        "--log-dir",
        type=Path,
        help="result directory; default: script/logs/end_to_end_cone_trials_<time>",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="validate configuration and print the random task sequence only",
    )
    return parser.parse_args(argv)


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def validate_args(args):
    if args.rounds <= 0:
        raise AutomationError("--rounds must be positive")
    for name in (
        "startup_timeout",
        "task_timeout",
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
        MONITOR_SCRIPT,
        GAZEBO_RECORDER_SCRIPT,
        MISSION_CONFIG,
        RANDOM_SPAWNER,
    ):
        if not path.is_file():
            raise AutomationError("missing required file {}".format(path))
    try:
        mission = yaml.safe_load(MISSION_CONFIG.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise AutomationError("cannot read {}: {}".format(MISSION_CONFIG, exc))
    if not isinstance(mission, dict) or mission.get("pipeline_stop_after") != "TASK_COMPLETED":
        raise AutomationError(
            "{} must set pipeline_stop_after: TASK_COMPLETED for an end-to-end run".format(
                MISSION_CONFIG
            )
        )


def append_section(stream, title, output):
    stream.write("\n===== {} =====\n".format(title))
    stream.write(output)
    if output and not output.endswith("\n"):
        stream.write("\n")
    stream.flush()


def new_record(
    round_number,
    task_id,
    target_class,
    round_dir,
    recording_round_dir,
    recording_enabled=True,
):
    return {
        "round": round_number,
        "task_id": task_id,
        "target_class": target_class,
        "success": False,
        "status": "not_started",
        "completed_stage": None,
        "error_code": None,
        "message": "",
        "last_operational_stage": None,
        "last_operational_stage_name": "",
        "preceding_navigation_failure": False,
        "delivery_navigation_failure": False,
        "cone_collision": False,
        "collision_cones": "",
        "direct_contact_cones": "",
        "moved_or_tilted_cones": "",
        "cone_monitor_status": "not_started",
        "contact_stream_status": "not_started",
        "gazebo_recording_status": (
            "not_started" if recording_enabled else "disabled"
        ),
        "gazebo_recording_path": str(
            recording_round_dir / "gazebo_world_state.log"
        ),
        "gazebo_recording_size_bytes": 0,
        "gazebo_recording_manifest": str(
            recording_round_dir / "gazebo_world_recording.json"
        ),
        "move_base_goal_count": 0,
        "duration_seconds": 0.0,
        "started_at": dt.datetime.now().astimezone().isoformat(timespec="seconds"),
        "round_dir": str(round_dir),
        "log_file": str(round_dir / "roslaunch.log"),
        "cone_monitor_file": str(round_dir / "cone_trial.json"),
        "stress_template_file": "",
    }


def start_cone_monitor(args, record, round_dir):
    ready_path = round_dir / ".cone_monitor.ready"
    monitor_log_path = round_dir / "cone_monitor.log"
    monitor_log = monitor_log_path.open("w", encoding="utf-8")
    monitor_arguments = [
        "python3",
        str(MONITOR_SCRIPT),
        "--output",
        record["cone_monitor_file"],
        "--ready-file",
        str(ready_path),
        "--round",
        str(record["round"]),
        "--task-id",
        record["task_id"],
        "--target-class",
        record["target_class"],
        "--motion-threshold",
        str(args.cone_motion_threshold),
        "--tilt-threshold-deg",
        str(args.cone_tilt_threshold_deg),
    ]
    if args.disable_contact_stream:
        monitor_arguments.append("--disable-contact-stream")
    command = ros_command(monitor_arguments)
    process = None
    try:
        process = subprocess.Popen(
            command,
            cwd=str(WORKSPACE),
            stdout=monitor_log,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )
        deadline = time.monotonic() + args.monitor_ready_timeout
        while time.monotonic() < deadline:
            if ready_path.is_file():
                return process, monitor_log, ready_path
            if process.poll() is not None:
                raise AutomationError(
                    "cone monitor exited during startup; see {}".format(monitor_log_path)
                )
            time.sleep(0.1)
        raise AutomationError(
            "cone monitor did not observe all ten cones within {:.1f}s; see {}".format(
                args.monitor_ready_timeout, monitor_log_path
            )
        )
    except Exception:
        stop_process_group(process, interrupt_timeout=2.0)
        monitor_log.close()
        raise


def start_gazebo_world_recorder(
    args,
    record,
    recording_round_dir,
    start_stage=5,
    start_immediately=False,
):
    recording_round_dir.mkdir(parents=True, exist_ok=False)
    ready_path = recording_round_dir / ".gazebo_world_recorder.ready"
    recorder_log_path = recording_round_dir / "gazebo_world_recorder.log"
    recorder_log = recorder_log_path.open("w", encoding="utf-8")
    recorder_arguments = [
        "python3",
        str(GAZEBO_RECORDER_SCRIPT),
        "--output-dir",
        str(recording_round_dir),
        "--round",
        str(record["round"]),
        "--task-id",
        record["task_id"],
    ]
    if start_immediately:
        recorder_arguments.append("--start-immediately")
    else:
        if start_stage is None:
            raise AutomationError(
                "start_stage is required unless start_immediately is enabled"
            )
        recorder_arguments.extend(["--start-stage", str(start_stage)])
    recorder_arguments.extend(["--ready-file", str(ready_path)])
    command = ros_command(recorder_arguments)
    process = None
    try:
        process = subprocess.Popen(
            command,
            cwd=str(WORKSPACE),
            stdout=recorder_log,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )
        deadline = time.monotonic() + args.gazebo_recording_ready_timeout
        while time.monotonic() < deadline:
            if ready_path.is_file():
                return process, recorder_log, ready_path
            if process.poll() is not None:
                raise AutomationError(
                    "Gazebo world recorder exited during startup; see {}".format(
                        recorder_log_path
                    )
                )
            time.sleep(0.1)
        raise AutomationError(
            "Gazebo world recorder was not ready within {:.1f}s; see {}".format(
                args.gazebo_recording_ready_timeout, recorder_log_path
            )
        )
    except Exception:
        stop_process_group(process, interrupt_timeout=4.0)
        recorder_log.close()
        raise


def read_json(path):
    try:
        return json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None


def add_gazebo_recording_result(record):
    if record["gazebo_recording_status"] == "disabled":
        return
    manifest_path = Path(record["gazebo_recording_manifest"])
    manifest = read_json(manifest_path)
    if manifest is None:
        record["gazebo_recording_status"] = "manifest_unavailable"
        return
    record["gazebo_recording_status"] = str(manifest.get("status", "unknown"))
    try:
        record["gazebo_recording_size_bytes"] = int(
            manifest.get("size_bytes", 0)
        )
    except (TypeError, ValueError):
        record["gazebo_recording_status"] = "manifest_invalid"
        record["gazebo_recording_size_bytes"] = 0
    recording_file = manifest.get("recording_file")
    if recording_file:
        record["gazebo_recording_path"] = str(manifest_path.parent / recording_file)


def load_monitor(record):
    path = Path(record["cone_monitor_file"])
    try:
        return json.loads(path.read_text(encoding="utf-8")), ""
    except (OSError, ValueError) as exc:
        return None, str(exc)


def is_navigation_failure(record, last_stage, delivery):
    if record["success"]:
        return False
    if last_stage is None:
        return False
    in_requested_phase = (
        last_stage in DELIVERY_NAVIGATION_STAGES if delivery else last_stage < 14
    )
    if not in_requested_phase:
        return False
    if record["error_code"] in NAVIGATION_ERROR_CODES:
        return True
    return (
        record["status"] == "task_timeout"
        and (
            last_stage in DELIVERY_NAVIGATION_STAGES
            if delivery
            else last_stage in PRE_NAVIGATION_STAGES
        )
    )


def add_monitor_result(record, monitor, monitor_error=""):
    if monitor is None:
        record["cone_monitor_status"] = "manifest_unavailable"
        record["contact_stream_status"] = "manifest_unavailable"
        if monitor_error:
            record["message"] = "{}; cone monitor: {}".format(
                record["message"], monitor_error
            ).strip("; ")
        return
    record["cone_monitor_status"] = monitor.get("status", "unknown")
    record["last_operational_stage"] = monitor.get("last_operational_stage")
    record["last_operational_stage_name"] = monitor.get(
        "last_operational_stage_name", ""
    )
    record["cone_collision"] = bool(monitor.get("collision_detected"))
    evidence = monitor.get("collision_evidence", {})
    collision_cones = monitor.get("collision_cones", [])
    direct_cones = evidence.get("direct_contact_cones", [])
    moved_cones = evidence.get("moved_or_tilted_cones", [])
    record["collision_cones"] = ";".join(collision_cones)
    record["direct_contact_cones"] = ";".join(direct_cones)
    record["moved_or_tilted_cones"] = ";".join(moved_cones)
    record["contact_stream_status"] = monitor.get("contact_stream", {}).get(
        "status", "unknown"
    )
    record["move_base_goal_count"] = len(monitor.get("move_base_goals", []))
    last_stage = record["last_operational_stage"]
    record["preceding_navigation_failure"] = is_navigation_failure(
        record, last_stage, delivery=False
    )
    record["delivery_navigation_failure"] = is_navigation_failure(
        record, last_stage, delivery=True
    )


def build_stress_template(record, monitor):
    if monitor is None or not record["cone_collision"]:
        return None
    initial_scene = monitor.get("initial_scene", {})
    cones = {
        name: pose
        for name, pose in initial_scene.items()
        if name.startswith("cone_")
    }
    if not cones:
        return None
    return {
        "schema_version": 1,
        "template_id": "collision_round_{:03d}".format(record["round"]),
        "source_round": record["round"],
        "source_task_id": record["task_id"],
        "target_class": record["target_class"],
        "source_monitor": record["cone_monitor_file"],
        "random_spawner": str(RANDOM_SPAWNER),
        "random_spawner_sha256": sha256(RANDOM_SPAWNER),
        "collision_cones": record["collision_cones"].split(";"),
        "direct_contact_cones": (
            record["direct_contact_cones"].split(";")
            if record["direct_contact_cones"]
            else []
        ),
        "moved_or_tilted_cones": (
            record["moved_or_tilted_cones"].split(";")
            if record["moved_or_tilted_cones"]
            else []
        ),
        "cones": cones,
        "replay_policy": (
            "test-only Gazebo repositioning after the unchanged random spawner finishes"
        ),
    }


def write_stress_template(record, monitor, round_dir):
    template = build_stress_template(record, monitor)
    if template is None:
        return None
    path = round_dir / "stress_template.json"
    path.write_text(
        json.dumps(template, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    record["stress_template_file"] = str(path)
    return template


def run_trial(args, round_number, target_class, output_dir, recording_run_dir):
    round_dir = output_dir / "round_{:03d}".format(round_number)
    round_dir.mkdir(parents=True, exist_ok=False)
    recording_round_dir = recording_run_dir / "round_{:03d}".format(round_number)
    task_id = "cone_e2e_{:03d}_{}".format(round_number, int(time.time()))
    record = new_record(
        round_number,
        task_id,
        target_class,
        round_dir,
        recording_round_dir,
        recording_enabled=not args.disable_gazebo_recording,
    )
    started = time.monotonic()
    launch_process = None
    launch_run_id = None
    monitor_process = None
    monitor_log = None
    monitor_ready_path = None
    recording_process = None
    recording_log = None
    recording_ready_path = None
    phase = "startup"

    with Path(record["log_file"]).open("w", encoding="utf-8") as log_file:
        try:
            if master_is_running():
                raise CleanupError(
                    "a ROS master is still running before round {}; refusing to "
                    "reuse a stale simulation".format(round_number)
                )
            launch_process = launch_simulation(args.gui, log_file)
            deadline = time.monotonic() + args.startup_timeout
            wait_for_ros("ROS master", ["rosnode", "list"], deadline)
            launch_run_id = read_ros_run_id()
            wait_for_simulation(args.startup_timeout)
            if launch_process.poll() is not None:
                raise AutomationError("roslaunch exited during startup")
            if args.startup_settle:
                time.sleep(args.startup_settle)

            if not args.disable_gazebo_recording:
                phase = "gazebo_recording_startup"
                recording_process, recording_log, recording_ready_path = (
                    start_gazebo_world_recorder(
                        args, record, recording_round_dir
                    )
                )

            phase = "cone_monitor_startup"
            monitor_process, monitor_log, monitor_ready_path = start_cone_monitor(
                args, record, round_dir
            )
            phase = "task"
            try:
                return_code, output = run_owned(
                    [
                        "rosrun",
                        "smart_factory_tests",
                        "send_navigation_task.py",
                        "--task-id",
                        task_id,
                        "--target-class",
                        target_class,
                    ],
                    timeout=args.task_timeout,
                )
            except AutomationError as exc:
                record["status"] = "task_timeout"
                record["message"] = str(exc)
                append_section(log_file, "task client timeout", str(exc))
            else:
                append_section(log_file, "task client", output)
                parsed = parse_task_result(output)
                if parsed is None:
                    record["status"] = "result_unavailable"
                    record["message"] = (
                        "task client exited with code {} without a parseable result".format(
                            return_code
                        )
                    )
                else:
                    record["completed_stage"] = parsed["completed_stage"]
                    record["error_code"] = parsed["error_code"]
                    record["message"] = parsed["message"]
                    record["success"] = bool(
                        return_code == 0
                        and parsed["reported_success"]
                        and parsed["completed_stage"] == 20
                    )
                    record["status"] = (
                        "task_completed" if record["success"] else "task_failed"
                    )
        except CleanupError:
            raise
        except AutomationError as exc:
            record["status"] = "{}_error".format(phase)
            record["message"] = str(exc)
            append_section(log_file, "automation error", str(exc))
        finally:
            try:
                stop_process_group(monitor_process, interrupt_timeout=6.0)
                if monitor_log is not None:
                    monitor_log.close()
                if monitor_ready_path is not None:
                    try:
                        monitor_ready_path.unlink()
                    except FileNotFoundError:
                        pass
            finally:
                try:
                    stop_process_group(recording_process, interrupt_timeout=12.0)
                    if recording_log is not None:
                        recording_log.close()
                    if recording_ready_path is not None:
                        try:
                            recording_ready_path.unlink()
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
    template = write_stress_template(record, monitor, round_dir)
    return record, template


def make_summary(results, requested_rounds, seed):
    by_class = {}
    for target_class in TASK_CLASSES:
        selected = [r for r in results if r["target_class"] == target_class]
        by_class[target_class] = {
            "rounds": len(selected),
            "task_successes": sum(bool(r["success"]) for r in selected),
            "preceding_navigation_failures": sum(
                bool(r["preceding_navigation_failure"]) for r in selected
            ),
            "cone_collision_rounds": sum(
                bool(r["cone_collision"]) for r in selected
            ),
        }
    successes = sum(bool(record["success"]) for record in results)
    return {
        "requested_rounds": requested_rounds,
        "completed_rounds": len(results),
        "task_successes": successes,
        "task_failures": len(results) - successes,
        "success_rate": round(successes / len(results), 4) if results else None,
        "preceding_navigation_failures": sum(
            bool(record["preceding_navigation_failure"]) for record in results
        ),
        "delivery_navigation_failures": sum(
            bool(record["delivery_navigation_failure"]) for record in results
        ),
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
            record.get("gazebo_recording_status") in {"complete", "disabled"}
            for record in results
        ),
        "infrastructure_errors": sum(
            record["status"].endswith("_error")
            or record["status"] == "result_unavailable"
            or record["cone_monitor_status"] != "complete"
            or record.get("gazebo_recording_status")
            not in {"complete", "disabled"}
            for record in results
        ),
        "seed": seed,
        "by_class": by_class,
    }


def write_reports(
    output_dir,
    results,
    templates,
    requested_rounds,
    seed,
    metadata,
    mirror_dir=None,
):
    csv_path = output_dir / "trials.csv"
    with csv_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS)
        writer.writeheader()
        writer.writerows(results)
    summary = make_summary(results, requested_rounds, seed)
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
    templates_path = output_dir / "collision_templates.json"
    templates_path.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "generated_at": dt.datetime.now().astimezone().isoformat(
                    timespec="seconds"
                ),
                "random_spawner": metadata["random_spawner"],
                "random_spawner_sha256": metadata["random_spawner_sha256"],
                "policy": (
                    "templates are applied by a test-only repositioning helper; "
                    "spawn_cubes.py remains unchanged"
                ),
                "templates": templates,
            },
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    if mirror_dir is not None:
        mirror_dir.mkdir(parents=True, exist_ok=True)
        for report in (csv_path, report_path, templates_path):
            shutil.copy2(report, mirror_dir / report.name)
    return csv_path, report_path, templates_path, summary


def print_summary(summary, csv_path, report_path, templates_path):
    rate = summary["success_rate"]
    print("\n===== end-to-end cone trial summary =====")
    print(
        "completed={}/{} task_successes={} task_failures={} success_rate={}".format(
            summary["completed_rounds"],
            summary["requested_rounds"],
            summary["task_successes"],
            summary["task_failures"],
            "n/a" if rate is None else "{:.1%}".format(rate),
        )
    )
    print(
        "preceding_navigation_failures={} delivery_navigation_failures={}".format(
            summary["preceding_navigation_failures"],
            summary["delivery_navigation_failures"],
        )
    )
    print(
        "cone_collision_rounds={} direct_contact_rounds={} movement_evidence_rounds={}".format(
            summary["cone_collision_rounds"],
            summary["direct_contact_rounds"],
            summary["movement_evidence_rounds"],
        )
    )
    print(
        "gazebo_recording_complete_rounds={}".format(
            summary["gazebo_recording_complete_rounds"]
        )
    )
    print("infrastructure_errors={}".format(summary["infrastructure_errors"]))
    print("csv={}".format(csv_path))
    print("json={}".format(report_path))
    print("collision_templates={}".format(templates_path))


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    seed = args.seed if args.seed is not None else secrets.randbits(64)
    rng = random.Random(seed)
    task_sequence = [rng.choice(TASK_CLASSES) for _ in range(args.rounds)]
    results = []
    templates = []
    output_dir = None
    recording_run_dir = None
    metadata = None
    try:
        validate_args(args)
        metadata = {
            "created_at": dt.datetime.now().astimezone().isoformat(timespec="seconds"),
            "seed": seed,
            "task_sequence": task_sequence,
            "mission_config": str(MISSION_CONFIG),
            "random_spawner": str(RANDOM_SPAWNER),
            "random_spawner_sha256": sha256(RANDOM_SPAWNER),
            "cone_collision_detection": (
                "direct car3/cone Gazebo contacts OR post-baseline cone motion/tilt"
            ),
            "navigation_isolation": (
                "Gazebo evidence is read-only and is never fed to the planner"
            ),
            "scene_policy": (
                "normal random scene from unchanged spawn_cubes.py; no fixed "
                "cone stress template is loaded"
            ),
            "gazebo_recording": {
                "enabled": not args.disable_gazebo_recording,
                "format": "Gazebo Classic replayable state.log (zlib)",
                "start_stage": "NAVIGATE_TO_PICKUP_STAGING",
                "root": str(args.recording_root.expanduser().resolve()),
                "live_gui_required": False,
                "gazebo_gui_enabled": args.gui,
                "gui_rendering": (
                    "hardware (WSLg D3D12, NVIDIA adapter)"
                    if args.gui
                    else "disabled"
                ),
                "rendering_environment_source": (
                    "gazebo_nav/launch/gazebo_nav.launch"
                ),
            },
        }
        if args.dry_run:
            print("rounds={} seed={}".format(args.rounds, seed))
            print(
                "gazebo_gui={} rendering={}".format(
                    "enabled" if args.gui else "disabled",
                    "hardware (WSLg D3D12, NVIDIA adapter)"
                    if args.gui
                    else "headless",
                )
            )
            for index, target_class in enumerate(task_sequence, 1):
                print("round_{:03d}: {}".format(index, target_class))
            print("spawn_cubes_sha256={}".format(metadata["random_spawner_sha256"]))
            print(
                "gazebo_recording={}".format(
                    "disabled"
                    if args.disable_gazebo_recording
                    else args.recording_root.expanduser().resolve()
                )
            )
            return 0
        if master_is_running():
            raise AutomationError(
                "a ROS master is already running; stop the existing ROS/Gazebo "
                "session before starting isolated trials"
            )
        timestamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        output_dir = (
            args.log_dir.expanduser().resolve()
            if args.log_dir is not None
            else WORKSPACE
            / "script"
            / "logs"
            / ("end_to_end_cone_trials_" + timestamp)
        )
        output_dir.mkdir(parents=True, exist_ok=False)
        recording_run_dir = (
            args.recording_root.expanduser().resolve()
            / "{}_seed{}".format(output_dir.name, seed)
        )
        if not args.disable_gazebo_recording:
            recording_run_dir.mkdir(parents=True, exist_ok=False)
        metadata["gazebo_recording"]["run_dir"] = str(recording_run_dir)
        print("workspace={}".format(WORKSPACE))
        print("rounds={} seed={} logs={}".format(args.rounds, seed, output_dir))
        print(
            "gazebo_gui={} rendering={}".format(
                "enabled" if args.gui else "disabled",
                "hardware (WSLg D3D12, NVIDIA adapter)"
                if args.gui
                else "headless",
            )
        )
        if not args.disable_gazebo_recording:
            print("gazebo_recording={}".format(recording_run_dir))
        print("random_spawner_sha256={}".format(metadata["random_spawner_sha256"]))

        for round_number, target_class in enumerate(task_sequence, 1):
            print(
                "[round {}/{}] target_class={}".format(
                    round_number, args.rounds, target_class
                ),
                flush=True,
            )
            record, template = run_trial(
                args, round_number, target_class, output_dir, recording_run_dir
            )
            results.append(record)
            if template is not None:
                templates.append(template)
            csv_path, report_path, templates_path, _summary = write_reports(
                output_dir,
                results,
                templates,
                args.rounds,
                seed,
                metadata,
                (
                    recording_run_dir
                    if not args.disable_gazebo_recording
                    else None
                ),
            )
            print(
                "  status={} success={} pre_nav_failure={} cone_collision={} "
                "cones={} recording={} goals={} duration={:.1f}s".format(
                    record["status"],
                    record["success"],
                    record["preceding_navigation_failure"],
                    record["cone_collision"],
                    record["collision_cones"] or "none",
                    record["gazebo_recording_status"],
                    record["move_base_goal_count"],
                    record["duration_seconds"],
                ),
                flush=True,
            )
            if round_number < args.rounds and args.restart_settle:
                time.sleep(args.restart_settle)

        csv_path, report_path, templates_path, summary = write_reports(
            output_dir,
            results,
            templates,
            args.rounds,
            seed,
            metadata,
            recording_run_dir if not args.disable_gazebo_recording else None,
        )
        print_summary(summary, csv_path, report_path, templates_path)
        return 0
    except KeyboardInterrupt:
        print("interrupted by user", file=sys.stderr)
        if output_dir is not None and metadata is not None:
            paths = write_reports(
                output_dir,
                results,
                templates,
                args.rounds,
                seed,
                metadata,
                (
                    recording_run_dir
                    if not args.disable_gazebo_recording
                    else None
                ),
            )
            print_summary(paths[3], paths[0], paths[1], paths[2])
        return 130
    except (AutomationError, CleanupError, OSError, subprocess.SubprocessError) as exc:
        print("run_end_to_end_cone_trials: {}".format(exc), file=sys.stderr)
        if output_dir is not None and metadata is not None:
            paths = write_reports(
                output_dir,
                results,
                templates,
                args.rounds,
                seed,
                metadata,
                (
                    recording_run_dir
                    if not args.disable_gazebo_recording
                    else None
                ),
            )
            print_summary(paths[3], paths[0], paths[1], paths[2])
        return 1


if __name__ == "__main__":
    sys.exit(main())
