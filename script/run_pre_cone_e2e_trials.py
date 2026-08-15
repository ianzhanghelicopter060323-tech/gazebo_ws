#!/usr/bin/env python3
"""Run isolated end-to-end trials through OBJECT_GRASPED.

Every round starts a fresh competition simulation.  The normal Gazebo cube
spawner therefore chooses a new random permutation and position for all three
cubes.  The generated mission configuration stops after the target has been
recognized, aligned with, and grasped, before delivery/cone-zone navigation.
"""

import argparse
import csv
import datetime as dt
import json
import os
from pathlib import Path
import random
import re
import secrets
import subprocess
import sys
import time

from capture_pickup_dataset import (
    AutomationError,
    CleanupError,
    DEFAULT_MISSION,
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
    GAZEBO_RECORDER_SCRIPT,
    add_gazebo_recording_result,
    append_section,
    start_gazebo_world_recorder,
)
from run_grasp_trials import (
    TASK_CLASSES,
    TASK_CLASS_MODELS,
    add_recognition_result,
    parse_task_result,
    read_target_ground_truth,
)


DEFAULT_ROUNDS = 100
DEFAULT_RECORDING_ROOT = WORKSPACE / "data" / "teb_pre_cone"
DEFAULT_PROGRESS_TIMEOUT = 90.0
PROGRESS_TIMEOUT_MARKER = "TASK_PROGRESS_TIMEOUT="
FEEDBACK_STAGE_PATTERN = re.compile(r"\bstage=(\d+)\s+retry=\d+\s+detail=")
STUCK_ERROR_CODES = {
    6: "navigation_timeout",
    7: "navigation_aborted",
    11: "alignment_failed",
}
CSV_FIELDS = (
    "round",
    "task_id",
    "target_class",
    "target_ground_truth_station",
    "selected_station",
    "observed_class_id",
    "recognition_correct",
    "recognition_status",
    "recognition_failure",
    "stuck",
    "stuck_reason",
    "success",
    "status",
    "completed_stage",
    "last_operational_stage",
    "error_code",
    "message",
    "cube_scene_status",
    "cube_scene_file",
    "gazebo_recording_status",
    "gazebo_recording_path",
    "gazebo_recording_size_bytes",
    "gazebo_recording_manifest",
    "performance_file",
    "progress_timeout_seconds",
    "duration_seconds",
    "started_at",
    "round_dir",
    "log_file",
)


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description=(
            "restart the complete simulation for every round, use fully random "
            "cube layouts and task classes, stop after OBJECT_GRASPED, and "
            "record replayable Gazebo state"
        )
    )
    parser.add_argument(
        "--rounds",
        type=int,
        default=DEFAULT_ROUNDS,
        help="number of isolated trials (default: 100)",
    )
    parser.add_argument(
        "--seed",
        type=int,
        help="task-class RNG seed; cube placement remains freshly randomized by Gazebo",
    )
    display = parser.add_mutually_exclusive_group()
    display.add_argument("--gui", dest="gui", action="store_true")
    display.add_argument("--headless", dest="gui", action="store_false")
    parser.set_defaults(gui=False)
    parser.add_argument("--startup-timeout", type=float, default=90.0)
    parser.add_argument("--startup-settle", type=float, default=8.0)
    parser.add_argument("--task-timeout", type=float, default=420.0)
    parser.add_argument(
        "--progress-timeout",
        type=float,
        default=DEFAULT_PROGRESS_TIMEOUT,
        help=(
            "classify and cancel a stuck round after this many seconds without "
            "a task stage/detail change (default: 90)"
        ),
    )
    parser.add_argument(
        "--gazebo-recording-ready-timeout", type=float, default=15.0
    )
    parser.add_argument("--restart-settle", type=float, default=3.0)
    parser.add_argument(
        "--recording-root",
        type=Path,
        default=DEFAULT_RECORDING_ROOT,
        help=(
            "replayable recording root (default: "
            "/home/ianichinose/gazebo_ws/data/teb_pre_cone)"
        ),
    )
    parser.add_argument(
        "--log-dir",
        type=Path,
        help="result directory; default: script/logs/<generated run name>",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="validate inputs and print the deterministic task-class sequence",
    )
    return parser.parse_args(argv)


def validate_args(args):
    if args.rounds <= 0:
        raise AutomationError("--rounds must be positive")
    for name in (
        "startup_timeout",
        "task_timeout",
        "progress_timeout",
        "gazebo_recording_ready_timeout",
    ):
        if getattr(args, name) <= 0.0:
            raise AutomationError(
                "--{} must be positive".format(name.replace("_", "-"))
            )
    for name in ("startup_settle", "restart_settle"):
        if getattr(args, name) < 0.0:
            raise AutomationError(
                "--{} must be non-negative".format(name.replace("_", "-"))
            )
    for path in (SETUP_FILE, DEFAULT_MISSION, GAZEBO_RECORDER_SCRIPT):
        if not path.is_file():
            raise AutomationError("missing required file {}".format(path))


def prepare_pre_cone_mission(output_dir):
    """Copy the active mission and stop it before delivery navigation."""
    source = DEFAULT_MISSION.read_text(encoding="utf-8")
    lines = source.splitlines()
    matches = [
        index
        for index, line in enumerate(lines)
        if re.match(r"^pipeline_stop_after\s*:", line)
    ]
    if len(matches) != 1:
        raise AutomationError(
            "mission configuration must define one top-level pipeline_stop_after"
        )
    lines[matches[0]] = "pipeline_stop_after: OBJECT_GRASPED"
    output = output_dir / "mission_pre_cone_object_grasped.yaml"
    output.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return output


def parse_progress_timeout(output):
    matches = [
        line[len(PROGRESS_TIMEOUT_MARKER) :]
        for line in output.splitlines()
        if line.startswith(PROGRESS_TIMEOUT_MARKER)
    ]
    if not matches:
        return None
    try:
        payload = json.loads(matches[-1])
    except ValueError:
        return {"invalid_marker": matches[-1]}
    return payload if isinstance(payload, dict) else {"invalid_marker": matches[-1]}


def last_feedback_stage(output):
    matches = list(FEEDBACK_STAGE_PATTERN.finditer(output))
    return int(matches[-1].group(1)) if matches else None


def classify_stuck(record, progress_timeout=False, outer_timeout=False):
    if outer_timeout:
        return True, "outer_task_timeout"
    if progress_timeout:
        return True, "no_task_progress"
    try:
        error_code = int(record["error_code"])
    except (TypeError, ValueError):
        error_code = None
    if error_code in STUCK_ERROR_CODES:
        return True, STUCK_ERROR_CODES[error_code]
    if record["status"] in {"object_grasped", "task_failed"}:
        return False, ""
    return None, "classification_unavailable"


def classify_recognition_failure(record):
    status = record.get("recognition_status")
    if record.get("recognition_correct") is True and status == "correct":
        return False
    if status in {"incorrect", "reported_target_class_mismatch"}:
        return True
    if status == "no_target_selected":
        stage = record.get("last_operational_stage")
        error_code = record.get("error_code")
        if error_code == 10:
            return True
        if error_code is not None and error_code != 10:
            return False
        return True if stage == 9 else None
    return None


def _directory_size(path):
    try:
        result = subprocess.run(
            ["du", "-sb", str(path)],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            timeout=3.0,
            check=False,
        )
        return int(result.stdout.split()[0]) if result.returncode == 0 else None
    except (OSError, ValueError, subprocess.SubprocessError):
        return None


def capture_performance_snapshot(label):
    """Return one low-overhead host/Gazebo resource sample for this round."""
    memory = {}
    try:
        for line in Path("/proc/meminfo").read_text(encoding="utf-8").splitlines():
            key, value = line.split(":", 1)
            if key in {"MemTotal", "MemAvailable", "SwapTotal", "SwapFree"}:
                memory[key] = int(value.split()[0]) * 1024
    except (OSError, ValueError):
        pass
    processes = {}
    try:
        result = subprocess.run(
            ["ps", "-eo", "comm=,pcpu=,rss="],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            timeout=3.0,
            check=False,
        )
        for line in result.stdout.splitlines():
            fields = line.split()
            if len(fields) != 3:
                continue
            name, cpu, rss = fields
            if name not in {
                "gzserver", "gzclient", "rviz", "python3", "move_base"
            }:
                continue
            entry = processes.setdefault(name, {"cpu_percent": 0.0, "rss_bytes": 0})
            entry["cpu_percent"] += float(cpu)
            entry["rss_bytes"] += int(rss) * 1024
    except (OSError, ValueError, subprocess.SubprocessError):
        pass
    gazebo_stats = ""
    try:
        result = subprocess.run(
            ["gz", "stats", "-p"],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            timeout=2.0,
            check=False,
        )
        gazebo_stats = result.stdout.strip().splitlines()[-1]
    except (OSError, IndexError, subprocess.SubprocessError):
        pass
    return {
        "label": label,
        "captured_at": dt.datetime.now().astimezone().isoformat(timespec="milliseconds"),
        "load_average": list(os.getloadavg()),
        "memory": memory,
        "processes": processes,
        "ros_log_size_bytes": _directory_size(Path.home() / ".ros" / "log"),
        "gazebo_stats": gazebo_stats,
    }


def read_cube_scene(round_dir):
    cubes = {}
    errors = []
    for target_class in TASK_CLASSES:
        station, x, y, error = read_target_ground_truth(target_class)
        cubes[TASK_CLASS_MODELS[target_class]] = {
            "class": target_class,
            "station": station,
            "x": x,
            "y": y,
            "error": error,
        }
        if station is None:
            errors.append("{}: {}".format(target_class, error or "unknown station"))
    stations = [entry["station"] for entry in cubes.values()]
    if not errors and len(set(stations)) != len(TASK_CLASSES):
        errors.append("cube stations are not a one-to-one permutation")
    payload = {
        "captured_at": dt.datetime.now().astimezone().isoformat(timespec="milliseconds"),
        "status": "complete" if not errors else "incomplete",
        "spawner_policy": "fresh process; random cube-area permutation and coordinates",
        "cubes": cubes,
        "errors": errors,
    }
    path = round_dir / "cube_scene.json"
    path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return payload, path


def new_record(args, round_number, task_id, target_class, round_dir, recording_dir):
    return {
        "round": round_number,
        "task_id": task_id,
        "target_class": target_class,
        "target_ground_truth_station": None,
        "selected_station": None,
        "observed_class_id": None,
        "recognition_correct": False,
        "recognition_status": "not_evaluated",
        "recognition_failure": None,
        "stuck": None,
        "stuck_reason": "classification_unavailable",
        "success": False,
        "status": "not_started",
        "completed_stage": None,
        "last_operational_stage": None,
        "error_code": None,
        "message": "",
        "cube_scene_status": "not_captured",
        "cube_scene_file": str(round_dir / "cube_scene.json"),
        "gazebo_recording_status": "not_started",
        "gazebo_recording_path": str(recording_dir / "gazebo_world_state.log"),
        "gazebo_recording_size_bytes": 0,
        "gazebo_recording_manifest": str(
            recording_dir / "gazebo_world_recording.json"
        ),
        "performance_file": str(round_dir / "performance.json"),
        "progress_timeout_seconds": args.progress_timeout,
        "duration_seconds": 0.0,
        "started_at": dt.datetime.now().astimezone().isoformat(timespec="seconds"),
        "round_dir": str(round_dir),
        "log_file": str(round_dir / "roslaunch.log"),
    }


def apply_task_result(record, output, return_code):
    parsed = parse_task_result(output)
    record["last_operational_stage"] = last_feedback_stage(output)
    progress_timeout = parse_progress_timeout(output)
    if parsed is None:
        record["status"] = (
            "progress_timeout" if progress_timeout is not None else "result_unavailable"
        )
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
            and parsed["completed_stage"] == 14
            and parsed["error_code"] == 0
        )
        record["status"] = "object_grasped" if record["success"] else "task_failed"
    if record["last_operational_stage"] is None:
        record["last_operational_stage"] = record["completed_stage"]
    record["stuck"], record["stuck_reason"] = classify_stuck(
        record, progress_timeout=progress_timeout is not None
    )


def run_trial(
    args,
    round_number,
    target_class,
    output_dir,
    recording_run_dir,
    mission_config,
):
    round_dir = output_dir / "round_{:03d}".format(round_number)
    round_dir.mkdir(parents=True, exist_ok=False)
    recording_dir = recording_run_dir / "round_{:03d}".format(round_number)
    task_id = "pre_cone_{:03d}_{}".format(round_number, int(time.time()))
    record = new_record(
        args, round_number, task_id, target_class, round_dir, recording_dir
    )
    launch_process = None
    launch_run_id = None
    recorder_process = None
    recorder_log = None
    recorder_ready = None
    phase = "startup"
    started = time.monotonic()
    performance = []

    with Path(record["log_file"]).open("w", encoding="utf-8") as log_file:
        try:
            if master_is_running():
                raise CleanupError(
                    "a ROS master is still running before round {}; stop the "
                    "existing ROS/Gazebo session".format(round_number)
                )
            launch_process = launch_simulation(
                args.gui,
                log_file,
                mission_config=mission_config,
                start_perception=True,
            )
            deadline = time.monotonic() + args.startup_timeout
            wait_for_ros("ROS master", ["rosnode", "list"], deadline)
            launch_run_id = read_ros_run_id()
            wait_for_simulation(args.startup_timeout)
            wait_for_ros(
                "move_base action server",
                ["rostopic", "echo", "-n", "1", "/move_base/status", "--noarr"],
                deadline,
            )
            wait_for_ros(
                "cube perception service",
                ["rosservice", "info", "/cube_locator/locate"],
                deadline,
            )
            if launch_process.poll() is not None:
                raise AutomationError("roslaunch exited during startup")
            if args.startup_settle:
                time.sleep(args.startup_settle)

            scene, scene_path = read_cube_scene(round_dir)
            record["cube_scene_status"] = scene["status"]
            record["cube_scene_file"] = str(scene_path)
            target_scene = scene["cubes"].get(
                TASK_CLASS_MODELS[target_class], {}
            )
            record["target_ground_truth_station"] = target_scene.get(
                "station"
            )
            append_section(
                log_file,
                "random cube scene",
                json.dumps(scene, ensure_ascii=False, sort_keys=True),
            )

            phase = "gazebo_recording_startup"
            recorder_process, recorder_log, recorder_ready = (
                start_gazebo_world_recorder(
                    args,
                    record,
                    recording_dir,
                    start_stage=None,
                    start_immediately=True,
                )
            )

            phase = "task"
            performance.append(capture_performance_snapshot("before_task"))
            command = [
                "rosrun",
                "smart_factory_tests",
                "send_navigation_task.py",
                "--task-id",
                task_id,
                "--target-class",
                target_class,
                "_progress_timeout:={}".format(args.progress_timeout),
            ]
            try:
                return_code, task_output = run_owned(
                    command, timeout=args.task_timeout
                )
            except AutomationError as exc:
                record["status"] = "task_timeout"
                record["message"] = str(exc)
                record["stuck"], record["stuck_reason"] = classify_stuck(
                    record, outer_timeout=True
                )
                append_section(log_file, "task client timeout", str(exc))
            else:
                append_section(log_file, "task client", task_output)
                apply_task_result(record, task_output, return_code)
            performance.append(capture_performance_snapshot("after_task"))
        except CleanupError:
            raise
        except AutomationError as exc:
            record["status"] = "{}_error".format(phase)
            record["message"] = str(exc)
            append_section(log_file, "automation error", str(exc))
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
                    Path(record["performance_file"]).write_text(
                        json.dumps(performance, ensure_ascii=False, indent=2) + "\n",
                        encoding="utf-8",
                    )

    add_recognition_result(record, Path(record["log_file"]))
    record["recognition_failure"] = classify_recognition_failure(record)
    add_gazebo_recording_result(record)
    return record


def make_summary(results, requested_rounds, seed):
    completed = len(results)
    successes = sum(record["success"] is True for record in results)
    stuck = sum(record["stuck"] is True for record in results)
    recognition_failures = sum(
        record["recognition_failure"] is True for record in results
    )
    return {
        "requested_rounds": requested_rounds,
        "completed_rounds": completed,
        "object_grasped_successes": successes,
        "success_rate": round(successes / completed, 4) if completed else None,
        "stuck_rounds": stuck,
        "recognition_failure_rounds": recognition_failures,
        "indeterminate_stuck_rounds": sum(
            record["stuck"] is None for record in results
        ),
        "indeterminate_recognition_rounds": sum(
            record["recognition_failure"] is None for record in results
        ),
        "complete_gazebo_recordings": sum(
            record["gazebo_recording_status"] == "complete" for record in results
        ),
        "verified_random_cube_scenes": sum(
            record["cube_scene_status"] == "complete" for record in results
        ),
        "seed": seed,
    }


def write_reports(output_dir, recording_run_dir, results, args, metadata):
    def write_csv(path):
        with path.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS)
            writer.writeheader()
            writer.writerows(results)

    summary = make_summary(results, args.rounds, metadata["task_seed"])
    report = {"metadata": metadata, "summary": summary, "trials": results}
    csv_path = output_dir / "trials.csv"
    json_path = output_dir / "summary.json"
    write_csv(csv_path)
    json_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    write_csv(recording_run_dir / "trials.csv")
    (recording_run_dir / "summary.json").write_text(
        json_path.read_text(encoding="utf-8"), encoding="utf-8"
    )
    return csv_path, json_path, summary


def print_summary(summary, csv_path, json_path):
    rate = summary["success_rate"]
    print("\n===== pre-cone end-to-end trial summary =====")
    print(
        "completed={}/{} object_grasped={} success_rate={}".format(
            summary["completed_rounds"],
            summary["requested_rounds"],
            summary["object_grasped_successes"],
            "n/a" if rate is None else "{:.1%}".format(rate),
        )
    )
    print(
        "stuck={} recognition_failures={} indeterminate_stuck={} "
        "indeterminate_recognition={}".format(
            summary["stuck_rounds"],
            summary["recognition_failure_rounds"],
            summary["indeterminate_stuck_rounds"],
            summary["indeterminate_recognition_rounds"],
        )
    )
    print(
        "recordings_complete={} random_scenes_verified={}".format(
            summary["complete_gazebo_recordings"],
            summary["verified_random_cube_scenes"],
        )
    )
    print("csv={}".format(csv_path))
    print("json={}".format(json_path))


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    results = []
    output_dir = None
    recording_run_dir = None
    metadata = None
    seed = args.seed if args.seed is not None else secrets.randbits(64)
    rng = random.Random(seed)
    task_sequence = [rng.choice(TASK_CLASSES) for _ in range(args.rounds)]

    try:
        validate_args(args)
        if args.dry_run:
            print("rounds={} task_seed={}".format(args.rounds, seed))
            print("recording_root={}".format(args.recording_root.expanduser().resolve()))
            print("test_scope=through_object_grasped")
            print("cube_randomization=fresh spawn_cubes.py process every round")
            for index, target_class in enumerate(task_sequence, 1):
                print("round_{:03d}: {}".format(index, target_class))
            return 0
        if master_is_running():
            raise CleanupError(
                "a ROS master is already running; stop the existing ROS/Gazebo "
                "session before starting isolated pre-cone trials"
            )

        timestamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        run_name = "pre_cone_e2e_trials_{}_seed{}".format(timestamp, seed)
        output_dir = (
            args.log_dir.expanduser().resolve()
            if args.log_dir is not None
            else WORKSPACE / "script" / "logs" / run_name
        )
        output_dir.mkdir(parents=True, exist_ok=False)
        recording_run_dir = args.recording_root.expanduser().resolve() / run_name
        recording_run_dir.mkdir(parents=True, exist_ok=False)
        mission_config = prepare_pre_cone_mission(output_dir)
        metadata = {
            "created_at": dt.datetime.now().astimezone().isoformat(timespec="seconds"),
            "run_name": run_name,
            "test_scope": "startup_through_object_grasped_before_cone_delivery",
            "pipeline_stop_after": "OBJECT_GRASPED",
            "rounds": args.rounds,
            "task_seed": seed,
            "cube_randomization": "fresh spawn_cubes.py process every round",
            "headless": not args.gui,
            "progress_timeout_seconds": args.progress_timeout,
            "mission_config": str(mission_config),
            "recording_root": str(args.recording_root.expanduser().resolve()),
            "recording_run_dir": str(recording_run_dir),
            "recording_start": "immediately before task submission",
        }
        print("workspace={}".format(WORKSPACE))
        print("rounds={} task_seed={} logs={}".format(args.rounds, seed, output_dir))
        print("recordings={}".format(recording_run_dir))
        print("test_scope=through_object_grasped")
        print("progress_timeout={:.1f}s".format(args.progress_timeout))

        for round_number, target_class in enumerate(task_sequence, 1):
            print(
                "[round {}/{}] target_class={} fresh_random_cubes".format(
                    round_number, args.rounds, target_class
                ),
                flush=True,
            )
            record = run_trial(
                args,
                round_number,
                target_class,
                output_dir,
                recording_run_dir,
                mission_config,
            )
            results.append(record)
            csv_path, json_path, _summary = write_reports(
                output_dir, recording_run_dir, results, args, metadata
            )
            print(
                "  status={} success={} stuck={} recognition_failure={} "
                "recognition={} recording={} duration={:.1f}s".format(
                    record["status"],
                    record["success"],
                    record["stuck"],
                    record["recognition_failure"],
                    record["recognition_status"],
                    record["gazebo_recording_status"],
                    record["duration_seconds"],
                ),
                flush=True,
            )
            if round_number < args.rounds and args.restart_settle:
                time.sleep(args.restart_settle)

        csv_path, json_path, summary = write_reports(
            output_dir, recording_run_dir, results, args, metadata
        )
        print_summary(summary, csv_path, json_path)
        return 0
    except KeyboardInterrupt:
        print("interrupted by user", file=sys.stderr)
        if output_dir is not None and recording_run_dir is not None and metadata:
            csv_path, json_path, summary = write_reports(
                output_dir, recording_run_dir, results, args, metadata
            )
            print_summary(summary, csv_path, json_path)
        return 130
    except (AutomationError, CleanupError, OSError, subprocess.SubprocessError) as exc:
        print("run_pre_cone_e2e_trials: {}".format(exc), file=sys.stderr)
        if output_dir is not None and recording_run_dir is not None and metadata:
            csv_path, json_path, summary = write_reports(
                output_dir, recording_run_dir, results, args, metadata
            )
            print_summary(summary, csv_path, json_path)
        return 1


if __name__ == "__main__":
    sys.exit(main())
