#!/usr/bin/env python3
"""Run isolated grasp simulations and record their success rate.

Each trial starts a fresh ``full_competition.launch`` instance.  Gazebo's
``spawn_cubes.py`` randomizes the three cubes on every launch, and this script
independently chooses one of the three task classes with equal probability.
"""

import argparse
import csv
import datetime as dt
import json
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
    SETUP_FILE,
    WORKSPACE,
    launch_simulation,
    master_is_running,
    read_ros_run_id,
    run_owned,
    stop_owned_launch,
    wait_for_ros,
    wait_for_simulation,
)


TASK_CLASSES = ("food", "daily", "electronics")
TASK_CLASS_IDS = {"food": 0, "daily": 1, "electronics": 2}
# cube_0/1/2 use Food.png, Daily_Necessities.png and Electronics.png.
TASK_CLASS_MODELS = {"food": "cube_0", "daily": "cube_1", "electronics": "cube_2"}
# Spawn regions from car3/scripts/spawn_cubes.py, matched to the mission's
# observation stations by their calibrated base pose and heading.
STATION_AREAS = {
    35: (-0.95, -0.77, -0.69, -0.36),
    36: (-1.56, -1.23, -0.01, 0.17),
    37: (-2.10, -1.92, -0.61, -0.28),
}
SUCCESS_STAGES = {14, 20}  # OBJECT_GRASPED and TASK_COMPLETED
RESULT_PATTERN = re.compile(
    r"success=(True|False)\s+stage=(\d+)\s+error_code=(\d+)\s+message=(.*)"
)
SELECTION_PATTERN = re.compile(
    r"selected seq(35|36|37) for target class (\d+) \(observed class (\d+)\)"
)
SUCCESS_STATION_PATTERN = re.compile(r"grasped and lifted from seq(35|36|37)")
FLOAT_PATTERN = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"
POSITION_PATTERN = re.compile(
    r"position:\s*\n\s*x:\s*(?P<x>{0})\s*\n\s*y:\s*(?P<y>{0})".format(
        FLOAT_PATTERN
    )
)
CSV_FIELDS = (
    "round",
    "task_id",
    "target_class",
    "target_ground_truth_station",
    "selected_station",
    "observed_class_id",
    "recognition_correct",
    "recognition_status",
    "success",
    "status",
    "completed_stage",
    "error_code",
    "message",
    "duration_seconds",
    "started_at",
    "log_file",
)


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description=(
            "restart the complete simulation for every trial, randomly send "
            "food/daily/electronics, and record grasp successes"
        )
    )
    parser.add_argument(
        "--rounds", type=int, default=40, help="number of simulation trials (default: 40)"
    )
    parser.add_argument(
        "--seed",
        type=int,
        help="random seed; omitted means generate and record a new seed",
    )
    parser.add_argument("--gui", action="store_true", help="show the Gazebo GUI")
    parser.add_argument(
        "--startup-timeout",
        type=float,
        default=90.0,
        help="wall-clock timeout for ROS/Gazebo readiness",
    )
    parser.add_argument(
        "--startup-settle",
        type=float,
        default=8.0,
        help="extra settling time after readiness checks",
    )
    parser.add_argument(
        "--task-timeout",
        type=float,
        default=420.0,
        help="wall-clock timeout for one navigation and grasp task",
    )
    parser.add_argument(
        "--restart-settle",
        type=float,
        default=3.0,
        help="delay after shutting down one simulation",
    )
    parser.add_argument(
        "--log-dir",
        help=(
            "result directory; default: "
            "<workspace>/script/logs/grasp_trials_<timestamp>"
        ),
    )
    return parser.parse_args(argv)


def validate_args(args):
    if args.rounds <= 0:
        raise AutomationError("--rounds must be positive")
    for name in ("startup_timeout", "task_timeout"):
        if getattr(args, name) <= 0.0:
            raise AutomationError("--{} must be positive".format(name.replace("_", "-")))
    for name in ("startup_settle", "restart_settle"):
        if getattr(args, name) < 0.0:
            raise AutomationError(
                "--{} must be non-negative".format(name.replace("_", "-"))
            )
    if not SETUP_FILE.is_file():
        raise AutomationError("missing {}; run catkin_make first".format(SETUP_FILE))


def parse_task_result(output):
    """Return the final structured result emitted by send_navigation_task.py."""
    matches = list(RESULT_PATTERN.finditer(output))
    if not matches:
        return None
    match = matches[-1]
    return {
        "reported_success": match.group(1) == "True",
        "completed_stage": int(match.group(2)),
        "error_code": int(match.group(3)),
        "message": match.group(4).strip(),
    }


def station_for_position(x, y, tolerance=0.03):
    for station, (x_min, x_max, y_min, y_max) in STATION_AREAS.items():
        if (
            x_min - tolerance <= x <= x_max + tolerance
            and y_min - tolerance <= y <= y_max + tolerance
        ):
            return station

    # A previously contacted cube can settle a few centimetres outside its
    # spawn rectangle.  The three regions are far apart, so nearest-rectangle
    # matching remains unambiguous while rejecting a genuinely displaced cube.
    distances = {}
    for station, (x_min, x_max, y_min, y_max) in STATION_AREAS.items():
        dx = max(x_min - x, 0.0, x - x_max)
        dy = max(y_min - y, 0.0, y - y_max)
        distances[station] = (dx * dx + dy * dy) ** 0.5
    nearest = min(distances, key=distances.get)
    return nearest if distances[nearest] <= 0.25 else None


def read_target_ground_truth(target_class):
    """Read the target cube's actual station from Gazebo model state."""
    model_name = TASK_CLASS_MODELS[target_class]
    try:
        return_code, output = run_owned(
            [
                "rosservice",
                "call",
                "/gazebo/get_model_state",
                "{model_name: %s}" % model_name,
            ],
            timeout=5.0,
        )
    except AutomationError as exc:
        return None, None, None, str(exc)
    if return_code != 0 or "success: True" not in output:
        return None, None, None, output.strip()
    match = POSITION_PATTERN.search(output)
    if match is None:
        return None, None, None, "could not parse Gazebo model position"
    x = float(match.group("x"))
    y = float(match.group("y"))
    return station_for_position(x, y), x, y, ""


def add_recognition_result(record, log_path):
    """Compare the mission's selected station with Gazebo ground truth."""
    try:
        log_text = log_path.read_text(encoding="utf-8", errors="replace")
    except OSError as exc:
        record["recognition_status"] = "log_unavailable: {}".format(exc)
        return

    matches = list(SELECTION_PATTERN.finditer(log_text))
    if matches:
        selected = matches[-1]
        record["selected_station"] = int(selected.group(1))
        record["observed_class_id"] = int(selected.group(3))
        if int(selected.group(2)) != TASK_CLASS_IDS[record["target_class"]]:
            record["recognition_status"] = "reported_target_class_mismatch"
            return
    elif record["success"]:
        # The Action result contains the station too, and is a useful fallback
        # if roslaunch output was buffered at shutdown.
        station_match = SUCCESS_STATION_PATTERN.search(record["message"])
        if station_match:
            record["selected_station"] = int(station_match.group(1))

    ground_truth = record["target_ground_truth_station"]
    selected_station = record["selected_station"]
    if ground_truth is None:
        record["recognition_status"] = "ground_truth_unavailable"
    elif selected_station is None:
        record["recognition_status"] = "no_target_selected"
    elif selected_station == ground_truth:
        record["recognition_correct"] = True
        record["recognition_status"] = "correct"
    else:
        record["recognition_status"] = "incorrect"


def new_record(round_number, task_id, target_class, log_path):
    return {
        "round": round_number,
        "task_id": task_id,
        "target_class": target_class,
        "target_ground_truth_station": None,
        "selected_station": None,
        "observed_class_id": None,
        "recognition_correct": False,
        "recognition_status": "not_evaluated",
        "success": False,
        "status": "not_started",
        "completed_stage": None,
        "error_code": None,
        "message": "",
        "duration_seconds": 0.0,
        "started_at": dt.datetime.now().astimezone().isoformat(timespec="seconds"),
        "log_file": str(log_path),
    }


def run_trial(args, round_number, target_class, log_path):
    task_id = "grasp_trial_{:03d}_{}".format(round_number, int(time.time()))
    record = new_record(round_number, task_id, target_class, log_path)
    started = time.monotonic()
    launch_process = None
    launch_run_id = None
    phase = "startup"

    with log_path.open("w", encoding="utf-8") as log_file:
        try:
            if master_is_running():
                raise CleanupError(
                    "a ROS master is still running before round {}; refusing to "
                    "mix this trial with a stale simulation".format(round_number)
                )

            launch_process = launch_simulation(args.gui, log_file)
            master_deadline = time.monotonic() + args.startup_timeout
            wait_for_ros("ROS master", ["rosnode", "list"], master_deadline)
            launch_run_id = read_ros_run_id()
            wait_for_simulation(args.startup_timeout)
            if launch_process.poll() is not None:
                raise AutomationError("roslaunch exited during startup")

            if args.startup_settle:
                time.sleep(args.startup_settle)

            station, cube_x, cube_y, ground_truth_error = read_target_ground_truth(
                target_class
            )
            record["target_ground_truth_station"] = station
            if station is None:
                log_file.write(
                    "\n===== Gazebo ground truth =====\n"
                    "model={} station=unknown detail={}\n".format(
                        TASK_CLASS_MODELS[target_class], ground_truth_error
                    )
                )
            else:
                log_file.write(
                    "\n===== Gazebo ground truth =====\n"
                    "model={} class={} position=({:.6f}, {:.6f}) station={}\n".format(
                        TASK_CLASS_MODELS[target_class],
                        target_class,
                        cube_x,
                        cube_y,
                        station,
                    )
                )
            log_file.flush()

            phase = "task"
            command = [
                "rosrun",
                "smart_factory_tests",
                "send_navigation_task.py",
                "--task-id",
                task_id,
                "--target-class",
                target_class,
            ]
            return_code, task_output = run_owned(command, timeout=args.task_timeout)
            log_file.write("\n===== task client =====\n")
            log_file.write(task_output)
            log_file.flush()

            parsed = parse_task_result(task_output)
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
                    and parsed["completed_stage"] in SUCCESS_STAGES
                )
                record["status"] = "grasp_succeeded" if record["success"] else "task_failed"
        except CleanupError:
            raise
        except AutomationError as exc:
            record["status"] = "{}_error".format(phase)
            record["message"] = str(exc)
            log_file.write("\n===== automation error =====\n{}\n".format(exc))
            log_file.flush()
        finally:
            try:
                stop_owned_launch(launch_process, launch_run_id)
            finally:
                record["duration_seconds"] = round(time.monotonic() - started, 3)

    add_recognition_result(record, log_path)
    return record


def make_summary(results, requested_rounds, seed):
    by_class = {}
    for target_class in TASK_CLASSES:
        class_results = [r for r in results if r["target_class"] == target_class]
        succeeded = sum(bool(r["success"]) for r in class_results)
        correctly_recognized = sum(
            bool(r["recognition_correct"]) for r in class_results
        )
        by_class[target_class] = {
            "trials": len(class_results),
            "successes": succeeded,
            "success_rate": (
                round(succeeded / len(class_results), 4) if class_results else None
            ),
            "correct_recognitions": correctly_recognized,
            "recognition_accuracy": (
                round(correctly_recognized / len(class_results), 4)
                if class_results
                else None
            ),
        }

    successes = sum(bool(record["success"]) for record in results)
    correct_recognitions = sum(
        bool(record["recognition_correct"]) for record in results
    )
    infrastructure_errors = sum(
        record["status"] in {"startup_error", "task_error", "result_unavailable"}
        for record in results
    )
    return {
        "requested_rounds": requested_rounds,
        "completed_rounds": len(results),
        "successes": successes,
        "failures": len(results) - successes,
        "success_rate": round(successes / len(results), 4) if results else None,
        "correct_recognitions": correct_recognitions,
        "recognition_accuracy": (
            round(correct_recognitions / len(results), 4) if results else None
        ),
        "infrastructure_errors": infrastructure_errors,
        "seed": seed,
        "by_class": by_class,
    }


def write_reports(output_dir, results, requested_rounds, seed):
    csv_path = output_dir / "trials.csv"
    with csv_path.open("w", encoding="utf-8", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDS)
        writer.writeheader()
        writer.writerows(results)

    report = {
        "summary": make_summary(results, requested_rounds, seed),
        "trials": results,
    }
    json_path = output_dir / "summary.json"
    with json_path.open("w", encoding="utf-8") as json_file:
        json.dump(report, json_file, ensure_ascii=False, indent=2)
        json_file.write("\n")
    return csv_path, json_path, report["summary"]


def print_summary(summary, csv_path, json_path):
    rate = summary["success_rate"]
    rate_text = "n/a" if rate is None else "{:.1%}".format(rate)
    recognition_rate = summary["recognition_accuracy"]
    recognition_rate_text = (
        "n/a" if recognition_rate is None else "{:.1%}".format(recognition_rate)
    )
    print("\n===== grasp trial summary =====")
    print(
        "completed={}/{} successes={} failures={} success_rate={}".format(
            summary["completed_rounds"],
            summary["requested_rounds"],
            summary["successes"],
            summary["failures"],
            rate_text,
        )
    )
    print(
        "correct_recognitions={}/{} recognition_accuracy={}".format(
            summary["correct_recognitions"],
            summary["completed_rounds"],
            recognition_rate_text,
        )
    )
    for target_class in TASK_CLASSES:
        class_summary = summary["by_class"][target_class]
        class_rate = class_summary["success_rate"]
        class_rate_text = "n/a" if class_rate is None else "{:.1%}".format(class_rate)
        class_recognition_rate = class_summary["recognition_accuracy"]
        class_recognition_rate_text = (
            "n/a"
            if class_recognition_rate is None
            else "{:.1%}".format(class_recognition_rate)
        )
        print(
            "  {}: grasp={}/{} ({}) recognition={}/{} ({})".format(
                target_class,
                class_summary["successes"],
                class_summary["trials"],
                class_rate_text,
                class_summary["correct_recognitions"],
                class_summary["trials"],
                class_recognition_rate_text,
            )
        )
    print("infrastructure_errors={}".format(summary["infrastructure_errors"]))
    print("csv={}".format(csv_path))
    print("json={}".format(json_path))


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    results = []
    output_dir = None
    seed = args.seed if args.seed is not None else secrets.randbits(64)

    try:
        validate_args(args)
        if master_is_running():
            raise AutomationError(
                "a ROS master is already running; stop the existing ROS/Gazebo "
                "session before starting isolated grasp trials"
            )

        timestamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        output_dir = (
            Path(args.log_dir).expanduser().resolve()
            if args.log_dir
            else WORKSPACE / "script" / "logs" / ("grasp_trials_" + timestamp)
        )
        output_dir.mkdir(parents=True, exist_ok=False)
        rng = random.Random(seed)

        print("workspace={}".format(WORKSPACE))
        print("rounds={} seed={} logs={}".format(args.rounds, seed, output_dir))

        for round_number in range(1, args.rounds + 1):
            target_class = rng.choice(TASK_CLASSES)
            log_path = output_dir / "round_{:03d}.log".format(round_number)
            print(
                "[round {}/{}] target_class={}".format(
                    round_number, args.rounds, target_class
                ),
                flush=True,
            )
            record = run_trial(args, round_number, target_class, log_path)
            results.append(record)
            csv_path, json_path, _summary = write_reports(
                output_dir, results, args.rounds, seed
            )
            print(
                "  status={} recognition={} grasp_success={} duration={:.1f}s".format(
                    record["status"],
                    record["recognition_status"],
                    record["success"],
                    record["duration_seconds"],
                ),
                flush=True,
            )
            if round_number < args.rounds and args.restart_settle:
                time.sleep(args.restart_settle)

        _csv_path, _json_path, summary = write_reports(
            output_dir, results, args.rounds, seed
        )
        print_summary(summary, _csv_path, _json_path)
        return 0
    except KeyboardInterrupt:
        print("interrupted by user", file=sys.stderr)
        if output_dir is not None:
            csv_path, json_path, summary = write_reports(
                output_dir, results, args.rounds, seed
            )
            print_summary(summary, csv_path, json_path)
        return 130
    except (AutomationError, CleanupError, OSError, subprocess.SubprocessError) as exc:
        print("run_grasp_trials: {}".format(exc), file=sys.stderr)
        if output_dir is not None:
            csv_path, json_path, summary = write_reports(
                output_dir, results, args.rounds, seed
            )
            print_summary(summary, csv_path, json_path)
        return 1


if __name__ == "__main__":
    sys.exit(main())
