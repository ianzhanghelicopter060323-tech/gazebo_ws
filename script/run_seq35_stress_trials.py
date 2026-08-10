#!/usr/bin/env python3
"""Run isolated seq34->seq35 problem-spot OCR stress trials."""

import argparse
import csv
import datetime as dt
import hashlib
import json
import math
from pathlib import Path
import random
import subprocess
import sys
import time

import yaml

from capture_pickup_dataset import (
    AutomationError,
    CleanupError,
    SETUP_FILE,
    WORKSPACE,
    arm_scan_message,
    launch_simulation,
    master_is_running,
    read_ros_run_id,
    run_owned,
    stop_owned_launch,
    wait_for_ros,
    wait_for_simulation,
)


DEFAULT_CONFIG = WORKSPACE / "src" / "car3" / "config" / "calibration_cubes.yaml"
DEFAULT_OUTPUT_ROOT = (
    WORKSPACE / "data" / "35seq_fix" / "seq35_stress_problem_spot"
)
SCENE_HELPER = WORKSPACE / "script" / "_setup_seq35_stress_scene.py"
CAPTURE_HELPER = WORKSPACE / "script" / "capture_seq35_stress_frames.py"
OCR_PYTHON = WORKSPACE / ".venv" / "ocr" / "bin" / "python"
CLASS_IDS = {"food": 0, "daily": 1, "electronics": 2}
MANIFEST_FIELDS = (
    "attempt",
    "capture_slot",
    "case_name",
    "target_class",
    "target_model",
    "requested_x",
    "requested_y",
    "requested_yaw",
    "status",
    "ocr_success",
    "classification_correct",
    "valid_observations",
    "object_translation_m",
    "object_yaw_change_rad",
    "object_moved",
    "failure_reason",
    "duration_seconds",
    "log_file",
    "summary_file",
)


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description=(
            "restart the simulation, place the configured target class near "
            "the seq35 stress pose, navigate from seq34, and save "
            "five diagnostic RGB-D OCR frames per case"
        )
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output-root", type=Path)
    parser.add_argument("--count", type=int)
    parser.add_argument("--max-attempts", type=int)
    parser.add_argument("--seed", type=int)
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--startup-timeout", type=float, default=90.0)
    parser.add_argument("--startup-settle", type=float, default=3.0)
    parser.add_argument("--navigation-timeout", type=float, default=45.0)
    parser.add_argument("--arm-settle", type=float, default=1.0)
    parser.add_argument("--restart-settle", type=float, default=3.0)
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="validate configuration and print the planned cases without ROS",
    )
    return parser.parse_args(argv)


def finite_float(value, label):
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise AutomationError("{} must be numeric".format(label)) from exc
    if not math.isfinite(result):
        raise AutomationError("{} must be finite".format(label))
    return result


def load_config(path):
    path = path.expanduser().resolve()
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise AutomationError("cannot load {}: {}".format(path, exc))
    root = data.get("seq35_stress_problem_spot", {}) if isinstance(data, dict) else {}
    if not isinstance(root, dict) or not root:
        raise AutomationError("missing seq35_stress_problem_spot configuration")

    classes = tuple(str(value) for value in root.get("classes", ()))
    if not classes or any(value not in CLASS_IDS for value in classes):
        raise AutomationError("classes must use food/daily/electronics")
    if len(classes) != len(set(classes)):
        raise AutomationError("classes must not contain duplicates")
    model_by_class = root.get("model_by_class", {})
    if not isinstance(model_by_class, dict) or set(model_by_class) != set(classes):
        raise AutomationError("model_by_class must define every configured class")
    if set(model_by_class.values()) != {"cube_0", "cube_1", "cube_2"}:
        raise AutomationError("model_by_class must map one-to-one to cube_0/1/2")
    class_selection = str(root.get("class_selection", "shuffled_balanced"))
    if class_selection not in ("shuffled_balanced", "fixed"):
        raise AutomationError(
            "class_selection must be shuffled_balanced or fixed"
        )
    target_class = str(root.get("target_class", ""))
    if class_selection == "fixed" and target_class not in classes:
        raise AutomationError(
            "fixed class_selection requires target_class from classes"
        )

    target = root.get("target", {})
    center = target.get("center", {}) if isinstance(target, dict) else {}
    jitter = target.get("jitter", {}) if isinstance(target, dict) else {}
    bounds = target.get("bounds", {}) if isinstance(target, dict) else {}
    sampling = str(target.get("sampling", "uniform_jitter"))
    if sampling not in (
        "fixed_point",
        "uniform_jitter",
        "evenly_spaced_segment",
    ):
        raise AutomationError(
            "target.sampling must be fixed_point, uniform_jitter, or "
            "evenly_spaced_segment"
        )
    target_config = {
        "sampling": sampling,
        "x": finite_float(center.get("x"), "target.center.x"),
        "y": finite_float(center.get("y"), "target.center.y"),
        "yaw": finite_float(center.get("yaw"), "target.center.yaw"),
        "z": finite_float(center.get("z", 0.02), "target.center.z"),
        "jitter_x": finite_float(jitter.get("x", 0.0), "target.jitter.x"),
        "jitter_y": finite_float(jitter.get("y", 0.0), "target.jitter.y"),
        "jitter_yaw": finite_float(
            jitter.get("yaw", 0.0), "target.jitter.yaw"
        ),
        "x_min": finite_float(bounds.get("x_min"), "target.bounds.x_min"),
        "x_max": finite_float(bounds.get("x_max"), "target.bounds.x_max"),
        "y_min": finite_float(bounds.get("y_min"), "target.bounds.y_min"),
        "y_max": finite_float(bounds.get("y_max"), "target.bounds.y_max"),
    }
    if any(
        target_config[name] < 0.0
        for name in ("jitter_x", "jitter_y", "jitter_yaw")
    ):
        raise AutomationError("target jitter must not be negative")
    if sampling == "fixed_point" and (
        target_config["jitter_x"] != 0.0
        or target_config["jitter_y"] != 0.0
        or target_config["jitter_yaw"] != 0.0
    ):
        raise AutomationError("fixed_point sampling requires zero target jitter")
    if not (
        target_config["x_min"] <= target_config["x"] <= target_config["x_max"]
        and target_config["y_min"] <= target_config["y"] <= target_config["y_max"]
    ):
        raise AutomationError("target center lies outside configured bounds")
    if (
        target_config["x"] - target_config["jitter_x"] < target_config["x_min"]
        or target_config["x"] + target_config["jitter_x"] > target_config["x_max"]
        or target_config["y"] - target_config["jitter_y"] < target_config["y_min"]
        or target_config["y"] + target_config["jitter_y"] > target_config["y_max"]
    ):
        raise AutomationError(
            "target center +/- jitter must remain inside the legal seq35 region"
        )
    if sampling == "evenly_spaced_segment":
        segment = target.get("segment")
        if not isinstance(segment, dict):
            raise AutomationError("target.segment must be configured")
        for endpoint in ("start", "end"):
            raw = segment.get(endpoint)
            if not isinstance(raw, dict):
                raise AutomationError(
                    "target.segment.{} must be a mapping".format(endpoint)
                )
            target_config["segment_{}_x".format(endpoint)] = finite_float(
                raw.get("x"), "target.segment.{}.x".format(endpoint)
            )
            target_config["segment_{}_y".format(endpoint)] = finite_float(
                raw.get("y"), "target.segment.{}.y".format(endpoint)
            )
        for endpoint in ("start", "end"):
            x = target_config["segment_{}_x".format(endpoint)]
            y = target_config["segment_{}_y".format(endpoint)]
            if not (
                target_config["x_min"] <= x <= target_config["x_max"]
                and target_config["y_min"] <= y <= target_config["y_max"]
            ):
                raise AutomationError(
                    "target.segment.{} lies outside configured bounds".format(
                        endpoint
                    )
                )
        if (
            target_config["segment_start_x"]
            == target_config["segment_end_x"]
            and target_config["segment_start_y"]
            == target_config["segment_end_y"]
        ):
            raise AutomationError("target.segment endpoints must differ")

    other_stations = root.get("other_stations")
    if not isinstance(other_stations, list) or len(other_stations) != 2:
        raise AutomationError("other_stations must contain seq36 and seq37")
    parsed_stations = []
    for raw in other_stations:
        if not isinstance(raw, dict):
            raise AutomationError("other station entries must be mappings")
        parsed_stations.append(
            {
                "number": int(raw["number"]),
                "x": finite_float(raw.get("x"), "other station x"),
                "y": finite_float(raw.get("y"), "other station y"),
                "yaw": finite_float(raw.get("yaw"), "other station yaw"),
                "z": finite_float(raw.get("z", 0.02), "other station z"),
            }
        )
    if {station["number"] for station in parsed_stations} != {36, 37}:
        raise AutomationError("other_stations must be numbered 36 and 37")

    def configured_pose(key, include_z):
        raw = root.get(key)
        if not isinstance(raw, dict):
            raise AutomationError("{} must be a mapping".format(key))
        pose = {
            "sequence": int(raw["sequence"]),
            "x": finite_float(raw.get("x"), "{}.x".format(key)),
            "y": finite_float(raw.get("y"), "{}.y".format(key)),
            "yaw": finite_float(raw.get("yaw"), "{}.yaw".format(key)),
        }
        if include_z:
            pose["z"] = finite_float(raw.get("z", 0.01), "{}.z".format(key))
        return pose

    arm = tuple(float(value) for value in root.get("arm_scan_positions", ()))
    if len(arm) != 5 or not all(math.isfinite(value) for value in arm):
        raise AutomationError("arm_scan_positions must contain five finite values")

    result = {
        "path": path,
        "output_root": Path(root.get("output_root", DEFAULT_OUTPUT_ROOT)).expanduser().resolve(),
        "count": int(root.get("count", 20)),
        "max_attempts": int(root.get("max_attempts", 50)),
        "seed": int(root.get("seed", 550035)),
        "classes": classes,
        "class_selection": class_selection,
        "target_class": target_class if class_selection == "fixed" else None,
        "model_by_class": {key: str(value) for key, value in model_by_class.items()},
        "target": target_config,
        "other_stations": tuple(parsed_stations),
        "robot_start": configured_pose("robot_start", True),
        "navigation_goal": configured_pose("navigation_goal", False),
        "arm_scan_positions": arm,
    }
    if result["count"] <= 0 or result["max_attempts"] < result["count"]:
        raise AutomationError("count must be positive and max_attempts >= count")
    if result["robot_start"]["sequence"] != 34:
        raise AutomationError("robot_start.sequence must be 34")
    if result["navigation_goal"]["sequence"] != 35:
        raise AutomationError("navigation_goal.sequence must be 35")
    return result


def balanced_class_schedule(classes, count, rng):
    quotient, remainder = divmod(count, len(classes))
    bonus_order = list(classes)
    rng.shuffle(bonus_order)
    schedule = []
    for name in classes:
        schedule.extend([name] * (quotient + (name in bonus_order[:remainder])))
    rng.shuffle(schedule)
    return schedule


def target_positions(target, count, rng):
    if target["sampling"] == "fixed_point":
        return [
            (target["x"], target["y"], target["yaw"])
            for _unused in range(count)
        ]

    if target["sampling"] == "uniform_jitter":
        return [
            (
                target["x"]
                + rng.uniform(-target["jitter_x"], target["jitter_x"]),
                target["y"]
                + rng.uniform(-target["jitter_y"], target["jitter_y"]),
                target["yaw"]
                + rng.uniform(-target["jitter_yaw"], target["jitter_yaw"]),
            )
            for _unused in range(count)
        ]

    if count == 1:
        fractions = [0.5]
    else:
        fractions = [index / float(count - 1) for index in range(count)]
    positions = [
        (
            target["segment_start_x"]
            + fraction
            * (target["segment_end_x"] - target["segment_start_x"]),
            target["segment_start_y"]
            + fraction
            * (target["segment_end_y"] - target["segment_start_y"]),
            target["yaw"],
        )
        for fraction in fractions
    ]
    rng.shuffle(positions)
    return positions


def build_case_plans(config, count, seed):
    rng = random.Random(seed)
    if config["class_selection"] == "fixed":
        schedule = [config["target_class"]] * count
    else:
        schedule = balanced_class_schedule(config["classes"], count, rng)
    plans = []
    all_models = tuple(sorted(config["model_by_class"].values()))
    target = config["target"]
    positions = target_positions(target, count, rng)
    for slot, (target_class, (x, y, yaw)) in enumerate(
        zip(schedule, positions), 1
    ):
        target_model = config["model_by_class"][target_class]
        remaining_models = [name for name in all_models if name != target_model]
        rng.shuffle(remaining_models)
        models = {
            target_model: {
                "x": x,
                "y": y,
                "yaw": yaw,
                "z": target["z"],
                "station": 35,
            }
        }
        for model, station in zip(remaining_models, config["other_stations"]):
            models[model] = dict(station)
        plans.append(
            {
                "capture_slot": slot,
                "seed": seed,
                "target_sampling": target["sampling"],
                "target_class": target_class,
                "target_class_id": CLASS_IDS[target_class],
                "target_model": target_model,
                "models": models,
                "robot_start": dict(config["robot_start"]),
                "navigation_goal": dict(config["navigation_goal"]),
                "arm_scan_positions": list(config["arm_scan_positions"]),
            }
        )
    return plans


def git_state():
    def command(arguments):
        result = subprocess.run(
            arguments,
            cwd=str(WORKSPACE),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            check=False,
        )
        return result.stdout.strip() if result.returncode == 0 else "unavailable"

    return {
        "commit": command(["git", "rev-parse", "HEAD"]),
        "status": command(["git", "status", "--short"]),
    }


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def append_section(log_file, title, output):
    log_file.write("\n===== {} =====\n".format(title))
    log_file.write(output)
    if output and not output.endswith("\n"):
        log_file.write("\n")
    log_file.flush()


def wait_for_pre_capture_health(timeout):
    """Check only capture dependencies, with a fresh timeout per probe.

    The generic dataset health check walks the complete simulation readiness
    list under one shared deadline and finishes by probing the mission server.
    This stress client does not use that server after navigation.  On a live
    but moderately loaded simulation, the shared 15-second budget therefore
    produced false infrastructure failures after a successful seq35 arrival.
    """
    checks = (
        (
            "RGB camera frame",
            ["rostopic", "echo", "-n", "1", "/camera/rgb/image_raw", "--noarr"],
            None,
        ),
        (
            "AMCL pose",
            ["rostopic", "echo", "-n", "1", "/amcl_pose", "--noarr"],
            None,
        ),
        (
            "running arm controller",
            ["rosservice", "call", "/controller_manager/list_controllers", "{}"],
            "arm_controller",
        ),
    )
    for label, command, required_text in checks:
        wait_for_ros(
            label,
            command,
            time.monotonic() + timeout,
            required_text=required_text,
        )


def parse_marker(output, marker):
    matches = [line[len(marker) :] for line in output.splitlines() if line.startswith(marker)]
    if not matches:
        raise AutomationError("command output did not contain {}".format(marker))
    return matches[-1]


def target_motion(scene_payload, capture_summary, model_name):
    try:
        initial = scene_payload["measured"][model_name]
        pose = capture_summary["runtime_state"][model_name]["pose"]
        final = pose["position"]
        orientation = pose["orientation"]
        final_yaw = math.atan2(
            2.0
            * (
                orientation["w"] * orientation["z"]
                + orientation["x"] * orientation["y"]
            ),
            1.0
            - 2.0
            * (
                orientation["y"] * orientation["y"]
                + orientation["z"] * orientation["z"]
            ),
        )
        yaw_change = math.atan2(
            math.sin(final_yaw - initial["yaw"]),
            math.cos(final_yaw - initial["yaw"]),
        )
        translation = math.hypot(
            final["x"] - initial["x"], final["y"] - initial["y"]
        )
    except (KeyError, TypeError, ValueError):
        return None
    return {
        "initial_pose": dict(initial),
        "capture_pose": {
            "x": float(final["x"]),
            "y": float(final["y"]),
            "z": float(final["z"]),
            "yaw": float(final_yaw),
        },
        "translation_xy_m": float(translation),
        "yaw_change_rad": float(yaw_change),
        # Gazebo's resting numerical noise is far below these thresholds.
        # This flags physical motion without labeling its cause as collision.
        "movement_detected": bool(
            translation > 0.002 or abs(yaw_change) > math.radians(2.0)
        ),
    }


def execute_case(args, config, plan, attempt, run_name, run_log_dir):
    case_name = "case_{:03d}".format(plan["capture_slot"])
    retry_number = int(plan.get("retry_number", 1))
    if retry_number > 1:
        case_name += "_retry_{:02d}".format(retry_number)
    case_log_dir = run_log_dir / "attempt_{:03d}".format(attempt)
    case_log_dir.mkdir(parents=True, exist_ok=False)
    case_file = case_log_dir / "case_plan.json"
    case_file.write_text(
        json.dumps(plan, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    log_path = case_log_dir / "roslaunch.log"
    launch_process = None
    launch_run_id = None
    started = time.monotonic()
    summary_path = None
    summary = None
    status = "infrastructure_error"
    failure_reason = ""

    with log_path.open("w", encoding="utf-8") as log_file:
        try:
            if master_is_running():
                raise CleanupError(
                    "ROS master is already running; refusing to reuse another simulation"
                )
            launch_process = launch_simulation(
                args.gui,
                log_file,
                start_perception=False,
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
            if launch_process.poll() is not None:
                raise AutomationError("roslaunch exited during startup")
            time.sleep(args.startup_settle)

            return_code, output = run_owned(
                [
                    "python3",
                    str(SCENE_HELPER),
                    "--case-file",
                    str(case_file),
                ],
                timeout=30.0,
            )
            append_section(log_file, "stress scene setup", output)
            if return_code != 0:
                raise AutomationError("failed to set seq35 stress scene")
            scene_payload = json.loads(parse_marker(output, "SEQ35_STRESS_SCENE="))
            (case_log_dir / "scene_measured.json").write_text(
                json.dumps(scene_payload, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )

            goal = config["navigation_goal"]
            return_code, output = run_owned(
                [
                    "python3",
                    str(WORKSPACE / "script" / "_send_observation_goal.py"),
                    "--x",
                    str(goal["x"]),
                    "--y",
                    str(goal["y"]),
                    "--yaw",
                    str(goal["yaw"]),
                    "--timeout",
                    str(args.navigation_timeout),
                ],
                timeout=args.navigation_timeout + 15.0,
            )
            append_section(log_file, "seq34 to seq35 navigation", output)
            if return_code != 0:
                status = "navigation_failed"
                raise AutomationError("seq34 to seq35 navigation failed")

            return_code, output = run_owned(
                [
                    "rostopic",
                    "pub",
                    "-1",
                    "/arm_controller/command",
                    "trajectory_msgs/JointTrajectory",
                    arm_scan_message(config["arm_scan_positions"]),
                ],
                timeout=10.0,
            )
            append_section(log_file, "arm scan pose", output)
            if return_code != 0:
                raise AutomationError("failed to command arm scan pose")
            time.sleep(args.arm_settle)
            wait_for_pre_capture_health(min(15.0, args.startup_timeout))

            return_code, output = run_owned(
                [
                    str(OCR_PYTHON),
                    str(CAPTURE_HELPER),
                    "--config",
                    str(config["path"]),
                    "--output-root",
                    str(config["output_root"]),
                    "--run-name",
                    run_name,
                    "--case-name",
                    case_name,
                    "--expected-class",
                    plan["target_class"],
                    "--target-model",
                    plan["target_model"],
                ],
                timeout=120.0,
            )
            append_section(log_file, "five-frame OCR capture", output)
            if return_code != 0:
                raise AutomationError("five-frame OCR capture failed")
            summary_path = Path(
                parse_marker(output, "SEQ35_STRESS_SUMMARY=")
            ).expanduser().resolve()
            summary = json.loads(summary_path.read_text(encoding="utf-8"))
            motion = target_motion(scene_payload, summary, plan["target_model"])
            if motion is not None:
                summary["target_motion"] = motion
                summary_path.write_text(
                    json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
                    encoding="utf-8",
                )
            status = "captured"
            return (
                case_name,
                status,
                summary_path,
                summary,
                "",
                time.monotonic() - started,
                log_path,
            )
        except (AutomationError, json.JSONDecodeError, OSError) as exc:
            failure_reason = str(exc)
            append_section(log_file, "attempt failure", failure_reason)
            return (
                case_name,
                status,
                summary_path,
                summary,
                failure_reason,
                time.monotonic() - started,
                log_path,
            )
        finally:
            stop_owned_launch(launch_process, launch_run_id)


def manifest_record(
    attempt,
    plan,
    case_name,
    status,
    summary_path,
    summary,
    attempt_error,
    duration,
    log_path,
):
    target_pose = plan["models"][plan["target_model"]]
    motion = None if summary is None else summary.get("target_motion")
    return {
        "attempt": attempt,
        "capture_slot": plan["capture_slot"],
        "case_name": case_name,
        "target_class": plan["target_class"],
        "target_model": plan["target_model"],
        "requested_x": round(target_pose["x"], 9),
        "requested_y": round(target_pose["y"], 9),
        "requested_yaw": round(target_pose["yaw"], 9),
        "status": status,
        "ocr_success": "" if summary is None else bool(summary.get("success")),
        "classification_correct": (
            "" if summary is None else bool(summary.get("classification_correct"))
        ),
        "valid_observations": (
            "" if summary is None else int(summary.get("valid_observations", 0))
        ),
        "object_translation_m": (
            "" if motion is None else round(motion["translation_xy_m"], 9)
        ),
        "object_yaw_change_rad": (
            "" if motion is None else round(motion["yaw_change_rad"], 9)
        ),
        "object_moved": (
            "" if motion is None else bool(motion["movement_detected"])
        ),
        "failure_reason": (
            attempt_error if summary is None else summary.get("failure_reason", "")
        ),
        "duration_seconds": round(duration, 3),
        "log_file": str(log_path),
        "summary_file": "" if summary_path is None else str(summary_path),
    }


def write_manifest(path, records):
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=MANIFEST_FIELDS)
        writer.writeheader()
        writer.writerows(records)


def write_summary(path, args, config, run_name, records):
    captured = [record for record in records if record["status"] == "captured"]
    ocr_successes = sum(record["ocr_success"] is True for record in captured)
    correct = sum(record["classification_correct"] is True for record in captured)
    moved = sum(record["object_moved"] is True for record in captured)
    payload = {
        "run_name": run_name,
        "requested_captures": args.count,
        "completed_captures": len(captured),
        "attempts": len(records),
        "ocr_successes": ocr_successes,
        "ocr_success_rate": (
            round(ocr_successes / len(captured), 4) if captured else None
        ),
        "correct_classifications": correct,
        "classification_accuracy": (
            round(correct / len(captured), 4) if captured else None
        ),
        "object_motion_cases": moved,
        "seed": args.seed,
        "target_sampling": config["target"]["sampling"],
        "config": str(config["path"]),
        "config_sha256": sha256(config["path"]),
        "records": records,
    }
    path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    return payload


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    try:
        config = load_config(args.config)
        args.count = config["count"] if args.count is None else args.count
        args.max_attempts = (
            config["max_attempts"] if args.max_attempts is None else args.max_attempts
        )
        args.seed = config["seed"] if args.seed is None else args.seed
        if args.output_root is not None:
            config["output_root"] = args.output_root.expanduser().resolve()
        if args.count <= 0 or args.max_attempts < args.count:
            raise AutomationError("count must be positive and max-attempts >= count")
        if any(
            not math.isfinite(value) or value < 0.0
            for value in (
                args.startup_timeout,
                args.startup_settle,
                args.navigation_timeout,
                args.arm_settle,
                args.restart_settle,
            )
        ):
            raise AutomationError("timeouts and settle durations must be non-negative")
        if args.startup_timeout <= 0.0 or args.navigation_timeout <= 0.0:
            raise AutomationError("startup and navigation timeouts must be positive")
        for required in (SETUP_FILE, SCENE_HELPER, CAPTURE_HELPER, OCR_PYTHON):
            if not required.exists():
                raise AutomationError("missing required path: {}".format(required))

        plans = build_case_plans(config, args.count, args.seed)
        if args.dry_run:
            print(json.dumps(plans, ensure_ascii=False, indent=2))
            return 0
        if master_is_running():
            raise AutomationError(
                "a ROS master is already running; stop it before isolated stress trials"
            )

        timestamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        run_name = "run_{}_seed{}".format(timestamp, args.seed)
        output_root = config["output_root"]
        for dataset in ("rgb_dataset", "depth_dataset", "ocr_json", "ocr_retan"):
            (output_root / dataset).mkdir(parents=True, exist_ok=True)
        run_log_dir = output_root / "logs" / run_name
        run_log_dir.mkdir(parents=True, exist_ok=False)
        run_metadata = {
            "run_name": run_name,
            "created_at": dt.datetime.now().astimezone().isoformat(timespec="seconds"),
            "count": args.count,
            "max_attempts": args.max_attempts,
            "seed": args.seed,
            "target_sampling": config["target"],
            "config": str(config["path"]),
            "config_sha256": sha256(config["path"]),
            "git": git_state(),
            "plans": plans,
        }
        (run_log_dir / "run_config.json").write_text(
            json.dumps(run_metadata, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        manifest_path = run_log_dir / "manifest.csv"
        summary_path = output_root / "ocr_json" / run_name / "summary.json"
        summary_path.parent.mkdir(parents=True, exist_ok=False)

        records = []
        completed = 0
        attempt = 0
        retry_counts = {}
        while completed < args.count:
            attempt += 1
            if attempt > args.max_attempts:
                raise AutomationError(
                    "reached max-attempts={} after {}/{} captures".format(
                        args.max_attempts, completed, args.count
                    )
                )
            plan = dict(plans[completed])
            retry_counts[completed] = retry_counts.get(completed, 0) + 1
            plan["retry_number"] = retry_counts[completed]
            print(
                "[attempt {}/{}] capture {}/{} class={} position=({:.4f}, {:.4f})".format(
                    attempt,
                    args.max_attempts,
                    completed + 1,
                    args.count,
                    plan["target_class"],
                    plan["models"][plan["target_model"]]["x"],
                    plan["models"][plan["target_model"]]["y"],
                ),
                flush=True,
            )
            try:
                result = execute_case(
                    args, config, plan, attempt, run_name, run_log_dir
                )
            except CleanupError:
                raise
            (
                case_name,
                status,
                case_summary_path,
                case_summary,
                attempt_error,
                duration,
                log_path,
            ) = result
            record = manifest_record(
                attempt,
                plan,
                case_name,
                status,
                case_summary_path,
                case_summary,
                attempt_error,
                duration,
                log_path,
            )
            records.append(record)
            write_manifest(manifest_path, records)
            write_summary(summary_path, args, config, run_name, records)
            if status == "captured":
                completed += 1
                print(
                    "  captured {}: OCR success={} correct={} valid={}".format(
                        case_name,
                        record["ocr_success"],
                        record["classification_correct"],
                        record["valid_observations"],
                    )
                )
            else:
                print("  attempt did not produce a capture; see {}".format(log_path))
            if completed < args.count:
                time.sleep(args.restart_settle)

        payload = write_summary(summary_path, args, config, run_name, records)
        print(
            "completed {}/{} captures; OCR success={}/{}; summary={}".format(
                payload["completed_captures"],
                payload["requested_captures"],
                payload["ocr_successes"],
                payload["completed_captures"],
                summary_path,
            )
        )
        return 0
    except KeyboardInterrupt:
        print("interrupted by user", file=sys.stderr)
        return 130
    except (AutomationError, OSError, ValueError, yaml.YAMLError) as exc:
        print("run_seq35_stress_trials: {}".format(exc), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
