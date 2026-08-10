#!/usr/bin/env python3
"""Run two full-pipeline seq35 supplemental-observation validations.

The normal cube and cone spawners are left untouched.  After each fresh
simulation starts, the existing isolated scene helper moves the food cube to
one endpoint of the requested seq35 boundary, then the ordinary Mission action
runs navigation, observation, alignment, and grasping.
"""

import argparse
import datetime as dt
import json
from pathlib import Path
import re
import sys
import time

from capture_pickup_dataset import (
    AutomationError,
    CleanupError,
    WORKSPACE,
    launch_simulation,
    master_is_running,
    read_ros_run_id,
    run_owned,
    stop_owned_launch,
    wait_for_ros,
    wait_for_simulation,
)
from run_grasp_trials import parse_task_result


SCENE_HELPER = WORKSPACE / "script" / "_setup_seq35_stress_scene.py"
DEFAULT_OUTPUT_ROOT = (
    WORKSPACE
    / "data"
    / "35seq_fix"
    / "seq35_stress_problem_spot"
    / "supplemental_validation"
)
CASES = (
    ("lower_endpoint", -0.77, -0.69),
    ("upper_endpoint", -0.77, -0.36),
)
PRIMARY_PATTERN = re.compile(
    r"task=.* stage=LOCALIZE_TARGET .*: "
    r"seq35 primary RGB-D observation attempt"
)
SUPPLEMENTAL_PATTERN = re.compile(
    r"task=.* stage=OBSERVE_PICKUP_CANDIDATE .*: "
    r"moving arm to seq35 supplemental camera observation pose"
)


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description=(
            "run the complete food pickup pipeline at the two x=-0.77 "
            "seq35 boundary endpoints"
        )
    )
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--startup-timeout", type=float, default=90.0)
    parser.add_argument("--startup-settle", type=float, default=8.0)
    parser.add_argument("--task-timeout", type=float, default=420.0)
    parser.add_argument("--restart-settle", type=float, default=3.0)
    return parser.parse_args(argv)


def case_plan(name, x, y):
    return {
        "name": name,
        "models": {
            "cube_0": {
                "x": x,
                "y": y,
                "z": 0.02,
                "yaw": -1.5708,
                "station": 35,
                "class": "food",
            },
            "cube_1": {
                "x": -1.395,
                "y": 0.08,
                "z": 0.02,
                "yaw": 0.0,
                "station": 36,
                "class": "daily",
            },
            "cube_2": {
                "x": -2.01,
                "y": -0.445,
                "z": 0.02,
                "yaw": 1.5708,
                "station": 37,
                "class": "electronics",
            },
        },
        "robot_start": {"x": 0.0, "y": 0.0, "z": 0.01, "yaw": 0.0},
    }


def validate_args(args):
    for name in ("startup_timeout", "task_timeout"):
        if getattr(args, name) <= 0.0:
            raise AutomationError("--{} must be positive".format(name.replace("_", "-")))
    for name in ("startup_settle", "restart_settle"):
        if getattr(args, name) < 0.0:
            raise AutomationError(
                "--{} must be non-negative".format(name.replace("_", "-"))
            )
    if not SCENE_HELPER.is_file():
        raise AutomationError("missing scene helper {}".format(SCENE_HELPER))


def analyze_log(log_path):
    text = log_path.read_text(encoding="utf-8", errors="replace")
    return {
        "primary_observation_attempts": len(PRIMARY_PATTERN.findall(text)),
        "supplemental_pose_used": bool(SUPPLEMENTAL_PATTERN.search(text)),
        "supplemental_log_matches": len(SUPPLEMENTAL_PATTERN.findall(text)),
    }


def run_case(args, output_dir, index, name, x, y):
    case_dir = output_dir / "round_{:02d}_{}".format(index, name)
    case_dir.mkdir(parents=True, exist_ok=False)
    plan = case_plan(name, x, y)
    case_file = case_dir / "case_plan.json"
    case_file.write_text(
        json.dumps(plan, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    log_path = case_dir / "full_pipeline.log"
    launch_process = None
    launch_run_id = None
    started = time.monotonic()
    result = {
        "round": index,
        "name": name,
        "food_pose": {"x": x, "y": y, "z": 0.02, "yaw": -1.5708},
        "case_file": str(case_file),
        "log_file": str(log_path),
        "status": "not_started",
        "task_result": None,
    }

    with log_path.open("w", encoding="utf-8") as log_file:
        try:
            if master_is_running():
                raise CleanupError(
                    "a ROS master is already running before validation round {}".format(
                        index
                    )
                )
            launch_process = launch_simulation(args.gui, log_file)
            deadline = time.monotonic() + args.startup_timeout
            wait_for_ros("ROS master", ["rosnode", "list"], deadline)
            launch_run_id = read_ros_run_id()
            wait_for_simulation(args.startup_timeout)
            wait_for_ros(
                "mission node",
                ["rosnode", "list"],
                deadline,
                required_text="/smart_factory_mission",
            )
            wait_for_ros(
                "cube locator",
                ["rosnode", "list"],
                deadline,
                required_text="/cube_locator",
            )
            if launch_process.poll() is not None:
                raise AutomationError("roslaunch exited during startup")
            if args.startup_settle:
                time.sleep(args.startup_settle)

            setup_code, setup_output = run_owned(
                ["python3", str(SCENE_HELPER), "--case-file", str(case_file)],
                timeout=40.0,
            )
            log_file.write("\n===== isolated scene setup =====\n")
            log_file.write(setup_output)
            if setup_output and not setup_output.endswith("\n"):
                log_file.write("\n")
            log_file.flush()
            if setup_code != 0:
                raise AutomationError("isolated scene setup failed")

            task_code, task_output = run_owned(
                [
                    "rosrun",
                    "smart_factory_tests",
                    "send_navigation_task.py",
                    "--task-id",
                    "seq35_supplemental_validation_{:02d}".format(index),
                    "--target-class",
                    "food",
                ],
                timeout=args.task_timeout,
            )
            log_file.write("\n===== task client =====\n")
            log_file.write(task_output)
            if task_output and not task_output.endswith("\n"):
                log_file.write("\n")
            log_file.flush()
            result["task_result"] = parse_task_result(task_output)
            result["task_client_exit_code"] = task_code
            result["status"] = (
                "task_succeeded"
                if task_code == 0
                and result["task_result"] is not None
                and result["task_result"]["reported_success"]
                else "task_failed"
            )
        except CleanupError:
            raise
        except (AutomationError, OSError, ValueError) as exc:
            result["status"] = "validation_error"
            result["error"] = str(exc)
            log_file.write("\n===== validation error =====\n{}\n".format(exc))
            log_file.flush()
        finally:
            stop_owned_launch(launch_process, launch_run_id)

    result.update(analyze_log(log_path))
    result["duration_seconds"] = round(time.monotonic() - started, 3)
    (case_dir / "result.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    return result


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    try:
        validate_args(args)
        timestamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        output_dir = args.output_root.expanduser().resolve() / (
            "run_{}".format(timestamp)
        )
        output_dir.mkdir(parents=True, exist_ok=False)
        results = []
        for index, (name, x, y) in enumerate(CASES, 1):
            print(
                "[round {}/2] food at ({:.2f}, {:.2f})".format(index, x, y),
                flush=True,
            )
            result = run_case(args, output_dir, index, name, x, y)
            results.append(result)
            print(
                "  status={} primary_attempts={} supplemental_used={}".format(
                    result["status"],
                    result["primary_observation_attempts"],
                    result["supplemental_pose_used"],
                ),
                flush=True,
            )
            if index < len(CASES) and args.restart_settle:
                time.sleep(args.restart_settle)
        summary = {
            "created_at": dt.datetime.now().astimezone().isoformat(timespec="seconds"),
            "normal_spawn_files_modified": False,
            "results": results,
        }
        summary_path = output_dir / "summary.json"
        summary_path.write_text(
            json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        print("summary={}".format(summary_path))
        return 0 if all(r["status"] == "task_succeeded" for r in results) else 1
    except KeyboardInterrupt:
        print("interrupted by user", file=sys.stderr)
        return 130
    except (AutomationError, CleanupError, OSError, ValueError) as exc:
        print("run_seq35_supplemental_validation: {}".format(exc), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
