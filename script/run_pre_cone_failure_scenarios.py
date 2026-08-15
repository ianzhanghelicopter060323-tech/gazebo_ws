#!/usr/bin/env python3
"""Run bounded fixed-scene regressions for the five failed pre-cone rounds."""

import argparse
import datetime as dt
import json
from pathlib import Path
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
from run_pre_cone_e2e_trials import prepare_pre_cone_mission


DEFAULT_CASES = (
    WORKSPACE / "script" / "config" /
    "pre_cone_failure_scenarios_20260816.json"
)
SETUP_HELPER = WORKSPACE / "script" / "_setup_pre_cone_failure_scene.py"


def parse_args(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument("--cases", type=Path, default=DEFAULT_CASES)
    parser.add_argument("--case-id", action="append", default=[])
    parser.add_argument("--max-attempts", type=int, default=3)
    parser.add_argument("--startup-timeout", type=float, default=90.0)
    parser.add_argument("--task-timeout", type=float, default=720.0)
    parser.add_argument("--progress-timeout", type=float, default=120.0)
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--output-dir", type=Path)
    return parser.parse_args(argv)


def load_cases(path, selected):
    payload = json.loads(path.read_text(encoding="utf-8"))
    cases = payload.get("cases", [])
    by_id = {case["id"]: case for case in cases}
    unknown = set(selected) - set(by_id)
    if unknown:
        raise AutomationError("unknown case ids: {}".format(sorted(unknown)))
    return [by_id[name] for name in selected] if selected else cases


def run_attempt(args, case, attempt, output_dir, mission_config):
    attempt_dir = output_dir / case["id"] / "attempt_{:02d}".format(attempt)
    attempt_dir.mkdir(parents=True, exist_ok=False)
    case_file = attempt_dir / "case.json"
    case_file.write_text(json.dumps(case, indent=2) + "\n", encoding="utf-8")
    launch = None
    run_id = None
    started = time.monotonic()
    record = {
        "case_id": case["id"],
        "source_round": case["source_round"],
        "attempt": attempt,
        "target_class": case["target_class"],
        "success": False,
        "duration_seconds": None,
        "result": None,
    }
    with (attempt_dir / "roslaunch.log").open("w", encoding="utf-8") as log:
        try:
            if master_is_running():
                raise CleanupError("ROS master is already running")
            launch = launch_simulation(
                args.gui,
                log,
                mission_config=mission_config,
                start_perception=True,
            )
            deadline = time.monotonic() + args.startup_timeout
            wait_for_ros("ROS master", ["rosnode", "list"], deadline)
            run_id = read_ros_run_id()
            wait_for_simulation(args.startup_timeout)
            wait_for_ros(
                "mission action",
                ["rostopic", "echo", "-n", "1", "/sim_task/state", "--noarr"],
                deadline,
            )
            time.sleep(5.0)
            setup_code, setup_output = run_owned(
                ["python3", str(SETUP_HELPER), "--case-file", str(case_file)],
                timeout=45.0,
            )
            (attempt_dir / "scene_setup.log").write_text(
                setup_output, encoding="utf-8"
            )
            if setup_code != 0:
                raise AutomationError("fixed scene setup failed")
            task_id = "{}_a{}_{}".format(case["id"], attempt, int(time.time()))
            code, output = run_owned(
                [
                    "rosrun", "smart_factory_tests", "send_navigation_task.py",
                    "--task-id", task_id,
                    "--target-class", case["target_class"],
                    "_progress_timeout:={}".format(args.progress_timeout),
                ],
                timeout=args.task_timeout,
            )
            (attempt_dir / "task.log").write_text(output, encoding="utf-8")
            parsed = parse_task_result(output)
            record["result"] = parsed
            record["success"] = bool(
                code == 0
                and parsed is not None
                and parsed["reported_success"]
                and parsed["completed_stage"] == 14
                and parsed["error_code"] == 0
            )
        except (AutomationError, CleanupError, OSError, ValueError) as exc:
            record["error"] = str(exc)
        finally:
            stop_owned_launch(launch, run_id)
            record["duration_seconds"] = round(time.monotonic() - started, 3)
            (attempt_dir / "result.json").write_text(
                json.dumps(record, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
    return record


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    if not 1 <= args.max_attempts <= 3:
        raise AutomationError("--max-attempts must be between 1 and 3")
    cases = load_cases(args.cases.expanduser().resolve(), args.case_id)
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = (
        args.output_dir.expanduser().resolve()
        if args.output_dir
        else WORKSPACE / "script" / "logs" /
        ("pre_cone_failure_regression_" + stamp)
    )
    output_dir.mkdir(parents=True, exist_ok=False)
    mission_config = prepare_pre_cone_mission(output_dir)
    results = []
    for case in cases:
        for attempt in range(1, args.max_attempts + 1):
            print("running {} attempt {}/{}".format(case["id"], attempt, args.max_attempts), flush=True)
            record = run_attempt(
                args, case, attempt, output_dir, mission_config
            )
            results.append(record)
            print(json.dumps(record, ensure_ascii=False), flush=True)
            if record["success"]:
                break
    summary = {
        "output_dir": str(output_dir),
        "cases": len(cases),
        "passed": sum(
            any(r["case_id"] == case["id"] and r["success"] for r in results)
            for case in cases
        ),
        "results": results,
    }
    (output_dir / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print("summary=" + json.dumps(summary, ensure_ascii=False), flush=True)
    return 0 if summary["passed"] == len(cases) else 1


if __name__ == "__main__":
    raise SystemExit(main())
