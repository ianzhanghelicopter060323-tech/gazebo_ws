#!/usr/bin/env python3
"""Launch one simulation and navigate once to the formal seq35 pose."""

import argparse
import datetime as dt
import json
from pathlib import Path
import sys
import time

import yaml

from capture_pickup_dataset import (
    AutomationError,
    CleanupError,
    DEFAULT_MISSION,
    SETUP_FILE,
    WORKSPACE,
    launch_simulation,
    master_is_running,
    prepare_navigation_only_mission,
    read_ros_run_id,
    run_owned,
    stop_owned_launch,
    wait_for_ros,
    wait_for_simulation,
)


DEFAULT_LOG_ROOT = WORKSPACE / "script" / "logs"
CALIBRATION_MARKERS = (
    "fov_close_1",
    "fov_close_2",
    "fov_mid_1",
    "fov_mid_2",
    "fov_far_1",
    "fov_far_2",
)


def append_section(stream, title, content):
    stream.write("\n===== {} =====\n".format(title))
    stream.write(str(content))
    if not str(content).endswith("\n"):
        stream.write("\n")
    stream.flush()


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description=(
            "start one Gazebo/RViz session, navigate exactly once through the "
            "configured route to the formal seq35 observation pose, and stop "
            "before recognition or grasping"
        )
    )
    display = parser.add_mutually_exclusive_group()
    display.add_argument("--gui", dest="gui", action="store_true")
    display.add_argument("--headless", dest="gui", action="store_false")
    parser.set_defaults(gui=True)
    parser.add_argument(
        "--target-class",
        choices=("food", "daily", "electronics"),
        default="food",
        help="task field only; navigation stops before target selection",
    )
    parser.add_argument("--startup-timeout", type=float, default=90.0)
    parser.add_argument("--startup-settle", type=float, default=8.0)
    parser.add_argument("--navigation-timeout", type=float, default=300.0)
    parser.add_argument(
        "--exit-after-arrival",
        action="store_true",
        help="close Gazebo immediately after seq35 instead of waiting for Ctrl+C",
    )
    parser.add_argument(
        "--log-dir",
        type=Path,
        help="output directory; default: script/logs/seq35_observation_once_<time>",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="validate and print the one-shot configuration without starting ROS",
    )
    return parser.parse_args(argv)


def validate_args(args):
    for name in ("startup_timeout", "navigation_timeout"):
        if getattr(args, name) <= 0.0:
            raise AutomationError(
                "--{} must be positive".format(name.replace("_", "-"))
            )
    if args.startup_settle < 0.0:
        raise AutomationError("--startup-settle must be non-negative")
    for path in (SETUP_FILE, DEFAULT_MISSION):
        if not path.is_file():
            raise AutomationError("missing required file {}".format(path))


def read_seq35_pose(mission_path=DEFAULT_MISSION):
    try:
        payload = yaml.safe_load(mission_path.read_text(encoding="utf-8"))
        stations = payload["pickup"]["stations"]
        station = next(item for item in stations if int(item["number"]) == 35)
        pose = {key: float(station[key]) for key in ("x", "y", "yaw")}
    except (OSError, KeyError, StopIteration, TypeError, ValueError, yaml.YAMLError) as exc:
        raise AutomationError("cannot load formal seq35 pose: {}".format(exc))
    return pose


def require_calibration_markers(timeout):
    return_code, output = run_owned(
        ["rosservice", "call", "/gazebo/get_world_properties"],
        timeout=timeout,
    )
    missing = [name for name in CALIBRATION_MARKERS if name not in output]
    if return_code != 0 or missing:
        raise AutomationError(
            "orange calibration cubes are unavailable: {}".format(
                ", ".join(missing) if missing else "world query failed"
            )
        )


def run_once(args, output_dir, seq35_pose):
    if master_is_running():
        raise CleanupError(
            "a ROS master is already running; stop the existing ROS/Gazebo "
            "session before starting the one-shot seq35 run"
        )

    output_dir.mkdir(parents=True, exist_ok=False)
    mission_config = prepare_navigation_only_mission(output_dir)
    log_path = output_dir / "roslaunch.log"
    launch_process = None
    launch_run_id = None
    arrived = False

    with log_path.open("w", encoding="utf-8") as log_file:
        try:
            launch_process = launch_simulation(
                args.gui,
                log_file,
                mission_config=mission_config,
                start_perception=True,
                spawn_calibration_cubes=True,
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
            if args.startup_settle:
                time.sleep(args.startup_settle)
            require_calibration_markers(min(15.0, args.startup_timeout))

            task_id = "seq35_observation_{}".format(int(time.time()))
            print(
                "navigating once to seq35 x={x:.3f} y={y:.3f} "
                "yaw={yaw:.6f}".format(**seq35_pose),
                flush=True,
            )
            return_code, output = run_owned(
                [
                    "rosrun",
                    "smart_factory_tests",
                    "send_navigation_task.py",
                    "--task-id",
                    task_id,
                    "--target-class",
                    args.target_class,
                ],
                timeout=args.navigation_timeout,
            )
            append_section(log_file, "seq35 navigation client", output)
            if return_code != 0:
                raise AutomationError(
                    "seq35 navigation failed with exit code {}; see {}".format(
                        return_code, log_path
                    )
                )
            arrived = True
            print(
                "seq35 reached with the formal position-and-yaw constraint; "
                "orange calibration cubes are present",
                flush=True,
            )
            if not args.exit_after_arrival:
                print(
                    "Gazebo/RViz will remain open for observation-pose "
                    "calibration; press Ctrl+C to close this one run.",
                    flush=True,
                )
                try:
                    while launch_process.poll() is None:
                        time.sleep(0.5)
                except KeyboardInterrupt:
                    print("closing the seq35 calibration session", flush=True)
                else:
                    raise AutomationError(
                        "roslaunch exited while the seq35 session was being held"
                    )
        finally:
            stop_owned_launch(launch_process, launch_run_id)

    return arrived


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    try:
        validate_args(args)
        seq35_pose = read_seq35_pose()
        if args.dry_run:
            print("runs=1")
            print("gui={}".format(args.gui))
            print("pipeline_stop_after=ARRIVED_PICKUP_STAGING")
            print("formal_position_and_yaw_constraint=true")
            print("spawn_calibration_cubes=true")
            print("seq35_pose=" + json.dumps(seq35_pose, sort_keys=True))
            return 0
        timestamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        output_dir = (
            args.log_dir.expanduser().resolve()
            if args.log_dir is not None
            else DEFAULT_LOG_ROOT / "seq35_observation_once_{}".format(timestamp)
        )
        return 0 if run_once(args, output_dir, seq35_pose) else 1
    except KeyboardInterrupt:
        return 130
    except (AutomationError, CleanupError) as exc:
        print("run_seq35_observation_once: {}".format(exc), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
