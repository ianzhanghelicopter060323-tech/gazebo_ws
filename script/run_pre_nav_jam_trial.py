#!/usr/bin/env python3
"""Headless pre-navigation jam trial: stuck -> bounded recovery -> continue.

Launches the competition simulation without GUI, injects a deterministic jam
cube (invisible to the laser) on the pre-navigation fitted path, then executes
the full pickup staging route to seq35. Harvests bounded-recovery events to
prove the recovery layer resumed navigation after the jam instead of giving
up on the first no-progress verdict.
"""

import argparse
import csv
import json
import math
from pathlib import Path
import subprocess
import sys
import time

import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
import rospy

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
from smart_factory_navigation.msg import NavigateAction, NavigateGoal


SETUP_HELPER = WORKSPACE / "script" / "_setup_pre_nav_jam_scene.py"
POSE_SAMPLE_SCRIPT = WORKSPACE / "script" / "_sample_robot_pose.py"
CLIENT_SCRIPT = WORKSPACE / "script" / "_run_pre_navigation_route.py"
ROUTE_CONFIG = (
    WORKSPACE
    / "src"
    / "smart_factory_mission"
    / "config"
    / "pickup_staging_dev.yaml"
)
FITTED_PATH_CONFIG = (
    WORKSPACE
    / "src"
    / "smart_factory_navigation"
    / "config"
    / "pickup_staging_fitted_path.yaml"
)
NAVIGATION_CONFIG = (
    WORKSPACE
    / "src"
    / "smart_factory_navigation"
    / "config"
    / "navigation.yaml"
)
RESULT_MARKER = "PRE_NAVIGATION_RESULT="
AMCL_TOPIC = "/amcl_pose"
INITIALPOSE_TOPIC = "/initialpose"
DEFAULT_CASE = (
    WORKSPACE
    / "script"
    / "config"
    / "pre_nav_jam_trial.json"
)


def parse_args(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--case-file", type=Path, default=DEFAULT_CASE
    )
    parser.add_argument("--startup-timeout", type=float, default=90.0)
    parser.add_argument("--startup-settle", type=float, default=6.0)
    parser.add_argument("--navigation-timeout", type=float, default=300.0)
    parser.add_argument("--task-timeout", type=float, default=360.0)
    parser.add_argument("--final-position-tolerance", type=float, default=0.15)
    display = parser.add_mutually_exclusive_group()
    display.add_argument("--gui", dest="gui", action="store_true")
    display.add_argument("--headless", dest="gui", action="store_false")
    parser.set_defaults(gui=False)
    parser.add_argument(
        "--watch-tilt",
        action="store_true",
        help=(
            "observation mode: force Gazebo+RViz GUI and run the live pose "
            "sampler concurrently so roll/pitch is printed as it happens "
            "(and a pose_trace.csv + tilt_peak summary is recorded)"
        ),
    )
    parser.add_argument(
        "--trace-pose",
        action="store_true",
        help=(
            "record pose_trace.csv (x/y/z, roll/pitch/yaw) during the trial "
            "without requiring a GUI; combine with --headless for quiet "
            "validation runs that must confirm the robot stays level"
        ),
    )
    parser.add_argument("--tilt-degrees", type=float, default=5.0)
    parser.add_argument("--output-dir", type=Path)
    return parser.parse_args(argv)


def wait_for_clock(server_timeout):
    deadline = time.monotonic() + server_timeout
    while (
        rospy.get_param("/use_sim_time", False)
        and rospy.Time.now() == rospy.Time()
        and time.monotonic() < deadline
    ):
        time.sleep(0.05)


def seed_amcl(x, y, yaw):
    publisher = rospy.Publisher(
        INITIALPOSE_TOPIC, PoseWithCovarianceStamped, queue_size=1
    )
    for _ in range(20):
        message = PoseWithCovarianceStamped()
        message.header.frame_id = "map"
        message.header.stamp = rospy.Time.now()
        message.pose.pose.position.x = x
        message.pose.pose.position.y = y
        message.pose.pose.orientation.z = math.sin(yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(yaw / 2.0)
        message.pose.covariance[0] = 0.05
        message.pose.covariance[7] = 0.05
        message.pose.covariance[35] = 0.05
        publisher.publish(message)
        time.sleep(0.05)


def read_tilt_peak(csv_path):
    """Return (max_pitch_deg, max_roll_deg, t_sim_at_peak) from pose_trace.csv."""
    max_pitch = 0.0
    max_roll = 0.0
    peak_sim = None
    with csv_path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            pitch = abs(float(row["pitch_deg"]))
            roll = abs(float(row["roll_deg"]))
            if pitch > max_pitch:
                max_pitch = pitch
                peak_sim = row["t_sim"]
            if roll > max_roll:
                max_roll = roll
    return max_pitch, max_roll, peak_sim


def parse_client_result(output):
    matches = [
        line[len(RESULT_MARKER):]
        for line in output.splitlines()
        if line.startswith(RESULT_MARKER)
    ]
    if not matches:
        raise AutomationError(
            "navigation client did not emit {}".format(RESULT_MARKER)
        )
    try:
        return json.loads(matches[-1])
    except ValueError as exc:
        raise AutomationError(
            "navigation client result is invalid JSON"
        ) from exc


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    output_dir = (
        args.output_dir.expanduser().resolve()
        if args.output_dir
        else WORKSPACE
        / "script"
        / "logs"
        / ("pre_nav_jam_trial_" + time.strftime("%Y%m%d_%H%M%S"))
    )
    output_dir.mkdir(parents=True, exist_ok=True)

    case = json.loads(args.case_file.read_text(encoding="utf-8"))
    case = case.get("case", case)
    jam_cubes = case.get("jam_cubes", [])
    if not jam_cubes:
        raise AutomationError("case must define at least one jam_cube")
    robot = case.get("robot_pose", {"x": 0.0, "y": 0.0, "yaw": 0.0})
    gui = args.gui or args.watch_tilt

    case_file = output_dir / "case.json"
    case_file.write_text(json.dumps(case, indent=2) + "\n", encoding="utf-8")

    launch = None
    run_id = None
    sampler = None
    started = time.monotonic()
    record = {
        "case_id": case.get("id", "pre_nav_jam_trial"),
        "jam_cubes": jam_cubes,
        "success": False,
        "duration_seconds": None,
        "escape_events": [],
        "recovery_count": 0,
        "client_result": None,
    }
    with (output_dir / "roslaunch.log").open("w", encoding="utf-8") as log:
        try:
            if master_is_running():
                raise CleanupError("ROS master is already running")
            launch = launch_simulation(
                gui,
                log,
                goal_config=ROUTE_CONFIG,
                fitted_path_config=FITTED_PATH_CONFIG,
                start_perception=False,
                navigation_config=NAVIGATION_CONFIG,
            )
            deadline = time.monotonic() + args.startup_timeout
            wait_for_ros("ROS master", ["rosnode", "list"], deadline)
            run_id = read_ros_run_id()
            wait_for_simulation(args.startup_timeout)
            wait_for_ros(
                "move_base action server",
                ["rostopic", "echo", "-n", "1", "/move_base/status", "--noarr"],
                deadline,
            )
            if launch.poll() is not None:
                raise AutomationError("roslaunch exited during startup")
            time.sleep(args.startup_settle)

            setup_code, setup_output = run_owned(
                [
                    "python3",
                    str(SETUP_HELPER),
                    "--case-file",
                    str(case_file),
                ],
                timeout=45.0,
            )
            (output_dir / "scene_setup.log").write_text(
                setup_output, encoding="utf-8"
            )
            if setup_code != 0:
                raise AutomationError("jam scene setup failed")

            rospy.init_node("run_pre_nav_jam_trial", anonymous=True)
            wait_for_clock(10.0)
            seed_amcl(robot["x"], robot["y"], robot.get("yaw", 0.0))
            time.sleep(3.0)

            if args.watch_tilt or args.trace_pose:
                sampler_command = [
                    sys.executable,
                    str(POSE_SAMPLE_SCRIPT),
                    "--seconds",
                    str(int(args.navigation_timeout) + 120),
                    "--interval",
                    "0.3",
                    "--tilt-degrees",
                    str(args.tilt_degrees),
                    "--csv",
                    str(output_dir / "pose_trace.csv"),
                ]
                if args.trace_pose and not args.watch_tilt:
                    sampler_command.append("--quiet")
                sampler = subprocess.Popen(
                    sampler_command,
                    stdout=sys.stdout,
                    stderr=subprocess.STDOUT,
                )
                if args.watch_tilt:
                    print(
                        "watching tilt: pose sampler running; live roll/pitch "
                        "below, tilt mark >= {:.1f}deg, trace -> {}".format(
                            args.tilt_degrees,
                            output_dir / "pose_trace.csv",
                        ),
                        flush=True,
                    )
                else:
                    print(
                        "tracing pose -> {}".format(
                            output_dir / "pose_trace.csv"
                        ),
                        flush=True,
                    )

            return_code, output = run_owned(
                [
                    "python3",
                    str(CLIENT_SCRIPT),
                    "--route",
                    str(ROUTE_CONFIG),
                    "--request-id",
                    "pre_nav_jam_{}".format(int(time.time())),
                    "--navigation-timeout",
                    str(args.navigation_timeout),
                    "--final-position-tolerance",
                    str(args.final_position_tolerance),
                ],
                timeout=args.task_timeout,
            )
            (output_dir / "client.log").write_text(output, encoding="utf-8")
            if sampler is not None:
                sampler.terminate()
                try:
                    sampler.wait(timeout=10.0)
                except subprocess.TimeoutExpired:
                    sampler.kill()
                sampler = None
            client_result = parse_client_result(output)
            record["client_result"] = client_result
            client_success = bool(client_result.get("success"))
            status = str(client_result.get("status"))
            record["success"] = bool(
                client_success and status == "navigation_completed"
            )
            print(
                "navigation client: success={} status={} message={}".format(
                    client_success,
                    status,
                    client_result.get("message"),
                ),
                flush=True,
            )

            # Diagnose whether the robot moved the jam cubes during the trial.
            from gazebo_msgs.srv import GetModelState
            from gazebo_msgs.msg import ModelState as _MS

            try:
                rospy.wait_for_service("/gazebo/get_model_state", timeout=5.0)
                get_model = rospy.ServiceProxy(
                    "/gazebo/get_model_state", GetModelState
                )
                cube_final = {}
                for index, raw in enumerate(jam_cubes):
                    name = raw.get("name", "jam_cube_{}".format(index))
                    result = get_model(name, "world")
                    if result.success:
                        cube_final[name] = {
                            "initial": {
                                "x": raw["x"],
                                "y": raw["y"],
                            },
                            "final": {
                                "x": result.pose.position.x,
                                "y": result.pose.position.y,
                                "z": result.pose.position.z,
                            },
                            "moved": round(
                                math.hypot(
                                    result.pose.position.x - raw["x"],
                                    result.pose.position.y - raw["y"],
                                ),
                                4,
                            ),
                        }
                record["jam_cube_final_positions"] = cube_final
            except rospy.ServiceException as exc:
                record["jam_cube_final_positions_error"] = str(exc)
        except (AutomationError, CleanupError, OSError, ValueError) as exc:
            record["error"] = str(exc)
            print("run_pre_nav_jam_trial: {}".format(exc), flush=True)
        finally:
            if sampler is not None:
                sampler.terminate()
                try:
                    sampler.wait(timeout=10.0)
                except subprocess.TimeoutExpired:
                    sampler.kill()
            stop_owned_launch(launch, run_id)
            record["duration_seconds"] = round(time.monotonic() - started, 3)

    # Summarise the peak tilt observed in watch-tilt / trace-pose mode.
    if args.watch_tilt or args.trace_pose:
        trace = output_dir / "pose_trace.csv"
        if trace.exists():
            max_pitch, max_roll, peak_sim = read_tilt_peak(trace)
            record["tilt_peak"] = {
                "max_pitch_deg": round(max_pitch, 2),
                "max_roll_deg": round(max_roll, 2),
                "at_sim": peak_sim,
            }
            print(
                "tilt peak: max pitch={:.2f}deg max roll={:.2f}deg "
                "at_sim={}".format(max_pitch, max_roll, peak_sim),
                flush=True,
            )

    # Harvest bounded-recovery events from the launch log for the report.
    log_text = (output_dir / "roslaunch.log").read_text(
        encoding="utf-8", errors="replace"
    )
    for line in log_text.splitlines():
        if "bounded navigation recovery completed" in line:
            record["escape_events"].append(line.strip())
        elif "bounded recovery" in line and "[INFO]" in line:
            record["escape_events"].append(line.strip())
    # Each successful escape() run logs "bounded navigation recovery completed";
    # the route_executor's "bounded recovery N/M" states go to a topic, not the
    # launch log, so count the escape completions directly.
    record["recovery_count"] = sum(
        1
        for event in record["escape_events"]
        if "bounded navigation recovery completed" in event
    )

    (output_dir / "result.json").write_text(
        json.dumps(record, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print("result=" + json.dumps(record, ensure_ascii=False), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
