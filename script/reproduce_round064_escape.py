#!/usr/bin/env python3
"""Reproduce the round-064 stuck pose and exercise the bounded escape.

Drives the exact path that failed in round 064: the mission's
NAVIGATE_TO_PICKUP_CANDIDATE stage calls ``navigate_pose(...,
"navigating to requested pose")``.  This script sends one NAVIGATE_POSE
goal to the same action server after placing the robot at the recorded
stuck pose, so move_base detects no progress and the bounded escape runs
with the current 70%-clearance distance policy.
"""

import argparse
import json
import math
from pathlib import Path
import sys
import time

import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
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


SETUP_HELPER = WORKSPACE / "script" / "_setup_pre_cone_failure_scene.py"
SCAN_SECTOR_HALF_ANGLE = 0.52
AMCL_TOPIC = "/amcl_pose"
INITIALPOSE_TOPIC = "/initialpose"


def parse_args(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--case-file",
        type=Path,
        default=WORKSPACE / "script" / "config" / "round064_escape_repro.json",
    )
    parser.add_argument("--goal-x", type=float, default=-1.2243)
    parser.add_argument("--goal-y", type=float, default=-0.525)
    parser.add_argument("--goal-yaw", type=float, default=0.0)
    parser.add_argument("--startup-timeout", type=float, default=90.0)
    parser.add_argument("--task-timeout", type=float, default=240.0)
    parser.add_argument("--settle", type=float, default=12.0)
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--output-dir", type=Path)
    return parser.parse_args(argv)


def sector_clearance(scan, center):
    values = []
    for index, value in enumerate(scan.ranges):
        angle = scan.angle_min + index * scan.angle_increment
        delta = math.atan2(math.sin(angle - center), math.cos(angle - center))
        if (
            abs(delta) <= SCAN_SECTOR_HALF_ANGLE
            and math.isfinite(value)
            and scan.range_min <= value <= scan.range_max
        ):
            values.append(value)
    if not values:
        return math.nan
    values.sort()
    return values[max(0, int(0.1 * (len(values) - 1)))]


def wait_for_clock(server_timeout):
    deadline = time.monotonic() + server_timeout
    while (
        rospy.get_param("/use_sim_time", False)
        and rospy.Time.now() == rospy.Time()
        and time.monotonic() < deadline
    ):
        time.sleep(0.05)


def seed_amcl(x, y, yaw):
    publisher = rospy.Publisher(INITIALPOSE_TOPIC, PoseWithCovarianceStamped, queue_size=1)
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


def read_latest(topic, message_type, timeout):
    holder = []

    def callback(message):
        holder.append(message)

    subscriber = rospy.Subscriber(topic, message_type, callback, queue_size=1)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and not holder:
        time.sleep(0.1)
    subscriber.unregister()
    return holder[-1] if holder else None


def read_scan_and_report(rospy_node_ready=True):
    scan = read_latest("/scan", LaserScan, 8.0)
    if scan is None:
        return math.nan, math.nan
    front = sector_clearance(scan, 0.0)
    rear = sector_clearance(scan, math.pi)
    return front, rear


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    output_dir = (
        args.output_dir.expanduser().resolve()
        if args.output_dir
        else WORKSPACE
        / "script"
        / "logs"
        / ("round064_escape_repro_" + time.strftime("%Y%m%d_%H%M%S"))
    )
    output_dir.mkdir(parents=True, exist_ok=True)

    case = json.loads(args.case_file.read_text(encoding="utf-8"))
    case = case.get("case", case)
    robot = case["robot_pose"]
    if not all(
        math.isfinite(value)
        for value in (args.goal_x, args.goal_y, args.goal_yaw)
    ):
        raise AutomationError("goal pose must be finite")

    case_file = output_dir / "case.json"
    case_file.write_text(json.dumps(case, indent=2) + "\n", encoding="utf-8")

    launch = None
    run_id = None
    started = time.monotonic()
    record = {
        "case_id": case.get("id", "round_064_escape_repro"),
        "robot_pose": robot,
        "goal": {"x": args.goal_x, "y": args.goal_y, "yaw": args.goal_yaw},
        "success": False,
        "duration_seconds": None,
        "escape_events": [],
        "front_clearance": None,
        "rear_clearance": None,
    }
    with (output_dir / "roslaunch.log").open("w", encoding="utf-8") as log:
        try:
            if master_is_running():
                raise CleanupError("ROS master is already running")
            launch = launch_simulation(
                args.gui,
                log,
                start_perception=False,
            )
            deadline = time.monotonic() + args.startup_timeout
            wait_for_ros("ROS master", ["rosnode", "list"], deadline)
            run_id = read_ros_run_id()
            wait_for_simulation(args.startup_timeout)
            wait_for_ros(
                "navigation action",
                ["rostopic", "echo", "-n", "1", "/amcl_pose", "--noarr"],
                deadline,
            )
            time.sleep(5.0)

            setup_code, setup_output = run_owned(
                ["python3", str(SETUP_HELPER), "--case-file", str(case_file)],
                timeout=45.0,
            )
            (output_dir / "scene_setup.log").write_text(
                setup_output, encoding="utf-8"
            )
            if setup_code != 0:
                raise AutomationError("fixed scene setup failed")

            rospy.init_node("reproduce_round064_escape", anonymous=True)
            wait_for_clock(10.0)
            seed_amcl(robot["x"], robot["y"], robot.get("yaw", 0.0))
            time.sleep(args.settle)

            front, rear = read_scan_and_report()
            record["front_clearance"] = front
            record["rear_clearance"] = rear
            print(
                "reproduced clearance front={:.3f} rear={:.3f}".format(
                    front if math.isfinite(front) else float("nan"),
                    rear if math.isfinite(rear) else float("nan"),
                ),
                flush=True,
            )

            client = actionlib.SimpleActionClient(
                "/smart_factory/navigation", NavigateAction
            )
            if not client.wait_for_server(rospy.Duration(15.0)):
                raise AutomationError("navigation action server unavailable")

            goal = NavigateGoal()
            goal.command = NavigateGoal.NAVIGATE_POSE
            goal.request_id = "round064_escape_repro_{}".format(int(time.time()))
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = args.goal_x
            goal.target_pose.pose.position.y = args.goal_y
            goal.target_pose.pose.orientation.z = math.sin(args.goal_yaw / 2.0)
            goal.target_pose.pose.orientation.w = math.cos(args.goal_yaw / 2.0)

            def feedback_callback(feedback):
                print(
                    "feedback stage={} waypoint={}/{} retry={} detail={}".format(
                        feedback.phase,
                        feedback.current_waypoint,
                        feedback.waypoint_count,
                        feedback.retry_count,
                        feedback.detail,
                    ),
                    flush=True,
                )

            client.send_goal(goal, feedback_cb=feedback_callback)
            if not client.wait_for_result(
                rospy.Duration(args.task_timeout)
            ):
                client.cancel_goal()
                record["error"] = "navigation goal timed out after {:.0f}s".format(
                    args.task_timeout
                )
                print(record["error"], flush=True)
            else:
                state = client.get_state()
                result = client.get_result()
                record["goal_state"] = int(state)
                record["result_success"] = bool(result.success)
                record["result_error_code"] = int(result.error_code)
                record["result_message"] = str(result.message)
                print(
                    "navigation result state={} success={} error_code={} "
                    "message={}".format(
                        state,
                        result.success,
                        result.error_code,
                        result.message,
                    ),
                    flush=True,
                )
                record["success"] = bool(
                    state == GoalStatus.SUCCEEDED and result.success
                )
        except (AutomationError, CleanupError, OSError, ValueError) as exc:
            record["error"] = str(exc)
            print("reproduce_round064_escape: {}".format(exc), flush=True)
        finally:
            stop_owned_launch(launch, run_id)
            record["duration_seconds"] = round(time.monotonic() - started, 3)

    # Harvest bounded-recovery events from the launch log for the report.
    log_text = (output_dir / "roslaunch.log").read_text(
        encoding="utf-8", errors="replace"
    )
    for line in log_text.splitlines():
        if "bounded navigation recovery completed" in line:
            record["escape_events"].append(line.strip())
        elif "bounded recovery" in line and "[INFO]" in line:
            record["escape_events"].append(line.strip())

    (output_dir / "result.json").write_text(
        json.dumps(record, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print("result=" + json.dumps(record, ensure_ascii=False), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
