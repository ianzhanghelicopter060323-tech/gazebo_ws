#!/usr/bin/env python3
"""Execute the configured pickup route and accept seq35 by position only."""

import argparse
import json
import math
from pathlib import Path
import sys
import threading
import time

import actionlib
from geometry_msgs.msg import PoseStamped
import rospy
import yaml

from smart_factory_navigation.msg import (
    NavigateAction,
    NavigateGoal,
)


RESULT_MARKER = "PRE_NAVIGATION_RESULT="


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description=(
            "follow the configured pickup staging route and accept its final "
            "seq35 waypoint by XY distance without waiting for yaw alignment"
        )
    )
    parser.add_argument("--route", type=Path, required=True)
    parser.add_argument("--request-id", required=True)
    parser.add_argument("--action-name", default="/smart_factory/navigation")
    parser.add_argument("--server-wait-timeout", type=float, default=15.0)
    parser.add_argument("--localization-timeout", type=float, default=20.0)
    parser.add_argument("--navigation-timeout", type=float, default=300.0)
    parser.add_argument("--final-position-tolerance", type=float, default=0.15)
    return parser.parse_args(rospy.myargv(argv=[sys.argv[0]] + list(argv))[1:])


def quaternion_yaw(quaternion):
    return math.atan2(
        2.0
        * (
            quaternion.w * quaternion.z
            + quaternion.x * quaternion.y
        ),
        1.0
        - 2.0
        * (
            quaternion.y * quaternion.y
            + quaternion.z * quaternion.z
        ),
    )


def shortest_angular_distance(first, second):
    return math.atan2(math.sin(second - first), math.cos(second - first))


def make_pose(frame_id, x, y, yaw):
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
    pose.pose.orientation.w = math.cos(float(yaw) / 2.0)
    return pose


def load_route(path):
    path = path.expanduser().resolve()
    try:
        payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise ValueError("cannot read route {}: {}".format(path, exc))
    root = payload.get("pickup_staging", {}) if isinstance(payload, dict) else {}
    frame_id = str(root.get("frame_id", "")).strip()
    waypoints = root.get("waypoints")
    if not frame_id or not isinstance(waypoints, list) or len(waypoints) < 2:
        raise ValueError("route must contain a frame_id and at least two waypoints")
    result = []
    for index, raw in enumerate(waypoints, 1):
        try:
            values = tuple(float(raw[key]) for key in ("x", "y", "yaw"))
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("route waypoint {} is invalid".format(index)) from exc
        if not all(math.isfinite(value) for value in values):
            raise ValueError("route waypoint {} must be finite".format(index))
        result.append(make_pose(frame_id, *values))
    return frame_id, result


def wait_for_clock(timeout):
    deadline = time.monotonic() + timeout
    while (
        rospy.get_param("/use_sim_time", False)
        and rospy.Time.now() == rospy.Time()
        and time.monotonic() < deadline
        and not rospy.is_shutdown()
    ):
        time.sleep(0.05)
    return not (
        rospy.get_param("/use_sim_time", False)
        and rospy.Time.now() == rospy.Time()
    )


def wait_for_result_wall(client, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and not rospy.is_shutdown():
        if client.wait_for_result(rospy.Duration(0.1)):
            return True
    return False


def emit(payload):
    print(RESULT_MARKER + json.dumps(payload, ensure_ascii=False, sort_keys=True))


def failure_payload(message, status="client_error", error_code=255):
    return {
        "success": False,
        "status": status,
        "error_code": int(error_code),
        "server_error_code": None,
        "message": str(message),
        "acceptance_mode": "none",
        "yaw_alignment_required": False,
        "completed_waypoints": 0,
        "waypoint_count": 0,
        "final_position_error_m": None,
        "final_position_tolerance_m": None,
        "final_yaw_error_rad": None,
        "final_pose": None,
    }


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    try:
        if args.server_wait_timeout <= 0.0:
            raise ValueError("--server-wait-timeout must be positive")
        if args.localization_timeout <= 0.0:
            raise ValueError("--localization-timeout must be positive")
        if args.navigation_timeout <= 0.0:
            raise ValueError("--navigation-timeout must be positive")
        if args.final_position_tolerance <= 0.0:
            raise ValueError("--final-position-tolerance must be positive")
        frame_id, route = load_route(args.route)
    except ValueError as exc:
        emit(failure_payload(exc))
        return 2

    rospy.init_node("pre_navigation_route_test_client", anonymous=True)
    if not wait_for_clock(args.server_wait_timeout):
        emit(failure_payload("simulation clock did not start"))
        return 2

    client = actionlib.SimpleActionClient(args.action_name, NavigateAction)
    if not client.wait_for_server(rospy.Duration(args.server_wait_timeout)):
        emit(failure_payload("navigation action server is unavailable"))
        return 2

    localization_goal = NavigateGoal()
    localization_goal.command = NavigateGoal.WAIT_FOR_LOCALIZATION
    localization_goal.request_id = args.request_id
    client.send_goal(localization_goal)
    if not wait_for_result_wall(client, args.localization_timeout):
        client.cancel_goal()
        emit(failure_payload("localization readiness timed out", "localization_timeout", 4))
        return 1
    localization_result = client.get_result()
    if localization_result is None or not localization_result.success:
        error_code = 4 if localization_result is None else localization_result.error_code
        message = (
            "localization action returned no result"
            if localization_result is None
            else localization_result.message
        )
        emit(failure_payload(message, "localization_failed", error_code))
        return 1

    target = route[-1]
    target_yaw = quaternion_yaw(target.pose.orientation)
    state = {
        "accepted": False,
        "final_distance": None,
        "final_pose": None,
        "final_yaw_error": None,
        "current_waypoint": 0,
        "waypoint_count": 0,
    }
    lock = threading.Lock()
    final_position_reached = threading.Event()

    def feedback_callback(feedback):
        with lock:
            state["current_waypoint"] = int(feedback.current_waypoint)
            state["waypoint_count"] = int(feedback.waypoint_count)
            is_final = (
                feedback.waypoint_count > 0
                and feedback.current_waypoint == feedback.waypoint_count
            )
            if not is_final or not feedback.distance_to_goal_valid:
                return
            state["final_distance"] = float(feedback.distance_to_goal)
            if feedback.localized_pose_valid:
                pose = feedback.localized_pose
                localized_yaw = quaternion_yaw(pose.pose.orientation)
                state["final_pose"] = {
                    "frame_id": pose.header.frame_id or frame_id,
                    "x": float(pose.pose.position.x),
                    "y": float(pose.pose.position.y),
                    "yaw": float(localized_yaw),
                }
                state["final_yaw_error"] = abs(
                    shortest_angular_distance(localized_yaw, target_yaw)
                )
            if (
                not state["accepted"]
                and feedback.distance_to_goal <= args.final_position_tolerance
            ):
                state["accepted"] = True
                final_position_reached.set()

    route_goal = NavigateGoal()
    route_goal.command = NavigateGoal.EXECUTE_STAGING_ROUTE
    route_goal.request_id = args.request_id
    route_goal.waypoints = route
    started = time.monotonic()
    client.send_goal(route_goal, feedback_cb=feedback_callback)
    deadline = time.monotonic() + args.navigation_timeout
    cancel_sent = False
    finished = False
    while time.monotonic() < deadline and not rospy.is_shutdown():
        if final_position_reached.is_set() and not cancel_sent:
            client.cancel_goal()
            cancel_sent = True
        if client.wait_for_result(rospy.Duration(0.1)):
            finished = True
            break
    if not finished:
        client.cancel_goal()
        finished = wait_for_result_wall(client, 2.0)
    elapsed = time.monotonic() - started
    result = client.get_result()
    server_success = bool(result is not None and result.success)

    # Read one final localized pose after cancellation/success instead of
    # relying solely on the last feedback packet. A fast terminal transition
    # can otherwise finish between feedback heartbeats and leave the recorded
    # final error empty even though seq35 was reached.
    pose_goal = NavigateGoal()
    pose_goal.command = NavigateGoal.GET_LOCALIZED_POSE
    pose_goal.request_id = args.request_id + "_final_pose"
    pose_goal.frame_id = frame_id
    client.send_goal(pose_goal)
    if wait_for_result_wall(client, 5.0):
        pose_result = client.get_result()
        if pose_result is not None and pose_result.success and pose_result.pose_valid:
            pose = pose_result.localized_pose
            localized_yaw = quaternion_yaw(pose.pose.orientation)
            measured_distance = math.hypot(
                pose.pose.position.x - target.pose.position.x,
                pose.pose.position.y - target.pose.position.y,
            )
            with lock:
                state["final_distance"] = float(measured_distance)
                state["final_pose"] = {
                    "frame_id": pose.header.frame_id or frame_id,
                    "x": float(pose.pose.position.x),
                    "y": float(pose.pose.position.y),
                    "yaw": float(localized_yaw),
                }
                state["final_yaw_error"] = abs(
                    shortest_angular_distance(localized_yaw, target_yaw)
                )

    with lock:
        accepted = bool(state["accepted"])
        final_distance = state["final_distance"]
        final_pose = state["final_pose"]
        final_yaw_error = state["final_yaw_error"]
        waypoint_count = state["waypoint_count"] or len(route)
        current_waypoint = state["current_waypoint"]

    position_accepted = bool(
        final_distance is not None
        and final_distance <= args.final_position_tolerance + 1.0e-9
    )
    success = bool(
        position_accepted and (accepted or (finished and server_success))
    )
    if accepted:
        status = "navigation_completed"
        acceptance_mode = "position_tolerance"
        message = (
            "seq35 entered {:.3f} m position tolerance; final yaw was not required"
        ).format(args.final_position_tolerance)
    elif server_success:
        status = "navigation_completed"
        acceptance_mode = "move_base_succeeded"
        message = result.message
    elif not finished:
        status = "navigation_timeout"
        acceptance_mode = "none"
        message = "route navigation timed out after {:.1f}s".format(
            args.navigation_timeout
        )
    else:
        status = "navigation_failed"
        acceptance_mode = "none"
        message = (
            "navigation action returned no result"
            if result is None
            else result.message
        )

    payload = {
        "success": success,
        "status": status,
        "error_code": 0 if success else (255 if result is None else int(result.error_code)),
        "server_error_code": None if result is None else int(result.error_code),
        "message": message,
        "acceptance_mode": acceptance_mode,
        "yaw_alignment_required": False,
        "completed_waypoints": waypoint_count if success else current_waypoint,
        "waypoint_count": waypoint_count,
        "final_position_error_m": final_distance,
        "final_position_tolerance_m": float(args.final_position_tolerance),
        "final_yaw_error_rad": final_yaw_error,
        "final_pose": final_pose,
        "target_pose": {
            "frame_id": target.header.frame_id,
            "x": float(target.pose.position.x),
            "y": float(target.pose.position.y),
            "yaw": float(target_yaw),
        },
        "duration_seconds": round(elapsed, 3),
    }
    emit(payload)
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
