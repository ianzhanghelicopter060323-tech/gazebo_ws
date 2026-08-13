#!/usr/bin/env python3
"""Restore a fixed cone layout and preparation pose in a running Gazebo world."""

import argparse
import json
import math
from pathlib import Path
import sys
import time

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState
from geometry_msgs.msg import PoseWithCovarianceStamped
import rospy
from std_srvs.srv import Empty


CONE_NAMES = tuple("cone_{}".format(number) for number in range(10, 20))


class SceneError(RuntimeError):
    pass


def parse_args(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument("--case-file", type=Path, required=True)
    parser.add_argument(
        "--cones-only",
        action="store_true",
        help=(
            "restore only cone_10 through cone_19; leave car3 and AMCL "
            "untouched for an end-to-end task"
        ),
    )
    parser.add_argument("--service-timeout", type=float, default=10.0)
    parser.add_argument("--localization-timeout", type=float, default=15.0)
    parser.add_argument("--cone-position-tolerance", type=float, default=0.005)
    return parser.parse_args(rospy.myargv(argv=[sys.argv[0]] + list(argv))[1:])


def finite(value, label):
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise SceneError("{} must be numeric".format(label)) from exc
    if not math.isfinite(result):
        raise SceneError("{} must be finite".format(label))
    return result


def robot_pose(raw):
    if not isinstance(raw, dict):
        raise SceneError("robot_start must be a mapping")
    return {
        "position": {
            "x": finite(raw.get("x"), "robot_start.x"),
            "y": finite(raw.get("y"), "robot_start.y"),
            "z": finite(raw.get("z", 0.01), "robot_start.z"),
        },
        "orientation": {
            "x": 0.0,
            "y": 0.0,
            "z": math.sin(finite(raw.get("yaw"), "robot_start.yaw") / 2.0),
            "w": math.cos(finite(raw.get("yaw"), "robot_start.yaw") / 2.0),
        },
        "yaw": finite(raw.get("yaw"), "robot_start.yaw"),
    }


def model_pose(raw, label):
    if not isinstance(raw, dict):
        raise SceneError("{} must be a mapping".format(label))
    position = raw.get("position")
    orientation = raw.get("orientation")
    if not isinstance(position, dict) or not isinstance(orientation, dict):
        raise SceneError("{} must contain position and orientation".format(label))
    parsed = {
        "position": {
            axis: finite(position.get(axis), "{}.position.{}".format(label, axis))
            for axis in ("x", "y", "z")
        },
        "orientation": {
            axis: finite(
                orientation.get(axis), "{}.orientation.{}".format(label, axis)
            )
            for axis in ("x", "y", "z", "w")
        },
    }
    norm = math.sqrt(
        sum(value * value for value in parsed["orientation"].values())
    )
    if norm <= 1.0e-9:
        raise SceneError("{} orientation is zero".format(label))
    parsed["orientation"] = {
        axis: value / norm for axis, value in parsed["orientation"].items()
    }
    return parsed


def load_case(path, require_robot=True):
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        raise SceneError("cannot load {}: {}".format(path, exc))
    cones = data.get("cones") if isinstance(data, dict) else None
    if not isinstance(cones, dict) or set(cones) != set(CONE_NAMES):
        raise SceneError("case cones must define exactly cone_10 through cone_19")
    parsed = {
        "cones": {
            name: model_pose(cones[name], "cones.{}".format(name))
            for name in CONE_NAMES
        },
    }
    if require_robot:
        parsed["robot_start"] = robot_pose(data.get("robot_start"))
    return parsed


def to_model_state(name, pose):
    state = ModelState()
    state.model_name = name
    state.reference_frame = "world"
    for axis, value in pose["position"].items():
        setattr(state.pose.position, axis, value)
    for axis, value in pose["orientation"].items():
        setattr(state.pose.orientation, axis, value)
    return state


def yaw_from_orientation(orientation):
    return math.atan2(
        2.0 * (
            orientation.w * orientation.z + orientation.x * orientation.y
        ),
        1.0 - 2.0 * (
            orientation.y * orientation.y + orientation.z * orientation.z
        ),
    )


def publish_initial_pose(pose, timeout):
    publisher = rospy.Publisher(
        "/initialpose", PoseWithCovarianceStamped, queue_size=1, latch=True
    )
    deadline = time.monotonic() + timeout
    while publisher.get_num_connections() == 0 and time.monotonic() < deadline:
        time.sleep(0.05)
    if publisher.get_num_connections() == 0:
        raise SceneError("AMCL did not subscribe to /initialpose")
    message = PoseWithCovarianceStamped()
    message.header.frame_id = "map"
    message.pose.pose.position.x = pose["position"]["x"]
    message.pose.pose.position.y = pose["position"]["y"]
    message.pose.pose.orientation.z = pose["orientation"]["z"]
    message.pose.pose.orientation.w = pose["orientation"]["w"]
    message.pose.covariance[0] = 0.0004
    message.pose.covariance[7] = 0.0004
    message.pose.covariance[35] = 0.0003
    for _unused in range(5):
        message.header.stamp = rospy.Time.now()
        publisher.publish(message)
        time.sleep(0.2)


def wait_for_localization(expected, timeout):
    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline and not rospy.is_shutdown():
        try:
            message = rospy.wait_for_message(
                "/amcl_pose",
                PoseWithCovarianceStamped,
                timeout=min(1.0, max(0.1, deadline - time.monotonic())),
            )
        except rospy.ROSException:
            continue
        pose = message.pose.pose
        last = {
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "yaw": float(yaw_from_orientation(pose.orientation)),
        }
        yaw_error = math.atan2(
            math.sin(last["yaw"] - expected["yaw"]),
            math.cos(last["yaw"] - expected["yaw"]),
        )
        if (
            math.hypot(
                last["x"] - expected["position"]["x"],
                last["y"] - expected["position"]["y"],
            )
            <= 0.12
            and abs(yaw_error) <= 0.25
        ):
            return last
    raise SceneError(
        "AMCL did not converge near preparation pose within {:.1f}s; last={}".format(
            timeout, last
        )
    )


def call_empty(name, timeout, required=True):
    try:
        rospy.wait_for_service(name, timeout=timeout)
        rospy.ServiceProxy(name, Empty)()
    except (rospy.ROSException, rospy.ServiceException) as exc:
        if required:
            raise SceneError("{} failed: {}".format(name, exc))
        rospy.logwarn("optional service %s failed: %s", name, exc)


def verify_model(get_state, name, expected, xy_tolerance):
    response = get_state(name, "world")
    if not response.success:
        raise SceneError("cannot verify {}: {}".format(name, response.status_message))
    measured = {
        "x": float(response.pose.position.x),
        "y": float(response.pose.position.y),
        "z": float(response.pose.position.z),
        "yaw": float(yaw_from_orientation(response.pose.orientation)),
    }
    error = math.hypot(
        measured["x"] - expected["position"]["x"],
        measured["y"] - expected["position"]["y"],
    )
    if error > xy_tolerance:
        raise SceneError(
            "{} settled {:.4f} m from requested position".format(name, error)
        )
    return measured


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    try:
        if args.cone_position_tolerance <= 0.0:
            raise SceneError("--cone-position-tolerance must be positive")
        case = load_case(
            args.case_file.expanduser().resolve(), require_robot=not args.cones_only
        )
        rospy.init_node("setup_cone_move_stress_scene", anonymous=True)
        for service in (
            "/gazebo/set_model_state",
            "/gazebo/get_model_state",
            "/gazebo/pause_physics",
            "/gazebo/unpause_physics",
        ):
            rospy.wait_for_service(service, timeout=args.service_timeout)
        set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        get_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

        call_empty("/gazebo/pause_physics", args.service_timeout)
        try:
            for name in CONE_NAMES:
                response = set_state(to_model_state(name, case["cones"][name]))
                if not response.success:
                    raise SceneError(
                        "failed to place {}: {}".format(name, response.status_message)
                    )
            if not args.cones_only:
                response = set_state(to_model_state("car3", case["robot_start"]))
                if not response.success:
                    raise SceneError(
                        "failed to place car3: {}".format(response.status_message)
                    )
        finally:
            call_empty("/gazebo/unpause_physics", args.service_timeout)

        time.sleep(1.0)
        localized = None
        if not args.cones_only:
            publish_initial_pose(case["robot_start"], args.service_timeout)
            call_empty(
                "/request_nomotion_update", args.service_timeout, required=False
            )
        call_empty("/move_base/clear_costmaps", args.service_timeout)
        if not args.cones_only:
            localized = wait_for_localization(
                case["robot_start"], args.localization_timeout
            )
        measured = {
            name: verify_model(
                get_state,
                name,
                case["cones"][name],
                args.cone_position_tolerance,
            )
            for name in CONE_NAMES
        }
        if not args.cones_only:
            measured["car3"] = verify_model(
                get_state, "car3", case["robot_start"], 0.05
            )
        print(
            "CONE_MOVE_STRESS_SCENE="
            + json.dumps(
                {"measured": measured, "amcl": localized}, sort_keys=True
            )
        )
        return 0
    except (
        OSError,
        SceneError,
        rospy.ROSException,
        rospy.ServiceException,
        ValueError,
    ) as exc:
        print("setup_cone_move_stress_scene: {}".format(exc), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
