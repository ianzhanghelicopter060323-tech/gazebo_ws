#!/usr/bin/env python3
"""Apply one isolated seq35 stress-test scene to a running Gazebo instance."""

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


class SceneError(RuntimeError):
    pass


def parse_args(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument("--case-file", type=Path, required=True)
    parser.add_argument("--service-timeout", type=float, default=10.0)
    parser.add_argument("--localization-timeout", type=float, default=12.0)
    return parser.parse_args(argv)


def finite_pose(raw, label):
    if not isinstance(raw, dict):
        raise SceneError("{} must be a mapping".format(label))
    try:
        pose = {
            "x": float(raw["x"]),
            "y": float(raw["y"]),
            "z": float(raw.get("z", 0.0)),
            "yaw": float(raw.get("yaw", 0.0)),
        }
    except (KeyError, TypeError, ValueError) as exc:
        raise SceneError("{} contains an invalid pose".format(label)) from exc
    if not all(math.isfinite(value) for value in pose.values()):
        raise SceneError("{} contains a non-finite pose".format(label))
    return pose


def load_case(path):
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise SceneError("cannot read {}: {}".format(path, exc))
    if not isinstance(data, dict):
        raise SceneError("case file must contain a JSON object")
    models = data.get("models")
    if not isinstance(models, dict) or set(models) != {"cube_0", "cube_1", "cube_2"}:
        raise SceneError("case models must define exactly cube_0, cube_1, and cube_2")
    return {
        "models": {
            name: finite_pose(raw, "models.{}".format(name))
            for name, raw in models.items()
        },
        "robot_start": finite_pose(data.get("robot_start"), "robot_start"),
    }


def quaternion(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def model_state(name, pose):
    state = ModelState()
    state.model_name = name
    state.reference_frame = "world"
    state.pose.position.x = pose["x"]
    state.pose.position.y = pose["y"]
    state.pose.position.z = pose["z"]
    state.pose.orientation.z, state.pose.orientation.w = quaternion(pose["yaw"])
    state.twist.linear.x = 0.0
    state.twist.linear.y = 0.0
    state.twist.linear.z = 0.0
    state.twist.angular.x = 0.0
    state.twist.angular.y = 0.0
    state.twist.angular.z = 0.0
    return state


def yaw_from_quaternion(orientation):
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
    )


def pose_payload(pose):
    return {
        "x": float(pose.position.x),
        "y": float(pose.position.y),
        "z": float(pose.position.z),
        "yaw": float(yaw_from_quaternion(pose.orientation)),
    }


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
    message.pose.pose.position.x = pose["x"]
    message.pose.pose.position.y = pose["y"]
    message.pose.pose.position.z = 0.0
    message.pose.pose.orientation.z, message.pose.pose.orientation.w = quaternion(
        pose["yaw"]
    )
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
            "yaw": float(yaw_from_quaternion(pose.orientation)),
        }
        yaw_error = math.atan2(
            math.sin(last["yaw"] - expected["yaw"]),
            math.cos(last["yaw"] - expected["yaw"]),
        )
        if (
            math.hypot(last["x"] - expected["x"], last["y"] - expected["y"])
            <= 0.20
            and abs(yaw_error) <= 0.35
        ):
            return last
    raise SceneError(
        "AMCL did not converge near seq34 within {:.1f}s; last={}".format(
            timeout, last
        )
    )


def call_empty_service(name, timeout, required):
    try:
        rospy.wait_for_service(name, timeout=timeout)
        rospy.ServiceProxy(name, Empty)()
        return True
    except (rospy.ROSException, rospy.ServiceException) as exc:
        if required:
            raise SceneError("{} failed: {}".format(name, exc))
        rospy.logwarn("optional service %s failed: %s", name, exc)
        return False


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    try:
        case = load_case(args.case_file.expanduser().resolve())
        rospy.init_node("setup_seq35_stress_scene", anonymous=True)
        rospy.wait_for_service("/gazebo/set_model_state", timeout=args.service_timeout)
        rospy.wait_for_service("/gazebo/get_model_state", timeout=args.service_timeout)
        set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        get_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

        for name in sorted(case["models"]):
            response = set_state(model_state(name, case["models"][name]))
            if not response.success:
                raise SceneError(
                    "failed to place {}: {}".format(name, response.status_message)
                )
        response = set_state(model_state("car3", case["robot_start"]))
        if not response.success:
            raise SceneError("failed to place car3: {}".format(response.status_message))

        time.sleep(1.0)
        publish_initial_pose(case["robot_start"], args.service_timeout)
        call_empty_service("/request_nomotion_update", args.service_timeout, False)
        call_empty_service("/move_base/clear_costmaps", args.service_timeout, True)
        localized = wait_for_localization(
            case["robot_start"], args.localization_timeout
        )

        measured = {}
        for name in ("car3", "cube_0", "cube_1", "cube_2"):
            response = get_state(name, "world")
            if not response.success:
                raise SceneError(
                    "cannot verify {}: {}".format(name, response.status_message)
                )
            measured[name] = pose_payload(response.pose)
        result = {"measured": measured, "amcl": localized}
        print("SEQ35_STRESS_SCENE={}".format(json.dumps(result, sort_keys=True)))
        return 0
    except (
        OSError,
        SceneError,
        rospy.ROSException,
        rospy.ServiceException,
        ValueError,
    ) as exc:
        print("setup_seq35_stress_scene: {}".format(exc), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
