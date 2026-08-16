#!/usr/bin/env python3
"""Restore fixed pickup cubes/cones and optionally slow Gazebo physics."""

import argparse
import json
import math
from pathlib import Path
import sys
import time

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import (
    GetModelState,
    GetPhysicsProperties,
    SetModelState,
    SetPhysicsProperties,
)
import rospy
from std_srvs.srv import Empty
from tf.transformations import quaternion_from_euler


STATION_YAWS = {35: -1.5708, 36: 0.0, 37: 1.5708}
ROBOT_MODEL = "car3"


def parse_args(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument("--case-file", type=Path, required=True)
    parser.add_argument("--service-timeout", type=float, default=15.0)
    return parser.parse_args(rospy.myargv(argv=[sys.argv[0]] + list(argv))[1:])


def state(name, x, y, yaw, z, roll=0.0, pitch=0.0):
    message = ModelState()
    message.model_name = name
    message.reference_frame = "world"
    message.pose.position.x = float(x)
    message.pose.position.y = float(y)
    message.pose.position.z = float(z)
    quaternion = quaternion_from_euler(float(roll), float(pitch), float(yaw))
    message.pose.orientation.x = quaternion[0]
    message.pose.orientation.y = quaternion[1]
    message.pose.orientation.z = quaternion[2]
    message.pose.orientation.w = quaternion[3]
    return message


def call_empty(name, timeout):
    rospy.wait_for_service(name, timeout=timeout)
    rospy.ServiceProxy(name, Empty)()


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    payload = json.loads(args.case_file.read_text(encoding="utf-8"))
    case = payload.get("case", payload)
    cubes = case["cubes"]
    if set(cubes) != {"cube_0", "cube_1", "cube_2"}:
        raise ValueError("case must define cube_0, cube_1, and cube_2")

    rospy.init_node("setup_pre_cone_failure_scene", anonymous=True)
    for service in (
        "/gazebo/set_model_state",
        "/gazebo/get_model_state",
        "/gazebo/pause_physics",
        "/gazebo/unpause_physics",
    ):
        rospy.wait_for_service(service, timeout=args.service_timeout)
    set_model = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
    get_model = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

    call_empty("/gazebo/pause_physics", args.service_timeout)
    try:
        for name, raw in cubes.items():
            station = int(raw["station"])
            result = set_model(
                state(
                    name,
                    raw["x"],
                    raw["y"],
                    STATION_YAWS[station],
                    0.02,
                )
            )
            if not result.success:
                raise RuntimeError("failed to place {}: {}".format(name, result.status_message))
        for name, raw in case.get("cones", {}).items():
            result = set_model(state(name, raw["x"], raw["y"], 0.0, 0.0))
            if not result.success:
                raise RuntimeError("failed to place {}: {}".format(name, result.status_message))

        robot = case.get("robot_pose")
        if robot:
            result = set_model(
                state(
                    ROBOT_MODEL,
                    robot["x"],
                    robot["y"],
                    robot.get("yaw", 0.0),
                    robot.get("z", 0.15),
                    robot.get("roll", 0.0),
                    robot.get("pitch", 0.0),
                )
            )
            if not result.success:
                raise RuntimeError(
                    "failed to place {}: {}".format(ROBOT_MODEL, result.status_message)
                )

        physics_scale = float(case.get("physics_update_rate_scale", 1.0))
        if physics_scale != 1.0:
            rospy.wait_for_service("/gazebo/get_physics_properties", timeout=args.service_timeout)
            rospy.wait_for_service("/gazebo/set_physics_properties", timeout=args.service_timeout)
            current = rospy.ServiceProxy(
                "/gazebo/get_physics_properties", GetPhysicsProperties
            )()
            updated = rospy.ServiceProxy(
                "/gazebo/set_physics_properties", SetPhysicsProperties
            )(
                current.time_step,
                current.max_update_rate * physics_scale,
                current.gravity,
                current.ode_config,
            )
            if not updated.success:
                raise RuntimeError("failed to scale Gazebo physics: {}".format(updated.status_message))
    finally:
        call_empty("/gazebo/unpause_physics", args.service_timeout)

    time.sleep(1.0)
    measured = {}
    for name, raw in cubes.items():
        result = get_model(name, "world")
        if not result.success:
            raise RuntimeError("cannot verify {}".format(name))
        error = math.hypot(
            result.pose.position.x - float(raw["x"]),
            result.pose.position.y - float(raw["y"]),
        )
        if error > 0.01:
            raise RuntimeError("{} moved {:.4f}m during setup".format(name, error))
        measured[name] = {
            "x": result.pose.position.x,
            "y": result.pose.position.y,
            "error": error,
        }
    call_empty("/move_base/clear_costmaps", args.service_timeout)
    print("PRE_CONE_FAILURE_SCENE=" + json.dumps(measured, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
