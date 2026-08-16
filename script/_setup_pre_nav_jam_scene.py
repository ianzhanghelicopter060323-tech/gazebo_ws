#!/usr/bin/env python3
"""Spawn deterministic jam cubes on the pre-navigation fitted path.

A cube's 0.04 m height sits below the 0.087 m laser plane, so move_base never
sees it, plans a straight path through it, and the robot physically wedges on
it -- the same mechanism as the round-064 jam. The case file may also
reposition the robot with an optional ``robot_pose``.

Case JSON:
{
  "jam_cubes": [{"x": .., "y": .., "yaw": .., "z": .., "static": bool,
                 "size": [x, y, z],  # optional box size, default 0.04^3
                 "mass": ..}],
  "robot_pose": {"x": .., "y": .., "yaw": ..}     # optional
}
"""

import argparse
import json
import math
from pathlib import Path
import sys

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import DeleteModel, GetModelState, SetModelState, SpawnModel
from geometry_msgs.msg import Point, Pose, Quaternion
import rospy
from std_srvs.srv import Empty
from tf.transformations import quaternion_from_euler


CUBE_SDF = (
    Path(__file__).resolve().parent.parent
    / "src"
    / "car3"
    / "models"
    / "cube"
    / "model_0.sdf"
)
ROBOT_MODEL = "car3"


def parse_args(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument("--case-file", type=Path, required=True)
    parser.add_argument("--service-timeout", type=float, default=15.0)
    return parser.parse_args(rospy.myargv(argv=[sys.argv[0]] + list(argv))[1:])


def build_cube_sdf(static, mass, size=None):
    text = CUBE_SDF.read_text(encoding="utf-8")
    if static:
        text = text.replace("<static>false</static>", "<static>true</static>")
    if mass is not None:
        text = text.replace("<mass>0.02</mass>", "<mass>{:.6g}</mass>".format(mass))
    if size is not None:
        text = text.replace(
            "<size>0.04 0.04 0.04</size>",
            "<size>{} {} {}</size>".format(*size),
        )
    return text


def call_empty(name, timeout):
    rospy.wait_for_service(name, timeout=timeout)
    rospy.ServiceProxy(name, Empty)()


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    payload = json.loads(args.case_file.read_text(encoding="utf-8"))
    case = payload.get("case", payload)
    jam_cubes = case.get("jam_cubes", [])
    if not jam_cubes:
        raise ValueError("case must define at least one jam_cube")

    rospy.init_node("setup_pre_nav_jam_scene", anonymous=True)
    for service in (
        "/gazebo/set_model_state",
        "/gazebo/get_model_state",
        "/gazebo/spawn_sdf_model",
        "/gazebo/delete_model",
    ):
        rospy.wait_for_service(service, timeout=args.service_timeout)
    spawn_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
    delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
    set_model = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
    get_model = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

    call_empty("/gazebo/pause_physics", args.service_timeout)
    try:
        for index, raw in enumerate(jam_cubes):
            name = raw.get("name", "jam_cube_{}".format(index))
            try:
                delete_model(name)
            except rospy.ServiceException:
                pass
            sdf = build_cube_sdf(
                bool(raw.get("static", False)),
                raw.get("mass"),
                raw.get("size"),
            )
            yaw = float(raw.get("yaw", 0.0))
            pose = Pose(
                position=Point(
                    x=float(raw["x"]),
                    y=float(raw["y"]),
                    z=float(raw.get("z", 0.02)),
                ),
                orientation=Quaternion(
                    x=0.0,
                    y=0.0,
                    z=math.sin(yaw / 2.0),
                    w=math.cos(yaw / 2.0),
                ),
            )
            result = spawn_model(name, sdf, "", pose, "world")
            if not result.success:
                raise RuntimeError(
                    "failed to spawn {}: {}".format(name, result.status_message)
                )
            rospy.loginfo(
                "spawned jam cube %s -> (%.3f, %.3f) static=%s",
                name, raw["x"], raw["y"], raw.get("static", False),
            )

        robot = case.get("robot_pose")
        if robot:
            message = ModelState()
            message.model_name = ROBOT_MODEL
            message.reference_frame = "world"
            message.pose.position.x = float(robot["x"])
            message.pose.position.y = float(robot["y"])
            message.pose.position.z = float(robot.get("z", 0.15))
            q = quaternion_from_euler(
                0.0, 0.0, float(robot.get("yaw", 0.0))
            )
            message.pose.orientation.x = q[0]
            message.pose.orientation.y = q[1]
            message.pose.orientation.z = q[2]
            message.pose.orientation.w = q[3]
            result = set_model(message)
            if not result.success:
                raise RuntimeError(
                    "failed to place {}: {}".format(
                        ROBOT_MODEL, result.status_message
                    )
                )
    finally:
        call_empty("/gazebo/unpause_physics", args.service_timeout)

    call_empty("/move_base/clear_costmaps", args.service_timeout)
    print("PRE_NAV_JAM_SCENE=" + json.dumps({"jam_cubes": jam_cubes}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
