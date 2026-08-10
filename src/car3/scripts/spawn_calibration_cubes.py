#!/usr/bin/env python3
"""Spawn deterministic 4 cm cubes for pickup-region camera calibration."""

import math
from pathlib import Path

from gazebo_msgs.srv import DeleteModel, GetWorldProperties, SpawnModel
from geometry_msgs.msg import Point, Pose, Quaternion
import rospy
import yaml


def load_config(path):
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    config = data.get("calibration_cubes", {}) if isinstance(data, dict) else {}
    size = float(config.get("size", 0.04))
    z = float(config.get("z", size / 2.0))
    frame_id = str(config.get("frame_id", "world")).strip()
    material = str(config.get("material", "Gazebo/Orange")).strip()
    markers = config.get("markers", [])
    if not math.isfinite(size) or size <= 0.0:
        raise ValueError("calibration_cubes.size must be positive")
    if not math.isfinite(z):
        raise ValueError("calibration_cubes.z must be finite")
    if not frame_id:
        raise ValueError("calibration_cubes.frame_id must not be empty")
    if not material:
        raise ValueError("calibration_cubes.material must not be empty")
    if not isinstance(markers, list) or not markers:
        raise ValueError("calibration_cubes.markers must be a non-empty list")

    parsed = []
    names = set()
    for index, marker in enumerate(markers, 1):
        if not isinstance(marker, dict):
            raise ValueError("marker {} must be a mapping".format(index))
        name = str(marker.get("name", "")).strip()
        region = str(marker.get("region", "")).strip()
        if not name or name in names:
            raise ValueError("marker names must be non-empty and unique")
        names.add(name)
        try:
            x = float(marker["x"])
            y = float(marker["y"])
            yaw = float(marker.get("yaw", 0.0))
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("marker {} has invalid x/y/yaw".format(name)) from exc
        if not all(math.isfinite(value) for value in (x, y, yaw)):
            raise ValueError("marker {} has non-finite x/y/yaw".format(name))
        parsed.append((name, region, x, y, yaw))
    return frame_id, size, z, material, parsed


def cube_sdf(name, size, material):
    return """<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry><box><size>{size} {size} {size}</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><box><size>{size} {size} {size}</size></box></geometry>
        <material><script><name>{material}</name></script></material>
      </visual>
    </link>
  </model>
</sdf>
""".format(name=name, size=size, material=material)


def main():
    rospy.init_node("spawn_calibration_cubes")
    config_file = rospy.get_param("~config_file")
    # Preserve direct-script compatibility; full_competition.launch always
    # passes this parameter explicitly and defaults it to false.
    enabled = bool(rospy.get_param("~enabled", True))
    try:
        frame_id, size, z, material, markers = load_config(config_file)
    except (OSError, ValueError, yaml.YAMLError) as exc:
        rospy.logfatal("invalid calibration cube configuration: %s", exc)
        return 2

    rospy.wait_for_service("/gazebo/delete_model")
    rospy.wait_for_service("/gazebo/get_world_properties")
    delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
    world_properties = rospy.ServiceProxy(
        "/gazebo/get_world_properties", GetWorldProperties
    )

    existing_models = set(world_properties().model_names)
    for name, _region, _x, _y, _yaw in markers:
        if name in existing_models:
            response = delete_model(name)
            if not response.success:
                rospy.logerr(
                    "failed to remove stale calibration cube %s: %s",
                    name,
                    response.status_message,
                )
                return 1

    if not enabled:
        rospy.loginfo("camera calibration cubes disabled; stale markers removed")
        return 0

    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    spawn_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)

    for name, region, x, y, yaw in markers:
        pose = Pose(
            position=Point(x=x, y=y, z=z),
            orientation=Quaternion(
                x=0.0,
                y=0.0,
                z=math.sin(yaw / 2.0),
                w=math.cos(yaw / 2.0),
            ),
        )
        response = spawn_model(
            name,
            cube_sdf(name, size, material),
            "",
            pose,
            frame_id,
        )
        if not response.success:
            rospy.logerr("failed to spawn %s: %s", name, response.status_message)
            return 1
        rospy.loginfo(
            "spawned %s region=%s center=(%.3f, %.3f, %.3f)",
            name,
            region,
            x,
            y,
            z,
        )
    rospy.loginfo("spawned %d camera calibration cubes", len(markers))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
