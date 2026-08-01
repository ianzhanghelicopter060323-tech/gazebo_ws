#!/usr/bin/env python3
"""Headless mission monitor using Gazebo ground truth and the occupancy map."""

import argparse
import math
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET

from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rospy
from std_msgs.msg import Float64

from smart_factory_interfaces.msg import ExecuteTaskActionResult


class NavigationSafetyMonitor:
    def __init__(
        self,
        model_name,
        safety_radius,
        sample_period,
        footprint_bounds,
        map_cell_padding,
        wall_polygons,
    ):
        self.model_name = model_name
        self.safety_radius = safety_radius
        self.sample_period = sample_period
        self.result = None
        self.goal_count = 0
        self.pose_samples = 0
        self.progress_samples = 0
        self.progress_violations = 0
        self.maximum_progress = 0.0
        self._last_progress = None
        self._last_sample_time = None
        self._last_position = None
        self.travel_distance = 0.0
        self.minimum_center_clearance = float("inf")
        self.minimum_clearance_position = None
        self.minimum_footprint_margin = float("inf")
        self.minimum_footprint_pose = None
        self.minimum_wall_margin = float("inf")
        self.minimum_wall_pose = None
        self.minimum_wall_name = ""
        self.wall_contact_samples = 0
        self._footprint_bounds = footprint_bounds
        self._map_cell_padding = map_cell_padding
        self._wall_polygons = wall_polygons

        map_message = rospy.wait_for_message("/map", OccupancyGrid, timeout=30.0)
        self._occupied_world = self._occupied_cell_centers(map_message)
        rospy.Subscriber(
            "/gazebo/model_states", ModelStates, self._model_states_callback, queue_size=1
        )
        rospy.Subscriber(
            "/sim_task/path_progress", Float64, self._progress_callback, queue_size=20
        )
        rospy.Subscriber(
            "/sim_task/execute/result",
            ExecuteTaskActionResult,
            self._result_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            "/move_base/goal", rospy.AnyMsg, self._goal_callback, queue_size=20
        )

    @staticmethod
    def _occupied_cell_centers(message):
        data = np.asarray(message.data, dtype=np.int16).reshape(
            message.info.height, message.info.width
        )
        rows, columns = np.nonzero(data >= 50)
        local_x = (columns.astype(float) + 0.5) * message.info.resolution
        local_y = (rows.astype(float) + 0.5) * message.info.resolution
        orientation = message.info.origin.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        world_x = (
            message.info.origin.position.x + cosine * local_x - sine * local_y
        )
        world_y = (
            message.info.origin.position.y + sine * local_x + cosine * local_y
        )
        return np.column_stack((world_x, world_y))

    def _model_states_callback(self, message):
        try:
            index = message.name.index(self.model_name)
        except ValueError:
            return
        now = rospy.Time.now().to_sec()
        if (
            self._last_sample_time is not None
            and now - self._last_sample_time < self.sample_period
        ):
            return
        self._last_sample_time = now
        pose = message.pose[index]
        position = np.asarray([pose.position.x, pose.position.y])
        if self._last_position is not None:
            self.travel_distance += float(np.linalg.norm(position - self._last_position))
        self._last_position = position
        difference = self._occupied_world - position
        clearance = float(np.sqrt(np.min(np.einsum("ij,ij->i", difference, difference))))
        if clearance < self.minimum_center_clearance:
            self.minimum_center_clearance = clearance
            self.minimum_clearance_position = tuple(position)

        orientation = pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        # Transform occupied cell centers into base_link coordinates, then find
        # their distance to the measured base-link STL XY bounding rectangle.
        local_x = cosine * difference[:, 0] + sine * difference[:, 1]
        local_y = -sine * difference[:, 0] + cosine * difference[:, 1]
        x_min, x_max, y_min, y_max = self._footprint_bounds
        distance_x = np.maximum(np.maximum(x_min - local_x, 0.0), local_x - x_max)
        distance_y = np.maximum(np.maximum(y_min - local_y, 0.0), local_y - y_max)
        footprint_margin = float(
            np.sqrt(np.min(distance_x * distance_x + distance_y * distance_y))
            - self._map_cell_padding
        )
        if footprint_margin < self.minimum_footprint_margin:
            self.minimum_footprint_margin = footprint_margin
            self.minimum_footprint_pose = (position[0], position[1], yaw)

        robot_polygon = rectangle_polygon(
            position[0],
            position[1],
            yaw,
            x_min,
            x_max,
            y_min,
            y_max,
        )
        wall_name, wall_margin = min(
            (
                (name, polygon_distance(robot_polygon, polygon))
                for name, polygon in self._wall_polygons
            ),
            key=lambda item: item[1],
        )
        if wall_margin <= 1.0e-6:
            self.wall_contact_samples += 1
        if wall_margin < self.minimum_wall_margin:
            self.minimum_wall_margin = wall_margin
            self.minimum_wall_pose = (position[0], position[1], yaw)
            self.minimum_wall_name = wall_name
        self.pose_samples += 1

    def _progress_callback(self, message):
        progress = float(message.data)
        if self._last_progress is not None and progress + 1.0e-6 < self._last_progress:
            self.progress_violations += 1
        self._last_progress = progress
        self.maximum_progress = max(self.maximum_progress, progress)
        self.progress_samples += 1

    def _result_callback(self, message):
        self.result = message.result

    def _goal_callback(self, _message):
        self.goal_count += 1

    @property
    def conservative_margin(self):
        return self.minimum_center_clearance - self.safety_radius

    def print_summary(self):
        success = self.result.success if self.result is not None else False
        error_code = self.result.error_code if self.result is not None else -1
        message = self.result.message if self.result is not None else "no task result"
        print("MONITOR_RESULT success={} error_code={} message={}".format(
            success, error_code, message
        ))
        print(
            "MONITOR_PATH pose_samples={} travel_distance={:.3f} "
            "goal_count={} progress_samples={} max_progress={:.3f} "
            "progress_violations={}".format(
                self.pose_samples,
                self.travel_distance,
                self.goal_count,
                self.progress_samples,
                self.maximum_progress,
                self.progress_violations,
            )
        )
        print(
            "MONITOR_WALL physical_margin={:.6f} wall={} "
            "at=({:.3f},{:.3f},yaw={:.3f}) contact_samples={}".format(
                self.minimum_wall_margin,
                self.minimum_wall_name,
                self.minimum_wall_pose[0],
                self.minimum_wall_pose[1],
                self.minimum_wall_pose[2],
                self.wall_contact_samples,
            )
        )
        print(
            "MONITOR_FOOTPRINT map_margin={:.3f} at=({:.3f},{:.3f},yaw={:.3f}) "
            "bounds=({:.3f},{:.3f},{:.3f},{:.3f}) cell_padding={:.3f}".format(
                self.minimum_footprint_margin,
                self.minimum_footprint_pose[0],
                self.minimum_footprint_pose[1],
                self.minimum_footprint_pose[2],
                self._footprint_bounds[0],
                self._footprint_bounds[1],
                self._footprint_bounds[2],
                self._footprint_bounds[3],
                self._map_cell_padding,
            )
        )
        print(
            "MONITOR_CLEARANCE center={:.3f} safety_radius={:.3f} "
            "conservative_margin={:.3f} at=({:.3f},{:.3f})".format(
                self.minimum_center_clearance,
                self.safety_radius,
                self.conservative_margin,
                self.minimum_clearance_position[0],
                self.minimum_clearance_position[1],
            )
        )


def rectangle_polygon(center_x, center_y, yaw, x_min, x_max, y_min, y_max):
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    result = []
    for local_x, local_y in (
        (x_min, y_min),
        (x_max, y_min),
        (x_max, y_max),
        (x_min, y_max),
    ):
        result.append(
            (
                center_x + cosine * local_x - sine * local_y,
                center_y + sine * local_x + cosine * local_y,
            )
        )
    return result


def _cross(first, second, third):
    return (
        (second[0] - first[0]) * (third[1] - first[1])
        - (second[1] - first[1]) * (third[0] - first[0])
    )


def _segments_intersect(first, second, third, fourth):
    a = _cross(first, second, third)
    b = _cross(first, second, fourth)
    c = _cross(third, fourth, first)
    d = _cross(third, fourth, second)
    epsilon = 1.0e-12
    if a * b < -epsilon and c * d < -epsilon:
        return True

    def on_segment(point, start, end):
        return (
            min(start[0], end[0]) - epsilon
            <= point[0]
            <= max(start[0], end[0]) + epsilon
            and min(start[1], end[1]) - epsilon
            <= point[1]
            <= max(start[1], end[1]) + epsilon
        )

    return (
        (abs(a) <= epsilon and on_segment(third, first, second))
        or (abs(b) <= epsilon and on_segment(fourth, first, second))
        or (abs(c) <= epsilon and on_segment(first, third, fourth))
        or (abs(d) <= epsilon and on_segment(second, third, fourth))
    )


def _point_segment_distance(point, start, end):
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    denominator = dx * dx + dy * dy
    if denominator <= 1.0e-18:
        return math.hypot(point[0] - start[0], point[1] - start[1])
    fraction = (
        (point[0] - start[0]) * dx + (point[1] - start[1]) * dy
    ) / denominator
    fraction = max(0.0, min(1.0, fraction))
    return math.hypot(
        point[0] - (start[0] + fraction * dx),
        point[1] - (start[1] + fraction * dy),
    )


def polygon_distance(first, second):
    first_edges = list(zip(first, first[1:] + first[:1]))
    second_edges = list(zip(second, second[1:] + second[:1]))
    if any(
        _segments_intersect(a, b, c, d)
        for a, b in first_edges
        for c, d in second_edges
    ):
        return 0.0
    if _point_in_convex_polygon(first[0], second) or _point_in_convex_polygon(
        second[0], first
    ):
        return 0.0
    distances = []
    for point in first:
        distances.extend(
            _point_segment_distance(point, start, end)
            for start, end in second_edges
        )
    for point in second:
        distances.extend(
            _point_segment_distance(point, start, end)
            for start, end in first_edges
        )
    return min(distances)


def _point_in_convex_polygon(point, polygon):
    crosses = [
        _cross(start, end, point)
        for start, end in zip(polygon, polygon[1:] + polygon[:1])
    ]
    return all(value >= -1.0e-12 for value in crosses) or all(
        value <= 1.0e-12 for value in crosses
    )


def _pose(text):
    values = [float(value) for value in (text or "0 0 0 0 0 0").split()]
    return values[0], values[1], values[5]


def _compose(parent, child):
    cosine = math.cos(parent[2])
    sine = math.sin(parent[2])
    return (
        parent[0] + cosine * child[0] - sine * child[1],
        parent[1] + sine * child[0] + cosine * child[1],
        parent[2] + child[2],
    )


def load_wall_polygons(world_file):
    root = ET.parse(str(world_file)).getroot()
    model = next(model for model in root.iter("model") if model.get("name") == "math")
    model_pose = _pose(model.findtext("pose"))
    polygons = []
    for link in model.findall("link"):
        collision = link.find("collision")
        size_text = collision.findtext("geometry/box/size")
        if size_text is None:
            continue
        size = [float(value) for value in size_text.split()]
        world_pose = _compose(
            _compose(model_pose, _pose(link.findtext("pose"))),
            _pose(collision.findtext("pose")),
        )
        polygons.append(
            (
                link.get("name"),
                rectangle_polygon(
                    world_pose[0],
                    world_pose[1],
                    world_pose[2],
                    -size[0] / 2.0,
                    size[0] / 2.0,
                    -size[1] / 2.0,
                    size[1] / 2.0,
                ),
            )
        )
    return polygons


def main():
    workspace = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="car3")
    parser.add_argument("--safety-radius", type=float, default=0.17)
    parser.add_argument("--required-margin", type=float, default=0.03)
    parser.add_argument("--sample-period", type=float, default=0.05)
    parser.add_argument("--footprint-x-min", type=float, default=-0.164)
    parser.add_argument("--footprint-x-max", type=float, default=0.190)
    parser.add_argument("--footprint-y-min", type=float, default=-0.120)
    parser.add_argument("--footprint-y-max", type=float, default=0.120)
    parser.add_argument("--map-cell-padding", type=float, default=0.036)
    parser.add_argument(
        "--world-file",
        type=Path,
        default=workspace / "src/car3/world/math.world",
    )
    parser.add_argument("--timeout", type=float, default=300.0)
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    rospy.init_node("monitor_navigation_safety", anonymous=True)
    monitor = NavigationSafetyMonitor(
        args.model,
        args.safety_radius,
        args.sample_period,
        (
            args.footprint_x_min,
            args.footprint_x_max,
            args.footprint_y_min,
            args.footprint_y_max,
        ),
        args.map_cell_padding,
        load_wall_polygons(args.world_file),
    )
    deadline = time.monotonic() + args.timeout
    while not rospy.is_shutdown() and monitor.result is None:
        if time.monotonic() >= deadline:
            break
        time.sleep(0.1)
    if monitor.pose_samples == 0 or monitor.minimum_clearance_position is None:
        print("MONITOR_ERROR no Gazebo pose samples")
        return 3
    monitor.print_summary()
    if monitor.result is None:
        return 2
    if not monitor.result.success:
        return 1
    if monitor.progress_violations:
        return 4
    if monitor.minimum_wall_margin < args.required_margin:
        return 6
    return 0


if __name__ == "__main__":
    sys.exit(main())
