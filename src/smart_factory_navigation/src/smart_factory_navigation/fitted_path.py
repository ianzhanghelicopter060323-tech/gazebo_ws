"""Load the offline-fitted reference path and sequential execution points."""

from dataclasses import dataclass
import math


class PathConfigError(ValueError):
    pass


@dataclass(frozen=True)
class PathPoint:
    s: float
    x: float
    y: float
    yaw: float


def _finite_float(value, label):
    try:
        result = float(value)
    except (TypeError, ValueError):
        raise PathConfigError("{} must be numeric".format(label))
    if not math.isfinite(result):
        raise PathConfigError("{} must be finite".format(label))
    return result


def _path_points(raw_points, collection_label):
    if not isinstance(raw_points, list) or len(raw_points) < 2:
        raise PathConfigError(
            "{} must contain at least two points".format(collection_label)
        )
    points = []
    previous_s = None
    for index, raw in enumerate(raw_points):
        if not isinstance(raw, dict):
            raise PathConfigError(
                "{} point {} must be a mapping".format(
                    collection_label, index
                )
            )
        point = PathPoint(
            s=_finite_float(raw.get("s"), "point s"),
            x=_finite_float(raw.get("x"), "point x"),
            y=_finite_float(raw.get("y"), "point y"),
            yaw=_finite_float(raw.get("yaw"), "point yaw"),
        )
        if previous_s is not None and point.s <= previous_s:
            raise PathConfigError(
                "{} s values must increase strictly".format(collection_label)
            )
        previous_s = point.s
        points.append(point)
    if abs(points[0].s) > 1.0e-6:
        raise PathConfigError("{} must start at s=0".format(collection_label))
    return tuple(points)


class FittedPath:
    """Validated data needed by sequential fitted-waypoint execution."""

    def __init__(self, frame_id, points, execution_waypoints, final_goal):
        self.frame_id = frame_id
        self.points = tuple(points)
        self.execution_waypoints = tuple(execution_waypoints)
        self.final_goal = tuple(final_goal)

    @classmethod
    def from_config(cls, config):
        if not isinstance(config, dict) or not config.get("configured", False):
            raise PathConfigError("fitted path is not configured")
        frame_id = config.get("frame_id", "map")
        if not isinstance(frame_id, str) or not frame_id.strip():
            raise PathConfigError("fitted path frame_id must not be empty")

        points = _path_points(config.get("points"), "fitted path")
        execution_waypoints = _path_points(
            config.get("execution_waypoints"),
            "fitted execution waypoints",
        )
        if math.hypot(
            execution_waypoints[0].x - points[0].x,
            execution_waypoints[0].y - points[0].y,
        ) > 1.0e-6:
            raise PathConfigError(
                "fitted execution waypoints must start at the fitted path start"
            )
        if (
            abs(execution_waypoints[-1].s - points[-1].s) > 1.0e-6
            or math.hypot(
                execution_waypoints[-1].x - points[-1].x,
                execution_waypoints[-1].y - points[-1].y,
            )
            > 1.0e-6
        ):
            raise PathConfigError(
                "fitted execution waypoints must end at the fitted path end"
            )

        raw_final = config.get("final_goal")
        if not isinstance(raw_final, dict):
            raise PathConfigError("fitted path final_goal must be configured")
        final_goal = (
            _finite_float(raw_final.get("x"), "final_goal x"),
            _finite_float(raw_final.get("y"), "final_goal y"),
            _finite_float(raw_final.get("yaw"), "final_goal yaw"),
        )
        if math.hypot(
            final_goal[0] - points[-1].x,
            final_goal[1] - points[-1].y,
        ) > 1.0e-6:
            raise PathConfigError(
                "fitted path final_goal position must match the path end"
            )
        return cls(
            frame_id.strip(), points, execution_waypoints, final_goal
        )
