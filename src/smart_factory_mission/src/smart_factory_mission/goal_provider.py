"""Navigation goal providers.

The current provider reads a development-only staging route. It deliberately
refuses to return goals until the route is explicitly marked as configured.
The final competition provider will derive the goal from runtime perception.
"""

from dataclasses import dataclass
import math

import rospy
from geometry_msgs.msg import PoseStamped


class GoalUnavailable(RuntimeError):
    pass


@dataclass(frozen=True)
class DeliveryDestination:
    """Shared cone-entry alignment pose and one task-selected workshop."""

    name: str
    pose: PoseStamped
    entry_pose: PoseStamped
    entry_position_tolerance: float = 0.0
    entry_yaw_tolerance: float = 0.0
    position_tolerance: float = 0.0
    yaw_tolerance: float = 0.0


class DevelopmentGoalProvider:
    def __init__(
        self,
        namespace="~pickup_staging",
        delivery_namespace="~delivery",
    ):
        self.namespace = namespace.rstrip("/")
        self.delivery_namespace = delivery_namespace.rstrip("/")

    def get_pickup_staging_goals(self, _context):
        configured = rospy.get_param(
            self.namespace + "/configured", False
        )
        if not configured:
            raise GoalUnavailable(
                "pickup staging route is not configured; fill "
                "pickup_staging_dev.yaml and set configured: true"
            )

        frame_id = rospy.get_param(self.namespace + "/frame_id", "map")
        if not isinstance(frame_id, str) or not frame_id.strip():
            raise GoalUnavailable(
                "pickup staging route frame_id must not be empty"
            )

        waypoints = rospy.get_param(self.namespace + "/waypoints", [])
        if not isinstance(waypoints, list) or not waypoints:
            raise GoalUnavailable(
                "pickup staging route must contain at least one waypoint"
            )

        goals = []
        required = ("x", "y", "yaw")
        for index, waypoint in enumerate(waypoints):
            waypoint_number = index + 1
            if not isinstance(waypoint, dict):
                raise GoalUnavailable(
                    "pickup staging waypoint {} must be a mapping".format(
                        waypoint_number
                    )
                )

            missing = [key for key in required if key not in waypoint]
            if missing:
                raise GoalUnavailable(
                    "pickup staging waypoint {} is missing: {}".format(
                        waypoint_number, ", ".join(missing)
                    )
                )

            try:
                x = float(waypoint["x"])
                y = float(waypoint["y"])
                yaw = float(waypoint["yaw"])
            except (TypeError, ValueError):
                raise GoalUnavailable(
                    "pickup staging waypoint {} contains a non-numeric value".format(
                        waypoint_number
                    )
                )

            if not all(math.isfinite(value) for value in (x, y, yaw)):
                raise GoalUnavailable(
                    "pickup staging waypoint {} contains a non-finite value".format(
                        waypoint_number
                    )
                )

            goal = PoseStamped()
            goal.header.frame_id = frame_id.strip()
            goal.header.stamp = rospy.Time.now()
            goal.pose.position.x = x
            goal.pose.position.y = y
            goal.pose.position.z = 0.0
            goal.pose.orientation.z = math.sin(yaw / 2.0)
            goal.pose.orientation.w = math.cos(yaw / 2.0)
            goals.append(goal)

        return goals

    def get_delivery_destination(self, context):
        """Return the shared entry pose and task-selected workshop pose."""
        configured = rospy.get_param(
            self.delivery_namespace + "/configured", False
        )
        if not configured:
            raise GoalUnavailable("delivery workshop goals are not configured")

        frame_id = rospy.get_param(
            self.delivery_namespace + "/frame_id", "map"
        )
        if not isinstance(frame_id, str) or not frame_id.strip():
            raise GoalUnavailable("delivery frame_id must not be empty")

        entry = rospy.get_param(
            self.delivery_namespace + "/entry_pose", None
        )
        entry_pose = self._delivery_pose(
            entry, frame_id.strip(), "cone entry pose"
        )
        entry_position_tolerance, entry_yaw_tolerance = (
            self._delivery_tolerances(entry, "cone entry pose")
        )

        destinations = rospy.get_param(
            self.delivery_namespace + "/destinations", []
        )
        if not isinstance(destinations, list):
            raise GoalUnavailable("delivery destinations must be a list")

        matches = [
            destination
            for destination in destinations
            if isinstance(destination, dict)
            and destination.get("target_class") == context.target_class
        ]
        if len(matches) != 1:
            raise GoalUnavailable(
                "target_class {} must map to exactly one delivery workshop".format(
                    context.target_class
                )
            )

        destination = matches[0]
        missing = [key for key in ("name",) if key not in destination]
        if missing:
            raise GoalUnavailable(
                "delivery destination for target_class {} is missing: {}".format(
                    context.target_class, ", ".join(missing)
                )
            )
        name = destination["name"]
        if not isinstance(name, str) or not name.strip():
            raise GoalUnavailable("delivery destination name must not be empty")
        pose = self._delivery_pose(
            destination, frame_id.strip(), "delivery destination"
        )
        position_tolerance, yaw_tolerance = self._delivery_tolerances(
            destination, "delivery destination"
        )
        return DeliveryDestination(
            name=name.strip(),
            pose=pose,
            entry_pose=entry_pose,
            entry_position_tolerance=entry_position_tolerance,
            entry_yaw_tolerance=entry_yaw_tolerance,
            position_tolerance=position_tolerance,
            yaw_tolerance=yaw_tolerance,
        )

    @staticmethod
    def _delivery_tolerances(raw, label):
        try:
            position_tolerance = float(raw["position_tolerance"])
            yaw_tolerance_deg = float(raw["yaw_tolerance_deg"])
        except (KeyError, TypeError, ValueError):
            raise GoalUnavailable(
                "{} requires numeric position_tolerance and "
                "yaw_tolerance_deg".format(label)
            )
        if not (
            math.isfinite(position_tolerance) and position_tolerance > 0.0
        ):
            raise GoalUnavailable(
                "{} position_tolerance must be positive".format(label)
            )
        if not (
            math.isfinite(yaw_tolerance_deg)
            and 0.0 < yaw_tolerance_deg <= 180.0
        ):
            raise GoalUnavailable(
                "{} yaw_tolerance_deg must be in (0, 180]".format(label)
            )
        return position_tolerance, math.radians(yaw_tolerance_deg)

    @staticmethod
    def _delivery_pose(raw, frame_id, label):
        if not isinstance(raw, dict):
            raise GoalUnavailable("{} must be a mapping".format(label))
        missing = [key for key in ("x", "y", "yaw") if key not in raw]
        if missing:
            raise GoalUnavailable(
                "{} is missing: {}".format(label, ", ".join(missing))
            )
        try:
            x = float(raw["x"])
            y = float(raw["y"])
            yaw = float(raw["yaw"])
        except (TypeError, ValueError):
            raise GoalUnavailable("{} contains a non-numeric value".format(label))
        if not all(math.isfinite(value) for value in (x, y, yaw)):
            raise GoalUnavailable("{} contains a non-finite value".format(label))

        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose


def create_goal_provider(provider_type):
    if provider_type == "development":
        return DevelopmentGoalProvider()
    if provider_type == "vision":
        raise GoalUnavailable(
            "vision goal provider is reserved but not implemented yet"
        )
    raise GoalUnavailable("unknown goal provider: {}".format(provider_type))
