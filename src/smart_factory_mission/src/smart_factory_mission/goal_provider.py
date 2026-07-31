"""Navigation goal providers.

The current provider reads a development-only staging route. It deliberately
refuses to return goals until the route is explicitly marked as configured.
The final competition provider will derive the goal from runtime perception.
"""

import math

import rospy
from geometry_msgs.msg import PoseStamped


class GoalUnavailable(RuntimeError):
    pass


class DevelopmentGoalProvider:
    def __init__(self, namespace="~pickup_staging"):
        self.namespace = namespace.rstrip("/")

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

    def get_pickup_staging_goal(self, context):
        """Return the final staging pose for callers using the old API."""
        return self.get_pickup_staging_goals(context)[-1]


def create_goal_provider(provider_type):
    if provider_type == "development":
        return DevelopmentGoalProvider()
    if provider_type == "vision":
        raise GoalUnavailable(
            "vision goal provider is reserved but not implemented yet"
        )
    raise GoalUnavailable("unknown goal provider: {}".format(provider_type))
