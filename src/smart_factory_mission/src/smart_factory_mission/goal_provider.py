"""Navigation goal providers.

The current provider reads a development-only staging pose. It deliberately
refuses to return a goal until the pose is explicitly marked as configured.
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

    def get_pickup_staging_goal(self, _context):
        configured = rospy.get_param(
            self.namespace + "/configured", False
        )
        if not configured:
            raise GoalUnavailable(
                "pickup staging goal is not configured; fill "
                "pickup_staging_dev.yaml and set configured: true"
            )

        required = ("x", "y", "yaw")
        missing = [
            key
            for key in required
            if not rospy.has_param(self.namespace + "/" + key)
        ]
        if missing:
            raise GoalUnavailable(
                "pickup staging goal is missing: {}".format(", ".join(missing))
            )

        frame_id = rospy.get_param(self.namespace + "/frame_id", "map")
        x = float(rospy.get_param(self.namespace + "/x"))
        y = float(rospy.get_param(self.namespace + "/y"))
        yaw = float(rospy.get_param(self.namespace + "/yaw"))

        if not all(math.isfinite(value) for value in (x, y, yaw)):
            raise GoalUnavailable("pickup staging goal contains a non-finite value")

        goal = PoseStamped()
        goal.header.frame_id = frame_id
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        return goal


def create_goal_provider(provider_type):
    if provider_type == "development":
        return DevelopmentGoalProvider()
    if provider_type == "vision":
        raise GoalUnavailable(
            "vision goal provider is reserved but not implemented yet"
        )
    raise GoalUnavailable("unknown goal provider: {}".format(provider_type))
