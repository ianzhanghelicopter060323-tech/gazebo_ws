"""Fixed-standoff base planning for the calibrated arm grasp pose."""

import math

from geometry_msgs.msg import PoseStamped


class FixedGraspPlanner:
    """Place a visually located cube at one fixed TCP point in the base frame.

    The RGB-D measurement is corrected to the cube center with one signed,
    empirically calibrated forward offset.  The base is then placed so that
    the center coincides with the calibrated TCP position for the fixed grasp
    posture.
    """

    def __init__(
        self,
        target_forward,
        target_lateral=0.0,
        depth_to_center_forward=0.0,
        frame_id="map",
    ):
        values = (target_forward, target_lateral, depth_to_center_forward)
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("fixed-grasp offsets must be finite")
        if float(target_forward) <= 0.0:
            raise ValueError("target_forward must be positive")
        if not isinstance(frame_id, str) or not frame_id.strip():
            raise ValueError("frame_id must not be empty")
        self.target_forward = float(target_forward)
        self.target_lateral = float(target_lateral)
        self.depth_to_center_forward = float(depth_to_center_forward)
        self.frame_id = frame_id.strip()

    def goal_from_surface(self, surface_x, surface_y, base_yaw, stamp=None):
        values = (surface_x, surface_y, base_yaw)
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("surface point and yaw must be finite")
        cosine = math.cos(float(base_yaw))
        sine = math.sin(float(base_yaw))

        center_x = (
            float(surface_x) + self.depth_to_center_forward * cosine
        )
        center_y = (
            float(surface_y) + self.depth_to_center_forward * sine
        )

        # Rotate the fixed TCP target from base coordinates into map axes.
        target_map_x = (
            self.target_forward * cosine - self.target_lateral * sine
        )
        target_map_y = (
            self.target_forward * sine + self.target_lateral * cosine
        )

        goal = PoseStamped()
        goal.header.frame_id = self.frame_id
        if stamp is not None:
            goal.header.stamp = stamp
        goal.pose.position.x = center_x - target_map_x
        goal.pose.position.y = center_y - target_map_y
        goal.pose.orientation.z = math.sin(float(base_yaw) / 2.0)
        goal.pose.orientation.w = math.cos(float(base_yaw) / 2.0)
        return goal

    @staticmethod
    def correction_distance(current_x, current_y, goal):
        return math.hypot(
            float(goal.pose.position.x) - float(current_x),
            float(goal.pose.position.y) - float(current_y),
        )
