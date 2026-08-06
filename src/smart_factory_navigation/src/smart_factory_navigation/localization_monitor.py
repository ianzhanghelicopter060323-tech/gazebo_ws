"""AMCL and TF readiness checks shared by mission navigation behaviors."""

import math
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
import rospy
from std_srvs.srv import Empty
import tf2_ros


class LocalizationMonitor:
    """Own localization subscriptions and expose validated robot poses."""

    def __init__(self):
        self.map_frame = rospy.get_param("~localization/map_frame", "map")
        self.base_frame = rospy.get_param(
            "~localization/base_frame", "base_footprint"
        )
        self._amcl_topic = rospy.get_param(
            "~localization/amcl_topic", "/amcl_pose"
        )
        self._wait_timeout = rospy.get_param(
            "~localization/wait_timeout", 10.0
        )
        self._max_pose_age = rospy.get_param(
            "~localization/max_pose_age", 1.0
        )
        self._max_tf_age = rospy.get_param(
            "~localization/max_tf_age", 1.0
        )
        self._allow_stale_pose_with_fresh_tf = rospy.get_param(
            "~localization/allow_stale_pose_with_fresh_tf", True
        )
        self._max_xy_variance = rospy.get_param(
            "~localization/max_xy_variance", 0.0025
        )
        self._max_yaw_variance = rospy.get_param(
            "~localization/max_yaw_variance", 0.0025
        )
        self._require_tf = rospy.get_param(
            "~localization/require_tf", True
        )
        self._nomotion_update_service = str(
            rospy.get_param(
                "~localization/nomotion_update_service",
                "/request_nomotion_update",
            )
        ).strip()
        self._nomotion_update_period = float(
            rospy.get_param(
                "~localization/nomotion_update_period", 1.0
            )
        )
        if not (
            math.isfinite(self._nomotion_update_period)
            and self._nomotion_update_period > 0.0
        ):
            raise ValueError(
                "localization/nomotion_update_period must be positive"
            )

        self._latest_amcl = None
        self._nomotion_update = (
            rospy.ServiceProxy(self._nomotion_update_service, Empty)
            if self._nomotion_update_service
            else None
        )
        self._tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._amcl_sub = rospy.Subscriber(
            self._amcl_topic,
            PoseWithCovarianceStamped,
            self._amcl_callback,
            queue_size=1,
        )

    def _amcl_callback(self, message):
        self._latest_amcl = message

    def _map_to_base_tf_age(self):
        try:
            transform = self._tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rospy.Time(0),
                rospy.Duration(0.1),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            return None

        stamp = transform.header.stamp
        if stamp == rospy.Time():
            return None
        return (rospy.Time.now() - stamp).to_sec()

    def _tf_age_is_ready(self, age):
        if age is None or age < 0.0:
            return False
        if self._max_tf_age > 0.0 and age > self._max_tf_age:
            return False
        return True

    def pose_is_ready(self):
        if self._latest_amcl is None:
            return False

        stamp = self._latest_amcl.header.stamp
        if stamp == rospy.Time():
            return False
        age = (rospy.Time.now() - stamp).to_sec()
        if age < 0.0:
            return False
        pose_is_fresh = (
            self._max_pose_age <= 0.0 or age <= self._max_pose_age
        )

        covariance = self._latest_amcl.pose.covariance
        if covariance[0] > self._max_xy_variance:
            return False
        if covariance[7] > self._max_xy_variance:
            return False
        if covariance[35] > self._max_yaw_variance:
            return False

        needs_tf = (
            self._require_tf
            or (
                not pose_is_fresh
                and self._allow_stale_pose_with_fresh_tf
            )
        )
        tf_age = self._map_to_base_tf_age() if needs_tf else None
        tf_is_fresh = self._tf_age_is_ready(tf_age)

        if self._require_tf and not tf_is_fresh:
            return False

        if not pose_is_fresh:
            if not (
                self._allow_stale_pose_with_fresh_tf
                and tf_is_fresh
            ):
                return False
            rospy.loginfo(
                "AMCL pose is %.3fs old; accepting localization because "
                "%s -> %s TF is %.3fs old",
                age,
                self.map_frame,
                self.base_frame,
                tf_age,
            )

        return True

    def wait_until_ready(self, preempt_requested):
        """Wait for valid localization, preemption, shutdown, or timeout."""
        deadline = time.monotonic() + float(self._wait_timeout)
        next_nomotion_update = 0.0
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if preempt_requested():
                return False
            if self.pose_is_ready():
                return True
            now = time.monotonic()
            if self._nomotion_update is not None and now >= next_nomotion_update:
                try:
                    rospy.wait_for_service(
                        self._nomotion_update_service, timeout=0.2
                    )
                    self._nomotion_update()
                except (rospy.ROSException, rospy.ServiceException) as exc:
                    rospy.logwarn_throttle(
                        2.0,
                        "cannot request AMCL no-motion update from %s: %s",
                        self._nomotion_update_service,
                        exc,
                    )
                next_nomotion_update = now + self._nomotion_update_period
            rospy.sleep(0.1)
        return False

    @staticmethod
    def _quaternion_yaw(quaternion):
        return math.atan2(
            2.0
            * (
                quaternion.w * quaternion.z
                + quaternion.x * quaternion.y
            ),
            1.0
            - 2.0
            * (
                quaternion.y * quaternion.y
                + quaternion.z * quaternion.z
            ),
        )

    def localized_pose(self, frame_id):
        try:
            transform = self._tf_buffer.lookup_transform(
                frame_id,
                self.base_frame,
                rospy.Time(0),
                rospy.Duration(0.05),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:
            rospy.logwarn_throttle(
                2.0,
                "cannot read localized robot position in %s: %s",
                frame_id,
                exc,
            )
            return None
        return (
            transform.transform.translation.x,
            transform.transform.translation.y,
            self._quaternion_yaw(transform.transform.rotation),
        )

    def localized_xy(self, frame_id):
        localized = self.localized_pose(frame_id)
        return None if localized is None else localized[:2]

    def pose_is_within_radius(self, pose, radius):
        frame_id = pose.header.frame_id or self.map_frame
        localized = self.localized_xy(frame_id)
        if localized is None:
            return False
        return math.hypot(
            pose.pose.position.x - localized[0],
            pose.pose.position.y - localized[1],
        ) <= radius
