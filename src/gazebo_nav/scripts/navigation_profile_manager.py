#!/usr/bin/env python3
"""Apply named local-navigation profiles through dynamic_reconfigure."""

import threading
import time

from dynamic_reconfigure.client import Client
import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse


class NavigationProfileManager:
    """Lazily connect to move_base and apply a complete named profile."""

    AUTOMATIC = "automatic_navigation"
    CONE_ZONE = "cone_zone_avoidance"

    def __init__(self):
        self._move_base_namespace = rospy.get_param(
            "~move_base_namespace", "/move_base"
        ).rstrip("/")
        self._client_timeout = float(
            rospy.get_param("~client_timeout", 3.0)
        )
        self._initial_apply_timeout = float(
            rospy.get_param("~initial_apply_timeout", 60.0)
        )
        if self._client_timeout <= 0.0:
            raise ValueError("client_timeout must be positive")
        if self._initial_apply_timeout <= 0.0:
            raise ValueError("initial_apply_timeout must be positive")

        self._profiles = {
            self.AUTOMATIC: self._read_profile(self.AUTOMATIC),
            self.CONE_ZONE: self._read_profile(self.CONE_ZONE),
        }
        self._clients = {}
        self._switch_lock = threading.Lock()
        self._active_profile = None

        self._active_profile_pub = rospy.Publisher(
            "~active_profile", String, queue_size=1, latch=True
        )
        self._active_profile_pub.publish(String(data="unapplied"))

        apply_initial_profile = rospy.get_param(
            "~apply_initial_profile", False
        )
        if apply_initial_profile:
            initial_profile = rospy.get_param(
                "~initial_profile", self.AUTOMATIC
            )
            self._apply_initial_profile(initial_profile)

        self._automatic_service = rospy.Service(
            "~use_automatic_navigation",
            Trigger,
            self._use_automatic_navigation,
        )
        self._cone_service = rospy.Service(
            "~use_cone_zone_avoidance",
            Trigger,
            self._use_cone_zone_avoidance,
        )

        if apply_initial_profile:
            rospy.loginfo(
                "navigation profile manager ready with initial profile %s",
                self._active_profile,
            )
        else:
            rospy.loginfo(
                "navigation profile manager ready; profiles are not applied "
                "until a switch service is called"
            )

    def _read_profile(self, profile_name):
        parameter_name = "~profiles/{}".format(profile_name)
        profile = rospy.get_param(parameter_name, None)
        if not isinstance(profile, dict):
            raise ValueError(
                "navigation profile parameter {} must be a dictionary".format(
                    parameter_name
                )
            )
        if not isinstance(profile.get("DWAPlannerROS"), dict):
            raise ValueError(
                "{} must contain DWAPlannerROS parameters".format(
                    parameter_name
                )
            )
        local_costmap = profile.get("local_costmap")
        if not isinstance(local_costmap, dict):
            raise ValueError(
                "{} must contain local_costmap parameters".format(
                    parameter_name
                )
            )
        return profile

    def _server_name(self, relative_name):
        return "{}/{}".format(
            self._move_base_namespace, relative_name.lstrip("/")
        )

    def _get_client(self, relative_name):
        if relative_name not in self._clients:
            server_name = self._server_name(relative_name)
            self._clients[relative_name] = Client(
                server_name, timeout=self._client_timeout
            )
        return self._clients[relative_name]

    @staticmethod
    def _coerce_value(desired_value, current_value):
        """Match ROS dynamic parameter types, including footprint strings."""
        if isinstance(current_value, str) and not isinstance(
            desired_value, str
        ):
            return repr(desired_value)
        if isinstance(current_value, bool):
            return bool(desired_value)
        if isinstance(current_value, int) and not isinstance(
            current_value, bool
        ):
            return int(desired_value)
        if isinstance(current_value, float):
            return float(desired_value)
        return desired_value

    @classmethod
    def _supported_update(cls, desired, current):
        """Return only values exposed by this dynamic_reconfigure server."""
        update = {}
        for key, desired_value in desired.items():
            if key == "groups" or key not in current:
                continue
            if isinstance(desired_value, dict):
                continue
            value = cls._coerce_value(desired_value, current[key])
            if value != current[key]:
                update[key] = value
        return update

    def _profile_steps(self, profile_name):
        profile = self._profiles[profile_name]
        local_costmap = profile["local_costmap"]
        costmap_steps = [
            ("local_costmap", local_costmap),
            (
                "local_costmap/obstacle_layer",
                local_costmap.get("obstacle_layer", {}),
            ),
            (
                "local_costmap/inflation_layer",
                local_costmap.get("inflation_layer", {}),
            ),
        ]
        dwa_step = ("DWAPlannerROS", profile["DWAPlannerROS"])

        # Entering the cone zone lowers planner limits before costmap changes.
        # Returning to automatic navigation restores the map before any faster
        # planner limits are enabled.
        if profile_name == self.CONE_ZONE:
            return [dwa_step] + costmap_steps
        return costmap_steps + [dwa_step]

    def _apply_profile(self, profile_name):
        if profile_name not in self._profiles:
            return False, "unknown navigation profile: {}".format(
                profile_name
            )

        applied = []
        try:
            for relative_name, desired in self._profile_steps(profile_name):
                client = self._get_client(relative_name)
                current = client.get_configuration(
                    timeout=self._client_timeout
                )
                if current is None:
                    raise RuntimeError(
                        "no configuration received from {}".format(
                            self._server_name(relative_name)
                        )
                    )
                update = self._supported_update(desired, current)
                if not update:
                    continue
                previous = {key: current[key] for key in update}
                client.update_configuration(update)
                applied.append((relative_name, previous))
        except Exception as exc:
            rospy.logerr(
                "failed to apply navigation profile %s: %s",
                profile_name,
                exc,
            )
            for relative_name, previous in reversed(applied):
                try:
                    self._get_client(relative_name).update_configuration(
                        previous
                    )
                except Exception as rollback_exc:
                    rospy.logerr(
                        "failed to roll back %s: %s",
                        self._server_name(relative_name),
                        rollback_exc,
                    )
            return False, "failed to apply {}: {}".format(profile_name, exc)

        self._active_profile = profile_name
        rospy.set_param("~selected_profile", profile_name)
        self._active_profile_pub.publish(String(data=profile_name))
        rospy.loginfo("navigation profile applied: %s", profile_name)
        return True, "navigation profile applied: {}".format(profile_name)

    def _apply_initial_profile(self, profile_name):
        if profile_name not in self._profiles:
            raise ValueError(
                "unknown initial navigation profile: {}".format(profile_name)
            )

        deadline = time.monotonic() + self._initial_apply_timeout
        last_message = ""
        while not rospy.is_shutdown():
            success, last_message = self._apply_profile(profile_name)
            if success:
                return
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            rospy.logwarn(
                "initial navigation profile is not ready; retrying in "
                "0.5 s"
            )
            rospy.sleep(min(0.5, remaining))

        raise RuntimeError(
            "initial navigation profile failed within {:.1f}s: {}".format(
                self._initial_apply_timeout, last_message
            )
        )

    def _switch(self, profile_name):
        if not self._switch_lock.acquire(False):
            return TriggerResponse(
                success=False,
                message="another navigation profile switch is in progress",
            )
        try:
            success, message = self._apply_profile(profile_name)
            return TriggerResponse(success=success, message=message)
        finally:
            self._switch_lock.release()

    def _use_automatic_navigation(self, _request):
        return self._switch(self.AUTOMATIC)

    def _use_cone_zone_avoidance(self, _request):
        return self._switch(self.CONE_ZONE)


def main():
    rospy.init_node("navigation_profile_manager")
    NavigationProfileManager()
    rospy.spin()


if __name__ == "__main__":
    main()
