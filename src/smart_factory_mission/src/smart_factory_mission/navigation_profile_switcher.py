"""Waypoint-boundary navigation profile selection."""

import rospy
from std_srvs.srv import Trigger


class NavigationProfileSwitcher:
    """Select a profile before a waypoint without changing profile values."""

    AUTOMATIC = "automatic_navigation"
    CONE_ZONE = "cone_zone_avoidance"

    @classmethod
    def from_ros_params(cls):
        prefix = "~navigation/profile_switching"
        return cls(
            enabled=rospy.get_param(prefix + "/enabled", False),
            cone_start_waypoint=rospy.get_param(
                prefix + "/cone_zone_start_waypoint", 0
            ),
            cone_end_waypoint=rospy.get_param(
                prefix + "/cone_zone_end_waypoint", 0
            ),
            automatic_service=rospy.get_param(
                prefix + "/automatic_service",
                "/navigation_profile_manager/use_automatic_navigation",
            ),
            cone_service=rospy.get_param(
                prefix + "/cone_service",
                "/navigation_profile_manager/use_cone_zone_avoidance",
            ),
            service_wait_timeout=rospy.get_param(
                prefix + "/service_wait_timeout", 3.0
            ),
        )

    def __init__(
        self,
        enabled,
        cone_start_waypoint,
        cone_end_waypoint,
        automatic_service,
        cone_service,
        service_wait_timeout,
        wait_for_service=rospy.wait_for_service,
        service_proxy_factory=rospy.ServiceProxy,
    ):
        self.enabled = bool(enabled)
        self._cone_start_waypoint = int(cone_start_waypoint)
        self._cone_end_waypoint = int(cone_end_waypoint)
        self._service_wait_timeout = float(service_wait_timeout)
        self._wait_for_service = wait_for_service
        self._service_proxy_factory = service_proxy_factory
        self._service_names = {
            self.AUTOMATIC: str(automatic_service),
            self.CONE_ZONE: str(cone_service),
        }
        self._service_proxies = {}
        self._selected_profile = None

        if self._service_wait_timeout <= 0.0:
            raise ValueError(
                "navigation/profile_switching/service_wait_timeout must "
                "be positive"
            )
        if self.enabled and (
            self._cone_start_waypoint <= 0
            or self._cone_end_waypoint < self._cone_start_waypoint
        ):
            raise ValueError(
                "enabled profile switching requires positive cone-zone "
                "waypoint bounds with start <= end"
            )
        if self.enabled and any(
            not service_name
            for service_name in self._service_names.values()
        ):
            raise ValueError(
                "enabled profile switching requires both service names"
            )

    def validate_route(self, waypoint_count):
        if not self.enabled:
            return True, ""
        if self._cone_end_waypoint > waypoint_count:
            return False, (
                "cone-zone end waypoint {} exceeds route length {}"
            ).format(self._cone_end_waypoint, waypoint_count)
        return True, ""

    def _profile_for_waypoint(self, waypoint_number):
        if (
            self._cone_start_waypoint
            <= waypoint_number
            <= self._cone_end_waypoint
        ):
            return self.CONE_ZONE
        return self.AUTOMATIC

    def _proxy_for_profile(self, profile_name):
        if profile_name not in self._service_proxies:
            self._service_proxies[profile_name] = (
                self._service_proxy_factory(
                    self._service_names[profile_name], Trigger
                )
            )
        return self._service_proxies[profile_name]

    def switch_for_waypoint(self, waypoint_number):
        if not self.enabled:
            return True, ""

        profile_name = self._profile_for_waypoint(waypoint_number)
        if profile_name == self._selected_profile:
            return True, ""

        service_name = self._service_names[profile_name]
        try:
            self._wait_for_service(
                service_name, timeout=self._service_wait_timeout
            )
            response = self._proxy_for_profile(profile_name)()
        except (rospy.ROSException, rospy.ServiceException) as exc:
            return False, "profile service {} failed: {}".format(
                service_name, exc
            )

        if not response.success:
            return False, "profile service {} rejected switch: {}".format(
                service_name, response.message
            )

        self._selected_profile = profile_name
        return True, "selected navigation profile {} before waypoint {}".format(
            profile_name, waypoint_number
        )
