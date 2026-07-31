#!/usr/bin/env python3

import unittest

from smart_factory_mission.navigation_profile_switcher import (
    NavigationProfileSwitcher,
)


class Response:
    def __init__(self, success=True, message="ok"):
        self.success = success
        self.message = message


class NavigationProfileSwitcherTest(unittest.TestCase):
    def _make_switcher(self, enabled=True, responses=None):
        calls = []
        responses = responses or {}

        def wait_for_service(name, timeout):
            calls.append(("wait", name, timeout))

        def service_proxy_factory(name, _service_type):
            def call():
                calls.append(("call", name))
                return responses.get(name, Response())

            return call

        switcher = NavigationProfileSwitcher(
            enabled=enabled,
            cone_start_waypoint=3 if enabled else 0,
            cone_end_waypoint=4 if enabled else 0,
            automatic_service="/automatic",
            cone_service="/cone",
            service_wait_timeout=2.0,
            wait_for_service=wait_for_service,
            service_proxy_factory=service_proxy_factory,
        )
        return switcher, calls

    def test_disabled_switcher_never_calls_services(self):
        switcher, calls = self._make_switcher(enabled=False)

        success, message = switcher.switch_for_waypoint(3)

        self.assertTrue(success)
        self.assertEqual("", message)
        self.assertEqual([], calls)

    def test_switches_only_when_profile_changes(self):
        switcher, calls = self._make_switcher()

        for waypoint in (1, 2, 3, 4, 5):
            success, _message = switcher.switch_for_waypoint(waypoint)
            self.assertTrue(success)

        service_calls = [call for call in calls if call[0] == "call"]
        self.assertEqual(
            [
                ("call", "/automatic"),
                ("call", "/cone"),
                ("call", "/automatic"),
            ],
            service_calls,
        )

    def test_rejected_switch_is_not_remembered(self):
        switcher, calls = self._make_switcher(
            responses={"/cone": Response(False, "not ready")}
        )

        first_success, _message = switcher.switch_for_waypoint(3)
        second_success, _message = switcher.switch_for_waypoint(3)

        self.assertFalse(first_success)
        self.assertFalse(second_success)
        cone_calls = [call for call in calls if call == ("call", "/cone")]
        self.assertEqual(2, len(cone_calls))

    def test_route_bounds_must_fit_loaded_route(self):
        switcher, _calls = self._make_switcher()

        success, message = switcher.validate_route(3)

        self.assertFalse(success)
        self.assertIn("exceeds route length", message)


if __name__ == "__main__":
    unittest.main()
