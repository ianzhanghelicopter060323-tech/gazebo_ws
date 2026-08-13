#!/usr/bin/env python3
"""Run one delivery-only cone-zone navigation case in an active simulation."""

import argparse
import actionlib
import json
import math
from move_base_msgs.msg import MoveBaseAction
from pathlib import Path
import sys
import time

from geometry_msgs.msg import PoseStamped
import rospy

from smart_factory_interfaces.msg import ExecuteTaskAction, TaskState
from smart_factory_navigation import error_codes
from smart_factory_navigation.client import NavigationClient


RESULT_MARKER = "CONE_MOVE_STRESS_RESULT="
READINESS_MARKER = "CONE_MOVE_STRESS_READY="


class CaseError(RuntimeError):
    pass


def parse_args(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument("--case-file", type=Path, required=True)
    parser.add_argument("--readiness-only", action="store_true")
    parser.add_argument(
        "--require-localization",
        action="store_true",
        help=(
            "when used with --readiness-only, also require a fresh AMCL pose "
            "and map-to-base transform"
        ),
    )
    parser.add_argument("--navigation-action", default="/smart_factory/navigation")
    parser.add_argument("--move-base-action", default="/move_base")
    parser.add_argument("--mission-action", default="/sim_task/execute")
    parser.add_argument(
        "--require-mission-action",
        action="store_true",
        help="also require the end-to-end mission Action server",
    )
    parser.add_argument("--navigation-wait-timeout", type=float, default=60.0)
    parser.add_argument("--profile-wait-timeout", type=float, default=60.0)
    parser.add_argument(
        "--skip-profile-services",
        action="store_true",
        help=(
            "use the fixed move_base planner configuration without waiting for "
            "or switching navigation profiles"
        ),
    )
    parser.add_argument(
        "--automatic-service",
        default="/navigation_profile_manager/use_automatic_navigation",
    )
    parser.add_argument(
        "--cone-service",
        default="/navigation_profile_manager/use_cone_zone_avoidance",
    )
    return parser.parse_args(rospy.myargv(argv=[sys.argv[0]] + list(argv))[1:])


def wait_until_ready(
    wait_once,
    timeout,
    poll_timeout=2.0,
    monotonic=time.monotonic,
    shutdown=rospy.is_shutdown,
):
    """Retry a ROS readiness handshake against a wall-clock deadline."""
    timeout = float(timeout)
    poll_timeout = float(poll_timeout)
    if timeout <= 0.0 or poll_timeout <= 0.0:
        raise ValueError("readiness timeouts must be positive")
    deadline = monotonic() + timeout
    while not shutdown():
        remaining = deadline - monotonic()
        if remaining <= 0.0:
            return False
        if wait_once(min(poll_timeout, remaining)):
            return True
    return False


def wait_for_service(service_name, timeout):
    def wait_once(slice_timeout):
        try:
            rospy.wait_for_service(service_name, timeout=slice_timeout)
            return True
        except rospy.ROSException:
            return False

    return wait_until_ready(wait_once, timeout)


def wait_for_navigation_stack(args, navigation):
    if not args.skip_profile_services:
        for service_name in (args.automatic_service, args.cone_service):
            if not wait_for_service(service_name, args.profile_wait_timeout):
                raise CaseError(
                    "profile service {} is unavailable".format(service_name)
                )

    if not wait_until_ready(
        navigation.wait_for_server, args.navigation_wait_timeout
    ):
        raise CaseError(
            "navigation Action server {} is unavailable after {:.1f}s".format(
                args.navigation_action, args.navigation_wait_timeout
            )
        )

    move_base = actionlib.SimpleActionClient(args.move_base_action, MoveBaseAction)
    if not wait_until_ready(
        lambda timeout: move_base.wait_for_server(rospy.Duration(timeout)),
        args.navigation_wait_timeout,
    ):
        raise CaseError(
            "move_base Action server {} is unavailable after {:.1f}s".format(
                args.move_base_action, args.navigation_wait_timeout
            )
        )

    if args.require_mission_action:
        mission = actionlib.SimpleActionClient(
            args.mission_action, ExecuteTaskAction
        )
        if not wait_until_ready(
            lambda timeout: mission.wait_for_server(rospy.Duration(timeout)),
            args.navigation_wait_timeout,
        ):
            raise CaseError(
                "mission Action server {} is unavailable after {:.1f}s".format(
                    args.mission_action, args.navigation_wait_timeout
                )
            )


def finite_float(value, label, minimum=None):
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise CaseError("{} must be numeric".format(label)) from exc
    if not math.isfinite(result):
        raise CaseError("{} must be finite".format(label))
    if minimum is not None and result < minimum:
        raise CaseError("{} must be at least {}".format(label, minimum))
    return result


def load_case(path):
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        raise CaseError("cannot load {}: {}".format(path, exc))
    destination = data.get("destination") if isinstance(data, dict) else None
    if not isinstance(destination, dict):
        raise CaseError("case destination must be a mapping")
    task_id = str(data.get("task_id", "")).strip()
    target_class = str(data.get("target_class", "")).strip()
    if not task_id:
        raise CaseError("case task_id must not be empty")
    if target_class not in {"food", "daily", "electronics"}:
        raise CaseError("case target_class is invalid")
    return {
        "task_id": task_id,
        "target_class": target_class,
        "target_name": str(data.get("target_name", target_class)),
        "switch_distance": finite_float(
            data.get("switch_distance"), "switch_distance", minimum=1.0e-6
        ),
        "destination": {
            "frame_id": str(destination.get("frame_id", "map")).strip() or "map",
            "x": finite_float(destination.get("x"), "destination.x"),
            "y": finite_float(destination.get("y"), "destination.y"),
            "yaw": finite_float(destination.get("yaw"), "destination.yaw"),
            "position_tolerance": finite_float(
                destination.get("position_tolerance", 0.04),
                "destination.position_tolerance",
                minimum=1.0e-6,
            ),
            "yaw_tolerance": finite_float(
                destination.get("yaw_tolerance"),
                "destination.yaw_tolerance",
                minimum=1.0e-6,
            ),
        },
    }


def make_pose(config):
    pose = PoseStamped()
    pose.header.frame_id = config["frame_id"]
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = config["x"]
    pose.pose.position.y = config["y"]
    pose.pose.orientation.z = math.sin(config["yaw"] / 2.0)
    pose.pose.orientation.w = math.cos(config["yaw"] / 2.0)
    return pose


class StagePublisher:
    def __init__(self, task_id):
        self._task_id = task_id
        self._publisher = rospy.Publisher(
            "/sim_task/state", TaskState, queue_size=10, latch=True
        )

    def wait_for_connections(self, timeout=5.0):
        deadline = time.monotonic() + timeout
        while self._publisher.get_num_connections() == 0:
            if time.monotonic() >= deadline or rospy.is_shutdown():
                raise CaseError("no subscriber connected to /sim_task/state")
            time.sleep(0.05)

    def publish(self, state, detail, retry_count=0):
        message = TaskState()
        message.header.stamp = rospy.Time.now()
        message.task_id = self._task_id
        message.state = int(state)
        message.retry_count = int(retry_count)
        message.detail = detail
        self._publisher.publish(message)


def result_payload(case):
    return {
        "task_id": case.get("task_id", ""),
        "target_class": case.get("target_class", ""),
        "target_name": case.get("target_name", ""),
        "success": False,
        "status": "navigation_error",
        "completed_stage": TaskState.NAVIGATE_TO_DELIVERY,
        "error_code": error_codes.INTERNAL_ERROR,
        "message": "navigation was not attempted",
        "planner_mode": "fixed_teb",
        "profile_switching_required": True,
        "profile_switched": False,
        "profile_switch_distance_m": None,
        "minimum_goal_distance_m": None,
    }


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    case = {}
    payload = result_payload(case)
    stages = None
    try:
        case = load_case(args.case_file.expanduser().resolve())
        payload = result_payload(case)
        payload["profile_switching_required"] = not args.skip_profile_services
        rospy.init_node("cone_move_stress_navigation", anonymous=True)
        navigation = NavigationClient(args.navigation_action)
        wait_for_navigation_stack(args, navigation)
        if args.readiness_only:
            localization = None
            if args.require_localization:
                localization = navigation.wait_for_localization()
                if not localization.success:
                    raise CaseError(
                        localization.message or "localization is not ready"
                    )
            print(
                READINESS_MARKER
                + json.dumps(
                    {
                        "navigation_action": args.navigation_action,
                        "move_base_action": args.move_base_action,
                        "mission_action": (
                            args.mission_action
                            if args.require_mission_action
                            else None
                        ),
                        "planner_mode": (
                            "fixed_teb"
                            if args.skip_profile_services
                            else "navigation_profiles"
                        ),
                        "profile_services": (
                            []
                            if args.skip_profile_services
                            else [args.automatic_service, args.cone_service]
                        ),
                        "localization_ready": bool(
                            localization is not None and localization.success
                        ),
                        "ready": True,
                    },
                    ensure_ascii=False,
                    sort_keys=True,
                )
            )
            return 0
        readiness = navigation.wait_for_localization()
        if not readiness.success:
            payload["error_code"] = readiness.error_code
            raise CaseError(readiness.message or "localization is not ready")

        switcher = None
        if not args.skip_profile_services:
            from smart_factory_navigation.profile_switcher import (
                NavigationProfileSwitcher,
            )

            switcher = NavigationProfileSwitcher(
                enabled=True,
                automatic_service=args.automatic_service,
                cone_service=args.cone_service,
                service_wait_timeout=args.profile_wait_timeout,
            )
            selected, message = switcher.select_cone_zone_avoidance()
            if not selected:
                raise CaseError(message)

        stages = StagePublisher(case["task_id"])
        stages.wait_for_connections()
        stages.publish(
            TaskState.NAVIGATE_TO_DELIVERY,
            (
                "delivery-only stress case started with fixed TEB planner"
                if switcher is None
                else "delivery-only stress case started with cone-zone Profile"
            ),
        )
        time.sleep(0.2)

        transition = {"attempted": False, "switched": False, "failure": ""}

        def feedback(event):
            distance = event.distance_to_goal
            if distance is not None and math.isfinite(distance):
                current_minimum = payload["minimum_goal_distance_m"]
                payload["minimum_goal_distance_m"] = (
                    distance
                    if current_minimum is None
                    else min(current_minimum, distance)
                )
            stages.publish(
                TaskState.NAVIGATE_TO_DELIVERY,
                event.detail or "workshop move_base goal is active",
                event.retry_count,
            )
            if (
                switcher is None
                or
                transition["attempted"]
                or distance is None
                or not math.isfinite(distance)
                or distance > case["switch_distance"]
            ):
                return
            transition["attempted"] = True
            payload["profile_switch_distance_m"] = distance
            success, switch_message = switcher.select_automatic_navigation()
            if not success:
                transition["failure"] = switch_message
                return
            transition["switched"] = True
            payload["profile_switched"] = True
            stages.publish(
                TaskState.NAVIGATE_TO_DELIVERY,
                "within {:.2f} m of workshop; automatic Profile selected".format(
                    case["switch_distance"]
                ),
            )

        destination = case["destination"]
        navigation_result = navigation.navigate_pose(
            make_pose(destination),
            request_id=case["task_id"],
            feedback_cb=feedback,
            preempt_requested=lambda: bool(transition["failure"]),
            position_tolerance=destination["position_tolerance"],
            yaw_tolerance=destination["yaw_tolerance"],
        )
        if transition["failure"]:
            raise CaseError(transition["failure"])
        if not navigation_result.success:
            payload["error_code"] = navigation_result.error_code
            raise CaseError(navigation_result.message)

        if switcher is not None and not transition["switched"]:
            success, switch_message = switcher.select_automatic_navigation()
            transition["attempted"] = True
            if not success:
                raise CaseError(switch_message)
            transition["switched"] = True
            payload["profile_switched"] = True

        payload.update(
            {
                "success": True,
                "status": "navigation_completed",
                "completed_stage": TaskState.ARRIVED_DELIVERY,
                "error_code": error_codes.SUCCESS,
                "message": "reached {}".format(case["target_name"]),
            }
        )
        stages.publish(TaskState.ARRIVED_DELIVERY, payload["message"])
    except (CaseError, OSError, ValueError, rospy.ROSException) as exc:
        payload["message"] = str(exc)
        if stages is not None:
            try:
                stages.publish(TaskState.TASK_FAILED, str(exc))
            except Exception:
                pass

    print(RESULT_MARKER + json.dumps(payload, ensure_ascii=False, sort_keys=True))
    return 0 if payload["success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
