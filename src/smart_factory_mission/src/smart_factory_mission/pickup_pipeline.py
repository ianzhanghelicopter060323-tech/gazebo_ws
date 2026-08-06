"""Conditional 35 -> 36 -> 37 recognition, alignment, and fixed grasp."""

from dataclasses import dataclass
import math

from geometry_msgs.msg import PoseStamped
import rospy

from smart_factory_interfaces.srv import LocateCube, LocateCubeRequest
from smart_factory_mission import error_codes, states
from smart_factory_mission.fixed_grasp import FixedGraspPlanner
from smart_factory_mission.manipulation_stage import ManipulationStage


class PickupFailure(RuntimeError):
    def __init__(self, error_code, message):
        super().__init__(message)
        self.error_code = error_code


class PickupPreempted(RuntimeError):
    pass


@dataclass(frozen=True)
class CandidateStation:
    number: int
    x: float
    y: float
    yaw: float
    scan_positions: tuple

    def pose(self, frame_id):
        goal = PoseStamped()
        goal.header.frame_id = frame_id
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = self.x
        goal.pose.position.y = self.y
        goal.pose.orientation.z = math.sin(self.yaw / 2.0)
        goal.pose.orientation.w = math.cos(self.yaw / 2.0)
        return goal


class PickupPipeline:
    """Run the pickup-only milestone after the main route reaches seq35."""

    def __init__(self, config):
        if not isinstance(config, dict):
            raise ValueError("pickup configuration must be a mapping")
        self._enabled = bool(config.get("enabled", True))
        self._frame_id = str(config.get("frame_id", "map")).strip()
        self._service_name = str(
            config.get("perception_service", "/cube_locator/locate")
        ).strip()
        self._service_wait = float(config.get("perception_wait_timeout", 20.0))
        self._recognition_retries = int(config.get("recognition_retries", 1))
        self._alignment_tolerance = float(
            config.get("alignment_tolerance", 0.015)
        )
        self._maximum_alignment_correction = float(
            config.get("maximum_alignment_correction", 0.25)
        )
        self._maximum_alignment_iterations = int(
            config.get("maximum_alignment_iterations", 2)
        )
        self._stations = self._load_stations(config.get("stations", []))
        fixed = config.get("fixed_grasp", {})
        self._planner = FixedGraspPlanner(
            target_forward=float(fixed.get("target_forward", 0.356)),
            target_lateral=float(fixed.get("target_lateral", 0.0)),
            depth_to_center_forward=float(
                fixed.get("depth_to_center_forward", 0.0)
            ),
            frame_id=self._frame_id,
        )
        self._manipulation = ManipulationStage(config.get("manipulation", {}))
        self._validate()
        self._locate = rospy.ServiceProxy(self._service_name, LocateCube)

    @staticmethod
    def _load_stations(raw_stations):
        if not isinstance(raw_stations, list):
            raise ValueError("pickup stations must be a list")
        stations = []
        for raw in raw_stations:
            if not isinstance(raw, dict):
                raise ValueError("each pickup station must be a mapping")
            number = int(raw["number"])
            scan = raw.get("scan_positions")
            if not isinstance(scan, list) or len(scan) != 5:
                raise ValueError(
                    "station {} scan_positions must contain five values".format(
                        number
                    )
                )
            values = (
                float(raw["x"]),
                float(raw["y"]),
                float(raw["yaw"]),
                *(float(value) for value in scan),
            )
            if not all(math.isfinite(value) for value in values):
                raise ValueError("station {} contains non-finite values".format(number))
            stations.append(
                CandidateStation(number, values[0], values[1], values[2], values[3:])
            )
        return tuple(stations)

    def _validate(self):
        if not self._frame_id or not self._service_name:
            raise ValueError("pickup frame and perception service must not be empty")
        if tuple(station.number for station in self._stations) != (35, 36, 37):
            raise ValueError("pickup stations must be ordered exactly 35, 36, 37")
        if self._recognition_retries < 0:
            raise ValueError("recognition_retries must not be negative")
        if self._maximum_alignment_iterations <= 0:
            raise ValueError("maximum_alignment_iterations must be positive")
        for name, value in (
            ("perception_wait_timeout", self._service_wait),
            ("alignment_tolerance", self._alignment_tolerance),
            ("maximum_alignment_correction", self._maximum_alignment_correction),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError("{} must be finite and positive".format(name))
        if self._maximum_alignment_correction <= self._alignment_tolerance:
            raise ValueError(
                "maximum_alignment_correction must exceed alignment_tolerance"
            )

    @property
    def enabled(self):
        return self._enabled

    def wait_for_perception(self):
        try:
            rospy.wait_for_service(self._service_name, timeout=self._service_wait)
            return True
        except rospy.ROSException:
            return False

    def _observe(self, station, require_classification, state_machine, preempt):
        if preempt():
            raise PickupPreempted("task preempted before cube observation")
        state_machine.transition(
            states.OBSERVE_PICKUP_CANDIDATE,
            "moving arm to seq{} camera observation pose".format(station.number),
        )
        if not self._manipulation.move_arm(station.scan_positions, preempt):
            if preempt():
                raise PickupPreempted("task preempted while positioning camera")
            raise PickupFailure(
                error_codes.MANIPULATION_FAILED,
                "arm did not reach seq{} observation pose".format(station.number),
            )

        last_message = "no perception response"
        for attempt in range(self._recognition_retries + 1):
            if preempt():
                raise PickupPreempted("task preempted during cube recognition")
            state_machine.transition(
                states.LOCALIZE_TARGET,
                "seq{} RGB-D observation attempt {}/{}".format(
                    station.number,
                    attempt + 1,
                    self._recognition_retries + 1,
                ),
            )
            request = LocateCubeRequest()
            request.station = station.number
            request.require_classification = bool(require_classification)
            try:
                response = self._locate(request)
            except rospy.ServiceException as exc:
                last_message = "perception service failed: {}".format(exc)
                continue
            last_message = response.message
            if response.success:
                return response
            rospy.logwarn(
                "seq%d observation rejected (attempt %d): %s",
                station.number,
                attempt + 1,
                response.message,
            )
        raise PickupFailure(
            error_codes.OBJECT_NOT_FOUND,
            "seq{} has no stable cube observation: {}".format(
                station.number, last_message
            ),
        )

    def _choose_target(self, context, state_machine, navigate, preempt):
        for index, station in enumerate(self._stations):
            if index > 0:
                navigate(
                    station.pose(self._frame_id),
                    states.NAVIGATE_TO_PICKUP_CANDIDATE,
                    "navigating conditionally to seq{}".format(station.number),
                )
            # Seq37 is selected by elimination.  OCR still supplies a visual
            # box for depth localization, but its class is not compared.
            response = self._observe(
                station,
                require_classification=(station.number != 37),
                state_machine=state_machine,
                preempt=preempt,
            )
            if station.number == 37 or response.detected_class == context.target_class:
                rospy.loginfo(
                    "selected seq%d for target class %d (observed class %d)",
                    station.number,
                    context.target_class,
                    response.detected_class,
                )
                return station, response
            rospy.loginfo(
                "seq%d is stable class %d, not target %d; continuing",
                station.number,
                response.detected_class,
                context.target_class,
            )
        raise PickupFailure(error_codes.OBJECT_NOT_FOUND, "target cube was not selected")

    def _align(
        self,
        station,
        response,
        state_machine,
        navigate,
        localized_pose,
        preempt,
    ):
        latest = response
        for correction_index in range(self._maximum_alignment_iterations + 1):
            current = localized_pose(self._frame_id)
            if current is None:
                raise PickupFailure(
                    error_codes.ALIGNMENT_FAILED,
                    "localized base pose is unavailable during grasp alignment",
                )
            goal = self._planner.goal_from_surface(
                latest.point_map.point.x,
                latest.point_map.point.y,
                # Preserve the measured base heading and correct translation
                # only.  The grasp target is defined in the base frame, so an
                # arbitrary but fixed heading is valid and avoids DWA hunting
                # around a tighter-than-achievable station yaw tolerance.
                current[2],
                stamp=rospy.Time.now(),
            )
            correction = self._planner.correction_distance(current[0], current[1], goal)
            if correction <= self._alignment_tolerance:
                rospy.loginfo(
                    "seq%d fixed-standoff alignment converged at %.4fm",
                    station.number,
                    correction,
                )
                return latest
            if correction > self._maximum_alignment_correction:
                raise PickupFailure(
                    error_codes.ALIGNMENT_FAILED,
                    "seq{} requested {:.3f}m correction, above {:.3f}m limit".format(
                        station.number,
                        correction,
                        self._maximum_alignment_correction,
                    ),
                )
            if correction_index >= self._maximum_alignment_iterations:
                raise PickupFailure(
                    error_codes.ALIGNMENT_FAILED,
                    "seq{} did not converge within {} fixed-distance corrections; "
                    "remaining error {:.3f}m".format(
                        station.number,
                        self._maximum_alignment_iterations,
                        correction,
                    ),
                )
            navigate(
                goal,
                states.ALIGN_FOR_GRASP,
                "seq{} fixed-standoff correction {}/{}: {:.3f}m".format(
                    station.number,
                    correction_index + 1,
                    self._maximum_alignment_iterations,
                    correction,
                ),
            )
            latest = self._observe(
                station,
                require_classification=False,
                state_machine=state_machine,
                preempt=preempt,
            )
        raise PickupFailure(error_codes.ALIGNMENT_FAILED, "alignment loop ended unexpectedly")

    def run(self, context, state_machine, navigate, localized_pose, preempt):
        if not self._enabled:
            raise PickupFailure(
                error_codes.INTERNAL_ERROR, "pickup pipeline is disabled"
            )
        if not self.wait_for_perception():
            raise PickupFailure(
                error_codes.PERCEPTION_UNAVAILABLE,
                "cube locator service {} is unavailable".format(self._service_name),
            )

        station, response = self._choose_target(
            context, state_machine, navigate, preempt
        )

        state_machine.transition(
            states.OPEN_GRIPPER,
            "opening gripper before any grasp alignment or descent",
        )
        if not self._manipulation.open_gripper(preempt):
            if preempt():
                raise PickupPreempted("task preempted while opening gripper")
            raise PickupFailure(
                error_codes.MANIPULATION_FAILED,
                "gripper did not reach the open IDLE state",
            )

        self._align(
            station,
            response,
            state_machine,
            navigate,
            localized_pose,
            preempt,
        )

        state_machine.transition(
            states.GRASP_OBJECT,
            "moving open gripper to fixed grasp pose at seq{}".format(station.number),
        )
        if not self._manipulation.move_to_grasp_pose(preempt):
            if preempt():
                raise PickupPreempted("task preempted during grasp descent")
            raise PickupFailure(
                error_codes.MANIPULATION_FAILED,
                "arm did not reach the fixed grasp pose",
            )
        if not self._manipulation.wait_until_ready(preempt):
            raise PickupFailure(
                error_codes.GRASP_FAILED,
                "fixed grasp pose is not inside the cube grasp window",
            )

        state_machine.transition(
            states.VERIFY_GRASP,
            "closing gripper and waiting for GRASPING",
        )
        if not self._manipulation.close_and_verify(preempt):
            if preempt():
                raise PickupPreempted("task preempted while closing gripper")
            raise PickupFailure(
                error_codes.GRASP_FAILED,
                "gripper closed without entering GRASPING",
            )

        # The observation posture raises the TCP from 0.018 m to about 0.289 m
        # in the calibrated seq35 test, while preserving GRASPING.
        if not self._manipulation.move_arm(station.scan_positions, preempt):
            raise PickupFailure(
                error_codes.MANIPULATION_FAILED,
                "cube was grasped but the arm did not reach the lift pose",
            )
        if self._manipulation.grasp_state() != "GRASPING":
            raise PickupFailure(
                error_codes.GRASP_FAILED,
                "cube was lost while lifting from seq{}".format(station.number),
            )

        state_machine.transition(
            states.OBJECT_GRASPED,
            "seq{} target grasped and lifted with fixed 0.356m standoff".format(
                station.number
            ),
        )
        return station.number
