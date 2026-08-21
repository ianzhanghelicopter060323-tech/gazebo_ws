"""Laser-driven safe projection for the delivery entry waypoint.

The selector does not create or alter a global path.  It projects the existing
task-level entry waypoint within a small configured neighbourhood, using only
obstacle surfaces observed in the latest laser scan.  The selected waypoint is
then submitted to the ordinary navigation Action and official GlobalPlanner.
"""

from dataclasses import dataclass
import copy
import math
import threading
import time

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.srv import GetPlan, GetPlanRequest
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool
import tf2_ros


class EntrySelectionUnavailable(RuntimeError):
    """Raised when a fresh scan cannot produce a safe entry waypoint."""


class EntrySelectionPreempted(EntrySelectionUnavailable):
    """Raised when task cancellation interrupts entry selection."""


@dataclass(frozen=True)
class EntrySelectorConfig:
    candidate_max_shift: float = 0.30
    candidate_step: float = 0.03
    candidate_angle_samples: int = 36
    hard_min_clearance: float = 0.20
    desired_clearance: float = 0.28
    influence_radius: float = 0.80
    clearance_weight: float = 6.0
    shift_weight: float = 1.0

    def validate(self):
        numeric = (
            self.candidate_max_shift,
            self.candidate_step,
            self.hard_min_clearance,
            self.desired_clearance,
            self.influence_radius,
            self.clearance_weight,
            self.shift_weight,
        )
        if not all(math.isfinite(value) for value in numeric):
            raise ValueError("delivery entry selector values must be finite")
        if self.candidate_max_shift <= 0.0:
            raise ValueError("candidate_max_shift must be positive")
        if not 0.0 < self.candidate_step <= self.candidate_max_shift:
            raise ValueError(
                "candidate_step must be positive and no larger than "
                "candidate_max_shift"
            )
        if self.candidate_angle_samples < 8:
            raise ValueError("candidate_angle_samples must be at least 8")
        if self.hard_min_clearance <= 0.0:
            raise ValueError("hard_min_clearance must be positive")
        if self.desired_clearance < self.hard_min_clearance:
            raise ValueError(
                "desired_clearance must be at least hard_min_clearance"
            )
        if self.influence_radius <= self.candidate_max_shift:
            raise ValueError(
                "influence_radius must be larger than candidate_max_shift"
            )
        if self.clearance_weight <= 0.0 or self.shift_weight <= 0.0:
            raise ValueError("selector weights must be positive")


@dataclass(frozen=True)
class EntrySelection:
    x: float
    y: float
    shift: float
    clearance: float
    obstacle_points: int


@dataclass(frozen=True)
class ChannelSelectorConfig:
    horizon: float = 0.55
    fan_radii: tuple = ()
    trigger_lookahead: float = 1.00
    fan_half_angle: float = math.radians(60.0)
    fan_angle_step: float = math.radians(10.0)
    min_forward_progress: float = 0.20
    entry_depth_floor: float = 0.20
    trigger_clearance: float = 0.30
    hard_min_clearance: float = 0.20
    desired_clearance: float = 0.36
    clearance_weight: float = 20.0
    angular_weight: float = 0.10
    path_length_weight: float = 0.05
    escape_distance: float = 0.25
    escape_min_clearance: float = 0.18
    escape_max_drop: float = 0.02

    def validate(self):
        numeric = (
            self.horizon,
            self.trigger_lookahead,
            self.fan_half_angle,
            self.fan_angle_step,
            self.min_forward_progress,
            self.entry_depth_floor,
            self.trigger_clearance,
            self.hard_min_clearance,
            self.desired_clearance,
            self.clearance_weight,
            self.angular_weight,
            self.path_length_weight,
            self.escape_distance,
            self.escape_min_clearance,
            self.escape_max_drop,
        )
        if not all(math.isfinite(value) for value in numeric):
            raise ValueError("delivery channel selector values must be finite")
        if self.horizon <= 0.0:
            raise ValueError("channel horizon must be positive")
        if self.trigger_lookahead <= 0.0:
            raise ValueError("channel trigger_lookahead must be positive")
        if not self.fan_radii:
            fan_radii = (self.horizon,)
        else:
            fan_radii = self.fan_radii
        if not all(
            isinstance(value, (int, float))
            and math.isfinite(value)
            and value > 0.0
            for value in fan_radii
        ):
            raise ValueError(
                "channel fan_radii must contain positive finite values"
            )
        if tuple(sorted(set(fan_radii))) != tuple(fan_radii):
            raise ValueError(
                "channel fan_radii must be unique and increasing"
            )
        if not 0.0 < self.fan_half_angle < 0.5 * math.pi:
            raise ValueError(
                "channel fan_half_angle must be in (0, pi/2)"
            )
        if not 0.0 < self.fan_angle_step <= self.fan_half_angle:
            raise ValueError(
                "channel fan_angle_step must be positive and no larger than "
                "fan_half_angle"
            )
        if self.min_forward_progress <= 0.0:
            raise ValueError("channel min_forward_progress must be positive")
        if self.entry_depth_floor < 0.0:
            raise ValueError("channel entry_depth_floor must be nonnegative")
        if self.hard_min_clearance <= 0.0:
            raise ValueError("channel hard_min_clearance must be positive")
        if self.escape_distance <= 0.0:
            raise ValueError("channel escape_distance must be positive")
        if not 0.0 < self.escape_min_clearance <= self.hard_min_clearance:
            raise ValueError(
                "channel escape_min_clearance must be positive and no larger "
                "than hard_min_clearance"
            )
        if self.escape_max_drop < 0.0:
            raise ValueError("channel escape_max_drop must be nonnegative")
        if self.trigger_clearance < self.hard_min_clearance:
            raise ValueError(
                "channel trigger_clearance must be at least hard_min_clearance"
            )
        if self.desired_clearance < self.trigger_clearance:
            raise ValueError(
                "channel desired_clearance must be at least trigger_clearance"
            )
        if (
            self.clearance_weight <= 0.0
            or self.angular_weight <= 0.0
            or self.path_length_weight < 0.0
        ):
            raise ValueError("channel selector weights must be positive")

    @property
    def resolved_fan_radii(self):
        return self.fan_radii or (self.horizon,)


@dataclass(frozen=True)
class PathEvaluation:
    clearance: float
    length: float
    points: tuple = ()
    bottleneck_distance: float = math.nan
    start_clearance: float = math.nan
    recovery_clearance: float = math.nan


@dataclass(frozen=True)
class ChannelSelection:
    x: float
    y: float
    lateral_shift: float
    direct_clearance: float
    approach_clearance: float
    onward_clearance: float
    planned_clearance: float
    planned_length: float
    forward_distance: float
    reference_bottleneck_distance: float
    planned_bottleneck_distance: float
    planned_start_clearance: float
    planned_recovery_clearance: float
    used_escape_allowance: bool
    make_plan_goal_adjustment: float
    fan_angle: float
    forward_progress: float
    entry_depth: float
    obstacle_points: int


class ClearanceChannelSelector:
    """Choose a safe rolling waypoint from a direction-constrained fan."""

    def __init__(self, config=None):
        self.config = config or ChannelSelectorConfig()
        self.config.validate()

    @staticmethod
    def _point_to_segment_measurement(point, start, end):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        squared_length = dx * dx + dy * dy
        if squared_length <= 1.0e-12:
            return (
                math.hypot(point[0] - start[0], point[1] - start[1]),
                0.0,
            )
        projection = max(
            0.0,
            min(
                1.0,
                (
                    (point[0] - start[0]) * dx
                    + (point[1] - start[1]) * dy
                )
                / squared_length,
            ),
        )
        closest_x = start[0] + projection * dx
        closest_y = start[1] + projection * dy
        return (
            math.hypot(point[0] - closest_x, point[1] - closest_y),
            projection * math.sqrt(squared_length),
        )

    @classmethod
    def _point_to_segment_distance(cls, point, start, end):
        return cls._point_to_segment_measurement(point, start, end)[0]

    @classmethod
    def _segment_bottleneck(cls, start, end, points):
        if not points:
            return math.inf, 0.0
        return min(
            cls._point_to_segment_measurement(point, start, end)
            for point in points
        )

    @classmethod
    def _segment_clearance(cls, start, end, points):
        return cls._segment_bottleneck(start, end, points)[0]

    def _fan_angle_offsets(self):
        steps = int(
            math.ceil(self.config.fan_half_angle / self.config.fan_angle_step)
        )
        yield 0.0
        previous = 0.0
        for index in range(1, steps + 1):
            offset = min(
                index * self.config.fan_angle_step,
                self.config.fan_half_angle,
            )
            if offset <= previous + 1.0e-12:
                continue
            yield offset
            yield -offset
            previous = offset

    @staticmethod
    def _sanitized_path(reference_path_points, start, goal):
        path = []
        for point in reference_path_points or ():
            if (
                len(point) >= 2
                and math.isfinite(point[0])
                and math.isfinite(point[1])
            ):
                candidate = (float(point[0]), float(point[1]))
                if not path or math.hypot(
                    candidate[0] - path[-1][0],
                    candidate[1] - path[-1][1],
                ) > 1.0e-9:
                    path.append(candidate)
        if len(path) < 2:
            return [start, goal]
        if math.hypot(path[0][0] - start[0], path[0][1] - start[1]) > 1.0e-3:
            path.insert(0, start)
        return path

    @staticmethod
    def _path_length(path):
        return sum(
            math.hypot(
                path[index][0] - path[index - 1][0],
                path[index][1] - path[index - 1][1],
            )
            for index in range(1, len(path))
        )

    @classmethod
    def _sample_path(cls, path, distance):
        traversed = 0.0
        for index in range(1, len(path)):
            start = path[index - 1]
            end = path[index]
            dx = end[0] - start[0]
            dy = end[1] - start[1]
            segment_length = math.hypot(dx, dy)
            if segment_length <= 1.0e-12:
                continue
            if traversed + segment_length + 1.0e-9 >= distance:
                along = max(0.0, min(segment_length, distance - traversed))
                ratio = along / segment_length
                return (
                    (start[0] + ratio * dx, start[1] + ratio * dy),
                    (dx / segment_length, dy / segment_length),
                )
            traversed += segment_length
        return None

    @classmethod
    def _path_window_measurement(
        cls, path, start_distance, end_distance, points
    ):
        traversed = 0.0
        bottleneck = (math.inf, 0.0)
        for index in range(1, len(path)):
            start = path[index - 1]
            end = path[index]
            dx = end[0] - start[0]
            dy = end[1] - start[1]
            segment_length = math.hypot(dx, dy)
            if segment_length <= 1.0e-12:
                continue
            overlap_start = max(0.0, start_distance - traversed)
            overlap_end = min(segment_length, end_distance - traversed)
            if overlap_end <= overlap_start + 1.0e-12:
                traversed += segment_length
                if traversed + 1.0e-9 >= end_distance:
                    break
                continue
            start_ratio = overlap_start / segment_length
            end_ratio = overlap_end / segment_length
            clipped_start = (
                start[0] + start_ratio * dx,
                start[1] + start_ratio * dy,
            )
            clipped_end = (
                start[0] + end_ratio * dx,
                start[1] + end_ratio * dy,
            )
            clearance, local_distance = cls._segment_bottleneck(
                clipped_start, clipped_end, points
            )
            measurement = (
                clearance,
                traversed + overlap_start + local_distance,
            )
            if measurement < bottleneck:
                bottleneck = measurement
            traversed += segment_length
            if traversed + 1.0e-9 >= end_distance:
                break
        return bottleneck

    @classmethod
    def _path_prefix_measurement(cls, path, max_length, points):
        return cls._path_window_measurement(path, 0.0, max_length, points)

    @classmethod
    def _path_prefix_clearance(cls, path, max_length, points):
        return cls._path_prefix_measurement(path, max_length, points)[0]

    def select(
        self,
        start_x,
        start_y,
        goal_x,
        goal_y,
        obstacle_points,
        candidate_path_clearance=None,
        candidate_onward_path_clearance=None,
        reference_path_points=None,
        candidate_axis_yaw=None,
        entry_origin=None,
        entry_axis_yaw=None,
        minimum_entry_depth=None,
        force_selection=False,
        excluded_candidates=(),
        exclusion_radius=0.0,
        escape_min_clearance=None,
    ):
        if candidate_axis_yaw is None:
            candidate_axis_yaw = math.atan2(goal_y - start_y, goal_x - start_x)
        if entry_origin is None:
            entry_origin = (start_x, start_y)
        if entry_axis_yaw is None:
            entry_axis_yaw = candidate_axis_yaw
        if minimum_entry_depth is None:
            minimum_entry_depth = self.config.entry_depth_floor
        exclusion_radius = float(exclusion_radius)
        if not math.isfinite(exclusion_radius) or exclusion_radius < 0.0:
            raise EntrySelectionUnavailable(
                "delivery channel candidate exclusion radius is invalid"
            )
        if escape_min_clearance is None:
            escape_min_clearance = self.config.escape_min_clearance
        escape_min_clearance = float(escape_min_clearance)
        if (
            not math.isfinite(escape_min_clearance)
            or escape_min_clearance <= 0.0
            or escape_min_clearance > self.config.hard_min_clearance
        ):
            raise EntrySelectionUnavailable(
                "delivery channel escape minimum clearance is invalid"
            )
        excluded_candidates = tuple(
            (float(x), float(y)) for x, y in excluded_candidates
        )
        coordinates = (
            start_x,
            start_y,
            goal_x,
            goal_y,
            candidate_axis_yaw,
            entry_origin[0],
            entry_origin[1],
            entry_axis_yaw,
            minimum_entry_depth,
            exclusion_radius,
            escape_min_clearance,
        )
        if not all(math.isfinite(value) for value in coordinates):
            raise EntrySelectionUnavailable(
                "delivery channel start or goal pose is not finite"
            )
        if minimum_entry_depth < self.config.entry_depth_floor:
            minimum_entry_depth = self.config.entry_depth_floor
        points = [
            (float(x), float(y))
            for x, y in obstacle_points
            if math.isfinite(x) and math.isfinite(y)
        ]
        start = (start_x, start_y)
        remaining = math.hypot(goal_x - start_x, goal_y - start_y)
        fan_radii = tuple(
            radius
            for radius in self.config.resolved_fan_radii
            if radius + 1.0e-9 < remaining
        )
        if not fan_radii:
            return None
        if not points:
            return None

        reference_path = self._sanitized_path(
            reference_path_points,
            start,
            (goal_x, goal_y),
        )
        reference_length = self._path_length(reference_path)
        direct_clearance, reference_bottleneck_distance = (
            self._path_prefix_measurement(
                reference_path,
                min(self.config.trigger_lookahead, reference_length),
                points,
            )
        )
        if (
            not force_selection
            and direct_clearance + 1.0e-9
            >= self.config.trigger_clearance
        ):
            return None

        feasible = []
        candidate_count = 0
        onward_rejected = 0
        onward_no_plan_rejected = 0
        no_plan_rejected = 0
        planned_clearance_rejected = 0
        sector_rejected = 0
        excluded_rejected = 0
        best_rejected_onward = None
        best_rejected_plan = None
        axis_forward = (
            math.cos(candidate_axis_yaw),
            math.sin(candidate_axis_yaw),
        )
        axis_lateral = (-axis_forward[1], axis_forward[0])
        entry_forward = (
            math.cos(entry_axis_yaw),
            math.sin(entry_axis_yaw),
        )
        for forward_distance in fan_radii:
            for requested_fan_angle in self._fan_angle_offsets():
                candidate_count += 1
                heading = candidate_axis_yaw + requested_fan_angle
                requested_candidate = (
                    start[0] + forward_distance * math.cos(heading),
                    start[1] + forward_distance * math.sin(heading),
                )
                candidate = requested_candidate
                forward_progress = forward_distance * math.cos(
                    requested_fan_angle
                )
                lateral_shift = forward_distance * math.sin(
                    requested_fan_angle
                )
                entry_depth = (
                    (candidate[0] - entry_origin[0]) * entry_forward[0]
                    + (candidate[1] - entry_origin[1]) * entry_forward[1]
                )
                if (
                    forward_progress + 1.0e-9
                    < self.config.min_forward_progress
                    or entry_depth + 1.0e-9
                    < minimum_entry_depth
                ):
                    sector_rejected += 1
                    continue
                if exclusion_radius > 0.0 and any(
                    math.hypot(
                        candidate[0] - excluded[0],
                        candidate[1] - excluded[1],
                    )
                    < exclusion_radius
                    for excluded in excluded_candidates
                ):
                    excluded_rejected += 1
                    continue
                approach_clearance = self._segment_clearance(
                    start, candidate, points
                )
                onward_remaining = math.hypot(
                    goal_x - candidate[0], goal_y - candidate[1]
                )
                onward_distance = min(self.config.horizon, onward_remaining)
                if onward_remaining <= 1.0e-12:
                    onward_end = candidate
                else:
                    onward_end = (
                        candidate[0]
                        + onward_distance
                        * (goal_x - candidate[0])
                        / onward_remaining,
                        candidate[1]
                        + onward_distance
                        * (goal_y - candidate[1])
                        / onward_remaining,
                    )
                geometric_onward_clearance = self._segment_clearance(
                    candidate, onward_end, points
                )
                onward_clearance = geometric_onward_clearance
                # Without an official candidate->destination evaluator, retain
                # the legacy straight-line guard. Runtime supplies that evaluator
                # so a safe curved exit is not rejected before make_plan sees it.
                if (
                    candidate_onward_path_clearance is None
                    and onward_clearance + 1.0e-9
                    < self.config.hard_min_clearance
                ):
                    onward_rejected += 1
                    rejected = (
                        onward_clearance,
                        forward_distance,
                        lateral_shift,
                    )
                    if (
                        best_rejected_onward is None
                        or rejected[0] > best_rejected_onward[0]
                    ):
                        best_rejected_onward = rejected
                    continue
                planned_clearance = approach_clearance
                planned_length = math.hypot(
                    candidate[0] - start[0], candidate[1] - start[1]
                )
                planned_bottleneck_distance = math.nan
                planned_start_clearance = math.nan
                planned_recovery_clearance = math.nan
                used_escape_allowance = False
                if candidate_path_clearance is None:
                    if (
                        approach_clearance + 1.0e-9
                        < self.config.hard_min_clearance
                    ):
                        planned_clearance_rejected += 1
                        continue
                else:
                    evaluation = candidate_path_clearance(candidate)
                    if evaluation is None:
                        no_plan_rejected += 1
                        continue
                    if isinstance(evaluation, PathEvaluation):
                        planned_clearance = evaluation.clearance
                        planned_length = evaluation.length
                        planned_bottleneck_distance = (
                            evaluation.bottleneck_distance
                        )
                        planned_start_clearance = evaluation.start_clearance
                        planned_recovery_clearance = (
                            evaluation.recovery_clearance
                        )
                        if evaluation.points:
                            candidate = evaluation.points[-1]
                    else:
                        planned_clearance = float(evaluation)
                    planned_is_acceptable = (
                        planned_clearance + 1.0e-9
                        >= self.config.hard_min_clearance
                    )
                    if (
                        not planned_is_acceptable
                        and isinstance(evaluation, PathEvaluation)
                        and math.isfinite(planned_start_clearance)
                        and math.isfinite(planned_recovery_clearance)
                        and math.isfinite(planned_bottleneck_distance)
                    ):
                        inherited_floor = max(
                            escape_min_clearance,
                            min(
                                planned_start_clearance,
                                self.config.hard_min_clearance,
                            )
                            - self.config.escape_max_drop,
                        )
                        used_escape_allowance = (
                            planned_clearance + 1.0e-9 >= inherited_floor
                            and planned_bottleneck_distance
                            <= self.config.escape_distance + 1.0e-9
                            and planned_recovery_clearance + 1.0e-9
                            >= self.config.hard_min_clearance
                        )
                        planned_is_acceptable = used_escape_allowance
                    if not planned_is_acceptable:
                        planned_clearance_rejected += 1
                        rejected = (
                            planned_clearance,
                            forward_distance,
                            lateral_shift,
                            planned_bottleneck_distance,
                            planned_start_clearance,
                            planned_recovery_clearance,
                        )
                        if (
                            best_rejected_plan is None
                            or rejected[0] > best_rejected_plan[0]
                        ):
                            best_rejected_plan = rejected
                        continue
                candidate_dx = candidate[0] - start[0]
                candidate_dy = candidate[1] - start[1]
                forward_progress = (
                    candidate_dx * axis_forward[0]
                    + candidate_dy * axis_forward[1]
                )
                lateral_shift = (
                    candidate_dx * axis_lateral[0]
                    + candidate_dy * axis_lateral[1]
                )
                fan_angle = math.atan2(lateral_shift, forward_progress)
                entry_depth = (
                    (candidate[0] - entry_origin[0]) * entry_forward[0]
                    + (candidate[1] - entry_origin[1]) * entry_forward[1]
                )
                if (
                    forward_progress + 1.0e-9
                    < self.config.min_forward_progress
                    or abs(fan_angle)
                    > self.config.fan_half_angle + 1.0e-9
                    or entry_depth + 1.0e-9
                    < minimum_entry_depth
                ):
                    sector_rejected += 1
                    continue
                if exclusion_radius > 0.0 and any(
                    math.hypot(
                        candidate[0] - excluded[0],
                        candidate[1] - excluded[1],
                    )
                    < exclusion_radius
                    for excluded in excluded_candidates
                ):
                    excluded_rejected += 1
                    continue
                if candidate_onward_path_clearance is not None:
                    onward_evaluation = candidate_onward_path_clearance(
                        candidate
                    )
                    if onward_evaluation is None:
                        onward_no_plan_rejected += 1
                        continue
                    if isinstance(onward_evaluation, PathEvaluation):
                        onward_clearance = onward_evaluation.clearance
                    else:
                        onward_clearance = float(onward_evaluation)
                    if (
                        onward_clearance + 1.0e-9
                        < self.config.hard_min_clearance
                    ):
                        onward_rejected += 1
                        rejected = (
                            onward_clearance,
                            forward_distance,
                            lateral_shift,
                        )
                        if (
                            best_rejected_onward is None
                            or rejected[0] > best_rejected_onward[0]
                        ):
                            best_rejected_onward = rejected
                        continue
                effective_clearance = min(
                    planned_clearance, onward_clearance
                )
                deficit = max(
                    0.0, self.config.desired_clearance - effective_clearance
                )
                straight_distance = max(
                    1.0e-9,
                    math.hypot(
                        candidate[0] - start[0], candidate[1] - start[1]
                    ),
                )
                detour_ratio = max(
                    0.0, planned_length / straight_distance - 1.0
                )
                make_plan_goal_adjustment = math.hypot(
                    candidate[0] - requested_candidate[0],
                    candidate[1] - requested_candidate[1],
                )
                score = (
                    self.config.clearance_weight
                    * (deficit / self.config.desired_clearance) ** 2
                    + self.config.angular_weight
                    * (abs(fan_angle) / self.config.fan_half_angle) ** 2
                    + self.config.path_length_weight * detour_ratio**2
                )
                # Clearance dominates. If safety is effectively tied, prefer a
                # farther anchor so a short goal cannot pull the next global plan
                # straight back into the same narrow topology.
                feasible.append(
                    (
                        score,
                        -effective_clearance,
                        -forward_distance,
                        abs(lateral_shift),
                        -lateral_shift,
                        candidate,
                        approach_clearance,
                        onward_clearance,
                        planned_clearance,
                        planned_length,
                        forward_distance,
                        planned_bottleneck_distance,
                        planned_start_clearance,
                        planned_recovery_clearance,
                        used_escape_allowance,
                        make_plan_goal_adjustment,
                        fan_angle,
                        forward_progress,
                        entry_depth,
                        lateral_shift,
                    )
                )

        if not feasible:
            best_plan = "none"
            if best_rejected_plan is not None:
                best_plan = (
                    "clearance={:.3f},forward={:.2f},lateral={:+.2f},"
                    "bottleneck_s={:.3f},start={:.3f},recovery={:.3f}"
                ).format(*best_rejected_plan)
            best_onward = "none"
            if best_rejected_onward is not None:
                best_onward = (
                    "clearance={:.3f},forward={:.2f},lateral={:+.2f}"
                ).format(*best_rejected_onward)
            raise EntrySelectionUnavailable(
                "laser sees a tight delivery corridor (reference clearance "
                "{:.3f} m at s={:.3f} m), but no channel waypoint reaches "
                "{:.3f} m hard clearance; candidates={} onward_rejected={} "
                "onward_no_plan={} no_plan={} planned_clearance_rejected={} "
                "sector_rejected={} excluded_rejected={} "
                "best_plan=[{}] best_onward=[{}]".format(
                    direct_clearance,
                    reference_bottleneck_distance,
                    self.config.hard_min_clearance,
                    candidate_count,
                    onward_rejected,
                    onward_no_plan_rejected,
                    no_plan_rejected,
                    planned_clearance_rejected,
                    sector_rejected,
                    excluded_rejected,
                    best_plan,
                    best_onward,
                )
            )

        (
            _,
            _,
            _,
            _,
            _,
            candidate,
            approach_clearance,
            onward_clearance,
            planned_clearance,
            planned_length,
            forward_distance,
            planned_bottleneck_distance,
            planned_start_clearance,
            planned_recovery_clearance,
            used_escape_allowance,
            make_plan_goal_adjustment,
            fan_angle,
            forward_progress,
            entry_depth,
            lateral_shift,
        ) = min(feasible)
        return ChannelSelection(
            x=candidate[0],
            y=candidate[1],
            lateral_shift=lateral_shift,
            direct_clearance=direct_clearance,
            approach_clearance=approach_clearance,
            onward_clearance=onward_clearance,
            planned_clearance=planned_clearance,
            planned_length=planned_length,
            forward_distance=forward_distance,
            reference_bottleneck_distance=reference_bottleneck_distance,
            planned_bottleneck_distance=planned_bottleneck_distance,
            planned_start_clearance=planned_start_clearance,
            planned_recovery_clearance=planned_recovery_clearance,
            used_escape_allowance=used_escape_allowance,
            make_plan_goal_adjustment=make_plan_goal_adjustment,
            fan_angle=fan_angle,
            forward_progress=forward_progress,
            entry_depth=entry_depth,
            obstacle_points=len(points),
        )


class ClearanceEntrySelector:
    """Choose a nearby waypoint with a continuous laser-clearance cost."""

    def __init__(self, config=None):
        self.config = config or EntrySelectorConfig()
        self.config.validate()

    @staticmethod
    def _clearance(x, y, points):
        if not points:
            return math.inf
        return min(math.hypot(x - px, y - py) for px, py in points)

    def _candidates(self, nominal_x, nominal_y):
        yield nominal_x, nominal_y, 0.0
        rings = int(
            math.ceil(
                self.config.candidate_max_shift / self.config.candidate_step
            )
        )
        for ring in range(1, rings + 1):
            radius = min(
                ring * self.config.candidate_step,
                self.config.candidate_max_shift,
            )
            # Half-step phase alternation avoids repeatedly aligning all rings
            # with the same four Cartesian directions.
            phase = (
                math.pi / self.config.candidate_angle_samples
                if ring % 2 == 0
                else 0.0
            )
            for index in range(self.config.candidate_angle_samples):
                angle = (
                    2.0
                    * math.pi
                    * index
                    / self.config.candidate_angle_samples
                    + phase
                )
                yield (
                    nominal_x + radius * math.cos(angle),
                    nominal_y + radius * math.sin(angle),
                    radius,
                )

    def select(self, nominal_x, nominal_y, obstacle_points):
        if not all(math.isfinite(value) for value in (nominal_x, nominal_y)):
            raise EntrySelectionUnavailable("nominal entry pose is not finite")

        nearby = [
            (float(x), float(y))
            for x, y in obstacle_points
            if math.isfinite(x)
            and math.isfinite(y)
            and math.hypot(x - nominal_x, y - nominal_y)
            <= self.config.influence_radius
        ]
        if not nearby:
            return EntrySelection(
                nominal_x, nominal_y, 0.0, math.inf, 0
            )

        feasible = []
        for x, y, shift in self._candidates(nominal_x, nominal_y):
            clearance = self._clearance(x, y, nearby)
            if clearance + 1e-9 < self.config.hard_min_clearance:
                continue
            deficit = max(0.0, self.config.desired_clearance - clearance)
            score = (
                self.config.shift_weight
                * (shift / self.config.candidate_max_shift) ** 2
                + self.config.clearance_weight
                * (deficit / self.config.desired_clearance) ** 2
            )
            # Deterministic tie breakers: more clearance, then less shift,
            # then coordinates.  Repeated runs with one scan select one point.
            feasible.append((score, -clearance, shift, x, y))

        if not feasible:
            best_clearance = max(
                self._clearance(x, y, nearby)
                for x, y, _ in self._candidates(nominal_x, nominal_y)
            )
            raise EntrySelectionUnavailable(
                "no projected delivery entry pose reaches {:.3f} m hard "
                "clearance; best candidate is {:.3f} m".format(
                    self.config.hard_min_clearance, best_clearance
                )
            )

        _, negative_clearance, shift, x, y = min(feasible)
        return EntrySelection(
            x=x,
            y=y,
            shift=shift,
            clearance=-negative_clearance,
            obstacle_points=len(nearby),
        )


class RosDeliveryEntrySelector:
    """Acquire a fresh scan, transform it, and project an entry PoseStamped."""

    @staticmethod
    def _channel_handoff_is_ready(
        completed_waypoints,
        current_entry_depth,
        reference_clearance,
        previous_onward_clearance,
        minimum_waypoints,
        minimum_entry_depth,
        handoff_clearance,
    ):
        """Return whether ordinary HCP can safely own the remaining route."""
        return (
            completed_waypoints >= minimum_waypoints
            and current_entry_depth + 1.0e-9 >= minimum_entry_depth
            and max(reference_clearance, previous_onward_clearance) + 1.0e-9
            >= handoff_clearance
        )

    @staticmethod
    def _rolling_entry_depth_floor(
        current_entry_depth,
        fixed_floor,
        minimum_progress,
        handoff_depth,
        backtrack_tolerance,
    ):
        """Keep entrance progress without blocking lateral search inside."""
        if current_entry_depth < handoff_depth:
            return max(
                fixed_floor,
                min(handoff_depth, current_entry_depth + minimum_progress),
            )
        return max(
            fixed_floor,
            handoff_depth,
            current_entry_depth - backtrack_tolerance,
        )

    def __init__(self, raw_config=None, tf_buffer=None):
        raw = dict(raw_config or {})
        self.enabled = bool(raw.get("enabled", True))
        self.entry_selection_enabled = bool(
            raw.get("entry_selection_enabled", True)
        )
        self.scan_topic = str(raw.get("scan_topic", "/scan"))
        self.scan_timeout = float(raw.get("scan_timeout", 0.25))
        self.scan_wait_timeout = float(raw.get("scan_wait_timeout", 2.0))
        self.transform_timeout = float(raw.get("transform_timeout", 0.20))
        self.robot_base_frame = str(raw.get("robot_base_frame", "base_link"))
        self.approach_position_tolerance = float(
            raw.get("approach_position_tolerance", 0.35)
        )
        self.minimum_beams = int(raw.get("minimum_beams", 360))
        self.detection_min_range = float(
            raw.get("detection_min_range", 0.08)
        )
        self.detection_max_range = float(
            raw.get("detection_max_range", 1.50)
        )
        config = EntrySelectorConfig(
            candidate_max_shift=float(
                raw.get("candidate_max_shift", 0.30)
            ),
            candidate_step=float(raw.get("candidate_step", 0.03)),
            candidate_angle_samples=int(
                raw.get("candidate_angle_samples", 36)
            ),
            hard_min_clearance=float(
                raw.get("hard_min_clearance", 0.20)
            ),
            desired_clearance=float(
                raw.get("desired_clearance", 0.28)
            ),
            influence_radius=float(raw.get("influence_radius", 0.80)),
            clearance_weight=float(raw.get("clearance_weight", 6.0)),
            shift_weight=float(raw.get("shift_weight", 1.0)),
        )
        self._selector = ClearanceEntrySelector(config)
        self.channel_enabled = bool(raw.get("channel_enabled", True))
        self.channel_max_waypoints = int(raw.get("channel_max_waypoints", 2))
        self.channel_waypoint_position_tolerance = float(
            raw.get("channel_waypoint_position_tolerance", 0.12)
        )
        self.channel_waypoint_yaw_tolerance = math.radians(
            float(raw.get("channel_waypoint_yaw_tolerance_deg", 180.0))
        )
        self.channel_handoff_min_waypoints = int(
            raw.get("channel_handoff_min_waypoints", 2)
        )
        self.channel_handoff_min_entry_depth = float(
            raw.get("channel_handoff_min_entry_depth", 0.90)
        )
        self.channel_handoff_clearance = float(
            raw.get("channel_handoff_clearance", 0.28)
        )
        self.channel_handoff_position_tolerance = float(
            raw.get("channel_handoff_position_tolerance", 0.15)
        )
        self.channel_handoff_yaw_tolerance = math.radians(
            float(raw.get("channel_handoff_yaw_tolerance_deg", 180.0))
        )
        self.channel_waypoint_timeout = float(
            raw.get("channel_waypoint_timeout", 45.0)
        )
        self.channel_handoff_timeout = float(
            raw.get("channel_handoff_timeout", 88.0)
        )
        self.channel_rollback_timeout = float(
            raw.get("channel_rollback_timeout", 45.0)
        )
        self.channel_max_reselections = int(
            raw.get("channel_max_reselections", 2)
        )
        # When the FIRST rolling waypoint times out or aborts (the entry
        # bottleneck), the previous confirmed navigation point is the
        # laser-selected safe entry pose. Physically return to it before
        # re-selecting so the fan starts from a clean, un-jammed position
        # instead of repeating candidates around the stuck spot.
        self.channel_first_waypoint_rollback = bool(
            raw.get("channel_first_waypoint_rollback", True)
        )
        self.channel_failed_candidate_exclusion_radius = float(
            raw.get("channel_failed_candidate_exclusion_radius", 0.30)
        )
        self.channel_reselection_escape_min_clearance = float(
            raw.get("channel_reselection_escape_min_clearance", 0.16)
        )
        self.channel_make_plan_service = str(
            raw.get("channel_make_plan_service", "/move_base/make_plan")
        )
        self.channel_make_plan_wait_timeout = float(
            raw.get("channel_make_plan_wait_timeout", 1.0)
        )
        self.channel_make_plan_tolerance = float(
            raw.get("channel_make_plan_tolerance", 0.0)
        )
        self.channel_avoidance_lock_service = str(
            raw.get(
                "channel_avoidance_lock_service",
                "/move_base/AdaptiveTebLocalPlannerROS/"
                "set_avoidance_lock",
            )
        )
        self.channel_avoidance_lock_wait_timeout = float(
            raw.get("channel_avoidance_lock_wait_timeout", 1.0)
        )
        self.preparation_baseline_lock_service = str(
            raw.get(
                "preparation_baseline_lock_service",
                "/move_base/AdaptiveTebLocalPlannerROS/set_baseline_lock",
            )
        )
        self.preparation_baseline_lock_wait_timeout = float(
            raw.get("preparation_baseline_lock_wait_timeout", 1.0)
        )
        self.channel_entry_axis_waypoints = int(
            raw.get("channel_entry_axis_waypoints", 1)
        )
        self.channel_min_entry_depth_progress = float(
            raw.get("channel_min_entry_depth_progress", 0.0)
        )
        self.channel_entry_depth_backtrack_tolerance = float(
            raw.get("channel_entry_depth_backtrack_tolerance", 0.0)
        )
        raw_fan_radii = raw.get(
            "channel_fan_radii",
            raw.get(
                "channel_forward_distances",
                (raw.get("channel_horizon", 0.55),),
            ),
        )
        if not isinstance(raw_fan_radii, (list, tuple)):
            raise ValueError("channel_fan_radii must be a list")
        channel_config = ChannelSelectorConfig(
            horizon=float(raw.get("channel_horizon", 0.55)),
            fan_radii=tuple(
                float(value) for value in raw_fan_radii
            ),
            trigger_lookahead=float(
                raw.get("channel_trigger_lookahead", 1.00)
            ),
            fan_half_angle=math.radians(
                float(raw.get("channel_fan_half_angle_deg", 60.0))
            ),
            fan_angle_step=math.radians(
                float(raw.get("channel_fan_angle_step_deg", 10.0))
            ),
            min_forward_progress=float(
                raw.get("channel_min_forward_progress", 0.20)
            ),
            entry_depth_floor=float(
                raw.get("channel_entry_depth_floor", 0.20)
            ),
            trigger_clearance=float(
                raw.get("channel_trigger_clearance", 0.30)
            ),
            hard_min_clearance=float(
                raw.get("channel_hard_min_clearance", 0.20)
            ),
            desired_clearance=float(
                raw.get("channel_desired_clearance", 0.36)
            ),
            clearance_weight=float(
                raw.get("channel_clearance_weight", 20.0)
            ),
            angular_weight=float(
                raw.get(
                    "channel_angular_weight",
                    raw.get("channel_lateral_weight", 0.10),
                )
            ),
            path_length_weight=float(
                raw.get("channel_path_length_weight", 0.05)
            ),
            escape_distance=float(
                raw.get("channel_escape_distance", 0.25)
            ),
            escape_min_clearance=float(
                raw.get("channel_escape_min_clearance", 0.18)
            ),
            escape_max_drop=float(
                raw.get("channel_escape_max_drop", 0.02)
            ),
        )
        self._channel_selector = ClearanceChannelSelector(channel_config)
        self._validate_ros_config()

        self._condition = threading.Condition()
        self._latest_scan = None
        self._retained_delivery_points = []
        self._last_channel_selection = None
        self._channel_selection_history = []
        self._blocked_channel_waypoints = []
        self._tf_buffer = tf_buffer or tf2_ros.Buffer(
            cache_time=rospy.Duration(10.0)
        )
        self._tf_listener = (
            None if tf_buffer is not None else tf2_ros.TransformListener(
                self._tf_buffer
            )
        )
        self._subscriber = None
        self._make_plan = None
        self._set_avoidance_lock = None
        self._set_baseline_lock = None
        if self.enabled:
            if self.entry_selection_enabled or self.channel_enabled:
                self._subscriber = rospy.Subscriber(
                    self.scan_topic,
                    LaserScan,
                    self._scan_callback,
                    queue_size=1,
                )
            if self.channel_enabled:
                self._make_plan = rospy.ServiceProxy(
                    self.channel_make_plan_service, GetPlan
                )
            # The avoidance lock is also used by direct-goal delivery when the
            # rolling channel selector is disabled.
            self._set_avoidance_lock = rospy.ServiceProxy(
                self.channel_avoidance_lock_service, SetBool
            )
            self._set_baseline_lock = rospy.ServiceProxy(
                self.preparation_baseline_lock_service, SetBool
            )

    def _validate_ros_config(self):
        numeric = (
            self.scan_timeout,
            self.scan_wait_timeout,
            self.transform_timeout,
            self.approach_position_tolerance,
            self.channel_waypoint_position_tolerance,
            self.channel_waypoint_yaw_tolerance,
            self.channel_handoff_min_entry_depth,
            self.channel_handoff_clearance,
            self.channel_handoff_position_tolerance,
            self.channel_handoff_yaw_tolerance,
            self.channel_waypoint_timeout,
            self.channel_handoff_timeout,
            self.channel_rollback_timeout,
            self.channel_failed_candidate_exclusion_radius,
            self.channel_reselection_escape_min_clearance,
            self.channel_make_plan_wait_timeout,
            self.channel_make_plan_tolerance,
            self.channel_avoidance_lock_wait_timeout,
            self.preparation_baseline_lock_wait_timeout,
            self.channel_min_entry_depth_progress,
            self.channel_entry_depth_backtrack_tolerance,
            self.detection_min_range,
            self.detection_max_range,
        )
        if not all(math.isfinite(value) for value in numeric):
            raise ValueError("delivery entry ROS selector values must be finite")
        if not self.scan_topic:
            raise ValueError("scan_topic must not be empty")
        if not self.robot_base_frame:
            raise ValueError("robot_base_frame must not be empty")
        if not self.channel_make_plan_service:
            raise ValueError("channel_make_plan_service must not be empty")
        if not self.channel_avoidance_lock_service:
            raise ValueError(
                "channel_avoidance_lock_service must not be empty"
            )
        if not self.preparation_baseline_lock_service:
            raise ValueError(
                "preparation_baseline_lock_service must not be empty"
            )
        if self.scan_timeout <= 0.0 or self.scan_wait_timeout <= 0.0:
            raise ValueError("scan timeouts must be positive")
        if self.transform_timeout <= 0.0:
            raise ValueError("transform_timeout must be positive")
        if self.approach_position_tolerance <= 0.0:
            raise ValueError("approach_position_tolerance must be positive")
        if self.channel_max_waypoints <= 0:
            raise ValueError("channel_max_waypoints must be positive")
        if not 1 <= self.channel_handoff_min_waypoints <= self.channel_max_waypoints:
            raise ValueError(
                "channel_handoff_min_waypoints must be between one and "
                "channel_max_waypoints"
            )
        if not 0 <= self.channel_entry_axis_waypoints <= self.channel_max_waypoints:
            raise ValueError(
                "channel_entry_axis_waypoints must be between zero and "
                "channel_max_waypoints"
            )
        if self.channel_min_entry_depth_progress < 0.0:
            raise ValueError(
                "channel_min_entry_depth_progress must be nonnegative"
            )
        if self.channel_entry_depth_backtrack_tolerance < 0.0:
            raise ValueError(
                "channel_entry_depth_backtrack_tolerance must be "
                "nonnegative"
            )
        if self.channel_waypoint_position_tolerance <= 0.0:
            raise ValueError(
                "channel_waypoint_position_tolerance must be positive"
            )
        if not 0.0 < self.channel_waypoint_yaw_tolerance <= math.pi:
            raise ValueError(
                "channel_waypoint_yaw_tolerance_deg must be in (0, 180]"
            )
        if (
            self.channel_handoff_min_entry_depth
            < self._channel_selector.config.entry_depth_floor
        ):
            raise ValueError(
                "channel_handoff_min_entry_depth must be at least "
                "channel_entry_depth_floor"
            )
        if not (
            self._channel_selector.config.hard_min_clearance
            <= self.channel_handoff_clearance
            <= self._channel_selector.config.trigger_clearance
        ):
            raise ValueError(
                "channel_handoff_clearance must be between channel hard and "
                "trigger clearances"
            )
        if self.channel_handoff_position_tolerance <= 0.0:
            raise ValueError(
                "channel_handoff_position_tolerance must be positive"
            )
        if (
            self.channel_waypoint_timeout <= 0.0
            or self.channel_handoff_timeout <= 0.0
            or self.channel_rollback_timeout <= 0.0
        ):
            raise ValueError("channel navigation timeouts must be positive")
        if self.channel_max_reselections < 0:
            raise ValueError("channel_max_reselections must be nonnegative")
        if self.channel_failed_candidate_exclusion_radius <= 0.0:
            raise ValueError(
                "channel_failed_candidate_exclusion_radius must be positive"
            )
        if not (
            0.0 < self.channel_reselection_escape_min_clearance
            <= self._channel_selector.config.escape_min_clearance
        ):
            raise ValueError(
                "channel_reselection_escape_min_clearance must be positive "
                "and no larger than channel_escape_min_clearance"
            )
        if not 0.0 < self.channel_handoff_yaw_tolerance <= math.pi:
            raise ValueError(
                "channel_handoff_yaw_tolerance_deg must be in (0, 180]"
            )
        if self.channel_make_plan_wait_timeout <= 0.0:
            raise ValueError("channel_make_plan_wait_timeout must be positive")
        if self.channel_make_plan_tolerance < 0.0:
            raise ValueError("channel_make_plan_tolerance must be nonnegative")
        if self.channel_avoidance_lock_wait_timeout <= 0.0:
            raise ValueError(
                "channel_avoidance_lock_wait_timeout must be positive"
            )
        if self.preparation_baseline_lock_wait_timeout <= 0.0:
            raise ValueError(
                "preparation_baseline_lock_wait_timeout must be positive"
            )
        if self.minimum_beams <= 0:
            raise ValueError("minimum_beams must be positive")
        if not 0.0 <= self.detection_min_range < self.detection_max_range:
            raise ValueError("invalid delivery entry detection range")

    def _scan_callback(self, message):
        with self._condition:
            self._latest_scan = message
            self._condition.notify_all()

    def _fresh_scan(self, preempt_requested):
        deadline = time.monotonic() + self.scan_wait_timeout
        while not rospy.is_shutdown():
            if preempt_requested is not None and preempt_requested():
                raise EntrySelectionPreempted(
                    "delivery entry selection was preempted"
                )
            with self._condition:
                scan = self._latest_scan
                now = rospy.Time.now()
                if scan is not None and len(scan.ranges) >= self.minimum_beams:
                    age = (now - scan.header.stamp).to_sec()
                    if -0.05 <= age <= self.scan_timeout:
                        return scan
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    break
                self._condition.wait(timeout=min(0.05, remaining))
        raise EntrySelectionUnavailable(
            "no fresh {}-beam laser scan on {} within {:.2f} s".format(
                self.minimum_beams,
                self.scan_topic,
                self.scan_wait_timeout,
            )
        )

    @staticmethod
    def _yaw(quaternion):
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

    def _obstacle_points(self, scan, target_frame):
        try:
            transform = self._tf_buffer.lookup_transform(
                target_frame,
                scan.header.frame_id,
                scan.header.stamp,
                rospy.Duration(self.transform_timeout),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:
            raise EntrySelectionUnavailable(
                "cannot transform delivery-entry scan from {} to {}: {}".format(
                    scan.header.frame_id, target_frame, exc
                )
            )

        translation = transform.transform.translation
        yaw = self._yaw(transform.transform.rotation)
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        points = []
        angle = scan.angle_min
        lower = max(scan.range_min, self.detection_min_range)
        upper = min(scan.range_max, self.detection_max_range)
        for distance in scan.ranges:
            if math.isfinite(distance) and lower <= distance <= upper:
                local_x = distance * math.cos(angle)
                local_y = distance * math.sin(angle)
                points.append(
                    (
                        translation.x + cosine * local_x - sine * local_y,
                        translation.y + sine * local_x + cosine * local_y,
                    )
                )
            angle += scan.angle_increment
        if not points:
            raise EntrySelectionUnavailable(
                "fresh delivery-entry scan contains no usable obstacle points"
            )
        return points, (translation.x, translation.y)

    def _robot_origin(self, target_frame, stamp):
        try:
            transform = self._tf_buffer.lookup_transform(
                target_frame,
                self.robot_base_frame,
                stamp,
                rospy.Duration(self.transform_timeout),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:
            raise EntrySelectionUnavailable(
                "cannot transform delivery robot origin from {} to {}: {}".format(
                    self.robot_base_frame, target_frame, exc
                )
            )
        return (
            transform.transform.translation.x,
            transform.transform.translation.y,
        )

    def _official_path_evaluation(
        self,
        start,
        candidate,
        frame_id,
        stamp,
        obstacle_points,
        evaluation_horizon=None,
        recovery_distance=None,
    ):
        request = GetPlanRequest()
        request.start = PoseStamped()
        request.start.header.frame_id = frame_id
        request.start.header.stamp = stamp
        request.start.pose.position.x = start[0]
        request.start.pose.position.y = start[1]
        request.start.pose.orientation.w = 1.0
        request.goal = PoseStamped()
        request.goal.header.frame_id = frame_id
        request.goal.header.stamp = stamp
        request.goal.pose.position.x = candidate[0]
        request.goal.pose.position.y = candidate[1]
        heading = math.atan2(candidate[1] - start[1], candidate[0] - start[0])
        request.goal.pose.orientation.z = math.sin(0.5 * heading)
        request.goal.pose.orientation.w = math.cos(0.5 * heading)
        request.tolerance = self.channel_make_plan_tolerance
        try:
            response = self._make_plan(request)
        except rospy.ServiceException as exc:
            rospy.logwarn(
                "delivery channel make_plan candidate (%.3f, %.3f) failed: %s",
                candidate[0],
                candidate[1],
                exc,
            )
            return None
        path_points = [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in response.plan.poses
        ]
        if len(path_points) < 2:
            return None
        path_length = ClearanceChannelSelector._path_length(path_points)
        evaluation_length = path_length
        if evaluation_horizon is not None:
            evaluation_length = min(float(evaluation_horizon), path_length)
        bottleneck = ClearanceChannelSelector._path_prefix_measurement(
            path_points,
            evaluation_length,
            obstacle_points,
        )
        start_clearance = ClearanceChannelSelector._segment_clearance(
            path_points[0], path_points[0], obstacle_points
        )
        recovery_clearance = math.nan
        if (
            recovery_distance is not None
            and float(recovery_distance) + 1.0e-9 < evaluation_length
        ):
            recovery_clearance = (
                ClearanceChannelSelector._path_window_measurement(
                    path_points,
                    float(recovery_distance),
                    evaluation_length,
                    obstacle_points,
                )[0]
            )
        return PathEvaluation(
            clearance=bottleneck[0],
            length=path_length,
            points=tuple(path_points),
            bottleneck_distance=bottleneck[1],
            start_clearance=start_clearance,
            recovery_clearance=recovery_clearance,
        )

    def resolve(self, nominal_pose, preempt_requested=None):
        if not self.enabled or not self.entry_selection_enabled:
            return nominal_pose
        # A new delivery entry starts a new rolling-channel decision sequence.
        self._last_channel_selection = None
        self._channel_selection_history = []
        self._blocked_channel_waypoints = []
        frame_id = nominal_pose.header.frame_id.strip()
        if not frame_id:
            raise EntrySelectionUnavailable(
                "delivery entry pose frame_id must not be empty"
            )
        scan = self._fresh_scan(preempt_requested)
        points, _ = self._obstacle_points(scan, frame_id)
        nominal_x = nominal_pose.pose.position.x
        nominal_y = nominal_pose.pose.position.y
        selected = self._selector.select(nominal_x, nominal_y, points)
        # Keep the entry view for the rolling channel selector. A cone can be
        # hidden by the carried cube or robot body after the heading changes;
        # map-frame scan points remain valid for this short, static delivery.
        self._retained_delivery_points = list(points)

        resolved = copy.deepcopy(nominal_pose)
        resolved.header.stamp = rospy.Time.now()
        resolved.pose.position.x = selected.x
        resolved.pose.position.y = selected.y
        rospy.loginfo(
            "delivery entry laser projection: nominal=(%.3f, %.3f) "
            "selected=(%.3f, %.3f) shift=%.3f clearance=%.3f "
            "nearby_points=%d",
            nominal_x,
            nominal_y,
            selected.x,
            selected.y,
            selected.shift,
            selected.clearance,
            selected.obstacle_points,
        )
        return resolved

    def resolve_channel_waypoint(
        self,
        destination_pose,
        entry_pose,
        channel_index,
        preempt_requested=None,
        force_selection=False,
    ):
        """Return a laser-selected short-horizon goal, or None if already wide."""
        if not self.enabled or not self.channel_enabled:
            return None
        frame_id = destination_pose.header.frame_id.strip()
        if not frame_id:
            raise EntrySelectionUnavailable(
                "delivery destination frame_id must not be empty"
            )
        if entry_pose.header.frame_id.strip() != frame_id:
            raise EntrySelectionUnavailable(
                "delivery entry and destination poses must use the same frame"
            )
        if channel_index < 0:
            raise EntrySelectionUnavailable(
                "delivery channel waypoint index must be nonnegative"
            )
        scan = self._fresh_scan(preempt_requested)
        current_points, _ = self._obstacle_points(scan, frame_id)
        points = self._retained_delivery_points + current_points
        robot_origin = self._robot_origin(frame_id, scan.header.stamp)
        entry_yaw = self._yaw(entry_pose.pose.orientation)
        if channel_index < self.channel_entry_axis_waypoints:
            candidate_axis_yaw = entry_yaw
            candidate_axis_phase = "entry"
        else:
            candidate_axis_yaw = math.atan2(
                destination_pose.pose.position.y - robot_origin[1],
                destination_pose.pose.position.x - robot_origin[0],
            )
            candidate_axis_phase = "destination"
        try:
            rospy.wait_for_service(
                self.channel_make_plan_service,
                timeout=self.channel_make_plan_wait_timeout,
            )
        except rospy.ROSException as exc:
            raise EntrySelectionUnavailable(
                "official GlobalPlanner make_plan service {} is unavailable: {}".format(
                    self.channel_make_plan_service, exc
                )
            )
        reference_evaluation = self._official_path_evaluation(
            robot_origin,
            (
                destination_pose.pose.position.x,
                destination_pose.pose.position.y,
            ),
            frame_id,
            scan.header.stamp,
            points,
        )
        reference_path_points = (
            reference_evaluation.points
            if reference_evaluation is not None
            else None
        )
        entry_forward = (math.cos(entry_yaw), math.sin(entry_yaw))
        current_entry_depth = (
            (robot_origin[0] - entry_pose.pose.position.x) * entry_forward[0]
            + (robot_origin[1] - entry_pose.pose.position.y) * entry_forward[1]
        )
        reference_clearance = -math.inf
        if reference_path_points is not None:
            reference_path = self._channel_selector._sanitized_path(
                reference_path_points,
                robot_origin,
                (
                    destination_pose.pose.position.x,
                    destination_pose.pose.position.y,
                ),
            )
            reference_length = self._channel_selector._path_length(
                reference_path
            )
            reference_clearance = (
                self._channel_selector._path_prefix_clearance(
                    reference_path,
                    min(
                        self._channel_selector.config.trigger_lookahead,
                        reference_length,
                    ),
                    points,
                )
            )
        previous_onward_clearance = (
            self._last_channel_selection.onward_clearance
            if self._last_channel_selection is not None
            else -math.inf
        )
        if not force_selection and self._channel_handoff_is_ready(
            channel_index,
            current_entry_depth,
            reference_clearance,
            previous_onward_clearance,
            self.channel_handoff_min_waypoints,
            self.channel_handoff_min_entry_depth,
            self.channel_handoff_clearance,
        ):
            rospy.loginfo(
                "delivery channel dynamic-selector handoff: completed=%d "
                "entry_depth=%.3f reference_clearance=%.3f "
                "previous_onward_clearance=%.3f handoff_clearance=%.3f; "
                "ordinary avoidance HCP-TEB owns the remaining route",
                channel_index,
                current_entry_depth,
                reference_clearance,
                previous_onward_clearance,
                self.channel_handoff_clearance,
            )
            return None
        if reference_path_points is None:
            rospy.logwarn(
                "official GlobalPlanner has no current delivery-destination "
                "path; channel selector is falling back to straight reference "
                "geometry while still requiring every candidate to be "
                "officially reachable"
            )
        rolling_entry_depth_floor = self._rolling_entry_depth_floor(
            current_entry_depth,
            self._channel_selector.config.entry_depth_floor,
            self.channel_min_entry_depth_progress,
            self.channel_handoff_min_entry_depth,
            self.channel_entry_depth_backtrack_tolerance,
        )
        selected = self._channel_selector.select(
            robot_origin[0],
            robot_origin[1],
            destination_pose.pose.position.x,
            destination_pose.pose.position.y,
            points,
            candidate_path_clearance=lambda candidate: (
                self._official_path_evaluation(
                    robot_origin,
                    candidate,
                    frame_id,
                    scan.header.stamp,
                    points,
                    recovery_distance=(
                        self._channel_selector.config.escape_distance
                    ),
                )
            ),
            candidate_onward_path_clearance=lambda candidate: (
                self._official_path_evaluation(
                    candidate,
                    (
                        destination_pose.pose.position.x,
                        destination_pose.pose.position.y,
                    ),
                    frame_id,
                    scan.header.stamp,
                    points,
                    evaluation_horizon=(
                        self._channel_selector.config.horizon
                    ),
                )
            ),
            reference_path_points=reference_path_points,
            candidate_axis_yaw=candidate_axis_yaw,
            entry_origin=(
                entry_pose.pose.position.x,
                entry_pose.pose.position.y,
            ),
            entry_axis_yaw=entry_yaw,
            minimum_entry_depth=rolling_entry_depth_floor,
            force_selection=force_selection,
            excluded_candidates=self._blocked_channel_waypoints,
            exclusion_radius=(
                self.channel_failed_candidate_exclusion_radius
            ),
            escape_min_clearance=(
                self.channel_reselection_escape_min_clearance
                if force_selection
                else self._channel_selector.config.escape_min_clearance
            ),
        )
        if selected is None:
            if reference_path_points is None:
                rospy.loginfo(
                    "delivery channel laser check: no intermediate waypoint "
                    "required; straight reference or remaining distance did "
                    "not require a fan candidate"
                )
            else:
                reference_path = self._channel_selector._sanitized_path(
                    reference_path_points,
                    robot_origin,
                    (
                        destination_pose.pose.position.x,
                        destination_pose.pose.position.y,
                    ),
                )
                reference_length = self._channel_selector._path_length(
                    reference_path
                )
                reference_clearance, bottleneck_distance = (
                    self._channel_selector._path_prefix_measurement(
                        reference_path,
                        min(
                            self._channel_selector.config.trigger_lookahead,
                            reference_length,
                        ),
                        points,
                    )
                )
                remaining_distance = math.hypot(
                    destination_pose.pose.position.x - robot_origin[0],
                    destination_pose.pose.position.y - robot_origin[1],
                )
                if (
                    reference_clearance + 1.0e-9
                    >= self._channel_selector.config.trigger_clearance
                ):
                    rospy.loginfo(
                        "delivery channel laser check: official path "
                        "short-horizon corridor is wide; "
                        "reference_clearance=%.3f bottleneck_s=%.3f "
                        "trigger_clearance=%.3f remaining=%.3f; no "
                        "intermediate waypoint required",
                        reference_clearance,
                        bottleneck_distance,
                        self._channel_selector.config.trigger_clearance,
                        remaining_distance,
                    )
                else:
                    rospy.loginfo(
                        "delivery channel laser check: path remains below "
                        "the clearance trigger, but the destination is "
                        "inside the smallest usable fan radius; "
                        "reference_clearance=%.3f bottleneck_s=%.3f "
                        "trigger_clearance=%.3f remaining=%.3f "
                        "smallest_fan_radius=%.3f; handing off the short "
                        "final segment",
                        reference_clearance,
                        bottleneck_distance,
                        self._channel_selector.config.trigger_clearance,
                        remaining_distance,
                        min(
                            self._channel_selector.config.resolved_fan_radii
                        ),
                    )
            return None

        self._retained_delivery_points.extend(current_points)
        self._last_channel_selection = selected
        self._channel_selection_history.append(selected)

        resolved = copy.deepcopy(destination_pose)
        resolved.header.stamp = rospy.Time.now()
        resolved.pose.position.x = selected.x
        resolved.pose.position.y = selected.y
        # Orient the rolling goal along its incoming path. Pointing it from the
        # waypoint to the distant workshop encouraged the holonomic base to
        # keep the future heading and strafe sideways through the cone gap.
        heading = math.atan2(
            selected.y - robot_origin[1],
            selected.x - robot_origin[0],
        )
        resolved.pose.orientation.x = 0.0
        resolved.pose.orientation.y = 0.0
        resolved.pose.orientation.z = math.sin(0.5 * heading)
        resolved.pose.orientation.w = math.cos(0.5 * heading)
        rospy.loginfo(
            "delivery channel laser waypoint: selected=(%.3f, %.3f) "
            "phase=%s axis_yaw_deg=%.1f radius=%.3f fan_angle_deg=%+.1f "
            "forward=%.3f lateral_shift=%+.3f entry_depth=%.3f "
            "required_entry_depth=%.3f "
            "reference_clearance=%.3f "
            "approach_clearance=%.3f planned_clearance=%.3f "
            "reference_bottleneck_s=%.3f planned_bottleneck_s=%.3f "
            "planned_start_clearance=%.3f recovery_clearance=%.3f "
            "escape_allowance=%s goal_adjustment=%.3f planned_length=%.3f "
            "onward_clearance=%.3f points=%d "
            "current_points=%d forced=%s blocked_candidates=%d",
            selected.x,
            selected.y,
            candidate_axis_phase,
            math.degrees(candidate_axis_yaw),
            selected.forward_distance,
            math.degrees(selected.fan_angle),
            selected.forward_progress,
            selected.lateral_shift,
            selected.entry_depth,
            rolling_entry_depth_floor,
            selected.direct_clearance,
            selected.approach_clearance,
            selected.planned_clearance,
            selected.reference_bottleneck_distance,
            selected.planned_bottleneck_distance,
            selected.planned_start_clearance,
            selected.planned_recovery_clearance,
            selected.used_escape_allowance,
            selected.make_plan_goal_adjustment,
            selected.planned_length,
            selected.onward_clearance,
            selected.obstacle_points,
            len(current_points),
            force_selection,
            len(self._blocked_channel_waypoints),
        )
        return resolved

    def reject_channel_waypoint(self, pose):
        """Block one timed-out rolling target and restore prior history."""
        rejected = (
            float(pose.pose.position.x),
            float(pose.pose.position.y),
        )
        self._blocked_channel_waypoints.append(rejected)
        if self._channel_selection_history:
            latest = self._channel_selection_history[-1]
            if math.hypot(latest.x - rejected[0], latest.y - rejected[1]) < 0.05:
                self._channel_selection_history.pop()
        self._last_channel_selection = (
            self._channel_selection_history[-1]
            if self._channel_selection_history
            else None
        )
        rospy.logwarn(
            "delivery channel timed-out candidate blocked at (%.3f, %.3f) "
            "with radius %.3fm; blocked_candidates=%d",
            rejected[0],
            rejected[1],
            self.channel_failed_candidate_exclusion_radius,
            len(self._blocked_channel_waypoints),
        )

    def set_channel_avoidance_lock(self, enabled):
        """Pin the conservative TEB for cone-zone delivery navigation."""
        if not self.enabled:
            return
        try:
            rospy.wait_for_service(
                self.channel_avoidance_lock_service,
                timeout=self.channel_avoidance_lock_wait_timeout,
            )
            response = self._set_avoidance_lock(bool(enabled))
        except (rospy.ROSException, rospy.ServiceException) as exc:
            raise EntrySelectionUnavailable(
                "adaptive TEB avoidance lock service {} failed: {}".format(
                    self.channel_avoidance_lock_service, exc
                )
            )
        if not response.success:
            raise EntrySelectionUnavailable(
                "adaptive TEB did not confirm avoidance lock {}: {}".format(
                    "enabled" if enabled else "disabled",
                    response.message,
                )
            )
        rospy.loginfo(
            "delivery channel avoidance lock %s: %s",
            "enabled" if enabled else "disabled",
            response.message,
        )

    def set_preparation_baseline_lock(self, enabled):
        """Keep the early mission on the small-footprint baseline planner."""
        if not self.enabled:
            return
        try:
            rospy.wait_for_service(
                self.preparation_baseline_lock_service,
                timeout=self.preparation_baseline_lock_wait_timeout,
            )
            response = self._set_baseline_lock(bool(enabled))
        except (rospy.ROSException, rospy.ServiceException) as exc:
            raise EntrySelectionUnavailable(
                "adaptive TEB baseline lock service {} failed: {}".format(
                    self.preparation_baseline_lock_service, exc
                )
            )
        if not response.success:
            raise EntrySelectionUnavailable(
                "adaptive TEB did not confirm baseline lock {}: {}".format(
                    "enabled" if enabled else "disabled",
                    response.message,
                )
            )
        rospy.loginfo(
            "preparation baseline lock %s: %s",
            "enabled" if enabled else "disabled",
            response.message,
        )

    @property
    def requires_approach(self):
        """Whether the mission should enter laser visibility before resolve."""
        return self.enabled and self.entry_selection_enabled
