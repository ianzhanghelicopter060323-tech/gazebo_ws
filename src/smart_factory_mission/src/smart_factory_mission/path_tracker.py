"""Pure geometry for following an offline-fitted navigation reference path."""

from dataclasses import dataclass
import bisect
import math


class PathConfigError(ValueError):
    pass


@dataclass(frozen=True)
class PathPoint:
    s: float
    x: float
    y: float
    yaw: float
    curvature: float
    source_seq_start: int
    source_seq_end: int


@dataclass(frozen=True)
class PathProjection:
    s: float
    x: float
    y: float
    cross_track_error: float
    segment_index: int


@dataclass(frozen=True)
class TrackingTarget:
    progress_s: float
    cross_track_error: float
    lookahead: float
    target: PathPoint


@dataclass(frozen=True)
class HeadingLock:
    start_s: float
    full_lock_s: float
    release_start_s: float
    end_s: float
    yaw: float

    def weight(self, progress_s):
        if progress_s <= self.start_s or progress_s >= self.end_s:
            return 0.0
        if progress_s < self.full_lock_s:
            fraction = (progress_s - self.start_s) / (
                self.full_lock_s - self.start_s
            )
            return _smoothstep(fraction)
        if progress_s <= self.release_start_s:
            return 1.0
        fraction = (progress_s - self.release_start_s) / (
            self.end_s - self.release_start_s
        )
        return 1.0 - _smoothstep(fraction)


def _smoothstep(value):
    value = max(0.0, min(1.0, float(value)))
    return value * value * (3.0 - 2.0 * value)


def _angle_lerp(start, end, fraction):
    delta = math.atan2(math.sin(end - start), math.cos(end - start))
    return math.atan2(
        math.sin(start + fraction * delta),
        math.cos(start + fraction * delta),
    )


def _finite_float(value, label):
    try:
        result = float(value)
    except (TypeError, ValueError):
        raise PathConfigError("{} must be numeric".format(label))
    if not math.isfinite(result):
        raise PathConfigError("{} must be finite".format(label))
    return result


def _positive_seq(value, label):
    if isinstance(value, bool):
        raise PathConfigError("{} must be a positive integer".format(label))
    try:
        result = int(value)
    except (TypeError, ValueError):
        raise PathConfigError("{} must be a positive integer".format(label))
    if result <= 0 or result != value:
        raise PathConfigError("{} must be a positive integer".format(label))
    return result


class FittedPath:
    """Validated polyline with arc-length interpolation and seq landmarks."""

    def __init__(self, frame_id, points, anchor_s, anchor_xy, final_goal):
        self.frame_id = frame_id
        self.points = tuple(points)
        self.anchor_s = dict(anchor_s)
        self.anchor_xy = dict(anchor_xy)
        self.final_goal = tuple(final_goal)
        self._arc = tuple(point.s for point in self.points)
        self.total_length = self._arc[-1]

    @classmethod
    def from_config(cls, config):
        if not isinstance(config, dict) or not config.get("configured", False):
            raise PathConfigError("fitted path is not configured")
        frame_id = config.get("frame_id", "map")
        if not isinstance(frame_id, str) or not frame_id.strip():
            raise PathConfigError("fitted path frame_id must not be empty")

        raw_points = config.get("points")
        if not isinstance(raw_points, list) or len(raw_points) < 2:
            raise PathConfigError("fitted path must contain at least two points")
        points = []
        previous_s = None
        for index, raw in enumerate(raw_points):
            if not isinstance(raw, dict):
                raise PathConfigError("fitted path point {} must be a mapping".format(index))
            point = PathPoint(
                s=_finite_float(raw.get("s"), "point s"),
                x=_finite_float(raw.get("x"), "point x"),
                y=_finite_float(raw.get("y"), "point y"),
                yaw=_finite_float(raw.get("yaw"), "point yaw"),
                curvature=_finite_float(raw.get("curvature"), "point curvature"),
                source_seq_start=_positive_seq(
                    raw.get("source_seq_start"), "source_seq_start"
                ),
                source_seq_end=_positive_seq(
                    raw.get("source_seq_end"), "source_seq_end"
                ),
            )
            if previous_s is not None and point.s <= previous_s:
                raise PathConfigError("fitted path s values must increase strictly")
            previous_s = point.s
            points.append(point)
        if abs(points[0].s) > 1.0e-6:
            raise PathConfigError("fitted path must start at s=0")

        anchor_s = {}
        anchor_xy = {}
        raw_anchors = config.get("anchors")
        if not isinstance(raw_anchors, list) or len(raw_anchors) < 2:
            raise PathConfigError("fitted path must contain seq anchors")
        for raw in raw_anchors:
            if not isinstance(raw, dict):
                raise PathConfigError("fitted path anchor must be a mapping")
            sequence = _positive_seq(raw.get("seq"), "anchor seq")
            arc = _finite_float(raw.get("s"), "anchor s")
            anchor_x = _finite_float(raw.get("x"), "anchor x")
            anchor_y = _finite_float(raw.get("y"), "anchor y")
            if sequence in anchor_s:
                raise PathConfigError("duplicate fitted path anchor seq {}".format(sequence))
            if arc < -1.0e-6 or arc > points[-1].s + 1.0e-6:
                raise PathConfigError("anchor seq {} lies outside the path".format(sequence))
            anchor_s[sequence] = max(0.0, min(points[-1].s, arc))
            anchor_xy[sequence] = (anchor_x, anchor_y)

        raw_final = config.get("final_goal")
        if not isinstance(raw_final, dict):
            raise PathConfigError("fitted path final_goal must be configured")
        final_goal = (
            _finite_float(raw_final.get("x"), "final_goal x"),
            _finite_float(raw_final.get("y"), "final_goal y"),
            _finite_float(raw_final.get("yaw"), "final_goal yaw"),
        )
        return cls(frame_id.strip(), points, anchor_s, anchor_xy, final_goal)

    def interpolate(self, arc):
        arc = max(0.0, min(self.total_length, float(arc)))
        right = bisect.bisect_right(self._arc, arc)
        if right <= 0:
            return self.points[0]
        if right >= len(self.points):
            return self.points[-1]
        first = self.points[right - 1]
        second = self.points[right]
        fraction = (arc - first.s) / (second.s - first.s)
        yaw_delta = math.atan2(
            math.sin(second.yaw - first.yaw),
            math.cos(second.yaw - first.yaw),
        )
        yaw = math.atan2(
            math.sin(first.yaw + fraction * yaw_delta),
            math.cos(first.yaw + fraction * yaw_delta),
        )
        return PathPoint(
            s=arc,
            x=first.x + fraction * (second.x - first.x),
            y=first.y + fraction * (second.y - first.y),
            yaw=yaw,
            curvature=first.curvature
            + fraction * (second.curvature - first.curvature),
            source_seq_start=first.source_seq_start,
            source_seq_end=first.source_seq_end,
        )

    """
    ===============================
    机器人向参考路径做投影的投影点计算
    ===============================
    """
    def project(self, x, y, minimum_s=0.0, forward_window=None):
        """Project onto a forward-only section of the sampled polyline."""
        minimum_s = max(0.0, min(self.total_length, float(minimum_s)))
        maximum_s = self.total_length
        if forward_window is not None and forward_window > 0.0:
            maximum_s = min(self.total_length, minimum_s + float(forward_window))

        best = None
        for index, (first, second) in enumerate(zip(self.points, self.points[1:])):
            if second.s < minimum_s or first.s > maximum_s:
                continue

            dx = second.x - first.x
            dy = second.y - first.y
            length_squared = dx * dx + dy * dy

            if length_squared <= 1.0e-12:
                continue

            fraction = ((x - first.x) * dx + (y - first.y) * dy) / length_squared
            lower = max(0.0, (minimum_s - first.s) / (second.s - first.s))
            upper = min(1.0, (maximum_s - first.s) / (second.s - first.s))
            fraction = max(lower, min(upper, fraction))

            projected_x = first.x + fraction * dx
            projected_y = first.y + fraction * dy

            distance = math.hypot(x - projected_x, y - projected_y) # 机器人距离投影点的距离

            arc = first.s + fraction * (second.s - first.s)         # 弧长s计算
            candidate = (distance, -arc, index, arc, projected_x, projected_y)
            if best is None or candidate < best:
                best = candidate
            
        if best is None:
            point = self.interpolate(minimum_s)
            return PathProjection(
                minimum_s,
                point.x,
                point.y,
                math.hypot(x - point.x, y - point.y),
                max(0, len(self.points) - 2),
            )
        return PathProjection(
            s=best[3],
            x=best[4], # 投影点x
            y=best[5], # 投影点y
            cross_track_error=best[0],
            segment_index=best[2],
        )

    def maximum_abs_curvature(self, start_s, end_s):
        start_s = max(0.0, min(self.total_length, float(start_s)))
        end_s = max(start_s, min(self.total_length, float(end_s)))
        maximum = max(
            abs(self.interpolate(start_s).curvature),
            abs(self.interpolate(end_s).curvature),
        )
        left = bisect.bisect_left(self._arc, start_s)
        right = bisect.bisect_right(self._arc, end_s)
        for point in self.points[left:right]:
            maximum = max(maximum, abs(point.curvature))
        return maximum


class PathTracker:
    """Monotonic progress and curvature-adaptive moving target selection."""

    def __init__(
        self,
        path,
        lookahead_min,
        lookahead_max,
        curvature_gain,
        projection_window,
        direct_segments=(),
        heading_locks=(),
    ):
        self.path = path
        self.lookahead_min = float(lookahead_min)
        self.lookahead_max = float(lookahead_max)
        self.curvature_gain = float(curvature_gain)
        self.projection_window = float(projection_window)
        if not 0.0 < self.lookahead_min <= self.lookahead_max:
            raise PathConfigError("lookahead bounds must satisfy 0 < min <= max")
        if self.curvature_gain < 0.0:
            raise PathConfigError("lookahead curvature gain must not be negative")
        if self.projection_window <= 0.0:
            raise PathConfigError("projection window must be positive")
        self.direct_segments = []
        for start, end in direct_segments:
            if start not in path.anchor_s or end not in path.anchor_s:
                raise PathConfigError(
                    "direct segment {} -> {} has no fitted-path anchor".format(start, end)
                )
            start_s = path.anchor_s[start]
            end_s = path.anchor_s[end]
            if end_s <= start_s:
                raise PathConfigError("direct segment end must follow its start")
            self.direct_segments.append((start_s, end_s, start, end))
        self.heading_locks = []
        for lock in heading_locks:
            if not isinstance(lock, (tuple, list)) or len(lock) != 6:
                raise PathConfigError(
                    "heading lock must contain start, full-lock, end, "
                    "direction-start, direction-end seq and release distance"
                )
            (
                start_seq,
                full_lock_seq,
                end_seq,
                direction_start_seq,
                direction_end_seq,
                release_distance,
            ) = lock
            required = (
                start_seq,
                full_lock_seq,
                end_seq,
                direction_start_seq,
                direction_end_seq,
            )
            if any(sequence not in path.anchor_s for sequence in required):
                raise PathConfigError(
                    "heading lock references a seq with no fitted-path anchor"
                )
            start_s = path.anchor_s[start_seq]
            full_lock_s = path.anchor_s[full_lock_seq]
            end_s = path.anchor_s[end_seq]
            try:
                release_distance = float(release_distance)
            except (TypeError, ValueError):
                raise PathConfigError("heading lock release distance must be numeric")
            if not math.isfinite(release_distance) or release_distance <= 0.0:
                raise PathConfigError("heading lock release distance must be positive")
            release_start_s = end_s - release_distance
            if not start_s < full_lock_s <= release_start_s < end_s:
                raise PathConfigError(
                    "heading lock must complete after its start and before "
                    "its release interval"
                )
            direction_start = path.anchor_xy[direction_start_seq]
            direction_end = path.anchor_xy[direction_end_seq]
            direction_x = direction_end[0] - direction_start[0]
            direction_y = direction_end[1] - direction_start[1]
            if math.hypot(direction_x, direction_y) <= 1.0e-6:
                raise PathConfigError("heading lock direction anchors coincide")
            self.heading_locks.append(
                HeadingLock(
                    start_s=start_s,
                    full_lock_s=full_lock_s,
                    release_start_s=release_start_s,
                    end_s=end_s,
                    yaw=math.atan2(direction_y, direction_x),
                )
            )
        self.progress_s = 0.0

    def update(self, x, y):
        """按照算法逻辑更新机器人坐标，包括投影、计算前视距离，得到虚拟追踪点并返回这些值"""

        """将机器人位置投影到路径折线上"""
        projection = self.path.project(
            x,
            y,
            minimum_s=self.progress_s,
            forward_window=self.projection_window, # 在一个窗口内寻找距机器人最近的点
        )
        self.progress_s = max(self.progress_s, projection.s)
        preview_curvature = self.path.maximum_abs_curvature(
            self.progress_s,
            self.progress_s + self.lookahead_max,
        )

        """
        保证曲率越大，前视距离越小，自适应加强弯道通过能力
        """
        lookahead = self.lookahead_max / (
            1.0 + self.curvature_gain * preview_curvature
        ) 
        lookahead = max(self.lookahead_min, min(self.lookahead_max, lookahead)) # 范围限制

        """计算虚拟追踪点"""
        target_s = min(self.path.total_length, self.progress_s + lookahead)
        for start_s, end_s, _start_seq, _end_seq in self.direct_segments:
            if start_s <= self.progress_s < end_s:
                target_s = max(target_s, end_s)
                break
        target = self.path.interpolate(target_s) # 插值得到虚拟追踪点的x, y, yaw
        for lock in self.heading_locks:
            lock_weight = lock.weight(self.progress_s)
            if lock_weight <= 0.0:
                continue
            target = PathPoint(
                s=target.s,
                x=target.x,
                y=target.y,
                yaw=_angle_lerp(target.yaw, lock.yaw, lock_weight),
                curvature=target.curvature,
                source_seq_start=target.source_seq_start,
                source_seq_end=target.source_seq_end,
            )
            break
        return TrackingTarget(
            progress_s=self.progress_s,
            cross_track_error=projection.cross_track_error,
            lookahead=lookahead,
            target=target,
        )
