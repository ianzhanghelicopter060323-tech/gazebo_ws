#!/usr/bin/env python3
"""Fit the active pickup route and draw it over the occupancy map.

The implementation intentionally has no SciPy dependency.  It first applies a
small, chord-length-aware second-difference regularization to the active route
anchors, then interpolates the adjusted anchors with a parametric natural cubic
spline.  Mission direct segments are exported as true line segments, and
route-specific y floors prevent a spline from dipping below a measured safe
corridor.  The resulting curve is sampled more densely where curvature is high.
"""

import argparse
import ast
import math
import re
import shutil
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont
import yaml


SEQ_PATTERN = re.compile(r"\s*# seq (\d+)(?=\s|:|$)")
SEQ_LABEL_PATTERN = re.compile(r"\s*# seq\s+([^\s:]+)")
X_PATTERN = re.compile(r"\s*- x:\s*([-+0-9.eE]+)")
Y_PATTERN = re.compile(r"\s*y:\s*([-+0-9.eE]+)")
YAW_PATTERN = re.compile(r"\s*yaw:\s*([-+0-9.eE]+)")
DELIVERY_CLASS_COLORS = {
    0: (220, 65, 65),
    1: (45, 165, 95),
    2: (40, 115, 225),
}
DELIVERY_ENTRY_COLOR = (145, 45, 190)


def read_cube_spawn_areas(path):
    """Read CUBE_AREAS without importing the ROS-dependent spawn script."""
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    raw_areas = None
    for node in tree.body:
        if not isinstance(node, (ast.Assign, ast.AnnAssign)):
            continue
        targets = node.targets if isinstance(node, ast.Assign) else [node.target]
        if any(
            isinstance(target, ast.Name) and target.id == "CUBE_AREAS"
            for target in targets
        ):
            raw_areas = ast.literal_eval(node.value)
            break
    if raw_areas is None:
        raise ValueError("spawn script does not define CUBE_AREAS")
    if len(raw_areas) != 3:
        raise ValueError("CUBE_AREAS must contain exactly three regions")

    areas = []
    for index, values in enumerate(raw_areas):
        if not isinstance(values, (tuple, list)) or len(values) != 5:
            raise ValueError(
                "CUBE_AREAS entry {} must be (x_min, x_max, y_min, y_max, yaw)".format(
                    index
                )
            )
        x_min, x_max, y_min, y_max, yaw = [float(value) for value in values]
        if not all(math.isfinite(value) for value in (x_min, x_max, y_min, y_max, yaw)):
            raise ValueError("CUBE_AREAS contains a non-finite value")
        if x_min >= x_max or y_min >= y_max:
            raise ValueError("CUBE_AREAS contains an invalid rectangle")
        areas.append([None, x_min, x_max, y_min, y_max, yaw])

    # The observation names describe their spatial relation to the common
    # circumcenter: the leftmost region is far, the rightmost is close, and the
    # remaining upper region is mid. This avoids relying on list order.
    by_center_x = sorted(
        range(len(areas)), key=lambda i: (areas[i][1] + areas[i][2]) / 2.0
    )
    areas[by_center_x[0]][0] = "far_navi"
    areas[by_center_x[1]][0] = "mid"
    areas[by_center_x[2]][0] = "close_navi"
    return [tuple(area) for area in areas]


def read_delivery_navigation_goals(path):
    """Read the shared cone-entry pose and three task-selected workshops."""
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    delivery = data.get("delivery") if isinstance(data, dict) else None
    if not isinstance(delivery, dict) or delivery.get("configured") is not True:
        raise ValueError("delivery navigation goals are not configured")
    if delivery.get("frame_id") != "map":
        raise ValueError("delivery navigation goals must use the map frame")

    def pose(raw, label, color):
        if not isinstance(raw, dict):
            raise ValueError("{} must be a mapping".format(label))
        missing = [key for key in ("x", "y", "yaw") if key not in raw]
        if missing:
            raise ValueError(
                "{} is missing {}".format(label, ", ".join(missing))
            )
        values = [float(raw[key]) for key in ("x", "y", "yaw")]
        if not all(math.isfinite(value) for value in values):
            raise ValueError("{} contains a non-finite value".format(label))
        return (label, values[0], values[1], values[2], color)

    goals = [
        pose(
            delivery.get("entry_pose"),
            "cone preparation pose",
            DELIVERY_ENTRY_COLOR,
        )
    ]
    destinations = delivery.get("destinations")
    if not isinstance(destinations, list) or len(destinations) != 3:
        raise ValueError("delivery.destinations must contain exactly three goals")
    by_class = {}
    for destination in destinations:
        if not isinstance(destination, dict):
            raise ValueError("delivery destination must be a mapping")
        target_class = destination.get("target_class")
        if target_class not in DELIVERY_CLASS_COLORS or target_class in by_class:
            raise ValueError(
                "delivery destinations must uniquely cover target classes 0, 1, 2"
            )
        name = destination.get("name")
        if not isinstance(name, str) or not name.strip():
            raise ValueError("delivery destination name must not be empty")
        by_class[target_class] = pose(
            destination,
            name.strip(),
            DELIVERY_CLASS_COLORS[target_class],
        )
    if set(by_class) != set(DELIVERY_CLASS_COLORS):
        raise ValueError(
            "delivery destinations must uniquely cover target classes 0, 1, 2"
        )
    goals.extend(by_class[target_class] for target_class in sorted(by_class))
    return goals


def read_active_route(path):
    """Return active YAML points while retaining their original seq labels."""
    lines = path.read_text(encoding="utf-8").splitlines()
    sequence = None
    records = []
    for index, line in enumerate(lines):
        sequence_label_match = SEQ_LABEL_PATTERN.match(line)
        sequence_match = SEQ_PATTERN.match(line)
        if sequence_label_match and not sequence_match:
            raise ValueError(
                "seq labels must be positive integers; encode a logical "
                "fractional waypoint such as 34.5 with an integer ID such as 345"
            )
        if sequence_match:
            sequence = int(sequence_match.group(1))

        x_match = X_PATTERN.match(line)
        if not x_match:
            continue
        if sequence is None or index + 2 >= len(lines):
            raise ValueError("active route point is missing its seq, y, or yaw value")
        y_match = Y_PATTERN.match(lines[index + 1])
        yaw_match = YAW_PATTERN.match(lines[index + 2])
        if not y_match or not yaw_match:
            raise ValueError(
                "active x value is not followed by active y and yaw values"
            )
        records.append(
            (
                sequence,
                float(x_match.group(1)),
                float(y_match.group(1)),
                float(yaw_match.group(1)),
            )
        )

    if len(records) < 4:
        raise ValueError("at least four active route points are required")
    return records


def read_documented_route(path, active_sequences):
    """Read pose records from the first txt code block in docs/navi_pos.md.

    The document contains both PoseStamped and MoveBaseActionGoal layouts, so
    this parser follows each document block's first seq and first pose rather
    than assuming one fixed indentation level.  The fourth captured pose has a
    duplicated ``seq: 1`` header and is retained as seq 4 for compatibility
    with the development route.
    """
    text = path.read_text(encoding="utf-8")
    if "```txt" not in text:
        raise ValueError("route document has no ```txt pose block")
    pose_block = text.split("```txt", 1)[1].split("```", 1)[0]
    blocks = re.split(r"^\s*---\s*$", pose_block, flags=re.MULTILINE)
    records = []
    seen_sequences = set()
    for block in blocks:
        sequence_match = re.search(
            r"^\s*seq:\s*(\d+)\s*$", block, flags=re.MULTILINE
        )
        position_match = re.search(
            r"^\s*position:\s*\n"
            r"\s*x:\s*([-+0-9.eE]+)\s*\n"
            r"\s*y:\s*([-+0-9.eE]+)",
            block,
            flags=re.MULTILINE,
        )
        orientation_match = re.search(
            r"^\s*orientation:\s*\n"
            r"(?:\s*[xy]:\s*[-+0-9.eE]+\s*\n){2}"
            r"\s*z:\s*([-+0-9.eE]+)\s*\n"
            r"\s*w:\s*([-+0-9.eE]+)",
            block,
            flags=re.MULTILINE,
        )
        if not sequence_match or not position_match or not orientation_match:
            continue
        sequence = int(sequence_match.group(1))
        if sequence == 1 and sequence in seen_sequences:
            sequence = 4
        if sequence in seen_sequences:
            raise ValueError("route document contains duplicate seq {}".format(sequence))
        seen_sequences.add(sequence)
        quaternion_z = float(orientation_match.group(1))
        quaternion_w = float(orientation_match.group(2))
        records.append(
            (
                sequence,
                float(position_match.group(1)),
                float(position_match.group(2)),
                2.0 * math.atan2(quaternion_z, quaternion_w),
            )
        )

    by_sequence = {record[0]: record for record in records}
    missing = [sequence for sequence in active_sequences if sequence not in by_sequence]
    if missing:
        raise ValueError(
            "route document is missing active seq {}".format(missing)
        )
    return [by_sequence[sequence] for sequence in active_sequences]


def read_fit_constraints(path, active_sequences):
    """Read reference-path geometry constraints from navigation configuration."""
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    tracking = data.get("navigation", {}).get("path_tracking", {})
    raw_linear = tracking.get("direct_segments", [])
    raw_y_floors = tracking.get("fit_y_floor_segments", [])
    sequence_indices = {
        sequence: index for index, sequence in enumerate(active_sequences)
    }

    def validated_segments(raw_segments, label):
        if not isinstance(raw_segments, list):
            raise ValueError("{} must be a list".format(label))
        result = []
        for raw in raw_segments:
            if not isinstance(raw, (list, tuple)) or len(raw) != 2:
                raise ValueError(
                    "{} entries must contain start and end seq".format(label)
                )
            start, end = raw
            if start not in sequence_indices or end not in sequence_indices:
                raise ValueError(
                    "{} segment {} -> {} has no active route anchor".format(
                        label, start, end
                    )
                )
            if sequence_indices[end] != sequence_indices[start] + 1:
                raise ValueError(
                    "{} segment {} -> {} must join adjacent active anchors".format(
                        label, start, end
                    )
                )
            result.append((int(start), int(end)))
        return result

    return (
        validated_segments(raw_linear, "direct_segments"),
        validated_segments(raw_y_floors, "fit_y_floor_segments"),
    )


def chord_parameter(points):
    lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    if np.any(lengths <= 0.0):
        raise ValueError("active route contains duplicate consecutive points")
    return np.concatenate(([0.0], np.cumsum(lengths)))


def regularized_anchors(points, parameter, smoothing_lambda, fixed_indices=()):
    """Smooth anchors with a nonuniform second-derivative penalty.

    The first and last anchors remain exact.  Every interior active point stays
    in the least-squares data term, so no route observation is discarded.
    """
    count = len(points)
    penalty = np.zeros((count - 2, count), dtype=float)
    for row, index in enumerate(range(1, count - 1)):
        h_previous = parameter[index] - parameter[index - 1]
        h_next = parameter[index + 1] - parameter[index]
        interval_weight = math.sqrt((h_previous + h_next) / 2.0)
        common = interval_weight * 2.0 / (h_previous + h_next)
        penalty[row, index - 1] = common / h_previous
        penalty[row, index] = common * (-1.0 / h_previous - 1.0 / h_next)
        penalty[row, index + 1] = common / h_next

    normal = np.eye(count) + smoothing_lambda * (penalty.T @ penalty)
    fixed = sorted(set([0, count - 1] + list(fixed_indices)))
    if any(index < 0 or index >= count for index in fixed):
        raise ValueError("fixed anchor index is outside the active route")
    free = [index for index in range(count) if index not in fixed]
    anchors = points.copy()
    if free:
        anchors[free] = np.linalg.solve(
            normal[np.ix_(free, free)],
            points[free] - normal[np.ix_(free, fixed)] @ points[fixed],
        )
    anchors[fixed] = points[fixed]
    return anchors


class NaturalCubicPath:
    def __init__(self, parameter, points):
        self.parameter = parameter
        self.points = points
        self.second = np.column_stack(
            [self._second_derivatives(points[:, axis]) for axis in range(2)]
        )

    def _second_derivatives(self, values):
        count = len(values)
        result = np.zeros(count, dtype=float)
        intervals = np.diff(self.parameter)
        matrix = np.zeros((count - 2, count - 2), dtype=float)
        right = np.zeros(count - 2, dtype=float)

        for row, index in enumerate(range(1, count - 1)):
            previous = intervals[index - 1]
            following = intervals[index]
            if row > 0:
                matrix[row, row - 1] = previous
            matrix[row, row] = 2.0 * (previous + following)
            if row < count - 3:
                matrix[row, row + 1] = following
            right[row] = 6.0 * (
                (values[index + 1] - values[index]) / following
                - (values[index] - values[index - 1]) / previous
            )

        result[1:-1] = np.linalg.solve(matrix, right)
        return result

    def evaluate(self, query):
        query = np.asarray(query, dtype=float)
        indices = np.searchsorted(self.parameter, query, side="right") - 1
        indices = np.clip(indices, 0, len(self.points) - 2)
        starts = self.parameter[indices]
        ends = self.parameter[indices + 1]
        intervals = ends - starts
        left = (ends - query) / intervals
        right = (query - starts) / intervals

        position = np.zeros((len(query), 2), dtype=float)
        first = np.zeros_like(position)
        second = np.zeros_like(position)
        for axis in range(2):
            values = self.points[:, axis]
            moments = self.second[:, axis]
            m0 = moments[indices]
            m1 = moments[indices + 1]
            v0 = values[indices]
            v1 = values[indices + 1]
            h = intervals

            position[:, axis] = (
                m0 * (left * h) ** 3 / (6.0 * h)
                + m1 * (right * h) ** 3 / (6.0 * h)
                + (v0 - m0 * h * h / 6.0) * left
                + (v1 - m1 * h * h / 6.0) * right
            )
            first[:, axis] = (
                -m0 * (left * h) ** 2 / (2.0 * h)
                + m1 * (right * h) ** 2 / (2.0 * h)
                + (v1 - v0) / h
                - (m1 - m0) * h / 6.0
            )
            second[:, axis] = m0 * left + m1 * right
        return position, first, second


class ConstrainedPath:
    """Apply exact linear segments and lower-y corridor limits to a spline."""

    def __init__(
        self,
        base_curve,
        parameter,
        anchors,
        sequences,
        linear_segments=(),
        y_floor_segments=(),
    ):
        self.base_curve = base_curve
        self.parameter = parameter
        self.points = anchors
        sequence_indices = {
            sequence: index for index, sequence in enumerate(sequences)
        }
        self.linear_indices = {
            sequence_indices[start] for start, _end in linear_segments
        }
        self.y_floor_indices = {
            sequence_indices[start] for start, _end in y_floor_segments
        }

    def evaluate(self, query):
        query = np.asarray(query, dtype=float)
        position, first, second = self.base_curve.evaluate(query)
        indices = np.searchsorted(self.parameter, query, side="right") - 1
        indices = np.clip(indices, 0, len(self.points) - 2)

        for index in self.y_floor_indices:
            mask = indices == index
            if not np.any(mask):
                continue
            floor_y = min(self.points[index, 1], self.points[index + 1, 1])
            limited = mask & (position[:, 1] < floor_y)
            position[limited, 1] = floor_y
            first[limited, 1] = 0.0
            second[limited, 1] = 0.0

        for index in self.linear_indices:
            mask = indices == index
            if not np.any(mask):
                continue
            start = self.parameter[index]
            end = self.parameter[index + 1]
            interval = end - start
            fraction = ((query[mask] - start) / interval)[:, None]
            delta = self.points[index + 1] - self.points[index]
            position[mask] = self.points[index] + fraction * delta
            first[mask] = delta / interval
            second[mask] = 0.0

        return position, first, second


def curvature(first, second):
    speed = np.linalg.norm(first, axis=1)
    numerator = first[:, 0] * second[:, 1] - first[:, 1] * second[:, 0]
    return numerator / np.maximum(speed, 1.0e-9) ** 3, speed


def adaptive_samples(
    curve,
    parameter_end,
    epsilon,
    minimum,
    maximum,
    required_parameters=(),
):
    dense_parameter = np.linspace(
        0.0, parameter_end, max(3000, int(parameter_end / 0.002) + 1)
    )
    dense_points, first, second = curve.evaluate(dense_parameter)
    dense_curvature, speed = curvature(first, second)
    arc = np.concatenate(
        ([0.0], np.cumsum(np.linalg.norm(np.diff(dense_points, axis=0), axis=1)))
    )

    sample_arc = [0.0]
    while sample_arc[-1] < arc[-1]:
        local_curvature = abs(np.interp(sample_arc[-1], arc, dense_curvature))
        if local_curvature <= 1.0e-9:
            step = maximum
        else:
            step = math.sqrt(8.0 * epsilon / local_curvature)
        step = min(maximum, max(minimum, step))
        next_arc = min(arc[-1], sample_arc[-1] + step)
        if next_arc <= sample_arc[-1] + 1.0e-9:
            break
        sample_arc.append(next_arc)

    required_arc = np.interp(required_parameters, dense_parameter, arc)
    sample_arc = np.unique(
        np.concatenate((np.asarray(sample_arc), np.asarray(required_arc)))
    )
    sample_parameter = np.interp(sample_arc, arc, dense_parameter)
    samples, _sample_first, _sample_second = curve.evaluate(sample_parameter)
    return (
        dense_parameter,
        dense_points,
        dense_curvature,
        speed,
        arc,
        sample_parameter,
        sample_arc,
        samples,
    )


def export_path_config(
    output,
    route,
    records,
    parameter,
    curve,
    dense_parameter,
    dense_arc,
    sample_parameter,
    sample_arc,
    samples,
    smoothing_lambda,
    chord_error,
    minimum_spacing,
    maximum_spacing,
    occupied_samples,
    unknown_samples,
    minimum_map_clearance,
    linear_segments,
    y_floor_segments,
):
    """Write the fitted runtime reference path and original-seq arc markers."""
    _positions, first, second = curve.evaluate(sample_parameter)
    sample_curvature, _speed = curvature(first, second)
    sample_yaw = np.arctan2(first[:, 1], first[:, 0])

    anchors = []
    for record, anchor_parameter in zip(records, parameter):
        anchor_position, _first, _second = curve.evaluate([anchor_parameter])
        anchors.append(
            {
                "seq": int(record[0]),
                "s": float(np.interp(anchor_parameter, dense_parameter, dense_arc)),
                "x": float(anchor_position[0, 0]),
                "y": float(anchor_position[0, 1]),
            }
        )

    points = []
    sequences = [int(record[0]) for record in records]
    for index, (point, point_parameter) in enumerate(
        zip(samples, sample_parameter)
    ):
        segment = int(np.searchsorted(parameter, point_parameter, side="right") - 1)
        segment = max(0, min(segment, len(parameter) - 2))
        points.append(
            {
                "s": float(sample_arc[index]),
                "x": float(point[0]),
                "y": float(point[1]),
                "yaw": float(sample_yaw[index]),
                "curvature": float(sample_curvature[index]),
                "source_seq_start": sequences[segment],
                "source_seq_end": sequences[segment + 1],
            }
        )

    data = {
        "fitted_path": {
            "configured": True,
            "frame_id": "map",
            "source_route": str(route),
            "generation": {
                "method": "regularized_natural_cubic_with_constraints",
                "smoothing_lambda": float(smoothing_lambda),
                "chord_error": float(chord_error),
                "min_sample_spacing": float(minimum_spacing),
                "max_sample_spacing": float(maximum_spacing),
                "constraints": {
                    "linear_segments": [
                        [int(start), int(end)] for start, end in linear_segments
                    ],
                    "y_floor_segments": [
                        [int(start), int(end)] for start, end in y_floor_segments
                    ],
                },
                "map_diagnostics": {
                    "occupied_samples": int(occupied_samples),
                    "unknown_samples": int(unknown_samples),
                    "minimum_centerline_clearance": float(
                        minimum_map_clearance
                    ),
                    "centerline_safe": not (
                        occupied_samples or unknown_samples
                    ),
                },
            },
            "active_sequences": sequences,
            "anchors": anchors,
            "points": points,
            "final_goal": {
                "x": float(records[-1][1]),
                "y": float(records[-1][2]),
                "yaw": float(records[-1][3]),
            },
        }
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    warning = ""
    if occupied_samples or unknown_samples:
        warning = (
            "# WARNING: candidate centerline intersects occupied or unknown "
            "map cells.\n"
        )
    output.write_text(
        "# Generated by plot_fitted_navigation_path.py; do not edit by hand.\n"
        + warning
        + yaml.safe_dump(data, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )


def map_metadata(path):
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    image_path = Path(data["image"])
    if not image_path.is_absolute():
        image_path = path.parent / image_path
    return {
        "image": image_path,
        "resolution": float(data["resolution"]),
        "origin": tuple(float(value) for value in data["origin"][:2]),
        "negate": int(data.get("negate", 0)),
        "occupied_thresh": float(data.get("occupied_thresh", 0.65)),
        "free_thresh": float(data.get("free_thresh", 0.196)),
    }


def occupancy_diagnostics(image, metadata, path_points):
    pixels = np.asarray(image)
    height, width = pixels.shape
    resolution = metadata["resolution"]
    origin_x, origin_y = metadata["origin"]
    columns = np.floor((path_points[:, 0] - origin_x) / resolution).astype(int)
    rows = height - 1 - np.floor(
        (path_points[:, 1] - origin_y) / resolution
    ).astype(int)
    inside = (
        (columns >= 0) & (columns < width) & (rows >= 0) & (rows < height)
    )
    if not np.all(inside):
        raise ValueError("fitted path leaves the occupancy-map bounds")

    values = pixels[rows, columns]
    if metadata["negate"]:
        occupancy = values.astype(float) / 255.0
    else:
        occupancy = (255.0 - values.astype(float)) / 255.0
    occupied_samples = int(np.count_nonzero(occupancy > metadata["occupied_thresh"]))
    unknown_samples = int(
        np.count_nonzero(
            (occupancy >= metadata["free_thresh"])
            & (occupancy <= metadata["occupied_thresh"])
        )
    )

    if metadata["negate"]:
        occupied_pixels = np.argwhere(
            pixels.astype(float) / 255.0 > metadata["occupied_thresh"]
        )
    else:
        occupied_pixels = np.argwhere(
            (255.0 - pixels.astype(float)) / 255.0
            > metadata["occupied_thresh"]
        )

    minimum_clearance = float("inf")
    minimum_index = 0
    for start in range(0, len(path_points), 200):
        distance_squared = (
            (rows[start : start + 200, None] - occupied_pixels[None, :, 0]) ** 2
            + (columns[start : start + 200, None] - occupied_pixels[None, :, 1]) ** 2
        )
        local = np.sqrt(np.min(distance_squared, axis=1)) * resolution
        local_index = int(np.argmin(local))
        if float(local[local_index]) < minimum_clearance:
            minimum_clearance = float(local[local_index])
            minimum_index = start + local_index
    return occupied_samples, unknown_samples, minimum_clearance, minimum_index


def load_font(size, bold=False):
    filename = "DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf"
    path = Path("/usr/share/fonts/truetype/dejavu") / filename
    try:
        return ImageFont.truetype(str(path), size=size)
    except OSError:
        return ImageFont.load_default()


def render_view(
    map_image,
    metadata,
    crop,
    scale,
    original_points,
    sequences,
    fitted_points,
    samples,
    clearance_point,
    label_points,
    spawn_areas,
    delivery_goals,
):
    left, top, right, bottom = crop
    nearest = getattr(Image, "Resampling", Image).NEAREST
    view = map_image.crop(crop).convert("RGB").resize(
        ((right - left) * scale, (bottom - top) * scale),
        nearest,
    )
    resolution = metadata["resolution"]
    origin_x, origin_y = metadata["origin"]
    map_height = map_image.height

    def transform(point):
        column = (point[0] - origin_x) / resolution
        row = map_height - 1 - (point[1] - origin_y) / resolution
        return ((column - left) * scale, (row - top) * scale)

    fitted_pixels = [transform(point) for point in fitted_points]
    original_pixels = [transform(point) for point in original_points]
    sample_pixels = [transform(point) for point in samples]

    area_colors = {
        "far_navi": (70, 150, 255),
        "mid": (255, 190, 30),
        "close_navi": (55, 190, 115),
    }
    area_overlay = Image.new("RGBA", view.size, (0, 0, 0, 0))
    area_draw = ImageDraw.Draw(area_overlay)
    area_label_positions = []
    for label, x_min, x_max, y_min, y_max, _yaw in spawn_areas:
        x1, y1 = transform((x_min, y_max))
        x2, y2 = transform((x_max, y_min))
        color = area_colors[label]
        area_draw.rectangle(
            (x1, y1, x2, y2),
            fill=color + (70,),
            outline=color + (235,),
            width=max(2, scale // 2),
        )
        area_label_positions.append((label, (x1 + x2) / 2.0, min(y1, y2)))
    view = Image.alpha_composite(view.convert("RGBA"), area_overlay).convert("RGB")

    envelope = Image.new("RGBA", view.size, (0, 0, 0, 0))
    envelope_draw = ImageDraw.Draw(envelope)
    envelope_width = max(2, round(2.0 * 0.11 / resolution * scale))
    envelope_draw.line(
        fitted_pixels,
        fill=(255, 70, 70, 55),
        width=envelope_width,
        joint="curve",
    )
    view = Image.alpha_composite(view.convert("RGBA"), envelope).convert("RGB")
    draw = ImageDraw.Draw(view)
    if label_points:
        for label, center_x, top_y in area_label_positions:
            text = "{} spawn".format(label)
            font = load_font(max(12, scale + 5), bold=True)
            text_width, text_height = draw.textsize(text, font=font)
            draw.text(
                (center_x - text_width / 2.0, top_y - text_height - 5),
                text,
                fill=area_colors[label],
                font=font,
                stroke_width=2,
                stroke_fill="white",
            )
    draw.line(original_pixels, fill=(235, 139, 28), width=max(2, 2 * scale))
    draw.line(fitted_pixels, fill=(210, 35, 45), width=max(3, 2 * scale), joint="curve")

    sample_radius = max(2, scale // 2)
    for x, y in sample_pixels:
        draw.ellipse(
            (x - sample_radius, y - sample_radius, x + sample_radius, y + sample_radius),
            fill=(0, 185, 210),
            outline=(0, 70, 85),
        )

    point_radius = max(3, scale - 1)
    for sequence, (x, y) in zip(sequences, original_pixels):
        draw.ellipse(
            (x - point_radius, y - point_radius, x + point_radius, y + point_radius),
            fill=(35, 90, 210),
            outline="white",
            width=max(1, scale // 3),
        )
        if label_points:
            draw.text(
                (x + point_radius + 2, y - point_radius - 5),
                str(sequence),
                fill=(10, 25, 80),
                font=load_font(max(11, scale + 5), bold=True),
                stroke_width=2,
                stroke_fill="white",
            )

    for point, color in (
        (original_pixels[0], (20, 175, 55)),
        (original_pixels[-1], (145, 45, 190)),
    ):
        x, y = point
        radius = point_radius + max(3, scale // 2)
        draw.ellipse((x - radius, y - radius, x + radius, y + radius), outline=color, width=max(2, scale // 2))

    heading_length = 0.45
    for label, goal_x, goal_y, goal_yaw, color in delivery_goals:
        x, y = transform((goal_x, goal_y))
        arrow_x, arrow_y = transform(
            (
                goal_x + heading_length * math.cos(goal_yaw),
                goal_y + heading_length * math.sin(goal_yaw),
            )
        )
        draw.line(
            (x, y, arrow_x, arrow_y),
            fill=color,
            width=max(3, scale // 2),
        )
        delta_x = arrow_x - x
        delta_y = arrow_y - y
        arrow_pixels = math.hypot(delta_x, delta_y)
        if arrow_pixels > 0.0:
            unit_x = delta_x / arrow_pixels
            unit_y = delta_y / arrow_pixels
            normal_x = -unit_y
            normal_y = unit_x
            head_length = max(8, 2 * scale)
            head_width = max(5, scale)
            base_x = arrow_x - head_length * unit_x
            base_y = arrow_y - head_length * unit_y
            draw.polygon(
                (
                    (arrow_x, arrow_y),
                    (
                        base_x + head_width * normal_x,
                        base_y + head_width * normal_y,
                    ),
                    (
                        base_x - head_width * normal_x,
                        base_y - head_width * normal_y,
                    ),
                ),
                fill=color,
            )
        goal_radius = max(4, scale)
        draw.ellipse(
            (
                x - goal_radius,
                y - goal_radius,
                x + goal_radius,
                y + goal_radius,
            ),
            fill=color,
            outline="white",
            width=max(2, scale // 3),
        )
        if label_points:
            font = load_font(max(12, scale + 5), bold=True)
            text_width, text_height = draw.textsize(label, font=font)
            label_x = x + goal_radius + 4
            if label_x + text_width + 3 > view.width:
                label_x = x - goal_radius - text_width - 4
            label_y = max(2, min(y - text_height - 3, view.height - text_height - 2))
            draw.text(
                (label_x, label_y),
                label,
                fill=color,
                font=font,
                stroke_width=2,
                stroke_fill="white",
            )

    cx, cy = transform(clearance_point)
    radius = point_radius + scale
    draw.ellipse(
        (cx - radius, cy - radius, cx + radius, cy + radius),
        outline=(220, 0, 180),
        width=max(2, scale // 2),
    )
    return view


def create_figure(
    map_image,
    metadata,
    records,
    fitted_points,
    samples,
    clearance_point,
    diagnostics,
    spawn_areas,
    delivery_goals,
):
    points = np.asarray([[record[1], record[2]] for record in records])
    sequences = [record[0] for record in records]
    full = render_view(
        map_image,
        metadata,
        (0, 0, map_image.width, map_image.height),
        2,
        points,
        sequences,
        fitted_points,
        samples,
        clearance_point,
        False,
        spawn_areas,
        delivery_goals,
    )

    resolution = metadata["resolution"]
    origin_x, origin_y = metadata["origin"]
    margin = 0.55
    area_corners = np.asarray(
        [
            corner
            for _label, x_min, x_max, y_min, y_max, _yaw in spawn_areas
            for corner in ((x_min, y_min), (x_max, y_max))
        ]
    )
    delivery_crop_points = np.asarray(
        [
            point
            for _label, x, y, yaw, _color in delivery_goals
            for point in (
                (x, y),
                (x + 0.45 * math.cos(yaw), y + 0.45 * math.sin(yaw)),
            )
        ]
    )
    crop_points = np.vstack((fitted_points, area_corners, delivery_crop_points))
    minimum = np.min(crop_points, axis=0) - margin
    maximum = np.max(crop_points, axis=0) + margin
    left = max(0, int(math.floor((minimum[0] - origin_x) / resolution)))
    right = min(
        map_image.width,
        int(math.ceil((maximum[0] - origin_x) / resolution)) + 1,
    )
    top = max(
        0,
        map_image.height
        - 1
        - int(math.ceil((maximum[1] - origin_y) / resolution)),
    )
    bottom = min(
        map_image.height,
        map_image.height
        - int(math.floor((minimum[1] - origin_y) / resolution))
        + 1,
    )
    zoom = render_view(
        map_image,
        metadata,
        (left, top, right, bottom),
        8,
        points,
        sequences,
        fitted_points,
        samples,
        clearance_point,
        True,
        spawn_areas,
        delivery_goals,
    )

    margin_px = 24
    header = 70
    footer = 220
    width = full.width + zoom.width + 3 * margin_px
    height = header + max(full.height, zoom.height) + footer
    canvas = Image.new("RGB", (width, height), "white")
    canvas.paste(full, (margin_px, header))
    canvas.paste(zoom, (full.width + 2 * margin_px, header))
    draw = ImageDraw.Draw(canvas)
    draw.text(
        (margin_px, 16),
        "Fitted pickup-staging path and delivery navigation goals on math_newest.pgm",
        fill=(20, 20, 20),
        font=load_font(26, bold=True),
    )
    legend_y = header + max(full.height, zoom.height) + 18
    legend = [
        ("active input points / original seq", (35, 90, 210)),
        ("original active-point polyline", (235, 139, 28)),
        ("constrained fitted path", (210, 35, 45)),
        ("curvature-adaptive path samples", (0, 185, 210)),
        ("0.11 m centerline envelope", (255, 150, 150)),
        ("minimum-clearance location", (220, 0, 180)),
        ("far_navi cube spawn region", (70, 150, 255)),
        ("mid cube spawn region", (255, 190, 30)),
        ("close_navi cube spawn region", (55, 190, 115)),
        ("cone preparation pose / heading", DELIVERY_ENTRY_COLOR),
        ("food workshop goal / heading", DELIVERY_CLASS_COLORS[0]),
        ("daily workshop goal / heading", DELIVERY_CLASS_COLORS[1]),
        ("electronics workshop goal / heading", DELIVERY_CLASS_COLORS[2]),
    ]
    x = margin_px
    y = legend_y
    for index, (label, color) in enumerate(legend):
        if index and index % 3 == 0:
            x = margin_px
            y += 32
        draw.line((x, y + 9, x + 28, y + 9), fill=color, width=6)
        draw.text((x + 36, y), label, fill=(25, 25, 25), font=load_font(16))
        x += 330

    max_curvature, min_radius, clearance, sample_count, max_shift = diagnostics
    summary = (
        f"inputs={len(records)}  samples={sample_count}  "
        f"max curvature={max_curvature:.2f} 1/m  min radius={min_radius:.3f} m  "
        f"min map clearance={clearance:.2f} m  max anchor shift={max_shift:.3f} m"
    )
    draw.text(
        (margin_px, y + 42),
        summary,
        fill=(35, 35, 35),
        font=load_font(17, bold=True),
    )
    return canvas


def main():
    workspace = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--route",
        type=Path,
        default=workspace
        / "src/smart_factory_mission/config/pickup_staging_dev.yaml",
    )
    parser.add_argument(
        "--map-yaml",
        type=Path,
        default=workspace / "src/gazebo_map/maps/math_newest.yaml",
    )
    parser.add_argument(
        "--mission-config",
        type=Path,
        default=workspace
        / "src/smart_factory_navigation/config/navigation.yaml",
        help=(
            "read direct-segment and fitted y-floor constraints from the "
            "navigation configuration"
        ),
    )
    parser.add_argument(
        "--cube-spawn-script",
        type=Path,
        default=workspace / "src/car3/scripts/spawn_cubes.py",
        help="read the three CUBE_AREAS rectangles for plot annotations",
    )
    parser.add_argument(
        "--delivery-goals",
        type=Path,
        default=workspace
        / "src/smart_factory_mission/config/delivery_goals.yaml",
        help="render the cone preparation pose and three workshop goals",
    )
    parser.add_argument(
        "--route-doc",
        type=Path,
        help=(
            "read coordinates from the first txt pose block in this document; "
            "the active seq set still comes from --route"
        ),
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=workspace / "docs/navigation_path_fit_math_newest.png",
    )
    parser.add_argument(
        "--path-output",
        type=Path,
        default=workspace
        / "src/smart_factory_navigation/config/pickup_staging_fitted_path.yaml",
    )
    parser.add_argument("--copy-to", type=Path)
    parser.add_argument(
        "--render-only",
        action="store_true",
        help="render diagnostics without replacing --path-output",
    )
    parser.add_argument(
        "--allow-unsafe-render",
        action="store_true",
        help=(
            "with --render-only, draw a candidate even when its centerline "
            "enters occupied or unknown map cells"
        ),
    )
    parser.add_argument(
        "--allow-unsafe-path-output",
        action="store_true",
        help=(
            "explicitly replace --path-output even when the fitted centerline "
            "enters occupied or unknown map cells"
        ),
    )
    parser.add_argument("--smoothing-lambda", type=float, default=3.0e-4)
    parser.add_argument("--chord-error", type=float, default=0.01)
    parser.add_argument("--min-sample-spacing", type=float, default=0.08)
    parser.add_argument("--max-sample-spacing", type=float, default=0.30)
    args = parser.parse_args()
    if args.allow_unsafe_render and not args.render_only:
        parser.error("--allow-unsafe-render requires --render-only")
    if args.allow_unsafe_path_output and args.render_only:
        parser.error("--allow-unsafe-path-output cannot be used with --render-only")

    route_records = read_active_route(args.route)
    source_route = args.route
    if args.route_doc:
        records = read_documented_route(
            args.route_doc, [record[0] for record in route_records]
        )
        source_route = args.route_doc
    else:
        records = route_records
    sequences = [record[0] for record in records]
    spawn_areas = read_cube_spawn_areas(args.cube_spawn_script)
    delivery_goals = read_delivery_navigation_goals(args.delivery_goals)
    linear_segments, y_floor_segments = read_fit_constraints(
        args.mission_config, sequences
    )
    sequence_indices = {
        sequence: index for index, sequence in enumerate(sequences)
    }
    fixed_indices = set()
    for start, end in linear_segments + y_floor_segments:
        fixed_indices.add(sequence_indices[start])
        fixed_indices.add(sequence_indices[end])
    points = np.asarray([[record[1], record[2]] for record in records])
    parameter = chord_parameter(points)
    anchors = regularized_anchors(
        points,
        parameter,
        args.smoothing_lambda,
        fixed_indices=fixed_indices,
    )
    base_curve = NaturalCubicPath(parameter, anchors)
    curve = ConstrainedPath(
        base_curve,
        parameter,
        anchors,
        sequences,
        linear_segments=linear_segments,
        y_floor_segments=y_floor_segments,
    )
    (
        dense_parameter,
        fitted,
        fitted_curvature,
        speed,
        dense_arc,
        sample_parameter,
        sample_arc,
        samples,
    ) = adaptive_samples(
        curve,
        parameter[-1],
        args.chord_error,
        args.min_sample_spacing,
        args.max_sample_spacing,
        required_parameters=parameter,
    )

    metadata = map_metadata(args.map_yaml)
    map_image = Image.open(metadata["image"]).convert("L")
    occupied, unknown, clearance, clearance_index = occupancy_diagnostics(
        map_image, metadata, fitted
    )
    allow_unsafe = args.allow_unsafe_render or args.allow_unsafe_path_output
    if (occupied or unknown) and not allow_unsafe:
        raise RuntimeError(
            f"fitted path is unsafe: occupied={occupied}, unknown={unknown}"
        )

    if not args.render_only:
        export_path_config(
            args.path_output,
            source_route,
            records,
            parameter,
            curve,
            dense_parameter,
            dense_arc,
            sample_parameter,
            sample_arc,
            samples,
            args.smoothing_lambda,
            args.chord_error,
            args.min_sample_spacing,
            args.max_sample_spacing,
            occupied,
            unknown,
            clearance,
            linear_segments,
            y_floor_segments,
        )

    maximum_curvature = float(np.max(np.abs(fitted_curvature)))
    minimum_radius = 1.0 / maximum_curvature
    maximum_shift = float(np.max(np.linalg.norm(anchors - points, axis=1)))
    figure = create_figure(
        map_image,
        metadata,
        records,
        fitted,
        samples,
        fitted[clearance_index],
        (
            maximum_curvature,
            minimum_radius,
            clearance,
            len(samples),
            maximum_shift,
        ),
        spawn_areas,
        delivery_goals,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.save(args.output)
    if args.copy_to:
        args.copy_to.mkdir(parents=True, exist_ok=True)
        shutil.copy2(args.output, args.copy_to / args.output.name)

    print(f"output={args.output}")
    print(f"source_route={source_route}")
    print(
        "delivery_navigation_goals={}".format(
            [
                (label, x, y, yaw)
                for label, x, y, yaw, _color in delivery_goals
            ]
        )
    )
    if args.render_only:
        print("path_output=unchanged (render-only)")
    else:
        print(f"path_output={args.path_output}")
    if args.copy_to:
        print(f"copy={args.copy_to / args.output.name}")
    print(f"active_sequences={[record[0] for record in records]}")
    print(
        "cube_spawn_areas={}".format(
            [
                (label, x_min, x_max, y_min, y_max)
                for label, x_min, x_max, y_min, y_max, _yaw in spawn_areas
            ]
        )
    )
    print(f"linear_segments={linear_segments} y_floor_segments={y_floor_segments}")
    print(f"active_points={len(records)} adaptive_samples={len(samples)}")
    print(
        "max_anchor_shift={:.4f} max_curvature={:.3f} "
        "min_radius={:.3f} min_speed_derivative={:.3f}".format(
            maximum_shift,
            maximum_curvature,
            minimum_radius,
            float(np.min(speed)),
        )
    )
    print(
        f"occupied_samples={occupied} unknown_samples={unknown} "
        f"minimum_map_clearance={clearance:.3f}"
    )


if __name__ == "__main__":
    main()
