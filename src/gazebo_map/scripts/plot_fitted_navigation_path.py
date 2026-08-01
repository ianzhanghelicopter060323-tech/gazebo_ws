#!/usr/bin/env python3
"""Fit the active pickup route and draw it over the occupancy map.

The implementation intentionally has no SciPy dependency.  It first applies a
small, chord-length-aware second-difference regularization to the active route
anchors, then interpolates the adjusted anchors with a parametric natural cubic
spline.  The resulting curve is sampled more densely where curvature is high.
"""

import argparse
import math
import re
import shutil
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont
import yaml


SEQ_PATTERN = re.compile(r"\s*# seq (\d+)")
X_PATTERN = re.compile(r"\s*- x:\s*([-+0-9.eE]+)")
Y_PATTERN = re.compile(r"\s*y:\s*([-+0-9.eE]+)")
YAW_PATTERN = re.compile(r"\s*yaw:\s*([-+0-9.eE]+)")


def read_active_route(path):
    """Return active YAML points while retaining their original seq labels."""
    lines = path.read_text(encoding="utf-8").splitlines()
    sequence = None
    records = []
    for index, line in enumerate(lines):
        sequence_match = SEQ_PATTERN.match(line)
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


def chord_parameter(points):
    lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    if np.any(lengths <= 0.0):
        raise ValueError("active route contains duplicate consecutive points")
    return np.concatenate(([0.0], np.cumsum(lengths)))


def regularized_anchors(points, parameter, smoothing_lambda):
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
    fixed = [0, count - 1]
    free = list(range(1, count - 1))
    anchors = points.copy()
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


def curvature(first, second):
    speed = np.linalg.norm(first, axis=1)
    numerator = first[:, 0] * second[:, 1] - first[:, 1] * second[:, 0]
    return numerator / np.maximum(speed, 1.0e-9) ** 3, speed


def adaptive_samples(curve, parameter_end, epsilon, minimum, maximum):
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

    sample_arc = np.asarray(sample_arc)
    sample_parameter = np.interp(sample_arc, arc, dense_parameter)
    samples = np.column_stack(
        [
            np.interp(sample_arc, arc, dense_points[:, 0]),
            np.interp(sample_arc, arc, dense_points[:, 1]),
        ]
    )
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
                "method": "regularized_natural_cubic_c2",
                "smoothing_lambda": float(smoothing_lambda),
                "chord_error": float(chord_error),
                "min_sample_spacing": float(minimum_spacing),
                "max_sample_spacing": float(maximum_spacing),
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
    output.write_text(
        "# Generated by plot_fitted_navigation_path.py; do not edit by hand.\n"
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
    )

    resolution = metadata["resolution"]
    origin_x, origin_y = metadata["origin"]
    margin = 0.55
    minimum = np.min(fitted_points, axis=0) - margin
    maximum = np.max(fitted_points, axis=0) + margin
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
    )

    margin_px = 24
    header = 70
    footer = 145
    width = full.width + zoom.width + 3 * margin_px
    height = header + max(full.height, zoom.height) + footer
    canvas = Image.new("RGB", (width, height), "white")
    canvas.paste(full, (margin_px, header))
    canvas.paste(zoom, (full.width + 2 * margin_px, header))
    draw = ImageDraw.Draw(canvas)
    draw.text(
        (margin_px, 16),
        "Fitted pickup-staging global path on math_newest.pgm",
        fill=(20, 20, 20),
        font=load_font(26, bold=True),
    )
    legend_y = header + max(full.height, zoom.height) + 18
    legend = [
        ("active input points / original seq", (35, 90, 210)),
        ("original active-point polyline", (235, 139, 28)),
        ("regularized C2 cubic path", (210, 35, 45)),
        ("curvature-adaptive path samples", (0, 185, 210)),
        ("0.11 m centerline envelope", (255, 150, 150)),
        ("minimum-clearance location", (220, 0, 180)),
    ]
    x = margin_px
    y = legend_y
    for index, (label, color) in enumerate(legend):
        if index == 3:
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
        "--output",
        type=Path,
        default=workspace / "docs/navigation_path_fit_math_newest.png",
    )
    parser.add_argument(
        "--path-output",
        type=Path,
        default=workspace
        / "src/smart_factory_mission/config/pickup_staging_fitted_path.yaml",
    )
    parser.add_argument("--copy-to", type=Path)
    parser.add_argument("--smoothing-lambda", type=float, default=3.0e-4)
    parser.add_argument("--chord-error", type=float, default=0.01)
    parser.add_argument("--min-sample-spacing", type=float, default=0.08)
    parser.add_argument("--max-sample-spacing", type=float, default=0.30)
    args = parser.parse_args()

    records = read_active_route(args.route)
    points = np.asarray([[record[1], record[2]] for record in records])
    parameter = chord_parameter(points)
    anchors = regularized_anchors(points, parameter, args.smoothing_lambda)
    curve = NaturalCubicPath(parameter, anchors)
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
    )

    metadata = map_metadata(args.map_yaml)
    map_image = Image.open(metadata["image"]).convert("L")
    occupied, unknown, clearance, clearance_index = occupancy_diagnostics(
        map_image, metadata, fitted
    )
    if occupied or unknown:
        raise RuntimeError(
            f"fitted path is unsafe: occupied={occupied}, unknown={unknown}"
        )

    export_path_config(
        args.path_output,
        args.route,
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
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.save(args.output)
    if args.copy_to:
        args.copy_to.mkdir(parents=True, exist_ok=True)
        shutil.copy2(args.output, args.copy_to / args.output.name)

    print(f"output={args.output}")
    print(f"path_output={args.path_output}")
    if args.copy_to:
        print(f"copy={args.copy_to / args.output.name}")
    print(f"active_sequences={[record[0] for record in records]}")
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
