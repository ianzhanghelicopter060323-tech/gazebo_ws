import math
import sys
from pathlib import Path

import numpy as np
from PIL import Image

sys.path.insert(0, "src/gazebo_map/scripts")
import plot_fitted_navigation_path as fit_path

route = fit_path.read_active_route(
    Path("src/smart_factory_mission/config/pickup_staging_dev.yaml")
)
new_21_25 = {
    21: (2.608218193054199, -1.0063811540603638, -0.0),
    22: (2.5764060020446777, -0.977721095085144, 0.0),
    23: (2.284475564956665, -1.0575637817382812, 0.0),
    24: (1.9826356172561646, -1.0287030935287476, -0.0),
    25: (1.731062412261963, -1.0351046323776245, -0.0),
}


def replace(records, sequence, x, y):
    for index, record in enumerate(records):
        if record[0] == sequence:
            records[index] = (sequence, x, y, record[3])
            return
    raise ValueError(sequence)


def build_candidate():
    records = list(route)
    for sequence, (x, y, _yaw) in new_21_25.items():
        replace(records, sequence, x, y)
    replace(records, 16, 2.480, 0.000)
    replace(records, 17, 2.620, -0.150)
    index = next(i for i, record in enumerate(records) if record[0] == 17)
    records.insert(
        index,
        (160, 2.590356111526489, 0.008197352290153503, 0.0),
    )
    return records


def evaluate(records):
    points = np.asarray([[record[1], record[2]] for record in records])
    parameter = fit_path.chord_parameter(points)
    anchors = fit_path.regularized_anchors(points, parameter, 3.0e-4)
    curve = fit_path.NaturalCubicPath(parameter, anchors)
    dense_parameter, fitted, fitted_curvature, speed, dense_arc, sample_parameter, sample_arc, samples = fit_path.adaptive_samples(
        curve, parameter[-1], 0.01, 0.08, 0.30
    )
    metadata = fit_path.map_metadata(Path("src/gazebo_map/maps/math_newest.yaml"))
    image = Image.open(metadata["image"]).convert("L")
    occupied, unknown, clearance, _ = fit_path.occupancy_diagnostics(
        image, metadata, fitted
    )
    sign = np.sign(fitted_curvature)
    sign[np.abs(fitted_curvature) < 0.25] = 0.0
    sign_changes = np.count_nonzero(sign[1:] * sign[:-1] < 0)
    print("sequences:", [record[0] for record in records])
    print(
        "points=%d samples=%d max_curvature=%.3f min_radius=%.3f "
        "max_anchor_shift=%.3f occupied=%d unknown=%d clearance=%.3f "
        "sign_changes=%d"
        % (
            len(records),
            len(samples),
            np.max(np.abs(fitted_curvature)),
            1.0 / np.max(np.abs(fitted_curvature)),
            np.max(np.linalg.norm(anchors - points, axis=1)),
            occupied,
            unknown,
            clearance,
            sign_changes,
        )
    )
    anchor_s = dict(zip([record[0] for record in records], parameter))
    for start, end in (
        (14, 15), (15, 16), (16, 160), (160, 17), (17, 18),
        (20, 21), (21, 22), (22, 23), (23, 24), (24, 25), (25, 27),
    ):
        lo, hi = sorted((anchor_s[start], anchor_s[end]))
        mask = (dense_parameter >= lo) & (dense_parameter <= hi)
        local = fitted_curvature[mask]
        print(
            "segment %s->%s length=%.3f max_abs_curv=%.3f "
            "local_sign_changes=%d"
            % (
                start,
                end,
                hi - lo,
                np.max(np.abs(local)),
                np.count_nonzero(np.sign(local[1:]) * np.sign(local[:-1]) < 0),
            )
        )


evaluate(build_candidate())
