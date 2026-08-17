#!/usr/bin/env python3
"""Compute corridor (lateral free-space) width along the pre-seq35 staging route.

Reads math_newest.pgm (map_saver P5) + pickup_staging_fitted_path.yaml dense
points. For each path point, casts a ray to the nearest occupied cell in the
lateral (perpendicular to heading) direction on both sides, then reports the
width distribution over the pre-seq35 segment (execution waypoints 1..30).

PGM pixel mapping: col = (x - ox)/res ; row = height-1 - (y - oy)/res
(origin is the LOWER-LEFT corner; image row 0 is the NORTH edge).
"""
import math
import sys
import yaml

MAP_PATH = "/home/ianichinose/gazebo_ws/src/gazebo_map/maps/math_newest.pgm"
MAP_YAML = "/home/ianichinose/gazebo_ws/src/gazebo_map/maps/math_newest.yaml"
PATH_YAML = "/home/ianichinose/gazebo_ws/src/smart_factory_navigation/config/pickup_staging_fitted_path.yaml"


def load_pgm(path):
    with open(path, "rb") as f:
        data = f.read()
    idx = 0
    parts = []
    while len(parts) < 4:
        while data[idx:idx + 1].isspace():
            idx += 1
        if data[idx:idx + 1] == b"#":
            while data[idx:idx + 1] != b"\n":
                idx += 1
            continue
        start = idx
        while not data[idx:idx + 1].isspace():
            idx += 1
        parts.append(data[start:idx])
    magic, width_s, height_s, maxval_s = parts
    assert magic == b"P5", magic
    width, height, maxval = int(width_s), int(height_s), int(maxval_s)
    while data[idx:idx + 1].isspace():
        idx += 1
    assert maxval == 255
    raw = data[idx:]
    n = width * height
    assert len(raw) >= n
    return width, height, list(raw[:n])


def main():
    with open(MAP_YAML) as f:
        meta = yaml.safe_load(f)
    res = meta["resolution"]
    ox, oy, _ = meta["origin"]
    w_, h_, pixels = load_pgm(MAP_PATH)

    # occupied mask (map_saver: 0=free, 254=occupied-inverse... value 0 = occupied)
    occ = [1 if p <= 127 else 0 for p in pixels]

    def pixel_of(x, y):
        col = int((x - ox) / res)
        row = h_ - 1 - int((y - oy) / res)
        if not (0 <= col < w_ and 0 <= row < h_):
            return None
        return row * w_ + col

    def ray_dist(x, y, ux, uy, max_len=12.0):
        """Distance in meters to first occupied cell along unit dir from (x,y).
        Returns None if the ray exits the map before hitting an obstacle."""
        steps = int(max_len / res) + 1
        for i in range(1, steps + 1):
            idx = pixel_of(x + ux * res * i, y + uy * res * i)
            if idx is None:
                return None
            if occ[idx]:
                return res * i
        return None

    with open(PATH_YAML) as f:
        doc = yaml.safe_load(f)
    fp = doc["fitted_path"]
    points = fp["points"]
    wp = fp["execution_waypoints"]

    # Corridor width at each dense point: left+right lateral clearances.
    widths = []
    for p in points:
        x, y = p["x"], p["y"]
        yaw = p["yaw"]
        lx, ly = math.cos(yaw + math.pi / 2), math.sin(yaw + math.pi / 2)
        rx, ry = math.cos(yaw - math.pi / 2), math.sin(yaw - math.pi / 2)
        dl = ray_dist(x, y, lx, ly)
        dr = ray_dist(x, y, rx, ry)
        if dl is None or dr is None:
            widths.append(None)
        else:
            widths.append(dl + dr)

    finite = [w for w in widths if w is not None]
    print("dense points:", len(points), "  finite widths:", len(finite))
    print("corridor width: min=%.3f  median=%.3f  mean=%.3f  max=%.3f"
          % (min(finite), sorted(finite)[len(finite) // 2],
             sum(finite) / len(finite), max(finite)))

    fs = sorted(finite)
    for q in (10, 25, 50, 75, 90, 95, 99, 100):
        print("  p%02d = %.3f m" % (q, fs[min(len(fs) - 1, int(q / 100.0 * (len(fs) - 1)))]))

    # Waypoint-level: width at each execution waypoint (nearest dense point by s)
    print("\nexecution waypoint corridor widths (pre-seq35 segment):")
    for w in wp:
        num = w["waypoint"]
        target_s = w["s"]
        best = min(points, key=lambda p: abs(p["s"] - target_s))
        i = points.index(best)
        ww = widths[i]
        ws = "%.3f" % ww if ww is not None else "  --  "
        print("  wp%2d s=%7.3f (%7.3f,%7.3f) width=%s m" % (num, w["s"], w["x"], w["y"], ws))

    print("\nmax corridor width (pre-seq35): %.3f m" % max(finite))
    print("90%% of max corridor width: %.3f m" % (0.9 * max(finite)))


if __name__ == "__main__":
    sys.exit(main())
