#!/usr/bin/env python3
"""Replay the latest electronics delivery scene for five navigation trials."""

import argparse
import csv
import datetime as dt
from pathlib import Path
import subprocess
import sys


WORKSPACE = Path(__file__).resolve().parents[1]
HARNESS = WORKSPACE / "script" / "run_cone_move_stress_trials.py"
TEMPLATE_MANIFEST = (
    WORKSPACE
    / "script"
    / "config"
    / "fixed_cone_e2e_20260821_223335_round004.json"
)
TEMPLATE_ID = "collision_round_004"
TARGET_SEQUENCE = ("electronics",) * 5
DEFAULT_RECORDING_ROOT = (
    WORKSPACE / "data" / "cone_zone" / "workshop_navigation_stress"
)


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    display = parser.add_mutually_exclusive_group()
    display.add_argument(
        "--gui",
        dest="gui",
        action="store_true",
        help="start the Gazebo client window",
    )
    display.add_argument(
        "--headless",
        dest="gui",
        action="store_false",
        help="run without gzclient (default)",
    )
    parser.set_defaults(gui=False)
    parser.add_argument("--startup-timeout", type=float, default=90.0)
    parser.add_argument("--navigation-timeout", type=float, default=180.0)
    parser.add_argument(
        "--recording-root",
        type=Path,
        default=DEFAULT_RECORDING_ROOT,
        help="Gazebo world-state recording root",
    )
    parser.add_argument(
        "--log-dir",
        type=Path,
        help=(
            "result directory; default: "
            "script/logs/fixed_workshop_navigation_stress_<time>"
        ),
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="validate the fixed five-round schedule without starting Gazebo",
    )
    return parser.parse_args(argv)


def default_log_dir(now=None):
    timestamp = (now or dt.datetime.now()).strftime("%Y%m%d_%H%M%S")
    return (
        WORKSPACE
        / "script"
        / "logs"
        / "fixed_workshop_navigation_stress_{}".format(timestamp)
    )


def build_command(args, run_dir):
    return [
        sys.executable,
        str(HARNESS),
        "--templates",
        str(TEMPLATE_MANIFEST),
        "--template-id",
        TEMPLATE_ID,
        "--target-sequence",
        ",".join(TARGET_SEQUENCE),
        "--log-dir",
        str(run_dir),
        "--recording-root",
        str(args.recording_root.expanduser().resolve()),
        "--startup-timeout",
        str(args.startup_timeout),
        "--navigation-timeout",
        str(args.navigation_timeout),
        "--gui" if args.gui else "--headless",
    ] + (["--dry-run"] if args.dry_run else [])


def report(csv_path):
    with csv_path.open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    successes = sum(row.get("success") == "True" for row in rows)
    collisions = sum(row.get("cone_collision") == "True" for row in rows)
    recordings = sum(
        row.get("gazebo_recording_status") == "complete" for row in rows
    )
    print(
        "workshop navigation stress result: completed={}/5 success={} "
        "collisions={} recordings={}".format(
            len(rows), successes, collisions, recordings
        )
    )
    return (
        len(rows) == 5
        and successes == 5
        and collisions == 0
        and recordings == 5
    )


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    if args.startup_timeout <= 0.0:
        raise SystemExit("--startup-timeout must be positive")
    if args.navigation_timeout <= 0.0:
        raise SystemExit("--navigation-timeout must be positive")
    run_dir = (
        args.log_dir.expanduser().resolve()
        if args.log_dir is not None
        else default_log_dir()
    )
    command = build_command(args, run_dir)
    print("fixed_scene_manifest={}".format(TEMPLATE_MANIFEST))
    print("fixed_scene_template={}".format(TEMPLATE_ID))
    print("rounds=5 targets={}".format(",".join(TARGET_SEQUENCE)))
    print(
        "gui={} recording_root={}".format(
            args.gui, args.recording_root.expanduser().resolve()
        )
    )
    print("+ " + " ".join(str(part) for part in command))
    result = subprocess.run(command)
    if args.dry_run:
        return result.returncode
    csv_path = run_dir / "trials.csv"
    if not csv_path.is_file():
        return result.returncode if result.returncode else 1
    accepted = report(csv_path)
    return 0 if result.returncode == 0 and accepted else 1


if __name__ == "__main__":
    raise SystemExit(main())
