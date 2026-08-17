#!/usr/bin/env python3
"""Replay the 7 cone-zone channel-failure scenarios under a tuned avoidance
footprint, with the Gazebo GUI, and report whether each scenario is solved.

Purpose
-------
The most recent 40-round end-to-end run failed 7 rounds (4, 9, 16, 21, 22, 24,
33), all of which jammed in the cone-zone entry bottleneck.  Every failure
surfaced as GOAL_UNAVAILABLE (error code 5) after the rolling-waypoint channel
could not produce a safe route for the 0.30 m avoidance footprint.  This script
replays those exact cone layouts through the full mission (pre-navigation
included, as in the original run) and lets the user judge whether a smaller
avoidance footprint improves passability.

Acceptance criterion (per the user)
-----------------------------------
A scenario counts as solved when it reaches the designated workshop without a
cone collision twice in a row (two clean passes).  With 7 scenarios that is 14
trials; the harness round-robins the templates so trials 1-7 are pass 1 and
trials 8-14 are pass 2.

Scenarios are replayed through ``run_fixed_cone_e2e_stress_trials.py`` (the
unchanged full-mission harness) with a fixed-cone manifest whose layouts were
extracted from the original run's Gazebo state logs.  Pre-navigation is
included because the mission server owns the rolling-waypoint channel; there is
no supported skip-to-prep mechanism and the mission files are intentionally not
modified.

Usage
-----
    python3 script/run_avoidance_footprint_pressure_test.py [--rounds 14]

The experiment label is derived from the avoidance footprint half-extent
currently configured in ``adaptive_teb_params.yaml`` and the channel selector
hard-clearance floor in ``delivery_goals.yaml`` (e.g. 0.12 + 0.16 ->
avoidfp_0p12_hardclear_0p16) so the harness's label/runtime validation passes
automatically.  Edit either yaml between iterations and re-run; the derived
label follows.
"""

import argparse
import csv
import datetime as dt
import subprocess
import sys
from pathlib import Path

import yaml

WORKSPACE = Path(__file__).resolve().parent.parent
SCRIPT_DIR = Path(__file__).resolve().parent
ADAPTIVE_TEB_CONFIG = (
    WORKSPACE
    / "src"
    / "gazebo_nav"
    / "launch"
    / "config"
    / "move_base"
    / "adaptive_teb_params.yaml"
)
DELIVERY_GOALS_CONFIG = (
    WORKSPACE
    / "src"
    / "smart_factory_mission"
    / "config"
    / "delivery_goals.yaml"
)
HARNESS = SCRIPT_DIR / "run_fixed_cone_e2e_stress_trials.py"
DEFAULT_MANIFEST = (
    SCRIPT_DIR
    / "config"
    / "cone_stress_channel_failures_rounds_004_009_016_021_022_024_033.json"
)
DEFAULT_SOURCE_ROUNDS = (4, 9, 16, 21, 22, 24, 33)


def finite(value, label):
    value = float(value)
    if value != value or value in (float("inf"), float("-inf")):
        raise ValueError("{} must be finite".format(label))
    return value


def configured_footprint_half_extent():
    """Read the avoidance footprint half-extent from adaptive_teb_params.yaml."""
    document = yaml.safe_load(ADAPTIVE_TEB_CONFIG.read_text(encoding="utf-8"))
    vertices = document["AdaptiveTebLocalPlannerROS"]["avoidance"][
        "footprint_model"
    ]["vertices"]
    return max(
        max(
            abs(finite(point[0], "avoidance footprint x")),
            abs(finite(point[1], "avoidance footprint y")),
        )
        for point in vertices
    )


def label_for_half_extent(half_extent):
    return "avoidfp_" + repr(half_extent).replace(".", "p")


def configured_channel_hard_clearance():
    """Read the channel selector hard-clearance floor from delivery_goals.yaml."""
    document = yaml.safe_load(DELIVERY_GOALS_CONFIG.read_text(encoding="utf-8"))
    return finite(
        document["delivery"]["entry_selector"]["channel_hard_min_clearance"],
        "channel hard clearance",
    )


def configured_channel_fan_half_angle():
    """Read the channel fan half-angle (degrees) from delivery_goals.yaml."""
    document = yaml.safe_load(DELIVERY_GOALS_CONFIG.read_text(encoding="utf-8"))
    return finite(
        document["delivery"]["entry_selector"]["channel_fan_half_angle_deg"],
        "channel fan half angle",
    )


def label_for_configuration(half_extent, hard_clearance, fan_half_angle=None):
    label = label_for_half_extent(half_extent)
    if hard_clearance is not None:
        label += "_hardclear_" + repr(hard_clearance).replace(".", "p")
    if fan_half_angle is not None:
        label += "_fan{}".format(int(round(fan_half_angle)))
    return label


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--rounds", type=int, default=14,
        help="total trials; default 14 (7 scenarios x 2 clean passes)",
    )
    display = parser.add_mutually_exclusive_group()
    display.add_argument(
        "--gui", dest="gui", action="store_true",
        help="start the Gazebo client window (default)",
    )
    display.add_argument(
        "--headless", dest="gui", action="store_false",
        help="run Gazebo without the client window",
    )
    parser.set_defaults(gui=True)
    parser.add_argument(
        "--templates", type=Path, default=DEFAULT_MANIFEST,
        help="fixed-cone template manifest (default: the 7 failed rounds)",
    )
    parser.add_argument(
        "--source-round", type=int, action="append", dest="source_rounds",
        default=list(DEFAULT_SOURCE_ROUNDS),
        help="scenario source round; may be repeated (default: 4 9 16 21 22 24 33)",
    )
    parser.add_argument(
        "--experiment-label", default=None,
        help="label passed to the harness; defaults to avoidfp_<half_extent>",
    )
    parser.add_argument(
        "--task-timeout", type=float, default=600.0,
        help="wall-clock deadline per task (default: 600 s; headroom for a "
        "rollback+reselect cycle after a first-waypoint stuck)",
    )
    parser.add_argument(
        "--task-progress-timeout", type=float, default=480.0,
        help="cancel a task after this many seconds without a new mission "
        "stage/detail (default: 480 s; a delivery-channel rollback stays in "
        "the same stage for several minutes)",
    )
    parser.add_argument(
        "--dry-run", action="store_true",
        help="validate configuration and print the schedule without launching",
    )
    return parser.parse_args(argv)


def run_harness(args, run_dir):
    command = [
        sys.executable,
        str(HARNESS),
        "--rounds", str(args.rounds),
        "--templates", str(args.templates),
        "--log-dir", str(run_dir),
        "--experiment-label", args.experiment_label,
        "--task-timeout", str(args.task_timeout),
        "--task-progress-timeout", str(args.task_progress_timeout),
    ]
    for source_round in args.source_rounds:
        command.extend(["--source-round", str(source_round)])
    if args.gui:
        command.append("--gui")
    else:
        command.append("--headless")
    if args.dry_run:
        command.append("--dry-run")
    print("+ " + " ".join(str(part) for part in command))
    result = subprocess.run(command)
    return result.returncode


def report_acceptance(csv_path):
    """Per-scenario acceptance: >=2 clean passes and no collision."""
    rows = list(csv.DictReader(csv_path.open(newline="", encoding="utf-8")))
    if not rows:
        print("no trial rows in {}".format(csv_path))
        return False
    by_round = {}
    for row in rows:
        by_round.setdefault(row["source_round"], []).append(row)
    print()
    print("=" * 70)
    print("per-scenario acceptance (clean pass = success and no cone collision)")
    print("=" * 70)
    header = "{:<6} {:<12} {:<7} {:<9} {:<9} {:<6}".format(
        "round", "target", "clean", "failed", "collided", "verdict"
    )
    print(header)
    print("-" * 70)
    all_pass = True
    for source_round in sorted(by_round, key=int):
        trials = by_round[source_round]
        clean = sum(
            1
            for r in trials
            if r.get("success") == "True" and r.get("cone_collision") != "True"
        )
        failed = sum(1 for r in trials if r.get("success") != "True")
        collided = sum(1 for r in trials if r.get("cone_collision") == "True")
        target = trials[0].get("target_class", "")
        pass_ = clean >= 2 and collided == 0
        all_pass = all_pass and pass_
        print("{:<6} {:<12} {:<7} {:<9} {:<9} {:<6}".format(
            source_round, target, clean, failed, collided,
            "PASS" if pass_ else "FAIL",
        ))
    print("=" * 70)
    print("overall: {}".format("ALL SCENARIOS SOLVED" if all_pass else "NOT ALL SOLVED"))
    return all_pass


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    half_extent = configured_footprint_half_extent()
    hard_clearance = configured_channel_hard_clearance()
    fan_half_angle = configured_channel_fan_half_angle()
    if args.experiment_label is None:
        args.experiment_label = label_for_configuration(
            half_extent, hard_clearance, fan_half_angle
        )
    if not args.templates.is_file():
        raise SystemExit("manifest not found: {}".format(args.templates))
    if not HARNESS.is_file():
        raise SystemExit("harness not found: {}".format(HARNESS))
    print(
        "avoidance footprint half-extent={:.3f} channel_hard_clearance={:.3f} "
        "fan_half_angle={:.0f} label={}".format(
            half_extent, hard_clearance, fan_half_angle, args.experiment_label
        )
    )
    timestamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = (
        WORKSPACE
        / "script"
        / "logs"
        / "fixed_cone_e2e_stress_{}_{}".format(timestamp, args.experiment_label)
    )
    print(
        "scenarios={} rounds={} gui={} log_dir={}".format(
            len(args.source_rounds), args.rounds, args.gui, run_dir
        )
    )
    if args.dry_run:
        print("dry-run: validating against the harness and printing the schedule")
    returncode = run_harness(args, run_dir)
    if args.dry_run:
        return 0 if returncode == 0 else 1
    csv_path = run_dir / "trials.csv"
    print("harness exit code={}".format(returncode))
    if csv_path.is_file():
        accepted = report_acceptance(csv_path)
        return 0 if (returncode == 0 and accepted) else 1
    print("no trials.csv at {}".format(csv_path))
    return returncode if returncode != 0 else 1


if __name__ == "__main__":
    sys.exit(main())
