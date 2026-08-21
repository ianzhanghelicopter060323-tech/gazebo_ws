#!/usr/bin/env python3
"""Replay source round 004 as a five-round full end-to-end stress test."""

import argparse
import csv
import datetime as dt
from pathlib import Path
import subprocess
import sys


WORKSPACE = Path(__file__).resolve().parents[1]
HARNESS = WORKSPACE / "script" / "run_fixed_cone_e2e_stress_trials.py"
TEMPLATE = (
    WORKSPACE
    / "script"
    / "config"
    / "fixed_cone_e2e_20260821_223335_round004.json"
)
DEFAULT_RECORDING_ROOT = (
    WORKSPACE / "data" / "cone_zone" / "end_to_end_stress"
)
DEFAULT_LABEL = "round004_cone15_electronics"


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--rounds",
        type=int,
        default=5,
        help="number of full-mission repetitions (default: 5)",
    )
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
    parser.add_argument(
        "--recording-root",
        type=Path,
        default=DEFAULT_RECORDING_ROOT,
        help="Gazebo world-state recording root",
    )
    parser.add_argument(
        "--log-dir",
        type=Path,
        help="result directory; default: script/logs/fixed_cone_e2e_stress_<time>_round004_cone15_electronics",
    )
    parser.add_argument(
        "--experiment-label",
        default=DEFAULT_LABEL,
        help="filesystem-safe experiment label",
    )
    parser.add_argument("--task-timeout", type=float, default=600.0)
    parser.add_argument("--task-progress-timeout", type=float, default=480.0)
    parser.add_argument(
        "--disable-gazebo-recording",
        action="store_true",
        help="disable per-round Gazebo world-state recording",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="validate the fixed scene and print the five-round schedule",
    )
    return parser.parse_args(argv)


def build_command(args, run_dir):
    command = [
        sys.executable,
        str(HARNESS),
        "--rounds",
        str(args.rounds),
        "--templates",
        str(TEMPLATE),
        "--source-round",
        "4",
        "--log-dir",
        str(run_dir),
        "--recording-root",
        str(args.recording_root),
        "--experiment-label",
        args.experiment_label,
        "--task-timeout",
        str(args.task_timeout),
        "--task-progress-timeout",
        str(args.task_progress_timeout),
        "--gui" if args.gui else "--headless",
    ]
    if args.disable_gazebo_recording:
        command.append("--disable-gazebo-recording")
    if args.dry_run:
        command.append("--dry-run")
    return command


def report(csv_path, requested_rounds):
    with csv_path.open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    successes = sum(row.get("success") == "True" for row in rows)
    collisions = sum(row.get("cone_collision") == "True" for row in rows)
    clean = sum(
        row.get("success") == "True"
        and row.get("cone_collision") == "False"
        for row in rows
    )
    print(
        "round004 pressure result: completed={}/{} success={} clean={} "
        "collisions={}".format(
            len(rows), requested_rounds, successes, clean, collisions
        )
    )
    return len(rows) == requested_rounds and clean == requested_rounds


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    if args.rounds <= 0:
        raise SystemExit("--rounds must be positive")
    timestamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = (
        args.log_dir.expanduser().resolve()
        if args.log_dir is not None
        else WORKSPACE
        / "script"
        / "logs"
        / "fixed_cone_e2e_stress_{}_{}".format(
            timestamp, args.experiment_label
        )
    )
    command = build_command(args, run_dir)
    print("source_run=end_to_end_cone_trials_20260821_223335_seed8402119400439022545")
    print("source_round=4 target=electronics collision=cone_15")
    print("rounds={} gui={} recording_root={}".format(
        args.rounds, args.gui, args.recording_root.expanduser().resolve()
    ))
    print("+ " + " ".join(str(part) for part in command))
    result = subprocess.run(command)
    if args.dry_run:
        return result.returncode
    csv_path = run_dir / "trials.csv"
    if not csv_path.is_file():
        return result.returncode if result.returncode else 1
    accepted = report(csv_path, args.rounds)
    return 0 if result.returncode == 0 and accepted else 1


if __name__ == "__main__":
    raise SystemExit(main())
