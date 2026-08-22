#!/usr/bin/env python3
"""Replay every valid failed scene from source run 20260822_160246 twice."""

import argparse
import csv
from collections import Counter
import datetime as dt
from pathlib import Path
import subprocess
import sys


WORKSPACE = Path(__file__).resolve().parents[1]
HARNESS = WORKSPACE / "script" / "run_fixed_cone_e2e_stress_trials.py"
TEMPLATE_MANIFEST = (
    WORKSPACE
    / "script"
    / "config"
    / "fixed_failed_scenes_20260822_160246.json"
)
SOURCE_RUN = (
    "end_to_end_cone_trials_20260822_160246_seed5559192371148458023"
)
SOURCE_ROUNDS = (11, 24, 37, 41, 50, 51, 55, 57, 58)
REPETITIONS_PER_SCENE = 2
TOTAL_ROUNDS = len(SOURCE_ROUNDS) * REPETITIONS_PER_SCENE
DEFAULT_RECORDING_ROOT = (
    WORKSPACE / "data" / "cone_zone" / "end_to_end_stress"
)
DEFAULT_EXPERIMENT_LABEL = "source160246_failed_scenes_2each"


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
    parser.add_argument("--startup-timeout", type=float, default=180.0)
    parser.add_argument("--startup-retries", type=int, default=2)
    parser.add_argument("--task-timeout", type=float, default=480.0)
    parser.add_argument("--task-progress-timeout", type=float, default=360.0)
    parser.add_argument(
        "--recording-root", type=Path, default=DEFAULT_RECORDING_ROOT
    )
    parser.add_argument(
        "--log-dir",
        type=Path,
        help=(
            "result directory; default: "
            "script/logs/failed_scenes_20260822_160246_stress_<time>"
        ),
    )
    parser.add_argument(
        "--disable-gazebo-recording",
        action="store_true",
        help="disable per-round Gazebo world-state recording",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="validate the nine-scene, eighteen-round schedule only",
    )
    return parser.parse_args(argv)


def default_log_dir(now=None):
    timestamp = (now or dt.datetime.now()).strftime("%Y%m%d_%H%M%S")
    return (
        WORKSPACE
        / "script"
        / "logs"
        / "failed_scenes_20260822_160246_stress_{}".format(timestamp)
    )


def build_command(args, run_dir):
    command = [
        sys.executable,
        str(HARNESS),
        "--rounds",
        str(TOTAL_ROUNDS),
        "--templates",
        str(TEMPLATE_MANIFEST),
    ]
    for source_round in SOURCE_ROUNDS:
        command.extend(("--source-round", str(source_round)))
    command.extend(
        (
            "--log-dir",
            str(run_dir),
            "--recording-root",
            str(args.recording_root.expanduser().resolve()),
            "--experiment-label",
            DEFAULT_EXPERIMENT_LABEL,
            "--startup-timeout",
            str(args.startup_timeout),
            "--startup-retries",
            str(args.startup_retries),
            "--task-timeout",
            str(args.task_timeout),
            "--task-progress-timeout",
            str(args.task_progress_timeout),
            "--gui" if args.gui else "--headless",
        )
    )
    if args.disable_gazebo_recording:
        command.append("--disable-gazebo-recording")
    if args.dry_run:
        command.append("--dry-run")
    return command


def report(csv_path):
    with csv_path.open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    counts = Counter(int(row["source_round"]) for row in rows)
    successes = sum(row.get("success") == "True" for row in rows)
    collisions = sum(row.get("cone_collision") == "True" for row in rows)
    recording_failures = sum(
        row.get("gazebo_recording_status") not in {"complete", "disabled"}
        for row in rows
    )
    print(
        "failed-scene stress result: completed={}/{} success={} "
        "collisions={} recording_failures={}".format(
            len(rows), TOTAL_ROUNDS, successes, collisions, recording_failures
        )
    )
    for source_round in SOURCE_ROUNDS:
        print(
            "source_round={:03d} completed={}/{}".format(
                source_round, counts[source_round], REPETITIONS_PER_SCENE
            )
        )
    return (
        len(rows) == TOTAL_ROUNDS
        and all(
            counts[source_round] == REPETITIONS_PER_SCENE
            for source_round in SOURCE_ROUNDS
        )
        and successes == TOTAL_ROUNDS
        and collisions == 0
        and recording_failures == 0
    )


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    if args.startup_timeout <= 0.0:
        raise SystemExit("--startup-timeout must be positive")
    if args.startup_retries < 0:
        raise SystemExit("--startup-retries must be non-negative")
    if args.task_timeout <= 0.0:
        raise SystemExit("--task-timeout must be positive")
    if args.task_progress_timeout <= 0.0:
        raise SystemExit("--task-progress-timeout must be positive")
    run_dir = (
        args.log_dir.expanduser().resolve()
        if args.log_dir is not None
        else default_log_dir()
    )
    command = build_command(args, run_dir)
    print("source_run={}".format(SOURCE_RUN))
    print("source_rounds={}".format(list(SOURCE_ROUNDS)))
    print(
        "scenes={} repetitions_per_scene={} total_rounds={}".format(
            len(SOURCE_ROUNDS), REPETITIONS_PER_SCENE, TOTAL_ROUNDS
        )
    )
    print(
        "gui={} gazebo_recording={} recording_root={}".format(
            args.gui,
            not args.disable_gazebo_recording,
            args.recording_root.expanduser().resolve(),
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
