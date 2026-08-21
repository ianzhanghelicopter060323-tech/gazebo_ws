#!/usr/bin/env python3
"""Run 100 normal random-scene end-to-end trials without Gazebo GUI."""

import argparse
import datetime as dt
import json
from pathlib import Path
import secrets
import subprocess
import sys


WORKSPACE = Path(__file__).resolve().parents[1]
HARNESS = WORKSPACE / "script" / "run_end_to_end_cone_trials.py"
DEFAULT_RECORDING_ROOT = (
    WORKSPACE / "data" / "cone_zone" / "end_to_end_test"
)
ACTIVE_RUN_MARKER = DEFAULT_RECORDING_ROOT / ".active_end_to_end_100_run.json"
ROUNDS = 100


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--seed",
        type=int,
        help="task RNG seed; omitted means generate and record one",
    )
    display = parser.add_mutually_exclusive_group()
    display.add_argument("--gui", dest="gui", action="store_true")
    display.add_argument("--headless", dest="gui", action="store_false")
    parser.set_defaults(gui=False)
    parser.add_argument("--startup-timeout", type=float, default=90.0)
    parser.add_argument("--task-timeout", type=float, default=480.0)
    parser.add_argument(
        "--recording-root", type=Path, default=DEFAULT_RECORDING_ROOT
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="validate and print the 100-round task sequence only",
    )
    return parser.parse_args(argv)


def build_run_identity(args, now=None):
    timestamp = (now or dt.datetime.now()).strftime("%Y%m%d_%H%M%S")
    seed = args.seed if args.seed is not None else secrets.randbits(64)
    log_name = "end_to_end_cone_trials_{}".format(timestamp)
    recording_name = "{}_seed{}".format(log_name, seed)
    return seed, log_name, recording_name


def marker_path(recording_root):
    root = recording_root.expanduser().resolve()
    if root == DEFAULT_RECORDING_ROOT.resolve():
        return ACTIVE_RUN_MARKER
    return root / ACTIVE_RUN_MARKER.name


def write_marker(path, payload):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def build_command(args, seed, log_name):
    command = [
        sys.executable,
        str(HARNESS),
        "--rounds",
        str(ROUNDS),
        "--seed",
        str(seed),
        "--log-dir",
        str(WORKSPACE / "script" / "logs" / log_name),
        "--recording-root",
        str(args.recording_root.expanduser().resolve()),
        "--startup-timeout",
        str(args.startup_timeout),
        "--task-timeout",
        str(args.task_timeout),
        "--gui" if args.gui else "--headless",
    ]
    if args.dry_run:
        command.append("--dry-run")
    return command


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    if args.startup_timeout <= 0.0:
        raise SystemExit("--startup-timeout must be positive")
    if args.task_timeout <= 0.0:
        raise SystemExit("--task-timeout must be positive")
    seed, log_name, recording_name = build_run_identity(args)
    command = build_command(args, seed, log_name)
    marker = marker_path(args.recording_root)
    marker_payload = {
        "schema_version": 1,
        "created_at": dt.datetime.now().astimezone().isoformat(timespec="seconds"),
        "rounds": ROUNDS,
        "seed": seed,
        "gui": args.gui,
        "log_name": log_name,
        "recording_name": recording_name,
        "recording_root": str(args.recording_root.expanduser().resolve()),
        "status": "dry_run" if args.dry_run else "running",
    }
    if not args.dry_run:
        write_marker(marker, marker_payload)

    print("rounds={} seed={} gui={}".format(ROUNDS, seed, args.gui))
    print("recording_run={}".format(recording_name))
    if not args.dry_run:
        print("cleanup_marker={}".format(marker))
    print("+ " + " ".join(str(part) for part in command))
    result = subprocess.run(command)
    if not args.dry_run:
        marker_payload.update(
            {
                "completed_at": dt.datetime.now().astimezone().isoformat(
                    timespec="seconds"
                ),
                "return_code": result.returncode,
                "status": "complete" if result.returncode == 0 else "failed",
            }
        )
        write_marker(marker, marker_payload)
    return result.returncode


if __name__ == "__main__":
    raise SystemExit(main())
