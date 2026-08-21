#!/usr/bin/env python3
"""Delete proven-clean recordings while the active 100-round run progresses."""

import argparse
import json
from pathlib import Path
import subprocess
import sys


WORKSPACE = Path(__file__).resolve().parents[1]
CLEANER = WORKSPACE / "script" / "prune_successful_gazebo_recordings.py"
DEFAULT_RECORDING_ROOT = (
    WORKSPACE / "data" / "cone_zone" / "end_to_end_test"
)
DEFAULT_MARKER = DEFAULT_RECORDING_ROOT / ".active_end_to_end_100_run.json"


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--marker", type=Path, default=DEFAULT_MARKER)
    parser.add_argument("--interval", type=float, default=5.0)
    return parser.parse_args(argv)


def load_marker(path):
    path = path.expanduser().resolve()
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        raise ValueError(
            "cannot read active 100-round marker {}: {}".format(path, exc)
        )
    if not isinstance(payload, dict) or payload.get("rounds") != 100:
        raise ValueError("active marker does not describe a 100-round run")
    run_name = str(payload.get("recording_name", ""))
    log_name = str(payload.get("log_name", ""))
    raw_root = str(payload.get("recording_root", "")).strip()
    root = Path(raw_root).expanduser().resolve()
    if (
        not run_name.startswith("end_to_end_cone_trials_")
        or "_seed" not in run_name
        or not log_name.startswith("end_to_end_cone_trials_")
        or run_name.rpartition("_seed")[0] != log_name
        or not raw_root
    ):
        raise ValueError("active marker contains an invalid run identity")
    return run_name, root


def build_command(run_name, recording_root, interval):
    return [
        sys.executable,
        str(CLEANER),
        "--watch",
        "--apply",
        "--run",
        run_name,
        "--normal-e2e-root",
        str(recording_root),
        "--logs-root",
        str(WORKSPACE / "script" / "logs"),
        "--interval",
        str(interval),
        "--stop-after-rounds",
        "100",
    ]


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    if args.interval <= 0.0:
        raise SystemExit("--interval must be positive")
    try:
        run_name, recording_root = load_marker(args.marker)
    except ValueError as exc:
        print("watch_100_end_to_end_recordings: {}".format(exc), file=sys.stderr)
        return 1
    command = build_command(run_name, recording_root, args.interval)
    print("monitoring_run={}".format(run_name))
    print("policy=delete only fully successful, collision-free recordings")
    print("+ " + " ".join(str(part) for part in command))
    return subprocess.run(command).returncode


if __name__ == "__main__":
    raise SystemExit(main())
