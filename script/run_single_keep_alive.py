#!/usr/bin/env python3
"""Run a small set of fixed-cone rounds but LEAVE the simulation running.

The stock harness tears the ROS/Gazebo launch down in every round's ``finally``
(``stop_owned_launch``).  When the user wants to inspect the robot's post-failure
state in the Gazebo GUI, that teardown defeats the purpose.  This driver runs the
unmodified harness (``run_fixed_cone_e2e_stress_trials.main``) with
``stop_owned_launch`` monkey-patched to a no-op, so after the last round the
roslaunch / gzserver / mission server / GUI stay alive.  The user kills them
manually.

Example (source 16, one round, GUI):
    python3 script/run_single_keep_alive.py --rounds 1 --source-round 16 --gui
"""

import sys
from pathlib import Path

import run_fixed_cone_e2e_stress_trials as harness

harness.stop_owned_launch = lambda *args, **kwargs: None  # keep sim alive


def main(argv):
    argv = list(argv)
    if not any(a.startswith("--templates") for a in argv):
        argv += [
            "--templates",
            str(
                Path(__file__).resolve().parent
                / "config"
                / "cone_stress_channel_failures_rounds_004_009_016_021_022_024_033.json"
            ),
        ]
    print("[keep-alive] patched stop_owned_launch -> no-op; sim stays up after round(s)")
    return harness.main(argv)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
