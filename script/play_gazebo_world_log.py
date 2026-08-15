#!/usr/bin/env python3
"""Validate and replay a per-round Gazebo world-state recording."""

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

from capture_pickup_dataset import (
    AutomationError,
    CleanupError,
    WORKSPACE,
    master_is_running,
    ros_command,
    stop_process_group,
)


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="replay gazebo_world_state.log with workspace resources"
    )
    parser.add_argument(
        "recording",
        type=Path,
        help=(
            "gazebo_world_state.log, gazebo_world_recording.json, or the "
            "round_NNN directory containing them"
        ),
    )
    parser.add_argument(
        "--run-immediately",
        action="store_true",
        help="start playback immediately instead of opening paused",
    )
    parser.add_argument("--headless", action="store_true", help="do not open gzclient")
    parser.add_argument(
        "--software-rendering",
        action="store_true",
        help="force Mesa llvmpipe instead of the default hardware renderer",
    )
    parser.add_argument(
        "--gpu-adapter",
        default="NVIDIA",
        help="WSLg D3D12 adapter name substring (default: NVIDIA)",
    )
    return parser.parse_args(argv)


def resolve_log_path(recording):
    """Resolve a state log from a log path, recording manifest, or round dir."""
    path = recording.expanduser().resolve()
    if path.is_dir():
        manifest = path / "gazebo_world_recording.json"
        if manifest.is_file():
            path = manifest
        else:
            path = path / "gazebo_world_state.log"

    if path.name == "gazebo_world_recording.json" and path.is_file():
        try:
            metadata = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, ValueError) as exc:
            raise AutomationError(
                "Gazebo recording manifest is invalid: {} ({})".format(path, exc)
            )
        recording_file = metadata.get("recording_file")
        if not isinstance(recording_file, str) or not recording_file.strip():
            raise AutomationError(
                "Gazebo recording manifest has no recording_file: {}".format(path)
            )
        path = (path.parent / recording_file).resolve()

    return path


def validate_log(path):
    if not path.is_file():
        raise AutomationError("Gazebo log does not exist: {}".format(path))
    result = subprocess.run(
        ["gz", "log", "-i", "-f", str(path)],
        cwd=str(WORKSPACE),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        timeout=15.0,
        check=False,
    )
    if result.returncode != 0 or "Log Version:" not in result.stdout:
        raise AutomationError(
            "Gazebo could not parse {}: {}".format(path, result.stdout.strip())
        )
    return result.stdout.strip()


def rendering_environment(args):
    environment = os.environ.copy()
    if args.software_rendering:
        environment["LIBGL_ALWAYS_SOFTWARE"] = "1"
        environment["GALLIUM_DRIVER"] = "llvmpipe"
        environment.pop("MESA_D3D12_DEFAULT_ADAPTER_NAME", None)
        return environment, "software (llvmpipe)"

    environment["LIBGL_ALWAYS_SOFTWARE"] = "0"
    # WSLg exposes the physical GPU through Mesa's D3D12 Gallium driver.
    # Only force that driver when the WSL D3D12 runtime is actually present;
    # native Linux keeps its normal GLVND / driver selection.
    if Path("/usr/lib/wsl/lib/libd3d12.so").is_file():
        environment["GALLIUM_DRIVER"] = "d3d12"
        environment["MESA_D3D12_DEFAULT_ADAPTER_NAME"] = args.gpu_adapter
        return environment, "hardware (WSLg D3D12, adapter={})".format(
            args.gpu_adapter
        )
    environment.pop("GALLIUM_DRIVER", None)
    environment.pop("MESA_D3D12_DEFAULT_ADAPTER_NAME", None)
    return environment, "hardware (system OpenGL driver)"


def opengl_renderer(environment):
    try:
        result = subprocess.run(
            ["glxinfo", "-B"],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=10.0,
            check=False,
            env=environment,
        )
    except (OSError, subprocess.SubprocessError):
        return "unavailable"
    for line in result.stdout.splitlines():
        if line.startswith("OpenGL renderer string:"):
            return line.split(":", 1)[1].strip()
    return "unavailable"


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    playback_process = None
    try:
        path = resolve_log_path(args.recording)
        info = validate_log(path)
        playback_environment, rendering_mode = rendering_environment(args)
        if master_is_running():
            raise CleanupError(
                "a ROS master is already running; stop the simulation before playback"
            )
        command = ros_command(
            [
                "roslaunch",
                "smart_factory_bringup",
                "play_gazebo_world_log.launch",
                "log_file:={}".format(path),
                "paused:={}".format("false" if args.run_immediately else "true"),
                "gui:={}".format("false" if args.headless else "true"),
            ]
        )
        print(info)
        print("replaying {}".format(path), flush=True)
        print("rendering_mode={}".format(rendering_mode), flush=True)
        print(
            "opengl_renderer={}".format(opengl_renderer(playback_environment)),
            flush=True,
        )
        print(
            "playback opens paused; click Gazebo's play button to begin"
            if not args.run_immediately
            else "playback starts immediately",
            flush=True,
        )
        playback_process = subprocess.Popen(
            command,
            cwd=str(WORKSPACE),
            start_new_session=True,
            env=playback_environment,
        )
        return playback_process.wait()
    except KeyboardInterrupt:
        stop_process_group(playback_process, interrupt_timeout=12.0)
        return 130
    except (AutomationError, CleanupError, OSError, subprocess.SubprocessError) as exc:
        stop_process_group(playback_process, interrupt_timeout=12.0)
        print("play_gazebo_world_log: {}".format(exc), file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
