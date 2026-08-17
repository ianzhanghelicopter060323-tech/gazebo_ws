#!/usr/bin/env python3
"""Run isolated pickup-route navigation trials through seq35."""

import argparse
import csv
import datetime as dt
import json
import math
import os
from pathlib import Path
import socket
import subprocess
import sys
import time

import yaml

from capture_pickup_dataset import (
    AutomationError,
    CleanupError,
    SETUP_FILE,
    WORKSPACE,
    launch_simulation,
    master_is_running,
    read_ros_run_id,
    run_owned,
    stop_owned_launch,
    stop_process_group,
    wait_for_ros,
    wait_for_simulation,
)
from run_end_to_end_cone_trials import (
    GAZEBO_RECORDER_SCRIPT,
    add_gazebo_recording_result,
    append_section,
    start_gazebo_world_recorder,
)


DEFAULT_ROUNDS = 100
# These defaults intentionally isolate this test from the normal 11311/11345
# pair. Override them from the command line when another isolated run uses them.
DEFAULT_ROS_MASTER_PORT = 1132
DEFAULT_GAZEBO_MASTER_PORT = 1133
DEFAULT_RECORDING_ROOT = WORKSPACE / "data" / "pre_navigation_test"
DEFAULT_ROUTE_CONFIG = (
    WORKSPACE
    / "src"
    / "smart_factory_mission"
    / "config"
    / "pickup_staging_dev.yaml"
)
DEFAULT_FITTED_PATH_CONFIG = (
    WORKSPACE
    / "src"
    / "smart_factory_navigation"
    / "config"
    / "pickup_staging_fitted_path.yaml"
)
DEFAULT_NAVIGATION_CONFIG = (
    WORKSPACE
    / "src"
    / "smart_factory_navigation"
    / "config"
    / "navigation.yaml"
)
CLIENT_SCRIPT = WORKSPACE / "script" / "_run_pre_navigation_route.py"
RESULT_MARKER = "PRE_NAVIGATION_RESULT="
BASELINE_LOCK_SERVICE = (
    "/move_base/AdaptiveTebLocalPlannerROS/set_baseline_lock"
)
CSV_FIELDS = (
    "round",
    "task_id",
    "success",
    "status",
    "error_code",
    "server_error_code",
    "message",
    "acceptance_mode",
    "yaw_alignment_required",
    "completed_waypoints",
    "waypoint_count",
    "final_position_error_m",
    "final_position_tolerance_m",
    "final_yaw_error_rad",
    "final_x",
    "final_y",
    "final_yaw",
    "gazebo_recording_status",
    "gazebo_recording_path",
    "gazebo_recording_size_bytes",
    "gazebo_recording_manifest",
    "ros_master_port",
    "gazebo_master_port",
    "duration_seconds",
    "started_at",
    "round_dir",
    "log_file",
)


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description=(
            "restart an isolated headless simulation for every round, follow "
            "the configured pickup route through seq35, accept seq35 by "
            "position without final yaw alignment, and record Gazebo state"
        )
    )
    parser.add_argument(
        "--rounds",
        type=int,
        default=DEFAULT_ROUNDS,
        help="number of isolated trials (default: 100)",
    )
    display = parser.add_mutually_exclusive_group()
    display.add_argument("--gui", dest="gui", action="store_true")
    display.add_argument("--headless", dest="gui", action="store_false")
    parser.set_defaults(gui=False)
    parser.add_argument(
        "--ros-master-port",
        type=int,
        default=DEFAULT_ROS_MASTER_PORT,
        help="isolated ROS master TCP port (default: 1132)",
    )
    parser.add_argument(
        "--gazebo-master-port",
        type=int,
        default=DEFAULT_GAZEBO_MASTER_PORT,
        help="isolated Gazebo master TCP port (default: 1133)",
    )
    parser.add_argument(
        "--seq35-position-tolerance",
        type=float,
        default=None,
        help=(
            "optional test-only override for the seq35 XY arrival radius; "
            "the default reads navigation/final_pass_radius from "
            "--navigation-config"
        ),
    )
    parser.add_argument("--startup-timeout", type=float, default=90.0)
    parser.add_argument("--startup-settle", type=float, default=8.0)
    parser.add_argument("--navigation-timeout", type=float, default=300.0)
    parser.add_argument(
        "--gazebo-recording-ready-timeout", type=float, default=15.0
    )
    parser.add_argument("--restart-settle", type=float, default=3.0)
    parser.add_argument(
        "--recording-root", type=Path, default=DEFAULT_RECORDING_ROOT
    )
    parser.add_argument("--route-config", type=Path, default=DEFAULT_ROUTE_CONFIG)
    parser.add_argument(
        "--fitted-path-config", type=Path, default=DEFAULT_FITTED_PATH_CONFIG
    )
    parser.add_argument(
        "--navigation-config", type=Path, default=DEFAULT_NAVIGATION_CONFIG
    )
    parser.add_argument(
        "--log-dir",
        type=Path,
        help="result directory; default: script/logs/pre_navigation_trials_<time>_ros<port>",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="validate configuration and print paths/ports without starting ROS",
    )
    return parser.parse_args(argv)


def finite_float(value, label):
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise AutomationError("{} must be numeric".format(label)) from exc
    if not math.isfinite(result):
        raise AutomationError("{} must be finite".format(label))
    return result


def validate_route_pair(route_path, fitted_path):
    try:
        route_payload = yaml.safe_load(route_path.read_text(encoding="utf-8"))
        fitted_payload = yaml.safe_load(fitted_path.read_text(encoding="utf-8"))
        route = route_payload["pickup_staging"]
        waypoints = route["waypoints"]
        final_goal = fitted_payload["fitted_path"]["final_goal"]
    except (OSError, KeyError, TypeError, yaml.YAMLError) as exc:
        raise AutomationError("cannot load route/fitted path configuration: {}".format(exc))
    if not isinstance(waypoints, list) or len(waypoints) < 2:
        raise AutomationError("pickup route must contain at least two waypoints")
    final = waypoints[-1]
    final_values = tuple(
        finite_float(final.get(key), "route final {}".format(key))
        for key in ("x", "y", "yaw")
    )
    fitted_values = tuple(
        finite_float(final_goal.get(key), "fitted final {}".format(key))
        for key in ("x", "y", "yaw")
    )
    position_error = math.hypot(
        final_values[0] - fitted_values[0], final_values[1] - fitted_values[1]
    )
    yaw_error = abs(
        math.atan2(
            math.sin(final_values[2] - fitted_values[2]),
            math.cos(final_values[2] - fitted_values[2]),
        )
    )
    if position_error > 1.0e-4 or yaw_error > 1.0e-4:
        raise AutomationError(
            "fitted path is stale relative to the active route; regenerate it"
        )
    return {
        "frame_id": str(route.get("frame_id", "map")),
        "source_waypoint_count": len(waypoints),
        "seq35_pose": {
            "x": final_values[0],
            "y": final_values[1],
            "yaw": final_values[2],
        },
    }


def load_final_pass_radius(navigation_path):
    try:
        payload = yaml.safe_load(
            navigation_path.read_text(encoding="utf-8")
        )
        raw_radius = payload["navigation"]["final_pass_radius"]
    except (OSError, KeyError, TypeError, yaml.YAMLError) as exc:
        raise AutomationError(
            "cannot load navigation/final_pass_radius: {}".format(exc)
        )
    radius = finite_float(raw_radius, "navigation/final_pass_radius")
    if radius <= 0.0:
        raise AutomationError(
            "navigation/final_pass_radius must be positive for this test"
        )
    return radius


def validate_args(args):
    if args.rounds <= 0:
        raise AutomationError("--rounds must be positive")
    for name in ("ros_master_port", "gazebo_master_port"):
        value = getattr(args, name)
        if not 1 <= value <= 65535:
            raise AutomationError(
                "--{} must be between 1 and 65535".format(name.replace("_", "-"))
            )
    if args.ros_master_port == args.gazebo_master_port:
        raise AutomationError("ROS and Gazebo master ports must differ")
    for name in (
        "startup_timeout",
        "navigation_timeout",
        "gazebo_recording_ready_timeout",
    ):
        if getattr(args, name) <= 0.0:
            raise AutomationError("--{} must be positive".format(name.replace("_", "-")))
    for name in ("startup_settle", "restart_settle"):
        if getattr(args, name) < 0.0:
            raise AutomationError(
                "--{} must be non-negative".format(name.replace("_", "-"))
            )
    args.route_config = args.route_config.expanduser().resolve()
    args.fitted_path_config = args.fitted_path_config.expanduser().resolve()
    args.navigation_config = args.navigation_config.expanduser().resolve()
    for path in (
        SETUP_FILE,
        CLIENT_SCRIPT,
        GAZEBO_RECORDER_SCRIPT,
        args.route_config,
        args.fitted_path_config,
        args.navigation_config,
    ):
        if not path.is_file():
            raise AutomationError("missing required file {}".format(path))
    formal_final_pass_radius = load_final_pass_radius(args.navigation_config)
    if args.seq35_position_tolerance is None:
        args.seq35_position_tolerance = formal_final_pass_radius
    else:
        args.seq35_position_tolerance = finite_float(
            args.seq35_position_tolerance, "--seq35-position-tolerance"
        )
        if args.seq35_position_tolerance <= 0.0:
            raise AutomationError(
                "--seq35-position-tolerance must be positive"
            )
    route_metadata = validate_route_pair(
        args.route_config, args.fitted_path_config
    )
    route_metadata["formal_final_pass_radius_m"] = formal_final_pass_radius
    return route_metadata


def master_uris(args):
    return {
        "ROS_MASTER_URI": "http://127.0.0.1:{}".format(args.ros_master_port),
        "GAZEBO_MASTER_URI": "http://127.0.0.1:{}".format(
            args.gazebo_master_port
        ),
        "ROS_IP": "127.0.0.1",
    }


def configure_master_environment(args, ros_log_dir=None):
    values = master_uris(args)
    os.environ.update(values)
    if ros_log_dir is not None:
        ros_log_dir.mkdir(parents=True, exist_ok=True)
        os.environ["ROS_LOG_DIR"] = str(ros_log_dir)
    return values


def tcp_listener_exists(port, timeout=0.25):
    try:
        with socket.create_connection(("127.0.0.1", int(port)), timeout=timeout):
            return True
    except OSError:
        return False


def ensure_isolated_ports_free(args):
    occupied = [
        port
        for port in (args.ros_master_port, args.gazebo_master_port)
        if tcp_listener_exists(port)
    ]
    if occupied:
        raise CleanupError(
            "isolated master port(s) already have a listener: {}".format(
                ", ".join(str(port) for port in occupied)
            )
        )


def running_gazebo_server_pids():
    """Return live gzserver PIDs, regardless of their Gazebo master port."""
    result = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            command = (entry / "cmdline").read_bytes().split(b"\0")
            state = (entry / "stat").read_text(encoding="utf-8").split(") ", 1)[1][0]
        except (FileNotFoundError, PermissionError, ProcessLookupError, IndexError):
            continue
        if state == "Z":
            continue
        executables = {
            Path(value.decode("utf-8", errors="replace")).name
            for value in command
            if value
        }
        if any(name == "gzserver" or name.startswith("gzserver-") for name in executables):
            result.append(int(entry.name))
    return sorted(result)


def ensure_no_concurrent_gazebo():
    pids = running_gazebo_server_pids()
    if pids:
        raise CleanupError(
            "another gzserver is already running (PID{} {}); stop it before "
            "starting the pre-navigation stability test".format(
                "s" if len(pids) != 1 else "", ", ".join(map(str, pids))
            )
        )


def wait_for_isolated_ports_free(args, timeout=10.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if not any(
            tcp_listener_exists(port)
            for port in (args.ros_master_port, args.gazebo_master_port)
        ):
            return
        time.sleep(0.2)
    ensure_isolated_ports_free(args)


def parse_client_result(output):
    matches = [
        line[len(RESULT_MARKER) :]
        for line in output.splitlines()
        if line.startswith(RESULT_MARKER)
    ]
    if not matches:
        raise AutomationError("navigation client did not emit {}".format(RESULT_MARKER))
    try:
        payload = json.loads(matches[-1])
    except ValueError as exc:
        raise AutomationError("navigation client result is invalid JSON") from exc
    if not isinstance(payload, dict) or "success" not in payload:
        raise AutomationError("navigation client result is incomplete")
    return payload


def new_record(args, round_number, task_id, round_dir, recording_round_dir):
    return {
        "round": round_number,
        "task_id": task_id,
        "success": False,
        "status": "not_started",
        "error_code": None,
        "server_error_code": None,
        "message": "",
        "acceptance_mode": "none",
        "yaw_alignment_required": False,
        "completed_waypoints": 0,
        "waypoint_count": 0,
        "final_position_error_m": None,
        "final_position_tolerance_m": args.seq35_position_tolerance,
        "final_yaw_error_rad": None,
        "final_x": None,
        "final_y": None,
        "final_yaw": None,
        "gazebo_recording_status": "not_started",
        "gazebo_recording_path": str(recording_round_dir / "gazebo_world_state.log"),
        "gazebo_recording_size_bytes": 0,
        "gazebo_recording_manifest": str(
            recording_round_dir / "gazebo_world_recording.json"
        ),
        "ros_master_port": args.ros_master_port,
        "gazebo_master_port": args.gazebo_master_port,
        "duration_seconds": 0.0,
        "started_at": dt.datetime.now().astimezone().isoformat(timespec="seconds"),
        "round_dir": str(round_dir),
        "log_file": str(round_dir / "roslaunch.log"),
    }


def apply_client_result(record, payload, return_code):
    for key in (
        "status",
        "error_code",
        "server_error_code",
        "message",
        "acceptance_mode",
        "yaw_alignment_required",
        "completed_waypoints",
        "waypoint_count",
        "final_position_error_m",
        "final_position_tolerance_m",
        "final_yaw_error_rad",
    ):
        if key in payload:
            record[key] = payload[key]
    pose = payload.get("final_pose")
    if isinstance(pose, dict):
        record["final_x"] = pose.get("x")
        record["final_y"] = pose.get("y")
        record["final_yaw"] = pose.get("yaw")
    tolerance = record["final_position_tolerance_m"]
    error = record["final_position_error_m"]
    position_accepted = (
        isinstance(error, (int, float))
        and isinstance(tolerance, (int, float))
        and error <= tolerance + 1.0e-9
    )
    record["success"] = bool(
        return_code == 0
        and payload.get("success") is True
        and record["status"] == "navigation_completed"
        and record["error_code"] == 0
        and record["yaw_alignment_required"] is False
        and position_accepted
    )
    if not record["success"] and record["status"] == "navigation_completed":
        record["status"] = "navigation_client_validation_failed"


def run_trial(args, round_number, output_dir, recording_run_dir):
    round_dir = output_dir / "round_{:03d}".format(round_number)
    round_dir.mkdir(parents=True, exist_ok=False)
    recording_round_dir = recording_run_dir / "round_{:03d}".format(round_number)
    task_id = "pre_nav_{:03d}_{}".format(round_number, int(time.time()))
    record = new_record(args, round_number, task_id, round_dir, recording_round_dir)
    launch_process = None
    launch_run_id = None
    recorder_process = None
    recorder_log = None
    recorder_ready = None
    phase = "startup"
    started = time.monotonic()

    with Path(record["log_file"]).open("w", encoding="utf-8") as log_file:
        try:
            if master_is_running():
                raise CleanupError(
                    "isolated ROS master is still running before round {}".format(
                        round_number
                    )
                )
            ensure_no_concurrent_gazebo()
            ensure_isolated_ports_free(args)
            launch_process = launch_simulation(
                args.gui,
                log_file,
                goal_config=args.route_config,
                fitted_path_config=args.fitted_path_config,
                start_perception=False,
                navigation_config=args.navigation_config,
            )
            deadline = time.monotonic() + args.startup_timeout
            wait_for_ros("isolated ROS master", ["rosnode", "list"], deadline)
            launch_run_id = read_ros_run_id()
            wait_for_simulation(args.startup_timeout)
            wait_for_ros(
                "move_base action server",
                ["rostopic", "echo", "-n", "1", "/move_base/status", "--noarr"],
                deadline,
            )
            if launch_process.poll() is not None:
                raise AutomationError("roslaunch exited during startup")
            if args.startup_settle:
                time.sleep(args.startup_settle)

            # The isolated client calls NavigationAction directly and bypasses
            # MissionServer's early-mission lock. Mirror the formal seq1--35
            # policy before sending any route goal.
            phase = "baseline_lock"
            return_code, lock_output = run_owned(
                [
                    "rosservice",
                    "call",
                    BASELINE_LOCK_SERVICE,
                    "data: true",
                ],
                timeout=10.0,
            )
            append_section(log_file, "pre-navigation baseline lock", lock_output)
            if return_code != 0 or "success: True" not in lock_output:
                raise AutomationError(
                    "adaptive TEB did not confirm the pre-navigation "
                    "baseline lock"
                )

            phase = "gazebo_recording_startup"
            recorder_process, recorder_log, recorder_ready = (
                start_gazebo_world_recorder(
                    args,
                    record,
                    recording_round_dir,
                    start_stage=None,
                    start_immediately=True,
                )
            )

            phase = "navigation"
            try:
                return_code, output = run_owned(
                    [
                        "python3",
                        str(CLIENT_SCRIPT),
                        "--route",
                        str(args.route_config),
                        "--request-id",
                        task_id,
                        "--navigation-timeout",
                        str(args.navigation_timeout),
                        "--final-position-tolerance",
                        str(args.seq35_position_tolerance),
                    ],
                    timeout=args.navigation_timeout + 30.0,
                )
            except AutomationError as exc:
                record["status"] = "navigation_timeout"
                record["error_code"] = 6
                record["message"] = str(exc)
                append_section(log_file, "navigation client timeout", str(exc))
            else:
                append_section(log_file, "pre-navigation client", output)
                payload = parse_client_result(output)
                apply_client_result(record, payload, return_code)
        except CleanupError:
            raise
        except AutomationError as exc:
            record["status"] = "{}_error".format(phase)
            record["message"] = str(exc)
            append_section(log_file, "automation error", str(exc))
        finally:
            try:
                stop_process_group(recorder_process, interrupt_timeout=12.0)
                if recorder_log is not None:
                    recorder_log.close()
                if recorder_ready is not None:
                    try:
                        recorder_ready.unlink()
                    except FileNotFoundError:
                        pass
            finally:
                try:
                    stop_owned_launch(launch_process, launch_run_id)
                finally:
                    record["duration_seconds"] = round(
                        time.monotonic() - started, 3
                    )

    add_gazebo_recording_result(record)
    return record


def make_summary(results, requested_rounds):
    successes = sum(record["success"] is True for record in results)
    return {
        "requested_rounds": requested_rounds,
        "completed_rounds": len(results),
        "navigation_successes": successes,
        "navigation_failures": len(results) - successes,
        "success_rate": round(successes / len(results), 4) if results else None,
        "position_tolerance_acceptances": sum(
            record["acceptance_mode"] == "position_tolerance" for record in results
        ),
        "gazebo_recording_complete_rounds": sum(
            record["gazebo_recording_status"] == "complete" for record in results
        ),
        "infrastructure_errors": sum(
            str(record["status"]).endswith("_error")
            or record["gazebo_recording_status"] != "complete"
            for record in results
        ),
    }


def write_reports(output_dir, results, args, metadata):
    def write_csv(path):
        with path.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS)
            writer.writeheader()
            writer.writerows(results)

    csv_path = output_dir / "trials.csv"
    write_csv(csv_path)
    summary = make_summary(results, args.rounds)
    report_path = output_dir / "summary.json"
    report_path.write_text(
        json.dumps(
            {"metadata": metadata, "summary": summary, "trials": results},
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    recording_run_dir = Path(metadata["recording_run_dir"])
    write_csv(recording_run_dir / "trials.csv")
    (recording_run_dir / "summary.json").write_text(
        report_path.read_text(encoding="utf-8"), encoding="utf-8"
    )
    return csv_path, report_path, summary


def print_summary(summary, csv_path, report_path):
    rate = summary["success_rate"]
    print("\n===== pre-navigation seq35 trial summary =====")
    print(
        "completed={}/{} successes={} failures={} success_rate={}".format(
            summary["completed_rounds"],
            summary["requested_rounds"],
            summary["navigation_successes"],
            summary["navigation_failures"],
            "n/a" if rate is None else "{:.1%}".format(rate),
        )
    )
    print(
        "position_tolerance_acceptances={} recordings_complete={} infrastructure_errors={}".format(
            summary["position_tolerance_acceptances"],
            summary["gazebo_recording_complete_rounds"],
            summary["infrastructure_errors"],
        )
    )
    print("csv={}".format(csv_path))
    print("json={}".format(report_path))


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    results = []
    output_dir = None
    metadata = None
    try:
        route_metadata = validate_args(args)
        uris = master_uris(args)
        if args.dry_run:
            print("rounds={}".format(args.rounds))
            print("ROS_MASTER_URI={}".format(uris["ROS_MASTER_URI"]))
            print("GAZEBO_MASTER_URI={}".format(uris["GAZEBO_MASTER_URI"]))
            print("headless={}".format(not args.gui))
            print("seq35_position_tolerance={:.3f}".format(args.seq35_position_tolerance))
            print("route_config={}".format(args.route_config))
            print("fitted_path_config={}".format(args.fitted_path_config))
            print("navigation_config={}".format(args.navigation_config))
            print("recording_root={}".format(args.recording_root.expanduser().resolve()))
            return 0

        timestamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        run_name = "pre_navigation_trials_{}_ros{}".format(
            timestamp, args.ros_master_port
        )
        output_dir = (
            args.log_dir.expanduser().resolve()
            if args.log_dir is not None
            else WORKSPACE / "script" / "logs" / run_name
        )
        ensure_isolated_ports_free(args)
        ensure_no_concurrent_gazebo()
        output_dir.mkdir(parents=True, exist_ok=False)
        recording_run_dir = args.recording_root.expanduser().resolve() / run_name
        recording_run_dir.mkdir(parents=True, exist_ok=False)
        uris = configure_master_environment(args, output_dir / "ros_logs")

        metadata = {
            "created_at": dt.datetime.now().astimezone().isoformat(timespec="seconds"),
            "run_name": run_name,
            "rounds": args.rounds,
            "headless": not args.gui,
            "ros_master_uri": uris["ROS_MASTER_URI"],
            "gazebo_master_uri": uris["GAZEBO_MASTER_URI"],
            "route_config": str(args.route_config),
            "fitted_path_config": str(args.fitted_path_config),
            "navigation_config": str(args.navigation_config),
            "route": route_metadata,
            "seq35_position_tolerance_m": args.seq35_position_tolerance,
            "yaw_alignment_required": False,
            "recording_root": str(args.recording_root.expanduser().resolve()),
            "recording_run_dir": str(recording_run_dir),
            "recording_start": "immediately before navigation client",
            "baseline_lock_service": BASELINE_LOCK_SERVICE,
        }
        print("workspace={}".format(WORKSPACE))
        print("rounds={} logs={}".format(args.rounds, output_dir))
        print("ROS_MASTER_URI={}".format(uris["ROS_MASTER_URI"]))
        print("GAZEBO_MASTER_URI={}".format(uris["GAZEBO_MASTER_URI"]))
        print("gazebo_recording={}".format(recording_run_dir))
        print(
            "seq35_acceptance=XY <= {:.3f}m; yaw alignment disabled".format(
                args.seq35_position_tolerance
            )
        )

        for round_number in range(1, args.rounds + 1):
            print(
                "[round {}/{}] isolated pre-navigation to seq35".format(
                    round_number, args.rounds
                ),
                flush=True,
            )
            record = run_trial(args, round_number, output_dir, recording_run_dir)
            results.append(record)
            csv_path, report_path, _summary = write_reports(
                output_dir, results, args, metadata
            )
            print(
                "  status={} success={} acceptance={} final_error={} recording={} duration={:.1f}s".format(
                    record["status"],
                    record["success"],
                    record["acceptance_mode"],
                    (
                        "n/a"
                        if record["final_position_error_m"] is None
                        else "{:.3f}m".format(record["final_position_error_m"])
                    ),
                    record["gazebo_recording_status"],
                    record["duration_seconds"],
                ),
                flush=True,
            )
            if round_number < args.rounds:
                wait_for_isolated_ports_free(args)
                if args.restart_settle:
                    time.sleep(args.restart_settle)

        csv_path, report_path, summary = write_reports(
            output_dir, results, args, metadata
        )
        print_summary(summary, csv_path, report_path)
        return 0
    except KeyboardInterrupt:
        print("interrupted by user", file=sys.stderr)
        if output_dir is not None and metadata is not None:
            csv_path, report_path, summary = write_reports(
                output_dir, results, args, metadata
            )
            print_summary(summary, csv_path, report_path)
        return 130
    except (AutomationError, CleanupError, OSError, subprocess.SubprocessError) as exc:
        print("run_pre_navigation_trials: {}".format(exc), file=sys.stderr)
        if output_dir is not None and metadata is not None:
            csv_path, report_path, summary = write_reports(
                output_dir, results, args, metadata
            )
            print_summary(summary, csv_path, report_path)
        return 1


if __name__ == "__main__":
    sys.exit(main())
