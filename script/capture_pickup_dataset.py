#!/usr/bin/env python3
"""Restart Gazebo, navigate to the active observation yaw, and capture images."""

import argparse
import datetime as dt
import importlib.util
import math
import os
from pathlib import Path
import re
import shlex
import signal
import subprocess
import sys
import time


WORKSPACE = Path(__file__).resolve().parents[1]
SETUP_FILE = WORKSPACE / "devel" / "setup.bash"
DEFAULT_LOG_ROOT = WORKSPACE / "script" / "logs"
DEFAULT_ROUTE = (
    WORKSPACE / "src" / "smart_factory_mission" / "config" / "pickup_staging_dev.yaml"
)
DEFAULT_MISSION = (
    WORKSPACE / "src" / "smart_factory_mission" / "config" / "mission.yaml"
)
PATH_FITTER = WORKSPACE / "src" / "gazebo_map" / "scripts" / "plot_fitted_navigation_path.py"
INDEX_PATTERN = re.compile(r"%(?:0\d+)?[di]")
OBSERVATION_POSES = {
    "close_navi": (-1.43569836894096, -0.49503905352630406, -0.05199587614654322),
    "mid": (-1.43569836894096, -0.49503905352630406, 1.50013918066071),
    "far_navi": (-1.43569836894096, -0.49503905352630406, 3.054681877745692),
}
OUTPUT_PREFIXES = {
    "close_navi": "close",
    "mid": "mid",
    "far_navi": "far",
}
# Camera observation pose recorded in
# docs/物品抓取坐标和机械臂记录.md.  Do not use the lower grasp pose here:
# it points the arm-mounted camera at a near-uniform gray surface.
DEFAULT_ARM_SCAN_POSITIONS = (0.0, 0.0, 0.55, 2.2, 0.0)


class AutomationError(RuntimeError):
    pass


class CleanupError(AutomationError):
    """The isolated ROS/Gazebo launch could not be fully removed."""


def ros_command(arguments):
    command = "source {} && exec {}".format(
        shlex.quote(str(SETUP_FILE)), shlex.join([str(arg) for arg in arguments])
    )
    return ["bash", "-lc", command]


def stop_process_group(process, interrupt_timeout=12.0):
    if process is None or process.poll() is not None:
        return
    for sig, timeout in (
        (signal.SIGINT, interrupt_timeout),
        (signal.SIGTERM, 5.0),
        (signal.SIGKILL, 2.0),
    ):
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            return
        try:
            process.wait(timeout=timeout)
            return
        except subprocess.TimeoutExpired:
            continue


def _process_info(pid):
    """Return (start_ticks, parent_pid, state) for one Linux process."""
    try:
        raw = (Path("/proc") / str(pid) / "stat").read_text(encoding="utf-8")
    except (FileNotFoundError, PermissionError, ProcessLookupError):
        return None
    closing = raw.rfind(")")
    if closing < 0:
        return None
    fields = raw[closing + 2 :].split()
    if len(fields) <= 19:
        return None
    try:
        return fields[19], int(fields[1]), fields[0]
    except ValueError:
        return None


def _process_identity(pid):
    info = _process_info(pid)
    return None if info is None else (int(pid), info[0])


def _identity_is_live(identity):
    pid, start_ticks = identity
    info = _process_info(pid)
    return info is not None and info[0] == start_ticks and info[2] != "Z"


def _process_command(pid):
    try:
        raw = (Path("/proc") / str(pid) / "cmdline").read_bytes()
    except (FileNotFoundError, PermissionError, ProcessLookupError):
        return ""
    return raw.replace(b"\0", b" ").decode("utf-8", errors="replace")


def _descendant_identities(root_pid):
    children = {}
    identities = {}
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        pid = int(entry.name)
        info = _process_info(pid)
        if info is None:
            continue
        start_ticks, parent_pid, _state = info
        identities[pid] = (pid, start_ticks)
        children.setdefault(parent_pid, []).append(pid)

    result = set()
    pending = [int(root_pid)]
    while pending:
        parent = pending.pop()
        for child in children.get(parent, ()):
            if child in pending or identities[child] in result:
                continue
            result.add(identities[child])
            pending.append(child)
    return result


def _run_id_process_identities(run_id):
    if not run_id:
        return set()
    marker = "/.ros/log/{}/".format(run_id)
    result = set()
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        pid = int(entry.name)
        if marker not in _process_command(pid):
            continue
        identity = _process_identity(pid)
        if identity is not None:
            result.add(identity)
    return result


def _wait_for_identities(identities, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        remaining = {identity for identity in identities if _identity_is_live(identity)}
        if not remaining:
            return set()
        time.sleep(0.1)
    return {identity for identity in identities if _identity_is_live(identity)}


def _signal_identities(identities, sig):
    for pid, _start_ticks in sorted(identities, reverse=True):
        if pid == os.getpid() or not _identity_is_live((pid, _start_ticks)):
            continue
        try:
            os.kill(pid, sig)
        except (PermissionError, ProcessLookupError):
            continue


def read_ros_run_id():
    return_code, output = run_owned(["rosparam", "get", "/run_id"], timeout=3.0)
    if return_code != 0 or not output.strip():
        raise AutomationError("ROS master did not provide /run_id")
    return output.strip().strip("'\"")


def stop_owned_launch(process, run_id):
    """Stop roslaunch and every process carrying its unique ROS run_id."""
    if process is None:
        return

    identities = _descendant_identities(process.pid)
    root_identity = _process_identity(process.pid)
    if root_identity is not None:
        identities.add(root_identity)
    identities.update(_run_id_process_identities(run_id))

    if process.poll() is None:
        try:
            os.killpg(process.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=12.0)
        except subprocess.TimeoutExpired:
            pass

    # roslaunch children create their own sessions. Re-scan by run_id after
    # graceful shutdown so re-parented controller/Gazebo processes stay owned.
    identities.update(_descendant_identities(process.pid))
    identities.update(_run_id_process_identities(run_id))
    remaining = _wait_for_identities(identities, 0.5)
    for sig, timeout in ((signal.SIGTERM, 5.0), (signal.SIGKILL, 2.0)):
        if not remaining:
            break
        _signal_identities(remaining, sig)
        remaining = _wait_for_identities(remaining, timeout)

    if remaining:
        raise CleanupError(
            "failed to stop owned ROS processes: {}".format(
                ", ".join(str(pid) for pid, _start in sorted(remaining))
            )
        )

    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        if not master_is_running(timeout=1.0):
            return
        time.sleep(0.2)
    raise CleanupError(
        "ROS master remained available after stopping run_id {}".format(
            run_id or "unknown"
        )
    )


def run_owned(arguments, timeout, stdout=subprocess.PIPE):
    process = subprocess.Popen(
        ros_command(arguments),
        cwd=str(WORKSPACE),
        stdout=stdout,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )
    try:
        output, _ = process.communicate(timeout=timeout)
    except subprocess.TimeoutExpired:
        stop_process_group(process, interrupt_timeout=2.0)
        raise AutomationError(
            "command timed out after {:.1f}s: {}".format(
                timeout, shlex.join([str(arg) for arg in arguments])
            )
        )
    return process.returncode, output or ""


def master_is_running(timeout=3.0):
    try:
        return run_owned(["rosnode", "list"], timeout=timeout)[0] == 0
    except AutomationError:
        return False


def wait_for_ros(label, arguments, deadline, required_text=None):
    last_output = ""
    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        try:
            return_code, output = run_owned(
                arguments, timeout=max(0.5, min(5.0, remaining))
            )
            last_output = output.strip()
            if return_code == 0 and (
                required_text is None or required_text in output
            ):
                return
        except AutomationError as exc:
            last_output = str(exc)
        time.sleep(0.5)
    detail = last_output[-500:] if last_output else "no response"
    raise AutomationError("waiting for {} timed out: {}".format(label, detail))


def wait_for_simulation(timeout):
    deadline = time.monotonic() + timeout
    wait_for_ros("ROS master", ["rosnode", "list"], deadline)
    wait_for_ros(
        "Gazebo car3 model",
        ["rosservice", "call", "/gazebo/get_model_state", "{model_name: car3}"],
        deadline,
        required_text="success: True",
    )
    for cube_index in range(3):
        wait_for_ros(
            "cube_{}".format(cube_index),
            [
                "rosservice",
                "call",
                "/gazebo/get_model_state",
                "{model_name: cube_%d}" % cube_index,
            ],
            deadline,
            required_text="success: True",
        )
    wait_for_ros(
        "Gazebo clock",
        ["rostopic", "echo", "-n", "1", "/clock", "--noarr"],
        deadline,
    )
    wait_for_ros(
        "RGB camera frame",
        [
            "rostopic",
            "echo",
            "-n",
            "1",
            "/camera/rgb/image_raw",
            "--noarr",
        ],
        deadline,
    )
    wait_for_ros(
        "AMCL pose",
        ["rostopic", "echo", "-n", "1", "/amcl_pose", "--noarr"],
        deadline,
    )
    wait_for_ros(
        "mission server", ["rosnode", "info", "/smart_factory_mission"], deadline
    )


def wait_for_capture_health(timeout):
    """Re-check live simulation and camera state immediately before capture."""
    wait_for_simulation(timeout)
    deadline = time.monotonic() + timeout
    wait_for_ros(
        "running arm controller",
        ["rosservice", "call", "/controller_manager/list_controllers", "{}"],
        deadline,
        required_text="arm_controller",
    )


def matching_image_count(filename_format):
    matches = INDEX_PATTERN.findall(filename_format)
    if len(matches) != 1:
        raise AutomationError(
            "--output-format must contain exactly one integer placeholder, "
            "for example mid_auto_%04i.png"
        )
    path = Path(filename_format).expanduser().resolve()
    placeholder = INDEX_PATTERN.search(path.name)
    width_match = re.fullmatch(r"%0(\d+)[di]", placeholder.group(0))
    if width_match is None:
        index_expression = r"\d+"
    else:
        index_expression = r"\d{{{}}}".format(int(width_match.group(1)))
    filename_pattern = re.compile(
        "^{}{}{}$".format(
            re.escape(path.name[: placeholder.start()]),
            index_expression,
            re.escape(path.name[placeholder.end() :]),
        )
    )
    if not path.parent.is_dir():
        return 0
    return len(
        [
            candidate
            for candidate in path.parent.iterdir()
            if candidate.is_file() and filename_pattern.fullmatch(candidate.name)
        ]
    )


def arm_scan_message(positions):
    values = ", ".join(repr(float(value)) for value in positions)
    return (
        "{joint_names: [arm_joint1, arm_joint2, arm_joint3, arm_joint4, arm_joint5], "
        "points: [{positions: [" + values + "], "
        "time_from_start: {secs: 3, nsecs: 0}}]}"
    )


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description=(
            "repeat fresh Gazebo startup -> pickup-route navigation -> one RGB "
            "capture until the output pattern contains the requested image count"
        )
    )
    parser.add_argument("--count", type=int, default=40, help="target total images")
    parser.add_argument(
        "--view",
        choices=tuple(OBSERVATION_POSES),
        default="mid",
        help="pickup region to face from the common observation point",
    )
    parser.add_argument(
        "--output-format",
        default=None,
        help="numbered PNG/JPEG output format",
    )
    parser.add_argument(
        "--target-class",
        choices=("food", "daily", "electronics"),
        default="food",
        help="navigation-only task field; it does not affect the current route",
    )
    parser.add_argument("--gui", action="store_true", help="show Gazebo GUI")
    parser.add_argument(
        "--startup-timeout", type=float, default=90.0, help="ROS readiness timeout"
    )
    parser.add_argument(
        "--startup-settle",
        type=float,
        default=8.0,
        help="extra wall-clock settling time after ROS readiness",
    )
    parser.add_argument(
        "--navigation-timeout",
        type=float,
        default=300.0,
        help="timeout for one navigation attempt",
    )
    parser.add_argument(
        "--view-alignment-timeout",
        type=float,
        default=30.0,
        help="timeout for the final same-position move_base yaw goal",
    )
    parser.add_argument(
        "--route-end-seq",
        type=int,
        help=(
            "generate an isolated goal/path configuration ending at this "
            "active route sequence"
        ),
    )
    parser.add_argument(
        "--route-end-pose",
        type=float,
        nargs=3,
        metavar=("X", "Y", "YAW"),
        help=(
            "use this private x/y/yaw pose for --route-end-seq instead of "
            "requiring that sequence to be active in the shared route file"
        ),
    )
    parser.add_argument(
        "--skip-view-alignment",
        action="store_true",
        help="use the final route pose directly instead of sending another move_base goal",
    )
    parser.add_argument(
        "--navigation-only",
        action="store_true",
        help=(
            "stop the mission at the route endpoint and disable perception; "
            "the image is captured without OCR or class selection"
        ),
    )
    parser.add_argument(
        "--photo-settle",
        type=float,
        default=1.0,
        help="settling time after navigation success before capture",
    )
    parser.add_argument(
        "--skip-arm-pose",
        action="store_true",
        help="do not command the recorded camera scan pose before capture",
    )
    parser.add_argument(
        "--arm-scan-positions",
        type=float,
        nargs=5,
        default=DEFAULT_ARM_SCAN_POSITIONS,
        metavar=("J1", "J2", "J3", "J4", "J5"),
        help="five arm joint positions used for the camera observation pose",
    )
    parser.add_argument(
        "--restart-settle",
        type=float,
        default=3.0,
        help="delay after shutting down one simulation",
    )
    parser.add_argument(
        "--max-attempts",
        type=int,
        default=80,
        help="stop with failure after this many launches",
    )
    parser.add_argument(
        "--log-dir", default=str(DEFAULT_LOG_ROOT), help="directory for roslaunch logs"
    )
    args = parser.parse_args(argv)
    if args.output_format is None:
        prefix = OUTPUT_PREFIXES[args.view]
        args.output_format = str(
            WORKSPACE / "data" / args.view / (prefix + "_auto_%04i.png")
        )
    return args


def validate_args(args):
    if args.count <= 0:
        raise AutomationError("--count must be positive")
    if args.route_end_seq is not None and args.route_end_seq <= 0:
        raise AutomationError("--route-end-seq must be positive")
    if args.route_end_pose is not None and args.route_end_seq is None:
        raise AutomationError("--route-end-pose requires --route-end-seq")
    if args.route_end_pose is not None and not all(
        math.isfinite(value) for value in args.route_end_pose
    ):
        raise AutomationError("--route-end-pose must contain finite values")
    if not all(math.isfinite(value) for value in args.arm_scan_positions):
        raise AutomationError("--arm-scan-positions must contain finite values")
    if args.max_attempts < args.count:
        raise AutomationError("--max-attempts must be at least --count")
    for name in (
        "startup_timeout",
        "startup_settle",
        "navigation_timeout",
        "view_alignment_timeout",
        "photo_settle",
        "restart_settle",
    ):
        value = getattr(args, name)
        if not isinstance(value, float) or not (value >= 0.0):
            raise AutomationError("--{} must be non-negative".format(name.replace("_", "-")))
    if (
        args.startup_timeout == 0.0
        or args.navigation_timeout == 0.0
        or args.view_alignment_timeout == 0.0
    ):
        raise AutomationError(
            "startup, navigation, and view-alignment timeouts must be positive"
        )
    matching_image_count(args.output_format)
    if not SETUP_FILE.is_file():
        raise AutomationError("missing {}; run catkin_make first".format(SETUP_FILE))
    if args.route_end_seq is not None:
        if not DEFAULT_ROUTE.is_file():
            raise AutomationError("missing route configuration: {}".format(DEFAULT_ROUTE))
        if not PATH_FITTER.is_file():
            raise AutomationError("missing path fitter: {}".format(PATH_FITTER))
    if args.navigation_only and not DEFAULT_MISSION.is_file():
        raise AutomationError("missing mission configuration: {}".format(DEFAULT_MISSION))


def prepare_truncated_route(end_sequence, output_dir, end_pose=None):
    """Build isolated mission inputs ending at one explicit route sequence."""
    spec = importlib.util.spec_from_file_location("pickup_path_fitter", PATH_FITTER)
    if spec is None or spec.loader is None:
        raise AutomationError("cannot load path fitter: {}".format(PATH_FITTER))
    fitter = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(fitter)
    records = fitter.read_active_route(DEFAULT_ROUTE)
    if end_pose is None:
        matching_indices = [
            index for index, record in enumerate(records) if record[0] == end_sequence
        ]
        if len(matching_indices) != 1:
            raise AutomationError(
                "active route must contain seq {} exactly once".format(end_sequence)
            )
        records = records[: matching_indices[0] + 1]
    else:
        # Keep the shared route read-only. This allows a dataset wrapper to use
        # a disabled observation point as its private terminal waypoint.
        records = [record for record in records if record[0] < end_sequence]
        records.append((end_sequence,) + tuple(end_pose))

    route_path = output_dir / "pickup_staging_to_seq_{}.yaml".format(end_sequence)
    route_lines = [
        "# Generated by capture_pickup_dataset.py; isolated dataset route.",
        "pickup_staging:",
        "  configured: true",
        "  frame_id: map",
        "  waypoints:",
    ]
    for sequence, x, y, yaw in records:
        route_lines.extend(
            [
                "    # seq {}".format(sequence),
                "    - x: {!r}".format(x),
                "      y: {!r}".format(y),
                "      yaw: {!r}".format(yaw),
            ]
        )
    route_path.write_text("\n".join(route_lines) + "\n", encoding="utf-8")

    path_path = output_dir / "fitted_path_to_seq_{}.yaml".format(end_sequence)
    preview_path = output_dir / "fitted_path_to_seq_{}.png".format(end_sequence)
    return_code, fitter_output = run_owned(
        [
            "python3",
            str(PATH_FITTER),
            "--route",
            str(route_path),
            "--path-output",
            str(path_path),
            "--output",
            str(preview_path),
        ],
        timeout=60.0,
    )
    if return_code != 0:
        raise AutomationError(
            "failed to fit route ending at seq {}: {}".format(
                end_sequence, fitter_output.strip()[-1000:]
            )
        )
    return route_path, path_path, preview_path, fitter_output


def prepare_navigation_only_mission(output_dir):
    """Copy the competition mission while stopping before OCR/manipulation."""
    source = DEFAULT_MISSION.read_text(encoding="utf-8")
    lines = source.splitlines()
    matches = [
        index
        for index, line in enumerate(lines)
        if re.match(r"^pipeline_stop_after\s*:", line)
    ]
    if len(matches) != 1:
        raise AutomationError(
            "mission configuration must define one top-level pipeline_stop_after"
        )
    lines[matches[0]] = "pipeline_stop_after: ARRIVED_PICKUP_STAGING"
    output = output_dir / "mission_navigation_only.yaml"
    output.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return output


def launch_simulation(
    gui,
    log_file,
    goal_config=None,
    fitted_path_config=None,
    mission_config=None,
    start_perception=True,
):
    command = [
        "roslaunch",
        "smart_factory_bringup",
        "full_competition.launch",
        "gazebo_gui:={}".format("true" if gui else "false"),
        "start_rviz:=false",
        "start_perception:={}".format("true" if start_perception else "false"),
    ]
    if goal_config is not None:
        command.append("goal_config:={}".format(goal_config))
    if fitted_path_config is not None:
        command.append(
            "fitted_path_config:={}".format(fitted_path_config)
        )
    if mission_config is not None:
        command.append("mission_config:={}".format(mission_config))
    return subprocess.Popen(
        ros_command(command),
        cwd=str(WORKSPACE),
        stdout=log_file,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )


def capture_one(args, attempt, log_path):
    launch_process = None
    launch_run_id = None
    with log_path.open("w", encoding="utf-8") as log_file:
        try:
            if master_is_running():
                raise CleanupError(
                    "ROS master is still running before attempt {}; refusing "
                    "to reuse a stale simulation".format(attempt)
                )
            launch_process = launch_simulation(
                args.gui,
                log_file,
                goal_config=args.generated_goal_config,
                fitted_path_config=args.generated_path_config,
                mission_config=args.generated_mission_config,
                start_perception=not args.navigation_only,
            )
            master_deadline = time.monotonic() + args.startup_timeout
            wait_for_ros("ROS master", ["rosnode", "list"], master_deadline)
            launch_run_id = read_ros_run_id()
            wait_for_simulation(args.startup_timeout)
            if launch_process.poll() is not None:
                raise AutomationError("roslaunch exited during startup")
            print("  simulation ready; settling {:.1f}s".format(args.startup_settle))
            time.sleep(args.startup_settle)

            task_id = "ocr_{}_{:03d}_{:03d}".format(
                OUTPUT_PREFIXES[args.view],
                int(time.time()) % 1000, attempt
            )
            if args.route_end_seq is None:
                destination = "configured pickup route"
            else:
                destination = "route ending at seq {}".format(args.route_end_seq)
            print("  navigating through {} as task {}".format(destination, task_id))
            return_code, navigation_output = run_owned(
                [
                    "rosrun",
                    "smart_factory_tests",
                    "send_navigation_task.py",
                    "--task-id",
                    task_id,
                    "--target-class",
                    args.target_class,
                ],
                timeout=args.navigation_timeout,
            )
            log_file.write("\n===== navigation client =====\n")
            log_file.write(navigation_output)
            log_file.flush()
            if return_code != 0:
                raise AutomationError(
                    "navigation failed with exit code {}; see {}".format(
                        return_code, log_path
                    )
                )

            if not args.skip_view_alignment:
                view_x, view_y, view_yaw = OBSERVATION_POSES[args.view]
                print(
                    "  aligning at the circumcenter to {} yaw={:.6f}".format(
                        args.view, view_yaw
                    )
                )
                return_code, alignment_output = run_owned(
                    [
                        "python3",
                        str(WORKSPACE / "script" / "_send_observation_goal.py"),
                        "--x",
                        str(view_x),
                        "--y",
                        str(view_y),
                        "--yaw",
                        str(view_yaw),
                        "--timeout",
                        str(args.view_alignment_timeout),
                    ],
                    timeout=args.view_alignment_timeout + 15.0,
                )
                log_file.write("\n===== observation yaw alignment =====\n")
                log_file.write(alignment_output)
                log_file.flush()
                if return_code != 0:
                    raise AutomationError(
                        "{} yaw alignment failed with exit code {}; see {}".format(
                            args.view, return_code, log_path
                        )
                    )
            else:
                print("  using final route pose; no extra move_base alignment goal")

            if not args.skip_arm_pose:
                print("  moving arm to the recorded camera scan pose")
                return_code, arm_output = run_owned(
                    [
                        "rostopic",
                        "pub",
                        "-1",
                        "/arm_controller/command",
                        "trajectory_msgs/JointTrajectory",
                        arm_scan_message(args.arm_scan_positions),
                    ],
                    timeout=10.0,
                )
                log_file.write("\n===== arm camera pose =====\n")
                log_file.write(arm_output)
                log_file.flush()
                if return_code != 0:
                    raise AutomationError(
                        "arm scan pose failed with exit code {}; see {}".format(
                            return_code, log_path
                        )
                    )

            print("  navigation succeeded; settling {:.1f}s".format(args.photo_settle))
            time.sleep(args.photo_settle)
            print("  rechecking Gazebo, controllers, clock, and camera")
            wait_for_capture_health(min(15.0, args.startup_timeout))
            return_code, capture_output = run_owned(
                [
                    "rosrun",
                    "smart_factory_perception",
                    "imgsave",
                    args.output_format,
                    "--timeout",
                    "10",
                    "--warmup-frames",
                    "8",
                    "--min-color-fraction",
                    "0.001",
                    "--color-delta",
                    "20",
                ],
                timeout=15.0,
            )
            log_file.write("\n===== image capture =====\n")
            log_file.write(capture_output)
            log_file.flush()
            if return_code != 0:
                raise AutomationError(
                    "image capture failed with exit code {}; see {}".format(
                        return_code, log_path
                    )
                )
            saved_lines = [line.strip() for line in capture_output.splitlines() if line.strip()]
            return saved_lines[-1] if saved_lines else "image saved"
        finally:
            stop_owned_launch(launch_process, launch_run_id)


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    try:
        validate_args(args)
        existing = matching_image_count(args.output_format)
        if existing >= args.count:
            print(
                "target already satisfied: {} matching images (requested {})".format(
                    existing, args.count
                )
            )
            return 0
        if master_is_running():
            raise AutomationError(
                "a ROS master is already running; stop the existing ROS/Gazebo "
                "session before starting this isolated restart loop"
            )

        timestamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        log_dir = Path(args.log_dir).expanduser().resolve() / timestamp
        log_dir.mkdir(parents=True, exist_ok=False)
        args.generated_goal_config = None
        args.generated_path_config = None
        args.generated_mission_config = None
        if args.route_end_seq is not None:
            (
                args.generated_goal_config,
                args.generated_path_config,
                preview_path,
                fitter_output,
            ) = prepare_truncated_route(
                args.route_end_seq,
                log_dir,
                end_pose=args.route_end_pose,
            )
            (log_dir / "path_fitter.log").write_text(fitter_output, encoding="utf-8")
            print("generated_goal_config={}".format(args.generated_goal_config))
            print("generated_path_config={}".format(args.generated_path_config))
            print("generated_path_preview={}".format(preview_path))
        if args.navigation_only:
            args.generated_mission_config = prepare_navigation_only_mission(log_dir)
            print(
                "generated_mission_config={}".format(
                    args.generated_mission_config
                )
            )
            print("perception=disabled (no OCR or class selection)")
        print("workspace={}".format(WORKSPACE))
        print("view={}".format(args.view))
        print("output_format={}".format(Path(args.output_format).expanduser()))
        print("existing={} target={} logs={}".format(existing, args.count, log_dir))

        attempt = 0
        while matching_image_count(args.output_format) < args.count:
            attempt += 1
            if attempt > args.max_attempts:
                raise AutomationError(
                    "reached --max-attempts={} before collecting {} images".format(
                        args.max_attempts, args.count
                    )
                )
            completed = matching_image_count(args.output_format)
            print(
                "[attempt {}/{}] captured {}/{}".format(
                    attempt, args.max_attempts, completed, args.count
                ),
                flush=True,
            )
            log_path = log_dir / "attempt_{:03d}.log".format(attempt)
            try:
                saved_path = capture_one(args, attempt, log_path)
                completed = matching_image_count(args.output_format)
                print("  saved {} ({}/{})".format(saved_path, completed, args.count))
            except CleanupError:
                raise
            except AutomationError as exc:
                print("  attempt failed: {}".format(exc), file=sys.stderr)
            if matching_image_count(args.output_format) < args.count:
                print("  restarting after {:.1f}s".format(args.restart_settle))
                time.sleep(args.restart_settle)

        print("completed: {} matching images".format(args.count))
        return 0
    except KeyboardInterrupt:
        print("interrupted by user", file=sys.stderr)
        return 130
    except AutomationError as exc:
        print("capture_pickup_dataset: {}".format(exc), file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
