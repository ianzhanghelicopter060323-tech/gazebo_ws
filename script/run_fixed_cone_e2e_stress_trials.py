#!/usr/bin/env python3
"""Run deterministic end-to-end trials using recorded fixed cone layouts."""

import argparse
import csv
import datetime as dt
import json
import math
from pathlib import Path
import re
import shutil
import subprocess
import sys
import threading
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
)
import run_end_to_end_cone_trials as e2e
from run_cone_move_stress_trials import load_runtime_config, parse_marker


DEFAULT_TEMPLATE_MANIFEST = (
    WORKSPACE
    / "script"
    / "config"
    / "fixed_cone_e2e_rounds_007_009_012_032.json"
)
DEFAULT_RECORDING_ROOT = (
    WORKSPACE / "data" / "cone_zone" / "end_to_end_stress"
)
SCENE_HELPER = WORKSPACE / "script" / "_setup_cone_move_stress_scene.py"
NAVIGATION_HELPER = WORKSPACE / "script" / "_run_cone_move_navigation.py"
ADAPTIVE_DIAGNOSTICS_RECORDER = (
    WORKSPACE / "script" / "record_adaptive_teb_diagnostics.py"
)
GLOBAL_COSTMAP_CONFIG = (
    WORKSPACE
    / "src"
    / "gazebo_nav"
    / "launch"
    / "config"
    / "move_base"
    / "global_costmap_params.yaml"
)
GLOBAL_PLANNER_CONFIG = (
    WORKSPACE
    / "src"
    / "gazebo_nav"
    / "launch"
    / "config"
    / "move_base"
    / "global_planner_params.yaml"
)
COSTMAP_COMMON_CONFIG = GLOBAL_COSTMAP_CONFIG.with_name(
    "costmap_common_params.yaml"
)
LOCAL_COSTMAP_CONFIG = GLOBAL_COSTMAP_CONFIG.with_name(
    "local_costmap_params.yaml"
)
MOVE_BASE_CONFIG = GLOBAL_COSTMAP_CONFIG.with_name("move_base_params.yaml")
TEB_LOCAL_PLANNER_CONFIG = GLOBAL_COSTMAP_CONFIG.with_name(
    "teb_local_planner_params.yaml"
)
ADAPTIVE_TEB_CONFIG = GLOBAL_COSTMAP_CONFIG.with_name(
    "adaptive_teb_params.yaml"
)
DELIVERY_GOALS_CONFIG = (
    WORKSPACE
    / "src"
    / "smart_factory_mission"
    / "config"
    / "delivery_goals.yaml"
)
ADAPTIVE_TEB_PLUGIN = (
    "smart_factory_adaptive_teb/AdaptiveTebLocalPlannerROS"
)
NAVIGATION_CONFIG_FILES = (
    e2e.MISSION_CONFIG,
    DELIVERY_GOALS_CONFIG,
    GLOBAL_COSTMAP_CONFIG,
    GLOBAL_PLANNER_CONFIG,
    COSTMAP_COMMON_CONFIG,
    LOCAL_COSTMAP_CONFIG,
    MOVE_BASE_CONFIG,
    TEB_LOCAL_PLANNER_CONFIG,
    ADAPTIVE_TEB_CONFIG,
)
RUNTIME_PARAMETER_NAMES = (
    "/smart_factory_navigation/navigation/goal_timeout",
    "/move_base/base_global_planner",
    "/move_base/base_local_planner",
    "/move_base/global_costmap/robot_radius",
    "/move_base/global_costmap/inflation_layer/inflation_radius",
    "/move_base/global_costmap/inflation_layer/cost_scaling_factor",
    "/move_base/local_costmap/footprint",
    "/move_base/local_costmap/footprint_padding",
    "/move_base/local_costmap/inflation_layer/inflation_radius",
    "/move_base/local_costmap/inflation_layer/cost_scaling_factor",
    "/move_base/GlobalPlanner/cost_factor",
    "/move_base/GlobalPlanner/neutral_cost",
    "/move_base/GlobalPlanner/use_dijkstra",
    "/move_base/GlobalPlanner/use_grid_path",
    "/move_base/TebLocalPlannerROS/min_obstacle_dist",
    "/move_base/TebLocalPlannerROS/inflation_dist",
    "/move_base/TebLocalPlannerROS/weight_obstacle",
    "/move_base/TebLocalPlannerROS/weight_viapoint",
    "/move_base/TebLocalPlannerROS/enable_homotopy_class_planning",
    "/move_base/TebLocalPlannerROS/max_number_classes",
    "/move_base/TebLocalPlannerROS/roadmap_graph_no_samples",
    "/move_base/TebLocalPlannerROS/roadmap_graph_area_width",
    "/move_base/TebLocalPlannerROS/selection_obst_cost_scale",
    "/move_base/TebLocalPlannerROS/selection_viapoint_cost_scale",
    "/move_base/TebLocalPlannerROS/selection_prefer_initial_plan",
    "/move_base/TebLocalPlannerROS/selection_cost_hysteresis",
    "/move_base/TebLocalPlannerROS/switching_blocking_period",
    "/move_base/TebLocalPlannerROS/viapoints_all_candidates",
    "/move_base/TebLocalPlannerROS/footprint_model/type",
    "/move_base/TebLocalPlannerROS/footprint_model/vertices",
    # Capture the complete selector and both immutable child profiles in one
    # document so adaptive runs remain exactly reproducible.
    "/move_base/AdaptiveTebLocalPlannerROS",
)
EXPECTED_SOURCE_ROUNDS = (7, 9, 12, 32)
DEFAULT_SOURCE_ROUNDS = EXPECTED_SOURCE_ROUNDS
CSV_FIELDS = (
    e2e.CSV_FIELDS[:1]
    + (
        "scenario_id",
        "source_round",
        "repetition",
        "attempt",
        "infrastructure_retry_count",
    )
    + e2e.CSV_FIELDS[1:]
    + (
        "case_file",
        "scene_measurement_file",
        "navigation_parameter_file",
        "adaptive_monitor_file",
        "adaptive_monitor_status",
        "adaptive_mode_event_count",
        "adaptive_diagnostic_sample_count",
        "adaptive_avoidance_entries",
        "adaptive_avoidance_used",
        "adaptive_avoidance_command_samples",
        "adaptive_avoidance_failure_recovery_samples",
        "adaptive_avoidance_infeasible_samples",
        "adaptive_baseline_fallback_events",
        "adaptive_baseline_fallback_samples",
        "adaptive_baseline_infeasible_samples",
        "adaptive_both_planners_infeasible_samples",
        "adaptive_minimum_plan_clearance_m",
    )
)


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description=(
            "run complete pickup-and-delivery tasks with one or more recorded "
            "fixed cone layouts"
        )
    )
    parser.add_argument(
        "--rounds", type=int, default=20, help="number of trials (default: 20)"
    )
    parser.add_argument(
        "--templates", type=Path, default=DEFAULT_TEMPLATE_MANIFEST
    )
    parser.add_argument(
        "--source-round",
        type=int,
        action="append",
        dest="source_rounds",
        help=(
            "override the default source rounds 7, 9, 12, and 32; may be "
            "repeated for "
            "multiple round-robin scenarios"
        ),
    )
    display = parser.add_mutually_exclusive_group()
    display.add_argument(
        "--gui",
        dest="gui",
        action="store_true",
        help="start the Gazebo client window (gzclient)",
    )
    display.add_argument(
        "--headless",
        dest="gui",
        action="store_false",
        help="run Gazebo without the client window (default)",
    )
    parser.set_defaults(gui=False)
    parser.add_argument(
        "--startup-timeout",
        type=float,
        default=180.0,
        help=(
            "wall-clock deadline for each startup dependency; GUI startup can "
            "be substantially slower (default: 180)"
        ),
    )
    parser.add_argument("--startup-settle", type=float, default=8.0)
    parser.add_argument("--navigation-ready-timeout", type=float, default=60.0)
    parser.add_argument(
        "--startup-retries",
        type=int,
        default=2,
        help=(
            "relaunch the same logical round after infrastructure startup "
            "failure (default: 2 retries)"
        ),
    )
    parser.add_argument("--scene-timeout", type=float, default=30.0)
    parser.add_argument("--task-timeout", type=float, default=480.0)
    parser.add_argument(
        "--task-progress-timeout",
        type=float,
        default=360.0,
        help=(
            "cancel a task after this many wall-clock seconds without a new "
            "mission stage/detail (default: 360, six minutes)"
        ),
    )
    parser.add_argument(
        "--status-interval",
        type=float,
        default=30.0,
        help="print the current task stage at this wall-clock interval",
    )
    parser.add_argument("--monitor-ready-timeout", type=float, default=15.0)
    parser.add_argument(
        "--adaptive-monitor-ready-timeout", type=float, default=15.0
    )
    parser.add_argument(
        "--gazebo-recording-ready-timeout", type=float, default=15.0
    )
    parser.add_argument("--restart-settle", type=float, default=3.0)
    parser.add_argument("--cone-motion-threshold", type=float, default=0.01)
    parser.add_argument("--cone-tilt-threshold-deg", type=float, default=5.0)
    parser.add_argument("--disable-contact-stream", action="store_true")
    parser.add_argument(
        "--recording-root", type=Path, default=DEFAULT_RECORDING_ROOT
    )
    parser.add_argument("--disable-gazebo-recording", action="store_true")
    parser.add_argument("--log-dir", type=Path)
    parser.add_argument(
        "--experiment-label",
        default="",
        help=(
            "optional filesystem-safe label appended to the run directory and "
            "stored in summary metadata"
        ),
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="validate configuration and print the deterministic schedule only",
    )
    return parser.parse_args(argv)


def finite(value, label):
    try:
        parsed = float(value)
    except (TypeError, ValueError) as exc:
        raise AutomationError("{} must be numeric".format(label)) from exc
    if not math.isfinite(parsed):
        raise AutomationError("{} must be finite".format(label))
    return parsed


def validate_pose(pose, label):
    if not isinstance(pose, dict):
        raise AutomationError("{} must be a mapping".format(label))
    position = pose.get("position")
    orientation = pose.get("orientation")
    if not isinstance(position, dict) or not isinstance(orientation, dict):
        raise AutomationError(
            "{} must contain position and orientation".format(label)
        )
    for axis in ("x", "y", "z"):
        finite(position.get(axis), "{}.position.{}".format(label, axis))
    quaternion = [
        finite(orientation.get(axis), "{}.orientation.{}".format(label, axis))
        for axis in ("x", "y", "z", "w")
    ]
    if math.sqrt(sum(value * value for value in quaternion)) <= 1.0e-9:
        raise AutomationError("{} orientation is zero".format(label))


def load_templates(path, selected_rounds=None):
    path = path.expanduser().resolve()
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        raise AutomationError("cannot read {}: {}".format(path, exc))
    templates = payload.get("templates") if isinstance(payload, dict) else None
    if not isinstance(templates, list):
        raise AutomationError("template manifest must contain a templates list")

    expected_cones = {"cone_{}".format(number) for number in range(10, 20)}
    by_round = {}
    ids = set()
    for template in templates:
        if not isinstance(template, dict):
            raise AutomationError("every template must be a mapping")
        template_id = str(template.get("template_id", "")).strip()
        try:
            source_round = int(template.get("source_round"))
        except (TypeError, ValueError) as exc:
            raise AutomationError("template source_round must be an integer") from exc
        if not template_id or template_id in ids or source_round in by_round:
            raise AutomationError("template ids and source rounds must be unique")
        if template.get("target_class") not in {"food", "daily", "electronics"}:
            raise AutomationError("{} has invalid target_class".format(template_id))
        cones = template.get("cones")
        if not isinstance(cones, dict) or set(cones) != expected_cones:
            raise AutomationError(
                "{} must contain cone_10 through cone_19".format(template_id)
            )
        for cone_name, pose in cones.items():
            validate_pose(pose, "{}.{}".format(template_id, cone_name))
        ids.add(template_id)
        by_round[source_round] = template

    if selected_rounds:
        if len(set(selected_rounds)) != len(selected_rounds):
            raise AutomationError("--source-round values must not be repeated")
        unknown = set(selected_rounds) - set(by_round)
        if unknown:
            raise AutomationError(
                "unknown source rounds: {}".format(
                    ", ".join(str(value) for value in sorted(unknown))
                )
            )
        return [by_round[value] for value in selected_rounds]

    if tuple(sorted(by_round)) != EXPECTED_SOURCE_ROUNDS:
        raise AutomationError(
            "default manifest must define exactly source rounds 7, 9, 12, and 32"
        )
    return [by_round[value] for value in EXPECTED_SOURCE_ROUNDS]


def build_plans(templates, rounds, runtime):
    repetitions = {template["template_id"]: 0 for template in templates}
    plans = []
    for index in range(rounds):
        template = templates[index % len(templates)]
        scenario_id = template["template_id"]
        repetitions[scenario_id] += 1
        target_class = template["target_class"]
        destination = runtime["destinations"][target_class]
        plans.append(
            {
                "round": index + 1,
                "scenario_id": scenario_id,
                "source_round": int(template["source_round"]),
                "source_task_id": str(template.get("source_task_id", "")),
                "repetition": repetitions[scenario_id],
                "target_class": target_class,
                "target_name": destination["name"],
                "switch_distance": runtime["switch_distance"],
                "destination": {
                    key: value
                    for key, value in destination.items()
                    if key != "name"
                },
                "cones": template["cones"],
            }
        )
    return plans


def read_global_footprint():
    try:
        payload = yaml.safe_load(GLOBAL_COSTMAP_CONFIG.read_text(encoding="utf-8"))
        config = payload["global_costmap"]
        padding = finite(config.get("footprint_padding", 0.0), "footprint_padding")
    except (OSError, yaml.YAMLError, KeyError, TypeError) as exc:
        raise AutomationError(
            "cannot read fixed global robot geometry from {}: {}".format(
                GLOBAL_COSTMAP_CONFIG, exc
            )
        )
    footprint = config.get("footprint")
    if footprint is None:
        try:
            radius = finite(config["robot_radius"], "robot_radius")
        except (KeyError, TypeError) as exc:
            raise AutomationError(
                "global_costmap must define footprint or robot_radius"
            ) from exc
        if radius <= 0.0:
            raise AutomationError("global_costmap.robot_radius must be positive")
        return {"type": "radius", "radius": radius, "padding": padding}
    if not isinstance(footprint, list) or len(footprint) < 3:
        raise AutomationError("global_costmap.footprint must contain at least 3 points")
    points = []
    for index, point in enumerate(footprint):
        if not isinstance(point, list) or len(point) != 2:
            raise AutomationError("global footprint point {} is invalid".format(index))
        points.append([finite(point[0], "footprint.x"), finite(point[1], "footprint.y")])
    return {"type": "polygon", "points": points, "padding": padding}


def validate_args(args):
    e2e.validate_args(args)
    if args.rounds <= 0:
        raise AutomationError("--rounds must be positive")
    if args.startup_retries < 0:
        raise AutomationError("--startup-retries must be non-negative")
    if args.experiment_label and not re.fullmatch(
        r"[A-Za-z0-9][A-Za-z0-9_.-]*", args.experiment_label
    ):
        raise AutomationError(
            "--experiment-label must contain only letters, digits, dot, dash, "
            "or underscore"
        )
    for name in (
        "navigation_ready_timeout",
        "scene_timeout",
        "task_progress_timeout",
        "status_interval",
        "adaptive_monitor_ready_timeout",
    ):
        if getattr(args, name) <= 0.0:
            raise AutomationError("--{} must be positive".format(name.replace("_", "-")))
    for path in (
        SETUP_FILE,
        args.templates.expanduser().resolve(),
        SCENE_HELPER,
        NAVIGATION_HELPER,
        ADAPTIVE_DIAGNOSTICS_RECORDER,
        *NAVIGATION_CONFIG_FILES,
    ):
        if not path.is_file():
            raise AutomationError("missing required file {}".format(path))


def new_record(
    plan, round_dir, recording_round_dir, recording_enabled, attempt=1
):
    task_id = "fixed_cone_e2e_{:03d}_a{:02d}_{}".format(
        plan["round"], attempt, int(time.time())
    )
    record = e2e.new_record(
        plan["round"],
        task_id,
        plan["target_class"],
        round_dir,
        recording_round_dir,
        recording_enabled=recording_enabled,
    )
    record.update(
        {
            "scenario_id": plan["scenario_id"],
            "source_round": plan["source_round"],
            "repetition": plan["repetition"],
            "attempt": attempt,
            "infrastructure_retry_count": attempt - 1,
            "case_file": str(round_dir / "case_plan.json"),
            "scene_measurement_file": str(round_dir / "scene_measured.json"),
            "navigation_parameter_file": str(
                round_dir / "navigation_parameters.json"
            ),
            "adaptive_monitor_file": str(
                round_dir / "adaptive_teb_diagnostics.json"
            ),
            "adaptive_monitor_status": "not_started",
            "adaptive_mode_event_count": 0,
            "adaptive_diagnostic_sample_count": 0,
            "adaptive_avoidance_entries": 0,
            "adaptive_avoidance_used": False,
            "adaptive_avoidance_command_samples": 0,
            "adaptive_avoidance_failure_recovery_samples": 0,
            "adaptive_avoidance_infeasible_samples": 0,
            "adaptive_baseline_fallback_events": 0,
            "adaptive_baseline_fallback_samples": 0,
            "adaptive_baseline_infeasible_samples": 0,
            "adaptive_both_planners_infeasible_samples": 0,
            "adaptive_minimum_plan_clearance_m": None,
        }
    )
    return record


def check_navigation_readiness(args, case_file, log_file):
    return_code, output = run_owned(
        [
            "python3",
            str(NAVIGATION_HELPER),
            "--case-file",
            str(case_file),
            "--readiness-only",
            "--require-mission-action",
            "--require-localization",
            "--skip-profile-services",
            "--navigation-wait-timeout",
            str(args.navigation_ready_timeout),
        ],
        timeout=args.navigation_ready_timeout * 3.0 + 10.0,
    )
    e2e.append_section(log_file, "navigation stack readiness", output)
    if return_code != 0:
        raise AutomationError("navigation stack readiness check failed")


def announce_phase(round_number, phase, started, detail=""):
    suffix = " {}".format(detail) if detail else ""
    print(
        "  [round {:03d}] phase={} elapsed={:.1f}s{}".format(
            round_number, phase, time.monotonic() - started, suffix
        ),
        flush=True,
    )


def wait_for_trial_simulation(timeout, report=None):
    """Wait only for dependencies needed before the staging route starts.

    RGB-D image readiness is intentionally left to the perception pipeline. It
    is not needed for the preceding navigation route and waiting for it here
    made a visible Gazebo window look like a navigation goal had already been
    sent when the runner was still in startup.
    """
    def wait(label, arguments, required_text=None):
        if report is not None:
            report(label)
        # Give each independent dependency its own deadline. A single shared
        # deadline caused late checks to receive as little as 0.5 seconds when
        # gzclient made earlier Gazebo calls slow.
        wait_for_ros(
            label,
            arguments,
            time.monotonic() + timeout,
            required_text=required_text,
        )

    wait("ROS master", ["rosnode", "list"])
    wait(
        "Gazebo required models",
        ["rostopic", "echo", "-n", "1", "/gazebo/model_states/name"],
        required_text=("car3", "cube_0", "cube_1", "cube_2"),
    )
    wait("Gazebo clock", ["rostopic", "echo", "-n", "1", "/clock", "--noarr"])
    wait(
        "AMCL pose",
        ["rostopic", "echo", "-n", "1", "/amcl_pose", "--noarr"],
    )
    wait(
        "mission server",
        ["rosnode", "info", "/smart_factory_mission"],
    )


def capture_navigation_parameters(output_path, log_file):
    """Persist the effective ROS parameters that identify a tuning run."""
    values = {}
    for parameter_name in RUNTIME_PARAMETER_NAMES:
        return_code, output = run_owned(
            ["rosparam", "get", parameter_name], timeout=10.0
        )
        e2e.append_section(
            log_file, "runtime parameter {}".format(parameter_name), output
        )
        if return_code != 0:
            raise AutomationError(
                "cannot read runtime parameter {}".format(parameter_name)
            )
        try:
            values[parameter_name] = yaml.safe_load(output)
        except yaml.YAMLError as exc:
            raise AutomationError(
                "cannot parse runtime parameter {}".format(parameter_name)
            ) from exc
    output_path.write_text(
        json.dumps(values, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return values


def _label_number(value):
    return float(value.replace("p", "."))


def configured_avoidance_footprint_half_extent():
    """Read the declared avoidance polygon before starting Gazebo."""
    try:
        document = yaml.safe_load(ADAPTIVE_TEB_CONFIG.read_text(encoding="utf-8"))
        vertices = document["AdaptiveTebLocalPlannerROS"]["avoidance"][
            "footprint_model"
        ]["vertices"]
        half_extent = max(
            max(abs(finite(point[0], "avoidance footprint x")),
                abs(finite(point[1], "avoidance footprint y")))
            for point in vertices
        )
    except (OSError, yaml.YAMLError, KeyError, TypeError, IndexError) as exc:
        raise AutomationError(
            "cannot read avoidance footprint from {}: {}".format(
                ADAPTIVE_TEB_CONFIG, exc
            )
        )
    if half_extent <= 0.0:
        raise AutomationError("adaptive avoidance footprint must be positive")
    return half_extent


def validate_source_experiment_label(label):
    """Catch a stale footprint label before opening Gazebo/RViz."""
    match = re.search(r"(?:^|_)avoidfp_([0-9]+p[0-9]+)(?:_|$)", label)
    if match is None:
        return
    encoded = _label_number(match.group(1))
    configured = configured_avoidance_footprint_half_extent()
    if not math.isclose(encoded, configured, rel_tol=0.0, abs_tol=1.0e-9):
        raise AutomationError(
            "experiment label avoidance footprint half-extent={} does not "
            "match {} value={}; Gazebo was not started and no task was "
            "submitted".format(encoded, ADAPTIVE_TEB_CONFIG, configured)
        )


def validate_runtime_experiment_label(label, parameters):
    """Reject recognized label tokens that disagree with loaded ROS params."""
    if not label:
        return
    expected = (
        (
            "global inflation",
            r"(?:^|_)(?:ginf|global_inf)_([0-9]+p[0-9]+)(?:_|$)",
            "/move_base/global_costmap/inflation_layer/inflation_radius",
        ),
        (
            "global cost scaling",
            r"(?:^|_)gscale_([0-9]+p[0-9]+)(?:_|$)",
            "/move_base/global_costmap/inflation_layer/cost_scaling_factor",
        ),
        (
            "neutral cost",
            r"(?:^|_)nc([0-9]+(?:p[0-9]+)?)(?:_|$)",
            "/move_base/GlobalPlanner/neutral_cost",
        ),
        (
            "cost factor",
            r"(?:^|_)cf_([0-9]+p[0-9]+)(?:_|$)",
            "/move_base/GlobalPlanner/cost_factor",
        ),
    )
    mismatches = []
    for description, pattern, parameter_name in expected:
        match = re.search(pattern, label)
        if match is None:
            continue
        encoded = _label_number(match.group(1))
        actual = float(parameters[parameter_name])
        if not math.isclose(encoded, actual, rel_tol=0.0, abs_tol=1.0e-9):
            mismatches.append(
                "{} label={} runtime={}".format(description, encoded, actual)
            )

    if re.search(r"(?:^|_)astar(?:_|$)", label) and bool(
        parameters["/move_base/GlobalPlanner/use_dijkstra"]
    ):
        mismatches.append("label requests A* but use_dijkstra=true")
    if re.search(r"(?:^|_)dijkstra(?:_|$)", label) and not bool(
        parameters["/move_base/GlobalPlanner/use_dijkstra"]
    ):
        mismatches.append("label requests Dijkstra but use_dijkstra=false")
    teb_plugins = {
        "teb_local_planner/TebLocalPlannerROS",
        ADAPTIVE_TEB_PLUGIN,
    }
    if re.search(r"(?:^|_)teb(?:_|$)", label) and parameters.get(
        "/move_base/base_local_planner"
    ) not in teb_plugins:
        mismatches.append(
            "label requests TEB but base_local_planner={}".format(
                parameters.get("/move_base/base_local_planner")
            )
        )
    if re.search(r"(?:^|_)adaptive(?:_|$)", label) and parameters.get(
        "/move_base/base_local_planner"
    ) != ADAPTIVE_TEB_PLUGIN:
        mismatches.append(
            "label requests adaptive TEB but base_local_planner={}".format(
                parameters.get("/move_base/base_local_planner")
            )
        )

    footprint_match = re.search(
        r"(?:^|_)avoidfp_([0-9]+p[0-9]+)(?:_|$)", label
    )
    if footprint_match is not None:
        adaptive = parameters.get("/move_base/AdaptiveTebLocalPlannerROS")
        try:
            vertices = adaptive["avoidance"]["footprint_model"]["vertices"]
            actual_half_extent = max(
                max(abs(float(point[0])), abs(float(point[1])))
                for point in vertices
            )
        except (KeyError, TypeError, ValueError, IndexError):
            mismatches.append("adaptive avoidance footprint is unavailable")
        else:
            encoded = _label_number(footprint_match.group(1))
            if not math.isclose(
                encoded, actual_half_extent, rel_tol=0.0, abs_tol=1.0e-9
            ):
                mismatches.append(
                    "avoidance footprint half-extent label={} runtime={}".format(
                        encoded, actual_half_extent
                    )
                )

    if mismatches:
        raise AutomationError(
            "experiment label does not match runtime parameters: {}".format(
                "; ".join(mismatches)
            )
        )


def adaptive_planner_enabled(parameters):
    return (
        parameters.get("/move_base/base_local_planner")
        == ADAPTIVE_TEB_PLUGIN
    )


def start_adaptive_diagnostics_monitor(args, record, round_dir):
    """Start a per-round recorder and require the plugin's latched mode topic."""
    ready_path = round_dir / ".adaptive_teb_monitor.ready"
    monitor_log_path = round_dir / "adaptive_teb_monitor.log"
    monitor_log = monitor_log_path.open("w", encoding="utf-8")
    command = e2e.ros_command(
        [
            "python3",
            str(ADAPTIVE_DIAGNOSTICS_RECORDER),
            "--output",
            record["adaptive_monitor_file"],
            "--ready-file",
            str(ready_path),
            "--round",
            str(record["round"]),
            "--task-id",
            record["task_id"],
        ]
    )
    process = None
    try:
        process = subprocess.Popen(
            command,
            cwd=str(WORKSPACE),
            stdout=monitor_log,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )
        deadline = time.monotonic() + args.adaptive_monitor_ready_timeout
        while time.monotonic() < deadline:
            if ready_path.is_file():
                record["adaptive_monitor_status"] = "monitoring"
                return process, monitor_log, ready_path
            if process.poll() is not None:
                raise AutomationError(
                    "adaptive TEB monitor exited during startup; see {}".format(
                        monitor_log_path
                    )
                )
            time.sleep(0.1)
        raise AutomationError(
            "adaptive TEB mode topic was unavailable for {:.1f}s; see {}".format(
                args.adaptive_monitor_ready_timeout, monitor_log_path
            )
        )
    except Exception:
        stop_process_group(process, interrupt_timeout=3.0)
        monitor_log.close()
        raise


def add_adaptive_diagnostics_result(record):
    if record["adaptive_monitor_status"] == "disabled":
        return
    path = Path(record["adaptive_monitor_file"])
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        record["adaptive_monitor_status"] = "manifest_unavailable"
        return
    if (
        payload.get("status") != "complete"
        or payload.get("task_id") != record["task_id"]
        or payload.get("round") != record["round"]
    ):
        record["adaptive_monitor_status"] = "manifest_invalid"
        return
    summary = payload.get("summary")
    if not isinstance(summary, dict):
        record["adaptive_monitor_status"] = "manifest_invalid"
        return
    record["adaptive_monitor_status"] = "complete"
    mappings = {
        "adaptive_mode_event_count": "mode_event_count",
        "adaptive_diagnostic_sample_count": "diagnostic_sample_count",
        "adaptive_avoidance_entries": "avoidance_entries",
        "adaptive_avoidance_used": "avoidance_used",
        "adaptive_avoidance_command_samples": "avoidance_command_samples",
        "adaptive_avoidance_failure_recovery_samples": (
            "avoidance_failure_recovery_samples"
        ),
        "adaptive_avoidance_infeasible_samples": (
            "avoidance_infeasible_samples"
        ),
        "adaptive_baseline_fallback_events": "baseline_fallback_events",
        "adaptive_baseline_fallback_samples": "baseline_fallback_samples",
        "adaptive_baseline_infeasible_samples": "baseline_infeasible_samples",
        "adaptive_both_planners_infeasible_samples": (
            "both_planners_infeasible_samples"
        ),
        "adaptive_minimum_plan_clearance_m": "minimum_plan_clearance_m",
    }
    for record_key, summary_key in mappings.items():
        if summary_key in summary:
            record[record_key] = summary[summary_key]


def _task_state_snapshot(task_id):
    return_code, output = run_owned(
        ["rostopic", "echo", "-n", "1", "/sim_task/state", "--noarr"],
        timeout=5.0,
    )
    if return_code != 0:
        return None
    document = output.rsplit("\n---", 1)[0]
    try:
        payload = yaml.safe_load(document)
    except yaml.YAMLError:
        return None
    if not isinstance(payload, dict) or payload.get("task_id") != task_id:
        return None
    return payload


def task_status_heartbeat(stop_event, task_id, interval, started):
    while not stop_event.wait(interval):
        try:
            state = _task_state_snapshot(task_id)
        except AutomationError as exc:
            print(
                "  task heartbeat unavailable elapsed={:.1f}s: {}".format(
                    time.monotonic() - started, exc
                ),
                flush=True,
            )
            continue
        if state is None:
            print(
                "  task heartbeat waiting elapsed={:.1f}s task_id={}".format(
                    time.monotonic() - started, task_id
                ),
                flush=True,
            )
            continue
        detail = " ".join(str(state.get("detail", "")).split())
        print(
            "  task heartbeat elapsed={:.1f}s state={} retry={} detail={}".format(
                time.monotonic() - started,
                state.get("state"),
                state.get("retry_count"),
                detail[:180],
            ),
            flush=True,
        )


def progress_timeout_payload(output):
    marker = "TASK_PROGRESS_TIMEOUT="
    for line in output.splitlines():
        if marker not in line:
            continue
        raw = line.split(marker, 1)[1].strip()
        try:
            payload = json.loads(raw)
        except ValueError:
            return {"error": "invalid progress-timeout marker", "raw": raw}
        return payload if isinstance(payload, dict) else {"raw": payload}
    return None


def run_trial(args, plan, output_dir, recording_run_dir, attempt=1):
    round_number = plan["round"]
    suffix = (
        "round_{:03d}".format(round_number)
        if attempt == 1
        else "round_{:03d}_retry_{:02d}".format(round_number, attempt - 1)
    )
    round_dir = output_dir / suffix
    round_dir.mkdir(parents=True, exist_ok=False)
    recording_round_dir = recording_run_dir / suffix
    record = new_record(
        plan,
        round_dir,
        recording_round_dir,
        recording_enabled=not args.disable_gazebo_recording,
        attempt=attempt,
    )
    case_payload = dict(plan)
    case_payload.update(
        {
            "schema_version": 1,
            "task_id": record["task_id"],
            "attempt": attempt,
        }
    )
    case_file = Path(record["case_file"])
    case_text = json.dumps(case_payload, ensure_ascii=False, indent=2) + "\n"
    case_file.write_text(case_text, encoding="utf-8")

    started = time.monotonic()
    launch_process = None
    launch_run_id = None
    monitor_process = None
    monitor_log = None
    monitor_ready = None
    recorder_process = None
    recorder_log = None
    recorder_ready = None
    adaptive_process = None
    adaptive_log = None
    adaptive_ready = None
    phase = "startup"

    with Path(record["log_file"]).open("w", encoding="utf-8") as log_file:
        try:
            announce_phase(round_number, "startup", started, "launching ROS/Gazebo")
            if master_is_running():
                raise CleanupError(
                    "a ROS master is still running before round {}; refusing to "
                    "reuse a stale simulation".format(round_number)
                )
            launch_process = launch_simulation(args.gui, log_file)
            deadline = time.monotonic() + args.startup_timeout
            wait_for_ros("ROS master", ["rosnode", "list"], deadline)
            launch_run_id = read_ros_run_id()
            wait_for_trial_simulation(
                args.startup_timeout,
                report=lambda label: announce_phase(
                    round_number, "startup_wait", started, label
                ),
            )
            if launch_process.poll() is not None:
                raise AutomationError("roslaunch exited during startup")
            if args.startup_settle:
                announce_phase(
                    round_number,
                    "startup_settle",
                    started,
                    "{:.1f}s".format(args.startup_settle),
                )
                time.sleep(args.startup_settle)

            phase = "navigation_snapshot"
            announce_phase(round_number, phase, started)
            runtime_parameters = capture_navigation_parameters(
                Path(record["navigation_parameter_file"]), log_file
            )

            phase = "configuration_validation"
            announce_phase(round_number, phase, started)
            validate_runtime_experiment_label(
                args.experiment_label, runtime_parameters
            )
            if not adaptive_planner_enabled(runtime_parameters):
                record["adaptive_monitor_status"] = "disabled"

            phase = "scene_setup"
            announce_phase(round_number, phase, started, "restoring fixed cones")
            return_code, output = run_owned(
                [
                    "python3",
                    str(SCENE_HELPER),
                    "--case-file",
                    str(case_file),
                    "--cones-only",
                ],
                timeout=args.scene_timeout,
            )
            e2e.append_section(log_file, "fixed end-to-end cone scene", output)
            if return_code != 0:
                raise AutomationError("fixed cone scene setup failed")
            measured = parse_marker(output, "CONE_MOVE_STRESS_SCENE=")
            Path(record["scene_measurement_file"]).write_text(
                json.dumps(measured, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )

            # The cone helper pauses physics and clears both costmaps. Check a
            # fresh localization result afterwards instead of relying only on
            # the AMCL message observed before scene restoration.
            phase = "navigation_readiness"
            announce_phase(
                round_number,
                phase,
                started,
                "checking actions, fresh AMCL and map TF",
            )
            check_navigation_readiness(args, case_file, log_file)

            if not args.disable_gazebo_recording:
                phase = "gazebo_recording_startup"
                announce_phase(round_number, phase, started)
                recorder_process, recorder_log, recorder_ready = (
                    e2e.start_gazebo_world_recorder(
                        args, record, recording_round_dir, start_stage=5
                    )
                )
                (recording_round_dir / "case_plan.json").write_text(
                    case_text, encoding="utf-8"
                )

            phase = "cone_monitor_startup"
            announce_phase(round_number, phase, started)
            monitor_process, monitor_log, monitor_ready = e2e.start_cone_monitor(
                args, record, round_dir
            )

            if adaptive_planner_enabled(runtime_parameters):
                phase = "adaptive_monitor_startup"
                announce_phase(round_number, phase, started)
                adaptive_process, adaptive_log, adaptive_ready = (
                    start_adaptive_diagnostics_monitor(args, record, round_dir)
                )

            phase = "task"
            announce_phase(
                round_number,
                phase,
                started,
                "publishing task; no-progress timeout={:.0f}s".format(
                    args.task_progress_timeout
                ),
            )
            heartbeat_stop = threading.Event()
            heartbeat = threading.Thread(
                target=task_status_heartbeat,
                args=(
                    heartbeat_stop,
                    record["task_id"],
                    args.status_interval,
                    started,
                ),
                name="fixed-cone-task-heartbeat",
                daemon=True,
            )
            heartbeat.start()
            try:
                try:
                    return_code, output = run_owned(
                        [
                            "rosrun",
                            "smart_factory_tests",
                            "send_navigation_task.py",
                            "--task-id",
                            record["task_id"],
                            "--target-class",
                            plan["target_class"],
                            "_server_wait_timeout:={}".format(
                                args.navigation_ready_timeout
                            ),
                            "_progress_timeout:={}".format(
                                args.task_progress_timeout
                            ),
                        ],
                        timeout=args.task_timeout,
                    )
                finally:
                    heartbeat_stop.set()
                    heartbeat.join(timeout=2.0)
            except AutomationError as exc:
                record["status"] = "task_timeout"
                record["message"] = str(exc)
                e2e.append_section(log_file, "task client timeout", str(exc))
            else:
                e2e.append_section(log_file, "task client", output)
                progress_timeout = progress_timeout_payload(output)
                parsed = e2e.parse_task_result(output)
                if parsed is None:
                    if progress_timeout is not None:
                        record["status"] = "task_timeout"
                        record["message"] = (
                            "task made no mission-stage progress for {:.1f}s"
                        ).format(
                            float(
                                progress_timeout.get(
                                    "inactive_seconds",
                                    args.task_progress_timeout,
                                )
                            )
                        )
                    else:
                        record["status"] = "result_unavailable"
                        record["message"] = (
                            "task client exited with code {} without a parseable result"
                        ).format(return_code)
                else:
                    record["completed_stage"] = parsed["completed_stage"]
                    record["error_code"] = parsed["error_code"]
                    record["message"] = parsed["message"]
                    record["success"] = bool(
                        return_code == 0
                        and parsed["reported_success"]
                        and parsed["completed_stage"] == 20
                        and progress_timeout is None
                    )
                    if progress_timeout is not None:
                        record["status"] = "task_timeout"
                        record["message"] = (
                            "task made no mission-stage progress for {:.1f}s"
                        ).format(
                            float(
                                progress_timeout.get(
                                    "inactive_seconds",
                                    args.task_progress_timeout,
                                )
                            )
                        )
                    else:
                        record["status"] = (
                            "task_completed" if record["success"] else "task_failed"
                        )
        except CleanupError:
            raise
        except AutomationError as exc:
            record["status"] = "{}_error".format(phase)
            record["message"] = str(exc)
            e2e.append_section(log_file, "automation error", str(exc))
        finally:
            try:
                stop_process_group(adaptive_process, interrupt_timeout=6.0)
                if adaptive_log is not None:
                    adaptive_log.close()
                if adaptive_ready is not None:
                    try:
                        adaptive_ready.unlink()
                    except FileNotFoundError:
                        pass
            finally:
                try:
                    stop_process_group(monitor_process, interrupt_timeout=6.0)
                    if monitor_log is not None:
                        monitor_log.close()
                    if monitor_ready is not None:
                        try:
                            monitor_ready.unlink()
                        except FileNotFoundError:
                            pass
                finally:
                    try:
                        stop_process_group(
                            recorder_process, interrupt_timeout=12.0
                        )
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

    monitor, monitor_error = e2e.load_monitor(record)
    e2e.add_monitor_result(record, monitor, monitor_error)
    e2e.add_gazebo_recording_result(record)
    add_adaptive_diagnostics_result(record)
    return record


def is_retryable_infrastructure_failure(record):
    if record["status"] in {
        "startup_error",
        "navigation_readiness_error",
        "navigation_snapshot_error",
        "scene_setup_error",
        "gazebo_recording_startup_error",
        "cone_monitor_startup_error",
        "adaptive_monitor_startup_error",
    }:
        return True
    return (
        record["status"] == "result_unavailable"
        and "code 2" in str(record.get("message", ""))
    )


def infrastructure_retries_exhausted(record, attempt, startup_retries):
    """Return true only after the final infrastructure attempt also failed."""
    return (
        attempt > startup_retries
        and is_retryable_infrastructure_failure(record)
    )


def is_fatal_batch_failure(record):
    """Reject a batch whose declared experiment no longer matches its config.

    Continuing after this error used to relaunch every remaining scenario
    without ever submitting a task. In GUI mode that looked like Gazebo was
    frozen at the start pose, while a manual task (which bypassed this guard)
    still moved the robot.
    """
    return record["status"] == "configuration_validation_error"


def make_summary(results, requested_rounds, plans):
    successes = sum(bool(record["success"]) for record in results)
    by_scenario = {}
    scenario_ids = []
    for plan in plans:
        if plan["scenario_id"] not in scenario_ids:
            scenario_ids.append(plan["scenario_id"])
    for scenario_id in scenario_ids:
        expected = [plan for plan in plans if plan["scenario_id"] == scenario_id]
        selected = [
            record for record in results if record["scenario_id"] == scenario_id
        ]
        issue_rounds = [
            record
            for record in selected
            if bool(record["preceding_navigation_failure"])
            or bool(record["delivery_navigation_failure"])
            or bool(record["cone_collision"])
        ]
        by_scenario[scenario_id] = {
            "source_round": expected[0]["source_round"],
            "target_class": expected[0]["target_class"],
            "requested_rounds": len(expected),
            "completed_rounds": len(selected),
            "task_successes": sum(bool(record["success"]) for record in selected),
            "preceding_navigation_failures": sum(
                bool(record["preceding_navigation_failure"]) for record in selected
            ),
            "delivery_navigation_failures": sum(
                bool(record["delivery_navigation_failure"]) for record in selected
            ),
            "cone_collision_rounds": sum(
                bool(record["cone_collision"]) for record in selected
            ),
            "adaptive_avoidance_rounds": sum(
                bool(record.get("adaptive_avoidance_used"))
                for record in selected
            ),
            "adaptive_avoidance_entries": sum(
                int(record.get("adaptive_avoidance_entries", 0))
                for record in selected
            ),
            "adaptive_baseline_fallback_events": sum(
                int(record.get("adaptive_baseline_fallback_events", 0))
                for record in selected
            ),
            "tuning_issue_rounds": len(issue_rounds),
            "requires_further_tuning": bool(issue_rounds),
        }
    infrastructure_errors = sum(
        str(record["status"]).endswith("_error")
        or record["status"] == "result_unavailable"
        or record["cone_monitor_status"] != "complete"
        or record["gazebo_recording_status"] not in {"complete", "disabled"}
        or record.get("adaptive_monitor_status", "disabled")
        not in {"complete", "disabled"}
        for record in results
    )
    completed = len(results) == requested_rounds
    acceptance_passed = (
        completed
        and infrastructure_errors == 0
        and all(
            not scenario["requires_further_tuning"]
            for scenario in by_scenario.values()
        )
    )
    return {
        "requested_rounds": requested_rounds,
        "completed_rounds": len(results),
        "task_successes": successes,
        "task_failures": len(results) - successes,
        "success_rate": round(successes / len(results), 4) if results else None,
        "preceding_navigation_failures": sum(
            bool(record["preceding_navigation_failure"]) for record in results
        ),
        "delivery_navigation_failures": sum(
            bool(record["delivery_navigation_failure"]) for record in results
        ),
        "cone_collision_rounds": sum(
            bool(record["cone_collision"]) for record in results
        ),
        "adaptive_avoidance_rounds": sum(
            bool(record.get("adaptive_avoidance_used")) for record in results
        ),
        "adaptive_avoidance_entries": sum(
            int(record.get("adaptive_avoidance_entries", 0))
            for record in results
        ),
        "adaptive_baseline_fallback_events": sum(
            int(record.get("adaptive_baseline_fallback_events", 0))
            for record in results
        ),
        "adaptive_avoidance_infeasible_samples": sum(
            int(record.get("adaptive_avoidance_infeasible_samples", 0))
            for record in results
        ),
        "adaptive_both_planners_infeasible_samples": sum(
            int(record.get("adaptive_both_planners_infeasible_samples", 0))
            for record in results
        ),
        "gazebo_recording_complete_rounds": sum(
            record["gazebo_recording_status"] in {"complete", "disabled"}
            for record in results
        ),
        "infrastructure_errors": infrastructure_errors,
        "acceptance_rule": (
            "every scenario must complete all repetitions without any "
            "preceding-navigation failure, delivery-navigation failure, or "
            "cone collision"
        ),
        "acceptance_passed": acceptance_passed,
        "requires_further_tuning": completed and not acceptance_passed,
        "by_scenario": by_scenario,
    }


def write_reports(
    output_dir,
    results,
    plans,
    metadata,
    infrastructure_attempts=None,
    mirror_dir=None,
):
    csv_path = output_dir / "trials.csv"
    with csv_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS)
        writer.writeheader()
        writer.writerows(results)
    summary = make_summary(results, len(plans), plans)
    report_path = output_dir / "summary.json"
    report_path.write_text(
        json.dumps(
            {
                "metadata": metadata,
                "summary": summary,
                "trials": results,
                "infrastructure_attempts": infrastructure_attempts or [],
            },
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    if mirror_dir is not None:
        mirror_dir.mkdir(parents=True, exist_ok=True)
        shutil.copy2(csv_path, mirror_dir / csv_path.name)
        shutil.copy2(report_path, mirror_dir / report_path.name)
    return csv_path, report_path, summary


def print_summary(summary, csv_path, report_path):
    rate = summary["success_rate"]
    print("\n===== fixed cone end-to-end stress summary =====")
    print(
        "completed={}/{} successes={} failures={} success_rate={}".format(
            summary["completed_rounds"],
            summary["requested_rounds"],
            summary["task_successes"],
            summary["task_failures"],
            "n/a" if rate is None else "{:.1%}".format(rate),
        )
    )
    print(
        "pre_navigation_failures={} delivery_navigation_failures={} collisions={}".format(
            summary["preceding_navigation_failures"],
            summary["delivery_navigation_failures"],
            summary["cone_collision_rounds"],
        )
    )
    print(
        "recordings_complete={} infrastructure_errors={}".format(
            summary["gazebo_recording_complete_rounds"],
            summary["infrastructure_errors"],
        )
    )
    print(
        "adaptive_avoidance_rounds={} entries={} avoidance_infeasible_samples={} "
        "baseline_fallback_events={}".format(
            summary["adaptive_avoidance_rounds"],
            summary["adaptive_avoidance_entries"],
            summary["adaptive_avoidance_infeasible_samples"],
            summary["adaptive_baseline_fallback_events"],
        )
    )
    print(
        "acceptance_passed={} requires_further_tuning={}".format(
            summary["acceptance_passed"], summary["requires_further_tuning"]
        )
    )
    for scenario_id, scenario in summary["by_scenario"].items():
        print(
            "scenario={} source_round={} issues={} requires_further_tuning={}".format(
                scenario_id,
                scenario["source_round"],
                scenario["tuning_issue_rounds"],
                scenario["requires_further_tuning"],
            )
        )
    print("csv={}".format(csv_path))
    print("json={}".format(report_path))


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    results = []
    plans = []
    output_dir = None
    recording_run_dir = None
    metadata = None
    infrastructure_attempts = []
    try:
        validate_args(args)
        validate_source_experiment_label(args.experiment_label)
        selected_rounds = (
            args.source_rounds
            if args.source_rounds is not None
            else list(DEFAULT_SOURCE_ROUNDS)
        )
        templates = load_templates(args.templates, selected_rounds)
        runtime = load_runtime_config()
        global_footprint = read_global_footprint()
        plans = build_plans(templates, args.rounds, runtime)
        template_path = args.templates.expanduser().resolve()
        metadata = {
            "created_at": dt.datetime.now().astimezone().isoformat(timespec="seconds"),
            "test_type": "fixed cone layout, complete pickup-and-delivery pipeline",
            "experiment_label": args.experiment_label,
            "template_manifest": str(template_path),
            "template_manifest_sha256": e2e.sha256(template_path),
            "source_rounds": [template["source_round"] for template in templates],
            "schedule": [
                {
                    "round": plan["round"],
                    "scenario_id": plan["scenario_id"],
                    "source_round": plan["source_round"],
                    "repetition": plan["repetition"],
                    "target_class": plan["target_class"],
                }
                for plan in plans
            ],
            "scene_isolation": (
                "only cones are restored; car3, AMCL, and pickup cubes are untouched"
            ),
            "navigation_ready_timeout_seconds": args.navigation_ready_timeout,
            "startup_dependency_timeout_seconds": args.startup_timeout,
            "whole_task_timeout_seconds": args.task_timeout,
            "task_progress_timeout_seconds": args.task_progress_timeout,
            "status_interval_seconds": args.status_interval,
            "adaptive_monitor_ready_timeout_seconds": (
                args.adaptive_monitor_ready_timeout
            ),
            "adaptive_diagnostics": {
                "mode_topic": (
                    "/move_base/AdaptiveTebLocalPlannerROS/adaptive_mode"
                ),
                "diagnostics_topic": (
                    "/move_base/AdaptiveTebLocalPlannerROS/"
                    "adaptive_diagnostics"
                ),
            },
            "startup_retries": args.startup_retries,
            "global_footprint_config": str(GLOBAL_COSTMAP_CONFIG),
            "global_footprint_config_sha256": e2e.sha256(GLOBAL_COSTMAP_CONFIG),
            "global_footprint": global_footprint,
            "navigation_config_files": {
                str(path): {"sha256": e2e.sha256(path)}
                for path in NAVIGATION_CONFIG_FILES
            },
            "mission_config": str(e2e.MISSION_CONFIG),
            "random_spawner": str(e2e.RANDOM_SPAWNER),
            "random_spawner_sha256": e2e.sha256(e2e.RANDOM_SPAWNER),
            "gazebo_gui_enabled": args.gui,
            "gazebo_recording": {
                "enabled": not args.disable_gazebo_recording,
                "format": "Gazebo Classic replayable state.log (zlib)",
                "start_stage": "NAVIGATE_TO_PICKUP_STAGING",
                "root": str(args.recording_root.expanduser().resolve()),
            },
        }

        if args.dry_run:
            print(
                "scenarios={} rounds={} gazebo_gui={}".format(
                    len(templates),
                    len(plans),
                    "enabled" if args.gui else "disabled",
                )
            )
            for plan in plans:
                print(
                    "round_{:03d}: source_round={} repetition={} target={}".format(
                        plan["round"],
                        plan["source_round"],
                        plan["repetition"],
                        plan["target_class"],
                    )
                )
            print("global_footprint={}".format(global_footprint))
            print("startup_retries={}".format(args.startup_retries))
            print(
                "startup_dependency_timeout_seconds={}".format(
                    args.startup_timeout
                )
            )
            print("whole_task_timeout_seconds={}".format(args.task_timeout))
            print(
                "task_progress_timeout_seconds={}".format(
                    args.task_progress_timeout
                )
            )
            print("recording_root={}".format(args.recording_root.expanduser().resolve()))
            return 0

        if master_is_running():
            raise AutomationError(
                "a ROS master is already running; stop the manual test before "
                "starting isolated stress trials"
            )
        timestamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        run_name = "fixed_cone_e2e_stress_" + timestamp
        if args.experiment_label:
            run_name += "_" + args.experiment_label
        output_dir = (
            args.log_dir.expanduser().resolve()
            if args.log_dir is not None
            else WORKSPACE
            / "script"
            / "logs"
            / run_name
        )
        output_dir.mkdir(parents=True, exist_ok=False)
        recording_run_dir = (
            args.recording_root.expanduser().resolve() / output_dir.name
        )
        if not args.disable_gazebo_recording:
            recording_run_dir.mkdir(parents=True, exist_ok=False)
        metadata["gazebo_recording"]["run_dir"] = str(recording_run_dir)

        print("workspace={}".format(WORKSPACE))
        print("rounds={} logs={}".format(len(plans), output_dir))
        print("gazebo_gui={}".format("enabled" if args.gui else "disabled"))
        print("global_footprint={}".format(global_footprint))
        print("startup_dependency_timeout_seconds={}".format(args.startup_timeout))
        print("whole_task_timeout_seconds={}".format(args.task_timeout))
        print(
            "task_progress_timeout_seconds={}".format(
                args.task_progress_timeout
            )
        )
        if not args.disable_gazebo_recording:
            print("gazebo_recording={}".format(recording_run_dir))

        for plan in plans:
            print(
                "[round {}/{}] source_round={} repetition={} target={}".format(
                    plan["round"],
                    len(plans),
                    plan["source_round"],
                    plan["repetition"],
                    plan["target_class"],
                ),
                flush=True,
            )
            record = None
            abort_batch_for_infrastructure = False
            for attempt in range(1, args.startup_retries + 2):
                record = run_trial(
                    args,
                    plan,
                    output_dir,
                    recording_run_dir,
                    attempt=attempt,
                )
                if not is_retryable_infrastructure_failure(record):
                    break
                infrastructure_attempts.append(record)
                if attempt <= args.startup_retries:
                    print(
                        "  infrastructure failure: {}; relaunching logical "
                        "round (retry {}/{})".format(
                            record["status"], attempt, args.startup_retries
                        ),
                        flush=True,
                    )
                    if args.restart_settle:
                        time.sleep(args.restart_settle)
                else:
                    abort_batch_for_infrastructure = (
                        infrastructure_retries_exhausted(
                            record, attempt, args.startup_retries
                        )
                    )
            assert record is not None
            results.append(record)
            csv_path, report_path, _summary = write_reports(
                output_dir,
                results,
                plans,
                metadata,
                infrastructure_attempts,
                recording_run_dir,
            )
            print(
                "  status={} success={} pre_nav_failure={} delivery_failure={} "
                "collision={} cones={} adaptive_entries={} adaptive_infeasible={} "
                "recording={} duration={:.1f}s".format(
                    record["status"],
                    record["success"],
                    record["preceding_navigation_failure"],
                    record["delivery_navigation_failure"],
                    record["cone_collision"],
                    record["collision_cones"] or "none",
                    record["adaptive_avoidance_entries"],
                    record["adaptive_avoidance_infeasible_samples"],
                    record["gazebo_recording_status"],
                    record["duration_seconds"],
                ),
                flush=True,
            )
            if abort_batch_for_infrastructure:
                raise AutomationError(
                    "aborting batch after logical round {} failed all {} "
                    "infrastructure startup attempts (last status: {})".format(
                        plan["round"], args.startup_retries + 1, record["status"]
                    )
                )
            if is_fatal_batch_failure(record):
                raise AutomationError(
                    "aborting batch after configuration validation failed in "
                    "logical round {}; no task was submitted. Fix the "
                    "experiment label or restore the intended configuration "
                    "before retrying: {}".format(
                        plan["round"], record["message"]
                    )
                )
            if plan["round"] < len(plans) and args.restart_settle:
                time.sleep(args.restart_settle)

        csv_path, report_path, summary = write_reports(
            output_dir,
            results,
            plans,
            metadata,
            infrastructure_attempts,
            recording_run_dir,
        )
        print_summary(summary, csv_path, report_path)
        return 0
    except KeyboardInterrupt:
        print("interrupted by user", file=sys.stderr)
        if output_dir is not None and metadata is not None:
            paths = write_reports(
                output_dir,
                results,
                plans,
                metadata,
                infrastructure_attempts,
                recording_run_dir,
            )
            print_summary(paths[2], paths[0], paths[1])
        return 130
    except (AutomationError, CleanupError, OSError, subprocess.SubprocessError) as exc:
        print("run_fixed_cone_e2e_stress_trials: {}".format(exc), file=sys.stderr)
        if output_dir is not None and metadata is not None:
            paths = write_reports(
                output_dir,
                results,
                plans,
                metadata,
                infrastructure_attempts,
                recording_run_dir,
            )
            print_summary(paths[2], paths[0], paths[1])
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
