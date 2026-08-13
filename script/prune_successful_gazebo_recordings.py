#!/usr/bin/env python3
"""Conservatively remove recordings from rounds proven to be error-free."""

import argparse
import csv
import datetime as dt
import fcntl
import json
import math
import os
from pathlib import Path
import re
import sys
import time


WORKSPACE = Path(__file__).resolve().parents[1]
DEFAULT_DATA_ROOT = WORKSPACE / "data" / "35seq_fix" / "end_to_end_test"
DEFAULT_CONE_VIDEO_ROOT = WORKSPACE / "data" / "cone_zone" / "end_to_end_test"
DEFAULT_CONE_STRESS_ROOT = (
    WORKSPACE / "data" / "cone_zone" / "end_to_end_test" / "conse_stress"
)
DEFAULT_FIXED_CONE_E2E_ROOT = (
    WORKSPACE / "data" / "cone_zone" / "end_to_end_stress"
)
DEFAULT_PRE_NAVIGATION_ROOT = WORKSPACE / "data" / "pre_navigation_test"
DEFAULT_LOGS_ROOT = WORKSPACE / "script" / "logs"
ROUND_PATTERN = re.compile(r"round_(\d+)$")
RECORDING_FILES = (
    "gazebo_world_state.log",
    "gazebo_world_recording.json",
    "gazebo_world_recorder.log",
)
CONE_RECORDING_FILES = RECORDING_FILES
CONE_VIDEO_FILES = CONE_RECORDING_FILES  # Backward-compatible import name.


class CleanupError(RuntimeError):
    pass


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description=(
            "remove legacy Gazebo world recordings after a correct successful "
            "grasp, and navigation/cone-test world recordings only after a "
            "fully error-free round"
        )
    )
    parser.add_argument("--data-root", type=Path, default=DEFAULT_DATA_ROOT)
    parser.add_argument(
        "--cone-recording-root",
        "--cone-video-root",
        dest="cone_video_root",
        type=Path,
        default=DEFAULT_CONE_VIDEO_ROOT,
    )
    parser.add_argument(
        "--cone-stress-root",
        type=Path,
        default=DEFAULT_CONE_STRESS_ROOT,
    )
    parser.add_argument(
        "--pre-navigation-root",
        type=Path,
        default=DEFAULT_PRE_NAVIGATION_ROOT,
    )
    parser.add_argument(
        "--fixed-cone-e2e-root",
        type=Path,
        default=DEFAULT_FIXED_CONE_E2E_ROOT,
    )
    parser.add_argument("--logs-root", type=Path, default=DEFAULT_LOGS_ROOT)
    parser.add_argument(
        "--run",
        action="append",
        dest="runs",
        help=(
            "process only this recording run directory name; may be repeated "
            "(default: scan all recognized runs under every root)"
        ),
    )
    parser.add_argument(
        "--apply",
        action="store_true",
        help="perform permanent deletion; without this flag only preview",
    )
    parser.add_argument(
        "--watch",
        action="store_true",
        help="keep scanning all recording roots for newly completed rounds",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=5.0,
        help="seconds between scans in --watch mode (default: 5)",
    )
    return parser.parse_args(argv)


def parse_strict_bool(value):
    normalized = str(value).strip().lower()
    if normalized == "true":
        return True
    if normalized == "false":
        return False
    return None


def parse_strict_int(value):
    try:
        return int(str(value).strip())
    except (TypeError, ValueError):
        return None


def parse_strict_float(value):
    try:
        result = float(str(value).strip())
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def log_run_name(photo_run_name):
    prefix, separator, seed = photo_run_name.rpartition("_seed")
    if not separator or not prefix.startswith("grasp_trials_") or not seed.isdigit():
        return None
    return prefix


def cone_log_run_name(video_run_name):
    prefix, separator, seed = video_run_name.rpartition("_seed")
    if (
        not separator
        or not prefix.startswith("end_to_end_cone_trials_")
        or not seed.isdigit()
    ):
        return None
    return prefix


def cone_stress_log_run_name(recording_run_name):
    if not re.fullmatch(r"cone_move_stress_\d{8}_\d{6}", recording_run_name):
        return None
    return recording_run_name


def fixed_cone_e2e_log_run_name(recording_run_name):
    if not re.fullmatch(
        r"fixed_cone_e2e_stress_\d{8}_\d{6}(?:_[A-Za-z0-9][A-Za-z0-9_.-]*)?",
        recording_run_name,
    ):
        return None
    return recording_run_name


def pre_navigation_log_run_name(recording_run_name):
    if not re.fullmatch(
        r"pre_navigation_trials_\d{8}_\d{6}_ros\d+", recording_run_name
    ):
        return None
    return recording_run_name


def load_trial_rows(csv_path):
    try:
        with csv_path.open("r", encoding="utf-8", newline="") as csv_file:
            reader = csv.DictReader(csv_file)
            if reader.fieldnames is None or "round" not in reader.fieldnames:
                raise CleanupError("{} is missing required column round".format(csv_path))
            rows = {}
            for row in reader:
                round_number = parse_strict_int(row.get("round"))
                if round_number is not None:
                    rows[round_number] = row
            return rows
    except OSError as exc:
        raise CleanupError("could not read {}: {}".format(csv_path, exc))


def recording_candidates(round_dir, filenames=RECORDING_FILES):
    return [
        round_dir / filename
        for filename in filenames
        if (round_dir / filename).is_file()
    ]


def verify_delete_targets(round_dir, candidates, allowed_filenames=RECORDING_FILES):
    round_root = round_dir.resolve(strict=True)
    verified = []
    for candidate in candidates:
        if candidate.name not in allowed_filenames:
            raise CleanupError("unexpected recording filename {}".format(candidate))
        resolved = candidate.resolve(strict=True)
        try:
            resolved.relative_to(round_root)
        except ValueError:
            raise CleanupError(
                "refusing to delete recording outside {}: {}".format(
                    round_root, resolved
                )
            )
        if not resolved.is_file():
            raise CleanupError("recording target is not a file: {}".format(resolved))
        verified.append(candidate)
    return verified


def base_result(dataset, run_dir, round_dir, candidates):
    match = ROUND_PATTERN.fullmatch(round_dir.name)
    return {
        "dataset": dataset,
        "run": run_dir.name,
        "round": int(match.group(1)) if match else None,
        "round_dir": str(round_dir),
        "eligible_for_recording_deletion": False,
        "reason": "not_evaluated",
        "recording_files": [str(path) for path in candidates],
        "recording_bytes": sum(path.stat().st_size for path in candidates),
        "deleted_files": [],
        "deleted_bytes": 0,
    }


def evaluate_round(run_dir, round_dir, row):
    """Evaluate the legacy seq35 world-state recording policy."""
    candidates = recording_candidates(round_dir)
    result = base_result("seq35_world", run_dir, round_dir, candidates)
    recognition_correct = (
        parse_strict_bool(row.get("recognition_correct")) if row else None
    )
    grasp_success = parse_strict_bool(row.get("success")) if row else None
    eligible = recognition_correct is True and grasp_success is True

    if row is None:
        reason = "trial_result_missing"
    elif recognition_correct is not True and grasp_success is not True:
        reason = "recognition_not_correct_and_grasp_not_successful"
    elif recognition_correct is not True:
        reason = "recognition_not_correct"
    elif grasp_success is not True:
        reason = "grasp_not_successful"
    else:
        reason = "correct_recognition_and_successful_grasp"

    result.update(
        {
            "recognition_correct": recognition_correct,
            "grasp_success": grasp_success,
            "eligible_for_recording_deletion": eligible,
            "reason": reason,
        }
    )
    return result


def load_json(path):
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None


def cone_recording_checks(round_dir, row):
    """Return ordered checks; every item must be true before deletion."""
    recording_path = round_dir / "gazebo_world_state.log"
    manifest_path = round_dir / "gazebo_world_recording.json"
    manifest = load_json(manifest_path)
    recording_size = (
        recording_path.stat().st_size if recording_path.is_file() else 0
    )
    expected_path = str(recording_path.resolve())
    manifest_output = manifest.get("recording_file") if manifest else None
    manifest_size = parse_strict_int(manifest.get("size_bytes")) if manifest else None
    csv_size = (
        parse_strict_int(row.get("gazebo_recording_size_bytes")) if row else None
    )

    return (
        (row is not None, "trial_result_missing"),
        (parse_strict_bool(row.get("success")) is True if row else False,
         "task_not_successful"),
        (row.get("status") == "task_completed" if row else False,
         "task_status_not_completed"),
        (parse_strict_int(row.get("completed_stage")) == 20 if row else False,
         "completed_stage_not_task_completed"),
        (parse_strict_int(row.get("error_code")) == 0 if row else False,
         "task_error_code_not_zero"),
        (parse_strict_bool(row.get("preceding_navigation_failure")) is False
         if row else False, "preceding_navigation_failed_or_unknown"),
        (parse_strict_bool(row.get("delivery_navigation_failure")) is False
         if row else False, "delivery_navigation_failed_or_unknown"),
        (parse_strict_bool(row.get("cone_collision")) is False if row else False,
         "cone_collision_or_unknown"),
        (row.get("cone_monitor_status") == "complete" if row else False,
         "cone_monitor_incomplete"),
        (row.get("contact_stream_status") in {"stopped", "disabled"}
         if row else False, "contact_stream_incomplete"),
        (row.get("gazebo_recording_status") == "complete" if row else False,
         "gazebo_recording_incomplete"),
        (manifest is not None, "gazebo_recording_manifest_invalid"),
        (manifest.get("status") == "complete" if manifest else False,
         "gazebo_recording_manifest_incomplete"),
        (manifest.get("task_id") == row.get("task_id")
         if manifest and row else False, "gazebo_recording_task_mismatch"),
        (row.get("gazebo_recording_path") == expected_path if row else False,
         "gazebo_recording_csv_path_mismatch"),
        (manifest_output == recording_path.name,
         "gazebo_recording_manifest_path_mismatch"),
        (recording_size > 0, "gazebo_recording_missing_or_empty"),
        (manifest_size == recording_size,
         "gazebo_recording_manifest_size_mismatch"),
        (csv_size == recording_size, "gazebo_recording_csv_size_mismatch"),
    )


def evaluate_cone_video_round(run_dir, round_dir, row):
    candidates = recording_candidates(round_dir, CONE_RECORDING_FILES)
    result = base_result("cone_zone_world", run_dir, round_dir, candidates)
    checks = cone_recording_checks(round_dir, row)
    failed = next((reason for passed, reason in checks if not passed), None)
    result.update(
        {
            "task_success": (
                parse_strict_bool(row.get("success")) if row else None
            ),
            "task_status": row.get("status") if row else None,
            "error_code": parse_strict_int(row.get("error_code")) if row else None,
            "preceding_navigation_failure": (
                parse_strict_bool(row.get("preceding_navigation_failure"))
                if row else None
            ),
            "delivery_navigation_failure": (
                parse_strict_bool(row.get("delivery_navigation_failure"))
                if row else None
            ),
            "cone_collision": (
                parse_strict_bool(row.get("cone_collision")) if row else None
            ),
            "cone_monitor_status": row.get("cone_monitor_status") if row else None,
            "contact_stream_status": (
                row.get("contact_stream_status") if row else None
            ),
            "gazebo_recording_status": (
                row.get("gazebo_recording_status") if row else None
            ),
            "eligible_for_recording_deletion": failed is None,
            "reason": failed or "round_completed_without_detected_errors",
        }
    )
    return result


def evaluate_fixed_cone_e2e_round(run_dir, round_dir, row):
    result = evaluate_cone_video_round(run_dir, round_dir, row)
    result["dataset"] = "fixed_cone_e2e_world"
    return result


def cone_stress_recording_checks(round_dir, row):
    """Require a clean delivery-only navigation before deleting its recording."""
    recording_path = round_dir / "gazebo_world_state.log"
    manifest_path = round_dir / "gazebo_world_recording.json"
    manifest = load_json(manifest_path)
    recording_size = (
        recording_path.stat().st_size if recording_path.is_file() else 0
    )
    expected_path = str(recording_path.resolve())
    manifest_output = manifest.get("recording_file") if manifest else None
    manifest_size = parse_strict_int(manifest.get("size_bytes")) if manifest else None
    csv_size = (
        parse_strict_int(row.get("gazebo_recording_size_bytes")) if row else None
    )
    return (
        (row is not None, "trial_result_missing"),
        (parse_strict_bool(row.get("success")) is True if row else False,
         "navigation_not_successful"),
        (row.get("status") == "navigation_completed" if row else False,
         "navigation_status_not_completed"),
        (parse_strict_int(row.get("completed_stage")) == 17 if row else False,
         "completed_stage_not_arrived_delivery"),
        (parse_strict_int(row.get("error_code")) == 0 if row else False,
         "navigation_error_code_not_zero"),
        (parse_strict_bool(row.get("navigation_failure")) is False
         if row else False, "navigation_failed_or_unknown"),
        (
            (
                parse_strict_bool(row.get("profile_switched")) is True
                or (
                    parse_strict_bool(row.get("profile_switching_required"))
                    is False
                    and row.get("planner_mode") == "fixed_teb"
                )
            )
            if row
            else False,
            "automatic_profile_switch_missing",
        ),
        (parse_strict_bool(row.get("cone_collision")) is False if row else False,
         "cone_collision_or_unknown"),
        (row.get("cone_monitor_status") == "complete" if row else False,
         "cone_monitor_incomplete"),
        (row.get("contact_stream_status") in {"stopped", "disabled"}
         if row else False, "contact_stream_incomplete"),
        (row.get("gazebo_recording_status") == "complete" if row else False,
         "gazebo_recording_incomplete"),
        (manifest is not None, "gazebo_recording_manifest_invalid"),
        (manifest.get("status") == "complete" if manifest else False,
         "gazebo_recording_manifest_incomplete"),
        (manifest.get("task_id") == row.get("task_id")
         if manifest and row else False, "gazebo_recording_task_mismatch"),
        (row.get("gazebo_recording_path") == expected_path if row else False,
         "gazebo_recording_csv_path_mismatch"),
        (manifest_output == recording_path.name,
         "gazebo_recording_manifest_path_mismatch"),
        (recording_size > 0, "gazebo_recording_missing_or_empty"),
        (manifest_size == recording_size,
         "gazebo_recording_manifest_size_mismatch"),
        (csv_size == recording_size, "gazebo_recording_csv_size_mismatch"),
    )


def evaluate_cone_stress_round(run_dir, round_dir, row):
    candidates = recording_candidates(round_dir, CONE_RECORDING_FILES)
    result = base_result("cone_move_stress_world", run_dir, round_dir, candidates)
    checks = cone_stress_recording_checks(round_dir, row)
    failed = next((reason for passed, reason in checks if not passed), None)
    result.update(
        {
            "navigation_success": (
                parse_strict_bool(row.get("success")) if row else None
            ),
            "navigation_status": row.get("status") if row else None,
            "navigation_failure": (
                parse_strict_bool(row.get("navigation_failure")) if row else None
            ),
            "profile_switched": (
                parse_strict_bool(row.get("profile_switched")) if row else None
            ),
            "planner_mode": row.get("planner_mode") if row else None,
            "profile_switching_required": (
                parse_strict_bool(row.get("profile_switching_required"))
                if row
                else None
            ),
            "cone_collision": (
                parse_strict_bool(row.get("cone_collision")) if row else None
            ),
            "cone_monitor_status": row.get("cone_monitor_status") if row else None,
            "contact_stream_status": (
                row.get("contact_stream_status") if row else None
            ),
            "gazebo_recording_status": (
                row.get("gazebo_recording_status") if row else None
            ),
            "eligible_for_recording_deletion": failed is None,
            "reason": failed or "round_completed_without_detected_errors",
        }
    )
    return result


def pre_navigation_recording_checks(round_dir, row):
    """Require a position-accepted seq35 arrival and a complete recording."""
    recording_path = round_dir / "gazebo_world_state.log"
    manifest_path = round_dir / "gazebo_world_recording.json"
    manifest = load_json(manifest_path)
    recording_size = (
        recording_path.stat().st_size if recording_path.is_file() else 0
    )
    expected_path = str(recording_path.resolve())
    manifest_output = manifest.get("recording_file") if manifest else None
    manifest_size = parse_strict_int(manifest.get("size_bytes")) if manifest else None
    csv_size = (
        parse_strict_int(row.get("gazebo_recording_size_bytes")) if row else None
    )
    completed_waypoints = (
        parse_strict_int(row.get("completed_waypoints")) if row else None
    )
    waypoint_count = parse_strict_int(row.get("waypoint_count")) if row else None
    final_error = (
        parse_strict_float(row.get("final_position_error_m")) if row else None
    )
    final_tolerance = (
        parse_strict_float(row.get("final_position_tolerance_m"))
        if row else None
    )
    start_condition = manifest.get("start_condition", {}) if manifest else {}
    return (
        (row is not None, "trial_result_missing"),
        (parse_strict_bool(row.get("success")) is True if row else False,
         "navigation_not_successful"),
        (row.get("status") == "navigation_completed" if row else False,
         "navigation_status_not_completed"),
        (parse_strict_int(row.get("error_code")) == 0 if row else False,
         "navigation_error_code_not_zero"),
        (parse_strict_int(row.get("server_error_code")) == 0
         if row else False, "navigation_server_error_code_not_zero"),
        (parse_strict_bool(row.get("yaw_alignment_required")) is False
         if row else False, "final_yaw_requirement_not_disabled"),
        (row.get("acceptance_mode") in {
            "position_tolerance", "move_base_succeeded"
         } if row else False, "seq35_acceptance_missing"),
        (waypoint_count is not None and waypoint_count > 0,
         "waypoint_count_invalid"),
        (completed_waypoints == waypoint_count,
         "route_waypoints_incomplete"),
        (final_error is not None and final_tolerance is not None
         and final_tolerance > 0.0 and final_error <= final_tolerance + 1.0e-9,
         "seq35_position_tolerance_not_met"),
        (row.get("gazebo_recording_status") == "complete" if row else False,
         "gazebo_recording_incomplete"),
        (manifest is not None, "gazebo_recording_manifest_invalid"),
        (manifest.get("status") == "complete" if manifest else False,
         "gazebo_recording_manifest_incomplete"),
        (start_condition.get("type") == "immediate",
         "gazebo_recording_start_condition_invalid"),
        (manifest.get("task_id") == row.get("task_id")
         if manifest and row else False, "gazebo_recording_task_mismatch"),
        (row.get("gazebo_recording_path") == expected_path if row else False,
         "gazebo_recording_csv_path_mismatch"),
        (manifest_output == recording_path.name,
         "gazebo_recording_manifest_path_mismatch"),
        (recording_size > 0, "gazebo_recording_missing_or_empty"),
        (manifest_size == recording_size,
         "gazebo_recording_manifest_size_mismatch"),
        (csv_size == recording_size, "gazebo_recording_csv_size_mismatch"),
    )


def evaluate_pre_navigation_round(run_dir, round_dir, row):
    candidates = recording_candidates(round_dir, RECORDING_FILES)
    result = base_result("pre_navigation_world", run_dir, round_dir, candidates)
    checks = pre_navigation_recording_checks(round_dir, row)
    failed = next((reason for passed, reason in checks if not passed), None)
    result.update(
        {
            "navigation_success": (
                parse_strict_bool(row.get("success")) if row else None
            ),
            "navigation_status": row.get("status") if row else None,
            "error_code": parse_strict_int(row.get("error_code")) if row else None,
            "server_error_code": (
                parse_strict_int(row.get("server_error_code"))
                if row else None
            ),
            "acceptance_mode": row.get("acceptance_mode") if row else None,
            "yaw_alignment_required": (
                parse_strict_bool(row.get("yaw_alignment_required"))
                if row else None
            ),
            "final_position_error_m": (
                parse_strict_float(row.get("final_position_error_m"))
                if row else None
            ),
            "final_position_tolerance_m": (
                parse_strict_float(row.get("final_position_tolerance_m"))
                if row else None
            ),
            "gazebo_recording_status": (
                row.get("gazebo_recording_status") if row else None
            ),
            "eligible_for_recording_deletion": failed is None,
            "reason": failed or "seq35_navigation_completed_without_errors",
        }
    )
    return result


def selected_run_dirs(data_root, requested_runs, name_parser=log_run_name):
    if requested_runs:
        return [data_root / name for name in requested_runs]
    return sorted(
        path
        for path in data_root.iterdir()
        if path.is_dir() and name_parser(path.name) is not None
    )


def process_root(
    data_root,
    logs_root,
    requested_runs,
    apply,
    name_parser,
    evaluator,
    allowed_filenames,
    emit,
):
    results = []
    for run_dir in selected_run_dirs(data_root, requested_runs, name_parser):
        if not run_dir.is_dir():
            if emit:
                print("[KEEP] {}: run directory missing".format(run_dir))
            continue
        resolved_run_dir = run_dir.resolve(strict=True)
        try:
            resolved_run_dir.relative_to(data_root)
        except ValueError:
            if emit:
                print("[KEEP] {}: run resolves outside data root".format(run_dir))
            continue
        run_dir = resolved_run_dir
        source_log_run = name_parser(run_dir.name)
        if source_log_run is None:
            if emit:
                print("[KEEP] {}: unrecognized run name".format(run_dir))
            continue
        trials_csv = logs_root / source_log_run / "trials.csv"
        if not trials_csv.is_file() and (run_dir / "trials.csv").is_file():
            # Newer runners mirror their result table beside the recordings so
            # cleanup remains possible when --log-dir uses a custom path.
            trials_csv = run_dir / "trials.csv"
        try:
            trial_rows = load_trial_rows(trials_csv)
        except CleanupError as exc:
            if emit:
                print("[KEEP] {}: {}".format(run_dir.name, exc))
            trial_rows = {}

        round_dirs = sorted(
            path for path in run_dir.iterdir()
            if path.is_dir() and ROUND_PATTERN.fullmatch(path.name)
        )
        for round_dir in round_dirs:
            resolved_round_dir = round_dir.resolve(strict=True)
            try:
                resolved_round_dir.relative_to(run_dir)
            except ValueError:
                if emit:
                    print("[KEEP] {}/{}: round resolves outside run".format(
                        run_dir.name, round_dir.name))
                continue
            round_dir = resolved_round_dir
            round_number = int(ROUND_PATTERN.fullmatch(round_dir.name).group(1))
            if not any((round_dir / name).is_file() for name in allowed_filenames):
                continue
            result = evaluator(run_dir, round_dir, trial_rows.get(round_number))
            candidates = [Path(path) for path in result["recording_files"]]
            label = "{}/{}/{}".format(
                result["dataset"], run_dir.name, round_dir.name
            )
            if result["eligible_for_recording_deletion"]:
                action = "DELETE" if apply else "WOULD DELETE"
                if emit:
                    print("[{}] {}: {} files, {:.2f} MiB".format(
                        action, label, len(candidates),
                        result["recording_bytes"] / (1024.0 * 1024.0)))
                if apply and candidates:
                    verified = verify_delete_targets(
                        round_dir, candidates, allowed_filenames
                    )
                    for candidate in verified:
                        size = candidate.stat().st_size
                        candidate.unlink()
                        result["deleted_files"].append(str(candidate))
                        result["deleted_bytes"] += size
            else:
                if emit:
                    print("[KEEP] {}: {}".format(label, result["reason"]))
            results.append(result)
    return results


def process(args, emit=True):
    data_root = args.data_root.expanduser().resolve()
    cone_root_arg = getattr(args, "cone_video_root", None)
    cone_root = cone_root_arg.expanduser().resolve() if cone_root_arg else None
    stress_root_arg = getattr(args, "cone_stress_root", None)
    stress_root = (
        stress_root_arg.expanduser().resolve() if stress_root_arg else None
    )
    pre_navigation_root_arg = getattr(args, "pre_navigation_root", None)
    pre_navigation_root = (
        pre_navigation_root_arg.expanduser().resolve()
        if pre_navigation_root_arg else None
    )
    fixed_cone_e2e_root_arg = getattr(args, "fixed_cone_e2e_root", None)
    fixed_cone_e2e_root = (
        fixed_cone_e2e_root_arg.expanduser().resolve()
        if fixed_cone_e2e_root_arg
        else None
    )
    logs_root = args.logs_root.expanduser().resolve()
    if not logs_root.is_dir():
        raise CleanupError("logs root does not exist: {}".format(logs_root))
    available_roots = [
        root
        for root in (
            data_root,
            cone_root,
            stress_root,
            pre_navigation_root,
            fixed_cone_e2e_root,
        )
        if root and root.is_dir()
    ]
    if not available_roots:
        raise CleanupError(
            "no recording root exists (checked {}, {}, {}, {}, and {})".format(
                data_root,
                cone_root or "disabled",
                stress_root or "disabled",
                pre_navigation_root or "disabled",
                fixed_cone_e2e_root or "disabled",
            )
        )

    results = []
    if data_root.is_dir():
        results.extend(process_root(
            data_root, logs_root, args.runs, args.apply, log_run_name,
            evaluate_round, RECORDING_FILES, emit))
    if cone_root is not None and cone_root.is_dir():
        results.extend(process_root(
            cone_root, logs_root, args.runs, args.apply, cone_log_run_name,
            evaluate_cone_video_round, CONE_RECORDING_FILES, emit))
    if stress_root is not None and stress_root.is_dir():
        results.extend(process_root(
            stress_root, logs_root, args.runs, args.apply,
            cone_stress_log_run_name, evaluate_cone_stress_round,
            CONE_RECORDING_FILES, emit))
    if pre_navigation_root is not None and pre_navigation_root.is_dir():
        results.extend(process_root(
            pre_navigation_root, logs_root, args.runs, args.apply,
            pre_navigation_log_run_name, evaluate_pre_navigation_round,
            RECORDING_FILES, emit))
    if fixed_cone_e2e_root is not None and fixed_cone_e2e_root.is_dir():
        results.extend(process_root(
            fixed_cone_e2e_root, logs_root, args.runs, args.apply,
            fixed_cone_e2e_log_run_name, evaluate_fixed_cone_e2e_round,
            CONE_RECORDING_FILES, emit))
    report_root = None
    if args.runs:
        # A targeted cleanup report belongs beside the selected run.  This is
        # especially important when all legacy/default roots exist: choosing a
        # fixed priority would otherwise place the audit in an unrelated root.
        report_root = next(
            (
                root
                for root in (
                    fixed_cone_e2e_root,
                    pre_navigation_root,
                    stress_root,
                    cone_root,
                    data_root,
                )
                if root is not None
                and root.is_dir()
                and any((root / run_name).is_dir() for run_name in args.runs)
            ),
            None,
        )
    if report_root is None:
        report_root = next(
            root
            for root in (
                stress_root,
                cone_root,
                data_root,
                pre_navigation_root,
                fixed_cone_e2e_root,
            )
            if root is not None and root.is_dir()
        )
    return report_root, results


def summarize(results):
    eligible = [r for r in results if r["eligible_for_recording_deletion"]]
    retained = [r for r in results if not r["eligible_for_recording_deletion"]]
    return {
        "rounds_examined": len(results),
        "eligible_rounds": len(eligible),
        "retained_rounds": len(retained),
        "eligible_bytes": sum(r["recording_bytes"] for r in eligible),
        "deleted_files": sum(len(r["deleted_files"]) for r in results),
        "deleted_bytes": sum(r["deleted_bytes"] for r in results),
    }


def write_report(data_root, results, summary):
    timestamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    report_path = data_root / "gazebo_recording_cleanup_{}.json".format(timestamp)
    payload = {
        "created_at": dt.datetime.now().astimezone().isoformat(timespec="seconds"),
        "summary": summary,
        "rounds": results,
    }
    report_path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    return report_path


def watch_signature(result):
    return (
        result["eligible_for_recording_deletion"],
        result["reason"],
        tuple(result["recording_files"]),
    )


def print_watch_result(result, previous_signature, apply):
    label = "{}/{}/round_{:03d}".format(
        result["dataset"], result["run"], result["round"]
    )
    if result["deleted_files"]:
        print("[DELETED] {}: {} files, freed {:.2f} MiB".format(
            label, len(result["deleted_files"]),
            result["deleted_bytes"] / (1024.0 * 1024.0)), flush=True)
        return
    if previous_signature == watch_signature(result):
        return
    if result["eligible_for_recording_deletion"]:
        if result["recording_files"] and not apply:
            print("[WOULD DELETE] {}: {} recording files".format(
                label, len(result["recording_files"])), flush=True)
        elif not result["recording_files"]:
            print("[CLEAN] {}: no recording files remain".format(label), flush=True)
        return
    print("[KEEP] {}: {}".format(label, result["reason"]), flush=True)


def watch(args):
    if args.interval <= 0.0:
        raise CleanupError("--interval must be positive")
    logs_root = args.logs_root.expanduser().resolve()
    if not logs_root.is_dir():
        raise CleanupError("logs root does not exist: {}".format(logs_root))
    lock_path = logs_root / ".gazebo_recording_cleanup_watcher.lock"
    lock_file = lock_path.open("a+", encoding="utf-8")
    try:
        fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        lock_file.seek(0)
        active_pid = lock_file.read().strip() or "unknown"
        lock_file.close()
        raise CleanupError(
            "another cleanup watcher is already running (pid={})".format(active_pid)
        )
    lock_file.seek(0)
    lock_file.truncate()
    lock_file.write(str(os.getpid()) + "\n")
    lock_file.flush()

    known = {}
    cumulative_deleted_files = 0
    cumulative_deleted_bytes = 0
    mode = "DELETE" if args.apply else "DRY RUN"
    try:
        print(
            "Watching legacy, cone-video, cone-stress, pre-navigation, and "
            "fixed-cone end-to-end "
            "roots every {:.1f}s "
            "(mode={}, pid={}). Press Ctrl-C to stop.".format(
                args.interval, mode, os.getpid()), flush=True)
        try:
            while True:
                try:
                    report_root, results = process(args, emit=False)
                except (CleanupError, OSError) as exc:
                    print("[ERROR] scan failed; will retry: {}".format(exc), flush=True)
                    time.sleep(args.interval)
                    continue
                deleted_this_scan = [r for r in results if r["deleted_files"]]
                for result in results:
                    key = (result["dataset"], result["run"], result["round"])
                    previous = known.get(key)
                    print_watch_result(result, previous, args.apply)
                    known[key] = watch_signature(result)
                if deleted_this_scan:
                    deletion_summary = summarize(deleted_this_scan)
                    cumulative_deleted_files += deletion_summary["deleted_files"]
                    cumulative_deleted_bytes += deletion_summary["deleted_bytes"]
                    try:
                        report_path = write_report(
                            report_root, deleted_this_scan, deletion_summary)
                        print("[REPORT] {}".format(report_path), flush=True)
                    except OSError as exc:
                        print("[ERROR] deletion audit report failed: {}".format(exc),
                              flush=True)
                time.sleep(args.interval)
        except KeyboardInterrupt:
            print("\nWatcher stopped: deleted_files={} freed={:.2f} MiB".format(
                cumulative_deleted_files,
                cumulative_deleted_bytes / (1024.0 * 1024.0)))
            return 0
    finally:
        fcntl.flock(lock_file.fileno(), fcntl.LOCK_UN)
        lock_file.close()


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    try:
        if args.watch:
            return watch(args)
        report_root, results = process(args)
        summary = summarize(results)
        mode = "APPLY" if args.apply else "DRY RUN"
        print("\n===== Gazebo recording cleanup ({}) =====".format(mode))
        print("examined={} eligible={} retained={} eligible_size={:.2f} MiB".format(
            summary["rounds_examined"], summary["eligible_rounds"],
            summary["retained_rounds"],
            summary["eligible_bytes"] / (1024.0 * 1024.0)))
        if args.apply:
            report_path = write_report(report_root, results, summary)
            print("deleted_files={} freed={:.2f} MiB".format(
                summary["deleted_files"],
                summary["deleted_bytes"] / (1024.0 * 1024.0)))
            print("report={}".format(report_path))
        else:
            print("No files were deleted. Re-run with --apply to delete.")
        return 0
    except (CleanupError, OSError) as exc:
        print("prune_successful_gazebo_recordings: {}".format(exc), file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
