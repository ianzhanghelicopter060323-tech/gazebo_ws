"""Frozen NDJSON protocol between the vehicle and the simulation bridge.

Implemented from `docs/Gazebo仿真双机通信接口开发任务书.md` section 6
("网络协议冻结版") together with the local deviation record in
`docs/Gazebo仿真双机通信接口开发任务书_本机偏差勘误.md`:

- one UTF-8 JSON object per line, terminated by ``\\n`` (NDJSON)
- ``schema_version=1``
- every message carries ``schema_version``, ``message_type``,
  ``session_id`` and ``timestamp``
- the bridge answers request/ack/progress/result and echoes the vehicle's
  ``request_session_id`` + ``request_id`` + ``order_id`` unchanged
- progress format (stage / stage_name / retry_count / detail) is frozen by
  the deviation record section 6

This module is deliberately free of ROS imports so it can be unit tested
on a desktop without a master.
"""

from __future__ import absolute_import

import json
import time

SCHEMA_VERSION = 1

# Single message size limit from the task book section 6.1.
MAX_MESSAGE_BYTES = 64 * 1024

MESSAGE_TYPES = ("heartbeat", "request", "ack", "progress", "result", "error")

# task_type accepted from the vehicle for a pick-and-place simulation.
TASK_TYPE_PICK_AND_PLACE = "gazebo_pick_and_place"

# target_class <-> simulation_category consistency, mirrors
# ExecuteTask.action FOOD/DAILY/ELECTRONICS and the task book section 6.4.
TARGET_CLASS_CATEGORIES = {
    0: "food",
    1: "daily",
    2: "electronics",
}

# Frozen stage names of ExecuteTask.action feedback (deviation record
# section 3: no stage 19, release verification folded into stage 18).
STAGE_NAMES = {
    0: "IDLE",
    1: "ACCEPT_TASK",
    2: "VALIDATE_TASK",
    3: "CHECK_LOCALIZATION",
    4: "GET_PICKUP_STAGING_GOAL",
    5: "NAVIGATE_TO_PICKUP_STAGING",
    6: "ARRIVED_PICKUP_STAGING",
    7: "OBSERVE_PICKUP_CANDIDATE",
    8: "NAVIGATE_TO_PICKUP_CANDIDATE",
    9: "LOCALIZE_TARGET",
    10: "ALIGN_FOR_GRASP",
    11: "OPEN_GRIPPER",
    12: "GRASP_OBJECT",
    13: "VERIFY_GRASP",
    14: "OBJECT_GRASPED",
    15: "GET_DELIVERY_GOAL",
    16: "NAVIGATE_TO_DELIVERY",
    17: "ARRIVED_DELIVERY",
    18: "RELEASE_OBJECT",
    20: "TASK_COMPLETED",
    250: "TASK_FAILED",
}

# Stage that proves the mission really completed: object released into the
# correct warehouse (task book section 6.6, handover section 4.3).
STAGE_TASK_COMPLETED = 20
STAGE_TASK_FAILED = 250

# Bridge protocol error codes. These describe protocol-level failures and
# are independent from the mission's ExecuteTaskResult.error_code values.
ERR_OK = 0
ERR_INVALID_JSON = 1
ERR_MESSAGE_TOO_LONG = 2
ERR_MISSING_FIELD = 3
ERR_BAD_SCHEMA_VERSION = 4
ERR_UNKNOWN_MESSAGE_TYPE = 5
ERR_INVALID_TASK_TYPE = 6
ERR_INVALID_CLASS = 7
ERR_REQUEST_CONFLICT = 8
ERR_BUSY = 9
ERR_INTERNAL = 10

ERROR_NAMES = {
    ERR_OK: "OK",
    ERR_INVALID_JSON: "invalid_json",
    ERR_MESSAGE_TOO_LONG: "message_too_long",
    ERR_MISSING_FIELD: "missing_field",
    ERR_BAD_SCHEMA_VERSION: "bad_schema_version",
    ERR_UNKNOWN_MESSAGE_TYPE: "unknown_message_type",
    ERR_INVALID_TASK_TYPE: "invalid_task_type",
    ERR_INVALID_CLASS: "invalid_class",
    ERR_REQUEST_CONFLICT: "request_conflict",
    ERR_BUSY: "busy",
    ERR_INTERNAL: "internal_error",
}

# Fields that define the task content for request deduplication. Volatile
# bookkeeping fields (timestamp, issued_at) are excluded so that a plain
# resend of the same request still fingerprints identically.
REQUEST_FINGERPRINT_FIELDS = (
    "task_type",
    "target_class",
    "simulation_product",
    "simulation_category",
    "simulation_warehouse",
    "physical_station",
    "order_id",
)

# Fields required on a request, with their expected python types.
REQUEST_REQUIRED_FIELDS = (
    ("schema_version", int),
    ("message_type", str),
    ("session_id", str),
    ("request_id", str),
    ("order_id", str),
    ("task_type", str),
    ("target_class", int),
    ("simulation_product", str),
    ("simulation_category", str),
    ("simulation_warehouse", str),
    ("physical_station", str),
)


class ProtocolError(ValueError):
    """A decoded message failed protocol validation.

    ``error_code`` is one of the ``ERR_*`` constants and is safe to put
    into an ``error`` reply.
    """

    def __init__(self, error_code, message):
        super(ProtocolError, self).__init__(message)
        self.error_code = error_code
        self.message = message


def now():
    """Wall-clock timestamp for protocol logging (no ROS dependency)."""
    return time.time()


def make_message(message_type, session_id, **fields):
    """Build a message with the frozen common fields."""
    payload = {
        "schema_version": SCHEMA_VERSION,
        "message_type": message_type,
        "session_id": session_id,
        "timestamp": now(),
    }
    payload.update(fields)
    return payload


def encode(payload):
    """Encode a message to one NDJSON line, enforcing the size limit."""
    line = json.dumps(payload, ensure_ascii=False, sort_keys=True)
    raw = line.encode("utf-8")
    if len(raw) > MAX_MESSAGE_BYTES:
        raise ProtocolError(
            ERR_MESSAGE_TOO_LONG,
            "encoded message %d bytes exceeds %d"
            % (len(raw), MAX_MESSAGE_BYTES),
        )
    return raw + b"\n"


def decode_line(raw):
    """Decode one NDJSON line with strict protocol validation.

    Raises :class:`ProtocolError` for oversized lines, invalid JSON,
    unknown message types and bad schema versions. Returns the payload
    dict on success.
    """
    if len(raw) > MAX_MESSAGE_BYTES:
        raise ProtocolError(
            ERR_MESSAGE_TOO_LONG,
            "received line %d bytes exceeds %d"
            % (len(raw), MAX_MESSAGE_BYTES),
        )
    try:
        payload = json.loads(raw.decode("utf-8"))
    except (ValueError, UnicodeDecodeError):
        raise ProtocolError(ERR_INVALID_JSON, "invalid JSON line")
    if not isinstance(payload, dict):
        raise ProtocolError(ERR_INVALID_JSON, "JSON payload is not an object")
    if payload.get("schema_version") != SCHEMA_VERSION:
        raise ProtocolError(
            ERR_BAD_SCHEMA_VERSION,
            "unsupported schema_version %r"
            % payload.get("schema_version"),
        )
    message_type = payload.get("message_type")
    if message_type not in MESSAGE_TYPES:
        raise ProtocolError(
            ERR_UNKNOWN_MESSAGE_TYPE,
            "unknown message_type %r" % (message_type,),
        )
    if not isinstance(payload.get("session_id"), str):
        raise ProtocolError(
            ERR_MISSING_FIELD,
            "missing or non-string session_id",
        )
    return payload


def _require_fields(payload, fields):
    for name, expected_type in fields:
        value = payload.get(name)
        if value is None or not isinstance(value, expected_type):
            raise ProtocolError(
                ERR_MISSING_FIELD,
                "missing or wrong-typed field %r" % (name,),
            )


def validate_request(payload):
    """Validate a request message and the category/class consistency."""
    _require_fields(payload, REQUEST_REQUIRED_FIELDS)
    if payload["message_type"] != "request":
        raise ProtocolError(
            ERR_UNKNOWN_MESSAGE_TYPE,
            "expected message_type=request, got %r"
            % (payload["message_type"],),
        )
    if payload["task_type"] != TASK_TYPE_PICK_AND_PLACE:
        raise ProtocolError(
            ERR_INVALID_TASK_TYPE,
            "unsupported task_type %r" % (payload["task_type"],),
        )
    target_class = payload["target_class"]
    category = TARGET_CLASS_CATEGORIES.get(target_class)
    if category is None:
        raise ProtocolError(
            ERR_INVALID_CLASS,
            "unknown target_class %r" % (target_class,),
        )
    if payload["simulation_category"] != category:
        raise ProtocolError(
            ERR_INVALID_CLASS,
            "target_class %r does not match simulation_category %r"
            % (target_class, payload["simulation_category"]),
        )
    return payload


def request_fingerprint(payload):
    """Canonical content fingerprint used for request deduplication.

    Two requests with the same ``request_id`` must have equal fingerprints
    to be treated as the same request.
    """
    return json.dumps(
        {name: payload.get(name) for name in REQUEST_FINGERPRINT_FIELDS},
        ensure_ascii=False,
        sort_keys=True,
    )


def make_ack(session_id, request):
    """Ack: bridge accepted the request and submitted the Action goal."""
    return make_message(
        "ack",
        session_id,
        request_session_id=request["session_id"],
        request_id=request["request_id"],
        order_id=request["order_id"],
        state="accepted",
    )


def make_progress(session_id, request, stage, detail="", retry_count=0):
    """Progress: one Action feedback mapped to a protocol message."""
    return make_message(
        "progress",
        session_id,
        request_session_id=request["session_id"],
        request_id=request["request_id"],
        order_id=request["order_id"],
        stage=stage,
        stage_name=STAGE_NAMES.get(stage, "UNKNOWN"),
        retry_count=retry_count,
        detail=detail or "",
    )


def make_result(
    session_id,
    request,
    success,
    completed_stage,
    error_code=ERR_OK,
    message="",
):
    """Final result; ``state`` is derived from ``success`` and the stage.

    Per the task book section 6.6 the bridge reports ``completed`` only
    when the Action really succeeded with ``completed_stage == 20``.
    """
    terminal_success = bool(success) and completed_stage == STAGE_TASK_COMPLETED
    return make_message(
        "result",
        session_id,
        request_session_id=request["session_id"],
        request_id=request["request_id"],
        order_id=request["order_id"],
        target_class=request["target_class"],
        state="completed" if terminal_success else "failed",
        success=terminal_success,
        completed_stage=completed_stage,
        completed_stage_name=STAGE_NAMES.get(completed_stage, "UNKNOWN"),
        rehearsal=False,
        error_code=error_code,
        message=message or "",
        request_issued_at=request.get("issued_at", 0.0),
    )


def make_error(
    session_id,
    request,
    error_code,
    message,
    reason=None,
):
    """Error reply; echoes the request identity when available."""
    request_session_id = None
    request_id = None
    order_id = None
    if isinstance(request, dict):
        request_session_id = request.get("session_id")
        request_id = request.get("request_id")
        order_id = request.get("order_id")
    return make_message(
        "error",
        session_id,
        request_session_id=request_session_id,
        request_id=request_id,
        order_id=order_id,
        error_code=error_code,
        reason=reason or ERROR_NAMES.get(error_code, "error"),
        message=message or "",
    )


def is_terminal_success(result):
    """True only when the frozen success condition of section 4.3 holds."""
    return (
        result.get("message_type") == "result"
        and result.get("state") == "completed"
        and result.get("success") is True
        and result.get("completed_stage") == STAGE_TASK_COMPLETED
    )
