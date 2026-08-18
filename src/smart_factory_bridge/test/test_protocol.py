#!/usr/bin/env python3
"""Protocol tests: encoding, validation, fingerprints, message builders."""

import json
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from smart_factory_bridge import protocol
from smart_factory_bridge.protocol import ProtocolError

VALID_REQUEST = {
    "schema_version": 1,
    "message_type": "request",
    "session_id": "car-uuid-1",
    "request_id": "order-7:simulation:abc",
    "order_id": "order-7",
    "task_type": "gazebo_pick_and_place",
    "target_class": 2,
    "simulation_product": "电脑",
    "simulation_category": "electronics",
    "simulation_warehouse": "电子产品生产车间",
    "physical_station": "C",
    "issued_at": 1234.0,
    "timestamp": 0.0,
}


class EncodeDecodeTest(unittest.TestCase):
    def test_round_trip(self):
        payload = protocol.make_message(
            "heartbeat", "sim-1", ready=True, busy=False
        )
        decoded = protocol.decode_line(protocol.encode(payload))
        self.assertEqual(decoded["message_type"], "heartbeat")
        self.assertEqual(decoded["session_id"], "sim-1")
        self.assertEqual(decoded["ready"], True)
        self.assertEqual(decoded["schema_version"], 1)

    def test_overlong_encode_rejected(self):
        with self.assertRaises(ProtocolError) as ctx:
            protocol.encode({"x": "y" * protocol.MAX_MESSAGE_BYTES})
        self.assertEqual(ctx.exception.error_code, protocol.ERR_MESSAGE_TOO_LONG)

    def test_overlong_decode_rejected(self):
        raw = b"x" * (protocol.MAX_MESSAGE_BYTES + 1) + b"\n"
        with self.assertRaises(ProtocolError) as ctx:
            protocol.decode_line(raw.rstrip(b"\n"))
        self.assertEqual(ctx.exception.error_code, protocol.ERR_MESSAGE_TOO_LONG)

    def test_invalid_json_rejected(self):
        with self.assertRaises(ProtocolError) as ctx:
            protocol.decode_line(b"{not json}")
        self.assertEqual(ctx.exception.error_code, protocol.ERR_INVALID_JSON)

    def test_non_object_json_rejected(self):
        with self.assertRaises(ProtocolError) as ctx:
            protocol.decode_line(b"[1, 2, 3]")
        self.assertEqual(ctx.exception.error_code, protocol.ERR_INVALID_JSON)

    def test_bad_schema_version_rejected(self):
        raw = json.dumps(
            {"schema_version": 2, "message_type": "heartbeat",
             "session_id": "s", "timestamp": 0.0}
        ).encode("utf-8")
        with self.assertRaises(ProtocolError) as ctx:
            protocol.decode_line(raw)
        self.assertEqual(ctx.exception.error_code, protocol.ERR_BAD_SCHEMA_VERSION)

    def test_unknown_message_type_rejected(self):
        raw = json.dumps(
            {"schema_version": 1, "message_type": "nonsense",
             "session_id": "s", "timestamp": 0.0}
        ).encode("utf-8")
        with self.assertRaises(ProtocolError) as ctx:
            protocol.decode_line(raw)
        self.assertEqual(
            ctx.exception.error_code, protocol.ERR_UNKNOWN_MESSAGE_TYPE
        )

    def test_missing_session_id_rejected(self):
        raw = json.dumps(
            {"schema_version": 1, "message_type": "heartbeat",
             "timestamp": 0.0}
        ).encode("utf-8")
        with self.assertRaises(ProtocolError) as ctx:
            protocol.decode_line(raw)
        self.assertEqual(ctx.exception.error_code, protocol.ERR_MISSING_FIELD)


class ValidateRequestTest(unittest.TestCase):
    def test_valid_request_passes(self):
        self.assertIsNotNone(protocol.validate_request(dict(VALID_REQUEST)))

    def test_missing_field_rejected(self):
        payload = dict(VALID_REQUEST)
        del payload["simulation_warehouse"]
        with self.assertRaises(ProtocolError) as ctx:
            protocol.validate_request(payload)
        self.assertEqual(ctx.exception.error_code, protocol.ERR_MISSING_FIELD)

    def test_wrong_type_rejected(self):
        payload = dict(VALID_REQUEST)
        payload["target_class"] = "2"
        with self.assertRaises(ProtocolError) as ctx:
            protocol.validate_request(payload)
        self.assertEqual(ctx.exception.error_code, protocol.ERR_MISSING_FIELD)

    def test_unknown_class_rejected(self):
        payload = dict(VALID_REQUEST)
        payload["target_class"] = 7
        payload["simulation_category"] = "electronics"
        with self.assertRaises(ProtocolError) as ctx:
            protocol.validate_request(payload)
        self.assertEqual(ctx.exception.error_code, protocol.ERR_INVALID_CLASS)

    def test_category_mismatch_rejected(self):
        payload = dict(VALID_REQUEST)
        payload["simulation_category"] = "food"
        with self.assertRaises(ProtocolError) as ctx:
            protocol.validate_request(payload)
        self.assertEqual(ctx.exception.error_code, protocol.ERR_INVALID_CLASS)

    def test_wrong_task_type_rejected(self):
        payload = dict(VALID_REQUEST)
        payload["task_type"] = "something_else"
        with self.assertRaises(ProtocolError) as ctx:
            protocol.validate_request(payload)
        self.assertEqual(
            ctx.exception.error_code, protocol.ERR_INVALID_TASK_TYPE
        )


class FingerprintTest(unittest.TestCase):
    def test_same_content_same_fingerprint(self):
        first = dict(VALID_REQUEST)
        second = dict(VALID_REQUEST)
        second["timestamp"] = 999.0
        second["issued_at"] = 0.0  # volatile fields must not matter
        self.assertEqual(
            protocol.request_fingerprint(first),
            protocol.request_fingerprint(second),
        )

    def test_different_content_different_fingerprint(self):
        first = dict(VALID_REQUEST)
        second = dict(VALID_REQUEST)
        second["target_class"] = 0
        second["simulation_category"] = "food"
        self.assertNotEqual(
            protocol.request_fingerprint(first),
            protocol.request_fingerprint(second),
        )


class MessageBuilderTest(unittest.TestCase):
    def setUp(self):
        self.request = dict(VALID_REQUEST)

    def test_ack_echoes_identity(self):
        ack = protocol.make_ack("sim-1", self.request)
        self.assertEqual(ack["message_type"], "ack")
        self.assertEqual(ack["state"], "accepted")
        self.assertEqual(ack["request_session_id"], "car-uuid-1")
        self.assertEqual(ack["request_id"], "order-7:simulation:abc")
        self.assertEqual(ack["order_id"], "order-7")

    def test_progress_echoes_and_stage_names(self):
        progress = protocol.make_progress(
            "sim-1", self.request, stage=15, detail="going", retry_count=1
        )
        self.assertEqual(progress["stage"], 15)
        self.assertEqual(progress["stage_name"], "GET_DELIVERY_GOAL")
        self.assertEqual(progress["detail"], "going")
        self.assertEqual(progress["retry_count"], 1)
        self.assertEqual(progress["request_session_id"], "car-uuid-1")

        self.assertEqual(
            protocol.make_progress("sim-1", self.request, 16)["stage_name"],
            "NAVIGATE_TO_DELIVERY",
        )
        self.assertEqual(
            protocol.make_progress("sim-1", self.request, 18)["stage_name"],
            "RELEASE_OBJECT",
        )
        self.assertEqual(
            protocol.make_progress("sim-1", self.request, 20)["stage_name"],
            "TASK_COMPLETED",
        )
        self.assertEqual(
            protocol.make_progress("sim-1", self.request, 250)["stage_name"],
            "TASK_FAILED",
        )

    def test_result_completed_only_at_stage_20(self):
        result = protocol.make_result(
            "sim-1", self.request, success=True,
            completed_stage=protocol.STAGE_TASK_COMPLETED,
        )
        self.assertEqual(result["state"], "completed")
        self.assertTrue(result["success"])
        self.assertEqual(result["completed_stage"], 20)
        self.assertEqual(result["completed_stage_name"], "TASK_COMPLETED")

    def test_result_stage_14_is_failed(self):
        result = protocol.make_result(
            "sim-1", self.request, success=True, completed_stage=14
        )
        self.assertEqual(result["state"], "failed")
        self.assertFalse(result["success"])

    def test_result_failed_flag_is_failed_even_at_20(self):
        result = protocol.make_result(
            "sim-1", self.request, success=False,
            completed_stage=protocol.STAGE_TASK_COMPLETED,
            error_code=9,
        )
        self.assertEqual(result["state"], "failed")
        self.assertEqual(result["error_code"], 9)

    def test_result_echoes_identity_and_rehearsal_false(self):
        result = protocol.make_result(
            "sim-1", self.request, success=True,
            completed_stage=protocol.STAGE_TASK_COMPLETED,
        )
        self.assertEqual(result["request_session_id"], "car-uuid-1")
        self.assertEqual(result["request_id"], "order-7:simulation:abc")
        self.assertEqual(result["order_id"], "order-7")
        self.assertEqual(result["target_class"], 2)
        self.assertIs(result["rehearsal"], False)
        self.assertEqual(result["request_issued_at"], 1234.0)

    def test_error_echoes_identity_and_reason(self):
        error = protocol.make_error(
            "sim-1", self.request, protocol.ERR_REQUEST_CONFLICT,
            "same request_id with different content",
        )
        self.assertEqual(error["message_type"], "error")
        self.assertEqual(error["error_code"], protocol.ERR_REQUEST_CONFLICT)
        self.assertEqual(error["reason"], "request_conflict")
        self.assertEqual(error["request_session_id"], "car-uuid-1")
        self.assertEqual(error["request_id"], "order-7:simulation:abc")

    def test_error_without_request(self):
        error = protocol.make_error(
            "sim-1", None, protocol.ERR_INVALID_JSON, "invalid JSON"
        )
        self.assertEqual(error["request_id"], None)


class TerminalSuccessTest(unittest.TestCase):
    def test_terminal_success_requires_all_conditions(self):
        ok = {
            "message_type": "result",
            "state": "completed",
            "success": True,
            "completed_stage": 20,
        }
        self.assertTrue(protocol.is_terminal_success(ok))
        bad = [
            dict(ok, state="failed"),
            dict(ok, success=False),
            dict(ok, completed_stage=14),
            dict(ok, message_type="progress"),
        ]
        for payload in bad:
            self.assertFalse(protocol.is_terminal_success(payload))


if __name__ == "__main__":
    unittest.main()
