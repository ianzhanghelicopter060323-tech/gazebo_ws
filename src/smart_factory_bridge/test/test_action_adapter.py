#!/usr/bin/env python3
"""Action adapter tests: feedback/progress and terminal result mapping."""

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from smart_factory_bridge.action_adapter import (
    ActionAdapter,
    ACTION_ABORTED,
    ACTION_LOST,
    ACTION_PREEMPTED,
    ACTION_SUCCEEDED,
)
from smart_factory_bridge import protocol

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


class ActionAdapterTest(unittest.TestCase):
    def setUp(self):
        self.adapter = ActionAdapter("sim-uuid-1")
        self.request = dict(VALID_REQUEST)

    def test_ack(self):
        ack = self.adapter.ack(self.request)
        self.assertEqual(ack["message_type"], "ack")
        self.assertEqual(ack["state"], "accepted")
        self.assertEqual(ack["request_session_id"], "car-uuid-1")

    def test_progress_from_feedback(self):
        progress = self.adapter.progress(
            self.request,
            {"current_stage": 16, "retry_count": 2, "detail": "rolling"},
        )
        self.assertEqual(progress["message_type"], "progress")
        self.assertEqual(progress["stage"], 16)
        self.assertEqual(progress["stage_name"], "NAVIGATE_TO_DELIVERY")
        self.assertEqual(progress["retry_count"], 2)
        self.assertEqual(progress["detail"], "rolling")

    def test_succeeded_stage_20_is_completed(self):
        result = self.adapter.result(
            self.request,
            ACTION_SUCCEEDED,
            {
                "success": True,
                "completed_stage": 20,
                "error_code": 0,
                "message": "target object placed in the correct warehouse",
            },
        )
        self.assertEqual(result["state"], "completed")
        self.assertTrue(result["success"])
        self.assertEqual(result["completed_stage"], 20)
        self.assertEqual(result["completed_stage_name"], "TASK_COMPLETED")
        self.assertEqual(result["error_code"], 0)
        self.assertIs(result["rehearsal"], False)
        self.assertEqual(result["request_issued_at"], 1234.0)

    def test_succeeded_stage_14_must_not_pass(self):
        result = self.adapter.result(
            self.request,
            ACTION_SUCCEEDED,
            {"success": True, "completed_stage": 14,
             "error_code": 0, "message": ""},
        )
        self.assertEqual(result["state"], "failed")
        self.assertFalse(result["success"])
        self.assertEqual(result["completed_stage"], 14)

    def test_succeeded_without_success_flag_fails(self):
        result = self.adapter.result(
            self.request,
            ACTION_SUCCEEDED,
            {"success": False, "completed_stage": 20,
             "error_code": 13, "message": "grasp failed"},
        )
        self.assertEqual(result["state"], "failed")
        self.assertEqual(result["error_code"], 13)

    def test_aborted_passes_error_code(self):
        result = self.adapter.result(
            self.request,
            ACTION_ABORTED,
            {"success": False, "completed_stage": 16,
             "error_code": 6, "message": "navigation timeout"},
        )
        self.assertEqual(result["state"], "failed")
        self.assertEqual(result["error_code"], 6)
        self.assertEqual(result["completed_stage"], 16)

    def test_aborted_without_error_code_uses_internal(self):
        result = self.adapter.result(self.request, ACTION_ABORTED, None)
        self.assertEqual(result["state"], "failed")
        self.assertEqual(result["error_code"], 255)

    def test_preempted_uses_preempt_code(self):
        result = self.adapter.result(self.request, ACTION_PREEMPTED, None)
        self.assertEqual(result["state"], "failed")
        self.assertEqual(result["error_code"], 8)
        self.assertIn("preempted", result["message"])

    def test_lost_connection_fails(self):
        result = self.adapter.result(self.request, ACTION_LOST, None)
        self.assertEqual(result["state"], "failed")
        self.assertFalse(result["success"])

    def test_identity_always_echoed(self):
        for terminal, payload in [
            (ACTION_SUCCEEDED, {"success": True, "completed_stage": 20,
                                "error_code": 0, "message": ""}),
            (ACTION_ABORTED, {"success": False, "completed_stage": 5,
                              "error_code": 7, "message": ""}),
        ]:
            result = self.adapter.result(self.request, terminal, payload)
            self.assertEqual(result["request_session_id"], "car-uuid-1")
            self.assertEqual(result["request_id"], "order-7:simulation:abc")
            self.assertEqual(result["order_id"], "order-7")
            self.assertEqual(result["target_class"], 2)
            self.assertIs(result["rehearsal"], False)


if __name__ == "__main__":
    unittest.main()
