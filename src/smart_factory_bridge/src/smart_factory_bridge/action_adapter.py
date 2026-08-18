"""Translate ``/sim_task/execute`` Action events into protocol messages.

The adapter is deliberately free of ROS and actionlib imports: the node
passes plain dicts (``{"success": ..., "completed_stage": ...}`` etc.),
so the mapping rules can be unit tested without a master.

Success rule (frozen in the task book section 6.6 and the handover
section 4.3): the bridge may only report ``completed`` when the Action
terminated with SUCCEEDED **and** ``result.success == true`` **and**
``result.completed_stage == 20``. Stage 14, aborts, preempts, failures
and missing stage 20 all map to a truthful ``failed`` result.
"""

from __future__ import absolute_import

from smart_factory_bridge import protocol

# actionlib GoalStatus terminal codes (same numbers as actionlib_msgs).
ACTION_SUCCEEDED = 3
ACTION_ABORTED = 4
ACTION_PREEMPTED = 5
ACTION_LOST = 6

# Mission error codes reused by the bridge for failures without a result
# payload (mirrors smart_factory_mission/error_codes.py).
MISSION_ERROR_TASK_PREEMPTED = 8
MISSION_ERROR_INTERNAL = 255


class ActionAdapter(object):
    """Build ack / progress / result payloads from Action events."""

    def __init__(self, session_id):
        self._session_id = session_id

    # ------------------------------------------------------------------

    def ack(self, request):
        """Ack sent right after the goal was submitted."""
        return protocol.make_ack(self._session_id, request)

    def progress(self, request, feedback):
        """Map one ExecuteTaskFeedback to a progress message.

        ``feedback`` is a dict with ``current_stage``, ``retry_count``
        and ``detail`` keys (mirrors ExecuteTaskFeedback fields).
        """
        return protocol.make_progress(
            self._session_id,
            request,
            stage=feedback.get("current_stage", 0),
            detail=feedback.get("detail", ""),
            retry_count=feedback.get("retry_count", 0),
        )

    def result(self, request, terminal_state, action_result):
        """Map the terminal Action state + result to a final result.

        ``terminal_state`` is one of the ``ACTION_*`` codes above;
        ``action_result`` is the result dict or ``None``.

        Returns a protocol ``result`` message. ``completed`` is sent only
        when every condition of the frozen success rule holds.
        """
        action_result = action_result or {}
        success = bool(action_result.get("success", False))
        completed_stage = action_result.get("completed_stage", 0)
        action_error = action_result.get("error_code", 0)

        if (
            terminal_state == ACTION_SUCCEEDED
            and success
            and completed_stage == protocol.STAGE_TASK_COMPLETED
        ):
            message = action_result.get("message", "") or (
                "target object placed in the correct warehouse"
            )
            return protocol.make_result(
                self._session_id,
                request,
                success=True,
                completed_stage=completed_stage,
                error_code=0,
                message=message,
            )

        if terminal_state == ACTION_PREEMPTED:
            error_code = (
                action_error
                if action_error
                else MISSION_ERROR_TASK_PREEMPTED
            )
            message = "simulation task preempted"
        elif terminal_state == ACTION_ABORTED:
            error_code = action_error or MISSION_ERROR_INTERNAL
            message = action_result.get("message", "") or "simulation task aborted"
        else:  # lost or unknown terminal state
            error_code = action_error or MISSION_ERROR_INTERNAL
            message = action_result.get("message", "") or (
                "simulation task connection lost"
            )
        return protocol.make_result(
            self._session_id,
            request,
            success=False,
            completed_stage=completed_stage,
            error_code=error_code,
            message=message,
        )
