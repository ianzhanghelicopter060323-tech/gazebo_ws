"""Stable navigation errors exposed by ``Navigate.action``.

Values shared with ``ExecuteTask.action`` intentionally keep the same number.
This is a compatibility aid, not a dependency on the mission package.
"""

SUCCESS = 0
MOVE_BASE_UNAVAILABLE = 3
LOCALIZATION_NOT_READY = 4
GOAL_UNAVAILABLE = 5
NAVIGATION_TIMEOUT = 6
NAVIGATION_ABORTED = 7
REQUEST_PREEMPTED = 8
# Alias used by the migrated implementation while its callers move to the
# navigation vocabulary.
TASK_PREEMPTED = REQUEST_PREEMPTED
ALIGNMENT_FAILED = 11
INVALID_COMMAND = 254
INTERNAL_ERROR = 255
