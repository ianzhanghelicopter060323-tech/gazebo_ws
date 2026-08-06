"""ROS-message-free values shared by the navigation server and client."""

from dataclasses import dataclass
from typing import Optional, Tuple

from smart_factory_navigation import states


@dataclass
class NavigationFeedbackEvent:
    phase: int = 0
    current_waypoint: int = 0
    waypoint_count: int = 0
    retry_count: int = 0
    path_progress: float = 0.0
    detail: str = ""


@dataclass
class NavigationResult:
    success: bool
    error_code: int
    message: str
    completed_waypoints: int = 0
    localized_pose: Optional[Tuple[float, float, float]] = None
    frame_id: str = ""


@dataclass
class RouteExecutionContext:
    """Minimal mutable context required by ``RouteExecutor``.

    Keeping this protocol in the navigation package removes the former hidden
    dependency on ``smart_factory_mission.TaskContext``.
    """

    task_id: str
    pickup_staging_goals: list
    current_waypoint_index: int = 0
    retry_count: int = 0


class NavigationStateRecorder:
    """Translate route transitions into package-neutral feedback events."""

    def __init__(self, context, publish):
        self._context = context
        self._publish = publish
        self.current_phase = states.IDLE
        self.detail = ""

    def transition(self, phase, detail):
        self.current_phase = phase
        self.detail = detail
        self._publish(
            NavigationFeedbackEvent(
                phase=phase,
                current_waypoint=self._context.current_waypoint_index + 1,
                waypoint_count=len(self._context.pickup_staging_goals),
                retry_count=self._context.retry_count,
                detail=detail,
            )
        )
