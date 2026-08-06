"""Public Python API for smart factory navigation."""

from smart_factory_navigation.base_alignment_controller import (
    BaseAlignmentController,
    BaseAlignmentFailure,
    BaseAlignmentPreempted,
)
from smart_factory_navigation.localization_monitor import LocalizationMonitor
from smart_factory_navigation.models import (
    NavigationFeedbackEvent,
    NavigationResult,
)
from smart_factory_navigation.route_executor import (
    RouteExecutor,
    RouteNavigationFailure,
    RouteNavigationPreempted,
)

__all__ = [
    "BaseAlignmentController",
    "BaseAlignmentFailure",
    "BaseAlignmentPreempted",
    "LocalizationMonitor",
    "NavigationFeedbackEvent",
    "NavigationResult",
    "RouteExecutor",
    "RouteNavigationFailure",
    "RouteNavigationPreempted",
]

# Import lazily from user code after catkin has generated NavigateAction. Keeping
# it out of package initialization lets pure geometry modules run before build.
