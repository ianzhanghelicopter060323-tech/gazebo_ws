# smart_factory_interfaces

Shared ROS message and action definitions. This package contains no task logic.

Current interfaces:

- `ExecuteTask.action`: vehicle/bridge to mission request, feedback, and result.
- `TaskState.msg`: observable mission state for logs and debugging.

Perception and manipulation interfaces will be added when those stages are
implemented.
