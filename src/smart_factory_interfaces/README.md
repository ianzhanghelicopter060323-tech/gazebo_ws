# smart_factory_interfaces

Shared ROS message and action definitions. This package contains no task logic.

Current interfaces:

- `ExecuteTask.action`: vehicle/bridge to mission request, feedback, and result.
- `TaskState.msg`: observable mission state for logs and debugging.
- `LocateCube.srv`: temporally stable class, OCR box, and RGB-D point in camera,
  base, and map frames.
