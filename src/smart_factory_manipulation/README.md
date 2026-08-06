# smart_factory_manipulation

Reusable arm, gripper, and fixed-grasp capabilities for the smart-factory task.

The package currently owns:

- `ManipulationStage`, which publishes fixed arm and gripper commands and
  verifies joint, grasp-readiness, and grasp-state feedback.
- `FixedGraspPlanner`, which converts a localized cube surface point into the
  base pose required to place the cube center at the calibrated TCP target.

Mission-specific sequencing remains in `smart_factory_mission`. In particular,
the 35 -> 36 -> 37 candidate policy, state transitions, perception retries,
and decisions about when to navigate or grasp are orchestration concerns and
are not part of this package.

The current mission supplies manipulation and fixed-grasp parameters from its
`pickup` configuration. The public implementation modules are:

```python
from smart_factory_manipulation.fixed_grasp import FixedGraspPlanner
from smart_factory_manipulation.manipulation_stage import ManipulationStage
```

`ManipulationStage` preserves the existing ROS interface:

| Direction | Topic | Type | Purpose |
| --- | --- | --- | --- |
| publish | `/arm_controller/command` | `trajectory_msgs/JointTrajectory` | command the five movable arm joints |
| publish | `/gripper_controller/command` | `std_msgs/Float64` | command gripper opening/closing |
| subscribe | `/joint_states` | `sensor_msgs/JointState` | verify arm and gripper positions |
| subscribe | `/grasp_attach/ready` | `std_msgs/Bool` | verify the grasp backend is ready |
| subscribe | `/grasp_attach/state` | `std_msgs/String` | verify `IDLE` and `GRASPING` states |

All topic names, fixed positions, tolerances, and timeouts can be overridden
through the constructor configuration. The defaults remain identical to the
former `smart_factory_mission.manipulation_stage` implementation.

Gazebo's robot/controller wiring and its physics-only grasp attachment node
remain part of the simulation model (`car3`). This package owns the reusable
commands, feedback checks, and fixed-TCP planning; it does not depend on Gazebo
and can therefore be reused with a hardware backend that exposes the same ROS
interfaces.

Run the package tests after sourcing the workspace:

```bash
python3 -m unittest discover \
  -s src/smart_factory_manipulation/test -p 'test_*.py' -v
```
