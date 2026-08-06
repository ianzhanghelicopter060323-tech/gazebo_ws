# smart_factory_mission

Mission orchestration for the incremental smart-factory task.

Runtime responsibilities are split across ROS Actions without changing the
external `ExecuteTask.action` contract:

- `mission_server.py` owns task validation, state/result publication, result
  caching, and high-level stage sequencing. It talks to navigation only through
  `/smart_factory/navigation`.
- `smart_factory_navigation` owns AMCL/TF readiness, the `move_base` client,
  route retry/progress behavior, and bounded `/cmd_vel` base alignment.

Arm, gripper, and fixed-grasp implementations live in
`smart_factory_manipulation`; the mission package retains only sequencing and
temporary compatibility imports for the former module paths.

The current milestone implements:

1. Receive `ExecuteTask.action`.
2. Validate the target class.
3. Ask the navigation Action server to check `move_base`, AMCL pose, and
   `map -> base_footprint` TF.
4. Load a development-only pickup staging pose.
5. Submit the route through `Navigate.action`. In `fitted_path_lookahead` mode,
   the navigation server publishes the offline-fitted reference as a
   latched `nav_msgs/Path`, project localization onto monotonic path progress,
   and replace curvature-adaptive moving `MoveBaseGoal` targets at a bounded
   rate. `legacy_waypoints` remains available as a configuration rollback.
6. Require the final `MoveBaseGoal` to return `SUCCEEDED`, preserving the DWA
   position and orientation tolerances, then enter `ARRIVED_PICKUP_STAGING`.
7. Observe candidates in order 35, 36, 37. A stable non-target result advances
   from 35 to 36; two stable non-target results select 37 by elimination.
8. Use synchronized RGB-D observations to place the cube at the fixed arm TCP
   target, leaving a calibrated 0.356 m base-to-cube-center standoff. Small
   corrections use a heading-preserving, low-speed omnidirectional translation
   loop and are followed by another RGB-D observation.
9. Open the gripper before alignment/descent, execute the recorded fixed grasp
   pose, require `ready`, close to 0.76, require `GRASPING`, and lift the cube.

It does not modify planner output or read Gazebo model ground truth. Normal route
and candidate-to-candidate motion stays under `move_base`; only the bounded
fixed-standoff correction publishes `/cmd_vel` directly, with zero angular
velocity and localization feedback.

Before running a task, fill `config/pickup_staging_dev.yaml` and set
`configured: true`.

`launch/mission.launch` is a component launch and starts only the mission node;
the `/smart_factory/navigation` Action server and perception must already be
available. Navigation configuration now lives under
`smart_factory_navigation/config/`. Use
`roslaunch smart_factory_bringup full_competition.launch` for the integrated
system. Camera-FOV calibration cubes are controlled by the bringup launch and
remain disabled by default.
