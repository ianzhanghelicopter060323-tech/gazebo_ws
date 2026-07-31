# smart_factory_mission

Mission orchestration for the incremental smart-factory task.

The current milestone implements:

1. Receive `ExecuteTask.action`.
2. Validate the target class.
3. Check `move_base`, AMCL pose, and `map -> base_footprint` TF.
4. Load a development-only pickup staging pose.
5. Send standard `MoveBaseGoal` messages along the configured route. Intermediate
   points normally advance on position alone when the robot enters
   `navigation/intermediate_pass_radius`. Waypoint numbers listed in
   `navigation/heading_constrained_waypoints` cancel the current move_base goal
   and rotate in place until `navigation/intermediate_yaw_tolerance` is met.
6. Require the final `MoveBaseGoal` to return `SUCCEEDED`, preserving the DWA
   position and orientation tolerances, then return `ARRIVED_PICKUP_STAGING`.

It does not modify planner output or read Gazebo model ground truth. It publishes
zero-linear-velocity `/cmd_vel` commands only during the explicit heading-alignment
phase, after cancelling the active `move_base` goal.

Before running a navigation task, fill
`config/pickup_staging_dev.yaml` and set `configured: true`.
