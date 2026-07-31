# smart_factory_mission

Mission orchestration for the incremental smart-factory task.

The current milestone implements:

1. Receive `ExecuteTask.action`.
2. Validate the target class.
3. Check `move_base`, AMCL pose, and `map -> base_footprint` TF.
4. Load a development-only pickup staging pose.
5. Send standard `MoveBaseGoal` messages along the configured route. Intermediate
   points advance on position alone when the robot enters
   `navigation/intermediate_pass_radius`; they do not require a stop or final yaw
   alignment.
6. Require the final `MoveBaseGoal` to return `SUCCEEDED`, preserving the DWA
   position and orientation tolerances, then return `ARRIVED_PICKUP_STAGING`.

It never publishes `/cmd_vel`, modifies planner output, or reads Gazebo model
ground truth.

Before running a navigation task, fill
`config/pickup_staging_dev.yaml` and set `configured: true`.
