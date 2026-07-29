# smart_factory_mission

Mission orchestration for the incremental smart-factory task.

The current milestone implements:

1. Receive `ExecuteTask.action`.
2. Validate the target class.
3. Check `move_base`, AMCL pose, and `map -> base_footprint` TF.
4. Load a development-only pickup staging pose.
5. Send a standard `MoveBaseGoal`.
6. Return `ARRIVED_PICKUP_STAGING`.

It never publishes `/cmd_vel`, modifies planner output, or reads Gazebo model
ground truth.

Before running a navigation task, fill
`config/pickup_staging_dev.yaml` and set `configured: true`.
