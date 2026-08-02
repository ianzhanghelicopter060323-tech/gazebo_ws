# smart_factory_mission

Mission orchestration for the incremental smart-factory task.

The current milestone implements:

1. Receive `ExecuteTask.action`.
2. Validate the target class.
3. Check `move_base`, AMCL pose, and `map -> base_footprint` TF.
4. Load a development-only pickup staging pose.
5. Publish the offline-fitted reference as a latched `nav_msgs/Path`, project
   localization onto monotonic path progress, and replace curvature-adaptive
   moving `MoveBaseGoal` targets at a bounded rate.
6. Require the final `MoveBaseGoal` to return `SUCCEEDED`, preserving the DWA
   position and orientation tolerances, then return `ARRIVED_PICKUP_STAGING`.

It does not modify planner output, read Gazebo model ground truth, or publish
motion commands directly. Robot motion remains owned by `move_base`.

Before running a navigation task, fill
`config/pickup_staging_dev.yaml` and set `configured: true`.
