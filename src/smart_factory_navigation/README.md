# smart_factory_navigation

This package owns localization readiness, `move_base` route execution, fitted
path tracking, and direct base alignment. It deliberately has no dependency on
`smart_factory_mission`.

## Action boundary

`Navigate.action` lives in this package rather than
`smart_factory_interfaces`: it is an internal subsystem contract consumed by
mission, while `ExecuteTask.action` remains the external task contract. This
keeps navigation implementation/versioning together and avoids expanding the
public simulator interface.

The default server is `/smart_factory/navigation` and supports:

- `EXECUTE_STAGING_ROUTE`
- `NAVIGATE_POSE`
- `ALIGN_FOR_GRASP`
- `WAIT_FOR_LOCALIZATION`
- `GET_LOCALIZED_POSE`

The pose-query result carries `pose_valid`, the requested `frame_id` in
`localized_pose.header.frame_id`, and a planar pose encoded as a normal
`geometry_msgs/PoseStamped`.

Start the subsystem with:

```bash
roslaunch smart_factory_navigation navigation.launch
```

The launch file loads `config/navigation.yaml` and the generated fitted path.
Route execution preserves the reference polyline for visualization and sends
its 30 generated execution waypoints to `move_base` one at a time. Intermediate
points advance inside the configured 0.15 m pass radius. The final staging goal
uses a 0.15 m position radius together with a 0.04 rad yaw tolerance.

## Python client API

Import with:

```python
from smart_factory_navigation.client import NavigationClient
```

The stable mission-facing methods are:

```text
wait_for_server(timeout=10.0) -> bool
execute_staging_route(waypoints, request_id="", feedback_cb=None,
                      preempt_requested=None) -> NavigationResult
navigate_pose(pose, request_id="", feedback_cb=None,
              preempt_requested=None) -> NavigationResult
align_for_grasp(pose, request_id="", feedback_cb=None,
                preempt_requested=None) -> NavigationResult
wait_for_localization(preempt_requested=lambda: False,
                      feedback_cb=None) -> NavigationResult
wait_until_ready(preempt_requested=lambda: False,
                 feedback_cb=None) -> bool
localized_pose(frame_id) -> Optional[(x, y, yaw)]
localized_xy(frame_id) -> Optional[(x, y)]
pose_is_within_radius(pose, radius) -> bool
```

`NavigationResult` contains `success`, `error_code`, `message`,
`completed_waypoints`, `localized_pose`, and `frame_id`. Feedback callbacks
receive a `NavigationFeedbackEvent`, not a generated ROS message.

An empty staging route is rejected with `GOAL_UNAVAILABLE`. This resolves the
old mode-dependent behavior where legacy mode reported immediate arrival but
fitted-path mode failed the route.

## Mission integration

`smart_factory_mission` now uses this client in the following sequence:

1. wait for the navigation Action server;
2. call `wait_for_localization` before goal lookup;
3. submit staging waypoints with `execute_staging_route`;
4. use `navigate_pose` and `align_for_grasp` from the pickup pipeline;
5. pass `localized_pose` to pickup calculations;
6. translate navigation feedback phases and error codes into mission state and
   `ExecuteTaskResult` values.

The former navigation/localization/direct-alignment parameter copies and Python
implementations have been removed from the mission package; this package is the
single implementation owner.
