# Navigation profiles

The two YAML files in this directory are complete snapshots of the current
local-navigation settings. They are deliberately kept separate so their values
can be tuned independently later.

`navigation_profile_manager.py` applies `automatic_navigation` at startup by
default. Call one of these services at a safe route boundary to select either
profile explicitly:

```bash
rosservice call /navigation_profile_manager/use_automatic_navigation
rosservice call /navigation_profile_manager/use_cone_zone_avoidance
```

The selected name is published as a latched string on
`/navigation_profile_manager/active_profile`. A switch updates only parameters
advertised by the corresponding ROS `dynamic_reconfigure` server. If a step
fails, the manager attempts to restore all values changed by that request.

Mission-driven switching is configured under
`navigation/profile_switching` in `smart_factory_mission/config/mission.yaml`.
It remains disabled until one-based, inclusive cone-zone waypoint bounds are
provided. When enabled, the mission requests a profile before sending the first
goal in each route section.

To select a different initial profile when launching navigation:

```bash
roslaunch gazebo_nav gazebo_nav.launch \
  initial_navigation_profile:=cone_zone_avoidance
```

To launch without applying either profile, pass
`apply_initial_navigation_profile:=false`.
