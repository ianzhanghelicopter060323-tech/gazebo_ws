# smart_factory_bringup

Unified launch entry points.

- `simulation.launch`: Gazebo, AMCL/`move_base`, and RViz only; it does not
  start the smart-factory navigation Action server or mission.
- `full_competition.launch`: the primary entry point for simulation, `move_base`,
  the smart-factory navigation Action server, perception, and the mission state
  machine.
- `nav_to_pickup_stage.launch`: compatibility alias for the former entry point;
  new commands should use `full_competition.launch`.

Run the integrated system with:

```bash
roslaunch smart_factory_bringup full_competition.launch
```

`full_competition.launch` starts each subsystem exactly once. Its main switches
are `start_navigation`, `start_navigation_server`, `start_gazebo`, `start_rviz`,
and `start_perception`. Here `start_navigation` controls the existing
Gazebo/`move_base` component launch, while `start_navigation_server` controls the
independent `/smart_factory/navigation` Action server. Gazebo is owned by the
former component, so `start_gazebo` is meaningful only while
`start_navigation:=true`.

To start navigation against an already-running Gazebo instance, use:

```bash
roslaunch smart_factory_bringup full_competition.launch start_gazebo:=false
```

To reuse an already-running Gazebo/`move_base` stack while starting a fresh
smart-factory navigation Action server, use:

```bash
roslaunch smart_factory_bringup full_competition.launch start_navigation:=false
```

If the smart-factory navigation Action server is already running as well, add
`start_navigation_server:=false`. Its Action name, underlying `move_base` name,
and configuration files can be selected with `navigation_action_name`,
`move_base_action`, `navigation_config`, and `fitted_path_config`; the mission
uses the same `navigation_action_name` and waits up to
`navigation_server_wait_timeout` seconds for it.

Do not launch `simulation.launch` beside `full_competition.launch`: the former
is a component-level debugging entry and both commands would otherwise request
the same Gazebo/navigation nodes.

Camera-FOV calibration cubes remain opt-in:

```bash
roslaunch smart_factory_bringup full_competition.launch \
  spawn_calibration_cubes:=true
```

For component-level debugging, `roslaunch smart_factory_mission mission.launch`
starts only the mission node. Navigation and perception must already be running.
The legacy `nav_to_pickup_stage.launch` keeps its original behavior and therefore
does not start perception unless `start_perception:=true` is supplied explicitly.
