# smart_factory_bringup

Unified launch entry points.

- `simulation.launch`: Gazebo, navigation, and RViz only.
- `nav_to_pickup_stage.launch`: simulation plus the current mission milestone.
- `full_competition.launch`: reserved for the future complete pipeline.

Run the current stage with:

```bash
roslaunch smart_factory_bringup nav_to_pickup_stage.launch
```
