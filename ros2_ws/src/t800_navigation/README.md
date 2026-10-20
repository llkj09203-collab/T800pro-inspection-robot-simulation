# T800 mapping and navigation

This package owns the T800 mapping adapters, saved-map selection,
localization and Nav2 configuration. Simulation state/control and TF are
provided by `t800_mujoco_bridge`; raw 3D lidar points are provided by
`t800_mujoco_lidar`.

## Mapping pipelines

GLIM is the default mapping pipeline:

```text
/lidar/points_raw + /lidar/imu
              -> GLIM -> /glim_ros/map
              -> elevation_mapper -> /elevation_map/points
                                  -> /map
```

The project development image installs the CPU build of `glim_ros` and the
default launch starts it with `config/elevation/glim_cpu/`:

```bash
WITH_GLIM=y bash docker/generate.sh
```

```bash
ros2 launch t800_navigation mapping.launch.py
```

For debugging an already-running GLIM process, disable the managed node:

```bash
ros2 launch t800_navigation mapping.launch.py start_glim:=false
```

The mapper converts GLIM's global cloud into a robot-aware 2.5D occupancy grid.
Step height, slope, roughness and body-clearance limits are configured in
`config/elevation/traversability.yaml`. Save its lossless layers with:

```bash
ros2 service call /save_elevation_map std_srvs/srv/Trigger '{}'
```

For conventional 2D SLAM, use the fallback pipeline:

```text
/lidar/points_raw -> pointcloud_to_laserscan -> /scan -> slam_toolbox -> /map
```

```bash
ros2 launch t800_navigation mapping_2d.launch.py
```

Save either published occupancy grid with a name registered for the active
scene:

```bash
ros2 run nav2_map_server map_saver_cli -f \
  src/t800_navigation/maps/mixed_map
```

## Localization and navigation

`navigation.launch.py` does not build a map. It selects the saved map registered
for the simulator's active scene in `config/navigation/scene_maps.yaml`, starts
the MuJoCo bridge, converts the 3D cloud to `/scan`, and starts Nav2 with AMCL.

```bash
ros2 launch t800_navigation navigation.launch.py
```

The complete Nav2 configuration is stored in
`config/navigation/nav2_params.yaml`. The controller is a rotation shim around
DWB, and the global planner is NavFn.

Send a parameterized single goal with:

```bash
ros2 run t800_navigation single_goal --ros-args \
  -p x:=2.0 -p y:=0.0 -p yaw:=0.0 -p goal_name:=P01
```

Start RViz independently:

```bash
ros2 launch t800_visualization rviz.launch.py
```

The TF ownership is:

```text
map -> odom                  AMCL while navigating; mapper while mapping
odom -> base_link            t800_mujoco_bridge
base_link -> ... -> lidar    robot transforms
```

## Build and test

From `ros2_ws`:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test --packages-select t800_navigation
colcon test-result --verbose
```
