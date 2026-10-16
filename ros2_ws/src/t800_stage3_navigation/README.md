# T800 Stage 3 navigation

The current mapping path is GLIM 3D lidar-inertial SLAM followed by the
`elevation_mapper` robot-aware 2.5D compression node. The optional
`mapping.launch.py` path provides a conventional 2D `slam_toolbox` workflow.

```text
/lidar/points_raw + /lidar/imu -> GLIM -> /glim_ros/map
                                  |
                                  +-> elevation_mapper -> /elevation_map/points
                                                        -> /map
```

`/map` is a Nav2-compatible `nav_msgs/OccupancyGrid`.  Its occupancy value is
derived from configured T800 Pro step height, slope, roughness and body
clearance limits rather than treating every lidar return as an obstacle.  The
lossless height layers can be saved with the `save_elevation_map` service.

This package owns mapping, localization and Nav2 configuration. Simulation
state/control and TF are provided by `t800_mujoco_bridge`; raw 3D points are
provided independently by `t800_mujoco_lidar`. The standard ROS 2
`pointcloud_to_laserscan` node converts `/lidar/points_raw` to `/scan` for the
2D navigation stack. The simulation TF path is:

```text
odom -> base_link -> head_pitch_link -> head_yaw_link -> lidar_link
```

`map -> odom` remains the responsibility of SLAM while mapping and AMCL while
localizing.

The active, previously validated simulation keeps a 120-by-90-degree field with
asymmetric vertical limits: 33.10 degrees above and 56.90 degrees below the
horizontal. The vendor document's approximate 240x180 and 120-by-85-degree
specifications are recorded separately in
`assets/config/t800pro/lidar_sim/default.yaml`; they do not silently override
the active values before calibration resolves the difference.

Build inside the project container:

```bash
source /opt/ros/humble/setup.bash
cd /path/to/engineai_robotics_native_sdk/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Start the optional 2D mapper:

```bash
ros2 launch t800_stage3_navigation mapping.launch.py
```

Start the ROS side of GLIM mapping after launching GLIM with the matching
configuration in `config/glim_cpu/`:

```bash
ros2 launch t800_stage3_navigation glim_mapping.launch.py
```

Start the independent lidar package before mapping or navigation:

```bash
ros2 launch t800_mujoco_lidar lidar.launch.py
```

The CPU GLIM profile keeps up to 15,000 preprocessed points per frame and emits
0.10 m submaps. Use `--angular-stride 2` only as an explicit low-compute mode;
it reduces the point rate to one eighth when combined with 5 Hz operation.

RViz shows `/map` as the planar navigation cost map and
`/elevation_map/points` as `2.5D Elevation`. The latter uses ground height for
XYZ position and `traversability_cost` for color.

Save the 2.5D layers (`ground`, `ceiling`, `roughness`, `slope_deg`, `count`,
and robot-specific `cost`):

```bash
ros2 service call /save_elevation_map std_srvs/srv/Trigger '{}'
```

Save a map after coverage is complete:

```bash
ros2 run nav2_map_server map_saver_cli -f maps/stage3_groundfix
```

Start localization and Nav2 with the saved map:

```bash
ros2 launch t800_stage3_navigation navigation.launch.py \
  map:=/absolute/path/maps/stage3_groundfix.yaml
```

The `map` argument is required so a Stage-3 map cannot be silently displayed
over an unrelated default MuJoCo world. This launch starts the MuJoCo ROS
bridge, the standard
`pointcloud_to_laserscan` converter and Nav2. It deliberately does not start
MuJoCo, the lidar package or RViz.

Start visualization separately:

```bash
ros2 launch t800_visualization rviz.launch.py
```

The points in `config/points.yaml` are simulation reference points and must be
recalibrated when the map or robot start pose changes.
