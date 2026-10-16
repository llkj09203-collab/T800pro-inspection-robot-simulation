# t800_mujoco_lidar

Independent ROS 2 source package for the T800 Pro MuJoCo CPU ray lidar.

## Scope

This package has one public responsibility:

```text
EngineAI MuJoCo lidar/state snapshot (LCM)
  -> MuJoCo mj_multiRay
  -> /lidar/points_raw (sensor_msgs/msg/PointCloud2)
```

It does **not** provide odometry, TF, IMU, velocity control, mapping,
localization, navigation, or a 2D `LaserScan` conversion.

The robot-state bridge or `robot_state_publisher` remains responsible for the
dynamic transform chain ending at `lidar_link`. This package guarantees that
the cloud coordinates and `frame_id` use that sensor frame.

## Public interface

| Topic | Type | QoS |
|---|---|---|
| `/lidar/points_raw` | `sensor_msgs/msg/PointCloud2` | best effort, volatile, depth 5 |

The cloud uses `lidar_link` with ROS axes `+X forward, +Y left, +Z up` and
fields `x`, `y`, `z`, `intensity`, `ring`, and `time`.

The current model is an instantaneous CPU ray capture. It has no material
reflectivity or rolling firing model, so `intensity` and per-point `time` are
explicitly zero. `header.stamp` is the MuJoCo snapshot sample time, not the ROS
publish/receive time.

## Build

```bash
cd /home/llkj/engineai_robotics_native_sdk/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select t800_mujoco_lidar
source install/setup.bash
```

Python dependencies `lcm`, `mujoco`, `numpy`, and `yaml` must be available in
the EngineAI development container.

## Run

Start a native MuJoCo lidar scenario independently. The lidar package does not
start, stop, or otherwise manage the simulator. It reads the active scenario
name from `lidar/state` and selects the matching model automatically. Then
start only the sensor node:

```bash
ros2 launch t800_mujoco_lidar lidar.launch.py
```

The normal `bash scripts/run_mujoco.sh t800pro` startup is supported and is
reported as scenario `default`. `ENGINEAI_LIDAR_SCENE` remains optional and is
only a world-selection mechanism; it is not required to enable the lidar.

By default the package sends at most 500 sampled rays back to the active
MuJoCo window for visualization. This does not change the public point cloud.
Disable it for headless operation with:

```bash
ros2 launch t800_mujoco_lidar lidar.launch.py \
  visualize_rays_in_mujoco:=false
```

MuJoCo, the T800 motion controller, navigation, mapping, and visualization all
remain separate processes outside this package.

After discovery, the scene file is selected by convention from:

```text
<sdk_root>/assets/resource/<robot_model>_lidar_<scenario>.xml
```

An explicit XML can be selected with `model_path:=/absolute/path/model.xml`.

Verify the output:

```bash
ros2 topic info /lidar/points_raw -v
ros2 topic hz /lidar/points_raw
ros2 topic echo /lidar/points_raw --once --field header
```

## Current calibration status

The `depth` MuJoCo camera is retained as the provisional lidar mount. The
active model is 240 x 180 rays, 120 degree horizontal FOV, 33.1 degrees up,
56.9 degrees down, 0.2--30 m, and at most 10 Hz. These values are configurable
and must be replaced with the calibrated real sensor model when it becomes
available.

The LCM `lidar/state` protocol is part of this project's simulation extension,
not an upstream EngineAI public lidar protocol. LCM remains private to this
package; consumers only depend on the ROS 2 PointCloud2 topic.
