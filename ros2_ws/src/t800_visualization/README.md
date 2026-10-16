# t800_visualization

RViz is deliberately independent from simulation, lidar and navigation:

```bash
ros2 launch t800_visualization rviz.launch.py
```

The default configuration subscribes to the standard `/map`, `/scan`,
`/lidar/points_raw`, `/plan` and TF interfaces. It does not start producers for
those topics.

For the default MuJoCo world, where no matching saved navigation map exists,
use the sensor-only view instead:

```bash
ros2 launch t800_visualization sensors.launch.py
```

It uses `lidar_link` as its fixed frame, enables the raw 3D cloud and does not draw
an unrelated saved map. Both configurations use an unrestricted Orbit view.
