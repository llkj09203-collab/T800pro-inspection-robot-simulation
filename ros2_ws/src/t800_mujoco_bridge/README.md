# t800_mujoco_bridge

Independent bridge for the EngineAI MuJoCo simulation. It converts the native
LCM state stream to ROS 2 odometry, IMU and TF, and forwards ROS 2 `cmd_vel`
commands through the existing guarded LCM control path.

It does not start MuJoCo, generate lidar points, convert point clouds, start
Nav2 or start RViz.

```bash
ros2 launch t800_mujoco_bridge bridge.launch.py
```

Public ROS interfaces:

- `/odom` (`nav_msgs/msg/Odometry`)
- `/lidar/imu` (`sensor_msgs/msg/Imu`)
- `/simulation_collision` (`std_msgs/msg/Bool`)
- `/tf` and `/tf_static`
- `/cmd_vel` input (`geometry_msgs/msg/Twist`)
- `/reset_simulation` (`std_srvs/srv/Trigger`)
- `/t800/control/set_manual` (`std_srvs/srv/Trigger`)
- `/t800/control/set_auto` (`std_srvs/srv/Trigger`)

## Control authority

The bridge starts in `MANUAL` mode. In this mode the physical or virtual
gamepad owns all motion axes and Nav2 commands cannot overwrite them. Switch
to autonomous navigation explicitly with:

```bash
ros2 service call /t800/control/set_auto std_srvs/srv/Trigger "{}"
```

Return control to the remote before teleoperation with:

```bash
ros2 service call /t800/control/set_manual std_srvs/srv/Trigger "{}"
```

The RViz `T800 Waypoints` panel exposes the same operations as `Manual
Control` and `Auto Navigation`. In AUTO mode a stale navigation command stops
the robot; in MANUAL mode the navigation gate leaves remote motion axes
untouched. Emergency-stop and motion-state buttons remain available in both
modes.

Command smoothing follows the selected authority. Manual commands retain the
EngineAI walking controller's original gamepad low-pass filter. Autonomous
commands bypass that gamepad-specific filter because Nav2 already applies its
configured velocity and acceleration smoothing. The navigation heartbeat,
timeout stop and policy-command magnitude limit remain active in AUTO mode.
Switching authority resets the gamepad filter state so commands from one mode
cannot leak into the other.

## Velocity conversion

`/cmd_vel` carries standard ROS linear velocity (m/s) and angular velocity
(rad/s). The EngineAI walking controller used by this simulation accepts a
normalized policy command instead of those SI units. The bridge therefore:

- blocks negative linear commands so navigation is forward-only;
- compensates for the steering input dead zone;
- divides each axis by `linear_input_max` or `angular_input_max` so values
  with different SI units become a dimensionless direction;
- preserves the resulting forward/turn ratio and scales the combined vector
  to `policy_command_magnitude`.

The policy values in `config/bridge.yaml` must not be interpreted as measured
robot velocity.
