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
