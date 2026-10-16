# T800 Pro 巡检机器人仿真开发（ROS 2）

本工作区保存 T800 Pro 在 MuJoCo（机器人动力学仿真器）中的雷达、仿真桥接、
建图导航和 RViz（ROS 可视化工具）配置。四部分彼此独立，可按任务组合启动。
`build/`、`install/`和`log/`是可重新生成的 Colcon 构建产物，不进入 Git。

## 构建

```bash
source /opt/ros/humble/setup.bash
cd /path/to/engineai_robotics_native_sdk/ros2_ws
colcon build --symlink-install
source install/setup.bash
colcon test --packages-select \
  t800_mujoco_lidar t800_mujoco_bridge \
  t800_stage3_navigation t800_visualization
colcon test-result --verbose
```

## 软件包

- `t800_mujoco_lidar`：读取 MuJoCo 状态并发布三维点云 `/lidar/points_raw`。
- `t800_mujoco_bridge`：在 LCM（轻量通信机制）和 ROS 2 之间转换里程计、IMU、TF 与速度指令。
- `t800_stage3_navigation`：点云转二维激光、建图、定位与 Nav2（ROS 2 导航框架）配置。
- `t800_visualization`：独立的 RViz 显示配置。

正式 Stage3 地图位于 `src/t800_stage3_navigation/maps/`。

## Stage3 导航启动

先进入 EngineAI 开发容器，并在四个终端中依次执行：

```bash
# 终端 1：MuJoCo 仿真（选择 Stage3 场景）
ENGINEAI_LIDAR_SCENE=stage3_mixed bash scripts/run_mujoco.sh t800pro

# 终端 2：三维雷达
cd ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch t800_mujoco_lidar lidar.launch.py

# 终端 3：桥接、二维激光转换、定位和导航
cd ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch t800_stage3_navigation navigation.launch.py \
  map:=$(pwd)/src/t800_stage3_navigation/maps/stage3_groundfix.yaml

# 终端 4：RViz
cd ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch t800_visualization rviz.launch.py
```

当前结果用于仿真验证。雷达安装外参、噪声模型、运动控制精度和导航参数仍需在
真机接口与标定数据明确后继续校准。
