# T800 Pro 巡检机器人仿真开发（ROS 2）

本工作区保存 T800 Pro 在 MuJoCo（机器人动力学仿真器）中的雷达、仿真桥接、
建图导航和 RViz（ROS 可视化工具）配置。仿真、控制器、遥控器、雷达、导航和
可视化彼此独立，可按任务组合启动。场景只在仿真入口选择一次。
`build/`、`install/`和`log/`是可重新生成的 Colcon 构建产物，不进入 Git。
默认建图链路是 GLIM 加本工作区的高程/通行性转换；SLAM Toolbox
作为备用二维建图方式保留。

首次创建包含 GLIM CPU版的可选导航开发镜像：

```bash
cd <engineai_robotics_native_sdk>
WITH_GLIM=y bash docker/generate.sh
```

## 构建

```bash
source /opt/ros/humble/setup.bash
cd <engineai_robotics_native_sdk>/ros2_ws
colcon build --symlink-install
source install/setup.bash
colcon test --packages-select \
  t800_mujoco_lidar t800_mujoco_bridge \
  t800_navigation t800_waypoints t800_visualization
colcon test-result --verbose
```

## 软件包

- `t800_mujoco_lidar`：读取 MuJoCo 状态并发布三维点云 `/lidar/points_raw`。
- `t800_mujoco_bridge`：在 LCM（轻量通信机制）和 ROS 2 之间转换里程计、IMU、TF 与速度指令。
- `t800_navigation`：点云转二维激光、建图、定位与 Nav2（ROS 2 导航框架）配置。
- `t800_waypoints`：RViz 航点编辑、路线保存和逐点巡检任务执行。
- `t800_visualization`：独立的 RViz 显示配置。

正式导航地图位于 `src/t800_navigation/maps/`。

## T800 导航启动

先进入 EngineAI 开发容器，并在六个终端中依次执行。每个 ROS 2 终端均先执行：

```bash
cd <engineai_robotics_native_sdk>/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

然后分别运行：

```bash
# 终端 1：MuJoCo 仿真（选择 mixed 场景）
ros2 launch t800_simulation mixed.launch.py

# 终端 2：官方运动控制器（在 SDK 根目录运行）
cd <engineai_robotics_native_sdk>
bash scripts/start_controller.sh

# 终端 3：虚拟遥控器（在 SDK 根目录运行）
cd <engineai_robotics_native_sdk>
bash scripts/start_gamepad.sh

# 终端 4：三维雷达
ros2 launch t800_mujoco_lidar lidar.launch.py

# 终端 5：桥接、二维激光转换、定位和导航
# 地图根据终端 1 的活动场景自动选择，不再传 map 参数。
ros2 launch t800_navigation navigation.launch.py

# 终端 6：RViz（显示项由用户自行添加或打开已有配置）
rviz2
```

需要在地图中编辑和保存巡检路线时，另外启动：

```bash
ros2 launch t800_waypoints waypoints.launch.py
```

在 RViz 选择 `Add Waypoint` 添加位置、用 `Interact` 拖动或旋转，保存后
在 `T800 Waypoints` 面板中直接保存、删除、清空、开始或停止巡检，不需要再
输入服务命令。

`navigation.launch.py` 只用于加载已有地图进行定位和导航，不执行建图。新场景
使用 `mapping.launch.py` 自动启动 GLIM，并把 `/glim_ros/map` 转换为二维通行
地图；备用二维方式为 `mapping_2d.launch.py`（SLAM Toolbox）。地图保存
后，在 `config/navigation/scene_maps.yaml` 中登记一次即可供导航自动选择。

当前结果用于仿真验证。雷达安装外参、噪声模型、运动控制精度和导航参数仍需在
真机接口与标定数据明确后继续校准。
