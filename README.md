# T800pro巡检机器人仿真开发

本仓库保存 T800 Pro 巡检机器人仿真项目自行开发的扩展代码，不包含 EngineAI
官方 SDK 的完整源码。项目基于 ROS 2 Humble、MuJoCo（机器人动力学仿真器）、
LCM（轻量通信机制）、GLIM（三维激光惯性建图框架）和 Nav2（ROS 2 导航框架），
当前包括三维雷达、仿真桥接、通用场景建图导航、巡检航点任务与 RViz
（ROS 可视化工具）配置。

## 仓库内容

```text
ros2_ws/src/
├── t800_mujoco_lidar       # MuJoCo 三维射线雷达与 PointCloud2 发布
├── t800_mujoco_bridge      # LCM/ROS 2 里程计、IMU、TF 和速度指令桥接
├── t800_runtime_environment # ROS 2/DDS 公共运行环境
├── t800_simulation         # 按场景命名的 MuJoCo 仿真入口
├── t800_navigation         # GLIM/二维建图、定位、点云转换与 Nav2 配置
├── t800_waypoints          # 航点保存、路线管理与逐点巡检任务
├── t800_waypoint_rviz_plugins # RViz 航点工具与控制面板
└── t800_visualization      # 独立 RViz 配置
scenes/                     # 本项目 mixed、office、warehouse 场景和雷达参数
patches/                    # 接入官方 SDK 所需的本项目修改
```

## 依赖

- Ubuntu 22.04
- ROS 2 Humble
- EngineAI 官方
  [`engineai_robotics_native_sdk`](https://github.com/engineai-robotics/engineai_robotics_native_sdk)
- MuJoCo、LCM、NumPy、PyYAML
- Nav2、`pointcloud_to_laserscan`；三维建图时另需 GLIM

`patches/engineai_native_sdk_integration.patch`以官方 `main` 提交
`335c60e`为基线生成，`patches/scene_selection_runtime.patch`在其上补充统一场景
选择和无参数控制器入口，`patches/glim_navigation_runtime.patch`再加入通用场景
命名及可选 GLIM容器环境，`patches/navigation_control_modes.patch`补充手动/自主
控制权切换与导航停机保护。四个补丁都只包含本项目的接口扩展，不包含完整官方源码。

## 接入官方 SDK

```bash
# 1. 下载官方 SDK
git clone https://github.com/engineai-robotics/engineai_robotics_native_sdk.git
cd engineai_robotics_native_sdk

# 2. 应用本项目补丁（请替换扩展仓库的实际路径）
git apply /path/to/T800pro-inspection-robot-simulation/patches/engineai_native_sdk_integration.patch
git apply /path/to/T800pro-inspection-robot-simulation/patches/scene_selection_runtime.patch
git apply /path/to/T800pro-inspection-robot-simulation/patches/glim_navigation_runtime.patch
git apply /path/to/T800pro-inspection-robot-simulation/patches/navigation_control_modes.patch

# 3. 安装本项目场景文件
cp /path/to/T800pro-inspection-robot-simulation/scenes/environment/navigation_ground.xml \
  assets/resource/environment/
cp /path/to/T800pro-inspection-robot-simulation/scenes/t800pro_lidar_*.xml \
  assets/resource/
mkdir -p assets/config/t800pro/lidar_sim
cp /path/to/T800pro-inspection-robot-simulation/scenes/config/lidar.yaml \
  assets/config/t800pro/lidar_sim/default.yaml
```

完成后按照官方说明构建 SDK，并设置其位置：

```bash
export ENGINEAI_ROBOTICS_DIR=/absolute/path/to/engineai_robotics_native_sdk
```

## 构建 ROS 2 扩展

需要 GLIM 三维建图时，首次生成开发容器使用：

```bash
cd /absolute/path/to/engineai_robotics_native_sdk
WITH_GLIM=y bash docker/generate.sh
```

```bash
cd /path/to/T800pro-inspection-robot-simulation/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 导航启动

在 EngineAI 开发容器的六个终端中依次运行。场景只在第一个终端选择：

```bash
# 终端 1：MuJoCo mixed 仿真
cd /path/to/T800pro-inspection-robot-simulation/ros2_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch t800_simulation mixed.launch.py

# 终端 2：官方运动控制器
cd "$ENGINEAI_ROBOTICS_DIR"
bash scripts/start_controller.sh

# 终端 3：虚拟遥控器
cd "$ENGINEAI_ROBOTICS_DIR"
bash scripts/start_gamepad.sh

# 终端 4：三维雷达
cd /path/to/T800pro-inspection-robot-simulation/ros2_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch t800_mujoco_lidar lidar.launch.py

# 终端 5：仿真桥接、二维激光转换、定位与导航
# 自动读取终端 1 的活动场景并选择对应地图。
ros2 launch t800_navigation navigation.launch.py

# 终端 6：RViz；显示项由用户自行添加或载入配置
rviz2
```

需要在 RViz 中编辑、保存并执行巡检路线时，另外启动：

```bash
ros2 launch t800_waypoints waypoints.launch.py
```

在 RViz 中使用 `Add Waypoint` 添加位置，通过 `T800 Waypoints` 面板保存路线、
启动或停止逐点巡检。手动遥控与自主导航采用显式模式切换，自主指令超时后请求停止。

新场景还没有保存地图时，不启动导航，改用
`ros2 launch t800_navigation mapping.launch.py` 启动 GLIM 三维建图及二维通行
地图转换；备用二维建图入口为 `mapping_2d.launch.py`。RViz、雷达及其他显示功能
不依赖已有地图；地图保存并登记到 `config/navigation/scene_maps.yaml` 后，导航会
自动选择它。

## 当前范围

当前成果用于仿真验证。雷达安装外参、传感器时间戳、噪声模型、机器人安全外廓、
速度限制及导航参数仍需依据真机接口和标定数据继续调整。

## 许可证

本仓库自行开发部分采用 BSD 3-Clause License。EngineAI 官方 SDK 及其他第三方
依赖仍分别遵循其原始许可证。
