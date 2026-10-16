# T800pro巡检机器人仿真开发

本仓库保存 T800 Pro 巡检机器人仿真项目自行开发的扩展代码，不包含 EngineAI
官方 SDK 的完整源码。项目基于 ROS 2 Humble、MuJoCo（机器人动力学仿真器）、
LCM（轻量通信机制）和 Nav2（ROS 2 导航框架），当前包括三维雷达、仿真桥接、
Stage3 建图导航与 RViz（ROS 可视化工具）配置。

## 仓库内容

```text
ros2_ws/src/
├── t800_mujoco_lidar       # MuJoCo 三维射线雷达与 PointCloud2 发布
├── t800_mujoco_bridge      # LCM/ROS 2 里程计、IMU、TF 和速度指令桥接
├── t800_stage3_navigation  # 建图、定位、点云转换与 Nav2 配置
└── t800_visualization      # 独立 RViz 配置
scenes/                     # 本项目 Stage3 场景和雷达参数
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
`335c60e`为基线生成，只包含本项目的接口扩展，不包含完整官方源码。

## 接入官方 SDK

```bash
# 1. 下载官方 SDK
git clone https://github.com/engineai-robotics/engineai_robotics_native_sdk.git
cd engineai_robotics_native_sdk

# 2. 应用本项目补丁（请替换扩展仓库的实际路径）
git apply /path/to/T800pro-inspection-robot-simulation/patches/engineai_native_sdk_integration.patch

# 3. 安装本项目场景文件
cp /path/to/T800pro-inspection-robot-simulation/scenes/environment/stage3_ground.xml \
  assets/resource/environment/
cp /path/to/T800pro-inspection-robot-simulation/scenes/t800pro_lidar_stage3_*.xml \
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

```bash
cd /path/to/T800pro-inspection-robot-simulation/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Stage3 导航启动

在 EngineAI 开发容器的四个终端中依次运行：

```bash
# 终端 1：MuJoCo Stage3 仿真
cd "$ENGINEAI_ROBOTICS_DIR"
ENGINEAI_LIDAR_SCENE=stage3_mixed bash scripts/run_mujoco.sh t800pro

# 终端 2：三维雷达
cd /path/to/T800pro-inspection-robot-simulation/ros2_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch t800_mujoco_lidar lidar.launch.py

# 终端 3：仿真桥接、二维激光转换、定位与导航
ros2 launch t800_stage3_navigation navigation.launch.py \
  map:=$(pwd)/src/t800_stage3_navigation/maps/stage3_groundfix.yaml

# 终端 4：RViz
ros2 launch t800_visualization rviz.launch.py
```

## 当前范围

当前成果用于仿真验证。雷达安装外参、传感器时间戳、噪声模型、机器人安全外廓、
速度限制及导航参数仍需依据真机接口和标定数据继续调整。

## 许可证

本仓库自行开发部分采用 BSD 3-Clause License。EngineAI 官方 SDK 及其他第三方
依赖仍分别遵循其原始许可证。
