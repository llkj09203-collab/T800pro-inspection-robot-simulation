"""Shared construction for scene-specific MuJoCo launch files."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo


def find_sdk_root():
    configured = os.environ.get("ENGINEAI_ROBOTICS_DIR")
    candidates = []
    if configured:
        candidates.append(Path(configured))
    candidates.append(Path.cwd())
    candidates.extend(Path.cwd().parents)
    try:
        share = Path(get_package_share_directory("t800_simulation"))
        candidates.extend(share.parents)
    except PackageNotFoundError:
        # Source-tree tests run before the package has entered the ament index.
        pass
    for candidate in candidates:
        if (candidate / "scripts" / "run_mujoco.sh").is_file():
            return candidate
    raise RuntimeError(
        "Cannot locate EngineAI SDK root containing scripts/run_mujoco.sh")


def simulation_launch(scene):
    root = find_sdk_root()
    command = ["bash", str(root / "scripts" / "run_mujoco.sh"),
               "t800pro", scene]
    return LaunchDescription([
        LogInfo(msg=f'Starting T800 Pro MuJoCo scene "{scene}"'),
        ExecuteProcess(cmd=command, output="screen"),
    ])
