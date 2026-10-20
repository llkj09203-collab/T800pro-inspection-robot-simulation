from pathlib import Path

import yaml


def load_params():
    package_root = Path(__file__).resolve().parents[1]
    return yaml.safe_load(
        (package_root / "config" / "navigation" / "nav2_params.yaml")
        .read_text(encoding="utf-8"))


def test_navigation_is_forward_only():
    params = load_params()
    smoother = params["velocity_smoother"]["ros__parameters"]
    assert smoother["max_velocity"] == [0.5, 0.0, 0.5]
    assert smoother["min_velocity"] == [0.0, 0.0, -0.5]


def test_rotation_shim_wraps_dwb_controller():
    params = load_params()
    controller = params["controller_server"]["ros__parameters"]["FollowPath"]
    assert controller["plugin"] == (
        "nav2_rotation_shim_controller::RotationShimController")
    assert controller["primary_controller"] == "dwb_core::DWBLocalPlanner"
    assert controller["angular_dist_threshold"] == 0.785398
    assert controller["rotate_to_goal_heading"] is False


def test_amcl_and_navfn_are_explicit():
    params = load_params()
    assert params["amcl"]["ros__parameters"]["base_frame_id"] == "base_link"
    planner = params["planner_server"]["ros__parameters"]["GridBased"]
    assert planner["plugin"] == "nav2_navfn_planner/NavfnPlanner"
