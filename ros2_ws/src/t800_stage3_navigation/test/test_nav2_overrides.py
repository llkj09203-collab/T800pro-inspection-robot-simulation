from pathlib import Path

import yaml


def load_overrides():
    package_root = Path(__file__).resolve().parents[1]
    return yaml.safe_load(
        (package_root / "config" / "nav2_overrides.yaml").read_text())


def test_navigation_is_forward_only():
    replacements = load_overrides()["text_replacements"]
    assert replacements["min_velocity: [-0.26, 0.0, -1.0]"] == (
        "min_velocity: [0.0, 0.0, -0.5]")


def test_rotation_shim_wraps_dwb_controller():
    replacements = load_overrides()["text_replacements"]
    controller = replacements['      plugin: "dwb_core::DWBLocalPlanner"']
    assert "nav2_rotation_shim_controller::RotationShimController" in controller
    assert 'primary_controller: "dwb_core::DWBLocalPlanner"' in controller
    assert "angular_dist_threshold: 0.785398" in controller
    assert "rotate_to_goal_heading: false" in controller
