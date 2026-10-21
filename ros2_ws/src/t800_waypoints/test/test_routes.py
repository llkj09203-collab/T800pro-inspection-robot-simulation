import math

import pytest

from t800_waypoints.routes import (
    Route, Waypoint, load_route, quaternion_from_yaw, save_route,
    yaw_from_quaternion,
)


def test_route_round_trip(tmp_path):
    path = tmp_path / "route.yaml"
    expected = Route("mixed_map", "map", [
        Waypoint("P01", 1.0, -2.0, 0.5, "wait", 3.0),
    ])
    save_route(path, expected)
    assert load_route(path) == expected


def test_missing_route_is_empty(tmp_path):
    assert load_route(tmp_path / "missing.yaml") == Route("", "map", [])


def test_duplicate_names_are_rejected(tmp_path):
    path = tmp_path / "route.yaml"
    path.write_text(
        "version: 1\nmap: mixed_map\nframe_id: map\nwaypoints:\n"
        "- {name: P01, x: 0, y: 0}\n- {name: P01, x: 1, y: 1}\n",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="unique"):
        load_route(path)


def test_yaw_round_trip():
    yaw = -1.2
    x, y, z, w = quaternion_from_yaw(yaw)
    quaternion = type("Quaternion", (), {"x": x, "y": y, "z": z, "w": w})()
    assert math.isclose(yaw_from_quaternion(quaternion), yaw)
