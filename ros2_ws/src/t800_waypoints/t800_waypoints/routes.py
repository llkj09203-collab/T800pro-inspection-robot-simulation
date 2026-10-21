"""Validated, portable YAML representation for T800 waypoint routes."""

from dataclasses import asdict, dataclass
import math
from pathlib import Path
import tempfile

import yaml


@dataclass
class Waypoint:
    name: str
    x: float
    y: float
    yaw: float = 0.0
    action: str = "none"
    stay_seconds: float = 0.0

    @classmethod
    def from_dict(cls, data):
        required = {"name", "x", "y"}
        missing = required.difference(data)
        if missing:
            raise ValueError(f"waypoint is missing: {', '.join(sorted(missing))}")
        waypoint = cls(
            name=str(data["name"]),
            x=float(data["x"]),
            y=float(data["y"]),
            yaw=float(data.get("yaw", 0.0)),
            action=str(data.get("action", "none")),
            stay_seconds=float(data.get("stay_seconds", 0.0)),
        )
        if not waypoint.name or any(not math.isfinite(value) for value in (
                waypoint.x, waypoint.y, waypoint.yaw, waypoint.stay_seconds)):
            raise ValueError("waypoint values must be finite and name must not be empty")
        if waypoint.stay_seconds < 0.0:
            raise ValueError("stay_seconds must not be negative")
        return waypoint


@dataclass
class Route:
    map: str
    frame_id: str
    waypoints: list


def load_route(path):
    path = Path(path).expanduser()
    if not path.is_file():
        return Route(map="", frame_id="map", waypoints=[])
    document = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if document.get("version") != 1:
        raise ValueError(f"unsupported route version in {path}")
    waypoints = [Waypoint.from_dict(item) for item in document.get("waypoints", [])]
    names = [waypoint.name for waypoint in waypoints]
    if len(names) != len(set(names)):
        raise ValueError("waypoint names must be unique")
    return Route(
        map=str(document.get("map", "")),
        frame_id=str(document.get("frame_id", "map")),
        waypoints=waypoints,
    )


def save_route(path, route):
    path = Path(path).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    document = {
        "version": 1,
        "map": route.map,
        "frame_id": route.frame_id,
        "waypoints": [asdict(waypoint) for waypoint in route.waypoints],
    }
    with tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", dir=path.parent, delete=False) as stream:
        yaml.safe_dump(document, stream, sort_keys=False, allow_unicode=True)
        temporary = Path(stream.name)
    temporary.replace(path)


def quaternion_from_yaw(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def yaw_from_quaternion(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )
