"""Resolve the navigation map selected by the active MuJoCo process."""

import os
from pathlib import Path

import yaml


DEFAULT_STATE_FILE = Path(f"/tmp/engineai_active_scene_{os.getuid()}")


def read_active_scene(state_file=None):
    path = Path(state_file or os.environ.get(
        "ENGINEAI_SCENE_STATE_FILE", DEFAULT_STATE_FILE))
    if not path.is_file():
        raise RuntimeError(
            "No active MuJoCo scene was found. Start the simulation before navigation.")
    fields = {}
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        if "=" not in raw_line:
            continue
        key, value = raw_line.split("=", 1)
        if key in {"version", "scene", "robot", "pid", "sim_pid"}:
            fields[key] = value
    if fields.get("version") != "1" or not fields.get("scene"):
        raise RuntimeError(f"Invalid MuJoCo scene state file: {path}")
    try:
        process_id = int(fields["pid"])
        os.kill(process_id, 0)
    except (KeyError, ValueError, ProcessLookupError):
        raise RuntimeError(
            "The recorded MuJoCo process is no longer running. Restart the simulation.")
    except PermissionError:
        # A container/user boundary can prevent signalling an otherwise valid
        # process. The simulator state stream remains the authoritative check.
        pass
    return fields


def resolve_scene_map(package_share, state_file=None):
    package_share = Path(package_share)
    active = read_active_scene(state_file)
    manifest_path = package_share / "config" / "scene_maps.yaml"
    manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    map_name = manifest.get("scenes", {}).get(active["scene"])
    if not map_name:
        raise RuntimeError(
            f'MuJoCo scene "{active["scene"]}" has no saved navigation map. '
            "Build and register a map for this scene before starting navigation.")
    map_path = package_share / "maps" / map_name
    if not map_path.is_file():
        raise RuntimeError(
            f'Map registered for scene "{active["scene"]}" does not exist: {map_path}')
    return active, str(map_path)
