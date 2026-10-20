import os
from pathlib import Path

import pytest

from t800_navigation.scene_selection import (
    read_active_scene, resolve_scene_map,
)


def write_state(path, scene, pid=None):
    path.write_text(
        f"version=1\nscene={scene}\nrobot=t800pro\npid={pid or os.getpid()}\n",
        encoding="utf-8")


def make_share(tmp_path):
    (tmp_path / "config" / "navigation").mkdir(parents=True)
    (tmp_path / "maps").mkdir()
    (tmp_path / "config" / "navigation" / "scene_maps.yaml").write_text(
        "version: 1\nscenes:\n  mixed: test_map.yaml\n",
        encoding="utf-8")
    (tmp_path / "maps" / "test_map.yaml").write_text(
        "image: test_map.pgm\n", encoding="utf-8")
    return tmp_path


def test_active_scene_selects_registered_map(tmp_path):
    state = tmp_path / "state"
    write_state(state, "mixed")
    active, selected = resolve_scene_map(make_share(tmp_path / "share"), state)
    assert active["scene"] == "mixed"
    assert selected.endswith("/maps/test_map.yaml")


def test_scene_without_saved_map_is_rejected(tmp_path):
    state = tmp_path / "state"
    write_state(state, "default")
    with pytest.raises(RuntimeError, match="has no saved navigation map"):
        resolve_scene_map(make_share(tmp_path / "share"), state)


def test_stale_simulator_process_is_rejected(tmp_path):
    state = tmp_path / "state"
    write_state(state, "mixed", 99999999)
    with pytest.raises(RuntimeError, match="no longer running"):
        read_active_scene(state)
