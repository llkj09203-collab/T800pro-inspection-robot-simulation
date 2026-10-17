import os
from pathlib import Path

import pytest

from t800_stage3_navigation.scene_selection import (
    read_active_scene, resolve_scene_map,
)


def write_state(path, scene, pid=None):
    path.write_text(
        f"version=1\nscene={scene}\nrobot=t800pro\npid={pid or os.getpid()}\n",
        encoding="utf-8")


def make_share(tmp_path):
    (tmp_path / "config").mkdir(parents=True)
    (tmp_path / "maps").mkdir()
    (tmp_path / "config" / "scene_maps.yaml").write_text(
        "version: 1\nscenes:\n  stage3_mixed: stage3.yaml\n",
        encoding="utf-8")
    (tmp_path / "maps" / "stage3.yaml").write_text(
        "image: stage3.pgm\n", encoding="utf-8")
    return tmp_path


def test_active_scene_selects_registered_map(tmp_path):
    state = tmp_path / "state"
    write_state(state, "stage3_mixed")
    active, selected = resolve_scene_map(make_share(tmp_path / "share"), state)
    assert active["scene"] == "stage3_mixed"
    assert selected.endswith("/maps/stage3.yaml")


def test_scene_without_saved_map_is_rejected(tmp_path):
    state = tmp_path / "state"
    write_state(state, "default")
    with pytest.raises(RuntimeError, match="has no saved navigation map"):
        resolve_scene_map(make_share(tmp_path / "share"), state)


def test_stale_simulator_process_is_rejected(tmp_path):
    state = tmp_path / "state"
    write_state(state, "stage3_mixed", 99999999)
    with pytest.raises(RuntimeError, match="no longer running"):
        read_active_scene(state)
