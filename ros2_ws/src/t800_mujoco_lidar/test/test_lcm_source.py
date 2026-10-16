import json
import time

from t800_mujoco_lidar.lcm_source import LatestState


def make_state(scenario="stage3_mixed", stamp=None):
    return json.dumps({
        "version": 3,
        "scenario": scenario,
        "stamp": time.time() if stamp is None else stamp,
    }).encode()


def test_accepts_scenario_reported_by_mujoco():
    latest = LatestState(timeout=0.8)
    latest.receive("lidar/state", make_state("stage3_office"))
    assert latest.get()["scenario"] == "stage3_office"


def test_newer_scenario_replaces_previous_state():
    latest = LatestState(timeout=0.8)
    now = time.time()
    latest.receive("lidar/state", make_state("stage3_office", now - 0.02))
    latest.receive("lidar/state", make_state("stage3_warehouse", now - 0.01))
    assert latest.get()["scenario"] == "stage3_warehouse"


def test_rejects_missing_scenario():
    latest = LatestState(timeout=0.8)
    payload = json.dumps({"version": 3, "scenario": "", "stamp": time.time()}).encode()
    latest.receive("lidar/state", payload)
    assert latest.get() is None
    assert latest.rejected == 1
