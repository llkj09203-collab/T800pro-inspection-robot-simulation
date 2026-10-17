from pathlib import Path

from t800_simulation.launching import find_sdk_root


def test_sdk_root_contains_simulator_script():
    root = find_sdk_root()
    assert (Path(root) / "scripts" / "run_mujoco.sh").is_file()
