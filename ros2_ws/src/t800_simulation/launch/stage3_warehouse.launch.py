from t800_simulation.launching import simulation_launch


def generate_launch_description():
    # Compatibility entry point. Prefer warehouse.launch.py.
    return simulation_launch("warehouse")
