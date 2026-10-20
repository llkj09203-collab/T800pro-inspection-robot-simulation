from t800_simulation.launching import simulation_launch


def generate_launch_description():
    # Compatibility entry point. Prefer office.launch.py.
    return simulation_launch("office")
