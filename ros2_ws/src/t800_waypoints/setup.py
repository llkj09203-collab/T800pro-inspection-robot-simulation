from glob import glob
from setuptools import find_packages, setup

package_name = "t800_waypoints"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/routes", glob("routes/*.yaml")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    entry_points={"console_scripts": [
        "waypoint_editor = t800_waypoints.editor_node:main",
        "mission_manager = t800_waypoints.mission_node:main",
    ]},
)
