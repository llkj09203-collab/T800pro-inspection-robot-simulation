from glob import glob
from setuptools import find_packages, setup

package_name = "t800_stage3_navigation"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/config/glim_cpu", glob("config/glim_cpu/*.json")),
        ("share/" + package_name + "/maps", glob("maps/*")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    entry_points={"console_scripts": [
        "elevation_mapper = t800_stage3_navigation.elevation_mapper:main",
        "single_goal = t800_stage3_navigation.single_goal:main",
    ]},
)
