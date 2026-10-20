from glob import glob
from pathlib import Path
from setuptools import find_packages, setup

package_name = "t800_navigation"


def package_tree(source):
    """Install a configuration tree while preserving its relative layout."""
    entries = []
    for directory in sorted(path for path in Path(source).rglob("*") if path.is_dir()):
        files = sorted(str(path) for path in directory.iterdir() if path.is_file())
        if files:
            relative = directory.relative_to(source)
            entries.append((f"share/{package_name}/{source}/{relative}", files))
    root_files = sorted(str(path) for path in Path(source).iterdir() if path.is_file())
    if root_files:
        entries.append((f"share/{package_name}/{source}", root_files))
    return entries

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/maps", glob("maps/*")),
    ] + package_tree("config"),
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    entry_points={"console_scripts": [
        "elevation_mapper = t800_navigation.elevation_mapper:main",
        "single_goal = t800_navigation.single_goal:main",
    ]},
)
