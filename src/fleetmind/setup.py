from glob import glob

from setuptools import find_packages, setup

package_name = "fleetmind"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/web", glob("web/*")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="thanhbao",
    maintainer_email="angyen3@gmail.com",
    description="FleetMind multi-robot mission control",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "robot_node = fleetmind.robot_node:main",
            "fleet_manager = fleetmind.fleet_manager:main",
            "cli = fleetmind.fleet_cli:main",
            "fpv_server = fleetmind.fpv_server:main",
            "viz_node = fleetmind.viz_node:main",
        ],
    },
)
