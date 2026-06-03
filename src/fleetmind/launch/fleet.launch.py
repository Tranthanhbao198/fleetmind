"""Bring up a fleet: 1 dog + 2 drones (all mock) + the fleet manager.

Each robot runs in its own namespace so its action server is /<id>/mission.
Set drone1's adapter to px4 to drive a real PX4 SITL instance:
    ros2 launch fleetmind fleet.launch.py drone1_adapter:=px4
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _robot(robot_id, robot_type, start, adapter="mock", speed=1.5):
    return Node(
        package="fleetmind",
        executable="robot_node",
        namespace=robot_id,
        name="robot_node",
        parameters=[{
            "robot_id": robot_id,
            "robot_type": robot_type,
            "adapter": adapter,
            "speed": speed,
            "start": start,
        }],
        output="screen",
    )


def generate_launch_description():
    drone1_adapter = LaunchConfiguration("drone1_adapter")
    return LaunchDescription([
        DeclareLaunchArgument("drone1_adapter", default_value="mock",
                              description="adapter for drone1: mock | px4"),
        Node(package="fleetmind", executable="fleet_manager",
             name="fleet_manager", output="screen",
             parameters=[{"robot_ids": ["dog1", "drone1", "drone2"]}]),
        _robot("dog1", "dog", [0.0, 0.0, 0.0], speed=1.5),
        _robot("drone1", "drone", [2.0, 0.0, 2.0], adapter=drone1_adapter, speed=3.0),
        _robot("drone2", "drone", [-2.0, 0.0, 2.0], speed=3.0),
    ])
