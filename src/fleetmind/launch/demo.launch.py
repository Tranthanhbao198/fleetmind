"""Full demo: fleet + rosbridge (web<->ROS bridge) + FPV/console server.

    ros2 launch fleetmind demo.launch.py
    # then open http://localhost:8080/

Brings up the same fleet as fleet.launch.py, plus:
  - rosbridge_websocket on :9090  (the web console subscribes/publishes here)
  - fpv_server on :8080           (serves the console + MJPEG FPV streams)
  - viz_node + RViz2              (real 3D view of the fleet; disable with rviz:=false)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _robot(robot_id, robot_type, start, speed=1.5):
    return Node(
        package="fleetmind", executable="robot_node",
        namespace=robot_id, name="robot_node",
        parameters=[{"robot_id": robot_id, "robot_type": robot_type,
                     "adapter": "mock", "speed": speed, "start": start}],
        output="screen",
    )


def generate_launch_description():
    rviz_cfg = os.path.join(get_package_share_directory("fleetmind"), "rviz", "fleetmind.rviz")
    use_rviz = LaunchConfiguration("rviz")
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true", description="launch RViz2 3D view"),
        Node(package="rosbridge_server", executable="rosbridge_websocket",
             name="rosbridge_websocket", output="screen",
             parameters=[{"port": 9090}]),
        Node(package="fleetmind", executable="fleet_manager", name="fleet_manager",
             output="screen", parameters=[{"robot_ids": ["dog1", "drone1", "drone2"]}]),
        Node(package="fleetmind", executable="fpv_server", name="fpv_server",
             output="screen", parameters=[{"port": 8080}]),
        Node(package="fleetmind", executable="viz_node", name="viz_node", output="screen"),
        Node(package="rviz2", executable="rviz2", name="rviz2",
             arguments=["-d", rviz_cfg], output="screen",
             condition=IfCondition(use_rviz)),
        _robot("dog1", "dog", [0.0, 0.0, 0.0], speed=1.5),
        _robot("drone1", "drone", [2.0, 0.0, 2.0], speed=3.0),
        _robot("drone2", "drone", [-2.0, 0.0, 2.0], speed=3.0),
    ])
