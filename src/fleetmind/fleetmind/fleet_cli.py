"""Tiny operator CLI: send a mission command to the fleet in one line.

  ros2 run fleetmind cli <target> <command> [numbers... | follow_target]

Examples:
  ros2 run fleetmind cli dog1   waypoint 5 3
  ros2 run fleetmind cli drone2 patrol 6 0 3 -6 0 3
  ros2 run fleetmind cli drone1 follow dog1
  ros2 run fleetmind cli dog1   return
  ros2 run fleetmind cli all    stop
"""
import sys
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from fleetmind_msgs.msg import FleetCommand


def _parse(argv):
    if len(argv) < 2:
        print(__doc__)
        sys.exit(1)
    msg = FleetCommand()
    msg.target = argv[0]
    msg.command = argv[1].upper()
    rest = argv[2:]
    if msg.command == "FOLLOW":
        msg.follow_target = rest[0] if rest else ""
    elif msg.command in ("WAYPOINT", "PATROL"):
        nums = [float(v) for v in rest]
        # group into (x, y, z) triples; pad missing z with 0
        for i in range(0, len(nums), 3):
            chunk = nums[i:i + 3]
            while len(chunk) < 3:
                chunk.append(0.0)
            msg.waypoints.append(Point(x=chunk[0], y=chunk[1], z=chunk[2]))
        msg.loop = msg.command == "PATROL"
    return msg


def main():
    rclpy.init()
    node = Node("fleet_cli")
    pub = node.create_publisher(FleetCommand, "/fleet/command", 10)
    msg = _parse(sys.argv[1:])
    # wait for fleet_manager to subscribe so the message isn't dropped
    for _ in range(50):
        if pub.get_subscription_count() > 0:
            break
        rclpy.spin_once(node, timeout_sec=0.1)
    pub.publish(msg)
    time.sleep(0.3)
    node.get_logger().info(f"sent {msg.command} -> {msg.target}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
