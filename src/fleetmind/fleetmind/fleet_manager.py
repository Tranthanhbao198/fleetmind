"""Fleet manager: one operator -> many robots.

Listens on a single /fleet/command topic and fans each command out to the
matching robot mission action server(s). `target = "all"` broadcasts. This is the
multi-robot control layer the AR UI (or CLI) talks to.

Action clients are created up front from the `robot_ids` parameter — creating
them inside a spinning callback corrupts the executor wait-set in rclpy.
"""
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from fleetmind_msgs.action import MissionCommand
from fleetmind_msgs.msg import FleetCommand, RobotState


class FleetManager(Node):
    def __init__(self):
        super().__init__("fleet_manager")
        self.declare_parameter("robot_ids", ["dog1", "drone1", "drone2"])
        robot_ids = self.get_parameter("robot_ids").value

        self._cbg = ReentrantCallbackGroup()
        self._action_clients = {
            rid: ActionClient(self, MissionCommand, f"/{rid}/mission", callback_group=self._cbg)
            for rid in robot_ids
        }
        self._types = {}            # robot_id -> robot_type (for display)

        self.create_subscription(RobotState, "/fleet/states", self._on_state, 10, callback_group=self._cbg)
        self.create_subscription(FleetCommand, "/fleet/command", self._on_command, 10, callback_group=self._cbg)
        self.get_logger().info(f"fleet_manager up — managing {robot_ids}")

    def _on_state(self, msg):
        self._types[msg.robot_id] = msg.robot_type

    def _on_command(self, msg):
        targets = list(self._action_clients) if msg.target == "all" else [msg.target]
        for rid in targets:
            self._dispatch(rid, msg)

    def _dispatch(self, rid, fleet_cmd):
        client = self._action_clients.get(rid)
        if client is None:
            self.get_logger().warn(f"unknown robot {rid}")
            return
        if not client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn(f"{rid}: mission server not available")
            return
        goal = MissionCommand.Goal()
        goal.command = fleet_cmd.command
        goal.waypoints = fleet_cmd.waypoints
        goal.follow_target = fleet_cmd.follow_target
        goal.loop = fleet_cmd.loop
        client.send_goal_async(goal)
        self.get_logger().info(f"-> {rid}: {fleet_cmd.command}")


def main():
    rclpy.init()
    node = FleetManager()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
