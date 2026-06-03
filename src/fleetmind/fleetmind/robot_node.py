"""A single robot: wraps an adapter, runs the task behaviors, publishes state.

The 5 mission commands (WAYPOINT / FOLLOW / RETURN / STOP / PATROL) are exposed
as a single ROS2 Action because they are long-running and need live feedback +
cancellation — the right primitive here, not a topic or service.
"""
import math
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import Point
from fleetmind_msgs.action import MissionCommand
from fleetmind_msgs.msg import RobotState

from .mock_adapter import MockAdapter

STATES_TOPIC = "/fleet/states"   # absolute: every robot publishes here for the UI + follow


class RobotNode(Node):
    def __init__(self):
        super().__init__("robot_node")
        self.declare_parameter("robot_id", "dog1")
        self.declare_parameter("robot_type", "dog")
        self.declare_parameter("adapter", "mock")
        self.declare_parameter("speed", 1.5)
        self.declare_parameter("start", [0.0, 0.0, 0.0])

        self.robot_id = self.get_parameter("robot_id").value
        self.robot_type = self.get_parameter("robot_type").value
        adapter_kind = self.get_parameter("adapter").value
        speed = self.get_parameter("speed").value
        start = tuple(float(v) for v in self.get_parameter("start").value)

        if adapter_kind == "px4":
            from .px4_adapter import Px4Adapter
            self.adapter = Px4Adapter(self, speed=speed, start=start)
        else:
            self.adapter = MockAdapter(self.robot_type, speed, start)
        self.adapter.set_home(*start)

        self.current_task = "IDLE"
        self.battery = 100.0
        self._others = {}            # robot_id -> (x, y, z)
        self._last = self.get_clock().now()
        self._exec_lock = threading.Lock()   # only one mission runs at a time
        self._preempt = threading.Event()    # asks the running mission to yield
        self._epoch_lock = threading.Lock()
        self._goal_epoch = 0                 # newest goal wins; older queued goals are dropped

        cbg = ReentrantCallbackGroup()
        self.state_pub = self.create_publisher(RobotState, STATES_TOPIC, 10)
        self.create_subscription(RobotState, STATES_TOPIC, self._on_state, 10, callback_group=cbg)
        self.create_timer(0.05, self._tick, callback_group=cbg)
        self._server = ActionServer(
            self, MissionCommand, "mission",
            execute_callback=self._execute,
            goal_callback=lambda _g: GoalResponse.ACCEPT,
            cancel_callback=lambda _c: CancelResponse.ACCEPT,
            callback_group=cbg,
        )
        self.get_logger().info(f"[{self.robot_id}] {self.robot_type} up ({adapter_kind} adapter)")

    # ---- periodic: advance sim + broadcast state ----
    def _tick(self):
        now = self.get_clock().now()
        dt = (now - self._last).nanoseconds * 1e-9
        self._last = now
        self.adapter.step(dt)
        if self.current_task not in ("IDLE", "STOP"):
            self.battery = max(0.0, self.battery - 0.05 * dt)

        x, y, z, yaw = self.adapter.get_pose()
        msg = RobotState()
        msg.robot_id = self.robot_id
        msg.robot_type = self.robot_type
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = x, y, z
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        msg.current_task = self.current_task
        msg.active = self.current_task not in ("IDLE",)
        msg.battery = self.battery
        self.state_pub.publish(msg)

    def _on_state(self, msg):
        if msg.robot_id != self.robot_id:
            p = msg.pose.position
            self._others[msg.robot_id] = (p.x, p.y, p.z)

    # ---- the task behaviors ----
    def _execute(self, goal_handle):
        # Single-mission policy: a new goal preempts the running one. Ask the
        # current mission to yield, wait for the lock, then take over.
        with self._epoch_lock:
            self._goal_epoch += 1
            my_epoch = self._goal_epoch
        self._preempt.set()
        with self._exec_lock:
            # A still-newer goal arrived while we waited -> drop this stale one,
            # so a queued FOLLOW can't resume after a STOP.
            if my_epoch != self._goal_epoch:
                goal_handle.abort()
                r = MissionCommand.Result()
                r.success, r.message = False, "superseded"
                return r
            self._preempt.clear()
            return self._run_mission(goal_handle)

    def _run_mission(self, goal_handle):
        g = goal_handle.request
        cmd = g.command.upper()
        self.current_task = cmd
        result = MissionCommand.Result()
        self.get_logger().info(f"[{self.robot_id}] mission: {cmd}")

        try:
            if cmd == "STOP":
                self.adapter.stop()
                result.success, result.message = True, "stopped"

            elif cmd == "RETURN":
                self.adapter.return_home()
                if not self._run_to_target(goal_handle):
                    return self._terminate(goal_handle, result)
                result.success, result.message = True, "home"

            elif cmd in ("WAYPOINT", "PATROL"):
                wps = [(p.x, p.y, p.z) for p in g.waypoints]
                if not wps:
                    result.success, result.message = False, "no waypoints"
                else:
                    loop = g.loop or cmd == "PATROL"
                    done = False
                    while rclpy.ok() and not done:
                        for wp in wps:
                            self.adapter.goto(*wp)
                            if not self._run_to_target(goal_handle):
                                return self._terminate(goal_handle, result)
                        done = not loop
                    result.success, result.message = True, "waypoints done"

            elif cmd == "FOLLOW":
                tgt = g.follow_target
                while rclpy.ok():
                    if self._stop_requested(goal_handle):
                        return self._terminate(goal_handle, result)
                    pos = self._others.get(tgt)
                    if pos:
                        ox, oy, oz = pos
                        # trail 1.5 m behind on x; drone hovers 2 m up
                        alt = 2.0 if self.robot_type == "drone" else 0.0
                        self.adapter.goto(ox - 1.5, oy, alt or oz)
                    self._feedback(goal_handle, f"following {tgt}")
                    time.sleep(0.1)

            else:
                result.success, result.message = False, f"unknown command {cmd}"

        finally:
            self.current_task = "IDLE"

        goal_handle.succeed()
        return result

    def _stop_requested(self, goal_handle):
        return goal_handle.is_cancel_requested or self._preempt.is_set()

    def _run_to_target(self, goal_handle):
        """Poll until the adapter reaches its setpoint; False if it must stop."""
        while rclpy.ok():
            if self._stop_requested(goal_handle):
                return False
            if self.adapter.reached:
                return True
            self._feedback(goal_handle, "en route")
            time.sleep(0.1)
        return False

    def _feedback(self, goal_handle, status):
        x, y, z, _ = self.adapter.get_pose()
        fb = MissionCommand.Feedback()
        fb.status = status
        fb.distance_remaining = float(self.adapter.distance_remaining)
        fb.current_position = Point(x=x, y=y, z=z)
        goal_handle.publish_feedback(fb)

    def _terminate(self, goal_handle, result):
        """End a mission that was cancelled (by client) or preempted (new goal)."""
        self.adapter.stop()
        self.current_task = "IDLE"
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result.success, result.message = False, "canceled"
        else:
            goal_handle.abort()
            result.success, result.message = False, "preempted"
        return result


def main():
    rclpy.init()
    node = RobotNode()
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
