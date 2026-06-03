"""Publishes TF + visualization Markers from /fleet/states so RViz2 renders the
fleet in real 3D (dogs on the ground, drones at altitude with a drop-line, plus
motion trails). RViz is the standard ROS 3D viewer — driving it with markers is
the lightweight way to get a real 3D scene without a physics sim.
"""
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, TransformStamped
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from fleetmind_msgs.msg import RobotState


class VizNode(Node):
    def __init__(self):
        super().__init__("viz_node")
        self.pub = self.create_publisher(MarkerArray, "/fleet/markers", 10)
        self.tfb = TransformBroadcaster(self)
        self.states = {}
        self.index = {}
        self.trails = {}
        self.create_subscription(RobotState, "/fleet/states", self._on_state, 10)
        self.create_timer(0.1, self._publish)
        self.get_logger().info("viz_node up — markers on /fleet/markers (frame: map)")

    def _on_state(self, msg):
        self.states[msg.robot_id] = msg
        self.index.setdefault(msg.robot_id, len(self.index))
        p = msg.pose.position
        tr = self.trails.setdefault(msg.robot_id, [])
        if tr:
            d2 = (p.x - tr[-1][0]) ** 2 + (p.y - tr[-1][1]) ** 2 + (p.z - tr[-1][2]) ** 2
            if d2 < 0.01:           # barely moved -> skip (keeps the line clean)
                return
            if d2 > 1.0:            # implausible jump (glitch/teleport) -> restart trail
                tr.clear()
        tr.append((p.x, p.y, p.z))
        if len(tr) > 200:
            tr.pop(0)

    def _publish(self):
        arr = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for rid, msg in self.states.items():
            idx = self.index[rid]
            p = msg.pose.position
            is_drone = msg.robot_type == "drone"
            col = (0.3, 0.8, 1.0) if is_drone else (0.4, 1.0, 0.4)

            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = "map"
            t.child_frame_id = rid
            t.transform.translation.x = p.x
            t.transform.translation.y = p.y
            t.transform.translation.z = p.z
            t.transform.rotation = msg.pose.orientation
            self.tfb.sendTransform(t)

            body = self._base(stamp, "body", idx, col)
            body.pose = msg.pose
            if is_drone:
                body.type = Marker.CYLINDER
                body.scale.x = body.scale.y = 0.7
                body.scale.z = 0.18
            else:
                body.type = Marker.CUBE
                body.scale.x, body.scale.y, body.scale.z = 0.7, 0.4, 0.35
            arr.markers.append(body)

            label = self._base(stamp, "label", idx, (1.0, 1.0, 1.0))
            label.type = Marker.TEXT_VIEW_FACING
            label.pose.position.x = p.x
            label.pose.position.y = p.y
            label.pose.position.z = p.z + 0.7
            label.scale.z = 0.4
            label.text = f"{rid} [{msg.current_task}]"
            arr.markers.append(label)

            if is_drone:
                drop = self._base(stamp, "alt", idx, (0.3, 0.8, 1.0, 0.4))
                drop.type = Marker.LINE_LIST
                drop.scale.x = 0.03
                drop.points = [Point(x=p.x, y=p.y, z=0.0), Point(x=p.x, y=p.y, z=p.z)]
                arr.markers.append(drop)

            tr = self.trails.get(rid, [])
            if len(tr) > 1:
                trail = self._base(stamp, "trail", idx, col + (0.6,))
                trail.type = Marker.LINE_STRIP
                trail.scale.x = 0.05
                trail.points = [Point(x=a, y=b, z=c) for a, b, c in tr]
                arr.markers.append(trail)
        self.pub.publish(arr)

    def _base(self, stamp, ns, idx, col):
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = stamp
        m.ns = ns
        m.id = idx
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.color.r, m.color.g, m.color.b = col[0], col[1], col[2]
        m.color.a = col[3] if len(col) > 3 else 1.0
        return m


def main():
    rclpy.init()
    node = VizNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
