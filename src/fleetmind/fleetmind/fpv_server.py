"""FPV video server: renders a synthetic first-person view per robot and serves
it as MJPEG over HTTP — standing in for a real onboard camera streamed to AR
glasses. Also serves the web operator console (static files) on the same port.

  http://localhost:8080/                  -> AR operator console
  http://localhost:8080/stream?robot=dog1 -> MJPEG FPV for that robot

The view is stylised (horizon + scrolling perspective grid + other-robot blips +
an AR HUD), driven live by /fleet/states. No camera, no Gazebo needed.
"""
import math
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from fleetmind_msgs.msg import RobotState

W, H = 640, 360
HORIZON = 150
FOCAL = 320.0


def _yaw_from_quat(z, w):
    return 2.0 * math.atan2(z, w)


def render(target_id, states):
    """Return a JPEG (bytes) of the FPV for target_id given all robot states."""
    me = states.get(target_id)
    img = np.zeros((H, W, 3), np.uint8)
    # sky / ground
    img[:HORIZON] = (60, 35, 20)
    img[HORIZON:] = (25, 30, 25)

    if me is None:
        cv2.putText(img, f"NO SIGNAL: {target_id}", (W // 2 - 120, H // 2),
                    cv2.FONT_HERSHEY_DUPLEX, 0.8, (60, 60, 200), 2)
        return _encode(img)

    x, y, z, yaw = me["x"], me["y"], me["z"], me["yaw"]
    cx = W // 2

    # perspective ground grid, scrolling with forward travel
    along = x * math.cos(yaw) + y * math.sin(yaw)
    for i in range(1, 11):
        d = i - (along % 1.0)
        if d <= 0:
            continue
        sy = int(HORIZON + FOCAL * 0.5 / d)
        if HORIZON < sy < H:
            shade = max(30, 90 - i * 6)
            cv2.line(img, (0, sy), (W, sy), (shade, shade, shade), 1)
    for gx in range(-6, 7):
        # vertical grid lines fan out from the vanishing point
        x2 = cx + int(gx * FOCAL / 1.0)
        cv2.line(img, (cx, HORIZON), (x2, H), (45, 55, 45), 1)
    cv2.line(img, (0, HORIZON), (W, HORIZON), (120, 140, 120), 1)

    # other robots as blips projected into view
    for rid, s in states.items():
        if rid == target_id:
            continue
        dx, dy = s["x"] - x, s["y"] - y
        fwd = dx * math.cos(yaw) + dy * math.sin(yaw)
        rt = -dx * math.sin(yaw) + dy * math.cos(yaw)
        if fwd < 0.5:
            continue
        sx = int(cx + (rt / fwd) * FOCAL)
        sy = int(HORIZON + (FOCAL * 0.5) / fwd)
        if 0 < sx < W and HORIZON < sy < H:
            col = (80, 200, 255) if s["type"] == "drone" else (120, 255, 120)
            r = max(4, int(40 / fwd))
            cv2.rectangle(img, (sx - r, sy - r), (sx + r, sy + r), col, 2)
            cv2.putText(img, f"{rid} {fwd:.0f}m", (sx - r, sy - r - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, col, 1)

    _hud(img, me, target_id)
    return _encode(img)


def _hud(img, me, target_id):
    g = (90, 230, 120)
    cv2.rectangle(img, (2, 2), (W - 3, H - 3), g, 1)
    cv2.line(img, (W // 2 - 12, H // 2), (W // 2 + 12, H // 2), g, 1)
    cv2.line(img, (W // 2, H // 2 - 12), (W // 2, H // 2 + 12), g, 1)
    cv2.putText(img, f"{target_id.upper()} [{me['type']}]  CAM-1", (10, 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, g, 1)
    cv2.putText(img, f"TASK {me['task']}", (10, H - 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, g, 1)
    cv2.putText(img, f"X{me['x']:+.1f} Y{me['y']:+.1f} Z{me['z']:+.1f}  HDG{math.degrees(me['yaw']) % 360:03.0f}",
                (10, H - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, g, 1)
    bat = int(me["battery"])
    cv2.putText(img, f"BAT {bat}%", (W - 110, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, g, 1)
    cv2.rectangle(img, (W - 115, 30), (W - 15, 40), g, 1)
    cv2.rectangle(img, (W - 114, 31), (W - 115 + bat, 39), g, -1)
    cv2.circle(img, (W - 24, H - 24), 6, (60, 60, 230), -1)
    cv2.putText(img, "REC", (W - 70, H - 19), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (60, 60, 230), 1)


def _encode(img):
    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 70])
    return buf.tobytes()


def make_handler(node, web_dir):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def _send_static(self, path):
            rel = path.lstrip("/") or "index.html"
            full = os.path.join(web_dir, rel)
            if not os.path.isfile(full):
                self.send_error(404)
                return
            ctype = ("text/html" if full.endswith(".html")
                     else "application/javascript" if full.endswith(".js")
                     else "text/css" if full.endswith(".css") else "text/plain")
            with open(full, "rb") as f:
                data = f.read()
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            parsed = urlparse(self.path)
            if parsed.path != "/stream":
                self._send_static(parsed.path)
                return
            robot = parse_qs(parsed.query).get("robot", ["dog1"])[0]
            self.send_response(200)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                while True:
                    with node.lock:
                        snapshot = dict(node.states)
                    frame = render(robot, snapshot)
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                     b"Content-Length: " + str(len(frame)).encode()
                                     + b"\r\n\r\n" + frame + b"\r\n")
                    time.sleep(1 / 15.0)
            except (BrokenPipeError, ConnectionResetError):
                pass

    return Handler


class FpvServer(Node):
    def __init__(self):
        super().__init__("fpv_server")
        self.declare_parameter("port", 8080)
        self.declare_parameter("web_dir", "")
        self.states = {}
        self.lock = threading.Lock()
        self.create_subscription(RobotState, "/fleet/states", self._on_state, 10)

        web_dir = self.get_parameter("web_dir").value
        if not web_dir:
            from ament_index_python.packages import get_package_share_directory
            web_dir = os.path.join(get_package_share_directory("fleetmind"), "web")
        port = self.get_parameter("port").value
        httpd = ThreadingHTTPServer(("0.0.0.0", port), make_handler(self, web_dir))
        threading.Thread(target=httpd.serve_forever, daemon=True).start()
        self.get_logger().info(f"console: http://localhost:{port}/  | web_dir={web_dir}")

    def _on_state(self, msg):
        with self.lock:
            self.states[msg.robot_id] = {
                "x": msg.pose.position.x,
                "y": msg.pose.position.y,
                "z": msg.pose.position.z,
                "yaw": _yaw_from_quat(msg.pose.orientation.z, msg.pose.orientation.w),
                "type": msg.robot_type,
                "task": msg.current_task,
                "battery": msg.battery,
            }


def main():
    rclpy.init()
    node = FpvServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
