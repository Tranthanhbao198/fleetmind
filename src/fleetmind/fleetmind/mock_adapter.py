"""Lightweight kinematic mock of a robot — no Gazebo needed.

Integrates a constant-speed point toward its setpoint. A `dog` is constrained to
the ground plane (z=0); a `drone` may move in 3D.
"""
import math

from .adapter_base import IRobotAdapter


class MockAdapter(IRobotAdapter):
    def __init__(self, robot_type="dog", speed=1.5, start=(0.0, 0.0, 0.0)):
        self.robot_type = robot_type
        self.speed = float(speed)
        self.x, self.y, self.z = (float(v) for v in start)
        self.yaw = 0.0
        self.tx, self.ty, self.tz = self.x, self.y, self.z
        self.home = (self.x, self.y, self.z)
        self._tol = 0.15

    def get_pose(self):
        return (self.x, self.y, self.z, self.yaw)

    def goto(self, x, y, z):
        self.tx = float(x)
        self.ty = float(y)
        self.tz = float(z) if self.robot_type == "drone" else 0.0

    def stop(self):
        self.tx, self.ty, self.tz = self.x, self.y, self.z

    def set_home(self, x, y, z):
        self.home = (float(x), float(y), float(z) if self.robot_type == "drone" else 0.0)

    def return_home(self):
        self.goto(*self.home)

    @property
    def distance_remaining(self):
        return math.dist((self.x, self.y, self.z), (self.tx, self.ty, self.tz))

    @property
    def reached(self):
        return self.distance_remaining < self._tol

    def step(self, dt):
        d = self.distance_remaining
        if d < 1e-6:
            return
        move = min(self.speed * dt, d)
        self.x += (self.tx - self.x) / d * move
        self.y += (self.ty - self.y) / d * move
        self.z += (self.tz - self.z) / d * move
        if d > self._tol:
            self.yaw = math.atan2(self.ty - self.y, self.tx - self.x)
