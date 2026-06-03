"""Abstract robot adapter.

This is the heart of the integration design: the mission/task layer only ever
talks to this interface, so swapping a real SDK (PX4/MAVSDK, Unitree, DJI) in
place of the mock never touches the behavior logic. Write one adapter per SDK.
"""
from abc import ABC, abstractmethod


class IRobotAdapter(ABC):
    @abstractmethod
    def get_pose(self):
        """Return current pose as (x, y, z, yaw)."""

    @abstractmethod
    def goto(self, x, y, z):
        """Command the robot toward a position setpoint (non-blocking)."""

    @abstractmethod
    def stop(self):
        """Hold current position / zero velocity."""

    @abstractmethod
    def return_home(self):
        """Go back to the stored home/launch position."""

    @abstractmethod
    def set_home(self, x, y, z):
        """Store the home/launch position."""

    @abstractmethod
    def step(self, dt):
        """Advance internal simulation by dt seconds.

        Real SDK adapters (PX4 etc.) move on their own, so this is a no-op there.
        """

    @property
    @abstractmethod
    def reached(self):
        """True when the current setpoint has been reached."""

    @property
    @abstractmethod
    def distance_remaining(self):
        """Distance to current setpoint, in meters."""
