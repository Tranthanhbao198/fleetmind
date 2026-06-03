"""Real-drone adapter: PX4 SITL over MAVLink via MAVSDK.

Same IRobotAdapter contract as MockAdapter, so the mission layer is unchanged.
MAVSDK is asyncio-based, so we run an event loop in a background thread and the
synchronous adapter methods just hand coroutines to it.

Optional. Requires:  pip install mavsdk  + a running PX4 SITL instance
(default udp://:14540). Until then, robots run with the mock adapter.

Positions use a local NED-ish convention (x,y,z meters from launch); we convert
to PX4 offboard NED setpoints. Good enough for a demo, not survey-grade.
"""
import asyncio
import math
import threading

from .adapter_base import IRobotAdapter


class Px4Adapter(IRobotAdapter):
    def __init__(self, node, speed=3.0, start=(0.0, 0.0, 0.0), url="udp://:14540"):
        self._node = node
        self.speed = speed
        self.url = url
        self._pos = list(start)          # latest telemetry (x, y, z)
        self._yaw = 0.0
        self._target = list(start)
        self.home = list(start)
        self._tol = 0.6
        self._ready = False

        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()
        self._submit(self._connect())

    # ---- asyncio plumbing ----
    def _run_loop(self):
        asyncio.set_event_loop(self._loop)
        self._loop.run_forever()

    def _submit(self, coro):
        return asyncio.run_coroutine_threadsafe(coro, self._loop)

    async def _connect(self):
        from mavsdk import System
        from mavsdk.offboard import PositionNedYaw, OffboardError
        self._PositionNedYaw = PositionNedYaw
        self._OffboardError = OffboardError
        self._drone = System()
        await self._drone.connect(system_address=self.url)
        async for state in self._drone.core.connection_state():
            if state.is_connected:
                break
        # PX4 rejects arm/offboard until the EKF has a position + home fix.
        async for h in self._drone.telemetry.health():
            if h.is_global_position_ok and h.is_home_position_ok:
                break
        self._loop.create_task(self._telemetry())
        self._loop.create_task(self._attitude())

        await self._drone.action.arm()
        await self._drone.action.set_takeoff_altitude(max(2.0, self.home[2]))
        await self._drone.action.takeoff()
        await asyncio.sleep(8)
        # offboard needs a setpoint streaming before it will start
        await self._drone.offboard.set_position_ned(self._PositionNedYaw(0, 0, -2.0, 0))
        try:
            await self._drone.offboard.start()
        except self._OffboardError as e:
            print(f"[px4] offboard start failed: {e}")
        self._ready = True

    async def _telemetry(self):
        async for p in self._drone.telemetry.position_velocity_ned():
            n = p.position
            # NED (north, east, down) -> our (x=north, y=east, z=up)
            self._pos = [n.north_m, n.east_m, -n.down_m]

    async def _attitude(self):
        async for a in self._drone.telemetry.attitude_euler():
            self._yaw = math.radians(a.yaw_deg)

    # ---- IRobotAdapter ----
    def get_pose(self):
        return (self._pos[0], self._pos[1], self._pos[2], self._yaw)

    def goto(self, x, y, z):
        self._target = [float(x), float(y), float(z)]
        if self._ready:
            self._submit(self._drone.offboard.set_position_ned(
                self._PositionNedYaw(float(x), float(y), -float(z), 0.0)))

    def stop(self):
        self.goto(*self._pos)

    def set_home(self, x, y, z):
        self.home = [float(x), float(y), float(z)]

    def return_home(self):
        if self._ready:
            self._submit(self._drone.action.return_to_launch())

    @property
    def distance_remaining(self):
        return math.dist(self._pos, self._target)

    @property
    def reached(self):
        return self._ready and self.distance_remaining < self._tol

    def step(self, dt):
        pass  # PX4 flies itself
