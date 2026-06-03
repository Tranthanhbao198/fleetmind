#!/usr/bin/env python3
"""Standalone PX4 SITL smoke test over MAVSDK/MAVLink — proves the toolchain
works independent of the ROS2 fleet.

Run AFTER PX4 SITL is up (it listens on udp://:14540):
    /usr/bin/python3 tools/px4_test.py

Sequence: connect -> wait EKF health -> arm -> takeoff -> offboard -> fly a 5 m
square -> return to launch. Prints live position so you can see it move.
"""
import asyncio
import math

from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw


async def run():
    drone = System()
    print("connecting to udp://:14540 ...")
    await drone.connect(system_address="udp://:14540")

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("connected to PX4 SITL")
            break

    print("waiting for EKF position + home fix ...")
    async for h in drone.telemetry.health():
        if h.is_global_position_ok and h.is_home_position_ok:
            print("EKF ready")
            break

    # live position printer
    async def show():
        async for p in drone.telemetry.position_velocity_ned():
            n = p.position
            print(f"  pos NED  N={n.north_m:+.1f}  E={n.east_m:+.1f}  D={n.down_m:+.1f}")
            await asyncio.sleep(1.0)
    printer = asyncio.ensure_future(show())

    print("arming ..."); await drone.action.arm()
    print("takeoff ..."); await drone.action.set_takeoff_altitude(3.0)
    await drone.action.takeoff()
    await asyncio.sleep(8)

    print("starting offboard ...")
    await drone.offboard.set_position_ned(PositionNedYaw(0, 0, -3.0, 0))
    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(f"offboard start failed: {e._result.result}")
        await drone.action.land()
        return

    square = [(5, 0), (5, 5), (0, 5), (0, 0)]
    for (n, e) in square:
        print(f"goto N={n} E={e} (alt 3 m)")
        await drone.offboard.set_position_ned(PositionNedYaw(float(n), float(e), -3.0, 0.0))
        await asyncio.sleep(6)

    print("returning to launch (RTL) ...")
    await drone.offboard.stop()
    await drone.action.return_to_launch()
    await asyncio.sleep(15)
    printer.cancel()
    print("DONE — MAVSDK/MAVLink/offboard/RTL all worked.")


if __name__ == "__main__":
    asyncio.run(run())
