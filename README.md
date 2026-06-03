# FleetMind — Multi-Robot Mission Control (ROS2)

A simulation-only mission-control system for a heterogeneous fleet (robot dogs +
drones) driven by **one operator** — no physical hardware required.

The point of the project is the **integration architecture**, not the robots:
one abstract adapter interface, swappable SDK backends, a unified task layer, and
a fleet manager — so a real PX4/Unitree/DJI SDK drops in without touching the
mission logic.

```
            AR / web UI  or  CLI  (operator)
                      │  /fleet/command  (FleetCommand)
                ┌─────▼──────┐
                │ fleet_      │   one operator -> many robots
                │ manager     │   fans out to each robot's mission action
                └──┬───────┬──┘
       /dog1/mission│       │/drone1/mission   (MissionCommand action)
            ┌────────▼─┐  ┌──▼────────┐
            │robot_node│  │robot_node │   task behaviors: WAYPOINT/FOLLOW/
            │  (dog1)  │  │ (drone1)  │   RETURN/STOP/PATROL, with preemption
            └────┬─────┘  └────┬──────┘
           adapter│            │adapter
            ┌──────▼──┐   ┌─────▼──────────┐
            │MockAdapt│   │Px4Adapter      │   <- swap SDK here, mission layer
            │(kinemat)│   │(MAVSDK/MAVLink)│      never changes
            └─────────┘   └────────────────┘
                      │ /fleet/states (RobotState @ 20 Hz) -> UI + FOLLOW
```

## Packages
- **fleetmind_msgs** — `MissionCommand.action`, `RobotState.msg`, `FleetCommand.msg`
- **fleetmind** — adapters, the robot node, the fleet manager, an operator CLI

## Key design decisions
- **Adapter pattern (`IRobotAdapter`)** — the mission/task layer only knows the
  interface; `MockAdapter` and `Px4Adapter` are interchangeable. Adding Unitree
  or DJI = one new adapter, zero changes upstream. This is the core "integration"
  story.
- **5 commands as a single ROS2 Action** — mission commands are long-running and
  need live feedback + cancellation, so Action is the right primitive (not Topic
  or Service).
- **Single-mission preemption** — a new goal preempts the running one (cooperative
  `threading.Event` + lock). Without this, a `STOP` would race a live `PATROL`
  loop and never actually stop the robot.
- **Namespaces for multi-robot** — each robot is `/<id>/...`; the fleet manager
  holds an action client per robot and broadcasts with `target: all`.

## Build
```bash
cd ~/fleetmind_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## Run
```bash
# terminal 1 — bring up 1 dog + 2 drones + fleet manager (all mock)
ros2 launch fleetmind fleet.launch.py

# terminal 2 — drive it (the ./fm wrapper sources ROS for you)
./fm dog1   waypoint 5 3            # go to (5,3)
./fm drone2 patrol 6 0 3 -6 0 3     # loop between two 3D waypoints
./fm drone1 follow dog1             # trail dog1 by 1.5 m, hover 2 m up
./fm dog1   return                  # back to launch position
./fm all    stop                    # stop everyone (preempts running missions)

# watch the fleet
ros2 topic echo /fleet/states
```
(`ros2 run` is unavailable in this install, so `./fm` calls the entry point
directly. `ros2 topic pub /fleet/command fleetmind_msgs/msg/FleetCommand ...`
also works.)

## AR operator console + FPV video
```bash
ros2 launch fleetmind demo.launch.py     # fleet + rosbridge(:9090) + web/FPV(:8080)
# open http://localhost:8080/  in a browser
```
The web console (`web/`, served by `fpv_server`) is the simulated **AR glasses**:
- a **2D map** of the fleet with live trails + headings (from `/fleet/states`),
- a **first-person MJPEG video** panel per robot (synthetic onboard cam + AR HUD,
  rendered by OpenCV in `fpv_server` — stands in for a real camera stream),
- **controls**: pick a robot (or `all`), click the map to send a WAYPOINT, or hit
  FOLLOW / PATROL / RETURN / STOP.

Browser ↔ ROS2 goes over **rosbridge** (`roslibjs` ⇄ `rosbridge_websocket`):
the page subscribes to `/fleet/states` and publishes `/fleet/command`, which the
fleet manager fans out to the robots — verified end-to-end.

`demo.launch.py` also starts **RViz2** for a real 3D view: `viz_node` turns
`/fleet/states` into TF + a `MarkerArray` on `/fleet/markers` — dogs as cubes on
the ground, drones as discs at altitude with drop-lines + motion trails + task
labels. Orbit/zoom with the mouse. Disable with `rviz:=false`.

## Run with PX4 SITL (optional)
> PX4 **SITL** is a software-in-the-loop simulator — the drone stays fully
> virtual, no hardware involved.
```bash
pip install mavsdk
# start PX4 SITL listening on udp://:14540, then:
ros2 launch fleetmind fleet.launch.py drone1_adapter:=px4
```
`Px4Adapter` arms, takes off, and uses offboard position setpoints + RTL — same
`IRobotAdapter` contract, so WAYPOINT/RETURN/STOP work unchanged.

## Future work
- Run the PX4 adapter against a live PX4 SITL instance
- WebRTC (low-latency) FPV in place of MJPEG
- docker-compose one-command bringup
