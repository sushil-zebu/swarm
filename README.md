<<<<<<< HEAD
# swarm
=======
# Decentralized Drone Swarm ROS 2 Workspace

ROS 2 workspace for experimenting with decentralized PX4 drone swarm control in
SITL. Each drone runs its own controller node, exchanges peer state, and sends
PX4 Offboard setpoints through a shared safety layer.

The main package is `swarm_core`.

## Features

- Multi-drone PX4 SITL with Gazebo
- One ROS 2 controller node per drone
- Peer-to-peer swarm state broadcasting
- Control Barrier Function collision avoidance
- Velocity and acceleration limiting before PX4 commands
- Mission-based controller structure using `FlightMission`
- Example missions: triangle formation, helix stack, CBF arming test

## Repository Layout

| Path | Description |
|---|---|
| `src/swarm_core` | Main C++ swarm controller package |
| `src/swarm_interfaces` | Custom ROS 2 messages/actions |
| `src/px4_msgs` | PX4 message definitions |
| `src/px4_ros_com` | PX4 ROS 2 communication support |
| `src/swarm_bringup` | Bringup package |
| `start_sitl.sh` | Starts PX4 SITL vehicles and XRCE agents |
| `DEVELOPER_DOC.md` | Detailed developer onboarding notes |

## Quick Start

Build the main package:

```bash
cd ~/sushil/de-swarm/swarm_ros_ws
colcon build --packages-select swarm_core
source install/setup.bash
```

Start PX4 SITL and Gazebo:

```bash
./start_sitl.sh
```

Launch the swarm controller nodes:

```bash
ros2 launch swarm_core swarm_fleet.launch.py
```

Run the CBF unit test:

```bash
colcon test --packages-select swarm_core --ctest-args -R test_cbf_avoidance
```

## System Overview

```text
Gazebo + PX4 SITL
        |
        v
PX4 ROS 2 topics
        |
        v
DroneControllerNode
        |
        +-- PX4Interface
        +-- PeerTracker
        +-- CbfAvoidance
        +-- FlightMission
```

`DroneControllerNode` owns the shared infrastructure. Mission behavior lives in
separate mission files such as `mission_triangle.cpp`, `mission_helix.cpp`, and
`cbf_test.cpp`.

## Fleet Mapping

| Drone | Namespace | PX4 instance | MAV system ID | Gazebo model |
|---|---|---:|---:|---|
| 1 | `/drone_1` | 1 | 2 | `x500_1` |
| 2 | `/drone_2` | 2 | 3 | `x500_2` |
| 3 | `/drone_3` | 3 | 4 | `x500_3` |
| 4 | `/drone_4` | 4 | 5 | `x500_4` |

## Configuration

Main parameters live in:

```text
src/swarm_core/config/swarm_params.yaml
```

Fleet identity and spawn offsets live in:

```text
src/swarm_core/launch/swarm_fleet.launch.py
```

The active mission is currently selected in:

```text
src/swarm_core/src/drone_controller_node.cpp
```

## Notes

This workspace is currently SITL-focused. Before using this structure on real
hardware, verify frame conventions, failsafes, geofence behavior, RC/manual
override, peer-loss behavior, and conservative speed/acceleration limits.

For deeper architecture notes, mission-writing steps, parameters, and debugging
commands, see [DEVELOPER_DOC.md](DEVELOPER_DOC.md).
>>>>>>> 1e25e6b (initial push of swarm)
