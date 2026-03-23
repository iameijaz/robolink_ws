# robolink

Async Python SDK for industrial robot arm control via ROS2.

Built from real CR5 and UR5 fiber composite layup work at NUST (2023–2024).  
Currently migrating from ROS1 Noetic to ROS2 Jazzy — this is v0.1.0 of that migration.



## Demo

[<img width="531" height="430" alt="image" src="https://github.com/user-attachments/assets/6cf68e46-3f26-4c96-adc6-4609382eb4d5" />
]

CR5 arm executing a layup sequence — 8 waypoints, raster pattern,
controlled entirely from async Python with no manual intervention.

## Quickstart
```bash
# Terminal 1 — start RViz
source ~/robolink_ws/install/setup.bash
ros2 launch dobot_description sdk.launch.py

# Terminal 2 — run demo
cd robolink_ws/src/robolink
python3 examples/layup_demo.py
```
Motion sequences are programmed in V-Alpha Script, a domain-specific language I designed for robot arm control. See [README.md](https://github.com/iameijaz/robolink_ws/blob/jazzy/src/robolink/V_ALPHA_SCRIPT.md)  for the language reference and architecture.

Here is an example code in .vas:

```python
import asyncio
from robolink import ArmClient, JointState, LayupSequence

async def main():
    # Raster pattern — mirrors real fiber composite layup from NUST trials
    sequence = (
        LayupSequence("carbon_ply_1")
        .add_waypoint(JointState(j1=0.0, j2=-0.8, j3=0.5, j5=1.5))
        .add_waypoint(JointState(j1=0.0, j2=-0.6, j3=0.5, j5=1.5))
        .add_waypoint(JointState(j1=0.0, j2=-0.4, j3=0.5, j5=1.5))
        .add_waypoint(JointState(j1=0.0, j2=-0.2, j3=0.5, j5=1.5))
        .add_waypoint(JointState(j1=0.2, j2=-0.2, j3=0.5, j5=1.5))
        .add_waypoint(JointState(j1=0.2, j2=-0.4, j3=0.5, j5=1.5))
        .add_waypoint(JointState(j1=0.2, j2=-0.6, j3=0.5, j5=1.5))
        .add_waypoint(JointState(j1=0.2, j2=-0.8, j3=0.5, j5=1.5))
    )

    async with ArmClient() as arm:
        await arm.go_home()
        await arm.execute_layup(sequence)

asyncio.run(main())
```

## Architecture
```
User code (async Python)
        ↓
ArmClient — async context manager, typed API
        ↓
rclpy — publishes JointState to /joint_states
        ↓
robot_state_publisher (ROS2 Jazzy)
        ↓
RViz2 — CR5 arm visualization
```

The SDK is backend-agnostic by design. MoveIt2 motion planning
replaces direct joint publishing in v0.2.0 — the async API stays identical.

## Supported robots

| Robot | Status |
|---|---|
| Dobot CR5 | ✅ v0.1.0 — joint state publishing |
| Universal Robots UR5 | 🔄 v0.2.0 |

## Roadmap

| Feature | Version |
|---|---|
| Joint state publishing + RViz2 | ✅ v0.1.0 |
| MoveIt2 motion planning | 🔄 v0.2.0 |
| Real hardware (CR5 TCP/IP) | 🔄 v0.2.0 |
| UR5 backend | 🔄 v0.2.0 |

## Background

Original ROS1 Noetic implementation is in `legacy/` — real cartesian
path execution on CR5 and UR5 arms for fiber composite layup at NUST (2023–2024).
The ROS2 migration preserves the same motion patterns with a clean
async Python interface on top.

## Requirements

- ROS2 Jazzy (Ubuntu 24.04)
- Python 3.10+

## Installation
```bash
git clone https://github.com/iameijaz/robolink_ws.git
cd robolink_ws/src/robolink
python3 -m venv .venv --system-site-packages
source .venv/bin/activate
pip install -e ".[dev]"
```
