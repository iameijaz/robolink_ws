# robolink

Async Python SDK for industrial robot arm control via ROS2.

Built from real CR5 and UR5 fiber composite layup work at NUST (2023–2024).
Currently migrating from ROS1 Noetic to ROS2 Jazzy — this is v0.1.0 of that migration.

## Demo

[screenshot/GIF here]

CR5 arm executing a layup sequence — 8 waypoints, raster pattern,
controlled entirely from async Python with no manual intervention.

## Quickstart
```bash
# Terminal 1 — start RViz
source ~/robolink_ws/install/setup.bash
ros2 launch dobot_description sdk.launch.py

# Terminal 2 — run demo
pip install robolink
python3 examples/layup_demo.py
```
```python
import asyncio
from robolink import ArmClient, JointState, LayupSequence

async def main():
    # Sweep joint2 across 8 positions — mirrors real layup raster pattern
    sequence = LayupSequence.sweep(
        name="carbon_ply_1",
        axis="j2",
        start=-0.8,
        end=0.8,
        steps=8,
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
robot_state_publisher (ROS2)
        ↓
RViz — CR5 arm visualization
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
| Joint state publishing + RViz | ✅ v0.1.0 |
| MoveIt2 motion planning | 🔄 v0.2.0 |
| Real hardware (CR5 TCP/IP) | 🔄 v0.2.0 |
| UR5 backend | 🔄 v0.2.0 |

## Background

Original ROS1 Noetic implementation is in `legacy/` — real cartesian
path execution on CR5 and UR5 arms for fiber composite layup at NUST.
The ROS2 migration preserves the same motion patterns with a clean
async Python interface on top.

## Requirements

- ROS2 Jazzy (Ubuntu 24.04)
- Python 3.10+

## Installation
```bash
git clone https://github.com/iameijaz/robolink.git
cd robolink
python3 -m venv .venv --system-site-packages
source .venv/bin/activate
pip install -e ".[dev]"
```
