# robolink

Async Python SDK for industrial robot arm control via ROS2.

Built from real CR5 and UR5 fiber composite layup work at NUST.
Currently migrating from ROS1 Noetic to ROS2 Jazzy.

## Quickstart
```python
import asyncio
from robolink import ArmClient, JointState, LayupSequence

async def main():
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

## Requirements

- ROS2 Jazzy
- Python 3.10+

## Status

| Feature | Status |
|---|---|
| Joint state publishing via ROS2 | ✅ v0.1.0 |
| RViz visualization | ✅ v0.1.0 |
| MoveIt2 motion planning | 🔄 v0.2.0 |
| Real hardware (CR5 TCP) | 🔄 v0.2.0 |

## Background

Original ROS1 Noetic code for CR5 layup is in `legacy/`.
ROS2 migration is ongoing.
