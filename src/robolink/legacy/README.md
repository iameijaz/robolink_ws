# Legacy — ROS1 Noetic CR5 code

Original fiber composite layup scripts from real CR5 and UR5 hardware
runs at NUST Pakistan (Nov 2023 – Mar 2024).

Stack: ROS1 Noetic + moveit_commander + catkin (Ubuntu 20.04)

This is the code that motivated robolink. The pain of working with
moveit_commander directly — no clean API, no async, no typed models —
is exactly what the SDK solves.

## What this does
- `cr5_layup_ros1.py` — cartesian raster path for fiber composite layup
  on real CR5 arm. Includes go_to_joint_state(), plan_cartesian_path(),
  and execute_plan().

## Why it's here
robolink v0.2.0 will reintroduce MoveIt2 motion planning as a backend,
replacing the direct joint state publishing in v0.1.0.
The architecture stays identical — same async API, smarter execution.
