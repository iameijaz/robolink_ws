"""
CR5 fiber composite layup demo — robolink SDK.

Replicates the raster sweep pattern from real NUST layup trials,
originally written for ROS1 Noetic moveit_commander.
Now running on ROS2 Jazzy via direct joint state publishing.

Usage:
    # Terminal 1 — start the robot visualizer
    source ~/robolink_ws/install/setup.bash
    ros2 launch dobot_description display.launch.py

    # Terminal 2 — run this demo
    source ~/robolink_ws/install/setup.bash
    python3 examples/layup_demo.py
"""
import asyncio
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from robolink import ArmClient, JointState, LayupSequence


async def main():
    async with ArmClient(step_delay=0.8) as arm:

        # Move to safe start
        await arm.go_home()

        # ── Layup sequence ─────────────────────────────────────────────
        # Sweep joint2 (shoulder) back and forth — two passes at
        # different joint1 angles. Mirrors the Y-axis raster from
        # the original cartesian layup script.

        sequence = (
            LayupSequence("carbon_ply_1")
            # Pass 1 — sweep j2 forward
            .add_waypoint(JointState(j1=0.0,  j2=-0.8, j3=0.5, j5=1.5))
            .add_waypoint(JointState(j1=0.0,  j2=-0.6, j3=0.5, j5=1.5))
            .add_waypoint(JointState(j1=0.0,  j2=-0.4, j3=0.5, j5=1.5))
            .add_waypoint(JointState(j1=0.0,  j2=-0.2, j3=0.5, j5=1.5))
            # Pass 2 — sweep j2 back (reverse direction)
            .add_waypoint(JointState(j1=0.2,  j2=-0.2, j3=0.5, j5=1.5))
            .add_waypoint(JointState(j1=0.2,  j2=-0.4, j3=0.5, j5=1.5))
            .add_waypoint(JointState(j1=0.2,  j2=-0.6, j3=0.5, j5=1.5))
            .add_waypoint(JointState(j1=0.2,  j2=-0.8, j3=0.5, j5=1.5))
        )

        print(f"\nSequence: {sequence}")
        results = await arm.execute_layup(sequence)

        success_count = sum(1 for r in results if r.success)
        print(f"\nCompleted {success_count}/{len(results)} waypoints successfully.")

        # Return home
        await arm.go_home()


if __name__ == "__main__":
    asyncio.run(main())
