import asyncio
from concurrent.futures import ThreadPoolExecutor
from robolink.models import JointState, MoveResult, LayupSequence
from robolink.exceptions import (
    BackendNotAvailableError, ArmConnectionError, MoveTimeoutError
)

_executor = ThreadPoolExecutor(max_workers=2)

JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]


class ArmClient:
    """
    Async Python client for CR5 robot arm control.

    Publishes JointState messages via ROS2, visualized in RViz2.
    Publishes TCP trail markers to /tcp_trail for path visualization.
    Backend: robot_state_publisher — no MoveIt2 required for v0.1.0.

    Note: This is the ROS2 Jazzy port of the original ROS1 Noetic
    moveit_commander-based implementation used for real CR5 layup
    trials at NUST. MoveIt2 motion planning support is planned for v0.2.0.

    Example:
        async with ArmClient() as arm:
            await arm.go_home()
            await arm.execute_layup(sequence)
    """

    def __init__(
        self,
        node_name: str = "robolink_client",
        step_delay: float = 0.5,
        timeout: float = 30.0,
    ):
        self.node_name = node_name
        self.step_delay = step_delay
        self.timeout = timeout
        self._node = None
        self._publisher = None
        self._marker_pub = None
        self._gripper_pub = None
        self._connected = False
        self._marker_id = 0
        self._trail_positions = []

    # ── lifecycle ──────────────────────────────────────────────────────────

    async def __aenter__(self):
        await self.connect()
        return self

    async def __aexit__(self, *args):
        await self.disconnect()

    async def connect(self) -> None:
        try:
            await asyncio.get_event_loop().run_in_executor(
                _executor, self._init_ros
            )
            self._connected = True
        except ImportError:
            raise BackendNotAvailableError(
                "rclpy not found. Source your ROS2 workspace:\n"
                "  source /opt/ros/jazzy/setup.bash\n"
                "  source ~/robolink_ws/install/setup.bash"
            )

    def _init_ros(self) -> None:
        import rclpy
        from sensor_msgs.msg import JointState as RosJointState
        from visualization_msgs.msg import MarkerArray
        from std_msgs.msg import Bool

        rclpy.init()
        self._node = rclpy.create_node(self.node_name)

        self._publisher = self._node.create_publisher(
            RosJointState, '/joint_states', 10
        )
        self._marker_pub = self._node.create_publisher(
            MarkerArray, '/tcp_trail', 10
        )
        self._gripper_pub = self._node.create_publisher(
            Bool, '/gripper_command', 10
        )

    async def disconnect(self) -> None:
        if self._node:
            await asyncio.get_event_loop().run_in_executor(
                _executor, self._shutdown_ros
            )
        self._connected = False

    def _shutdown_ros(self) -> None:
        import rclpy
        self._node.destroy_node()
        rclpy.shutdown()

    def _require_connected(self) -> None:
        if not self._connected:
            raise ArmConnectionError(
                "Not connected. Use 'async with ArmClient()' "
                "or call await arm.connect() first."
            )

    # ── publishing ─────────────────────────────────────────────────────────

    def _publish(self, state: JointState) -> None:
        import rclpy
        from sensor_msgs.msg import JointState as RosJointState

        msg = RosJointState()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = state.to_list()
        self._publisher.publish(msg)
        rclpy.spin_once(self._node, timeout_sec=0.1)

    def _publish_trail_marker(self, state: JointState) -> None:
        """
        Add a sphere marker at an estimated TCP position.

        Position is approximated from joint angles — a proper FK
        implementation using the URDF DH parameters is planned for v0.2.0.
        For now, j2 and j3 dominate vertical position on the CR5,
        giving a visually meaningful trail even without full FK.
        """
        from visualization_msgs.msg import Marker, MarkerArray

        # Approximate TCP position from dominant joints
        # CR5 rough estimates: arm length ~0.4m per major link
        j2, j3, j5 = state.j2, state.j3, state.j5
        import math
        x = round(0.4 * math.cos(j2) * math.cos(state.j1), 3)
        y = round(0.4 * math.cos(j2) * math.sin(state.j1), 3)
        z = round(0.4 + 0.35 * math.sin(j2) + 0.3 * math.sin(j2 + j3), 3)

        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self._node.get_clock().now().to_msg()
        marker.ns = 'tcp_trail'
        marker.id = self._marker_id
        self._marker_id += 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.018
        marker.scale.y = 0.018
        marker.scale.z = 0.018
        marker.color.r = 0.0
        marker.color.g = 0.85
        marker.color.b = 1.0
        marker.color.a = 0.9

        self._trail_positions.append(marker)
        arr = MarkerArray()
        arr.markers = self._trail_positions
        self._marker_pub.publish(arr)

    # ── motion commands ────────────────────────────────────────────────────

    async def move_to(
        self, state: JointState, delay: float | None = None
    ) -> MoveResult:
        self._require_connected()
        try:
            async with asyncio.timeout(self.timeout):
                await asyncio.get_event_loop().run_in_executor(
                    _executor, self._publish, state
                )
                await asyncio.get_event_loop().run_in_executor(
                    _executor, self._publish_trail_marker, state
                )
                await asyncio.sleep(
                    delay if delay is not None else self.step_delay
                )
                return MoveResult(success=True, joint_state=state)
        except asyncio.TimeoutError:
            raise MoveTimeoutError(
                f"move_to({state}) timed out after {self.timeout}s."
            )

    async def go_home(self) -> MoveResult:
        """Move to all-zeros home position."""
        print("Moving to home...")
        return await self.move_to(JointState.home(), delay=1.0)

    async def go_to_layup_ready(self) -> MoveResult:
        """Move to pre-configured layup start position."""
        print("Moving to layup ready...")
        return await self.move_to(JointState.layup_ready(), delay=1.5)

    async def set_gripper(self, closed: bool) -> None:
        """
        Actuate gripper.

        Publishes to /gripper_command (std_msgs/Bool).
        True = close, False = open.

        In simulation: no physical effect — command is published and logged.
        In v0.2.0: connects to real gripper hardware driver.
        """
        self._require_connected()
        await asyncio.get_event_loop().run_in_executor(
            _executor, self._publish_gripper, closed
        )

    def _publish_gripper(self, closed: bool) -> None:
        import rclpy
        from std_msgs.msg import Bool
        msg = Bool()
        msg.data = closed
        self._gripper_pub.publish(msg)
        rclpy.spin_once(self._node, timeout_sec=0.05)
        state = "CLOSED" if closed else "OPEN"
        print(f"    [gripper] → {state}")

    # ── layup ──────────────────────────────────────────────────────────────

    async def execute_layup(
        self, sequence: LayupSequence
    ) -> list[MoveResult]:
        """Execute a full layup sequence."""
        self._require_connected()
        print(f"Executing layup '{sequence.name}' — {len(sequence)} waypoints")
        results = []
        for i, waypoint in enumerate(sequence.waypoints):
            print(f"  waypoint {i+1}/{len(sequence)}: {waypoint}")
            result = await self.move_to(waypoint)
            results.append(result)
        print(f"Layup '{sequence.name}' complete.")
        return results

    # ── streaming ──────────────────────────────────────────────────────────

    async def stream_joints(
        self, states: list[JointState], interval: float = 0.5
    ):
        """Async generator — yields each JointState as it is published."""
        self._require_connected()
        for state in states:
            await asyncio.get_event_loop().run_in_executor(
                _executor, self._publish, state
            )
            yield state
            await asyncio.sleep(interval)
