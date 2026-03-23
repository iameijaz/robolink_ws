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

    Publishes JointState messages via ROS2, visualized in RViz.
    Backend: robot_state_publisher — no MoveIt required.

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
        self.step_delay = step_delay  # seconds between waypoints
        self.timeout = timeout
        self._node = None
        self._publisher = None
        self._connected = False

    # ── lifecycle ──────────────────────────────────────────────────────────

    async def __aenter__(self):
        await self.connect()
        return self

    async def __aexit__(self, *args):
        await self.disconnect()

    async def connect(self) -> None:
        """Initialize ROS2 node and joint state publisher."""
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
        from sensor_msgs.msg import JointState as RosJointState  # noqa: F401
        rclpy.init()
        self._node = rclpy.create_node(self.node_name)
        self._publisher = self._node.create_publisher(
            __import__('sensor_msgs.msg', fromlist=['JointState']).JointState,
            '/joint_states',
            10,
        )
        # Add to _init_ros:
        from visualization_msgs.msg import MarkerArray
        self._marker_pub = self._node.create_publisher(
            MarkerArray, '/tcp_trail', 10
        )
        self._markers = MarkerArray()
        self._marker_id = 0

    async def disconnect(self) -> None:
        """Shutdown ROS2 node cleanly."""
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
        """Publish a single JointState message synchronously."""
        import rclpy
        from sensor_msgs.msg import JointState as RosJointState
        from builtin_interfaces.msg import Duration

        msg = RosJointState()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = state.to_list()
        self._publisher.publish(msg)

        # Spin once so the message actually goes out
        rclpy.spin_once(self._node, timeout_sec=0.1)
    def _publish_trail_marker(self, position: tuple) -> None:
        """Add a sphere marker at the given position."""
        from visualization_msgs.msg import Marker
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self._node.get_clock().now().to_msg()
        marker.ns = 'tcp_trail'
        marker.id = self._marker_id
        self._marker_id += 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.015
        marker.scale.y = 0.015
        marker.scale.z = 0.015
        marker.color.r = 0.0
        marker.color.g = 0.8
        marker.color.b = 1.0
        marker.color.a = 1.0
        self._markers.markers.append(marker)
        self._marker_pub.publish(self._markers)
    # ── motion commands ────────────────────────────────────────────────────

    async def move_to(
        self, state: JointState, delay: float | None = None
    ) -> MoveResult:
        """
        Move arm to joint configuration.
        Publishes the target state and waits step_delay seconds.

        Args:
            state: target JointState in radians
            delay: override default step_delay for this move
        """
        self._require_connected()
        try:
            async with asyncio.timeout(self.timeout):
                await asyncio.get_event_loop().run_in_executor(
                    _executor, self._publish, state
                )
                await asyncio.sleep(delay if delay is not None else self.step_delay)
                return MoveResult(success=True, joint_state=state)
        except asyncio.TimeoutError:
            raise MoveTimeoutError(
                f"move_to({state}) timed out after {self.timeout}s."
            )

    async def go_home(self) -> MoveResult:
        """Move to all-zeros home position."""
        print("Moving to home position...")
        return await self.move_to(JointState.home(), delay=1.0)

    async def go_to_layup_ready(self) -> MoveResult:
        """
        Move to pre-configured layup start position.
        Joint values from real CR5 layup trials at NUST.
        """
        print("Moving to layup ready position...")
        return await self.move_to(JointState.layup_ready(), delay=1.5)

    # ── layup ──────────────────────────────────────────────────────────────

    async def execute_layup(
        self, sequence: LayupSequence
    ) -> list[MoveResult]:
        """
        Execute a full layup sequence.

        Moves through all waypoints in order, publishing each
        JointState with step_delay between movements.

        This is the primary interface for fiber composite layup operations.
        Mirrors the cartesian path execution from the original ROS1 script.

        Args:
            sequence: LayupSequence containing ordered waypoints

        Returns:
            List of MoveResult — one per waypoint
        """
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
        """
        Async generator — yields each JointState as it is published.
        Useful for monitoring execution progress.

        Example:
            async for state in arm.stream_joints(sequence.waypoints):
                print(f"At: {state}")
        """
        self._require_connected()
        for state in states:
            await asyncio.get_event_loop().run_in_executor(
                _executor, self._publish, state
            )
            yield state
            await asyncio.sleep(interval)
