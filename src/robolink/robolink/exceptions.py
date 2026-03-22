class RobolinkError(Exception):
    """Base exception. Catch this to handle any robolink failure."""

class BackendNotAvailableError(RobolinkError):
    """ROS2/rclpy not installed or not sourced."""

class ArmConnectionError(RobolinkError):
    """Client not connected. Use async with ArmClient() or call connect()."""

class MoveTimeoutError(RobolinkError):
    """Motion command exceeded timeout."""

class ExecutionError(RobolinkError):
    """Joint state publication failed."""
