from dataclasses import dataclass, field
from typing import Optional
import numpy as np


@dataclass
class JointState:
    """6-DOF joint configuration in radians."""
    j1: float = 0.0
    j2: float = 0.0
    j3: float = 0.0
    j4: float = 0.0
    j5: float = 0.0
    j6: float = 0.0

    def to_list(self) -> list[float]:
        return [self.j1, self.j2, self.j3, self.j4, self.j5, self.j6]

    @classmethod
    def home(cls) -> "JointState":
        """All-zeros home position."""
        return cls(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    @classmethod
    def layup_ready(cls) -> "JointState":
        """Pre-configured layup start position from real CR5 runs at NUST."""
        return cls(
            j1=-2.9367,
            j2=-0.4508,
            j3=-1.2933,
            j4=0.1699,
            j5=1.5341,
            j6=0.1852,
        )

    def __repr__(self):
        vals = ", ".join(f"{v:.3f}" for v in self.to_list())
        return f"JointState([{vals}])"


@dataclass
class MoveResult:
    success: bool
    message: str = ""
    joint_state: Optional[JointState] = None


@dataclass
class LayupSequence:
    """
    Named sequence of joint configurations for fiber composite layup.

    Example:
        seq = LayupSequence("carbon_ply_1")
        seq.add_waypoint(JointState(j1=0.1, j2=-0.5, j3=0.3))
        seq.add_waypoint(JointState(j1=0.2, j2=-0.5, j3=0.3))

    Or use sweep() to generate a raster pattern automatically:
        seq = LayupSequence.sweep(
            axis="j2",
            start=-0.5,
            end=0.5,
            steps=8,
            base=JointState(j1=0.0, j3=0.3, j4=0.0, j5=1.5, j6=0.0)
        )
    """
    name: str
    waypoints: list[JointState] = field(default_factory=list)

    def add_waypoint(self, state: JointState) -> "LayupSequence":
        """Fluent API — chain calls to build the sequence."""
        self.waypoints.append(state)
        return self

    @classmethod
    def sweep(
        cls,
        name: str,
        axis: str,
        start: float,
        end: float,
        steps: int,
        base: Optional[JointState] = None,
    ) -> "LayupSequence":
        """
        Generate a sweep (raster) pattern along one joint axis.
        Mirrors the manual waypoint pattern from the original ROS1 layup script.

        Args:
            axis:  which joint to sweep — 'j1' through 'j6'
            start: starting angle in radians
            end:   ending angle in radians
            steps: number of waypoints
            base:  base JointState for all other joints (defaults to home)
        """
        seq = cls(name=name)
        base = base or JointState.home()
        for val in np.linspace(start, end, steps):
            state = JointState(
                j1=base.j1, j2=base.j2, j3=base.j3,
                j4=base.j4, j5=base.j5, j6=base.j6,
            )
            setattr(state, axis, round(float(val), 4))
            seq.waypoints.append(state)
        return seq

    def __len__(self) -> int:
        return len(self.waypoints)

    def __repr__(self) -> str:
        return f"LayupSequence('{self.name}', {len(self)} waypoints)"
