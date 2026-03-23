"""
V-Alpha Script interpreter — v0.1.0

A domain-specific language for industrial robot arm control.
Designed to be readable by non-roboticists while mapping cleanly
onto real motion primitives.

Full V-Alpha pipeline (in progress):
    Intent → Admission → Kinematic Proof → Execution → Supervision

v0.1.0 implements: Intent + Execution
v0.2.0 planned:    Admission + Kinematic Proof (IK backend, singularity detection)

Syntax:
    MOVJ(0, -0.8, 0.5, 0, 1.5, 0)   — joint move, 6 values in radians
    MOVJ(0)                           — broadcast: all joints to 0
    MOVJ(HOME)                        — named preset
    MOVJ(LAYUP_READY)                 — named preset
    MOVL(x, y, z, rx, ry, rz)        — Cartesian linear move (v0.2.0)
    WAIT(1.5)                         — pause N seconds
    SPEED(0.7)                        — set speed multiplier 0.0–1.0
    REPEAT(3) ... END                 — repeat a block N times
    # comment                         — ignored
"""
from __future__ import annotations
import asyncio
import re
from pathlib import Path
from dataclasses import dataclass
from typing import Any

from robolink.models import JointState
from robolink.exceptions import RobolinkError


class ScriptError(RobolinkError):
    """Raised on V-Alpha Script syntax or runtime errors."""


# ── AST nodes ──────────────────────────────────────────────────────────────

@dataclass
class MovJCmd:
    state: JointState

@dataclass
class MovLCmd:
    x: float; y: float; z: float
    rx: float; ry: float; rz: float

@dataclass
class WaitCmd:
    seconds: float

@dataclass
class SpeedCmd:
    value: float

@dataclass
class RepeatCmd:
    times: int
    body: list[Any]


Command = MovJCmd | MovLCmd | WaitCmd | SpeedCmd | RepeatCmd


# ── Parser ─────────────────────────────────────────────────────────────────

class ScriptParser:
    """
    Parses V-Alpha Script source into a list of Command objects.

    Args:
        presets: optional dict of named JointState presets.
                 Defaults to loading presets.yaml from the package directory.
                 Pass your own to support different robot models:
                     ScriptParser(presets={"HOME": JointState(...)})
    """

    def __init__(self, presets: dict[str, JointState] | None = None):
        if presets is not None:
            self.presets = presets
        else:
            import yaml
            presets_file = Path(__file__).parent / "presets.yaml"
            raw = yaml.safe_load(presets_file.read_text())
            self.presets = {
                name: JointState(**vals)
                for name, vals in raw.items()
            }

    def parse(self, source: str) -> list[Command]:
        lines = self._clean(source)
        commands, _ = self._parse_block(lines, 0)
        return commands

    def _clean(self, source: str) -> list[tuple[int, str]]:
        result = []
        for i, line in enumerate(source.splitlines(), start=1):
            line = line.split('#')[0].strip()
            if line:
                result.append((i, line))
        return result

    def _parse_block(
        self, lines: list[tuple[int, str]], pos: int
    ) -> tuple[list[Command], int]:
        commands = []
        while pos < len(lines):
            lineno, line = lines[pos]
            token = line.split('(')[0].strip().upper()

            if token == 'END':
                return commands, pos + 1

            elif token == 'MOVJ':
                commands.append(self._parse_movj(lineno, line))
                pos += 1

            elif token == 'MOVL':
                commands.append(self._parse_movl(lineno, line))
                pos += 1

            elif token == 'WAIT':
                commands.append(self._parse_single_float(
                    lineno, line, WaitCmd, 'WAIT'
                ))
                pos += 1

            elif token == 'SPEED':
                cmd = self._parse_single_float(
                    lineno, line, SpeedCmd, 'SPEED'
                )
                if not 0.0 <= cmd.value <= 1.0:
                    raise ScriptError(
                        f"Line {lineno}: SPEED must be between 0.0 and 1.0, "
                        f"got {cmd.value}"
                    )
                commands.append(cmd)
                pos += 1

            elif token == 'REPEAT':
                times = self._parse_repeat(lineno, line)
                body, pos = self._parse_block(lines, pos + 1)
                if not body:
                    raise ScriptError(
                        f"Line {lineno}: REPEAT block is empty — "
                        f"add commands before END"
                    )
                commands.append(RepeatCmd(times=times, body=body))

            else:
                raise ScriptError(
                    f"Line {lineno}: Unknown command '{token}'. "
                    f"Valid commands: MOVJ, MOVL, WAIT, SPEED, REPEAT"
                )

        return commands, pos

    def _args(self, lineno: int, line: str) -> str:
        m = re.match(r'^\w+\((.+)\)\s*$', line, re.IGNORECASE)
        if not m:
            raise ScriptError(
                f"Line {lineno}: Syntax error in '{line}'. "
                f"Expected: COMMAND(args)"
            )
        return m.group(1).strip()

    def _parse_movj(self, lineno: int, line: str) -> MovJCmd:
        raw = self._args(lineno, line)

        # Named preset — case insensitive
        if raw.upper() in self.presets:
            return MovJCmd(state=self.presets[raw.upper()])

        parts = [p.strip() for p in raw.split(',')]

        # Scalar broadcast — MOVJ(0) means all joints to 0
        if len(parts) == 1:
            try:
                val = float(parts[0])
            except ValueError:
                raise ScriptError(
                    f"Line {lineno}: Unknown preset or invalid value '{parts[0]}'.\n"
                    f"Named presets: {', '.join(self.presets.keys())}\n"
                    f"Or use a number: MOVJ(0)"
                )
            return MovJCmd(state=JointState(val, val, val, val, val, val))

        # Full 6-tuple
        if len(parts) != 6:
            raise ScriptError(
                f"Line {lineno}: MOVJ requires 1 value (broadcast), "
                f"6 values (explicit), or a named preset. "
                f"Got {len(parts)} values.\n"
                f"  MOVJ(0)                         — all joints to 0\n"
                f"  MOVJ(0, -0.8, 0.5, 0, 1.5, 0)  — explicit 6-tuple\n"
                f"  MOVJ(HOME)                       — named preset"
            )

        try:
            vals = [float(p) for p in parts]
        except ValueError as e:
            raise ScriptError(f"Line {lineno}: Invalid joint value — {e}")

        return MovJCmd(state=JointState(*vals))

    def _parse_movl(self, lineno: int, line: str) -> MovLCmd:
        raw = self._args(lineno, line)
        parts = [p.strip() for p in raw.split(',')]
        if len(parts) != 6:
            raise ScriptError(
                f"Line {lineno}: MOVL requires exactly 6 values: "
                f"x, y, z, rx, ry, rz. Got {len(parts)}.\n"
                f"  Example: MOVL(0.3, 0.1, 0.4, 180, 0, 0)"
            )
        try:
            x, y, z, rx, ry, rz = [float(p) for p in parts]
        except ValueError as e:
            raise ScriptError(f"Line {lineno}: Invalid value in MOVL — {e}")
        return MovLCmd(x=x, y=y, z=z, rx=rx, ry=ry, rz=rz)

    def _parse_single_float(self, lineno, line, cls, name):
        raw = self._args(lineno, line)
        try:
            return cls(float(raw))
        except ValueError:
            raise ScriptError(
                f"Line {lineno}: {name} requires a number. "
                f"Got '{raw}'. Example: {name}(1.5)"
            )

    def _parse_repeat(self, lineno: int, line: str) -> int:
        raw = self._args(lineno, line)
        try:
            n = int(raw)
        except ValueError:
            raise ScriptError(
                f"Line {lineno}: REPEAT requires an integer. "
                f"Got '{raw}'. Example: REPEAT(3)"
            )
        if n < 1:
            raise ScriptError(
                f"Line {lineno}: REPEAT count must be >= 1, got {n}"
            )
        return n


# ── Executor ───────────────────────────────────────────────────────────────

class ScriptRunner:
    """
    Executes a parsed V-Alpha Script against a robot client.

    Implements Intent + Execution layers of the V-Alpha pipeline.
    Admission and Kinematic Proof are planned for v0.2.0.

    Required interface — the arm argument must implement:

        async def move_to(state: JointState, delay: float) -> MoveResult
            Move to joint configuration and wait.

        async def go_home() -> MoveResult
            Move to all-zeros home position.

        async def go_to_layup_ready() -> MoveResult
            Move to layup start position.

        float step_delay
            Default seconds between moves.

    robolink.client.ArmClient satisfies this interface.
    Any client implementing the same methods will work.
    """

    def __init__(self, arm, speed: float = 1.0):
        self.arm = arm
        self.speed = speed
        self.parser = ScriptParser()

    async def run_file(self, path: str | Path) -> None:
        """Parse and execute a .vas script file."""
        source = Path(path).read_text()
        commands = self.parser.parse(source)
        name = Path(path).name
        print(f"[V-Alpha] Running '{name}' — {len(commands)} top-level commands")
        await self._execute(commands)
        print(f"[V-Alpha] '{name}' complete.")

    async def run_source(self, source: str) -> None:
        """Parse and execute a .vas script string."""
        commands = self.parser.parse(source)
        await self._execute(commands)

    async def _execute(self, commands: list[Command]) -> None:
        for cmd in commands:
            await self._run_one(cmd)

    async def _run_one(self, cmd: Command) -> None:

        if isinstance(cmd, MovJCmd):
            print(f"  MOVJ → {cmd.state}")
            delay = self.arm.step_delay / max(self.speed, 0.01)
            await self.arm.move_to(cmd.state, delay=delay)

        elif isinstance(cmd, MovLCmd):
            raise NotImplementedError(
                f"MOVL({cmd.x}, {cmd.y}, {cmd.z}, "
                f"{cmd.rx}, {cmd.ry}, {cmd.rz})\n"
                f"MOVL requires an IK backend to resolve Cartesian pose "
                f"to joint angles — not implemented in v0.1.0.\n"
                f"See V_ALPHA_SCRIPT.md for the backend contract. "
                f"Planned for v0.2.0 with MoveIt2."
            )

        elif isinstance(cmd, WaitCmd):
            print(f"  WAIT({cmd.seconds}s)")
            await asyncio.sleep(cmd.seconds)

        elif isinstance(cmd, SpeedCmd):
            print(f"  SPEED({cmd.value})")
            self.speed = cmd.value

        elif isinstance(cmd, RepeatCmd):
            print(f"  REPEAT({cmd.times})")
            for i in range(cmd.times):
                print(f"    — iteration {i+1}/{cmd.times}")
                await self._execute(cmd.body)