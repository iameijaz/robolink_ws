"""
V-Alpha Script interpreter — v0.1.1

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
    GRIPPER(OPEN)                     — open gripper
    GRIPPER(CLOSE)                    — close gripper
    WAIT(1.5)                         — pause N seconds (must be >= 0)
    SPEED(0.7)                        — set speed multiplier 0.0–1.0
    REPEAT(3) ... END                 — repeat a block N times
    # comment                         — ignored

Changelog v0.1.1 vs v0.1.0:
    Fix #1 — Unterminated REPEAT (missing END) now raises ScriptError instead
              of silently returning an empty body command list. Previously the
              parser returned commands=[RepeatCmd(times=N, body=[])] which
              the empty-body guard happened to catch in some cases but not
              all — specifically when REPEAT contained commands but the END
              was simply absent.
    Fix #2 — Orphan END (END without a matching REPEAT) now raises ScriptError.
              Previously _parse_block returned on the first END it found even
              at the top level, silently discarding any commands that followed.
    Fix #3 — WAIT with a negative value now raises ScriptError at parse time.
              Previously WAIT(-1) parsed fine but caused asyncio.sleep(-1) to
              raise a ValueError at runtime — an unpleasant surprise mid-run.
    Fix #4 — Preset keys are normalised to uppercase when loaded from
              presets.yaml, so robot operators don't need to know that keys
              must be uppercase in the YAML file.
    Polish — richer error messages throughout, with fix hints for operators.
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
class GripperCmd:
    action: str  # "OPEN" or "CLOSE"

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


Command = MovJCmd | MovLCmd | GripperCmd | WaitCmd | SpeedCmd | RepeatCmd


# ── Parser ─────────────────────────────────────────────────────────────────

class ScriptParser:
    """
    Parses V-Alpha Script source into a list of Command objects.

    Args:
        presets: optional dict of named JointState presets.
                 Defaults to loading presets.yaml from the package directory.
                 Pass your own to support different robot models:
                     ScriptParser(presets={"HOME": JointState(...)})

                 Keys are normalised to uppercase, so both "home" and "HOME"
                 work as dict keys and match MOVJ(HOME) in scripts.
    """

    def __init__(self, presets: dict[str, JointState] | None = None):
        if presets is not None:
            # FIX #4: normalise all keys to uppercase so MOVJ(HOME) always works
            # regardless of how the caller spells the key.
            self.presets = {k.upper(): v for k, v in presets.items()}
        else:
            import yaml
            presets_file = Path(__file__).parent / "presets.yaml"
            raw = yaml.safe_load(presets_file.read_text())
            self.presets = {
                name.upper(): JointState(**vals)
                for name, vals in raw.items()
            }

    def parse(self, source: str) -> list[Command]:
        lines = self._clean(source)
        # FIX #2: top-level parse must reject orphan ENDs.
        # We use a dedicated top-level loop rather than reusing _parse_block,
        # so that END at depth-0 is always an error.
        commands = self._parse_toplevel(lines)
        return commands

    def _clean(self, source: str) -> list[tuple[int, str]]:
        result = []
        for i, line in enumerate(source.splitlines(), start=1):
            line = line.split('#')[0].strip()
            if line:
                result.append((i, line))
        return result

    def _parse_toplevel(self, lines: list[tuple[int, str]]) -> list[Command]:
        """
        Parse the top-level command list.  Orphan END tokens are an error here.
        """
        commands: list[Command] = []
        pos = 0
        while pos < len(lines):
            lineno, line = lines[pos]
            token = line.split('(')[0].strip().upper()

            if token == 'END':
                raise ScriptError(
                    f"Line {lineno}: 'END' without a matching REPEAT.\n"
                    f"Remove this END, or wrap the preceding commands in REPEAT(N) ... END."
                )

            pos = self._dispatch(commands, lines, pos, lineno, line, token)

        return commands

    def _parse_block(
        self, lines: list[tuple[int, str]], pos: int, repeat_lineno: int
    ) -> tuple[list[Command], int]:
        """
        Parse commands inside a REPEAT block until END is found.

        FIX #1: If the source ends without an END, raise a clear error
        instead of returning silently (which previously left the RepeatCmd
        with an empty body, triggering the empty-body guard only by accident).
        """
        commands: list[Command] = []
        while pos < len(lines):
            lineno, line = lines[pos]
            token = line.split('(')[0].strip().upper()

            if token == 'END':
                return commands, pos + 1

            pos = self._dispatch(commands, lines, pos, lineno, line, token)

        # Fell off the end of source without finding END
        raise ScriptError(
            f"Line {repeat_lineno}: REPEAT block is never closed — "
            f"add END after the last command in the block.\n"
            f"  REPEAT({lines[repeat_lineno - 1][1] if repeat_lineno <= len(lines) else '?'})\n"
            f"      ...\n"
            f"  END    ← missing"
        )

    def _dispatch(
        self,
        commands: list[Command],
        lines: list[tuple[int, str]],
        pos: int,
        lineno: int,
        line: str,
        token: str,
    ) -> int:
        """Route one line to its parser and append the result to *commands*."""
        if token == 'MOVJ':
            commands.append(self._parse_movj(lineno, line))
            return pos + 1

        elif token == 'MOVL':
            commands.append(self._parse_movl(lineno, line))
            return pos + 1

        elif token == 'GRIPPER':
            commands.append(self._parse_gripper(lineno, line))
            return pos + 1

        elif token == 'WAIT':
            commands.append(self._parse_wait(lineno, line))
            return pos + 1

        elif token == 'SPEED':
            cmd = self._parse_single_float(lineno, line, SpeedCmd, 'SPEED')
            if not 0.0 <= cmd.value <= 1.0:
                raise ScriptError(
                    f"Line {lineno}: SPEED must be between 0.0 and 1.0, "
                    f"got {cmd.value}.\n"
                    f"  SPEED(0.5)   — half speed\n"
                    f"  SPEED(1.0)   — full speed"
                )
            commands.append(cmd)
            return pos + 1

        elif token == 'REPEAT':
            times = self._parse_repeat_header(lineno, line)
            body, new_pos = self._parse_block(lines, pos + 1, lineno)
            if not body:
                raise ScriptError(
                    f"Line {lineno}: REPEAT block is empty — "
                    f"add at least one command before END."
                )
            commands.append(RepeatCmd(times=times, body=body))
            return new_pos

        else:
            raise ScriptError(
                f"Line {lineno}: Unknown command '{token}'.\n"
                f"Valid commands: MOVJ, MOVL, GRIPPER, WAIT, SPEED, REPEAT\n"
                f"Did you mean one of those? Check spelling and capitalisation."
            )

    # ── argument extractor ────────────────────────────────────────────────

    def _args(self, lineno: int, line: str) -> str:
        """
        Extract the argument string from COMMAND(args).

        The regex requires at least one non-whitespace character inside the
        parens, so COMMAND() is always a syntax error with a helpful message.
        """
        m = re.match(r'^\w+\((.+)\)\s*$', line, re.IGNORECASE)
        if not m:
            # Give a specific hint for the common empty-parens mistake
            if re.match(r'^\w+\(\s*\)\s*$', line, re.IGNORECASE):
                cmd = line.split('(')[0].strip().upper()
                raise ScriptError(
                    f"Line {lineno}: {cmd}() has no arguments.\n"
                    f"Provide at least one argument, e.g.:\n"
                    + self._empty_args_hint(cmd)
                )
            raise ScriptError(
                f"Line {lineno}: Syntax error in '{line}'.\n"
                f"Expected: COMMAND(args)  — e.g. MOVJ(0) or GRIPPER(OPEN)"
            )
        return m.group(1).strip()

    @staticmethod
    def _empty_args_hint(cmd: str) -> str:
        hints = {
            'MOVJ':    '  MOVJ(0)  or  MOVJ(HOME)',
            'MOVL':    '  MOVL(0.3, 0.1, 0.4, 180, 0, 0)',
            'GRIPPER': '  GRIPPER(OPEN)  or  GRIPPER(CLOSE)',
            'WAIT':    '  WAIT(1.5)',
            'SPEED':   '  SPEED(0.8)',
            'REPEAT':  '  REPEAT(3)',
        }
        return hints.get(cmd, f'  {cmd}(<value>)')

    # ── per-command parsers ───────────────────────────────────────────────

    def _parse_movj(self, lineno: int, line: str) -> MovJCmd:
        raw = self._args(lineno, line)

        if raw.upper() in self.presets:
            return MovJCmd(state=self.presets[raw.upper()])

        parts = [p.strip() for p in raw.split(',')]

        if len(parts) == 1:
            try:
                val = float(parts[0])
            except ValueError:
                preset_list = ', '.join(sorted(self.presets.keys()))
                raise ScriptError(
                    f"Line {lineno}: Unknown preset or invalid value '{parts[0]}'.\n"
                    f"Available presets: {preset_list or '(none defined)'}\n"
                    f"Or use a number: MOVJ(0)"
                )
            return MovJCmd(state=JointState(val, val, val, val, val, val))

        if len(parts) != 6:
            raise ScriptError(
                f"Line {lineno}: MOVJ requires 1 value (broadcast), "
                f"6 values (explicit joints), or a named preset. "
                f"Got {len(parts)} value(s).\n"
                f"  MOVJ(0)                         — all joints to 0\n"
                f"  MOVJ(0, -0.8, 0.5, 0, 1.5, 0)  — explicit 6-tuple (radians)\n"
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

    def _parse_gripper(self, lineno: int, line: str) -> GripperCmd:
        raw = self._args(lineno, line).strip().upper()
        if raw not in ("OPEN", "CLOSE"):
            raise ScriptError(
                f"Line {lineno}: GRIPPER argument must be OPEN or CLOSE. "
                f"Got '{raw}'.\n"
                f"  GRIPPER(OPEN)\n"
                f"  GRIPPER(CLOSE)"
            )
        return GripperCmd(action=raw)

    def _parse_wait(self, lineno: int, line: str) -> WaitCmd:
        """
        FIX #3: Reject negative wait durations at parse time.
        asyncio.sleep(negative) raises ValueError at runtime — catch it here
        so the operator sees a clear error with the line number.
        """
        raw = self._args(lineno, line)
        try:
            seconds = float(raw)
        except ValueError:
            raise ScriptError(
                f"Line {lineno}: WAIT requires a number. "
                f"Got '{raw}'.  Example: WAIT(1.5)"
            )
        if seconds < 0:
            raise ScriptError(
                f"Line {lineno}: WAIT duration must be >= 0, got {seconds}.\n"
                f"Use WAIT(0) to yield control without pausing."
            )
        return WaitCmd(seconds=seconds)

    def _parse_single_float(self, lineno: int, line: str, cls, name: str):
        raw = self._args(lineno, line)
        try:
            return cls(float(raw))
        except ValueError:
            raise ScriptError(
                f"Line {lineno}: {name} requires a number. "
                f"Got '{raw}'.  Example: {name}(1.5)"
            )

    def _parse_repeat_header(self, lineno: int, line: str) -> int:
        raw = self._args(lineno, line)
        try:
            n = int(raw)
        except ValueError:
            raise ScriptError(
                f"Line {lineno}: REPEAT requires a whole number (integer). "
                f"Got '{raw}'.  Example: REPEAT(3)"
            )
        if n < 1:
            raise ScriptError(
                f"Line {lineno}: REPEAT count must be >= 1, got {n}.\n"
                f"Use REPEAT(1) to execute a block exactly once."
            )
        return n

    # kept for backwards compatibility — internal callers now use _parse_repeat_header
    _parse_repeat = _parse_repeat_header


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

        async def set_gripper(closed: bool) -> None
            Actuate gripper. True = close, False = open.

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
        """Parse and execute a V-Alpha Script string."""
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
                f"to joint angles — not implemented in v0.1.x.\n"
                f"See V_ALPHA_SCRIPT.md for the backend contract. "
                f"Planned for v0.2.0 with MoveIt2."
            )

        elif isinstance(cmd, GripperCmd):
            print(f"  GRIPPER({cmd.action})")
            await self.arm.set_gripper(cmd.action == "CLOSE")

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
