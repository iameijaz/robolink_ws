"""
V-Alpha Script interpreter — v0.1.2
"""
from __future__ import annotations
import asyncio
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

class ScriptError(Exception):
    pass

JOINT_NAMES = ("j1", "j2", "j3", "j4", "j5", "j6")
N_JOINTS    = len(JOINT_NAMES)

@dataclass
class JointState:
    j1: float = 0.0; j2: float = 0.0; j3: float = 0.0
    j4: float = 0.0; j5: float = 0.0; j6: float = 0.0
    def to_list(self): return [self.j1,self.j2,self.j3,self.j4,self.j5,self.j6]
    def with_overrides(self, overrides):
        vals = dict(zip(JOINT_NAMES, self.to_list()))
        vals.update(overrides)
        return JointState(**vals)
    def __repr__(self):
        return (f"JointState({self.j1:.3f},{self.j2:.3f},{self.j3:.3f},"
                f"{self.j4:.3f},{self.j5:.3f},{self.j6:.3f})")

PRESETS: Dict[str, JointState] = {
    "HOME":        JointState(0.0,0.0,0.0,0.0,0.0,0.0),
    "LAYUP_READY": JointState(-2.9367,-0.4508,-1.2933,0.1699,1.5341,0.1852),
}

@dataclass
class MovJCmd:
    state: Optional[JointState]; partial: bool = False
    overrides: Dict[str,float] = field(default_factory=dict)
@dataclass
class MovLCmd:
    x:float; y:float; z:float; rx:float; ry:float; rz:float
@dataclass
class GripperCmd:
    action: str
@dataclass
class IOCmd:
    name: str; state: str
@dataclass
class WaitCmd:
    seconds: float
@dataclass
class SpeedCmd:
    value: float
@dataclass
class RepeatCmd:
    times: int; body: List[Any]

Command = MovJCmd|MovLCmd|GripperCmd|IOCmd|WaitCmd|SpeedCmd|RepeatCmd

class ScriptParser:
    def __init__(self, n_joints: int = N_JOINTS):
        self.n_joints = n_joints
        self.valid_joints = {f"j{i+1}" for i in range(n_joints)}

    def parse(self, source: str) -> List[Command]:
        return self._parse_toplevel(self._clean(source))

    def _clean(self, source):
        result = []
        for i, line in enumerate(source.splitlines(), start=1):
            line = line.split('#')[0].strip()
            if line: result.append((i, line))
        return result

    def _parse_toplevel(self, lines):
        commands, pos = [], 0
        while pos < len(lines):
            lineno, line = lines[pos]
            token = line.split('(')[0].strip().upper()
            if token == 'END':
                raise ScriptError(
                    f"Line {lineno}: 'END' without a matching REPEAT.\n"
                    f"Remove this END or wrap commands in REPEAT(N) ... END.")
            pos = self._dispatch(commands, lines, pos, lineno, line, token)
        return commands

    def _parse_block(self, lines, pos, repeat_lineno):
        commands = []
        while pos < len(lines):
            lineno, line = lines[pos]
            token = line.split('(')[0].strip().upper()
            if token == 'END':
                return commands, pos + 1
            pos = self._dispatch(commands, lines, pos, lineno, line, token)
        raise ScriptError(
            f"Line {repeat_lineno}: REPEAT block is never closed.\n"
            f"Add END after the last command in the block.")

    def _dispatch(self, commands, lines, pos, lineno, line, token):
        if token == 'MOVJ':
            commands.append(self._parse_movj(lineno, line))
        elif token == 'MOVL':
            commands.append(self._parse_movl(lineno, line))
        elif token == 'GRIPPER':
            commands.append(self._parse_gripper(lineno, line))
        elif token == 'IO':
            commands.append(self._parse_io(lineno, line))
        elif token == 'WAIT':
            commands.append(self._parse_wait(lineno, line))
        elif token == 'SPEED':
            cmd = self._parse_single_float(lineno, line, SpeedCmd, 'SPEED')
            if not 0.0 <= cmd.value <= 1.0:
                raise ScriptError(f"Line {lineno}: SPEED must be 0.0–1.0, got {cmd.value}.")
            commands.append(cmd)
        elif token == 'REPEAT':
            times = self._parse_repeat_count(lineno, line)
            body, pos = self._parse_block(lines, pos + 1, lineno)
            if not body:
                raise ScriptError(f"Line {lineno}: REPEAT block is empty.")
            commands.append(RepeatCmd(times=times, body=body))
            return pos
        else:
            raise ScriptError(f"Line {lineno}: Unknown command '{token}'.")
        return pos + 1

    def _raw_args(self, lineno, line):
        m = re.match(r'^\w+\((.+)\)\s*$', line, re.IGNORECASE)
        if not m:
            raise ScriptError(f"Line {lineno}: Syntax error in '{line}'.")
        return m.group(1).strip()

    def _parse_movj(self, lineno, line):
        raw = self._raw_args(lineno, line)
        if raw.upper() in PRESETS:
            return MovJCmd(state=PRESETS[raw.upper()])
        if '=' in raw:
            return self._parse_movj_partial(lineno, raw)
        parts = [p.strip() for p in raw.split(',')]
        if len(parts) == 1:
            try: val = float(parts[0])
            except ValueError:
                raise ScriptError(f"Line {lineno}: Unknown preset or invalid value '{parts[0]}'.")
            return MovJCmd(state=JointState(val,val,val,val,val,val))
        if len(parts) != self.n_joints:
            raise ScriptError(
                f"Line {lineno}: MOVJ requires 1 value, {self.n_joints} values, "
                f"a preset, or keyword args. Got {len(parts)}.")
        try: vals = [float(p) for p in parts]
        except ValueError as e:
            raise ScriptError(f"Line {lineno}: Invalid joint value — {e}")
        return MovJCmd(state=JointState(*vals))

    def _parse_movj_partial(self, lineno, raw):
        overrides = {}
        lt = re.search(r'\blast_two\s*=\s*([^,=]+),\s*([^,=]+?)(?:\s*,|\s*$)', raw)
        if lt:
            try: v5, v6 = float(lt.group(1).strip()), float(lt.group(2).strip())
            except ValueError:
                raise ScriptError(f"Line {lineno}: last_two= requires two numbers.")
            overrides["j5"] = v5; overrides["j6"] = v6
            raw = re.sub(r'\blast_two\s*=\s*[^,=]+,\s*[^,=]+', '', raw).strip().strip(',').strip()
        for key, val_str in re.findall(r'(\w+)\s*=\s*([+-]?\d*\.?\d+)', raw):
            kl = key.lower()
            if kl not in self.valid_joints:
                raise ScriptError(
                    f"Line {lineno}: '{key}' is not a valid joint name. "
                    f"Valid: {', '.join(sorted(self.valid_joints))}.")
            overrides[kl] = float(val_str)
        if not overrides:
            raise ScriptError(f"Line {lineno}: MOVJ keyword form requires at least one joint.")
        return MovJCmd(state=None, partial=True, overrides=overrides)

    def _parse_movl(self, lineno, line):
        raw = self._raw_args(lineno, line)
        parts = [p.strip() for p in raw.split(',')]
        if len(parts) != 6:
            raise ScriptError(f"Line {lineno}: MOVL requires exactly 6 values. Got {len(parts)}.")
        try: x,y,z,rx,ry,rz = [float(p) for p in parts]
        except ValueError as e:
            raise ScriptError(f"Line {lineno}: Invalid value in MOVL — {e}")
        return MovLCmd(x=x,y=y,z=z,rx=rx,ry=ry,rz=rz)

    def _parse_gripper(self, lineno, line):
        raw = self._raw_args(lineno, line).strip().upper()
        if raw not in ("OPEN","CLOSE"):
            raise ScriptError(f"Line {lineno}: GRIPPER must be OPEN or CLOSE. Got '{raw}'.")
        return GripperCmd(action=raw)

    def _parse_io(self, lineno, line):
        raw = self._raw_args(lineno, line)
        parts = [p.strip().upper() for p in raw.split(',')]
        if len(parts) != 2:
            raise ScriptError(f"Line {lineno}: IO requires two arguments: name and state.")
        name, state = parts
        if not re.match(r'^[A-Z][A-Z0-9_]*$', name):
            raise ScriptError(f"Line {lineno}: IO name '{name}' is invalid.")
        if state not in ("ON","OFF"):
            raise ScriptError(f"Line {lineno}: IO state must be ON or OFF. Got '{state}'.")
        return IOCmd(name=name, state=state)

    def _parse_wait(self, lineno, line):
        raw = self._raw_args(lineno, line)
        try: seconds = float(raw)
        except ValueError:
            raise ScriptError(f"Line {lineno}: WAIT requires a number. Got '{raw}'.")
        if seconds < 0:
            raise ScriptError(f"Line {lineno}: WAIT duration must be >= 0, got {seconds}.")
        return WaitCmd(seconds=seconds)

    def _parse_single_float(self, lineno, line, cls, name):
        raw = self._raw_args(lineno, line)
        try: return cls(float(raw))
        except ValueError:
            raise ScriptError(f"Line {lineno}: {name} requires a number. Got '{raw}'.")

    def _parse_repeat_count(self, lineno, line):
        raw = self._raw_args(lineno, line)
        try: n = int(raw)
        except ValueError:
            raise ScriptError(f"Line {lineno}: REPEAT requires an integer. Got '{raw}'.")
        if n < 1:
            raise ScriptError(f"Line {lineno}: REPEAT count must be >= 1, got {n}.")
        return n

class ScriptRunner:
    def __init__(self, arm, speed: float = 1.0):
        self.arm = arm; self.speed = speed; self.parser = ScriptParser()
    async def run_file(self, path):
        source = Path(path).read_text(); commands = self.parser.parse(source)
        print(f"[V-Alpha] Running '{Path(path).name}' — {len(commands)} top-level commands")
        await self._execute(commands); print(f"[V-Alpha] complete.")
    async def run_source(self, source):
        await self._execute(self.parser.parse(source))
    async def _execute(self, commands):
        for cmd in commands: await self._run_one(cmd)
    async def _run_one(self, cmd):
        if isinstance(cmd, MovJCmd):
            target = self.arm.current_joints.with_overrides(cmd.overrides) if cmd.partial else cmd.state
            print(f"  MOVJ → {target}")
            await self.arm.move_to(target, delay=self.arm.step_delay/max(self.speed,0.01))
        elif isinstance(cmd, MovLCmd):
            print(f"  MOVL({cmd.x},{cmd.y},{cmd.z},{cmd.rx},{cmd.ry},{cmd.rz}) [deferred v0.2]")
        elif isinstance(cmd, GripperCmd):
            print(f"  GRIPPER({cmd.action})")
            await self.arm.set_gripper(cmd.action == "CLOSE")
        elif isinstance(cmd, IOCmd):
            print(f"  IO({cmd.name},{cmd.state})")
            await self.arm.set_io(cmd.name, cmd.state == "ON")
        elif isinstance(cmd, WaitCmd):
            print(f"  WAIT({cmd.seconds}s)"); await asyncio.sleep(cmd.seconds)
        elif isinstance(cmd, SpeedCmd):
            print(f"  SPEED({cmd.value})"); self.speed = cmd.value
        elif isinstance(cmd, RepeatCmd):
            for i in range(cmd.times): await self._execute(cmd.body)
