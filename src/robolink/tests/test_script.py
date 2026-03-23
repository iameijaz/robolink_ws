import pytest
from robolink.script import ScriptParser, ScriptError
from robolink.script import (
    MovJCmd, MovLCmd, GripperCmd, WaitCmd, SpeedCmd, RepeatCmd
)


def parse(source):
    return ScriptParser().parse(source)


# ── MOVJ ──────────────────────────────────────────────────────────────────

def test_movj_explicit():
    cmds = parse("MOVJ(0, -0.8, 0.5, 0, 1.5, 0)")
    assert isinstance(cmds[0], MovJCmd)
    assert cmds[0].state.j2 == -0.8
    assert cmds[0].state.j5 == 1.5


def test_movj_broadcast_zero():
    cmds = parse("MOVJ(0)")
    assert all(v == 0.0 for v in cmds[0].state.to_list())


def test_movj_broadcast_nonzero():
    cmds = parse("MOVJ(0.5)")
    assert all(v == 0.5 for v in cmds[0].state.to_list())


def test_movj_preset_home():
    cmds = parse("MOVJ(HOME)")
    assert all(v == 0.0 for v in cmds[0].state.to_list())


def test_movj_preset_case_insensitive():
    cmds = parse("MOVJ(home)")
    assert all(v == 0.0 for v in cmds[0].state.to_list())


def test_movj_preset_layup_ready():
    cmds = parse("MOVJ(LAYUP_READY)")
    assert cmds[0].state.j1 == -2.9367


def test_movj_wrong_count_raises():
    with pytest.raises(ScriptError, match="1 value"):
        parse("MOVJ(0, 0, 0)")


def test_movj_unknown_preset_raises():
    with pytest.raises(ScriptError, match="Unknown preset"):
        parse("MOVJ(UNKNOWN)")


# ── MOVL ──────────────────────────────────────────────────────────────────

def test_movl_parses():
    cmds = parse("MOVL(0.3, 0.1, 0.4, 180, 0, 0)")
    assert isinstance(cmds[0], MovLCmd)
    assert cmds[0].x == 0.3
    assert cmds[0].rx == 180.0


def test_movl_wrong_count_raises():
    with pytest.raises(ScriptError, match="exactly 6 values"):
        parse("MOVL(0.3, 0.1, 0.4)")


# ── GRIPPER ───────────────────────────────────────────────────────────────

def test_gripper_open():
    cmds = parse("GRIPPER(OPEN)")
    assert isinstance(cmds[0], GripperCmd)
    assert cmds[0].action == "OPEN"


def test_gripper_close():
    cmds = parse("GRIPPER(CLOSE)")
    assert cmds[0].action == "CLOSE"


def test_gripper_case_insensitive():
    cmds = parse("GRIPPER(open)")
    assert cmds[0].action == "OPEN"


def test_gripper_invalid_raises():
    with pytest.raises(ScriptError, match="OPEN or CLOSE"):
        parse("GRIPPER(HALF)")


# ── WAIT / SPEED ──────────────────────────────────────────────────────────

def test_wait():
    cmds = parse("WAIT(1.5)")
    assert isinstance(cmds[0], WaitCmd)
    assert cmds[0].seconds == 1.5


def test_wait_bad_value_raises():
    with pytest.raises(ScriptError, match="requires a number"):
        parse("WAIT(abc)")


def test_speed_valid():
    cmds = parse("SPEED(0.7)")
    assert isinstance(cmds[0], SpeedCmd)
    assert cmds[0].value == 0.7


def test_speed_out_of_range_raises():
    with pytest.raises(ScriptError, match="0.0"):
        parse("SPEED(1.5)")


# ── REPEAT ────────────────────────────────────────────────────────────────

def test_repeat_block():
    source = "REPEAT(2)\n    MOVJ(0, -0.5, 0, 0, 0, 0)\n    WAIT(0.5)\nEND"
    cmds = parse(source)
    assert isinstance(cmds[0], RepeatCmd)
    assert cmds[0].times == 2
    assert len(cmds[0].body) == 2


def test_empty_repeat_raises():
    with pytest.raises(ScriptError, match="empty"):
        parse("REPEAT(3)\nEND")


# ── Comments ──────────────────────────────────────────────────────────────

def test_comment_full_line_ignored():
    cmds = parse("# full line comment\nMOVJ(HOME)")
    assert len(cmds) == 1


def test_comment_inline_ignored():
    cmds = parse("MOVJ(HOME) # go home")
    assert isinstance(cmds[0], MovJCmd)


def test_unknown_command_raises():
    with pytest.raises(ScriptError, match="Unknown command"):
        parse("FLY(0, 0, 0)")


# ── Full scripts ──────────────────────────────────────────────────────────

def test_full_layup_script():
    source = """
# carbon layup test
MOVJ(HOME)
SPEED(0.7)
MOVJ(0, -0.8, 0.5, 0, 1.5, 0)
MOVJ(0, -0.6, 0.5, 0, 1.5, 0)
REPEAT(2)
    MOVJ(0, -0.4, 0.5, 0, 1.5, 0)
    WAIT(0.3)
END
MOVJ(HOME)
"""
    cmds = parse(source)
    assert len(cmds) == 6
    assert isinstance(cmds[0], MovJCmd)
    assert isinstance(cmds[1], SpeedCmd)
    assert isinstance(cmds[4], RepeatCmd)
    assert cmds[4].times == 2


def test_full_pick_and_place_script():
    source = """
MOVJ(HOME)
SPEED(0.8)
MOVJ(0.0, -0.5, 0.8, 0, 1.2, 0)
MOVJ(0.0, -0.5, 0.6, 0, 1.2, 0)
GRIPPER(CLOSE)
WAIT(0.4)
MOVJ(0.0, -0.5, 0.8, 0, 1.2, 0)
MOVJ(0.5, -0.3, 0.6, 0, 1.2, 0)
GRIPPER(OPEN)
WAIT(0.4)
MOVJ(HOME)
"""
    cmds = parse(source)
    assert len(cmds) == 11
    gripper_cmds = [c for c in cmds if isinstance(c, GripperCmd)]
    assert len(gripper_cmds) == 2
    assert gripper_cmds[0].action == "CLOSE"
    assert gripper_cmds[1].action == "OPEN"


# ── Bug regression tests (v0.1.1) ─────────────────────────────────────────

def test_bug1_unterminated_repeat_raises():
    """Bug #1 — REPEAT without END must raise, not silently return empty body."""
    with pytest.raises(ScriptError, match="never closed"):
        parse("REPEAT(3)\n    MOVJ(0)\n# no END")


def test_bug1_unterminated_repeat_with_preceding_commands():
    """Bug #1 — commands before unterminated REPEAT must not be silently dropped."""
    with pytest.raises(ScriptError, match="never closed"):
        parse("MOVJ(HOME)\nREPEAT(2)\n    WAIT(0.5)")


def test_bug2_orphan_end_raises():
    """Bug #2 — END without matching REPEAT must raise at top level."""
    with pytest.raises(ScriptError, match="without a matching REPEAT"):
        parse("WAIT(1.0)\nEND\nWAIT(2.0)")


def test_bug2_orphan_end_does_not_truncate():
    """Bug #2 — commands after orphan END must not be silently dropped."""
    with pytest.raises(ScriptError):
        parse("MOVJ(HOME)\nEND\nMOVJ(HOME)")


def test_bug3_negative_wait_raises():
    """Bug #3 — WAIT with negative value must raise at parse time."""
    with pytest.raises(ScriptError, match=">= 0"):
        parse("WAIT(-1.5)")


def test_bug3_zero_wait_is_valid():
    """Bug #3 — WAIT(0) is valid — yields control without pausing."""
    cmds = parse("WAIT(0)")
    assert cmds[0].seconds == 0.0


def test_bug4_preset_lowercase_key_works():
    """Bug #4 — PRESETS dict uses uppercase keys, MOVJ(home) still matches."""
    from robolink.script import ScriptParser, PRESETS
    # PRESETS keys are uppercase — MOVJ(home) is uppercased during parse
    cmds = ScriptParser().parse("MOVJ(home)")
    assert isinstance(cmds[0], MovJCmd)
    assert all(v == 0.0 for v in cmds[0].state.to_list())


def test_bug4_preset_mixed_case_yaml_keys():
    """Bug #4 — MOVJ(HOME) works regardless of script capitalisation."""
    from robolink.script import ScriptParser
    cmds = ScriptParser().parse("MOVJ(Home)")
    assert isinstance(cmds[0], MovJCmd)
    assert all(v == 0.0 for v in cmds[0].state.to_list())
