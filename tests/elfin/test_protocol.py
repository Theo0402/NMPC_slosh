"""The command table against elfin_commands.md, and its formatting rules (codebase plan R-B)."""
import math
import pathlib
import re

import pytest

from tools.elfin_py import protocol as P

ROOT = pathlib.Path(__file__).resolve().parents[2]
COMMANDS_MD = (ROOT / "docs/plans/elfin_commands.md").read_text(encoding="utf-8")

# Force-sensor commands are left out of the table (owner, week 1).
FORCE_OMITTED = {"ReadFTCabData", "SetForceZero", "SetFTPosition", "SetForceControlState",
                 "ReadFTControlState", "HRSetForceDataLimit"}
# Errors in the protocol document, as elfin_commands.md lists them ("Errors in the protocol document").
KNOWN_ERRATA = {"OSCmd,1;", "ConnectToBox,0,;"}


def _documented_strings():
    inline = re.findall(r"`([A-Za-z][A-Za-z0-9]*,[^`]*;)`", COMMANDS_MD)
    blocks = re.findall(r"(?m)^(WayPoint,0,\S*;)", COMMANDS_MD)
    return [s for s in inline + blocks if "…" not in s and not s.startswith("Name,")]


def _is_format_line(fields):
    return any(re.match(r"(n|d|s|str)[A-Z]", f) for f in fields)


def test_every_documented_example_matches_the_table():
    checked = 0
    for s in _documented_strings():
        name = s.split(",", 1)[0]
        if name in FORCE_OMITTED or s in KNOWN_ERRATA:
            continue
        assert name in P.COMMANDS, f"documented but missing from the table: {s}"
        fields = s[:-2].split(",")[1:] if s.endswith(",;") else None
        assert fields is not None, s
        if fields == [""]:
            fields = []
        if fields[:1] == ["OK"]:
            P.parse_reply(name, fields[1:])          # reply example: count and types
        elif _is_format_line(fields):
            assert [p.name for p in P.COMMANDS[name].params] == fields, s
        else:
            P.decode_message(s)                      # command example: count and types
        checked += 1
    assert checked >= 30


def test_forbidden_and_unused_are_refused():
    for name in P.FORBIDDEN_NAMES:
        with pytest.raises(P.RefusedCommand):
            P.check_sendable(name)
    with pytest.raises(P.RefusedCommand):
        P.check_sendable("SetOverride")
    with pytest.raises(P.RefusedCommand):
        P.check_sendable("SetForceZero")
    assert P.check_sendable("PushServoJ").page == 173


def test_float_formatting_is_fixed_point_with_4_decimals():
    assert P.encode("StartServo", 0, 0.2, 0.02) == "StartServo,0,0.2000,0.0200,;"
    assert P.encode("PushServoJ", 0, 0, -0.0, 1e-5, 12.34567, -170, 90) == \
        "PushServoJ,0,0.0000,-0.0000,0.0000,12.3457,-170.0000,90.0000,;"
    assert P.encode("ReadRobotModel") == "ReadRobotModel,;"
    assert P.encode("GrpEnable", 0) == "GrpEnable,0,;"


@pytest.mark.parametrize("bad", [math.nan, math.inf, -math.inf, True, "1.0", None])
def test_non_finite_and_mistyped_floats_are_refused(bad):
    with pytest.raises(P.ProtocolError):
        P.encode("StartServo", 0, bad, 0.02)


@pytest.mark.parametrize("bad", ["", "a,b", "a;b", "a b", "Ünï"])
def test_bad_strings_are_refused(bad):
    with pytest.raises(P.ProtocolError):
        P.encode("WayPoint", 0, *[0.0] * 12, bad, "Base", 20, 100, 0, 0, 1, 0, 0, 0, "1")


def test_int_parameters_refuse_floats_and_bools():
    for bad in (0.0, True, "0"):
        with pytest.raises(P.ProtocolError):
            P.encode("GrpEnable", bad)


def test_parameter_count_is_checked():
    with pytest.raises(P.ProtocolError):
        P.encode("PushServoJ", 0, 1, 2, 3)


def test_plain_int_override(monkeypatch):
    monkeypatch.setitem(P.FORMAT_OVERRIDES, ("WayPoint", "dVelocity"), "plain_int")
    s = P.encode("WayPoint", 0, *[0.0] * 12, "TCP", "Base", 20, 100, 0, 0, 1, 0, 0, 0, "1")
    assert ",TCP,Base,20,100.0000,0.0000,0,1," in s


def test_decode_refuses_exponents_and_bad_counts():
    with pytest.raises(P.ProtocolError):
        P.decode_message("StartServo,0,1e-05,0.02,;")
    with pytest.raises(P.ProtocolError):
        P.decode_message("StartServo,0,0.2,;")
    with pytest.raises(P.ProtocolError):
        P.decode_message("StartServo,0,0.2,0.02;")
    assert P.decode_message("StartServo,0,0.2,0.02,;") == ("StartServo", [0, 0.2, 0.02])


def test_reply_parsing():
    r = P.parse_reply("ReadRobotState", "1 1 0 0 0 0 0 0 0 1 1 0 0".split())
    assert r["nMovingState"] == 1 and r["nElectrify"] == 1 and len(r) == 13
    with pytest.raises(P.ProtocolError):
        P.parse_reply("ReadRobotState", ["1"] * 12)
    with pytest.raises(P.ProtocolError):
        P.parse_reply("ReadRobotState", ["x"] * 13)
