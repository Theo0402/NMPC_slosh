"""The workspace fan (owner request, 2026-11-26): a wedge round the base's Z axis, taught by free drive at its edges.
No jog or point move may end outside it, no straight move may cut through the clearance round the base, and an arm
outside it may only be brought back towards it."""
import pytest

from tests.workbench.test_arm_control import bench, connect, do  # noqa: F401 (bench is a fixture)
from tests.workbench.test_workbench import wait_run
from tools.elfin_py import jog, link

FAN = [-30.0, 30.0, 300.0, 500.0, 400.0, 500.0]      # round the mock's start pose X 420, Y 0, Z 445 (angle 0, r 420)
OWNER = [176.4, 273.8, 150.0, 764.6, 80.0, 667.5]    # the owner's fan, 2026-11-26


# ------------------------------------------------------------------ the check itself
def test_inside_passes_and_each_measure_outside_is_refused():
    jog.check_fan(FAN, [420.0, 0.0, 445.0])
    jog.check_fan(FAN, [400.0, 230.0, 500.0])                             # angle 29.9 deg, Z max: at the edges
    for target, what in (([400.0, 300.0, 445.0], "angle"), ([200.0, 0.0, 445.0], "below r_min"),
                         ([520.0, 0.0, 445.0], "above r_max"), ([420.0, 0.0, 390.0], "below z_min"),
                         ([420.0, 0.0, 510.0], "above z_max")):
        with pytest.raises(jog.CommandRefused, match=what):
            jog.check_fan(FAN, target)
    jog.check_fan(None, [9999.0, 0.0, 0.0])                               # no fan: nothing to check


def test_angles_wrap_round_180_and_0():
    jog.check_fan(OWNER, [-500.0, 0.0, 300.0])                            # 180 deg: inside the owner's fan
    jog.check_fan(OWNER, [-300.0, -300.0, 300.0])                         # 225 deg
    with pytest.raises(jog.CommandRefused, match="angle"):
        jog.check_fan(OWNER, [300.0, -300.0, 300.0])                      # 315 deg
    jog.check_fan([350.0, 20.0, 100.0, 800.0, 0.0, 900.0], [500.0, 0.0, 100.0])   # a fan across 0 deg


def test_a_straight_move_may_not_cut_through_the_clearance():
    fan = [-80.0, 80.0, 300.0, 800.0, 0.0, 1000.0]
    a, b = [100.0, -390.0, 500.0], [100.0, 390.0, 500.0]                 # both inside (r 403); the chord passes 100 mm out
    jog.check_fan(fan, b)
    with pytest.raises(jog.CommandRefused, match="go round in two moves"):
        jog.check_fan(fan, b, a)
    jog.check_fan(fan, [600.0, 0.0, 500.0], a)                           # outwards: fine
    jog.check_fan(fan, [600.0, 0.0, 500.0], [200.0, 0.0, 500.0])         # from inside the clearance, outwards: fine


def test_an_arm_outside_may_only_come_back():
    now = [550.0, 0.0, 445.0]                                             # 50 mm beyond r_max
    jog.check_fan(FAN, [530.0, 0.0, 445.0], now)                          # back towards the fan
    with pytest.raises(jog.CommandRefused, match="only moves back towards it"):
        jog.check_fan(FAN, [560.0, 0.0, 445.0], now)                      # further out
    with pytest.raises(jog.CommandRefused, match="above z_max"):
        jog.check_fan(FAN, [530.0, 0.0, 510.0], now)                      # back in r but out in Z
    with pytest.raises(jog.CommandRefused, match="above r_max"):
        jog.check_fan(FAN, [530.0, 0.0, 445.0], now, inside=True)         # path points must be inside


def test_unusable_fans_refuse_the_session():
    s = dict(jog.DEFAULTS)
    jog.check_settings(dict(s, workspace_fan=FAN))
    jog.check_settings(dict(s, workspace_fan=OWNER))
    for bad, why in (([0.0, 200.0, 100.0, 500.0, 0.0, 100.0], "at most 180"),
                     ([10.0, 10.0, 100.0, 500.0, 0.0, 100.0], "more than 0"),
                     ([0.0, 90.0, 500.0, 100.0, 0.0, 100.0], "r_min"),
                     ([0.0, 90.0, -1.0, 100.0, 0.0, 100.0], "r_min"),
                     ([0.0, 90.0, 100.0, 500.0, 100.0, 0.0], "z_min"),
                     ([0.0, 90.0, 100.0], "6 numbers"), ("fan", "6 numbers")):
        with pytest.raises(link.Refused, match=why):
            jog.check_settings(dict(s, workspace_fan=bad))


# ------------------------------------------------------------------ enforced by the jog tool (mock)
def test_the_jog_tool_keeps_moves_inside_the_saved_fan(bench):
    bench.edit_coeffs({"workspace_fan": FAN}, "test fan")
    ready = connect(bench)
    assert ready["settings"]["workspace_fan"] == FAN
    try:
        assert do(bench, "jog", {"axis": "x", "dir": 1, "step_mm": 10})["ok"]           # r 430: inside
        r = do(bench, "jog", {"axis": "x", "dir": 1, "step_mm": 40})                    # r 470: inside
        assert r["ok"], r
        r = do(bench, "jog", {"axis": "x", "dir": 1, "step_mm": 40})                    # r 510: outside
        assert not r["ok"] and "above r_max" in r["error"]
        far = {"name": "far", "pose": [460.0, 0.0, 470.0, 180.0, 0.0, 180.0], "tool": [0.0] * 6, "user": [0.0] * 6}
        out = {"name": "out", "pose": [460.0, 0.0, 520.0, 180.0, 0.0, 180.0], "tool": [0.0] * 6, "user": [0.0] * 6}
        r = do(bench, "path", {"points": [far, out], "dwell_s": 0})
        assert not r["ok"] and "above z_max" in r["error"]                              # refused before any move
        assert bench.run.running                                                         # refusals never end it
        do(bench, "end", {})
        assert wait_run(bench)["exit_code"] == 0
    finally:
        if bench.run.running:
            bench.run.stop()
            bench.run.thread.join(timeout=30)


def test_the_fan_is_a_documented_coefficient():
    from tests.workbench.test_workbench import ROOT
    assert "workspace_fan" in (ROOT / "docs/stages.md").read_text(encoding="utf-8")
    assert "workspace_fan" in (ROOT / "config/workbench/stages.yaml").read_text(encoding="utf-8")
