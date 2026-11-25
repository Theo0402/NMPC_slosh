"""The workspace box (owner request, 2026-11-25): a fixed cuboid taught on the Arm control tab by jogging to the corners
of the free space. No jog or point move may end outside it; an arm outside it may only be brought back towards it."""
import pytest

from analysis.logread import read_trial
from tests.workbench.test_arm_control import bench, connect, do  # noqa: F401 (bench is a fixture)
from tests.workbench.test_workbench import wait_run
from tools.elfin_py import jog, link
from tools.workbench.bench import Refused

WS = [400.0, 440.0, -20.0, 20.0, 430.0, 460.0]        # round the mock's start pose X 420, Y 0, Z 445


# ------------------------------------------------------------------ the check itself
def test_targets_inside_pass_and_outside_are_refused():
    jog.check_workspace(WS, [420.0, 0.0, 445.0])
    jog.check_workspace(WS, [400.0, 20.0, 460.0])                         # on the faces counts as inside
    for target in ([441.0, 0.0, 445.0], [420.0, -25.0, 445.0], [420.0, 0.0, 429.0]):
        with pytest.raises(jog.CommandRefused, match="outside the workspace box"):
            jog.check_workspace(WS, target)
    jog.check_workspace(None, [9999.0, 0.0, 0.0])                         # no box: nothing to check


def test_an_arm_outside_may_only_come_back():
    now = [460.0, 0.0, 445.0]                                             # 20 mm beyond X max
    jog.check_workspace(WS, [450.0, 0.0, 445.0], now)                     # back towards the box
    jog.check_workspace(WS, [460.0, 0.0, 440.0], now)                     # along an axis that is inside
    with pytest.raises(jog.CommandRefused, match="only moves back towards it"):
        jog.check_workspace(WS, [470.0, 0.0, 445.0], now)                 # further out
    with pytest.raises(jog.CommandRefused, match="outside the workspace box"):
        jog.check_workspace(WS, [450.0, 0.0, 470.0], now)                 # back in X but out in Z


def test_unusable_boxes_refuse_the_session():
    s = dict(jog.DEFAULTS)
    jog.check_settings(dict(s, workspace_box_mm=WS))
    for bad in ([400.0, 440.0, 20.0, -20.0, 430.0, 460.0], [1.0, 2.0, 3.0], "box"):
        with pytest.raises(link.Refused, match="workspace_box_mm"):
            jog.check_settings(dict(s, workspace_box_mm=bad))


# ------------------------------------------------------------------ teaching it on the workbench
def test_corners_become_a_new_coefficient_version_with_a_record(bench):
    v0 = bench.coeffs.current_version()
    r = bench.set_workspace([[400, 20, 430], [440, -20, 460], [420, 0, 445]], 5, "small bench, taught by jogging")
    assert r["box"] == [405.0, 435.0, -15.0, 15.0, 435.0, 455.0] and r["version"] == v0 + 1
    view = bench.workspace_view()
    assert view["box"] == r["box"] and "3 corners, 5 mm margin" in view["source"]
    assert view["last_record"]["corners"][0] == [400.0, 20.0, 430.0] and view["last_record"]["coeff_version"] == v0 + 1
    assert bench.clear_workspace("moved the arm")["box"] is None and bench.workspace_view()["box"] is None


@pytest.mark.parametrize("corners, margin, note, why", [
    ([[400, 0, 430]], 0, "n", "at least two corners"),
    ([[400, 0, 430], [440, 0, 460]], 0, "n", "only 0.0 mm along Y"),
    ([[400, -20, 430], [440, 20, 460]], 16, "n", "only 8.0 mm along X"),
    ([[400, -20, 430], [440, 20, 460]], -1, "n", "margin"),
    ([[400, -20, 430], ["a", 20, 460]], 0, "n", "X, Y, Z"),
    ([[400, -20, 430], [440, 20, 460]], 0, " ", "note"),
])
def test_unusable_corners_are_refused(bench, corners, margin, note, why):
    v0 = bench.coeffs.current_version()
    with pytest.raises(Refused, match=why):
        bench.set_workspace(corners, margin, note)
    assert bench.coeffs.current_version() == v0                            # nothing saved


# ------------------------------------------------------------------ enforced by the jog tool (mock)
def test_the_jog_tool_keeps_moves_inside_the_saved_box(bench):
    bench.set_workspace([[400, -20, 430], [440, 20, 460]], 0, "test box")
    ready = connect(bench)
    assert ready["settings"]["workspace_box_mm"] == WS
    try:
        assert do(bench, "jog", {"axis": "x", "dir": 1, "step_mm": 10})["ok"]           # X 430: inside
        r = do(bench, "jog", {"axis": "x", "dir": 1, "step_mm": 25})                    # X 455: outside
        assert not r["ok"] and "outside the workspace box" in r["error"]
        far = {"name": "far", "pose": [420.0, 0.0, 470.0, 180.0, 0.0, 180.0], "tool": [0.0] * 6, "user": [0.0] * 6}
        r = do(bench, "path", {"points": [far], "dwell_s": 0})
        assert not r["ok"] and "outside the workspace box" in r["error"]
        assert bench.run.running                                                         # refusals never end it
        do(bench, "end", {})
        run = wait_run(bench)
        assert run["exit_code"] == 0
        moves = [r["data"].decode() for r in read_trial(run["trial_path"]).stream("wire_tx")
                 if r["data"].startswith(b"WayPoint")]
        assert len(moves) == 1                                                           # only the move inside
    finally:
        if bench.run.running:
            bench.run.stop()
            bench.run.thread.join(timeout=30)


def test_a_box_saved_during_a_session_applies_from_the_next_one(bench):
    connect(bench)
    try:
        bench.set_workspace([[400, -20, 430], [440, 20, 460]], 0, "saved while connected")
        assert bench.run.arm["ready"]["settings"]["workspace_box_mm"] is None             # this session: unchanged
        assert do(bench, "jog", {"axis": "x", "dir": 1, "step_mm": 25})["ok"]
        do(bench, "end", {})
        wait_run(bench)
        assert connect(bench)["settings"]["workspace_box_mm"] == WS
    finally:
        if bench.run.running:
            bench.run.stop()
            bench.run.thread.join(timeout=30)


def test_the_page_and_the_api_have_the_workspace_box():
    from tests.workbench.test_workbench import ROOT
    page = (ROOT / "tools/workbench/static/index.html").read_text(encoding="utf-8")
    assert "Record corner here" in page and "/api/workspace" in page
    server = (ROOT / "tools/workbench/server.py").read_text(encoding="utf-8")
    assert '"/api/workspace"' in server and '"/api/workspace/clear"' in server
    assert "workspace_box_mm" in (ROOT / "docs/stages.md").read_text(encoding="utf-8")
