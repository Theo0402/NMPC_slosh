"""Arm control (stage 1.6): jog, points and path runs through the workbench, against the mock and the dry
run. Every move must be a linear WayPoint at the jog speed with the orientation held; refusals must not
end the session; the stop button must end a move with GrpStop and the session with GrpDisable."""
import time

import pytest

from analysis.logread import read_trial
from tests.workbench.test_workbench import temp_configs
from tools.elfin_py import jog
from tools.workbench.bench import Refused, Workbench


@pytest.fixture
def bench(tmp_path):
    confirmed, arm = temp_configs(tmp_path)
    b = Workbench(data_dir=tmp_path / "data", journal=tmp_path / "journal.md", confirmed=confirmed,
                  arm_config=arm, host_problem="this is a test, not the Linux machine")
    b.edit_coeffs({"jog_speed_mm_s": 20.0, "jog_acc_mm_s2": 100.0}, "faster for the tests (still = confirmed)")
    yield b
    if b.run is not None and b.run.running:
        b.run.stop()
        b.run.thread.join(timeout=30)


def connect(b, mode="mock"):
    b.start("arm_control", mode, assume_yes=True)
    t_end = time.monotonic() + 30
    while b.run.arm["ready"] is None and b.run.running and time.monotonic() < t_end:
        time.sleep(0.05)
    assert b.run.arm["ready"] is not None, list(b.run.console)[-10:]
    return b.run.arm["ready"]


def do(b, name, args, timeout=30):
    cid = b.command(name, args)["id"]
    t_end = time.monotonic() + timeout
    while cid not in b.run.arm["results"] and b.run.running and time.monotonic() < t_end:
        time.sleep(0.05)
    return b.run.arm["results"].get(cid)


def finish(b):
    t_end = time.monotonic() + 30
    while b.run.running and time.monotonic() < t_end:
        time.sleep(0.05)
    b.run.thread.join(timeout=10)
    return read_trial(b.run.trial_path)


def sent(trial):
    return [r["data"].decode() for r in trial.stream("wire_tx")]


def test_settings_come_from_the_coefficient_snapshot():
    assert jog.settings(None) == jog.DEFAULTS
    s = jog.settings({"coeffs": {"jog_speed_mm_s": {"value": 5.0, "source": "hand-set"}, "path_dwell_s": {"value": None}}})
    assert s["jog_speed_mm_s"] == 5.0 and s["path_dwell_s"] == jog.DEFAULTS["path_dwell_s"]


def test_jog_moves_one_axis_linearly_with_the_orientation_held(bench):
    home = connect(bench)
    assert do(bench, "jog", {"axis": "x", "dir": 1, "step_mm": 10})["ok"]
    assert do(bench, "jog", {"axis": "z", "dir": -1, "step_mm": 5})["ok"]
    pose = bench.run.arm["pose"]["pose"]
    assert pose[:3] == pytest.approx([home["pose"][0] + 10, home["pose"][1], home["pose"][2] - 5])
    assert pose[3:] == pytest.approx(home["pose"][3:])
    assert do(bench, "end", {})["ok"]
    trial = finish(bench)
    assert bench.run.exit_code == 0
    moves = [s.split(",") for s in sent(trial) if s.startswith("WayPoint")]
    assert len(moves) == 2
    for m in moves:                                   # linear (nMoveType 1) at the jog speed and acceleration
        assert m[19] == "1" and float(m[16]) == 20.0 and float(m[17]) == 100.0
        assert m[14:16] == ["TCP", "Base"]
    assert sent(trial)[-1].startswith("GrpDisable")


def test_refusals_do_not_end_the_session(bench):
    home = connect(bench)
    r = do(bench, "jog", {"axis": "y", "dir": 1, "step_mm": 500})
    assert not r["ok"] and "jog_max_step_mm" in r["error"]
    far = {"name": "far", "pose": [home["pose"][0] + 400] + home["pose"][1:], "tool": home["tool"], "user": home["user"]}
    r = do(bench, "goto", {"point": far})
    assert not r["ok"] and "jog box" in r["error"]
    tilted = {"name": "tilt", "pose": home["pose"][:5] + [home["pose"][5] + 10], "tool": home["tool"], "user": home["user"]}
    r = do(bench, "goto", {"point": tilted})
    assert not r["ok"] and "rotate" in r["error"]
    assert bench.run.running and do(bench, "jog", {"axis": "y", "dir": -1, "step_mm": 1})["ok"]
    do(bench, "end", {})
    trial = finish(bench)
    assert len([s for s in sent(trial) if s.startswith("WayPoint")]) == 1


def test_path_runs_start_middle_end_after_checking_every_point(bench):
    home = connect(bench)
    pt = lambda n, dx, dz=0.0: {"name": n, "pose": [home["pose"][0] + dx, home["pose"][1], home["pose"][2] + dz]
                                + home["pose"][3:], "tool": home["tool"], "user": home["user"]}
    bad = do(bench, "path", {"points": [pt("S", 0), pt("M1", 300), pt("E", 20)], "dwell_s": 0})
    assert not bad["ok"] and "jog box" in bad["error"]      # refused before any move
    r = do(bench, "path", {"points": [pt("S", 0), pt("M1", 15, -5), pt("E", 30)], "dwell_s": 0.1})
    assert r["ok"], r
    assert bench.run.arm["pose"]["pose"][:3] == pytest.approx([home["pose"][0] + 30, home["pose"][1], home["pose"][2]])
    do(bench, "end", {})
    trial = finish(bench)
    assert len([s for s in sent(trial) if s.startswith("WayPoint")]) == 2   # S equals the start pose: no move


def test_stop_during_a_move_sends_grpstop_then_disables(bench):
    home = connect(bench)
    bench.edit_coeffs({"jog_speed_mm_s": 2.0}, "slow, so the stop lands mid-move")     # next session only
    cid = bench.command("jog", {"axis": "x", "dir": 1, "step_mm": 50})["id"]
    time.sleep(1.0)
    assert bench.run.arm["busy"] is not None
    bench.stop()
    trial = finish(bench)
    s = [x.split(",", 1)[0] for x in sent(trial)]
    assert "GrpStop" in s and s[-1] == "GrpDisable" and s.index("GrpStop") < len(s) - 1
    texts = [r["text"].decode() for r in trial.stream("event")]
    assert any("stop requested: stop button" in t for t in texts)
    assert bench.run.exit_code == 2 and cid not in bench.run.arm["results"]


def test_commands_need_a_ready_session(bench):
    with pytest.raises(Refused, match="no arm-control session"):
        bench.command("jog", {"axis": "x", "dir": 1, "step_mm": 1})


def test_dry_run_session(bench):
    connect(bench, "dry-run")
    assert do(bench, "jog", {"axis": "y", "dir": 1, "step_mm": 2})["ok"]
    do(bench, "end", {})
    finish(bench)
    assert bench.run.exit_code == 0


def test_saved_paths(bench):
    p = {"name": "start", "role": "start", "pose": [420, 0, 445, 180, 0, 180], "tool": [0] * 6, "user": [0] * 6}
    bench.save_path("demo1", [p, dict(p, name="end", role="end")], "first path")
    assert [x["role"] for x in bench.paths()["demo1"]["points"]] == ["start", "end"]
    with pytest.raises(Refused, match="one start"):
        bench.save_path("demo2", [p, p])
    with pytest.raises(Refused, match="path name"):
        bench.save_path("../evil", [p])


def test_practice_uses_the_mock_limits_and_never_unlocks_the_arm(tmp_path):
    confirmed, arm = temp_configs(tmp_path, limits=False)          # like the real file before stage 1.1
    b = Workbench(data_dir=tmp_path / "data", journal=tmp_path / "journal.md", confirmed=confirmed, arm_config=arm,
                  host_problem="")
    with pytest.raises(Refused, match="dry run and mock only"):
        b.start("arm_control", "arm", practice=True)
    b.start("arm_control", "mock", assume_yes=True, practice=True)
    t_end = time.monotonic() + 30
    while b.run.arm["ready"] is None and b.run.running and time.monotonic() < t_end:
        time.sleep(0.05)
    assert b.run.arm["ready"] is not None, list(b.run.console)[-5:]
    do(b, "end", {})
    trial = finish(b)
    assert "PRACTICE COPY" in trial.header["config"]["e03pro.yaml"]
    assert "limits_readback:\n  record_id: null" in arm.read_text(encoding="utf-8")   # the real file is untouched
    for _ in range(50):
        if b.status.runs("arm_control", "mock"):
            break
        time.sleep(0.1)
    assert b.status.runs("arm_control", "mock")[-1]["practice"] is True
    assert b.status.status("arm_control", "mock")["status"] == "not run"


def test_idle_timeout_ends_the_session_and_disables_the_arm(bench):
    bench.edit_coeffs({"jog_idle_timeout_s": 5.0}, "short, for the test")
    ready = connect(bench)
    assert ready["settings"]["jog_idle_timeout_s"] == 5.0
    t0 = time.monotonic()
    time.sleep(3.0)
    left = bench.run.arm["pose"]["idle_left_s"]
    assert 0 < left <= 2.6                                  # every pose report carries the seconds left
    assert do(bench, "keep", {})["ok"]                      # keep connected: the timer restarts
    time.sleep(3.5)
    assert bench.run.running                                # 6.5 s after connecting: still on
    trial = finish(bench)                                   # ends by itself, about 5 s after the keep
    assert time.monotonic() - t0 < 20
    assert bench.run.exit_code == 0 and bench.run.arm["idle_timeout"]["after_s"] == 5.0
    texts = [r["text"].decode() for r in trial.stream("event")]
    assert any(t.startswith("idle timeout: no command for 5 s") for t in texts)
    assert sent(trial)[-1].startswith("GrpDisable")


def test_every_question_comes_before_the_arm_is_enabled(bench):
    connect(bench)
    do(bench, "end", {})
    trial = finish(bench)
    # the log is append-only, so record order is time order (Windows' monotonic clock ticks only every 15.6 ms)
    order = list(enumerate(trial.records))
    asked = [i for i, (s, r) in order if s == "event" and r["text"].decode().startswith("prompt")]
    enable = [i for i, (s, r) in order if s == "wire_tx" and r["data"].startswith(b"GrpEnable")]
    assert len(asked) == 2 and len(enable) == 1 and max(asked) < enable[0]
