"""Every tool that can move the arm runs with --dry-run and --mock; an injected Fail stops it; late
replies are flagged; requests above confirmed.yaml are refused (codebase plan §9, week 1)."""
import _thread
import pathlib
import re
import threading
import time

import pytest

from analysis.logread import read_trial
from tools.elfin_mock import sim
from tools.elfin_py import bringup_read, config, link, motion, move_test, pose_recorder, servo_ladder, state_stream, triallog

ROOT = pathlib.Path(__file__).resolve().parents[2]
MOCK_LIMITS = """limits_readback:
  record_id: test
  joint_max_vel_deg_s: [170.0, 170.0, 170.0, 170.0, 180.0, 180.0]
  joint_max_acc_deg_s2: [360.0, 360.0, 360.0, 360.0, 360.0, 360.0]
  joint_max_jerk_deg_s3: [3600.0, 3600.0, 3600.0, 3600.0, 3600.0, 3600.0]
  linear_max: [1000.0, 2500.0, 10000000.0]
  tolerance: 0.001
"""
NO_LIMITS = """limits_readback:
  record_id: null
  joint_max_vel_deg_s: null
  joint_max_acc_deg_s2: null
  joint_max_jerk_deg_s3: null
  linear_max: null
  tolerance: 0.001
"""                                                    # e03pro.yaml as it was before stage 1.1 read the arm
SHORT = motion.LadderProfile(amplitude_deg=1.0, period_s=2.0, motion_s=2.0, taper_s=0.5, hold_s=0.3)


def configs(tmp_path, limits=True, period=0.1, ranges=None):
    arm = (ROOT / "config/arm/e03pro.yaml").read_text(encoding="utf-8")
    start = arm.index("limits_readback:")
    end = arm.index("\n\n", start)
    arm = arm[:start] + (MOCK_LIMITS if limits else NO_LIMITS).rstrip("\n") + arm[end:]
    # the tests set the joint ranges themselves (null unless given), whatever the real file holds
    arm = re.sub(r"(?m)^joint_ranges_deg:.*$", f"joint_ranges_deg: {ranges if ranges else 'null'}", arm)
    conf = (ROOT / "config/arm/confirmed.yaml").read_text(encoding="utf-8")
    conf = conf.replace("servo_period_s: 0.2 ", f"servo_period_s: {period} ")
    arm = arm.replace("period_s: 0.2 ", f"period_s: {max(period, 0.1)} ")
    (tmp_path / "e03pro.yaml").write_text(arm, encoding="utf-8")
    (tmp_path / "confirmed.yaml").write_text(conf, encoding="utf-8")
    return ["--confirmed", str(tmp_path / "confirmed.yaml"), "--arm-config", str(tmp_path / "e03pro.yaml"),
            "--data-dir", str(tmp_path / "data"), "--assume-yes"]


def mock_args(srv):
    return ["--mock", "--cmd-port", str(srv.cmd_port), "--state-port", str(srv.state_port)]


def only_trial(tmp_path):
    files = sorted((tmp_path / "data" / "trials").glob("*.bin"))
    assert len(files) == 1
    return read_trial(files[0])


def sent(trial):
    return [r["data"].decode() for r in trial.stream("wire_tx")]


def names(trial):
    return [s.split(",", 1)[0] for s in sent(trial)]


def last_event(trial):
    """The last event before the 'trial log closed' marker, which every closed log ends with (review D8)."""
    ev = trial.stream("event")
    assert ev[-1]["kind"] == triallog.EV_MARKER and ev[-1]["text"] == triallog.CLOSED.encode()
    return ev[-2]


def test_config_files_load():
    confirmed, arm, texts = config.load()
    assert confirmed["servo_period_s"] == 0.2 and confirmed["joint_acc_deg_s2"] == 100.0
    assert arm["mounting"]["payload_kg"] == 0.0
    assert config.limits_known(arm)                    # recorded from the arm by stage 1.1 (2026-11-24)


def test_bringup_read_dry_run(tmp_path, capsys):
    assert bringup_read.main(["--dry-run"] + configs(tmp_path, limits=False)) == 0
    t = only_trial(tmp_path)
    assert t.header["mode"] == "dry-run" and t.header["dataset"] == "scratch"
    assert "ReadRobotModel,;" in sent(t) and "ReadLinearMaxVel,0,;" in sent(t)
    assert not {"GrpEnable", "Electrify", "WayPoint"} & set(names(t))
    assert "record_id: " + t.header["trial_id"] in capsys.readouterr().out


def test_bringup_read_mock_logs_fragments(tmp_path, mock):
    assert bringup_read.main(mock_args(mock) + configs(tmp_path)) == 0
    t = only_trial(tmp_path)
    rx = t.stream("wire_rx")
    assert len(rx) > len(t.stream("wire_tx"))                 # replies arrived in fragments, logged as they came
    assert t.stream("event")[0]["kind"] == 5                    # the arm block


def test_state_stream_mock(tmp_path, mock):
    assert state_stream.main(mock_args(mock) + configs(tmp_path) + ["--seconds", "1.0"]) == 0
    t = only_trial(tmp_path)
    assert len(t.stream("state_raw")) >= 15 and len(t.stream("robot_state")) == len(t.stream("state_raw"))
    assert t.stream("wire_tx") == []                             # port 10003 not used


def test_move_test_mock(tmp_path, mock):
    assert move_test.main(mock_args(mock) + configs(tmp_path)) == 0
    t = only_trial(tmp_path)
    n = names(t)
    assert n.index("GrpEnable") < n.index("WayPoint") and n[-1] == "GrpDisable"
    assert ",20.0000,100.0000,0.0000,0,1,0,0,0,1,;" in sent(t)[n.index("WayPoint")]
    assert mock.arm.stats["forbidden"] == 0 and mock.arm.stats["dropped_extra_messages"] == 0
    assert abs(mock.arm.q[5] - (sim.MOCK_HOME_DEG[5] + 10.0)) < 1e-6
    assert abs(mock.arm.pose[2] - (sim.MOCK_HOME_POSE[2] + 20.0)) < 1e-6


def _move_test_metrics(tmp_path, mock_factory, name, fail_rules):
    from tools.workbench.evaluate import tool_metrics
    srv = mock_factory(fail_rules=fail_rules)
    (tmp_path / name).mkdir()
    code = move_test.main(mock_args(srv) + configs(tmp_path / name))
    trial = sorted((tmp_path / name / "data" / "trials").glob("*.bin"))[0]
    return tool_metrics("tools.elfin_py.move_test", trial, code, None), names(read_trial(trial))


def test_failed_grpdisable_fails_the_motion_stage(tmp_path, mock_factory):
    """The 1.3 arm runs of 2026-11-25 ended with 'GrpDisable,Fail,20018' and first read "criteria met"."""
    m, n = _move_test_metrics(tmp_path, mock_factory, "clean", [])
    assert m["completed"] is True and m["grpdisable_ok"] is True and n[-1] == "GrpDisable"
    m, n = _move_test_metrics(tmp_path, mock_factory, "refused", [("GrpDisable", 1, 20018)])
    assert n.count("GrpDisable") == 1                    # one try; the operator disables from the pendant
    assert m["fail_replies"] == 1 and m["grpdisable_ok"] is False and m["completed"] is False


def test_move_test_dry_run_with_return(tmp_path):
    assert move_test.main(["--dry-run", "--return"] + configs(tmp_path)) == 0
    assert names(only_trial(tmp_path)).count("WayPoint") == 4


def test_move_test_returns_with_a_straight_move_last(tmp_path, mock):
    """J6 back first, then the flange down: GrpDisable answered 20018 only after a finished joint move (Q20)."""
    assert move_test.main(mock_args(mock) + configs(tmp_path) + ["--return"]) == 0
    moves = [s.split(",") for s in sent(only_trial(tmp_path)) if s.startswith("WayPoint")]
    assert [m[19] for m in moves] == ["0", "1", "0", "1"]                 # nMoveType: joint, linear, joint, linear
    assert abs(mock.arm.q[5] - sim.MOCK_HOME_DEG[5]) < 1e-6 and abs(mock.arm.pose[2] - sim.MOCK_HOME_POSE[2]) < 1e-6


def test_refuses_without_recorded_limits(tmp_path, mock):
    with pytest.raises(link.Refused, match="not recorded"):
        move_test.main(mock_args(mock) + configs(tmp_path, limits=False))
    t = only_trial(tmp_path)
    assert "GrpEnable" not in names(t) and last_event(t)["kind"] == 7


def test_refuses_unpowered_and_never_sends_electrify(tmp_path, mock_factory):
    srv = mock_factory(powered=False)
    with pytest.raises(link.Refused, match="unpowered"):
        move_test.main(mock_args(srv) + configs(tmp_path))
    assert not {"Electrify", "GrpEnable"} & set(names(only_trial(tmp_path)))


def test_refuses_estop_channel_disagreement(tmp_path, mock_factory):
    srv = mock_factory(estop_channel_mismatch=True)
    with pytest.raises(link.Refused, match="disagree"):
        move_test.main(mock_args(srv) + configs(tmp_path))


def test_refuses_nonzero_frames(tmp_path, mock_factory):
    srv = mock_factory(tool_frame=[0.0, 0.0, 72.0, 0.0, 0.0, 0.0])
    with pytest.raises(link.Refused, match="tool_frame"):
        move_test.main(mock_args(srv) + configs(tmp_path))


def test_error_asks_before_grpreset(tmp_path, mock_factory, monkeypatch):
    srv = mock_factory()
    srv.arm.error_code, srv.arm.error_axis = 12345, 2
    args = mock_args(srv) + [a for a in configs(tmp_path) if a != "--assume-yes"]
    monkeypatch.setattr("builtins.input", lambda prompt: "no")
    with pytest.raises(link.Refused, match="declined GrpReset"):
        move_test.main(args)
    n = names(only_trial(tmp_path))
    assert "ReadAxisErrorCode" in n and "GrpReset" not in n


def test_error_cleared_after_yes(tmp_path, mock_factory):
    srv = mock_factory()
    srv.arm.error_code, srv.arm.error_axis = 12345, 2
    assert move_test.main(mock_args(srv) + configs(tmp_path)) == 0
    n = names(only_trial(tmp_path))
    assert n.index("ReadAxisErrorCode") < n.index("GrpReset") < n.index("GrpEnable")


def test_assume_yes_and_windows_refused_on_arm(tmp_path):
    with pytest.raises(SystemExit):
        bringup_read.main(["--arm"] + configs(tmp_path))


def test_wsl_detection(tmp_path):
    pv = tmp_path / "version"
    pv.write_text("Linux version 5.15.167.4-microsoft-standard-WSL2 (gcc ...)", encoding="utf-8")
    assert link.running_in_wsl(str(pv), environ={})
    pv.write_text("Linux version 6.8.0-45-generic (buildd@lcy02) #45-Ubuntu SMP PREEMPT_RT", encoding="utf-8")
    assert not link.running_in_wsl(str(pv), environ={})
    assert link.running_in_wsl(str(pv), environ={"WSL_DISTRO_NAME": "Ubuntu-24.04"})
    assert not link.running_in_wsl(str(tmp_path / "missing"), environ={})


def test_arm_host_rules():
    assert "win32" in link.arm_host_problem(platform="win32", wsl=False)
    assert "WSL" in link.arm_host_problem(platform="linux", wsl=True)
    assert link.arm_host_problem(platform="linux", wsl=False) is None


@pytest.mark.skipif(link.arm_host_problem() is None, reason="this is the Linux machine: --arm would really connect")
def test_arm_refused_on_development_machines(tmp_path):
    args = ["--arm", "--data-dir", str(tmp_path / "data")]      # --arm takes only the owner's config files (S14)
    with pytest.raises(link.Refused, match="Linux machine only"):
        bringup_read.main(args)
    t = only_trial(tmp_path)
    assert t.stream("wire_tx") == [] and last_event(t)["kind"] == 7


def test_ladder_mock_completes(tmp_path, mock):
    rc = servo_ladder.run(mock_args(mock) + configs(tmp_path, period=0.1), profile=SHORT)
    t = only_trial(tmp_path)
    n = names(t)
    assert n.index("StartServo") < n.index("PushServoJ") < n.index("GrpStop") < len(n) - 1 and n[-1] == "GrpDisable"
    planned = len(motion.sample(SHORT, sim.MOCK_HOME_DEG, 0.1))
    assert len(t.stream("arm_cmd")) == len(t.stream("arm_ack")) == planned
    assert all(a["status"] == 0 for a in t.stream("arm_ack"))
    summary = t.stream("event")[-2] if t.stream("event")[-1]["kind"] != 6 else t.stream("event")[-1]
    assert b'"verdict"' in summary["text"]
    assert rc in (0, 1)        # timing on Windows is not representative; the verdict is not asserted here
    assert abs(mock.arm.q[5] - sim.MOCK_HOME_DEG[5]) < 1e-3


def test_ladder_dry_run(tmp_path):
    servo_ladder.run(["--dry-run"] + configs(tmp_path, period=0.1), profile=SHORT)
    assert "PushServoJ" in names(only_trial(tmp_path))


def test_ladder_refuses_above_confirmed_before_enabling(tmp_path, mock):
    fast = motion.LadderProfile(amplitude_deg=50.0, period_s=2.0, motion_s=2.0, taper_s=0.5, hold_s=0.3)
    with pytest.raises(motion.LimitRefused):
        servo_ladder.run(mock_args(mock) + configs(tmp_path, period=0.1), profile=fast)
    n = names(only_trial(tmp_path))
    assert not {"GrpEnable", "StartServo", "GrpDisable"} & set(n)     # the plan is checked before GrpEnable (S9)


def test_ladder_stops_on_fail(tmp_path, mock_factory):
    srv = mock_factory(fail_rules=[("PushServoJ", 5, 40014)])
    assert servo_ladder.run(mock_args(srv) + configs(tmp_path, period=0.1), profile=SHORT) == 1
    t = only_trial(tmp_path)
    acks = t.stream("arm_ack")
    assert len(acks) == 5 and acks[-1]["status"] == 1 and acks[-1]["fail_code"] == 40014
    n = names(t)
    assert n.count("PushServoJ") == 5 and "GrpStop" in n and n[-1] == "GrpDisable"


def test_ladder_flags_late_replies_and_decelerates(tmp_path, mock_factory):
    srv = mock_factory(late_reply_rules=[("PushServoJ", k, 0.25) for k in (4, 5, 6)])
    assert servo_ladder.run(mock_args(srv) + configs(tmp_path, period=0.1), profile=SHORT) == 1
    t = only_trial(tmp_path)
    assert sum(a["late"] for a in t.stream("arm_ack")) >= 2
    texts = [e["text"].decode() for e in t.stream("event")]
    assert any("streamed deceleration" in s for s in texts)
    assert names(t)[-1] == "GrpDisable"


def test_ladder_ctrl_c_decelerates_and_disables(tmp_path, mock):
    # Ctrl-C once the mock has seen the stream start, not on a fixed timer: on a slow file system (WSL on /mnt/c)
    # the start-up's git call can take over a second, and a Ctrl-C there escaped to pytest itself
    done = threading.Event()

    def ctrl_c_while_streaming():
        while not done.is_set():
            if mock.arm.stats["pushes"] >= 5:
                _thread.interrupt_main()
                return
            time.sleep(0.01)
    presser = threading.Thread(target=ctrl_c_while_streaming, daemon=True)
    presser.start()
    try:
        servo_ladder.run(mock_args(mock) + configs(tmp_path, period=0.1), profile=SHORT)
    finally:
        done.set()
        presser.join(timeout=5)
    t = only_trial(tmp_path)
    texts = [e["text"].decode() for e in t.stream("event")]
    assert any("operator stop (Ctrl-C" in s for s in texts)
    n = names(t)
    assert "GrpStop" in n and n[-1] == "GrpDisable"
    assert mock.arm.stats["dropped_extra_messages"] == 0      # one message at a time held through the interrupt


def test_pose_recorder_mock(tmp_path, mock):
    poses = tmp_path / "poses.yaml"
    poses.write_text("poses:\n  - [0.0, 0.0, 90.0, 0.0, 90.0, 5.0]\n  - [5.0, 0.0, 90.0, 0.0, 90.0, 5.0]\n", encoding="utf-8")
    ranges = "[[-360.0, 360.0], [-135.0, 135.0], [-153.0, 153.0], [-360.0, 360.0], [-180.0, 180.0], [-360.0, 360.0]]"
    assert pose_recorder.main(mock_args(mock) + configs(tmp_path, ranges=ranges) + ["--poses", str(poses)]) == 0
    t = only_trial(tmp_path)
    assert names(t).count("WayPoint") == 2 and names(t).count("ReadActPos") >= 6


def test_pose_recorder_refuses_without_ranges(tmp_path, mock):
    poses = tmp_path / "poses.yaml"
    poses.write_text("poses:\n  - [0.0, 0.0, 90.0, 0.0, 90.0, 5.0]\n", encoding="utf-8")
    with pytest.raises(link.Refused, match="joint_ranges_deg"):
        pose_recorder.main(mock_args(mock) + configs(tmp_path) + ["--poses", str(poses)])


def test_decel_respects_acceleration():
    out = motion.decel_to_rest([0, 0, 0, 0, 0, 1.0], [0, 0, 0, 0, 0, 0.0], 0.1, 100.0)
    xs = [0.0, 1.0] + [q[5] for q in out]
    acc = max(abs(xs[k + 1] - 2 * xs[k] + xs[k - 1]) / 0.01 for k in range(1, len(xs) - 1))
    assert acc <= 100.0 + 1e-9 and xs[-1] == xs[-2]


def test_owner_ladder_profile_is_within_the_starting_values():
    confirmed, _, _ = config.load()
    p = motion.LadderProfile()
    samples = motion.sample(p, sim.MOCK_HOME_DEG, confirmed["servo_period_s"])
    rep = motion.check_stream(samples, confirmed["servo_period_s"], confirmed, p)
    assert len(samples) == 121 and samples[0] == samples[-1] == sim.MOCK_HOME_DEG
    # the tapers add w'(t) sin terms: 4.11 deg/s and 7.74 deg/s^2, vs 3.14 and 1.97 for the plain sine
    assert abs(rep["peak_speed_deg_s"] - 4.114) < 0.01 and abs(rep["peak_acc_deg_s2"] - 7.740) < 0.01
