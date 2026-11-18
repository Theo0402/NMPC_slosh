"""The pre-hardware review's safety fixes S1-S4 and S8-S14 (2026-11-18): the ladder's step check and stop ramp,
GrpStop before GrpDisable, lenient motion replies, a failed log write, questions before enabling, the enable flag,
one message at a time after a time-out, the end of a move, termination signals, and the mode flags. The stream
tests run on a simulated clock, so they are exact and fast; the rest run against the mock."""
import json
import os
import signal
import subprocess
import sys
import time

import pytest

from analysis.logread import latest, read_trial
from tests.elfin.test_tools import ROOT, SHORT, configs, mock_args, names, only_trial
from tools.elfin_mock import sim
from tools.elfin_mock.server import MockServer
from tools.elfin_py import bringup_read, cli, link, motion, move_test, pose_recorder, protocol, servo_ladder, triallog

CONFIRMED = {"stream_joint_speed_deg_s": 20.0, "stream_joint_acc_deg_s2": 100.0}
Q0 = list(sim.MOCK_HOME_DEG)


# ------------------------------------------------------------------ the stream loop on a simulated clock
class FakeTime:
    def __init__(self):
        self.ns = 10 ** 12
        self.on_sleep = None

    def monotonic_ns(self):
        return self.ns

    def monotonic(self):
        return self.ns / 1e9

    def sleep(self, s):
        if self.on_sleep is not None:
            self.on_sleep()
        self.ns += max(0, int(round(s * 1e9)))


class FakeArm:
    """Answers at once, except that the reply to push n can be stalled; records what the arm received, when."""

    def __init__(self, clock, stall=None):
        self.clock, self.stall = clock, stall or {}
        self.pushes, self.t_push = [], []
        self.motion_may_run = False

    def cmd(self, name, *args):
        if name == "PushServoJ":
            self.pushes.append(list(args[1:]))
            self.t_push.append(self.clock.ns)
            self.clock.ns += int((0.001 + self.stall.get(len(self.pushes), 0.0)) * 1e9)
        return {}

    def drain(self):
        pass


class FakeLog:
    failed = None

    def __init__(self):
        self.events = []
        self.events_at_stop = []

    def event(self, kind, text):
        self.events.append(text if isinstance(text, str) else repr(text))

    def arm_cmd(self, *a):
        pass

    def arm_ack(self, *a):
        pass


class FakeStates:
    error = None

    def __init__(self, clock):
        self.clock = clock

    def latest(self):
        return self.clock.ns, {"state": {}, "q_rad": None}


def run_stream(monkeypatch, period, stall=None, stop_before_push=None):
    clock = FakeTime()
    monkeypatch.setattr(servo_ladder, "time", clock)
    arm, log = FakeArm(clock, stall), FakeLog()
    if stop_before_push is not None:           # one stop request, arriving while the loop waits for slot n
        def maybe_stop():
            if len(arm.pushes) == stop_before_push and not log.events_at_stop:
                log.events_at_stop.append(len(arm.pushes))
                signal.getsignal(signal.SIGINT)(signal.SIGINT, None)
        clock.on_sleep = maybe_stop
    profile = motion.LadderProfile()
    samples = motion.sample(profile, Q0, period)
    before = signal.getsignal(signal.SIGINT)
    out = servo_ladder.stream(arm, log, FakeStates(clock), samples, period, 0.02, Q0, profile, dict(CONFIRMED))
    assert signal.getsignal(signal.SIGINT) is before          # the stream's stop flag handler is removed again
    return out, arm, log, samples


def received_peaks(arm, period):
    return motion.peaks_samples([Q0, Q0] + arm.pushes, period)


@pytest.mark.parametrize("period", [0.04, 0.02, 0.01])
@pytest.mark.parametrize("t_stop", [3.0, 7.3, 12.0])
def test_a_stop_during_the_wait_decelerates_from_what_was_sent(monkeypatch, period, t_stop):
    # review S2: the ramp used to start from the sample that was never sent (up to ~500 deg/s^2 at 0.01 s)
    out, arm, log, samples = run_stream(monkeypatch, period, stop_before_push=int(t_stop / period))
    assert out["decelerated"] and out["abort"] == servo_ladder.OPERATOR_STOP
    v, a = received_peaks(arm, period)
    assert v <= 20.0 and a <= 100.0 * (1 + 1e-6), (v, a)
    assert arm.pushes[-1] == arm.pushes[-2] and len(arm.pushes) < len(samples)


@pytest.mark.parametrize("period", [0.02, 0.01])
def test_a_late_push_that_would_jump_too_far_decelerates_instead(monkeypatch, period):
    # review S1: a 100 ms stall in mid-motion used to send a step of ten samples at once
    k = int(7.3 / period)
    out, arm, log, samples = run_stream(monkeypatch, period, stall={k: 0.1})
    assert out["decelerated"] and out["abort"].startswith("late push: skipping")
    v, a = received_peaks(arm, period)
    assert v <= 20.0 and a <= 100.0 * (1 + 1e-6), (v, a)
    gaps = [b - a for a, b in zip(arm.t_push[k:], arm.t_push[k + 1:])]
    assert min(gaps) >= period * 1e9 * 0.999       # the deceleration is never skipped or bunched: the schedule shifts
    assert out["schedule_shifts"] >= 1


def test_a_late_push_during_a_hold_still_skips(monkeypatch):
    out, arm, log, samples = run_stream(monkeypatch, 0.01, stall={50: 0.1})      # t = 0.5 s, inside the 2 s hold
    assert not out["decelerated"] and out["abort"] is None
    assert out["skipped"] >= 9 and len(arm.pushes) == len(samples) - out["skipped"]
    v, a = received_peaks(arm, 0.01)
    assert v <= 20.0 and a <= 100.0 * (1 + 1e-6)


def test_the_mock_receives_nothing_above_the_confirmed_limits_after_late_replies(tmp_path, mock_factory):
    srv = mock_factory(late_reply_rules=[("PushServoJ", k, 0.25) for k in (4, 5, 6, 12)])
    servo_ladder.run(mock_args(srv) + configs(tmp_path, period=0.1), profile=SHORT)
    pushes = [[float(x) for x in m.split(",")[2:8]] for _, m, _ in srv.arm.log if m.startswith("PushServoJ")]
    v, a = motion.peaks_samples([Q0, Q0] + pushes, 0.1)
    assert v <= 20.0 and a <= 100.0 * (1 + 1e-6), (v, a)


# ------------------------------------------------------------------ S3: GrpStop before GrpDisable on any exit mid-move
class StallDuringMove(sim.MockArm):
    stalled = False

    def reply_extra_delay(self, text):
        if text.startswith("ReadRobotState") and self.motion is not None and not self.stalled:
            self.stalled = True
            return 2.5                                    # the client gives up after 2 s
        return super().reply_extra_delay(text)


def serve(arm):
    return MockServer(arm, cmd_port=0, state_port=0).start()


def test_a_reply_timeout_during_a_move_sends_grpstop_then_grpdisable(tmp_path):
    srv = serve(StallDuringMove())
    try:
        with pytest.raises(TimeoutError):
            move_test.main(mock_args(srv) + configs(tmp_path))
        n = names(only_trial(tmp_path))
        assert n.index("WayPoint") < n.index("GrpStop") < len(n) - 1 and n[-1] == "GrpDisable"
        assert srv.arm.stats["dropped_extra_messages"] == 0      # the late reply was read before GrpStop
        assert not srv.arm.enabled
    finally:
        srv.stop()


def test_a_refused_move_still_ends_with_grpstop_then_grpdisable(tmp_path, mock_factory):
    srv = mock_factory(fail_rules=[("WayPoint", 1, 990004)])
    with pytest.raises(link.ElfinError):
        move_test.main(mock_args(srv) + configs(tmp_path))
    n = names(only_trial(tmp_path))
    assert n.index("WayPoint") < n.index("GrpStop") < n.index("GrpDisable") == len(n) - 1


# ------------------------------------------------------------------ S4: an action's OK is accepted whatever follows
def test_action_replies_accept_fields_after_ok_and_reads_stay_strict():
    assert protocol.parse_reply("WayPoint", ["7"]) == {} and protocol.parse_reply("GrpDisable", []) == {}
    assert protocol.is_action("PushServoJ") and not protocol.is_action("ReadActPos")
    with pytest.raises(protocol.ProtocolError):
        protocol.parse_reply("ReadRobotState", ["0"] * 12)


class ExtraFields(sim.MockArm):
    def _cmd_WayPoint(self, name, args, now):
        return super()._cmd_WayPoint(name, args, now).replace("WayPoint,OK,;", "WayPoint,OK,7,;")


def test_a_waypoint_reply_with_extra_fields_does_not_end_the_run(tmp_path):
    srv = serve(ExtraFields())
    try:
        assert move_test.main(mock_args(srv) + configs(tmp_path)) == 0
        t = only_trial(tmp_path)
        texts = [e["text"].decode() for e in t.stream("event")]
        assert sum("WayPoint replied OK with 1 field(s) after OK: ['7']" in s for s in texts) == 1   # once per command
        assert names(t).count("WayPoint") == 2 and "GrpStop" not in names(t)
    finally:
        srv.stop()


# ------------------------------------------------------------------ S8: a failed log write is a stop
def failing_writer(monkeypatch, when):
    W = latest().Writer
    real = W.write

    def write(self, stream, **fields):
        if when(stream, fields):
            raise OSError(28, "No space left on device")
        return real(self, stream, **fields)
    monkeypatch.setattr(W, "write", write)


def test_a_failed_log_write_stops_the_move_and_still_disables(tmp_path, mock, monkeypatch):
    failing_writer(monkeypatch, lambda s, f: s == "wire_tx" and f["data"].startswith(b"WayPoint"))
    with pytest.raises(cli.LogIncomplete):
        move_test.main(mock_args(mock) + configs(tmp_path))
    got = [m.split(",", 1)[0] for _, m, _ in mock.arm.log]       # what the controller received, logged or not
    assert got[-1] == "GrpDisable" and "GrpStop" in got and not mock.arm.enabled


def test_a_failed_log_write_mid_stream_decelerates_unlogged(tmp_path, mock, monkeypatch):
    count = {"n": 0}

    def when(stream, fields):
        count["n"] += stream == "arm_cmd"
        return stream == "arm_cmd" and count["n"] == 12
    failing_writer(monkeypatch, when)
    with pytest.raises(cli.LogIncomplete):
        servo_ladder.run(mock_args(mock) + configs(tmp_path, period=0.1), profile=SHORT)
    got = [m for _, m, _ in mock.arm.log]
    pushes = [[float(x) for x in m.split(",")[2:8]] for m in got if m.startswith("PushServoJ")]
    planned = len(motion.sample(SHORT, Q0, 0.1))
    assert 12 < len(pushes) < planned + 10 and got[-1].startswith("GrpDisable")
    v, a = motion.peaks_samples([Q0, Q0] + pushes, 0.1)
    assert a <= 100.0 * (1 + 1e-6)


def test_the_trial_log_never_raises_and_reports_once(tmp_path):
    log = triallog.TrialLog("t", "dry-run", "scratch", {}, data_dir=tmp_path)
    seen = []
    log.on_failure = seen.append
    log.w.f.close()                                  # every write now fails
    log.event(4, "x")
    log.wire_tx(10003, b"GrpDisable,0,;", 1)
    log.close()
    assert isinstance(log.failed, ValueError) and len(seen) == 1


# ------------------------------------------------------------------ S9: no question waits with the arm enabled
class MovesOnEnable(sim.MockArm):
    def _cmd_GrpEnable(self, name, args, now):
        self.q[5] += 2.0                             # more than ENABLE_MOVE_TOL_DEG
        return super()._cmd_GrpEnable(name, args, now)


def test_the_ladder_refuses_if_the_joints_moved_between_plan_and_enable(tmp_path):
    srv = serve(MovesOnEnable())
    try:
        with pytest.raises(link.Refused, match="moved"):
            servo_ladder.run(mock_args(srv) + configs(tmp_path, period=0.1), profile=SHORT)
        n = names(only_trial(tmp_path))
        assert "GrpEnable" in n and "StartServo" not in n and n[-1] == "GrpDisable"
    finally:
        srv.stop()


def test_pose_recorder_asks_pose_1_before_enabling_and_later_ones_with_a_timeout(tmp_path, mock, monkeypatch):
    calls = []

    def make_ask(args, log, chan=None):
        def ask(question, timeout=None):
            calls.append((question, timeout))
            log.event(4, f"prompt: {question}")
            return not question.startswith("Move to pose 2")       # as if nobody answered in time
        return ask
    monkeypatch.setattr(cli, "make_ask", make_ask)
    poses = tmp_path / "poses.yaml"
    poses.write_text("poses:\n  - [0.0, 0.0, 90.0, 0.0, 90.0, 5.0]\n  - [5.0, 0.0, 90.0, 0.0, 90.0, 5.0]\n", encoding="utf-8")
    ranges = "[[-360.0, 360.0], [-135.0, 135.0], [-153.0, 153.0], [-360.0, 360.0], [-180.0, 180.0], [-360.0, 360.0]]"
    with pytest.raises(link.Refused, match="before pose 2"):
        pose_recorder.main(mock_args(mock) + configs(tmp_path, ranges=ranges) + ["--poses", str(poses)])
    assert [t for q, t in calls if q.startswith("Move to pose")] == [None, pose_recorder.QUESTION_TIMEOUT_S]
    t = only_trial(tmp_path)
    order = list(enumerate(t.records))
    pose1 = [i for i, (s, r) in order if s == "event" and r["text"].decode().startswith("prompt: Move to pose 1")]
    enable = [i for i, (s, r) in order if s == "wire_tx" and r["data"].startswith(b"GrpEnable")]
    assert pose1[0] < enable[0]
    n = names(t)
    assert n.count("WayPoint") == 1 and n[-1] == "GrpDisable"


# ------------------------------------------------------------------ S10: the enable flag and the exit sequence
def test_a_lost_grpenable_reply_still_ends_in_grpdisable(tmp_path, mock_factory):
    srv = mock_factory(late_reply_rules=[("GrpEnable", 1, 2.5)])
    with pytest.raises(TimeoutError):
        move_test.main(mock_args(srv) + configs(tmp_path))
    n = names(only_trial(tmp_path))
    assert n[-1] == "GrpDisable" and "WayPoint" not in n
    assert srv.arm.stats["dropped_extra_messages"] == 0 and not srv.arm.enabled


def test_the_exit_sequence_goes_on_when_reading_the_reply_in_flight_fails(tmp_path):
    log = triallog.TrialLog("t", "dry-run", "scratch", {}, data_dir=tmp_path)
    arm = link.Arm("dry-run", log, echo=False)
    arm.enabled_by_us = arm.motion_may_run = True

    def broken(*a, **k):
        raise OSError("socket gone")
    arm.drain = broken
    arm.exit_sequence()
    log.close()
    sent = [r["data"].decode().split(",", 1)[0] for r in read_trial(log.path).stream("wire_tx")]
    assert sent[0] == "GrpStop" and sent[-1] == "GrpDisable"


# ------------------------------------------------------------------ S11: one message at a time after a time-out
def summary(tmp_path):
    return json.loads(next((tmp_path / "data" / "trials").glob("*.summary.json")).read_text(encoding="utf-8"))


def test_a_push_that_timed_out_has_its_reply_read_before_the_next_message(tmp_path, mock_factory):
    srv = mock_factory(late_reply_rules=[("PushServoJ", 8, 2.6)])       # the client gives up after 2 s
    assert servo_ladder.run(mock_args(srv) + configs(tmp_path, period=0.1), profile=SHORT) == 1
    assert srv.arm.stats["dropped_extra_messages"] == 0                  # nothing went out over the late reply
    after = summary(tmp_path)["after_stream"]
    assert after["owed_reply"] == {"owed": 1, "read": True}
    assert "error" not in after["observed"][0] and after["grpstop"] == "OK"
    assert names(only_trial(tmp_path))[-1] == "GrpDisable"


def test_out_of_step_only_the_stop_commands_go_out_and_replies_match_by_name(tmp_path, mock_factory):
    srv = mock_factory(late_reply_rules=[("PushServoJ", 8, 5.0)])       # later than the time-out and the wait after it
    assert servo_ladder.run(mock_args(srv) + configs(tmp_path, period=0.1), profile=SHORT) == 1
    t = only_trial(tmp_path)
    texts = [e["text"].decode() for e in t.stream("event")]
    assert any(s.startswith("OUT OF STEP") for s in texts)
    assert any(s.startswith("out of step: dropped a late reply while waiting for GrpStop: b'PushServoJ,OK,'")
               for s in texts)
    after = summary(tmp_path)["after_stream"]
    assert after["owed_reply"] == {"owed": 1, "read": False}
    assert "OutOfStep" in after["observed"][0]["error"]                  # the watching is skipped
    # the first GrpStop reached the controller while it was still busy and was dropped (PROTO p.1): sent again
    assert after["grpstop"] == "OK at the second try" and srv.arm.stats["dropped_extra_messages"] == 1
    assert names(t)[-1] == "GrpDisable" and not srv.arm.enabled          # GrpDisable reached the controller


# ------------------------------------------------------------------ S12: a move counts as done only once it is done
def test_a_late_moving_flag_does_not_end_the_wait_early(tmp_path, mock_factory):
    srv = mock_factory(motion_start_delay_s=0.8)       # nMovingState stays 0 for longer than the old 0.5 s grace
    assert move_test.main(mock_args(srv) + configs(tmp_path)) == 0     # before: the second WayPoint failed (busy)
    assert abs(srv.arm.q[5] - (sim.MOCK_HOME_DEG[5] + 10.0)) < 1e-6
    assert abs(srv.arm.pose[2] - (sim.MOCK_HOME_POSE[2] + 20.0)) < 1e-6
    assert srv.arm.counts.get("WayPoint") == 2


# ------------------------------------------------------------------ S13: termination signals and an ignored SIGINT
def test_an_ignored_sigint_is_restored_and_sigterm_acts_like_ctrl_c():
    before = signal.signal(signal.SIGINT, signal.SIG_IGN)          # as after a background start from a script
    try:
        old = cli.install_stop_signals()
        try:
            assert signal.getsignal(signal.SIGINT) is signal.default_int_handler
            assert signal.getsignal(signal.SIGTERM) is cli.like_ctrl_c
            with pytest.raises(KeyboardInterrupt):
                cli.like_ctrl_c(signal.SIGTERM, None)
            assert cli.stop_reason() == "SIGTERM"
            signal.signal(signal.SIGINT, signal.SIG_IGN)            # as while the exit sequence runs
            cli.like_ctrl_c(signal.SIGTERM, None)                   # then it changes nothing
        finally:
            cli.restore_signals(old)
        assert signal.getsignal(signal.SIGINT) is signal.SIG_IGN
    finally:
        signal.signal(signal.SIGINT, before)
        cli.received["signal"] = None


@pytest.mark.skipif(os.name != "posix", reason="SIGTERM comes from the OS on POSIX (the Linux machine)")
def test_sigterm_mid_stream_decelerates_and_disables(tmp_path, mock):
    argv = [sys.executable, "-u", "-m", "tools.elfin_py.servo_ladder"] + mock_args(mock) + configs(tmp_path, period=0.1)
    with open(tmp_path / "console.txt", "w") as out:
        p = subprocess.Popen(argv, cwd=ROOT, stdin=subprocess.DEVNULL, stdout=out, stderr=subprocess.STDOUT,
                             env=dict(os.environ, PYTHONUNBUFFERED="1"))
        try:
            t_end = time.monotonic() + 30
            while time.monotonic() < t_end:                         # wait until it streams
                files = sorted((tmp_path / "data" / "trials").glob("*.bin"))
                if files and len(read_trial(files[0]).stream("arm_cmd")) >= 15:
                    break
                time.sleep(0.1)
            p.send_signal(signal.SIGTERM)
            p.wait(timeout=60)
        finally:
            if p.poll() is None:
                p.kill()
    t = only_trial(tmp_path)
    texts = [e["text"].decode() for e in t.stream("event")]
    assert any(s.startswith("streamed deceleration: SIGTERM received") for s in texts), texts[-5:]
    n = names(t)
    assert n[-1] == "GrpDisable" and n.count("PushServoJ") < len(motion.sample(motion.LadderProfile(), Q0, 0.1))


# ------------------------------------------------------------------ S14: the mode flags
def test_mock_mode_talks_to_loopback_only(tmp_path, mock):
    with pytest.raises(SystemExit, match="loopback"):
        bringup_read.main(["--mock", "--ip", "192.168.0.10"] + configs(tmp_path))
    assert not (tmp_path / "data" / "trials").exists()              # refused before anything was opened
    assert bringup_read.main(mock_args(mock) + ["--ip", "127.0.0.1"] + configs(tmp_path)) == 0


def test_arm_mode_takes_only_the_owners_config_files(tmp_path):
    with pytest.raises(SystemExit, match="refused with --arm"):
        bringup_read.main(["--arm"] + [a for a in configs(tmp_path) if a != "--assume-yes"])
    assert not (tmp_path / "data" / "trials").exists()


# ------------------------------------------------------------------ S17: watching the arm while streaming
class PolledArm(FakeArm):
    """FakeArm that also answers ReadRobotState; from poll n on it reports the e-stop."""

    def __init__(self, clock, estop_from_poll=None):
        super().__init__(clock)
        self.polls, self.estop_from_poll = 0, estop_from_poll

    def state(self):
        self.polls += 1
        self.clock.ns += 1_000_000                       # 1 ms reply
        estop = self.estop_from_poll is not None and self.polls >= self.estop_from_poll
        return {"nEmergencyStop": 1 if estop else 0, "nSaftyGuard": 0, "nErrorState": 0, "nErrorCode": 0,
                "nErrorAxis": 0, "nEnableState": 1}


def run_polled_stream(monkeypatch, period, poll_s, **arm_kw):
    clock = FakeTime()
    monkeypatch.setattr(servo_ladder, "time", clock)
    arm, log = PolledArm(clock, **arm_kw), FakeLog()
    profile = motion.LadderProfile()
    samples = motion.sample(profile, Q0, period)
    out = servo_ladder.stream(arm, log, FakeStates(clock), samples, period, 0.02, Q0, profile, dict(CONFIRMED),
                              state_poll_s=poll_s)
    return out, arm, samples


def test_readrobotstate_between_pushes_stops_the_stream_on_an_estop(monkeypatch):
    out, arm, samples = run_polled_stream(monkeypatch, 0.02, 0.1, estop_from_poll=40)
    assert out["stopped_on_reply"] and out["abort"].endswith("e-stop or light curtain active")
    assert out["state_polls"]["sent"] == 40 and not out["decelerated"]
    assert 0.1 * 39 <= len(arm.pushes) * 0.02 <= 0.1 * 41 + 0.04      # about every 0.1 s of stream


def test_polls_never_take_a_slot_they_do_not_fit(monkeypatch):
    out, arm, samples = run_polled_stream(monkeypatch, 0.002, 0.1)  # a 2 ms slot: 1 ms push + 1 ms poll do not fit 2x
    assert out["state_polls"]["sent"] == 0 and out["state_polls"]["skipped_no_time"] > 0
    assert out["late"] == 0 and len(arm.pushes) == len(samples)


class DisabledStates(FakeStates):
    def latest(self):
        enabled = 0 if self.clock.ns > 10 ** 12 + 5 * 10 ** 9 else 1  # the state stream reports a disable after 5 s
        return self.clock.ns, {"state": {"robotEnabled": enabled}, "q_rad": None}


def test_the_state_streams_enable_flag_starts_the_deceleration(monkeypatch):
    clock = FakeTime()
    monkeypatch.setattr(servo_ladder, "time", clock)
    arm, log = FakeArm(clock), FakeLog()
    profile = motion.LadderProfile()
    samples = motion.sample(profile, Q0, 0.02)
    out = servo_ladder.stream(arm, log, DisabledStates(clock), samples, 0.02, 0.02, Q0, profile, dict(CONFIRMED))
    assert out["decelerated"] and out["abort"] == "state stream reports robotEnabled = 0"


def test_the_ladder_polls_between_pushes_on_the_mock_one_message_at_a_time(tmp_path, mock):
    assert servo_ladder.run(mock_args(mock) + configs(tmp_path, period=0.1), profile=SHORT) in (0, 1)
    s = summary(tmp_path)
    assert s["state_polls"]["sent"] >= 10 and s["state_polls"]["median_ms"] is not None
    assert "StateAndError.robotEnabled" in s["datasheet_keys"] or any("robotEnabled" in k for k in s["datasheet_keys"])
    n = names(only_trial(tmp_path))
    first, last = n.index("PushServoJ"), len(n) - 1 - n[::-1].index("PushServoJ")
    assert "ReadRobotState" in n[first:last] and mock.arm.stats["dropped_extra_messages"] == 0


def test_the_poll_can_be_turned_off(tmp_path, mock):
    servo_ladder.run(mock_args(mock) + configs(tmp_path, period=0.1) + ["--state-poll-s", "0"], profile=SHORT)
    n = names(only_trial(tmp_path))
    first, last = n.index("PushServoJ"), len(n) - 1 - n[::-1].index("PushServoJ")
    assert "ReadRobotState" not in n[first:last] and summary(tmp_path)["state_polls"]["sent"] == 0


# ------------------------------------------------------------------ S20: joint speeds during WayPoint moves
class TooFast(sim.MockArm):
    """A controller whose joint moves run 1.5 times faster than dVelocity."""

    def _cmd_WayPoint(self, name, args, now):
        reply = super()._cmd_WayPoint(name, args, now)
        m = self.motion
        if m is not None and m["kind"] == "joint":
            m["v"] *= 1.5
            m["T"] = sim._trapezoid_duration(m["dist"], m["v"], m["a"])
        return reply


def test_a_joint_faster_than_confirmed_stops_the_move(tmp_path):
    srv = serve(TooFast())
    try:
        with pytest.raises(link.JointSpeedExceeded, match="above the confirmed joint speed 20.0 deg/s"):
            move_test.main(mock_args(srv) + configs(tmp_path))
        n = names(only_trial(tmp_path))
        assert n.index("WayPoint") < n.index("GrpStop") < len(n) - 1 and n[-1] == "GrpDisable"
        assert n.count("WayPoint") == 1
    finally:
        srv.stop()


def test_every_move_logs_its_peak_joint_speed(tmp_path, mock):
    assert move_test.main(mock_args(mock) + configs(tmp_path)) == 0
    peaks = [json.loads(e["text"]) for e in only_trial(tmp_path).stream("event")
             if e["text"].startswith(b'{"confirmed_joint_speed_deg_s"')]
    assert len(peaks) == 2                                   # the joint move and the linear move
    assert 15.0 < max(peaks[0]["move_peak_joint_speed_deg_s"]) <= 20.0 * 1.1


# ------------------------------------------------------------------ W8: the ladder's records
def test_a_fail_code_is_the_first_field_after_fail_not_its_digits():
    exc = link._as_failed("PushServoJ", link.ElfinError("PushServoJ failed, error code ['40014', '3']"))
    assert exc.code == 40014 and exc.fields == ["40014", "3"]                # the old reading gave 400143
    assert link._as_failed("PushServoJ", link.ElfinError("unexpected reply to PushServoJ: b'X,OK,'")) is None


class SlowLog:
    """Every wire record takes 30 ms to write, as on a slow disk."""
    failed = None

    def wire_tx(self, *a):
        time.sleep(0.03)

    def wire_rx(self, *a):
        time.sleep(0.03)


def test_reply_times_leave_out_the_logs_own_writes():
    s = link.LoggedSocket(link.DryRunSocket(sim.MockArm()), SlowLog())
    t0 = time.monotonic_ns()
    s.sendall(b"ReadRobotState,0,;")
    while s.outstanding > 0:
        s.recv(4096)
    assert time.monotonic_ns() - t0 >= 55e6 and s.reply_ns() < 10e6         # before: the 60 ms of writes counted


# ------------------------------------------------------------------ W10: the arm block
def test_every_enabling_tool_logs_the_arm_block_before_grpenable(tmp_path, mock):
    servo_ladder.run(mock_args(mock) + configs(tmp_path, period=0.1), profile=SHORT)
    order = list(enumerate(only_trial(tmp_path).records))
    blocks = [(i, json.loads(r["text"])) for i, (s, r) in order if s == "event" and r["kind"] == triallog.EV_ARM_BLOCK]
    enable = [i for i, (s, r) in order if s == "wire_tx" and r["data"].startswith(b"GrpEnable")]
    assert len(blocks) == 1 and blocks[0][0] < enable[0]
    b = blocks[0][1]
    assert b["model"] == "MOCK" and b["servo_period_s"] == 0.1 and b["servo_lookahead_s"] == 0.02
    assert b["readback"]["joint_max_vel_deg_s"] and b["mounting_confirmed"]["tcp_name"] == "TCP"
    assert b["controller_version"].startswith("[MISSING]")                  # e03pro.yaml has none entered yet
