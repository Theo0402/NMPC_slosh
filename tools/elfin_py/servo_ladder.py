"""Servo ladder (example 3, extended): stream a slow J6 sine with StartServo + PushServoJ.

    python -m tools.elfin_py.servo_ladder --dry-run | --mock | --arm

One period per run, at confirmed.yaml's servo_period_s and servo_lookahead_s; the tool never
prompts for a period (owner, week 1). The owner lowers the period in confirmed.yaml between runs:
    T_s = 0.2 -> 0.1 -> 0.05 -> 0.04 -> 0.03 -> 0.02 -> 0.015 -> 0.01 s, lookahead 0.02 s throughout.
Run it on the arm from the Linux machine only.

Motion (owner, week 1): J6 = q0 + 5 deg * w(t) * sin(2 pi t / 10 s) for 20 s, w a Tukey window
with 2 s cosine tapers, plus 2 s of streamed holds at q0 before and after. The planned peak
speed and acceleration are computed numerically and checked against confirmed.yaml first.

The plan, its check against confirmed.yaml and the question "Start streaming?" all come before
GrpEnable, so no question waits with the arm enabled; after enabling, the joints are read again and
the run is refused if they moved more than ENABLE_MOVE_TOL_DEG (pre-hardware review S9).

Pushes go out at fixed slots on time.monotonic_ns(). One message at a time (PROTO p.1): each push
waits for its reply. A push whose slot had passed when its turn came is late; slots missed
entirely are skipped, never made up (as elfin_io will do, codebase plan §6), but only if the jump
stays within confirmed.yaml's streamed speed and acceleration. Every push is checked that way, from
the last two samples actually sent; a push that would exceed them starts the deceleration instead
(review S1; owner's go-ahead 2026-11-18: skip while within the limits, otherwise decelerate). How the
controller treats late pushes, and whether its limits apply while streaming, is not in the knowledge
files [MISSING] (elfin_commands.md, PROTO p.172-175).

Stops (codebase plan §3.8 items 6 and 7; owner, week 1):
- a push answered Fail, or no reply: streaming stops at once;
- ReadRobotState between pushes showing the e-stop or light curtain, an error or the arm disabled,
  or not answering: streaming stops at once. It goes out about every 0.1 s (--state-poll-s; 0 = off),
  only when the slot has twice the slowest poll reply left, because the Datasheet's documented keys
  have no e-stop or light-curtain flag (elfin_commands.md, PROTO p.185-188; whether ReadRobotState
  is answered between pushes, and at what cost, is the test of §3.8 item 7 [UNTESTED]). The summary
  lists the Datasheet keys that arrived, for week 1's check;
- more than 1 % of the planned pushes late, a stale or missing state stream, the state stream
  reporting an error code, robotEnabled = 0 or robotPaused = 1, a joint other than J6 moving, J6
  leaving its planned band, a push above the confirmed limits, a failed trial-log write, or a stop
  request (Ctrl-C, the workbench's stop button or closed channel, SIGTERM or SIGHUP): a streamed
  deceleration to rest, then stop.
While streaming, a stop request only sets a flag; the loop acts on it between pushes, so a push is
never cut in half and the deceleration starts from the last two samples the arm received (review
S2). Its samples are never skipped: if they fall behind, the schedule shifts by the delay instead.
After the stream: watch the arm with ReadRobotState for 1 s with no pushes, send GrpStop
[UNTESTED], wait for nMovingState = 0, and record what each showed. GrpDisable runs on every exit.

Then the stop rules are evaluated: any Fail; > 1 % late pushes; median reply time > T_s/2;
commanded vs state-stream J6 that looks wrong; audible or visible roughness (asked, on the arm).
"""
import json
import math
import signal
import socket
import statistics
import sys
import threading
import time

from tools.elfin_py import cli, link, motion, protocol, triallog
from tools.elfin_py.elfin_min import ElfinError

FIRST_RUN = ("Reply time per push (median, maximum) and late count; commanded vs state-stream J6; any Fail code\n"
             "(the manual's failure example shows 40014, meaning not given; PROTO p.173); what the arm does when\n"
             "pushes stop; what GrpStop does. Stop and report on any Fail, > 1 % late pushes, a median reply time\n"
             "above T_s/2, J6 that looks wrong, or audible or visible roughness.\n"
             "ReadRobotState also goes out between pushes, about every 0.1 s [UNTESTED]: its reply times and how\n"
             "often the slot had no time for it are in the summary (codebase plan §3.8 item 7).\n"
             "Hit the e-stop if any joint other than J6 moves, or J6 moves more than about 5 deg from its start.")

GUARD_OTHER_JOINTS_DEG = 0.5    # any other joint further than this from q0: decelerate
GUARD_J6_MARGIN_DEG = 2.0       # J6 outside q0 +- (amplitude + this): decelerate
STATE_STALE_S = 0.5             # no Datasheet packet for this long: decelerate (20 Hz nominal)
LATE_LIMIT = 0.01               # > 1 % of planned pushes late: stop (owner)
OBSERVE_S = 1.0                 # watch after the last push, before GrpStop
J6_RESIDUAL_FLAG_DEG = 0.5      # delay-compensated J6 error above this is flagged as "looks wrong"
ENABLE_MOVE_TOL_DEG = 1.0       # joints moved more than this between the plan and the enable: refuse [proposal]
STATE_POLL_S = 0.1              # ReadRobotState between pushes about this often; 0 = off [proposal]
POLL_MARGIN = 2.0               # ... only if the slot has this many times the slowest poll reply left [proposal]
DEG = math.pi / 180.0
OPERATOR_STOP = "operator stop (Ctrl-C, or the workbench: stop button or channel closed)"


def _reply_ns(arm, t_before, t_after):
    """A reply time on the wire: from the message's send to its reply's end, less the trial log's own writes in
    between (link.LoggedSocket.reply_ns; pre-hardware review W8). Falls back to the time around the whole call
    where the socket has no such record for this call (a test's stand-in arm)."""
    sock = getattr(arm, "sock", None)
    tx = getattr(sock, "last_tx_ns", None)
    r = sock.reply_ns() if tx is not None and hasattr(sock, "reply_ns") and t_before <= tx <= t_after else None
    return r if r is not None else t_after - t_before


def stream(arm, log, states, samples, period, lookahead, q0, profile, confirmed, state_poll_s=0.0):
    """Push the samples; return a dict of what happened. With state_poll_s > 0, ReadRobotState goes out between
    pushes about every state_poll_s, when the slot has time for it (codebase plan §3.8 item 7; review S17)."""
    T_ns = int(round(period * 1e9))
    vmax, amax = confirmed["stream_joint_speed_deg_s"], confirmed["stream_joint_acc_deg_s2"]
    n_planned = len(samples)
    seq = list(samples)
    poll_ms, polls_skipped, last_poll_ns = [], 0, 0

    def poll_state():
        """ReadRobotState between pushes (PROTO p.61-62): what the Datasheet's documented keys lack, the e-stop
        and light-curtain flags (PROTO p.185-188). Returns why streaming must stop at once, or None."""
        t_a = time.monotonic_ns()
        try:
            s = arm.state()
        except (ElfinError, OSError, protocol.ProtocolError) as exc:
            return f"ReadRobotState between pushes failed: {exc!r}"
        poll_ms.append(_reply_ns(arm, t_a, time.monotonic_ns()) / 1e6)
        if s["nEmergencyStop"] or s["nSaftyGuard"]:
            return "ReadRobotState between pushes: e-stop or light curtain active"
        if s["nErrorState"]:
            return f"ReadRobotState between pushes: error code {s['nErrorCode']} on axis {s['nErrorAxis']}"
        if not s["nEnableState"]:
            return "ReadRobotState between pushes: the arm is not enabled"
        return None
    acks, cmd_series = [], []
    late = skipped = shifts = 0
    abort = None
    decelerating = False
    stop_now = False
    push_id = 0
    sent = [list(q0), list(q0)]       # the last two samples the arm received; at rest at q0 before the first push
    stop = {"reason": None}
    band = (q0[5] - profile.amplitude_deg - GUARD_J6_MARGIN_DEG, q0[5] + profile.amplitude_deg + GUARD_J6_MARGIN_DEG)

    def on_stop_request(signum, frame):
        # Ctrl-C, the workbench's stop (channel.interrupt_main), a failed log write (cli.session) or SIGTERM/SIGHUP
        # (cli.like_ctrl_c): a flag only
        if stop["reason"] is None:
            if log.failed is not None:
                stop["reason"] = f"trial log write failed ({log.failed!r})"
            elif signum != signal.SIGINT:
                stop["reason"] = f"{signal.Signals(signum).name} received (the terminal closed, or the process was told to end)"
            else:
                stop["reason"] = OPERATOR_STOP

    def start_decel(i, reason):
        """Replace seq[i:] (nothing from index i on has been sent) with a deceleration to rest from the last two
        samples sent, at the confirmed streamed acceleration (codebase plan §3.8 item 6; review S2)."""
        nonlocal seq, decelerating, abort
        abort = reason
        decelerating = True
        ramp = motion.decel_to_rest(sent[1], sent[0], period, amax)
        seq = seq[:i] + ramp
        log.event(triallog.EV_PHASE, f"streamed deceleration: {reason}")
        print(f"DECELERATING: {reason}", file=sys.stderr, flush=True)
        _, a = motion.peaks_samples([sent[0], sent[1]] + ramp, period)
        if a > amax * (1 + 1e-9):      # not possible by construction (motion.decel_to_rest); reported if it happens
            msg = (f"the deceleration's peak acceleration {a:.3f} deg/s^2 is above the confirmed {amax}: sent anyway, "
                   "since a stream that stops without one is the worse case")
            print(msg, file=sys.stderr, flush=True)
            log.event(triallog.EV_ERROR, msg)

    def jump_ok(i, j):
        """Skipping to slot j sends seq[j] right after the last sample sent: check that push and the next."""
        v, a = motion.step_peaks(sent[0], sent[1], seq[j], period)
        if not motion.within(v, a, vmax, amax):
            return False
        if j + 1 < len(seq):
            v, a = motion.step_peaks(sent[1], seq[j], seq[j + 1], period)
            return motion.within(v, a, vmax, amax)
        return True

    arm.cmd("StartServo", 0, period, lookahead)
    log.event(triallog.EV_PHASE, f"StartServo {period} s / {lookahead} s; {n_planned} pushes planned")
    old_handler = None
    if threading.current_thread() is threading.main_thread():
        old_handler = signal.signal(signal.SIGINT, on_stop_request)
    try:
        t0 = time.monotonic_ns() + T_ns
        i = 0
        while i < len(seq):
            if stop["reason"] is not None and not decelerating:
                start_decel(i, stop["reason"])
            slot = t0 + i * T_ns
            now = time.monotonic_ns()
            is_late = now > slot
            if is_late:
                late += 1
                j = min((now - t0) // T_ns, len(seq) - 1)
                if j > i:
                    if not decelerating and jump_ok(i, j):
                        skipped += j - i          # missed slots are skipped, never made up (codebase plan §6)
                        i = j
                    else:
                        if not decelerating:
                            start_decel(i, f"late push: skipping {j - i} slot(s) would exceed the confirmed "
                                           f"{vmax} deg/s, {amax} deg/s^2")
                        t0 = now - i * T_ns       # the deceleration is never skipped: shift the schedule instead
                        shifts += 1
                    slot = t0 + i * T_ns
            else:
                while True:                       # a stop request during the wait only sets the flag
                    rest = slot - time.monotonic_ns()
                    if rest <= 0:
                        break
                    time.sleep(rest / 1e9)
                if stop["reason"] is not None and not decelerating:
                    start_decel(i, stop["reason"])
            if not decelerating:                  # every push, against confirmed.yaml (review S1)
                v, a = motion.step_peaks(sent[0], sent[1], seq[i], period)
                if not motion.within(v, a, vmax, amax):
                    start_decel(i, f"push {i} would mean {v:.3f} deg/s, {a:.3f} deg/s^2, above the confirmed "
                                   f"{vmax} deg/s, {amax} deg/s^2")
            q = seq[i]
            t_send = time.monotonic_ns()
            log.arm_cmd(push_id, i, [x * DEG for x in q], slot, t_send)
            status, code = 0, None
            try:
                arm.cmd("PushServoJ", 0, *q)
            except link.CommandFailed as exc:       # Name,Fail,code,; the code is its first field (PROTO p.3)
                status, code = 1, exc.code
            except (ElfinError, protocol.ProtocolError):
                status = 2                          # a reply, but not this push's
            except (socket.timeout, TimeoutError, ConnectionError, OSError):
                status = 3
            t_reply = time.monotonic_ns()
            rtt = _reply_ns(arm, t_send, t_reply) if status in (0, 1) else t_reply - t_send
            log.arm_ack(push_id, i, status, code, t_reply, rtt, is_late)
            acks.append({"push_id": push_id, "i": i, "status": status, "code": code, "late": is_late,
                         "reply_ms": rtt / 1e6})
            cmd_series.append((t_send, q[5]))
            push_id += 1
            if status != 0:
                abort = f"push {push_id} answered {('Fail', 'an unexpected reply', 'nothing')[status - 1]}" + \
                        (f" (code {code})" if code is not None else "")
                stop_now = True
                print(f"STOPPED STREAMING: {abort}", file=sys.stderr, flush=True)
                log.event(triallog.EV_ERROR, abort)
                break
            sent = [sent[1], list(q)]
            if not decelerating:
                reason = None
                if late > LATE_LIMIT * n_planned:
                    reason = f"{late} late pushes of {n_planned} planned (> 1 %)"
                latest = states.latest()
                if states.error is not None:
                    reason = reason or f"state stream failed: {states.error!r}"
                elif latest is None or (time.monotonic_ns() - latest[0]) / 1e9 > STATE_STALE_S:
                    reason = reason or "state stream stale"
                else:
                    p = latest[1]
                    st = p["state"]
                    if st.get("Error_Code"):
                        reason = reason or f"state stream reports error code {st['Error_Code']}"
                    if st.get("robotEnabled") == 0:          # Datasheet keys, PROTO p.185-188 (review S17)
                        reason = reason or "state stream reports robotEnabled = 0"
                    if st.get("robotPaused") == 1:
                        reason = reason or "state stream reports robotPaused = 1"
                    if p["q_rad"]:
                        qd = [x / DEG for x in p["q_rad"]]
                        if any(abs(qd[j] - q0[j]) > GUARD_OTHER_JOINTS_DEG for j in range(5)):
                            reason = reason or f"a joint other than J6 moved: {qd}"
                        if not band[0] <= qd[5] <= band[1]:
                            reason = reason or f"J6 at {qd[5]:.3f} deg, outside the planned band {band}"
                if reason:
                    start_decel(i + 1, reason)
            if state_poll_s > 0 and i + 1 < len(seq):             # codebase plan §3.8 item 7 [UNTESTED]
                now = time.monotonic_ns()
                if now - last_poll_ns >= state_poll_s * 1e9:
                    left_ms = (t0 + (i + 1) * T_ns - now) / 1e6
                    typical_ms = max(poll_ms) if poll_ms else statistics.median(a["reply_ms"] for a in acks)
                    if left_ms > POLL_MARGIN * typical_ms:
                        last_poll_ns = now
                        problem = poll_state()
                        if problem:
                            abort, stop_now = problem, True
                            print(f"STOPPED STREAMING: {problem}", file=sys.stderr, flush=True)
                            log.event(triallog.EV_ERROR, problem)
                            break
                    else:
                        polls_skipped += 1
            i += 1
    finally:
        if old_handler is not None:
            signal.signal(signal.SIGINT, old_handler)
    if stop["reason"] is not None and not decelerating and not stop_now:
        abort = stop["reason"]           # it came with the last push: the stream had ended at rest anyway
    return {"acks": acks, "cmd_series": cmd_series, "late": late, "skipped": skipped, "schedule_shifts": shifts,
            "abort": abort, "stopped_on_reply": stop_now, "n_planned": n_planned, "n_sent": len(acks),
            "decelerated": decelerating,
            "state_polls": {"every_s": state_poll_s, "sent": len(poll_ms), "skipped_no_time": polls_skipped,
                            "reply_ms": poll_ms}}


def end_of_stream(arm, log):
    """Read any reply still owed (a push that timed out; one message at a time, PROTO p.1; review S11), watch with
    no pushes, then GrpStop [UNTESTED], then wait for nMovingState = 0. Out of step, the watching is skipped and
    GrpStop and the wait still go out (link.Arm.cmd)."""
    out = {"owed_reply": arm.settle_owed("the end of the stream"), "observed": [], "grpstop": None, "settled": None}
    t_end = time.monotonic() + OBSERVE_S
    while time.monotonic() < t_end:
        try:
            s = arm.state()
            out["observed"].append({k: s[k] for k in ("nMovingState", "nInPos", "nErrorState", "nErrorCode", "nEnableState")})
        except (ElfinError, OSError, protocol.ProtocolError) as exc:     # link.OutOfStep is an OSError
            out["observed"].append({"error": repr(exc)})
            break
        time.sleep(0.05)
    log.event(triallog.EV_PHASE, "GrpStop after the stream [UNTESTED]")
    stopped = arm.stop_motion("the end of the stream")      # GrpStop (once more if no reply), then the wait for rest
    out["grpstop"] = stopped["grpstop"]
    out["settled"] = ({"after_s": stopped["settled_after_s"], "state": stopped.get("state")}
                      if stopped["settled_after_s"] is not None else {"error": stopped.get("settled")})
    return out           # stop_motion cleared motion_may_run: the exit sequence goes straight to GrpDisable


def j6_tracking(cmd_series, packets, t_from, t_to):
    """Commanded J6 (at send times) vs state-stream J6: best constant delay and residuals."""
    st = [(t, p["q_rad"][5] / DEG) for t, p in packets if p["q_rad"] and t_from <= t <= t_to]
    if len(cmd_series) < 2 or len(st) < 5:
        return {"packets": len(st), "note": "too few samples"}
    ct = [c[0] for c in cmd_series]
    cq = [c[1] for c in cmd_series]

    def cmd_at(t):
        if t <= ct[0]:
            return cq[0]
        if t >= ct[-1]:
            return cq[-1]
        lo, hi = 0, len(ct) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            lo, hi = (mid, hi) if ct[mid] <= t else (lo, mid)
        f = (t - ct[lo]) / (ct[hi] - ct[lo])
        return cq[lo] + f * (cq[hi] - cq[lo])

    def errors(lag_ns):
        return [q - cmd_at(t - lag_ns) for t, q in st]

    best = None
    for lag_ms in range(0, 1001, 5):
        e = errors(lag_ms * 1_000_000)
        rms = math.sqrt(sum(x * x for x in e) / len(e))
        if best is None or rms < best[1]:
            best = (lag_ms, rms, max(abs(x) for x in e))
    raw = errors(0)
    return {"packets": len(st), "best_delay_ms": best[0], "rms_at_best_deg": best[1], "max_at_best_deg": best[2],
            "max_raw_deg": max(abs(x) for x in raw)}


def run(argv=None, profile=motion.LadderProfile()):
    ap = cli.parser(__doc__)
    ap.add_argument("--state-poll-s", type=float, default=STATE_POLL_S,
                    help=f"ReadRobotState between pushes about this often, s (default {STATE_POLL_S}; 0 = off). "
                         "Codebase plan §3.8 item 7: needed while the Datasheet's e-stop and light-curtain flags "
                         "are not known to exist")
    args = ap.parse_args(argv)
    if args.state_poll_s < 0:
        ap.error("--state-poll-s must be 0 (off) or positive")
    with cli.session(args, "servo_ladder", "killtest", FIRST_RUN) as (arm, log, confirmed, arm_cfg, ask):
        period, lookahead = confirmed["servo_period_s"], confirmed["servo_lookahead_s"]
        states = link.StateStream(arm, log).start()
        result = after = check = None
        q_plan = []
        try:
            with arm.disable_on_exit():
                def plan_and_ask():
                    """Before GrpEnable (review S9): the plan, its check against confirmed.yaml, the go."""
                    q_plan[:] = arm.act_pos()[0]
                    samples = motion.sample(profile, q_plan, period)
                    c = motion.check_stream(samples, period, confirmed, profile)
                    print(f"plan: T_s {period} s, lookahead {lookahead} s, {len(samples)} pushes over {profile.total_s} s; "
                          f"peak {c['peak_speed_deg_s']:.3f} deg/s (limit {c['limit_speed']}), "
                          f"{c['peak_acc_deg_s2']:.3f} deg/s^2 (limit {c['limit_acc']})", flush=True)
                    if not ask("Start streaming? The arm is enabled next, and the stream starts right after it "
                               "with no further question."):
                        raise link.Refused("operator did not start the stream")

                arm.prepare_and_enable(arm_cfg, ask, before_enable=plan_and_ask,
                                       arm_block={"servo_period_s": period, "servo_lookahead_s": lookahead})
                q0 = arm.act_pos()[0]           # after enabling ("sound and slight movement", LINUX p.17-18)
                moved = max(abs(a - b) for a, b in zip(q0, q_plan))
                if moved > ENABLE_MOVE_TOL_DEG:
                    raise link.Refused(f"the joints moved {moved:.3f} deg between the plan and the enable (more than "
                                       f"{ENABLE_MOVE_TOL_DEG} deg): {q_plan} -> {q0}; start again")
                samples = motion.sample(profile, q0, period)     # from where the arm is now; same profile, same peaks
                check = motion.check_stream(samples, period, confirmed, profile)
                plan = {"period_s": period, "lookahead_s": lookahead, "pushes": len(samples), "q0_deg": q0,
                        "q_before_enable_deg": list(q_plan), "moved_at_enable_deg": moved,
                        "profile": profile.__dict__, "limits": check}
                log.event(triallog.EV_INFO, plan)
                arm.check_before_motion()
                t_wait = time.monotonic() + 1.0
                while states.latest() is None and time.monotonic() < t_wait:
                    time.sleep(0.01)
                if states.latest() is None:
                    raise link.Refused("no Datasheet packets on the state port: refusing to stream unwatched")
                t_start = time.monotonic_ns()
                result = stream(arm, log, states, samples, period, lookahead, q0, profile, confirmed,
                                state_poll_s=args.state_poll_s)
                t_stream_end = time.monotonic_ns()
                after = end_of_stream(arm, log)
        finally:
            states.stop()
        if result is None:
            return 2
        replies = sorted(a["reply_ms"] for a in result["acks"])
        late_frac = result["late"] / result["n_planned"]
        tracking = j6_tracking(result["cmd_series"], states.packets, t_start, t_stream_end)
        fails = [a for a in result["acks"] if a["status"] != 0]
        moving_in_stream = sorted({p["state"].get("robotMoving") for t, p in states.packets if t_start <= t <= t_stream_end})
        rough = None
        if args.mode == "arm":
            rough = not ask("Was the motion smooth: no audible or visible roughness?")
        j6_flag = (result["abort"] is not None and "J6" in (result["abort"] or "")) or \
                  tracking.get("max_at_best_deg", 0.0) > J6_RESIDUAL_FLAG_DEG or tracking.get("packets", 0) < 5
        rules = {
            "any_fail": bool(fails),
            "late_over_1pct": late_frac > LATE_LIMIT,
            "median_reply_over_half_period": bool(replies) and statistics.median(replies) > 500 * period,
            "j6_looks_wrong": j6_flag,
            "roughness": rough,
            "aborted": result["abort"],
        }
        stop = any(v for k, v in rules.items() if k != "aborted") or result["abort"] is not None
        summary = {
            "trial_id": log.trial_id, "period_s": period, "lookahead_s": lookahead,
            "pushes_planned": result["n_planned"], "pushes_sent": result["n_sent"],
            "late": result["late"], "late_fraction": late_frac, "skipped_slots": result["skipped"],
            "schedule_shifts": result["schedule_shifts"],
            "reply_ms": {"median": statistics.median(replies) if replies else None,
                         "p95": replies[int(0.95 * (len(replies) - 1))] if replies else None,
                         "max": replies[-1] if replies else None},
            "fails": fails, "decelerated": result["decelerated"], "abort": result["abort"],
            "j6_tracking": tracking, "robotMoving_during_stream": moving_in_stream,
            "after_stream": after, "state_packets": len(states.packets), "limits": check,
            "datasheet_keys": sorted(states.keys),        # which keys the real packets carry (§3.8 item 7, week 1)
            "state_polls": {k: v for k, v in result["state_polls"].items() if k != "reply_ms"} | {
                "median_ms": statistics.median(result["state_polls"]["reply_ms"]) if result["state_polls"]["reply_ms"] else None,
                "max_ms": max(result["state_polls"]["reply_ms"]) if result["state_polls"]["reply_ms"] else None},
            "stop_rules": rules, "verdict": "STOP AND REPORT" if stop else "OK: the owner may confirm the next period",
        }
        log.event(triallog.EV_SUMMARY, summary)
        (log.path.with_suffix(".summary.json")).write_text(json.dumps(summary, indent=2, default=str), encoding="utf-8")
        r = summary["reply_ms"]
        print(f"\n{summary['pushes_sent']}/{summary['pushes_planned']} pushes; late {result['late']} "
              f"({100 * late_frac:.2f} %), skipped slots {result['skipped']}; reply median {r['median']:.2f} ms, "
              f"p95 {r['p95']:.2f} ms, max {r['max']:.2f} ms")
        print(f"J6 tracking: {tracking}")
        sp = summary["state_polls"]
        print(f"ReadRobotState between pushes: {sp['sent']} sent, {sp['skipped_no_time']} skipped for lack of slot "
              f"time; reply median {sp['median_ms']} ms, max {sp['max_ms']} ms")
        print(f"after the stream: GrpStop {after['grpstop']}; settled {after['settled']}")
        for k, v in rules.items():
            print(f"  {k:32s} {v}")
        print(f"VERDICT: {summary['verdict']}")
        return 1 if stop else 0


def main(argv=None):
    return run(argv)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        sys.exit(f"stopped: {exc}")
