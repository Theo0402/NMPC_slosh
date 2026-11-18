"""The logged link to the arm: elfin_min.py unchanged, wrapped at the socket (codebase plan §3.8, LG4).

- Every byte sent and received on port 10003 is logged with time.monotonic_ns().
- Only commands the command table marks as used can be sent; parameters are formatted by the
  table and passed to Elfin.cmd() as strings (codebase plan R-B).
- Checks before motion, the enable sequence and the exit sequence follow codebase plan §3.8
  and the owner's week-1 decisions:
    * refuse if the e-stop or light curtain is active, or if the redundant channels disagree;
    * never send Electrify, StartMaster, ConnectToBox, BlackOut or OSCmd; refuse if unpowered;
    * after an error, print the code and axis and ask the operator before GrpReset;
    * read back the limits and the current frames and refuse if they differ from e03pro.yaml;
    * the operator confirms payload and TCP before GrpEnable;
    * on every exit, in a finally block: read the reply of any message in flight, send GrpStop if a move
      may be under way and wait for rest, then GrpDisable. Each step runs even if the one before failed
      (pre-hardware review S3, S10; owner's go-ahead 2026-11-18);
    * one message at a time also after a time-out: the reply still owed is read before the next message,
      and if it never comes only the stop commands go out, their replies matched by name (review S11).
Modes: "arm" (Linux only), "mock" (the local mock controller), "dry-run" (no connection; the
mock's simulation answers in-process and every command is printed).
Standard library only.
"""
import ast
import contextlib
import os
import signal
import socket
import sys
import threading
import time

from tools.elfin_py import datasheet, protocol
from tools.elfin_py.elfin_min import Elfin, ElfinError

CMD_PORT_LOG = 10003
MOTION_COMMANDS = ("WayPoint", "StartServo", "PushServoJ")   # after one of these, a move may be under way
STOP_SETTLE_S = 10.0     # longest wait for nMovingState = 0 after GrpStop, as the ladder and jog already wait [proposal]
REPLY_WAIT_S = 2.0       # a reply still owed after a time-out is waited for this much longer [proposal]
AT_TARGET = {"joint_deg": 0.1, "mm": 0.5, "rot_deg": 0.2}     # "at the target" for wait_done [proposal] [UNTESTED]
JOINT_SPEED_POLL_S = 0.05    # ReadActJointVel this often while waiting for a WayPoint move [proposal]
JOINT_SPEED_MARGIN = 0.10    # stop the move above the confirmed joint speed by more than this fraction [proposal]


def _loud(log, msg):
    """A message the operator must see: stderr (never raises) and the trial log."""
    try:
        print(msg, file=sys.stderr, flush=True)
    except Exception:
        pass
    log.event(3, msg)


class Refused(RuntimeError):
    """A check failed: the tool refuses to go on (codebase plan §3.8)."""


class OutOfStep(ConnectionError):
    """A reply never came, so the link is out of step (one message at a time, PROTO p.1): only the stop
    commands go out after this (pre-hardware review S11)."""


class JointSpeedExceeded(Refused):
    """A joint ran faster than confirmed.yaml's joint speed during a WayPoint move (pre-hardware review S20)."""


class CommandFailed(ElfinError):
    """The controller answered Name,Fail,ErrorCode,; (elfin_commands.md, PROTO p.3). `code` is the integer code, the
    first field after Fail; its meaning is in HansRobot_ErrorCode.docx, which is not attached [MISSING]."""

    def __init__(self, name, fields):
        self.name, self.fields = name, list(fields)
        try:
            self.code = int(float(self.fields[0])) if self.fields else None
        except ValueError:
            self.code = None
        super().__init__(f"{name} failed, error code {self.fields}")


def _as_failed(name, exc):
    """elfin_min raises ElfinError(f"{name} failed, error code {fields}") on Fail; rebuild the fields from it, so
    that no code is pieced together from digits (pre-hardware review W8)."""
    text, head = str(exc), f"{name} failed, error code "
    if not text.startswith(head):
        return None
    try:
        fields = ast.literal_eval(text[len(head):])
    except (ValueError, SyntaxError):
        return None
    return CommandFailed(name, fields) if isinstance(fields, list) else None


def running_in_wsl(proc_version="/proc/version", environ=None):
    """True inside WSL: its kernel names Microsoft in /proc/version, and WSL sets WSL_DISTRO_NAME
    or WSL_INTEROP. WSL counts as Linux to sys.platform but is a development machine only."""
    env = os.environ if environ is None else environ
    if env.get("WSL_DISTRO_NAME") or env.get("WSL_INTEROP"):
        return True
    try:
        with open(proc_version, encoding="utf-8", errors="replace") as f:
            return "microsoft" in f.read().lower()
    except OSError:
        return False


def arm_host_problem(platform=None, wsl=None):
    """Why this computer may not run a tool with --arm, or None if it may."""
    platform = sys.platform if platform is None else platform
    if not platform.startswith("linux"):
        return f"this is {platform}, not Linux"
    if running_in_wsl() if wsl is None else wsl:
        return "this is WSL on the development PC, not the Linux machine"
    return None


def frame_problems(tool, user, mounting):
    """The current frames (as ReadActPos returns them, PROTO p.66-67) vs e03pro.yaml's mounting."""
    ftol = mounting.get("frame_tolerance", 1e-3)
    out = []
    for k, got in (("tool_frame", tool), ("user_frame", user)):
        if any(abs(a - b) > ftol for a, b in zip(got, mounting[k])):
            out.append(f"{k}: read {got}, e03pro.yaml has {mounting[k]}")
    return out


class LoggedSocket:
    """Wraps the socket elfin_min.Elfin uses. Counts messages sent and replies seen, so that
    a message in flight can be drained before GrpDisable."""

    def __init__(self, sock, log, port=CMD_PORT_LOG, echo=False):
        self.sock, self.log, self.port, self.echo = sock, log, port, echo
        self.sent = 0
        self.replies = 0
        # reply times without our own log writes (pre-hardware review W8): when the last message went out, when the
        # last reply's end was read, and how long the log writes (and dry-run echoes) took in between
        self.last_tx_ns = self.last_rx_ns = None
        self._log_ns = self._rx_log_ns = 0

    @property
    def outstanding(self):
        return self.sent - self.replies

    def reply_ns(self):
        """The last reply's time on the wire: its send to its end, less our log writes in between; None if no reply
        has been read since the last send."""
        if self.last_tx_ns is None or self.last_rx_ns is None or self.last_rx_ns < self.last_tx_ns:
            return None
        return max(0, self.last_rx_ns - self.last_tx_ns - self._rx_log_ns)

    def _logged(self, fn, *args):
        t0 = time.monotonic_ns()
        try:
            fn(*args)
        finally:
            self._log_ns += time.monotonic_ns() - t0

    def sendall(self, data):
        self.sent += data.count(b";")       # counted first: a reply that may be owed is waited for, never skipped
        t = time.monotonic_ns()
        try:
            self.sock.sendall(data)
            self.last_tx_ns, self._log_ns = t, 0
        finally:
            # logged after the send; TrialLog never raises (a failed write marks the log and requests a stop), so a
            # full disk cannot keep this message, GrpStop or GrpDisable from going out (review S8)
            self._logged(self.log.wire_tx, self.port, data, t)
            if self.echo:
                self._logged(print, "  ->", data.decode("ascii", "replace"))

    def recv(self, n):
        chunk = self.sock.recv(n)
        t = time.monotonic_ns()
        if b";" in chunk:
            self.last_rx_ns, self._rx_log_ns = t, self._log_ns
        self._logged(self.log.wire_rx, self.port, chunk, t)
        self.replies += chunk.count(b";")
        if self.echo and chunk:
            self._logged(print, "  <-", chunk.decode("ascii", "replace"))
        return chunk

    def settimeout(self, t):
        self.sock.settimeout(t)

    def setsockopt(self, *a):
        self.sock.setsockopt(*a)

    def close(self):
        self.sock.close()


class DryRunSocket:
    """No connection: the mock's simulation answers in-process (codebase plan §3.8 item 10)."""

    def __init__(self, mock_arm):
        self.arm = mock_arm
        self.rx = b""
        self.timeout = None

    def sendall(self, data):
        text = data.decode("ascii")
        for msg in [m + ";" for m in text.split(";") if m]:
            self.rx += self.arm.handle(msg).encode("ascii")

    def recv(self, n):
        if not self.rx:
            raise socket.timeout("dry run: no reply pending")
        out, self.rx = self.rx[:n], self.rx[n:]
        return out

    def settimeout(self, t):
        self.timeout = t

    def setsockopt(self, *a):
        pass

    def close(self):
        pass


class Arm:
    def __init__(self, mode, log, ip="127.0.0.1", cmd_port=10003, state_port=10004, mock_arm=None,
                 timeout=2.0, echo=None, connect=True):
        if mode == "arm":
            where = arm_host_problem()
            if where:
                raise Refused(f"on the arm, run from the Linux machine only (owner, week 1): {where}")
        self.mode, self.log, self.ip, self.state_port = mode, log, ip, state_port
        echo = (mode == "dry-run") if echo is None else echo
        self.r = None
        self.timeout = timeout          # the socket's reply time-out, s
        self.enabled_by_us = False     # the exit sequence runs only if this tool enabled the arm
        self.motion_may_run = False    # set when a motion command is sent, cleared once rest is seen
        self.out_of_step = False       # a reply never came: only the stop commands go out (review S11)
        self.joint_speed_limit = None  # deg/s: confirmed.yaml's joint speed, set by cli.session (review S20)
        self._extra_seen = set()
        if mode == "dry-run":
            from tools.elfin_mock.sim import MockArm
            self.mock_arm = mock_arm or MockArm()
        else:
            self.mock_arm = None
        if not connect:            # e.g. the state-stream tool, which needs port 10004 only
            return
        if mode == "dry-run":
            self.r = Elfin.__new__(Elfin)
            self.r.buf = b""
            self.sock = LoggedSocket(DryRunSocket(self.mock_arm), log, echo=echo)
            self.r.sock = self.sock
        else:
            self.r = Elfin(ip, cmd_port, timeout=timeout)
            self.sock = LoggedSocket(self.r.sock, log, echo=echo)
            self.r.sock = self.sock

    def close(self):
        if self.r is not None:
            self.r.sock.close()

    # ------------------------------------------------------------------ commands
    def cmd(self, name, *values, exiting=False):
        """Send one command through the table; return the reply as a dict by the table's names.
        Raises ElfinError on Fail (PROTO p.3), and ProtocolError if a read reply has another shape. An action
        command's OK is accepted whatever follows it (protocol.parse_reply); fields after it are logged.

        One message at a time (PROTO p.1; review S11): a reply still owed by an earlier message (after a time-out)
        is read first. If it never comes, the link is out of step. From then on only the commands of the stop and
        exit sequence go out (exiting=True: GrpStop, ReadRobotState while waiting for rest, GrpDisable), and their
        replies are matched by name, late replies being logged and dropped; any other command raises OutOfStep."""
        protocol.check_sendable(name)
        params = protocol.format_params(name, *values)
        if not self.out_of_step and self.sock.outstanding > 0:
            self.settle_owed(name)
        if self.out_of_step and not exiting:
            raise OutOfStep(f"the link to the arm is out of step (a reply never came): {name} was not sent")
        if name in MOTION_COMMANDS:
            self.motion_may_run = True           # before sending: if the reply is lost, the move may still run
        if self.out_of_step:
            fields = self._cmd_by_name(name, params)
        else:
            try:
                fields = self.r.cmd(name, *params)   # raises ElfinError on Fail (PROTO p.3)
            except ElfinError as exc:
                failed = _as_failed(name, exc)
                if failed is None:
                    raise                            # an unexpected reply, not a Fail
                raise failed from None
        if fields and protocol.is_action(name) and name not in self._extra_seen:
            self._extra_seen.add(name)           # once per command: the raw bytes of every reply are in wire_rx
            msg = (f"{name} replied OK with {len(fields)} field(s) after OK: {fields}. elfin_commands.md does not "
                   "state them [UNTESTED]; the command was accepted and the run goes on.")
            print(f"note: {msg}", flush=True)
            self.log.event(4, msg)
        return protocol.parse_reply(name, fields)

    def settle_owed(self, before):
        """Read any reply still owed before `before` goes out (one message at a time, PROTO p.1), waiting up to
        REPLY_WAIT_S. If it never comes, the link is out of step (see cmd). Returns what happened, for the log,
        or None if nothing was owed."""
        owed = self.sock.outstanding
        if self.out_of_step or owed <= 0:
            return None
        self.drain(REPLY_WAIT_S)
        if self.sock.outstanding <= 0:
            self.log.event(4, f"read {owed} late reply(ies) before sending {before} (one message at a time, PROTO p.1)")
            return {"owed": owed, "read": True}
        self.out_of_step = True
        self.sock.replies = self.sock.sent       # replies are matched by name from here on
        _loud(self.log, f"OUT OF STEP: no reply came to the previous message (if it went out at all), even "
                        f"{REPLY_WAIT_S:g} s after its time-out. Only the stop commands go out from now on "
                        f"(GrpStop, ReadRobotState while waiting for rest, GrpDisable); {before} was next.")
        return {"owed": owed, "read": False}

    def _cmd_by_name(self, name, params):
        """Out of step: send, then read replies until the one for `name`; late replies to earlier messages are
        logged and dropped. Every reply starts with its command's name (Name,OK,...,; or Name,Fail,code,;
        elfin_commands.md, PROTO p.3). The parsing is elfin_min.Elfin.cmd's."""
        r = self.r
        r.sock.sendall((",".join([name, *params]) + ",;").encode("ascii"))
        t_end = time.monotonic() + self.timeout
        try:
            while True:
                while b";" not in r.buf:
                    left = t_end - time.monotonic()
                    if left <= 0:
                        raise TimeoutError(f"no reply to {name} within {self.timeout:g} s (out of step)")
                    r.sock.settimeout(left)
                    try:
                        chunk = r.sock.recv(4096)
                    except (socket.timeout, TimeoutError):
                        if self.mode == "dry-run":          # nothing can arrive later in a dry run
                            raise TimeoutError(f"no reply to {name} (dry run, out of step)") from None
                        continue
                    if not chunk:
                        raise ConnectionError("controller closed the connection")
                    r.buf += chunk
                reply, _, r.buf = r.buf.partition(b";")
                f = [x.strip() for x in reply.decode("ascii", "replace").split(",")]
                if f and f[-1] == "":
                    f.pop()
                if len(f) >= 2 and f[0] == name:
                    self.sock.replies = self.sock.sent
                    if f[1] != "OK":
                        raise CommandFailed(name, f[2:])
                    return f[2:]
                self.log.event(4, f"out of step: dropped a late reply while waiting for {name}: {reply!r}")
        finally:
            r.sock.settimeout(self.timeout)

    def state(self, exiting=False):
        return self.cmd("ReadRobotState", 0, exiting=exiting)

    def act_pos(self):
        v = self.cmd("ReadActPos", 0)
        vals = list(v.values())
        return vals[0:6], vals[6:12], vals[12:18], vals[18:24]

    def wait_for(self, test, timeout, period=0.01, exiting=False):
        t_end = time.monotonic() + timeout
        while True:
            s = self.state(exiting=exiting)
            if test(s):
                return s
            if time.monotonic() > t_end:
                raise TimeoutError(f"gave up waiting; last state {s}")
            time.sleep(period)

    def at_target(self, joints=None, pose=None):
        """True if ReadActPos (PROTO p.66-67) shows the arm within AT_TARGET of the joint target (deg) or the
        pose target (mm, deg)."""
        j, p, _, _ = self.act_pos()
        if joints is not None and any(abs(a - b) > AT_TARGET["joint_deg"] for a, b in zip(j, joints)):
            return False
        if pose is not None:
            if any(abs(a - b) > AT_TARGET["mm"] for a, b in zip(p[:3], pose[:3])):
                return False
            if any(abs((a - b + 180.0) % 360.0 - 180.0) > AT_TARGET["rot_deg"] for a, b in zip(p[3:6], pose[3:6])):
                return False
        return True

    def wait_done(self, timeout=60.0, start_grace=0.5, joints=None, pose=None):
        """elfin_min's wait_done, through the table: poll every 10 ms until nMovingState = 0 (elfin_examples.md;
        PROTO p.189-191). How soon nMovingState turns 1 after the reply is not stated [MISSING], so a 0 seen
        before the move was seen to start counts as done only if the arm is already at the target: `joints` (deg)
        or `pose` (mm, deg), compared with ReadActPos (review S12). Without a target, as before: start_grace s
        without a start count as done.

        With joint_speed_limit set, the joint speeds are read every JOINT_SPEED_POLL_S (ReadActJointVel, deg/s,
        PROTO p.71); a joint above the limit by more than JOINT_SPEED_MARGIN raises JointSpeedExceeded, and the
        exit sequence stops the move. The peak per joint is logged for each move (review S20)."""
        t0, started = time.monotonic(), False
        peak, next_speed_read = [0.0] * 6, 0.0
        limit = self.joint_speed_limit
        while True:
            s = self.state()
            if s["nErrorState"]:
                raise ElfinError(f"motion error {s['nErrorCode']} on axis {s['nErrorAxis']}")
            started = started or s["nMovingState"] == 1
            t = time.monotonic() - t0
            if limit is not None and t >= next_speed_read:
                next_speed_read = t + JOINT_SPEED_POLL_S
                qd = list(self.cmd("ReadActJointVel", 0).values())
                peak = [max(p, abs(v)) for p, v in zip(peak, qd)]
                over = [f"J{j} {abs(v):.2f} deg/s" for j, v in enumerate(qd, 1) if abs(v) > limit * (1 + JOINT_SPEED_MARGIN)]
                if over:
                    raise JointSpeedExceeded(
                        f"{', '.join(over)} during the move, above the confirmed joint speed {limit} deg/s "
                        f"(+{JOINT_SPEED_MARGIN:.0%}, confirmed.yaml): stopping the move")
            if s["nMovingState"] == 0:
                if started or ((joints is not None or pose is not None) and self.at_target(joints, pose)):
                    self.motion_may_run = False  # rest seen after the move, or at the target: no GrpStop on exit
                    if limit is not None:
                        self.log.event(4, {"move_peak_joint_speed_deg_s": [round(p, 3) for p in peak],
                                           "confirmed_joint_speed_deg_s": limit})
                    return s
                if joints is None and pose is None and t > start_grace:
                    return s
            if t > timeout:
                raise TimeoutError("move did not finish")
            time.sleep(0.01)

    # ------------------------------------------------------------------ checks (codebase plan §3.8 item 2)
    def check_before_motion(self):
        s = self.state()
        problems = []
        if s["nEmergencyStop"]:
            problems.append("e-stop active (nEmergencyStop = 1)")
        if s["nSaftyGuard"]:
            problems.append("light curtain triggered (nSaftyGuard = 1)")
        if s["nErrorState"]:
            problems.append(f"error state, code {s['nErrorCode']} on axis {s['nErrorAxis']}")
        if not s["nEnableState"]:
            problems.append("not enabled (nEnableState = 0)")
        e = self.cmd("ReadEmergencyInfo", 0)
        if e["nESTO_Error"] or e["nSafetyGuard_Error"]:
            problems.append(f"redundant safety channels disagree: {e}")
        if problems:
            raise Refused("refusing to move: " + "; ".join(problems))
        return s, e

    def readback(self, arm_cfg):
        """Limits and current frames vs e03pro.yaml (codebase plan §3.8 item 8). Returns a dict for the log."""
        out = {"joint_max_vel_deg_s": list(self.cmd("ReadJointMaxVel", 0).values()),
               "joint_max_acc_deg_s2": list(self.cmd("ReadJointMaxAcc", 0).values()),
               "joint_max_jerk_deg_s3": list(self.cmd("ReadJointMaxJerk", 0).values()),
               "linear_max": list(self.cmd("ReadLinearMaxVel", 0).values())}
        _, _, tool, user = self.act_pos()
        out["tool_frame"], out["user_frame"] = tool, user
        return out

    @staticmethod
    def compare_readback(rb, arm_cfg):
        problems = []
        lr = arm_cfg.get("limits_readback") or {}
        tol = lr.get("tolerance", 1e-3)
        for k in ("joint_max_vel_deg_s", "joint_max_acc_deg_s2", "joint_max_jerk_deg_s3", "linear_max"):
            want = lr.get(k)
            if want is None:
                problems.append(f"{k} not recorded in e03pro.yaml yet (run bringup_read first)")
            elif len(want) != len(rb[k]) or any(abs(a - b) > tol for a, b in zip(want, rb[k])):
                problems.append(f"{k}: read {rb[k]}, e03pro.yaml has {want}")
        problems += frame_problems(rb["tool_frame"], rb["user_frame"], arm_cfg["mounting"])
        return problems

    # ------------------------------------------------------------------ enable and exit
    def log_arm_block(self, readback, arm_cfg, extra=None):
        """Codebase plan §5's arm block, as the event of kind 5 that schema/fields.yaml gives it (the header is written
        before the arm is reached): the model string (ReadRobotModel, elfin_commands.md, PROTO p.16), the controller
        version as entered from the pendant's About screen (elfin_overview.md, LINUX p.19; e03pro.yaml identity),
        the limits and frames read back at start-up (PROTO p.56-59, p.66-67), the payload and TCP in use as the
        operator confirmed them, and what the tool adds (the servo period and lookahead in use). The confirmed values
        are in the header (pre-hardware review W10)."""
        try:
            model = self.cmd("ReadRobotModel").get("strModel")
        except Exception as exc:                 # a read: report it, never stop the start-up for it
            model = f"not read: {exc!r}"
        ident = arm_cfg.get("identity") or {}
        block = {"model": model, "model_in_config": ident.get("model"),
                 "controller_version": ident.get("controller_version") or "[MISSING] not entered in e03pro.yaml",
                 "readback": readback, "mounting_confirmed": arm_cfg.get("mounting"), **(extra or {})}
        self.log.event(5, block)
        return block

    def prepare_and_enable(self, arm_cfg, ask, before_enable=None, _depth=0, arm_block=None):
        """The owner's enable sequence (week 1). `ask(question) -> bool` is the operator prompt.
        `before_enable()`, if given, runs after the operator's confirmation and right before GrpEnable: a tool's
        last question goes there, so that no question waits with the arm enabled (pre-hardware review S9).
        `arm_block` adds the tool's own entries to the arm block (log_arm_block)."""
        s = self.state()
        if s["nEmergencyStop"] or s["nSaftyGuard"]:
            raise Refused("e-stop or light curtain is active")
        if not s["nElectrify"]:
            raise Refused("the arm is unpowered (nElectrify = 0): power it on from the pendant. "
                          "The tools never send Electrify.")
        e = self.cmd("ReadEmergencyInfo", 0)
        if e["nESTO_Error"] or e["nSafetyGuard_Error"]:
            raise Refused(f"redundant safety channels disagree: {e}")
        if s["nErrorState"]:
            codes = self.cmd("ReadAxisErrorCode", 0)
            msg = (f"The arm reports an error: code {s['nErrorCode']} on axis {s['nErrorAxis']} "
                   f"(ReadRobotState, axis 0-5); ReadAxisErrorCode: {codes}. "
                   "The code list is not in the knowledge files [MISSING].")
            print(msg)
            self.log.event(3, msg)
            if _depth or not ask("Send GrpReset to clear it?"):
                raise Refused("error not cleared; operator declined GrpReset")
            self.cmd("GrpReset", 0)
            self.wait_for(lambda st: st["nErrorState"] == 0, timeout=5)
            return self.prepare_and_enable(arm_cfg, ask, before_enable, _depth=1, arm_block=arm_block)
        rb = self.readback(arm_cfg)
        problems = self.compare_readback(rb, arm_cfg)
        if problems:
            raise Refused("start-up read-back differs from e03pro.yaml: " + "; ".join(problems))
        m = arm_cfg["mounting"]
        if not ask(f"Mounting: {m['description']}. Does the pendant show payload {m['payload_kg']} kg and "
                   f"tool frame '{m['tcp_name']}' = {m['tool_frame']}? Is the e-stop in reach and the "
                   "workspace clear? Enabling releases the brakes with sound and slight movement."):
            raise Refused("operator did not confirm payload, TCP and workspace")
        self.log.event(4, f"operator confirmed mounting {m}")
        self.log_arm_block(rb, arm_cfg, arm_block)
        if before_enable is not None:
            before_enable()
        if not s["nEnableState"]:
            # set before sending (review S10): if the reply is lost or the wait fails, the exit still disables
            self.enabled_by_us = True
            self.cmd("GrpEnable", 0)
            self.wait_for(lambda st: st["nEnableState"] == 1, timeout=30)
        else:
            self.enabled_by_us = True     # we will still disable it on exit
        return self.check_before_motion()

    def drain(self, timeout=REPLY_WAIT_S):
        """Read the reply of any message in flight (one message at a time, PROTO p.1). A Fail among the replies
        read here is reported: its code would otherwise be lost. Out of step, replies are matched by name
        instead, so there is nothing to wait for."""
        if self.out_of_step:
            return
        t_end = time.monotonic() + timeout
        got = bytes(self.r.buf)
        self.r.sock.settimeout(0.2)
        try:
            while self.sock.outstanding > 0 and time.monotonic() < t_end:
                try:
                    chunk = self.r.sock.recv(4096)
                except (socket.timeout, TimeoutError):
                    if self.mode == "dry-run":          # nothing can arrive later in a dry run
                        break
                    continue
                if not chunk:
                    break
                got += chunk
        finally:
            self.r.sock.settimeout(self.timeout)
            self.r.buf = b""
        if b",Fail," in got:
            _loud(self.log, f"a late reply read before the next message was a Fail: {got!r}")

    def stop_motion(self, why):
        """GrpStop (elfin_commands.md: "stop motion", PROTO p.20; what it does on this arm, in servo mode
        included, is [UNTESTED]), then wait up to STOP_SETTLE_S for nMovingState = 0 (PROTO p.61-62,
        p.189-191). Reads the reply of any message in flight first (one message at a time, PROTO p.1).
        Never raises: the caller sends GrpDisable next. Returns what happened, for the log."""
        out = {"why": why, "grpstop": None, "settled_after_s": None}
        try:
            self.drain()
        except Exception as exc:
            out["drain"] = repr(exc)
        for attempt in (1, 2):
            try:
                self.cmd("GrpStop", 0, exiting=True)
                out["grpstop"] = "OK" if attempt == 1 else "OK at the second try"
                break
            except TimeoutError as exc:          # no reply: it may have been dropped (PROTO p.1), so once more
                out["grpstop"] = repr(exc)
            except Exception as exc:
                out["grpstop"] = repr(exc)
                break
        t0 = time.monotonic()
        try:
            out["state"] = self.wait_for(lambda st: st["nMovingState"] == 0, timeout=STOP_SETTLE_S, exiting=True)
            out["settled_after_s"] = round(time.monotonic() - t0, 3)
        except Exception as exc:
            out["settled"] = repr(exc)
        self.motion_may_run = False              # GrpStop has been tried; a second one would add nothing
        self.log.event(2, {"GrpStop [UNTESTED on this arm]": out})
        if out["settled_after_s"] is None:
            _loud(self.log, f"after GrpStop ({why}) the arm did not report rest within {STOP_SETTLE_S:g} s "
                            f"({out}): GrpDisable follows; use the e-stop if it is still moving")
        return out

    def exit_sequence(self):
        """What every tool that enabled the arm runs on exit (codebase plan §3.8 item 8; pre-hardware review S3,
        S10): read the reply of any message in flight; GrpStop and a wait for rest if a move may be under
        way (what GrpDisable does to a moving arm is not in the knowledge files [MISSING]: elfin_commands.md
        says only "disable", PROTO p.18); then GrpDisable. Each step runs even if the one before failed."""
        try:
            self.drain()
        except Exception as exc:
            _loud(self.log, f"reading the reply in flight failed on exit ({exc!r}); going on")
        if self.motion_may_run:
            self.stop_motion("exit while a move may be under way")
        try:
            self.cmd("GrpDisable", 0, exiting=True)
            self.log.event(2, "GrpDisable sent on exit")
        except Exception as exc:          # report loudly; nothing else can be done from here
            _loud(self.log, f"GrpDisable FAILED on exit ({exc!r}): disable from the pendant or use the e-stop")

    @contextlib.contextmanager
    def disable_on_exit(self):
        """The exit sequence in a finally block, also on an exception or Ctrl-C (owner, week 1). Ctrl-C and the
        workbench's stop are ignored while it runs: the e-stop is the operator's next step, not a second stop."""
        try:
            yield self
        finally:
            old = None
            if threading.current_thread() is threading.main_thread():
                old = signal.signal(signal.SIGINT, signal.SIG_IGN)
            try:
                if self.enabled_by_us:
                    self.exit_sequence()
            finally:
                if old is not None:
                    signal.signal(signal.SIGINT, old)


class StateStream:
    """Background reader of the Datasheet (port 10004): logs state_raw and robot_state and keeps
    the parsed packets for the tool. In dry-run, packets come from the in-process mock."""

    def __init__(self, arm, log):
        self.log, self.packets, self.error = log, [], None
        self.keys = set()
        self._stop = threading.Event()
        self._mock = arm.mock_arm if arm.mode == "dry-run" else None
        self._reader = None if self._mock else datasheet.DatasheetReader(arm.ip, arm.state_port)
        self.byte_orders = set()
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self.thread.start()
        return self

    def stop(self):
        self._stop.set()
        if self._reader:
            self._reader.close()
        self.thread.join(timeout=2)

    def latest(self):
        return self.packets[-1] if self.packets else None

    def _handle(self, arrival, raw, d):
        parsed = datasheet.parse_state(d)
        self.log.state_raw(arrival, raw)
        self.log.robot_state(arrival, parsed)
        self.keys.update(datasheet.key_tree(d))
        self.packets.append((arrival, parsed))

    def _run(self):
        try:
            if self._mock:
                while not self._stop.is_set():
                    raw = self._mock.datasheet_packet()
                    t = time.monotonic_ns()
                    _, n, order = datasheet.split_header(raw[:12])
                    self.byte_orders.add(order)
                    self._handle(t, raw, datasheet.decode_body(raw[12:]))
                    self._stop.wait(1.0 / self._mock.cfg.datasheet_rate_hz)
            else:
                for arrival, raw, d in self._reader:
                    self._handle(arrival, raw, d)
                    self.byte_orders = self._reader.byte_orders
                    if self._stop.is_set():
                        break
        except Exception as exc:       # recorded; the tool's staleness check acts on it
            if not self._stop.is_set():
                self.error = exc
