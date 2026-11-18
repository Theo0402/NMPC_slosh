"""Simulated Han's controller for the mock (codebase plan §3.8 item 10).

It tests our framing and our reactions, not the arm's behaviour. Everything the knowledge
files leave open ([MISSING], [UNTESTED]) is a configurable choice here, so that tests can
exercise each possibility rather than one guess:
- what nMovingState shows in servo mode (`servo_moving_state`);
- what happens when pushes stop (`on_push_starvation`);
- whether GrpStop ends servo mode (`grpstop_ends_servo`);
- the Datasheet's byte order (`datasheet_byte_order`).

Values the mock reports (model string, limits, home pose) are mock values, labelled so,
taken from the manual's examples where elfin_commands.md quotes them. The mock has no
kinematics: a linear move changes the pose and leaves the joints alone.

Mock-only Fail codes are the E_* constants below; the real list, HansRobot_ErrorCode.docx,
is [MISSING] (elfin_commands.md, PROTO p.3).
"""
import json
import math
import random
import struct
import threading
import time
from dataclasses import dataclass, field

from tools.elfin_py import protocol

# Mock-only failure codes. They start at 990001 so they cannot be mistaken for real ones.
E_FORMAT = 990001         # message does not match the command table
E_FORBIDDEN = 990002      # a command the owner forbade (Electrify, StartMaster, ...)
E_NOT_ENABLED = 990003    # motion while disabled
E_BUSY = 990004           # motion command while a motion runs (one motion at a time, PROTO p.1)
E_NOT_SERVO = 990005      # PushServoJ outside servo mode
E_LIMIT = 990006          # outside a protocol range or the mock's joint ranges
E_UNSUPPORTED = 990007    # valid, but the mock does not simulate it
E_ERROR_ACTIVE = 990008   # motion while the error state is set
E_ESTOP = 990009          # e-stop or light curtain active
E_STARVED = 990010        # pushes stopped and on_push_starvation = "error"
E_UNUSED = 990011         # documented command that no tool should send
E_INJECTED_ERROR = 990012 # error injected by a test (e.g. a simulated collision stop)

MOCK_HOME_DEG = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]        # the joint target of the manual's WayPoint example (p.139)
MOCK_HOME_POSE = [420.0, 0.0, 445.0, 180.0, 0.0, 180.0]  # the pose of the same example


@dataclass
class MockConfig:
    model: str = "MOCK"
    powered: bool = True                  # nElectrify at start: the operator powers on from the pendant
    estop: bool = False
    safety_guard: bool = False
    estop_channel_mismatch: bool = False  # ReadEmergencyInfo nESTO_Error = 1
    q0_deg: list = field(default_factory=lambda: list(MOCK_HOME_DEG))
    pose0: list = field(default_factory=lambda: list(MOCK_HOME_POSE))
    tool_frame: list = field(default_factory=lambda: [0.0] * 6)
    user_frame: list = field(default_factory=lambda: [0.0] * 6)
    frame_names: tuple = ("TCP", "Base", "CUPRIM")
    # the manual's read-back examples (elfin_commands.md, PROTO p.56-59); mock values
    joint_max_vel: list = field(default_factory=lambda: [170.0] * 4 + [180.0] * 2)
    joint_max_acc: list = field(default_factory=lambda: [360.0] * 6)
    joint_max_jerk: list = field(default_factory=lambda: [3600.0] * 6)
    linear_max: list = field(default_factory=lambda: [1000.0, 2500.0, 10000000.0])
    # mock joint ranges; the real ones are [UNCLEAR] (elfin_overview.md)
    joint_ranges: list = field(default_factory=lambda: [(-360, 360), (-135, 135), (-153, 153),
                                                        (-360, 360), (-360, 360), (-360, 360)])
    enable_delay_s: float = 0.2
    motion_start_delay_s: float = 0.02    # how soon nMovingState turns 1 is [MISSING]
    # servo tracking: the setpoint ramps to each push over one period, starting servo_delay_s
    # after the push arrives, and the joints follow it with a first-order lag
    servo_delay_s: float = 0.05
    servo_lag_s: float = 0.03
    servo_moving_state: str = "tracking"  # tracking | always1 | always0
    on_push_starvation: str = "hold"      # hold | error
    starvation_periods: float = 3.0
    grpstop_ends_servo: bool = True
    late_push_factor: float = 1.5         # an interval above this x T counts as late (mock statistic)
    # replies
    reply_delay_s: float = 0.002
    reply_jitter_s: float = 0.001
    fragment_replies: bool = True
    datasheet_byte_order: str = "<"
    datasheet_rate_hz: float = 20.0
    seed: int = 1
    # fault injection; occurrence counts are 1-based per command name
    fail_rules: list = field(default_factory=list)         # [(name, occurrence, code)]
    late_reply_rules: list = field(default_factory=list)   # [(name, occurrence, extra_s)]
    drop_after_messages: int = 0                           # close the connection after N messages (0 = never)
    error_after_pushes: tuple = ()                         # (n, code, axis): error on the n-th push


def mirror_config(arm_cfg):
    """MockConfig keyword arguments that make the mock report what e03pro.yaml expects: the limits
    read back from the arm, the mounting's tool and user frames, and the pendant's joint ranges, where
    the file has them. A rehearsal (dry run or mock) of an arm run then takes the arm run's code path
    instead of stopping at the read-back check. Null entries keep the mock's own values, so a tool
    still refuses when the file lacks them. The mock still tests our framing and reactions, not the arm."""
    out = {}
    lr = arm_cfg.get("limits_readback") or {}
    for key, name in (("joint_max_vel_deg_s", "joint_max_vel"), ("joint_max_acc_deg_s2", "joint_max_acc"),
                      ("joint_max_jerk_deg_s3", "joint_max_jerk"), ("linear_max", "linear_max")):
        if lr.get(key) is not None:
            out[name] = [float(x) for x in lr[key]]
    m = arm_cfg.get("mounting") or {}
    for key in ("tool_frame", "user_frame"):
        if m.get(key) is not None:
            out[key] = [float(x) for x in m[key]]
    if arm_cfg.get("joint_ranges_deg") is not None:
        out["joint_ranges"] = [(float(lo), float(hi)) for lo, hi in arm_cfg["joint_ranges_deg"]]
    return out


class MockArm:
    """The controller's state and its replies. Thread-safe; time comes from `clock`."""

    def __init__(self, config=None, clock=time.monotonic):
        self.cfg = config or MockConfig()
        self.clock = clock
        self.lock = threading.RLock()
        self.rng = random.Random(self.cfg.seed)
        c = self.cfg
        self.powered = c.powered
        self.enabled = False
        self.enable_at = None
        self.error_code, self.error_axis = 0, 0
        self.q = list(c.q0_deg)
        self.qd = [0.0] * 6
        self.pose = list(c.pose0)
        self.motion = None          # dict for a WayPoint move
        self.servo = None           # dict while in servo mode
        self.settling = None        # setpoint after GrpStop ended a servo stream
        self.last_t = self.clock()
        self.counts = {}            # per command name
        self.stats = {"messages": 0, "format_errors": 0, "forbidden": 0, "unused": 0, "pushes": 0,
                      "late_pushes": 0, "push_intervals": [], "dropped_extra_messages": 0}
        self.log = []               # (t, message, reply) for tests

    # ------------------------------------------------------------------ simulation
    def _advance(self, now):
        dt_total = now - self.last_t
        if dt_total <= 0:
            return
        if self.enable_at is not None and now >= self.enable_at:
            self.enabled, self.enable_at = True, None
        if self.motion is None and self.servo is None and self.settling is None:
            self.qd = [0.0] * 6
            self.last_t = now
            return
        steps = max(1, int(math.ceil(dt_total / 0.001)))
        dt = dt_total / steps
        q_prev = list(self.q)
        for i in range(1, steps + 1):
            t = self.last_t + i * dt
            if self.motion is not None:
                self._step_waypoint(t)
            elif self.servo is not None:
                self._step_servo(t, dt)
            elif self.settling is not None:
                self._step_lag(self.settling, dt)
                if max(abs(a - b) for a, b in zip(self.q, self.settling)) < 1e-4:
                    self.settling = None
        self.qd = [(a - b) / dt_total for a, b in zip(self.q, q_prev)]
        self.last_t = now

    def _step_waypoint(self, t):
        m = self.motion
        tau = t - m["t0"]
        if tau < 0:
            return
        s = _trapezoid(tau, m["dist"], m["v"], m["a"]) / m["dist"] if m["dist"] > 0 else 1.0
        vec = "q" if m["kind"] == "joint" else "pose"
        cur = [a + s * (b - a) for a, b in zip(m["start"], m["end"])]
        setattr(self, vec, cur)
        if tau >= m["T"]:
            setattr(self, vec, list(m["end"]))
            self.motion = None

    def _step_lag(self, target, dt):
        k = min(1.0, dt / self.cfg.servo_lag_s) if self.cfg.servo_lag_s > 0 else 1.0
        self.q = [q + k * (r - q) for q, r in zip(self.q, target)]

    def _step_servo(self, t, dt):
        s = self.servo
        while s["pending"] and s["pending"][0][0] + self.cfg.servo_delay_s <= t:
            _, target = s["pending"].pop(0)
            s["sp_from"], s["sp_to"], s["sp_t0"] = self._setpoint(t), target, t
        sp = self._setpoint(t)
        self._step_lag(sp, dt)
        if (self.cfg.on_push_starvation == "error" and s["last_push"] is not None
                and t - s["last_push"] > self.cfg.starvation_periods * s["T"]):
            self._set_error(E_STARVED, 0)

    def _setpoint(self, t):
        s = self.servo
        if s["sp_to"] is None:
            return list(self.q)
        f = min(1.0, max(0.0, (t - s["sp_t0"]) / s["T"]))
        return [a + f * (b - a) for a, b in zip(s["sp_from"], s["sp_to"])]

    def _set_error(self, code, axis):
        self.error_code, self.error_axis = code, axis
        self.motion = None
        if self.servo is not None:
            self.settling = None
            self.servo = None

    # ------------------------------------------------------------------ state views
    def moving(self):
        if self.motion is not None:
            return 1 if self.clock() >= self.motion["t0"] else 0
        if self.servo is not None:
            mode = self.cfg.servo_moving_state
            if mode == "always1":
                return 1
            if mode == "always0":
                return 0
            s = self.servo
            recent = s["last_push"] is not None and self.clock() - s["last_push"] < 2 * s["T"]
            sp = self._setpoint(self.clock())
            return 1 if recent or max(abs(a - b) for a, b in zip(self.q, sp)) > 1e-4 else 0
        if self.settling is not None:
            return 1
        return 0

    def robot_state(self):
        err = 1 if self.error_code else 0
        moving = self.moving()
        return [moving, 1 if self.enabled else 0, err, self.error_code, self.error_axis,
                1 if self.enabled else 0, 0, 1 if self.cfg.estop else 0, 1 if self.cfg.safety_guard else 0,
                1 if self.powered else 0, 1, 0 if (self.motion is not None) else 1, 0 if moving else 1]

    # ------------------------------------------------------------------ messages
    def handle(self, text):
        """One complete message in, one reply out (both str)."""
        with self.lock:
            now = self.clock()
            self._advance(now)
            self.stats["messages"] += 1
            reply = self._handle(text, now)
            self.log.append((now, text, reply))
            return reply

    def _fail(self, name, code):
        return f"{name},Fail,{code},;"

    def _ok(self, name, values=()):
        return ",".join([name, "OK", *values]) + ",;"

    def _handle(self, text, now):
        try:
            name, args = protocol.decode_message(text)
        except protocol.ProtocolError:
            self.stats["format_errors"] += 1
            head = text.split(",", 1)[0] or "Unknown"
            return self._fail(head, E_FORMAT)
        cmd = protocol.COMMANDS[name]
        self.counts[name] = n = self.counts.get(name, 0) + 1
        for rule_name, occurrence, code in self.cfg.fail_rules:
            if rule_name == name and occurrence == n:
                return self._fail(name, code)
        if cmd.use == protocol.FORBIDDEN:
            self.stats["forbidden"] += 1
            return self._fail(name, E_FORBIDDEN)
        if cmd.use == protocol.UNUSED:
            self.stats["unused"] += 1
            return self._fail(name, E_UNUSED)
        handler = getattr(self, "_cmd_" + name, None)
        if handler is not None:
            return handler(name, args, now)
        return self._read(name)

    def _read(self, name):
        f3 = lambda xs: [f"{x:.3f}" for x in xs]
        c = self.cfg
        values = {
            "ReadControllerState": ["1"],
            "ReadRobotModel": [c.model],
            "IsSimulation": ["1"],
            "ReadRobotState": [str(v) for v in self.robot_state()],
            "ReadActPos": f3(self.q + self.pose + c.tool_frame + c.user_frame),
            "ReadCmdPos": f3(self._cmd_q() + self.pose),
            "ReadCmdJointVel": f3(self.qd), "ReadActJointVel": f3(self.qd),
            "ReadCmdTcpVel": f3([0.0] * 6), "ReadActTcpVel": f3([0.0] * 6),
            "ReadCmdJointCur": f3([0.0] * 6), "ReadActJointCur": f3([0.0] * 6),
            "ReadJointMaxVel": f3(c.joint_max_vel), "ReadJointMaxAcc": f3(c.joint_max_acc),
            "ReadJointMaxJerk": f3(c.joint_max_jerk), "ReadLinearMaxVel": f3(c.linear_max),
            "ReadEmergencyInfo": ["1" if c.estop_channel_mismatch else "0", "1" if c.estop else "0",
                                  "0", "1" if c.safety_guard else "0"],
            "ReadAxisErrorCode": [str(self.error_code)] + [str(self.error_code if self.error_axis == i else 0)
                                                           for i in range(6)],
            "ReadCurFSM": ["0"],
        }
        return self._ok(name, values[name])

    def _cmd_q(self):
        if self.servo is not None and self.servo["sp_to"] is not None:
            return list(self.servo["sp_to"])
        if self.motion is not None and self.motion["kind"] == "joint":
            return list(self.motion["end"])
        return list(self.q)

    def _motion_allowed(self, name):
        if self.cfg.estop or self.cfg.safety_guard:
            return self._fail(name, E_ESTOP)
        if self.error_code:
            return self._fail(name, E_ERROR_ACTIVE)
        if not self.enabled:
            return self._fail(name, E_NOT_ENABLED)
        return None

    def _cmd_GrpEnable(self, name, args, now):
        if not self.powered:
            return self._fail(name, E_NOT_ENABLED)
        if self.cfg.estop or self.cfg.safety_guard:
            return self._fail(name, E_ESTOP)
        if not self.enabled and self.enable_at is None:
            self.enable_at = now + self.cfg.enable_delay_s
        return self._ok(name)

    def _cmd_GrpDisable(self, name, args, now):
        self.enabled, self.enable_at = False, None
        self.motion, self.servo, self.settling = None, None, None
        return self._ok(name)

    def _cmd_GrpReset(self, name, args, now):
        if self.cfg.estop:
            return self._fail(name, E_ESTOP)
        self.error_code, self.error_axis = 0, 0
        return self._ok(name)

    def _cmd_GrpStop(self, name, args, now):
        self.motion = None
        if self.servo is not None and self.cfg.grpstop_ends_servo:
            self.settling = self._setpoint(now)
            self.servo = None
        return self._ok(name)

    def _cmd_WayPoint(self, name, args, now):
        bad = self._motion_allowed(name)
        if bad:
            return bad
        if self.motion is not None or self.servo is not None or self.settling is not None:
            return self._fail(name, E_BUSY)
        (_, x, y, z, rx, ry, rz, j1, j2, j3, j4, j5, j6, tcp, ucs, vel, acc, radius,
         move_type, use_joint, seek, io_bit, io_state, cmd_id) = args
        if tcp not in self.cfg.frame_names or ucs not in self.cfg.frame_names:
            return self._fail(name, E_LIMIT)
        if move_type == 0:
            if use_joint != 1:
                return self._fail(name, E_UNSUPPORTED)          # IK is not simulated
            if not (0 < vel <= 200 and 0 < acc <= 720):          # protocol ranges, PROTO p.49-50
                return self._fail(name, E_LIMIT)
            end = [j1, j2, j3, j4, j5, j6]
            if any(not (lo <= v <= hi) for v, (lo, hi) in zip(end, self.cfg.joint_ranges)):
                return self._fail(name, E_LIMIT)
            start, kind = list(self.q), "joint"
            dist = max(abs(a - b) for a, b in zip(start, end))
        elif move_type == 1:
            if not (0 < vel <= 3000 and 0 < acc <= 2500):        # protocol ranges, PROTO p.51-52
                return self._fail(name, E_LIMIT)
            end = [x, y, z, rx, ry, rz]
            start, kind = list(self.pose), "linear"
            dist = math.dist(start[:3], end[:3]) or max(abs(a - b) for a, b in zip(start[3:], end[3:]))
        else:
            return self._fail(name, E_UNSUPPORTED)
        T = _trapezoid_duration(dist, vel, acc)
        self.motion = {"kind": kind, "start": start, "end": end, "dist": dist, "v": vel, "a": acc,
                       "t0": now + self.cfg.motion_start_delay_s, "T": T}
        return self._ok(name)

    def _cmd_StartServo(self, name, args, now):
        bad = self._motion_allowed(name)
        if bad:
            return bad
        _, period, lookahead = args
        if period <= 0 or lookahead <= 0:
            return self._fail(name, E_LIMIT)
        if self.motion is not None:
            return self._fail(name, E_BUSY)
        self.settling = None
        self.servo = {"T": period, "lookahead": lookahead, "pending": [], "sp_from": None, "sp_to": None,
                      "sp_t0": now, "last_push": None}
        return self._ok(name)

    def _cmd_PushServoJ(self, name, args, now):
        bad = self._motion_allowed(name)
        if bad:
            return bad
        if self.servo is None:
            return self._fail(name, E_NOT_SERVO)
        target = list(args[1:])
        if any(not (lo <= v <= hi) for v, (lo, hi) in zip(target, self.cfg.joint_ranges)):
            return self._fail(name, E_LIMIT)
        s = self.servo
        ref = s["pending"][-1][1] if s["pending"] else (s["sp_to"] or self.q)
        if max(abs(a - b) for a, b in zip(target, ref)) / s["T"] > 200.0:   # protocol joint-speed range
            return self._fail(name, E_LIMIT)
        self.stats["pushes"] += 1
        if s["last_push"] is not None:
            interval = now - s["last_push"]
            self.stats["push_intervals"].append(interval)
            if interval > self.cfg.late_push_factor * s["T"]:
                self.stats["late_pushes"] += 1
        s["last_push"] = now
        s["pending"].append((now, target))
        inj = self.cfg.error_after_pushes
        if inj and self.stats["pushes"] == inj[0]:
            self._set_error(inj[1], inj[2])
        return self._ok(name)

    # ------------------------------------------------------------------ fault hooks
    def reply_extra_delay(self, text):
        name = text.split(",", 1)[0]
        n = self.counts.get(name, 0)
        extra = sum(d for rn, occ, d in self.cfg.late_reply_rules if rn == name and occ == n)
        return extra + self.cfg.reply_delay_s + self.rng.uniform(0, self.cfg.reply_jitter_s)

    def set_estop(self, active):
        """E-stop cuts the arm's 48 V; after a reset the arm is unpowered (elfin_overview.md,
        PROTO p.60, LINUX p.122)."""
        with self.lock:
            self._advance(self.clock())
            self.cfg.estop = bool(active)
            if active:
                self.enabled, self.enable_at, self.powered = False, None, False
                self._set_error(E_ESTOP, 0)

    def power_on_from_pendant(self):
        with self.lock:
            self.powered = True

    def inject_error(self, code=E_INJECTED_ERROR, axis=0):
        with self.lock:
            self._advance(self.clock())
            self._set_error(code, axis)

    # ------------------------------------------------------------------ Datasheet
    def datasheet_packet(self):
        """One Datasheet packet in the manual's format and with its quirks (elfin_commands.md,
        PROTO p.185-188): RBTL, total length, data length, then JSON with numbers under
        PosAndVel as strings and a trailing comma after "nAxisGroupErrorCode":[0]. Only the keys
        that elfin_commands.md lists are sent; the real packets' keys are checked in week 1."""
        with self.lock:
            self._advance(self.clock())
            st = self.robot_state()
            s3 = lambda xs: [f"{x:.3f}" for x in xs]
            pv = {"Actual_Position": s3(self.q + self.pose), "Actual_PCS_Base": s3(self.pose),
                  "Actual_Joint_Velocity": s3(self.qd), "Actual_Joint_Acceleration": s3([0.0] * 6),
                  "Actual_Joint_Current": s3([0.0] * 6), "Actual_Override": "1.000"}
            se = {"robotState": 0, "robotEnabled": st[1], "robotPaused": st[6], "robotMoving": st[0],
                  "robotBlendingDone": st[11], "InPos": st[12],
                  "Error_AxisID": (self.error_axis + 1) if self.error_code else 0,   # 1-6 here [UNCLEAR]
                  "Error_Code": self.error_code, "BrakeState": [1 if self.enabled else 0] * 6}
            hl = {"EtherCAT_FramesPerSecond": 1000, "EtherCAT_TotalLostFrame": 0}
        se_text = json.dumps(se, separators=(",", ":"))[:-1] + ',"nAxisGroupErrorCode":[0],}'
        body = ('{"PosAndVel":' + json.dumps(pv, separators=(",", ":")) + ',"StateAndError":' + se_text
                + ',"HardLoad":' + json.dumps(hl, separators=(",", ":")) + "}").encode("utf-8")
        head = b"RBTL" + struct.pack(self.cfg.datasheet_byte_order + "II", len(body) + 12, len(body))
        return head + body


def _trapezoid_duration(dist, v, a):
    if dist <= 0:
        return 0.0
    if dist >= v * v / a:
        return dist / v + v / a
    return 2.0 * math.sqrt(dist / a)


def _trapezoid(t, dist, v, a):
    """Distance travelled at time t on a rest-to-rest trapezoid."""
    T = _trapezoid_duration(dist, v, a)
    if t >= T:
        return dist
    if dist >= v * v / a:
        ta = v / a
        if t < ta:
            return 0.5 * a * t * t
        if t < T - ta:
            return 0.5 * a * ta * ta + v * (t - ta)
        r = T - t
        return dist - 0.5 * a * r * r
    ta = T / 2
    if t < ta:
        return 0.5 * a * t * t
    r = T - t
    return dist - 0.5 * a * r * r
