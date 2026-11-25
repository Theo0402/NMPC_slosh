"""Arm control for the workbench: jog with buttons, and move through start, middle and end points.

    Started by the workbench's Arm control tab:  python -m tools.elfin_py.jog --dry-run | --mock | --arm

Every move is a linear WayPoint (nMoveType 1) with the orientation held: to the current pose plus one
step, or to a recorded point (elfin_commands.md, PROTO p.137-139). Speed and acceleration are
jog_speed_mm_s and jog_acc_mm_s2, checked against confirmed.yaml before anything is sent; the tool
refuses, it never clips (codebase plan §3.8 item 5). MoveRelL (PROTO p.130) is not used: it takes no
speed or acceleration, so its peak speed could not be checked, and its unit is [UNCLEAR].

Rules (codebase plan §3.8): one message at a time; the checks of item 2 before every move; each move
waits for nMovingState = 0 (PROTO p.189-191); the frames must match e03pro.yaml before a linear move;
a Fail ends the session, and GrpDisable runs on every exit. No move leaves the jog box round the pose
where this session enabled the arm, and a recorded point must have the session's orientation, so the
cup keeps the orientation it had at the start (the operator confirms it is upright).

Workspace box (owner request, 2026-11-25): workspace_box_mm = [x_min, x_max, y_min, y_max, z_min, z_max] is a fixed
cuboid in the coordinates ReadActPos reports (PROTO p.66-67), taught on the Arm control tab by jogging to the corners
of the free space. No move may end outside it. Moves are straight lines, so a move between two points inside the box
stays inside it. If the arm is outside the box when a move is asked for, a move is allowed only if it ends no further
outside on any axis (so the operator can bring it back in); a path's points must all be inside. The box bounds the
TCP point only: the cup holder's size and the elbow are not checked, so teach the corners with that in mind.

The stop button (or a dead workbench) during a move sends GrpStop (PROTO p.20; what it does on this arm
is [UNTESTED]), waits for the arm to settle, and ends the session. Any other error during a move (no reply,
a protocol or socket error) ends the session the same way: the exit sequence sends GrpStop before GrpDisable
whenever a move may be under way (link.Arm.exit_sequence). The e-stop stays within reach.

Idle timeout (owner request): with no command for jog_idle_timeout_s (5 minutes by default [proposal]), the session
ends by itself and GrpDisable is sent, so the arm is never left enabled with nobody at the page. Every pose report
carries the seconds left; the page warns in the last minute and offers "Keep connected" (the keep command). All of
the session's questions are asked before the arm is enabled, so an unanswered question never leaves it enabled.

Commands on the channel ({"type": "cmd", "id", "name", "args"}):
    jog   {"axis": "x"|"y"|"z", "dir": 1|-1, "step_mm": float}
    goto  {"point": {"name", "pose": [6], "tool": [6], "user": [6]}}
    path  {"points": [point, ...], "dwell_s": float}      current pose -> each point in turn
    keep  {}                                              no motion: restarts the idle timer
    end   {}
"""
import math
import signal
import sys
import threading
import time

from tools.elfin_py import cli, link, motion, triallog
from tools.elfin_py.elfin_min import ElfinError

FIRST_RUN = ("Jog moves are linear, at the jog speed (10 mm/s by default), orientation held. Watch the first\n"
             "step of each axis: the arm must move the way the arrow says, by the step shown. Hit the e-stop if it\n"
             "moves the wrong way, rotates the cup, or keeps going.")
DEFAULTS = {"jog_speed_mm_s": 10.0, "jog_acc_mm_s2": 50.0, "jog_max_step_mm": 50.0,
            "jog_box_mm": [200.0, 200.0, 150.0], "path_dwell_s": 2.0,
            "jog_idle_timeout_s": 300.0, "workspace_box_mm": None}            # as config/params.schema.yaml
AXES = {"x": 0, "y": 1, "z": 2}
ORIENT_TOL_DEG = 0.5
IDLE_POLL_S = 0.5


class CommandRefused(ValueError):
    """This one command is refused; the session goes on."""


def _number(x, what):
    """A finite number, or this command is refused: a malformed command never ends the session (pre-hardware
    review I4)."""
    if isinstance(x, bool):
        raise CommandRefused(f"{what}: a number is needed, not {x!r}")
    try:
        v = float(x)
    except (TypeError, ValueError):
        raise CommandRefused(f"{what}: a number is needed, not {x!r}") from None
    if not math.isfinite(v):
        raise CommandRefused(f"{what}: {v} is not a finite number")
    return v


def _vector(x, what, n=6):
    if not isinstance(x, (list, tuple)) or len(x) != n:
        raise CommandRefused(f"{what}: {n} numbers are needed, not {x!r}")
    return [_number(v, f"{what}[{i}]") for i, v in enumerate(x)]


def check_settings(s):
    """The session's settings, checked once before anything is enabled; a bad value refuses the session."""
    problems = []
    for k in ("jog_speed_mm_s", "jog_acc_mm_s2", "jog_max_step_mm", "jog_idle_timeout_s"):
        try:
            if not _number(s[k], k) > 0:
                problems.append(f"{k} must be positive, not {s[k]!r}")
        except CommandRefused as exc:
            problems.append(str(exc))
    try:
        box = _vector(s["jog_box_mm"], "jog_box_mm", 3)
        if min(box) <= 0:
            problems.append(f"jog_box_mm needs 3 positive values, not {s['jog_box_mm']!r}")
    except CommandRefused as exc:
        problems.append(str(exc))
    try:
        if _number(s["path_dwell_s"], "path_dwell_s") < 0:
            problems.append("path_dwell_s must not be negative")
    except CommandRefused as exc:
        problems.append(str(exc))
    if s.get("workspace_box_mm") is not None:
        try:
            ws = _vector(s["workspace_box_mm"], "workspace_box_mm", 6)
            for i, name in enumerate("XYZ"):
                if not ws[2 * i] < ws[2 * i + 1]:
                    problems.append(f"workspace_box_mm: {name} min {ws[2 * i]} must be below {name} max {ws[2 * i + 1]}")
        except CommandRefused as exc:
            problems.append(str(exc))
    if problems:
        raise link.Refused("the Arm control settings are not usable: " + "; ".join(problems) +
                           " (the Coefficients tab)")


def settings(block=None):
    coeffs = (block or {}).get("coeffs") or {}            # {name: {"value", "source", "note"}}
    out = {}
    for k, v in DEFAULTS.items():
        entry = coeffs.get(k)
        val = entry.get("value") if isinstance(entry, dict) else entry
        out[k] = v if val is None else val
    return out


def angle_diff(a, b):
    return (a - b + 180.0) % 360.0 - 180.0


def outside_by(ws, xyz):
    """How far each of X, Y, Z lies outside the workspace box ws = [x_min, x_max, ...] (0 inside), in mm."""
    return [max(ws[2 * i] - xyz[i], xyz[i] - ws[2 * i + 1], 0.0) for i in range(3)]


def check_workspace(ws, target, now_xyz=None):
    """Refuse a move that ends outside the workspace box. With now_xyz (where the arm is), a target outside is
    allowed only if it is no further outside on any axis, so an arm outside the box can be brought back in."""
    if ws is None:
        return
    out_t = outside_by(ws, target)
    out_n = outside_by(ws, now_xyz) if now_xyz is not None else [0.0, 0.0, 0.0]
    for i, name in enumerate("XYZ"):
        if out_t[i] > out_n[i] + 1e-6:
            lo, hi = ws[2 * i], ws[2 * i + 1]
            raise CommandRefused(f"{name} {target[i]:.1f} mm is outside the workspace box ({name} {lo:.1f} to {hi:.1f} mm, "
                                 "workspace_box_mm)" + (f"; the arm is outside it now, so only moves back towards it are "
                                                        "allowed" if any(out_n) else ""))


class Session:
    def __init__(self, arm, log, chan, mounting, s):
        self.arm, self.log, self.chan, self.m, self.s = arm, log, chan, mounting, s
        self.home = None
        self.moving = False
        self.last_activity = time.monotonic()

    def idle_left(self):
        return self.s["jog_idle_timeout_s"] - (time.monotonic() - self.last_activity)

    def read(self):
        joints, pose, tool, user = self.arm.act_pos()
        return {"joints": joints, "pose": pose, "tool": tool, "user": user}

    def emit_pose(self, kind="pose"):
        p = self.read()
        self.chan.emit(kind, moving=self.moving, home=self.home, idle_left_s=round(self.idle_left(), 1), **p)
        return p

    def check_target(self, target, now, inside=False):
        """inside=True: the target must be inside the workspace box even if the arm is outside it now (paths)."""
        if self.home is None:
            raise CommandRefused("the session has not started")
        check_workspace(self.s.get("workspace_box_mm"), target, None if inside else now["pose"][:3])
        box = self.s["jog_box_mm"]
        for i, name in enumerate("XYZ"):
            if abs(target[i] - self.home[i]) > box[i] + 1e-9:
                raise CommandRefused(f"{name} {target[i]:.1f} mm is {abs(target[i] - self.home[i]):.1f} mm from where the "
                                     f"session started; the jog box allows {box[i]:.0f} mm (jog_box_mm)")
        for i, name in zip(range(3, 6), ("Rx", "Ry", "Rz")):
            if abs(angle_diff(target[i], now["pose"][i])) > ORIENT_TOL_DEG:
                raise CommandRefused(f"{name} of the target differs from the current one by more than "
                                     f"{ORIENT_TOL_DEG} deg: the cup would rotate")

    def move_to(self, target, cmd_id):
        """One linear move, orientation held, then wait for it to finish."""
        self.arm.check_before_motion()                     # codebase plan §3.8 item 2
        now = self.read()
        problems = link.frame_problems(now["tool"], now["user"], self.m)
        if problems:
            raise link.Refused("frames differ from e03pro.yaml: " + "; ".join(problems))
        self.check_target(target, now)
        dist = math.dist(target[:3], now["pose"][:3])
        if dist < 1e-6:
            return now
        v, a = self.s["jog_speed_mm_s"], self.s["jog_acc_mm_s2"]
        self.log.event(triallog.EV_PHASE, {"move": cmd_id, "from": now["pose"], "to": target, "v": v, "a": a})
        self.moving = True
        self.chan.emit("pose", moving=True, home=self.home, target=target, **now)
        self.arm.cmd("WayPoint", 0, *target, *now["joints"], self.m["tcp_name"], self.m["ucs_name"],
                     float(v), float(a), 0.0, 1, 0, 0, 0, 0, str(cmd_id))
        self.arm.wait_done(timeout=dist / v + 2 * v / a + 10.0, pose=target)    # review S12
        self.moving = False
        return self.emit_pose()

    def point_target(self, point, now):
        if not isinstance(point, dict):
            raise CommandRefused(f"a point is an object with name, pose, tool and user, not {point!r}")
        pose = _vector(point.get("pose"), f"point {point.get('name')!r} pose")
        for k in ("tool", "user"):
            if point.get(k) is not None:
                frame = _vector(point[k], f"point {point.get('name')!r} {k} frame")
                if any(abs(x - y) > 1e-3 for x, y in zip(frame, now[k])):
                    raise CommandRefused(f"point {point.get('name')!r} was recorded with another {k} frame")
        return pose

    def handle(self, msg):
        name, args, cid = msg.get("name"), msg.get("args") or {}, msg.get("id")
        if not isinstance(args, dict):
            raise CommandRefused(f"the arguments of {name!r} are an object, not {args!r}")
        if name == "jog":
            axis, d = AXES.get(args.get("axis")), args.get("dir")
            if axis is None or isinstance(d, bool) or d not in (1, -1):
                raise CommandRefused("jog needs axis x, y or z and dir 1 or -1")
            step = _number(args.get("step_mm"), "step_mm")
            if not 0 < step <= self.s["jog_max_step_mm"]:
                raise CommandRefused(f"step {step} mm: allowed up to {self.s['jog_max_step_mm']} mm (jog_max_step_mm)")
            now = self.read()
            target = list(now["pose"])
            target[axis] += d * step
            self.move_to(target, cid)
        elif name == "goto":
            now = self.read()
            self.move_to(self.point_target(args.get("point"), now), cid)
        elif name == "path":
            points = args.get("points") or []
            if not isinstance(points, list) or not points:
                raise CommandRefused("the path has no points")
            dwell = _number(args.get("dwell_s", self.s["path_dwell_s"]), "dwell_s")
            if dwell < 0:
                raise CommandRefused(f"dwell_s {dwell} s: must not be negative")
            now = self.read()
            targets = [self.point_target(p, now) for p in points]
            for t in targets:                            # check every point before the first move
                self.check_target(t, now, inside=True)
            for k, (p, t) in enumerate(zip(points, targets), 1):
                self.chan.emit("progress", id=cid, step=k, of=len(points), point=p.get("name"))
                self.move_to(t, f"{cid}.{k}")
                t_end = time.monotonic() + dwell
                while time.monotonic() < t_end:
                    time.sleep(0.05)
        elif name == "keep":
            pass                                       # the idle timer restarts below, as after any command
        elif name == "end":
            return False
        else:
            raise CommandRefused(f"unknown command {name!r}")
        return True

    def loop(self):
        last_poll = 0.0
        while True:
            if self.chan.stopped:               # normally the interrupt arrives first; never idle on after a stop
                raise KeyboardInterrupt
            msg = self.chan.next_command(timeout=0.1)
            if msg is None:
                if self.idle_left() <= 0:
                    t = self.s["jog_idle_timeout_s"]
                    self.log.event(triallog.EV_MARKER, f"idle timeout: no command for {t:g} s; session ended, GrpDisable")
                    self.chan.emit("idle_timeout", after_s=t)
                    print(f"IDLE TIMEOUT: no command for {t:g} s; ending the session and disabling the arm", flush=True)
                    return
                if time.monotonic() - last_poll > IDLE_POLL_S:
                    self.emit_pose()
                    last_poll = time.monotonic()
                continue
            cid = msg.get("id")
            try:
                self.chan.emit("busy", id=cid, name=msg.get("name"))
                go_on = self.handle(msg)
                self.chan.emit("done", id=cid, ok=True)
            except CommandRefused as exc:
                self.log.event(triallog.EV_REFUSAL, f"command {msg.get('name')} refused: {exc}")
                self.chan.emit("done", id=cid, ok=False, error=str(exc))
                go_on = True
            self.last_activity = time.monotonic()      # a command, even a refused one, means someone is there
            if not go_on:
                return

    def stop_motion(self):
        """After a stop request: read any reply in flight; GrpStop and a wait for rest if a move was under way
        (link.Arm.stop_motion). Any other error is left to the exit sequence, which does the same."""
        if threading.current_thread() is threading.main_thread():
            signal.signal(signal.SIGINT, signal.SIG_IGN)         # the e-stop is the next step, not a second Ctrl-C
        try:
            if self.moving or self.arm.motion_may_run:
                self.arm.stop_motion("stop request during a move")
            else:
                self.arm.drain()
        finally:
            self.moving = False


def main(argv=None):
    ap = cli.parser(__doc__)
    args = ap.parse_args(argv)
    with cli.session(args, "jog", "scratch", FIRST_RUN) as (arm, log, confirmed, arm_cfg, ask):
        chan = args.channel
        if chan is None:
            raise link.Refused("the jog tool takes its commands from the workbench: python -m tools.workbench, Arm control")
        s = settings(cli.workbench_block() if chan else None)
        check_settings(s)                          # before anything is enabled (review I4)
        motion.check_waypoint("linear", s["jog_speed_mm_s"], s["jog_acc_mm_s2"], confirmed)
        log.event(triallog.EV_INFO, {"jog_settings": s})
        m = arm_cfg["mounting"]
        stopped = None
        sess = Session(arm, log, chan, m, s)
        # ReadActPos (PROTO p.66-67) before enabling: whether reads need the arm enabled is not stated in the
        # knowledge files [UNTESTED]; bring-up reads the same way without enabling (review I5: the citation was wrong)
        now = sess.read()
        problems = link.frame_problems(now["tool"], now["user"], m)
        if problems:
            raise link.Refused("frames differ from e03pro.yaml: " + "; ".join(problems))
        # every question before the arm is enabled: an unanswered one never leaves it enabled
        if not ask("Is the cup upright and clamped? Every jog and point move keeps the orientation it has now."):
            raise link.Refused("operator did not confirm the cup's orientation")
        with arm.disable_on_exit():
            arm.prepare_and_enable(arm_cfg, ask)
            now = sess.read()                      # after enabling ("sound and slight movement", LINUX p.17-18)
            sess.home = list(now["pose"])
            sess.last_activity = time.monotonic()
            chan.emit("ready", settings=s, confirmed={k: confirmed[k] for k in ("linear_speed_mm_s", "linear_acc_mm_s2")},
                      tcp=m["tcp_name"], ucs=m["ucs_name"], **now, home=sess.home)
            try:
                sess.loop()
            except KeyboardInterrupt:              # the stop button, a closed channel, or Ctrl-C
                sess.stop_motion()
                stopped = cli.stop_reason(chan)
            except ElfinError:
                sess.moving = False
                raise
        if stopped:                                # GrpDisable has been sent by now
            log.event(triallog.EV_MARKER, f"stopped: {stopped}")
            print(f"STOPPED ({stopped}): GrpStop if a move was under way, then GrpDisable", flush=True)
            return 2
        return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        sys.exit(f"stopped: {exc}")
