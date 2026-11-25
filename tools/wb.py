"""The workbench from the command line, no browser (owner request, 2026-11-25). Standard library only.

    python -m tools.wb up                     start the workbench in the background (if it is not running)
    python -m tools.wb down                   stop it (a running tool gets its stop first)
    python -m tools.wb status                 the current run, its questions and the arm's pose
    python -m tools.wb stages                 every stage with its status per mode
    python -m tools.wb run STAGE MODE [--override REASON] [--note TEXT] [--practice] [--yes] [--opt KEY=VALUE ...]
    python -m tools.wb watch [--timeout S]    follow the run until it asks something, is ready, or ends
    python -m tools.wb answer ID yes|no       answer the run's question ID
    python -m tools.wb stop                   the STOP button
    python -m tools.wb form STAGE --note TEXT [FIELD=VALUE ...] [--override REASON]
    python -m tools.wb accept RUN_ID --note TEXT [CHECK=yes ...]
    python -m tools.wb coeff NAME=VALUE ... --note TEXT
Arm control (a running arm_control session):
    python -m tools.wb pose | jog AXIS MM | tilt DEG | roll DEG | goto X Y Z | keep | end
    python -m tools.wb corner | corners [--clear] | ws | ws-save --note TEXT [--margin MM] | ws-clear --note TEXT

It only talks to the workbench's API on 127.0.0.1 (tools/workbench/server.py), exactly as the page does: every check,
lock, refusal, question, run folder and journal line stays the workbench's. It never answers a question by itself.
MODE is dry-run, mock or arm (analysis for stages such as 0.1 host_check; forms use "form"); jog takes a signed step,
e.g. "jog z -5". goto moves in a straight line to X Y Z with the
orientation held (the jog tool refuses anything outside the jog box or the workspace box). tilt raises (+) or
lowers (-) the J6 axis in place, e.g. "tilt 3.2"; pose shows the J6 axis's angle from level. roll turns about the
J6 axis in place, + counterclockwise seen from in front of the flange.
"""
import argparse
import json
import os
import pathlib
import signal
import subprocess
import sys
import time
import urllib.error
import urllib.request

ROOT = pathlib.Path(__file__).resolve().parent.parent
PORT = int(os.environ.get("SLOSH_WB_PORT", "8765"))


class WbError(RuntimeError):
    pass


def data_dir():
    return pathlib.Path(os.environ.get("SLOSH_DATA") or ROOT / "data")


def api(path, body=None, timeout=30):
    url = f"http://127.0.0.1:{PORT}{path}"
    data = None if body is None else json.dumps(body).encode("utf-8")
    req = urllib.request.Request(url, data=data, method="GET" if body is None else "POST",
                                 headers={"X-Slosh-Workbench": "1", "Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return json.loads(r.read() or b"null")
    except urllib.error.HTTPError as exc:
        try:
            msg = json.loads(exc.read())
            msg = msg.get("refused") or msg.get("error") or msg
        except ValueError:
            msg = exc.reason
        raise WbError(f"REFUSED: {msg}" if exc.code == 409 else f"error {exc.code}: {msg}") from None
    except (urllib.error.URLError, ConnectionError, TimeoutError) as exc:
        raise WbError(f"the workbench does not answer on port {PORT} ({exc}): start it with  python -m tools.wb up")


def running():
    try:
        api("/api/overview", timeout=5)
        return True
    except WbError as exc:
        if "does not answer" in str(exc):
            return False
        raise


# ------------------------------------------------------------------ the workbench process
def cmd_up(a):
    if running():
        print(f"the workbench is already running on port {PORT}")
        return 0
    log = data_dir() / "workbench" / "server.log"
    log.parent.mkdir(parents=True, exist_ok=True)
    with open(log, "a", encoding="utf-8") as out:
        subprocess.Popen([sys.executable, "-u", "-m", "tools.workbench", "--port", str(PORT)], cwd=ROOT,
                         stdin=subprocess.DEVNULL, stdout=out, stderr=subprocess.STDOUT, start_new_session=True)
    for _ in range(60):
        time.sleep(0.5)
        if running():
            print(f"the workbench is running on port {PORT} (log: {log})")
            return 0
    raise WbError(f"the workbench did not start; see {log}")


def cmd_down(a):
    lock = data_dir() / "workbench" / "workbench.lock"
    if not lock.exists():
        print("no workbench holds the data folder")
        return 0
    pid = json.loads(lock.read_text(encoding="utf-8")).get("pid")
    try:
        os.kill(int(pid), signal.SIGTERM)          # handled like Ctrl-C: the running tool is stopped first
    except (ProcessLookupError, TypeError, ValueError):
        print(f"process {pid} is not running (a stale lock; the next start takes it over)")
        return 0
    for _ in range(80):
        time.sleep(0.5)
        if not lock.exists():
            print("the workbench stopped")
            return 0
    raise WbError(f"process {pid} has not stopped after 40 s")


# ------------------------------------------------------------------ runs
def current():
    return api("/api/run")


def fmt_pose(p):
    if not p or not p.get("pose"):
        return "pose unknown"
    x = p["pose"]
    import math                                       # J6 axis = tool +z; its up part is cos(Ry)cos(Rx) (F2)
    up = math.cos(math.radians(x[4])) * math.cos(math.radians(x[3]))
    return (f"X {x[0]:.1f}  Y {x[1]:.1f}  Z {x[2]:.1f} mm   Rx {x[3]:.2f}  Ry {x[4]:.2f}  Rz {x[5]:.2f} deg   "
            f"J6 axis {math.degrees(math.asin(max(-1.0, min(1.0, up)))):+.2f} deg from level")


def show_questions(r):
    for q in r.get("questions") or []:
        print(f"QUESTION {q['id']}: {q['question']}")
        print(f"   answer with:  python -m tools.wb answer {q['id']} yes   (or no)")


def show_result(r):
    rec = next((x for x in api(f"/api/stage/{r['stage']}")["runs"] if x.get("run_id") == r["run_id"]), None)
    print(f"ENDED: exit code {r['exit_code']}" + (f", criteria {'met' if rec.get('auto_ok') else 'NOT met'}" if rec else ""))
    m = (rec or {}).get("metrics") or {}
    for k in ("refusals", "errors", "stopped", "problem_list"):
        if m.get(k):
            print(f"  {k}: {m[k]}")


def cmd_status(a):
    o = api("/api/overview")
    r = o.get("run")
    print(f"host: {o['host_problem'] or 'arm mode allowed'}   coefficients v{o['coeff_version']}")
    if not r:
        print("no run yet")
        return 0
    print(f"run {r['run_id']}: {r['stage']} {r['mode']} " + ("RUNNING" if r["running"] else f"ended, exit code {r['exit_code']}"))
    arm = r.get("arm") or {}
    if r["stage"] == "arm_control" and r["running"]:
        state = "busy: " + arm["busy"]["name"] if arm.get("busy") else ("ready" if arm.get("ready") else "connecting")
        print(f"arm control: {state};  {fmt_pose(arm.get('pose'))}")
        if arm.get("ready"):
            s = arm["ready"]["settings"]
            print(f"  jog box ±{s['jog_box_mm']} mm round the start; workspace box {s.get('workspace_box_mm')}; "
                  f"workspace fan {s.get('workspace_fan')}")
    show_questions(r)
    return 0


def cmd_stages(a):
    o = api("/api/overview")
    for s in o["stages"]:
        if s["status"] != "built":
            continue
        modes = "  ".join(f"{m}:{st}" for m, st in s["per_mode"].items())
        print(f"{s['num']:>4}  {s['id']:<18} {modes}" + ("   (locked)" if s["locked"] else ""))
    return 0


def watch(timeout=120.0, since=None, until_end=False):
    """Print the run's new console lines until it asks something, an Arm control session is ready, or it ends
    (until_end: only when it ends)."""
    r = current()
    if not r:
        print("no run")
        return None
    n = r["console_len"] if since is None else since
    t_end = time.monotonic() + timeout
    while True:
        r = api(f"/api/run?since={n}")
        for line in r["console"]:
            print("  " + line)
        n = r["console_len"]
        if not r["running"]:
            show_result(r)
            return r
        if r["questions"] and not until_end:
            show_questions(r)
            return r
        arm = r.get("arm") or {}
        if r["stage"] == "arm_control" and arm.get("ready") and not arm.get("busy") and not until_end:
            print(f"READY: the arm is enabled.  {fmt_pose(arm.get('pose'))}")
            return r
        if time.monotonic() > t_end:
            print(f"still running after {timeout:g} s; follow it with  python -m tools.wb watch")
            return r
        time.sleep(0.5)


def cmd_run(a):
    opts = dict(kv.split("=", 1) for kv in a.opt or [])
    if a.yes and a.mode == "arm":
        raise WbError("--yes is for dry-run and mock only: on the arm every question goes to the operator")
    res = api("/api/run", {"stage": a.stage, "mode": a.mode, "note": a.note or "", "assume_yes": a.yes,
                           "options": opts, "override_reason": a.override, "practice": a.practice})
    print(f"started {res['run_id']}")
    watch(a.timeout, since=0)
    return 0


def cmd_watch(a):
    watch(a.timeout)
    return 0


def cmd_answer(a):
    r = current()
    if not r or not r["running"]:
        raise WbError("no run is waiting for an answer")
    if int(a.id) not in [q["id"] for q in r["questions"]]:
        raise WbError(f"question {a.id} is not open; open: {[q['id'] for q in r['questions']]}")
    api("/api/answer", {"id": int(a.id), "text": a.text, "run_id": r["run_id"]})
    print(f"answered {a.id}: {a.text}")
    watch(a.timeout)
    return 0


def cmd_stop(a):
    r = current()
    print("STOP sent" if api("/api/stop", {"run_id": r["run_id"] if r else None})["stop_sent"] else "STOP: the tool had ended")
    watch(60, until_end=True)
    return 0


def checks(pairs):
    out = {}
    for kv in pairs or []:
        k, v = kv.split("=", 1)
        out[k] = v.lower() in ("1", "yes", "true", "y", "ok") if v.lower() in (
            "1", "0", "yes", "no", "true", "false", "y", "n", "ok") else v
    return out


def cmd_form(a):
    res = api("/api/form", {"stage": a.stage, "values": checks(a.values), "note": a.note,
                            "override_reason": a.override})
    run = res["run"]
    print(f"form {a.stage}: " + ("all criteria met" if run.get("auto_ok") else "NOT passed") +
          (f"; result {res['result']}" if res.get("result") else ""))
    return 0


def cmd_accept(a):
    print(api("/api/accept", {"run_id": a.run_id, "note": a.note, "operator_checks": checks(a.checks)}))
    return 0


def cmd_coeff(a):
    changes = {k: json.loads(v) for k, v in (kv.split("=", 1) for kv in a.pairs)}
    res = api("/api/coeffs", {"changes": changes, "note": a.note})
    print(f"coefficients v{res['version']}: " + "; ".join(f"{d['name']} {d['old']} -> {d['new']}" for d in res["diff"]))
    return 0


# ------------------------------------------------------------------ arm control
def arm_ready():
    r = current()
    if not r or not r["running"] or r["stage"] != "arm_control":
        raise WbError("no Arm control session is running: python -m tools.wb run arm_control MODE")
    if not (r.get("arm") or {}).get("ready"):
        raise WbError("the session is not ready yet: python -m tools.wb watch")
    return r


def send(name, args, timeout=120.0):
    r = arm_ready()
    cid = api("/api/cmd", {"name": name, "args": args, "run_id": r["run_id"]})["id"]
    t_end = time.monotonic() + timeout
    while time.monotonic() < t_end:
        r = current()
        res = (r.get("results") or {}).get(str(cid))
        if res is not None:
            print(("done" if res.get("ok") else f"REFUSED: {res.get('error')}") + f"   {fmt_pose(r['arm'].get('pose'))}")
            return res
        if not r["running"]:
            show_result(r)
            return None
        time.sleep(0.3)
    print(f"no result after {timeout:g} s: python -m tools.wb status")
    return None


def cmd_pose(a):
    print(fmt_pose(arm_ready()["arm"].get("pose")))
    return 0


def cmd_jog(a):
    step = float(a.mm)
    if step == 0:
        raise WbError("a step of 0 mm")
    send("jog", {"axis": a.axis.lower(), "dir": 1 if step > 0 else -1, "step_mm": abs(step)})
    return 0


def cmd_turn(name):
    def run(a):
        deg = float(a.deg)
        if deg == 0:
            raise WbError(f"a {name} of 0 deg")
        send(name, {"deg": deg})
        return 0
    return run


def cmd_goto(a):
    p = arm_ready()["arm"]["pose"]
    pose = [float(a.x), float(a.y), float(a.z)] + list(p["pose"][3:])
    send("goto", {"point": {"name": "goto", "pose": pose, "tool": p["tool"], "user": p["user"]}})
    return 0


def cmd_simple(name):
    def run(a):
        send(name, {})
        if name == "end":                              # wait until the arm is disabled and the run is saved
            watch(60, until_end=True)
        return 0
    return run


CORNERS = lambda: data_dir() / "workbench" / "ws_corners.json"   # noqa: E731


def load_corners():
    p = CORNERS()
    return json.loads(p.read_text(encoding="utf-8")) if p.exists() else []


def cmd_corner(a):
    x = arm_ready()["arm"]["pose"]["pose"]
    c = load_corners() + [[round(x[0], 1), round(x[1], 1), round(x[2], 1)]]
    CORNERS().write_text(json.dumps(c), encoding="utf-8")
    print(f"corner {len(c)}: X {c[-1][0]}  Y {c[-1][1]}  Z {c[-1][2]} mm" +
          (f"   ({len(c) - 1} earlier corner(s) kept; start over with  corners --clear)" if len(c) > 1 else ""))
    return 0


def cmd_corners(a):
    if a.clear:
        CORNERS().unlink(missing_ok=True)
        print("corners cleared")
        return 0
    c = load_corners()
    for i, p in enumerate(c, 1):
        print(f"C{i}: X {p[0]}  Y {p[1]}  Z {p[2]} mm")
    if len(c) >= 2:
        print("box round them: " + "  ".join(f"{n} {min(p[i] for p in c)} to {max(p[i] for p in c)}"
                                             for i, n in enumerate("XYZ")) + " mm")
    return 0


def cmd_ws(a):
    v = api("/api/workspace")
    print(f"workspace box: {v['box']} (coefficients v{v['version']}, {v['source']})")
    return 0


def cmd_ws_save(a):
    res = api("/api/workspace", {"corners": load_corners(), "margin_mm": a.margin, "note": a.note})
    CORNERS().unlink(missing_ok=True)
    print(f"workspace box saved as coefficients v{res['version']}: {res['box']}; it applies from the next Arm control session")
    return 0


def cmd_ws_clear(a):
    res = api("/api/workspace/clear", {"note": a.note})
    print(f"workspace box removed (coefficients v{res['version']})")
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(prog="python -m tools.wb", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    for name, fn in (("up", cmd_up), ("down", cmd_down), ("status", cmd_status), ("stages", cmd_stages),
                     ("pose", cmd_pose), ("keep", cmd_simple("keep")), ("end", cmd_simple("end")),
                     ("corner", cmd_corner), ("ws", cmd_ws), ("stop", cmd_stop)):
        sub.add_parser(name).set_defaults(fn=fn)
    p = sub.add_parser("run")
    p.add_argument("stage")
    p.add_argument("mode", choices=["dry-run", "mock", "arm", "analysis"])
    p.add_argument("--override")
    p.add_argument("--note")
    p.add_argument("--practice", action="store_true")
    p.add_argument("--yes", action="store_true", help="dry-run and mock only: answer the questions yes")
    p.add_argument("--opt", action="append")
    p.add_argument("--timeout", type=float, default=120.0)
    p.set_defaults(fn=cmd_run)
    p = sub.add_parser("watch")
    p.add_argument("--timeout", type=float, default=120.0)
    p.set_defaults(fn=cmd_watch)
    p = sub.add_parser("answer")
    p.add_argument("id")
    p.add_argument("text", choices=["yes", "no"])
    p.add_argument("--timeout", type=float, default=120.0)
    p.set_defaults(fn=cmd_answer)
    p = sub.add_parser("form")
    p.add_argument("stage")
    p.add_argument("values", nargs="*")
    p.add_argument("--note", required=True)
    p.add_argument("--override")
    p.set_defaults(fn=cmd_form)
    p = sub.add_parser("accept")
    p.add_argument("run_id")
    p.add_argument("checks", nargs="*")
    p.add_argument("--note", required=True)
    p.set_defaults(fn=cmd_accept)
    p = sub.add_parser("coeff")
    p.add_argument("pairs", nargs="+")
    p.add_argument("--note", required=True)
    p.set_defaults(fn=cmd_coeff)
    p = sub.add_parser("jog")
    p.add_argument("axis", choices=["x", "y", "z", "X", "Y", "Z"])
    p.add_argument("mm")
    p.set_defaults(fn=cmd_jog)
    for name in ("tilt", "roll"):
        p = sub.add_parser(name)
        p.add_argument("deg")
        p.set_defaults(fn=cmd_turn(name))
    p = sub.add_parser("goto")
    for k in "xyz":
        p.add_argument(k)
    p.set_defaults(fn=cmd_goto)
    p = sub.add_parser("corners")
    p.add_argument("--clear", action="store_true")
    p.set_defaults(fn=cmd_corners)
    p = sub.add_parser("ws-save")
    p.add_argument("--note", required=True)
    p.add_argument("--margin", type=float, default=0.0)
    p.set_defaults(fn=cmd_ws_save)
    p = sub.add_parser("ws-clear")
    p.add_argument("--note", required=True)
    p.set_defaults(fn=cmd_ws_clear)
    a = ap.parse_args(argv)
    try:
        return a.fn(a)
    except WbError as exc:
        print(exc, file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
