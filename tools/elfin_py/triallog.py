"""A trial log for the Python tools: the same .bin format as the RT process, written through the
generated stdlib writer (codebase plan §5; owner, week 1). Host times are time.monotonic_ns();
wall-clock time appears once, in the header.

A failed write (a full disk, an I/O error) never raises into the tool (pre-hardware review S8): the log
records the error, writes nothing more (the file ends at or inside the last record, which the reader
reports as truncated), warns once on stderr and calls `on_failure`, which the tools set to a stop request.
The tool then stops as it does for the stop button, sending GrpStop and GrpDisable unlogged if it must,
and ends with a non-zero exit code ("trial log incomplete")."""
import datetime
import json
import os
import pathlib
import platform
import subprocess
import sys
import threading
import time

from analysis.logread import latest

ROOT = pathlib.Path(__file__).resolve().parents[2]
L = latest()

EV_MARKER, EV_PHASE, EV_ERROR, EV_INFO, EV_ARM_BLOCK, EV_SUMMARY, EV_REFUSAL = 1, 2, 3, 4, 5, 6, 7
DATASETS = ("tuning", "eval", "killtest", "scratch")
CLOSED = "trial log closed"          # the last record of every log closed normally (pre-hardware review D8)


def is_closed(path, tail=4096):
    """True if the trial log ends with the CLOSED marker that close() writes: the tool is done with it. A log
    without it is still being written, or its tool ended without closing it."""
    try:
        with open(path, "rb") as f:
            f.seek(0, os.SEEK_END)
            f.seek(max(0, f.tell() - tail))
            return CLOSED.encode("utf-8") in f.read()
    except OSError:
        return False


NOT_CODE = ("docs/journal.md",)   # notes the workbench appends to; they do not make the tree dirty (P2)


def git_state():
    def run(*args):
        # stdin=DEVNULL: on Windows, inheriting a stdin that another thread is reading (the workbench
        # channel) blocks process creation
        r = subprocess.run(["git", *args], cwd=ROOT, capture_output=True, text=True, stdin=subprocess.DEVNULL)
        return r.stdout.strip() if r.returncode == 0 else None
    status = run("status", "--porcelain") or ""
    dirty = [ln for ln in status.splitlines() if ln[3:].strip().strip('"') not in NOT_CODE]
    return run("rev-parse", "HEAD"), bool(dirty)


class TrialLog:
    def __init__(self, tool, mode, dataset, config_texts, data_dir=None, extra=None):
        if dataset not in DATASETS:
            raise ValueError(f"dataset must be one of {DATASETS}")
        git_hash, dirty = git_state()
        if dataset == "eval" and (dirty or git_hash is None):
            raise RuntimeError("recording an eval trial from a dirty tree is a hard error (P2)")
        data_dir = pathlib.Path(data_dir or os.environ.get("SLOSH_DATA", ROOT / "data"))
        trials = data_dir / "trials"
        trials.mkdir(parents=True, exist_ok=True)
        now = datetime.datetime.now(datetime.timezone.utc)
        base = f"{now:%Y%m%d-%H%M%S}-{tool}-{mode}"
        self.trial_id, n = base, 1
        while (trials / f"{self.trial_id}.bin").exists():
            n += 1
            self.trial_id = f"{base}-{n}"
        self.path = trials / f"{self.trial_id}.bin"
        header = {
            "trial_id": self.trial_id, "tool": tool, "mode": mode, "dataset": dataset,
            "argv": sys.argv, "wall_clock_utc": now.isoformat(), "monotonic_ns_at_header": time.monotonic_ns(),
            "git_hash": git_hash, "dirty": dirty, "python": sys.version, "platform": platform.platform(),
            "host_clock": "time.monotonic_ns (CLOCK_MONOTONIC on Linux)",
            "config": config_texts,      # full text, not a path or a hash (codebase plan §5)
        }
        header.update(extra or {})
        self.w = L.Writer(self.path, header)
        self.failed = None          # the first write error; nothing is written after it
        self.on_failure = None      # called once with that error (cli.session: a stop request)
        self._fail_lock = threading.Lock()
        self._closed = False

    def close(self):
        if self._closed:
            return
        self._closed = True
        if self.failed is None:           # the last record: the tool is done with this log (review D8)
            try:
                self.w.write("event", t_host_ns=time.monotonic_ns(), kind=EV_MARKER, text=CLOSED.encode("utf-8"))
            except Exception as exc:
                self._fail(exc, stop=False)
        try:
            self.w.close()
        except Exception as exc:
            self._fail(exc, stop=False)   # the tool is ending anyway; cli.session reports it

    def _fail(self, exc, stop=True):
        with self._fail_lock:             # the state-stream thread writes too: one failure, one stop request
            if self.failed is not None:
                return
            self.failed = exc
        try:
            print(f"TRIAL LOG WRITE FAILED ({exc!r}): {self.path} is incomplete from here on; stopping",
                  file=sys.stderr, flush=True)
        except Exception:
            pass
        if stop and self.on_failure is not None:
            try:
                self.on_failure(exc)
            except Exception:
                pass

    def _write(self, stream, **fields):
        if self.failed is not None or self._closed:     # closed: a late write (the state-stream thread) is dropped
            return
        try:
            self.w.write(stream, **fields)
        except Exception as exc:          # never into the tool: its stop and exit sequence must still run
            self._fail(exc)

    # -- records
    def event(self, kind, text):
        if not isinstance(text, str):
            text = json.dumps(text, sort_keys=True, default=str)
        self._write("event", t_host_ns=time.monotonic_ns(), kind=kind, text=text.encode("utf-8"))

    def wire_tx(self, port, data, t_ns):
        self._write("wire_tx", t_host_ns=t_ns, port=port, data=data)

    def wire_rx(self, port, data, t_ns):
        self._write("wire_rx", t_host_ns=t_ns, port=port, data=data)

    def state_raw(self, t_ns, data):
        self._write("state_raw", t_host_ns=t_ns, data=data)

    def robot_state(self, t_ns, parsed):
        z6 = [0.0] * 6
        st = parsed["state"]
        valid = 0
        for bit, key in enumerate(("q_rad", "qd_rad_s", "qdd_rad_s2", "current_A")):
            if parsed[key] is not None:
                valid |= 1 << bit
        state_ok = all(v is not None for v in st.values())
        if state_ok:
            valid |= 1 << 4
        g = lambda k: (st.get(k) or 0) if state_ok else 0
        self._write("robot_state", t_host_ns=t_ns, valid=valid,
                     q_rad=parsed["q_rad"] or z6, qd_rad_s=parsed["qd_rad_s"] or z6,
                     qdd_rad_s2=parsed["qdd_rad_s2"] or z6, current_A=parsed["current_A"] or z6,
                     robot_state=g("robotState"), enabled=g("robotEnabled"), paused=g("robotPaused"),
                     moving=g("robotMoving"), blending_done=g("robotBlendingDone"), in_pos=g("InPos"),
                     error_axis=g("Error_AxisID"), error_code=g("Error_Code"))

    def arm_cmd(self, push_id, tick, q_rad, t_slot_ns, t_send_ns):
        self._write("arm_cmd", push_id=push_id, tick=tick, q_cmd_rad=q_rad, t_slot_ns=t_slot_ns, t_send_ns=t_send_ns)

    def arm_ack(self, push_id, tick, status, fail_code, t_reply_ns, reply_time_ns, late):
        self._write("arm_ack", push_id=push_id, tick=tick, status=status,
                     fail_code_valid=1 if fail_code is not None else 0, fail_code=fail_code or 0,
                     t_reply_ns=t_reply_ns, reply_time_ns=reply_time_ns, late=1 if late else 0)
