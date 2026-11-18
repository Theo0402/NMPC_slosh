"""Start a stage tool as its own process and talk to it over the channel (codebase plan §12).

The tool runs in its own process group, so that a crash or a Ctrl-C in the workbench's terminal does not
cut it off; if the workbench dies, the tool's stdin closes and the tool stops itself safely
(tools/elfin_py/channel.py; its prints can no longer fail, tools/elfin_py/cli.py). The workbench never
writes to the arm.

The reader thread never stops reading while the tool lives (pre-hardware review S7): a line it cannot handle,
or a run-folder write that fails (a full disk), is noted on the console and the reading goes on, so the tool
never blocks on a full pipe. The stop is sent before anything is written about it.
"""
import collections
import json
import os
import re
import subprocess
import sys
import threading
import time

from tools.elfin_py.channel import PREFIX
from tools.workbench.livelog import LiveLog

CONSOLE_KEEP = 3000                # console lines the page can fetch; console.log in the run folder has all
BANNER = re.compile(r"commands (\d+), Datasheet (\d+)")


def _group_kwargs():
    if os.name == "nt":
        return {"creationflags": subprocess.CREATE_NEW_PROCESS_GROUP}
    return {"start_new_session": True}


class MockProcess:
    """The mock controller (python -m tools.elfin_mock) on free ports, mirroring e03pro.yaml."""

    def __init__(self, root, arm_config, python=sys.executable, extra=()):
        argv = [python, "-u", "-m", "tools.elfin_mock", "--cmd-port", "0", "--state-port", "0",
                "--mirror-config", str(arm_config), *extra]
        self.proc = subprocess.Popen(argv, cwd=root, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                                     stderr=subprocess.STDOUT, text=True, encoding="utf-8", errors="replace",
                                     **_group_kwargs())
        self.lines = []
        deadline = time.monotonic() + 20
        self.ports = None
        while time.monotonic() < deadline:
            line = self.proc.stdout.readline()
            if not line:
                break
            self.lines.append(line.rstrip())
            m = BANNER.search(line)
            if m:
                self.ports = (int(m.group(1)), int(m.group(2)))
                break
        if self.ports is None:
            self.stop()
            raise RuntimeError("the mock controller did not start: " + " | ".join(self.lines[-5:]))
        threading.Thread(target=self._drain, daemon=True).start()

    def _drain(self):
        for line in self.proc.stdout:
            self.lines.append(line.rstrip())

    def stop(self):
        if self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(5)
            except subprocess.TimeoutExpired:
                self.proc.kill()


class ToolRun:
    """One run of one stage tool. on_exit(run) is called from the reader thread when it ends."""

    def __init__(self, run_id, stage, mode, argv, env, root, on_exit=None, mock=None, sink=None):
        self.run_id, self.stage, self.mode, self.argv = run_id, stage, mode, argv
        self.sink = sink               # the run folder: every console line, question, answer and command
        self.console = collections.deque(maxlen=CONSOLE_KEEP)   # the newest lines; the run folder has all
        self.console_total = 0         # every line ever added: the page asks for lines after N (review I1)
        self._console_lock = threading.Lock()
        self.questions = {}            # id -> question, until answered
        self.trial_path = self.trial_id = None
        self.live = None
        self.exit_code = None
        self.finished = False          # True once the run is saved (on_exit returned), not just when the tool exits
        self.started = time.time()
        self.stop_sent = False
        self.notices = []
        self.arm = {"ready": None, "pose": None, "busy": None, "results": {}, "progress": None,
                    "idle_timeout": None}                                            # jog tool state
        self._cmd_id = 0
        self.mock = mock
        self._on_exit = on_exit
        self._wlock = threading.Lock()
        self.proc = subprocess.Popen(argv, cwd=root, env=env, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                     stderr=subprocess.STDOUT, text=True, encoding="utf-8", errors="replace",
                                     bufsize=1, **_group_kwargs())
        self.thread = threading.Thread(target=self._read, daemon=True)
        self.thread.start()

    @property
    def running(self):
        """Until the run is saved (pre-hardware review S19): after the tool exits, the mock is stopped (up to 5 s)
        and the run record and folder are written; a run started in between would be checked against older runs."""
        return not self.finished

    def note(self, line):
        self._log(line)

    def _append(self, line):
        with self._console_lock:
            self.console.append(line)
            self.console_total += 1

    def _log(self, line):
        self._append(line)
        self._sink("console", line)

    def _sink(self, method, *args, **kw):
        if self.sink is not None:
            try:
                getattr(self.sink, method)(*args, **kw)
            except Exception as exc:            # the run folder must never break the run
                self._sink_error(exc)

    _sink_failed = None

    def _sink_error(self, exc):
        if self._sink_failed is None:           # once: a full disk would otherwise repeat it for every line
            self._sink_failed = exc
            self._append(f"[RUN FOLDER WRITE FAILED: {exc!r}; the console below is kept in memory only]")

    def _send(self, obj):
        with self._wlock:
            if self.proc.poll() is not None or self.proc.stdin.closed:
                return False
            try:
                self.proc.stdin.write(json.dumps(obj) + "\n")
                self.proc.stdin.flush()
                return True
            except (OSError, ValueError):
                return False

    def answer(self, qid, text):
        qid = int(qid)
        if qid not in self.questions:
            return False
        self.questions.pop(qid)
        self._log(f"[answered {text!r}]")
        self._sink("answer", qid, text)
        return self._send({"type": "answer", "id": qid, "text": text})

    def command(self, name, args):
        """Send one command to the jog tool; its result arrives as a 'done' event with the same id. The session
        counts as busy from now, before the tool says so, so that a second click cannot send a second move
        (pre-hardware review S16); the tool's 'done' clears it."""
        with self._wlock:
            self._cmd_id += 1
            cid = self._cmd_id
            self.arm["busy"] = {"id": cid, "name": name, "relayed": True}
        if not self._send({"type": "cmd", "id": cid, "name": name, "args": args or {}}):
            if (self.arm.get("busy") or {}).get("id") == cid:
                self.arm["busy"] = None
            return None
        self._log(f"[command {cid}] {name} {json.dumps(args or {})[:160]}")
        self._sink("command", cid, name, args or {})
        return cid

    def stop(self):
        self.stop_sent = True
        ok = self._send({"type": "stop"})       # first: nothing written about it may hold it back (review S7)
        self._log("[STOP sent]" if ok else "[STOP: the tool's input is closed; it has ended or is ending]")
        return ok

    def _read(self):
        try:
            for line in self.proc.stdout:
                try:
                    self._line(line.rstrip("\n"))
                except Exception as exc:        # one bad line never ends the reading (review S7)
                    self._append(f"[workbench could not handle a line from the tool: {exc!r}]")
        except Exception as exc:                # the pipe itself failed: the tool's end is still waited for below
            self._append(f"[workbench stopped reading the tool's output: {exc!r}]")
        self._ended()

    def _ended(self):
        try:
            for step in (self._wait_exit, self.questions.clear, self._poll_live, self._stop_mock, self._close_stdin):
                try:
                    step()
                except Exception as exc:
                    self._append(f"[workbench, after the tool ended: {exc!r}]")
            if self._on_exit:
                self._on_exit(self)
        finally:
            self.finished = True

    def _wait_exit(self):
        self.exit_code = self.proc.wait()

    def _poll_live(self):
        if self.live:
            self.live.poll()

    def _stop_mock(self):
        if self.mock:
            self.mock.stop()

    def _close_stdin(self):
        with self._wlock:
            try:
                self.proc.stdin.close()
            except OSError:
                pass

    def _line(self, line):
        if not line.startswith(PREFIX):
            self._log(line)
            return
        try:
            msg = json.loads(line[len(PREFIX):])
        except ValueError:
            self._log(line)
            return
        kind = msg.get("type")
        if kind == "ask":
            self.questions[int(msg["id"])] = msg["question"]
            self._log(f"[question] {msg['question']}")
            self._sink("question", int(msg["id"]), msg["question"])
        elif kind == "withdrawn":                   # the tool stopped waiting for this answer (review S9)
            self.questions.pop(int(msg["id"]), None)
            self._log(f"[question {msg['id']} withdrawn] {msg.get('reason', '')}")
            self._sink("answer", int(msg["id"]), None, by=f"nobody: {msg.get('reason', 'withdrawn')}")
        elif kind == "auto_answer":
            self._sink("question", None, msg.get("question"))
            self._sink("answer", None, msg.get("answer"), by="--assume-yes (dry run or mock only)")
        elif kind == "trial":
            self.trial_path, self.trial_id = msg.get("path"), msg.get("trial_id")
            self.live = LiveLog(self.trial_path)
            self._log(f"[trial] {self.trial_id} -> {self.trial_path}")
        elif kind in ("ready", "pose"):
            if kind == "ready":
                self.arm["ready"] = msg
                self._log("[ready] arm enabled; jog and point moves accepted")
                self._sink("command_event", msg)
            self.arm["pose"] = msg                  # 2 Hz while idle: in the trial log, not in console.log
        elif kind == "busy":
            self.arm["busy"] = msg
            self._sink("command_event", msg)
        elif kind == "idle_timeout":
            self.arm["idle_timeout"] = msg
            self._sink("command_event", msg)
            self._log(f"[idle timeout] no command for {msg.get('after_s')} s: the session ends, the arm is disabled")
        elif kind == "progress":
            self.arm["progress"] = msg
            self._sink("command_event", msg)
        elif kind == "done":
            self.arm["results"][int(msg["id"])] = msg
            self.arm["busy"] = None
            self.arm["progress"] = None
            self._sink("command_event", msg)
            self._log(f"[done {msg.get('id')}] " + ("ok" if msg.get("ok") else f"refused: {msg.get('error')}"))
        else:
            self.notices.append(msg)
            self._log(f"[{kind}] {msg.get('reason', '')}")

    def view(self, since=0):
        """The run for the page. `since` counts every console line ever added (console_total), so the page keeps
        getting new lines after the buffer is full; lines older than the buffer are in the run folder's console.log
        (pre-hardware review I1: the view stopped at 3,000 lines)."""
        with self._console_lock:
            lines, total = list(self.console), self.console_total
        first = total - len(lines)                 # the number of the oldest line still held
        if since < first:
            lines = [f"[{first - since} older line(s) not shown here: they are in the run folder's console.log]"] + lines
        else:
            lines = lines[since - first:]
        if self.live:
            self.live.poll()
        return {"run_id": self.run_id, "stage": self.stage, "mode": self.mode, "running": self.running,
                "exit_code": self.exit_code, "trial_id": self.trial_id, "trial_path": self.trial_path,
                "questions": [{"id": k, "question": v} for k, v in sorted(self.questions.items())],
                "console": lines, "console_len": total,
                "live": self.live.snapshot() if self.live else None, "stop_sent": self.stop_sent,
                "arm": {k: v for k, v in self.arm.items() if k != "results"},
                "results": {k: v for k, v in sorted(self.arm["results"].items())[-20:]},
                "elapsed_s": round(time.time() - self.started, 1)}
