"""The tool side of the workbench channel (codebase plan §12). Standard library only.

Active only when the workbench started the tool (environment SLOSH_WORKBENCH=1); otherwise the tools
behave exactly as before (terminal prompts, Ctrl-C).
- stdout: lines that start with '@@wb ' carry one JSON object (the trial's path, questions, notices);
  every other line is console text.
- stdin: one JSON object per line from the workbench, {"type": "answer", "id": n, "text": "yes"},
  {"type": "cmd", "id": n, "name": ..., "args": {...}} (the jog tool's commands) or {"type": "stop"}. End of file means the workbench died or let go: that is a stop too.

A stop raises KeyboardInterrupt in the main thread exactly as Ctrl-C does, so the tool runs its Ctrl-C
code: the servo ladder decelerates by streaming, and every tool that enabled the arm sends GrpDisable on
exit (codebase plan §3.8 items 6 and 8). After a stop every question is answered "no", so nothing new
starts with nobody holding the stop button. This is a software stop; the e-stop stays within reach.
A tool started with SIGINT ignored gets Python's handler back at the start of its session (cli.py), or the
interrupt would do nothing (pre-hardware review S13).
"""
import json
import os
import queue
import signal
import sys
import threading
import time

PREFIX = "@@wb "
ENV = "SLOSH_WORKBENCH"


def active(environ=None):
    return (os.environ if environ is None else environ).get(ENV) == "1"


def interrupt_main():
    """Deliver SIGINT to the main thread, as Ctrl-C does. If the tool has set SIGINT to SIG_IGN (while it
    disables the arm), nothing happens; while the servo ladder streams, its own handler only sets a flag that
    the stream loop acts on between pushes (tools/elfin_py/servo_ladder.py)."""
    if hasattr(signal, "pthread_kill"):                       # POSIX: wakes the main thread's sleep too
        signal.pthread_kill(threading.main_thread().ident, signal.SIGINT)
    else:                                                     # Windows: the C handler sets the event sleep waits on
        signal.raise_signal(signal.SIGINT)


class Channel:
    def __init__(self, stdin=None, stdout=None, interrupt=interrupt_main):
        self._in = sys.stdin if stdin is None else stdin
        self._out = sys.stdout if stdout is None else stdout
        self._interrupt = interrupt
        self._answers = {}
        self._next_id = 0
        self._cv = threading.Condition()
        self._write_lock = threading.Lock()
        self.stopped = None                                   # the reason, once stopped
        self.on_stop = None                                   # called with the reason (logs it in the trial)
        self.commands = queue.Queue()                         # {"type": "cmd", "id", "name", "args"} (jog tool)
        self._thread = threading.Thread(target=self._read, name="workbench-channel", daemon=True)
        self._thread.start()

    def emit(self, kind, **fields):
        line = PREFIX + json.dumps(dict(fields, type=kind), sort_keys=True, default=str)
        with self._write_lock:
            self._out.write(line + "\n")
            self._out.flush()

    def ask(self, question, timeout=None):
        """The operator's answer as text, or None once the channel has stopped or `timeout` s have passed
        without an answer. A question that timed out is withdrawn from the workbench's page."""
        with self._cv:
            if self.stopped:
                return self._after_stop()
            self._next_id += 1
            qid = self._next_id
        self.emit("ask", id=qid, question=question)
        deadline = None if timeout is None else time.monotonic() + timeout
        with self._cv:
            while qid not in self._answers and not self.stopped:
                if deadline is not None and time.monotonic() >= deadline:
                    break
                self._cv.wait(0.2)
            if qid in self._answers:
                return self._answers.pop(qid)
            stopped = self.stopped
        if not stopped:
            self.emit("withdrawn", id=qid, reason=f"no answer within {timeout:g} s: taken as no")
            return None
        return self._after_stop()

    def next_command(self, timeout):
        """The next command from the workbench, or None (none came, or the channel has stopped)."""
        if self.stopped:
            return None
        try:
            return self.commands.get(timeout=timeout)
        except queue.Empty:
            return None

    @staticmethod
    def _after_stop(wait_s=1.0):
        # The stop's interrupt is on its way to the main thread: let it arrive here, inside ask(), rather
        # than in the cleanup the tool runs next (GrpDisable). If SIGINT is ignored by then, answer "no".
        deadline = time.monotonic() + wait_s
        while time.monotonic() < deadline:
            time.sleep(0.05)
        return None

    def _read(self):
        try:
            for line in self._in:
                line = line.strip()
                if not line:
                    continue
                try:
                    msg = json.loads(line)
                except ValueError:
                    continue
                if not isinstance(msg, dict):
                    continue
                if msg.get("type") == "answer":
                    with self._cv:
                        self._answers[int(msg.get("id", -1))] = str(msg.get("text", ""))
                        self._cv.notify_all()
                elif msg.get("type") == "cmd":
                    if not self.stopped:
                        self.commands.put(msg)
                elif msg.get("type") == "stop":
                    self._stop("stop button")
        except (OSError, ValueError):
            pass
        self._stop("workbench channel closed")

    def _stop(self, reason):
        with self._cv:
            if self.stopped:
                return
            self.stopped = reason
            self._cv.notify_all()
        # the stop first (pre-hardware review S7): the notice and the log line below may block (a full pipe,
        # a slow disk) or fail, and must never hold the stop back
        self._interrupt()
        if self.on_stop is not None:
            try:
                self.on_stop(reason)
            except Exception:
                pass
        if self._write_lock.acquire(timeout=1.0):  # never wait behind a write that is stuck on a full pipe
            try:
                self._out.write(PREFIX + json.dumps({"type": "stopping", "reason": reason}, sort_keys=True) + "\n")
                self._out.flush()
            except (OSError, ValueError):
                pass
            finally:
                self._write_lock.release()
