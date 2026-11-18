"""One workbench per data folder (pre-hardware review D6). Standard library only.

data/workbench/workbench.lock names the workbench that uses the folder: a random token, its process id, the machine
(backup.machine_key: Windows, WSL and the Linux machine share data/ but not process ids) and a heartbeat it renews
every HEARTBEAT_S. A second workbench refuses to start while the heartbeat is fresh; a lock whose heartbeat is older
than STALE_S was left by a workbench that died, and is taken over (said so on start-up). Without the lock, a second
workbench on another port, or one in WSL beside one on Windows, closed the first one's live run folder as
INTERRUPTED at its start-up. A workbench whose lock was taken over (it was suspended longer than STALE_S, say)
starts no more runs.
"""
import json
import os
import pathlib
import threading
import time
import uuid

from tools.workbench.backup import machine_key

LOCK_NAME = "workbench.lock"
HEARTBEAT_S = 10.0       # [proposal]
STALE_S = 60.0           # [proposal]


class LockHeld(RuntimeError):
    """Another workbench uses this data folder."""


class DataLock:
    def __init__(self, data_dir):
        self.path = pathlib.Path(data_dir) / "workbench" / LOCK_NAME
        self.token = uuid.uuid4().hex
        self.taken_over = None       # the stale lock this one replaced, if any
        self.lost = None             # the lock that replaced this one, if another workbench took over
        self._stop = threading.Event()
        self._thread = None

    def read(self):
        try:
            info = json.loads(self.path.read_text(encoding="utf-8"))
            return info if isinstance(info, dict) else {}
        except FileNotFoundError:
            return None
        except (OSError, ValueError):
            return {}                # unreadable: treated as stale below (its heartbeat is unknown)

    def _record(self):
        return {"token": self.token, "pid": os.getpid(), "machine": machine_key(),
                "started": time.strftime("%Y-%m-%d %H:%M:%S"), "heartbeat": time.time()}

    def _write(self, rec, exclusive=False):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        text = json.dumps(rec, indent=1, sort_keys=True)
        if exclusive:
            with open(self.path, "x", encoding="utf-8") as f:
                f.write(text)
            return
        tmp = self.path.with_name(f"{LOCK_NAME}.{self.token}.tmp")
        tmp.write_text(text, encoding="utf-8")
        os.replace(tmp, self.path)

    def acquire(self):
        self._rec = self._record()
        try:
            self._write(self._rec, exclusive=True)
        except FileExistsError:
            info = self.read()
            if info is not None:
                age = time.time() - float(info.get("heartbeat") or 0)
                if age < STALE_S:
                    raise LockHeld(f"another workbench uses this data folder ({self.path.parent.parent}): process "
                                   f"{info.get('pid')} on {info.get('machine')}, started {info.get('started')}, "
                                   f"alive {age:.0f} s ago. Close that one first; two workbenches on one data folder "
                                   "would close each other's runs") from None
                self.taken_over = info
            self._write(self._rec)
            time.sleep(0.2)                       # two take-overs at once: the last write wins, the other refuses
            if (self.read() or {}).get("token") != self.token:
                raise LockHeld("another workbench took this data folder at the same moment; close it first") from None
        self._thread = threading.Thread(target=self._beat, name="data-lock", daemon=True)
        self._thread.start()
        return self

    def _beat(self):
        while not self._stop.wait(HEARTBEAT_S):
            info = self.read()
            if info is not None and info.get("token") not in (None, self.token):
                self.lost = info                  # another workbench took over: this one starts no more runs
                return
            try:
                self._write(dict(self._rec, heartbeat=time.time()))
            except OSError:
                pass                              # a reader had it open (Windows): the next beat renews it

    def release(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2)
        if (self.read() or {}).get("token") == self.token:
            try:
                self.path.unlink()
            except OSError:
                pass
