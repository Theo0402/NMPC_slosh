"""The tool side of the channel, the host check (stage 0.1) and backup.sh (stage 9.2)."""
import io
import json
import os
import pathlib
import shutil
import subprocess
import sys
import threading
import time

import pytest

from analysis.logread import latest
from tools import host_check
from tools.elfin_py import channel

ROOT = pathlib.Path(__file__).resolve().parents[2]


class Pipe(io.TextIOBase):
    """A stdin the test writes to; iteration blocks until a line or close()."""

    def __init__(self):
        self.lines, self.closed_ = [], False
        self.cv = threading.Condition()

    def put(self, obj):
        with self.cv:
            self.lines.append(json.dumps(obj) + "\n")
            self.cv.notify_all()

    def close(self):
        with self.cv:
            self.closed_ = True
            self.cv.notify_all()

    def __iter__(self):
        while True:
            with self.cv:
                while not self.lines and not self.closed_:
                    self.cv.wait()
                if self.lines:
                    yield self.lines.pop(0)
                else:
                    return


def make():
    stops = []
    pipe, out = Pipe(), io.StringIO()
    ch = channel.Channel(stdin=pipe, stdout=out, interrupt=lambda: stops.append(time.monotonic()))
    return ch, pipe, out, stops


def test_channel_is_off_unless_the_workbench_started_the_tool():
    assert not channel.active({})
    assert channel.active({"SLOSH_WORKBENCH": "1"})


def test_answers_come_back_by_id():
    ch, pipe, out, stops = make()
    t = threading.Thread(target=lambda: pipe.put({"type": "answer", "id": 1, "text": "yes"}))
    threading.Timer(0.2, t.start).start()
    assert ch.ask("Start streaming?") == "yes"
    msg = json.loads(out.getvalue().splitlines()[0][len(channel.PREFIX):])
    assert msg == {"type": "ask", "id": 1, "question": "Start streaming?"}
    assert not stops


def test_stop_interrupts_once_and_later_questions_get_no():
    ch, pipe, out, stops = make()
    pipe.put({"type": "stop"})
    for _ in range(50):
        if stops:
            break
        time.sleep(0.02)
    assert len(stops) == 1 and ch.stopped == "stop button"
    assert ch.ask("Send GrpReset?") is None
    pipe.close()                                   # EOF after a stop does not interrupt again
    time.sleep(0.1)
    assert len(stops) == 1


def test_end_of_file_is_a_stop():
    ch, pipe, out, stops = make()
    seen = []
    ch.on_stop = seen.append
    pipe.close()
    for _ in range(50):
        if stops and '"type": "stopping"' in out.getvalue():     # the notice follows the stop (review S7)
            break
        time.sleep(0.02)
    assert ch.stopped == "workbench channel closed" and seen == ["workbench channel closed"] and len(stops) == 1
    assert '"type": "stopping"' in out.getvalue()


def test_host_check_writes_its_metrics(tmp_path):
    res = tmp_path / "r.json"
    assert host_check.main(["--data-dir", str(tmp_path / "data"), "--result-json", str(res)]) == 0
    m = json.loads(res.read_text())
    assert m["data_free_gb"] > 0 and m["monotonic_resolution_s"] > 0
    assert m["arm_host_ok"] is (sys.platform.startswith("linux") and m["arm_host_problem"] is None)


def write_trial(path, dataset):
    L = latest()
    with L.Writer(path, {"trial_id": path.stem, "dataset": dataset}) as w:
        w.write("event", t_host_ns=1, kind=4, text=b"x")


@pytest.mark.skipif(shutil.which("rsync") is None or shutil.which("bash") is None, reason="needs bash and rsync (Linux)")
def test_backup_copies_bin_files_and_eval_images_only(tmp_path):
    data = tmp_path / "data"
    (data / "trials").mkdir(parents=True)
    for name, ds in (("a", "scratch"), ("b", "eval"), ("c", "killtest")):
        write_trial(data / "trials" / f"{name}.bin", ds)
        (data / "trials" / f"{name}.img").write_bytes(b"SLOSHIMG")
    (data / "workbench").mkdir()
    (data / "workbench" / "audit.jsonl").write_text("{}\n")
    run = data / "runs" / "2026-11-18_101500_bringup_read_mock"   # a run folder with its trial hard-linked
    run.mkdir(parents=True)
    (run / "summary.txt").write_text("RUN\n")
    try:
        os.link(data / "trials" / "b.bin", run / "trial.bin")
        linked = True
    except OSError:
        linked = False
    dst = tmp_path / "disk"
    dst.mkdir()
    res = tmp_path / "r.json"
    env = dict(os.environ, BACKUP_DISK=str(dst))
    env.pop("BACKUP_OFFSITE", None)
    r = subprocess.run(["bash", "tools/backup.sh", "--data-dir", str(data), "--result-json", str(res)], cwd=ROOT,
                       env=env, capture_output=True, text=True, timeout=60)
    assert r.returncode == 0, r.stderr
    got = sorted(str(p.relative_to(dst / "slosh-data")).replace("\\", "/") for p in (dst / "slosh-data").rglob("*") if p.is_file())
    want = ["runs/2026-11-18_101500_bringup_read_mock/summary.txt", "trials/a.bin", "trials/b.bin", "trials/b.img",
            "trials/c.bin", "trials/c.img", "workbench/audit.jsonl"]
    if linked:
        want.append("runs/2026-11-18_101500_bringup_read_mock/trial.bin")
        assert os.path.samefile(dst / "slosh-data" / "runs" / "2026-11-18_101500_bringup_read_mock" / "trial.bin",
                                dst / "slosh-data" / "trials" / "b.bin")     # rsync -H kept the link
    assert sorted(got) == sorted(want)
    assert json.loads(res.read_text())["bin_files"] == 3
    assert json.loads(res.read_text())["backup_ok"] is True


@pytest.mark.skipif(shutil.which("bash") is None, reason="needs bash")
def test_backup_refuses_without_a_disk(tmp_path):
    res = tmp_path / "r.json"
    env = dict(os.environ)
    env.pop("BACKUP_DISK", None)
    r = subprocess.run(["bash", "tools/backup.sh", "--data-dir", str(tmp_path), "--result-json", str(res)], cwd=ROOT,
                       env=env, capture_output=True, text=True, timeout=60)
    assert r.returncode == 3 and "MISSING" in r.stderr
    assert json.loads(res.read_text())["backup_ok"] is False
