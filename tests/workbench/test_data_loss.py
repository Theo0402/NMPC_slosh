"""The pre-hardware review's data-loss fixes D3-D8 (2026-11-18): the backup folder per machine, backup.sh keeping
the copies it replaces, the journal appended, one workbench per data folder, unreadable records skipped, and trial
logs still open left alone."""
import json
import os
import pathlib
import shutil
import subprocess
import sys
import time

import pytest

from analysis.logread import latest
from tests.workbench.test_channel_and_tools import write_trial
from tests.workbench.test_workbench import ROOT, bench  # noqa: F401 (bench is a fixture)
from tools.elfin_py import triallog
from tools.workbench import backup, runfolder
from tools.workbench.backup import Backup
from tools.workbench.datalock import DataLock, LockHeld
from tools.workbench.status import RUNS_HEADING


# ------------------------------------------------------------------ D3: the backup folder, per machine, checked on use
def test_the_backup_folder_is_kept_per_machine(tmp_path, monkeypatch):
    data = tmp_path / "data"
    data.mkdir()
    b = Backup(data, ROOT)
    monkeypatch.setattr(backup, "machine_key", lambda: "Windows:LAB")
    b.set_folder(tmp_path / "bk_windows")
    monkeypatch.setattr(backup, "machine_key", lambda: "WSL:LAB")
    assert b.settings()["folder"] is None and b.dest() is None       # not set up on this machine yet
    b.set_folder(tmp_path / "bk_wsl")
    monkeypatch.setattr(backup, "machine_key", lambda: "Windows:LAB")
    assert b.dest() == tmp_path / "bk_windows" / "slosh-data"         # the other machine's choice is kept


def test_an_old_single_folder_counts_only_where_it_is_a_full_path(tmp_path):
    data = tmp_path / "data"
    (data / "workbench").mkdir(parents=True)
    settings = data / "workbench" / "backup_settings.json"
    other = "/home/kim/slosh-backup" if os.name == "nt" else "C:\\Users\\User\\slosh-backup"
    settings.write_text(json.dumps({"folder": other, "auto": True}), encoding="utf-8")
    b = Backup(data, ROOT)
    assert b.dest() is None and b.status()["folder"] is None          # before: a relative path inside the repo
    settings.write_text(json.dumps({"folder": str(tmp_path / "bk"), "auto": True}), encoding="utf-8")
    assert b.dest() == tmp_path / "bk" / "slosh-data"                 # where it is a full path, it still works
    settings.write_text(json.dumps({"folders": {backup.machine_key(): str(data / "inside")}}), encoding="utf-8")
    assert b.dest() is None and "cannot be inside the data folder" in b.status()["folder_problem"]


# ------------------------------------------------------------------ D4: backup.sh keeps what a changed file replaces
@pytest.mark.skipif(shutil.which("rsync") is None or shutil.which("bash") is None, reason="needs bash and rsync (Linux)")
def test_backup_sh_keeps_the_copy_a_changed_file_replaces(tmp_path):
    data = tmp_path / "data"
    (data / "trials").mkdir(parents=True)
    (data / "workbench").mkdir()
    write_trial(data / "trials" / "a.bin", "scratch")
    audit = data / "workbench" / "audit.jsonl"
    audit.write_text('{"n": 1}\n', encoding="utf-8")
    dst = tmp_path / "disk"
    dst.mkdir()
    env = dict(os.environ, BACKUP_DISK=str(dst))
    env.pop("BACKUP_OFFSITE", None)

    def run():
        res = tmp_path / "r.json"
        r = subprocess.run(["bash", "tools/backup.sh", "--data-dir", str(data), "--result-json", str(res)], cwd=ROOT,
                           env=env, capture_output=True, text=True, timeout=60)
        assert r.returncode == 0, r.stderr
        return json.loads(res.read_text()), r
    first, _ = run()
    assert first["replaced_files"] == 0
    audit.write_text('{"n": 1}\n{"n": 2}\n', encoding="utf-8")         # a record that grows: copied again
    with open(data / "trials" / "a.bin", "ab") as f:                    # a trial that changed: damage (A4)
        f.write(b"damage")
    second, r = run()
    assert (dst / "slosh-data" / "workbench" / "audit.jsonl").read_text() == '{"n": 1}\n{"n": 2}\n'
    rdir = pathlib.Path(second["replaced_dir"])
    assert (rdir / "workbench" / "audit.jsonl").read_text() == '{"n": 1}\n'     # the old copy is kept, not lost
    assert (rdir / "trials" / "a.bin").stat().st_size == (data / "trials" / "a.bin").stat().st_size - 6
    assert second["replaced_files"] == 2 and second["replaced_bin_files"] == 1 and "WARNING" in r.stderr


# ------------------------------------------------------------------ D5: the journal is appended, never rewritten
def test_journal_lines_are_appended_never_rewritten(bench):
    p = bench.status.journal_path
    p.write_bytes(b"# Journal\n\n- **W1 d1.** the owner's own line\n")
    before = p.read_bytes()
    bench.status.journal("run", {"stage": "move_test", "mode": "mock", "verdict": "criteria met"})
    after = p.read_bytes()
    assert after.startswith(before) and b"\r" not in after and after.count(RUNS_HEADING.encode()) == 1
    p.write_bytes(after.replace(b"\n", b"\r\n"))                       # a file with CRLF endings keeps them
    crlf = p.read_bytes()
    bench.status.journal("run", {"stage": "move_test"})
    added = p.read_bytes()[len(crlf):]
    assert p.read_bytes().startswith(crlf) and added.endswith(b"\r\n") and b"\n" not in added.replace(b"\r\n", b"")


# ------------------------------------------------------------------ D6: one workbench per data folder
def test_one_workbench_per_data_folder(tmp_path):
    first = DataLock(tmp_path / "data").acquire()
    try:
        with pytest.raises(LockHeld, match="another workbench uses this data folder"):
            DataLock(tmp_path / "data").acquire()
    finally:
        first.release()
    DataLock(tmp_path / "data").acquire().release()                   # free again once released


def test_a_lock_left_by_a_dead_workbench_is_taken_over(tmp_path):
    (tmp_path / "data" / "workbench").mkdir(parents=True)
    (tmp_path / "data" / "workbench" / "workbench.lock").write_text(json.dumps(
        {"token": "old", "pid": 1, "machine": "x", "started": "then", "heartbeat": time.time() - 120}), encoding="utf-8")
    lock = DataLock(tmp_path / "data").acquire()
    try:
        assert lock.taken_over["token"] == "old"
    finally:
        lock.release()
    assert not (tmp_path / "data" / "workbench" / "workbench.lock").exists()


def test_a_second_workbench_process_refuses_the_same_data_folder(tmp_path):
    args = [sys.executable, "-u", "-m", "tools.workbench", "--port", "0", "--data-dir", str(tmp_path / "data"),
            "--backup-dir", str(tmp_path / "backup")]
    first = subprocess.Popen(args, cwd=ROOT, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT, text=True)
    try:
        t_end, seen = time.monotonic() + 60, []
        while time.monotonic() < t_end:
            line = first.stdout.readline()
            if not line:
                break
            seen.append(line)
            if line.startswith("workbench on http"):
                break
        assert any(s.startswith("workbench on http") for s in seen), seen
        second = subprocess.run(args, cwd=ROOT, stdin=subprocess.DEVNULL, capture_output=True, text=True, timeout=60)
        assert second.returncode != 0 and "another workbench uses this data folder" in second.stderr, second
    finally:
        first.terminate()
        first.wait(timeout=10)


# ------------------------------------------------------------------ D7: one unreadable record never stops the workbench
def test_a_corrupt_run_json_is_reported_and_skipped(bench):
    good = runfolder.RunFolder(bench.runs_dir, "move_test", "mock")
    good.console("left open by a workbench that died")
    good._console.close()
    bad = runfolder.RunFolder(bench.runs_dir, "servo_ladder", "mock")
    bad._console.close()
    (bad.path / "run.json").write_text("{ half a record", encoding="utf-8")
    out = bench.recover_runs(quiet_s=0)
    assert [n for n, _ in out["unreadable"]] == [bad.name] and out["interrupted"] == [good.name]
    assert not runfolder.is_complete(bad.path)                          # left as it is, for the owner to look at
    (bench.data_dir / "workbench" / "runs" / "broken.json").write_text("{", encoding="utf-8")
    bench.status._runs = None
    ov = bench.overview()                                               # the page still loads
    assert any(k.endswith("broken.json") for k in ov["unreadable_records"])


def test_run_records_are_written_whole(tmp_path):
    rf = runfolder.RunFolder(tmp_path, "move_test", "mock")
    rf.write_json("run.json", {"a": 1})
    rec = rf.finish({"stage": "move_test", "mode": "mock"})
    assert json.loads((rf.path / "run.json").read_text(encoding="utf-8"))["run_folder"] == rec["run_folder"]
    assert not list(rf.path.glob("*.tmp")) and runfolder.is_complete(rf.path)


# ------------------------------------------------------------------ D8: trials from a terminal, and trials still open
def test_trials_still_open_are_left_alone_and_terminal_runs_are_labelled(bench):
    trials = bench.data_dir / "trials"
    trials.mkdir(parents=True, exist_ok=True)
    waiting = trials / "20261118-120000-move_test-mock.bin"           # a terminal run waiting at a prompt
    with latest().Writer(waiting, {"trial_id": waiting.stem, "tool": "move_test", "mode": "mock", "dataset": "scratch",
                                   "wall_clock_utc": "2026-11-18T12:00:00+00:00"}) as w:
        w.write("event", t_host_ns=1, kind=4, text=b"prompt: Start streaming?")
    os.utime(waiting, (time.time() - 120, time.time() - 120))
    done = triallog.TrialLog("move_test", "mock", "scratch", {}, data_dir=bench.data_dir)    # a finished one
    done.event(4, "done")
    done.close()
    out = bench.recover_runs(quiet_s=60)
    assert out["still_open"] == [waiting.stem] and len(out["reconstructed"]) == 1
    folder = bench.runs_dir / out["reconstructed"][0]
    rec = json.loads((folder / "run.json").read_text(encoding="utf-8"))
    assert rec["trial_id"] == done.trial_id and rec["outside_workbench"] and rec["log_closed"]
    assert "started outside the workbench" in (folder / "summary.txt").read_text(encoding="utf-8")
    os.utime(waiting, (time.time() - 25 * 3600, time.time() - 25 * 3600))     # a day later: abandoned, taken in
    out = bench.recover_runs(quiet_s=60)
    assert not out["still_open"] and len(out["reconstructed"]) == 1
    rec = json.loads((bench.runs_dir / out["reconstructed"][0] / "run.json").read_text(encoding="utf-8"))
    assert rec["trial_id"] == waiting.stem and rec["log_closed"] is False
