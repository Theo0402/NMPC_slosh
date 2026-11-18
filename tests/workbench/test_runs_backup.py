"""Run folders (one per run, never changed after it ends) and the local backup (new files only, never deletes,
restore asks before overwriting). Against the mock and the dry run; no arm. Codebase plan §8, §12."""
import json
import os
import pathlib
import shutil
import threading
import time

import pytest

from analysis.logread import latest
from tests.workbench.test_workbench import PREFLIGHT, temp_configs
from tools.workbench import runfolder
from tools.workbench.backup import Backup, BackupError
from tools.workbench.bench import Refused, Workbench
from tools.workbench.server import serve

ROOT = pathlib.Path(__file__).resolve().parents[2]


@pytest.fixture
def bench(tmp_path):
    confirmed, arm = temp_configs(tmp_path)
    b = Workbench(data_dir=tmp_path / "data", journal=tmp_path / "journal.md", confirmed=confirmed,
                  arm_config=arm, host_problem="this is a test, not the Linux machine")
    b.opened = []
    b.opener = b.opened.append
    yield b
    if b.run is not None and b.run.running:
        b.run.stop()
        b.run.thread.join(timeout=30)
    b.backup.wait(60)


def run_to_end(b, *args, **kw):
    b.start(*args, **kw)
    t_end = time.monotonic() + 90
    while b.run.running and time.monotonic() < t_end:
        for qid in list(b.run.questions):
            b.answer(qid, "yes")
        time.sleep(0.05)
    assert not b.run.running
    b.run.thread.join(timeout=10)
    for _ in range(100):
        if runfolder.is_complete(b.runs_dir / b.run.sink.name):
            return b.runs_dir / b.run.sink.name
        time.sleep(0.05)
    raise AssertionError("run folder not completed")


def read_only(p):
    return not os.access(p, os.W_OK)


# ------------------------------------------------------------------------------------------- run folders
def test_a_mock_run_gets_a_complete_read_only_folder(bench):
    d = run_to_end(bench, "bringup_read", "mock", note="folder test")
    assert runfolder.NAME_RE.match(d.name) and d.name.endswith("_bringup_read_mock")
    files = {str(p.relative_to(d)).replace("\\", "/") for p in d.rglob("*") if p.is_file()}
    for f in ("summary.txt", "run.json", "console.log", "config/confirmed.yaml", "config/e03pro.yaml",
              "config/stage.json", "coeffs_v000001.json", "trial.txt", "mock.log", "COMPLETE"):
        assert f in files, f
    rec = json.loads((d / "run.json").read_text(encoding="utf-8"))
    assert rec["exit_code"] == 0 and rec["auto_ok"] and rec["metrics"]["problems"] == 0 and rec["criteria"]
    assert rec["run_folder"] == d.name and rec["stage_num"] == "1.1"
    console = (d / "console.log").read_text(encoding="utf-8")
    for line in bench.run.console:                               # the whole console, not only the tail
        assert line in console
    assert "[wb " in console and "command:" in console
    assert (d / "config" / "confirmed.yaml").read_text() == bench.confirmed_path.read_text()
    trial = pathlib.Path(bench.run.trial_path)
    if (d / "trial.bin").exists():
        assert os.path.samefile(d / "trial.bin", trial)          # a link: no extra space
    assert runfolder.sha256_file(trial) in (d / "trial.txt").read_text()
    summary = (d / "summary.txt").read_text(encoding="utf-8")
    assert "PASSED its criteria" in summary and "exit code  0" in summary and "OWNER" in summary
    assert all(read_only(p) for p in d.rglob("*") if p.is_file())
    with pytest.raises(runfolder.RunFolderError, match="never written again"):
        bench.run.sink.write_json("x.json", {})
    bench.run.sink.console("late line")                          # dropped, never appended
    assert "late line" not in (d / "console.log").read_text(encoding="utf-8")
    assert d.name in bench.status.journal_path.read_text(encoding="utf-8")


def test_arm_control_practice_keeps_questions_and_commands(bench):
    bench.edit_coeffs({"jog_speed_mm_s": 20.0, "jog_acc_mm_s2": 100.0}, "faster for the test")
    bench.start("arm_control", "mock", practice=True)              # questions answered on the "page"
    t_end = time.monotonic() + 30
    while bench.run.arm["ready"] is None and time.monotonic() < t_end:
        for qid in list(bench.run.questions):
            bench.answer(qid, "yes")
        time.sleep(0.05)

    def do(name, args):
        cid = bench.command(name, args)["id"]
        while cid not in bench.run.arm["results"] and bench.run.running:
            time.sleep(0.05)
        return bench.run.arm["results"][cid]

    assert do("jog", {"axis": "x", "dir": 1, "step_mm": 5})["ok"]
    assert not do("jog", {"axis": "x", "dir": 1, "step_mm": 500})["ok"]
    do("end", {})
    while bench.run.running:
        time.sleep(0.05)
    bench.run.thread.join(10)
    d = bench.runs_dir / bench.run.sink.name
    for _ in range(100):
        if runfolder.is_complete(d):
            break
        time.sleep(0.05)
    assert d.name.endswith("_arm_control_mock-practice")
    q = [json.loads(x) for x in (d / "questions.jsonl").read_text(encoding="utf-8").splitlines()]
    asked = [x for x in q if x["event"] == "asked"]
    answered = [x for x in q if x["event"] == "answered"]
    assert len(asked) >= 2 and len(answered) == len(asked) and all(a["answer"] == "yes" for a in answered)
    assert any("upright" in x["question"] for x in asked)
    c = [json.loads(x) for x in (d / "commands.jsonl").read_text(encoding="utf-8").splitlines()]
    sent = [x for x in c if x["event"] == "sent"]
    done = [x for x in c if x["event"] == "done"]
    assert [x["name"] for x in sent] == ["jog", "jog", "end"]
    assert [x["ok"] for x in done] == [True, False, True] and "jog_max_step_mm" in done[1]["error"]
    assert all("t_mono_ns" in x for x in c)
    assert "PRACTICE" in (d / "summary.txt").read_text(encoding="utf-8")
    assert "limits_readback:            # PRACTICE" in (d / "config" / "e03pro.yaml").read_text(encoding="utf-8")


def test_dry_run_records_auto_answers_and_forms_get_folders(bench):
    d = run_to_end(bench, "move_test", "dry-run", assume_yes=True)
    assert d.name.endswith("_move_test_dry-run")
    q = [json.loads(x) for x in (d / "questions.jsonl").read_text(encoding="utf-8").splitlines()]
    assert any(x["event"] == "answered" and x["by"].startswith("--assume-yes") for x in q)
    res = bench.submit_form("preflight", PREFLIGHT, "bench ready", override_reason="test")
    f = bench.runs_dir / res["run"]["run_folder"]
    assert f.name.endswith("_preflight_form") and runfolder.is_complete(f)
    assert json.loads((f / "form.json").read_text())["values"]["fill_height_mm"] == 60.2
    assert "no trial log" in (f / "trial.txt").read_text()


def test_past_runs_list_detail_plot_and_open(bench):
    d = run_to_end(bench, "servo_ladder", "dry-run", assume_yes=True)
    runs = bench.past_runs()["runs"]
    assert runs[0]["name"] == d.name and runs[0]["trial_bytes"] > 0
    det = bench.past_run(d.name)
    assert det["summary"].startswith("RUN") and "StartServo" in det["console"]
    assert det["plot"]["n_cmd"] > 100 and det["plot"]["n_state"] > 10     # the joint plot comes from the trial
    bench.open_run_folder(d.name)
    assert bench.opened == [d.resolve()]
    with pytest.raises(Refused):
        bench.open_run_folder("../../etc")
    with pytest.raises(Refused):
        bench.past_run("2026-01-01_000000_nothing_mock")


# ------------------------------------------------------------------------------------------- backup
def test_backup_copies_only_what_is_new_and_never_deletes(bench, tmp_path):
    bk = tmp_path / "bk"
    bench.backup_settings(str(bk), auto=False)
    a = run_to_end(bench, "bringup_read", "mock")
    b = run_to_end(bench, "move_test", "dry-run", assume_yes=True)
    first = bench.backup.run("test")
    assert first["copied_files"] > 0 and not first["conflicts"] and not first["errors"]
    again = bench.backup.run("test")
    assert again["copied_files"] == 0
    held = bk / "slosh-data"
    assert (held / "runs" / a.name / "COMPLETE").exists() and (held / "runs" / b.name / "summary.txt").exists()
    if (held / "runs" / a.name / "trial.bin").exists():          # re-linked to the backup's own copy
        tid = json.loads((a / "run.json").read_text())["trial_id"]
        assert os.path.samefile(held / "runs" / a.name / "trial.bin", held / "trials" / f"{tid}.bin")
    st = bench.backup_view()
    assert st["runs_held"] == 2 and st["trials_held"] == 2 and st["last_backup"]["reason"] == "test"
    runfolder.make_writable(a)                                   # the owner deletes a run from this PC
    shutil.rmtree(a)
    bench.backup.run("test")
    assert (held / "runs" / a.name / "summary.txt").exists()     # still in the backup
    out = bench.restore_run(a.name)
    assert not out["needs_confirmation"] and out["restored"]
    assert (a / "summary.txt").exists() and runfolder.is_complete(a)
    assert all(read_only(p) for p in a.rglob("*") if p.is_file())
    assert bench.restore_run(a.name)["restored"] == []           # nothing left to restore


def test_conflicts_are_reported_and_restore_asks_before_overwriting(bench, tmp_path):
    bench.backup_settings(str(tmp_path / "bk"), auto=False)
    a = run_to_end(bench, "bringup_read", "mock")
    bench.backup.run("test")
    held = tmp_path / "bk" / "slosh-data" / "runs" / a.name / "console.log"
    before = held.read_bytes()
    runfolder.make_writable(a)                                   # damage the copy on this PC
    with open(a / "console.log", "a", encoding="utf-8") as f:
        f.write("damaged\n")
    report = bench.backup.run("test")
    assert any("console.log" in c for c in report["conflicts"]) and held.read_bytes() == before
    out = bench.restore_run(a.name)                              # without the owner's yes: nothing changes
    assert out["needs_confirmation"] and any(p.endswith("console.log") for p in out["differ"])
    assert (a / "console.log").read_text(encoding="utf-8").endswith("damaged\n")
    out = bench.restore_run(a.name, overwrite=True)
    assert (a / "console.log").read_bytes() == before
    kept = [pathlib.Path(k) for k in out["kept_aside"]]
    assert kept and kept[0].read_text(encoding="utf-8").endswith("damaged\n")   # the current file was kept


def test_backup_waits_while_a_run_is_active(tmp_path):
    data = tmp_path / "data"
    (data / "trials").mkdir(parents=True)
    L = latest()
    with L.Writer(data / "trials" / "t1.bin", {"dataset": "scratch"}) as w:
        w.write("event", t_host_ns=1, kind=4, text=b"x")
    busy = threading.Event()
    busy.set()
    bk = Backup(data, ROOT, is_busy=lambda: True if busy.is_set() else None)
    bk.set_folder(tmp_path / "bk")
    bk.start("test")
    time.sleep(1.0)
    assert bk.progress and "paused" in bk.progress
    assert not (tmp_path / "bk" / "slosh-data" / "trials" / "t1.bin").exists()
    busy.clear()
    bk.wait(30)
    assert (tmp_path / "bk" / "slosh-data" / "trials" / "t1.bin").exists()


def test_images_follow_the_plan_policy(tmp_path):
    data = tmp_path / "data"
    (data / "trials").mkdir(parents=True)
    L = latest()
    for name, ds in (("s", "scratch"), ("e", "eval"), ("k", "killtest"), ("t", "tuning")):
        with L.Writer(data / "trials" / f"{name}.bin", {"dataset": ds}) as w:
            w.write("event", t_host_ns=1, kind=4, text=b"x")
        (data / "trials" / f"{name}.img").write_bytes(b"SLOSHIMG" + b"\0" * 100)
    bk = Backup(data, ROOT)
    bk.set_folder(tmp_path / "bk")
    bk.run("test")
    got = sorted(p.name for p in (tmp_path / "bk" / "slosh-data" / "trials").iterdir())
    assert got == ["e.bin", "e.img", "k.bin", "k.img", "s.bin", "t.bin"]      # codebase plan §8


def test_backup_folder_must_be_outside_the_data_and_the_repo(bench, tmp_path):
    with pytest.raises(Refused, match="inside the data folder"):
        bench.backup_settings(str(bench.data_dir / "bk"))
    with pytest.raises(Refused, match="inside the repository"):
        bench.backup_settings(str(ROOT / "bk"))
    with pytest.raises(Refused, match="full path"):
        bench.backup_settings("relative/path")
    with pytest.raises(Refused, match="choose the backup folder"):
        bench.backup_now()


def test_interrupted_and_older_runs_get_folders(bench, tmp_path):
    rf = runfolder.RunFolder(bench.runs_dir, "servo_ladder", "mock")      # a workbench that died mid-run
    rf.console("trial started")
    rf._console.close()
    old = bench.data_dir / "workbench" / "runs"
    old.mkdir(parents=True, exist_ok=True)
    (old / "20261112-141835-servo_ladder-dry-run.json").write_text(json.dumps(
        {"run_id": "20261112-141835-servo_ladder-dry-run", "stage": "servo_ladder", "mode": "dry-run",
         "started_utc": "2026-11-12T14:18:35+00:00", "exit_code": 1, "auto_ok": False,
         "console_tail": ["REFUSED: limits not recorded"], "trial_id": None}), encoding="utf-8")
    out = bench.recover_runs(quiet_s=0)
    assert out["interrupted"] == [rf.name] and len(out["reconstructed"]) == 1
    s = (rf.path / "summary.txt").read_text(encoding="utf-8")
    assert "INTERRUPTED" in s and runfolder.is_complete(rf.path)
    r = bench.runs_dir / out["reconstructed"][0]
    assert "RECONSTRUCTED" in (r / "summary.txt").read_text(encoding="utf-8")
    assert "REFUSED: limits not recorded" in (r / "console.log").read_text(encoding="utf-8")
    assert bench.recover_runs(quiet_s=0) == {"interrupted": [], "reconstructed": [], "unreadable": [],
                                             "still_open": []}                         # once only


def test_a_second_workbench_cannot_share_the_port(bench):
    first = serve(bench, port=0)
    try:
        with pytest.raises(OSError):
            serve(bench, port=first.server_address[1])
    finally:
        first.server_close()
