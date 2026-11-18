"""The pre-hardware review's inconvenience and documentation fixes I1-I5 (2026-11-18): the console view past its
buffer, Open folder without a file manager, the two time bases, malformed Arm control commands, and the docs."""
import http.client
import json
import os
import sys
import threading

from analysis.logread import read_trial
from tests.workbench.test_arm_control import do
from tests.workbench.test_safety_workbench import arm_session
from tests.workbench.test_workbench import ROOT, bench, temp_configs, wait_run  # noqa: F401 (bench is a fixture)
from tools.workbench import runfolder
from tools.workbench.bench import Workbench
from tools.workbench.runner import ToolRun
from tools.workbench.server import serve


# ------------------------------------------------------------------ I1
def test_the_console_view_keeps_up_past_its_buffer():
    script = "import sys\nfor k in range(5000):\n    print(f'line {k}')\nsys.stdout.flush()\n"
    run = ToolRun("r", "x", "mock", [sys.executable, "-u", "-c", script], dict(os.environ), ROOT)
    run.thread.join(timeout=60)
    assert run.view(4990)["console"] == [f"line {k}" for k in range(4990, 5000)]      # before: all 3,000, forever
    v = run.view(0)
    assert v["console_len"] == 5000 and v["console"][0].startswith("[2000 older line(s) not shown here")
    assert v["console"][-1] == "line 4999" and run.view(5000)["console"] == []


# ------------------------------------------------------------------ I2
def test_open_folder_without_a_file_manager_gives_the_path(bench):
    rf = runfolder.RunFolder(bench.runs_dir, "move_test", "mock")
    rf.finish({"stage": "move_test", "mode": "mock"})

    def no_file_manager(path):
        raise FileNotFoundError(2, "No such file or directory", "xdg-open")
    bench.opener = no_file_manager
    r = bench.open_run_folder(rf.name)
    assert r["opened"] is None and r["path"].endswith(rf.name) and "open this folder yourself" in r["note"]


def test_an_unexpected_error_is_an_answer_not_a_dropped_connection(bench):
    def broken():
        raise OSError("disk gone")
    bench.backup_view = broken
    httpd = serve(bench, port=0)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    port = httpd.server_address[1]
    try:
        c = http.client.HTTPConnection("127.0.0.1", port, timeout=10)
        c.request("GET", "/api/backup", headers={"Host": f"127.0.0.1:{port}", "X-Slosh-Workbench": "1"})
        r = c.getresponse()
        assert r.status == 500 and "disk gone" in json.loads(r.read())["error"]
    finally:
        httpd.shutdown()
        httpd.server_close()


# ------------------------------------------------------------------ I3
def test_both_time_bases_are_named_and_open_folders_show_as_running(tmp_path):
    rf = runfolder.RunFolder(tmp_path, "move_test", "mock")
    rf.write_json("run.json", {"run_id": "x", "stage": "move_test", "mode": "mock", "status": "running"})
    assert runfolder.list_runs(tmp_path)[0]["result"] == "RUNNING or not finished"      # before: DID NOT PASS
    rf.finish({"stage": "move_test", "mode": "mock", "auto_ok": True})
    s = (rf.path / "summary.txt").read_text(encoding="utf-8")
    assert "are local time" in s and "use UTC" in s


# ------------------------------------------------------------------ I4
def test_malformed_arm_control_commands_are_refused_and_the_session_goes_on(tmp_path):
    b = arm_session(tmp_path)
    try:
        for name, args in (("jog", {"axis": "x", "dir": "up", "step_mm": 5}),
                           ("jog", {"axis": "x", "dir": 1, "step_mm": "five"}),
                           ("goto", {}),
                           ("goto", {"point": {"name": "p", "pose": [1.0, 2.0, 3.0]}}),
                           ("path", {"points": [{"name": "p", "pose": [float("nan")] * 6}]}),
                           ("path", {"points": "S,M,E"})):
            r = do(b, name, args)
            assert r is not None and not r["ok"], (name, args, r)          # before: the session ended
            assert b.run.running, (name, args)
        do(b, "end", {})
        run = wait_run(b)
        assert run["exit_code"] == 0
        sent = [r["data"].decode().split(",", 1)[0] for r in read_trial(run["trial_path"]).stream("wire_tx")]
        assert "WayPoint" not in sent and sent[-1] == "GrpDisable"           # none of them moved the arm
    finally:
        if b.run.running:
            b.run.stop()
            b.run.thread.join(timeout=30)


def test_unusable_arm_control_settings_refuse_the_session_before_enabling(tmp_path):
    confirmed, arm = temp_configs(tmp_path)
    b = Workbench(data_dir=tmp_path / "data", journal=tmp_path / "journal.md", confirmed=confirmed, arm_config=arm,
                  host_problem="this is a test, not the Linux machine")
    v = b.coeffs.snapshot_path()
    snap = json.loads(v.read_text(encoding="utf-8"))
    snap["values"]["jog_box_mm"]["value"] = [200.0]                     # one value instead of three
    v.write_text(json.dumps(snap), encoding="utf-8")
    b.start("arm_control", "mock", assume_yes=True)
    run = wait_run(b)
    assert run["exit_code"] != 0 and any("jog_box_mm" in line for line in run["console_tail"])
    assert b"GrpEnable" not in b"".join(r["data"] for r in read_trial(run["trial_path"]).stream("wire_tx"))


# ------------------------------------------------------------------ I5
def test_recovered_runs_get_their_journal_line(bench):
    rf = runfolder.RunFolder(bench.runs_dir, "servo_ladder", "mock")      # left open by a workbench that died
    rf._console.close()
    bench.recover_runs(quiet_s=0)
    text = bench.status.journal_path.read_text(encoding="utf-8")
    assert f"run_folder {rf.name}" in text and "interrupted: the workbench stopped during the run" in text


def test_the_docs_the_review_found_stale():
    readme = (ROOT / "README.md").read_text(encoding="utf-8")
    assert "analysis/cup.py" not in readme and "3.11.9" in readme
    assert "**[proposal]**" in (ROOT / "docs" / "plans" / "README.md").read_text(encoding="utf-8")
    h9 = [ln for ln in (ROOT / "docs" / "open_questions.md").read_text(encoding="utf-8").splitlines()
          if ln.startswith("| H9 |")][0]
    assert "stage 9.2" in h9
    jog = (ROOT / "tools" / "elfin_py" / "jog.py").read_text(encoding="utf-8")
    assert "reading needs no enable (PROTO" not in jog
    assert "0.8 MB per hour" not in (ROOT / "docs" / "journal.md").read_text(encoding="utf-8").replace(
        "first written here as 0.8", "")
