"""The pre-hardware review's fixes on the workbench side (2026-11-18): S5, a dead workbench whose pipe nobody reads;
S6, practice runs never count; S7, the stop and the reader survive a failing run folder or a blocked pipe; S9, a
question that times out is withdrawn from the page."""
import io
import json
import os
import subprocess
import sys
import threading
import time

import pytest

from analysis.logread import read_trial
from tests.workbench.test_channel_and_tools import Pipe
from tests.workbench.test_workbench import ROOT, bench, temp_configs, wait_run  # noqa: F401 (bench is a fixture)
from tools.elfin_py import channel
from tools.workbench.bench import Refused, Workbench
from tools.workbench.runner import MockProcess, ToolRun


# ------------------------------------------------------------------ S5: the workbench dies; its pipe has no reader
FAKE_WORKBENCH_PIPE = r"""
import os, subprocess, sys, threading, time, pathlib
argv, env, flag = sys.argv[1:-1], dict(os.environ, SLOSH_WORKBENCH="1"), pathlib.Path(sys.argv[-1])
kw = {"creationflags": subprocess.CREATE_NEW_PROCESS_GROUP} if os.name == "nt" else {"start_new_session": True}
p = subprocess.Popen(argv, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, env=env, **kw)
flag.write_text(str(p.pid))
threading.Thread(target=lambda: [None for _ in p.stdout], daemon=True).start()   # reads while it lives
time.sleep(float(os.environ["DIE_AFTER"]))
os._exit(1)          # the workbench crashes: the tool's stdin closes, and its stdout has no reader any more
"""


def test_a_dead_workbench_with_a_real_pipe_still_gets_the_deceleration(tmp_path):
    confirmed, arm = temp_configs(tmp_path)
    mock = MockProcess(ROOT, arm)
    try:
        script = tmp_path / "fake_workbench.py"
        script.write_text(FAKE_WORKBENCH_PIPE, encoding="utf-8")
        tool = [sys.executable, "-u", "-m", "tools.elfin_py.servo_ladder", "--mock",
                "--cmd-port", str(mock.ports[0]), "--state-port", str(mock.ports[1]), "--assume-yes",
                "--confirmed", str(confirmed), "--arm-config", str(arm), "--data-dir", str(tmp_path / "data")]
        env = dict(os.environ, DIE_AFTER="4", PYTHONUNBUFFERED="1")
        r = subprocess.run([sys.executable, str(script), *tool, str(tmp_path / "tool.pid")], cwd=ROOT, env=env,
                           timeout=60)
        assert r.returncode == 1
        trials = sorted((tmp_path / "data" / "trials").glob("*.bin"))
        assert len(trials) == 1
        t_end = time.monotonic() + 60                              # the orphaned tool finishes on its own
        while time.monotonic() < t_end:
            tr = read_trial(trials[0])
            sent = [x["data"].decode().split(",", 1)[0] for x in tr.stream("wire_tx")]
            if sent and sent[-1] == "GrpDisable":
                break
            time.sleep(0.2)
        assert sent[-1] == "GrpDisable", sent[-5:]
        recs = tr.records
        k = next(i for i, (s, x) in enumerate(recs)
                 if s == "event" and x["text"].decode().startswith("streamed deceleration: operator stop"))
        assert sum(1 for s, _ in recs[k:] if s == "arm_cmd") >= 2    # the ramp went out after the failed print
        texts = [x["text"].decode() for s, x in recs if s == "event"]
        assert any("stop requested: workbench channel closed" in t for t in texts)
    finally:
        mock.stop()


# ------------------------------------------------------------------ S6: practice never counts
def test_a_practice_run_cannot_be_accepted(tmp_path):
    confirmed, arm = temp_configs(tmp_path, limits=False)
    b = Workbench(data_dir=tmp_path / "data", journal=tmp_path / "journal.md", confirmed=confirmed, arm_config=arm,
                  host_problem="")
    b.start("move_test", "dry-run", assume_yes=True, practice=True)
    run = wait_run(b)
    assert run["practice"] and run["auto_ok"], run["console_tail"]
    with pytest.raises(Refused, match="practice"):
        b.accept(run["run_id"], "it should not count")
    assert b.status.status("move_test", "dry-run")["status"] == "not run"
    assert any("has not passed in dry-run" in l["reason"] for l in b.status.locks("move_test", "arm", None))


def test_an_accepted_result_never_stands_in_for_a_rehearsal(bench):
    made = {"run_id": "hand-made", "stage": "move_test", "mode": "mock", "metrics": {},
            "inputs_hash": bench.status.inputs_hash("move_test")}
    bench.status.accept(made, "as an accepted practice run would have been", {}, [])
    assert bench.status.status("move_test", "mock")["status"] == "not run"
    assert any("has not passed in mock" in l["reason"] for l in bench.status.locks("move_test", "arm", None))


# ------------------------------------------------------------------ S7: the reader and the stop never wait on a write
class FailingSink:
    def __getattr__(self, name):
        def fail(*a, **k):
            raise OSError(28, "No space left on device")
        return fail


CHATTY = r'''
import sys
for k in range(2000):                      # about 100 kB: more than a pipe holds
    print(f"line {k} " + "x" * 40)
sys.stdout.flush()
print("got " + sys.stdin.readline().strip(), flush=True)
'''


def test_the_reader_goes_on_when_the_run_folder_cannot_be_written():
    run = ToolRun("r1", "x", "mock", [sys.executable, "-u", "-c", CHATTY], dict(os.environ), ROOT, sink=FailingSink())
    t_end = time.monotonic() + 30
    while not any(l.startswith("line 1999 ") for l in list(run.console)[-5:]) and time.monotonic() < t_end:
        time.sleep(0.05)
    assert run.stop()
    run.thread.join(timeout=30)
    assert run.exit_code == 0 and not run.running
    console = list(run.console)
    assert sum("RUN FOLDER WRITE FAILED" in l for l in console) == 1
    assert any(l.startswith('got {"type": "stop"}') for l in console)


class BlockedOut(io.TextIOBase):
    """A stdout whose writes hang, as on a full pipe nobody reads."""

    def __init__(self):
        self.release = threading.Event()

    def write(self, s):
        self.release.wait()
        return len(s)

    def flush(self):
        pass


def test_the_stop_reaches_the_tool_even_when_its_output_is_blocked():
    stops, out, pipe = [], BlockedOut(), Pipe()
    ch = channel.Channel(stdin=pipe, stdout=out, interrupt=lambda: stops.append(time.monotonic()))
    threading.Thread(target=lambda: ch.emit("pose", x=1), daemon=True).start()   # stuck, holding the write lock
    time.sleep(0.1)
    pipe.put({"type": "stop"})
    t_end = time.monotonic() + 3
    while not stops and time.monotonic() < t_end:
        time.sleep(0.02)
    out.release.set()
    assert len(stops) == 1 and ch.stopped == "stop button"


# ------------------------------------------------------------------ S9: a question that times out leaves the page
def test_a_question_that_times_out_is_withdrawn():
    stops, out, pipe = [], io.StringIO(), Pipe()
    ch = channel.Channel(stdin=pipe, stdout=out, interrupt=lambda: stops.append(1))
    t0 = time.monotonic()
    assert ch.ask("Move to pose 2?", timeout=0.5) is None
    assert 0.4 < time.monotonic() - t0 < 3 and not stops and not ch.stopped
    msgs = [json.loads(line[len(channel.PREFIX):]) for line in out.getvalue().splitlines()]
    assert [m["type"] for m in msgs] == ["ask", "withdrawn"] and msgs[0]["id"] == msgs[1]["id"]


ASKS_THEN_WITHDRAWS = r'''
import json, time
P = "@@wb "
print(P + json.dumps({"type": "ask", "id": 1, "question": "Move to pose 2?"}), flush=True)
time.sleep(1.0)
print(P + json.dumps({"type": "withdrawn", "id": 1, "reason": "no answer within 0.5 s: taken as no"}), flush=True)
time.sleep(0.5)
'''


def test_the_page_drops_a_withdrawn_question():
    run = ToolRun("r2", "x", "mock", [sys.executable, "-u", "-c", ASKS_THEN_WITHDRAWS], dict(os.environ), ROOT)
    t_end = time.monotonic() + 10
    while not run.questions and time.monotonic() < t_end:
        time.sleep(0.02)
    assert run.questions == {1: "Move to pose 2?"}
    while run.questions and time.monotonic() < t_end:
        time.sleep(0.02)
    assert run.questions == {} and any("[question 1 withdrawn]" in l for l in run.console)
    run.thread.join(timeout=10)
    assert run.exit_code == 0


# ------------------------------------------------------------------ S14: arm runs use the owner's config files only
def test_the_workbench_refuses_arm_runs_with_other_config_files(tmp_path):
    confirmed, arm = temp_configs(tmp_path)
    b = Workbench(data_dir=tmp_path / "data", journal=tmp_path / "journal.md", confirmed=confirmed, arm_config=arm,
                  host_problem="")                                  # pretend this is the Linux machine
    with pytest.raises(Refused, match="config/arm/confirmed.yaml"):
        b.start("bringup_read", "arm", override_reason="testing the refusal")
    assert not [a for a in b.status.audit_log() if a["kind"] == "override"]     # refused before the override


# ------------------------------------------------------------------ S15: only this machine's page may use the API
def test_the_api_refuses_other_websites(bench):
    import http.client
    from tools.workbench.server import serve
    httpd = serve(bench, port=0)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    port = httpd.server_address[1]
    own = {"Host": f"127.0.0.1:{port}", "X-Slosh-Workbench": "1"}

    def call(method, path, headers, body=None):
        c = http.client.HTTPConnection("127.0.0.1", port, timeout=10)
        try:
            c.request(method, path, body=body, headers=headers)
            r = c.getresponse()
            return r.status, r.read()
        finally:
            c.close()
    try:
        assert call("GET", "/api/overview", own)[0] == 200
        assert call("GET", "/", {"Host": f"127.0.0.1:{port}"})[0] == 200                    # the page itself
        assert call("GET", "/", {"Host": f"evil.example:{port}"})[0] == 403                 # DNS rebinding
        assert call("GET", "/api/overview", {"Host": f"localhost:{port}"})[0] == 403        # no page header
        js = dict(own, **{"Content-Type": "application/json"})
        assert call("POST", "/api/stop", js, b"{}")[0] == 409                               # allowed: nothing runs
        assert call("POST", "/api/stop", dict(own, **{"Content-Type": "text/plain"}), b"{}")[0] == 403
        assert call("POST", "/api/stop", {"Host": own["Host"], "Content-Type": "application/json"}, b"{}")[0] == 403
        assert call("POST", "/api/stop", dict(js, Origin="https://evil.example"), b"{}")[0] == 403
        assert call("POST", "/api/stop", dict(js, Origin=f"http://127.0.0.1:{port}"), b"{}")[0] == 409
        assert call("POST", "/api/stop", dict(js, **{"Sec-Fetch-Site": "cross-site"}), b"{}")[0] == 403
        assert call("POST", "/api/stop", js, b"[]")[0] == 400                               # not a JSON object
    finally:
        httpd.shutdown()
        httpd.server_close()


# ------------------------------------------------------------------ S16: answers and commands belong to their run
def test_an_answer_for_an_earlier_run_is_refused(bench):
    bench.start("move_test", "mock")
    t_end = time.monotonic() + 30
    while not bench.run.questions and time.monotonic() < t_end:
        time.sleep(0.05)
    qid = next(iter(bench.run.questions))
    with pytest.raises(Refused, match="out of date"):
        bench.answer(qid, "yes", run_id="20261118-000000-move_test-mock")
    bench.answer(qid, "no", run_id=bench.run.run_id)              # the current run's id: accepted
    run = wait_run(bench)
    assert not run["auto_ok"]                                     # it was told no and refused to go on


def arm_session(tmp_path):
    confirmed, arm = temp_configs(tmp_path)
    b = Workbench(data_dir=tmp_path / "data", journal=tmp_path / "journal.md", confirmed=confirmed, arm_config=arm,
                  host_problem="this is a test, not the Linux machine")
    b.edit_coeffs({"jog_speed_mm_s": 5.0, "jog_acc_mm_s2": 100.0}, "slow, for the tests")
    b.start("arm_control", "mock", assume_yes=True)
    t_end = time.monotonic() + 30
    while b.run.arm["ready"] is None and b.run.running and time.monotonic() < t_end:
        time.sleep(0.05)
    assert b.run.arm["ready"] is not None, list(b.run.console)[-10:]
    return b


def test_a_second_click_cannot_send_a_second_move_and_stale_commands_are_refused(tmp_path):
    b = arm_session(tmp_path)
    try:
        with pytest.raises(Refused, match="out of date"):
            b.command("jog", {"axis": "x", "dir": 1, "step_mm": 5}, run_id="20261118-000000-arm_control-mock")
        b.command("jog", {"axis": "x", "dir": 1, "step_mm": 5}, run_id=b.run.run_id)
        with pytest.raises(Refused, match="under way"):            # at once, before the tool reports busy
            b.command("jog", {"axis": "x", "dir": 1, "step_mm": 5}, run_id=b.run.run_id)
        assert b.stop(run_id="20261118-000000-some-older-run")      # a stop always stops the current run
        b.run.thread.join(timeout=30)
        assert not b.run.running
        trial = read_trial(b.run.trial_path)
        sent = [r["data"].decode().split(",", 1)[0] for r in trial.stream("wire_tx")]
        assert sent.count("WayPoint") <= 1 and sent[-1] == "GrpDisable"     # never a second move; the stop worked
    finally:
        if b.run.running:
            b.run.stop()
            b.run.thread.join(timeout=30)


# ------------------------------------------------------------------ S19: a run is running until it is saved
def test_a_run_counts_as_running_until_it_is_saved():
    saved = threading.Event()

    def on_exit(run):
        time.sleep(1.0)                                            # as stopping the mock and writing the folder take
        saved.set()
    run = ToolRun("r3", "x", "mock", [sys.executable, "-c", "pass"], dict(os.environ), ROOT, on_exit=on_exit)
    t_end = time.monotonic() + 10
    while run.exit_code is None and time.monotonic() < t_end:
        time.sleep(0.01)
    assert run.exit_code == 0 and run.running and not saved.is_set()
    run.thread.join(timeout=10)
    assert saved.is_set() and not run.running
