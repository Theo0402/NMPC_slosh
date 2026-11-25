"""The workbench (codebase plan §12, §9 workbench row): registry, coefficient versions, staleness, locks,
confirmed.yaml, and the stop paths against the mock. Never touches config/arm/confirmed.yaml: every
test works on temporary copies."""
import json
import os
import pathlib
import subprocess
import sys
import time

import pytest

from analysis.logread import read_trial
from tools.workbench import gen_docs
from tools.workbench.bench import CONFIRM_PHRASE, Refused, Workbench
from tools.workbench.coeffs import CoeffError
from tools.workbench.stages import Registry, RegistryError

ROOT = pathlib.Path(__file__).resolve().parents[2]
MOCK_LIMITS = """limits_readback:
  record_id: test
  joint_max_vel_deg_s: [170.0, 170.0, 170.0, 170.0, 180.0, 180.0]
  joint_max_acc_deg_s2: [360.0, 360.0, 360.0, 360.0, 360.0, 360.0]
  joint_max_jerk_deg_s3: [3600.0, 3600.0, 3600.0, 3600.0, 3600.0, 3600.0]
  linear_max: [1000.0, 2500.0, 10000000.0]
  tolerance: 0.001
"""
NO_LIMITS = """limits_readback:
  record_id: null
  joint_max_vel_deg_s: null
  joint_max_acc_deg_s2: null
  joint_max_jerk_deg_s3: null
  linear_max: null
  tolerance: 0.001
"""                                                    # e03pro.yaml as it was before stage 1.1 read the arm


def temp_configs(tmp_path, limits=True, period=0.1):
    arm = (ROOT / "config/arm/e03pro.yaml").read_text(encoding="utf-8")
    start = arm.index("limits_readback:")
    end = arm.index("\n\n", start)
    arm = arm[:start] + (MOCK_LIMITS if limits else NO_LIMITS).rstrip("\n") + arm[end:]
    conf = (ROOT / "config/arm/confirmed.yaml").read_text(encoding="utf-8")
    conf = conf.replace("servo_period_s: 0.2 ", f"servo_period_s: {period} ")
    arm = arm.replace("period_s: 0.2 ", f"period_s: {max(period, 0.1)} ")
    (tmp_path / "e03pro.yaml").write_text(arm, encoding="utf-8")
    (tmp_path / "confirmed.yaml").write_text(conf, encoding="utf-8")
    (tmp_path / "journal.md").write_text("# Journal\n\n- **W1 d1.** test\n", encoding="utf-8")
    return tmp_path / "confirmed.yaml", tmp_path / "e03pro.yaml"


@pytest.fixture
def bench(tmp_path):
    confirmed, arm = temp_configs(tmp_path)
    return Workbench(data_dir=tmp_path / "data", journal=tmp_path / "journal.md", confirmed=confirmed,
                     arm_config=arm, host_problem="this is a test, not the Linux machine")


def wait_run(b, timeout=90):
    t_end = time.monotonic() + timeout
    while b.run.running and time.monotonic() < t_end:
        time.sleep(0.1)
    assert not b.run.running, "the tool did not finish"
    b.run.thread.join(timeout=10)
    for _ in range(50):                      # _finish runs on the reader thread after the exit
        if b.status.run(b.run.run_id):
            return b.status.run(b.run.run_id)
        time.sleep(0.1)
    raise AssertionError("no run record")


# ---------------------------------------------------------------------------------------- registry
def test_registry_is_a_valid_dag_with_plan_sections():
    reg = Registry()
    assert len(reg.order) == len(set(reg.order))
    order = reg.topological()
    for sid in reg.order:
        s = reg.stages[sid]
        assert s["plan"] and s["output"] and s["purpose"]
        for r in s["requires"]:
            assert order.index(r) < order.index(sid)
    # every stage of the request's Part 2 is there, in its group
    nums = {reg.stages[s]["num"] for s in reg.order}
    for n in ("0.2", "1.1", "1.4", "2.1", "2.2", "3.1", "3.3", "4.1", "4.5", "5.1", "6.5", "7.1", "8.1", "9.2"):
        assert n in nums


def test_registry_refuses_a_cycle(tmp_path):
    text = (ROOT / "config/workbench/stages.yaml").read_text(encoding="utf-8")
    text = text.replace("    requires: []\n    inputs:\n      - The Linux machine", "    requires: [teardown]\n    inputs:\n      - The Linux machine", 1)
    text = text.replace("    requires: []\n    adjust: []\n    inputs:\n      - Nothing running", "    requires: [host_check]\n    adjust: []\n    inputs:\n      - Nothing running", 1)
    p = tmp_path / "stages.yaml"
    p.write_text(text, encoding="utf-8")
    with pytest.raises(RegistryError, match="cycle"):
        Registry(p)


def test_stage_reference_is_up_to_date():
    assert gen_docs.main(["--check"]) == 0, "run python -m tools.workbench.gen_docs"


def test_gates_lock_but_do_not_make_results_stale():
    reg = Registry()
    assert reg.stages["preflight"]["gate"] and reg.stages["host_check"]["gate"]
    assert reg.stale_after(["fill_tolerance_mm"]) == ["preflight", "meniscus_map"]
    assert "fill_by_weight" not in reg.stages          # the owner measures the fill and enters it
    assert "servo_ladder" not in reg.stale_after(["water_temp_range_c"])
    assert "camera_gate" in reg.stale_after(["reproj_rms_max_px"])


# ---------------------------------------------------------------------------------------- coefficients
def test_coefficient_versions_diff_revert_and_ranges(bench):
    c = bench.coeffs
    assert c.current_version() == 1
    assert c.value("zeta_target") == 0.3 and c.value("n_valid_min") == 16
    with pytest.raises(CoeffError, match="note"):
        c.edit({"zeta_target": 0.25}, "")
    with pytest.raises(CoeffError, match="outside the allowed range"):
        c.edit({"zeta_target": 0.5}, "too much")
    with pytest.raises(CoeffError, match="lives in"):
        c.edit({"servo_period_s": 0.1}, "never here")          # confirmed.yaml is the owner's file
    r = bench.edit_coeffs({"zeta_target": 0.25}, "try less damping")
    assert r["version"] == 2 and r["diff"][0]["old"] == 0.3 and r["diff"][0]["new"] == 0.25
    assert "corrector" in r["stale"]
    assert c.snapshot(2)["values"]["zeta_target"]["source"] == "hand-set"
    back = bench.revert_coeffs(1, "back to the plan")
    assert back["version"] == 3 and c.value("zeta_target") == 0.3
    assert [v["version"] for v in c.versions()] == [1, 2, 3]
    assert json.loads(c.snapshot_path(2).read_text())["values"]["zeta_target"]["value"] == 0.25   # never rewritten
    journal = bench.status.journal_path.read_text(encoding="utf-8")
    assert "## Runs (written by the workbench)" in journal and "try less damping" in journal
    assert journal.index("- **W1 d1.** test") < journal.index("## Runs")


def test_file_and_code_homes_are_read_only_views(bench):
    assert bench.coeffs.value("servo_period_s") == 0.1          # from the temporary confirmed.yaml
    assert bench.coeffs.value("ladder_late_limit") == 0.01      # from the servo ladder's code
    assert bench.coeffs.value("cup_inner_radius_m") == 0.040
    rows = {r["name"]: r for r in bench.coeffs.table()["rows"]}
    assert not rows["servo_period_s"]["editable"] and rows["zeta_target"]["editable"]


PREFLIGHT = {"estop_test": True, "workspace_clear": True, "tray_shield": True, "cables": True, "holder_bolts": True,
             "lighting_fixed": True, "lighting_setup": "L1", "payload_g": 760.0, "fill_height_mm": 60.2,
             "water_temp_c": 21.5}


def test_preflight_takes_the_measured_fill_and_goes_stale_on_a_change(bench):
    res = bench.submit_form("preflight", PREFLIGHT, "bench ready", override_reason="test: no host check here")
    assert res["result"] is not None, res["run"]["criteria"]
    assert res["run"]["metrics"]["fill_height_mm"] == 60.2 and res["run"]["metrics"]["fill_fraction"] == 0.602
    assert bench.status.status("preflight", "form")["status"] == "passed"
    bench.edit_coeffs({"water_temp_range_c": [18.0, 28.0]}, "tighter")
    assert bench.status.status("preflight", "form")["status"] == "stale"


def test_preflight_needs_the_fill_height(bench):
    values = dict(PREFLIGHT, fill_height_mm=None)
    res = bench.submit_form("preflight", values, "forgot the fill", override_reason="test")
    assert res["result"] is None
    assert any(c["id"] == "fill_height_mm" and not c["ok"] for c in res["run"]["criteria"])


# ---------------------------------------------------------------------------------------- locks
def test_arm_mode_is_refused_off_the_linux_machine(bench):
    with pytest.raises(Refused, match="not the Linux machine"):
        bench.start("bringup_read", "arm", override_reason="I insist")
    with pytest.raises(Refused, match="never auto-answered"):
        bench.start("bringup_read", "arm", assume_yes=True)


def test_arm_run_needs_prerequisites_and_rehearsals(tmp_path):
    confirmed, arm = temp_configs(tmp_path)
    b = Workbench(data_dir=tmp_path / "data", journal=tmp_path / "journal.md", confirmed=confirmed, arm_config=arm,
                  host_problem="")                                # pretend this is the Linux machine
    locks = [l["reason"] for l in b.status.locks("servo_ladder", "arm", None)]
    assert any("move_test has not passed" in r for r in locks)
    assert any("not passed in dry-run" in r for r in locks) and any("not passed in mock" in r for r in locks)
    assert b.status.locks("servo_ladder", "mock", None) == []      # rehearsals are never locked
    with pytest.raises(Refused, match="override needs a reason"):
        b.start("servo_ladder", "arm")


def test_planned_stages_cannot_run(bench):
    with pytest.raises(Refused, match="planned"):
        bench.start("killtest1", "mock")


# ---------------------------------------------------------------------------------------- confirmed.yaml
def test_confirmed_yaml_changes_only_with_the_typed_phrase(bench):
    before = bench.confirmed_path.read_text(encoding="utf-8")
    with pytest.raises(Refused, match="type the phrase"):
        bench.change_confirmed({"servo_period_s": 0.05}, "yes", "smooth at 0.1 s")
    with pytest.raises(Refused, match="previous step"):
        bench.change_confirmed({"servo_period_s": 0.05}, CONFIRM_PHRASE, "")
    with pytest.raises(Refused, match="positive number"):
        bench.change_confirmed({"servo_period_s": -1}, CONFIRM_PHRASE, "x")
    assert bench.confirmed_path.read_text(encoding="utf-8") == before
    r = bench.change_confirmed({"servo_period_s": 0.05}, CONFIRM_PHRASE, "0.1 s: 0 late, median 3 ms, smooth")
    after = bench.confirmed_path.read_text(encoding="utf-8")
    assert "servo_period_s: 0.05 " in after and "ONLY THE OWNER EDITS THIS FILE" in after   # comments kept
    assert r["old"]["servo_period_s"] == 0.1 and "git commit" in r["commit"]
    audit = bench.status.audit_log()
    assert audit[-1]["kind"] == "confirmed_change" and "0.1 s: 0 late" in audit[-1]["note"]
    assert pathlib.Path(bench.confirmed_path).parent != ROOT / "config" / "arm"


# ---------------------------------------------------------------------------------------- runs on the mock
def test_mock_bringup_read_through_the_channel_and_accept(bench):
    bench.start("bringup_read", "mock", note="workbench test")
    run = wait_run(bench)
    assert run["exit_code"] == 0, run["console_tail"]
    assert run["auto_ok"], run["criteria"]
    trial = read_trial(run["trial_path"])
    assert trial.header["workbench"]["stage"] == "bringup_read"
    assert trial.header["workbench"]["coeff_version"] == 1
    status = bench.status.status("bringup_read", "mock")
    assert status["status"] == "passed"                            # a rehearsal needs no accept
    with pytest.raises(Refused, match="note"):
        bench.accept(run["run_id"], "")
    with pytest.raises(Refused, match="unticked"):
        bench.accept(run["run_id"], "limits pasted")                # the operator criterion
    res = bench.accept(run["run_id"], "mock limits", {"ranges": True})
    assert res["result_id"] == "bringup_read/0001"


def test_dry_run_mirrors_the_config(bench):
    bench.start("move_test", "dry-run", assume_yes=True)
    run = wait_run(bench)
    assert run["auto_ok"], run["console_tail"]


def test_questions_go_through_the_channel(bench):
    bench.start("move_test", "mock")
    t_end = time.monotonic() + 30
    answered = 0
    while bench.run.running and time.monotonic() < t_end:
        for q in list(bench.run.questions.items()):
            bench.answer(q[0], "yes")
            answered += 1
        time.sleep(0.05)
    run = wait_run(bench)
    assert answered >= 1 and run["auto_ok"], run["console_tail"]


def test_stop_button_decelerates_the_mock_ladder_and_disables(bench):
    bench.start("servo_ladder", "mock", assume_yes=True)
    t_end = time.monotonic() + 30
    while time.monotonic() < t_end:                                # wait until it streams
        if bench.run.live:
            bench.run.live.poll()
            if bench.run.live.snapshot()["n_cmd"] >= 15:
                break
        time.sleep(0.05)
    assert bench.run.live.snapshot()["n_cmd"] >= 15
    bench.stop()
    run = wait_run(bench)
    assert run["stopped"] and not run["auto_ok"]
    trial = read_trial(run["trial_path"])
    texts = [r["text"].decode() for r in trial.stream("event")]
    assert any("stop requested: stop button" in t for t in texts)
    assert any("streamed deceleration: operator stop" in t for t in texts)
    sent = [r["data"].decode().split(",", 1)[0] for r in trial.stream("wire_tx")]
    assert sent[-1] == "GrpDisable"
    assert len([s for s in sent if s == "PushServoJ"]) < 230       # it did not run to the end


FAKE_WORKBENCH = r"""
import os, subprocess, sys, time, pathlib
argv, env, flag = sys.argv[1:-1], dict(os.environ, SLOSH_WORKBENCH="1"), pathlib.Path(sys.argv[-1])
kw = {"creationflags": subprocess.CREATE_NEW_PROCESS_GROUP} if os.name == "nt" else {"start_new_session": True}
out = open(flag.with_suffix(".out"), "w")
p = subprocess.Popen(argv, stdin=subprocess.PIPE, stdout=out, stderr=subprocess.STDOUT, env=env, **kw)
flag.write_text(str(p.pid))
time.sleep(float(os.environ["DIE_AFTER"]))
os._exit(1)          # the workbench crashes: the OS closes the tool's stdin
"""


def test_a_dead_workbench_stops_the_tool_safely(tmp_path):
    from tools.workbench.runner import MockProcess
    confirmed, arm = temp_configs(tmp_path)
    mock = MockProcess(ROOT, arm)
    try:
        script = tmp_path / "fake_workbench.py"
        script.write_text(FAKE_WORKBENCH, encoding="utf-8")
        flag = tmp_path / "tool.pid"
        tool = [sys.executable, "-u", "-m", "tools.elfin_py.servo_ladder", "--mock",
                "--cmd-port", str(mock.ports[0]), "--state-port", str(mock.ports[1]), "--assume-yes",
                "--confirmed", str(confirmed), "--arm-config", str(arm), "--data-dir", str(tmp_path / "data")]
        env = dict(os.environ, DIE_AFTER="4", PYTHONUNBUFFERED="1")
        r = subprocess.run([sys.executable, str(script), *tool, str(flag)], cwd=ROOT, env=env, timeout=60)
        assert r.returncode == 1                                   # the fake workbench died
        trials = sorted((tmp_path / "data" / "trials").glob("*.bin"))
        assert len(trials) == 1
        t_end = time.monotonic() + 60                              # the orphaned tool finishes on its own
        while time.monotonic() < t_end:
            tr = read_trial(trials[0])
            sent = [x["data"].decode().split(",", 1)[0] for x in tr.stream("wire_tx")]
            if sent and sent[-1] == "GrpDisable":
                break
            time.sleep(0.2)
        texts = [x["text"].decode() for x in tr.stream("event")]
        assert any("stop requested: workbench channel closed" in t for t in texts)
        assert any("streamed deceleration: operator stop" in t for t in texts)
        assert sent[-1] == "GrpDisable", sent[-5:]
    finally:
        mock.stop()
