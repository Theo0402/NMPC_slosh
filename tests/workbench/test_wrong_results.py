"""The pre-hardware review's wrong-result fixes on the workbench side (2026-11-18): W1 the gate's inputs and
front-end, W4 staleness along the whole chain, W5 no older pass after a newer failure, W6 apostrophes in the
coefficient editor, W7 complete coefficient snapshots, W9 stage 1.6's criteria from the trial log."""
import json
import pathlib
import re

import pytest

from analysis.logread import read_trial
from tests.analysis.test_analysis import gate_doc
from tests.workbench.test_arm_control import do
from tests.workbench.test_safety_workbench import arm_session
from tests.workbench.test_workbench import ROOT, bench, wait_run  # noqa: F401 (bench is a fixture)
from tools.workbench.bench import Refused
from tools.workbench.stages import primary_mode


# ------------------------------------------------------------------ W1
def test_editing_the_gate_input_makes_the_gate_stale(bench):
    p = bench.data_dir / "workbench" / "gate_input.json"
    h0 = bench.status.inputs_hash("camera_gate")
    p.write_text(json.dumps(gate_doc()), encoding="utf-8")
    h1 = bench.status.inputs_hash("camera_gate")
    p.write_text(json.dumps(dict(gate_doc(), front_end="cnn")), encoding="utf-8")
    assert len({h0, h1, bench.status.inputs_hash("camera_gate")}) == 3


def test_gate_memo_v2_counts_only_after_the_cnn_stage(bench):
    assert bench._analysis_metrics("camera_gate", {"front_end": "classical"})["front_end_accepted"] is True
    assert bench._analysis_metrics("camera_gate", {"front_end": "cnn"})["front_end_accepted"] is False
    assert "camera_gate" not in bench.reg.stages["cnn_frontend"]["requires"]      # no deadlock on the CNN path
    assert "camera_gate" in bench.reg.stages["pickup_shake"]["requires"]


# ------------------------------------------------------------------ W4
def test_staleness_follows_the_whole_chain_of_prerequisites(bench):
    sid = "servo_ladder"
    far = [u for u in bench.reg.upstream(sid) if u not in bench.reg.stages[sid]["requires"]
           and not bench.reg.stages[u]["gate"]]
    assert far, "servo_ladder has prerequisites more than one step up"
    h0 = bench.status.inputs_hash(sid)
    up = far[0]
    bench.status.accept({"run_id": "hand-made", "stage": up, "mode": primary_mode(bench.reg.stages[up]),
                         "inputs_hash": "x", "metrics": {}}, "a new result two steps up", {}, [])
    assert bench.status.inputs_hash(sid) != h0 and up in bench.status.inputs(sid)["upstream"]


# ------------------------------------------------------------------ W5
def test_an_older_pass_cannot_be_accepted_after_a_newer_failure(bench):
    h = bench.status.inputs_hash("bringup_read")
    good = {"problems": 0, "limits_read": True, "limits_in_config": True}      # it met every criterion
    ok = {"run_id": "20261118-100000-bringup_read-mock", "stage": "bringup_read", "mode": "mock", "auto_ok": True,
          "started_utc": "2026-11-18T10:00:00+00:00", "inputs_hash": h, "metrics": good}
    bad = dict(ok, run_id="20261118-110000-bringup_read-mock", auto_ok=False, started_utc="2026-11-18T11:00:00+00:00",
               metrics={"problems": 3})
    bench.status.save_run(ok)
    bench.status.save_run(bad)
    with pytest.raises(Refused, match="a later mock run of this stage did not pass"):
        bench.accept(ok["run_id"], "the earlier pass", {"ranges": True})
    bench.status.save_run(dict(bad, run_id="20261118-090000-bringup_read-mock", started_utc="2026-11-18T09:00:00+00:00"))
    bench.status._runs = None
    later_ok = dict(ok, run_id="20261118-120000-bringup_read-mock", started_utc="2026-11-18T12:00:00+00:00")
    bench.status.save_run(later_ok)
    assert bench.accept(later_ok["run_id"], "the newest run passed", {"ranges": True})["result_id"]   # earlier fails: fine


# ------------------------------------------------------------------ W6
def test_the_coefficient_editor_keeps_apostrophes():
    html = (ROOT / "tools" / "workbench" / "static" / "index.html").read_text(encoding="utf-8")
    esc = re.search(r"const esc = .*", html).group(0)
    assert "'" in esc.split("replace(")[1].split(",")[0] and "&#39;" in esc          # escaped in every attribute
    assert "value='${esc(" not in html                                              # and no single-quoted values


# ------------------------------------------------------------------ W7
def test_a_run_gets_every_coefficient_even_ones_newer_than_its_version(bench):
    v = bench.coeffs.snapshot_path()
    snap = json.loads(v.read_text(encoding="utf-8"))
    snap["values"].pop("jog_speed_mm_s")                                            # as in data/'s v000001
    v.write_text(json.dumps(snap), encoding="utf-8")
    full = bench.coeffs.complete_snapshot()
    assert full["filled_from_schema"] == ["jog_speed_mm_s"]
    assert full["values"]["jog_speed_mm_s"]["value"] == 10.0 and "schema default" in full["values"]["jog_speed_mm_s"]["source"]
    bench.start("move_test", "dry-run", assume_yes=True)
    run = wait_run(bench)
    coeffs = read_trial(run["trial_path"]).header["workbench"]["coeffs"]
    assert coeffs["jog_speed_mm_s"]["value"] == 10.0                               # the header carries it now
    assert (pathlib.Path(bench.runs_dir) / run["run_folder"] / f"coeffs_v{snap['version']:06d}.json").exists()


# ------------------------------------------------------------------ W9
def test_stage_1_6_reads_refusals_and_grpdisable_from_the_log(tmp_path):
    b = arm_session(tmp_path)
    assert not do(b, "jog", {"axis": "y", "dir": 1, "step_mm": 500})["ok"]        # refused: above jog_max_step_mm
    do(b, "end", {})
    run = wait_run(b)
    crit = {c["id"]: c["ok"] for c in run["criteria"]}
    assert run["exit_code"] == 0 and crit == {"ended": True, "no_fail": True, "no_refusal": False, "disabled": True}
    assert not run["auto_ok"]                                                      # before: passed on the exit code
