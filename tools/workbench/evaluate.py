"""Metrics from a finished run, and the stage's criteria checked against them (codebase plan §12).
Standard library only. The trial log is the record (codebase plan R-A): arm-tool metrics come from its
events, never from what the tool printed."""
import json
import math
import pathlib

from analysis.logread import read_trial
from tools import yamlmini
from tools.elfin_py import protocol, triallog

LIMIT_KEYS = {"ReadJointMaxVel": "joint_max_vel_deg_s", "ReadJointMaxAcc": "joint_max_acc_deg_s2",
              "ReadJointMaxJerk": "joint_max_jerk_deg_s3", "ReadLinearMaxVel": "linear_max"}


def _events(trial, kind):
    out = []
    for r in trial.stream("event"):
        if r["kind"] == kind:
            text = r["text"].decode("utf-8", "replace")
            try:
                out.append(json.loads(text))
            except ValueError:
                out.append(text)
    return out


def _texts(trial, *kinds):
    return [r["text"].decode("utf-8", "replace") for r in trial.stream("event") if r["kind"] in kinds]


def tool_metrics(tool, trial_path, exit_code, arm_config):
    """Named metrics for a tool that writes a trial log."""
    m = {"exit_code": exit_code, "exit_ok": exit_code == 0}
    if not trial_path or not pathlib.Path(trial_path).exists():
        m["trial"] = False
        return m
    trial = read_trial(trial_path)
    m["trial"] = True
    m["truncated"] = trial.truncated
    m["refusals"] = _texts(trial, triallog.EV_REFUSAL)
    m["errors"] = [t[:300] for t in _texts(trial, triallog.EV_ERROR)]
    m["stopped"] = [t for t in _texts(trial, triallog.EV_MARKER) if t.startswith("stopped")]
    # from the log, not from the exit code (pre-hardware review W9): a tool can end with 0 after refusals
    replies = b"".join(r["data"] for r in trial.stream("wire_rx")).split(b";")
    m["fail_replies"] = sum(1 for x in replies if b",Fail," in x)        # Name,Fail,code,; (PROTO p.3)
    m["refusal_count"] = len(m["refusals"])
    phases = _texts(trial, triallog.EV_PHASE)
    m["grpdisable_ok"] = ("GrpDisable sent on exit" in phases and
                          not any(t.startswith("GrpDisable FAILED") for t in m["errors"]))
    summaries = [e for e in _events(trial, triallog.EV_SUMMARY) if isinstance(e, dict)]
    summary = summaries[-1] if summaries else None
    name = tool.rsplit(".", 1)[-1]
    if name == "bringup_read":
        blocks = [e for e in _events(trial, triallog.EV_ARM_BLOCK) if isinstance(e, dict)]
        block = blocks[-1] if blocks else {"reads": {}, "problems": ["no bring-up block"]}
        errs = [e for e in _events(trial, triallog.EV_ERROR) if isinstance(e, dict) and "problems" in e]
        problems = errs[-1]["problems"] if errs else block.get("problems", [])
        reads = block.get("reads") or {}
        m["problems"] = len(problems) if exit_code == 0 or problems else 1
        m["problem_list"] = problems
        m["limits_read"] = all(reads.get(k) for k in LIMIT_KEYS)
        m["model_string"] = (reads.get("ReadRobotModel") or {}).get("strModel")
        lr = (yamlmini.load(arm_config).get("limits_readback") or {})
        same = m["limits_read"] and lr.get("record_id") is not None
        for cmd, key in LIMIT_KEYS.items():
            if not same:
                break
            d = reads.get(cmd) or {}            # the log sorts keys: order them as the command table does
            got, want = [d.get(f.name) for f in protocol.COMMANDS[cmd].reply], lr.get(key)
            tol = lr.get("tolerance") or 0.0
            same = (want is not None and len(got) == len(want) and None not in got
                    and all(abs(a - b) <= tol for a, b in zip(got, want)))
        m["limits_in_config"] = bool(same)
    elif name == "state_stream":
        s = summary or {}
        m.update({k: s.get(k) for k in ("packets", "gap_mean_ms", "gap_max_ms", "safety_keys", "byte_orders")})
        m["stream_ok"] = bool(s.get("packets")) and not s.get("error") and exit_code == 0
    elif name == "move_test":
        m["completed"] = exit_code == 0 and summary is not None
        m["format_accepted"] = bool(summary and summary.get("format_test"))
    elif name == "servo_ladder":
        s = summary or {}
        m["verdict"] = s.get("verdict")
        m["verdict_ok"] = exit_code == 0 and str(s.get("verdict", "")).startswith("OK")
        m["late_fraction"] = s.get("late_fraction")
        m["median_reply_ms"] = (s.get("reply_ms") or {}).get("median")
        m["period_s"] = s.get("period_s")
        m["pushes"] = [s.get("pushes_sent"), s.get("pushes_planned")]
        m["stop_rules"] = s.get("stop_rules")
    elif name == "pose_recorder":
        poses = (summary or {}).get("poses") or []
        m["poses_recorded"] = len(poses)
        m["all_poses"] = exit_code == 0 and len(poses) > 0
    return m


def form_metrics(sid, values, coeff_value):
    """Metrics computed from a form's fields."""
    m = dict(values)
    if sid == "preflight" and values.get("fill_height_mm") is not None:
        try:                                   # the owner measures the fill height; this only records the fraction
            m["fill_fraction"] = round(values["fill_height_mm"] / 1000.0 / coeff_value("cup_inner_height_m"), 4)
        except (TypeError, ValueError, ZeroDivisionError):
            pass
    return m


def check(criterion, metrics, coeff_value):
    """(ok, detail) for one auto criterion."""
    metric = criterion["metric"]
    if metric not in metrics or metrics[metric] is None:
        return False, f"{metric} not measured"
    x = metrics[metric]
    op = criterion["op"]
    if op in ("is_true", "is_false"):
        ok = bool(x) if op == "is_true" else not bool(x)
        return ok, f"{metric} = {x}"
    if "coeff" in criterion:
        ref = coeff_value(criterion["coeff"])
        label = f"{criterion['coeff']} = {ref}"
        if ref is None:
            return False, f"{criterion['coeff']} is not set [MISSING]"
    else:
        ref, label = criterion["value"], str(criterion["value"])
    try:
        if op == "in_range":
            ok = ref[0] <= x <= ref[1]
        elif op == "abs_le":
            ok = abs(x) <= ref
        else:
            ok = {"eq": x == ref, "le": x <= ref, "ge": x >= ref, "lt": x < ref, "gt": x > ref}[op]
    except (TypeError, IndexError) as exc:
        return False, f"cannot compare {metric} = {x!r} with {label}: {exc}"
    if isinstance(ok, float) and math.isnan(ok):
        ok = False
    return bool(ok), f"{metric} = {x} ({op} {label})"


def evaluate(stage, metrics, coeff_value, operator_checks=None):
    """Every criterion of the stage: [{id, text, kind, ok, detail}], and whether the auto ones all pass."""
    operator_checks = operator_checks or {}
    out, auto_ok = [], True
    for cid, c in stage["criteria"].items():
        if c["kind"] == "auto":
            ok, detail = check(c, metrics, coeff_value)
            auto_ok = auto_ok and ok
        else:
            ok, detail = bool(operator_checks.get(cid)), "ticked by the owner" if operator_checks.get(cid) else "to tick"
        out.append({"id": cid, "text": c["text"], "kind": c["kind"], "ok": ok, "detail": detail})
    for fid, f in stage["fields"].items():          # a form's tick boxes are criteria too
        if f["type"] == "check":
            ok = bool(metrics.get(fid))
            out.append({"id": fid, "text": f["label"], "kind": "form", "ok": ok, "detail": "ticked" if ok else "not ticked"})
            auto_ok = auto_ok and ok
        elif f["type"] in ("number", "text") and metrics.get(fid) in (None, ""):
            out.append({"id": fid, "text": f["label"], "kind": "form", "ok": False, "detail": "missing"})
            auto_ok = False
    return out, auto_ok
