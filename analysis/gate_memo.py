"""The kill-test-2 gate memo (research plan §7.1, "The kill-test-2 gate"). Standard library only.

    python -m analysis.gate_memo [--input data/workbench/gate_input.json] [--memo docs/memos/camera-gate.md]
                                 [--result-json PATH] [--data-dir DIR]

Input: one entry per lighting set-up, with the numbers measured in kill test 2(a)-(c) (stages 3.1, 3.3
and 4.3). Missing numbers are reported as missing and fail their criterion. The thresholds come from the
workbench's coefficients (the current version), else the plan's values.

    {"front_end": "classical",
     "setups": [{"id": "L1", "liquid": "clear", "description": "...",
                 "sigma_s_mm": 0.03, "min_tracked_crest_mm": 0.8, "crest_error_mm": null,
                 "reproj_rms_px": 0.21, "jitter_ms": 0.08, "tau_cam_ms": 6.5,
                 "ringdown_above_deadband_s": 1.4,
                 "deadband_cause": null, "ringdown_plots": ["data/.../L1_ringdown.png"],
                 "info": {"dropped_frames_60s": 0, "bias_vs_amplitude": "...", ...}}]}

The gate passes if at least one clear-water set-up meets all six criteria. Dyed set-ups are reported
as the contrast reference and never pass the gate on their own.

front_end is the one whose numbers these are: "classical" first; "cnn" for gate memo v2, after stage 4.5
accepted the CNN front-end. The verdict names it, and the workbench passes stage 4.4 with "cnn" only once
stage 4.5 has passed (pre-hardware review W1).

Where to go after a fail follows research plan §7.1 and R1 (§8). Items 3-5 do not depend on the edge step:
fix the calibration or the timing. Items 1, 2 and 6 go to the CNN only when they come from the edge step
(contrast, reflections, blur), not from contact-line pinning, which is physics and survives every front-end;
a dead band too large because of pinning goes straight to R1's measurement-method pivot. The memo cannot
tell the two apart: the owner writes "deadband_cause": "pinning" or "edge_step" for each clear set-up whose
item 6 failed, after looking at its ring-down plots (stage 4.3); until then the memo asks for it.
"""
import argparse
import datetime
import json
import os
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]

# (key, metric, threshold coefficient, plan default, comparison, text)
CRITERIA = [
    ("sigma_s", "sigma_s_mm", "gate_sigma_s_max_mm", 0.05, "le", "per-frame noise sigma_s [mm]"),
    ("tracking", "min_tracked_crest_mm", "gate_track_min_crest_mm", 1.0, "le",
     "smallest crest the first harmonic still tracks [mm]"),
    ("reprojection", "reproj_rms_px", "reproj_rms_max_px", 0.3, "le", "reprojection RMS on held-out images [px]"),
    ("jitter", "jitter_ms", "jitter_max_ms", 0.2, "le", "frame-interval jitter [ms]"),
    ("tau_cam", "tau_cam_ms", "tau_cam_max_ms", 10.0, "le", "tau_cam [ms]"),
    ("deadband", "ringdown_above_deadband_s", "gate_deadband_window_s_min", 1.0, "ge",
     "ring-down above the dead band after the standard kick [s]"),
]
DETECTOR = {"sigma_s", "tracking", "deadband"}      # the items a different edge step could change
CAUSES = ("pinning", "edge_step")                     # what made item 6 fail: the owner says (research §7.1, R1)
FRONT_ENDS = ("classical", "cnn")                     # codebase plan §5, edge_method


def route(setup, rows):
    """For one clear set-up that failed: 'fix' (an item that does not depend on the edge step failed), 'classify'
    (item 6 failed and its cause is not given yet), 'pinning' (item 6 failed from pinning: R1's pivot), or 'cnn'
    (every failure is in the edge step)."""
    failing = {r["key"] for r in rows if not r["ok"]}
    if failing - DETECTOR:
        return "fix"
    if "deadband" in failing:
        cause = setup.get("deadband_cause")
        if cause not in CAUSES:
            return "classify"
        if cause == "pinning":
            return "pinning"
    return "cnn"


def thresholds(coeffs=None):
    coeffs = coeffs or {}
    return {key: (coeffs.get(name) if coeffs.get(name) is not None else default)
            for key, _, name, default, _, _ in CRITERIA}


def judge(setup, limits):
    rows, ok_all = [], True
    for key, metric, name, _, op, text in CRITERIA:
        x, lim = setup.get(metric), limits[key]
        if x is None:
            ok, shown = False, "missing"
        else:
            ok = x <= lim if op == "le" else x >= lim
            shown = f"{x:g}"
        ok_all = ok_all and ok
        rows.append({"key": key, "text": text, "value": shown, "limit": f"{'<=' if op == 'le' else '>='} {lim:g}",
                     "ok": ok})
    return rows, ok_all


def memo(doc, limits, today=None):
    today = today or datetime.date.today().isoformat()
    front_end = doc.get("front_end", "classical")
    if front_end not in FRONT_ENDS:
        raise ValueError(f"front_end must be one of {FRONT_ENDS}, not {front_end!r}")
    lines = [f"# Camera gate memo (kill test 2(a)-(c))" + (" v2: the CNN front-end" if front_end == "cnn" else ""), "",
             f"Front-end: **{front_end}**. Written {today} by `analysis/gate_memo.py` from "
             f"`{doc.get('_input', 'the input file')}`. Research plan §7.1 defines the gate; thresholds are "
             "[target] or [proposal] values from the coefficient store.", ""]
    results, passing, routes = [], [], {}
    for s in doc.get("setups", []):
        rows, ok = judge(s, limits)
        clear = s.get("liquid") == "clear"
        results.append({"id": s.get("id"), "liquid": s.get("liquid"), "passes": ok and clear, "rows": rows})
        if ok and clear:
            passing.append(s.get("id"))
        elif clear:
            routes[s.get("id")] = route(s, rows)
    hdr = "| Criterion | limit | " + " | ".join(f"{r['id']} ({r['liquid']})" for r in results) + " |"
    lines += [hdr, "|---|---|" + "---|" * len(results)]
    for i, (key, _, _, _, _, text) in enumerate(CRITERIA):
        cells = [f"{r['rows'][i]['value']} {'pass' if r['rows'][i]['ok'] else '**fail**'}" for r in results]
        lines.append(f"| {i + 1}. {text} | {results[0]['rows'][i]['limit'] if results else ''} | " + " | ".join(cells) + " |")
    lines.append("| **Set-up passes (clear water only)** | | " +
                 " | ".join("**PASS**" if r["passes"] else ("reference" if r["liquid"] != "clear" else "**FAIL**")
                            for r in results) + " |")
    lines += [""]
    detector = "the classical detector" if front_end == "classical" else "the CNN front-end"
    by_route = {k: sorted(str(sid) for sid, r in routes.items() if r == k) for k in ("cnn", "classify", "fix", "pinning")}
    if passing:
        verdict = f"PASS: {detector} passes in clear water under {', '.join(map(str, passing))}."
        nxt = "Kill test 2(d) and everything after it run with this front-end and lighting set-up."
        next_step = "kill_test_2d"
    elif not routes:
        verdict, next_step = "FAIL: no clear-water set-up was measured.", "measure"
        nxt = "Measure at least one clear-water set-up (stages 3.1, 3.3 and 4.3)."
    else:
        verdict = f"FAIL: no clear-water set-up meets all six with {detector}."
        plots = [p for s in doc.get("setups", []) if str(s.get("id")) in by_route["classify"]
                 for p in (s.get("ringdown_plots") or [])]
        advice = {
            "cnn": (f"Under {', '.join(by_route['cnn'])} every failing item comes from the edge step: the CNN "
                    "fallback applies (docs/design/cnn_fallback.md; research plan §7.1). Dyed water stays the last "
                    "resort." if front_end == "classical" else
                    f"The CNN front-end fails too (under {', '.join(by_route['cnn'])}, in the edge step): classical "
                    "detection on dyed water is the last resort, and the paper says so (research plan §7.1; R1 "
                    "step 2)."),
            "fix": ("At least one failing item (" + ", ".join(sorted(
                        {r["key"] for res in results if str(res["id"]) in by_route["fix"]
                         for r in res["rows"] if not r["ok"]} - DETECTOR)) +
                    f", under {', '.join(by_route['fix'])}) does not depend on the edge step: fix the calibration "
                    "or the timing and run the gate again before considering the CNN."),
            "classify": (f"Item 6 failed under {', '.join(by_route['classify'])}: say whether the dead band comes from "
                         "contact-line pinning or from the edge step (contrast, reflections, blur), by writing "
                         "\"deadband_cause\": \"pinning\" or \"edge_step\" for each of them in the gate input, after "
                         "looking at the ring-down plots of stage 4.3" + (f" ({', '.join(plots)})" if plots else "") +
                         ". The CNN answers only the edge step; pinning is physics and survives every front-end "
                         "(research plan §7.1; R1). Then run the gate again."),
            "pinning": (f"The dead band under {', '.join(by_route['pinning'])} comes from contact-line pinning, which "
                        "no front-end removes: neither the CNN nor dye answers it, and a dead band too large for the "
                        "per-pickup fit goes straight to R1's measurement-method pivot (refraction of a printed "
                        "pattern on the cup bottom, or floating markers; research plan §8, R1 step 3)."),
        }
        order = [k for k in ("cnn", "fix", "classify", "pinning") if by_route[k]]   # the first one is the next step
        next_step = {"cnn": "cnn_fallback" if front_end == "classical" else "dyed_water",
                     "fix": "fix_calibration_or_timing", "classify": "classify_deadband",
                     "pinning": "measurement_pivot"}[order[0]]
        nxt = advice[order[0]] + "".join(f" Also: {advice[k]}" for k in order[1:])
    lines += [f"**Verdict.** {verdict}", "", f"**Next.** {nxt}", ""]
    causes = {str(s.get("id")): s.get("deadband_cause") for s in doc.get("setups", []) if s.get("deadband_cause")}
    if causes:
        lines += ["**Dead-band cause, as the owner classified it:** " +
                  ", ".join(f"{k} {v}" for k, v in sorted(causes.items())), ""]
    info = [(s.get("id"), s.get("info")) for s in doc.get("setups", []) if s.get("info")]
    if info:
        lines += ["**For information** ((a)-(c)'s other measurements):", ""]
        lines += [f"- {sid}: " + "; ".join(f"{k} {v}" for k, v in d.items()) for sid, d in info]
        lines.append("")
    crest_err = [(s.get("id"), s.get("crest_error_mm")) for s in doc.get("setups", [])]
    lines += ["**Item 2, the other reading** ([UNCLEAR], docs/open_questions.md Q2): crest error against a reference, "
              "where measured: " + ", ".join(f"{sid} {v if v is not None else 'not measured'}" for sid, v in crest_err), ""]
    return "\n".join(lines), {"any_setup_passes": bool(passing), "passing_setups": passing, "setups": results,
                              "front_end": front_end, "next_step": next_step, "routes": routes,
                              "deadband_causes": causes}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--data-dir", default=os.environ.get("SLOSH_DATA", str(ROOT / "data")))
    ap.add_argument("--input")
    ap.add_argument("--memo", default=str(ROOT / "docs" / "memos" / "camera-gate.md"))
    ap.add_argument("--result-json")
    a = ap.parse_args(argv)
    inp = pathlib.Path(a.input or pathlib.Path(a.data_dir) / "workbench" / "gate_input.json")
    if not inp.exists():
        print(f"no gate input at {inp}: the numbers come from stages 3.1, 3.3 and 4.3 (research plan §7.1)",
              file=sys.stderr)
        return 2
    doc = json.loads(inp.read_text(encoding="utf-8"))
    doc["_input"] = str(inp)
    coeffs = {}
    snap = os.environ.get("SLOSH_WB_COEFFS")
    if snap and pathlib.Path(snap).exists():
        coeffs = {k: v.get("value") for k, v in json.loads(pathlib.Path(snap).read_text(encoding="utf-8"))["values"].items()}
    text, result = memo(doc, thresholds(coeffs))
    out = pathlib.Path(a.memo)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(text, encoding="utf-8")
    print(text)
    if a.result_json:
        pathlib.Path(a.result_json).write_text(json.dumps(dict(result, memo=str(out)), indent=1), encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
