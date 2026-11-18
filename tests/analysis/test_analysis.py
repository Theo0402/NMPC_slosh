"""Stage analysis pieces on synthetic data: the ring-down fit and the
dispersion check (stage 4.4/5.1), the gate memo (stage 4.5)."""
import json
import math
import pathlib

import pytest

from analysis import gate_memo

np = pytest.importorskip("numpy")
from analysis.identify import ringdown as rd  # noqa: E402

ROOT = pathlib.Path(__file__).resolve().parents[2]


def test_dispersion_relation_matches_the_plan_table():
    for h, f0 in ((0.040, 3.30), (0.060, 3.37), (0.080, 3.38)):     # research §2.2
        assert rd.omega0_theory(0.040, h) / (2 * math.pi) == pytest.approx(f0, abs=0.006)


def synthetic(f0=3.37, zeta=0.01, amp=5.0, noise=0.05, fps=200.0, dur=1.5, seed=1):
    rng = np.random.default_rng(seed)
    t = np.arange(0, dur, 1 / fps)
    w0 = 2 * math.pi * f0
    y = amp * np.exp(-zeta * w0 * t) * np.cos(w0 * math.sqrt(1 - zeta ** 2) * t + 0.7) + 0.3 + 0.1 * t
    return t, y + rng.normal(0, noise, t.size)


def test_ringdown_fit_recovers_f0_and_zeta():
    t, y = synthetic()
    r = rd.fit(t, y)
    assert r["f0_hz"] == pytest.approx(3.37, rel=1e-3)     # research §3.2 expects ± 0.3 %
    assert r["zeta"] == pytest.approx(0.01, rel=0.15)
    assert r["omega0_std"] > 0 and r["zeta_std"] > 0
    assert rd.dispersion_mismatch(r["omega0"], 0.040, 0.060) == pytest.approx(0.0, abs=0.003)


def test_ringdown_fit_window_ends_at_the_dead_band():
    t, y = synthetic(dur=3.0)
    r = rd.fit(t, y, deadband=3.0)
    expected = math.log(5.0 / 3.0) / (0.01 * 2 * math.pi * 3.37)
    assert r["window_s"] == pytest.approx(expected, abs=0.05)
    with pytest.raises(ValueError, match="too short"):
        rd.fit(t, y, deadband=4.8)


def test_ringdown_is_deterministic_on_the_same_data():
    t, y = synthetic(seed=3)
    assert rd.fit(t, y) == rd.fit(t, y)                     # codebase §9: same file, same fit


def test_repeatability_and_isotropy():
    s = rd.spread([3.36, 3.37, 3.38, 3.37])
    assert s["rel_max_dev"] == pytest.approx(0.01 / 3.37, rel=1e-6)
    assert rd.isotropy(3.37, 3.40) == pytest.approx(0.03 / 3.385)


def gate_doc(sigma=0.03):
    return {"front_end": "classical", "setups": [
        {"id": "L1", "liquid": "clear", "sigma_s_mm": sigma, "min_tracked_crest_mm": 0.8, "reproj_rms_px": 0.21,
         "jitter_ms": 0.08, "tau_cam_ms": 6.5, "ringdown_above_deadband_s": 1.4, "info": {"dropped_frames_60s": 0}},
        {"id": "L2", "liquid": "clear", "sigma_s_mm": 0.09, "min_tracked_crest_mm": 2.5, "reproj_rms_px": 0.21,
         "jitter_ms": 0.08, "tau_cam_ms": 6.5, "ringdown_above_deadband_s": None},
        {"id": "D1", "liquid": "dyed", "sigma_s_mm": 0.01, "min_tracked_crest_mm": 0.3, "reproj_rms_px": 0.21,
         "jitter_ms": 0.08, "tau_cam_ms": 6.5, "ringdown_above_deadband_s": 2.0}]}


def test_gate_memo_passes_on_one_clear_setup():
    text, res = gate_memo.memo(gate_doc(), gate_memo.thresholds({}), today="W3")
    assert res["any_setup_passes"] and res["passing_setups"] == ["L1"]
    assert "PASS" in text and "missing" in text and "reference" in text


def test_gate_memo_fails_to_the_cnn_only_for_detector_items():
    doc = gate_doc(sigma=0.2)
    text, res = gate_memo.memo(doc, gate_memo.thresholds({}), today="W3")
    assert not res["any_setup_passes"] and "CNN fallback applies" in text
    doc["setups"][0]["jitter_ms"] = 0.5                     # timing does not depend on the edge step
    text, _ = gate_memo.memo(doc, gate_memo.thresholds({}), today="W3")
    assert "does not depend on the edge step" in text and "jitter" in text


def test_gate_memo_cli_writes_the_memo_and_the_result(tmp_path):
    inp = tmp_path / "gate_input.json"
    inp.write_text(json.dumps(gate_doc()), encoding="utf-8")
    out, res = tmp_path / "camera-gate.md", tmp_path / "r.json"
    assert gate_memo.main(["--input", str(inp), "--memo", str(out), "--result-json", str(res)]) == 0
    assert json.loads(res.read_text())["any_setup_passes"] is True
    assert out.read_text(encoding="utf-8").startswith("# Camera gate memo")


def test_gate_memo_routes_a_dead_band_failure_by_its_cause():
    # review W1: the CNN answers item 6 only when the edge step causes it; pinning survives every front-end
    doc = gate_doc()
    doc["setups"] = [dict(doc["setups"][0], ringdown_above_deadband_s=0.5, ringdown_plots=["L1_ringdown.png"])]
    text, res = gate_memo.memo(doc, gate_memo.thresholds({}), today="W3")
    assert res["next_step"] == "classify_deadband" and "L1_ringdown.png" in text and "CNN fallback applies" not in text
    doc["setups"][0]["deadband_cause"] = "pinning"
    text, res = gate_memo.memo(doc, gate_memo.thresholds({}), today="W3")
    assert res["next_step"] == "measurement_pivot" and "R1" in text and "CNN fallback applies" not in text
    doc["setups"][0]["deadband_cause"] = "edge_step"
    text, res = gate_memo.memo(doc, gate_memo.thresholds({}), today="W3")
    assert res["next_step"] == "cnn_fallback" and "CNN fallback applies" in text


def test_gate_memo_names_the_front_end_it_judges():
    doc = dict(gate_doc(), front_end="cnn")
    text, res = gate_memo.memo(doc, gate_memo.thresholds({}), today="W3")
    assert "PASS: the CNN front-end passes" in text and "v2" in text and res["front_end"] == "cnn"
    doc["setups"] = [dict(doc["setups"][0], sigma_s_mm=0.2)]          # the CNN fails in the edge step: dye is next
    text, res = gate_memo.memo(doc, gate_memo.thresholds({}), today="W3")
    assert res["next_step"] == "dyed_water" and "the classical detector passes" not in text
    with pytest.raises(ValueError, match="front_end"):
        gate_memo.memo(dict(doc, front_end="magic"), gate_memo.thresholds({}), today="W3")
