"""Log round trip and generator freshness (codebase plan §9, week-1 gates)."""
import struct
import subprocess
import sys
import pathlib

import pytest

from analysis.logread import read_trial, slog_v1 as L
from schema import gen_py

ROOT = pathlib.Path(__file__).resolve().parents[2]


def sample(stream):
    """One record with distinct, non-trivial values for every field."""
    s = L.STREAMS[stream]
    out = {}
    fmt_codes = s.fixed.format.lstrip("<")
    codes = []
    num = ""
    for ch in fmt_codes:
        if ch.isdigit():
            num += ch
        else:
            codes.append((ch, int(num or 1)))
            num = ""
    for (name, n), (code, cnt) in zip(s.fields, codes):
        assert n == cnt
        if code == "d":
            vals = [1.25 * (i + 1) * (-1) ** i + 1e-9 for i in range(n)]
        elif code in "qi":
            vals = [-(7 + i) for i in range(n)]
        else:
            vals = [3 + i for i in range(n)]
        out[name] = vals[0] if n == 1 else vals
    if s.var:
        out[s.var] = bytes(range(256)) + b"PushServoJ,0,1.0000,;"
    return out


def write_all(path, kind):
    with L.Writer(path, {"trial_id": "t", "z": 1, "a": 2}, file=kind) as w:
        for name, s in L.STREAMS.items():
            if s.file == kind:
                w.write(name, **sample(name))
    return path


@pytest.mark.parametrize("kind", ["bin", "img"])
def test_every_record_type_round_trips(tmp_path, kind):
    t = read_trial(write_all(tmp_path / f"t.{kind}", kind))
    assert t.header["schema_version"] == 1 and t.header["file"] == kind
    assert not t.truncated and t.unknown_tags == 0
    names = [n for n, s in L.STREAMS.items() if s.file == kind]
    assert [n for n, _ in t.records] == names
    for (name, rec) in t.records:
        assert rec == sample(name)


def test_truncated_last_record_is_end_of_trial(tmp_path):
    p = write_all(tmp_path / "t.bin", "bin")
    blob = p.read_bytes()
    full = L.read_bytes(blob)
    last_len = len(L.pack(full.records[-1][0], full.records[-1][1])) + 6
    for cut in range(1, last_len):
        t = L.read_bytes(blob[:-cut])
        assert t.truncated
        assert t.records == full.records[:-1]


def test_unknown_tags_are_skipped(tmp_path):
    p = write_all(tmp_path / "t.bin", "bin")
    blob = p.read_bytes() + struct.pack("<HI", 60000, 3) + b"xyz"
    t = L.read_bytes(blob)
    assert t.unknown_tags == 1 and not t.truncated


def test_header_keys_sorted_and_existing_files_never_overwritten(tmp_path):
    p = write_all(tmp_path / "t.bin", "bin")
    blob = p.read_bytes()
    (n,) = struct.unpack_from("<I", blob, 8)
    assert blob[12:12 + n].startswith(b'{"a": 2')
    with pytest.raises(FileExistsError):
        L.Writer(p, {})


def test_bad_records_are_refused(tmp_path):
    with L.Writer(tmp_path / "t.bin", {}) as w:
        with pytest.raises(ValueError):
            w.write("arm_cmd", push_id=1, tick=1, q_cmd_rad=[0.0] * 5, t_slot_ns=0, t_send_ns=0)
        with pytest.raises(ValueError):
            w.write("event", t_host_ns=0, kind=1)
        with pytest.raises(ValueError):
            w.write("image", **sample("image"))     # belongs in the .img file


def test_generated_files_are_fresh():
    r = subprocess.run([sys.executable, "-m", "schema.gen_py", "--check"], cwd=ROOT, capture_output=True, text=True)
    assert r.returncode == 0, r.stderr


def test_schema_validation_catches_mistakes():
    s = gen_py.load_schema()
    s["streams"][0]["fields"].insert(0, {"name": "x", "type": "bytes", "unit": "-"})
    with pytest.raises(ValueError):
        gen_py.validate(s)
