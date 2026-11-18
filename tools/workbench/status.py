"""Runs, accepted results, staleness, locks, the audit file and the journal (codebase plan §12).
Standard library only.

data/workbench/runs/<run_id>.json           every run, passed or not
data/workbench/results/<stage>/<NNNN>.json  accepted results, immutable
data/workbench/audit.jsonl                  one JSON object per accept, override, edit, limit change
docs/journal.md, section "Runs"             one line per run, accept, override and edit (P4)
"""
import datetime
import hashlib
import json
import os
import pathlib
import threading

from tools import yamlmini
from tools.workbench.coeffs import _atomic_write
from tools.workbench.stages import primary_mode

RUNS_HEADING = "## Runs (written by the workbench)"


def utc_now():
    return datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="seconds")


def session_id():
    """A session is a local calendar day (per-session stages go stale the next day)."""
    return datetime.date.today().isoformat()


def file_part_hash(root, spec, file_map=None):
    """sha256 of a config file, or of one key path inside it: 'path' or 'path#key.sub'."""
    path, _, key = spec.partition("#")
    p = pathlib.Path((file_map or {}).get(path) or pathlib.Path(root) / path)
    if not p.exists():
        return "missing"
    if not key:
        return hashlib.sha256(p.read_bytes()).hexdigest()
    d = yamlmini.load(p)
    for part in key.split("."):
        d = d.get(part) if isinstance(d, dict) else None
    return hashlib.sha256(json.dumps(d, sort_keys=True).encode()).hexdigest()


class Status:
    def __init__(self, registry, coeffs, data_dir, journal_path, root, file_map=None):
        self.reg, self.coeffs, self.root = registry, coeffs, pathlib.Path(root)
        self.file_map = file_map or {}
        self.dir = pathlib.Path(data_dir) / "workbench"
        (self.dir / "runs").mkdir(parents=True, exist_ok=True)
        (self.dir / "results").mkdir(parents=True, exist_ok=True)
        self.journal_path = pathlib.Path(journal_path)
        self.lock = threading.Lock()

    # ------------------------------------------------------------------ records
    def save_run(self, run):
        _atomic_write(self.dir / "runs" / f"{run['run_id']}.json", json.dumps(run, indent=1, sort_keys=True, default=str))
        self._runs = None

    _runs = None
    _results = None

    unreadable = None               # records that could not be read: {path: error}, skipped (pre-hardware review D7)

    def _load(self, p):
        try:
            rec = json.loads(p.read_text(encoding="utf-8"))
            if isinstance(rec, dict):
                return rec
            raise ValueError("not a JSON object")
        except (OSError, ValueError) as exc:
            if self.unreadable is None:
                self.unreadable = {}
            self.unreadable[str(p)] = repr(exc)
            return None

    def _all_runs(self):
        if self._runs is None:          # only this process writes these files; reload after each write
            recs = [self._load(p) for p in sorted((self.dir / "runs").glob("*.json"))
                    if not p.name.endswith(".result.json")]
            self._runs = [r for r in recs if r is not None]
        return self._runs

    def runs(self, sid=None, mode=None):
        return [r for r in self._all_runs() if (sid is None or r["stage"] == sid) and (mode is None or r["mode"] == mode)]

    def run(self, run_id):
        p = self.dir / "runs" / f"{run_id}.json"
        return self._load(p) if p.exists() else None

    def results(self, sid, mode=None):
        if self._results is None:
            self._results = {}
        if sid not in self._results:
            d = self.dir / "results" / sid
            recs = [self._load(p) for p in sorted(d.glob("*.json"))] if d.exists() else []
            self._results[sid] = [r for r in recs if r is not None]
        return [r for r in self._results[sid] if mode is None or r["mode"] == mode]

    def latest_result(self, sid, mode):
        res = self.results(sid, mode)
        return res[-1] if res else None

    def accept(self, run, note, operator_checks, criteria):
        sid = run["stage"]
        with self.lock:
            d = self.dir / "results" / sid
            d.mkdir(parents=True, exist_ok=True)
            n = len(list(d.glob("*.json"))) + 1
            res = {"result_id": f"{sid}/{n:04d}", "stage": sid, "mode": run["mode"], "run_id": run["run_id"],
                   "trial_id": run.get("trial_id"), "metrics": run.get("metrics"), "criteria": criteria,
                   "operator_checks": operator_checks, "note": note, "inputs_hash": run["inputs_hash"],
                   "session": run.get("session"), "coeff_version": run.get("coeff_version"),
                   "accepted_utc": utc_now()}
            _atomic_write(d / f"{n:04d}.json", json.dumps(res, indent=1, sort_keys=True, default=str), exclusive=True)
            self._results = None
        self.audit("accept", stage=sid, mode=run["mode"], result_id=res["result_id"], run_id=run["run_id"], note=note)
        return res

    # ------------------------------------------------------------------ audit and journal
    def audit(self, kind, **fields):
        rec = dict(fields, kind=kind, utc=utc_now())
        with self.lock:
            with open(self.dir / "audit.jsonl", "a", encoding="utf-8") as f:
                f.write(json.dumps(rec, sort_keys=True, default=str) + "\n")
        self.journal(kind, fields)
        return rec

    def audit_log(self, limit=200):
        p = self.dir / "audit.jsonl"
        if not p.exists():
            return []
        return [json.loads(x) for x in p.read_text(encoding="utf-8").splitlines()[-limit:]]

    def journal(self, kind, fields):
        bits = [utc_now().replace("+00:00", "Z"), kind]
        if fields.get("stage"):
            s = self.reg.stages.get(fields["stage"], {})
            bits.append(f"{s.get('num', '?')} {fields['stage']}")
        for key in ("mode", "verdict", "trial_id", "run_folder", "result_id", "version", "changed", "reason", "note"):
            v = fields.get(key)
            if v not in (None, "", []):
                bits.append(f"{key} {v if not isinstance(v, list) else ', '.join(map(str, v))}")
        line = "- " + " · ".join(str(b).replace("\n", " ").replace("\r", " ") for b in bits)
        with self.lock:
            # appended, never rewritten (pre-hardware review D5): a crash or a full disk mid-write cannot cut the
            # file short, the owner's own lines are never touched, and the file keeps its own line endings
            p = self.journal_path
            old = p.read_bytes() if p.exists() else b""
            nl = b"\r\n" if b"\r\n" in old else b"\n"
            add = b"" if old else b"# Journal" + nl
            if old and not old.endswith(b"\n"):
                add += nl
            if RUNS_HEADING.encode("utf-8") not in old:
                add += nl + RUNS_HEADING.encode("utf-8") + nl + nl
            add += line.encode("utf-8") + nl
            with open(p, "ab") as f:
                f.write(add)
                f.flush()
                os.fsync(f.fileno())

    # ------------------------------------------------------------------ staleness
    def inputs(self, sid):
        """What a stage's result depends on. Its hash is stored with every run and result."""
        s = self.reg.stages[sid]
        snap = self.coeffs.snapshot()
        coeffs = {}
        for n in self.reg.input_coeffs(sid):
            try:
                coeffs[n] = self.coeffs.value(n, snap)
            except Exception as exc:           # an unreadable file is an input state too
                coeffs[n] = f"unreadable: {exc!r}"
        upstream = {}
        for r in self.reg.upstream(sid):          # the whole chain, as the coefficients are (pre-hardware review W4)
            if self.reg.stages[r]["gate"]:
                continue
            res = self.latest_result(r, primary_mode(self.reg.stages[r]))
            upstream[r] = res["result_id"] if res else None
        files = {spec: file_part_hash(self.root, spec, self.file_map) for spec in s["files"]}
        out = {"coeffs": coeffs, "upstream": upstream, "files": files}
        if s["per_session"]:
            out["session"] = session_id()
        return out

    def inputs_hash(self, sid):
        return hashlib.sha256(json.dumps(self.inputs(sid), sort_keys=True, default=str).encode()).hexdigest()

    def status(self, sid, mode, current_hash=None):
        """not run | passed | failed | stale, with the result or run it rests on."""
        current_hash = current_hash or self.inputs_hash(sid)
        res = self.latest_result(sid, mode)
        runs = [r for r in self.runs(sid, mode) if not r.get("practice")]   # practice never unlocks anything
        last = runs[-1] if runs else None
        if mode in ("dry-run", "mock"):
            # a rehearsal counts from its latest run, never from an accepted result: those may predate the
            # practice rule (pre-hardware review S6), and a rehearsal needs no accept
            if last is None:
                return {"status": "not run"}
            if not last.get("auto_ok"):
                return {"status": "failed", "run_id": last["run_id"]}
            return {"status": "passed" if last["inputs_hash"] == current_hash else "stale", "run_id": last["run_id"]}
        if res is None:
            if last is not None and not last.get("auto_ok"):
                return {"status": "failed", "run_id": last["run_id"]}
            return {"status": "not run"}
        if last is not None and last["run_id"] != res["run_id"] and not last.get("auto_ok") \
                and last.get("ended_utc", "") > res["accepted_utc"]:
            return {"status": "failed", "run_id": last["run_id"], "result_id": res["result_id"]}
        if res["inputs_hash"] != current_hash:
            return {"status": "stale", "result_id": res["result_id"]}
        return {"status": "passed", "result_id": res["result_id"]}

    def passed(self, sid, mode=None):
        mode = mode or primary_mode(self.reg.stages[sid])
        return self.status(sid, mode)["status"] == "passed"

    def locks(self, sid, mode, host_problem):
        """Reasons a run in this mode is locked. Dry runs and mock runs are never locked."""
        s = self.reg.stages[sid]
        if mode in ("dry-run", "mock"):
            return []
        out = []
        if mode == "arm" and host_problem:
            out.append({"reason": f"not the Linux machine: {host_problem}", "overridable": False})
        for r in s["requires"]:
            if not self.passed(r):
                rs = self.reg.stages[r]
                out.append({"reason": f"{rs['num']} {r} has not passed ({primary_mode(rs)})", "overridable": True})
        if mode == "arm":
            for m in ("dry-run", "mock"):
                if m in s["modes"] and not self.passed(sid, m):
                    out.append({"reason": f"this stage has not passed in {m} with the current inputs",
                                "overridable": True})
        return out
