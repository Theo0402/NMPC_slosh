"""The stage registry (config/workbench/stages.yaml) and the coefficient definitions
(config/params.schema.yaml, tunables:). Standard library only (codebase plan §12)."""
import pathlib

from tools import yamlmini

ROOT = pathlib.Path(__file__).resolve().parents[2]
STAGES = ROOT / "config" / "workbench" / "stages.yaml"
PARAMS = ROOT / "config" / "params.schema.yaml"

MODES = ("dry-run", "mock", "arm", "form", "analysis")
STATUSES = ("built", "planned", "conditional")
OPS = ("is_true", "is_false", "eq", "le", "ge", "lt", "gt", "abs_le", "in_range")
TYPES = ("f64", "int", "str", "enum", "f64_list", "bool")
MARKERS = ("target", "unverified", "MISSING", "proposal", "owner", "plan", "measured")


class RegistryError(ValueError):
    pass


def primary_mode(stage):
    """The mode whose result counts as the stage having passed: the arm, else the form, else analysis."""
    for m in ("arm", "form", "analysis", "mock", "dry-run"):
        if m in stage["modes"]:
            return m
    raise RegistryError(f"{stage['id']}: no modes")


class Registry:
    def __init__(self, stages_path=STAGES, params_path=PARAMS):
        self.stages_path, self.params_path = pathlib.Path(stages_path), pathlib.Path(params_path)
        doc = yamlmini.load(self.stages_path)
        self.groups = doc.get("groups") or {}
        self.order = list(doc["stages"])
        self.stages = {}
        for sid, s in doc["stages"].items():
            s = dict(s)
            s["id"] = sid
            for key in ("requires", "inputs", "adjust", "files", "args"):
                s[key] = list(s.get(key) or [])
            for key in ("criteria", "fields", "options", "sets"):
                s[key] = dict(s.get(key) or {})
            s.setdefault("per_session", False)
            # a gate (host check, the per-session checklists) locks what follows but is not a data input:
            # its new results and its coefficients do not make downstream results stale
            s["gate"] = bool(s.get("gate", s["per_session"]))
            self.stages[sid] = s
        params = yamlmini.load(self.params_path)
        self.tunables = {}
        for name, d in (params.get("tunables") or {}).items():
            d = dict(d)
            d["name"] = name
            d.setdefault("home", "store")
            self.tunables[name] = d
        self.validate()

    # ------------------------------------------------------------------ checks
    def validate(self):
        problems = []
        nums = set()
        for sid in self.order:
            s = self.stages[sid]
            for key in ("num", "group", "title", "purpose", "plan", "modes", "status", "tool", "output"):
                if not s.get(key):
                    problems.append(f"{sid}: '{key}' missing")
            if s.get("num") in nums:
                problems.append(f"{sid}: duplicate num {s.get('num')}")
            nums.add(s.get("num"))
            if s.get("group") not in self.groups:
                problems.append(f"{sid}: unknown group {s.get('group')}")
            for m in s.get("modes") or []:
                if m not in MODES:
                    problems.append(f"{sid}: unknown mode {m}")
            if s.get("status") not in STATUSES:
                problems.append(f"{sid}: status must be one of {STATUSES}")
            if "form" in (s.get("modes") or []) and s.get("tool") != "form":
                problems.append(f"{sid}: a form stage has tool 'form'")
            for r in s["requires"]:
                if r not in self.stages:
                    problems.append(f"{sid}: requires unknown stage {r}")
            for c in s["adjust"]:
                if c not in self.tunables:
                    problems.append(f"{sid}: adjusts unknown coefficient {c}")
            for cid, c in s["criteria"].items():
                if c.get("kind") not in ("auto", "operator"):
                    problems.append(f"{sid}.{cid}: kind must be auto or operator")
                if c.get("kind") == "auto":
                    if c.get("op") not in OPS:
                        problems.append(f"{sid}.{cid}: op must be one of {OPS}")
                    if not c.get("metric"):
                        problems.append(f"{sid}.{cid}: an auto criterion names its metric")
                    if c.get("op") not in ("is_true", "is_false") and "value" not in c and "coeff" not in c:
                        problems.append(f"{sid}.{cid}: needs a value or a coeff")
                    if c.get("coeff") and c["coeff"] not in self.tunables:
                        problems.append(f"{sid}.{cid}: unknown coeff {c['coeff']}")
            for f in s["fields"].values():
                if f.get("type") not in ("check", "number", "text"):
                    problems.append(f"{sid}: field type must be check, number or text")
            for coeff in s["sets"]:
                if coeff not in self.tunables:
                    problems.append(f"{sid}: sets unknown coefficient {coeff}")
        for name, d in self.tunables.items():
            if d.get("stage") not in self.stages:
                problems.append(f"tunable {name}: unknown stage {d.get('stage')}")
            if d.get("type") not in TYPES:
                problems.append(f"tunable {name}: type must be one of {TYPES}")
            if d.get("marker") not in MARKERS:
                problems.append(f"tunable {name}: marker must be one of {MARKERS}")
            home = d["home"]
            if home == "store":
                if "default" not in d:
                    problems.append(f"tunable {name}: a stored value needs a default (null if none)")
                if d.get("type") in ("f64", "int", "f64_list") and not d.get("range"):
                    problems.append(f"tunable {name}: numeric, needs a range")
                if d.get("type") == "enum" and not d.get("values"):
                    problems.append(f"tunable {name}: enum, needs values")
            elif not (home.startswith("file:") or home.startswith("code:")) or "#" not in home:
                problems.append(f"tunable {name}: home must be store, file:<path>#<key> or code:<module>#<NAME>")
        try:
            self.topological()
        except RegistryError as exc:
            problems.append(str(exc))
        if problems:
            raise RegistryError("stage registry:\n  " + "\n  ".join(problems))

    def topological(self):
        seen, done, out = set(), set(), []

        def visit(sid, path):
            if sid in done:
                return
            if sid in seen:
                raise RegistryError("cycle in requires: " + " -> ".join(path + [sid]))
            seen.add(sid)
            for r in self.stages[sid]["requires"]:
                visit(r, path + [sid])
            done.add(sid)
            out.append(sid)

        for sid in self.order:
            visit(sid, [])
        return out

    # ------------------------------------------------------------------ queries
    def upstream(self, sid):
        """Every stage this one depends on, transitively."""
        out, todo = [], list(self.stages[sid]["requires"])
        while todo:
            r = todo.pop()
            if r not in out:
                out.append(r)
                todo.extend(self.stages[r]["requires"])
        return sorted(out, key=self.order.index)

    def downstream(self, sid):
        return [s for s in self.order if sid in self.upstream(s)]

    def stale_after(self, changed):
        """Stages whose results go stale when these coefficients change."""
        return [s for s in self.order if set(changed) & set(self.input_coeffs(s))]

    def owned(self, sid):
        return [n for n, d in self.tunables.items() if d["stage"] == sid]

    def input_coeffs(self, sid):
        """Coefficients whose change makes this stage stale: its own and every upstream stage's."""
        stages = [sid] + [u for u in self.upstream(sid) if not self.stages[u]["gate"]]
        s = self.stages[sid]
        named = set(s["adjust"]) | {c["coeff"] for c in s["criteria"].values() if c.get("coeff")}
        return [n for n, d in self.tunables.items() if d["stage"] in stages or n in named]
