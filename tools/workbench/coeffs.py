"""Versioned coefficient store (codebase plan §12). Standard library only.

data/workbench/coeffs/v000001.json, v000002.json, ... are immutable full snapshots:
    {"version": n, "parent": n-1, "created_utc": ..., "note": ..., "changed": [...],
     "values": {name: {"value": ..., "source": ..., "note": ...}}}
CURRENT holds the live version number. Version 1 is the plan defaults from config/params.schema.yaml.
An edit needs a note and makes a new version; a revert makes a new version equal to an old one.
Nothing is ever rewritten. Values whose home is a config file or a code constant are read from there
and are not editable here.
"""
import datetime
import importlib
import json
import math
import os
import pathlib
import threading

from tools import yamlmini


class CoeffError(ValueError):
    pass


def _now():
    return datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="seconds")


def _atomic_write(path, text, exclusive=False):
    path = pathlib.Path(path)
    if exclusive and path.exists():
        raise CoeffError(f"{path} exists: versions are never overwritten")
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(text, encoding="utf-8")
    os.replace(tmp, path)


def _number(x):
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


def check_value(defn, value):
    """Raise CoeffError if value is not allowed by the definition (type and range)."""
    name, typ = defn["name"], defn["type"]
    if value is None:
        if defn.get("default", "x") is None:
            return
        raise CoeffError(f"{name}: a value is required")
    rng = defn.get("range")

    def in_range(v):
        if rng and not (rng[0] <= v <= rng[1]):
            raise CoeffError(f"{name}: {v} is outside the allowed range [{rng[0]}, {rng[1]}] {defn.get('unit', '')}")

    if typ == "f64":
        if not _number(value):
            raise CoeffError(f"{name}: a finite number is required, got {value!r}")
        in_range(value)
    elif typ == "int":
        if isinstance(value, bool) or not isinstance(value, int):
            raise CoeffError(f"{name}: an integer is required, got {value!r}")
        in_range(value)
    elif typ == "f64_list":
        if not isinstance(value, list) or not all(_number(v) for v in value):
            raise CoeffError(f"{name}: a list of finite numbers is required, got {value!r}")
        for v in value:
            in_range(v)
    elif typ == "str":
        if not isinstance(value, str) or not value.strip():
            raise CoeffError(f"{name}: a non-empty string is required")
    elif typ == "enum":
        if value not in defn["values"]:
            raise CoeffError(f"{name}: must be one of {defn['values']}")
    elif typ == "bool":
        if not isinstance(value, bool):
            raise CoeffError(f"{name}: true or false")
    else:
        raise CoeffError(f"{name}: unknown type {typ}")


def read_home(home, root, file_map=None):
    """The current value of a file- or code-homed coefficient. file_map redirects repo paths (tests)."""
    kind, rest = home.split(":", 1)
    where, key = rest.split("#", 1)
    if kind == "file":
        d = yamlmini.load((file_map or {}).get(where) or pathlib.Path(root) / where)
        for part in key.split("."):
            d = (d or {}).get(part) if isinstance(d, dict) else None
        return d
    if kind == "code":
        return getattr(importlib.import_module(where), key)
    raise CoeffError(f"unknown home {home}")


class CoeffStore:
    def __init__(self, registry, data_dir, root=None, file_map=None):
        self.reg = registry
        self.file_map = file_map or {}
        self.root = pathlib.Path(root) if root else registry.stages_path.parents[2]
        self.dir = pathlib.Path(data_dir) / "workbench" / "coeffs"
        self.dir.mkdir(parents=True, exist_ok=True)
        self.lock = threading.Lock()
        if not (self.dir / "CURRENT").exists():
            values = {n: {"value": d.get("default"), "source": f"plan default ({d.get('from', '-')})", "note": ""}
                      for n, d in registry.tunables.items() if d["home"] == "store"}
            self._write(1, None, "plan defaults", sorted(values), values)

    # ------------------------------------------------------------------ files
    def _path(self, n):
        return self.dir / f"v{n:06d}.json"

    def _write(self, n, parent, note, changed, values):
        snap = {"version": n, "parent": parent, "created_utc": _now(), "note": note, "changed": changed,
                "values": values}
        _atomic_write(self._path(n), json.dumps(snap, indent=1, sort_keys=True), exclusive=True)
        _atomic_write(self.dir / "CURRENT", str(n))
        return snap

    def current_version(self):
        return int((self.dir / "CURRENT").read_text(encoding="utf-8").strip())

    def snapshot(self, n=None):
        n = self.current_version() if n is None else int(n)
        p = self._path(n)
        if not p.exists():
            raise CoeffError(f"no coefficient version {n}")
        return json.loads(p.read_text(encoding="utf-8"))

    def snapshot_path(self, n=None):
        return self._path(self.current_version() if n is None else int(n))

    def complete_snapshot(self, n=None):
        """The version's values plus the schema default of every stored coefficient it lacks (one added to the schema
        after the version was saved), each marked so (pre-hardware review W7). Versions stay as they were saved; this
        is what a run gets, so its trial header carries every value the tool uses."""
        snap = self.snapshot(n)
        values, filled = dict(snap["values"]), []
        for name, d in self.reg.tunables.items():
            if d["home"] == "store" and name not in values:
                values[name] = {"value": d.get("default"), "note": "",
                                "source": f"schema default: not in version {snap['version']} (added to the schema later)"}
                filled.append(name)
        return dict(snap, values=values, filled_from_schema=sorted(filled))

    def versions(self):
        out = []
        for p in sorted(self.dir.glob("v*.json")):
            s = json.loads(p.read_text(encoding="utf-8"))
            out.append({k: s[k] for k in ("version", "parent", "created_utc", "note", "changed")})
        return out

    # ------------------------------------------------------------------ values
    def value(self, name, snap=None):
        d = self.reg.tunables[name]
        if d["home"] != "store":
            return read_home(d["home"], self.root, self.file_map)
        snap = snap or self.snapshot()
        return snap["values"].get(name, {}).get("value", d.get("default"))

    def values(self, names=None, snap=None):
        snap = snap or self.snapshot()
        return {n: self.value(n, snap) for n in (names or self.reg.tunables)}

    def table(self):
        """Every coefficient, grouped by the order of the stages, for the editor."""
        snap = self.snapshot()
        rows = []
        for sid in self.reg.order:
            for name in self.reg.owned(sid):
                d = self.reg.tunables[name]
                entry = snap["values"].get(name, {})
                try:
                    v = self.value(name, snap)
                    err = None
                except Exception as exc:          # a file not yet filled in, say
                    v, err = None, repr(exc)
                rows.append({"name": name, "stage": sid, "num": self.reg.stages[sid]["num"], "value": v,
                             "unit": d.get("unit"), "range": d.get("range"), "values": d.get("values"),
                             "type": d["type"], "marker": d.get("marker"), "home": d["home"],
                             "editable": d["home"] == "store",
                             "source": entry.get("source") if d["home"] == "store" else d["home"],
                             "note": entry.get("note", ""), "doc": d.get("doc"), "from": d.get("from"),
                             "error": err})
        return {"version": snap["version"], "rows": rows}

    # ------------------------------------------------------------------ edits
    def edit(self, changes, note, source="hand-set"):
        """changes: {name: value}. Returns the new snapshot and its diff against the old one."""
        if not note or not str(note).strip():
            raise CoeffError("every edit needs a note")
        if not changes:
            raise CoeffError("nothing to change")
        with self.lock:
            old = self.snapshot()
            values = json.loads(json.dumps(old["values"]))
            for name, value in changes.items():
                d = self.reg.tunables.get(name)
                if d is None:
                    raise CoeffError(f"unknown coefficient {name}")
                if d["home"] != "store":
                    raise CoeffError(f"{name} lives in {d['home']}; edit it there")
                if d["type"] == "f64" and isinstance(value, int) and not isinstance(value, bool):
                    value = float(value)
                if d["type"] == "f64_list" and isinstance(value, list):
                    value = [float(v) if isinstance(v, int) and not isinstance(v, bool) else v for v in value]
                check_value(d, value)
                values[name] = {"value": value, "source": source, "note": str(note)}
            changed = sorted(n for n in changes if values[n]["value"] != old["values"].get(n, {}).get("value"))
            if not changed:
                raise CoeffError("the new values equal the current ones")
            snap = self._write(old["version"] + 1, old["version"], str(note), changed, values)
        return snap, self.diff(old["version"], snap["version"])

    def revert(self, to_version, note):
        if not note or not str(note).strip():
            raise CoeffError("a revert needs a note")
        with self.lock:
            old = self.snapshot()
            target = self.snapshot(to_version)
            changed = sorted(n for n in set(old["values"]) | set(target["values"])
                             if old["values"].get(n, {}).get("value") != target["values"].get(n, {}).get("value"))
            if not changed:
                raise CoeffError(f"version {to_version} equals the current one")
            values = json.loads(json.dumps(target["values"]))
            snap = self._write(old["version"] + 1, old["version"], f"revert to v{to_version}: {note}", changed, values)
        return snap, self.diff(old["version"], snap["version"])

    def diff(self, a, b):
        sa, sb = self.snapshot(a), self.snapshot(b)
        out = []
        for n in sorted(set(sa["values"]) | set(sb["values"])):
            va, vb = sa["values"].get(n, {}), sb["values"].get(n, {})
            if va.get("value") != vb.get("value") or va.get("source") != vb.get("source"):
                out.append({"name": n, "old": va.get("value"), "new": vb.get("value"),
                            "old_source": va.get("source"), "new_source": vb.get("source"),
                            "unit": self.reg.tunables.get(n, {}).get("unit")})
        return out
