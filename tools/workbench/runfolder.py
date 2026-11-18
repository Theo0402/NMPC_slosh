"""One folder per run, for debugging (codebase plan §8, §12). Standard library only.

    data/runs/<YYYY-MM-DD>_<HHMMSS>_<stage>_<mode>/      local date and time of the start
        summary.txt        read this first: what ran, when, how it ended, which criteria passed
        run.json           the run record: command line, exit code, metrics, criteria, times, coefficient version
        console.log        the tool's whole console output as it came, with the workbench's own notes marked [wb]
        questions.jsonl    every question the tool asked and the answer given, with times
        commands.jsonl     every Arm control command sent (jog, goto, path, end) and its result
        form.json          a form's values (form stages)
        mock.log           the mock controller's output (mock runs)
        config/            the config files the run used (practice copies included) and the stage's registry entry
        coeffs_vNNNNNN.json  the coefficient version the run used (copy of the immutable snapshot)
        trial.bin          the trial log: a hard link to data/trials/<trial_id>.bin, so it costs no space
        trial.txt          where the trial log is, its size and SHA-256 (always written; the link if trial.bin can't be)
        COMPLETE           written last

Rules: the trial log itself stays where the tool wrote it and is never renamed, moved or edited (A4); the folder
only links to it. Images (.img) are referred to, never linked or copied. Once COMPLETE exists, every file in the
folder is read-only and this code refuses to write there again. Times in the jsonl files are local ISO time and
time.monotonic_ns(), the host clock the trial logs use (codebase plan §5), so they line up with the trial's records.
"""
import datetime
import hashlib
import json
import os
import pathlib
import re
import shutil
import stat
import threading
import time

NAME_RE = re.compile(r"^\d{4}-\d{2}-\d{2}_\d{6}_[A-Za-z0-9_]+_[A-Za-z0-9\-]+(_\d+)?$")
CONSOLE_VIEW_BYTES = 1_000_000


class RunFolderError(RuntimeError):
    pass


def local_now():
    return datetime.datetime.now().astimezone()


def stamp(t=None):
    return (t or local_now()).isoformat(timespec="seconds")


def write_atomic(path, text):
    """Write to <name>.tmp, flush to disk, then rename over the target: a crash or a full disk leaves the old file
    or the new one, never half of one (pre-hardware review D7)."""
    path = pathlib.Path(path)
    tmp = path.with_name(path.name + ".tmp")
    with open(tmp, "w", encoding="utf-8", newline="\n") as f:
        f.write(text)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)


def sha256_file(path, chunk=8 << 20):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        while True:
            b = f.read(chunk)
            if not b:
                break
            h.update(b)
    return h.hexdigest()


def make_read_only(folder):
    """Files read-only (Windows: the read-only attribute); on POSIX the folder itself too."""
    folder = pathlib.Path(folder)
    for p in folder.rglob("*"):
        if p.is_file():
            os.chmod(p, stat.S_IREAD | stat.S_IRGRP | stat.S_IROTH)
    if os.name != "nt":
        for d in sorted([p for p in folder.rglob("*") if p.is_dir()], key=lambda p: -len(p.parts)) + [folder]:
            os.chmod(d, 0o555)


def make_writable(folder):
    """Only for an owner-confirmed restore over a damaged folder."""
    folder = pathlib.Path(folder)
    if os.name != "nt":
        os.chmod(folder, 0o755)
        for d in folder.rglob("*"):
            if d.is_dir():
                os.chmod(d, 0o755)
    for p in folder.rglob("*"):
        if p.is_file():
            os.chmod(p, stat.S_IREAD | stat.S_IWRITE)


def folder_name(stage, mode, practice, when, root):
    mode_part = f"{mode}-practice" if practice else mode
    base = f"{when:%Y-%m-%d_%H%M%S}_{stage}_{mode_part}"
    name, n = base, 1
    while (pathlib.Path(root) / name).exists() or (pathlib.Path(root) / (name + ".partial")).exists():
        n += 1
        name = f"{base}_{n}"
    return name


class RunFolder:
    """Written while the run goes on, closed by finish(). Thread-safe: the reader thread of the tool and the
    web server's threads write to it."""

    def __init__(self, root, stage, mode, practice=False, when=None):
        self.root = pathlib.Path(root)
        self.root.mkdir(parents=True, exist_ok=True)
        when = when or local_now()
        self.name = folder_name(stage, mode, practice, when, self.root)
        self.path = self.root / self.name
        self.path.mkdir()
        self.started = when
        self._lock = threading.Lock()
        self._closed = False
        self._console = open(self.path / "console.log", "a", encoding="utf-8", newline="\n")

    @classmethod
    def reopen(cls, path):
        """An open (not COMPLETE) folder left by a workbench that stopped mid-run, to close it."""
        path = pathlib.Path(path)
        if is_complete(path):
            raise RunFolderError(f"{path.name} is complete; it is never written again")
        self = cls.__new__(cls)
        self.root, self.name, self.path = path.parent, path.name, path
        try:
            self.started = datetime.datetime.strptime(path.name[:17], "%Y-%m-%d_%H%M%S").astimezone()
        except ValueError:
            self.started = None
        self._lock = threading.Lock()
        self._closed = False
        self._console = open(path / "console.log", "a", encoding="utf-8", newline="\n")
        return self

    # ------------------------------------------------------------------ during the run
    def _check(self):
        if self._closed or (self.path / "COMPLETE").exists():
            raise RunFolderError(f"{self.name} is complete; it is never written again")

    def _append_json(self, fname, obj):
        with self._lock:
            self._check()
            with open(self.path / fname, "a", encoding="utf-8", newline="\n") as f:
                f.write(json.dumps(obj, sort_keys=True, default=str) + "\n")

    def console(self, line):
        with self._lock:
            if self._closed:
                return                          # late output after the end is dropped, never appended
            self._console.write(line.rstrip("\n") + "\n")
            self._console.flush()

    def note(self, text):
        self.console(f"[wb {stamp()}] {text}")

    def question(self, qid, text):
        self._append_json("questions.jsonl", {"event": "asked", "id": qid, "question": text, "t": stamp(),
                                              "t_mono_ns": time.monotonic_ns()})

    def answer(self, qid, text, by="owner (workbench page)"):
        self._append_json("questions.jsonl", {"event": "answered", "id": qid, "answer": text, "by": by, "t": stamp(),
                                              "t_mono_ns": time.monotonic_ns()})

    def command(self, cid, name, args):
        self._append_json("commands.jsonl", {"event": "sent", "id": cid, "name": name, "args": args, "t": stamp(),
                                             "t_mono_ns": time.monotonic_ns()})

    def command_event(self, msg):
        self._append_json("commands.jsonl", dict(msg, event=msg.get("type"), t=stamp(), t_mono_ns=time.monotonic_ns()))

    def write_json(self, fname, obj):
        with self._lock:
            self._check()
            write_atomic(self.path / fname, json.dumps(obj, indent=1, sort_keys=True, default=str))

    def copy_inputs(self, files, coeff_snapshot, stage_entry):
        """The config files the run starts with, the coefficient version, the stage's registry entry."""
        with self._lock:
            self._check()
            cfg = self.path / "config"
            cfg.mkdir(exist_ok=True)
            for label, src in files.items():
                src = pathlib.Path(src)
                if src.exists():
                    shutil.copyfile(src, cfg / label)
            (cfg / "stage.json").write_text(json.dumps(stage_entry, indent=1, sort_keys=True, default=str),
                                            encoding="utf-8")
            if coeff_snapshot and pathlib.Path(coeff_snapshot).exists():
                shutil.copyfile(coeff_snapshot, self.path / pathlib.Path(coeff_snapshot).name.replace("v", "coeffs_v", 1))

    # ------------------------------------------------------------------ the end
    def finish(self, record, trial_path=None, extra_logs=None, summary_lines=None):
        """Write the record, the trial link, the summary, then COMPLETE; make it all read-only."""
        with self._lock:
            self._check()
            self._console.close()
            for fname, lines in (extra_logs or {}).items():
                (self.path / fname).write_text("\n".join(lines) + ("\n" if lines else ""), encoding="utf-8")
            link = link_trial(self.path, trial_path)
            rec = dict(record, run_folder=self.name, trial_link=link)
            write_atomic(self.path / "run.json", json.dumps(rec, indent=1, sort_keys=True, default=str))
            write_atomic(self.path / "summary.txt", summary_text(rec, self.started, summary_lines))
            write_atomic(self.path / "COMPLETE", stamp() + "\n")          # last: the folder is whole
            self._closed = True
        make_read_only(self.path)
        return rec


def link_trial(folder, trial_path):
    """trial.txt always; trial.bin as a hard link when the file system allows (no extra space)."""
    folder = pathlib.Path(folder)
    if not trial_path:
        (folder / "trial.txt").write_text("no trial log: this run did not write one\n", encoding="utf-8")
        return {"kind": "none"}
    tp = pathlib.Path(trial_path)
    if not tp.exists():
        (folder / "trial.txt").write_text(f"trial log expected at {tp} but not found\n", encoding="utf-8")
        return {"kind": "missing", "path": str(tp)}
    info = {"path": str(tp), "trial_id": tp.stem, "bytes": tp.stat().st_size, "sha256": sha256_file(tp)}
    try:
        os.link(tp, folder / "trial.bin")
        info["kind"] = "hard link"
    except OSError as exc:
        info["kind"] = "path only"
        info["link_error"] = repr(exc)
    img = tp.with_suffix(".img")
    if img.exists():
        info["images"] = {"path": str(img), "bytes": img.stat().st_size, "note": "images are referred to, not linked or copied"}
    lines = [f"trial log: {info['path']}", f"trial id: {info['trial_id']}", f"bytes: {info['bytes']}",
             f"sha256: {info['sha256']}",
             "trial.bin in this folder is a hard link to it (the same file, no extra space)"
             if info["kind"] == "hard link" else f"no hard link here ({info.get('link_error')}); open the path above"]
    if "images" in info:
        lines.append(f"images: {info['images']['path']} ({info['images']['bytes']} bytes)")
    (folder / "trial.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")
    return info


def verdict(rec):
    if rec.get("interrupted"):
        return "INTERRUPTED (the workbench stopped during the run)"
    if rec.get("reconstructed"):
        if rec.get("outside_workbench"):
            return "RECONSTRUCTED (started outside the workbench, from a terminal)"
        return "RECONSTRUCTED (a run from before run folders existed)"
    if rec.get("stopped"):
        return "STOPPED by the operator"
    return "PASSED its criteria" if rec.get("auto_ok") else "DID NOT PASS its criteria"


def summary_text(rec, started=None, extra=None):
    crit = rec.get("criteria") or []
    lines = [
        f"RUN        {rec.get('run_folder', '')}",
        f"stage      {rec.get('stage_num', '')} {rec.get('stage')} ({rec.get('stage_title', '')})",
        f"mode       {rec.get('mode')}{'  PRACTICE: the mock limits, never counts for the arm' if rec.get('practice') else ''}",
        f"started    {rec.get('started_local') or (stamp(started) if started else '')}",
        f"ended      {rec.get('ended_local', '')}",
        f"result     {verdict(rec)}",
        f"exit code  {rec.get('exit_code')}",
        f"trial      {rec.get('trial_id') or '-'}",
        f"times      the folder name, started and ended are local time ({local_now().isoformat()[-6:]} from UTC); "
        "run and trial ids and the journal use UTC",                       # review I3: two time bases, both named
        f"coeffs     version {rec.get('coeff_version')}",
        f"note       {rec.get('note') or '-'}",
    ]
    if rec.get("override"):
        lines.append(f"override   {rec['override']}")
    if crit:
        lines += ["", "criteria"]
        label = lambda c: "PASS " if c.get("ok") else ("OWNER" if c.get("kind") == "operator" else "FAIL ")
        lines += [f"  {label(c)}  {c.get('text')}  ({'you tick this when you accept' if label(c) == 'OWNER' else c.get('detail')})"
                  for c in crit]
    m = rec.get("metrics") or {}
    if m.get("refusals"):
        lines += ["", "refusals"] + [f"  {r}" for r in m["refusals"]]
    if m.get("errors"):
        lines += ["", "errors"] + [f"  {e}" for e in m["errors"]]
    if extra:
        lines += [""] + list(extra)
    lines += ["", "files: console.log (everything printed), questions.jsonl, commands.jsonl, run.json, config/,",
              "coeffs_v*.json, trial.bin or trial.txt. Open trial.bin with analysis.logread.read_trial."]
    return "\n".join(lines) + "\n"


# ---------------------------------------------------------------------- reading folders back
def is_complete(path):
    return (pathlib.Path(path) / "COMPLETE").exists()


def check_name(name):
    if not isinstance(name, str) or not NAME_RE.match(name):
        raise RunFolderError(f"not a run folder name: {name!r}")
    return name


def folder_bytes(path):
    """Bytes of the folder's own files; a hard-linked trial.bin is counted separately (it is the trial's)."""
    own = trial = 0
    for p in pathlib.Path(path).rglob("*"):
        if p.is_file():
            if p.name == "trial.bin":
                trial += p.stat().st_size
            else:
                own += p.stat().st_size
    return own, trial


def list_runs(root):
    root = pathlib.Path(root)
    out = []
    if not root.exists():
        return out
    for d in sorted(root.iterdir(), reverse=True):
        if not d.is_dir() or not NAME_RE.match(d.name):
            continue
        try:
            rec = json.loads((d / "run.json").read_text(encoding="utf-8")) if (d / "run.json").exists() else {}
        except ValueError:
            rec = {}
        own, trial = folder_bytes(d)
        if not trial and isinstance(rec.get("trial_link"), dict):
            trial = rec["trial_link"].get("bytes") or 0
        out.append({"name": d.name, "complete": is_complete(d), "stage": rec.get("stage"),
                    "stage_num": rec.get("stage_num"), "stage_title": rec.get("stage_title"), "mode": rec.get("mode"),
                    "practice": bool(rec.get("practice")), "started_local": rec.get("started_local"),
                    "ended_local": rec.get("ended_local"), "exit_code": rec.get("exit_code"),
                    # a folder without COMPLETE is still going (or awaits the next start-up), whatever its record
                    "result": verdict(rec) if rec and is_complete(d) else "RUNNING or not finished",
                    "auto_ok": rec.get("auto_ok"),
                    "trial_id": rec.get("trial_id"), "folder_bytes": own, "trial_bytes": trial})
    return out


def read_run(root, name, trials_dir=None):
    d = pathlib.Path(root) / check_name(name)
    if not d.is_dir():
        raise RunFolderError(f"no run folder {name}")

    def text(f, limit=None):
        p = d / f
        if not p.exists():
            return None
        data = p.read_bytes()
        cut = limit is not None and len(data) > limit
        t = data[-limit:].decode("utf-8", "replace") if cut else data.decode("utf-8", "replace")
        return (f"[showing the last {limit} bytes of {len(data)}; the whole file is in the folder]\n" + t) if cut else t

    def jsonl(f):
        t = text(f)
        return [json.loads(x) for x in t.splitlines() if x.strip()] if t else []

    rec = json.loads(text("run.json")) if (d / "run.json").exists() else {}
    trial = d / "trial.bin"
    if not trial.exists():
        tid = rec.get("trial_id")
        cand = [pathlib.Path(trials_dir) / f"{tid}.bin"] if (tid and trials_dir) else []
        cand += [pathlib.Path(rec["trial_link"]["path"])] if isinstance(rec.get("trial_link"), dict) and rec["trial_link"].get("path") else []
        trial = next((c for c in cand if c.exists()), None)
    return {"name": name, "path": str(d), "complete": is_complete(d), "summary": text("summary.txt"),
            "record": rec, "console": text("console.log", CONSOLE_VIEW_BYTES), "questions": jsonl("questions.jsonl"),
            "commands": jsonl("commands.jsonl"), "form": json.loads(text("form.json")) if (d / "form.json").exists() else None,
            "mock_log": text("mock.log", 200_000), "trial_path": str(trial) if trial else None,
            "files": sorted(str(p.relative_to(d)).replace("\\", "/") for p in d.rglob("*") if p.is_file())}
