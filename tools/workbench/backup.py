"""Local backup of run folders and trial logs, and restore from it (codebase plan §8, §12). Standard library only.

    <backup folder>/slosh-data/runs/<run folder>/     copies of data/runs/<run folder>/ (trial.bin re-linked)
    <backup folder>/slosh-data/trials/<trial>.bin     copies of data/trials/*.bin, every dataset
    <backup folder>/slosh-data/trials/<trial>.img     images of eval and killtest trials only (§8's image policy)
    <backup folder>/slosh-data/backup_log.jsonl       one line per backup

Rules: copy only what is new; never delete or overwrite anything in the backup (a file that differs from the
backup copy is reported as a conflict and left alone); never copy a run that is still going; pause while a run is
active, so that a copy never competes with a run for the disk; files appear only when complete (written under a
.partial name, then renamed). Restoring never overwrites current data unless the owner confirms; even then the
current file is kept beside it, renamed, not deleted.

This backup sits on the same PC as the data: it protects against deleted or damaged files, not against a failed
disk. The second disk and the offsite copy stay tools/backup.sh (§8).

The backup folder is kept per machine (Windows, WSL and the Linux machine share data/ but not paths), and it is
checked again every time it is used (pre-hardware review D3): a Windows path read under WSL or Linux is a relative
path, and the backups then landed inside the repository.
"""
import datetime
import hashlib
import json
import os
import pathlib
import platform
import shutil
import socket
import struct
import threading
import time

from tools.elfin_py.link import running_in_wsl
from tools.workbench import runfolder

CHUNK = 8 << 20
KEEP_FREE_BYTES = 10 * 10 ** 9          # leave at least 10 GB free on the backup's disk [proposal]
IMAGE_DATASETS = ("eval", "killtest")   # codebase plan §8: .img of eval and killtest trials only


class BackupError(RuntimeError):
    pass


def default_folder():
    return pathlib.Path.home() / "slosh-backup"


def machine_key():
    """This computer and system, e.g. 'Windows:LAB-PC', 'WSL:LAB-PC', 'Linux:rt-machine'."""
    system = platform.system()
    if system == "Linux" and running_in_wsl():
        system = "WSL"
    return f"{system}:{socket.gethostname()}"


def trial_dataset(bin_path):
    try:
        with open(bin_path, "rb") as f:
            head = f.read(12)
            (n,) = struct.unpack_from("<I", head, 8)
            return json.loads(f.read(n).decode("utf-8")).get("dataset")
    except (OSError, ValueError, struct.error):
        return None


class Backup:
    def __init__(self, data_dir, repo_root, is_busy=lambda: None, settings_path=None):
        self.data = pathlib.Path(data_dir)
        self.repo = pathlib.Path(repo_root)
        self.settings_path = pathlib.Path(settings_path or self.data / "workbench" / "backup_settings.json")
        self.is_busy = is_busy            # returns the active run's trial path (or True) while a run goes on
        self.lock = threading.Lock()
        self.thread = None
        self.progress = None
        self.last_report = None

    # ------------------------------------------------------------------ settings
    def _raw(self):
        """The settings file: {"folders": {machine: path}, "auto": bool}; an older file had one "folder" for all."""
        if self.settings_path.exists():
            try:
                raw = json.loads(self.settings_path.read_text(encoding="utf-8"))
                return raw if isinstance(raw, dict) else {}
            except ValueError:
                return {}
        return {}

    def settings(self):
        """This machine's settings: {"folder", "auto", "machine"}. The older single folder counts only on a machine
        where it is a full path."""
        raw, key = self._raw(), machine_key()
        folder = (raw.get("folders") or {}).get(key)
        legacy = raw.get("folder")
        if folder is None and legacy and pathlib.Path(legacy).is_absolute():
            folder = legacy
        return {"folder": folder, "auto": raw.get("auto", True), "machine": key}

    def folder_problem(self, folder):
        """Why this folder cannot hold the backup on this machine, or None."""
        p = pathlib.Path(folder).expanduser()
        if not p.is_absolute():
            return (f"{folder} is not a full path on this machine ({machine_key()}): "
                    "give one, e.g. C:\\Users\\you\\slosh-backup or /home/you/slosh-backup")
        rp = p.resolve()
        for inside, what in ((self.data.resolve(), "the data folder"), (self.repo.resolve(), "the repository")):
            if rp == inside or inside in rp.parents:
                return f"the backup folder cannot be inside {what} ({inside})"
        return None

    def set_folder(self, folder, auto=True):
        problem = self.folder_problem(folder)
        if problem:
            raise BackupError(problem)
        p = pathlib.Path(folder).expanduser()
        p.mkdir(parents=True, exist_ok=True)
        raw = self._raw()
        raw["folders"] = dict(raw.get("folders") or {}, **{machine_key(): str(p)})
        raw["auto"] = bool(auto)
        self.settings_path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.settings_path.with_suffix(".json.tmp")
        tmp.write_text(json.dumps(raw, indent=1, sort_keys=True), encoding="utf-8")
        os.replace(tmp, self.settings_path)
        return self.settings()

    def dest(self):
        """<folder>/slosh-data for this machine, or None if no usable folder is set here (checked on every use)."""
        f = self.settings().get("folder")
        if not f or self.folder_problem(f):
            return None
        return pathlib.Path(f).expanduser() / "slosh-data"

    # ------------------------------------------------------------------ status
    def status(self):
        s = self.settings()
        d = self.dest()
        out = {"folder": s.get("folder"), "suggested": str(default_folder()), "auto": s.get("auto", True),
               "machine": s["machine"], "folder_problem": self.folder_problem(s["folder"]) if s.get("folder") else None,
               "set_up": d is not None, "running": bool(self.thread and self.thread.is_alive()),
               "progress": self.progress, "last_report": self.last_report}
        if d is None:
            return out
        runs = [x for x in (d / "runs").iterdir() if x.is_dir() and runfolder.NAME_RE.match(x.name)] if (d / "runs").exists() else []
        trials = list((d / "trials").glob("*.bin")) if (d / "trials").exists() else []
        held = sum(p.stat().st_size for p in d.rglob("*") if p.is_file() and p.name != "trial.bin") if d.exists() else 0
        log = self.log()
        probe = d
        while not probe.exists() and probe.parent != probe:
            probe = probe.parent
        out.update(runs_held=len(runs), trials_held=len(trials), bytes_held=held,
                   last_backup=log[-1] if log else None, free_bytes=shutil.disk_usage(probe).free)
        return out

    def log(self):
        d = self.dest()
        p = d / "backup_log.jsonl" if d else None
        if not p or not p.exists():
            return []
        return [json.loads(x) for x in p.read_text(encoding="utf-8").splitlines() if x.strip()]

    # ------------------------------------------------------------------ copying
    def _wait_idle(self):
        while self.is_busy():
            self.progress = dict(self.progress or {}, paused="a run is active; the backup waits for it to end")
            time.sleep(0.5)
        if self.progress and "paused" in self.progress:
            self.progress.pop("paused")

    def _copy_file(self, src, dst, report):
        """Copy one file unless the backup already has it. Never overwrites: a different file there is a conflict."""
        src, dst = pathlib.Path(src), pathlib.Path(dst)
        size = src.stat().st_size
        if dst.exists():
            if dst.stat().st_size == size and (size > 64 << 20 or runfolder.sha256_file(dst) == runfolder.sha256_file(src)):
                report["skipped"] += 1
                return "same"
            report["conflicts"].append(f"{dst} differs from {src}; the backup copy was left as it is")
            return "conflict"
        dst.parent.mkdir(parents=True, exist_ok=True)
        free = shutil.disk_usage(dst.parent).free
        if free - size < KEEP_FREE_BYTES:
            raise BackupError(f"not enough space on the backup disk for {src.name}: {free / 1e9:.1f} GB free, "
                              f"{KEEP_FREE_BYTES / 1e9:.0f} GB must stay free")
        part = dst.with_name(dst.name + ".partial")
        h = hashlib.sha256()
        with open(src, "rb") as fi, open(part, "wb") as fo:
            while True:
                self._wait_idle()
                b = fi.read(CHUNK)
                if not b:
                    break
                h.update(b)
                fo.write(b)
            fo.flush()
            os.fsync(fo.fileno())
        if part.stat().st_size != size:
            raise BackupError(f"{src.name}: copied {part.stat().st_size} bytes of {size}")
        os.replace(part, dst)
        shutil.copystat(src, dst)
        report["copied_files"] += 1
        report["copied_bytes"] += size
        rel = str(dst.relative_to(self.dest())).replace("\\", "/").replace(".partial/", "/")
        report["manifest"].append({"file": rel, "bytes": size, "sha256": h.hexdigest()})
        return "copied"

    def _copy_run(self, src_dir, dst_root, trials_dst, report):
        """A run folder: into <name>.partial, then renamed, so a half-copied folder never looks complete."""
        name = src_dir.name
        final = dst_root / name
        if final.exists():
            for p in src_dir.rglob("*"):          # the backup already has it: only check that nothing differs
                if p.is_file() and p.name != "trial.bin":
                    q = final / p.relative_to(src_dir)
                    if not q.exists() or q.stat().st_size != p.stat().st_size:
                        report["conflicts"].append(f"{q} differs from {p} (or is missing); the backup was left as it is")
            report["skipped"] += 1
            return
        part = dst_root / (name + ".partial")
        for p in sorted(src_dir.rglob("*")):
            if p.is_file() and p.name != "trial.bin":
                self._copy_file(p, part / p.relative_to(src_dir), report)
        rec = json.loads((src_dir / "run.json").read_text(encoding="utf-8")) if (src_dir / "run.json").exists() else {}
        tid = rec.get("trial_id")
        if tid and (trials_dst / f"{tid}.bin").exists() and not (part / "trial.bin").exists():
            try:
                os.link(trials_dst / f"{tid}.bin", part / "trial.bin")
            except OSError:
                pass                              # trial.txt still says where it is
        os.replace(part, final)
        runfolder.make_read_only(final)
        report["runs_copied"].append(name)

    def run(self, reason="button"):
        d = self.dest()
        if d is None:
            raise BackupError("no backup folder yet: choose one on the Backup tab")
        t0 = time.monotonic()
        report = {"started": runfolder.stamp(), "reason": reason, "copied_files": 0, "copied_bytes": 0, "skipped": 0,
                  "conflicts": [], "runs_copied": [], "trials_copied": [], "manifest": [], "errors": []}
        self.progress = {"phase": "listing"}
        try:
            (d / "runs").mkdir(parents=True, exist_ok=True)
            (d / "trials").mkdir(parents=True, exist_ok=True)
            active = self.is_busy()
            active = pathlib.Path(active).resolve() if isinstance(active, (str, os.PathLike)) else None
            trials = sorted((self.data / "trials").glob("*.bin")) if (self.data / "trials").exists() else []
            files = []
            for t in trials:
                if active is not None and t.resolve() == active:
                    continue                      # still being written
                files.append(t)
                img = t.with_suffix(".img")
                if img.exists() and trial_dataset(t) in IMAGE_DATASETS:
                    files.append(img)
            runs = [r for r in sorted((self.data / "runs").iterdir()) if r.is_dir() and runfolder.NAME_RE.match(r.name)
                    and runfolder.is_complete(r)] if (self.data / "runs").exists() else []
            n = len(files) + len(runs)
            for k, f in enumerate(files, 1):
                self.progress = {"phase": "trials", "item": f.name, "done": k - 1, "of": n}
                if self._copy_file(f, d / "trials" / f.name, report) == "copied":
                    report["trials_copied"].append(f.name)
            for k, r in enumerate(runs, len(files) + 1):
                self.progress = {"phase": "runs", "item": r.name, "done": k - 1, "of": n}
                self._copy_run(r, d / "runs", d / "trials", report)
        except Exception as exc:                  # reported, never silent; nothing in the backup is removed
            report["errors"].append(repr(exc))
        report["duration_s"] = round(time.monotonic() - t0, 2)
        report["ended"] = runfolder.stamp()
        line = {k: v for k, v in report.items() if k != "manifest"}
        line["files"] = report["manifest"]
        with open(d / "backup_log.jsonl", "a", encoding="utf-8") as f:
            f.write(json.dumps(line, sort_keys=True) + "\n")
        self.last_report = {k: v for k, v in report.items() if k != "manifest"}
        self.progress = None
        return self.last_report

    def start(self, reason="button"):
        with self.lock:
            if self.thread and self.thread.is_alive():
                return False
            if self.dest() is None:
                raise BackupError("no backup folder yet: choose one on the Backup tab")
            self.thread = threading.Thread(target=self.run, args=(reason,), name="backup", daemon=True)
            self.thread.start()
            return True

    def wait(self, timeout=None):
        if self.thread:
            self.thread.join(timeout)

    # ------------------------------------------------------------------ restore
    def restore(self, name, overwrite=False):
        """Copy a run folder (and its trial) from the backup back into the data folder. Without overwrite, nothing
        that exists and differs is touched: the answer lists the differences for the owner to decide."""
        runfolder.check_name(name)
        d = self.dest()
        if d is None:
            raise BackupError("no backup folder set")
        src = d / "runs" / name
        if not src.is_dir():
            raise BackupError(f"{name} is not in the backup")
        target = self.data / "runs" / name
        rec = json.loads((src / "run.json").read_text(encoding="utf-8")) if (src / "run.json").exists() else {}
        tid = rec.get("trial_id")
        pairs = [(p, target / p.relative_to(src)) for p in sorted(src.rglob("*")) if p.is_file() and p.name != "trial.bin"]
        if tid and (d / "trials" / f"{tid}.bin").exists():
            pairs.append((d / "trials" / f"{tid}.bin", self.data / "trials" / f"{tid}.bin"))
        missing, same, differ = [], [], []
        for a, b in pairs:
            if not b.exists():
                missing.append((a, b))
            elif b.stat().st_size == a.stat().st_size and runfolder.sha256_file(a) == runfolder.sha256_file(b):
                same.append((a, b))
            else:
                differ.append((a, b))
        if differ and not overwrite:
            return {"needs_confirmation": True, "restored": [], "differ": [str(b) for _, b in differ],
                    "missing": [str(b) for _, b in missing]}
        stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
        kept = []
        if target.exists() and (missing or differ):
            runfolder.make_writable(target)
        for a, b in differ:                       # owner confirmed: keep the current file beside, never delete it
            aside = b.with_name(f"{b.name}.before-restore-{stamp}")
            if os.name == "nt" and b.exists():
                os.chmod(b, 0o666)
            os.replace(b, aside)
            kept.append(str(aside))
        restored = []
        for a, b in missing + differ:
            b.parent.mkdir(parents=True, exist_ok=True)
            part = b.with_name(b.name + ".partial")
            shutil.copyfile(a, part)
            os.replace(part, b)
            shutil.copystat(a, b)
            restored.append(str(b))
        cur = self.data / "trials" / f"{tid}.bin" if tid else None
        link = target / "trial.bin"
        if cur is not None and cur.exists() and target.exists():
            if link.exists() and not os.path.samefile(link, cur):     # it pointed at the replaced trial: keep it aside
                runfolder.make_writable(target)
                aside = link.with_name(f"trial.bin.before-restore-{stamp}")
                if os.name == "nt":
                    os.chmod(link, 0o666)
                os.replace(link, aside)
                kept.append(str(aside))
            if not link.exists():
                if os.name != "nt":
                    os.chmod(target, 0o755)
                try:
                    os.link(cur, link)
                except OSError:
                    pass                          # trial.txt says where the trial is
        for k in kept:
            os.chmod(k, 0o444)
        if target.exists():
            runfolder.make_read_only(target)
        return {"needs_confirmation": False, "restored": restored, "kept_aside": kept,
                "unchanged": [str(b) for _, b in same]}
