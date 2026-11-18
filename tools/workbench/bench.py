"""The workbench: stages, runs, accepts, coefficients and the confirmed limits (codebase plan §12).

It starts one stage tool at a time and reads what the tool writes; it never talks to the arm itself.
Standard library only.
"""
import datetime
import json
import os
import pathlib
import re
import shutil
import subprocess
import sys
import threading
import time

from tools.elfin_py import config as armconfig
from tools.elfin_py import link, triallog
from analysis.logread import read_trial
from tools.workbench import evaluate, runfolder
from tools.workbench.backup import Backup, BackupError
from tools.workbench.coeffs import CoeffError, CoeffStore
from tools.workbench.datalock import DataLock
from tools.workbench.livelog import LiveLog
from tools.workbench.runner import MockProcess, ToolRun
from tools.workbench.stages import ROOT, Registry, primary_mode
from tools.workbench.status import Status, session_id, utc_now

CONFIRM_PHRASE = "I confirm the new limits"
ARM_MODES = ("dry-run", "mock", "arm")
OPEN_TRIAL_GRACE_S = 24 * 3600.0   # a trial log never closed counts as abandoned after a day unchanged [proposal]


class Refused(RuntimeError):
    """The workbench will not do this; the message says why."""


def open_in_file_manager(path):
    """Show a folder in the operating system's file manager (Explorer, or xdg-open; from WSL, Explorer)."""
    path = str(path)
    if sys.platform == "win32":
        os.startfile(path)                                     # noqa: S606 (a local folder the workbench chose)
    elif link.running_in_wsl():
        win = subprocess.run(["wslpath", "-w", path], capture_output=True, text=True, stdin=subprocess.DEVNULL).stdout.strip()
        subprocess.Popen(["explorer.exe", win], stdin=subprocess.DEVNULL)
    else:
        subprocess.Popen(["xdg-open", path], stdin=subprocess.DEVNULL)


def _trial_from_console(folder):
    """The trial ID from a run folder's console ('[trial] <id> -> <path>'), for folders closed before the record."""
    p = pathlib.Path(folder) / "console.log"
    if not p.exists():
        return None
    for line in p.read_text(encoding="utf-8", errors="replace").splitlines():
        if line.startswith("[trial] "):
            return line.split()[1]
    return None


class Workbench:
    def __init__(self, root=ROOT, data_dir=None, journal=None, confirmed=None, arm_config=None,
                 stages=None, params=None, python=sys.executable, host_problem=None):
        self.root = pathlib.Path(root)
        self.reg = Registry(stages or self.root / "config/workbench/stages.yaml",
                            params or self.root / "config/params.schema.yaml")
        self.data_dir = pathlib.Path(data_dir or os.environ.get("SLOSH_DATA") or self.root / "data")
        self.confirmed_path = pathlib.Path(confirmed or armconfig.CONFIRMED)
        self.arm_config_path = pathlib.Path(arm_config or armconfig.ARM)
        fmap = {"config/arm/confirmed.yaml": self.confirmed_path, "config/arm/e03pro.yaml": self.arm_config_path,
                # the gate's input lives in the data folder, wherever that is (review W1: 4.4 goes stale on an edit)
                "data/workbench/gate_input.json": self.data_dir / "workbench" / "gate_input.json"}
        self.file_map = fmap
        self.coeffs = CoeffStore(self.reg, self.data_dir, self.root, fmap)
        self.status = Status(self.reg, self.coeffs, self.data_dir, journal or self.root / "docs/journal.md", self.root,
                             fmap)
        self.python = python
        self._host_problem = host_problem            # tests inject this; None means ask the tools' own check
        self.run = None
        self.run_folder = None
        self.lock = threading.Lock()
        self.runs_dir = self.data_dir / "runs"       # one folder per run (codebase plan §8, §12)
        self.backup = Backup(self.data_dir, self.root, is_busy=self._busy_trial)
        self.opener = open_in_file_manager           # tests replace it
        self.data_lock = None                        # claim_data_folder(): one workbench per data folder (D6)

    def claim_data_folder(self):
        """Take data/workbench/workbench.lock, or raise datalock.LockHeld (pre-hardware review D6)."""
        self.data_lock = DataLock(self.data_dir).acquire()
        return self.data_lock

    def release_data_folder(self):
        if self.data_lock is not None:
            self.data_lock.release()

    def _busy_trial(self):
        """While a run goes on: its trial path (or True before the tool has announced it). None when idle."""
        r = self.run
        if r is None or not r.running:
            return None
        return r.trial_path or True

    # ------------------------------------------------------------------ host
    def host_problem(self):
        if self._host_problem is not None:
            return self._host_problem or None
        return link.arm_host_problem()

    def coeff_value(self, name):
        return self.coeffs.value(name)

    # ------------------------------------------------------------------ views
    def overview(self):
        hp = self.host_problem()
        rows = []
        for sid in self.reg.order:
            s = self.reg.stages[sid]
            h = self.status.inputs_hash(sid)
            rows.append({"id": sid, "num": s["num"], "group": s["group"], "title": s["title"], "status": s["status"],
                         "modes": s["modes"], "primary": primary_mode(s),
                         "per_mode": {m: self.status.status(sid, m, h)["status"] for m in s["modes"]},
                         "locked": bool(self.status.locks(sid, primary_mode(s), hp))})
        return {"stages": rows, "groups": self.reg.groups, "host_problem": hp, "session": session_id(),
                "coeff_version": self.coeffs.current_version(), "run": self.run.view(10 ** 9) if self.run else None,
                "confirmed_uncommitted": self.confirmed_uncommitted(),
                "unreadable_records": self.status.unreadable}      # skipped, never fatal (review D7)

    def stage_view(self, sid):
        s = self.reg.stages[sid]
        hp = self.host_problem()
        h = self.status.inputs_hash(sid)
        return {"stage": s, "per_mode": {m: self.status.status(sid, m, h) for m in s["modes"]},
                "locks": {m: self.status.locks(sid, m, hp) for m in s["modes"]},
                "runs": self.status.runs(sid)[-10:], "results": self.status.results(sid)[-10:],
                "coeffs": [r for r in self.coeffs.table()["rows"] if r["name"] in s["adjust"]],
                "upstream": self.reg.upstream(sid), "downstream": self.reg.downstream(sid),
                "host_problem": hp}

    # ------------------------------------------------------------------ runs
    def _argv(self, s, mode, note, assume_yes, options, result_json, mock, confirmed=None, arm_config=None):
        tool = s["tool"]
        if tool.endswith(".sh"):
            argv = ["bash", tool]
        else:
            argv = [self.python, "-u", "-m", tool]
        if mode in ARM_MODES:
            argv.append(f"--{mode}")
            if mode == "dry-run":
                argv.append("--mirror-config")
            if mode == "mock":
                argv += ["--cmd-port", str(mock.ports[0]), "--state-port", str(mock.ports[1])]
            argv += ["--note", note or ""]
            if assume_yes:
                argv.append("--assume-yes")
            confirmed, arm_config = confirmed or self.confirmed_path, arm_config or self.arm_config_path
            if confirmed != armconfig.CONFIRMED:
                argv += ["--confirmed", str(confirmed)]
            if arm_config != armconfig.ARM:
                argv += ["--arm-config", str(arm_config)]
        else:
            argv += ["--result-json", str(result_json)]
        argv += ["--data-dir", str(self.data_dir)]
        argv += list(s["args"])
        for key, val in (options or {}).items():
            opt = s["options"].get(key)
            if opt is None:
                raise Refused(f"unknown option {key}")
            if val not in (None, ""):
                argv += [opt["flag"], str(val)]
        return argv

    def practice_configs(self):
        """Copies of confirmed.yaml and e03pro.yaml with the mock's limits in place of the arm's, for practising
        in dry run or mock before the arm's limits have been read (stage 1.1). Never used for the arm."""
        from tools.elfin_mock.sim import MockConfig
        d = self.data_dir / "workbench" / "practice"
        d.mkdir(parents=True, exist_ok=True)
        mc = MockConfig()
        fmt = lambda v: "[" + ", ".join(f"{x:.1f}" for x in v) + "]"
        block = ("limits_readback:            # PRACTICE: the mock's values, not the arm's\n"
                 "  record_id: practice-mock\n"
                 f"  joint_max_vel_deg_s: {fmt(mc.joint_max_vel)}\n  joint_max_acc_deg_s2: {fmt(mc.joint_max_acc)}\n"
                 f"  joint_max_jerk_deg_s3: {fmt(mc.joint_max_jerk)}\n  linear_max: {fmt(mc.linear_max)}\n"
                 "  tolerance: 0.001")
        arm = self.arm_config_path.read_text(encoding="utf-8")
        start = arm.index("limits_readback:")
        end = arm.index("\n\n", start)
        arm = "# PRACTICE COPY written by the workbench: never used on the arm\n" + arm[:start] + block + arm[end:]
        (d / "e03pro.yaml").write_text(arm, encoding="utf-8")
        (d / "confirmed.yaml").write_text(self.confirmed_path.read_text(encoding="utf-8"), encoding="utf-8")
        armconfig.load(d / "confirmed.yaml", d / "e03pro.yaml")
        return d / "confirmed.yaml", d / "e03pro.yaml"

    def start(self, sid, mode, note="", assume_yes=False, options=None, override_reason=None, practice=False):
        s = self.reg.stages.get(sid)
        if s is None:
            raise Refused(f"no stage {sid}")
        if mode not in s["modes"]:
            raise Refused(f"{sid} has no {mode} mode")
        if mode == "form":
            raise Refused("a form stage is submitted, not run")
        if s["status"] != "built":
            raise Refused(f"{s['num']} {sid}: the tool is {s['status']} ({s['tool']})")
        if mode == "arm" and assume_yes:
            raise Refused("prompts are never auto-answered on the arm")
        if practice and mode not in ("dry-run", "mock"):
            raise Refused("practice runs are for dry run and mock only")
        if self.data_lock is not None and self.data_lock.lost:
            lost = self.data_lock.lost
            raise Refused(f"another workbench took over this data folder (process {lost.get('pid')} on "
                          f"{lost.get('machine')}): this one starts no more runs; close it")
        with self.lock:
            if self.run is not None and self.run.running:
                raise Refused(f"{self.run.stage} is still running; stop it or wait")
            locks = [] if practice else self.status.locks(sid, mode, self.host_problem())
            hard = [l["reason"] for l in locks if not l["overridable"]]
            if hard:
                raise Refused("; ".join(hard))
            if locks and not (override_reason and override_reason.strip()):
                raise Refused("locked: " + "; ".join(l["reason"] for l in locks) + " (an override needs a reason)")
            if mode == "arm" and (pathlib.Path(self.confirmed_path).resolve() != armconfig.CONFIRMED.resolve()
                                  or pathlib.Path(self.arm_config_path).resolve() != armconfig.ARM.resolve()):
                raise Refused("arm runs use config/arm/confirmed.yaml and config/arm/e03pro.yaml only; this workbench "
                              "was started with other files (pre-hardware review S14)")
            run_id = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%d-%H%M%S-") + f"{sid}-{mode}"
            if locks:
                self.status.audit("override", stage=sid, mode=mode, run_id=run_id, reason=override_reason,
                                  locks=[l["reason"] for l in locks])
            result_json = self.data_dir / "workbench" / "runs" / f"{run_id}.result.json"
            confirmed, arm_config = self.practice_configs() if practice else (self.confirmed_path, self.arm_config_path)
            rf, snap_path = self._open_folder(s, mode, practice, confirmed, arm_config)   # the run's snapshot
            try:
                mock = MockProcess(self.root, arm_config, self.python) if mode == "mock" else None
            except Exception as exc:
                rf.finish(self._folder_record(s, {"run_id": run_id, "stage": sid, "mode": mode, "practice": practice,
                                                  "note": note, "error": repr(exc)}))
                raise
            meta = {"run_id": run_id, "stage": sid, "mode": mode, "practice": bool(practice), "note": note}
            try:
                argv = self._argv(s, mode, ("PRACTICE: " if practice else "") + (note or ""), assume_yes, options,
                                  result_json, mock, confirmed, arm_config)
                env = dict(os.environ, SLOSH_WORKBENCH="1", SLOSH_WB_STAGE=sid, SLOSH_WB_COEFFS=str(snap_path),
                           PYTHONUNBUFFERED="1", PYTHONIOENCODING="utf-8", SLOSH_DATA=str(self.data_dir))
                meta = {"run_id": run_id, "stage": sid, "mode": mode, "note": note, "argv": argv,
                        "started_utc": utc_now(), "inputs_hash": self.status.inputs_hash(sid),
                        "coeff_version": self.coeffs.current_version(), "session": session_id(),
                        "override": override_reason if locks else None, "result_json": str(result_json),
                        "practice": bool(practice), "run_folder": rf.name,
                        "started_local": runfolder.stamp(rf.started)}
                rf.write_json("run.json", dict(self._folder_record(s, meta), status="running"))
                rf.note("command: " + subprocess.list2cmdline([str(a) for a in argv]))
                if practice:
                    rf.note("PRACTICE: copies of the config with the mock's limits; never counts for the arm")
                if locks:
                    rf.note(f"OVERRIDE of {[l['reason'] for l in locks]}: {override_reason}")
                if mock:
                    rf.note(f"mock controller on ports {mock.ports}")
                self.run_folder = rf
                self.run = ToolRun(run_id, sid, mode, argv, env, self.root,
                                   on_exit=lambda r: self._finish(r, meta), mock=mock, sink=rf)
            except Exception as exc:
                if mock:
                    mock.stop()
                try:
                    rf.finish(self._folder_record(s, dict(meta, error=repr(exc))))
                except runfolder.RunFolderError:
                    pass                                   # already closed; the original error matters
                raise
        return run_id

    def _finish(self, run, meta):
        s = self.reg.stages[meta["stage"]]
        try:
            if meta["mode"] in ARM_MODES:
                metrics = evaluate.tool_metrics(s["tool"], run.trial_path, run.exit_code, self.arm_config_path)
            else:
                p = pathlib.Path(meta["result_json"])
                metrics = json.loads(p.read_text(encoding="utf-8")) if p.exists() else {}
                metrics.update(exit_code=run.exit_code, exit_ok=run.exit_code == 0)
                metrics = self._analysis_metrics(meta["stage"], metrics)
            crit, auto_ok = evaluate.evaluate(s, metrics, self.coeff_value)
        except Exception as exc:                       # an evaluation error is a failed run, never a pass
            metrics, crit, auto_ok = {"evaluation_error": repr(exc)}, [], False
        auto_ok = auto_ok and run.exit_code == 0 and not run.stop_sent
        rec = dict(meta, ended_utc=utc_now(), exit_code=run.exit_code, trial_id=run.trial_id,
                   trial_path=run.trial_path, metrics=metrics, criteria=crit, auto_ok=auto_ok,
                   stopped=run.stop_sent, console_tail=list(run.console)[-60:])
        self.status.save_run(rec)
        rf = run.sink
        if rf is not None:
            try:
                rf.note(f"tool ended, exit code {run.exit_code}")
                rf.finish(self._folder_record(s, dict(rec, ended_local=runfolder.stamp())), run.trial_path,
                          extra_logs={"mock.log": run.mock.lines} if run.mock else None)
            except Exception as exc:                   # reported; the run record above is already safe
                self.status.audit("run_folder_error", stage=meta["stage"], run_id=meta["run_id"], note=repr(exc))
        self.status.journal("run", {"stage": meta["stage"], "mode": meta["mode"], "trial_id": run.trial_id,
                                    "run_folder": meta.get("run_folder"),
                                    "verdict": "criteria met" if auto_ok else ("stopped" if run.stop_sent else "failed")})
        self._auto_backup(f"after run {meta.get('run_folder')}")

    def _analysis_metrics(self, sid, metrics):
        """What an analysis stage's own result cannot know. The gate (4.4): its memo v2 with the CNN's numbers counts
        only once the CNN front-end has passed stage 4.5 (pre-hardware review W1)."""
        if sid == "camera_gate":
            fe = metrics.get("front_end", "classical")
            metrics["front_end_accepted"] = fe == "classical" or (fe == "cnn" and self.status.passed("cnn_frontend"))
        return metrics

    # ------------------------------------------------------------------ run folders
    def _folder_record(self, s, rec):
        return dict(rec, stage_num=s["num"], stage_title=s["title"])

    def _open_folder(self, s, mode, practice, confirmed, arm_config):
        """The run folder with its inputs. Returns it, and the path of the complete coefficient snapshot it holds:
        the version's values plus the schema default of any coefficient the version lacks (review W7)."""
        rf = runfolder.RunFolder(self.runs_dir, s["id"], mode, practice)
        files = {"confirmed.yaml": confirmed, "e03pro.yaml": arm_config}
        for spec in s["files"]:
            rel = spec.split("#", 1)[0]
            name = pathlib.Path(rel).name
            if name not in files:
                files[name] = self.file_map.get(rel) or self.root / rel
        rf.copy_inputs(files, None, s)
        snap = self.coeffs.complete_snapshot()
        name = f"coeffs_v{snap['version']:06d}.json"
        rf.write_json(name, snap)
        return rf, rf.path / name

    def _auto_backup(self, reason):
        try:
            if self.backup.settings().get("folder") and self.backup.settings().get("auto", True):
                self.backup.start(reason)
        except BackupError:
            pass

    def recover_runs(self, quiet_s=60.0, open_grace_s=OPEN_TRIAL_GRACE_S):
        """At start-up: close run folders left open by a workbench that stopped mid-run (marked INTERRUPTED), and
        give trials without a folder one of their own (marked RECONSTRUCTED): runs started outside the workbench, from
        a terminal, or from before run folders existed.
        - A run folder or run record that cannot be read is reported and left alone (pre-hardware review D7).
        - A trial log that is not closed yet (triallog.is_closed) is left alone while it has changed within
          open_grace_s: a tool started from a terminal may be waiting at a prompt (review D8)."""
        out = {"interrupted": [], "reconstructed": [], "unreadable": [], "still_open": []}
        self.runs_dir.mkdir(parents=True, exist_ok=True)
        known_runs, known_trials = set(), set()
        for d in self.runs_dir.iterdir():
            if not d.is_dir() or not runfolder.NAME_RE.match(d.name):
                continue
            try:
                rec = json.loads((d / "run.json").read_text(encoding="utf-8")) if (d / "run.json").exists() else {}
                if not isinstance(rec, dict):
                    raise ValueError("not a JSON object")
            except (OSError, ValueError) as exc:
                out["unreadable"].append((d.name, repr(exc)))
                known_trials.add(_trial_from_console(d))    # its trial is not given a second folder
                continue
            known_runs.add(rec.get("run_id"))
            trial = rec.get("trial_id") or _trial_from_console(d)
            known_trials.add(trial)
            if runfolder.is_complete(d) or (self.run is not None and self.run_folder and self.run_folder.name == d.name):
                continue
            tpath = self.data_dir / "trials" / f"{trial}.bin" if trial else None
            if tpath is not None and tpath.exists() and not triallog.is_closed(tpath) \
                    and time.time() - tpath.stat().st_mtime < quiet_s:
                continue                              # its tool may still be writing; next start-up closes it
            s = self.reg.stages.get(rec.get("stage"), {"num": "?", "title": "?"})
            rf = runfolder.RunFolder.reopen(d)
            rf.note("the workbench stopped before this run ended; closed at the next start-up")
            rf.finish(dict(rec, interrupted=True, trial_id=trial, ended_local=None, stage_num=s.get("num"),
                           stage_title=s.get("title")), tpath if tpath is not None and tpath.exists() else None)
            out["interrupted"].append(d.name)
            self.status.journal("run", {"stage": rec.get("stage"), "mode": rec.get("mode"), "trial_id": trial,
                                        "run_folder": d.name,                 # every run gets its line (review I5)
                                        "verdict": "interrupted: the workbench stopped during the run"})
        records = []
        rdir = self.data_dir / "workbench" / "runs"
        for p in sorted(rdir.glob("*.json")) if rdir.exists() else []:
            if p.name.endswith(".result.json"):
                continue
            try:
                r = json.loads(p.read_text(encoding="utf-8"))
                if not isinstance(r, dict):
                    raise ValueError("not a JSON object")
                records.append(r)
            except (OSError, ValueError) as exc:
                out["unreadable"].append((f"workbench/runs/{p.name}", repr(exc)))
        for rec in records:
            if rec.get("run_folder") or rec.get("run_id") in known_runs:
                continue
            out["reconstructed"].append(self._reconstruct(rec, rec.get("trial_path")))
            known_trials.add(rec.get("trial_id"))
        for t in sorted((self.data_dir / "trials").glob("*.bin")) if (self.data_dir / "trials").exists() else []:
            if t.stem in known_trials:
                continue
            if self._busy_trial() and str(t) == str(self._busy_trial()):
                continue
            age = time.time() - t.stat().st_mtime
            if not triallog.is_closed(t) and age < open_grace_s:
                if age >= quiet_s:                    # quiet but not closed: say so; a later start-up takes it in
                    out["still_open"].append(t.stem)
                continue
            out["reconstructed"].append(self._reconstruct(None, t))
        return out

    def _reconstruct(self, rec, trial_path):
        trial = None
        if trial_path and pathlib.Path(trial_path).exists():
            try:
                trial = read_trial(trial_path)
            except Exception:
                trial = None
        h = trial.header if trial else {}
        wb = h.get("workbench") or {}
        if rec is None:
            rec = {"run_id": None, "stage": wb.get("stage") or h.get("tool"), "mode": h.get("mode"),
                   "trial_id": pathlib.Path(trial_path).stem, "trial_path": str(trial_path),
                   "coeff_version": wb.get("coeff_version"), "note": h.get("operator_note"),
                   "started_utc": h.get("wall_clock_utc"),
                   "outside_workbench": bool(trial) and not h.get("workbench"),     # a terminal run (review D8)
                   "log_closed": triallog.is_closed(trial_path)}
        when = None
        for key in ("started_utc",):
            try:
                when = datetime.datetime.fromisoformat(str(rec.get(key)).replace("Z", "+00:00")).astimezone()
            except ValueError:
                when = None
        if when is None and h.get("wall_clock_utc"):
            when = datetime.datetime.fromisoformat(h["wall_clock_utc"]).astimezone()
        sid = rec.get("stage") or "unknown"
        s = self.reg.stages.get(sid, {"num": "?", "title": sid})
        practice = bool(rec.get("practice")) or "PRACTICE" in str(h.get("operator_note", ""))
        rf = runfolder.RunFolder(self.runs_dir, sid, rec.get("mode") or "unknown", practice, when=when)
        tail = rec.get("console_tail")
        rf.console("[reconstructed: " + ("only the last lines the run record kept" if tail else
                                         "a run started outside the workbench; from its trial log only, no console "
                                         "was kept" if rec.get("outside_workbench") else
                                         "from the trial log only; no console was kept")
                   + ("; the trial log was never closed (its tool ended without closing it)"
                      if rec.get("log_closed") is False else "") + "]")
        for line in tail or []:
            rf.console(line)
        if trial is not None:
            for r in trial.stream("event"):
                text = r["text"].decode("utf-8", "replace")
                if r["kind"] == 4 and text.startswith(("prompt", "prompt auto-answered")):
                    rf._append_json("questions.jsonl", {"event": "from the trial log", "text": text,
                                                         "t_mono_ns": r["t_host_ns"]})
            cfg = h.get("config") or {}
            if cfg:
                (rf.path / "config").mkdir(exist_ok=True)
                for name, text in cfg.items():
                    (rf.path / "config" / name).write_text(text, encoding="utf-8")
        rf.finish(dict(rec, reconstructed=True, practice=practice, stage_num=s.get("num"), stage_title=s.get("title"),
                       started_local=runfolder.stamp(when) if when else None),
                  trial_path if trial_path and pathlib.Path(trial_path).exists() else None)
        self.status.journal("run", {"stage": sid if sid in self.reg.stages else None, "mode": rec.get("mode"),
                                    "trial_id": rec.get("trial_id"), "run_folder": rf.name,
                                    "verdict": "reconstructed: " + ("started outside the workbench"
                                                                    if rec.get("outside_workbench") else
                                                                    "from before run folders existed")})
        return rf.name

    def past_runs(self, source="data"):
        root = self._runs_root(source)
        return {"source": source, "root": str(root) if root else None,
                "runs": runfolder.list_runs(root) if root else []}

    def _runs_root(self, source):
        if source == "data":
            return self.runs_dir
        if source == "backup":
            d = self.backup.dest()
            return d / "runs" if d else None
        raise Refused(f"unknown source {source}")

    def past_run(self, name, source="data"):
        root = self._runs_root(source)
        if root is None:
            raise Refused("no backup folder set")
        trials = self.data_dir / "trials" if source == "data" else self.backup.dest() / "trials"
        try:
            d = runfolder.read_run(root, name, trials)
        except runfolder.RunFolderError as exc:
            raise Refused(str(exc))
        d["plot"] = None
        if d["trial_path"]:
            try:
                live = LiveLog(d["trial_path"])
                live.poll()
                d["plot"] = live.snapshot()
            except Exception as exc:
                d["plot_error"] = repr(exc)
        d["source"] = source
        return d

    def open_run_folder(self, name, source="data"):
        root = self._runs_root(source)
        if root is None:
            raise Refused("no backup folder set")
        try:
            path = (root / runfolder.check_name(name)).resolve()
        except runfolder.RunFolderError as exc:
            raise Refused(str(exc))
        if root.resolve() not in path.parents or not path.is_dir():
            raise Refused(f"no run folder {name}")
        try:
            self.opener(path)
        except OSError as exc:                  # no file manager here (no xdg-open, say): give the path instead (I2)
            return {"opened": None, "path": str(path),
                    "note": f"no file manager could be started ({exc.__class__.__name__}); open this folder yourself"}
        return {"opened": str(path), "path": str(path)}

    # ------------------------------------------------------------------ backup
    def backup_view(self):
        st = self.backup.status()
        runs = runfolder.list_runs(self.runs_dir)
        trials = list((self.data_dir / "trials").glob("*.bin")) if (self.data_dir / "trials").exists() else []
        imgs = list((self.data_dir / "trials").glob("*.img")) if (self.data_dir / "trials").exists() else []
        st["data"] = {"runs": len(runs), "run_folder_bytes": sum(r["folder_bytes"] for r in runs),
                      "trial_bytes": sum(t.stat().st_size for t in trials), "image_bytes": sum(i.stat().st_size for i in imgs),
                      "free_bytes": shutil.disk_usage(self.data_dir if self.data_dir.exists() else self.root).free,
                      "runs_dir": str(self.runs_dir)}
        return st

    def backup_now(self):
        try:
            if not self.backup.settings().get("folder"):
                raise Refused("choose the backup folder first (Backup tab)")
            started = self.backup.start("button")
        except BackupError as exc:
            raise Refused(str(exc))
        return {"started": started, "note": None if started else "a backup is already running"}

    def backup_settings(self, folder, auto=True):
        try:
            s = self.backup.set_folder(folder, auto)
        except BackupError as exc:
            raise Refused(str(exc))
        self.status.audit("backup_folder", note=f"{s['folder']} (auto after each run: {s['auto']})")
        return s

    def restore_run(self, name, overwrite=False):
        if self.run is not None and self.run.running:
            raise Refused("a run is going on; restore when it has ended")
        try:
            out = self.backup.restore(name, overwrite)
        except (BackupError, runfolder.RunFolderError) as exc:
            raise Refused(str(exc))
        if not out.get("needs_confirmation"):
            self.status.audit("restore", note=f"{name}: restored {len(out['restored'])} files, "
                                              f"kept {len(out.get('kept_aside', []))} current files aside")
        return out

    # ------------------------------------------------------------------ arm control (jog tool)
    def _same_run(self, run_id, what):
        """A page that shows an earlier run must not act on a newer one (pre-hardware review S16)."""
        if run_id is not None and run_id != self.run.run_id:
            raise Refused(f"that {what} was for run {run_id}, but the current run is {self.run.run_id}: "
                          "the page is out of date; reload it")

    def command(self, name, args, run_id=None):
        """A command for the running jog session (Arm control tab). The tool checks and refuses; this only relays."""
        if self.run is None or not self.run.running or self.run.stage != "arm_control":
            raise Refused("no arm-control session is running: connect first")
        self._same_run(run_id, "command")
        if self.run.arm.get("ready") is None:
            raise Refused("the arm is not ready yet: answer the questions first")
        if self.run.arm.get("busy"):
            raise Refused("a move is under way; wait for it, or press STOP")
        if name not in ("jog", "goto", "path", "keep", "end"):
            raise Refused(f"unknown command {name}")
        cid = self.run.command(name, args)
        if cid is None:
            raise Refused("the session has ended")
        return {"id": cid}

    def _points_dir(self):
        d = self.data_dir / "workbench" / "points"
        d.mkdir(parents=True, exist_ok=True)
        return d

    def paths(self):
        out = {}
        for p in sorted(self._points_dir().glob("*.json")):
            out[p.stem] = json.loads(p.read_text(encoding="utf-8"))
        return out

    def save_path(self, name, points, note=""):
        """A named list of points (start, middles, end) recorded from the arm's pose."""
        if not re.fullmatch(r"[A-Za-z0-9_\-]{1,40}", name or ""):
            raise Refused("a path name is 1-40 letters, digits, - or _")
        clean = []
        for p in points or []:
            try:
                pt = {"name": str(p["name"])[:20], "role": p.get("role", "middle"),
                      "pose": [float(x) for x in p["pose"]], "tool": [float(x) for x in p["tool"]],
                      "user": [float(x) for x in p["user"]], "joints": [float(x) for x in p.get("joints") or []]}
            except (KeyError, TypeError, ValueError):
                raise Refused("each point needs a name and pose, tool and user frames of 6 numbers")
            if len(pt["pose"]) != 6 or len(pt["tool"]) != 6 or len(pt["user"]) != 6:
                raise Refused("pose, tool and user frames have 6 values each")
            clean.append(pt)
        roles = [pt["role"] for pt in clean]
        if roles and (roles.count("start") > 1 or roles.count("end") > 1):
            raise Refused("one start and one end at most")
        path = self._points_dir() / f"{name}.json"
        path.write_text(json.dumps({"name": name, "note": note, "saved_utc": utc_now(), "points": clean}, indent=1),
                        encoding="utf-8")
        self.status.audit("points", path=name, changed=[pt["name"] for pt in clean], note=note)
        return {"name": name, "points": clean}

    def stop(self, run_id=None):
        """The stop button. It always stops the current run, even from a page that shows an earlier one."""
        if self.run is None or not self.run.running:
            raise Refused("nothing is running")
        sent = self.run.stop()
        if run_id is not None and run_id != self.run.run_id:
            self.run.note(f"[the STOP came from a page that showed run {run_id}]")
        return sent

    def answer(self, qid, text, run_id=None):
        if self.run is None or not self.run.running:
            raise Refused("nothing is running")
        self._same_run(run_id, "answer")
        if not self.run.answer(qid, text):
            raise Refused("that question is no longer open")

    # ------------------------------------------------------------------ accept and forms
    def accept(self, run_id, note, operator_checks=None):
        run = self.status.run(run_id)
        if run is None:
            raise Refused(f"no run {run_id}")
        s = self.reg.stages[run["stage"]]
        if run.get("practice"):           # pre-hardware review S6: an accepted practice run unlocked the arm
            raise Refused("a practice run ran on copies of the config with the mock's limits, not confirmed.yaml; "
                          "it never counts and cannot be accepted. Run the stage without practice.")
        if not note or not note.strip():
            raise Refused("accepting needs a note")
        if not run.get("auto_ok"):
            raise Refused("the run did not meet its criteria; it cannot be accepted")
        later = [r for r in self.status.runs(run["stage"], run["mode"]) if not r.get("practice")
                 and (r.get("started_utc") or r["run_id"]) > (run.get("started_utc") or run["run_id"])
                 and not r.get("auto_ok")]
        if later:                                  # review W5: a failed later run must not disappear behind it
            raise Refused(f"a later {run['mode']} run of this stage did not pass ({later[-1]['run_id']}); an older "
                          "pass cannot stand in for it: run the stage again")
        if run["inputs_hash"] != self.status.inputs_hash(run["stage"]):
            raise Refused("the inputs changed since this run (coefficients, config or upstream results); run it again")
        crit, ok = evaluate.evaluate(s, run["metrics"], self.coeff_value, operator_checks or {})
        if not all(c["ok"] for c in crit):
            raise Refused("unticked or failed criteria: " + ", ".join(c["id"] for c in crit if not c["ok"]))
        res = self.status.accept(run, note, operator_checks or {}, crit)
        for coeff, metric in s["sets"].items():           # a calibration stage puts its measured values in the store
            self.coeffs.edit({coeff: run["metrics"][metric]}, note=f"accepted {res['result_id']}",
                             source=f"trial {run.get('trial_id') or run_id}")
        return res

    def submit_form(self, sid, values, note, operator_checks=None, override_reason=None):
        s = self.reg.stages[sid]
        if "form" not in s["modes"]:
            raise Refused(f"{sid} is not a form")
        if not note or not note.strip():
            raise Refused("a form needs a note")
        locks = self.status.locks(sid, "form", self.host_problem())
        if locks and not (override_reason and override_reason.strip()):
            raise Refused("locked: " + "; ".join(l["reason"] for l in locks) + " (an override needs a reason)")
        if locks:
            self.status.audit("override", stage=sid, mode="form", reason=override_reason,
                              locks=[l["reason"] for l in locks])
        clean = {}
        for fid, f in s["fields"].items():
            v = (values or {}).get(fid)
            if f["type"] == "check":
                clean[fid] = bool(v)
            elif f["type"] == "number":
                try:
                    clean[fid] = float(v) if v not in (None, "") else None
                except (TypeError, ValueError):
                    raise Refused(f"{fid}: a number is needed")
            else:
                clean[fid] = str(v).strip() if v not in (None, "") else None
        metrics = evaluate.form_metrics(sid, clean, self.coeff_value)
        if sid == "payload_tcp":
            metrics["bringup_fresh"] = self.status.passed("bringup_read", "arm")
        if sid == "freeze":
            metrics["spill_outcome_defined"] = self.coeff_value("spill_outcome_definition") is not None
        crit, auto_ok = evaluate.evaluate(s, metrics, self.coeff_value, operator_checks or {})
        run_id = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%d-%H%M%S-") + f"{sid}-form"
        run = {"run_id": run_id, "stage": sid, "mode": "form", "note": note, "started_utc": utc_now(),
               "ended_utc": utc_now(), "inputs_hash": self.status.inputs_hash(sid), "session": session_id(),
               "coeff_version": self.coeffs.current_version(), "metrics": metrics, "criteria": crit,
               "auto_ok": auto_ok and all(c["ok"] for c in crit), "exit_code": 0}
        rf, _ = self._open_folder(s, "form", False, self.confirmed_path, self.arm_config_path)
        run.update(run_folder=rf.name, started_local=runfolder.stamp(rf.started), ended_local=runfolder.stamp())
        rf.write_json("form.json", {"values": clean, "operator_checks": operator_checks or {}, "note": note,
                                    "override": override_reason if locks else None})
        rf.note(f"form submitted: {'all criteria met' if run['auto_ok'] else 'not passed'}")
        self.status.save_run(run)
        rf.finish(self._folder_record(s, dict(run, override=override_reason if locks else None)))
        if not run["auto_ok"]:
            self.status.journal("form", {"stage": sid, "verdict": "not passed", "note": note, "run_folder": rf.name})
            self._auto_backup(f"after form {rf.name}")
            return {"run": run, "result": None}
        res = self.status.accept(run, note, operator_checks or {}, crit)
        self._auto_backup(f"after form {rf.name}")
        return {"run": run, "result": res}

    # ------------------------------------------------------------------ coefficients
    def edit_coeffs(self, changes, note):
        try:
            snap, diff = self.coeffs.edit(changes, note)
        except CoeffError as exc:
            raise Refused(str(exc))
        stale = self.reg.stale_after(snap["changed"])
        self.status.audit("coefficients", version=snap["version"], changed=snap["changed"], note=note)
        return {"version": snap["version"], "diff": diff, "stale": stale}

    def revert_coeffs(self, to_version, note):
        try:
            snap, diff = self.coeffs.revert(to_version, note)
        except CoeffError as exc:
            raise Refused(str(exc))
        self.status.audit("revert", version=snap["version"], changed=snap["changed"], note=note)
        return {"version": snap["version"], "diff": diff}

    # ------------------------------------------------------------------ confirmed.yaml (owner only)
    def confirmed_view(self):
        confirmed, _, texts = armconfig.load(self.confirmed_path, self.arm_config_path)
        return {"values": {k: confirmed[k] for k in armconfig.CONFIRMED_KEYS}, "text": texts["confirmed.yaml"],
                "uncommitted": self.confirmed_uncommitted(), "phrase": CONFIRM_PHRASE,
                "path": str(self.confirmed_path)}

    def confirmed_uncommitted(self):
        try:
            r = subprocess.run(["git", "status", "--porcelain", "--", str(self.confirmed_path)], cwd=self.root,
                               capture_output=True, text=True, timeout=10, stdin=subprocess.DEVNULL)
        except (OSError, subprocess.TimeoutExpired):
            return None
        return bool(r.stdout.strip()) if r.returncode == 0 else None

    def change_confirmed(self, changes, phrase, note):
        """The owner's typed change to confirmed.yaml. The tools keep refusing anything above it."""
        if (phrase or "").strip() != CONFIRM_PHRASE:
            raise Refused(f"type the phrase exactly: {CONFIRM_PHRASE}")
        if not note or not note.strip():
            raise Refused("say what was seen at the previous step")
        if not changes:
            raise Refused("nothing to change")
        text = self.confirmed_path.read_text(encoding="utf-8")
        old = {}
        for key, value in changes.items():
            if key not in armconfig.CONFIRMED_KEYS:
                raise Refused(f"{key} is not a confirmed.yaml key")
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not value > 0:
                raise Refused(f"{key}: a positive number is needed")
            pat = re.compile(rf"^({re.escape(key)}:\s*)([-+0-9.eE]+)(.*)$", re.M)
            m = pat.search(text)
            if not m:
                raise Refused(f"{key} not found in {self.confirmed_path.name}")
            old[key] = float(m.group(2))
            text = text[:m.start()] + f"{m.group(1)}{float(value)!r}{m.group(3)}" + text[m.end():]
        tmp = self.confirmed_path.with_suffix(".yaml.tmp")
        tmp.write_text(text, encoding="utf-8")
        armconfig.load(tmp, self.arm_config_path)          # refuse a file the tools would refuse
        os.replace(tmp, self.confirmed_path)
        self.status.audit("confirmed_change", changed=[f"{k}: {old[k]} -> {changes[k]}" for k in changes], note=note,
                          path=str(self.confirmed_path))
        return {"old": old, "new": changes,
                "commit": f'git commit -m "confirmed.yaml: {", ".join(f"{k} {changes[k]}" for k in changes)}" '
                          f"-- {self.confirmed_path.relative_to(self.root) if self.confirmed_path.is_relative_to(self.root) else self.confirmed_path}"}
