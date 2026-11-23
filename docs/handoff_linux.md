# Handoff: continuing on the Linux machine (2026-11-23)

This file carries the context of the long Windows conversation (2026-11-12 to 2026-11-23) to a new Claude Code
session on the Linux machine. Read it with `CLAUDE.md` (the standing rules), then `docs/plans/README.md`.
Where this file and a newer `docs/journal.md` or `docs/open_questions.md` disagree, the newer files win.

## 1. How to start the new session

```bash
cd ~/NMPC_slosh                       # wherever you cloned https://github.com/Theo0402/NMPC_slosh
git pull
python3 -m venv .venv && . .venv/bin/activate
pip install -r requirements-dev.txt -r requirements-analysis.txt
python -m pytest                      # expect all pass, a few skipped (2026-11-18: 222 passed in WSL, 219 + 3 skipped on Windows)
claude                                # Claude Code loads CLAUDE.md by itself
```

First message to the new session, for example:

> Read docs/handoff_linux.md and CLAUDE.md. We are on the Linux machine now. Start with section 6 (first steps on Linux).

The folder name does not matter: no code depends on it being called `slosh`.

## 2. The project in one paragraph

A Han's Robot Elfin E03-Pro arm carries an open, partly filled cup of water (40 mm inner radius) along fast paths
without spilling. A Hikrobot MV-CS016-10UM camera with a Kowa LM4NCL 3.5 mm lens, on the Slim Cup Holder Rev F,
looks straight down into the cup and measures the wall contact line all the way round; the controller closes the
loop on it. Arm interface: the HansRobot V5 TCP protocol (commands on port 10003, state stream on 10004), joint
streaming with `StartServo` + `PushServoJ`. Step 1 (residual-slosh corrector) targets IROS 2027, step 2 (NMPC) ICRA
2028 / RA-L. Plans: `docs/plans/research_plan_v3_camera.md` (what and why; §0 = owner decisions) and
`docs/plans/codebase_plan_v3_camera.md` (how; §3.8 = operating the arm, §11 = build order, §12 = the workbench).

Owner amendment (hardware phase): a learned contact-line front-end (a small 1D CNN per ray) is allowed **only** as the
fallback after the kill-test-2 gate fails. Order: classical/clear → CNN/clear → classical/dyed. Design in
`docs/design/cnn_fallback.md`; **no CNN code until the gate memo says fail**.

## 3. Machines

| Machine | Role |
|---|---|
| Windows dev PC | Where everything so far was written. Python 3.11.9 venv `slosh\.venv`. Only ~83 GB free (2026-11-18), so camera data (≈3 GB per 30 s trial) cannot live there. |
| WSL2 Ubuntu 24.04 on that PC | Linux test bed (Python 3.12, venv `~/.venvs/slosh`). `--arm` is refused from WSL. |
| **The Linux machine** | All arm runs, the real-time process, timing, and the camera data (with the second-disk backup, codebase §8). Python 3.12 (owner's choice). Its distribution and PREEMPT_RT kernel are still unchecked (open question F14). |

`--arm` is refused on any OS other than Linux and from WSL (`tools/elfin_py/link.py`, `arm_host_problem`).

## 4. What is built

All in commit 434c0c7 "Slosh software before the hardware sessions" (95 files), pushed to `origin/master`.

- **Arm side** (`tools/elfin_py/`, standard library only): the command table `protocol.py` (the protocol reference;
  `elfin_min.py` stays unchanged as the bring-up tool), the logged link `link.py`, and the tools: `bringup_read`
  (stage 1.1), `state_stream` (1.2), `move_test` (1.3), `servo_ladder` (1.4), `pose_recorder` (1.5), `jog` (1.6, Arm
  control), `motion.py`, `triallog.py`, `channel.py` (the tools' side of the workbench's stop/question/command channel),
  `cli.py`.
- **Mock controller** `tools/elfin_mock/` (`--mirror-config` reports `config/arm/e03pro.yaml`'s limits and frames).
- **Log schema v1**: `schema/fields.yaml` → `python -m schema.gen_py` → `analysis/logread/slog_v1.py` and
  `docs/data_dictionary.md`. Not frozen yet (open item D9).
- **Workbench** `tools/workbench/` (`python -m tools.workbench`, stdlib, 127.0.0.1:8765): one page per stage from
  `config/workbench/stages.yaml` (`docs/stages.md` is generated from it), versioned coefficients with notes, diff and
  revert, the Arm control tab, run folders `data/runs/<date>_<time>_<stage>_<mode>/` (read-only once complete), Past
  runs and Backup tabs, one workbench per data folder (`data/workbench/workbench.lock`).
- **Analysis**: `analysis/identify/ringdown.py` (ring-down fit, dispersion check), `analysis/gate_memo.py` (the
  kill-test-2 gate memo). The fill level is measured and entered by the owner.
- **Host and backup**: `tools/host_check.py` (stage 0.1), `tools/backup.sh` (stage 9.2; refuses until `BACKUP_DISK`
  is set).
- **Stages**: 0.1–1.6, 2.1, 3.5, 4.4, 8.0, 9.2, 9.3 are built; the camera, FK, tracker, RT process and evaluation
  stages are planned (see `docs/stages.md`). No stage has passed on the arm yet. No arm run has happened.

## 5. The pre-hardware review (2026-11-18) and what changed

A read-only review found 44 items. The owner approved fixes in batches; 39 are fixed and tested.

**Safety S1–S20 (all fixed).** Highlights:
- Servo ladder: a stop request only sets a flag, acted on between pushes; every push is step-checked against
  `confirmed.yaml`; the deceleration ramp starts from the last two samples actually sent; `ReadRobotState` is polled
  between pushes (`--state-poll-s`, 0.1 s; 0 = off) while the e-stop and light-curtain flags are unknown (Q18); a
  joint-speed check (+10 %, Q18); a late push skips only within `confirmed.yaml`, otherwise decelerates (Q14).
- Exit sequence: drain, then `GrpStop` and a wait of up to 10 s for rest when a move may be under way, then
  `GrpDisable` (Q15; written into codebase §3.8 item 8). `GrpStop` is [UNTESTED] on this arm; what `GrpDisable` does to a
  moving arm is [MISSING].
- Link: replies matched by command name in out-of-step mode; a reply still owed after its 2 s time-out is waited for
  2 s more, then only stop commands go out (Q17); `CommandFailed` takes the Fail code from the reply's first field;
  reply time measured on the wire minus log-write time.
- Logging: the arm block is logged as event kind 5 before `GrpEnable`; a failed log write stops the run
  (`LogIncomplete`); every trial log ends with a "trial log closed" record.
- Process: SIGTERM and SIGHUP act like Ctrl-C; stdout/stderr writes can't fail (SafeStream); questions asked with the
  arm enabled end the run after 300 s (Q16); the ladder asks before enabling and refuses if joints moved > 1°.
- Workbench: local API guard (Host, Origin, Sec-Fetch-Site, the `X-Slosh-Workbench` header, JSON POSTs); `run_id` on
  answers and commands; the session is busy from the click so a second click can't send a second move; the reader
  thread never stops reading.

**Data loss D3–D8 (fixed).** Backup folder kept per machine; `backup.sh` keeps replaced copies; journal lines
appended, never rewritten; the data-folder lock with heartbeat (Q19); atomic writes for run records; trial logs end with
the closed marker.

**Wrong results W1, W4–W10 (fixed).** The gate memo routes a failed dead band by the owner's `deadband_cause`
(pinning → R1 pivot, edge_step → CNN); gate memo v2 (front_end cnn) counts only after 4.5; staleness follows the whole
prerequisite chain; an older pass can't be accepted after a newer failure; each run gets a complete coefficient
snapshot.

**Inconvenience I1–I5 (fixed).** Console view past 3,000 lines; Open folder falls back to showing the path; time bases
named; malformed jog commands refused; stale docs fixed (journal: ~1.6 MB/h, not 0.8).

**New tests**: `tests/elfin/test_safety.py`, `tests/workbench/test_safety_workbench.py`, `test_data_loss.py`,
`test_wrong_results.py`, `test_inconvenience.py`.

Defaults the owner has not yet confirmed are written down as **[proposal]** in `docs/open_questions.md` Q14–Q19.

## 6. First steps on Linux

1. Set up and run the tests (section 1). Report any failure before anything else.
2. **Stage 0.1**: `python -m tools.host_check` (reports only, changes nothing). Answer F14 (distribution, PREEMPT_RT,
   Python 3.12) in `docs/open_questions.md`.
3. Decide where `data/` lives: on the data disk, not the system disk (camera data is ≈3 GB per 30 s trial).
4. Start the workbench (`python -m tools.workbench`) and set the **Backup** tab's folder for this machine (default
   `~/slosh-backup`; the setting is kept per machine). For `backup.sh` set `BACKUP_DISK` once the second disk exists (H9).
5. Rehearse stages 1.1–1.6 in **dry run and mock** on this machine first. The runs in the journal from 2026-11-12
   were Windows dry-run/mock runs from before the review fixes; `data/` from Windows was not copied, so this machine
   starts with no runs.
6. Then the owner runs the week-1 bring-up on the arm (README table; codebase §3.8, §11): `bringup_read` → paste its
   limits block into `config/arm/e03pro.yaml` → `state_stream` → `move_test` → `servo_ladder` → `pose_recorder`.
   Only the owner edits `config/arm/confirmed.yaml` and starts `--arm` runs.

## 7. Open items (owner decisions)

From the review:
- **D2**: the second backup disk and an offsite target (H9). Should the local backup go to a different disk from
  `data/` by default?
- **D9**: freeze log schema v1 (`schema/fields.yaml`; no frozen copy exists under `schema/versions/`) before the first
  arm session.
- **W2** (review question 6): `research_plan_v3_camera.md` ~line 422 says the shift is "1 %". Is that with the
  predictor or without? Without it, the review computed ~12 % at ζ_target 0.3.
- **W3** (review question 5): `config/params.schema.yaml` ~line 687 limits ζ_target to a range that blocks the
  ablation values in research plan ~lines 398 and 535. Widen to [0.1, 0.5], or change the ablation values?
- Review question 8, unanswered: on lab days, will tools also run from a terminal (README bring-up table), or only
  from the workbench? It decides how much S13 and D8 matter.
- **Gate memo and dyed water**: the gate memo can't pass on dyed water, although research §7.1 allows dyed water as
  the last resort. Not fixed yet.
- **The 12 proposed functions** from the review (not built; each needs the owner's go-ahead). The top five:
  1. *Rehearse, then run*: the arm button for a stage stays off until a dry run and a mock run of the same tool,
     same inputs, passed the same day; the rehearsal's plan (commands, peak speeds) shown next to the button.
  2. *Arm-state banner and recovery guide*: after any tool ends, read its trial log (GrpEnable sent? GrpDisable OK?
     move under way? last flags, Fail codes) and show a red banner with recovery steps if the disable isn't confirmed.
  3. *Servo-ladder assistant*: a panel with the ladder's steps (0.2 s down to 0.01 s) and each step's runs and verdicts.
  4. *Start-of-day and end-of-day checks*: one page each.
  5. *Fault drills on the mock, and stop-path tests through real pipes*: switches in the mock that inject faults.
  The other seven are in the Windows conversation's review report; ask the owner if they are needed.

Other open questions: `docs/open_questions.md` (H1–H11 missing hardware, Q1–Q19 decisions, F1–F15 facts the
sources lack, e.g. F1 the E03-Pro kinematic model, F8 the Fail code meanings).

## 8. Working between machines

- Code moves by git only: commit and push on one machine, `git pull` on the other. Remote:
  `https://github.com/Theo0402/NMPC_slosh`, branch `master`. Line endings are normalised to LF (`.gitattributes`).
- `data/` never goes into git. Run data moves by the workbench's Backup tab or `tools/backup.sh`.
- A running workbench keeps the code it started with: restart it after a `git pull`.
- Claude's memory from the Windows conversation stays on Windows; everything it held that matters is in this file,
  `CLAUDE.md`, `docs/journal.md` and `docs/open_questions.md`.
