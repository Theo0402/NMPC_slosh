# Workbench (design)

The owner's operator interface for the hardware phase (codebase plan §12). Markers as in the plans.

## 1. Shape

```
 browser (any OS)  ──HTTP, 127.0.0.1 only──▶  workbench process (python -m tools.workbench, stdlib only)
                                                 │  reads: config/workbench/stages.yaml, config/params.schema.yaml,
                                                 │         config/arm/*.yaml, data/trials/*.bin (as they grow)
                                                 │  writes: data/workbench/ (coefficients, results, audit), journal
                                                 │
                                   one stage tool at a time, its own process and process group
                                                 ▼
                    stage tool (e.g. python -m tools.elfin_py.servo_ladder --mock)  ── TCP ──▶  mock or arm
                      stdout: console text + "@@wb {json}" lines (trial path, questions, results)
                      stdin:  {"type":"answer",...} and {"type":"stop"}; EOF = stop
                      writes: data/trials/<trial>.bin  (the record; the workbench only reads it)
```

- **Standard library only:** `http.server`, `subprocess`, `json`, `threading`. The page is one HTML file with
  plain JavaScript; plots are SVG polylines drawn by the page. No framework and no plotting library: nothing here
  needs one, and the Linux machine then needs nothing beyond Python 3.12.
- **Listens on 127.0.0.1** only. On the Linux machine the owner opens it in a local browser.
- **Never in the loop.** The workbench sends no command to the arm; the stage tools do, under codebase §3.8, with
  every refusal they already have. Mock mode starts the mock controller as another process on free ports.

## 1a. Arm control tab

The first tab. Connect (dry run, mock, or the arm on the Linux machine) starts `tools/elfin_py/jog.py` (stage 1.6),
which enables the arm with the usual questions and then takes commands on the channel: `jog` (one axis, one step),
`goto` (a recorded point), `path` (current pose → start → middles → end, with a pause at each), `keep` (no motion;
restarts the idle timer), `end`. With no command for `jog_idle_timeout_s` (300 s) the session ends by itself and
disables the arm; the page counts down in the last minute. All questions come before the arm is enabled. Every move is
a linear `WayPoint` with the orientation held at the jog speed, checked against `confirmed.yaml`; steps, the box round
the start pose and the orientation are checked before sending; a refusal leaves the session running; STOP sends
`GrpStop` [UNTESTED] and ends the session with `GrpDisable`. Points are taken from the pose the tool reports and
saved as named paths in `data/workbench/points/`. The diagram is two SVG views (X–Y from above, X–Z from the side)
centred on where the session started, with the box, the points, the path and the cup's current position.

## 1b. Run folders, Past runs, Backup

Every run the workbench starts (tool runs, Arm control sessions, practice runs, forms) gets
`data/runs/<YYYY-MM-DD>_<HHMMSS>_<stage>_<mode>/`, local start time, `-practice` after the mode for practice runs.
Run and trial IDs and the journal use UTC; `summary.txt` and the Past runs tab say which is which:

| File | What |
|---|---|
| `summary.txt` | plain text: stage, mode, start and end, result, exit code, trial, coefficient version, each criterion |
| `run.json` | the run record: command line, exit code, metrics, criteria, times |
| `console.log` | the whole console, as it came; the workbench's own notes are `[wb …]` lines |
| `questions.jsonl` | each question, and each answer with who gave it (the owner, or `--assume-yes`) |
| `commands.jsonl` | each Arm control command (jog, goto, path, end) and its result, refusals included |
| `config/`, `coeffs_vNNNNNN.json` | the config files the run started with, and its complete coefficient snapshot: the version's values plus the schema default of any coefficient added after it, marked so |
| `form.json`, `mock.log` | a form's values; the mock controller's output |
| `trial.bin`, `trial.txt` | a hard link to the trial log (no extra space), and its path, size and SHA-256 |
| `COMPLETE` | written last; then every file is read-only and the workbench never writes there again |

The runner writes to the folder as the run goes on (`tools/workbench/runner.py` → `tools/workbench/runfolder.py`),
so a crash leaves everything up to that moment; the next start-up closes such a folder as INTERRUPTED. A trial with
no run folder (a tool started from a terminal, or a run from before run folders existed) gets a RECONSTRUCTED folder
from its record and trial header, once its log is closed (§6a). Both get a journal line, as every run does.

**Past runs tab**: the folders on this PC or in the backup, newest first; a folder without `COMPLETE` shows as
running or not finished; a click shows the summary, the joint plot (commanded vs state stream, read from the trial),
questions and answers, commands and the whole console, and "Open folder" opens it in Explorer (or `xdg-open`; from
WSL, Explorer); where no file manager can be started, the page shows the folder's path instead.

**Console:** the page shows the newest lines and keeps up however long the run; the whole console is in the run
folder's `console.log`.

**Backup tab** (`tools/workbench/backup.py`): folder (default `~/slosh-backup`), back up after every run (on by
default), last backup, runs and trials held, free space, "Back up now", restore. Copies only what is new, into
`<folder>/slosh-data/{runs,trials}`; never deletes or overwrites (a difference is a conflict, reported); pauses while
a run is going on; `.img` only for eval and killtest trials (codebase §8); leaves 10 GB free. Restore copies a run
folder and its trial back; a difference with the current data needs the owner's confirmation, and the current file
is then kept beside, renamed `…before-restore-<time>`.

## 2. Stages and status

`config/workbench/stages.yaml` (readable by `tools/yamlmini.py`) lists every stage: group, title, purpose,
inputs, tool, the coefficients the owner can adjust, pass criteria, output, plan section, modes, prerequisites,
and whether the tool is built. `docs/stages.md` is generated from it (`python -m tools.workbench.gen_docs`; a test
fails if it is out of date).

- **Modes:** `dry-run`, `mock`, `arm` for tools that talk to the arm; `form` for a checklist answered on the page;
  `analysis` for a script on recorded data. `arm` is offered only when `link.arm_host_problem()` returns nothing.
- **Status per (stage, mode):** not run / passed / failed / stale.
  - *passed*: an accepted result exists, and its inputs hash still matches.
  - *failed*: the latest run's criteria failed and nothing was accepted after it.
  - *stale*: an accepted result whose inputs changed since (below).
  - In `dry-run` and `mock` the status comes from the latest run that is not a practice run, with no accept;
    accepted results never count there, and a practice run cannot be accepted (pre-hardware review S6).
- **Locks:** a stage is runnable on the arm when every prerequisite has passed on the arm (a `form` or `analysis`
  stage counts in its own mode), and the stage itself has passed in `dry-run` and `mock` with the same inputs.
  A dry run or mock run is allowed when the prerequisites have passed in any mode. The owner may override a lock:
  a typed reason, logged to the audit file and the journal, and shown on the stage.
- **Inputs hash:** at accept time the workbench hashes (a) the values of every coefficient owned by the stage or
  any stage upstream of it, and (b) the accepted result IDs of the upstream stages. A changed hash makes the
  result stale. Upstream = the transitive prerequisites.

## 3. Runs, criteria and accept

1. **Run:** the workbench builds the command line from the stage's tool and the mode (mock: the mock's ports;
   all: `--data-dir`, `--note`), starts it with `SLOSH_WORKBENCH=1`, and reads its stdout line by line.
2. **Questions** (`ask`) appear on the page; the owner answers. In dry run and mock the owner may tick "answer yes",
   which passes `--assume-yes` (the tools refuse that flag with `--arm`).
3. **Live plots** read the trial's `.bin` incrementally from the path the tool announces: joint commands
   (`arm_cmd`) against the state stream (`robot_state`). Later stages add the camera image with the detected edge,
   η(θ) and the ring-down fit from their own records.
4. **Results:** when the tool exits, its evaluator reads the trial log (the `summary` event, the bring-up block, the
   refusals and the exit code) into named metrics, and each criterion of the stage is checked: `auto` criteria
   against a number or a coefficient, `operator` criteria by a tick box on the page.
5. **Accept** (only if every criterion passed, the run is not a practice run, and no later run of the stage in the
   same mode failed): an immutable record `data/workbench/results/<stage>/<n>.json`
   with the trial ID, metrics, criteria, mode, inputs hash and the owner's note; calibration stages also write their
   measured values into the coefficient store, with the trial ID as the source.

## 4. Coefficients

- **Definitions:** `config/params.schema.yaml`, section `tunables:`: per name, the owning stage, unit, range,
  marker, default and the plan section the default comes from.
- **Values:** `data/workbench/coeffs/v000001.json`, `v000002.json`, …: each a full snapshot
  `{values: {name: {value, source, note}}, parent, note, changed, created_utc}`; `CURRENT` names the live one.
  Version 1 is the plan defaults.
- **Edit:** value(s) plus a note → range check → new version; the page shows the diff against the parent.
  **Revert:** one click, a new version whose values equal the chosen old one (nothing is rewritten).
- **Who reads them:** tools started by the workbench get the run's complete snapshot (the version, with schema
  defaults for any coefficient added after it) from the run folder, and it goes into their trial header (§12); the
  future RT process reads the snapshot at trial start only, like the params file (codebase §11).
- **Read-only here:** `confirmed.yaml` (see §5), and values whose home is a file the bring-up tools read
  (`e03pro.yaml`, `camera.yaml`, `cup/r40.yaml`); those are shown with the file as their source and edited in the file.

## 5. `confirmed.yaml`

Shown read-only. A change needs the owner to type the new value(s) and the phrase
`I confirm the new limits` on the page, plus a note of what was seen at the previous step. The workbench then
writes the file (only the changed lines), appends the journal line and the audit event, and shows a warning until
`git` reports the file committed: each change is the owner's own commit (codebase §3.8 item 5). The tools refuse
anything above the file regardless. No agent edits it; the tests use a temporary copy.

## 6. Stop, and a workbench that dies

- **Stop button** (on every page) → `{"type":"stop"}` on the tool's stdin → the tool's channel thread interrupts the
  main thread exactly as Ctrl-C does (POSIX: `SIGINT` to the main thread; Windows: `signal.raise_signal`), before
  it writes or logs anything about the stop, so a blocked pipe or a slow disk cannot hold it back. The workbench
  sends the stop before it logs it.
  The servo ladder then decelerates by streaming: while it streams, the interrupt only sets a flag that the loop
  acts on between pushes, so the ramp starts from the last two samples actually sent. Every tool that enabled the
  arm runs the exit sequence: `GrpStop` and a wait for rest if a move may be under way, then `GrpDisable`
  (`link.Arm.exit_sequence`; pre-hardware review S2, S3).
- **Workbench crash:** the tool sees EOF on stdin and does the same, then answers "no" to any further question.
  Its prints can no longer fail once nobody reads its output (`cli.SafeStream`, review S5). The tool runs in its own
  process group, so the crash itself cannot kill it mid-stream. It is a software stop; the e-stop stays within reach
  (`GrpStop` during streaming is **[UNTESTED]**).
- **The workbench keeps reading** the tool's output while the tool lives, whatever fails on its side (a line it
  cannot handle, a run folder on a full disk), so a tool never blocks on a full pipe (review S7).
- **A failed trial-log write** is a stop: the tool stops as above, sends `GrpStop`/`GrpDisable` unlogged if it
  must, and ends with "trial log incomplete" (review S8).
- **Questions with the arm enabled** time out (pose recorder: 300 s **[proposal]**); the page drops a question the
  tool has withdrawn. The servo ladder asks "Start streaming?" before `GrpEnable` (review S9).
- **A reply that never comes** (after its 2 s time-out and 2 s more **[proposal]**) puts the link out of step: the
  tool stops, and only `GrpStop`, the wait for rest and `GrpDisable` go out, their replies matched by name (one
  message at a time, PROTO p.1; review S11).
- **Termination signals:** SIGTERM and SIGHUP (the terminal closed, `kill`, a shutdown) do what Ctrl-C does, and a
  tool started with SIGINT ignored gets Python's handler back, so the stop button works (review S13).
- **Arm runs use the owner's files only:** `config/arm/confirmed.yaml` and `config/arm/e03pro.yaml`, refused
  otherwise by the workbench and by the tools; `--mock` reaches loopback addresses only (review S14).
- **Only this machine's page may use the API** (review S15): the Host header must name the workbench (no DNS
  rebinding), a browser's Origin must be the page itself, requests marked cross-site are refused, and every
  `/api/` request carries `X-Slosh-Workbench: 1` (POSTs as JSON), which another site cannot send without a CORS
  preflight that the server never answers.
- **Answers and commands carry the run's id** and are refused for any other run, so a page left open on an earlier
  run cannot answer or move for a newer one; the stop button always stops the current run. A jog or point command
  marks the session busy as soon as it is passed on, so a second click cannot send a second move. The page keeps
  polling after a failed request and says "connection lost" until it is back (review S16).
- **A run counts as running until it is saved** (the mock stopped, the run record and folder written), so the next
  run's locks are checked against it (review S19).

## 6a. Keeping the records (pre-hardware review D3–D8)

- **One workbench per data folder:** `data/workbench/workbench.lock` names the workbench using it (process, machine,
  a heartbeat renewed every 10 s **[proposal]**). A second one, on another port or in WSL beside Windows, refuses to
  start; a lock not renewed for 60 s **[proposal]** was left by a workbench that died and is taken over (D6).
- **The backup folder is kept per machine** and checked on every use, so a Windows path read under WSL can no longer
  send backups into the repository (D3).
- **The journal is appended to**, one line at a time, never rewritten, in the file's own line endings (D5).
- **Run records are written whole** (a temporary file, then a rename). A run folder or run record that cannot be
  read is reported and skipped: it no longer stops the workbench from starting or the page from loading (D7).
- **Every trial log ends with a "trial log closed" record.** At start-up a trial without a run folder gets one only
  once its log is closed, or after a day without change **[proposal]**: a tool started from a terminal may be
  waiting at a prompt. Such runs are labelled "started outside the workbench" (D8).

## 6b. Results that must not mislead (pre-hardware review W1, W4–W10)

- **Staleness follows the whole chain** of prerequisites, as the coefficients already did (W4).
- **An older pass cannot be accepted after a newer run in the same mode failed** (W5).
- **Every run gets a complete coefficient snapshot** in its run folder: the version's values plus the schema default
  of any coefficient added to the schema after the version was saved, marked so; the trial header carries it (W7).
- **Stage 1.6** passes only if the trial log shows no Fail reply, no refused command and an acknowledged
  `GrpDisable`, besides the exit code (W9).
- **The gate (4.4)** has `gate_input.json` among its inputs, so an edit makes it stale; it names the front-end it
  judges, routes a failed dead band by its cause (pinning or edge step, the owner's call), and passes with
  `front_end: cnn` (gate memo v2) only once stage 4.5 has passed (W1).

## 7. Files

| Path | What |
|---|---|
| `tools/workbench/` | `bench.py` (the workbench: runs, accepts, forms, start-up recovery), `stages.py` (registry), `coeffs.py` (versions), `status.py` (results, staleness, audit, journal), `runner.py` (processes and channel), `runfolder.py` (run folders), `backup.py` (local backup), `datalock.py` (one workbench per data folder), `evaluate.py` (metrics per tool), `livelog.py` (incremental reader), `server.py` (HTTP API), `static/index.html`, `gen_docs.py` |
| `tools/elfin_py/channel.py` | the tool side of the channel (stdlib), used by `cli.py` only when `SLOSH_WORKBENCH=1` |
| `config/workbench/stages.yaml` | the registry |
| `data/workbench/` | `coeffs/`, `results/`, `runs/` (run records), `points/`, `practice/`, `audit.jsonl`, `backup_settings.json`, `gate_input.json`, `workbench.lock` |
| `data/runs/` | one folder per run (§1b) |

## 8. What is deliberately left out

Authentication (local only), several tools at once, editing files other than through the paths above, and any
plotting that needs NumPy in the workbench process: analysis stages run as their own scripts and write results the
page draws.
