# slosh

Software for fast transport of an open cup with the camera in the loop. The plans, the
notation and the rules are in [`docs/plans/README.md`](docs/plans/README.md); read that first.
Daily notes: [`docs/journal.md`](docs/journal.md).

## Set up

```bash
python3 -m venv .venv && . .venv/bin/activate        # Python 3.12 (the Windows dev venv is 3.11.9); Windows: .venv\Scripts\activate
pip install -r requirements-dev.txt                   # pytest, PyYAML: development only
python -m pytest                                      # tests run against the mock; no arm needed
```

The tools that talk to the arm, and the workbench, use the standard library only, so on the Linux
machine they need nothing but Python 3.12, the owner's choice (3.8 or later works for elfin_min.py). The
Windows development venv is still Python 3.11.9; the tests pass on both (WSL runs 3.12). The identification
fits need NumPy: `pip install -r requirements-analysis.txt`. Run every command from this directory.

## The workbench (codebase plan §12)

```bash
python -m tools.workbench            # then open http://127.0.0.1:8765
```

From the Windows PC, double-click `tools\start_workbench.bat`: it opens the SSH tunnel, starts the workbench on the
Linux machine (`ssh slosh-linux`, folder `~/Desktop/NMPC_slosh`; set `SLOSH_HOST` / `SLOSH_DIR` for others) and
opens the page. Closing its window stops the workbench; a tool still running gets its stop first (SIGHUP acts like
Ctrl-C).

Without a browser, `python -m tools.wb` does the same through the workbench's API (every check, lock, question and run
folder stays the workbench's):

```bash
python -m tools.wb up                                   # start the workbench in the background
python -m tools.wb run bringup_read arm --override "..." # run a stage; questions are printed with their numbers
python -m tools.wb answer 1 yes                         # the operator's answer (never given by the client itself)
python -m tools.wb run arm_control arm                  # then: jog z -5 | goto X Y Z | pose | corner | ws-save --note ... | end
python -m tools.wb stop                                 # the STOP button
python -m tools.wb down
```

The first tab, **Arm control**, connects to the arm (dry run, mock, or the real arm on the Linux machine), moves the
cup slowly with arrow buttons (straight lines, cup orientation held, 10 mm/s by default), sets start, middle and
end points where the arm is, saves them as a path, and runs the path with a pause at each point. A diagram shows
the cup from above and from the side. The other tabs give one page for every stage of the hardware phase, in order (`docs/stages.md`, generated from
`config/workbench/stages.yaml`): run a stage in dry run, mock or (on the Linux machine only) on the
arm, answer its questions, watch the live plot, see the results against the pass criteria, accept.
A stage's arm run stays locked until the stages it requires have passed on the arm and the stage
itself has passed in dry run and mock. Every coefficient is on the Coefficients tab: each edit needs
a note and makes a new version, with a diff and one-click revert. `confirmed.yaml` is read-only there
unless the owner types the confirmation phrase. The STOP button is a software stop (the tool
decelerates and disables); keep the e-stop within reach. If the workbench dies, the running tool
stops itself the same way. Every run gets its own folder in `data/runs/` (summary, whole console, questions and answers, commands, config,
coefficient version, the trial log linked), shown on the **Past runs** tab; the **Backup** tab copies new run folders
and trial logs to `~/slosh-backup` (default) after every run and on request, and restores from there.
Workbench data lives in `data/workbench/`; it adds one line per run to the
journal's "Runs" section. One workbench at a time may use a data folder (`data/workbench/workbench.lock`): a
second one, on another port or in WSL beside Windows, refuses to start. Restart the workbench after the code
changes, since a running one keeps the code it started with.

## Week-1 bring-up order (codebase plan §3.8, §11)

Each tool runs first with `--dry-run`, then with `--mock` (start `python -m tools.elfin_mock` in a
second terminal), then with `--arm` from the Linux machine with the owner present. `--arm` is
refused on any other OS. Every run writes a trial to `data/trials/`.

| Step | Command | Moves? |
|---|---|---|
| 1 | `python -m tools.elfin_py.bringup_read --arm --controller-version "<About screen>"` | no; prints the limits block for `config/arm/e03pro.yaml` |
| 2 | `python -m tools.elfin_py.state_stream --arm` | no; packet rate, format, keys |
| 3 | `python -m tools.elfin_py.move_test --arm` | J6 +10°, then 20 mm up; tests `20.0000` |
| 4 | `python -m tools.elfin_py.servo_ladder --arm` | J6 ±5° sine, streamed at `confirmed.yaml`'s period |
| 5 | `python -m tools.elfin_py.pose_recorder --arm` | the poses in `config/arm/fk_poses.yaml` |

Steps 3–5 refuse to enable until step 1's limits are in `e03pro.yaml`, the arm is powered from
the pendant, and the operator confirms payload and TCP. Only the owner edits
`config/arm/confirmed.yaml`.

## Layout so far

- `docs/plans/`: the plans and the Elfin knowledge files
- `config/`: arm, cup, camera, parameter schema
- `schema/fields.yaml` → `python -m schema.gen_py` → `analysis/logread/slog_v1.py`, `docs/data_dictionary.md`
- `tools/elfin_py/`: `elfin_min.py` (unchanged), `protocol.py` (the command table), the logged link, the tools
- `tools/elfin_mock/`: the mock controller (`--mirror-config` reports `e03pro.yaml`'s limits and frames for rehearsals)
- `tools/workbench/`: the operator interface; `tools/elfin_py/channel.py` is the tools' side of its stop/question channel
- `tools/host_check.py`, `tools/backup.sh`: stages 0.1 and 9.2
- `analysis/identify/ringdown.py` (ring-down fit, dispersion check), `analysis/gate_memo.py` (the kill-test-2
  gate memo); the fill level is measured and entered by the owner (no fill-from-weight)
- `docs/design/`: the CNN fallback (built only if the gate fails) and the workbench; `docs/open_questions.md`
- `tests/`: `elfin/`, `schema/`, `workbench/`, `analysis/`
