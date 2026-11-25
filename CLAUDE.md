# Working rules for this repository (slosh)

One-person research project: an Elfin E03-Pro arm carries an open cup of water fast without spilling,
with a camera in the loop. The owner decides; Claude assists. Read `docs/handoff_linux.md` first in a new
session: it says where the work stands and what comes next.

## Rules the owner set (always in force)

- **Robot facts come only from the Elfin knowledge files** `docs/plans/elfin_*.md`, cited by file and
  manual page, e.g. "(elfin_commands.md, PROTO p.172)". Never invent a command, parameter, unit or limit.
  If the files don't cover it, write **[MISSING]** and suggest a test. Do not use the PDFs as sources.
- **Keep the plans' markers**: [unverified], [target], [MISSING], [UNTESTED], [UNCLEAR], [E03], [proposal].
- **Don't reargue research plan §0** (`docs/plans/research_plan_v3_camera.md`). Those are the owner's decisions.
- **Anything that talks to the arm follows codebase plan §3.8** (`docs/plans/codebase_plan_v3_camera.md`):
  one message at a time; raise on `Fail`; checks before motion; `GrpDisable` in `finally`; every motion tool
  runs `--dry-run`, then `--mock`, then `--arm` with the owner present.
- **No speed or stream-rate increase without asking the owner.** When a number turns out different from
  what was promised, say so plainly.
- **Only the owner edits `config/arm/confirmed.yaml`.**
- **Arm runs only at the owner's word (owner's change, 2026-11-25).** Claude may run arm commands on the Linux
  machine through `python -m tools.wb` (the workbench's API) when the owner asks in chat and is at the e-stop.
  Every question a tool asks goes to the owner in chat; Claude passes on the owner's answer and never answers one
  itself (`--yes` is refused on the arm). Never get round the workbench's checks, locks or refusals. Dry runs, mock
  runs and tests as before.
- **`data/` is never tracked in git** (`.gitignore`). Trial files are never renamed or edited.
- Which file wins when two disagree: `docs/plans/README.md` ("Which file wins").
- Record decisions and fixes in `docs/journal.md` (one line per day, week numbers only) and owner questions
  in `docs/open_questions.md`.
- Commit or push only when the owner asks. Never put the owner's personal details in requests to outside
  services.

## Commands

```bash
python3 -m venv .venv && . .venv/bin/activate
pip install -r requirements-dev.txt -r requirements-analysis.txt
python -m pytest                      # against the mock; no arm needed
python -m tools.workbench             # http://127.0.0.1:8765
python -m tools.host_check            # stage 0.1
```

Run every command from this directory. Restart the workbench after code changes.
