# Codebase structure and build plan — v3 (camera in the loop)

Companion to `research_plan_v3_camera.md`. One person, C++ and Python. Week numbers only: the step-1 submission is week 25 (IROS 2027; confirm the deadline on the official site). `README.md` defines the frames, symbols and units used here and says which document wins when two disagree.

Scope: the decisions that are cheap in week 1 and expensive in week 20. Everything not listed here is deliberately unplanned (§10).

**Markers**, as in the research plan: **[unverified]** not checked or not yet measured; **[target]** an expected outcome; **[MISSING]** no source states it, and the text says how to find out; **[UNTESTED]** and **[UNCLEAR]** as in the lab's Elfin knowledge files; **[E03]** found in the lab's earlier records of this same arm (serial MH217340001), likely to hold but to be confirmed. Elfin facts cite the knowledge file and the manual page it cites, e.g. (elfin_commands.md, PROTO p.172).

**Changes from v2** (each is argued at the point it appears):
1. **Camera in the loop.** `CameraFrontend` is the primary `IMeasurementSource`. `FTFrontend` is parked: a declaration, no code. The EtherCAT/SOEM thread, the IMU driver, the rigid-wrench subtraction and libfranka are gone (§2, §6).
2. **Elfin E03-Pro over the V5 TCP protocol.** A C++ port of the lab's `elfin_min.py`, a local mock controller, and `IArmLink` as the interface for the arm-interface pivot (§2, §6). The lab's rules for operating the arm are written down as code requirements (§3.8).
3. **Log.** New streams for frames, image features, measurements, commanded kinematics, the arm link and the raw wire traffic; the frame IDs consumed per tick in `intake`; the calibration ID in the header; ROI images in a second append-only file (§5).
4. **Threads.** Six threads instead of two, and no clock domain that the loop does not stamp (§6).
5. **Replay at two levels**: from the logged features, and from the stored images through the same image stage (§7).
6. **R-D's named pivots** are now the measurement method and the arm interface (§1).
7. **Tests, build order and slip branches** rewritten (§9, §11). The logging rules are renamed LG1–LG4, so that they no longer clash with the simulation layers L0–L2; the milestones are MS1–MS6 for the same reason.

**Amendments in the hardware phase** (owner): the learned contact-line front-end is allowed as the fallback (research plan §0 amendment, §2.3, §7.1; here §2, §5, §9, §10), and the owner asked for one operator interface for all the stages, the workbench (§12).

Unchanged in substance: R-A to R-C; the coding rules D1–D6, RT1–RT5 (with RT3's error sources replaced), F2–F3, N1–N4, LG1–LG3 (v2's L1–L3), A1–A4 and P1–P4; the single CasADi source of the model layer; data governance; and the "not built" list.

---

## 1. Design rules

Four rules generate most of the structure below. When a later decision is unclear, decide it by asking which rule it serves.

**R-A. The log records what happened; it does not record hypotheses.**
- Anything unrecoverable must be in the log: raw images and frame metadata, robot state, every message to and from the arm, which frame each tick consumed, every branch the code took.
- Derived quantities *are* logged when they are witnesses of what the online system did: `meas`, `est`, `pred`, `corr` and `ref` are all recomputable and all logged, because the determinism test compares against them and because forensics on a bad trial needs them.
- What stays out is anything downstream of a constant still under test. No `crest_mm`: κ is a derived constant that kill test 5 may overturn, so a logged crest is a hypothesis frozen into an archive. Log `ŝ` and recompute. Same for modal energy and anything divided by ω₀.
- **Applied to the camera:** the front-end's output stops at wall elevation. `meas` holds η per sector and its harmonics, which depend only on geometry and the calibration, itself a record with an ID. Anything that needs κ (`z` → `s`, the crest) happens downstream; where it is logged (`est`, `safety`), it is a witness.
- Schema changes are additive and versioned; old trials stay readable forever.

**R-B. One implementation of each model.** The model that runs at the 1 kHz tick, the model the simulator integrates, and (step 2) the model acados compiles are generated from one symbolic source. The harness drives the real C++ through pybind; there is no Python twin of the tracker. In v3 the rule also covers two new things:
- **The lens model** has one implementation, OpenCV's, used offline. Calibration fits it, the ray table is computed from it, and the L1 camera model projects with it. The C++ never evaluates a lens model; it reads the ray table.
- **The learned front-end** (only if the classical detector fails the kill-test-2 gate) has one implementation that runs online, in replay and in every evaluation: the C++ inference in `rt/core`, reached from Python through `rt/bindings`. PyTorch exists only to train it, in `analysis/cnn/`; a parity test holds the two to 1e-5 (§9), as the golden vectors hold the generated C to the CasADi source.
- **The protocol** has one reference, the command table (`tools/elfin_py/protocol.py`). It types every parameter from the PROTO formats in elfin_commands.md (`n…` integer, `d…` float with 4 decimals, `s…`/`str…` string) and refuses NaN and infinity. The C++ client is tested byte-for-byte against the strings it produces (§9). `elfin_min.py` stays unchanged as the bring-up client: the wrapper formats the parameters into strings before calling `cmd()`, which sends strings as they are. Whether the controller accepts `20.0000` where the manual's examples show `20` is **[UNTESTED]**; example 2's first move tests it, and a refusal becomes a per-parameter override in the table, with no other code change.

**R-C. The offline path and the online path run the same code.** Not equivalent code: the same compiled objects. The estimator replayed from a recorded trial must reproduce what it produced live, and so must the image stage replayed from stored images. A post-hoc filter change then costs a script, not a repeat of the 780-trial grid; a post-hoc image-stage change costs a re-run over stored images.

**R-D. Pivots are config or one file.** The named pivots:
1. the measurement method (research plan R1): contact line → refraction of a printed pattern on the cup bottom, or floating markers. Clear → dyed water is config;
2. the arm interface (R3): path A (TCP servo streaming) → path B (the vendor's EtherCAT driver);
3. the parked wrist F/T sensor.

None of them may change an existing log field or the model layer. A pivot may add one class behind an existing interface and, if it needs one, a raw stream or a feature type (additive and versioned, R-A). Test any structural decision against all three.

---

## 2. Repository layout

```
slosh/
  schema/                      log layer is GENERATED, not hand-written
    fields.yaml                name, type, unit, frame, since_version
    gen_cpp.py  gen_py.py  gen_docs.py
    versions/                  frozen fields.yaml per schema version

  config/
    arm/e03pro.yaml            address, ports, servo settings in use; limits read back at bring-up
                               (with the record ID of the read)
    arm/confirmed.yaml         owner-confirmed speeds and stream rate; only the owner edits it (§3.8)
    cup/r40.yaml               inner radius and height, wall thickness, yoke
    camera.yaml                pixel format, ROI, exposure, gain, frame rate, trigger mode;
                               lens f-number and focus state (operator-entered)
    paths.yaml  gains.yaml  experiment.yaml
    params.schema.yaml         one definition of the model parameter set;
                               generates C++ struct + Python dataclass + header block

  models/                      SINGLE SOURCE OF TRUTH (Python + CasADi)
    frames.py                  frame graph and transforms; naming enforced here
    model_a.py                 LTV system matrices, nonlinear form, crest; wrench (parked, not generated)
    measurement.py             camera measurement model: H, D_z, mode shape and gradient, noise form
    kinematics.py              E03-Pro FK and Jacobian (Pinocchio-CasADi) from the checked model
    model_b.py                 step 2, empty until step 2's month 1
    mapping.py                 chi, kappa, R/L; pendulum <-> modal
    codegen.py                 emits C into rt/core/gen/
    golden/
      vectors.json             data, not code: diffable and reviewable
      run_golden.py

  rt/
    core/                      PURE: no I/O, no threads, no drivers, no clock
      state.hpp                LoopState (control) and VisionState (vision): POD aggregates
      frontend/
        measurement.hpp        Measurement, IMeasurementSource
        camera/                CameraFrontend: image stage (rim and contact line along table rays)
                               and measurement stage (wall elevation, harmonics, rim registration)
          edge_cnn.hpp         ONLY IF the classical detector fails the kill-test-2 gate: the 1D CNN
                               edge step, plain Eigen, fixed run time, weights in a POD struct
                               (config switch edge_method: classical | cnn; research plan §2.3)
        ft.hpp                 FTFrontend: PARKED, declaration only
      clock/                   camera clock -> host clock fit, a pure function of logged pairs
      kin/                     FK of the q_cmd history -> gamma_cmd, omega, alpha; servo-response model
      estimator/               LTV-KF, delayed-measurement history, predictor ladder
      corrector/  tracker/  safety/
      gen/                     GENERATED: never hand-edited, gitignored
    app/                       IMPURE: everything that touches the world
      main.cpp  loop.cpp       the 1 kHz control thread
      cam/                     acquisition thread over the camera library; frame ring
      vision.cpp               vision thread: calls rt/core/frontend/camera once per frame
      elfin/                   C++ port of elfin_min.py: command client (port 10003),
                               Datasheet reader (port 10004)
      armlink/                 IArmLink: tcp_servo (path A); ethercat (path B, empty until the pivot)
      log/                     record rings, record writer, image writer
    bindings/                  pybind over rt/core, links the SAME static library

  sim/
    l0/                        reduced-model physics tests
    harness/
      drive_replay.py          recorded trial -> rt/core, both replay levels (§7)
      drive_l1.py              kinematics + servo model + camera model + delays -> rt/core
      camera_model.py          synthetic contact-line points and frame timing (OpenCV projection
                               with a calibration record)
      servo_model.py  delay.py

  analysis/
    logread/                   generated reader, schema-versioned
    identify/                  ring-down fits (matrix pencil + NLS), servo-response fit, clock fit
    cup.py  gate_memo.py       fill from weight; the kill-test-2 gate memo (research plan §7.1)
    cnn/                       ONLY IF the gate fails: dataset extraction, labels, PyTorch training in
                               its own venv (requirements-cnn.txt), weight export; never imported by rt/
    figures/                   one script per paper figure; each emits .pdf + .json
    paper_numbers.py           sidecars -> LaTeX macro file

  tools/
    rtlint/                    the RT rules, as a CI check
    record.py                  only writer of manifest rows; owns randomization
    elfin_py/                  elfin_min.py UNCHANGED from the knowledge files; the command table
                               (protocol.py, the protocol reference); a logging wrapper;
                               bring-up examples 1-4; the servo ladder
    elfin_mock/                local mock controller on ports 10003 and 10004 (§3.8)
    calib/                     ChArUco calibration, T_KC, ray table, masks, meniscus map,
                               focus and lighting preview
    cam_bench/                 frame rate, jitter, dropped frames, noise (kill test 2(a)-(c))
    led_latency/  clock_sync/  latency_chirp/
    workbench/                 the operator interface (§12): stdlib web page, stage registry runner,
                               coefficient store; never in the loop
    label/                     ONLY IF the gate fails: the contact-line labelling tool (CNN labels)
    backup.sh

  docs/
    memos/                     latency, servo-ladder, fk-check, killtest1, camera-go-no-go (R1),
                               corrector-choice
    data_dictionary.md         generated from schema/
    journal.md                 one line per day

  tests/
    data/reference_trial_v1.bin, reference_trial_v1.img   vendored 3 s trial with its images (git-lfs)
    l0/  rt/  identify/  antifight/  image/  elfin/

  config/workbench/stages.yaml the stage registry of the hardware phase (§12) -> docs/stages.md

  data/                        not in git
    trials/  images/  params/  calib/  latency/  annotations.csv  manifest.csv
    workbench/                 coefficient versions, accepted stage results, audit log (§12)
    runs/                      one folder per workbench run, read-only once complete (§8, §12)
  ~/slosh-backup/slosh-data/   the local backup of run folders and trial logs (§8); outside the repo
    cnn/                       ONLY IF the gate fails: datasets, labels, weights with their hashes
```

Two languages, one boundary: nothing Python runs inside the control cycle, and nothing C++ makes a figure. The command table is the protocol reference and `elfin_min.py` the bring-up client (R-B); neither runs during an experiment. Python tools that talk to the arm use the standard library only and build on `elfin_min.py` (the lab's rule); analysis, calibration and code generation use NumPy, SciPy, OpenCV and CasADi.

**Where the code runs.**
- **Target.** The real-time process runs on a separate Linux machine. §6 and D4 describe that machine: `SCHED_FIFO`, isolated cores, `mlockall`, the pinned flags, `backup.sh`.
- **Development.** On a Windows PC, in WSL2 (Ubuntu), with the same compiler and flags as the target, so that only timing and hardware differ. There is no MSVC build.
- **Portable:** `rt/core`, `models/`, `analysis/`, `sim/` and the tests build and run on both. `rt/core` includes no OS headers.
- **Linux-only:** `rt/app`. Its OS calls (scheduling, sockets, camera acquisition) sit behind one small file per concern; the development build compiles a stub or a non-real-time version of each, for running against the mock, never for timing.
- **Target-only runs:** timing gates, the allocation hook under load, camera acquisition and anything on the arm. The development machine runs unit tests, golden vectors, replay, the mock arm and dry runs.
- **Python tools on the arm** run from the Linux machine only. Windows runs of them are against the mock: Python's sleep timing there is not representative.

**Why `core` / `app` is split.**
- It makes "no I/O in the cycle" a build-system fact instead of a discipline, and it gives `rtlint` a directory instead of a judgement call.
- The RT binary and the pybind module link *one* compilation of the estimator and of the image stage. That is what makes the 1e-12 estimator replay and the bit-exact image replay (§7) meaningful rather than a compiler-flags argument.

**Why identified parameters live in `data/params/`, not `config/`.** They are outputs with provenance and IDs, not configuration. Keeping them in `config/` invites hand-editing, which breaks the `params_hash` lineage. Calibration records (`data/calib/`) and latency records (`data/latency/`) follow the same rule.

---

## 3. Coding rules

Enforced by `tools/rtlint` and CI where possible, by convention where not. Each exists because of a specific failure mode in this project.

### 3.1 Determinism (these are what make §7 work)

- **D1. No hidden inputs in the cycle.** Nothing in `rt/core` reads the wall clock, an environment variable, a file, an RNG or a global. Time arrives as an argument.
- **D2. One state struct per thread, snapshottable.** All module state lives in POD aggregates reachable from `LoopState` (control thread) or `VisionState` (vision thread). Replay can memcpy them at t = 0 and step forward, so hidden state is impossible by construction. This also gives mid-trial replay, wanted when a 30 s trial fails at second 22.
- **D3. Physics modules are pure**: `step(const State&, const Input&, const Params&) -> State`. Mutation happens in one place in `loop.cpp` and one in `vision.cpp`.
- **D4. Pinned float flags, one build.** `-O2 -march=x86-64-v3 -ffp-contract=off`. Never `-ffast-math`: it lets the compiler assume no NaNs, which silently deletes the safety guards. `rt/core` compiles once into a static library that both consumers link.
- **D5. No iteration over unordered containers** and no ordering that depends on pointer values, including in the setup code that writes the log header.
- **D6. Seeds are logged**: margin Monte Carlo, trial randomization, synthetic-noise and synthetic-image runs.

### 3.2 Real time

- **RT1. `tools/rtlint` greps `rt/core`, `rt/app/loop.cpp` and `rt/app/vision.cpp`** for `new`, `malloc`, `std::vector`, `std::string`, `std::map`, `throw`, `printf`, `system_clock`, `sleep`, `mutex`, `shared_ptr`. Runs in CI. There is no reviewer; this is the reviewer.
- **RT2. Fixed capacity, runtime size.** `Eigen::Matrix<double, kMaxMeas, kStateDim>` with an integer `n <= kMaxMeas`, operating on `H.topRows(n)`. Generality without allocation. Frame buffers, the delayed-measurement history and every ring are allocated once at start-up.
- **RT3. Errors convert at exactly one site per thread.** Camera-library error codes, socket errors and the Elfin's `Fail` replies are converted in `rt/app` to a status word and logged. Nothing throws in `rt/core`. Outside the cycle (start-up, bring-up tools) a `Fail` raises an error, as `elfin_min.py` does.
- **RT4. Every tick emits a record, including degraded ones.** A dropped tick that logs nothing is indistinguishable in analysis from a tick that went fine. The same holds for every frame in the vision thread.
- **RT5. Status flags, never sentinels.** No `-1` for invalid, no NaN as a signal. Validity travels beside the data.
- **RT6. Nothing in the control thread blocks.** The push–reply exchange with the arm and the camera grab block in their own threads (§6). The control thread only reads and writes lock-free rings and mailboxes.

### 3.3 Frames and units

- **F1. Every vector name carries frame and units**: `a_P_W`, `eta_C_m`, `uv_px`. The frames are W, F, K and C, plus the hinge P and the image coordinates `(u, v)` (`README.md`). The measurement chain goes pixel → ray in K → wall cylinder in C, and γ goes from FK in W into C. This is where a sign error will live.
- **F2. `R_AB` maps B→A**, so composition reads left to right: `R_WC = R_WF * R_FC`; `T_KC` maps points in C to K. Never a bare `R`.
- **F3. Quaternions in the loop, Euler only in figures.** Fix the sign hemisphere on construction (`if (q.w() < 0) q.coeffs() *= -1`).
- **F4. SI everywhere in code and log**, with two exceptions at the edges: pixels in image features, and the Elfin's degrees and millimetres, which exist only inside the command client. `PushServoJ` takes degrees (elfin_commands.md, PROTO p.173), `SetPayload` kg and mm (PROTO p.48), `WayPoint` mm, degrees, °/s or mm/s (PROTO p.137–139). One conversion function per command, each with a test. Millimetres in the log will cost a factor of 1000 somewhere.

### 3.4 Numerics

- **N1. Joseph-form covariance update, plus explicit symmetrization** (`P = 0.5*(P + P.transpose())`) every step. A 30 s trial is ~30,000 predict steps and ~6,000 delayed updates, each followed by re-propagation, on a lightly damped mode. The standard form loses symmetry and then positive-definiteness, and it fails as a slow drift in σ that corrupts the safety threshold rather than as a crash.
- **N2. Second-order series discretization, with the error bound in a comment and a test.** At ω₀·dt ≈ 21 × 0.001 = 0.021, `Φ ≈ I + A·dt + A²dt²/2` leaves a first neglected term of ≈ (ω₀dt)³/6 ≈ 1.5e-6 per step. That is cheaper than a per-tick matrix exponential, and the error is writable.
- **N3. Debug-build invariants**:
  - `P` symmetric and PD; the innovation covariance's condition number; a NaN guard on every state;
  - `|s|/L < 0.3`, Model A's stated validity bound. Log a flag rather than assert, because trials near the rim will violate it and you need to know which;
  - history times strictly increasing; a measurement older than the history is dropped and flagged, never applied.
- **N4. Golden vectors include the nasty cases**:
  - large `ω_z` (the Coriolis term `−2ω_z J ṡ` is the most likely sign error in the model);
  - `G` far from `g`; h/R at both extremes;
  - near-zero `|s|`, where the `sqrt(a²+b²+ε²)` smoothing bites;
  - large `s*` (lean > 20°), where the plane gain and the mode gain of `D_z` differ most;
  - a delayed update one tick old and one 100 ms old.

  Tolerance 1e-10 **relative**.

### 3.5 Logging

- **LG1. Every behavioural branch emits a bit**: corrector clamped, frame gated, measurement older than the history, predict-only, push late, push answered `Fail`, safety monitor considered but did not trigger. One flag word per tick. Otherwise "why did this trial behave differently" is archaeology.
- **LG2. Requested and applied, always both**: `Δa`; the tilt-law rate limiter; the tracker's joint-acceleration scaling; the `q_cmd` the tracker computed and the `q_cmd` that was pushed.
- **LG3. Consumed IDs are logged**: the frame IDs whose measurements each tick applied, with their `t_valid`; and, for each push, the tick whose `q_cmd` it carried. Without them replay cannot reproduce which tick saw which frame (§7).
- **LG4. Everything sent to and received from the arm is logged with host timestamps**, as bytes, in the RT process and in every Python tool. This is the lab's rule, and it is also the only record of what the controller was told.

### 3.6 Analysis and paper

- **A1. Every figure script emits a sidecar.** `fig07_*.py` writes `fig07.pdf` **and** `fig07.json` with the plotted numbers, the trial IDs consumed, the git hash and the manifest query. `paper_numbers.py` turns sidecars into LaTeX macros. This is what stops the abstract saying 23 % while the figure shows 21 %.
- **A2. Notebooks are scratch** and are never a figure source.
- **A3. Analysis scripts assert on the `dataset` tag**, with the kill-test-3 exemption as a named constant with a comment, written now, not in week 21.
- **A4. No trial file is ever renamed, moved or edited.** Corrections are additive rows in `data/annotations.csv`, keyed by trial ID. The same holds for image files and calibration records.

### 3.7 Solo-developer process

- **P1. Milestone tags** (`git tag ms1-infra`), so that a figure's git hash resolves to something meaningful at submission.
- **P2. Recording an `eval` trial from a dirty tree is a hard error**, not a flag. `docs/journal.md` alone does not make the tree dirty: it is notes, not code, and the workbench appends to it after every run (§12).
- **P3. Decision memos are files**: latency, the servo ladder, the FK check, kill test 1, the camera go/no-go (R1), the week-6 corrector choice. Three paragraphs each: what was measured, what was decided, what would reverse it. These become the paper's methods section and cannot be reconstructed in week 22.
- **P4. `docs/journal.md`, one line per day.** The only defence against "why did I change that gain in week 9". The workbench adds one line per run, accept, override or coefficient edit, in the journal's separate "Runs" section, so the daily lines stay one per day (§12).

### 3.8 Operating the arm (new)

These are the lab's rules for the Elfin, written as code requirements. They apply to the RT process and to every tool that talks to the arm.

1. **One message at a time.** Wait for each reply before sending the next message; a `Fail` reply is an error (elfin_commands.md, PROTO p.1, p.3). In start-up code it raises. In the streaming thread it stops streaming and ends the trial (RT3).
2. **Check before any motion.**
   - `ReadRobotState` must show nEmergencyStop = 0, nSaftyGuard = 0 (light curtain), nErrorState = 0 and nEnableState = 1 (elfin_commands.md, PROTO p.61–62).
   - `ReadEmergencyInfo` must show no disagreement between the redundant channels (PROTO p.60).

   Otherwise refuse to move.
3. **After an error**, print the error code and axis (`ReadRobotState`, `ReadAxisErrorCode`; PROTO p.61, p.64) and ask the operator before sending `GrpReset` (PROTO p.19), so that what faulted (a collision stop, say) is seen before it is cleared. Wait for its success before any further motion (PROTO p.189), then run check 2 again.
4. **Wait for each move to finish with `ReadRobotState`.**
   - Waypoint moves (approach, return): poll nMovingState every 10 ms after first waiting for the move to start, as `wait_done` in `elfin_min.py` does (elfin_commands.md, PROTO p.189–191; elfin_examples.md). How soon nMovingState turns 1 is **[MISSING]**, so when the target is known a 0 seen before the start counts as done only once `ReadActPos` shows the arm at the target (PROTO p.66–67; pre-hardware review S12, 2026-11-18).
   - After a servo stream, wait until nMovingState = 0. What it shows in servo mode is **[MISSING]**; the ladder records it.
5. **Start slow, and go faster only with the owner's confirmation.**
   - Start at 20 °/s and 100 °/s² for joint moves, 20 mm/s and 100 mm/s² for linear moves, and 20 °/s and 100 °/s² for streamed joints, streaming at the manual's 0.2 s period with 0.02 s lookahead (elfin_commands.md, PROTO p.172). The accelerations are example 2's values (elfin_examples.md).
   - `config/arm/confirmed.yaml` holds the highest joint speed and acceleration, linear speed and acceleration, streamed joint speed and acceleration, and the shortest servo period and its lookahead that the owner has confirmed.
   - Every tool computes the planned peak speed and acceleration of every motion before sending it (numerically, for a streamed trajectory) and refuses if either exceeds those values. The client and the tracker refuse; they do not clip. A linear WayPoint's joint speeds are not planned, and that dVelocity and dAcc are the controller's peaks is **[UNTESTED]** (elfin_commands.md, PROTO p.137–139), so the tools also read the joint speeds during every move (`ReadActJointVel`, PROTO p.71) and stop it above the confirmed joint speed (pre-hardware review S20, 2026-11-18).
   - The servo ladder runs one period per invocation, at the period in `confirmed.yaml`; it never prompts for one. Its sequence and stop rules are in research plan §9, week 1.
   - Only the owner edits that file. Each change is its own commit, with a journal line saying what was seen at the previous step. The workbench shows it read-only and writes it only after the owner types the new value and the confirmation phrase (§12); no agent edits it.
6. **Servo streaming is [UNTESTED].**
   - Log the reply time of every push and count late pushes.
   - There is no documented way to leave servo mode (elfin_commands.md, PROTO p.172–175). A stop is therefore a streamed deceleration to rest, then the end of the stream, then `GrpStop` (PROTO p.20), whose effect on servo mode is **[UNTESTED]**.
   - The e-stop stays within the operator's reach.
7. **Watch the arm during streaming.**
   - The state stream's documented keys include the error code and the enable state but no e-stop or light-curtain flag (elfin_commands.md, PROTO p.185–188). Week 1 checks which keys the real packets carry.
   - If those flags are absent, the ladder tests whether `ReadRobotState` can be sent between pushes and what it costs in slot time **[UNTESTED]**.
   - A push answered with `Fail` is treated as a stop either way.
8. **Settings sent over TCP revert at controller restart** (elfin_overview.md, PROTO p.5). Start-up therefore reads back what can be read: the limits (PROTO p.56–59) and the current tool and user frames that `ReadActPos` returns (PROTO p.66–67). It refuses to enable if they differ from the config. No command in the knowledge files reads the payload back **[MISSING]**, so payload and TCP are set on the pendant, where settings are saved, and the operator confirms them at each start-up. Payload and TCP must be right before enabling (elfin_overview.md, LINUX p.17).
   - **Power and enable.** No tool sends `Electrify`, `StartMaster`, `ConnectToBox`, `BlackOut` or `OSCmd` (elfin_commands.md, PROTO p.8–13). If the arm is unpowered (nElectrify = 0), the tool refuses and the operator powers it on from the pendant. Only commands that the command table marks as used can be sent at all.
   - **Disable on every exit.** A tool that enabled the arm sends `GrpDisable` (PROTO p.18) in a `finally` block, so that it also runs on an exception or Ctrl-C. If a message is in flight, its reply is read first (one message at a time, PROTO p.1). If a move may be under way, `GrpStop` (PROTO p.20) goes first, with a bounded wait for nMovingState = 0, because what `GrpDisable` does to a moving arm is **[MISSING]** (owner's go-ahead on the pre-hardware review, 2026-11-18).
   - **Frames.** While the flange is bare, the pendant shows payload 0 kg, and the tools pass `TCP`/`Base` and require both frames to be zero before a linear move (elfin_examples.md, example 2). With the holder on, the payload is the weighed holder, cup and water, with the centre of mass converted into the flange frame found by the jog test; the TCP is the cup axis at rim height, taught on the pendant as `CUPRIM`; and the tools pass `CUPRIM` and check the frame values that `ReadActPos` returns against the config.
9. **The ROS EtherCAT driver's config files** hold this unit's E03 encoder zero counts and Elfin3 torque factors (elfin_overview.md). They are still not used as they are: path B starts by checking them against the arm and getting the vendor's confirmation for the E03-Pro.
10. **Every tool that can move the arm runs without it.**
    - `--dry-run`: no connection; every command is printed and logged with timestamps.
    - `--mock`: the local mock controller (`tools/elfin_mock`). It checks message format and parameter counts, enforces one message at a time (it drops extra messages, as the real controller does; PROTO p.1), splits replies into fragments, and streams Datasheet packets in the manual's format, including its quirks (string numbers, a trailing comma; PROTO p.185–186). It simulates servo tracking with a configurable delay and lag, and injects `Fail` replies, late replies and dropped connections.

    The mock tests our framing and our reactions, not the arm's behaviour.
11. **Every motion tool's header carries a "first run on the real arm" note**: what to watch, and when to hit the e-stop. The notes for the first month's tools:

| Tool | Watch the first time on the real arm |
|---|---|
| Bring-up, examples 1–2 (elfin_examples.md) | The model string and the limits read back; the current tool and user frames are zero before any linear move; enabling releases the brakes "with sound and slight movement" (elfin_overview.md, LINUX p.17–18) |
| Servo ladder (example 3 extended) | Reply time per push (median, maximum) and late count at each period; commanded vs state-stream J6; any `Fail` code (the manual's failure example shows 40014, meaning not given; elfin_commands.md, PROTO p.173); what the arm does when pushes stop; what `GrpStop` does. Stop the ladder and report on any `Fail`, more than 1 % late pushes, a median reply time above T_s/2, commanded vs state-stream J6 that looks wrong, or audible or visible roughness |
| Latency chirp | Start at a small amplitude and a low top frequency; servo faults or audible resonance; the static target stays in view |
| Kill-test-1 runner | Each speed step confirmed; planned limit ratios printed before each run and realized ratios after; collision stops (elfin_overview.md, LINUX p.123–124) |
| Shake routine | Splashing at a 3–5 mm kick; the cup stays clamped; the ring-down is visible in the live harmonics |
| First closed-loop trial (week 6) | Corrector authority and clamp flags; the corrector is off during the shake; a frame dropout switches to open loop |

---

## 4. The model layer

**Source of truth: `models/model_a.py` and `models/measurement.py`**, in CasADi. They expose:
- `sys_matrices(params, gamma_xy, G, omega, alpha) -> A, B`: the continuous LTV system of research plan §2.2, with the `G(t)` stiffness and the forcing-disturbance input `d`;
- `nonlinear_rhs(...)`: the spherical-pendulum form, for L0 and as the step-2 fallback;
- `crest(s, s_star, params)`: research plan §4;
- `meas_matrices(params) -> H, D_z`: `H = [κI, 0, 0, I, 0]` and `D_z = (R/L − κ)I` (research plan §2.3, §4);
- `mode_shape(r, theta, h_over_R)` and its gradient. A pivot method's point-elevation or point-slope measurement is then a few lines in its front-end, not a model change (R-D);
- `noise_form(params, sdot, s_minus_sstar, t_exp)`: the form of `R(t)`; the coefficients come from kill tests 2 and 5;
- `wrench(...)`: **parked**. It stays in the source and is not generated, so that the F/T pivot remains one file.

Also: `chi`, `kappa` and `r_over_l` in `mapping.py`; frames in `frames.py`; FK and the Jacobian of the E03-Pro in `kinematics.py` (Pinocchio–CasADi), once the kinematic model has been checked against `ReadActPos` poses. The model comes from the E03 STEP model in the attachments, or from an E03 description in the vendor's driver if it has one; the knowledge files mention only the `elfin5` URDF (elfin_overview.md).

**`codegen.py` emits C into `rt/core/gen/`** for the system matrices, the measurement matrices, FK and the Jacobian. The C++ does the discretization, the Kalman update and the IK iteration in hand-written Eigen. Do not codegen the filter: the series expansion and the covariance update belong where the numerics are visible (N1, N2).

**Golden-vector test.** `models/golden/vectors.json` holds ~50 fixed input tuples spanning the operating range, including the N4 cases, plus ~20 joint configurations for FK. CI runs the Python model and the generated C on all of them at 1e-10 relative, under the pinned flags of D4. This is the only thing standing between you and silent drift, and it costs an hour.

**Parameter set.** `config/params.schema.yaml` defines it once and generates the C++ struct, the Python dataclass and the log header block. It holds:
- ω₀, ζ, h, m_ℓ;
- `p_h` with its source (theory or tilt kick);
- m₁, theory only, carried for documentation and the parked F/T pivot;
- the intervals passed downstream (research plan §3.2);
- the IDs of the identification trial, the calibration record and the latency records that go with them.

κ, R/L and χ are derived from h/R in the model layer, never stored. The set appears in the identification output, the RT process, the header, the NMPC and every figure; defining it five times is how a unit mismatch survives to week 20.

**Not parameters**: the calibration (intrinsics, distortion, `T_KC`, ray table, masks, meniscus map) and the servo response are records with their own IDs (§8). The parameter file names them.

Model B lands in step 2's month 1 against the same interface. If it does, `codegen.py` also feeds acados; that is why the source is CasADi rather than hand-written C++.

---

## 5. The log

Everything in both papers reads from this. It is the only artifact where a late change is unaffordable.

**Generated, not hand-written.** `schema/fields.yaml` carries name, type, unit, frame and `since_version` for every field. It generates the C++ writer, the Python reader and `docs/data_dictionary.md`. Hand-maintaining a binary writer and a NumPy reader in two languages across N schema versions is a guaranteed silent-corruption source, and the data dictionary is wanted for the paper's reproducibility statement anyway.

**Format.** Two files per trial:
- `<trial>.bin`: header + append-only tagged records;
- `<trial>.img`: append-only image records (frame ID, ROI geometry, losslessly compressed pixels).

No rewriting, no in-place edits, no trailer that must be present for a file to parse. A trial killed mid-run by a stop must still be readable: the reader treats a truncated last record as the end of the trial, not as an error.

**Records.** Every record is a tag, a length and a payload. Streams of fixed length keep fixed payloads; `wire_tx`, `wire_rx`, `state_raw` and the image records carry variable-length bytes.

**Header** (JSON, length-prefixed):
- `schema_version` (integer, bumped on any change);
- `git_hash`, `dirty` flag, build timestamp, compiler flag string;
- **full config snapshot**, including `confirmed.yaml` and `camera.yaml`, not a path or a hash;
- `params_hash` and the full parameter set, plus the identification trial ID that produced them;
- **calibration ID and hash**, and the residual of the session's check image (research plan R15);
- **front-end block**: `edge_method` (`classical` or `cnn`); for the CNN, the model ID, the weight-file hash, the dataset hash and the training-config hash (research plan §2.3). Additive (R-A); `cam_feat`'s edge strength carries the confidence for either front-end;
- **workbench block**, when the workbench started the trial: the coefficient version and its full snapshot, and the stage ID (§12);
- **camera block**: model and serial number, firmware, camera library and its version, pixel format, ROI, exposure, gain, frame rate, trigger mode, the timestamp tick rate as reported (its true rate is **[MISSING]** until measured); lens f-number and focus state as entered by the operator;
- **arm block**: the model string (`ReadRobotModel`; elfin_commands.md, PROTO p.16), the controller version (pendant About screen; elfin_overview.md, LINUX p.19), the limits read back at start-up (PROTO p.56–59), the payload and TCP in use, the servo period and lookahead in use, and the confirmed values. The header is written before the arm is reached, so the block goes in as the event of kind 5 (`schema/fields.yaml`) once the start-up reads are done; the confirmed values stay in the header (pre-hardware review W10, 2026-11-18);
- trial metadata: cup ID, nominal fill, the fill height as measured by the owner, liquid (clear or dyed), lighting set-up ID, method (M1–M4), path family, speed factor, repeat index, **randomization block ID and seed**;
- `dataset` tag: `tuning` | `eval` | `killtest` | `scratch` (§8);
- IDs of the latency records (τ_a and the servo response, τ_cam and the timestamp latch event, the LED clock check), **as metadata, unapplied**;
- associated media: scale reading, operator note.

**Streams**, one tag each:

| Stream | Rate | Contents |
|---|---|---|
| `frame` | per frame | frame number, camera timestamp (raw ticks), trigger counter, host arrival time, exposure time, gain, dropped-frame count, index of its image record (or "not stored") |
| `cam_feat` | per frame | feature type; per sector: rim edge and contact-line edge as sub-pixel `(u, v)`, edge strength, valid flags; image-stage duration |
| `meas` | per frame | η per sector; harmonics `C₀, C₁, S₁, C₂, S₂`; rim registration (pose correction, residual in px); the `Measurement` (n, z, H, D_z, base R, t_exp); `t_valid` on the host clock and the clock fit's current offset and drift; flags |
| `intake` | 1 kHz | frame IDs whose measurements this tick applied, with their `t_valid` (4 slots and an overflow flag, enough at 200 fps); measurements dropped as older than the history; the latest state packet and push reply seen |
| `kin` | 1 kHz | the delayed `q_cmd` sample used; γ_cmd, ω, α in C; `s*` |
| `est` | 1 kHz | ŝ, ṡ̂, d̂, b̂_z (δ̂); P (upper triangle); innovation and NIS; gate flags; predict-only flag; ticks re-propagated |
| `pred` | 1 kHz | predicted state at a **ladder** of horizons (0, 10, 20, 40, 60, 80 ms) |
| `corr` | 1 kHz | Δa requested, Δa applied, saturation and scaling flags |
| `ref` | 1 kHz | p_d, v_d, a_d, j_d, R_d, ω_d, α_d, ideal-minus-realizable lean |
| `arm_cmd` | per push | push ID, the tick whose `q_cmd` it carries, `q_cmd` (rad), host send time |
| `arm_ack` | per reply | push ID, OK or `Fail` with its code, host receive time, reply time, late flag |
| `robot_state` | ≈ 20 Hz | the parsed Datasheet: joint positions, velocities, accelerations and currents; state and error fields; host arrival time. The packet has no timestamp (elfin_commands.md, PROTO p.185–188) |
| `wire_tx`, `wire_rx` | per message | every string sent and received on port 10003, verbatim, with host times (path A) |
| `state_raw` | ≈ 20 Hz | every Datasheet packet on port 10004, as bytes, with host arrival time |
| `safety` | 1 kHz | monitor state, η̂ + 3σ, trigger reason |
| `flags` | 1 kHz | one word: every branch taken this tick (LG1) |
| `timing` | 1 kHz | tick duration, overrun counter, ring depths, image-writer backlog, the last frame's vision-stage time |
| `sync` | event | LED commands timestamped at the tick; trigger pulses sent, if the camera is hardware-triggered; electrode transitions |
| `event` | event | operator markers, phase transitions, errors converted at the RT3 sites |

**Every record carries two timestamps**: the source clock, where one exists, and the host monotonic receipt. The host clock is `CLOCK_MONOTONIC`; the Python tools read the same clock with `time.monotonic_ns()`, so their records align with the C++ process's. Wall-clock time is recorded once, in the header. Timing figures count only from the Linux machine. Alignment is an analysis-time operation, always reversible. The one alignment applied online is the camera-clock mapping, because the estimator needs `t_valid` on the host clock. It is computed by a pure function of the logged (camera timestamp, host arrival) pairs, logged as a witness, and can be re-fitted offline.

**Three additions v2 lacked, all load-bearing.**
1. **`intake` records frame IDs.** The vision thread and the control tick are separate clock domains. Which tick first sees a frame's measurement depends on arrival timing that replay cannot know. Logging the consumed frame IDs costs ~16 bytes per tick; it is the difference between a determinism test that passes and one you disable in week 9.
2. **`t_valid` is the exposure midpoint, not the arrival time.** Which event the camera's timestamp latches (exposure start, exposure end or readout) is **[MISSING]**. The LED test settles it in week 1, and the offset goes in a latency record, not in code.
3. **Images are logged**, in their own file, so that the image stage can change after the fact (R-C).

**Image policy** (decided in week 1):
- every frame: the ROI annulus, from just inside the lowest contact line to just outside the rim, lossless;
- every 20th frame **[target]**: the full frame, lossless, for forensics and calibration checks;
- compression in the image-writer thread, never in the loop (zstd at a fast level or similar; ratio and CPU cost measured in week 1 **[unverified]**);
- if the writer falls behind, it drops image records, never features or measurements, and flags the drop.

**Sizing** (computed for Mono8 at 200 fps and the Rev F geometry, before compression) **[unverified]**:
- the annulus ≈ 96 MB/s, ≈ 2.9 GB per 30 s trial;
- full frames every 20th frame ≈ 16 MB/s;
- ≈ 2.3 TB for the ≈ 800 trials of step 1;
- the other streams ≈ 1.5 MB/s, ≈ 45 MB per trial.

Every full frame at 200 fps would be 311 MB/s; that is why only the annulus is kept for every frame.

**The LED and the trigger.**
- The camera's embedded timestamp makes frame timing exact on the camera clock (the timestamp, frame number and trigger counter are listed on the MaxxVision page). The LED is the check: a PC-driven LED in view, switched at known ticks (logged in `sync`), whose first visible frame gives τ_cam and the timestamp's latch event.
- Do not snap LED events to frame boundaries. At 200 fps, quantization alone is ±2.5 ms ≈ ±3° at 3.37 Hz, out of a 20° phase-error target. Interpolate on the frame's exposure window.
- If the camera is hardware-triggered from the control tick through its opto-isolated input (Line 0), the trigger time is on the host clock and the clock fit becomes a check. This needs a trigger output device **[MISSING: choice]** with its own measured latency. Decide in week 1.

**Reader.** Generated per schema version; version-N readers stay in the tree forever. A round-trip test writes one record of every type, including an image record, and reads it back.

---

## 6. Threads, the cycle and the interfaces

**Threads.** The RT threads are `SCHED_FIFO`, CPU-isolated and memory-locked. Priorities and core assignment are set in week 1 and verified by the timing test.

| Thread | Rate | Does | Hands to |
|---|---|---|---|
| `cam` | per frame | grabs from the camera library; copies the ROI and metadata into a preallocated frame ring; no processing | `vision` (SPSC ring) |
| `vision` | per frame | image stage + measurement stage, both pure `rt/core`; budget ≤ 3 ms **[target]** at 200 fps | `control` (Measurement ring); image writer |
| `control` | 1 kHz tick | the cycle below | `elfin_io` (latest-value mailbox) |
| `elfin_io` | every T_s | waits for the push slot; takes the latest `q_cmd`; sends `PushServoJ`; blocks for the reply; logs both | port 10003; acks to `control` |
| `state_reader` | ≈ 20 Hz | reads Datasheet packets on port 10004; stamps them on arrival; parses them into a POD | `control` (SPSC ring) |
| writers | — | drain the record rings and the image ring to disk; compress images | disk |

The camera library's own transfer threads are outside our control, and whether their priority can be set is **[MISSING]**; the timing test measures their effect.

**Why `elfin_io` is its own thread.** Path A's push is a blocking request and reply: one message at a time (elfin_commands.md, PROTO p.1). A reply slower than a tick would stall the loop.
- The control thread offers its latest `q_cmd` every tick.
- `elfin_io` sends whatever is current at its slot and logs which tick's sample it carried (LG3).
- It never queues a second push. A slot missed while waiting for a reply is logged as late, not made up.

**Why vision is not in the control thread.** A 3 ms image stage would cost three ticks.

**Cycle order** (control thread):
1. read the clock;
2. drain the state ring and the ack ring;
3. safety pre-checks: errors, `Fail`, stale state;
4. drain the Measurement ring;
5. estimator: for each new measurement, roll back to its `t_valid`, update, and re-propagate to now; otherwise predict only;
6. predictor ladder;
7. corrector;
8. tracker: IK, then the anti-fight rules;
9. offer `q_cmd` to `elfin_io`;
10. safety monitor;
11. push the log records.

**Predict-only is the normal path.** At 200 fps and a 1 kHz tick, four ticks in five have no measurement. Build it in week 4 as the main path, not as a dropout special case in week 9.

**The delayed-measurement history.** A fixed-capacity ring of `(t, x, P, inputs)` covering ≥ 100 ms **[target]**, POD inside `LoopState`. A measurement older than the history is dropped and flagged (LG1). The cost is ~10 predict steps per frame **[unverified]**.

**A trial, end to end** (path A):
1. checks (§3.8 item 2);
2. a joint `WayPoint` to the start pose at the confirmed speed, then wait for the end (item 4);
3. `StartServo` with the confirmed period and lookahead;
4. stream: the shake, or the transport;
5. a streamed deceleration to rest;
6. end the stream; `GrpStop` **[UNTESTED]**; wait;
7. return.

The shake is its own trial (§11).

**Interfaces that exist because a pivot needs them** (three; resist a fourth):

```cpp
// R1 pivot lives here. A source turns its raw input into a measurement MODEL:
// the camera measures wall elevation, not force.
struct Measurement {
  int64_t source_id;                          // frame ID, or the source's sequence number
  int     n;                                  // rows used, <= kMaxMeas
  Eigen::Matrix<double, kMaxMeas, 1>          z;
  Eigen::Matrix<double, kMaxMeas, kStateDim>  H;
  Eigen::Matrix<double, kMaxMeas, 2>          D_z;    // multiplies s*(t_valid); the estimator evaluates
                                                      // s* from its own input history
  Eigen::Matrix<double, kMaxMeas, kMaxMeas>   R_cov;  // base covariance; the estimator adds the
                                                      // state-dependent terms (research plan §2.3)
  double   t_exp;                             // exposure time, for the blur term
  double   t_valid;                           // host clock: exposure midpoint, via the clock fit
  uint16_t flags;                             // valid, sectors gated, rim-registration quality
};

class IMeasurementSource {   // CameraFrontend | pivots: refraction, markers | FTFrontend (parked)
 public:
  virtual bool process(const SourceInput& in, Measurement& out) = 0;   // false: nothing usable
};
```

What changed from v2's interface:
- `process()` is called once per raw input by the thread that owns the source (the vision thread, for the camera). It is not polled by the control tick.
- `t_valid` is in the past by τ_cam, and the estimator's history handles that.
- `D_z` carries the direct feedthrough of the quasi-static surface (research plan §2.3).
- v2's lesson stands. A wrench-shaped interface would have forced the camera to fabricate a pseudo-wrench by twice-differentiating a visual measurement. With `(z, H, D_z, R_cov)` supplied by the source, the KF does a generic sized update, and each pivot is one class plus a config line.
- `FTFrontend` keeps a declaration, so that the R-D test can be argued, and no code.

```cpp
struct Correction { Vec2 delta_a; double delta_tilt; uint8_t channel; };

class ICorrector {                            // options A / B / C
 public:
  virtual void notify_estimate(const PredictedState&) = 0;   // C re-plans here
  virtual Correction sample(double t) = 0;                   // A/B ignore t
};
```

Unchanged from v2. A single `compute(const PredictedState&)` would be a per-tick feedback map and silently exclude option C, a burst planned over about one mode period and re-planned on an estimate event. Under `compute()` it becomes a state machine hiding inside a function that claims to be stateless, and that state, unlogged, breaks replay. Splitting the interface costs nothing and keeps week 6's decision open.

```cpp
// Arm-interface pivot lives here: path A (TCP servo streaming) or path B (EtherCAT driver).
struct ArmCommand { int64_t tick; Vec6 q_cmd_rad; };
struct ArmAck     { int64_t push_id, tick; double t_send, t_reply; uint16_t status; };

class IArmLink {             // TcpServoLink (path A) | EtherCatLink (path B: empty until the pivot)
 public:
  virtual void offer(const ArmCommand&) = 0;        // control thread, every tick, never blocks
  virtual bool poll_ack(ArmAck& out) = 0;           // control thread
  virtual bool poll_state(RobotState& out) = 0;     // control thread
};
```

**Why `IArmLink`.** Path B changes who runs the joints: the PC at 1 ms instead of the Han's controller (elfin_overview.md). Behind `IArmLink` the tracker, the estimator's kinematic input and the streams `arm_cmd`, `arm_ack` and `robot_state` stay as they are. Path B adds its own raw stream in place of `wire_tx`/`wire_rx` (additive). `TcpServoLink` owns the `elfin_io` and `state_reader` threads.

**The C++ Elfin client** ports `elfin_min.py`'s behaviour:
- the same message format, with every parameter formatted as the command table types it (R-B);
- the same reply parsing, including replies split across reads;
- `Fail` handled per RT3.

The Datasheet reader handles what example 4 handles (elfin_examples.md): the `RBTL` header, the byte order chosen so that total = data + 12 (PROTO p.188), string numbers and the trailing comma. It stamps each packet on arrival.

**Where the anti-fight rules live.** In the **tracker**, not the corrector: the authority cap, the joint-acceleration check (80 % of the limits read in week 1, and never above the confirmed values of §3.8), the bandpass, and disable-on-dropout. They must apply to all three options, and the tilt law must never see `delta_a`.

**Option B's open question, flagged, not solved.** If B is chosen, `delta_tilt` sums with the tilt law's output *somewhere*. The estimator's γ_cmd, computed from the commanded trajectory, then sees the sum. Whether that is benign or a loop through the tilt law is not settled by this document. It is settled in week 6 if B wins, and the memo (P3) records it.

**Safety monitor.** It runs regardless of estimator health. Its checks:
- a NaN guard on every state;
- a frame-dropout timeout: open loop after 30 ms, decelerate after a longer dropout **[target]**;
- state-stream staleness;
- Elfin errors and `Fail` replies, passed through;
- the `η̂ + 3σ` threshold;
- the spill flag (rim electrodes, optional).

Its action is a streamed deceleration to rest (§3.8 item 6), commanded independently of the corrector.

---

## 7. Replay and determinism

`sim/harness/drive_replay.py` reconstructs a trial and drives **the same compiled `rt/core`** through `rt/bindings`, at two levels.

1. **Estimator level.**
   - **Input:** `meas`, the `q_cmd` history behind `kin`, `intake` and the header's parameters.
   - **Output:** `est`, `pred` and `corr`, re-propagation included.
   - **Requirement:** agreement to **1e-12 relative on `ŝ`, `ṡ̂` and the diagonal of `P`**. Entries of `P` span orders of magnitude, so an absolute tolerance is meaningless.
2. **Image level.**
   - **Input:** the stored ROI images (`.img`) and the calibration record.
   - **Requirement:** the image stage reproduces `cam_feat` bit-for-bit on the same build and machine (one compiled library, D4), and to 1e-12 relative across machines. The measurement stage then reproduces `meas` to 1e-12 relative.

**Which build defines it.** Determinism is defined on the target build. A WSL build on the development machine, possibly on a different CPU, has to match the target's results to 1e-12 relative, not bit for bit.

CI runs both on `tests/data/reference_trial_v1`, from week 4 on. It is a vendored 3 s trial with its images in git-lfs (≈ 0.3 GB raw, computed **[unverified]**), so the image level runs on its first second.

**What changed from v2, and what did not.**
- **Changed:** the alignment replay must reproduce is now frame to tick, so `intake` holds frame IDs; and there is a second, image-level test.
- **Unchanged:** the single compiled library and the pinned flags (D4). The RT binary and a separately compiled module would not produce identical floats under `-march=native` with FMA contraction. Also unchanged: the relative tolerance and the reference trial in CI.

**What this buys, concretely:**
- a better predictor in week 10 is validated on week-4 data;
- a better edge detector re-runs on stored images;
- a new calibration re-maps old features to wall elevation without images, because `cam_feat` stores pixel coordinates and the calibration maps them to rays;
- the latency sweep is done post hoc from the `pred` ladder;
- kill test 3's artificial-delay runs can be re-analysed without hardware.

---

## 8. Data governance

**Manifest.** `data/manifest.csv`, one row per trial, appended by `tools/record.py` from the header. It is the index for every figure script. Never edited by hand; corrections go in `annotations.csv` (A4).

**Randomization.** `record.py` block-randomizes the method within each (path, speed factor) cell and logs the block ID and seed. Running all M1, then all M4, confounds the method comparison with water temperature, lighting and calibration drift, cup wear and the operator's improving reset technique. It is twenty lines of code, and it is a reviewer question better answered "yes" than with a paragraph.

**Tuning/eval separation.** The `dataset` tag is set at record time, and analysis scripts assert on it.
- Kill test 3's runs are the tuning set for the estimator and corrector gains.
- The research plan's §7.2 grid is `eval`. It is never used to choose a gain, a κ, a noise coefficient, a filter structure or an image-stage setting. Without this separation the prediction-vs-frame figure argues in a circle, and a reviewer will find it.
- The one exemption: kill test 3 is also a paper figure, where it characterizes the *baseline* rather than the method. It is a named constant in the assert, written in week 7.

**Calibration records.**
- **Contents.** `data/calib/<id>/` holds the calibration images, the fitted model and its held-out error, `T_KC`, the ray table, the masks, the meniscus and fill map, and a hash.
- **Rules.** Immutable; every trial header names one. Changing focus, iris, the shim or the holder means a new calibration ID.
- **Session check.** A check image at the start of each session is compared with the record (rim residual). Above 0.5 px **[target]** the session stops for recalibration (research plan R15).

**Latency records.** `data/latency/<id>/` holds τ_a with the servo-response fit, τ_cam with the timestamp latch event, and the LED clock check.
- They are referenced by ID and never applied at record time. The exception is the online camera-clock fit, which is logged as a witness (§5).
- They are re-measured periodically; each measurement is its own record. Since offsets are never applied at record time, a re-measurement invalidates no data.

**Run folders** (owner request, hardware phase). Every run the workbench starts, Arm control sessions, practice runs and forms included, gets `data/runs/<YYYY-MM-DD>_<HHMMSS>_<stage>_<mode>/` (local start time; `-practice` after the mode for practice runs). It holds everything needed to debug the run afterwards:
- `summary.txt`, plain text, read first;
- `run.json`, the run record: command line, exit code, metrics, criteria, times, coefficient version;
- `console.log`, the tool's whole console output, with the workbench's own notes marked `[wb]`;
- `questions.jsonl` (each question and its answer, and who answered: the owner on the page, or `--assume-yes` in dry run or mock) and `commands.jsonl` (each Arm control command and its result). Both carry local time and `time.monotonic_ns()`, the host clock of the trial logs, so they line up with the trial's records (§5);
- `config/` (the config files the run used, practice copies included, and the stage's registry entry), `coeffs_vNNNNNN.json` (the coefficient version used), `form.json` for forms, `mock.log` for mock runs;
- `trial.bin`, a hard link to `data/trials/<trial>.bin`, so it costs no space, and `trial.txt` with the trial's path, size and SHA-256. The trial log is never moved, renamed or copied into the folder (A4). Images (`.img`) are referred to, never linked.

`COMPLETE` is written last. From then on every file in the folder is read-only and the workbench never writes there again. A folder left open by a workbench that stopped mid-run is closed at the next start-up and marked INTERRUPTED. Runs from before run folders existed get a folder marked RECONSTRUCTED, with what their records kept. The run records in `data/workbench/runs/` stay as they were.

**Local backup** (owner request; on the same PC as the data). After every run (it waits while a run is going on), or with "Back up now", the workbench copies what is new to `<backup folder>/slosh-data/`:
- run folders (`runs/`; `trial.bin` re-linked to the backup's own copy);
- trial logs (`trials/*.bin`, every dataset);
- images (`.img`) of `eval` and `killtest` trials only, as below.

The default backup folder is `~/slosh-backup` (on the development PC, `C:\Users\User\slosh-backup`), chosen on the Backup tab and never inside `data/` or the repository. Each machine (Windows, WSL, the Linux machine) keeps its own folder, checked again on every use (pre-hardware review D3, 2026-11-18). The backup copies only what is new and never deletes or overwrites anything there: a file that differs is reported as a conflict and left alone. Every backup is a line in `backup_log.jsonl` with each copied file's SHA-256. It always leaves 10 GB free on its disk **[proposal]**. Restoring a run puts the run folder and its trial back in `data/`. If a current file differs from the backup, nothing changes until the owner confirms; the current file is then kept beside the restored one, renamed `…before-restore-<time>`, not deleted. This copy protects against deleted or damaged files, not against a failed disk: that is what the second disk below is for.

**Backup.** `tools/backup.sh`, nightly:
- `.bin` files of every dataset go to a second physical disk and offsite;
- `.img` files of `eval` and `killtest` trials go to the second disk, and offsite if the upload rate allows (≈ 2–3 TB in all **[unverified]**);
- the run folders and the small records (`data/workbench`, calibration, parameters, latency, manifest, annotations) go to both, with `rsync -H` so that the run folders' hard-linked trials stay links;
- a file that changed since the last backup is copied again, and the copy it replaces is kept in `slosh-data/replaced/<time>/` (`rsync --backup`), never lost; a changed `.bin` is reported, since trial files are never edited (A4) (pre-hardware review D4, 2026-11-18).

Size the disks in week 1, before the grid, not during it. It is irreplaceable single-copy data on one workstation, R8 is rated high, and this is the cheapest large risk reduction in the project. The development PC is not where camera data can live: it had 83 GB free when this was written, about 28 camera trials of 3 GB.

---

## 9. Test inventory

| Test | Where | Gate |
|---|---|---|
| Log round-trip: every record type including images, every schema version | CI | week 1 |
| Generated reader and writer agree with `fields.yaml` | CI | week 1 |
| Loop timing: jitter histogram and zero overruns over 60 s under defined load | `tests/rt`, target machine only | week 1 (scaffold), **re-run week 4** |
| Allocation hook: zero allocations inside the control cycle and the vision stage | debug build; under load on the target only | week 2 (slipped from week 1, §11) |
| `rtlint` clean on `rt/core`, `loop.cpp` and `vision.cpp` | CI | week 2 (slipped from week 1, §11) |
| Elfin client conformance: C++ command strings byte-identical to the command table's for a fixed list of commands; the reply parser on recorded and synthetic replies, including `Fail` and fragmented replies; the C++ client reproduces the week-1 Python ladder's numbers on the arm | `tests/elfin`, CI; bench | week 2 (slipped from week 1, §11) |
| Mock arm: every motion tool runs with `--dry-run` and `--mock`; an injected `Fail` stops it; late replies are flagged; requests above `confirmed.yaml` are refused | `tests/elfin`, CI | week 1 |
| Camera acquisition: the working frame rate held for 60 s with no dropped frames; embedded timestamps monotonic; frame-number gaps counted | bench | week 1 |
| L0 physics: energy conserved at ζ = 0; slosh-free input ⇒ `s ≡ s*`; ω₀ scales with √G; χ consistent with the centre-of-mass shift | `tests/l0` | week 2 |
| Golden vectors: Python vs generated C (system, measurement, FK), pinned flags, 1e-10 relative | CI | week 2 |
| FK vs `ReadActPos` at ~20 configurations, recorded in week 1 on the bare flange with zero TCP and user frame. Positions first; orientations once the Rx, Ry, Rz convention is established, e.g. from a pose where one joint alone has moved (the knowledge files don't state it) | bench | week 2 |
| Discretization error bound vs the matrix exponential at ω₀·dt = 0.021 | `tests/l0` | week 2 |
| Measurement round trip: known contact-line points → OpenCV projection with a calibration record → C++ measurement stage → η within 0.01 mm **[target]**; an injected camera–cup pose error is recovered by rim registration | `tests/image` | week 3 |
| Image stage on synthetic images (edges at known positions, with blur, noise and a meniscus band): sub-pixel bias < 0.05 px **[target]**, and σ vs noise | `tests/image` | week 3 |
| Clock fit: a synthetic offset and drift recovered; on hardware, checked against the LED | `tests/rt`, bench | week 3 |
| Identification repeatability: the same ring-down file gives the same fit; and across repeat recordings | `tests/identify` | week 3 |
| Covariance stays symmetric and PD over 30 s | `tests/rt` | week 4 |
| Delayed update: the live filter matches, to 1e-12, an offline filter that applies the same measurements at their `t_valid` in order | `tests/rt` | week 4 |
| Replay determinism, both levels, on the reference trial | CI | week 4 |
| Safety on the mock: injected frame dropout, NaN, `Fail`, loss of the state stream and the spill flag each give a streamed deceleration | `tests/rt` | week 4 |
| Predict-only path: 200 Hz measurements, 1 kHz tick | `tests/rt` | week 4 |
| Anti-fight: an over-limit request is clamped and flagged; > 80 % joint acceleration scales the correction; a dropout zeroes it within one tick | `tests/antifight` | week 6 |
| Valid-ray rule: fewer than N_valid valid rays, or a quadrant with fewer than 2, flags the measurement invalid; an invalid frame runs the corrector as after a dropout (open loop, M3), and a stream of invalid frames never raises the commanded acceleration (research plan §2.3) | `tests/image`, `tests/rt` | week 4 |
| Workbench: the stage registry is a DAG and names a plan section per stage; coefficient versions, diff, one-click revert and range refusal; staleness propagates downstream; arm mode refused off the Linux machine; `confirmed.yaml` written only after the typed confirmation (on a temporary copy); the stop button, and a killed workbench, each end a mock servo stream with a streamed deceleration and `GrpDisable` (§12) | `tests/workbench`, CI | hardware phase |
| Run folders and local backup: every mock, dry-run, practice and form run gets a complete, read-only folder with the whole console, questions and answers, commands and results, config, coefficient version and a linked trial; an interrupted run is closed at start-up; the backup copies only what is new, never deletes, waits while a run is active, follows the image policy, reports a differing file as a conflict; restore asks before overwriting and keeps the current file (§8, §12) | `tests/workbench`, CI (Windows and WSL) | hardware phase |
| **Only if the gate fails:** CNN parity, C++ vs PyTorch (float64) on every output ≤ 1e-5 on a fixed set of profiles; weight-file round trip and hash; no allocation and a constant operation count per frame | `tests/image`, CI | CNN week 2 |
| **Only if the gate fails:** CNN run time ≤ 1 ms per frame on an isolated core, and the whole image stage within 3 ms **[target]** | `tests/rt`, target machine only | CNN week 2 |

The week-1 timing gate proves the scaffolding: no estimator and no generated matrices, with the camera, vision and image writer running on synthetic work. "Under load" means:
- the camera at the working rate and the image writer compressing;
- the logger writing at full rate;
- a CPU stress process on the non-isolated cores.

The week-4 re-run, with the full cycle, is the real gate.

---

## 10. Not being built

Listed so that the temptation is recognisable when it arrives:

- an abstraction over "which liquid model": there is one until step 2;
- a general experiment-runner or trial-scheduling framework; `record.py`, a shell script and the manifest are enough. The workbench (§12) runs one stage tool at a time at the owner's click; it schedules nothing, randomizes nothing and owns no manifest rows;
- a plugin system for the corrector beyond the three named options;
- a camera abstraction beyond `IMeasurementSource`; OpenCV or any other image library inside the cycle (the image stage is plain code on a raw buffer and a ray table);
- learned image models, **except** the contact-line front-end fallback (owner amendment, research plan §0): built only if the classical detector fails the kill-test-2 gate, it replaces the edge step and nothing else, behind the existing `edge_method` config switch inside `CameraFrontend`, not behind a new interface. No learned model in the estimator, the controller or the measurement model; no training code anywhere under `rt/`;
- `FTFrontend` code or any other force-sensor code, including the protocol's force commands such as `SetForceZero` (the owner's decision in week 1: the camera replaced the F/T sensor); an IMU driver; an EtherCAT/SOEM thread (path B gets one only if that pivot is taken);
- a second camera or any other independent ground-truth rig (decision: the in-loop camera only). The CNN fallback's side reference camera for labels (`docs/design/cnn_fallback.md`, option (i)) would change this line and research plan §0's ground-truth decision; it is not built unless the owner makes that change;
- ROS in the control path;
- anything anticipating the NMPC beyond keeping the model source in CasADi;
- a GUI inside the RT process, or any GUI in the loop; matplotlib after the fact. The workbench (§12) is the one operator interface, a separate process that talks to the tools and the RT process through files and a status/command channel, at the owner's request. The focus and lighting preview in `tools/calib` is a tool, not part of the RT process;
- premature parameterisation of the tilt law;
- a Python twin of the tracker (R-B).

Model B may not survive kill test 5. The corrector has three candidates, and week 6 picks one. The measurement method has one candidate, one fallback for its edge step (the CNN) and two pivots, and kill test 2 says whether either is needed. Writing interfaces for systems you have not measured is how a week disappears. Nothing added in v3 abstracts over an unmeasured system: it is schema, build layout, lint, the arm rules and the three interfaces the named pivots need.

---

## 11. Build order, weeks 1–4

The research side of these weeks (orders, vendor questions, kill tests) is in the research plan §9. This is the software.

**Week 1: log, config, arm tools and mock, servo ladder, camera timing.**
- `schema/fields.yaml` and the generators; the record and image writers, the reader and the round-trip test. The Python tools write the same `.bin` format through a generated, standard-library-only writer, tagged `scratch` (bring-up) or `killtest` (ladder).
- The config tree: `arm/e03pro.yaml`, `arm/confirmed.yaml` (20 °/s and 100 °/s² for joint moves, 20 mm/s and 100 mm/s² for linear moves, 20 °/s and 100 °/s² streamed, 0.2 s / 0.02 s), `cup/r40.yaml`, `camera.yaml`, `params.schema.yaml`.
- `tools/elfin_py`: `elfin_min.py` vendored unchanged, the command table, a logging wrapper (it wraps the socket, so `elfin_min.py` stays untouched), examples 1, 2 and 4, the servo ladder (example 3 extended) and the pose recorder for the week-2 FK check.
- `tools/elfin_mock`. Bring-up runs each tool with `--dry-run`, then `--mock`, then on the arm with the owner, from the Linux machine.
- `rt/app/elfin`: the C++ client and Datasheet reader, with the conformance tests.
- The 1 kHz control-tick skeleton with the rings, the `elfin_io` and `state_reader` threads, the timing test, the allocation hook and `rtlint`.
- `rt/app/cam`: acquisition with the embedded timestamp, frame number and trigger counter; the sustained-rate test; `tools/led_latency`.
- `backup.sh`, with the disks sized for the image policy.
- Still to confirm on the arm (the owner's list): the controller version (5.51.5_EPro4ms.beta.20230522 at inspection; elfin_overview.md, QC p.2–3), the IP address (192.168.0.10 in the lab's scripts), the joint limits, and the shortest servo period that works. About 20 `ReadActPos` poses are recorded during bring-up for the week-2 FK check.

**Protect and slip.** Protected, in order: arm bring-up, the servo ladder, the latency memo, and the backup (once real data exists). If week 1 overruns, these slip, in order:
1. the allocation hook and `rtlint`, to week 2;
2. the C++ client: week 1's ladder runs in Python on the Linux machine, and the C++ client must reproduce its numbers in week 2;
3. frames and arm in one trial: the camera's timing tests are recorded separately in week 1 and combined with the arm in week 2.

*Done when:* the servo-ladder results (push reply times, late counts, the confirmed T_s) and τ_cam are numbers in `docs/memos/latency.md`, and the backup has run once.

**Week 2: models, kinematics, tracker, kill test 1.**
- `models/model_a.py`, `measurement.py`, `frames.py` and `kinematics.py`; codegen; golden vectors; L0 tests.
- Whatever slipped from week 1: the allocation hook and `rtlint`; the C++ client reproducing the Python ladder's numbers; the first trial with frames and arm together.
- The FK check against the week-1 `ReadActPos` poses (memo): positions first, orientations once the Rx, Ry, Rz convention is established. The IK tracker on 6 joints. The tilt law and the reference generator, logging the ideal-minus-realizable lean.
- `tools/latency_chirp`: the camera on a static target with the cup removed, and a chirp on one wrist joint. It gives τ_a and the servo-response record (research plan §4).
- Kill test 1 recording (empty cup), in owner-confirmed speed steps. It starts in week 2 and may run into week 3.

*Done when:* the L0, golden and FK tests are green; the τ_a record exists; the kill-test-1 memo is written from logged limit ratios.

*Note the research plan's gap (kept from v2):* kill test 1 runs with an empty cup, and its outcome rule refers to "speeds where spill occurs". Nothing spills empty, so the memo records which limit binds and at what speed factor. The outcome-A/B call waits until kill test 3 exists in weeks 6–8. Record the limit ratios with the full payload (holder, camera, lens and cup), not the bare cup.

**Week 3: calibration, CameraFrontend, identification.**
- `tools/calib`: ChArUco capture and fit (fisheye vs rational model); `T_KC` from the rim target; the ray table, masks and meniscus map; the calibration record.
- `rt/core/frontend/camera`: the image stage and the measurement stage, with the round-trip and synthetic-image tests; `rt/core/clock`.
- `analysis/identify`: matrix pencil + NLS on the first harmonic; ring-down recording.
- Kill test 2(a)–(c) recordings with `tools/cam_bench`.

*Done when:* the round-trip test passes; the fitter returns f₀ and ζ repeatably from the same file and across repeat recordings; and `CameraFrontend` drives the estimator interface on recorded frames.

**Week 4: estimator live, replay, the camera go/no-go.**
- The LTV-KF at the 1 kHz tick on generated matrices, with the Joseph form, the delayed-measurement history, the predict-only path, the prediction ladder, and the `d` and `b_z` states.
- The replay driver at both levels; the determinism tests in CI.
- Kill test 2(d).

*Done when:* the determinism tests pass on a moving-arm trial; the timing gate passes with the full cycle; and the camera go/no-go memo (R1) is written.

**Where identification runs.** Decide this in week 3, not later.
- **The collision.** The research plan shakes at every pickup (online), but `analysis/identify` is Python. If Python fits and the RT process loads the result, that collides with "no file I/O in the cycle". If C++ fits, the week-3 gate tests a fitter that never runs in anger, and there are two nonlinear least-squares implementations to keep in agreement, which is exactly what R-B forbids.
- **Resolution of record:**
  1. the shake is its own trial;
  2. the RT process records it and stops;
  3. Python fits and writes a params file to `data/params/`;
  4. the RT process loads parameters at trial start only.

  The cost is two seconds of dead time between shake and transport. There is one implementation, and the `params_hash` lineage in the header already supports it.

**Slip branches.**
- **Lens, holder or cups late (research plan R2).** Weeks 1–2 need no camera on the arm. Acquisition, timing and the front-end are developed with the camera on a bench stand with any C-mount lens, and the calibration is redone when the LM4NCL and Rev F are fitted. Kill tests 2(d) and 3 move to arrival week + 1; kill test 1 is unaffected.
- **Servo period too long (R3).** If the ladder has not reached T_s ≤ 20 ms **[target]** by week 2:
  - everything above proceeds at the shortest confirmed period;
  - L1 is run at the measured period, to show whether the predictor or option C can carry step 1;
  - the vendor question and the path-B evaluation start, and the decision is a memo in week 5.
- **Camera library.** If the vendor's SDK does not deliver the embedded timestamp and frame number with each frame, switch to another USB3 Vision/GenICam library in week 1. The `frame` stream does not change.

The claim "by week 4 the structural decisions are behind you" holds only for the schema, the threading, the interfaces and the arm rules, which is the point of building those first.

---

*The four genuinely week-1 decisions:*
1. *`intake` with the consumed frame IDs, and the `frame` stream, in the log schema;*
2. *the threading model, including whether the camera free-runs with a clock fit or is triggered from the tick;*
3. *the generalized `Measurement` struct, with `t_valid` at the exposure midpoint, and the delayed-measurement history;*
4. *the image policy, the disks and the backup.*

*Everything else in this document can move.*

---

## 12. The workbench (owner request, hardware phase)

One operator interface from which the owner runs every stage of the hardware phase, from the bench check to the with-vs-without comparison, sees each result against its pass criteria, accepts it, and adjusts every coefficient. Design notes: `docs/design/workbench.md`.

**What it is.** A local web page served by `python -m tools.workbench`: Python 3.12, standard library only (`http.server`, `subprocess`, `json`); the page is plain HTML and JavaScript with its plots drawn as SVG, no framework. The same workbench runs on Windows (dry run, mock, recorded data) and on the Linux machine (also the arm).

**What it is not.** It is not in the control loop and not part of the RT process, and it never sends a command to the arm itself: the stage tools do, under §3.8. It is not a scheduler (§10): `tools/record.py` keeps the randomization and the manifest.

**Stages.** `config/workbench/stages.yaml` lists the stages in the order of the hardware phase, each with purpose, inputs, tool, adjustable coefficients, pass criteria, output, plan section, modes and prerequisites. `docs/stages.md` is generated from it.
- Status per stage and mode: not run / passed / failed / stale. The stage list shows the arm status, with the dry-run and mock status beside it.
- A stage is locked until its prerequisites have passed on the arm. The owner may override a lock with a typed reason; the override is logged (audit file and journal) and shown on the stage.
- A run on the arm is allowed only on the Linux machine (the tools' own check, `link.arm_host_problem`) and only after the same stage has passed in dry run and in mock with the current inputs (§3.8 item 10).
- **Stale:** a stage's accepted result is stale when a coefficient owned by it or by any stage upstream of it changes, or an upstream stage accepts a new result.

**Stage page.** A mode selector (dry run / mock / arm, the last greyed out off the Linux machine); run; the stop button, visible on every page; the tool's console and its questions, answered on the page and never auto-answered on the arm; live plots read from the trial's `.bin` as it is written (joint commands vs state now; the camera image with the detected edge, η(θ) and the ring-down fit when those stages exist); the results against each pass criterion; and **accept**, which writes the result to an immutable, versioned record in `data/workbench/results/<stage>/` and, for a calibration stage, puts its measured values in the coefficient store with the trial ID as their source.

**Coefficient editor.** Every tunable value in one place, grouped by stage, with its value, unit, source (a trial ID, "hand-set" or "plan default" with the section), allowed range and marker. Definitions, ranges and defaults are in `config/params.schema.yaml` (`tunables:`); the values are in `data/workbench/coeffs/`, one immutable full snapshot per version.
- Every edit needs a note, makes a new version and shows the diff against the version it replaces. Revert is one click: a new version equal to the old one; history is never rewritten.
- A value outside its range is refused.
- Every trial the workbench starts carries the coefficient version and its full snapshot in its header (§5).
- Changing a coefficient marks every stage downstream of its owning stage stale.
- Values that live in config files the bring-up tools already read (`e03pro.yaml`, `camera.yaml`, `cup/r40.yaml`) are shown with the file as their source.

**Arm control tab (owner request).** The first tab: connect and enable (dry run, mock, or the arm on the Linux machine), move the cup slowly with arrow buttons (X, Y, Z in steps of 1–50 mm), set start, middle and end points from where the arm is, save them as a named path, go to a point, and run the path with a pause at each point. A diagram shows the cup from above and from the side with the points, the path and the allowed box. The page only sends commands; the tool `tools/elfin_py/jog.py` (stage 1.6) moves the arm under §3.8:
- every move is a linear `WayPoint` with the orientation held, at `jog_speed_mm_s` and `jog_acc_mm_s2` (10 mm/s and 50 mm/s² **[proposal]**), checked against `confirmed.yaml` before sending (elfin_commands.md, PROTO p.137–139). `MoveRelL` (PROTO p.130) is not used: it takes no speed or acceleration, so its speed could not be checked, and its unit is **[UNCLEAR]**;
- the checks of §3.8 item 2 before every move, the frames checked against `e03pro.yaml`, one move at a time, each waited for (PROTO p.189–191);
- no move leaves `jog_box_mm` round the pose where the session enabled the arm, a step is at most `jog_max_step_mm`, and a point must have the session's orientation; a refused command does not end the session, a `Fail` does;
- the stop button during a move sends `GrpStop` (PROTO p.20) **[UNTESTED]** on this arm, waits for rest and ends the session with `GrpDisable`;
- **idle timeout** (owner request): with no command for `jog_idle_timeout_s` (300 s **[proposal]**), the session ends by itself and sends `GrpDisable`, so the arm is never left enabled with nobody at the page. The page warns in the last minute and offers "Keep connected" (a command with no motion that restarts the timer). Every question of the session is asked before the arm is enabled, so an unanswered question never leaves it enabled.

At 50 mm/s² the quasi-static tilt of the surface is a/g ≈ 0.3° and the wall rises ≈ 0.2 mm (40 mm × 0.05/9.81, computed) **[unverified]**, against 20–60 mm of freeboard. Joint jogging is not offered, because it would tilt the cup.

**Speed limits.** `config/arm/confirmed.yaml` is shown read-only. The workbench writes it only after the owner types the new value and the confirmation phrase, records the owner's note of what was seen at the previous step in the journal and the audit file, and warns while the change is uncommitted: it is still the owner's own commit (§3.8 item 5). The tools refuse anything above the file, whoever started them. No agent edits it.

**Stop.** The stop button sends `stop` on the running tool's command channel, and the tool raises the same interrupt as Ctrl-C, so the same code runs: a streamed deceleration to rest during a stream, then `GrpDisable` on exit (§3.8 items 6, 8). It is a software stop, not a replacement for the e-stop, which stays within reach: what `GrpStop` does during servo streaming is **[UNTESTED]**.

**Isolation and the channel.**
- Each tool runs as its own process, in its own process group, so a crash or a Ctrl-C in the workbench's terminal does not cut a stream off without its deceleration.
- Status and questions go from the tool to the workbench as JSON lines on stdout prefixed `@@wb `; everything else on stdout is console text. Commands and answers go the other way as JSON lines on stdin.
- **If the workbench dies,** the tool's stdin closes. The tool treats that as a stop (deceleration, `GrpDisable`, the trial closed), and it answers "no" to any further question: no motion continues with nobody holding the stop button. When the C++ RT process exists, it implements the same channel in a non-RT thread, with the same rule.
- The live plots read the trial log as it is written; the reader takes a truncated last record as the end so far (§5). Nothing in the loop waits for the workbench.

**Past runs and Backup tabs.** Every run gets its run folder (§8). The Past runs tab lists them, from this PC or from the backup, newest first. Clicking one shows its summary, questions and answers, commands, whole console and joint plot (commanded vs state stream, from the trial log), and opens the folder in the file manager. A run in the backup can be restored from there. The Backup tab shows the backup folder, the last backup, how many runs and trials it holds, the free space, and "Back up now". Only one workbench can serve a port: a second one on the same port refuses to start rather than share it, so two workbenches never write run folders or backups at once.

**Logging.** Every run writes its `.bin` trial, schema v1, exactly as the tool does without the workbench, its run folder (§8), and one line in the journal's "Runs" section (stage, mode, trial ID, run folder, verdict) (P4). Accepts, overrides, coefficient edits and `confirmed.yaml` changes are journal lines too, and events in `data/workbench/audit.jsonl`.
