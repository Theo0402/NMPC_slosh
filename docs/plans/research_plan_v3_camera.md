# Fast transport of an open cup of water — research plan v3 (camera in the loop)

Step 1 → IROS 2027 (confirm the deadline on the official site). Step 2 → ICRA 2028 or RA-L (deadline **[unverified]**). One person.

v3 replaces the wrist force/torque + IMU design of v1 with a camera that looks straight down into the cup, and the Franka Panda with a Han's Robot Elfin E03-Pro. `README.md` lists what changed, says which document wins when two disagree, and defines the frames, symbols and units used here. The companion software plan is `codebase_plan_v3_camera.md`.

**Markers.** **[unverified]**: a number or fact not checked or not yet measured. Values computed from a model or a datasheet carry it until they are measured. **[target]**: an expected outcome, not a result. **[MISSING]**: no source states it; the text says how to measure it. **[UNTESTED]**, **[UNCLEAR]** and **[E03]** are carried over from the Elfin knowledge files: not tried on hardware; sources ambiguous or contradictory; found in the lab's earlier records of this same arm (the factory inspection, the MATLAB scripts, the ROS driver config): likely to hold, but confirm, since settings and firmware may have changed. Elfin facts cite the knowledge file and the manual page it cites, e.g. (elfin_commands.md, PROTO p.172).

---

## 0. Problem, decisions, assumptions, departures, riskiest assumption, scope

### The problem in three sentences

An Elfin E03-Pro must carry an open, partially filled cup along agile paths as fast as its actuators allow without spilling, and slosh-free methods leave speed on the table: the wrist's rotation rate caps the usable jerk, and latency and tracking error excite sloshing the planner never budgeted. Step 1 closes a loop on the liquid. A camera on the cup holder looks straight down into the cup and measures the free surface's elevation at the wall all the way round. A short shake at pickup identifies this cup's first sloshing mode with the same camera. An estimator tracks the residual slosh from camera frames, predicting between frames and across the image-processing and actuation delays. A correction layered on the slosh-free tracker then damps it, so the arm runs closer to its limits at equal spill. Step 2 reuses every component and replaces the slosh-free constraint with a rim-clearance budget on a nonlinear curved-surface model with cup orientation as a decision variable; whether step 2 is worth doing on this arm is settled by the bottleneck test, not by argument.

### Decisions already made (stated, not argued here)

| Item | Decision |
|---|---|
| Arm | Han's Robot Elfin E03-Pro, 6 joints: the lab's arm, serial MH217340001, with a Mini Cabinet HRC-E0610-A3 (elfin_overview.md, QC p.1) |
| Arm interface | Han's V5 TCP protocol: commands on port 10003, state stream on port 10004 (elfin_overview.md, PROTO p.7). Motion is streamed with `StartServo` + `PushServoJ` (joint positions in degrees; elfin_commands.md, PROTO p.172–173), called **path A**. If path A cannot meet the timing, the pivot is the vendor's ROS EtherCAT driver (**path B**; elfin_overview.md) |
| Liquid sensor | Camera in the loop: Hikrobot MV-CS016-10UM with a Kowa LM4NCL 3.5 mm lens on the Slim Cup Holder Rev F (F.1 flange), looking straight down the cup axis |
| What the camera measures | The wall contact line, i.e. the wall elevation η(θ) all the way round the inner wall. It must work with **clear water**; dyed water is the contrast reference and the last-resort fallback |
| Contact-line front-end (owner amendment, hardware phase) | Classical edge detection along the ray table in clear water (§2.3) is the default. Only if it fails the kill-test-2 gate (§7.1) does a learned front-end, a small 1D CNN per ray, replace the edge step, with the same output. Fallback order: classical/clear → CNN/clear → classical/dyed (last resort). See "Amendment" at the end of this section |
| IMU | None. Effective gravity comes from robot kinematics: the commanded trajectory through forward kinematics, delayed and shaped by the identified servo response |
| Ground truth | The in-loop camera only. Spilled mass from a 0.01 g scale; rim electrodes are optional (binary rim-wet flag) |
| Software | One C++ real-time process; Python for analysis, identification fits, simulation and code generation |
| Cups | One cup for now: inner radius R = 40 mm |
| Schedule | Week numbers only, no calendar dates |
| Wrist F/T sensor | Parked: not bought, not built, kept as a named pivot |

### Assumptions for everything else

| Item | Assumed | Consequence if wrong |
|---|---|---|
| Elfin limits | Placeholders from the protocol's ranges: joint velocity 0–200 °/s, joint acceleration 0–720 °/s², linear velocity 0–3000 mm/s, linear acceleration 0–2500 mm/s². These "depend on the model" (elfin_commands.md, PROTO p.49–52). The manual's read-back examples are 170/180 °/s and 360 °/s², and they don't say which model they come from **[unverified]**. Whether any limit applies to servo streams **[MISSING]** (elfin_overview.md) | Week 1 reads the real values with `ReadJointMaxVel`, `ReadJointMaxAcc` and `ReadLinearMaxVel` (PROTO p.56–59). If linear acceleration really caps near 2.5 m/s², the "fast" regime and kill test 1 change (R10) |
| Servo streaming | The manual's only example is a 0.2 s period with 0.02 s lookahead. No minimum period, lookahead semantics, way to leave servo mode, or late-push behaviour is documented **[MISSING]**. One message at a time: each push must be answered before the next (elfin_commands.md, PROTO p.1, p.172–175) | The corrector needs a push period T_s ≤ 20 ms **[target]** (§1). Found in week 1 by the servo ladder (§9); R3 |
| Robot state | Datasheet JSON on port 10004 at 20 Hz (10005/10006 at 10/5 Hz): joint position, velocity, acceleration and current, with no timestamp, so packets are stamped on arrival (elfin_commands.md, PROTO p.7, p.185–188) | Too slow to see the arm's motion in the slosh band directly; actual motion is measured with the camera on a static target (§4) |
| Arm records | The lab's records describe this unit (elfin_overview.md): IP 192.168.0.10 in the lab's scripts (MATLAB Connect_robotarm.m:3); controller 5.51.5_EPro4ms.beta.20230522 at inspection (QC p.2–3); an E03 STEP model (`HansRobot_E03 3kg 590_总线末端`); and the ROS driver's E03 encoder zero counts, equal to the inspection report's origin counts (QC p.3). The CAD file name suggests a 3 kg payload, ≈ 590 mm reach and the bus-end variant **[unverified]**. The E03-Pro's payload, reach, joint-speed and torque ratings are not in the attachments **[MISSING]**. Holder, cup and water come to ≈ 0.8–1 kg, about a third of 3 kg | Week 1: controller version on the pendant (it may have changed since inspection; LINUX p.19); `ReadRobotModel` (the manual's example reply `5KG` is not expected; PROTO p.16); payload and reach from the vendor's E03-Pro datasheet; check that kill test 1's paths fit the workspace with the holder fitted |
| Kinematic model | The E03's kinematic model, from the E03 STEP model in the attachments or from an E03 description in the vendor's ROS driver if it has one; the knowledge files mention only the `elfin5` URDF **[MISSING]**. Joint ranges disagree between sources **[UNCLEAR]** (elfin_overview.md) | Record ~20 `ReadActPos` poses (elfin_commands.md, PROTO p.66) during week-1 bring-up, on the bare flange with zero TCP and user frame, and compare them with FK in week 2: positions first, orientations once the Rx, Ry, Rz convention is established (the knowledge files don't state it) |
| Camera | Hikrobot MV-CS016-10UM, from the MaxxVision datasheet page: Sony IMX273, 1/2.9″, global shutter, mono, 1440 × 1080, 3.45 µm pixels; ≤ 249.1 fps at full frame in Mono8; exposure ≥ 15 µs in standard mode; gain 0–17 dB; USB3 Vision / GenICam; 6-pin Hirose with one opto-isolated input, one opto-isolated output and one non-isolated GPIO; free-run, pulse-edge and pulse-width triggering; embedded timestamp, frame number and trigger counter; 29 × 29 × 30 mm; ≈ 80 g. **[MISSING]**, each measured in week 1 (§4, kill test 2): which event the timestamp latches (exposure start, end or readout), its clock rate and drift, the frame rate at a reduced ROI, and the exposure-to-host latency | — |
| Lens | Kowa LM4NCL. From the Kowa product page: 3.5 mm, F1.4–16, 1/1.8″, −28 % TV distortion, focus 0.2 m to infinity, Ø31 × 30.5 mm, 60 g, 100 lp/mm at the centre and 60 lp/mm at the margin, back focus 8.9 mm in air. From a Kowa datasheet: focal length ±5 %, front clear aperture Ø19.6 mm, flange focal length 17.526 mm in air, and an angle-of-view table for the 1/2″ and 1/3″ formats. That datasheet also lists 73 g, 1/2″ and 1/3″ formats and 40 lp/mm at the corner, so it may be an older edition **[UNCLEAR]**: check the clear aperture and the field on the lens in hand. On the IMX273, computed from the datasheet's angle table **[unverified]**: field 80° × 60° × 101°, distortion −7 % at the short image edge, −15 % at the long edge, −26 % in the corners. Needs a 0.1 mm C-mount shim to focus at ~110 mm (thin-lens estimate) **[unverified]** | Calibration is mandatory before any image is used (§3.1) |
| Holder | Slim Cup Holder Rev F with the F.1 flange (`README.md`, Sources): V palm with silicone pads, knob clamp, cup-specific yoke, rim tab that sets the cup height. The design puts the lens front 88.5 mm above the rim with the camera's long image axis along J6; flange interface ISO 9409-1-50-4-M6 from the E03-Pro flange drawing. Model estimates **[unverified]**: field at the rim ≈ 148 × 102 mm; holder ≈ 0.44 kg including camera, lens and hardware, with centre of mass (42, 1, 107) mm in F; spine first bending mode ≈ 196 Hz fore–aft and 137 Hz sideways | — |
| Cup | Clear acrylic cylinder, R = 40 mm inner radius, 100 mm inner height; water at 40/60/80 % (h = 40/60/80 mm, freeboard 60/40/20 mm). Wall thickness t_w and outer diameter **[MISSING]**. Rev F yokes exist for Ø70/80/90 outer; this cup needs one for Ø(80 + 2t_w), and the V palm then puts the camera axis √2·t_w off the cup axis, which the calibration absorbs | — |
| Lighting and background | **[MISSING]**. Chosen in kill test 2 from: diffuse light from above with a light sleeve round the cup; a patterned sleeve (clear water); dyed water as the reference | — |
| Compute | One workstation with a PREEMPT_RT kernel **[unverified]**; GPU only for the particle validator | — |
| Simulators | None on hand. The reduced-model simulator is written in Python/CasADi (the NMPC needs it anyway); Genesis or Taichi-MPM is installed in month 2 of step 2, not before | — |
| NMPC first, RL stretch | Confirmed | — |

### Where this plan departs from v1 (argued, not silent)

1. **Camera in the loop; the F/T sensor parked.** A camera above the cup measures the free surface directly, so there is no rigid inertial wrench to subtract. v1's riskiest assumption, that a 0.03 N slosh wrench can be separated from a 3 N rigid wrench during motion, goes away with the sensor. What is lost: the modal mass m₁ (nothing measures force) and the "through the robot's own sensor" framing.
2. **Measure the wall contact line.** The rim constraint is about wall elevation, and the contact line is the wall elevation, measured round the whole circumference. Perspective turns height into image radius: about 1.8, 2.3 and 3.1 px per mm of elevation at 40, 60 and 80 % fill (computed from the Rev F geometry and the lens's angle table) **[unverified]**. Clear water is a requirement; dye is a contrast reference, not the plan.
3. **No IMU.** Without the F/T sensor there is no IMU to come with it. γ is computed from the commanded trajectory and an identified servo response, so tracking error reaches the estimator only through the camera. A forcing-disturbance state absorbs it (§4), and R13 watches it.
4. **Position streaming, not torque control.** The Elfin's controller servos its joints and the PC streams joint positions. The 1 kHz torque tracker and the Franka filter and reflex settings go. The estimator's internal tick stays at 1 kHz **[target]**: it is cheap and keeps the discretization bound of the codebase plan (N2).
5. **Validation against the same camera.** There is no independent instrument. The estimator is validated on prediction, i.e. its τ-ahead prediction scored against frames that arrive later, and on filter consistency, not on agreement with a second sensor (§4, §7).
6. **Kill-test order and gating kept from v1.** Test 2 is now the camera test. It needs the holder printed and the lens fitted, and it runs in weeks 3–4.
7. **One cup.** v1's second cup (R = 55 mm) existed to buy 1.9× F/T signal, and it does not fit Rev F. A second geometry comes back later on a Rev F yoke (Ø70 or Ø90 outer).

Kept from v1, unchanged in substance:
8. **Model A keeps the frame-kinematic and vertical-gravity terms.** The fixed-frequency oscillator is a diagnostic, not the fallback of record. Under slosh-free lean at 5 m/s² the effective vertical gravity is 11.0 m/s², a +6 % frequency shift; over one second at 3.4 Hz that is ~70° of phase.
9. **The Emergency-Stopping paper (Hynninen & Kyrki 2026)** is both step 2's nearest baseline and the nearest scooping threat. Re-implement its constraint in the same solver, and post step 1 to arXiv on the submission day.

### The riskiest assumption (front-loaded)

**That the actuation path is fast enough.**
- **The budget.** The corrector needs the whole loop to stay below ω₀·τ = 60° (≈ 49 ms at 3.37 Hz), or the model-based predictor to bridge the rest. The loop is exposure, transfer, image processing, the wait for the next push, the controller's lookahead and servo, then the joints.
- **The camera half** is measurable and largely compensated, because every frame carries its exposure time (§4).
- **The arm half is undocumented.** The manual's only servo example uses a 0.2 s period, two thirds of a mode period, and the lookahead's effect on delay is not stated.
- **Mitigation.** The week-1 servo ladder measures it before anything is built on it. If it fails, the corrector falls back to option C or to correcting only during hold and deceleration phases, or the arm interface pivots to path B (R3).

**Second: that the contact line measures the wall elevation faithfully, in clear water, during motion.** Four things can break it, and kill test 2 measures each one:
- A contact line on acrylic can stay pinned at small amplitudes; the resulting dead band sets the smallest residual the loop can see (R1).
- The meniscus adds an offset that depends on the contact angle.
- Clear water gives little contrast. The fallbacks, in order: the learned front-end on clear water (§2.3), then dye.
- Motion blurs the line.

The learned front-end can help only with the contrast, reflection and blur items. Pinning and the meniscus are physics: no detector removes them, so the dead band and the meniscus offset are measured the same way whichever front-end runs.

Third, kept from v1: residual slosh in a well-tuned slosh-free baseline may be too small to matter at speeds where nothing else saturates. Kill test 3 measures it, and the evaluation pushes speed until the baseline spills, so the loop's value is "how much further", not "whether".

### Out of scope (both steps)

Non-Newtonian, foaming or opaque liquids (clear and dyed water only); non-axisymmetric containers as a claim; grasp planning, cup detection, collision avoidance and clutter; human handover; bimanual; learned image models in the loop, **except** the contact-line front-end fallback below (owner amendment); learned policies as a contribution (the RL extension is trained on the validated reduced model and reported, if at all, as a stretch result); any new fluid model; sim-to-real as a contribution; container design (it is the pivot if step 2's premise fails).

### Amendment: the learned front-end fallback (owner, hardware phase)

This is the one change the owner made to this section. Nothing else in §0 changes.
- **Before:** "learned image models in the loop (classical image processing only)" was out of scope.
- **Now:** a learned image model is allowed in the loop as the **contact-line front-end fallback**. It may replace only the edge step of the image stage (§2.3). Its output is the classical detector's output: per ray, a sub-pixel edge position and a confidence. The measurement stage, η(θ), the harmonic fit, the estimator and the controllers stay unchanged and do not know which front-end ran.
- **When:** only if the classical detector fails the gate at the end of kill test 2(a)–(c) (§7.1).
- **Fallback order:** classical/clear → CNN/clear → classical/dyed (last resort). Dyed water stays the contrast reference.
- **Still out of scope:** learned models anywhere else in the loop (estimator, controller, the measurement model), a learned image model as a contribution, and learned policies.
- **Ground truth is unchanged:** the in-loop camera only. A side reference camera for training labels would be a further change to this section, which the owner has not made (docs/open_questions.md).
- **Where it lands:** §2.3 (the front-end), §7.1 (the gate), §8 (R1, R16), codebase plan §10 and §12, and `docs/design/cnn_fallback.md`.

---

## 1. Architecture (shared; step-2 additions marked)

Frames (defined in `README.md`): **W** world, the robot base frame; **F** the flange (hand) frame of the holder; **K** the camera frame; **C** the cup frame, with origin at the pendulum hinge **P** on the cup axis and `z_C` pointing out of the cup; image coordinates `(u, v)` in pixels. v1's sensor frame S and IMU frame I no longer exist.

| Block | Rate | In → Out | Step 2 |
|---|---|---|---|
| Reference generator | offline / on demand | path family + speed factor + fill → `p_d, v_d, a_d, j_d` of P in W, trial metadata | The timing law becomes the NMPC's progress variable; the path stays |
| Tilt law | 1 kHz tick, precomputed | `a_d` → `R_d, ω_d, α_d`: `z_C ∥ (a_d − g_W)`, yaw by minimal rotation, then rate-limited to the wrist budget. The ideal-minus-realizable difference is logged: it is the budgeted excitation | Absorbed: orientation is a decision variable |
| Tracker | 1 kHz tick; a push every T_s | `p_d + Δp, R_d` → IK on 6 joints (Pinocchio) → `q_cmd` → `PushServoJ` in degrees every T_s. The Han's controller interpolates (lookahead T_la) and servos the joints; that behaviour is **[MISSING]** and is identified in week 1–2 | Streams the NMPC's `q` reference |
| Camera | f_cam ≤ 249.1 fps at full frame (datasheet); 200 fps **[target]** | Exposure t_exp ≤ 1 ms **[target]**; free-run or hardware-triggered on the opto-input (decided in week 1) | Unchanged |
| Front-end, image stage | per frame | raw frame + ray table from the calibration → rim circle, contact-line points in N_sec sectors (N_sec = 32 **[target]**), quality flags | Unchanged |
| Front-end, measurement stage | per frame | features + calibration → wall elevation `η_k` per sector → harmonics `(C₀, C₁, S₁, C₂, S₂)` → measurement `(z, H, R_cov, t_valid)`, with `t_valid` the exposure midpoint (§2.3) | Unchanged |
| Kinematic input | 1 kHz tick | `q_cmd` history → FK → `a_P, R_WC, ω, α` → `γ_cmd`, delayed by τ_a and shaped by the identified servo response | Unchanged |
| Identifier | per trial at pickup (≈ 3 s) | wall elevation during the shake → `ω₀, ζ, h, m_ℓ` (+ `p_h`, optional) + intervals (§3) | Loads per-cup backbone/swirl coefficients from the offline library |
| Estimator | 1 kHz tick; update per frame | `γ_cmd, ω, α`, delayed frame measurements, parameters → `ŝ, ṡ̂, P`, crest `η̂ ± σ`, τ-ahead prediction (§4) | Also supplies the NMPC's initial liquid state and covariance each cycle |
| Corrector | 1 kHz tick | `ŝ(t+τ_a), ṡ̂(t+τ_a)` → `Δa` in the cup-horizontal plane → `Δp, Δv` offsets to the tracker, never to the tilt law (§5.1) | Absorbed into the NMPC |
| NMPC | — | — | 50–100 Hz acados RTI on `(q, q̇, q̈, liquid)`; rim-clearance constraint; solver monitor; fallback = last feasible plan + decelerate (§5.2). Output streamed through the tracker at T_s |
| Safety monitor | 1 kHz tick | `η̂, P`, frame status, Elfin replies and state → hold or decelerate (by streaming) when `η̂ + 3σ > freeboard − margin`; open loop on frame dropout; stop on an Elfin error | Unchanged |
| Spill and rim | per trial / event | 0.01 g scale (spilled mass); optional rim electrodes (rim-wet flag) | Unchanged |
| Logger | native rates, host monotonic clock | everything, including frame metadata, image features and the ROI images (codebase plan §5) | Unchanged |

**Latency budget.** All values are to be measured in weeks 1–3; none is measured yet. τ_total is the delay from the exposure midpoint to the joints moving.

| Segment | Expected | Status and how it is measured |
|---|---|---|
| Exposure midpoint, t_exp/2 | ≤ 0.5 ms | **[target]**; needs enough light at f/4–f/5.6 (kill test 2) |
| Readout + USB3 transfer of a full 1440 × 1080 Mono8 frame (1.56 MB; 311 MB/s at 200 fps) | a few ms | **[MISSING]**; LED test: a PC-driven LED in view, LED command time vs the first frame that shows it |
| SDK and driver buffering | — | **[MISSING]**; same test, plus embedded timestamp vs host arrival |
| Image stage (rim + contact line along precomputed rays) | ≤ 3 ms | **[target]**; logged per frame |
| Measurement stage + KF update + re-propagation from the exposure time | < 0.2 ms | **[target]**; logged per tick |
| Wait for the next tick | ≤ 1 ms | by design |
| Wait for the next push slot | ≤ T_s (mean T_s/2) | T_s **[MISSING]**: week-1 servo ladder, starting from the manual's 0.2 s |
| Push send → reply | — | **[MISSING]**; logged for every push |
| Controller lookahead | T_la (manual example 0.02 s) | Its effect on delay **[MISSING]**; identified from commanded vs actual motion |
| Joint servo + mechanics | — | **[MISSING]**; camera-on-target chirp (§4) |

**The rule (kept from v1).** The corrector tolerates ω₀·τ up to ~60° before its damping sign becomes unreliable: τ < 1/(6 f₀) = 50.5, 49.5 and 49.3 ms at 40, 60 and 80 % fill (computed). Above ~25 ms it relies on the model-based prediction (§4). The camera part of τ is known for every frame from its timestamps, so the delayed-measurement update applies each frame at the right time and propagates it to the present with the known inputs. The actuation part, from push to motion, is bridged by the predictor using the model and the known future reference. The gate is therefore on prediction accuracy at the horizon τ_cam + τ_a, not on the raw delay.

**Software.** One C++ process for everything in the loop:
- the control tick, the camera acquisition and image stage, the Elfin command I/O (port 10003), the state-stream reader (port 10004) and the logger;
- the Elfin client is a C++ port of the lab's Python client `elfin_min.py`, which stays as the bring-up tool; the protocol reference is the command table both clients are tested against (codebase plan R-B);
- the camera is driven by the vendor's MVS SDK or another USB3 Vision/GenICam library. Decide in week 1; the requirement is access to the embedded timestamp and frame number.

Python does analysis, identification fits, calibration, the reduced-model simulator and acados code generation (step 2). No ROS in the control path. The structure, schema and rules are in the codebase plan.

**Data flow in one line.** Reference → tilt law → tracker (IK → `PushServoJ`) → Han's controller → arm; camera → image stage → measurement stage → estimator (+ commanded kinematics) → (step 1) corrector → tracker | (step 2) NMPC → tracker; the scale and the electrodes sit outside the loop.

---

## 2. Liquid models

### 2.1 Notation (both steps)

- `γ = R_CW(g_W − a_P)`: effective gravity at the hinge P, in C. `G := −γ_z > 0` (effective gravity along the cup axis). `γ_xy` its lateral part. Slosh-free ⇔ `γ_xy = 0`.
- **From the commanded kinematics** (no IMU): `γ_cmd(t) = R_CW(t)(g_W − a_P(t))`. Here `a_P`, `R_CW`, `ω`, `α` come from forward kinematics of the commanded joint trajectory, evaluated at `t − τ_a` and passed through the identified servo response. The rotation terms enter through the acceleration of the hinge itself.
  - In Rev F the cup axis sits ≈ 72 mm along J6 from the flange face, with the liquid's centre of mass near the J6 axis by design. Rolling about J6 therefore barely moves P.
  - Rotations about J5 and J4 do move it: 10 rad/s² at 0.07 m is 0.7 m/s² of hinge acceleration **[unverified arithmetic on assumed numbers]**. That is why tilting the cup about the flange is itself a slosh excitation.
  - The commanded value misses the arm's tracking error; §4 adds a disturbance state for it.
- `ω, α`: cup angular velocity and acceleration in C (commanded kinematics). `J = [[0, −1], [1, 0]]`.
- `θ`: angle round the cup axis in C, measured from `x_C` toward `y_C`; `e(θ) = (cos θ, sin θ)`.
- Cylinder constants: `ξ₁ = 1.8412` (first zero of `J₁'`), `J₁(ξ₁) = 0.5819`, `J₀(ξ₁) = 0.3161`; second antisymmetric mode `ξ₂ = 5.331`, `J₁(ξ₂) = −0.346`.

### 2.2 Model A (step 1): the first antisymmetric mode as a pendulum in the cup frame

**Parameters** (radius R, fill height h; sources Abramson 1966, Dodge 2000, Ibrahim 2005; verify the mass fraction against Ibrahim's table before it goes in a paper):

- `ω₀² = (g ξ₁ / R) · tanh(ξ₁ h / R)`
- `L = R / (ξ₁ tanh(ξ₁ h/R)) = g / ω₀²` (pendulum length)
- `m₁ / m_ℓ = 2R tanh(ξ₁ h/R) / (ξ₁ (ξ₁² − 1) h)` (sloshing mass fraction). **Not identified any more** (§3); the theory value is kept for documentation and the parked F/T pivot.
- `p_h`: hinge position on the cup axis (height of P above the inner bottom). Theory gives it as a function of h/R (Dodge 2000, §2). v1 identified it from the moment/force ratio; the camera can only identify it from rotations, or take the theory value (§3).
- `ζ`: identified. Prior from the Mikishev–Dorozhkin viscous estimate as quoted in Ibrahim 2005: ≈ 0.5 % for R = 40 mm, h = 60 mm **[coefficient unverified]**. Real cups with a moving contact line sit at 0.5–2 %, and the contact line is what the camera watches.
- `κ`: wall crest per unit modal displacement for the linear mode shape, `η_wall = κ |s − s*|` for the dynamic part, with `κ = 2 J₁(ξ₁) tanh(ξ₁ h/R) / (J₀(ξ₁)(ξ₁² − 1)) = 1.54 · tanh(ξ₁ h/R)`. Derived from the centre-of-mass equivalence of the pendulum model; check numerically against the mode shape before use.
- `R/L = ξ₁ tanh(ξ₁ h/R)`: wall elevation per unit quasi-static displacement `s*` (a plane; §2.5).

Nominal values, water, primary cup R = 40 mm. The last two columns are computed and **[unverified]**: the contact-line shift uses the Rev F geometry, the LM4NCL's angle table and an on-axis camera; τ₆₀ = 1/(6 f₀).

| fill | h | freeboard | f₀ | L | m_ℓ | m₁ (theory) | κ | R/L | contact-line shift per mm of elevation | τ₆₀ |
|---|---|---|---|---|---|---|---|---|---|---|
| 40 % | 40 mm | 60 mm | 3.30 Hz | 22.8 mm | 201 g | 87 g | 1.46 | 1.75 | 1.8 px | 50.5 ms |
| 60 % | 60 mm | 40 mm | 3.37 Hz | 21.9 mm | 302 g | 91 g | 1.53 | 1.83 | 2.3 px | 49.5 ms |
| 80 % | 80 mm | 20 mm | 3.38 Hz | 21.8 mm | 402 g | 91 g | 1.54 | 1.84 | 3.1 px | 49.3 ms |

**Second geometry, deferred.** The cups that fit Rev F are Ø70 and Ø90 outer; for inner radius ≈ 35 or 45 mm at 60 % fill, f₀ ≈ 3.6 and 3.2 Hz (from the formula above) **[unverified]**.

**Second antisymmetric mode** (ξ₂ = 5.331) at 5.8 Hz on the primary cup carries ~9 % of a tilted plane's wall elevation. It has the same `cos θ` dependence round the wall, so the camera's first harmonic sees it. It is separated in frequency, is part of the measurement noise model in step 1, and becomes a second oscillator only if the residual spectrum shows it.

**Dynamics** (linear time-varying in the lateral state `s ∈ R²`, `s = L·(n_x, n_y)`, valid for `|s|/L ≲ 0.3` and `|ω_x|, |ω_y| ≪ ω₀`):

```
s̈ + 2ζω₀ ṡ + (G/L) s = γ_xy + L(α_y, −α_x) + L ω_z (ω_x, ω_y) − 2 ω_z J ṡ
```

- **Stiffness.** `G/L` is the instantaneous stiffness; `g/L = ω₀²` is only the rest value.
- **The kinematic terms.** The three terms on the right make `s` a cup-frame quantity. A pure rotation of the cup about the hinge moves `s` by `−L·(tilt)` without exciting the mode, which is the pendulum's way of saying "the surface stays level". They come from the commanded kinematics and cost nothing.
- **Equilibrium.** `s* = L γ_xy / G` is the level surface; the residual slosh is `s − s*`.

Full nonlinear form (simulation; the step-2 fallback), `n ∈ S²` the pendulum direction, `ν = n × ṅ`:

```
ṅ = ν × n
ν̇ = n × ( γ/L − α × n − ω × (ω × n) − 2 ω × (ν × n) ) − 2ζω₀ ν
```

**Wrench output: parked.** v1's excess wrench `ΔF = m₁[s̈ + 2ω × ṡ + α × s + ω × (ω × s)]`, `ΔM = p_h × ΔF − m₁ L e_z × γ_xy` stays in the model source so that the F/T pivot remains one file. Nothing in v3's loop uses it.

**Fallback A′.** Fixed stiffness `ω₀²`, kinematic terms dropped. Acceptable only below ~10° of lean and ~0.5 rad/s of wrist rate, i.e. outside this project's regime. Keep it as a diagnostic and as the "naïve estimator" ablation.

**What Model A does not capture.** Crest asymmetry, amplitude-dependent frequency, the true swirl stability boundary (a pendulum does swirl, with the wrong boundary), higher modes, contact-line pinning and the dynamic contact angle.

### 2.3 Camera measurement model (step 1)

**Wall elevation.** The elevation of the free surface at the inner wall, at angle θ in C, relative to the rest level:

```
η(θ, t) = e(θ)ᵀ [ (R/L) s* + κ (s − s*) ] + η₀(t) + η_h(θ, t)
```

- **First term.** The quasi-static level surface is a plane and exact for `s*`; κ is the dynamic first mode's gain (§2.2). This is §2.5 written as a measurement: the plane gain applies to `s*`, the mode-shape gain to `s − s*`.
- **`η₀`.** The axisymmetric part: fill-level error, meniscus offset and the second-order (0,1) mode.
- **`η_h`.** Higher harmonics in θ (the (2,1) pair, second order) plus the second radial mode, which shares the `cos θ` harmonic.
- **Consistency check.** The two ways of relating the static plane to `s` differ by ~1 % (plane: `R/L = 1.84 tanh`; centre-of-mass equivalence: `1.82 tanh`) **[unverified]**. Check numerically against the mode shape, as for κ.

**From image to elevation.** The contact-line point `(R cos θ, R sin θ, z₀ + η)` in C, with z₀ the height of the rest level above P, is mapped to K by the calibrated pose `T_KC`, then to pixels by the calibrated lens model (intrinsics and distortion, §3.1).
- **Image stage.** Finds the contact line along precomputed rays in N_sec sectors.
- **Measurement stage.** Intersects each detected ray with the wall cylinder and returns `η_k`. The contact line is seen through air, so no refraction enters this geometry.
- **The rim as reference.** The rim is a rigid circle of known radius and height in C. It is fitted in every frame and re-registers `T_KC`, which removes holder vibration between camera and cup (spine modes ≈ 140–200 Hz, estimate) and slow creep.
- **Occlusion.** Rev F's view check finds nothing between the lens and the rim except the rim tab. The tab hides a short arc of the back wall when the liquid there comes within a few mm of the rim: 3.6 mm for an Ø80-outer cup **[unverified]**; recompute for the real cup. Those sectors are masked from the calibration, not detected.

**The edge step: classical by default, learned only as the fallback** (owner amendment, §0). Both front-ends sample the same intensity profile along each ray of the ray table, in a window round the expected radius from the fill and meniscus map (§3.1), and both return the same thing per ray: the edge position along the ray in sub-pixel units (hence its `(u, v)`), a confidence in [0, 1] and a valid flag. That is what `cam_feat` logs (codebase plan §5), and it is all the measurement stage reads.
- **Classical (default).** Smoothed derivative along the ray; the strongest edge inside the window; sub-pixel position from a parabola through the derivative peak; confidence from the edge strength over the local noise. Bias < 0.05 px on synthetic images **[target]** (codebase plan §9).
- **Learned (fallback, only after the classical detector fails the kill-test-2 gate, §7.1).** A 1D CNN per ray, ≤ ~50k parameters **[target]**, on the same profile; the neighbouring rays may be added as input channels. It outputs a heatmap over the window and a "no edge visible" score. The position is a soft-argmax restricted to a few samples round the heatmap's peak, so that a second peak (a reflection, a droplet) lowers the confidence instead of pulling the position halfway to it. Confidence = (1 − the no-edge score) × the heatmap mass near the peak, calibrated on held-out data. Design, labels, training and acceptance: `docs/design/cnn_fallback.md`.
- **Which rays count.** A ray counts when its confidence is at least c_min (0.9 on the calibrated scale **[proposal; set at the gate]**) and it passes the 3σ gate below; masked sectors never count. If fewer than N_valid = 16 of the 32 rays count, or any 90° quadrant has fewer than 2 **[proposal]**, the frame's measurement is flagged invalid. 16 rays raise σ_s by √2 over 32, to ≈ 0.020 mm at 60 % fill (computed from the figures below) **[unverified]**, and the quadrant rule keeps the five-coefficient harmonic fit well conditioned.
- **An invalid frame is a dropout.** The estimator predicts only, and the corrector is disabled as after a dropout (§5.1, item 5), so the controller runs open loop (M3) until valid frames return. Bad vision can remove the correction; it can never enlarge it, because the corrector's authority cap and the tracker's 80 % joint-acceleration rule (§5.1, item 4) bound what any measurement can ask for.
- **What no front-end changes.** Contact-line pinning and the meniscus are physics. The dead band and the meniscus offset are measured the same way whichever front-end runs, and the learned front-end's labels must never come from a model of how the line *should* move inside the dead band (it would learn to hide the dead band).

**What the estimator receives.** The sector elevations are fitted by least squares over the valid sectors (masked and gated sectors removed) as `η_k ≈ C₀ + C₁ cos θ_k + S₁ sin θ_k + C₂ cos 2θ_k + S₂ sin 2θ_k`. The estimator receives the first harmonic:

```
z = (C₁, S₁) = κ s + (R/L − κ) s*(γ_cmd) + b_z + v
```

- **Logged with it:** `C₀, C₂, S₂`, used for the fill level (§3), Model B's second-order terms and kill test 5.
- **Alternative:** a sector-level `z` (n = N_sec rows) fits the same interface, if the harmonic fit turns out to hide useful structure.

**Noise model.** `v = v_edge + v_blur + v_model`, plus systematic terms.
- **Edge localisation.** σ_edge ≤ 0.2 px per sector point **[target]**. That gives σ_η ≈ 0.08 mm per sector and σ_s ≈ 0.014 mm per frame at 60 % fill with 32 sectors (computed) **[unverified]**. For comparison, v1 needed the F/T sensor to resolve ~0.02 N ≈ 0.7 mm of crest. In practice the floor is set by the systematic terms below, not by pixels.
- **Blur.** Contact-line speed × t_exp. A 10 mm crest at 3.37 Hz moves the line at ≈ 0.21 m/s, which is 0.5 px of blur in 1 ms at 60 % fill (computed) **[unverified]**.
- **Model error.** Higher modes and nonlinearity, growing roughly with amplitude² (kill test 5).
- **Systematic terms:**
  - the meniscus offset, calibrated at rest per cup and fill;
  - contact-line hysteresis, a dead band in amplitude (kill test 2);
  - in clear water, the meniscus band below the line, which is seen through water and can pull the detected edge.
- **Covariance.** `R(t) = R₀ + R₁ |ṡ̂|² t_exp² + R₂ |ŝ − s*|⁴`. This is the form only; the coefficients come from kill tests 2 and 5 **[unverified]**, and they are fitted separately for each front-end and lighting set-up (a learned front-end does not inherit the classical coefficients).
- **Gating** at 3σ rejects splashes, droplets on the wall and reflections.

**Rate.** One measurement per frame at f_cam. The estimator predicts at the tick between frames.

### 2.4 Model B (step 2): nonlinear curved free surface, single-dominant multimodal system

**Source.** The Narimanov–Moiseev single-dominant asymptotic modal system for an upright circular cylinder: Faltinsen & Timokha, *Sloshing*, CUP 2009, ch. 9; Lukovsky, *Nonlinear Dynamics: Mathematical Models for Rigid Bodies with a Liquid*, De Gruyter 2015; Raynovskyy & Timokha, *Sloshing in Upright Circular Containers*, CRC 2020. Chapter and equation numbers to be pinned when the books are on the desk.

**Structure.**
- **Unknowns.** The two first-mode generalized coordinates `(a, b)` for the `cos θ` and `sin θ` modes.
- **Slaved modes.** The second-order modes, axisymmetric (0,1) and the (2,1) pair, are slaved and quadratic in `(a, b, ȧ, ḃ)`.
- **Result.** A pair of second-order ODEs, cubic in `(a, b)`, whose coupling coefficients depend on h/R only (they are integrals of the linear eigenfunctions; the references tabulate them).
- **Forcing.** Lateral acceleration `γ_xy`, and angular motion via the Stokes–Joukowski potential.

**Our modifications** (validated in kill tests 4–5, not claimed as model contributions):
1. `g → G(t)` in the linear stiffness. This is legitimate because the cubic coefficients are geometric and `g` enters only the restoring term; confirm it on the derivation's potential-energy term before relying on it.
2. The frame-kinematic terms of Model A added consistently. This approximation is valid for `|ω|/ω₀ ≲ 0.15`.
3. Linear damping `2ζω₀` on `(ȧ, ḃ)` with the identified ζ.

Validity: amplitudes below ~0.3 R and depths away from the critical depth (h/R ≳ 0.6 **[unverified threshold]**; ours are 1–2).

**Output.** `η(R, θ, t) = J₁(ξ₁)(a cos θ + b sin θ) + β₀(a,b,ȧ,ḃ) + β₂(·) cos 2θ + β₂′(·) sin 2θ + O(3)`. The camera measures exactly this function at the wall, so Model B can be checked harmonic by harmonic: `β₀ ↔ C₀`, `(β₂, β₂′) ↔ (C₂, S₂)`. The rim constraint uses a smooth upper bound on the maximum over θ:

```
η̄ = J₁(ξ₁)·sqrt(a² + b² + ε²) + c_a(h/R)(a² + b²) + c_v(h/R)(ȧ² + ḃ²)/ω₀²
```

`c_a` and `c_v` are collected from the slaved-mode expressions (a conservative combination of the `2θ` and axisymmetric terms). This is the quantity kill test 5 checks. If the measured crest tracks the linear first-harmonic prediction up to the rim, `c_a = c_v = 0` and Model B is dropped from the paper with a sentence saying so.

**Mapping to Model A's state** (hand-over from the step-1 estimator): `s = χ·(a, b)` with `χ = J₀(ξ₁)(ξ₁² − 1) / (2 tanh(ξ₁ h/R)) = 0.378 / tanh(ξ₁ h/R)`.

**Fallback B′.** Model A's nonlinear pendulum with an amplitude-dependent stiffness `ω₀²(1 − μ ρ²/R²)`, `ρ = |s − s*|`, with μ fitted from the measured backbone. The crest map is `η_wall = κ ρ (1 + c_nl ρ/R)` with `c_nl` fitted from kill test 5. Swirl: the spherical pendulum's conical-motion boundary (Miles, *Physica D* 1984) will not match the fluid's; report the mismatch and the margin over it.

### 2.5 Why a plane is not enough

A plane matched to the surface slope at the centre over-predicts the first mode's wall elevation by `ξ₁/2 ÷ J₁(ξ₁) = 0.92/0.58 = 1.58×`. The linear mode shape under-predicts the crest at large amplitude, because nonlinear crests are higher than troughs. The plane is exactly right for the quasi-static part `s*` and exactly wrong for the dynamic part `s − s*`, and the rim constraint depends on their sum. The camera measurement inherits this: `z` uses the plane gain `R/L` for `s*` and the mode gain κ for `s − s*`.

---

## 3. Identification protocol

### 3.1 Once per cup and lens setting (offline, ~2 h)

1. **Camera calibration.** Done after focus and iris are locked and the shim is fitted; any later change to them means calibrating again.
   - **Intrinsics and distortion.** Use a ChArUco board (corners are still found when the board leaves the frame, which is how the image corners get covered), about 120 × 90 mm with 10 mm squares, flat and rigid, square size measured with calipers. Take 30–50 images between 88 and 160 mm from the lens front, tilted up to ~40°, covering every part of the image. Fit OpenCV's fisheye model and its rational model, and keep the one with the lower error on held-out images. Target: RMS reprojection error < 0.3 px **[target]**, with corner residuals not much worse than the centre's.
   - **Cup in the camera frame.** A flat printed target resting on the rim tab at rim height gives `T_KC`: the rim plane and the cup axis in K. The rim's image becomes the per-frame reference.
   - **Ray table.** Precomputed pixel coordinates along the measurement rays, N_sec sectors × M samples, from the lens model. Built once per calibration.
   - **Calibration record.** Gets an ID and a hash, and every trial header carries both (codebase plan §5).
2. **Meniscus offset and fill-level map.** Fill the cup to 3–5 levels and record the contact line at rest. The owner measures each fill height directly and enters it (owner decision, hardware phase: not by weight); read the flat surface, not the meniscus at the wall. This gives the offset and scale of the camera's fill estimate per fill. Check it against the measured fill: h ± 0.5 mm **[target]**, which now also depends on how well h is read by hand **[unverified]**.
3. **Payload for the arm's controller.** Weigh holder + cup (+ water at the nominal fill); take the centre of mass from the CAD model and enter it on the pendant.
   - `SetPayload,nRbtID,dMass,dX,dY,dZ,;` takes kg and mm; the offset range is printed as ≥ 0 (elfin_commands.md, PROTO p.48).
   - Settings sent over TCP revert at controller restart, while settings made on the pendant are saved (elfin_overview.md, PROTO p.5); payload and TCP must be right before enabling (elfin_overview.md, LINUX p.17).
   - The CAD centre of mass is in F; the controller wants it in its own flange frame, whose axis directions are **[MISSING]** (the Elfin drawing gives the bolt pattern, not the axes). Find them in week 1 by jogging along each tool axis from the pendant at its slowest jog speed and noting which way the holder moves. If a component comes out negative, test whether the printed ≥ 0 range is enforced.
   - TCP: the cup axis at rim height, so that the controller's TCP-based safety limits watch the cup **[decision for week 1]**.
   - v1's rigid-payload identification and liquid-inertia fit are dropped: nothing subtracts a rigid wrench any more.
4. **Step 2 only, from the kill-test-4/5 session.** Backbone (peak-response frequency vs amplitude) at 3–4 forcing amplitudes, from the camera's first-harmonic amplitude, gives one cubic coefficient, checked against the theoretical value. The planar→swirl transition frequency and amplitude, from the first harmonic's phase rotating, are checked against the theory's prediction with the identified ζ. There is no separate swirl parameter to fit: if the prediction is off, ζ and the cubic coefficient are re-fitted within their intervals and the misfit reported.

### 3.2 At every pickup (online, ≈ 3 s; a 1 s version exists at the cost of ζ accuracy)

| Phase | Motion | Signal used | Yields |
|---|---|---|---|
| Hold, 0.5 s | still | contact line at rest; the mean wall level `C₀` through the calibrated geometry | `h`, then `m_ℓ = ρ_w π R² h` (ρ_w the density of water); L, κ, χ and the theory values of `p_h` and m₁ from h/R; `b_z` initialized (`s = s*` is known at rest) |
| Kick x | lateral displacement of P along `x_C`, 3–5 mm in 0.10–0.15 s (cosine ramp), streamed at T_s **[target]**. If T_s is too long to shape it, a controller-planned linear move of the same displacement, with spectral content at 3.3–3.4 Hz | `γ_cmd`; wall elevation | forced response; no m₁, because the camera measures displacement, not force |
| Hold, 1.0–1.5 s | still | first-harmonic `(C₁, S₁)` ring-down at f_cam | `ω_damped, σ` by matrix-pencil initialization + nonlinear LS on `A e^{−σt} cos(ω_damped t + φ) + k₀ + k₁ t` (k₀, k₁ an offset and a drift); `ω₀ = sqrt(ω_damped² + σ²)`, `ζ = σ/ω₀`. The fit window ends where the amplitude reaches the contact-line dead band (kill test 2) |
| Kick y + hold | as above, along `y_C` | as above | isotropy check: a circular cup must agree to < 1 %; disagreement flags a non-circular or off-centre cup, or a calibration error |
| Tilt kick (optional) | rotation of ±2–3° about a joint axis at known distance from P | ring-down amplitude vs commanded α | `p_h`. A rotation about the hinge itself excites nothing; the response grows with the hinge-to-axis distance **[unverified accuracy]** |
| Consistency | — | ω₀ vs the dispersion relation at the camera-measured h | mismatch > 3 % → widen intervals, flag the trial (wrong R, foam, non-water, bad calibration) |

**What the camera can and cannot identify.**
- **Directly:** `f₀` (ω₀) and ζ from the ring-down; h and `m_ℓ` from the rest level, after the once-per-cup meniscus calibration; isotropy.
- **Not at all: m₁.** The modal mass relates surface motion to force, and no force is measured. Nothing in the camera loop uses it; it scaled the wrench. The theory value stays in the parameter set for documentation and for the parked F/T pivot.
- **Only differently: `p_h`.** v1 read it from the moment/force ratio. The camera can read it only from the response to rotations (tilt kick), or use the theory value. The estimator needs it only through γ at P under rotation, so a ±10 mm error is a small forcing error **[unverified]**; quantify it in L1.

**Expected accuracy** **[unverified]**: f₀ ± 0.3 %; ζ ± 30 % from 1 s, ± 15 % from 3 s, and amplitude-dependent because contact-line damping is; h ± 0.5 mm; `p_h` ± 10 mm (tilt kick or theory). Damping accuracy matters least: ζ enters the corrector only as a small offset to the added damping, and the estimator corrects amplitude drift from measurements.

**Intervals passed downstream**: ω₀ ± 1 %, ζ ± 30 %, h ± 1 mm, cubic coefficient ± 20 %. These are what the margin in §5.2 is built from.

**What is novel here and what is not.** Matl et al. (IROS 2019) read mass and volume from the wrist at rest. This reads the dynamic mode (frequency, damping) and the fill visually, with the same camera that closes the loop. Whether earlier work identifies a sloshing mode from a camera on a manipulator is **[unverified]**; it is part of the week-1 prior-art search (§9). Do not claim more than that.

---

## 4. Estimator

**Choice.** A linear time-varying Kalman filter, not an EKF: Model A is linear in the state given the inputs, and the parameters come from the shake. Three options were considered:
1. an LTV-KF with fixed parameters and disturbance states;
2. an EKF that also estimates `ω₀, ζ` online;
3. a bank of 3–5 KFs over the parameter interval with likelihood weighting.

(2) risks divergence exactly when the arm is aggressive. (3) is trivially cheap at this size and is the upgrade if the residual monitor shows frequency mismatch on long paths. Start with (1).

**State** `x = [s (2), ṡ (2), d (2), b_z (2), δ (1, optional)]`:
- `s, ṡ`: modal displacement and velocity.
- `d`: a forcing-disturbance random walk on the right-hand side of §2.2, i.e. the difference between commanded and actual effective gravity. This is the arm's tracking error, which nothing else measures without an IMU.
- `b_z`: a measurement-bias random walk (camera-to-cup registration, meniscus asymmetry).
- `δ`: optionally, a slow stiffness drift (only if the residual monitor asks for it).

v1's force bias `b` and motion-correlated gains `k` modelled the rigid-wrench error and are gone with it. The state dimension stays at 8–9.

**Process.** The LTV equation of §2.2 with inputs `(γ_cmd, α, ω)` from the commanded kinematics, delayed by τ_a and shaped by the identified servo response, plus `d`. It is discretized at the 1 kHz tick **[target]** with the second-order series of the codebase plan (N2). Process noise on `ṡ` is sized from two sources: the budgeted-excitation log (the tilt law's ideal-minus-realizable difference), so the filter expects excitation where the plan predicts it; and the residual of the servo model (weeks 1–2), so it expects tracking error where the arm produces it.

**Measurements.** One per frame, as in §2.3: `z = H x + D_z s*(γ_cmd) + v` with `H = [κ I, 0, 0, I, 0]` (a zero column for the optional δ) and direct feedthrough `D_z = (R/L − κ) I`, evaluated at the frame's `t_e`. `R(t)` and the 3σ gating are as in §2.3. The codebase plan's `Measurement` struct carries `z`, `H`, `D_z` and the base of `R`.

**Timing: delayed measurements.**
- **When a frame is valid.** At its exposure midpoint `t_e`: the camera timestamp, mapped to the host clock by a per-trial offset-and-drift fit.
- **When it arrives.** τ_cam later: ≈ 5–10 ms **[target]**.
- **History.** The filter keeps a fixed-capacity history of state, covariance and inputs covering ≥ 100 ms **[target]**.
- **Update.** When a frame's measurement arrives, the filter updates at `t_e` and re-propagates to the current tick. This is deterministic given the logged inputs and the consumed frame IDs (codebase plan §7).
- **Between frames: predict-only.** At 200 fps and a 1 kHz tick, four ticks in five have no measurement. This is the normal path, not an exception.

**Latency offsets.** Each is measured, and each measurement is its own record:
1. **Camera clock to host.** A per-trial fit from (embedded timestamp, host arrival) pairs, offset plus drift. Two independent oscillators drift, so a single number would be wrong by the end of a 30 s trial. The fit is checked by a PC-driven LED in view.
2. **τ_cam.** Per frame, logged.
3. **τ_a and the servo response** (command → motion). Commanded joint positions are compared against (i) the 20 Hz state stream, for slow content, and (ii) the camera itself on a static printed target on the table with the cup removed, for content up to 10–20 Hz. In the second case the image motion is the arm's actual motion, on the same clock as the frames. The fit gives a delay, a lag model and the lookahead's effect.

The corrector receives `x̂(t + τ_a)`, propagated with the model and the known future reference input.

**Outputs.**
- `ŝ, ṡ̂, P`.
- Crest `η̂ = |(R/L) s* + κ (ŝ − s*)|` in step 1, or `η̄(a, b, ȧ, ḃ)` via χ in step 2, with σ from P.
- Modal energy `E = ½(|ṡ|² + ω₀²|s|²)`.
- A residual monitor (whiteness and spectral peak of the innovation) that flags parameter mismatch.

**Where the estimate goes.** Step 1: the corrector (§5.1), the safety monitor and the paper's prediction-vs-frame figure. Step 2: the NMPC's initial liquid state each cycle and the estimation part of the margin (`3σ` of the crest, propagated along the horizon).

**Validation targets** **[target]**. The in-loop camera is the only truth, so the tests are prediction tests and consistency tests, not agreement with a second sensor:
1. **Prediction.** The τ-ahead prediction, with τ = τ_cam + τ_a, scored against the frames that arrive later: crest correlation > 0.9, amplitude error < 20 %, phase error < 20° at the mode.
2. **Held-out frames.** Run on every second frame and score on the others, with the same targets.
3. **Consistency.** The normalized innovation squared stays within its 95 % band on ≥ 90 % of a trial, and the innovation is uncorrelated with the commanded acceleration; if it is correlated, `d` is mis-sized or the servo model is off (R13).

If the amplitude error is systematically off, suspect in this order: κ (the mode shape), then the meniscus and contact-line dead band, then the servo model.

---

## 5. Controllers

### 5.1 Step 1: residual-slosh correction layered on a slosh-free tracker

**The three layers**, from slow to fast:
- the **tilt law**, feedforward computed from the *nominal* `a_d`;
- the **tracker**, IK at the 1 kHz tick with a push every T_s;
- the **corrector**, at the 1 kHz tick, with feedback from the estimator.

The corrector adds damping to the mode through a lateral acceleration in the cup-horizontal plane:

```
Δa = sat( −k_d ṡ̂(t+τ_a) − k_p (ŝ(t+τ_a) − s*(t+τ_a)) ),   k_d = 2 ζ_target ω₀,  ζ_target ∈ [0.2, 0.4],  k_p ≈ 0
Δp̈ = R·[Δa; 0]   integrated to Δp, Δv and added to the streamed reference before IK
```

- **Effect.** With `ζ_target = 0.3` the time constant of the residual drops from ~9 s (ζ = 0.5 %) to 0.16 s; a 5 mm residual (crest ~7 mm) is gone in half a maneuver.
- **Demand.** ±1–2 m/s² at 3.4 Hz: ±2–4 mm of path deviation, ~20–40 m/s³ of jerk, no wrist motion.
- **Streaming requirement.** Streamed as joint positions, the wiggle needs ≥ 10 pushes per mode period, so T_s ≤ 29 ms **[unverified rule of thumb]**.

**Three actuation options and what has to be true for each** (decision: A now; B and C as pivots):

| Option | Shape | What must be true |
|---|---|---|
| A. Lateral-acceleration damping (above) | small bandpassed wiggle in the path | The arm has ≥ 2 m/s² of acceleration headroom in the cup-horizontal plane during the maneuver (kill test 1 logs this against the limits read in week 1); T_s ≤ 20 ms **[target]**; latency ω₀τ < 60° or the predictor works |
| B. Tilt-based damping (modulate `γ_xy` by tilting) | oscillating lean of ±3–5° | The wrist has spare rate, i.e. the translational channel, not the wrist, is what saturates. Unlikely, but exactly right if kill test 1 gives outcome B on some paths |
| C. Model-based counter-kick (a one-period input-shaping burst that drives `s − s*` to zero, re-planned once per estimate) | deadbeat, discrete | Parameters known to ~2 % in ω₀, the arm has jerk headroom mid-maneuver, and the estimate is trustworthy at the moment of the burst. Faster than A but brittle; the natural fallback if T_s or τ_a are too long for A (R3) |

**Why the layers do not fight.**
1. The tilt law never sees `Δa`; if it did, it would tilt to cancel the very lateral gravity the corrector is injecting.
2. `Δa` is added to the streamed reference, so the Han's controller is not asked to fight a disturbance; it is asked to follow a slightly different reference.
3. The estimator's input is the commanded γ, computed from that same corrected reference and delayed by τ_a, so the correction is in the model as long as the servo model holds. The disturbance state `d` and R13 cover the rest.
4. Authority is capped: `|Δa| ≤ 2 m/s²` **[target; re-check against the E03-Pro limits]**. Online, the joint-acceleration references implied by `Δa` must stay below 80 % of the limits read in week 1, or `Δa` is scaled down.
5. The corrector is disabled during the shake and for 30 ms after any frame dropout or rejected frame. It is bandpassed to 0.5–2 ω₀, so bias and slow model error cannot leak into a slow drift of the path.

**Stability margin.** A lightly damped second-order mode with delayed velocity feedback: the added damping is `k_d cos(ω₀ τ)` and the stiffness change is `k_d ω₀ sin(ω₀ τ)`.
- At τ = 20 ms, `ω₀ τ = 24°`: 10 % of the intended damping is lost and the frequency shifts 1 %.
- At 50 ms it is 60°, and the predictor is mandatory.

These figures are for the raw delay τ = τ_cam + τ_a, from a frame's exposure to the joints moving, with no prediction. The estimator bridges the camera part by applying each frame at its exposure time and propagating to the present with the known inputs. The predictor bridges the actuation part (push wait, lookahead, servo and mechanics) with the model and the known future reference. What is left is prediction error, so the gate is prediction accuracy at τ_cam + τ_a (§1, §4). The Han's controller's own tracking dynamics, once identified, are included in the simulation (§6), not neglected.

**Optional second loop (cheap, ablation-worthy if time allows): camera-measured tilt feedback.** The low-passed quasi-static part of the first harmonic estimates `γ_xy/G` directly, because a level surface is perpendicular to effective gravity. Feeding it to the tilt law with a low gain reduces excitation at the source (tracking error and part of the latency) within the wrist's rate limit. It replaces v1's IMU-measured-γ loop. Not a claim; an ablation row.

### 5.2 Step 2: tilt-and-budget NMPC

**Formulation of record** (option 1 below), per RTI cycle, N shooting nodes:

- **Variables**: states `x = (q, q̇, q̈, λ, λ̇, a, b, ȧ, ḃ)` (6+6+6+2+4 = 24), inputs `u = (q⃛, λ̈)` (7), with λ the path-progress variable.
- **Dynamics**: triple integrator in joint space; cup kinematics from `q` (Pinocchio–CasADi frame position, velocity and acceleration of P, rotation, `ω`, `α`); `γ = R^T(g_W − a_P)`; Model B (or B′) driven by `γ, ω, α` with the `G(t)` stiffness; second-order modes algebraic. The Han's controller's identified response (delay and lag from `q_cmd` to `q`) is part of the model, because the NMPC commands it rather than torques.
- **Path**: `p_P(q_k) = Γ(λ_k)` as a soft tube constraint (5 mm, ℓ₁ penalty). The tube is where the tilt–budget blend and the corrective wiggles live.
- **Cost**: maximize progress `λ_N` (minimum-time in path-following form) + a small quadratic jerk penalty + a terminal penalty on modal energy + a weak posture/yaw regularizer to stay away from wrist singularities. No term asks for any orientation.
- **Constraints**:
  - joint velocity, acceleration and jerk at 0.9× the values read in week 1 with `ReadJointMaxVel`, `ReadJointMaxAcc` and `ReadJointMaxJerk` (jerk is read-only; elfin_commands.md, PROTO p.56–58), and joint positions inside the pendant's joint ranges (the protocol has no read command for them; elfin_overview.md, LINUX p.50). Whether the controller enforces any of these on servo streams is **[MISSING]** (elfin_overview.md), so the NMPC enforces them itself;
  - rim clearance `η̄(x_k) ≤ H_c − h − m_k`, with H_c the cup's inner height, so that `H_c − h` is the freeboard (soft, exact ℓ₁ penalty; `m_k` the node-dependent margin below);
  - a terminal free-oscillation crest `κ·sqrt(ρ_N² + (ρ̇_N/ω₀)²) ≤ H_c − h − m_N`, so that the ring-down after the horizon cannot spill;
  - torque: there is no torque interface on path A, and the E03-Pro's torque ratings are **[MISSING]** (elfin_overview.md). Log the joint currents from the state stream (elfin_commands.md, PROTO p.185–188) in kill test 1 and add an inverse-dynamics bound only if they approach the rated values once those are known.
- **Horizon and discretization**: 1.5–2.0 s, N = 75–100 at 20 ms, ERK4 with 4 substeps (5 ms ≈ 60 samples per period, above the 20-per-period rule). For long paths, a receding horizon with the terminal constraint carries the guarantee. The output `q` is streamed through the tracker at T_s.
- **Solver**: acados SQP-RTI with HPIPM, partial condensing (N/4), Levenberg–Marquardt regularization, shifted warm start. The initial state is the streamed `(q_cmd, q̇_cmd, q̈_cmd)` and the estimator's `(ŝ, ṡ̂)` mapped through χ.
- **Real-time budget**: ≤ 10 ms solve at 50 Hz **[unverified]**; benchmark in the first simulation month of step 2. Above 20 ms, use N = 50 at 30 ms nodes with 6 substeps, then option 3.
- **Margin** `m_k = m_model + m_param + m_est,k + m_track`:
  - `m_model` from the kill-test-5 error-vs-amplitude curve at the constraint's operating amplitude;
  - `m_param` from Monte-Carlo over the identified intervals (95th-percentile crest minus nominal over the planned maneuver);
  - `m_est,k` = 3σ of the crest propagated to node k (it grows along the horizon);
  - `m_track` from kill test 3's residual-excitation statistics.

  Expect 3–6 mm of the 20–60 mm freeboard **[target]**. Report the split: it is the paper's argument for why the nonlinear surface and the camera loop matter (they shrink `m_model` and `m_est`). If the widened margin costs more than 25 % of the achievable speed-up, upgrade to a 3-scenario NMPC (nominal, `ω₀ ± interval`) at ~3× solve cost.

**Three formulations and what has to be true** (decision: 1, with 3 as the fallback; condition to change: solve time):

| Option | Shape | What must be true |
|---|---|---|
| 1. Joint-space NMPC (above) | exact joint limits; orientation free by construction | acados solves in ≤ 10–20 ms with Pinocchio–CasADi kinematics at N = 75–100; joint limits, not torque, are what binds |
| 2. Task-space NMPC on the cup pose with proxy limits (a bound on the cup's angular speed ‖ω_C‖, Cartesian bounds) + joint tracker | light model, fast | The proxy never underestimates the real joint demand by more than 20 % along the test paths (check by IK on planned trajectories); otherwise residuals rise and "jointly optimized under wrist-rate limits" weakens |
| 3. Task-space NMPC with joint constraints linearized along the previous RTI solution (`J(q_k)` from the last plan) | middle path; RTI already linearizes | `q` changes by a few percent per cycle, so the linearization holds at 50 Hz |

**Handling swirl.** Nothing special in the controller: Model B contains the planar–swirl coupling, so the constraint on `η̄` (rotationally invariant in `(a, b)`) already covers rotary sloshing. The demonstration is on the circle path at the period kill test 4 identifies. The baselines' pendulum is what fails there.

**Relation to the slosh-free methods.** Setting the budget to zero (`m_k` = freeboard) forces `γ_xy = 0` and recovers the slosh-free condition; the step-1 stack is the zero-budget special case with a fixed tilt law. Say this once in the paper.

**Fallback if the bottleneck test gives outcome B on all paths.** Orientation is fixed upright, the NMPC keeps the rim budget and the nonlinear surface, and the paper's second half becomes container geometry (what freeboard and radius the optimizer wants).

---

## 6. Simulation pipeline and validation plan

**Three layers, one codebase.**

| Layer | Contents | Used for |
|---|---|---|
| L0: reduced models | Models A, A′, B, B′ in CasADi (the same symbolic code the NMPC compiles), and the camera measurement model of §2.3: wall elevation → contact-line image points through the calibrated lens model, with the occlusion mask. Unit tests: energy conserved at ζ = 0; slosh-free input ⇒ `s ≡ s*`; frequency scales with `√G`; χ consistent with the centre-of-mass shift; measurement geometry round trip (η → pixel → η) | Estimator and controller design; the NMPC model; error curves |
| L1: robot inner loop | Pinocchio model of the E03-Pro (source and check **[MISSING]**, §0); the tracker and estimator as implemented (the same C++ through pybind); the Han's servo response (period, lookahead, delay, lag) identified in weeks 1–2; a commanded-vs-actual γ error model from the same identification; the camera chain (exposure, timestamp jitter, τ_cam, dropped frames, blur, edge noise, meniscus offset, the contact-line dead band from kill test 2) | Closed-loop stack tests; latency, servo-period and parameter sweeps; the "unbudgeted excitation" mechanism figure |
| L2: particle validator | Genesis (MPM/SPH on the GPU) or Taichi-MPM, driven kinematically by recorded cup motions; outputs the wall elevation round the cup. ~0.5 mm particle spacing → ~10⁶ particles for the 40 mm cup, hours per maneuver **[unverified]** | Crest beyond the hardware's safe envelope (large amplitude, breaking); a second opinion on crest asymmetry; residual statistics between reduced model and "truth". Never a training environment, never in the loop |

Ranking of truths for the crest: hardware camera (kill test 5) > particle simulator > reduced model. MPM at this scale has no surface tension or contact line, so its millimetre-level crest is not to be trusted over the camera; that is why the definitive error-vs-amplitude curve is a hardware measurement.

**Validation plan.**

1. **Envelope tests** (L0+L1): grid over path family × peak acceleration (up to the E03-Pro limits) × peak jerk × fill 40/60/80 %. Record predicted crest, estimator error, corrector authority used, joint-limit ratios. The envelope is where estimator error < 20 % and the crest stays below the rim with margin.
2. **Error-vs-amplitude curves**:
   - (i) simulation: Model A vs B vs particle, crest vs modal amplitude up to the rim;
   - (ii) hardware (kill test 5): crest (max over θ of η) vs first-harmonic amplitude, both from the camera.

   `m_model` is read off curve (ii) at the operating amplitude. Curve (ii) is a single-sensor result, and the paper says so.
3. **Latency sweep**: total delay to bridge ∈ {20, 40, 60, 80} ms and T_s ∈ {10, 20, 40} ms → residual amplitude after correction, with and without the predictor. Sets the latency and servo-period tolerance the paper reports.
4. **Parameter-error sweep**: ω₀ ± 3 %, ζ ± 50 %, h ± 2 mm → correction degradation. Sets the identification accuracy needed. It is also the figure that justifies per-cup identification: nominal parameters at a ±10 % fill error should visibly lose damping.
5. **Mechanism decomposition** (for step 1's third contribution): in L1, switch on one imperfection at a time (wrist rate limit only, tracking error only, latency only) and attribute the residual crest to each; then confirm on hardware with kill test 3's artificial-delay runs.
6. **Step 2 pre-hardware gate**: the L0+L1 speed-up of tilt-and-budget over slosh-free, at equal margin, on the three path families with the measured E03-Pro limits. The go rule in §7.4 uses this number.

---

## 7. Experiment plan

### 7.1 The five kill tests

| # | Test | Gates | When | Procedure | Expected **[target]** | Rule |
|---|---|---|---|---|---|---|
| 1 | Bottleneck | Step 2 | Weeks 2–3, empty cup | First size the paths to the E03-Pro's workspace with the holder fitted (reach ≈ 590 mm by the CAD file name **[unverified]**); the 0.4 m line, the 0.20 m circle and the Lissajous figures may need shrinking. Slosh-free geometric tracker (Arrizabalaga-style) on: line 0.4 m; circles r = 0.10/0.15/0.20 m; Lissajous (Arrizabalaga's 3.75 s and 4.5 s figures, then time-scaled); one vertical loop. Scale the time law in owner-confirmed steps until failure. Log the maximum ratio of each joint's velocity and acceleration (and jerk, by differencing `q_cmd`) and the Cartesian ratios to the limits read in week 1; the first to reach 0.95 is the bottleneck. Also log servo faults, late pushes and push reply times | Re-derived in week 1 from the E03-Pro's limits; v1's Panda numbers no longer apply. Prior: on a 6-joint arm the wrist joints bind at the acceleration ramps and on tight circles **[unverified]**. If linear acceleration really caps near 2.5 m/s² (R10), Cartesian limits or the servo path may bind first | Outcome A (the wrist binds on ≥ 1 family at speeds where spill occurs): step 2 as planned. Outcome B (Cartesian acceleration/jerk, or the servo path, binds everywhere): step 2 loses orientation freedom and pivots to rim budget + geometry. Step 1 is unaffected either way. Nothing spills with an empty cup, so the memo records which limit binds and at what speed factor; the A/B call is made once kill test 3 exists. Also record the realized lean-rate-vs-jerk relation for the mechanism figure |
| 2 | Camera measurement and latency | Step 1 | Week 3 (a–c), week 4 (d) | **(a) Static.** Ring-downs at kicks giving ≈ 0.5, 1, 2, 5, 10 mm of crest; 40/60/80 % fill; clear and dyed water; 2–3 lighting/background set-ups. Measure: per-frame noise on s; bias vs amplitude; the smallest amplitude at which the first harmonic still decays smoothly (the contact-line dead band); meniscus offset vs fill. **(b) Calibration.** Fisheye vs rational model; RMS reprojection error on held-out images; per-frame rim-registration residual; focus from the rim to 70 mm below it at the working f-number. **(c) Timing**, over ≥ 60 s at the working frame rate: frame-interval jitter from the embedded timestamps; dropped frames; τ_cam by the LED test; image-stage time. **(d) Moving.** The same kicks during a slow motion (0.5 m/s²) and during the fastest owner-confirmed motion. Measure: detection dropouts; blur; rim-registration residual (holder vibration); innovation vs commanded acceleration; prediction at τ_cam + τ_a vs later frames | (a) σ_s ≤ 0.05 mm per frame; the first harmonic tracks down to ≤ 1 mm of crest in clear water. (b) RMS ≤ 0.3 px. (c) Jitter ≤ 0.2 ms; τ_cam ≤ 10 ms; no dropped frames in 60 s. (d) Dropouts < 1 % of frames; tracking at ≥ 2 mm of crest during the fast motion (≥ 2 m/s² **[target]**) | **Gate at the end of (a)–(c)** (below the table): the classical detector in clear water; if it fails, the CNN front-end in clear water; if that fails, dyed water. Go if (d) holds at 2 mm of crest in clear water on the primary cup at 60 % fill with the front-end that passed the gate, and prediction meets §4's targets. If it holds only with dye: dyed water for the evaluation, and the paper says so. If neither: measurement-method pivot (R1). If ω₀·(τ_cam + τ_a) > 60° and the predictor misses §4's targets: R3 |
| 3 | Residual-slosh measurement | Motivates step 1 | Weeks 6–8 | Slosh-free baseline with water, at increasing speed factor in owner-confirmed steps. Record the in-loop camera's raw crest, the estimator's crest and the spilled mass. Repeat with 0/20/40 ms of artificial delay in the command path | Residual crest 3–10 mm at the speeds where the wrist rate saturates; residual grows with commanded jerk and with added delay | If residual < 2 mm everywhere below spill, step 1's evaluation becomes "how far beyond the baseline's spill speed does the loop get", a weaker but still valid story |
| 4 | Swirl sweep | Step 2's rotary claim | Step-2 data session (week 14), same rig | The arm drives the cup in small circles (1–3 mm radius) at 0.85–1.15 ω₀ in 0.02 steps, 10 s each; the same with planar lateral forcing. A 3 mm circle at 3.4 Hz is 1.4 m/s² and needs T_s ≤ 29 ms. Detect swirl as the first harmonic's phase rotating at the forcing frequency; compare with the Model-B prediction | A planar→swirl transition band near resonance at partial fill | No swirl below the spill amplitude ⇒ drop the rotary claim (keep it as discussion). Swirl present ⇒ the circle-path demo in step 2 uses this period |
| 5 | Crest vs amplitude | Step 2's nonlinear-model claim | Same session | Planar forcing at ω₀, amplitude ramped until the crest reaches the rim, at 60 % and 80 % fill. Crest (max over θ of η) vs first-harmonic amplitude, both from the in-loop camera; overlay linear mode and Model B, harmonic by harmonic (`C₀, C₂, S₂` vs `β₀, β₂, β₂′`). The plane comparison needs the centre slope, which the camera does not see; it comes from the model (§2.5). Near the rim the meniscus and, at the back wall, the rim tab limit the measurement; those sectors are masked | The linear mode shape under-predicts the crest by 15–30 % near the rim at 80 % fill | Linear tracks to the rim within 10 % of freeboard ⇒ B is dropped and B′ with `c_nl = 0` is used. Deviation > 15 % at ≥ 1/3 freeboard ⇒ B stays |

**The kill-test-2 gate** (owner amendment, hardware phase; §0). At the end of (a)–(c), in week 3, the classical detector passes if, in **clear water** under **at least one** lighting set-up, it meets all six:
1. per-frame noise σ_s ≤ 0.05 mm **[target]**;
2. crest tracking ≤ 1 mm in clear water **[target]**: the first harmonic follows the ring-down smoothly down to ≤ 1 mm of crest, which is (a)'s target in the table. The owner's wording, "crest tracking error ≤ 1 mm", may also mean an error bound against a reference; that reading is **[UNCLEAR]** (docs/open_questions.md), and the memo reports both;
3. reprojection RMS ≤ 0.3 px **[target]**;
4. frame jitter ≤ 0.2 ms **[target]**;
5. τ_cam ≤ 10 ms **[target]**;
6. a dead band small enough for the per-pickup fit (§3.2): after the standard kick, the ring-down stays above the dead band for ≥ 1.0 s, the window §3.2 needs for ζ ± 30 % **[proposal]**.

Items 3–5 do not depend on the edge detector. If one of them fails, the CNN does not help: recalibrate, or fix the timing, and run the gate again. The CNN fallback answers failures of items 1, 2 and 6 that come from the edge step (contrast, reflections, blur), not from pinning.

**The gate memo** (`docs/memos/camera-gate.md`, one page, written by `python -m analysis.gate_memo`) gives the six numbers and a pass or fail for each lighting set-up, plus, for information, (a)–(c)'s other measurements: dropped frames in 60 s, bias vs amplitude, meniscus offset vs fill, the rim-registration residual and focus.

**If the classical detector fails:** the CNN front-end (§2.3, `docs/design/cnn_fallback.md`) is trained on clear water and accepted on the same six numbers, measured on held-out sessions and on one lighting set-up held out of training altogether, plus a confidence-calibration check, R₀, R₁ and R₂ refitted for it, and a comparison with the classical detector on dyed water. If it fails too: classical detection on dyed water (last resort), and the paper says so. (d) and everything after it run with whichever front-end passed.

### 7.2 Step-1 evaluation (weeks 12–18)

**Methods**:
- (M1) Arrizabalaga-style geometric tracker, open-loop on the liquid;
- (M2) Muchacho-style linearized-pendulum QP;
- (M3) ours open-loop: same tracker, shake-identified model, no correction;
- (M4) ours closed-loop.

M1 and M2 were published on the Panda, so both are re-implemented on the E03-Pro (reusing the authors' code where it is public); say so, and report the tracking error of every method so the comparison is honest.

**Grid**:
- **Primary cup at 60 %:** 3 paths × 6 speed factors bracketing M1's spill point × 4 methods × 5 trials = 360 trials.
- **40 % and 80 % fill:** M1 and M4 at 4 speeds (120 each).
- **Ablations**, on the primary cup at 60 %, line and circle: identified vs nominal parameters (nominal = theory at the nominal fill with a ±10 % fill error injected); predictor on/off; `ζ_target` ∈ {0.1, 0.3, 0.5}; the optional camera-measured tilt loop on/off.
- **Total:** about 780 trials at ~30 s each including refill and reset, 5–7 lab days **[unverified]**. The second cup is deferred.

**Metrics**: motion time at equal spill outcome; peak crest at equal motion time; spill rate over 5 trials; peak acceleration and jerk actually achieved (commanded and, at 20 Hz, measured); prediction error against later and held-out frames; identification repeatability over ≥ 30 pickups.

**Primary figures**:
- peak crest and spilled mass vs speed factor per method (one panel per path);
- the fastest no-spill speed per method, with confidence intervals;
- raw wall elevation vs estimate and τ-ahead prediction on one aggressive trial;
- the residual-mechanism decomposition;
- identification scatter (f₀, ζ, h) across pickups.

### 7.3 Step-2 evaluation (months 4–5 after the step-1 submission)

- **Paths and fills:** the same as step 1.
- **Baselines:** M1, and an angle-limit MPC (the Emergency-Stopping formulation: bilinear pendulum, 5° inclination bound), re-implemented in the same acados framework.
- **Ours:** rim budget + orientation free + Model B + camera loop.
- **Ablations:** orientation fixed upright; linear vs nonlinear surface; camera loop on/off; identified vs nominal parameters; scenario vs widened margin.
- **Swirl:** the demonstration on the circle at the kill-test-4 period.
- **Targets:** solve rate ≥ 50 Hz. The achieved-acceleration target (v1: ≥ 9.5 m/s², to be comparable with the automatic-machine numbers) is re-set after week 1 to what the E03-Pro allows **[target]**.

### 7.4 Go/no-go between the steps (after the step-1 submission)

Go to step 2 if all three hold:
1. kill test 1 gave outcome A on at least one path family;
2. the step-1 stack works: the estimator meets §4's prediction targets, and the correction removes ≥ 50 % of the residual crest at M1's spill speed;
3. the L0+L1 simulation with the measured E03-Pro limits shows a ≥ 1.3× speed-up of tilt-and-budget over slosh-free at equal margin on ≥ 2 of 3 families.

If (1) fails: container-geometry pivot. If (2) fails: step 2 becomes an open-loop rim-budget NMPC, and the better use of the year is an RA-L extension of step 1 with the robustness study. If (3) fails (speed-up < 1.15×): RA-L extension of step 1 with partial fill and identification, no step 2.

---

## 8. Risk register

Likelihood is my judgement, not a measurement.

| # | Risk | Likelihood / impact | Early signal | Kill criterion | Pivot |
|---|---|---|---|---|---|
| R1 | The contact line does not give a faithful wall elevation: low contrast in clear water, pinning dead band, meniscus effects, blur, reflections | 35 % for the classical detector / fatal to step 1's sensor claim | Kill test 2(a), week 3; the gate memo (§7.1) | No tracking at 2 mm of crest with the classical detector, the CNN front-end, or dye, under any lighting candidate | In order (owner amendment, §0): (1) the CNN front-end on clear water (§2.3, R16), the same output, so config plus one file; (2) dyed water, with clear water reported as failed; (3) the measurement-method pivot: refraction of a printed pattern on the cup bottom (surface slope, clear water), or floating markers. Each is a front-end change, not a schema change (codebase plan R-D). Steps (1) and (2) address contrast, reflections and blur; pinning and the meniscus are physics and survive every front-end, so a dead band too large for §3.2 goes straight to (3) |
| R16 | The CNN front-end (only if the classical detector fails the gate) fails acceptance or is confidently wrong: labels in clear water are poor, it does not carry over to the held-out lighting set-up, or it finds a reflection with high confidence | 40 % / costs 2.5–3.5 weeks and the buffer **[unverified]** | Validation error on held-out sessions; the held-out lighting set-up; the confidence-calibration check | Fails the gate numbers on held-out data after one round of more labels | Dyed water (R1 step 2). Meanwhile, safety does not depend on it: low-confidence rays are missing, too few valid rays make the frame invalid, and an invalid frame runs the controller open loop (M3) (§2.3); the 3σ gate and the authority cap bound what a confidently wrong frame can do |
| R2 | Lens, holder parts or cups not on the bench by week 3 (lead times **[MISSING]**) | 30 % / schedule | Order confirmations, week 1 | Not mounted by week 4 | Kill test 1 needs no camera. Meanwhile develop acquisition, timing (kill test 2(c)) and the front-end with the camera on a bench stand above the cup, with any C-mount lens at hand; interfaces unchanged, calibration redone once the LM4NCL and Rev F are fitted. Kill tests 2(d) and 3 need the camera on the arm and slip with it |
| R3 | Actuation path too slow: the shortest stable T_s, or τ_a, gives ω₀·(τ_cam + τ_a) > 60° and the predictor misses §4's targets | 40 % / degrades or kills the corrector | Servo ladder, week 1; camera-on-target chirp, week 2 | Prediction error at the horizon above §4's targets | Option C (counter-kick, re-planned per estimate). Or correct only in hold and deceleration phases. Or the arm-interface pivot to path B. Its `elfin_drivers.yaml` holds this unit's encoder zero counts (equal to the inspection report's origin counts) and Elfin3 torque factors (elfin_overview.md, QC p.3), but it is still not used as it is: check it against the arm and have the vendor confirm it for the E03-Pro. Path B also needs the bus-end variant of the driver (the CAD file name says bus end **[unverified]**), Ubuntu 20.04 + ROS noetic + a PREEMPT_RT kernel, or the driver's EtherCAT code lifted out of ROS; and whether any controller safety function still applies is not stated, so treat it as unprotected (elfin_overview.md) |
| R4 | Residual slosh in M1 too small to matter below its spill speed | 30 % / weakens motivation | Kill test 3 | < 2 mm everywhere below spill | Evaluate beyond M1's spill speed; emphasise identification and robustness |
| R5 | Elfin faults, collision stops or limits at the speeds the baselines need | 40 % / slows the evaluation | Kill test 1 | Cannot reach the baselines' published timing | Report achieved speeds and tracking error; the comparison is on our hardware anyway |
| R6 | Water on the lens, camera or arm (lens front 88.5 mm above the rim) | 30 % / hardware | First spill trial | — | Drip shield; a lens protector window, calibrated with it fitted; tray; no dye near the arm |
| R7 | ζ identified to only ±50 % (contact-line damping is amplitude-dependent) | 50 % / low | Shake repeatability | — | Report; the corrector barely depends on it |
| R8 | One-person bandwidth: illness, hardware failure | high / schedule | — | — | 2–3 weeks of buffer in §9; writing starts in week 19 in parallel with the last data |
| R9 | Scooping, including published camera-based slosh feedback | see below | Alerts on "slosh", "liquid transport", "spill", "visual feedback" | — | Preprint on submission day; keep the three claims separable; the identification-at-pickup claim is distinct |
| R10 | The E03-Pro's limits put the "fast" regime below v1's: the protocol's linear-acceleration setting stops at 2500 mm/s² (elfin_commands.md, PROTO p.52), and whether it or the joint limits act on servo streams is not stated (elfin_overview.md) | 40 % / changes the bottleneck and the speed targets | Week-1 limit read-out; kill test 1 | — | Report achieved accelerations; re-derive kill test 1's prediction; re-set step 2's acceleration target. The story survives if the wrist still binds |
| R11 | acados solve time (step 2) | 40 % / rate | First sim month of step 2 | > 20 ms at N = 75 | Formulation 3 (§5.2) |
| R12 | Model-B coefficients transcribed wrong | 40 % / wrong constraint | Backbone check against a published curve | Backbone off by > 10 % | B′ with fitted coefficients |
| R13 | Commanded γ is wrong (no IMU): tracking error mis-states `s*` and the forcing | 35 % / biases the estimate and the correction | Innovation correlated with commanded acceleration or jerk (kill test 2(d), kill test 3) | Correlation persists after the servo model and `d` are re-fitted | Richer servo model; then an IMU on the holder (parked: it adds a clock domain, a mount and a calibration) |
| R14 | Servo-mode safety unknowns: no documented exit, unknown late-push behaviour, and a stop command cannot interleave a pending push (one message at a time; elfin_commands.md, PROTO p.1) | 30 % / hardware and schedule | Week-1 ladder, on the mock and at slow speed | — | Decelerate by streaming; `GrpStop` only after the stream ends **[UNTESTED]**; e-stop within reach; the codebase plan's arm-operation rules |
| R15 | Calibration drift: focus or iris ring knocked, shim, PLA holder creep, temperature | 30 % / measurement bias | Rim-registration residual trend; a check image at the start of each session | Residual > 0.5 px **[target]** | Re-calibrate; lock the rings; check before each session |

**Scooping detail.**
- **TUM/Haddadin (Muchacho, Arrizabalaga):** own the two Panda slosh-free baselines and could add sensor feedback to their tracker in a workshop paper at any time. The most likely source of a partial scoop.
- **Aalto (Hynninen, Kyrki):** the 2026 MPC lists partial fill and geometry as future work, which is step 2's ground. A 2027 follow-up with a live sensor is plausible.
- **Politecnico di Milano (Rocco, Zanchettin) and Bologna (Carricato, Di Leva):** feedforward-heavy spill-constraint lines whose natural next step is feedback.
- **Kiel (Meurer):** dynamic optimization plus FEM identification; least likely to add hardware feedback soon.
- **Camera-based slosh sensing** exists in other settings. Whether anyone has closed a camera loop on a carried open cup is the week-1 search **[unverified]**.

Mitigation beyond the preprint: keep the three step-1 claims separable, so that losing one does not sink the paper.

---

## 9. Timeline (25 weeks, one person; week numbers only)

### Milestones

| Week | Milestone | Definition of done |
|---|---|---|
| 4 | **MS1 infrastructure + gates** | Servo period, τ_a and the servo response measured at owner-confirmed settings; τ_cam and frame jitter measured; camera calibrated; slosh-free tracker runs all path families at confirmed speeds; kill test 1 reported; kill test 2(a–c) passed; the shake routine returns f₀ and ζ within 1 % / 30 % of repeat runs |
| 8 | **MS2 closed loop exists** | Estimator meets §4's prediction targets on moving-arm data; kill tests 2(d) and 3 done; correction reduces the residual on the line path |
| 13 | **MS3 method complete** | Correction on line, circle and Lissajous; M1/M2 baselines running; latency and parameter sweeps done in simulation; ablation code ready |
| 14 | **MS4 step-2 data banked** | Kill tests 4 and 5 recorded (analysis deferred) |
| 19 | **MS5 evaluation complete** | Full grid of §7.2 recorded and checked for gaps |
| 23 | **MS6 draft complete** | All figures final; text reviewed once by someone outside the project |
| 25 | **Submission** | IROS 2027; arXiv the same day |
| after submission | Go/no-go for step 2 (§7.4) | — |

Buffer: weeks 16–17 are slack, and week 20 is a second data pass for gaps; total slack ≈ 3 weeks. Writing starts in week 19, overlapping the last data.

If the classical detector fails the kill-test-2 gate in week 3, the CNN front-end costs ≈ 2.5–3.5 weeks **[unverified]**, most of the buffer. Development of the estimator, kill test 2(d) and the shake routine then continue on dyed water as the contrast reference, and MS1's clear-water items (kill test 2(a–c) passed in clear water; the shake routine in clear water) move to the CNN's acceptance. The slipped items are listed in `docs/design/cnn_fallback.md`.

### Step 2 outline (six months after the step-1 submission; ICRA 2028 or RA-L)

1. **Month 1.** Analysis of kill tests 4–5; Model B implementation and backbone check; acados joint-space NMPC in simulation, timing benchmark.
2. **Month 2.** Margin derivation; scenario variant if needed; hardware NMPC tracking with the rim budget on the upright cup.
3. **Month 3.** Orientation free; angle-limit baseline re-implemented.
4. **Month 4.** Evaluation grid; swirl demo.
5. **Month 5.** Ablations; second cup on a Rev F yoke.
6. **Month 6.** Writing; submit.

If step 1 is accepted in the meantime, the related-work section of step 2 cites it.

### Week by week, first month

The software tasks and their done-criteria are in the codebase plan §11; the items below are the research side.

**Week 1: orders, bring-up, timing.**
- **Order or make:**
  - the camera and the Kowa LM4NCL, if not already on hand (the camera is listed as available on request; lead times **[MISSING]**), and a C-mount shim set (0.05/0.1/0.2/0.5 mm);
  - print the Rev F holder with the F.1 base and buy its parts (`README.md`, Sources);
  - several clear acrylic cylinders, R = 40 mm inner and 100 mm tall; measure the wall thickness, then print the matching yoke;
  - a 0.01 g scale; lighting candidates (diffuse LED panel or ring; light and patterned sleeves); food dye;
  - a drip tray and a lens shield; conductive tape (optional electrodes);
  - a PC-driven LED and, if hardware triggering is chosen, a trigger source for the camera's opto-input (USB GPIO or a microcontroller) **[MISSING: choice]**;
  - a rigid ChArUco board, ~120 × 90 mm with 10 mm squares.
- **Ask the vendor in writing:**
  - the servo-streaming behaviour: minimum period, lookahead semantics, how to leave servo mode, what happens on late pushes, whether limits and collision detection act during streaming;
  - confirmation that the shipped `elfin_drivers.yaml` fits this E03-Pro (path B), and whether it is the bus-end or 485-end variant;
  - the E03-Pro's kinematic parameters, payload, reach and torque ratings;
  - the error-code list `HansRobot_ErrorCode.docx` and the SDK manuals the protocol refers to (elfin_overview.md, PROTO p.3, p.192): `Fail` replies carry codes whose meanings are not in the knowledge files.
- **Prior-art search:** camera-based closed-loop slosh suppression, and visual identification of sloshing modes, on manipulators. This keeps the "first" claim honest (R9).
- **Arm bring-up** with the knowledge files' examples 1–4 (elfin_examples.md), each first against the local mock: model string (`ReadRobotModel`; elfin_commands.md, PROTO p.16) and controller version (pendant About screen; elfin_overview.md, LINUX p.19); limits (`ReadJointMaxVel`, `ReadJointMaxAcc`, `ReadJointMaxJerk`, `ReadLinearMaxVel`; PROTO p.56–59) into the single YAML every tool reads; current tool and user frames; state-stream rate, format and the keys it really carries. First bring-up is on the bare flange: payload 0 kg and the default zero TCP and user frame, checked on the pendant before enabling. Payload and TCP (§3.1) are set when the holder goes on, before enabling. About 20 `ReadActPos` poses are recorded for the week-2 FK check.
- **Servo ladder**, run from the Linux machine, with the owner present and every step confirmed (codebase plan §3.8): `StartServo` with a slow J6 sine (example 3: 5° amplitude, 10 s period, 20 s of motion with a Tukey window of 2 s cosine tapers at each end, plus 2 s of streamed holds at q0 before and after), push reply times and late pushes logged. One period per run: T_s = 0.2 → 0.1 → 0.05 → 0.04 → 0.03 → 0.02 → 0.015 → 0.01 s, each step confirmed by the owner in `confirmed.yaml`. The lookahead stays at 0.02 s for the whole ladder and gets its own test later. At each step log commanded vs state-stream joint positions, and record what the arm does when pushes stop and what `GrpStop` does **[UNTESTED]**. Stop and report instead of continuing on any `Fail`, more than 1 % late pushes, a median reply time above T_s/2, commanded vs state-stream J6 that looks wrong, or audible or visible roughness.
- **Camera bring-up:** SDK; embedded timestamp and frame number; sustained frame rate without drops at full frame; frame-interval jitter; LED latency test.
- **Deliverable:** latency memo (confirmed T_s, push reply times, τ_cam) and the logger. The camera's timing tests are recorded separately; a slow streamed motion recorded together with frames moves to week 2 (codebase plan §11).

**Week 2: tracker, kill test 1, actuation delay.**
- IK tracker streaming the tilt law's references; check FK against the week-1 `ReadActPos` poses (positions first, orientations once the Rx, Ry, Rz convention is established). Reference generator for the four path families with a speed factor.
- Camera on Rev F with the cup removed, looking at a static target on the table: chirp on one wrist joint → τ_a and the servo response model; both go into L1.
- Kill test 1 (empty cup) in owner-confirmed speed steps, starting this week and possibly running into week 3; kill-test-1 memo (one page: which limit binds on which path, plus the lean-rate-vs-jerk curve).
- L0: Model A (LTV and nonlinear) and the measurement model in CasADi; unit tests.
- **Deliverable:** kill-test-1 memo; τ_a memo; L0 passing tests.

**Week 3: calibration, front-end, static camera tests.**
- Fit the shim, focus, lock focus and iris; calibrate (§3.1); meniscus offset and fill-level map.
- Contact-line front-end on dyed water first, then on clear water under the lighting candidates.
- Kill test 2(a)–(c); matrix-pencil fitter on the first harmonic; f₀ and ζ vs the dispersion relation and the viscous prior.
- **Deliverable:** per-frame noise and dead-band table, calibration record, first shake routine.

**Week 4: estimator on the moving arm.**
- Estimator live at the tick with delayed-frame updates and predict-only steps; prediction ladder; replay determinism (codebase plan §7).
- Kill test 2(d): kicks during slow and fast motions; the prediction-vs-frame and consistency table → the go/no-go on the camera loop.
- MS1 review: a one-page status against the §7.4 criteria; adjust weeks 5–8.

**Weeks 5–8 in one line each.**
- **Week 5:** prediction against later frames on moving-arm ring-downs; predictor at τ_cam + τ_a.
- **Week 6:** corrector option A on the line path; anti-fight rules; authority cap.
- **Week 7:** kill test 3 across speed factors and artificial delays.
- **Week 8:** correction on the circle; MS2 review.

---

## 10. Paper skeletons

### 10.1 Step 1 (IROS 2027)

**Title options.**
1. *Closing the Loop on the Liquid: Camera Feedback for Fast Slosh-Free Transport*
2. *Shake, Watch, Carry: Per-Cup Identification and Residual-Slosh Correction from a Cup-Facing Camera*
3. *Fast Slosh-Free Transport Is Not Slosh-Free: Measuring and Correcting Residual Sloshing at the Contact Line*

**Abstract (draft, ~150 words).** Slosh-free transport tilts an open container so that effective gravity stays aligned with its axis. On a real arm the wrist cannot rotate fast enough, and latency and tracking error excite sloshing the planner never budgeted, so these methods run below the arm's limits. We close a loop on the liquid with a camera on the cup holder that watches the liquid's contact line on the cup wall. A short shake at pickup identifies the container's first sloshing mode (frequency, damping and fill) from the same camera. A time-varying Kalman filter formulated in the cup frame tracks the residual slosh from delayed camera frames, predicting across the image-processing and actuation delays. An acceleration-level correction damps the residual on top of an unchanged slosh-free tracker. On a Han's Robot Elfin E03-Pro across line, circle and Lissajous paths at 40–80 % fill, we quantify where residual slosh comes from, and show that the closed loop reduces peak rim elevation by [x] % at equal speed and raises the fastest spill-free speed by [y] % over open-loop slosh-free baselines. *(Fill x, y from data; delete any claim the data does not support.)*

**Contributions (three, matching the verified gaps).**
1. First closed-loop correction of residual sloshing during transport from a camera watching the liquid in the carried cup. Prior manipulator work is open-loop on the liquid, with cameras used only for validation; the "first" is **[unverified]** until the week-1 search.
2. Per-container identification of the sloshing mode at grasp time (frequency, damping and fill) from the same camera, feeding the estimator and the correction.
3. A measured account of why residual slosh exists in fast slosh-free motion: wrist-rate limit → jerk cap; latency and tracking error → unbudgeted excitation. Decomposed in simulation and confirmed on hardware.

**Figure list.**
1. Concept: slosh-free ideal vs realized lean, and where the residual comes from.
2. Architecture, rates and the latency chain (camera + actuation).
3. Model A in the cup frame and the camera measurement: wall elevation from the contact line, plane vs mode gains; the frequency-modulation term.
4. Shake routine: raw wall elevation, fitted ring-down, identification scatter over 30 pickups and three fills.
5. Prediction vs later frames on an aggressive trial; innovation.
6. Residual crest vs commanded jerk and vs added delay (kill test 3).
7. Main result: peak crest and spill vs speed factor for M1–M4 on three paths.
8. Ablations: identified vs nominal parameters, predictor on/off, `ζ_target`.

Video: side-by-side at the fastest spill-free speed of each method.

**Related-work positioning.**
- *Slosh-free transport on the Panda* (Muchacho 2022; Arrizabalaga 2024): we keep their tilt law and add sensing; the comparison is at equal tracker, on our arm.
- *Bounded-slosh feedforward* (Maderna 2018; Di Leva 2021/22; Grundelius 1999/2000): a different constraint, all open-loop; step 2's territory, cited but not compared here.
- *Tilt compensation and filtered feedforward* (Moriello 2017/18; Reinhold 2019; Rothenbuchner 2024): feedforward; we are the feedback complement.
- *MPC with a no-spill constraint* (Hynninen & Kyrki 2026): no live liquid sensing, near-brim fill; we sense and go to partial fill.
- *Haptic liquid perception* (Matl 2019): static mass and volume; we identify the dynamic mode, visually.
- *Spill classifier at partial fill* (Abderezaei 2024): quasi-static.
- *Visual liquid sensing*: position it after the week-1 search.

Never claim the spill constraint, the slosh-free condition, a new fluid model, or sim-to-real.

**If the CNN front-end is used** (§0 amendment): say so in the method, the abstract's sensing sentence and the limitations. It is a learned contact-line detector trained on labelled frames of this cup under these lighting set-ups; report its labels, splits, held-out-lighting results and confidence calibration. Contribution 1 is unchanged in substance (the loop, the estimator and the correction are model-based), but drop any wording that implies classical, training-free sensing, and do not claim the detector as a contribution or as general liquid perception. Exact wording: `docs/design/cnn_fallback.md`.

### 10.2 Step 2 (ICRA 2028 / RA-L)

**Title options.**
1. *Tilt and Budget: Jointly Optimized Orientation and Rim Clearance for Agile Transport of Partially Filled Open Containers*
2. *Beyond Slosh-Free: Rim-Clearance NMPC with a Nonlinear Free Surface and Camera Feedback*
3. *Spend the Freeboard: Fast Liquid Transport under a Curved-Surface Spill Constraint*

**Abstract (draft).** Slosh-free transport is sufficient for not spilling, not necessary, and it is throttled by the wrist. We make container orientation a decision variable and replace the slosh-free constraint with a rim-clearance budget, evaluated on a nonlinear curved free-surface model with true wall elevation and solved as a joint-space NMPC in real time; the slosh-free methods are the zero-budget special case. Tilt cancels the acceleration the wrist can follow, and the freeboard absorbs the transients it cannot. The liquid state comes from the camera loop and per-container identification of our earlier work. On a Han's Robot Elfin E03-Pro at 40–80 % fill we [beat] slosh-free and angle-limit baselines on line, circle and Lissajous paths, handle rotary sloshing on circular paths, and show which part of the freeboard each model ingredient buys back. *(Bracketed verbs are placeholders.)*

**Contributions.**
1. Joint optimization of orientation and rim clearance under wrist-rate limits, with slosh-free as the zero-budget case.
2. A nonlinear curved free surface with wall elevation inside a real-time NMPC constraint, with the margin decomposed by source.
3. Rotary sloshing handled and demonstrated in robot transport.
4. Partial fill, 40–80 %, with per-container identification.

**Figure list.**
1. Tilt-only vs budget-only vs blend on one path: lean, crest, time.
2. Bottleneck test: which limit binds where.
3. Model B vs linear crest (kill test 5), with the margin read-off.
4. Swirl sweep (kill test 4) vs theory.
5. NMPC solution anatomy on the circle: orientation, crest budget use, joint-limit activity.
6. Main result across paths, fills and baselines.
7. Ablations: orientation fixed/free, linear/nonlinear surface, camera loop on/off, margin type.
8. Solve time and margin split.

**Positioning.** The spill constraint is Maderna/Di Leva; the tilt is Muchacho/Moriello; the MPC with an inclination bound is Hynninen & Kyrki. None blends tilt with a budget, none uses fluid nonlinearity or wall elevation in the constraint, none treats swirl (Orr 2021 shows why pendulums miss it), and none goes to partial fill with live sensing. Check Maderna's full text again before writing this paragraph; it is the closest.

---

## Appendix A — numbers and provenance

| Quantity | Value | Status |
|---|---|---|
| IROS 2027 deadline | — | confirm on the official site |
| Elfin protocol ranges | joint velocity 0–200 °/s; joint acceleration 0–720 °/s²; linear velocity 0–3000 mm/s (default 2000); linear acceleration 0–2500 mm/s² (default 2500); jerk read-only | elfin_commands.md / elfin_overview.md, PROTO p.49–52, p.56–59; "depend on the model"; E03-Pro values read in week 1 |
| Elfin state stream | 20 / 10 / 5 Hz on ports 10004 / 10005 / 10006; JSON; no timestamp | elfin_overview.md, elfin_commands.md, PROTO p.7, p.185–188 |
| Servo streaming | example period 0.2 s, lookahead 0.02 s; minimum period, exit and late-push behaviour **[MISSING]** | elfin_commands.md, PROTO p.172–175 |
| E03-Pro kinematics | E03 STEP model or a vendor E03 description; the knowledge files mention only the `elfin5` URDF **[MISSING]** | elfin_overview.md |
| E03-Pro payload and reach | 3 kg, ≈ 590 mm, read from the CAD file name | elfin_overview.md **[unverified]**; confirm with the vendor's datasheet |
| Camera MV-CS016-10UM | IMX273, global shutter, 1440 × 1080, 3.45 µm, ≤ 249.1 fps at full frame, exposure ≥ 15 µs, embedded timestamp/frame number/trigger counter, I/O and triggers as §0, ≈ 80 g | MaxxVision product page (listed as available on request); timestamp semantics, ROI frame rate and latency **[MISSING]** |
| Lens Kowa LM4NCL | 3.5 mm, F1.4–16, 1/1.8″, −28 % TV, focus 0.2 m to infinity, Ø31 × 30.5 mm, 60 g, 100 / 60 lp/mm (centre / margin), back focus 8.9 mm; from the datasheet: ±5 % focal length, front clear aperture Ø19.6 mm, angle-of-view table | Kowa product page; Kowa datasheet, whose 73 g, 1/2″ and 1/3″ formats and 40 lp/mm corner figure differ from the page **[UNCLEAR]** |
| Lens on the IMX273 | field 80° × 60° × 101°; distortion −7 / −15 / −26 % at short edge / long edge / corner | computed from the datasheet's angle table **[unverified]** |
| Rev F geometry | lens front 88.5 mm above the rim (design); field at the rim ≈ 148 × 102 mm; holder ≈ 0.44 kg; spine mode ≈ 196 / 137 Hz | Rev F model **[unverified]** except the design dimension |
| Contact-line sensitivity | 1.8 / 2.3 / 3.1 px per mm of wall elevation at 40 / 60 / 80 % fill | computed (Rev F geometry, lens angle table, on-axis camera) **[unverified]** |
| Per-frame noise on s | σ_s ≈ 0.014 mm at 60 % fill from a 0.2 px edge target and 32 sectors | computed **[unverified]** |
| Depth of field and focus | f/4.4 keeps blur under 2 px from the rim to 70 mm below it; focus ≈ 114 mm from the lens front; needs ≈ 0.05 mm of extra back focus (a 0.05–0.1 mm shim) | thin-lens estimate **[unverified]** |
| Frame data rate | 1.56 MB per full Mono8 frame; 311 MB/s at 200 fps | arithmetic |
| Latency limit τ₆₀ | 50.5 / 49.5 / 49.3 ms at 40 / 60 / 80 % fill | 1/(6 f₀), from §2.2 |
| `ξ₁, J₁(ξ₁), J₀(ξ₁)`; `ξ₂, J₁(ξ₂)` | 1.8412, 0.5819, 0.3161; 5.331, −0.346 | standard |
| Modal fraction, pendulum length, frequency | §2.2 | Abramson/Dodge/Ibrahim; verify the mass-fraction formula against Ibrahim's table |
| ζ prior 0.5 % | Mikishev–Dorozhkin-type formula | coefficients unverified |
| `κ = 1.54 tanh(ξ₁h/R)`, `χ = 0.378/tanh`, `R/L = 1.84 tanh` | derived in this plan | check numerically |
| Latencies, servo period, push reply time, acados ≤ 10 ms, particle sim hours | guesses | measure |
| All expected outcomes in §7 | targets | — |

## Appendix B — references to pin before writing

Carried over from v1:
- **Slosh transport and control:** Maderna et al., ICRA 2018 · Di Leva et al., ICAR 2021; Multibody Syst. Dyn. 2022 · Grundelius & Bernhardsson 1999/2000 · Muchacho et al., IROS 2022 · Arrizabalaga et al., ICRA 2024 · Moriello et al., ICRA 2017; Control Eng. Practice 2018 · Reinhold, Amersdorfer, Meurer, IROS 2019; Acta Mechanica 2024 · Hynninen & Kyrki, arXiv 2604.16667 (2026) · Abderezaei et al., IROS 2024 · Rothenbuchner et al., JSV 2024 · Orr, AAS 21-433 (2021) · Matl, Matthew, Bajcsy, IROS 2019 · Aribowo et al. 2010/2015.
- **Sloshing models:** Abramson (ed.), NASA SP-106, 1966 · Dodge, *The New Dynamic Behavior of Liquids in Moving Containers*, SwRI 2000 · Ibrahim, *Liquid Sloshing Dynamics*, CUP 2005 · Faltinsen & Timokha, *Sloshing*, CUP 2009 · Lukovsky, De Gruyter 2015 · Raynovskyy & Timokha, CRC 2020 · Miles, *Physica D* 1984 (spherical pendulum) · Hutton 1963 (rotary sloshing in cylinders).

Added for the camera design:
- **Camera calibration:** Zhang, "A flexible new technique for camera calibration", IEEE TPAMI 2000 · Kannala & Brandt, "A generic camera model and calibration method for conventional, wide-angle, and fish-eye lenses", IEEE TPAMI 2006 · Garrido-Jurado et al., ArUco markers, Pattern Recognition 2014.
- **Contact lines and damping:** de Gennes, "Wetting: statics and dynamics", Rev. Mod. Phys. 1985 · Keulegan, J. Fluid Mech. 1959 (damping of standing waves).
- **For the measurement-method pivot:** Moisy, Rabaud & Salsac, "A synthetic Schlieren method for the measurement of the topography of a liquid interface", Exp. Fluids 2009.

Chapter, equation and table numbers for the model sources are to be pinned from the physical books. Treat every benchmark number in the survey table as unverified until read in the primary source.
