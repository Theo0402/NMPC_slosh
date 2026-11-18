# Fast transport of an open cup, camera in the loop: planning package v3

These are the plans for a one-person research project. A Han's Robot Elfin E03-Pro arm carries an open, partially filled cup of water along fast paths without spilling. A camera on the cup holder looks straight down into the cup, measures the sloshing, and closes a loop on it. Step 1 targets IROS 2027; step 2 targets ICRA 2028 or RA-L.

The package is meant to be used without any other context. What it relies on but does not contain is listed under "Outside this zip".

## Files

| File | What it is | Contents |
|---|---|---|
| `research_plan_v3_camera.md` | The research plan: what is built and measured, and why | §0 decisions, assumptions, departures from v1, riskiest assumption, scope · §1 architecture and latency budget · §2 liquid models and the camera measurement model · §3 identification · §4 estimator · §5 controllers (step-1 corrector, step-2 NMPC) · §6 simulation · §7 experiments and the five kill tests · §8 risk register · §9 timeline · §10 paper skeletons · Appendix A numbers and provenance · Appendix B references |
| `codebase_plan_v3_camera.md` | The software plan: how it is built | §1 design rules R-A to R-D · §2 repository layout · §3 coding rules, including §3.8 on operating the arm · §4 model layer · §5 log schema · §6 threads, cycle and interfaces · §7 replay and determinism · §8 data governance · §9 tests · §10 what is not built · §11 build order for weeks 1–4 · §12 the workbench |
| `README.md` | This file: what each file is, which one wins, and the shared vocabulary | — |

Read this file first. Then read research plan §0, §1 and §9, and codebase plan §3.8 and §11.

## Which file wins when two disagree

Highest first:
1. **The owner's decisions** (below, and research plan §0). Only the owner changes them. A measurement that undercuts one triggers the pivot the plans name for it; it does not silently change the decision.
2. **Measurements.** Once a memo, a calibration record or a latency record exists, its value replaces any **[unverified]** or **[target]** number in either plan.
3. **Facts about the robot**: the lab's Elfin knowledge files and the manual pages they cite ("Outside this zip"). A plan that disagrees with them about a command, a unit, a limit or a behaviour is wrong.
4. **Camera and lens specifications**: the product pages and datasheets under "Sources". Where those disagree with each other, the plans mark it **[UNCLEAR]**.
5. **What is built and measured, and why**: models, parameters, identification, estimator, controllers, experiments, kill tests, risks and schedule. The research plan wins.
6. **How the software is built**: layout, log schema, threads, interfaces, coding rules, the rules for operating the arm, and the software build order. The codebase plan wins.
7. **Notation and units**: this README wins.

Some topics appear in both plans: the latency budget, the work of weeks 1–4, and the timing of the kill tests. There, the research plan owns the numbers and the codebase plan owns the software tasks. When you find a disagreement, fix the losing file and note the fix in the project journal (codebase plan §3.7, P4).

## Decisions already made

| Item | Decision |
|---|---|
| Arm | Han's Robot Elfin E03-Pro, 6 joints: the lab's arm, serial MH217340001 |
| Arm interface | **Path A**: the Han's V5 TCP protocol, with commands on port 10003 and a state stream on port 10004, and joint positions streamed with `StartServo` + `PushServoJ`. **Path B**, the pivot: the vendor's ROS EtherCAT driver |
| Liquid sensor | Camera in the loop is the main design: a Hikrobot MV-CS016-10UM with a Kowa LM4NCL 3.5 mm lens on the Slim Cup Holder Rev F (F.1 flange), looking straight down the cup axis |
| What the camera measures | The wall contact line, i.e. the elevation of the free surface at the wall all the way round. It must work with clear water; dyed water is the contrast reference and the last-resort fallback |
| Contact-line front-end (owner amendment, hardware phase) | Classical edge detection in clear water by default. If it fails the kill-test-2 gate, a small 1D CNN per ray replaces the edge step, with the same output; fallback order classical/clear → CNN/clear → classical/dyed (research plan §0 "Amendment", §2.3, §7.1) |
| Operator interface (owner request, hardware phase) | One workbench page for every stage and coefficient, a separate process outside the loop (codebase plan §12) |
| IMU | None. Effective gravity comes from the commanded trajectory through forward kinematics, with the measured servo response |
| Ground truth | The in-loop camera only. The estimator is validated on prediction and on consistency. Spilled mass from a 0.01 g scale; rim electrodes optional |
| Software | One C++ real-time process. Python for analysis, calibration, identification fits, simulation and code generation. `elfin_min.py` stays as the bring-up tool, unchanged; the command table (`tools/elfin_py/protocol.py`) is the protocol reference |
| Cups | One for now: 40 mm inner radius. A second geometry later |
| Schedule | Week numbers only, no calendar dates |
| Wrist F/T sensor | Parked: the measurement interface admits it, no code is written. The EtherCAT/SOEM sensor thread is dropped |
| Control | NMPC first; reinforcement learning is a stretch |

The plans also build on these defaults, set when the plans were rewritten, which the owner did not object to:
- the sloshing mass m₁ is not identified;
- the hinge height p_h comes from theory or an optional tilt kick;
- the payload setting comes from weighing plus the CAD centre of mass;
- a stop is a streamed deceleration (`GrpStop` is untested);
- every increase in speed or stream rate needs the owner's confirmation;
- the "first" claim stays **[unverified]** until the week-1 prior-art search.

## What changed from the earlier versions

The earlier research plan (v1) and codebase plan (v2) assumed a Franka Panda, a wrist force/torque sensor with an IMU, a 1 kHz torque loop and an EtherCAT sensor thread. v3 differs as follows:
- **Measurement.** The camera measures the free surface, so there is no rigid wrench to subtract, and everything that existed for that is gone: bias and gain states, payload identification, the sensor thread.
- **Estimator.** Frames arrive at camera rate and after an image-processing delay. The estimator applies each frame at its exposure time and predicts in between.
- **Identification.** The camera identifies f₀, ζ and the fill. It cannot identify m₁ at all, and it gets p_h only from rotations or from theory.
- **Kill test 2** was F/T signal-to-noise. It is now a camera test: latency, frame-time jitter, noise on s, distortion calibration. The risks are updated to match.
- **Arm.** Joint positions are streamed to the Han's controller instead of commanding torque.
- **Cups.** The second cup is deferred, so the evaluation grid is ≈ 780 trials instead of ≈ 900.

Neither earlier file is needed to use this package.

## Markers

| Marker | Meaning |
|---|---|
| **[unverified]** | A number or fact not checked or not yet measured, including values computed from a model or a datasheet |
| **[target]** | An expected outcome, not a result |
| **[MISSING]** | No source states it; the text says how to measure or find it |
| **[UNTESTED]** | Not tried on the hardware |
| **[UNCLEAR]** | The sources are ambiguous or contradict each other |
| **[proposal]** | A value or rule the software proposes, neither decided by the owner nor measured; it stands until the owner confirms or changes it (`docs/open_questions.md` lists them) |
| **[E03]** | Found in the lab's earlier records of this same arm (serial MH217340001): the factory inspection, the MATLAB scripts, the ROS driver config. Likely to hold, but confirm, since settings and firmware may have changed |

Elfin facts are cited as (knowledge file, manual page), e.g. (elfin_commands.md, PROTO p.172).

## Shared vocabulary

### Frames and points

| Symbol | Name | Definition |
|---|---|---|
| W | world | The robot's base frame, in which the controller reports base-frame poses. `g_W` is gravity in W |
| F | holder (hand) frame | Origin at the centre of the Elfin flange's Ø63 mm spigot face, on the J6 axis. x along J6, away from the flange and toward the cup; z along the cup axis, out of the cup (up when J6 is horizontal and the cup upright); y = z × x. The Rev F model and its centre-of-mass figures use this frame. It is not the controller's own flange frame, whose axis directions are **[MISSING]** (research plan §3.1) |
| K | camera frame | The calibrated pinhole frame of the lens model, OpenCV convention: origin at the projection centre, z along the optical axis toward the scene, x along increasing u, y along increasing v |
| C | cup frame | Origin at the hinge P. z_C along the cup axis, out of the cup (parallel to z_F); x_C and y_C parallel to x_F and y_F |
| P | hinge | Model A's pendulum hinge, on the cup axis at height p_h above the inner bottom |
| (u, v) | image coordinates | Pixels, OpenCV convention (origin at the centre of the top-left pixel); u along the image's long side (1440 px), v along the short side (1080 px). The long side runs along J6 |
| θ | wall angle | Angle round the cup axis in C, measured from x_C toward y_C; `e(θ) = (cos θ, sin θ)` |

**Names in code.** `a_P_W` is the acceleration of point P expressed in W. `R_AB` maps vectors from B to A, so `R_WC = R_WF * R_FC`. `T_KC` maps points from C to K. Quaternions are used in the loop, Euler angles only in figures.

### Symbols

Units are given in SI; the plans often quote mm, ms, Hz and degrees for readability.

**Cup and liquid**

| Symbol | Meaning |
|---|---|
| R | Inner radius of the cup, 0.040 m. R alone is always the radius; R(t) and R₀–R₂ are the measurement-noise covariance and its coefficients; `R_AB` is a rotation |
| H_c | Inner height of the cup, 0.100 m |
| h | Fill height (depth at rest). Fill % = h / H_c; 40 / 60 / 80 % is h = 40 / 60 / 80 mm |
| freeboard | H_c − h: 60 / 40 / 20 mm |
| t_w | Wall thickness of the cup, **[MISSING]** |
| m_ℓ, ρ_w | Liquid mass (kg); density of water |

**The first sloshing mode (Model A)**

| Symbol | Meaning |
|---|---|
| ξ₁, ξ₂ | 1.8412 and 5.331: the first two zeros of J₁′ (J₀, J₁ are Bessel functions) |
| ω₀, f₀ | Natural frequency of the first antisymmetric mode at rest, rad/s and Hz (f₀ = ω₀/2π): 3.30 / 3.37 / 3.38 Hz at 40 / 60 / 80 % fill (computed) |
| ζ | Damping ratio of that mode |
| ω_damped, σ | Damped frequency and decay rate of a ring-down: ω₀ = √(ω_damped² + σ²), ζ = σ/ω₀. σ with a subscript, or in "3σ", is a standard deviation |
| L | Pendulum length, g/ω₀² |
| m₁ | Sloshing (modal) mass; theory value only in v3 |
| p_h | Height of the hinge P above the inner bottom |
| κ | Wall elevation per unit dynamic modal displacement, 1.54 tanh(ξ₁h/R) |
| R/L | Wall elevation per unit quasi-static displacement s*, ξ₁ tanh(ξ₁h/R) (a plane) |
| χ | Model A ↔ Model B mapping, s = χ·(a, b), χ = 0.378 / tanh(ξ₁h/R) |
| s ∈ R² | Lateral modal displacement of the pendulum, in C, m; s = L·(n_x, n_y) |
| n, ν | Pendulum direction (a unit vector) and its angular velocity, in the nonlinear form |
| s* | L γ_xy / G: the quasi-static equilibrium, i.e. the level surface |
| s − s*, ρ | The residual slosh and its amplitude, ρ = \|s − s*\| |
| E | Modal energy, ½(\|ṡ\|² + ω₀²\|s\|²) |

**Gravity and kinematics**

| Symbol | Meaning |
|---|---|
| g, g_W | 9.81 m/s²; the gravity vector in W |
| a_P | Acceleration of P in W |
| γ | Effective gravity at P in C, `R_CW (g_W − a_P)`; γ_xy its lateral part; G = −γ_z > 0, the effective gravity along the cup axis |
| γ_cmd | γ computed from the commanded trajectory by forward kinematics, delayed by τ_a and shaped by the identified servo response |
| ω, α | Angular velocity and acceleration of the cup, in C |
| J | The 2 × 2 rotation by 90°, [[0, −1], [1, 0]]. J₀, J₁ are Bessel functions; J1–J6 are the arm's joints |
| q, q̇, q̈, q⃛ | Joint positions (rad), velocities, accelerations, jerks; `q_cmd` the joint positions streamed to the arm |

**Camera measurement**

| Symbol | Meaning |
|---|---|
| η(θ, t) | Wall elevation: the height of the free surface at the inner wall above the rest level, at angle θ. η_k in sector k; η₀ the axisymmetric part; η_h the higher harmonics; z₀ the rest level's height above P |
| N_sec | Number of wall sectors, 32 **[target]** |
| C₀, C₁, S₁, C₂, S₂ | Least-squares harmonics of the sector elevations: η_k ≈ C₀ + C₁ cos θ_k + S₁ sin θ_k + C₂ cos 2θ_k + S₂ sin 2θ_k. Capitals, so that S₁ is not read as a component of s |
| z | The measurement given to the estimator, (C₁, S₁) |
| H, D_z | Measurement matrix and feedthrough: z = H x + D_z s* + v, with H = [κI, 0, 0, I, 0] and D_z = (R/L − κ) I |
| v, R(t) | Measurement noise and its covariance, R(t) = R₀ + R₁\|ṡ̂\|² t_exp² + R₂\|ŝ − s*\|⁴ (`R_cov` in code) |
| σ_edge, σ_η, σ_s | Standard deviations of one edge point (px), of a sector's elevation, and of s per frame |
| T_KC | The calibrated pose that maps points in C to K |
| ray table | Precomputed pixel positions along the measurement rays, N_sec sectors × M samples, built from the calibration |

**Estimator**

| Symbol | Meaning |
|---|---|
| x | The state [s, ṡ, d, b_z, δ] |
| d | Forcing disturbance: the difference between commanded and actual effective gravity, m/s² |
| b_z | Measurement bias, m |
| δ | Slow stiffness drift (optional) |
| P | The state covariance. As a point, P is the hinge; code names keep them apart (`P_cov`, `p_P_W`) |
| ŝ, ṡ̂, η̂ | Estimated displacement, velocity and crest |
| NIS | Normalized innovation squared |
| k₀, k₁ | Offset and drift terms of the ring-down fit |

**Timing**

| Symbol | Meaning |
|---|---|
| tick | One cycle of the 1 kHz control thread |
| f_cam, t_exp | Frame rate (200 fps **[target]**) and exposure time (≤ 1 ms **[target]**) |
| t_e | A frame's exposure midpoint: the time its measurement is valid (`t_valid` in code) |
| τ_cam | From t_e to the measurement being available to the estimator |
| τ_a | Actuation delay: from a `q_cmd` computed at a tick to the joints moving, as identified by the servo-response fit. It includes the wait for the push slot, the reply, the controller's lookahead and the joint servo |
| τ, τ_total | τ = τ_cam + τ_a, the delay the corrector must bridge; τ_total, from the exposure midpoint to the joints moving |
| τ₆₀ | 1/(6 f₀): the delay at which ω₀τ = 60° (≈ 49–51 ms) |
| T_s, T_la | The servo push period and lookahead: `StartServo`'s dServoTime and dLookaheadTime |

**Control**

| Symbol | Meaning |
|---|---|
| p_d, v_d, a_d, j_d | Reference position, velocity, acceleration and jerk of P, in W |
| R_d, ω_d, α_d | Reference orientation, angular velocity and angular acceleration of the cup, from the tilt law |
| Δa | The corrector's lateral acceleration in the cup-horizontal plane; Δp, Δv its integrals, added to the reference |
| k_d, k_p, ζ_target | Corrector gains, k_d = 2 ζ_target ω₀ and k_p ≈ 0; ζ_target is the damping the corrector aims for |
| λ, Γ(λ) | The NMPC's path-progress variable, and the path |
| N | Number of shooting nodes in the NMPC |
| m_k | The NMPC's crest margin at node k: m_model + m_param + m_est,k + m_track |

**Model B (step 2)**

| Symbol | Meaning |
|---|---|
| (a, b) | Generalized coordinates of the cos θ and sin θ first modes |
| β₀, β₂, β₂′ | Slaved second-order mode amplitudes: axisymmetric, cos 2θ, sin 2θ |
| η̄ | Smooth upper bound on the crest; c_a and c_v are its coefficients and ε a smoothing constant |
| μ, c_nl | Model B′'s softening coefficient and crest-nonlinearity coefficient |

### Units

- **Code and logs**: SI (m, s, rad, kg, N, m/s²). Image features are in pixels (px).
- **The Elfin protocol, and only inside the command client**: degrees for joints and rotations, mm for positions, °/s and °/s² for joint speed and acceleration, mm/s and mm/s² for linear moves, kg and mm for the payload (elfin_commands.md).
- **Fill**: percent of the cup's inner height.
- **Times**: on the host's monotonic clock unless named otherwise. Camera timestamps are in camera ticks, whose rate is **[MISSING]** until measured.

### Names used in both plans

| Name | Meaning |
|---|---|
| Step 1, step 2 | Closed loop on a slosh-free tracker (IROS 2027); tilt-and-budget NMPC (ICRA 2028 or RA-L) |
| Models A, A′, B, B′ | Liquid models (research plan §2) |
| Options A, B, C | Corrector designs (research plan §5.1): lateral-acceleration damping, tilt-based damping, counter-kick |
| Formulations 1–3 | NMPC formulations (research plan §5.2) |
| Paths A, B | Arm interfaces: TCP servo streaming; the EtherCAT driver |
| Outcomes A, B | Kill-test-1 results: the wrist binds; Cartesian limits bind everywhere |
| M1–M4 | Evaluation methods (research plan §7.2) |
| MS1–MS6 | Milestones (research plan §9) |
| Kill tests 1–5 | Bottleneck; camera measurement and latency; residual slosh; swirl; crest vs amplitude (research plan §7.1) |
| R1–R15 | Risks (research plan §8) |
| L0, L1, L2 | Simulation layers: reduced models, robot inner loop, particle validator (research plan §6) |
| R-A to R-D | Design rules (codebase plan §1) |
| D, RT, F, N, LG, A, P | Coding-rule families (codebase plan §3) |
| tuning, eval, killtest, scratch | Dataset tags (codebase plan §8) |
| Calibration record, latency record, params file | Measured outputs, each with an ID (codebase plan §8) |
| Shake | The ≈ 3 s identification motion at pickup (research plan §3.2) |
| Confirmed values | The speeds and stream rate the owner has confirmed (codebase plan §3.8) |

## Hardware, as far as the sources go

- **Arm.** Han's Robot Elfin E03-Pro, serial MH217340001, with a Mini Cabinet HRC-E0610-A3 and a Han's controller that speaks the V5 TCP protocol. From the lab's records: IP 192.168.0.10 and controller 5.51.5_EPro4ms.beta.20230522 at inspection; 3 kg payload and ≈ 590 mm reach by the CAD file name **[unverified]**. Still to confirm on the arm (the owner's list): controller version, IP address, joint limits, and the shortest servo period that works. Whether a force sensor is fitted no longer matters: the camera replaced it, and no force-sensor code is written.
- **Camera.** Hikrobot MV-CS016-10UM:
  - Sony IMX273, a 1/2.9″ global-shutter mono sensor, 1440 × 1080 at 3.45 µm, up to 249.1 fps at full frame in Mono8;
  - USB3 Vision / GenICam;
  - a 6-pin I/O connector with one opto-isolated input, one opto-isolated output and one GPIO;
  - an embedded timestamp, frame number and trigger counter;
  - 29 × 29 × 30 mm, ≈ 80 g.

  Source: the MaxxVision page, which lists the camera as available on request.
- **Lens.** Kowa LM4NCL, 3.5 mm, C-mount. The Kowa page and a Kowa datasheet differ in some figures (**[UNCLEAR]**, research plan §0).
- **Holder.** Slim Cup Holder Rev F with the F.1 flange base; design files and model at the link under Sources. It has the ISO 9409-1-50-4-M6 flange interface from the E03-Pro flange drawing and yokes for 70, 80 and 90 mm outer cup diameters. Model estimates **[unverified]**:
  - lens front 88.5 mm above the rim (a design dimension), with the image's long side along J6;
  - holder ≈ 0.44 kg including camera, lens and hardware, centre of mass ≈ (42, 1, 107) mm in F.
- **Cup.** Clear acrylic cylinder, 40 mm inner radius, 100 mm inner height; wall thickness **[MISSING]**.
- **Spill measurement.** A 0.01 g scale; optional rim electrodes made of conductive tape.

## Outside this zip

- **The lab's Elfin knowledge files**: `elfin_overview.md`, `elfin_commands.md` and `elfin_examples.md`, kept in the owner's project. Every Elfin citation in the plans points into them, and `elfin_examples.md` contains `elfin_min.py` and examples 1–5. They cite the Han's manuals by key:
  - PROTO: the HansRobot V5 TCP protocol, HR-TCP-V1.0.0.3;
  - LINUX: the Linux system manual for the Elfin cobot;
  - NET: the network setup guide;
  - ROS: the vendor's EtherCAT driver;
  - QC: the factory inspection report of this arm (serial MH217340001), where it is called an E03.

  Page numbers are the printed footer numbers. Copies of the three knowledge files sit beside this file in `docs/plans/`.
- **The Rev F print package** (STL, 3MF and GLB files with their own README) and its model page.
- **The superseded v1 research plan and v2 codebase plan.**

## Sources

- [Hikrobot MV-CS016-10UM, MaxxVision product page](https://www.maxxvision.com/produkte/kameras/usb3-vision-kameras/2734/mv-cs016-10um); the page links the [datasheet](https://www.maxxvision.com/downloads/Cameras/USB30/Hikrobot/Hikrobot_MV-CS016-10UMUC.pdf)
- [Kowa LM4NCL, Kowa product page](https://www.kowa-lenses.com/LM4NCL-1-1.8-3.5mm-C-Mount-wide-angle-lens/10096)
- [Kowa LM4NCL datasheet (copy hosted by Artisan Technology Group)](https://www.artisantg.com/info/Kowa_LM4NCL_Datasheet2_202012122515.pdf)
- [Slim Cup Holder Rev F, with the F.1 flange](https://claude.ai/artifact/VkAcmKJ6P4NimSgWEwgmC4): the owner's model page, private unless shared
