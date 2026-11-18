# CNN fallback for the contact-line front-end (design)

Owner amendment, hardware phase (research plan §0 "Amendment", §2.3, §7.1, §8 R1/R16; codebase plan §2, §5, §9, §10).
Markers as in the plans. Numbers here are **[proposal]** (my choice, to be confirmed or changed by the owner),
**[target]** or **[unverified]** until measured.

**Nothing in this document is built before the classical detector fails the kill-test-2 gate.**
The gate memo (`docs/memos/camera-gate.md`, written by `python -m analysis.gate_memo`) decides.
Until then, only the parts both front-ends share exist: the per-ray output contract and the valid-ray rule.

Only failures that come from the edge step lead here (research plan §7.1). When item 6 (the dead band) fails, the
memo asks the owner first whether it comes from contact-line pinning or from the edge step (`deadband_cause` in
`gate_input.json`, judged from the ring-down plots); pinning goes to R1's measurement-method pivot, not here. After
the CNN's acceptance (stage 4.5), stage 4.4 runs again with `front_end: cnn` and the CNN's numbers: that is gate
memo v2. It counts only once 4.5 has passed, and stage 5.1 follows from 4.4 as before (pre-hardware review W1).

## 1. What it replaces, and what it must not change

The image stage samples an intensity profile along each of the 32 rays of the ray table, in a window round
the expected radius from the fill and meniscus map. The **edge step** turns each profile into an edge position.
The CNN replaces the edge step only.

| | Classical (default) | CNN (fallback) |
|---|---|---|
| Input per ray | profile, M samples, 1 px apart, centred on r_exp(θ_k) | the same profile; optionally rays k−1 and k+1 as channels |
| Output per ray | edge position along the ray (sub-pixel), confidence ∈ [0, 1], valid flag | the same three |
| Logged in | `cam_feat` (edge `(u, v)`, edge strength = confidence, valid flags) | the same fields; the header's front-end block names the model (codebase plan §5) |
| Downstream | measurement stage → η(θ) → harmonic fit → estimator → controller | **unchanged**, and blind to which front-end ran |

Switch: `edge_method: classical | cnn` in the camera config, inside `CameraFrontend` (codebase plan §10: no new interface).

## 2. Default model: a small 1D CNN per ray

- **Window.** M = 128 samples at 1 px (±64 px round r_exp). At 80 % fill and 3.1 px per mm that is ±20 mm,
  the whole freeboard **[unverified]**. Profile normalised by subtracting its mean and dividing by a fixed
  32 grey levels, so that low contrast stays visible to the network as low contrast **[proposal]**.
- **Layers** (ReLU after each, zero padding, all sizes fixed at compile time):

| Layer | In → out channels | Kernel | Stride / dilation | Positions |
|---|---|---|---|---|
| L1 | 1 (or 3) → 16 | 7 | 1 / 1 | 128 |
| L2 | 16 → 16 | 5 | 2 / 1 | 64 |
| L3 | 16 → 16 | 5 | 1 / 2 | 64 |
| L4 | 16 → 16 | 5 | 1 / 4 | 64 |
| L5, heatmap | 16 → 1 | 1 | — | 64 cells of 2 px |
| No-edge head | mean and max of L4 over positions (32) → 1, logistic | — | — | — |

  About 4.1k parameters (4.3k with neighbour channels) and a receptive field of 59 px (computed).
- **Decoding.** Find the heatmap's peak cell; take a softmax over the 5 cells round it and their mean position
  (sub-pixel); the confidence is (1 − no-edge score) × the share of the whole heatmap's softmax mass inside those
  5 cells. A second peak elsewhere (a reflection, a droplet) lowers the confidence; it cannot pull the position
  halfway to it, as a global soft-argmax would. Confidence is calibrated on validation data (one temperature).
- **Why this size.** The 1 ms budget **[target]**, not the 50k-parameter ceiling, sets it. This model costs
  ≈ 8.4 M multiply-adds per frame for 32 rays (computed), ≈ 0.5–1 ms at 8–16 G multiply-adds/s on one core in
  double precision **[unverified]**; benchmark on the Linux machine. A full-resolution version (no stride in L2)
  costs ≈ 16 M; a 50k-parameter model would cost ≈ 10× the budget. If the benchmark misses 1 ms: fewer channels,
  a shorter window, or float32 (then re-check parity).
- **2D model:** only if the 1D model fails on held-out data, and only with a stated reason: for example, the
  errors come from structure across rays (a droplet spanning sectors, a reflection that a single ray cannot tell
  from the line). Its cost would have to be argued against the 3 ms image-stage budget first.

## 3. Labels for clear water

The hard part: in clear water nothing but a person (or another camera) sees where the line is.

| Option | What it gives | Cost and limits | Verdict |
|---|---|---|---|
| (i) Side reference camera through the acrylic, synchronized, training and validation only | the best labels during motion: the contact line seen from the side | **hardware [MISSING]**: a second camera and lens, a mount (world-fixed: static ring-downs only; on the holder: weight, a second cable), the trigger output device for synchronization (already **[MISSING: choice]**), side lighting, its own calibration to the cup frame; sees only the sector facing it (rotate J6 to cover others). **It contradicts research plan §0 ("Ground truth: the in-loop camera only") and codebase plan §10**, so it needs a further §0 change the owner has not made | only if the owner amends §0; then as a validation set, not the main label source |
| (ii) Static fills set by weight | the mean level exactly; with the meniscus offset, every ray's label at rest | the meniscus offset in clear water is itself the unknown. Fix: weigh the same fill twice, dyed then clear, and take the offset from the dyed images, assuming the dye does not change the meniscus **[unverified]**. Extend with **static tilts**: the arm holds the cup at known small tilts (one joint, a few degrees, at the confirmed speed), the level surface then gives η(θ) = R tan β cos(θ − θ_t) round the whole wall. Contact-angle hysteresis leaves an error of roughly ±0.5 mm **[unverified]** | yes: free, exact geometry, covers all 32 rays; the bias check for everything else |
| (iii) Labelling tool seeded by the classical detector | dynamic frames, every sector | the operator corrects a curve (the five-harmonic fit of η(θ)), not 32 points, with the video before and after the frame visible; marks rays "no edge visible". ≈ 15–30 s per frame **[unverified]** | yes: the main source of dynamic labels |
| (iv) Augmentation: noise, blur, brightness, reflections, droplets | many times the data | it multiplies labels, it does not create them; reflections and droplets are synthesized from real examples cut from images | yes, in training only; never in validation or test |

**Recommendation:** (iii) for dynamic frames, (ii) with static tilts for exact labels and for checking (iii)'s
bias, and (iv) in training. (i) only after a §0 change, as validation.

**Never** label frames by fitting the ring-down model and reading the label off the fit. Inside the dead band the
real line stops moving smoothly, and a network trained on model labels would learn to hide exactly that.

**How many** **[unverified]**: 2,000–4,000 labelled frames (64k–128k ray profiles), at least a third of them
below 2 mm of crest where the gate is decided, spread over ≥ 150 ring-downs, plus the static scenes. Check with a
learning curve (25 / 50 / 100 % of the training set): if the held-out error is still falling at 100 %, label more.

## 4. Splits

- **Unit:** the group (session, fill level, lighting set-up). A whole group goes into one split; frames and
  ring-downs are never split, because frames from one ring-down are correlated.
- **Held out completely:** one lighting set-up, chosen before training (a seeded draw, written down), in no
  training or validation group. Reported separately.
- **The rest:** groups to train / validation / test ≈ 60 / 20 / 20 by a seeded assignment, each fill present in
  each split. Needs ≥ 3 sessions per lighting set-up **[proposal]**.
- The split file is part of the dataset and of the dataset hash.

## 5. Training (never in `rt/`)

- `analysis/cnn/`, in its own venv (`requirements-cnn.txt`: PyTorch CPU build, NumPy); a GPU is not needed at this size.
- Dataset extraction reads the stored `.img` ROI images and the calibration's ray table, so it samples exactly
  what the online image stage samples; through the pybind module, the same code (R-C).
- Loss: cross-entropy of the heatmap against a Gaussian of σ = 1 cell at the label, plus L1 on the decoded
  position, plus binary cross-entropy on "no edge"; rays without an edge skip the position terms.
- Fixed seeds (Python, NumPy, PyTorch), deterministic algorithms, one thread.
- Written next to the weights in `data/cnn/models/<model_id>/`: `weights.slcnn`, `model.json` (architecture, the
  dataset hash, the training config and its hash, the git hash, the seed, the validation metrics), and the
  weight file's SHA-256.

## 6. Inference in `rt/core`

- Plain C++/Eigen, no OS headers, no allocation in the loop, a constant operation count per frame (fixed loops;
  ReLU and the 5-cell decode cost the same for every input). Double precision first.
- **Weights:** the flat file `weights.slcnn` (magic `SLCNN001`, a length-prefixed JSON header with the
  architecture, tensor shapes and offsets, then float64 little-endian tensors in a fixed order) is read once at
  start-up by `rt/app` into a fixed-capacity POD struct that `rt/core` receives. The loader refuses a shape that
  differs from the compiled architecture; the hash goes in the trial header.
- **Parity:** a fixed set of 1,000 profiles (synthetic and recorded) through PyTorch in float64 and through the
  C++ via pybind; every logit, position and confidence within 1e-5 (CI).
- **Budget:** ≤ 1 ms per frame on an isolated core of the Linux machine, and the whole image stage within 3 ms
  **[target]** (target machine only).
- Replay: the image-level replay of codebase plan §7 covers it unchanged; it is part of the image stage.

## 7. Safety

- A ray whose calibrated confidence is below c_min = 0.9 **[proposal]** counts as missing.
- Fewer than N_valid = 16 valid rays of 32, or a 90° quadrant with fewer than 2 **[proposal]**: the measurement
  is invalid, the estimator predicts only, and the corrector is disabled as after a frame dropout, so the
  controller runs open loop (M3) (research plan §2.3, §5.1). Masked sectors never count.
- Bad vision can only remove the correction. The corrector's authority cap and the tracker's 80 % joint-acceleration
  rule bound what any measurement can ask for, and every planned motion is checked against `confirmed.yaml` first.
- The 3σ innovation gate stays, and so does the harmonic fit's residual check, which rejects rays inconsistent with
  a smooth η(θ).
- Shadow mode first: before the CNN closes the loop, both front-ends run on the same frames (offline, from stored
  images) and disagreements above 1 px **[proposal]** are reviewed.

## 8. Acceptance

All on the held-out test groups, and separately on the held-out lighting set-up:
1. the six gate numbers of research plan §7.1, measured as for the classical detector;
2. **confidence calibration:** reliability diagram; expected calibration error ≤ 0.05 **[proposal]**; above c_min,
   ≥ 99 % of rays within 1 px of the label **[proposal]**;
3. **R₀, R₁, R₂** of research plan §2.3 refitted for the CNN; the classical coefficients are not reused;
4. **dyed water,** where the classical detector is reliable: the CNN against it, median |Δr| ≤ 0.2 px and σ_s
   not worse **[proposal]**. This is the check that the CNN finds the same line, not a different feature.

## 9. Paper wording

- Method: "The contact line is detected along precomputed rays by a small one-dimensional convolutional network
  (≈ 4k parameters) trained on N labelled frames of this cup under K lighting set-ups; labels came from [source].
  It returns a position and a calibrated confidence per ray; low-confidence rays are discarded, and frames with
  too few valid rays are treated as dropouts."
- Report: labels and their checked bias, the splits, the held-out-lighting result, the calibration plot, and the
  dyed-water comparison.
- Scope: §0 now allows a learned contact-line detector in the loop; it is still not a contribution. Contribution 1
  ("first closed-loop correction … from a camera watching the liquid") is unchanged in substance, because the loop,
  the estimator and the correction stay model-based. Remove any wording that implies training-free, classical
  sensing, and do not claim generality beyond this cup and these lighting set-ups (limitations section).
- If dye was needed after all: say so; the clear-water result is then reported as failed (research plan §7.1).

## 10. Schedule

Cost if the gate fails at the end of week 3: **≈ 2.5–3.5 weeks [unverified]**, one person.

| CNN week | Work |
|---|---|
| 1 | `tools/label` (seeded by the classical detector); dataset extraction from stored images; static-fill and static-tilt recordings; ≈ 1,000 frames labelled |
| 2 | training pipeline; first model; C++ inference, parity test, benchmark on the Linux machine; ≈ 2,000 more frames |
| 3 | acceptance on held-out groups and the held-out lighting set-up; calibration; R₀–R₂; dyed-water comparison; gate memo v2 |
| +0.5 | one round of extra labels if the learning curve asks for it |

**MS1 (week 4) items that slip** (research plan §9):
- "kill test 2(a–c) passed": in clear water it moves to the CNN's acceptance (≈ week 6–7). The dyed-water gate can pass in week 4.
- "the shake routine returns f₀ and ζ within 1 % / 30 % of repeat runs": in clear water, likewise; on dyed water in week 4.
- Kill test 2(d) (week 4): on dyed water in week 4 as tuning data, repeated in clear water after acceptance.

**Not slipped:** the servo period, τ_a and the servo response; τ_cam and frame jitter; the camera calibration; the
slosh-free tracker on all path families; kill test 1. These do not depend on the edge step.

**Knock-on:** MS2 (week 8) loses margin, because kill test 3 is recorded in clear water once the CNN passes; the
2.5–3.5 weeks come out of the ≈ 3 weeks of buffer (weeks 16–17 and 20).
