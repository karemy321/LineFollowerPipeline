# LineFollowerPipeline v3 — Implementation Plan (glare robustness + distractor rejection)

> **Audience:** Claude Code, working in the project repository.
> **Status:** decided plan. This file merges two independent analyses (Claude + Gemini) and already
> resolves the disagreements between them — do not re-litigate the decisions in §3; implement them.
> If something in the code contradicts this file, trust the code, note it, and adapt.

---

## 0. How to work

1. Read `LineFollowerPipeline.java` fully, `README.md`, and the existing tests before changing anything.
2. Run `cd LineFollowerLibrary && gradlew test` first and record the baseline (expected: 26 passing).
3. Work **phase by phase** (§5). After each phase: build, run all tests, update this file's checklist
   (§9), and make one git commit per phase. Do not start a phase while the previous one is red.
4. Write the new failing simulator tests (Phase 0) **before** the fixes, so every fix is proven.
5. Ask the user before: changing a public method signature, changing a default that alters v2
   behaviour without a flag, or touching the Android app's camera code in a way you cannot test.
6. Build output goes to `C:/RoboPhoneBuild/library` (outside OneDrive — its file locks break Gradle).

## 1. Hard constraints (unchanged from v2)

- Single file `LineFollowerPipeline.java`, Java 8, **zero external dependencies**, no Android APIs.
- **Zero heap allocations per frame** in `processFrame*` (allocate everything in `init()`).
- **No public API break.** Nothing removed or renamed. Only add `Config` / `FrameData` fields.
- Output contract unchanged: `result[0]` steering `-50…+50` (neg = turn left), `result[1]` tracking
  `trackingMinValue…100`. Steering sign and scale must stay the same.
- All three input formats (gray, YUV 4:2:0, ARGB) keep working.
- The old behaviour must be recoverable with config flags (as v2 did for v1):
  `corridorMode = false` + `glareMask = false` ⇒ exact v2 behaviour, and the v2 tests still pass.
- Performance budget: ≤ 1.5 ms/frame at 960×1280 YUV on a desktop CPU (v2 ≈ 0.9 ms). Measure it.

## 2. Diagnosis (verified against the code — line numbers are for the v2 file as uploaded)

### 2.1 Why strong light makes the robot lose the track

| # | Cause | Where |
|---|---|---|
| G1 | **Single point of failure.** The look-ahead is only traced if the H-ROI (base of the ⊥) found the line: `traceLookAhead(..., trackFound ? centroidX : -1, ...)` and the loop condition `row >= vRoiTop && trackFound`. Glare on the near strip ⇒ the perfectly visible line ahead is thrown away. | `runPipeline` ~L2056; `traceLookAhead` loop ~L2273 |
| G2 | **Hold is blind and too short.** `trackHoldFrames = 6` (0.1–0.2 s) only freezes the last output; a glare patch takes far longer to pass under the strip ⇒ after the hold steering is pinned to ±50. | `runPipeline` ~L2015–2041 |
| G3 | **One global Otsu threshold for the whole 960-px strip.** A bright patch makes Otsu split "glare vs. non-glare" instead of "line vs. floor". | `stripThreshold` ~L1580, `otsuThreshold` ~L2582 |
| G4 | **Saturated pixels are counted.** Pixels at Y≈255 carry no colour information but are classified as line or floor through the linear projection. | `prepareWorkImage` ~L1159 |
| G5 | **Distractor rejection makes glare worse.** Line merged with a glare blob ⇒ band > `bandMaxWidthMult`× ⇒ rejected / trimmed ⇒ "no line". | `considerRun` ~L1725, `trimBand` ~L1769 |
| G6 | **Look-ahead stops at a glare gap.** More than `maxGap = max(8, hRoiThickPx)` empty rows ends the trace ⇒ reads as a dead end ⇒ tracking drops. | `traceLookAhead` ~L2199, ~L2362 |

### 2.2 Why similar-coloured objects still fool it sometimes

| # | Cause | Where |
|---|---|---|
| D1 | The decision is made **in one strip only** (width + position + line-ness). There is no check that the candidate continues as a line in the rows above. | `findBand` / `considerRun` |
| D2 | `trimBand` can **carve a tape-width piece out of a blob/shadow** and adopt it as the line. | `trimBand` ~L1769 |
| D3 | Fill-only test: a run that passes the threshold with the right width is accepted. A real tape has a **sharp floor→line edge and a sharp line→floor edge**; shadows and blobs usually do not. | `considerRun`, `traceLookAhead` row scan |
| D4 | Known v2 limit: an object of exactly the tape width at the predicted position. | README "Remaining limit" |

## 3. Decisions (merged from both analyses)

**Adopted:**
- **Longitudinal corridor (the supervisor's proposal, refined).** Stop treating the H-ROI as a gate.
  Detect the line in many horizontal slices along a corridor that follows the line from the bottom
  strip up to `vRoiTop`, fit one curve on the **floor plane**, and derive offset/heading/tracking from
  the curve. The corridor is **re-centred every frame on the predicted curve**; it is *not* a fixed
  band learned at calibration (calibration provides only colours, width, zero column, heading zero).
- **Corridor width** ≈ 2.2× the calibrated line width at each row (perspective-scaled with
  `rowScale`), never narrower than line width + 2×`runGapTol`, so both tape edges stay visible.
- **Robust floor-plane parabola** `x(u) = A + B·u + C·u²` (u = forward distance beyond the H-ROI row,
  already used in `computeHeading`). `A` = reconstructed near-line position (even when the near rows
  are hidden), `B` = heading, `C` = curvature. Robust = IRLS with Huber/Tukey weights or a small
  deterministic RANSAC — no random allocation.
- **Per-slice detector:** saturation mask + local contrast + edge-pair verification (Gemini's
  edge-pair idea, adopted).
- **Multi-row continuity score** for candidates (Gemini's idea, adopted) — a cheap first stage before
  the robust fit, and the acceptance test for `trimBand`.
- **Temporal filter:** small constant-velocity Kalman (or α-β) filter on (A, B, C) so the robot can
  coast through a glare region lasting ~0.5–1 s without jumps.
- **Fallback:** keep the full v2 band logic for (re)acquisition, calibration, and low-confidence
  frames (90° corners, junctions, line leaving the frame).
- **Camera side (Android app):** lock AE + AWB after calibration; slight negative exposure
  compensation. Optional hardware: linear/circular polariser film over the lens.

**Rejected / corrected:**
- ✗ Gemini's line numbers (975, 1126, 705, 474–645, 804–865) are wrong — use §2.
- ✗ "Fisher puts all weight on Y because Y variance is large" — wrong direction: `w = Σ⁻¹·Δm`, so large
  Y variance *reduces* Y weight. Y dominates only when the line/floor difference is brightness.
  ⇒ Do **not** blindly cap `kY`. For achromatic pairs (black/white/grey) the signal *is* luminance;
  glare is handled by the saturation mask + local contrast. Only for clearly chromatic pairs
  (chroma separation alone ≥ `colorMinSeparation`) add an option to prefer chroma (see §6, `chromaBias`).
- ✗ Fully independent slices with no prediction (OpenMV style) — raises distractor risk. Slices are
  independent of the H-ROI, but search **inside the predicted corridor** while tracking; full-width
  independent search only when acquiring.
- ✗ "All 26 tests will keep passing unchanged" — tests that check the v2 loss/hold behaviour
  (e.g. `lostLineReportsTrackLost`) test exactly what changes. Keep them under the v2 flags; add v3 tests.
- ✗ Full inverse-perspective warp of the image, neural nets, particle filters — too heavy for the
  constraints; benefit small here.

## 4. Target architecture (per frame, corridor mode)

```
prepareWorkImage(vRoiTop, hRoiBot)                    // line-ness image (unchanged)
  + saturation mask (G4)                              // masked = "unknown", never line/floor
predict curve (A,B,C) from Kalman                      // prior
for each slice s in N slices (bottom strip … vRoiTop):  // independent of H-ROI success
    corridor = predicted x(u_s) ± halfWidth(s)         // full width when not tracking
    local-contrast profile across corridor              // per-row, O(W)
    candidates = runs with edge-pair (rise + fall) at expected width(s) ±30%
    pick candidate nearest prediction; record (u_s, x_s, weight_s) or "no observation"
continuity score per chain of slices                   // reject chains < minChainSlices
robust parabola fit in floor coords (IRLS/RANSAC)      // weights from residuals + slice quality
Kalman update with fit (measurement noise from fit RMS & coverage)
confidence = coverage × fit quality × near-coverage
if confidence high:    offset from A, heading from B, tracking from C + visible length
elif low but recent:   coast on Kalman prediction (time-limited, see glareCoastMs)
else:                  fall back to v2 band logic (acquire / corners / lost line)
```

Mapping to existing outputs (keep the v2 formulas so steering gain stays valid):
- `offsetTerm` = (x_img(A at H-ROI row) − `steerRefX`) / halfW × 100, where x_img is the image
  column of floor point (u=0, x=A) — i.e. exactly the v2 offset, computed from the curve.
- `headingTerm` = v2 formula with ψ = atan(B) − ψ₀.
- Tracking: lean of the curve ahead (use curve x at the far-weighted look-ahead distance vs.
  `trackRefX`, same as v2) combined with the road-length part; masked rows **do not** count as
  "road ended" (G6).

## 5. Phases

### Phase 0 — Failing tests first (TrackSim)
Add scenarios to `TrackSim` / tests (each in gray and YUV):
1. `glareNearStrip` — saturated ellipse covering the H-ROI and the nearest 20 % of the V-ROI, for
   ~40 frames while driving straight and on a gentle curve. Expect: `trackFound` stays true (or
   `lineHeld`/coasting), steering continuous (|Δsteer| ≤ 6 per frame), never pinned to ±50.
2. `glareMidLookAhead` — glare band in the middle of the look-ahead. Expect: tracking does not drop
   as if it were a dead end.
3. `glareOnTapeOnly` — specular highlight on the centre of the tape (edges still visible).
4. `sameWidthObjectBesideLine` — dark rectangle of tape width, 1.5 line widths beside the line, short
   (ends after a few slices). Expect: never locked.
5. `blobTouchingLine` — wide dark blob merged with the line. Expect: centroid error ≤ 0.3 line width.
6. `ninetyDegreeCorner` and `lineEnds` — must still behave as v2 (fallback works).
Record which fail on v2 (most of 1–5 should).

### Phase 1 — Quick wins inside v2 structure (low risk)
- **Saturation mask** (`glareMask`, `glareLumaMax = 245`, ARGB: any channel ≥ 250): masked pixels are
  excluded from histograms, column profiles and row runs. Columns that are mostly masked are
  "unknown": a band may bridge across them (do not split, do not count as floor).
- **Look-ahead without base:** if the H-ROI finds nothing, still trace the look-ahead, seeded from the
  predicted column (`lastCentroidX + centroidVel`) with a widened window; if a consistent line is
  found ahead, reconstruct the base from the floor fit (A) and report `trackFound = true` with a new
  `FrameData.baseReconstructed = true`.
- **Masked rows are not gaps** in `traceLookAhead` (G6); only rows with valid pixels and no line count.
- **Time-based hold/coast:** replace the 6-frame hold by `glareCoastMs` (default 600 ms) of coasting on
  the last curve, only while the predicted line is mid-image (keep v2's edge rule: leaving at the
  frame edge pins at once).
- **Local threshold for the strip:** compute Otsu per segment (e.g. 8 segments across the width) or a
  Bradley-style local mean, ignoring masked pixels (G3).

### Phase 2 — Corridor tracker (core of v3)
- New `Config.corridorMode` (default `true`), `corridorSlices` (12), `corridorWidthMult` (2.2),
  `minChainSlices` (4), robust-fit params, Kalman noise params (§6).
- Implement §4. Pre-allocate all slice arrays in `init()`.
- Slices: spaced uniformly in **floor distance** (not image rows) between the H-ROI and `vRoiTop`,
  each a few rows thick (average rows inside a slice for noise robustness).
- Per-slice detector: local contrast on the line-ness image (pixel minus mean of the corridor's
  outer parts, or minus a running mean with window ≈ 3× expected width); edge-pair check with edge
  distance and sharpness threshold **scaled per slice** (far slices are blurrier).
- Robust fit + Kalman as in §3. Expose `FrameData.corridorCoverage` (0…1), `corridorFitRms`,
  `curveA`, `curveB`, `curveC` (floor units), `coasting`.
- Fallback rules: confidence < `corridorMinConfidence` for > coast time, or residual pattern of a
  corner (near half fits, far half does not) ⇒ v2 path for that frame; re-enter corridor mode when a
  v2 detection + look-ahead agree.
- Calibration: also run the corridor during phase 2 of calibration to validate it (median of slice
  widths vs. `rowScale` prediction); store nothing new that breaks `requestRecalibration()`.

### Phase 3 — Distractor hardening
- `trimBand` (D2): accept a trimmed sub-band only if it continues the corridor chain of the slices
  above (position within 0.5 line width of the curve at that slice). Otherwise reject.
- Edge-pair verification on H-ROI candidates too (D3) — in v2 path as well, behind `edgePairCheck`.
- Continuity score in acquisition (D1): when searching full-width, rank candidates by how many
  slices above continue them with perspective-consistent width.

### Phase 4 — Android app (RoboPhoneApp)
- After calibration completes: lock AE and AWB (Camera2Interop `CONTROL_AE_LOCK`,
  `CONTROL_AWB_LOCK`, or CameraX 1.7+ `FocusMeteringAction` locking mode if available); unlock on
  RECALIBRATE. Optional exposure compensation index ≈ −1 EV, as a setting.
- HUD: optionally show `corridorCoverage`, `coasting`, `baseReconstructed`.
- If you cannot run the app, implement behind a setting and tell the user exactly what to test.

### Phase 5 — Docs and tuning
- Update `README.md` ("What's New — v3" table in the same style as v2, new Config/FrameData rows,
  how to get v2 behaviour back), and write `_investigation/CHANGES_v3.md` with test results,
  timings, and the closed-loop simulation results if the existing simulator supports it.
- Re-check the Blockly starting values (steering gain c, power mode) still hold; document any change.

## 6. New Config fields (proposed defaults — tune with the simulator)

| Field | Default | Purpose |
|---|---|---|
| `glareMask` | `true` | Exclude saturated pixels. `false` = v2. |
| `glareLumaMax` | `245` | Y at/above which a pixel is masked (ARGB: any channel ≥ 250). |
| `glareCoastMs` | `600` | Max time to coast on the predicted curve while the line is hidden mid-image. |
| `corridorMode` | `true` | Corridor tracker. `false` = v2 pipeline. |
| `corridorSlices` | `12` | Number of slices from H-ROI to `vRoiTop`. |
| `corridorWidthMult` | `2.2` | Corridor half-width = mult × expected line width / 2 … per slice. |
| `minChainSlices` | `4` | Min consecutive consistent slices for a valid chain. |
| `corridorMinConfidence` | `0.35` | Below this the frame uses coast / fallback. |
| `edgePairCheck` | `true` | Require floor→line and line→floor edges. |
| `chromaBias` | `0.0` | 0…1; >0 only applied when chroma alone separates line/floor. |
| Kalman `q*`, `r*` | tune | Process / measurement noise. |

New FrameData fields (additive): `baseReconstructed`, `coasting`, `corridorCoverage`,
`corridorFitRms`, `curveA`, `curveB`, `curveC`, `maskedFrac` (fraction of masked pixels in view).

## 7. Acceptance criteria

- All Phase 0 scenarios pass in corridor mode; with `corridorMode=false, glareMask=false` all
  original 26 tests pass unchanged.
- Glare scenarios: no ±50 pin, |Δsteering| per frame ≤ 6 while coasting, `trackFound || coasting`.
- Distractor scenarios: no lock on the object; centroid error ≤ 0.3 line width.
- Straight / curve / corner / dead-end behaviour no worse than v2 (compare steering traces).
- Zero allocations per frame (verify, e.g. allocation counter in a test or by inspection), timing
  within budget.

## 8. Answers from the user (already given — do not ask again) and what they imply

1. **Input:** live phone camera. Verify in `RoboPhoneApp` `MainActivity` which path is used (README says
   CameraX → packed YUV 4:2:0 → `calibrateYuv420`/`processFrameYuv420`); optimise and test YUV first,
   but keep gray and ARGB working.
2. **Colours are not fixed** — any two clearly different colours (not close to each other).
   ⇒ Both cases must work: achromatic pairs (black/white/grey — luminance is the signal, glare handled
   by mask + local contrast) and chromatic pairs (chroma helps; `chromaBias` only when chroma alone
   separates). Every Phase 0 glare/distractor scenario must run with at least: black-on-white,
   white-on-black, and one chromatic pair (e.g. red on green / black on blue).
3. **Glossy surface: usually the track (tape), not the floor — not guaranteed.**
   ⇒ Highest priority scenario is `glareOnTapeOnly` (specular highlight on the tape centre / near part
   of the tape, edges still visible) → edge-pair detection and "masked columns bridge the band" are key.
   Still test glare on the floor (`glareNearStrip`) since it is not excluded.
4. **Speed up to ~50–60 cm/s.** ⇒ coasting is distance-critical: 600 ms ≈ 30–36 cm of blind driving.
   Coast on the Kalman-predicted *curve* (not a frozen steering value), keep `glareCoastMs` at 600 as
   an upper bound, and fade confidence during coasting so a curve is still followed. Simulate the
   scenarios at 0.6 m/s with the app's real frame rate (measure it in the app / use 30 fps as default).
5. **90° corners possible but rare.** ⇒ The fallback to the v2 band logic for corners is mandatory and
   must be covered by `ninetyDegreeCorner` tests in corridor mode; corridor mode must not "smooth over"
   a corner (detect the near-fits / far-does-not residual pattern and hand over to v2).
6. Still unknown — ask only if needed: whether the app can be tested on the robot, and the deadline.

## 9. Progress checklist (update as you go)

- [x] Phase 0 — failing scenarios added, baseline recorded (commit on branch `v3`)
  - `TrackSim`: image-fixed glare ellipses (saturating or partial; optionally only on the tape's
    centre, edges dark) and extra objects in the line colour. Existing renders are unchanged.
  - `V3ScenarioTest` (0.6 m/s, 30 fps; one 60 fps case; gray + YUV from the same render): each test
    is tagged with the phase expected to fix it and skipped until then (`-Dv3.runAll=true` runs all).
    `AllocationTest` (0 bytes / 1000 frames, gray/YUV/ARGB, incl. glare and lost-line frames) and
    `PerfBench` (`gradlew perf`), `gradlew checkJava8` added.
  - **Results on v2** (pairs black-on-white, white-on-black, black-on-blue; red-on-green YUV-only):

    | Scenario | Phase | v2 |
    |---|---|---|
    | `glareNearStrip` (straight + curve, 40 frames) | 1 | **FAIL** 14/14 runs (pins ±50 after the 6-frame hold) |
    | `glareMidLookAhead` | 1 | **FAIL** 6/6 (tracking 100 → 23: read as a dead end) |
    | `glareOnTapeOnly` (saturated centre) | 1 | **FAIL** 8/14 (black-on-white, red-on-green, black-on-blue gray; white-on-black passes) |
    | `glareOnTapeOnlyPartial` (Y≈150–220 centre, req. 3) | 2 | **FAIL** 6/10 (one half of the split tape picked, ~8 px off) |
    | `edgeExitUnderGlare` (req. 4) | 1 | **FAIL** 4/4 (held / lost under glare before the exit) |
    | `sameWidthObjectBesideLine` | 0 | pass |
    | `blobTouchingLine` | 0 | pass |
    | `ninetyDegreeCorner` (R 3 cm, v3 vs v2 flags) | 0 | pass (regression guard) |
    | `lineEnds` (v3 vs v2 flags) | 0 | pass (regression guard) |

  - Suite: 36 tests — 31 pass, 5 skipped; v2 timing (`gradlew perf`, 960×1280 YUV, glare frames
    included): **0.99–1.04 ms/frame** mean.
- [ ] Phase 1 — glare mask, look-ahead without base, masked rows ≠ gaps, time-based coast, local threshold
- [ ] Phase 2 — corridor tracker + robust fit + Kalman + fallback
- [ ] Phase 3 — trimBand continuity, edge-pair, continuity-ranked acquisition
- [ ] Phase 4 — AE/AWB lock in app
- [ ] Phase 5 — README v3, CHANGES_v3.md, timings, Blockly values re-checked

## 10. Notes from the code check before Phase 0 (2026-11-09)

**Baseline:** `gradlew test` → **26/26 pass** (LineFollowerPipelineTest 20, TrackSimTest 6; ~26 s,
TrackSimTest ≈ 18 s of it). Build output goes to `C:/RoboPhoneBuild/library` as configured; no
OneDrive lock problems. (Gradle's small project cache `LineFollowerLibrary/.gradle/` does sit inside
OneDrive; it is git-ignored and caused no trouble.) JDK: none on PATH — Android Studio's JBR 21
(`C:\Program Files\Android\Android Studio\jbr`) is used via `JAVA_HOME` per command.

**Plan vs code — all §2 line numbers and names match the v2 file** (2619 lines): `runPipeline` L1864,
hold L2012–2044, `traceLookAhead(..., trackFound ? centroidX : -1, ...)` L2056, loop L2273,
`maxGap` L2199, gap break L2362, `stripThreshold` L1580, `otsuThreshold` L2582,
`prepareWorkImage` L1159, `considerRun` L1725, `trimBand` L1769. No corrections needed.

**Differences / facts to keep in mind:**
- **Java level.** The main file compiles with `javac --release 8` (checked). But `build.gradle` sets
  Java 11, and the tests use Java 9+ APIs (`List.of`). Constraint applies to the main file only; check
  `--release 8` after every phase.
- **Frame size and rate.** `MainActivity` requests 1280×720 @ 60 fps, but per CHANGES_v2 §0 the camera
  delivers 1280×960 → **960×1280 portrait** (verified there from screenshots). Perf is measured at
  960×1280. The app *targets* 60 fps; §8.4 says simulate at 30 fps by default — 30 fps is the worse
  case for coasting (0.6 m/s ⇒ 2 cm/frame, as `TrackSimTest` already steps). Both 30 and 60 are run.
- **CameraX is 1.4.0** (`libs.versions.toml`), so Phase 4 uses Camera2Interop
  `CONTROL_AE_LOCK` / `CONTROL_AWB_LOCK` (the "1.7+" path in §5 does not apply).
- **TrackSim tests are YUV-only at 320×240 (landscape)**; `TrackSim.render` returns ARGB, so the
  Phase 0 scenarios render once and feed both gray and YUV.
- **Which flag gates Phase 1.** §1 requires `corridorMode=false, glareMask=false` ⇒ exact v2, but
  Phase 1 items (look-ahead without base, masked rows ≠ gaps, time-based coast, local threshold) are
  not all "masking". Decision: **all Phase 1 behaviour is gated by `glareMask`**; with it off, v2's
  `trackHoldFrames` logic runs unchanged.
- **Time base for `glareCoastMs`.** `processFrame*` has no timestamp, and adding one would change the
  public API. Coast time is measured with `System.nanoTime()` (as `updateFps` already does). For
  deterministic tests (simulated frames are not rendered in real time), an additive Config field
  `frameIntervalMs` (default `0` = use the clock) is added.
- **Investigation harness.** `_investigation/harness` has a closed-loop simulator (`ClosedLoop`) and a
  zero-allocation/timing tool (`Perf`, uses ThreadMXBean). `build.sh` compiles the "ORIGINAL" from the
  v1 backup `Desktop\fixed\LineFollowerPipeline_backup_2026-10-18\` (present). For v3 the
  comparison baseline is v2 = commit `202e70f`.
- **PDF has a generator.** `_investigation/pdf_source/build_guide.py` builds
  `LineFollowerPipeline_Integration_Guide.pdf` from `guide_template.html` + README.md (tables and code
  copied from README), printed with headless Edge; `check_docs.py` cross-checks code ↔ README ↔ PDF.
  Edge, Python and PyMuPDF 1.28.2 are installed, so Phase 5 regenerates the PDF (template prose and
  the NEW/CHANGED badge lists need v3 updates).
- **Git.** HEAD is **detached** at `202e70f` (fresh clone, no local branch); the remote also has an
  unrelated tag `v4.0.1`. Commits go on branch **`v3`** (created from `202e70f`; not pushed).

## 11. Additional requirements from the user (2026-11-09) — binding for Phases 0–2

1. **Phase 1 gating (approved).** All Phase 1 behaviour is behind `glareMask`. With `glareMask = false`
   the v2 behaviour, including the 6-frame `trackHoldFrames` hold, is exactly unchanged.
2. **Coast timing (approved).** Elapsed time from `System.nanoTime()` (monotonic), or
   `Config.frameIntervalMs` when > 0 (default `0` = use the clock; tests set it for repeatability).
   The time between two frames is **clamped to 100 ms**, and a frame-count limit
   (`glareCoastMaxFrames`) ends the coast as a backup, so a pause or a dropped frame cannot use up the
   coast in one step.
3. **Partial glare on the tape.** Specular light on glossy dark tape often raises only the tape centre
   to intermediate luma (Y ≈ 150–220), below the 245 mask. Row profile: floor, edge, dark tape, bright
   centre, dark tape, edge, floor.
   - Phase 0: a variant of `glareOnTapeOnly` for dark-on-light and for a chromatic pair.
   - The detector (edge-pair / local contrast) must accept the two outer edges at the expected
     perspective width and bridge the brightened centre — not split it into two thin bands, and not
     reject it as too wide.
4. **Edge exits beat coasting.** When the base is reconstructed from the look-ahead
   (`baseReconstructed`) or the line is coasting: if the reconstructed / predicted near position lies
   in the frame-edge zone (v2 rule: `max(lineWidth, 0.12·W)`), or the look-ahead shows the line leaving
   through the left/right frame edge, steering is pinned to that edge **at once**, exactly like v2 — no
   coasting.
   - Test: a sharp turn where the line exits at the edge while glare covers the near strip ⇒ immediate
     pinning.
