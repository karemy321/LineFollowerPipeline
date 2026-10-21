# LineFollowerPipeline v2 — distractor robustness + whole-T steering

Baseline ("ORIGINAL"): commit `ec96af5` (= `origin/main`), backed up unchanged to
`Desktop\fixed\LineFollowerPipeline_backup_2026-10-18\`.

---

## 0. How the ORIGINAL pipeline works (read from the code before any change)

Line numbers refer to the original
`LineFollowerLibrary/src/main/java/com/example/linefollower/LineFollowerPipeline.java` (1955 lines).

**Input (L552–636, L838–870).** Three formats, each with a `calibrate…` and a `processFrame…` method:
grayscale `byte[]` (Y only), YUV 4:2:0 (full-res Y + packed half-res U and V) and ARGB `int[]`.
The Android app (`MainActivity.analyzeFrame`) uses **YUV 4:2:0**. It asks CameraX for 1280×720,
but with CameraX's default 4:3 aspect-ratio strategy the camera delivers **1280×960**, rotated to
portrait: **960 wide × 1280 high** (verified from the overlay geometry in the screenshots: the
24-row strip is 20 screen px tall and the V-ROI top is 533 px above it → 0.833 screen px per frame
px, which only fits 960×1280). `bottomIgnoreFrac = 0.25`, the rest are defaults. With those settings
the H-ROI is rows 936–960 (pushed up out of the ignored robot band) and the V-ROI is rows 296–936.

**Automatic colour learning, `Config.autoColor = true` (L876–929, L1012–1343).**
1. *Phase 1* (`colorCalibFrames` = 5 frames): up to 2048 (Y,U,V,column) samples per frame are
   grid-sampled from the H-ROI.
2. `learnColorModel` runs k-means in YUV space with k = 2, then 3, then 4 clusters.
   - k = 2: the **floor** is the cluster holding the majority of BOTH edge zones (5 % of the width
     at each side); the other is the line, and it must form one compact band (≥ 60 % of its samples
     in one run of columns).
   - k = 3/4 (shadows / uneven light split the floor): the line is a cluster that almost never
     reaches the edges (≤ 20 %), is compact and ≤ 70 % of the width; nearest to the centre wins.
3. A **Fisher discriminant** w = Σ⁻¹(m_line − m_nearest-floor) (pooled covariance, regularised)
   gives the single colour direction that best separates line from floor. Rejected if the
   Mahalanobis separation < 4 or the YUV distance < 12 (`ISSUE_LOW_CONTRAST`), or if another floor
   cluster projects too close to the line.
4. Every later frame is projected on w into a "line-ness" image where **floor ≈ 64 and line ≈ 192**,
   so the line is always BRIGHT on that image whatever the real colours (this is how polarity is
   handled — bright-on-dark and dark-on-bright end up identical). With YUV/ARGB input, two colours
   of equal brightness (red on green) are separable; with grayscale input only brightness differs.

**Manual mode, `autoColor = false` (L955–969).** Plain luma is thresholded directly;
`trackIsBright` chooses the polarity.

**Per-frame threshold (L1376–1401).** Otsu on the H-ROI histogram. If the two Otsu classes differ by
less than `minContrast` (24) there is "no line" (`NO_TRACK`). In colour mode the threshold is never
below `colorMinThreshold` (112, the midpoint of 64…192). The V-ROI gets its own subsampled Otsu.

**Calibration phase 2 (L890–924).** On the line-ness image the best band is searched full-width,
preferring where colour learning saw the line. After `calibFrames` (10) detections the MEDIAN
centroid becomes `steerRefX` (steering zero) and the median band width `calibWidthPx`.
`trackRefX` is derived from `steerRefX` through the perspective ratio (not measured).
A uniform floor never calibrates: k-means finds no separated pair (`ISSUE_LOW_CONTRAST`).

**Band (line) selection in the H-ROI (L1409–1478, L1544–1632).**
The strip is collapsed to `colWeight[col]` = number of track pixels per column. Runs of non-zero
columns (gaps ≤ W/50 bridged) with ≥ 1 % of the strip pixels are candidates, scored as
`pixels / (1 + (dist/scale)²) × sqrt(width ratio to expected)`.
- **Not locked** (start / re-acquire): full width, prefer near `steerRefX` and the calibrated width.
- **Locked** (`trackMemory`): only a gate of half-width `max(48, 3 × lastTrackWidthPx)` (≤ W/2)
  around the last centroid is searched, scored by distance only (**width is ignored**).
  `lastTrackWidthPx` is whatever the last accepted band measured — **unbounded**, so one wide dark
  band makes the gate (and the V-ROI box) grow to the whole frame.
- **Relock**: if a band outside the lock scores 1.5× better (near `steerRefX`, calibrated width) for
  6 consecutive frames, the lock jumps to it (6 frames = 0.1 s at 60 Hz).
- **Lost**: if nothing is in the gate, steering is pinned to ±100 internal (±50 output) on the side
  of the last position, tracking is forced to the minimum, and after 8 frames the lock is released.
  A line hidden for one frame in the MIDDLE of the image also produces a full ±50 kick.

**V-ROI look-ahead (L1637–1777).** Row by row upward from the H-ROI, inside ±`vRoiHalfWidthPx`
(80) of the previous row's centre, the run nearest the prediction is accepted if its width ≥ 0.25 ×
`lastTrackWidthPx` and its centre moved ≤ max(8, 0.75 × `lastTrackWidthPx`) (both unbounded for
the same reason). **Run width has no upper limit**, so a dark object touching the window is
accepted. Tracking stops after max(8, `hRoiThickPx`) empty rows. A least-squares fit of column vs.
row gives `tiltDeg` — **only drawn, not used**.

**Steering (L1783–1785, L936).** `internal = clamp((centroidX − steerRefX) / (W/2) × 100, ±100)`;
output `result[0] = round(0.5 × internal)` ∈ **[-50, +50]**, negative = line left. Only the H-ROI
centroid matters, so any heading that crosses the strip at the same column gives the same value.

**Tracking (L1787–1808, L940).** `intersection = |lookAheadX − trackRefX| / (W/2 × trackOffsetFullFrac) × 100`
(lookAheadX = far-weighted centroid of the V-ROI rows), raised to `100 × (1 − visibleRoad/0.8)`
when the visible road ahead is short; 100 when the line is not found or lost.
Output `result[1] = max(trackingMinValue = 5, round(100 − intersection))` ∈ **[5, 100]**.

**FrameData (L284–341)** — imgW/H, H-ROI/V-ROI/ignore fractions, steerRefXFrac, trackRefXFrac,
halfBot/TopFrac, centroidXFrac, trackWidthFrac, tiltDeg, look-ahead poly-line (≤ 48 points),
trackFound, isCalibrated, calibPhase, calibProgress, calibIssue, colorModelReady,
line/floorColorArgb, colorSeparation, threshold, steering, tracking.

**Note on geometry.** `horizonYFrac = 0.5 − tilt/fov` (L685) is a linear approximation; for an
ideal pinhole camera at 45°/60° the horizon is at 0.5 − tan45°/(2·tan30°) = −0.366, not −0.25.
It is only used for the tracking road-length term and the overlay taper; v2 keeps it unchanged
there (to not change tracking) and uses the exact pinhole model for the new heading term.

**Real-robot screenshots (`test_images/`).** Black tape on a reddish-brown wood table, a black
leather chair and light floor tiles beyond the table edge in the upper half of the view. In two
screenshots (18.40.04 (1)/(2)) the green centroid arrow is far to the right of the tape and the
V-ROI box is ~half the frame wide: the lock is on a wide patch of (darker) wood, i.e. the band
width — and with it the gate — has blown up. In 18.40.04 steering is pinned at +50 while the tape
is visible near the centre (lock lost / pinned).

---

## 1. Results in short

| | ORIGINAL | NEW (v2) |
|---|---|---|
| Distractor frame tests (12 scenarios × 2 floors) — max steering error vs the clean floor | up to **45** (6 of 12 scenarios fail per floor; lock on the object in up to 90/90 frames) | **0** in every scenario (steering and tracking), 0 wrong-lock frames |
| Heading test: line crossing the strip at the calibrated column, robot at −30…+30° | **0 for every angle** (the lecturer's observation) | −17 … +17, proportional to the angle (measured heading within 0.2° of truth) |
| Closed loop, realistic timing, 160 runs (5 tracks × 2 floors × ± distractors × 8 gain settings) | 128 completed, 4.0 % of frames locked on the wrong object | **146** completed (all 14 failures are c = 0.3, a turning-radius limit), 0.3 % wrong lock |
| Closed loop, lecturer's program c = 0.5 (40 runs, realistic) | 32 completed — all 8 failures are runs with chair-like distractors | **40** completed |
| Recovery from a 15° heading error, c = 0.5, x = 0 / x = 1 | settles in 8.9 s / 3.2 s | 6.6 s / 2.0 s, ~20 % less overshoot |
| Mean lateral error on curves without distractors | lower (offset-only is a well-tuned pure pursuit for c = 0.5) | ~0.2–1.5 cm higher on tight curves (heading adds steering into the curve) |
| `processFrame` cost, 960×1280 YUV, this PC (6 runs) | 0.87–0.98 ms | 0.89–0.94 ms (within run-to-run noise) |
| Heap allocation per 1000 frames (YUV, gray, ARGB) | 0 / 0 / 0 B | 0 / 0 / 0 B |
| Regression tests (colour pairs, manual mode, recalibration, empty floor, ranges) | 22/22 | 22/22 |
| Existing project JUnit tests (`src/test`) | 24/24 | 16/24 before the tests were updated — all 8 differences are the two intended new behaviours; **26/26** after updating them for v2 (see §6.6) |
| Output ranges in every test frame | S ∈ [−50, 50], T ∈ [5, 100] | S ∈ [−50, 50], T ∈ [5, 100] |

**Recommended starting values** for the lecturer's program (details in §7): steering gain **c = 1.0**
(not 0.5 — the output range was halved to ±50 in commit 584d353, so 0.5 now gives only half the
steering authority it had when the program was written), power **x = 0, b = 8, c = 5** first, then
**x = 1** once it follows reliably.

---

## 2. What changed, file by file

### `LineFollowerLibrary/src/main/java/com/example/linefollower/LineFollowerPipeline.java` (1955 → 2619 lines)

Line numbers are in the NEW file.

| Lines | Change |
|---|---|
| 29–49 | Class comment: algorithm steps updated (predicted gate, hold, distractor rejection, look-ahead width rules, steering = offset + heading). |
| 193–255 | **New Config options**: `rejectDistractors` (202), `bandMaxWidthMult` (207), `bandMinWidthMult` (212), `trackHoldFrames` (219), `steerOffsetWeight` (235), `steerHeadingWeight` (236), `steerLookAhead` (244), `steerHeadingMinRows` (250), `steerHeadingCalibMaxDeg` (255), each with a comment. |
| 327–341 | `ControlOutput` (internal): offset/heading terms, heading, confidence, held flag, rejected count. |
| 418–431 | **New FrameData fields**: `steerOffsetTerm`, `steerHeadingTerm`, `headingDeg`, `headingConfidence`, `lineHeld`, `rejectedBands`. No existing member removed or renamed. |
| 483–510 | New state: calibrated line-ness level (`calibLevel`), calibration heading zero, prediction (`centroidVel`, `lastImgSlope`), hold counter, last outputs. |
| 570–605 | New buffers/state: per-row floor geometry `rowFwd`/`rowScale`, floor-fit sums, trace results, `colLevel` (per-column line-ness sum). |
| 847–872 | `init()`: exact pinhole floor geometry per row from `cameraTiltDeg` / `verticalFovDeg` (allocated once). |
| 1051–1118 | `calibrateBound()` phase 2: width test against the colour-learned width, records the median line-ness (`calibLevel`) and — when the heading term is on — traces the look-ahead and records the median heading as the steering zero (`headingRefDeg`). |
| 1140–1146 | `processBound()`: marks callback as a processing frame (new FrameData fields are 0 during calibration). Output scaling (×0.5 → ±50) and the tracking floor are unchanged. |
| 1610–1645 | `buildColumnProfile()` also sums the line-ness per column; new `bandLevel()`. |
| 1655–1806 | `findBand()` gets `filter` / `trim` flags (with both false it is byte-for-byte the v1 code path). New `setBandLimits()`, `levelFactor()`, `considerRun()` (width test, line-ness weighting, pixel cap), `trimBand()` (strong core / continuing edge), `sumRange()`. |
| 1864–2146 | `runPipeline()` rewritten around the same stages: slant-corrected expected width, gate centred on the **predicted** position and capped by the acceptable width, continuity-based locked scoring, stricter relock (1949–2001), velocity update, **hold** of the last output when the line vanishes mid-image (2012–2047), Stage 3 moved to `traceLookAhead()`, **new steering** (2071–2101), hold of outputs (2123–2130), new output fields. |
| 2155–2175 | `lookAheadThreshold()` — the v1 V-ROI Otsu code, unchanged, moved into a method (calibration now uses it too). |
| 2195–2386 | `traceLookAhead()` — the v1 row tracker, unchanged when `filter` is false; with `filter`: prediction from the local drift (clamped ±2 col/row), per-row expected width from perspective × slant (slant fixed per frame), scan window widened to at least half the expected width + 2 × the bridged gap (at 960 px the tape is wider than the default ±80 px window), runs > 1.3× expected or cut by the window are reduced to the edge that continues the line, a run filling the whole window is rejected; also accumulates the floor-plane points and a near-strip image slope. |
| 2392–2460 | `addFloorPoint()` and `computeHeading()` — parabola fit on the floor, tangent at the strip, confidence from rows / span / fit residual. |
| 2463–2473 | `resetLineModel()` (used by `init()` and `requestRecalibration()`). |
| 2524–2533 | `fireCallback()` fills the new FrameData fields. |

With `rejectDistractors = false` and `steerHeadingWeight = 0` the new library is **bit-identical** to the original: 10 closed-loop runs (≈ 55 000 frames, both resolutions tried) produced identical CSV rows (§6.1).

### `README.md` (486 → 538 lines)

| Lines | Change |
|---|---|
| 140–141 | `cameraTiltDeg` / `verticalFovDeg` now also drive the heading — measure them. |
| 147–148 | Track memory: predicted position; gate capped. |
| 155–171 | **New section "Distractor Rejection"** + table of `rejectDistractors`, `bandMaxWidthMult`, `bandMinWidthMult`, `trackHoldFrames`. |
| 173–196 | **New section "Steering from the Whole ⊥"**: formula, ψ / ψ₀ / confidence, table of `steerOffsetWeight`, `steerHeadingWeight`, `steerLookAhead`, `steerHeadingMinRows`, `steerHeadingCalibMaxDeg`. |
| 359–364 | FrameData table: the six new fields. |
| 501–503 | Behaviour reference: robot at an angle, distractor beside the line, line hidden. |
| 515 | Note: heading needs correct camera geometry; the app runs at 960 × 1280. |

### Not changed
`RoboPhoneApp/` (builds unchanged against the new library, see §8), `line_follower_block.js`, `line_follower.xml`, the PDF. (The unit tests in `LineFollowerLibrary/src/test/` were updated afterwards for v2, see §6.6.)

---

## 3. How distractors are now rejected (plain words)

The colour model makes a black chair look exactly as "line-like" as black tape, so colour cannot
tell them apart. v2 uses what calibration learned about **this** line instead:

1. **It must be as wide as the line.** Calibration measures the tape's width in the strip. A band
   wider than 2× that (corrected for how slanted the line currently crosses the strip) or narrower
   than 0.3× is not the line. A chair seat, a bag, a dark area at the frame edge or a shadow is far
   wider than tape.
2. **It must be as line-like as the line.** Calibration also records how strongly the tape's pixels
   pass the threshold. Shadowed wood only just passes it, the tape passes it clearly; among
   acceptable bands the more line-like one wins.
3. **It must continue the line.** While locked, the strip is only searched near where the line is
   *predicted* to be (last position + recent motion), and the nearest acceptable band on the scale of
   one line width wins — a blob 2 cm beside the tape no longer competes. The gate can never become
   wider than an acceptable band, so it cannot grow to the whole frame (the screenshot failure).
4. **A line merged with an object is cut back.** When a blob or shadow touches the tape, the band
   becomes too wide. v2 then keeps only the part that continues the line: the columns that are as
   line-like as the calibrated tape, or else the band edge that sits where the line's edge was last
   frame (the object is on the other side). The look-ahead tracker does the same on every row (with
   the width expected at that row's distance), so it can neither jump onto nor slowly "creep" into
   a chair leg or seat beside the line.
5. **A hidden line is held, not chased.** If the line vanishes in the middle of the image, the last
   steering and tracking are held for up to 6 frames (0.1 s at 60 Hz) instead of jumping to ±50
   (v1 did that on a single bad frame). A line that leaves at the frame edge — a real sharp turn — is
   still pinned to that edge at once, exactly like v1.
6. **The lock is not stolen.** The v1 "relock" moved the lock to any better-looking band after 6
   frames (0.1 s at 60 Hz) — a chair leg near the centre during a turn qualified. v2 only does that
   quickly if the current lock itself looks wrong; a valid-looking lock needs 3× longer.

## 4. How the new steering is computed

```
offset   = steerOffsetWeight  × (centroidX − steerRefX) / (W/2) × 100                     (v1)
heading  = steerHeadingWeight × conf × 100 × L × tan(ψ − ψ₀) / (H-ROI floor half-width)
L        = steerLookAhead × (floor distance camera → H-ROI)                   (default 0.5)
steering = clamp(offset + heading, −100, +100);   output = round(steering / 2) ∈ [−50, +50]
```

* **ψ** — direction of the line on the floor where it crosses the strip, relative to the robot
  (+ = line heads right). The H-ROI centroid and every look-ahead row on the line are projected onto
  the floor with an exact pinhole model of the camera (`cameraTiltDeg`, `verticalFovDeg`), points up
  to 1.5× the strip distance beyond the strip are fitted with a parabola x = A + B·u + C·u², and
  ψ = atan(B), the tangent at the strip.
* **ψ₀** — the median ψ during calibration (phone yaw on its mount); ignored if > 10°.
  A robot correctly placed on a straight line reads 0 (verified: 0 ± 0 in all regression scenes).
* **conf** ∈ [0, 1] — full with ≥ 24 rows spanning ≥ half the strip distance and the points within a
  quarter line width of the parabola; fades to 0 with too little line ahead (end of line, line
  leaving the frame) or a line that is not a smooth curve (a 90° corner). With conf = 0 the steering
  is exactly v1.
* **Interpretation** — offset + heading is the lateral position, in v1 steering units, of a
  pure-pursuit target point L beyond the strip along the line. Units and calibration are the same as
  v1, so the same gain c means the same thing on a straight line without heading error.

**Why this form (what was tried).** All variants were run through the closed-loop simulator:
* A straight-line fit through the whole look-ahead (the existing `tiltDeg` idea): on curves it
  measures the *chord*, adds a second "turn in" on top of the offset and makes the robot cut inside
  (oval: mean lateral error 0.3 → 5 cm).
* Parabola tangent extrapolated back to the camera (removes the curve bias in theory): flips sign
  approaching a 90° corner (the line ahead is an L, not a parabola) → robot stuck oscillating.
  Adding a fit-quality test and a fit window fixed the corner, but it was not better than the
  tangent at the strip and is more fragile.
* Tangent at the strip with L = 0.5 / 0.75 / 1.0: larger L = faster recovery from heading errors
  but more curve error and, with high gain plus latency, weaving. **L = 0.5** was the best balance.

An honest observation from the simulation: the v1 offset-only steering is already a pure-pursuit
controller with a ~19 cm look-ahead and is stable and accurate on curves for c ≈ 0.5–1. The heading
term clearly helps where the lecturer saw the problem — a robot at an angle to the line straightens
out faster and with less overshoot — and is required to distinguish headings at all, but it costs a
little accuracy on tight curves. Setting `steerHeadingWeight = 0` restores v1 steering exactly if the
real robot behaves better without it.

---

## 5. Test setup (all in `_investigation/`, plain Java, no dependencies)

**Baseline.** The ORIGINAL library is compiled from the backup, the NEW one from the working copy;
the same harness (compiled against the original API, v2 fields read by reflection) runs both.

**Camera / frames.** 960 × 1280 YUV 4:2:0 (the app's real analysis size), app Config
(`bottomIgnoreFrac = 0.25`, tilt 45°, vFOV 60°). Pinhole camera 20 cm above the floor, tilted 45°,
60° vertical FOV, 8 cm ahead of the axle, 1 cm right of centre; 25 mm tape appears ~130 px wide in
the strip (screenshots: ~110 px). Rendered by ray-casting a 2 mm floor map per pixel, with image
vignetting (−25 % corners), far rows dimmer (−15 %), Gaussian sensor noise (σ = 4 Y, 2 UV),
and world-space uneven lighting (×0.72 … ×1.17).

**Floors.** White tape on dark carpet (`white_on_dark`), black tape on reddish-brown wood with
grain (`black_on_wood`); regression also white on brown wood, black on white, blue on yellow, red on
green (equal brightness).

**Distractors** (same colour as the tape: black on wood, white on dark): blobs (bag / seat shadow)
beside the line with 2–6 cm gaps, upright chair legs rendered in 3-D (they lean outward in the
image like real legs), a 0.5 m dark (or bright) region beyond a "table edge" 9 cm from the line,
and a blob on the outside of every turn's apex. They come into view as the robot drives.

**Tracks.** straight 2 m; gentle curve R 0.8 m 90°; S-curve R 0.4 m ±90°; sharp ~90° corner
(R 4 cm); oval 2 × 1 m + 2 × R 0.35 m semicircles (full lap 4.2 m). Each run calibrates at the start
(robot correctly placed), finishes 0.3 m before the end of the tape (one lap for the oval), and
fails if the axle is > 12 cm from the line ("left the track") or the robot stalls.

**Robot / control.** Differential drive, wheelbase 13 cm, 1.0 m/s at power 100 (power 5 = 5 cm/s),
first-order motor lag 80 ms. The lecturer's program exactly:
`motor steering = (0 − Steering) × c` with "Invert Motors" (= turn toward the line),
`motor power = round(x × (Power / 8) + 5)`; "move steering" modelled like the LEGO block (inner
wheel = power × (1 − |steer|/50)). Gains c ∈ {0.3, 0.5, 1.0, 2.0}, x ∈ {0, 1}.

**Regimes.** `ideal`: 33 ms latency, control every frame (60 Hz). `real`: 150 ms latency
(camera + Bluetooth + motors), Blockly loop 20 Hz, motor dead zone 2. `stress`: robot 3× faster,
200 ms, 15 Hz, dead zone 3. `tilt35`: real camera tilt 35° while the library is configured for 45°.
Recovery tests: robot turned ±15° / 30° after calibration.

**Metrics per run.** completed / left the track; mean and max lateral error of the axle; RMS and
max heading error; steering sign changes per second (|S| > 1) and steering standard deviation
(oscillation); % frames with the line lost; % frames locked on the wrong object (detected centroid
> 80 px from the true tape column in the strip); settle time after a disturbance.

---

## 6. Before / after results

### 6.1 Equivalence check
New library with `rejectDistractors = false, steerHeadingWeight = 0` vs original, 10 closed-loop
runs (all tracks, with/without distractors): **identical** output CSV (every metric to the last digit).

### 6.2 Closed-loop simulator (the most important test)


### Closed-loop summary per regime (all tracks × both polarities × with/without distractors × all gains)

| regime | runs | completed orig → new | mean lateral err cm | worst lateral err cm | RMS heading err ° | steering sign changes /s | lost % of frames | wrong-lock % of frames |
|---|---:|---|---|---|---|---|---|---|
| ideal | 160 | 124 → 146 | 1.26 → 1.54 | 12.1 → 12.0 | 10.7 → 6.9 | 0.05 → 0.07 | 7.4 → 4.8 | 4.17 → 0.24 |
| real | 160 | 128 → 146 | 1.24 → 1.49 | 12.1 → 12.1 | 8.5 → 7.0 | 0.06 → 0.09 | 7.3 → 4.8 | 4.04 → 0.31 |
| stress | 80 | 68 → 72 | 1.05 → 1.36 | 12.1 → 12.2 | 10.0 → 11.3 | 0.37 → 0.42 | 8.3 → 8.4 | 4.60 → 0.92 |
| tilt35 | 40 | 34 → 36 | 1.57 → 2.29 | 12.0 → 12.1 | 9.6 → 8.4 | 0.04 → 0.10 | 4.9 → 1.1 | 4.04 → 0.03 |

#### Regime `real` by track and distractors (all gains, both polarities)

| track | distractors | completed orig → new | mean lateral err cm | worst lateral err cm | sign changes /s | wrong-lock % |
|---|---|---|---|---|---|---|
| straight | no | 16/16 → 16/16 | 0.00 → 0.00 | 0.0 → 0.0 | 0.00 → 0.00 | 0.00 → 0.00 |
| straight | yes | 16/16 → 16/16 | 0.00 → 0.00 | 0.0 → 0.0 | 0.00 → 0.00 | 0.00 → 0.00 |
| gentle | no | 16/16 → 16/16 | 0.92 → 1.14 | 2.7 → 3.1 | 0.00 → 0.02 | 0.00 → 0.00 |
| gentle | yes | 12/16 → 16/16 | 1.41 → 1.14 | 12.0 → 3.1 | 0.15 → 0.02 | 6.54 → 0.00 |
| scurve | no | 16/16 → 16/16 | 1.69 → 1.84 | 8.0 → 5.9 | 0.04 → 0.08 | 0.00 → 0.00 |
| scurve | yes | 12/16 → 16/16 | 1.58 → 1.84 | 12.1 → 5.9 | 0.17 → 0.08 | 9.12 → 0.00 |
| sharp90 | no | 12/16 → 12/16 | 1.51 → 1.71 | 12.0 → 12.1 | 0.01 → 0.26 | 0.39 → 0.18 |
| sharp90 | yes | 8/16 → 12/16 | 1.80 → 1.74 | 12.1 → 12.0 | 0.10 → 0.29 | 12.53 → 2.05 |
| oval | no | 12/16 → 14/16 | 1.78 → 2.97 | 12.0 → 12.0 | 0.01 → 0.04 | 0.02 → 0.00 |
| oval | yes | 8/16 → 12/16 | 1.67 → 2.56 | 12.0 → 12.0 | 0.09 → 0.10 | 11.82 → 0.87 |

#### Regime `real` by steering gain c and power mode x (all tracks, both polarities, with/without distractors)

| c | x | completed orig → new | mean lateral err cm | worst lateral err cm | sign changes /s | steering std |
|---|---|---|---|---|---|---|
| 0.30 | 0 | 10/20 → 12/20 | 1.47 → 1.03 | 12.1 → 12.0 | 0.01 → 0.04 | 16.9 → 18.4 |
| 0.30 | 1 | 10/20 → 14/20 | 1.92 → 1.31 | 12.1 → 12.1 | 0.03 → 0.07 | 17.5 → 18.5 |
| 0.50 | 0 | 16/20 → 20/20 | 0.69 → 0.91 | 12.0 → 5.4 | 0.06 → 0.03 | 13.5 → 12.3 |
| 0.50 | 1 | 16/20 → 20/20 | 0.86 → 1.04 | 12.0 → 5.2 | 0.08 → 0.09 | 14.8 → 12.8 |
| 1.00 | 0 | 18/20 → 20/20 | 1.00 → 1.68 | 12.0 → 9.2 | 0.03 → 0.05 | 8.9 → 6.6 |
| 1.00 | 1 | 18/20 → 20/20 | 1.20 → 1.77 | 12.0 → 9.2 | 0.06 → 0.07 | 9.7 → 7.5 |
| 2.00 | 0 | 20/20 → 20/20 | 1.31 → 2.06 | 8.5 → 10.3 | 0.05 → 0.05 | 4.8 → 3.8 |
| 2.00 | 1 | 20/20 → 20/20 | 1.44 → 2.16 | 8.4 → 10.4 | 0.12 → 0.32 | 5.7 → 5.2 |

#### Regime `ideal` by track and distractors (all gains, both polarities)

| track | distractors | completed orig → new | mean lateral err cm | worst lateral err cm | sign changes /s | wrong-lock % |
|---|---|---|---|---|---|---|
| straight | no | 16/16 → 16/16 | 0.00 → 0.00 | 0.0 → 0.0 | 0.00 → 0.00 | 0.00 → 0.00 |
| straight | yes | 16/16 → 16/16 | 0.00 → 0.00 | 0.0 → 0.0 | 0.00 → 0.00 | 0.00 → 0.00 |
| gentle | no | 16/16 → 16/16 | 0.93 → 1.15 | 2.7 → 3.1 | 0.00 → 0.00 | 0.00 → 0.00 |
| gentle | yes | 12/16 → 16/16 | 1.43 → 1.16 | 12.0 → 3.1 | 0.11 → 0.00 | 6.59 → 0.00 |
| scurve | no | 16/16 → 16/16 | 1.67 → 1.89 | 7.3 → 5.9 | 0.04 → 0.07 | 0.00 → 0.00 |
| scurve | yes | 12/16 → 16/16 | 1.62 → 1.90 | 12.0 → 5.9 | 0.18 → 0.07 | 9.18 → 0.00 |
| sharp90 | no | 12/16 → 12/16 | 1.53 → 1.75 | 12.0 → 12.0 | 0.00 → 0.23 | 0.35 → 0.15 |
| sharp90 | yes | 4/16 → 12/16 | 1.92 → 1.77 | 12.1 → 12.0 | 0.12 → 0.27 | 13.69 → 1.98 |
| oval | no | 12/16 → 14/16 | 1.78 → 3.02 | 12.0 → 12.0 | 0.01 → 0.03 | 0.02 → 0.01 |
| oval | yes | 8/16 → 12/16 | 1.69 → 2.76 | 12.0 → 12.0 | 0.08 → 0.04 | 11.91 → 0.28 |

#### Regime `ideal` by steering gain c and power mode x (all tracks, both polarities, with/without distractors)

| c | x | completed orig → new | mean lateral err cm | worst lateral err cm | sign changes /s | steering std |
|---|---|---|---|---|---|---|
| 0.30 | 0 | 10/20 → 12/20 | 1.47 → 1.15 | 12.0 → 12.0 | 0.01 → 0.03 | 16.9 → 18.0 |
| 0.30 | 1 | 10/20 → 14/20 | 1.91 → 1.45 | 12.1 → 12.0 | 0.03 → 0.06 | 17.5 → 17.3 |
| 0.50 | 0 | 16/20 → 20/20 | 0.71 → 0.93 | 12.0 → 5.6 | 0.06 → 0.03 | 13.4 → 12.2 |
| 0.50 | 1 | 16/20 → 20/20 | 0.92 → 1.08 | 12.0 → 5.6 | 0.07 → 0.08 | 14.6 → 12.6 |
| 1.00 | 0 | 18/20 → 20/20 | 0.99 → 1.69 | 12.0 → 9.2 | 0.05 → 0.07 | 8.8 → 6.6 |
| 1.00 | 1 | 18/20 → 20/20 | 1.21 → 1.80 | 12.0 → 9.2 | 0.08 → 0.12 | 9.8 → 7.2 |
| 2.00 | 0 | 18/20 → 20/20 | 1.36 → 2.07 | 8.9 → 10.3 | 0.05 → 0.07 | 5.6 → 3.6 |
| 2.00 | 1 | 18/20 → 20/20 | 1.48 → 2.16 | 8.8 → 10.3 | 0.08 → 0.11 | 6.6 → 4.2 |

#### Regime `stress` by steering gain c and power mode x (all tracks, both polarities, with/without distractors)

| c | x | completed orig → new | mean lateral err cm | worst lateral err cm | sign changes /s | steering std |
|---|---|---|---|---|---|---|
| 0.50 | 0 | 18/20 → 20/20 | 0.49 → 0.98 | 12.1 → 8.7 | 0.09 → 0.13 | 13.6 → 11.6 |
| 0.50 | 1 | 14/20 → 18/20 | 1.25 → 1.14 | 12.1 → 12.2 | 0.26 → 0.30 | 19.1 → 18.5 |
| 1.00 | 0 | 20/20 → 20/20 | 0.94 → 1.78 | 8.1 → 9.8 | 0.19 → 0.36 | 9.2 → 9.2 |
| 1.00 | 1 | 16/20 → 14/20 | 1.51 → 1.55 | 12.1 → 12.1 | 0.93 → 0.89 | 22.5 → 26.9 |

#### Regime `tilt35` by steering gain c and power mode x (all tracks, both polarities, with/without distractors)

| c | x | completed orig → new | mean lateral err cm | worst lateral err cm | sign changes /s | steering std |
|---|---|---|---|---|---|---|
| 0.50 | 0 | 8/10 → 10/10 | 1.06 → 1.96 | 12.0 → 9.8 | 0.03 → 0.03 | 13.2 → 11.8 |
| 0.50 | 1 | 8/10 → 10/10 | 1.30 → 2.16 | 12.0 → 9.8 | 0.06 → 0.06 | 14.2 → 12.2 |
| 1.00 | 0 | 9/10 → 8/10 | 1.84 → 2.46 | 12.0 → 12.0 | 0.02 → 0.13 | 8.5 → 6.9 |
| 1.00 | 1 | 9/10 → 8/10 | 2.08 → 2.58 | 12.0 → 12.1 | 0.07 → 0.19 | 9.4 → 7.8 |

#### Lecturer's program (c = 0.5), regime `real` — every run

| track | floor | distractors | x | result orig → new | mean lat cm | max lat cm | max heading err ° | sign changes /s | lost % | wrong-lock % |
|---|---|---|---|---|---|---|---|---|---|---|
| straight | white_on_dark | no | 0 | DONE → DONE | 0.00 → 0.00 | 0.0 → 0.0 | 0 → 0 | 0.00 → 0.00 | 0.0 → 0.0 | 0.0 → 0.0 |
| straight | white_on_dark | no | 1 | DONE → DONE | 0.00 → 0.00 | 0.0 → 0.0 | 0 → 0 | 0.00 → 0.00 | 0.0 → 0.0 | 0.0 → 0.0 |
| straight | white_on_dark | yes | 0 | DONE → DONE | 0.00 → 0.00 | 0.0 → 0.0 | 0 → 0 | 0.00 → 0.00 | 0.0 → 0.0 | 0.0 → 0.0 |
| straight | white_on_dark | yes | 1 | DONE → DONE | 0.00 → 0.00 | 0.0 → 0.0 | 0 → 0 | 0.00 → 0.00 | 0.0 → 0.0 | 0.0 → 0.0 |
| straight | black_on_wood | no | 0 | DONE → DONE | 0.00 → 0.00 | 0.0 → 0.0 | 0 → 0 | 0.00 → 0.00 | 0.0 → 0.0 | 0.0 → 0.0 |
| straight | black_on_wood | no | 1 | DONE → DONE | 0.00 → 0.00 | 0.0 → 0.0 | 0 → 0 | 0.00 → 0.00 | 0.0 → 0.0 | 0.0 → 0.0 |
| straight | black_on_wood | yes | 0 | DONE → DONE | 0.00 → 0.00 | 0.0 → 0.0 | 0 → 0 | 0.00 → 0.00 | 0.0 → 0.0 | 0.0 → 0.0 |
| straight | black_on_wood | yes | 1 | DONE → DONE | 0.00 → 0.00 | 0.0 → 0.0 | 0 → 0 | 0.00 → 0.00 | 0.0 → 0.0 | 0.0 → 0.0 |
| gentle | white_on_dark | no | 0 | DONE → DONE | 0.37 → 0.56 | 0.9 → 0.9 | 2 → 4 | 0.00 → 0.00 | 0.0 → 0.0 | 0.0 → 0.0 |
| gentle | white_on_dark | no | 1 | DONE → DONE | 0.37 → 0.59 | 0.8 → 0.9 | 2 → 4 | 0.00 → 0.00 | 0.0 → 0.0 | 0.0 → 0.0 |
| gentle | white_on_dark | yes | 0 | DONE → DONE | 0.77 → 0.56 | 3.0 → 0.9 | 15 → 4 | 0.05 → 0.00 | 0.4 → 0.0 | 4.6 → 0.0 |
| gentle | white_on_dark | yes | 1 | DONE → DONE | 0.85 → 0.59 | 2.9 → 0.9 | 15 → 4 | 0.11 → 0.00 | 0.8 → 0.0 | 6.7 → 0.0 |
| gentle | black_on_wood | no | 0 | DONE → DONE | 0.37 → 0.55 | 0.9 → 0.9 | 2 → 4 | 0.00 → 0.00 | 0.0 → 0.0 | 0.0 → 0.0 |
| gentle | black_on_wood | no | 1 | DONE → DONE | 0.38 → 0.57 | 0.8 → 0.9 | 2 → 4 | 0.00 → 0.00 | 0.0 → 0.0 | 0.0 → 0.0 |
| gentle | black_on_wood | yes | 0 | DONE → DONE | 0.77 → 0.55 | 3.0 → 0.9 | 15 → 4 | 0.05 → 0.00 | 0.4 → 0.0 | 4.6 → 0.0 |
| gentle | black_on_wood | yes | 1 | DONE → DONE | 0.85 → 0.57 | 2.9 → 0.9 | 15 → 4 | 0.11 → 0.00 | 0.8 → 0.0 | 6.7 → 0.0 |
| scurve | white_on_dark | no | 0 | DONE → DONE | 0.57 → 1.39 | 2.1 → 2.8 | 11 → 17 | 0.02 → 0.05 | 0.0 → 0.0 | 0.0 → 0.0 |
| scurve | white_on_dark | no | 1 | DONE → DONE | 0.45 → 1.41 | 1.6 → 2.8 | 9 → 16 | 0.05 → 0.11 | 0.0 → 0.0 | 0.0 → 0.0 |
| scurve | white_on_dark | yes | 0 | DONE → DONE | 0.80 → 1.39 | 3.2 → 2.8 | 11 → 17 | 0.27 → 0.05 | 0.0 → 0.0 | 3.4 → 0.0 |
| scurve | white_on_dark | yes | 1 | DONE → DONE | 0.50 → 1.41 | 1.8 → 2.8 | 9 → 16 | 0.05 → 0.11 | 0.0 → 0.0 | 2.2 → 0.0 |
| scurve | black_on_wood | no | 0 | DONE → DONE | 0.57 → 1.40 | 2.1 → 2.8 | 11 → 17 | 0.02 → 0.05 | 0.0 → 0.0 | 0.0 → 0.0 |
| scurve | black_on_wood | no | 1 | DONE → DONE | 0.45 → 1.42 | 1.7 → 2.8 | 9 → 16 | 0.05 → 0.11 | 0.0 → 0.0 | 0.0 → 0.0 |
| scurve | black_on_wood | yes | 0 | DONE → DONE | 0.80 → 1.40 | 3.3 → 2.8 | 11 → 17 | 0.27 → 0.05 | 0.0 → 0.0 | 3.4 → 0.0 |
| scurve | black_on_wood | yes | 1 | DONE → DONE | 0.68 → 1.42 | 2.5 → 2.8 | 9 → 16 | 0.05 → 0.11 | 0.0 → 0.0 | 4.4 → 0.0 |
| sharp90 | white_on_dark | no | 0 | DONE → DONE | 0.95 → 0.80 | 5.1 → 5.4 | 48 → 46 | 0.00 → 0.04 | 16.2 → 12.7 | 0.3 → 0.2 |
| sharp90 | white_on_dark | no | 1 | DONE → DONE | 1.15 → 1.05 | 4.9 → 5.2 | 49 → 47 | 0.08 → 0.17 | 29.9 → 25.2 | 0.8 → 0.4 |
| sharp90 | white_on_dark | yes | 0 | LEFT@1.14m → DONE | 1.88 → 1.04 | 12.0 → 5.4 | 48 → 46 | 0.09 → 0.12 | 23.7 → 13.1 | 19.0 → 1.6 |
| sharp90 | white_on_dark | yes | 1 | LEFT@1.14m → DONE | 2.82 → 1.30 | 12.0 → 5.2 | 49 → 47 | 0.17 → 0.33 | 39.1 → 25.2 | 23.4 → 2.6 |
| sharp90 | black_on_wood | no | 0 | DONE → DONE | 0.94 → 0.80 | 5.1 → 5.4 | 48 → 46 | 0.00 → 0.04 | 16.2 → 12.6 | 0.3 → 0.1 |
| sharp90 | black_on_wood | no | 1 | DONE → DONE | 1.15 → 1.06 | 4.9 → 5.2 | 48 → 46 | 0.08 → 0.17 | 29.9 → 24.4 | 0.8 → 0.3 |
| sharp90 | black_on_wood | yes | 0 | LEFT@1.14m → DONE | 1.88 → 0.90 | 12.0 → 5.4 | 48 → 46 | 0.09 → 0.12 | 23.4 → 13.1 | 19.3 → 0.6 |
| sharp90 | black_on_wood | yes | 1 | LEFT@1.14m → DONE | 2.82 → 1.17 | 12.0 → 5.2 | 48 → 46 | 0.17 → 0.33 | 40.1 → 24.8 | 22.5 → 0.9 |
| oval | white_on_dark | no | 0 | DONE → DONE | 0.27 → 1.73 | 1.4 → 3.8 | 7 → 12 | 0.02 → 0.03 | 0.0 → 0.0 | 0.0 → 0.0 |
| oval | white_on_dark | no | 1 | DONE → DONE | 0.22 → 2.05 | 1.3 → 3.8 | 6 → 12 | 0.04 → 0.08 | 0.0 → 0.0 | 0.1 → 0.0 |
| oval | white_on_dark | yes | 0 | LEFT@1.51m → DONE | 1.33 → 1.73 | 12.0 → 3.8 | 33 → 12 | 0.12 → 0.03 | 12.6 → 0.0 | 16.3 → 0.0 |
| oval | white_on_dark | yes | 1 | LEFT@1.51m → DONE | 2.12 → 2.05 | 12.0 → 3.8 | 33 → 12 | 0.27 → 0.08 | 21.9 → 0.0 | 23.5 → 0.0 |
| oval | black_on_wood | no | 0 | DONE → DONE | 0.28 → 1.73 | 1.4 → 3.8 | 7 → 12 | 0.02 → 0.03 | 0.0 → 0.0 | 0.1 → 0.0 |
| oval | black_on_wood | no | 1 | DONE → DONE | 0.22 → 2.05 | 1.3 → 3.8 | 6 → 12 | 0.04 → 0.08 | 0.0 → 0.0 | 0.0 → 0.0 |
| oval | black_on_wood | yes | 0 | LEFT@1.51m → DONE | 1.33 → 1.73 | 12.0 → 3.8 | 33 → 12 | 0.12 → 0.03 | 12.6 → 0.0 | 16.3 → 0.0 |
| oval | black_on_wood | yes | 1 | LEFT@1.51m → DONE | 2.12 → 2.05 | 12.0 → 3.8 | 33 → 12 | 0.27 → 0.08 | 21.8 → 0.0 | 23.6 → 0.0 |

#### Lecturer's program (c = 0.5), regime `ideal` — every run

| track | floor | distractors | x | result orig → new | mean lat cm | max lat cm | max heading err ° | sign changes /s | lost % | wrong-lock % |
|---|---|---|---|---|---|---|---|---|---|---|
| straight | white_on_dark | no | 0 | DONE → DONE | 0.00 → 0.00 | 0.0 → 0.0 | 0 → 0 | 0.00 → 0.00 | 0.0 → 0.0 | 0.0 → 0.0 |
| straight | white_on_dark | no | 1 | DONE → DONE | 0.00 → 0.00 | 0.0 → 0.0 | 0 → 0 | 0.00 → 0.00 | 0.0 → 0.0 | 0.0 → 0.0 |
| straight | white_on_dark | yes | 0 | DONE → DONE | 0.00 → 0.00 | 0.0 → 0.0 | 0 → 0 | 0.00 → 0.00 | 0.0 → 0.0 | 0.0 → 0.0 |
| straight | white_on_dark | yes | 1 | DONE → DONE | 0.00 → 0.00 | 0.0 → 0.0 | 0 → 0 | 0.00 → 0.00 | 0.0 → 0.0 | 0.0 → 0.0 |
| straight | black_on_wood | no | 0 | DONE → DONE | 0.00 → 0.00 | 0.0 → 0.0 | 0 → 0 | 0.00 → 0.00 | 0.0 → 0.0 | 0.0 → 0.0 |
| straight | black_on_wood | no | 1 | DONE → DONE | 0.00 → 0.00 | 0.0 → 0.0 | 0 → 0 | 0.00 → 0.00 | 0.0 → 0.0 | 0.0 → 0.0 |
| straight | black_on_wood | yes | 0 | DONE → DONE | 0.00 → 0.00 | 0.0 → 0.0 | 0 → 0 | 0.00 → 0.00 | 0.0 → 0.0 | 0.0 → 0.0 |
| straight | black_on_wood | yes | 1 | DONE → DONE | 0.00 → 0.00 | 0.0 → 0.0 | 0 → 0 | 0.00 → 0.00 | 0.0 → 0.0 | 0.0 → 0.0 |
| gentle | white_on_dark | no | 0 | DONE → DONE | 0.37 → 0.57 | 0.9 → 0.9 | 3 → 4 | 0.00 → 0.00 | 0.0 → 0.0 | 0.0 → 0.0 |
| gentle | white_on_dark | no | 1 | DONE → DONE | 0.38 → 0.60 | 0.8 → 0.9 | 2 → 4 | 0.00 → 0.00 | 0.0 → 0.0 | 0.0 → 0.0 |
| gentle | white_on_dark | yes | 0 | DONE → DONE | 0.77 → 0.57 | 2.9 → 0.9 | 14 → 4 | 0.05 → 0.00 | 0.4 → 0.0 | 4.6 → 0.0 |
| gentle | white_on_dark | yes | 1 | DONE → DONE | 0.87 → 0.60 | 2.9 → 0.9 | 14 → 4 | 0.12 → 0.00 | 0.8 → 0.0 | 7.0 → 0.0 |
| gentle | black_on_wood | no | 0 | DONE → DONE | 0.38 → 0.57 | 0.9 → 0.9 | 3 → 4 | 0.00 → 0.00 | 0.0 → 0.0 | 0.0 → 0.0 |
| gentle | black_on_wood | no | 1 | DONE → DONE | 0.38 → 0.60 | 0.8 → 0.9 | 2 → 4 | 0.00 → 0.00 | 0.0 → 0.0 | 0.0 → 0.0 |
| gentle | black_on_wood | yes | 0 | DONE → DONE | 0.78 → 0.57 | 2.9 → 0.9 | 14 → 4 | 0.05 → 0.00 | 0.4 → 0.0 | 4.6 → 0.0 |
| gentle | black_on_wood | yes | 1 | DONE → DONE | 0.87 → 0.60 | 2.9 → 0.9 | 14 → 4 | 0.12 → 0.00 | 0.8 → 0.0 | 6.9 → 0.0 |
| scurve | white_on_dark | no | 0 | DONE → DONE | 0.62 → 1.43 | 2.2 → 2.8 | 11 → 17 | 0.02 → 0.05 | 0.0 → 0.0 | 0.0 → 0.0 |
| scurve | white_on_dark | no | 1 | DONE → DONE | 0.57 → 1.50 | 2.0 → 2.8 | 10 → 17 | 0.05 → 0.12 | 0.0 → 0.0 | 0.0 → 0.0 |
| scurve | white_on_dark | yes | 0 | DONE → DONE | 0.81 → 1.43 | 3.1 → 2.8 | 11 → 17 | 0.27 → 0.05 | 0.0 → 0.0 | 3.2 → 0.0 |
| scurve | white_on_dark | yes | 1 | DONE → DONE | 0.78 → 1.50 | 2.8 → 2.8 | 10 → 17 | 0.05 → 0.12 | 0.0 → 0.0 | 4.1 → 0.0 |
| scurve | black_on_wood | no | 0 | DONE → DONE | 0.62 → 1.44 | 2.2 → 2.8 | 11 → 17 | 0.02 → 0.05 | 0.0 → 0.0 | 0.0 → 0.0 |
| scurve | black_on_wood | no | 1 | DONE → DONE | 0.57 → 1.51 | 2.0 → 2.8 | 10 → 17 | 0.05 → 0.12 | 0.0 → 0.0 | 0.0 → 0.0 |
| scurve | black_on_wood | yes | 0 | DONE → DONE | 0.81 → 1.44 | 3.2 → 2.8 | 11 → 17 | 0.27 → 0.05 | 0.0 → 0.0 | 3.2 → 0.0 |
| scurve | black_on_wood | yes | 1 | DONE → DONE | 0.76 → 1.51 | 2.7 → 2.8 | 10 → 17 | 0.05 → 0.12 | 0.0 → 0.0 | 3.7 → 0.0 |
| sharp90 | white_on_dark | no | 0 | DONE → DONE | 0.96 → 0.82 | 5.3 → 5.6 | 47 → 45 | 0.00 → 0.04 | 14.2 → 10.1 | 0.4 → 0.2 |
| sharp90 | white_on_dark | no | 1 | DONE → DONE | 1.21 → 1.16 | 5.3 → 5.6 | 47 → 45 | 0.00 → 0.09 | 25.5 → 18.4 | 0.5 → 0.4 |
| sharp90 | white_on_dark | yes | 0 | LEFT@1.14m → DONE | 1.92 → 1.01 | 12.0 → 5.6 | 47 → 45 | 0.09 → 0.12 | 21.4 → 10.6 | 19.3 → 1.4 |
| sharp90 | white_on_dark | yes | 1 | LEFT@1.14m → DONE | 2.88 → 1.30 | 12.0 → 5.6 | 47 → 45 | 0.17 → 0.33 | 33.4 → 18.7 | 24.6 → 2.4 |
| sharp90 | black_on_wood | no | 0 | DONE → DONE | 0.96 → 0.82 | 5.3 → 5.6 | 47 → 45 | 0.00 → 0.04 | 14.2 → 9.7 | 0.4 → 0.1 |
| sharp90 | black_on_wood | no | 1 | DONE → DONE | 1.21 → 1.17 | 5.3 → 5.6 | 47 → 45 | 0.00 → 0.09 | 25.5 → 18.2 | 0.4 → 0.3 |
| sharp90 | black_on_wood | yes | 0 | LEFT@1.14m → DONE | 1.91 → 0.92 | 12.0 → 5.6 | 47 → 45 | 0.09 → 0.12 | 21.2 → 10.2 | 19.4 → 0.6 |
| sharp90 | black_on_wood | yes | 1 | LEFT@1.14m → DONE | 2.89 → 1.21 | 12.0 → 5.6 | 47 → 45 | 0.17 → 0.25 | 33.6 → 18.9 | 24.3 → 0.7 |
| oval | white_on_dark | no | 0 | DONE → DONE | 0.30 → 1.75 | 1.5 → 3.8 | 7 → 12 | 0.02 → 0.03 | 0.0 → 0.0 | 0.1 → 0.0 |
| oval | white_on_dark | no | 1 | DONE → DONE | 0.27 → 2.09 | 1.4 → 3.8 | 7 → 12 | 0.04 → 0.08 | 0.0 → 0.0 | 0.1 → 0.0 |
| oval | white_on_dark | yes | 0 | LEFT@1.52m → DONE | 1.37 → 1.75 | 12.0 → 3.8 | 32 → 12 | 0.12 → 0.03 | 13.1 → 0.0 | 16.2 → 0.0 |
| oval | white_on_dark | yes | 1 | LEFT@1.51m → DONE | 2.20 → 2.09 | 12.0 → 3.8 | 32 → 12 | 0.27 → 0.08 | 22.4 → 0.0 | 23.7 → 0.0 |
| oval | black_on_wood | no | 0 | DONE → DONE | 0.30 → 1.75 | 1.5 → 3.8 | 7 → 12 | 0.02 → 0.03 | 0.0 → 0.0 | 0.1 → 0.0 |
| oval | black_on_wood | no | 1 | DONE → DONE | 0.27 → 2.09 | 1.4 → 3.8 | 7 → 12 | 0.04 → 0.08 | 0.0 → 0.0 | 0.0 → 0.0 |
| oval | black_on_wood | yes | 0 | LEFT@1.52m → DONE | 1.37 → 1.75 | 12.0 → 3.8 | 32 → 12 | 0.12 → 0.03 | 13.1 → 0.0 | 16.3 → 0.0 |
| oval | black_on_wood | yes | 1 | LEFT@1.51m → DONE | 2.20 → 2.09 | 12.0 → 3.8 | 32 → 12 | 0.27 → 0.08 | 22.5 → 0.0 | 23.7 → 0.0 |

#### Heading-error recovery (straight track, robot turned after calibration; realistic timing; both polarities)

| start yaw | c | x | max lateral err cm orig → new | settle time s (|lat| < 1 cm and heading < 3°) orig → new | sign changes /s |
|---|---|---|---|---|---|
| 15° | 0.30 | 0 | 2.75 → 2.35 | 14.07 → 11.69 | 0.03 → 0.03 |
| 15° | 0.30 | 1 | 2.69 → 2.23 | 5.03 → 4.12 | 0.19 → 0.10 |
| 15° | 0.50 | 0 | 1.76 → 1.44 | 8.89 → 6.56 | 0.03 → 0.03 |
| 15° | 0.50 | 1 | 1.65 → 1.27 | 3.20 → 1.96 | 0.10 → 0.10 |
| 15° | 1.00 | 0 | 0.84 → 0.44 | 1.58 → 1.08 | 0.00 → 0.00 |
| 15° | 1.00 | 1 | 0.70 → 0.46 | 0.75 → 0.87 | 0.10 → 0.11 |
| 15° | 2.00 | 0 | 0.24 → 0.22 | 0.78 → 0.77 | 0.00 → 0.00 |
| 15° | 2.00 | 1 | 0.34 → 0.46 | 1.62 → 8.53 | 0.70 → 1.92 |
| -15° | 0.30 | 0 | 2.75 → 2.36 | 14.07 → 11.68 | 0.03 → 0.03 |
| -15° | 0.30 | 1 | 2.70 → 2.24 | 5.06 → 4.15 | 0.19 → 0.10 |
| -15° | 0.50 | 0 | 1.77 → 1.44 | 8.92 → 6.63 | 0.03 → 0.03 |
| -15° | 0.50 | 1 | 1.66 → 1.29 | 3.23 → 2.03 | 0.10 → 0.11 |
| -15° | 1.00 | 0 | 0.84 → 0.44 | 1.60 → 1.08 | 0.00 → 0.00 |
| -15° | 1.00 | 1 | 0.70 → 0.47 | 0.78 → 0.65 | 0.11 → 0.00 |
| -15° | 2.00 | 0 | 0.24 → 0.22 | 0.78 → 0.77 | 0.00 → 0.00 |
| -15° | 2.00 | 1 | 0.29 → 0.44 | 1.62 → 8.80 | 0.65 → 1.94 |
| 30° | 0.30 | 0 | 5.21 → 5.06 | 20.73 → 15.93 | 0.06 → 0.03 |
| 30° | 0.30 | 1 | 5.18 → 5.02 | 8.99 → 7.02 | 0.16 → 0.08 |
| 30° | 0.50 | 0 | 3.16 → 2.89 | 11.68 → 10.49 | 0.03 → 0.03 |
| 30° | 0.50 | 1 | 3.08 → 2.80 | 4.85 → 4.23 | 0.09 → 0.09 |
| 30° | 1.00 | 0 | 1.14 → 0.98 | 3.71 → 1.60 | 0.00 → 0.00 |
| 30° | 1.00 | 1 | 1.09 → 0.94 | 2.25 → 1.91 | 0.10 → 0.10 |
| 30° | 2.00 | 0 | 0.14 → 0.07 | 1.02 → 0.90 | 0.00 → 0.03 |
| 30° | 2.00 | 1 | 0.39 → 0.54 | 1.87 → 9.39 | 0.64 → 1.88 |


**Reading the closed-loop tables.**
* **Every** failure of the NEW library in `ideal` and `real` (14 each) is c = 0.3: with the ±50
  output, c = 0.3 gives at most 15 motor steering, physically too little to turn on the 35 cm oval or
  a 90° corner — the ORIGINAL fails these too. With c = 0.5 (the lecturer's value) NEW completes
  40/40 in `real`, ORIGINAL 32/40.
* All extra failures of the ORIGINAL are runs with distractors (gentle curve at c = 1, oval and sharp
  corner at c = 0.5, sharp corner at c = 2 in `ideal`), with 6–30 % wrong-lock frames — the
  distractor problem the lecturer reported.
* Without distractors the ORIGINAL is a little more accurate on tight curves (oval, c = 0.5: mean
  lateral error 0.28 → 1.73 cm, max 1.4 → 3.8 cm) — the cost of the heading term described in §4.
* `stress` (3× faster, 200 ms lag): 68 → 72 of 80. Both weave with c = 1, x = 1; NEW's remaining
  failures are c = 1/x = 1 and the sharp corner at c = 0.5/x = 1.
* `tilt35` (camera really tilted 35°, Config says 45°): 34 → 36 of 40. ORIGINAL loses 6 runs to
  distractors; NEW loses the 4 sharp-corner runs at c = 1.0 (with and without distractors, 0 % wrong
  lock): the wrong geometry exaggerates the corner's heading and the robot turns toward the second
  leg ~10 cm early, cutting the corner by just over the 12 cm limit. With the correct tilt, or with
  c = 0.5, these runs complete → **set `cameraTiltDeg` to the real mount angle** (§9).

### 6.3 Heading-angle test (Task 2)
Straight line, robot rotated about its axle so that the line crosses the strip's centre row
exactly at the calibrated column (top block) or 60 px to its right (bottom block). Average of 30 frames.


| floor | line crosses strip at | robot heading vs line | steering orig | steering new | new: offset part | new: heading part | new: measured heading ° |
|---|---|---|---:|---:|---:|---:|---:|
| black_on_wood | calibrated column | -30° | 0 | -17 | -0.1 | -16.9 | -29.9 |
| black_on_wood | calibrated column | -20° | 0 | -11 | -0.1 | -10.9 | -19.9 |
| black_on_wood | calibrated column | -15° | 0 | -8 | -0.1 | -7.9 | -14.9 |
| black_on_wood | calibrated column | -10° | 0 | -5 | -0.1 | -4.9 | -9.9 |
| black_on_wood | calibrated column | -5° | 0 | -3 | 0.0 | -3.0 | -4.9 |
| black_on_wood | calibrated column | 0° | 0 | 0 | 0.0 | 0.0 | 0.1 |
| black_on_wood | calibrated column | 5° | 0 | 3 | 0.0 | 3.0 | 5.1 |
| black_on_wood | calibrated column | 10° | 0 | 5 | 0.0 | 5.0 | 10.1 |
| black_on_wood | calibrated column | 15° | 0 | 8 | 0.0 | 8.0 | 15.1 |
| black_on_wood | calibrated column | 20° | 0 | 11 | 0.0 | 11.0 | 20.1 |
| black_on_wood | calibrated column | 30° | 0 | 17 | 0.0 | 17.0 | 30.1 |
| black_on_wood | +60 px | -30° | 6 | -11 | 6.1 | -17.1 | -29.9 |
| black_on_wood | +60 px | -20° | 6 | -5 | 6.1 | -11.1 | -19.9 |
| black_on_wood | +60 px | -15° | 6 | -2 | 6.1 | -8.1 | -14.9 |
| black_on_wood | +60 px | -10° | 6 | 1 | 6.1 | -5.1 | -9.9 |
| black_on_wood | +60 px | -5° | 6 | 4 | 6.1 | -2.1 | -4.9 |
| black_on_wood | +60 px | 0° | 6 | 6 | 6.3 | -0.3 | 0.1 |
| black_on_wood | +60 px | 5° | 6 | 9 | 6.3 | 2.7 | 5.1 |
| black_on_wood | +60 px | 10° | 6 | 12 | 6.3 | 5.7 | 10.1 |
| black_on_wood | +60 px | 15° | 6 | 14 | 6.3 | 7.7 | 15.1 |
| black_on_wood | +60 px | 20° | 6 | 17 | 6.3 | 10.7 | 20.1 |
| black_on_wood | +60 px | 30° | 6 | 23 | 6.3 | 16.7 | 30.1 |
| white_on_dark | calibrated column | -30° | 0 | -17 | -0.1 | -16.9 | -30.0 |
| white_on_dark | calibrated column | -20° | 0 | -11 | -0.1 | -10.9 | -20.0 |
| white_on_dark | calibrated column | -15° | 0 | -8 | -0.1 | -7.9 | -15.0 |
| white_on_dark | calibrated column | -10° | 0 | -5 | -0.1 | -4.9 | -10.0 |
| white_on_dark | calibrated column | -5° | 0 | -3 | -0.1 | -2.9 | -5.0 |
| white_on_dark | calibrated column | 0° | 0 | 0 | -0.1 | 0.1 | 0.0 |
| white_on_dark | calibrated column | 5° | 0 | 2 | -0.1 | 2.1 | 5.0 |
| white_on_dark | calibrated column | 10° | 0 | 5 | -0.1 | 5.1 | 10.0 |
| white_on_dark | calibrated column | 15° | 0 | 8 | -0.1 | 8.1 | 15.0 |
| white_on_dark | calibrated column | 20° | 0 | 11 | -0.1 | 11.1 | 20.0 |
| white_on_dark | calibrated column | 30° | 0 | 17 | -0.1 | 17.1 | 30.0 |
| white_on_dark | +60 px | -30° | 6 | -11 | 6.1 | -17.1 | -30.0 |
| white_on_dark | +60 px | -20° | 6 | -5 | 6.1 | -11.1 | -20.0 |
| white_on_dark | +60 px | -15° | 6 | -2 | 6.1 | -8.1 | -15.0 |
| white_on_dark | +60 px | -10° | 6 | 1 | 6.1 | -5.1 | -10.0 |
| white_on_dark | +60 px | -5° | 6 | 4 | 6.1 | -2.1 | -5.0 |
| white_on_dark | +60 px | 0° | 6 | 6 | 6.1 | -0.1 | 0.0 |
| white_on_dark | +60 px | 5° | 6 | 9 | 6.1 | 2.9 | 5.0 |
| white_on_dark | +60 px | 10° | 6 | 11 | 6.1 | 4.9 | 10.0 |
| white_on_dark | +60 px | 15° | 6 | 14 | 6.1 | 7.9 | 15.1 |
| white_on_dark | +60 px | 20° | 6 | 17 | 6.1 | 10.9 | 20.0 |
| white_on_dark | +60 px | 30° | 6 | 23 | 6.1 | 16.9 | 30.0 |


The ORIGINAL gives the same value for every angle (0, or 6 when crossing at +60 px). The NEW
heading estimate matches the true angle within 0.2° and the steering grows with the angle.

### 6.4 Distractor frame tests (Task 1)
Robot static on a straight line (1 cm off-centre), calibrated on the clean floor. Two library
instances see identical frames, one without and one with the distractor; "error" = difference of
their outputs. Moving cases slide the object from 12 cm to 1 cm from the tape edge in 60 frames,
hold 60, and slide back.


| floor | scenario | frames | max steering error orig → new | mean steering error | max tracking error | wrong-lock frames | line-lost frames | max centroid jump px |
|---|---|---:|---|---|---|---|---|---|
| black_on_wood | blob_beside_HROI | 90 | 34 → 0 | 27.3 → 0.0 | 67 → 0 | 75 → 0 | 0 → 0 | 320 → 0 |
| black_on_wood | blob_beside_VROI | 90 | 0 → 0 | 0.0 → 0.0 | 0 → 0 | 0 → 0 | 0 → 0 | 0 → 0 |
| black_on_wood | chair_leg | 90 | 0 → 0 | 0.0 → 0.0 | 0 → 0 | 0 → 0 | 0 → 0 | 0 → 0 |
| black_on_wood | edge_region | 90 | 34 → 0 | 27.3 → 0.0 | 66 → 0 | 75 → 0 | 0 → 0 | 320 → 0 |
| black_on_wood | blob_touching_line | 90 | 22 → 0 | 21.9 → 0.0 | 60 → 0 | 90 → 0 | 0 → 0 | 57 → 0 |
| black_on_wood | blob_touching_VROI | 90 | 0 → 0 | 0.0 → 0.0 | 3 → 0 | 0 → 0 | 0 → 0 | 0 → 0 |
| black_on_wood | blob_moving_in | 180 | 36 → 0 | 11.0 → 0.0 | 73 → 0 | 65 → 0 | 0 → 0 | 340 → 0 |
| black_on_wood | leg_moving_in | 180 | 0 → 0 | 0.0 → 0.0 | 0 → 0 | 0 → 0 | 0 → 0 | 0 → 0 |
| black_on_wood | line_hidden_4f | 90 | 45 → 0 | 4.5 → 0.0 | 88 → 0 | 0 → 0 | 9 → 4 | 0 → 0 |
| black_on_wood | yawed_with_blob | 90 | 0 → 0 | 0.0 → 0.0 | 0 → 0 | 0 → 0 | 0 → 0 | 0 → 0 |
| black_on_wood | seat_over_line_ahead | 90 | 0 → 0 | 0.0 → 0.0 | 2 → 0 | 0 → 0 | 0 → 0 | 0 → 0 |
| black_on_wood | shadow_beside_line | 90 | 23 → 0 | 22.9 → 0.0 | 45 → 0 | 90 → 0 | 0 → 0 | 58 → 0 |
| white_on_dark | blob_beside_HROI | 90 | 34 → 0 | 27.3 → 0.0 | 67 → 0 | 75 → 0 | 0 → 0 | 320 → 0 |
| white_on_dark | blob_beside_VROI | 90 | 0 → 0 | 0.0 → 0.0 | 0 → 0 | 0 → 0 | 0 → 0 | 0 → 0 |
| white_on_dark | chair_leg | 90 | 0 → 0 | 0.0 → 0.0 | 0 → 0 | 0 → 0 | 0 → 0 | 0 → 0 |
| white_on_dark | edge_region | 90 | 34 → 0 | 27.3 → 0.0 | 66 → 0 | 75 → 0 | 0 → 0 | 320 → 0 |
| white_on_dark | blob_touching_line | 90 | 22 → 0 | 21.9 → 0.0 | 60 → 0 | 90 → 0 | 0 → 0 | 58 → 0 |
| white_on_dark | blob_touching_VROI | 90 | 0 → 0 | 0.0 → 0.0 | 2 → 0 | 0 → 0 | 0 → 0 | 0 → 0 |
| white_on_dark | blob_moving_in | 180 | 36 → 0 | 11.0 → 0.0 | 73 → 0 | 65 → 0 | 0 → 0 | 340 → 0 |
| white_on_dark | leg_moving_in | 180 | 0 → 0 | 0.0 → 0.0 | 0 → 0 | 0 → 0 | 0 → 0 | 0 → 0 |
| white_on_dark | line_hidden_4f | 90 | 45 → 0 | 4.5 → 0.0 | 88 → 0 | 0 → 0 | 9 → 4 | 0 → 0 |
| white_on_dark | yawed_with_blob | 90 | 0 → 0 | 0.0 → 0.0 | 0 → 0 | 0 → 0 | 0 → 0 | 0 → 0 |
| white_on_dark | seat_over_line_ahead | 90 | 0 → 0 | 0.0 → 0.0 | 1 → 0 | 0 → 0 | 0 → 0 | 0 → 0 |
| white_on_dark | shadow_beside_line | 90 | 0 → 0 | 0.0 → 0.0 | 0 → 0 | 0 → 0 | 0 → 0 | 0 → 0 |


In `line_hidden_4f` the line is really invisible for 4 frames: NEW reports those 4 frames as not
found (`lineHeld = true`) but holds steering and tracking, so the outputs do not change; ORIGINAL
kicks steering to +50 and tracking to the minimum and needs 9 frames to recover.

### 6.5 Regression tests (existing behaviour)
Robot on a straight line at 960 × 1280; centred, then 2 cm right and 2 cm left of the line.


| test | orig | new | values orig | values new |
|---|---|---|---|---|
| autoColor white_on_dark (YUV) | PASS | PASS | calFrames=15 colorModel=true S(centre)=0.0 T=100 S(robot 2cm right)=-10.0 S(2cm left)=10.0 | calFrames=15 colorModel=true S(centre)=0.0 T=100 S(robot 2cm right)=-10.0 S(2cm left)=10.0 |
| autoColor white_on_brown (YUV) | PASS | PASS | calFrames=15 colorModel=true S(centre)=0.0 T=100 S(robot 2cm right)=-10.0 S(2cm left)=10.0 | calFrames=15 colorModel=true S(centre)=0.0 T=100 S(robot 2cm right)=-10.0 S(2cm left)=10.0 |
| autoColor black_on_wood (YUV) | PASS | PASS | calFrames=15 colorModel=true S(centre)=0.0 T=100 S(robot 2cm right)=-10.0 S(2cm left)=10.0 | calFrames=15 colorModel=true S(centre)=0.0 T=100 S(robot 2cm right)=-10.0 S(2cm left)=10.0 |
| autoColor black_on_white (YUV) | PASS | PASS | calFrames=15 colorModel=true S(centre)=0.0 T=100 S(robot 2cm right)=-10.0 S(2cm left)=10.0 | calFrames=15 colorModel=true S(centre)=0.0 T=100 S(robot 2cm right)=-10.0 S(2cm left)=10.0 |
| autoColor blue_on_yellow (YUV) | PASS | PASS | calFrames=15 colorModel=true S(centre)=0.0 T=100 S(robot 2cm right)=-10.0 S(2cm left)=10.0 | calFrames=15 colorModel=true S(centre)=0.0 T=100 S(robot 2cm right)=-10.0 S(2cm left)=10.0 |
| autoColor red_on_green (YUV) | PASS | PASS | calFrames=15 colorModel=true S(centre)=0.0 T=100 S(robot 2cm right)=-10.0 S(2cm left)=10.0 | calFrames=15 colorModel=true S(centre)=0.0 T=100 S(robot 2cm right)=-10.0 S(2cm left)=10.0 |
| autoColor white_on_dark (GRAY) | PASS | PASS | calFrames=15 colorModel=true S(centre)=0.0 T=100 S(robot 2cm right)=-10.0 S(2cm left)=10.0 | calFrames=15 colorModel=true S(centre)=0.0 T=100 S(robot 2cm right)=-10.0 S(2cm left)=10.0 |
| autoColor white_on_dark (ARGB) | PASS | PASS | calFrames=15 colorModel=true S(centre)=0.0 T=100 S(robot 2cm right)=-10.0 S(2cm left)=10.0 | calFrames=15 colorModel=true S(centre)=0.0 T=100 S(robot 2cm right)=-10.0 S(2cm left)=10.0 |
| autoColor black_on_wood (GRAY) | PASS | PASS | calFrames=15 colorModel=true S(centre)=0.0 T=100 S(robot 2cm right)=-10.0 S(2cm left)=10.0 | calFrames=15 colorModel=true S(centre)=0.0 T=100 S(robot 2cm right)=-10.0 S(2cm left)=10.0 |
| autoColor black_on_wood (ARGB) | PASS | PASS | calFrames=15 colorModel=true S(centre)=0.0 T=100 S(robot 2cm right)=-10.0 S(2cm left)=10.0 | calFrames=15 colorModel=true S(centre)=0.0 T=100 S(robot 2cm right)=-10.0 S(2cm left)=10.0 |
| autoColor red_on_green (GRAY |  equal brightness) must NOT calibrate |  equal brightness) must NOT calibrate | PASS | PASS |
| manual trackIsBright=true white_on_dark | PASS | PASS | calFrames=10 colorModel=false S(centre)=0.0 T=100 S(robot 2cm right)=-10.0 S(2cm left)=10.0 | calFrames=10 colorModel=false S(centre)=0.0 T=100 S(robot 2cm right)=-10.0 S(2cm left)=10.0 |
| manual trackIsBright=false black_on_white | PASS | PASS | calFrames=10 colorModel=false S(centre)=0.0 T=100 S(robot 2cm right)=-10.0 S(2cm left)=10.0 | calFrames=10 colorModel=false S(centre)=0.0 T=100 S(robot 2cm right)=-10.0 S(2cm left)=10.0 |
| manual trackIsBright=false black_on_wood | PASS | PASS | calFrames=10 colorModel=false S(centre)=0.0 T=100 S(robot 2cm right)=-10.0 S(2cm left)=10.0 | calFrames=10 colorModel=false S(centre)=0.0 T=100 S(robot 2cm right)=-10.0 S(2cm left)=10.0 |
| recal step 1 white_on_dark | PASS | PASS | calFrames=15 colorModel=true S(centre)=0.0 T=100 S(robot 2cm right)=-10.0 S(2cm left)=10.0 | calFrames=15 colorModel=true S(centre)=0.0 T=100 S(robot 2cm right)=-10.0 S(2cm left)=10.0 |
| recal step 2 black_on_wood | PASS | PASS | calFrames=15 colorModel=true S(centre)=0.0 T=100 S(robot 2cm right)=-10.0 S(2cm left)=10.0 | calFrames=15 colorModel=true S(centre)=0.0 T=100 S(robot 2cm right)=-10.0 S(2cm left)=10.0 |
| recal step 3 white_on_dark again | PASS | PASS | calFrames=15 colorModel=true S(centre)=0.0 T=100 S(robot 2cm right)=-10.0 S(2cm left)=10.0 | calFrames=15 colorModel=true S(centre)=0.0 T=100 S(robot 2cm right)=-10.0 S(2cm left)=10.0 |
| empty floor (black_on_wood floor |  no tape) must NOT calibrate |  no tape) must NOT calibrate | PASS | PASS |
| empty floor (white_on_dark floor |  no tape) must NOT calibrate |  no tape) must NOT calibrate | PASS | PASS |
| empty floor (black_on_white floor |  no tape) must NOT calibrate |  no tape) must NOT calibrate | PASS | PASS |
| straight scripted poses (steering) | PASS | PASS | lat+0cm/yaw+0:0.0 lat+0cm/yaw+5:9.0 lat+0cm/yaw-5:-9.0 lat-1cm/yaw+0:-5.0 lat-1cm/yaw+5:4.0 lat-1cm/yaw-5:-14.0 lat-3cm/yaw+0:-16.0 lat-3cm/yaw+5:-7.0 lat-3cm/yaw-5:-24.0 lat+1cm/yaw+0:5.0 lat+1cm/yaw+5:14.0 lat+1cm/yaw-5:-4.0 lat+3cm/yaw+0:16.0 lat+3cm/yaw+5:25.0 lat+3cm/yaw-5:7.0 lat+0cm/yaw+0:0.0 lat+0cm/yaw+5:9.0 lat+0cm/yaw-5:-9.0 | lat+0cm/yaw+0:0.0 lat+0cm/yaw+5:11.0 lat+0cm/yaw-5:-11.0 lat-1cm/yaw+0:-5.0 lat-1cm/yaw+5:6.0 lat-1cm/yaw-5:-17.0 lat-3cm/yaw+0:-16.0 lat-3cm/yaw+5:-4.0 lat-3cm/yaw-5:-27.0 lat+1cm/yaw+0:5.0 lat+1cm/yaw+5:17.0 lat+1cm/yaw-5:-6.0 lat+3cm/yaw+0:16.0 lat+3cm/yaw+5:27.0 lat+3cm/yaw-5:4.0 lat+0cm/yaw+0:0.0 lat+0cm/yaw+5:11.0 lat+0cm/yaw-5:-11.0 |
| steering within [-50 | 50] and tracking within [5 | 50] and tracking within [5 | 100] (all frames above) | 100] (all frames above) |


* Automatic colour learning works for bright-on-dark and dark-on-bright pairs and for colour pairs
  of equal brightness (YUV); equal brightness in grayscale still (correctly) refuses to calibrate.
* Manual mode (`autoColor = false`, `trackIsBright = true / false`) unchanged.
* Recalibration bright → dark → bright works.
* An empty floor (wood, carpet, white) never calibrates (`ISSUE_LOW_CONTRAST`), as before.
* On a straight line with no heading error the NEW steering equals the ORIGINAL (±1 from rounding);
  with ±5° yaw it differs by 2–3 — the heading term, by design.

### 6.6 Existing project unit tests (`LineFollowerLibrary/src/test`, 24 tests, JUnit 4)
| Library | Result |
|---|---|
| ORIGINAL | 24 / 24 pass |
| NEW, default Config | 16 / 24 — 7 tests expect 19 for a line 60 px right and get 22; `lostLineReportsTrackLost` expects tracking 5 on the first blank frame and gets the held 98 |
| NEW with only `steerHeadingWeight = 0` and `trackHoldFrames = 0` (compiled copy in `_investigation/build/variant_src`) | **24 / 24 pass** |

The 7 tests draw a perfectly *vertical* line shifted sideways on a flat 320 × 240 image. Through a
45°-tilted camera that is a line on the floor heading outward, so the heading term rightly adds 3.
The 8th is the new hold behaviour. Since these tests encode v1 steering exactly, they need updated
expected values (or those two settings). `gradlew jar` / `assembleDebug` do not run them; `gradlew test` / `build` would.

**Update — tests changed for v2 (`gradlew test`: 26 / 26 pass with the default Config).**
* The 7 steering tests no longer expect a fixed 19. They check that the offset part
  (`steerOffsetTerm`) is still 19 ± 2, that the heading part (`steerHeadingTerm`) steers the same
  way, and that the output equals offset + heading. A centred line must still give 0.
* `lostLineReportsTrackLost` now checks that the last values are held (`lineHeld`) for
  `trackHoldFrames` frames and that tracking drops to 5 on the next frame.
* Two new tests keep the v1 behaviour covered: `v1SteeringIsOffsetOnly` (±19 with
  `steerHeadingWeight = 0`) and `v1LostLineDropsTrackingAtOnce` (tracking 5 on the first blank
  frame with `trackHoldFrames = 0`).

### 6.7 Zero allocation and timing (`Perf`, 960 × 1280, this PC, JDK 21)

| | ORIGINAL | NEW |
|---|---|---|
| Heap bytes allocated by 1000 `processFrameYuv420` / `processFrame` (gray) / `processFrameArgb` (callback registered) | 0 / 0 / 0 | 0 / 0 / 0 |
| ms per frame, YUV, mean (six runs on three occasions) | 0.89, 0.98, 0.90, 0.87, 0.92, 0.92 | 0.90, 0.91, 0.89, 0.92, 0.94, 0.94 |
| median / p95 / p99 (last run) | 0.92 / 1.01 / 1.17 | 0.94 / 1.03 / 1.20 |

The difference is inside run-to-run noise. On a phone expect several times these numbers, still
far below the 16.7 ms budget of 60 Hz; the added work is O(rows) in the look-ahead plus one extra
add per strip pixel.

### 6.8 Real-image test (lecturer's screenshots)
`test_images/` holds 2 Blockly screenshots and 3 app screenshots on the wood table with the black
chair. Each app screenshot was converted to a 960 × 1280 frame: the camera area was mapped back
through the overlay's FILL_CENTER geometry (scale 0.833, offset 60.8 / 116.8 screen px, derived from
the drawn strip and V-ROI), the translucent cyan (strip) and magenta (V-ROI) fills were inverted,
strokes / the green centroid line / labels were in-painted from neighbours, and
Y = 0.299 R + 0.587 G + 0.114 B (U, V BT.601). The tape span in the strip was annotated by hand.
Converted frames: `snaps/real_*.png`. 60 noisy frames per test, first 10 ignored.


| calibrated on | test image | calibrated | tape centre (annotated) | mean centroid orig → new | px outside tape | wrong-lock frames (of 50) | mean steering | mean tracking |
|---|---|---|---|---|---|---|---|---|
| 18.40.04.jpeg | 18.40.04.jpeg | true | 416 | 443 → 443 | 0 → 0 | 0 → 0 | 0 → 6 | 8 → 8 |
| itself | 18.40.04.jpeg | true | 416 | 443 → 443 | 0 → 0 | 0 → 0 | 0 → 6 | 8 → 8 |
| 18.40.04.jpeg | 18.40.04 (1).jpeg | true | 352 | 325 → 325 | 0 → 0 | 0 → 0 | -12 → -11 | 5 → 6 |
| itself | 18.40.04 (1).jpeg | false | 352 | 317 → 317 | 0 → 0 | 0 → 0 | -17 → -35 | 14 → 19 |
| 18.40.04.jpeg | 18.40.04 (2).jpeg | true | 320 | 243 → 243 | 18 → 18 | 0 → 0 | -21 → -35 | 5 → 5 |
| itself | 18.40.04 (2).jpeg | false | 320 | 208 → 208 | 52 → 52 | 50 → 50 | -28 → -44 | 23 → 18 |


Both versions lock inside (or within 18 px of) the tape on all three frames and never on the
chair; calibrating directly on (1)/(2) fails for both because the tape is not near the centre there
(calibration assumes the robot is placed on the line). The screenshot failures (lock on a wide wood
patch, gate as wide as half the frame) are the result of a *history* of frames and cannot be
reproduced from one static screenshot; their mechanism is reproduced by `shadow_beside_line` /
`edge_region` (§6.4) and the closed-loop distractor runs. The preview screenshot is not the analysis
frame (different processing, overlay over the strip), so this test is indicative only.

---

## 7. Recommended starting values for the lecturer's program

Steering block: `(a − b) × c` with a = 0, b = Steering → **c = 1.0**.
* Steering was rescaled from ±100 to ±50 in commit 584d353, after the program was written: with
  c = 0.5 the robot now gets at most ±25 motor steering (inner wheel at half speed, ≈ 20 cm turning
  radius). In the simulation c = 0.3 fails every tight turn and c = 0.5 is marginal on a 90° corner;
  **c = 1.0 completed all 20 runs per power mode in both `ideal` and `real`** (worst lateral error
  9 cm) and restores the authority c = 0.5 had with ±100. c = 0.5 also completed every `real` run
  with the new library (40/40), so it is an acceptable, gentler alternative on tracks without sharp
  corners; prefer c = 1.0 when there are 90° turns, after setting `cameraTiltDeg` correctly.
* c = 2.0 also completes everything but with x = 1 it weaves slightly (1.9 sign changes/s after a
  disturbance); keep c ≤ 1.0 when power follows tracking.

Power block: `round(x × (a / b) + c)` with a = Power:
* Start with **x = 0, c = 5** (constant power 5). If the robot does not move at power 5 (motor dead
  zone), raise c to 6–8 rather than the steering gain.
* Then **x = 1, b = 8**: power 5 … 17.5, slower before turns (tracking drops). With c = 1.0 this
  completed every track in `real`; in `stress` (much faster robot, 200 ms lag) it weaved — if the
  real robot weaves with x = 1, increase b (e.g. b = 12 → power 5 … 13) before lowering the
  steering gain.
* Measured loop rate: the simulation assumed 20 Hz Blockly loop + 150 ms total latency. If the
  lecturer's rate test shows much less than ~15 Hz, keep x = 0.

Library: keep the defaults (`steerHeadingWeight = 1`, `steerLookAhead = 0.5`). If the robot follows
curves worse than before but straightens out better, lower `steerLookAhead` (0.25); if it still
hesitates when crossing the line at an angle, raise it (0.75–1.0).

---

## 8. Build

* Library JAR: `gradlew jar` in `LineFollowerLibrary/` (build dir already outside OneDrive in
  `build.gradle`; no OneDrive lock problem occurred, so no init script was needed) →
  `C:\RoboPhoneBuild\library\libs\LineFollowerPipeline-1.0.0.jar` (31 319 bytes), copy in
  `_investigation/jar/`. `javap` confirms the new FrameData fields.
* **Not copied to `RoboPhoneApp/app/libs/`**: that folder is empty and unused — since commit 08b19a1
  the app compiles the library **from source** via `includeBuild("../LineFollowerLibrary")`
  (`settings.gradle.kts`), so a JAR there would be an unused copy that can go stale. Say if you
  still want it there.
* Android app: `gradlew assembleDebug` **succeeds** (`C:\RoboPhoneBuild\app\outputs\apk\debug\app-debug.apk`,
  built with the new library). The checkout has no `local.properties`, so the SDK path was given by
  the `ANDROID_HOME` environment variable for that command only (no file created). The app's code was
  not changed; it does not show the new FrameData fields yet (optional).

---

## 9. Known limitations / to verify on the real robot

1. **Camera geometry.** The heading uses `cameraTiltDeg` / `verticalFovDeg`. The screenshots show the
   wall and floor tiles at the top of the view, i.e. the real tilt is probably well below 45°
   (≈ 25–35°). A wrong value scales the heading — with 35° real vs 45° configured the robot cut a 90°
   corner too early at c = 1.0 (§6.2). Measure it:
   place the robot on a straight line at a known angle, e.g. 20°, and check `FrameData.headingDeg`).
2. **Calibrate on a straight section**, robot straight on the line — the heading zero is learned
   there (limited to ±10°).
3. **Curves.** The heading term adds some turn-in on tight curves (robot runs 1–2 cm more on the
   inside of a R = 35 cm curve). Watch the robot on the tightest curve; lower `steerLookAhead` if it
   cuts it.
4. **Gain vs. latency.** With c ≥ 1 and x = 1 on a fast robot with large latency, both versions weave
   (`stress`). The library cannot remove control-loop lag; reduce b (more power reduction) or c.
5. **Distractor rules rely on calibration.** A dark object of exactly the tape's width, touching
   nothing and lying where the line is predicted (e.g. a black cable crossing right in front of the
   robot at a junction-like angle) can still be taken for the line. Recalibrate when the tape width
   changes (different tape).
6. **Hold.** A line that disappears mid-image is reported lost only after 6 frames (0.1 s); during the
   hold `trackFound` is false and `lineHeld` true.
7. **Simulator assumptions** (not measured on the robot): speed 5 cm/s at power 5, wheelbase 13 cm,
   LEGO-like steering block, 150 ms latency, dead zone 2. The real preview screenshots are not the
   analysis frames. Verify on the robot: distractor runs with the black chair beside the track (watch
   `FrameData.rejectedBands` / the green centroid line), the heading test above, and the loop rate.
8. **Existing unit tests** need new expected values for the heading term / hold (§6.6).

## 10. What in `LineFollowerPipeline_Integration_Guide.pdf` is now outdated (PDF not edited)

| Page | Outdated content |
|---|---|
| 1 | "What's new in this version" — misses distractor rejection, offset + heading steering, hold, new FrameData fields. |
| 2 | Output format: steering is no longer "only the line's position in the strip" — it also includes the heading part. |
| 3 | Camera Mount table: `cameraTiltDeg` / `verticalFovDeg` now also drive the steering heading and the look-ahead width test. |
| 4 | Track Memory table: gate around the *predicted* position, capped width; `trackRelockFrames` stricter for a valid-looking lock; new options `rejectDistractors`, `bandMaxWidthMult`, `bandMinWidthMult`, `trackHoldFrames`, `steerOffsetWeight`, `steerHeadingWeight`, `steerLookAhead`, `steerHeadingMinRows`, `steerHeadingCalibMaxDeg` are missing. |
| 4 | "While the locked line is lost, steering reads exactly ±50 toward the side it left" — now only when it leaves at the frame edge; a line vanishing mid-image is held first. |
| 5 | Calibration: now also records the line's line-ness level and the heading zero (calibrate on a straight section). |
| 6–7 | FrameData table: `steerOffsetTerm`, `steerHeadingTerm`, `headingDeg`, `headingConfidence`, `lineHeld`, `rejectedBands` missing; `tiltDeg` is still image-space (the steering uses `headingDeg`). |
| 9 | Full example uses `1280, 720`; the app actually receives 960 × 1280. Behaviour reference lacks "robot at an angle", "distractor beside the line", "line hidden". |
| 10 | Notes: "A line that jumps sideways by more than the gate…" still true but the gate follows the predicted position; performance figures (0.7 ms at 720 × 1280) — v2 measured 0.9 ms at 960 × 1280 on this PC. |

---

## 11. How to rerun all tests

From `_investigation/` in Git Bash (JDK 21 from Android Studio; plain Java, no other dependencies):

```bash
./build.sh              # compiles ORIGINAL (from the backup), NEW (working copy) and the harness
./run_tests.sh          # frame tests, heading, regression, real images, perf, JUnit, report tables (~15 min)
./run_closed_loop.sh    # closed-loop campaign, 2 × 488 runs (about 1–1.5 h on 20 threads)
java -cp build/harness harness.Report results results/report_tables.md   # rebuild the tables
```

Single pieces: `harness.FrameTests distract|heading <csv>`, `harness.Regression <csv>`,
`harness.Perf both`, `harness.RealImage <csv> snaps <calib.jpeg> <test.jpeg>…`,
`harness.ClosedLoop <csv> sweep 18` with `-Dsweep.t/s/d/c/x/v=…`, `-Dsim.lat/hz/dead/tilt/yaw0=…`,
`-Dlib.set=field=value,…` (any Config field), `-Dtrace=1` (per-frame trace). Run with
`-cp "build/orig;build/harness"` or `-cp "build/new;build/harness"` and `-Dlib.tag=orig|new`.

Harness sources: `harness/src/harness/` — `Cam` (pinhole camera), `World` / `Scenes` (floor maps,
tracks, colours, distractors), `Render` (YUV renderer + ground truth), `Lib` (adapter, app Config),
`ClosedLoop` (simulator), `FrameTests`, `Regression`, `Perf`, `RealImage`, `CurveBias`, `Report`,
`Summary`, `Table`, `Snap`, `Probe` (helpers).

## 12. Files changed (`git status`)

```
$ git status --short   (HEAD still ec96af5 = origin/main, nothing committed)
 M LineFollowerLibrary/src/main/java/com/example/linefollower/LineFollowerPipeline.java
 M README.md
?? _investigation/
$ git log --oneline -1
ec96af5 Update Integration Guide PDF: steering [-50, +50], tracking minimum 5
```

* `LineFollowerLibrary/.../LineFollowerPipeline.java` and `README.md` — the intended changes.
* `_investigation/` — untracked (it is **not** in `.gitignore`; add it there or keep it out of commits). Holds this report, the harness, results, converted images and the JAR.
* Outside the repository: backup `Desktop\fixed\LineFollowerPipeline_backup_2026-10-18\`; Gradle outputs in `C:\RoboPhoneBuild\` (as configured by the project).
