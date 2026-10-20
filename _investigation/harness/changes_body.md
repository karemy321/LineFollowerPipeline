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
| Existing project JUnit tests (`src/test`) | 24/24 | 16/24 with v2 defaults — all 8 differences are the two intended new behaviours; **24/24** with `steerHeadingWeight = 0` and `trackHoldFrames = 0` (see §6.6) |
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
`RoboPhoneApp/` (builds unchanged against the new library, see §8), `line_follower_block.js`, `line_follower.xml`, the PDF, the existing unit tests in `LineFollowerLibrary/src/test/`.

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

<<T-closed>>

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

<<T-heading>>

The ORIGINAL gives the same value for every angle (0, or 6 when crossing at +60 px). The NEW
heading estimate matches the true angle within 0.2° and the steering grows with the angle.

### 6.4 Distractor frame tests (Task 1)
Robot static on a straight line (1 cm off-centre), calibrated on the clean floor. Two library
instances see identical frames, one without and one with the distractor; "error" = difference of
their outputs. Moving cases slide the object from 12 cm to 1 cm from the tape edge in 60 frames,
hold 60, and slide back.

<<T-distract>>

In `line_hidden_4f` the line is really invisible for 4 frames: NEW reports those 4 frames as not
found (`lineHeld = true`) but holds steering and tracking, so the outputs do not change; ORIGINAL
kicks steering to +50 and tracking to the minimum and needs 9 frames to recover.

### 6.5 Regression tests (existing behaviour)
Robot on a straight line at 960 × 1280; centred, then 2 cm right and 2 cm left of the line.

<<T-regression>>

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
expected values (or those two settings) — I did **not** edit the project's test files; tell me if you
want that done. `gradlew jar` / `assembleDebug` do not run them; `gradlew test` / `build` would.

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

<<T-real>>

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

<<GIT-STATUS>>
