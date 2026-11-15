# Camera Offset Removal

## Context

This is the Robophone line-following robot. The code lives in two repos:
- **Robophone_App**: the Android app (`MainActivity`, `LineFollowerHelper`).
- **Robophone/LineFollowerPipeline**: the line follower library, included in the app as the `libs/LineFollowerPipeline` submodule.

**Current state: the robot works perfectly.** The previous round of changes (pipeline v6.1.0, documented in `STEERING_IMPROVEMENTS.md` and `CHANGES.md`) fixed the steering discontinuities in turns. It was tested on the real robot with excellent results. **This state must be protected.**

## The Remaining Issue

In sharp turns the line almost leaves the camera frame, and sometimes the robot loses the path. The discontinuity itself is solved. The lecturer wants fewer line losses in turns.

## The Lecturer's Suggestion

The camera is not in the center of the smartphone, so the code applies an **offset** to steer the robot's center over the line instead of the camera. The lecturer suggests removing this offset:

- The camera will be over the line, and the robot's center will run parallel to the line, shifted by the camera's distance from the phone's center.
- Tracking will not be exactly symmetric. That's fine, and even good: the metal ball caster at the front of the robot will no longer ride over the tape, which has some height and disturbs the robot.
- The offset may also add error in some situations.

## Why This Should Help

The camera is on the right side of the phone. With the offset, the line sits off-center in the image (expected: left of center), so one side of the frame has less margin, and turns toward that side lose the line first. With offset 0, the line sits in the center, the margin is the same on both sides, and the worst-case margin improves. The track has turns in both directions, so a balanced margin is the right target.

Trade-off: the side that had extra margin (expected: right turns) loses that extra. It ends up with the same margin as the other side.

## Goal

Remove the camera offset (set it to 0) with the **smallest possible change**, and change nothing else.

---

## Ground Rules (apply to every phase)

- **Do not break what works.** The current robot-tested state is the baseline.
- **Minimal change only.** Ideally a single config value. No refactoring, renaming, cleanup or tuning changes.
- **Switchable.** It must be possible to restore the original offset by changing one value. Keep the original value as a named constant (e.g. `CAMERA_OFFSET_ORIGINAL`).
- **Report only, don't change,** anything outside the offset. If you find something else worth changing, propose it and wait.
- **Stop after each phase.** Show what you did and wait for my approval.
- **Ask before guessing** if the code is ambiguous.
- **I test on the real robot.** You can't. Verify in the simulator and tests, and don't claim it works on the robot.

---

## Phase 0: Save the Current State (no code changes)

**Tasks**
1. Make sure both repos (the app and the pipeline submodule) have clean working trees.
2. Find and record the **exact configuration that runs on the robot today**: every flag and parameter set in the app (for example `USE_TREND_HOLD`, `USE_ROW_WISE_BASE`, `HOLD_POWER_SCALE`, `steeringBaseRows`), and the pipeline version the app uses.
3. Create a tag `robot-approved-before-offset-removal` in **both** repos on the current commits, and push the tags to GitHub.
4. Create a branch `offset-removal` in each repo where changes will be needed.
5. Write the recorded configuration into this file under "Saved Robot Configuration" below, with the tag names and commit hashes, and how to return to this state.

**Done when:** Tags are pushed, the configuration is recorded, and I've approved.

---

## Phase 1: Discovery and Measurement (no code changes)

**Tasks**
1. Find where the camera offset is **defined** (file, line, name, value, units) and **every place it is used**. For each use, explain what it affects: the steering error, the search corridor, the tracker or Kalman state, the lock, edge-exit logic, calibration, the HUD, or anything else.
2. Determine **what the offset compensates for**: only the camera's sideways position, or also a camera rotation (yaw)? If it compensates rotation too, removing it could make the robot drive at a slight angle. Report this clearly.
3. Check the **image orientation**: on which side of the image center does the line appear today on a straight line? Confirm whether this matches "camera on the right side of the phone".
4. **Measure the margins** at the steering strip, in pixels and as a percentage of half the frame width, for the frame size used on the robot:
   - today: margin to the left edge and to the right edge
   - with offset 0: margin to each edge
5. **Estimate the physical shift** of the robot's center from the line in centimeters, if the code or config gives enough information (camera height, field of view, offset). If not, say so.
6. **Report only, don't change:** does the code already slow down in sharp turns (power reduced when steering is high)? Where, and with which parameters? This is for a possible later step.
7. Propose the exact minimal change for Phase 2.

**Done when:** The report is delivered and I've approved the proposed change.

---

## Phase 2: Remove the Offset

**Tasks**
1. Set the offset to 0 in the way approved in Phase 1. Prefer changing one config value in the app over changing pipeline code. Keep the original value as a named constant.
2. If the offset is used in several places, change **only** what was approved in Phase 1.
3. **Verify in the simulator against the baseline tag:**
   - With the original offset: output is byte-identical to the baseline.
   - With offset 0: straight lines stay smooth (straight-line dither within the existing limit), and turns in **both directions** are checked separately. Report line losses or edge exits for left and right turns, before and after.
   - The trend hold, glare coast and edge-exit pin still behave as before.
4. Build the app and run all tests. The results must match the baseline counts. If any test assumes the original offset, report it before changing anything.

**Done when:** The diff is shown, the comparison table (left turns / right turns / straight, before and after) is delivered, the build and tests pass, and I've approved.

---

## Phase 3: Wrap-Up

**Tasks**
1. Bump versions as the project does (pipeline tag and app `versionCode`) if a release is needed. Ask first.
2. Update `CHANGES.md`: what changed, why (the lecturer's reasoning), the new and original values, the measured margin gain, and how to revert.
3. Add a short robot test checklist:
   - straight line: smooth, robot runs parallel to the tape and the ball caster does not ride on it
   - sharp left turns: fewer line losses than before
   - sharp right turns: still OK
   - start-up: a small initial sideways correction is expected
   - how to revert to the original offset
4. Commit on the `offset-removal` branch(es) and push. Don't merge to main.

**Done when:** I've approved and everything is pushed.

---

## Saved Robot Configuration

Recorded in Phase 0 (2026-11-15).

**Repo layout (differs from the Context section above):** there is **one** repo,
https://github.com/karemy321/LineFollowerPipeline. It holds both the library
(`LineFollowerLibrary/`) and the app (`RoboPhoneApp/`). The app builds the library from
source (`includeBuild("../LineFollowerLibrary")` in `RoboPhoneApp/settings.gradle.kts`).
There is no submodule, no `LineFollowerHelper` and no `CHANGES.md`; the previous round is
documented in `STEERING_IMPROVEMENTS.md`. One tag covers both the app and the library.

**Saved state**

| Item | Value |
|---|---|
| Tag | `robot-approved-before-offset-removal` (annotated, pushed) |
| Commit | `083518d7c3aa0d5de0bacdab2ddb4e18420f1888` on branch `steering-improvements` |
| Content | pipeline v6.1.1 (`49a92ad`) + latency logging in `MainActivity` (`083518d`). This is the exact build installed on the robot. |
| Work branch | `offset-removal` (from the tag) |
| App version | `versionCode 1`, `versionName "1.0"`; library artifact `1.0.0` |
| Camera stream | ImageAnalysis 1280×720 requested, the phone delivers 720×960 portrait at ~30 fps (TARGET_FPS 60 not reached) |

**App constants (`MainActivity`)**

| Constant | Value | Original (v6.0.0) |
|---|---|---|
| `USE_RED_ZONE_EXCLUSION` | `true` | `true` |
| `USE_WIDE_STEERING_BASE` | `false` | `false` |
| `STEERING_BASE_ROWS` | `72` (unused while the wide base is off) | — |
| `USE_ROW_WISE_BASE` | **`true`** | `false` |
| `ROW_WISE_BASE_ROWS` | `48` | — |
| `USE_TREND_HOLD` | **`true`** | `false` (did not exist) |
| `HOLD_POWER_SCALE` | `1.0` | — |
| `APP_VFOV_DEG` | `60` (real VFOV measured 72.5°, not changed) | `60` |
| `MEASURE_LATENCY` | `true` (logging only) | — |
| `VERBOSE` | `false` | `false` |
| SharedPreferences | AE/AWB lock off, v3 HUD off, EV 0 (defaults; the phone may hold other saved values) | |

**Config set by the app (`MainActivity`, pipeline set-up)**

`autoColor = true`, `thresholdMargin = 0`, `hRoiCenterYFrac = 0.85`, `hRoiThickPx = 24`,
`bottomIgnoreFrac = 0.25`, `useRedZoneExclusion = true`, `wideSteeringBase = false`,
`steeringBaseRows = 72`, `rowWiseSteeringBase = true`, `rowWiseBaseRows = 48`,
`useTrendHold = true`, `holdPowerScale = 1.0`, `vRoiHalfWidthPx = 80`,
`cameraTiltDeg = 45`, `verticalFovDeg = 60`.

**Library defaults in effect (not set by the app)**

`corridorMode = true`, `glareMask = true` (`glareCoastMs = 600`), `trackMemory = true`,
`rejectDistractors = true`, `edgePairCheck = true`, `steerOffsetWeight = 1.0`,
`steerHeadingWeight = 1.0`, `steerLookAhead = 0.5`, `steeringScaleNormalize = true`,
`rowWiseGroupRows = 6`, `rowWiseMinValidFrac = 0.5`, `maxSteeringJump = 25`,
`trendWindowFrames = 4`, `maxHoldMs = 300`, `maxJumpHoldMs = 70`,
`trackLengthSlowdown = true`, `trackingMinValue = 5`, `calibFrames = 10`.

**How to return to this state**

```
git fetch --tags
git checkout robot-approved-before-offset-removal      # or: git checkout -b back robot-approved-before-offset-removal
```
Then build and install the app as usual (Gradle in `RoboPhoneApp/`). The library is built
from the same checkout, so nothing else needs to be switched.

---

## Phase 1 Report (Discovery and Measurement)

No code was changed. Line numbers refer to
`LineFollowerLibrary/src/main/java/com/example/linefollower/LineFollowerPipeline.java` at the tag.

### 1. Where the offset is defined and used

**There is no fixed offset value.** The "offset" is the calibrated straight-ahead column
`steerRefX` (pixels, image column):
- `:1237` init: `steerRefX = width * 0.5f`, the image centre, until calibrated.
- `:1422–1442` calibration: the line's centroid in the steering strip is sampled on
  `calibFrames = 10` frames, and `steerRefX` = their **median**. The robot is placed with its
  centre over a straight line (`ROBOT_TEST_CHECKLIST.md` step 6), so the line's position in the
  image at that moment becomes "straight ahead".
- `:1445` derived from it: `trackRefX = W/2 + (steerRefX − W/2) × laPerspectiveRatio`, the same
  offset projected to the look-ahead row.

The offset is `steerRefX − W/2` px, re-learned every calibration and every RECALIBRATE.

**Every use of `steerRefX` and what it affects**

| Line | Use | Affects |
|---|---|---|
| 2837, 4061 | `offsetTerm = (centroid − steerRefX) / halfW × 100` (v2 strip path and v3 corridor path) | **steering error**, the main use |
| 2586, 2619, 3124–3125 | `findBand(…, steerRefX, …)`: preferred column when searching or re-acquiring the line, and in the continuity check | **search** and **lock** (which band is taken) |
| 2610 | `scoreBand(…, steerRefX, …)` lock-correction score | **lock** (relock) |
| 2745–2747, 2784–2786, 4035 | edge pin: side = `lastCentroidX ≥ steerRefX`, gate parked at `steerRefX ± halfW` | **edge-exit** side choice and gate park position. The pinned steering is ±100 either way. |
| 2703 | glare pin: take the line back only on the pinned side, relative to `steerRefX` | **glare / edge exit** |
| 2821 | look-ahead X when there is no look-ahead = `steerRefX` | **tracking** (power) fallback |
| 2982 | `predictedSteering`: `xRef` = `steerRefX` on the floor | **pin cap** steering under glare |
| 2863, 4195 via `trackRefX` | `intersection = |lookAheadX − trackRefX| / span` | **tracking (power)**: power = 100 − intersection |
| 4496 → app `OverlayView:223, 264, 309` | `FrameData.steerRefXFrac` / `trackRefXFrac` | **HUD / overlay**: the dashed reference line and the steering marker |

**Not affected:** the corridor Kalman state (`kx`, floor coordinates around the optical axis,
`:3262`, `:3896–3902`), the edge zones (`edgeZone` from the image edges, `:2766`), the colour,
width and level calibration, the glare mask, the trend hold, and the row-wise base geometry.

### 2. What the offset compensates for

`steerRefX` takes in **everything** that puts the line off-centre at calibration:
1. the camera's sideways position (the intended part);
2. camera yaw, as forward distance to the strip × tan(yaw). The strip is ≈ 9.7 cm ahead of the
   camera (geometry below), so 1° of yaw ≈ 0.17 cm ≈ 5 px;
3. any placement error of the robot at calibration.

**Camera rotation is also compensated separately**, by `headingRefDeg` (`:1448–1452`): the
median line heading at calibration becomes heading zero, used if |h| ≤ 10°
(`steerHeadingCalibMaxDeg`). It is subtracted in the heading term (`:2843`, `:4070`).

**Conclusion:** if only `steerRefX` is moved to the centre and `headingRefDeg` is kept, the
robot **does not drive at an angle**. In steady state, steering is 0 when the offset term and the
heading term are both 0: the robot is parallel to the line (heading = calibrated zero) and the
line crosses the strip at the image centre. The robot centre then runs parallel to the tape,
shifted by camera offset + 9.7 cm × tan(yaw).

### 3. Image orientation: which side the line appears on today

**Not measurable here:** the phone is not connected, there are no robot recordings in the repo,
and the app does not log `steerRefX`. Expected from the code: back camera, portrait, forward = up,
no mirroring, and `offsetTerm > 0` = turn right. A camera to the right of the robot centre sees
the line to its **left**, so `steerRefX < 360`. This matches "line left of centre".
**To confirm on the robot:** after calibration, the overlay draws the dashed vertical reference
line at `steerRefX`. Check whether it is left of the image centre, and how far.

### 4. Margins at the steering strip (frame 720 × 960, strip rows 696–720)

Half width = 360 px. With the offset `d = 360 − steerRefX` px (line left of centre):

| | left margin | right margin | worst case |
|---|---|---|---|
| today | 360 − d px | 360 + d px | 360 − d px (left turns lose the line first) |
| offset 0 | 360 px (100 %) | 360 px (100 %) | 360 px |
| gain | | | **+d px = +d/360 of half width** on the weaker side, −d px on the stronger side |

Examples: d = 20 px → 94 % / 106 %, gain 5.6 %; d = 40 px → 89 % / 111 %, gain 11 %;
d = 60 px → 83 % / 117 %, gain 17 %; d = 80 px → 78 % / 122 %, gain 22 %.
The real d comes from the robot (item 3).

### 5. Physical shift

From the camera height 20 cm and tilt 45° (`ROBOT_TEST_CHECKLIST.md`), at the strip (row 708):

| FOV used | cm per px | floor half width | strip distance ahead of camera |
|---|---|---|---|
| real vFOV 72.5° (logged on the phone) | **0.032 cm/px** | 11.5 cm | 9.7 cm |
| config 60° (what the pipeline assumes) | 0.027 cm/px | 9.6 cm | |

The robot centre's shift from the line after the change ≈ the camera's sideways distance from the
robot centre line (about d × 0.032 cm, e.g. 40 px ≈ 1.3 cm, 60 px ≈ 1.9 cm). Measuring the
camera-to-centre distance with a ruler gives it directly. Yaw adds 0.17 cm per degree.

### 6. Report only: does the code slow down in sharp turns?

Yes, but by **look-ahead lean and visible road**, not by the steering value:
- `:2860–2869` and `farIntersection :4191`: intersection = |look-ahead X − trackRefX| /
  (halfW × `trackOffsetFullFrac` = 1.0) × 100, raised by `trackLengthSlowdown` (true, with
  `trackLengthFullFrac` = 0.8) when less road is visible. In v3 the curve sharpness `curveC` also
  enters.
- `:1494`: tracking = max(`trackingMinValue` = 5, 100 − intersection). README: hard turn → 5–30.
- The robot's Blockly program uses it only with x = 1:
  motor power = round(x × Power / 8 + 5). At x = 0 (the recommended mode at our speed), **power
  is constant** and nothing slows down in turns. Which x runs on the robot is not in this repo.
- Related: tracking uses `trackRefX`. If `steerRefX` changed without it, a straight line would
  read as a lean and lower the power. That is why both must move together.

### 7. Proposed minimal change for Phase 2

The pipeline has no config value for the offset, so **one small pipeline change is needed**. It is
done at the source, so all the uses above stay consistent:

- Library `Config`: `public float steerRefOffsetScale = 1.0f;` (1 = the calibrated offset,
  original; 0 = no offset, steer the line to the image centre).
- Calibration, `:1442–1445` only:
  `steerRefX = scale == 1f ? median : W/2 + (median − W/2) × scale;` and `trackRefX` from it, as
  today. With 1.0, the code path is the original one, so the output is byte-identical.
- `headingRefDeg` (the yaw zero), the colour, width and level calibration: **unchanged**.
- App `MainActivity`: `CAMERA_OFFSET_SCALE_ORIGINAL = 1.0f`,
  `CAMERA_OFFSET_SCALE = 0.0f`, passed as `cfg.steerRefOffsetScale`. To revert, set
  `CAMERA_OFFSET_SCALE = CAMERA_OFFSET_SCALE_ORIGINAL`.

**Rejected alternative:** changing only the steering formula (`offsetTerm`). The search
preference, the edge-pin side, the glare take-back side and the power reference would still use
the old column, and a straight line would read as a lean (lower power).

---

## Phase 2 Report (Offset Removed)

### The change (3 + 3 lines)

- Library `Config`: `public float steerRefOffsetScale = 1.0f;` (with a comment).
- Library calibration, right after the median:
  `if (cfg.steerRefOffsetScale != 1f) steerRefX = W/2 + (steerRefX − W/2) × scale;`
  At 1.0 the original line runs alone. `trackRefX` is computed from `steerRefX` on the next line,
  as before. `headingRefDeg` is unchanged.
- App `MainActivity`: `CAMERA_OFFSET_SCALE_ORIGINAL = 1.0f`, `CAMERA_OFFSET_SCALE = 0.0f`,
  `cfg.steerRefOffsetScale = CAMERA_OFFSET_SCALE`.

### Verification in the simulator (robot configuration)

Robot-like closed loop: frame 720 × 960, real vFOV 72.5° rendered (config 60°), 30 fps, camera
20 cm high, 45°, 8 cm ahead of the axle, mounted 1, 2 or 3 cm right of the centre line.
Row-wise base 48 and trend hold on. The robot calibrates with its centre over the line, then the
lecturer's Blockly law drives it. Two colour schemes, with and without distractors, c = 0.5 / 1.0,
x = 0 / 1. 144 runs per cell. Harness: `_investigation/harness` (copied to the scratchpad and
extended with frame size, camera offset, FOV, fps, left/right turn tracks and pin counting).
- **real:** 150 ms latency, 30 Hz, dead zone 2, power 5 ≈ 5 cm/s.
- **stress:** 3× faster, 200 ms, 15 Hz, dead zone 3.
- **glare:** real, plus glare over the strip in 3 lamp zones.

**1. Original value (1.0): identical to the baseline tag.**
- Closed loop: all 1,296 runs (3 regimes × 3 offsets × 144) give the same CSV as
  `robot-approved-before-offset-removal`, timing column excluded.
- Frame by frame: every FrameData field and every output, 304 drives, 320 × 240, 720 × 1280
  and 720 × 960, gray and YUV, with glare and gaps, robot config: **byte-identical** to the baseline (152 drives at 320 × 240 / 720 × 1280 and 32 at 720 × 960, at the default and with 1.0 set explicitly). At 0 the same 180 HOLD frames occur in the 720 × 960 set.

**2. Offset 0, before → after.** Cells: runs finished / runs, then line-lost events. Left 90° and
right 90° are 90° corners of radius 4, 10 and 20 cm; S = two 25 cm 90° arcs.

| Regime | cam offset | Left 90° | Right 90° | S, left first | S, right first | Straight (steer std / pins) |
|---|---|---|---|---|---|---|
| real | 1 cm | 41/48, 34 → 46/48, 17 | 47/48, 17 → 46/48, 20 | 16/16, 0 → 16/16, 0 | 16/16, 0 → 16/16, 0 | 0.56 / 0 → 0.85 / 0 |
| real | 2 cm | 43/48, 41 → 43/48, 28 | 47/48, 17 → 46/48, 15 | 16/16, 0 → 16/16, 0 | 16/16, 0 → 16/16, 0 | 0.44 / 0 → 1.17 / 0 |
| real | 3 cm | 43/48, 32 → 46/48, 14 | 48/48, 11 → 45/48, 18 | 16/16, 0 → 16/16, 0 | 16/16, 0 → 16/16, 0 | 0.30 / 0 → 1.50 / 0 |
| stress | 1 cm | 36/48, 67 → 37/48, 53 | 36/48, 56 → 36/48, 62 | 15/16, 13 → 11/16, 19 | 14/16, 20 → 15/16, 30 | 4.15 / 3 → 11.05 / 28 |
| stress | 2 cm | 34/48, 77 → 35/48, 62 | 37/48, 63 → 36/48, 69 | 13/16, 17 → 12/16, 29 | 12/16, 21 → 15/16, 33 | 4.00 / 4 → 12.37 / 28 |
| stress | 3 cm | 32/48, 87 → 36/48, 53 | 37/48, 64 → 37/48, 64 | 13/16, 20 → 11/16, 22 | 12/16, 20 → 16/16, 22 | 4.08 / 3 → 13.40 / 36 |
| glare | 1 cm | 20/48, 384 → 25/48, 366 | 28/48, 459 → 28/48, 474 | 8/16, 206 → 9/16, 271 | 9/16, 296 → 10/16, 277 | 0.73 / 0 → 0.94 / 0 |
| glare | 2 cm | 21/48, 275 → 32/48, 295 | 38/48, 250 → 29/48, 248 | 9/16, 253 → 9/16, 223 | 6/16, 272 → 10/16, 241 | 0.60 / 0 → 1.23 / 0 |
| glare | 3 cm | 18/48, 304 → 27/48, 384 | 33/48, 346 → 27/48, 457 | 11/16, 226 → 9/16, 227 | 8/16, 267 → 9/16, 293 | 0.48 / 0 → 1.55 / 0 |

**What it shows**
- **Left turns improve** (the side that had the small margin): in real, lost events fall by
  32–56 % and finished runs go up at 1 and 3 cm. Under glare, finished runs go +5, +11 and +9.
- **Right turns get somewhat worse** (the side that had the extra margin): in real, −1 to −3
  finished runs. Under glare at 2–3 cm, −9 and −6. This is the trade-off the doc expected.
- **Straight line:** the larger std is the **start-up correction** after calibration: the robot
  moves the camera over the line, 1–3 cm sideways. Started with the camera already over the line,
  the straight steady state equals the baseline:

  | real, straight 2 m + 3 m | baseline | offset 0, camera over the line at start |
  |---|---|---|
  | 1 cm: steer std / max / sign changes per s | 0.58 / 1.16 / 0.12 | 0.59 / 1.20 / 0.13 |
  | 2 cm | 0.46 / 0.90 / 0.04 | 0.57 / 1.01 / 0.07 |
  | 3 cm | 0.32 / 0.73 / 0.01 | 0.47 / 0.92 / 0.03 |

  The stress-regime straight weave (std 11–13, pins) appears only with x = 1 (power follows
  Tracking, about 0.5 m/s). The **baseline weaves the same way** after the same 2 cm kick
  (std 11–13 with c = 0.5, 35 with c = 1.0): the fast loop is marginal on its own (README: keep
  x = 0 at that speed). The start-up correction only triggers it.
- The robot centre runs parallel to the tape, shifted by the camera offset: mean lateral error on
  the straight is 0.89 / 1.76 / 2.64 cm for 1 / 2 / 3 cm, as intended.
- **Trend hold, glare coast, edge-exit pin:** unchanged code, and identical at 1.0. At 0 they keep
  working: pins and holds still occur (pin events above).

### Build and tests

- App `assembleDebug`: builds.
- Library tests: **64 tests, 63 pass, 1 skipped, 0 failures**, the same counts as the baseline (52 original + 5 SteeringBaseTest + 7 TrendHoldTest). No test assumes the original offset, and none needed changing.
- Note: the first full run had 1 failure in `RecorderTest.recorderOffChangesNothingAndWritesNothing` (tracking 82 vs 83 on frame 71). It ran while 4 simulator jobs loaded the CPU. That test does not set `frameIntervalMs`, so its coast and hold clocks use the wall clock. It passed 3 / 3 alone and in the full run on an idle CPU. It is a pre-existing timing sensitivity, not caused by this change (the default path is byte-identical).

---

## Progress Tracking

- [x] Phase 0: State saved and tagged, configuration recorded, approved
  - [x] Working tree clean (latency code committed as `083518d`; `answer.txt` left untracked by choice)
  - [x] Configuration recorded (above)
  - [x] Tag `robot-approved-before-offset-removal` created on `083518d` and pushed
  - [x] Branch `offset-removal` created (one repo, so one branch)
  - [x] Approved
- [x] Phase 1: Discovery and measurement report delivered, change approved
  - [x] Report delivered (above)
  - [ ] Real offset d and line side read from the robot overlay (optional, by you)
  - [x] Proposed change approved
- [x] Phase 2: Offset removed, verified in the simulator, approved
  - [x] Change made (library 1 field + 2 lines, app 2 constants + 1 line)
  - [x] 1.0 byte-identical to the baseline (closed loop and frame by frame)
  - [x] Comparison table (left / right / straight) delivered
  - [x] Build and tests pass (64: 63 pass, 1 skipped)
  - [x] Approved
- [ ] Phase 3: Docs, checklist, commit and push
  - [x] Version: tag `v6.2.0` only (app `versionCode` stays 1, as in earlier rounds), as you chose
  - [x] `CHANGES.md` created in the root (no CHANGES.md existed): what, why, values, margin gain, revert
  - [x] Robot test checklist (in `CHANGES.md`)
  - [x] Committed and pushed on `offset-removal`, tag `v6.2.0` pushed, not merged to main
  - [ ] Approved
