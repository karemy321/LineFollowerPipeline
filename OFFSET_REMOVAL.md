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

## Progress Tracking

- [ ] Phase 0: State saved and tagged, configuration recorded, approved
  - [x] Working tree clean (latency code committed as `083518d`; `answer.txt` left untracked by choice)
  - [x] Configuration recorded (above)
  - [x] Tag `robot-approved-before-offset-removal` created on `083518d` and pushed
  - [x] Branch `offset-removal` created (one repo, so one branch)
  - [ ] Approved
- [ ] Phase 1: Discovery and measurement report delivered, change approved
- [ ] Phase 2: Offset removed, verified in the simulator, approved
- [ ] Phase 3: Docs, checklist, commit and push
