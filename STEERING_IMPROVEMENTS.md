# Steering Improvements: Final Project Changes

## Context

This is a line-following robot. The current algorithm works **very well**: it is fast and stable. After a review meeting with our lecturer, we found two remaining issues to address before closing the project:

1. **Discontinuity points in turns.** At a few points in turns, the detected path "jumps" for a moment. We believe this happens because the **base region used to compute steering is too narrow**.
2. **The red zone no longer exists.** We changed the robot's physical structure. The area of the image that used to be excluded (the "red zone", where the robot's own body interfered with detection) is now clean, and the path is clearly visible there.

## Goal

Make exactly two improvements:

1. **Widen the steering base.** Compute steering from a wider region, including the area that was previously the red zone.
2. **Hold the trend on line loss.** If a discontinuity happens or the robot briefly loses the path, keep the recent trend of `power` and `steering` so the robot finds the path again quickly instead of reacting to bad data.

Do nothing else.

---

## Ground Rules (apply to every phase)

- **Do not break what already works.** The current behavior is the baseline we are protecting. Treat every change as risky.
- **Minimal, focused changes only.** No refactoring, renaming, reformatting, cleanup, or "improvements" outside the two goals.
- **Do not change existing tuning.** Leave gains (P/I/D or similar), base speed, thresholds, image processing, and color detection untouched unless a phase below explicitly requires it. If you think something else should change, propose it and wait.
- **Every new behavior must be switchable.** Add a config flag or constant for each change so we can return to the exact original behavior by changing one value. Keep the original code path intact behind the flag; do not delete it.
- **Keep old values.** When you change a parameter, keep the original value in a comment or as a named constant (e.g. `STEERING_BASE_ROWS_ORIGINAL`).
- **Work in phases and stop after each one.** At the end of each phase, show the diff, explain what changed and why, give test instructions, and **wait for my approval** before continuing.
- **Ask before guessing.** If the code is ambiguous (for example, it is unclear which region is the steering base or which code handles the red zone), stop and ask.
- **I test on the real robot.** You cannot run the robot. Make sure the code at least runs or compiles where possible (syntax check, existing tests, a dry run on a recorded frame or video if one exists). Do not claim it "works on the robot."

---

## Phase 0: Safety Snapshot and Code Discovery (no code changes)

**Tasks**
1. If the project uses git: make sure the working tree is clean, commit or tag the current state as `baseline-before-steering-improvements`, and create a branch `steering-improvements`. If it does not use git, copy the relevant source files to a `backup_baseline/` folder.
2. Read the code and find:
   - Where the **steering base / ROI** is defined: rows, columns, band height and width, sample points, or lookahead distance.
   - Exactly **how steering is computed** from that region: centroid offset, angle between points, line fit, error to center, and so on.
   - Where the **red zone** is defined and excluded: mask, row/column cutoff, if-condition, or constants.
   - Where **power** (speed) and **steering** are computed and sent to the motors.
   - What currently happens when **the line is not detected** or detection is unreliable.
   - The control loop rate (frames per second or loop period), so time-based limits can be set correctly.

**Deliverable:** A short report listing files, functions, line numbers, and current parameter values for each item above, plus a short proposed plan for Phases 1 and 2 based on what you found.

**Done when:** The baseline is saved, the report is delivered, and I have approved the plan.

---

## Phase 1: Widen the Steering Base

**Tasks**
1. Remove the red-zone exclusion **behind a flag** (e.g. `USE_RED_ZONE_EXCLUSION = False`). Keep the original exclusion code reachable by setting the flag to `True`.
2. Widen the region used for steering so it uses more of the image, including the former red zone. Depending on how the code works, this may mean:
   - more rows or a taller band,
   - more sample points spread along the path, or
   - a fit (for example, a line or low-order polynomial) over the whole band instead of one or two narrow points.
   Choose the approach that fits the existing design with the smallest change, and explain why.
3. **Keep the steering output scale consistent.** A wider base can change the magnitude of the steering value (for example, if steering comes from an angle or a distance between points). Normalize it so that, on a straight line and a gentle turn, steering values stay in the same range as before. Otherwise the existing gains will effectively change.
4. Make the new region size configurable (e.g. `STEERING_BASE_ROWS`, `STEERING_BASE_WIDTH`), with the original values kept as named constants.
5. If there is an existing debug view or overlay, draw the new steering base on it so I can see the region during testing. Debug only; it must not affect control.

**Done when:**
- The flag switches cleanly between the old and new base.
- The code runs or compiles without errors.
- You have given me the diff, the old and new region parameters, and a test checklist (straight line, gentle turn, the turns that showed discontinuities, the former red-zone area).
- **Stop and wait for my test results.**

---

## Phase 2: Hold the Trend on Discontinuity or Line Loss

**Tasks**
1. **Define a discontinuity** clearly and in one place, configurable:
   - the line is not detected, or detection quality is below a threshold (e.g. too few pixels or points), **or**
   - the detected position or steering jumps by more than `MAX_STEERING_JUMP` compared to the recent trend in a single frame.
2. **Track the recent trend** of valid `steering` and `power` values, for example a short moving average over the last `TREND_WINDOW` valid frames.
3. **During a discontinuity:**
   - keep using the trend values for `steering` and `power` instead of the bad measurement,
   - do not let the bad frame update the trend, and
   - keep this up for at most `MAX_HOLD_TIME` (milliseconds or frames, based on the loop rate found in Phase 0).
4. **When the path is found again:** resume normal control. If switching back causes a visible jerk, blend from the held values to the measured values over a few frames (`RECOVERY_BLEND_FRAMES`). Keep this simple.
5. **If the hold time runs out** and the path is still lost: fall back to exactly what the code does today on line loss. Do not invent new behavior.
6. Put the whole feature behind a flag (e.g. `USE_TREND_HOLD = True`).
7. Add a simple state indicator (`TRACKING` / `HOLDING` / `LOST`) to the existing log or debug overlay so I can see when the hold activates.

**Done when:**
- The flag switches cleanly between the old and new behavior.
- Normal tracking (no discontinuity) is **unchanged**. Confirm in code that the trend logic does not alter `power` or `steering` while tracking is valid.
- The code runs or compiles without errors.
- You have given me the diff, every new parameter with its default value and a short explanation, and a test checklist (normal laps, the problem turns, briefly covering or interrupting the line, a longer loss to check the timeout fallback).
- **Stop and wait for my test results.**

---

## Phase 3: Tuning and Wrap-Up

**Tasks**
1. Based on my test feedback, adjust **only** the new parameters from Phases 1 and 2. Do not touch the original tuning.
2. Once I confirm the results are good:
   - Make sure all flags default to the new, approved behavior.
   - Make sure the original behavior is still reachable through the flags.
   - Remove temporary debug prints that are no longer needed (keep the optional overlay or state indicator if it is behind a debug flag).
3. Write a short `CHANGES.md` with: what changed, why, every new parameter and its final value, how to revert to the baseline (flags plus the git tag or backup), and known limitations.
4. If using git, commit with a clear message.

**Done when:** I have approved the final behavior and `CHANGES.md` is in the repo.

---

## Progress Tracking

Keep this checklist updated in this file as you go:

- [x] Phase 0: Baseline saved, code report delivered, plan approved
  - [x] Baseline saved (2026-11-14): branch `steering-improvements`, tag `baseline-before-steering-improvements` (commit 4965f2b). Library tests at baseline: 50 pass, 1 skipped (CapturedScenesTest, needs phone captures).
  - [x] Code report and proposed plan delivered (2026-11-14)
  - [x] Open questions answered, plan approved (2026-11-14). Robot host = RoboPhoneApp `MainActivity` (the only compiled Config); Phase 2: hold keeps power (`holdPowerScale`, default 1.0), glare coast untouched, edge exits never held.
- [ ] Phase 1: Wider steering base implemented, tested by me, approved
  - [x] Implemented (2026-11-14), not committed: flags `useRedZoneExclusion`, `wideSteeringBase` / `steeringBaseRows`, `steeringScaleNormalize` in Config; switched on in `MainActivity`. Library tests 53 pass, 1 skipped; app builds.
  - [x] Library defaults switched to the new base (2026-11-14): `useRedZoneExclusion = false`, `wideSteeringBase = true`.
  - [x] Original flags verified identical to the baseline tag: 112 simulated drives, every output field byte-identical; original tests 50 pass, 1 skipped.
  - [ ] Open: with the new defaults 4 existing tests fail (StraightLineTest ±1 dither 31/30; TrackSimTest and edgeExitUnderGlare scene setups; sharpCurveNearGlare steering against the line). Glare + sharp curve is worse with the new base at 720×1280 too (53 vs 17 frames). Waiting for a decision.
  - [x] steeringBaseRows sweep 72 / 48 / 36 / 24 (2026-11-14): no value removes the glare regression (glare band over the strip: 146–207 steering jumps ≥ 20 at every height vs 10 at baseline; sharpCurveNearGlare fails at every height) and the straight-line dither is over its limit at every height (31–69 vs 30). Clean drives: no jumps at any height. Stopped as instructed: tests not pinned, nothing committed. Waiting for a decision.
  - [x] Option 3 (strip widened upward U72 / U48, strip moved to 0.78 / 0.81 with 48 rows): every candidate has a glare regression (wrong-side pins, or 12–115 steering jumps ≥ 20 vs 0–10). Not adopted.
  - [x] Row-wise steering base (2026-11-14): `rowWiseSteeringBase`, `rowWiseBaseRows`, `rowWiseGroupRows`, `rowWiseMinValidFrac`. Strip and glare code unchanged; the offset comes from the median of row groups above the strip, glared / lineless groups dropped. Sweep: no regression in the 112 drives or the glare scenes (identical to baseline in all glare cells). Scenario tests: RW72 fails StraightLineTest (dither 77/150, 31 sign flips) and sharpCurveNearGlare; RW48 fails sharpCurveNearGlare (capped pin 20 vs allowed 18). Not default.
  - [x] Defaults back to the original (`useRedZoneExclusion = true`, `wideSteeringBase = false`, `rowWiseSteeringBase = false`); all new code kept behind the flags; the app uses the original too. Original flags verified byte-identical to the baseline tag (112 drives). Tests: original 50 pass + 1 skipped, plus 5 new SteeringBaseTest pass. Build passes.
  - [x] Approved (option 1): original defaults, row-wise base 48 rows behind `rowWiseSteeringBase` for robot tests. Committed on `steering-improvements` (2026-11-14).
  - [ ] Tested on the robot

**How to test Phase 1 on the robot.** In `RoboPhoneApp/app/src/main/java/com/example/firstproject/MainActivity.java` (constants near the top of the class):

| Constant | Robot test | Original (revert) |
|---|---|---|
| `USE_ROW_WISE_BASE` | `true` | `false` |
| `ROW_WISE_BASE_ROWS` | `48` (recommended) | — (not used when off) |
| `USE_RED_ZONE_EXCLUSION` | `true` (keep) | `true` |
| `USE_WIDE_STEERING_BASE` | `false` (keep) | `false` |

Rebuild and install the app. On screen, a dashed white "BASE" box appears above the cyan strip (rows 912–960 of 1280); its label shows how many row groups found the line cleanly, "(strip)" when the strip's own position was used. To revert: `USE_ROW_WISE_BASE = false` (exactly the original steering, proven byte-identical in the simulator), or check out the tag `baseline-before-steering-improvements`.

**Known limitation (Phase 1):** no variant of the wider steering base passed all simulator checks, so the robot runs the original base. In the simulator, moving or widening the strip itself makes glare handling worse. The row-wise base leaves glare handling unchanged, but its unfiltered per-frame position adds straight-line dither (72 rows) and steers about 2 units harder than allowed in one glare case (48 rows). To try it on the robot: `USE_ROW_WISE_BASE = true` in `MainActivity` (48 rows); the overlay then shows the base and "BASE valid/groups".
- [ ] Phase 2: Trend hold implemented, tested by me, approved
- [ ] Phase 3: Tuned, cleaned up, `CHANGES.md` written
