# LineFollowerPipeline v3 — glare robustness, corridor tracker, distractor hardening

Branch `v3` (from `202e70f`, the v2 state). Plan and decisions: `LINE_FOLLOWER_V3_PLAN.md`
(§11 user requirements, §12 capped edge pin, §13 bounded blob slowdown). On-robot tests:
`ROBOT_TEST_CHECKLIST.md`. **Everything below is simulation; robot results are pending.**

## 1. Results in short

| | v2 (`202e70f`) | v3 |
|---|---|---|
| Glare scenarios (`V3ScenarioTest`, 0.6 m/s, 30 fps, gray + YUV, 3–4 colour pairs) | glare over the strip: steering kicked to ±50 after the 6-frame hold (14/14 runs fail); glare band ahead read as a dead end (tracking 100 → 23, 6/6 fail); highlight on the tape centre: 8/14 (saturated) and 6/10 (partial) runs fail | **all pass**: never pinned, \|Δsteer\| ≤ 6 per frame, within 10 of the clean view; tracking not dropped; tape found with centre error ≤ 0.3 line width |
| Edge exit while glare hides the strip | held / lost before the exit (4/4 fail) | pinned to the exit side, at once or before v2 (pass) |
| Sharp curve (R 15 cm) under near glare | — (v2 holds, then pins) | 0 wrong-direction frames; in each spell where the line runs off the frame edge above the glare the first 5 frames are capped at the filter's steering (median 6, worst 16 from v2 on the clean view), never ±50 |
| Distractors: short object at the straight-ahead column when re-acquiring; line running into a dark mat | locked on the object 3+ frames; tracking stays 100 across the mat | never locked; tracking ramps to the minimum like a plain dead end |
| Distractors already handled by v2 (same-width object, blob, soft shadow) | pass | pass (beside a blob of the line's colour, tracking dips while the blob fills the far view — bounded at ≥ 60 once the line is seen past it) |
| 90° corner, dead end vs the v2 settings | — | steering within 5, tracking within 10 of v2 |
| Unit tests | 26/26 | **44/44** (26 original — `lostLineReportsTrackLost` and the v1 config now set `glareMask = false`, as planned — + 4 coast-clock tests, 13 scenarios, 1 allocation test) |
| `corridorMode = false, glareMask = false` | — | **identical to v2 on every frame** (72 runs × 40 frames, 4 colour pairs × 6 scenes incl. glare × gray / YUV / ARGB, 13 FrameData fields), re-checked after every phase |
| Heap allocation per 1000 frames (gray / YUV / ARGB, glare + lost frames) | 0 / 0 / 0 B | **0 / 0 / 0 B** (`AllocationTest`) |
| ms per frame, 960×1280 YUV, desktop, same JVM, alternating blocks (`gradlew perf`) | 1.03–1.07 | **0.59–0.65** (clean 0.74–0.77, glare over the strip 0.17–0.32, tape glare 0.83–0.86); p99 1.35–1.45 |
| Closed loop, glare over the near strip (realistic speed / 3× speed, 40 runs each) | 11 / 7 completed | **37 / 32** completed |
| Closed loop, no glare (realistic / 3× speed) | 120 / 72 completed | 120 / 73 completed |

## 2. What changed (by phase)

| Phase | Commit | Change |
|---|---|---|
| 0 | `ec1b356` | `TrackSim`: image-fixed glare (saturating / partial / tape-centre only) and extra objects; `V3ScenarioTest` scenarios tagged with the phase that fixes them; `AllocationTest`; `PerfBench` (`gradlew perf`); `gradlew checkJava8`. |
| 1 | `e15b3b8` | `glareMask`: saturated pixels masked (Y ≥ `glareLumaMax`; ARGB any channel ≥ 250, tested from the Y plane directly); local Otsu per eighth of the strip; look-ahead traced without the base and the base rebuilt from the floor fit; glare rows are not gaps; time-based coast (`glareCoastMs`, 100 ms clamp per frame, `glareCoastMaxFrames`, `frameIntervalMs`); edge exits pinned at once. |
| 2 | `6d3011c` | `corridorMode`: 12 slices, edge-pair detector (glare columns interpolated — bridges a brightened tape centre), Kalman filter on the floor curve, far-only measurements of heading / curvature with a weak position, hand-over to v2 for jumps / corners / dead ends / edge exits / low confidence / relocks; calibration measures a width gain per slice; lazy line-ness rows; app PERF logging. |
| 2b | `7f0bd76` | Capped edge pin (§12): under glare and while the filter was following a curve toward that edge, the pin is capped at the filter's steering for ≤ 166.7 ms, then ±50. |
| 3 | `02d8cbd` | Continuity helper; D1 continuity-ranked (re)acquisition; D2 trimmed bands / cut look-ahead rows; D3 `edgePairCheck`. |
| — | `efaaf50` | Bounded D2 slowdown (§13): tracking ≥ 60 when the line is seen continuing past a blob. |
| 4 | `055630a` | App: AE/AWB lock after calibration (Camera2Interop), EV compensation, v3 HUD line — all off by default. |
| 5 | `4550e56` + this | Filter tuning from the closed loop (see §5); harness glare; README, PDF, this report. |

Public API: unchanged — only Config fields (`glareMask`, `glareLumaMax`, `glareCoastMs`,
`glareCoastMaxFrames`, `frameIntervalMs`, `corridorMode`, `corridorSlices`, `corridorWidthMult`,
`minChainSlices`, `corridorMinConfidence`, `corridorProcessNoise`, `edgePairCheck`) and FrameData
fields (`baseReconstructed`, `coasting`, `maskedFrac`, `corridorCoverage`, `corridorFitRms`,
`curveA`, `curveB`, `curveC`) were added. One file, Java 8 (`checkJava8`), no dependencies.

## 3. How v3 handles glare (plain words)

1. **Saturated pixels are "unknown"**, not line and not floor. They are left out of the thresholds;
   inside a band they count as part of it, so a highlight on the tape centre does not split the line.
2. **The line ahead stands in for the hidden near line.** If glare covers the scan strip, the line is
   followed from the look-ahead and its position in the strip is rebuilt from it. When the visible
   part is only far ahead and curves sharply, rebuilding would be a guess, so the pipeline coasts.
3. **The corridor needs both edges of the tape** in each slice. The two outer edges of a tape whose
   centre is brightened (Y ≈ 150–220) still form the best pair; the inner edges of the highlight form
   pairs a third as wide and are never chosen.
4. **Far-only observations still count**: they update the filter's heading and curvature, the near
   position comes from its prediction, and Tracking never rises while unsure (lower for a sharp curve).
5. **Coast, but never through an exit**: hidden mid-image ⇒ coast ≤ 600 ms with fading heading and
   falling Tracking; leaving at the edge ⇒ ±50 at once, capped only in the §12 case.

## 4. Unit / scenario tests (`LineFollowerLibrary`, `gradlew test`, ~1 min)

On the v2 code (Phase 0 baseline) → v3:

| Scenario | v2 | v3 |
|---|---|---|
| `glareNearStrip` (straight + curve, 40 glare frames, incl. 60 fps) | 14/14 runs fail | pass |
| `glareMidLookAhead` | 6/6 fail | pass |
| `glareOnTapeOnly` (saturated centre) | 8/14 fail | pass |
| `glareOnTapeOnlyPartial` (Y ≈ 150–220, req. 3) | 6/10 fail | pass |
| `edgeExitUnderGlare` (req. 4) | 4/4 fail | pass |
| `sharpCurveNearGlare` (Phase 2 request) | — | pass (see §1) |
| `reacquireBesideShortObject` (D1) | 6/6 fail | pass |
| `deadEndIntoMat` (D2) | 6/6 fail | pass |
| `softShadowBesideLine` (D3), `sameWidthObjectBesideLine`, `blobTouchingLine` (12 cm + 4 cm) | pass | pass |
| `ninetyDegreeCorner`, `lineEnds` (v3 vs the v2 settings) | — | pass |

## 5. Closed-loop simulator

Simulator (`_investigation/harness`, from v2): differential-drive robot, 960×1280 camera frames
rendered at 60 Hz (20 cm high, tilt 45°, 60° vFOV, 8 cm ahead of the axle, 1 cm right), the
lecturer's Blockly law `motor steering = (0 − Steering) × c`, `power = round(x × Power/8 + 5)`,
first-order motor lag, command latency, Blockly loop rate and motor dead zone. **New for v3:**
`-Dsim.glare=near` — an image-fixed saturated ellipse over the scan strip and the nearest ~20 % of
the look-ahead, switched on in three "lamp zones" of 15 % of the track each. Command:
`./run_v3_closed_loop.sh` (560 runs, ~10 min on 20 threads).

**Reading the tables**
- **Glare** is where v3 differs: realistic speed **11 → 37 of 40** completed, 3× speed **7 → 32 of
  40**; frames on the wrong object 29 % → 1 %, frames without the line 31 % → 7 %. v2 typically left
  the track at the first lamp zone (±50 kick after the 6-frame hold, or a wrong lock on the glare).
  The remaining v3 failures are the oval and the sharp 90° corner at 3× speed / c = 1, x = 1.
- **No glare, realistic speed:** 120 / 120 for both; mean and worst lateral errors equal within
  0.1–0.2 cm (curves slightly better, sharp corner 0.2 cm worse).
- **No glare, 3× speed (≈ 0.5 m/s):** 72 → 73 of 80. **On the straight, x = 1 with c = 1.0 weaves**
  (steering std ≈ 34, lateral ±3 cm, still completes) — this regime is at the stability limit of the
  loop (200 ms latency, 15 Hz): v2 reads exactly 0 on a perfect straight and so is never excited,
  v3's small steering dither (below) is enough to start the oscillation. CHANGES_v2 §6.2 already
  showed v2 weaving at c = 1 / x = 1 in this regime after a disturbance. With x = 0 the straight is
  calm (std 1–2, ≤ 0.3 cm).
- **Steering sign changes** rise (realistic 0.10 → 0.33 /s): on a straight v3 steering dithers by ±1
  where v2 reads 0 in the simulator. A first version dithered more (0.51 /s); the heading estimate
  wandered. Fixed in Phase 5 (`4550e56`): heading / curvature process noise that depends on the
  curvature (small on straights), slice 0 from the strip's band centre when it is a clean full band,
  and 5–13 rows per near slice. Straight-line runs are now 0.1–0.75 sign changes /s, ≤ 0.3 cm.
- `lib ms` is the library time per frame with 18 runs sharing the CPU; see §1 for the clean timing.

Tracks: straight 2 m, gentle curve R 0.8 m, S-curve R 0.4 m, sharp ~90° corner (R 4 cm), oval with R 0.35 m ends; floors white_on_dark and black_on_wood; with / without distractors (not in the glare regimes); Blockly gain c and power mode x as listed. A run fails when the axle is > 12 cm from the line. v3 runs with `frameIntervalMs = 16.667` (the simulator renders at 60 Hz, not in real time). `lib ms` is the library time per frame while 18 runs share the CPU (not comparable with `gradlew perf`).


### Regime `real` — realistic timing (150 ms latency, 20 Hz Blockly loop, dead zone 2), 5–17 cm/s

| runs | completed v2 → v3 | mean lateral err cm | worst lateral err cm | steering sign changes /s | lost % of frames | wrong-lock % | lib ms/frame |
|---:|---|---|---|---|---|---|---|
| 120 | 120 → 120 | 1.60 → 1.62 | 10.4 → 10.6 | 0.10 → 0.33 | 1.3 → 1.4 | 0.06 → 0.15 | 2.16 → 1.66 |

| track | distractors | completed v2 → v3 | mean lat cm | worst lat cm | lost % | wrong-lock % |
|---|---|---|---|---|---|---|
| gentle | no | 12/12 → 12/12 | 1.30 → 1.29 | 3.1 → 3.0 | 0.0 → 0.0 | 0.00 → 0.00 |
| gentle | yes | 12/12 → 12/12 | 1.30 → 1.29 | 3.1 → 3.0 | 0.0 → 0.0 | 0.00 → 0.00 |
| oval | no | 12/12 → 12/12 | 2.85 → 2.85 | 7.8 → 7.7 | 0.0 → 0.0 | 0.00 → 0.00 |
| oval | yes | 12/12 → 12/12 | 2.85 → 2.85 | 7.8 → 7.7 | 0.0 → 0.0 | 0.00 → 0.00 |
| scurve | no | 12/12 → 12/12 | 2.16 → 2.15 | 5.9 → 5.8 | 0.0 → 0.0 | 0.00 → 0.00 |
| scurve | yes | 12/12 → 12/12 | 2.16 → 2.15 | 5.9 → 5.8 | 0.0 → 0.0 | 0.00 → 0.00 |
| sharp90 | no | 12/12 → 12/12 | 1.68 → 1.72 | 10.4 → 10.6 | 6.2 → 7.4 | 0.08 → 0.67 |
| sharp90 | yes | 12/12 → 12/12 | 1.74 → 1.75 | 10.4 → 10.5 | 6.3 → 6.7 | 0.48 → 0.87 |
| straight | no | 12/12 → 12/12 | 0.00 → 0.07 | 0.0 → 0.3 | 0.0 → 0.0 | 0.00 → 0.00 |
| straight | yes | 12/12 → 12/12 | 0.00 → 0.07 | 0.0 → 0.3 | 0.0 → 0.0 | 0.00 → 0.00 |

| c | x | completed v2 → v3 | mean lat cm | worst lat cm | sign changes /s |
|---|---|---|---|---|---|
| 0.50 | 0 | 20/20 → 20/20 | 0.91 → 0.92 | 5.4 → 6.3 | 0.03 → 0.18 |
| 0.50 | 1 | 20/20 → 20/20 | 1.04 → 1.05 | 5.2 → 5.4 | 0.09 → 0.25 |
| 1.00 | 0 | 20/20 → 20/20 | 1.68 → 1.69 | 9.2 → 9.2 | 0.05 → 0.19 |
| 1.00 | 1 | 20/20 → 20/20 | 1.77 → 1.81 | 9.2 → 9.5 | 0.07 → 0.27 |
| 2.00 | 0 | 20/20 → 20/20 | 2.06 → 2.08 | 10.3 → 10.6 | 0.05 → 0.26 |
| 2.00 | 1 | 20/20 → 20/20 | 2.16 → 2.17 | 10.4 → 10.5 | 0.32 → 0.86 |

### Regime `stress` — 3× faster (power 17 ≈ 0.5 m/s), 200 ms latency, 15 Hz, dead zone 3

| runs | completed v2 → v3 | mean lateral err cm | worst lateral err cm | steering sign changes /s | lost % of frames | wrong-lock % | lib ms/frame |
|---:|---|---|---|---|---|---|---|
| 80 | 72 → 73 | 1.36 → 1.45 | 12.2 → 12.2 | 0.42 → 0.70 | 8.4 → 8.3 | 0.92 → 0.69 | 3.65 → 2.60 |

| track | distractors | completed v2 → v3 | mean lat cm | worst lat cm | lost % | wrong-lock % |
|---|---|---|---|---|---|---|
| gentle | no | 8/8 → 8/8 | 1.00 → 1.00 | 3.9 → 4.0 | 4.6 → 4.5 | 0.00 → 0.00 |
| gentle | yes | 8/8 → 8/8 | 1.00 → 1.00 | 3.7 → 4.2 | 3.8 → 4.5 | 0.55 → 0.29 |
| oval | no | 8/8 → 8/8 | 2.01 → 2.07 | 8.1 → 8.0 | 6.8 → 7.0 | 0.06 → 0.05 |
| oval | yes | 6/8 → 6/8 | 2.15 → 2.20 | 12.0 → 12.1 | 6.0 → 9.0 | 5.08 → 1.81 |
| scurve | no | 8/8 → 8/8 | 1.64 → 1.63 | 5.7 → 6.1 | 5.2 → 4.9 | 0.00 → 0.00 |
| scurve | yes | 6/8 → 6/8 | 1.94 → 1.92 | 12.1 → 12.1 | 11.4 → 11.5 | 0.51 → 0.30 |
| sharp90 | no | 8/8 → 7/8 | 1.98 → 1.89 | 9.8 → 12.1 | 19.5 → 18.9 | 0.25 → 0.56 |
| sharp90 | yes | 4/8 → 6/8 | 1.91 → 1.86 | 12.2 → 12.2 | 27.1 → 19.6 | 2.75 → 3.05 |
| straight | no | 8/8 → 8/8 | 0.00 → 0.44 | 0.0 → 3.1 | 0.0 → 2.0 | 0.00 → 0.00 |
| straight | yes | 8/8 → 8/8 | 0.00 → 0.50 | 0.0 → 3.2 | 0.0 → 1.5 | 0.00 → 0.79 |

| c | x | completed v2 → v3 | mean lat cm | worst lat cm | sign changes /s |
|---|---|---|---|---|---|
| 0.50 | 0 | 20/20 → 19/20 | 0.98 → 0.97 | 8.7 → 12.1 | 0.13 → 0.29 |
| 0.50 | 1 | 18/20 → 18/20 | 1.14 → 1.23 | 12.2 → 12.2 | 0.30 → 0.63 |
| 1.00 | 0 | 20/20 → 20/20 | 1.78 → 1.79 | 9.8 → 10.2 | 0.36 → 0.57 |
| 1.00 | 1 | 14/20 → 16/20 | 1.55 → 1.81 | 12.1 → 12.1 | 0.89 → 1.32 |

Runs whose outcome changed:

| track | floor | distractors | c | x | v2 | v3 |
|---|---|---|---|---|---|---|
| sharp90 | white_on_dark | yes | 0.50 | 0 | DONE | LEFT@0.78m |
| sharp90 | white_on_dark | yes | 0.50 | 1 | LEFT@0.84m | DONE |
| sharp90 | white_on_dark | yes | 1.00 | 1 | LEFT@0.81m | DONE |
| sharp90 | black_on_wood | no | 0.50 | 1 | DONE | LEFT@0.92m |
| sharp90 | black_on_wood | yes | 1.00 | 1 | LEFT@0.81m | DONE |

### Regime `glare` — realistic timing + glare over the near strip in 3 lamp zones (15 % of the track each)

| runs | completed v2 → v3 | mean lateral err cm | worst lateral err cm | steering sign changes /s | lost % of frames | wrong-lock % | lib ms/frame |
|---:|---|---|---|---|---|---|---|
| 40 | 11 → 37 | 3.01 → 1.57 | 12.1 → 12.1 | 0.16 → 0.27 | 30.7 → 6.9 | 29.07 → 1.26 | 3.02 → 2.46 |

| track | distractors | completed v2 → v3 | mean lat cm | worst lat cm | lost % | wrong-lock % |
|---|---|---|---|---|---|---|
| gentle | no | 2/8 → 8/8 | 3.69 → 1.01 | 12.0 → 2.3 | 34.3 → 0.0 | 25.00 → 0.00 |
| oval | no | 0/8 → 6/8 | 2.68 → 2.88 | 12.1 → 12.0 | 13.3 → 14.6 | 43.62 → 1.62 |
| scurve | no | 3/8 → 8/8 | 3.49 → 1.84 | 12.1 → 4.8 | 36.2 → 1.1 | 28.05 → 0.02 |
| sharp90 | no | 4/8 → 7/8 | 2.63 → 2.07 | 12.0 → 12.1 | 39.4 → 19.1 | 19.94 → 4.66 |
| straight | no | 2/8 → 8/8 | 2.57 → 0.06 | 12.0 → 0.3 | 30.1 → 0.0 | 28.75 → 0.00 |

| c | x | completed v2 → v3 | mean lat cm | worst lat cm | sign changes /s |
|---|---|---|---|---|---|
| 0.50 | 0 | 4/10 → 8/10 | 2.75 → 1.12 | 12.0 → 12.1 | 0.12 → 0.23 |
| 0.50 | 1 | 5/10 → 9/10 | 3.04 → 1.51 | 12.1 → 12.0 | 0.22 → 0.27 |
| 1.00 | 0 | 1/10 → 10/10 | 2.69 → 1.77 | 12.0 → 10.4 | 0.11 → 0.26 |
| 1.00 | 1 | 1/10 → 10/10 | 3.57 → 1.89 | 12.1 → 10.3 | 0.17 → 0.32 |

Runs whose outcome changed:

| track | floor | distractors | c | x | v2 | v3 |
|---|---|---|---|---|---|---|
| straight | white_on_dark | no | 0.50 | 0 | LEFT@0.52m | DONE |
| straight | white_on_dark | no | 0.50 | 1 | LEFT@0.56m | DONE |
| straight | white_on_dark | no | 1.00 | 0 | LEFT@0.93m | DONE |
| straight | white_on_dark | no | 1.00 | 1 | LEFT@0.98m | DONE |
| straight | black_on_wood | no | 1.00 | 0 | LEFT@0.47m | DONE |
| straight | black_on_wood | no | 1.00 | 1 | LEFT@0.52m | DONE |
| gentle | white_on_dark | no | 0.50 | 0 | LEFT@1.26m | DONE |
| gentle | white_on_dark | no | 0.50 | 1 | LEFT@1.27m | DONE |
| gentle | white_on_dark | no | 1.00 | 0 | LEFT@1.05m | DONE |
| gentle | white_on_dark | no | 1.00 | 1 | LEFT@1.07m | DONE |
| gentle | black_on_wood | no | 1.00 | 0 | LEFT@0.43m | DONE |
| gentle | black_on_wood | no | 1.00 | 1 | LEFT@0.41m | DONE |
| scurve | white_on_dark | no | 1.00 | 0 | LEFT@0.56m | DONE |
| scurve | white_on_dark | no | 1.00 | 1 | LEFT@0.55m | DONE |
| scurve | black_on_wood | no | 0.50 | 0 | LEFT@0.64m | DONE |
| scurve | black_on_wood | no | 1.00 | 0 | LEFT@0.52m | DONE |
| scurve | black_on_wood | no | 1.00 | 1 | LEFT@0.45m | DONE |
| sharp90 | white_on_dark | no | 0.50 | 0 | LEFT@0.46m | DONE |
| sharp90 | white_on_dark | no | 0.50 | 1 | LEFT@0.48m | DONE |
| sharp90 | black_on_wood | no | 0.50 | 0 | DONE | LEFT@0.87m |
| sharp90 | black_on_wood | no | 1.00 | 0 | LEFT@0.41m | DONE |
| sharp90 | black_on_wood | no | 1.00 | 1 | LEFT@0.41m | DONE |
| oval | white_on_dark | no | 0.50 | 0 | LEFT@0.73m | DONE |
| oval | white_on_dark | no | 1.00 | 0 | LEFT@0.79m | DONE |
| oval | white_on_dark | no | 1.00 | 1 | LEFT@2.03m | DONE |
| oval | black_on_wood | no | 0.50 | 1 | LEFT@1.05m | DONE |
| oval | black_on_wood | no | 1.00 | 0 | LEFT@0.74m | DONE |
| oval | black_on_wood | no | 1.00 | 1 | LEFT@0.74m | DONE |

### Regime `glarestress` — 3× faster + glare zones

| runs | completed v2 → v3 | mean lateral err cm | worst lateral err cm | steering sign changes /s | lost % of frames | wrong-lock % | lib ms/frame |
|---:|---|---|---|---|---|---|---|
| 40 | 7 → 32 | 3.07 → 1.60 | 12.2 → 12.2 | 0.51 → 0.88 | 34.1 → 15.2 | 27.88 → 1.17 | 2.36 → 3.40 |

| track | distractors | completed v2 → v3 | mean lat cm | worst lat cm | lost % | wrong-lock % |
|---|---|---|---|---|---|---|
| gentle | no | 1/8 → 8/8 | 3.45 → 1.03 | 12.1 → 4.4 | 41.0 → 6.5 | 25.10 → 0.50 |
| oval | no | 0/8 → 3/8 | 2.72 → 2.58 | 12.2 → 12.2 | 27.9 → 24.7 | 30.97 → 1.83 |
| scurve | no | 4/8 → 8/8 | 3.40 → 1.64 | 12.1 → 5.6 | 36.3 → 11.4 | 24.47 → 0.41 |
| sharp90 | no | 2/8 → 5/8 | 2.60 → 2.31 | 12.2 → 12.2 | 25.5 → 30.2 | 32.02 → 2.93 |
| straight | no | 0/8 → 8/8 | 3.17 → 0.45 | 12.1 → 4.5 | 40.1 → 3.2 | 26.84 → 0.17 |

| c | x | completed v2 → v3 | mean lat cm | worst lat cm | sign changes /s |
|---|---|---|---|---|---|
| 0.50 | 0 | 1/10 → 9/10 | 2.69 → 1.26 | 12.2 → 12.1 | 0.44 → 0.47 |
| 0.50 | 1 | 3/10 → 8/10 | 3.04 → 1.24 | 12.2 → 12.2 | 0.51 → 0.96 |
| 1.00 | 0 | 0/10 → 9/10 | 3.35 → 1.84 | 12.1 → 12.1 | 0.26 → 0.70 |
| 1.00 | 1 | 3/10 → 6/10 | 3.21 → 2.08 | 12.1 → 12.2 | 0.84 → 1.41 |

Runs whose outcome changed:

| track | floor | distractors | c | x | v2 | v3 |
|---|---|---|---|---|---|---|
| straight | white_on_dark | no | 0.50 | 0 | LEFT@0.57m | DONE |
| straight | white_on_dark | no | 0.50 | 1 | LEFT@1.22m | DONE |
| straight | white_on_dark | no | 1.00 | 0 | LEFT@0.06m | DONE |
| straight | white_on_dark | no | 1.00 | 1 | LEFT@0.17m | DONE |
| straight | black_on_wood | no | 0.50 | 0 | LEFT@0.56m | DONE |
| straight | black_on_wood | no | 0.50 | 1 | LEFT@1.22m | DONE |
| straight | black_on_wood | no | 1.00 | 0 | LEFT@0.07m | DONE |
| straight | black_on_wood | no | 1.00 | 1 | LEFT@1.03m | DONE |
| gentle | white_on_dark | no | 0.50 | 0 | LEFT@0.43m | DONE |
| gentle | white_on_dark | no | 0.50 | 1 | LEFT@1.26m | DONE |
| gentle | white_on_dark | no | 1.00 | 0 | LEFT@0.21m | DONE |
| gentle | white_on_dark | no | 1.00 | 1 | LEFT@1.00m | DONE |
| gentle | black_on_wood | no | 0.50 | 0 | LEFT@1.33m | DONE |
| gentle | black_on_wood | no | 1.00 | 0 | LEFT@0.11m | DONE |
| gentle | black_on_wood | no | 1.00 | 1 | LEFT@0.62m | DONE |
| scurve | white_on_dark | no | 0.50 | 0 | LEFT@1.33m | DONE |
| scurve | white_on_dark | no | 1.00 | 0 | LEFT@0.21m | DONE |
| scurve | white_on_dark | no | 1.00 | 1 | LEFT@1.75m | DONE |
| scurve | black_on_wood | no | 1.00 | 0 | LEFT@0.46m | DONE |
| sharp90 | white_on_dark | no | 0.50 | 0 | LEFT@0.93m | DONE |
| sharp90 | white_on_dark | no | 0.50 | 1 | LEFT@0.86m | DONE |
| sharp90 | white_on_dark | no | 1.00 | 0 | LEFT@0.04m | DONE |
| sharp90 | white_on_dark | no | 1.00 | 1 | DONE | LEFT@1.03m |
| sharp90 | black_on_wood | no | 0.50 | 0 | LEFT@0.93m | DONE |
| sharp90 | black_on_wood | no | 1.00 | 0 | LEFT@0.07m | DONE |
| sharp90 | black_on_wood | no | 1.00 | 1 | DONE | LEFT@0.85m |
| oval | white_on_dark | no | 1.00 | 0 | LEFT@0.35m | DONE |
| oval | black_on_wood | no | 0.50 | 0 | LEFT@0.83m | DONE |
| oval | black_on_wood | no | 0.50 | 1 | LEFT@2.59m | DONE |

## 6. Recommended starting values (Blockly program) — re-checked

Re-checked in the closed loop (§5) for v3:

| Setting | Start with | Result in simulation |
|---|---|---|
| Steering gain **c** | **`1.0`** (unchanged) | Realistic speed: 120 / 120 for c = 0.5, 1.0 and 2.0. Under glare c = 1.0 completed **20 / 20** (x = 0 and 1), c = 0.5 17 / 20. |
| Power mode **x** | **`0`** first; **`1`** only at low speed | At realistic speed x = 1 is fine (20 / 20). At **~0.5–0.6 m/s** (3× regime) x = 1 with c = 1.0 weaves on straights and completes 16 / 20 (glare: 6 / 10); with **x = 0, c = 1.0: 20 / 20** (glare: 9 / 10). If x = 1 is wanted at that speed, use c = 0.5 or a larger divisor (`Power / 12`). |
| Offset **+ 5** | `5` (unchanged) | Raise to 6–8 if the robot does not move at power 5. |
| Library | v3 defaults | `corridorMode`, `glareMask`, `edgePairCheck` on; `cameraTiltDeg` = real mount angle. For a v2 comparison on the robot: `corridorMode = false, glareMask = false`. |

These values come from simulation; the robot test (`ROBOT_TEST_CHECKLIST.md`, tests 1, 6) decides.

## 7. Known limitations / to verify on the robot

1. **Simulation only.** Glare is an image-fixed saturated ellipse; real highlights move, flicker and
   have colour fringes. Test 3 of the checklist is the important one.
2. **Capped pin accuracy** (§12): the capped value is a median of 6, at worst 16, from what v2 reads on a
   clean view — always the same direction, never ±50 (±50 stays the edge-exit signal).
3. **Blob of the line's own colour touching the line**: Tracking dips while the blob fills the far view
   (it looks exactly like a line running into a mat); bounded at ≥ 60 once the line is seen past it.
4. **Tape-width object at the predicted position while locked** can still be taken for the line while
   it is in the scan strip (v2 limit); it is no longer taken when re-acquiring.
5. **Straight-line steering** dithers by ±1 occasionally (closed loop: 0.1–0.75 sign changes per
   second, lateral error ≤ 0.3 cm) where v2 reads exactly 0 in the simulator.
6. **Camera geometry** (`cameraTiltDeg`, `verticalFovDeg`) still matters; the per-slice width gain
   from calibration absorbs small tilt errors only.
7. **Time base**: the coast uses `System.nanoTime()` on the phone; in simulation/replay set
   `frameIntervalMs`.
8. **App settings** (AE/AWB lock, EV) are untested on a real phone; with all off the camera set-up is
   unchanged.

## 8. How to rerun

```bash
cd LineFollowerLibrary
gradlew test                    # 44 tests; -Dv3.runAll=true also runs not-yet-enabled scenarios
gradlew perf checkJava8         # ms per frame (v3 vs v2 settings); Java 8 check
cd ../_investigation
./run_v3_closed_loop.sh         # v2 (202e70f) vs v3, 4 regimes, ~10 min on 20 threads
py -X utf8 summarize_v3.py > results/v3/summary.md
py -X utf8 pdf_source/build_guide.py           # README.md + template -> ../LineFollowerPipeline_Integration_Guide.pdf
py -X utf8 pdf_source/check_docs.py ../LineFollowerLibrary/src/main/java/com/example/linefollower/LineFollowerPipeline.java ../README.md <old.pdf> ../LineFollowerPipeline_Integration_Guide.pdf
```
