"""Text blocks for CHANGES_v3.md (filled by fill_changes_v3.py)."""

APPROACH = r'''Everything is in `LineFollowerPipeline.java` (single file, Java 8, no dependencies, no Android
APIs); the v3 code is the section "v3 PATH TRACKING".

**Colour model — line / floor / unreliable.** From calibration each class keeps its mean colour
(Y, U, V) and covariance. A pixel is compared with each class along a *ray* `g · mean` (a
brightness change of line and floor alike — shadow, uneven light) with `g` limited to the local
light level ×/÷ `lightLocalRange`; the local level is measured from the floor-like samples of the
same profile (median) or, on full-row scans, by a sliding mean. Samples that also match the line
colour are never used to measure the floor's light (with brightness only, a dark line would
otherwise be "floor in deep shadow"). Mahalanobis distances dL, dF: **line** if dL ≤ σ² and the
line fits clearly better (dF − dL > 6, i.e. ≈ 20 : 1), **floor** likewise, otherwise
**unreliable** — glare, a colour patch, an object, a blurred mix, anything that is neither.
A line-ness value (linear between the two colours) places edges with sub-sample precision.

**Calibration.** (1) Colours: a tall region above the robot is sampled; k-means 2…4; floor = the
cluster dominating both edges (or, when a shadow splits it, the largest share of the edges); line
= the cluster forming a narrow band in ≥ half of 6 row blocks (blocks where a disturbance merges
are outvoted); other clusters → a second floor colour, a light change of floor/line, or a
disturbance (excluded). (2) Geometry, `calibFrames` frames: every 8th row from the robot to the
top is scanned (sliding light level), the line is followed upward (prediction + slope); rows with
**clean edges on both sides** give centre and width (a single mixed edge sample counts as a
blurred edge). ROI bottom = just above `bottomIgnoreFrac` (raised past non-floor rows); ROI top =
the highest good row with mostly bad rows above it, minus `roiTopMarginFrac` of the visible
distance. Width model w(v) = A + B·v (robust IRLS) → camera tilt tan T = w(cy) / (f · B) (used if
consistent and within 25° of the configured tilt); line width on the floor; robust line fit of the
centres on the floor → straight-ahead offset and heading zero (|ψ₀| ≤ `steerHeadingCalibMaxDeg`).
The calibrated straight line (plus the part under the robot) is the initial path memory.

**Tracing.** The path is a chain of points `pathStep` apart in the robot frame (x forward, y right,
camera heights). Per frame: (1) the remembered chain is moved by the predicted motion; (2) every
remembered point inside the ROI is measured with a profile across it (`measureAt`: ≤ 72 samples,
~10 per line width, 2×2-averaged pixels, read directly from the Y/U/V or ARGB arrays — no
image-sized buffer is touched) — near → far, with the residual and its slope propagated to the next
point; (3) with `TRACE_BOTH` the gaps between two measured points are re-read with a prediction
interpolated from both sides; (4) the motion correction is fitted (2 DOF: travel along x and
rotation about the axle, weighted least squares on the normal residuals, Huber, weak prior);
(5) every point gets its final position: measured, or remembered + rigid correction + the leftover
residual interpolated from its measured neighbours; (6) the path is extended beyond its far end
(and down to the ROI bottom) into new territory: predict by direction + decaying curvature,
measure, bridge unreadable stretches (≤ `pathMaxGap`), on clean floor try a sharp-corner fan
(±25…100°, never back toward the robot) and then a jump search along the curve; a trace that would
run onto itself is stopped; (7) the chain is re-sampled and a fold-back (> 150° within 3 steps) is
cut. Without memory (start, lost for `trackLostGraceFrames`, or the memory contradicted by clean
floor / by a full line-like run beside it) the view is searched: 8 rows across the full width,
near → far (far → near with `TRACE_TOP_DOWN`), best candidate with the line's width and clean
edges (preferring straight ahead / the side the line was lost to), a direction fan, then extension
both ways.

**Profile analysis** (one place for all measurements): runs of LINE samples (1-sample gaps
bridged). Both edges against floor and width 0.55…1.6× expected → measured (quality ≈ 1). One edge
against floor, the other unknown → the clean edge ± width/2 (0.6) — the "only one border visible +
known width" case. Too wide with an edge where the line's edge is predicted → that edge (0.55) —
the line merged with a blob, or the border of a large dark/bright region only if it is exactly
where the line edge should be. Both edges unknown → only a full-width run (0.35). Weaker evidence
must lie closer to the prediction (half the gate). Nothing acceptable: mostly unknown pixels or a
line-like run that does not fit → **unreliable**; clean floor (≥ 70 %) under the normal light level
→ **absent**; outside the image → out.

**Memory bookkeeping.** Measured → confidence restored, age 0. Remembered → confidence decays with
`pathMemoryFrames` (4× slower while the path is tracked). "Absent" before a measured section is a
gap (unreliable), only trailing absent points end the path. Remembered points on the smooth
continuation of the measured points (quadratic fit, ≤ 0.35 line widths) are confirmed. Path
confidence (for tracking): measured points, or points with measured path on both sides within
`pathMaxGap`, count 1; others their memory confidence.

**Smoothing / control.** Local weighted quadratics x(s), y(s) over ±3 points (tricube ×
confidence × Tukey re-weighting) → positions, tangents; curvature from the change of direction over
±3 points. Steering: pure pursuit from the axle (`pursuitPivotBehind` behind the camera point) to
the first point at `pursuitLookAhead` (shortened to L / (1 + L·κ/2) before sharp bends, ≥ 0.5),
κ = 2·y/d², steering = `steerPerCurvature` × κ × auto-scale, ±50; visibility guard (path leaving
the image side within 1.5 ROI-bottom distances raises the steering toward it, fast attack / slow
release). Tracking: min over the path ahead of the curvature-allowed value relaxed with distance,
the requested turn |κ|, the dead-end length ramp, confidence < 0.5 scaling; light smoothing;
rate limit `powerRiseRate` / `powerFallRate`. Lost logic: measured → normal; remembered path in
unreliable pixels or below the view (blind zone) → follow it (`lineHeld`); clean floor mid-image →
hold `trackHoldFrames` (keeps ramping down if the line's end was seen); otherwise ±50 toward the
side the robot was turning to / the path leads, tracking minimum; after `trackLostGraceFrames`
the whole view is searched.'''

DECISIONS = r'''| # | Decision | Why (measured where possible) |
|---|---|---|
| 1 | **Unreliable is a third class**, not a weak "line" or "floor" | The task's central requirement. A disturbance that fits neither colour model must not become a decision; making it neutral lets memory and continuity decide. Disturbance matrix: v2 mean POWER drop 12.4 → v3 3.1 (§6.2). |
| 2 | Colour match **relative to the local light level** | Shadows, uneven light and dim far rows change line and floor alike; an absolute colour test made a shadowed floor "unreliable" and a dimmed white tape "floor". The local level comes from floor-like samples only (line-like samples excluded — essential for grayscale). |
| 3 | **Floor coordinates** for tracing, fitting and memory ("top to bottom", reading b) | Measured: matrix pass 243 vs 159, closed-loop RMS 1.14 vs 2.82 cm, lost frames 25 vs 216 (§6.6). Only path points are converted; no warp. |
| 4 | **Trace near → far + re-read gaps from both sides** (`TRACE_BOTH`; reading a) | Measured: best matrix (max \|ΔSTEER\| 39 vs 63) and least weaving; far → near alone is slightly worse on disturbances (the far end is thin, dim and next to background), near → far alone lacks the two-sided gap re-read. With a path memory the order matters much less than without one. |
| 5 | **Path memory moved by 2-DOF ego-motion** (travel + rotation about the axle) | A 3-DOF fit mixed sideways shift and yaw when only far points were visible (glare near the robot) and bent the remembered near part by 10°+. A wheeled robot cannot slide sideways; the 2-DOF model fixed it. A turn-rate prior (speed × path curvature at the axle, only on clear curves) carries the memory through blind stretches. |
| 6 | Along-track motion on a straight line is unobservable → memory confirmed by **continuity**, not by age | Remembered points under a glare that moves with the robot never "slide out"; with age-based confidence the power dropped although the path was certain. |
| 7 | **"Absent" only if no measured path follows** + **normal light** | A black line whitened by glare on a white floor looks like bare floor; the line visible beyond it proves a gap. Deep shadow / glare change the light level → never "absent". |
| 8 | **Pure pursuit** with adaptive look-ahead, `steerPerCurvature` = 30 per 1/h | The task asked for it; in steady state on a circle it has zero error when the scale matches the robot. 30 ≈ c = 1 on the simulated robot (wheelbase 0.65 h). Adaptive shortening before sharp bends follows a 4 cm-radius corner instead of cutting it. |
| 9 | **Automatic steering scale** (`steerAutoGain`) | Pure pursuit relies on the robot turning as much as asked; the library cannot know the program gain c or the robot. While following a curve steadily the robot turns as much as the path curves, so path curvature / requested curvature measures it. Helps c = 0.5 (§6.3). |
| 10 | **Visibility guard** (fast attack, slow release) | At low gain (c = 0.5) a tight curve leaves the image side before the robot turns enough; v2 survived those because of its ±50 pin. The guard raises steering only when the path exits the side close in front (never in normal following); a plain on/off version made the robot weave. |
| 11 | **Tracking**: curvature relaxed with distance + requested turn + dead-end length; smoothed and rate-limited | The task asks for smooth power without collapses. A corner 45 cm ahead must not slow the robot to the minimum yet; a robot at an angle must slow even on a straight; rate limits remove stop-and-go. |
| 12 | Memory is replaced by a fresh search **only on evidence against it** | A fresh search whenever "nothing confirmed" switched to a blob outside a curve at c = 0.5 (`ideal` 172 → 149 completed in an intermediate version). Evidence = clean floor where the memory says the line is, or a full line-like run just beside it (the line moved). |
| 13 | ROI found at calibration, **full width**, robot body from `bottomIgnoreFrac` | The task. The top keeps background (floor, chairs beyond a table edge) out; the full width is needed in curves. |
| 14 | Camera tilt **learned from the width per row**, within ±25° of the configured value | The real tilt is uncertain (CHANGES_v2 §9: probably 25–35°, configured 45°). Synthetic flat test images (constant width) would otherwise read as "straight down". `tilt35` regime: §6.3. |
| 15 | Before calibration the v2 pipeline answers | Keeps the documented "processFrame before calibration" behaviour (grayscale + `trackIsBright`). |
| 16 | v2 pipeline untouched behind `pathTracking = false` | Comparison; bit-exact (§6.1). |'''

TESTS = r'''`gradlew test`: **45 / 45 pass** (`LineFollowerPipelineTest` 20, `TrackSimTest` 6, `PathTrackingTest` 19).

**Existing tests (26)** — run against v3 before any change: most failed, all for one of these
reasons (the expectations were written for the ⊥ pipeline):

| Test(s) | Why it failed with v3 | Change |
|---|---|---|
| 7 colour/format tests (`whiteOnBlack_allFormats`, `blackOnBlue_…`, `whiteOnBrown_…`, `darkOnWhite_…`, `redOnGreen_…colourInput`, `survivesLightingChange…`, `classicGrayscaleMode…`) | They expect exactly 19 (v2 offset part) + a heading part for a line 60 px to the side. v3 steers by pure pursuit: ~8–9 for the same image — gentler on a pure offset (the zig-zag fix). | Check: line found, steering toward the line (3 … 50), offset part and `pathOffset` on the line's side, steering = offset + heading parts. |
| `v1SteeringIsOffsetOnly`, `v1LostLineDropsTrackingAtOnce` | They test the v1/v2 pipeline. | `v1Behaviour()` now also sets `pathTracking = false` (the switch exists for exactly this). |
| `hardTurnKeepsTrackFound` | Tracking is rate-limited in v3 (the "no jumps" requirement): 92 → 84 → 76 → … on the first frames of a sudden curve. | The turn is shown for 15 frames before tracking is checked (< 60). |
| `previouslyWorkingCombinationsUnchanged`, `darkLinesWithShadowBesideTheLine` (via `assertSteers`) | Exactly ±8 for a 2 cm offset (v2 gain); and a 2 cm jump between two frames. | ±(3 … 15) and symmetric; each pose held for 3 frames (a robot cannot jump 2 cm in one frame and v3 follows a memory). |
| `lostLineReadsExactlyFullSteering` | v3 follows the remembered path while the line is only out of view in front of the robot (`lineHeld`). | Frames with `trackFound == false` **and** `lineHeld == false` must read exactly ±50. |
| `steeringReturnsToZeroAfterTurns`, `wrongLockAfterTurnIsCorrected` (via `assertEndsCentred`) | In the scripted 4 cm corner cut (2 cm per frame, pose not driven by the steering) v3 is back on the line 3 frames later than v2 (memory-held frames). | The last 5 instead of 8 frames are checked. |
| `wideLineIsStillTheLine`, `uniformFloorNeverCalibrates`, `redOnGreen…grayscaleCannotCalibrate`, `lostLineReportsTrackLost`, `trackingRampsDownBeforeDeadEnd` | Real v3 bugs found by them (wide band rejected, issue code order, grayscale light estimate, hold after a mid-image loss, dead-end ramp ending in a jump) | **Fixed in the library**, tests unchanged. |

**New tests** (`PathTrackingTest`, one per scenario, `TrackSim` extended with render-time
disturbances): straight; gentle and sharp curves left / right; S-curve; line leaving the side
(±50 toward it); start 3 cm off / 15° turned (steers back); table edge with background at the top
(ROI stops below it); a tape-coloured blob and a bright stripe beside the line (unchanged
STEER/POWER); the robot's own shadow beside and over the line, 4 colour pairs; glare (the real
example) on a straight for 4 colour pairs and on a curve; glare during calibration; a disturbance
matrix of 4 colour pairs × 6 types (room shadow, uneven light, colour patch, covering object,
colour cast, full glare); disturbances during calibration; the new `FrameData` fields
(unreliable grid, memory points, sources, confidence); and the option switches
(`TRACE_BOTTOM_UP`, `TRACE_TOP_DOWN`, `pathFloorSpace = false`, `pathMemory = false`,
`steerAutoGain = false`, `pathTracking = false`). Disturbance tests require the task's criteria:
max |ΔSTEER| ≤ 3, mean ≤ 1, max POWER drop ≤ 10, mean ≤ 3.'''
