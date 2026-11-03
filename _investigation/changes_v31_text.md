## v3.1 — steady outputs under real glare

*Added after the first test on the real robot. Sections 1–12 below describe v3 as first
delivered ("v3 before" in the tables here); the numbers in this section are the final ones.*

### The report

On the real robot, with glare from the room light on the path, STEER and POWER did not stay
steady: they kept jumping between several values. This was given top priority over every
simulated case.

### Reproducing it

The first simulator drew glare as a smooth, steady whitening spot — v3 passed it, which is why
the problem was not seen. Real glare is different, and the harness now has it
(`Disturb` type `glareReal`, also `TrackSim.realGlare` for the unit tests):

- the core **clips** to white (flat 255, no colour), with a soft halo;
- the surface (varnish, wood grain, tape) breaks it into **streaks** that are fixed on the floor,
  so they change as the robot moves;
- it **flickers** every frame (mains light against the exposure, auto-exposure hunting);
- the tape is glossier than the floor;
- the spot moves with the camera (a lamp's reflection), so it is present **during calibration** too;
- plus camera **vibration** (rows shift ±2 px), whole-frame exposure flicker (±3 %) and the phone's
  **30 fps** (the closed loop ran at 60 fps before; `sim.fps`, `sim.vib`).

New tools: `Jitter` (scripted drive at 30 fps, each case run with and without glare, frame-to-frame
changes of STEER / POWER compared), `CalProbe` (calibration diagnostics row by row), steering jitter
in `ClosedLoop` (`steerJerk`), `run_suite.sh` / `cmp_suite.py` / `report_v31.py`.

### Causes (found by tracing frame by frame)

1. **The motion estimate drifted.** The path memory is carried from frame to frame by the robot's
   estimated motion. Each frame's correction was added to the speed estimate at 60 %, with nothing
   pulling it back. With glare hiding most of the line, the correction came from 3–5 noisy points:
   the forward speed drifted to 0.18 instead of 0.067 camera heights per frame, the turn rate to
   4°/frame instead of 1.5°, and once to 25°/frame (a curvature prior that was not limited). The
   remembered path bent and slid, the pursuit target jumped toward the robot, STEER jumped
   (5 → 27 → 18 → 5, or ±50 when the bent path left the image).
2. **Every measurement replaced the memory.** Where a glare edge flickers, a path point was measured
   in one frame (at a slightly different place) and not in the next — the path moved back and forth
   with the flicker.
3. **Calibration with the glare in view learned the glare as the line**: a bright band straight
   ahead is what the colour step looked for, and the tape — broken by the glare in several row
   blocks — lost to it (in the jitter set, 21 of 84 calibrations with glare failed or learned the
   glare). The ROI was also cut below the glare: rows that match neither colour ended it, as a table
   edge does.
4. A remembered path that had drifted into the glare was never checked against the line visible
   right next to it.
5. Whole-number outputs flipped between neighbours on a steady path; the "line leaves at the side"
   boost (visibility guard) was switched off from one frame to the next.

### Fixes — all generic (no glare- or tape-specific rule)

| | Fix |
|---|---|
| Motion | A **Kalman filter** on (forward travel, turn) per frame: the prior is the predicted motion with its uncertainty; each measured point adds information in proportion to its precision (measurement + memory variance), outliers are down-weighted (Huber). Many clean points correct it fully; a few noisy ones barely; motion along a straight line (invisible in the line) stays at the prediction. The positions are corrected in full every frame; the motion for the next frame changes at most as fast as a robot accelerates (≈ 2 m/s², 20 rad/s²). The curvature prior is used only while nothing is measured. |
| Memory | Every point carries a **variance**. A measured point is **fused** with its remembered position: the rigid part of the difference (the robot's motion) is applied in full; the rest — the shape of a line that does not change on the floor — is averaged (Kalman gain ≈ 0.4 for a clean point). If a point is far off, or its neighbours are off together in the same direction (the memory was wrong, e.g. a curve seen for the first time), the measurement is taken over at once — no lag on turns. |
| Calibration — line colour | The line is found as a **stripe**: rows are scanned with the floor colour (from the outer 30 % of the image on each side, robustly trimmed, at each row's own light level); every non-floor run with **sharp edges against the floor on both sides** is a candidate; the line is the longest **chain** of runs that continues row by row (about the previous width, where the line leads, of its colour; gaps allowed) — in the image, so it does not depend on the configured tilt. The line colour is the most common run colour in the chain (rows tinted by a disturbance change their tint from row to row and are left out). Glare, shadows and uneven light have soft edges; blobs do not keep one width. If no stripe is found, the cluster-based choice of v3 (now: band in the most of *all* row blocks) is the fallback. |
| Calibration — colours under a disturbance near the robot | The colour samples come from the region near the robot. When a glare is there: the **floor** model is trimmed at 3σ (if ≥ 3 % of the floor cluster are outliers — a glare halo, lighter and greyer floor — they are removed, else the plain cluster model is kept); the **line** colour is taken from the stripe rows with the **highest contrast** against the floor beside them (glare, haze and halos always lower it), and the stripe's own pixels join the line cluster when the samples hold too few line pixels. Without this the black tape fitted both colours (= unreliable) after calibrating in glare, or the line was learned too light. |
| Calibration — ROI | The ROI top is the highest row where the **floor is there** (≥ 20 % floor-coloured samples) and the line was seen with a second line row below; a glare or a shadow over part of the rows in between does not end it, a table edge (no floor beyond) still does. |
| Unconfirmed memory | A remembered path that no measurement confirms for 3 frames gives way to a line found by a fresh search (≥ 5 measured points, running within 45° of the memory). |
| Outputs | Whole numbers with hysteresis 0.8 (steering and power); the visibility guard is released smoothly. (A hysteresis of 4 on the power *target* was tried and removed: it left small lasting offsets.) |
| Steering scale | `steerAutoGain` learns from the steering **actually sent** (including held frames, the guard and short full turns) — before, it saw only the weak commands and lowered the gain of an under-steering robot. |
| Light | The range of plausible floor light levels follows the room light slowly (×0.5 … ×2.5 around the calibration level); a profile with no floor in that range but a floor-coloured majority at one consistent level takes its light level from that majority (the line is always the minority of a profile across it). |

Kept unchanged on purpose (measured worse): letting the light reference itself follow the room
light (broke neutral colour pairs — a black line passed for white floor in deep shadow:
84 → 69 of 100 matrix cases); a faster steering-scale adaptation (no gain); a one-sided seed for
the fresh search (created a wrong lock on a glare edge); a stripe-only calibration without fallback
(failed legitimate synthetic cases).
