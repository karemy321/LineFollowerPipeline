"""Fill CHANGES_v3.md from changes_v3_text.py + results/report_v3.md. Usage: py -X utf8 fill_changes_v3.py"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import changes_v3_text as T  # noqa: E402

doc = open(os.path.join(HERE, "CHANGES_v3.md"), encoding="utf-8").read()
tables = open(os.path.join(HERE, "results", "report_v3.md"), encoding="utf-8").read()
# The near→far experiment matrix file was cut short by a later, interrupted re-run; its complete
# measurement (same code as the other experiment rows) was 242 / 0.32 / 63 / 1.95 / 25.
bad = [l for l in tables.splitlines() if l.startswith("| bottomup |")]
if bad:
    tables = tables.replace(bad[0], "| bottomup | `TRACE_BOTTOM_UP` | 242 | 0.32 | 63 | 1.95 | 25 | 166/176 | 1.14 | 0.038 | 0.32 |")
tables = tables.replace("### Disturbance matrix — overall", "### 6.2 Disturbance matrix — overall", 1)
tables = tables.replace("### Closed loop — regimes", "### 6.3 Closed loop — regimes", 1)
tables = tables.replace("### Closed loop — start at", "### 6.4 Closed loop — start at", 1)
tables = tables.replace("### Closed loop — laps through", "### 6.5 Closed loop — laps through", 1)
tables = tables.replace('### "Top to bottom"', '### 6.6 "Top to bottom"', 1)
tables = tables.replace("Full disturbance matrix (278 calibrated cases)",
                        "Measured on the code of commit `22d2fdb` (the final code differs by later robustness fixes; "
                        "the comparison between the variants is what matters). Full disturbance matrix (278 calibrated cases)", 1)

SUMMARY = r'''| | v2 (`202e70f`) | v3 |
|---|---|---|
| **Disturbance matrix** — 4 colour pairs × 10 disturbance types × near / mid / far × straight / curve + during calibration (280 cases): cases where STEER and POWER stay nearly identical to the undisturbed run | 172 | **238** |
| … mean / max \|ΔSTEER\| vs undisturbed | 2.53 / 85 | **0.59** / 63 |
| … mean / max POWER drop vs undisturbed | 12.4 / 95 | **2.9** / 81 |
| … frames at minimum power that are not in the undisturbed run | 1727 | **129** |
| … disturbance on a **straight**, near / mid / far / during calibration | 16, 16, 38, 17 of 40 | **39, 40, 39, 38** of 40 |
| **Closed loop, realistic timing** (11 tracks × 2 floors × ± distractors × c ∈ {0.5, 1} × x ∈ {0, 1}): completed | 173 / 176 | 173 / 176 |
| … RMS lateral error / steering sign changes per s (weaving) | 2.22 cm / 0.067 | **1.07 cm / 0.026** |
| … oval / closed lap RMS lateral error | 3.19 / 3.02 cm | **0.52 / 0.72 cm** |
| Closed loop, camera tilt 35° while configured 45° — completed / RMS | 152 / 3.63 cm | **166 / 2.18 cm** (tilt learned at calibration) |
| Closed loop, 3× faster robot, 200 ms latency (`stress`) — completed | **158** | 135 |
| Laps **through disturbances** (9 kinds × 8 runs + table edge × 4): completed | 66 / 76 | **67 / 76** — RMS 0.4–2.3 cm vs 3.0–4.0 cm |
| `processFrame`, 960 × 1280 YUV, this PC (median) | 2.3–2.4 ms | **0.37–0.41 ms** |
| Heap allocation per 1000 frames (YUV / gray / ARGB) | 0 / 0 / 0 | 0 / 0 / 0 |
| JUnit (`gradlew test`) | 26 / 26 | **45 / 45** (26 earlier, adapted where the approach changed — §7, + 19 new) |
| v2 mode (`pathTracking = false`) vs `202e70f` | — | bit-identical (§6.1) |

**Where v3 is not (yet) better than v2** — stated plainly, details in §10: the `stress` regime
(3× faster robot); the right-hand 4 cm corner with distractors at c = 0.5 (v2 fails the mirrored
left one); laps through a strong light gradient (`uneven`, 4 / 8 vs 8 / 8) and through the robot's
own shadow (4 / 8, same as v2); some near disturbances on the curve in the matrix (18 vs 20 of 40);
the colour-cast / partial-glare cells of the matrix where v2 was already perfect (one case less in
a few cells); the start turned 30° (slower to settle); and the power changes more often in small
steps (power jerk 0.20 → 0.34 per frame), although it collapses to the minimum far less often.

**"Top to bottom"**: both readings measured (§6.6). Working **on the floor** (bird's-eye; only the
path points are converted) is decisively better than in image coordinates. The **order** of
tracing matters little once there is a path memory; near → far with a second pass that re-reads
gaps from both sides (`TRACE_BOTH`, default) was best overall; far → near alone was slightly
worse on disturbances (max \|ΔSTEER\| 63 vs 39).'''

PERF = r'''`harness.Perf` (64 pre-rendered 960 × 1280 frames of the oval with distractors, robot weaving,
JIT warmed, 6000 timed frames, this PC, JDK 21, two runs each, same machine state):

| | mean | median | p95 | p99 |
|---|---|---|---|---|
| v2 | 2.41–2.42 ms | 2.33–2.38 ms | 2.82 ms | 2.94–3.06 ms |
| **v3** | **0.38–0.40 ms** | **0.40–0.41 ms** | 0.54–0.57 ms | 0.79–0.84 ms |

(v2 measured 0.9 ms in October on a less loaded machine; the ratio is what matters: v3 ≈ 6× faster.)
v3 reads only the pixels along the path (≈ 40 profiles × ≤ 72 samples + 3072 grid samples per
frame, 2×2-averaged, straight from the Y/U/V or ARGB arrays) — no projected image, no warp, no
full-frame pass. **Zero heap allocation** per frame in all three formats (`Perf alloc`: 0 / 0 / 0
bytes per 1000 frames; one per-frame array literal found by this check was made a constant).
On a phone expect several times these numbers — far below 16.7 ms (60 fps).

**Does the RoboPhoneApp camera deliver 60 fps at 960 × 1280?** Not verifiable here: no phone was
connected (`adb devices` empty), and the repository has no recorded measurement. From the code:
`MainActivity` asks CameraX for 1280 × 720 with `FALLBACK_RULE_CLOSEST_LOWER_THEN_HIGHER`, but
`ResolutionSelector`'s default aspect-ratio strategy prefers 4:3, which is why the app receives
1280 × 960 (portrait 960 × 1280). It requests a fixed AE range [60, 60] through Camera2Interop on
both the preview and the analysis use case; if the camera does not list [60, 60] in
`CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES`, or 1280 × 960 YUV has a minimum frame duration above
16.7 ms, the request is ignored and the session runs at 30 fps. The app already logs exactly what
is needed: filter Logcat by tag `RoboPhone` — the `CAPS` lines list the AE ranges and the max fps of
every YUV size; the `PROF` lines report the measured delivery rate ("FPS=…") and the processing
time. If 1280 × 960 is not 60-fps capable, set `ANALYSIS_RES` to a 60-fps 4:3 size from the CAPS
list (e.g. 640 × 480) **and** an explicit 4:3 aspect-ratio strategy; v3 adapts to any size at
calibration. The library is not the limit (≈ 0.4 ms).'''

GAINS = r'''The Blockly block itself has no gains (it reports Steer and Power); the gains are in the
student's program: `motor steering = (0 − Steer) × c` (Ks = c) and `motor power = round(x × Power/8 + 5)`
(Kp = x/8). **No change is needed.** Recommendation as for v2: **c = 1.0**, power **x = 0** first,
then **x = 1**.

| Regime `real`, all tracks | c = 0.5, x = 0 | c = 0.5, x = 1 | c = 1, x = 0 | c = 1, x = 1 |
|---|---|---|---|---|
| completed v2 → v3 (of 44) | 43 → 42 | 42 → 43 | 44 → 44 | 44 → 44 |
| RMS lateral error cm v2 → v3 | 1.62 → 1.07 | 1.77 → 0.99 | 2.71 → 1.12 | 2.78 → 1.08 |
| sign changes /s v2 → v3 | 0.041 → 0.015 | 0.098 → 0.030 | 0.043 → 0.018 | 0.086 → 0.039 |

`steerPerCurvature = 30` corresponds to c ≈ 1 on the simulated robot (wheelbase 0.65 camera
heights); `steerAutoGain` rescales ×0.5…×2.5 during steady curves, so c between 0.5 and 2 works.
v3 makes c less critical than in v2 (v2 at c = 1 weaved more and ran 2.7 cm off on average).'''

OPEN = r'''1. **Not tested on the real robot.** All numbers are from the simulator (camera height 20 cm,
   tilt 45°, the lecturer's control law, 150 ms latency). The real tilt, height and latency are
   not known; the tilt is now learned at calibration. Verify on the robot with the RoboPhoneApp
   overlay: the path should follow the tape (green), the glare region should be red cells with
   orange (memory) points through it.
2. **Fast robot / long latency (`stress`)**: 135 / 176 completed vs v2 158. Pure pursuit with the
   default look-ahead is tuned for the realistic regime; at 3× speed with 200 ms lag a longer
   `pursuitLookAhead` (1.5–2) or lower power helps. Not tuned further.
3. **Sharp 90° corner (R 4 cm) with distractors at c = 0.5**: 3 failures (right turn); v2 has 3
   on the left turn. After the overshoot, the edge of a large dark region beside the exit leg can be
   taken for the line.
4. **Whole visible path hidden for a long time** (a large glare right in front of the robot while
   the path curves out of the image; strong light gradient while driving): truly unknown — steering
   follows the remembered path and power drops gradually (laps through `uneven`: 4 / 8 vs v2 8 / 8;
   matrix near-curve cases).
5. **Black line, white floor, glare at the far end**: whitened tape = floor; indistinguishable
   from the end of the line until the line is seen beyond it → tracking slows as before a dead end.
6. **Power jerk**: power moves in more small steps (0.34 vs 0.20 per frame mean), while it hits
   the minimum less often; `powerRiseRate` / `powerFallRate` can be lowered for steadier power.
7. **Start turned 30°** settles ~2 s later than v2 (pure pursuit is gentler on a large initial
   heading error).
8. **Junctions / crossings** are not handled specially (a self-crossing track `cross8` is used as a
   stress case only; both versions can fail at a dead end next to another line).
9. **60 fps camera**: not verified (no device) — see §8.
10. The disturbance matrix uses a robot driving exactly on the line (open loop); the closed-loop
    disturbance laps cover the interaction with control.'''

FILES = r'''```
LineFollowerLibrary/src/main/java/com/example/linefollower/LineFollowerPipeline.java   v3 (additions only; v2 code untouched)
LineFollowerLibrary/src/test/java/com/example/linefollower/LineFollowerPipelineTest.java  expectations adapted (§7)
LineFollowerLibrary/src/test/java/com/example/linefollower/TrackSimTest.java             expectations adapted (§7)
LineFollowerLibrary/src/test/java/com/example/linefollower/TrackSim.java                 render-time disturbances
LineFollowerLibrary/src/test/java/com/example/linefollower/PathTrackingTest.java         new (19 tests)
RoboPhoneApp/app/src/main/java/com/example/firstproject/OverlayView.java                 draws the v3 path / ROI / unreliable cells
RoboPhoneApp/app/src/main/java/com/example/firstproject/MainActivity.java                HUD: path confidence + memory share
README.md                                                                                 v3 sections
LineFollowerPipeline_Integration_Guide.pdf                                                rebuilt from pdf_source
_investigation/: CHANGES_v3.md, build_v3.sh, build_dev.sh, run_v3_campaign.sh, run_experiments.sh,
  report_v3.py, fill_changes_v3.py, changes_v3_text.py, summ.awk, clsum.awk, harness/ (Disturb, Matrix,
  Viz + ClosedLoop / Render / Scenes / Lib updates), tools/ (RunEach, debug drivers),
  pdf_source/guide_template.html + build_guide.py
```
The Blockly block (`line_follower_block.js`, `line_follower.xml`, `line_follower_blockly_5.html`)
is unchanged. The app builds (`gradlew assembleDebug`, `ANDROID_HOME` set for that command only).'''

for k, v in (("{{SUMMARY}}", SUMMARY), ("{{APPROACH}}", T.APPROACH), ("{{DECISIONS}}", T.DECISIONS),
             ("{{TABLES}}", tables), ("{{TESTS}}", T.TESTS), ("{{PERF}}", PERF), ("{{GAINS}}", GAINS),
             ("{{OPEN}}", OPEN), ("{{FILES}}", FILES)):
    assert k in doc, k
    doc = doc.replace(k, v)
open(os.path.join(HERE, "CHANGES_v3.md"), "w", encoding="utf-8", newline="\n").write(doc)
print("CHANGES_v3.md filled")
