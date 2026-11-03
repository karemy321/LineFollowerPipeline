### v3.1 against v2 — final numbers (success rate = completed / measured)

Matrix, regimes, starts, disturbance laps, jitter and the clean-calibration glare laps were
measured on build `v3c3`; the final code differs from it only in calibration (the colour rules
under a disturbance near the robot) and in having no power-target hysteresis — the glare laps
calibrated with the glare in view were measured on the final code.

| | v2 | v3 before | **v3.1** |
|---|---|---|---|
| **STEER change per frame with realistic glare** (no-glare value in brackets; scripted drive, 30 fps, 84 cases, calibrated clean) | 2.37 (1.65) | 0.82 (0.75) | **0.59 (0.49)** |
| … the same, calibrated with the glare in view | 2.26 (1.65) | 1.01 (0.75), 21 of 84 calibrations failed | **0.74 (0.49)**, 12 failed |
| **POWER change per frame with glare** (no-glare) | 5.49 (2.27) | 0.74 (0.49) | **0.63 (0.40)** |
| STEER jumps ≥ 5 in 84 glare drives (no-glare drives) | 1254 (546) | 98 (28) | **76 (28)** |
| Glare laps at 30 fps, spot mid-view, calibrated with glare | 32/32 (100 %) | – | **32/32 (100 %)** |
| … spot near the robot, calibrated with glare | 27/32 (84 %) | – | **29/32 (91 %)** |
| … spot near, calibrated clean | 27/32 (84 %) | 29/32 | **32/32 (100 %)** |
| … very large spot (hides most of the line), calibrated clean | **25/32 (78 %)** | 17/32 | 15/32 (47 %) |
| … lamp-tube streak lying along the whole line, calibrated clean | **25/32 (78 %)** | 24/32 | 21/32 (66 %) |
| **Disturbance matrix** (280 cases) | 172 (61 %) | 238 (85 %) | **250 (89 %)** |
| … disturbance near the robot on a curve (40) | 20 | 18 | **22** |
| **Laps through disturbances** (88) | 78 (89 %) | 79 | **88 (100 %)** |
| … strong light gradient (8) | 8 | 4 | **8** |
| Closed loop, realistic timing (176) | **173 (98 %)** | 173 | 168 (95 %) |
| Closed loop, phone 30 fps + vibration (176) | **174 (99 %)** | – | 170 (97 %) |
| Closed loop, camera tilt off by 10° (176) | 152 (86 %) | **166 (94 %)** | 157 (89 %) |
| Closed loop, 3× faster robot, 200 ms delay (176) | **158 (90 %)** | 135 (77 %) | 141 (80 %) |
| Starts turned / shifted (80) | 80 | 80 | **80 (100 %)** — 30° start settles ≈ 2 s later than v2 |
| RMS lateral error, realistic timing | 2.22 cm | 1.07 cm | **1.26 cm** |
| POWER steps per frame, realistic timing | **0.20** | 0.34 | 0.31 |
| `processFrame` 960 × 1280 YUV (median) | 1.0 ms | 0.4 ms | **0.20 ms**, no allocation |
| JUnit | 26/26 | 45/45 | **47/48** (`glareDuringCalibration`: mean POWER drop 3.3, limit 3.0) |

### Still a problem (open)

- **Gain 0.5 with distractors** beside the line: realistic timing 36/44 (v2 41), camera tilt off by
  10° 30/44 (v2 44). The cause is the motion filter that makes the outputs steady under glare; at
  gain 1.0 (recommended, what the robot uses) all 88 runs complete. Trade-off kept on purpose.
- **Glare that hides almost the whole line** (very large spot, a lamp-tube streak exactly along the
  line): v2 completes more laps (it jitters strongly but keeps a lock); v3.1 follows its memory and
  slows down, then may lose the line. Calibration in such glare fails (or, for black tape on wood,
  can learn the glare) — recalibrate a little further along.
- **Glare right in front of the robot on an S-curve**: 4–20 STEER jumps per drive (clean ≈ 1).
- 3× faster robot with long delay: 141 vs v2 158; POWER steps 0.31 vs v2 0.20 per frame;
  start turned 30° settles about 2 s later.
- The "top to bottom" experiment could not be re-run on the final code (the system stopped it for
  low memory twice, the PC had < 2 GB free); its table (§6.6) is from the earlier v3.
- Nothing is measured on the real robot yet beyond the report that started v3.1.
