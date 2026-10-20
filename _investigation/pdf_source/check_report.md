### Config fields (code vs README vs PDF)

| Field | Type (code) | Default (code) | README row: type / default | README | PDF (new) | PDF (old) |
|---|---|---|---|---|---|---|
| `autoColor` | boolean | `true` | ✅ boolean / true | ✅ | ✅ | yes |
| `trackIsBright` | boolean | `true` | ✅ boolean / true | ✅ | ✅ | yes |
| `colorCalibFrames` | int | `5` | ✅ int / 5 | ✅ | ✅ | yes |
| `colorMinSeparation` | float | `4.0f` | ✅ float / 4.0f | ✅ | ✅ | yes |
| `colorMinDistance` | float | `12f` | ✅ float / 12f | ✅ | ✅ | yes |
| `thresholdMargin` | int | `0` | ✅ int / 0 | ✅ | ✅ | yes |
| `minContrast` | int | `24` | ✅ int / 24 | ✅ | ✅ | yes |
| `colorMinThreshold` | int | `112` | ✅ int / 112 | ✅ | ✅ | yes |
| `vRoiAdaptiveThreshold` | boolean | `true` | ✅ boolean / true | ✅ | ✅ | yes |
| `hRoiCenterYFrac` | float | `0.85f` | ✅ float / 0.85f | ✅ | ✅ | yes |
| `hRoiThickPx` | int | `24` | ✅ int / 24 | ✅ | ✅ | yes |
| `bottomIgnoreFrac` | float | `0.15f` | ✅ float / 0.15f | ✅ | ✅ | yes |
| `vRoiHalfWidthPx` | int | `80` | ✅ int / 80 | ✅ | ✅ | yes |
| `vRoiHeightFrac` | float | `0.5f` | ✅ float / 0.5f | ✅ | ✅ | yes |
| `minWeightFrac` | float | `0.01f` | ✅ float / 0.01f | ✅ | ✅ | yes |
| `curveFullScaleDeg` | float | `30f` | ✅ float / 30f | ✅ | ✅ | **no — added** |
| `cameraTiltDeg` | float | `45f` | ✅ float / 45f | ✅ | ✅ | yes |
| `verticalFovDeg` | float | `60f` | ✅ float / 60f | ✅ | ✅ | yes |
| `trackMemory` | boolean | `true` | ✅ boolean / true | ✅ | ✅ | yes |
| `trackMemoryGateMult` | float | `3.0f` | ✅ float / 3.0f | ✅ | ✅ | yes |
| `trackMemoryMinGatePx` | int | `48` | ✅ int / 48 | ✅ | ✅ | yes |
| `trackLostGraceFrames` | int | `8` | ✅ int / 8 | ✅ | ✅ | yes |
| `trackRelockFrames` | int | `6` | ✅ int / 6 | ✅ | ✅ | yes |
| `rejectDistractors` | boolean | `true` | ✅ boolean / true | ✅ | ✅ | **no — added** |
| `bandMaxWidthMult` | float | `2.0f` | ✅ float / 2.0f | ✅ | ✅ | **no — added** |
| `bandMinWidthMult` | float | `0.3f` | ✅ float / 0.3f | ✅ | ✅ | **no — added** |
| `trackHoldFrames` | int | `6` | ✅ int / 6 | ✅ | ✅ | **no — added** |
| `steerOffsetWeight` | float | `1.0f` | ✅ float / 1.0f | ✅ | ✅ | **no — added** |
| `steerHeadingWeight` | float | `1.0f` | ✅ float / 1.0f | ✅ | ✅ | **no — added** |
| `steerLookAhead` | float | `0.5f` | ✅ float / 0.5f | ✅ | ✅ | **no — added** |
| `steerHeadingMinRows` | int | `24` | ✅ int / 24 | ✅ | ✅ | **no — added** |
| `steerHeadingCalibMaxDeg` | float | `10f` | ✅ float / 10f | ✅ | ✅ | **no — added** |
| `trackOffsetFullFrac` | float | `1.0f` | ✅ float / 1.0f | ✅ | ✅ | yes |
| `trackLengthSlowdown` | boolean | `true` | ✅ boolean / true | ✅ | ✅ | yes |
| `trackLengthFullFrac` | float | `0.8f` | ✅ float / 0.8f | ✅ | ✅ | yes |
| `trackingMinValue` | int | `5` | ✅ int / 5 | ✅ | ✅ | yes |
| `calibFrames` | int | `10` | ✅ int / 10 | ✅ | ✅ | yes |

37 public Config fields.

### FrameData fields (README vs PDF)

| Field | README | PDF (new) | PDF (old) |
|---|---|---|---|
| `imgW` | ✅ | ✅ | yes |
| `imgH` | ✅ | ✅ | yes |
| `hRoiTopFrac` | ✅ | ✅ | yes |
| `hRoiBotFrac` | ✅ | ✅ | yes |
| `vRoiTopFrac` | ✅ | ✅ | yes |
| `ignoreTopFrac` | ✅ | ✅ | yes |
| `steerRefXFrac` | ✅ | ✅ | yes |
| `trackRefXFrac` | ✅ | ✅ | yes |
| `halfBotFrac` | ✅ | ✅ | yes |
| `halfTopFrac` | ✅ | ✅ | yes |
| `centroidXFrac` | ✅ | ✅ | yes |
| `trackWidthFrac` | ✅ | ✅ | yes |
| `tiltDeg` | ✅ | ✅ | yes |
| `lookAheadColFrac` | ✅ | ✅ | yes |
| `lookAheadRowFrac` | ✅ | ✅ | yes |
| `lookAheadPointCount` | ✅ | ✅ | yes |
| `trackFound` | ✅ | ✅ | yes |
| `isCalibrated` | ✅ | ✅ | yes |
| `calibPhase` | ✅ | ✅ | yes |
| `calibProgress` | ✅ | ✅ | yes |
| `calibIssue` | ✅ | ✅ | yes |
| `colorModelReady` | ✅ | ✅ | yes |
| `lineColorArgb` | ✅ | ✅ | yes |
| `floorColorArgb` | ✅ | ✅ | yes |
| `colorSeparation` | ✅ | ✅ | yes |
| `threshold` | ✅ | ✅ | yes |
| `steering` | ✅ | ✅ | yes |
| `tracking` | ✅ | ✅ | yes |
| `steerOffsetTerm` | ✅ | ✅ | **no — added** |
| `steerHeadingTerm` | ✅ | ✅ | **no — added** |
| `headingDeg` | ✅ | ✅ | **no — added** |
| `headingConfidence` | ✅ | ✅ | **no — added** |
| `lineHeld` | ✅ | ✅ | **no — added** |
| `rejectedBands` | ✅ | ✅ | **no — added** |

34 public FrameData fields.

### README examples in the PDF

22 README `java` blocks; all present in the new PDF (first and last line of each found).

### Outdated values

| Pattern | README | PDF (new) | PDF (old) |
|---|---|---|---|
| -100 … +100 steering | 0 | 0 | 0 |
| tracking minimum 2 ("2 … 100") | 0 | 0 | 0 |
| 1280, 720 frame size | 0 | 0 | 2 |
| data.someXFrac placeholder | 0 | 0 | 1 |
| STEER bar −100…+100 | 0 | 0 | 0 |

RESULT: OK — no mismatches
