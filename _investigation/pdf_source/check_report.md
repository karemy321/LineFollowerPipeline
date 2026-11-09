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
| `curveFullScaleDeg` | float | `30f` | ✅ float / 30f | ✅ | ✅ | yes |
| `cameraTiltDeg` | float | `45f` | ✅ float / 45f | ✅ | ✅ | yes |
| `verticalFovDeg` | float | `60f` | ✅ float / 60f | ✅ | ✅ | yes |
| `trackMemory` | boolean | `true` | ✅ boolean / true | ✅ | ✅ | yes |
| `trackMemoryGateMult` | float | `3.0f` | ✅ float / 3.0f | ✅ | ✅ | yes |
| `trackMemoryMinGatePx` | int | `48` | ✅ int / 48 | ✅ | ✅ | yes |
| `trackLostGraceFrames` | int | `8` | ✅ int / 8 | ✅ | ✅ | yes |
| `trackRelockFrames` | int | `6` | ✅ int / 6 | ✅ | ✅ | yes |
| `rejectDistractors` | boolean | `true` | ✅ boolean / true | ✅ | ✅ | yes |
| `bandMaxWidthMult` | float | `2.0f` | ✅ float / 2.0f | ✅ | ✅ | yes |
| `bandMinWidthMult` | float | `0.3f` | ✅ float / 0.3f | ✅ | ✅ | yes |
| `trackHoldFrames` | int | `6` | ✅ int / 6 | ✅ | ✅ | yes |
| `glareMask` | boolean | `true` | ✅ boolean / true | ✅ | ✅ | **no — added** |
| `glareLumaMax` | int | `245` | ✅ int / 245 | ✅ | ✅ | **no — added** |
| `glareCoastMs` | float | `600f` | ✅ float / 600f | ✅ | ✅ | **no — added** |
| `glareCoastMaxFrames` | int | `36` | ✅ int / 36 | ✅ | ✅ | **no — added** |
| `frameIntervalMs` | float | `0f` | ✅ float / 0f | ✅ | ✅ | **no — added** |
| `corridorMode` | boolean | `true` | ✅ boolean / true | ✅ | ✅ | **no — added** |
| `corridorSlices` | int | `12` | ✅ int / 12 | ✅ | ✅ | **no — added** |
| `corridorWidthMult` | float | `2.2f` | ✅ float / 2.2f | ✅ | ✅ | **no — added** |
| `minChainSlices` | int | `4` | ✅ int / 4 | ✅ | ✅ | **no — added** |
| `corridorMinConfidence` | float | `0.35f` | ✅ float / 0.35f | ✅ | ✅ | **no — added** |
| `corridorProcessNoise` | float | `1.0f` | ✅ float / 1.0f | ✅ | ✅ | **no — added** |
| `edgePairCheck` | boolean | `true` | ✅ boolean / true | ✅ | ✅ | **no — added** |
| `steerOffsetWeight` | float | `1.0f` | ✅ float / 1.0f | ✅ | ✅ | yes |
| `steerHeadingWeight` | float | `1.0f` | ✅ float / 1.0f | ✅ | ✅ | yes |
| `steerLookAhead` | float | `0.5f` | ✅ float / 0.5f | ✅ | ✅ | yes |
| `steerHeadingMinRows` | int | `24` | ✅ int / 24 | ✅ | ✅ | yes |
| `steerHeadingCalibMaxDeg` | float | `10f` | ✅ float / 10f | ✅ | ✅ | yes |
| `trackOffsetFullFrac` | float | `1.0f` | ✅ float / 1.0f | ✅ | ✅ | yes |
| `trackLengthSlowdown` | boolean | `true` | ✅ boolean / true | ✅ | ✅ | yes |
| `trackLengthFullFrac` | float | `0.8f` | ✅ float / 0.8f | ✅ | ✅ | yes |
| `trackingMinValue` | int | `5` | ✅ int / 5 | ✅ | ✅ | yes |
| `calibFrames` | int | `10` | ✅ int / 10 | ✅ | ✅ | yes |

49 public Config fields.

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
| `steerOffsetTerm` | ✅ | ✅ | yes |
| `steerHeadingTerm` | ✅ | ✅ | yes |
| `headingDeg` | ✅ | ✅ | yes |
| `headingConfidence` | ✅ | ✅ | yes |
| `lineHeld` | ✅ | ✅ | yes |
| `rejectedBands` | ✅ | ✅ | yes |
| `baseReconstructed` | ✅ | ✅ | **no — added** |
| `coasting` | ✅ | ✅ | **no — added** |
| `maskedFrac` | ✅ | ✅ | **no — added** |
| `corridorCoverage` | ✅ | ✅ | **no — added** |
| `corridorFitRms` | ✅ | ✅ | **no — added** |

39 public FrameData fields.

### README examples in the PDF

23 README `java` blocks; all present in the new PDF (first and last line of each found).

### Outdated values

| Pattern | README | PDF (new) | PDF (old) |
|---|---|---|---|
| -100 … +100 steering | 0 | 0 | 0 |
| tracking minimum 2 ("2 … 100") | 0 | 0 | 0 |
| 1280, 720 frame size | 0 | 0 | 0 |
| data.someXFrac placeholder | 0 | 0 | 0 |
| STEER bar −100…+100 | 0 | 0 | 0 |

RESULT: OK — no mismatches
