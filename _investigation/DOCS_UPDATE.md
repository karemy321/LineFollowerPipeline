# Documentation update — README.md + Integration Guide PDF (v2)

Date: 20 Oct 2026. Scope: documentation only. **No code file was changed.** The library source still holds the
uncommitted v2 work from 18 Oct (last modified 2026-10-18 22:39, before this update). Nothing was committed or pushed.

Backups of the documents before this update: `_investigation/docs_backup/README.md` and
`_investigation/docs_backup/LineFollowerPipeline_Integration_Guide.pdf`.

Sources of truth: `LineFollowerLibrary/src/main/java/com/example/linefollower/LineFollowerPipeline.java` (final
authority), `_investigation/CHANGES_v2.md`, commits `584d353` / `ec96af5` (steering ±50, tracking minimum 5) and
`08b19a1` (automatic colour learning). Statements that are not obvious from reading the code were checked by
running the code (§5).

---

## 1. How the PDF was produced and how it was regenerated

**Findings**

| Evidence | Result |
|---|---|
| PDF metadata (current and `08b19a1` versions) | Creator `Mozilla/5.0 … HeadlessChrome/154 … Edg/154`, Producer `Skia/PDF m154`, created 2026-10-08 15:02 UTC → printed from an **HTML page with headless Microsoft Edge** ("print to PDF"). A4 (594.96 × 841.92 pt), no header/footer, no page numbers. |
| `git log` of the PDF | `7ae1a39` (Jul), `08b19a1` (8 Oct, 416 KB), `ec96af5` (11 Oct, 1.24 MB). |
| `ec96af5` version vs `08b19a1` | Same creation date, but two extra non-subset fonts (`Segoe UI Regular`, `Consolas Regular`) and garbled glyphs (`﴾ ﴿` Arabic ornate parentheses, `‐`) on pages 2, 7, 9, 10. So the 11 Oct update **patched the text inside the existing PDF** (e.g. "-100 … +100" → "‐50 … +50", "2" → "5"); it was not re-printed. |
| Source search | No HTML / Markdown / DOCX / LaTeX / script for the guide in the repo, in any commit, in `_investigation/`, in the sibling folders (`beta`, `robophone`, backups) or in earlier Claude session scratchpads. |

**Conclusion:** no source exists, so the guide was **recreated faithfully**. It is now generated from a source kept in
`_investigation/pdf_source/`:

| File | Role |
|---|---|
| `guide_template.html` | The guide's prose and CSS. Styling reconstructed from the original PDF (measured with PyMuPDF): `@page` A4, margins 14 / 13 / 16 / 13 mm; Segoe UI 9.6 pt body, line-height 1.45, text `#1b1f24`; title 26 pt bold; eyebrow 8.5 pt bold `#1f5fae` spaced capitals; subtitle 12 pt / meta 8.8 pt `#5b6470`; H2 14 pt bold `#1f5fae` with 2 px rule; H3 11 pt bold; Consolas 8.8 pt code, background `#f3f5f7`, border `#d9dee4`, 3 px blue left bar, comments `#6a737d`; tables 8.89 pt, header `#f4f6f8` 8.2 pt semibold uppercase `#5b6470`; call-outs blue `#e8f0fa/#c4d7ef`, amber `#fff4e2/#f0d3a6`, green `#e7f5ec/#bfe3cc`; numbered step circles; green `NEW` badge 7 pt; page breaks before Calibration, Runtime Utility Methods, Full Example and Notes (as in the original). New: an amber `CHANGED` badge in the same style. |
| `build_guide.py` | Fills the template. **Every table and every code example is copied from README.md at build time**, so the PDF and README cannot disagree. It refuses code lines too wide for the page, then prints with headless Edge (`--headless --no-pdf-header-footer --print-to-pdf`): the same Chromium/Skia engine as the original (now m155). |
| `guide.html` | The filled HTML of the last build. |
| `check_docs.py` | Cross-checks code ↔ README ↔ PDF (§3). |
| `positions.py` | Compares text positions of the same elements in two PDFs (used to match the layout). |

Regenerate (from `_investigation/`):

```
py -X utf8 pdf_source/build_guide.py      # README.md + template -> ../LineFollowerPipeline_Integration_Guide.pdf
py -X utf8 pdf_source/check_docs.py ../LineFollowerLibrary/src/main/java/com/example/linefollower/LineFollowerPipeline.java ../README.md docs_backup/LineFollowerPipeline_Integration_Guide.pdf ../LineFollowerPipeline_Integration_Guide.pdf
py -X utf8 pdf_compare/compare.py docs_backup/LineFollowerPipeline_Integration_Guide.pdf ../LineFollowerPipeline_Integration_Guide.pdf pdf_compare
```

**Layout match (same element measured in old vs new PDF):**
- Eyebrow, title, subtitle, meta line and "What's new" heading sit at identical positions (±0.0 pt).
- Body, table, code and step positions are within 1 pt.
- Font sizes are the same everywhere.

**Fonts and characters:**
- The new PDF uses Segoe UI (Regular, Semibold, Bold, Italic), Consolas, and Cambria Math for the "⊥" symbol only. Segoe UI has no ⊥; it is drawn upright, so no synthetic Type3 font is produced.
- No garbled characters (the old PDF had `﴾ ﴿ ‐`).
- The original had no images or diagrams, so none are missing.

The guide grew from **10 to 18 pages** because of the new sections and the Range column. As in the original, the
forced page break before *Calibration* leaves the page before it (new p. 9) mostly empty.

---

## 2. Changes, section by section

### README.md (structure, headings, tables and tone kept; extended)

| Section | Change |
|---|---|
| **What's New** *(new, near the top)* | v2 table: change / what it means / code change needed? (distractor rejection, whole-⊥ steering, hold, predicted lock, calibration learns more, new FrameData fields). "Do I need to change my integration code?": no, the changes are purely additive (verified with `javap`, §5). How to restore exact v1 behaviour (snippet). Earlier changes: 11 Oct (steering ±50, so double the gain; `trackingMinValue` 2 → 5) and 8 Oct (autoColor, `trackIsBright` only a fallback, YUV/ARGB, …). |
| Minimal Integration | Calibration warning: robot straight, on a straight section, floor visible on both sides. |
| Output Format | Steering meaning rewritten: sign convention (negative = turn left; covers offset **and** heading), composed of offset + heading, ±50 toward the side where the line was lost. |
| Optional Configuration | `configure()` called later discards calibration. **New Range column in all 8 Config tables.** |
| Line Colour | `autoColor` = "Normal setup"; `trackIsBright` = "Manual fallback only", with exactly when it is used (autoColor off, or before colours are learned); RoboPhoneApp uses `bottomIgnoreFrac = 0.25`. |
| Scan Strip / Camera Mount | The strip moves up out of the ignored band; the tilt must keep the look-ahead below the horizon. |
| Track Memory | Gate is at most half the frame; relock needs 3× the frames to leave a valid-looking lock. |
| Distractor Rejection | **New "What you must ensure" list**: calibrate on the line with no distractor in the strip, use one reasonable tape width, recalibrate on tape change, set the camera geometry; plus the remaining limit. `rejectDistractors = false` also turns off hold and prediction; `trackHoldFrames` only works with `rejectDistractors` and is capped by `trackLostGraceFrames`. |
| Steering from the Whole ⊥ | ⊥ explained (base = H-ROI, vertical part = V-ROI line). Formula rewritten to match `runPipeline()` exactly: weights applied inside the clamp, heading angle limited to ±75°, clamp ±100, `result[0] = round(steering/2)`. W, centroid and calibrated column defined; sign paragraph and how the two parts map to `steerOffsetTerm` / `steerHeadingTerm`. |
| Sensitivity | **`curveFullScaleDeg` documented**: a public field that the code never reads (no effect). |
| Calibration Frames / Calibration | Phase 2 records the median position, line width, line-ness level and heading zero (with the exact acceptance rule); at least 15 frames with defaults; **checklist**; **"If calibration cannot complete"** (no timeout; what `processFrame()` does before calibration); `setTrackColors()` does nothing with autoColor off and throws on equal colours; `ISSUE_LOW_CONTRAST`: "even out the lighting" (taken from the PDF). |
| **When the Line Is Lost or Hidden** *(new)* | Table of every case (visible, hidden within the hold, hidden after the hold, leaves at the edge, returns within the grace period, gone after it, new line) with Steering, Tracking, `trackFound` and `lineHeld`; plus the `trackMemory = false` case. Verified by `readme_examples/probe/LostProbe.java` (§5). |
| Runtime Utility Methods | Rewritten as **compilable** statements (the old listing was not valid Java); **`init(width, height, config)` added** (public, previously undocumented). |
| Frame Callback / FrameData | Callback thread stated. Clarified: `centroidXFrac` is −1 while the line is held; `tiltDeg` is image-space, for drawing only, 0 before calibration; colours are 0 until learned; steering is 0 and tracking 100 during calibration; use `trackFound`, not `tracking > 5`. The six v2 fields, including when each is 0. |
| Drawing the overlay | `data.someXFrac` (no such field) replaced by real fields; state readout shows calibration % and the held state; **new steering debug readout** example using the v2 fields. |
| Full Example | Frame size **960 × 1280** (the app's real size) instead of 1280 × 720; drive only once calibrated. |
| Behaviour Reference | Added "Back on a straight after a turn" (previously only in the PDF). |
| **Recommended Starting Values** *(new)* | Lecturer's program: steering `(0 − Steering) × c` with **c = 1.0** (0.5 is gentler); power `round(x × (Power / 8) + 5)` with **x = 0 first, then 1**; dead-zone and weaving advice; keep library defaults (CHANGES_v2 §7). |
| Notes | Thread note includes `requestRecalibration()`; recalibrate on tape-width change; gate is around the predicted position; harsh-scene and performance notes added from the PDF (performance updated to ≈ 0.9 ms at 960 × 1280, zero allocation). |
| RoboPhoneApp | **HUD STEER bar −50…+50** (the README said −100…+100; the app uses ±50); the app does not show the v2 fields yet. |
| Building & Testing | **8 of 24 unit tests fail with v2 defaults** because they expect v1 values; they pass with `steerHeadingWeight = 0` and `trackHoldFrames = 0` (verified, §5). |

### LineFollowerPipeline_Integration_Guide.pdf (same look; content updated; new sections in the same style)

| Old page → new page | Change |
|---|---|
| 1 → 1 | Meta line now "Updated October 2026 (v2)". **"What's new in this version" box replaced** with the v2 changes (the old box described the 8 Oct release). |
| — → 1–2 | **New section "Changelog — What an Integrator Needs to Know"**: badge legend, v2 table, "Do I need to change my code?" box, v1-restore snippet, earlier changes (11 Oct ±50 / min 5; 8 Oct colour learning). |
| 1–2 → 2–3 | Minimal Integration and its boxes: the calibration box adds "straight, on a straight section". |
| 2 → 3 | Input Formats: content unchanged (table now generated from the README). |
| 2 → 3–4 | Output Format: steering meaning (sign convention, offset + heading, ±50 when lost); garbled `‐50 …` and `﴾default 5﴿` fixed. |
| 2–4 → 4–9 | Optional Configuration: the configure() note; **Range column**; tables generated from the README. The old `NEW` badges (8 Oct features) were replaced by v2 `NEW` / `CHANGED` badges. Descriptions updated as in the README (autoColor/trackIsBright, vRoiHalfWidthPx, cameraTiltDeg, track memory, relock). Track Memory paragraph: "steering reads exactly ±50" now applies only at the frame edge; a hidden line is held first. **New sub-sections "Distractor Rejection"** (rules, "What you must ensure" box, table) and **"Steering from the Whole ⊥"** (formula, symbols, signs, table). Sensitivity: `curveFullScaleDeg` added. Tracking Minimum: code example added (as in the README). |
| 5 → 10–11 | Calibration: phase 2 updated (`CHANGED`), 15 frames, `ISSUE_*` table, **new checklist**, **new "If calibration cannot complete" box**, `setTrackColors()` note, recalibrate on tape-width change. |
| — → 11 | **New section "When the Line Is Lost or Hidden"**. |
| 6 → 12 | Runtime Utility Methods: compilable statements; `init()` added. |
| 6–7 → 12–14 | FrameData table: the six v2 fields (`NEW`) and clarifications; garbled `﴾a hard turn also reads 5﴿` fixed. |
| 7–8 → 14–15 | Drawing the overlay: `someXFrac` fixed; state readout shows the held state; **new steering debug readout**. |
| 9 → 16–17 | Full Example: 960 × 1280; drive only when calibrated. Behaviour Reference: three v2 rows (`NEW`); "±50 (exactly)" → "±50" (after the grace period it can read slightly less); the old `NEW` on the dead-end row removed. |
| — → 17 | **New section "Recommended Starting Values (lecturer's test program)"**. |
| 10 → 18 | Notes: same as the README (tape width, predicted gate, camera geometry, 960 × 1280, performance ≈ 0.9 ms). RoboPhoneApp: v2 fields not shown in the app; **new box on the 8 failing unit tests**. |

---

## 3. Every Config and FrameData field — code vs README vs PDF

Generated by `pdf_source/check_docs.py` (full output: `pdf_source/check_report.md`). The type/default column compares
the README row with the **code** declaration. The "PDF (old)" column shows what the previous PDF lacked.

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

### README examples in the PDF, and outdated values

All 22 README `java` blocks are present in the new PDF (the first and last line of each were found).

| Outdated pattern | README | PDF (new) | PDF (old) |
|---|---|---|---|
| `-100 … +100` steering (outside the "was" note) | 0 | 0 | 0 (patched on 11 Oct) |
| tracking `2 … 100` | 0 | 0 | 0 (patched on 11 Oct) |
| `1280, 720` frame size | 0 | 0 | 2 |
| `data.someXFrac` placeholder | 0 | 0 | 1 |
| STEER bar `−100…+100` | 0 | 0 | 0 (old README: 1) |

**Result: no mismatches.** All 37 Config fields (type and default identical to the code) and all 34 FrameData fields
are documented in both documents, every README example is in the PDF, and no outdated value remains. The PDF tables
are generated from the README tables, so ranges, defaults and descriptions are identical by construction.

---

## 4. README examples — compile result

`readme_examples/extract_readme_examples.py` extracts **every** ```` ```java ```` block of README.md into
`readme_examples/src/ReadmeExamples.java`: one method per block, plus stub types for the placeholders the README uses
(`camera`, `robot`, `myOverlay`, `canvas`, `status`, `Yuv`, …). No snippet is edited.

```
javac --release 8 -Xlint:all -encoding UTF-8 -d readme_examples/classes ^
      ../LineFollowerLibrary/src/main/java/com/example/linefollower/LineFollowerPipeline.java ^
      readme_examples/src/ReadmeExamples.java                      (JDK 21.0.10, Android Studio jbr)
```

| | Result |
|---|---|
| New README (22 `java` blocks) | **exit 0, no errors, no warnings** (`readme_examples/compile_log.txt` is empty) |
| Old README (backup), same procedure, as a negative control | exit 1, **23 errors** (`readme_examples/compile_log_old_readme.txt`): the utility-method listings were not valid Java, and `data.someXFrac` does not exist |
| Library alone with `--release 8` | compiles, so "Java 8 or higher" is still correct (Gradle builds it for Java 11) |

---

## 5. Other checks run (evidence for statements in the documents)

| Check | Result |
|---|---|
| Public API diff, `ec96af5` vs current (`javap -public`, files in `readme_examples/api/`) | Only additions: 9 Config fields and 6 FrameData fields; no method or field removed or renamed. Hence "no code change needed". |
| Lost / hidden line behaviour (`readme_examples/probe/LostProbe.java`, synthetic 320 × 240 frames) | Hidden mid-image: S/T held for 6 frames with `trackFound=false` and `lineHeld=true`, then S = +50, T = 5. Leaving at the right edge: S = +50, T = 5 at once, and kept. Calibration took 15 frames. |
| Project unit tests (JUnit 4.13.2, current library) | **16 / 24 pass**. The 8 failures are the 7 tests that expect v1 steering, plus `lostLineReportsTrackLost`. With `steerHeadingWeight = 0` and `trackHoldFrames = 0` (copy in `readme_examples/variant_v1defaults/`): **24 / 24**. |
| App HUD range (`MainActivity.java`, `activity_main.xml`) | STEER SeekBar is −50…+50, so the README was corrected. |

---

## 6. Visual check — before / after page images

| What | Path |
|---|---|
| OLD pages (10), 110 dpi | `_investigation/pdf_compare/old/page_01.png` … `page_10.png` |
| NEW pages (18), 110 dpi | `_investigation/pdf_compare/new/page_01.png` … `page_18.png` |
| Side by side (old left; right = the new page with the same section), 100 dpi | `_investigation/pdf_compare/side_by_side/old_01__new_01.png`, `old_02__new_03.png`, `old_03__new_04.png`, `old_04__new_06.png`, `old_05__new_10.png`, `old_06__new_12.png`, `old_07__new_14.png`, `old_08__new_14.png`, `old_09__new_16.png`, `old_10__new_18.png` |
| Original 8 Oct print used to measure the styling | `_investigation/pdf_work/guide_08b19a1.pdf` (plus zoomed crops `zoom_*.png` and extracted text `old_text.txt`) |

Every new page was inspected. Problems found and fixed during the check:
- Code comments were clipped at the right edge in *Runtime Utility Methods*. The lines were shortened, and the build now refuses code lines longer than 103 characters.
- ⊥ inside bold or italic text was rendered through a synthetic Type3 font. It is now drawn upright with Cambria Math.
- Consolas Bold appeared where code sat inside bold text. Code is now pinned to regular weight, as in the original.
- Unchanged fields carried misleading `CHANGED` badges. These were removed.

---

## 7. git status

```
$ git status --short
 M LineFollowerLibrary/src/main/java/com/example/linefollower/LineFollowerPipeline.java   <- already modified before this task (v2 code, 18 Oct); not touched
 M LineFollowerPipeline_Integration_Guide.pdf                                             <- this update
 M README.md                                                                              <- this update
?? _investigation/                                                                        <- this report, PDF source/generator, checks, images
```

Files changed by this task: `README.md`, `LineFollowerPipeline_Integration_Guide.pdf`, and files under
`_investigation/` only. Nothing was committed or pushed.
