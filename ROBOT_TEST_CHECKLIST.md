# Robot test checklist — LineFollowerPipeline v3

Branch `v3`. So far everything has been tested only in simulation. The tests run in two stages:

- **Stage A — by hand, app only.** RoboPhoneApp on the phone; you move the robot (or the phone on
  its mount) along the track by hand. No motors and no Blockly. This checks what the camera
  pipeline sees and outputs.
- **Stage B — closed loop inside RoboPhone.** The library runs inside the RoboPhone system and the
  Blockly program drives the robot. Start this stage only after Stage A looks good.

Fill in the table at the end of each stage and send it back, together with the recorded files
(§A0.4). The defaults are tuned from these results.

---

## Stage A — manual tests with RoboPhoneApp

### A0. Preparation (once)

1. **Install** the app from `RoboPhoneApp/` (Android Studio ▶ Run, or `gradlew installDebug`). It
   builds the v3 library from source.
2. **Logcat** on the PC (USB debugging): `adb logcat -s RoboPhone`
   - `PERF fps=… pipe_ms=… pipe_max_ms=… frame_ms=… frames=30` — every 30 frames.
   - `AE/AWB LOCKED after calibration` / `AE/AWB unlocked` — only when the lock setting is on.
   - `EV compensation -1.0 -> index -3 (step 0.33)` — when EV is changed.
   - `REC off: <path>` / `CAPTURE saved <path>` — recording files (§A0.4).
3. **HUD controls.** All are OFF / 0 by default.
   - Row 1, remembered between app starts, so check them before each test:
     - **AE/AWB LOCK**: locks exposure and white balance when calibration finishes; RECALIBRATE
       unlocks first.
     - **EV**: cycles 0, −0.5, −1, −1.5, −2, +0.5, +1 (exposure compensation).
     - **v3 HUD**: two extra lines, see below. **Tick it for every test.**
   - Row 2, **off at every app start**:
     - **REC CSV**: per-frame log.
     - **CAPTURE**: keeps the last ~2 s of frames and auto-saves them on events.
     - **SAVE 2s**: saves the capture now.
4. **Recording.** Turn it on for every Stage A test.
   - **Tick REC CSV before each test and untick it after.** Each tick starts a new file, so one file
     per test (or per pass) keeps things simple. Write the time or file name in the table.
   - **Tick CAPTURE for tests A3–A8.** The app then saves the last ~2 s of camera frames (1/4 size)
     on its own whenever the line is coasted, the near line is rebuilt, a band is rejected, the line
     is lost, or the steering jumps by ≥ 30. At most 10 captures per app start, each about 15 MB.
     **Tap SAVE 2s right after anything odd you see.** A toast says "capture saved".
   - Files are in `/sdcard/Android/data/com.example.firstproject/files/linefollower/`:
     - `lf_YYYYMMDD_HHMMSS.csv`: one line per frame, every FrameData field plus `t_ms` and `lib_ms`.
     - `lf_….cfg.txt`: the Config used.
     - `lfcap_YYYYMMDD_HHMMSS_<reason>.bin`: a capture.
   - Pull them to the PC:

     ```
     adb pull /sdcard/Android/data/com.example.firstproject/files/linefollower ./recordings
     ```

     If `adb pull` is refused (some Android 11+ phones), use Android Studio ▸ View ▸ Tool Windows ▸
     **Device Explorer** ▸ `sdcard/Android/data/com.example.firstproject/files/linefollower` ▸
     right-click ▸ Save As.
   - **Uninstalling the app or "Clear storage" deletes these files.** Pull them first.
     `installDebug` over the old app keeps them.
   - Summarise on the PC: `py tools/analyze_recording.py recordings` (add `--from 12 --to 20` for one
     pass, in seconds from the file start). It prints:
     - steering: largest step, jumps ≥ 25, frames at ±50 and how many of them had glare in view;
     - tracking;
     - coast, hold and rebuilt-base spells, and rejected bands, with their times;
     - fps and `lib_ms`.
   - Replay a capture on the PC: `gradlew replay -Pcapture=recordings/lfcap_….bin`, run in
     `LineFollowerLibrary/`.
5. **Calibrate** as before. Put the robot straight on a straight section, with the line in the cyan
   STEER strip, floor visible on both sides, and nothing else in the strip. Wait for TRACKING.
6. **Moving by hand.**
   - **Push the whole robot** rather than the bare phone, so the camera height and tilt (20 cm,
     45°) stay as calibrated. If you move only the phone on its mount, keep the mount height and angle.
   - Keep the robot's centre over the line and turn it with the line, the way the robot would drive.
   - **Passes** are timed over a measured 1 m:

     | Pass | Time for 1 m | Speed |
     |---|---|---|
     | slow | ~10 s | ~0.1 m/s |
     | normal | ~3 s | ~0.3 m/s, an easy walking push |
     | fast | ~1.5–2 s | ~0.5–0.6 m/s, the target robot speed |

**What the HUD shows**

| HUD | FrameData field | Meaning |
|---|---|---|
| STEER number + bar | `steering` | −50 … +50. **Negative = line is to the left, positive = line is to the right.** |
| TRACK % + bar | `tracking` | 100 = straight road ahead; drops before curves and when the line is lost |
| TRACKING / TRACK LOST | `trackFound` | line found in the scan strip this frame |
| `COAST` | `coasting` (and `lineHeld`) | line hidden mid-image, the pipeline coasts on its last motion (status reads TRACK LOST) |
| `HOLD` | `lineHeld` without coasting | v2-style hold; should not appear with v3 defaults |
| `RECON` | `baseReconstructed` | near line hidden (glare), its position rebuilt from the line farther ahead |
| `corr` | `corridorCoverage` | 0 … 1, share of the corridor slices on the line; 0 = v2 logic used this frame (normal on corners / line ends) |
| `rms` | `corridorFitRms` | corridor fit error in line widths (small = good) |
| `mask` | `maskedFrac` | share of the view masked as glare |
| `rej` | `rejectedBands` | candidate bands rejected or trimmed as distractors this frame |
| `AE` / `EV` | — | `LOCKED` or `auto`; exposure compensation |
| 2nd line: `max step`, `jumps(>=25)`, `frames at ±50` | from `steering` | largest steering change between two frames, count of changes ≥ 25, count of frames at ±50. **Tap the line to reset to 0 before each pass.** |
| `REC`, `CAP n` | — | recording on; captures saved so far |

**General pass rule for every Stage A test**
- **Right direction:** steering points the right way. If the line is to the right of the robot's
  centre, STEER is positive.
- **Smooth:** steering changes smoothly. No jumps ≥ 25 on straights and gentle curves.
- **No ±50 in glare:** `frames at ±50` stays 0 in glare. ±50 is correct only when the line really
  leaves the side of the image: corners and deliberate exits.

### A1. Baseline: straight and curves, lock OFF

- **Set up:** AE/AWB LOCK off, EV 0, normal room light. Calibrate. REC CSV on.
- **Do:**
  1. Push the robot along a straight, then through a gentle curve: slow, normal and fast passes.
  2. Standing on a straight, shift the robot sideways ~2 cm left of the line, then ~2 cm right.
  3. Standing on the line, turn it ~10° left, then ~10° right.
- **Watch:** STEER, TRACK, `corr`, `max step`, `jumps`.
- **Pass:**
  - STEER is about 0 (±3) centred on a straight;
  - positive when the line is right of the robot (robot shifted left), negative the other way;
  - TRACK ≥ 80 on straights and lower before curves;
  - `corr` mostly ≥ 0.8 on straights;
  - `jumps` 0 on all passes;
  - TRACKING the whole time.

### A2. AE/AWB lock OFF vs ON

- **Set up:** first lock OFF; then tick AE/AWB LOCK, press RECALIBRATE with the robot on the line,
  and wait for TRACKING. Logcat must show `AE/AWB LOCKED after calibration`, and the HUD `AE LOCKED`.
- **Do (each setting):**
  1. Robot standing on the line: shade the track with your hand, then switch a lamp on and off.
  2. Do one normal pass along a straight and a curve.
  3. With the lock ON, press RECALIBRATE. Logcat shows `AE/AWB unlocked`, then `LOCKED` again.
- **Watch:** whether the preview brightness changes on its own; STEER and TRACK while shading; `corr`.
- **Pass:**
  - with the lock ON, the image does not re-expose;
  - STEER stays within ±5 of its unshaded value with both settings;
  - lock and unlock appear in logcat;
  - the pass is as good as A1.
  - If the line becomes too dark to see in shade with the lock ON, note where.

### A3. Glare on the near track (glossy tape under a lamp)

- **Set up:** a desk lamp low and to the front, so you see a bright highlight on the glossy tape just
  in front of the robot, in or just above the cyan STEER strip. Lock OFF, EV 0 first. REC CSV and
  CAPTURE on.
- **Do:**
  1. Robot standing on a straight under the highlight: watch for 10 s.
  2. Push it slowly, then normally, then fast through the highlight on a straight. Reset the jump
     counters before each pass.
  3. Move the lamp so the highlight is on a gentle curve, and push through it.
  4. Repeat 1–3 with **EV −1**, then with **EV −1 + AE/AWB LOCK**. Recalibrate outside the glare,
     then move into it.
- **Watch:** `mask`, `RECON`, `COAST`, STEER, TRACK, `frames at ±50`, `max step`.
  - `RECON` and `COAST` (with TRACK LOST) for a moment are expected.
  - TRACK goes down a little.
- **Pass:**
  - standing: STEER within ±5 of the value without the lamp;
  - passes: `frames at ±50` = 0, `max step` ≤ ~10, steering in the right direction;
  - TRACKING returns within ~0.2 s after the highlight.

### A4. Glare on the floor

- **Set up:** the lamp highlight (or window light) on the floor **beside** the line, not on the tape.
  First in the STEER strip, then farther ahead in the view. Lock OFF, EV 0. REC CSV and CAPTURE on.
- **Do:** robot standing for 10 s with the highlight in the strip, then in the look-ahead; then
  normal and fast passes past it.
- **Watch:**
  - the green centre line in the overlay stays on the tape;
  - STEER, `mask`, `rej`, `frames at ±50`.
- **Pass:**
  - STEER within ±5 of the value without the lamp;
  - the centre line never moves onto the highlight;
  - `frames at ±50` = 0, `jumps` 0.

### A5. A dark object beside the line

- **Set up:** beside a straight, place:
  - (a) a black bag or a chair leg 3–5 cm from the tape;
  - (b) a short piece of the same tape (5–6 cm long), parallel to the line and 3 cm away.

  Lock OFF, EV 0. REC CSV and CAPTURE on.
- **Do:** robot standing with the object in the STEER strip; then slow, normal and fast passes
  past each object.
- **Watch:** the overlay centre line, STEER, `rej` (may count up, which is fine), `max step`.
- **Pass:**
  - STEER within 3 of the value without the object;
  - STEER never moves toward the object;
  - the centre line never jumps onto it;
  - `jumps` 0.

### A6. A blob touching the line

- **Set up:** a dark object touching the tape from the side:
  - (a) a large one, ~12 cm, such as a bag or a sheet of black card;
  - (b) a small one, ~4 cm.

  Lock OFF, EV 0. REC CSV and CAPTURE on.
- **Do:** normal and fast passes past each blob.
- **Watch:** the centre line, STEER, TRACK, `rej`.
- **Pass:**
  - the centre line stays on the tape;
  - STEER doesn't move toward the blob by more than ~5;
  - TRACK may dip next to the blob but is **≥ 60 once the line past the blob is visible**, and is
    back to normal after it.

### A7. A 90° corner, moved through by hand

- **Set up:** a sharp 90° corner. Lock OFF, EV 0. REC CSV and CAPTURE on.
- **Do:**
  1. Push the robot into the corner and turn it through, as the robot would: slow, normal, fast;
     3 times each.
  2. Once with the lamp highlight of A3 placed just before the corner.
- **Watch:** TRACK before the corner, STEER direction and how long ±50 lasts, STEER on the next straight.
- **Pass:**
  - TRACK ramps down before the corner;
  - STEER goes toward the turn and never the opposite way;
  - ±50 only while the line leaves the image at the side (a short pin is correct);
  - STEER back near 0 on the next straight;
  - with the lamp: no wrong-way steering.

### A8. A deliberate exit off the line

- **Set up:** a straight. Lock OFF, EV 0. REC CSV and CAPTURE on.
- **Do:**
  1. Slide the robot sideways off the line until the line leaves the image, ~15–20 cm (in
     simulation it leaves after ~15 cm). Slide to the left, then to the right; slow and fast.
  2. Bring it back onto the line.
  3. Without moving, cover the line mid-image (strip and the view ahead) with a sheet of
     floor-coloured paper: once briefly (~0.3 s), then for ~2 s.
- **Watch:** STEER, TRACK, TRACK LOST, `COAST`, and how fast TRACKING returns.
- **Pass:**
  - 1: STEER goes to the side where the line went (line leaves on the right → +50) and stays
    there while the line is off; TRACK at its minimum.
  - 2: TRACKING again as soon as the line is back in the strip; STEER comes back smoothly from
    ±50 toward 0, with no jump to the opposite side.
  - 3, brief cover: `COAST` with TRACK going down, **no ±50**, and TRACKING again with STEER as
    before.
  - 3, long cover: `COAST` for at most 0.6 s (`glareCoastMs`), then ±50 toward the side the line
    was last on (v2's "line lost" behaviour). The CSV shows how long the coast lasted.

### A9. PERF numbers

- **Set up:** logcat running; tracking on the track.
- **Do:** 1 minute of normal passes (straights and curves); repeat in the glare of A3; repeat once
  with REC CSV and CAPTURE on.
- **Write down:** `fps`, `pipe_ms`, `pipe_max_ms`, `frame_ms` from 3–4 `PERF` lines; the phone model.
  - `pipe_ms` includes the recorder.
  - The CSV's `lib_ms` is the library alone; `analyze_recording.py` prints it.
- **Pass:** `fps` ≥ 25; `pipe_ms` ≤ 8; `pipe_max_ms` ≤ 20.
  - On the desktop the library takes 0.6 ms per frame (v2: 1.0 ms), and capture adds ~0.2 ms; a
    phone is expected to be several times slower.

### Stage A results — fill in and send back

Steering values are STEER as shown. "Jumps / ±50" = the HUD's `jumps(>=25)` / `frames at ±50`
after the pass.

| # | Test | Variant | Pass / fail | Numbers | CSV / capture files | Notes |
|---|---|---|---|---|---|---|
| A1 | Baseline | slow / normal / fast | | steer centred: · steer 2 cm left/right: · steer ±10°: · min TRACK on straight: · corr: · jumps: | | |
| A2 | Lock OFF | — | | steer shade / no shade: · image re-exposes (y/n): | | |
| A2 | Lock ON | — | | steer shade / no shade: · image fixed (y/n): · lock/unlock in logcat: | | |
| A3 | Near glare | EV 0, lock off | | mask: · RECON/COAST: · steer standing min/max: · max step: · ±50: · back to TRACKING (s): | | |
| A3 | Near glare | EV −1 | | mask: · RECON/COAST: · steer standing min/max: · max step: · ±50: | | |
| A3 | Near glare | EV −1 + lock | | mask: · RECON/COAST: · steer standing min/max: · max step: · ±50: | | |
| A4 | Floor glare | in strip / ahead | | steer with/without: · centre line moved (y/n): · ±50: · jumps: | | |
| A5 | Object beside | (a) bag / leg | | steer with/without: · max steer passing: · jumped onto it (y/n): | | |
| A5 | Object beside | (b) short tape | | steer with/without: · max steer passing: · jumped onto it (y/n): | | |
| A6 | Blob touching | (a) ~12 cm | | steer shift: · min TRACK: · TRACK once line visible past it: | | |
| A6 | Blob touching | (b) ~4 cm | | steer shift: · min TRACK: · TRACK once line visible past it: | | |
| A7 | 90° corner | slow / normal / fast | | min TRACK before: · ±50 how long: · wrong-way (y/n): · steer after: | | |
| A7 | 90° corner | with lamp | | wrong-way (y/n): · ±50: | | |
| A8 | Exit off line | left / right | | steer while off: · frames to re-find: · jump on return (y/n): | | |
| A8 | Line covered | brief / long | | brief: COAST seen · ±50 (should be 0) · long: coast length (CSV) · ±50 side: | | |
| A9 | PERF | normal | | fps: · pipe_ms: · pipe_max_ms: · frame_ms: | | phone: |
| A9 | PERF | glare | | fps: · pipe_ms: · pipe_max_ms: · frame_ms: | | |
| A9 | PERF | REC + CAPTURE on | | fps: · pipe_ms: · pipe_max_ms: · lib_ms (CSV): | | |

Also useful: the tape type (glossy / matte, width), the floor, and a photo or screenshot of any
failure with the HUD visible.

---

## Stage B — closed loop inside RoboPhone

### B0. Preparation

1. **Integration.** Use the library in the RoboPhone system through `LineFollowerHelper`; the sketch
   is in `line_follower_block.js`, part 4.
   - Use the same Config as RoboPhoneApp, but set `cameraTiltDeg` to the real mount angle.
   - The camera may deliver grayscale (`getGrayscaleFrame`) or YUV. Note which one; with grayscale
     the line and floor must differ in brightness.
2. **Recording.** It works the same way. The recorder uses only the pipeline's FrameCallback, so it
   runs inside any host app:

   ```java
   LineFollowerHelper.init(camera);
   LineFollowerHelper.startRecording(context.getExternalFilesDir("linefollower"), true, true); // CSV, capture
   …  // Blockly loop
   LineFollowerHelper.saveCapture();   // optional: e.g. from a button or a "save capture" block
   LineFollowerHelper.stopRecording(); // when the program stops (also done by release())
   ```

   - If the host has its own FrameCallback (an overlay), pass it as the third argument of
     `Recorder.attach` instead of `null`. The pipeline has only one callback slot.
   - Files go to `/sdcard/Android/data/<host package>/files/linefollower/`. Find the package with
     `adb shell pm list packages | findstr -i robo`, then:

     ```
     adb pull /sdcard/Android/data/<host package>/files/linefollower ./recordings_b
     ```

     If that is refused, use Android Studio's Device Explorer as in §A0.4.
   - If the helper has no `Context`, any folder the host app may write to works.
   - Without a HUD, `py tools/analyze_recording.py recordings_b` gives the same numbers: steering
     jumps, ±50, coasting, fps and `lib_ms`.
3. **Blockly program** (lecturer's test program): `motor steering = (0 − Steering) × c`,
   `power = round(x × Power / 8 + 5)`.
   - **Simulation result, to be confirmed in Stage B:** c = 1.0; x = 0 (constant power) at
     ~0.5–0.6 m/s; x = 1 only at low speed.
   - B2 and B3 test exactly this.
4. Mark a start line and time laps with a phone stopwatch. **Record every run** (CSV + capture); one
   file per run is easiest (stop and start the program).

### B1. Weaving on straights

- **Set up:** c = 1.0, x = 0 (power 5). A long straight (≥ 2 m). Lock OFF, EV 0.
- **Do:** 3 runs along the straight, starting centred; then 2 runs starting ~3 cm beside the line.
- **Look for:** does the robot weave (visible side-to-side swing) or settle on the line?
- **Write down:**
  - weaving (y/n, how many swings per metre);
  - largest side error (cm, by eye);
  - from the CSV: steering sign changes per second and steering std on the straight.
- **Pass:** no visible weaving; starting beside the line it settles within ~0.5 m; sign changes
  ≤ ~1 per second.

### B2. Blockly values (c, x)

- **Set up:** a lap with straights, gentle curves and one 90° corner. Lock OFF, EV 0. Normal power.
- **Do:** 2 laps for each of:
  - c = 1.0, x = 0
  - c = 1.0, x = 1
  - c = 0.5, x = 0
  - c = 0.5, x = 1
- **Write down:** laps completed; times off the line and where; weaving (y/n); lap time.
- **Pass / result:** the best pair. The simulation predicts c = 1.0 completes everything at normal
  speed; c = 0.5 is gentler but slower in sharp corners.

### B3. About 0.6 m/s

- **Set up:** set the power so the robot does ~0.6 m/s on a straight (3 m in ~5 s). Lock OFF, EV 0;
  then repeat the best variant with lock ON.
- **Do:** 2 laps each:
  - c = 1.0, x = 0 (the simulation recommendation);
  - c = 1.0, x = 1 (the simulation predicts weaving on straights);
  - c = 0.5, x = 1 (or `Power / 12`).
- **Write down:** measured speed; laps; times off the line and where; weaving on straights (y/n);
  slows before turns (y/n).
- **Pass:** at least one variant does 2 laps without leaving the line and without weaving. Note
  whether it is the simulation's choice.

### B4. Full laps under test conditions

- **Set up:** the best values from B2 / B3:
  - (a) normal track;
  - (b) a lamp highlight on the tape at one place (as A3);
  - (c) a dark object beside the line and a blob touching it (as A5 / A6);
  - (d) the same as (b) with AE/AWB LOCK + EV −1, if A3 showed it helps.
- **Do:** 3 laps each.
- **Write down:** laps completed (of 3); where it left the line; captures saved (names); anything odd.
- **Pass:** 3 of 3 laps in (a) – (c).

### B5. Timing inside RoboPhone

- From the B4 CSVs: `analyze_recording.py` prints fps and `lib_ms` (median / 95 % / max).
- **Pass:** fps ≥ 25; `lib_ms` 95 % ≤ 8 ms.

### Stage B results — fill in and send back

| # | Test | Variant | Pass / fail | Numbers | CSV / capture files | Notes |
|---|---|---|---|---|---|---|
| B0 | Setup | — | — | frame format (gray / YUV): · size: · phone: · tilt: · power for 0.6 m/s: | | |
| B1 | Straight | centred | | weaving: · max side error (cm): · sign changes/s: · steer std: | | |
| B1 | Straight | start 3 cm off | | settles within (m): · weaving: | | |
| B2 | Values | c 1.0, x 0 | | laps /2: · off line: · weaving: · lap time: | | |
| B2 | Values | c 1.0, x 1 | | laps /2: · off line: · weaving: · lap time: | | |
| B2 | Values | c 0.5, x 0 | | laps /2: · off line: · weaving: · lap time: | | |
| B2 | Values | c 0.5, x 1 | | laps /2: · off line: · weaving: · lap time: | | |
| B3 | ~0.6 m/s | c 1.0, x 0 | | speed: · laps /2: · off line: · weaving: | | |
| B3 | ~0.6 m/s | c 1.0, x 1 | | speed: · laps /2: · off line: · weaving: | | |
| B3 | ~0.6 m/s | c 0.5, x 1 | | speed: · laps /2: · off line: · weaving: | | |
| B3 | ~0.6 m/s | best + lock ON | | speed: · laps /2: · off line: · weaving: | | |
| B4 | Laps | (a) normal | | laps /3: · where off: | | |
| B4 | Laps | (b) lamp glare | | laps /3: · where off: | | |
| B4 | Laps | (c) objects | | laps /3: · where off: | | |
| B4 | Laps | (d) glare + lock + EV −1 | | laps /3: · where off: | | |
| B5 | Timing | from B4 CSVs | | fps: · lib_ms median / 95 % / max: | | |

Also useful: the Blockly program as used (a screenshot), the battery state, and a video of any
failure.
