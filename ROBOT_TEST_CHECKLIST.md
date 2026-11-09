# On-robot test checklist — LineFollowerPipeline v3

Branch `v3`. Everything below has only been tested in simulation so far. Please run the seven tests,
fill in the table at the end and send it back — the defaults will be tuned from it.

## 0. Preparation (once)

1. Build and install the app from `RoboPhoneApp/` (Android Studio ▶ Run, or
   `gradlew installDebug`). It uses the v3 library from source.
2. Logcat on the PC while testing (USB debugging):

   ```
   adb logcat -s RoboPhone
   ```

   Useful lines (tag `RoboPhone`):
   - `PERF fps=… pipe_ms=… pipe_max_ms=… frame_ms=… frames=30` — every 30 frames.
   - `AE/AWB LOCKED after calibration` / `AE/AWB unlocked` — only when the lock setting is on.
   - `EV compensation -1.0 -> index -3 (step 0.33)` — when EV is changed.
3. New controls in the HUD (row under the FPS line). **All are OFF / 0 by default**, and are
   remembered between app starts — check their state before each test:
   - **AE/AWB LOCK** — locks exposure and white balance when calibration finishes; RECALIBRATE
     unlocks first.
   - **EV** button — cycles 0, −0.5, −1, −1.5, −2, +0.5, +1 (exposure compensation).
   - **v3 HUD** — shows an extra line:
     `corr 0.95  rms 0.08  COAST RECON HOLD mask 3%  AE auto  EV +0.0`
     - `corr` — corridor coverage 0…1 (0 = this frame used the v2 logic; normal on corners / ends).
     - `rms` — corridor fit error in line widths (small = good).
     - `COAST` — line hidden mid-image, coasting on the prediction (status reads TRACK LOST).
     - `RECON` — near line hidden (glare), its position taken from the line farther ahead.
     - `HOLD` — v2-style hold (only with glare handling off; should not appear with v3 defaults).
     - `mask` — share of the view masked as glare.
     - `AE` — `LOCKED` or `auto`; `EV` — current compensation.
4. Calibrate as before: robot straight on a straight section, line in the STEER strip, floor visible
   on both sides, nothing else in the strip. Wait for TRACKING.
5. Tick **v3 HUD** for all tests (it only changes the display).

**Blockly program:** the recommended start (README) is steering gain **c = 1.0**, power **x = 0**
(constant power 5) first, then **x = 1**. For test 6 use the power that gives about 0.6 m/s.

## 1. v3, lock OFF (baseline)

- **Set up:** AE/AWB LOCK off, EV 0. Normal track, normal room light. Calibrate.
- **Do:** let the robot run the whole track 2 times.
- **Look for:** stays on the line; steering calm on straights; TRACK drops before curves; no
  repeated TRACK LOST on straights.
- **Write down:** completed laps (0/1/2), number of times it left the line, typical `corr` on a
  straight, typical STEER on a straight, the lowest TRACK seen.
- **Pass:** 2 laps completed without leaving the line; `corr` mostly ≥ 0.8 on straights.

## 2. v3, lock ON

- **Set up:** tick AE/AWB LOCK, EV 0. Press RECALIBRATE (robot on the line), wait for TRACKING.
  Logcat must show `AE/AWB LOCKED after calibration`; the v3 HUD shows `AE LOCKED`.
- **Do:** (a) run the track 2 laps; (b) with the robot standing on the line, shade the track with
  your hand / switch a lamp on and off — the preview brightness should no longer change on its own;
  (c) press RECALIBRATE — logcat `AE/AWB unlocked`, then `LOCKED` again after calibration.
- **Write down:** laps completed, times it left the line, whether the image stayed fixed in (b),
  whether (c) unlocked and re-locked, STEER on a straight under shade vs no shade.
- **Pass:** same as test 1, the image does not re-expose in (b), lock / unlock appear in logcat.
  If the robot does worse than test 1 (e.g. the line is too dark in shaded parts), note where.

## 3. Glare under a lamp (glossy tape, near track)

- **Set up:** a desk lamp low and to the front, so you see a bright highlight on the glossy tape
  just in front of the robot (in or just above the cyan STEER strip). Lock OFF, EV 0 first.
- **Do:**
  1. Robot standing on a straight under the highlight: watch the HUD for 10 s.
  2. Drive slowly through the highlight (straight), then through a gentle curve with the
     highlight on it.
  3. Repeat 1–2 with **EV −1**, then with **EV −1 + AE/AWB LOCK** (recalibrate outside the glare,
     then drive into it).
- **Look for:** no ±50 kick and no sudden STEER jump while the highlight passes; status may read
  TRACK LOST with `COAST` / `RECON` for a moment — that is expected; the robot keeps going straight
  and slows down a little.
- **Write down (per variant):** `mask` %, whether `RECON` / `COAST` appear, STEER while standing
  (min / max), the largest STEER jump while driving through, did it leave the line (yes / no).
- **Pass:** standing: STEER stays within ±5 of the value without the lamp; driving: no ±50,
  no jump larger than ~10, stays on the line.

## 4. A dark object beside the line

- **Set up:** a dark object of the line's colour beside a straight: (a) a black bag or chair leg
  3–5 cm from the tape, (b) a short piece of the same tape (5–6 cm long) parallel to the line, 3 cm
  away, (c) a dark object touching the tape. Lock OFF, EV 0.
- **Do:** robot standing so the object is in the STEER strip, then drive past each object.
- **Look for:** the green centre line in the overlay stays on the tape; STEER does not move
  toward the object; TRACK may dip a little beside (c) and must come back once the line is clear.
- **Write down (per object):** STEER standing with and without the object, max STEER while
  passing, did the centre line jump onto the object (yes / no), lowest TRACK while passing.
- **Pass:** STEER differs by ≤ 3 from without the object; never locks onto the object;
  for (c) TRACK not below ~60 once the line is visible past the object.

## 5. A 90° corner

- **Set up:** a sharp 90° corner on the track. Lock OFF, EV 0. Use the normal Blockly values.
- **Do:** drive into the corner 3 times; then once with the lamp highlight of test 3 placed just
  before the corner.
- **Look for:** TRACK ramps down before the corner, the robot turns, STEER returns to about 0 on
  the next straight. A short ±50 when the line leaves at the side is v2 behaviour and OK. With the
  lamp: the steering goes the right way (never the opposite way).
- **Write down:** corners taken (of 3), lowest TRACK before the corner, whether ±50 appeared and for
  how long (frames / roughly in seconds), with the lamp: taken yes / no, any wrong-way steering.
- **Pass:** 3 of 3 taken (or as good as v2 was on this track); with the lamp no wrong-way steering.

## 6. Driving at about 0.6 m/s

- **Set up:** set the power so the robot does about 0.6 m/s on a straight (time it: 3 m in about
  5 s). Lock OFF, EV 0; then repeat with lock ON.
- **Do:** 2 laps each.
- **Look for:** keeps the line on curves; slows before turns (TRACK); no oscillation on straights.
- **Write down:** measured speed (m and s), laps completed, times it left the line, where; does it
  weave on straights (yes / no).
- **Pass:** 2 laps without leaving the line; no weaving on straights.

## 7. PERF numbers

- **Set up:** any of the above while tracking; logcat running.
- **Do:** let it track for 1 minute (straights and curves). Repeat once in the glare of test 3.
- **Write down:** from 3–4 `PERF` lines: `fps`, `pipe_ms`, `pipe_max_ms`, `frame_ms`; and the
  phone model.
- **Pass:** `fps` ≥ 25; `pipe_ms` ≤ 8 (a quarter of a 30 fps frame); `pipe_max_ms` ≤ 20.
  (Desktop: 0.6 ms v3 / 1.0 ms v2 per frame — a phone is expected to be several times slower.)

## Results — fill in and send back

| # | Test | Variant | Result (pass / fail) | Numbers (see "write down") | Notes |
|---|---|---|---|---|---|
| 1 | v3, lock OFF | — | | laps: · left line: · corr: · steer: · min TRACK: | |
| 2 | v3, lock ON | — | | laps: · left line: · image fixed: · lock/unlock: | |
| 3 | Glare | EV 0, lock off | | mask: · RECON/COAST: · steer min/max: · max jump: · left line: | |
| 3 | Glare | EV −1 | | mask: · RECON/COAST: · steer min/max: · max jump: · left line: | |
| 3 | Glare | EV −1 + lock | | mask: · RECON/COAST: · steer min/max: · max jump: · left line: | |
| 4 | Object beside | (a) bag / leg | | steer with/without: · max steer: · jumped: | |
| 4 | Object beside | (b) short tape | | steer with/without: · max steer: · jumped: | |
| 4 | Object beside | (c) touching | | steer with/without: · jumped: · min TRACK: | |
| 5 | 90° corner | plain | | taken: /3 · min TRACK: · ±50 (how long): | |
| 5 | 90° corner | with lamp | | taken: · wrong-way steering: | |
| 6 | ~0.6 m/s | lock off | | speed: · laps: · left line: · weaving: | |
| 6 | ~0.6 m/s | lock on | | speed: · laps: · left line: · weaving: | |
| 7 | PERF | normal | | fps: · pipe_ms: · pipe_max_ms: · frame_ms: | phone: |
| 7 | PERF | glare | | fps: · pipe_ms: · pipe_max_ms: · frame_ms: | |

Also useful: the Blockly values used (c, x, b, offset), the tape type (glossy / matte, width) and
floor, and a photo or screenshot of any failure with the HUD visible.
