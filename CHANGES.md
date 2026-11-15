# Changes

## v6.2.0: Camera offset removed (branch `offset-removal`)

Full details, measurements and the simulator report are in `OFFSET_REMOVAL.md`.
The previous round (v6.1.0 / v6.1.1, trend hold and row-wise base) is in `STEERING_IMPROVEMENTS.md`.

### What changed

The robot now steers so that the **camera** is over the line, not the robot's centre.

- Library `Config.steerRefOffsetScale` (new, default `1.0` = original): how much of the calibrated
  straight-ahead column's distance from the image centre is kept. `0` = the image centre.
  Applied once, at the end of calibration. Everything that uses the straight-ahead column
  follows it: steering, line search, edge pins, glare take-back, and the power reference.
- The heading zero (camera yaw, `headingRefDeg`) is **not** changed, so the robot runs
  **parallel** to the line, not at an angle.
- App `MainActivity`: `CAMERA_OFFSET_SCALE = 0.0f` (new), `CAMERA_OFFSET_SCALE_ORIGINAL = 1.0f`.

| Value | Original | New |
|---|---|---|
| `CAMERA_OFFSET_SCALE` → `Config.steerRefOffsetScale` | `1.0` (calibrated column) | `0.0` (image centre) |

### Why (the lecturer's reasoning)

- The camera is not in the centre of the phone. The calibrated offset steers the robot's centre
  over the line, so the line sits off-centre in the image. One side of the frame has less margin,
  and sharp turns to that side lose the line first.
- With offset 0 the line sits in the image centre, with the same margin on both sides. The
  robot's centre runs parallel to the tape, shifted by the camera's distance from the centre.
- The metal ball caster at the front no longer rides over the tape, whose height disturbs the robot.

### Margin gain

At the steering strip (frame 720 × 960, half width 360 px, about 0.032 cm per px):

| Camera offset | Offset in the image | Weaker side today → after | Stronger side today → after |
|---|---|---|---|
| 1 cm | ≈ 31 px | 91 % → 100 % (+9 %) | 109 % → 100 % |
| 2 cm | ≈ 62 px | 83 % → 100 % (+17 %) | 117 % → 100 % |
| 3 cm | ≈ 94 px | 74 % → 100 % (+26 %) | 126 % → 100 % |

The real value on the robot is not measured yet. After calibration, the overlay's dashed
reference line shows it: its distance from the image centre is the offset.

**Simulator, closed loop (not the robot):**
- Left turns (the weaker side) lose the line less often: 32–56 % fewer losses at normal speed.
- Right turns are somewhat worse.
- Straight lines are as smooth as before, once the start-up correction is done.
- With the original value, the output is byte-identical to `robot-approved-before-offset-removal`.

### How to revert

- In `MainActivity`: `CAMERA_OFFSET_SCALE = CAMERA_OFFSET_SCALE_ORIGINAL;` (1.0). Rebuild and install.
  The output is then exactly the robot-approved state.
- Or check out the tag: `git checkout robot-approved-before-offset-removal`.

### Robot test checklist

Calibrate as before: robot centre over a straight line, then wait for TRACKING.

1. **Start-up.** A small sideways correction right after calibration is **expected**: the robot
   moves 1–3 cm so the camera is over the line. It should settle within about a second.
   After calibration, the dashed reference line in the overlay is at the image centre.
2. **Straight line.** Smooth, no weaving. The robot runs **parallel** to the tape, shifted to one
   side. The ball caster does **not** ride on the tape.
   - If it weaves at speed: that is the fast loop (x = 1). Use x = 0 or a lower c, as before.
3. **Sharp left turns.** Fewer line losses than before (the camera is on the right, so the line
   was left of centre).
4. **Sharp right turns.** Still OK. They have less margin than before (the same as the left side
   now). Watch for a line loss here.
5. **Optional:** note where the dashed reference line sat **before** the change (tag
   `robot-approved-before-offset-removal`), left or right of centre and how far. That is the real
   margin gain.
6. **If anything is worse:** revert with one value (`CAMERA_OFFSET_SCALE = CAMERA_OFFSET_SCALE_ORIGINAL`),
   see above.
