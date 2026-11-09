package com.example.linefollower;

/**
 * Standalone line-following processor for the ROBOPHONE environment.
 * No Android or camera dependencies — accepts raw pixel arrays directly.
 *
 * Usage:
 *   LineFollowerPipeline pipeline = new LineFollowerPipeline();
 *   while (!pipeline.calibrateYuv420(y, u, v, width, height)) { ...next frame... }
 *   int[] result = pipeline.processFrameYuv420(y, u, v, width, height);
 *   // result[0] = Steering  [-50 … +50]    (negative = line left, positive = line right)
 *   // result[1] = Tracking  [ min … 100  ]  (100 = straight path ahead, min = hard turn / line lost)
 *
 * Three input formats are accepted (each has a calibrate… and processFrame… method):
 *   • Grayscale  — byte[] luma, one byte per pixel.
 *   • YUV 4:2:0  — full-res Y plane + half-res U and V planes (Android camera format).
 *   • ARGB       — int[] packed 0xAARRGGBB pixels.
 *
 * Any two DIFFERENT colours work — white on black, black on blue, white on brown,
 * red on green… During calibration the pipeline samples the near scan strip,
 * splits the pixels into two colour clusters (k-means), decides which is the line
 * (the floor is the cluster that fills both frame edges), and learns the single
 * colour direction that best separates them (Fisher linear discriminant). Every
 * frame is then projected onto that direction, giving a "line-ness" image in
 * which the line is bright and the floor is dark, whatever the real colours are.
 * With colour input even two colours of equal brightness (e.g. red on green) are
 * separable; with grayscale input the two colours must differ in brightness.
 *
 * Algorithm (per frame, on the line-ness image):
 *   1. Build a histogram of the Horizontal ROI and run Otsu's method (from
 *      scratch — no OpenCV) to pick the threshold that best separates the track
 *      from the floor under the current lighting.
 *   2. Threshold: pixels on the track side of (Otsu ± bias) are "track".
 *   3. Line-lock: collapse the H-ROI to a per-column track profile and select the
 *      strongest contiguous BAND. With track memory on, only a gate around the
 *      band's predicted position is searched, so reflections elsewhere are ignored
 *      and the band is followed to the frame edge. When the band finally leaves the
 *      gate at the frame edge, steering pins to that edge and a new line is
 *      re-acquired (full-width) only after a short grace period; if it vanishes
 *      mid-image (hidden) the last output is held for a few frames instead.
 *      Distractor rejection: a band only counts as the line if its width and
 *      "line-ness" match what calibration learned about the line; a line merged
 *      with a dark object or shadow is cut back to the part that continues it.
 *   4. V-ROI look-ahead: the line is followed row by row up the frame (same width
 *      rules, so the tracker cannot step onto a nearby object) → tracking value
 *      (100 = straight path ahead) and the line's heading on the floor.
 *   5. Steering = H-ROI offset + heading part (pure-pursuit target point), internal
 *      [-100 … +100], output scaled to [-50 … +50].
 *
 * All per-frame buffers are pre-allocated in init(); the processFrame methods
 * perform zero heap allocations.
 */
public final class LineFollowerPipeline {

    // =========================================================================
    //  Configuration
    // =========================================================================

    public static final class Config {

        // ── Colours ─────────────────────────────────────────────────────────
        // When ON (default) the line and floor colours are LEARNED during
        // calibration, so any two different colours work and trackIsBright is
        // ignored. Turn OFF for the classic grayscale mode, where the brightness
        // channel is thresholded directly and trackIsBright picks the polarity.
        // (Alternatively call setTrackColors() to give the two colours yourself.)
        public boolean autoColor = true;

        // Only used when autoColor == false (or before the colours are learned):
        // true  = BRIGHT line on DARK floor
        // false = DARK   line on BRIGHT floor
        public boolean trackIsBright = true;

        // Number of calibration frames sampled to learn the two colours. More =
        // more robust against a single noisy frame, slightly slower startup.
        public int colorCalibFrames = 5;

        // Minimum separation between the learned line and floor colours, in units
        // of their pooled standard deviation (Mahalanobis distance). Below this the
        // two clusters are just noise or lighting on one surface, not a line on a
        // floor, so the colour samples are discarded and calibration retries.
        public float colorMinSeparation = 4.0f;

        // Minimum plain distance between the two learned colours (grey-levels in
        // YUV space). Rejects "two colours" that are really sensor noise on a
        // perfectly uniform surface.
        public float colorMinDistance = 12f;

        // Optional bias applied on top of the Otsu threshold (in grey-levels).
        // Otsu self-calibrates to the lighting, so 0 is the right default. Nudge
        // positive to reject more floor (fewer false positives), negative if the
        // line is being missed. The bias is added toward the track side.
        public int thresholdMargin = 0;

        // Minimum difference (grey-levels) between the mean of the two classes Otsu
        // splits a region into. Otsu ALWAYS finds a split — on a plain floor with no
        // line it just splits the sensor noise — so below this contrast the region
        // is treated as containing no line at all. Set to 0 to disable.
        public int minContrast = 24;

        // In learned-colour mode the floor colour maps to 64 and the line colour to
        // 192 on the line-ness image. The threshold is never allowed below this
        // value, so floor pixels (even noisy ones) cannot be taken for the line.
        public int colorMinThreshold = 112;

        // When ON, the look-ahead (V-ROI) gets its OWN Otsu threshold instead of
        // reusing the H-ROI one. The far look-ahead line is dimmer than the near
        // line (perspective + lighting falloff), so the near-tuned threshold can
        // miss its far end — the tracker then loses the line early and the track
        // value plateaus (never reaches 100 on a hard curve). A second Otsu computed
        // from a subsampled V-ROI histogram is matched to the look-ahead's own
        // lighting, so the dim far line is detected and tracked deeper into the turn.
        // Costs ~0.1 ms/frame (subsampled, zero-allocation). Turn OFF to fall back
        // to a single shared H-ROI threshold for both stages.
        public boolean vRoiAdaptiveThreshold = true;

        // H-ROI: where the horizontal scan strip sits (fraction of frame height).
        // 0.85 = 85 % from the top, i.e. near the bottom of the frame.
        public float hRoiCenterYFrac = 0.85f;
        public int   hRoiThickPx    = 24;       // strip height in pixels

        // Fraction of the frame BOTTOM occupied by the robot's own body (chassis,
        // wheels) once the phone is fully seated in the mount. The H-ROI (base of
        // the ⊥) is kept entirely ABOVE this band, so robot parts never enter the
        // Otsu / centroid math. 0 = nothing ignored; raise until the robot is fully
        // excluded. Only pulls the strip up when it would otherwise overlap.
        public float bottomIgnoreFrac = 0.15f;

        // V-ROI: how wide the vertical column is (pixels each side of centroid).
        // Keep this wider than the road line so the line stays inside even on curves.
        public int vRoiHalfWidthPx = 80;

        // V-ROI look-ahead HEIGHT: how far up the frame (as a fraction of frame
        // height) the look-ahead reaches ABOVE the H-ROI. The far top of the frame
        // is dim and noisy, which made the track value jumpy, so we stop short of it.
        // 0.5 = look half a frame ahead of the robot. Smaller = shorter & steadier
        // (but less warning of far turns); larger = looks farther but noisier.
        public float vRoiHeightFrac = 0.5f;

        // Minimum fraction of H-ROI pixels that must be "track" to trust centroid.
        public float minWeightFrac = 0.01f;

        // Tilt angle (degrees, from vertical) at which the straightness/curvature
        // signal reaches 100. Smaller = more sensitive to rotation; larger = the
        // value only climbs for harder turns. 30° is a responsive default.
        public float curveFullScaleDeg = 30f;

        // ── Perspective / mounting geometry ─────────────────────────────────
        // The phone is on a fixed mount tilted so its optical axis is depressed
        // this many degrees below horizontal (45° = halfway between straight
        // ahead and straight down). Together with the camera's vertical field of
        // view this fixes where the HORIZON (vanishing line of the floor) sits in
        // the image — and a flat track of constant real width projects narrower
        // the closer a row is to that horizon. We use this to taper the look-ahead
        // geometry so "narrower farther away" is handled instead of assumed flat.
        public float cameraTiltDeg  = 45f;
        // Approximate vertical field of view of the camera in degrees. Most phone
        // back cameras are ~55–65° vertical; 60° is a safe default. Tune if known.
        public float verticalFovDeg = 60f;

        // ── Track memory / continuity ───────────────────────────────────────
        // When ON, the pipeline LOCKS onto the band it is currently on and, on
        // every later frame, only looks for the track inside a "gate" window
        // around its last known position. Light reflections or stray spots
        // OUTSIDE that gate can no longer pull the centroid, so steering follows
        // the real line all the way to the frame edge. The lock is only released
        // (and the whole width searched again for a NEW line) after the track has
        // been gone for trackLostGraceFrames — i.e. once you have moved fully off
        // it. Turn OFF to fall back to a memoryless full-width search every frame.
        public boolean trackMemory = true;

        // Gate half-width = trackMemoryGateMult × the measured track width, with a
        // floor of trackMemoryMinGatePx. Wider = tolerates faster sideways motion
        // but lets nearer reflections in; tighter = stricter rejection. 3× the line
        // width is a good balance for 30 fps.
        public float trackMemoryGateMult  = 3.0f;
        public int   trackMemoryMinGatePx = 48;

        // While the locked line is missing from the gate, steering is pinned to the
        // edge it left (±100) and the turn signal is forced to 100 %. After this
        // many consecutive missing frames the lock is dropped and a full-width
        // search re-acquires a new line. ~8 frames ≈ 0.25 s at 30 fps.
        public int trackLostGraceFrames = 8;

        // Lock correction. If the lock ever ends up on the wrong thing (e.g. after
        // the line was lost in a turn and a reflection or a neighbouring tape was
        // grabbed), the pipeline keeps checking the whole strip: when a band that
        // looks much more like the line (near the calibrated straight-ahead column,
        // with the calibrated line width) is seen outside the lock for this many
        // consecutive frames, the lock moves to it. 0 = never correct the lock.
        public int trackRelockFrames = 6;

        // ── Distractor rejection (dark chair, bag, shadow, table edge …) ─────
        // Objects with a colour close to the line's pass the threshold just like
        // the line. When ON, a candidate band only counts as the line if it LOOKS
        // like the line learned at calibration: its width (corrected for the
        // line's current slant in the image) and how line-like its pixels are.
        // A band that is too wide (the line merged with a blob or shadow) is cut
        // back to the part that continues the line from the previous frame, and
        // the look-ahead tracker applies the same width test row by row. Turn OFF
        // for the original (v1) band selection.
        public boolean rejectDistractors = true;

        // Widest band still accepted as the line, as a multiple of the expected
        // (calibrated, slant-corrected) line width. A real line stays within
        // about ±30 %; a line merged with a blob or shadow is at least ~2.5×.
        public float bandMaxWidthMult = 2.0f;

        // Narrowest band accepted as the line, as a fraction of the calibrated
        // width (bands cut by the frame edge are exempt). Rejects specks / legs
        // seen end-on that are much thinner than the tape.
        public float bandMinWidthMult = 0.3f;

        // When the locked line disappears in the MIDDLE of the image (hidden by a
        // shadow, glare or an object passing over it) the last steering and
        // tracking values are HELD for up to this many frames instead of jumping
        // to a full turn. A line that leaves at the frame EDGE (a real turn) is
        // still pinned to that edge at once. 6 frames = 0.1 s at 60 Hz.
        public int trackHoldFrames = 6;

        // ── Glare robustness (v3) ───────────────────────────────────────────
        // Strong light (a lamp or a window reflected by glossy tape or floor) saturates pixels:
        // they no longer show their colour, so they can be read neither as line nor as floor.
        // With glareMask ON such pixels are MASKED ("unknown"): they are left out of every
        // threshold histogram, and inside a band they count as part of it, so a highlight on
        // the tape centre does not split the line and a fully saturated piece of tape of the
        // right width is still found. In addition, with glareMask ON:
        //   • if glare hides the line in the scan strip, the line is followed from the look-ahead
        //     (the base of the ⊥ is reconstructed from the line ahead — FrameData.baseReconstructed);
        //   • look-ahead rows hidden by glare are skipped, not taken for the end of the road;
        //   • a line hidden completely in the middle of the image is coasted on for up to
        //     glareCoastMs (instead of the trackHoldFrames hold);
        //   • the scan strip gets one Otsu threshold per eighth of its width.
        // OFF = exactly the v2 behaviour (including the trackHoldFrames hold).
        public boolean glareMask = true;

        // A pixel whose brightness (Y, 0–255) is at or above this is masked as glare.
        // ARGB input: a pixel with any of R, G, B ≥ 250 is masked.
        public int glareLumaMax = 245;

        // Longest time (ms) the line is coasted on while it is hidden in the MIDDLE of the image
        // (glareMask only). While coasting the line position continues its last motion, the
        // heading part of the steering fades out and the tracking value ramps down toward the
        // minimum, so the robot slows while it cannot see. A line that leaves at the frame edge,
        // or that the look-ahead shows leaving through the edge, is pinned to that edge at once
        // and never coasted. At 0.6 m/s, 600 ms is about 36 cm of blind driving. 0 = no coasting.
        public float glareCoastMs = 600f;

        // Backup limit for the coast: it also ends after this many frames, whatever the clock
        // says (36 = 600 ms at 60 fps).
        public int glareCoastMaxFrames = 36;

        // Time between frames used by the coast clock, in ms. 0 (default) = measured with
        // System.nanoTime(). Set it for repeatable results (tests, simulation, replay). Either way
        // one frame counts at most 100 ms, so a pause or a dropped frame cannot use up the coast
        // in one step.
        public float frameIntervalMs = 0f;

        // ── Corridor tracker (v3) ───────────────────────────────────────────
        // When ON, a locked line is followed by a curve on the FLOOR, x = A + B·u + C·u²
        // (u = distance beyond the scan strip, x = sideways position): every frame the line is
        // looked for in corridorSlices thin slices along a corridor around the predicted curve,
        // from the scan strip up to the far end of the look-ahead. Each slice needs both edges
        // of the tape (floor→line and line→floor) at the expected width, so a highlight on the
        // tape centre does not split it and a shadow without sharp edges is not taken for it.
        // The slices update a small Kalman filter of the curve: the steering offset comes from
        // A, the heading from B. If glare hides the near part, the line seen farther ahead
        // still updates the heading and curvature while the near position comes from the
        // filter's prediction. Acquisition, sharp corners, junctions, a line leaving the
        // frame and frames with too little evidence use the v2 logic. OFF = v2 tracking.
        public boolean corridorMode = true;

        // Number of slices from the scan strip to the far end of the look-ahead (4 … 32).
        public int corridorSlices = 12;

        // Corridor width per slice, as a multiple of the expected (perspective-scaled) line
        // width. It is never narrower than the line plus a margin, so both tape edges are seen.
        public float corridorWidthMult = 2.2f;

        // Slices on the line needed (when the near part is visible) to use the corridor result.
        public int minChainSlices = 4;

        // Below this confidence (coverage × fit quality, 0 … 1) the frame uses the v2 logic.
        public float corridorMinConfidence = 0.35f;

        // Scale of the Kalman process noise (how fast the curve may change between frames).
        // Larger = follows sudden changes sooner, smooths less.
        public float corridorProcessNoise = 1.0f;

        // Distractor hardening (v3, active with corridorMode): a candidate band in the scan
        // strip must show a sharp floor→line AND line→floor edge at its width — a shadow or a
        // blob with soft edges only counts a third as much as a band with both edges (it is
        // not rejected outright, so a blurred image still works). In addition, with
        // corridorMode: a band cut out of a wider one is only accepted if the line continues
        // up the look-ahead, the look-ahead does not count a trailing stretch of such cut-out
        // rows as visible road, and when (re)acquiring the line the candidate that continues
        // farthest up the look-ahead wins. OFF = only the edge check is skipped.
        public boolean edgePairCheck = true;

        // ── Steering from the whole ⊥ (offset + heading) ─────────────────────
        // Steering = offset part + heading part:
        //   offset  = steerOffsetWeight × (line column in the H-ROI − calibrated
        //             column) / half the frame width            (v1 steering)
        //   heading = steerHeadingWeight × L × tan(line heading) / (half the
        //             floor width seen by the H-ROI)
        // The line heading is the direction of the line where it crosses the
        // H-ROI, from a least-squares parabola through the H-ROI centroid and the
        // near look-ahead rows, projected onto the floor with the camera geometry
        // (cameraTiltDeg, verticalFovDeg) so perspective does not bend it, minus
        // the heading seen at calibration. Together the two parts are the lateral
        // position of a pure-pursuit target L further along the line, in the same
        // units as the v1 steering, so a robot correctly placed on a straight line
        // still reads 0. steerHeadingWeight = 0 gives exactly the v1 steering.
        public float steerOffsetWeight  = 1.0f;
        public float steerHeadingWeight = 1.0f;

        // L, the look-ahead distance of the target point, as a multiple of the
        // floor distance from the camera to the H-ROI. Larger = the heading counts
        // more: a robot at an angle to the line straightens out sooner, but on
        // tight curves the robot runs a little more on the inside, and with a high
        // steering gain plus delay it can start to weave. 0.5 was the best balance
        // in the closed-loop simulation (see _investigation/CHANGES_v2.md).
        public float steerLookAhead = 0.5f;

        // Minimum number of look-ahead rows on the line (plus the H-ROI) for the
        // heading to be used at full weight. With fewer (end of the line, a very
        // sharp turn, line leaving the frame) the heading part fades out and the
        // steering falls back to the offset only.
        public int steerHeadingMinRows = 24;

        // The line heading seen during calibration is the heading zero (phone
        // yaw on its mount). It is ignored if larger than this many degrees, as
        // the robot was then clearly not placed straight on a straight line.
        public float steerHeadingCalibMaxDeg = 10f;

        // ── Track value (look-ahead position) calibration ───────────────────
        // The tracking output is derived from STEERING applied to the LOOK-AHEAD —
        // the V-ROI in front of the robot — using the same band-lock memory as
        // steering. Internally it reads 0 when the upcoming line sits straight
        // ahead, rises as the line ahead leans to the side (an upcoming turn), and
        // hits 100 once it reaches the frame edge or is lost; the output is that
        // value inverted (100 = straight). trackOffsetFullFrac sets how far the
        // look-ahead line must lean, as a fraction of the half-frame width, to
        // read a full turn: 1.0 = only at the very frame edge; lower it (e.g. 0.6)
        // to make turns register sooner.
        public float trackOffsetFullFrac = 1.0f;

        // Road-length slowdown. The tracking value is ALSO limited by how much of
        // the line is still visible ahead in the look-ahead, measured as real floor
        // distance (perspective-corrected from cameraTiltDeg / verticalFovDeg). So
        // when the line ends (dead end) or stops at a sharp corner, tracking ramps
        // down linearly with the remaining road instead of dropping from 100 to the
        // minimum in one frame. Tracking is the lower of this and the lean-based
        // value, so a straight, fully visible road still reads 100.
        public boolean trackLengthSlowdown = true;

        // Fraction of the look-ahead's floor distance the line must reach for the
        // length part to read 100. Slightly below 1 so the dim, noisy far edge of
        // the look-ahead never pulls tracking down on a straight road.
        public float trackLengthFullFrac = 0.8f;

        // Minimum value the Tracking output is allowed to reach (default 5).
        // Prevents the robot from receiving a full-stop signal, which can cause
        // mechanical bugs. Set to 0 to allow the full 0-100 range.
        public int trackingMinValue = 5;

        // ── Auto-calibration (camera mounting offset) ───────────────────────
        // The phone camera is not on the robot's centre line, so when the robot is
        // correctly placed on the road the NEAR line does NOT sit at image centre.
        // We ASSUME the robot starts correctly placed and, over the first few solid
        // detections, record where the near line (steering) actually sits — that
        // becomes the steering zero reference, so a correctly placed robot reads
        // steering 0. The look-ahead (track) zero is NOT measured from the road
        // (on a curve the road ahead genuinely leans, so measuring it would wrongly
        // flatten the turn to 0); instead it is DERIVED from the steering offset
        // through the camera's perspective geometry, capturing only the mounting
        // offset. The steering reference is taken as the MEDIAN of this many solid
        // detections (more = steadier / more outlier-proof capture, slower startup).
        public int calibFrames = 10;
    }

    // =========================================================================
    //  Calibration phase / issue codes (reported in FrameData)
    // =========================================================================

    /** Calibration phase: sampling the scan strip to learn the line and floor colours. */
    public static final int CALIB_LEARNING_COLORS = 0;
    /** Calibration phase: measuring where the line sits (steering zero reference). */
    public static final int CALIB_FINDING_CENTER  = 1;
    /** Calibration complete — processFrame is ready. */
    public static final int CALIB_DONE            = 2;

    /** No calibration problem. */
    public static final int ISSUE_NONE         = 0;
    /** The two colours in view are too similar (or the view is one uniform surface). Retrying. */
    public static final int ISSUE_LOW_CONTRAST = 1;
    /** No colour fills both frame edges, so the floor could not be told apart from the line. Retrying. */
    public static final int ISSUE_NO_FLOOR     = 2;
    /** The colours are learned but no solid line band was found in the scan strip this frame. */
    public static final int ISSUE_NO_LINE      = 3;

    // =========================================================================
    //  Internal output holder (not part of the public API)
    // =========================================================================

    private static final class ControlOutput {
        float   steering     = 0f;
        float   intersection = 0f;
        int     centroidX    = -1;
        boolean trackFound   = false;
        float   tiltDeg      = 0f;
        float   trackWidthPx = 0f;
        int     threshold    = 128;
        float   offsetTerm   = 0f;    // internal units [-100 … 100]
        float   headingTerm  = 0f;    // internal units
        float   headingDeg   = 0f;
        float   headingConf  = 0f;
        boolean held         = false;
        int     rejected     = 0;
        boolean coasting     = false;
        boolean reconstructed = false;
    }

    // =========================================================================
    //  Callback interface — implement this to receive overlay geometry each frame
    // =========================================================================

    /**
     * Implement this interface and register it via {@link #setFrameCallback} to
     * receive geometry and state data on every calibration frame and every
     * processFrame call — everything needed to draw a dynamic road overlay.
     * The callback runs on the thread that called calibrate / processFrame.
     */
    public interface FrameCallback {
        void onFrame(FrameData data);
    }

    /**
     * All geometry and state data needed to draw a visual overlay.
     * The same instance is reused every call — copy any fields you need to keep
     * beyond the current frame.
     */
    public static final class FrameData {
        /** Frame width in pixels (the coordinate system all fractions refer to). */
        public int imgW;
        /** Frame height in pixels. */
        public int imgH;
        /** H-ROI top edge as a fraction of frame height. */
        public float hRoiTopFrac;
        /** H-ROI bottom edge as a fraction of frame height. */
        public float hRoiBotFrac;
        /** V-ROI top (far) edge as a fraction of frame height. */
        public float vRoiTopFrac;
        /** Top of the ignored robot-body band as a fraction of frame height (1 = none). */
        public float ignoreTopFrac;
        /** Calibrated straight-ahead column for steering, fraction of frame width. */
        public float steerRefXFrac;
        /** Calibrated straight-ahead column for the look-ahead, fraction of frame width. */
        public float trackRefXFrac;
        /** Near (bottom) half-width of the V-ROI box, fraction of frame width. */
        public float halfBotFrac;
        /** Far (top) half-width of the V-ROI box (tapered by perspective), fraction of frame width. */
        public float halfTopFrac;
        /** Detected near-line position as a fraction of frame width; -1 if not found. */
        public float centroidXFrac;
        /** Measured line width in the H-ROI as a fraction of frame width. */
        public float trackWidthFrac;
        /** Tilt of the look-ahead line from vertical, in degrees (positive = leans right). */
        public float tiltDeg;
        /** Dynamic road poly-line column positions (fraction of frame width), nearest to farthest. */
        public final float[] lookAheadColFrac = new float[LA_MAX_PTS];
        /** Dynamic road poly-line row positions (fraction of frame height), nearest to farthest. */
        public final float[] lookAheadRowFrac = new float[LA_MAX_PTS];
        /** Number of valid points in the look-ahead path arrays (0 = none). */
        public int lookAheadPointCount;
        /** True when the line was detected in the H-ROI this frame. */
        public boolean trackFound;
        /** True once calibration is complete. */
        public boolean isCalibrated;
        /** Calibration phase: CALIB_LEARNING_COLORS, CALIB_FINDING_CENTER or CALIB_DONE. */
        public int calibPhase;
        /** Overall calibration progress [0 … 1]. */
        public float calibProgress;
        /** Most recent calibration problem (ISSUE_* constant), ISSUE_NONE if none. */
        public int calibIssue;
        /** True when the frames are being processed in learned-colour mode. */
        public boolean colorModelReady;
        /** Learned line colour (0xAARRGGBB); 0 until learned. */
        public int lineColorArgb;
        /** Learned floor colour (0xAARRGGBB); 0 until learned. */
        public int floorColorArgb;
        /** Separation of the learned colours in pooled standard deviations (0 until learned). */
        public float colorSeparation;
        /** Threshold used in the H-ROI this frame (on the line-ness / grey image, 0–255); -1 = no contrast, no line. */
        public int threshold;
        /** Steering value [-50 … +50]; negative = line left, positive = line right. */
        public int steering;
        /** Tracking value [min … 100]; 100 = straight road ahead, low = turn / line lost. */
        public int tracking;
        /** Offset part of the steering (H-ROI position), in steering output units. */
        public float steerOffsetTerm;
        /** Heading part of the steering (line direction from the whole ⊥), in steering output units. */
        public float steerHeadingTerm;
        /** Line heading relative to the robot on the floor, perspective-corrected, minus the
         *  calibrated zero; degrees, positive = line heads to the right. 0 when unknown. */
        public float headingDeg;
        /** Weight [0 … 1] the heading got this frame (0 = too little look-ahead, offset only). */
        public float headingConfidence;
        /** True when the line is momentarily hidden and the last values are being held. */
        public boolean lineHeld;
        /** Number of candidate bands rejected or trimmed as distractors this frame (H-ROI). */
        public int rejectedBands;
        /** True when the line was hidden in the scan strip (glare) and its position there was
         *  reconstructed from the look-ahead; trackFound is then true. (glareMask) */
        public boolean baseReconstructed;
        /** True while the line is hidden mid-image and the pipeline coasts on its last motion
         *  (glareMask; lineHeld is also true). trackFound is false. */
        public boolean coasting;
        /** Fraction [0 … 1] of the pixels in the scan strip + look-ahead masked as glare. */
        public float maskedFrac;
        /** Corridor tracker: fraction [0 … 1] of the near slices on the line this frame
         *  (0 when the frame used the v2 logic). */
        public float corridorCoverage;
        /** Corridor tracker: RMS distance of the slices from the fitted curve, in line widths. */
        public float corridorFitRms;
        /** Corridor curve on the floor, x = curveA + curveB·u + curveC·u² (camera heights; u =
         *  forward distance beyond the scan strip, x = sideways, + = right). 0 when not used. */
        public float curveA, curveB, curveC;
    }

    // =========================================================================
    //  State
    // =========================================================================

    private Config  cfg         = new Config();   // default config until configure() is called
    private boolean initialized;

    private int imgW, imgH;
    private int hRoiTop, hRoiBot;  // H-ROI row range [hRoiTop, hRoiBot)
    private int vRoiTop, vRoiBot;  // V-ROI row range [vRoiTop, vRoiBot)
    private int ignoreTop;         // first row of the ignored robot-body band
    private int lastCentroidX;
    private float lastTrackWidthPx; // most recent measured track width (held on loss)

    // ── Current frame (bound by the public calibrate / processFrame methods) ──
    private static final int FMT_GRAY = 0, FMT_YUV420 = 1, FMT_ARGB = 2;
    private int    srcFormat;
    private byte[] srcY, srcU, srcV;   // Y full-res; U/V packed half-res planes
    private int[]  srcArgb;
    private int    uvW;                // width of the packed chroma planes

    // Line-ness work image (frame-sized, filled only on the rows actually used).
    private byte[]  work;
    // Polarity of the image returned by prepareWorkImage(): true = track is bright.
    private boolean workBright;

    // ── Track memory (line-lock) state ──────────────────────────────────────
    // trackAcquired: we currently hold a lock on a specific band and only search
    //   a gate around lastCentroidX. lostFrames: consecutive frames the locked
    //   line has been absent from the gate; once it exceeds the configured grace
    //   the lock is dropped and the next frame searches the full width.
    private boolean trackAcquired;
    private int     lostFrames;

    // ── Auto-calibration state (camera mounting offset) ─────────────────────
    // steerRefX / trackRefX are the image columns that count as "straight ahead"
    // for the near (steering) and look-ahead (track) measurements — captured once
    // from the median of the first calibFrames solid detections, assuming the
    // robot starts correctly placed. Until then they default to image centre.
    private boolean calibrated;
    private int     calibCount;
    private int[]   calibSteerSamples;   // near-line centroids gathered for the median
    private float   steerRefX, trackRefX;
    private int     calibIssue;
    private int[]   calibWidthSamples;   // near-line band widths gathered for the median
    // Typical line width in the H-ROI (px): the median measured during calibration
    // (0 = unknown). Used to recognise the real line when (re-)acquiring.
    private float   calibWidthPx;
    // Where colour learning saw the line band (column, width in px; 0 = unknown).
    // Guides the first line search in calibration phase 2.
    private float   learnedLineX, learnedLineW;
    // Consecutive frames a better line candidate was seen outside the line lock.
    private int     relockCount;
    // Typical "line-ness" of line pixels in the H-ROI (0–255, track side = high),
    // median over calibration (0 = unknown). A band whose pixels only just pass
    // the threshold (shadowed floor) is less line-like than the real line.
    private float   calibLevel;
    private int[]   calibLevelSamples;
    // Line heading (deg) seen during calibration = heading zero; samples for its median.
    private float   headingRefDeg;
    private float[] calibHeadingSamples;
    private int     calibHeadingCount;

    // ── Distractor rejection / short-term prediction ────────────────────────
    // centroidVel: smoothed per-frame motion of the line in the H-ROI (px/frame),
    //   so the gate and the band preference follow the PREDICTED position.
    // lastImgSlope: image slope d(col)/d(row) of the line near the H-ROI, from the
    //   previous frame; a slanted line crosses the strip as a wider band.
    // holdFrames: consecutive frames the line has been held (hidden mid-image).
    private float   centroidVel;
    private float   lastImgSlope;
    private int     holdFrames;
    private float   lastSteering, lastIntersection;
    private int     rejectedBands;
    private boolean bandTrimmed;     // set by findBand: the chosen band was cut out of a wider one

    // ── Colour model (learned during calibration, or preset) ────────────────
    // The line-ness value of a pixel is a linear function of its colour:
    //     f = (kY·Y + kU·U + kV·V + k0) >> 10        (YUV / grayscale input)
    //     f = (kR·R + kG·G + kB·B + kC) >> 10        (ARGB input, same function)
    // scaled so the floor colour maps to ~64 and the line colour to ~192.
    private static final int SAMPLES_PER_FRAME = 2048;
    private boolean colorReady;
    private boolean colorPreset;        // set via setTrackColors(); survives re-init
    private int     colorFramesSampled;
    private int     sampleCount;
    private short[] sampY, sampU, sampV, sampCol;
    private byte[]  sampLabel;
    private int     kY, kU, kV, k0, kR, kG, kB, kC, kGrayOff;
    private int     lineColorArgb, floorColorArgb;
    private float   colorSeparation;
    // Preset model kept so init() can restore it.
    private float   presetY0, presetU0, presetV0, presetY1, presetU1, presetV1;

    // ── Dynamic look-ahead path (for the overlay to follow the road) ────────
    // A small poly-line of the detected line's centre, sampled from nearest to
    // farthest across the V-ROI, in FRACTIONAL image coordinates. Rebuilt each
    // frame from the same band-locked tracking that drives the track value, so
    // the overlay bends along the tape instead of drawing a straight box.
    private static final int LA_MAX_PTS = 48;
    private final float[] laPathColFrac = new float[LA_MAX_PTS];
    private final float[] laPathRowFrac = new float[LA_MAX_PTS];
    private int laPathCount;

    // Apparent-width ratio between the (far-weighted) look-ahead row and the near
    // H-ROI row. A fixed real lateral offset — the camera's mount offset — projects
    // to a pixel offset that scales with apparent width, so this ratio maps the
    // steering offset to the look-ahead's straight-ahead reference (camera offset
    // only, no road curvature). Computed once in init() from the mount geometry.
    private float laPerspectiveRatio = 1f;
    private float lookAheadRowFrac   = 0.5f;

    // Image row (fractional, 0=top … 1=bottom) of the floor's horizon / vanishing
    // line, derived from the mount tilt. May be negative (horizon above the frame),
    // which is expected for a steep downward tilt like 45°.
    private float horizonYFrac;

    // ── Frames-per-second measurement (perf tracking) ───────────────────────
    // Counts frames within each rolling 1-second wall-clock window; the count
    // from the most recently COMPLETED second is exposed as the current FPS.
    private long fpsWindowStartNs;   // start timestamp of the in-progress window
    private int  fpsFrameCount;      // frames seen so far in the in-progress window
    private int  framesPerSecond;    // frames counted in the last completed second
    private long totalFrames;        // cumulative frames processed since init()
    private long firstFrameNs;       // timestamp of the very first processed frame
    private long elapsedNs;          // wall-clock nanos since the first frame

    // Pre-allocated 256-bin histogram, reused every frame (no GC churn).
    private final int[] histogram = new int[256];

    // Second histogram for the look-ahead (V-ROI) Otsu — the far region is dimmer,
    // so it gets its own threshold. Reused every frame (no allocation).
    private final int[] histogramV = new int[256];

    // Class-mean difference of the most recent otsuThreshold() split.
    private float otsuContrast;

    // Pre-allocated per-row centroid buffer for the V-ROI tilt fit (sized in init).
    private float[] rowCx;

    // Floor geometry per image row (exact pinhole model from cameraTiltDeg and
    // verticalFovDeg, sized in init). A pixel (col, row) sees the floor point
    //   forward = rowFwd[row],   lateral = rowScale[row] × (col + 0.5 − W/2)
    // in units of the camera height (lateral positive = right). rowScale also
    // gives the apparent width of the line: width_px ∝ 1 / rowScale.
    private float[] rowFwd, rowScale;
    private float   floorFwdH, floorScaleH;   // the same at the H-ROI centre row

    // Least-squares sums of the line on the FLOOR (lateral vs forward), filled by
    // traceLookAhead() together with the image-space fit.
    private int     floorN;
    // Sums over u = forward − (forward at the H-ROI) for a parabola fit x = A + B·u + C·u².
    private double  fU, fUU, fUUU, fUUUU, fX, fUX, fUUX, fXX;
    private float   fUmin, fUmax;
    private float   floorSpan;     // forward extent of the fitted points (camera heights)
    // Heading of the last trace (deg, before the calibrated zero) and its weight [0 … 1].
    private float   fitHeadingDeg, fitConfidence;
    private double  fitC;          // curvature term C of the last heading fit (0 = straight fit)
    // Only the line up to this many H-ROI distances beyond the H-ROI is fitted: the
    // heading is wanted near the robot, and far rows may already be in the next turn.
    private static final float FIT_DEPTH = 1.5f;

    // Look-ahead trace results (set by traceLookAhead).
    private int     traceFitN, traceFarRow, traceLaCount;
    private double  traceSlope, traceNearSlope, traceLaW, traceLaM;

    // Pre-allocated per-column "track pixel count" profile of the H-ROI (sized in
    // init). Lets Stage 2 isolate the strongest contiguous BAND instead of
    // averaging every track pixel, which is what makes the line-lock possible.
    private int[] colWeight;
    // Per-column sum of the line-ness of those track pixels (same strip), so the
    // mean line-ness of any band is colLevel / colWeight summed over its columns.
    private int[] colLevel;

    // Result of the most recent findBand() call.
    private long   bandSum;
    private double bandMoment;
    private int    bandStart, bandEnd;   // [bandStart, bandEnd) columns; -1 if none
    private float  bandScore;

    // ── Glare handling (v3, Config.glareMask) ───────────────────────────────
    // maskActive: the glare mask is in use for the current frame (processing only, never
    //   during calibration, which assumes a clean view).
    // glareMap: ARGB input only — 1 = saturated pixel, on the rows prepared for the current
    //   frame (gray / YUV input test Y directly, see setupGlareSource).
    // colMask: per column, masked pixels in the H-ROI; colThr: per column, the local H-ROI
    //   threshold (one Otsu per THR_SEGMENTS-th of the width, interpolated).
    private static final int THR_SEGMENTS = 8;
    private static final int GLARE_CHANNEL_MAX = 250;   // ARGB: any channel at/above this = glare
    private boolean maskActive;
    private byte[]  glareMap;
    private int[]   colMask, colThr;
    private byte[]  glareSrc;          // glare test source: Y plane (gray / YUV) or glareMap (ARGB)
    private int     glareLim;          // ... a pixel is glare when (glareSrc[i] & 0xFF) >= glareLim
    private int     maskedSamples;     // look-ahead pixels sampled for the masked fraction
    private int     maskedSampleHits;  // ... of which masked
    private final int[] segHist = new int[THR_SEGMENTS * 256];
    private final int[] segThr  = new int[THR_SEGMENTS];
    private int     maskedPixels;      // masked pixels in the H-ROI (exact)
    // Coast clock and state: frameDtMs = time since the previous processed frame (clamped);
    // coastMs / coastFrames = how much of the coast budget the current hidden spell has used.
    private static final float MAX_FRAME_DT_MS = 100f;
    private long    lastFrameNs;
    private float   frameDtMs;
    private float   coastMs;
    private int     coastFrames;
    private float   lastHeadTerm;      // heading part of the steering on the last frame with the line
    private boolean glarePinned;       // pinned while the strip was hidden (lock parked at the edge)
    // Phase 3: the frame being processed (for continuity / edge checks inside findBand).
    private byte[]  curPixels;
    private boolean curBright;
    private boolean hardening;         // distractor hardening active this frame (corridorMode, calibrated)
    // Capped edge pin (plan §12): curve history of the corridor filter and the cap's clock.
    private int     curveSide;         // side (-1 / +1) the filter's curve bends to (0 = straight)
    private int     curveFrames;       // consecutive confident corridor frames bending to curveSide
    private int     sinceCurveFrames = 1000;   // frames since the last such frame
    private float   pinCapMs;          // time spent in a capped pin
    private int     pinCapFrames;
    private boolean pinCapEscalated;   // cap time used up: full pin until the line is seen
    private boolean pinCapActive;      // this frame's pin was capped
    private int     glarePinSide;      // ... to this side (-1 / +1)
    private int     traceFirstRow;     // nearest look-ahead row on the line in the last trace
    private int     traceCutRow = -1;  // D2: first row of a dropped trailing cut stretch (-1 = none)
    // Extra look-ahead results (seeded trace without a base, glare rows).
    private int     traceEdgeSide;     // -1 / +1: nearest line row runs off the left / right frame edge
    private int     traceAccepted;     // look-ahead rows on the line
    // Linear floor fit over the NEAREST part of the trace (for reconstructing the base).
    private int     nfN;
    private double  nfU, nfUU, nfX, nfUX, nfXX, nfLimit;

    private final ControlOutput output = new ControlOutput();

    // Reused output array — no allocation on the hot path.
    private final int[]      result    = new int[2];

    // Callback registered by the caller — null if not set.
    private FrameCallback    frameCallback;
    // Pre-allocated FrameData — filled and passed to the callback each frame.
    private final FrameData  frameData = new FrameData();
    // True while fireCallback runs for a processFrame call (not a calibration frame).
    private boolean          isProcessing;

    // =========================================================================
    //  Public API — setup
    // =========================================================================

    /**
     * Register a callback to receive geometry and state data on every frame.
     * Pass null to unregister. Safe to call at any time.
     *
     * The callback fires after every calibrate call and after every processFrame
     * call, so an overlay can be driven from a single listener without polling
     * any getters manually.
     *
     * @param callback  implementation of {@link FrameCallback}, or null to remove
     */
    public void setFrameCallback(FrameCallback callback) {
        this.frameCallback = callback;
    }

    /**
     * Optionally set a custom configuration before the first frame. If not
     * called, default Config values are used automatically. Changing the config
     * discards any calibration (except colours given via setTrackColors).
     *
     * @param config  pipeline tuning parameters
     */
    public void configure(Config config) {
        this.cfg = config;
        initialized = false;   // force re-init with the new config on next frame
    }

    /**
     * Tell the pipeline the line and floor colours instead of learning them during
     * calibration. Calibration then only measures the steering reference. The
     * colours survive configure() / re-init until {@link #clearTrackColors()}.
     * Has no effect when Config.autoColor is false.
     *
     * @param lineArgb   line colour, 0xAARRGGBB (alpha ignored)
     * @param floorArgb  floor colour, 0xAARRGGBB (alpha ignored)
     */
    public void setTrackColors(int lineArgb, int floorArgb) {
        presetY1 = lumaOf(lineArgb);  presetU1 = chromaUOf(lineArgb);  presetV1 = chromaVOf(lineArgb);
        presetY0 = lumaOf(floorArgb); presetU0 = chromaUOf(floorArgb); presetV0 = chromaVOf(floorArgb);
        colorPreset = true;
        applyPresetColors();
    }

    /** Forget colours given via setTrackColors(); the next calibration learns them. */
    public void clearTrackColors() {
        colorPreset = false;
        requestRecalibration();
    }

    // =========================================================================
    //  Public API — calibration (one method per input format)
    // =========================================================================

    /**
     * Feed one grayscale frame into the calibration process.
     *
     * Place the robot correctly on the line, then call this repeatedly (one call
     * per camera frame) until it returns true. Calibration first learns the line
     * and floor colours (Config.colorCalibFrames frames, skipped when autoColor is
     * off or colours were preset), then collects Config.calibFrames solid
     * detections and takes their median as the steering zero reference.
     *
     * @param gray    packed grayscale pixels, row-major, one byte per pixel
     * @param width   frame width  in pixels
     * @param height  frame height in pixels
     * @return true when calibration is complete and processFrame is ready to use
     */
    public boolean calibrate(byte[] gray, int width, int height) {
        bindGray(gray, width, height);
        return calibrateBound();
    }

    /**
     * Feed one YUV 4:2:0 frame into the calibration process. See
     * {@link #calibrate(byte[], int, int)} for the procedure.
     *
     * @param y       Y plane, packed (no row padding), width × height bytes
     * @param u       U (Cb) plane, packed, ((width+1)/2) × ((height+1)/2) bytes
     * @param v       V (Cr) plane, packed, same size as u
     * @param width   frame width  in pixels
     * @param height  frame height in pixels
     * @return true when calibration is complete
     */
    public boolean calibrateYuv420(byte[] y, byte[] u, byte[] v, int width, int height) {
        bindYuv420(y, u, v, width, height);
        return calibrateBound();
    }

    /**
     * Feed one ARGB frame into the calibration process. See
     * {@link #calibrate(byte[], int, int)} for the procedure.
     *
     * @param argb    packed 0xAARRGGBB pixels, row-major, width × height entries
     * @param width   frame width  in pixels
     * @param height  frame height in pixels
     * @return true when calibration is complete
     */
    public boolean calibrateArgb(int[] argb, int width, int height) {
        bindArgb(argb, width, height);
        return calibrateBound();
    }

    // =========================================================================
    //  Public API — per-frame processing (one method per input format)
    // =========================================================================

    /**
     * Process one grayscale frame and return the two robot control values.
     *
     * The pipeline auto-initialises on the first call (or whenever the frame
     * dimensions change). Calibrate first; before calibration the steering zero
     * is image centre and colours are not learned (grayscale + trackIsBright).
     *
     * @param gray    packed grayscale pixels, row-major, one byte per pixel (0–255).
     *                Array length must be >= width * height.
     * @param width   frame width  in pixels
     * @param height  frame height in pixels
     * @return int[2] (same array instance reused each call — copy if you need to keep values):
     *         [0] = Steering  [-50 … +50]    negative = line is left, positive = line is right
     *         [1] = Tracking  [ min … 100  ]  100 = straight path ahead, min = hard turn / line lost
     */
    public int[] processFrame(byte[] gray, int width, int height) {
        bindGray(gray, width, height);
        return processBound();
    }

    /**
     * Process one YUV 4:2:0 frame. Same output as {@link #processFrame(byte[], int, int)}.
     * Colour input lets the pipeline separate line and floor colours of equal brightness.
     *
     * @param y       Y plane, packed (no row padding), width × height bytes
     * @param u       U (Cb) plane, packed, ((width+1)/2) × ((height+1)/2) bytes
     * @param v       V (Cr) plane, packed, same size as u
     * @param width   frame width  in pixels
     * @param height  frame height in pixels
     */
    public int[] processFrameYuv420(byte[] y, byte[] u, byte[] v, int width, int height) {
        bindYuv420(y, u, v, width, height);
        return processBound();
    }

    /**
     * Process one ARGB frame. Same output as {@link #processFrame(byte[], int, int)}.
     *
     * @param argb    packed 0xAARRGGBB pixels, row-major, width × height entries
     * @param width   frame width  in pixels
     * @param height  frame height in pixels
     */
    public int[] processFrameArgb(int[] argb, int width, int height) {
        bindArgb(argb, width, height);
        return processBound();
    }

    // =========================================================================
    //  Lifecycle
    // =========================================================================

    /**
     * Explicitly initialise (or re-initialise) the pipeline for a given frame size.
     * Called automatically when needed; only call this directly if you want to
     * change Config mid-run without waiting for a frame. Discards calibration.
     *
     * @param width   frame width  in pixels
     * @param height  frame height in pixels
     * @param config  pipeline tuning parameters
     */
    public void init(int width, int height, Config config) {
        if (width <= 0 || height <= 0) {
            throw new IllegalArgumentException("Frame size must be positive: " + width + "x" + height);
        }
        this.cfg  = config;
        this.imgW = width;
        this.imgH = height;

        final int thick = Math.max(1, Math.min(config.hRoiThickPx, height));
        int hRoiCY = (int)(config.hRoiCenterYFrac * height);
        hRoiTop = Math.max(0,      hRoiCY - thick / 2);
        hRoiBot = Math.min(height, hRoiTop + thick);

        // Robot-body exclusion: never let the strip dip into the bottom band where
        // the robot's own chassis is visible. If it would, slide the whole strip up
        // so its BOTTOM edge sits at the top of the ignored band.
        final float ignoreFrac = Math.max(0f, Math.min(0.9f, config.bottomIgnoreFrac));
        ignoreTop = (int)((1f - ignoreFrac) * height);
        if (hRoiBot > ignoreTop) {
            hRoiBot = Math.max(thick, ignoreTop);
            hRoiTop = Math.max(0, hRoiBot - thick);
        }

        vRoiBot = hRoiTop;   // V-ROI bottom sits just above the H-ROI
        // Cap the look-ahead height so it stops short of the dim, noisy far top of
        // the frame: it reaches at most vRoiHeightFrac of the frame above the H-ROI.
        vRoiTop = Math.max(0, vRoiBot - Math.round(config.vRoiHeightFrac * height));

        // Horizon row from the mount geometry. The optical axis hits the image
        // centre (yFrac 0.5); the horizon lies cameraTiltDeg ABOVE the axis (it is
        // the horizontal viewing direction), and half the vertical FOV spans from
        // centre to the top edge. So the horizon sits (tilt / (fov/2)) of the way
        // from centre toward the top:  horizonYFrac = 0.5 − tilt/fov.
        final float fov = Math.max(1f, config.verticalFovDeg);
        horizonYFrac = 0.5f - config.cameraTiltDeg / fov;

        // Effective (far-weighted) look-ahead row and the perspective ratio between
        // it and the near H-ROI row. The look-ahead value weights farther rows more
        // (weight = vRoiBot − row), so the "effective" row is the weight-centroid of
        // the V-ROI span; the ratio of apparent-width scales at these two rows maps a
        // near lateral offset (camera mount) to its far-projected column. Straight
        // road ⇒ track ~0, curve ⇒ genuine turn.
        {
            final double a = vRoiTop, b = vRoiBot;
            final double wSum  = (b - a) * (b - a) * 0.5;                                    // ∫(b−r)dr
            final double rwSum = b * (b * b - a * a) / 2.0 - (b * b * b - a * a * a) / 3.0;  // ∫r(b−r)dr
            final double effRow = (wSum > 1e-6) ? rwSum / wSum : (a + b) * 0.5;
            lookAheadRowFrac = (float) (effRow / height);
            final float scaleNear = perspectiveWidthScale((hRoiTop + hRoiBot) * 0.5f / height);
            final float scaleFar  = perspectiveWidthScale(lookAheadRowFrac);
            laPerspectiveRatio = (scaleNear > 1e-4f) ? scaleFar / scaleNear : 1f;
        }

        // Exact pinhole floor geometry per row (used by the heading and the width
        // tests). Focal length from the vertical FOV; a row at normalised offset
        // yn below the image centre looks down at (tilt + atan yn), so a floor
        // point there lies forward = (cos t − yn·sin t)/(yn·cos t + sin t) and a
        // column offset of 1 px is 1/(f·(yn·cos t + sin t)) camera heights sideways.
        // Rows at or above the horizon get NaN (never used: the V-ROI is below it).
        {
            final double f  = (height * 0.5) / Math.tan(Math.toRadians(fov * 0.5));
            final double t  = Math.toRadians(config.cameraTiltDeg);
            final double st = Math.sin(t), ct = Math.cos(t);
            rowFwd   = new float[height];
            rowScale = new float[height];
            for (int r = 0; r < height; r++) {
                final double yn = (r + 0.5 - height * 0.5) / f;
                final double down = yn * ct + st;
                if (down > 1e-3) {
                    rowFwd[r]   = (float) ((ct - yn * st) / down);
                    rowScale[r] = (float) (1.0 / (f * down));
                } else {
                    rowFwd[r] = Float.NaN; rowScale[r] = Float.NaN;
                }
            }
            final int hc = Math.min(height - 1, (hRoiTop + hRoiBot) / 2);
            floorFwdH   = rowFwd[hc];
            floorScaleH = rowScale[hc];
        }

        rowCx             = new float[height];
        initCorridor(config, width, height);
        colWeight         = new int[width];
        colLevel          = new int[width];
        work              = new byte[width * height];
        glareMap          = new byte[width * height];
        colMask           = new int[width];
        colThr            = new int[width];
        final int maxSamples = Math.max(1, config.colorCalibFrames) * SAMPLES_PER_FRAME;
        sampY             = new short[maxSamples];
        sampU             = new short[maxSamples];
        sampV             = new short[maxSamples];
        sampCol           = new short[maxSamples];
        sampLabel         = new byte[maxSamples];
        sampleCount        = 0;
        colorFramesSampled = 0;
        colorReady         = false;
        colorSeparation    = 0f;
        lineColorArgb      = 0;
        floorColorArgb     = 0;
        if (colorPreset) applyPresetColors();

        trackAcquired     = false;
        lostFrames        = 0;
        calibrated        = false;
        calibCount        = 0;
        calibIssue        = ISSUE_NONE;
        calibSteerSamples = new int[Math.max(1, config.calibFrames)];
        calibWidthSamples = new int[Math.max(1, config.calibFrames)];
        calibWidthPx      = 0f;
        learnedLineX      = 0f;
        learnedLineW      = 0f;
        relockCount       = 0;
        calibLevelSamples   = new int[Math.max(1, config.calibFrames)];
        calibHeadingSamples = new float[Math.max(1, config.calibFrames)];
        resetLineModel();
        laPathCount       = 0;
        steerRefX         = width * 0.5f;   // default = image centre until calibrated
        trackRefX         = width * 0.5f;
        lastCentroidX     = width / 2;
        lastTrackWidthPx  = config.vRoiHalfWidthPx;   // sensible fallback before first detection
        output.threshold  = 128;
        fpsWindowStartNs  = 0L;
        fpsFrameCount     = 0;
        framesPerSecond   = 0;
        totalFrames       = 0L;
        firstFrameNs      = 0L;
        elapsedNs         = 0L;
        initialized       = true;
    }

    /** Mark the pipeline uninitialised; the next frame re-initialises it (and recalibration is needed). */
    public void release() {
        initialized = false;
    }

    public boolean isInitialized()    { return initialized; }

    /** Frames processed during the most recently completed 1-second window (FPS). */
    public int getFramesPerSecond()   { return framesPerSecond; }

    /** Cumulative number of frames processed since the last init(). */
    public long getTotalFrames()      { return totalFrames; }

    /** Wall-clock seconds elapsed since the first processed frame. */
    public double getElapsedSeconds() { return elapsedNs / 1e9; }

    // Geometry getters — useful for any host UI that needs to draw sensor overlays.
    public int getHRoiTop()          { return hRoiTop; }
    public int getHRoiBot()          { return hRoiBot; }
    /** Top row of the V-ROI look-ahead (its far edge); the box spans [vRoiTop, hRoiTop). */
    public int getVRoiTop()          { return vRoiTop; }

    /** True once calibration is complete. */
    public boolean isCalibrated()    { return calibrated; }
    /** True when frames are processed in learned / preset colour mode. */
    public boolean isColorModelReady() { return colorReady; }
    /** Learned (or preset) line colour as 0xAARRGGBB; 0 until known. */
    public int getLineColorArgb()    { return lineColorArgb; }
    /** Learned (or preset) floor colour as 0xAARRGGBB; 0 until known. */
    public int getFloorColorArgb()   { return floorColorArgb; }
    /** Calibrated straight-ahead column for steering, as a fraction of frame width. */
    public float getSteerRefXFrac()  { return (imgW > 0) ? steerRefX / imgW : 0.5f; }
    /** Calibrated straight-ahead column for the look-ahead, as a fraction of frame width. */
    public float getTrackRefXFrac()  { return (imgW > 0) ? trackRefX / imgW : 0.5f; }

    /**
     * Discard the calibration so the next calibrate calls re-learn the colours
     * (unless preset via setTrackColors) and the steering reference.
     */
    public void requestRecalibration() {
        calibrated         = false;
        calibCount         = 0;
        calibIssue         = ISSUE_NONE;
        calibWidthPx       = 0f;
        learnedLineX       = 0f;
        learnedLineW       = 0f;
        relockCount        = 0;
        resetLineModel();
        trackAcquired      = false;
        lostFrames         = 0;
        laPathCount        = 0;
        sampleCount        = 0;
        colorFramesSampled = 0;
        if (colorPreset) {
            applyPresetColors();
        } else {
            colorReady      = false;
            colorSeparation = 0f;
            lineColorArgb   = 0;
            floorColorArgb  = 0;
        }
    }

    // ── Dynamic look-ahead path (nearest → farthest), fractional image coords ──
    /** Number of valid points in the look-ahead path this frame (0 = none). */
    public int     getLookAheadPointCount() { return laPathCount; }
    /** Per-point column positions as a fraction of frame width (shared buffer). */
    public float[] getLookAheadColFracs()   { return laPathColFrac; }
    /** Per-point row positions as a fraction of frame height (shared buffer). */
    public float[] getLookAheadRowFracs()   { return laPathRowFrac; }
    public int getImgW()             { return imgW; }
    public int getImgH()             { return imgH; }
    public int getVRoiHalfWidthPx()  { return cfg.vRoiHalfWidthPx; }

    /** Fractional image row (0=top…1=bottom) of the floor horizon from the mount tilt. */
    public float getHorizonYFrac()   { return horizonYFrac; }

    /**
     * Relative apparent-width scale of a fixed real-world width at fractional row
     * {@code yFrac}. For a flat floor the projected width shrinks linearly toward
     * the horizon, so the scale is simply the distance below the horizon line.
     * Returns ~0 at the horizon and grows toward the bottom of the frame; values
     * are only meaningful as ratios between two rows.
     */
    public float perspectiveWidthScale(float yFrac) {
        return Math.max(0f, yFrac - horizonYFrac);
    }

    // =========================================================================
    //  Frame binding (input formats)
    // =========================================================================

    private void bindGray(byte[] gray, int width, int height) {
        ensureInit(width, height);
        if (gray == null || gray.length < width * height) {
            throw new IllegalArgumentException("gray must hold width*height bytes");
        }
        srcFormat = FMT_GRAY; srcY = gray;
    }

    private void bindYuv420(byte[] y, byte[] u, byte[] v, int width, int height) {
        ensureInit(width, height);
        final int cw = (width + 1) / 2, ch = (height + 1) / 2;
        if (y == null || y.length < width * height) {
            throw new IllegalArgumentException("y must hold width*height bytes");
        }
        if (u == null || v == null || u.length < cw * ch || v.length < cw * ch) {
            throw new IllegalArgumentException("u and v must each hold ((width+1)/2)*((height+1)/2) bytes");
        }
        srcFormat = FMT_YUV420; srcY = y; srcU = u; srcV = v; uvW = cw;
    }

    private void bindArgb(int[] argb, int width, int height) {
        ensureInit(width, height);
        if (argb == null || argb.length < width * height) {
            throw new IllegalArgumentException("argb must hold width*height pixels");
        }
        srcFormat = FMT_ARGB; srcArgb = argb;
    }

    private void ensureInit(int width, int height) {
        if (!initialized || imgW != width || imgH != height) {
            init(width, height, cfg);
        }
    }

    // =========================================================================
    //  Calibration / processing drivers
    // =========================================================================

    private boolean calibrateBound() {
        if (calibrated) return true;
        laPathCount = 0;
        maskActive  = false;            // calibration assumes a clean view (v2 code path)

        boolean lineFound = false;
        int     centroidX = -1;

        if (cfg.autoColor && !colorReady) {
            // Phase 1: sample colours from the scan strip, learn the model once enough
            // frames are in. No line position is known yet.
            sampleStripColors();
            if (++colorFramesSampled >= Math.max(1, cfg.colorCalibFrames)) {
                learnColorModel();
            }
        } else {
            // Phase 2: find the line band (full-width) and record its centroid.
            // With the heading term on, the look-ahead rows are prepared too, so the
            // line heading seen while the robot stands straight becomes the zero.
            final boolean wantHeading = cfg.steerHeadingWeight != 0f;
            final byte[] pix = prepareWorkImage(wantHeading ? vRoiTop : hRoiTop, hRoiBot);
            final int threshold = stripThreshold(pix, workBright);
            output.threshold = threshold;
            buildColumnProfile(pix, threshold, workBright);
            // Prefer the band where colour learning saw the line (else image centre),
            // so a shadow edge or a neighbouring tape is not taken as the line.
            final float prefX = learnedLineW > 0f ? learnedLineX : imgW * 0.5f;
            final double minWeight = (double) cfg.minWeightFrac * Math.max(1, hRoiBot - hRoiTop) * imgW;
            // Width test against the width colour learning saw (when known): a band
            // merged with a neighbouring dark area must not become the reference.
            final boolean filter = cfg.rejectDistractors && learnedLineW > 0f;
            setBandLimits(threshold, workBright, learnedLineW, 0f);
            findBand(0, imgW, minWeight, prefX, imgW * 0.25f, learnedLineW, filter, false);

            if (bandStart >= 0) {
                lineFound = true;
                calibIssue = ISSUE_NONE;
                centroidX = (int) (bandMoment / bandSum);
                lastCentroidX    = centroidX;
                lastTrackWidthPx = Math.max(1, bandEnd - bandStart);
                if (calibCount < calibSteerSamples.length) {
                    calibSteerSamples[calibCount] = centroidX;
                    calibWidthSamples[calibCount] = bandEnd - bandStart;
                    calibLevelSamples[calibCount] = Math.round(bandLevel(bandStart, bandEnd));
                }
                if (wantHeading) {
                    final int vThr = lookAheadThreshold(pix, threshold, workBright);
                    traceLookAhead(pix, vThr, workBright, centroidX, false);
                    if (cfg.corridorMode) calibrateCorridorWidths(pix, workBright, calibCount);
                    laPathCount = 0;   // calibration draws no look-ahead path
                    computeHeading();
                    if (fitConfidence >= 0.999f && calibHeadingCount < calibHeadingSamples.length) {
                        calibHeadingSamples[calibHeadingCount++] = fitHeadingDeg;
                    }
                }
                if (++calibCount >= Math.max(1, cfg.calibFrames)) {
                    final int n = Math.min(calibCount, calibSteerSamples.length);
                    java.util.Arrays.sort(calibSteerSamples, 0, n);
                    java.util.Arrays.sort(calibWidthSamples, 0, n);
                    java.util.Arrays.sort(calibLevelSamples, 0, n);
                    steerRefX    = calibSteerSamples[n / 2];                // robust median
                    calibWidthPx = calibWidthSamples[n / 2];
                    calibLevel   = calibLevelSamples[n / 2];
                    trackRefX    = imgW * 0.5f + (steerRefX - imgW * 0.5f) * laPerspectiveRatio;
                    // Heading zero: median heading of the calibration frames, used only
                    // if most frames saw enough look-ahead and it is plausibly small.
                    headingRefDeg = 0f;
                    if (calibHeadingCount * 2 >= n) {
                        java.util.Arrays.sort(calibHeadingSamples, 0, calibHeadingCount);
                        final float h = calibHeadingSamples[calibHeadingCount / 2];
                        if (Math.abs(h) <= cfg.steerHeadingCalibMaxDeg) headingRefDeg = h;
                    }
                    if (cfg.corridorMode) finishCorridorWidths(n);
                    calibrated   = true;
                }
            } else {
                calibIssue = ISSUE_NO_LINE;
            }
        }

        // Fire the callback so the host can show calibration progress in its overlay.
        fireCallback(0, 100, lineFound, centroidX);
        return calibrated;
    }

    private int[] processBound() {
        updateFps();
        // Coast clock: time since the previous frame, clamped so a pause cannot use up the coast.
        final long now = System.nanoTime();
        float dt = cfg.frameIntervalMs > 0f ? cfg.frameIntervalMs
                 : (lastFrameNs != 0L ? (now - lastFrameNs) / 1e6f : 0f);
        frameDtMs   = Math.max(0f, Math.min(MAX_FRAME_DT_MS, dt));
        lastFrameNs = now;
        maskActive  = cfg.glareMask;
        if (maskActive) setupGlareSource(vRoiTop, hRoiBot);
        corrUsed = false;
        relockedThisFrame = false;
        corrPredValid = false;
        pinCapActive = false;
        if (sinceCurveFrames < 1000) sinceCurveFrames++;
        // Corridor frames convert only the rows they read (lazy); v2 frames the whole look-ahead.
        lazyRows = false;
        final byte[] pix = (cfg.corridorMode && kfReady && trackAcquired && calibrated)
                ? prepareLazy() : prepareWorkImage(vRoiTop, hRoiBot);
        runPipeline(pix, imgW, imgH, workBright);
        // Internal steering is [-100 … +100]; the output is scaled to [-50 … +50].
        result[0] = Math.round(output.steering * 0.5f);
        // Invert the internal intersection scale:
        //   internal 0   (straight) → output 100 (no braking needed)
        //   internal 100 (hard turn / lost) → output min (brake / slow down)
        result[1] = Math.max(cfg.trackingMinValue, Math.round(100f - output.intersection));
        isProcessing = true;
        fireCallback(result[0], result[1], output.trackFound, output.centroidX);
        isProcessing = false;
        return result;
    }

    // =========================================================================
    //  Line-ness image
    // =========================================================================

    /**
     * Returns the single-channel image the detector runs on, valid on rows
     * [r0, r1), and sets {@link #workBright} to its polarity. With a colour model
     * the image is the line-ness projection (line bright, floor dark); otherwise
     * it is plain luma with polarity Config.trackIsBright.
     */
    private byte[] prepareWorkImage(int r0, int r1) {
        final int W = imgW;
        final boolean project = cfg.autoColor && colorReady;

        if (!project) {
            workBright = cfg.trackIsBright;
            if (srcFormat != FMT_ARGB) return srcY;          // Y plane is the luma image
            final int[] src = srcArgb;
            final byte[] out = work;
            for (int i = r0 * W, end = r1 * W; i < end; i++) {
                final int c = src[i];
                out[i] = (byte) ((77 * ((c >> 16) & 0xFF) + 150 * ((c >> 8) & 0xFF) + 29 * (c & 0xFF)) >> 8);
            }
            return out;
        }

        workBright = true;
        final byte[] out = work;
        if (srcFormat == FMT_GRAY) {
            final byte[] ys = srcY;
            final int ky = kY, off = kGrayOff;
            for (int i = r0 * W, end = r1 * W; i < end; i++) {
                out[i] = clampByte((ky * (ys[i] & 0xFF) + off) >> 10);
            }
        } else if (srcFormat == FMT_YUV420) {
            final byte[] ys = srcY, us = srcU, vs = srcV;
            final int ky = kY, ku = kU, kv = kV, off = k0, cw = uvW;
            for (int row = r0; row < r1; row++) {
                final int base   = row * W;
                final int uvBase = (row >> 1) * cw;
                for (int col = 0; col < W; col++) {
                    final int uvi = uvBase + (col >> 1);
                    out[base + col] = clampByte((ky * (ys[base + col] & 0xFF)
                            + ku * (us[uvi] & 0xFF) + kv * (vs[uvi] & 0xFF) + off) >> 10);
                }
            }
        } else {
            final int[] src = srcArgb;
            final int kr = kR, kg = kG, kb = kB, off = kC;
            for (int i = r0 * W, end = r1 * W; i < end; i++) {
                final int c = src[i];
                out[i] = clampByte((kr * ((c >> 16) & 0xFF) + kg * ((c >> 8) & 0xFF)
                        + kb * (c & 0xFF) + off) >> 10);
            }
        }
        return out;
    }

    /**
     * v3 (glareMask): where the glare test reads from. A pixel is glare when its brightness Y is
     * at least Config.glareLumaMax (gray / YUV input: read straight from the Y plane, no extra
     * pass) or when any of R, G, B is at least 250 (ARGB input: a 0/1 map is built for rows
     * [r0, r1)). Saturated pixels carry no colour information, so they are neither line nor
     * floor. Test a pixel i with {@code (glareSrc[i] & 0xFF) >= glareLim}.
     */
    private void setupGlareSource(int r0, int r1) {
        maskedPixels = 0;
        maskedSamples = 0;
        maskedSampleHits = 0;
        if (srcFormat != FMT_ARGB) {
            glareSrc = srcY;
            glareLim = Math.max(1, Math.min(255, cfg.glareLumaMax));
            return;
        }
        final int W = imgW;
        final int[] s = srcArgb;
        final byte[] m = glareMap;
        final int lim = GLARE_CHANNEL_MAX;
        for (int i = r0 * W, end = r1 * W; i < end; i++) {
            final int c = s[i];
            m[i] = (((c >> 16) & 0xFF) >= lim || ((c >> 8) & 0xFF) >= lim || (c & 0xFF) >= lim) ? (byte) 1 : (byte) 0;
        }
        glareSrc = m;
        glareLim = 1;
    }

    private static byte clampByte(int v) {
        return (byte) (v < 0 ? 0 : (v > 255 ? 255 : v));
    }

    // =========================================================================
    //  Colour learning (calibration phase 1)
    // =========================================================================

    /** Grid-sample up to SAMPLES_PER_FRAME (Y,U,V,column) tuples from the H-ROI. */
    private void sampleStripColors() {
        final int W = imgW;
        final int rows  = Math.max(1, hRoiBot - hRoiTop);
        final int nRows = Math.min(8, rows);
        final int stepR = Math.max(1, rows / nRows);
        final int stepC = Math.max(1, (W * nRows + SAMPLES_PER_FRAME - 1) / SAMPLES_PER_FRAME);
        final int cap   = sampY.length;

        for (int row = hRoiTop; row < hRoiBot && sampleCount < cap; row += stepR) {
            for (int col = 0; col < W && sampleCount < cap; col += stepC) {
                final int i = row * W + col;
                int y, u, v;
                if (srcFormat == FMT_GRAY) {
                    y = srcY[i] & 0xFF; u = 128; v = 128;
                } else if (srcFormat == FMT_YUV420) {
                    final int uvi = (row >> 1) * uvW + (col >> 1);
                    y = srcY[i] & 0xFF; u = srcU[uvi] & 0xFF; v = srcV[uvi] & 0xFF;
                } else {
                    final int c = srcArgb[i];
                    y = lumaOf(c); u = chromaUOf(c); v = chromaVOf(c);
                }
                sampY[sampleCount]   = (short) y;
                sampU[sampleCount]   = (short) u;
                sampV[sampleCount]   = (short) v;
                sampCol[sampleCount] = (short) col;
                sampleCount++;
            }
        }
    }

    /**
     * Group the gathered samples into colour clusters, decide which cluster is the
     * line, and derive the line-ness projection. Two clusters (line + floor) are
     * tried first; if that does not give a clean answer — typically because a
     * shadow or uneven light splits the floor into a lit and a dark part, and the
     * dark part lands in the same cluster as a dark line — 3 and then 4 clusters
     * are tried, with the extra clusters counted as floor. On failure the samples
     * are discarded (calibration keeps sampling) and calibIssue says why.
     * Runs once per calibration — allocation here is fine.
     */
    private void learnColorModel() {
        final int n = sampleCount;
        sampleCount = 0;
        colorFramesSampled = 0;
        if (n < 16) { calibIssue = ISSUE_LOW_CONTRAST; return; }

        int twoClusterIssue = ISSUE_NONE;
        for (int k = 2; k <= MAX_CLUSTERS; k++) {
            if (tryColorModel(n, k)) { calibIssue = ISSUE_NONE; return; }
            if (k == 2) twoClusterIssue = calibIssue;
        }
        calibIssue = twoClusterIssue;   // report the plain line-vs-floor diagnosis
    }

    private static final int MAX_CLUSTERS = 4;

    /** One clustering attempt with {@code k} clusters. Sets calibIssue on failure. */
    private boolean tryColorModel(int n, int k) {
        final double[] c = new double[k * 3];
        if (!kMeans(n, k, c)) { calibIssue = ISSUE_LOW_CONTRAST; return false; }

        // Edge-zone membership: the robot starts ON the line, so the line occupies a
        // band in the middle of the strip while the floor fills both frame edges.
        final int edge = Math.max(1, imgW / 20);
        final int[] left = new int[k], right = new int[k], count = new int[k];
        int leftTotal = 0, rightTotal = 0;
        for (int i = 0; i < n; i++) {
            final int col = sampCol[i], lbl = sampLabel[i];
            count[lbl]++;
            if (col < edge)              { left[lbl]++;  leftTotal++;  }
            else if (col >= imgW - edge) { right[lbl]++; rightTotal++; }
        }

        final double[] band = new double[2];   // {centre column, width} of the line's band
        int line = -1;
        if (k == 2) {
            // The floor is the cluster holding the majority of BOTH edge zones.
            int floor = -1;
            for (int j = 0; j < 2; j++) {
                if (left[j] * 2 > leftTotal && right[j] * 2 > rightTotal) floor = j;
            }
            if (floor < 0) { calibIssue = ISSUE_NO_FLOOR; return false; }
            line = 1 - floor;
            // Contrast first (the more basic diagnosis), then the line must form
            // one compact band — otherwise the split was not line vs floor.
            if (!fitDiscriminant(n, k, c, count, line)) return false;
            if (clusterBand(n, line, band) < 0.6) {
                colorReady      = false;
                colorSeparation = 0f;
                lineColorArgb   = 0;
                floorColorArgb  = 0;
                calibIssue      = ISSUE_NO_FLOOR;
                return false;
            }
            learnedLineX = (float) band[0];
            learnedLineW = (float) band[1];
            return true;
        } else {
            // The line is a cluster that (almost) never reaches either edge and forms
            // one compact band; if several qualify, the one nearest the image centre.
            double bestDist = Double.MAX_VALUE;
            final double[] b = new double[2];
            for (int j = 0; j < k; j++) {
                if (count[j] < n / 100) continue;
                if (left[j] > 0.2 * leftTotal || right[j] > 0.2 * rightTotal) continue;
                if (clusterBand(n, j, b) < 0.6 || b[1] > 0.7 * imgW) continue;
                final double d = Math.abs(b[0] - imgW * 0.5);
                if (d < bestDist) { bestDist = d; line = j; band[0] = b[0]; band[1] = b[1]; }
            }
            if (line < 0) { calibIssue = ISSUE_NO_FLOOR; return false; }
        }

        if (!fitDiscriminant(n, k, c, count, line)) return false;
        learnedLineX = (float) band[0];
        learnedLineW = (float) band[1];
        return true;
    }

    /**
     * Deterministic k-means in YUV space. Seeds: the sample farthest from the
     * overall mean, then repeatedly the sample farthest from all seeds so far.
     * Leaves labels in sampLabel and centres in {@code c} (k × {Y,U,V}).
     * Returns false if a cluster ends up empty.
     */
    private boolean kMeans(int n, int k, double[] c) {
        double mY = 0, mU = 0, mV = 0;
        for (int i = 0; i < n; i++) { mY += sampY[i]; mU += sampU[i]; mV += sampV[i]; }
        mY /= n; mU /= n; mV /= n;
        final int s = farthestSample(n, mY, mU, mV);
        c[0] = sampY[s]; c[1] = sampU[s]; c[2] = sampV[s];
        for (int j = 1; j < k; j++) {
            int best = 0; double bestD = -1;
            for (int i = 0; i < n; i++) {
                double dMin = Double.MAX_VALUE;
                for (int q = 0; q < j; q++) {
                    dMin = Math.min(dMin, sq(sampY[i] - c[q * 3]) + sq(sampU[i] - c[q * 3 + 1])
                                          + sq(sampV[i] - c[q * 3 + 2]));
                }
                if (dMin > bestD) { bestD = dMin; best = i; }
            }
            c[j * 3] = sampY[best]; c[j * 3 + 1] = sampU[best]; c[j * 3 + 2] = sampV[best];
        }

        final long[]   cnt = new long[k];
        final double[] sum = new double[k * 3];
        for (int iter = 0; iter < 20; iter++) {
            boolean changed = false;
            java.util.Arrays.fill(cnt, 0);
            java.util.Arrays.fill(sum, 0);
            for (int i = 0; i < n; i++) {
                final double y = sampY[i], u = sampU[i], v = sampV[i];
                int lbl = 0; double dBest = Double.MAX_VALUE;
                for (int q = 0; q < k; q++) {
                    final double d = sq(y - c[q * 3]) + sq(u - c[q * 3 + 1]) + sq(v - c[q * 3 + 2]);
                    if (d < dBest) { dBest = d; lbl = q; }
                }
                if (iter == 0 || sampLabel[i] != lbl) changed = true;
                sampLabel[i] = (byte) lbl;
                cnt[lbl]++;
                sum[lbl * 3] += y; sum[lbl * 3 + 1] += u; sum[lbl * 3 + 2] += v;
            }
            for (int q = 0; q < k; q++) {
                if (cnt[q] == 0) return false;
                c[q * 3]     = sum[q * 3]     / cnt[q];
                c[q * 3 + 1] = sum[q * 3 + 1] / cnt[q];
                c[q * 3 + 2] = sum[q * 3 + 2] / cnt[q];
            }
            if (!changed) break;
        }
        return true;
    }

    /**
     * How compact cluster {@code cl} is across the strip: a column belongs to the
     * cluster when most of its samples do; member columns are grouped into runs
     * (small gaps bridged) and the run holding most of the cluster's samples is
     * its band. Writes {centre column, width} of that band to {@code out} and
     * returns the fraction of the cluster's samples inside it (1 = one clean band).
     */
    private double clusterBand(int n, int cl, double[] out) {
        final int W = imgW;
        final int[] total = new int[W], in = new int[W];
        int clTotal = 0;
        for (int i = 0; i < n; i++) {
            total[sampCol[i]]++;
            if (sampLabel[i] == cl) { in[sampCol[i]]++; clTotal++; }
        }
        if (clTotal == 0) return 0;
        final int gapTol = runGapTol();
        int bestIn = 0, bestStart = -1, bestEnd = -1;
        int curIn = 0, curStart = -1, curLast = -1;
        for (int col = 0; col <= W; col++) {
            final boolean sampled = col < W && total[col] > 0;
            final boolean member  = sampled && in[col] * 2 > total[col];
            if (member) {
                if (curStart < 0) { curStart = col; curIn = 0; }
                curIn += in[col]; curLast = col;
            } else if (curStart >= 0 && (col == W || (sampled && col - curLast > gapTol))) {
                if (curIn > bestIn) { bestIn = curIn; bestStart = curStart; bestEnd = curLast + 1; }
                curStart = -1;
            }
        }
        if (bestStart < 0) return 0;
        out[0] = (bestStart + bestEnd) * 0.5;
        out[1] = bestEnd - bestStart;
        return bestIn / (double) clTotal;
    }

    /**
     * Fisher discriminant between the line cluster and the NEAREST floor cluster:
     * w = Σ⁻¹·(m_line − m_floor), with Σ the pooled within-cluster covariance.
     * Every other floor cluster must project at least about as far from the line,
     * so no part of the floor reads as line. With two clusters this is exactly
     * the classic line-vs-floor discriminant.
     */
    private boolean fitDiscriminant(int n, int k, double[] c, int[] count, int line) {
        // ── Pooled within-cluster covariance (3×3, symmetric) ──
        final double[] cov = new double[6];   // yy yu yv uu uv vv
        for (int i = 0; i < n; i++) {
            final int q = sampLabel[i] * 3;
            final double dy = sampY[i] - c[q], du = sampU[i] - c[q + 1], dv = sampV[i] - c[q + 2];
            cov[0] += dy * dy; cov[1] += dy * du; cov[2] += dy * dv;
            cov[3] += du * du; cov[4] += du * dv; cov[5] += dv * dv;
        }
        // Regularise: a floor of ~1.5 grey-levels of noise per channel keeps the
        // matrix invertible (grayscale input has zero chroma variance) and stops a
        // perfectly clean channel from getting an unbounded weight.
        final double reg = 2.0;
        for (int j = 0; j < 6; j++) cov[j] /= n;
        cov[0] += reg; cov[3] += reg; cov[5] += reg;

        final double a = cov[0], b = cov[1], cc = cov[2], d = cov[3], e = cov[4], f = cov[5];
        final double i00 = d * f - e * e, i01 = cc * e - b * f, i02 = b * e - cc * d;
        final double i11 = a * f - cc * cc, i12 = b * cc - a * e, i22 = a * d - b * b;
        final double det = a * i00 + b * i01 + cc * i02;
        if (Math.abs(det) < 1e-9) { calibIssue = ISSUE_LOW_CONTRAST; return false; }

        // ── Nearest floor cluster (smallest Mahalanobis distance to the line) ──
        final int L = line * 3;
        int floor = -1, mainFloor = -1;
        double bestM = Double.MAX_VALUE;
        for (int q = 0; q < k; q++) {
            if (q == line) continue;
            final double qY = c[L] - c[q * 3], qU = c[L + 1] - c[q * 3 + 1], qV = c[L + 2] - c[q * 3 + 2];
            final double m = (qY * (i00 * qY + i01 * qU + i02 * qV)
                            + qU * (i01 * qY + i11 * qU + i12 * qV)
                            + qV * (i02 * qY + i12 * qU + i22 * qV)) / det;
            if (m < bestM) { bestM = m; floor = q; }
            if (mainFloor < 0 || count[q] > count[mainFloor]) mainFloor = q;
        }
        final int F = floor * 3;
        final double dY = c[L] - c[F], dU = c[L + 1] - c[F + 1], dV = c[L + 2] - c[F + 2];
        final double dist = Math.sqrt(dY * dY + dU * dU + dV * dV);
        final double wY = (i00 * dY + i01 * dU + i02 * dV) / det;
        final double wU = (i01 * dY + i11 * dU + i12 * dV) / det;
        final double wV = (i02 * dY + i12 * dU + i22 * dV) / det;

        // Mahalanobis separation: sqrt(dᵀ Σ⁻¹ d) = sqrt(w·d).
        final double wd = wY * dY + wU * dU + wV * dV;
        final double separation = Math.sqrt(Math.max(0, wd));
        if (separation < cfg.colorMinSeparation || dist < cfg.colorMinDistance) {
            calibIssue = ISSUE_LOW_CONTRAST;
            return false;
        }
        // Other floor clusters must sit on the floor side of the nearest one.
        for (int q = 0; q < k; q++) {
            if (q == line || q == floor) continue;
            final double wq = wY * (c[L] - c[q * 3]) + wU * (c[L + 1] - c[q * 3 + 1])
                            + wV * (c[L + 2] - c[q * 3 + 2]);
            if (wq < 0.8 * wd) { calibIssue = ISSUE_LOW_CONTRAST; return false; }
        }

        setProjection(wY, wU, wV, c[F], c[F + 1], c[F + 2], wd);
        colorSeparation = (float) separation;
        lineColorArgb   = yuvToArgb(c[L], c[L + 1], c[L + 2]);
        floorColorArgb  = yuvToArgb(c[mainFloor * 3], c[mainFloor * 3 + 1], c[mainFloor * 3 + 2]);
        colorReady      = true;
        return true;
    }

    private int farthestSample(int n, double y, double u, double v) {
        int best = 0; double bestD = -1;
        for (int i = 0; i < n; i++) {
            final double dd = sq(sampY[i] - y) + sq(sampU[i] - u) + sq(sampV[i] - v);
            if (dd > bestD) { bestD = dd; best = i; }
        }
        return best;
    }

    private static double sq(double x) { return x * x; }

    /**
     * Install the projection f = w·(p − floor) scaled so the floor maps to 64 and
     * the line to 192 ({@code wd} = w·(line − floor) > 0), as fixed-point (×1024)
     * coefficients for both YUV and RGB input.
     */
    private void setProjection(double wY, double wU, double wV,
                               double floorY, double floorU, double floorV, double wd) {
        final double s  = 128.0 / wd;
        final double sy = wY * s, su = wU * s, sv = wV * s;
        final double off = 64.0 - (sy * floorY + su * floorU + sv * floorV);
        kY = (int) Math.round(sy * 1024);
        kU = (int) Math.round(su * 1024);
        kV = (int) Math.round(sv * 1024);
        k0 = (int) Math.round(off * 1024) + 512;              // +512 → round-to-nearest after >> 10
        kGrayOff = k0 + 128 * kU + 128 * kV;

        // Same function expressed in RGB, using the conversions in lumaOf/chromaUOf/chromaVOf:
        //   Y = .301R + .586G + .113B,  U = −.168R − .332G + .5B + 128,  V = .5R − .418G − .082B + 128
        final double rY = 77 / 256.0,  gY = 150 / 256.0, bY = 29 / 256.0;
        final double rU = -43 / 256.0, gU = -85 / 256.0, bU = 128 / 256.0;
        final double rV = 128 / 256.0, gV = -107 / 256.0, bV = -21 / 256.0;
        kR = (int) Math.round((sy * rY + su * rU + sv * rV) * 1024);
        kG = (int) Math.round((sy * gY + su * gU + sv * gV) * 1024);
        kB = (int) Math.round((sy * bY + su * bU + sv * bV) * 1024);
        kC = (int) Math.round((off + 128 * su + 128 * sv) * 1024) + 512;
    }

    /** Build the projection from colours given via setTrackColors(). */
    private void applyPresetColors() {
        final double dY = presetY1 - presetY0, dU = presetU1 - presetU0, dV = presetV1 - presetV0;
        final double wd = dY * dY + dU * dU + dV * dV;
        if (wd < 1.0) {
            throw new IllegalArgumentException("Line and floor colours must differ");
        }
        // No noise statistics are known, so the direction is the plain colour difference.
        setProjection(dY, dU, dV, presetY0, presetU0, presetV0, wd);
        colorSeparation = 0f;
        lineColorArgb   = yuvToArgb(presetY1, presetU1, presetV1);
        floorColorArgb  = yuvToArgb(presetY0, presetU0, presetV0);
        colorReady      = true;
    }

    // RGB → YUV (BT.601 full range, integer approximation — matches setProjection).
    private static int lumaOf(int c) {
        return (77 * ((c >> 16) & 0xFF) + 150 * ((c >> 8) & 0xFF) + 29 * (c & 0xFF)) >> 8;
    }
    private static int chromaUOf(int c) {
        return ((-43 * ((c >> 16) & 0xFF) - 85 * ((c >> 8) & 0xFF) + 128 * (c & 0xFF)) >> 8) + 128;
    }
    private static int chromaVOf(int c) {
        return ((128 * ((c >> 16) & 0xFF) - 107 * ((c >> 8) & 0xFF) - 21 * (c & 0xFF)) >> 8) + 128;
    }

    private static int yuvToArgb(double y, double u, double v) {
        final int r = clamp255(y + 1.402 * (v - 128));
        final int g = clamp255(y - 0.344 * (u - 128) - 0.714 * (v - 128));
        final int b = clamp255(y + 1.772 * (u - 128));
        return 0xFF000000 | (r << 16) | (g << 8) | b;
    }

    private static int clamp255(double x) {
        return (int) Math.max(0, Math.min(255, Math.round(x)));
    }

    // =========================================================================
    //  Shared detection stages (used by calibration and the hot path)
    // =========================================================================

    /**
     * Stage 1: Otsu threshold of the H-ROI, biased by thresholdMargin toward the
     * track side. Returns {@link #NO_TRACK} when the strip has too little contrast
     * to contain a line.
     */
    private int stripThreshold(byte[] pixels, boolean bright) {
        if (maskActive) return stripThresholdMasked(pixels, bright);
        final int W = imgW;
        final int[] hist = histogram;
        java.util.Arrays.fill(hist, 0);
        for (int row = hRoiTop; row < hRoiBot; row++) {
            final int base = row * W;
            for (int col = 0; col < W; col++) {
                hist[pixels[base + col] & 0xFF]++;
            }
        }
        return finishThreshold(otsuThreshold(hist, (long) (hRoiBot - hRoiTop) * W), bright);
    }

    /**
     * v3 Stage 1 (glareMask): the same Otsu threshold, but saturated pixels are left out (G4),
     * and in learned-colour mode each eighth of the strip also gets its own Otsu threshold
     * (G3: a bright patch in one part of the strip no longer sets the threshold for the rest).
     * A segment without enough contrast (only floor, or only line) uses the strip-wide value.
     * The per-column result (segment values interpolated between segment centres) goes to
     * colThr; colMask gets the masked-pixel count per column. Returns the strip-wide threshold.
     */
    private int stripThresholdMasked(byte[] pixels, boolean bright) {
        final int W = imgW, S = THR_SEGMENTS;
        final int[] hist = histogram, sh = segHist, cm = colMask;
        final byte[] gs = glareSrc;
        final int gl = glareLim;
        java.util.Arrays.fill(hist, 0);
        java.util.Arrays.fill(sh, 0);
        java.util.Arrays.fill(cm, 0, W, 0);
        long n = 0;
        for (int row = hRoiTop; row < hRoiBot; row++) {
            final int base = row * W;
            for (int col = 0; col < W; col++) {
                if ((gs[base + col] & 0xFF) >= gl) { cm[col]++; maskedPixels++; continue; }
                final int v = pixels[base + col] & 0xFF;
                hist[v]++;
                sh[(col * S / W) * 256 + v]++;
                n++;
            }
        }
        final int global = finishThreshold(otsuThreshold(hist, n), bright);
        final int[] st = segThr;
        final boolean local = cfg.autoColor && colorReady;
        for (int s = 0; s < S; s++) {
            st[s] = global;
            if (!local) continue;
            long cnt = 0;
            for (int g = 0; g < 256; g++) cnt += sh[s * 256 + g];
            if (cnt == 0) continue;
            // Otsu on this segment (copied into the shared histogram buffer, which is free now).
            System.arraycopy(sh, s * 256, histogramV, 0, 256);
            final int t = otsuThreshold(histogramV, cnt);
            if (otsuContrast >= cfg.minContrast) st[s] = finishThreshold(t, bright);
        }
        final int[] ct = colThr;
        for (int col = 0; col < W; col++) {
            final float pos = (col + 0.5f) * S / W - 0.5f;
            final int s0 = Math.max(0, Math.min(S - 1, (int) Math.floor(pos)));
            final int s1 = Math.min(S - 1, s0 + 1);
            final float t = Math.max(0f, Math.min(1f, pos - s0));
            final int a = st[s0], b = st[s1];
            ct[col] = a == NO_TRACK ? b : (b == NO_TRACK ? a : Math.round(a + (b - a) * t));
        }
        otsuContrast = 0;
        return global;
    }

    /** Threshold value meaning "no line in this region" (no pixel passes it). */
    private static final int NO_TRACK = -1;

    // For a bright track: threshold = Otsu + margin  (looking for bright pixels)
    // For a dark  track: threshold = Otsu − margin  (looking for dark  pixels)
    // Uses otsuContrast from the otsuThreshold() call that produced {@code otsu}.
    private int finishThreshold(int otsu, boolean bright) {
        if (otsuContrast < cfg.minContrast) return NO_TRACK;
        int t = bright ? Math.min(240, otsu + cfg.thresholdMargin)
                       : Math.max(15,  otsu - cfg.thresholdMargin);
        if (cfg.autoColor && colorReady) t = Math.max(t, cfg.colorMinThreshold);
        return t;
    }

    /** True when {@code px} is on the track side of {@code threshold}. */
    private static boolean isTrack(int px, int threshold, boolean bright) {
        if (threshold == NO_TRACK) return false;
        return bright ? (px >= threshold) : (px <= threshold);
    }

    /**
     * Collapse the H-ROI into a per-column count of track pixels (colWeight) and
     * the sum of their line-ness (colLevel; line-ness = pixel value for a bright
     * track, 255 − value for a dark one, so "more line-like" is always higher).
     */
    private void buildColumnProfile(byte[] pixels, int threshold, boolean bright) {
        final int W = imgW;
        final int[] colW = colWeight;
        final int[] colL = colLevel;
        java.util.Arrays.fill(colW, 0, W, 0);
        java.util.Arrays.fill(colL, 0, W, 0);
        if (maskActive) {
            // Per-column (local) threshold; masked pixels are neither line nor floor.
            final int[] ct = colThr;
            final byte[] gs = glareSrc;
            final int gl = glareLim;
            for (int row = hRoiTop; row < hRoiBot; row++) {
                final int base = row * W;
                for (int col = 0; col < W; col++) {
                    final int t = ct[col];
                    if (t == NO_TRACK || (gs[base + col] & 0xFF) >= gl) continue;
                    final int px = pixels[base + col] & 0xFF;
                    if (bright ? (px >= t) : (px <= t)) {
                        colW[col]++;
                        colL[col] += bright ? px : 255 - px;
                    }
                }
            }
            return;
        }
        if (threshold == NO_TRACK) return;
        for (int row = hRoiTop; row < hRoiBot; row++) {
            final int base = row * W;
            for (int col = 0; col < W; col++) {
                final int px = pixels[base + col] & 0xFF;
                if (bright ? (px >= threshold) : (px <= threshold)) {
                    colW[col]++;
                    colL[col] += bright ? px : 255 - px;
                }
            }
        }
    }

    /** Mean line-ness of the track pixels in columns [start, end). */
    private float bandLevel(int start, int end) {
        long n = 0, s = 0;
        for (int c = Math.max(0, start), e = Math.min(imgW, end); c < e; c++) { n += colWeight[c]; s += colLevel[c]; }
        return n > 0 ? (float) s / n : 0f;
    }

    /**
     * Best contiguous track run of colWeight in [lo, hi). Small gaps (up to
     * runGapTol columns, e.g. specular noise that briefly breaks the line) are
     * bridged so a single line is not split into competing fragments.
     *
     * Only runs with at least {@code minWeight} track pixels qualify. Among those
     * the run with the highest {@link #scoreBand score} wins: its strength,
     * discounted by distance from {@code prefX} (on the scale {@code scale}) and,
     * when {@code expectW} > 0, by how far its width is from the expected width.
     * Writes bandSum / bandMoment / bandStart / bandEnd / bandScore
     * (bandStart = -1 if no run qualifies).
     */
    //
    // With {@code filter} (Config.rejectDistractors) a run must also look like the
    // line learned at calibration (see setBandLimits): runs narrower than fMinW
    // are skipped, runs wider than fMaxW are skipped too — or, with {@code trim}
    // (line locked), cut back to the part that continues the predicted line —
    // and the score is weighted by how line-like the run's pixels are.
    private void findBand(int lo, int hi, double minWeight, float prefX, float scale, float expectW,
                          boolean filter, boolean trim) {
        final int[] colW = colWeight, cm = colMask;
        final int runGapTol = runGapTol();
        // Glare (v3): a column that is at least a quarter masked is "unknown" and counts as part
        // of a band (a highlight on the tape centre does not split it; a saturated stretch of
        // tape of the right width is still a candidate — scored low, as it has no line-ness).
        final boolean mask = maskActive;
        final int maskMin = Math.max(1, (hRoiBot - hRoiTop) / 4);
        long   curSum = 0;
        double curMoment = 0;
        int    curStart = -1, curLast = -1, gapCols = 0;
        bandSum = 0; bandMoment = 0; bandStart = -1; bandEnd = -1; bandScore = -1f; bandTrimmed = false;
        for (int col = lo; col <= hi; col++) {
            int w = (col < hi) ? colW[col] : 0;
            if (mask && col < hi && cm[col] >= maskMin) w += cm[col];
            if (w > 0) {
                if (curStart < 0) { curStart = col; curSum = 0; curMoment = 0; }
                curSum += w; curMoment += (double) col * w; gapCols = 0; curLast = col;
            } else if (curStart >= 0 && (col == hi || ++gapCols > runGapTol)) {   // run ends
                if (curSum >= minWeight) {
                    if (filter) {
                        considerRun(curStart, curLast + 1, curSum, curMoment, minWeight, prefX, scale, expectW, trim);
                    } else {
                        final float s = scoreBand(curSum, (float) (curMoment / curSum),
                                                  curLast + 1 - curStart, prefX, scale, expectW);
                        if (s > bandScore) {
                            bandScore = s; bandSum = curSum; bandMoment = curMoment;
                            bandStart = curStart; bandEnd = curLast + 1;
                        }
                    }
                }
                curStart = -1;
            }
        }
    }

    // ── Distractor rejection: what the line must look like (set per frame) ──
    private float fMinW, fMaxW;           // accepted band width range (px)
    private float fExpW;                  // expected band width for the current slant (px; 0 = unknown)
    private float fPredX, fPredW;         // predicted line centre and width (px), for trimming
    private float fLevelThr, fLevelLine;  // line-ness at the threshold / of the calibrated line
    // Sub-band written by trimBand().
    private int    trimStart, trimEnd;
    private long   trimSum;
    private double trimMoment;

    /**
     * Set the width / line-ness limits for the next findBand calls.
     * {@code slantW}: expected band width (px) for the line's current slant
     * (0 = unknown → no width limits); {@code minRefW}: width the minimum is
     * relative to (0 = no minimum).
     */
    private void setBandLimits(int threshold, boolean bright, float slantW, float minRefW) {
        fExpW = slantW;
        fMaxW = slantW > 0f ? cfg.bandMaxWidthMult * slantW + 4f : Float.MAX_VALUE;
        fMinW = minRefW > 0f ? cfg.bandMinWidthMult * minRefW : 0f;
        fLevelThr  = threshold == NO_TRACK ? 0f : (bright ? threshold : 255 - threshold);
        fLevelLine = calibLevel;
    }

    /** Line-likeness [0.1 … 1] of a band with mean line-ness {@code level}; 1 when unknown. */
    private float levelFactor(float level) {
        final float span = fLevelLine - fLevelThr;
        if (span < 8f) return 1f;                         // line level unknown / too close to threshold
        return Math.max(0.1f, Math.min(1f, (level - fLevelThr) / span));
    }

    /** Score one run under the distractor rules (filter on) and keep it if best so far. */
    private void considerRun(int start, int end, long sum, double moment, double minWeight,
                             float prefX, float scale, float expectW, boolean trim) {
        boolean trimmed = false;
        final int width = end - start;
        if (maskActive) {
            // Glare (v3): a band made mostly of masked columns has no colour to judge it by.
            // It may be the line (a saturated stretch of tape) only if it has the calibrated
            // width itself — no slant allowance, no trimming (a glare patch is not cut to size).
            long line = 0;
            for (int c = start; c < end; c++) line += colWeight[c];
            if (4 * line < sum) {
                final float calW = expectedLineWidth();
                if (calW <= 0f || width > 1.3f * calW + 4f || width < fMinW) { rejectedBands++; return; }
                trim = false;
            }
        }
        if (width > fMaxW) {
            // Too wide for the line: a line merged with a blob / shadow, or a blob.
            rejectedBands++;
            if (!trim || !trimBand(start, end)) return;
            start = trimStart; end = trimEnd; sum = trimSum; moment = trimMoment;
            if (sum < minWeight * 0.5 || sum <= 0) return;
            trimmed = true;
        } else if (trim && fExpW > 0f && width > 1.3f * fExpW && trimBand(start, end)
                   && trimEnd - trimStart < width) {
            // Wider than the line but not impossibly so: the line has started to
            // merge with something beside it. Keep the part that continues the
            // line, so the centroid does not creep toward the object.
            rejectedBands++;
            start = trimStart; end = trimEnd; sum = trimSum; moment = trimMoment;
            trimmed = true;
        } else if (width < fMinW && start > 0 && end < imgW) {
            rejectedBands++;                               // much thinner than the tape
            return;
        }
        final int w = end - start;
        // D2 (v3): a piece cut out of a wider band is the line only if the line continues up
        // the look-ahead from it — otherwise it is the edge of a mat or blob the line ran into.
        if (trimmed && hardening && continuity((float) (moment / sum), lastImgSlope) < 2
                && coveredAbove((float) (moment / sum), lastImgSlope) >= 2) { rejectedBands++; return; }
        // Pixel count, capped at what a line of the expected width can hold, so a
        // bigger object of acceptable width does not win just by being bigger.
        long s = sum;
        if (expectW > 0f) s = Math.min(s, (long) (1.2f * expectW * Math.max(1, hRoiBot - hRoiTop)));
        float score = scoreBand(s, (float) (moment / sum), w, prefX, scale, expectW);
        score *= levelFactor(bandLevel(start, end));
        // D3 (v3): both tape edges must be sharp; a soft-edged shadow / blob counts a third.
        if (hardening && cfg.edgePairCheck && !trimmed && start > 0 && end < imgW && !hasEdgePair(start, end)) score *= 0.3f;
        if (score > bandScore) {
            bandScore = score; bandSum = sum; bandMoment = moment;
            bandStart = start; bandEnd = end; bandTrimmed = trimmed;
        }
    }

    /**
     * Cut the line out of an over-wide run [start, end) using the prediction
     * (fPredX, fPredW). First try the "strong core": columns whose pixels are as
     * line-like as the calibrated line (a shadow or dark floor next to the line is
     * usually less so). Otherwise keep the edge that sits where the predicted
     * line edge is (the line merged with an object on its OTHER side).
     * Writes trimStart / trimEnd / trimSum / trimMoment; false if neither works.
     */
    private boolean trimBand(int start, int end) {
        final int[] colW = colWeight, colL = colLevel;
        final int rows = Math.max(1, hRoiBot - hRoiTop);
        final float predW = Math.max(4f, fPredW);
        // (1) strong core
        final float span = fLevelLine - fLevelThr;
        if (span >= 8f) {
            final float strong = fLevelThr + 0.6f * span;
            final int gapTol = runGapTol();
            int bestS = -1, bestE = -1; float bestD = Float.MAX_VALUE;
            int cs = -1, cl = -1;
            for (int c = start; c <= end; c++) {
                final boolean ok = c < end && colW[c] * 2 >= rows && colL[c] >= strong * colW[c];
                if (ok) { if (cs < 0) cs = c; cl = c; }
                else if (cs >= 0 && (c == end || c - cl > gapTol)) {
                    final int w = cl + 1 - cs;
                    final float d = Math.abs((cs + cl + 1) * 0.5f - fPredX);
                    if (w >= fMinW && w <= fMaxW && d <= Math.max(predW, 24f) && d < bestD) { bestD = d; bestS = cs; bestE = cl + 1; }
                    cs = -1;
                }
            }
            if (bestS >= 0) return sumRange(bestS, bestE);
        }
        // (2) the edge that continues the predicted line
        final float tol = Math.max(8f, 0.3f * predW);
        final boolean leftOk  = Math.abs(start - (fPredX - predW * 0.5f)) <= tol;
        final boolean rightOk = Math.abs(end   - (fPredX + predW * 0.5f)) <= tol;
        if (leftOk && !rightOk)  return sumRange(start, Math.min(end, start + Math.round(predW)));
        if (rightOk && !leftOk)  return sumRange(Math.max(start, end - Math.round(predW)), end);
        return false;
    }

    private boolean sumRange(int s, int e) {
        long sum = 0; double m = 0;
        for (int c = s; c < e; c++) { sum += colWeight[c]; m += (double) c * colWeight[c]; }
        trimStart = s; trimEnd = e; trimSum = sum; trimMoment = m;
        return sum > 0;
    }

    /**
     * How much a band looks like the line we want: its pixel count, halved at a
     * distance of {@code scale} from {@code prefX}, and scaled by
     * sqrt(narrower / wider) of its width vs {@code expectW} (ignored if 0).
     */
    private static float scoreBand(long sum, float centre, int width,
                                   float prefX, float scale, float expectW) {
        float s = sum;
        if (scale > 0f) {
            final float d = (centre - prefX) / scale;
            s /= 1f + d * d;
        }
        if (expectW > 0f && width > 0) {
            s *= (float) Math.sqrt(Math.min(width, expectW) / Math.max(width, expectW));
        }
        return s;
    }

    /** Expected line width in the H-ROI (px): calibrated, else as seen by colour learning, else 0. */
    private float expectedLineWidth() {
        return calibWidthPx > 0f ? calibWidthPx : learnedLineW;
    }

    /**
     * Fraction [0 … 1] of the look-ahead's floor distance covered by the line when
     * it reaches image row {@code row}: 0 at the V-ROI bottom, 1 at its top. On a
     * flat floor the distance to the point seen at row y grows as 1/(y − horizon),
     * so equal road lengths map to equal fractions (far rows cover more floor).
     */
    private float visibleRoadFrac(int row) {
        final float near = floorDistance(vRoiBot), far = floorDistance(vRoiTop);
        if (far - near < 1e-6f) return 1f;
        return Math.max(0f, Math.min(1f, (floorDistance(row) - near) / (far - near)));
    }

    /** Relative floor distance of the point seen at image row {@code row} (arbitrary units). */
    private float floorDistance(int row) {
        return 1f / Math.max(1e-3f, row / (float) imgH - horizonYFrac);
    }

    // A candidate must score this many times higher than the locked band to take over.
    private static final float RELOCK_RATIO = 1.5f;

    private int runGapTol() {
        return Math.max(2, imgW / 50);
    }

    // =========================================================================
    //  Hot path — zero heap allocations per frame
    // =========================================================================

    /**
     * Core pipeline. Reads single-channel pixels (row-major, rowStride = width)
     * where track pixels are bright when {@code bright} is true, and writes the
     * results into the internal {@code output} holder.
     */
    private void runPipeline(byte[] pixels, int width, int height, boolean bright) {
        curPixels = pixels;
        curBright = bright;
        final int   W     = width;
        final float halfW = W * 0.5f;
        final boolean filter = cfg.rejectDistractors;

        // ── Stage 1: Otsu adaptive threshold from the H-ROI histogram ──────────
        // Recalibrates every frame, so uneven / changing lighting is handled.
        final int threshold = stripThreshold(pixels, bright);

        // ── Stage 2: gated line-lock → steering ────────────────────────────────
        //
        //   Frame layout (row 0 = top = farthest floor, bottom = nearest robot):
        //
        //   row 0   ┌──────────────────────────────────┐
        //           │   [V-ROI] narrow column above    │
        //   hRoiTop ╠══════════[ H-ROI ]═══════════════╣  ← scan strip
        //   hRoiBot ╚══════════════════════════════════╝
        //
        // Instead of averaging EVERY track pixel across the strip (which lets a
        // reflection anywhere drag the centroid toward the middle), we collapse the
        // H-ROI into a per-column profile and lock onto ONE contiguous band — the
        // track. With track memory on we only scan a gate around the band's
        // PREDICTED position, so spots outside the gate are ignored and steering
        // can run all the way to the edge. The lock is held until the line has been
        // gone for trackLostGraceFrames (you have moved fully off it); only then do
        // we search the whole width for a new line.
        //
        // Distractor rejection (rejectDistractors): a dark chair, bag or shadow
        // passes the threshold just like the line, so a band only counts as the
        // line if its width matches the calibrated line width (corrected for the
        // line's slant) and its pixels are as line-like as the calibrated line.
        // A band that is the line MERGED with such an object is cut back to the
        // part that continues the predicted line.
        buildColumnProfile(pixels, threshold, bright);
        rejectedBands = 0;
        hardening = cfg.corridorMode && filter && calibrated && expectedLineWidth() > 0f;

        final int    hRoiRows  = Math.max(1, hRoiBot - hRoiTop);
        final double minWeight = (double) cfg.minWeightFrac * hRoiRows * W;
        final float  expectW   = expectedLineWidth();

        // Expected band width for the line's current slant in the strip: a line
        // crossing at image slope s (columns per row) is wider by sqrt(1+s²), plus
        // s × the strip height. The slope comes from the previous frame's fit.
        float slantW = expectW;
        if (filter && expectW > 0f) {
            final float s = Math.max(-3f, Math.min(3f, lastImgSlope));
            slantW = expectW * (float) Math.sqrt(1f + s * s) + hRoiRows * Math.abs(s);
        }
        setBandLimits(threshold, bright, filter ? slantW : 0f, filter ? expectW : 0f);

        // Search window: a gate around the remembered (predicted) line when locked,
        // otherwise the full width (fresh start, or re-acquiring after a loss).
        final boolean locked = cfg.trackMemory && trackAcquired;
        final float predX = (locked && filter)
                ? Math.max(0f, Math.min(W - 1f, lastCentroidX + centroidVel)) : lastCentroidX;
        fPredX = predX;
        fPredW = lastTrackWidthPx;
        int gateHalf = W;
        if (locked) {
            // Gate from the measured line width — with distractor rejection never
            // wider than an acceptable band, so it cannot grow to the whole frame.
            final float refW = (filter && expectW > 0f) ? Math.min(lastTrackWidthPx, fMaxW) : lastTrackWidthPx;
            gateHalf = Math.max(cfg.trackMemoryMinGatePx, Math.round(cfg.trackMemoryGateMult * refW));
            gateHalf = Math.min(gateHalf, W / 2);
        }
        final int gateC    = Math.round(predX);
        final int searchLo = locked ? Math.max(0, gateC - gateHalf) : 0;
        final int searchHi = locked ? Math.min(W, gateC + gateHalf) : W;
        // Candidate scoring (see scoreBand):
        //   • locked    — inside the gate, prefer the band nearest the predicted
        //                 position (with distractor rejection: on the scale of one
        //                 line width, with the expected width and line-ness).
        //   • searching — across the frame, prefer a band near the calibrated
        //                 straight-ahead column with the calibrated line width, so a
        //                 reflection or a neighbouring tape is not grabbed just
        //                 because it happens to be the biggest thing in view.
        final float lineScale  = W * 0.25f;
        if (locked) {
            if (filter) findBand(searchLo, searchHi, minWeight, predX,
                                 Math.max(24f, slantW > 0f ? slantW : lastTrackWidthPx), slantW, true, true);
            else        findBand(searchLo, searchHi, minWeight, lastCentroidX, gateHalf, 0f, false, false);
        } else {
            findBand(0, W, minWeight, steerRefX, lineScale, expectW, filter, false);
            // D1 (v3): (re)acquiring — the candidate that continues up the look-ahead wins.
            if (hardening && bandStart >= 0) acquireByContinuity(minWeight, lineScale, expectW, filter);
        }

        // D1 (v3): the locked line was hidden (coast) or lost — the band now in the gate may be
        // something else at the predicted place; rank it against the rest by continuity.
        if (locked && hardening && bandStart >= 0 && (coastFrames > 0 || lostFrames > 0)) {
            acquireByContinuity(minWeight, lineScale, expectW, filter);
        }

        // Lock correction: while locked, also look at the whole strip. If a band that
        // is clearly a better line candidate (by the "searching" score) sits outside
        // the locked band for trackRelockFrames frames in a row, the lock was on the
        // wrong thing — move it there. With distractor rejection the locked band must
        // ALSO look doubtful (trimmed / wrong width / not line-like), or the better
        // candidate must persist three times longer: a chair leg near the centre must
        // not steal the lock from the real line during a turn.
        if (locked && bandStart >= 0 && cfg.trackRelockFrames > 0) {
            final long    lockSum   = bandSum;
            final double  lockMom   = bandMoment;
            final int     lockStart = bandStart, lockEnd = bandEnd;
            final boolean lockTrim  = bandTrimmed;
            float lockScore = scoreBand(lockSum, (float) (lockMom / lockSum),
                                        lockEnd - lockStart, steerRefX, lineScale, expectW);
            boolean doubtful = true;
            if (filter) {
                final float lvl = levelFactor(bandLevel(lockStart, lockEnd));
                lockScore *= lvl;
                final float ratio = slantW > 0f ? (lockEnd - lockStart) / slantW : 1f;
                doubtful = lockTrim || lvl < 0.5f || ratio < 0.6f || ratio > 1.6f;
            }
            final int savedRejected = rejectedBands;
            findBand(0, W, minWeight, steerRefX, lineScale, expectW, filter, false);
            rejectedBands = savedRejected;
            final boolean better = bandStart >= 0
                    && (bandEnd <= lockStart || bandStart >= lockEnd)
                    && bandScore > RELOCK_RATIO * lockScore;
            final int need = doubtful ? cfg.trackRelockFrames : 3 * cfg.trackRelockFrames;
            if (better && ++relockCount >= need) {
                relockCount = 0;                          // adopt the candidate (already in band*)
                relockedThisFrame = true;
                centroidVel = 0f;
            } else {
                if (!better) relockCount = 0;
                bandSum = lockSum; bandMoment = lockMom; bandStart = lockStart; bandEnd = lockEnd;
                bandTrimmed = lockTrim;
            }
        } else {
            relockCount = 0;
        }

        // ── v3 corridor tracker (locked line, filter running) ──────────────────
        if (cfg.corridorMode && kfReady && locked && filter && calibrated) {
            final boolean sf = bandStart >= 0;
            if (corridorFrame(pixels, bright, threshold, sf, sf ? (int) (bandMoment / bandSum) : -1)) {
                output.threshold = threshold;
                return;
            }
        }
        ensureRows(vRoiTop, vRoiBot);              // v2 logic reads the whole look-ahead

        boolean trackFound;
        int     centroidX;
        boolean held               = false;
        float   forcedIntersection = -1f;   // ≥ 0 overrides the look-ahead stage
        float   pinnedSteering     = 0f;    // ±100 while the locked line is lost
        boolean reconstructed      = false; // v3: base hidden, rebuilt from the look-ahead
        boolean coastingNow        = false; // v3: line hidden mid-image, coasting
        boolean traced             = false; // look-ahead already traced this frame
        final boolean glare        = maskActive && filter;

        if (bandStart >= 0) {
            // Locked onto the line this frame.
            trackFound       = true;
            centroidX        = (int) (bandMoment / bandSum);
            if (filter) {
                // Short-term motion of the line (px/frame) for the next prediction;
                // a fresh lock starts at rest. Clamped so one odd frame cannot fling it.
                float d = locked ? centroidX - lastCentroidX : 0f;
                final float lim = 0.5f * Math.max(20f, slantW > 0f ? slantW : lastTrackWidthPx);
                d = Math.max(-lim, Math.min(lim, d));
                centroidVel = locked ? 0.5f * centroidVel + 0.5f * d : 0f;
            }
            lastCentroidX    = centroidX;
            lastTrackWidthPx = Math.max(1, bandEnd - bandStart);
            trackAcquired    = true;
            lostFrames       = 0;
            holdFrames       = 0;
            coastMs          = 0f;
            coastFrames      = 0;
            glarePinned      = false;
            resetPinCap();
        } else if (locked && glare) {
            // ── v3: the locked line is not visible in the strip (glare, shadow, object) ──
            // Follow it from the look-ahead instead, starting at the predicted column. The first
            // line row is searched across the whole width (nearest to the prediction wins), so a
            // line already curving off toward a frame edge is seen; the base is only rebuilt if
            // it lies within the lock gate of the prediction.
            traceLookAhead(pixels, lookAheadThreshold(pixels, threshold, bright), bright,
                           -1, predX, W, true);
            traced = true;
            final float edgeZone = Math.max(lastTrackWidthPx, 0.12f * W);
            // After a pin under glare the lock gate is parked at the frame edge; with the strip
            // blind, a consistent line ahead anywhere mid-image may be taken back.
            float recon = reconstructBase(predX, glarePinned ? W : gateHalf);
            // The corridor filter was tracking: a rebuilt base far from its prediction is a
            // guess (sharp curve seen from afar) — coast instead.
            if (!Float.isNaN(recon) && corrPredValid
                    && Math.abs(recon - corrPredCol) > Math.max(lastTrackWidthPx, 3f * corrPredSd)) recon = Float.NaN;
            // Taking the line back after a pin: only on the side it was pinned to.
            if (!Float.isNaN(recon) && glarePinned && (recon - steerRefX) * glarePinSide < 0f) recon = Float.NaN;
            // Edge exits beat coasting: the line ahead runs off a frame edge, or its
            // reconstructed base lies in the edge zone → pin to that edge at once (as v2).
            int exitSide = traceAccepted > 0 ? traceEdgeSide : 0;
            // The corridor filter was following a curve that itself reaches this frame edge at
            // the row where the line touches it: the curve explains it, not an exit → no pin.
            if (exitSide != 0 && corrPredValid && curveReachesEdge(traceFirstRow, exitSide, edgeZone)) exitSide = 0;
            if (exitSide == 0 && !Float.isNaN(recon)) {
                if (recon < edgeZone) exitSide = -1;
                else if (recon > W - edgeZone) exitSide = 1;
            }
            final boolean midImage = predX > edgeZone && predX < W - edgeZone;
            if (exitSide == 0 && !Float.isNaN(recon)) {
                trackFound    = true;
                reconstructed = true;
                glarePinned   = false;
                resetPinCap();
                centroidX     = Math.max(0, Math.min(W - 1, Math.round(recon)));
                float d = centroidX - lastCentroidX;
                final float lim = 0.5f * Math.max(20f, slantW > 0f ? slantW : lastTrackWidthPx);
                d = Math.max(-lim, Math.min(lim, d));
                centroidVel   = 0.5f * centroidVel + 0.5f * d;
                lastCentroidX = centroidX;
                lostFrames    = 0;
                holdFrames    = 0;
                coastMs       = 0f;
                coastFrames   = 0;
            } else {
                trackFound = false;
                coastMs += frameDtMs;
                coastFrames++;
                if (exitSide == 0 && midImage && cfg.glareCoastMs > 0f
                        && coastMs <= cfg.glareCoastMs && coastFrames <= cfg.glareCoastMaxFrames) {
                    // Hidden mid-image: coast on the predicted position (lock kept).
                    coastingNow   = true;
                    lastCentroidX = Math.round(predX);
                    centroidVel  *= 0.9f;
                    centroidX     = lastCentroidX;
                } else {
                    // Left at an edge, or the coast is used up: pin like v2.
                    lostFrames++;
                    final boolean onRight = exitSide != 0 ? exitSide > 0
                                                          : lastCentroidX >= Math.round(steerRefX);
                    centroidX     = onRight ? Math.min(W - 1, Math.round(steerRefX + halfW))
                                            : Math.max(0,     Math.round(steerRefX - halfW));
                    lastCentroidX = centroidX;
                    centroidVel   = 0f;
                    pinnedSteering     = exitSide != 0 ? glarePinSteering(exitSide)
                                       : (glarePinned && !pinCapEscalated) ? glarePinSteering(glarePinSide)
                                       : (onRight ? 100f : -100f);
                    forcedIntersection = 100f;
                    coastMs     = cfg.glareCoastMs;              // no new coast until the line is seen
                    coastFrames = cfg.glareCoastMaxFrames;
                    glarePinned = true;
                    glarePinSide = onRight ? 1 : -1;
                }
                if (lostFrames > cfg.trackLostGraceFrames) trackAcquired = false;
            }
        } else if (locked) {
            trackFound  = false;
            lostFrames++;
            // Where the line should be now. If that is well inside the image, the
            // line did not leave the view — it is hidden (shadow, glare, an object
            // over it) or briefly unreadable: hold the last output for a few frames.
            final float edgeZone = Math.max(lastTrackWidthPx, 0.12f * W);
            final boolean midImage = filter && predX > edgeZone && predX < W - edgeZone;
            if (midImage && holdFrames < cfg.trackHoldFrames) {
                holdFrames++;
                held          = true;
                lastCentroidX = Math.round(predX);
                centroidVel  *= 0.8f;
                centroidX     = lastCentroidX;
            } else {
                // The line we were following just left the gate → you have moved fully
                // off it. Pin steering hard toward the edge it exited and flag a 100 %
                // turn. Keep the gate parked at that edge so the SAME line re-locks if
                // it comes back; only after the grace period do we hunt a new one.
                // Pin toward the edge it exited, RELATIVE to the calibrated centre. The
                // steering is set to exactly ±100 below (the clamped pixel column alone
                // would read less than 100 when the calibrated centre is off-centre).
                final boolean onRight = lastCentroidX >= Math.round(steerRefX);
                centroidX     = onRight ? Math.min(W - 1, Math.round(steerRefX + halfW))
                                        : Math.max(0,     Math.round(steerRefX - halfW));
                lastCentroidX = centroidX;
                centroidVel   = 0f;
                pinnedSteering     = onRight ? 100f : -100f;
                forcedIntersection = 100f;
            }
            if (lostFrames > cfg.trackLostGraceFrames) {
                trackAcquired = false;     // release lock → full-width re-acquire next frame
            }
        } else {
            // No lock and nothing found — blank view; hold neutral.
            trackFound = false;
            centroidX  = lastCentroidX;
        }

        // (Steering & track values are computed after Stage 3, once the look-ahead
        //  position is known.)

        // ── Stage 3: look-ahead along the line (the vertical part of the ⊥) ────
        if (!traced) {
            final int vThreshold = lookAheadThreshold(pixels, threshold, bright);
            traceLookAhead(pixels, vThreshold, bright, trackFound ? centroidX : -1, filter);
        }
        laPathCount = traceLaCount;   // publish the path for any external overlay
        final int    farRow = traceFarRow;
        final double slope  = traceSlope;
        if (trackFound && traceFitN >= 2) lastImgSlope = (float) -traceNearSlope;   // columns per row, upward, near the strip

        // 3b. Line tilt from the least-squares slope  b = d(col)/d(row) = tan(θ).
        //     Rows grow downward, so a line leaning right has a NEGATIVE slope.
        //     (Image-space; for the overlay. Steering uses the floor heading below.)
        final float tiltDeg = (float) Math.toDegrees(Math.atan(-slope));

        // 3c. Look-ahead line position (band-locked, far-weighted centroid).
        final boolean lookAheadValid = (traceLaW > 0.0);
        final float   lookAheadX     = lookAheadValid ? (float) (traceLaM / traceLaW) : steerRefX;

        // ── Steering: offset of the ⊥ base + heading of the whole ⊥ ─────────────
        //   offset  = (centroid − calibrated column) / half width × 100      (v1)
        //   heading = 100 × L × tan(ψ − ψ₀) / (floor half-width at the H-ROI)
        // ψ is the direction of the line ON THE FLOOR relative to the robot, where it
        // crosses the H-ROI: a parabola is fitted through the H-ROI centroid and the
        // near look-ahead rows on the line (perspective removed with the camera
        // geometry), see computeHeading. ψ₀ is the calibrated zero and
        // L = steerLookAhead × floor distance from the camera to the H-ROI. The sum
        // is the lateral position, in v1 steering units, of a pure-pursuit target
        // point L further along the line. The heading part fades out
        // (headingConfidence) when too little of the line is visible ahead or the
        // line is not a smooth curve (a corner), leaving the v1 offset-only steering.
        final float offsetTerm = (centroidX - steerRefX) / halfW * 100f;
        float headingTerm = 0f, headingDeg = 0f, conf = 0f;
        if (trackFound && cfg.steerHeadingWeight != 0f) {
            computeHeading();
            conf = fitConfidence;
            if (conf > 0f) {
                headingDeg = fitHeadingDeg - headingRefDeg;
                final double psi = Math.toRadians(Math.max(-75f, Math.min(75f, headingDeg)));
                final double lookAhead = cfg.steerLookAhead * floorFwdH;
                final double halfFloor = floorScaleH * halfW;
                headingTerm = (float) (100.0 * lookAhead * Math.tan(psi) / Math.max(1e-6, halfFloor));
            }
        }
        float steering = (pinnedSteering != 0f) ? pinnedSteering
                : Math.max(-100f, Math.min(100f, cfg.steerOffsetWeight * offsetTerm
                                                + cfg.steerHeadingWeight * conf * headingTerm));

        // 3d. Intersection — internal scale (0 = straight, 100 = hard turn / lost).
        //     Inverted to the TRACKING output in processBound().
        float intersection;
        if (!trackFound || !lookAheadValid) {
            intersection = 100f;                          // no road ahead → off
        } else {
            final float span = Math.max(1f, halfW * cfg.trackOffsetFullFrac);
            intersection = Math.max(0f, Math.min(100f,
                    Math.abs(lookAheadX - trackRefX) / span * 100f));
            // Road-length part: less visible road ahead → closer to a full stop,
            // linear in floor distance (a dead end or a sharp corner ramps down).
            if (cfg.trackLengthSlowdown && farRow >= 0) {
                final float full   = Math.max(1e-3f, cfg.trackLengthFullFrac);
                final float length = Math.min(1f, visibleRoadFrac(farRow) / full);
                intersection = Math.max(intersection, 100f * (1f - length));
            }
        }

        // D2 bound (user decision, plan §13): the look-ahead dropped a stretch cut from a mass of
        // the line's colour, but corridor slices beyond it still show the line's two edges on its
        // continuation — a blob beside a line that goes on, not a line ending: slow down, but
        // not below tracking 60.
        if (trackFound && hardening && traceCutRow >= 0 && intersection > 40f
                && continuity(centroidX, lastImgSlope, traceCutRow) >= 2) {
            intersection = 40f;
        }

        // Track-memory override: once the locked line has left the gate we report a
        // hard turn (100 %) regardless of the look-ahead geometry, signalling that
        // you have moved fully off the line and a new one is being sought.
        if (forcedIntersection >= 0f) intersection = forcedIntersection;

        // v3 coast: steering from the predicted position plus the last heading part, which
        // fades out over the coast; tracking ramps from its last value toward the minimum.
        if (coastingNow) {
            final float used = Math.min(1f, Math.max(coastMs / Math.max(1e-3f, cfg.glareCoastMs),
                    coastFrames / (float) Math.max(1, cfg.glareCoastMaxFrames)));
            steering = Math.max(-100f, Math.min(100f,
                    cfg.steerOffsetWeight * offsetTerm + (1f - used) * lastHeadTerm));
            intersection = lastIntersection + (100f - lastIntersection) * used;
            // Line still seen farther ahead (ambiguous sharp curve): never faster than that
            // observation allows (its lean, visible length and curvature).
            if (traceAccepted > 0) {
                intersection = Math.max(intersection, farIntersection(
                        lookAheadValid ? lookAheadX : Float.NaN, farRow, floorFitC()));
            }
        }

        // Line momentarily hidden mid-image: hold the last good outputs.
        if (held) {
            steering     = lastSteering;
            intersection = lastIntersection;
        } else if (trackFound) {
            lastSteering     = steering;
            lastIntersection = intersection;
            lastHeadTerm     = cfg.steerHeadingWeight * conf * headingTerm;
        }

        // ── Write internal output ──────────────────────────────────────────────
        output.steering     = steering;
        output.intersection = intersection;
        output.centroidX    = trackFound ? centroidX : -1;
        output.tiltDeg      = trackFound ? tiltDeg : 0f;
        output.trackWidthPx = lastTrackWidthPx;   // measured width, held on track loss
        output.trackFound   = trackFound;
        output.threshold    = threshold;
        output.offsetTerm   = trackFound ? cfg.steerOffsetWeight * offsetTerm : 0f;
        output.headingTerm  = trackFound ? cfg.steerHeadingWeight * conf * headingTerm : 0f;
        output.headingDeg   = headingDeg;
        output.headingConf  = conf;
        output.held         = held || coastingNow;
        output.rejected     = rejectedBands;
        output.coasting     = coastingNow;
        output.reconstructed = reconstructed;
        if (cfg.corridorMode) corridorSync(trackFound, centroidX, coastingNow || held || pinCapActive, conf);
    }

    /** True when the filter's curve (as predicted this frame) is in the edge zone of that side at that row. */
    private boolean curveReachesEdge(int row, int side, float edgeZone) {
        if (row < vRoiTop || row >= vRoiBot || Float.isNaN(rowFwd[row])) return false;
        final double u = rowFwd[row] - floorFwdH;
        final double x = kxPred[0] + kxPred[1] * u + kxPred[2] * u * u;
        final float col = (float) (imgW * 0.5f - 0.5f + x / rowScale[row]);
        return side < 0 ? col <= edgeZone : col >= imgW - edgeZone;
    }

    // A capped pin turns into the full v2 pin after this long (5 frames at 30 fps).
    private static final float PIN_CAP_MS = 5000f / 30f;
    private static final int   PIN_CAP_MIN_CURVE_FRAMES = 3, PIN_CAP_MAX_GAP = 6;

    /**
     * Steering (internal, ±100) for a pin toward {@code side} under glare. If the corridor filter
     * had been following a curve toward that edge with confidence over the last frames, the
     * line running off the edge is most likely that curve, not the robot leaving the line:
     * steering is capped at the filter's own predicted steering (offset + heading + curvature,
     * clamped) instead of ±100. After PIN_CAP_MS (or 5 frames without a clock) it escalates to
     * the full pin. Without that curve history (a real overshoot) it is the full pin at once.
     */
    private float glarePinSteering(int side) {
        final float full = side > 0 ? 100f : -100f;
        final float stripPx = Math.max(1f, (hRoiBot - hRoiTop) * (float) imgW);
        if (pinCapEscalated || !corrPredValid || curveSide != side || maskedPixels < 0.2f * stripPx
                || curveFrames < PIN_CAP_MIN_CURVE_FRAMES || sinceCurveFrames > PIN_CAP_MAX_GAP) return full;
        pinCapMs += frameDtMs;
        pinCapFrames++;
        final boolean timeUp = frameDtMs > 0f ? pinCapMs > PIN_CAP_MS + 1e-3f : pinCapFrames > 5;
        if (timeUp || pinCapFrames > 10) { pinCapEscalated = true; return full; }
        final float pred = predictedSteering(pinTargetU());
        if (pred * side <= 0f) return full;
        pinCapActive = true;
        sinceCurveFrames = 0;            // still following that curve: keep its history fresh
        // Never the full ±50 (internal ±100): that stays the signal "the line left at the edge".
        return Math.max(-96f, Math.min(96f, pred));
    }

    /**
     * The filter's predicted steering for the capped pin: pure pursuit from the camera to the
     * filter's curve point at forward distance u beyond the H-ROI (where the line is still seen,
     * so the point is backed by the far measurements; heading and curvature enter through the
     * curve), expressed as the lateral position the arc reaches at the v2 target distance
     * (H-ROI + steerLookAhead), in v2 steering units. The hidden near position is not used.
     * In the simulated sharp curves this reads ~10–15 below v2 on the clean view (the base has
     * moved sideways along the curve while hidden) — always the same direction.
     */
    private float predictedSteering(double u) {
        final float halfW = imgW * 0.5f;
        final double halfFloor = Math.max(1e-6, floorScaleH * halfW);
        final double xRef = (steerRefX - (halfW - 0.5f)) * floorScaleH;          // calibrated straight ahead
        final double xt = kxPred[0] + kxPred[1] * u + kxPred[2] * u * u - xRef;   // curve point, lateral
        final double zt = floorFwdH + u;
        final double target = floorFwdH * (1.0 + cfg.steerLookAhead);
        return (float) (100.0 * xt * (target * target) / Math.max(1e-6, zt * zt) / halfFloor);
    }

    /** Distance (beyond the H-ROI) of the nearest line row of the last look-ahead trace, else FIT_DEPTH. */
    private double pinTargetU() {
        final int r = traceFirstRow;
        if (traceAccepted > 0 && r >= vRoiTop && r < vRoiBot && !Float.isNaN(rowFwd[r])) return rowFwd[r] - floorFwdH;
        return FIT_DEPTH * floorFwdH;
    }

    /** The line was seen (strip, rebuilt base or corridor): no pin cap in progress. */
    private void resetPinCap() {
        pinCapMs = 0f; pinCapFrames = 0; pinCapEscalated = false;
    }

    /** Corridor frame: remember which way the filter's curve bends (by at least one line width at FIT_DEPTH). */
    private void noteCurve(double wf, boolean confident) {
        final double uF = FIT_DEPTH * floorFwdH;
        final double lean = kx[1] * uF + kx[2] * uF * uF;
        final int side = !confident || Math.abs(lean) < wf ? 0 : (lean > 0 ? 1 : -1);
        if (side == 0) { curveFrames = 0; curveSide = 0; return; }
        if (side == curveSide) curveFrames++; else { curveSide = side; curveFrames = 1; }
        sinceCurveFrames = 0;
    }

    // ── Phase 3 helpers (distractor hardening) ─────────────────────────────

    /**
     * How many corridor slices (1 … cN−1) continue a line that crosses the strip at column
     * {@code col}: each slice is searched around the continuation (direction {@code slope0},
     * columns per row upward, refined by the slices found) with a window that grows with the
     * distance; it needs an edge pair at the expected width. Misses do not stop the search, so
     * a line merged with a blob over a stretch still counts once it is clean again. Glare
     * slices count as neither.
     */
    private int continuity(float col, float slope0) {
        return continuity(col, slope0, Integer.MAX_VALUE);
    }

    /** As above, counting only slices farther than image row {@code beyondRow} (rows above it). */
    private int continuity(float col, float slope0, int beyondRow) {
        final byte[] pix = curPixels;
        if (pix == null || cN < 2) return 0;
        final float expW = expectedLineWidth();
        if (expW <= 0f) return 0;
        final int runGap = runGapTol();
        float lastCol = col, slope = Math.max(-2f, Math.min(2f, slope0));
        int lastRow = cRow[0], found = 0, seen = 0;
        for (int s = 1; s < cN; s++) {
            final int row = cRow[s];
            final float pc = lastCol + slope * (lastRow - row);
            if (pc < -expW || pc > imgW + expW) break;
            final float e = expW * floorScaleH / cScale[s] * cGain[s];
            final float half = Math.max(1.1f * e, 0.5f * e + 2 * runGap) + 0.15f * (cRow[0] - row);
            ensureRows(cR0[s], cR1[s]);
            if (detectSlice(pix, curBright, cR0[s], cR1[s], 1, pc, e, Math.min(half, imgW * 0.5f), 1.5f)) {
                if (Math.abs(dCentre - pc) <= Math.max(0.5f * e, half * 0.6f)) {
                    if (seen > 0 && lastRow != row) slope = Math.max(-2f, Math.min(2f, (dCentre - lastCol) / (lastRow - row)));
                    lastCol = dCentre; lastRow = row; seen++;
                    if (row < beyondRow) found++;
                }
            }
        }
        return found;
    }

    /**
     * In how many of the 3 slices above the strip the continuation of column {@code col} is
     * covered by line-coloured pixels (mean line-ness past the threshold over one line width)
     * — e.g. the mat or blob a line ran into. Empty floor above (a corner) gives 0.
     */
    private int coveredAbove(float col, float slope0) {
        final byte[] pix = curPixels;
        if (pix == null) return 0;
        final float expW = expectedLineWidth();
        final float slope = Math.max(-2f, Math.min(2f, slope0));
        int n = 0;
        for (int s = 1; s < Math.min(cN, 4); s++) {
            final float e = expW * floorScaleH / cScale[s];
            final float pc = col + slope * (cRow[0] - cRow[s]);
            final int a = Math.max(0, Math.round(pc - 0.5f * e)), b = Math.min(imgW, Math.round(pc + 0.5f * e) + 1);
            if (b <= a) continue;
            ensureRows(cR0[s], cR1[s]);
            long sum = 0; int cnt = 0;
            for (int r = cR0[s]; r < cR1[s]; r++) {
                for (int c = a; c < b; c++) { final int v = pix[r * imgW + c] & 0xFF; sum += curBright ? v : 255 - v; cnt++; }
            }
            if (cnt > 0 && sum >= (long) (fLevelThr + 4f) * cnt) n++;
        }
        return n;
    }

    /**
     * D2: is the run [a, b) on this row uniformly line-like? Compares the tape-width part at the
     * kept edge (left if {@code keepLeft}) with the rest: a line beside a lighter shadow has a
     * clearly more line-like core; a mat or blob of the line's colour does not.
     */
    private boolean uniformRun(int base, int a, int b, boolean keepLeft, float expRow, boolean bright, float lvlThr) {
        final int k = Math.max(1, Math.round(expRow));
        final int ka = keepLeft ? a : Math.max(a, b - k), kb = keepLeft ? Math.min(b, a + k) : b;
        final boolean mask = maskActive;
        final byte[] gs = glareSrc, pix = curPixels;
        final int gl = glareLim;
        long sk = 0, sr = 0; int nk = 0, nr = 0;
        for (int c = a; c < b; c++) {
            if (mask && (gs[base + c] & 0xFF) >= gl) continue;
            final int v = pix[base + c] & 0xFF, l = bright ? v : 255 - v;
            if (c >= ka && c < kb) { sk += l; nk++; } else { sr += l; nr++; }
        }
        if (nk == 0 || nr == 0) return true;
        final float span = calibLevel - lvlThr;
        final float margin = span >= 8f ? 0.25f * span : 12f;
        return (float) sk / nk - (float) sr / nr < margin;
    }

    /** True when the strip rows show both edges of a band [start, end) (an edge pair at its width). */
    private boolean hasEdgePair(int start, int end) {
        final byte[] pix = curPixels;
        if (pix == null) return true;
        final float wid = end - start, c = 0.5f * (start + end);
        final float half = 0.5f * wid + 2 * runGapTol() + 2f;
        if (!detectSlice(pix, curBright, cR0[0], cR1[0], 1, c, wid, half, 1.5f)) return dUnknown;
        return Math.abs(dCentre - c) <= Math.max(2f, 0.35f * wid);
    }

    /**
     * D1: (re)acquiring the line. Besides the best band (already in band*), the best band left
     * and right of it are candidates; the one that continues farthest up the look-ahead (at
     * least 2 slices) wins, ties broken by the v2 score. If none continues, the v2 choice stays.
     */
    private void acquireByContinuity(double minWeight, float lineScale, float expectW, boolean filter) {
        final long   s0 = bandSum;  final double m0 = bandMoment;
        final int    a0 = bandStart, e0 = bandEnd;  final float sc0 = bandScore;  final boolean t0 = bandTrimmed;
        long bs = s0; double bm = m0; int ba = a0, be = e0; boolean bt = t0;
        int bestCont = continuity((float) (m0 / s0), 0f);
        float bestScore = sc0;
        final int savedRejected = rejectedBands;
        for (int side = 0; side < 2; side++) {
            if (side == 0) findBand(0, a0, minWeight, steerRefX, lineScale, expectW, filter, false);
            else           findBand(e0, imgW, minWeight, steerRefX, lineScale, expectW, filter, false);
            if (bandStart < 0) continue;
            final int cont = continuity((float) (bandMoment / bandSum), 0f);
            final boolean better = cont >= 2 && (cont > bestCont + 1 || (Math.abs(cont - bestCont) <= 1 && bandScore > bestScore && bestCont >= 2))
                                   || (bestCont < 2 && cont >= 2);
            if (better) {
                bestCont = cont; bestScore = bandScore;
                bs = bandSum; bm = bandMoment; ba = bandStart; be = bandEnd; bt = bandTrimmed;
            }
        }
        rejectedBands = savedRejected;
        bandSum = bs; bandMoment = bm; bandStart = ba; bandEnd = be; bandTrimmed = bt; bandScore = bestScore;
    }

    /** Curvature C of a parabola through the last trace's floor points (NaN if too few). */
    private float floorFitC() {
        if (floorN < 6 || floorSpan < 0.3f * floorFwdH) return Float.NaN;
        final double n = floorN, a12 = fU, a13 = fUU, a22 = fUU, a23 = fUUU, a33 = fUUUU;
        final double det = n * (a22 * a33 - a23 * a23) - a12 * (a12 * a33 - a23 * a13) + a13 * (a12 * a23 - a22 * a13);
        if (Math.abs(det) < 1e-12) return Float.NaN;
        final double detC = n * (a22 * fUUX - fUX * a23) - a12 * (a12 * fUUX - fUX * a13) + fX * (a12 * a23 - a22 * a13);
        return (float) (detC / det);
    }

    /**
     * Stage 3 threshold: adaptive to the (dimmer, farther) look-ahead.
     * The far look-ahead line is dimmer than the near line, so the H-ROI Otsu
     * can miss it — the tracker then loses the line early and the track value
     * plateaus. Build a subsampled histogram of the whole V-ROI band and run
     * Otsu again to get a threshold matched to the look-ahead's own lighting.
     */
    private int lookAheadThreshold(byte[] pixels, int threshold, boolean bright) {
        int vThreshold = threshold;
        if (cfg.vRoiAdaptiveThreshold && vRoiBot > vRoiTop) {
            final int W = imgW;
            final int[] histV = histogramV;
            java.util.Arrays.fill(histV, 0);
            final int stepR = Math.max(1, (vRoiBot - vRoiTop) / 240);   // ~240 rows sampled
            final int stepC = Math.max(1, W / 240);                     // ~240 cols sampled
            long cntV = 0;
            final byte[] gs = glareSrc;
            final int gl = glareLim;
            final boolean mask = maskActive;
            for (int row = vRoiTop; row < vRoiBot; row += stepR) {
                final int base = row * W;
                for (int col = 0; col < W; col += stepC) {
                    if (mask) {
                        maskedSamples++;
                        if ((gs[base + col] & 0xFF) >= gl) { maskedSampleHits++; continue; }   // glare: neither line nor floor
                    }
                    histV[pixels[base + col] & 0xFF]++; cntV++;
                }
            }
            final int t = finishThreshold(otsuThreshold(histV, cntV), bright);
            if (t != NO_TRACK) vThreshold = t;   // flat look-ahead → keep the strip threshold
        }
        return vThreshold;
    }

    /**
     * Stage 3: follow the line row by row up through the V-ROI from the H-ROI
     * centroid {@code centroidX} (-1 = no line: nothing traced). Results go to the
     * trace* fields (image fit, far row, far-weighted look-ahead centroid, overlay
     * path) and the floor fit (floorN, fU…, floorSpan) used for the heading.
     *
     * We TRACK the line up through the V-ROI, recentring the scan window on
     * the previous row's centroid, so curves and a skewed approach are
     * followed instead of leaking out of a fixed column. Only rows where the
     * line is actually VISIBLE are used; empty far-away rows are never counted.
     *
     * With {@code filter} (distractor rejection) a row's run must also not be
     * wider than the line can be at that row (calibrated width scaled by the
     * perspective and the line's slant). A wider run — the line merged with a
     * nearby dark object — keeps only the edge that continues the line, or the
     * row counts as a gap, so the tracker never jumps onto the object. The
     * window is also re-centred on the line's PREDICTED column (previous row +
     * its local drift), which follows curves more closely.
     */
    private void traceLookAhead(byte[] pixels, int vThreshold, boolean bright, int centroidX, boolean filter) {
        traceLookAhead(pixels, vThreshold, bright, centroidX, -1f, 0, filter);
    }

    /**
     * As above; with {@code centroidX} = -1 and {@code seedX} ≥ 0 (v3, base hidden by glare) the
     * trace starts without an H-ROI anchor from the predicted column {@code seedX}: until the
     * first line row is found the window and the allowed jump are widened to {@code seedHalf}
     * and more empty rows are tolerated. With the glare mask on, saturated pixels count as part
     * of a run, rows where the predicted line is mostly masked are skipped (not counted as a
     * gap, so glare is not taken for the end of the road), and traceEdgeSide records whether
     * the nearest line row runs off the left / right frame edge.
     */
    private void traceLookAhead(byte[] pixels, int vThreshold, boolean bright, int centroidX,
                                float seedX, int seedHalf, boolean filter) {
        final int     W      = imgW;
        final int     half   = cfg.vRoiHalfWidthPx;
        final float[] rowCx  = this.rowCx;
        final int     maxGap = Math.max(8, cfg.hRoiThickPx);   // stop after this many empty rows
        final int     seekGap = Math.max(maxGap, (vRoiBot - vRoiTop) / 3);
        final int     runGapTol = runGapTol();
        final boolean anchored   = centroidX >= 0;
        final boolean seeded     = !anchored && seedX >= 0f;
        final boolean trackFound = anchored || seeded;           // trace at all
        boolean       seeking    = seeded;                       // no line row found yet (seeded)
        final boolean mask       = maskActive;
        final byte[]  gs         = glareSrc;
        final int     gl         = glareLim;
        traceEdgeSide = 0;
        traceAccepted = 0;
        nfN = 0; nfU = 0; nfUU = 0; nfX = 0; nfUX = 0; nfXX = 0; nfLimit = 0;

        // Per-row line-lock limits (same memory idea as Stage 2, applied upward):
        //   • minRowW   — a row's run must be at least this many pixels to count,
        //                 so 1-px specular noise never seeds the tracker.
        //   • vMaxDrift — the run's centre may sit at most this far from where the
        //                 line was on the row below. A real line drifts only a few
        //                 px per row even on curves, so this is what stops the
        //                 tracker from JUMPING onto a separate object (table)
        //                 that happens to fall inside the scan window.
        final int minRowW   = Math.max(1, Math.round(0.25f * lastTrackWidthPx));
        final int vMaxDrift = Math.max(8, Math.round(0.75f * lastTrackWidthPx));
        // Distractor rejection: the line's expected width at each row is the
        // calibrated H-ROI width scaled by perspective (rowScale is inversely
        // proportional to apparent width).
        final float expectW = expectedLineWidth();
        final boolean widthTest = filter && expectW > 0f && floorScaleH > 0f;

        // Least-squares accumulators over rows with a detected centroid (y = row).
        int    fitN = 0;
        double sY = 0.0, sYY = 0.0, sC = 0.0, sYC = 0.0;
        // Same fit restricted to the rows just above the H-ROI: the line's slant where
        // it crosses the strip, for the strip's width test (far rows may curve away).
        final int nearRow = vRoiBot - Math.max(40, 4 * cfg.hRoiThickPx);
        int    nN = 0;
        double nY = 0.0, nYY = 0.0, nC = 0.0, nYC = 0.0;
        // Floor-plane fit: lateral X vs forward Z (camera heights).
        floorN = 0; fU = 0; fUU = 0; fUUU = 0; fUUUU = 0; fX = 0; fUX = 0; fUUX = 0; fXX = 0;
        fUmin = Float.MAX_VALUE; fUmax = -Float.MAX_VALUE;
        final float cx0 = W * 0.5f - 0.5f;   // column whose centre is the optical axis

        // Anchor the fit with the H-ROI centroid (longest, most reliable baseline).
        if (anchored) {
            final double yH = (hRoiTop + hRoiBot) * 0.5;
            fitN++; sY += yH; sYY += yH * yH; sC += centroidX; sYC += yH * centroidX;
            nN++;   nY += yH; nYY += yH * yH; nC += centroidX; nYC += yH * centroidX;
            final float z = floorFwdH, x = floorScaleH * (centroidX - cx0);
            addFloorPoint(z, x);
        }

        // Track the line from the bottom of the V-ROI upward, following drift.
        // Each row we lock onto the contiguous RUN closest to the line's
        // position on the row below — never the average of everything in the
        // window — and reject any run that is too small or too far (a different
        // object). Stop once the line has been missing for maxGap rows (end /
        // off the road), so empty frame above is never mistaken for the track.
        int    trackCenter = anchored ? centroidX : Math.round(seedX);
        float  drift       = filter ? Math.max(-2f, Math.min(2f, lastImgSlope)) : 0f;  // columns per row, upward
        // Width factor for the line's slant, from the previous frame (fixed for this
        // frame, so a distractor cannot inflate the allowed width row after row).
        final float slantK = (float) Math.sqrt(1f + drift * drift);
        double prevC       = anchored ? centroidX : seedX;
        int    prevRow     = (hRoiTop + hRoiBot) / 2;
        int    gap         = 0;
        int    farRow      = -1;   // farthest (topmost) V-ROI row where the line was accepted
        int    farRowEff   = -1;   // same, extended across glare rows right after it (road not ended)
        int    lastGlareRow = -1;  // seeking: last row whose window touched glare
        // D2 (v3): rows kept only through one edge of a wider run ("cut"); a trailing stretch of
        // them with no clean row after it is the edge of something the line ran into.
        int    cutRows = 0, lastCleanRow = -1;
        double cleanLaW = 0, cleanLaM = 0;
        boolean inCutStretch = false;
        int    cutStartRow = -1;
        float  prevSpan = 0f, prevExp = lastTrackWidthPx;
        final boolean d2 = filter && cfg.corridorMode && calibrated;
        final int haloRows = Math.max(4, Math.round(0.03f * imgH));
        long   runL = 0; int runN = 0;   // line-ness sum / count of the unmasked pixels of a run
        // "Strongly line-like": past 60 % of the way from the threshold to the calibrated line.
        final float lvlThr = vThreshold == NO_TRACK ? 0f : (bright ? vThreshold : 255 - vThreshold);
        final float strongLevel = calibLevel - lvlThr >= 8f ? lvlThr + 0.6f * (calibLevel - lvlThr) : 0f;
        // Look-ahead position accumulators: a centroid of the band-locked line over
        // the V-ROI, weighting FARTHER-ahead rows more so the value previews the
        // road in front of the robot rather than what is right under it.
        double laW = 0.0, laM = 0.0;

        // Dynamic look-ahead path (nearest → farthest) for the overlay to follow.
        // Seed it with the near H-ROI centroid so the drawn band starts on the line,
        // then push one point every laStride accepted rows (subsampled to LA_MAX_PTS).
        int laCount = 0;
        final int laStride = Math.max(1, (vRoiBot - vRoiTop) / Math.max(1, LA_MAX_PTS - 2));
        int laNext = 0;
        if (anchored) {
            laPathRowFrac[laCount] = (float) ((hRoiTop + hRoiBot) * 0.5) / imgH;
            laPathColFrac[laCount] = centroidX / (float) imgW;
            laCount++;
        }

        for (int row = vRoiBot - 1; row >= vRoiTop && trackFound; row--) {
            // Predicted centre on this row: the row below, plus (filter) the drift.
            final double pred = filter ? prevC + drift * (prevRow - row) : trackCenter;
            final int    pc   = filter ? (int) Math.round(pred) : trackCenter;
            // Expected width of the line on this row (filter only).
            float expRow = 0f, maxRow = Float.MAX_VALUE;
            int   rowHalf = half;
            if (widthTest) {
                expRow = expectW * floorScaleH / rowScale[row] * slantK;
                maxRow = cfg.bandMaxWidthMult * expRow + 4f;
                // The window must leave room beside the line (more than the gap that
                // is bridged), else a line merged with anything touches both window
                // edges and its true edges cannot be seen. Wide tape at high
                // resolution can be wider than 2 × vRoiHalfWidthPx.
                rowHalf = Math.max(half, (int) Math.ceil(0.5f * expRow) + 2 * runGapTol);
            }
            if (seeking) rowHalf = Math.max(rowHalf, seedHalf);
            final int vLeft  = Math.max(0,     pc - rowHalf);
            final int vRight = Math.min(W, pc + rowHalf);
            final int base   = row * W;

            // Seeking the line next to a glare patch: rows that touch the glare, and the halo just
            // above it (brightened but not saturated), can show rims that only just pass the
            // threshold and look like a line. There a run must be as line-like as the line
            // itself (strongLevel) to start the trace.
            boolean nearGlare = false;
            if (seeking && mask) {
                // Sparse sample of the (whole-width) window: touched by glare / nearly all glare.
                int hits = 0, n = 0;
                for (int col = vLeft; col < vRight; col += 8, n++) if ((gs[base + col] & 0xFF) >= gl) hits++;
                if (hits > 0) lastGlareRow = row;
                if (n > 0 && 10 * hits >= 9 * n) { rowCx[row] = Float.NaN; continue; }   // glare row: unknown
                nearGlare = lastGlareRow >= 0 && lastGlareRow - row <= haloRows;
            }

            // Scan the window, picking the run whose centre is nearest the
            // predicted centre. Small gaps inside a run are bridged.
            long   runW = 0;   double runM = 0;   int runStart = -1, runLast = -1, runGap = 0;
            double bestC = Double.NaN; long bestW = 0; double bestDist = Double.MAX_VALUE;
            int    bestS = -1, bestE = -1;
            boolean bestCut = false;
            int     bestSpan = 0;
            for (int col = vLeft; col <= vRight; col++) {
                // Glare (v3): a saturated pixel is "unknown" and counts as part of a run.
                final boolean masked = mask && col < vRight && (gs[base + col] & 0xFF) >= gl;
                final int     px     = col < vRight ? pixels[base + col] & 0xFF : 0;
                final boolean on = col < vRight && (masked || isTrack(px, vThreshold, bright));
                if (on) {
                    if (runStart < 0) { runStart = col; runW = 0; runM = 0; runL = 0; runN = 0; }
                    runW++; runM += col; runGap = 0; runLast = col;
                    if (!masked) { runN++; runL += bright ? px : 255 - px; }
                } else if (runStart >= 0 && (col == vRight || ++runGap > runGapTol)) {
                    double c = runM / runW; long wCount = runW;
                    boolean cut = false;
                    boolean ok = !nearGlare || (strongLevel > 0f && 2 * runN >= runW && runL >= strongLevel * runN);
                    final int span = runLast + 1 - runStart;
                    // A run cut by the scan window (not the frame edge) may be much wider.
                    final boolean clipL = runStart == vLeft && vLeft > 0, clipR = runLast + 1 == vRight && vRight < W;
                    final boolean clipped = clipL || clipR;
                    if (widthTest && clipL && clipR) ok = false;          // fills the whole window: not the line
                    else if (widthTest && (span > 1.3f * expRow || clipped)) {
                        // Wider than the line: the line merged with an object (which
                        // would make the centre creep toward it row after row). Keep
                        // the edge that continues the line; if neither edge does, use
                        // the run only if it is not too wide, else skip the row.
                        final double tol   = Math.max(4.0, 0.3 * expRow);
                        final boolean lOk  = Math.abs(runStart - (pred - expRow * 0.5)) <= tol;
                        final boolean rOk  = Math.abs(runLast + 1 - (pred + expRow * 0.5)) <= tol;
                        // D2: far wider than the line and uniformly line-like (no tape-width core that
                        // is more line-like than the rest, as a line beside a lighter shadow has).
                        final boolean far = d2 && span > 2.5f * expRow + 4f;
                        if (lOk && !rOk)      { c = runStart + expRow * 0.5;    wCount = Math.round(expRow);
                                                cut = far && uniformRun(base, runStart, runLast + 1, true, expRow, bright, lvlThr); }
                        else if (rOk && !lOk) { c = runLast + 1 - expRow * 0.5; wCount = Math.round(expRow);
                                                cut = far && uniformRun(base, runStart, runLast + 1, false, expRow, bright, lvlThr); }
                        else if (!(lOk && rOk) && (span > maxRow || clipped)) ok = false;
                    }
                    if (ok) {
                        final double d = Math.abs(c - (filter ? pred : trackCenter));
                        if (d < bestDist) { bestDist = d; bestC = c; bestW = wCount; bestS = runStart; bestE = runLast + 1; bestCut = cut; bestSpan = span; }
                    }
                    runStart = -1;
                }
            }

            // Accept only a run that is big enough AND close enough to the line we
            // are following; otherwise this row has no usable track → count a gap.
            if (bestW >= minRowW && bestDist <= (seeking ? Math.max(vMaxDrift, seedHalf) : vMaxDrift)) {
                if (traceAccepted == 0) {
                    traceFirstRow = row;
                    // Nearest line row: does it run off a frame edge (line leaving the view)?
                    traceEdgeSide = bestS <= 0 ? -1 : (bestE >= W ? 1 : 0);
                }
                seeking = false;
                traceAccepted++;
                farRowEff = row;
                rowCx[row]  = (float) bestC;
                // A cut-out stretch counts only if it starts abruptly (a mat / blob begins); a line
                // that widens gradually (a curve, perspective) is not one.
                boolean isCut = false;
                if (bestCut) {
                    if (!inCutStretch && prevSpan <= 1.6f * prevExp + 4f) cutStartRow = row;
                    if (inCutStretch || prevSpan <= 1.6f * prevExp + 4f) { inCutStretch = true; isCut = true; }
                } else {
                    inCutStretch = false;
                }
                prevSpan = bestSpan; prevExp = widthTest ? expRow : lastTrackWidthPx;
                if (isCut) cutRows++;
                else { cutRows = 0; lastCleanRow = row; cleanLaW = laW; cleanLaM = laM; }
                if (filter) {
                    // Local drift (columns per row upward), smoothed over rows.
                    final int dr = prevRow - row;
                    if (dr > 0) {
                        final float step = Math.max(-2f, Math.min(2f, (float) ((bestC - prevC) / dr)));
                        drift = Math.max(-2f, Math.min(2f, 0.9f * drift + 0.1f * step));
                    }
                    prevC = bestC; prevRow = row;
                }
                trackCenter = (int) (bestC + 0.5);        // follow the line upward
                gap = 0;
                farRow = row;                             // farthest row the line reaches
                final double laWeight = vRoiBot - row;    // farther ahead → weighted more
                laW += laWeight; laM += laWeight * bestC;
                fitN++; sY += row; sYY += (double) row * row;
                if (row >= nearRow) { nN++; nY += row; nYY += (double) row * row; nC += bestC; nYC += (double) row * bestC; }
                sC += bestC;       sYC += (double) row * bestC;
                final float z = rowFwd[row], x = rowScale[row] * (float) (bestC - cx0);
                addFloorPoint(z, x);
                // Subsample this row into the overlay path (current-frame data only).
                if (laCount < LA_MAX_PTS && --laNext <= 0) {
                    laPathRowFrac[laCount] = row / (float) imgH;
                    laPathColFrac[laCount] = (float) bestC / (float) imgW;
                    laCount++;
                    laNext = laStride;
                }
            } else {
                rowCx[row] = Float.NaN;                    // no line in this row
                if (mask && predictionMasked(base, pc, widthTest ? expRow : lastTrackWidthPx)) {
                    // Glare hides where the line should be: unknown, not a gap (G6).
                    if (gap == 0 && farRowEff >= 0) farRowEff = row;
                    continue;
                }
                if (++gap > (seeking ? seekGap : maxGap)) break;   // line ended → stop look-ahead
            }
        }

        traceCutRow = -1;
        if (cutRows > 0 && d2) {
            traceCutRow = cutStartRow;
            farRow = lastCleanRow; farRowEff = lastCleanRow;
            laW = lastCleanRow >= 0 ? cleanLaW : 0.0; laM = lastCleanRow >= 0 ? cleanLaM : 0.0;
        }

        // Image-space least-squares slope  b = d(col)/d(row).
        double slope = 0.0;
        if (fitN >= 2) {
            final double denom = (double) fitN * sYY - sY * sY;
            if (Math.abs(denom) > 1e-6) {
                slope = ((double) fitN * sYC - sY * sC) / denom;
            }
        }
        traceFitN    = fitN;
        traceSlope   = slope;
        double nearSlope = slope;
        if (nN >= 8) {
            final double nd = (double) nN * nYY - nY * nY;
            if (Math.abs(nd) > 1e-6) nearSlope = ((double) nN * nYC - nY * nC) / nd;
        }
        traceNearSlope = nearSlope;
        traceFarRow  = farRowEff;   // = farRow unless glare rows follow the last line row
        traceLaW     = laW;
        traceLaM     = laM;
        traceLaCount = laCount;
    }

    // =========================================================================
    //  Corridor tracker (v3, Config.corridorMode)
    // =========================================================================
    //
    // The locked line is a curve on the floor, x(u) = A + B·u + C·u² (camera heights; u =
    // forward distance beyond the H-ROI row, x = sideways, + = right), with the state
    // [A, B, C, dA, dB] in a small Kalman filter. Each frame:
    //   1. predict the curve (constant drift of A and B, process noise per frame);
    //   2. look for the line in cN slices, nearest first, each in a corridor around the
    //      predicted position (re-centred on the curve as it is updated slice by slice; beyond
    //      the fitted depth on the last two slices' direction); a slice needs a rising and a
    //      falling edge at the expected width (detectSlice) — glare-masked columns are
    //      interpolated, so a highlight on the tape centre is bridged;
    //   3. update the filter with each slice within FIT_DEPTH (gated by the innovation; a far
    //      slice that does not fit the near ones is a corner → v2). When the near part is
    //      hidden, the far slices count as weak position measurements plus heading and
    //      curvature measurements, and A comes from the prediction;
    //   4. offset / heading / tracking from the curve with the v2 formulas.
    // Anything it cannot handle returns false and the frame goes through the v2 logic, which
    // then re-initialises the filter.

    // Slice geometry (init): centre row, row range [r0, r1), forward distance u, floor scale.
    private int     cN, cFitSlices;
    private int[]   cRow, cR0, cR1;
    private float[] cU, cScale, cLaW, cGain;
    // Per-frame slice results.
    private static final byte SL_MISS = 0, SL_OBS = 1, SL_UNKNOWN = 2, SL_REJ = 3, SL_SKIP = 4;
    private byte[]  cState;
    private float[] cCol, cWid;
    private int[]   cClip;
    // Calibration samples of measured / expected slice width (for cGain).
    private float[] cGainSamples;
    private int[]   cGainCount;
    // Profile buffers for detectSlice (frame width).
    private float[] prof, profS, profPf, profG;
    private final float[] riseP = new float[32], riseG = new float[32], fallP = new float[32], fallG = new float[32];
    // detectSlice results.
    private float   dCentre, dWidth;
    private boolean dUnknown;
    private int     dClip;
    // Kalman filter (state x, covariance P row-major 5×5) and scratch.
    private final double[] kx = new double[5], kP = new double[25], kF = new double[25], kT = new double[25];
    private final double[] kPh = new double[5];
    private boolean kfReady;
    private double  kfLastNu2S;
    // Lazy line-ness rows: a row is converted at most once per frame (rowStamp == frameStamp).
    private int[]   rowStamp;
    private int     frameStamp;
    private boolean lazyRows;
    // Corridor outputs of the current frame.
    private boolean corrUsed;
    private float   corrCoverage, corrRms;
    private boolean relockedThisFrame;
    // The filter's predicted near column this frame (set when the corridor ran), for the v2
    // fallback: a base rebuilt by the v2 logic must agree with it.
    private boolean corrPredValid;
    private float   corrPredCol, corrPredSd;
    private final double[] kxPred = new double[5];

    /** Slice geometry from the floor model; called from init(). */
    private void initCorridor(Config config, int width, int height) {
        cN = Math.max(4, Math.min(32, config.corridorSlices));
        cRow = new int[cN]; cR0 = new int[cN]; cR1 = new int[cN];
        cU = new float[cN]; cScale = new float[cN]; cLaW = new float[cN]; cGain = new float[cN];
        cState = new byte[cN]; cCol = new float[cN]; cWid = new float[cN]; cClip = new int[cN];
        cGainSamples = new float[cN * Math.max(1, config.calibFrames)];
        cGainCount = new int[cN];
        prof = new float[width]; profS = new float[width]; profPf = new float[width + 1]; profG = new float[width];
        rowStamp = new int[height];
        frameStamp = 0;
        java.util.Arrays.fill(cGain, 1f);
        final int hc = Math.min(height - 1, (hRoiTop + hRoiBot) / 2);
        float uMax = 0f;
        for (int r = vRoiTop; r < vRoiBot && r < height; r++) {
            if (!Float.isNaN(rowFwd[r])) { uMax = rowFwd[r] - floorFwdH; break; }
        }
        int prevRow = hc;
        int fit = 0;
        for (int s = 0; s < cN; s++) {
            int row;
            if (s == 0) {
                row = hc;
                cR0[s] = Math.max(hRoiTop, hc - 3);
                cR1[s] = Math.min(hRoiBot, hc + 3);
            } else {
                final float want = floorFwdH + uMax * s / (cN - 1);
                row = vRoiBot - 1;
                for (int r = vRoiBot - 1; r >= vRoiTop; r--) {
                    if (Float.isNaN(rowFwd[r])) break;
                    row = r;
                    if (rowFwd[r] >= want) break;
                }
                if (row >= prevRow) row = Math.max(vRoiTop, prevRow - 1);
                // Rows averaged per slice: a quarter of the gap to the slice below, 5 … 13 rows
                // (near slices lie far apart in the image; more rows = a steadier heading).
                final int hh = Math.max(2, Math.min(6, (prevRow - row) / 4));
                cR0[s] = Math.max(vRoiTop, row - hh);
                cR1[s] = Math.min(vRoiBot, row + hh + 1);
            }
            cRow[s] = row;
            cU[s] = Float.isNaN(rowFwd[row]) ? uMax : rowFwd[row] - floorFwdH;
            cScale[s] = Float.isNaN(rowScale[row]) ? floorScaleH : rowScale[row];
            if (cU[s] <= FIT_DEPTH * floorFwdH) fit = s + 1;
            prevRow = row;
        }
        cFitSlices = Math.max(2, fit);
        // Look-ahead weight of the V-ROI rows each slice stands for (rows halfway to its
        // neighbours), the same far-weighting as the v2 look-ahead value.
        for (int s = 1; s < cN; s++) {
            final int lo = s + 1 < cN ? (cRow[s] + cRow[s + 1] + 1) / 2 : vRoiTop;
            final int hi = (cRow[s] + cRow[s - 1] + 1) / 2;
            double w = 0;
            for (int r = Math.max(vRoiTop, lo); r < Math.min(vRoiBot, hi); r++) w += vRoiBot - r;
            cLaW[s] = (float) Math.max(1.0, w);
        }
        kfReady = false;
    }

    // ── Lazy line-ness image ────────────────────────────────────────────────

    /**
     * Corridor frames read only the strip and the slice rows, so the line-ness image is
     * converted per row on demand. Returns the work image (rows not converted yet are filled by
     * {@link #ensureRows}); same polarity rules as {@link #prepareWorkImage}.
     */
    private byte[] prepareLazy() {
        lazyRows = true;
        frameStamp++;
        final boolean project = cfg.autoColor && colorReady;
        if (!project) {
            workBright = cfg.trackIsBright;
            if (srcFormat != FMT_ARGB) { lazyRows = false; return srcY; }   // luma needs no conversion
        } else {
            workBright = true;
        }
        ensureRows(hRoiTop, hRoiBot);
        return work;
    }

    /** Convert rows [r0, r1) of the work image if not done yet this frame (lazy mode only). */
    private void ensureRows(int r0, int r1) {
        if (!lazyRows) return;
        final int[] st = rowStamp;
        final int stamp = frameStamp;
        int a = r0;
        while (a < r1) {
            if (st[a] == stamp) { a++; continue; }
            int b = a;
            while (b < r1 && st[b] != stamp) { st[b] = stamp; b++; }
            convertRows(a, b);
            a = b;
        }
    }

    /** prepareWorkImage's conversion for rows [r0, r1) into {@link #work}. */
    private void convertRows(int r0, int r1) {
        final int W = imgW;
        final byte[] out = work;
        final boolean project = cfg.autoColor && colorReady;
        if (!project) {                                    // ARGB luma (gray luma is the Y plane)
            final int[] src = srcArgb;
            for (int i = r0 * W, end = r1 * W; i < end; i++) {
                final int c = src[i];
                out[i] = (byte) ((77 * ((c >> 16) & 0xFF) + 150 * ((c >> 8) & 0xFF) + 29 * (c & 0xFF)) >> 8);
            }
        } else if (srcFormat == FMT_GRAY) {
            final byte[] ys = srcY;
            final int ky = kY, off = kGrayOff;
            for (int i = r0 * W, end = r1 * W; i < end; i++) out[i] = clampByte((ky * (ys[i] & 0xFF) + off) >> 10);
        } else if (srcFormat == FMT_YUV420) {
            final byte[] ys = srcY, us = srcU, vs = srcV;
            final int ky = kY, ku = kU, kv = kV, off = k0, cw = uvW;
            for (int row = r0; row < r1; row++) {
                final int base = row * W, uvBase = (row >> 1) * cw;
                for (int col = 0; col < W; col++) {
                    final int uvi = uvBase + (col >> 1);
                    out[base + col] = clampByte((ky * (ys[base + col] & 0xFF)
                            + ku * (us[uvi] & 0xFF) + kv * (vs[uvi] & 0xFF) + off) >> 10);
                }
            }
        } else {
            final int[] src = srcArgb;
            final int kr = kR, kg = kG, kb = kB, off = kC;
            for (int i = r0 * W, end = r1 * W; i < end; i++) {
                final int c = src[i];
                out[i] = clampByte((kr * ((c >> 16) & 0xFF) + kg * ((c >> 8) & 0xFF) + kb * (c & 0xFF) + off) >> 10);
            }
        }
    }

    // ── Slice detector: edge pair at the expected width ─────────────────────

    /**
     * Look for the line in rows [r0, r1) (every rStep-th) within predCol ± half: the rows are
     * averaged into a line-ness profile (line = high), glare-masked columns are filled by
     * interpolation from their neighbours (a saturated tape centre is bridged), the profile is
     * smoothed and its rising and falling edges found. The line is the rise / fall PAIR whose
     * distance matches expPx (0.6 … maxWidMult ×) with a line-like inside, scored by edge
     * strength, width match and distance from predCol. Inner edges of a highlight on the tape
     * centre give pairs far narrower than the tape and are not chosen. A line cut by the frame
     * edge is accepted with one edge (dClip = −1 / +1). Writes dCentre / dWidth; sets dUnknown
     * when most of the predicted span is glare.
     */
    private boolean detectSlice(byte[] pix, boolean bright, int r0, int r1, int rStep,
                                float predCol, float expPx, float half, float maxWidMult) {
        dUnknown = false;
        dClip = 0;
        final int W = imgW;
        final int lo = Math.max(0, (int) Math.floor(predCol - half));
        final int hi = Math.min(W, (int) Math.ceil(predCol + half) + 1);
        if (hi - lo < 6 || r1 <= r0) return false;
        final float[] p = prof, ps = profS, pf = profPf, g = profG;
        final boolean mask = maskActive;
        final byte[] gs = glareSrc;
        final int gl = glareLim;
        int known = 0;
        for (int c = lo; c < hi; c++) {
            int s = 0, n = 0;
            for (int r = r0; r < r1; r += rStep) {
                final int i = r * W + c;
                if (mask && (gs[i] & 0xFF) >= gl) continue;
                final int v = pix[i] & 0xFF;
                s += bright ? v : 255 - v;
                n++;
            }
            if (n > 0) { p[c] = (float) s / n; known++; } else p[c] = Float.NaN;
        }
        // Unknown when the predicted line span (or most of the corridor) is glare.
        {
            final int a = Math.max(lo, Math.round(predCol - 0.5f * expPx)), b = Math.min(hi, Math.round(predCol + 0.5f * expPx) + 1);
            int k = 0;
            for (int c = a; c < b; c++) if (!Float.isNaN(p[c])) k++;
            if ((b > a && 2 * k < b - a) || 3 * known < hi - lo) { dUnknown = true; return false; }
        }
        // Fill glare columns by interpolation.
        int last = -1;
        for (int c = lo; c < hi; c++) {
            if (Float.isNaN(p[c])) continue;
            if (last < 0) { for (int q = lo; q < c; q++) p[q] = p[c]; }
            else if (c - last > 1) {
                final float a = p[last], d = (p[c] - a) / (c - last);
                for (int q = last + 1; q < c; q++) p[q] = a + d * (q - last);
            }
            last = c;
        }
        for (int q = last + 1; q < hi; q++) p[q] = p[last];
        // Box smoothing (radius rs) via prefix sums.
        final int rs = Math.max(1, Math.round(expPx / 8f));
        pf[lo] = 0f;
        for (int c = lo; c < hi; c++) pf[c + 1] = pf[c] + p[c];
        float pmin = Float.MAX_VALUE, pmax = -Float.MAX_VALUE;
        for (int c = lo; c < hi; c++) {
            final int a = Math.max(lo, c - rs), b = Math.min(hi, c + rs + 1);
            final float v = (pf[b] - pf[a]) / (b - a);
            ps[c] = v;
            if (v < pmin) pmin = v;
            if (v > pmax) pmax = v;
        }
        final float contrast = pmax - pmin;
        if (contrast < Math.max(8f, 0.75f * cfg.minContrast)) return false;
        // Edges: central difference of the smoothed profile; a clean step of contrast K gives
        // a peak of about 2K / (2·rs + 1). Rises and falls at ≥ half of that for this contrast.
        final float thrG = contrast / (2 * rs + 1);
        int nr = 0, nf = 0;
        g[lo] = 0f; g[hi - 1] = 0f;
        for (int c = lo + 1; c < hi - 1; c++) g[c] = ps[c + 1] - ps[c - 1];
        for (int c = lo + 2; c < hi - 2; c++) {
            final float v = g[c];
            if (v >= thrG && v >= g[c - 1] && v > g[c + 1] && nr < riseP.length) {
                riseP[nr] = c + subPeak(g[c - 1], v, g[c + 1]); riseG[nr++] = v;
            } else if (v <= -thrG && v <= g[c - 1] && v < g[c + 1] && nf < fallP.length) {
                fallP[nf] = c + subPeak(-g[c - 1], -v, -g[c + 1]); fallG[nf++] = -v;
            }
        }
        final float sig = Math.max(expPx, 0.5f * half);
        final float minW = 0.6f * expPx, maxW = maxWidMult * expPx;
        final float inner = pmin + 0.35f * contrast;
        final float norm = 2f * contrast / (2 * rs + 1);
        float best = 0f;
        for (int i = 0; i < nr; i++) {
            for (int j = 0; j < nf; j++) {
                final float wid = fallP[j] - riseP[i];
                if (wid < minW || wid > maxW) continue;
                if (meanRaw(riseP[i], fallP[j], lo, hi) < inner) continue;
                final float c = 0.5f * (riseP[i] + fallP[j]);
                final float dz = (c - predCol) / sig;
                final float sc = (riseG[i] + fallG[j]) / (2f * norm)
                        * (float) Math.sqrt(Math.min(wid, expPx) / Math.max(wid, expPx))
                        * (float) Math.exp(-0.5 * dz * dz);
                if (sc > best) { best = sc; dCentre = c; dWidth = wid; dClip = 0; }
            }
        }
        // Line cut by the frame edge: one edge, line-like up to the edge.
        if (lo == 0 && ps[0] >= pmin + 0.6f * contrast) {
            for (int j = 0; j < nf; j++) {
                final float wid = fallP[j] + 0.5f;
                if (wid < 0.25f * expPx || wid > maxW) continue;
                final float c = 0.5f * (fallP[j] - 0.5f);
                final float dz = (c - predCol) / sig;
                final float sc = 0.8f * fallG[j] / norm * (float) Math.exp(-0.5 * dz * dz);
                if (sc > best) { best = sc; dCentre = c; dWidth = wid; dClip = -1; }
            }
        }
        if (hi == W && ps[W - 1] >= pmin + 0.6f * contrast) {
            for (int i = 0; i < nr; i++) {
                final float wid = W - 0.5f - riseP[i];
                if (wid < 0.25f * expPx || wid > maxW) continue;
                final float c = 0.5f * (riseP[i] + W - 0.5f);
                final float dz = (c - predCol) / sig;
                final float sc = 0.8f * riseG[i] / norm * (float) Math.exp(-0.5 * dz * dz);
                if (sc > best) { best = sc; dCentre = c; dWidth = wid; dClip = 1; }
            }
        }
        return best > 0.25f;
    }

    private static float subPeak(float a, float b, float c) {
        final float d = a - 2f * b + c;
        return Math.abs(d) < 1e-6f ? 0f : Math.max(-0.5f, Math.min(0.5f, 0.5f * (a - c) / d));
    }

    /** Mean of the raw (filled) profile between two sub-pixel positions. */
    private float meanRaw(float a, float b, int lo, int hi) {
        final int i0 = Math.max(lo, (int) Math.ceil(a)), i1 = Math.min(hi, (int) Math.floor(b) + 1);
        if (i1 <= i0) return Float.MAX_VALUE;
        float s = 0f;
        for (int c = i0; c < i1; c++) s += prof[c];
        return s / (i1 - i0);
    }

    // ── Kalman filter on [A, B, C, dA, dB] ──────────────────────────────────

    private void kfInit(double a, double b, double c, double wf) {
        java.util.Arrays.fill(kP, 0.0);
        kx[0] = a; kx[1] = b; kx[2] = c; kx[3] = 0; kx[4] = 0;
        kP[0] = sq(0.3 * wf); kP[6] = sq(0.15); kP[12] = sq(0.4); kP[18] = sq(0.1 * wf); kP[24] = sq(0.03);
        kfReady = true;
    }

    /** x ← F x, P ← F P Fᵀ + Q; dt in frames (30 fps = 1). */
    private void kfPredict(double dt, double wf) {
        final double damp = 0.9;
        kx[0] += kx[3] * dt; kx[1] += kx[4] * dt; kx[3] *= damp; kx[4] *= damp;
        final double[] F = kF, P = kP, T = kT;
        java.util.Arrays.fill(F, 0.0);
        F[0] = 1; F[3] = dt; F[6] = 1; F[9] = dt; F[12] = 1; F[18] = damp; F[24] = damp;
        for (int i = 0; i < 5; i++) for (int j = 0; j < 5; j++) {
            double s = 0; for (int k = 0; k < 5; k++) s += F[i * 5 + k] * P[k * 5 + j]; T[i * 5 + j] = s;
        }
        for (int i = 0; i < 5; i++) for (int j = 0; j < 5; j++) {
            double s = 0; for (int k = 0; k < 5; k++) s += T[i * 5 + k] * F[j * 5 + k]; P[i * 5 + j] = s;
        }
        final double q = cfg.corridorProcessNoise * dt;
        // Steady sideways motion is carried by dA; A itself changes little between frames.
        // On a straight the heading and curvature change slowly relative to the noise of a few
        // slices (small noise: no heading wander, steady steering); on a curve (|C| >= 0.3, a
        // radius of ~1.7 camera heights or less) they change quickly while the robot enters it.
        final double curvy = Math.min(1.0, Math.abs(kx[2]) / 0.3);
        P[0]  += q * sq(0.1 * wf);
        P[6]  += q * sq(0.02 + 0.04 * curvy);
        P[12] += q * sq(0.08 + 0.07 * curvy);
        P[18] += q * sq(0.05 * wf);
        P[24] += q * sq(0.005 + 0.025 * curvy);
    }

    /** Variance of h·x for h = (h0, h1, h2, 0, 0). */
    private double kfVar(double h0, double h1, double h2) {
        final double[] P = kP;
        return h0 * (P[0] * h0 + P[1] * h1 + P[2] * h2)
             + h1 * (P[5] * h0 + P[6] * h1 + P[7] * h2)
             + h2 * (P[10] * h0 + P[11] * h1 + P[12] * h2);
    }

    /**
     * Scalar update with z = h·x + noise(R), h = (h0, h1, h2, 0, 0). Rejected when the innovation
     * is beyond gate² (in units of its variance); between 2² and gate² the noise is inflated
     * (Huber). Returns true if used; kfLastNu2S = normalised innovation².
     */
    private boolean kfUpdate(double h0, double h1, double h2, double z, double R, double gate2) {
        final double[] P = kP, Ph = kPh, x = kx;
        for (int i = 0; i < 5; i++) Ph[i] = P[i * 5] * h0 + P[i * 5 + 1] * h1 + P[i * 5 + 2] * h2;
        final double nu = z - (h0 * x[0] + h1 * x[1] + h2 * x[2]);
        double S = h0 * Ph[0] + h1 * Ph[1] + h2 * Ph[2] + R;
        final double n2 = nu * nu / S;
        kfLastNu2S = n2;
        if (n2 > gate2) return false;
        if (n2 > 4.0) S = h0 * Ph[0] + h1 * Ph[1] + h2 * Ph[2] + R * n2 / 4.0;
        for (int i = 0; i < 5; i++) x[i] += Ph[i] / S * nu;
        for (int i = 0; i < 5; i++) for (int j = 0; j < 5; j++) P[i * 5 + j] -= Ph[i] * Ph[j] / S;
        return true;
    }

    // ── One corridor frame ──────────────────────────────────────────────────

    /**
     * Track the locked line with the corridor. Returns false (nothing written) when the frame
     * must use the v2 logic: no line in any slice, a corner, too little evidence, or a relock.
     */
    private boolean corridorFrame(byte[] pixels, boolean bright, int threshold, boolean stripFound, int stripCx) {
        final int W = imgW;
        final float halfW = W * 0.5f, cx0 = W * 0.5f - 0.5f;
        final float expW = expectedLineWidth();
        if (expW <= 0f || floorScaleH <= 0f || relockedThisFrame) return false;
        final double wf = expW * floorScaleH;
        final double dt = frameDtMs > 0f ? Math.max(0.3, Math.min(3.0, frameDtMs / 33.333)) : 1.0;
        kfPredict(dt, wf);
        corrPredCol = (float) (W * 0.5f - 0.5f + kx[0] / floorScaleH);
        corrPredSd  = (float) (Math.sqrt(Math.max(0.0, kP[0])) / floorScaleH);
        corrPredValid = true;
        System.arraycopy(kx, 0, kxPred, 0, 5);
        final int runGap = runGapTol();
        final float nearU = 0.35f * FIT_DEPTH * floorFwdH;
        final float slantPrev = Math.max(-3f, Math.min(3f, lastImgSlope));

        int obs = 0, obsFit = 0, rejFit = 0, rejSign = 0, misses = 0, firstObs = -1, lastObs = -1, prevObs = -1;
        boolean chainEnded = false;
        int nearObs = 0, expectFit = 0;
        boolean farOnly = false;
        double rss = 0;
        int rssN = 0;
        for (int s = 0; s < cN; s++) {
            cState[s] = SL_SKIP;
            if (s < cFitSlices) expectFit++;
            final double u = cU[s];
            // Predicted floor position: the filter (updated by the slices below) within the
            // fitted depth; farther out, the direction of the last two line slices.
            double xp = kx[0] + kx[1] * u + kx[2] * u * u;
            double sd = Math.sqrt(Math.max(0.0, kfVar(1, u, u * u)));
            if (s >= cFitSlices && prevObs >= 0 && lastObs > prevObs) {
                final double x1 = (cCol[prevObs] - cx0) * cScale[prevObs], x2 = (cCol[lastObs] - cx0) * cScale[lastObs];
                final double du = cU[lastObs] - cU[prevObs];
                if (du > 1e-6) { xp = x2 + (x2 - x1) / du * (u - cU[lastObs]); sd = 0.5 * wf; }
            }
            final float colPred = (float) (cx0 + xp / cScale[s]);
            // Expected width here (perspective, calibrated gain) and slant in the image.
            final float expPx = expW * floorScaleH / cScale[s] * cGain[s];
            float sl = slantPrev;
            if (s > 0 && lastObs >= 0 && cRow[lastObs] != cRow[s]) sl = (colPred - cCol[lastObs]) / (cRow[lastObs] - cRow[s]);
            sl = Math.max(-3f, Math.min(3f, sl));
            final float slK = (float) Math.sqrt(1f + sl * sl);
            final float expS = expPx * slK + (cR1[s] - cR0[s] - 1) * Math.abs(sl);
            float half = Math.max(cfg.corridorWidthMult * 0.5f * expS, 0.5f * expS + 2 * runGap)
                       + (float) (3.0 * sd / cScale[s]);
            half = Math.min(half, halfW);
            if (colPred + half < 0 || colPred - half > W) { cState[s] = SL_MISS; if (obs > 0 && ++misses >= 2) { chainEnded = true; break; } continue; }
            ensureRows(cR0[s], cR1[s]);
            // Slice 0: a clean band found by the strip search (whole line width, not cut) is the
            // better measurement — its centre averages all strip rows; otherwise the edge pair.
            boolean found;
            final float bandW = bandEnd - bandStart, bandC = stripFound ? (float) (bandMoment / bandSum) : 0f;
            if (s == 0 && stripFound && !bandTrimmed && Math.abs(bandW - expS) <= 0.3f * expPx
                    && Math.abs(bandC - colPred) <= half) {
                dCentre = bandC; dWidth = bandW; dClip = bandStart <= 0 ? -1 : (bandEnd >= W ? 1 : 0); dUnknown = false;
                found = true;
            } else {
                found = detectSlice(pixels, bright, cR0[s], cR1[s], 1, colPred, expS, half, 1.5f);
            }
            if (!found && s == 0 && !dUnknown && stripFound && Math.abs(stripCx - colPred) <= half) {
                dCentre = stripCx; dWidth = lastTrackWidthPx; dClip = 0; found = true;   // v2 band as slice 0
            }
            if (dUnknown) { cState[s] = SL_UNKNOWN; continue; }
            if (!found) { cState[s] = SL_MISS; if (obs > 0 && ++misses >= 2) { chainEnded = true; break; } continue; }
            misses = 0;
            cCol[s] = dCentre; cWid[s] = dWidth / slK; cClip[s] = dClip;
            if (firstObs < 0) { firstObs = s; farOnly = u > nearU; }
            if (s < cFitSlices) {
                final double xm = (dCentre - cx0) * cScale[s];
                final double sp = Math.max(s == 0 ? 0.5 : 0.7, (s == 0 ? 0.04 : 0.08) * expPx) * cScale[s];
                double R = sp * sp;
                if (farOnly) {
                    // Near part hidden: the position of a far slice says little about where the
                    // line meets the strip (a curve) — weak, growing with the distance.
                    final double weak = Math.max(5.0 * sp, 0.5 * wf * u / Math.max(1e-3, floorFwdH));
                    R = weak * weak;
                }
                final boolean ok = kfUpdate(1, u, u * u, xm, R, 16.0);
                // The near line where the filter did not expect it (a jump, a new lock): the
                // v2 logic decides (gate, line left), then the filter restarts from it.
                if (!farOnly && u <= nearU && kfLastNu2S > 4.0) return false;
                if (ok) {
                    final double res = xm - (kx[0] + kx[1] * u + kx[2] * u * u);
                    rss += res * res; rssN++;
                    obsFit++;
                    if (u <= nearU) nearObs++;
                } else {
                    cState[s] = SL_REJ;
                    rejFit++;
                    rejSign += xm > xp ? 1 : -1;
                    continue;
                }
            }
            cState[s] = SL_OBS;
            obs++;
            prevObs = lastObs;
            lastObs = s;
        }

        // ── Decide whether this frame is the corridor's ──
        if (obs == 0) return false;
        // Coming back after the line was hidden / lost with the near part in view: the v2 logic
        // re-acquires it (continuity-ranked); the filter's wide windows could join two things.
        if (!farOnly && hardening && (coastFrames > 0 || lostFrames > 0)
                && maskedPixels < 0.2f * (hRoiBot - hRoiTop) * imgW) return false;   // (strip not under glare)
        // Corner / junction: the near part fits, farther slices do not (all on one side).
        if (rejFit >= 2 && Math.abs(rejSign) == rejFit && obsFit >= 2) return false;
        if (farOnly) {
            if (obsFit < 1) return false;           // one far slice + the prediction is enough
            // Heading and curvature of the far part, measured where it is seen.
            corridorFarMeasurements(cx0, wf);
            if (Math.sqrt(Math.max(0.0, kP[0])) > 1.0 * wf) return false;    // near position too uncertain
        } else {
            // The line ends inside the corridor (corner, junction, dead end, line leaving the
            // frame) or too few slices: the v2 logic handles it.
            if (chainEnded || obs < Math.max(2, cfg.minChainSlices)) return false;
        }
        final int unknownFit = countState(SL_UNKNOWN, cFitSlices);
        final float coverage = Math.min(1f, obsFit / (float) Math.max(1, expectFit - unknownFit));
        final double rms = rssN > 0 ? Math.sqrt(rss / rssN) : 0.0;
        final float fitQ = (float) Math.max(0.0, Math.min(1.0, 1.5 - 2.0 * rms / Math.max(1e-6, 0.5 * wf)));
        if (!farOnly && coverage * fitQ < cfg.corridorMinConfidence) return false;

        // ── Outputs from the curve (v2 formulas) ──
        final float colA = (float) (cx0 + kx[0] / floorScaleH);
        final float edgeZone = Math.max(lastTrackWidthPx, 0.12f * W);
        boolean pin = false;
        int pinSide = 0;
        if (farOnly) {
            // Edge exits beat the prediction: the near position in the edge zone, or the nearest
            // line slice cut by the frame edge.
            if (colA < edgeZone) { pin = true; pinSide = -1; }
            else if (colA > W - edgeZone) { pin = true; pinSide = 1; }
            else if (cClip[firstObs] != 0) { pin = true; pinSide = cClip[firstObs]; }
        }
        if (pin) {
            final float pinSteer = glarePinSteering(pinSide);
            kfReady = pinCapActive;                       // a capped pin keeps the filter predicting
            glarePinned = true;
            glarePinSide = pinSide;
            final boolean onRight = pinSide > 0;
            final int cxp = onRight ? Math.min(W - 1, Math.round(steerRefX + halfW)) : Math.max(0, Math.round(steerRefX - halfW));
            lastCentroidX = cxp; centroidVel = 0f; lostFrames++;
            coastMs = cfg.glareCoastMs; coastFrames = cfg.glareCoastMaxFrames;
            if (lostFrames > cfg.trackLostGraceFrames) trackAcquired = false;
            publishCorridorPath(false, cx0);
            writeOutput(pinSteer, 100f, -1, false, 0f, 0f, 0f, 0f, 0f, false, false, false, coverage, (float) (rms / wf));
            return true;
        }
        final int centroidX = Math.max(0, Math.min(W - 1, Math.round(colA)));
        // Lock state, as when the strip finds the line.
        float d = centroidX - lastCentroidX;
        final float lim = 0.5f * Math.max(20f, lastTrackWidthPx);
        d = Math.max(-lim, Math.min(lim, d));
        centroidVel = 0.5f * centroidVel + 0.5f * d;
        lastCentroidX = centroidX;
        if (cState[0] == SL_OBS && cClip[0] == 0) lastTrackWidthPx = Math.max(1f, cWid[0]);
        trackAcquired = true; lostFrames = 0; holdFrames = 0; coastMs = 0f; coastFrames = 0;
        resetPinCap();
        noteCurve(wf, farOnly || coverage * fitQ >= cfg.corridorMinConfidence);
        // Near image slope (columns per row upward) for the next frame's width test.
        if (lastObs > firstObs) {
            final int a = firstObs, b = nextObs(firstObs);
            if (b > a && cRow[a] != cRow[b]) lastImgSlope = (cCol[b] - cCol[a]) / (cRow[a] - cRow[b]);
        }

        final float offsetTerm = (centroidX - steerRefX) / halfW * 100f;
        float headingTerm = 0f, headingDeg = 0f, conf = 0f;
        if (cfg.steerHeadingWeight != 0f) {
            final float rowsK = Math.min(1f, (obsFit - 1) / 3f);
            float uMaxObs = 0f;
            for (int s = 0; s < cFitSlices; s++) if (cState[s] == SL_OBS) uMaxObs = cU[s];
            final float span = Math.min(1f, uMaxObs / Math.max(1e-3f, 0.5f * floorFwdH));
            conf = Math.max(0f, rowsK) * span * fitQ;
            if (conf > 0f) {
                headingDeg = (float) Math.toDegrees(Math.atan(kx[1])) - headingRefDeg;
                final double psi = Math.toRadians(Math.max(-75f, Math.min(75f, headingDeg)));
                headingTerm = (float) (100.0 * cfg.steerLookAhead * floorFwdH * Math.tan(psi) / Math.max(1e-6, floorScaleH * halfW));
            }
        }
        final float steering = Math.max(-100f, Math.min(100f,
                cfg.steerOffsetWeight * offsetTerm + cfg.steerHeadingWeight * conf * headingTerm));

        // Tracking: far-weighted look-ahead position + visible road length (v2), the far row
        // refined between the last line slice and the next one.
        double laW = 0, laM = 0;
        for (int s = 1; s < cN; s++) if (cState[s] == SL_OBS) { laW += cLaW[s]; laM += cLaW[s] * cCol[s]; }
        final int farRow = refineFarRow(pixels, bright, lastObs, cx0, expW);
        float intersection = farIntersection(laW > 0 ? (float) (laM / laW) : Float.NaN, farRow, Float.NaN);
        if (farOnly) {
            // Unsure of the near part: never faster than before, and slower for a sharp curve.
            intersection = Math.max(Math.max(intersection, lastIntersection),
                                    farIntersection(Float.NaN, -1, (float) kx[2]));
        }
        publishCorridorPath(true, cx0);
        lastSteering = steering;
        lastIntersection = intersection;
        lastHeadTerm = cfg.steerHeadingWeight * conf * headingTerm;
        // Image tilt from the line slices (overlay only).
        final float tilt = imageTilt();
        writeOutput(steering, intersection, centroidX, true,
                cfg.steerOffsetWeight * offsetTerm, cfg.steerHeadingWeight * conf * headingTerm,
                headingDeg, conf, tilt, false, false, farOnly, coverage, (float) (rms / wf));
        return true;
    }

    private int countState(byte st, int n) {
        int k = 0;
        for (int s = 0; s < n && s < cN; s++) if (cState[s] == st) k++;
        return k;
    }

    private int nextObs(int s) {
        for (int t = s + 1; t < cN; t++) if (cState[t] == SL_OBS) return t;
        return -1;
    }

    /**
     * Near part hidden: the observed far slices give the line's direction (slope at their mean
     * distance) and, with enough of them, its curvature — measurements of B + 2Cu and C, which
     * do not depend on A.
     */
    private void corridorFarMeasurements(float cx0, double wf) {
        int n = 0;
        double su = 0, sx = 0, suu = 0, sux = 0, suuu = 0, suuuu = 0, suux = 0;
        for (int s = 0; s < cFitSlices; s++) {
            if (cState[s] != SL_OBS) continue;
            final double u = cU[s], x = (cCol[s] - cx0) * cScale[s];
            n++; su += u; sx += x; suu += u * u; sux += u * x;
            suuu += u * u * u; suuuu += u * u * u * u; suux += u * u * x;
        }
        if (n < 2) return;
        final double um = su / n;
        final double d = n * suu - su * su;
        if (Math.abs(d) < 1e-9) return;
        final double slope = (n * sux - su * sx) / d;
        final double spanU = Math.sqrt(Math.max(1e-9, suu / n - um * um));
        final double sdSlope = Math.max(0.05, 0.3 * wf / Math.max(1e-3, spanU * Math.sqrt(n)));
        kfUpdate(0, 1, 2 * um, slope, sdSlope * sdSlope, 25.0);
        if (n >= 4 && spanU > 0.15 * floorFwdH) {
            final double a12 = su, a13 = suu, a22 = suu, a23 = suuu, a33 = suuuu;
            final double det = n * (a22 * a33 - a23 * a23) - a12 * (a12 * a33 - a23 * a13) + a13 * (a12 * a23 - a22 * a13);
            if (Math.abs(det) > 1e-12) {
                final double detC = n * (a22 * suux - sux * a23) - a12 * (a12 * suux - sux * a13) + sx * (a12 * a23 - a22 * a13);
                final double c = detC / det;
                final double sdC = Math.max(0.1, 0.6 * wf / Math.max(1e-3, spanU * spanU * Math.sqrt(n)));
                kfUpdate(0, 0, 1, c, sdC * sdC, 25.0);
            }
        }
    }

    /**
     * Farthest image row on the line: between the last line slice and the next slice, rows are
     * checked (every 2nd) on the line's continued direction until it is missed twice.
     */
    private int refineFarRow(byte[] pixels, boolean bright, int lastObs, float cx0, float expW) {
        if (lastObs < 0) return -1;
        int far = cRow[lastObs];
        if (lastObs == 0 || lastObs >= cN - 1) return lastObs >= cN - 1 ? Math.min(far, vRoiTop) : far;
        final int prev = lastObs > 0 ? lastObsBefore(lastObs) : -1;
        final float colL = cCol[lastObs];
        final float slope = prev >= 0 && cRow[prev] != cRow[lastObs]
                ? (cCol[lastObs] - cCol[prev]) / (cRow[prev] - cRow[lastObs]) : 0f;
        final int stop = cRow[lastObs + 1];
        int miss = 0;
        for (int r = cRow[lastObs] - 2; r > stop && r >= vRoiTop; r -= 2) {
            final float pc = colL + slope * (cRow[lastObs] - r);
            final float e = expW * floorScaleH / rowScale[r];
            ensureRows(r, r + 1);
            if (detectSlice(pixels, bright, r, r + 1, 1, pc, e * (float) Math.sqrt(1 + slope * slope),
                            Math.max(cfg.corridorWidthMult * 0.5f * e, 0.5f * e + 2 * runGapTol()), 1.5f)) {
                far = r; miss = 0;
            } else if (!dUnknown && ++miss >= 2) break;
        }
        return far;
    }

    private int lastObsBefore(int s) {
        for (int t = s - 1; t >= 0; t--) if (cState[t] == SL_OBS) return t;
        return -1;
    }

    /**
     * Internal intersection (0 = straight … 100 = hard turn / end) from the far-weighted
     * look-ahead column (NaN = none), the farthest line row (-1 = none) and, when given, the
     * curve's C (sharpness: 100 at a radius of about one H-ROI distance).
     */
    private float farIntersection(float lookAheadX, int farRow, float curveC) {
        float it = 0f;
        if (!Float.isNaN(lookAheadX)) {
            final float span = Math.max(1f, imgW * 0.5f * cfg.trackOffsetFullFrac);
            it = Math.min(100f, Math.abs(lookAheadX - trackRefX) / span * 100f);
        }
        if (cfg.trackLengthSlowdown && farRow >= 0) {
            final float full = Math.max(1e-3f, cfg.trackLengthFullFrac);
            final float length = Math.min(1f, visibleRoadFrac(farRow) / full);
            it = Math.max(it, 100f * (1f - length));
        }
        if (!Float.isNaN(curveC)) {
            final float kappa = 2f * Math.abs(curveC);                    // 1 / radius, camera heights
            it = Math.max(it, Math.min(100f, 100f * kappa * floorFwdH));
        }
        return Math.max(0f, Math.min(100f, it));
    }

    /** Overlay path: the strip point and every line slice (nearest first). */
    private void publishCorridorPath(boolean withBase, float cx0) {
        int n = 0;
        if (withBase) {
            laPathRowFrac[n] = (hRoiTop + hRoiBot) * 0.5f / imgH;
            laPathColFrac[n] = (float) (cx0 + kx[0] / floorScaleH) / imgW;
            n++;
        }
        for (int s = 1; s < cN && n < LA_MAX_PTS; s++) {
            if (cState[s] != SL_OBS) continue;
            laPathRowFrac[n] = cRow[s] / (float) imgH;
            laPathColFrac[n] = cCol[s] / imgW;
            n++;
        }
        laPathCount = n;
    }

    /** Lean of the line slices in the image, degrees from vertical (+ = leans right). */
    private float imageTilt() {
        int n = 0;
        double sy = 0, syy = 0, sc = 0, syc = 0;
        for (int s = 0; s < cN; s++) {
            if (cState[s] != SL_OBS) continue;
            final double y = cRow[s], c = cCol[s];
            n++; sy += y; syy += y * y; sc += c; syc += y * c;
        }
        if (n < 2) return 0f;
        final double den = n * syy - sy * sy;
        if (Math.abs(den) < 1e-6) return 0f;
        return (float) Math.toDegrees(Math.atan(-(n * syc - sy * sc) / den));
    }

    /** After a v2 frame: (re)start the filter from the v2 result, or drop it. */
    private void corridorSync(boolean trackFound, int centroidX, boolean coasting, float conf) {
        final float expW = expectedLineWidth();
        if (!trackFound || expW <= 0f || floorScaleH <= 0f) {
            if (!coasting) kfReady = false;
            return;
        }
        final double wf = expW * floorScaleH;
        final double a = (centroidX - (imgW * 0.5f - 0.5f)) * floorScaleH;
        final double b = conf > 0f ? Math.tan(Math.toRadians(fitHeadingDeg)) : 0.0;
        kfInit(a, b, conf > 0f ? fitC : 0.0, wf);
    }

    /** Write the internal output holder (corridor frames). */
    private void writeOutput(float steering, float intersection, int centroidX, boolean trackFound,
                             float offset, float heading, float headingDeg, float conf, float tilt,
                             boolean held, boolean coasting, boolean reconstructed,
                             float coverage, float rmsLines) {
        output.steering      = steering;
        output.intersection  = intersection;
        output.centroidX     = trackFound ? centroidX : -1;
        output.tiltDeg       = trackFound ? tilt : 0f;
        output.trackWidthPx  = lastTrackWidthPx;
        output.trackFound    = trackFound;
        output.offsetTerm    = trackFound ? offset : 0f;
        output.headingTerm   = trackFound ? heading : 0f;
        output.headingDeg    = headingDeg;
        output.headingConf   = conf;
        output.held          = held || coasting;
        output.rejected      = rejectedBands;
        output.coasting      = coasting;
        output.reconstructed = reconstructed;
        corrUsed     = true;
        corrCoverage = coverage;
        corrRms      = rmsLines;
    }

    /**
     * Calibration (phase 2, look-ahead traced): measure the line width in every slice on the
     * v2 trace and keep measured / expected; the median per slice becomes cGain, which
     * corrects the perspective width for a camera tilt that is not exactly cameraTiltDeg.
     */
    private void calibrateCorridorWidths(byte[] pix, boolean bright, int sample) {
        final float expW = expectedLineWidth() > 0f ? expectedLineWidth() : lastTrackWidthPx;
        if (expW <= 0f || sample >= Math.max(1, cfg.calibFrames)) return;
        for (int s = 1; s < cN; s++) {
            final float c = rowCx[cRow[s]];
            if (Float.isNaN(c)) continue;
            final float e = expW * floorScaleH / cScale[s];
            if (cGainCount[s] < Math.max(1, cfg.calibFrames)
                    && detectSlice(pix, bright, cR0[s], cR1[s], 1, c, e, Math.max(1.5f * e, 0.5f * e + 3 * runGapTol()), 2.0f)) {
                cGainSamples[s * Math.max(1, cfg.calibFrames) + cGainCount[s]++] = dWidth / e;
            }
        }
    }

    private void finishCorridorWidths(int frames) {
        final int stride = Math.max(1, cfg.calibFrames);
        for (int s = 0; s < cN; s++) {
            cGain[s] = 1f;
            final int n = cGainCount[s];
            if (n * 2 >= frames && n > 0) {
                java.util.Arrays.sort(cGainSamples, s * stride, s * stride + n);
                cGain[s] = Math.max(0.6f, Math.min(1.6f, cGainSamples[s * stride + n / 2]));
            }
            cGainCount[s] = 0;
        }
    }

    /**
     * Add one point of the line on the floor (forward z, lateral x; camera heights).
     * Points more than FIT_DEPTH H-ROI distances beyond the H-ROI are not fitted.
     */
    private void addFloorPoint(float z, float x) {
        final double u = z - floorFwdH;
        if (u > FIT_DEPTH * floorFwdH) return;
        // Nearest part of the line (points arrive nearest first): a straight-line fit over the
        // first NEAR_FIT_SPAN H-ROI distances, used to reconstruct the base when it is hidden.
        if (floorN == 0) nfLimit = u + NEAR_FIT_SPAN * floorFwdH;
        if (u <= nfLimit) {
            nfN++; nfU += u; nfUU += u * u; nfX += x; nfUX += u * x; nfXX += (double) x * x;
        }
        final double uu = u * u;
        floorN++;
        fU += u; fUU += uu; fUUU += uu * u; fUUUU += uu * uu;
        fX += x; fUX += u * x; fUUX += uu * x; fXX += (double) x * x;
        fUmin = (float) Math.min(fUmin, u); fUmax = (float) Math.max(fUmax, u);
        floorSpan = fUmax - fUmin;
    }

    // Forward extent (in H-ROI distances) of the near fit used to reconstruct a hidden base.
    private static final float NEAR_FIT_SPAN = 0.6f;

    /** True when most of the predicted line span on this row is masked as glare. */
    private boolean predictionMasked(int base, int pc, float expW) {
        final int hs = Math.max(2, Math.round(0.5f * Math.max(4f, expW)));
        final int a = Math.max(0, pc - hs), b = Math.min(imgW, pc + hs + 1);
        if (b <= a) return false;
        final byte[] gs = glareSrc;
        final int gl = glareLim;
        int n = 0;
        for (int c = a; c < b; c++) if ((gs[base + c] & 0xFF) >= gl) n++;
        return 2 * n >= b - a;
    }

    /**
     * v3: where the line crosses the H-ROI when the H-ROI itself is hidden — from a straight-line
     * fit on the floor through the nearest part of the look-ahead trace (seeded trace), at u = 0.
     * Returns the image column, or NaN when the trace is too short, not straight enough
     * (RMS > a quarter line width), or too far from the prediction {@code predX}
     * (more than {@code gateHalf}): then nothing consistent was seen ahead.
     */
    private float reconstructBase(float predX, int gateHalf) {
        final int minPts = Math.max(6, cfg.steerHeadingMinRows / 3);
        if (nfN < minPts || floorScaleH <= 0f) return Float.NaN;
        final float lineW = expectedLineWidth() > 0f ? expectedLineWidth() : lastTrackWidthPx;
        final double maxRms = 0.25 * lineW * floorScaleH;
        double A;
        if (floorN >= 2 * minPts && floorSpan >= 0.5f * floorFwdH) {
            // Long enough: parabola through all fitted points. On a curve a straight line
            // extrapolated back to the strip lands on the outside of the turn; the parabola
            // follows the start of the curve. A corner (not a smooth curve) fails the RMS test.
            final double n = floorN;
            final double a12 = fU, a13 = fUU, a22 = fUU, a23 = fUUU, a33 = fUUUU;
            final double det = n * (a22 * a33 - a23 * a23) - a12 * (a12 * a33 - a23 * a13) + a13 * (a12 * a23 - a22 * a13);
            if (Math.abs(det) < 1e-12) return Float.NaN;
            final double detA = fX * (a22 * a33 - a23 * a23) - a12 * (fUX * a33 - a23 * fUUX) + a13 * (fUX * a23 - a22 * fUUX);
            final double detB = n * (fUX * a33 - a23 * fUUX) - fX * (a12 * a33 - a23 * a13) + a13 * (a12 * fUUX - fUX * a13);
            final double detC = n * (a22 * fUUX - fUX * a23) - a12 * (a12 * fUUX - fUX * a13) + fX * (a12 * a23 - a22 * a13);
            A = detA / det;
            final double res = fXX - (A * fX + detB / det * fUX + detC / det * fUUX);
            if (Math.sqrt(Math.max(0.0, res) / n) > maxRms) return Float.NaN;
            // A sharp curve seen only from afar: the parabola and the straight line through the
            // nearest part disagree about where the line meets the strip → ambiguous, no base.
            final double nn = nfN, dl = nn * nfUU - nfU * nfU;
            if (Math.abs(dl) > 1e-12) {
                final double bl = (nn * nfUX - nfU * nfX) / dl, al = (nfX - bl * nfU) / nn;
                if (Math.abs(A - al) > 0.5 * lineW * floorScaleH) return Float.NaN;
            }
        } else {
            // Short trace: straight line through its nearest part.
            final double n = nfN, d = n * nfUU - nfU * nfU;
            if (Math.abs(d) < 1e-12) return Float.NaN;
            final double B = (n * nfUX - nfU * nfX) / d;
            A = (nfX - B * nfU) / n;
            final double res = nfXX - (A * nfX + B * nfUX);
            if (Math.sqrt(Math.max(0.0, res) / n) > maxRms) return Float.NaN;
        }
        final float col = (float) (imgW * 0.5f - 0.5f + A / floorScaleH);
        if (Math.abs(col - predX) > gateHalf) return Float.NaN;
        return col;
    }

    /**
     * Heading of the robot relative to the line from the last trace, into
     * fitHeadingDeg (deg, + = line heads right; before the calibrated zero) and
     * fitConfidence [0 … 1].
     *
     * The traced line on the floor (perspective removed) is fitted with a parabola
     * x = A + B·u + C·u² (u = forward distance beyond the H-ROI) and its direction
     * is taken at u = 0, i.e. where the line crosses the base of the ⊥. A parabola
     * gives that tangent without the bias a straight-line fit (the chord) would
     * have on a curve, and only the near look-ahead is fitted (FIT_DEPTH) so the
     * next turn does not bend it. With too little line visible for a parabola, the
     * straight-line direction is used.
     *
     * Confidence is full with at least steerHeadingMinRows rows spanning at least
     * half the H-ROI's floor distance, and with the points lying on the fitted
     * curve (RMS error ≤ a quarter of the line width). It fades to 0 with fewer
     * rows, a shorter span, or a line that is not a smooth curve (a 90° corner,
     * a junction) — the steering then falls back to the offset only.
     */
    private void computeHeading() {
        fitHeadingDeg = 0f;
        fitConfidence = 0f;
        fitC          = 0.0;
        if (floorN < 3) return;
        final double n = floorN;
        final double d2 = n * fUU - fU * fU;
        if (Math.abs(d2) < 1e-12) return;
        // Straight-line fit x = A + B·u (chord), and its residual.
        double B = (n * fUX - fU * fX) / d2;
        double A = (fX - B * fU) / n;
        double res = fXX - (A * fX + B * fUX);
        double slope = B;
        if (floorN >= 6 && floorSpan >= 0.8f * floorFwdH) {
            // Parabola: 3×3 normal equations, Cramer's rule.
            final double a11 = n,   a12 = fU,   a13 = fUU;
            final double a22 = fUU, a23 = fUUU, a33 = fUUUU;
            final double b1 = fX, b2 = fUX, b3 = fUUX;
            final double det = a11 * (a22 * a33 - a23 * a23) - a12 * (a12 * a33 - a23 * a13) + a13 * (a12 * a23 - a22 * a13);
            if (Math.abs(det) > 1e-12) {
                final double detA = b1 * (a22 * a33 - a23 * a23) - a12 * (b2 * a33 - a23 * b3) + a13 * (b2 * a23 - a22 * b3);
                final double detB = a11 * (b2 * a33 - a23 * b3) - b1 * (a12 * a33 - a23 * a13) + a13 * (a12 * b3 - b2 * a13);
                final double detC = a11 * (a22 * b3 - b2 * a23) - a12 * (a12 * b3 - b2 * a13) + b1 * (a12 * a23 - a22 * a13);
                A = detA / det; B = detB / det;
                final double C = detC / det;
                res   = fXX - (A * fX + B * fUX + C * fUUX);
                slope = B;                                              // tangent where the line crosses the H-ROI (u = 0)
                fitC  = C;
            }
        }
        fitHeadingDeg = (float) Math.toDegrees(Math.atan(slope));

        final float rows = Math.min(1f, (floorN - 1) / (float) Math.max(1, cfg.steerHeadingMinRows));
        final float span = Math.min(1f, floorSpan / Math.max(1e-3f, 0.5f * floorFwdH));
        // Fit quality: RMS distance of the points from the curve vs the line width.
        final float lineW = expectedLineWidth() > 0f ? expectedLineWidth() : lastTrackWidthPx;
        final double halfW = Math.max(1e-6, 0.5 * lineW * floorScaleH);
        final double rms = Math.sqrt(Math.max(0.0, res) / n);
        final float fit = (float) Math.max(0.0, Math.min(1.0, 1.5 - 2.0 * rms / halfW));
        fitConfidence = rows * span * fit;
    }

    /** Forget what was learned about the line beyond the colours (level, heading zero, motion). */
    private void resetLineModel() {
        kfReady           = false;
        curveSide = 0; curveFrames = 0; sinceCurveFrames = 1000; pinCapActive = false;
        resetPinCap();
        glarePinned       = false;
        if (cGain != null) java.util.Arrays.fill(cGain, 1f);
        if (cGainCount != null) java.util.Arrays.fill(cGainCount, 0);
        coastMs           = 0f;
        coastFrames       = 0;
        lastHeadTerm      = 0f;
        lastFrameNs       = 0L;
        calibLevel        = 0f;
        headingRefDeg     = 0f;
        calibHeadingCount = 0;
        centroidVel       = 0f;
        lastImgSlope      = 0f;
        holdFrames        = 0;
        lastSteering      = 0f;
        lastIntersection  = 0f;
        rejectedBands     = 0;
    }

    /** Fraction of the H-ROI + V-ROI masked as glare (H-ROI exact, V-ROI from the sampled histogram). */
    private float maskedFraction() {
        final float strip = (hRoiBot - hRoiTop) * (float) imgW, look = (vRoiBot - vRoiTop) * (float) imgW;
        final float lookMasked = maskedSamples > 0 ? look * maskedSampleHits / maskedSamples : 0f;
        final float total = strip + (maskedSamples > 0 ? look : 0f);
        return total > 0f ? Math.min(1f, (maskedPixels + lookMasked) / total) : 0f;
    }

    /** Fill the shared FrameData and hand it to the registered callback, if any. */
    private void fireCallback(int steering, int tracking, boolean trackFound, int centroidX) {
        if (frameCallback == null) return;
        final FrameData d = frameData;
        d.imgW         = imgW;
        d.imgH         = imgH;
        d.hRoiTopFrac  = hRoiTop / (float) imgH;
        d.hRoiBotFrac  = hRoiBot / (float) imgH;
        d.vRoiTopFrac  = vRoiTop / (float) imgH;
        d.ignoreTopFrac = ignoreTop / (float) imgH;
        d.steerRefXFrac = steerRefX / imgW;
        d.trackRefXFrac = trackRefX / imgW;
        // Box half-widths with perspective taper — uses the measured track width.
        final float boxW     = Math.max(12f, lastTrackWidthPx * 1.3f);
        d.halfBotFrac   = (boxW * 0.5f) / imgW;
        final float scaleNear = perspectiveWidthScale(d.hRoiTopFrac);
        final float scaleFar  = perspectiveWidthScale(d.vRoiTopFrac);
        final float taper     = (scaleNear > 1e-4f) ? scaleFar / scaleNear : 1f;
        d.halfTopFrac   = d.halfBotFrac * taper;
        // True detected position (not derived from the clamped steering value).
        d.centroidXFrac  = (trackFound && centroidX >= 0) ? centroidX / (float) imgW : -1f;
        d.trackWidthFrac = lastTrackWidthPx / imgW;
        d.tiltDeg        = calibrated ? output.tiltDeg : 0f;
        // Dynamic road path — copy so the caller's buffer is safe to read.
        final int n = Math.min(laPathCount, LA_MAX_PTS);
        System.arraycopy(laPathColFrac, 0, d.lookAheadColFrac, 0, n);
        System.arraycopy(laPathRowFrac, 0, d.lookAheadRowFrac, 0, n);
        d.lookAheadPointCount = n;
        // Calibration / colour state
        final boolean learnColors = cfg.autoColor && !colorPreset;
        d.isCalibrated  = calibrated;
        d.calibPhase    = calibrated ? CALIB_DONE
                        : (cfg.autoColor && !colorReady) ? CALIB_LEARNING_COLORS
                        : CALIB_FINDING_CENTER;
        final float colorPart = (cfg.autoColor && !colorReady)
                ? colorFramesSampled / (float) Math.max(1, cfg.colorCalibFrames) : 1f;
        final float centerPart = calibrated ? 1f
                : Math.min(1f, calibCount / (float) Math.max(1, cfg.calibFrames));
        d.calibProgress = learnColors ? 0.5f * colorPart + 0.5f * centerPart : centerPart;
        d.calibIssue      = calibrated ? ISSUE_NONE : calibIssue;
        d.colorModelReady = cfg.autoColor && colorReady;
        d.lineColorArgb   = lineColorArgb;
        d.floorColorArgb  = floorColorArgb;
        d.colorSeparation = colorSeparation;
        d.threshold       = output.threshold;
        // Control values
        d.trackFound   = trackFound;
        d.steering     = steering;
        d.tracking     = tracking;
        // Steering components (output units, i.e. half the internal scale) and
        // distractor-rejection state. Zero during calibration.
        final boolean live = calibrated && isProcessing;
        d.steerOffsetTerm   = live ? 0.5f * output.offsetTerm  : 0f;
        d.steerHeadingTerm  = live ? 0.5f * output.headingTerm : 0f;
        d.headingDeg        = live ? output.headingDeg  : 0f;
        d.headingConfidence = live ? output.headingConf : 0f;
        d.lineHeld          = live && output.held;
        d.rejectedBands     = live ? output.rejected : 0;
        d.baseReconstructed = live && output.reconstructed;
        d.coasting          = live && output.coasting;
        d.maskedFrac        = live && maskActive ? maskedFraction() : 0f;
        final boolean corr  = live && corrUsed;
        d.corridorCoverage  = corr ? corrCoverage : 0f;
        d.corridorFitRms    = corr ? corrRms : 0f;
        d.curveA            = corr ? (float) kx[0] : 0f;
        d.curveB            = corr ? (float) kx[1] : 0f;
        d.curveC            = corr ? (float) kx[2] : 0f;
        frameCallback.onFrame(d);
    }

    /**
     * Tally this frame into the current 1-second window. When a full second has
     * elapsed, the accumulated count becomes the reported FPS and a fresh window
     * starts — so {@link #getFramesPerSecond()} always reflects the last whole
     * second, not a cumulative total.
     */
    private void updateFps() {
        final long now = System.nanoTime();
        if (fpsWindowStartNs == 0L) { fpsWindowStartNs = now; firstFrameNs = now; }
        totalFrames++;
        elapsedNs = now - firstFrameNs;
        fpsFrameCount++;
        if (now - fpsWindowStartNs >= 1_000_000_000L) {   // one second elapsed
            framesPerSecond  = fpsFrameCount;
            fpsFrameCount    = 0;
            fpsWindowStartNs = now;
        }
    }

    // =========================================================================
    //  Otsu's method  (custom, from scratch — no OpenCV)
    // =========================================================================

    /**
     * Otsu's automatic threshold selection.
     *
     * Given a 256-bin histogram, this finds the level t that splits the pixels
     * into two classes — floor and track — so that the intra-class variance is
     * minimised. Minimising intra-class variance is mathematically equivalent to
     * MAXIMISING the between-class variance
     *
     *     σ_b²(t) = w0(t)·w1(t)·(µ0(t) − µ1(t))²
     *
     * which is the cheap single-pass form computed here (the total variance is
     * constant, so the t that maximises σ_b² is exactly the t that minimises the
     * weighted within-class variance). w0/w1 are the class pixel counts and
     * µ0/µ1 their mean levels. We track running sums so the whole sweep is a
     * single O(256) loop with no per-pixel work.
     *
     * @param hist  256-bin histogram (hist[g] = number of pixels with value g)
     * @param total number of pixels accumulated into {@code hist}
     * Also stores the difference between the two class means at that threshold
     * in {@link #otsuContrast} (0 when the histogram cannot be split).
     *
     * @return      optimal threshold in [0,255]; 128 if the histogram is empty
     */
    private int otsuThreshold(int[] hist, long total) {
        otsuContrast = 0;
        if (total <= 0) return 128;

        // Weighted sum of all levels: Σ g·hist[g].
        double sumAll = 0.0;
        for (int g = 0; g < 256; g++) {
            sumAll += (double) g * hist[g];
        }

        double sumBack = 0.0;   // Σ g·hist[g] for the background class [0..t]
        long   wBack   = 0;     // pixel count of the background class
        double bestVar = -1.0;
        int    bestT   = 128;

        // Sweep every candidate threshold, moving level t from background→foreground.
        for (int t = 0; t < 256; t++) {
            wBack += hist[t];
            if (wBack == 0) continue;          // background still empty
            long wFore = total - wBack;        // foreground class (t+1 .. 255)
            if (wFore == 0) break;             // foreground emptied — done

            sumBack += (double) t * hist[t];
            double meanBack = sumBack / wBack;
            double meanFore = (sumAll - sumBack) / wFore;

            double diff       = meanBack - meanFore;
            double betweenVar = (double) wBack * (double) wFore * diff * diff;

            if (betweenVar > bestVar) {
                bestVar = betweenVar;
                bestT   = t;
                otsuContrast = (float) -diff;
            }
        }
        return bestT;
    }
}
