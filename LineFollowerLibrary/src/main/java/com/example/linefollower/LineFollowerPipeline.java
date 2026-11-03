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

        // ═════════════════════════════════════════════════════════════════
        //  v3 — PATH TRACKING (the default)
        // ═════════════════════════════════════════════════════════════════
        // Floor distances below are in CAMERA HEIGHTS (the height of the phone's
        // camera above the floor): 1.0 = one camera height, e.g. 20 cm when the
        // camera is 20 cm above the floor. Curvatures are 1 / camera height.

        // ON (default): v3 — the whole visible path is traced, kept in a path
        // memory on the floor, and steering / tracking come from that path (pure
        // pursuit, curvature ahead). OFF: the v2 "⊥" pipeline (scan strip +
        // look-ahead), unchanged, for comparison. Before calibration v3 frames are
        // processed by the v2 pipeline.
        public boolean pathTracking = true;

        // Working region (ROI), found during calibration: its bottom is just above
        // the robot body (bottomIgnoreFrac, raised if the lowest rows are not
        // floor), its top where the floor / line stops being reliably visible
        // (table edge, background) minus this safety margin, as a fraction of the
        // visible floor distance. Each frame only reads pixels inside the ROI (full
        // image width — in curves the line reaches the image edge).
        public boolean roiAuto = true;
        public float roiTopMarginFrac = 0.10f;

        // Learn the camera tilt from how the line width changes with the image row
        // during calibration (a straight line of constant width narrows toward the
        // horizon). Used only if the measurement is consistent; otherwise
        // cameraTiltDeg is used.
        public boolean learnCameraTilt = true;

        // Distance between two traced path points (camera heights). 0.1 = 2 cm at
        // 20 cm camera height.
        public float pathStep = 0.10f;

        // Order in which the path is traced each frame (see TRACE_* constants):
        // near → far, far → near, or near → far plus a second pass that re-reads
        // the gaps between two good sections from both sides (default).
        public int pathTraceDirection = TRACE_BOTH;

        // ON (default): the path is traced, fitted and remembered on the FLOOR
        // (bird's-eye coordinates from the camera geometry; only the path points
        // are converted, never the image). OFF: the same algorithm in image
        // coordinates (for comparison only — perspective then bends straight lines).
        public boolean pathFloorSpace = true;

        // ON (default): the traced path is kept between frames and moved with the
        // robot's estimated motion, so a section that is hidden now (glare, shadow,
        // an object) is filled in from earlier frames. OFF: every frame starts from
        // scratch (for comparison).
        public boolean pathMemory = true;

        // How long (frames) a remembered path point stays trusted without being
        // seen again. Its confidence halves after about 0.7 × this many frames.
        public int pathMemoryFrames = 120;

        // Longest stretch (camera heights) of unreadable path that the tracer
        // bridges by continuity when it has no memory of it (e.g. a glare spot on
        // a part of the path never seen before, or a stretch that looks like bare
        // floor between two visible sections). 2.5 = 50 cm at 20 cm height.
        public float pathMaxGap = 2.5f;

        // Colour match tolerance in standard deviations of the colours learned at
        // calibration. A pixel that is within this of NEITHER the line nor the
        // floor colour is "unreliable" (glare, a colour patch, an object) — it
        // counts neither as line nor as floor.
        public float colorMatchSigma = 4.0f;

        // Brightness change (×/÷) allowed around the local light level when a
        // pixel is matched to the learned colours. Shadows and uneven light change
        // the brightness of line and floor alike, so the local level is measured
        // from the floor around the line and the colours are matched relative to it.
        public float lightLocalRange = 1.6f;

        // Pure pursuit: the steering aims at the point of the path this far
        // (camera heights) from the robot's turning point. Larger = smoother, less
        // weaving, but cuts tight curves a little; smaller = tighter, more weaving.
        public float pursuitLookAhead = 1.2f;

        // Turning point of the robot (axle centre) behind the camera's floor point,
        // in camera heights (0.4 = 8 cm at 20 cm height).
        public float pursuitPivotBehind = 0.4f;

        // Steering output per unit of pure-pursuit curvature (1 / camera height).
        // Steering = steerPerCurvature × 2·lateral / distance² of the target point,
        // limited to ±50. The default suits "move steering" with gain 1 on a robot
        // whose wheelbase is about 0.65 camera heights.
        public float steerPerCurvature = 30f;

        // Automatic steering scale. How much the robot actually turns for a given
        // Steering value depends on the program's gain and the robot (wheelbase,
        // motors), which the library cannot know. While the robot follows a curve
        // steadily it turns exactly as much as the path curves, so the ratio of the
        // path curvature to the curvature that was asked for reveals the robot's
        // response; the steering is scaled by its inverse (slowly, ×0.5 … ×2.5).
        // OFF: Steering = steerPerCurvature × pursuit curvature, always.
        public boolean steerAutoGain = true;

        // Tracking from the path ahead: 100 on a straight path, falling linearly
        // with the sharpest curvature found within powerPreview camera heights
        // ahead, from powerCurvatureDeadband (still 100) to powerFullCurvature
        // (the minimum). 1.5 = a curve of radius 0.67 camera heights (13 cm).
        public float powerFullCurvature = 1.5f;
        public float powerCurvatureDeadband = 0.2f;
        public float powerPreview = 2.0f;

        // Largest change of the tracking output per frame (up / down), so it moves
        // smoothly instead of jumping. A truly lost line still drops at once.
        public float powerRiseRate = 3f;
        public float powerFallRate = 8f;
    }

    // =========================================================================
    //  v3 path tracking — public constants
    // =========================================================================

    /** Config.pathTraceDirection: trace from the robot outward (near → far). */
    public static final int TRACE_BOTTOM_UP = 0;
    /** Config.pathTraceDirection: trace from the far end of the ROI toward the robot. */
    public static final int TRACE_TOP_DOWN  = 1;
    /** Config.pathTraceDirection: near → far, then gaps re-read from both sides (default). */
    public static final int TRACE_BOTH      = 2;

    /** Capacity of the FrameData path arrays. */
    public static final int PATH_MAX_POINTS = 64;
    /** Columns / rows of the FrameData.unreliableGrid over the ROI. */
    public static final int RELIABILITY_GRID_COLS = 24;
    public static final int RELIABILITY_GRID_ROWS = 32;

    /** FrameData.pathSource: point measured in this frame. */
    public static final byte PATH_SRC_FRESH      = 0;
    /** FrameData.pathSource: point from the path memory (outside the ROI, or not readable). */
    public static final byte PATH_SRC_MEMORY     = 1;
    /** FrameData.pathSource: point in an unreliable region this frame, filled from memory. */
    public static final byte PATH_SRC_UNRELIABLE = 2;
    /** FrameData.pathSource: point bridged by continuity (no memory, no measurement). */
    public static final byte PATH_SRC_PREDICTED  = 3;

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

        // ── v3 path tracking ────────────────────────────────────────────────
        /** True when this frame was processed by the v3 path tracker (false: v2 ⊥ pipeline). */
        public boolean pathMode;
        /** Working region (ROI) top / bottom edge as a fraction of frame height (full width). */
        public float roiTopFrac, roiBotFrac;
        /** Path points, nearest to farthest, as fractions of frame width / height (may lie
         *  outside 0…1 for remembered points that are out of view). */
        public final float[] pathColFrac = new float[PATH_MAX_POINTS];
        public final float[] pathRowFrac = new float[PATH_MAX_POINTS];
        /** The same points on the floor, robot frame, camera heights: forward / lateral (+ = right). */
        public final float[] pathFwd = new float[PATH_MAX_POINTS];
        public final float[] pathLat = new float[PATH_MAX_POINTS];
        /** Where each point came from: PATH_SRC_FRESH / _MEMORY / _UNRELIABLE / _PREDICTED. */
        public final byte[]  pathSource = new byte[PATH_MAX_POINTS];
        /** Number of valid path points (0 = no path). */
        public int pathPointCount;
        /** Points measured this frame / taken from memory / in an unreliable region this frame. */
        public int pathFreshPoints, pathMemoryPoints, pathUnreliablePoints;
        /** Fraction [0 … 1] of the path ahead of the robot that came from memory (not measured now). */
        public float pathMemoryFrac;
        /** Pure-pursuit target point: image fractions (may be outside 0…1) and floor (robot frame). */
        public float lookAheadXFrac, lookAheadYFrac, lookAheadFwd, lookAheadLat;
        /** True when a look-ahead target exists this frame. */
        public boolean lookAheadValid;
        /** Path curvature at the robot and the sharpest curvature ahead (1 / camera height, + = curves right). */
        public float pathCurvature, pathCurvatureAhead;
        /** Lateral offset of the path at the camera's floor point (camera heights, + = path is right). */
        public float pathOffset;
        /** Path direction at the robot relative to straight ahead, degrees (+ = heads right). */
        public float pathHeadingDeg;
        /** Length of the known path ahead of the robot (camera heights). */
        public float pathLength;
        /** Confidence [0 … 1] of the path ahead (1 = all measured now; drops only while it is unknown). */
        public float pathConfidence;
        /** Unreliable cells of the ROI (RELIABILITY_GRID_COLS × RELIABILITY_GRID_ROWS, row-major
         *  from the ROI top-left; 1 = pixels match neither the line nor the floor colours). */
        public final byte[] unreliableGrid = new byte[RELIABILITY_GRID_COLS * RELIABILITY_GRID_ROWS];
        /** Number of unreliable cells. */
        public int unreliableCells;
        /** Estimated robot motion since the previous frame: forward / lateral (camera heights), yaw (deg, + = right). */
        public float motionFwd, motionLat, motionYawDeg;
        /** Camera tilt used for the floor geometry (deg; learned at calibration or Config.cameraTiltDeg). */
        public float cameraTiltUsedDeg;
        /** Line width on the floor learned at calibration (camera heights). */
        public float lineWidthFloor;
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
    private static final int STRIPE_EXTRA = 512;
    private static final boolean STRIPE_LINE_MODEL = false;   // line model from the stripe statistics only (measured worse; the stripe pixels join the line cluster instead)
    private boolean colorReady;
    private boolean colorPreset;        // set via setTrackColors(); survives re-init
    private int     colorFramesSampled;
    private int     sampleCount;
    private short[] sampY, sampU, sampV, sampCol, sampRow;
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
        colWeight         = new int[width];
        colLevel          = new int[width];
        work              = new byte[width * height];
        // (+ STRIPE_EXTRA slots for the stripe's own pixels, v3 calibration only)
        final int maxSamples = Math.max(1, config.colorCalibFrames) * SAMPLES_PER_FRAME + STRIPE_EXTRA;
        sampY             = new short[maxSamples];
        sampU             = new short[maxSamples];
        sampV             = new short[maxSamples];
        sampCol           = new short[maxSamples];
        sampRow           = new short[maxSamples];
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
        // v3 path tracking
        final int prSize  = Math.max(PROF_MAX, width / 2 + 2);
        prY = new float[prSize]; prU = new float[prSize]; prV = new float[prSize];
        prLam = new float[prSize]; prTmp = new float[prSize]; prCls = new byte[prSize]; prU2 = new float[prSize];
        v3Ready           = false;
        modelReady        = false;
        nFloorModels      = 0;
        cgRow             = null;
        roiTop            = vRoiTop;
        roiBot            = hRoiBot;
        setGeometry(config.cameraTiltDeg);
        rfA = 0f; rfCos = 1f; rfSin = 0f;
        resetPathState();
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
        v3Ready            = false;
        modelReady         = false;
        clearCalibRows();
        resetPathState();
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
        if (cfg.pathTracking) return calibrateV3();
        if (calibrated) return true;
        laPathCount = 0;

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
        if (cfg.pathTracking && v3Ready && calibrated) return processBoundV3();
        updateFps();
        final byte[] pix = prepareWorkImage(vRoiTop, hRoiBot);
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
        final int cap   = sampY.length - STRIPE_EXTRA;

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
            if (!fitDiscriminant(n, k, c, count, line, true)) return false;
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

        if (!fitDiscriminant(n, k, c, count, line, true)) return false;
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
    private boolean fitDiscriminant(int n, int k, double[] c, int[] count, int line, boolean strictOthers) {
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
            if (strictOthers && wq < 0.8 * wd) { calibIssue = ISSUE_LOW_CONTRAST; return false; }
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
        final int[] colW = colWeight;
        final int runGapTol = runGapTol();
        long   curSum = 0;
        double curMoment = 0;
        int    curStart = -1, curLast = -1, gapCols = 0;
        bandSum = 0; bandMoment = 0; bandStart = -1; bandEnd = -1; bandScore = -1f; bandTrimmed = false;
        for (int col = lo; col <= hi; col++) {
            final int w = (col < hi) ? colW[col] : 0;
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
        // Pixel count, capped at what a line of the expected width can hold, so a
        // bigger object of acceptable width does not win just by being bigger.
        long s = sum;
        if (expectW > 0f) s = Math.min(s, (long) (1.2f * expectW * Math.max(1, hRoiBot - hRoiTop)));
        float score = scoreBand(s, (float) (moment / sum), w, prefX, scale, expectW);
        score *= levelFactor(bandLevel(start, end));
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
                centroidVel = 0f;
            } else {
                if (!better) relockCount = 0;
                bandSum = lockSum; bandMoment = lockMom; bandStart = lockStart; bandEnd = lockEnd;
                bandTrimmed = lockTrim;
            }
        } else {
            relockCount = 0;
        }

        boolean trackFound;
        int     centroidX;
        boolean held               = false;
        float   forcedIntersection = -1f;   // ≥ 0 overrides the look-ahead stage
        float   pinnedSteering     = 0f;    // ±100 while the locked line is lost

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
        final int vThreshold = lookAheadThreshold(pixels, threshold, bright);
        traceLookAhead(pixels, vThreshold, bright, trackFound ? centroidX : -1, filter);
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

        // Track-memory override: once the locked line has left the gate we report a
        // hard turn (100 %) regardless of the look-ahead geometry, signalling that
        // you have moved fully off the line and a new one is being sought.
        if (forcedIntersection >= 0f) intersection = forcedIntersection;

        // Line momentarily hidden mid-image: hold the last good outputs.
        if (held) {
            steering     = lastSteering;
            intersection = lastIntersection;
        } else if (trackFound) {
            lastSteering     = steering;
            lastIntersection = intersection;
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
        output.held         = held;
        output.rejected     = rejectedBands;
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
            for (int row = vRoiTop; row < vRoiBot; row += stepR) {
                final int base = row * W;
                for (int col = 0; col < W; col += stepC) {
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
        final int     W      = imgW;
        final int     half   = cfg.vRoiHalfWidthPx;
        final float[] rowCx  = this.rowCx;
        final int     maxGap = Math.max(8, cfg.hRoiThickPx);   // stop after this many empty rows
        final int     runGapTol = runGapTol();
        final boolean trackFound = centroidX >= 0;

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
        if (trackFound) {
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
        int    trackCenter = centroidX;
        float  drift       = filter ? Math.max(-2f, Math.min(2f, lastImgSlope)) : 0f;  // columns per row, upward
        // Width factor for the line's slant, from the previous frame (fixed for this
        // frame, so a distractor cannot inflate the allowed width row after row).
        final float slantK = (float) Math.sqrt(1f + drift * drift);
        double prevC       = centroidX;
        int    prevRow     = (hRoiTop + hRoiBot) / 2;
        int    gap         = 0;
        int    farRow      = -1;   // farthest (topmost) V-ROI row where the line was accepted
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
        if (trackFound) {
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
            final int vLeft  = Math.max(0,     pc - rowHalf);
            final int vRight = Math.min(W, pc + rowHalf);
            final int base   = row * W;

            // Scan the window, picking the run whose centre is nearest the
            // predicted centre. Small gaps inside a run are bridged.
            long   runW = 0;   double runM = 0;   int runStart = -1, runLast = -1, runGap = 0;
            double bestC = Double.NaN; long bestW = 0; double bestDist = Double.MAX_VALUE;
            for (int col = vLeft; col <= vRight; col++) {
                final boolean on = col < vRight && isTrack(pixels[base + col] & 0xFF, vThreshold, bright);
                if (on) {
                    if (runStart < 0) { runStart = col; runW = 0; runM = 0; }
                    runW++; runM += col; runGap = 0; runLast = col;
                } else if (runStart >= 0 && (col == vRight || ++runGap > runGapTol)) {
                    double c = runM / runW; long wCount = runW;
                    boolean ok = true;
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
                        if (lOk && !rOk)      { c = runStart + expRow * 0.5;    wCount = Math.round(expRow); }
                        else if (rOk && !lOk) { c = runLast + 1 - expRow * 0.5; wCount = Math.round(expRow); }
                        else if (!(lOk && rOk) && (span > maxRow || clipped)) ok = false;
                    }
                    if (ok) {
                        final double d = Math.abs(c - (filter ? pred : trackCenter));
                        if (d < bestDist) { bestDist = d; bestC = c; bestW = wCount; }
                    }
                    runStart = -1;
                }
            }

            // Accept only a run that is big enough AND close enough to the line we
            // are following; otherwise this row has no usable track → count a gap.
            if (bestW >= minRowW && bestDist <= vMaxDrift) {
                rowCx[row]  = (float) bestC;
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
                if (++gap > maxGap) break;                 // line ended → stop look-ahead
            }
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
        traceFarRow  = farRow;
        traceLaW     = laW;
        traceLaM     = laM;
        traceLaCount = laCount;
    }

    /**
     * Add one point of the line on the floor (forward z, lateral x; camera heights).
     * Points more than FIT_DEPTH H-ROI distances beyond the H-ROI are not fitted.
     */
    private void addFloorPoint(float z, float x) {
        final double u = z - floorFwdH;
        if (u > FIT_DEPTH * floorFwdH) return;
        final double uu = u * u;
        floorN++;
        fU += u; fUU += uu; fUUU += uu * u; fUUUU += uu * uu;
        fX += x; fUX += u * x; fUUX += uu * x; fXX += (double) x * x;
        fUmin = (float) Math.min(fUmin, u); fUmax = (float) Math.max(fUmax, u);
        floorSpan = fUmax - fUmin;
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
        fillPathFrameData(d);
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
    //  v3 PATH TRACKING
    // =========================================================================
    //
    //  Overview (details at each method):
    //   • Colour model: the line and the floor colours learned at calibration,
    //     each with its spread (covariance). A pixel is LINE, FLOOR or UNKNOWN —
    //     unknown when it fits neither (glare, a colour patch, an object, dirt).
    //     Brightness may change locally (shadow, uneven light): the local light
    //     level is measured from the floor next to the line and both colours are
    //     matched relative to it. Unknown pixels are never taken as line or floor.
    //   • Geometry: an exact pinhole model (tilt learned from the line width per
    //     row) converts image points ↔ floor points (bird's-eye view). Only path
    //     points are converted; no image is warped.
    //   • Path: a chain of points on the floor, one every Config.pathStep, in the
    //     robot frame (x forward, y right, camera heights). Kept between frames
    //     (path memory) and moved with the robot's motion, which is estimated from
    //     how the measured path shifts between frames.
    //   • Each frame the path is traced: at every point a short profile ACROSS the
    //     predicted path is sampled (a window on a subsampled image), and the
    //     line's edges are located. A measurement must have the line colour, the
    //     expected width and continue the path; otherwise the point is
    //     "unreliable" (filled from memory / continuity) or "absent" (clean floor
    //     where the line should be — real evidence that it is not there).
    //   • Control: pure pursuit on the smoothed path for steering; curvature ahead
    //     and the known path length for tracking (rate-limited).

    private static final byte CL_OUT = 0, CL_FLOOR = 1, CL_LINE = 2, CL_UNK = 3;
    private static final byte ST_NONE = 0, ST_MEAS = 1, ST_UNREL = 2, ST_ABSENT = 3, ST_OUT = 4;
    private static final int  END_OUT = 0, END_ABSENT = 1, END_UNKNOWN = 2;
    private static final int  NODE_MAX = 128, PROF_MAX = 72, FLOOR_MAX = 3;
    // Motion filter: initial (= largest) variance and per-frame growth (velX in camera
    // heights per frame, velT in radians per frame); largest remembered-point variance (× lineW²).
    private static final float MOTION_P0X = 0.15f * 0.15f, MOTION_P0T = 0.10f * 0.10f;
    private static final float MOTION_QX = 0.02f * 0.02f, MOTION_QT = 0.04f * 0.04f;
    private static final float MEM_VAR_MAX = 1f;
    // largest change of the motion per frame (acceleration ~2 m/s², ~20 rad/s² at 30 fps,
    // camera 0.2 m high) and the largest motion per frame
    private static final float MOTION_DX = 0.012f, MOTION_DT = 0.025f, VEL_X_MAX = 0.25f, VEL_T_MAX = 0.12f;

    private boolean v3Ready;               // calibrated in path mode

    // ── Geometry actually used (pinhole; tilt possibly learned) ─────────────
    private double gFoc, gCx, gCy, gSinT = Math.sin(Math.PI / 4), gCosT = Math.cos(Math.PI / 4);
    private float  gTiltDeg = 45f;
    // camera floor frame → robot frame (the calibrated straight line is the x axis)
    private float  rfA, rfCos = 1f, rfSin = 0f;
    // image-plane variant (Config.pathFloorSpace = false)
    private float  ipK = 1f, ipU0, ipV0, ipSlope;
    // line width in pixels as a function of the image row: w(v) = calWA + calWB·v
    private float  calWA, calWB;
    private int    roiTop, roiBot;          // ROI rows [roiTop, roiBot)
    private float  lineW;                   // line width in plane units
    private float  roiFarFwd;               // forward distance of the ROI top (plane units, centre)

    // ── Colour models (Y, U−128, V−128); inverse covariance ×6, a = Σ⁻¹m, k = mᵀΣ⁻¹m ──
    private final float[] cmL = new float[3], ciL = new float[6], caL = new float[3];
    private float  ckL;
    private final float[] cmF = new float[3 * FLOOR_MAX], ciF = new float[6 * FLOOR_MAX], caF = new float[3 * FLOOR_MAX];
    private final float[] ckF = new float[FLOOR_MAX];
    private int    nFloorModels;
    private boolean modelLumaOnly;          // Config.autoColor == false: brightness only
    private boolean modelReady;
    private float  dirLF0, dirLF1, dirLF2, dirLFn;   // line − floor (main) for the linear line-ness

    // ── Profile buffers (also used for full-row scans) ──────────────────────
    private float[] prY, prU, prV, prLam, prTmp, prU2;
    private byte[]  prCls;
    private int     prN;
    private float   prStep, prH0;
    private float   prGain = 1f;            // local light level of the last profile
    private float   lightRef = 1f;          // typical light level of the floor next to the line
    private float   ambRef = 1f;            // room light level for the floor plausibility range
    // result of analyzeProfile()
    private byte    msStatus;
    private float   msOff, msQual;
    private boolean msTrim;
    private boolean msLineNear;             // a line-like run lies just beside the prediction (not taken)
    // results of the conversions
    private float   pU, pV, qX, qY;
    // last sampled pixel
    private float   sY, sU, sV;
    // classify() side outputs
    private float   clsLam;

    // ── Path memory (robot frame of the previous frame) ─────────────────────
    private final float[] memX = new float[NODE_MAX], memY = new float[NODE_MAX], memConf = new float[NODE_MAX];
    private final short[] memAge = new short[NODE_MAX];
    private int     memN;

    // ── Path of the current frame ───────────────────────────────────────────
    private final float[] ndX = new float[NODE_MAX], ndY = new float[NODE_MAX];      // final positions
    private final float[] ndPX = new float[NODE_MAX], ndPY = new float[NODE_MAX];    // predicted (memory)
    private final float[] ndNX = new float[NODE_MAX], ndNY = new float[NODE_MAX];    // unit normals
    private final float[] ndR = new float[NODE_MAX], ndQ = new float[NODE_MAX], ndConf = new float[NODE_MAX];
    private final byte[]  ndSt = new byte[NODE_MAX], ndSrc = new byte[NODE_MAX];
    private final short[] ndAge = new short[NODE_MAX];
    private int     ndN;
    private final float[] tmpA = new float[NODE_MAX], tmpB = new float[NODE_MAX], tmpC = new float[NODE_MAX];
    private final byte[]  tmpS = new byte[NODE_MAX], tmpT = new byte[NODE_MAX];
    private final short[] tmpG = new short[NODE_MAX];
    // smoothed path
    private final float[] smX = new float[NODE_MAX], smY = new float[NODE_MAX];
    private final float[] smTX = new float[NODE_MAX], smTY = new float[NODE_MAX], smK = new float[NODE_MAX];
    private final float[] smW = new float[NODE_MAX];
    private int     pathEnd = END_OUT;      // how the traced path ended (far side)
    private float   pathExitSide;           // lateral side (+1 right / −1 left) the path left the view

    // ── Motion (per frame, robot frame) ─────────────────────────────────────
    private float   velX, velY, velT;       // estimated motion of the robot per frame
    private float   corrX, corrY, corrT;    // correction found this frame
    // Motion estimate as a Kalman filter: covariance of (velX, velT) — xx, xt, tt. A frame
    // whose measurements say little about the motion (a few noisy points under glare,
    // a straight line that hides forward motion) changes the estimate little.
    private float   mPxx, mPxt, mPtt;
    private float   kPxx, kPxt, kPtt;       // posterior of this frame (committed if used)
    private boolean motionMeasured;         // corrX / corrT come from this frame's measurements
    private boolean lastMotionMeasured;     // … and in the previous frame
    // Per-point position variance across the path (plane units²): memory, and the
    // measurement of this frame. Measured points are fused with the remembered ones.
    private final float[] memVar = new float[NODE_MAX], ndVar = new float[NODE_MAX], ndMV = new float[NODE_MAX];

    // ── Control state ───────────────────────────────────────────────────────
    private float   v3Tracking = 100f;      // rate-limited tracking output
    private int     outSteer = 0, outTrack = 100;   // last whole-number outputs (hysteresis)
    private static final float OUT_HYST = 0.8f;

    private static final float AG_RATE = 0.01f;   // steering-scale adaptation per frame
    private int     v3LostFrames, v3HoldFrames, v3BlindFrames;
    private float   v3LastSteer;
    private int     v3Rejected;
    // results of computeControl()
    private boolean ctTarget;
    private float   ctTX, ctTY, ctKappa, ctSteer, ctOffset, ctHeading, ctCurv, ctCurvAhead, ctLength, ctConf, ctMemFrac;
    private int     ctFresh, ctMem, ctUnrel;
    private boolean v3Found, v3Held, v3Pinned;
    // automatic steering scale: smoothed path curvature at the axle and issued curvature
    private float   gainCorr = 1f, agPath, agCmd;
    private float   ctCurvAheadPrev, ctTrackCurve = 100f, ctLook, ctCurvPivot, ctGuard;
    private float   roiNearFwd = 0.5f;      // forward distance of the ROI bottom (centre column)
    private float   guardU, guardSide;      // visibility guard state

    // ── Reliability grid ────────────────────────────────────────────────────
    private final byte[] relGrid = new byte[RELIABILITY_GRID_COLS * RELIABILITY_GRID_ROWS];
    private final float[] gridY = new float[4], gridU = new float[4], gridV = new float[4], gridG = new float[4];
    private int     relCells;

    // ── Calibration (geometry phase) accumulators ───────────────────────────
    private int[]   cgRow;                  // scan rows, bottom → top
    private float[] cgSumC, cgSumW, cgSumGood, cgSumFloor;
    private int[]   cgClean, cgFrames;
    private int     cgN;

    // =========================================================================
    //  v3 — colour model
    // =========================================================================

    /** Inverse of the symmetric 3×3 (yy yu yv uu uv vv) into out[off…off+5]; false if singular. */
    private static boolean invert3(double[] m, float[] out, int off) {
        final double a = m[0], b = m[1], c = m[2], d = m[3], e = m[4], f = m[5];
        final double i00 = d * f - e * e, i01 = c * e - b * f, i02 = b * e - c * d;
        final double i11 = a * f - c * c, i12 = b * c - a * e, i22 = a * d - b * b;
        final double det = a * i00 + b * i01 + c * i02;
        if (Math.abs(det) < 1e-9) return false;
        out[off] = (float) (i00 / det); out[off + 1] = (float) (i01 / det); out[off + 2] = (float) (i02 / det);
        out[off + 3] = (float) (i11 / det); out[off + 4] = (float) (i12 / det); out[off + 5] = (float) (i22 / det);
        return true;
    }

    /** Install one colour model from a mean and a covariance (yy yu yv uu uv vv). */
    private boolean setModel(boolean line, int slot, double mY, double mU, double mV, double[] cov) {
        final float[] m = line ? cmL : cmF, inv = line ? ciL : ciF, a = line ? caL : caF;
        final int o3 = line ? 0 : slot * 3, o6 = line ? 0 : slot * 6;
        if (!invert3(cov, inv, o6)) return false;
        m[o3] = (float) mY; m[o3 + 1] = (float) mU; m[o3 + 2] = (float) mV;
        a[o3]     = inv[o6] * m[o3] + inv[o6 + 1] * m[o3 + 1] + inv[o6 + 2] * m[o3 + 2];
        a[o3 + 1] = inv[o6 + 1] * m[o3] + inv[o6 + 3] * m[o3 + 1] + inv[o6 + 4] * m[o3 + 2];
        a[o3 + 2] = inv[o6 + 2] * m[o3] + inv[o6 + 4] * m[o3 + 1] + inv[o6 + 5] * m[o3 + 2];
        final float k = a[o3] * m[o3] + a[o3 + 1] * m[o3 + 1] + a[o3 + 2] * m[o3 + 2];
        if (k < 1e-6f) return false;
        if (line) ckL = k; else ckF[slot] = k;
        return true;
    }

    private void finishModels() {
        dirLF0 = cmL[0] - cmF[0]; dirLF1 = cmL[1] - cmF[1]; dirLF2 = cmL[2] - cmF[2];
        dirLFn = Math.max(1f, dirLF0 * dirLF0 + dirLF1 * dirLF1 + dirLF2 * dirLF2);
        modelReady = true;
    }

    /** Squared distance of colour (y,u,v) from model (inverse cov at inv[o6], a at a[o3], k), best gain in [gLo,gHi]. */
    private static float rayDist(float y, float u, float v, float[] inv, int o6, float[] a, int o3, float k,
                                 float gLo, float gHi) {
        final float q0 = y * (inv[o6] * y + 2f * (inv[o6 + 1] * u + inv[o6 + 2] * v))
                       + u * (inv[o6 + 3] * u + 2f * inv[o6 + 4] * v) + inv[o6 + 5] * v * v;
        final float q1 = a[o3] * y + a[o3 + 1] * u + a[o3 + 2] * v;
        float g = q1 / k;
        if (g < gLo) g = gLo; else if (g > gHi) g = gHi;
        final float d = q0 - 2f * g * q1 + g * g * k;
        return d > 0f ? d : 0f;
    }

    /**
     * Class of colour (y,u,v) for light level window [gLo, gHi]: LINE, FLOOR or
     * UNKNOWN (fits neither, or both about equally well). Also sets clsLam, a
     * linear line-ness (0 = floor colour, 1 = line colour) used to place edges.
     */
    private byte classify(float y, float u, float v, float gLo, float gHi, float gMid) {
        final float tau = cfg.colorMatchSigma * cfg.colorMatchSigma;
        final float dL = rayDist(y, u, v, ciL, 0, caL, 0, ckL, gLo, gHi);
        float dF = Float.MAX_VALUE;
        for (int j = 0; j < nFloorModels; j++) {
            final float d = rayDist(y, u, v, ciF, j * 6, caF, j * 3, ckF[j], gLo, gHi);
            if (d < dF) dF = d;
        }
        clsLam = ((y - gMid * cmF[0]) * dirLF0 + (u - gMid * cmF[1]) * dirLF1 + (v - gMid * cmF[2]) * dirLF2)
                 / (gMid * dirLFn);
        final boolean mL = dL <= tau, mF = dF <= tau;
        // both fit: the clearly better one (likelihood ratio > e³ ≈ 20 : 1), else unknown
        if (mL && (!mF || dF - dL > 6f)) return CL_LINE;
        if (mF && (!mL || dL - dF > 6f)) return CL_FLOOR;
        return CL_UNK;
    }

    /** Light level of colour (y,u,v) relative to the main floor colour, or −1 if it is not floor-like. */
    private float floorGain(float y, float u, float v) {
        final float[] inv = ciF, a = caF;
        final float q0 = y * (inv[0] * y + 2f * (inv[1] * u + inv[2] * v)) + u * (inv[3] * u + 2f * inv[4] * v) + inv[5] * v * v;
        final float q1 = a[0] * y + a[1] * u + a[2] * v;
        final float g = q1 / ckF[0];
        // plausible light changes only: with two neutral colours (white / dark grey) any
        // brightness could otherwise be "floor under some light". The range is centred on
        // the room's light level along the path (ambRef, which follows it slowly and only
        // within ×0.5…×2.5), so the robot can drive from a dim into a bright part.
        final float A = ambRef;
        if (g < 0.2f * A || g > 2.5f * A) return -1f;
        final float d = q0 - q1 * g;
        final float tau = cfg.colorMatchSigma * cfg.colorMatchSigma;
        if (d > 2f * tau) return -1f;
        // A pixel that is also the line colour under a plausible light level is not used
        // to measure the floor's light (with brightness only, a dark line is otherwise
        // "floor in deep shadow" and drags the estimate down).
        final float r = lightRange();
        final float dL = rayDist(y, u, v, ciL, 0, caL, 0, ckL, 1f / (r * r), r * r);
        return dL <= tau ? -1f : g;
    }

    /** Reads the 2×2-averaged colour around pixel (col,row) into sY / sU / sV (U, V centred on 0). */
    private void readPixel(int col, int row) {
        final int W = imgW;
        if (col < 0) col = 0; else if (col > W - 2) col = W - 2;
        if (row < 0) row = 0; else if (row > imgH - 2) row = imgH - 2;
        final int i = row * W + col;
        if (srcFormat == FMT_ARGB) {
            final int a = srcArgb[i], b = srcArgb[i + 1], c = srcArgb[i + W], d = srcArgb[i + W + 1];
            sY = 0.25f * (lumaOf(a) + lumaOf(b) + lumaOf(c) + lumaOf(d));
            if (modelLumaOnly) { sU = 0f; sV = 0f; return; }
            sU = 0.25f * (chromaUOf(a) + chromaUOf(b) + chromaUOf(c) + chromaUOf(d)) - 128f;
            sV = 0.25f * (chromaVOf(a) + chromaVOf(b) + chromaVOf(c) + chromaVOf(d)) - 128f;
        } else {
            final byte[] y = srcY;
            sY = 0.25f * ((y[i] & 0xFF) + (y[i + 1] & 0xFF) + (y[i + W] & 0xFF) + (y[i + W + 1] & 0xFF));
            if (srcFormat == FMT_YUV420 && !modelLumaOnly) {
                final int uvi = (row >> 1) * uvW + (col >> 1);
                sU = (srcU[uvi] & 0xFF) - 128f;
                sV = (srcV[uvi] & 0xFF) - 128f;
            } else { sU = 0f; sV = 0f; }
        }
    }

    // =========================================================================
    //  v3 — geometry (image ↔ plane)
    // =========================================================================

    /** Pinhole geometry for tilt {@code tiltDeg} (degrees below horizontal). */
    private void setGeometry(float tiltDeg) {
        gTiltDeg = tiltDeg;
        final double fov = Math.max(1f, cfg.verticalFovDeg);
        gFoc = (imgH * 0.5) / Math.tan(Math.toRadians(fov * 0.5));
        gCx = imgW * 0.5; gCy = imgH * 0.5;
        gSinT = Math.sin(Math.toRadians(tiltDeg)); gCosT = Math.cos(Math.toRadians(tiltDeg));
    }

    /** Plane point → continuous image position (pU, pV). False if the point cannot be seen (behind the camera). */
    private boolean planeToImage(float x, float y) {
        if (cfg.pathFloorSpace) {
            final float xc = x * rfCos - y * rfSin;
            final float yc = x * rfSin + y * rfCos + rfA;
            final double zc = xc * gCosT + gSinT;
            if (zc < 0.05) return false;
            pU = (float) (gCx + gFoc * yc / zc);
            pV = (float) (gCy + gFoc * (gCosT - xc * gSinT) / zc);
        } else {
            pV = ipV0 - x / ipK;
            pU = ipU0 + ipSlope * (ipV0 - pV) + y / ipK;
        }
        return true;
    }

    /** Continuous image position → plane point (qX, qY). False above the horizon. */
    private boolean imageToPlane(float u, float v) {
        if (cfg.pathFloorSpace) {
            final double yn = (v - gCy) / gFoc;
            final double down = yn * gCosT + gSinT;
            if (down < 1e-3) return false;
            final float xc = (float) ((gCosT - yn * gSinT) / down);
            final float yc = (float) ((u - gCx) / (gFoc * down)) - rfA;
            qX = xc * rfCos + yc * rfSin;
            qY = -xc * rfSin + yc * rfCos;
        } else {
            qX = (ipV0 - v) * ipK;
            qY = (u - ipU0 - ipSlope * (ipV0 - v)) * ipK;
        }
        return true;
    }

    /** Plane units per image pixel (sideways) at plane point (x, y). */
    private float pixelSizeAt(float x, float y) {
        if (!cfg.pathFloorSpace) return ipK;
        final double zc = (x * rfCos - y * rfSin) * gCosT + gSinT;
        return (float) (Math.max(0.05, zc) / gFoc);
    }

    /** Expected line width (plane units) at plane point (x, y). Overwrites pU / pV in image mode. */
    private float widthAt(float x, float y) {
        // The width measured per image row at calibration, converted to plane units at
        // this point: consistent with the image even if the camera tilt is not exact.
        if (!planeToImage(x, y)) return lineW;
        final float px = Math.max(3f, calWA + calWB * pV);
        return cfg.pathFloorSpace ? px * pixelSizeAt(x, y) : px * ipK;
    }

    /** True if plane point (x, y) is inside the ROI with a margin of {@code marginPx}. Sets pU / pV. */
    private boolean insideRoi(float x, float y, float marginPx) {
        if (!planeToImage(x, y)) return false;
        return pU >= marginPx && pU <= imgW - marginPx && pV >= roiTop + marginPx && pV <= roiBot - marginPx;
    }

    // =========================================================================
    //  v3 — one profile across the path
    // =========================================================================

    /**
     * Sample a profile across the path at plane point (px,py) along the unit normal
     * (nx,ny), ±half, and classify every sample relative to the local light level
     * (measured from the floor-like samples of this profile). Fills pr* arrays.
     */
    private void sampleProfile(float px, float py, float nx, float ny, float half, float wExp) {
        float step = Math.max(wExp / 10f, 1.6f * pixelSizeAt(px, py));
        int n = (int) (2f * half / step) + 1;
        if (n > PROF_MAX) { n = PROF_MAX; step = 2f * half / (n - 1); }
        if (n < 5) { n = 5; step = 2f * half / 4f; }
        prN = n; prStep = step; prH0 = -(n - 1) * step * 0.5f;
        int nG = 0, nR = 0, nV = 0;
        final float tau2 = 2f * cfg.colorMatchSigma * cfg.colorMatchSigma;
        for (int j = 0; j < n; j++) {
            final float s = prH0 + j * step;
            if (!planeToImage(px + s * nx, py + s * ny)) { prCls[j] = CL_OUT; continue; }
            if (pU < 0.5f || pV < roiTop + 0.5f || pU > imgW - 1.5f || pV > roiBot - 1.5f) { prCls[j] = CL_OUT; continue; }
            readPixel((int) (pU - 0.5f), (int) (pV - 0.5f));
            prY[j] = sY; prU[j] = sU; prV[j] = sV; prCls[j] = CL_UNK;
            nV++;
            final float g = floorGain(sY, sU, sV);
            if (g > 0f) prTmp[nG++] = g;
            // the floor colour at any light level (for the case below)
            final float q1 = caF[0] * sY + caF[1] * sU + caF[2] * sV, gr = q1 / ckF[0];
            if (gr > 0.1f && gr < 10f && rayDist(sY, sU, sV, ciF, 0, caF, 0, ckF[0], gr, gr) <= tau2) prU2[nR++] = gr;
        }
        // No floor within the usual light range, but most of the profile has the floor's
        // colour at one consistent light level (a part of the room much brighter or
        // darker than at calibration): that majority is the floor — the line is always
        // the minority of a profile across it. Its level is this profile's light level.
        if (nG < 4 && nR >= 4 && nR * 10 >= 6 * nV) {
            final float med = median(prU2, nR);
            int near = 0;
            for (int j = 0; j < nR; j++) if (prU2[j] >= med / 1.3f && prU2[j] <= med * 1.3f) near++;
            if (near * 10 >= 6 * nV) {
                nG = 0;
                for (int j = 0; j < nR; j++) if (prU2[j] >= med / 1.3f && prU2[j] <= med * 1.3f) prTmp[nG++] = prU2[j];
            }
        }
        classifyProfile(n, nG);
    }

    /** Classify prY/U/V[0…n) (CL_OUT entries kept) with the local light level from prTmp[0…nG). */
    private void classifyProfile(int n, int nG) {
        if (n > 2 * PROF_MAX) { classifyLongScan(n); return; }
        float gl = prGain;
        if (nG >= 4) gl = median(prTmp, nG);
        else if (nG > 0) gl = 0.5f * (prGain + median(prTmp, nG));
        prGain = gl;
        final float r = lightRange();
        final float gLo = gl / r, gHi = gl * r;
        for (int j = 0; j < n; j++) {
            if (prCls[j] == CL_OUT) { prLam[j] = 0f; continue; }
            prCls[j] = classify(prY[j], prU[j], prV[j], gLo, gHi, gl);
            prLam[j] = clsLam;
        }
    }

    /**
     * Long scans (a whole image row): the light level may change along the row (a
     * shadow edge), so each sample uses the mean light level of the floor-like
     * samples within ±16 samples (prefix sums of log gain), falling back to the
     * row's overall level where there are none.
     */
    private void classifyLongScan(int n) {
        final float r = lightRange();
        // prLam temporarily holds the prefix sums of log-gain, prTmp the prefix counts
        float sum = 0f, cnt = 0f, all = 0f; int allN = 0;
        for (int j = 0; j < n; j++) {
            final float g = prCls[j] == CL_OUT ? -1f : floorGain(prY[j], prU[j], prV[j]);
            if (g > 0f) { final float lg = (float) Math.log(g); sum += lg; cnt += 1f; all += lg; allN++; }
            prLam[j] = sum; prTmp[j] = cnt;
        }
        final float glAll = allN > 0 ? (float) Math.exp(all / allN) : prGain;
        final int H = 16;
        for (int j = 0; j < n; j++) {
            final int lo = Math.max(0, j - H) - 1, hi = Math.min(n - 1, j + H);
            final float c = prTmp[hi] - (lo >= 0 ? prTmp[lo] : 0f);
            final float sl = prLam[hi] - (lo >= 0 ? prLam[lo] : 0f);
            final float gl = c >= 4f ? (float) Math.exp(sl / c) : glAll;
            prU2[j] = gl;
        }
        prGain = glAll;
        for (int j = 0; j < n; j++) {
            if (prCls[j] == CL_OUT) { prLam[j] = 0f; continue; }
            final float gl = prU2[j];
            prCls[j] = classify(prY[j], prU[j], prV[j], gl / r, gl * r, gl);
            prLam[j] = clsLam;
        }
    }

    /**
     * Local light window (×/÷) for matching colours. With brightness only (grayscale
     * input or autoColor off) nothing but brightness separates line and floor, so the
     * window is narrower.
     */
    private float lightRange() {
        final float r = Math.max(1.05f, cfg.lightLocalRange);
        return (srcFormat == FMT_GRAY || modelLumaOnly) ? Math.min(r, 1.3f) : r;
    }

    /** Median of a[0…n) (partially reorders a). */
    private static float median(float[] a, int n) {
        for (int i = 1; i < n; i++) {
            final float x = a[i]; int j = i - 1;
            while (j >= 0 && a[j] > x) { a[j + 1] = a[j]; j--; }
            a[j + 1] = x;
        }
        return (n & 1) == 1 ? a[n / 2] : 0.5f * (a[n / 2 - 1] + a[n / 2]);
    }

    /** Edge position (profile offset) between samples i and i+1 where the line-ness crosses 0.5. */
    private float edgePos(int i) {
        final float a = prLam[i], b = prLam[i + 1];
        float t = 0.5f;
        if (Math.abs(b - a) > 1e-3f) t = (0.5f - a) / (b - a);
        if (t < 0f) t = 0f; else if (t > 1f) t = 1f;
        return prH0 + (i + t) * prStep;
    }

    // result of leftEdge / rightEdge: true when the edge is against floor
    private boolean edgeClean;

    /**
     * Left edge of the LINE run starting at sample a. Against floor = clean. One
     * mixed sample (partly line, partly floor: a blurred edge) between the run and
     * floor also counts as clean; the edge is placed where the line-ness crosses 0.5.
     */
    private float leftEdge(int a) {
        if (a > 0 && prCls[a - 1] == CL_FLOOR) { edgeClean = true; return edgePos(a - 1); }
        if (a > 1 && prCls[a - 1] == CL_UNK && prCls[a - 2] == CL_FLOOR && prLam[a - 1] > 0.1f && prLam[a - 1] < 0.9f) {
            edgeClean = true; return prLam[a - 1] >= 0.5f ? edgePos(a - 2) : edgePos(a - 1);
        }
        edgeClean = false;
        return prH0 + (a - 0.5f) * prStep;
    }

    /** Right edge of the LINE run ending at sample b (see leftEdge). */
    private float rightEdge(int b) {
        final int n = prN;
        if (b + 1 < n && prCls[b + 1] == CL_FLOOR) { edgeClean = true; return edgePos(b); }
        if (b + 2 < n && prCls[b + 1] == CL_UNK && prCls[b + 2] == CL_FLOOR && prLam[b + 1] > 0.1f && prLam[b + 1] < 0.9f) {
            edgeClean = true; return prLam[b + 1] >= 0.5f ? edgePos(b + 1) : edgePos(b);
        }
        edgeClean = false;
        return prH0 + (b + 0.5f) * prStep;
    }

    /**
     * Find the line in the current profile: the run of LINE samples whose width,
     * edges and position fit the line expected at offset 0 (± gate). Writes
     * msStatus (ST_MEAS / ST_UNREL / ST_ABSENT / ST_OUT), msOff, msQual, msTrim.
     *
     *   both edges against floor, width ≈ expected   → measured (quality ~1)
     *   one edge against floor, other unknown          → centre = clean edge ± width/2
     *     (glare / shadow / object over one side)        (quality 0.6)
     *   too wide but one edge where the line's edge     → that edge ± width/2 (0.55)
     *     is predicted (line merged with a blob)
     *   too wide / too thin, no edge fits                → unreliable (never a guess)
     *   no line, mostly unknown pixels near the path     → unreliable
     *   no line, clean floor where the path should be    → absent
     */
    private void analyzeProfile(float gate, float wExp) {
        final int n = prN;
        msStatus = ST_NONE; msOff = 0f; msQual = 0f; msTrim = false; msLineNear = false;
        final float tol = Math.max(0.3f * wExp, 1.5f * prStep);
        final float sig = Math.max(0.35f * wExp, 0.5f * gate);
        final float reach = gate + 0.5f * wExp;
        float best = 0f;
        boolean doubtNear = false;
        int j = 0;
        while (j < n) {
            if (prCls[j] != CL_LINE) { j++; continue; }
            final int a = j;
            int b = j, k = j + 1;
            while (k < n) {
                if (prCls[k] == CL_LINE) { b = k; k++; }
                else if (prCls[k] == CL_UNK && k + 1 < n && prCls[k + 1] == CL_LINE) k++;
                else break;
            }
            j = b + 1;
            final float eL = leftEdge(a); final boolean cl = edgeClean;
            final float eR = rightEdge(b); final boolean cr = edgeClean;
            final float width = eR - eL, ratio = width / wExp;
            final boolean near = eR >= -reach && eL <= reach;
            float c1 = 0f, q1 = 0f, c2 = 0f, q2 = 0f;
            boolean trim = false;
            if (cl && cr) {
                if (ratio >= 0.55f && ratio <= 1.6f) { c1 = 0.5f * (eL + eR); q1 = 1f - 0.4f * Math.abs(ratio - 1f); }
                else if (ratio > 1.6f) {
                    if (Math.abs(eL + 0.5f * wExp) <= tol) { c1 = eL + 0.5f * wExp; q1 = 0.55f; trim = true; }
                    if (Math.abs(eR - 0.5f * wExp) <= tol) { c2 = eR - 0.5f * wExp; q2 = 0.55f; trim = true; }
                    if (q1 == 0f && q2 == 0f && near) doubtNear = true;
                } else if (near) doubtNear = true;                 // much thinner than the line
            } else if (cl || cr) {
                // One edge against floor, the other against unknown pixels (a glare /
                // shadow / object over one side): the clean edge plus half the width.
                final float c = cl ? eL + 0.5f * wExp : eR - 0.5f * wExp;
                if (ratio >= 0.3f && ratio <= 1.6f) { c1 = c; q1 = 0.6f; }
                // Wider than the line: the line merged with something of its colour, or the
                // border of a large dark/bright area. Only an edge where the line's edge is
                // predicted is trusted.
                else if (Math.abs(c) <= Math.max(tol, 0.5f * wExp)) { c1 = c; q1 = 0.45f; trim = true; }
                else if (near) doubtNear = true;
            } else {
                // Unknown pixels on both sides: only a full-width run is trusted (a
                // partly hidden line would put its centre in the wrong place).
                if (ratio >= 0.8f && ratio <= 1.25f) { c1 = 0.5f * (eL + eR); q1 = 0.35f; }
                else if (near) doubtNear = true;
            }
            // weaker evidence (one clean edge, or none) must also sit closer to the prediction
            final float g1 = q1 >= 0.7f ? gate : Math.max(tol, 0.5f * gate);
            if (q1 > 0f && Math.abs(c1) <= g1) {
                final float z = c1 / sig, s = q1 * (float) Math.exp(-0.5f * z * z);
                if (s > best) { best = s; msOff = c1; msQual = q1; msTrim = trim; }
            } else if (q1 > 0f && near) { doubtNear = true; msLineNear = true; }
            if (q2 > 0f && Math.abs(c2) <= gate) {
                final float z = c2 / sig, s = q2 * (float) Math.exp(-0.5f * z * z);
                if (s > best) { best = s; msOff = c2; msQual = q2; msTrim = trim; }
            }
        }
        if (best > 0f) {
            msStatus = ST_MEAS;
            if (msTrim) v3Rejected++;
            else if (msQual >= 0.8f) {
                lightRef += 0.02f * (prGain - lightRef);   // typical light level
                ambRef = Math.max(0.5f, Math.min(2.5f, ambRef + 0.01f * (prGain - ambRef)));
            }
            return;
        }
        int nW = 0, nOut = 0, nF = 0, nU = 0;
        for (int i = 0; i < n; i++) {
            final float s = prH0 + i * prStep;
            if (s < -reach || s > reach) continue;
            nW++;
            final byte c = prCls[i];
            if (c == CL_OUT) nOut++; else if (c == CL_FLOOR) nF++; else nU++;
        }
        if (nW == 0 || nOut * 2 > nW) msStatus = ST_OUT;
        else if (doubtNear || nU * 100 >= 15 * nW) msStatus = ST_UNREL;
        else if (nF * 10 >= 7 * nW) {
            // Clean floor where the line should be — but only under normal light. Much
            // brighter (glare) or darker (deep shadow) than usual can hide a line in a
            // floor-like colour, so that is unreliable, not absent.
            final float rel = prGain / Math.max(0.05f, lightRef);
            msStatus = rel >= 0.65f && rel <= 1.4f ? ST_ABSENT : ST_UNREL;
        }
        else msStatus = ST_UNREL;
        if (doubtNear) v3Rejected++;
    }

    /** Measure the line at plane point (px,py), normal (nx,ny), within ±gate. */
    private void measureAt(float px, float py, float nx, float ny, float gate, float wExp) {
        sampleProfile(px, py, nx, ny, gate + 1.6f * wExp, wExp);
        analyzeProfile(gate, wExp);
    }

    // =========================================================================
    //  v3 — calibration
    // =========================================================================
    //
    //  Phase 1 (colours, Config.colorCalibFrames frames): pixels are sampled over a
    //  tall region above the robot (not only a strip), clustered (k-means, k = 2…4)
    //  and the line is the cluster forming a narrow band in most row blocks; the
    //  floor is the cluster filling both image edges. Further clusters are either
    //  a second floor colour (they also fill the edges) or a DISTURBANCE (glare,
    //  shadow patch, object) — excluded from both colour models.
    //  Phase 2 (geometry, Config.calibFrames frames): rows are scanned from the
    //  robot up to the top of the image; in each row the line is located with the
    //  colour model. Rows where the line has clean edges give the line width per
    //  row (→ perspective / camera tilt) and the line's position (→ straight-ahead
    //  and heading zero). Rows where the floor stops (table edge, background) set
    //  the top of the ROI. Disturbed rows (no clean edges) are simply not used.

    private boolean calibrateV3() {
        if (calibrated) return true;
        laPathCount = 0;
        boolean lineFound = false;
        int centroidX = -1;
        final boolean learnColors = cfg.autoColor && !colorReady;
        if (learnColors) {
            sampleRegionColors();
            if (++colorFramesSampled >= Math.max(1, cfg.colorCalibFrames)) learnColorModelV3();
        } else {
            if (!modelReady) buildFallbackModels();
            if (modelReady) {
                centroidX = calibGeometryFrame();
                lineFound = centroidX >= 0;
                if (lineFound) {
                    calibIssue = ISSUE_NONE;
                    if (++calibCount >= Math.max(1, cfg.calibFrames)) finishGeometry();
                } else calibIssue = ISSUE_NO_LINE;
            }
        }
        fillCalibPath();
        fireCallback(0, 100, lineFound, centroidX);
        return calibrated;
    }

    /** Rows of the colour-learning region: from the robot body up 45 % of the frame. */
    private int calibRegionTop() { return Math.max(0, ignoreTop - Math.round(0.45f * imgH)); }

    /** Grid-sample up to SAMPLES_PER_FRAME (Y,U,V,column,row) tuples from the region above the robot. */
    private void sampleRegionColors() {
        final int W = imgW, r0 = calibRegionTop(), r1 = Math.max(r0 + 1, ignoreTop);
        final int nRows = Math.min(24, r1 - r0);
        final float stepR = (r1 - r0) / (float) nRows;
        final int stepC = Math.max(1, (W * nRows + SAMPLES_PER_FRAME - 1) / SAMPLES_PER_FRAME);
        final int cap = sampY.length - STRIPE_EXTRA;
        for (int k = 0; k < nRows && sampleCount < cap; k++) {
            final int row = Math.min(r1 - 1, r0 + Math.round((k + 0.5f) * stepR));
            for (int col = 0; col < W && sampleCount < cap; col += stepC) {
                final int i = row * W + col;
                int y, u, v;
                if (srcFormat == FMT_GRAY) { y = srcY[i] & 0xFF; u = 128; v = 128; }
                else if (srcFormat == FMT_YUV420) {
                    final int uvi = (row >> 1) * uvW + (col >> 1);
                    y = srcY[i] & 0xFF; u = srcU[uvi] & 0xFF; v = srcV[uvi] & 0xFF;
                } else { final int c = srcArgb[i]; y = lumaOf(c); u = chromaUOf(c); v = chromaVOf(c); }
                sampY[sampleCount] = (short) y; sampU[sampleCount] = (short) u; sampV[sampleCount] = (short) v;
                sampCol[sampleCount] = (short) col; sampRow[sampleCount] = (short) row;
                sampleCount++;
            }
        }
    }

    /** Colour learning for v3 (runs once per calibration — allocation is fine here). */
    private void learnColorModelV3() {
        final int n = sampleCount;
        sampleCount = 0;
        colorFramesSampled = 0;
        if (n < 32) { calibIssue = ISSUE_LOW_CONTRAST; return; }
        int first = ISSUE_NONE;
        // First choice: the line found as a stripe (see detectStripe) with the floor of
        // a clustering; only if no clustering yields one, the line is chosen among the
        // clusters by its shape alone.
        final int passes = 2;
        for (int pass = 0; pass < passes; pass++) {
            for (int k = 2; k <= MAX_CLUSTERS; k++) {
                if (tryColorModelV3(n, k, pass == 0)) { calibIssue = ISSUE_NONE; return; }
                if (k == 2) first = calibIssue;
            }
        }
        colorReady = false;
        calibIssue = first;
    }

    // ── the line as a stripe (calibration only; allocation is fine here) ──
    private double stY, stU, stV;                     // stripe colour (U, V centred on 0)
    private final double[] stCov = new double[6];
    private float  stCol, stW;                        // centre / width (px) in the stripe's lowest row
    // the stripe's pooled runs (pixel row, interior columns) and the sample count after adoptStripe
    private final int[] stRunRow = new int[40], stRunX0 = new int[40], stRunX1 = new int[40];
    private int    stRunN, adoptN;

    /**
     * The line at calibration, found by what makes it a line rather than by its
     * colour: the robot stands on a straight section, so the line is a STRIPE —
     * one colour, a constant width on the floor, running straight ahead over many
     * rows, with SHARP edges against the floor on both sides. Light effects (glare,
     * shadows, uneven light) have soft edges; blobs (a patch, an object, glare) do not
     * keep one floor width over many rows. Rows are scanned with the floor model of
     * cluster {@code floorCl} (any brightness ×0.3…2.5 of it counts as floor); every
     * non-floor run with floor and sharp edges on both sides is a candidate; the
     * stripe is the candidate agreeing (width, position, colour) with runs in the
     * most rows. Its interior pixels give the line colour (stY/U/V, stCov).
     */
    private boolean detectStripe(int n, int floorCl) {
        // Floor colour from the floor cluster's samples in the outer 30 % on each side
        // (the robot stands on the line: it is not there), then robustly trimmed (3σ)
        // — a cluster may also hold some line pixels, which would make the line "floor".
        final float[] fi = new float[6], fa = new float[3];
        double my = 0, mu = 0, mv = 0;
        final double[] cov = new double[6];
        final int c0 = Math.round(0.3f * imgW), c1 = imgW - c0;
        for (int it = 0; it < 3; it++) {
            double sy = 0, su = 0, sv = 0; int cnt = 0;
            final double[] cv = new double[6];
            for (int pass = 0; pass < 2; pass++) {
                for (int i = 0; i < n; i++) {
                    if (sampLabel[i] != floorCl || (sampCol[i] >= c0 && sampCol[i] < c1)) continue;
                    final float y = sampY[i], u = sampU[i] - 128, v = sampV[i] - 128;
                    if (it > 0) {
                        final float dy = y - (float) my, du = u - (float) mu, dv = v - (float) mv;
                        final float d = dy * (fi[0] * dy + 2f * (fi[1] * du + fi[2] * dv)) + du * (fi[3] * du + 2f * fi[4] * dv) + fi[5] * dv * dv;
                        if (d > 9f) continue;
                    }
                    if (pass == 0) { sy += y; su += u; sv += v; cnt++; }
                    else {
                        final double dy = y - sy, du = u - su, dv = v - sv;
                        cv[0] += dy * dy; cv[1] += dy * du; cv[2] += dy * dv; cv[3] += du * du; cv[4] += du * dv; cv[5] += dv * dv;
                    }
                }
                if (pass == 0) { if (cnt < 16) return false; sy /= cnt; su /= cnt; sv /= cnt; }
            }
            for (int j = 0; j < 6; j++) cov[j] = cv[j] / cnt;
            cov[0] += 6.0; cov[3] += 3.0; cov[5] += 3.0;
            my = sy; mu = su; mv = sv;
            if (!invert3(cov, fi, 0)) return false;
        }
        final float fmY = (float) my, fmU = (float) mu, fmV = (float) mv;
        fa[0] = fi[0] * fmY + fi[1] * fmU + fi[2] * fmV;
        fa[1] = fi[1] * fmY + fi[3] * fmU + fi[4] * fmV;
        fa[2] = fi[2] * fmY + fi[4] * fmU + fi[5] * fmV;
        final float fk = fa[0] * fmY + fa[1] * fmU + fa[2] * fmV;
        if (fk < 1e-6f) return false;
        final float tau = cfg.colorMatchSigma * cfg.colorMatchSigma;

        setGeometry(cfg.cameraTiltDeg);
        final int W = imgW, NR = 40, step = Math.max(2, W / 240), ns = (W - 2) / step;
        final int rBot = Math.max(1, ignoreTop - 2), rTop = Math.max(0, ignoreTop - Math.round(0.6f * imgH));
        final float[] sy = new float[ns], su = new float[ns], sv = new float[ns];
        final boolean[] fl = new boolean[ns];
        final float[] gainBuf = new float[ns];
        final int MAXR = NR * 8;
        final float[] rC = new float[MAXR], rWpx = new float[MAXR];
        final float[] rMY = new float[MAXR], rMU = new float[MAXR], rMV = new float[MAXR], rCon = new float[MAXR];
        final int[] rRow = new int[MAXR], rRowPx = new int[MAXR], rX0 = new int[MAXR], rX1 = new int[MAXR];
        final double[] rS = new double[MAXR * 10];
        int nr = 0, rows = 0;
        for (int kr = 0; kr < NR; kr++) {
            final int row = rBot - Math.round(kr * (rBot - rTop) / (float) (NR - 1));
            final double yn = (row + 0.5 - gCy) / gFoc, down = yn * gCosT + gSinT;
            if (down < 0.02) continue;

            rows++;
            // the light level of this row: median gain of the floor-like samples on the outer sides
            int ng = 0;
            for (int j = 0; j < ns; j++) {
                readPixel(j * step, row);
                sy[j] = sY; su[j] = sU; sv[j] = sV;
                final int col = j * step;
                if (col >= c0 && col < c1) continue;
                final float q1 = fa[0] * sY + fa[1] * sU + fa[2] * sV, g = q1 / fk;
                if (g > 0.2f && g < 3f && rayDist(sY, sU, sV, fi, 0, fa, 0, fk, g, g) <= 2f * tau) gainBuf[ng++] = g;
            }
            final float gl = ng >= 4 ? median(gainBuf, ng) : 1f, lr = lightRange();
            for (int j = 0; j < ns; j++) fl[j] = rayDist(sy[j], su[j], sv[j], fi, 0, fa, 0, fk, gl / lr, gl * lr) <= tau;
            int j = 0;
            while (j < ns && nr < MAXR) {
                if (fl[j]) { j++; continue; }
                final int a = j;
                while (j < ns && !fl[j]) j++;
                final int b = j - 1, len = b - a + 1;
                if (a < 5 || b > ns - 6 || len < 3) continue;
                // interior: the middle 60 % of the run
                final int i0 = a + len / 5, i1 = b - len / 5;
                double ty = 0, tu = 0, tv = 0; int tn = 0;
                for (int i = i0; i <= i1; i++) { ty += sy[i]; tu += su[i]; tv += sv[i]; tn++; }
                ty /= tn; tu /= tn; tv /= tn;
                if (!sharpEdge(sy, su, sv, a, -1, ty, tu, tv) || !sharpEdge(sy, su, sv, b, 1, ty, tu, tv)) continue;
                final float cpx = (a + b + 1) * 0.5f * step;
                rRow[nr] = kr;
                rC[nr] = cpx; rWpx[nr] = len * step;
                rMY[nr] = (float) ty; rMU[nr] = (float) tu; rMV[nr] = (float) tv;
                rRowPx[nr] = row; rX0[nr] = i0 * step; rX1[nr] = i1 * step;
                {   // contrast against the floor beside it, relative to the floor's brightness
                    final float fy = 0.5f * (sy[a - 4] + sy[b + 4]), fu = 0.5f * (su[a - 4] + su[b + 4]), fv = 0.5f * (sv[a - 4] + sv[b + 4]);
                    final float dy = (float) ty - fy, du = (float) tu - fu, dv = (float) tv - fv;
                    rCon[nr] = (float) Math.sqrt(dy * dy + du * du + dv * dv) / (Math.abs(fy) + 20f);
                }
                final int o = nr * 10;
                for (int i = i0; i <= i1; i++) {
                    final double y = sy[i], u = su[i], v = sv[i];
                    rS[o] += 1; rS[o + 1] += y; rS[o + 2] += u; rS[o + 3] += v;
                    rS[o + 4] += y * y; rS[o + 5] += y * u; rS[o + 6] += y * v; rS[o + 7] += u * u; rS[o + 8] += u * v; rS[o + 9] += v * v;
                }
                nr++;
            }
        }
        if (nr == 0 || rows == 0) return false;
        // The stripe: from every run as a seed, follow the line row by row up and down
        // (in the image, so nothing depends on the configured camera tilt): each next
        // run continues the line — about the width of the previous one, where the line
        // leads, of its colour. Rows without such a run (hidden by a disturbance) are
        // skipped with growing tolerance. The longest chain is the line.
        final int[] chain = new int[NR], bestChain = new int[NR];
        int bestS = 0, best = -1;
        for (int i = 0; i < nr; i++) {
            final int s = stripeChain(i, nr, rRow, rC, rWpx, rMY, rMU, rMV, fmY, fmU, fmV, NR, chain);
            if (s > bestS || (s == bestS && best >= 0 && Math.abs(rC[i] - gCx) < Math.abs(rC[best] - gCx))) {
                bestS = s; best = i; System.arraycopy(chain, 0, bestChain, 0, s);
            }
        }
        if (System.getProperty("dbgs") != null) System.err.println("stripe: rows=" + rows + " runs=" + nr + " best=" + bestS);
        if (best < 0 || bestS < 6 || bestS * 100 < 15 * rows) return false;
        final int[] pickRow = new int[NR];
        final float[] pY = new float[NR], pU = new float[NR], pV = new float[NR];
        int np = 0;
        for (int c = 0; c < bestS; c++) {
            final int pick = bestChain[c];
            pickRow[np] = pick; pY[np] = rMY[pick]; pU[np] = rMU[pick]; pV[np] = rMV[pick]; np++;
        }
        // The line's colour: from the runs with the HIGHEST CONTRAST against the floor
        // beside them (the top 40 %). Glare, haze, a halo or a cast over part of the
        // stripe always bring line and floor closer together, so those rows are left out.
        final double[] S = new double[10];
        int lowRow = Integer.MAX_VALUE;
        {
            stRunN = 0;
            final float[] con = new float[np];
            for (int p = 0; p < np; p++) con[p] = rCon[pickRow[p]];
            final float[] srt = java.util.Arrays.copyOf(con, np);
            java.util.Arrays.sort(srt);
            final float thr = srt[Math.min(np - 1, Math.max(0, (int) Math.floor(0.6f * np)))];
            for (int p = 0; p < np; p++) {
                if (con[p] < thr) continue;
                final int pick = pickRow[p];
                for (int t = 0; t < 10; t++) S[t] += rS[pick * 10 + t];
                if (rRow[pick] < lowRow) { lowRow = rRow[pick]; stCol = rC[pick]; stW = rWpx[pick]; }
                if (stRunN < stRunRow.length) {
                    stRunRow[stRunN] = rRowPx[pick]; stRunX0[stRunN] = rX0[pick]; stRunX1[stRunN] = rX1[pick]; stRunN++;
                }
            }
            // position / width for the geometry phase: the lowest run of the whole chain
            for (int p = 0; p < np; p++) {
                final int pick = pickRow[p];
                if (rRow[pick] < lowRow) { lowRow = rRow[pick]; stCol = rC[pick]; stW = rWpx[pick]; }
            }
        }
        if (S[0] < 8) return false;
        stY = S[1] / S[0]; stU = S[2] / S[0]; stV = S[3] / S[0];
        stCov[0] = S[4] / S[0] - stY * stY; stCov[1] = S[5] / S[0] - stY * stU; stCov[2] = S[6] / S[0] - stY * stV;
        stCov[3] = S[7] / S[0] - stU * stU; stCov[4] = S[8] / S[0] - stU * stV; stCov[5] = S[9] / S[0] - stV * stV;
        stCov[0] += 6.0; stCov[3] += 3.0; stCov[5] += 3.0;
        return true;
    }

    /**
     * Chain of runs through seed run {@code seed}: upward and downward, row by row,
     * the run that continues the line (position from the chain's direction, width
     * within 30 % of the previous run's plus 3 % per skipped row, colour within 35 %
     * of the line–floor contrast of the seed). Writes the run indices to {@code out};
     * returns their number. A gap of more than NR/3 rows ends the chain.
     */
    private static int stripeChain(int seed, int nr, int[] row, float[] c, float[] w, float[] mY, float[] mU, float[] mV,
                                   float fY, float fU, float fV, int NR, int[] out) {
        final float cY = mY[seed] - fY, cU = mU[seed] - fU, cV = mV[seed] - fV;
        final float lim = Math.max(20f, 0.35f * (float) Math.sqrt(cY * cY + cU * cU + cV * cV));
        int n = 0;
        out[n++] = seed;
        for (int dir = -1; dir <= 1; dir += 2) {
            int last = seed, prev = -1;
            for (int kr = row[seed] + dir; kr >= 0 && kr < NR; kr += dir) {
                final int gap = Math.abs(kr - row[last]);
                if (gap > NR / 3) break;
                // predicted centre: continue the direction of the last two chain runs
                float pc = c[last];
                if (prev >= 0) pc += (c[last] - c[prev]) / (row[last] - row[prev]) * (kr - row[last]);
                final float tolC = w[last] * (0.6f + 0.1f * gap), tolW = 0.3f + 0.03f * gap;
                int pick = -1; float pd = Float.MAX_VALUE;
                for (int q = 0; q < nr; q++) {
                    if (row[q] != kr) continue;
                    final float d = Math.abs(c[q] - pc);
                    if (d > tolC || Math.abs(w[q] / w[last] - 1f) > tolW) continue;
                    final float dY = mY[q] - mY[seed], dU = mU[q] - mU[seed], dV = mV[q] - mV[seed];
                    if (dY * dY + dU * dU + dV * dV > lim * lim) continue;
                    if (d < pd) { pd = d; pick = q; }
                }
                if (pick < 0) continue;
                out[n++] = pick; prev = last; last = pick;
            }
        }
        return n;
    }

    /**
     * Sharp edge at run end {@code e} (dir −1 = left edge, +1 = right edge): from the
     * floor 4 samples outside to 2 samples inside, at most two samples lie between
     * the floor colour and the run's interior colour (a taped / painted edge, a few
     * pixels of blur) — a light effect fades over many samples.
     */
    private boolean sharpEdge(float[] y, float[] u, float[] v, int e, int dir, double mY, double mU, double mV) {
        final int f = e + 4 * dir;
        if (f < 0 || f >= y.length) return false;
        final double fY = y[f], fU = u[f], fV = v[f];
        final double dY = mY - fY, dU = mU - fU, dV = mV - fV, dd = dY * dY + dU * dU + dV * dV;
        if (dd < 100) return false;                                   // hardly different from the floor
        int mid = 0;
        for (int k = -3; k <= 2; k++) {
            final int i = e - k * dir;                                // outside (k < 0) … inside (k > 0)
            if (i < 0 || i >= y.length) continue;
            final double t = ((y[i] - fY) * dY + (u[i] - fU) * dU + (v[i] - fV) * dV) / dd;
            if (t > 0.2 && t < 0.8) mid++;
        }
        return mid <= 2;
    }

    /**
     * The stripe's colour becomes the line cluster: samples close to it (4σ) are
     * relabelled to a new cluster k; all cluster means and counts are recomputed.
     * Returns the new cluster count, or −1 if too few samples match.
     */
    private int adoptStripe(int n, int k, double[] c2, int[] count2, int floorCl) {
        final float[] si = new float[6];
        if (!invert3(stCov, si, 0)) return -1;
        int m = 0;
        for (int i = 0; i < n; i++) {
            final float y = sampY[i] - (float) stY, u = sampU[i] - 128 - (float) stU, v = sampV[i] - 128 - (float) stV;
            final float d = y * (si[0] * y + 2f * (si[1] * u + si[2] * v)) + u * (si[3] * u + 2f * si[4] * v) + si[5] * v * v;
            if (d <= 16f) { sampLabel[i] = (byte) k; m++; }
        }
        // the stripe's own pixels (they may be missing from the samples, which come from
        // the region near the robot — a disturbance there greys the line)
        final int cap = sampY.length;
        // (only when the samples hold few line pixels — in a clean scene they hold plenty)
        for (int r = 0; r < stRunN && n < cap && m < 64; r++) {
            for (int col = stRunX0[r]; col <= stRunX1[r] && n < cap; col += 2) {
                readPixel(col, stRunRow[r]);
                sampY[n] = (short) Math.round(sY); sampU[n] = (short) Math.round(sU + 128f); sampV[n] = (short) Math.round(sV + 128f);
                sampCol[n] = (short) col; sampRow[n] = (short) stRunRow[r]; sampLabel[n] = (byte) k;
                n++; m++;
            }
        }
        adoptN = n;
        if (m < 8) return -1;
        final int k2 = k + 1;
        final double[] sum = new double[k2 * 3];
        java.util.Arrays.fill(count2, 0);
        for (int i = 0; i < n; i++) {
            final int l = sampLabel[i];
            count2[l]++; sum[l * 3] += sampY[i]; sum[l * 3 + 1] += sampU[i]; sum[l * 3 + 2] += sampV[i];
        }
        for (int q = 0; q < k2; q++) {
            final int src = count2[q] > 0 ? q : floorCl;               // an emptied cluster: a copy of the floor
            final double cc = Math.max(1, count2[src]);
            c2[q * 3] = sum[src * 3] / cc; c2[q * 3 + 1] = sum[src * 3 + 1] / cc; c2[q * 3 + 2] = sum[src * 3 + 2] / cc;
        }
        return k2;
    }

    private boolean tryColorModelV3(int n, int k, boolean stripe) {
        final double[] c = new double[k * 3];
        if (!kMeans(n, k, c)) { calibIssue = ISSUE_LOW_CONTRAST; return false; }
        final int edge = Math.max(1, imgW / 20);
        final int[] left = new int[k], right = new int[k], count = new int[k];
        int leftTotal = 0, rightTotal = 0;
        for (int i = 0; i < n; i++) {
            final int col = sampCol[i], lbl = sampLabel[i];
            count[lbl]++;
            if (col < edge) { left[lbl]++; leftTotal++; }
            else if (col >= imgW - edge) { right[lbl]++; rightTotal++; }
        }
        // Two colours that are not really different (noise on one surface): low contrast.
        if (k == 2) {
            final double dd = Math.sqrt(sq(c[0] - c[3]) + sq(c[1] - c[4]) + sq(c[2] - c[5]));
            if (dd < cfg.colorMinDistance) { calibIssue = ISSUE_LOW_CONTRAST; return false; }
        }
        // Floor: the cluster holding the most of both edge zones (and ≥ 40 % of each).
        int floor = -1; double bestEdge = 0;
        for (int j = 0; j < k; j++) {
            final double l = left[j] / (double) Math.max(1, leftTotal), r = right[j] / (double) Math.max(1, rightTotal);
            if (l >= 0.4 && r >= 0.4 && l + r > bestEdge) { bestEdge = l + r; floor = j; }
        }
        if (floor < 0 && k > 2) {
            // A shadow or a light change splits the floor (one part on each edge): the
            // floor is the cluster with the largest share of both edges together (the
            // other part is the same floor under other light — see buildModelsV3).
            for (int j = 0; j < k; j++) {
                final double l = left[j] / (double) Math.max(1, leftTotal), r = right[j] / (double) Math.max(1, rightTotal);
                if (l + r >= 0.6 && l + r > bestEdge) { bestEdge = l + r; floor = j; }
            }
        }
        if (floor < 0) { calibIssue = ISSUE_NO_FLOOR; return false; }
        if (stripe) {
            if (!detectStripe(n, floor)) { calibIssue = ISSUE_NO_LINE; return false; }
            final int k2 = k + 1;
            final double[] c2 = new double[k2 * 3];
            final int[] count2 = new int[k2];
            if (adoptStripe(n, k, c2, count2, floor) < 0) { calibIssue = ISSUE_NO_LINE; return false; }
            n = adoptN;
            final int[] left2 = new int[k2], right2 = new int[k2];
            for (int i = 0; i < n; i++) {
                final int col = sampCol[i], lbl = sampLabel[i];
                if (col < edge) left2[lbl]++; else if (col >= imgW - edge) right2[lbl]++;
            }
            if (left2[k] > 0.4 * leftTotal || right2[k] > 0.4 * rightTotal) { calibIssue = ISSUE_NO_LINE; return false; }
            if (!fitDiscriminant(n, k2, c2, count2, k, false)) return false;
            if (!buildModelsV3(n, k2, c2, k, floor, left2, right2, leftTotal, rightTotal)) {
                colorReady = false; colorSeparation = 0f; lineColorArgb = 0; floorColorArgb = 0;
                calibIssue = ISSUE_LOW_CONTRAST;
                return false;
            }
            // The line model is the stripe's own colour (its highest-contrast rows) with a
            // wider spread for blurred edges: the samples gathered near the robot may be
            // greyed by a disturbance there and must not shift it.
            {
                final double[] cv = stCov.clone();
                for (int j = 0; j < 6; j++) cv[j] *= 2.0;
                cv[0] += 12.0; cv[3] += 4.0; cv[5] += 4.0;
                final float[] keepM = cmL.clone(), keepI = ciL.clone(), keepA = caL.clone();
                final float keepK = ckL;
                if (STRIPE_LINE_MODEL && setModel(true, 0, stY, stU, stV, cv)) {
                    finishModels();
                    final float r = lightRange();
                    boolean okM = classify(cmL[0], cmL[1], cmL[2], 1f / r, r, 1f) == CL_LINE;
                    for (int j = 0; j < nFloorModels && okM; j++) okM = classify(cmF[j * 3], cmF[j * 3 + 1], cmF[j * 3 + 2], 1f / r, r, 1f) == CL_FLOOR;
                    if (okM) lineColorArgb = yuvToArgb(stY, stU + 128, stV + 128);
                    else {   // keep the cluster model
                        System.arraycopy(keepM, 0, cmL, 0, 3); System.arraycopy(keepI, 0, ciL, 0, 6); System.arraycopy(keepA, 0, caL, 0, 3);
                        ckL = keepK; finishModels();
                    }
                }
            }
            learnedLineX = stCol;
            learnedLineW = stW;
            return true;
        }
        // Line: a cluster that (almost) never reaches the edges and forms a narrow
        // band in most row blocks (blocks where a disturbance merges with it are outvoted).
        // The line crosses the whole sampled height: of the candidates, the one that is a
        // narrow band in the most row blocks (a blob — glare, a patch, an object — fills
        // only some), then the one nearest straight ahead.
        int line = -1; double bestD = Double.MAX_VALUE, bestCov = 0;
        final double[] band = new double[3], b = new double[3];
        for (int j = 0; j < k; j++) {
            if (j == floor || count[j] < n / 200) continue;
            // (an object of the line's colour may touch an edge in a few rows: the
            // per-block band test decides, those blocks are outvoted)
            if (left[j] > 0.4 * leftTotal || right[j] > 0.4 * rightTotal) continue;
            final double cov = bandBlocks(n, j, b);
            if (cov < 0.5 || b[1] > (k == 2 ? 0.8 : 0.5) * imgW) continue;
            final double d = Math.abs(b[0] - imgW * 0.5);
            if (cov > bestCov + 0.1 || (cov > bestCov - 0.1 && d < bestD)) {
                bestCov = Math.max(cov, bestCov); bestD = d; line = j; band[0] = b[0]; band[1] = b[1];
            }
        }
        if (line < 0) { calibIssue = ISSUE_NO_FLOOR; return false; }
        if (!fitDiscriminant(n, k, c, count, line, false)) return false;   // v2 projection + reported colours
        if (!buildModelsV3(n, k, c, line, floor, left, right, leftTotal, rightTotal)) {
            colorReady = false; colorSeparation = 0f; lineColorArgb = 0; floorColorArgb = 0;
            calibIssue = ISSUE_LOW_CONTRAST;
            return false;
        }
        learnedLineX = (float) band[0];
        learnedLineW = (float) band[1];
        return true;
    }

    /**
     * Split the samples into 6 row blocks; in each, find the band (run of columns
     * where cluster {@code cl} is the majority) holding most of the cluster's
     * samples. Returns the fraction of ALL row blocks whose band holds ≥ 60 % of the
     * cluster's samples there (a line crosses every block; a blob of another colour
     * only some); out = {centre, width} of the lowest such band.
     */
    private double bandBlocks(int n, int cl, double[] out) {
        final int NB = 6, W = imgW;
        int rMin = Integer.MAX_VALUE, rMax = -1;
        for (int i = 0; i < n; i++) { rMin = Math.min(rMin, sampRow[i]); rMax = Math.max(rMax, sampRow[i]); }
        if (rMax < rMin) return 0;
        final float span = Math.max(1, rMax - rMin + 1) / (float) NB;
        final int[][] total = new int[NB][W], in = new int[NB][W];
        final int[] clTotal = new int[NB];
        for (int i = 0; i < n; i++) {
            final int blk = Math.min(NB - 1, (int) ((sampRow[i] - rMin) / span));
            total[blk][sampCol[i]]++;
            if (sampLabel[i] == cl) { in[blk][sampCol[i]]++; clTotal[blk]++; }
        }
        final int gapTol = runGapTol();
        int used = 0, ok = 0;
        out[0] = W * 0.5; out[1] = 0;
        for (int blk = NB - 1; blk >= 0; blk--) {      // bottom block first
            used++;
            if (clTotal[blk] < 8) continue;
            int bestIn = 0, bestS = -1, bestE = -1, curIn = 0, curS = -1, curLast = -1;
            for (int col = 0; col <= W; col++) {
                final boolean sampled = col < W && total[blk][col] > 0;
                final boolean member = sampled && in[blk][col] * 2 > total[blk][col];
                if (member) { if (curS < 0) { curS = col; curIn = 0; } curIn += in[blk][col]; curLast = col; }
                else if (curS >= 0 && (col == W || (sampled && col - curLast > gapTol))) {
                    if (curIn > bestIn) { bestIn = curIn; bestS = curS; bestE = curLast + 1; }
                    curS = -1;
                }
            }
            if (bestS >= 0 && bestIn >= 0.6 * clTotal[blk] && bestE - bestS <= 0.8 * W) {
                if (ok == 0) { out[0] = (bestS + bestE) * 0.5; out[1] = bestE - bestS; }
                ok++;
            }
        }
        return used == 0 ? 0 : ok / (double) used;
    }

    /** Build the v3 line / floor colour models from the clustered samples. */
    private boolean buildModelsV3(int n, int k, double[] c, int line, int floor,
                                  int[] left, int[] right, int leftTotal, int rightTotal) {
        modelLumaOnly = false;
        if (!modelFromCluster(n, line, true, 0)) return false;
        if (!floorModelRobust(n, floor) && !modelFromCluster(n, floor, false, 0)) return false;
        nFloorModels = 1;
        finishModels();
        // Other clusters: a second floor colour if it also fills both edges and is
        // not explained by the floor / line colour under other light; otherwise a
        // disturbance (glare, shadow patch, object) — excluded from the models.
        final float r = lightRange();
        for (int j = 0; j < k && nFloorModels < FLOOR_MAX; j++) {
            if (j == line || j == floor) continue;
            final float y = (float) c[j * 3], u = (float) c[j * 3 + 1] - 128f, v = (float) c[j * 3 + 2] - 128f;
            final byte cls = classify(y, u, v, 1f / (r * r), r * r, 1f);
            if (cls != CL_UNK) continue;                                   // shadowed / lit floor or line
            final double l = left[j] / (double) Math.max(1, leftTotal), rr = right[j] / (double) Math.max(1, rightTotal);
            if (l >= 0.15 && rr >= 0.15 && modelFromCluster(n, j, false, nFloorModels)) nFloorModels++;
        }
        // The two colours must stay apart even under a local light change.
        if (classify(cmL[0], cmL[1], cmL[2], 1f / r, r, 1f) != CL_LINE) return false;
        for (int j = 0; j < nFloorModels; j++) {
            if (classify(cmF[j * 3], cmF[j * 3 + 1], cmF[j * 3 + 2], 1f / r, r, 1f) != CL_FLOOR) return false;
        }
        return true;
    }

    /**
     * The main floor model, robust: the floor cluster's samples trimmed at 3σ
     * twice. A disturbance near the robot at calibration (a glare halo: lighter,
     * greyer floor) otherwise widens the floor colour towards the line's, and the line
     * then fits both colours (= unreliable) everywhere. False if too few samples.
     */
    private boolean floorModelRobust(int n, int cl) {
        double my = 0, mu = 0, mv = 0;
        final double[] cov = new double[6];
        final float[] inv = new float[6];
        int cnt = 0, total = 0;
        for (int it = 0; it < 3; it++) {
            double sy = 0, su = 0, sv = 0; cnt = 0;
            final double[] cv = new double[6];
            for (int pass = 0; pass < 2; pass++) {
                for (int i = 0; i < n; i++) {
                    if (sampLabel[i] != cl) continue;
                    final float y = sampY[i], u = sampU[i] - 128, v = sampV[i] - 128;
                    if (it > 0) {
                        final float dy = y - (float) my, du = u - (float) mu, dv = v - (float) mv;
                        final float d = dy * (inv[0] * dy + 2f * (inv[1] * du + inv[2] * dv)) + du * (inv[3] * du + 2f * inv[4] * dv) + inv[5] * dv * dv;
                        if (d > 9f) continue;
                    }
                    if (pass == 0) { sy += y; su += u; sv += v; cnt++; }
                    else {
                        final double dy = y - sy, du = u - su, dv = v - sv;
                        cv[0] += dy * dy; cv[1] += dy * du; cv[2] += dy * dv; cv[3] += du * du; cv[4] += du * dv; cv[5] += dv * dv;
                    }
                }
                if (pass == 0) { if (cnt < 32) return false; sy /= cnt; su /= cnt; sv /= cnt; }
            }
            if (it == 0) total = cnt;
            for (int j = 0; j < 6; j++) cov[j] = cv[j] / cnt;
            cov[0] += 6.0; cov[3] += 3.0; cov[5] += 3.0;
            my = sy; mu = su; mv = sv;
            if (!invert3(cov, inv, 0)) return false;
        }
        // only a cluster that really holds outliers (≥ 3 % trimmed, e.g. a glare halo)
        // gets the trimmed model; otherwise the plain cluster model is used
        if (cnt > 0.97 * total) return false;
        // (the trimming cuts the tails of the distribution: widen it back a little)
        for (int j = 0; j < 6; j++) cov[j] *= 1.5;
        return setModel(false, 0, my, mu, mv, cov);
    }

    /** Mean and covariance of cluster {@code cl} → model. */
    private boolean modelFromCluster(int n, int cl, boolean line, int slot) {
        double sy = 0, su = 0, sv = 0; int cnt = 0;
        for (int i = 0; i < n; i++) {
            if (sampLabel[i] != cl) continue;
            sy += sampY[i]; su += sampU[i] - 128; sv += sampV[i] - 128; cnt++;
        }
        if (cnt < 4) return false;
        final double my = sy / cnt, mu = su / cnt, mv = sv / cnt;
        final double[] cov = new double[6];
        for (int i = 0; i < n; i++) {
            if (sampLabel[i] != cl) continue;
            final double dy = sampY[i] - my, du = sampU[i] - 128 - mu, dv = sampV[i] - 128 - mv;
            cov[0] += dy * dy; cov[1] += dy * du; cov[2] += dy * dv; cov[3] += du * du; cov[4] += du * dv; cov[5] += dv * dv;
        }
        for (int j = 0; j < 6; j++) cov[j] /= cnt;
        cov[0] += 6.0; cov[3] += 3.0; cov[5] += 3.0;          // noise floor (σ ≈ 2.5 Y, 1.7 UV)
        return setModel(line, slot, my, mu, mv, cov);
    }

    /**
     * Models when the colours were not learned by clustering: preset colours
     * (setTrackColors) or the classic brightness-only mode (autoColor = false),
     * where the two brightness levels come from an Otsu split of the region above
     * the robot and trackIsBright says which one is the line.
     */
    private void buildFallbackModels() {
        final double[] cov = {100, 0, 0, 36, 0, 36};
        if (cfg.autoColor && colorPreset) {
            modelLumaOnly = false;
            if (setModel(true, 0, presetY1, presetU1 - 128, presetV1 - 128, cov)
                    && setModel(false, 0, presetY0, presetU0 - 128, presetV0 - 128, cov)) {
                nFloorModels = 1; finishModels();
            }
            return;
        }
        modelLumaOnly = true;
        final int[] hist = histogram;
        java.util.Arrays.fill(hist, 0);
        final int r0 = calibRegionTop(), r1 = Math.max(r0 + 1, ignoreTop), W = imgW;
        long cnt = 0;
        for (int row = r0; row < r1; row += 4) {
            for (int col = 0; col < W; col += 4) {
                final int i = row * W + col;
                final int y = srcFormat == FMT_ARGB ? lumaOf(srcArgb[i]) : srcY[i] & 0xFF;
                hist[y]++; cnt++;
            }
        }
        final int t = otsuThreshold(hist, cnt);
        if (otsuContrast < cfg.minContrast) { calibIssue = ISSUE_LOW_CONTRAST; return; }
        double s0 = 0, n0 = 0, s1 = 0, n1 = 0;
        for (int g = 0; g < 256; g++) { if (g <= t) { s0 += g * hist[g]; n0 += hist[g]; } else { s1 += g * hist[g]; n1 += hist[g]; } }
        final double dark = s0 / Math.max(1, n0), bright = s1 / Math.max(1, n1);
        double v0 = 0, v1 = 0;
        for (int g = 0; g < 256; g++) { if (g <= t) v0 += (g - dark) * (g - dark) * hist[g]; else v1 += (g - bright) * (g - bright) * hist[g]; }
        final double lineY = cfg.trackIsBright ? bright : dark, floorY = cfg.trackIsBright ? dark : bright;
        final double lineV = (cfg.trackIsBright ? v1 / Math.max(1, n1) : v0 / Math.max(1, n0)) + 6;
        final double floorV = (cfg.trackIsBright ? v0 / Math.max(1, n0) : v1 / Math.max(1, n1)) + 6;
        if (setModel(true, 0, lineY, 0, 0, new double[]{lineV, 0, 0, 4, 0, 4})
                && setModel(false, 0, floorY, 0, 0, new double[]{floorV, 0, 0, 4, 0, 4})) {
            nFloorModels = 1; finishModels();
        }
    }

    /** Allocate the geometry accumulators (scan rows from the robot body to the top). */
    private void initCalibRows() {
        final int step = Math.max(2, imgH / 160);
        int n = 0;
        for (int row = ignoreTop - 1 - step / 2; row >= 1; row -= step) n++;
        cgN = n;
        cgRow = new int[n]; cgSumC = new float[n]; cgSumW = new float[n]; cgSumGood = new float[n]; cgSumFloor = new float[n];
        cgClean = new int[n]; cgFrames = new int[n];
        int j = 0;
        for (int row = ignoreTop - 1 - step / 2; row >= 1 && j < n; row -= step) cgRow[j++] = row;
    }

    private void clearCalibRows() {
        if (cgRow == null) return;
        java.util.Arrays.fill(cgSumC, 0f); java.util.Arrays.fill(cgSumW, 0f); java.util.Arrays.fill(cgSumGood, 0f); java.util.Arrays.fill(cgSumFloor, 0f);
        java.util.Arrays.fill(cgClean, 0); java.util.Arrays.fill(cgFrames, 0);
    }

    /**
     * One geometry-calibration frame: locate the line in every scan row, from the
     * robot upward, following it. Returns the line column in the bottom rows (−1 if
     * the line was not found there).
     */
    private int calibGeometryFrame() {
        if (cgRow == null) initCalibRows();
        final int W = imgW;
        float pred = learnedLineW > 0f ? learnedLineX : W * 0.5f;
        float predW = learnedLineW > 0f ? learnedLineW : 0f;
        float slope = 0f;                 // columns per scan row (upward)
        int lastJ = -1, bottomCol = -1, found = 0;
        float firstC = -1f;
        for (int j = 0; j < cgN; j++) {
            final int row = cgRow[j];
            scanRowSamples(row, Math.max(2, (int) (predW > 0f ? predW / 10f : W / 160f)));
            final int n = prN;
            int good = 0, valid = 0, flr = 0;
            for (int i = 0; i < n; i++) { if (prCls[i] == CL_OUT) continue; valid++; if (prCls[i] != CL_UNK) good++; if (prCls[i] == CL_FLOOR) flr++; }
            cgFrames[j]++;
            cgSumGood[j] += valid > 0 ? good / (float) valid : 0f;
            cgSumFloor[j] += valid > 0 ? flr / (float) valid : 0f;
            // Candidate nearest the prediction with clean edges on both sides.
            final float p = lastJ >= 0 ? pred + slope * (j - lastJ) : pred;
            final float gate = lastJ >= 0 ? Math.max(12f, (predW > 0f ? predW : 40f) * Math.min(3f, 0.6f + 0.1f * (j - lastJ))) : W * 0.3f;
            float bestC = -1f, bestW = 0f, bestD = Float.MAX_VALUE;
            int i = 0;
            while (i < n) {
                if (prCls[i] != CL_LINE) { i++; continue; }
                final int a = i; int b = i, q = i + 1;
                while (q < n) {
                    if (prCls[q] == CL_LINE) { b = q; q++; }
                    else if (prCls[q] == CL_UNK && q + 1 < n && prCls[q + 1] == CL_LINE) q++;
                    else break;
                }
                i = b + 1;
                final float eL = leftEdge(a); if (!edgeClean) continue;
                final float eR = rightEdge(b); if (!edgeClean) continue;
                final float w = eR - eL, cc = 0.5f * (eL + eR);
                if (w < 3f) continue;
                if (predW > 0f && (w < 0.5f * predW * (lastJ >= 0 ? 0.8f : 0.5f) || w > 1.6f * predW)) continue;
                final float d = Math.abs(cc - p);
                if (d <= gate && d < bestD) { bestD = d; bestC = cc; bestW = w; }
            }
            if (bestC >= 0f) {
                cgClean[j]++; cgSumC[j] += bestC; cgSumW[j] += bestW;
                if (lastJ >= 0) {
                    final float sNew = (bestC - pred) / (j - lastJ);
                    slope = lastJ >= 0 && j - lastJ <= 3 ? 0.7f * slope + 0.3f * sNew : sNew;
                }
                pred = bestC; predW = predW > 0f ? 0.7f * predW + 0.3f * bestW : bestW; lastJ = j;
                if (bottomCol < 0 && j < 6) bottomCol = Math.round(bestC);
                if (found++ == 0) firstC = bestC;
            }
        }
        // The line counts as found when it was located cleanly in enough rows — not
        // necessarily the lowest ones (a disturbance may cover the near part).
        if (bottomCol < 0 && found >= 6) bottomCol = Math.round(firstC);
        return found >= 6 ? bottomCol : -1;
    }

    /** Sample image row {@code row} every {@code step} px (2×2 average) and classify with the row's light level. */
    private void scanRowSamples(int row, int step) {
        final int W = imgW;
        int n = 0, nG = 0;
        for (int col = 0; col + 1 < W && n < prY.length; col += step) {
            readPixel(col, row);
            prY[n] = sY; prU[n] = sU; prV[n] = sV; prCls[n] = CL_UNK;
            final float g = floorGain(sY, sU, sV);
            if (g > 0f && nG < prTmp.length) prTmp[nG++] = g;
            n++;
        }
        prN = n; prStep = step; prH0 = 0.5f + 0.5f;   // sample i covers column i·step … i·step+1 → centre i·step + 1
        prGain = 1f;
        classifyProfile(n, nG);
    }

    /**
     * End of the geometry phase: ROI, width model (→ tilt), line width on the floor,
     * straight-ahead / heading zero, initial path memory. On failure the geometry
     * phase restarts (calibCount = 0).
     */
    private void finishGeometry() {
        final int n = cgN;
        final boolean[] floorOk = new boolean[n], clean = new boolean[n];
        final float[] cC = new float[n], cW = new float[n];
        int nClean = 0;
        for (int j = 0; j < n; j++) {
            final int f = Math.max(1, cgFrames[j]);
            clean[j] = cgClean[j] * 2 >= f && cgClean[j] > 0;
            // the floor is there (a disturbance may cover part of the row: glare, a shadow,
            // an object — the row still belongs to the floor; beyond a table edge it does not)
            floorOk[j] = cgSumFloor[j] / f >= 0.2f;
            if (clean[j]) { cC[j] = cgSumC[j] / cgClean[j]; cW[j] = cgSumW[j] / cgClean[j]; nClean++; }
        }
        // ── ROI bottom: just above the robot body (skip low rows that are not floor) ──
        int jb = 0;
        while (jb < n / 6 && !(floorOk[jb] && (jb + 1 >= n || floorOk[jb + 1]))) jb++;
        if (jb >= n / 6) jb = 0;
        // ── ROI top: the highest floor row where the line was seen (with a second line
        //    row just below it). Rows that are disturbed in between do not end the ROI:
        //    only the end of the floor or of the visible line does. ──
        int jt = -1;
        for (int j = n - 1; j >= jb + 2; j--) {
            if (clean[j] && floorOk[j] && (clean[j - 1] || clean[j - 2])) { jt = j; break; }
        }
        if (!cfg.roiAuto) jt = n - 1;
        if (jt < 0 || nClean < 6 || jt - jb < 4) { calibIssue = ISSUE_NO_LINE; calibCount = 0; clearCalibRows(); return; }
        // ── width model w(v) = A + B·v, robust (rows with clean edges inside the ROI) ──
        double A = 0, B = 0;
        final float[] wt = new float[n];
        for (int j = jb; j <= jt; j++) wt[j] = clean[j] ? 1f : 0f;
        for (int it = 0; it < 4; it++) {
            double s = 0, sx = 0, sy = 0, sxx = 0, sxy = 0;
            for (int j = jb; j <= jt; j++) {
                if (wt[j] <= 0f) continue;
                final double x = cgRow[j] + 0.5, y = cW[j], w = wt[j];
                s += w; sx += w * x; sy += w * y; sxx += w * x * x; sxy += w * x * y;
            }
            final double det = s * sxx - sx * sx;
            if (s < 3 || Math.abs(det) < 1e-9) break;
            B = (s * sxy - sx * sy) / det; A = (sy - B * sx) / s;
            double mad = 0; int m = 0;
            for (int j = jb; j <= jt; j++) if (clean[j]) { mad += Math.abs(cW[j] - (A + B * (cgRow[j] + 0.5))); m++; }
            final double sc = Math.max(1.5, 2.5 * mad / Math.max(1, m));
            for (int j = jb; j <= jt; j++) {
                if (!clean[j]) continue;
                final double r = Math.abs(cW[j] - (A + B * (cgRow[j] + 0.5))) / sc;
                wt[j] = r >= 1 ? 0f : (float) ((1 - r * r) * (1 - r * r));
            }
        }
        calWA = (float) A; calWB = (float) B;
        // ── camera tilt from the width model (tan T = w(cy) / (f · dw/dv)) ──
        setGeometry(cfg.cameraTiltDeg);
        if (cfg.learnCameraTilt && B > 1e-6) {
            final double wc = A + B * gCy;
            final double t = Math.toDegrees(Math.atan2(wc, gFoc * B));
            int used = 0; for (int j = jb; j <= jt; j++) if (wt[j] > 0.3f) used++;
            final float spanFrac = (cgRow[jb] - cgRow[jt]) / (float) imgH;
            // trusted only near the configured tilt (a gross mismatch means the width
            // measurement is not a perspective view of a straight line)
            if (wc > 0 && t >= 15 && t <= 80 && Math.abs(t - cfg.cameraTiltDeg) <= 25 && used >= 8 && spanFrac >= 0.25f)
                setGeometry((float) t);
        }
        // ── ROI rows (top with the safety margin, as a fraction of the floor distance) ──
        roiBot = Math.min(ignoreTop, cgRow[jb] + Math.max(2, imgH / 320) + 1);
        int topRow = cgRow[jt];
        rfA = 0f; rfCos = 1f; rfSin = 0f;
        final boolean fs = cfg.pathFloorSpace;
        cfg.pathFloorSpace = true;     // geometry measurements are always made on the floor
        if (imageToPlane((float) gCx, roiBot - 0.5f)) {
            final float xb = qX;
            if (imageToPlane((float) gCx, topRow + 0.5f)) {
                final float xt = qX - cfg.roiTopMarginFrac * Math.max(0f, qX - xb);
                // invert forward distance → row on the centre column
                final double c = gCosT, sn = gSinT;
                final double yn = (c - xt * sn) / (xt * c + sn);
                topRow = (int) Math.ceil(gCy + gFoc * yn);
            }
        }
        roiTop = Math.max(0, Math.min(roiBot - 8, topRow));
        // ── line width on the floor and the calibrated line (straight-ahead, heading zero) ──
        int m = 0;
        final float[] wf = new float[n];
        double s = 0, sx = 0, sy = 0, sxx = 0, sxy = 0;
        for (int j = jb; j <= jt; j++) {
            if (wt[j] <= 0.3f || cgRow[j] < roiTop) continue;
            final float v = cgRow[j] + 0.5f;
            if (!imageToPlane(cC[j], v)) continue;
            final float x = qX, y = qY;
            wf[m++] = cW[j] * pixelSizeAt(x, y);
            s += 1; sx += x; sy += y; sxx += x * x; sxy += x * y;
        }
        if (m < 4) { cfg.pathFloorSpace = fs; calibIssue = ISSUE_NO_LINE; calibCount = 0; clearCalibRows(); return; }
        final float wFloor = median(wf, m);
        final double det = s * sxx - sx * sx;
        double bh = Math.abs(det) > 1e-9 ? (s * sxy - sx * sy) / det : 0;
        double ah = (sy - bh * sx) / s;
        if (Math.abs(Math.toDegrees(Math.atan(bh))) > cfg.steerHeadingCalibMaxDeg) {
            bh = 0; ah = sy / s;        // robot clearly not straight: keep only the offset
        }
        final double psi = Math.atan(bh);
        rfA = (float) ah; rfCos = (float) Math.cos(psi); rfSin = (float) Math.sin(psi);
        cfg.pathFloorSpace = fs;
        lineW = wFloor;
        // ── image-plane variant: pixel size at the ROI bottom = 1 unit ≈ 1 camera height ──
        {
            final boolean f2 = cfg.pathFloorSpace;
            cfg.pathFloorSpace = true;
            planeToImage(0f, 0f);
            final float u0 = pU;
            ipV0 = roiBot - 0.5f;
            imageToPlane(u0, ipV0);
            ipK = pixelSizeAt(qX, qY);
            final float xb = qX;
            planeToImage(xb, 0f); ipU0 = pU;
            planeToImage(xb + 1.5f, 0f);
            ipSlope = (pU - ipU0) / Math.max(1f, ipV0 - pV);
            cfg.pathFloorSpace = f2;
            if (!f2) lineW = Math.max(3f, calWA + calWB * ipV0) * ipK;
        }
        // ── legacy (v2) references for FrameData / getters ──
        {
            final int hr = (hRoiTop + hRoiBot) / 2;
            float bestD = Float.MAX_VALUE; float col = imgW * 0.5f;
            for (int j = 0; j < n; j++) if (clean[j] && Math.abs(cgRow[j] - hr) < bestD) { bestD = Math.abs(cgRow[j] - hr); col = cC[j]; }
            steerRefX = col; trackRefX = col;
            calibWidthPx = Math.max(1f, calWA + calWB * hr);
            lastCentroidX = Math.round(col);
            lastTrackWidthPx = calibWidthPx;
        }
        roiFarFwd = 0f;
        if (imageToPlane(imgW * 0.5f, roiTop + 0.5f)) roiFarFwd = qX;
        if (imageToPlane(imgW * 0.5f, roiBot - 0.5f)) roiNearFwd = qX;
        // ── initial path memory: the calibrated straight line ──
        resetPathState();
        final float step = Math.max(0.02f, cfg.pathStep);
        final float xs = -(cfg.pursuitPivotBehind + 0.2f);
        for (float x = xs; x <= roiFarFwd && memN < NODE_MAX; x += step) {
            memX[memN] = x; memY[memN] = 0f; memConf[memN] = 1f; memAge[memN] = 0; memVar[memN] = measVar(1f, 0f); memN++;
        }
        v3Ready = true;
        calibrated = true;
        calibIssue = ISSUE_NONE;
    }

    /** Forget the traced path, the motion estimate and the control state. */
    private void resetPathState() {
        memN = 0; ndN = 0;
        velX = 0f; velY = 0f; velT = 0f; corrX = 0f; corrY = 0f; corrT = 0f;
        mPxx = MOTION_P0X; mPxt = 0f; mPtt = MOTION_P0T; motionMeasured = false;
        v3Tracking = 100f; v3LostFrames = 0; v3HoldFrames = 0; v3BlindFrames = 0; v3LastSteer = 0f; outSteer = 0; outTrack = 100;
        prGain = 1f; lightRef = 1f; ambRef = 1f; pathEnd = END_OUT; pathExitSide = 0f;
        ctTarget = false;
        ctCurvAheadPrev = 100f; ctCurvPivot = 0f; ctCurv = 0f;
        gainCorr = 1f; agPath = 0f; agCmd = 0f; guardU = 0f; guardSide = 0f;
    }

    /**
     * Steady curve following: the robot turns as much as the path curves (agPath),
     * while it was sent agCmd — the curvature of the steering actually output
     * (including held frames, the visibility guard and full turns, so a robot that
     * only follows a curve with bursts of full steering is not mistaken for one that
     * over-steers). Their ratio is the robot's response; the scale converges slowly
     * to its inverse.
     */
    private void updateAutoGain(float steeringOut) {
        final float issued = steeringOut / Math.max(1e-3f, cfg.steerPerCurvature);
        agPath += 0.08f * (ctCurvPivot - agPath);
        agCmd  += 0.08f * (issued - agCmd);
        if (Math.abs(agPath) < 0.25f || Math.abs(agCmd) < 0.15f || agPath * agCmd <= 0f) return;
        final float g = Math.max(0.3f, Math.min(3f, agPath / agCmd));
        gainCorr += AG_RATE * (Math.max(0.5f, Math.min(2.5f, 1f / g)) - gainCorr);
    }

    /** During calibration the overlay shows the rows where the line was found (path arrays). */
    private void fillCalibPath() {
        ndN = 0;
        if (cgRow == null) return;
        for (int j = 0; j < cgN && ndN < NODE_MAX; j += 3) {
            if (cgClean[j] == 0) continue;
            ndX[ndN] = cgSumC[j] / cgClean[j];          // image column (calibration only)
            ndY[ndN] = cgRow[j] + 0.5f;
            ndSrc[ndN] = PATH_SRC_FRESH;
            ndN++;
        }
    }

    // =========================================================================
    //  v3 — tracing the path (hot path: no allocation)
    // =========================================================================

    // copy of the traced path (for trying a fresh search without losing it)
    private final float[] svX = new float[NODE_MAX], svY = new float[NODE_MAX], svConf = new float[NODE_MAX], svQ = new float[NODE_MAX];
    private final byte[]  svSt = new byte[NODE_MAX], svSrc = new byte[NODE_MAX];
    private final short[] svAge = new short[NODE_MAX];
    private final float[] svVar = new float[NODE_MAX], tmpV = new float[NODE_MAX];
    private int svN;
    private boolean memAbsent;              // this frame: clean floor where the remembered path is
    private boolean lastEndAbsent;          // the last traced path ended on clean floor (a dead end)

    private void saveNodes() {
        svN = ndN;
        System.arraycopy(ndX, 0, svX, 0, ndN); System.arraycopy(ndY, 0, svY, 0, ndN);
        System.arraycopy(ndConf, 0, svConf, 0, ndN); System.arraycopy(ndQ, 0, svQ, 0, ndN);
        System.arraycopy(ndSt, 0, svSt, 0, ndN); System.arraycopy(ndSrc, 0, svSrc, 0, ndN);
        System.arraycopy(ndAge, 0, svAge, 0, ndN); System.arraycopy(ndVar, 0, svVar, 0, ndN);
    }

    private void restoreNodes() {
        ndN = svN;
        System.arraycopy(svX, 0, ndX, 0, ndN); System.arraycopy(svY, 0, ndY, 0, ndN);
        System.arraycopy(svConf, 0, ndConf, 0, ndN); System.arraycopy(svQ, 0, ndQ, 0, ndN);
        System.arraycopy(svSt, 0, ndSt, 0, ndN); System.arraycopy(svSrc, 0, ndSrc, 0, ndN);
        System.arraycopy(svAge, 0, ndAge, 0, ndN); System.arraycopy(svVar, 0, ndVar, 0, ndN);
    }

    /** True if two directions differ by less than 45° (a found line replacing an unconfirmed memory runs about the same way). */
    private static boolean sameDirection(float ax, float ay, float bx, float by) {
        final float la = (float) Math.sqrt(ax * ax + ay * ay), lb = (float) Math.sqrt(bx * bx + by * by);
        if (la < 1e-6f || lb < 1e-6f) return false;
        return (ax * bx + ay * by) / (la * lb) > 0.707f;
    }

    private int countFresh() {
        int c = 0;
        for (int i = 0; i < ndN; i++) if (ndSrc[i] == PATH_SRC_FRESH) c++;
        return c;
    }

    /** Unit normals (pointing right of the path direction) of polyline x/y[0…n). */
    private static void normals(float[] x, float[] y, int n, float[] nx, float[] ny) {
        for (int i = 0; i < n; i++) {
            final int a = Math.max(0, i - 1), b = Math.min(n - 1, i + 1);
            float tx = x[b] - x[a], ty = y[b] - y[a];
            final float len = (float) Math.sqrt(tx * tx + ty * ty);
            if (len < 1e-6f) { tx = 1f; ty = 0f; } else { tx /= len; ty /= len; }
            nx[i] = -ty; ny[i] = tx;
        }
    }

    /** Move the remembered path by the predicted robot motion (constant velocity) and age it. */
    private void predictMemory() {
        // Prior: a robot that follows the path turns at (speed × path curvature).
        // Used only while the motion is not measured (the line hidden) — otherwise the
        // motion filter knows the turn better, and a wrong curvature near the axle
        // (memory points behind the camera) would only bend the memory.
        // (only on a clear curve: a small curvature may be the memory's own error)
        final float kp = Math.abs(ctCurvPivot) < 0.25f || Math.abs(ctCurvPivot) > 2.5f || lastMotionMeasured ? 0f : ctCurvPivot;
        if (kp != 0f) velT += Math.max(-MOTION_DT, Math.min(MOTION_DT, 0.25f * (velX * kp - velT)));
        velT = Math.max(-VEL_T_MAX, Math.min(VEL_T_MAX, velT));
        final float c = (float) Math.cos(velT), s = (float) Math.sin(velT);
        // While the path is being tracked (the motion is registered every frame) the
        // memory stays valid much longer; it fades at the full rate only while
        // nothing is measured.
        final float life = Math.max(1, cfg.pathMemoryFrames) * (v3LostFrames == 0 ? 4f : 1f);
        final float decay = (float) Math.exp(-1.0 / life);
        final float a = cfg.pursuitPivotBehind;
        // the motion may have changed since the last frame (acceleration, a new steering command)
        mPxx = Math.min(MOTION_P0X, mPxx + MOTION_QX); mPtt = Math.min(MOTION_P0T, mPtt + MOTION_QT);
        final float qs = shapeNoise();
        for (int i = 0; i < memN; i++) {
            memVar[i] = Math.min(MEM_VAR_MAX * lineW * lineW, memVar[i] + qs);
            // the robot moved forward velX and turned velT (+ = right) about its axle
            // (a wheeled robot cannot slide sideways): p' = R(−velT)·(p − (velX,0) − P) + P
            final float dx = memX[i] - velX + a, dy = memY[i];
            memX[i] = c * dx + s * dy - a;
            memY[i] = -s * dx + c * dy;
            memConf[i] *= decay;
            if (memAge[i] < Short.MAX_VALUE) memAge[i]++;
        }
    }

    /**
     * Trace the path using the remembered path as the prediction. Fills nd*; false
     * if the memory is clearly contradicted (clean floor where it says the line is,
     * nowhere confirmed) — the caller then searches from scratch.
     */
    private boolean traceWithMemory() {
        final int n = memN;
        ndN = n;
        for (int i = 0; i < n; i++) {
            ndPX[i] = memX[i]; ndPY[i] = memY[i]; ndConf[i] = memConf[i]; ndAge[i] = memAge[i]; ndVar[i] = memVar[i]; ndMV[i] = 0f;
            ndSt[i] = ST_NONE; ndR[i] = 0f; ndQ[i] = 0f; ndSrc[i] = PATH_SRC_MEMORY;
        }
        normals(ndPX, ndPY, n, ndNX, ndNY);
        for (int i = 0; i < n; i++) if (!insideRoi(ndPX[i], ndPY[i], 2f)) ndSt[i] = ST_OUT;

        final int dir = cfg.pathTraceDirection;
        if (dir == TRACE_TOP_DOWN) walkMemory(n - 1, -1, -1); else walkMemory(0, n, 1);
        if (dir == TRACE_BOTH) fillGaps();

        int meas = 0, absent = 0, unrel = 0;
        for (int i = 0; i < n; i++) {
            if (ndSt[i] == ST_MEAS) meas++; else if (ndSt[i] == ST_ABSENT) absent++; else if (ndSt[i] == ST_UNREL) unrel++;
        }
        if (meas == 0 && absent >= 2 && absent >= unrel) { memAbsent = true; return false; }

        estimateCorrection();
        assembleFromMemory();
        // Trailing points at the far end that are clean floor now: the path does not go there.
        while (ndN > 0 && ndSt[ndN - 1] == ST_ABSENT) ndN--;
        // Far end: continue into newly visible territory; near end: down to the ROI bottom.
        pathEnd = END_OUT;
        if (ndN >= 2) extendPath(true);
        if (ndN >= 2) extendPath(false);
        return ndN >= 2;
    }

    /** Walk the remembered points in index order a → b (step ±1), measuring visible ones. */
    private void walkMemory(int a, int b, int step) {
        final float ds = Math.max(0.02f, cfg.pathStep);
        boolean have = false;
        float rLast = 0f, gLast = 0f; int iLast = 0;
        for (int i = a; i != b; i += step) {
            if (ndSt[i] == ST_OUT) continue;
            final float wExp = widthAt(ndPX[i], ndPY[i]);
            final int gapN = have ? Math.abs(i - iLast) : 0;
            float pred = have ? rLast + gLast * (i - iLast) * ds : 0f;
            pred = Math.max(-2.5f * wExp, Math.min(2.5f * wExp, pred));
            final float age = Math.min(60, ndAge[i]);
            float gate = (have ? 0.5f : 0.9f) * wExp + 0.01f * age * wExp + 0.12f * wExp * gapN;
            gate = Math.min(2.5f * wExp, gate);
            measureAt(ndPX[i] + pred * ndNX[i], ndPY[i] + pred * ndNY[i], ndNX[i], ndNY[i], gate, wExp);
            ndSt[i] = msStatus;
            if (msStatus == ST_UNREL && msLineNear) ndQ[i] = -1f;      // marks "the line seems to have moved"
            if (msStatus == ST_MEAS) {
                final float r = pred + msOff;
                ndR[i] = r; ndQ[i] = msQual; ndMV[i] = measVar(msQual, prStep);
                if (have) {
                    final float g = (r - rLast) / ((i - iLast) * ds);
                    gLast = Math.max(-0.6f, Math.min(0.6f, 0.5f * gLast + 0.5f * g));
                }
                rLast = r; iLast = i; have = true;
            }
        }
    }

    /** Re-read runs of unmeasured visible points lying between two measured points, interpolating from both sides. */
    private void fillGaps() {
        final int n = ndN;
        int i = 0;
        while (i < n) {
            if (ndSt[i] != ST_MEAS) { i++; continue; }
            int j = i + 1;
            while (j < n && ndSt[j] != ST_MEAS) j++;
            if (j >= n) break;
            if (j - i > 1) {
                for (int k = i + 1; k < j; k++) {
                    if (ndSt[k] == ST_OUT) continue;
                    final float t = (k - i) / (float) (j - i);
                    final float pred = ndR[i] + t * (ndR[j] - ndR[i]);
                    final float wExp = widthAt(ndPX[k], ndPY[k]);
                    final int dist = Math.min(k - i, j - k);
                    final float gate = Math.min(2f * wExp, 0.45f * wExp + 0.08f * wExp * dist);
                    measureAt(ndPX[k] + pred * ndNX[k], ndPY[k] + pred * ndNY[k], ndNX[k], ndNY[k], gate, wExp);
                    if (msStatus == ST_MEAS) { ndSt[k] = ST_MEAS; ndR[k] = pred + msOff; ndQ[k] = msQual; ndMV[k] = measVar(msQual, prStep); }
                    else if (ndSt[k] != ST_ABSENT || msStatus == ST_UNREL) ndSt[k] = msStatus;
                }
            }
            i = j;
        }
    }

    /**
     * Robot-motion correction from the measured points: the small rigid motion
     * (corrX, corrY, corrT) that best moves the predicted path onto the measured
     * one — a Kalman update of the motion estimate. The prior is the predicted motion
     * with its uncertainty (mP); every measured point adds information in proportion
     * to its precision (1 / its variance: measurement + memory); outliers are
     * down-weighted (Huber). Many clean points correct the motion fully; a few noisy
     * ones (most of the line under glare) barely move it; the motion along a straight
     * line, which the line does not show, stays at the prediction.
     */
    private void estimateCorrection() {
        corrX = 0f; corrY = 0f; corrT = 0f; motionMeasured = false;
        final float a = cfg.pursuitPivotBehind;
        double det0 = (double) mPxx * mPtt - (double) mPxt * mPxt;
        if (det0 < 1e-14) det0 = 1e-14;
        final double i00 = mPtt / det0, i01 = -mPxt / det0, i11 = mPxx / det0;     // prior information
        double cx = 0, ct = 0, a00 = i00, a01 = i01, a11 = i11;
        for (int it = 0; it < 2; it++) {
            a00 = i00; a01 = i01; a11 = i11;
            double b0 = 0, b1 = 0;
            int m = 0;
            for (int i = 0; i < ndN; i++) {
                if (ndSt[i] != ST_MEAS) continue;
                // 2 degrees of freedom: travel along x, rotation about the axle (x = −a)
                final double jx = ndNX[i], jt = ndNY[i] * (ndPX[i] + a) - ndNX[i] * ndPY[i];
                final double var = ndMV[i] + ndVar[i];
                double w = 1.0 / Math.max(1e-8, var);
                if (it > 0) {
                    final double res = Math.abs(ndR[i] - (jx * cx + jt * ct)), hub = 2.0 * Math.sqrt(var);
                    if (res > hub) w *= hub / res;
                }
                a00 += w * jx * jx; a01 += w * jx * jt; a11 += w * jt * jt;
                b0 += w * jx * ndR[i]; b1 += w * jt * ndR[i];
                m++;
            }
            if (m < 2) return;
            final double det = a00 * a11 - a01 * a01;
            if (Math.abs(det) < 1e-14) return;
            cx = (a11 * b0 - a01 * b1) / det;
            ct = (a00 * b1 - a01 * b0) / det;
        }
        final double det = a00 * a11 - a01 * a01;
        kPxx = (float) (a11 / det); kPxt = (float) (-a01 / det); kPtt = (float) (a00 / det);
        corrX = (float) Math.max(-0.3, Math.min(0.3, cx));
        corrT = (float) Math.max(-0.15, Math.min(0.15, ct));
        corrY = corrT * a;
        motionMeasured = true;
    }

    /** Measurement variance (plane units²) of a profile measurement of quality q with sample step {@code step}. */
    private float measVar(float q, float step) {
        final float sd = (0.5f * step + 0.06f * lineW) / Math.max(0.2f, q);
        return sd * sd;
    }

    /** Per-frame growth of a remembered point's variance: the path is fixed on the floor; only small errors accumulate. */
    private float shapeNoise() { final float sd = 0.05f * lineW; return sd * sd; }

    /**
     * Final positions: measured points where measured; elsewhere the remembered
     * point moved by the motion correction plus the remaining residual interpolated
     * from the nearest measured points (so a hidden section bends with its ends).
     */
    private void assembleFromMemory() {
        final int n = ndN;
        final float decayAbsent = 0.6f;
        // Measured points are fused with the remembered point (a Kalman update across
        // the path). The rigid part of the difference is the robot's motion, applied in
        // full; the rest is the shape of a path that does not change on the floor:
        // measurement noise (glare edges, flicker), averaged over frames — unless the
        // two disagree beyond noise (then the memory was wrong and the measurement
        // replaces it): one point far off, or its neighbours off together in the same
        // direction (noise differs from point to point; a wrong memory does not).
        for (int i = 0; i < n; i++) tmpA[i] = ndSt[i] == ST_MEAS ? ndR[i] - rigidN(i) : 0f;
        for (int i = 0; i < n; i++) {
            if (ndSt[i] != ST_MEAS) continue;
            final float e = tmpA[i];
            final float P = ndVar[i], R = Math.max(1e-10f, ndMV[i]);
            float k = P / (P + R);
            float se = 0f; int m = 0;
            for (int q = Math.max(0, i - 3); q <= Math.min(n - 1, i + 3); q++) if (ndSt[q] == ST_MEAS) { se += tmpA[q]; m++; }
            final float mean = se / m;
            if (e * e > 9f * (P + R) || mean * mean * m > 6.25f * (P + R)) k = 1f;
            ndR[i] = rigidN(i) + k * e;
            ndVar[i] = k >= 1f ? R : (1f - k) * P;
        }
        for (int i = 0; i < n; i++) {
            final float px = ndPX[i], py = ndPY[i], nx = ndNX[i], ny = ndNY[i];
            if (ndSt[i] == ST_MEAS) {
                ndX[i] = px + ndR[i] * nx; ndY[i] = py + ndR[i] * ny;
                ndConf[i] = Math.max(ndQ[i], Math.min(1f, ndConf[i] + ndQ[i]));
                ndAge[i] = 0;
                ndSrc[i] = PATH_SRC_FRESH;
                continue;
            }
            final float bx = px + corrX - corrT * py, by = py + corrT * (px + cfg.pursuitPivotBehind);
            // remaining residual from the nearest measured neighbours
            int lo = i - 1; while (lo >= 0 && ndSt[lo] != ST_MEAS) lo--;
            int hi = i + 1; while (hi < n && ndSt[hi] != ST_MEAS) hi++;
            float rr = 0f;
            if (lo >= 0 && hi < n) {
                final float rl = ndR[lo] - rigidN(lo), rh = ndR[hi] - rigidN(hi);
                rr = rl + (rh - rl) * (i - lo) / (float) (hi - lo);
            } else if (lo >= 0) {
                rr = (ndR[lo] - rigidN(lo)) * Math.max(0f, 1f - (i - lo) / 8f);
            } else if (hi < n) {
                rr = (ndR[hi] - rigidN(hi)) * Math.max(0f, 1f - (hi - i) / 8f);
            }
            ndX[i] = bx + rr * nx; ndY[i] = by + rr * ny;
            // "Absent" before a measured section is a gap (the line is hidden by
            // something that looks like the floor), not the end of the line: only
            // clean floor after the last measured point can mean the line ends.
            if (ndSt[i] == ST_ABSENT && hi < n) ndSt[i] = ST_UNREL;
            if (ndSt[i] == ST_ABSENT) ndConf[i] *= decayAbsent;
            ndSrc[i] = ndSt[i] == ST_UNREL ? PATH_SRC_UNRELIABLE : PATH_SRC_MEMORY;
        }
    }

    /** Normal displacement of point i under the rigid correction. */
    private float rigidN(int i) {
        return ndNX[i] * corrX + (ndNY[i] * (ndPX[i] + cfg.pursuitPivotBehind) - ndNX[i] * ndPY[i]) * corrT;
    }

    /** Append (far = true) or prepend a point. */
    private boolean addNode(boolean far, float x, float y, byte st, byte src, float conf, float q) {
        if (ndN >= NODE_MAX) return false;
        int i = ndN;
        if (!far) {
            for (int k = ndN; k > 0; k--) {
                ndX[k] = ndX[k - 1]; ndY[k] = ndY[k - 1]; ndSt[k] = ndSt[k - 1]; ndSrc[k] = ndSrc[k - 1];
                ndConf[k] = ndConf[k - 1]; ndQ[k] = ndQ[k - 1]; ndAge[k] = ndAge[k - 1]; ndVar[k] = ndVar[k - 1];
            }
            i = 0;
        }
        ndX[i] = x; ndY[i] = y; ndSt[i] = st; ndSrc[i] = src; ndConf[i] = conf; ndQ[i] = q; ndAge[i] = 0;
        ndVar[i] = q > 0f ? measVar(q, prStep) : MEM_VAR_MAX * lineW * lineW;
        ndN++;
        return true;
    }

    /** Index of the k-th point from the far (or near) end. */
    private int endIdx(boolean far, int k) { return far ? ndN - 1 - k : k; }

    /**
     * Continue the path beyond its far end (or before its near end) into territory
     * not remembered yet: predict the next point from the end direction and
     * curvature, measure across it, and repeat. An unreadable stretch is bridged
     * (up to Config.pathMaxGap) and kept only if the line is found again beyond
     * it. Clean floor ahead triggers a corner search (a sharp turn) and then a
     * jump search (the line continues after a floor-coloured object); if both fail
     * the path ends there.
     */
    private void extendPath(boolean far) {
        final float ds = Math.max(0.02f, cfg.pathStep);
        final int e0 = endIdx(far, 0), e1 = endIdx(far, Math.min(ndN - 1, 2));
        // the end must be in view (else there is nothing to see beyond it) — except a
        // far end still below the ROI (under the robot): it is continued into view
        final boolean below = far && planeToImage(ndX[e0], ndY[e0]) && pV >= roiBot - 1f && pU > 0f && pU < imgW;
        if (!below && !insideRoi(ndX[e0], ndY[e0], 1f)) {
            if (far && planeToImage(ndX[e0], ndY[e0])) {
                pathEnd = END_OUT;
                pathExitSide = pU < imgW * 0.5f ? -1f : 1f;
                if (pV < roiTop + 2f) pathExitSide = 0f;
            }
            return;
        }
        float hx = ndX[e0] - ndX[e1], hy = ndY[e0] - ndY[e1];
        float len = (float) Math.sqrt(hx * hx + hy * hy);
        if (len < 1e-6f) return;
        hx /= len; hy /= len;
        float kappa = endCurvature(far) * (far ? 1f : -1f);
        float lx = ndX[e0], ly = ndY[e0];          // last measured (or end) point
        int pending = 0;                           // unmeasured points appended since the last measurement
        int absentRun = 0, trimRun = 0;
        float gap = 0f;
        final int budget = NODE_MAX - ndN;
        for (int step = 0; step < budget; step++) {
            // rotate the direction by the curvature (decaying, so a wrong estimate cannot spiral)
            final float a = kappa * ds;
            final float c = (float) Math.cos(a), s = (float) Math.sin(a);
            final float nhx = c * hx - s * hy, nhy = s * hx + c * hy;
            hx = nhx; hy = nhy;
            kappa *= 0.9f;
            final float px = lx + ds * hx * (pending + 1), py = ly + ds * hy * (pending + 1);
            final float nx = -hy, ny = hx;
            if (!insideRoi(px, py, 1f)) {
                if (far && planeToImage(px, py) && pV >= roiBot - 1f && pU > 0f && pU < imgW && step < 12) {
                    // still below the ROI: continue the remembered direction until it enters the view
                    if (!addNode(true, px, py, ST_OUT, PATH_SRC_PREDICTED, 0.3f, 0f)) break;
                    lx = px; ly = py; pending = 0;
                    continue;
                }
                if (far) {
                    pathEnd = END_OUT;
                    planeToImage(px, py);
                    pathExitSide = pU < imgW * 0.5f ? -1f : 1f;
                    if (pV < roiTop + 2f) pathExitSide = 0f;
                }
                break;
            }
            final float wExp = widthAt(px, py);
            final float gate = Math.min(3f * wExp, 0.7f * wExp + 0.3f * wExp * pending);
            measureAt(px, py, nx, ny, gate, wExp);
            if (msStatus == ST_MEAS && msTrim && ++trimRun > 6) { if (far) pathEnd = END_UNKNOWN; break; }
            if (msStatus == ST_MEAS && nearOwnPath(far, px + msOff * nx, py + msOff * ny)) { if (far) pathEnd = END_UNKNOWN; break; }
            if (msStatus == ST_MEAS && !msTrim) trimRun = 0;
            if (msStatus == ST_MEAS) {
                final float mx = px + msOff * nx, my = py + msOff * ny;
                // bridge points get positions on the straight segment last → this one
                for (int k = 0; k < pending; k++) {
                    final int idx = endIdx(far, pending - 1 - k);
                    final float t = (k + 1f) / (pending + 1f);
                    ndX[idx] = lx + t * (mx - lx); ndY[idx] = ly + t * (my - ly);
                }
                if (!addNode(far, mx, my, ST_MEAS, PATH_SRC_FRESH, msQual, msQual)) break;
                // new direction from the measured points (smoothed)
                float dx = mx - lx, dy = my - ly;
                final float dl = (float) Math.sqrt(dx * dx + dy * dy);
                if (dl > 1e-6f) {
                    dx /= dl; dy /= dl;
                    final float cross = hx * dy - hy * dx;            // turn since the prediction
                    kappa = Math.max(-2.5f, Math.min(2.5f, kappa + 0.5f * cross / Math.max(ds, dl)));
                    hx = 0.5f * hx + 0.5f * dx; hy = 0.5f * hy + 0.5f * dy;
                    final float hl = (float) Math.sqrt(hx * hx + hy * hy); hx /= hl; hy /= hl;
                }
                lx = mx; ly = my; pending = 0; absentRun = 0; gap = 0f;
                continue;
            }
            if (msStatus == ST_OUT) { if (far) pathEnd = END_OUT; break; }
            if (msStatus == ST_ABSENT) absentRun++; else absentRun = 0;
            gap += ds;
            if (absentRun >= 2 || (msStatus == ST_ABSENT && pending == 0 && absentRun >= 1 && step == 0)) {
                // clean floor where the path should continue: a corner, or a hidden piece, or the end
                removeTrailing(far, pending); pending = 0;
                if (cornerSearch(far, lx, ly, hx, hy)) {
                    final int i0 = endIdx(far, 0), i1 = endIdx(far, 1);
                    hx = ndX[i0] - ndX[i1]; hy = ndY[i0] - ndY[i1];
                    final float hl = (float) Math.sqrt(hx * hx + hy * hy); hx /= hl; hy /= hl;
                    lx = ndX[i0]; ly = ndY[i0]; kappa = 0f; absentRun = 0; gap = 0f;
                    continue;
                }
                if (jumpSearch(far, lx, ly, hx, hy, kappa / 0.9f)) {
                    final int i0 = endIdx(far, 0);
                    lx = ndX[i0]; ly = ndY[i0]; absentRun = 0; gap = 0f;
                    continue;
                }
                if (far) pathEnd = END_ABSENT;
                break;
            }
            if (gap > cfg.pathMaxGap) {
                // unreadable for longer than pathMaxGap: keep the bridged guess (it is
                // unknown, not absent — confidence fades), stop extending
                if (msStatus == ST_ABSENT) removeTrailing(far, pending);
                pending = 0; if (far) pathEnd = END_UNKNOWN; break;
            }
            if (!addNode(far, px, py, msStatus, PATH_SRC_PREDICTED, 0.25f, 0f)) break;
            pending++;
        }
        if (pending > 0) {
            // trailing unread points: drop clean-floor ones, keep unreliable ones (unknown, not absent)
            int k = 0;
            while (k < pending && ndSt[endIdx(far, k)] == ST_ABSENT) k++;
            removeTrailing(far, k);
            if (far && pathEnd == END_OUT) pathEnd = END_UNKNOWN;
        }
    }

    /**
     * True if (x, y) lies on the path already traced (closer than one line width to a
     * point more than 4 points from the growing end): the trace would fold back.
     */
    private boolean nearOwnPath(boolean far, float x, float y) {
        final float lim = lineW * lineW;
        for (int k = 5; k < ndN; k++) {
            final int i = endIdx(far, k);
            final float dx = ndX[i] - x, dy = ndY[i] - y;
            if (dx * dx + dy * dy < lim) return true;
        }
        return false;
    }

    /** Remove the last {@code k} points at the far (or near) end. */
    private void removeTrailing(boolean far, int k) {
        if (k <= 0) return;
        if (far) { ndN = Math.max(0, ndN - k); return; }
        final int m = Math.min(k, ndN);
        for (int i = m; i < ndN; i++) {
            ndX[i - m] = ndX[i]; ndY[i - m] = ndY[i]; ndSt[i - m] = ndSt[i]; ndSrc[i - m] = ndSrc[i];
            ndConf[i - m] = ndConf[i]; ndQ[i - m] = ndQ[i]; ndAge[i - m] = ndAge[i]; ndVar[i - m] = ndVar[i];
        }
        ndN -= m;
    }

    /** Curvature of the path near its far (near) end, from the last three points (1/plane unit, + = right). */
    private float endCurvature(boolean far) {
        if (ndN < 5) return 0f;
        final int a = endIdx(far, 0), b = endIdx(far, 2), c = endIdx(far, 4);
        final float x1 = ndX[c], y1 = ndY[c], x2 = ndX[b], y2 = ndY[b], x3 = ndX[a], y3 = ndY[a];
        final float ax = x2 - x1, ay = y2 - y1, bx = x3 - x2, by = y3 - y2;
        final float la = (float) Math.sqrt(ax * ax + ay * ay), lb = (float) Math.sqrt(bx * bx + by * by);
        if (la < 1e-6f || lb < 1e-6f) return 0f;
        final float cross = ax * by - ay * bx;
        final float k = 2f * cross / (la * lb * (la + lb));
        return Math.max(-2f, Math.min(2f, (far ? 1f : -1f) * k));
    }

    private static final float[] CORNER_ANGLES = {25f, 40f, 55f, 70f, 85f, 100f};

    /** The line turns sharply: probe directions turned up to ±100° from the last point. */
    private boolean cornerSearch(boolean far, float lx, float ly, float hx, float hy) {
        final float ds = Math.max(0.02f, cfg.pathStep);
        final float[] angles = CORNER_ANGLES;
        float bestQ = 0f, bx = 0f, by = 0f;
        for (float ang : angles) {
            for (int sgn = -1; sgn <= 1; sgn += 2) {
                final double a = Math.toRadians(ang * sgn);
                final float c = (float) Math.cos(a), s = (float) Math.sin(a);
                final float dx = c * hx - s * hy, dy = s * hx + c * hy;
                // never turn back toward the robot (a path does not double back)
                if (far && dx < -0.35f) continue;
                for (float d = 1.5f; d <= 2.6f; d += 1.0f) {
                    final float px = lx + d * ds * dx, py = ly + d * ds * dy;
                    if (!insideRoi(px, py, 1f) || nearOwnPath(far, px, py)) continue;
                    final float wExp = widthAt(px, py);
                    measureAt(px, py, -dy, dx, 0.5f * wExp, wExp);
                    if (msStatus == ST_MEAS && msQual >= 0.75f && msQual > bestQ + 0.05f) {
                        bestQ = msQual; bx = px - msOff * dy; by = py + msOff * dx;
                    }
                }
            }
            if (bestQ > 0f) break;          // prefer the gentlest turn that works
        }
        if (bestQ <= 0f) return false;
        return addNode(far, bx, by, ST_MEAS, PATH_SRC_FRESH, bestQ, bestQ);
    }

    /** The line may continue after a floor-coloured object: look further along the direction. */
    private boolean jumpSearch(boolean far, float lx, float ly, float hx, float hy, float kappa) {
        final float ds = Math.max(0.02f, cfg.pathStep);
        for (float d = 3f; d * ds <= cfg.pathMaxGap; d += 1.5f) {
            // follow the current curvature along the gap (chord of the arc)
            final float ang = 0.5f * kappa * d * ds;
            final float ca = (float) Math.cos(ang), sa = (float) Math.sin(ang);
            final float ux = ca * hx - sa * hy, uy = sa * hx + ca * hy;
            final float ang2 = kappa * d * ds;
            final float tx = (float) Math.cos(ang2) * hx - (float) Math.sin(ang2) * hy, ty = (float) Math.sin(ang2) * hx + (float) Math.cos(ang2) * hy;
            final float px = lx + d * ds * ux, py = ly + d * ds * uy;
            if (!insideRoi(px, py, 1f)) return false;
            final float wExp = widthAt(px, py);
            measureAt(px, py, -ty, tx, Math.min(2f * wExp, 0.5f * wExp + 0.05f * d * wExp), wExp);
            if (msStatus == ST_MEAS && msQual >= 0.75f) {
                final float mx = px - msOff * ty, my = py + msOff * tx;
                final int k = (int) d;
                for (int i = 1; i < k; i++) {
                    final float t = i / (float) k;
                    if (!addNode(far, lx + t * (mx - lx), ly + t * (my - ly), ST_UNREL, PATH_SRC_PREDICTED, 0.25f, 0f)) return false;
                }
                return addNode(far, mx, my, ST_MEAS, PATH_SRC_FRESH, msQual, msQual);
            }
        }
        return false;
    }

    /**
     * Find the path from scratch: scan a few image rows across the full width for a
     * run with the line's colour, width and clean edges (preferring straight
     * ahead, or the side where the line was last seen), find its direction, and
     * trace from there in both directions. Rows are tried near → far, or far →
     * near with TRACE_TOP_DOWN.
     */
    private boolean acquire() {
        ndN = 0;
        pathEnd = END_OUT;
        final int rows = 8;
        final boolean topDown = cfg.pathTraceDirection == TRACE_TOP_DOWN;
        for (int k = 0; k < rows; k++) {
            final int kk = topDown ? rows - 1 - k : k;
            final float fr = (kk + 0.5f) / rows;
            final int row = Math.max(roiTop + 1, Math.min(roiBot - 3, Math.round(roiBot - 2 - fr * (roiBot - roiTop - 4))));
            if (!seedRow(row)) continue;
            final float sx = qX, sy = qY, sq = msQual;
            // direction: probe a fan of directions
            final float ds = Math.max(0.02f, cfg.pathStep);
            final float base = topDown ? (float) Math.PI : 0f;
            float bestS = 0f, bdx = 0f, bdy = 0f, bmx = 0f, bmy = 0f;
            for (int a = -8; a <= 8; a++) {
                final double ang = base + Math.toRadians(10 * a);
                final float dx = (float) Math.cos(ang), dy = (float) Math.sin(ang);
                final float px = sx + 1.5f * ds * dx, py = sy + 1.5f * ds * dy;
                if (!insideRoi(px, py, 1f)) continue;
                final float wExp = widthAt(px, py);
                measureAt(px, py, -dy, dx, 0.45f * wExp, wExp);
                if (msStatus != ST_MEAS) continue;
                final float score = msQual * (1f - 0.25f * Math.abs(a) / 8f);
                if (score > bestS) { bestS = score; bdx = dx; bdy = dy; bmx = px - msOff * dy; bmy = py + msOff * dx; }
            }
            if (bestS <= 0f) continue;
            // two points: the seed and the probe, ordered near → far
            final boolean probeFar = bmx >= sx;
            ndN = 0;
            if (probeFar) { addNode(true, sx, sy, ST_MEAS, PATH_SRC_FRESH, sq, sq); addNode(true, bmx, bmy, ST_MEAS, PATH_SRC_FRESH, bestS, bestS); }
            else          { addNode(true, bmx, bmy, ST_MEAS, PATH_SRC_FRESH, bestS, bestS); addNode(true, sx, sy, ST_MEAS, PATH_SRC_FRESH, sq, sq); }
            extendPath(true);
            extendPath(false);
            if (ndN >= 3) {
                for (int i = 0; i < ndN; i++) ndAge[i] = 0;
                return true;
            }
        }
        ndN = 0;
        return false;
    }

    /** Best line candidate on image row {@code row} (full width). Sets qX / qY (plane) and msQual. */
    private boolean seedRow(int row) {
        final float wPx = Math.max(4f, calWA + calWB * (row + 0.5f));
        final int step = Math.max(2, Math.round(wPx / 9f));
        scanRowSamples(row, step);
        final int n = prN;
        // preferred column: straight ahead (robot frame y = pref) at this row's distance
        float prefU = imgW * 0.5f;
        if (imageToPlane(imgW * 0.5f, row + 0.5f) && planeToImage(qX, v3PrefLateral())) prefU = pU;
        float best = 0f, bestC = -1f, bestQ = 0f;
        int i = 0;
        while (i < n) {
            if (prCls[i] != CL_LINE) { i++; continue; }
            final int a = i; int b = i, q = i + 1;
            while (q < n) {
                if (prCls[q] == CL_LINE) { b = q; q++; }
                else if (prCls[q] == CL_UNK && q + 1 < n && prCls[q + 1] == CL_LINE) q++;
                else break;
            }
            i = b + 1;
            final float eL = leftEdge(a); final boolean cl = edgeClean;
            final float eR = rightEdge(b); final boolean cr = edgeClean;
            if (!cl && !cr) continue;
            final float ratio = (eR - eL) / wPx;
            float c, qual;
            if (cl && cr) { if (ratio < 0.6f || ratio > 1.5f) continue; c = 0.5f * (eL + eR); qual = 1f - 0.4f * Math.abs(ratio - 1f); }
            else {
                // only a line cut by the image edge (width plausible); a run against
                // unknown pixels is not a safe place to start
                final boolean atEdge = (!cl && a == 0) || (!cr && b == n - 1);
                if (!atEdge || ratio > 1.3f || ratio < 0.3f) continue;
                c = cl ? eL + 0.5f * wPx : eR - 0.5f * wPx; qual = 0.75f;
            }
            final float d = (c - prefU) / (imgW * 0.3f);
            final float score = qual / (1f + d * d);
            if (score > best) { best = score; bestC = c; bestQ = qual; }
        }
        if (bestC < 0f || bestQ < 0.55f) return false;
        if (!imageToPlane(bestC, row + 0.5f)) return false;
        msQual = bestQ;
        return true;
    }

    /** Lateral position (robot frame) where re-acquisition looks first. */
    private float v3PrefLateral() {
        return v3LostFrames > 0 ? 0.6f * pathExitSide * Math.max(0.5f, roiFarFwd * 0.25f) : 0f;
    }

    // =========================================================================
    //  v3 — path post-processing and control
    // =========================================================================

    /**
     * Re-sample the path at equal steps (Config.pathStep) from its first point, and
     * drop points behind the robot's turning point. Status / source / confidence /
     * age are taken from the nearer original point; FRESH wins over the others.
     */
    private void resamplePath() {
        final float ds = Math.max(0.02f, cfg.pathStep);
        final float xMin = -(cfg.pursuitPivotBehind + 0.4f);
        int start = 0;
        while (start < ndN - 2 && ndX[start + 1] < xMin) start++;
        int m = 0;
        float acc = 0f;               // distance along the original polyline to the next output point
        tmpA[m] = ndX[start]; tmpB[m] = ndY[start]; tmpC[m] = ndConf[start]; tmpS[m] = ndSt[start]; tmpT[m] = ndSrc[start]; tmpG[m] = ndAge[start]; tmpV[m] = ndVar[start]; m++;
        for (int i = start; i < ndN - 1 && m < NODE_MAX; i++) {
            final float dx = ndX[i + 1] - ndX[i], dy = ndY[i + 1] - ndY[i];
            final float seg = (float) Math.sqrt(dx * dx + dy * dy);
            if (seg < 1e-6f) continue;
            float pos = ds - acc;
            while (pos <= seg && m < NODE_MAX) {
                final float t = pos / seg;
                final int near = t < 0.5f ? i : i + 1, other = near == i ? i + 1 : i;
                tmpA[m] = ndX[i] + t * dx; tmpB[m] = ndY[i] + t * dy;
                final boolean fresh = ndSrc[near] == PATH_SRC_FRESH || (ndSrc[other] == PATH_SRC_FRESH && Math.abs(t - 0.5f) < 0.25f);
                tmpS[m] = fresh && ndSt[near] != ST_MEAS ? ST_MEAS : ndSt[near];
                tmpT[m] = fresh ? PATH_SRC_FRESH : ndSrc[near];
                tmpC[m] = ndConf[i] + t * (ndConf[i + 1] - ndConf[i]);
                tmpG[m] = (short) Math.min(ndAge[i], ndAge[i + 1]);
                tmpV[m] = ndVar[i] + t * (ndVar[i + 1] - ndVar[i]);
                m++;
                pos += ds;
            }
            acc = seg - (pos - ds);
        }
        for (int i = 0; i < m; i++) {
            ndX[i] = tmpA[i]; ndY[i] = tmpB[i]; ndConf[i] = tmpC[i]; ndSt[i] = tmpS[i]; ndSrc[i] = tmpT[i]; ndAge[i] = tmpG[i]; ndVar[i] = tmpV[i];
        }
        ndN = m;
        // A real path cannot fold back: a turn of more than 150° within 3 steps (no
        // corner is that tight) means the trace ran back along itself — cut it there.
        for (int i = 1; i + 3 < ndN; i++) {
            final float ax = ndX[i] - ndX[i - 1], ay = ndY[i] - ndY[i - 1];
            final float bx = ndX[i + 3] - ndX[i + 2], by = ndY[i + 3] - ndY[i + 2];
            final float la = dist(ax, ay), lb = dist(bx, by);
            if (la < 1e-6f || lb < 1e-6f) continue;
            if ((ax * bx + ay * by) / (la * lb) < -0.87f) { ndN = i + 1; break; }
        }
    }

    /**
     * Smooth, outlier-robust curve through the path points: at every point a
     * weighted local quadratic (x(s), y(s) over ±3 points, tricube × confidence
     * weights) gives position, direction and curvature; then points far from the
     * curve (Tukey) are down-weighted and the fit repeated.
     */
    private void smoothPath() {
        final int n = ndN, K = 3;
        for (int i = 0; i < n; i++) {
            float w = ndConf[i];
            if (ndSrc[i] == PATH_SRC_FRESH) w = Math.max(0.3f, w);
            else if (ndSrc[i] == PATH_SRC_PREDICTED) w *= 0.3f;
            else w *= 0.6f;
            smW[i] = Math.max(1e-3f, w);
            tmpC[i] = 1f;                                   // robust weight
        }
        for (int pass = 0; pass < 2; pass++) {
            for (int i = 0; i < n; i++) {
                double s0 = 0, s1 = 0, s2 = 0, s3 = 0, s4 = 0, x0 = 0, x1 = 0, x2 = 0, y0 = 0, y1 = 0, y2 = 0;
                for (int j = Math.max(0, i - K); j <= Math.min(n - 1, i + K); j++) {
                    final double t = j - i, u = Math.abs(t) / (K + 1.0);
                    final double tri = (1 - u * u * u); final double w = smW[j] * tmpC[j] * tri * tri * tri;
                    s0 += w; s1 += w * t; s2 += w * t * t; s3 += w * t * t * t; s4 += w * t * t * t * t;
                    x0 += w * ndX[j]; x1 += w * t * ndX[j]; x2 += w * t * t * ndX[j];
                    y0 += w * ndY[j]; y1 += w * t * ndY[j]; y2 += w * t * t * ndY[j];
                }
                final double det = s0 * (s2 * s4 - s3 * s3) - s1 * (s1 * s4 - s3 * s2) + s2 * (s1 * s3 - s2 * s2);
                if (Math.abs(det) < 1e-9 || n < 3) {
                    smX[i] = ndX[i]; smY[i] = ndY[i];
                    final int a = Math.max(0, i - 1), b = Math.min(n - 1, i + 1);
                    float tx = ndX[b] - ndX[a], ty = ndY[b] - ndY[a];
                    final float l = (float) Math.sqrt(tx * tx + ty * ty);
                    smTX[i] = l > 1e-6f ? tx / l : 1f; smTY[i] = l > 1e-6f ? ty / l : 0f; smK[i] = 0f;
                    continue;
                }
                // Cramer for [s0 s1 s2; s1 s2 s3; s2 s3 s4]·(c0,c1,c2) = (r0,r1,r2)
                final double ax0 = cramer(s0, s1, s2, s3, s4, x0, x1, x2, 0, det), ax1 = cramer(s0, s1, s2, s3, s4, x0, x1, x2, 1, det),
                             ax2 = cramer(s0, s1, s2, s3, s4, x0, x1, x2, 2, det);
                final double ay0 = cramer(s0, s1, s2, s3, s4, y0, y1, y2, 0, det), ay1 = cramer(s0, s1, s2, s3, s4, y0, y1, y2, 1, det),
                             ay2 = cramer(s0, s1, s2, s3, s4, y0, y1, y2, 2, det);
                smX[i] = (float) ax0; smY[i] = (float) ay0;
                final double sp = Math.sqrt(ax1 * ax1 + ay1 * ay1);
                if (sp < 1e-9) { smTX[i] = 1f; smTY[i] = 0f; smK[i] = 0f; continue; }
                smTX[i] = (float) (ax1 / sp); smTY[i] = (float) (ay1 / sp);
                smK[i] = (float) Math.max(-5, Math.min(5, 2 * (ax1 * ay2 - ay1 * ax2) / (sp * sp * sp)));
            }
            if (pass == 1) {
                // Curvature: change of direction over ±3 points of the smoothed curve
                // (direction is much less noisy than a second derivative).
                for (int i = 0; i < n; i++) {
                    final int a = Math.max(0, i - 3), b = Math.min(n - 1, i + 3);
                    if (b - a < 2) { tmpA[i] = 0f; continue; }
                    float arc = 0f;
                    for (int k = a; k < b; k++) arc += dist(smX[k + 1] - smX[k], smY[k + 1] - smY[k]);
                    final float cr = smTX[a] * smTY[b] - smTY[a] * smTX[b], dt = smTX[a] * smTX[b] + smTY[a] * smTY[b];
                    tmpA[i] = arc > 1e-6f ? (float) Math.atan2(cr, dt) / arc : 0f;
                }
                for (int i = 0; i < n; i++) smK[i] = tmpA[i];
            }
            if (pass == 0) {
                final float c = 0.6f * Math.max(1e-3f, lineW);
                for (int i = 0; i < n; i++) {
                    final float dx = ndX[i] - smX[i], dy = ndY[i] - smY[i];
                    final float r = (float) Math.sqrt(dx * dx + dy * dy) / c;
                    tmpC[i] = r >= 1f ? 0.02f : (1f - r * r) * (1f - r * r);
                }
            }
        }
    }

    /** Component {@code k} of the solution of the symmetric 3×3 moment system (Cramer). */
    private static double cramer(double s0, double s1, double s2, double s3, double s4,
                                 double r0, double r1, double r2, int k, double det) {
        if (k == 0) return (r0 * (s2 * s4 - s3 * s3) - s1 * (r1 * s4 - s3 * r2) + s2 * (r1 * s3 - s2 * r2)) / det;
        if (k == 1) return (s0 * (r1 * s4 - s3 * r2) - r0 * (s1 * s4 - s3 * s2) + s2 * (s1 * r2 - r1 * s2)) / det;
        return (s0 * (s2 * r2 - r1 * s3) - s1 * (s1 * r2 - r1 * s2) + r0 * (s1 * s3 - s2 * s2)) / det;
    }

    /**
     * Steering and tracking from the smoothed path.
     *   Steering: pure pursuit — the point T of the path at distance L
     *   (pursuitLookAhead) from the robot's turning point P; the arc from P through
     *   T has curvature κ = 2·lateral / distance²; steering = steerPerCurvature·κ.
     *   Tracking: 100 on a straight path, lower with the sharpest curvature within
     *   powerPreview ahead, and lower when the known path ends (clean floor) before
     *   the far edge of the view; scaled by the path confidence; rate-limited.
     */
    private void computeControl() {
        final int n = ndN;
        final float px = -cfg.pursuitPivotBehind, L0 = Math.max(0.05f, cfg.pursuitLookAhead);
        ctTarget = false; ctKappa = 0f; ctSteer = 0f;
        // robot point on the path: where the path crosses x = 0 (or its nearest point)
        int iR = 0; float tR = 0f;
        {
            boolean found = false;
            for (int i = 0; i < n - 1; i++) {
                if ((smX[i] <= 0f && smX[i + 1] > 0f)) { iR = i; tR = -smX[i] / (smX[i + 1] - smX[i]); found = true; break; }
            }
            if (!found) { iR = 0; tR = 0f; }
        }
        final boolean before = smX[0] > 0f;             // path starts ahead of the camera's floor point
        if (before) {
            ctOffset = smY[0] - smX[0] * (smTX[0] > 0.05f ? smTY[0] / smTX[0] : 0f);
            ctHeading = (float) Math.toDegrees(Math.atan2(smTY[0], smTX[0]));
            ctCurv = smK[0];
        } else {
            final int j = Math.min(n - 1, iR + 1);
            ctOffset = smY[iR] + tR * (smY[j] - smY[iR]);
            ctHeading = (float) Math.toDegrees(Math.atan2(smTY[iR] + tR * (smTY[j] - smTY[iR]), smTX[iR] + tR * (smTX[j] - smTX[iR])));
            ctCurv = smK[iR] + tR * (smK[j] - smK[iR]);
        }
        // closest point to the pivot, then the first point at distance ≥ L beyond it
        int iC = 0; float dC = Float.MAX_VALUE;
        for (int i = 0; i < n; i++) {
            final float dx = smX[i] - px;
            final float d = dx * dx + smY[i] * smY[i];
            if (d < dC) { dC = d; iC = i; }
        }
        // curvature of the path where the robot's axle is (for the turn-rate prior)
        ctCurvPivot = n > 0 ? smK[iC] : 0f;
        // Adaptive look-ahead: a sharp bend within reach shortens it, so a tight corner
        // is followed instead of cut (L / (1 + L·κ/2), at least half a camera height).
        float kNear = 0f;
        for (int i = iC; i < n && dist(smX[i] - px, smY[i]) <= L0; i++) {
            float ka = 0f; int kn = 0;
            for (int q = Math.max(0, i - 2); q <= Math.min(n - 1, i + 2); q++) { ka += smK[q]; kn++; }
            kNear = Math.max(kNear, Math.abs(ka / kn));
        }
        final float L = Math.max(Math.min(0.5f, L0), L0 / (1f + 0.5f * L0 * kNear));
        ctLook = L;
        for (int i = Math.max(iC, 1); i < n; i++) {
            final float d1 = dist(smX[i] - px, smY[i]);
            if (d1 >= L) {
                final float d0 = dist(smX[i - 1] - px, smY[i - 1]);
                final float t = d1 - d0 > 1e-6f ? (L - d0) / (d1 - d0) : 1f;
                final float tt = Math.max(0f, Math.min(1f, t));
                ctTX = smX[i - 1] + tt * (smX[i] - smX[i - 1]);
                ctTY = smY[i - 1] + tt * (smY[i] - smY[i - 1]);
                ctTarget = true;
                break;
            }
        }
        if (!ctTarget && n >= 1) {
            // path shorter than L: continue along its end direction
            final int e = n - 1;
            final float ex = smX[e] - px, ey = smY[e], tx = smTX[e], ty = smTY[e];
            final float b = ex * tx + ey * ty, c = ex * ex + ey * ey - L * L;
            final float disc = b * b - c;
            final float tau = disc >= 0f ? -b + (float) Math.sqrt(disc) : 0f;
            ctTX = smX[e] + Math.max(0f, tau) * tx; ctTY = smY[e] + Math.max(0f, tau) * ty;
            ctTarget = true;
        }
        if (ctTarget) {
            final float dx = ctTX - px, dy = ctTY;
            final float d2 = Math.max(1e-4f, dx * dx + dy * dy);
            ctKappa = 2f * dy / d2;
            if (dx < 0f) ctKappa = Math.signum(dy) * Math.max(Math.abs(ctKappa), 2f / L);
            ctSteer = Math.max(-50f, Math.min(50f, cfg.steerPerCurvature * ctKappa * (cfg.steerAutoGain ? gainCorr : 1f)));
        }
        // Visibility guard: the path leaves the image through a SIDE edge close in front
        // of the robot (the robot is not turning enough — low steering gain, a tight
        // curve). The steering toward that side is raised to at least 50 × urgency,
        // urgency growing from 0 (exit 1.5 ROI-bottom distances ahead) to 1 (exit at the
        // ROI bottom). Never triggers while the robot follows the path well.
        ctGuard = 0f;
        if (ctTarget) {
            final float xb = roiNearFwd, x0 = 1.5f * roiNearFwd;
            float xExit = -1f, side = 0f;
            for (int i = iC; i < n; i++) {
                if (smX[i] < xb || !planeToImage(smX[i], smY[i])) continue;
                if (pV < roiTop) break;                                   // leaves at the top: fine
                if (pU < 0f || pU > imgW) { xExit = smX[i]; side = pU < 0f ? -1f : 1f; break; }
            }
            // the traced path stops at the image edge: its end is the exit point
            if (xExit < 0f && pathEnd == END_OUT && pathExitSide != 0f && n > 0) { xExit = Math.max(xb, smX[n - 1]); side = pathExitSide; }
            // urgency with fast attack and slow release (no on/off toggling)
            final float u = xExit >= 0f ? Math.max(0f, Math.min(1f, (x0 - xExit) / Math.max(1e-3f, x0 - xb))) : 0f;
            if (xExit >= 0f && side != guardSide) { guardU = 0f; guardSide = side; }
            // slow release while the path still leaves the side, quick once it no longer does
            guardU = u > guardU ? guardU + 0.5f * (u - guardU) : guardU * (xExit >= 0f ? 0.95f : 0.75f);
            // (path straight ahead again: released quickly but smoothly — no jump in the output)
            if (xExit < 0f && Math.abs(ctTY) < 0.5f * lineW) guardU *= 0.6f;
            if (guardU > 0.02f && guardSide != 0f) {
                ctGuard = guardSide * 50f * guardU;
                if (guardSide * ctSteer < 50f * guardU) ctSteer = guardSide * 50f * guardU;
            }
        }
        // ── path statistics ahead of the robot ──
        float sRun = 0f, kMax = 0f, kSigned = 0f, len = 0f, confSum = 0f, wSum = 0f, memLen = 0f, tAllow = 100f;
        ctFresh = 0; ctMem = 0; ctUnrel = 0;
        int lastKnown = -1;
        for (int i = 0; i < n; i++) {
            if (ndSrc[i] == PATH_SRC_FRESH) ctFresh++; else if (ndSrc[i] == PATH_SRC_UNRELIABLE) ctUnrel++; else ctMem++;
        }
        final int i0 = before ? 0 : iR;
        // A point counts as known when measured now, or when it lies between two
        // points measured now (no more than pathMaxGap apart): a hidden stretch with
        // the path visible on both sides is not unknown. Otherwise: its memory confidence.
        final float ds = Math.max(0.02f, cfg.pathStep);
        final int gapMax = Math.max(1, Math.round(cfg.pathMaxGap / ds));
        int prevFresh = -1;
        for (int i = 0; i < n; i++) {
            if (ndSrc[i] == PATH_SRC_FRESH) { prevFresh = i; tmpA[i] = 1f; continue; }
            int next = i + 1; while (next < n && ndSrc[next] != PATH_SRC_FRESH) next++;
            final boolean bracketed = prevFresh >= 0 && next < n && next - prevFresh <= gapMax;
            tmpA[i] = bracketed ? 1f : (ndSrc[i] == PATH_SRC_PREDICTED ? 0.3f * ndConf[i] : Math.min(1f, ndConf[i]));
        }
        for (int i = i0; i < n; i++) {
            if (i > i0) sRun += dist(smX[i] - smX[i - 1], smY[i] - smY[i - 1]);
            if (ndConf[i] >= 0.2f) lastKnown = i;
            if (sRun <= cfg.powerPreview) {
                final float w = ndSrc[i] == PATH_SRC_PREDICTED ? 0.5f : 1f;
                {   // tracking allowed by this point: its curvature, relaxed with its distance
                    float ka = 0f; int kn = 0;
                    for (int q = Math.max(0, i - 2); q <= Math.min(n - 1, i + 2); q++) { ka += smK[q]; kn++; }
                    final float tc = curveTracking(Math.abs(ka / kn) * w);
                    final float s0 = 0.5f, sd = sRun + cfg.pursuitPivotBehind;
                    final float ramp = Math.max(0f, Math.min(1f, (sd - s0) / Math.max(1e-3f, cfg.powerPreview + cfg.pursuitPivotBehind - s0)));
                    if (ndConf[i] >= 0.15f) tAllow = Math.min(tAllow, tc + (100f - tc) * ramp);
                }
                // curvature averaged over ±2 points (the maximum of single noisy values would flicker)
                float ka = 0f; int kn = 0;
                for (int q = Math.max(0, i - 2); q <= Math.min(n - 1, i + 2); q++) { ka += smK[q]; kn++; }
                ka /= kn;
                if (Math.abs(ka) * w > kMax && ndConf[i] >= 0.15f) { kMax = Math.abs(ka) * w; kSigned = ka; }
                confSum += tmpA[i]; wSum += 1f;
                if (ndSrc[i] != PATH_SRC_FRESH && i > i0) memLen += dist(smX[i] - smX[i - 1], smY[i] - smY[i - 1]);
            }
        }
        len = 0f;
        for (int i = i0 + 1; i <= lastKnown; i++) len += dist(smX[i] - smX[i - 1], smY[i] - smY[i - 1]);
        if (before) len += Math.max(0f, smX[0]);
        ctLength = len;
        ctCurvAhead = kSigned;
        // the turn the robot is asked to make now counts too (a robot at an angle to the
        // path must turn hard even where the path itself is straight)
        if (ctTarget) tAllow = Math.min(tAllow, curveTracking(Math.abs(ctKappa)));
        ctTrackCurve = tAllow;
        ctConf = wSum > 0f ? confSum / wSum : 0f;
        ctMemFrac = Math.min(1f, memLen / Math.max(1e-3f, Math.min(cfg.powerPreview, Math.max(sRun, 1e-3f))));
    }

    private static float dist(float dx, float dy) { return (float) Math.sqrt(dx * dx + dy * dy); }

    /**
     * Remembered points that lie on the smooth continuation of the points measured
     * now (a quadratic fitted to them, within 0.35 line widths, at most 1.5 camera
     * heights beyond them) are confirmed by continuity: their confidence is
     * refreshed. Along a straight line the robot's forward motion cannot be seen
     * from the line itself, so a hidden stretch is otherwise never re-measured and
     * would only age.
     */
    private void confirmByContinuity() {
        int m = 0;
        double s0 = 0, s1 = 0, s2 = 0, s3 = 0, s4 = 0, r0 = 0, r1 = 0, r2 = 0;
        float xMin = Float.MAX_VALUE, xMax = -Float.MAX_VALUE;
        for (int i = 0; i < ndN; i++) {
            if (ndSrc[i] != PATH_SRC_FRESH) continue;
            final double x = ndX[i], y = ndY[i];
            s0 += 1; s1 += x; s2 += x * x; s3 += x * x * x; s4 += x * x * x * x;
            r0 += y; r1 += x * y; r2 += x * x * y;
            xMin = Math.min(xMin, ndX[i]); xMax = Math.max(xMax, ndX[i]); m++;
        }
        if (m < 5 || xMax - xMin < 0.5f) return;
        final double det = s0 * (s2 * s4 - s3 * s3) - s1 * (s1 * s4 - s3 * s2) + s2 * (s1 * s3 - s2 * s2);
        if (Math.abs(det) < 1e-9) return;
        final double c0 = cramer(s0, s1, s2, s3, s4, r0, r1, r2, 0, det), c1 = cramer(s0, s1, s2, s3, s4, r0, r1, r2, 1, det),
                     c2 = cramer(s0, s1, s2, s3, s4, r0, r1, r2, 2, det);
        // the fit must describe the measured points well (a path that turns back is not a function of x)
        double rss = 0;
        for (int i = 0; i < ndN; i++) {
            if (ndSrc[i] != PATH_SRC_FRESH) continue;
            final double e = ndY[i] - (c0 + c1 * ndX[i] + c2 * ndX[i] * ndX[i]);
            rss += e * e;
        }
        final float tol = 0.35f * lineW;
        if (Math.sqrt(rss / m) > tol) return;
        for (int i = 0; i < ndN; i++) {
            if (ndSrc[i] == PATH_SRC_FRESH || ndSt[i] == ST_ABSENT) continue;
            final float x = ndX[i];
            if (x < xMin - 1.5f || x > xMax + 1.5f) continue;
            if (Math.abs(ndY[i] - (c0 + c1 * x + c2 * x * x)) <= tol) ndConf[i] = Math.max(ndConf[i], 0.85f);
        }
    }

    /** Tracking target [min … 100] from the computed path statistics. */
    private float trackingTarget() {
        final float tMin = cfg.trackingMinValue;
        float t = ctTrackCurve;
        if (cfg.trackLengthSlowdown && pathEnd == END_ABSENT) {
            // linear in the known length: 100 at trackLengthFullFrac of the view's
            // depth, the minimum when the end reaches the bottom of the view
            final float full = Math.max(roiNearFwd + 1e-3f, cfg.trackLengthFullFrac * roiFarFwd);
            t = Math.min(t, 100f * Math.max(0f, Math.min(1f, (ctLength - roiNearFwd) / (full - roiNearFwd))));
        }
        if (ctConf < 0.5f) t = tMin + (t - tMin) * Math.max(0f, ctConf) / 0.5f;
        t = Math.max(tMin, Math.min(100f, t));
        // light smoothing of the target (the path is re-measured every frame)
        ctCurvAheadPrev += 0.3f * (t - ctCurvAheadPrev);
        if (t <= tMin + 0.5f && ctCurvAheadPrev < tMin + 2f) ctCurvAheadPrev = tMin;
        return ctCurvAheadPrev;
    }

    /** Tracking for curvature k (1/camera height): 100 up to the deadband, minimum at powerFullCurvature. */
    private float curveTracking(float k) {
        final float tMin = cfg.trackingMinValue;
        final float k0 = cfg.powerCurvatureDeadband, k1 = Math.max(k0 + 1e-3f, cfg.powerFullCurvature);
        final float c = Math.max(0f, Math.min(1f, (k - k0) / (k1 - k0)));
        return 100f - (100f - tMin) * c;
    }

    /** Reliability grid: 4 samples per cell over the ROI; a cell is unreliable if ≥ 2 match neither colour. */
    private void buildReliabilityGrid() {
        final int GC = RELIABILITY_GRID_COLS, GR = RELIABILITY_GRID_ROWS;
        final float cw = imgW / (float) GC, ch = (roiBot - roiTop) / (float) GR;
        final float r = lightRange();
        int cnt = 0;
        float gl = 1f;
        for (int gy = 0; gy < GR; gy++) {
            for (int gx = 0; gx < GC; gx++) {
                int nU = 0, nG = 0;
                for (int s = 0; s < 4; s++) {
                    final int col = (int) ((gx + 0.25f + 0.5f * (s & 1)) * cw);
                    final int row = roiTop + (int) ((gy + 0.25f + 0.5f * (s >> 1)) * ch);
                    readPixel(col, row);
                    gridY[s] = sY; gridU[s] = sU; gridV[s] = sV;
                    final float g = floorGain(sY, sU, sV);
                    if (g > 0f) gridG[nG++] = g;
                }
                // local light level of the cell (its floor-like samples), else the previous cell's
                if (nG > 0) gl = median(gridG, nG);
                for (int s = 0; s < 4; s++) {
                    final byte c = classify(gridY[s], gridU[s], gridV[s], gl / r, gl * r, gl);
                    if (c == CL_UNK) nU++;
                }
                final byte v = (byte) (nU >= 2 ? 1 : 0);
                relGrid[gy * GC + gx] = v;
                cnt += v;
            }
        }
        relCells = cnt;
    }

    /** One v3 frame: trace, control, memory update. Writes the internal output holder. */
    private void processV3() {
        v3Rejected = 0;
        memAbsent = false;
        buildReliabilityGrid();
        final boolean haveMem = cfg.pathMemory && memN >= 2;
        if (haveMem) predictMemory();
        boolean ok = haveMem && traceWithMemory();
        if (ok && v3Pinned && v3LostFrames > cfg.trackLostGraceFrames) {
            // Lost (not merely hidden) for a while and the memory confirms nothing:
            // search the whole view; a found line replaces the memory.
            boolean any = false;
            for (int i = 0; i < ndN && !any; i++) any = ndSt[i] == ST_MEAS;
            if (!any) ok = false;
        }
        if (ok) {
            // (Almost) nothing of the remembered path confirmed this frame and clean floor
            // where it should be: maybe the line moved more than expected. A fresh search decides: a
            // well-measured path found elsewhere replaces the memory; otherwise the
            // memory is kept (hidden line).
            // Only when there is evidence AGAINST the memory (clean floor where it says
            // the line is, or a full line-like run just beside it: the line moved); a
            // path that is merely out of view or unreadable is kept.
            int conf = 0, against = 0;
            for (int i = 0; i < ndN; i++) {
                if (ndSt[i] == ST_MEAS) conf++;
                else if (ndSt[i] == ST_ABSENT || (ndSt[i] == ST_UNREL && ndQ[i] < 0f)) against++;
            }
            if (conf < 3 && against >= 3) {
                saveNodes();
                final float cx = corrX, cy = corrY, ct = corrT;
                final int pe = pathEnd;
                if (acquire() && countFresh() >= 4) { corrX = 0f; corrY = 0f; corrT = 0f; motionMeasured = false; }
                else { restoreNodes(); corrX = cx; corrY = cy; corrT = ct; pathEnd = pe; }
            }
        }
        if (ok) {
            // The remembered path confirmed nowhere for a few frames (e.g. it drifted into
            // a glare / shadow region): a line that is actually measured elsewhere in the
            // view replaces it — an unconfirmed memory does not outweigh a real
            // measurement. A line that is merely hidden finds nothing and keeps its memory.
            int meas = 0;
            for (int i = 0; i < ndN; i++) if (ndSt[i] == ST_MEAS) meas++;
            if (meas > 0) v3BlindFrames = 0;
            else if (++v3BlindFrames >= 3) {
                saveNodes();
                final float cx = corrX, cy = corrY, ct = corrT;
                final int pe = pathEnd;
                final boolean mm = motionMeasured;
                // direction of the remembered path (its first → last point)
                final float mdx = svX[svN - 1] - svX[0], mdy = svY[svN - 1] - svY[0];
                if (acquire() && countFresh() >= 5 && sameDirection(mdx, mdy, ndX[ndN - 1] - ndX[0], ndY[ndN - 1] - ndY[0])) {
                    corrX = 0f; corrY = 0f; corrT = 0f; motionMeasured = false; v3BlindFrames = 0;
                }
                else { restoreNodes(); corrX = cx; corrY = cy; corrT = ct; pathEnd = pe; motionMeasured = mm; }
            }
        }
        if (!ok) { corrX = 0f; corrY = 0f; corrT = 0f; motionMeasured = false; ok = acquire(); }
        int fresh = 0;
        if (ok) {
            resamplePath();
            for (int i = 0; i < ndN; i++) if (ndSrc[i] == PATH_SRC_FRESH) fresh++;
            if (ndN >= 2) { confirmByContinuity(); smoothPath(); computeControl(); } else ok = false;
        }
        // ── found / hidden / lost ──
        final boolean found = ok && fresh >= 2;
        float steering, tracking;
        boolean held = false, pinned = false;
        if (found) {
            v3LostFrames = 0; v3HoldFrames = 0;
            lastEndAbsent = pathEnd == END_ABSENT;
            steering = ctSteer;
            final float tt = trackingTarget();
            v3Tracking += Math.max(-cfg.powerFallRate, Math.min(cfg.powerRiseRate, tt - v3Tracking));
            tracking = v3Tracking;
        } else {
            v3LostFrames++;
            // Why is nothing measured? Count what the remembered path shows now.
            int nOut = 0, nAbs = 0, nUnr = 0;
            for (int i = 0; i < ndN; i++) {
                if (ndSt[i] == ST_OUT) nOut++; else if (ndSt[i] == ST_ABSENT) nAbs++; else if (ndSt[i] == ST_UNREL) nUnr++;
            }
            // Path ahead of the robot that left the view at the BOTTOM (into the blind
            // zone in front of the robot, e.g. a sharp corner just before the robot)
            // versus through a SIDE edge (the robot is turning off the line).
            int nBelow = 0, nSide = 0;
            for (int i = 0; i < ndN; i++) {
                if (ndSt[i] != ST_OUT || ndX[i] < 0f || !planeToImage(ndX[i], ndY[i])) continue;
                if (pU < 0f || pU > imgW) nSide++; else if (pV >= roiTop) nBelow++;
            }
            final boolean blind = ok && nAbs == 0 && nBelow > 0 && nBelow >= nSide
                    && v3LostFrames <= cfg.pathMemoryFrames;
            final boolean hidden = ok && ((nUnr > 0 && nUnr >= nAbs) || blind) && v3LostFrames <= cfg.pathMemoryFrames;
            final boolean midImage = ((ok && nAbs > 0 && nAbs >= nOut) || (!ok && memAbsent)) && v3HoldFrames < cfg.trackHoldFrames;
            if (hidden) {
                // covered by a disturbance: follow the remembered path, confidence fades slowly
                steering = ctSteer;
                final float tt = trackingTarget();
                v3Tracking += Math.max(-cfg.powerFallRate, Math.min(cfg.powerRiseRate, tt - v3Tracking));
                tracking = v3Tracking;
                held = true;
            } else if (midImage) {
                v3HoldFrames++;
                // approaching a dead end (the line's end was seen): keep slowing down
                if (lastEndAbsent) v3Tracking = Math.max(cfg.trackingMinValue, v3Tracking - cfg.powerFallRate);
                steering = v3LastSteer; tracking = v3Tracking; held = true;
            } else {
                // left the view (or gone for good): full turn toward the side it was last on
                // the side the remembered path leads to (its farthest point ahead), else the target
                // the side the robot was already turning to (as v2), else where the
                // remembered path leads (its farthest point ahead), else the target
                float side = v3Pinned || Math.abs(v3LastSteer) >= 10f ? Math.signum(v3LastSteer) : 0f;
                for (int i = ndN - 1; i >= 0 && side == 0f; i--) if (ndX[i] > 0f && Math.abs(ndY[i]) > 0.5f * lineW) side = Math.signum(ndY[i]);
                if (side == 0f) side = Math.signum(ctTarget ? ctTY : 0f);
                if (side == 0f) side = pathExitSide != 0f ? pathExitSide : Math.signum(v3LastSteer);
                if (side == 0f) side = 1f;
                steering = 50f * side;
                tracking = cfg.trackingMinValue;
                pinned = true;
                v3Tracking = cfg.trackingMinValue;
                pathExitSide = side;
            }
        }
        if (found || held || pinned) v3LastSteer = steering;
        // (a short loss of the line in a curve — a full turn toward it — counts too: the
        // path curvature is still the last one measured)
        if ((found || held || (pinned && v3LostFrames <= 15)) && cfg.steerAutoGain) updateAutoGain(steering);
        v3Pinned = pinned;
        // ── motion estimate and memory for the next frame ──
        if (ok && haveMem && motionMeasured) {
            // Kalman update: the correction is the error of the predicted motion. The
            // positions were corrected in full this frame; the motion predicted for the
            // next frame changes at most as fast as a robot can accelerate (one odd
            // frame — glare, a flicker — cannot throw the memory off).
            velX = Math.max(-0.1f, Math.min(VEL_X_MAX, velX - Math.max(-MOTION_DX, Math.min(MOTION_DX, corrX))));
            velT = Math.max(-VEL_T_MAX, Math.min(VEL_T_MAX, velT - Math.max(-MOTION_DT, Math.min(MOTION_DT, corrT))));
            velY = velT * cfg.pursuitPivotBehind;
            mPxx = kPxx; mPxt = kPxt; mPtt = kPtt;
        } else if (!ok) {
            velX *= 0.95f; velY *= 0.9f; velT *= 0.9f;
        }
        if (ok) {
            memN = ndN;
            for (int i = 0; i < ndN; i++) { memX[i] = ndX[i]; memY[i] = ndY[i]; memConf[i] = Math.min(1f, ndConf[i]); memAge[i] = ndAge[i]; memVar[i] = ndVar[i]; }
            if (cfg.pathMemory && !found) {
                // nothing confirmed now: the memory fades (it is the only knowledge)
                for (int i = 0; i < memN; i++) memConf[i] *= 0.97f;
            }
        } else if (v3LostFrames > cfg.trackLostGraceFrames) {
            memN = 0;          // gone for good: search from scratch
        }
        v3Found = found; v3Held = held;
        lastMotionMeasured = ok && haveMem && motionMeasured;
        // ── internal output (same holder as v2; internal steering is ±100) ──
        output.steering     = 2f * steering;
        output.intersection = 100f - tracking;
        output.trackFound   = found;
        output.held         = held;
        output.rejected     = v3Rejected;
        output.threshold    = 128;
        output.headingDeg   = found || held ? ctHeading : 0f;
        output.headingConf  = found || held ? ctConf : 0f;
        final float offPart = ctTarget ? cfg.steerPerCurvature * 2f * ctOffset / Math.max(1e-3f, cfg.pursuitLookAhead * cfg.pursuitLookAhead) : 0f;
        output.offsetTerm   = found ? 2f * Math.max(-50f, Math.min(50f, offPart)) : 0f;
        output.headingTerm  = found ? 2f * steering - output.offsetTerm : 0f;
        // near-line column at the v2 strip row (legacy FrameData.centroidXFrac)
        int cx = -1;
        if (found) {
            final float hr = (hRoiTop + hRoiBot) * 0.5f;
            for (int i = 0; i < ndN - 1; i++) {
                if (!planeToImage(smX[i], smY[i])) continue;
                final float u0 = pU, v0 = pV;
                if (!planeToImage(smX[i + 1], smY[i + 1])) continue;
                if ((v0 - hr) * (pV - hr) <= 0f && Math.abs(pV - v0) > 1e-3f) {
                    cx = Math.round(u0 + (hr - v0) / (pV - v0) * (pU - u0)); break;
                }
            }
            if (cx < 0 && ndN > 0 && planeToImage(smX[0], smY[0])) cx = Math.round(pU);
            if (cx >= 0) { lastCentroidX = cx; }
            output.tiltDeg = 0f;
            if (ndN >= 2 && planeToImage(smX[0], smY[0])) {
                final float u0 = pU, v0 = pV;
                final int j = Math.min(ndN - 1, 6);
                if (planeToImage(smX[j], smY[j]) && v0 - pV > 1f) output.tiltDeg = (float) Math.toDegrees(Math.atan2(pU - u0, v0 - pV));
            }
            lastTrackWidthPx = Math.max(1f, calWA + calWB * hr);
        }
        output.centroidX    = found ? cx : -1;
        output.trackWidthPx = lastTrackWidthPx;
        // overlay poly-line of the legacy look-ahead arrays: the path (all sources)
        laPathCount = 0;
        if (ok) {
            final int stepIdx = Math.max(1, (ndN + LA_MAX_PTS - 1) / LA_MAX_PTS);
            for (int i = 0; i < ndN && laPathCount < LA_MAX_PTS; i += stepIdx) {
                if (!planeToImage(smX[i], smY[i])) continue;
                laPathColFrac[laPathCount] = pU / imgW; laPathRowFrac[laPathCount] = pV / imgH; laPathCount++;
            }
        }
    }

    /** v3 per-frame driver (after calibration). */
    private int[] processBoundV3() {
        updateFps();
        processV3();
        // Whole-number outputs with a little hysteresis: a value only changes when the
        // computed one has moved at least 0.8 away from it (no flipping between two
        // neighbouring numbers on a steady path). The ends (±50, minimum / 100) are
        // reached exactly.
        final float sRaw = output.steering * 0.5f, tRaw = 100f - output.intersection;
        if (Math.abs(sRaw - outSteer) >= OUT_HYST || Math.abs(sRaw) >= 49.5f) outSteer = Math.round(sRaw);
        if (Math.abs(tRaw - outTrack) >= OUT_HYST || tRaw >= 99.5f || tRaw <= cfg.trackingMinValue + 0.5f) outTrack = Math.round(tRaw);
        result[0] = Math.max(-50, Math.min(50, outSteer));
        result[1] = Math.max(cfg.trackingMinValue, Math.min(100, outTrack));
        isProcessing = true;
        fireCallback(result[0], result[1], output.trackFound, output.centroidX);
        isProcessing = false;
        return result;
    }

    /** Copy the v3 fields into FrameData. */
    private void fillPathFrameData(FrameData d) {
        d.pathMode = cfg.pathTracking;
        d.roiTopFrac = (cfg.pathTracking && v3Ready ? roiTop : vRoiTop) / (float) imgH;
        d.roiBotFrac = (cfg.pathTracking && v3Ready ? roiBot : hRoiBot) / (float) imgH;
        d.cameraTiltUsedDeg = v3Ready ? gTiltDeg : cfg.cameraTiltDeg;
        d.lineWidthFloor = v3Ready ? lineW : 0f;
        int np = 0;
        if (cfg.pathTracking && !calibrated) {
            // calibration: rows where the line was found (image coordinates)
            for (int i = 0; i < ndN && np < PATH_MAX_POINTS; i++) {
                d.pathColFrac[np] = ndX[i] / imgW; d.pathRowFrac[np] = ndY[i] / imgH;
                d.pathFwd[np] = 0f; d.pathLat[np] = 0f; d.pathSource[np] = PATH_SRC_FRESH; np++;
            }
        } else if (cfg.pathTracking && v3Ready && isProcessing) {
            final int stepIdx = Math.max(1, (ndN + PATH_MAX_POINTS - 1) / PATH_MAX_POINTS);
            for (int i = 0; i < ndN && np < PATH_MAX_POINTS; i += stepIdx) {
                if (!planeToImage(ndX[i], ndY[i])) continue;
                d.pathColFrac[np] = pU / imgW; d.pathRowFrac[np] = pV / imgH;
                d.pathFwd[np] = ndX[i]; d.pathLat[np] = ndY[i]; d.pathSource[np] = ndSrc[i]; np++;
            }
        }
        d.pathPointCount = np;
        final boolean live = cfg.pathTracking && v3Ready && isProcessing && ndN > 0;
        d.pathFreshPoints = live ? ctFresh : 0;
        d.pathMemoryPoints = live ? ctMem : 0;
        d.pathUnreliablePoints = live ? ctUnrel : 0;
        d.pathMemoryFrac = live ? ctMemFrac : 0f;
        d.lookAheadValid = live && ctTarget;
        d.lookAheadFwd = live ? ctTX : 0f; d.lookAheadLat = live ? ctTY : 0f;
        if (live && ctTarget && planeToImage(ctTX, ctTY)) { d.lookAheadXFrac = pU / imgW; d.lookAheadYFrac = pV / imgH; }
        else { d.lookAheadXFrac = -1f; d.lookAheadYFrac = -1f; }
        d.pathCurvature = live ? ctCurv : 0f;
        d.pathCurvatureAhead = live ? ctCurvAhead : 0f;
        d.pathOffset = live ? ctOffset : 0f;
        d.pathHeadingDeg = live ? ctHeading : 0f;
        d.pathLength = live ? ctLength : 0f;
        d.pathConfidence = live ? (v3Found ? ctConf : ctConf * 0.5f) : 0f;
        if (cfg.pathTracking && v3Ready && isProcessing) {
            System.arraycopy(relGrid, 0, d.unreliableGrid, 0, relGrid.length);
            d.unreliableCells = relCells;
        } else {
            java.util.Arrays.fill(d.unreliableGrid, (byte) 0);
            d.unreliableCells = 0;
        }
        d.motionFwd = live ? velX : 0f; d.motionLat = live ? velY : 0f;
        d.motionYawDeg = live ? (float) Math.toDegrees(velT) : 0f;
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
