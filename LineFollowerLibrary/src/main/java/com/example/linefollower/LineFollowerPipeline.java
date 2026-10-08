package com.example.linefollower;

/**
 * Standalone line-following processor for the ROBOPHONE environment.
 * No Android or camera dependencies — accepts raw pixel arrays directly.
 *
 * Usage:
 *   LineFollowerPipeline pipeline = new LineFollowerPipeline();
 *   while (!pipeline.calibrateYuv420(y, u, v, width, height)) { ...next frame... }
 *   int[] result = pipeline.processFrameYuv420(y, u, v, width, height);
 *   // result[0] = Steering  [-100 … +100]  (negative = line left, positive = line right)
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
 *      band's last position is searched, so reflections elsewhere are ignored and
 *      the band is followed to the frame edge → steering [-100 … +100]. When the
 *      band finally leaves the gate, steering pins to that edge (±100) and a new
 *      line is re-acquired (full-width) only after a short grace period.
 *   4. V-ROI look-ahead → tracking value (100 = straight path ahead).
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

        // Minimum value the Tracking output is allowed to reach (default 2).
        // Prevents the robot from receiving a full-stop signal, which can cause
        // mechanical bugs. Set to 0 to allow the full 0-100 range.
        public int trackingMinValue = 2;

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
        /** Steering value [-100 … +100]; negative = line left, positive = line right. */
        public int steering;
        /** Tracking value [min … 100]; 100 = straight road ahead, low = turn / line lost. */
        public int tracking;
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

    // Pre-allocated per-column "track pixel count" profile of the H-ROI (sized in
    // init). Lets Stage 2 isolate the strongest contiguous BAND instead of
    // averaging every track pixel, which is what makes the line-lock possible.
    private int[] colWeight;

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
     *         [0] = Steering  [-100 … +100]  negative = line is left, positive = line is right
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

        rowCx             = new float[height];
        colWeight         = new int[width];
        work              = new byte[width * height];
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
            final byte[] pix = prepareWorkImage(hRoiTop, hRoiBot);
            final int threshold = stripThreshold(pix, workBright);
            output.threshold = threshold;
            buildColumnProfile(pix, threshold, workBright);
            // Prefer the band where colour learning saw the line (else image centre),
            // so a shadow edge or a neighbouring tape is not taken as the line.
            final float prefX = learnedLineW > 0f ? learnedLineX : imgW * 0.5f;
            final double minWeight = (double) cfg.minWeightFrac * Math.max(1, hRoiBot - hRoiTop) * imgW;
            findBand(0, imgW, minWeight, prefX, imgW * 0.25f, learnedLineW);

            if (bandStart >= 0) {
                lineFound = true;
                calibIssue = ISSUE_NONE;
                centroidX = (int) (bandMoment / bandSum);
                lastCentroidX    = centroidX;
                lastTrackWidthPx = Math.max(1, bandEnd - bandStart);
                if (calibCount < calibSteerSamples.length) {
                    calibSteerSamples[calibCount] = centroidX;
                    calibWidthSamples[calibCount] = bandEnd - bandStart;
                }
                if (++calibCount >= Math.max(1, cfg.calibFrames)) {
                    final int n = Math.min(calibCount, calibSteerSamples.length);
                    java.util.Arrays.sort(calibSteerSamples, 0, n);
                    java.util.Arrays.sort(calibWidthSamples, 0, n);
                    steerRefX    = calibSteerSamples[n / 2];                // robust median
                    calibWidthPx = calibWidthSamples[n / 2];
                    trackRefX    = imgW * 0.5f + (steerRefX - imgW * 0.5f) * laPerspectiveRatio;
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
        final byte[] pix = prepareWorkImage(vRoiTop, hRoiBot);
        runPipeline(pix, imgW, imgH, workBright);
        result[0] = Math.round(output.steering);
        // Invert the internal intersection scale:
        //   internal 0   (straight) → output 100 (no braking needed)
        //   internal 100 (hard turn / lost) → output min (brake / slow down)
        result[1] = Math.max(cfg.trackingMinValue, Math.round(100f - output.intersection));
        fireCallback(result[0], result[1], output.trackFound, output.centroidX);
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

    /** Collapse the H-ROI into a per-column count of track pixels (colWeight). */
    private void buildColumnProfile(byte[] pixels, int threshold, boolean bright) {
        final int W = imgW;
        final int[] colW = colWeight;
        java.util.Arrays.fill(colW, 0, W, 0);
        if (threshold == NO_TRACK) return;
        for (int row = hRoiTop; row < hRoiBot; row++) {
            final int base = row * W;
            for (int col = 0; col < W; col++) {
                final int px = pixels[base + col] & 0xFF;
                if (bright ? (px >= threshold) : (px <= threshold)) colW[col]++;
            }
        }
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
    private void findBand(int lo, int hi, double minWeight, float prefX, float scale, float expectW) {
        final int[] colW = colWeight;
        final int runGapTol = runGapTol();
        long   curSum = 0;
        double curMoment = 0;
        int    curStart = -1, curLast = -1, gapCols = 0;
        bandSum = 0; bandMoment = 0; bandStart = -1; bandEnd = -1; bandScore = -1f;
        for (int col = lo; col <= hi; col++) {
            final int w = (col < hi) ? colW[col] : 0;
            if (w > 0) {
                if (curStart < 0) { curStart = col; curSum = 0; curMoment = 0; }
                curSum += w; curMoment += (double) col * w; gapCols = 0; curLast = col;
            } else if (curStart >= 0 && (col == hi || ++gapCols > runGapTol)) {   // run ends
                if (curSum >= minWeight) {
                    final float s = scoreBand(curSum, (float) (curMoment / curSum),
                                              curLast + 1 - curStart, prefX, scale, expectW);
                    if (s > bandScore) {
                        bandScore = s; bandSum = curSum; bandMoment = curMoment;
                        bandStart = curStart; bandEnd = curLast + 1;
                    }
                }
                curStart = -1;
            }
        }
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
        final int   rowStride = width;
        final float halfW = W * 0.5f;

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
        // track. With track memory on we only scan a gate around the band's last
        // known position, so spots outside the gate are ignored and steering can
        // run all the way to the edge. The lock is held until the line has been
        // gone for trackLostGraceFrames (you have moved fully off it); only then do
        // we search the whole width for a new line.
        buildColumnProfile(pixels, threshold, bright);

        final int    hRoiRows  = Math.max(1, hRoiBot - hRoiTop);
        final double minWeight = (double) cfg.minWeightFrac * hRoiRows * W;

        // Search window: a gate around the remembered line when locked, otherwise
        // the full width (fresh start, or re-acquiring after the line was lost).
        final boolean locked = cfg.trackMemory && trackAcquired;
        int gateHalf = W;
        if (locked) {
            gateHalf = Math.max(cfg.trackMemoryMinGatePx,
                                Math.round(cfg.trackMemoryGateMult * lastTrackWidthPx));
            gateHalf = Math.min(gateHalf, W / 2);
        }
        final int searchLo = locked ? Math.max(0, lastCentroidX - gateHalf) : 0;
        final int searchHi = locked ? Math.min(W, lastCentroidX + gateHalf) : W;
        // Candidate scoring (see scoreBand):
        //   • locked    — inside the gate, prefer the band nearest the last position.
        //   • searching — across the frame, prefer a band near the calibrated
        //                 straight-ahead column with the calibrated line width, so a
        //                 reflection or a neighbouring tape is not grabbed just
        //                 because it happens to be the biggest thing in view.
        final float expectW    = expectedLineWidth();
        final float lineScale  = W * 0.25f;
        if (locked) findBand(searchLo, searchHi, minWeight, lastCentroidX, gateHalf, 0f);
        else        findBand(0, W, minWeight, steerRefX, lineScale, expectW);
        final int runGapTol = runGapTol();

        // Lock correction: while locked, also look at the whole strip. If a band that
        // is clearly a better line candidate (by the "searching" score) sits outside
        // the locked band for trackRelockFrames frames in a row, the lock was on the
        // wrong thing — move it there.
        if (locked && bandStart >= 0 && cfg.trackRelockFrames > 0) {
            final long   lockSum   = bandSum;
            final double lockMom   = bandMoment;
            final int    lockStart = bandStart, lockEnd = bandEnd;
            final float  lockScore = scoreBand(lockSum, (float) (lockMom / lockSum),
                                               lockEnd - lockStart, steerRefX, lineScale, expectW);
            findBand(0, W, minWeight, steerRefX, lineScale, expectW);
            final boolean better = bandStart >= 0
                    && (bandEnd <= lockStart || bandStart >= lockEnd)
                    && bandScore > RELOCK_RATIO * lockScore;
            if (better && ++relockCount >= cfg.trackRelockFrames) {
                relockCount = 0;                          // adopt the candidate (already in band*)
            } else {
                if (!better) relockCount = 0;
                bandSum = lockSum; bandMoment = lockMom; bandStart = lockStart; bandEnd = lockEnd;
            }
        } else {
            relockCount = 0;
        }

        boolean trackFound;
        int     centroidX;
        float   forcedIntersection = -1f;   // ≥ 0 overrides the look-ahead stage
        float   pinnedSteering     = 0f;    // ±100 while the locked line is lost

        if (bandStart >= 0) {
            // Locked onto the line this frame.
            trackFound       = true;
            centroidX        = (int) (bandMoment / bandSum);
            lastCentroidX    = centroidX;
            lastTrackWidthPx = Math.max(1, bandEnd - bandStart);
            trackAcquired    = true;
            lostFrames       = 0;
        } else if (locked) {
            // The line we were following just left the gate → you have moved fully
            // off it. Pin steering hard toward the edge it exited and flag a 100 %
            // turn. Keep the gate parked at that edge so the SAME line re-locks if
            // it comes back; only after the grace period do we hunt a new one.
            trackFound  = false;
            lostFrames++;
            // Pin toward the edge it exited, RELATIVE to the calibrated centre. The
            // steering is set to exactly ±100 below (the clamped pixel column alone
            // would read less than 100 when the calibrated centre is off-centre).
            final boolean onRight = lastCentroidX >= Math.round(steerRefX);
            centroidX     = onRight ? Math.min(W - 1, Math.round(steerRefX + halfW))
                                    : Math.max(0,     Math.round(steerRefX - halfW));
            lastCentroidX = centroidX;
            pinnedSteering     = onRight ? 100f : -100f;
            forcedIntersection = 100f;
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

        // ── Stage 3 threshold: adaptive to the (dimmer, farther) look-ahead ───
        // The far look-ahead line is dimmer than the near line, so the H-ROI Otsu
        // can miss it — the tracker then loses the line early and the track value
        // plateaus. Build a subsampled histogram of the whole V-ROI band and run
        // Otsu again to get a threshold matched to the look-ahead's own lighting.
        int vThreshold = threshold;
        if (cfg.vRoiAdaptiveThreshold && vRoiBot > vRoiTop) {
            final int[] histV = histogramV;
            java.util.Arrays.fill(histV, 0);
            final int stepR = Math.max(1, (vRoiBot - vRoiTop) / 240);   // ~240 rows sampled
            final int stepC = Math.max(1, W / 240);                     // ~240 cols sampled
            long cntV = 0;
            for (int row = vRoiTop; row < vRoiBot; row += stepR) {
                final int base = row * rowStride;
                for (int col = 0; col < W; col += stepC) {
                    histV[pixels[base + col] & 0xFF]++; cntV++;
                }
            }
            final int t = finishThreshold(otsuThreshold(histV, cntV), bright);
            if (t != NO_TRACK) vThreshold = t;   // flat look-ahead → keep the strip threshold
        }

        // ── Stage 3: look-ahead along the line → intersection ──────────────────
        //
        // We TRACK the line up through the V-ROI, recentring the scan window on
        // the previous row's centroid, so curves and a skewed approach are
        // followed instead of leaking out of a fixed column. Only rows where the
        // line is actually VISIBLE are used; empty far-away rows are never counted.
        //
        final int     half   = cfg.vRoiHalfWidthPx;
        final float[] rowCx  = this.rowCx;
        final int     maxGap = Math.max(8, cfg.hRoiThickPx);   // stop after this many empty rows

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

        // Least-squares accumulators over rows with a detected centroid (y = row).
        int    fitN = 0;
        double sY = 0.0, sYY = 0.0, sC = 0.0, sYC = 0.0;

        // Anchor the fit with the H-ROI centroid (longest, most reliable baseline).
        if (trackFound) {
            final double yH = (hRoiTop + hRoiBot) * 0.5;
            fitN++; sY += yH; sYY += yH * yH; sC += centroidX; sYC += yH * centroidX;
        }

        // 3a. Track the line from the bottom of the V-ROI upward, following drift.
        //     Each row we lock onto the contiguous RUN closest to the line's
        //     position on the row below — never the average of everything in the
        //     window — and reject any run that is too small or too far (a different
        //     object). Stop once the line has been missing for maxGap rows (end /
        //     off the road), so empty frame above is never mistaken for the track.
        int trackCenter = centroidX;
        int gap         = 0;
        int farRow      = -1;   // farthest (topmost) V-ROI row where the line was accepted
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
            final int vLeft  = Math.max(0,     trackCenter - half);
            final int vRight = Math.min(width, trackCenter + half);
            final int base   = row * rowStride;

            // Scan the window, picking the run whose centre is nearest the
            // predicted centre (trackCenter). Small gaps inside a run are bridged.
            long   runW = 0;   double runM = 0;   int runStart = -1, runGap = 0;
            double bestC = Double.NaN; long bestW = 0; double bestDist = Double.MAX_VALUE;
            for (int col = vLeft; col < vRight; col++) {
                final int px = pixels[base + col] & 0xFF;
                if (isTrack(px, vThreshold, bright)) {
                    if (runStart < 0) { runStart = col; runW = 0; runM = 0; }
                    runW++; runM += col; runGap = 0;
                } else if (runStart >= 0 && ++runGap > runGapTol) {
                    final double c = runM / runW, d = Math.abs(c - trackCenter);
                    if (d < bestDist) { bestDist = d; bestC = c; bestW = runW; }
                    runStart = -1;
                }
            }
            if (runStart >= 0) {
                final double c = runM / runW, d = Math.abs(c - trackCenter);
                if (d < bestDist) { bestDist = d; bestC = c; bestW = runW; }
            }

            // Accept only a run that is big enough AND close enough to the line we
            // are following; otherwise this row has no usable track → count a gap.
            if (bestW >= minRowW && bestDist <= vMaxDrift) {
                rowCx[row]  = (float) bestC;
                trackCenter = (int) (bestC + 0.5);        // follow the line upward
                gap = 0;
                farRow = row;                             // farthest row the line reaches
                final double laWeight = vRoiBot - row;    // farther ahead → weighted more
                laW += laWeight; laM += laWeight * bestC;
                fitN++; sY += row; sYY += (double) row * row;
                sC += bestC;       sYC += (double) row * bestC;
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
        laPathCount = laCount;   // publish the path for any external overlay

        // 3b. Line tilt from the least-squares slope  b = d(col)/d(row) = tan(θ).
        //     Reported for overlays only — it does not feed the track value.
        //     Rows grow downward, so a line leaning right has a NEGATIVE slope.
        double slope = 0.0;
        if (fitN >= 2) {
            final double denom = (double) fitN * sYY - sY * sY;
            if (Math.abs(denom) > 1e-6) {
                slope = ((double) fitN * sYC - sY * sC) / denom;
            }
        }
        final float tiltDeg = (float) Math.toDegrees(Math.atan(-slope));

        // 3c. Look-ahead line position (band-locked, far-weighted centroid).
        final boolean lookAheadValid = (laW > 0.0);
        final float   lookAheadX     = lookAheadValid ? (float) (laM / laW) : steerRefX;

        // ── Steering: signed offset from the calibrated straight-ahead column ──
        final float steering = (pinnedSteering != 0f) ? pinnedSteering
                : Math.max(-100f, Math.min(100f, (centroidX - steerRefX) / halfW * 100f));

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

        // ── Write internal output ──────────────────────────────────────────────
        output.steering     = steering;
        output.intersection = intersection;
        output.centroidX    = trackFound ? centroidX : -1;
        output.tiltDeg      = trackFound ? tiltDeg : 0f;
        output.trackWidthPx = lastTrackWidthPx;   // measured width, held on track loss
        output.trackFound   = trackFound;
        output.threshold    = threshold;
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
