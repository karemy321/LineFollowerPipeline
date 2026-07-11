package com.example.firstproject;

/**
 * Standalone line-following processor for the ROBOPHONE environment.
 * No Android or camera dependencies — accepts raw grayscale byte arrays directly.
 *
 * Usage:
 *   LineFollowerPipeline pipeline = new LineFollowerPipeline();
 *   int[] result = pipeline.processFrame(grayscaleBytes, width, height);
 *   // result[0] = Steering  [-100 … +100]  (negative = line left, positive = line right)
 *   // result[1] = Tracking  [  0 … 100  ]  (100 = straight path ahead, 0 = hard turn / line lost)
 *
 * Algorithm:
 *   1. Build a grayscale histogram of the Horizontal ROI and run Otsu's method
 *      (from scratch — no OpenCV) to pick the threshold that best separates the
 *      track from the floor under the current lighting.
 *   2. Threshold: pixels above (Otsu ± bias) are "track" (bright = white line).
 *   3. Line-lock: collapse the H-ROI to a per-column white profile and select the
 *      strongest contiguous white BAND. With track memory on, only a gate around
 *      the band's last position is searched, so reflections elsewhere are ignored
 *      and the band is followed to the frame edge → steering [-100 … +100]. When
 *      the band finally leaves the gate, steering pins to that edge (±100) and a
 *      new line is re-acquired (full-width) only after a short grace period.
 *   4. V-ROI look-ahead → tracking value [0 … 100]
 *      (100 = straight path ahead / solidly on road, 0 = hard turn / line lost)
 *
 * All buffers are pre-allocated in init(); processFrame() performs zero heap allocations.
 */
public final class LineFollowerPipeline {

    // =========================================================================
    //  Configuration
    // =========================================================================

    public static final class Config {

        // true  = WHITE line on DARK floor  (your setup)
        // false = DARK  line on LIGHT floor
        public boolean trackIsBright = true;

        // Optional bias applied on top of the Otsu threshold (in grey-levels).
        // Otsu self-calibrates to the lighting, so 0 is the right default. Nudge
        // positive to reject more floor (fewer false positives), negative if the
        // line is being missed. The bias is added toward the track side: for a
        // bright track threshold = Otsu + margin, for a dark track Otsu - margin.
        public int thresholdMargin = 0;

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

        // ── Perspective / mounting geometry ──────────────────────────────────
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

        // ── Track memory / continuity ────────────────────────────────────────
        // When ON, the pipeline LOCKS onto the white band it is currently on and,
        // on every later frame, only looks for the track inside a "gate" window
        // around its last known position. Light reflections or stray bright spots
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

        // ── Track value (look-ahead position) calibration ────────────────────
        // The "track" output is STEERING applied to the LOOK-AHEAD — the purple
        // V-ROI in front of the robot — using the same band-lock memory as
        // steering. It reads 0 when the upcoming line sits straight ahead
        // (centred), rises as the line ahead leans to the side (an upcoming turn),
        // and hits 100 once it reaches the frame edge or is lost (you have moved
        // off the road). trackOffsetFullFrac sets how far the look-ahead line must
        // lean, as a fraction of the half-frame width, to read 100: 1.0 = only at
        // the very frame edge; lower it (e.g. 0.6) to make turns read higher sooner.
        public float trackOffsetFullFrac = 1.0f;

        // ── Auto-calibration (camera mounting offset) ────────────────────────
        // The phone camera is not on the robot's centre line, so when the robot is
        // correctly placed on the road the NEAR line does NOT sit at image centre.
        // We ASSUME the robot starts correctly placed and, over the first few solid
        // detections, record where the near line (steering) actually sits — that
        // becomes the steering zero reference, so a correctly placed robot reads
        // steering 0. The look-ahead (track) zero is NOT measured from the road
        // (on a curve the road ahead genuinely leans, so measuring it would wrongly
        // flatten the turn to 0); instead it is DERIVED from the steering offset
        // through the camera's perspective geometry, capturing only the mounting
        // offset. So a straight road reads track ~0 while a circle reads a real
        // turn. The steering reference is taken as the MEDIAN of this many solid
        // detections (more = steadier / more outlier-proof capture, slower startup).
        public int calibFrames = 10;
    }

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
    }

    // =========================================================================
    //  State
    // =========================================================================

    private Config  cfg         = new Config();   // default config until configure() is called
    private boolean initialized;

    private int imgW, imgH;
    private int hRoiTop, hRoiBot;  // H-ROI row range [hRoiTop, hRoiBot)
    private int vRoiTop, vRoiBot;  // V-ROI row range [vRoiTop, vRoiBot)
    private int lastCentroidX;
    private float lastTrackWidthPx; // most recent measured track width (held on loss)

    // ── Track memory (line-lock) state ───────────────────────────────────────
    // trackAcquired: we currently hold a lock on a specific white band and only
    //   search a gate around lastCentroidX. lostFrames: consecutive frames the
    //   locked line has been absent from the gate; once it exceeds the configured
    //   grace the lock is dropped and the next frame searches the full width.
    private boolean trackAcquired;
    private int     lostFrames;

    // ── Auto-calibration state (camera mounting offset) ──────────────────────
    // steerRefX / trackRefX are the image columns that count as "straight ahead"
    // for the near (steering) and look-ahead (track) measurements — captured once
    // by averaging the first calibFrames solid detections, assuming the robot
    // starts correctly placed. Until then they default to image centre.
    private boolean calibrated;
    private int     calibCount;
    private int[]   calibSteerSamples;   // near-line centroids gathered for the median
    private float   steerRefX, trackRefX;

    // ── Dynamic look-ahead path (for the overlay to follow the road) ─────────
    // A small poly-line of the detected line's centre, sampled from nearest to
    // farthest across the V-ROI, in FRACTIONAL image coordinates. Rebuilt each
    // frame from the same band-locked tracking that drives the track value, so
    // the purple overlay bends along the tape instead of drawing a straight box.
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

    // ── Frames-per-second measurement (perf tracking) ────────────────────────
    // Counts frames within each rolling 1-second wall-clock window; the count
    // from the most recently COMPLETED second is exposed as the current FPS.
    private long fpsWindowStartNs;   // start timestamp of the in-progress window
    private int  fpsFrameCount;      // frames seen so far in the in-progress window
    private int  framesPerSecond;    // frames counted in the last completed second
    private long totalFrames;        // cumulative frames processed since init()
    private long firstFrameNs;       // timestamp of the very first processed frame
    private long elapsedNs;          // wall-clock nanos since the first frame

    // Pre-allocated 256-bin grayscale histogram, reused every frame (no GC churn).
    private final int[] histogram = new int[256];

    // Second histogram for the look-ahead (V-ROI) Otsu — the far region is dimmer,
    // so it gets its own threshold. Reused every frame (no allocation).
    private final int[] histogramV = new int[256];

    // Pre-allocated per-row centroid buffer for the V-ROI tilt fit (sized in init).
    private float[] rowCx;

    // Pre-allocated per-column "white pixel count" profile of the H-ROI (sized in
    // init). Lets Stage 2 isolate the strongest contiguous white BAND instead of
    // averaging every bright pixel, which is what makes the line-lock possible.
    private int[] colWeight;

    private final ControlOutput output = new ControlOutput();

    // Reused output array — no allocation on the hot path.
    private final int[] result = new int[2];

    // =========================================================================
    //  Public API
    // =========================================================================

    /**
     * Optionally set a custom configuration before the first processFrame() call.
     * If not called, default Config values are used automatically.
     *
     * @param config  pipeline tuning parameters
     */
    public void configure(Config config) {
        this.cfg = config;
        initialized = false;   // force re-init with the new config on next frame
    }

    /**
     * Process one raw grayscale frame and return the two robot control values.
     *
     * The pipeline auto-initialises on the first call (or whenever the frame
     * dimensions change). Call {@link #configure(Config)} before the first frame
     * if you need non-default settings; otherwise the built-in defaults are used.
     *
     * @param rawGrayscaleData  packed grayscale pixels, row-major, one byte per pixel (0–255).
     *                          Array length must be >= width * height.
     * @param width             frame width  in pixels
     * @param height            frame height in pixels
     * @return int[2] (same array instance reused each call — copy if you need to keep values):
     *         [0] = Steering  [-100 … +100]  negative = line is left, positive = line is right
     *         [1] = Tracking  [  0 … 100  ]  100 = straight path ahead, 0 = hard turn / line lost
     */
    public int[] processFrame(byte[] rawGrayscaleData, int width, int height) {
        if (!initialized || imgW != width || imgH != height) {
            init(width, height, cfg);
        }
        runPipeline(rawGrayscaleData, width, height);
        result[0] = Math.round(output.steering);
        // Invert the internal intersection scale:
        //   internal 0   (straight) → output 100 (no braking needed)
        //   internal 100 (hard turn / lost) → output 0 (brake / slow down)
        result[1] = Math.round(100f - output.intersection);
        return result;
    }

    // =========================================================================
    //  Lifecycle
    // =========================================================================

    /**
     * Feed one frame into the calibration process.
     *
     * Place the robot correctly on the line, then call this method repeatedly
     * (one call per camera frame) until it returns true. Once it returns true
     * the straight-ahead reference is locked and you can switch to calling
     * processFrame() for normal operation.
     *
     * Internally this runs only the H-ROI detection (Otsu threshold + line-lock)
     * to find where the near line sits. It collects Config.calibFrames solid
     * detections and takes their median as the steering zero reference; the
     * look-ahead zero is derived from that via perspective geometry.
     *
     * @param rawGrayscaleData  packed grayscale pixels, row-major, one byte per pixel
     * @param width             frame width  in pixels
     * @param height            frame height in pixels
     * @return true when calibration is complete and processFrame() is ready to use
     */
    public boolean calibrate(byte[] rawGrayscaleData, int width, int height) {
        if (!initialized || imgW != width || imgH != height) {
            init(width, height, cfg);
        }
        if (calibrated) return true;

        final boolean bright    = cfg.trackIsBright;
        final int     W         = width;
        final int     rowStride = width;

        // Stage 1: Otsu threshold from the H-ROI histogram
        final int[] hist = histogram;
        java.util.Arrays.fill(hist, 0);
        final long count = (long)(hRoiBot - hRoiTop) * W;
        for (int row = hRoiTop; row < hRoiBot; row++) {
            final int base = row * rowStride;
            for (int col = 0; col < W; col++) {
                hist[rawGrayscaleData[base + col] & 0xFF]++;
            }
        }
        final int otsu = otsuThreshold(hist, count);
        final int threshold = bright
                ? Math.min(240, otsu + cfg.thresholdMargin)
                : Math.max(15,  otsu - cfg.thresholdMargin);

        // Stage 2: find the strongest contiguous white band centroid (full-width search)
        final int[] colW = colWeight;
        java.util.Arrays.fill(colW, 0, W, 0);
        for (int row = hRoiTop; row < hRoiBot; row++) {
            final int base = row * rowStride;
            for (int col = 0; col < W; col++) {
                final int px = rawGrayscaleData[base + col] & 0xFF;
                if (bright ? (px >= threshold) : (px <= threshold)) colW[col]++;
            }
        }

        final int    hRoiRows  = Math.max(1, hRoiBot - hRoiTop);
        final double minWeight = (double) cfg.minWeightFrac * hRoiRows * W;
        final int    runGapTol = Math.max(2, W / 50);

        long   bestSum = 0,  curSum = 0;
        double bestMoment = 0, curMoment = 0;
        int    bestStart = -1, curStart = -1, gapCols = 0;

        for (int col = 0; col < W; col++) {
            final int w = colW[col];
            if (w > 0) {
                if (curStart < 0) { curStart = col; curSum = 0; curMoment = 0; }
                curSum += w; curMoment += (double) col * w; gapCols = 0;
            } else if (curStart >= 0 && ++gapCols > runGapTol) {
                if (curSum > bestSum) { bestSum = curSum; bestMoment = curMoment; bestStart = curStart; }
                curStart = -1;
            }
        }
        if (curStart >= 0 && curSum > bestSum) {
            bestSum = curSum; bestMoment = curMoment; bestStart = curStart;
        }

        // If a solid line was found, collect its centroid as a calibration sample
        if (bestSum >= minWeight && bestStart >= 0) {
            final int centroidX = (int)(bestMoment / bestSum);
            if (calibCount < calibSteerSamples.length) calibSteerSamples[calibCount] = centroidX;
            if (++calibCount >= Math.max(1, cfg.calibFrames)) {
                final int n = Math.min(calibCount, calibSteerSamples.length);
                java.util.Arrays.sort(calibSteerSamples, 0, n);
                steerRefX  = calibSteerSamples[n / 2];                  // robust median
                trackRefX  = imgW * 0.5f + (steerRefX - imgW * 0.5f) * laPerspectiveRatio;
                calibrated = true;
            }
        }

        return calibrated;
    }

    /**
     * Explicitly initialise (or re-initialise) the pipeline for a given frame size.
     * Called automatically by processFrame() when needed; only call this directly
     * if you want to change Config mid-run without waiting for a frame.
     *
     * @param width   frame width  in pixels
     * @param height  frame height in pixels
     * @param config  pipeline tuning parameters
     */
    public void init(int width, int height, Config config) {
        this.cfg  = config;
        this.imgW = width;
        this.imgH = height;

        int hRoiCY = (int)(config.hRoiCenterYFrac * height);
        hRoiTop = Math.max(0,      hRoiCY - config.hRoiThickPx / 2);
        hRoiBot = Math.min(height, hRoiTop + config.hRoiThickPx);

        // Robot-body exclusion: never let the strip dip into the bottom band where
        // the robot's own chassis is visible. If it would, slide the whole strip up
        // so its BOTTOM edge sits at the top of the ignored band.
        final float ignoreFrac = Math.max(0f, Math.min(0.9f, config.bottomIgnoreFrac));
        final int   ignoreTop  = (int)((1f - ignoreFrac) * height);
        if (hRoiBot > ignoreTop) {
            hRoiBot = Math.max(config.hRoiThickPx, ignoreTop);
            hRoiTop = Math.max(0, hRoiBot - config.hRoiThickPx);
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
            final double wSum  = (b - a) * (b - a) * 0.5;                          // ∫(b−r)dr
            final double rwSum = b * (b * b - a * a) / 2.0 - (b * b * b - a * a * a) / 3.0; // ∫r(b−r)dr
            final double effRow = (wSum > 1e-6) ? rwSum / wSum : (a + b) * 0.5;
            lookAheadRowFrac = (height > 0) ? (float) (effRow / height) : 0.5f;
            final float scaleNear = perspectiveWidthScale((hRoiTop + hRoiBot) * 0.5f / height);
            final float scaleFar  = perspectiveWidthScale(lookAheadRowFrac);
            laPerspectiveRatio = (scaleNear > 1e-4f) ? scaleFar / scaleNear : 1f;
        }

        rowCx             = new float[Math.max(1, height)];
        colWeight         = new int[Math.max(1, width)];
        trackAcquired     = false;
        lostFrames        = 0;
        calibrated        = false;
        calibCount        = 0;
        calibSteerSamples = new int[Math.max(1, config.calibFrames)];
        laPathCount       = 0;
        steerRefX         = width * 0.5f;   // default = image centre until calibrated
        trackRefX         = width * 0.5f;
        lastCentroidX     = width / 2;
        lastTrackWidthPx  = config.vRoiHalfWidthPx;   // sensible fallback before first detection
        fpsWindowStartNs  = 0L;
        fpsFrameCount     = 0;
        framesPerSecond   = 0;
        totalFrames       = 0L;
        firstFrameNs      = 0L;
        elapsedNs         = 0L;
        initialized       = true;
    }

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

    /** True once the camera-offset references have been captured. */
    public boolean isCalibrated()    { return calibrated; }
    /** Calibrated straight-ahead column for steering, as a fraction of frame width. */
    public float getSteerRefXFrac()  { return (imgW > 0) ? steerRefX / imgW : 0.5f; }
    /** Calibrated straight-ahead column for the look-ahead, as a fraction of frame width. */
    public float getTrackRefXFrac()  { return (imgW > 0) ? trackRefX / imgW : 0.5f; }
    /** Discard the calibration so the next solid detections re-learn the offset. */
    public void requestRecalibration() {
        calibrated = false; calibCount = 0;
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
    public int getVRoiHalfWidthPx()  { return (cfg != null) ? cfg.vRoiHalfWidthPx : 40; }

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
    //  Hot path — zero heap allocations per frame
    // =========================================================================

    /**
     * Core pipeline. Reads raw grayscale pixels (row-major, rowStride = width)
     * and writes results into the internal {@code output} holder.
     * Called exclusively by {@link #processFrame}.
     */
    private void runPipeline(byte[] pixels, int width, int height) {
        updateFps();

        final boolean bright = cfg.trackIsBright;
        final int   W     = width;
        // rowStride == width because raw grayscale arrays have no padding.
        final int   rowStride = width;
        final float halfW = W * 0.5f;

        // ── Stage 1: Otsu adaptive threshold from the H-ROI histogram ─────────
        // Build a fresh grayscale histogram of the scan strip, then let Otsu's
        // method find the grey level that best separates track from floor. This
        // recalibrates every frame, so uneven / changing lighting is handled.
        final int[] hist = histogram;
        java.util.Arrays.fill(hist, 0);
        final long count = (long) (hRoiBot - hRoiTop) * W;
        for (int row = hRoiTop; row < hRoiBot; row++) {
            final int base = row * rowStride;
            for (int col = 0; col < W; col++) {
                hist[pixels[base + col] & 0xFF]++;
            }
        }
        final int otsu = otsuThreshold(hist, count);
        // Optional bias nudges the cut toward the track side (see Config doc).
        // For a white line: threshold = Otsu + margin  (looking for bright pixels)
        // For a dark  line: threshold = Otsu - margin  (looking for dark  pixels)
        final int threshold = bright
                ? Math.min(240, otsu + cfg.thresholdMargin)
                : Math.max(15,  otsu - cfg.thresholdMargin);

        // ── Stage 2: gated line-lock → steering ───────────────────────────────
        //
        //   Frame layout (row 0 = top = farthest floor, bottom = nearest robot):
        //
        //   row 0   ┌──────────────────────────────────┐
        //           │   [V-ROI] narrow column above    │
        //   hRoiTop ╠══════════[ H-ROI ]═══════════════╣  ← scan strip
        //   hRoiBot ╚══════════════════════════════════╝
        //
        // Instead of averaging EVERY bright pixel across the strip (which lets a
        // reflection anywhere drag the centroid toward the middle), we collapse the
        // H-ROI into a per-column white-pixel profile and lock onto ONE contiguous
        // white band — the track. With track memory on we only scan a gate around
        // the band's last known position, so bright spots outside the gate are
        // ignored and steering can run all the way to the edge. The lock is held
        // until the line has been gone for trackLostGraceFrames (you have moved
        // fully off it); only then do we search the whole width for a new line.
        final int[] colW = colWeight;
        java.util.Arrays.fill(colW, 0, W, 0);
        for (int row = hRoiTop; row < hRoiBot; row++) {
            final int base = row * rowStride;
            for (int col = 0; col < W; col++) {
                final int px = pixels[base + col] & 0xFF;
                if (bright ? (px >= threshold) : (px <= threshold)) colW[col]++;
            }
        }

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

        // Largest contiguous white run inside the search window. Small gaps (up to
        // runGapTol columns, e.g. specular noise that briefly breaks the line) are
        // bridged so a single line is not split into competing fragments.
        final int runGapTol = Math.max(2, W / 50);
        long   bestSum = 0,   curSum = 0;
        double bestMoment = 0, curMoment = 0;
        int    bestStart = -1, bestEnd = -1, curStart = -1, gapCols = 0;
        for (int col = searchLo; col < searchHi; col++) {
            final int w = colW[col];
            if (w > 0) {
                if (curStart < 0) { curStart = col; curSum = 0; curMoment = 0; }
                curSum += w; curMoment += (double) col * w; gapCols = 0;
            } else if (curStart >= 0 && ++gapCols > runGapTol) {   // run ends
                if (curSum > bestSum) {
                    bestSum = curSum; bestMoment = curMoment;
                    bestStart = curStart; bestEnd = col - gapCols + 1;
                }
                curStart = -1;
            }
        }
        if (curStart >= 0 && curSum > bestSum) {
            bestSum = curSum; bestMoment = curMoment; bestStart = curStart; bestEnd = searchHi;
        }

        boolean trackFound;
        int     centroidX;
        float   forcedIntersection = -1f;   // ≥ 0 overrides the look-ahead stage

        if (bestSum >= minWeight && bestStart >= 0) {
            // Locked onto the line this frame.
            trackFound       = true;
            centroidX        = (int) (bestMoment / bestSum);
            lastCentroidX    = centroidX;
            lastTrackWidthPx = Math.max(1, bestEnd - bestStart);
            trackAcquired    = true;
            lostFrames       = 0;
        } else if (locked) {
            // The line we were following just left the gate → you have moved fully
            // off it. Pin steering hard toward the edge it exited and flag a 100 %
            // turn. Keep the gate parked at that edge so the SAME line re-locks if
            // it comes back; only after the grace period do we hunt a new one.
            trackFound  = false;
            lostFrames++;
            // Pin toward the edge it exited, RELATIVE to the calibrated centre, so
            // a fully-lost line still reads steering ±100 after the offset below.
            final boolean onRight = lastCentroidX >= Math.round(steerRefX);
            centroidX     = onRight ? Math.min(W - 1, Math.round(steerRefX + halfW))
                                    : Math.max(0,     Math.round(steerRefX - halfW));
            lastCentroidX = centroidX;
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
        //  position is known and the one-time calibration has had a chance to run.)

        // ── Stage 3 threshold: adaptive to the (dimmer, farther) look-ahead ───
        // The far look-ahead line is dimmer than the near line, so the H-ROI Otsu
        // can miss it — the tracker then loses the line early and the track value
        // plateaus below 100. Build a subsampled histogram of the whole V-ROI band
        // (subsampled so the curving line is still captured cheaply) and run Otsu
        // again to get a threshold matched to the look-ahead's own lighting.
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
            final int otsuV = otsuThreshold(histV, cntV);
            vThreshold = bright ? Math.min(240, otsuV + cfg.thresholdMargin)
                                : Math.max(15,  otsuV - cfg.thresholdMargin);
        }

        // ── Stage 3: look-ahead straightness / curvature → intersection ───────
        //
        // We TRACK the line up through the V-ROI, recentring the scan window on
        // the previous row's centroid, so curves and a skewed approach are
        // followed instead of leaking out of a fixed column. From the tracked
        // centroids we derive ONE 0…100 signal where:
        //
        //     0   = the track goes perfectly straight ahead (straight up the
        //           frame, aligned with the robot — mirrors steering == 0),
        //   100   = a hard turn / the track bends sharply away.
        //
        // It is the larger of two terms, so it responds to BOTH rotation and curves:
        //   • angle = |tilt of the look-ahead line from vertical|  → grows as the
        //             robot rotates / the path leans left or right.
        //   • bend  = how far the centroids depart from a straight line
        //             → grows with genuine curvature beyond a straight lean.
        //
        // Only rows where the line is actually VISIBLE are used; empty far-away
        // rows are never counted, so the value can reach 0 and never sticks high.
        //
        final int     half   = cfg.vRoiHalfWidthPx;
        final float[] rowCx  = this.rowCx;
        final int     maxGap = Math.max(8, cfg.hRoiThickPx);   // stop after this many empty rows

        // Per-row line-lock limits (same memory idea as Stage 2, applied upward):
        //   • minRowW   — a row's white run must be at least this many pixels to
        //                 count, so 1-px specular noise never seeds the tracker.
        //   • vMaxDrift — the run's centre may sit at most this far from where the
        //                 line was on the row below. A real line drifts only a few
        //                 px per row even on curves, so this is what stops the
        //                 tracker from JUMPING onto a separate white object (table)
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
        //     Each row we lock onto the contiguous white RUN closest to the line's
        //     position on the row below — never the average of all white in the
        //     window — and reject any run that is too small or too far (a different
        //     object). Stop once the line has been missing for maxGap rows (end /
        //     off the road), so empty frame above is never mistaken for the track.
        int trackCenter = centroidX;
        int gap         = 0;
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

            // Scan the window, picking the white run whose centre is nearest the
            // predicted centre (trackCenter). Small gaps inside a run are bridged.
            long   runW = 0;   double runM = 0;   int runStart = -1, runGap = 0;
            double bestC = Double.NaN; long bestW = 0; double bestDist = Double.MAX_VALUE;
            for (int col = vLeft; col < vRight; col++) {
                final int px = pixels[base + col] & 0xFF;
                final boolean white = bright ? (px >= vThreshold) : (px <= vThreshold);
                if (white) {
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
        //     Kept only to lean the V-ROI overlay box with the line — it no longer
        //     feeds the track value below.
        double slope = 0.0;
        if (fitN >= 2) {
            final double denom = (double) fitN * sYY - sY * sY;
            if (Math.abs(denom) > 1e-6) {
                slope = ((double) fitN * sYC - sY * sC) / denom;
            }
        }
        final float tiltDeg = (float) Math.toDegrees(Math.atan(slope));

        // 3c. Look-ahead line position (band-locked, far-weighted centroid).
        final boolean lookAheadValid = (laW > 0.0);
        final float   lookAheadX     = lookAheadValid ? (float) (laM / laW) : steerRefX;

        // ── Steering: signed offset from the calibrated straight-ahead column ──
        //    Same per-pixel sensitivity as before (÷halfW), just zeroed at the
        //    calibrated reference instead of image centre.
        final float steering = Math.max(-100f, Math.min(100f,
                (centroidX - steerRefX) / halfW * 100f));

        // 3d. Intersection — internal scale (0 = straight, 100 = hard turn / lost).
        //     This is inverted to the TRACKING output in processFrame() so the caller
        //     receives 100 on a straight path and 0 on a hard turn or line loss.
        float intersection;
        if (!trackFound || !lookAheadValid) {
            intersection = 100f;                          // no road ahead → off
        } else {
            final float span = Math.max(1f, halfW * cfg.trackOffsetFullFrac);
            intersection = Math.max(0f, Math.min(100f,
                    Math.abs(lookAheadX - trackRefX) / span * 100f));
        }

        // Track-memory override: once the locked line has left the gate we report a
        // hard turn (100 %) regardless of the look-ahead geometry, signalling that
        // you have moved fully off the line and a new one is being sought.
        if (forcedIntersection >= 0f) intersection = forcedIntersection;

        // ── Write internal output ─────────────────────────────────────────────
        output.steering     = steering;
        output.intersection = intersection;
        output.centroidX    = trackFound ? centroidX : -1;
        output.tiltDeg      = trackFound ? tiltDeg : 0f;
        output.trackWidthPx = lastTrackWidthPx;   // measured width, held on track loss
        output.trackFound   = trackFound;
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
     * Given a 256-bin grayscale histogram, this finds the grey level t that
     * splits the pixels into two classes — floor and track — so that the
     * intra-class variance is minimised. Minimising intra-class variance is
     * mathematically equivalent to MAXIMISING the between-class variance
     *
     *     σ_b²(t) = w0(t)·w1(t)·(µ0(t) − µ1(t))²
     *
     * which is the cheap single-pass form computed here (the total variance is
     * constant, so the t that maximises σ_b² is exactly the t that minimises the
     * weighted within-class variance). w0/w1 are the class pixel counts and
     * µ0/µ1 their mean grey levels. We track running sums so the whole sweep is
     * a single O(256) loop with no per-pixel work.
     *
     * @param hist  256-bin histogram (hist[g] = number of pixels with value g)
     * @param total number of pixels accumulated into {@code hist}
     * @return      optimal threshold in [0,255]; 128 if the histogram is empty
     */
    private static int otsuThreshold(int[] hist, long total) {
        if (total <= 0) return 128;

        // Weighted sum of all grey levels: Σ g·hist[g].
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
            }
        }
        return bestT;
    }
}