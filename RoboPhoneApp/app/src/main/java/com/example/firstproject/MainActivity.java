package com.example.firstproject;

import android.Manifest;
import android.content.SharedPreferences;
import android.content.pm.PackageManager;
import android.graphics.Color;
import android.graphics.ImageFormat;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraManager;
import android.hardware.camera2.CaptureRequest;
import android.hardware.camera2.params.StreamConfigurationMap;
import android.os.Bundle;
import android.util.Log;
import android.util.Range;
import android.util.Size;
import android.view.View;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.CheckBox;
import android.widget.ProgressBar;
import android.widget.SeekBar;
import android.widget.TextView;

import androidx.annotation.NonNull;
import androidx.annotation.OptIn;
import androidx.appcompat.app.AppCompatActivity;
import androidx.camera.camera2.interop.Camera2CameraControl;
import androidx.camera.camera2.interop.Camera2CameraInfo;
import androidx.camera.camera2.interop.Camera2Interop;
import androidx.camera.camera2.interop.CaptureRequestOptions;
import androidx.camera.camera2.interop.ExperimentalCamera2Interop;
import androidx.camera.core.Camera;
import androidx.camera.core.CameraSelector;
import androidx.camera.core.ExposureState;
import androidx.camera.core.ImageAnalysis;
import androidx.camera.core.ImageProxy;
import androidx.camera.core.Preview;
import androidx.camera.core.resolutionselector.ResolutionSelector;
import androidx.camera.core.resolutionselector.ResolutionStrategy;
import androidx.camera.lifecycle.ProcessCameraProvider;
import androidx.camera.view.PreviewView;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import com.example.linefollower.LineFollowerPipeline;
import com.google.common.util.concurrent.ListenableFuture;

import java.nio.ByteBuffer;
import java.util.Arrays;
import java.util.Locale;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

/**
 * RoboPhone main activity.
 *
 * Camera: CameraX (no OpenCV).  Works on any Android device with Camera2 support.
 * Processing: LineFollowerPipeline (pure Java, no native libraries), fed full
 * colour YUV frames. The line and floor colours are learned during calibration,
 * so any two different colours work (white on black, black on blue, …).
 *
 * All display data comes from the library's FrameCallback:
 *   Overlay — ⊥ sensor, look-ahead band, calibrated reference (OverlayView)
 *   STEER   — SeekBar + number [-50…+50]
 *   TRACK   — ProgressBar + percentage [min…100%]
 *   Status  — calibration progress / TRACKING (green) / TRACK LOST (orange)
 *   Colours — learned line + floor swatches, threshold, colour separation
 *   RECALIBRATE — re-learns colours and straight-ahead reference
 */
public class MainActivity extends AppCompatActivity {

    private static final String TAG              = "RoboPhone";
    private static final int    CAMERA_PERM_CODE = 100;

    // Per-frame logging is OFF by default: at 60 fps the String.format + logcat
    // I/O on every frame is real overhead and can itself cap throughput. Flip to
    // true only when debugging a single frame's output.
    private static final boolean VERBOSE = false;

    // ── Target camera throughput ────────────────────────────────────────────────
    // We ask the camera for a 60 fps auto-exposure range at a modest analysis
    // resolution. High frame-rate sensor modes generally only exist at lower
    // resolutions, and the smaller frame also means less CPU work per frame.
    private static final int  TARGET_FPS        = 60;
    // 1280×720 is the most common 60 fps sensor mode; the small 640×480 binned mode
    // is frequently 30 fps-only. Adjust to whatever the capability probe reports as
    // 60 fps-capable for this device (see logCameraCapabilities()).
    private static final Size ANALYSIS_RES      = new Size(1280, 720);

    // ── Performance profiling ───────────────────────────────────────────────────
    // Emit an averaged summary every PROF_WINDOW frames (per-frame line is DEBUG).
    private static final int    PROF_WINDOW        = 30;
    // Robot speed we must support — used to report ground distance travelled per
    // frame, i.e. how far the robot moves blind between two processed frames.
    private static final double ROBOT_MAX_SPEED_MPS = 1.0;

    // ── CameraX ───────────────────────────────────────────────────────────────
    private PreviewView        previewView;
    private Camera             camera;          // bound camera (null until bound)
    // Real vertical field of view of the frames the pipeline gets (deg), from Camera2 focal
    // length + sensor size, for this stream's crop and orientation; 0 = not known yet. Only
    // logged and shown — the pipeline still uses Config.verticalFovDeg (60) until measured.
    private static final float APP_VFOV_DEG   = 60f;   // Config.verticalFovDeg used by the app
    private volatile float     measuredVFovDeg = 0f;
    private volatile float     measuredHFovDeg = 0f;
    private boolean            fovRequested    = false;   // camera thread

    // ── v3 test settings (all OFF by default, remembered in SharedPreferences) ──
    // AE/AWB lock: after calibration completes, auto-exposure and auto-white-balance are locked
    // (Camera2 CONTROL_AE_LOCK / CONTROL_AWB_LOCK through Camera2Interop), so a bright patch
    // cannot make the camera darken the whole image; unlocked again on RECALIBRATE.
    // EV: exposure compensation in EV (e.g. -1 keeps glossy tape from saturating).
    // v3 HUD: an extra HUD line with corridor coverage / fit, coasting, rebuilt base, glare.
    private static final String PREFS = "robophone_v3";
    private static final float[] EV_STEPS = {0f, -0.5f, -1f, -1.5f, -2f, 0.5f, 1f};
    private SharedPreferences prefs;
    private volatile boolean aeLockEnabled = false;
    private volatile boolean hudV3Enabled  = false;
    private float            evSetting     = 0f;
    private volatile boolean aeLocked      = false;   // lock currently applied to the camera
    private boolean          wasCalibrated = false;   // camera thread: calibration edge detection
    private CheckBox         cbAeLock, cbHudV3;
    private Button           btnEv;
    private TextView         tvV3;
    private final Runnable   lockRunnable = () -> setAeAwbLock(true);

    // ── Test recording (LineFollowerPipeline.Recorder; OFF at every start) ──
    // REC CSV: one CSV line per frame (FrameData + timestamp). CAPTURE: keep the last ~2 s of
    // frames (1/4 size) and save them on SAVE 2s or automatically on a coast / rebuilt base /
    // rejected band / loss / steering jump. Files: getExternalFilesDir("linefollower").
    private LineFollowerPipeline.Recorder recorder;
    private CheckBox         cbRec, cbCapture;
    private int              shownCaptures = 0;   // UI thread: captures already reported
    private String           shownError;          // UI thread: recorder error already logged
    // Steering-jump counters for the v3 HUD (camera thread writes; reset by tapping the line).
    private volatile int     hudJumps = 0, hudPin50 = 0, hudMaxStep = 0, hudRejected = 0;
    private int              prevSteer = Integer.MIN_VALUE;
    private volatile boolean resetJumps = false;
    private ExecutorService    cameraExecutor;

    // ── Vision pipeline ───────────────────────────────────────────────────────
    private final LineFollowerPipeline pipeline = new LineFollowerPipeline();

    // ── Sensor overlay ────────────────────────────────────────────────────────
    private OverlayView overlayView;

    // ── HUD views ─────────────────────────────────────────────────────────────
    private SeekBar     seekSteering;
    private ProgressBar pbTrack;
    private TextView    tvSteering;
    private TextView    tvIntersection;
    private TextView    tvStatus;
    private TextView    tvFrames;
    private TextView    tvDebug;
    private View        swatchLine;
    private View        swatchFloor;

    // ── Values shared between camera thread → UI thread ───────────────────────
    // Written by the pipeline's FrameCallback, read by applyHud().
    private volatile float   hudSteering     = 0f;
    private volatile float   hudIntersection = 0f;
    private volatile boolean hudTrackFound   = false;
    private volatile boolean hudCalibrated   = false;
    private volatile int     hudCalibPhase   = LineFollowerPipeline.CALIB_LEARNING_COLORS;
    private volatile float   hudCalibProgress = 0f;
    private volatile int     hudCalibIssue   = LineFollowerPipeline.ISSUE_NONE;
    private volatile boolean hudColorReady   = false;
    private volatile int     hudLineColor    = 0;
    private volatile int     hudFloorColor   = 0;
    private volatile float   hudSeparation   = 0f;
    private volatile int     hudThreshold    = 0;
    private volatile int     hudFps          = 0;
    private volatile long    hudTotalFrames  = 0L;
    private volatile double  hudElapsedSec   = 0.0;
    private volatile float   hudPipeMs       = 0f;    // avg ms per frame inside the pipeline (last window)
    private volatile float   hudFrameMs      = 0f;    // avg ms per frame for the whole analyzer (last window)
    private volatile boolean hudPending      = false;
    // v3 FrameData fields for the optional HUD line.
    private volatile float   hudCorrCoverage = 0f;
    private volatile float   hudCorrRms      = 0f;
    private volatile boolean hudCoasting     = false;
    private volatile boolean hudRecon        = false;
    private volatile boolean hudHeld         = false;
    private volatile float   hudMasked       = 0f;

    // Set by the RECALIBRATE button (UI thread), consumed on the camera thread so
    // the pipeline is only ever touched from one thread.
    private volatile boolean recalibrateRequested = false;

    // Pre-allocated Runnable — zero heap allocation per frame on camera thread
    private final Runnable hudRunnable = this::applyHud;

    // Pre-allocated pixel buffers — filled each frame from the camera planes
    // (padding stripped): full-res Y, half-res U and V, and one row scratch.
    private byte[] yFrame, uFrame, vFrame, rowScratch;

    // ── Profiling accumulators (camera thread only — no synchronisation needed) ─
    private long profLastStartNs   = 0L;   // start timestamp of the previous frame
    private long profWindowStartNs = 0L;   // start timestamp of the current window
    private long profSumProcNs     = 0L;   // Σ per-frame processing time in window
    private long profSumPipeNs     = 0L;   // Σ time inside the LineFollowerPipeline call only
    private long profMaxPipeNs     = 0L;   // slowest pipeline call in the window
    private int  profFrameCount    = 0;    // frames accumulated in current window
    private long lastPipeNs        = 0L;   // pipeline time of the current frame

    // =========================================================================
    //  Lifecycle
    // =========================================================================

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        setContentView(R.layout.activity_main);

        previewView    = findViewById(R.id.preview_view);
        overlayView    = findViewById(R.id.overlay_view);
        seekSteering   = findViewById(R.id.seek_steering);
        pbTrack        = findViewById(R.id.pb_track);
        tvSteering     = findViewById(R.id.tv_steering);
        tvIntersection = findViewById(R.id.tv_intersection);
        tvStatus       = findViewById(R.id.tv_status);
        tvFrames       = findViewById(R.id.tv_frames);
        tvDebug        = findViewById(R.id.tv_debug);
        swatchLine     = findViewById(R.id.swatch_line);
        swatchFloor    = findViewById(R.id.swatch_floor);
        Button btnRecalibrate = findViewById(R.id.btn_recalibrate);
        btnRecalibrate.setOnClickListener(v -> {
            if (aeLocked) setAeAwbLock(false);       // re-learn with live exposure / white balance
            recalibrateRequested = true;
        });

        // v3 test settings (defaults OFF → behaviour unchanged).
        prefs         = getSharedPreferences(PREFS, MODE_PRIVATE);
        aeLockEnabled = prefs.getBoolean("ae_lock", false);
        hudV3Enabled  = prefs.getBoolean("hud_v3", false);
        evSetting     = prefs.getFloat("ev", 0f);
        cbAeLock = findViewById(R.id.cb_ae_lock);
        cbHudV3  = findViewById(R.id.cb_hud_v3);
        btnEv    = findViewById(R.id.btn_ev);
        tvV3     = findViewById(R.id.tv_v3);
        cbAeLock.setChecked(aeLockEnabled);
        cbHudV3.setChecked(hudV3Enabled);
        tvV3.setVisibility(hudV3Enabled ? View.VISIBLE : View.GONE);
        btnEv.setText(evLabel());
        cbAeLock.setOnCheckedChangeListener((b, on) -> {
            aeLockEnabled = on;
            prefs.edit().putBoolean("ae_lock", on).apply();
            if (on && hudCalibrated) setAeAwbLock(true);
            else if (!on && aeLocked) setAeAwbLock(false);
        });
        cbHudV3.setOnCheckedChangeListener((b, on) -> {
            hudV3Enabled = on;
            prefs.edit().putBoolean("hud_v3", on).apply();
            tvV3.setVisibility(on ? View.VISIBLE : View.GONE);
        });
        btnEv.setOnClickListener(v -> {
            int i = 0;
            for (int k = 0; k < EV_STEPS.length; k++) if (EV_STEPS[k] == evSetting) i = k;
            evSetting = EV_STEPS[(i + 1) % EV_STEPS.length];
            prefs.edit().putFloat("ev", evSetting).apply();
            btnEv.setText(evLabel());
            applyExposureCompensation(true);
        });

        // Every calibration and processing frame reports here (camera thread). The recorder sits
        // in front of onPipelineFrame and only forwards while REC / CAPTURE are off.
        recorder = LineFollowerPipeline.Recorder.attach(pipeline,
                getExternalFilesDir("linefollower"), this::onPipelineFrame);
        cbRec     = findViewById(R.id.cb_rec);
        cbCapture = findViewById(R.id.cb_capture);
        cbRec.setOnCheckedChangeListener((b, on) -> {
            recorder.setRecording(on);
            Log.i(TAG, on ? "REC on" : "REC off: " + recorder.getLastCsvPath());
        });
        cbCapture.setOnCheckedChangeListener((b, on) -> {
            recorder.setCapture(on);
            recorder.setAutoCapture(on);
            Log.i(TAG, "CAPTURE " + (on ? "on (auto-save on events)" : "off"));
        });
        findViewById(R.id.btn_save_capture).setOnClickListener(v -> {
            if (!recorder.isCapturing()) { cbCapture.setChecked(true); return; }   // first tap: start keeping frames
            recorder.captureNow();
        });
        tvV3.setOnClickListener(v -> resetJumps = true);

        // Dedicated background thread for image analysis
        cameraExecutor = Executors.newSingleThreadExecutor();

        requestCameraPermissionIfNeeded();
    }

    @Override
    protected void onResume() {
        super.onResume();
        if (hasCameraPermission()) startCamera();
    }

    @Override
    protected void onPause() {
        super.onPause();
        // CameraX unbinds automatically on lifecycle pause. Close the CSV so the file is complete.
        if (cbRec != null && cbRec.isChecked()) cbRec.setChecked(false);
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        if (cameraExecutor != null) cameraExecutor.shutdown();
        if (recorder != null) recorder.close();
        pipeline.release();
    }

    // =========================================================================
    //  CameraX setup
    // =========================================================================

    private void startCamera() {
        ListenableFuture<ProcessCameraProvider> future =
                ProcessCameraProvider.getInstance(this);

        future.addListener(() -> {
            try {
                ProcessCameraProvider provider = future.get();
                bindCamera(provider);
            } catch (ExecutionException | InterruptedException e) {
                Log.e(TAG, "CameraProvider error: " + e.getMessage());
                runOnUiThread(() -> tvStatus.setText("CAMERA INIT FAILED"));
            }
        }, ContextCompat.getMainExecutor(this));
    }

    @OptIn(markerClass = ExperimentalCamera2Interop.class)
    private void bindCamera(ProcessCameraProvider provider) {
        provider.unbindAll();

        // Log what this camera actually supports so we can pick a 60 fps config.
        logCameraCapabilities();

        final Range<Integer> fpsRange = new Range<>(TARGET_FPS, TARGET_FPS);
        int rotation = getWindowManager().getDefaultDisplay().getRotation();

        // Constrain BOTH surfaces to the same resolution. The session runs at the
        // rate of its slowest surface, so an unconstrained full-screen preview can
        // pin the whole session to 30 fps even when analysis could go faster — we
        // must hold the preview to a 60 fps-capable size too.
        ResolutionSelector resSelector = new ResolutionSelector.Builder()
                .setResolutionStrategy(new ResolutionStrategy(
                        ANALYSIS_RES,
                        ResolutionStrategy.FALLBACK_RULE_CLOSEST_LOWER_THEN_HIGHER))
                .build();

        // ── Preview — renders straight to PreviewView ─────────────────────────
        Preview.Builder previewBuilder = new Preview.Builder()
                .setResolutionSelector(resSelector);
        new Camera2Interop.Extender<>(previewBuilder)
                .setCaptureRequestOption(
                        CaptureRequest.CONTROL_AE_TARGET_FPS_RANGE, fpsRange);
        Preview preview = previewBuilder.build();
        preview.setSurfaceProvider(previewView.getSurfaceProvider());

        // ── ImageAnalysis — delivers YUV_420_888 frames for processing ─────────
        // setOutputImageRotationEnabled(true) rotates the Y-plane data to match
        // the display orientation automatically, so we never need to rotate pixels.
        ImageAnalysis.Builder analysisBuilder = new ImageAnalysis.Builder()
                .setTargetRotation(rotation)
                .setResolutionSelector(resSelector)
                // KEEP_ONLY_LATEST is the right choice for real-time control: it
                // never queues stale frames, so we always process the freshest one.
                // It does NOT cap the rate — if the camera delivers 60 and a frame
                // is processed in <16.6 ms, nothing is dropped.
                .setBackpressureStrategy(ImageAnalysis.STRATEGY_KEEP_ONLY_LATEST)
                .setOutputImageRotationEnabled(true);

        // Ask the camera (via Camera2) to lock auto-exposure to a 60 fps range so
        // it won't lengthen exposure and drop to ~30 fps in normal lighting.
        new Camera2Interop.Extender<>(analysisBuilder)
                .setCaptureRequestOption(
                        CaptureRequest.CONTROL_AE_TARGET_FPS_RANGE, fpsRange);

        ImageAnalysis analysis = analysisBuilder.build();
        analysis.setAnalyzer(cameraExecutor, this::analyzeFrame);

        try {
            camera = provider.bindToLifecycle(
                    this,
                    CameraSelector.DEFAULT_BACK_CAMERA,
                    preview,
                    analysis);
            aeLocked = false;                                   // a new session starts unlocked
            applyExposureCompensation(false);                   // no call at all while EV = 0
            if (aeLockEnabled && pipeline.isCalibrated()) setAeAwbLock(true);
            Log.i(TAG, "Camera bound successfully");
            runOnUiThread(() -> tvStatus.setText("Camera ready"));
        } catch (Exception e) {
            Log.e(TAG, "bindToLifecycle failed: " + e.getMessage());
            runOnUiThread(() -> tvStatus.setText("BIND FAILED: " + e.getMessage()));
        }
    }

    /**
     * Dumps the back camera's real capabilities to Logcat so we can choose a config
     * that actually reaches 60 fps:
     *   • CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES — if [60,60] isn't listed, the
     *     camera will silently ignore our request and fall back to 30.
     *   • Per-size max fps for YUV_420_888 (the ImageAnalysis format), derived from
     *     the minimum frame duration — tells us which resolutions support 60 fps.
     *   • Dedicated constrained high-speed sizes/ranges, if any.
     * Filter Logcat by tag "RoboPhone" and look for "CAPS".
     */
    private void logCameraCapabilities() {
        try {
            CameraManager cm = (CameraManager) getSystemService(CAMERA_SERVICE);
            if (cm == null) return;
            for (String id : cm.getCameraIdList()) {
                CameraCharacteristics ch = cm.getCameraCharacteristics(id);
                Integer facing = ch.get(CameraCharacteristics.LENS_FACING);
                if (facing == null || facing != CameraCharacteristics.LENS_FACING_BACK) continue;

                Range<Integer>[] aeRanges =
                        ch.get(CameraCharacteristics.CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES);
                Log.i(TAG, "CAPS cam " + id + " AE fps ranges: " + Arrays.toString(aeRanges));

                StreamConfigurationMap map =
                        ch.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP);
                if (map != null) {
                    Size[] sizes = map.getOutputSizes(ImageFormat.YUV_420_888);
                    if (sizes != null) {
                        for (Size s : sizes) {
                            long minDurNs = map.getOutputMinFrameDuration(
                                    ImageFormat.YUV_420_888, s);
                            long maxFps = (minDurNs > 0) ? Math.round(1e9 / minDurNs) : 0;
                            Log.i(TAG, "CAPS   YUV " + s + " -> max " + maxFps + " fps");
                        }
                    }
                    Size[] hs = map.getHighSpeedVideoSizes();
                    if (hs != null) {
                        for (Size s : hs) {
                            Log.i(TAG, "CAPS   HIGH-SPEED " + s + " ranges "
                                    + Arrays.toString(map.getHighSpeedVideoFpsRangesFor(s)));
                        }
                    }
                }
            }
        } catch (Exception e) {
            Log.w(TAG, "logCameraCapabilities failed: " + e.getMessage());
        }
    }

    // =========================================================================
    //  Frame analysis  — runs on cameraExecutor (background thread)
    // =========================================================================

    private void analyzeFrame(ImageProxy imageProxy) {
        final long frameStartNs = System.nanoTime();
        try {
            final int width  = imageProxy.getWidth();
            final int height = imageProxy.getHeight();
            final ImageProxy.PlaneProxy[] planes = imageProxy.getPlanes();

            if (!fovRequested) {   // once: the real FOV for this frame size / orientation
                fovRequested = true;
                runOnUiThread(() -> computeFieldOfView(width, height));
            }

            // Lazy-configure the library on the first frame when dimensions are known.
            if (!pipeline.isInitialized()) {
                LineFollowerPipeline.Config cfg = new LineFollowerPipeline.Config();
                cfg.autoColor        = true;    // learn line + floor colours at calibration
                cfg.thresholdMargin  = 0;
                cfg.hRoiCenterYFrac  = 0.85f;
                cfg.hRoiThickPx      = 24;
                cfg.bottomIgnoreFrac = 0.25f;
                cfg.vRoiHalfWidthPx  = 80;
                cfg.cameraTiltDeg    = 45f;
                cfg.verticalFovDeg   = APP_VFOV_DEG;   // still the guess; see computeFieldOfView()
                pipeline.configure(cfg);
                final int cw = (width + 1) / 2, ch = (height + 1) / 2;
                yFrame     = new byte[width * height];
                uFrame     = new byte[cw * ch];
                vFrame     = new byte[cw * ch];
                rowScratch = new byte[Math.max(planes[1].getRowStride(), planes[2].getRowStride())];
                Log.i(TAG, "Pipeline configured: " + width + "x" + height
                        + "  yRowStride=" + planes[0].getRowStride()
                        + "  uvPixelStride=" + planes[1].getPixelStride());
            }

            if (recalibrateRequested) {
                recalibrateRequested = false;
                pipeline.requestRecalibration();
                Log.i(TAG, "Recalibration requested");
            }

            // Copy the three planes into packed buffers, stripping row padding.
            copyLumaPlane(planes[0], yFrame, width, height);
            copyChromaPlane(planes[1], uFrame, (width + 1) / 2, (height + 1) / 2, rowScratch);
            copyChromaPlane(planes[2], vFrame, (width + 1) / 2, (height + 1) / 2, rowScratch);

            // Calibration phase — learns the colours, then the straight-ahead
            // reference. The FrameCallback reports progress to the HUD/overlay.
            final long pipeStartNs = System.nanoTime();
            if (!pipeline.isCalibrated()) {
                pipeline.calibrateYuv420(yFrame, uFrame, vFrame, width, height);
            } else {
                // Run the library — returns [steering, tracking]. Overlay + HUD are
                // updated from the FrameCallback fired inside this call.
                final int[] result = pipeline.processFrameYuv420(yFrame, uFrame, vFrame, width, height);
                if (VERBOSE) Log.d(TAG, "S=" + result[0] + "  T=" + result[1]);
            }
            lastPipeNs = System.nanoTime() - pipeStartNs;   // pipeline only (no plane copies)

            profileFrame(frameStartNs, System.nanoTime());

        } finally {
            imageProxy.close();
        }
    }

    /** Copy the Y plane into {@code dst} (width × height, no padding). */
    private static void copyLumaPlane(ImageProxy.PlaneProxy plane, byte[] dst, int width, int height) {
        final ByteBuffer buf = plane.getBuffer();
        final int rowStride  = plane.getRowStride();
        buf.rewind();
        if (rowStride == width) {
            buf.get(dst, 0, width * height);
        } else {
            for (int row = 0; row < height; row++) {
                buf.position(row * rowStride);
                buf.get(dst, row * width, width);
            }
        }
    }

    /**
     * Copy a half-resolution chroma plane (U or V) into {@code dst} (cw × ch,
     * packed). Handles both planar (pixelStride 1) and interleaved (pixelStride 2,
     * NV12/NV21-style) layouts that YUV_420_888 devices deliver.
     */
    private static void copyChromaPlane(ImageProxy.PlaneProxy plane, byte[] dst,
                                        int cw, int ch, byte[] scratch) {
        final ByteBuffer buf = plane.getBuffer();
        final int rowStride  = plane.getRowStride();
        final int pixStride  = plane.getPixelStride();
        buf.rewind();
        if (pixStride == 1 && rowStride == cw) {
            buf.get(dst, 0, cw * ch);
            return;
        }
        final int rowLen = (cw - 1) * pixStride + 1;   // last row may be shorter than rowStride
        for (int row = 0; row < ch; row++) {
            buf.position(row * rowStride);
            if (pixStride == 1) {
                buf.get(dst, row * cw, cw);
            } else {
                buf.get(scratch, 0, rowLen);
                final int base = row * cw;
                for (int col = 0; col < cw; col++) dst[base + col] = scratch[col * pixStride];
            }
        }
    }

    // =========================================================================
    //  Pipeline callback — camera thread, fired by calibrate / processFrame
    // =========================================================================

    private void onPipelineFrame(LineFollowerPipeline.FrameData d) {
        if (overlayView != null) overlayView.setFrame(d);

        hudSteering      = d.steering;
        hudIntersection  = d.tracking;
        hudTrackFound    = d.trackFound;
        hudCalibrated    = d.isCalibrated;
        hudCalibPhase    = d.calibPhase;
        hudCalibProgress = d.calibProgress;
        hudCalibIssue    = d.calibIssue;
        hudColorReady    = d.colorModelReady;
        hudLineColor     = d.lineColorArgb;
        hudFloorColor    = d.floorColorArgb;
        hudSeparation    = d.colorSeparation;
        hudThreshold     = d.threshold;
        hudCorrCoverage  = d.corridorCoverage;
        hudCorrRms       = d.corridorFitRms;
        hudCoasting      = d.coasting;
        hudRecon         = d.baseReconstructed;
        hudHeld          = d.lineHeld;
        hudMasked        = d.maskedFrac;
        hudRejected      = d.rejectedBands;
        // Steering jumps between consecutive frames (v3 HUD: "no ±50 jumps in glare").
        if (resetJumps) { resetJumps = false; hudJumps = 0; hudPin50 = 0; hudMaxStep = 0; }
        if (d.isCalibrated && wasCalibrated) {           // processing frames only
            if (prevSteer != Integer.MIN_VALUE) {
                final int step = Math.abs(d.steering - prevSteer);
                if (step > hudMaxStep) hudMaxStep = step;
                if (step >= 25) hudJumps++;
            }
            if (Math.abs(d.steering) >= 50) hudPin50++;
            prevSteer = d.steering;
        } else {
            prevSteer = Integer.MIN_VALUE;
        }
        // Calibration just finished: lock exposure / white balance if enabled (UI thread).
        if (d.isCalibrated && !wasCalibrated && aeLockEnabled) runOnUiThread(lockRunnable);
        wasCalibrated = d.isCalibrated;
        hudFps           = pipeline.getFramesPerSecond();
        hudTotalFrames   = pipeline.getTotalFrames();
        hudElapsedSec    = pipeline.getElapsedSeconds();
        if (!hudPending) {
            hudPending = true;
            runOnUiThread(hudRunnable);
        }
    }

    // =========================================================================
    //  Real-time performance profiling  — runs on the camera thread
    // =========================================================================

    /**
     * Logs per-frame processing time and a rolling FPS so we can prove the
     * pipeline runs in real time (fast enough not to drop frames) at the target
     * robot speed.
     *
     *   • processing time = frameEnd − frameStart  (CPU work for one frame)
     *   • FPS             = frames ÷ wall-clock span of the window
     *                       (interval between frame STARTS, so it captures the
     *                        true throughput, including any camera-delivery gaps)
     *
     * @param startNs {@link System#nanoTime()} captured at the start of the cycle
     * @param endNs   {@link System#nanoTime()} captured at the end of the cycle
     */
    private void profileFrame(long startNs, long endNs) {
        final long procNs = endNs - startNs;
        // Gap since the previous frame's start → instantaneous frame interval / FPS.
        final long intervalNs = (profLastStartNs > 0L) ? (startNs - profLastStartNs) : 0L;
        final double instFps  = (intervalNs > 0L) ? 1e9 / intervalNs : 0.0;

        // Per-frame detail (DEBUG): raw start/end timestamps + processing time.
        if (VERBOSE) Log.d(TAG, String.format(Locale.US,
                "PROF frame  start=%d ns  end=%d ns  proc=%.2f ms  dt=%.2f ms  ~%.1f fps",
                startNs, endNs, procNs / 1e6, intervalNs / 1e6, instFps));

        // Accumulate a window for a stable averaged summary.
        if (profWindowStartNs == 0L) profWindowStartNs = startNs;
        profSumProcNs += procNs;
        profSumPipeNs += lastPipeNs;
        if (lastPipeNs > profMaxPipeNs) profMaxPipeNs = lastPipeNs;
        profLastStartNs = startNs;

        if (++profFrameCount >= PROF_WINDOW) {
            final long   spanNs    = endNs - profWindowStartNs;
            final double avgProcMs = (profSumProcNs / (double) profFrameCount) / 1e6;
            final double fps       = (spanNs > 0) ? profFrameCount * 1e9 / spanNs : 0.0;
            final double budgetMs  = (fps > 0) ? 1000.0 / fps : 0.0;     // delivery interval
            // Real-time if the CPU work per frame fits inside the delivery interval;
            // otherwise frames are being dropped (STRATEGY_KEEP_ONLY_LATEST).
            final boolean realtime = avgProcMs < budgetMs;
            final double  cmPerFrame = ROBOT_MAX_SPEED_MPS * (budgetMs / 1000.0) * 100.0;

            Log.i(TAG, String.format(Locale.US,
                    "PROF %d frames: avg proc=%.2f ms  FPS=%.1f  budget=%.1f ms  %s  "
                    + "(@%.1f m/s → %.1f cm/frame blind)  last-sec=%d fps  "
                    + "total=%d frames in %.1fs",
                    profFrameCount, avgProcMs, fps, budgetMs,
                    realtime ? "REAL-TIME OK" : "DROPPING FRAMES",
                    ROBOT_MAX_SPEED_MPS, cmPerFrame, pipeline.getFramesPerSecond(),
                    pipeline.getTotalFrames(), pipeline.getElapsedSeconds()));

            // One easy-to-grep line for measuring on the phone:  adb logcat -s RoboPhone | grep PERF
            //   fps      = pipeline.getFramesPerSecond() (frames in the last full second)
            //   pipe_ms  = time inside the LineFollowerPipeline call (avg / max of the window)
            //   frame_ms = whole analyzer: plane copies + pipeline
            final double avgPipeMs = (profSumPipeNs / (double) profFrameCount) / 1e6;
            Log.i(TAG, String.format(Locale.US,
                    "PERF fps=%d pipe_ms=%.2f pipe_max_ms=%.2f frame_ms=%.2f frames=%d",
                    pipeline.getFramesPerSecond(), avgPipeMs, profMaxPipeNs / 1e6, avgProcMs, profFrameCount));
            hudPipeMs  = (float) avgPipeMs;
            hudFrameMs = (float) avgProcMs;

            // Reset the window.
            profFrameCount    = 0;
            profSumProcNs     = 0L;
            profSumPipeNs     = 0L;
            profMaxPipeNs     = 0L;
            profWindowStartNs = 0L;
        }
    }

    // =========================================================================
    //  HUD update — runs on the UI thread
    // =========================================================================

    private void applyHud() {
        final float   s     = hudSteering;
        final float   t     = hudIntersection;
        final boolean found = hudTrackFound;

        // Steering SeekBar: range [0, 100], centre = 50
        seekSteering.setProgress((int)(50f + s), false);
        tvSteering.setText(String.format(Locale.US, "%+.0f", s));

        // Track ProgressBar: range [0, 100]
        pbTrack.setProgress((int)t);
        tvIntersection.setText(String.format(Locale.US, "%.0f%%", t));

        // Colour code by how centred / how much track is ahead
        final int steerColor;
        if (!found)             steerColor = Color.parseColor("#FF6D00"); // orange: lost
        else if (Math.abs(s) < 10f) steerColor = Color.parseColor("#00E676"); // green: on track
        else if (Math.abs(s) < 25f) steerColor = Color.parseColor("#FFD600"); // yellow: small correction
        else                    steerColor = Color.parseColor("#FF1744"); // red: large correction

        tvSteering.setTextColor(steerColor);
        seekSteering.setThumbTintList(
                android.content.res.ColorStateList.valueOf(steerColor));

        final int trackColor = t > 60f ? Color.parseColor("#00E676")
                             : t > 25f ? Color.parseColor("#FFD600")
                                       : Color.parseColor("#FF1744");
        pbTrack.setProgressTintList(
                android.content.res.ColorStateList.valueOf(trackColor));
        tvIntersection.setTextColor(trackColor);

        if (hudCalibrated) {
            tvStatus.setText(found ? "TRACKING" : "TRACK LOST");
            tvStatus.setTextColor(found ? Color.parseColor("#00E676")
                                        : Color.parseColor("#FF6D00"));
        } else {
            tvStatus.setText(calibrationMessage());
            tvStatus.setTextColor(hudCalibIssue == LineFollowerPipeline.ISSUE_NONE
                    ? Color.parseColor("#FFD600") : Color.parseColor("#FF6D00"));
        }

        tvFrames.setText(String.format(Locale.US,
                "FPS: %d   pipe: %.2f ms   frame: %.2f ms   total: %d   time: %.1fs",
                hudFps, hudPipeMs, hudFrameMs, hudTotalFrames, hudElapsedSec));

        // Learned colours + vision debug
        final boolean colors = hudColorReady;
        swatchLine.setBackgroundColor(colors ? hudLineColor : Color.DKGRAY);
        swatchFloor.setBackgroundColor(colors ? hudFloorColor : Color.DKGRAY);
        final String thr = hudThreshold < 0 ? "none" : String.valueOf(hudThreshold);
        final String fov = measuredVFovDeg > 0f
                ? String.format(Locale.US, "  vFOV %.1f°", measuredVFovDeg) : "";
        tvDebug.setText((colors
                ? String.format(Locale.US, "thr: %s  sep: %.1fσ", thr, hudSeparation)
                : String.format(Locale.US, "thr: %s  sep: --", thr)) + fov);

        if (hudV3Enabled) {
            tvV3.setText(String.format(Locale.US,
                    "corr %.2f  rms %.2f  %s%s%s mask %.0f%%  rej %d  AE %s  EV %+.1f%n"
                    + "max step %d  jumps(>=25) %d  frames at ±50 %d  (tap to reset)%s%s",
                    hudCorrCoverage, hudCorrRms,
                    hudCoasting ? "COAST " : "", hudRecon ? "RECON " : "",
                    (hudHeld && !hudCoasting) ? "HOLD " : "",
                    hudMasked * 100f, hudRejected, aeLocked ? "LOCKED" : "auto", evSetting,
                    hudMaxStep, hudJumps, hudPin50,
                    recorder.isRecording() ? "  REC" : "",
                    recorder.isCapturing() ? "  CAP " + recorder.getCaptureCount()
                            + (recorder.isCaptureBusy() ? " saving" : "") : ""));
        }
        // A capture was saved: tell the tester where (logcat + short toast).
        final int caps = recorder.getCaptureCount();
        if (caps != shownCaptures) {
            shownCaptures = caps;
            Log.i(TAG, "CAPTURE saved " + recorder.getLastCapturePath());
            android.widget.Toast.makeText(this, "capture saved (" + caps + ")", android.widget.Toast.LENGTH_SHORT).show();
        }
        if (recorder.getLastError() != null) {
            final String err = recorder.getLastError();
            if (!err.equals(shownError)) { shownError = err; Log.w(TAG, "Recorder: " + err); }
        }

        hudPending = false;
    }

    // =========================================================================
    //  v3 camera settings (only used when enabled; defaults leave the camera untouched)
    // =========================================================================

    private String evLabel() {
        return evSetting == 0f ? "EV 0" : String.format(Locale.US, "EV %+.1f", evSetting);
    }

    /**
     * Lock (or unlock) auto-exposure and auto-white-balance through Camera2Interop. Called on
     * the UI thread: after calibration completes (if the setting is on), and unlock on
     * RECALIBRATE or when the setting is switched off.
     */
    @OptIn(markerClass = ExperimentalCamera2Interop.class)
    private void setAeAwbLock(boolean lock) {
        if (camera == null) return;
        try {
            final CaptureRequestOptions opts = new CaptureRequestOptions.Builder()
                    .setCaptureRequestOption(CaptureRequest.CONTROL_AE_LOCK, lock)
                    .setCaptureRequestOption(CaptureRequest.CONTROL_AWB_LOCK, lock)
                    .build();
            Camera2CameraControl.from(camera.getCameraControl()).setCaptureRequestOptions(opts);
            aeLocked = lock;
            Log.i(TAG, "AE/AWB " + (lock ? "LOCKED after calibration" : "unlocked"));
        } catch (Exception e) {
            Log.w(TAG, "AE/AWB lock failed: " + e.getMessage());
        }
    }

    /**
     * Apply the EV setting as CameraX exposure compensation (index = EV / step, clamped to the
     * device range). With EV 0 nothing is sent unless {@code force} (the user changed it back).
     */
    private void applyExposureCompensation(boolean force) {
        if (camera == null || (evSetting == 0f && !force)) return;
        try {
            final ExposureState es = camera.getCameraInfo().getExposureState();
            if (!es.isExposureCompensationSupported()) { Log.w(TAG, "EV compensation not supported"); return; }
            final float step = es.getExposureCompensationStep().floatValue();
            final Range<Integer> range = es.getExposureCompensationRange();
            int idx = step > 0f ? Math.round(evSetting / step) : 0;
            idx = Math.max(range.getLower(), Math.min(range.getUpper(), idx));
            camera.getCameraControl().setExposureCompensationIndex(idx);
            Log.i(TAG, String.format(Locale.US, "EV compensation %+.1f -> index %d (step %.2f)", evSetting, idx, step));
        } catch (Exception e) {
            Log.w(TAG, "EV compensation failed: " + e.getMessage());
        }
    }

    /**
     * Real field of view of the frames handed to the pipeline, from the bound camera's Camera2
     * characteristics: FOV = 2·atan(size / 2f) with f = LENS_INFO_AVAILABLE_FOCAL_LENGTHS[0] and
     * size = the sensor's physical size (SENSOR_INFO_PHYSICAL_SIZE) scaled to the active array,
     * then cropped to the stream's aspect ratio (CameraX scales the active array and crops the
     * centre). The frame's vertical axis is the sensor axis with as many pixels as the frame
     * height (the app rotates the frames to portrait). Logged as "FOV ..." and shown on the HUD.
     * Not applied: Config.verticalFovDeg stays 60 until the number is confirmed.
     */
    @OptIn(markerClass = ExperimentalCamera2Interop.class)
    private void computeFieldOfView(int frameW, int frameH) {
        if (camera == null) return;
        try {
            final Camera2CameraInfo info = Camera2CameraInfo.from(camera.getCameraInfo());
            final float[] focal = info.getCameraCharacteristic(CameraCharacteristics.LENS_INFO_AVAILABLE_FOCAL_LENGTHS);
            final android.util.SizeF phys = info.getCameraCharacteristic(CameraCharacteristics.SENSOR_INFO_PHYSICAL_SIZE);
            final Size pix = info.getCameraCharacteristic(CameraCharacteristics.SENSOR_INFO_PIXEL_ARRAY_SIZE);
            final android.graphics.Rect act = info.getCameraCharacteristic(CameraCharacteristics.SENSOR_INFO_ACTIVE_ARRAY_SIZE);
            final Integer orient = info.getCameraCharacteristic(CameraCharacteristics.SENSOR_ORIENTATION);
            if (focal == null || focal.length == 0 || phys == null || pix == null || act == null) {
                Log.w(TAG, "FOV: camera characteristics incomplete");
                return;
            }
            final double f = focal[0];
            final double[] r = FieldOfView.compute(f, phys.getWidth(), phys.getHeight(), pix.getWidth(), pix.getHeight(),
                    act.width(), act.height(), frameW, frameH);
            measuredVFovDeg = (float) r[0];
            measuredHFovDeg = (float) r[1];
            Log.i(TAG, String.format(Locale.US,
                    "FOV vertical=%.1f deg horizontal=%.1f deg (frame %dx%d; f=%.2f mm; sensor %.2fx%.2f mm,"
                    + " active %dx%d of %dx%d px; crop %.2fx%.2f mm; sensor orientation %s)."
                    + " Config.verticalFovDeg is still %.0f.",
                    measuredVFovDeg, measuredHFovDeg, frameW, frameH, f, phys.getWidth(), phys.getHeight(),
                    act.width(), act.height(), pix.getWidth(), pix.getHeight(), r[2], r[3],
                    orient, APP_VFOV_DEG));
            if (focal.length > 1) Log.i(TAG, "FOV: lens has several focal lengths " + Arrays.toString(focal) + "; used the first");
        } catch (Exception e) {
            Log.w(TAG, "FOV computation failed: " + e.getMessage());
        }
    }

    /** Status text while calibrating: phase, progress, and any problem detected. */
    private String calibrationMessage() {
        final int pct = Math.round(hudCalibProgress * 100f);
        switch (hudCalibIssue) {
            case LineFollowerPipeline.ISSUE_LOW_CONTRAST:
                return "CALIBRATING " + pct + "% — line and floor colours too similar, retrying";
            case LineFollowerPipeline.ISSUE_NO_FLOOR:
                return "CALIBRATING " + pct + "% — place the robot ON the line (floor on both sides)";
            case LineFollowerPipeline.ISSUE_NO_LINE:
                return "CALIBRATING " + pct + "% — line not visible in the STEER strip";
            default:
                return hudCalibPhase == LineFollowerPipeline.CALIB_LEARNING_COLORS
                        ? "CALIBRATING " + pct + "% — learning line / floor colours"
                        : "CALIBRATING " + pct + "% — finding straight-ahead";
        }
    }

    // =========================================================================
    //  Camera permission
    // =========================================================================

    private boolean hasCameraPermission() {
        return ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA)
                == PackageManager.PERMISSION_GRANTED;
    }

    private void requestCameraPermissionIfNeeded() {
        if (hasCameraPermission()) {
            startCamera();
        } else {
            ActivityCompat.requestPermissions(
                    this, new String[]{Manifest.permission.CAMERA}, CAMERA_PERM_CODE);
        }
    }

    @Override
    public void onRequestPermissionsResult(int requestCode,
                                           @NonNull String[] permissions,
                                           @NonNull int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        if (requestCode == CAMERA_PERM_CODE
                && grantResults.length > 0
                && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
            startCamera();
        } else {
            Log.e(TAG, "Camera permission denied.");
            tvStatus.setText("CAMERA PERMISSION DENIED");
        }
    }
}