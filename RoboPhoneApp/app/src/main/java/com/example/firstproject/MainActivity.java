package com.example.firstproject;

import android.Manifest;
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
import android.view.WindowManager;
import android.widget.ProgressBar;
import android.widget.SeekBar;
import android.widget.TextView;

import androidx.annotation.NonNull;
import androidx.annotation.OptIn;
import androidx.appcompat.app.AppCompatActivity;
import androidx.camera.camera2.interop.Camera2Interop;
import androidx.camera.camera2.interop.ExperimentalCamera2Interop;
import androidx.camera.core.CameraSelector;
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
 * Processing: LineFollowerPipeline (pure Java, no native libraries).
 * Track colour: WHITE line on BLACK floor — set trackIsBright=false for the opposite.
 *
 * On-screen HUD shows:
 *   STEER  — SeekBar + number [-100…+100]
 *   TRACK  — ProgressBar + percentage [0…100%]
 *   Status — TRACKING (green) or TRACK LOST (orange)
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

    // ── Values shared between camera thread → UI thread ───────────────────────
    private volatile float   hudSteering     = 0f;
    private volatile float   hudIntersection = 0f;
    private volatile boolean hudTrackFound   = false;
    private volatile int     hudFps          = 0;
    private volatile long    hudTotalFrames  = 0L;
    private volatile double  hudElapsedSec   = 0.0;
    private volatile boolean hudPending      = false;

    // Pre-allocated Runnable — zero heap allocation per frame on camera thread
    private final Runnable hudRunnable = this::applyHud;

    // Pre-allocated pixel buffer — filled each frame from the camera Y-plane.
    private byte[] frameBuffer;

    // ── Profiling accumulators (camera thread only — no synchronisation needed) ─
    private long profLastStartNs   = 0L;   // start timestamp of the previous frame
    private long profWindowStartNs = 0L;   // start timestamp of the current window
    private long profSumProcNs     = 0L;   // Σ per-frame processing time in window
    private int  profFrameCount    = 0;    // frames accumulated in current window

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
        // CameraX unbinds automatically on lifecycle pause
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        if (cameraExecutor != null) cameraExecutor.shutdown();
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
            provider.bindToLifecycle(
                    this,
                    CameraSelector.DEFAULT_BACK_CAMERA,
                    preview,
                    analysis);
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
            final int width     = imageProxy.getWidth();
            final int height    = imageProxy.getHeight();
            final ImageProxy.PlaneProxy yPlane    = imageProxy.getPlanes()[0];
            final ByteBuffer            yBuffer   = yPlane.getBuffer();
            final int                   rowStride = yPlane.getRowStride();

            // Lazy-configure the library on the first frame when dimensions are known.
            if (!pipeline.isInitialized()) {
                LineFollowerPipeline.Config cfg = new LineFollowerPipeline.Config();
                cfg.trackIsBright    = true;
                cfg.thresholdMargin  = 0;
                cfg.hRoiCenterYFrac  = 0.85f;
                cfg.hRoiThickPx      = 24;
                cfg.bottomIgnoreFrac = 0.25f;
                cfg.vRoiHalfWidthPx  = 80;
                cfg.cameraTiltDeg    = 45f;
                cfg.verticalFovDeg   = 60f;
                pipeline.configure(cfg);
                frameBuffer = new byte[width * height];
                Log.i(TAG, "Pipeline configured: " + width + "x" + height
                        + "  rowStride=" + rowStride);
                final float ignoreFrac = cfg.bottomIgnoreFrac;
                runOnUiThread(() -> {
                    tvStatus.setText("Calibrating...");
                    if (overlayView != null) overlayView.setBottomIgnoreFrac(ignoreFrac);
                });
            }

            // Extract grayscale bytes into the pre-allocated buffer, stripping padding.
            yBuffer.rewind();
            if (rowStride == width) {
                yBuffer.get(frameBuffer);
            } else {
                for (int row = 0; row < height; row++) {
                    yBuffer.position(row * rowStride);
                    yBuffer.get(frameBuffer, row * width, width);
                }
            }

            // Calibration phase — feed frames until the straight-ahead reference is locked.
            if (!pipeline.isCalibrated()) {
                final boolean done = pipeline.calibrate(frameBuffer, width, height);
                runOnUiThread(() -> tvStatus.setText(done ? "Calibrated" : "Calibrating..."));
                profileFrame(frameStartNs, System.nanoTime());
                return;
            }

            // Run the library — returns [steering, tracking]
            final int[]   result    = pipeline.processFrame(frameBuffer, width, height);
            final int     steering  = result[0];
            final int     tracking  = result[1];
            final boolean trackFound = tracking > 2;

            if (VERBOSE) Log.d(TAG, "S=" + steering + "  T=" + tracking + "  found=" + trackFound);

            // ── Update overlay ────────────────────────────────────────────────
            if (overlayView != null) {
                final int   w = pipeline.getImgW();
                final int   h = pipeline.getImgH();
                final float trackWidthPx = pipeline.getVRoiHalfWidthPx() * 2f;
                final float boxWidthPx   = Math.max(12f, trackWidthPx * 1.3f);
                final float halfBotFrac  = (boxWidthPx * 0.5f) / w;
                final float hTopFrac     = pipeline.getHRoiTop() / (float) h;
                final float vTopFrac     = pipeline.getVRoiTop() / (float) h;
                final float scaleNear    = pipeline.perspectiveWidthScale(hTopFrac);
                final float scaleFar     = pipeline.perspectiveWidthScale(vTopFrac);
                final float taper        = (scaleNear > 1e-4f) ? (scaleFar / scaleNear) : 1f;
                final float halfTopFrac  = halfBotFrac * taper;
                final float steerRefX    = pipeline.getSteerRefXFrac() * w;
                final int   cx           = (int)(steerRefX + (steering / 100f) * (w * 0.5f));
                final float boxCenterFrac = pipeline.getSteerRefXFrac();
                final float shearFrac     = pipeline.getTrackRefXFrac() - boxCenterFrac;

                overlayView.setLookAheadPath(
                        pipeline.getLookAheadColFracs(),
                        pipeline.getLookAheadRowFracs(),
                        pipeline.getLookAheadPointCount());
                overlayView.update(
                        hTopFrac,
                        pipeline.getHRoiBot() / (float) h,
                        vTopFrac,
                        boxCenterFrac,
                        halfBotFrac,
                        halfTopFrac,
                        cx / (float) w,
                        shearFrac,
                        trackFound);
            }

            // ── Schedule HUD update ───────────────────────────────────────────
            hudSteering     = steering;
            hudIntersection = tracking;
            hudTrackFound   = trackFound;
            hudFps          = pipeline.getFramesPerSecond();
            hudTotalFrames  = pipeline.getTotalFrames();
            hudElapsedSec   = pipeline.getElapsedSeconds();
            if (!hudPending) {
                hudPending = true;
                runOnUiThread(hudRunnable);
            }

            final long frameEndNs = System.nanoTime();
            profileFrame(frameStartNs, frameEndNs);

        } finally {
            imageProxy.close();
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

            // Reset the window.
            profFrameCount    = 0;
            profSumProcNs     = 0L;
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

        // Steering SeekBar: range [0, 200], centre = 100
        seekSteering.setProgress((int)(100f + s), false);
        tvSteering.setText(String.format(Locale.US, "%+.0f", s));

        // Track ProgressBar: range [0, 100]
        pbTrack.setProgress((int)t);
        tvIntersection.setText(String.format(Locale.US, "%.0f%%", t));

        // Colour code by how centred / how much track is ahead
        final int steerColor;
        if (!found)             steerColor = Color.parseColor("#FF6D00"); // orange: lost
        else if (Math.abs(s) < 20f) steerColor = Color.parseColor("#00E676"); // green: on track
        else if (Math.abs(s) < 50f) steerColor = Color.parseColor("#FFD600"); // yellow: small correction
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

        tvStatus.setText(found ? "TRACKING" : "TRACK LOST");
        tvStatus.setTextColor(found ? Color.parseColor("#00E676")
                                    : Color.parseColor("#FF6D00"));

        tvFrames.setText(String.format(Locale.US,
                "FPS: %d    total: %d    time: %.1fs",
                hudFps, hudTotalFrames, hudElapsedSec));

        hudPending = false;
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