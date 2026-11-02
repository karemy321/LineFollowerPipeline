package com.example.firstproject;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.DashPathEffect;
import android.graphics.Paint;
import android.graphics.Path;
import android.util.AttributeSet;
import android.view.View;

import com.example.linefollower.LineFollowerPipeline;

/**
 * Transparent overlay drawn on top of the camera PreviewView.
 *
 * Everything drawn here comes straight from the library's
 * {@link LineFollowerPipeline.FrameData} — no geometry is recomputed in the app.
 *
 * v3 path tracking (default, FrameData.pathMode):
 *
 *   ┌──────────────────────────────────────────┐
 *   │  ROI (white frame) — the region read      │
 *   │      ░░ unreliable cells (red) ░░         │  glare / shadow / object / colour patch
 *   │        ●  path measured now (green)       │
 *   │       ●   path from memory (cyan)         │
 *   │      ●    hidden part filled from memory  │  (orange) / bridged (yellow)
 *   │     ◎ ← look-ahead target (magenta)       │
 *   │     ↑ steering arrow from the robot       │
 *   │▒▒▒▒▒▒▒▒▒▒▒▒ IGNORED (robot) ▒▒▒▒▒▒▒▒▒▒▒▒▒│
 *   └──────────────────────────────────────────┘
 *
 * v2 (Config.pathTracking = false): the H-ROI strip and the V-ROI look-ahead band (⊥).
 *
 * The camera image is mapped to the view exactly as PreviewView's default
 * FILL_CENTER scale type does (scale to cover, centre, crop the overflow), so
 * the overlay lines up with the preview even when the aspect ratios differ.
 *
 * {@link #setFrame} is safe to call from any thread.
 */
public final class OverlayView extends View {

    // ── Paints (pre-allocated — never created inside onDraw) ──────────────────

    private final Paint hFillPaint      = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint hStrokePaint    = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint vFillPaint      = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint vStrokePaint    = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint centroidPaint   = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint refPaint        = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint cornerPaint     = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint labelPaint      = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint ignorePaint     = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint ignoreLinePaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    // v3
    private final Paint roiPaint        = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint unreliablePaint = new Paint();
    private final Paint pathPaint       = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint pointPaint      = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint targetPaint     = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Path  arrowPath  = new Path();  // reused in onDraw — no allocation
    private final Path  vRoiPath   = new Path();  // reused — straight V-ROI trapezoid
    private final Path  bandPath   = new Path();  // reused — dynamic (curved) V-ROI band
    private final Path  centerPath = new Path();  // reused — dynamic centroid poly-line

    private static final int COLOR_TRACKING    = Color.parseColor("#00E676");
    private static final int COLOR_LOST        = Color.parseColor("#FF6D00");
    private static final int COLOR_CALIBRATING = Color.parseColor("#FFD600");
    // path point sources (FrameData.pathSource)
    private static final int COLOR_FRESH       = Color.parseColor("#00E676");   // measured now
    private static final int COLOR_MEMORY      = Color.parseColor("#00B8FF");   // from memory
    private static final int COLOR_UNRELIABLE  = Color.parseColor("#FF9100");   // hidden now, from memory
    private static final int COLOR_PREDICTED   = Color.parseColor("#FFEA00");   // bridged by continuity
    private static final int COLOR_TARGET      = Color.MAGENTA;

    private static final int GRID = LineFollowerPipeline.RELIABILITY_GRID_COLS * LineFollowerPipeline.RELIABILITY_GRID_ROWS;

    // ── Frame snapshots ───────────────────────────────────────────────────────
    // The camera thread writes into `pending` under the lock; onDraw copies it
    // into `drawn` under the same lock and then draws without holding it.
    private final Object   lock    = new Object();
    private final Snapshot pending = new Snapshot();
    private final Snapshot drawn   = new Snapshot();

    /** Copy of the FrameData fields the overlay draws. */
    private static final class Snapshot {
        boolean valid;
        int     imgW, imgH;
        float   hRoiTopFrac, hRoiBotFrac, vRoiTopFrac, ignoreTopFrac;
        float   steerRefXFrac, trackRefXFrac, halfBotFrac, halfTopFrac;
        float   centroidXFrac;
        int     steering, tracking;
        boolean trackFound, isCalibrated, lineHeld;
        final float[] laCol = new float[48];
        final float[] laRow = new float[48];
        int     laCount;
        // v3
        boolean pathMode, lookAheadValid;
        float   roiTopFrac, roiBotFrac, lookX, lookY, memoryFrac, confidence;
        final float[] pCol = new float[LineFollowerPipeline.PATH_MAX_POINTS];
        final float[] pRow = new float[LineFollowerPipeline.PATH_MAX_POINTS];
        final byte[]  pSrc = new byte[LineFollowerPipeline.PATH_MAX_POINTS];
        int     pCount;
        final byte[]  grid = new byte[GRID];
        int     cells;

        void copyFrom(LineFollowerPipeline.FrameData d) {
            valid         = true;
            imgW          = d.imgW;          imgH          = d.imgH;
            hRoiTopFrac   = d.hRoiTopFrac;   hRoiBotFrac   = d.hRoiBotFrac;
            vRoiTopFrac   = d.vRoiTopFrac;   ignoreTopFrac = d.ignoreTopFrac;
            steerRefXFrac = d.steerRefXFrac; trackRefXFrac = d.trackRefXFrac;
            halfBotFrac   = d.halfBotFrac;   halfTopFrac   = d.halfTopFrac;
            centroidXFrac = d.centroidXFrac; steering      = d.steering;   tracking = d.tracking;
            trackFound    = d.trackFound;    isCalibrated  = d.isCalibrated; lineHeld = d.lineHeld;
            laCount = Math.min(d.lookAheadPointCount, laCol.length);
            System.arraycopy(d.lookAheadColFrac, 0, laCol, 0, laCount);
            System.arraycopy(d.lookAheadRowFrac, 0, laRow, 0, laCount);
            pathMode   = d.pathMode;
            roiTopFrac = d.roiTopFrac;       roiBotFrac = d.roiBotFrac;
            lookAheadValid = d.lookAheadValid;
            lookX      = d.lookAheadXFrac;   lookY      = d.lookAheadYFrac;
            memoryFrac = d.pathMemoryFrac;   confidence = d.pathConfidence;
            pCount = Math.min(d.pathPointCount, pCol.length);
            System.arraycopy(d.pathColFrac, 0, pCol, 0, pCount);
            System.arraycopy(d.pathRowFrac, 0, pRow, 0, pCount);
            System.arraycopy(d.pathSource, 0, pSrc, 0, pCount);
            System.arraycopy(d.unreliableGrid, 0, grid, 0, GRID);
            cells = d.unreliableCells;
        }

        void copyFrom(Snapshot s) {
            valid         = s.valid;
            imgW          = s.imgW;          imgH          = s.imgH;
            hRoiTopFrac   = s.hRoiTopFrac;   hRoiBotFrac   = s.hRoiBotFrac;
            vRoiTopFrac   = s.vRoiTopFrac;   ignoreTopFrac = s.ignoreTopFrac;
            steerRefXFrac = s.steerRefXFrac; trackRefXFrac = s.trackRefXFrac;
            halfBotFrac   = s.halfBotFrac;   halfTopFrac   = s.halfTopFrac;
            centroidXFrac = s.centroidXFrac; steering      = s.steering;   tracking = s.tracking;
            trackFound    = s.trackFound;    isCalibrated  = s.isCalibrated; lineHeld = s.lineHeld;
            laCount = s.laCount;
            System.arraycopy(s.laCol, 0, laCol, 0, laCount);
            System.arraycopy(s.laRow, 0, laRow, 0, laCount);
            pathMode   = s.pathMode;
            roiTopFrac = s.roiTopFrac;       roiBotFrac = s.roiBotFrac;
            lookAheadValid = s.lookAheadValid;
            lookX      = s.lookX;            lookY      = s.lookY;
            memoryFrac = s.memoryFrac;       confidence = s.confidence;
            pCount = s.pCount;
            System.arraycopy(s.pCol, 0, pCol, 0, pCount);
            System.arraycopy(s.pRow, 0, pRow, 0, pCount);
            System.arraycopy(s.pSrc, 0, pSrc, 0, pCount);
            System.arraycopy(s.grid, 0, grid, 0, GRID);
            cells = s.cells;
        }
    }

    // Image → view mapping for the current draw (FILL_CENTER).
    private float mapScaleX, mapScaleY, mapOffX, mapOffY;

    // =========================================================================

    public OverlayView(Context context, AttributeSet attrs) {
        super(context, attrs);
        initPaints();
    }

    public OverlayView(Context context) {
        super(context);
        initPaints();
    }

    private void initPaints() {
        // H-ROI — cyan fill + stroke (stroke turns yellow while calibrating)
        hFillPaint.setStyle(Paint.Style.FILL);
        hFillPaint.setColor(Color.CYAN);
        hFillPaint.setAlpha(45);

        hStrokePaint.setStyle(Paint.Style.STROKE);
        hStrokePaint.setStrokeWidth(4f);

        // V-ROI — magenta fill + stroke
        vFillPaint.setStyle(Paint.Style.FILL);
        vFillPaint.setColor(Color.MAGENTA);
        vFillPaint.setAlpha(35);

        vStrokePaint.setStyle(Paint.Style.STROKE);
        vStrokePaint.setColor(Color.MAGENTA);
        vStrokePaint.setStrokeWidth(4f);
        vStrokePaint.setAlpha(230);

        // Centroid line — green when tracking, orange when lost
        centroidPaint.setStyle(Paint.Style.STROKE);
        centroidPaint.setStrokeWidth(5f);

        // Calibrated straight-ahead dashed reference
        refPaint.setStyle(Paint.Style.STROKE);
        refPaint.setColor(Color.WHITE);
        refPaint.setAlpha(140);
        refPaint.setStrokeWidth(2f);
        refPaint.setPathEffect(new DashPathEffect(new float[]{14f, 9f}, 0f));

        // Corner accent marks at T-junction
        cornerPaint.setStyle(Paint.Style.STROKE);
        cornerPaint.setColor(Color.WHITE);
        cornerPaint.setAlpha(200);
        cornerPaint.setStrokeWidth(3f);

        // Labels
        labelPaint.setStyle(Paint.Style.FILL);
        labelPaint.setColor(Color.WHITE);
        labelPaint.setAlpha(210);
        labelPaint.setTextSize(34f);
        labelPaint.setFakeBoldText(true);

        // Ignored bottom band — dim red shading + a solid boundary line
        ignorePaint.setStyle(Paint.Style.FILL);
        ignorePaint.setColor(Color.RED);
        ignorePaint.setAlpha(70);

        ignoreLinePaint.setStyle(Paint.Style.STROKE);
        ignoreLinePaint.setColor(Color.RED);
        ignoreLinePaint.setAlpha(220);
        ignoreLinePaint.setStrokeWidth(3f);

        // v3: ROI frame, unreliable cells, path, target
        roiPaint.setStyle(Paint.Style.STROKE);
        roiPaint.setColor(Color.WHITE);
        roiPaint.setAlpha(170);
        roiPaint.setStrokeWidth(3f);

        unreliablePaint.setStyle(Paint.Style.FILL);
        unreliablePaint.setColor(Color.RED);
        unreliablePaint.setAlpha(80);

        pathPaint.setStyle(Paint.Style.STROKE);
        pathPaint.setStrokeWidth(7f);
        pathPaint.setStrokeCap(Paint.Cap.ROUND);

        pointPaint.setStyle(Paint.Style.FILL);

        targetPaint.setStyle(Paint.Style.STROKE);
        targetPaint.setColor(COLOR_TARGET);
        targetPaint.setStrokeWidth(5f);
    }

    // =========================================================================
    //  Call from the pipeline's FrameCallback (camera thread)
    // =========================================================================

    /** Take a copy of this frame's overlay data and schedule a redraw. Any thread. */
    public void setFrame(LineFollowerPipeline.FrameData data) {
        synchronized (lock) {
            pending.copyFrom(data);
        }
        postInvalidate();
    }

    // =========================================================================
    //  Drawing — no allocations here
    // =========================================================================

    private float mx(float xFrac) { return mapOffX + xFrac * mapScaleX; }
    private float my(float yFrac) { return mapOffY + yFrac * mapScaleY; }

    @Override
    protected void onDraw(Canvas canvas) {
        synchronized (lock) {
            drawn.copyFrom(pending);
        }
        final Snapshot s = drawn;
        if (!s.valid || s.imgW <= 0 || s.imgH <= 0) return;

        // FILL_CENTER: scale the image to cover the view, centre it, crop overflow.
        final float W = getWidth(), H = getHeight();
        final float scale = Math.max(W / s.imgW, H / s.imgH);
        mapScaleX = s.imgW * scale;
        mapScaleY = s.imgH * scale;
        mapOffX   = (W - mapScaleX) * 0.5f;
        mapOffY   = (H - mapScaleY) * 0.5f;

        // ── Ignored bottom band (red) — robot body, excluded from processing ─
        if (s.ignoreTopFrac < 1f) {
            final float ignoreTop = my(s.ignoreTopFrac);
            canvas.drawRect(0f, ignoreTop, W, H, ignorePaint);
            canvas.drawLine(0f, ignoreTop, W, ignoreTop, ignoreLinePaint);
            canvas.drawText("IGNORED (robot)", 8f, ignoreTop + 34f, labelPaint);
        }

        if (s.pathMode) drawPath(canvas, s, W);
        else            drawT(canvas, s, W);
    }

    /** v3: ROI, unreliable cells, the traced path by source, the pure-pursuit target. */
    private void drawPath(Canvas canvas, Snapshot s, float W) {
        final float top = my(s.roiTopFrac), bot = my(s.roiBotFrac);
        final int GC = LineFollowerPipeline.RELIABILITY_GRID_COLS, GR = LineFollowerPipeline.RELIABILITY_GRID_ROWS;

        // Unreliable regions (glare, shadow, object, colour patch …)
        if (s.isCalibrated && s.cells > 0) {
            final float cw = mapScaleX / GC, ch = (bot - top) / GR;
            for (int gy = 0; gy < GR; gy++) {
                for (int gx = 0; gx < GC; gx++) {
                    if (s.grid[gy * GC + gx] == 0) continue;
                    final float x = mapOffX + gx * cw, y = top + gy * ch;
                    canvas.drawRect(x, y, x + cw, y + ch, unreliablePaint);
                }
            }
        }

        // ROI frame (yellow while calibrating)
        roiPaint.setColor(s.isCalibrated ? Color.WHITE : COLOR_CALIBRATING);
        roiPaint.setAlpha(170);
        canvas.drawRect(mx(0f) + 2f, top, mx(1f) - 2f, bot, roiPaint);

        // Path: segments coloured by where each point came from
        final int n = s.pCount;
        for (int i = 0; i + 1 < n; i++) {
            pathPaint.setColor(sourceColor(s.pSrc[i + 1]));
            canvas.drawLine(mx(s.pCol[i]), my(s.pRow[i]), mx(s.pCol[i + 1]), my(s.pRow[i + 1]), pathPaint);
        }
        for (int i = 0; i < n; i++) {
            pointPaint.setColor(sourceColor(s.pSrc[i]));
            canvas.drawCircle(mx(s.pCol[i]), my(s.pRow[i]), 6f, pointPaint);
        }

        // Look-ahead target and the steering arrow from the robot (bottom centre)
        final float baseX = mx(s.steerRefXFrac), baseY = bot;
        if (s.lookAheadValid) {
            final float tx = mx(s.lookX), ty = my(s.lookY);
            canvas.drawCircle(tx, ty, 22f, targetPaint);
            canvas.drawLine(baseX, baseY, tx, ty, targetPaint);
        }
        final int stateColor = !s.isCalibrated ? COLOR_CALIBRATING
                             : s.trackFound ? COLOR_TRACKING : COLOR_LOST;
        centroidPaint.setColor(stateColor);
        final float tipX = baseX + s.steering / 50f * 0.25f * mapScaleX;
        arrowPath.rewind();
        arrowPath.moveTo(tipX, baseY - 60f);
        arrowPath.lineTo(baseX - 18f, baseY - 10f);
        arrowPath.lineTo(baseX + 18f, baseY - 10f);
        arrowPath.close();
        canvas.drawPath(arrowPath, centroidPaint);

        // Labels
        canvas.drawText(s.isCalibrated ? "ROI" : "CALIBRATING", 8f, top + 34f, labelPaint);
        if (s.isCalibrated) {
            final String state = s.trackFound ? "PATH" : (s.lineHeld ? "MEMORY" : "LOST");
            canvas.drawText(state, 8f, top + 74f, labelPaint);
        }
    }

    private static int sourceColor(byte src) {
        switch (src) {
            case LineFollowerPipeline.PATH_SRC_FRESH:      return COLOR_FRESH;
            case LineFollowerPipeline.PATH_SRC_MEMORY:     return COLOR_MEMORY;
            case LineFollowerPipeline.PATH_SRC_UNRELIABLE: return COLOR_UNRELIABLE;
            default:                                       return COLOR_PREDICTED;
        }
    }

    /** v2 (Config.pathTracking = false): the ⊥ — H-ROI strip and the V-ROI look-ahead band. */
    private void drawT(Canvas canvas, Snapshot s, float W) {
        final float hTop = my(s.hRoiTopFrac);
        final float hBot = my(s.hRoiBotFrac);
        final float vTop = my(s.vRoiTopFrac);
        final float halfBot = s.halfBotFrac * mapScaleX;   // near (bottom) half-width
        final float halfTop = s.halfTopFrac * mapScaleX;   // far  (top)    half-width
        final float refX = mx(s.steerRefXFrac);
        final int   stateColor = !s.isCalibrated ? COLOR_CALIBRATING
                               : s.trackFound    ? COLOR_TRACKING : COLOR_LOST;

        // ── 1. V-ROI (magenta) — vertical leg of ⊥ ────────────────────────────
        final int     n       = s.laCount;
        final boolean dynamic = s.trackFound && n >= 2;
        final float   span    = Math.max(1f, hTop - vTop);   // near→far vertical extent

        if (dynamic) {
            bandPath.rewind();
            for (int i = 0; i < n; i++) {
                final float y = my(s.laRow[i]);
                final float x = mx(s.laCol[i]);
                final float half = halfBot + (halfTop - halfBot) * clamp01((hTop - y) / span);
                if (i == 0) bandPath.moveTo(x - half, y);
                else        bandPath.lineTo(x - half, y);
            }
            for (int i = n - 1; i >= 0; i--) {
                final float y = my(s.laRow[i]);
                final float x = mx(s.laCol[i]);
                final float half = halfBot + (halfTop - halfBot) * clamp01((hTop - y) / span);
                bandPath.lineTo(x + half, y);
            }
            bandPath.close();
            canvas.drawPath(bandPath, vFillPaint);
            canvas.drawPath(bandPath, vStrokePaint);
        } else {
            final float topX = mx(s.trackRefXFrac);
            final float blX = refX - halfBot, brX = refX + halfBot;   // bottom (near)
            final float tlX = topX - halfTop, trX = topX + halfTop;   // top    (far)
            vRoiPath.rewind();
            vRoiPath.moveTo(blX, hTop);
            vRoiPath.lineTo(brX, hTop);
            vRoiPath.lineTo(trX, vTop);
            vRoiPath.lineTo(tlX, vTop);
            vRoiPath.close();
            canvas.drawPath(vRoiPath, vFillPaint);
            canvas.drawPath(vRoiPath, vStrokePaint);

            final float arm = 22f;
            canvas.drawLine(blX - 2f, hTop, blX - 2f, hTop + arm, cornerPaint);
            canvas.drawLine(brX + 2f, hTop, brX + 2f, hTop + arm, cornerPaint);
            canvas.drawLine(tlX - 2f, vTop, tlX - 2f, vTop + arm, cornerPaint);
            canvas.drawLine(trX + 2f, vTop, trX + 2f, vTop + arm, cornerPaint);
        }

        // ── 2. H-ROI strip (cyan; yellow outline while calibrating) ───────────
        hStrokePaint.setColor(s.isCalibrated ? Color.CYAN : COLOR_CALIBRATING);
        hStrokePaint.setAlpha(230);
        canvas.drawRect(0f, hTop, W, hBot, hFillPaint);
        canvas.drawRect(0f, hTop, W, hBot, hStrokePaint);

        // ── 3. Calibrated straight-ahead reference (dashed white) ─────────────
        canvas.drawLine(refX, hTop - 20f, refX, hBot + 20f, refPaint);

        // ── 4. Centroid — follows the line through the curve ──────────────────
        centroidPaint.setColor(stateColor);
        final float cx;
        if (s.centroidXFrac >= 0f) {
            cx = mx(s.centroidXFrac);                  // real detected position
        } else {
            cx = mx(clamp01(s.steerRefXFrac + s.steering / 100f));
        }
        if (dynamic) {
            centerPath.rewind();
            for (int i = 0; i < n; i++) {
                final float y = my(s.laRow[i]);
                final float x = mx(s.laCol[i]);
                if (i == 0) centerPath.moveTo(x, y);
                else        centerPath.lineTo(x, y);
            }
            canvas.drawPath(centerPath, centroidPaint);
        } else if (s.centroidXFrac >= 0f) {
            canvas.drawLine(cx, hTop, cx, hBot, centroidPaint);
        }

        if (s.centroidXFrac >= 0f || s.isCalibrated) {
            final float tri = 18f;
            arrowPath.rewind();
            arrowPath.moveTo(cx, hTop - 4f);
            arrowPath.lineTo(cx - tri * 0.6f, hTop - tri);
            arrowPath.lineTo(cx + tri * 0.6f, hTop - tri);
            arrowPath.close();
            canvas.drawPath(arrowPath, centroidPaint);
        }

        // ── 5. Labels ─────────────────────────────────────────────────────────
        final float turnX = dynamic ? mx(s.laCol[n - 1]) - halfTop : mx(s.trackRefXFrac) - halfTop;
        canvas.drawText("TURN", turnX + 6f, vTop + 30f, labelPaint);
        canvas.drawText(s.isCalibrated ? "STEER" : "CALIBRATING", 6f, hTop - 10f, labelPaint);
    }

    /** Clamp to the unit interval [0,1]. */
    private static float clamp01(float v) {
        return v < 0f ? 0f : (v > 1f ? 1f : v);
    }
}
