package com.example.firstproject;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.DashPathEffect;
import android.graphics.Paint;
import android.graphics.Path;
import android.util.AttributeSet;
import android.view.View;

/**
 * Transparent overlay drawn on top of the camera PreviewView.
 *
 * Shows the inverted-T (⊥) virtual sensor so you can position the phone:
 *
 *   ┌──────────────────────────────────────────┐
 *   │   ║                              ║       │
 *   │   ║   V-ROI  (magenta box)       ║       │  fixed at frame centre
 *   │   ║   Track-continuity sensor    ║       │
 *   │   ║                              ║       │
 *   │   ║         │ centroid           ║       │  green/orange — moves with line
 *   │   ║         │                   ║       │
 *   │   ╚═════════╪═══════════════════╝       │
 *   ╠═════════════╪═════════════════════════════╣  H-ROI (cyan) — full-width strip
 *   │    · · · · ·┼· · · · · · · · ·           │  dashed white = frame centre ref
 *   └──────────────────────────────────────────┘
 *
 * The T is FIXED at the frame centre so you can use it to aim the camera.
 * When the road line sits inside the T and the green centroid line is in the
 * middle of the H-ROI, steering = 0 and the camera is correctly positioned.
 *
 * {@link #update} is safe to call from any thread.
 */
public final class OverlayView extends View {

    // ── Paints (pre-allocated — never created inside onDraw) ──────────────────

    private final Paint hFillPaint    = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint hStrokePaint  = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint vFillPaint    = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint vStrokePaint  = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint centroidPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint centerRefPaint= new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint cornerPaint   = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint labelPaint    = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint ignorePaint   = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint ignoreLinePaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Path  arrowPath    = new Path();  // reused in onDraw — no allocation
    private final Path  vRoiPath     = new Path();  // reused — sheared V-ROI outline
    private final Path  bandPath     = new Path();  // reused — dynamic (curved) V-ROI band
    private final Path  centerPath   = new Path();  // reused — dynamic centroid poly-line

    // ── Geometry set by update() — fractional [0,1] of frame dimensions ───────

    // H-ROI: fixed Y position (matches pipeline's hRoiCenterYFrac)
    private volatile float hRoiTopFrac = 0.82f;
    private volatile float hRoiBotFrac = 0.88f;

    // V-ROI: a perspective TRAPEZOID. Centred horizontally at vRoiCenterFrac, with
    // a half-width that tapers from vRoiHalfBotFrac at the near (H-ROI) edge to the
    // smaller vRoiHalfTopFrac at the far (top) edge — modelling how the fixed-width
    // track projects narrower the farther ahead the tilted camera looks.
    private volatile float vRoiCenterFrac  = 0.50f;
    private volatile float vRoiHalfBotFrac = 0.15f;
    private volatile float vRoiHalfTopFrac = 0.05f;
    // Top (far) edge of the V-ROI as a fraction of frame height. The look-ahead no
    // longer spans to the frame top (0f); it stops short so the noisy far region is
    // excluded. Matches the pipeline's vRoiTop.
    private volatile float vRoiTopFrac     = 0.40f;

    // Centroid: dynamic — moves left/right as the detected line moves
    private volatile float   centroidXFrac = 0.50f;
    private volatile boolean trackFound    = false;

    // Tilt shear: horizontal offset (fraction of frame width) applied to the TOP
    // edge of the V-ROI relative to its bottom, so the box leans with the line.
    private volatile float   vRoiShearFrac = 0f;

    // Dynamic look-ahead path (nearest → farthest), fractional image coords. When
    // present (laPointCount ≥ 2) the V-ROI is drawn as a band that BENDS along these
    // points — following the real tape through a curve — instead of a straight box.
    private final float[]  laColFrac   = new float[48];
    private final float[]  laRowFrac   = new float[48];
    private volatile int   laPointCount = 0;

    // Bottom band (fraction of frame height) that is EXCLUDED from all processing
    // because the robot's own body is visible there. Drawn as a dim shaded strip
    // so the exclusion line can be tuned until the robot is fully covered.
    private volatile float   bottomIgnoreFrac = 0f;

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
        // H-ROI — cyan fill + stroke
        hFillPaint.setStyle(Paint.Style.FILL);
        hFillPaint.setColor(Color.CYAN);
        hFillPaint.setAlpha(45);

        hStrokePaint.setStyle(Paint.Style.STROKE);
        hStrokePaint.setColor(Color.CYAN);
        hStrokePaint.setStrokeWidth(4f);
        hStrokePaint.setAlpha(230);

        // V-ROI — magenta fill + stroke
        vFillPaint.setStyle(Paint.Style.FILL);
        vFillPaint.setColor(Color.MAGENTA);
        vFillPaint.setAlpha(35);

        vStrokePaint.setStyle(Paint.Style.STROKE);
        vStrokePaint.setColor(Color.MAGENTA);
        vStrokePaint.setStrokeWidth(4f);
        vStrokePaint.setAlpha(230);

        // Centroid vertical line — green when tracking, orange when lost
        centroidPaint.setStyle(Paint.Style.STROKE);
        centroidPaint.setStrokeWidth(5f);

        // Frame-centre dashed reference
        centerRefPaint.setStyle(Paint.Style.STROKE);
        centerRefPaint.setColor(Color.WHITE);
        centerRefPaint.setAlpha(110);
        centerRefPaint.setStrokeWidth(2f);
        centerRefPaint.setPathEffect(new DashPathEffect(new float[]{14f, 9f}, 0f));

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
    }

    /** Set the excluded bottom band (fraction of frame height) and redraw. */
    public void setBottomIgnoreFrac(float frac) {
        this.bottomIgnoreFrac = frac;
        postInvalidate();
    }

    /**
     * Supply the dynamic look-ahead path (nearest → farthest, fractional image
     * coords) that the purple V-ROI band should bend along. The arrays are COPIED,
     * so the caller may reuse them on the next frame. Call before {@link #update}.
     */
    public void setLookAheadPath(float[] col, float[] row, int count) {
        final int n = Math.max(0, Math.min(count, laColFrac.length));
        System.arraycopy(col, 0, laColFrac, 0, n);
        System.arraycopy(row, 0, laRowFrac, 0, n);
        laPointCount = n;
    }

    // =========================================================================
    //  Call from the camera analysis thread
    // =========================================================================

    /**
     * Update sensor geometry and schedule a redraw.  Safe to call from any thread.
     *
     * The V-ROI is a perspective trapezoid: centred at {@code vRoiCenterFrac} with
     * a near (bottom) half-width {@code vRoiHalfBotFrac} that tapers to the smaller
     * far (top) half-width {@code vRoiHalfTopFrac}, so it matches how the track
     * narrows with distance under the camera's tilt.
     */
    public void update(float hRoiTopFrac, float hRoiBotFrac,
                       float vRoiTopFrac, float vRoiCenterFrac,
                       float vRoiHalfBotFrac, float vRoiHalfTopFrac,
                       float centroidXFrac, float vRoiShearFrac,
                       boolean trackFound) {
        this.hRoiTopFrac     = hRoiTopFrac;
        this.hRoiBotFrac     = hRoiBotFrac;
        this.vRoiTopFrac     = vRoiTopFrac;
        this.vRoiCenterFrac  = vRoiCenterFrac;
        this.vRoiHalfBotFrac = vRoiHalfBotFrac;
        this.vRoiHalfTopFrac = vRoiHalfTopFrac;
        this.centroidXFrac   = centroidXFrac;
        this.vRoiShearFrac   = vRoiShearFrac;
        this.trackFound      = trackFound;
        postInvalidate();
    }

    // =========================================================================
    //  Drawing — no allocations here
    // =========================================================================

    @Override
    protected void onDraw(Canvas canvas) {
        final float W = getWidth();
        final float H = getHeight();

        final float hTop = hRoiTopFrac   * H;
        final float hBot = hRoiBotFrac   * H;
        final float vTop = vRoiTopFrac   * H;        // far (top) edge of the V-ROI
        final float vC   = vRoiCenterFrac * W;
        final float halfBot = vRoiHalfBotFrac * W;   // near (bottom) half-width
        final float halfTop = vRoiHalfTopFrac * W;   // far  (top)    half-width
        final float cx   = centroidXFrac * W;
        final float cxMid = W * 0.5f;
        // Horizontal offset of the V-ROI top edge so the box leans with the line.
        final float shear = vRoiShearFrac * W;

        // Trapezoid corners: wide near edge at the H-ROI, narrow far edge at the top.
        final float blX = vC - halfBot,          brX = vC + halfBot;          // bottom
        final float tlX = vC + shear - halfTop,  trX = vC + shear + halfTop;  // top

        // ── 0. Ignored bottom band (red) — robot body, excluded from processing ─
        if (bottomIgnoreFrac > 0f) {
            final float ignoreTop = (1f - bottomIgnoreFrac) * H;
            canvas.drawRect(0f, ignoreTop, W, H, ignorePaint);
            canvas.drawLine(0f, ignoreTop, W, ignoreTop, ignoreLinePaint);
            canvas.drawText("IGNORED (robot)", 8f, ignoreTop + 34f, labelPaint);
        }

        // ── 1. V-ROI column (magenta) — vertical leg of ⊥ ─────────────────────
        // When a live look-ahead path is available we draw the V-ROI as a BAND that
        // bends along the detected tape (curves with the road); otherwise we fall
        // back to the straight perspective trapezoid (before first detection / lost).
        final int   n       = laPointCount;
        final boolean dynamic = trackFound && n >= 2;
        final float span    = (hTop - vTop);         // near→far vertical extent (>0)

        if (dynamic) {
            // Band outline: left edge nearest→farthest, then right edge back. The
            // half-width tapers per row between the near (wide) and far (narrow) ends.
            bandPath.rewind();
            for (int i = 0; i < n; i++) {
                final float y = laRowFrac[i] * H;
                final float x = laColFrac[i] * W;
                final float t = clamp01((hTop - y) / span);
                final float half = halfBot + (halfTop - halfBot) * t;
                if (i == 0) bandPath.moveTo(x - half, y);
                else        bandPath.lineTo(x - half, y);
            }
            for (int i = n - 1; i >= 0; i--) {
                final float y = laRowFrac[i] * H;
                final float x = laColFrac[i] * W;
                final float t = clamp01((hTop - y) / span);
                final float half = halfBot + (halfTop - halfBot) * t;
                bandPath.lineTo(x + half, y);
            }
            bandPath.close();
            canvas.drawPath(bandPath, vFillPaint);
            canvas.drawPath(bandPath, vStrokePaint);
        } else {
            vRoiPath.rewind();
            vRoiPath.moveTo(blX, hTop);           // bottom-left  (at the H-ROI, near)
            vRoiPath.lineTo(brX, hTop);           // bottom-right
            vRoiPath.lineTo(trX, vTop);           // top-right    (narrow + leaned, far)
            vRoiPath.lineTo(tlX, vTop);           // top-left
            vRoiPath.close();
            canvas.drawPath(vRoiPath, vFillPaint);
            canvas.drawPath(vRoiPath, vStrokePaint);

            // T-junction corner accents — only meaningful for the straight guide box.
            final float arm = 22f;
            canvas.drawLine(blX - 2f, hTop, blX - 2f, hTop + arm, cornerPaint);
            canvas.drawLine(brX + 2f, hTop, brX + 2f, hTop + arm, cornerPaint);
            canvas.drawLine(tlX - 2f, vTop, tlX - 2f, vTop + arm, cornerPaint);
            canvas.drawLine(trX + 2f, vTop, trX + 2f, vTop + arm, cornerPaint);
        }

        // ── 2. H-ROI strip (cyan) — horizontal bar of ⊥ ──────────────────────
        canvas.drawRect(0f, hTop, W, hBot, hFillPaint);
        canvas.drawRect(0f, hTop, W, hBot, hStrokePaint);

        // ── 4. Frame-centre dashed reference (white) ─────────────────────────
        canvas.drawLine(cxMid, hTop, cxMid, hBot, centerRefPaint);

        // ── 5. Centroid (dynamic) — follows the tape through the curve ─────────
        centroidPaint.setColor(trackFound
                ? Color.parseColor("#00E676")    // bright green = tracking
                : Color.parseColor("#FF6D00"));  // orange = lost
        if (dynamic) {
            centerPath.rewind();
            for (int i = 0; i < n; i++) {
                final float y = laRowFrac[i] * H;
                final float x = laColFrac[i] * W;
                if (i == 0) centerPath.moveTo(x, y);
                else        centerPath.lineTo(x, y);
            }
            canvas.drawPath(centerPath, centroidPaint);
        } else {
            canvas.drawLine(cx, vTop, cx, hBot, centroidPaint);
        }

        // Small triangle pointer at the H-ROI to highlight the near centroid.
        final float tri = 18f;
        arrowPath.rewind();
        arrowPath.moveTo(cx, hTop - 4f);
        arrowPath.lineTo(cx - tri * 0.6f, hTop - tri);
        arrowPath.lineTo(cx + tri * 0.6f, hTop - tri);
        arrowPath.close();
        canvas.drawPath(arrowPath, centroidPaint);   // reuse same colour paint

        // ── 6. Labels ─────────────────────────────────────────────────────────
        // Anchor "TURN" to the far end of whichever V-ROI shape we drew.
        final float turnX = dynamic ? laColFrac[n - 1] * W - halfTop : tlX;
        canvas.drawText("TURN", turnX + 6f, vTop + 30f, labelPaint);
        canvas.drawText("STEER", 6f, hTop - 10f, labelPaint);
    }

    /** Clamp to the unit interval [0,1]. */
    private static float clamp01(float v) {
        return v < 0f ? 0f : (v > 1f ? 1f : v);
    }
}