package com.example.linefollower;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.Random;
import java.util.function.IntToDoubleFunction;

import org.junit.Test;

/**
 * Synthetic-frame tests: a floor of one colour with a line of another drawn on
 * it, rendered as ARGB and converted to grayscale / YUV 4:2:0 as needed.
 */
public class LineFollowerPipelineTest {

    private static final int W = 320, H = 240;

    // ── Scene rendering ─────────────────────────────────────────────────────

    /** A line of {@code widthPx} whose centre column at each row is {@code centerAt(row)}. */
    private static int[] scene(int floorRgb, int lineRgb, IntToDoubleFunction centerAt,
                               int widthPx, double noise, double gain, long seed) {
        final Random rnd = new Random(seed);
        final int[] px = new int[W * H];
        for (int row = 0; row < H; row++) {
            final double c = centerAt.applyAsDouble(row);
            for (int col = 0; col < W; col++) {
                final boolean onLine = Math.abs(col - c) <= widthPx / 2.0;
                final int base = onLine ? lineRgb : floorRgb;
                px[row * W + col] = 0xFF000000
                        | (ch(base >> 16, rnd, noise, gain) << 16)
                        | (ch(base >> 8,  rnd, noise, gain) << 8)
                        |  ch(base,       rnd, noise, gain);
            }
        }
        return px;
    }

    private static int ch(int v, Random rnd, double noise, double gain) {
        final double x = (v & 0xFF) * gain + rnd.nextGaussian() * noise;
        return (int) Math.max(0, Math.min(255, Math.round(x)));
    }

    private static int[] straight(int floorRgb, int lineRgb, int centerX, int widthPx) {
        return scene(floorRgb, lineRgb, r -> centerX, widthPx, 4, 1.0, centerX * 31L + widthPx);
    }

    private static byte[] toGray(int[] argb) {
        final byte[] g = new byte[argb.length];
        for (int i = 0; i < argb.length; i++) {
            final int c = argb[i];
            g[i] = (byte) ((77 * ((c >> 16) & 0xFF) + 150 * ((c >> 8) & 0xFF) + 29 * (c & 0xFF)) >> 8);
        }
        return g;
    }

    /** Returns {Y, U, V} with U/V averaged over 2×2 blocks (packed half-res planes). */
    private static byte[][] toYuv420(int[] argb) {
        final int cw = (W + 1) / 2, chH = (H + 1) / 2;
        final byte[] y = toGray(argb);
        final byte[] u = new byte[cw * chH], v = new byte[cw * chH];
        for (int r = 0; r < chH; r++) {
            for (int c = 0; c < cw; c++) {
                int su = 0, sv = 0, n = 0;
                for (int dr = 0; dr < 2; dr++) {
                    for (int dc = 0; dc < 2; dc++) {
                        final int rr = 2 * r + dr, cc = 2 * c + dc;
                        if (rr >= H || cc >= W) continue;
                        final int p = argb[rr * W + cc];
                        final int R = (p >> 16) & 0xFF, G = (p >> 8) & 0xFF, B = p & 0xFF;
                        su += ((-43 * R - 85 * G + 128 * B) >> 8) + 128;
                        sv += ((128 * R - 107 * G - 21 * B) >> 8) + 128;
                        n++;
                    }
                }
                u[r * cw + c] = (byte) (su / n);
                v[r * cw + c] = (byte) (sv / n);
            }
        }
        return new byte[][]{y, u, v};
    }

    // ── Format-agnostic driver ──────────────────────────────────────────────

    private enum Fmt { GRAY, YUV, ARGB }

    private static boolean calibrate(LineFollowerPipeline p, Fmt f, int[] argb) {
        switch (f) {
            case GRAY: return p.calibrate(toGray(argb), W, H);
            case YUV:  { byte[][] yuv = toYuv420(argb); return p.calibrateYuv420(yuv[0], yuv[1], yuv[2], W, H); }
            default:   return p.calibrateArgb(argb, W, H);
        }
    }

    private static int[] process(LineFollowerPipeline p, Fmt f, int[] argb) {
        final int[] r;
        switch (f) {
            case GRAY: r = p.processFrame(toGray(argb), W, H); break;
            case YUV:  { byte[][] yuv = toYuv420(argb); r = p.processFrameYuv420(yuv[0], yuv[1], yuv[2], W, H); break; }
            default:   r = p.processFrameArgb(argb, W, H);
        }
        return r.clone();
    }

    /** Calibrate on a centred line, then check centred / right / left steering. */
    private static void assertFollows(Fmt f, int floorRgb, int lineRgb) {
        final LineFollowerPipeline p = new LineFollowerPipeline();
        final int[] centred = straight(floorRgb, lineRgb, W / 2, 30);
        int frames = 0;
        while (!calibrate(p, f, centred)) {
            assertTrue("calibration did not finish", ++frames < 50);
        }
        assertTrue(p.isColorModelReady());

        final int[] c = process(p, f, centred);
        assertEquals("centred steering", 0, c[0], 3);
        assertTrue("straight road should give high tracking: " + c[1], c[1] > 90);

        final int[] right = process(p, f, straight(floorRgb, lineRgb, W / 2 + 60, 30));
        assertEquals("right steering", 37, right[0], 4);

        // Back through the centre: a 120 px jump in one frame is outside the
        // line-lock gate and is (correctly) treated as leaving the line.
        process(p, f, centred);
        final int[] left = process(p, f, straight(floorRgb, lineRgb, W / 2 - 60, 30));
        assertEquals("left steering", -37, left[0], 4);
    }

    private static int rgb(int r, int g, int b) { return (r << 16) | (g << 8) | b; }

    private static final int BLACK = rgb(20, 20, 20);
    private static final int WHITE = rgb(235, 235, 235);
    private static final int BLUE  = rgb(30, 60, 200);
    private static final int BROWN = rgb(120, 70, 30);
    // Same brightness (Y ≈ 88), different hue: only colour input can separate these.
    private static final int RED   = rgb(200, 40, 40);
    private static final int GREEN = rgb(40, 122, 40);

    // ── Colour combinations ─────────────────────────────────────────────────

    @Test public void whiteOnBlack_allFormats() {
        for (Fmt f : Fmt.values()) assertFollows(f, BLACK, WHITE);
    }

    @Test public void blackOnBlue_allFormats() {
        for (Fmt f : Fmt.values()) assertFollows(f, BLUE, BLACK);
    }

    @Test public void whiteOnBrown_allFormats() {
        for (Fmt f : Fmt.values()) assertFollows(f, BROWN, WHITE);
    }

    @Test public void darkOnWhite_allFormats() {
        for (Fmt f : Fmt.values()) assertFollows(f, WHITE, BLACK);
    }

    @Test public void redOnGreen_equalBrightness_colourInput() {
        assertFollows(Fmt.YUV,  GREEN, RED);
        assertFollows(Fmt.ARGB, GREEN, RED);
    }

    @Test public void redOnGreen_equalBrightness_grayscaleCannotCalibrate() {
        final LineFollowerPipeline p = new LineFollowerPipeline();
        final int[] issue = {-1};
        p.setFrameCallback(d -> issue[0] = d.calibIssue);
        final byte[] g = toGray(straight(GREEN, RED, W / 2, 30));
        for (int i = 0; i < 30; i++) assertFalse(p.calibrate(g, W, H));
        assertEquals(LineFollowerPipeline.ISSUE_LOW_CONTRAST, issue[0]);
    }

    @Test public void learnedColoursAreReported() {
        final LineFollowerPipeline p = new LineFollowerPipeline();
        final int[] img = straight(BROWN, WHITE, W / 2, 30);
        while (!p.calibrateArgb(img, W, H)) { /* calibrating */ }
        assertColorNear(WHITE, p.getLineColorArgb());
        assertColorNear(BROWN, p.getFloorColorArgb());
    }

    private static void assertColorNear(int expected, int actual) {
        for (int s = 0; s <= 16; s += 8) {
            assertEquals("channel " + s, (expected >> s) & 0xFF, (actual >> s) & 0xFF, 12);
        }
    }

    // ── Calibration robustness ──────────────────────────────────────────────

    @Test public void uniformFloorNeverCalibrates() {
        final LineFollowerPipeline p = new LineFollowerPipeline();
        final int[] issue = {-1};
        p.setFrameCallback(d -> issue[0] = d.calibIssue);
        final int[] img = scene(BLUE, BLUE, r -> -1000, 1, 4, 1.0, 7);
        for (int i = 0; i < 30; i++) assertFalse(p.calibrateArgb(img, W, H));
        assertEquals(LineFollowerPipeline.ISSUE_LOW_CONTRAST, issue[0]);
    }

    @Test public void wideLineIsStillTheLine() {
        // The line covers 60 % of the strip — more than the floor. The floor is
        // still identified because it is the colour on both frame edges.
        final LineFollowerPipeline p = new LineFollowerPipeline();
        final int[] img = straight(BLACK, WHITE, W / 2, (int) (W * 0.6));
        while (!p.calibrateArgb(img, W, H)) { /* calibrating */ }
        assertColorNear(WHITE, p.getLineColorArgb());
    }

    @Test public void cameraOffsetIsCalibratedOut() {
        // Line sits 40 px right of image centre when the robot is correctly placed.
        final LineFollowerPipeline p = new LineFollowerPipeline();
        final int[] img = straight(BLACK, WHITE, W / 2 + 40, 30);
        while (!p.calibrateArgb(img, W, H)) { /* calibrating */ }
        assertEquals(0, p.processFrameArgb(img, W, H)[0], 3);
        assertEquals((W / 2 + 40) / (float) W, p.getSteerRefXFrac(), 0.02f);
    }

    @Test public void presetColoursSkipColourLearning() {
        final LineFollowerPipeline p = new LineFollowerPipeline();
        p.setTrackColors(0xFF000000 | BLACK, 0xFF000000 | BLUE);
        final int[] phase = {-1};
        p.setFrameCallback(d -> phase[0] = d.calibPhase);
        p.calibrateArgb(straight(BLUE, BLACK, W / 2, 30), W, H);
        assertEquals(LineFollowerPipeline.CALIB_FINDING_CENTER, phase[0]);
        assertTrue(p.isColorModelReady());
    }

    @Test public void recalibrationLearnsNewColours() {
        final LineFollowerPipeline p = new LineFollowerPipeline();
        final int[] a = straight(BLACK, WHITE, W / 2, 30);
        while (!p.calibrateArgb(a, W, H)) { /* calibrating */ }
        p.requestRecalibration();
        assertFalse(p.isCalibrated());
        assertFalse(p.isColorModelReady());
        final int[] b = straight(BLUE, BLACK, W / 2, 30);
        while (!p.calibrateArgb(b, W, H)) { /* calibrating */ }
        assertColorNear(BLACK, p.getLineColorArgb());
        assertEquals(0, p.processFrameArgb(b, W, H)[0], 3);
    }

    @Test public void classicGrayscaleModeStillWorks() {
        final LineFollowerPipeline p = new LineFollowerPipeline();
        final LineFollowerPipeline.Config cfg = new LineFollowerPipeline.Config();
        cfg.autoColor = false;
        cfg.trackIsBright = true;
        p.configure(cfg);
        final byte[] g = toGray(straight(BLACK, WHITE, W / 2, 30));
        assertTrue("no colour phase — finishes after calibFrames", calibrateN(p, g, cfg.calibFrames));
        assertFalse(p.isColorModelReady());
        assertEquals(37, p.processFrame(toGray(straight(BLACK, WHITE, W / 2 + 60, 30)), W, H)[0], 4);
    }

    private static boolean calibrateN(LineFollowerPipeline p, byte[] g, int n) {
        boolean done = false;
        for (int i = 0; i < n; i++) done = p.calibrate(g, W, H);
        return done;
    }

    // ── Runtime behaviour ───────────────────────────────────────────────────

    @Test public void survivesLightingChangeAfterCalibration() {
        final LineFollowerPipeline p = new LineFollowerPipeline();
        final int[] img = straight(BROWN, WHITE, W / 2, 30);
        while (!p.calibrateArgb(img, W, H)) { /* calibrating */ }
        final int[] dim = scene(BROWN, WHITE, r -> W / 2 + 60, 30, 4, 0.6, 99);
        assertEquals(37, p.processFrameArgb(dim, W, H)[0], 4);
    }

    @Test public void hardTurnKeepsTrackFound() {
        // Near strip straight, look-ahead bends hard to the right and off the frame.
        final LineFollowerPipeline p = new LineFollowerPipeline();
        final int[] img = straight(BLACK, WHITE, W / 2, 30);
        while (!p.calibrateArgb(img, W, H)) { /* calibrating */ }

        final LineFollowerPipeline.FrameData[] last = {null};
        p.setFrameCallback(d -> last[0] = d);
        final int[] turn = scene(BLACK, WHITE, r -> {
            final double t = Math.max(0, (180 - r) / 120.0);
            return W / 2 + 220 * t * t;
        }, 30, 4, 1.0, 5);
        final int[] out = p.processFrameArgb(turn, W, H);
        assertTrue("turn should lower tracking: " + out[1], out[1] < 60);
        assertTrue("line still in view → trackFound", last[0].trackFound);
        assertTrue("look-ahead path follows the curve", last[0].lookAheadPointCount > 5);
        assertTrue("curve leans right", last[0].tiltDeg > 0);
    }

    @Test public void reportsRealCentroidAndWidth() {
        final LineFollowerPipeline p = new LineFollowerPipeline();
        final int[] img = straight(BLACK, WHITE, W / 2, 30);
        while (!p.calibrateArgb(img, W, H)) { /* calibrating */ }

        final LineFollowerPipeline.FrameData[] last = {null};
        p.setFrameCallback(d -> last[0] = d);
        // Line far right, past the ±100 steering clamp of the old derived centroid.
        p.processFrameArgb(straight(BLACK, WHITE, 300, 30), W, H);
        assertEquals(300f / W, last[0].centroidXFrac, 0.02f);
        // Width is measured without the bridged gap columns (31 ± 2 px).
        assertEquals(31f / W, last[0].trackWidthFrac, 2f / W);
    }

    @Test public void lostLineReportsTrackLost() {
        final LineFollowerPipeline p = new LineFollowerPipeline();
        final int[] img = straight(BLACK, WHITE, W / 2, 30);
        while (!p.calibrateArgb(img, W, H)) { /* calibrating */ }
        p.processFrameArgb(img, W, H);
        final int[] out = p.processFrameArgb(scene(BLACK, BLACK, r -> 0, 1, 4, 1.0, 3), W, H);
        assertEquals(2, out[1]);
    }

    @Test(expected = IllegalArgumentException.class)
    public void rejectsShortBuffer() {
        new LineFollowerPipeline().processFrame(new byte[10], W, H);
    }
}
