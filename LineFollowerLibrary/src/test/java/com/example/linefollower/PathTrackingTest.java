package com.example.linefollower;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.function.Consumer;

import org.junit.Test;

/**
 * v3 path tracking — one regression test per scenario of the v3 work (see
 * _investigation/CHANGES_v3.md). Rendered with {@link TrackSim} (320 × 240,
 * camera 15 cm high, tilted 45°, app configuration). Disturbance tests drive the
 * same scripted poses twice — on the clean floor and with the disturbance — and
 * require STEER and POWER to stay (nearly) identical.
 */
public class PathTrackingTest {

    private static final int W = 320, H = 240;
    // colour pairs: {floor, line}
    private static final int[] BLACK_ON_WOOD  = {0x8C5F3C, 0x1E1C1A};
    private static final int[] WHITE_ON_DARK  = {0x202020, 0xEBEBEB};
    private static final int[] DARK_ON_LIGHT  = {0xE2E2DE, 0x1A1A1C};
    private static final int[] RED_ON_GREEN   = {0x287A28, 0xC82828};   // equal brightness
    private static final int[][] PAIRS = {BLACK_ON_WOOD, WHITE_ON_DARK, DARK_ON_LIGHT, RED_ON_GREEN};

    // Disturbance criteria (from the task): STEER and POWER nearly identical to the clean run.
    private static final int MAX_STEER_DIFF = 3, MAX_POWER_DROP = 10;
    private static final double MEAN_STEER_DIFF = 1.0, MEAN_POWER_DROP = 3.0;

    // ── driving helpers ───────────────────────────────────────────────────

    /** Result of one scripted drive. */
    private static final class Run {
        int[] steer, power; boolean[] found;
        LineFollowerPipeline.FrameData last;
        LineFollowerPipeline pipeline;
    }

    private static TrackSim sim(int[] pair, TrackSim.Path path) {
        final TrackSim s = new TrackSim(W, H);
        s.floorRgb = pair[0]; s.lineRgb = pair[1];
        s.tracks.add(path.points());
        return s;
    }

    private static boolean calibrate(LineFollowerPipeline p, TrackSim sim, double[] pose) {
        for (int i = 0; i < 80; i++) {
            final byte[][] yuv = sim.toYuv420(sim.render(pose[0], pose[1], pose[2]));
            if (p.calibrateYuv420(yuv[0], yuv[1], yuv[2], W, H)) return true;
        }
        return false;
    }

    /**
     * Calibrate at path index {@code cal}, then drive along the path (camera on the
     * line, tangent) from index {@code from} to {@code to}, {@code step} samples
     * (5 mm each) per frame; frames before {@code measureFrom} are not recorded.
     */
    private static Run drive(TrackSim sim, TrackSim.Path path, int cal, int from, int to, int step, int measureFrom,
                             Consumer<LineFollowerPipeline.Config> cfgEdit) {
        final LineFollowerPipeline p = new LineFollowerPipeline();
        final LineFollowerPipeline.Config cfg = TrackSim.appConfig();
        if (cfgEdit != null) cfgEdit.accept(cfg);
        p.configure(cfg);
        final LineFollowerPipeline.FrameData[] last = {null};
        p.setFrameCallback(d -> last[0] = d);
        assertTrue("calibration", calibrate(p, sim, path.pose(cal)));
        final int n = Math.max(0, (to - measureFrom) / step + 1);
        final Run r = new Run();
        r.steer = new int[n]; r.power = new int[n]; r.found = new boolean[n];
        int k = 0;
        for (int i = from; i <= to && i < path.size(); i += step) {
            final double[] pose = path.pose(i);
            final byte[][] yuv = sim.toYuv420(sim.render(pose[0], pose[1], pose[2]));
            final int[] o = p.processFrameYuv420(yuv[0], yuv[1], yuv[2], W, H);
            if (i >= measureFrom && k < n) { r.steer[k] = o[0]; r.power[k] = o[1]; r.found[k] = last[0].trackFound || last[0].lineHeld; k++; }
        }
        r.steer = java.util.Arrays.copyOf(r.steer, k); r.power = java.util.Arrays.copyOf(r.power, k); r.found = java.util.Arrays.copyOf(r.found, k);
        r.last = last[0]; r.pipeline = p;
        return r;
    }

    /** Clean vs disturbed drive on a path: STEER / POWER must stay nearly identical. */
    private static void assertUndisturbed(String what, int[] pair, TrackSim.Path path, int cal, int from, int to, int measureFrom,
                                          Consumer<TrackSim> disturbance, boolean disturbDuringCalibration) {
        final TrackSim clean = sim(pair, path), dirty = sim(pair, path);
        disturbance.accept(dirty);
        if (!disturbDuringCalibration) dirty.effectsOn = false;
        final Run c = drive(clean, path, cal, from, from, 1, Integer.MAX_VALUE, null);   // (calibrated pipeline, unused)
        final LineFollowerPipeline pc = new LineFollowerPipeline(), pd = new LineFollowerPipeline();
        pc.configure(TrackSim.appConfig()); pd.configure(TrackSim.appConfig());
        assertTrue(what + ": clean calibration", calibrate(pc, clean, path.pose(cal)));
        assertTrue(what + ": calibration with the disturbance", calibrate(pd, dirty, path.pose(cal)));
        dirty.effectsOn = true;
        double sumDS = 0, sumDrop = 0; int maxDS = 0, maxDrop = 0, n = 0;
        for (int i = from; i <= to; i += 2) {
            final double[] pose = path.pose(i);
            final byte[][] a = clean.toYuv420(clean.render(pose[0], pose[1], pose[2]));
            final int[] oc = pc.processFrameYuv420(a[0], a[1], a[2], W, H).clone();
            final byte[][] b = dirty.toYuv420(dirty.render(pose[0], pose[1], pose[2]));
            final int[] od = pd.processFrameYuv420(b[0], b[1], b[2], W, H).clone();
            if (i < measureFrom) continue;
            final int ds = Math.abs(od[0] - oc[0]), drop = oc[1] - od[1];
            maxDS = Math.max(maxDS, ds); maxDrop = Math.max(maxDrop, drop); sumDS += ds; sumDrop += Math.max(0, drop); n++;
        }
        assertTrue(what + " measured frames", n > 5 && c != null);
        final String v = String.format(" (max |dS| %d, mean %.2f, max power drop %d, mean %.2f)", maxDS, sumDS / n, maxDrop, sumDrop / n);
        assertTrue(what + ": steering changed" + v, maxDS <= MAX_STEER_DIFF && sumDS / n <= MEAN_STEER_DIFF);
        assertTrue(what + ": power dropped" + v, maxDrop <= MAX_POWER_DROP && sumDrop / n <= MEAN_POWER_DROP);
    }

    private static TrackSim.Path straight() { return new TrackSim.Path(0, 0, 0).straight(2.0); }

    // ── geometry scenarios ────────────────────────────────────────────────

    @Test public void straightLine_zeroSteeringFullPower() {
        final TrackSim.Path path = straight();
        final Run r = drive(sim(BLACK_ON_WOOD, path), path, 10, 10, 200, 3, 40, null);
        for (int i = 0; i < r.steer.length; i++) {
            assertEquals("steering on a straight line", 0, r.steer[i], 1);
            assertEquals("power on a straight line", 100, r.power[i], 3);
            assertTrue(r.found[i]);
        }
        final LineFollowerPipeline.FrameData d = r.last;
        assertTrue("v3 mode", d.pathMode);
        assertTrue("path traced", d.pathPointCount >= 10 && d.pathFreshPoints >= 10);
        assertTrue("ROI inside the frame", d.roiTopFrac >= 0f && d.roiTopFrac < d.roiBotFrac && d.roiBotFrac <= 1f);
        assertTrue("look-ahead point straight ahead", d.lookAheadValid && Math.abs(d.lookAheadLat) < 0.05f);
        assertEquals("camera tilt learned close to the real 45°", 45f, d.cameraTiltUsedDeg, 6f);
    }

    /** Curve of radius r (m), angle (deg, + = left): steering into the curve, lower power, back to 0 after. */
    private static void assertCurve(double r, double angle) {
        final TrackSim.Path path = new TrackSim.Path(0, 0, 0).straight(0.6).arc(r, angle).straight(0.8);
        final int arcStart = 120, arcEnd = arcStart + (int) Math.round(Math.abs(Math.toRadians(angle)) * r / 0.005);
        final Run run = drive(sim(BLACK_ON_WOOD, path), path, 10, 10, path.size() - 100, 2, 10, null);
        final int sign = angle > 0 ? -1 : 1;     // left curve → negative steering
        int into = 0, minPower = 100;
        for (int i = 0; i < run.steer.length; i++) {
            final int idx = 10 + 2 * i;
            if (idx > arcStart + 10 && idx < arcEnd) { if (sign * run.steer[i] > 2) into++; minPower = Math.min(minPower, run.power[i]); }
            assertTrue("line kept (found or followed from memory) at " + idx, run.found[i] || idx > arcEnd);
        }
        assertTrue("steers into the curve (" + into + " frames)", into >= (arcEnd - arcStart) / 4);
        if (r < 0.3) assertTrue("power lower in a sharp curve: " + minPower, minPower < 80);
        for (int i = run.steer.length - 5; i < run.steer.length; i++) assertEquals("straight again after the curve", 0, run.steer[i], 2);
    }

    @Test public void gentleCurveLeft()  { assertCurve(0.6, 90); }
    @Test public void gentleCurveRight() { assertCurve(0.6, -90); }
    @Test public void sharpCurveLeft()   { assertCurve(0.15, 90); }
    @Test public void sharpCurveRight()  { assertCurve(0.15, -90); }

    @Test public void sCurve_steeringFollowsBothBends() {
        final TrackSim.Path path = new TrackSim.Path(0, 0, 0).straight(0.6).arc(0.3, 90).arc(0.3, -90).straight(0.6);
        final Run run = drive(sim(BLACK_ON_WOOD, path), path, 10, 10, path.size() - 100, 2, 10, null);
        boolean left = false, right = false;
        for (int s : run.steer) { if (s <= -3) left = true; if (s >= 3 && left) right = true; }
        assertTrue("left bend then right bend: " + java.util.Arrays.toString(run.steer), left && right);
    }

    @Test public void lineLeavesTheSide_fullTurnTowardIt() {
        // robot drifts and turns away from the line until it leaves the image on the right
        final TrackSim.Path path = straight();
        final TrackSim s = sim(BLACK_ON_WOOD, path);
        final LineFollowerPipeline p = new LineFollowerPipeline();
        p.configure(TrackSim.appConfig());
        final LineFollowerPipeline.FrameData[] last = {null};
        p.setFrameCallback(d -> last[0] = d);
        assertTrue(calibrate(p, s, path.pose(10)));
        boolean sawPin = false;
        for (int k = 0; k <= 40; k++) {
            final double[] pose = path.pose(20 + k);
            final double lat = 0.005 * k, yaw = Math.toRadians(1.5 * k);         // robot moves left and turns left
            final byte[][] yuv = s.toYuv420(s.render(pose[0] - lat * Math.sin(pose[2]), pose[1] + lat * Math.cos(pose[2]), pose[2] + yaw));
            final int[] o = p.processFrameYuv420(yuv[0], yuv[1], yuv[2], W, H);
            if (!last[0].trackFound && !last[0].lineHeld) { assertEquals("full turn toward the line (right)", 50, o[0]); sawPin = true; }
        }
        assertTrue("the line left the view", sawPin);
    }

    @Test public void startAtAnAngleOrOffset_steersBackTowardTheLine() {
        final TrackSim.Path path = straight();
        for (double[] start : new double[][]{{0.03, 0}, {-0.03, 0}, {0, 15}, {0, -15}}) {
            final TrackSim s = sim(BLACK_ON_WOOD, path);
            final LineFollowerPipeline p = new LineFollowerPipeline();
            p.configure(TrackSim.appConfig());
            assertTrue(calibrate(p, s, path.pose(10)));
            final double[] pose = path.pose(30);
            final double lat = start[0], yaw = Math.toRadians(start[1]);
            int st = 0;
            for (int k = 0; k < 4; k++) {
                final byte[][] yuv = s.toYuv420(s.render(pose[0] - lat * Math.sin(pose[2]), pose[1] + lat * Math.cos(pose[2]), pose[2] + yaw));
                st = p.processFrameYuv420(yuv[0], yuv[1], yuv[2], W, H)[0];
            }
            // robot left of the line (lat > 0) or turned left (yaw > 0): the line is to the right → steer right
            final double side = lat != 0 ? Math.signum(lat) : Math.signum(yaw);
            assertTrue("start " + start[0] + " m / " + start[1] + "°: steering " + st, side * st >= 3);
        }
    }

    // ── distractors / background ──────────────────────────────────────────

    @Test public void tableEdgeAtTheTop_roiStopsBeforeIt() {
        final TrackSim.Path path = straight();
        final TrackSim s = sim(BLACK_ON_WOOD, path).tableEdge(0.42, 0x3C3C46);
        final Run r = drive(s, path, 10, 10, 30, 2, 10, null);
        assertTrue("ROI top below the image top (background cut off): " + r.last.roiTopFrac, r.last.roiTopFrac > 0.1f);
        for (int i = 0; i < r.steer.length; i++) assertEquals("steering", 0, r.steer[i], 2);
    }

    @Test public void distractorBlobBesideTheLine_ignored() {
        for (int[] pair : new int[][]{BLACK_ON_WOOD, WHITE_ON_DARK}) {
            final int tape = pair[1];
            assertUndisturbed("blob beside the line", pair, straight(), 10, 40, 160, 60,
                    s -> s.patch(0.55, -0.06, 0.06, 0.025, tape), false);
        }
    }

    @Test public void brightStripeBesideTheLine_ignored() {
        assertUndisturbed("bright stripe", BLACK_ON_WOOD, straight(), 10, 40, 160, 60,
                s -> s.stripe(-0.025, 0.2, 0.9, 0.004, 0xEBE1D2), false);
        assertUndisturbed("bright stripe (white line)", WHITE_ON_DARK, straight(), 10, 40, 160, 60,
                s -> s.stripe(-0.025, 0.2, 0.9, 0.003, 0xEBEBEB), false);
    }

    @Test public void robotsOwnShadow_besideAndOverTheLine() {
        for (int[] pair : PAIRS) {
            final String p = String.format("%06X/%06X ", pair[0], pair[1]);
            assertUndisturbed(p + "robot shadow beside", pair, straight(), 10, 40, 160, 60, s -> s.robotShadow(0.12, 0.03, 0.06, 0.03, 0.45), false);
            assertUndisturbed(p + "robot shadow over", pair, straight(), 10, 40, 160, 60, s -> s.robotShadow(0.12, 0.0, 0.06, 0.03, 0.45), false);
        }
    }

    // ── glare (problem 5) ─────────────────────────────────────────────────

    /** The real example: the tape runs straight up the image; glare over ~45–70 % of the ROI, mostly one side. */
    @Test public void glareOnTheLine_straight() {
        for (int[] pair : PAIRS)
            assertUndisturbed(String.format("%06X/%06X glare", pair[0], pair[1]), pair, straight(), 10, 40, 200, 60, s -> s.glare(0.22, 0.008, 0.10, 0.04, 0.95), false);
    }

    @Test public void glareOnTheLine_curve() {
        final TrackSim.Path path = new TrackSim.Path(0, 0, 0).straight(0.6).arc(0.5, 90).straight(0.6);
        assertUndisturbed("glare on a curve", BLACK_ON_WOOD, path, 10, 100, 260, 140, s -> s.glare(0.25, 0.0, 0.07, 0.04, 0.9), false);
    }

    @Test public void glareDuringCalibration() {
        for (int[] pair : PAIRS)
            assertUndisturbed("glare during calibration", pair, straight(), 10, 12, 120, 20, s -> s.glare(0.25, 0.008, 0.08, 0.04, 0.95), true);
    }

    // ── disturbance matrix: colour pairs × disturbance types ──────────────

    @Test public void disturbanceMatrix_straight() {
        for (int[] pair : PAIRS) {
            final String p = String.format("%06X/%06X ", pair[0], pair[1]);
            assertUndisturbed(p + "room shadow", pair, straight(), 10, 40, 200, 60, s -> s.lightSpot(0.6, 0.0, 0.06, 0.15, 0.35), false);
            assertUndisturbed(p + "uneven light", pair, straight(), 10, 40, 200, 60, s -> s.gradient(0.55, 0.05, 0.45, 1.3), false);
            assertUndisturbed(p + "colour patch", pair, straight(), 10, 40, 200, 60, s -> s.patch(0.6, 0.0, 0.06, 0.10, 0x5A6E96), false);
            assertUndisturbed(p + "covering object", pair, straight(), 10, 40, 200, 60, s -> s.cover(0.57, -0.04, 0.60, 0.04, 0x8C8C8C), false);
            assertUndisturbed(p + "colour cast", pair, straight(), 10, 40, 200, 60, s -> s.cast(0.6, 0.0, 0.1, 1.3, 1.0, 0.7), false);
            assertUndisturbed(p + "glare (full)", pair, straight(), 10, 40, 200, 60, s -> s.glare(0.30, 0.0, 0.05, 0.04, 0.97), false);
        }
    }

    @Test public void disturbanceDuringCalibration() {
        for (int[] pair : PAIRS) {
            assertUndisturbed("room shadow during calibration", pair, straight(), 10, 12, 120, 20, s -> s.lightSpot(0.35, 0.0, 0.06, 0.15, 0.35), true);
            assertUndisturbed("covering object during calibration", pair, straight(), 10, 12, 120, 20, s -> s.cover(0.33, -0.04, 0.36, 0.04, 0x8C8C8C), true);
        }
    }

    // ── FrameData / options ───────────────────────────────────────────────

    @Test public void unreliableRegionsAndMemoryAreReported() {
        final TrackSim.Path path = straight();
        final TrackSim s = sim(BLACK_ON_WOOD, path).glare(0.25, 0.008, 0.10, 0.04, 0.95);
        final Run r = drive(s, path, 10, 10, 120, 2, 40, null);
        final LineFollowerPipeline.FrameData d = r.last;
        assertTrue("unreliable cells reported: " + d.unreliableCells, d.unreliableCells > 0);
        int set = 0; for (byte b : d.unreliableGrid) set += b;
        assertEquals("grid count", d.unreliableCells, set);
        assertTrue("part of the path from memory: " + d.pathMemoryPoints, d.pathMemoryPoints + d.pathUnreliablePoints > 0);
        assertTrue("memory fraction in range", d.pathMemoryFrac >= 0f && d.pathMemoryFrac <= 1f);
        boolean fresh = false, notFresh = false;
        for (int i = 0; i < d.pathPointCount; i++) { if (d.pathSource[i] == LineFollowerPipeline.PATH_SRC_FRESH) fresh = true; else notFresh = true; }
        assertTrue("both measured and remembered points", fresh && notFresh);
        assertTrue("confidence stays high (memory + both sides visible): " + d.pathConfidence, d.pathConfidence > 0.5f);
    }

    @Test public void traceOptionsAndV2ModeStillWork() {
        final TrackSim.Path path = straight();
        final Consumer<LineFollowerPipeline.Config>[] variants = new Consumer[]{
                (Consumer<LineFollowerPipeline.Config>) c -> c.pathTraceDirection = LineFollowerPipeline.TRACE_BOTTOM_UP,
                (Consumer<LineFollowerPipeline.Config>) c -> c.pathTraceDirection = LineFollowerPipeline.TRACE_TOP_DOWN,
                (Consumer<LineFollowerPipeline.Config>) c -> c.pathFloorSpace = false,
                (Consumer<LineFollowerPipeline.Config>) c -> c.pathMemory = false,
                (Consumer<LineFollowerPipeline.Config>) c -> c.steerAutoGain = false,
                (Consumer<LineFollowerPipeline.Config>) c -> c.pathTracking = false};
        for (Consumer<LineFollowerPipeline.Config> v : variants) {
            final Run r = drive(sim(BLACK_ON_WOOD, path), path, 10, 10, 100, 3, 40, v);
            for (int i = 0; i < r.steer.length; i++) {
                assertEquals("steering", 0, r.steer[i], 2);
                assertTrue("power", r.power[i] >= 90);
            }
        }
        final Run v2 = drive(sim(BLACK_ON_WOOD, path), path, 10, 10, 40, 3, 20, c -> c.pathTracking = false);
        assertFalse("v2 mode reports pathMode = false", v2.last.pathMode);
    }
}
