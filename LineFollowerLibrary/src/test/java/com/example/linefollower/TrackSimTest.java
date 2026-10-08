package com.example.linefollower;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

import org.junit.Test;

/**
 * Regression tests on rendered camera views (see {@link TrackSim}): realistic
 * floors with shadows / texture, and a robot driving through turns.
 */
public class TrackSimTest {

    private static final int W = 320, H = 240;
    private static final int BROWN = 0x8C5F3C, GREY_BROWN = 0x6E6560, BLACK = 0x1E1C1A,
                             WHITE = 0xEBEBEB, BLUE = 0x1E3CC8, NIGHT = 0x202020;

    private static final Consumer<TrackSim> PLAIN  = s -> { };
    private static final Consumer<TrackSim> SHADOW = s -> { s.shadowEdgeFrac = 0.4; s.shadowGain = 0.45; };
    private static final Consumer<TrackSim> GRAIN_VIGNETTE = s -> { s.grain = 0.12; s.vignette = 0.4; };

    // ── Colour learning ─────────────────────────────────────────────────────

    /** Calibrates on a straight line; returns {centred, line 2 cm right, line 2 cm left} steering. */
    private static int[] calibrateAndSteer(int floor, int line, Consumer<TrackSim> cond) {
        final TrackSim sim = new TrackSim(W, H);
        sim.floorRgb = floor; sim.lineRgb = line;
        cond.accept(sim);
        sim.tracks.add(new TrackSim.Path(0, 0, 0).straight(1.0).points());
        final LineFollowerPipeline p = new LineFollowerPipeline();
        p.configure(TrackSim.appConfig());
        int frames = 0;
        while (!calibrate(p, sim, 0.05, 0, 0)) {
            assertTrue(String.format("calibration failed for %06X/%06X", floor, line), ++frames < 60);
        }
        return new int[]{process(p, sim, 0.05, 0, 0)[0],
                         process(p, sim, 0.05, 0.02, 0)[0],
                         process(p, sim, 0.05, -0.02, 0)[0]};
    }

    private static void assertSteers(int floor, int line, Consumer<TrackSim> cond) {
        final int[] s = calibrateAndSteer(floor, line, cond);
        assertEquals("centred", 0, s[0], 2);
        assertEquals("2 cm right", 15, s[1], 3);
        assertEquals("2 cm left", -15, s[2], 3);
    }

    @Test public void darkLinesWithShadowBesideTheLine() {
        // A shadow over part of the floor used to make every DARK line fail to
        // calibrate: the shadowed floor and the tape fell into one colour cluster.
        assertSteers(BROWN, BLACK, SHADOW);
        assertSteers(GREY_BROWN, BLACK, SHADOW);
        assertSteers(BLUE, BLACK, SHADOW);
    }

    @Test public void previouslyWorkingCombinationsUnchanged() {
        for (Consumer<TrackSim> cond : List.of(PLAIN, GRAIN_VIGNETTE, SHADOW)) {
            assertSteers(BLUE, BLACK, cond);
            assertSteers(BLUE, WHITE, cond);
            assertSteers(NIGHT, WHITE, cond);
            assertSteers(BROWN, WHITE, cond);
            assertSteers(BROWN, BLACK, cond);
        }
    }

    // ── Steering through turns ──────────────────────────────────────────────

    /**
     * Drives straight → arc → straight. The robot cuts the corner by `cut` metres
     * and, with `overshoot`, lags its heading so the line leaves the view. A wide
     * distractor tape can be placed parallel to the exit straight. Returns the
     * steering for every frame from the start of the arc; the robot is exactly
     * on the line for the last frames.
     */
    private static List<int[]> drive(double r, double angle, double cut, boolean overshoot,
                                     double distractor) {
        final TrackSim sim = new TrackSim(W, H);
        sim.floorRgb = BLUE; sim.lineRgb = WHITE;
        final TrackSim.Path path = new TrackSim.Path(0, 0, 0).straight(0.5).arc(r, angle).straight(0.8);
        sim.tracks.add(path.points());
        final int arcStart = 100, arcLen = (int) Math.round(Math.abs(Math.toRadians(angle)) * r / 0.005);
        if (distractor != 0) {
            final double[] e = path.pose(arcStart + arcLen + 20);
            sim.wideTracks.add(new TrackSim.Path(e[0] + distractor * Math.sin(e[2]),
                    e[1] - distractor * Math.cos(e[2]), e[2]).straight(0.6).points());
        }
        final LineFollowerPipeline p = new LineFollowerPipeline();
        p.configure(TrackSim.appConfig());
        final boolean[] found = {false};
        p.setFrameCallback(d -> found[0] = d.trackFound);
        final double[] start = path.pose(10);
        while (!calibrate(p, sim, start[0], start[1], start[2])) { /* calibrating */ }

        final List<int[]> out = new ArrayList<>();   // {steering, trackFound}
        final int bumpStart = arcStart - arcLen / 2, bumpEnd = arcStart + arcLen + arcLen / 2;
        for (int i = arcStart - 40; i < path.size() - 100; i += 4) {
            final double[] pose = path.pose(i);
            double lat = 0, dh = 0;
            if (i > bumpStart && i < bumpEnd) {
                final double t = (i - bumpStart) / (double) (bumpEnd - bumpStart);
                lat = cut * Math.sin(Math.PI * t) * Math.signum(angle);
                if (overshoot) dh = -Math.toRadians(35) * Math.sin(Math.PI * t) * Math.signum(angle);
            }
            final int s = process(p, sim, pose[0] - lat * Math.sin(pose[2]),
                                  pose[1] + lat * Math.cos(pose[2]), pose[2] + dh)[0];
            out.add(new int[]{s, found[0] ? 1 : 0});
        }
        return out;
    }

    private static void assertEndsCentred(List<int[]> run) {
        for (int i = run.size() - 8; i < run.size(); i++) {
            assertEquals("steering back to 0 on the straight (frame " + i + ")", 0, run.get(i)[0], 2);
            assertEquals("line found", 1, run.get(i)[1]);
        }
    }

    @Test public void steeringReturnsToZeroAfterTurns() {
        assertEndsCentred(drive(0.25, 90, 0, false, 0));
        assertEndsCentred(drive(0.12, 90, 0.04, false, 0));
        assertEndsCentred(drive(0.10, 180, 0.03, false, 0));   // U-turn: parallel track in view
        assertEndsCentred(drive(0.12, 90, 0.04, true, 0));     // line lost in the turn
    }

    @Test public void wrongLockAfterTurnIsCorrected() {
        // The line is lost in the turn and a wide tape next to the exit is grabbed.
        // Previously steering then stayed at about -68 for good; now the lock moves
        // back to the real line within trackRelockFrames.
        assertEndsCentred(drive(0.12, 90, 0.04, true, -0.09));
        assertEndsCentred(drive(0.12, 90, 0.04, true, 0.09));
        assertEndsCentred(drive(0.12, 90, 0.0, false, 0.09));
    }

    @Test public void lostLineReadsExactlyFullSteering() {
        boolean sawLost = false;
        for (int[] f : drive(0.12, 90, 0.04, true, 0)) {
            if (f[1] == 0) { assertEquals(100, Math.abs(f[0])); sawLost = true; }
        }
        assertTrue("scenario should lose the line", sawLost);
    }

    // ── Tracking: gradual slowdown before a dead end ────────────────────────

    /** Tracking per 2 cm while driving a 0.8 m straight line up to and past its end. */
    private static List<Integer> trackingToDeadEnd(Consumer<TrackSim> cond) {
        final TrackSim sim = new TrackSim(W, H);
        sim.floorRgb = BLUE; sim.lineRgb = WHITE;
        cond.accept(sim);
        sim.tracks.add(new TrackSim.Path(0, 0, 0).straight(0.8).points());
        final LineFollowerPipeline p = new LineFollowerPipeline();
        p.configure(TrackSim.appConfig());
        while (!calibrate(p, sim, 0.02, 0, 0)) { /* calibrating */ }
        final List<Integer> out = new ArrayList<>();
        for (double x = 0.10; x <= 0.85; x += 0.02) out.add(process(p, sim, x, 0, 0)[1]);
        return out;
    }

    @Test public void trackingRampsDownBeforeDeadEnd() {
        for (Consumer<TrackSim> cond : List.of(PLAIN, GRAIN_VIGNETTE)) {
            final List<Integer> t = trackingToDeadEnd(cond);
            assertEquals("full speed while the road ahead is long", 100, (int) t.get(0));
            assertEquals("minimum at the end", 2, (int) t.get(t.size() - 1));
            int ramp = 0;
            for (int i = 1; i < t.size(); i++) {
                assertTrue("never rises approaching the end: " + t, t.get(i) <= t.get(i - 1) + 2);
                assertTrue("no sudden drop (was 100 → 2 in one frame): " + t, t.get(i - 1) - t.get(i) <= 20);
                if (t.get(i) < 98 && t.get(i) > 2) ramp++;
            }
            assertTrue("gradual ramp over several frames: " + t, ramp >= 6);
        }
    }

    // ── helpers ─────────────────────────────────────────────────────────────

    private static boolean calibrate(LineFollowerPipeline p, TrackSim sim, double x, double y, double h) {
        final byte[][] yuv = sim.toYuv420(sim.render(x, y, h));
        return p.calibrateYuv420(yuv[0], yuv[1], yuv[2], W, H);
    }

    private static int[] process(LineFollowerPipeline p, TrackSim sim, double x, double y, double h) {
        final byte[][] yuv = sim.toYuv420(sim.render(x, y, h));
        return p.processFrameYuv420(yuv[0], yuv[1], yuv[2], W, H).clone();
    }
}
