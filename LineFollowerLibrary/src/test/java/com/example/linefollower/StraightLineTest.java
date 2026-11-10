package com.example.linefollower;

import static org.junit.Assert.assertTrue;

import java.util.Locale;

import org.junit.Test;

/**
 * Plan §14: driving straight along a straight line, centred (open loop), the steering should read
 * 0 as with v2, not ±1. The ±1 comes from the corridor filter's heading: ~1° off and wandering with
 * the floor texture under the slices. corridorHeadingDeadbandDeg (1°) removes most of it; the test
 * also checks no sign flips and never more than ±1.
 */
public class StraightLineTest {

    private static final int W = 320, H = 240, N = 150;

    /** {frames with steering != 0, sign flips between consecutive non-zero values, max |steering|}. */
    static int[] drive(int floor, int line, double grain, double vignette, boolean yuv, float deadband, boolean v2) {
        final TrackSim sim = new TrackSim(W, H);
        sim.floorRgb = floor; sim.lineRgb = line; sim.grain = grain; sim.vignette = vignette;
        sim.camLateral = 0.01;                                   // camera 1 cm right of centre
        final TrackSim.Path path = new TrackSim.Path(0, 0, 0).straight(3.0);
        sim.tracks.add(path.points());
        final LineFollowerPipeline.Config c = TrackSim.appConfig();
        c.frameIntervalMs = 16.667f;
        c.corridorHeadingDeadbandDeg = deadband;
        if (v2) { c.corridorMode = false; c.glareMask = false; }
        final LineFollowerPipeline p = new LineFollowerPipeline();
        p.configure(c);
        double[] q = path.pose(5);
        int[] img = sim.render(q[0], q[1], q[2]);
        for (int i = 0; i < 60; i++) {
            final boolean done;
            if (yuv) { final byte[][] f = sim.toYuv420(img); done = p.calibrateYuv420(f[0], f[1], f[2], W, H); }
            else done = p.calibrate(sim.toYuv420(img)[0], W, H);
            if (done) break;
        }
        int nonZero = 0, flips = 0, last = 0, max = 0;
        for (int i = 0; i < N; i++) {
            q = path.pose(Math.min(path.size() - 1, 6 + i));      // ~0.6 m/s at 60 fps
            img = sim.render(q[0], q[1], q[2]);
            final int s;
            if (yuv) { final byte[][] f = sim.toYuv420(img); s = p.processFrameYuv420(f[0], f[1], f[2], W, H)[0]; }
            else s = p.processFrame(sim.toYuv420(img)[0], W, H)[0];
            if (s != 0) nonZero++;
            if (s != 0 && last != 0 && Integer.signum(s) != Integer.signum(last)) flips++;
            if (s != 0) last = s;
            max = Math.max(max, Math.abs(s));
        }
        return new int[]{nonZero, flips, max};
    }

    @Test public void straightLineReadsZero() {
        final int[][] floors = {{0x202020, 0xEBEBEB}, {0x8A6A48, 0x1E1C1A}, {0xEBEBEB, 0x1E1C1A}};
        final String[] names = {"white on dark", "black on wood", "black on white"};
        int worstNonZero = 0, worstFlips = 0, worstMax = 0, sumV3 = 0, sumOff = 0;
        for (int k = 0; k < floors.length; k++) for (int fmt = 0; fmt < 2; fmt++) {
            final double grain = k == 1 ? 0.12 : 0.04, vig = 0.3;
            final int[] v3 = drive(floors[k][0], floors[k][1], grain, vig, fmt == 1, 1.0f, false);
            final int[] v3off = drive(floors[k][0], floors[k][1], grain, vig, fmt == 1, 0f, false);
            final int[] v2 = drive(floors[k][0], floors[k][1], grain, vig, fmt == 1, 1.0f, true);
            System.out.printf(Locale.ROOT, "%-15s %-4s  frames != 0 of %d / sign flips / max |S|:  v3 %d/%d/%d   v3 no deadband %d/%d/%d   v2 %d/%d/%d%n",
                    names[k], fmt == 1 ? "yuv" : "gray", N, v3[0], v3[1], v3[2], v3off[0], v3off[1], v3off[2], v2[0], v2[1], v2[2]);
            sumV3 += v3[0]; sumOff += v3off[0];
            assertTrue("v2 settings read 0 on the straight", v2[0] == 0);
            worstNonZero = Math.max(worstNonZero, v3[0]); worstFlips = Math.max(worstFlips, v3[1]); worstMax = Math.max(worstMax, v3[2]);
        }
        // Measured 2026-11-10: deadband 1° → 1–22 frames of 150 (no deadband 11–45); a ±1 remains
        // where the corridor heading is off by more than the deadband (white on dark, YUV).
        assertTrue("steering != 0 in at most 20 % of the frames: " + worstNonZero, worstNonZero <= N / 5);
        assertTrue("the deadband at least halves the ±1 frames: " + sumV3 + " vs " + sumOff, 2 * sumV3 <= sumOff);
        assertTrue("no sign flips: " + worstFlips, worstFlips == 0);
        assertTrue("never more than ±1: " + worstMax, worstMax <= 1);
    }
}
