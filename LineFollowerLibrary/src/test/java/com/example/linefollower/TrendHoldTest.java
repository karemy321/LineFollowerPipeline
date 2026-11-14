package com.example.linefollower;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;

import org.junit.Test;

/**
 * Steering improvements, Phase 2 (STEERING_IMPROVEMENTS.md): the trend hold. Every check runs a
 * pipeline with the hold ON next to one with it OFF on the SAME frames: the outputs must be
 * identical on every frame that is not held, and the held frames must carry the trend.
 */
public class TrendHoldTest {

    private static final int W = 320, H = 240;
    private static final float DT = 1000f / 30f;    // 30 fps

    /** Per frame: {steering, tracking, trendState (hold-on pipeline), trackFound (off)}. */
    private static final class Run {
        final List<int[]> on = new ArrayList<>(), off = new ArrayList<>();
        final List<Integer> state = new ArrayList<>();
        final List<Boolean> found = new ArrayList<>();
    }

    private static LineFollowerPipeline.Config cfg(boolean hold, boolean glareMask) {
        final LineFollowerPipeline.Config c = TrackSim.appConfig();
        c.frameIntervalMs = DT;
        c.useTrendHold = hold;
        if (!glareMask) { c.glareMask = false; c.corridorMode = false; }
        return c;
    }

    /** Drive the poses through both pipelines (calibrated on pose 0's frame). */
    private static Run drive(TrackSim sim, List<double[]> poses, LineFollowerPipeline.Config on, LineFollowerPipeline.Config off) {
        final LineFollowerPipeline a = new LineFollowerPipeline(), b = new LineFollowerPipeline();
        a.configure(on); b.configure(off);
        final int[] st = new int[1];
        final boolean[] fd = new boolean[1];
        a.setFrameCallback(d -> st[0] = d.trendState);
        b.setFrameCallback(d -> fd[0] = d.trackFound);
        final double[] c0 = poses.get(0);
        final byte[] cal = sim.toYuv420(sim.render(c0[0], c0[1], c0[2]))[0];
        for (int i = 0; i < 60; i++) if (a.calibrate(cal, W, H) & b.calibrate(cal, W, H)) break;
        final Run r = new Run();
        for (int k = 1; k < poses.size(); k++) {
            final double[] q = poses.get(k);
            final byte[] f = sim.toYuv420(sim.render(q[0], q[1], q[2]))[0];
            r.on.add(a.processFrame(f, W, H).clone());
            r.off.add(b.processFrame(f, W, H).clone());
            r.state.add(st[0]);
            r.found.add(fd[0]);
        }
        return r;
    }

    private static List<double[]> along(TrackSim.Path p, int from, int to, int step) {
        final List<double[]> out = new ArrayList<>();
        out.add(p.pose(5));
        for (int i = from; i < Math.min(to, p.size()); i += step) out.add(p.pose(i));
        return out;
    }

    /** Track with a gap [g0, g1) (path sample indices, 5 mm each). */
    private static TrackSim gapSim(TrackSim.Path p, int g0, int g1) {
        final TrackSim s = new TrackSim(W, H);
        s.floorRgb = 0xEBEBEB; s.lineRgb = 0x1E1C1A; s.camLateral = 0.01;
        final double[][] pts = p.points();
        s.tracks.add(new double[][]{Arrays.copyOfRange(pts[0], 0, g0), Arrays.copyOfRange(pts[1], 0, g0)});
        if (g1 < pts[0].length)
            s.tracks.add(new double[][]{Arrays.copyOfRange(pts[0], g1, pts[0].length), Arrays.copyOfRange(pts[1], g1, pts[1].length)});
        return s;
    }

    private static int count(Run r, int state) {
        int n = 0;
        for (int s : r.state) if (s == state) n++;
        return n;
    }

    /** Not held ⇒ identical output. */
    private static void assertUnchangedWhenNotHeld(Run r) {
        for (int k = 0; k < r.on.size(); k++)
            if (r.state.get(k) != LineFollowerPipeline.TREND_HOLDING)
                assertArrayEquals("frame " + k + " not held but changed", r.off.get(k), r.on.get(k));
    }

    @Test public void cleanDrivesAreNeverHeldAndUnchanged() {
        for (int path = 0; path < 3; path++) for (int g = 0; g < 2; g++) {
            final TrackSim.Path p = new TrackSim.Path(0, 0, 0).straight(0.5);
            if (path == 0) p.straight(1.0); else if (path == 1) p.arc(0.6, 90).straight(0.5); else p.arc(0.8, 60).arc(-0.8, 120).straight(0.3);
            final TrackSim s = new TrackSim(W, H);
            s.floorRgb = 0xEBEBEB; s.lineRgb = 0x1E1C1A; s.camLateral = 0.01;
            s.tracks.add(p.points());
            final Run r = drive(s, along(p, 6, p.size() - 60, 4), cfg(true, g == 0), cfg(false, g == 0));
            assertEquals("path " + path + " glareMask " + (g == 0) + ": nothing held", 0, count(r, LineFollowerPipeline.TREND_HOLDING));
            for (int k = 0; k < r.on.size(); k++) assertArrayEquals("frame " + k, r.off.get(k), r.on.get(k));
        }
    }

    @Test public void gapMidImageIsHeldWithTheTrend() {
        // glareMask off: an 8 cm gap in the tape at 0.7 m; 4 cm per frame at 30 fps ≈ 1.2 m/s.
        final TrackSim.Path p = new TrackSim.Path(0, 0, 0).straight(0.5).arc(0.6, 90).straight(0.5);
        final TrackSim s = gapSim(p, 140, 156);
        final LineFollowerPipeline.Config on = cfg(true, false);
        on.holdPowerScale = 0.5f;
        final Run r = drive(s, along(p, 60, 220, 8), on, cfg(false, false));
        final int held = count(r, LineFollowerPipeline.TREND_HOLDING);
        System.out.println("gap, glareMask off: held frames " + held);
        assertTrue("the gap is held", held > 0);
        assertUnchangedWhenNotHeld(r);
        // A held frame carries the average of the last valid outputs (at most trendWindowFrames).
        for (int k = 0; k < r.on.size(); k++) {
            if (r.state.get(k) != LineFollowerPipeline.TREND_HOLDING) continue;
            int j = k - 1;
            while (r.state.get(j) == LineFollowerPipeline.TREND_HOLDING) j--;
            float sS = 0f, sP = 0f;
            int n = 0;
            for (int i = j; i >= 0 && n < on.trendWindowFrames && r.state.get(i) == LineFollowerPipeline.TREND_TRACKING; i--, n++) {
                sS += r.off.get(i)[0]; sP += r.off.get(i)[1];
            }
            assertEquals("held steering = trend", Math.round(sS / n), r.on.get(k)[0]);
            final float lastValid = r.off.get(j)[1];
            assertEquals("held power = min(trend, last valid) × holdPowerScale",
                    Math.max(on.trackingMinValue, Math.round(Math.min(sP / n, lastValid) * 0.5f)), r.on.get(k)[1]);
            assertTrue("never faster than the last valid frame", r.on.get(k)[1] <= Math.max(on.trackingMinValue, lastValid));
        }
    }

    @Test public void heldPowerNeverAboveTheLastValidPower() {
        // The look-ahead sees the gap as the end of the road, so the power ramps down to the
        // minimum before the line disappears (39 → 26 → 12 → 5 here). The average of those is
        // 21; the hold must keep 5 — the robot never speeds up when the line disappears.
        final TrackSim.Path p = new TrackSim.Path(0, 0, 0).straight(0.5).arc(0.6, 90).straight(0.5);
        final List<double[]> poses = new ArrayList<>();
        poses.add(p.pose(5));
        for (int i = 60; i < 140; i += 6) poses.add(p.pose(6 + i));
        final LineFollowerPipeline.Config on = cfg(true, false), off = cfg(false, false);
        on.frameIntervalMs = 16.667f; off.frameIntervalMs = 16.667f;
        final Run r = drive(gapSim(p, 140, 156), poses, on, off);
        int held = 0;
        final StringBuilder sb = new StringBuilder();
        for (int k = 0; k < r.on.size(); k++) {
            sb.append(String.format(Locale.ROOT, "[%d/%d%s] ", r.off.get(k)[1], r.on.get(k)[1],
                    r.state.get(k) == LineFollowerPipeline.TREND_HOLDING ? " H" : ""));
            if (r.state.get(k) != LineFollowerPipeline.TREND_HOLDING) continue;
            held++;
            int j = k - 1;
            while (r.state.get(j) == LineFollowerPipeline.TREND_HOLDING) j--;
            assertEquals("held power = last valid power (it was ramping down)", r.off.get(j)[1], r.on.get(k)[1]);
        }
        System.out.println("ramp-down before a gap, power off/on: " + sb);
        assertTrue("the gap is held", held > 0);
        assertUnchangedWhenNotHeld(r);
        // The ramp itself: the last valid power before the hold is the minimum (5), and so is the hold.
        for (int k = 0; k < r.on.size(); k++)
            if (r.state.get(k) == LineFollowerPipeline.TREND_HOLDING) assertEquals("holds 5, not 21", 5, r.on.get(k)[1]);
    }

    @Test public void longLossFallsBackAfterMaxHoldMs() {
        // The tape ends at 0.7 m (glareMask off): held for at most maxHoldMs, then the original output.
        final TrackSim.Path p = new TrackSim.Path(0, 0, 0).straight(2.0);
        final TrackSim s = gapSim(p, 140, Integer.MAX_VALUE);
        final LineFollowerPipeline.Config on = cfg(true, false);
        final Run r = drive(s, along(p, 60, 300, 8), on, cfg(false, false));
        final int held = count(r, LineFollowerPipeline.TREND_HOLDING);
        System.out.println("tape end, glareMask off: held frames " + held + " (limit " + on.maxHoldMs + " ms at " + DT + " ms)");
        assertTrue("held", held > 0);
        assertTrue("at most maxHoldMs: " + held, (held - 1) * DT <= on.maxHoldMs + 1e-3f);
        assertUnchangedWhenNotHeld(r);
        final int last = r.on.size() - 1;
        assertEquals("ends LOST with the original output", LineFollowerPipeline.TREND_LOST, (int) r.state.get(last));
    }

    @Test public void glareMaskOnLossIsLeftToTheCoast() {
        // Default settings (glareMask on): the same gap is the glare coast's case; nothing held.
        final TrackSim.Path p = new TrackSim.Path(0, 0, 0).straight(0.5).arc(0.6, 90).straight(0.5);
        final Run r = drive(gapSim(p, 140, 156), along(p, 60, 220, 8), cfg(true, true), cfg(false, true));
        assertEquals("nothing held", 0, count(r, LineFollowerPipeline.TREND_HOLDING));
        for (int k = 0; k < r.on.size(); k++) assertArrayEquals("frame " + k, r.off.get(k), r.on.get(k));
    }

    @Test public void edgeExitIsNeverHeld() {
        // The tape turns away sharply while the robot drives straight on: the line leaves at
        // the frame edge. Every frame the original pins at ±50 must be unchanged and not held.
        final TrackSim.Path track = new TrackSim.Path(0, 0, 0).straight(0.5).arc(0.12, 90).straight(0.6);
        final TrackSim.Path robot = new TrackSim.Path(0, 0, 0).straight(1.2);
        for (int g = 0; g < 2; g++) {
            final TrackSim s = new TrackSim(W, H);
            s.floorRgb = 0xEBEBEB; s.lineRgb = 0x1E1C1A; s.camLateral = 0.01;
            s.tracks.add(track.points());
            final Run r = drive(s, along(robot, 40, 200, 6), cfg(true, g == 0), cfg(false, g == 0));
            int pins = 0;
            for (int k = 0; k < r.on.size(); k++) {
                if (Math.abs(r.off.get(k)[0]) == 50 && !r.found.get(k)) {
                    pins++;
                    assertTrue("edge pin " + k + " held", r.state.get(k) != LineFollowerPipeline.TREND_HOLDING);
                    assertArrayEquals("edge pin " + k + " unchanged", r.off.get(k), r.on.get(k));
                }
            }
            System.out.println("edge exit, glareMask " + (g == 0) + ": pin frames " + pins);
            assertTrue("the scene leaves at the edge", pins > 0);
            assertUnchangedWhenNotHeld(r);
        }
    }

    /** Flat synthetic frame: a vertical dark stripe 30 px wide centred at {@code cx} on a light floor. */
    private static int[] stripe(int cx) {
        final int[] px = new int[W * H];
        final java.util.Random rnd = new java.util.Random(cx);
        for (int i = 0; i < px.length; i++) {
            final int col = i % W;
            final int v = (Math.abs(col - cx) <= 15 ? 20 : 235) + (int) Math.round(rnd.nextGaussian() * 3);
            final int c = Math.max(0, Math.min(255, v));
            px[i] = 0xFF000000 | (c << 16) | (c << 8) | c;
        }
        return px;
    }

    @Test public void jumpIsHeldThenAccepted() {
        // On simulated camera images the pipeline's own lock spreads a sideways step over several
        // frames (no jump to hold). Flat frames make a real one-frame jump: the line steps 80 px
        // and stays there. Held for at most maxJumpHoldMs, then accepted (original output again).
        final LineFollowerPipeline.Config on = new LineFollowerPipeline.Config(), off = new LineFollowerPipeline.Config();
        on.frameIntervalMs = DT; off.frameIntervalMs = DT; off.useTrendHold = false;
        final LineFollowerPipeline a = new LineFollowerPipeline(), b = new LineFollowerPipeline();
        a.configure(on); b.configure(off);
        final int[] st = new int[1];
        a.setFrameCallback(d -> st[0] = d.trendState);
        final int[] centred = stripe(W / 2), moved = stripe(W / 2 - 80);
        for (int i = 0; i < 60; i++) if (a.calibrateArgb(centred, W, H) & b.calibrateArgb(centred, W, H)) break;
        final Run r = new Run();
        for (int k = 0; k < 12; k++) {
            final int[] f = k < 4 ? centred : moved;
            r.on.add(a.processFrameArgb(f, W, H).clone());
            r.off.add(b.processFrameArgb(f, W, H).clone());
            r.state.add(st[0]);
        }
        final int held = count(r, LineFollowerPipeline.TREND_HOLDING);
        final StringBuilder sb = new StringBuilder();
        for (int k = 0; k < 12; k++) sb.append(String.format(Locale.ROOT, "[%d %+d/%+d%s] ", k, r.off.get(k)[0], r.on.get(k)[0],
                r.state.get(k) == LineFollowerPipeline.TREND_HOLDING ? " H" : ""));
        System.out.println("80 px step, steering off/on: " + sb);
        assertTrue("the step is a jump: " + r.off.get(4)[0], Math.abs(r.off.get(4)[0] - r.off.get(3)[0]) > on.maxSteeringJump);
        assertEquals("the jump frame is held", LineFollowerPipeline.TREND_HOLDING, (int) r.state.get(4));
        assertEquals("held at the trend", r.off.get(3)[0], r.on.get(4)[0]);
        assertTrue("at most maxJumpHoldMs: " + held, (held - 1) * DT <= on.maxJumpHoldMs + 1e-3f);
        assertUnchangedWhenNotHeld(r);
        assertArrayEquals("accepted afterwards", r.off.get(11), r.on.get(11));
    }
}
