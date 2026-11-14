package com.example.linefollower;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Locale;

import org.junit.Test;

/**
 * Steering improvements, Phase 1 (STEERING_IMPROVEMENTS.md): the wider steering base without the
 * red zone. Checks that (1) the flags at their original values give exactly the original output,
 * (2) the new strip sits where intended and the look-ahead keeps its far end, and (3) on the app's
 * 720 × 1280 frame the normalised steering stays in the same range as the original on a straight
 * line and on gentle turns (open loop, simulated camera).
 */
public class SteeringBaseTest {

    private static final int W = 720, H = 1280;

    /** The app's config with the ORIGINAL steering base (the library defaults are the new one). */
    private static LineFollowerPipeline.Config baseline() {
        final LineFollowerPipeline.Config c = TrackSim.appConfig();
        c.frameIntervalMs = 33.333f;
        c.useRedZoneExclusion = true;
        c.wideSteeringBase    = false;
        return c;
    }

    private static LineFollowerPipeline.Config wide(boolean normalise) {
        final LineFollowerPipeline.Config c = baseline();
        c.useRedZoneExclusion    = false;
        c.wideSteeringBase       = true;
        c.steeringBaseRows       = 72;
        c.steeringScaleNormalize = normalise;
        return c;
    }

    private static TrackSim sim() {
        final TrackSim s = new TrackSim(W, H);
        s.floorRgb = 0xEBEBEB; s.lineRgb = 0x1E1C1A;   // black tape on a light floor
        s.camLateral = 0.01;
        return s;
    }

    private static byte[][] frame(TrackSim s, double[] q) {
        return s.toYuv420(s.render(q[0], q[1], q[2]));
    }

    /** Calibrated on frame {@code f} (the same frame for every pipeline compared: the sim adds noise). */
    private static LineFollowerPipeline calibrated(LineFollowerPipeline.Config c, byte[][] f) {
        final LineFollowerPipeline p = new LineFollowerPipeline();
        p.configure(c);
        for (int i = 0; i < 60; i++) if (p.calibrateYuv420(f[0], f[1], f[2], W, H)) return p;
        throw new AssertionError("calibration did not finish");
    }

    /** The robot moved {@code lateral} m to its right and turned {@code dHeadDeg} from the path pose. */
    private static double[] offset(double[] q, double lateral, double dHeadDeg) {
        final double ch = Math.cos(q[2]), sh = Math.sin(q[2]);
        return new double[]{q[0] + lateral * sh, q[1] - lateral * ch, q[2] + Math.toRadians(dHeadDeg)};
    }

    /** With the original base the rescaling must do nothing. (Identity with the baseline CODE is
     *  checked separately against the tagged version: see STEERING_IMPROVEMENTS.md, Phase 1.) */
    @Test public void originalFlagsGiveTheOriginalOutput() {
        final TrackSim s = sim();
        final TrackSim.Path path = new TrackSim.Path(0, 0, 0).straight(0.6).arc(0.8, 60);
        s.tracks.add(path.points());
        final LineFollowerPipeline.Config explicit = baseline();
        explicit.steeringScaleNormalize = false;      // has no effect with the original base
        final byte[][] cal = frame(s, path.pose(5));
        final LineFollowerPipeline a = calibrated(baseline(), cal);
        final LineFollowerPipeline b = calibrated(explicit, cal);
        assertEquals(a.getHRoiTop(), b.getHRoiTop());
        assertEquals(a.getHRoiBot(), b.getHRoiBot());
        for (int i = 6; i < path.size(); i += 6) {
            final double[] q = offset(path.pose(i), 0.01 * Math.sin(i * 0.05), 0);
            final byte[][] f = s.toYuv420(s.render(q[0], q[1], q[2]));
            assertArrayEquals("frame " + i, a.processFrameYuv420(f[0], f[1], f[2], W, H),
                                             b.processFrameYuv420(f[0], f[1], f[2], W, H));
        }
    }

    @Test public void newBaseGeometry() {
        final LineFollowerPipeline o = new LineFollowerPipeline(), n = new LineFollowerPipeline();
        final LineFollowerPipeline.Config defaults = TrackSim.appConfig();   // library defaults = original base
        o.configure(defaults); o.init(W, H, defaults);
        n.configure(wide(true)); n.init(W, H, wide(true));
        System.out.printf(Locale.ROOT, "strip rows: original %d–%d, new %d–%d; look-ahead top: original %d, new %d%n",
                o.getHRoiTop(), o.getHRoiBot(), n.getHRoiTop(), n.getHRoiBot(), o.getVRoiTop(), n.getVRoiTop());
        assertEquals("original strip pushed up by the red zone", 960, o.getHRoiBot());
        assertEquals(24, o.getHRoiBot() - o.getHRoiTop());
        assertEquals("new strip: 72 rows centred on 0.85 × 1280", 1052, n.getHRoiTop());
        assertEquals(1124, n.getHRoiBot());
        assertEquals("look-ahead keeps its far end", o.getVRoiTop(), n.getVRoiTop());
    }

    /** Steering after a few frames at a fixed pose (the tracker has state, so let it settle). */
    private static int steerAt(LineFollowerPipeline p, byte[][] f) {
        int st = 0;
        for (int k = 0; k < 20; k++) st = p.processFrameYuv420(f[0], f[1], f[2], W, H)[0];
        return st;
    }

    @Test public void steeringStaysInTheOriginalRange() {
        final double[] lateral = {-0.03, -0.015, 0, 0.015, 0.03};
        final double[] heading = {-6, 0, 6};
        // A straight line and two gentle turns (radius 1.0 m and 0.6 m, camera height 0.15 m).
        final String[] names = {"straight", "turn r=1.0", "turn r=0.6"};
        final int m = heading.length * lateral.length;
        double sumNorm = 0, sumRaw = 0;
        int n = 0;
        double worstGainDev = 0;
        for (int t = 0; t < names.length; t++) {
            final TrackSim s = sim();
            final TrackSim.Path path = new TrackSim.Path(0, 0, 0).straight(0.5);
            if (t == 1) path.arc(1.0, 90); else if (t == 2) path.arc(0.6, 90); else path.straight(1.0);
            s.tracks.add(path.points());
            final int at = t == 0 ? 120 : 130;        // 0.65 m along: on the turn, robot on the arc
            final byte[][] cal = frame(s, path.pose(5));
            final double[] lat = new double[m], dh = new double[m], so = new double[m], sn = new double[m], sr = new double[m];
            final StringBuilder row = new StringBuilder();
            int k = 0;
            for (double h : heading) for (double l : lateral) {
                final byte[][] f = frame(s, offset(path.pose(at), l, h));
                lat[k] = l * 100; dh[k] = h;
                so[k] = steerAt(calibrated(baseline(), cal), f);
                sn[k] = steerAt(calibrated(wide(true), cal), f);
                sr[k] = steerAt(calibrated(wide(false), cal), f);
                row.append(String.format(Locale.ROOT, "  [%+.1fcm %+.0f°] %+.0f/%+.0f/%+.0f", lat[k], h, so[k], sn[k], sr[k]));
                sumNorm += Math.abs(sn[k] - so[k]); sumRaw += Math.abs(sr[k] - so[k]); n++;
                k++;
            }
            // Gains from a least-squares fit  steering = a + b·lateral(cm) + c·angle(°).
            final double[] go = fit(lat, dh, so), gn = fit(lat, dh, sn), gr = fit(lat, dh, sr);
            System.out.println(names[t] + "  steering original/new/new-without-rescaling:" + row);
            System.out.printf(Locale.ROOT, "%s  per cm: original %.2f, new %.2f (×%.2f), without rescaling %.2f (×%.2f)"
                            + "   per degree: original %.2f, new %.2f (×%.2f), without rescaling %.2f (×%.2f)%n",
                    names[t], go[1], gn[1], gn[1] / go[1], gr[1], gr[1] / go[1],
                    go[2], gn[2], gn[2] / go[2], gr[2], gr[2] / go[2]);
            worstGainDev = Math.max(worstGainDev, Math.max(Math.abs(gn[1] / go[1] - 1), Math.abs(gn[2] / go[2] - 1)));
        }
        System.out.printf(Locale.ROOT, "|new − original| per pose: mean %.2f   without rescaling: mean %.2f;"
                + "   largest gain difference new vs original: %.0f %%%n", sumNorm / n, sumRaw / n, 100 * worstGainDev);
        assertTrue("rescaled steering closer to the original than without rescaling", sumNorm <= sumRaw);
        assertTrue("both gains within ±20 % of the original: " + worstGainDev, worstGainDev <= 0.20);
    }

    // ── Row-wise steering base (Config.rowWiseSteeringBase; default off) ─────

    private static LineFollowerPipeline.Config rowWise() {
        final LineFollowerPipeline.Config c = baseline();
        c.rowWiseSteeringBase = true;
        c.rowWiseBaseRows = 48;
        return c;
    }

    /** Clean view: every group finds the line, the base is used, steering as the original (±2). */
    @Test public void rowWiseBaseKeepsTheSteeringOnACleanView() {
        final TrackSim s = sim();
        final TrackSim.Path path = new TrackSim.Path(0, 0, 0).straight(0.5).arc(0.6, 90).straight(0.3);
        s.tracks.add(path.points());
        final byte[][] cal = frame(s, path.pose(5));
        final LineFollowerPipeline o = calibrated(baseline(), cal), r = calibrated(rowWise(), cal);
        final LineFollowerPipeline.FrameData[] last = new LineFollowerPipeline.FrameData[1];
        r.setFrameCallback(d -> last[0] = d);
        {   // first frame after calibration: the line is locked on it; the base needs the lock
            final byte[][] f = frame(s, path.pose(6));
            o.processFrameYuv420(f[0], f[1], f[2], W, H); r.processFrameYuv420(f[0], f[1], f[2], W, H);
            assertTrue("no base before the line is locked", !last[0].rowBaseUsed);
        }
        int used = 0, n = 0, worst = 0;
        for (int i = 10; i < 200; i += 12) {
            final byte[][] f = frame(s, offset(path.pose(i), 0.012 * Math.sin(i * 0.05), 0));
            final int so = o.processFrameYuv420(f[0], f[1], f[2], W, H)[0];
            final int sr = r.processFrameYuv420(f[0], f[1], f[2], W, H)[0];
            n++;
            if (last[0].rowBaseUsed) used++;
            worst = Math.max(worst, Math.abs(sr - so));
            assertEquals("all groups valid on a clean view", last[0].rowBaseGroups, last[0].rowBaseValidGroups);
            assertTrue("no base discontinuity on a clean view", !last[0].rowBaseDiscontinuity);
        }
        System.out.printf(Locale.ROOT, "row-wise base, clean view: used %d/%d frames, worst |steering − original| %d%n", used, n, worst);
        assertEquals("base used on every frame", n, used);
        assertTrue("steering within ±2 of the original: " + worst, worst <= 2);
    }

    /** Glare over the strip rows only: the glared groups are dropped, the clear ones above steer. */
    @Test public void rowWiseBaseDropsGlaredGroups() {
        final TrackSim clean = sim(), glare = sim();
        final TrackSim.Path path = new TrackSim.Path(0, 0, 0).straight(1.5);
        clean.tracks.add(path.points());
        glare.tracks.add(path.points());
        // Saturated patch over rows ~0.735–0.755 (the bottom of the 48-row base: rows 912–960).
        glare.glares.add(new TrackSim.Glare(0.5, 0.745, 0.30, 0.012, 400).halo(0.05));
        final byte[][] cal = frame(clean, path.pose(5));
        final LineFollowerPipeline r = calibrated(rowWise(), cal), o = calibrated(baseline(), cal);
        final LineFollowerPipeline.FrameData[] last = new LineFollowerPipeline.FrameData[1];
        r.setFrameCallback(d -> last[0] = d);
        for (int i = 10; i < 40; i += 6) {                       // lock on the clean view first
            final byte[][] f = frame(clean, path.pose(i));
            r.processFrameYuv420(f[0], f[1], f[2], W, H); o.processFrameYuv420(f[0], f[1], f[2], W, H);
        }
        int dropped = 0, used = 0, n = 0, worst = 0;
        for (int i = 40; i < 120; i += 8) {
            final double[] q = offset(path.pose(i), 0.01, 0);
            final byte[][] fg = frame(glare, q), fc = frame(clean, q);
            final int sr = r.processFrameYuv420(fg[0], fg[1], fg[2], W, H)[0];
            final int so = o.processFrameYuv420(fc[0], fc[1], fc[2], W, H)[0];   // original, clean view
            n++;
            if (last[0].rowBaseValidGroups < last[0].rowBaseGroups) dropped++;
            if (last[0].rowBaseUsed) { used++; worst = Math.max(worst, Math.abs(sr - so)); }
        }
        System.out.printf(Locale.ROOT, "row-wise base, glare over the strip: groups dropped on %d/%d frames, base used on %d, worst |steering − clean original| %d%n",
                dropped, n, used, worst);
        assertTrue("glared groups are dropped", dropped == n);
        assertTrue("the base is still used", used > 0);
        assertTrue("steering from the clear groups within ±3 of the clean view: " + worst, worst <= 3);
    }

    /** Least squares  y = a + b·x1 + c·x2  →  {a, b, c}. */
    private static double[] fit(double[] x1, double[] x2, double[] y) {
        final double[][] A = new double[3][4];
        for (int i = 0; i < y.length; i++) {
            final double[] r = {1, x1[i], x2[i]};
            for (int p = 0; p < 3; p++) { for (int q = 0; q < 3; q++) A[p][q] += r[p] * r[q]; A[p][3] += r[p] * y[i]; }
        }
        for (int p = 0; p < 3; p++) {                        // Gauss-Jordan (3 × 3, well conditioned)
            final double d = A[p][p];
            for (int q = 0; q < 4; q++) A[p][q] /= d;
            for (int o = 0; o < 3; o++) if (o != p) {
                final double f = A[o][p];
                for (int q = 0; q < 4; q++) A[o][q] -= f * A[p][q];
            }
        }
        return new double[]{A[0][3], A[1][3], A[2][3]};
    }
}
