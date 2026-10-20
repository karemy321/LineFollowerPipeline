package harness;

import com.example.linefollower.LineFollowerPipeline;

import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Regression tests of existing behaviour, run identically on the ORIGINAL and NEW
 * library (720×960, app configuration, perspective camera, uneven lighting).
 *
 * Each check prints PASS/FAIL with the measured values. Usage: Regression <out.csv>
 */
public final class Regression {
    static final Cam CAM = ClosedLoop.CAM;
    static final int W = ClosedLoop.W, H = ClosedLoop.H;
    static final double S0 = 0.6;
    static final List<String> rows = new ArrayList<>();
    static int fails = 0;
    static int minS = 999, maxS = -999, minT = 999, maxT = -999;

    static void check(String test, boolean ok, String values) {
        if (!ok) fails++;
        final String line = String.format(Locale.ROOT, "%s,%s,%s,%s", Lib.TAG, test, ok ? "PASS" : "FAIL", values.replace(',', ';'));
        rows.add(line);
        System.out.println(line);
    }

    static World straight(String scheme) { return Scenes.build(Scenes.track("straight3"), Scenes.scheme(scheme), false, true); }

    static World emptyFloor(String scheme) {
        final World.Track t = Scenes.track("straight3");
        final Scenes.Scheme sc = Scenes.scheme(scheme);
        final World w = new World(-1, -1, 4, 1, 0.002);
        w.fill(sc.floor);
        w.outY = World.Col.y(sc.outside); w.outU = World.Col.u(sc.outside); w.outV = World.Col.v(sc.outside);
        w.track = t;
        w.light((x, y) -> (int) Math.round(1024 * (0.72 + 0.45 * World.noise(x, y, 0.6, 99))));
        return w;
    }

    enum Fmt { YUV, GRAY, ARGB }

    static boolean cal(Lib l, Render r, Fmt f) {
        switch (f) {
            case GRAY: return l.p.calibrate(r.y, W, H);
            case ARGB: return l.p.calibrateArgb(r.toArgb(), W, H);
            default:   return l.calibrate(r);
        }
    }
    static int[] proc(Lib l, Render r, Fmt f) {
        final int[] o;
        switch (f) {
            case GRAY: o = l.p.processFrame(r.y, W, H); break;
            case ARGB: o = l.p.processFrameArgb(r.toArgb(), W, H); break;
            default:   o = l.process(r);
        }
        minS = Math.min(minS, o[0]); maxS = Math.max(maxS, o[0]); minT = Math.min(minT, o[1]); maxT = Math.max(maxT, o[1]);
        return o.clone();
    }

    /** Calibrate centred, then measure steering centred / 2 cm right / 2 cm left (robot shifted), tracking. */
    static void follows(String name, World w, Fmt f, Lib l, boolean expectCal) {
        final Render r = new Render(CAM, w, f == Fmt.ARGB ? 0 : ClosedLoop.ROW_LO, f == Fmt.ARGB ? H : ClosedLoop.ROW_HI, 11, 0.25, 0.15);
        int frames = 0; boolean ok = false;
        while (frames < 200) { r.render(S0, 0, 0); frames++; if (cal(l, r, f)) { ok = true; break; } }
        if (!expectCal) {
            for (int i = 0; i < 100 && !ok; i++) { r.render(S0, 0, 0); if (cal(l, r, f)) ok = true; }
            check(name + " must NOT calibrate", !ok, "calibrated=" + ok + " after 300 frames; calibIssue=" + l.calibIssue);
            return;
        }
        if (!ok) { check(name, false, "did not calibrate in 200 frames; calibIssue=" + l.calibIssue); return; }
        final double cS = avg(l, r, f, 0, 0), cT = lastT;
        final double rS = avg(l, r, f, -0.02, 0);  // robot 2 cm RIGHT of the line → line appears LEFT → negative
        final double c2 = avg(l, r, f, 0, 0);
        final double lS = avg(l, r, f, 0.02, 0);   // robot 2 cm LEFT → positive
        final boolean pass = Math.abs(cS) <= 2 && cT >= 90 && rS < -5 && lS > 5 && Math.abs(c2) <= 2;
        check(name, pass, String.format(Locale.ROOT, "calFrames=%d colorModel=%b S(centre)=%.1f T=%.0f S(robot 2cm right)=%.1f S(2cm left)=%.1f",
                frames, l.p.isColorModelReady(), cS, cT, rS, lS));
    }

    static double lastT;
    /** Move to the pose over 10 frames (so the lock follows), then average 20 frames. */
    static double avg(Lib l, Render r, Fmt f, double latLeft, double yawDeg) {
        for (int k = 1; k <= 10; k++) { r.render(S0, latLeft * k / 10.0, Math.toRadians(yawDeg) * k / 10.0); proc(l, r, f); }
        double s = 0, t = 0;
        for (int k = 0; k < 20; k++) { r.render(S0, latLeft, Math.toRadians(yawDeg)); final int[] o = proc(l, r, f); s += o[0]; t += o[1]; }
        lastT = t / 20;
        return s / 20;
    }

    public static void main(String[] a) throws Exception {
        Locale.setDefault(Locale.ROOT);

        // ── 1. autoColor on many colour pairs (YUV = app input) ──
        for (String sc : new String[]{"white_on_dark", "white_on_brown", "black_on_wood", "black_on_white", "blue_on_yellow", "red_on_green"})
            follows("autoColor " + sc + " (YUV)", straight(sc), Fmt.YUV, new Lib(), true);
        // other input formats
        for (String sc : new String[]{"white_on_dark", "black_on_wood"}) {
            follows("autoColor " + sc + " (GRAY)", straight(sc), Fmt.GRAY, new Lib(), true);
            follows("autoColor " + sc + " (ARGB)", straight(sc), Fmt.ARGB, new Lib(), true);
        }
        // equal brightness, grayscale: cannot be separated (existing, documented behaviour)
        follows("autoColor red_on_green (GRAY, equal brightness)", straight("red_on_green"), Fmt.GRAY, new Lib(), false);

        // ── 2. manual mode autoColor = false ──
        { final Lib l = new Lib(); l.cfg.autoColor = false; l.cfg.trackIsBright = true; l.reconfigure();
          follows("manual trackIsBright=true white_on_dark", straight("white_on_dark"), Fmt.YUV, l, true); }
        { final Lib l = new Lib(); l.cfg.autoColor = false; l.cfg.trackIsBright = false; l.reconfigure();
          follows("manual trackIsBright=false black_on_white", straight("black_on_white"), Fmt.YUV, l, true); }
        { final Lib l = new Lib(); l.cfg.autoColor = false; l.cfg.trackIsBright = false; l.reconfigure();
          follows("manual trackIsBright=false black_on_wood", straight("black_on_wood"), Fmt.YUV, l, true); }

        // ── 3. recalibration: bright track → dark track → bright track ──
        {
            final Lib l = new Lib();
            follows("recal step 1 white_on_dark", straight("white_on_dark"), Fmt.YUV, l, true);
            l.p.requestRecalibration();
            follows("recal step 2 black_on_wood", straight("black_on_wood"), Fmt.YUV, l, true);
            l.p.requestRecalibration();
            follows("recal step 3 white_on_dark again", straight("white_on_dark"), Fmt.YUV, l, true);
        }

        // ── 4. empty floor without tape must not calibrate ──
        for (String sc : new String[]{"black_on_wood", "white_on_dark", "black_on_white"})
            follows("empty floor (" + Scenes.scheme(sc).name + " floor, no tape)", emptyFloor(sc), Fmt.YUV, new Lib(), false);

        // ── 5. straight line, no distractors: steering for scripted poses (compare across libs) ──
        {
            final World w = straight("black_on_wood");
            final Lib l = new Lib();
            final Render r = new Render(CAM, w, ClosedLoop.ROW_LO, ClosedLoop.ROW_HI, 13, 0.25, 0.15);
            l.calibrateAt(r, S0, 0, 0, 300);
            final StringBuilder sb = new StringBuilder();
            for (double lat : new double[]{0, -0.01, -0.03, 0.01, 0.03, 0})
                for (double yaw : new double[]{0, 5, -5}) {
                    final double s = avg(l, r, Fmt.YUV, lat, yaw);
                    sb.append(String.format(Locale.ROOT, "lat%+.0fcm/yaw%+.0f:%.1f ", lat * 100, yaw, s));
                }
            check("straight scripted poses (steering)", true, sb.toString().trim());
        }

        // ── 6. output ranges over every frame above ──
        check("steering within [-50,50] and tracking within [5,100] (all frames above)",
              minS >= -50 && maxS <= 50 && minT >= 5 && maxT <= 100, "S in [" + minS + "," + maxS + "] T in [" + minT + "," + maxT + "]");

        try (PrintWriter pw = new PrintWriter(a[0], "UTF-8")) {
            pw.println("lib,test,result,values");
            for (String s : rows) pw.println(s);
        }
        System.out.println(fails == 0 ? "ALL PASS" : fails + " FAIL(S)");
    }
}
