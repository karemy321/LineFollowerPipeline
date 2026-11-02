package harness;

import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

/**
 * Disturbance matrix: colour pairs × disturbance types × position in the image
 * (near / middle / far) × straight / curve, plus every type present during
 * calibration. The robot drives exactly along the line (scripted, 4 mm per frame);
 * every frame is rendered twice — clean and disturbed — and fed to one library
 * instance each. Reported per case: STEER and POWER of the disturbed run against
 * the clean run over the 60 measured frames.
 *
 * Usage: Matrix <out.csv> [threads] [filter-substring]
 * Run with the library under test first on the classpath (-Dlib.tag=v2|v3).
 */
public final class Matrix {
    static final String[] SCHEMES = {"black_on_wood", "white_on_dark", "black_on_white", "red_on_green"};
    static final String[] TYPES = {"glare_part", "glare_full", "glare_big", "shadow_room", "shadow_robot",
                                   "uneven", "patch", "cover", "stripe", "cast"};
    static final String[] POS = {"near", "mid", "far"};
    static final double STEP = 0.004;          // metres per frame (0.24 m/s at 60 fps)
    static final int MEAS = 60;                // measured frames
    static final boolean TRACE = System.getProperty("trace") != null;

    static double posDist(String p) { return p.equals("near") ? 0.26 : p.equals("mid") ? 0.42 : 0.60; }

    static String patchColour(String scheme) {
        switch (scheme) {
            case "white_on_dark": return "C87828";
            case "black_on_white": return "8CB4DC";
            case "red_on_green": return "DCC832";
            default: return "5A6E96";
        }
    }

    /** Disturbance spec for a type at forward distance D (m ahead of the axle) when the robot is at sMid. */
    static String spec(String type, String scheme, double sMid, double D) {
        final double S = sMid + D;
        switch (type) {
            case "glare_part":  return f("glare:fwd=%.3f,lat=0.012,rf=0.08,rl=0.05,i=0.6", D);
            case "glare_full":  return f("glare:fwd=%.3f,lat=0.0,rf=0.08,rl=0.06,i=0.97", D);
            case "glare_big":   return f("glare:fwd=%.3f,lat=0.015,rf=0.16,rl=0.08,i=0.95", D);
            case "shadow_room": return f("shadowRoom:s=%.3f,lat=0.02,a=0.06,b=0.15,k=0.35", S);
            case "shadow_robot":return f("shadowRobot:fwd=%.3f,lat=0.02,a=0.08,b=0.05,k=0.4", D);
            case "uneven":      return f("uneven:s=%.3f,lo=0.35,hi=1.45,w=0.05", S);
            case "patch":       return f("patch:s=%.3f,lat=0.0,a=0.08,b=0.14,rgb=%s", S, patchColour(scheme));
            case "cover":       return f("cover:s=%.3f,lat=0.0,a=0.03,b=0.05,rgb=8C8C8C", S);
            case "stripe":      return f("stripe:s=%.3f,len=0.4,off=0.032,wid=0.004", S);
            case "cast":        return f("cast:s=%.3f,lat=0.0,r=0.12,du=-20,dv=30", S);
            default: throw new IllegalArgumentException(type);
        }
    }
    static String f(String fmt, Object... a) { return String.format(Locale.ROOT, fmt, a); }

    static final class Case {
        final String scheme, track, type, pos; final boolean calib;
        Case(String scheme, String track, String type, String pos, boolean calib) {
            this.scheme = scheme; this.track = track; this.type = type; this.pos = pos; this.calib = calib;
        }
        String name() { return scheme + "," + track + "," + type + "," + pos + "," + (calib ? 1 : 0); }
    }

    static final Map<String, World> WORLDS = new ConcurrentHashMap<>();
    static World world(String track, String scheme) {
        return WORLDS.computeIfAbsent(track + "|" + scheme, k -> Scenes.build(Scenes.track(track), Scenes.scheme(scheme), false, true));
    }

    /** sMid of the measured window on each track. */
    static double sMid(String track) { return track.equals("arcL") ? 1.25 : 0.6; }

    static String run(Case c) {
        final World w = world(c.track, c.scheme);
        final World.Track t = w.track;
        final double sCal = 0.05;
        final double sm = c.calib ? sCal + STEP * MEAS / 2 : sMid(c.track);
        final double D = c.calib ? posDist("mid") : posDist(c.pos);
        final double sRef = c.calib ? sCal : sm;
        final Render rc = new Render(ClosedLoop.CAM, w, 0, ClosedLoop.ROW_HI, 17, 0.25, 0.15);
        final Render rd = new Render(ClosedLoop.CAM, w, 0, ClosedLoop.ROW_HI, 17, 0.25, 0.15);
        final String sp = spec(c.type, c.scheme, c.calib ? sCal : sm, D);
        rd.disturb = Disturb.parse(sp, w, t, sRef);
        final Lib lc = new Lib(), ld = new Lib();
        // calibrate (disturbed library sees the disturbance only in the 'calib' cases)
        final int ic = (int) Math.round(sCal / World.STEP);
        final Disturb keep = rd.disturb;
        if (!c.calib) rd.disturb = null;
        int okc = lc.calibrateAt(rc, t.x[ic], t.y[ic], t.hd[ic], 400);
        int okd = ld.calibrateAt(rd, t.x[ic], t.y[ic], t.hd[ic], 400);
        rd.disturb = keep;
        if (okc < 0 || okd < 0) return f("%s,%s,NOCAL,%d,%d", Lib.TAG, c.name(), okc, okd);
        final double s0 = c.calib ? sCal : sm - STEP * MEAS / 2;
        double maxDS = 0, sumDS = 0, maxDrop = 0, sumDrop = 0, sumAbsDT = 0, sumSc = 0, sumTc = 0, maxAbsDT = 0;
        int lost = 0, n = 0;
        for (double s = sCal; s < s0 + STEP * MEAS - 1e-9; s += STEP) {
            final int i = (int) Math.round(s / World.STEP);
            final double x = t.x[i] + (s - i * World.STEP) * Math.cos(t.hd[i]), y = t.y[i] + (s - i * World.STEP) * Math.sin(t.hd[i]);
            rc.render(x, y, t.hd[i]); final int[] oc = lc.process(rc).clone();
            rd.render(x, y, t.hd[i]); final int[] od = ld.process(rd).clone();
            if (TRACE && (s >= s0 - 1e-9 || System.getProperty("trace.all") != null)) System.out.printf(Locale.ROOT, "s=%.3f clean S=%d T=%d | dist S=%d T=%d  %s%n", s, oc[0], oc[1], od[0], od[1], ld.last != null && Lib.isV3() ? Viz.line(ld, s, od).substring(18) : "");
            if (System.getProperty("snap") != null && Math.abs(s - Double.parseDouble(System.getProperty("snap"))) < STEP / 2) {
                try { Viz.save(rd, ld.last, "snaps/v3/matrix_snap.png"); Viz.save(rc, lc.last, "snaps/v3/matrix_snap_clean.png"); } catch (Exception e) { throw new RuntimeException(e); }
            }
            if (s < s0 - 1e-9) continue;
            final double dS = Math.abs(od[0] - oc[0]), drop = oc[1] - od[1];
            maxDS = Math.max(maxDS, dS); sumDS += dS; maxDrop = Math.max(maxDrop, drop); sumDrop += Math.max(0, drop);
            sumAbsDT += Math.abs(drop); maxAbsDT = Math.max(maxAbsDT, Math.abs(drop));
            sumSc += oc[0]; sumTc += oc[1];
            if (od[1] <= 5 && oc[1] > 5) lost++;
            n++;
        }
        return f("%s,%s,%d,%.1f,%.2f,%.1f,%.2f,%.2f,%.1f,%d,%.1f,%.1f,%s", Lib.TAG, c.name(), n, maxDS, sumDS / n, maxDrop, sumDrop / n,
                 sumAbsDT / n, maxAbsDT, lost, sumSc / n, sumTc / n, sp.replace(',', ' '));
    }

    static final String HEADER = "lib,scheme,track,type,pos,calib,frames,maxDeltaSteer,meanDeltaSteer,maxPowerDrop,meanPowerDrop,meanAbsDeltaPower,maxAbsDeltaPower,lostFrames,cleanMeanSteer,cleanMeanPower,spec";

    static List<Case> cases() {
        final List<Case> l = new ArrayList<>();
        for (String sc : SCHEMES) for (String tr : new String[]{"straight3", "arcL"}) for (String ty : TYPES) for (String p : POS)
            l.add(new Case(sc, tr, ty, p, false));
        for (String sc : SCHEMES) for (String ty : TYPES) l.add(new Case(sc, "straight3", ty, "mid", true));
        return l;
    }

    public static void main(String[] a) throws Exception {
        Locale.setDefault(Locale.ROOT);
        final int threads = a.length > 1 ? Integer.parseInt(a[1]) : Math.max(1, Runtime.getRuntime().availableProcessors() - 2);
        final String filter = a.length > 2 ? a[2] : "";
        final List<Case> cs = new ArrayList<>();
        for (Case c : cases()) if (filter.isEmpty() || matches(c.name(), filter)) cs.add(c);
        final ExecutorService ex = Executors.newFixedThreadPool(threads);
        final List<Future<String>> fs = new ArrayList<>();
        for (Case c : cs) fs.add(ex.submit(() -> run(c)));
        final long t0 = System.currentTimeMillis();
        try (PrintWriter pw = new PrintWriter(a[0], "UTF-8")) {
            pw.println(HEADER);
            int done = 0;
            for (Future<String> f : fs) {
                final String line = f.get();
                pw.println(line); pw.flush();
                if (System.getProperty("verbose") != null) System.out.println(line);
                if (++done % 40 == 0) System.out.println("  " + done + "/" + cs.size() + "  " + (System.currentTimeMillis() - t0) / 1000 + " s");
            }
        }
        ex.shutdown();
        System.out.println("wrote " + a[0] + " (" + cs.size() + " cases, " + (System.currentTimeMillis() - t0) / 1000 + " s)");
    }

    /** filter: comma-separated substrings, all must occur. */
    static boolean matches(String name, String filter) {
        for (String f : filter.split("\\+")) if (!name.contains(f)) return false;
        return true;
    }
}
