package harness;

import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Random;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

/**
 * Output stability under real glare. The robot drives along the line at the phone's
 * frame rate (30 fps, 0.4 m/s), weaving slightly as a real robot does, with camera
 * vibration (pitch: rows shift ±2 px, small yaw / sideways shake) and whole-frame
 * exposure flicker. Each case is run twice with the same motion and noise — without
 * glare and with a realistic glare spot ("glareReal": clipped core, streaky,
 * flickering every frame) — and the frame-to-frame changes of STEER and POWER are
 * compared.
 *
 *   jitS / jitT   mean |ΔSTEER| / |ΔPOWER| between consecutive frames
 *   p95S          95th percentile of |ΔSTEER|
 *   jumpS         frames with |ΔSTEER| ≥ 5
 *   hfS           RMS of STEER minus its centred 7-frame mean (frame-to-frame noise)
 *   biasS         mean |STEER(glare) − STEER(clean)|
 *   minT, lost    lowest POWER, frames reported lost
 *
 * Usage: Jitter <out.csv> [threads] [filter]
 */
public final class Jitter {
    static final String[] SCHEMES = {"black_on_wood", "white_on_dark", "black_on_white", "red_on_green"};
    static final String[] TRACKS = {"straight3", "arcL", "scurve"};
    /** Glare variants: spot ahead of the axle (m), sideways, radii, peak. */
    static final String[][] GLARE = {
        {"near",   "glareReal:fwd=0.27,lat=0.010,rf=0.07,rl=0.05,i=1.6"},
        {"mid",    "glareReal:fwd=0.42,lat=0.010,rf=0.09,rl=0.06,i=1.6"},
        {"far",    "glareReal:fwd=0.60,lat=0.010,rf=0.12,rl=0.07,i=1.6"},
        {"big",    "glareReal:fwd=0.42,lat=0.015,rf=0.18,rl=0.09,i=2.0"},
        {"beside", "glareReal:fwd=0.40,lat=0.050,rf=0.10,rl=0.04,i=1.8"},
        {"streak", "glareReal:fwd=0.45,lat=0.000,rf=0.30,rl=0.025,i=1.8"},
        {"weak",   "glareReal:fwd=0.40,lat=0.010,rf=0.10,rl=0.06,i=0.8"},
    };
    static final double FPS = 30, V = 0.4, STEP = V / FPS;
    static final int WARM = 15;
    static final boolean TRACE = System.getProperty("trace") != null;

    static final class Case {
        final String scheme, track, glare, spec;
        Case(String scheme, String track, String glare, String spec) { this.scheme = scheme; this.track = track; this.glare = glare; this.spec = spec; }
        String name() { return scheme + "," + track + "," + glare; }
    }

    static final Map<String, World> WORLDS = new ConcurrentHashMap<>();
    static World world(String track, String scheme) {
        return WORLDS.computeIfAbsent(track + "|" + scheme, k -> Scenes.build(Scenes.track(track), Scenes.scheme(scheme), false, true));
    }

    static double[] s01(String track) {
        switch (track) {
            case "arcL": return new double[]{0.45, 2.45};
            case "scurve": return new double[]{0.05, 1.70};
            default: return new double[]{0.05, 2.05};
        }
    }

    /** One drive; returns S and T per frame (index 0 = S, 1 = T, 2 = found). */
    static int[][] drive(Case c, String spec, boolean calibWithDisturb) {
        final World w = world(c.track, c.scheme);
        final World.Track t = w.track;
        final double[] se = s01(c.track);
        final Render r = new Render(ClosedLoop.CAM, w, 0, ClosedLoop.ROW_HI, 23, 0.25, 0.15);
        final int i0 = (int) Math.round(se[0] / World.STEP);
        if (spec != null) r.disturb = Disturb.parse(spec, w, t, se[0]);
        final Disturb keep = r.disturb;
        if (!calibWithDisturb) r.disturb = null;
        final Lib lib = new Lib();
        if (lib.calibrateAt(r, t.x[i0], t.y[i0], t.hd[i0], 400) < 0) return null;
        r.disturb = keep;
        final Random rnd = new Random(99);          // same motion / vibration in every run
        final int n = (int) ((se[1] - se[0]) / STEP);
        final int[][] o = new int[3][n];
        int shift = 0;
        for (int k = 0; k < n; k++) {
            final double s = se[0] + k * STEP;
            final int i = (int) Math.round(s / World.STEP);
            // weave: ±6 mm, 2 s period, with the matching heading; vibration on top
            final double tt = k / FPS;
            final double weave = 0.006 * Math.sin(2 * Math.PI * tt / 2.0);
            final double wHead = 0.006 * 2 * Math.PI / 2.0 / V * Math.cos(2 * Math.PI * tt / 2.0);
            final double[] p = Scenes.offset(t, i, weave + 0.001 * rnd.nextGaussian());
            final double th = t.hd[i] + wHead + Math.toRadians(0.3) * rnd.nextGaussian();
            shift = Math.max(-2, Math.min(2, shift + (rnd.nextInt(3) - 1)));
            r.rowShift = shift;
            r.exposure = 1 + 0.03 * rnd.nextGaussian();
            r.render(p[0], p[1], th);
            final int[] out = lib.process(r);
            o[0][k] = out[0]; o[1][k] = out[1]; o[2][k] = lib.trackFound ? 1 : 0;
            if (TRACE) System.out.println(String.format(Locale.ROOT, "%s k=%d ", spec == null ? "clean" : "glare", k) + Viz.line(lib, s, out));
        }
        return o;
    }

    static String stats(int[][] o, int[][] ref) {
        final int n = o[0].length;
        double jS = 0, jT = 0, hf = 0, bias = 0; int jumps = 0, m = 0, lost = 0, minT = 999;
        final double[] dS = new double[n];
        for (int k = WARM; k < n; k++) {
            final double a = Math.abs(o[0][k] - o[0][k - 1]);
            dS[m] = a; jS += a; jT += Math.abs(o[1][k] - o[1][k - 1]);
            if (a >= 5) jumps++;
            int lo = Math.max(WARM, k - 3), hi = Math.min(n - 1, k + 3); double mean = 0;
            for (int q = lo; q <= hi; q++) mean += o[0][q];
            mean /= (hi - lo + 1);
            hf += (o[0][k] - mean) * (o[0][k] - mean);
            if (ref != null) bias += Math.abs(o[0][k] - ref[0][k]);
            if (o[2][k] == 0) lost++;
            minT = Math.min(minT, o[1][k]);
            m++;
        }
        final double[] srt = Arrays.copyOf(dS, m); Arrays.sort(srt);
        return String.format(Locale.ROOT, "%.3f,%.3f,%.1f,%d,%.3f,%.3f,%d,%d", jS / m, jT / m, srt[(int) (0.95 * (m - 1))], jumps,
                Math.sqrt(hf / m), ref == null ? 0 : bias / m, minT, lost);
    }

    static final String HEADER = "lib,scheme,track,glare,calib,jitS,jitT,p95S,jumpS,hfS,biasS,minT,lost,clean_jitS,clean_jitT,clean_p95S,clean_jumpS,clean_hfS,clean_biasS,clean_minT,clean_lost,spec";

    static List<String> run(Case c) {
        final List<String> rows = new ArrayList<>();
        final int[][] clean = drive(c, null, false);
        if (clean == null) { rows.add(String.format("%s,%s,NOCAL", Lib.TAG, c.name())); return rows; }
        final String cs = stats(clean, null);
        for (int cal = 0; cal < 2; cal++) {
            final int[][] g = drive(c, c.spec, cal == 1);
            if (g == null) { rows.add(String.format("%s,%s,%d,NOCAL", Lib.TAG, c.name(), cal)); continue; }
            rows.add(String.format(Locale.ROOT, "%s,%s,%d,%s,%s,%s", Lib.TAG, c.name(), cal, stats(g, clean), cs, c.spec.replace(',', ' ')));
        }
        return rows;
    }

    static List<Case> cases() {
        final List<Case> l = new ArrayList<>();
        for (String sc : SCHEMES) for (String tr : TRACKS) for (String[] g : GLARE) l.add(new Case(sc, tr, g[0], g[1]));
        return l;
    }

    public static void main(String[] a) throws Exception {
        Locale.setDefault(Locale.ROOT);
        final int threads = a.length > 1 ? Integer.parseInt(a[1]) : Math.max(1, Runtime.getRuntime().availableProcessors() - 2);
        final String filter = a.length > 2 ? a[2] : "";
        final List<Case> cs = new ArrayList<>();
        for (Case c : cases()) if (filter.isEmpty() || Matrix.matches(c.name(), filter)) cs.add(c);
        final ExecutorService ex = Executors.newFixedThreadPool(threads);
        final List<Future<List<String>>> fs = new ArrayList<>();
        for (Case c : cs) fs.add(ex.submit(() -> run(c)));
        final long t0 = System.currentTimeMillis();
        try (PrintWriter pw = new PrintWriter(a[0], "UTF-8")) {
            pw.println(HEADER);
            for (Future<List<String>> f : fs) for (String line : f.get()) { pw.println(line); pw.flush(); if (System.getProperty("verbose") != null) System.out.println(line); }
        }
        ex.shutdown();
        System.out.println("wrote " + a[0] + " (" + cs.size() + " cases, " + (System.currentTimeMillis() - t0) / 1000 + " s)");
    }
}
