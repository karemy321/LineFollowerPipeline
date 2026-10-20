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
 * Closed-loop simulator: differential-drive robot + tilted phone camera rendering
 * the floor at 60 Hz, the library under test, and the lecturer's Blockly control law:
 *
 *   motor steering = (0 − Steering) × c        ("Invert Motors" = true → turns TOWARD the line)
 *   motor power    = round(x × (Power / 8) + 5)
 *
 * "move steering" is modelled like the LEGO block: the outer wheel runs at the given
 * power, the inner wheel at power × (1 − |steer|/50) (stops at 50, reverses beyond).
 * Wheel speed = power/100 × vmax, first-order motor lag tau, and the command computed
 * from frame k is applied {@code latency} frames later.
 *
 * Usage: ClosedLoop <out.csv> [matrix=main|speed|quick|gains] [threads=N]
 */
public final class ClosedLoop {

    // Camera / robot geometry (see CHANGES_v2.md, "simulator")
    static final int W = 960, H = 1280;   // app frames: 1280×960 (4:3) rotated to portrait
    // Camera: 60° vFOV, tilt 45°, 20 cm high, 8 cm ahead of the axle, 1 cm right of centre (overridable).
    static final Cam CAM = new Cam(W, H, 60, Double.parseDouble(System.getProperty("sim.tilt", "45")),
            Double.parseDouble(System.getProperty("sim.camH", "0.20")), Double.parseDouble(System.getProperty("sim.camFwd", "0.08")), 0.01);
    static final int ROW_LO = 280, ROW_HI = 970, H_ROW = 948;   // library reads rows 296…960
    static final double DT = 1.0 / 60, WHEELBASE = 0.13, TAU = 0.08;
    // Real-robot effects (system properties): command latency (s), Blockly loop rate (Hz),
    // motor dead zone (wheel power below which the wheel does not turn).
    static final double SIM_LAT  = Double.parseDouble(System.getProperty("sim.lat", "0.0333"));
    static final double SIM_HZ   = Double.parseDouble(System.getProperty("sim.hz", "60"));
    static final double SIM_DEAD = Double.parseDouble(System.getProperty("sim.dead", "0"));
    static final String SIM_TAG  = String.format(Locale.ROOT, "lat%.0fms/%.0fHz/dead%.0f/fwd%s/tilt%s/yaw%s/lat0%s", SIM_LAT * 1000, SIM_HZ, SIM_DEAD,
            System.getProperty("sim.camFwd", "0.08"), System.getProperty("sim.tilt", "45"), System.getProperty("sim.yaw0", "0"), System.getProperty("sim.lat0", "0"));
    static final int LATENCY = (int) Math.round(SIM_LAT * 60);
    static final int CTRL_EVERY = Math.max(1, (int) Math.round(60 / SIM_HZ));
    static final double OFF_TRACK = 0.12;
    static final double YAW0 = Double.parseDouble(System.getProperty("sim.yaw0", "0"));
    static final double LAT0 = Double.parseDouble(System.getProperty("sim.lat0", "0"));
    static final boolean TRACE = System.getProperty("trace") != null;
    static final int TRACE_EVERY = Integer.getInteger("trace.every", 6);

    static final class Spec {
        final String track, scheme; final boolean distract; final double c, x, vmax; final String extra;
        Spec(String track, String scheme, boolean distract, double c, double x, double vmax, String extra) {
            this.track = track; this.scheme = scheme; this.distract = distract; this.c = c; this.x = x; this.vmax = vmax; this.extra = extra;
        }
        String key() { return track + "|" + scheme + "|" + (distract ? 1 : 0); }
    }

    static final Map<String, World> WORLDS = new ConcurrentHashMap<>();
    static World world(Spec s) {
        return WORLDS.computeIfAbsent(s.key(), k -> Scenes.build(Scenes.track(s.track), Scenes.scheme(s.scheme), s.distract, true));
    }

    static String run(Spec s, Cam cam) {
        final World w = world(s);
        final World.Track t = w.track;
        final Render r = new Render(cam, w, ROW_LO, ROW_HI, (s.key() + s.c + s.x).hashCode(), 0.25, 0.15);
        final Lib lib = new Lib();
        if (s.extra != null && !s.extra.isEmpty())
            for (String kv : s.extra.split(";")) lib.set(kv.split("=")[0], kv.split("=")[1]);

        double px = t.x[0], py = t.y[0], th = t.hd[0];
        final int calib = lib.calibrateAt(r, px, py, th, 400);
        if (calib < 0) return row(s, "NOCAL", 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        // Optional start disturbance after calibration: robot turned YAW0 degrees (+ = left)
        // about its axle and shifted LAT0 metres to the left.
        th += Math.toRadians(YAW0);
        px -= LAT0 * Math.sin(t.hd[0]); py += LAT0 * Math.cos(t.hd[0]);
        double settleT = 0;

        double vl = 0, vr = 0, cmdSt = 0, cmdPw = 0;
        final double[][] queue = new double[LATENCY + 1][2];
        int qi = 0;
        int idx = 0; double progress = 0;
        final int n = t.n();
        final double v5 = s.vmax * 0.05;
        final double tMax = 2.5 * t.length / v5 + 5;
        int frames = 0, signChanges = 0, lastSign = 0, lostFrames = 0, lostEvents = 0, wrongLock = 0;
        boolean prevFound = true;
        double sumLat = 0, maxLat = 0, sumHead2 = 0, maxHead = 0, sumS = 0, sumS2 = 0, sumAbsS = 0;
        int minS = 999, maxS = -999, minT = 999, maxT = -999;
        long libNs = 0;
        String result = "TIMEOUT";
        final int startEval = 60;   // ignore the first second for oscillation stats

        while (frames * DT < tMax) {
            r.render(px, py, th);
            final long t0 = System.nanoTime();
            final int[] out = lib.process(r);
            libNs += System.nanoTime() - t0;
            final int S = out[0], T = out[1];
            frames++;
            minS = Math.min(minS, S); maxS = Math.max(maxS, S); minT = Math.min(minT, T); maxT = Math.max(maxT, T);

            // ── metrics on the current pose ──
            if (!lib.trackFound) { lostFrames++; if (prevFound) lostEvents++; }
            prevFound = lib.trackFound;
            if (lib.trackFound) {
                final double truth = r.truthCol(px, py, th, H_ROW, idx, 160, n, t.closed);
                final double cx = lib.centroidXFrac * W;
                if (Double.isNaN(truth) || Math.abs(cx - truth) > 80) wrongLock++;
            }
            if (frames > startEval) {
                final int sg = S > 1 ? 1 : (S < -1 ? -1 : 0);
                if (sg != 0) { if (lastSign != 0 && sg != lastSign) signChanges++; lastSign = sg; }
                sumS += S; sumS2 += (double) S * S; sumAbsS += Math.abs(S);
            }

            if (TRACE && frames % TRACE_EVERY == 0) {
                final double latT = -(px - t.x[idx]) * Math.sin(t.hd[idx]) + (py - t.y[idx]) * Math.cos(t.hd[idx]);
                System.out.printf(Locale.ROOT, "t=%6.2f s=%5.2f lat=%6.2f he=%6.1f S=%4d T=%4d off=%6.1f head=%6.1f hdeg=%6.1f conf=%.2f found=%b%n",
                        frames * DT, progress * World.STEP, 100 * latT, Math.toDegrees(Scenes.angDiff(th, t.hd[idx])), S, T,
                        lib.fd("steerOffsetTerm"), lib.fd("steerHeadingTerm"), lib.fd("headingDeg"), lib.fd("headingConfidence"), lib.trackFound);
            }
            // ── controller (lecturer's Blockly program) ──
            final double motorSteer = (0 - S) * s.c;
            final double power = Math.round(s.x * (T / 8.0) + 5);
            if ((frames - 1) % CTRL_EVERY == 0) { cmdSt = -motorSteer; cmdPw = power; }   // Blockly loop samples
            queue[qi][0] = cmdSt;         // Invert Motors = true (cmdSt = −motorSteer)
            queue[qi][1] = cmdPw;
            qi = (qi + 1) % queue.length;
            final double eff = queue[qi][0], pw = queue[qi][1];   // command from LATENCY frames ago
            final double st = Math.max(-100, Math.min(100, eff));
            final double inner = pw * (1 - Math.abs(st) / 50.0);
            final double tl = dead(st >= 0 ? pw : inner) / 100.0 * s.vmax;   // steer > 0 → turn right
            final double tr = dead(st >= 0 ? inner : pw) / 100.0 * s.vmax;

            // ── kinematics (4 sub-steps) ──
            for (int k = 0; k < 4; k++) {
                final double h = DT / 4;
                vl += (tl - vl) * h / TAU; vr += (tr - vr) * h / TAU;
                final double v = (vl + vr) / 2, om = (vr - vl) / WHEELBASE;
                px += v * Math.cos(th + om * h / 2) * h; py += v * Math.sin(th + om * h / 2) * h;
                th += om * h;
            }

            // ── progress / errors ──
            int best = idx; double bd = Double.MAX_VALUE;
            for (int d = -40; d <= 200; d++) {
                int j = idx + d;
                if (t.closed) j = ((j % n) + n) % n; else if (j < 0 || j >= n) continue;
                final double dd = (t.x[j] - px) * (t.x[j] - px) + (t.y[j] - py) * (t.y[j] - py);
                if (dd < bd) { bd = dd; best = j; }
            }
            int step = best - idx;
            if (t.closed) { if (step > n / 2) step -= n; if (step < -n / 2) step += n; }
            progress += step; idx = best;
            final double lat = -(px - t.x[idx]) * Math.sin(t.hd[idx]) + (py - t.y[idx]) * Math.cos(t.hd[idx]);
            final double he = Math.toDegrees(Math.abs(Scenes.angDiff(th, t.hd[idx])));
            // On an open track past its end the nearest point is the end point: stop there.
            if (!t.closed && idx >= n - 60) { result = "DONE"; break; }   // finish 0.30 m before the tape ends
            if (t.closed && progress >= n) { result = "DONE"; break; }
            final double al = Math.abs(lat);
            sumLat += al; maxLat = Math.max(maxLat, al);
            sumHead2 += he * he; maxHead = Math.max(maxHead, he);
            if (al > 0.01 || he > 3) settleT = frames * DT;   // last time outside 1 cm / 3°
            if (al > OFF_TRACK) { result = "LEFT@" + String.format(Locale.ROOT, "%.2fm", progress * World.STEP); break; }
        }
        final int ne = Math.max(1, frames - startEval);
        final double mean = sumS / ne;
        return row(s, result, frames * DT, frames,
                100 * sumLat / Math.max(1, frames), 100 * maxLat,
                Math.sqrt(sumHead2 / Math.max(1, frames)), maxHead,
                signChanges / Math.max(1e-9, ne * DT), Math.sqrt(Math.max(0, sumS2 / ne - mean * mean)), sumAbsS / ne,
                100.0 * lostFrames / Math.max(1, frames), lostEvents, 100.0 * wrongLock / Math.max(1, frames),
                minS, maxS, minT, maxT, libNs / 1e6 / Math.max(1, frames), settleT);
    }

    static final String HEADER = "lib,track,scheme,distract,c,x,vmax,extra,sim,result,time_s,frames,meanLat_cm,maxLat_cm,rmsHead_deg,maxHead_deg,"
            + "signChg_per_s,steerStd,steerMeanAbs,lost_pct,lostEvents,wrongLock_pct,minS,maxS,minT,maxT,lib_ms,settle_s";

    static double dead(double p) { return Math.abs(p) < SIM_DEAD ? 0 : p; }

    static String row(Spec s, String res, double time, int frames, double meanLat, double maxLat, double rmsHead, double maxHead,
                      double sc, double sstd, double smean, double lostPct, int lostEv, double wrongPct,
                      int minS, int maxS, int minT, int maxT, double ms, double settle) {
        return String.format(Locale.ROOT, "%s,%s,%s,%d,%.2f,%.0f,%.2f,%s,%s,%s,%.1f,%d,%.2f,%.2f,%.2f,%.1f,%.2f,%.2f,%.2f,%.2f,%d,%.2f,%d,%d,%d,%d,%.3f,%.2f",
                Lib.TAG, s.track, s.scheme, s.distract ? 1 : 0, s.c, s.x, s.vmax, s.extra == null ? "" : s.extra.replace(',', ' '), SIM_TAG,
                res, time, frames, meanLat, maxLat, rmsHead, maxHead, sc, sstd, smean, lostPct, lostEv, wrongPct,
                minS, maxS, minT, maxT, ms, settle);
    }

    static List<Spec> matrix(String m, String extra) {
        final List<Spec> l = new ArrayList<>();
        final String[] schemes = {"white_on_dark", "black_on_wood"};
        final double[][] ctrls = {{0.5, 0}, {0.5, 1}, {0.3, 0}, {1.0, 0}, {1.0, 1}, {2.0, 0}};
        switch (m) {
            case "main":
                for (String tr : Scenes.TRACKS) for (String sc : schemes) for (int d = 0; d < 2; d++)
                    for (double[] c : ctrls) l.add(new Spec(tr, sc, d == 1, c[0], c[1], 0.5, extra));
                break;
            case "speed":   // faster robot: 1.0 m/s at power 100
                for (String tr : Scenes.TRACKS) for (String sc : schemes) for (int d = 0; d < 2; d++)
                    for (double[] c : new double[][]{{0.5, 0}, {1.0, 0}, {1.0, 1}, {2.0, 0}}) l.add(new Spec(tr, sc, d == 1, c[0], c[1], 1.0, extra));
                break;
            case "sweep": {  // -Dsweep.t=tracks -Dsweep.s=schemes -Dsweep.d=0,1 -Dsweep.c=gains -Dsweep.x=0,1 -Dsweep.v=vmax
                final String[] tr = System.getProperty("sweep.t", String.join(",", Scenes.TRACKS)).split(",");
                final String[] sc = System.getProperty("sweep.s", "black_on_wood").split(",");
                final String[] ds = System.getProperty("sweep.d", "0").split(",");
                final String[] cs = System.getProperty("sweep.c", "0.5").split(",");
                final String[] xs = System.getProperty("sweep.x", "0").split(",");
                final String[] vs = System.getProperty("sweep.v", "0.5").split(",");
                for (String v : vs) for (String t : tr) for (String s : sc) for (String d : ds) for (String c : cs) for (String x : xs)
                    l.add(new Spec(t, s, d.equals("1"), Double.parseDouble(c), Double.parseDouble(x), Double.parseDouble(v), extra));
                break;
            }
            case "quick":
                for (String tr : Scenes.TRACKS) for (int d = 0; d < 2; d++) l.add(new Spec(tr, "black_on_wood", d == 1, 0.5, 0, 0.5, extra));
                break;
            default: throw new IllegalArgumentException(m);
        }
        return l;
    }

    public static void main(String[] a) throws Exception {
        Locale.setDefault(Locale.ROOT);
        final String out = a[0];
        final String m = a.length > 1 ? a[1] : "main";
        final int threads = a.length > 2 ? Integer.parseInt(a[2]) : Math.max(1, Runtime.getRuntime().availableProcessors() - 2);
        final String extra = a.length > 3 ? a[3] : "";
        final List<Spec> specs = matrix(m, extra);
        final ExecutorService ex = Executors.newFixedThreadPool(threads);
        final List<Future<String>> fs = new ArrayList<>();
        for (Spec s : specs) fs.add(ex.submit(() -> run(s, CAM)));
        final long t0 = System.currentTimeMillis();
        try (PrintWriter pw = new PrintWriter(out, "UTF-8")) {
            pw.println(HEADER);
            int done = 0;
            for (Future<String> f : fs) {
                final String line = f.get();
                pw.println(line); pw.flush();
                if (++done % 10 == 0) System.out.println("  " + done + "/" + specs.size() + "  " + (System.currentTimeMillis() - t0) / 1000 + " s");
            }
        }
        ex.shutdown();
        System.out.println("wrote " + out + " (" + specs.size() + " runs, " + (System.currentTimeMillis() - t0) / 1000 + " s)");
    }
}
