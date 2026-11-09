package com.example.linefollower;

import java.util.Arrays;
import java.util.Locale;

/**
 * Manual benchmark (not a JUnit test): ms per processFrameYuv420 at the app's real frame size,
 * 960 × 1280, app config. Frames: a gentle-curve drive weaving ±2 cm; a third clean, a third
 * with glare over the near strip, a third with a partial highlight on the tape. The v3 defaults
 * and the v2 flags (glareMask / corridorMode off) are timed in alternating blocks in the same
 * JVM, so machine noise affects both alike. Run with  gradlew perf.
 */
public final class PerfBench {
    static final int W = 960, H = 1280, N = 24, BLOCK = 600, BLOCKS = 10;

    public static void main(String[] args) {
        Locale.setDefault(Locale.ROOT);
        final TrackSim sim = new TrackSim(W, H);
        sim.floorRgb = 0xEBEBEB; sim.lineRgb = 0x1E1C1A;
        final TrackSim.Path path = new TrackSim.Path(0, 0, 0).straight(0.4).arc(0.6, 60).straight(1.0);
        sim.tracks.add(path.points());
        final double[] c = path.pose(10);
        final byte[][] cal = sim.toYuv420(sim.render(c[0], c[1], c[2]));
        final byte[][][] f = new byte[N][][];
        for (int i = 0; i < N; i++) {
            sim.glares.clear();
            if (i % 3 == 1) sim.glares.add(V3ScenarioTest.nearStripGlare());
            if (i % 3 == 2) sim.glares.add(V3ScenarioTest.tapeGlare(170));
            final double[] p = path.pose(20 + 6 * i);
            final double lat = 0.02 * Math.sin(i * 0.7);
            f[i] = sim.toYuv420(sim.render(p[0] - lat * Math.sin(p[2]), p[1] + lat * Math.cos(p[2]), p[2]));
        }
        final LineFollowerPipeline v3 = pipeline(V3ScenarioTest.v3Config(30), cal);
        final LineFollowerPipeline v2 = pipeline(V3ScenarioTest.v2Config(30), cal);
        for (int i = 0; i < 5000; i++) { run(v3, f[i % N]); run(v2, f[i % N]); }   // warm-up (JIT)

        final long[][] ns = new long[2][BLOCK * BLOCKS];   // [0] = v3, [1] = v2
        for (int b = 0; b < BLOCKS; b++) {
            for (int which = 0; which < 2; which++) {
                final LineFollowerPipeline p = which == 0 ? v3 : v2;
                for (int i = 0; i < BLOCK; i++) {
                    final long t0 = System.nanoTime();
                    run(p, f[i % N]);
                    ns[which][b * BLOCK + i] = System.nanoTime() - t0;
                }
            }
        }
        report("v3 defaults", ns[0]);
        report("v2 flags   ", ns[1]);
    }

    private static LineFollowerPipeline pipeline(LineFollowerPipeline.Config cfg, byte[][] cal) {
        final LineFollowerPipeline p = new LineFollowerPipeline();
        p.configure(cfg);
        p.setFrameCallback(d -> { });
        for (int i = 0; i < 200 && !p.calibrateYuv420(cal[0], cal[1], cal[2], W, H); i++) { }
        if (!p.isCalibrated()) throw new IllegalStateException("no calibration");
        return p;
    }

    private static void run(LineFollowerPipeline p, byte[][] f) {
        p.processFrameYuv420(f[0], f[1], f[2], W, H);
    }

    private static void report(String name, long[] all) {
        final String[] kind = {"clean", "strip glare", "tape glare"};
        final StringBuilder sb = new StringBuilder();
        for (int k = 0; k < 3; k++) {
            double s = 0; int n = 0;
            for (int i = 0; i < all.length; i++) if ((i % BLOCK) % N % 3 == k) { s += all[i]; n++; }
            sb.append(String.format("  %s=%.3f", kind[k], s / n / 1e6));
        }
        final long[] a = all.clone();
        Arrays.sort(a);
        double sum = 0;
        for (long x : a) sum += x;
        System.out.printf("%s ms/frame (YUV 960x1280): mean=%.3f median=%.3f p95=%.3f p99=%.3f |%s%n", name,
                sum / a.length / 1e6, a[a.length / 2] / 1e6, a[(int) (a.length * 0.95)] / 1e6,
                a[(int) (a.length * 0.99)] / 1e6, sb);
    }
}
