package com.example.linefollower;

import java.util.Arrays;
import java.util.Locale;

/**
 * Manual benchmark (not a JUnit test): ms per processFrameYuv420 at the app's real frame size,
 * 960 × 1280, app config. Frames: a gentle-curve drive weaving ±2 cm, a third with glare over the
 * near strip and a third with a partial highlight on the tape. Run with  gradlew perf.
 */
public final class PerfBench {
    public static void main(String[] args) {
        Locale.setDefault(Locale.ROOT);
        final int W = 960, H = 1280, N = 24;
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
        final LineFollowerPipeline lp = new LineFollowerPipeline();
        final LineFollowerPipeline.Config cfg = TrackSim.appConfig();
        lp.configure(cfg);
        lp.setFrameCallback(d -> { });
        for (int i = 0; i < 200 && !lp.calibrateYuv420(cal[0], cal[1], cal[2], W, H); i++) { }
        if (!lp.isCalibrated()) throw new IllegalStateException("no calibration");
        for (int i = 0; i < 5000; i++) lp.processFrameYuv420(f[i % N][0], f[i % N][1], f[i % N][2], W, H);
        final int M = 6000;
        final long[] ns = new long[M];
        for (int rep = 0; rep < 2; rep++) {
            for (int i = 0; i < M; i++) {
                final long t0 = System.nanoTime();
                lp.processFrameYuv420(f[i % N][0], f[i % N][1], f[i % N][2], W, H);
                ns[i] = System.nanoTime() - t0;
            }
        }
        Arrays.sort(ns);
        double sum = 0;
        for (long x : ns) sum += x;
        System.out.printf("ms per frame (YUV 960x1280): mean=%.3f median=%.3f p95=%.3f p99=%.3f%n",
                sum / M / 1e6, ns[M / 2] / 1e6, ns[(int) (M * 0.95)] / 1e6, ns[(int) (M * 0.99)] / 1e6);
    }
}
