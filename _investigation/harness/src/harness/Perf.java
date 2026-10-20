package harness;

import java.lang.management.ManagementFactory;
import java.util.Arrays;
import java.util.Locale;

/**
 * Zero-allocation check (ThreadMXBean.getThreadAllocatedBytes over 1000 frames, all
 * three input formats, frame callback registered like the app) and per-frame cost
 * (960×1280 YUV, app config) on pre-rendered frames: clean straight, oval with
 * distractors, and a robot weaving across the line.
 * Usage: Perf <alloc|time|both>
 */
public final class Perf {
    static final int W = ClosedLoop.W, H = ClosedLoop.H, N = 64;

    public static void main(String[] a) {
        Locale.setDefault(Locale.ROOT);
        final String mode = a.length > 0 ? a[0] : "both";
        // ── pre-render frames: a drive around the oval with distractors, weaving ±2 cm ──
        final World.Track t = Scenes.track("oval");
        final World w = Scenes.build(t, Scenes.scheme("black_on_wood"), true, true);
        final Render r = new Render(ClosedLoop.CAM, w, 0, H, 21, 0.25, 0.15);
        final byte[][] ys = new byte[N][], us = new byte[N][], vs = new byte[N][];
        final int[][] argb = new int[N][];
        for (int i = 0; i < N; i++) {
            final int k = (int) ((0.05 + 4.0 * i / N) / World.STEP) % t.n();
            final double lat = 0.02 * Math.sin(i * 0.7), yaw = Math.toRadians(6 * Math.sin(i * 0.45));
            final double[] p = Scenes.offset(t, k, lat);
            r.render(p[0], p[1], t.hd[k] + yaw);
            ys[i] = r.y.clone(); us[i] = r.u.clone(); vs[i] = r.v.clone(); argb[i] = r.toArgb();
        }
        final byte[] gray0 = ys[0];
        final Lib lib = new Lib();
        r.render(t.x[0], t.y[0], t.hd[0]);
        if (lib.calibrateAt(r, t.x[0], t.y[0], t.hd[0], 300) < 0) throw new IllegalStateException("no calibration");
        // warm-up (JIT)
        for (int i = 0; i < 5000; i++) lib.p.processFrameYuv420(ys[i % N], us[i % N], vs[i % N], W, H);

        if (!mode.equals("time")) {
            final com.sun.management.ThreadMXBean mx = (com.sun.management.ThreadMXBean) ManagementFactory.getThreadMXBean();
            final long tid = Thread.currentThread().getId();
            long b0 = mx.getThreadAllocatedBytes(tid);
            for (int i = 0; i < 1000; i++) lib.p.processFrameYuv420(ys[i % N], us[i % N], vs[i % N], W, H);
            final long yuv = mx.getThreadAllocatedBytes(tid) - b0;
            // grayscale + ARGB (separate pipelines so each format is calibrated)
            final Lib lg = new Lib();
            for (int i = 0; i < 300 && !lg.p.calibrate(gray0, W, H); i++) { }
            for (int i = 0; i < 3000; i++) lg.p.processFrame(ys[i % N], W, H);
            b0 = mx.getThreadAllocatedBytes(tid);
            for (int i = 0; i < 1000; i++) lg.p.processFrame(ys[i % N], W, H);
            final long gray = mx.getThreadAllocatedBytes(tid) - b0;
            final Lib la = new Lib();
            for (int i = 0; i < 300 && !la.p.calibrateArgb(argb[0], W, H); i++) { }
            for (int i = 0; i < 3000; i++) la.p.processFrameArgb(argb[i % N], W, H);
            b0 = mx.getThreadAllocatedBytes(tid);
            for (int i = 0; i < 1000; i++) la.p.processFrameArgb(argb[i % N], W, H);
            final long ar = mx.getThreadAllocatedBytes(tid) - b0;
            System.out.printf("%s,alloc_bytes_1000_frames,yuv=%d,gray=%d,argb=%d,%s%n", Lib.TAG, yuv, gray, ar,
                    (yuv == 0 && gray == 0 && ar == 0) ? "PASS" : "FAIL");
        }
        if (!mode.equals("alloc")) {
            final int M = 6000;
            final long[] ns = new long[M];
            for (int rep = 0; rep < 2; rep++)   // second pass is reported
                for (int i = 0; i < M; i++) {
                    final long t0 = System.nanoTime();
                    lib.p.processFrameYuv420(ys[i % N], us[i % N], vs[i % N], W, H);
                    ns[i] = System.nanoTime() - t0;
                }
            Arrays.sort(ns);
            double sum = 0; for (long x : ns) sum += x;
            System.out.printf("%s,ms_per_frame_yuv960x1280,mean=%.3f,median=%.3f,p95=%.3f,p99=%.3f%n", Lib.TAG,
                    sum / M / 1e6, ns[M / 2] / 1e6, ns[(int) (M * 0.95)] / 1e6, ns[(int) (M * 0.99)] / 1e6);
        }
    }
}
