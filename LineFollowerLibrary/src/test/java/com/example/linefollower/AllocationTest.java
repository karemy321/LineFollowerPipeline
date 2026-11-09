package com.example.linefollower;

import static org.junit.Assert.assertEquals;
import static org.junit.Assume.assumeTrue;

import java.lang.management.ManagementFactory;

import org.junit.Test;

/**
 * Plan §1 / §7: processFrame* must not allocate. Frames include clean, glare-over-strip and
 * line-lost views so the glare / coast code paths run too. Measured with the per-thread
 * allocation counter of HotSpot (exact; JIT work happens on other threads).
 */
public class AllocationTest {

    private static final int W = 320, H = 240, N = 12;

    @Test public void processFrameAllocatesNothing() {
        final java.lang.management.ThreadMXBean base = ManagementFactory.getThreadMXBean();
        assumeTrue("HotSpot allocation counter not available", base instanceof com.sun.management.ThreadMXBean);
        final com.sun.management.ThreadMXBean mx = (com.sun.management.ThreadMXBean) base;
        final long tid = Thread.currentThread().getId();

        final TrackSim sim = new TrackSim(W, H);
        sim.floorRgb = 0xEBEBEB; sim.lineRgb = 0x1E1C1A;
        final TrackSim.Path path = new TrackSim.Path(0, 0, 0).straight(0.4).arc(0.6, 60).straight(1.0);
        sim.tracks.add(path.points());
        final byte[][][] yuv = new byte[N][][];
        final int[][] argb = new int[N][];
        for (int i = 0; i < N; i++) {
            sim.glares.clear();
            if (i % 3 == 1) sim.glares.add(V3ScenarioTest.nearStripGlare());
            if (i % 3 == 2) sim.glares.add(V3ScenarioTest.tapeGlare(170));
            final double[] p = path.pose(20 + 8 * i);
            argb[i] = i == N - 1 ? sim.render(5, 5, 0) : sim.render(p[0], p[1], p[2]);   // last: line lost
            yuv[i] = sim.toYuv420(argb[i]);
        }
        sim.glares.clear();
        final double[] c = path.pose(10);
        final int[] calib = sim.render(c[0], c[1], c[2]);
        final byte[][] cy = sim.toYuv420(calib);

        for (int fmt = 0; fmt < 3; fmt++) {
            final LineFollowerPipeline p = new LineFollowerPipeline();
            p.configure(TrackSim.appConfig());
            final int[] sink = new int[1];
            p.setFrameCallback(d -> sink[0] += d.steering);          // like the app: callback registered
            for (int i = 0; i < 100; i++) {
                if (fmt == 0 ? p.calibrate(cy[0], W, H)
                     : fmt == 1 ? p.calibrateYuv420(cy[0], cy[1], cy[2], W, H) : p.calibrateArgb(calib, W, H)) break;
            }
            for (int i = 0; i < 3000; i++) frame(p, fmt, yuv[i % N], argb[i % N]);   // warm-up (JIT)
            final long b0 = mx.getThreadAllocatedBytes(tid);
            for (int i = 0; i < 1000; i++) frame(p, fmt, yuv[i % N], argb[i % N]);
            final long bytes = mx.getThreadAllocatedBytes(tid) - b0;
            assertEquals(new String[]{"gray", "yuv", "argb"}[fmt] + ": bytes allocated by 1000 frames", 0L, bytes);
        }
    }

    private static void frame(LineFollowerPipeline p, int fmt, byte[][] yuv, int[] argb) {
        if (fmt == 0)      p.processFrame(yuv[0], W, H);
        else if (fmt == 1) p.processFrameYuv420(yuv[0], yuv[1], yuv[2], W, H);
        else               p.processFrameArgb(argb, W, H);
    }
}
