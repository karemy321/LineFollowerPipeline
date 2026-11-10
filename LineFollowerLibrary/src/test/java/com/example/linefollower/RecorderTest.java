package com.example.linefollower;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;
import static org.junit.Assume.assumeTrue;

import java.io.File;
import java.io.IOException;
import java.lang.management.ManagementFactory;
import java.lang.reflect.Field;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

/**
 * LineFollowerPipeline.Recorder: CSV log, capture file, desktop replay, auto-capture, and that it
 * neither changes the pipeline's output nor allocates on the camera thread.
 */
public class RecorderTest {

    private static final int W = 320, H = 240;

    @Rule public TemporaryFolder tmp = new TemporaryFolder();

    /** Frames along a straight + curve, with glare over the strip on some of them. */
    private static List<byte[][]> frames(int n, boolean glare) {
        final TrackSim sim = new TrackSim(W, H);
        sim.floorRgb = 0xEBEBEB; sim.lineRgb = 0x1E1C1A;
        final TrackSim.Path path = new TrackSim.Path(0, 0, 0).straight(0.4).arc(0.6, 60).straight(1.0);
        sim.tracks.add(path.points());
        final List<byte[][]> out = new ArrayList<>();
        for (int i = 0; i < n; i++) {
            sim.glares.clear();
            if (glare && i % 10 >= 6) sim.glares.add(V3ScenarioTest.nearStripGlare());
            final double[] p = path.pose(Math.min(path.size() - 1, 10 + i));
            out.add(sim.toYuv420(sim.render(p[0], p[1], p[2])));
        }
        return out;
    }

    private static byte[][] offLine() {
        final TrackSim sim = new TrackSim(W, H);
        sim.floorRgb = 0xEBEBEB; sim.lineRgb = 0x1E1C1A;
        sim.tracks.add(new TrackSim.Path(0, 0, 0).straight(1.0).points());
        return sim.toYuv420(sim.render(5, 5, 0));
    }

    /** Calibrate on the first frame, then process all; returns the number of calibration calls. */
    private static int run(LineFollowerPipeline p, List<byte[][]> fr) {
        int calib = 0;
        while (calib < 100) {
            calib++;
            final byte[][] c = fr.get(0);
            if (p.calibrateYuv420(c[0], c[1], c[2], W, H)) break;
        }
        for (byte[][] f : fr) p.processFrameYuv420(f[0], f[1], f[2], W, H);
        return calib;
    }

    @Test public void recorderOffChangesNothingAndWritesNothing() {
        final List<byte[][]> fr = frames(60, true);
        final LineFollowerPipeline a = new LineFollowerPipeline(), b = new LineFollowerPipeline();
        a.configure(TrackSim.appConfig()); b.configure(TrackSim.appConfig());
        final List<int[]> sa = new ArrayList<>(), sb = new ArrayList<>();
        final File dir = new File(tmp.getRoot(), "rec");
        final LineFollowerPipeline.Recorder rec = LineFollowerPipeline.Recorder.attach(a, dir,
                d -> sa.add(new int[]{d.steering, d.tracking, d.trackFound ? 1 : 0}));
        b.setFrameCallback(d -> sb.add(new int[]{d.steering, d.tracking, d.trackFound ? 1 : 0}));
        run(a, fr); run(b, fr);
        rec.close();
        assertEquals(sb.size(), sa.size());
        for (int i = 0; i < sa.size(); i++) assertArrayEquals("frame " + i, sb.get(i), sa.get(i));
        assertFalse("no folder / files when nothing is switched on", dir.exists());
    }

    @Test public void csvLogHasOneLinePerFrameWithTheFrameData() throws IOException {
        final List<byte[][]> fr = frames(80, true);
        final LineFollowerPipeline p = new LineFollowerPipeline();
        final LineFollowerPipeline.Config cfg = TrackSim.appConfig();
        p.configure(cfg);
        final List<int[]> seen = new ArrayList<>();
        final File dir = new File(tmp.getRoot(), "rec");
        final LineFollowerPipeline.Recorder rec = LineFollowerPipeline.Recorder.attach(p, dir,
                d -> seen.add(new int[]{d.steering, d.tracking, d.trackFound ? 1 : 0, d.coasting ? 1 : 0, d.rejectedBands}));
        rec.setRecording(true);
        final int calib = run(p, fr);
        rec.close();
        assertEquals(0, rec.getDroppedLines());
        assertNotNull(rec.getLastCsvPath());
        final List<String> lines = Files.readAllLines(new File(rec.getLastCsvPath()).toPath(), StandardCharsets.US_ASCII);
        final List<String> cols = Arrays.asList(lines.get(0).split(","));
        assertEquals(LineFollowerPipeline.Recorder.CSV_COLUMNS, lines.get(0));
        assertEquals("header + one line per frame", 1 + calib + fr.size(), lines.size());
        final int iT = cols.indexOf("t_ms"), iS = cols.indexOf("steering"), iTr = cols.indexOf("tracking"),
                  iF = cols.indexOf("track_found"), iC = cols.indexOf("coasting"), iR = cols.indexOf("rejected_bands"),
                  iP = cols.indexOf("processing");
        double lastT = -1;
        for (int i = 1; i < lines.size(); i++) {
            final String[] v = lines.get(i).split(",", -1);
            assertEquals("columns in line " + i, cols.size(), v.length);
            final int[] e = seen.get(i - 1);
            assertEquals("steering " + i, e[0], Integer.parseInt(v[iS]));
            assertEquals("tracking " + i, e[1], Integer.parseInt(v[iTr]));
            assertEquals("track_found " + i, e[2], Integer.parseInt(v[iF]));
            assertEquals("coasting " + i, e[3], Integer.parseInt(v[iC]));
            assertEquals("rejected " + i, e[4], Integer.parseInt(v[iR]));
            assertEquals("processing " + i, i > calib ? 1 : 0, Integer.parseInt(v[iP]));
            final double t = Double.parseDouble(v[iT]);
            assertTrue("time increases", t >= lastT);
            lastT = t;
            for (String x : v) Double.parseDouble(x.matches("[0-9a-f]{6}") ? "0" : x);   // every field is a number
        }
        final String cfgText = new String(Files.readAllBytes(new File(rec.getLastCsvPath().replace(".csv", ".cfg.txt")).toPath()),
                StandardCharsets.UTF_8);
        assertTrue(cfgText.contains("corridorMode=" + cfg.corridorMode));
        assertTrue(cfgText.contains("glareMask=" + cfg.glareMask));
    }

    @Test public void configTextRoundTrips() throws IllegalAccessException {
        final LineFollowerPipeline.Config c = TrackSim.appConfig();
        c.glareLumaMax = 241; c.corridorProcessNoise = 1.75f; c.edgePairCheck = false;
        final LineFollowerPipeline.Config r = CaptureReplay.config(LineFollowerPipeline.Recorder.configText(c));
        for (Field f : LineFollowerPipeline.Config.class.getFields()) {
            if (java.lang.reflect.Modifier.isStatic(f.getModifiers())) continue;
            assertEquals(f.getName(), f.get(c), f.get(r));
        }
    }

    @Test public void captureReplaysOnTheDesktop() throws IOException {
        final List<byte[][]> fr = frames(90, true);
        final LineFollowerPipeline p = new LineFollowerPipeline();
        p.configure(TrackSim.appConfig());
        final File dir = new File(tmp.getRoot(), "rec");
        final List<Integer> steer = new ArrayList<>();
        final LineFollowerPipeline.Recorder rec = LineFollowerPipeline.Recorder.attach(p, dir, d -> steer.add(d.steering));
        rec.captureFrames = 60;
        rec.captureScale = 2;
        rec.setCapture(true);
        final int calib = run(p, fr);
        rec.captureNow();
        final byte[][] last = fr.get(fr.size() - 1);
        p.processFrameYuv420(last[0], last[1], last[2], W, H);   // the save happens on this frame
        rec.close();
        assertEquals(1, rec.getCaptureCount());
        final File f = new File(rec.getLastCapturePath());
        assertTrue(f.getName().endsWith("_manual.bin"));
        final CaptureReplay.Capture c = CaptureReplay.read(f);
        assertEquals(W, c.fullW); assertEquals(H, c.fullH); assertEquals(2, c.scale);
        assertTrue(c.chroma);
        assertEquals(Math.min(calib, 48), c.calibFrames());
        assertEquals(60, c.frames.size() - c.calibFrames());
        final CaptureReplay.Result r = CaptureReplay.replay(c, CaptureReplay.config(c.config));
        assertTrue("replay calibrates on the saved calibration frames", r.calibrated);
        int sum = 0, max = 0;
        for (int i = 0; i < r.run.size(); i++) {
            assertEquals("phone value saved", (int) steer.get(steer.size() - r.run.size() + i), r.run.get(i).steering);
            final int d = Math.abs(r.replay.get(i)[0] - r.run.get(i).steering);
            sum += d; max = Math.max(max, d);
        }
        final double mean = sum / (double) r.run.size();
        System.out.printf("capture replay (1/2 scale): |steering diff| mean %.2f max %d%n", mean, max);
        assertTrue("replay follows the phone: mean " + mean, mean <= 3.0);
        assertTrue("replay follows the phone: max " + max, max <= 15);
    }

    @Test public void autoCaptureSavesWhenTheLineGoes() throws IOException {
        final List<byte[][]> fr = frames(40, false);
        final byte[][] off = offLine();
        for (int i = 0; i < 12; i++) fr.add(off);
        final LineFollowerPipeline p = new LineFollowerPipeline();
        p.configure(TrackSim.appConfig());
        final File dir = new File(tmp.getRoot(), "rec");
        final LineFollowerPipeline.Recorder rec = LineFollowerPipeline.Recorder.attach(p, dir, null);
        rec.captureFrames = 20;
        rec.postTriggerFrames = 4;
        rec.setCapture(true);
        rec.setAutoCapture(true);
        run(p, fr);
        rec.close();
        assertEquals(1, rec.getCaptureCount());
        final File f = new File(rec.getLastCapturePath());
        // The line vanishes mid-image, so the pipeline coasts first: the first event is the coast.
        assertTrue(f.getName(), f.getName().endsWith("_coast.bin") || f.getName().endsWith("_lost.bin"));
        final CaptureReplay.Capture c = CaptureReplay.read(f);
        assertEquals(20, c.frames.size() - c.calibFrames());
        final CaptureReplay.Frame lastFrame = c.frames.get(c.frames.size() - 1);
        assertEquals("ends 4 frames after the event, line still gone", 0, lastFrame.flags & 1);
    }

    /** Recording + capture on: the camera thread (calibrate / processFrame + callback) allocates nothing. */
    @Test public void recordingAllocatesNothingOnTheCameraThread() {
        final java.lang.management.ThreadMXBean base = ManagementFactory.getThreadMXBean();
        assumeTrue("HotSpot allocation counter not available", base instanceof com.sun.management.ThreadMXBean);
        final com.sun.management.ThreadMXBean mx = (com.sun.management.ThreadMXBean) base;
        final long tid = Thread.currentThread().getId();
        final List<byte[][]> fr = frames(24, true);
        fr.add(offLine());
        final LineFollowerPipeline p = new LineFollowerPipeline();
        p.configure(TrackSim.appConfig());
        final int[] sink = new int[1];
        final LineFollowerPipeline.Recorder rec = LineFollowerPipeline.Recorder.attach(p, new File(tmp.getRoot(), "rec"),
                d -> sink[0] += d.steering);
        rec.captureFrames = 30;
        rec.maxCaptures = 1000;
        rec.setRecording(true);
        rec.setCapture(true);
        rec.setAutoCapture(true);
        run(p, fr);
        for (int i = 0; i < 3000; i++) { final byte[][] f = fr.get(i % fr.size()); p.processFrameYuv420(f[0], f[1], f[2], W, H); }
        final long b0 = mx.getThreadAllocatedBytes(tid);
        for (int i = 0; i < 1000; i++) {
            if (i == 500) rec.captureNow();
            final byte[][] f = fr.get(i % fr.size());
            p.processFrameYuv420(f[0], f[1], f[2], W, H);
        }
        final long bytes = mx.getThreadAllocatedBytes(tid) - b0;
        rec.close();
        assertEquals("bytes allocated on the camera thread by 1000 recorded frames", 0L, bytes);
        assertTrue("captures were saved", rec.getCaptureCount() > 0);
    }
}
