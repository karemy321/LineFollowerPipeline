package com.example.linefollower;

import java.io.BufferedInputStream;
import java.io.DataInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Desktop replay of a capture saved on the phone by {@link LineFollowerPipeline.Recorder}
 * ({@code lfcap_*.bin}). The frames are scaled back to the phone's frame size (bilinear), the
 * pipeline is configured with the Config saved in the capture, calibrated on the saved
 * calibration frames and then run on the saved frames, with the coast clock set from the
 * phone's timestamps. Prints phone vs replay steering / tracking per frame.
 *
 * <pre>  gradlew replay -Pcapture=C:/path/lfcap_20261110_142233_coast.bin [-Pset=glareMask=false,...]</pre>
 *
 * The replay is close, not bit-exact: the frames were downscaled (Recorder.captureScale), and the
 * pipeline starts the saved window without the phone's earlier lock / filter state.
 */
public final class CaptureReplay {

    static final class Frame {
        boolean calib;
        int frameNo, steering, tracking, flags, rejected;
        long tNs;
        float masked;
        byte[] y, u, v;
    }

    static final class Capture {
        int fullW, fullH, scale, w, h;
        boolean chroma;
        long epochMs;
        String reason, config;
        final List<Frame> frames = new ArrayList<>();
        int calibFrames() { int n = 0; for (Frame f : frames) if (f.calib) n++; return n; }
    }

    /** Per run frame: phone values and replay values. */
    static final class Result {
        boolean calibrated;
        final List<Frame> run = new ArrayList<>();
        final List<int[]> replay = new ArrayList<>();     // {steering, tracking, trackFound, coasting, recon, rejected}
    }

    static Capture read(File file) throws IOException {
        try (DataInputStream in = new DataInputStream(new BufferedInputStream(new FileInputStream(file), 1 << 16))) {
            final byte[] magic = new byte[5];
            in.readFully(magic);
            if (!"LFCAP".equals(new String(magic, "US-ASCII"))) throw new IOException("not a capture file: " + file);
            final int version = in.readUnsignedByte();
            if (version != LineFollowerPipeline.Recorder.CAPTURE_VERSION) throw new IOException("capture version " + version);
            final Capture c = new Capture();
            c.fullW = in.readInt(); c.fullH = in.readInt(); c.scale = in.readInt();
            c.w = in.readInt(); c.h = in.readInt(); c.chroma = in.readBoolean();
            c.epochMs = in.readLong();
            c.reason = in.readUTF();
            c.config = in.readUTF();
            final int n = in.readInt();
            final int cw = (c.w + 1) / 2, ch = (c.h + 1) / 2;
            for (int i = 0; i < n; i++) {
                final Frame f = new Frame();
                f.calib = in.readUnsignedByte() == 0;
                f.frameNo = in.readInt(); f.tNs = in.readLong();
                f.steering = in.readInt(); f.tracking = in.readInt();
                f.flags = in.readInt(); f.rejected = in.readInt(); f.masked = in.readFloat();
                f.y = new byte[c.w * c.h];
                in.readFully(f.y);
                if (c.chroma) {
                    f.u = new byte[cw * ch]; f.v = new byte[cw * ch];
                    in.readFully(f.u); in.readFully(f.v);
                }
                c.frames.add(f);
            }
            return c;
        }
    }

    /** Config from the {@code name=value} text saved by the recorder (unknown names are skipped). */
    static LineFollowerPipeline.Config config(String text) {
        final LineFollowerPipeline.Config cfg = new LineFollowerPipeline.Config();
        for (String line : text.split("\n")) {
            line = line.trim();
            if (line.isEmpty() || line.startsWith("#")) continue;
            final int eq = line.indexOf('=');
            if (eq > 0) set(cfg, line.substring(0, eq).trim(), line.substring(eq + 1).trim());
        }
        return cfg;
    }

    static void set(LineFollowerPipeline.Config cfg, String name, String value) {
        try {
            final Field f = LineFollowerPipeline.Config.class.getField(name);
            final Class<?> t = f.getType();
            if (t == int.class) f.setInt(cfg, Integer.parseInt(value));
            else if (t == float.class) f.setFloat(cfg, Float.parseFloat(value));
            else if (t == boolean.class) f.setBoolean(cfg, Boolean.parseBoolean(value));
            else if (t == double.class) f.setDouble(cfg, Double.parseDouble(value));
            else if (t == long.class) f.setLong(cfg, Long.parseLong(value));
        } catch (NoSuchFieldException e) {
            System.err.println("replay: unknown Config field " + name + " (skipped)");
        } catch (IllegalAccessException e) {
            throw new IllegalStateException(e);
        }
    }

    /** Bilinear upscale of a {@code w × h} plane to {@code W × H} (scale factor {@code s}). */
    static byte[] upscale(byte[] src, int w, int h, int W, int H, int s) {
        final byte[] dst = new byte[W * H];
        for (int y = 0; y < H; y++) {
            final float fy = Math.max(0f, Math.min(h - 1f, (y + 0.5f) / s - 0.5f));
            final int y0 = (int) fy, y1 = Math.min(h - 1, y0 + 1);
            final float ay = fy - y0;
            for (int x = 0; x < W; x++) {
                final float fx = Math.max(0f, Math.min(w - 1f, (x + 0.5f) / s - 0.5f));
                final int x0 = (int) fx, x1 = Math.min(w - 1, x0 + 1);
                final float ax = fx - x0;
                final float top = (src[y0 * w + x0] & 0xFF) * (1 - ax) + (src[y0 * w + x1] & 0xFF) * ax;
                final float bot = (src[y1 * w + x0] & 0xFF) * (1 - ax) + (src[y1 * w + x1] & 0xFF) * ax;
                dst[y * W + x] = (byte) Math.round(top * (1 - ay) + bot * ay);
            }
        }
        return dst;
    }

    /** Replay the capture through a fresh pipeline configured with {@code cfg}. */
    static Result replay(Capture c, LineFollowerPipeline.Config cfg) {
        final LineFollowerPipeline p = new LineFollowerPipeline();
        p.configure(cfg);
        final int W = c.fullW, H = c.fullH, s = c.scale;
        final int cw = (c.w + 1) / 2, ch = (c.h + 1) / 2, CW = (W + 1) / 2, CH = (H + 1) / 2;
        final float clock = cfg.frameIntervalMs;
        final int[] last = new int[6];
        p.setFrameCallback(d -> {
            last[2] = d.trackFound ? 1 : 0; last[3] = d.coasting ? 1 : 0;
            last[4] = d.baseReconstructed ? 1 : 0; last[5] = d.rejectedBands;
        });
        final Result r = new Result();
        long prevT = 0L;
        for (Frame f : c.frames) {
            final byte[] y = upscale(f.y, c.w, c.h, W, H, s);
            final byte[] u = c.chroma ? upscale(f.u, cw, ch, CW, CH, s) : null;
            final byte[] v = c.chroma ? upscale(f.v, cw, ch, CW, CH, s) : null;
            if (f.calib || !p.isCalibrated()) {
                if (c.chroma) p.calibrateYuv420(y, u, v, W, H); else p.calibrate(y, W, H);
                prevT = f.tNs;
                if (f.calib) continue;
            }
            // Coast clock from the phone's timestamps (the replay itself runs much faster).
            if (clock <= 0f) cfg.frameIntervalMs = prevT != 0L ? Math.max(0.1f, (f.tNs - prevT) / 1e6f) : 16.667f;
            prevT = f.tNs;
            final int[] o = c.chroma ? p.processFrameYuv420(y, u, v, W, H) : p.processFrame(y, W, H);
            last[0] = o[0]; last[1] = o[1];
            r.run.add(f);
            r.replay.add(last.clone());
        }
        cfg.frameIntervalMs = clock;
        r.calibrated = p.isCalibrated();
        return r;
    }

    private static String flags(int trackFound, int coasting, int recon, int rejected) {
        return (trackFound == 1 ? "F" : "-") + (coasting == 1 ? "C" : "-") + (recon == 1 ? "R" : "-")
                + (rejected > 0 ? "x" + rejected : "  ");
    }

    public static void main(String[] args) throws IOException {
        if (args.length < 1) {
            System.err.println("usage: CaptureReplay <lfcap_*.bin> [name=value,...]   (gradlew replay -Pcapture=... -Pset=...)");
            System.exit(2);
        }
        final Capture c = read(new File(args[0]));
        final LineFollowerPipeline.Config cfg = config(c.config);
        if (args.length > 1 && !args[1].trim().isEmpty()) {
            for (String kv : args[1].split(",")) {
                final int eq = kv.indexOf('=');
                if (eq > 0) set(cfg, kv.substring(0, eq).trim(), kv.substring(eq + 1).trim());
            }
        }
        System.out.printf(Locale.US, "capture %s: reason=%s, frame %dx%d saved at 1/%d (%dx%d)%s, %d calibration + %d frames%n",
                new File(args[0]).getName(), c.reason, c.fullW, c.fullH, c.scale, c.w, c.h,
                c.chroma ? " YUV" : " gray", c.calibFrames(), c.frames.size() - c.calibFrames());
        final Result r = replay(c, cfg);
        if (!r.calibrated) System.out.println("WARNING: the saved calibration frames did not complete calibration;"
                + " the first run frames were used to calibrate.");
        System.out.println("  frame   t_ms | phone: steer track flags | replay: steer track flags");
        long t0 = r.run.isEmpty() ? 0L : r.run.get(0).tNs;
        int sumDiff = 0, maxDiff = 0;
        for (int i = 0; i < r.run.size(); i++) {
            final Frame f = r.run.get(i);
            final int[] o = r.replay.get(i);
            final int diff = Math.abs(o[0] - f.steering);
            sumDiff += diff; maxDiff = Math.max(maxDiff, diff);
            System.out.printf(Locale.US, "  %5d %7.1f |   %+4d   %3d  %-6s |   %+4d   %3d  %-6s%s%n",
                    f.frameNo, (f.tNs - t0) / 1e6, f.steering, f.tracking,
                    flags(f.flags & 1, (f.flags >> 2) & 1, (f.flags >> 3) & 1, f.rejected),
                    o[0], o[1], flags(o[2], o[3], o[4], o[5]), diff >= 10 ? "   <-- " + diff : "");
        }
        if (!r.run.isEmpty()) System.out.printf(Locale.US,
                "steering |phone - replay|: mean %.1f, max %d over %d frames (flags: F found, C coasting, R base rebuilt, xN rejected bands)%n",
                sumDiff / (double) r.run.size(), maxDiff, r.run.size());
    }
}
