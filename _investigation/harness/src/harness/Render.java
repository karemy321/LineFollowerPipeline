package harness;

import java.util.Random;

/**
 * Renders the robot camera's YUV 4:2:0 view of a {@link World} for a robot pose.
 * Adds image-space uneven lighting (vignette + far rows dimmer) and Gaussian
 * sensor noise. Only rows [rowLo, rowHi) are rendered (the library reads nothing
 * else); the rest of the frame is mid-grey. Not thread-safe: one per thread.
 */
final class Render {
    final Cam cam;
    final World w;
    final int W, H, cw, chh;
    final byte[] y, u, v;
    final int rowLo, rowHi;
    double noiseSigma = 4.0;        // luma sensor noise (grey levels)
    private final short[] gain;     // image-space brightness gain ×1024 per luma pixel
    private static final int NT = 1 << 20;
    private final byte[] noiseTab = new byte[NT];
    private final Random rnd;

    Render(Cam cam, World w, int rowLo, int rowHi, long seed, double vignette, double farDim) {
        this.cam = cam; this.w = w; W = cam.W; H = cam.H;
        cw = (W + 1) / 2; chh = (H + 1) / 2;
        y = new byte[W * H]; u = new byte[cw * chh]; v = new byte[cw * chh];
        java.util.Arrays.fill(y, (byte) 128); java.util.Arrays.fill(u, (byte) 128); java.util.Arrays.fill(v, (byte) 128);
        this.rowLo = Math.max(0, rowLo); this.rowHi = Math.min(H, rowHi);
        rnd = new Random(seed);
        gain = new short[W * H];
        for (int r = 0; r < H; r++) for (int c = 0; c < W; c++) {
            final double dx = (c - W / 2.0) / (W / 2.0), dy = (r - H / 2.0) / (H / 2.0);
            final double g = (1 - vignette * 0.5 * (dx * dx + dy * dy)) * (1 - farDim * (1 - r / (double) H));
            gain[r * W + c] = (short) Math.round(1024 * g);
        }
        setNoise(noiseSigma);
    }

    void setNoise(double sigma) {
        noiseSigma = sigma;
        for (int i = 0; i < NT; i++) noiseTab[i] = (byte) Math.max(-127, Math.min(127, Math.round(rnd.nextGaussian() * sigma)));
    }

    /** Render pose (x, y, heading rad). */
    void render(double px, double py, double th) {
        final double c = Math.cos(th), s = Math.sin(th);
        final World w = this.w;
        final int nx = w.nx, ny = w.ny;
        final double inv = 1.0 / w.res, x0 = w.x0, y0 = w.y0;
        final float[] F = cam.fwd, L = cam.left;
        final byte[] WY = w.Y, WU = w.U, WV = w.V;
        int noff = rnd.nextInt(NT);
        for (int r = rowLo; r < rowHi; r++) {
            final int base = r * W;
            final boolean chromaRow = (r & 1) == 0;
            final int cbase = (r >> 1) * cw;
            for (int col = 0; col < W; col++) {
                final int i = base + col;
                final float f = F[i];
                int yy, uu, vv;
                if (f != f) { yy = w.outY; uu = w.outU; vv = w.outV; }
                else {
                    final double wx = px + f * c - L[i] * s, wy = py + f * s + L[i] * c;
                    final int mi = (int) ((wx - x0) * inv), mj = (int) ((wy - y0) * inv);
                    if (mi < 0 || mj < 0 || mi >= nx || mj >= ny || wx < x0 || wy < y0) { yy = w.outY; uu = w.outU; vv = w.outV; }
                    else { final int m = mj * nx + mi; yy = WY[m] & 0xFF; uu = WU[m] & 0xFF; vv = WV[m] & 0xFF; }
                }
                final int g = gain[i];
                int val = ((yy * g) >> 10) + noiseTab[noff++ & (NT - 1)];
                y[i] = (byte) (val < 0 ? 0 : (val > 255 ? 255 : val));
                if (chromaRow && (col & 1) == 0) {
                    final int ci = cbase + (col >> 1);
                    u[ci] = (byte) World.Col.clamp(128 + (((uu - 128) * g) >> 10) + (noiseTab[noff++ & (NT - 1)] >> 1));
                    v[ci] = (byte) World.Col.clamp(128 + (((vv - 128) * g) >> 10) + (noiseTab[noff++ & (NT - 1)] >> 1));
                }
            }
        }
        for (World.Upright o : w.uprights) drawUpright(o, px, py, c, s);
    }

    /** Vertical cylinder: project base and top, fill the column between them. */
    private void drawUpright(World.Upright o, double px, double py, double c, double s) {
        final double dx = o.x - px, dy = o.y - py;
        final double pf = dx * c + dy * s, pl = -dx * s + dy * c;   // robot frame
        final double[] b = cam.project(pf, pl, 0), t = cam.project(pf, pl, o.height);
        if (b == null || t == null) return;
        final int r0 = (int) Math.max(rowLo, Math.floor(Math.min(b[1], t[1])));
        final int r1 = (int) Math.min(rowHi - 1, Math.ceil(Math.max(b[1], t[1])));
        for (int r = r0; r <= r1; r++) {
            final double a = (b[1] == t[1]) ? 0 : (r - b[1]) / (t[1] - b[1]);
            final double colC = b[0] + a * (t[0] - b[0]);
            final double depth = b[2] + a * (t[2] - b[2]);
            final double half = cam.f * o.r / depth;
            final int c0 = (int) Math.max(0, Math.floor(colC - half)), c1 = (int) Math.min(W - 1, Math.ceil(colC + half));
            for (int col = c0; col <= c1; col++) {
                final int i = r * W + col;
                y[i] = (byte) World.Col.clamp(((o.yv * gain[i]) >> 10) + noiseTab[(i * 7) & (NT - 1)]);
                if ((r & 1) == 0 && (col & 1) == 0) {
                    final int ci = (r >> 1) * cw + (col >> 1);
                    u[ci] = (byte) o.uv; v[ci] = (byte) o.vv;
                }
            }
        }
    }

    /** Packed ARGB of the current frame (for ARGB-format tests). */
    int[] toArgb() {
        final int[] out = new int[W * H];
        for (int r = 0; r < H; r++) for (int col = 0; col < W; col++) {
            final int Y = y[r * W + col] & 0xFF, U = (u[(r >> 1) * cw + (col >> 1)] & 0xFF) - 128, V = (v[(r >> 1) * cw + (col >> 1)] & 0xFF) - 128;
            final int R = World.Col.clamp((int) Math.round(Y + 1.402 * V));
            final int G = World.Col.clamp((int) Math.round(Y - 0.344 * U - 0.714 * V));
            final int B = World.Col.clamp((int) Math.round(Y + 1.772 * U));
            out[r * W + col] = 0xFF000000 | (R << 16) | (G << 8) | B;
        }
        return out;
    }

    /**
     * Ground truth: centre column of the tape crossing image row {@code row} that
     * belongs to the track near centre-line index {@code nearIdx} (±window samples);
     * NaN if the tape does not cross that row.
     */
    double truthCol(double px, double py, double th, int row, int nearIdx, int window, int nTrack, boolean closed) {
        final double c = Math.cos(th), s = Math.sin(th);
        double best = Double.NaN, bestD = Double.MAX_VALUE;
        int runStart = -1;
        for (int col = 0; col <= W; col++) {
            boolean tape = false;
            if (col < W) {
                final int i = row * W + col;
                final float f = cam.fwd[i];
                if (f == f) {
                    final double wx = px + f * c - cam.left[i] * s, wy = py + f * s + cam.left[i] * c;
                    final int mi = (int) ((wx - w.x0) / w.res), mj = (int) ((wy - w.y0) / w.res);
                    if (mi >= 0 && mj >= 0 && mi < w.nx && mj < w.ny) {
                        final int k = w.tapeIdx[mj * w.nx + mi] - 1;
                        if (k >= 0) {
                            int d = Math.abs(k - nearIdx);
                            if (closed) d = Math.min(d, nTrack - d);
                            tape = d <= window;
                        }
                    }
                }
            }
            if (tape && runStart < 0) runStart = col;
            if (!tape && runStart >= 0) {
                final double cc = (runStart + col - 1) * 0.5;
                if (Math.abs(col - runStart) >= 3) {
                    final double d = Math.abs(cc - W / 2.0);
                    if (d < bestD) { bestD = d; best = cc; }
                }
                runStart = -1;
            }
        }
        return best;
    }
}
