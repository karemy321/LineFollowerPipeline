package harness;

import java.util.ArrayList;
import java.util.List;

/**
 * Top-down floor map (raster, YUV per cell) with tape, flat distractors painted on
 * the floor, and upright distractors (chair legs / boxes) rendered as 3-D columns.
 * World frame: x right, y up (metres), heading CCW from +x.
 */
final class World {

    // ── Track centre line ──────────────────────────────────────────────────
    static final double STEP = 0.005;   // 5 mm between centre-line samples

    static final class Track {
        final double[] x, y, hd;
        final boolean closed;
        final double length;
        final String name;
        Track(String name, double[] x, double[] y, double[] hd, boolean closed) {
            this.name = name; this.x = x; this.y = y; this.hd = hd; this.closed = closed;
            this.length = (x.length - 1) * STEP;
        }
        int n() { return x.length; }
    }

    /** Builds a centre line from straight / arc pieces, sampled every 5 mm. */
    static final class PathBuilder {
        final List<double[]> pts = new ArrayList<>();
        double x, y, hd;
        PathBuilder(double x, double y, double hdDeg) { this.x = x; this.y = y; this.hd = Math.toRadians(hdDeg); add(); }
        private void add() { pts.add(new double[]{x, y, hd}); }
        PathBuilder straight(double len) {
            final int n = (int) Math.round(len / STEP);
            for (int i = 0; i < n; i++) { x += STEP * Math.cos(hd); y += STEP * Math.sin(hd); add(); }
            return this;
        }
        /** Arc of radius r; positive angle turns LEFT. */
        PathBuilder arc(double r, double angleDeg) {
            final double total = Math.toRadians(angleDeg);
            final int n = Math.max(1, (int) Math.round(Math.abs(total) * r / STEP));
            final double dA = total / n;
            for (int i = 0; i < n; i++) {
                hd += dA * 0.5;
                final double ds = Math.abs(dA) * r;
                x += ds * Math.cos(hd); y += ds * Math.sin(hd);
                hd += dA * 0.5;
                add();
            }
            return this;
        }
        Track build(String name, boolean closed) {
            final int n = pts.size() - (closed ? 1 : 0);
            final double[] xs = new double[n], ys = new double[n], hs = new double[n];
            for (int i = 0; i < n; i++) { xs[i] = pts.get(i)[0]; ys[i] = pts.get(i)[1]; hs[i] = pts.get(i)[2]; }
            return new Track(name, xs, ys, hs, closed);
        }
    }

    /** Upright object: vertical cylinder (chair leg / box edge) standing on the floor. */
    static final class Upright {
        final double x, y, r, height; final int yv, uv, vv;
        Upright(double x, double y, double r, double height, int rgb) {
            this.x = x; this.y = y; this.r = r; this.height = height;
            yv = Col.y(rgb); uv = Col.u(rgb); vv = Col.v(rgb);
        }
    }

    // ── Raster ─────────────────────────────────────────────────────────────
    final double x0, y0, res;
    final int nx, ny;
    final byte[] Y, U, V;
    /** Tape centre-line index + 1 for tape cells, 0 elsewhere (ground truth). */
    final int[] tapeIdx;
    /** 1 = flat distractor cell. */
    final byte[] distr;
    int outY, outU, outV;   // colour outside the map
    final List<Upright> uprights = new ArrayList<>();
    Track track;

    World(double x0, double y0, double x1, double y1, double res) {
        this.x0 = x0; this.y0 = y0; this.res = res;
        nx = (int) Math.ceil((x1 - x0) / res); ny = (int) Math.ceil((y1 - y0) / res);
        Y = new byte[nx * ny]; U = new byte[nx * ny]; V = new byte[nx * ny];
        tapeIdx = new int[nx * ny]; distr = new byte[nx * ny];
    }

    double cx(int i) { return x0 + (i + 0.5) * res; }
    double cy(int j) { return y0 + (j + 0.5) * res; }

    void set(int idx, int rgb) { Y[idx] = (byte) Col.y(rgb); U[idx] = (byte) Col.u(rgb); V[idx] = (byte) Col.v(rgb); }

    interface Tex { int rgb(double x, double y); }

    void fill(Tex t) {
        for (int j = 0; j < ny; j++) for (int i = 0; i < nx; i++) set(j * nx + i, t.rgb(cx(i), cy(j)));
    }

    /** Paint the tape (half-width hw) along the track with the given texture. */
    void paintTape(Track t, double hw, Tex tex) {
        this.track = t;
        final int rad = (int) Math.ceil(hw / res) + 1;
        for (int k = 0; k < t.n(); k++) {
            final int ci = (int) ((t.x[k] - x0) / res), cj = (int) ((t.y[k] - y0) / res);
            for (int dj = -rad; dj <= rad; dj++) for (int di = -rad; di <= rad; di++) {
                final int i = ci + di, j = cj + dj;
                if (i < 0 || j < 0 || i >= nx || j >= ny) continue;
                final double dx = cx(i) - t.x[k], dy = cy(j) - t.y[k];
                if (dx * dx + dy * dy > hw * hw) continue;
                final int idx = j * nx + i;
                if (tapeIdx[idx] == 0) set(idx, tex.rgb(cx(i), cy(j)));
                tapeIdx[idx] = k + 1;
            }
        }
    }

    /** Flat ellipse (bag, chair seat shadow …) centred (x,y), semi-axes a,b, rotated rotDeg. */
    void paintEllipse(double x, double y, double a, double b, double rotDeg, Tex tex) {
        final double c = Math.cos(Math.toRadians(rotDeg)), s = Math.sin(Math.toRadians(rotDeg));
        final double m = Math.max(a, b);
        for (int j = (int) ((y - m - y0) / res); j <= (int) ((y + m - y0) / res); j++)
            for (int i = (int) ((x - m - x0) / res); i <= (int) ((x + m - x0) / res); i++) {
                if (i < 0 || j < 0 || i >= nx || j >= ny) continue;
                final double dx = cx(i) - x, dy = cy(j) - y;
                final double u = (dx * c + dy * s) / a, v = (-dx * s + dy * c) / b;
                if (u * u + v * v > 1) continue;
                final int idx = j * nx + i;
                set(idx, tex.rgb(cx(i), cy(j))); distr[idx] = 1; tapeIdx[idx] = 0;
            }
    }

    /** Flat region: every cell where {@code inside} holds gets {@code tex} (e.g. the floor beyond a table edge). */
    interface Region { boolean in(double x, double y); }
    void paintRegion(Region r, Tex tex) {
        for (int j = 0; j < ny; j++) for (int i = 0; i < nx; i++) {
            final double x = cx(i), y = cy(j);
            if (!r.in(x, y)) continue;
            final int idx = j * nx + i;
            set(idx, tex.rgb(x, y)); distr[idx] = 1; tapeIdx[idx] = 0;
        }
    }

    /** Multiply every cell's brightness by gain(x,y) (uneven lighting, applied after painting). */
    void light(Tex gain1024) {
        for (int j = 0; j < ny; j++) for (int i = 0; i < nx; i++) {
            final int idx = j * nx + i;
            final int g = gain1024.rgb(cx(i), cy(j));
            Y[idx] = (byte) Col.clamp(((Y[idx] & 0xFF) * g) >> 10);
            U[idx] = (byte) Col.clamp(128 + ((((U[idx] & 0xFF) - 128) * g) >> 10));
            V[idx] = (byte) Col.clamp(128 + ((((V[idx] & 0xFF) - 128) * g) >> 10));
        }
    }

    // ── Colour helpers (BT.601, same integer formulas as the library) ─────
    static final class Col {
        static int rgb(int r, int g, int b) { return (clamp(r) << 16) | (clamp(g) << 8) | clamp(b); }
        static int y(int c) { return (77 * ((c >> 16) & 0xFF) + 150 * ((c >> 8) & 0xFF) + 29 * (c & 0xFF)) >> 8; }
        static int u(int c) { return clamp(((-43 * ((c >> 16) & 0xFF) - 85 * ((c >> 8) & 0xFF) + 128 * (c & 0xFF)) >> 8) + 128); }
        static int v(int c) { return clamp(((128 * ((c >> 16) & 0xFF) - 107 * ((c >> 8) & 0xFF) - 21 * (c & 0xFF)) >> 8) + 128); }
        static int clamp(int v) { return v < 0 ? 0 : (v > 255 ? 255 : v); }
        static int scale(int rgb, double g) {
            return rgb((int) Math.round(((rgb >> 16) & 0xFF) * g), (int) Math.round(((rgb >> 8) & 0xFF) * g),
                       (int) Math.round((rgb & 0xFF) * g));
        }
    }

    // ── Value noise (smooth, deterministic) ───────────────────────────────
    static double hash(long ix, long iy, long seed) {
        long h = ix * 374761393L + iy * 668265263L + seed * 2147483647L;
        h = (h ^ (h >>> 13)) * 1274126177L;
        h ^= h >>> 16;
        return ((h & 0xFFFFFF) / (double) 0xFFFFFF);   // [0,1]
    }
    /** Smooth value noise in [0,1] with feature size {@code scale} metres. */
    static double noise(double x, double y, double scale, long seed) {
        final double fx = x / scale, fy = y / scale;
        final long ix = (long) Math.floor(fx), iy = (long) Math.floor(fy);
        final double tx = fx - ix, ty = fy - iy;
        final double sx = tx * tx * (3 - 2 * tx), sy = ty * ty * (3 - 2 * ty);
        final double a = hash(ix, iy, seed), b = hash(ix + 1, iy, seed);
        final double c = hash(ix, iy + 1, seed), d = hash(ix + 1, iy + 1, seed);
        return (a + (b - a) * sx) + ((c + (d - c) * sx) - (a + (b - a) * sx)) * sy;
    }
}
