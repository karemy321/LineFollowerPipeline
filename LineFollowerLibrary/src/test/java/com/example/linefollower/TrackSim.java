package com.example.linefollower;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/**
 * Tiny camera simulator: renders what the robot's tilted phone camera sees of a
 * flat floor with tape track(s) on it. World units are metres; the robot pose is
 * (x, y, heading) with heading 0 = +x and positive = counter-clockwise (left).
 */
final class TrackSim {

    final int w, h;
    double camHeight = 0.15, tiltDeg = 45, vfovDeg = 60;
    double camLateral = 0;        // camera mounted this far RIGHT of the robot centre line
    double tapeHalfWidth = 0.01;  // 2 cm tape

    int    floorRgb = 0x202020, lineRgb = 0xEBEBEB;
    double grain    = 0;          // wood-grain texture amplitude (fraction of brightness)
    double vignette = 0;          // brightness loss at the image corners (fraction)
    double noise    = 3;          // per-channel sensor noise (grey levels)
    // Shadow: image columns left of shadowEdgeFrac (fraction of width) are darkened
    // to shadowGain of their brightness (robot / person shadow across the floor).
    double shadowEdgeFrac = 0, shadowGain = 1;
    // Extra (distractor) tapes: drawn with their own half-width.
    final List<double[][]> wideTracks = new ArrayList<>();
    double wideHalfWidth = 0.02;
    private final Random rnd = new Random(42);

    /**
     * Specular glare: an ellipse fixed in the IMAGE (a reflection of a distant light stays at the
     * same place in the view while the robot drives). Inside it {@code amp} grey levels are added
     * to every channel (amp of ~255 or more saturates), fading to 0 over a halo of {@code halo} x
     * the radius. With {@code tapeOnly} only the (glossy) tape gets it, not the (matte) floor; with
     * {@code centreFrac} > 0 the highlight is also limited to the tape centre: it falls off as
     * 1 - (d / (centreFrac * tapeHalfWidth))^2 with the distance d from the tape's centre line, so
     * the tape edges stay dark ("floor, edge, dark tape, bright centre, dark tape, edge, floor").
     * Centre and radii are fractions of the image width / height.
     */
    static final class Glare {
        final double cx, cy, rx, ry, amp;
        double halo = 0.15, centreFrac = 0;
        boolean tapeOnly;
        Glare(double cx, double cy, double rx, double ry, double amp) {
            this.cx = cx; this.cy = cy; this.rx = rx; this.ry = ry; this.amp = amp;
        }
        Glare halo(double h)          { halo = h; return this; }
        Glare tapeCentre(double frac) { tapeOnly = true; centreFrac = frac; return this; }
    }
    final List<Glare> glares = new ArrayList<>();

    /** A distractor drawn in the line colour: a polyline {xs, ys} with its own half-width. */
    static final class Obj {
        final double[][] pts; final double halfWidth;
        Obj(double[][] pts, double halfWidth) { this.pts = pts; this.halfWidth = halfWidth; }
    }
    final List<Obj> objects = new ArrayList<>();

    /** Each track is a dense polyline {xs, ys}. */
    final List<double[][]> tracks = new ArrayList<>();

    TrackSim(int w, int h) { this.w = w; this.h = h; }

    // ── Track building ──────────────────────────────────────────────────────

    /** Builds a centre line from straight / arc pieces, sampled every 5 mm. */
    static final class Path {
        final List<Double> xs = new ArrayList<>(), ys = new ArrayList<>(), hs = new ArrayList<>();
        double x, y, hd;
        Path(double x, double y, double hd) { this.x = x; this.y = y; this.hd = hd; add(); }
        private void add() { xs.add(x); ys.add(y); hs.add(hd); }
        Path straight(double len) {
            for (double s = 0.005; s <= len + 1e-9; s += 0.005) {
                x += 0.005 * Math.cos(hd); y += 0.005 * Math.sin(hd); add();
            }
            return this;
        }
        /** Arc of radius r; positive angle (degrees) turns left. */
        Path arc(double r, double angleDeg) {
            final double total = Math.toRadians(angleDeg), step = 0.005 / r * Math.signum(total);
            for (double a = 0; Math.abs(a) < Math.abs(total) - 1e-9; a += step) {
                hd += step;
                x += 0.005 * Math.cos(hd); y += 0.005 * Math.sin(hd); add();
            }
            return this;
        }
        double[][] points() {
            final double[] px = new double[xs.size()], py = new double[ys.size()];
            for (int i = 0; i < px.length; i++) { px[i] = xs.get(i); py[i] = ys.get(i); }
            return new double[][]{px, py};
        }
        /** Pose {x, y, heading} at sample index i (5 mm per index). */
        double[] pose(int i) { return new double[]{xs.get(i), ys.get(i), hs.get(i)}; }
        int size() { return xs.size(); }
    }

    // ── Rendering ───────────────────────────────────────────────────────────

    int[] render(double rx, double ry, double heading) {
        final int[] px = new int[w * h];
        final double f = (h / 2.0) / Math.tan(Math.toRadians(vfovDeg / 2));
        final double st = Math.sin(Math.toRadians(tiltDeg)), ct = Math.cos(Math.toRadians(tiltDeg));
        final double ch = Math.cos(heading), sh = Math.sin(heading);
        final double rightX = sh, rightY = -ch;
        final double camX = rx + camLateral * rightX, camY = ry + camLateral * rightY;

        for (int v = 0; v < h; v++) {
            final double yc = (v + 0.5 - h / 2.0) / f;
            final double down = st + ct * yc, fwd = ct - st * yc;
            for (int u = 0; u < w; u++) {
                final double xc = (u + 0.5 - w / 2.0) / f;
                int base = floorRgb;
                double bright = 1.0;
                double tapeDist = -1;          // distance to the main track's centre line (-1 = not on it)
                boolean onLine = false;
                if (down > 1e-6) {
                    final double t = camHeight / down;
                    final double wx = camX + t * fwd * ch + t * xc * rightX;
                    final double wy = camY + t * fwd * sh + t * xc * rightY;
                    tapeDist = tapeDistance(tracks, tapeHalfWidth, wx, wy);
                    onLine = tapeDist >= 0 || onTape(wideTracks, wideHalfWidth, wx, wy) || onObject(wx, wy);
                    if (onLine) base = lineRgb;
                    else if (grain > 0) bright += grain * Math.sin(wx * 2 * Math.PI / 0.03 + 3 * Math.sin(wy * 8));
                }
                double add = 0;
                for (Glare g : glares) {
                    final double nx = (u + 0.5 - g.cx * w) / (g.rx * w), ny = (v + 0.5 - g.cy * h) / (g.ry * h);
                    final double r = Math.sqrt(nx * nx + ny * ny);
                    double gf = r <= 1 ? 1 : (r < 1 + g.halo ? 1 - (r - 1) / g.halo : 0);
                    if (gf <= 0 || (g.tapeOnly && !onLine)) continue;
                    if (g.centreFrac > 0) {
                        if (tapeDist < 0) continue;
                        final double q = tapeDist / (g.centreFrac * tapeHalfWidth);
                        gf *= Math.max(0, 1 - q * q);
                    }
                    add += g.amp * gf;
                }
                if (u < shadowEdgeFrac * w) bright *= shadowGain;
                if (vignette > 0) {
                    final double nx = (u - w / 2.0) / (w / 2.0), ny = (v - h / 2.0) / (h / 2.0);
                    bright *= 1 - vignette * (nx * nx + ny * ny) / 2;
                }
                px[v * w + u] = 0xFF000000
                        | (chan(base >> 16, bright, add) << 16) | (chan(base >> 8, bright, add) << 8)
                        | chan(base, bright, add);
            }
        }
        return px;
    }

    private int chan(int c, double bright, double add) {
        final double x = (c & 0xFF) * bright + add + rnd.nextGaussian() * noise;
        return (int) Math.max(0, Math.min(255, Math.round(x)));
    }

    private boolean onObject(double x, double y) {
        for (Obj o : objects) {
            if (segDistance(o.pts, o.halfWidth, x, y) >= 0) return true;
        }
        return false;
    }

    /** Distance from (x, y) to the nearest centre line in {@code list} if within halfWidth, else -1. */
    private static double tapeDistance(List<double[][]> list, double halfWidth, double x, double y) {
        double best = -1;
        for (double[][] t : list) {
            final double d = segDistance(t, halfWidth, x, y);
            if (d >= 0 && (best < 0 || d < best)) best = d;
        }
        return best;
    }

    private static double segDistance(double[][] t, double halfWidth, double x, double y) {
        final double reach = halfWidth + 0.01;
        final double[] xs = t[0], ys = t[1];
        double best = Double.MAX_VALUE;
        for (int i = 0; i + 1 < xs.length; i++) {
            final double ax = xs[i], ay = ys[i], bx = xs[i + 1], by = ys[i + 1];
            if (Math.abs(x - ax) > reach || Math.abs(y - ay) > reach) continue;
            final double dx = bx - ax, dy = by - ay, len2 = dx * dx + dy * dy;
            double s = ((x - ax) * dx + (y - ay) * dy) / len2;
            s = Math.max(0, Math.min(1, s));
            final double ex = ax + s * dx - x, ey = ay + s * dy - y;
            best = Math.min(best, ex * ex + ey * ey);
        }
        return best <= halfWidth * halfWidth ? Math.sqrt(best) : -1;
    }

    private static boolean onTape(List<double[][]> list, double halfWidth, double x, double y) {
        final double r2 = halfWidth * halfWidth;
        final double reach = halfWidth + 0.01;
        for (double[][] t : list) {
            final double[] xs = t[0], ys = t[1];
            for (int i = 0; i + 1 < xs.length; i++) {
                final double ax = xs[i], ay = ys[i], bx = xs[i + 1], by = ys[i + 1];
                // quick reject (segments are 5 mm long)
                if (Math.abs(x - ax) > reach || Math.abs(y - ay) > reach) continue;
                final double dx = bx - ax, dy = by - ay, len2 = dx * dx + dy * dy;
                double s = ((x - ax) * dx + (y - ay) * dy) / len2;
                s = Math.max(0, Math.min(1, s));
                final double ex = ax + s * dx - x, ey = ay + s * dy - y;
                if (ex * ex + ey * ey <= r2) return true;
            }
        }
        return false;
    }

    // ── Format conversion ───────────────────────────────────────────────────

    /** {Y, U, V} with U/V averaged over 2×2 blocks (packed half-res planes). */
    byte[][] toYuv420(int[] argb) {
        final int cw = (w + 1) / 2, chH = (h + 1) / 2;
        final byte[] y = new byte[w * h], u = new byte[cw * chH], v = new byte[cw * chH];
        for (int i = 0; i < argb.length; i++) {
            final int c = argb[i];
            y[i] = (byte) ((77 * ((c >> 16) & 0xFF) + 150 * ((c >> 8) & 0xFF) + 29 * (c & 0xFF)) >> 8);
        }
        for (int r = 0; r < chH; r++) {
            for (int c = 0; c < cw; c++) {
                int su = 0, sv = 0, n = 0;
                for (int dr = 0; dr < 2; dr++) {
                    for (int dc = 0; dc < 2; dc++) {
                        final int rr = 2 * r + dr, cc = 2 * c + dc;
                        if (rr >= h || cc >= w) continue;
                        final int p = argb[rr * w + cc];
                        final int R = (p >> 16) & 0xFF, G = (p >> 8) & 0xFF, B = p & 0xFF;
                        su += ((-43 * R - 85 * G + 128 * B) >> 8) + 128;
                        sv += ((128 * R - 107 * G - 21 * B) >> 8) + 128;
                        n++;
                    }
                }
                u[r * cw + c] = (byte) (su / n);
                v[r * cw + c] = (byte) (sv / n);
            }
        }
        return new byte[][]{y, u, v};
    }

    /** The RoboPhoneApp configuration. */
    static LineFollowerPipeline.Config appConfig() {
        final LineFollowerPipeline.Config cfg = new LineFollowerPipeline.Config();
        cfg.hRoiCenterYFrac  = 0.85f;
        cfg.hRoiThickPx      = 24;
        cfg.bottomIgnoreFrac = 0.25f;
        cfg.vRoiHalfWidthPx  = 80;
        cfg.cameraTiltDeg    = 45f;
        cfg.verticalFovDeg   = 60f;
        return cfg;
    }
}
