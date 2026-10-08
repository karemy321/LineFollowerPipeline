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
                if (down > 1e-6) {
                    final double t = camHeight / down;
                    final double wx = camX + t * fwd * ch + t * xc * rightX;
                    final double wy = camY + t * fwd * sh + t * xc * rightY;
                    if (onTape(tracks, tapeHalfWidth, wx, wy) || onTape(wideTracks, wideHalfWidth, wx, wy)) base = lineRgb;
                    else if (grain > 0) bright += grain * Math.sin(wx * 2 * Math.PI / 0.03 + 3 * Math.sin(wy * 8));
                }
                if (u < shadowEdgeFrac * w) bright *= shadowGain;
                if (vignette > 0) {
                    final double nx = (u - w / 2.0) / (w / 2.0), ny = (v - h / 2.0) / (h / 2.0);
                    bright *= 1 - vignette * (nx * nx + ny * ny) / 2;
                }
                px[v * w + u] = 0xFF000000
                        | (chan(base >> 16, bright) << 16) | (chan(base >> 8, bright) << 8) | chan(base, bright);
            }
        }
        return px;
    }

    private int chan(int c, double bright) {
        final double x = (c & 0xFF) * bright + rnd.nextGaussian() * noise;
        return (int) Math.max(0, Math.min(255, Math.round(x)));
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
