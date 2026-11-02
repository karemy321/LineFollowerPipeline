package harness;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Local disturbances applied while rendering (so the clean and the disturbed run
 * share one world). Each effect is defined either on the FLOOR (world coordinates:
 * a room-fixed shadow, uneven light, a colour patch, a covering object, a scratch,
 * a colour cast, the background beyond a table edge) or relative to the ROBOT /
 * camera (the robot's own shadow; glare, whose spot is where the lamp is mirrored
 * toward the camera, so it moves with the camera).
 *
 * Spec: "type:key=val,key=val;type:..." — see {@link #parse}. Types:
 *   glare        lamp mirror spot: fwd (m ahead of the axle at the reference pose), lat, rf, rl (spot radii, m), i (0…1)
 *   shadowRobot  ellipse in the robot frame: fwd, lat, a, b, k (brightness factor)
 *   shadowRoom   floor ellipse: s (track position m), lat, a (along), b (across), k
 *   uneven       floor light gradient around track position s: lo, hi (gain), w (width m)
 *   patch        floor ellipse of another colour under the line: s, lat, a, b, rgb (hex)
 *   cover        floor rectangle OVER the line: s, lat, a (along), b (across), rgb
 *   stripe       thin bright scratch beside the line: s, len, off (m from the line centre), wid, rgb
 *   cast         floor disc with tinted light: s, lat, r, du, dv
 *   edge         background beyond the line x·d > s (table edge across the track at position s): rgb
 */
final class Disturb {
    interface Effect {
        /** Modify yuv (Y, U, V of the floor point, before camera gain/noise). wx/wy world, rf/rl robot frame. */
        void apply(int[] yuv, double wx, double wy, double rf, double rl, boolean tape);
        default void frame(double px, double py, double th) { }
    }

    final List<Effect> fx = new ArrayList<>();
    final String spec;
    Disturb(String spec) { this.spec = spec; }
    boolean active() { return !fx.isEmpty(); }

    void frame(double px, double py, double th) { for (Effect e : fx) e.frame(px, py, th); }
    void apply(int[] yuv, double wx, double wy, double rf, double rl, boolean tape) {
        for (Effect e : fx) e.apply(yuv, wx, wy, rf, rl, tape);
    }

    // ── helpers ────────────────────────────────────────────────────────────
    static double smooth(double e) { return e <= 0 ? 0 : (e >= 1 ? 1 : e * e * (3 - 2 * e)); }
    /** 1 inside the ellipse (u²+v² ≤ 1), soft edge of relative width soft. */
    static double ellipse(double u, double v, double soft) {
        final double r = Math.sqrt(u * u + v * v);
        return 1 - smooth((r - 1) / soft + 1);
    }
    static void scaleLight(int[] yuv, double g) {
        yuv[0] = World.Col.clamp((int) Math.round(yuv[0] * g));
        yuv[1] = World.Col.clamp((int) Math.round(128 + (yuv[1] - 128) * g));
        yuv[2] = World.Col.clamp((int) Math.round(128 + (yuv[2] - 128) * g));
    }
    static void blend(int[] yuv, int rgb, double a) {
        final int Y = World.Col.y(rgb), U = World.Col.u(rgb), V = World.Col.v(rgb);
        yuv[0] = (int) Math.round(yuv[0] + (Y - yuv[0]) * a);
        yuv[1] = (int) Math.round(yuv[1] + (U - yuv[1]) * a);
        yuv[2] = (int) Math.round(yuv[2] + (V - yuv[2]) * a);
    }

    /** Track frame at position s (metres along the track): {x, y, cos, sin}. */
    static double[] at(World.Track t, double s) {
        final int i = Math.max(0, Math.min(t.n() - 1, (int) Math.round(s / World.STEP)));
        return new double[]{t.x[i], t.y[i], Math.cos(t.hd[i]), Math.sin(t.hd[i])};
    }

    /**
     * Parse a spec. The reference pose (robot axle at track position sRef) places the
     * robot-relative glare lamp in the world; floor effects use track positions.
     */
    static Disturb parse(String spec, World w, World.Track t) { return parse(spec, w, t, 0); }

    static Disturb parse(String spec, World w, World.Track t, double sRef) {
        final Disturb d = new Disturb(spec == null ? "" : spec);
        if (spec == null || spec.trim().isEmpty() || spec.equals("none")) return d;
        for (String part : spec.split(";")) {
            final String[] tp = part.split(":", 2);
            final String type = tp[0].trim();
            final java.util.Map<String, String> kv = new java.util.HashMap<>();
            if (tp.length > 1) for (String p : tp[1].split(",")) { final String[] e = p.split("="); if (e.length == 2) kv.put(e[0].trim(), e[1].trim()); }
            d.fx.add(make(type, kv, w, t, sRef));
        }
        return d;
    }

    static double g(java.util.Map<String, String> kv, String k, double def) { return kv.containsKey(k) ? Double.parseDouble(kv.get(k)) : def; }
    static int rgb(java.util.Map<String, String> kv, String k, int def) { return kv.containsKey(k) ? Integer.parseInt(kv.get(k), 16) : def; }

    static Effect make(String type, java.util.Map<String, String> kv, World w, World.Track t, double sRef) {
        final Cam cam = ClosedLoop.CAM;
        switch (type) {
            case "glare": {
                // Lamp high above the floor (2 m). Its mirror spot on the floor lies on the line from the
                // camera's ground point toward the lamp's ground point, at h/(h+H) of the way.
                final double H = 2.0, h = cam.h;
                final double fwd = g(kv, "fwd", 0.4), lat = g(kv, "lat", 0.012);
                final double rf = g(kv, "rf", 0.07), rl = g(kv, "rl", 0.05), I = g(kv, "i", 0.9);
                // reference camera ground point and the wanted spot (world)
                final double[] r = at(t, sRef);
                final double cgx = r[0] + cam.camFwd * r[2], cgy = r[1] + cam.camFwd * r[3];
                final double sx = r[0] + fwd * r[2] + lat * r[3], sy = r[1] + fwd * r[3] - lat * r[2];
                // spot = cg + (L − cg)·h/(h+H)  →  L = cg + (spot − cg)·(h+H)/h
                final double lx = cgx + (sx - cgx) * (h + H) / h, ly = cgy + (sy - cgy) * (h + H) / h;
                return new Effect() {
                    double spx, spy, ex, ey;   // current spot and the elongation axis (toward the lamp)
                    public void frame(double px, double py, double th) {
                        final double gx = px + cam.camFwd * Math.cos(th) + cam.camLat * Math.sin(th);
                        final double gy = py + cam.camFwd * Math.sin(th) - cam.camLat * Math.cos(th);
                        spx = gx + (lx - gx) * h / (h + H); spy = gy + (ly - gy) * h / (h + H);
                        final double dx = lx - gx, dy = ly - gy, l = Math.max(1e-6, Math.hypot(dx, dy));
                        ex = dx / l; ey = dy / l;
                    }
                    public void apply(int[] yuv, double wx, double wy, double rfx, double rlx, boolean tape) {
                        final double dx = wx - spx, dy = wy - spy;
                        final double a = (dx * ex + dy * ey) / rf, b = (-dx * ey + dy * ex) / rl;
                        final double q = a * a + b * b;
                        if (q > 9) return;
                        final double s = I * Math.exp(-q) * (tape ? 1.0 : 0.85);
                        yuv[0] = (int) Math.round(yuv[0] + s * (245 - yuv[0]));
                        yuv[1] = (int) Math.round(128 + (yuv[1] - 128) * (1 - s));
                        yuv[2] = (int) Math.round(128 + (yuv[2] - 128) * (1 - s));
                    }
                };
            }
            case "shadowRobot": {
                final double fwd = g(kv, "fwd", 0.22), lat = g(kv, "lat", 0.03), a = g(kv, "a", 0.09), b = g(kv, "b", 0.06), k = g(kv, "k", 0.45);
                return (yuv, wx, wy, rf, rl, tape) -> {
                    final double m = ellipse((rf - fwd) / a, (-rl - lat) / b, 0.25);
                    if (m > 0) scaleLight(yuv, 1 - (1 - k) * m);
                };
            }
            case "shadowRoom": {
                final double[] c = at(t, g(kv, "s", 1.0));
                final double lat = g(kv, "lat", 0.0), a = g(kv, "a", 0.08), b = g(kv, "b", 0.2), k = g(kv, "k", 0.4);
                return (yuv, wx, wy, rf, rl, tape) -> {
                    final double dx = wx - c[0], dy = wy - c[1];
                    final double al = dx * c[2] + dy * c[3], ac = -(-dx * c[3] + dy * c[2]) - lat;
                    final double m = ellipse(al / a, ac / b, 0.2);
                    if (m > 0) scaleLight(yuv, 1 - (1 - k) * m);
                };
            }
            case "uneven": {
                final double[] c = at(t, g(kv, "s", 1.0));
                final double lo = g(kv, "lo", 0.4), hi = g(kv, "hi", 1.4), wd = g(kv, "w", 0.06);
                return (yuv, wx, wy, rf, rl, tape) -> {
                    final double al = (wx - c[0]) * c[2] + (wy - c[1]) * c[3];
                    final double ac = -(-(wx - c[0]) * c[3] + (wy - c[1]) * c[2]);
                    final double gg = lo + (hi - lo) * (0.5 + 0.5 * Math.tanh(al / wd)) * (0.8 + 0.2 * Math.cos(ac / 0.12));
                    scaleLight(yuv, gg);
                };
            }
            case "patch": {
                final double[] c = at(t, g(kv, "s", 1.0));
                final double lat = g(kv, "lat", 0.0), a = g(kv, "a", 0.10), b = g(kv, "b", 0.16);
                final int col = rgb(kv, "rgb", 0x5A6E96);
                return (yuv, wx, wy, rf, rl, tape) -> {
                    if (tape) return;
                    final double dx = wx - c[0], dy = wy - c[1];
                    final double m = ellipse((dx * c[2] + dy * c[3]) / a, (-(-dx * c[3] + dy * c[2]) - lat) / b, 0.05);
                    if (m > 0) blend(yuv, col, m);
                };
            }
            case "cover": {
                final double[] c = at(t, g(kv, "s", 1.0));
                final double lat = g(kv, "lat", 0.0), a = g(kv, "a", 0.03), b = g(kv, "b", 0.05);
                final int col = rgb(kv, "rgb", 0x8C8C8C);
                return (yuv, wx, wy, rf, rl, tape) -> {
                    final double dx = wx - c[0], dy = wy - c[1];
                    final double al = dx * c[2] + dy * c[3], ac = -(-dx * c[3] + dy * c[2]) - lat;
                    if (Math.abs(al) <= a && Math.abs(ac) <= b) blend(yuv, col, 1.0);
                };
            }
            case "stripe": {
                final double s0 = g(kv, "s", 1.0), len = g(kv, "len", 0.4), off = g(kv, "off", 0.03), wid = g(kv, "wid", 0.004);
                final int col = rgb(kv, "rgb", 0xEBE1D2);
                final double[] c = at(t, s0);
                return (yuv, wx, wy, rf, rl, tape) -> {
                    if (tape) return;
                    final double dx = wx - c[0], dy = wy - c[1];
                    final double al = dx * c[2] + dy * c[3], ac = -(-dx * c[3] + dy * c[2]);
                    if (al < -len / 2 || al > len / 2) return;
                    final double e = Math.abs(ac - off);
                    if (e <= wid / 2) blend(yuv, col, 0.9);
                };
            }
            case "cast": {
                final double[] c = at(t, g(kv, "s", 1.0));
                final double lat = g(kv, "lat", 0.0), r = g(kv, "r", 0.12), du = g(kv, "du", -18), dv = g(kv, "dv", 28);
                return (yuv, wx, wy, rf, rl, tape) -> {
                    final double dx = wx - c[0], dy = wy - c[1];
                    final double al = dx * c[2] + dy * c[3], ac = -(-dx * c[3] + dy * c[2]) - lat;
                    final double m = ellipse(Math.hypot(al, ac) / r, 0, 0.3);
                    if (m <= 0) return;
                    // tinted light multiplies the colour: the chroma shift scales with brightness
                    final double k = yuv[0] / 128.0;
                    yuv[1] = World.Col.clamp((int) Math.round(yuv[1] + du * m * k));
                    yuv[2] = World.Col.clamp((int) Math.round(yuv[2] + dv * m * k));
                };
            }
            case "edge": {
                final double[] c = at(t, g(kv, "s", 1.0));
                final int col = rgb(kv, "rgb", 0x3C3C46);
                return (yuv, wx, wy, rf, rl, tape) -> {
                    final double al = (wx - c[0]) * c[2] + (wy - c[1]) * c[3];
                    if (al <= 0) return;
                    final double n = World.noise(wx, wy, 0.04, 77);
                    final int base = World.Col.scale(col, 0.8 + 0.4 * n);
                    final double ch = World.noise(wx, wy, 0.25, 78);
                    blend(yuv, ch > 0.72 ? 0x16161A : base, 1.0);    // dark "chair" blobs on the floor below
                };
            }
            default: throw new IllegalArgumentException("disturbance type " + type);
        }
    }

    @Override public String toString() { return String.format(Locale.ROOT, "Disturb[%s]", spec); }
}
