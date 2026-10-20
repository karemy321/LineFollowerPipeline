package harness;

import java.util.ArrayList;
import java.util.List;

/** Track shapes, floor/tape colour schemes and distractor layouts used by all tests. */
final class Scenes {

    static final double TAPE_HW = 0.0125;   // 25 mm tape

    // ── Tracks (all start at the origin heading +x, with ≥ 0.4 m straight for calibration) ──
    static World.Track track(String name) {
        switch (name) {
            case "straight": return new World.PathBuilder(0, 0, 0).straight(2.0).build(name, false);
            case "straight3": return new World.PathBuilder(0, 0, 0).straight(3.0).build(name, false);   // frame tests
            case "gentle":   return new World.PathBuilder(0, 0, 0).straight(0.45).arc(0.8, 90).straight(0.5).build(name, false);
            case "scurve":   return new World.PathBuilder(0, 0, 0).straight(0.45).arc(0.4, 90).arc(0.4, -90).straight(0.5).build(name, false);
            case "sharp90":  return new World.PathBuilder(0, 0, 0).straight(0.8).arc(0.04, -90).straight(0.7).build(name, false);
            case "oval":     return new World.PathBuilder(0, 0, 0).straight(1.0).arc(0.35, 180).straight(1.0).arc(0.35, 180).build(name, true);
            default: throw new IllegalArgumentException(name);
        }
    }
    static final String[] TRACKS = {"straight", "gentle", "scurve", "sharp90", "oval"};

    // ── Colour schemes ─────────────────────────────────────────────────────
    /** Floor texture, tape colour, distractor colour, colour beyond the map. */
    static final class Scheme {
        final String name; final World.Tex floor; final int tape, distr, outside; final boolean grainy;
        Scheme(String name, World.Tex floor, int tape, int distr, int outside, boolean grainy) {
            this.name = name; this.floor = floor; this.tape = tape; this.distr = distr; this.outside = outside; this.grainy = grainy;
        }
    }

    static int rgb(int r, int g, int b) { return World.Col.rgb(r, g, b); }

    static World.Tex wood(int base, long seed) {
        return (x, y) -> {
            // Grain: stripes along x whose phase wanders, plus fine streaks and a little noise.
            final double ph = y * 30 + 2.5 * World.noise(x, y, 0.25, seed);
            final double g = 1 + 0.09 * Math.sin(2 * Math.PI * ph) + 0.05 * Math.sin(2 * Math.PI * (y * 140 + 0.5 * x))
                           + 0.06 * (World.hash((long) (x * 500), (long) (y * 500), seed) - 0.5);
            return World.Col.scale(base, g);
        };
    }
    static World.Tex carpet(int base, long seed) {
        return (x, y) -> World.Col.scale(base, 1 + 0.18 * (World.hash((long) (x * 400), (long) (y * 400), seed) - 0.5)
                                             + 0.10 * (World.noise(x, y, 0.05, seed + 1) - 0.5));
    }
    static World.Tex plain(int base) { return (x, y) -> base; }
    static World.Tex speckle(int base, long seed) {
        return (x, y) -> World.Col.scale(base, 1 + 0.08 * (World.hash((long) (x * 400), (long) (y * 400), seed) - 0.5));
    }

    static Scheme scheme(String name) {
        switch (name) {
            // Closed-loop polarities
            case "white_on_dark":  return new Scheme(name, carpet(rgb(52, 52, 56), 7), rgb(232, 232, 226), rgb(228, 228, 228), rgb(40, 40, 44), false);
            case "black_on_wood":  return new Scheme(name, wood(rgb(168, 92, 56), 11), rgb(28, 27, 30), rgb(22, 22, 24), rgb(200, 196, 186), true);
            // Extra regression schemes
            case "white_on_brown": return new Scheme(name, wood(rgb(120, 70, 30), 13), rgb(232, 232, 226), rgb(228, 228, 228), rgb(60, 50, 40), true);
            case "black_on_white": return new Scheme(name, speckle(rgb(226, 226, 222), 17), rgb(26, 26, 28), rgb(22, 22, 24), rgb(220, 220, 220), false);
            case "blue_on_yellow": return new Scheme(name, speckle(rgb(232, 200, 40), 19), rgb(30, 60, 200), rgb(30, 60, 200), rgb(220, 200, 60), false);
            case "red_on_green":   return new Scheme(name, speckle(rgb(40, 122, 40), 23), rgb(200, 40, 40), rgb(200, 40, 40), rgb(40, 122, 40), false);
            default: throw new IllegalArgumentException(name);
        }
    }

    // ── World construction ─────────────────────────────────────────────────
    static World build(World.Track t, Scheme sc, boolean distractors, boolean unevenLight) {
        double minX = 1e9, minY = 1e9, maxX = -1e9, maxY = -1e9;
        for (int i = 0; i < t.n(); i++) {
            minX = Math.min(minX, t.x[i]); maxX = Math.max(maxX, t.x[i]);
            minY = Math.min(minY, t.y[i]); maxY = Math.max(maxY, t.y[i]);
        }
        final double m = 1.0;
        final World w = new World(minX - m, minY - m, maxX + m, maxY + m, 0.002);
        w.fill(sc.floor);
        w.outY = World.Col.y(sc.outside); w.outU = World.Col.u(sc.outside); w.outV = World.Col.v(sc.outside);
        final int tape = sc.tape;
        w.paintTape(t, TAPE_HW, (x, y) -> World.Col.scale(tape, 1 + 0.04 * (World.hash((long) (x * 500), (long) (y * 500), 3) - 0.5)));
        if (distractors) addDistractors(w, t, sc);
        if (unevenLight) w.light((x, y) -> (int) Math.round(1024 * (0.72 + 0.45 * World.noise(x, y, 0.6, 99))));
        return w;
    }

    /** Distance from (x,y) to the track centre line (brute force, setup only). */
    static double distToTrack(World.Track t, double x, double y) {
        double best = Double.MAX_VALUE;
        for (int i = 0; i < t.n(); i += 2) best = Math.min(best, (t.x[i] - x) * (t.x[i] - x) + (t.y[i] - y) * (t.y[i] - y));
        return Math.sqrt(best);
    }

    /** Point at index i offset {@code lat} metres to the LEFT of the track. */
    static double[] offset(World.Track t, int i, double lat) {
        return new double[]{t.x[i] - lat * Math.sin(t.hd[i]), t.y[i] + lat * Math.cos(t.hd[i])};
    }

    /**
     * Chair-like distractors in the distractor colour: flat blobs (seat shadow, bag),
     * upright legs, a dark region along a straight (table edge / floor beyond it),
     * and blobs on the OUTSIDE of every turn (where the camera looks mid-turn).
     * Nothing is placed in the first 0.45 m (the calibration spot) or on the tape.
     */
    static void addDistractors(World w, World.Track t, Scheme sc) {
        final int col = sc.distr;
        final World.Tex dtex = (x, y) -> World.Col.scale(col, 1 + 0.06 * (World.hash((long) (x * 300), (long) (y * 300), 5) - 0.5));
        final double keep = TAPE_HW + 0.012;    // never paint within 12 mm of the tape edge
        final int startIdx = (int) (0.45 / World.STEP);
        final int n = t.n();
        int type = 0;
        double side = 1;
        for (int i = startIdx; i < n - (int) (0.15 / World.STEP); i += (int) (0.30 / World.STEP)) {
            switch (type % 5) {
                case 0: {   // wide blob beside the line (bag / seat shadow), inner edge ~5 cm from the tape edge
                    final double[] p = offset(t, i, side * 0.12);
                    paintEllipseAway(w, t, p[0], p[1], 0.11, 0.06, Math.toDegrees(t.hd[i]), dtex, keep);
                    break;
                }
                case 1: {   // two chair legs
                    final double[] p = offset(t, i, side * 0.11), q = offset(t, Math.min(n - 1, i + 30), side * 0.16);
                    w.uprights.add(new World.Upright(p[0], p[1], 0.016, 0.45, col));
                    w.uprights.add(new World.Upright(q[0], q[1], 0.016, 0.45, col));
                    break;
                }
                case 2: {   // dark/bright region along the line (floor beyond a table edge), 0.5 m long
                    final int i0 = i, i1 = Math.min(n - 1, i + (int) (0.5 / World.STEP));
                    final double sd = side;
                    final double[] a = offset(t, i0, 0), b = offset(t, i1, 0);
                    final double minx = Math.min(a[0], b[0]) - 0.4, maxx = Math.max(a[0], b[0]) + 0.4;
                    final double miny = Math.min(a[1], b[1]) - 0.4, maxy = Math.max(a[1], b[1]) + 0.4;
                    w.paintRegion((x, y) -> {
                        if (x < minx || x > maxx || y < miny || y > maxy) return false;
                        // signed lateral distance w.r.t. the local track direction at the nearest index in [i0,i1]
                        int bi = i0; double bd = Double.MAX_VALUE;
                        for (int k = i0; k <= i1; k += 4) {
                            final double d = (t.x[k] - x) * (t.x[k] - x) + (t.y[k] - y) * (t.y[k] - y);
                            if (d < bd) { bd = d; bi = k; }
                        }
                        if (bi == i0 || bi >= i1 - 4) return false;
                        final double lat = -(x - t.x[bi]) * Math.sin(t.hd[bi]) + (y - t.y[bi]) * Math.cos(t.hd[bi]);
                        return sd * lat > 0.09 && sd * lat < 0.45 && distToTrack(t, x, y) > 0.09;
                    }, dtex);
                    break;
                }
                case 3: {   // smaller blob very close to the line (inner edge ~2.5 cm from the tape edge)
                    final double[] p = offset(t, i, side * 0.075);
                    paintEllipseAway(w, t, p[0], p[1], 0.05, 0.04, Math.toDegrees(t.hd[i]), dtex, keep);
                    break;
                }
                default: {  // one thick leg close to the line
                    final double[] p = offset(t, i, side * 0.06);
                    w.uprights.add(new World.Upright(p[0], p[1], 0.02, 0.45, col));
                }
            }
            type++;
            side = -side;
        }
        // Blobs on the outside of each turn, at the apex.
        int k = startIdx;
        while (k < n - 10) {
            if (Math.abs(angDiff(t.hd[Math.min(n - 1, k + 10)], t.hd[k])) > Math.toRadians(4)) {
                int e = k;
                while (e < n - 10 && Math.abs(angDiff(t.hd[Math.min(n - 1, e + 10)], t.hd[e])) > Math.toRadians(2)) e++;
                final int apex = (k + e) / 2;
                final double turn = Math.signum(angDiff(t.hd[e], t.hd[k]));   // + = left turn
                final double[] p = offset(t, apex, -turn * 0.16);              // outside of the turn
                paintEllipseAway(w, t, p[0], p[1], 0.09, 0.07, Math.toDegrees(t.hd[apex]), dtex, keep);
                k = e + 40;
            } else k++;
        }
    }

    static void paintEllipseAway(World w, World.Track t, double x, double y, double a, double b, double rot,
                                 World.Tex tex, double keep) {
        final double c = Math.cos(Math.toRadians(rot)), s = Math.sin(Math.toRadians(rot));
        w.paintRegion((px, py) -> {
            final double dx = px - x, dy = py - y;
            if (Math.abs(dx) > Math.max(a, b) || Math.abs(dy) > Math.max(a, b)) return false;
            final double u = (dx * c + dy * s) / a, v = (-dx * s + dy * c) / b;
            return u * u + v * v <= 1 && distToTrack(t, px, py) > keep;
        }, tex);
    }

    static double angDiff(double a, double b) {
        double d = a - b;
        while (d > Math.PI) d -= 2 * Math.PI;
        while (d < -Math.PI) d += 2 * Math.PI;
        return d;
    }

    static List<String> list(String... s) { final List<String> l = new ArrayList<>(); for (String x : s) l.add(x); return l; }
}
