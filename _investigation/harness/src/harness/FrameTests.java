package harness;

import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Scripted frame tests on a straight track (720×960 YUV, app config).
 *
 *  distract — Task 1: chair-like distractors beside / touching / moving toward the line.
 *             Two library instances see the same poses: one the CLEAN floor, one the
 *             floor WITH the distractor. Reported: steering/tracking deviation from the
 *             clean run, frames locked on the wrong object, frames the line was lost.
 *  heading  — Task 2: robot at several heading angles with the line crossing the
 *             H-ROI at the calibrated column; the steering value per angle.
 *
 * Usage: FrameTests <distract|heading> <out.csv>
 */
public final class FrameTests {
    static final Cam CAM = ClosedLoop.CAM;
    static final int W = ClosedLoop.W, H_ROW = ClosedLoop.H_ROW;
    static final double S0 = 0.6;            // robot axle position along the track
    static final double D_HROI = 0.194;      // floor distance axle → H-ROI centre (camFwd + 0.57 h)

    interface Scenario { void apply(World w, int frame); }

    static World base(String scheme) {
        return Scenes.build(Scenes.track("straight3"), Scenes.scheme(scheme), false, true);
    }

    static World copy(World src) {
        final World w = new World(src.x0, src.y0, src.x0 + src.nx * src.res, src.y0 + src.ny * src.res, src.res);
        System.arraycopy(src.Y, 0, w.Y, 0, src.Y.length);
        System.arraycopy(src.U, 0, w.U, 0, src.U.length);
        System.arraycopy(src.V, 0, w.V, 0, src.V.length);
        System.arraycopy(src.tapeIdx, 0, w.tapeIdx, 0, src.tapeIdx.length);
        w.outY = src.outY; w.outU = src.outU; w.outV = src.outV; w.track = src.track;
        return w;
    }

    static void restore(World w, World src) {
        System.arraycopy(src.Y, 0, w.Y, 0, src.Y.length);
        System.arraycopy(src.U, 0, w.U, 0, src.U.length);
        System.arraycopy(src.V, 0, w.V, 0, src.V.length);
        System.arraycopy(src.tapeIdx, 0, w.tapeIdx, 0, src.tapeIdx.length);
        w.uprights.clear();
    }

    // ── geometry helpers: robot at (S0, latR) heading yaw; distractor positions relative to the TRACK ──
    static World.Tex dark(World w, String scheme) {
        final int c = Scenes.scheme(scheme).distr;
        return (x, y) -> World.Col.scale(c, 1 + 0.06 * (World.hash((long) (x * 300), (long) (y * 300), 5) - 0.5));
    }

    /** Ellipse whose INNER edge is {@code gap} metres from the tape edge, side +1 = left, -1 = right. */
    static void blob(World w, World.Tex t, double fwd, double side, double gap, double a, double b) {
        final double lat = side * (Scenes.TAPE_HW + gap + b);
        w.paintEllipse(S0 + fwd, lat, a, b, 0, t);
    }

    static final class Case {
        final String name; final Scenario sc; final int frames; final double latR, yawDeg;
        Case(String name, int frames, double latR, double yawDeg, Scenario sc) { this.name = name; this.frames = frames; this.latR = latR; this.yawDeg = yawDeg; this.sc = sc; }
    }

    static List<Case> cases(String scheme, World[] holder) {
        final List<Case> l = new ArrayList<>();
        // 1. Wide blob beside the line inside the H-ROI (2 cm gap), static.
        l.add(new Case("blob_beside_HROI", 90, 0.01, 0, (w, f) -> blob(w, dark(w, scheme), D_HROI, -1, 0.02, 0.10, 0.06)));
        // 2. Wide blob beside the line in the V-ROI only (1.5 cm gap), static.
        l.add(new Case("blob_beside_VROI", 90, 0.01, 0, (w, f) -> blob(w, dark(w, scheme), 0.40, -1, 0.008, 0.12, 0.08)));
        // 3. Thick vertical chair leg 3.5 cm from the tape edge, 30 cm ahead.
        l.add(new Case("chair_leg", 90, 0.01, 0, (w, f) -> w.uprights.add(new World.Upright(S0 + 0.30, -(Scenes.TAPE_HW + 0.015 + 0.025), 0.025, 0.5, Scenes.scheme(scheme).distr))));
        // 4. Dark region filling the frame edge (floor beyond a table edge), 4 cm from the tape edge.
        l.add(new Case("edge_region", 90, 0.01, 0, (w, f) -> w.paintRegion((x, y) -> y < -(Scenes.TAPE_HW + 0.02) && x > S0 - 0.2 && x < S0 + 1.0, dark(w, scheme))));
        // 5. Blob TOUCHING the line in the H-ROI (overlaps the tape edge by 5 mm).
        l.add(new Case("blob_touching_line", 90, 0.01, 0, (w, f) -> blob(w, dark(w, scheme), D_HROI, -1, -0.005, 0.08, 0.05)));
        // 5b. Blob TOUCHING the line in the V-ROI (35 cm ahead).
        l.add(new Case("blob_touching_VROI", 90, 0.01, 0, (w, f) -> blob(w, dark(w, scheme), 0.35, 1, -0.004, 0.10, 0.06)));
        // 6. Blob sliding in from the right edge to a 1 cm gap (60 f), staying (60 f), leaving (60 f).
        l.add(new Case("blob_moving_in", 180, 0.01, 0, (w, f) -> {
            final double gap = f < 60 ? 0.12 - 0.11 * f / 60.0 : (f < 120 ? 0.01 : 0.01 + 0.11 * (f - 120) / 60.0);
            blob(w, dark(w, scheme), D_HROI, -1, gap, 0.09, 0.06);
        }));
        // 7. Chair leg sliding in toward the line and back.
        l.add(new Case("leg_moving_in", 180, 0.01, 0, (w, f) -> {
            final double gap = f < 60 ? 0.12 - 0.11 * f / 60.0 : (f < 120 ? 0.01 : 0.01 + 0.11 * (f - 120) / 60.0);
            w.uprights.add(new World.Upright(S0 + 0.25, -(Scenes.TAPE_HW + gap + 0.025), 0.025, 0.5, Scenes.scheme(scheme).distr));
        }));
        // 8. Line momentarily hidden in the H-ROI (glare / object passing over it) for 4 frames.
        l.add(new Case("line_hidden_4f", 90, 0.01, 0, (w, f) -> {
            if (f >= 40 && f < 44) {
                final World b = holder[0];
                final World.Tex floor = (x, y) -> {   // copy the clean floor 4 cm to the side
                    final int i = (int) ((x - b.x0) / b.res), j = (int) ((y - 0.06 - b.y0) / b.res);
                    final int m = j * b.nx + i;
                    return yuvToRgb(b.Y[m] & 0xFF, b.U[m] & 0xFF, b.V[m] & 0xFF);
                };
                w.paintRegion((x, y) -> Math.abs(y) < 0.03 && Math.abs(x - (S0 + D_HROI)) < 0.03, floor);
            }
        }));
        // 9. Robot yawed 12° with a blob on the side the line leans toward.
        l.add(new Case("yawed_with_blob", 90, 0.0, -12, (w, f) -> blob(w, dark(w, scheme), 0.30, -1, 0.02, 0.10, 0.06)));
        // 10. Large dark region (chair seat) straddling the line 40–60 cm ahead.
        l.add(new Case("seat_over_line_ahead", 90, 0.01, 0, (w, f) -> w.paintEllipse(S0 + 0.52, -0.03, 0.10, 0.14, 0, dark(w, scheme))));
        // 11. Strong shadow darkening the floor right of the line (the screenshot failure).
        l.add(new Case("shadow_beside_line", 90, 0.01, 0, (w, f) -> {
            for (int j = 0; j < w.ny; j++) for (int i = 0; i < w.nx; i++) {
                final double x = w.cx(i), y = w.cy(j);
                if (y < -(Scenes.TAPE_HW + 0.004) && y > -0.25 && x > S0 - 0.2 && x < S0 + 1.2) {
                    final int m = j * w.nx + i;
                    w.Y[m] = (byte) (((w.Y[m] & 0xFF) * 45) / 100);
                    w.U[m] = (byte) (128 + (((w.U[m] & 0xFF) - 128) * 45) / 100);
                    w.V[m] = (byte) (128 + (((w.V[m] & 0xFF) - 128) * 45) / 100);
                }
            }
        }));
        return l;
    }

    static int yuvToRgb(int Y, int U, int V) {
        return World.Col.rgb((int) Math.round(Y + 1.402 * (V - 128)), (int) Math.round(Y - 0.344 * (U - 128) - 0.714 * (V - 128)),
                             (int) Math.round(Y + 1.772 * (U - 128)));
    }

    static String distractRow(String scheme, Case c, World clean, World dirty) {
        final Lib a = new Lib(), b = new Lib();
        restore(dirty, clean);
        final Render ra = new Render(CAM, clean, ClosedLoop.ROW_LO, ClosedLoop.ROW_HI, 5, 0.25, 0.15);
        final Render rb = new Render(CAM, dirty, ClosedLoop.ROW_LO, ClosedLoop.ROW_HI, 5, 0.25, 0.15);
        // Calibrate both on the clean, correctly placed robot.
        if (a.calibrateAt(ra, S0, 0, 0, 300) < 0 || b.calibrateAt(rb, S0, 0, 0, 300) < 0) return scheme + "," + c.name + ",NOCAL";
        final double th = Math.toRadians(c.yawDeg);
        // Settle 20 frames at the test pose before the distractor appears.
        for (int i = 0; i < 20; i++) { ra.render(S0, c.latR, th); a.process(ra); rb.render(S0, c.latR, th); b.process(rb); }
        double maxDS = 0, sumDS = 0, maxDT = 0, maxJump = 0;
        int wrong = 0, lost = 0, n = 0;
        double prevCx = Double.NaN;
        for (int f = 0; f < c.frames; f++) {
            restore(dirty, clean);
            c.sc.apply(dirty, f);
            ra.render(S0, c.latR, th); final int[] oa = a.process(ra).clone();
            rb.render(S0, c.latR, th); final int[] ob = b.process(rb).clone();
            final double dS = Math.abs(ob[0] - oa[0]), dT = Math.abs(ob[1] - oa[1]);
            maxDS = Math.max(maxDS, dS); sumDS += dS; maxDT = Math.max(maxDT, dT); n++;
            final double truth = rb.truthCol(S0, c.latR, th, H_ROW, (int) ((S0 + D_HROI) / World.STEP), 200, dirty.track.n(), false);
            if (b.trackFound) {
                final double cx = b.centroidXFrac * W;
                if (!Double.isNaN(truth) && Math.abs(cx - truth) > 80) wrong++;
                if (!Double.isNaN(prevCx)) maxJump = Math.max(maxJump, Math.abs(cx - prevCx));
                prevCx = cx;
            } else if (a.trackFound) lost++;
        }
        return String.format(Locale.ROOT, "%s,%s,%s,%d,%.1f,%.2f,%.1f,%d,%d,%.0f", Lib.TAG, scheme, c.name, n, maxDS, sumDS / n, maxDT, wrong, lost, maxJump);
    }

    // ── heading test ──────────────────────────────────────────────────────
    static List<String> heading(String scheme) {
        final List<String> out = new ArrayList<>();
        final World w = base(scheme);
        for (double crossPx : new double[]{0, 60}) {
            for (double ang : new double[]{-30, -20, -15, -10, -5, 0, 5, 10, 15, 20, 30}) {
                final Lib lib = new Lib();
                final Render r = new Render(CAM, w, ClosedLoop.ROW_LO, ClosedLoop.ROW_HI, 9, 0.25, 0.15);
                if (lib.calibrateAt(r, S0, 0, 0, 300) < 0) { out.add(scheme + ",NOCAL"); continue; }
                final double refCol = lib.p.getSteerRefXFrac() * W;
                // Floor point (robot frame) seen at the calibrated column (+crossPx) on the H-ROI centre row.
                final double[] P = CAM.floorAt(refCol + crossPx, H_ROW + 0.5);
                // Robot rotated by `ang` (positive = line appears to head RIGHT); put P on the line y = 0.
                final double th = Math.toRadians(ang);
                final double ry = -(P[0] * Math.sin(th) + P[1] * Math.cos(th));
                final double rx = S0 - (P[0] * Math.cos(th) - P[1] * Math.sin(th)) + 0.2;
                double sS = 0, sT = 0, sOff = 0, sHd = 0, sTilt = 0; int n = 0, found = 0;
                // Approach in 10 frames from the calibrated pose so the lock follows, then measure 30 frames.
                for (int k = 1; k <= 40; k++) {
                    final double a = Math.min(1.0, k / 10.0);
                    final double x = S0 + (rx - S0) * a, y = ry * a, h = th * a;
                    r.render(x, y, h);
                    final int[] o = lib.process(r);
                    if (k > 10) {
                        sS += o[0]; sT += o[1]; n++; if (lib.trackFound) found++;
                        sOff += lib.fd("steerOffsetTerm"); sHd += lib.fd("headingDeg"); sTilt += lib.tiltDeg;
                    }
                }
                // Distance from the robot axle to the line and robot heading error (ground truth).
                out.add(String.format(Locale.ROOT, "%s,%s,%.0f,%.0f,%.1f,%.1f,%.1f,%.1f,%.1f,%d,%.1f",
                        Lib.TAG, scheme, crossPx, ang, sS / n, sT / n, sOff / n, sHd / n, sTilt / n, found, 100 * ry));
            }
        }
        return out;
    }

    public static void main(String[] args) throws Exception {
        Locale.setDefault(Locale.ROOT);
        final String mode = args[0];
        if (mode.equals("debug")) {   // debug <scheme> <case> : per-frame dump of the dirty run
            final String scheme = args[1];
            final World clean = base(scheme), dirty = copy(clean);
            for (Case c : cases(scheme, new World[]{clean})) {
                if (!c.name.equals(args[2])) continue;
                final Lib b = new Lib();
                final Render rb = new Render(CAM, dirty, ClosedLoop.ROW_LO, ClosedLoop.ROW_HI, 5, 0.25, 0.15);
                restore(dirty, clean);
                b.calibrateAt(rb, S0, 0, 0, 300);
                final double th = Math.toRadians(c.yawDeg);
                for (int f = -5; f < Math.min(c.frames, 60); f++) {
                    restore(dirty, clean);
                    if (f >= 0) c.sc.apply(dirty, f);
                    rb.render(S0, c.latR, th);
                    final int[] o = b.process(rb);
                    final double truth = rb.truthCol(S0, c.latR, th, H_ROW, (int) ((S0 + D_HROI) / World.STEP), 200, dirty.track.n(), false);
                    if (System.getProperty("path") != null && f >= -1 && f <= 0) { final StringBuilder sb = new StringBuilder("   path:"); for (int q = 0; q < b.last.lookAheadPointCount; q++) sb.append(String.format(Locale.ROOT, " %.0f@%.0f", b.last.lookAheadColFrac[q] * W, b.last.lookAheadRowFrac[q] * 960)); System.out.println(sb); }
                    System.out.printf(Locale.ROOT, "f=%3d S=%4d T=%4d cx=%6.1f truth=%6.1f w=%5.1f off=%6.1f head=%6.1f hdeg=%6.1f conf=%.2f held=%.0f rej=%.0f thr=%d la=%d%n",
                            f, o[0], o[1], b.centroidXFrac * W, truth, b.trackWidthFrac * W, b.fd("steerOffsetTerm"), b.fd("steerHeadingTerm"),
                            b.fd("headingDeg"), b.fd("headingConfidence"), b.fd("lineHeld"), b.fd("rejectedBands"), b.threshold, b.last.lookAheadPointCount);
                }
                if (args.length > 3) Snap.save(rb, args[3]);
            }
            return;
        }
        try (PrintWriter pw = new PrintWriter(args[1], "UTF-8")) {
            if (mode.equals("distract")) {
                pw.println("lib,scheme,case,frames,maxDeltaSteer,meanDeltaSteer,maxDeltaTrack,wrongLockFrames,lostFrames,maxCentroidJumpPx");
                for (String scheme : new String[]{"black_on_wood", "white_on_dark"}) {
                    final World clean = base(scheme);
                    final World dirty = copy(clean);
                    final World[] holder = {clean};
                    for (Case c : cases(scheme, holder)) {
                        final String line = distractRow(scheme, c, clean, dirty);
                        System.out.println(line);
                        pw.println(line);
                    }
                }
            } else {
                pw.println("lib,scheme,crossOffsetPx,angleDeg,steering,tracking,offsetTerm,headingDeg,tiltDeg,foundFrames,axleLateral_cm");
                for (String scheme : new String[]{"black_on_wood", "white_on_dark"})
                    for (String l : heading(scheme)) { System.out.println(l); pw.println(l); }
            }
        }
    }
}
