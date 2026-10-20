package harness;

import java.util.Locale;

/**
 * Heading-estimate bias on curves: the robot is placed exactly ON the line and
 * TANGENT to it (true heading error 0) at several points of each track; prints the
 * library's steering, offset part, heading part and heading estimate.
 * Usage: CurveBias <track> <s1,s2,...>
 */
public final class CurveBias {
    public static void main(String[] a) {
        Locale.setDefault(Locale.ROOT);
        final World.Track t = Scenes.track(a[0]);
        final World w = Scenes.build(t, Scenes.scheme("black_on_wood"), false, true);
        for (String ss : a[1].split(",")) {
            final int i = (int) (Double.parseDouble(ss) / World.STEP);
            final Lib lib = new Lib();
            final Render r = new Render(ClosedLoop.CAM, w, ClosedLoop.ROW_LO, ClosedLoop.ROW_HI, 3, 0.25, 0.15);
            if (lib.calibrateAt(r, t.x[0], t.y[0], t.hd[0], 300) < 0) { System.out.println("NOCAL"); return; }
            // walk along the line to the test point so the lock follows
            int[] o = null;
            for (int k = 0; k <= i; k += 4) { r.render(t.x[k], t.y[k], t.hd[k]); o = lib.process(r); }
            double sS = 0, sO = 0, sH = 0, sD = 0, sC = 0; int n = 0;
            for (int k = 0; k < 20; k++) { r.render(t.x[i], t.y[i], t.hd[i]); o = lib.process(r); sS += o[0]; sO += lib.fd("steerOffsetTerm"); sH += lib.fd("steerHeadingTerm"); sD += lib.fd("headingDeg"); sC += lib.fd("headingConfidence"); n++; }
            System.out.printf("%s s=%.2f  S=%6.1f  off=%6.1f  head=%6.1f  headingDeg=%6.1f  conf=%.2f%n", a[0], i * World.STEP, sS / n, sO / n, sH / n, sD / n, sC / n);
        }
    }
}
