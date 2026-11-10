package harness;

import java.util.Locale;

/**
 * Steering noise with the robot standing still on a straight (no motion, sensor noise only):
 * per pose, the spread of Steering, its offset / heading parts and the heading in degrees.
 * Usage: StaticProbe [frames]   (-Dlib.tag, -Dlib.set as for ClosedLoop)
 */
public final class StaticProbe {
    public static void main(String[] a) {
        Locale.setDefault(Locale.ROOT);
        final int n = a.length > 0 ? Integer.parseInt(a[0]) : 300;
        for (String scheme : new String[]{"white_on_dark", "black_on_wood"}) {
            final World w = Scenes.build(Scenes.track("straight"), Scenes.scheme(scheme), false, true);
            final World.Track t = w.track;
            // poses: centred; 0.3 cm left; 1 deg turned
            final double[][] poses = {{0, 0}, {0.003, 0}, {0, 1.0}};
            for (double[] pose : poses) {
                final Render r = new Render(ClosedLoop.CAM, w, ClosedLoop.ROW_LO, ClosedLoop.ROW_HI, 1234, 0.25, 0.15);
                final Lib lib = new Lib();
                final int i0 = 40;
                double px = t.x[i0], py = t.y[i0], th = t.hd[i0];
                if (lib.calibrateAt(r, px, py, th, 400) < 0) { System.out.println("NOCAL"); continue; }
                px -= pose[0] * Math.sin(th); py += pose[0] * Math.cos(th); th += Math.toRadians(pose[1]);
                int[] hist = new int[101];
                double sS = 0, sS2 = 0, sH = 0, sH2 = 0, sO = 0, sO2 = 0, sD = 0, sD2 = 0;
                int flips = 0, last = 0, nonzero = 0;
                for (int k = 0; k < n; k++) {
                    r.render(px, py, th);
                    final int S = lib.process(r)[0];
                    if (k < 30) continue;                       // settle
                    hist[S + 50]++;
                    if (S != 0) nonzero++;
                    final int sg = Integer.signum(S);
                    if (sg != 0) { if (last != 0 && sg != last) flips++; last = sg; }
                    final double o = lib.fd("steerOffsetTerm"), h = lib.fd("steerHeadingTerm"), d = lib.fd("headingDeg");
                    sS += S; sS2 += S * S; sO += o; sO2 += o * o; sH += h; sH2 += h * h; sD += d; sD2 += d * d;
                }
                final int m = n - 30;
                final StringBuilder hs = new StringBuilder();
                for (int v = 0; v <= 100; v++) if (hist[v] > 0) hs.append(v - 50).append(':').append(hist[v]).append(' ');
                System.out.printf("%s %-13s lat=%.1fcm yaw=%.0fdeg  S mean %+.2f sd %.2f  nonzero %d/%d flips %d | off %+.2f sd %.2f | head %+.2f sd %.2f | hdeg %+.2f sd %.2f | %s%n",
                        Lib.TAG, scheme, pose[0] * 100, pose[1], sS / m, sd(sS, sS2, m), nonzero, m, flips,
                        sO / m, sd(sO, sO2, m), sH / m, sd(sH, sH2, m), sD / m, sd(sD, sD2, m), hs);
            }
        }
    }
    static double sd(double s, double s2, int n) { return Math.sqrt(Math.max(0, s2 / n - (s / n) * (s / n))); }
}
