package harness;

import java.lang.reflect.Field;
import java.util.Locale;

/**
 * Calibration diagnostics: calibrate on a rendered pose (optionally with a disturbance)
 * and print, per frame, the calibration state; at the end the per-row statistics of
 * the geometry phase (read from the library's private fields).
 *
 * Usage: CalProbe <track> <scheme> <s> [disturb-spec]
 */
public final class CalProbe {
    static Object get(Object o, String f) throws Exception {
        final Field x = o.getClass().getDeclaredField(f); x.setAccessible(true); return x.get(o);
    }
    public static void main(String[] a) throws Exception {
        Locale.setDefault(Locale.ROOT);
        final World.Track t = Scenes.track(a[0]);
        final World w = Scenes.build(t, Scenes.scheme(a[1]), false, true);
        final double s = Double.parseDouble(a[2]);
        final Render r = new Render(ClosedLoop.CAM, w, 0, ClosedLoop.ROW_HI, 23, 0.25, 0.15);
        if (a.length > 3) r.disturb = Disturb.parse(a[3], w, t, s);
        final Lib lib = new Lib();
        final int i = (int) Math.round(s / World.STEP);
        int f = 0;
        boolean ok = false;
        while (f < Integer.getInteger("frames", 60) && !ok) {
            r.render(t.x[i], t.y[i], t.hd[i]);
            ok = lib.calibrate(r);
            f++;
            if ((f % 10 == 0 && f <= 30) || ok) System.out.printf("frame %d phase=%d issue=%d prog=%.2f ok=%b%n", f, lib.last.calibPhase, lib.last.calibIssue, lib.last.calibProgress, ok);
            if (System.getProperty("rows") != null && (f == 60 || ok)) {
                final int[] row = (int[]) get(lib.p, "cgRow");
                final int[] clean = (int[]) get(lib.p, "cgClean"), frames = (int[]) get(lib.p, "cgFrames");
                float[] flo0; try { flo0 = (float[]) get(lib.p, "cgSumFloor"); } catch (Exception e) { flo0 = new float[row.length]; }
                final float[] flo = flo0;
                final float[] good = (float[]) get(lib.p, "cgSumGood"), sc = (float[]) get(lib.p, "cgSumC"), sw = (float[]) get(lib.p, "cgSumW");
                if (row == null) continue;
                for (int j = 0; j < row.length; j += 2) {
                    final int fr = Math.max(1, frames[j]);
                    System.out.printf("  row %4d clean %3d/%3d good %.2f floor %.2f c=%.1f w=%.1f%n", row[j], clean[j], fr, good[j] / fr, flo[j] / fr,
                            clean[j] > 0 ? sc[j] / clean[j] : -1, clean[j] > 0 ? sw[j] / clean[j] : -1);
                }
            }
        }
        System.out.println("line model " + java.util.Arrays.toString((float[]) get(lib.p, "cmL")) + " floor models " + java.util.Arrays.toString((float[]) get(lib.p, "cmF")) + " n=" + get(lib.p, "nFloorModels") + " lumaOnly=" + get(lib.p, "modelLumaOnly"));
        System.out.println("roiTop=" + get(lib.p, "roiTop") + " roiBot=" + get(lib.p, "roiBot") + " tilt=" + get(lib.p, "gTiltDeg") + " lineW=" + get(lib.p, "lineW"));
    }
}
