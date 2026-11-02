import com.example.linefollower.LineFollowerPipeline;
import java.util.Random;
public class CalDbg {
    static byte[] g(int[] a) { final byte[] o = new byte[a.length]; for (int i = 0; i < a.length; i++) { final int c = a[i]; o[i] = (byte) ((77 * ((c >> 16) & 255) + 150 * ((c >> 8) & 255) + 29 * (c & 255)) >> 8); } return o; }
    public static void main(String[] a) {
        final int shift = a.length > 3 ? Integer.parseInt(a[3]) : 0;
        final int W = 320, H = 240;
        final int floor = Integer.parseInt(a[0], 16), line = Integer.parseInt(a[1], 16), width = Integer.parseInt(a[2]);
        final Random rnd = new Random(1);
        final int[] px = new int[W * H];
        for (int r = 0; r < H; r++) for (int c = 0; c < W; c++) {
            final int b = Math.abs(c - W / 2) <= width / 2.0 ? line : floor;
            int o = 0xFF000000;
            for (int s = 16; s >= 0; s -= 8) o |= Math.max(0, Math.min(255, (int) Math.round(((b >> s) & 255) + rnd.nextGaussian() * 4))) << s;
            px[r * W + c] = o;
        }
        final LineFollowerPipeline p = new LineFollowerPipeline();
        p.setFrameCallback(d -> System.out.println("phase=" + d.calibPhase + " issue=" + d.calibIssue + " prog=" + d.calibProgress + " pts=" + d.pathPointCount + " roi=" + d.roiTopFrac + "-" + d.roiBotFrac + " tilt=" + d.cameraTiltUsedDeg + " S=" + d.steering + " T=" + d.tracking + " found=" + d.trackFound + " held=" + d.lineHeld));
        final boolean gray = System.getProperty("gray") != null;
        for (int i = 0; i < 25; i++) if (gray ? p.calibrate(g(px), W, H) : p.calibrateArgb(px, W, H)) { System.out.println("calibrated after " + (i + 1)); break; }
        for (int i = 0; i < 3; i++) { if (gray) p.processFrame(g(px), W, H); else p.processFrameArgb(px, W, H); }
        final int[] q = new int[W * H];
        for (int r = 0; r < H; r++) for (int c = 0; c < W; c++) { int cc = c - shift; q[r * W + c] = cc >= 0 && cc < W ? px[r * W + cc] : px[r * W + Math.min(W - 1, c + 200)]; }
        System.out.println("--- shifted " + shift);
        if (System.getProperty("turn") != null) {
            final int[] t = new int[W * H];
            for (int r = 0; r < H; r++) { final double tt = Math.max(0, (180 - r) / 120.0); final double cc = W / 2 + 220 * tt * tt;
                for (int c = 0; c < W; c++) t[r * W + c] = Math.abs(c - cc) <= width / 2.0 ? (0xFF000000 | line) : (0xFF000000 | floor); }
            for (int i = 0; i < 4; i++) p.processFrameArgb(t, W, H);
            return;
        }
        for (int i = 0; i < 3; i++) { if (gray) p.processFrame(g(q), W, H); else p.processFrameArgb(q, W, H); }
    }
}
