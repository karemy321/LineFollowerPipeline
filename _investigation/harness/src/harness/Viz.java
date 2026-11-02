package harness;

import com.example.linefollower.LineFollowerPipeline;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.image.BufferedImage;
import java.io.File;
import javax.imageio.ImageIO;

/**
 * Debug picture of one frame with the library's v3 overlay data drawn on it:
 * ROI (white), unreliable grid cells (red), path points (green = measured now,
 * cyan = memory, orange = unreliable region filled from memory, yellow =
 * bridged), pure-pursuit target (magenta), v2 strip (thin cyan lines).
 *
 * Usage: Viz <out.png> <track> <scheme> <distract 0|1> <s_metres> [lat_m] [yaw_deg] [disturb-spec]
 * The robot calibrates at the track start and drives along the line to s (memory builds up).
 */
public final class Viz {
    public static void main(String[] a) throws Exception {
        final World.Track t = Scenes.track(a[1]);
        final World w = Scenes.build(t, Scenes.scheme(a[2]), a[3].equals("1"), true);
        final double sEnd = Double.parseDouble(a[4]);
        final double lat = a.length > 5 ? Double.parseDouble(a[5]) : 0, yaw = a.length > 6 ? Math.toRadians(Double.parseDouble(a[6])) : 0;
        final String dist = a.length > 7 ? a[7] : "";
        final Disturb d = Disturb.parse(dist, w, t);
        final Render r = new Render(ClosedLoop.CAM, w, 0, ClosedLoop.H, 1, 0.25, 0.15);
        r.disturb = d;
        final Lib lib = new Lib();
        final double s0 = Double.parseDouble(System.getProperty("viz.s0", "0"));
        final int i0 = (int) (s0 / World.STEP);
        if (lib.calibrateAt(r, t.x[i0], t.y[i0], t.hd[i0], 400) < 0) System.out.println("NO CALIBRATION issue=" + lib.calibIssue);
        final int iEnd = (int) (sEnd / World.STEP);
        int[] o = null;
        for (int i = i0; i <= iEnd; i += 2) {
            final double f = iEnd == i0 ? 1 : (i - i0) / (double) (iEnd - i0);
            final double[] p = Scenes.offset(t, i, lat * f);
            r.render(p[0], p[1], t.hd[i] + yaw * f);
            o = lib.process(r);
            if (System.getProperty("viz.trace") != null) System.out.println(line(lib, i * World.STEP, o));
        }
        if (o == null) { final double[] p = Scenes.offset(t, iEnd, lat); r.render(p[0], p[1], t.hd[iEnd] + yaw); o = lib.process(r); }
        System.out.println(line(lib, sEnd, o));
        save(r, lib.last, a[0]);
    }

    static String line(Lib lib, double s, int[] o) {
        final LineFollowerPipeline.FrameData d = lib.last;
        return String.format(java.util.Locale.ROOT,
                "s=%.2f S=%d T=%d found=%b held=%b pts=%d fresh=%d mem=%d unrel=%d memFrac=%.2f off=%.3f head=%.1f k=%.2f kA=%.2f len=%.2f conf=%.2f tgt=(%.2f,%.2f) mot=(%.3f,%.3f,%.2f) cells=%d tilt=%.1f wf=%.3f",
                s, o[0], o[1], d.trackFound, d.lineHeld, d.pathPointCount, d.pathFreshPoints, d.pathMemoryPoints, d.pathUnreliablePoints,
                d.pathMemoryFrac, d.pathOffset, d.pathHeadingDeg, d.pathCurvature, d.pathCurvatureAhead, d.pathLength, d.pathConfidence,
                d.lookAheadFwd, d.lookAheadLat, d.motionFwd, d.motionLat, d.motionYawDeg, d.unreliableCells, d.cameraTiltUsedDeg, d.lineWidthFloor);
    }

    static void save(Render r, LineFollowerPipeline.FrameData d, String path) throws Exception {
        final int[] argb = r.toArgb();
        final BufferedImage img = new BufferedImage(r.W, r.H, BufferedImage.TYPE_INT_RGB);
        img.setRGB(0, 0, r.W, r.H, argb, 0, r.W);
        final Graphics2D g = img.createGraphics();
        if (d != null) draw(g, d, r.W, r.H);
        g.dispose();
        ImageIO.write(img, "png", new File(path));
    }

    static void draw(Graphics2D g, LineFollowerPipeline.FrameData d, int W, int H) {
        final int GC = LineFollowerPipeline.RELIABILITY_GRID_COLS, GR = LineFollowerPipeline.RELIABILITY_GRID_ROWS;
        final float top = d.roiTopFrac * H, bot = d.roiBotFrac * H;
        g.setColor(new Color(255, 0, 0, 90));
        for (int gy = 0; gy < GR; gy++) for (int gx = 0; gx < GC; gx++) {
            if (d.unreliableGrid[gy * GC + gx] == 0) continue;
            g.fillRect(Math.round(gx * W / (float) GC), Math.round(top + gy * (bot - top) / GR), Math.round(W / (float) GC), Math.round((bot - top) / GR) + 1);
        }
        g.setStroke(new BasicStroke(3));
        g.setColor(Color.WHITE);
        g.drawRect(1, Math.round(top), W - 3, Math.round(bot - top));
        g.setColor(new Color(0, 255, 255, 160));
        g.setStroke(new BasicStroke(1));
        g.drawLine(0, Math.round(d.hRoiTopFrac * H), W, Math.round(d.hRoiTopFrac * H));
        g.drawLine(0, Math.round(d.hRoiBotFrac * H), W, Math.round(d.hRoiBotFrac * H));
        for (int i = 0; i < d.pathPointCount; i++) {
            final Color c;
            switch (d.pathSource[i]) {
                case LineFollowerPipeline.PATH_SRC_FRESH: c = Color.GREEN; break;
                case LineFollowerPipeline.PATH_SRC_MEMORY: c = Color.CYAN; break;
                case LineFollowerPipeline.PATH_SRC_UNRELIABLE: c = Color.ORANGE; break;
                default: c = Color.YELLOW;
            }
            g.setColor(c);
            final int x = Math.round(d.pathColFrac[i] * W), y = Math.round(d.pathRowFrac[i] * H);
            g.fillOval(x - 6, y - 6, 12, 12);
        }
        if (d.lookAheadValid) {
            g.setColor(Color.MAGENTA);
            final int x = Math.round(d.lookAheadXFrac * W), y = Math.round(d.lookAheadYFrac * H);
            g.setStroke(new BasicStroke(4));
            g.drawOval(x - 14, y - 14, 28, 28);
        }
        g.setColor(Color.WHITE);
        g.drawString(String.format("S=%d T=%d %s", d.steering, d.tracking, d.trackFound ? "found" : (d.lineHeld ? "held" : "LOST")), 10, 20);
    }
}
