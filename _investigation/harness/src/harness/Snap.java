package harness;

import java.awt.image.BufferedImage;
import java.io.File;
import javax.imageio.ImageIO;

/** Debug helper: saves a rendered frame (luma) as PNG.  Snap <out.png> <track> <scheme> <distract 0|1> <s_metres> [lateral_m] [yaw_deg] */
public final class Snap {
    public static void main(String[] a) throws Exception {
        final World.Track t = Scenes.track(a[1]);
        final World w = Scenes.build(t, Scenes.scheme(a[2]), a[3].equals("1"), true);
        final int i = (int) (Double.parseDouble(a[4]) / World.STEP);
        final double lat = a.length > 5 ? Double.parseDouble(a[5]) : 0, yaw = a.length > 6 ? Math.toRadians(Double.parseDouble(a[6])) : 0;
        final double[] p = Scenes.offset(t, i, lat);
        final Render r = new Render(ClosedLoop.CAM, w, 0, ClosedLoop.H, 1, 0.25, 0.15);
        r.render(p[0], p[1], t.hd[i] + yaw);
        save(r, a[0]);
    }

    static void save(Render r, String path) throws Exception {
        final int[] argb = r.toArgb();
        final BufferedImage img = new BufferedImage(r.W, r.H, BufferedImage.TYPE_INT_RGB);
        img.setRGB(0, 0, r.W, r.H, argb, 0, r.W);
        ImageIO.write(img, "png", new File(path));
    }
}
