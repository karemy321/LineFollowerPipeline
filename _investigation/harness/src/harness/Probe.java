package harness;

import java.awt.image.BufferedImage;
import java.io.File;
import javax.imageio.ImageIO;

/** Debug helper: print rows/columns where strongly cyan or magenta overlay pixels are. Probe <img> <x> <y> */
public final class Probe {
    public static void main(String[] a) throws Exception {
        final BufferedImage img = ImageIO.read(new File(a[0]));
        final int x = Integer.parseInt(a[1]), y = Integer.parseInt(a[2]);
        System.out.println("size " + img.getWidth() + "x" + img.getHeight());
        StringBuilder sb = new StringBuilder("column x=" + x + ": ");
        for (int r = 0; r < img.getHeight(); r++) { final String k = kind(img.getRGB(x, r)); if (!k.isEmpty()) sb.append(r).append(k).append(' '); }
        System.out.println(sb);
        sb = new StringBuilder("row y=" + y + ": ");
        for (int c = 0; c < img.getWidth(); c++) { final String k = kind(img.getRGB(c, y)); if (!k.isEmpty()) sb.append(c).append(k).append(' '); }
        System.out.println(sb);
    }
    static String kind(int p) {
        final int R = (p >> 16) & 255, G = (p >> 8) & 255, B = p & 255;
        if (G > 200 && B > 200 && R < 120) return "c";      // cyan stroke
        if (R > 180 && B > 180 && G < 90) return "m";       // magenta stroke
        if (G > 170 && R < 110 && B < 150) return "g";      // green centroid line
        return "";
    }
}
