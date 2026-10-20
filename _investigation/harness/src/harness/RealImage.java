package harness;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.PrintWriter;
import java.util.Locale;
import java.util.Random;
import javax.imageio.ImageIO;

/**
 * Real-image test from the lecturer's screenshots (black tape on a wood table, black
 * chair and floor tiles beyond the table edge).
 *
 * Screenshot → camera frame (960×1280, the app's analysis size): the overlay draws the
 * frame FILL_CENTER-scaled into the preview; from the drawn strip (rows 936–960) and
 * V-ROI top (row 296) the mapping is  screen = (60.8 + col·0.833, 116.8 + row·0.833)
 * in the 922×2048 screenshot. Overlay removal: translucent cyan (strip, α=45/255) and
 * magenta (V-ROI, α=35/255) fills are inverted; opaque strokes, the green centroid
 * line and white labels are in-painted from neighbouring pixels.
 * Y = 0.299R + 0.587G + 0.114B, U/V (BT.601) for the YUV input the app uses.
 *
 * Usage: RealImage <out.csv> <png-dir> <calib.jpeg> <test.jpeg>...
 */
public final class RealImage {
    static final int W = 960, H = 1280;
    static final double S = 0.8330, OX = 60.8, OY = 116.8;

    static final class Frame { byte[] y, u, v; int[] rgb; }

    static Frame load(String path) throws Exception {
        final BufferedImage img = ImageIO.read(new File(path));
        final int[] rgb = new int[W * H];
        final boolean[] mask = new boolean[W * H];
        // per-frame-row magenta polygon extent (V-ROI box) for fill inversion
        for (int r = 0; r < H; r++) {
            int mL = -1, mR = -1;
            for (int c = 0; c < W; c++) {
                final int p = sample(img, OX + (c + 0.5) * S, OY + (r + 0.5) * S);
                rgb[r * W + c] = p;
                final int R = (p >> 16) & 255, G = (p >> 8) & 255, B = p & 255;
                final boolean magenta = R > 150 && B > 150 && R - G > 70 && B - G > 70;
                final boolean cyan    = G > 170 && B > 170 && G - R > 90;
                final boolean green   = G > 150 && G - R > 70 && G - B > 40;
                final boolean white   = R > 228 && G > 228 && B > 228;
                if (magenta || cyan || green || white) mask[r * W + c] = true;
                if (magenta) { if (mL < 0) mL = c; mR = c; }
            }
            // invert the translucent magenta fill between the box edges (V-ROI rows only)
            if (mL >= 0 && mR - mL > 20 && r >= 280 && r < 936)
                for (int c = mL; c <= mR; c++) if (!mask[r * W + c]) rgb[r * W + c] = unblend(rgb[r * W + c], 0xFF00FF, 35 / 255.0);
            // invert the translucent cyan strip fill
            if (r >= 936 && r < 960)
                for (int c = 0; c < W; c++) if (!mask[r * W + c]) rgb[r * W + c] = unblend(rgb[r * W + c], 0x00FFFF, 45 / 255.0);
        }
        // strip strokes (4 screen px ≈ 5 frame rows at each edge) → mask
        for (int r = 932; r < 941; r++) for (int c = 0; c < W; c++) mask[r * W + c] = true;
        for (int r = 956; r < 965 && r < H; r++) for (int c = 0; c < W; c++) mask[r * W + c] = true;
        // grow the mask by 2 px (anti-aliased stroke edges)
        final boolean[] m2 = mask.clone();
        for (int r = 2; r < H - 2; r++) for (int c = 2; c < W - 2; c++) if (mask[r * W + c])
            for (int dr = -2; dr <= 2; dr++) for (int dc = -2; dc <= 2; dc++) m2[(r + dr) * W + c + dc] = true;
        inpaint(rgb, m2);
        final Frame f = new Frame();
        f.rgb = rgb;
        f.y = new byte[W * H];
        final int cw = W / 2, ch = H / 2;
        f.u = new byte[cw * ch]; f.v = new byte[cw * ch];
        for (int i = 0; i < W * H; i++) {
            final int p = rgb[i], R = (p >> 16) & 255, G = (p >> 8) & 255, B = p & 255;
            f.y[i] = (byte) World.Col.clamp((int) Math.round(0.299 * R + 0.587 * G + 0.114 * B));
        }
        for (int r = 0; r < ch; r++) for (int c = 0; c < cw; c++) {
            final int p = rgb[(2 * r) * W + 2 * c];
            f.u[r * cw + c] = (byte) World.Col.u(p); f.v[r * cw + c] = (byte) World.Col.v(p);
        }
        return f;
    }

    static int unblend(int p, int over, double a) {
        int out = 0;
        for (int sh = 16; sh >= 0; sh -= 8) {
            final double o = (p >> sh) & 255, c = (over >> sh) & 255;
            out |= World.Col.clamp((int) Math.round((o - a * c) / (1 - a))) << sh;
        }
        return out;
    }

    /** Fill masked pixels by linear interpolation between the nearest clean pixels on the row (else column). */
    static void inpaint(int[] rgb, boolean[] m) {
        for (int r = 0; r < H; r++) {
            int c = 0;
            while (c < W) {
                if (!m[r * W + c]) { c++; continue; }
                final int s = c; while (c < W && m[r * W + c]) c++;
                final int L = s - 1, R = c;
                for (int k = s; k < c; k++) {
                    final int p;
                    if (L >= 0 && R < W && R - L < 60) p = lerp(rgb[r * W + L], rgb[r * W + R], (k - L) / (double) (R - L));
                    else if (L >= 0 && R - L < 60) p = rgb[r * W + L];
                    else if (R < W && R - L < 60) p = rgb[r * W + R];
                    else continue;   // long horizontal run (strip stroke) → vertical pass
                    rgb[r * W + k] = p; m[r * W + k] = false;
                }
            }
        }
        for (int c = 0; c < W; c++) {
            int r = 0;
            while (r < H) {
                if (!m[r * W + c]) { r++; continue; }
                final int s = r; while (r < H && m[r * W + c]) r++;
                final int T = s - 1, B = r;
                for (int k = s; k < r; k++) {
                    final int p = (T >= 0 && B < H) ? lerp(rgb[T * W + c], rgb[B * W + c], (k - T) / (double) (B - T)) : rgb[(T >= 0 ? T : B) * W + c];
                    rgb[k * W + c] = p;
                }
            }
        }
    }

    static int lerp(int a, int b, double t) {
        int out = 0;
        for (int sh = 16; sh >= 0; sh -= 8) out |= (int) Math.round(((a >> sh) & 255) * (1 - t) + ((b >> sh) & 255) * t) << sh;
        return out;
    }

    static int sample(BufferedImage img, double x, double y) {
        final int xi = (int) Math.max(0, Math.min(img.getWidth() - 1, Math.round(x - 0.5)));
        final int yi = (int) Math.max(0, Math.min(img.getHeight() - 1, Math.round(y - 0.5)));
        return img.getRGB(xi, yi) & 0xFFFFFF;
    }

    /** Ground truth: centre of the darkest wide run in the H-ROI rows 940–955 nearest to {@code near}. */
    static double tapeCol(Frame f, double near) {
        final double[] col = new double[W];
        for (int r = 941; r < 955; r++) for (int c = 0; c < W; c++) col[c] += (f.y[r * W + c] & 255) / 14.0;
        final double[] sorted = col.clone(); java.util.Arrays.sort(sorted);
        final double thr = sorted[W / 2] - 30;   // darker than the strip median by 30 grey levels
        double best = Double.NaN, bd = 1e9;
        int s = -1;
        for (int c = 0; c <= W; c++) {
            final boolean dark = c < W && col[c] < thr;
            if (dark && s < 0) s = c;
            if (!dark && s >= 0) { if (c - s >= 50 && c - s <= 260) { final double m = (s + c - 1) / 2.0; if (Math.abs(m - near) < bd) { bd = Math.abs(m - near); best = m; } } s = -1; }
        }
        return best;
    }

    static void savePng(Frame f, String path) throws Exception {
        final BufferedImage o = new BufferedImage(W, H, BufferedImage.TYPE_INT_RGB);
        for (int i = 0; i < W * H; i++) { final int g = f.y[i] & 255; o.setRGB(i % W, i / W, (g << 16) | (g << 8) | g); }
        ImageIO.write(o, "png", new File(path));
    }

    /** Add sensor noise so repeated frames are not identical. */
    static void noisy(Frame src, byte[] y, Random rnd) {
        for (int i = 0; i < y.length; i++) y[i] = (byte) World.Col.clamp((src.y[i] & 255) + (int) Math.round(rnd.nextGaussian() * 3));
    }

    public static void main(String[] a) throws Exception {
        Locale.setDefault(Locale.ROOT);
        try (PrintWriter pw = new PrintWriter(a[0], "UTF-8")) {
            pw.println("lib,calibratedOn,testImage,calibrated,tapeCentre,meanCentroid,maxPxOutsideTape,wrongLockFrames,lostFrames,meanSteer,meanTrack,meanHeadingDeg");
            final Frame cal = load(a[2]);
            savePng(cal, a[1] + "/real_" + new File(a[2]).getName().replace(".jpeg", ".png"));
            for (int ti = 2; ti < a.length; ti++) {
                final Frame test = load(a[ti]);
                savePng(test, a[1] + "/real_" + new File(a[ti]).getName().replace(".jpeg", ".png"));
                for (int mode = 0; mode < 2; mode++) {   // 0: calibrate on the calibration image, 1: on the test image itself
                    final Frame c = mode == 0 ? cal : test;
                    final Lib lib = new Lib();
                    final Random rnd = new Random(7);
                    final byte[] y = new byte[W * H];
                    boolean ok = false;
                    for (int i = 0; i < 300 && !ok; i++) { noisy(c, y, rnd); ok = lib.p.calibrateYuv420(y, c.u, c.v, W, H); }
                    // Tape span in the strip, annotated by hand from the screenshots (frame columns).
                    final String tn = new File(a[ti]).getName();
                    final double[] span = tn.contains("(1)") ? new double[]{296, 407} : tn.contains("(2)") ? new double[]{260, 380} : new double[]{388, 443};
                    final double truth = (span[0] + span[1]) / 2;
                    double sc = 0, maxErr = 0, ss = 0, st = 0, sh = 0; int wrong = 0, lost = 0, n = 0;
                    for (int i = 0; i < 60; i++) {
                        noisy(test, y, rnd);
                        final int[] o = lib.p.processFrameYuv420(y, test.u, test.v, W, H);
                        if (i < 10) continue;   // let the lock settle after the jump
                        n++; ss += o[0]; st += o[1]; sh += Double.isNaN(lib.fd("headingDeg")) ? 0 : lib.fd("headingDeg");
                        if (!lib.trackFound) { lost++; continue; }
                        final double cx = lib.centroidXFrac * W;
                        sc += cx;
                        final double err = cx < span[0] ? span[0] - cx : (cx > span[1] ? cx - span[1] : 0);   // distance outside the tape
                        maxErr = Math.max(maxErr, err);
                        if (err > 20) wrong++;
                    }
                    final String line = String.format(Locale.ROOT, "%s,%s,%s,%b,%.0f,%.0f,%.0f,%d,%d,%.1f,%.1f,%.1f", Lib.TAG,
                            mode == 0 ? new File(a[2]).getName() : "itself", new File(a[ti]).getName(), ok, truth,
                            (n - lost) > 0 ? sc / (n - lost) : -1, maxErr, wrong, lost, ss / n, st / n, sh / n);
                    System.out.println(line);
                    pw.println(line);
                }
            }
        }
    }
}
