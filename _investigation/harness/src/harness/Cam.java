package harness;

/**
 * Pinhole camera fixed on the robot, tilted down by {@code tiltDeg}.
 *
 * Robot frame: fwd = forward from the axle centre, left = to the left, up = up (metres).
 * Camera sits at (camFwd, -camLat, h): camLat > 0 = mounted RIGHT of the centre line.
 * Image: col 0 = left, row 0 = top; pixel centres at +0.5.
 *
 * For every pixel the floor point it sees (in the robot frame) is constant, so it is
 * precomputed once; rendering is then a pose transform + map lookup per pixel.
 */
final class Cam {
    final int W, H;
    final double f, cx, cy, h, tilt, camFwd, camLat;
    final double st, ct;
    /** Floor point (robot frame) seen by each luma pixel; NaN above the horizon. */
    final float[] fwd, left;

    Cam(int W, int H, double vfovDeg, double tiltDeg, double h, double camFwd, double camLat) {
        this.W = W; this.H = H;
        this.f = (H / 2.0) / Math.tan(Math.toRadians(vfovDeg / 2));
        this.cx = W / 2.0; this.cy = H / 2.0;
        this.h = h; this.tilt = Math.toRadians(tiltDeg);
        this.camFwd = camFwd; this.camLat = camLat;
        st = Math.sin(tilt); ct = Math.cos(tilt);
        fwd = new float[W * H]; left = new float[W * H];
        for (int r = 0; r < H; r++) {
            for (int c = 0; c < W; c++) {
                final double[] p = floorAt(c + 0.5, r + 0.5);
                final int i = r * W + c;
                if (p == null) { fwd[i] = Float.NaN; left[i] = Float.NaN; }
                else { fwd[i] = (float) p[0]; left[i] = (float) p[1]; }
            }
        }
    }

    /** Floor point {fwd, left} (robot frame) seen at image position (col, row); null above horizon. */
    double[] floorAt(double col, double row) {
        final double xn = (col - cx) / f, yn = (row - cy) / f;
        final double down = yn * ct + st;
        if (down <= 1e-6) return null;
        final double s = h / down;
        return new double[]{camFwd + s * (ct - yn * st), -camLat - s * xn};
    }

    /** Image {col, row, depth} of robot-frame point (fwd, left, up); null if behind the camera. */
    double[] project(double pf, double pl, double pu) {
        final double df = pf - camFwd, dl = pl + camLat, du = pu - h;
        final double xc = -dl;
        final double yc = -st * df - ct * du;
        final double zc =  ct * df - st * du;
        if (zc <= 1e-4) return null;
        return new double[]{cx + f * xc / zc, cy + f * yc / zc, zc};
    }
}
