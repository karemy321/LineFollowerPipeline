package com.example.linefollower;
/** Debug: TrackSimTest.drive(r, angle, cut, overshoot, distractor) printed frame by frame. */
public class TsDbg {
    public static void main(String[] a) {
        final double r = Double.parseDouble(a[0]), angle = Double.parseDouble(a[1]), cut = Double.parseDouble(a[2]);
        final boolean overshoot = a[3].equals("1");
        final int W = 320, H = 240;
        final TrackSim sim = new TrackSim(W, H);
        sim.floorRgb = 0x1E3CC8; sim.lineRgb = 0xEBEBEB;
        final TrackSim.Path path = new TrackSim.Path(0, 0, 0).straight(0.5).arc(r, angle).straight(0.8);
        sim.tracks.add(path.points());
        final int arcStart = 100, arcLen = (int) Math.round(Math.abs(Math.toRadians(angle)) * r / 0.005);
        final LineFollowerPipeline p = new LineFollowerPipeline();
        p.configure(TrackSim.appConfig());
        final LineFollowerPipeline.FrameData[] last = {null};
        p.setFrameCallback(d -> last[0] = d);
        final double[] start = path.pose(10);
        while (true) { final byte[][] yuv = sim.toYuv420(sim.render(start[0], start[1], start[2])); if (p.calibrateYuv420(yuv[0], yuv[1], yuv[2], W, H)) break; }
        final int bumpStart = arcStart - arcLen / 2, bumpEnd = arcStart + arcLen + arcLen / 2;
        int f = 0;
        for (int i = arcStart - 40; i < path.size() - 100; i += 4, f++) {
            final double[] pose = path.pose(i);
            double lat = 0, dh = 0;
            if (i > bumpStart && i < bumpEnd) {
                final double t = (i - bumpStart) / (double) (bumpEnd - bumpStart);
                lat = cut * Math.sin(Math.PI * t) * Math.signum(angle);
                if (overshoot) dh = -Math.toRadians(35) * Math.sin(Math.PI * t) * Math.signum(angle);
            }
            final byte[][] yuv = sim.toYuv420(sim.render(pose[0] - lat * Math.sin(pose[2]), pose[1] + lat * Math.cos(pose[2]), pose[2] + dh));
            final int[] o = p.processFrameYuv420(yuv[0], yuv[1], yuv[2], W, H);
            final LineFollowerPipeline.FrameData d = last[0];
            System.out.printf("f=%d i=%d S=%d T=%d found=%b held=%b pts=%d fresh=%d off=%.3f head=%.1f tgt=(%.2f,%.2f) mot=(%.3f,%.2f) tilt=%.1f%n", f, i, o[0], o[1], d.trackFound, d.lineHeld, d.pathPointCount, d.pathFreshPoints, d.pathOffset, d.pathHeadingDeg, d.lookAheadFwd, d.lookAheadLat, d.motionFwd, d.motionYawDeg, d.cameraTiltUsedDeg);
        }
    }
}
