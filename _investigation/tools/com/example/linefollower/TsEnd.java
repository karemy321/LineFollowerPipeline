package com.example.linefollower;
public class TsEnd {
    public static void main(String[] a) {
        final int W = 320, H = 240;
        final TrackSim sim = new TrackSim(W, H);
        sim.floorRgb = 0x1E3CC8; sim.lineRgb = 0xEBEBEB;
        sim.tracks.add(new TrackSim.Path(0, 0, 0).straight(0.8).points());
        final LineFollowerPipeline p = new LineFollowerPipeline();
        p.configure(TrackSim.appConfig());
        final LineFollowerPipeline.FrameData[] last = {null};
        p.setFrameCallback(d -> last[0] = d);
        while (true) { final byte[][] y = sim.toYuv420(sim.render(0.02, 0, 0)); if (p.calibrateYuv420(y[0], y[1], y[2], W, H)) break; }
        for (double x = 0.10; x <= 0.85; x += 0.02) {
            final byte[][] y = sim.toYuv420(sim.render(x, 0, 0));
            final int[] o = p.processFrameYuv420(y[0], y[1], y[2], W, H);
            final LineFollowerPipeline.FrameData d = last[0];
            if (x > 0.55) System.out.printf("x=%.2f T=%d S=%d found=%b held=%b len=%.2f pts=%d fresh=%d conf=%.2f mot=%.3f%n", x, o[1], o[0], d.trackFound, d.lineHeld, d.pathLength, d.pathPointCount, d.pathFreshPoints, d.pathConfidence, d.motionFwd);
        }
    }
}
