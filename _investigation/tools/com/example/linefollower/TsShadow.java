package com.example.linefollower;
public class TsShadow {
    public static void main(String[] a) {
        final int W = 320, H = 240;
        final TrackSim sim = new TrackSim(W, H);
        sim.floorRgb = 0x8C5F3C; sim.lineRgb = 0x1E1C1A;
        final TrackSim.Path path = new TrackSim.Path(0, 0, 0).straight(2.0);
        sim.tracks.add(path.points());
        sim.robotShadow(0.12, 0.0, 0.06, 0.03, 0.45);
        final LineFollowerPipeline p = new LineFollowerPipeline();
        final LineFollowerPipeline.Config cfg = TrackSim.appConfig();
        
        p.configure(cfg);
        final LineFollowerPipeline.FrameData[] last = {null};
        p.setFrameCallback(d -> last[0] = d);
        sim.effectsOn = false;
        while (true) { final double[] q = path.pose(10); final byte[][] y = sim.toYuv420(sim.render(q[0], q[1], q[2])); if (p.calibrateYuv420(y[0], y[1], y[2], W, H)) break; }
        sim.effectsOn = true;
        for (int i = 40; i <= 160; i += 2) {
            final double[] q = path.pose(i);
            final byte[][] y = sim.toYuv420(sim.render(q[0], q[1], q[2]));
            final int[] o = p.processFrameYuv420(y[0], y[1], y[2], W, H);
            if (i % 6 != 0) continue;
            final LineFollowerPipeline.FrameData d = last[0];
            final StringBuilder sb = new StringBuilder(String.format("i=%d S=%d T=%d found=%b cells=%d pts:", i, o[0], o[1], d.trackFound, d.unreliableCells));
            for (int k = 0; k < d.pathPointCount; k += 2) sb.append(String.format(" %.0f@%.0f%s", d.pathColFrac[k] * W, d.pathRowFrac[k] * H, d.pathSource[k] == 0 ? "" : "m"));
            System.out.println(sb);
        }
    }
}
