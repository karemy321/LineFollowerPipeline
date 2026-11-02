package com.example.linefollower;
public class TsSteer {
    public static void main(String[] a) {
        final int W = 320, H = 240;
        final TrackSim sim = new TrackSim(W, H);
        sim.floorRgb = Integer.parseInt(a[0], 16); sim.lineRgb = Integer.parseInt(a[1], 16);
        if (a.length > 2 && a[2].equals("shadow")) { sim.shadowEdgeFrac = 0.4; sim.shadowGain = 0.45; }
        sim.tracks.add(new TrackSim.Path(0, 0, 0).straight(1.0).points());
        final LineFollowerPipeline p = new LineFollowerPipeline();
        p.configure(TrackSim.appConfig());
        final LineFollowerPipeline.FrameData[] last = {null};
        p.setFrameCallback(d -> last[0] = d);
        int f = 0;
        while (true) { final byte[][] y = sim.toYuv420(sim.render(0.05, 0, 0)); if (p.calibrateYuv420(y[0], y[1], y[2], W, H)) break; if (++f > 60) { System.out.println("NOCAL"); return; } }
        for (double lat : new double[]{0, 0.02, -0.02, 0}) {
            final byte[][] y = sim.toYuv420(sim.render(0.05, lat, 0));
            final int[] o = p.processFrameYuv420(y[0], y[1], y[2], W, H);
            final LineFollowerPipeline.FrameData d = last[0];
            System.out.printf("lat=%.2f S=%d T=%d found=%b held=%b fresh=%d off=%.3f tgt=(%.2f,%.2f) cells=%d wf=%.3f roi=%.2f-%.2f%n", lat, o[0], o[1], d.trackFound, d.lineHeld, d.pathFreshPoints, d.pathOffset, d.lookAheadFwd, d.lookAheadLat, d.unreliableCells, d.lineWidthFloor, d.roiTopFrac, d.roiBotFrac);
        }
    }
}
