package com.example.linefollower;

import java.util.function.Consumer;

/** Manual experiment runner (not a JUnit test): prints pipeline behaviour on simulated scenes. */
public class SimExperiment {

    static final int W = 320, H = 240;
    static final int BROWN = 0x8C5F3C, DARK_BROWN = 0x6E4A2E, GREY_BROWN = 0x6E6560,
                     BLACK = 0x1E1C1A, WHITE = 0xEBEBEB, BLUE = 0x1E3CC8, NIGHT = 0x202020;

    public static void main(String[] args) {
        if (args.length > 0 && args[0].equals("tracking")) { tracking(); return; }
        colours();
        turns();
        tracking();
    }

    // ── 3. Tracking value approaching a dead end / a sharp turn ────────────

    static void tracking() {
        System.out.println("=== tracking (steer/track per 2 cm) ===");
        trackingRun("dead end (straight 0.8 m, robot drives to the end)", 0, 0, 0.8);
        trackingRun("90deg sharp corner R=0.03", 0.03, 90, 0.5);
        trackingRun("90deg curve R=0.25", 0.25, 90, 0.5);
        trackingRun("straight with grain+vignette (should stay 100)", 0, 0, 2.0);
    }

    static void trackingRun(String name, double r, double angle, double straightLen) {
        final TrackSim sim = new TrackSim(W, H);
        sim.floorRgb = BLUE; sim.lineRgb = WHITE;
        if (name.contains("grain")) { sim.grain = 0.12; sim.vignette = 0.4; }
        final TrackSim.Path path = new TrackSim.Path(0, 0, 0).straight(straightLen);
        if (angle != 0) path.arc(r, angle).straight(0.6);
        sim.tracks.add(path.points());
        final LineFollowerPipeline p = new LineFollowerPipeline();
        p.configure(TrackSim.appConfig());
        final double[] start = path.pose(4);
        while (true) {
            final byte[][] yuv = sim.toYuv420(sim.render(start[0], start[1], start[2]));
            if (p.calibrateYuv420(yuv[0], yuv[1], yuv[2], W, H)) break;
        }
        final StringBuilder sb = new StringBuilder();
        final int end = (int) Math.round(straightLen / 0.005);
        for (int i = 4; i < end + 20 && i < path.size(); i += 4) {
            if (i < end - 140) continue;    // start ~70 cm before the end / corner
            final double[] pose = path.pose(Math.min(i, end));
            final double x = i <= end ? pose[0] : pose[0] + (i - end) * 0.005 * Math.cos(pose[2]);
            final double y = i <= end ? pose[1] : pose[1] + (i - end) * 0.005 * Math.sin(pose[2]);
            final byte[][] yuv = sim.toYuv420(sim.render(x, y, pose[2]));
            final int[] out = p.processFrameYuv420(yuv[0], yuv[1], yuv[2], W, H);
            sb.append(out[1]).append(' ');
        }
        System.out.println(name + "\n   " + sb);
    }

    // ── 1. Colour learning on realistic floors ─────────────────────────────

    static void colours() {
        System.out.println("=== colour learning ===");
        final int[][] pairs = {{BROWN, BLACK}, {DARK_BROWN, BLACK}, {GREY_BROWN, BLACK}, {BROWN, WHITE},
                               {BLUE, BLACK}, {BLUE, WHITE}, {NIGHT, WHITE}};
        final String[] names = {"brown/black", "darkbrown/black", "greybrown/black", "brown/white",
                                "blue/black", "blue/white", "black/white"};
        final String[] condNames = {"plain", "grain+vig", "heavy grain+vig", "shadow 40%", "shadow+grain+vig"};
        @SuppressWarnings("unchecked")
        final Consumer<TrackSim>[] conds = new Consumer[]{
            (Consumer<TrackSim>) s -> { },
            (Consumer<TrackSim>) s -> { s.grain = 0.12; s.vignette = 0.4; },
            (Consumer<TrackSim>) s -> { s.grain = 0.2;  s.vignette = 0.6; },
            (Consumer<TrackSim>) s -> { s.shadowEdgeFrac = 0.4; s.shadowGain = 0.45; },
            (Consumer<TrackSim>) s -> { s.shadowEdgeFrac = 0.4; s.shadowGain = 0.45; s.grain = 0.12; s.vignette = 0.4; },
        };
        for (int p = 0; p < pairs.length; p++) {
            for (int c = 0; c < conds.length; c++) {
                System.out.printf("%-16s %-17s: %s%n", names[p], condNames[c],
                        colourRun(pairs[p][0], pairs[p][1], conds[c]));
            }
        }
    }

    static String colourRun(int floor, int line, Consumer<TrackSim> cond) {
        final TrackSim sim = new TrackSim(W, H);
        sim.floorRgb = floor; sim.lineRgb = line;
        cond.accept(sim);
        sim.tracks.add(new TrackSim.Path(0, 0, 0).straight(1.0).points());
        final LineFollowerPipeline p = new LineFollowerPipeline();
        p.configure(TrackSim.appConfig());
        final int[] issue = {0};
        p.setFrameCallback(d -> issue[0] = d.calibIssue);
        int frames = 0;
        boolean done = false;
        while (!done && frames < 60) {
            final byte[][] yuv = sim.toYuv420(sim.render(0.05, 0, 0));
            done = p.calibrateYuv420(yuv[0], yuv[1], yuv[2], W, H);
            frames++;
        }
        if (!done) return "FAILED issue=" + issue[0];
        // steering when centred, and with the robot 2 cm left of the line (line appears right)
        final byte[][] c = sim.toYuv420(sim.render(0.05, 0, 0));
        final int s0 = p.processFrameYuv420(c[0], c[1], c[2], W, H)[0];
        final byte[][] r = sim.toYuv420(sim.render(0.05, 0.02, 0));
        final int s1 = p.processFrameYuv420(r[0], r[1], r[2], W, H)[0];
        final byte[][] l = sim.toYuv420(sim.render(0.05, -0.02, 0));
        final int s2 = p.processFrameYuv420(l[0], l[1], l[2], W, H)[0];
        return String.format("ok in %2d frames line=%06X floor=%06X steer centred=%d +2cm=%d -2cm=%d",
                frames, p.getLineColorArgb() & 0xFFFFFF, p.getFloorColorArgb() & 0xFFFFFF, s0, s1, s2);
    }

    // ── 2. Steering through and after a turn ───────────────────────────────

    static void turns() {
        System.out.println("=== turns ===");
        turnRun("ideal follow, 90deg R=0.25", 0.25, 90, 0, false, 0);
        turnRun("cut corner 4cm, 90deg R=0.12", 0.12, 90, 0.04, false, 0);
        turnRun("U-turn R=0.10 (parallel track visible)", 0.10, 180, 0.03, false, 0);
        turnRun("overshoot (line lost), 90deg R=0.12", 0.12, 90, 0.04, true, 0);
        turnRun("overshoot + wide distractor 9cm right of exit", 0.12, 90, 0.04, true, 0.09);
        turnRun("overshoot + wide distractor 9cm left of exit", 0.12, 90, 0.04, true, -0.09);
        turnRun("follow + wide distractor 9cm right of exit", 0.12, 90, 0.0, false, 0.09);
    }

    /**
     * Robot drives a straight → arc → straight track. Its path deviates from the
     * centre line by `cut` metres around the arc (smooth bump) and is exactly on
     * the line again on the second straight. With `overshoot`, the robot also
     * lags its heading so the line leaves view. `distractor` ≠ 0 adds a wide
     * parallel tape that far to the right of the exit straight.
     */
    static void turnRun(String name, double r, double angle, double cut, boolean overshoot,
                        double distractor) {
        final TrackSim sim = new TrackSim(W, H);
        sim.floorRgb = BLUE; sim.lineRgb = WHITE;
        final TrackSim.Path path = new TrackSim.Path(0, 0, 0).straight(0.5).arc(r, angle).straight(0.8);
        sim.tracks.add(path.points());
        final int arcStart = (int) Math.round(0.5 / 0.005);
        final int arcLen   = (int) Math.round(Math.abs(Math.toRadians(angle)) * r / 0.005);
        if (distractor != 0) {
            final double[] e = path.pose(arcStart + arcLen + 20);
            final double ox = e[0] + distractor * Math.sin(e[2]), oy = e[1] - distractor * Math.cos(e[2]);
            sim.wideTracks.add(new TrackSim.Path(ox, oy, e[2]).straight(0.6).points());
        }

        final LineFollowerPipeline p = new LineFollowerPipeline();
        p.configure(TrackSim.appConfig());
        final boolean[] found = {false};
        p.setFrameCallback(d -> found[0] = d.trackFound);
        final double[] start = path.pose(10);
        while (true) {
            final byte[][] yuv = sim.toYuv420(sim.render(start[0], start[1], start[2]));
            if (p.calibrateYuv420(yuv[0], yuv[1], yuv[2], W, H)) break;
        }

        final StringBuilder sb = new StringBuilder();
        final int step = 4;   // 2 cm per frame
        final int bumpStart = arcStart - arcLen / 2, bumpEnd = arcStart + arcLen + arcLen / 2;
        for (int i = 10; i < path.size() - 100; i += step) {
            if (i < arcStart - 40) continue;   // skip the boring first straight
            final double[] pose = path.pose(i);
            double lat = 0, dh = 0;
            if (i > bumpStart && i < bumpEnd) {
                final double t = (i - bumpStart) / (double) (bumpEnd - bumpStart);
                lat = cut * Math.sin(Math.PI * t) * Math.signum(angle);   // toward inside of the turn
                if (overshoot) dh = -Math.toRadians(35) * Math.sin(Math.PI * t) * Math.signum(angle);
            }
            final double x = pose[0] - lat * Math.sin(pose[2]);
            final double y = pose[1] + lat * Math.cos(pose[2]);
            final byte[][] yuv = sim.toYuv420(sim.render(x, y, pose[2] + dh));
            final int[] out = p.processFrameYuv420(yuv[0], yuv[1], yuv[2], W, H);
            final String tag = i < arcStart ? "S1" : i < arcStart + arcLen ? "ARC" : "S2";
            sb.append(String.format("%s:%d%s ", tag, out[0], found[0] ? "" : "!"));
        }
        System.out.println(name + "\n   " + sb);
    }
}
