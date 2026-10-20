import com.example.linefollower.LineFollowerPipeline;
public class LostProbe {
    static final int W = 320, H = 240;
    static byte[] frame(int cx, int half) {           // white vertical line on dark floor
        byte[] g = new byte[W * H];
        for (int r = 0; r < H; r++) for (int c = 0; c < W; c++)
            g[r * W + c] = (byte) ((cx >= 0 && Math.abs(c - cx) <= half) ? 220 : 40);
        return g;
    }
    public static void main(String[] a) {
        LineFollowerPipeline p = new LineFollowerPipeline();
        final LineFollowerPipeline.FrameData[] last = new LineFollowerPipeline.FrameData[1];
        p.setFrameCallback(d -> last[0] = d);
        int n = 0; while (!p.calibrate(frame(160, 10), W, H)) n++;
        System.out.println("calibrated after " + (n + 1) + " frames, phase=" + last[0].calibPhase);
        String hdr = "frame  S   T  found held";
        System.out.println("-- line hidden mid-image (blank frames)"); System.out.println(hdr);
        p.processFrame(frame(170, 10), W, H); print("ok", p, last[0]);
        for (int i = 0; i < 12; i++) { p.processFrame(frame(-1, 0), W, H); print("blank" + i, p, last[0]); }
        LineFollowerPipeline q = new LineFollowerPipeline();
        q.setFrameCallback(d -> last[0] = d);
        while (!q.calibrate(frame(160, 10), W, H)) { }
        System.out.println("-- line leaves at right edge"); System.out.println(hdr);
        int[] xs = {180, 220, 260, 290, 310, 318};
        for (int x : xs) { q.processFrame(frame(x, 10), W, H); print("x=" + x, q, last[0]); }
        for (int i = 0; i < 12; i++) { q.processFrame(frame(-1, 0), W, H); print("gone" + i, q, last[0]); }
    }
    static void print(String t, LineFollowerPipeline p, LineFollowerPipeline.FrameData d) {
        System.out.printf("%-7s %3d %3d %-5s %-5s off=%.1f head=%.1f conf=%.2f%n", t, d.steering, d.tracking, d.trackFound, d.lineHeld, d.steerOffsetTerm, d.steerHeadingTerm, d.headingConfidence);
    }
}
