package harness;

import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.List;
import java.util.Locale;

/**
 * One-line summary per closed-loop CSV: completed runs and averaged metrics.
 * Usage: Summary <label=file.csv> ...
 */
public final class Summary {
    public static void main(String[] a) throws Exception {
        Locale.setDefault(Locale.ROOT);
        System.out.printf("%-26s %5s %5s %9s %9s %9s %9s %9s %8s %8s%n", "variant", "runs", "done", "meanLat", "worstLat", "rmsHead", "signChg/s", "steerStd", "lost%", "wrong%");
        for (String arg : a) {
            final String label = arg.contains("=") ? arg.substring(0, arg.indexOf('=')) : arg;
            final String file = arg.contains("=") ? arg.substring(arg.indexOf('=') + 1) : arg;
            final List<String> lines = Files.readAllLines(Paths.get(file));
            final String[] h = lines.get(0).split(",", -1);
            int n = 0, done = 0;
            double lat = 0, worst = 0, head = 0, sc = 0, sd = 0, lost = 0, wrong = 0;
            for (int i = 1; i < lines.size(); i++) {
                final String[] c = lines.get(i).split(",", -1);
                if (c.length < h.length) continue;
                n++;
                if (get(h, c, "result").equals("DONE")) done++;
                lat += d(h, c, "meanLat_cm"); worst = Math.max(worst, d(h, c, "maxLat_cm"));
                head += d(h, c, "rmsHead_deg"); sc += d(h, c, "signChg_per_s"); sd += d(h, c, "steerStd");
                lost += d(h, c, "lost_pct"); wrong += d(h, c, "wrongLock_pct");
            }
            System.out.printf("%-26s %5d %5d %9.2f %9.2f %9.2f %9.2f %9.2f %8.2f %8.2f%n", label, n, done, lat / n, worst, head / n, sc / n, sd / n, lost / n, wrong / n);
        }
    }
    static String get(String[] h, String[] c, String k) { for (int i = 0; i < h.length; i++) if (h[i].equals(k)) return c[i]; return ""; }
    static double d(String[] h, String[] c, String k) { try { return Double.parseDouble(get(h, c, k)); } catch (Exception e) { return 0; } }
}
