package harness;

import java.io.PrintWriter;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.function.Predicate;

/**
 * Builds the markdown tables of CHANGES_v2.md from the result CSVs (orig vs new).
 * Usage: Report <results-dir> <out.md>
 */
public final class Report {
    static String dir;
    static PrintWriter out;

    static final class Rows {
        final String[] h; final List<String[]> r = new ArrayList<>();
        Rows(String file) throws Exception {
            final List<String> l = Files.readAllLines(Paths.get(file));
            h = l.get(0).split(",", -1);
            for (int i = 1; i < l.size(); i++) { final String[] c = l.get(i).split(",", -1); if (c.length >= h.length) r.add(c); }
        }
        int col(String k) { for (int i = 0; i < h.length; i++) if (h[i].equals(k)) return i; throw new IllegalArgumentException(k); }
        String s(String[] row, String k) { return row[col(k)]; }
        double d(String[] row, String k) { try { return Double.parseDouble(row[col(k)]); } catch (Exception e) { return 0; } }
    }

    /** {runs, done, meanLat, worstLat, rmsHead, signChg, steerStd, lost%, wrong%, settle} for rows matching p. */
    static double[] agg(Rows R, Predicate<String[]> p) {
        final double[] a = new double[10];
        for (String[] row : R.r) {
            if (!p.test(row)) continue;
            a[0]++;
            if (R.s(row, "result").equals("DONE")) a[1]++;
            a[2] += R.d(row, "meanLat_cm"); a[3] = Math.max(a[3], R.d(row, "maxLat_cm"));
            a[4] += R.d(row, "rmsHead_deg"); a[5] += R.d(row, "signChg_per_s"); a[6] += R.d(row, "steerStd");
            a[7] += R.d(row, "lost_pct"); a[8] += R.d(row, "wrongLock_pct"); a[9] += R.d(row, "settle_s");
        }
        for (int i : new int[]{2, 4, 5, 6, 7, 8, 9}) a[i] /= Math.max(1, a[0]);
        return a;
    }

    static String pair(double o, double n, String fmt) { return String.format(Locale.ROOT, fmt + " → " + fmt, o, n); }

    static void closedSummary() throws Exception {
        out.println("### Closed-loop summary per regime (all tracks × both polarities × with/without distractors × all gains)\n");
        out.println("| regime | runs | completed orig → new | mean lateral err cm | worst lateral err cm | RMS heading err ° | steering sign changes /s | lost % of frames | wrong-lock % of frames |");
        out.println("|---|---:|---|---|---|---|---|---|---|");
        for (String reg : new String[]{"ideal", "real", "stress", "tilt35"}) {
            final Rows o = new Rows(dir + "/closed/" + reg + "_orig.csv"), n = new Rows(dir + "/closed/" + reg + "_new.csv");
            final double[] a = agg(o, r -> true), b = agg(n, r -> true);
            out.printf(Locale.ROOT, "| %s | %.0f | %.0f → %.0f | %s | %s | %s | %s | %s | %s |%n", reg, a[0], a[1], b[1],
                    pair(a[2], b[2], "%.2f"), pair(a[3], b[3], "%.1f"), pair(a[4], b[4], "%.1f"), pair(a[5], b[5], "%.2f"),
                    pair(a[7], b[7], "%.1f"), pair(a[8], b[8], "%.2f"));
        }
        out.println();
    }

    static void closedByTrack(String reg) throws Exception {
        final Rows o = new Rows(dir + "/closed/" + reg + "_orig.csv"), n = new Rows(dir + "/closed/" + reg + "_new.csv");
        out.println("#### Regime `" + reg + "` by track and distractors (all gains, both polarities)\n");
        out.println("| track | distractors | completed orig → new | mean lateral err cm | worst lateral err cm | sign changes /s | wrong-lock % |");
        out.println("|---|---|---|---|---|---|---|");
        for (String t : Scenes.TRACKS) for (String d : new String[]{"0", "1"}) {
            final Predicate<String[]> p = r -> o.s(r, "track").equals(t) && o.s(r, "distract").equals(d);
            final double[] a = agg(o, p), b = agg(n, p);
            out.printf(Locale.ROOT, "| %s | %s | %.0f/%.0f → %.0f/%.0f | %s | %s | %s | %s |%n", t, d.equals("1") ? "yes" : "no",
                    a[1], a[0], b[1], b[0], pair(a[2], b[2], "%.2f"), pair(a[3], b[3], "%.1f"), pair(a[5], b[5], "%.2f"), pair(a[8], b[8], "%.2f"));
        }
        out.println();
    }

    static void closedByGain(String reg) throws Exception {
        final Rows o = new Rows(dir + "/closed/" + reg + "_orig.csv"), n = new Rows(dir + "/closed/" + reg + "_new.csv");
        out.println("#### Regime `" + reg + "` by steering gain c and power mode x (all tracks, both polarities, with/without distractors)\n");
        out.println("| c | x | completed orig → new | mean lateral err cm | worst lateral err cm | sign changes /s | steering std |");
        out.println("|---|---|---|---|---|---|---|");
        for (String c : new String[]{"0.30", "0.50", "1.00", "2.00"}) for (String x : new String[]{"0", "1"}) {
            final Predicate<String[]> p = r -> o.s(r, "c").equals(c) && o.s(r, "x").equals(x);
            final double[] a = agg(o, p), b = agg(n, p);
            if (a[0] == 0) continue;
            out.printf(Locale.ROOT, "| %s | %s | %.0f/%.0f → %.0f/%.0f | %s | %s | %s | %s |%n", c, x, a[1], a[0], b[1], b[0],
                    pair(a[2], b[2], "%.2f"), pair(a[3], b[3], "%.1f"), pair(a[5], b[5], "%.2f"), pair(a[6], b[6], "%.1f"));
        }
        out.println();
    }

    static void lecturer(String reg) throws Exception {
        final Rows o = new Rows(dir + "/closed/" + reg + "_orig.csv"), n = new Rows(dir + "/closed/" + reg + "_new.csv");
        out.println("#### Lecturer's program (c = 0.5), regime `" + reg + "` — every run\n");
        out.println("| track | floor | distractors | x | result orig → new | mean lat cm | max lat cm | max heading err ° | sign changes /s | lost % | wrong-lock % |");
        out.println("|---|---|---|---|---|---|---|---|---|---|---|");
        for (String[] r : o.r) {
            if (!o.s(r, "c").equals("0.50")) continue;
            String[] m = null;
            for (String[] q : n.r) if (n.s(q, "track").equals(o.s(r, "track")) && n.s(q, "scheme").equals(o.s(r, "scheme"))
                    && n.s(q, "distract").equals(o.s(r, "distract")) && n.s(q, "c").equals("0.50") && n.s(q, "x").equals(o.s(r, "x"))) m = q;
            if (m == null) continue;
            out.printf(Locale.ROOT, "| %s | %s | %s | %s | %s → %s | %s | %s | %s | %s | %s | %s |%n", o.s(r, "track"), o.s(r, "scheme"),
                    o.s(r, "distract").equals("1") ? "yes" : "no", o.s(r, "x"), o.s(r, "result"), n.s(m, "result"),
                    pair(o.d(r, "meanLat_cm"), n.d(m, "meanLat_cm"), "%.2f"), pair(o.d(r, "maxLat_cm"), n.d(m, "maxLat_cm"), "%.1f"),
                    pair(o.d(r, "maxHead_deg"), n.d(m, "maxHead_deg"), "%.0f"), pair(o.d(r, "signChg_per_s"), n.d(m, "signChg_per_s"), "%.2f"),
                    pair(o.d(r, "lost_pct"), n.d(m, "lost_pct"), "%.1f"), pair(o.d(r, "wrongLock_pct"), n.d(m, "wrongLock_pct"), "%.1f"));
        }
        out.println();
    }

    static void recovery() throws Exception {
        out.println("#### Heading-error recovery (straight track, robot turned after calibration; realistic timing; both polarities)\n");
        out.println("| start yaw | c | x | max lateral err cm orig → new | settle time s (|lat| < 1 cm and heading < 3°) orig → new | sign changes /s |");
        out.println("|---|---|---|---|---|---|");
        for (String y : new String[]{"15", "-15", "30"}) {
            final Rows o = new Rows(dir + "/closed/yaw" + y + "_orig.csv"), n = new Rows(dir + "/closed/yaw" + y + "_new.csv");
            for (String c : new String[]{"0.30", "0.50", "1.00", "2.00"}) for (String x : new String[]{"0", "1"}) {
                final Predicate<String[]> p = r -> o.s(r, "c").equals(c) && o.s(r, "x").equals(x);
                final double[] a = agg(o, p), b = agg(n, p);
                out.printf(Locale.ROOT, "| %s° | %s | %s | %s | %s | %s |%n", y, c, x, pair(a[3], b[3], "%.2f"), pair(a[9], b[9], "%.2f"), pair(a[5], b[5], "%.2f"));
            }
        }
        out.println();
    }

    static void heading() throws Exception {
        final Rows o = new Rows(dir + "/heading_orig.csv"), n = new Rows(dir + "/heading_new.csv");
        out.println("| floor | line crosses strip at | robot heading vs line | steering orig | steering new | new: offset part | new: heading part | new: measured heading ° |");
        out.println("|---|---|---|---:|---:|---:|---:|---:|");
        for (int i = 0; i < o.r.size(); i++) {
            final String[] a = o.r.get(i), b = n.r.get(i);
            final double off = n.d(b, "offsetTerm");
            out.printf(Locale.ROOT, "| %s | %s | %s° | %.0f | %.0f | %.1f | %.1f | %.1f |%n", o.s(a, "scheme"),
                    o.s(a, "crossOffsetPx").equals("0") ? "calibrated column" : "+" + o.s(a, "crossOffsetPx") + " px",
                    o.s(a, "angleDeg"), o.d(a, "steering"), n.d(b, "steering"), off, n.d(b, "steering") - off, n.d(b, "headingDeg"));
        }
        out.println();
    }

    static void distract() throws Exception {
        final Rows o = new Rows(dir + "/distract_orig.csv"), n = new Rows(dir + "/distract_new.csv");
        out.println("| floor | scenario | frames | max steering error orig → new | mean steering error | max tracking error | wrong-lock frames | line-lost frames | max centroid jump px |");
        out.println("|---|---|---:|---|---|---|---|---|---|");
        for (int i = 0; i < o.r.size(); i++) {
            final String[] a = o.r.get(i), b = n.r.get(i);
            out.printf(Locale.ROOT, "| %s | %s | %s | %s | %s | %s | %s | %s | %s |%n", o.s(a, "scheme"), o.s(a, "case"), o.s(a, "frames"),
                    pair(o.d(a, "maxDeltaSteer"), n.d(b, "maxDeltaSteer"), "%.0f"), pair(o.d(a, "meanDeltaSteer"), n.d(b, "meanDeltaSteer"), "%.1f"),
                    pair(o.d(a, "maxDeltaTrack"), n.d(b, "maxDeltaTrack"), "%.0f"), pair(o.d(a, "wrongLockFrames"), n.d(b, "wrongLockFrames"), "%.0f"),
                    pair(o.d(a, "lostFrames"), n.d(b, "lostFrames"), "%.0f"), pair(o.d(a, "maxCentroidJumpPx"), n.d(b, "maxCentroidJumpPx"), "%.0f"));
        }
        out.println();
    }

    static void regression() throws Exception {
        final Rows o = new Rows(dir + "/regression_orig.csv"), n = new Rows(dir + "/regression_new.csv");
        out.println("| test | orig | new | values orig | values new |");
        out.println("|---|---|---|---|---|");
        for (int i = 0; i < o.r.size(); i++) {
            final String[] a = o.r.get(i), b = n.r.get(i);
            out.printf("| %s | %s | %s | %s | %s |%n", o.s(a, "test"), o.s(a, "result"), n.s(b, "result"), o.s(a, "values"), n.s(b, "values"));
        }
        out.println();
    }

    static void real() throws Exception {
        final Rows o = new Rows(dir + "/real_orig.csv"), n = new Rows(dir + "/real_new.csv");
        out.println("| calibrated on | test image | calibrated | tape centre (annotated) | mean centroid orig → new | px outside tape | wrong-lock frames (of 50) | mean steering | mean tracking |");
        out.println("|---|---|---|---|---|---|---|---|---|");
        for (int i = 0; i < o.r.size(); i++) {
            final String[] a = o.r.get(i), b = n.r.get(i);
            out.printf(Locale.ROOT, "| %s | %s | %s | %s | %s | %s | %s | %s | %s |%n", o.s(a, "calibratedOn").replace("WhatsApp Image 2026-10-16 at ", ""),
                    o.s(a, "testImage").replace("WhatsApp Image 2026-10-16 at ", ""), o.s(a, "calibrated"), o.s(a, "tapeCentre"),
                    pair(o.d(a, "meanCentroid"), n.d(b, "meanCentroid"), "%.0f"), pair(o.d(a, "maxPxOutsideTape"), n.d(b, "maxPxOutsideTape"), "%.0f"),
                    pair(o.d(a, "wrongLockFrames"), n.d(b, "wrongLockFrames"), "%.0f"), pair(o.d(a, "meanSteer"), n.d(b, "meanSteer"), "%.0f"),
                    pair(o.d(a, "meanTrack"), n.d(b, "meanTrack"), "%.0f"));
        }
        out.println();
    }

    public static void main(String[] a) throws Exception {
        Locale.setDefault(Locale.ROOT);
        dir = a[0];
        try (PrintWriter pw = new PrintWriter(a[1], "UTF-8")) {
            out = pw;
            final Map<String, Runnable> parts = new LinkedHashMap<>();
            out.println("<!-- generated by harness.Report -->\n## T-closed\n"); closedSummary();
            for (String r : new String[]{"real", "ideal"}) { closedByTrack(r); closedByGain(r); }
            closedByGain("stress"); closedByGain("tilt35");
            lecturer("real"); lecturer("ideal");
            recovery();
            out.println("## T-heading\n"); heading();
            out.println("## T-distract\n"); distract();
            out.println("## T-regression\n"); regression();
            out.println("## T-real\n"); real();
        }
    }
}
