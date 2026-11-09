package com.example.linefollower;

import static org.junit.Assert.fail;

import java.io.IOException;
import java.io.PrintWriter;
import java.lang.reflect.Field;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Locale;
import java.util.function.Consumer;
import java.util.function.IntPredicate;

import org.junit.Assume;
import org.junit.Test;

/**
 * v3 scenarios (LINE_FOLLOWER_V3_PLAN.md, Phase 0): glare and distractors, rendered with
 * {@link TrackSim} while the robot drives at 0.6 m/s. Every scenario renders each frame once and
 * feeds it to a grayscale and a YUV pipeline. A "test" pipeline (v3 defaults) sees the scene with
 * the glare / object; a "ref" pipeline sees the same poses without it (or, for the corner and
 * dead-end checks, the same images with the v2 flags), so its output is the expected behaviour.
 *
 * Each test states the phase in which it is expected to pass ({@link #requirePhase}); until that
 * phase is implemented it is skipped, so the suite stays green. Run every scenario regardless
 * with {@code gradlew test -Dv3.runAll=true}. Per-frame traces go to build/v3-scenarios/.
 */
public class V3ScenarioTest {

    /** Highest plan phase implemented in the library. Scenarios for later phases are skipped. */
    static final int IMPLEMENTED_PHASE = 1;

    static final int W = 320, H = 240;
    static final double SPEED = 0.6;          // m/s, user's top speed (plan §8.4)
    static final double TAPE_W = 0.02;        // TrackSim tape width (m)

    private static final int WHITE = 0xEBEBEB, BLACK = 0x1E1C1A, NIGHT = 0x202020, BLUE = 0x1E3CC8,
                             RED = 0xC82828, GREEN = 0x287A28;

    /** Colour pairs (plan §8.2): achromatic both ways + chromatic. */
    enum Pair {
        BLACK_ON_WHITE(WHITE, BLACK, true), WHITE_ON_BLACK(NIGHT, WHITE, true),
        BLACK_ON_BLUE(BLUE, BLACK, true), WHITE_ON_BLUE(BLUE, WHITE, true),
        RED_ON_GREEN(GREEN, RED, false);   // equal brightness: colour input only
        final int floor, line; final boolean grayOk;
        Pair(int floor, int line, boolean grayOk) { this.floor = floor; this.line = line; this.grayOk = grayOk; }
    }

    static final List<Pair> MAIN_PAIRS = Arrays.asList(Pair.BLACK_ON_WHITE, Pair.WHITE_ON_BLACK, Pair.BLACK_ON_BLUE);

    // ── Phase gate ──────────────────────────────────────────────────────────

    private static void requirePhase(int phase) {
        Assume.assumeTrue("expected to pass from plan phase " + phase,
                phase <= IMPLEMENTED_PHASE || Boolean.getBoolean("v3.runAll"));
    }

    // ── Configs (v3 fields set by reflection so this file compiles against every phase) ──

    static LineFollowerPipeline.Config v3Config(double fps) {
        final LineFollowerPipeline.Config c = TrackSim.appConfig();
        set(c, "frameIntervalMs", (float) (1000.0 / fps));
        return c;
    }

    /** Plan §1: corridorMode = false + glareMask = false ⇒ exact v2 behaviour. */
    static LineFollowerPipeline.Config v2Config(double fps) {
        final LineFollowerPipeline.Config c = v3Config(fps);
        set(c, "glareMask", false);
        set(c, "corridorMode", false);
        return c;
    }

    private static void set(Object o, String name, Object v) {
        try {
            o.getClass().getField(name).set(o, v);
        } catch (NoSuchFieldException e) {
            // field not added yet in this phase
        } catch (IllegalAccessException e) {
            throw new IllegalStateException(e);
        }
    }

    private static boolean flag(LineFollowerPipeline.FrameData d, String name) {
        try {
            final Field f = d.getClass().getField(name);
            return f.getBoolean(d);
        } catch (NoSuchFieldException e) {
            return false;
        } catch (IllegalAccessException e) {
            throw new IllegalStateException(e);
        }
    }

    // ── One frame's output ──────────────────────────────────────────────────

    static final class F {
        int s, t; boolean found, held, coasting, recon; float cx = -1, widthPx;
        boolean pinned() { return Math.abs(s) == 50 && !found && !coasting; }
        @Override public String toString() {
            return String.format(Locale.ROOT, "s=%3d t=%3d %s%s%s%s cx=%6.1f", s, t, found ? "F" : "-",
                    held ? "H" : "-", coasting ? "C" : "-", recon ? "R" : "-", cx);
        }
    }

    // ── Scenario description and runner ─────────────────────────────────────

    static final class Scene {
        String name;
        Pair pair;
        double fps = 30;
        TrackSim.Path path;
        Consumer<TrackSim> world = s -> { };          // objects, extra tapes
        List<TrackSim.Glare> glare = new ArrayList<>();
        IntPredicate glareOn = k -> true;             // which frames get the glare
        int calibIndex = 10;                          // path index of the calibration pose
        List<double[]> poses = new ArrayList<>();     // robot poses {x, y, heading} per frame
        boolean refSeesClean = true;                  // ref pipeline: clean image (else the same image)
        LineFollowerPipeline.Config testCfg, refCfg;
    }

    /** Output per format: [0] = GRAY, [1] = YUV; each {test[], ref[]}. */
    static final class Result {
        final Scene sc;
        final F[][] test = new F[2][], ref = new F[2][];
        Result(Scene sc) { this.sc = sc; }
        int frames() { return sc.poses.size(); }
        boolean has(int fmt) { return test[fmt] != null; }
    }

    static final String[] FMT = {"gray", "yuv"};

    static Result run(Scene sc) {
        if (sc.testCfg == null) sc.testCfg = v3Config(sc.fps);
        if (sc.refCfg == null)  sc.refCfg  = sc.testCfg;
        final TrackSim sim = new TrackSim(W, H);
        sim.floorRgb = sc.pair.floor; sim.lineRgb = sc.pair.line;
        sim.tracks.add(sc.path.points());
        sc.world.accept(sim);

        final int f0 = sc.pair.grayOk ? 0 : 1;
        final LineFollowerPipeline[] test = new LineFollowerPipeline[2], ref = new LineFollowerPipeline[2];
        final F[] cur = new F[1];
        for (int f = f0; f < 2; f++) {
            test[f] = new LineFollowerPipeline(); test[f].configure(copy(sc.testCfg));
            ref[f]  = new LineFollowerPipeline(); ref[f].configure(copy(sc.refCfg));
            test[f].setFrameCallback(d -> capture(d, cur[0]));
            ref[f].setFrameCallback(d -> capture(d, cur[0]));
        }
        // Calibrate every pipeline on the same clean frames.
        final double[] cp = sc.path.pose(sc.calibIndex);
        boolean done = false;
        for (int i = 0; i < 80 && !done; i++) {
            final byte[][] yuv = sim.toYuv420(sim.render(cp[0], cp[1], cp[2]));
            done = true;
            cur[0] = new F();
            for (int f = f0; f < 2; f++) {
                done &= calibrate(test[f], f, yuv);
                done &= calibrate(ref[f], f, yuv);
            }
        }
        if (!done) throw new AssertionError(sc.name + ": calibration did not finish");

        final Result r = new Result(sc);
        final int n = sc.poses.size();
        for (int f = f0; f < 2; f++) { r.test[f] = new F[n]; r.ref[f] = new F[n]; }
        for (int k = 0; k < n; k++) {
            final double[] p = sc.poses.get(k);
            final boolean g = !sc.glare.isEmpty() && sc.glareOn.test(k);
            sim.glares.clear();
            final byte[][] clean = sim.toYuv420(sim.render(p[0], p[1], p[2]));
            byte[][] seen = clean;
            if (g) {
                sim.glares.addAll(sc.glare);
                seen = sim.toYuv420(sim.render(p[0], p[1], p[2]));
                sim.glares.clear();
            }
            for (int f = f0; f < 2; f++) {
                r.test[f][k] = cur[0] = new F();
                process(test[f], f, seen);
                r.ref[f][k] = cur[0] = new F();
                process(ref[f], f, sc.refSeesClean ? clean : seen);
            }
        }
        trace(r);
        return r;
    }

    /** Copy a Config field by field (each pipeline gets its own instance). */
    static LineFollowerPipeline.Config copy(LineFollowerPipeline.Config c) {
        final LineFollowerPipeline.Config o = new LineFollowerPipeline.Config();
        for (Field f : LineFollowerPipeline.Config.class.getFields()) {
            try { f.set(o, f.get(c)); } catch (IllegalAccessException e) { throw new IllegalStateException(e); }
        }
        return o;
    }

    private static void capture(LineFollowerPipeline.FrameData d, F out) {
        out.s = d.steering; out.t = d.tracking; out.found = d.trackFound; out.held = d.lineHeld;
        out.coasting = flag(d, "coasting"); out.recon = flag(d, "baseReconstructed");
        out.cx = d.centroidXFrac >= 0 ? d.centroidXFrac * d.imgW : -1;
        out.widthPx = d.trackWidthFrac * d.imgW;
    }

    private static boolean calibrate(LineFollowerPipeline p, int fmt, byte[][] yuv) {
        return fmt == 0 ? p.calibrate(yuv[0], W, H) : p.calibrateYuv420(yuv[0], yuv[1], yuv[2], W, H);
    }

    private static void process(LineFollowerPipeline p, int fmt, byte[][] yuv) {
        if (fmt == 0) p.processFrame(yuv[0], W, H);
        else          p.processFrameYuv420(yuv[0], yuv[1], yuv[2], W, H);
    }

    private static void trace(Result r) {
        final String dir = System.getProperty("v3.reportDir");
        if (dir == null) return;
        try {
            final Path d = Paths.get(dir);
            Files.createDirectories(d);
            try (PrintWriter w = new PrintWriter(Files.newBufferedWriter(
                    d.resolve(r.sc.name.replaceAll("[^A-Za-z0-9_.-]", "_") + ".txt"), StandardCharsets.UTF_8))) {
                for (int k = 0; k < r.frames(); k++) {
                    final StringBuilder sb = new StringBuilder(String.format(Locale.ROOT, "%3d", k));
                    for (int f = 0; f < 2; f++) {
                        if (!r.has(f)) continue;
                        sb.append(" | ").append(FMT[f]).append(" test ").append(r.test[f][k])
                          .append("  ref ").append(r.ref[f][k]);
                    }
                    w.println(sb);
                }
            }
        } catch (IOException e) {
            throw new IllegalStateException(e);
        }
    }

    // ── Pose helpers ────────────────────────────────────────────────────────

    /** Poses along the path, at the given speed and frame rate, starting at sample index i0. */
    static List<double[]> along(TrackSim.Path path, int i0, int frames, double fps) {
        final int step = (int) Math.round(SPEED / fps / 0.005);   // path samples are 5 mm apart
        final List<double[]> out = new ArrayList<>();
        for (int k = 0; k < frames; k++) out.add(path.pose(Math.min(path.size() - 1, i0 + k * step)));
        return out;
    }

    // ── Scenario-level checks ───────────────────────────────────────────────

    /** Collects violations for one scenario; fails with a readable list. */
    static final class Check {
        final List<String> errors = Collections.synchronizedList(new ArrayList<>());
        final java.util.Set<String> failing = java.util.Collections.synchronizedSet(new java.util.TreeSet<>());
        void that(boolean ok, String scene, String fmt, int k, String what, Object detail) {
            if (!ok) failing.add(scene + " [" + fmt + "]");
            if (!ok && errors.size() < 400) errors.add(scene + " [" + fmt + "] frame " + k + ": " + what + " (" + detail + ")");
        }
        void done(String title) {
            if (errors.isEmpty()) return;
            final String head = String.join(", ", failing);
            fail(title + " — " + failing.size() + " failing scene/format runs: " + head + "\n  "
                    + String.join("\n  ", errors.subList(0, Math.min(12, errors.size()))));
        }
    }

    /**
     * Glare ride (plan §7): the line is followed or coasted ({@code trackFound || coasting}),
     * steering never pinned to ±50 and continuous (|Δsteer| ≤ 6 per frame), and not far from the
     * clean reference (|s − ref| ≤ 10, a sanity check that it is not just frozen).
     */
    static void glareRide(Check c, Result r, boolean requireFound) {
        for (int f = 0; f < 2; f++) {
            if (!r.has(f)) continue;
            for (int k = 0; k < r.frames(); k++) {
                final F t = r.test[f][k], ref = r.ref[f][k];
                final boolean ok = requireFound ? t.found : (t.found || t.coasting || t.held);
                c.that(ok, r.sc.name, FMT[f], k, requireFound ? "line not found" : "line neither found nor coasting", t);
                c.that(Math.abs(t.s) < 50, r.sc.name, FMT[f], k, "steering pinned", t);
                if (k > 0) c.that(Math.abs(t.s - r.test[f][k - 1].s) <= 6, r.sc.name, FMT[f], k,
                        "steering jump > 6", r.test[f][k - 1].s + " -> " + t.s);
                c.that(Math.abs(t.s - ref.s) <= 10, r.sc.name, FMT[f], k, "steering differs from clean ref by > 10",
                        t + " / ref " + ref);
            }
        }
    }

    /** Detected centre within {@code frac} line widths of the clean reference, on every frame. */
    static void centroidNearRef(Check c, Result r, double frac) {
        for (int f = 0; f < 2; f++) {
            if (!r.has(f)) continue;
            for (int k = 0; k < r.frames(); k++) {
                final F t = r.test[f][k], ref = r.ref[f][k];
                if (!ref.found) continue;
                final double tol = frac * Math.max(4, ref.widthPx);
                c.that(t.found && Math.abs(t.cx - ref.cx) <= tol, r.sc.name, FMT[f], k,
                        String.format(Locale.ROOT, "centre off the line by > %.1f px", tol), t + " / ref " + ref);
            }
        }
    }

    /** Runs scenes in parallel (each is independent) and applies the check. */
    static void runAll(String title, List<Scene> scenes, java.util.function.BiConsumer<Check, Result> check) {
        final Check c = new Check();
        scenes.parallelStream().forEach(sc -> check.accept(c, run(sc)));
        c.done(title);
    }

    // ── Glare scenes ────────────────────────────────────────────────────────

    /** Saturated ellipse over the H-ROI and the nearest ~20 % of the V-ROI (image rows ~125–192). */
    static TrackSim.Glare nearStripGlare() {
        return new TrackSim.Glare(0.5, 0.66, 0.40, 0.14, 400);
    }

    static TrackSim.Path straightPath()  { return new TrackSim.Path(0, 0, 0).straight(1.8); }
    static TrackSim.Path gentleCurve()   { return new TrackSim.Path(0, 0, 0).straight(0.4).arc(0.6, 60).straight(1.0); }

    /** 10 clean frames, {@code glareFrames} with glare, 10 clean frames. */
    static Scene glareDrive(String name, Pair p, TrackSim.Path path, TrackSim.Glare g, int glareFrames, double fps) {
        final Scene s = new Scene();
        s.name = name + "_" + p + "_" + (int) fps + "fps";
        s.pair = p; s.fps = fps; s.path = path;
        s.glare.add(g);
        s.glareOn = k -> k >= 10 && k < 10 + glareFrames;
        s.poses = along(path, 20, 20 + glareFrames, fps);
        return s;
    }

    /** Plan Phase 0 #1: glare over the near strip, straight and gentle curve, ~40 frames. */
    @Test public void glareNearStrip() {
        requirePhase(1);
        final List<Scene> scenes = new ArrayList<>();
        for (Pair p : MAIN_PAIRS) {
            scenes.add(glareDrive("glareNearStrip_straight", p, straightPath(), nearStripGlare(), 40, 30));
            scenes.add(glareDrive("glareNearStrip_curve", p, gentleCurve(), nearStripGlare(), 40, 30));
        }
        scenes.add(glareDrive("glareNearStrip_curve", Pair.BLACK_ON_WHITE, gentleCurve(), nearStripGlare(), 40, 60));
        runAll("glareNearStrip", scenes, (c, r) -> glareRide(c, r, false));
    }

    /** Plan Phase 0 #2: glare band across the middle of the look-ahead — not a dead end. */
    @Test public void glareMidLookAhead() {
        requirePhase(1);
        final List<Scene> scenes = new ArrayList<>();
        for (Pair p : MAIN_PAIRS) {
            scenes.add(glareDrive("glareMidLookAhead", p, straightPath(),
                    new TrackSim.Glare(0.5, 0.40, 3.0, 0.075, 400).halo(0.1), 40, 30));
        }
        runAll("glareMidLookAhead", scenes, (c, r) -> {
            glareRide(c, r, true);
            for (int f = 0; f < 2; f++) {
                if (!r.has(f)) continue;
                for (int k = 0; k < r.frames(); k++) {
                    c.that(r.test[f][k].t >= r.ref[f][k].t - 10, r.sc.name, FMT[f], k,
                            "tracking dropped (glare read as a dead end)", r.test[f][k] + " / ref " + r.ref[f][k]);
                }
            }
        });
    }

    /** Highlight on the glossy tape's centre over the near part of the view; edges stay visible. */
    static TrackSim.Glare tapeGlare(double amp) {
        return new TrackSim.Glare(0.5, 0.68, 0.40, 0.22, amp).tapeCentre(0.6);
    }

    /** Plan Phase 0 #3 (highest priority, §8.3): saturated specular highlight on the tape centre. */
    @Test public void glareOnTapeOnly() {
        requirePhase(1);
        final List<Scene> scenes = new ArrayList<>();
        final List<Pair> pairs = new ArrayList<>(MAIN_PAIRS);
        pairs.add(Pair.RED_ON_GREEN);
        for (Pair p : pairs) {
            scenes.add(glareDrive("glareOnTapeOnly_straight", p, straightPath(), tapeGlare(450), 40, 30));
            scenes.add(glareDrive("glareOnTapeOnly_curve", p, gentleCurve(), tapeGlare(450), 40, 30));
        }
        runAll("glareOnTapeOnly", scenes, (c, r) -> { glareRide(c, r, true); centroidNearRef(c, r, 0.3); });
    }

    /**
     * User requirement 3: PARTIAL glare — dark glossy tape whose centre is raised only to
     * Y ≈ 150–220 (below the 245 mask): floor, edge, dark tape, bright centre, dark tape, edge,
     * floor. Must be one band of the expected width, not two thin bands and not "too wide".
     */
    @Test public void glareOnTapeOnlyPartial() {
        requirePhase(2);
        final List<Scene> scenes = new ArrayList<>();
        for (Pair p : Arrays.asList(Pair.BLACK_ON_WHITE, Pair.BLACK_ON_BLUE, Pair.RED_ON_GREEN)) {
            scenes.add(glareDrive("glareOnTapeOnlyPartial_straight", p, straightPath(), tapeGlare(170), 40, 30));
            scenes.add(glareDrive("glareOnTapeOnlyPartial_curve", p, gentleCurve(), tapeGlare(170), 40, 30));
        }
        runAll("glareOnTapeOnlyPartial", scenes, (c, r) -> { glareRide(c, r, true); centroidNearRef(c, r, 0.3); });
    }

    // ── Distractor scenes ───────────────────────────────────────────────────

    /** Straight drive past an object in the line colour lying beside the line (right side). */
    static Scene objectDrive(String name, Pair p, double lateralCentre, double length, double halfWidth) {
        final Scene s = new Scene();
        s.name = name + "_" + p;
        s.pair = p; s.path = straightPath();
        s.world = sim -> sim.objects.add(new TrackSim.Obj(
                new TrackSim.Path(0.55, -lateralCentre, 0).straight(length).points(), halfWidth));
        s.poses = along(s.path, 20, 32, 30);
        s.refSeesClean = true;
        return s;
    }

    /**
     * Objects are part of the world, so the "clean" image contains them too. For the distractor
     * scenes the ref must see the line alone: run the ref on a separate world without the object.
     */
    static Result runDistractor(Scene withObject) {
        final Result r = run(withObject);
        final Scene clean = new Scene();
        clean.name = withObject.name + "_clean"; clean.pair = withObject.pair; clean.path = straightPath();
        clean.poses = withObject.poses; clean.testCfg = withObject.testCfg;
        final Result rc = run(clean);
        for (int f = 0; f < 2; f++) if (r.has(f)) r.ref[f] = rc.test[f];
        return r;
    }

    /** Plan Phase 0 #4: same-width object, gap of 1.5 line widths, short. Never locked. */
    @Test public void sameWidthObjectBesideLine() {
        requirePhase(0);
        final Check c = new Check();
        MAIN_PAIRS.parallelStream().forEach(p -> {
            final double gap = 1.5 * TAPE_W;
            final Result r = runDistractor(objectDrive("sameWidthObjectBesideLine", p, TAPE_W + gap, 0.06, TAPE_W / 2));
            centroidNearRef(c, r, 0.3);
            for (int f = 0; f < 2; f++) {
                if (!r.has(f)) continue;
                for (int k = 0; k < r.frames(); k++) {
                    c.that(Math.abs(r.test[f][k].s - r.ref[f][k].s) <= 3, r.sc.name, FMT[f], k,
                            "steering pulled toward the object", r.test[f][k] + " / ref " + r.ref[f][k]);
                }
            }
        });
        c.done("sameWidthObjectBesideLine");
    }

    /** Plan Phase 0 #5: wide blob (8 cm) merged with the line. Centre error ≤ 0.3 line width. */
    @Test public void blobTouchingLine() {
        requirePhase(0);
        final Check c = new Check();
        MAIN_PAIRS.parallelStream().forEach(p -> {
            final double blobHalf = 0.04;
            final Result r = runDistractor(objectDrive("blobTouchingLine", p, TAPE_W / 2 + blobHalf, 0.12, blobHalf));
            centroidNearRef(c, r, 0.3);
        });
        c.done("blobTouchingLine");
    }

    // ── Fallback behaviour must stay v2 (plan Phase 0 #6, §8.5) ─────────────

    /** v3 defaults vs v2 flags on identical frames. */
    static Scene versusV2(String name, Pair p, TrackSim.Path path, List<double[]> poses, int calibIndex) {
        final Scene s = new Scene();
        s.name = name + "_" + p;
        s.pair = p; s.path = path; s.poses = poses; s.calibIndex = calibIndex;
        s.testCfg = v3Config(30); s.refCfg = v2Config(30);
        s.refSeesClean = false;
        return s;
    }

    /** Sharp 90° corner (R = 3 cm), robot on the line through the turn. */
    @Test public void ninetyDegreeCorner() {
        requirePhase(0);
        final Check c = new Check();
        MAIN_PAIRS.parallelStream().forEach(p -> {
            final TrackSim.Path path = new TrackSim.Path(0, 0, 0).straight(0.5).arc(0.03, 90).straight(1.0);
            final List<double[]> poses = new ArrayList<>();
            for (int i = 40; i < path.size() - 80; i += 4) poses.add(path.pose(i));
            final Result r = run(versusV2("ninetyDegreeCorner", p, path, poses, 10));
            for (int f = 0; f < 2; f++) {
                if (!r.has(f)) continue;
                final F[] t = r.test[f], v2 = r.ref[f];
                int foundOnlyInV2 = 0;
                for (int k = 0; k < t.length; k++) {
                    if (v2[k].found && t[k].found) c.that(Math.abs(t[k].s - v2[k].s) <= 5, r.sc.name, FMT[f], k,
                            "steering differs from v2 by > 5", t[k] + " / v2 " + v2[k]);
                    if (v2[k].found && !t[k].found) foundOnlyInV2++;
                    if (t[k].pinned() && v2[k].pinned()) c.that(Integer.signum(t[k].s) == Integer.signum(v2[k].s),
                            r.sc.name, FMT[f], k, "pinned to the other side than v2", t[k] + " / v2 " + v2[k]);
                }
                c.that(foundOnlyInV2 <= 1, r.sc.name, FMT[f], -1, "frames where v2 sees the line but v3 does not", foundOnlyInV2);
                for (int k = t.length - 8; k < t.length; k++) {
                    c.that(t[k].found && Math.abs(t[k].s) <= 2, r.sc.name, FMT[f], k, "not back on the line after the corner", t[k]);
                }
            }
        });
        c.done("ninetyDegreeCorner");
    }

    /** Dead end: tracking ramps down like v2 (within 10) and reaches the minimum. */
    @Test public void lineEnds() {
        requirePhase(0);
        final Check c = new Check();
        MAIN_PAIRS.parallelStream().forEach(p -> {
            final TrackSim.Path path = new TrackSim.Path(0, 0, 0).straight(0.8);
            final List<double[]> poses = new ArrayList<>();
            for (double x = 0.10; x <= 0.85 + 1e-9; x += 0.02) poses.add(new double[]{x, 0, 0});
            final Result r = run(versusV2("lineEnds", p, path, poses, 4));
            for (int f = 0; f < 2; f++) {
                if (!r.has(f)) continue;
                final F[] t = r.test[f], v2 = r.ref[f];
                c.that(t[0].t == 100, r.sc.name, FMT[f], 0, "full speed while the road ahead is long", t[0]);
                c.that(t[t.length - 1].t == 5, r.sc.name, FMT[f], t.length - 1, "minimum at the end", t[t.length - 1]);
                for (int k = 1; k < t.length; k++) {
                    c.that(t[k].t <= t[k - 1].t + 2, r.sc.name, FMT[f], k, "tracking rises approaching the end", t[k]);
                    c.that(t[k - 1].t - t[k].t <= 20, r.sc.name, FMT[f], k, "sudden drop", t[k - 1].t + " -> " + t[k].t);
                    c.that(Math.abs(t[k].t - v2[k].t) <= 10, r.sc.name, FMT[f], k, "tracking differs from v2 by > 10",
                            t[k] + " / v2 " + v2[k]);
                }
            }
        });
        c.done("lineEnds");
    }

    // ── User requirement 4: an edge exit beats coasting ─────────────────────

    /**
     * The robot overshoots a sharp turn so the line leaves the view at the frame edge — while
     * glare covers the near strip the whole time. Reference: the same drive without glare, v2
     * flags (v2 pins at once when the line leaves at the edge). The glare run must follow the line
     * until then (pinned early only to the exit side, once v2 also turns that way), pin to the same side
     * no later than one frame after v2, and never coast through the exit.
     */
    @Test public void edgeExitUnderGlare() {
        requirePhase(1);
        final Check c = new Check();
        Arrays.asList(Pair.WHITE_ON_BLUE, Pair.BLACK_ON_WHITE).parallelStream().forEach(p -> {
            final double r0 = 0.12, angle = 90, cut = 0.04;
            final TrackSim.Path path = new TrackSim.Path(0, 0, 0).straight(0.5).arc(r0, angle).straight(0.8);
            final int arcStart = 100, arcLen = (int) Math.round(Math.toRadians(angle) * r0 / 0.005);
            final int bumpStart = arcStart - arcLen / 2, bumpEnd = arcStart + arcLen + arcLen / 2;
            final List<double[]> poses = new ArrayList<>();
            for (int i = arcStart - 40; i < path.size() - 100; i += 4) {
                final double[] pose = path.pose(i);
                double lat = 0, dh = 0;
                if (i > bumpStart && i < bumpEnd) {
                    final double t = (i - bumpStart) / (double) (bumpEnd - bumpStart);
                    lat = cut * Math.sin(Math.PI * t);
                    dh = -Math.toRadians(35) * Math.sin(Math.PI * t);
                }
                poses.add(new double[]{pose[0] - lat * Math.sin(pose[2]), pose[1] + lat * Math.cos(pose[2]), pose[2] + dh});
            }
            final Scene s = new Scene();
            s.name = "edgeExitUnderGlare_" + p;
            s.pair = p; s.path = path; s.poses = poses; s.calibIndex = 10;
            s.glare.add(nearStripGlare());
            s.glareOn = k -> k >= 5;                  // once the line is locked
            s.testCfg = v3Config(30); s.refCfg = v2Config(30);
            final Result r = run(s);
            for (int f = 0; f < 2; f++) {
                if (!r.has(f)) continue;
                final F[] t = r.test[f], ref = r.ref[f];
                int pinK = -1;
                for (int k = 0; k < ref.length && pinK < 0; k++) if (ref[k].pinned()) pinK = k;
                if (pinK < 0) { c.that(false, r.sc.name, FMT[f], -1, "reference never left the line at the edge", ""); continue; }
                final int side = Integer.signum(ref[pinK].s);
                for (int k = 0; k < pinK; k++) {
                    if (t[k].pinned()) {
                        // Requirement 4 pins as soon as the look-ahead shows the line leaving
                        // through the edge — possibly before v2 (which watches the hidden base)
                        // gets there. Allowed if it is the exit side and v2 already turns that way.
                        c.that(Integer.signum(t[k].s) == side && Integer.signum(ref[k].s) == side
                                && Math.abs(ref[k].s) >= 10, r.sc.name, FMT[f], k,
                                "pinned before the line heads for the edge", t[k] + " / ref " + ref[k]);
                    } else {
                        c.that(t[k].found || t[k].coasting, r.sc.name, FMT[f], k, "line lost under the glare", t[k]);
                        c.that(Math.abs(t[k].s - ref[k].s) <= 15, r.sc.name, FMT[f], k, "not following the line",
                                t[k] + " / ref " + ref[k]);
                    }
                }
                final boolean pinnedInTime = (t[pinK].pinned() && Integer.signum(t[pinK].s) == side)
                        || (pinK + 1 < t.length && t[pinK + 1].pinned() && Integer.signum(t[pinK + 1].s) == side);
                c.that(pinnedInTime, r.sc.name, FMT[f], pinK, "not pinned to the exit edge within 1 frame of v2",
                        t[pinK] + " / ref " + ref[pinK]);
                for (int k = pinK; k < t.length && ref[k].pinned(); k++) {
                    c.that(!t[k].coasting, r.sc.name, FMT[f], k, "coasting through an edge exit", t[k]);
                }
            }
        });
        c.done("edgeExitUnderGlare");
    }
}
