package harness;

import com.example.linefollower.LineFollowerPipeline;

import java.lang.reflect.Field;

/**
 * Thin adapter around the library under test. Compiled against the ORIGINAL API only;
 * fields that exist only in v2 (Config weights, FrameData components) are reached by
 * reflection, so the same harness runs unchanged on both versions.
 */
final class Lib {
    final LineFollowerPipeline p = new LineFollowerPipeline();
    final LineFollowerPipeline.Config cfg = new LineFollowerPipeline.Config();
    /** Copy of the latest FrameData values we use. */
    volatile float centroidXFrac = -1, tiltDeg, trackWidthFrac;
    volatile boolean trackFound;
    volatile int steering, tracking, calibIssue, threshold;
    LineFollowerPipeline.FrameData last;

    static final String TAG = System.getProperty("lib.tag", "?");

    /** Same settings as RoboPhoneApp's MainActivity. */
    Lib() {
        cfg.autoColor = true;
        cfg.thresholdMargin = 0;
        cfg.hRoiCenterYFrac = 0.85f;
        cfg.hRoiThickPx = 24;
        cfg.bottomIgnoreFrac = 0.25f;
        cfg.vRoiHalfWidthPx = 80;
        cfg.cameraTiltDeg = 45f;
        cfg.verticalFovDeg = 60f;
        for (String kv : System.getProperty("lib.set", "").split(",")) if (kv.contains("=")) set(kv.split("=")[0], kv.split("=")[1]);
        p.configure(cfg);
        p.setFrameCallback(d -> {
            last = d;
            centroidXFrac = d.centroidXFrac; tiltDeg = d.tiltDeg; trackWidthFrac = d.trackWidthFrac;
            trackFound = d.trackFound; steering = d.steering; tracking = d.tracking;
            calibIssue = d.calibIssue; threshold = d.threshold;
        });
    }

    /** Set a Config field by name (works for v2-only fields). Returns false if absent. */
    boolean set(String name, String value) {
        try {
            final Field f = LineFollowerPipeline.Config.class.getField(name.trim());
            final Class<?> t = f.getType();
            if (t == boolean.class) f.setBoolean(cfg, Boolean.parseBoolean(value.trim()));
            else if (t == int.class) f.setInt(cfg, Integer.parseInt(value.trim()));
            else if (t == float.class) f.setFloat(cfg, Float.parseFloat(value.trim()));
            else return false;
            p.configure(cfg);
            return true;
        } catch (NoSuchFieldException e) {
            return false;
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }

    void reconfigure() { p.configure(cfg); }

    /** Read a FrameData float/int field by name; NaN if it does not exist (original library). */
    double fd(String name) {
        if (last == null) return Double.NaN;
        try {
            final Field f = LineFollowerPipeline.FrameData.class.getField(name);
            final Class<?> t = f.getType();
            if (t == float.class) return f.getFloat(last);
            if (t == int.class) return f.getInt(last);
            if (t == boolean.class) return f.getBoolean(last) ? 1 : 0;
        } catch (NoSuchFieldException e) {
            return Double.NaN;
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        return Double.NaN;
    }

    static boolean isV2() {
        try { LineFollowerPipeline.FrameData.class.getField("headingDeg"); return true; }
        catch (NoSuchFieldException e) { return false; }
    }

    boolean calibrate(Render r) { return p.calibrateYuv420(r.y, r.u, r.v, r.W, r.H); }
    int[] process(Render r) { return p.processFrameYuv420(r.y, r.u, r.v, r.W, r.H); }

    /** Calibrate on the rendered pose; returns frames used or -1 if it never finished. */
    int calibrateAt(Render r, double x, double y, double th, int maxFrames) {
        for (int i = 1; i <= maxFrames; i++) {
            r.render(x, y, th);
            if (calibrate(r)) return i;
        }
        return -1;
    }
}
