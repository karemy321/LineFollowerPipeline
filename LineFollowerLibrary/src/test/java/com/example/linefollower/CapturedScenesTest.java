package com.example.linefollower;

import static org.junit.Assert.assertFalse;
import static org.junit.Assume.assumeTrue;

import java.io.File;
import java.io.IOException;
import java.util.Arrays;

import org.junit.Test;

/**
 * Replays every capture saved on the phone that was copied to {@code src/test/captures/}
 * (see the README there). Skipped while the folder holds none. Prints, per file, how close the
 * desktop replay is to what the phone did; scene-specific checks are added during tuning.
 */
public class CapturedScenesTest {

    @Test public void replayCapturedScenes() throws IOException {
        final File dir = new File(System.getProperty("captures.dir", "src/test/captures"));
        final File[] files = dir.listFiles((d, n) -> n.startsWith("lfcap_") && n.endsWith(".bin"));
        assumeTrue("no captures in " + dir.getAbsolutePath(), files != null && files.length > 0);
        Arrays.sort(files);
        for (File f : files) {
            final CaptureReplay.Capture c = CaptureReplay.read(f);
            final CaptureReplay.Result r = CaptureReplay.replay(c, CaptureReplay.config(c.config));
            assertFalse(f.getName() + ": no frames after calibration", r.run.isEmpty());
            int sum = 0, max = 0, pins = 0;
            for (int i = 0; i < r.run.size(); i++) {
                final int d = Math.abs(r.replay.get(i)[0] - r.run.get(i).steering);
                sum += d; max = Math.max(max, d);
                if (Math.abs(r.replay.get(i)[0]) >= 50) pins++;
            }
            System.out.printf("%s (%s): %d frames, replay calibrated %s, |phone - replay| mean %.1f max %d, replay ±50 frames %d%n",
                    f.getName(), c.reason, r.run.size(), r.calibrated, sum / (double) r.run.size(), max, pins);
        }
    }
}
