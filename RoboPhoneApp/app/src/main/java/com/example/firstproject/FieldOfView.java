package com.example.firstproject;

/**
 * Field of view of a camera stream from Camera2 data (pure arithmetic, unit-tested).
 *
 * FOV = 2·atan(size / 2f). The sensor's physical size covers the whole pixel array; the active
 * array is the part that is read out. CameraX scales the active array to the stream size and
 * crops the centre when the aspect ratios differ. The frame's vertical axis is the sensor axis
 * that has as many pixels as the frame height (frames may be rotated to portrait).
 */
final class FieldOfView {
    private FieldOfView() { }

    /**
     * @return {vertical, horizontal} FOV of the frame in degrees, plus the crop in mm
     *         {.., .., cropW, cropH} in sensor orientation
     */
    static double[] compute(double focalMm, double physWmm, double physHmm, int pixW, int pixH,
                            int activeW, int activeH, int frameW, int frameH) {
        final double actW = physWmm * activeW / pixW, actH = physHmm * activeH / pixH;
        final boolean sensorLandscape = activeW >= activeH;
        final int sw = sensorLandscape ? Math.max(frameW, frameH) : Math.min(frameW, frameH);
        final int sh = sensorLandscape ? Math.min(frameW, frameH) : Math.max(frameW, frameH);
        double cropW = actW, cropH = actH;
        final double streamAspect = sw / (double) sh, activeAspect = activeW / (double) activeH;
        if (streamAspect > activeAspect) cropH = actW / streamAspect;   // wider stream: top / bottom cut
        else cropW = actH * streamAspect;                                // narrower stream: sides cut
        final double fovW = Math.toDegrees(2 * Math.atan(cropW / (2 * focalMm)));
        final double fovH = Math.toDegrees(2 * Math.atan(cropH / (2 * focalMm)));
        final boolean verticalIsSensorH = frameH == sh;
        return new double[]{verticalIsSensorH ? fovH : fovW, verticalIsSensorH ? fovW : fovH, cropW, cropH};
    }
}
