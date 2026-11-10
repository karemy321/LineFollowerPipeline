package com.example.firstproject;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class FieldOfViewTest {
    // A typical 4:3 phone sensor: 5.64 × 4.23 mm, f = 4.38 mm, 4000 × 3000 px, all active.
    private static double[] fov(int w, int h) {
        return FieldOfView.compute(4.38, 5.64, 4.23, 4000, 3000, 4000, 3000, w, h);
    }

    @Test public void portrait4by3UsesTheSensorWidthVertically() {
        final double[] r = fov(960, 1280);                    // app: 1280 × 960 rotated to portrait
        assertEquals(Math.toDegrees(2 * Math.atan(5.64 / 8.76)), r[0], 1e-9);   // 65.5°
        assertEquals(Math.toDegrees(2 * Math.atan(4.23 / 8.76)), r[1], 1e-9);   // 51.6°
        assertEquals(65.5, r[0], 0.1);
    }

    @Test public void landscapeFrameSwapsTheAxes() {
        final double[] r = fov(1280, 960);
        assertEquals(fov(960, 1280)[1], r[0], 1e-9);
        assertEquals(fov(960, 1280)[0], r[1], 1e-9);
    }

    @Test public void sixteenByNineCropsTheShortSide() {
        final double[] r = fov(720, 1280);                    // 1280 × 720 stream, portrait
        assertEquals(65.5, r[0], 0.1);                         // long side: full sensor width
        assertEquals(Math.toDegrees(2 * Math.atan(5.64 * 9 / 16 / 8.76)), r[1], 1e-9);   // 39.8°
        assertEquals(5.64 * 9 / 16, r[3], 1e-9);
    }

    @Test public void inactiveBorderPixelsAreLeftOut() {
        final double[] r = FieldOfView.compute(4.38, 5.64, 4.23, 4032, 3024, 4000, 3000, 960, 1280);
        assertEquals(Math.toDegrees(2 * Math.atan(5.64 * 4000 / 4032 / 8.76)), r[0], 1e-9);
    }

    @Test public void squareishStreamCropsTheSides() {
        final double[] r = fov(1080, 1080);                   // 1:1 stream from a 4:3 sensor
        assertEquals(Math.toDegrees(2 * Math.atan(4.23 / 8.76)), r[0], 1e-9);
        assertEquals(r[0], r[1], 1e-9);
    }
}
