src_old_readme\ReadmeExamples.java:99: error: ';' expected
        pipeline.getFramesPerSecond()   // frames processed in the last completed second
                                     ^
src_old_readme\ReadmeExamples.java:100: error: ';' expected
        pipeline.getTotalFrames()       // cumulative frame count since init
                                 ^
src_old_readme\ReadmeExamples.java:101: error: ';' expected
        pipeline.getElapsedSeconds()    // wall-clock seconds since first frame
                                    ^
src_old_readme\ReadmeExamples.java:102: error: ';' expected
        pipeline.isCalibrated()         // true once calibration is complete
                               ^
src_old_readme\ReadmeExamples.java:103: error: ';' expected
        pipeline.isColorModelReady()    // true once the line/floor colours are learned (or preset)
                                    ^
src_old_readme\ReadmeExamples.java:104: error: ';' expected
        pipeline.getLineColorArgb()     // learned line colour, 0xAARRGGBB
                                   ^
src_old_readme\ReadmeExamples.java:105: error: ';' expected
        pipeline.getFloorColorArgb()    // learned floor colour, 0xAARRGGBB
                                    ^
src_old_readme\ReadmeExamples.java:106: error: ';' expected
        pipeline.requestRecalibration() // re-learn colours + straight-ahead — call calibrate() again before processFrame()
                                       ^
src_old_readme\ReadmeExamples.java:107: error: ';' expected
        pipeline.release()              // shut down and reset
                          ^
src_old_readme\ReadmeExamples.java:108: error: ';' expected
        pipeline.isInitialized()        // false until the first frame (or after release()/configure())
                                ^
src_old_readme\ReadmeExamples.java:109: error: ';' expected
        pipeline.setFrameCallback(cb)   // register overlay callback; pass null to unregister
                                     ^
src_old_readme\ReadmeExamples.java:114: error: not a statement
        pipeline.getImgW() / getImgH()                 // frame size in pixels
                           ^
src_old_readme\ReadmeExamples.java:114: error: ';' expected
        pipeline.getImgW() / getImgH()                 // frame size in pixels
                                      ^
src_old_readme\ReadmeExamples.java:115: error: not a statement
        pipeline.getHRoiTop() / getHRoiBot()           // scan-strip rows
                              ^
src_old_readme\ReadmeExamples.java:115: error: ';' expected
        pipeline.getHRoiTop() / getHRoiBot()           // scan-strip rows
                                            ^
src_old_readme\ReadmeExamples.java:116: error: ';' expected
        pipeline.getVRoiTop()                          // far edge row of the look-ahead
                             ^
src_old_readme\ReadmeExamples.java:117: error: ';' expected
        pipeline.getVRoiHalfWidthPx()                  // look-ahead half-width (config)
                                     ^
src_old_readme\ReadmeExamples.java:118: error: not a statement
        pipeline.getSteerRefXFrac() / getTrackRefXFrac()   // calibrated straight-ahead columns
                                    ^
src_old_readme\ReadmeExamples.java:118: error: ';' expected
        pipeline.getSteerRefXFrac() / getTrackRefXFrac()   // calibrated straight-ahead columns
                                                        ^
src_old_readme\ReadmeExamples.java:119: error: not a statement
        pipeline.getLookAheadPointCount() / getLookAheadColFracs() / getLookAheadRowFracs()
                                                                   ^
src_old_readme\ReadmeExamples.java:119: error: ';' expected
        pipeline.getLookAheadPointCount() / getLookAheadColFracs() / getLookAheadRowFracs()
                                                                                           ^
src_old_readme\ReadmeExamples.java:120: error: ';' expected
        pipeline.getHorizonYFrac()                     // floor horizon row from the mount tilt
                                  ^
src_old_readme\ReadmeExamples.java:121: error: ';' expected
        pipeline.perspectiveWidthScale(yFrac)          // relative apparent width at a row
                                             ^
23 errors
