# Captured scenes from the phone

Put `lfcap_*.bin` files saved by `LineFollowerPipeline.Recorder` (CAPTURE in RoboPhoneApp, or
`LineFollowerHelper.startRecording(..., capture = true)` in RoboPhone) in this folder.
`CapturedScenesTest` replays every file here on each `gradlew test` and prints phone vs replay
steering; checks for a specific scene are added there during tuning.

Replay one file by hand (from `LineFollowerLibrary/`):

    gradlew replay -Pcapture=src/test/captures/lfcap_20261110_142233_coast.bin
    gradlew replay -Pcapture=... -Pset=glareMask=false,corridorMode=false    # same scene, v2 logic

A capture at the default 1/4 size is about 15 MB (960 × 1280 YUV, 120 frames + calibration).
