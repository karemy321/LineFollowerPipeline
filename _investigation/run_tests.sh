#!/usr/bin/env bash
# All non-closed-loop tests, ORIGINAL vs NEW, then the report tables.
# Run from _investigation/ after ./build.sh. Results in results/, converted images in snaps/.
set -e
cd "$(dirname "$0")"
JDK="${JAVA_HOME:-/c/Program Files/Android/Android Studio/jbr}"
J="$JDK/bin/java"
T=test_images
IMG0="$T/WhatsApp Image 2026-10-16 at 18.40.04.jpeg"
IMG1="$T/WhatsApp Image 2026-10-16 at 18.40.04 (1).jpeg"
IMG2="$T/WhatsApp Image 2026-10-16 at 18.40.04 (2).jpeg"
JU="build/lib/junit-4.13.2.jar;build/lib/hamcrest-core-1.3.jar"

for l in orig new; do
  echo "== $l: distractor frames";  "$J" -Xmx6g -Dlib.tag=$l -cp "build/$l;build/harness" harness.FrameTests distract results/distract_$l.csv > /dev/null
  echo "== $l: heading test";       "$J" -Xmx6g -Dlib.tag=$l -cp "build/$l;build/harness" harness.FrameTests heading  results/heading_$l.csv  > /dev/null
  echo "== $l: regression";         "$J" -Xmx4g -Dlib.tag=$l -cp "build/$l;build/harness" harness.Regression results/regression_$l.csv | tail -1
  echo "== $l: real images";        "$J" -Xmx4g -Dlib.tag=$l -cp "build/$l;build/harness" harness.RealImage results/real_$l.csv snaps "$IMG0" "$IMG1" "$IMG2" > /dev/null
done
echo "== zero allocation + timing (2 runs each)"
for rep in 1 2; do for l in orig new; do "$J" -Xmx4g -Dlib.tag=$l -cp "build/$l;build/harness" harness.Perf both; done; done | tee results/perf.txt
echo "== existing JUnit tests"
for l in orig new; do echo "-- $l"; "$J" -cp "build/$l;build/junit_$l;$JU" org.junit.runner.JUnitCore com.example.linefollower.LineFollowerPipelineTest com.example.linefollower.TrackSimTest | grep -E "^OK|Tests run" || true; done
echo "-- new with steerHeadingWeight=0, trackHoldFrames=0"
"$J" -cp "build/new_noHeadNoHold;build/junit_new;$JU" org.junit.runner.JUnitCore com.example.linefollower.LineFollowerPipelineTest com.example.linefollower.TrackSimTest | grep -E "^OK|Tests run" || true
if [ -f results/closed/real_new.csv ]; then
  "$J" -cp build/harness harness.Report results results/report_tables.md && echo "report tables: results/report_tables.md"
else
  echo "(run ./run_closed_loop.sh, then: java -cp build/harness harness.Report results results/report_tables.md)"
fi
