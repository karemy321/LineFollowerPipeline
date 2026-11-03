#!/usr/bin/env bash
# Compile + run the project's JUnit tests against build/v3 (fast). Extra args → JVM options. Run from _investigation/.
cd "$(dirname "$0")"
JDK="${JAVA_HOME:-/c/Program Files/Android/Android Studio/jbr}/bin"
JD=build/junit_${LIB:-v3}; rm -rf $JD && mkdir -p $JD
"$JDK/javac" -encoding UTF-8 -nowarn -cp "build/${LIB:-v3};build/lib/junit-4.13.2.jar" -d $JD ../LineFollowerLibrary/src/test/java/com/example/linefollower/*.java 2>&1 | grep -v "^Note" 
"$JDK/java" -Xmx2g "$@" -cp "build/${LIB:-v3};$JD;build/lib/junit-4.13.2.jar;build/lib/hamcrest-core-1.3.jar" org.junit.runner.JUnitCore \
  com.example.linefollower.LineFollowerPipelineTest com.example.linefollower.TrackSimTest com.example.linefollower.PathTrackingTest 2>&1 \
  | grep -E "^OK|^Tests run|^[0-9]+\) |AssertionError|expected" | head -40
