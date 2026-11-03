#!/usr/bin/env bash
# Rebuild only the v3 library (working copy) into build/v3 and the harness. Run from _investigation/.
cd "$(dirname "$0")"
JDK="${JAVA_HOME:-/c/Program Files/Android/Android Studio/jbr}/bin"
rm -rf build/v3 && mkdir -p build/v3
"$JDK/javac" -encoding UTF-8 --release 8 -Xlint:all -Xlint:-options -d build/v3 ../LineFollowerLibrary/src/main/java/com/example/linefollower/LineFollowerPipeline.java || exit 1
"$JDK/javac" -encoding UTF-8 -nowarn -cp build/v3 -d build/harness harness/src/harness/*.java || exit 1
echo "fast build OK"
