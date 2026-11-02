#!/usr/bin/env bash
# Compile the working copy into build/v3dev (+ JUnit tests into build/junit_dev) without touching build/v3.
cd "$(dirname "$0")"
JDK="${JAVA_HOME:-/c/Program Files/Android/Android Studio/jbr}/bin"
rm -rf build/v3dev build/junit_dev; mkdir -p build/v3dev build/junit_dev
"$JDK/javac" -encoding UTF-8 --release 8 -Xlint:all -Xlint:-options -d build/v3dev ../LineFollowerLibrary/src/main/java/com/example/linefollower/LineFollowerPipeline.java || exit 1
"$JDK/javac" -encoding UTF-8 -nowarn -cp "build/v3dev;build/lib/junit-4.13.2.jar" -d build/junit_dev ../LineFollowerLibrary/src/test/java/com/example/linefollower/*.java || exit 1
echo "dev build OK"
