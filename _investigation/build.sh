#!/usr/bin/env bash
# Compile ORIGINAL (from the backup), NEW (working copy), the test harness and the
# project's own JUnit tests against both. Run from _investigation/ in Git Bash.
set -e
cd "$(dirname "$0")"
JDK="${JAVA_HOME:-/c/Program Files/Android/Android Studio/jbr}"
export PATH="$JDK/bin:$PATH"
BACKUP="${BACKUP:-../../LineFollowerPipeline_backup_2026-10-18}"
SRC=com/example/linefollower/LineFollowerPipeline.java

rm -rf build/orig build/new build/harness build/junit_orig build/junit_new build/new_noHeadNoHold
mkdir -p build/orig build/new build/harness build/lib build/variant_src results snaps
javac -encoding UTF-8 --release 11 -d build/orig "$BACKUP/LineFollowerLibrary/src/main/java/$SRC"
javac -encoding UTF-8 --release 11 -Xlint:all -d build/new "../LineFollowerLibrary/src/main/java/$SRC"
javac -encoding UTF-8 -nowarn -cp build/orig -d build/harness harness/src/harness/*.java

# NEW with only the heading term and the hold switched off (for the existing unit tests, §6.6)
sed 's|public float steerHeadingWeight = 1.0f;|public float steerHeadingWeight = 0.0f;|; s|public int trackHoldFrames = 6;|public int trackHoldFrames = 0;|' \
    "../LineFollowerLibrary/src/main/java/$SRC" > build/variant_src/LineFollowerPipeline.java
javac -encoding UTF-8 --release 11 -d build/new_noHeadNoHold build/variant_src/LineFollowerPipeline.java

# JUnit jars from the Gradle cache (copied locally: Windows javac/java need plain paths)
G="$HOME/.gradle/caches/modules-2/files-2.1"
cp "$G"/junit/junit/4.13.2/*/junit-4.13.2.jar build/lib/ 2>/dev/null || true
cp "$G"/org.hamcrest/hamcrest-core/1.3/*/hamcrest-core-1.3.jar build/lib/ 2>/dev/null || true
for l in orig new; do
  mkdir -p build/junit_$l
  javac -encoding UTF-8 -nowarn -cp "build/$l;build/lib/junit-4.13.2.jar" -d build/junit_$l ../LineFollowerLibrary/src/test/java/com/example/linefollower/*.java
done
echo "build OK"
