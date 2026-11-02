#!/usr/bin/env bash
# Compile v2 (commit 202e70f, extracted to v2src/), v3 (working copy, Java 8), the
# harness (against v3; v3-only fields are only touched when the v3 library is
# loaded) and the project's JUnit tests against v3. Run from _investigation/.
set -e
cd "$(dirname "$0")"
JDK="${JAVA_HOME:-/c/Program Files/Android/Android Studio/jbr}"
export PATH="$JDK/bin:$PATH"
SRC=com/example/linefollower/LineFollowerPipeline.java
if [ ! -f v2src/$SRC ]; then mkdir -p v2src/com/example/linefollower; git show 202e70f:LineFollowerLibrary/src/main/java/$SRC > v2src/$SRC; fi
rm -rf build/v2 build/v3 build/harness build/junit_v3
mkdir -p build/v2 build/v3 build/harness build/lib build/junit_v3 results snaps
javac -encoding UTF-8 --release 8 -nowarn -d build/v2 v2src/$SRC 2>/dev/null
javac -encoding UTF-8 --release 8 -Xlint:all -Xlint:-options -d build/v3 ../LineFollowerLibrary/src/main/java/$SRC
javac -encoding UTF-8 -nowarn -cp build/v3 -d build/harness harness/src/harness/*.java
G="$HOME/.gradle/caches/modules-2/files-2.1"
cp "$G"/junit/junit/4.13.2/*/junit-4.13.2.jar build/lib/ 2>/dev/null || true
cp "$G"/org.hamcrest/hamcrest-core/1.3/*/hamcrest-core-1.3.jar build/lib/ 2>/dev/null || true
javac -encoding UTF-8 -nowarn -cp "build/v3;build/lib/junit-4.13.2.jar" -d build/junit_v3 ../LineFollowerLibrary/src/test/java/com/example/linefollower/*.java
echo "build OK"
