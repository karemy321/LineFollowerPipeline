#!/usr/bin/env bash
# v3 closed-loop campaign: v2 (commit 202e70f) vs v3 (working copy), identical scenarios.
# Run from _investigation/ in Git Bash. Results: results/v3/*.csv, summary: results/v3/summary.md
#   ./run_v3_closed_loop.sh            # build + all regimes
#   REGIMES="real glare" ./run_v3_closed_loop.sh
set -e
cd "$(dirname "$0")"
JDK="${JAVA_HOME:-/c/Program Files/Android/Android Studio/jbr}"
J="$JDK/bin/java"; JC="$JDK/bin/javac"
THREADS="${THREADS:-18}"
SEP=";"; [[ "$(uname -s)" != MINGW* && "$(uname -s)" != MSYS* ]] && SEP=":"
SRC=com/example/linefollower/LineFollowerPipeline.java

rm -rf build/v2 build/v3 build/harness_v3 build/v2src
mkdir -p build/v2 build/v3 build/harness_v3 build/v2src/com/example/linefollower results/v3
git -C .. show 202e70f:LineFollowerLibrary/src/main/java/$SRC > build/v2src/$SRC
"$JC" -encoding UTF-8 --release 11 -nowarn -d build/v2 build/v2src/$SRC
"$JC" -encoding UTF-8 --release 11 -nowarn -d build/v3 ../LineFollowerLibrary/src/main/java/$SRC
"$JC" -encoding UTF-8 -nowarn -cp build/v2 -d build/harness_v3 harness/src/harness/*.java

run() {   # run <name> <lib v2|v3> <java -D options...>
  local name=$1 lib=$2; shift 2
  local libset=""
  # v3: coast clock in simulated time (the simulator renders at 60 Hz, not in real time)
  [[ $lib == v3 ]] && libset="-Dlib.set=frameIntervalMs=16.667"
  "$J" -Xmx10g -Dlib.tag=$lib $libset "$@" -cp "build/$lib${SEP}build/harness_v3" harness.ClosedLoop "results/v3/${name}_$lib.csv" sweep "$THREADS" > /dev/null
  echo "  done: ${name}_$lib"
}

ALL="-Dsweep.t=straight,gentle,scurve,sharp90,oval -Dsweep.s=white_on_dark,black_on_wood"
REAL="-Dsim.lat=0.15 -Dsim.hz=20 -Dsim.dead=2"
STRESS="-Dsim.lat=0.2 -Dsim.hz=15 -Dsim.dead=3"
for reg in ${REGIMES:-real stress glare glarestress}; do
  for lib in ${LIBS:-v2 v3}; do
    case $reg in
      # realistic timing: 150 ms latency, 20 Hz Blockly loop, dead zone 2; power 5…17 → 5…17 cm/s
      real)        run real $lib $ALL -Dsweep.d=0,1 -Dsweep.c=0.5,1.0,2.0 -Dsweep.x=0,1 -Dsweep.v=1.0 $REAL ;;
      # 3× faster (power 17 ≈ 0.5 m/s — the user's 0.5–0.6 m/s), 200 ms latency, 15 Hz, dead zone 3
      stress)      run stress $lib $ALL -Dsweep.d=0,1 -Dsweep.c=0.5,1.0 -Dsweep.x=0,1 -Dsweep.v=3.0 $STRESS ;;
      # glare over the near strip in 3 lamp zones (15 % of the track each)
      glare)       run glare $lib $ALL -Dsweep.d=0 -Dsweep.c=0.5,1.0 -Dsweep.x=0,1 -Dsweep.v=1.0 $REAL -Dsim.glare=near ;;
      glarestress) run glarestress $lib $ALL -Dsweep.d=0 -Dsweep.c=0.5,1.0 -Dsweep.x=0,1 -Dsweep.v=3.0 $STRESS -Dsim.glare=near ;;
    esac
  done
done
echo "closed-loop v3 campaign finished"
