#!/usr/bin/env bash
# Closed-loop simulator campaign: ORIGINAL vs NEW library, identical scenarios.
# Run from _investigation/ after build.sh. Results: results/closed/*.csv
set -e
cd "$(dirname "$0")"
JAVA="${JAVA_HOME:-/c/Program Files/Android/Android Studio/jbr}/bin/java"
THREADS="${THREADS:-18}"
mkdir -p results/closed
CP_SEP=";"; [[ "$(uname -s)" != MINGW* && "$(uname -s)" != MSYS* ]] && CP_SEP=":"

run() {   # run <name> <lib orig|new> <java -D options...>
  local name=$1 lib=$2; shift 2
  "$JAVA" -Xmx10g -Dlib.tag=$lib "$@" -cp "build/$lib${CP_SEP}build/harness" harness.ClosedLoop "results/closed/${name}_$lib.csv" sweep "$THREADS" > /dev/null
  echo "  done: ${name}_$lib"
}

ALL="-Dsweep.t=straight,gentle,scurve,sharp90,oval -Dsweep.s=white_on_dark,black_on_wood -Dsweep.d=0,1 -Dsweep.c=0.3,0.5,1.0,2.0 -Dsweep.x=0,1 -Dsweep.v=1.0"
for lib in ${LIBS:-orig new}; do
  # 1. ideal: 33 ms latency, 60 Hz control, no motor dead zone
  run ideal $lib $ALL -Dsim.lat=0.0333 -Dsim.hz=60 -Dsim.dead=0
  # 2. realistic: 150 ms latency (camera + Bluetooth + motors), 20 Hz Blockly loop, dead zone 2
  run real $lib $ALL -Dsim.lat=0.15 -Dsim.hz=20 -Dsim.dead=2
  # 3. stress: 3x faster robot, 200 ms latency, 15 Hz loop, dead zone 3
  run stress $lib -Dsweep.s=white_on_dark,black_on_wood -Dsweep.d=0,1 -Dsweep.c=0.5,1.0 -Dsweep.x=0,1 -Dsweep.v=3.0 -Dsim.lat=0.2 -Dsim.hz=15 -Dsim.dead=3
  # 4. camera tilt mismatch: real 35 deg, library configured 45 deg (realistic timing)
  run tilt35 $lib -Dsweep.s=black_on_wood -Dsweep.d=0,1 -Dsweep.c=0.5,1.0 -Dsweep.x=0,1 -Dsweep.v=1.0 -Dsim.lat=0.15 -Dsim.hz=20 -Dsim.dead=2 -Dsim.tilt=35
  # 5. heading recovery: robot turned after calibration (realistic timing), straight track
  for yaw in 15 30 -15; do
    run yaw$yaw $lib -Dsweep.t=straight -Dsweep.s=black_on_wood,white_on_dark -Dsweep.c=0.3,0.5,1.0,2.0 -Dsweep.x=0,1 -Dsweep.v=1.0 -Dsim.lat=0.15 -Dsim.hz=20 -Dsim.dead=2 -Dsim.yaw0=$yaw
  done
done
echo "closed-loop campaign finished"
