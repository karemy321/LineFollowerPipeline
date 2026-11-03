#!/usr/bin/env bash
# "Top to bottom" experiment: trace direction and floor vs image space, plus memory ablation.
# Each variant: full disturbance matrix + realistic closed-loop set + realistic-glare jitter.
# Memory-safe: one Java process at a time, 2 GB each (the first run with 14 GB was
# stopped by the system for low memory). Run from _investigation/.
#   bash run_experiments.sh [lib-dir under build/, default v3] [out-dir, default results/exp]
cd "$(dirname "$0")"
J="${JAVA_HOME:-/c/Program Files/Android/Android Studio/jbr}/bin/java"
LIB=${1:-v3}; O=${2:-results/exp}; TH=${TH:-12}
mkdir -p $O
T=straight,gentle,gentleR,scurve,scurveR,sharp90,sharp90L,tightL,tightR,oval,lap
for v in "both:pathTraceDirection=2" "bottomup:pathTraceDirection=0" "topdown:pathTraceDirection=1" "image:pathFloorSpace=false" "nomemory:pathMemory=false"; do
  name=${v%%:*}; set=${v#*:}
  X="-Xmx2g -Dlib.tag=v3 -Dlib.set=$set"
  "$J" $X -cp "build/$LIB;build/harness" harness.Matrix $O/matrix_$name.csv $TH > /dev/null
  "$J" $X -Dsweep.t=$T -Dsweep.s=black_on_wood,white_on_dark -Dsweep.d=0,1 -Dsweep.c=0.5,1.0 -Dsweep.x=0,1 -Dsweep.v=1.0 -Dsim.lat=0.15 -Dsim.hz=20 -Dsim.dead=2 -cp "build/$LIB;build/harness" harness.ClosedLoop $O/real_$name.csv sweep $TH > /dev/null
  "$J" $X -cp "build/$LIB;build/harness" harness.Jitter $O/jitter_$name.csv $TH > /dev/null
  echo "done $name $(date +%H:%M)"
done
