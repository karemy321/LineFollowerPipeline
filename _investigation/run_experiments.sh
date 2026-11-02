#!/usr/bin/env bash
# "Top to bottom" experiment: trace direction and floor vs image space, plus memory ablation.
# Each variant: full disturbance matrix + realistic closed-loop set. Run from _investigation/ after build_v3.sh.
cd "$(dirname "$0")"
J="${JAVA_HOME:-/c/Program Files/Android/Android Studio/jbr}/bin/java"
T=straight,gentle,gentleR,scurve,scurveR,sharp90,sharp90L,tightL,tightR,oval,lap
for v in "bottomup:pathTraceDirection=0" "topdown:pathTraceDirection=1" "both:pathTraceDirection=2" "image:pathFloorSpace=false" "nomemory:pathMemory=false"; do
  name=${v%%:*}; set=${v#*:}
  "$J" -Xmx14g -Dlib.tag=v3 -Dlib.set=$set -cp "build/v3;build/harness" harness.Matrix results/exp/matrix_$name.csv 18 > /dev/null
  "$J" -Xmx14g -Dlib.tag=v3 -Dlib.set=$set -Dsweep.t=$T -Dsweep.s=black_on_wood,white_on_dark -Dsweep.d=0,1 -Dsweep.c=0.5,1.0 -Dsweep.x=0,1 -Dsweep.v=1.0 -Dsim.lat=0.15 -Dsim.hz=20 -Dsim.dead=2 -cp "build/v3;build/harness" harness.ClosedLoop results/exp/real_$name.csv sweep 18 > /dev/null
  echo "done $name"
done
