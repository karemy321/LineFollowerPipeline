#!/usr/bin/env bash
# wobble_run2.sh <lib dir> <tag> : like wobble_run.sh, from 4 starts (lat0 0/1 cm x yaw0 0/2 deg); JOPT = extra java options
cd "$(dirname "$0")"
J="/c/Program Files/Android/Android Studio/jbr/bin/java"
lib=$1; tag=$2; ls=""; [[ $lib != v2 ]] && ls="-Dlib.set=frameIntervalMs=16.667${LIBSET:+,$LIBSET}"
ALL="-Dsweep.t=straight,gentle,scurve,sharp90,oval -Dsweep.s=white_on_dark,black_on_wood -Dsweep.d=0 -Dsweep.c=1.0 -Dsweep.x=0,1"
for st in "0 0" "0.01 0" "0 2" "0.01 2"; do set -- $st
  "$J" -Dlib.tag=$tag $ls $JOPT -Dsim.lat0=$1 -Dsim.yaw0=$2 $ALL -Dsweep.v=1.0 -Dsim.lat=0.15 -Dsim.hz=20 -Dsim.dead=2 -cp "build/$lib;build/harness_v3" harness.ClosedLoop "results/wobble/real_${tag}_$1_$2.csv" sweep 18 > /dev/null
  "$J" -Dlib.tag=$tag $ls $JOPT -Dsim.lat0=$1 -Dsim.yaw0=$2 $ALL -Dsweep.v=3.0 -Dsim.lat=0.2 -Dsim.hz=15 -Dsim.dead=3 -cp "build/$lib;build/harness_v3" harness.ClosedLoop "results/wobble/stress_${tag}_$1_$2.csv" sweep 18 > /dev/null
done
for reg in real stress; do head -1 "results/wobble/${reg}_${tag}_0_0.csv" > "results/wobble/${reg}_${tag}4.csv"; for f in results/wobble/${reg}_${tag}_0*.csv; do tail -n +2 "$f"; done >> "results/wobble/${reg}_${tag}4.csv"; done
echo done $tag
