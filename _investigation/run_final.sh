#!/usr/bin/env bash
# Final measurements (Run from _investigation/): the full suite on the final v3 build, the
# v2 parts that the first campaign did not have (30 fps regime, jitter, realistic glare),
# then the "top to bottom" experiment on the final code. One Java process at a time.
cd "$(dirname "$0")"
J="${JAVA_HOME:-/c/Program Files/Android/Android Studio/jbr}/bin/java"
LIB=${1:-v3c3}
date
TH=14 bash run_suite.sh $LIB results/suite/$LIB "matrix regimes start dist jitter glare"; echo "v3 suite done $(date +%H:%M)"
TH=14 bash run_suite.sh v2 results/suite/v2 "jitter glare"
TR=straight,gentle,gentleR,scurve,scurveR,sharp90,sharp90L,tightL,tightR,oval,lap
"$J" -Xmx2g -Dlib.tag=v2 -Dsweep.t=$TR -Dsweep.s=black_on_wood,white_on_dark -Dsweep.d=0,1 -Dsweep.c=0.5,1.0 -Dsweep.x=0,1 -Dsweep.v=1.0 -Dsim.lat=0.15 -Dsim.hz=20 -Dsim.dead=2 -Dsim.fps=30 -Dsim.vib=1 -cp "build/v2;build/harness" harness.ClosedLoop results/suite/v2/phone30.csv sweep 14 > /dev/null
echo "v2 parts done $(date +%H:%M)"
TH=14 bash run_experiments.sh $LIB results/exp_final
echo "all done $(date +%H:%M)"
