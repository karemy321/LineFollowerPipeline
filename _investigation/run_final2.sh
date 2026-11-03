#!/usr/bin/env bash
# Rest of run_final.sh after the matrix (Run from _investigation/). Waits for a running
# matrix process (PID $1) to end first.
cd "$(dirname "$0")"
J="${JAVA_HOME:-/c/Program Files/Android/Android Studio/jbr}/bin/java"
while tasklist //FI "PID eq $1" 2>/dev/null | grep -q java; do sleep 20; done
echo "matrix done $(date +%H:%M)"
LIB=v3c3
TH=14 bash run_suite.sh $LIB results/suite/$LIB "regimes start dist jitter glare"; echo "v3 suite done $(date +%H:%M)"
TH=14 bash run_suite.sh v2 results/suite/v2 "jitter glare"
TR=straight,gentle,gentleR,scurve,scurveR,sharp90,sharp90L,tightL,tightR,oval,lap
"$J" -Xmx2g -Dlib.tag=v2 -Dsweep.t=$TR -Dsweep.s=black_on_wood,white_on_dark -Dsweep.d=0,1 -Dsweep.c=0.5,1.0 -Dsweep.x=0,1 -Dsweep.v=1.0 -Dsim.lat=0.15 -Dsim.hz=20 -Dsim.dead=2 -Dsim.fps=30 -Dsim.vib=1 -cp "build/v2;build/harness" harness.ClosedLoop results/suite/v2/phone30.csv sweep 14 > /dev/null
echo "v2 parts done $(date +%H:%M)"
TH=14 bash run_experiments.sh $LIB results/exp_final
echo "all done $(date +%H:%M)"
