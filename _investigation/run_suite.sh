#!/usr/bin/env bash
# Memory-safe evaluation suite for one library build (Run from _investigation/).
#   bash run_suite.sh <lib-dir-under-build> <out-dir> [parts]
# parts (default all): matrix regimes start dist jitter glare
# Each Java process is capped at 2 GB and runs alone (the 14 GB runs of the first
# campaign were killed by the system for low memory).
cd "$(dirname "$0")"
J="${JAVA_HOME:-/c/Program Files/Android/Android Studio/jbr}/bin/java"
LIB=$1; O=$2; PARTS=${3:-"matrix regimes start dist jitter glare"}
TH=${TH:-10}
mkdir -p $O
X="-Xmx2g -Dlib.tag=$LIB"
CP="build/$LIB;build/harness"
TR=straight,gentle,gentleR,scurve,scurveR,sharp90,sharp90L,tightL,tightR,oval,lap
cl() { local out=$1; shift; "$J" $X "$@" -cp "$CP" harness.ClosedLoop $O/$out sweep $TH > /dev/null; }
for p in $PARTS; do case $p in
  matrix) "$J" $X -cp "$CP" harness.Matrix $O/matrix.csv $TH > /dev/null; echo "matrix done" ;;
  regimes)
    ALL="-Dsweep.t=$TR -Dsweep.s=black_on_wood,white_on_dark -Dsweep.d=0,1 -Dsweep.c=0.5,1.0 -Dsweep.x=0,1"
    cl ideal.csv  $ALL -Dsweep.v=1.0 -Dsim.lat=0.0333 -Dsim.hz=60 -Dsim.dead=0
    cl real.csv   $ALL -Dsweep.v=1.0 -Dsim.lat=0.15 -Dsim.hz=20 -Dsim.dead=2
    cl stress.csv $ALL -Dsweep.v=3.0 -Dsim.lat=0.2 -Dsim.hz=15 -Dsim.dead=3
    cl tilt35.csv $ALL -Dsweep.v=1.0 -Dsim.lat=0.15 -Dsim.hz=20 -Dsim.dead=2 -Dsim.tilt=35
    cl phone30.csv $ALL -Dsweep.v=1.0 -Dsim.lat=0.15 -Dsim.hz=20 -Dsim.dead=2 -Dsim.fps=30 -Dsim.vib=1
    echo "regimes done" ;;
  start)
    for st in "yaw15:-Dsim.yaw0=15" "yaw30:-Dsim.yaw0=30" "yawm15:-Dsim.yaw0=-15" "lat3:-Dsim.lat0=0.03" "latm3:-Dsim.lat0=-0.03"; do
      cl start_${st%%:*}.csv -Dsweep.t=straight,gentle -Dsweep.s=black_on_wood,white_on_dark -Dsweep.d=0 -Dsweep.c=0.5,1.0 -Dsweep.x=0,1 -Dsweep.v=1.0 -Dsim.lat=0.15 -Dsim.hz=20 -Dsim.dead=2 ${st#*:}
    done; echo "start done" ;;
  dist)
    LAP="-Dsweep.s=black_on_wood,white_on_dark -Dsweep.d=0 -Dsweep.c=1.0 -Dsweep.x=0,1 -Dsweep.v=1.0 -Dsim.lat=0.15 -Dsim.hz=20 -Dsim.dead=2"
    for d in \
      "clean:" \
      "shadowRobot:shadowRobot:fwd=0.24,lat=0.03,a=0.09,b=0.06,k=0.4" \
      "glareStraight:glare:ref=0.3,fwd=0.42,lat=0.015,rf=0.16,rl=0.08,i=0.95" \
      "glareCurve:glare:ref=1.6,fwd=0.42,lat=0.015,rf=0.12,rl=0.07,i=0.95" \
      "shadowRoom:shadowRoom:s=0.9,lat=0.02,a=0.10,b=0.2,k=0.35|shadowRoom:s=2.0,lat=0.0,a=0.08,b=0.2,k=0.35" \
      "uneven:uneven:s=0.8,lo=0.35,hi=1.45,w=0.08" \
      "patch:patch:s=0.9,lat=0,a=0.12,b=0.16,rgb=5A6E96|patch:s=2.2,lat=0,a=0.1,b=0.16,rgb=C87828" \
      "cover:cover:s=0.9,lat=0,a=0.03,b=0.05,rgb=8C8C8C|cover:s=2.3,lat=0,a=0.03,b=0.05,rgb=8C8C8C" \
      "stripe:stripe:s=0.7,len=0.6,off=0.032,wid=0.004|stripe:s=3.4,len=0.5,off=-0.032,wid=0.004" \
      "cast:cast:s=0.9,lat=0,r=0.15,du=-20,dv=30|cast:s=2.1,lat=0,r=0.15,du=25,dv=-15" ; do
      name=${d%%:*}; spec=${d#*:}
      if [ -z "$spec" ]; then cl dist_${name}.csv -Dsweep.t=lap,oval $LAP
      else cl dist_${name}.csv -Dsweep.t=lap,oval $LAP -Dsim.disturb="$spec"; fi
    done
    cl dist_edge.csv -Dsweep.t=oval $LAP -Dsim.disturb="edge:s=1.0,off=0.45"
    cl dist_edgeclean.csv -Dsweep.t=oval $LAP
    echo "dist done" ;;
  jitter) "$J" $X -cp "$CP" harness.Jitter $O/jitter.csv $TH > /dev/null; echo "jitter done" ;;
  glare)
    B="-Dsweep.t=lap,oval,scurve,gentle -Dsweep.s=black_on_wood,white_on_dark,black_on_white,red_on_green -Dsweep.d=0 -Dsweep.c=1.0 -Dsweep.x=0,1 -Dsweep.v=1.0 -Dsim.fps=30 -Dsim.lat=0.15 -Dsim.hz=20 -Dsim.dead=2 -Dsim.vib=1"
    for d in "clean:" \
             "glareMid:glareReal:ref=0.3,fwd=0.42,lat=0.010,rf=0.09,rl=0.06,i=1.6" \
             "glareNear:glareReal:ref=0.3,fwd=0.27,lat=0.010,rf=0.07,rl=0.05,i=1.6" \
             "glareBig:glareReal:ref=0.3,fwd=0.42,lat=0.015,rf=0.18,rl=0.09,i=2.0" \
             "glareStreak:glareReal:ref=0.3,fwd=0.45,lat=0.0,rf=0.30,rl=0.025,i=1.8" ; do
      name=${d%%:*}; spec=${d#*:}
      if [ -z "$spec" ]; then cl glare_${name}.csv $B
      else cl glare_${name}.csv $B -Dsim.disturb="$spec"; cl glarecal_${name}.csv $B -Dsim.disturb="$spec" -Dsim.calibClean=1; fi
    done
    echo "glare done" ;;
esac; done
