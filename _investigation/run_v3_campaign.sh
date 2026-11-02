#!/usr/bin/env bash
# Full v2-vs-v3 campaign (Run from _investigation/ after build_v3.sh). Results in results/final/.
#  1. disturbance matrix (280 cases)                         matrix_<lib>.csv
#  2. closed-loop regimes over 11 tracks                     ideal/real/stress/tilt35_<lib>.csv
#  3. start at an angle / sideways offset                    start_<lib>.csv
#  4. closed-loop lap / oval / straight with disturbances    dist_<name>_<lib>.csv (+ clean reference)
cd "$(dirname "$0")"
J="${JAVA_HOME:-/c/Program Files/Android/Android Studio/jbr}/bin/java"
O=results/final; mkdir -p $O
TR=straight,gentle,gentleR,scurve,scurveR,sharp90,sharp90L,tightL,tightR,oval,lap
cl() { local out=$1 lib=$2; shift 2; "$J" -Xmx14g -Dlib.tag=$lib "$@" -cp "build/$lib;build/harness" harness.ClosedLoop $O/$out sweep 18 > /dev/null; }
for lib in ${LIBS:-v2 v3}; do
  "$J" -Xmx14g -Dlib.tag=$lib -cp "build/$lib;build/harness" harness.Matrix $O/matrix_$lib.csv 18 > /dev/null; echo "matrix $lib"
  ALL="-Dsweep.t=$TR -Dsweep.s=black_on_wood,white_on_dark -Dsweep.d=0,1 -Dsweep.c=0.5,1.0 -Dsweep.x=0,1"
  cl ideal_$lib.csv  $lib $ALL -Dsweep.v=1.0 -Dsim.lat=0.0333 -Dsim.hz=60 -Dsim.dead=0
  cl real_$lib.csv   $lib $ALL -Dsweep.v=1.0 -Dsim.lat=0.15 -Dsim.hz=20 -Dsim.dead=2
  cl stress_$lib.csv $lib $ALL -Dsweep.v=3.0 -Dsim.lat=0.2 -Dsim.hz=15 -Dsim.dead=3
  cl tilt35_$lib.csv $lib $ALL -Dsweep.v=1.0 -Dsim.lat=0.15 -Dsim.hz=20 -Dsim.dead=2 -Dsim.tilt=35
  echo "regimes $lib"
  for st in "yaw15:-Dsim.yaw0=15" "yaw30:-Dsim.yaw0=30" "yawm15:-Dsim.yaw0=-15" "lat3:-Dsim.lat0=0.03" "latm3:-Dsim.lat0=-0.03"; do
    cl start_${st%%:*}_$lib.csv $lib -Dsweep.t=straight,gentle -Dsweep.s=black_on_wood,white_on_dark -Dsweep.d=0 -Dsweep.c=0.5,1.0 -Dsweep.x=0,1 -Dsweep.v=1.0 -Dsim.lat=0.15 -Dsim.hz=20 -Dsim.dead=2 ${st#*:}
  done
  echo "start $lib"
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
    if [ -z "$spec" ]; then cl dist_${name}_$lib.csv $lib -Dsweep.t=lap,oval $LAP
    else cl dist_${name}_$lib.csv $lib -Dsweep.t=lap,oval $LAP -Dsim.disturb="$spec"; fi
  done
  cl dist_edge_$lib.csv $lib -Dsweep.t=oval $LAP -Dsim.disturb="edge:s=1.0,off=0.45"
  cl dist_edgeclean_$lib.csv $lib -Dsweep.t=oval $LAP
  echo "disturbances $lib"
done
echo "campaign finished"
