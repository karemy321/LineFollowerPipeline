#!/usr/bin/env bash
# Closed-loop laps with realistic glare at the phone's 30 fps, with vibration (Run from _investigation/).
# Usage: LIBS="v2 v3" bash run_glare_cl.sh <outdir>
cd "$(dirname "$0")"
J="${JAVA_HOME:-/c/Program Files/Android/Android Studio/jbr}/bin/java"
O=${1:-results/glcl}; mkdir -p $O
B="-Dsweep.s=black_on_wood,white_on_dark,black_on_white,red_on_green -Dsweep.d=0 -Dsweep.c=1.0 -Dsweep.x=0,1 -Dsweep.v=1.0 -Dsim.fps=30 -Dsim.lat=0.15 -Dsim.hz=20 -Dsim.dead=2 -Dsim.vib=1"
for lib in ${LIBS:-v2 v3}; do
  for d in "clean:" \
           "glareMid:glareReal:ref=0.3,fwd=0.42,lat=0.010,rf=0.09,rl=0.06,i=1.6" \
           "glareNear:glareReal:ref=0.3,fwd=0.27,lat=0.010,rf=0.07,rl=0.05,i=1.6" \
           "glareBig:glareReal:ref=0.3,fwd=0.42,lat=0.015,rf=0.18,rl=0.09,i=2.0" \
           "glareStreak:glareReal:ref=0.3,fwd=0.45,lat=0.0,rf=0.30,rl=0.025,i=1.8" ; do
    name=${d%%:*}; spec=${d#*:}
    if [ -z "$spec" ]; then "$J" -Xmx2g -Dlib.tag=$lib $B -Dsweep.t=lap,oval,scurve,gentle -cp "build/$lib;build/harness" harness.ClosedLoop $O/${name}_$lib.csv sweep 10 > /dev/null
    else "$J" -Xmx2g -Dlib.tag=$lib $B -Dsweep.t=lap,oval,scurve,gentle -Dsim.disturb="$spec" -Dsim.calibClean=1 -cp "build/$lib;build/harness" harness.ClosedLoop $O/${name}_$lib.csv sweep 10 > /dev/null; fi
    printf "%-5s %-12s " $lib $name; awk -f clsum.awk $O/${name}_$lib.csv
  done
done
