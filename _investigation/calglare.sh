#!/usr/bin/env bash
# Calibration under realistic glare, all colour pairs (prints learned line colour and ROI). Run from _investigation/.
J="${JAVA_HOME:-/c/Program Files/Android/Android Studio/jbr}/bin/java"
for sc in black_on_wood white_on_dark black_on_white red_on_green; do
  for g in "clean:" "near:glareReal:fwd=0.27,lat=0.010,rf=0.07,rl=0.05,i=1.6" "mid:glareReal:fwd=0.42,lat=0.010,rf=0.09,rl=0.06,i=1.6" "far:glareReal:fwd=0.60,lat=0.010,rf=0.12,rl=0.07,i=1.6" "big:glareReal:fwd=0.42,lat=0.015,rf=0.18,rl=0.09,i=2.0" "beside:glareReal:fwd=0.40,lat=0.050,rf=0.10,rl=0.04,i=1.8" "streak:glareReal:fwd=0.45,lat=0.000,rf=0.30,rl=0.025,i=1.8" "weak:glareReal:fwd=0.40,lat=0.010,rf=0.10,rl=0.06,i=0.8"; do
    out=$("$J" -Xmx700m -Dlib.tag=v3 -Dframes=120 -cp "build/${LIB:-v3};build/harness" harness.CalProbe ${TRACK:-straight3} $sc 0.05 ${g#*:} | grep -E 'ok=true|line model|roiTop' | tr '\n' ' ')
    ok=$(echo "$out" | grep -c "ok=true"); lm=$(echo "$out" | sed -n 's/.*line model \[\([0-9.-]*\), \([0-9.-]*\), \([0-9.-]*\)\].*/\1 \2 \3/p' | awk '{printf "%.0f/%.0f/%.0f",$1,$2,$3}')
    roi=$(echo "$out" | sed -n 's/.*roiTop=\([0-9]*\) roiBot=\([0-9]*\) tilt=\([0-9.]*\) lineW=\([0-9.]*\).*/\1-\2 tilt \3 w \4/p')
    printf "%-15s %-7s ok=%d line=%-14s roi=%s\n" $sc ${g%%:*} $ok "$lm" "$roi"
  done
done
