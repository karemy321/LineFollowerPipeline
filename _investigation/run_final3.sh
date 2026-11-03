#!/usr/bin/env bash
# Wait for the v3 suite (bash PID $1) to end, then the "top to bottom" experiment, 1 GB heap.
cd "$(dirname "$0")"
while tasklist //FI "PID eq $1" 2>/dev/null | grep -q bash; do sleep 30; done
echo "v3 suite (jitter, glare) done $(date +%H:%M)"
XMX=1g TH=10 bash run_experiments.sh v3c3 results/exp_final
echo "experiment done $(date +%H:%M)"
