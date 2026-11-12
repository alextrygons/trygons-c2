#!/bin/sh
# Step HX2: the harbour mutation checks (docs/harbour_design.md 9.2) - each AURIX-side protection removed in turn
# (app/harbour.c / smallboat.c HARB_MUTATE, never set in a real build: tools/check.sh greps it) must make its check FAIL:
#   1 the speed cap removed        -> storm I86
#   2 the corridor HOLD removed    -> storm I89
#   3 the lease HOLD removed       -> storm I92
#   4 the turn-rate slew removed   -> sim HB6
#   5 the lost-link grace removed  -> storm I85
# Usage (Git Bash, from anywhere):  sh tests/acceptance/storm/mutate_harbour.sh [episodes]   (default 200)
# Exit 0 = every mutation was caught. Builds into test_results/mut/ (never the real storm.exe / autopilot_sim.exe).
cd "$(dirname "$0")/../../.." || exit 1
EP=${1:-200}
OUT=test_results/mut
mkdir -p "$OUT"
SRCS="$(ls app/*.c) $(ls sim/*.c | grep -v sim_main.c) hal/tc_nv.c hal/tc_modbus.c hal/dronecan.c hal/can_act.c hal/can_safe.c hal/can_relay.c hal/tc_pulse.c hal/tc_stack.c hal/libcanard/canard.c"
bad=0
for m in 1 2 3 5; do
    case $m in 1) inv=I86;; 2) inv=I89;; 3) inv=I92;; 5) inv=I85;; esac
    gcc -std=c99 -O2 -DCANARD_64_BIT=1 -DHARB_MUTATE=$m -Iapp -Ihal -Isim $SRCS tests/acceptance/storm/storm.c \
        -o "$OUT/storm_mut$m.exe" -lm -lws2_32 || { echo "mutation $m: BUILD FAILED"; bad=1; continue; }
    mkdir -p "$OUT/run$m"
    ( cd "$OUT/run$m" && timeout 2400 ../storm_mut$m.exe "$EP" 1 --first-fail > out.txt 2>&1 )
    if grep -q "^FAIL seed [0-9]* $inv " "$OUT/run$m/out.txt"; then
        echo "mutation $m caught by $inv: $(grep -m1 "^FAIL seed [0-9]* $inv " "$OUT/run$m/out.txt" | cut -c1-160)"
    else
        echo "mutation $m NOT CAUGHT by $inv"; bad=1
    fi
done
# 4: the slew - the sim test HB6
gcc -std=c99 -O2 -DCANARD_64_BIT=1 -DHARB_MUTATE=4 -Iapp -Ihal $(ls app/*.c) $(ls sim/*.c) hal/tc_nv.c hal/tc_modbus.c \
    hal/dronecan.c hal/can_act.c hal/can_safe.c hal/can_relay.c hal/tc_pulse.c hal/tc_stack.c hal/libcanard/canard.c \
    -o "$OUT/sim_mut4.exe" -lm -lws2_32 || { echo "mutation 4: BUILD FAILED"; bad=1; }
mkdir -p "$OUT/run4"
( cd "$OUT/run4" && timeout 900 ../sim_mut4.exe hb > out.txt 2>&1 )
line4=$(grep -m1 -F "[FAIL] HB6 " "$OUT/run4/out.txt" | cut -c1-160)
if [ -n "$line4" ]; then echo "mutation 4 caught by HB6: $line4"
else echo "mutation 4 NOT CAUGHT by HB6"; bad=1; fi
[ $bad = 0 ] && echo "HARBOUR MUTATIONS: ALL CAUGHT" || echo "HARBOUR MUTATIONS: SOME NOT CAUGHT"
exit $bad
