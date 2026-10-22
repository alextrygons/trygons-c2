#!/bin/sh
# The test gate: run it before every commit (Git Bash, from anywhere):
#     sh tools/check.sh
# sim build without warnings, all sim tests (analog + USB bridge), TriCore
# compile of app/ + hal/ + Cpu*_Main.c, HAL / throttle-writer checks, the
# expected memory per core (CPU1 RAM must stay below 60 %), Python syntax,
# the gateway tests. Ends with "GATE: OK" or "GATE: FAILED".
cd /c/trygons/TRYGONS_C2 || exit 1
fail=0

echo "== sim build"
out=$(cmd //c "sim\\build_sim.bat" 2>&1)
echo "$out" | grep -v "^Run it\|^Compiling"
echo "$out" | grep -qi "warning\|error\|FAILED" && { echo "BUILD HAS WARNINGS/ERRORS"; fail=1; }

echo "== tests"
cd sim
if ./autopilot_sim.exe all > "$TEMP/all_tests.txt" 2>&1; then
    echo "all tests: exit 0"
else
    echo "all tests: exit $? (FAIL)"; fail=1
fi
grep -E "^\[(PASS|FAIL)\]" "$TEMP/all_tests.txt"
tail -1 "$TEMP/all_tests.txt"
if ./autopilot_sim.exe all --bridge > "$TEMP/all_bridge.txt" 2>&1; then
    echo "all tests through the USB bridge: exit 0"
else
    echo "all tests through the USB bridge: FAIL"; fail=1
fi
grep -E "^\[(FAIL|N/A )\]" "$TEMP/all_bridge.txt"
tail -1 "$TEMP/all_bridge.txt"
cd ..

echo "== TriCore compile"
OUT="$TEMP/tricore_obj"; mkdir -p "$OUT"
INC=$(sed -n 23,75p .cproject | grep -o 'ProjName}/[^&]*' | sed 's#}$##; s#ProjName}/#-I#; s#^-I$#-I.#' | tr '\n' ' ')
B=/c/Infineon/AURIX-Studio-1.10.40/tools/Compilers/tricore-gcc11/bin
for f in app/*.c hal/hal_tc375.c Cpu0_Main.c Cpu1_Main.c Cpu2_Main.c; do
    msg=$("$B/tricore-elf-gcc.exe" -mcpu=tc39xx -std=c99 -Wall -Wextra -O1 -I. $INC -c "$f" -o "$OUT/$(basename "$f" .c).o" 2>&1)
    if [ $? -ne 0 ] || [ -n "$msg" ]; then echo "TriCore FAIL/WARN $f"; echo "$msg" | head -20; fail=1; else echo "TriCore OK $f"; fi
done
# all HAL functions used by app/ must exist in hal_tc375.c (link check)
"$B/tricore-elf-nm.exe" -u "$OUT"/*.o 2>/dev/null | awk '$1=="U" && $2 ~ /^hal_/ {print $2}' | sort -u > "$TEMP/hal_used.txt"
"$B/tricore-elf-nm.exe" "$OUT/hal_tc375.o" | grep ' T hal_' | grep -o 'hal_[a-z_0-9]*' | sort -u > "$TEMP/hal_def.txt"
missing=$(comm -23 "$TEMP/hal_used.txt" "$TEMP/hal_def.txt")
if [ -n "$missing" ]; then echo "MISSING in hal_tc375.c: $missing"; fail=1; else echo "hal_tc375.c provides every HAL function used"; fi

# the throttle servo is written ONLY by app/servo.c (power limit clamp in the lowest layer)
writers=$(grep -rln --include=*.c "hal_set_throttle_pwm *(" app Cpu0_Main.c Cpu1_Main.c Cpu2_Main.c 2>/dev/null | grep -v "^app/servo.c$")
if [ -n "$writers" ]; then echo "THROTTLE SERVO WRITTEN OUTSIDE servo.c: $writers"; fail=1; else echo "throttle servo written only by app/servo.c"; fi

# expected memory per core from the TriCore objects (CPU1 = safety monitor: max 60 %)
echo "== memory per core (expected, from the objects)"
/c/msys64/ucrt64/bin/python.exe tools/mapreport.py --objects "$OUT" > "$TEMP/mem_report.txt" 2>&1
mrc=$?
sed -n '/RAM PER CORE/,$p' "$TEMP/mem_report.txt"
[ $mrc -ne 0 ] && { echo "MEMORY CHECK FAILED"; fail=1; }

# every Python file of the ground station / tools must at least parse
for f in groundstation/*.py tools/*.py; do
    /c/msys64/ucrt64/bin/python.exe -c "import ast,sys; ast.parse(open(sys.argv[1],encoding='utf-8').read())" "$f" 2>/dev/null || { echo "PYTHON SYNTAX ERROR: $f"; fail=1; }
done
echo "python files parse"
echo "== chart tests"
( cd groundstation && /c/msys64/ucrt64/bin/python.exe test_charts.py > "$TEMP/chart_tests.txt" 2>&1 ) || { echo "CHART TESTS FAILED"; fail=1; }
grep -E "^\[FAIL\]|CHART TESTS" "$TEMP/chart_tests.txt"
echo "== gateway tests"
( cd gateway && /c/msys64/ucrt64/bin/python.exe test_gateway.py > "$TEMP/gw_tests.txt" 2>&1 ) || { echo "GATEWAY TESTS FAILED"; fail=1; }
grep -E "^\[FAIL\]|GATEWAY TESTS" "$TEMP/gw_tests.txt"
[ $fail -eq 0 ] && echo "GATE: OK" || echo "GATE: FAILED"
exit $fail
