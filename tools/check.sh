#!/bin/sh
# The test gate: run it before every commit (Git Bash, from anywhere):
#     sh tools/check.sh
# sim build without warnings, all sim tests (analog + USB bridge), TriCore
# compile of app/ + hal/ + Cpu*_Main.c, HAL / throttle-writer checks, the
# expected memory per core (CPU1 RAM must stay below 60 %), the board's FAT32
# writer on PC disk images, Python syntax, the gateway tests.
# Ends with "GATE: OK" or "GATE: FAILED".
cd "$(dirname "$0")/.." || exit 1
# Python: $PYTHON, else MSYS2's (MANUAL chapter 3), else "python" on the PATH
PY=${PYTHON:-/c/msys64/ucrt64/bin/python.exe}
[ -x "$PY" ] || PY=python
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
# every include folder of .cproject (all configurations, each folder once, in ADS order)
INC=$(grep -o 'ProjName}/[^&]*' .cproject | sed 's#}$##; s#ProjName}/#-I#; s#^-I$#-I.#' | awk '!seen[$0]++' | tr '\n' ' ')
# the TriCore gcc of AURIX Development Studio: $TRICORE_BIN, else the newest ADS under C:\Infineon
B=${TRICORE_BIN:-$(ls -d /c/Infineon/AURIX-Studio-*/tools/Compilers/tricore-gcc*/bin 2>/dev/null | sort -V | tail -1)}
[ -x "$B/tricore-elf-gcc.exe" ] || { echo "TriCore gcc NOT FOUND (install ADS or set TRICORE_BIN)"; fail=1; }
for f in app/*.c hal/*.c Cpu0_Main.c Cpu1_Main.c Cpu2_Main.c; do
    msg=$("$B/tricore-elf-gcc.exe" -mcpu=tc39xx -std=c99 -Wall -Wextra -O1 -I. $INC -c "$f" -o "$OUT/$(basename "$f" .c).o" 2>&1)
    if [ $? -ne 0 ] || [ -n "$msg" ]; then echo "TriCore FAIL/WARN $f"; echo "$msg" | head -20; fail=1; else echo "TriCore OK $f"; fi
done
# the big-boat build of core 0 (not linked with the rest, only compiled)
msg=$("$B/tricore-elf-gcc.exe" -mcpu=tc39xx -std=c99 -Wall -Wextra -O1 -DBOAT_VARIANT_BIG -I. $INC -c Cpu0_Main.c -o "$TEMP/cpu0_big.o" 2>&1)
if [ $? -ne 0 ] || [ -n "$msg" ]; then echo "TriCore FAIL/WARN Cpu0_Main.c (big boat)"; echo "$msg" | head -20; fail=1; else echo "TriCore OK Cpu0_Main.c (big boat)"; fi
# the FIRST TEST build (steering only; empty value = same as "#define FIRST_TEST" in Cpu0_Main.c, no redefinition warning)
msg=$("$B/tricore-elf-gcc.exe" -mcpu=tc39xx -std=c99 -Wall -Wextra -O1 -DFIRST_TEST= -I. $INC -c Cpu0_Main.c -o "$TEMP/cpu0_ft.o" 2>&1)
if [ $? -ne 0 ] || [ -n "$msg" ]; then echo "TriCore FAIL/WARN Cpu0_Main.c (FIRST TEST)"; echo "$msg" | head -20; fail=1; else echo "TriCore OK Cpu0_Main.c (FIRST TEST)"; fi
# all HAL functions used by app/ must exist in hal_tc375.c (link check)
"$B/tricore-elf-nm.exe" -u "$OUT"/*.o 2>/dev/null | awk '$1=="U" && $2 ~ /^hal_/ {print $2}' | sort -u > "$TEMP/hal_used.txt"
"$B/tricore-elf-nm.exe" "$OUT/hal_tc375.o" | grep ' T hal_' | grep -o 'hal_[a-z_0-9]*' | sort -u > "$TEMP/hal_def.txt"
missing=$(comm -23 "$TEMP/hal_used.txt" "$TEMP/hal_def.txt")
if [ -n "$missing" ]; then echo "MISSING in hal_tc375.c: $missing"; fail=1; else echo "hal_tc375.c provides every HAL function used"; fi
# the board drivers (hal/tc_*.c): every driver function used is defined
"$B/tricore-elf-nm.exe" -u "$OUT"/*.o 2>/dev/null | awk '$1=="U" && $2 ~ /^(uart|spi|bno|gps|sd|fat32|adc)_/ {print $2}' | sort -u > "$TEMP/drv_used.txt"
"$B/tricore-elf-nm.exe" "$OUT"/tc_*.o | awk '$2=="T" {print $3}' | sort -u > "$TEMP/drv_def.txt"
missing=$(comm -23 "$TEMP/drv_used.txt" "$TEMP/drv_def.txt")
if [ -n "$missing" ]; then echo "MISSING driver functions: $missing"; fail=1; else echo "every board driver function used is defined"; fi
# every iLLD (Ifx*) function used has its .c file present and NOT excluded in .cproject (else the ADS link fails)
"$B/tricore-elf-nm.exe" -u "$OUT"/*.o 2>/dev/null | awk '$1=="U" && $2 ~ /^Ifx/ {print $2}' | sort -u > "$TEMP/ifx_used.txt"
"$PY" tests/acceptance/check_ads_ifx.py --symbols "$TEMP/ifx_used.txt" || fail=1

# the throttle servo is written ONLY by app/servo.c (power limit clamp in the lowest layer)
writers=$(grep -rln --include=*.c "hal_set_throttle_pwm *(" app Cpu0_Main.c Cpu1_Main.c Cpu2_Main.c 2>/dev/null | grep -v "^app/servo.c$")
if [ -n "$writers" ]; then echo "THROTTLE SERVO WRITTEN OUTSIDE servo.c: $writers"; fail=1; else echo "throttle servo written only by app/servo.c"; fi

# ADS compiles every C/C++ file in the project folder: every top-level folder
# that is not board firmware must be excluded in .cproject (all configurations)
"$PY" tests/acceptance/check_ads_excludes.py || fail=1

# the sim's test hooks (raw gear / throttle) never in the board firmware
if "$B/tricore-elf-nm.exe" "$OUT/smallboat.o" 2>/dev/null | grep -q "sb_test_drive"; then
    echo "sb_test_drive (sim test hook) IS IN THE FIRMWARE"; fail=1
else
    echo "no sim test hooks in the firmware"
fi

# the board's monitor trip drives the outputs itself (works with core 0 frozen)
safe=$(sed -n '/^void hal_safe_outputs/,/^}/p' hal/hal_tc375.c)
if echo "$safe" | grep -q "servo_write(THROTTLE_TOM_CH" && echo "$safe" | grep -q "servo_write(GEAR_TOM_CH" &&
   echo "$safe" | grep -q "motor_off()"; then
    echo "hal_safe_outputs (board) writes idle + neutral + motor off itself"
else
    echo "hal_safe_outputs (board) does NOT drive the outputs itself"; fail=1
fi

# expected memory per core from the TriCore objects (CPU1 = safety monitor: max 60 %)
echo "== memory per core (expected, from the objects)"
"$PY" tools/mapreport.py --objects "$OUT" > "$TEMP/mem_report.txt" 2>&1
mrc=$?
sed -n '/RAM PER CORE/,$p' "$TEMP/mem_report.txt"
[ $mrc -ne 0 ] && { echo "MEMORY CHECK FAILED"; fail=1; }

# every Python file of the ground station / tools must at least parse
for f in groundstation/*.py tools/*.py; do
    "$PY" -c "import ast,sys; ast.parse(open(sys.argv[1],encoding='utf-8').read())" "$f" 2>/dev/null || { echo "PYTHON SYNTAX ERROR: $f"; fail=1; }
done
echo "python files parse"
echo "== FAT32 writer of the board (PC disk images)"
"$PY" tests/hal_pc/fat32_check.py > "$TEMP/fat32_tests.txt" 2>&1 || { echo "FAT32 TESTS FAILED"; fail=1; }
grep -E "^\[FAIL\]|FAT32 TESTS|BUILD PROBLEM" "$TEMP/fat32_tests.txt"
echo "== chart tests"
( cd groundstation && "$PY" test_charts.py > "$TEMP/chart_tests.txt" 2>&1 ) || { echo "CHART TESTS FAILED"; fail=1; }
grep -E "^\[FAIL\]|CHART TESTS" "$TEMP/chart_tests.txt"
echo "== ground station language tests (el / en)"
( cd groundstation && PYTHONIOENCODING=utf-8 "$PY" test_lang.py > "$TEMP/lang_tests.txt" 2>&1 ) || { echo "LANGUAGE TESTS FAILED"; fail=1; }
grep -E "^\[FAIL\]|LANGUAGE TESTS" "$TEMP/lang_tests.txt"
echo "== ground station sound + voice tests"
( cd groundstation && PYTHONIOENCODING=utf-8 "$PY" test_sound.py > "$TEMP/sound_tests.txt" 2>&1 ) || { echo "SOUND TESTS FAILED"; fail=1; }
grep -E "^\[(FAIL|N/A )\]|SOUND TESTS" "$TEMP/sound_tests.txt"
echo "== avoidance planner tests"
( cd gateway && "$PY" test_avoid.py > "$TEMP/avoid_tests.txt" 2>&1 ) || { echo "AVOIDANCE TESTS FAILED"; fail=1; }
grep -E "^\[FAIL\]|AVOIDANCE TESTS" "$TEMP/avoid_tests.txt"
echo "== gateway tests"
( cd gateway && "$PY" test_gateway.py > "$TEMP/gw_tests.txt" 2>&1 ) || { echo "GATEWAY TESTS FAILED"; fail=1; }
grep -E "^\[FAIL\]|GATEWAY TESTS" "$TEMP/gw_tests.txt"
[ $fail -eq 0 ] && echo "GATE: OK" || echo "GATE: FAILED"
exit $fail
