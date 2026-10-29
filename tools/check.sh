#!/bin/sh
# The test gate: run it before every commit (Git Bash, from anywhere):
#     sh tools/check.sh
# sim build without warnings, all sim tests (analog + USB bridge + DroneCAN servos worst-case / fast), TriCore
# compile of app/ + hal/ + Cpu*_Main.c, HAL / throttle-writer / relay-writer / thruster-writer checks, the
# expected memory per core (CPU1 RAM must stay below 60 %), the board's FAT32
# writer on PC disk images, the board's data-flash record log on a PC flash
# image, the DroneCAN layer (dronecan.c + libcanard) against an independent
# encoder, Python syntax, the gateway tests.
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
# step 11e: every test again on the DroneCAN servos - the worst-case (throttle 6 s, steering 12 s lock to lock)
# and the fast (2 s / 3 s) servos; the PWM-motor-only tests are N/A there
for v in can can-fast; do
    if ./autopilot_sim.exe all --$v > "$TEMP/all_$v.txt" 2>&1; then
        echo "all tests on the DroneCAN servos (--$v): exit 0"
    else
        echo "all tests on the DroneCAN servos (--$v): FAIL"; fail=1
    fi
    grep -E "^\[FAIL\]" "$TEMP/all_$v.txt"
    tail -1 "$TEMP/all_$v.txt"
done
cd ..

echo "== TriCore compile"
OUT="$TEMP/tricore_obj"; mkdir -p "$OUT"
# every include folder of .cproject (all configurations, each folder once, in ADS order)
INC=$(grep -o 'ProjName}/[^&]*' .cproject | sed 's#}$##; s#ProjName}/#-I#; s#^-I$#-I.#' | awk '!seen[$0]++' | tr '\n' ' ')
# the TriCore gcc of AURIX Development Studio: $TRICORE_BIN, else the newest ADS under C:\Infineon
B=${TRICORE_BIN:-$(ls -d /c/Infineon/AURIX-Studio-*/tools/Compilers/tricore-gcc*/bin 2>/dev/null | sort -V | tail -1)}
[ -x "$B/tricore-elf-gcc.exe" ] || { echo "TriCore gcc NOT FOUND (install ADS or set TRICORE_BIN)"; fail=1; }
for f in app/*.c hal/*.c hal/libcanard/*.c Cpu0_Main.c Cpu1_Main.c Cpu2_Main.c; do
    msg=$("$B/tricore-elf-gcc.exe" -mcpu=tc39xx -std=c99 -Wall -Wextra -O1 -I. $INC -c "$f" -o "$OUT/$(basename "$f" .c).o" 2>&1)
    if [ $? -ne 0 ] || [ -n "$msg" ]; then echo "TriCore FAIL/WARN $f"; echo "$msg" | head -20; fail=1; else echo "TriCore OK $f"; fi
done
# the big-boat build of core 0 (not linked with the rest, only compiled)
msg=$("$B/tricore-elf-gcc.exe" -mcpu=tc39xx -std=c99 -Wall -Wextra -O1 -DBOAT_VARIANT_BIG -I. $INC -c Cpu0_Main.c -o "$TEMP/cpu0_big.o" 2>&1)
if [ $? -ne 0 ] || [ -n "$msg" ]; then echo "TriCore FAIL/WARN Cpu0_Main.c (big boat)"; echo "$msg" | head -20; fail=1; else echo "TriCore OK Cpu0_Main.c (big boat)"; fi
# the FIRST TEST build (steering only; empty value = same as "#define FIRST_TEST" in Cpu0_Main.c, no redefinition warning)
msg=$("$B/tricore-elf-gcc.exe" -mcpu=tc39xx -std=c99 -Wall -Wextra -O1 -DFIRST_TEST= -I. $INC -c Cpu0_Main.c -o "$TEMP/cpu0_ft.o" 2>&1)
if [ $? -ne 0 ] || [ -n "$msg" ]; then echo "TriCore FAIL/WARN Cpu0_Main.c (FIRST TEST)"; echo "$msg" | head -20; fail=1; else echo "TriCore OK Cpu0_Main.c (FIRST TEST)"; fi
# step 11f: the board paths behind the hardware flags (the flags stay 0 in hal_pins.h until the owner confirms):
# the DroneCAN relay board + the DroneCAN servos (RELAY_BUS_DRONECAN), and the RS485 Modbus relay board fitted
OUTV="$TEMP/tricore_obj_variants"; mkdir -p "$OUTV"
for v in "can:-DHAL_CAN_FITTED=1 -DHAL_CAN_SERVOS=1 -DHAL_RELAY_FITTED=1 -DRELAY_BUS=2" "modbus:-DHAL_RELAY_FITTED=1 -DRELAY_BUS=1"; do
    name=${v%%:*}; defs=${v#*:}
    msg=$("$B/tricore-elf-gcc.exe" -mcpu=tc39xx -std=c99 -Wall -Wextra -O1 $defs -I. $INC -c hal/hal_tc375.c -o "$OUTV/hal_tc375_$name.o" 2>&1)
    if [ $? -ne 0 ] || [ -n "$msg" ]; then echo "TriCore FAIL/WARN hal_tc375.c (relay board on $name, flags 1)"; echo "$msg" | head -20; fail=1; else echo "TriCore OK hal_tc375.c (relay board on $name, flags 1)"; fi
done
# step 13: the analog joystick + paddlewheel board paths (HAL_JOY_FITTED / HAL_PADDLE_FITTED stay 0 until wired)
for f in hal_tc375 tc_adc; do
    msg=$("$B/tricore-elf-gcc.exe" -mcpu=tc39xx -std=c99 -Wall -Wextra -O1 -DHAL_JOY_FITTED=1 -DHAL_PADDLE_FITTED=1 -I. $INC -c hal/$f.c -o "$OUTV/${f}_sensors.o" 2>&1)
    if [ $? -ne 0 ] || [ -n "$msg" ]; then echo "TriCore FAIL/WARN $f.c (analog joystick + paddlewheel, flags 1)"; echo "$msg" | head -20; fail=1; else echo "TriCore OK $f.c (analog joystick + paddlewheel, flags 1)"; fi
done
# all HAL functions used by app/ must exist in hal_tc375.c (link check)
"$B/tricore-elf-nm.exe" -u "$OUT"/*.o 2>/dev/null | awk '$1=="U" && $2 ~ /^hal_/ {print $2}' | sort -u > "$TEMP/hal_used.txt"
"$B/tricore-elf-nm.exe" "$OUT/hal_tc375.o" | grep ' T hal_' | grep -o 'hal_[a-z_0-9]*' | sort -u > "$TEMP/hal_def.txt"
missing=$(comm -23 "$TEMP/hal_used.txt" "$TEMP/hal_def.txt")
if [ -n "$missing" ]; then echo "MISSING in hal_tc375.c: $missing"; fail=1; else echo "hal_tc375.c provides every HAL function used"; fi
# the board drivers (hal/tc_*.c): every driver function used is defined
"$B/tricore-elf-nm.exe" -u "$OUT"/*.o "$OUTV"/*.o 2>/dev/null | awk '$1=="U" && $2 ~ /^((uart|spi|bno|gps|sd|fat32|adc|nvlog|mb|can|dc|cr|pulse)_|canard[A-Z])/ {print $2}' | sort -u > "$TEMP/drv_used.txt"
"$B/tricore-elf-nm.exe" "$OUT"/tc_*.o "$OUT"/dronecan.o "$OUT"/canard.o "$OUT"/can_relay.o | awk '$2=="T" {print $3}' | sort -u > "$TEMP/drv_def.txt"
missing=$(comm -23 "$TEMP/drv_used.txt" "$TEMP/drv_def.txt")
if [ -n "$missing" ]; then echo "MISSING driver functions: $missing"; fail=1; else echo "every board driver function used is defined"; fi
# every iLLD (Ifx*) function used has its .c file present and NOT excluded in .cproject (else the ADS link fails)
"$B/tricore-elf-nm.exe" -u "$OUT"/*.o "$OUTV"/*.o 2>/dev/null | awk '$1=="U" && $2 ~ /^Ifx/ {print $2}' | sort -u > "$TEMP/ifx_used.txt"
"$PY" tests/acceptance/check_ads_ifx.py --symbols "$TEMP/ifx_used.txt" || fail=1

# the throttle servo is written ONLY by app/servo.c (power limit clamp in the lowest layer)
writers=$(grep -rln --include=*.c "hal_set_throttle_pwm *(" app Cpu0_Main.c Cpu1_Main.c Cpu2_Main.c 2>/dev/null | grep -v "^app/servo.c$")
if [ -n "$writers" ]; then echo "THROTTLE SERVO WRITTEN OUTSIDE servo.c: $writers"; fail=1; else echo "throttle servo written only by app/servo.c"; fi
# the relay board is driven ONLY by app/relays.c (table, interlocks, time limits; never CPU1 / CPU2)
writers=$(grep -rlnE --include=*.c "(^|[^A-Za-z0-9_])hal_relay_[a-z]+ *\(" app sim/sim_sb.c sim/sim_main.c sim/sim_server.c sim/sim_world.c Cpu0_Main.c Cpu1_Main.c Cpu2_Main.c 2>/dev/null | grep -v "^app/relays.c$")
if [ -n "$writers" ]; then echo "RELAY BOARD DRIVEN OUTSIDE relays.c: $writers"; fail=1; else echo "relay board driven only by app/relays.c"; fi
# the DP thrusters are written ONLY by app/thruster.c (clamp, rate limit, heat budget, 0 outside DP; CLAUDE.md P)
writers=$(grep -rlnE --include=*.c "(^|[^A-Za-z0-9_])hal_set_thrusters *\(" app sim/sim_sb.c sim/sim_main.c sim/sim_server.c sim/sim_world.c Cpu0_Main.c Cpu1_Main.c Cpu2_Main.c 2>/dev/null | grep -v "^app/thruster.c$")
if [ -n "$writers" ]; then echo "THRUSTERS WRITTEN OUTSIDE thruster.c: $writers"; fail=1; else echo "DP thrusters written only by app/thruster.c"; fi
# the engine relays only through the engine rules (app/engine.c, CLAUDE.md N): the STARTER (hold-to-crank) and
# IGNITION / SHUTOFF ON (idle + neutral + e-stop released); the software reset only from smallboat.c
writers=$(grep -rlnE --include=*.c "(^|[^A-Za-z0-9_])relays_(starter_pulse|starter_off|engine_set) *\(" app Cpu0_Main.c Cpu1_Main.c Cpu2_Main.c 2>/dev/null | grep -v "^app/engine.c$\|^app/relays.c$")
if [ -n "$writers" ]; then echo "ENGINE RELAYS DRIVEN OUTSIDE engine.c: $writers"; fail=1; else echo "STARTER / IGNITION / SHUTOFF driven only by app/engine.c"; fi
writers=$(grep -rlnE --include=*.c "(^|[^A-Za-z0-9_])hal_system_reset *\(" app Cpu0_Main.c Cpu1_Main.c Cpu2_Main.c 2>/dev/null | grep -v "^app/smallboat.c$")
if [ -n "$writers" ]; then echo "SOFTWARE RESET OUTSIDE smallboat.c: $writers"; fail=1; else echo "software reset only by app/smallboat.c (remote restart)"; fi

# DroneCAN servos (step 11e): ONLY app/servo.c writes the servos (throttle / gear / steering position, power-limit
# clamp in the lowest layer) and ONLY the CAN output layer hal/can_act.c turns them into ArrayCommands (CPU1's
# safe frames: hal/can_safe.c through hal_can_safe_send); the relay hardpoints ONLY by hal/can_relay.c (step 11f)
writers=$(grep -rlnE --include=*.c "(^|[^A-Za-z0-9_])dc_send_actuator *\(" app hal Cpu0_Main.c Cpu1_Main.c Cpu2_Main.c 2>/dev/null | grep -v "^hal/can_act.c$\|^hal/dronecan.c$")
if [ -n "$writers" ]; then echo "SERVO COMMANDS OVER CAN OUTSIDE hal/can_act.c: $writers"; fail=1; else echo "servo ArrayCommands sent only by hal/can_act.c (the CAN output layer)"; fi
writers=$(grep -rlnE --include=*.c "(^|[^A-Za-z0-9_])dc_send_[a-z]+ *\(" app hal/hal_tc375.c Cpu0_Main.c Cpu1_Main.c Cpu2_Main.c 2>/dev/null)
if [ -n "$writers" ]; then echo "COMMANDS OVER CAN IN THE FIRMWARE OUTSIDE THE CAN LAYER: $writers"; fail=1; else echo "no CAN command outside the CAN layer (hal/can_act.c, hal/can_relay.c)"; fi
writers=$(grep -rlnE --include=*.c "(^|[^A-Za-z0-9_])dc_send_hardpoint *\(" app hal Cpu0_Main.c Cpu1_Main.c Cpu2_Main.c 2>/dev/null | grep -v "^hal/can_relay.c$\|^hal/dronecan.c$")
if [ -n "$writers" ]; then echo "RELAY HARDPOINTS OVER CAN OUTSIDE hal/can_relay.c: $writers"; fail=1; else echo "relay hardpoint.Command sent only by hal/can_relay.c (behind hal_relay_*, i.e. app/relays.c)"; fi
writers=$(grep -rlnE --include=*.c "(^|[^A-Za-z0-9_])cr_(set|pulse|init|tick|service) *\(" app Cpu0_Main.c Cpu1_Main.c Cpu2_Main.c 2>/dev/null)
if [ -n "$writers" ]; then echo "THE CAN RELAY BACK-END DRIVEN OUTSIDE THE HAL: $writers"; fail=1; else echo "the CAN relay back-end is reached only through hal_relay_* (app/relays.c)"; fi
writers=$(grep -rlnE --include=*.c "(^|[^A-Za-z0-9_])(hal_set_gear_pwm|hal_set_steer_pos|ca_set_[a-z_]+) *\(" app Cpu0_Main.c Cpu1_Main.c Cpu2_Main.c 2>/dev/null | grep -v "^app/servo.c$")
if [ -n "$writers" ]; then echo "GEAR / STEERING SERVO WRITTEN OUTSIDE servo.c: $writers"; fail=1; else echo "gear servo + steering position servo written only by app/servo.c"; fi
# the bus is serviced / read by app/smallboat.c (core 0); the 10 ms actuator task by Cpu0_Main.c; CPU1's monitor
# only reads the task counter and sends its safe frames
writers=$(grep -rlnE --include=*.c "(^|[^A-Za-z0-9_])hal_can_[a-z_]+ *\(" app Cpu0_Main.c Cpu1_Main.c Cpu2_Main.c 2>/dev/null | grep -v "^app/smallboat.c$\|^app/monitor.c$\|^Cpu0_Main.c$")
if [ -n "$writers" ]; then echo "CAN BUS USED OUTSIDE smallboat.c / monitor.c / Cpu0_Main.c: $writers"; fail=1; else echo "CAN bus used only by app/smallboat.c + Cpu0_Main.c (core 0) and the CPU1 monitor's safe path"; fi
bad=$(grep -nE "hal_can_[a-z_]+ *\(" app/monitor.c | grep -v "hal_can_act_count\|hal_can_safe_send")
if [ -n "$bad" ]; then echo "CPU1 MONITOR USES MORE OF THE CAN BUS THAN ITS SAFE PATH: $bad"; fail=1; fi
bad=$(grep -nE "hal_can_[a-z_]+ *\(" Cpu0_Main.c | grep -v "hal_can_tick")
if [ -n "$bad" ]; then echo "Cpu0_Main.c USES MORE OF THE CAN BUS THAN THE ACTUATOR TASK: $bad"; fail=1; fi

# ADS compiles every C/C++ file in the project folder: every top-level folder
# that is not board firmware must be excluded in .cproject (all configurations)
"$PY" tests/acceptance/check_ads_excludes.py || fail=1

# the sim's test hooks (raw gear / throttle) never in the board firmware
if "$B/tricore-elf-nm.exe" "$OUT/smallboat.o" 2>/dev/null | grep -q "sb_test_drive\|sb_test_thrust"; then
    echo "sb_test_drive / sb_test_thrust (sim test hooks) IN THE FIRMWARE"; fail=1
else
    echo "no sim test hooks in the firmware"
fi

# the board's monitor trip drives the outputs itself (works with core 0 frozen)
safe=$(sed -n '/^void hal_safe_outputs/,/^}/p' hal/hal_tc375.c)
if echo "$safe" | grep -q "servo_write(THROTTLE_TOM_CH" && echo "$safe" | grep -q "servo_write(GEAR_TOM_CH" &&
   echo "$safe" | grep -q "motor_off()" && echo "$safe" | grep -q "thrusters_stop()"; then
    echo "hal_safe_outputs (board) writes idle + neutral + motor off + thrusters 0 itself"
else
    echo "hal_safe_outputs (board) does NOT drive the outputs itself"; fail=1
fi
stop=$(sed -n '/^static void thrusters_stop/,/^}/p' hal/hal_tc375.c)
if echo "$stop" | grep -q "servo_write(THR_BOW_TOM_CH, THR_STOP_US)" && echo "$stop" | grep -q "servo_write(THR_STERN_TOM_CH, THR_STOP_US)"; then
    echo "thrusters_stop (board) writes both thruster shadow registers to the stop pulse"
else
    echo "thrusters_stop (board) does NOT stop both thrusters"; fail=1
fi

# expected memory per core from the TriCore objects (CPU1 = safety monitor: max 60 %)
echo "== memory per core (expected, from the objects)"
"$PY" tools/mapreport.py --objects "$OUT" > "$TEMP/mem_report.txt" 2>&1
mrc=$?
sed -n '/RAM PER CORE/,$p' "$TEMP/mem_report.txt"
[ $mrc -ne 0 ] && { echo "MEMORY CHECK FAILED"; fail=1; }

# every Python file of the ground station / tools must at least parse
for f in groundstation/*.py tools/*.py academy/*.py; do
    "$PY" -c "import ast,sys; ast.parse(open(sys.argv[1],encoding='utf-8').read())" "$f" 2>/dev/null || { echo "PYTHON SYNTAX ERROR: $f"; fail=1; }
done
echo "python files parse"
echo "== FAT32 writer of the board (PC disk images)"
"$PY" tests/hal_pc/fat32_check.py > "$TEMP/fat32_tests.txt" 2>&1 || { echo "FAT32 TESTS FAILED"; fail=1; }
grep -E "^\[FAIL\]|FAT32 TESTS|BUILD PROBLEM" "$TEMP/fat32_tests.txt"
echo "== data flash record log of the board (PC flash image: power losses, swaps, corruption)"
"$PY" tests/hal_pc/nv_check.py > "$TEMP/nv_tests.txt" 2>&1 || { echo "NV TESTS FAILED"; fail=1; }
grep -E "^\[FAIL\]|NV TESTS|BUILD PROBLEM" "$TEMP/nv_tests.txt"
echo "== DroneCAN layer of the board (dronecan.c + libcanard vs an independent encoder; driver state machine)"
"$PY" tests/hal_pc/dronecan_check.py > "$TEMP/dronecan_tests.txt" 2>&1 || { echo "DRONECAN TESTS FAILED"; fail=1; }
grep -E "^\[FAIL\]|DRONECAN TESTS|BUILD PROBLEM" "$TEMP/dronecan_tests.txt"
echo "== chart tests"
( cd groundstation && "$PY" test_charts.py > "$TEMP/chart_tests.txt" 2>&1 ) || { echo "CHART TESTS FAILED"; fail=1; }
grep -E "^\[FAIL\]|CHART TESTS" "$TEMP/chart_tests.txt"
echo "== ground station language tests (el / en)"
( cd groundstation && PYTHONIOENCODING=utf-8 "$PY" test_lang.py > "$TEMP/lang_tests.txt" 2>&1 ) || { echo "LANGUAGE TESTS FAILED"; fail=1; }
grep -E "^\[FAIL\]|LANGUAGE TESTS" "$TEMP/lang_tests.txt"
echo "== boats + two screens (boats/, boat confirm before HELM, F11, map in screen 2)"
( cd groundstation && PYTHONIOENCODING=utf-8 "$PY" test_boats.py > "$TEMP/boat_tests.txt" 2>&1 ) || { echo "BOAT TESTS FAILED"; fail=1; }
grep -E "^\[FAIL\]|BOAT TESTS" "$TEMP/boat_tests.txt"
echo "== screen 2 (navigation & control): layout 1366x768 / 1920x1080 at 100-150 %, roles, map editing, one look"
( cd groundstation && PYTHONIOENCODING=utf-8 "$PY" test_screen2.py > "$TEMP/screen2_tests.txt" 2>&1 ) || { echo "SCREEN 2 TESTS FAILED"; fail=1; }
grep -E "^\[FAIL\]|SCREEN 2 TESTS" "$TEMP/screen2_tests.txt"
echo "== ground station sound + voice tests"
( cd groundstation && PYTHONIOENCODING=utf-8 "$PY" test_sound.py > "$TEMP/sound_tests.txt" 2>&1 ) || { echo "SOUND TESTS FAILED"; fail=1; }
grep -E "^\[(FAIL|N/A )\]|SOUND TESTS" "$TEMP/sound_tests.txt"
echo "== avoidance planner tests"
( cd gateway && "$PY" test_avoid.py > "$TEMP/avoid_tests.txt" 2>&1 ) || { echo "AVOIDANCE TESTS FAILED"; fail=1; }
grep -E "^\[FAIL\]|AVOIDANCE TESTS" "$TEMP/avoid_tests.txt"
echo "== gateway PC on Ubuntu + Windows (serial names, config, install files)"
( cd gateway && "$PY" test_osdep.py > "$TEMP/osdep_tests.txt" 2>&1 ) || { echo "OS TESTS FAILED"; fail=1; }
grep -E "^\[FAIL\]|OS TESTS" "$TEMP/osdep_tests.txt"
echo "== academy (step 15+): no boat link - only the simulator on this PC or recorded files; banner; content; ADS exclude"
PYTHONIOENCODING=utf-8 "$PY" academy/test_academy.py > "$TEMP/academy_tests.txt" 2>&1 || { echo "ACADEMY TESTS FAILED"; fail=1; }
grep -E "^\[FAIL\]|ACADEMY TESTS" "$TEMP/academy_tests.txt"
echo "== academy practical test (step 21): read only + CONFIRM_ONLY (changes nothing on the boat), no bypass of any safety rule"
PYTHONIOENCODING=utf-8 "$PY" academy/practical/test_practical.py > "$TEMP/practical_tests.txt" 2>&1 || { echo "PRACTICAL TESTS FAILED"; fail=1; }
grep -E "^\[FAIL\]|PRACTICAL TESTS" "$TEMP/practical_tests.txt"
echo "== gateway tests"
( cd gateway && "$PY" test_gateway.py > "$TEMP/gw_tests.txt" 2>&1 ) || { echo "GATEWAY TESTS FAILED"; fail=1; }
grep -E "^\[FAIL\]|GATEWAY TESTS" "$TEMP/gw_tests.txt"
[ $fail -eq 0 ] && echo "GATE: OK" || echo "GATE: FAILED"
exit $fail
