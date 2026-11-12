#!/bin/sh
# The test gate: run it before every commit (Git Bash, from anywhere).
#     sh tools/check.sh                     the FULL gate = every stage below in sequence, then the summary
#     sh tools/check.sh --list              the stages
#     sh tools/check.sh --stage <name>      ONE stage alone (a partial check, never "GATE: OK")
# CL4 rest (owner 2026-11-11): the gate is split in stages that each run well under 45 minutes (the owner's rule: kill any
# test that runs longer than 45 minutes). Every stage runs as its OWN child shell under a watchdog (TRYGONS_STAGE_LIMIT_S,
# default 2700 s = 45 min): over the limit = the stage's shell is killed + the processes it provably started are stopped
# (its own strays state) and it is reported STALLED = a FAIL, never a pass. Every stage has its own strays.py
# `mark --begin` / `check <suite>` / `report`; the final summary lists every stage with its result + seconds. A stage that
# is missing, skipped (no result), crashed or timed out FAILS the full gate.
# Together the stages are what the one script did before: sim build without warnings, all sim tests (analog + USB bridge +
# DroneCAN servos worst-case / fast), TriCore compile of app/ + hal/ + Cpu*_Main.c, HAL / writer checks, stack check,
# expected memory per core (CPU1 RAM below 60 %), FAT32 / data flash / DroneCAN PC checks, Python syntax, ground station,
# academy, assistant, UAV / SAR / ADS-B and gateway tests.
# Ends with "GATE: OK" or "GATE: FAILED" (full gate only).
cd "$(dirname "$0")/.." || exit 1
# Python: $PYTHON, else MSYS2's (MANUAL chapter 3), else "python" on the PATH
PY=${PYTHON:-/c/msys64/ucrt64/bin/python.exe}
[ -x "$PY" ] || PY=python
fail=0
STAGES="docs firmware station academy gateway-misc gateway"
STAGE_LIMIT_S=${TRYGONS_STAGE_LIMIT_S:-2700}

# CL3 (W10 / W11): no stray process. tools/strays.py knows THIS project's processes (python / pythonw, autopilot_sim,
# mediamtx, ffmpeg, workers - by the repo folder in their command line, or a child of one; never another folder, never
# the owner's other programs). What runs before a stage is listed and never blamed; after every suite what it LEFT
# BEHIND is reported (name, PID, suite), stopped, and fails the stage at the end ("no leftover processes").
# CL3-r: `mark --begin` puts THIS shell into the stage's job object, so only a process the suites PROVABLY started (in
# the job, not a demo / station / gateway or a child of one) is ever stopped; anything else new is reported only, and
# a process list / state file that can't be read fails the check loudly (nothing stopped). State file per repo folder
# AND per stage.
TAG=$("$PY" tools/strays.py tag)
RES="$TEMP/trygons_gate_$TAG"           # per-stage result files (one line: OK|FAILED|STALLED|CRASHED seconds)
STRAYS_STATE="$TEMP/trygons_strays_gate_$TAG.json"
strays_check() { "$PY" tools/strays.py check "$1" --state "$STRAYS_STATE" || { echo "LEFTOVER CHECK FAILED after: $1 (see the lines above)"; fail=1; }; }
stage_begin() {
    echo "== stage $1: stray processes (CL3): selftest, what runs before the stage"
    "$PY" tools/strays.py selftest || { echo "STRAYS SELFTEST FAILED"; fail=1; }
    "$PY" tools/strays.py mark --begin --shell "$(cat /proc/$$/winpid 2>/dev/null)" --state "$STRAYS_STATE" || { echo "STRAYS MARK FAILED"; fail=1; }
}
stage_end() {
    echo "== stage $1: stray processes (CL3): leftovers of this stage"
    "$PY" tools/strays.py report --state "$STRAYS_STATE" || { echo "NO-LEFTOVERS CHECK FAILED (tools/strays.py report)"; fail=1; }
}

# ---- stage docs: progress archive, python syntax, talking-manual index / question list / extract floor (fast)
stage_docs() {
    stage_begin docs
echo "== progress archive (CL4a)"
"$PY" tools/check_progress_archive.py || { echo "PROGRESS ARCHIVE CHECK FAILED"; fail=1; }
# every Python file of the ground station / tools must at least parse
for f in groundstation/*.py tools/*.py academy/*.py gateway/uav/*.py; do
    "$PY" -c "import ast,sys; ast.parse(open(sys.argv[1],encoding='utf-8').read())" "$f" 2>/dev/null || { echo "PYTHON SYNTAX ERROR: $f"; fail=1; }
done
echo "python files parse"
echo "== talking manual index (AM3, AM3-r MED-1): the COMMITTED groundstation/assistant/index/ must match the manuals / the tour / the parser - a stale index FAILS (rebuild: python tools/assistant_index.py build, then commit it); the question list (tests/assistant: all, in-sample, HELD-OUT)"
PYTHONIOENCODING=utf-8 "$PY" tools/assistant_index.py check > "$TEMP/asst_index.txt" 2>&1 || { echo "ASSISTANT INDEX STALE - run: python tools/assistant_index.py build, then commit the index (see $TEMP/asst_index.txt)"; fail=1; }
cat "$TEMP/asst_index.txt"
PYTHONIOENCODING=utf-8 "$PY" tools/assistant_index.py eval > "$TEMP/asst_eval.txt" 2>&1 || { echo "ASSISTANT QUESTION LIST BROKEN (see $TEMP/asst_eval.txt)"; fail=1; }
cat "$TEMP/asst_eval.txt"
echo "== talking manual extract score floor (AM4-r3 LOW-b, W112 / W117): the BM25 scores still separate what the floor was tuned on (tests/assistant/floor.json); moved = re-tune the floor on the TUNING part, then: python tools/assistant_index.py floor --write"
PYTHONIOENCODING=utf-8 "$PY" tools/assistant_index.py floor > "$TEMP/asst_floor.txt" 2>&1 || { echo "ASSISTANT EXTRACT FLOOR: THE SEARCH SCORES MOVED - re-tune it (see $TEMP/asst_floor.txt)"; fail=1; }
cat "$TEMP/asst_floor.txt"
strays_check "assistant index / eval / floor"
    stage_end docs
}

# ---- stage firmware: sim build + all sim tests (analog, bridge, DroneCAN worst / fast), TriCore compile, HAL / writer checks, stack check, memory, FAT32 / NV / DroneCAN PC checks, api_review
stage_firmware() {
    stage_begin firmware
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
strays_check "sim tests (all, --bridge, --can, --can-fast)"

echo "== TriCore compile"
OUT="$TEMP/tricore_obj"; mkdir -p "$OUT"
rm -f "$OUT"/*.ci "$TEMP"/tricore_obj_variants/*.ci "$TEMP"/cpu0_big.ci    # step 13s: call graphs of THIS build only
# every include folder of .cproject (all configurations, each folder once, in ADS order)
INC=$(grep -o 'ProjName}/[^&]*' .cproject | sed 's#}$##; s#ProjName}/#-I#; s#^-I$#-I.#' | awk '!seen[$0]++' | tr '\n' ' ')
# the TriCore gcc of AURIX Development Studio: $TRICORE_BIN, else the newest ADS under C:\Infineon
B=${TRICORE_BIN:-$(ls -d /c/Infineon/AURIX-Studio-*/tools/Compilers/tricore-gcc*/bin 2>/dev/null | sort -V | tail -1)}
[ -x "$B/tricore-elf-gcc.exe" ] || { echo "TriCore gcc NOT FOUND (install ADS or set TRICORE_BIN)"; fail=1; }
for f in app/*.c hal/*.c hal/libcanard/*.c Cpu0_Main.c Cpu1_Main.c Cpu2_Main.c; do
    msg=$("$B/tricore-elf-gcc.exe" -mcpu=tc39xx -std=c99 -Wall -Wextra -O1 -fcallgraph-info=su -I. $INC -c "$f" -o "$OUT/$(basename "$f" .c).o" 2>&1)
    if [ $? -ne 0 ] || [ -n "$msg" ]; then echo "TriCore FAIL/WARN $f"; echo "$msg" | head -20; fail=1; else echo "TriCore OK $f"; fi
done
# the big-boat build of core 0 (not linked with the rest, only compiled)
msg=$("$B/tricore-elf-gcc.exe" -mcpu=tc39xx -std=c99 -Wall -Wextra -O1 -fcallgraph-info=su -DBOAT_VARIANT_BIG -I. $INC -c Cpu0_Main.c -o "$TEMP/cpu0_big.o" 2>&1)
if [ $? -ne 0 ] || [ -n "$msg" ]; then echo "TriCore FAIL/WARN Cpu0_Main.c (big boat)"; echo "$msg" | head -20; fail=1; else echo "TriCore OK Cpu0_Main.c (big boat)"; fi
# the FIRST TEST build (steering only; empty value = same as "#define FIRST_TEST" in Cpu0_Main.c, no redefinition warning)
msg=$("$B/tricore-elf-gcc.exe" -mcpu=tc39xx -std=c99 -Wall -Wextra -O1 -DFIRST_TEST= -I. $INC -c Cpu0_Main.c -o "$TEMP/cpu0_ft.o" 2>&1)
if [ $? -ne 0 ] || [ -n "$msg" ]; then echo "TriCore FAIL/WARN Cpu0_Main.c (FIRST TEST)"; echo "$msg" | head -20; fail=1; else echo "TriCore OK Cpu0_Main.c (FIRST TEST)"; fi
# step 11f: the board paths behind the hardware flags (the flags stay 0 in hal_pins.h until the owner confirms):
# the DroneCAN relay board + the DroneCAN servos (RELAY_BUS_DRONECAN), and the RS485 Modbus relay board fitted
OUTV="$TEMP/tricore_obj_variants"; mkdir -p "$OUTV"
for v in "can:-DHAL_CAN_FITTED=1 -DHAL_CAN_SERVOS=1 -DHAL_RELAY_FITTED=1 -DRELAY_BUS=2" "modbus:-DHAL_RELAY_FITTED=1 -DRELAY_BUS=1"; do
    name=${v%%:*}; defs=${v#*:}
    msg=$("$B/tricore-elf-gcc.exe" -mcpu=tc39xx -std=c99 -Wall -Wextra -O1 -fcallgraph-info=su $defs -I. $INC -c hal/hal_tc375.c -o "$OUTV/hal_tc375_$name.o" 2>&1)
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
# step HX2: HARB_MUTATE (the storm's mutation runs, tests/acceptance/storm/mutate_harbour.sh) is never set by any build
# file, the board's project or app/ itself
if grep -lE "HARB_MUTATE=|define +HARB_MUTATE +[1-9]" sim/build_sim.bat tests/acceptance/storm/build_storm.bat .cproject app/*.c app/*.h hal/*.c Cpu*_Main.c 2>/dev/null; then echo "HARB_MUTATE SET IN A BUILD"; fail=1; else echo "no build sets HARB_MUTATE (harbour mutation hooks off)"; fi
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
bad=$(grep -nE "hal_can_[a-z_]+ *\(" Cpu0_Main.c | grep -v "hal_can_tick\|hal_can_servos_off")   # (13f: the big boat switches the CAN servos off before hal_init)
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
# step 13d (HIGH-1): a trip before core 0's first step never forces 0 us: the board HAL replaces an out-of-range
# pulse, the monitor falls back, Cpu0_Main.c publishes the safe pulses BEFORE hal_init (both variants; step 13e: CPU1
# already runs then) and the monitor reads hal_servos_can() LIVE until core 0's loop runs (13e, review 1f7817c HIGH-1)
if echo "$safe" | grep -q "HAL_SAFE_THR_US" && echo "$safe" | grep -q "HAL_SAFE_GEAR_US" &&
   grep -q "monitor_safe_us(sh->thr_idle_us, MON_SAFE_THR_US)" app/monitor.c &&
   grep -q "monitor_safe_us(sh->gear_n_us, MON_SAFE_GEAR_US)" app/monitor.c &&
   grep -q "m->can_hw = hal_servos_can()" app/monitor.c && grep -q "(!m->started && m->can_hw)" app/monitor.c &&
   grep -q "if (!m->started && !m->can_hw) m->can_hw = hal_servos_can();" app/monitor.c &&
   [ "$(sed -n '1,/^    hal_init();/p' Cpu0_Main.c | grep -c 'monitor_publish_safe(&g_mon')" -ge 2 ]; then
    echo "safe pulses before core 0's first step: board HAL guard, monitor fallback + live hal_servos_can, published before hal_init"
else
    echo "safe pulses before core 0's first step NOT guarded (hal_safe_outputs / monitor.c / Cpu0_Main.c)"; fail=1
fi
# step 13f (review 94afdc4 MED-1 / LOW-4 / LOW-5): hal_init sends the first throttle IDLE tick right after ca_init,
# then the transfer ID + a barrier BEFORE CPU1 may use the reserved buffers; the big boat switches the servos off
# before hal_init (OD-12); the small boat's NV boot loop keeps ticking
cpi=$(sed -n '/^static void can_port_init/,/^}/p' hal/hal_tc375.c)
if echo "$cpi" | tr -d '
' | grep -q "if (!s_no_servos).*ca_init();.*ca_tick();.*s_can_tid = dc_cmd_tid();.*__dsync();.*s_servos_can = TRUE;" &&
   [ "$(sed -n '1,/^    hal_init();/p' Cpu0_Main.c | grep -c 'hal_can_servos_off();')" -eq 1 ] &&
   sed -n '/^    {   \/\* boot counter/,/^    }/p' Cpu0_Main.c | grep -q "sb_bootnv_begin(&bn, hal_millis(), NV_BOOT_MS, true);" &&
   sed -n '/^    {   \/\* boot counter/,/^    }/p' Cpu0_Main.c | grep -q "while (sb_bootnv_step(&bn, hal_millis()))" &&
   sed -n '/^bool sb_bootnv_step/,/^}/p' app/smallboat.c | grep -q "hal_can_tick();" &&
   sed -n '/^bool sb_bootnv_step/,/^}/p' app/smallboat.c | grep -q "hal_watchdog_kick();"; then
    echo "boot: throttle IDLE from hal_init + the NV boot loop (watchdog kicked), barrier before CPU1's flag, big boat without CAN servos"
else
    echo "boot throttle IDLE / big-boat CAN servos NOT as step 13f (hal_tc375.c can_port_init / Cpu0_Main.c)"; fail=1
fi
stop=$(sed -n '/^static void thrusters_stop/,/^}/p' hal/hal_tc375.c)
if echo "$stop" | grep -q "servo_write(THR_BOW_TOM_CH, THR_STOP_US)" && echo "$stop" | grep -q "servo_write(THR_STERN_TOM_CH, THR_STOP_US)"; then
    echo "thrusters_stop (board) writes both thruster shadow registers to the stop pulse"
else
    echo "thrusters_stop (board) does NOT stop both thrusters"; fail=1
fi

# step 13s: stack safety from the TriCore objects' call graphs (GCC -fcallgraph-info): no recursion (every function-
# pointer call resolved by tools/stackcheck.py INDIRECT, the table proved against the source: 13s review MED-1), worst case per core x 1.5 inside the linker scripts' stacks
# + CSA (both scripts the same); the small boat, the board with the DroneCAN servos + relay board, the big boat.
# The ADS map's own check (TASKING's numbers) = tools/mapreport.py "<map>" after every ADS build (owner).
echo "== stack check (call graph: recursion, pointer calls, worst case per core vs the linker scripts)"
"$PY" tools/stackcheck.py --selftest > "$TEMP/stack_self.txt" 2>&1 || { echo "STACKCHECK SELFTEST FAILED"; grep FAIL "$TEMP/stack_self.txt"; fail=1; }
tail -1 "$TEMP/stack_self.txt"
"$PY" tools/mapreport.py --selftest > "$TEMP/map_self.txt" 2>&1 || { echo "MAPREPORT SELFTEST FAILED"; grep FAIL "$TEMP/map_self.txt"; fail=1; }
tail -1 "$TEMP/map_self.txt"
"$PY" tools/stackcheck.py "$OUT" > "$TEMP/stack_check.txt" 2>&1 || { echo "STACK CHECK FAILED (small boat)"; fail=1; }
cat "$TEMP/stack_check.txt"
canci=$(ls "$OUT"/*.ci | grep -v "/hal_tc375.ci$")
"$PY" tools/stackcheck.py $canci "$OUTV/hal_tc375_can.ci" > "$TEMP/stack_check_can.txt" 2>&1 || { echo "STACK CHECK FAILED (DroneCAN servos + relay board)"; cat "$TEMP/stack_check_can.txt"; fail=1; }
grep "STACK CHECK" "$TEMP/stack_check_can.txt" | sed 's/$/ (DroneCAN servos + relay board, flags 1)/'
bigci=$(ls "$OUT"/*.ci | grep -v "/Cpu0_Main.ci$")
"$PY" tools/stackcheck.py $bigci "$TEMP/cpu0_big.ci" > "$TEMP/stack_check_big.txt" 2>&1 || { echo "STACK CHECK FAILED (big boat)"; cat "$TEMP/stack_check_big.txt"; fail=1; }
grep "STACK CHECK" "$TEMP/stack_check_big.txt" | sed 's/$/ (big boat)/'
# BENCH_DIAG's length: app/proto.h = groundstation/proto.py (the stack block, step 13s)
if [ "$(grep -o 'PROTO_BENCH_DIAG_LEN  *[0-9]*' app/proto.h | grep -o '[0-9]*$')" = "$(grep -o '^BENCH_DIAG_LEN = [0-9]*' groundstation/proto.py | grep -o '[0-9]*$')" ] &&
   [ "$(grep -o 'PROTO_BENCH_DIAG_STACK_OFF  *[0-9]*' app/proto.h | grep -o '[0-9]*$')" = "$(grep -o '^BENCH_DIAG_STACK_OFF = [0-9]*' groundstation/proto.py | grep -o '[0-9]*$')" ]; then
    echo "BENCH_DIAG stack block: app/proto.h = groundstation/proto.py"
else
    echo "BENCH_DIAG stack block: app/proto.h and groundstation/proto.py DIFFER"; fail=1
fi

# expected memory per core from the TriCore objects (CPU1 = safety monitor: max 60 %)
echo "== memory per core (expected, from the objects)"
"$PY" tools/mapreport.py --objects "$OUT" > "$TEMP/mem_report.txt" 2>&1
mrc=$?
sed -n '/RAM PER CORE/,$p' "$TEMP/mem_report.txt"
[ $mrc -ne 0 ] && { echo "MEMORY CHECK FAILED"; fail=1; }

echo "== FAT32 writer of the board (PC disk images)"
"$PY" tests/hal_pc/fat32_check.py > "$TEMP/fat32_tests.txt" 2>&1 || { echo "FAT32 TESTS FAILED"; fail=1; }
grep -E "^\[FAIL\]|FAT32 TESTS|BUILD PROBLEM" "$TEMP/fat32_tests.txt"
echo "== data flash record log of the board (PC flash image: power losses, swaps, corruption)"
"$PY" tests/hal_pc/nv_check.py > "$TEMP/nv_tests.txt" 2>&1 || { echo "NV TESTS FAILED"; fail=1; }
grep -E "^\[FAIL\]|NV TESTS|BUILD PROBLEM" "$TEMP/nv_tests.txt"
echo "== DroneCAN layer of the board (dronecan.c + libcanard vs an independent encoder; driver state machine)"
"$PY" tests/hal_pc/dronecan_check.py > "$TEMP/dronecan_tests.txt" 2>&1 || { echo "DRONECAN TESTS FAILED"; fail=1; }
grep -E "^\[FAIL\]|DRONECAN TESTS|BUILD PROBLEM" "$TEMP/dronecan_tests.txt"
strays_check "hal_pc checks (fat32, nv, dronecan)"
echo "== API review tool (tools/api_review.py, fake client, no network)"
"$PY" tools/test_api_review.py > "$TEMP/apirev_tests.txt" 2>&1 || { echo "API REVIEW TESTS FAILED"; fail=1; }
grep -E "^\[FAIL\]|API REVIEW TESTS" "$TEMP/apirev_tests.txt"
strays_check "test_api_review"
    stage_end firmware
}

# ---- stage station: ground station tests (charts, lang, boats, screen 2, look, track, screenrec, yesno, accel, hailer, trends, sound, UAV / SAR / ADS-B station)
stage_station() {
    stage_begin station
echo "== chart tests"
( cd groundstation && "$PY" test_charts.py > "$TEMP/chart_tests.txt" 2>&1 ) || { echo "CHART TESTS FAILED"; fail=1; }
grep -E "^\[FAIL\]|CHART TESTS" "$TEMP/chart_tests.txt"
strays_check "test_charts"
echo "== ground station language tests (el / en)"
( cd groundstation && PYTHONIOENCODING=utf-8 "$PY" test_lang.py > "$TEMP/lang_tests.txt" 2>&1 ) || { echo "LANGUAGE TESTS FAILED"; fail=1; }
grep -E "^\[FAIL\]|LANGUAGE TESTS" "$TEMP/lang_tests.txt"
strays_check "test_lang"
echo "== boats + two screens (boats/, automatic name check + HELM at once, one-move change + leave warning, F11, map in screen 2)"
( cd groundstation && PYTHONIOENCODING=utf-8 "$PY" test_boats.py > "$TEMP/boat_tests.txt" 2>&1 ) || { echo "BOAT TESTS FAILED"; fail=1; }
grep -E "^\[FAIL\]|BOAT TESTS" "$TEMP/boat_tests.txt"
strays_check "test_boats"
echo "== screen 2 (navigation & control): layout 1366x768 / 1920x1080 at 100-150 %, roles, map editing, one look"
( cd groundstation && PYTHONIOENCODING=utf-8 "$PY" test_screen2.py > "$TEMP/screen2_tests.txt" 2>&1 ) || { echo "SCREEN 2 TESTS FAILED"; fail=1; }
grep -E "^\[FAIL\]|SCREEN 2 TESTS" "$TEMP/screen2_tests.txt"
strays_check "test_screen2"
echo "== professional look (13i): start-up screen, app icon, window titles, About, academy theme, shortcuts, no consoles"
( cd groundstation && PYTHONIOENCODING=utf-8 "$PY" test_look.py > "$TEMP/look_tests.txt" 2>&1 ) || { echo "LOOK TESTS FAILED"; fail=1; }
grep -E "^\[FAIL\]|LOOK TESTS" "$TEMP/look_tests.txt"
strays_check "test_look"
echo "== track (13n3): per boat, crash-safe on disk, no point limit, NEW TRACK, saved list, GPX export / import"
( cd groundstation && PYTHONIOENCODING=utf-8 "$PY" test_track.py > "$TEMP/track_tests.txt" 2>&1 ) || { echo "TRACK TESTS FAILED"; fail=1; }
grep -E "^\[FAIL\]|TRACK TESTS" "$TEMP/track_tests.txt"
strays_check "test_track"
echo "== screen recording + KEEP 5' / 1h (13n4): ring of both screens, SAVE / KEEP screen part, low disk, failures, buttons, real ffmpeg"
( cd groundstation && PYTHONIOENCODING=utf-8 "$PY" test_screenrec.py > "$TEMP/screenrec_tests.txt" 2>&1 ) || { echo "SCREEN RECORDING TESTS FAILED"; fail=1; }
grep -E "^\[FAIL\]|SCREEN RECORDING TESTS" "$TEMP/screenrec_tests.txt"
strays_check "test_screenrec"
echo "== YES / NO answers on the shore joystick (13p): trigger / thumb / stick Y / hat, 0.5 s guard, one press = one thing (station + gateway)"
( cd groundstation && PYTHONIOENCODING=utf-8 "$PY" test_yesno.py > "$TEMP/yesno_tests.txt" 2>&1 ) || { echo "YES / NO TESTS FAILED"; fail=1; }
grep -E "^\[FAIL\]|YES / NO TESTS" "$TEMP/yesno_tests.txt"
strays_check "test_yesno"
echo "== Bench tab stack use (13s review LOW-3): BENCH_DIAG stack block decoded + drawn (display only)"
( cd groundstation && PYTHONIOENCODING=utf-8 "$PY" test_benchstack.py > "$TEMP/benchstack_tests.txt" 2>&1 ) || { echo "BENCH STACK TESTS FAILED"; fail=1; }
grep -E "^\[FAIL\]|BENCH STACK TESTS" "$TEMP/benchstack_tests.txt"
strays_check "test_benchstack"
echo "== acceleration frame (FW-TR, display / logging only): proto.py = app/proto.h, decode, gateway pass-through, log event 31 (tlg / log2csv)"
( cd groundstation && PYTHONIOENCODING=utf-8 "$PY" test_accel.py > "$TEMP/accel_tests.txt" 2>&1 ) || { echo "ACCEL FRAME TESTS FAILED"; fail=1; }
grep -E "^\[FAIL\]|ACCEL FRAME TESTS|Traceback" "$TEMP/accel_tests.txt"
strays_check "test_accel"
echo "== harbour autonomy protocol mirrors (HX2): proto.py = app/proto.h / harbour.h, HARBOUR_STAT decode, ACCEPT rule, alarm / event / result 129"
( cd groundstation && PYTHONIOENCODING=utf-8 "$PY" test_harbour_proto.py > "$TEMP/harbour_proto_tests.txt" 2>&1 ) || { echo "HARBOUR PROTO TESTS FAILED"; fail=1; }
grep -E "^\[FAIL\]|HARBOUR PROTO TESTS|Traceback" "$TEMP/harbour_proto_tests.txt"
strays_check "test_harbour_proto"
echo "== loudhailer on the station (13r): voice codec + signed packets, ΟΜΙΛΙΑ press-and-hold on both screens, not on the joystick, ready messages, Tools page, devices"
( cd groundstation && PYTHONIOENCODING=utf-8 "$PY" test_hailer.py > "$TEMP/hailer_tests.txt" 2>&1 ) || { echo "LOUDHAILER STATION TESTS FAILED"; fail=1; }
grep -E "^\[FAIL\]|LOUDHAILER STATION TESTS" "$TEMP/hailer_tests.txt"
strays_check "test_hailer"
echo "== trend panel TASEIS (TR2): ring buffer, sampler from real frames, graphs + click, palette + night, read-only proof TG6-TG9 (static, import graph, trapped senders, narrow view), CPU bounds; layout = test_screen2 S29"
( cd groundstation && PYTHONIOENCODING=utf-8 "$PY" test_trends.py > "$TEMP/trend_tests.txt" 2>&1 ) || { echo "TREND TESTS FAILED"; fail=1; }
grep -E "^\[FAIL\]|TREND TESTS" "$TEMP/trend_tests.txt"
strays_check "test_trends"
echo "== ground station sound + voice tests"
( cd groundstation && PYTHONIOENCODING=utf-8 "$PY" test_sound.py > "$TEMP/sound_tests.txt" 2>&1 ) || { echo "SOUND TESTS FAILED"; fail=1; }
grep -E "^\[(FAIL|N/A )\]|SOUND TESTS" "$TEMP/sound_tests.txt"
strays_check "test_sound"
echo "== UAV magazine on the station (U2): pictures, the Tools page «ΜΑΓΓΕΜΑ UAV», answers in the screen language"
( cd groundstation && PYTHONIOENCODING=utf-8 "$PY" test_uavui.py > "$TEMP/uavui_tests.txt" 2>&1 ) || { echo "UAV STATION TESTS FAILED"; fail=1; }
grep -E "^\[FAIL\]|UAV STATION TESTS" "$TEMP/uavui_tests.txt"
strays_check "test_uavui"
echo "== Search and rescue on the station (PF3): MOB marker, alarm, SAVE, offers, stand-off route"
( cd groundstation && PYTHONIOENCODING=utf-8 "$PY" test_sar.py > "$TEMP/sar_tests.txt" 2>&1 ) || { echo "SAR STATION TESTS FAILED"; fail=1; }
grep -E "^\[FAIL\]|SAR STATION TESTS" "$TEMP/sar_tests.txt"
strays_check "test_sar"
echo "== ADS-B on the station (PF5): map, notices, voice, the proposal asked once and never automatic"
( cd groundstation && PYTHONIOENCODING=utf-8 "$PY" test_adsbui.py > "$TEMP/adsbui_tests.txt" 2>&1 ) || { echo "ADS-B STATION TESTS FAILED"; fail=1; }
grep -E "^\[FAIL\]|ADS-B STATION TESTS" "$TEMP/adsbui_tests.txt"
strays_check "test_adsbui"
    stage_end station
}

# ---- stage academy: talking manual tests, tour pictures, academy, practical test
stage_academy() {
    stage_begin academy
echo "== talking manual (AM2 + AM3 + AM4): read-only proof AS1-AS3, real models offline AS4 + AS24 language model (N/A without them); ANSWER AS19-AS23 (post-check names + numbers, GPU guard, retrieval only, worker answer, get_llm pins, no llama-server), push-to-talk + loudhailer wins, own mic, restarts, gateway PC, screens; FIND AS14-AS18 (index, stale check, normalisation, passages, retrieval in-sample + held-out, worker find + stale index)"
( cd groundstation && PYTHONIOENCODING=utf-8 "$PY" test_assistant.py > "$TEMP/assistant_tests.txt" 2>&1 ) || { echo "ASSISTANT TESTS FAILED"; fail=1; }
grep -E "^\[(FAIL|N/A)\]|ASSISTANT TESTS" "$TEMP/assistant_tests.txt"
strays_check "test_assistant"
echo "== guided tour pictures (13t): made again from the demo when the layout changed (tools/demo_screens.py --tour --if-stale)"
PYTHONIOENCODING=utf-8 "$PY" tools/demo_screens.py --tour --if-stale > "$TEMP/tour_shots.txt" 2>&1 || { echo "TOUR PICTURES FAILED (see $TEMP/tour_shots.txt)"; fail=1; }
grep -E "^tour pictures|TOUR PROBLEM" "$TEMP/tour_shots.txt"
strays_check "tour pictures (demo_screens --tour --if-stale)"
echo "== academy (step 15+): no boat link - only the simulator on this PC or recorded files; banner; content; ADS exclude"
PYTHONIOENCODING=utf-8 "$PY" academy/test_academy.py > "$TEMP/academy_tests.txt" 2>&1 || { echo "ACADEMY TESTS FAILED"; fail=1; }
grep -E "^\[FAIL\]|ACADEMY TESTS" "$TEMP/academy_tests.txt"
strays_check "test_academy"
echo "== academy practical test (step 21): read only + CONFIRM_ONLY (changes nothing on the boat), no bypass of any safety rule"
PYTHONIOENCODING=utf-8 "$PY" academy/practical/test_practical.py > "$TEMP/practical_tests.txt" 2>&1 || { echo "PRACTICAL TESTS FAILED"; fail=1; }
grep -E "^\[FAIL\]|PRACTICAL TESTS" "$TEMP/practical_tests.txt"
strays_check "test_practical"
    stage_end academy
}

# ---- stage gateway-misc: avoidance, OS, setup, loudhailer, UAV magazine, SITL (N/A without it), ADS-B, harbour sensors, SAR demo
stage_gateway_misc() {
    stage_begin gateway-misc
echo "== avoidance planner tests"
( cd gateway && "$PY" test_avoid.py > "$TEMP/avoid_tests.txt" 2>&1 ) || { echo "AVOIDANCE TESTS FAILED"; fail=1; }
grep -E "^\[FAIL\]|AVOIDANCE TESTS" "$TEMP/avoid_tests.txt"
strays_check "test_avoid"
echo "== gateway PC on Ubuntu + Windows (serial names, config, install files)"
( cd gateway && "$PY" test_osdep.py > "$TEMP/osdep_tests.txt" 2>&1 ) || { echo "OS TESTS FAILED"; fail=1; }
grep -E "^\[FAIL\]|OS TESTS" "$TEMP/osdep_tests.txt"
strays_check "test_osdep"
echo "== gateway one-command setup (13j): setup_all.sh with fake answers + stubbed commands (nothing changes), shellcheck / bash -n, no secret in the log"
( cd gateway && PYTHONIOENCODING=utf-8 "$PY" test_setup.py > "$TEMP/setup_tests.txt" 2>&1 ) || { echo "SETUP TESTS FAILED"; fail=1; }
grep -E "^\[FAIL\]|^ST1: shellcheck|SETUP TESTS" "$TEMP/setup_tests.txt"
strays_check "test_setup"
echo "== loudhailer on the gateway (13r): first speaker wins, half duplex, recording in SAVE / KEEP, link priority, no effect on driving, failure = notice"
( cd gateway && PYTHONIOENCODING=utf-8 "$PY" test_loudhailer.py > "$TEMP/loudhailer_tests.txt" 2>&1 ) || { echo "LOUDHAILER TESTS FAILED"; fail=1; }
grep -E "^\[FAIL\]|LOUDHAILER TESTS" "$TEMP/loudhailer_tests.txt"
strays_check "test_loudhailer"
echo "== UAV magazine on the gateway (U2): own MAVLink codec (+ pymavlink cross-check), ALLOW-list, writer check, no AURIX path, read-only registration check, registry, GPS warm start, simulated drones, router process, the real gateway"
( cd gateway && PYTHONIOENCODING=utf-8 "$PY" test_uav.py > "$TEMP/uav_tests.txt" 2>&1 ) || { echo "UAV TESTS FAILED"; fail=1; }
grep -E "^\[(FAIL|N/A )\]|UAV TESTS|STALLED" "$TEMP/uav_tests.txt"
strays_check "test_uav"
echo "== UAV with real ArduPilot SITL (X2): only when SITL is installed + verified (python tools/get_sitl.py); else N/A"
if "$PY" tools/get_sitl.py --check > "$TEMP/sitl_check.txt" 2>&1; then
  PYTHONIOENCODING=utf-8 "$PY" tools/run_sitl.py --log "$TEMP/sitl_tests.txt" > "$TEMP/sitl_run.txt" 2>&1 || { echo "SITL TESTS FAILED (see $TEMP/sitl_tests.txt)"; fail=1; }
  grep -E "^\[(FAIL|N/A )\]|^SITL TESTS|STALLED" "$TEMP/sitl_tests.txt"
  strays_check "test_sitl"
else
  echo "SITL TESTS: N/A ($(cat "$TEMP/sitl_check.txt"))"
fi
echo "== ADS-B receive only (PF5): SBS / Beast parsing, aircraft state, filters, CPA geometry, warning levels, proposals, no receiver / no data, the simulated helicopter, receive-only proof"
( cd gateway && PYTHONIOENCODING=utf-8 "$PY" test_adsb.py > "$TEMP/adsb_tests.txt" 2>&1 ) || { echo "ADS-B TESTS FAILED"; fail=1; }
grep -E "^\[FAIL\]|ADS-B TESTS" "$TEMP/adsb_tests.txt"
strays_check "test_adsb"
echo "== harbour sensors (HX3): the interface, simulated radar + LiDAR vs the world truth, noise / drop-outs, faults -> health + notices, worker crash / hang restart, the real-driver skeleton (W148), no AURIX / no network"
( cd gateway && PYTHONIOENCODING=utf-8 "$PY" test_harbour_sensors.py > "$TEMP/harbour_sensor_tests.txt" 2>&1 ) || { echo "HARBOUR SENSOR TESTS FAILED"; fail=1; }
grep -E "^\[FAIL\]|HARBOUR SENSOR TESTS" "$TEMP/harbour_sensor_tests.txt"
strays_check "test_harbour_sensors"
echo "== harbour perception (HX4): the fusion tracker (radar + LiDAR + camera + AIS) + the static map with the simulated sensors: association, crossing, static map, small craft / person-like, clutter / ghosts, camera / AIS-only never steer_ok, unsure flags, lifetime, timing, determinism, read only, mutations"
( cd gateway && PYTHONIOENCODING=utf-8 "$PY" test_harbour_tracker.py > "$TEMP/harbour_tracker_tests.txt" 2>&1 ) || { echo "HARBOUR TRACKER TESTS FAILED"; fail=1; }
grep -E "^\[FAIL\]|HARBOUR TRACKER TESTS|passed" "$TEMP/harbour_tracker_tests.txt"
strays_check "test_harbour_tracker"
echo "== harbour planner (HX5): velocity-obstacle planner in the corridor + COLREGs + unsure = STOP, steering only on radar / LiDAR-confirmed tracks, 15 / 8 m, the cap, the kind-4 navreq glue (sim gate, fold, 5 Hz lease), mutations; a short avoidance storm (I86h-I90) + its mutation check"
( cd gateway && PYTHONIOENCODING=utf-8 "$PY" test_harbour_planner.py > "$TEMP/harbour_planner_tests.txt" 2>&1 ) || { echo "HARBOUR PLANNER TESTS FAILED"; fail=1; }
grep -E "^\[FAIL\]|HARBOUR PLANNER TESTS|passed" "$TEMP/harbour_planner_tests.txt"
( PYTHONIOENCODING=utf-8 "$PY" tests/acceptance/storm_harbour.py --episodes 30 --seed 101 > "$TEMP/harbour_storm.txt" 2>&1 ) || { echo "HARBOUR STORM FAILED"; fail=1; }
grep -E "^FAIL|HARBOUR STORM" "$TEMP/harbour_storm.txt"
( PYTHONIOENCODING=utf-8 "$PY" tests/acceptance/storm_harbour.py --mutation-check --episodes 25 --seed 1 > "$TEMP/harbour_storm_mut.txt" 2>&1 ) || { echo "HARBOUR STORM MUTATION CHECK FAILED"; fail=1; }
grep -E "mutation|MUTATION CHECK" "$TEMP/harbour_storm_mut.txt"
strays_check "test_harbour_planner"
echo "== SAR demo guide (PF6): progress model, receive-only, ADS-B cue, simulated person, launcher"
( PYTHONIOENCODING=utf-8 "$PY" demo/test_sar_demo.py > "$TEMP/sardemo_tests.txt" 2>&1 ) || { echo "SAR DEMO TESTS FAILED"; fail=1; }
grep -E "^\[FAIL\]|SAR DEMO TESTS" "$TEMP/sardemo_tests.txt"
strays_check "test_sar_demo"
    stage_end gateway-misc
}

# ---- stage gateway: gateway/test_gateway.py (every section in its own process)
stage_gateway() {
    stage_begin gateway
echo "== gateway tests"
( cd gateway && "$PY" test_gateway.py > "$TEMP/gw_tests.txt" 2>&1 ) || { echo "GATEWAY TESTS FAILED"; fail=1; }
grep -E "^\[FAIL\]|GATEWAY TESTS|STALLED|CRASHED" "$TEMP/gw_tests.txt"   # W50s: a stalled section is killed + named
strays_check "test_gateway"
    stage_end gateway
}

# ---- driver
do_stage() {            # run ONE stage in its own child shell under the watchdog; result -> $RES/<stage>.result
    s=$1; mkdir -p "$RES"; rm -f "$RES/$s.result"
    t0=$(date +%s)
    echo; echo "######## STAGE $s (limit ${STAGE_LIMIT_S} s)"
    timeout -k 30 "$STAGE_LIMIT_S" sh "$0" --run-stage "$s"; rc=$?
    dt=$(( $(date +%s) - t0 ))
    if [ $rc -eq 124 ] || [ $rc -eq 137 ]; then
        echo "STAGE $s: STALLED (over ${STAGE_LIMIT_S} s, killed) - a FAIL, never a pass"
        # the stage's shell is gone: stop what it provably started (its own strays state), nothing else
        "$PY" tools/strays.py check "stage $s watchdog" --state "$STRAYS_STATE" || true
        echo "STALLED $dt" > "$RES/$s.result"
    elif [ ! -f "$RES/$s.result" ]; then
        echo "STAGE $s: CRASHED (exit $rc, no result written) - a FAIL"
        echo "CRASHED $dt" > "$RES/$s.result"
    fi
    echo "######## STAGE $s: $(cut -d' ' -f1 "$RES/$s.result") in ${dt} s"
    [ "$(cut -d' ' -f1 "$RES/$s.result")" = OK ]
}
summary() {             # aggregate EVERY stage: a missing result = FAIL
    echo; echo "== gate summary"
    ok=1
    for s in $STAGES; do
        if [ -f "$RES/$s.result" ]; then
            st=$(cut -d' ' -f1 "$RES/$s.result"); sec=$(cut -d' ' -f2 "$RES/$s.result")
        else st="MISSING"; sec=0; fi
        printf '  %-14s %-8s %s s\n' "$s" "$st" "$sec"
        [ "$st" = OK ] || ok=0
    done
    [ $ok -eq 1 ]
}
case "${1:-}" in
    --list) for s in $STAGES; do echo "$s"; done; exit 0 ;;
    --run-stage)            # internal: the child shell of do_stage
        s=$2; fn="stage_$(echo "$s" | tr - _)"
        case " $STAGES " in *" $s "*) ;; *) echo "unknown stage $s"; exit 2 ;; esac
        mkdir -p "$RES"; t0=$(date +%s)
        "$fn"
        dt=$(( $(date +%s) - t0 ))
        if [ $fail -eq 0 ]; then echo "OK $dt" > "$RES/$s.result"; else echo "FAILED $dt" > "$RES/$s.result"; fi
        exit $fail ;;
    --stage)
        s=${2:-}
        case " $STAGES " in *" $s "*) ;; *) echo "usage: sh tools/check.sh --stage <$(echo $STAGES | tr ' ' '|')>"; exit 2 ;; esac
        do_stage "$s" && { echo "STAGE $s: OK (a partial check - the full gate needs every stage)"; exit 0; }
        echo "STAGE $s: FAILED"; exit 1 ;;
    "") ;;
    *) echo "usage: sh tools/check.sh [--list | --stage <name>]"; exit 2 ;;
esac
rm -rf "$RES"
for s in $STAGES; do do_stage "$s" || fail=1; done
summary || fail=1
[ $fail -eq 0 ] && echo "GATE: OK" || echo "GATE: FAILED"
exit $fail
