@echo off
rem Builds the PC simulation: app\ + sim\ (no hardware code) + hal\tc_nv.c
rem (the board's data-flash record log, pure C: sim test DF4 drives it) + hal\tc_modbus.c
rem (the board's Modbus RTU master, pure C: the sim's relay board talks to it, tests RB1-RB8)
rem + hal\dronecan.c + hal\libcanard\canard.c (DroneCAN, pure C: the sim's CAN bus + nodes, tests CN).
rem CANARD_64_BIT=1: libcanard's memory-pool index on a 64-bit PC (the TriCore is 32-bit: default).
rem Usage: double-click, or run  sim\build_sim.bat  from any folder.
rem Result: sim\autopilot_sim.exe

setlocal enabledelayedexpansion
cd /d "%~dp0.."

set SRCS=
for %%f in (app\*.c sim\*.c hal\tc_nv.c hal\tc_modbus.c hal\dronecan.c hal\can_act.c hal\can_safe.c hal\can_relay.c hal\libcanard\canard.c) do set SRCS=!SRCS! %%f

echo Compiling:%SRCS%
gcc -std=c99 -O2 -Wall -Wextra -DCANARD_64_BIT=1 -Iapp -Ihal %SRCS% -o sim\autopilot_sim.exe -lm -lws2_32
if errorlevel 1 (
    echo BUILD FAILED
    exit /b 1
)
echo BUILD OK: sim\autopilot_sim.exe
echo Run it from the sim folder:  cd sim  then  autopilot_sim.exe
