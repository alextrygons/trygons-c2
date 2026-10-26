@echo off
rem Builds the PC simulation: app\ + sim\ (no hardware code) + hal\tc_nv.c
rem (the board's data-flash record log, pure C: sim test DF4 drives it).
rem Usage: double-click, or run  sim\build_sim.bat  from any folder.
rem Result: sim\autopilot_sim.exe

setlocal enabledelayedexpansion
cd /d "%~dp0.."

set SRCS=
for %%f in (app\*.c sim\*.c hal\tc_nv.c) do set SRCS=!SRCS! %%f

echo Compiling:%SRCS%
gcc -std=c99 -O2 -Wall -Wextra -Iapp -Ihal %SRCS% -o sim\autopilot_sim.exe -lm -lws2_32
if errorlevel 1 (
    echo BUILD FAILED
    exit /b 1
)
echo BUILD OK: sim\autopilot_sim.exe
echo Run it from the sim folder:  cd sim  then  autopilot_sim.exe
