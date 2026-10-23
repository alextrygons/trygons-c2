@echo off
rem Builds the fault-storm runner: app\ + sim\ (unchanged, without sim_main.c) + storm.c
rem Result: test_results\storm.exe
setlocal enabledelayedexpansion
cd /d "%~dp0..\..\.."
if not exist test_results mkdir test_results
set SRCS=
for %%f in (app\*.c) do set SRCS=!SRCS! %%f
for %%f in (sim\*.c) do if /I not "%%~nxf"=="sim_main.c" set SRCS=!SRCS! %%f
gcc -std=c99 -O2 -Wall -Wextra -Iapp -Ihal -Isim %SRCS% tests\acceptance\storm\storm.c -o test_results\storm.exe -lm -lws2_32
if errorlevel 1 (
    echo BUILD FAILED
    exit /b 1
)
echo BUILD OK: test_results\storm.exe
