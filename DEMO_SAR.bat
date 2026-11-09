@echo off
rem The SAR demo (MOB, UAV, search pattern, mark from video, ADS-B helicopter): the demo + the presenter guide window. MANUAL.txt: SAR DEMO.
rem Started with pythonw (no console window): the start-up screen shows the progress and any error;
rem the messages also go to demo_run\logs\demo.log. Without pythonw: the old way, in this window.
title Trygons C2 SAR demo
rem Python: %PYTHON%, else MSYS2's (MANUAL chapter 3), else "python" on the PATH
cd /d "%~dp0"
set "PY=%PYTHON%"
if not defined PY if exist "C:\msys64\ucrt64\bin\python.exe" set "PY=C:\msys64\ucrt64\bin\python.exe"
if not defined PY python -c "import sys" >nul 2>nul && set "PY=python"
if not defined PY (
  echo Python was not found. Install it as in MANUAL.txt chapter 3, or set PYTHON=C:\path\to\python.exe
  pause
  exit /b 1
)
rem pythonw (no console) ONLY next to the checked Python - the one that has tkinter + Pillow for the
rem start-up screen (13i2: never another Python found on the PATH); otherwise the console way below
set "PYW="
set "PYEXE="
for /f "delims=" %%I in ('call "%PY%" -c "import sys, tkinter, PIL; print(sys.executable)" 2^>nul') do set "PYEXE=%%I"
if defined PYEXE for %%I in ("%PYEXE%") do if exist "%%~dpIpythonw.exe" set "PYW=%%~dpIpythonw.exe"
if defined PYW (
  start "" "%PYW%" demo\demo.py start --sar --splash
  exit /b 0
)
"%PY%" demo\demo.py start --sar --splash
if errorlevel 1 (
  echo.
  echo The demo did not start - see the messages above.
  pause
  exit /b 1
)
echo.
echo This window can be closed. STOP_DEMO.bat stops everything.
timeout /t 15
