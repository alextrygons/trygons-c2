@echo off
rem The whole C2 on this PC (simulator, gateway, cameras, AIS, ground stations). MANUAL.txt: DEMO.
rem Started with pythonw (no console window): the start-up screen shows the progress and any error;
rem the messages also go to demo_run\logs\demo.log. Without pythonw: the old way, in this window.
title Trygons C2 demo
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
set "PYW="
for %%I in ("%PY%") do if exist "%%~dpIpythonw.exe" set "PYW=%%~dpIpythonw.exe"
if not defined PYW for /f "delims=" %%I in ('where pythonw 2^>nul') do if not defined PYW set "PYW=%%I"
if defined PYW (
  start "" "%PYW%" demo\demo.py start --splash
  exit /b 0
)
"%PY%" demo\demo.py start --splash
if errorlevel 1 (
  echo.
  echo The demo did not start - see the messages above.
  pause
  exit /b 1
)
echo.
echo This window can be closed. STOP_DEMO.bat stops everything.
timeout /t 15
