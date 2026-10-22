@echo off
rem The whole C2 on this PC (simulator, gateway, cameras, AIS, ground stations). MANUAL.txt: DEMO.
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
"%PY%" demo\demo.py start
if errorlevel 1 (
  echo.
  echo The demo did not start - see the messages above.
  pause
  exit /b 1
)
echo.
echo This window can be closed. STOP_DEMO.bat stops everything.
timeout /t 15
