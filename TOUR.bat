@echo off
rem The guided tour of the station (academy/tour_app.py, Greek) - no demo, no boat, no gateway needed.
rem MANUAL.txt: THE GUIDED TOUR. Started with pythonw (no console window) next to the checked Python.
title Trygons C2 tour
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
rem pythonw (no console) ONLY next to the checked Python - the one that has tkinter + Pillow (never another
rem pythonw found on the PATH, as START_DEMO.bat); otherwise the console way below
set "PYW="
set "PYEXE="
for /f "delims=" %%I in ('call "%PY%" -c "import sys, tkinter, PIL; print(sys.executable)" 2^>nul') do set "PYEXE=%%I"
if defined PYEXE for %%I in ("%PYEXE%") do if exist "%%~dpIpythonw.exe" set "PYW=%%~dpIpythonw.exe"
if defined PYW (
  start "" "%PYW%" academy\tour_app.py --lang el
  exit /b 0
)
"%PY%" academy\tour_app.py --lang el
if errorlevel 1 (
  echo.
  echo The tour did not start - see the messages above.
  pause
  exit /b 1
)
