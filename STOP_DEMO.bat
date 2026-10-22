@echo off
rem Stops everything START_DEMO.bat started.
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
"%PY%" demo\demo.py stop
timeout /t 5
