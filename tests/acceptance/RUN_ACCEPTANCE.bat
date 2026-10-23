@echo off
rem The acceptance test with one command (MANUAL.txt chapter "TESTING").
rem   RUN_ACCEPTANCE.bat          quick (about 10-15 min)
rem   RUN_ACCEPTANCE.bat full     everything (several hours)
rem Results: test_results\acceptance_summary.txt (+ the logs next to it). Report: TEST_REPORT.md
cd /d "%~dp0..\.."
set "PY=%PYTHON%"
if not defined PY if exist "C:\msys64\ucrt64\bin\python.exe" set "PY=C:\msys64\ucrt64\bin\python.exe"
if not defined PY python -c "import sys" >nul 2>nul && set "PY=python"
if not defined PY (
  echo Python was not found. Install it as in MANUAL.txt chapter 3, or set PYTHON=C:\path\to\python.exe
  pause
  exit /b 1
)
set "MODE=%~1"
if not defined MODE set "MODE=quick"
"%PY%" tests\acceptance\run_acceptance.py %MODE%
echo.
echo Done - see test_results\acceptance_summary.txt
pause
