@echo off
rem Trygons gateway on a Windows 11 gateway PC - the Windows twin of systemd's
rem "Restart=always": runs gateway.py, and starts it again 2 s after it stops.
rem Started at boot by the Task Scheduler task "Trygons Gateway" (install_gateway.ps1);
rem by hand: run_gateway.bat [python.exe] [gateway.json]   (Ctrl+C, then Y, to stop).
rem The log goes to %PROGRAMDATA%\trygons\logs\gateway.log (gateway.old.log = the one before).
setlocal
for %%I in ("%~dp0..") do set "GWDIR=%%~fI"
cd /d "%GWDIR%"
set "PY=%~1"
if not defined PY set "PY=%PYTHON%"
if not defined PY if exist "C:\msys64\ucrt64\bin\python.exe" set "PY=C:\msys64\ucrt64\bin\python.exe"
if not defined PY set "PY=python"
set "CFG=%~2"
if not defined CFG set "CFG=%PROGRAMDATA%\trygons\gateway.json"
set "LOGDIR=%PROGRAMDATA%\trygons\logs"
if not exist "%LOGDIR%" mkdir "%LOGDIR%"
:again
if exist "%LOGDIR%\gateway.log" for %%F in ("%LOGDIR%\gateway.log") do if %%~zF GTR 50000000 move /y "%LOGDIR%\gateway.log" "%LOGDIR%\gateway.old.log" >nul
echo %date% %time% starting the gateway (%PY%, %CFG%)>> "%LOGDIR%\gateway.log"
"%PY%" "%GWDIR%\gateway.py" --config "%CFG%" >> "%LOGDIR%\gateway.log" 2>&1
echo %date% %time% the gateway stopped (exit %errorlevel%), restart in 2 s>> "%LOGDIR%\gateway.log"
timeout /t 2 /nobreak >nul
goto again
