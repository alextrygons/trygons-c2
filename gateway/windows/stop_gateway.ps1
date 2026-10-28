# Stops the Windows gateway (ADMIN PowerShell): the task, its restart loop and gateway.py.
#   powershell -ExecutionPolicy Bypass -File gateway\windows\stop_gateway.ps1
# Start it again: Start-ScheduledTask "Trygons Gateway" (or reboot).
# Only processes whose command line is THIS repository's run_gateway.bat / gateway\gateway.py
# are stopped - never other Python programs (e.g. the ground station).
$ErrorActionPreference = "Continue"
Stop-ScheduledTask -TaskName "Trygons Gateway" -ErrorAction SilentlyContinue
$repo = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
Get-CimInstance Win32_Process | Where-Object {
    $_.CommandLine -and $_.CommandLine.Contains($repo) -and
    ($_.CommandLine -match 'run_gateway\.bat' -or $_.CommandLine -match 'gateway\.py"?\s+--config')
} | ForEach-Object { Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue; Write-Host "stopped $($_.ProcessId)" }
