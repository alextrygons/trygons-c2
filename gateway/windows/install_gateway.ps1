# Trygons gateway PC - Windows 11 setup (README section 3W). Run ONCE in an
# ADMIN PowerShell from the cloned repository:
#   powershell -ExecutionPolicy Bypass -File gateway\windows\install_gateway.ps1
# Optional: -Python C:\path\python.exe   -SkipPower   -SkipFirewall   -SkipUpdates
# Safe to run again. It never touches the AURIX (that only sees the USB-UART adapter).
# Undo: uninstall_gateway.ps1 (keeps your data and gateway.json).
param([string]$Python = "", [switch]$SkipPower, [switch]$SkipFirewall, [switch]$SkipUpdates)
$ErrorActionPreference = "Stop"
$Repo = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$Data = Join-Path $env:PROGRAMDATA "trygons"
$Task = "Trygons Gateway"

$admin = ([Security.Principal.WindowsPrincipal][Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole(
    [Security.Principal.WindowsBuiltInRole]::Administrator)
if (-not $admin) { Write-Host "Run this in an ADMIN PowerShell (right click - Run as administrator)."; exit 1 }

Write-Host "== Python (needs pyserial)"
if (-not $Python) { $Python = $env:PYTHON }
if (-not $Python -and (Test-Path "C:\msys64\ucrt64\bin\python.exe")) { $Python = "C:\msys64\ucrt64\bin\python.exe" }
if (-not $Python) { $c = Get-Command python -ErrorAction SilentlyContinue; if ($c) { $Python = $c.Source } }
if (-not $Python) { Write-Host "Python not found: install it (MANUAL.txt chapter 3) or pass -Python"; exit 1 }
& $Python -c "import serial" 2>$null
if ($LASTEXITCODE -ne 0) { Write-Host "pyserial missing: $Python -m pip install pyserial   (then run this again)"; exit 1 }
Write-Host "   $Python"

Write-Host "== folders $Data"
New-Item -ItemType Directory -Force (Join-Path $Data "data"), (Join-Path $Data "logs") | Out-Null
$cfg = Join-Path $Data "gateway.json"; $sec = Join-Path $Data "secrets.json"
if (-not (Test-Path $cfg)) { Copy-Item (Join-Path $Repo "gateway\config.example.json") $cfg }
if (-not (Test-Path $sec)) { Copy-Item (Join-Path $Repo "gateway\secrets.example.json") $sec }
# secrets.json: only SYSTEM and the administrators may read it (= chmod 600)
icacls $sec /inheritance:r /grant:r "*S-1-5-18:F" "*S-1-5-32-544:F" | Out-Null

Write-Host "== Task Scheduler task '$Task' (at boot, as SYSTEM, restarts, keeps running on battery)"
$bat = Join-Path $Repo "gateway\windows\run_gateway.bat"
$act = New-ScheduledTaskAction -Execute "cmd.exe" -Argument "/c `"`"$bat`" `"$Python`" `"$cfg`"`""
$trg = New-ScheduledTaskTrigger -AtStartup
$pri = New-ScheduledTaskPrincipal -UserId "SYSTEM" -LogonType ServiceAccount -RunLevel Highest
# the battery is the UPS: the task must NOT stop when the PC runs on battery
$set = New-ScheduledTaskSettingsSet -AllowStartIfOnBatteries -DontStopIfGoingOnBatteries -StartWhenAvailable `
    -ExecutionTimeLimit ([TimeSpan]::Zero) -RestartCount 999 -RestartInterval (New-TimeSpan -Minutes 1) `
    -MultipleInstances IgnoreNew
Register-ScheduledTask -TaskName $Task -Action $act -Trigger $trg -Principal $pri -Settings $set -Force | Out-Null

if (-not $SkipPower) {
    Write-Host "== power: never sleep, lid closed = keep running, USB never suspended (AC and battery)"
    powercfg /change standby-timeout-ac 0; powercfg /change standby-timeout-dc 0
    powercfg /change hibernate-timeout-ac 0; powercfg /change hibernate-timeout-dc 0
    powercfg /setacvalueindex SCHEME_CURRENT SUB_BUTTONS LIDACTION 0
    powercfg /setdcvalueindex SCHEME_CURRENT SUB_BUTTONS LIDACTION 0
    # USB selective suspend off (the USB-UART adapter, the AIS receiver, the Ethernet adapters)
    powercfg /setacvalueindex SCHEME_CURRENT 2a737441-1930-4402-8d77-b2bebba308a3 48e6b7a6-50f5-4782-a5d4-53bb8f07e226 0
    powercfg /setdcvalueindex SCHEME_CURRENT 2a737441-1930-4402-8d77-b2bebba308a3 48e6b7a6-50f5-4782-a5d4-53bb8f07e226 0
    powercfg /setactive SCHEME_CURRENT
    powercfg /hibernate off
}

if (-not $SkipFirewall) {
    Write-Host "== firewall: gateway ports only from Tailscale (100.64.0.0/10) and the boat LAN (10.42.0.0/24)"
    Get-NetFirewallRule -Group "Trygons gateway" -ErrorAction SilentlyContinue | Remove-NetFirewallRule
    $from = @("100.64.0.0/10", "10.42.0.0/24")
    New-NetFirewallRule -DisplayName "Trygons gateway TCP" -Group "Trygons gateway" -Direction Inbound `
        -Protocol TCP -LocalPort 5761, 8080, 8554, 8889 -RemoteAddress $from -Action Allow | Out-Null
    New-NetFirewallRule -DisplayName "Trygons gateway UDP" -Group "Trygons gateway" -Direction Inbound `
        -Protocol UDP -LocalPort 5761, 8189 -RemoteAddress $from -Action Allow | Out-Null
}

if (-not $SkipUpdates) {
    Write-Host "== Windows Update: ask before download, never restart by itself (Windows 11 Pro)"
    $au = "HKLM:\SOFTWARE\Policies\Microsoft\Windows\WindowsUpdate\AU"
    New-Item -Force $au | Out-Null
    Set-ItemProperty $au -Name AUOptions -Type DWord -Value 2
    Set-ItemProperty $au -Name NoAutoRebootWithLoggedOnUsers -Type DWord -Value 1
}

Write-Host ""
Write-Host "Done. Next (README section 5W):"
Write-Host "  $Python $Repo\gateway\gateway.py --list-serial      -> names into $cfg"
Write-Host "  $Python $Repo\gateway\gateway.py --config $cfg --check-config"
Write-Host "  Start-ScheduledTask '$Task'   (log: $Data\logs\gateway.log)"
Write-Host "By hand in Lenovo Vantage: Device - Power - Battery Charge Threshold ON, stop at 80 %."
