# Removes the Windows gateway task and its firewall rules (ADMIN PowerShell):
#   powershell -ExecutionPolicy Bypass -File gateway\windows\uninstall_gateway.ps1
# Keeps %PROGRAMDATA%\trygons (data, gateway.json, secrets.json) and the power settings.
$ErrorActionPreference = "Continue"
& (Join-Path $PSScriptRoot "stop_gateway.ps1")
Unregister-ScheduledTask -TaskName "Trygons Gateway" -Confirm:$false -ErrorAction SilentlyContinue
Get-NetFirewallRule -Group "Trygons gateway" -ErrorAction SilentlyContinue | Remove-NetFirewallRule
Write-Host "removed the task 'Trygons Gateway' and the firewall rules; data kept in $env:PROGRAMDATA\trygons"
