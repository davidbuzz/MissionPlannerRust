# Starts run-planner.ps1 in the tiny10 VM as a scheduled task in the logged-in user's session,
# so the window opens on the VM's desktop and the screenshot sees it. Watch C:\setup\run.log.
$action = New-ScheduledTaskAction -Execute 'powershell.exe' -Argument '-NoProfile -ExecutionPolicy Bypass -File C:\setup\run-planner.ps1'
$principal = New-ScheduledTaskPrincipal -UserId $env:USERNAME -LogonType Interactive
$settings = New-ScheduledTaskSettingsSet -ExecutionTimeLimit (New-TimeSpan -Minutes 10) -AllowStartIfOnBatteries -DontStopIfGoingOnBatteries
Unregister-ScheduledTask -TaskName RunPlanner -Confirm:$false -ErrorAction SilentlyContinue
Register-ScheduledTask -TaskName RunPlanner -Action $action -Principal $principal -Settings $settings | Out-Null
Start-ScheduledTask -TaskName RunPlanner
Start-Sleep 3
(Get-ScheduledTask -TaskName RunPlanner).State
