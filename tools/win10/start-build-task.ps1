# Starts build-planner.ps1 in the tiny10 VM as a scheduled task, so it outlives the SSH session
# that started it. Watch C:\setup\build.log for "done"; the compiler's output is in
# C:\setup\cargo-build.log.
$action = New-ScheduledTaskAction -Execute 'powershell.exe' -Argument '-NoProfile -ExecutionPolicy Bypass -File C:\setup\build-planner.ps1'
$principal = New-ScheduledTaskPrincipal -UserId $env:USERNAME -LogonType Interactive
$settings = New-ScheduledTaskSettingsSet -ExecutionTimeLimit (New-TimeSpan -Hours 3) -AllowStartIfOnBatteries -DontStopIfGoingOnBatteries
Unregister-ScheduledTask -TaskName BuildPlanner -Confirm:$false -ErrorAction SilentlyContinue
Register-ScheduledTask -TaskName BuildPlanner -Action $action -Principal $principal -Settings $settings | Out-Null
Start-ScheduledTask -TaskName BuildPlanner
Start-Sleep 5
(Get-ScheduledTask -TaskName BuildPlanner).State
