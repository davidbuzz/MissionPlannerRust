# Starts install-toolchain.ps1 in the tiny10 VM as a scheduled task: it then runs elevated
# (RunLevel Highest, no consent prompt) and outlives the SSH session that started it, which a
# Start-Process child did not (2026-09-26). The task runs as the logged-in user; the VM is
# always logged in when this is used. Watch C:\setup\install.log for "done".
$action = New-ScheduledTaskAction -Execute 'powershell.exe' -Argument '-NoProfile -ExecutionPolicy Bypass -File C:\setup\install-toolchain.ps1'
$principal = New-ScheduledTaskPrincipal -UserId $env:USERNAME -RunLevel Highest -LogonType Interactive
$settings = New-ScheduledTaskSettingsSet -ExecutionTimeLimit (New-TimeSpan -Hours 3) -AllowStartIfOnBatteries -DontStopIfGoingOnBatteries
Unregister-ScheduledTask -TaskName InstallToolchain -Confirm:$false -ErrorAction SilentlyContinue
Register-ScheduledTask -TaskName InstallToolchain -Action $action -Principal $principal -Settings $settings | Out-Null
Start-ScheduledTask -TaskName InstallToolchain
Start-Sleep 5
(Get-ScheduledTask -TaskName InstallToolchain).State
