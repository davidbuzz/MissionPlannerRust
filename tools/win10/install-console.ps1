# Installs the tiny10 VM's console window (console-runner.ps1) as the ClaudeConsole task: at
# every logon of this user, in their session - so the window is on the desktop - and elevated, as
# the SSH sessions that ran the earlier jobs were, with no time limit. Starts it now as well. Run
# over SSH with console-runner.ps1 and console-wait.ps1 already in C:\setup.
#
# The queue it runs from is closed to all but Administrators and SYSTEM: whatever lands there is
# run elevated.
$queue = 'C:\setup\queue'
New-Item -ItemType Directory -Force -Path $queue, (Join-Path $queue 'done') | Out-Null
icacls $queue /inheritance:r /grant:r '*S-1-5-32-544:(OI)(CI)F' '*S-1-5-18:(OI)(CI)F' | Out-Null

$action = New-ScheduledTaskAction -Execute 'powershell.exe' -Argument '-NoProfile -NoExit -ExecutionPolicy Bypass -File C:\setup\console-runner.ps1'
$principal = New-ScheduledTaskPrincipal -UserId $env:USERNAME -LogonType Interactive -RunLevel Highest
$trigger = New-ScheduledTaskTrigger -AtLogOn -User $env:USERNAME
$settings = New-ScheduledTaskSettingsSet -ExecutionTimeLimit ([TimeSpan]::Zero) -AllowStartIfOnBatteries -DontStopIfGoingOnBatteries -MultipleInstances IgnoreNew
Register-ScheduledTask -TaskName ClaudeConsole -Action $action -Principal $principal -Trigger $trigger -Settings $settings -Force | Out-Null
Start-ScheduledTask -TaskName ClaudeConsole
Start-Sleep 3
"ClaudeConsole: $((Get-ScheduledTask -TaskName ClaudeConsole).State)"
