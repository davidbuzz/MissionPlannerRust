# Builds the planner in the tiny10 VM: the debug binary of mp-gui, logged to C:\setup\build.log.
# Run detached (start-build-task.ps1) so an SSH session's end does not end it. The Build Tools'
# environment is what rustup's cargo finds through vswhere; nothing else is on the path.
$ErrorActionPreference = 'Continue'
$log = 'C:\setup\build.log'
function Log($m) { "$(Get-Date -Format s) $m" | Tee-Object -FilePath $log -Append }
Log "start"
Set-Location C:\src\MissionPlannerRust
$env:CARGO_BUILD_JOBS = '5'
& "$env:USERPROFILE\.cargo\bin\cargo.exe" build -p mp-gui 2>&1 | Tee-Object -FilePath C:\setup\cargo-build.log
Log "cargo build exit $LASTEXITCODE"
if (Test-Path target\debug\planner.exe) { Log "planner.exe $((Get-Item target\debug\planner.exe).Length) bytes" }
Log "done"
