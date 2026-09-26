# The GUI suite on Windows: tools/gui-suite.sh's loop over tests/gui/*.gui, each script run by
# tools/win10/gui-test.ps1 with the argument its header names, one line per script -
# "name: PASS", "name: PASS over budget <s>", "name: FAIL <first failures>", "name: SKIP <why>" -
# then a count. The results go to C:\setup\suite\results.txt and the shared folder, to be set
# beside a Linux run of the same commit.
#
#   powershell -File tools\win10\gui-suite.ps1 -All
#   powershell -File tools\win10\gui-suite.ps1 -Names plan-fence-clear,config-radio
#
# A script linked to tcp:127.0.0.1:5760 gets this VM's own SITL there, started once and kept, as
# the Linux suite keeps tools/sitl's: the Cygwin ArduCopter the planner's SIMULATION page fetched
# into Documents\MissionPlannerRust\sitl (ArduPilot's published Stable, where the Linux suite runs
# a 4.8.0-dev build - a difference to look at before calling a failure Windows'), with
# tools/sitl/params/copter.parm, in a scratch directory. The laptop's SITL is never used.
param(
    [string[]]$Names = @(),
    [switch]$All,
    [string]$Root = 'C:\src\MissionPlannerRust',
    [string]$Out = 'C:\setup\suite',
    [int]$ScriptTimeout = 300,
    # gui-test.ps1's -TimeScale: 2 by default, so a script is failed for what it found and not for
    # a VM's pace; "PASS over budget" still marks one slower than its budget.
    [double]$TimeScale = 2
)
$ErrorActionPreference = 'Continue'
New-Item -ItemType Directory -Force -Path $Out | Out-Null
Set-Location $Root
# `-Names a,b` reaches a script run with -File as one string.
$Names = @($Names | ForEach-Object { $_ -split ',' } | Where-Object { $_ })
if ($All) {
    $Names = Get-ChildItem tests\gui\*.gui | Where-Object { $_.Name -notmatch '-bench\.gui$' } |
        ForEach-Object { $_.BaseName } | Sort-Object
}
$Sitl = Join-Path $env:USERPROFILE 'Documents\MissionPlannerRust\sitl\ArduCopter.exe'

# Whether something listens on 127.0.0.1:5760. Not a MAVLink read, as tools/sitl/start-sitl.sh
# makes on Linux: this SITL waits for its first client before it starts, and a check that was
# that client left the next - the planner - unheard (2026-09-26).
function Test-Sitl([int]$seconds = 5) {
    $deadline = (Get-Date).AddSeconds($seconds)
    do {
        if (netstat -an | Select-String -Pattern ':5760\s+\S+\s+LISTENING') { return $true }
        Start-Sleep -Milliseconds 250
    } while ((Get-Date) -lt $deadline)
    return $false
}
function Start-Sitl {
    if (Test-Sitl 2) { return $true }
    if (-not (Test-Path $Sitl)) { Write-Host "no SITL at ${Sitl}: the SIMULATION page fetches it"; return $false }
    $dir = Join-Path $env:TEMP ('suite-sitl-' + [guid]::NewGuid().ToString('N').Substring(0, 8))
    New-Item -ItemType Directory -Path $dir | Out-Null
    $params = Join-Path $Root 'tools\sitl\params\copter.parm'
    $script:SitlProcess = Start-Process -FilePath $Sitl -WorkingDirectory $dir -WindowStyle Minimized -PassThru `
        -ArgumentList @('--model', 'quad', '--speedup', '1', '--defaults', "`"$params`"")
    for ($i = 0; $i -lt 30; $i++) {
        Start-Sleep 2
        if (Test-Sitl 3) { Write-Host "SITL up (pid $($script:SitlProcess.Id), in $dir)"; return $true }
    }
    Write-Host 'SITL did not come up'
    return $false
}

$results = New-Object System.Collections.ArrayList
$counts = @{ PASS = 0; OVER = 0; FAIL = 0; SKIP = 0 }
$started = Get-Date
foreach ($name in $Names) {
    $script = "tests\gui\$name.gui"
    if (-not (Test-Path $script)) { $line = "${name}: FAIL no script"; $counts.FAIL++; [void]$results.Add($line); Write-Output $line; continue }
    $text = Get-Content -Raw $script
    # The header's command line names the argument; a "Run as" line with none is a script that
    # starts without a link on purpose; otherwise a port or log named in the header.
    $arg = ''
    $m = [regex]::Match($text, "gui-test\.sh tests/gui/$([regex]::Escape($name))\.gui -- (\S+)")
    if ($m.Success) { $arg = $m.Groups[1].Value }
    elseif (-not [regex]::IsMatch($text, "gui-test\.sh tests/gui/$([regex]::Escape($name))\.gui[ \t]*(\r?\n|$)")) {
        $m = [regex]::Match($text, '(tcp:127\.0\.0\.1:5760|file:[^ ,\r\n]+\.tlog)')
        if ($m.Success) { $arg = $m.Groups[1].Value }
    }
    if ($arg -eq 'tcp:127.0.0.1:5760' -and -not (Start-Sitl)) {
        $line = "${name}: SKIP no SITL"; $counts.SKIP++; [void]$results.Add($line); Write-Output $line; continue
    }
    $log = Join-Path $Out "gui-$name.log"
    $argsList = @('-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', 'tools\win10\gui-test.ps1', '-Script', $script, '-TimeScale', $TimeScale)
    if ($arg) { $argsList += @('-Link', $arg) }
    $run = Start-Process powershell -ArgumentList $argsList -NoNewWindow -PassThru -RedirectStandardOutput $log -RedirectStandardError "$log.err"
    # Windows PowerShell reports no ExitCode for a process whose handle was not read while it ran.
    $null = $run.Handle
    if (-not $run.WaitForExit($ScriptTimeout * 1000)) {
        Stop-Process -Id $run.Id -Force -ErrorAction SilentlyContinue
        Get-Process planner -ErrorAction SilentlyContinue | Stop-Process -Force -ErrorAction SilentlyContinue
        $line = "${name}: FAIL timed out after $ScriptTimeout s"; $counts.FAIL++
    } else {
        $body = (Get-Content $log -ErrorAction SilentlyContinue) + (Get-Content "$log.err" -ErrorAction SilentlyContinue)
        $took = ($body | Select-String -Pattern '^took ([0-9.]+) s' | Select-Object -Last 1)
        $took = if ($took) { $took.Matches[0].Groups[1].Value } else { '?' }
        switch ($run.ExitCode) {
            0 { $line = "${name}: PASS $took s"; $counts.PASS++ }
            4 { $line = "${name}: PASS over budget $took s"; $counts.OVER++ }
            3 { $why = ($body | Select-String -Pattern '^SKIP' | Select-Object -First 1); $line = "${name}: SKIP $why"; $counts.SKIP++ }
            default {
                $first = ($body | Select-String -Pattern '^FAIL|could not|setup exited|app exited|no window' | Select-Object -First 2 | ForEach-Object { $_.Line.Trim() }) -join ' | '
                $line = "${name}: FAIL $first"; $counts.FAIL++
            }
        }
    }
    [void]$results.Add($line)
    Write-Output $line
}
$summary = "suite done in {0:N0} s: {1} pass, {2} pass over budget, {3} fail, {4} skip, of {5}" -f `
    ((Get-Date) - $started).TotalSeconds, $counts.PASS, $counts.OVER, $counts.FAIL, $counts.SKIP, $Names.Count
Write-Output $summary
[void]$results.Add($summary)
$results | Set-Content (Join-Path $Out 'results.txt')
Copy-Item (Join-Path $Out 'results.txt') \\VBOXSVR\vmshare\win-suite-results.txt -ErrorAction SilentlyContinue
if ($script:SitlProcess -and -not $script:SitlProcess.HasExited) { Stop-Process -Id $script:SitlProcess.Id -Force -ErrorAction SilentlyContinue }
exit ([int]($counts.FAIL -gt 0))
