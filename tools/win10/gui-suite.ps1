# The GUI suite on Windows: tools/gui-suite.sh's loop over tests/gui/*.gui, each script run by
# tools/win10/gui-test.ps1 with the argument its header names, one line per script -
# "name: PASS", "name: PASS over budget <s>", "name: FAIL <first failures>", "name: SKIP <why>" -
# then a count. The results go to C:\setup\suite\results.txt and the shared folder, to be set
# beside a Linux run of the same commit.
#
#   powershell -File tools\win10\gui-suite.ps1 -All
#   powershell -File tools\win10\gui-suite.ps1 -Names plan-fence-clear,config-radio
#
# A script linked to tcp:127.0.0.1:5760, or whose header says it needs SITL, gets this VM's own
# SITL there, one for the whole suite as the Linux suite keeps tools/sitl's one SITL - kept
# running across scripts by tools\win10\sitl-relay.ps1: the Cygwin ArduCopter the planner's
# SIMULATION page fetched
# into Documents\MissionPlannerRust\sitl (ArduPilot's published Stable, where the Linux suite runs
# a 4.8.0-dev build - a difference to look at before calling a failure Windows'), with
# tools/sitl/params/copter.parm, in a scratch directory. The laptop's SITL is never used.
param(
    [string[]]$Names = @(),
    [switch]$All,
    [string]$Root = 'C:\src\MissionPlannerRust',
    [string]$Out = 'C:\setup\suite',
    [int]$ScriptTimeout = 300,
    # gui-test.ps1's -TimeScale, on top of its -BudgetScale 2: 1, so a script stops 3 s after its
    # doubled budget, as the Linux runner stops one 3 s after its budget. It was 2, which with the
    # doubling ran a failing script to four times its budget - plan-text's 28 s to a 115 s hard
    # stop - and the owner asked for Windows to give up as quickly as Linux (2026-09-26).
    [double]$TimeScale = 1
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

# One SITL directory for the suite, as the Linux suite runs one SITL throughout: gui-test.ps1
# starts the simulator there whenever nothing listens on 5760, so its eeprom - the parameters
# scripts set - carries from script to script.
$SitlDir = Join-Path $env:TEMP ('suite-sitl-' + (Get-Date -Format 'yyyyMMdd-HHmmss'))
$SitlParams = Join-Path $Root 'tools\sitl\params\copter.parm'

# One SITL for the suite, behind tools\win10\sitl-relay.ps1, started for the first script that
# needs it: the Cygwin build exits when its client leaves, and the relay keeps it running from one
# script to the next as the Linux suite's runs. Any SITL or relay an earlier run left is stopped.
Get-CimInstance Win32_Process -Filter "Name = 'powershell.exe'" |
    Where-Object { $_.CommandLine -match 'sitl-relay\.ps1' } |
    ForEach-Object { Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue }
Get-Process ArduCopter -ErrorAction SilentlyContinue | Where-Object { $_.Path -eq $Sitl } | Stop-Process -Force -ErrorAction SilentlyContinue
$relay = $null
function Start-Relay {
    $relayArgs = @('-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', 'tools\win10\sitl-relay.ps1',
        '-SitlExe', "`"$Sitl`"", '-SitlDir', "`"$SitlDir`"", '-SitlParams', "`"$SitlParams`"")
    $process = Start-Process powershell -ArgumentList $relayArgs -WindowStyle Minimized -PassThru
    $status = Join-Path $SitlDir 'relay.status'
    for ($i = 0; $i -lt 120 -and -not (Test-Path $status); $i++) { Start-Sleep -Milliseconds 250 }
    Write-Host "SITL relay $($process.Id): $(if (Test-Path $status) { Get-Content $status } else { 'no SITL yet' })"
    return $process
}

$results = New-Object System.Collections.ArrayList
$counts = @{ PASS = 0; OVER = 0; FAIL = 0; SKIP = 0 }
# Each passing script's Windows budget from this run: its time, rounded up with one to spare; a
# script stopped by the hard stop alone, its budget and five more.
$measured = @{}
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
    # A script connecting to SITL through the screen's own boxes names no link (main-connect), and
    # says "Needs SITL" in its header instead.
    $needsSitl = $arg -eq 'tcp:127.0.0.1:5760' -or $text -match 'Needs SITL'
    # A script that launches its own SITL on 5760 (sitl-launch) gets the port free: the suite's
    # relay and its SITL stop, and the next script that needs SITL starts them again.
    if ($text -match 'launches its own SITL') {
        $needsSitl = $false
        if ($relay) {
            $status = Join-Path $SitlDir 'relay.status'
            $sitlPid = if (Test-Path $status) { ((Get-Content $status) -split ' ')[0] } else { $null }
            Stop-Process -Id $relay.Id -Force -ErrorAction SilentlyContinue
            if ($sitlPid) { Stop-Process -Id $sitlPid -Force -ErrorAction SilentlyContinue }
            Remove-Item $status -ErrorAction SilentlyContinue
            $relay = $null
            Start-Sleep -Seconds 1
        }
    }
    if ($needsSitl -and -not (Test-Path $Sitl)) {
        $line = "${name}: SKIP no SITL at $Sitl"; $counts.SKIP++; [void]$results.Add($line); Write-Output $line; continue
    }
    if ($needsSitl -and -not $relay) { $relay = Start-Relay }
    $log = Join-Path $Out "gui-$name.log"
    $argsList = @('-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', 'tools\win10\gui-test.ps1', '-Script', $script, '-TimeScale', $TimeScale)
    if ($arg) { $argsList += @('-Link', $arg) }
    if ($needsSitl) { $argsList += @('-SitlExe', "`"$Sitl`"", '-SitlDir', "`"$SitlDir`"", '-SitlParams', "`"$SitlParams`"") }
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
        if ($took -ne '?' -and ($run.ExitCode -eq 0 -or $run.ExitCode -eq 4)) {
            $seconds = [double]::Parse($took, [Globalization.CultureInfo]::InvariantCulture)
            $measured[$name] = [int][Math]::Ceiling($seconds) + 1
        }
        switch ($run.ExitCode) {
            0 { $line = "${name}: PASS $took s"; $counts.PASS++ }
            4 { $line = "${name}: PASS over budget $took s"; $counts.OVER++ }
            3 { $why = ($body | Select-String -Pattern '^SKIP' | Select-Object -First 1); $line = "${name}: SKIP $why"; $counts.SKIP++ }
            default {
                $first = ($body | Select-String -Pattern '^FAIL|could not|setup exited|app exited|no window' | Select-Object -First 2 | ForEach-Object { $_.Line.Trim() }) -join ' | '
                $line = "${name}: FAIL $first"; $counts.FAIL++
                # A run whose one failure is the hard stop found nothing wrong but ran out of time,
                # and so never records one: as tools/gui-budgets.py retime has it, its budget gets
                # the margin and two more, for the next run to record over (plan-survey's 180
                # lines, 2026-09-26).
                $fails = @($body | Where-Object { $_ -match '^FAIL' })
                $used = $body | Select-String -Pattern 'budget ([0-9.]+) s from here' | Select-Object -First 1
                if ($fails.Count -gt 0 -and $used -and @($fails | Where-Object { $_ -notmatch '^FAIL: hard stop' }).Count -eq 0) {
                    $seconds = [double]::Parse($used.Matches[0].Groups[1].Value, [Globalization.CultureInfo]::InvariantCulture)
                    $measured[$name] = [int][Math]::Ceiling($seconds) + 5
                }
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

# The Windows budgets, tools\win10\budgets.txt, which gui-test.ps1 reads: every script that passed
# in this run - over its budget or not - gets its time rounded up with one to spare, the rule
# tools/gui-budgets.py record keeps the Linux budgets by, and a run then stops three seconds past
# it; the rest keep what they had. The file's comment lines are kept. Written here and to the
# shared folder, from which it goes into the repository.
$budgetsFile = Join-Path $Root 'tools\win10\budgets.txt'
$comments = @(); $budgets = @{}
if (Test-Path $budgetsFile) {
    foreach ($entry in Get-Content $budgetsFile) {
        if ($entry.StartsWith('#')) { $comments += $entry }
        elseif ($entry -match '^(\S+)\s+([0-9.]+)\s*$') { $budgets[$Matches[1]] = [int][double]::Parse($Matches[2], [Globalization.CultureInfo]::InvariantCulture) }
    }
}
$new = 0; $changed = 0
foreach ($name in $measured.Keys) {
    if (-not $budgets.ContainsKey($name)) { $new++ } elseif ($budgets[$name] -ne $measured[$name]) { $changed++ }
    $budgets[$name] = $measured[$name]
}
# In ordinal order, as a sort on the laptop orders it, so the file changes only where a time does.
$names = [string[]]@($budgets.Keys)
[Array]::Sort($names, [StringComparer]::Ordinal)
$lines = $comments + @($names | ForEach-Object { "$_ $($budgets[$_])" })
[IO.File]::WriteAllLines($budgetsFile, [string[]]$lines)
Copy-Item $budgetsFile \\VBOXSVR\vmshare\win-budgets.txt -ErrorAction SilentlyContinue
$line = "budgets: $($measured.Count) measured in this run ($new new, $changed changed); $($budgets.Count) scripts have a Windows budget"
Write-Output $line
[void]$results.Add($line)
$results | Set-Content (Join-Path $Out 'results.txt')
Copy-Item (Join-Path $Out 'results.txt') \\VBOXSVR\vmshare\win-suite-results.txt -ErrorAction SilentlyContinue
if ($relay) { Stop-Process -Id $relay.Id -Force -ErrorAction SilentlyContinue }
Get-Process ArduCopter -ErrorAction SilentlyContinue | Where-Object { $_.Path -eq $Sitl } | Stop-Process -Force -ErrorAction SilentlyContinue
exit ([int]($counts.FAIL -gt 0))
