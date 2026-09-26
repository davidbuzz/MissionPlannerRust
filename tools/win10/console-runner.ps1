# The tiny10 VM's console for work sent from the laptop: a PowerShell window on the logged-in
# desktop that runs each job dropped into C:\setup\queue, one at a time in name order, showing the
# job's text and then its output as it comes, so the work can be watched from the VM's console.
#
# Started at logon by the ClaudeConsole task (install-console.ps1); jobs arrive through
# tools/win10/vm-run.sh, which waits on console-wait.ps1 for the output and the exit code. A job's
# output is kept in queue\done\<name>.out and its exit code in <name>.exit.
#
# A job is run by a PowerShell of its own, in this window's console and session, so anything it
# starts - the planner's window - opens on this desktop too, and nothing it sets or defines (an
# environment variable, an Add-Type) outlives it. Its output is shown line by line.
$ErrorActionPreference = 'Continue'
$queue = 'C:\setup\queue'
$done = Join-Path $queue 'done'
New-Item -ItemType Directory -Force -Path $queue, $done | Out-Null
$PID | Set-Content -Path (Join-Path $queue 'runner.pid')

$ui = $Host.UI.RawUI
$ui.WindowTitle = 'Claude at work - jobs from the laptop'
try {
    $ui.BufferSize = New-Object Management.Automation.Host.Size(150, 9999)
    $ui.WindowSize = New-Object Management.Automation.Host.Size(130, 40)
} catch {}

Write-Host 'Work sent from the laptop runs here, one job at a time: the job, then its output.' -ForegroundColor Cyan
Write-Host 'Closing this window stops it; it opens again at the next logon (task ClaudeConsole).' -ForegroundColor DarkCyan

# One line of a job's output, whatever stream it came from, as text.
function Format-Line($item) {
    if ($item -is [string]) { return $item }
    # A native command's stderr line, which Windows PowerShell 5.1 wraps as an error record: the
    # line is its TargetObject (a blank line's ToString is the exception's type name).
    if ($item -is [Management.Automation.ErrorRecord]) {
        if ($item.TargetObject -is [string]) { return $item.TargetObject }
        return $item.ToString()
    }
    if ($item -is [Management.Automation.InformationRecord]) { return "$($item.MessageData)" }
    if ($item -is [Management.Automation.WarningRecord]) { return "WARNING: $($item.Message)" }
    if ($item -is [Management.Automation.VerboseRecord]) { return "VERBOSE: $($item.Message)" }
    return ($item | Out-String -Width 150).TrimEnd()
}

while ($true) {
    $job = Get-ChildItem -Path $queue -Filter '*.ps1' | Sort-Object Name | Select-Object -First 1
    if (-not $job) {
        Start-Sleep -Milliseconds 400
        continue
    }
    $name = $job.BaseName
    $out = Join-Path $done "$name.out"
    $exitFile = Join-Path $done "$name.exit"
    Write-Host ''
    Write-Host ('==== {0}  {1}' -f (Get-Date -Format 'HH:mm:ss'), $name) -ForegroundColor Cyan
    Get-Content -Path $job.FullName | ForEach-Object { Write-Host "  | $_" -ForegroundColor DarkGray }
    Write-Host '  ----' -ForegroundColor DarkGray

    $stream = [IO.File]::Open($out, 'Create', 'Write', 'ReadWrite')
    $writer = New-Object IO.StreamWriter($stream, (New-Object Text.UTF8Encoding($false)))
    $writer.AutoFlush = $true
    $started = Get-Date
    $global:LASTEXITCODE = 0
    $code = 0
    try {
        & powershell.exe -NoProfile -ExecutionPolicy Bypass -File $job.FullName 2>&1 | ForEach-Object {
            $line = Format-Line $_
            if ($_ -is [Management.Automation.ErrorRecord]) { Write-Host $line -ForegroundColor Yellow } else { Write-Host $line }
            $writer.WriteLine($line)
        }
        if ($null -ne $global:LASTEXITCODE) { $code = $global:LASTEXITCODE }
    } catch {
        $line = "the job stopped: $($_.Exception.Message)"
        Write-Host $line -ForegroundColor Red
        $writer.WriteLine($line)
        $code = 1
    }
    $writer.Close()
    Move-Item -Path $job.FullName -Destination (Join-Path $done "$name.ps1") -Force
    "$code" | Set-Content -Path $exitFile
    $seconds = [int]((Get-Date) - $started).TotalSeconds
    $colour = if ($code -eq 0) { 'Green' } else { 'Red' }
    Write-Host ('==== {0}  exit {1} after {2} s' -f (Get-Date -Format 'HH:mm:ss'), $code, $seconds) -ForegroundColor $colour
}
