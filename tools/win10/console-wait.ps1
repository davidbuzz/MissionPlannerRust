# The laptop's end of a console job (vm-run.sh runs this over SSH): releases the job it copied
# into C:\setup\queue as <name>.part to the console window (console-runner.ps1), streams the job's
# output back as the window shows it, and exits with the job's exit code - 125 when the window is
# not running, as it is not until someone logs on at the VM.
param([Parameter(Mandatory = $true)][string]$Name)
$queue = 'C:\setup\queue'
$done = Join-Path $queue 'done'
$out = Join-Path $done "$Name.out"
$exitFile = Join-Path $done "$Name.exit"

function Test-Runner {
    $pidFile = Join-Path $queue 'runner.pid'
    if (-not (Test-Path $pidFile)) { return $false }
    $id = [int](Get-Content -Path $pidFile)
    return [bool](Get-Process -Id $id -ErrorAction SilentlyContinue)
}

if (-not (Test-Runner)) {
    Remove-Item -Path (Join-Path $queue "$Name.part") -ErrorAction SilentlyContinue
    Write-Output 'The VM console window is not running: log on at the VM (the ClaudeConsole task starts it), or run install-console.ps1.'
    exit 125
}
Rename-Item -Path (Join-Path $queue "$Name.part") -NewName "$Name.ps1"

$position = 0
while ($true) {
    $finished = Test-Path $exitFile
    if (Test-Path $out) {
        $stream = [IO.File]::Open($out, 'Open', 'Read', 'ReadWrite')
        [void]$stream.Seek($position, 'Begin')
        $reader = New-Object IO.StreamReader($stream, (New-Object Text.UTF8Encoding($false)))
        $text = $reader.ReadToEnd()
        $position = $stream.Position
        $reader.Close()
        if ($text) { [Console]::Out.Write($text); [Console]::Out.Flush() }
    }
    if ($finished) { exit [int](Get-Content -Path $exitFile) }
    if (-not (Test-Runner)) {
        Write-Output 'The VM console window closed while the job ran.'
        exit 125
    }
    Start-Sleep -Milliseconds 500
}
