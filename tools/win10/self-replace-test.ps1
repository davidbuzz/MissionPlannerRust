# What Windows lets be done to the file of a program while that program runs: the facts
# updates.md's "the planner replaces itself" rests on (the owner's question of 2026-11-18, "can we
# test it?"). It uses only what Windows ships: a copy of cmd.exe stands for the running planner
# and a copy of hostname.exe for the new one downloaded beside it.
#
#   tools/win10/vm-run.sh self-replace tools/win10/self-replace-test.ps1
#
# Part A, another process acting on the running program's file: overwrite it, delete it, rename
# it, put the new file under the old name, start that, delete the renamed one. Part B, the
# running program doing the rename and the copy to itself, then starting what is now under its
# own name. Each line says what was expected and what Windows did; the exit code is the number of
# lines that went the other way.

$ErrorActionPreference = 'Stop'
$sys = Join-Path $env:SystemRoot 'System32'
$oldSource = Join-Path $sys 'cmd.exe'
$newSource = Join-Path $sys 'hostname.exe'
$root = Join-Path $env:PUBLIC 'mp-selfreplace'
if (Test-Path $root) { Remove-Item -Recurse -Force $root }
$unexpected = 0

function Hash($path) { (Get-FileHash -Algorithm SHA256 $path).Hash }

# One operation: what was expected of it, 'allowed' or 'refused', and what happened.
function Try-It([string]$what, [string]$expected, [scriptblock]$operation) {
    $got = 'allowed'
    $why = ''
    try { & $operation } catch { $got = 'refused'; $why = $_.Exception.GetBaseException().Message }
    $mark = if ($got -eq $expected) { 'ok  ' } else { $script:unexpected++; 'DIFF' }
    $line = "  $mark $what`: $got (expected $expected)"
    if ($why) { $line += " - $why" }
    Write-Output $line
}

# A fact that is simply true or not.
function Check([string]$what, [bool]$holds) {
    $mark = if ($holds) { 'ok  ' } else { $script:unexpected++; 'DIFF' }
    Write-Output "  $mark $what`: $holds"
}

$version = (Get-ItemProperty 'HKLM:\SOFTWARE\Microsoft\Windows NT\CurrentVersion')
Write-Output ("Windows: {0} {1} build {2}; {3} is {4}" -f $version.ProductName, $version.DisplayVersion,
    $version.CurrentBuild, $root, ([IO.DriveInfo]::new($root.Substring(0, 1))).DriveFormat)

# ---- Part A: another process acts on the running program's file ------------------------------
Write-Output ''
Write-Output 'Part A: another process acts on the file of a running program'
$dir = Join-Path $root 'a'
New-Item -ItemType Directory -Path $dir | Out-Null
$app = Join-Path $dir 'app.exe'
$aside = "$app.old"
Copy-Item $oldSource $app
$running = Start-Process $app -ArgumentList '/k' -PassThru -WindowStyle Hidden
Start-Sleep -Seconds 1
Check 'the program runs from app.exe' ((-not $running.HasExited) -and ($running.Path -eq $app))

Try-It 'overwrite app.exe with the new file' 'refused' { [IO.File]::Copy($newSource, $app, $true) }
Try-It 'delete app.exe' 'refused' { [IO.File]::Delete($app) }
Try-It 'rename app.exe to app.exe.old' 'allowed' { [IO.File]::Move($app, $aside) }
Try-It 'copy the new file to app.exe' 'allowed' { [IO.File]::Copy($newSource, $app) }
$said = (& $app | Out-String).Trim()
Check "app.exe now runs the new program (said '$said')" ($said -ieq $env:COMPUTERNAME)
$running.Refresh()
Check 'the old program is still running, from the renamed file' (-not $running.HasExited)
Try-It 'delete app.exe.old while the old program runs' 'refused' { [IO.File]::Delete($aside) }
Stop-Process -Id $running.Id -Force
$running.WaitForExit()
Start-Sleep -Milliseconds 500
Try-It 'delete app.exe.old after the old program has ended' 'allowed' { [IO.File]::Delete($aside) }
Check 'app.exe.old is gone' (-not (Test-Path $aside))

# ---- Part B: the running program does it to itself --------------------------------------------
Write-Output ''
Write-Output 'Part B: the running program renames its own file, copies the new one in, starts it'
$dir = Join-Path $root 'b'
New-Item -ItemType Directory -Path $dir | Out-Null
$app = Join-Path $dir 'app.exe'
$aside = "$app.old"
Copy-Item $oldSource $app
# app.exe is cmd.exe here: it renames app.exe, the file it is running from, copies the new
# program to that name, and runs it - the planner's own three steps.
$said = (& $app /d /c "ren $app app.exe.old && copy /y $newSource $app >nul && $app" | Out-String).Trim()
Check "the old program's last act started the new one (said '$said')" ($said -ieq $env:COMPUTERNAME)
Check 'app.exe is the new file' ((Test-Path $app) -and ((Hash $app) -eq (Hash $newSource)))
Check 'app.exe.old is the old file' ((Test-Path $aside) -and ((Hash $aside) -eq (Hash $oldSource)))
Try-It 'delete app.exe.old at the next start' 'allowed' { [IO.File]::Delete($aside) }

Remove-Item -Recurse -Force $root
Write-Output ''
Write-Output "self-replace-test: $unexpected line(s) other than expected"
exit $unexpected
