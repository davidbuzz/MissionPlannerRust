# The SIMULATION tab's copter picture on Windows, the steps of tests/gui/sitl-launch.gui: the
# planner started on the SIMULATION screen with a scratch data directory, the copter picture
# clicked where the planner says it is (MP_PROBE), and what it then believes (MP_FACTS) checked -
# the Cygwin SITL fetched from firmware.ardupilot.org and started (`SITL.cs:269-456, 604-738`),
# FLIGHT DATA connected to it on tcp:127.0.0.1:5760, its heartbeat, its parameters, a flight mode
# changed and changed back. PLAN.md §13.6 row 78 owes one real launch on each OS, and the GUI
# runner (tools/gui-test.sh) is Linux only (D19), so this is that script's steps in PowerShell,
# with user32 for the clicks.
#
# Run in the VM's console window, where it can be watched (it clicks in the VM's desktop):
#   tools/win10/vm-run.sh sitl-launch tools/win10/sitl-launch.ps1
# The facts file and a screenshot are left in C:\setup (win10-sitl.facts, win10-sitl.png) and
# copied to the shared folder.
param([string]$Exe = 'C:\src\MissionPlannerRust\target\debug\planner.exe')
$ErrorActionPreference = 'Stop'

Add-Type @'
using System;
using System.Runtime.InteropServices;
public static class U32 {
    [StructLayout(LayoutKind.Sequential)] public struct POINT { public int X; public int Y; }
    [DllImport("user32.dll")] public static extern bool ClientToScreen(IntPtr hwnd, ref POINT p);
    [DllImport("user32.dll")] public static extern bool SetForegroundWindow(IntPtr hwnd);
    [DllImport("user32.dll")] public static extern bool SetCursorPos(int x, int y);
    [DllImport("user32.dll")] public static extern void mouse_event(uint flags, uint dx, uint dy, uint data, UIntPtr extra);
    [DllImport("user32.dll")] public static extern void keybd_event(byte vk, byte scan, uint flags, UIntPtr extra);
    [DllImport("user32.dll")] public static extern bool SetWindowPos(IntPtr hwnd, IntPtr after, int x, int y, int cx, int cy, uint flags);
    [DllImport("user32.dll")] public static extern int GetSystemMetrics(int index);
}
'@
Add-Type -AssemblyName System.Drawing

if (-not (Test-Path $Exe)) { Write-Output "no planner at $Exe"; exit 1 }
if (netstat -an | Select-String -Pattern ':5760\s+\S+\s+LISTENING') {
    Write-Output 'port 5760 is busy: this test launches its own SITL there; stop the one running first'
    exit 3
}

$work = Join-Path $env:TEMP ('mp-sitl-' + [guid]::NewGuid().ToString('N').Substring(0, 8))
New-Item -ItemType Directory -Path $work | Out-Null
$facts = Join-Path $work 'facts'
$probe = Join-Path $work 'probe'
# Stable, as `cmb_version` index 2: the release the Cygwin folder is fetched for.
'<?xml version="1.0" encoding="utf-8"?><Config><sitl_download_version>2</sitl_download_version></Config>' |
    Set-Content -Path (Join-Path $work 'config.xml') -Encoding ASCII

# The planner writes each file to a temporary name and renames it over the last; a reader that
# does not share delete would make that rename fail.
function Read-Shared($path) {
    try {
        $stream = New-Object IO.FileStream($path, 'Open', 'Read', ([IO.FileShare]'ReadWrite, Delete'))
        $reader = New-Object IO.StreamReader($stream)
        $text = $reader.ReadToEnd()
        $reader.Close()
        return $text
    } catch { return $null }
}

function Read-Facts {
    $table = @{}
    $text = Read-Shared $facts
    if ($text) {
        foreach ($line in $text -split "`r?`n") {
            $at = $line.IndexOf(' = ')
            if ($at -gt 0) { $table[$line.Substring(0, $at)] = $line.Substring($at + 3) }
        }
    }
    return $table
}

$script:failures = 0
$script:started = Get-Date
function Stamp { '[t=+{0:N1}s]' -f ((Get-Date) - $script:started).TotalSeconds }

# `expect <fact> <value>`, `~ <substring>` or `> <number>`, waiting up to $Within seconds.
function Expect($Key, $Op, $Want, [int]$Within = 10) {
    $deadline = (Get-Date).AddSeconds($Within)
    $got = $null
    do {
        $got = (Read-Facts)[$Key]
        if ($null -ne $got) {
            $ok = switch ($Op) {
                '=' { $got -eq $Want }
                '~' { $got.Contains($Want) }
                '>' { ($got -as [double]) -gt $Want }
            }
            if ($ok) { Write-Output "$(Stamp) PASS $Key $Op $Want"; return }
        }
        Start-Sleep -Milliseconds 200
    } while ((Get-Date) -lt $deadline)
    Write-Output "$(Stamp) FAIL $Key $Op $Want - found '$got' after $Within s"
    $script:failures++
}

# `click <control>`: its centre from the probe, in the window's client area, clicked on screen.
function Click($Name) {
    $deadline = (Get-Date).AddSeconds(10)
    do {
        $line = (Read-Shared $probe) -split "`n" | Where-Object { $_.Contains("`"$Name`"") } | Select-Object -First 1
        if ($line -and $line -match '"centre_x": ([0-9.-]+), "centre_y": ([0-9.-]+)') {
            $point = New-Object U32+POINT
            $point.X = [int][double]$Matches[1]
            $point.Y = [int][double]$Matches[2]
            [void][U32]::ClientToScreen($script:hwnd, [ref]$point)
            # Above everything, the simulator's console included, which opens over the planner.
            [void][U32]::SetWindowPos($script:hwnd, [IntPtr](-1), 0, 0, 0, 0, 0x0003)
            # An Alt tap lets a background process bring a window forward.
            [U32]::keybd_event(0x12, 0, 0, [UIntPtr]::Zero)
            [U32]::keybd_event(0x12, 0, 2, [UIntPtr]::Zero)
            [void][U32]::SetForegroundWindow($script:hwnd)
            Start-Sleep -Milliseconds 150
            [void][U32]::SetCursorPos($point.X, $point.Y)
            Start-Sleep -Milliseconds 30
            [U32]::mouse_event(0x2, 0, 0, 0, [UIntPtr]::Zero)
            [U32]::mouse_event(0x4, 0, 0, 0, [UIntPtr]::Zero)
            Write-Output "$(Stamp) click $Name at screen $($point.X),$($point.Y)"
            Start-Sleep -Milliseconds 100
            return
        }
        Start-Sleep -Milliseconds 200
    } while ((Get-Date) -lt $deadline)
    Write-Output "$(Stamp) FAIL click $Name - not in the probe"
    $script:failures++
}

function Shot {
    # SM_CXSCREEN and SM_CYSCREEN, asked now: Screen.Bounds keeps the size it first saw.
    $size = New-Object System.Drawing.Size([U32]::GetSystemMetrics(0), [U32]::GetSystemMetrics(1))
    $bitmap = New-Object System.Drawing.Bitmap $size.Width, $size.Height
    $graphics = [System.Drawing.Graphics]::FromImage($bitmap)
    $graphics.CopyFromScreen([System.Drawing.Point]::Empty, [System.Drawing.Point]::Empty, $size)
    $bitmap.Save('C:\setup\win10-sitl.png', [System.Drawing.Imaging.ImageFormat]::Png)
    $graphics.Dispose(); $bitmap.Dispose()
}

$env:MP_FACTS = $facts
$env:MP_PROBE = $probe
$env:MP_SCREEN = 'simulation'
# The VM's screen is 1600 x 1200 (VBoxManage setvideomodehint, 2026-09-26): the SIMULATION page's
# pictures sit lower than a 700-pixel window reaches, and a click there lands on the taskbar.
$env:MP_WINDOW = '1560x1080'
$env:MP_CONFIG_XML = Join-Path $work 'config.xml'
$env:MP_NO_TILES = '1'
$env:LOCALAPPDATA = $work
Write-Output "$(Stamp) starting $Exe with its data directory in $work"
# In this window's console rather than one of its own, which would open over the planner.
$planner = Start-Process -FilePath $Exe -PassThru -NoNewWindow -RedirectStandardError (Join-Path $work 'stderr.log') -RedirectStandardOutput (Join-Path $work 'stdout.log')
try {
    $deadline = (Get-Date).AddSeconds(60)
    do { Start-Sleep -Milliseconds 250; $planner.Refresh() } while ($planner.MainWindowHandle -eq [IntPtr]::Zero -and -not $planner.HasExited -and (Get-Date) -lt $deadline)
    if ($planner.MainWindowHandle -eq [IntPtr]::Zero) { throw 'the planner showed no window' }
    $script:hwnd = $planner.MainWindowHandle
    # At the screen's top left, whole: SWP_NOSIZE | SWP_NOZORDER.
    [void][U32]::SetWindowPos($script:hwnd, [IntPtr]::Zero, 0, 0, 0, 0, 0x0005)
    Write-Output "$(Stamp) window up"

    Expect 'screen' '=' 'simulation'
    Expect 'sitl.launcher' '=' 'cygwin'
    Expect 'sitl.version' '=' 'Stable'
    Expect 'sitl.running' '=' 'false'
    # The copter picture: the Cygwin folder fetched and the simulator started, on a thread.
    Click 'sitl-picture-quad'
    Expect 'sitl.running' '=' 'true'
    Expect 'sitl.outcome' '=' 'connect tcp:127.0.0.1:5760' -Within 180
    Expect 'sitl.arguments' '~' '--serial0 tcp:0'
    # `MainV2.View.ShowScreen(screens[0])` and `doConnect(comPort, "preset", "5760")`.
    Expect 'screen' '=' 'fly'
    Expect 'vehicle.connected' '=' 'true' -Within 20
    Expect 'vehicle.count' '>' 0 -Within 30
    Expect 'vehicle.mode' '=' 'Stabilize' -Within 15
    Expect 'params.held' '>' 1000 -Within 60
    Click 'fly-tab-actions'
    Expect 'fly.tab' '=' 'tabActions'
    Click 'AltHold'
    Expect 'vehicle.mode' '=' 'AltHold' -Within 15
    Click 'Stabilize'
    Expect 'vehicle.mode' '=' 'Stabilize' -Within 15
    Shot
} catch {
    Write-Output "$(Stamp) FAIL $($_.Exception.Message)"
    $script:failures++
} finally {
    Copy-Item -Path $facts -Destination C:\setup\win10-sitl.facts -ErrorAction SilentlyContinue
    if (-not $planner.HasExited) {
        [void]$planner.CloseMainWindow()
        if (-not $planner.WaitForExit(5000)) { Stop-Process -Id $planner.Id -Force }
    }
    # The simulator started from the scratch directory, if it outlived the planner.
    Get-Process | Where-Object { $_.Path -and $_.Path.StartsWith($work) } | ForEach-Object {
        Write-Output "$(Stamp) stopping $($_.ProcessName) ($($_.Id)), left running"
        Stop-Process -Id $_.Id -Force
    }
    foreach ($file in 'win10-sitl.facts', 'win10-sitl.png') {
        Copy-Item -Path (Join-Path C:\setup $file) -Destination \\VBOXSVR\vmshare\ -ErrorAction SilentlyContinue
    }
    Get-Content -Path (Join-Path $work 'stderr.log') -Tail 5 -ErrorAction SilentlyContinue | ForEach-Object { "stderr: $_" }
}
Write-Output "$(Stamp) $(if ($script:failures) { "$($script:failures) failed" } else { 'all passed' })"
exit [int]($script:failures -gt 0)
