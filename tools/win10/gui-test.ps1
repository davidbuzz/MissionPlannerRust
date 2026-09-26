# The GUI runner for Windows: tools/gui-test.sh's script language, run against planner.exe with
# user32 in place of xdotool. It reads the same tests/gui/*.gui files and says the same things -
# a line stamped with its time per step, "  ok   ..." or "FAIL line N: ..." per check, and
# "<script>: passed in N s" or "<script>: N failure(s)" at the end - so a Windows run can be put
# beside a Linux one line for line (DELIVERABLES D19: the runner was Linux only).
#
#   powershell -File tools\win10\gui-test.ps1 -Script tests\gui\x.gui [-Link tcp:127.0.0.1:5760]
#
# Run in the VM's console window through tools/win10/gui-suite.ps1, which gives each script the
# link its header names. Where Windows differs from the Linux runner:
#
# * `setup` lines are bash; they run in Git for Windows' bash, from the repository, with $WORK a
#   forward-slash Windows path both bash and the application read. One whose command is not there
#   (python3, cargo) exits 127, and the script is skipped (exit 3) as the Linux runner skips one
#   whose setup exits 3.
# * The data directory is `Documents\MissionPlannerRust` on Windows (mp_settings, D11), so
#   `env XDG_DATA_HOME dir` sets USERPROFILE to `dir\.home`, whose `Documents` is a junction to
#   `dir`: the application then finds `dir\MissionPlannerRust`, where the script's setup put its
#   config.xml. LOCALAPPDATA and ProgramData (the data directory proper, as the C#'s) go to `dir`
#   too, and `env TMPDIR dir` sets TEMP and TMP.
# * A script that gives no data directory and no MP_CONFIG_XML gets an empty config.xml of its
#   own, not a copy of the VM's: the VM's is the owner's, and one saved with a link to the
#   laptop's SITL connected a test to it (2026-09-26).
# * Keys are xdotool's names, sent with keybd_event; text is typed as Unicode with SendInput.
# * The window is put at the screen's top left and kept topmost, so nothing opened over it - a
#   simulator's console - takes a click; the pointer is the VM's own.
# * `restart` posts WM_CLOSE, as the close box does; `expect a below b` and its kin read the probe
#   with ConvertFrom-Json.
# * A run over its budget says so on a line of its own, "budget only" when every check held, as
#   a debug build under a VM is slower than the Linux machine the budgets were measured on.
param(
    [Parameter(Mandatory = $true)][string]$Script,
    [string]$Link = '',
    [string]$Exe = 'C:\src\MissionPlannerRust\target\debug\planner.exe',
    [string]$Root = 'C:\src\MissionPlannerRust',
    [int]$ExpectWaitMs = 10000,
    [double]$HardStopMargin = 3,
    # The Windows budgets, one line per script, "name seconds": its time in a passing run here,
    # which gui-suite.ps1 records after every run - tools\win10\budgets.txt by default.
    [string]$Budgets = '',
    # A script with no Windows time yet has its own budget, the Linux one, doubled (the owner,
    # 2026-09-26: "double the timeout budget on windows"): the budgets were measured on the Linux
    # machine, and here a debug build runs in a VM that shares its cores. Over-budget and the hard
    # stop both count from the doubled one.
    [double]$BudgetScale = 2,
    # Budgets and the hard stop stretched by this much: the budgets were measured on the Linux
    # machine, and a debug build in a VM sharing that machine's cores runs slower. The time is
    # still reported, and a run over the script's own budget still says so.
    [double]$TimeScale = 1,
    # Leaves the scratch directory - the application's logs, config and data - for a look.
    [switch]$Keep,
    # The SITL for a link to tcp:127.0.0.1:5760, from gui-suite.ps1: started in this directory
    # whenever nothing listens there - before the setups, before the application starts and
    # before a `restart` - and given time to settle (Ensure-Sitl). The Cygwin build exits when
    # its client leaves - Mission Planner never reconnects - where the Linux suite's listens
    # again; restarting it in one directory keeps its parameters, as the Linux one keeps them
    # across the suite.
    [string]$SitlExe = '',
    [string]$SitlDir = '',
    [string]$SitlParams = ''
)
$ErrorActionPreference = 'Stop'
# PATH as the registry has it now, not as the console window inherited it at logon: a program
# installed since - Python for the setups' python3, 2026-09-26 - is on it for the setups at once.
$env:Path = @([Environment]::GetEnvironmentVariable('Path', 'Machine'),
    [Environment]::GetEnvironmentVariable('Path', 'User')) -join ';'
# What lives under the real profile stays found when a script gives the application a profile of
# its own (`env XDG_DATA_HOME`, below): cargo and rustup, which plugins.gui's setup builds its wasm
# plugins with ("can't find crate for `core`" under the script's empty .rustup), and GStreamer's
# plugin registry, one for every script - under the script's LOCALAPPDATA it was built afresh each
# run, which on Windows outlasts fly-gstreamer's wait for frames (2026-09-26).
if (-not $env:CARGO_HOME) { $env:CARGO_HOME = Join-Path $env:USERPROFILE '.cargo' }
if (-not $env:RUSTUP_HOME) { $env:RUSTUP_HOME = Join-Path $env:USERPROFILE '.rustup' }
if (-not $env:GST_REGISTRY_1_0) { $env:GST_REGISTRY_1_0 = 'C:\setup\gstreamer-registry.x86_64.bin' }
# And built before any script waits for frames: GStreamer's first run scans every plugin, longer
# than a script waits in this VM, and a launcher stopped mid-scan writes no registry - so each
# video script began the scan again and none got its frames (2026-09-26). Once, then kept.
if (-not (Test-Path $env:GST_REGISTRY_1_0)) {
    $inspect = @('C:\Program Files\gstreamer\1.0\msvc_x86_64\bin\gst-inspect-1.0.exe',
        'C:\gstreamer\1.0\msvc_x86_64\bin\gst-inspect-1.0.exe') | Where-Object { Test-Path $_ } | Select-Object -First 1
    if ($inspect) { & $inspect coreelements 2>&1 | Out-Null }
}
$Bash = 'C:\Program Files\Git\bin\bash.exe'

Add-Type @'
using System;
using System.Runtime.InteropServices;
public static class GuiInput {
    [StructLayout(LayoutKind.Sequential)] public struct POINT { public int X; public int Y; }
    [DllImport("user32.dll")] public static extern bool ClientToScreen(IntPtr hwnd, ref POINT p);
    [DllImport("user32.dll")] public static extern bool SetForegroundWindow(IntPtr hwnd);
    [DllImport("user32.dll")] public static extern bool SetCursorPos(int x, int y);
    [DllImport("user32.dll")] public static extern void mouse_event(uint flags, uint dx, uint dy, int data, UIntPtr extra);
    [DllImport("user32.dll")] public static extern void keybd_event(byte vk, byte scan, uint flags, UIntPtr extra);
    [DllImport("user32.dll")] public static extern bool SetWindowPos(IntPtr hwnd, IntPtr after, int x, int y, int cx, int cy, uint flags);
    [DllImport("user32.dll")] public static extern bool PostMessage(IntPtr hwnd, uint msg, IntPtr w, IntPtr l);
    [DllImport("user32.dll")] public static extern bool IsWindow(IntPtr hwnd);
    [DllImport("user32.dll")] public static extern int GetSystemMetrics(int index);
    [DllImport("user32.dll")] public static extern IntPtr GetForegroundWindow();

    [StructLayout(LayoutKind.Sequential)] struct KEYBDINPUT { public ushort wVk; public ushort wScan; public uint dwFlags; public uint time; public IntPtr dwExtraInfo; }
    // sizeof(INPUT) is 40 on x64 - the union's largest member, MOUSEINPUT, is 32 - and SendInput
    // refuses a smaller size.
    [StructLayout(LayoutKind.Explicit, Size = 40)] struct INPUT { [FieldOffset(0)] public uint type; [FieldOffset(8)] public KEYBDINPUT ki; }
    [DllImport("user32.dll")] static extern uint SendInput(uint n, INPUT[] inputs, int size);

    [DllImport("user32.dll")] static extern short VkKeyScanW(char c);

    /// One character typed as the keyboard types it: its key, with Shift, Ctrl or Alt as the
    /// layout needs (VkKeyScan). The application reads text from the WM_CHAR a real key makes;
    /// an injected Unicode character (VK_PACKET) never reached its text boxes (2026-09-26).
    /// Unicode only for a character no key on the layout makes.
    public static void TypeChar(char c) {
        short scan = VkKeyScanW(c);
        if (scan == -1) { TypeUnicode(c); return; }
        byte vk = (byte)(scan & 0xFF);
        int shifts = (scan >> 8) & 0xFF;
        if ((shifts & 1) != 0) keybd_event(0x10, 0, 0, UIntPtr.Zero);
        if ((shifts & 2) != 0) keybd_event(0x11, 0, 0, UIntPtr.Zero);
        if ((shifts & 4) != 0) keybd_event(0x12, 0, 0, UIntPtr.Zero);
        keybd_event(vk, 0, 0, UIntPtr.Zero);
        keybd_event(vk, 0, 2, UIntPtr.Zero);
        if ((shifts & 4) != 0) keybd_event(0x12, 0, 2, UIntPtr.Zero);
        if ((shifts & 2) != 0) keybd_event(0x11, 0, 2, UIntPtr.Zero);
        if ((shifts & 1) != 0) keybd_event(0x10, 0, 2, UIntPtr.Zero);
    }

    /// One character typed as Unicode, down and up.
    public static void TypeUnicode(char c) {
        INPUT[] inputs = new INPUT[2];
        inputs[0].type = 1; inputs[0].ki.wScan = c; inputs[0].ki.dwFlags = 0x0004;
        inputs[1].type = 1; inputs[1].ki.wScan = c; inputs[1].ki.dwFlags = 0x0004 | 0x0002;
        SendInput(2, inputs, Marshal.SizeOf(typeof(INPUT)));
    }
}
'@
Add-Type -AssemblyName System.Drawing

$ScriptPath = (Resolve-Path $Script).Path
$ScriptName = [IO.Path]::GetFileName($ScriptPath)
# Named as tools/gui-test.sh names its scratch directory (`mktemp -t planner-work-XXXXXX`): a script
# may check that a path the application shows is in it (fly-poi.gui).
$Work = Join-Path $env:TEMP ('planner-work-' + [guid]::NewGuid().ToString('N').Substring(0, 8))
New-Item -ItemType Directory -Path $Work | Out-Null
$WorkFwd = $Work.Replace('\', '/')
$FactsFile = Join-Path $Work '.facts'
$ProbeFile = Join-Path $Work '.probe.json'
$script:Failures = 0
$script:BudgetOver = $false
$script:App = $null
$script:Hwnd = [IntPtr]::Zero
$script:NextWaitMs = $null
$script:LastValue = ''
$Budget = 5.0

function Now-Ms { [long]([DateTime]::UtcNow - [DateTime]'1970-01-01').TotalMilliseconds }
function Secs([long]$ms) { '{0:N3}' -f ($ms / 1000.0) }
# Said with Write-Host: in PowerShell a function's Write-Output is its return value, and a check
# whose verdict was its result would be swallowed with it (the first Windows run lost its FAILs).
function Say([string]$text) { Write-Host $text }
function Fail([string]$text) { Say "FAIL $text"; $script:Failures++ }

# ---- reading what the application publishes ---------------------------------------------------

# Its files are written to a temporary name and renamed over the last: a reader that did not
# share delete would make that rename fail.
function Read-Shared([string]$path) {
    try {
        $stream = New-Object IO.FileStream($path, 'Open', 'Read', ([IO.FileShare]'ReadWrite, Delete'))
        $reader = New-Object IO.StreamReader($stream)
        $text = $reader.ReadToEnd()
        $reader.Close()
        return $text
    } catch { return $null }
}
function Stamp([string]$path) {
    try { return (Get-Item -LiteralPath $path -ErrorAction Stop).LastWriteTimeUtc.Ticks } catch { return 0 }
}
function Read-Facts {
    $table = @{}
    $text = Read-Shared $FactsFile
    if ($text) {
        foreach ($line in $text -split "`r?`n") {
            $at = $line.IndexOf(' = ')
            if ($at -gt 0) { $table[$line.Substring(0, $at)] = $line.Substring($at + 3) }
        }
    }
    return $table
}
# The probe's text; one control per line, as probe.rs writes it.
function Read-Probe { return Read-Shared $ProbeFile }
# One control's rectangle from the probe's text, found by its line: quicker than parsing the whole
# file for each poll, which on a page of a thousand controls took longer than the poll's gap.
function Probe-Entry([string]$probe, [string]$name) {
    if (-not $probe) { return $null }
    $pattern = '"' + [regex]::Escape($name) + '": \{ "x": ([-0-9.]+), "y": ([-0-9.]+), "width": ([-0-9.]+), "height": ([-0-9.]+), "centre_x": ([-0-9.]+), "centre_y": ([-0-9.]+) \}'
    $m = [regex]::Match($probe, $pattern)
    if (-not $m.Success) { return $null }
    $v = 1..6 | ForEach-Object { [double]$m.Groups[$_].Value }
    return [pscustomobject]@{ x = $v[0]; y = $v[1]; width = $v[2]; height = $v[3]; centre_x = $v[4]; centre_y = $v[5] }
}

# Waits for the facts file to be written `count` times: a frame drawn after the last input.
function Wait-Publishes([int]$count) {
    $last = Stamp $FactsFile
    $seen = 0
    for ($i = 0; $i -lt 40; $i++) {
        Start-Sleep -Milliseconds 50
        $now = Stamp $FactsFile
        if ($now -ne $last) { $last = $now; $seen++; if ($seen -ge $count) { return } }
    }
}

# A control's point in the window: its centre, or `name@fxXfy` a fraction across and down it.
# Waits up to ten seconds for the control, then for its rectangle to read the same three times
# running, 60 ms apart - a tab strip moves as a vehicle connects. gui-click.sh waits for the whole probe to be still for 300 ms; on Windows some control
# moves every frame and that wait always ran its full 1.2 s.
function Resolve-Control([string]$target) {
    $name, $fraction = $target -split '@', 2
    $deadline = (Get-Date).AddMilliseconds(10000)
    $entry = $null
    do {
        $entry = Probe-Entry (Read-Probe) $name
        if ($entry) { break }
        Start-Sleep -Milliseconds 50
    } while ((Get-Date) -lt $deadline)
    if (-not $entry) { return $null }
    $same = 0
    for ($i = 0; $i -lt 25; $i++) {
        Start-Sleep -Milliseconds 60
        $again = Probe-Entry (Read-Probe) $name
        if (-not $again) { return $null }
        $still = $again.x -eq $entry.x -and $again.y -eq $entry.y -and $again.width -eq $entry.width -and $again.height -eq $entry.height
        $entry = $again
        if ($still) { $same++; if ($same -ge 3) { break } } else { $same = 0 }
    }
    if ($fraction) {
        $fx, $fy = $fraction -split '[x,]', 2
        return @([int]($entry.x + $entry.width * [double]$fx), [int]($entry.y + $entry.height * [double]$fy))
    }
    return @([int]$entry.centre_x, [int]$entry.centre_y)
}

# ---- driving the window -----------------------------------------------------------------------

function Raise-Window {
    # Topmost (HWND_TOPMOST, SWP_NOMOVE | SWP_NOSIZE), and forward when it is not: an Alt tap
    # lets a process that is not in the foreground bring a window there. Only then - a bare Alt
    # pressed and released over the window puts it in menu mode, which swallowed the keys typed
    # next (fly-flytohere-alt.gui, 2026-09-26).
    [void][GuiInput]::SetWindowPos($script:Hwnd, [IntPtr](-1), 0, 0, 0, 0, 0x0003)
    if ([GuiInput]::GetForegroundWindow() -ne $script:Hwnd) {
        [GuiInput]::keybd_event(0x12, 0, 0, [UIntPtr]::Zero)
        [GuiInput]::keybd_event(0x12, 0, 2, [UIntPtr]::Zero)
        [void][GuiInput]::SetForegroundWindow($script:Hwnd)
    }
}
function Move-Pointer([int[]]$at) {
    $point = New-Object GuiInput+POINT
    $point.X = $at[0]; $point.Y = $at[1]
    [void][GuiInput]::ClientToScreen($script:Hwnd, [ref]$point)
    [void][GuiInput]::SetCursorPos($point.X, $point.Y)
    return "$($at[0]),$($at[1])"
}
function Press([int]$button) {
    $flags = switch ($button) { 1 { @(0x2, 0x4) } 2 { @(0x20, 0x40) } 3 { @(0x8, 0x10) } }
    [GuiInput]::mouse_event($flags[0], 0, 0, 0, [UIntPtr]::Zero)
    [GuiInput]::mouse_event($flags[1], 0, 0, 0, [UIntPtr]::Zero)
}
function Wheel([string]$direction) {
    $delta = if ($direction -eq 'up') { 120 } else { -120 }
    [GuiInput]::mouse_event(0x0800, 0, 0, $delta, [UIntPtr]::Zero)
}

# xdotool's key names, as the scripts use them.
$Keys = @{
    'return' = 0x0D; 'escape' = 0x1B; 'tab' = 0x09; 'backspace' = 0x08; 'delete' = 0x2E
    'home' = 0x24; 'end' = 0x23; 'left' = 0x25; 'up' = 0x26; 'right' = 0x27; 'down' = 0x28
    'page_up' = 0x21; 'prior' = 0x21; 'page_down' = 0x22; 'next' = 0x22; 'space' = 0x20
    'minus' = 0xBD; 'plus' = 0xBB; 'equal' = 0xBB; 'period' = 0xBE; 'comma' = 0xBC
    'insert' = 0x2D
}
$Modifiers = @{ 'ctrl' = 0x11; 'control' = 0x11; 'shift' = 0x10; 'alt' = 0x12; 'super' = 0x5B }
# Keys that need KEYEVENTF_EXTENDEDKEY.
$Extended = @(0x2E, 0x24, 0x23, 0x25, 0x26, 0x27, 0x28, 0x21, 0x22, 0x2D)
function Key-Code([string]$name) {
    $lower = $name.ToLower()
    if ($Keys.ContainsKey($lower)) { return $Keys[$lower] }
    if ($lower -match '^f([0-9]{1,2})$') { return 0x6F + [int]$Matches[1] }
    if ($name.Length -eq 1) {
        $c = [char]$name.ToUpper()
        if (($c -ge 'A' -and $c -le 'Z') -or ($c -ge '0' -and $c -le '9')) { return [int]$c }
    }
    return $null
}
function Send-Key([string]$chord) {
    $parts = $chord -split '\+'
    $mods = @()
    foreach ($m in $parts[0..($parts.Count - 2)]) {
        if ($parts.Count -gt 1) {
            $code = $Modifiers[$m.ToLower()]
            if ($null -eq $code) { return $false }
            $mods += $code
        }
    }
    $code = Key-Code $parts[-1]
    if ($null -eq $code) { return $false }
    foreach ($m in $mods) { [GuiInput]::keybd_event([byte]$m, 0, 0, [UIntPtr]::Zero) }
    $ext = if ($Extended -contains $code) { 1 } else { 0 }
    [GuiInput]::keybd_event([byte]$code, 0, $ext, [UIntPtr]::Zero)
    [GuiInput]::keybd_event([byte]$code, 0, $ext -bor 2, [UIntPtr]::Zero)
    [array]::Reverse($mods)
    foreach ($m in $mods) { [GuiInput]::keybd_event([byte]$m, 0, 2, [UIntPtr]::Zero) }
    return $true
}

function Shot([string]$name) {
    try {
        $size = New-Object System.Drawing.Size([GuiInput]::GetSystemMetrics(0), [GuiInput]::GetSystemMetrics(1))
        $bitmap = New-Object System.Drawing.Bitmap $size.Width, $size.Height
        $graphics = [System.Drawing.Graphics]::FromImage($bitmap)
        $graphics.CopyFromScreen([System.Drawing.Point]::Empty, [System.Drawing.Point]::Empty, $size)
        $out = Join-Path 'C:\setup\shots' $name
        New-Item -ItemType Directory -Force -Path (Split-Path $out) | Out-Null
        $bitmap.Save($out, [System.Drawing.Imaging.ImageFormat]::Png)
        $graphics.Dispose(); $bitmap.Dispose()
        Say "screenshot: $out"
    } catch {}
}

# ---- the script's directives -----------------------------------------------------------------

$lines = Get-Content -LiteralPath $ScriptPath
$envs = New-Object System.Collections.ArrayList
$setups = New-Object System.Collections.ArrayList
# A line may be for one platform: `windows: expect ...` runs here without its prefix, and
# `linux:` or `macos:` lines are for the other runners - where the C# itself differs by platform
# (the SITL page's launcher, the line ends a file is written with). $null for a line that is not
# this platform's.
function For-ThisPlatform([string]$text) {
    if ($text -match '^(linux|windows|macos):\s*(.*)$') {
        if ($Matches[1] -ne 'windows') { return $null }
        return $Matches[2]
    }
    return $text
}

$tiles = 'off'
$screen = $null
$window = $null
$n = 0
foreach ($raw in $lines) {
    $n++
    if ($raw -match '^\s*#') { continue }
    $line = ($raw -split '\s#', 2)[0].Trim()
    if (-not $line) { continue }
    $line = For-ThisPlatform $line
    if ($null -eq $line) { continue }
    $words = $line -split '\s+'
    switch ($words[0]) {
        'screen' { $screen = if ($words.Count -gt 1) { $words[1] } else { 'fly' } }
        'window' { $window = if ($words.Count -gt 1) { $words[1] } else { '1600x1200' } }
        'tiles' { $tiles = if ($words.Count -gt 1) { $words[1] } else { 'off' } }
        'env' {
            $value = ($line -replace '^env\s+\S+\s*', '').Replace('$WORK', $WorkFwd)
            [void]$envs.Add(@($words[1], $value))
        }
        'setup' { [void]$setups.Add(@($n, ($line -replace '^setup\s+', '').Replace('$WORK', $WorkFwd))) }
        'budget' { $Budget = [double]$words[1] }
    }
}
$ScriptBudget = $Budget
# The script's budget on Windows, once a passing run here has measured one (the owner,
# 2026-09-26: "made so its successful test run time + 3 sec", as on Linux, where
# tools/gui-budgets.py record keeps the scripts' own budgets); until then its own doubled.
if (-not $Budgets) { $Budgets = Join-Path $Root 'tools\win10\budgets.txt' }
$BaseName = [IO.Path]::GetFileNameWithoutExtension($ScriptName)
$Measured = $null
if (Test-Path $Budgets) {
    foreach ($entry in Get-Content $Budgets) {
        if ($entry -match '^(\S+)\s+([0-9.]+)\s*$' -and $Matches[1] -eq $BaseName) {
            $Measured = [double]::Parse($Matches[2], [Globalization.CultureInfo]::InvariantCulture)
        }
    }
}
if ($null -ne $Measured) {
    $Budget = $Measured
    $BudgetFrom = "measured on Windows; the script's own is $ScriptBudget s"
} else {
    $Budget = $Budget * $BudgetScale
    $BudgetFrom = "$BudgetScale x the script's $ScriptBudget s, not yet timed on Windows"
}

# The window: what the script asks for, no larger than the screen's work area allows.
$areaW = [GuiInput]::GetSystemMetrics(16); $areaH = [GuiInput]::GetSystemMetrics(17)
if ($window) {
    $w, $h = $window -split 'x'
    $w = [Math]::Min([int]$w, $areaW - 16); $h = [Math]::Min([int]$h, $areaH - 40)
    $window = "${w}x$h"
}

$env:MP_NO_RECORD = '1'
switch ($tiles) {
    'on' { Remove-Item Env:MP_NO_TILES -ErrorAction SilentlyContinue; Remove-Item Env:MP_OFFLINE -ErrorAction SilentlyContinue }
    'offline' { Remove-Item Env:MP_NO_TILES -ErrorAction SilentlyContinue; $env:MP_OFFLINE = '1' }
    default { $env:MP_NO_TILES = '1' }
}
$env:MP_PROBE = $ProbeFile
$env:MP_FACTS = $FactsFile
$env:MP_SETTINGS = Join-Path $Work '.settings.conf'
if ($screen) { $env:MP_SCREEN = $screen }
if ($window) { $env:MP_WINDOW = $window }
$Junctions = New-Object System.Collections.ArrayList
foreach ($pair in $envs) {
    Set-Item -Path "Env:$($pair[0])" -Value $pair[1]
    if ($pair[0] -eq 'XDG_DATA_HOME') {
        $data = $pair[1].Replace('/', '\')
        New-Item -ItemType Directory -Force -Path (Join-Path $data '.home') | Out-Null
        $documents = Join-Path $data '.home\Documents'
        New-Item -ItemType Junction -Path $documents -Target $data | Out-Null
        [void]$Junctions.Add($documents)
        $env:USERPROFILE = Join-Path $data '.home'
        $env:LOCALAPPDATA = $data
        # And the data directory, which on Windows is CommonApplicationData's, as the C#'s
        # (Settings.GetDataDirectory): left at C:\ProgramData, fly-camera read the terrain tiles a
        # session by hand had fetched there, where the script's own directory has none.
        $env:ProgramData = $data
    }
    if ($pair[0] -eq 'TMPDIR') { $env:TEMP = $pair[1].Replace('/', '\'); $env:TMP = $env:TEMP }
}
# The SITL, before the setups - a setup may talk to its second port, 5762 - and again before the
# application starts. In the suite it is gui-suite.ps1's one SITL behind tools\win10\sitl-relay.ps1,
# which keeps it running across scripts ("relay.status" in its directory); run alone, this starts
# one whenever nothing listens, as Mission Planner starts it, `--serial0 tcp:0` (`SITL.cs:664`), so
# it boots at once rather than waiting for its first client. Either way a SITL younger than
# $SitlSettle seconds is given the rest to find its GPS, set its home and pass its prearm checks
# before a script uses it: the Linux suite's has been up for minutes by then, and scripts run
# against one seconds old found no home (`plan-home-marker-vehicle` kept its planned Brisbane
# home), no position ("Bad Lat/Long") and "Not Ready to Arm" (the owner's report, 2026-09-26).
$SitlSettle = 30
function Ensure-Sitl {
    $script:SitlWaitedMs = 0
    if (-not $SitlExe) { return }
    $status = Join-Path $SitlDir 'relay.status'
    $started = $null
    if (Test-Path $status) {
        # The relay's SITL; between its exit and its restart the status names a process gone.
        for ($i = 0; $i -lt 40 -and -not $started; $i++) {
            $fields = @((Get-Content $status -ErrorAction SilentlyContinue) -split ' ')
            if ($fields.Count -eq 2 -and (Get-Process -Id $fields[0] -ErrorAction SilentlyContinue)) {
                $started = [DateTime]::Parse($fields[1], [Globalization.CultureInfo]::InvariantCulture,
                    [Globalization.DateTimeStyles]::RoundtripKind).ToLocalTime()
            } else {
                Start-Sleep -Milliseconds 250
            }
        }
    } else {
        for ($attempt = 0; $attempt -lt 2; $attempt++) {
            if (netstat -an | Select-String -Pattern ':5760\s+\S+\s+LISTENING') { break }
            if ($attempt -eq 0) {
                New-Item -ItemType Directory -Force -Path $SitlDir | Out-Null
                Start-Process -FilePath $SitlExe -WorkingDirectory $SitlDir -WindowStyle Minimized `
                    -ArgumentList @('--model', 'quad', '--speedup', '1', '--defaults', "`"$SitlParams`"", '--serial0', 'tcp:0') | Out-Null
                Say "SITL started in $SitlDir"
            }
            for ($i = 0; $i -lt 60; $i++) {
                if (netstat -an | Select-String -Pattern ':5760\s+\S+\s+LISTENING') { break }
                Start-Sleep -Milliseconds 250
            }
        }
        $sitl = Get-Process ArduCopter -ErrorAction SilentlyContinue | Where-Object { $_.Path -eq $SitlExe } |
            Sort-Object StartTime | Select-Object -First 1
        if ($sitl) { $started = $sitl.StartTime }
    }
    if (-not $started) { Say 'SITL did not come up'; return }
    $wait = ($started.AddSeconds($SitlSettle) - (Get-Date)).TotalSeconds
    if ($wait -gt 0) {
        Say ("SITL settling for {0:N0} s" -f $wait)
        $script:SitlWaitedMs = [int]($wait * 1000)
        Start-Sleep -Milliseconds $script:SitlWaitedMs
    }
}
Ensure-Sitl

$setupNo = 0
foreach ($setup in $setups) {
    $env:WORK = $WorkFwd
    # From a file, not `bash -c`: Windows PowerShell 5.1 drops the double quotes inside an
    # argument it hands a native program, and a setup's printf of an XML declaration lost its
    # quotes that way (2026-09-26).
    $setupNo++
    $file = Join-Path $Work ".setup-$setupNo.sh"
    [IO.File]::WriteAllText($file, $setup[1] + "`n", (New-Object Text.UTF8Encoding($false)))
    Push-Location $Root
    # A native program's stderr under 'Stop' is a terminating error in Windows PowerShell 5.1:
    # a setup that said "python3: No such file" ended the run instead of skipping it.
    $ErrorActionPreference = 'Continue'
    & $Bash $file.Replace('\', '/') 2>&1 | ForEach-Object { "setup: $_" }
    $status = $LASTEXITCODE
    $ErrorActionPreference = 'Stop'
    Pop-Location
    if ($status -eq 127 -or $status -eq 3) {
        Write-Output "SKIP line $($setup[0]): setup exited $status, the test cannot run here: $($setup[1])"
        exit 3
    }
    if ($status -ne 0) {
        Write-Output "line $($setup[0]): setup exited $status, not running the test: $($setup[1])"
        exit 2
    }
}
# A config.xml of its own, empty, when the script did not give it a data directory or a file.
if (-not $env:MP_CONFIG_XML) {
    $data = Join-Path $env:USERPROFILE 'Documents\MissionPlannerRust'
    if (-not $data.StartsWith($Work)) { $env:MP_CONFIG_XML = Join-Path $Work 'config.xml' }
}

# ---- the application ----------------------------------------------------------------------------

function Start-App {
    Ensure-Sitl
    $arguments = @{ FilePath = $Exe; PassThru = $true; NoNewWindow = $true; WorkingDirectory = $Root
        RedirectStandardOutput = (Join-Path $Work 'stdout.log'); RedirectStandardError = (Join-Path $Work 'stderr.log') }
    if ($Link) { $arguments.ArgumentList = $Link }
    $script:App = Start-Process @arguments
    $deadline = (Get-Date).AddSeconds(60)
    do {
        Start-Sleep -Milliseconds 200
        $script:App.Refresh()
        if ($script:App.HasExited) { Write-Output 'app exited before showing a window'; exit 1 }
    } while ($script:App.MainWindowHandle -eq [IntPtr]::Zero -and (Get-Date) -lt $deadline)
    if ($script:App.MainWindowHandle -eq [IntPtr]::Zero) { Write-Output "no window owned by pid $($script:App.Id) appeared"; exit 1 }
    $script:Hwnd = $script:App.MainWindowHandle
    # At the top left of the screen (SWP_NOSIZE | SWP_NOZORDER), then on top.
    [void][GuiInput]::SetWindowPos($script:Hwnd, [IntPtr]::Zero, 0, 0, 0, 0, 0x0005)
    Raise-Window
    # Its first frames: the facts written twice, then the probe still for 300 ms.
    for ($i = 0; $i -lt 60 -and -not (Test-Path $FactsFile); $i++) { Start-Sleep -Milliseconds 50 }
    $first = Stamp $FactsFile
    for ($i = 0; $i -lt 60; $i++) { Start-Sleep -Milliseconds 50; if ((Stamp $FactsFile) -ne $first) { break } }
    $stamp = 0; $same = 0
    for ($i = 0; $i -lt 30; $i++) {
        $now = Stamp $ProbeFile
        if ($stamp -ne 0 -and $now -eq $stamp) { $same++; if ($same -ge 3) { break } } else { $same = 0 }
        $stamp = $now
        Start-Sleep -Milliseconds 100
    }
}
function Stop-App {
    if ($script:App -and -not $script:App.HasExited) {
        [void][GuiInput]::PostMessage($script:Hwnd, 0x0010, [IntPtr]::Zero, [IntPtr]::Zero)
        if (-not $script:App.WaitForExit(5000)) { Stop-Process -Id $script:App.Id -Force -ErrorAction SilentlyContinue }
    }
}

# ---- checks ------------------------------------------------------------------------------------

function Is-Number([string]$s) { return $s -match '^-?[0-9]+(\.[0-9]+)?$' }
# One check of an expect line: $true when it holds; the verdict said only when $say.
function Expect-Once([string[]]$w, [bool]$say, [int]$lineNo) {
    $key = $w[1]; $op = $w[2]
    $facts = Read-Facts
    $known = $facts.ContainsKey($key)
    $got = if ($known) { $facts[$key] } else { '' }
    $script:LastValue = $got
    if ($op -eq '~') {
        $want = ($w[3..($w.Count - 1)] -join ' ')
        # A path is written with / in the scripts, and Windows gives \ where the application joins
        # one on (Path.Combine, as the C# joins it): a separator matches either.
        if ($got.Replace('\', '/').Contains($want.Replace('\', '/'))) { if ($say) { Say "  ok   $key contains '$want'" }; return $true }
        if ($say) { Fail "line ${lineNo}: $key is '$got', expected to contain '$want'" }
        return $false
    }
    if ($op -eq '>' -or $op -eq '>=' -or $op -eq '<') {
        $want = $w[3]
        if (-not $known) { if ($say) { Fail "line ${lineNo}: no such fact '$key'" }; return $false }
        $holds = (Is-Number $got) -and (Is-Number $want) -and $(switch ($op) {
                '>' { [double]$got -gt [double]$want } '>=' { [double]$got -ge [double]$want } '<' { [double]$got -lt [double]$want } })
        if ($holds) { if ($say) { Say "  ok   $key = $got $op $want" }; return $true }
        if ($say) { Fail "line ${lineNo}: $key is '$got', expected $op $want" }
        return $false
    }
    if ('below', 'above', 'left-of', 'right-of' -contains $op) {
        $probe = Read-Probe
        $a = Probe-Entry $probe $key; $b = Probe-Entry $probe $w[3]
        if (-not $a -or -not $b) { if ($say) { Fail "line ${lineNo}: $key is not $op $($w[3]): missing" }; return $false }
        $holds = switch ($op) {
            'below' { $a.y -ge $b.y + $b.height } 'above' { $a.y + $a.height -le $b.y }
            'left-of' { $a.x + $a.width -le $b.x } 'right-of' { $a.x -ge $b.x + $b.width } }
        if ($holds) { if ($say) { Say "  ok   $key $op $($w[3])" }; return $true }
        if ($say) { Fail "line ${lineNo}: $key is not $op $($w[3])" }
        return $false
    }
    $want = ($w[2..($w.Count - 1)] -join ' ')
    if (-not $known) { if ($say) { Fail "line ${lineNo}: no such fact '$key'" }; return $false }
    # A path is written with / in the scripts, and Windows gives \ where the application joins one
    # on: a separator matches either, as in a contains check.
    if ($got.Replace('\', '/') -eq $want.Replace('\', '/')) { if ($say) { Say "  ok   $key = $want" }; return $true }
    if ($say) { Fail "line ${lineNo}: $key is '$got', expected '$want'" }
    return $false
}

# ---- the run -----------------------------------------------------------------------------------

$tLaunch = Now-Ms
Start-App
$t0 = Now-Ms
Write-Output "window after $(Secs ($t0 - $tLaunch)) s; budget $Budget s from here ($BudgetFrom)"
$hardStopMs = [long](($Budget * $TimeScale + $HardStopMargin) * 1000)

function Check-HardStop {
    if ((Now-Ms) - $t0 -gt $hardStopMs) {
        Say "FAIL: hard stop - $ScriptName was still running $($Budget * $TimeScale + $HardStopMargin) s after its window"
        Shot ([IO.Path]::GetFileNameWithoutExtension($ScriptName) + '-hardstop.png')
        $script:Failures++
        Stop-App
        Say "${ScriptName}: $($script:Failures) failure(s)"
        exit 1
    }
}

try {
    $lineNo = 0
    foreach ($raw in $lines) {
        $lineNo++
        if ($raw -match '^\s*#') { continue }
        # `$WORK` is the scratch directory on every line, as in `env` and `setup`: a path typed
        # into a box or expected back is the run's own, not a fixed /tmp the planner reads as
        # C:\tmp where Git Bash's setup wrote to the temporary folder.
        $line = ($raw -split '\s#', 2)[0].Trim().Replace('$WORK', $WorkFwd)
        if (-not $line) { continue }
        $line = For-ThisPlatform $line
        if ($null -eq $line) { continue }
        $w = $line -split '\s+'
        Check-HardStop
        Write-Output ("t=+{0}s line {1}: {2}" -f (Secs ((Now-Ms) - $t0)), $lineNo, $line)
        if ('click', 'doubleclick', 'hover', 'scroll', 'reveal', 'type', 'key' -contains $w[0]) {
            if (-not [GuiInput]::IsWindow($script:Hwnd)) { Fail "line ${lineNo}: the application has no window"; break }
        }
        switch ($w[0]) {
            { 'screen', 'window', 'tiles', 'env', 'setup', 'budget' -contains $_ } { }
            'settle' { Start-Sleep -Milliseconds ([int]([double]$(if ($w.Count -gt 1) { $w[1] } else { 1 }) * 1000)) }
            'click' {
                $target = $w[1]; $button = 1
                if ($target.EndsWith(':right')) { $button = 3; $target = $target.Substring(0, $target.Length - 6) }
                elseif ($target.EndsWith(':middle')) { $button = 2; $target = $target.Substring(0, $target.Length - 7) }
                Wait-Publishes 1
                Raise-Window
                $at = Resolve-Control $target
                if ($at) {
                    Write-Output "clicking '$target' (button $button) at window-relative $(Move-Pointer $at)"
                    Start-Sleep -Milliseconds 30
                    Press $button
                } else { Fail "line ${lineNo}: could not click '$target'" }
                Start-Sleep -Milliseconds 100
            }
            'doubleclick' {
                Raise-Window
                $at = Resolve-Control $w[1]
                if ($at) {
                    Write-Output "double-clicking '$($w[1])' at window-relative $(Move-Pointer $at)"
                    Start-Sleep -Milliseconds 50
                    Press 1; Start-Sleep -Milliseconds 80; Press 1
                } else { Fail "line ${lineNo}: could not double-click '$($w[1])'" }
                Start-Sleep -Milliseconds 100
            }
            'hover' {
                $at = Resolve-Control $w[1]
                if ($at) { Write-Output "hovering '$($w[1])' at window-relative $(Move-Pointer $at)" }
                else { Fail "line ${lineNo}: could not hover '$($w[1])'" }
                Start-Sleep -Milliseconds 100
            }
            'scroll' {
                $direction = if ($w.Count -gt 2) { $w[2] } else { 'down' }
                $notches = if ($w.Count -gt 3) { [int]$w[3] } else { 1 }
                # 150 ms between notches, not 60: gpui's Windows backend drops an input message
                # that arrives while its input callback is still out (`callbacks.input.take()`),
                # and a debug build's frame in the VM can outlast 60 ms - log-zoom's two notches
                # zoomed once one run in two (2026-09-27).
                $gap = if ($w.Count -gt 4) { [int]$w[4] } else { 150 }
                Raise-Window
                $at = Resolve-Control $w[1]
                if ($at) {
                    Write-Output "scrolling '$($w[1])' $direction $notches at window-relative $(Move-Pointer $at)"
                    Start-Sleep -Milliseconds 50
                    for ($i = 0; $i -lt $notches; $i++) { Wheel $direction; Start-Sleep -Milliseconds $gap }
                } else { Fail "line ${lineNo}: could not scroll '$($w[1])'" }
                Start-Sleep -Milliseconds 100
            }
            'reveal' {
                $list = $w[1]; $entryName = $w[2]; $revealed = $false; $direction = 'down'
                Raise-Window
                if (-not (Probe-Entry (Read-Probe) $entryName)) {
                    $at = Resolve-Control $list
                    if ($at) { [void](Move-Pointer $at); Start-Sleep -Milliseconds 50; for ($i = 0; $i -lt 45; $i++) { Wheel 'up'; Start-Sleep -Milliseconds 60 }; Start-Sleep -Milliseconds 100 }
                }
                for ($i = 0; $i -lt 45; $i++) {
                    $probe = Read-Probe
                    $l = Probe-Entry $probe $list; $e = Probe-Entry $probe $entryName
                    if ($l -and $e) {
                        $top = $l.y; $bottom = $l.y + $l.height; $cy = $e.centre_y
                        if ($top + 3 -le $cy -and $cy -le $bottom - 3) { $revealed = $true; break }
                        $direction = if ($cy -gt $bottom - 3) { 'down' } else { 'up' }
                    } else { $direction = 'down' }
                    $at = Resolve-Control $list
                    if (-not $at) { break }
                    [void](Move-Pointer $at); Start-Sleep -Milliseconds 50; Wheel $direction; Start-Sleep -Milliseconds 120
                }
                if ($revealed) { Write-Output "revealed '$entryName' in '$list'" }
                else { Fail "line ${lineNo}: could not reveal '$entryName' in '$list' ($direction)" }
                Start-Sleep -Milliseconds 100
            }
            'type' {
                $text = ($line -replace '^type\s', '')
                Wait-Publishes 2
                Raise-Window
                foreach ($c in $text.ToCharArray()) { [GuiInput]::TypeChar($c); Start-Sleep -Milliseconds 30 }
                Start-Sleep -Milliseconds 300
            }
            'key' {
                Wait-Publishes 2
                Raise-Window
                if (-not (Send-Key $w[1])) { Fail "line ${lineNo}: no key named '$($w[1])' here" }
                Start-Sleep -Milliseconds 100
            }
            'restart' {
                [void][GuiInput]::PostMessage($script:Hwnd, 0x0010, [IntPtr]::Zero, [IntPtr]::Zero)
                if (-not $script:App.WaitForExit(10000)) {
                    Fail "line ${lineNo}: the application did not exit when its window was closed"
                    Stop-Process -Id $script:App.Id -Force -ErrorAction SilentlyContinue
                }
                Set-Content -Path $FactsFile -Value '' -ErrorAction SilentlyContinue
                Set-Content -Path $ProbeFile -Value '' -ErrorAction SilentlyContinue
                Write-Output "restarted after line $($lineNo - 1)"
                Start-App
                # A SITL that left with the application and settled again is not the script's time.
                $t0 += $script:SitlWaitedMs
            }
            'within' { $script:NextWaitMs = [long]([double]$w[1] * 1000) }
            'expect' {
                $tExpect = Now-Ms
                $waitMs = if ($null -ne $script:NextWaitMs) { $script:NextWaitMs } else { $ExpectWaitMs }
                $script:NextWaitMs = $null
                $held = $false
                while ($true) {
                    if (Expect-Once $w $false $lineNo) { $held = $true; break }
                    if ((Now-Ms) - $tExpect -ge $waitMs) { break }
                    if ((Now-Ms) - $t0 -gt $hardStopMs) { break }
                    Start-Sleep -Milliseconds 50
                }
                if ($held) { Write-Output "  ok   $($w[1]) = $($script:LastValue)" } else { [void](Expect-Once $w $true $lineNo) }
                $waited = (Now-Ms) - $tExpect
                if ($waited -gt 150) { Write-Output "       (held after $(Secs $waited) s)" }
            }
            default { Fail "line ${lineNo}: unknown directive '$($w[0])'" }
        }
    }
} finally {
    $total = (Now-Ms) - $t0
    Stop-App
    # Whatever the application started from the scratch directory - the SIMULATION page's SITL,
    # fetched there by sitl-launch.gui - ends with the script. On Linux SITL ends with its parent
    # (SITL_State.cpp checks getppid()); the Cygwin build outlives the planner, and one left
    # running held the VM console's output open, so the suite's job never ended (2026-09-26).
    Get-Process -ErrorAction SilentlyContinue | Where-Object { $_.Path -and $_.Path.StartsWith($Work) } |
        Stop-Process -Force -ErrorAction SilentlyContinue
    # The junctions first - removed as links, not followed - or the scratch directory's removal
    # would walk into itself.
    if ($Keep) {
        Say "kept: $Work"
    } else {
        foreach ($junction in $Junctions) { try { [IO.Directory]::Delete($junction) } catch {} }
        Remove-Item -Recurse -Force $Work -ErrorAction SilentlyContinue
    }
}

Write-Output "took $(Secs $total) s from its window (launch $(Secs ($t0 - $tLaunch)) s)"
if ($total -gt $Budget * 1000) {
    $script:BudgetOver = $true
    Write-Output "OVER BUDGET: $ScriptName took $(Secs $total) s; the budget is $Budget s$(if ($script:Failures -eq 0) { ' (budget only)' })"
}
if ($script:Failures -gt 0) {
    Write-Output "${ScriptName}: $($script:Failures) failure(s)"
    exit 1
}
if ($script:BudgetOver) { exit 4 }
Write-Output "${ScriptName}: passed in $(Secs $total) s"
exit 0
