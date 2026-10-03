# The Windows release build in the tiny10 VM, the planner alone (headless-planner is an internal
# testing tool, not part of the application): the commit named, from the bundle in the share,
# built with the release profile and the C runtime linked in (no Visual C++ Redistributable
# needed), opened and painted once, and packaged with the licence
# files into S:\release\ as a zip with its SHA256 - the same contents as the macOS archive.
#   tools/win10/vm-run.sh release-build tools/win10/release-build.ps1   (after setting $Commit)
param([string]$Commit = $env:MPR_COMMIT)
$ErrorActionPreference = 'Continue'
$started = Get-Date
Set-Location C:\src\MissionPlannerRust
Write-Output "== fetching $Commit from \\VBOXSVR\vmshare\mpr.bundle"
git fetch -q \\VBOXSVR\vmshare\mpr.bundle main:refs/remotes/bundle/main 2>&1 | Out-String
git checkout -q -f $Commit 2>&1 | Out-String
$head = (git rev-parse HEAD).Trim()
Write-Output "at $head"
if (-not $head.StartsWith($Commit.Substring(0, 7))) { Write-Output "RELEASE BUILD FAILED: not at $Commit"; exit 1 }
$env:RUSTFLAGS = '-C target-feature=+crt-static'
$env:CARGO_BUILD_JOBS = '4'
Write-Output "== cargo build --release ($((Get-Date).ToString('HH:mm')))"
& "$env:USERPROFILE\.cargo\bin\cargo.exe" build --release -p mp-gui --bin planner 2>&1 |
    Where-Object { $_ -notmatch '^\s+(Compiling|Downloaded|Downloading)' } | ForEach-Object { "$_" }
$code = $LASTEXITCODE
Write-Output "cargo exit $code after $([int]((Get-Date) - $started).TotalMinutes) minutes"
if ($code -ne 0) { Write-Output "RELEASE BUILD FAILED"; exit $code }
Write-Output "== smoke: the release planner opens and paints"
$env:MP_SMOKE = '1'; $env:MP_NO_TILES = '1'; $env:MP_NO_RECORD = '1'
& .\target\release\planner.exe 2>&1 | Select-Object -Last 5 | ForEach-Object { "$_" }
$smoke = $LASTEXITCODE
Write-Output "smoke exit $smoke"
Remove-Item Env:MP_SMOKE, Env:MP_NO_TILES, Env:MP_NO_RECORD
$v = (Select-String -Path Cargo.toml -Pattern '^version = "(.*)"' | Select-Object -First 1).Matches[0].Groups[1].Value
$n = "missionplanner-rust_${v}_windows-x86_64"
$stage = "C:\setup\release\$n"
Remove-Item -Recurse -Force C:\setup\release -ErrorAction SilentlyContinue
New-Item -ItemType Directory -Force -Path $stage | Out-Null
Copy-Item target\release\planner.exe, LICENSE, NOTICE, THIRD_PARTY_LICENSES, README.md $stage
$rustc = (& "$env:USERPROFILE\.cargo\bin\rustc.exe" --version)
@(
  "MissionPlannerRust $v for Windows on x86-64",
  "",
  "Source:  https://github.com/davidbuzz/MissionPlannerRust",
  "Commit:  $head",
  "Built:   $((Get-Date).ToUniversalTime().ToString('yyyy-MM-dd')) on Windows 10 (the tiny10 VM), $rustc, MSVC",
  "Command: cargo build --release -p mp-gui --bin planner",
  "         with the C runtime linked in (+crt-static): no Visual C++ Redistributable needed",
  "",
  "planner.exe  the ground control station",
  "",
  "Unsigned: Windows SmartScreen may ask before the first start.",
  "Licensed under the GNU GPL version 3 (LICENSE); NOTICE and THIRD_PARTY_LICENSES",
  "record what it derives from and what it is built from."
) | Set-Content "$stage\BUILD-INFO.txt"
Compress-Archive -Path $stage -DestinationPath "C:\setup\release\$n.zip"
$hash = (Get-FileHash "C:\setup\release\$n.zip" -Algorithm SHA256).Hash.ToLower()
"$hash  $n.zip" | Set-Content "C:\setup\release\$n.zip.sha256"
New-Item -ItemType Directory -Force -Path S:\release | Out-Null
Copy-Item "C:\setup\release\$n.zip", "C:\setup\release\$n.zip.sha256" S:\release\
Write-Output ("planner.exe {0:N1} MB" -f ((Get-Item "target\release\planner.exe").Length / 1MB))
Write-Output "$hash  $n.zip"
Write-Output "RELEASE BUILD DONE in $([int]((Get-Date) - $started).TotalMinutes) minutes, smoke exit $smoke"
