# The updater on Windows, in the VM: the unit tests of crates/mp-update, and a planner.exe built
# as a release updating itself (tools/release/self-update-test.py). Both are cross-compiled on the
# Linux box and put, with the test, in the folder the VM sees as S:\updater by the owner or on his
# word - it is outside the repository (how_to_do_a_beta_or_release.md, "Before a release").
#
#   tools/win10/vm-run.sh updater tools/win10/updater-test.ps1
#
# Everything is copied to C:\mp-updater-test, run there and removed. The exit code is the tests'
# plus the number of the self-update's lines that were other than expected.
$ErrorActionPreference = 'Stop'
$dir = 'C:\mp-updater-test'
if (Test-Path $dir) { Remove-Item -Recurse -Force $dir }
New-Item -ItemType Directory -Path $dir | Out-Null
Copy-Item S:\updater\planner.exe, S:\updater\mp_update-tests.exe, S:\updater\self-update-test.py $dir
$version = (Get-ItemProperty 'HKLM:\SOFTWARE\Microsoft\Windows NT\CurrentVersion')
Write-Output ("Windows: {0} {1} build {2}" -f $version.ProductName, $version.DisplayVersion, $version.CurrentBuild)
Write-Output '== the updater crate''s unit tests'
$ErrorActionPreference = 'Continue'
& "$dir\mp_update-tests.exe" 2>&1 | Select-String -Pattern 'test result|FAILED|panicked|failed' | ForEach-Object { $_.Line }
$tests = $LASTEXITCODE
Write-Output "tests exit $tests"
Write-Output '== planner.exe --version'
& "$dir\planner.exe" --version
Write-Output '== a real planner.exe updating itself'
python "$dir\self-update-test.py" "$dir\planner.exe"
$self = $LASTEXITCODE
Write-Output "self-update exit $self"
Remove-Item -Recurse -Force $dir
exit ($tests + $self)
