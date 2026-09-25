# Runs the planner built in the tiny10 VM against the SITL on the laptop (10.0.2.2:5760 from the
# guest), publishing its facts to C:\setup\planner.facts, and after twenty seconds takes a
# screenshot of the desktop to S:\win10-planner.png (the shared folder) and closes it. The
# facts file is what the first Windows run is judged by: a window up, the link open, a
# heartbeat heard (vehicle.count), the parameters in. Detached through start-run-task.ps1.
$ErrorActionPreference = 'Continue'
$log = 'C:\setup\run.log'
function Log($m) { "$(Get-Date -Format s) $m" | Tee-Object -FilePath $log -Append }
Log "start"
$exe = 'C:\src\MissionPlannerRust\target\debug\planner.exe'
if (-not (Test-Path $exe)) { Log "no planner.exe"; exit 1 }
$env:MP_FACTS = 'C:\setup\planner.facts'
$env:MP_NO_TILES = '1'
Remove-Item C:\setup\planner.facts -ErrorAction SilentlyContinue
$p = Start-Process -FilePath $exe -ArgumentList 'tcp:10.0.2.2:5760' -PassThru -RedirectStandardError C:\setup\planner.stderr.log -RedirectStandardOutput C:\setup\planner.stdout.log
Log "planner pid $($p.Id)"
Start-Sleep 20
Add-Type -AssemblyName System.Drawing
Add-Type -AssemblyName System.Windows.Forms
$bounds = [System.Windows.Forms.Screen]::PrimaryScreen.Bounds
$bmp = New-Object System.Drawing.Bitmap $bounds.Width, $bounds.Height
$g = [System.Drawing.Graphics]::FromImage($bmp)
$g.CopyFromScreen($bounds.Location, [System.Drawing.Point]::Empty, $bounds.Size)
$shot = 'C:\setup\win10-planner.png'
$bmp.Save($shot, [System.Drawing.Imaging.ImageFormat]::Png)
Copy-Item $shot \\VBOXSVR\vmshare\win10-planner.png -ErrorAction SilentlyContinue
Log "screenshot $shot"
if (Test-Path C:\setup\planner.facts) {
    Copy-Item C:\setup\planner.facts \\VBOXSVR\vmshare\win10-planner.facts -ErrorAction SilentlyContinue
    Log "facts: $((Get-Content C:\setup\planner.facts | Measure-Object -Line).Lines) lines"
} else { Log "no facts file" }
Stop-Process -Id $p.Id -ErrorAction SilentlyContinue
Log "exited: $($p.HasExited)"
Log "done"
