# Rust toolchain for the Mission Planner port, in the tiny10 VM: Visual Studio Build Tools
# (the C++ workload with the Windows SDK, and CMake) and rustup with the MSVC stable toolchain.
# Runs detached as a scheduled task (start-install-task.ps1) so an SSH session's end does not
# end it, elevated so the installers need no consent prompt, and never reboots: a 3010 from the
# VS installer is logged for the human to act on. Progress and errors go to C:\setup\install.log.
$ErrorActionPreference = 'Continue'
$log = 'C:\setup\install.log'
New-Item -ItemType Directory -Force -Path C:\setup | Out-Null
function Log($m) { "$(Get-Date -Format s) $m" | Tee-Object -FilePath $log -Append }
Log "start as $env:USERNAME, elevated: $(([Security.Principal.WindowsPrincipal][Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator))"
[Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12
try {
    if (-not (Test-Path C:\setup\vs_BuildTools.exe)) {
        Invoke-WebRequest -Uri https://aka.ms/vs/17/release/vs_BuildTools.exe -OutFile C:\setup\vs_BuildTools.exe
    }
    Log "vs bootstrapper present"
    $p = Start-Process -FilePath C:\setup\vs_BuildTools.exe -ArgumentList '--quiet','--wait','--norestart','--nocache','--add','Microsoft.VisualStudio.Workload.VCTools','--add','Microsoft.VisualStudio.Component.VC.CMake.Project','--includeRecommended' -Wait -PassThru
    Log "vs build tools exit $($p.ExitCode) (0 ok, 3010 wants a reboot - tell the human, do not reboot)"
} catch {
    Log "vs build tools failed: $($_.Exception.Message)"
}
try {
    if (-not (Test-Path C:\setup\rustup-init.exe)) {
        Invoke-WebRequest -Uri https://win.rustup.rs/x86_64 -OutFile C:\setup\rustup-init.exe
    }
    Log "rustup-init present"
    $p = Start-Process -FilePath C:\setup\rustup-init.exe -ArgumentList '-y','--default-toolchain','stable-x86_64-pc-windows-msvc','--profile','minimal','-c','clippy' -Wait -PassThru -NoNewWindow
    Log "rustup exit $($p.ExitCode)"
} catch {
    Log "rustup failed: $($_.Exception.Message)"
}
Log "done"
