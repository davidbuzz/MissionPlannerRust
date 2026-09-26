# The Windows 10 VM: how it is reached and how the port is built there

The Windows build of the planner is made and run in `tiny10`, a Windows 10 VM in Oracle
VirtualBox on this laptop. This is what was set up on 2026-09-26, so it can be done again on
another machine, and what to type to use it.

## The VM

| | |
|---|---|
| Name | `tiny10` (VirtualBox 7.2.16) |
| Guest | Windows 10 Enterprise LTSC, build 19044, 64-bit; a stripped image, so the OpenSSH feature package is not in it |
| Resources | 6 CPUs, 12 GB, 80 GB free on `C:`, VirtualBox WDDM graphics adapter |
| Network | NAT. From inside the guest this laptop is `10.0.2.2`, so SITL running here is `tcp:10.0.2.2:5760` |
| Guest Additions | 7.2.16, installed |
| Account | `user`, an administrator |

Lifecycle from the host shell:

```sh
VBoxManage startvm tiny10 --type gui        # start, with its window
VBoxManage controlvm tiny10 savestate       # suspend to disk
VBoxManage controlvm tiny10 acpipowerbutton # shut down
VBoxManage showvminfo tiny10 --machinereadable | grep VMState
VBoxManage snapshot tiny10 take <name>      # only with the VM off or saved - see below
```

**Never reboot it and expect it to come back.** A restart lands on the login screen, and
nothing (not sshd's key login either) is reachable until the human logs in at the console.
Installers are run with `--norestart`; an exit code asking for a reboot (3010) is reported, not
acted on.

**Never take a live snapshot of it.** `VBoxManage snapshot tiny10 take <name> --live` on the
running VM aborted it on 2026-09-26 (state "aborted", as if the power had been pulled). Save or
shut it down first. The one snapshot that exists, `before-rust-toolchain-2026-09-26`, was taken
of that aborted state, before any toolchain went in.

**Resources on the host.** With the VM up (12 GB of the laptop's 31) there is room for one
cargo build on the Linux side, not two, and none while the VM itself is building. The session
crashed on 2026-09-26 with the VM, two subagents' gpui builds, a third build and a GUI test
suite running at once.

## Reaching it: SSH

Windows' own `Add-WindowsCapability -Online -Name OpenSSH.Server~~~~0.0.1.0` fails on this
image (`0x800f0950`, the feature source is not there), so the standalone Win32-OpenSSH MSI was
used instead. In the VM, as administrator, in PowerShell:

```powershell
Invoke-WebRequest -Uri https://github.com/PowerShell/Win32-OpenSSH/releases/download/10.0.0.0p2-Preview/OpenSSH-Win64-v10.0.0.0.msi -OutFile $env:TEMP\OpenSSH.msi
msiexec /i $env:TEMP\OpenSSH.msi /qn
Start-Sleep 10; Get-Service sshd            # Running
```

The MSI registers `sshd` as an automatic service, starts it, opens port 22 in the firewall, and
makes PowerShell the login shell. The host's public key goes into the administrators' key file
(the account is an administrator; a plain account would use `C:\Users\<name>\.ssh\authorized_keys`
and no `icacls`):

```powershell
$k = 'ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAILbaInw8fgn76i0rkI7YXvmmpR63RRbHiCCpdYATrgDA buzz'
Set-Content -Path C:\ProgramData\ssh\administrators_authorized_keys -Value $k
icacls C:\ProgramData\ssh\administrators_authorized_keys /inheritance:r /grant "Administrators:F" /grant "SYSTEM:F"
```

On the host, a NAT port forward from the laptop's 2222 to the guest's 22 (added to the running
VM; it is kept in the VM's configuration):

```sh
VBoxManage controlvm tiny10 natpf1 "ssh,tcp,,2222,,22"
ssh -p 2222 user@localhost                  # PowerShell prompt in the VM
scp -P 2222 file user@localhost:C:/setup/   # files in and out
```

The key is this laptop's `~/.ssh/id_ed25519`. A command runs as `ssh -p 2222 user@localhost
'<powershell>'`; a long one is started detached so the SSH session's end does not end it:
`Start-Process powershell -ArgumentList "-NoProfile","-ExecutionPolicy","Bypass","-File","C:\setup\x.ps1" -WindowStyle Hidden`,
writing its own log.

## Files both ways: the shared folder

The host directory `/home/buzz/vmshare` is shared into the VM as `vmshare`, reachable at once as
`\\VBOXSVR\vmshare` and auto-mounted as `S:`. It was added to the running VM, which VirtualBox
only allows as *transient*, so it is gone after a power-off; make it permanent with the VM off:

```sh
VBoxManage sharedfolder add tiny10 --name vmshare --hostpath /home/buzz/vmshare --automount --auto-mount-point S:
# or, on a running VM, again but transient:
VBoxManage sharedfolder add tiny10 --name vmshare --hostpath /home/buzz/vmshare --transient --automount --auto-mount-point S:
```

Use it for installers, built binaries, logs and screenshots. Do not build the Rust tree on it:
`vboxsf` is slow, and cargo's file locks and symlinks misbehave there. The repository is cloned
onto the VM's own disk.

## The toolchain

`tools/win10/install-toolchain.ps1` (version controlled here; copied to `C:\setup\` in the VM)
does the whole install, detached, logging to `C:\setup\install.log`:

1. Visual Studio Build Tools 2022 from `https://aka.ms/vs/17/release/vs_BuildTools.exe`, run
   as `--quiet --wait --norestart --nocache --add Microsoft.VisualStudio.Workload.VCTools --add
   Microsoft.VisualStudio.Component.VC.CMake.Project --includeRecommended` (the C++ workload
   with the Windows SDK and CMake; the MSVC linker and headers a `-msvc` Rust target needs).
2. `rustup-init.exe` from `https://win.rustup.rs/x86_64`, run as `-y --default-toolchain
   stable-x86_64-pc-windows-msvc --profile minimal -c clippy`.

Git was already in the image (`C:\Program Files\Git\cmd\git.exe`). Python, CMake (before step
1) and winget were not. The log's last line reads `done` when both have finished; `vs build
tools exit 0` and `rustup exit 0` are the two results to look for (3010 from the VS installer
means a reboot is wanted).

## Building the port in the VM

Done on 2026-09-26. The repository reaches the VM as a git bundle through the share
(`git bundle create /home/buzz/vmshare/mpr.bundle HEAD main` on the laptop, then in the VM
`git clone -b main \\VBOXSVR\vmshare\mpr.bundle C:\src\MissionPlannerRust`; later commits
come the same way, as a bundle to `git pull` from). `rust-toolchain.toml` pins 1.95.0, which
rustup installed on first use (`rustup toolchain install 1.95.0-x86_64-pc-windows-msvc -c
rustfmt -c clippy -c rust-src` ahead of time saves the wait).

`tools/win10/build-planner.ps1`, started detached by `start-build-task.ps1`, runs
`cargo build -p mp-gui` with `CARGO_BUILD_JOBS=5`, logging to `C:\setup\build.log` and the
compiler's output to `C:\setup\cargo-build.log`. The first build took 31 minutes on the VM's
six cores (the zed checkout for gpui is the long download) and produced
`target\debug\planner.exe`, 88.9 MB, with one warning - a joystick field the Windows build
never reads, since the stick reader is Linux's; annotated with the reason.

## Running it against the laptop's SITL

`tools/win10/run-planner.ps1`, started by `start-run-task.ps1` as a task in the logged-in
session (so its window is on the VM's desktop), runs `planner.exe tcp:10.0.2.2:5760` with
`MP_FACTS=C:\setup\planner.facts` and `MP_NO_TILES=1`, waits twenty seconds, takes a screenshot of
the desktop to `S:\win10-planner.png`, copies the facts to `S:\win10-planner.facts` and closes
the planner. The first run, 2026-09-26: the window came up on the VirtualBox WDDM adapter
(gpui's Direct3D 11 path), the link opened, 1,400 frames were heard, `vehicle.count` 1,
`params.held` 1408, the flight screen with the HUD reading "Ready to Arm" and the SITL's
banner in the Messages panel; 2,921 fact lines. That is the first Windows run of the
application.

## The GUI suite in the VM

`tools/win10/gui-test.ps1` runs one `tests/gui/*.gui` script on user32 as `tools/gui-test.sh`
runs it on X11, and `tools/win10/gui-suite.ps1 -All` (or `-Names a,b`) runs the lot, one line a
script, the results to `C:\setup\suite\results.txt` and the share's `win-suite-results.txt`.
Jobs go through `tools/win10/vm-run.sh` so they show on the VM's console window. A script that
needs SITL gets the VM's own - the Cygwin ArduCopter the planner's SIMULATION page fetched into
`Documents\MissionPlannerRust\sitl` - one for the whole suite behind
`tools/win10/sitl-relay.ps1`, as the Cygwin build exits when its client leaves. Each script's
Windows budget is in `tools/win10/budgets.txt`, which the suite rewrites after every run.

**Windows Defender Firewall asks about every ArduCopter.exe (Buzz, 2026-09-26).** The first
time an `ArduCopter.exe` opens its TCP ports, Windows shows "Windows Defender Firewall has
blocked some features of this app" for it, and somebody at the VM has to click **Allow
access**. The firewall remembers the answer per program *path*, so it asks again for any copy
in a new place:

- the suite's SITL, `C:\Users\user\Documents\MissionPlannerRust\sitl\ArduCopter.exe` - once;
- the SITL that `sitl.gui` and `sitl-launch.gui` have the planner fetch and start in the run's
  own scratch directory (`%TEMP%\planner-work-*\...\sitl\ArduCopter.exe`) - a new path, and so
  a new prompt, on every run of those two scripts;
- `planner.exe` itself asked once, on its first run (answered Allow on 2026-09-26); rebuilding
  it in place keeps the path, so it has not asked since.

Until it is answered the prompt sits over the desktop, where the runner's clicks can land on it
instead of the planner.

**Turned off by policy, 2026-09-26 (Buzz's ask).** `tools/win10/quiet-firewall.ps1` sets
`NotifyOnListen False` for the Domain, Private and Public profiles in the local group policy
(`Set-NetFirewallProfile -PolicyStore localhost`) and refreshes policy (`gpupdate /target:computer
/force`); the active store then reads `NotifyOnListen False` with the firewall still enabled and
inbound still `Block` by default. Checked: a copy of `ArduCopter.exe` started from a path the
firewall had never seen listened on its port and no "Windows Security Alert" appeared. A new
program that listens is still blocked from connections off the VM - it is only no longer asked
about - and everything the GUI suite does is loopback, which the firewall does not filter. Undo
with the same command and `-NotifyOnListen NotConfigured`.

## Where the VM tooling lives

Everything written for the VM on the Linux side is in `tools/win10/` in this repository, not
in the share or the VM alone: the toolchain installer above, the build and run scripts that
follow it, the console (`console-runner.ps1`, `vm-run.sh`) and the GUI suite's runner, relay
and budgets. The share and `C:\setup\` hold copies.

## What is recorded elsewhere

The memory note `.claude/memory/windows-vm-tiny10.md` holds the access details for future
sessions; PLAN.md §13 and NOT_DONE_YET_MATRIX.md track the Windows row ("First Windows run of
the application").
