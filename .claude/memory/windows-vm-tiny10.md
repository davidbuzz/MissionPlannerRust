---
name: windows-vm-tiny10
description: Buzz's Windows 10 VM "tiny10" in VirtualBox on this laptop, reachable by SSH (port 2222, user `user`, PowerShell shell) with a shared folder; set up 2026-09-26 for the Windows work
metadata:
  type: reference
---

**The VM:** VirtualBox 7.2 VM `tiny10` (a stripped Windows 10, 6 CPUs, 12 GB, NAT networking,
Guest Additions 7.2.16). Buzz allowed launching and running tasks on it on 2026-09-26 ("i want
to allow you to launch and run tasks on it").

**Access:** `ssh -p 2222 user@localhost` from this laptop (key: this laptop's
`~/.ssh/id_ed25519`, in `C:\ProgramData\ssh\administrators_authorized_keys`; the account
`user` is an administrator). The login shell is PowerShell (Win32-OpenSSH 10.0.0.0 MSI, since
the stripped image has no OpenSSH capability: `Add-WindowsCapability` fails 0x800f0950). The NAT
port forward `ssh,tcp,,2222,,22` was added with `VBoxManage controlvm tiny10 natpf1` on a
running VM; it persists in the VM's config. From inside the guest this laptop is `10.0.2.2`
(SITL at `tcp:10.0.2.2:5760`).

**Files:** host `/home/buzz/vmshare` is the VM's share `vmshare` (`\\VBOXSVR\vmshare`, auto-mount
`S:`), added transient while the VM ran - re-add it permanently when the VM is off
(`VBoxManage sharedfolder add tiny10 --name vmshare --hostpath /home/buzz/vmshare --automount
--auto-mount-point S:`). Use it to move installers, binaries, logs and screenshots; never build
the Rust tree on it (vboxsf is slow, and cargo's locks and symlinks misbehave) - the repo goes on
the VM's own disk.

**Never reboot or power-cycle it and expect it back** (Buzz, 2026-09-26: "it requires me, the
human, to be present and log it back in") - a restart lands on the login screen with no
network services until he logs in. Installers run with `--norestart`; an exit code that asks for
a reboot (3010) is reported to him, never acted on. A live snapshot aborted the VM once; snapshots
only with it saved or off.

**2026-09-26 04:29 local: the VM was killed by the OOM killer** (a release build beside it, see
[[one-build-at-a-time]]) and is "aborted"; Buzz starts it and logs in - I do not start it
(it would boot to the login screen and hold 12 GB for nothing).

**2026-09-26 ~11:00 local: the whole laptop was rebooted.** The VM came back running with Buzz
logged in at the console and sshd up (Automatic service), the clone at `C:\src\MissionPlannerRust`
(at `eadb48a`, a debug `planner.exe` from 2026-09-25) and the toolchain intact. The transient
`vmshare` was gone - re-added with `VBoxManage sharedfolder add tiny10 --name vmshare --hostpath
/home/buzz/vmshare --automount --auto-mount-point S: --transient` (the guest reads
`\\VBOXSVR\vmshare` at once; `S:` may need a new logon). A host reboot also empties `/tmp`, so
every scratch target directory is gone and the next build is cold.

**Work in the VM is watched:** jobs go through `tools/win10/vm-run.sh` into the "Claude at work"
window on its desktop - see [[vm-work-shows-on-its-console]].

**Lifecycle from here:** NEVER save, pause, power off or live-snapshot it - see
[[never-save-the-vm]] ("SAVE = things IMMEDIATELY stop working, do not do"). `VBoxManage startvm
tiny10 --type gui` only to bring it back after something else stopped it; `VBoxManage guestcontrol tiny10 run/copyto` works too but wants
the password on the command line, so SSH is the way. Take a snapshot before installing
toolchains. See [[delegate-to-opus-subagents]] (agents never open windows: that includes the
VM's desktop) and [[gui-runs-stay-short]].

**GUI suite in the VM (2026-09-26):** the clone at `C:\src\MissionPlannerRust` fetches
`\\VBOXSVR\vmshare\mpr.bundle` (`git bundle create /home/buzz/vmshare/mpr.bundle main` on the
laptop), then `git reset --hard origin/main` and a re-checkout (`git rm -r --cached . ; git reset
--hard`) so `.gitattributes` applies. The Cygwin SITL exits within 3 s of its client leaving -
measured - so the suite keeps one alive behind `tools/win10/sitl-relay.ps1`. Windows budgets live
in `tools/win10/budgets.txt`, written by the suite after each run (the share's `win-budgets.txt`
is taken into the repo, LF, ordinal order). Hold `build.lock` for the whole suite run: host builds
beside it slowed the VM and 31 scripts hit their hard stop. The VM's screen is 1640x1320 since
2026-09-26 23:00 (setvideomodehint, Buzz's go), work area 1640x1280: a 1600x1200 window fits;
at 1600x1200 it came out ~1100 high and `plan-add-below` and `setup-list` failed on that alone.
It was set live: re-send the hint if a restart brings it back smaller.

**Windows Defender Firewall prompt (Buzz, 2026-09-26):** every `ArduCopter.exe` at a new path
shows "blocked some features of this app" the first time it opens its ports, and Buzz has to
click **Allow access** at the VM - once for the suite's SITL in `Documents\MissionPlannerRust\sitl`,
and on every run of `sitl.gui`/`sitl-launch.gui`, whose SITL is fetched into the run's scratch
directory. Turned off by local group policy on his ask the same day
(`tools/win10/quiet-firewall.ps1`: NotifyOnListen False, gpupdate); a SITL from a new path then
listened with no alert. If it ever prompts again, the policy was undone or reset. Written up in
win10_vm_setup.md.

