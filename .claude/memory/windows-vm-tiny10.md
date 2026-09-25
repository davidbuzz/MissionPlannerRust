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

**Lifecycle from here:** `VBoxManage startvm tiny10 --type gui`, `controlvm tiny10 savestate`,
`snapshot tiny10 take <name>`; `VBoxManage guestcontrol tiny10 run/copyto` works too but wants
the password on the command line, so SSH is the way. Take a snapshot before installing
toolchains. See [[delegate-to-opus-subagents]] (agents never open windows: that includes the
VM's desktop) and [[gui-runs-stay-short]].
