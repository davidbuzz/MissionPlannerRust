---
name: macos-vm-qemu
description: The local macOS Sonoma VM under QEMU/KVM in ~/macos-vm (OSX-KVM): what boots it, how to start/stop/see it, and that Buzz drives its screen himself (2026-10-03/04)
metadata:
  type: reference
---

**Where:** `~/macos-vm` on the root disk (nvme1n1p2) - never `/media/buzz/10A`
([[10a-is-for-backups]]). `OSX-KVM/` (kholia/OSX-KVM clone), `OSX-KVM/BaseSystem.img` (Sonoma
recovery, fetched with `fetch-macOS-v2.py -s sonoma`, verified against Apple's chunklist),
`OSX-KVM/mac_hdd_ng.img` (64 GB qcow2, the install; the disk inside is named "buzz"),
`OpenCore-debug.img`, `OVMF_VARS.fd`, `start.sh`, `vmctl.py`, `watchdog.sh`.

**What boots it (found 2026-10-03):** the CPU model `Skylake-Client,-hle,-rtm,...` (start.sh's
default, CPU_MODEL=skylake) - with OSX-KVM's older Haswell-noTSX note for Sonoma the kernel reset
the instant boot.efi handed over ("HANDOFF TO XNU", then OVMF again), which is what defeated
the Docker-OSX attempts too; and `OC=debug`, a copy of OSX-KVM's OpenCore with the picker's
Timeout 0 (its 2-second auto-boot picked the "EFI" entry and failed "Already started"), HaltLevel
0, the log on serial and file only, and `-v` boot-args. `OC=debug ./start.sh` writes the serial
log to `serial.log`.

**Seeing and driving it:** VNC on 127.0.0.1:5907 (Remmina `-c vnc://127.0.0.1:5907` on
DISPLAY=:0); QMP on `qmp.sock` through `vmctl.py` (shot, key, type, click). Pointer events need the
USB tablet made QEMU's current mouse (`mouse_set`, done by vmctl), and `-monitor/-serial none`
(no text consoles) or targeted input crashes QEMU 8.2. **Buzz drives its screen** (2026-10-04:
"no you won't, i will") - I send it no keys or clicks unless Buzz asks for that action.

**Lifecycle:** stop with `vmctl.py powerdown` (then `quit` only if it ignores that), never while
it is installing or building unless Buzz says so - see [[acceptable-is-not-do-it-now]]. After the
forced stop of 2026-10-04 the picker waited as usual and Buzz chose the installer; when a boot
gets past the picker, ask Buzz rather than guessing why.

**Buzz's notes:** `hackintosh-vm-dev.md` at the repository root (untracked, his) - keep it up to
date as the VM progresses (2026-10-04: "pls keep this file up to date"): the status table on top,
and at the bottom a short note that Docker-OSX was tried several ways and not used because it
would not boot - its commands removed at Buzz's request (2026-10-04).

**Installed 2026-10-04 12:05**, desktop 12:11; the local account is `user` - its password is in
Buzz's untracked hackintosh-vm-dev.md, never in a committed file (this repository is public).
