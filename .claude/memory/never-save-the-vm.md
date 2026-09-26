---
name: never-save-the-vm
description: NEVER savestate (or pause, snapshot live, power off, reset) the tiny10 VM - Buzz, 2026-09-26: "save is a forced shutdown", "SAVE = things IMMEDIATELY stop working, do not do"
metadata:
  type: feedback
---

On 2026-09-26 ~12:42 I ran `VBoxManage controlvm tiny10 savestate` for a Linux release build
(my own earlier rule said release builds wait for the VM to be saved or off). Buzz: "you stopped
win vm?", "save is a forced shutdown", "SAVE = things IMMEDIATELY stop working, do not do." A
suite running in the VM, his session and anything he was doing stopped mid-flight.

**Why:** the VM is his working machine and he watches work in it; a save stops everything at once,
from his side no different from pulling the plug.

**How to apply:** never `savestate`, `pause`, `poweroff`, `reset`, `acpipowerbutton` or a live
snapshot of tiny10 without his explicit word for that one occasion. Heavy host builds run beside
the running VM instead: a release build at `CARGO_BUILD_JOBS=2`-3 inside
`systemd-run --user --scope -p MemoryMax=...` so memory pressure kills the build, not gnome-shell
or VirtualBox (see [[one-build-at-a-time]]); or wait for him to turn the VM off himself.
See [[windows-vm-tiny10]].
