---
name: windows-unsafe-rulings
description: Buzz's rulings on Windows-only ports that need unsafe FFI - camera capture yes (one file), joystick not now (2026-09-27)
metadata:
  type: project
---

The workspace denies `unsafe`; Buzz allows it file by file for Windows APIs that have no safe
route. Standing so far:

- `crates/mp-transport/src/win32.rs` (SetupAPI board detection) - allowed 2026-09-26.
- **Camera capture on Windows** (the C#'s DirectShow: Video Device, Video Format, Start/Stop, the
  HUD's camera) - **yes, unsafe in one file** of mp-video, each call with what makes it sound;
  tested with a webcam passed through to the VM. Asked and answered 2026-09-27.
- **Joystick on Windows** (JoystickWindows.cs, DirectInput) - **not now** (2026-09-27). The matrix
  row stays open; do not start it without asking again.

**Why:** his call on where `unsafe` goes; see [[delegate-to-opus-subagents]] for who ports it.

**How to apply:** one module per API with `#![allow(unsafe_code)]` and a SAFETY note per call, as
win32.rs does; nowhere else.
