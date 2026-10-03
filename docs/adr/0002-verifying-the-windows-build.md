# ADR 0002: Verifying the Windows build

Status: accepted
Date: 2026-09-23

## Context

Deliverable 7 asks for a Windows build that opens a window and paints. Nobody working on this project has a
Windows machine, and the Direct3D 11 backend gpui selects on Windows had, up to this point, never
been run — not once, by anyone.

What existed was compilation checking, and it is worth being precise about how little that proves:

- The `test` job builds the whole workspace on `windows-latest`, including `mp-gui`. So the
  Windows code paths compile and link.
- The `cross-compile` job runs `cargo check --target x86_64-pc-windows-gnu` for the non-GUI
  crates, keeping the core portable.

Neither runs a graphics backend. The failures specific to a graphics backend all happen at
runtime: a device that will not create, a swap chain the driver refuses, a shader that compiles
on one backend's compiler and not another's, a surface format nobody supports. Every one of those
kills the application at startup on the machine that has the problem, and every one is invisible
to a build that does not open a window. A ground station that compiles on Windows and dies on
launch is, to the pilot holding the laptop, a ground station that does not exist.

Cross-compiling the GUI from Linux was tried and does not work here: `ring`, reached through
`ureq`'s rustls, needs a C toolchain for the target, and there is no mingw-w64 installed.
`x86_64-pc-windows-gnu` is also the wrong ABI to be proving things about — gpui's Windows backend
goes through `windows-rs` against MSVC. Installing a cross toolchain would have produced a binary
built differently from the one users get, which is a worse thing to trust than no binary at all.

## Decision

Add a smoke mode to the real binary and run it on all three platforms in CI.

`MP_SMOKE=1` makes `planner` exit 0 once it has painted three frames, and exit non-zero if it has
not painted them within thirty seconds. Three specific choices:

**Frames are counted in `Render::render`**, which gpui calls once per painted frame. Every earlier
signal — a window handle existing, the executor running, `open_window` returning `Ok` — survives on
a machine where the GPU never produced a pixel, and that machine is exactly what this is looking
for.

**Three frames, not one.** A first frame can come from a path a broken backend still limps
through; a second means the swap chain is cycling.

**The watchdog is a thread, not a task.** What is being tested is whether the render loop runs at
all. A watchdog scheduled on the same executor would never fire on the machine where it does not,
and a hang reports nothing.

It is an environment variable rather than a flag so it can be set for a run of the real binary
with its real arguments. A smoke test of a special build proves something about the special build.

On Linux CI, `xvfb` supplies a display and llvmpipe does the rendering — slow, and entirely
sufficient, because the question is whether the backend comes up. On Windows, Direct3D 11 falls
back to WARP where a runner has no GPU; WARP still goes through device creation, swap chain and
shader compilation, which is the whole of what is being checked.

## What this verifies, and what it does not

Verified on every push, on `windows-latest`:

- The Windows build compiles, links and starts.
- The Direct3D 11 backend initialises, creates a surface and paints frames.
- The process exits cleanly.

Not verified, and it should not be claimed:

- **What the window looks like.** A smoke test cannot see. Fonts, DPI scaling, window decorations
  and the whole layout are unchecked on Windows; the progress screenshots are all from Linux.
- **Hardware Direct3D.** WARP is a software rasteriser. A driver bug, or anything depending on a
  feature level WARP reports but a real GPU does not, is not covered.
- **Anything that needs hardware**: serial port enumeration against a real board, USB hotplug,
  joystick input.
- **That it is usable.** Starting is the floor, not the bar.

Closing those needs a person at a Windows machine, and the first one to close is how it looks:
that is where a cross-platform GUI usually differs, and it is the one a screenshot settles in a
minute.

## Consequences

The Direct3D 11 path goes from never-run to run-on-every-push, which is the largest available
improvement without a Windows machine, and CI now fails on a class of bug it previously could not
see on any platform — including Linux and macOS, whose backends were equally unexercised at
runtime.

The cost is a CI step per platform, a few seconds each, and a counter incremented once per frame
in `render`, which is a relaxed atomic add against a function that is doing layout.

A smoke test that passes is easy to read as "Windows works". It does not mean that. This document
exists so the next person to ask has the list above rather than a green tick.
