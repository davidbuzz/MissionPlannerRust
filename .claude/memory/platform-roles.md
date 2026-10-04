---
name: platform-roles
description: Where each platform is built and tested (Buzz, 2026-10-05) - Linux on this machine, macOS on tridge's Mac, Windows only by CI; tiny10 is not used for builds
metadata:
  type: project
---

Buzz, 2026-10-05: "linux=local, and osx=tridge, and windows = leave that for CI".

- **Linux:** build and test here: `cargo` under the build lock, and the GUI scripts headless
  through `tools/gui-headless.sh`.
- **macOS:** tridge's Mac ([[tridge-mac]]). Builds, tests and clippy go there by patch, through
  `~/mpr-*.sh`, and the planner Buzz looks at runs from `~/mpr-show.command`. The layout tour,
  `~/mpr-tour.command`, also runs there.
- **Windows:** CI only, from the `test (windows-latest)` job. Don't start the tiny10 VM for
  builds or tests. On 2026-10-05 Buzz had it stopped, for the memory it holds: 12 GB of the
  host's 31. See [[windows-vm-tiny10]], and [[never-save-the-vm]] still holds.

**Why:** the host's memory cannot hold tiny10, the macOS VM, rust-analyzer and a build at once,
and tridge's Mac builds natively and fast.

**How to apply:**
- A Windows-only problem goes in as a fix with a test, and CI's Windows job proves it.
- Don't bring tiny10 up for it unless Buzz says so.
