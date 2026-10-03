---
name: one-build-at-a-time
description: This laptop (31 GB, 16 CPUs) ran out of RAM and CPU on 2026-09-26 with the Windows VM (12 GB), two agent gpui builds, my build and GUI tests at once - the session crashed; one cargo build at a time, agents' builds job-limited, and no build while the VM runs a build
metadata:
  type: feedback
---

**What happened (2026-09-26 ~01:00 local):** the session, the SITL, a GUI suite, the VM
restart and a subagent all died together. Buzz: "oops we crashed due to out of ram and
cpus". Running at that moment: the `tiny10` VM (12 GB, 6 CPUs), two Opus agents each doing a
full gpui build in its own target directory (`target-agent1`, `target-agent2`, ~8 GB and all
cores each at the link step), my own `cargo test -p mp-gui` in `target-solo`, VS Code's
rust-analyzer `cargo check --workspace` in `target/`, a GUI test suite, and SITL. 31 GB and 16
cores are not enough for that.

**Why:** a fresh gpui build is the heaviest thing here; two of them plus the VM's fixed 12 GB
leave nothing for the session itself.

**Again on 2026-09-26 04:26-04:29 local:** one release build of `mp-gui` (`cargo build --release`,
`CARGO_BUILD_JOBS=8`, optimised with debuginfo) beside the running VM was enough on its own:
systemd-oomd killed gnome-shell (the session's fail screen has covered the desktop since - its
unit runs `gnome-session-ctl --shutdown` when stopped, so it is never killed; only Buzz's log
out clears it) and three minutes later the OOM killer killed the VirtualBox process (the VM
"aborted"; it needs Buzz to start it and log in). Release rustc on the gpui tree takes far
more memory per job than a debug build.

**How to apply:**
- With the VM up: a debug build at `CARGO_BUILD_JOBS=4`; a release build at 2-3 jobs inside
  `systemd-run --user --scope -p MemoryMax=...` so the kernel kills the build, not the desktop or
  the VM. NEVER save the VM for a build ([[never-save-the-vm]]: Buzz, 2026-09-26, "SAVE = things
  IMMEDIATELY stop working, do not do"); 6 jobs only once he has turned the VM off himself.
- A shared lock serialises every cargo run, mine and the agents': `flock <scratchpad>/build.lock
  env CARGO_BUILD_JOBS=6 CARGO_TARGET_DIR=... cargo ...`; GUI runs that must be quiet (the storm)
  hold the same lock.
- One cargo build or test compile at a time on this machine, mine or an agent's. An agent gets
  `CARGO_BUILD_JOBS=6` in its brief and is told to build only when told the machine is free, or
  I start agents one at a time and wait for the first's build to finish before the second's.
- With the VM running (or building in it), nothing else builds here. Start the VM's builds
  when the Linux side is idle.
- GUI suites run on an otherwise idle machine: `tools/gui-suite.sh` waits for load < 20, which
  the crash showed is too high - budgets miss under load anyway.
- A separate target directory per agent stays (shared ones thrash, see
  [[worktree-agents-share-the-target-dir]]), but the memory cost is the point: the fresh build
  of each is the spike.
- Everything background died with the session: the SITL (its parent shell), the suite, the
  agents. After a crash: restart SITL, check `git status` (the tree survives), check each
  agent's worktree for partial work before resuming it. See [[delegate-to-opus-subagents]],
  [[no-foreground-waiting]].
- Never edit the main tree's Rust sources while a build that will be shipped (the release into
  dist/) compiles from it: on 2026-09-26 I changed mp-transport and mp-firmware mid-release and
  could not say which version each crate was built from. Work in a worktree until it finishes.
- Release beside the running VM, measured 2026-09-26: the canonical profile (fat LTO, 1 codegen
  unit, debug=1) was OOM-killed at an 11 GB scope compiling mp-gui - VirtualBox's guest RAM does
  not show in `ps` but leaves ~9 GB available. What fits: `CARGO_PROFILE_RELEASE_DEBUG=0` (dist is
  stripped anyway), `CARGO_PROFILE_RELEASE_LTO=thin`, `CARGO_PROFILE_RELEASE_CODEGEN_UNITS=16`,
  2 jobs, `MemoryMax=8G`. Say so when handing the build over; fat LTO needs the VM off (his word).
- **2026-09-26 13:08-13:12, the third memory crash, and how it happened** (Buzz: "mem issue - pls
  check what kernel did"): (1) kernel global OOM killed the release's rustc at 11.2 GB - an 11 GB
  scope cap is no protection when less than that is actually free; (2) systemd-oomd then killed
  VS Code (`app-org.chromium.Chromium-<pid>.scope` IS VS Code's Electron), and with it this
  session and both agents, because **every cargo I start runs inside VS Code's cgroup** (extension
  host -> claude -> bash -> cargo): oomd blames the builds' memory on VS Code. And the VM was
  compiling in the guest at the same time as a host build - guest builds grow VirtualBox from ~5 GB
  toward its full 12 GB, host RAM the `ps` list never shows.
  **How to apply:** every host build goes in its own scope, `systemd-run --user --scope -p
  MemoryHigh=<n> -p MemoryMax=<n> -p OOMScoreAdjust=1000 ...`, sized from `free`'s *available*
  minus a margin (never more than available), so pressure and kills land on the build, not VS
  Code. Never a host build while the VM builds, and never a VM build while the host builds - one
  or the other, checked before starting either.

**With agents on the lock (2026-10-02):** six agents' builds queue on the same lock and a 10-minute
background job of the coordinator's starves (two GUI runs were killed waiting). So agents build
through `<scratchpad>/cargo-agent.sh`, which first waits while `<scratchpad>/coordinator-wants-lock`
exists; the coordinator runs its builds and tests through `<scratchpad>/mine.sh <command>`, which
raises that flag, takes the lock, and drops the flag after. GUI runs hold the lock for the build
only, then run the scripts without it (`gui-bugs5.sh`'s shape).

**A process started under the lock keeps it (2026-10-02):** `flock FILE CMD` hands the lock's
descriptor to CMD and everything CMD starts; the Xvfb that `tools/gui-headless.sh` started inside
a locked GUI run held the build lock for an hour after the run ended, and every build on the
machine waited on a display server. Start long-lived helpers outside the lock, or with the
descriptors closed (`3>&- ... 9>&-`, as gui-headless.sh now does); `lsof <lock>` names the holder.

**Killing a build (2026-10-03):** `kill <cargo pid>` leaves its rustc children running as
orphans, and they inherit the `flock` descriptor, so the lock stays held until the last of them
finishes (a Windows-target rustpython_vm ran five more minutes); kill the rustc PIDs too
(`pgrep -f "rust[c] .*--target x86_64-pc-windows-gnu"`) when the lock is wanted now. Chains
launched from a Claude shell died with VS Code's crash while queued on the lock; a running cargo
survived it. Check the chain logs for their DONE lines after any restart, never assume.
