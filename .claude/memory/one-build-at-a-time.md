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

**How to apply:**
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
