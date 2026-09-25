---
name: scratch-target-dirs-fill-the-disk
description: The session's cargo target directories under the scratchpad grew to 118 GB on 2026-09-26 and filled the root filesystem (Buzz's whole disk, 915 GB at 98%); every tool output and a commit then failed with ENOSPC - prune them as they go
metadata:
  type: feedback
---

**What happened (2026-09-26 ~05:30 local):** `target-solo` (debug + release, with debuginfo) was
65 GB - 39 GB of test executables in `debug/deps` (every `cargo test --workspace` leaves the
last hash's binaries behind, 100-500 MB each) and 19 GB of `debug/incremental`, with a dozen
`planner-<hash>` caches of 1.3 GB each - plus 14 GB and 9 GB per agent target directory. The
root filesystem hit 0 bytes free: the Bash tool's own output files could not be written
("Command output was lost"), rustc could not open an incremental session, and the commit chain
died with exit 128.

**Why:** `/tmp` is on the root filesystem here, not a tmpfs, and that disk is Buzz's, 96% full
without me. Cargo never garbage-collects a target directory.

**How to apply:**
- Before a workspace test run or a release build, `df -h /` - keep at least 40 GB free.
- Prune, in this order: a finished agent's `target-agentN` (whole); `debug/incremental/planner-*`
  but the two newest (`ls -dt planner-* | tail -n +3 | xargs rm -rf`); executables in
  `debug/deps` over 30 MB and older than a couple of hours (`find ... -perm -u+x ! -name '*.so'
  ! -name '*.rlib' -size +30M -mmin +150 -delete` - they relink in seconds, the rlibs stay);
  incremental dirs untouched for three hours.
- One target directory per agent stays (see [[one-build-at-a-time]]), deleted the moment the
  agent's patch is taken.
- After an ENOSPC: check `git log`/`git status` before assuming a commit landed; every log
  written during the window may be truncated.
