---
name: worktree-agents-share-the-target-dir
description: Agents in .claude/worktrees build into the main tree's target/, so their artifacts overwrite each other and the main tree's; touch a crate's lib.rs before an integration build
metadata:
  type: project
---

Worktree agents (`isolation: "worktree"`, checkouts under `.claude/worktrees/agent-*`) build
into `/home/buzz/MissionPlannerRust/target`, the main tree's target directory, and cargo names a
worktree's `mp-vehicle` (or any workspace crate) identically to the main tree's. Observed
2026-09-24: the main tree's `mp-mission` was reported `Fresh` while its rlib lacked modules that
were in its source, because a worktree agent's build had replaced the artifact; the CurrentState
agent saw the reverse.

**Why:** cargo's fingerprint does not tell the two source trees apart, so whichever built last
wins, and the next build in the other tree trusts the wrong artifact.

**How to apply:**

- Before an integration build or test in the main tree while any worktree agent is running,
  `touch crates/<crate>/src/lib.rs` for the crates those agents own (or all of `mp-mission`,
  `mp-vehicle`, `mp-tiles`, `mp-link`), so cargo rebuilds them from the main tree's source.
- Tell worktree agents the same, so a "cannot find X in crate" on something plainly there is
  read as a stale artifact, not a code error.
- A test result an agent reports was produced against whatever artifact was present at the
  time; the coordinator's own run after the merge is the one that counts. See
  [[delegate-to-opus-subagents]] and [[verify-before-committing]].
