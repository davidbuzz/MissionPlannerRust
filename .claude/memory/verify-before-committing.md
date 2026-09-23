---
name: verify-before-committing
description: Run clippy as its own command and read it; never chain it with git commit
metadata:
  type: feedback
---

**`cargo clippy` and `git commit` go in separate commands.** Chained with `&&` or `;`, the commit
runs whatever clippy said, because the output scrolls past and the shell does not care.

**Why:** this shipped warnings four times in one session — a truncating cast, two
assertions-on-constants, an unused import, an empty format string — each in a commit whose message
implied the tree was clean. Each then needed a follow-up commit or an amend, which is more work
than reading the output would have been, and an amend after a push is worse.

**How to apply:**

1. `cargo fmt --all && cargo clippy --workspace --all-targets` as one command. Read the result.
2. Only then `git add -A && git commit`.
3. A quick check that scales: `cargo clippy ... 2>&1 | grep -cE "^warning: [a-z]"` should print 0.
   Note the `[a-z]` — it skips `warning: mp-gui@0.1.0: using a local link stub for
   libxkbcommon-x11`, which is a build script note on this machine and not a code warning.

The same applies to `cargo test`: run it, read it, then commit. A test count in a commit message
that nobody checked is a claim, not a verification. See [[never-stop-after-a-commit]] - continuing
straight to the next task does not mean skipping the check before the commit.
