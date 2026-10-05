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

**A ledger's readers live in other crates.** `mp-vehicle`'s CurrentState ledger feeds
`mp-gui`'s quick view test (every done numeric property needs a reader): committing mp-vehicle
after `cargo test -p mp-vehicle -p mp-link` alone left mp-gui red (2026-09-24). Before a commit,
`cargo test --workspace`, not the changed crates' tests.

**A merge's scripts are not the merge's proof.** The state wiring merged at 12:16 on 2026-09-24
moved `StreamRates::set_backups` after the connect; its own scripts passed, and `config-radio.gui`,
last run at 06:30, had been failing since (the vehicle's rates raced start-up). A merge that touches
start-up, `main.rs`, `telemetry.rs` or a static every screen reads gets the whole suite
(`tools/gui-suite.sh -o <dir> $(ls tests/gui/*.gui | xargs -n1 basename | sed 's/\.gui$//')`,
about 80 minutes, quiet machine, no GUI rebuild while it runs), not the merged rows' scripts. A
`PASS` logged before the merge proves nothing about the tree after it.

**A new dependency edge is checked by xtask, not by the crates.** `xtask/tests/graph.rs` holds
PLAN.md §5.1's layers; mag calb log's commit of 2026-10-06 gave mp-calibration (L3) a dependency on
mp-log (L4), which its own crates' tests, clippy and the wasm check all passed, and the graph test
refused (the code moved up to mp-log the same day). Any Cargo.toml change: `cargo test -p xtask
--test graph --test licences` before the commit, at least, when the whole workspace is too long.
