# Project memory index

One line per memory. Files live in `.claude/memory/` in this repository so they are version
controlled; the per-project memory directory holds symlinks to them.

**CRITICAL:**
- [NOT IN THE C#, NOT IN SCOPE](not-in-the-csharp-not-in-scope.md) — never invent a feature Mission Planner does not have; ask instead
- [Port from the C# source](port-from-the-csharp-source.md) — references/missionplanner is the spec; read the .cs, never recall it
- [Match the original layout](match-the-original-layout.md) — Buzz is the oracle on look and feel; default to MP's arrangement, read it from the .resx

- [No foreground waiting](no-foreground-waiting.md) — background long operations, poll cheaply, never block the session
- [GUI runs stay short](gui-runs-stay-short.md) — windows live ~5s, pinned to DP-1-3; never debug by re-running the GUI
- [GUI tests take the mouse](gui-tests-take-the-mouse.md) — scripts drive the real pointer; run them only on Buzz's word, prefer headless checks, stop means now
- [Verify edits applied](verify-edits-applied.md) — a replacement that matches nothing looks like success
- [Never stop after a commit](never-stop-after-a-commit.md) — chain to the next task; a commit is not an exit condition
- [Verify before committing](verify-before-committing.md) — clippy is its own command, read it, then commit
- [Autotests mandatory](autotests-mandatory.md) — every change ships with a test that fails if it stops working; test the path the product takes
- [Delegate to Opus subagents](delegate-to-opus-subagents.md) — at most three at once (2026-09-25), disjoint files, never a window, never a commit
- [Worktree agents share the target dir](worktree-agents-share-the-target-dir.md) — touch a crate's lib.rs before an integration build while agents run
- [Drop-downs escape the page](dropdowns-escape-the-page.md) — deferred+anchored, thirty rows, wheel-scrolled; the page clips anything else
- [Kill by PID, not pattern](kill-by-pid-not-pattern.md) — pkill -f matches the calling shell; pgrep -fa, then kill the PIDs
