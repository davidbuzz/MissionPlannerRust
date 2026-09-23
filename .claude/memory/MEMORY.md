# Project memory index

One line per memory. Files live in `.claude/memory/` in this repository so they are version
controlled; the per-project memory directory holds symlinks to them.

**CRITICAL:**
- [NOT IN THE C#, NOT IN SCOPE](not-in-the-csharp-not-in-scope.md) — never invent a feature Mission Planner does not have; ask instead
- [Port from the C# source](port-from-the-csharp-source.md) — referneces/missionplanner is the spec; read the .cs, never recall it
- [Match the original layout](match-the-original-layout.md) — Buzz is the oracle on look and feel; default to MP's arrangement, read it from the .resx

- [No foreground waiting](no-foreground-waiting.md) — background long operations, poll cheaply, never block the session
- [GUI runs stay short](gui-runs-stay-short.md) — windows live ~5s, pinned to DP-1-3; never debug by re-running the GUI
- [Verify edits applied](verify-edits-applied.md) — a replacement that matches nothing looks like success
- [Never stop after a commit](never-stop-after-a-commit.md) — chain to the next task; a commit is not an exit condition
- [Verify before committing](verify-before-committing.md) — clippy is its own command, read it, then commit
