# Project memory index

One line per memory. Files live in `.claude/memory/` in this repository so they are version
controlled; the per-project memory directory holds symlinks to them.

- [No foreground waiting](no-foreground-waiting.md) — background long operations, poll cheaply, never block the session
- [GUI runs stay short](gui-runs-stay-short.md) — windows live ~5s and clean up; never leave MP_BENCH on
- [Verify edits applied](verify-edits-applied.md) — a replacement that matches nothing looks like success
- [Never stop after a commit](never-stop-after-a-commit.md) — chain to the next task; a commit is not an exit condition
