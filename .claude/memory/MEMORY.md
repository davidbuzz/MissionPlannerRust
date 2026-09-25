# Project memory index

One line per memory. Files live in `.claude/memory/` in this repository so they are version
controlled; the per-project memory directory holds symlinks to them.

**CRITICAL:**
- [NOT IN THE C#, NOT IN SCOPE](not-in-the-csharp-not-in-scope.md) — never invent a feature Mission Planner does not have; ask instead
- [Port from the C# source](port-from-the-csharp-source.md) — references/missionplanner is the spec; read the .cs, never recall it
- [No callers, no port](no-callers-no-port.md) — a .cs file or function nobody calls is recorded out of scope, never ported (Buzz, 2026-09-25)
- [Match the original layout](match-the-original-layout.md) — Buzz is the oracle on look and feel; default to MP's arrangement, read it from the .resx
- [No dialogs for avoidable errors](no-dialogs-for-avoidable-errors.md) — an error the main window can show as state/connectivity/colour never gets a message box (Buzz, 2026-09-25)
- [Mute on language](mute-on-language.md) — English when `language` is empty; no i18n questions or reports until told otherwise (Buzz, 2026-09-25)

- [No foreground waiting](no-foreground-waiting.md) — background long operations, poll cheaply, never block the session
- [GUI runs stay short](gui-runs-stay-short.md) — windows live ~5s, pinned to DP-1-3; never debug by re-running the GUI
- [GUI tests take the mouse](gui-tests-take-the-mouse.md) — scripts drive the real pointer; run them only on Buzz's word, prefer headless checks, stop means now
- [Verify edits applied](verify-edits-applied.md) — a replacement that matches nothing looks like success
- [Never stop after a commit](never-stop-after-a-commit.md) — chain to the next task; a commit is not an exit condition
- [Verify before committing](verify-before-committing.md) — clippy is its own command, read it, then commit
- [Autotests mandatory](autotests-mandatory.md) — every change ships with a test that fails if it stops working; test the path the product takes
- [Delegate to Opus subagents](delegate-to-opus-subagents.md) — at most six at once (2026-09-25), disjoint files, never a window, never a commit
- [Worktree agents share the target dir](worktree-agents-share-the-target-dir.md) — touch a crate's lib.rs before an integration build while agents run
- [Drop-downs escape the page](dropdowns-escape-the-page.md) — deferred+anchored, thirty rows, wheel-scrolled; the page clips anything else
- [Kill by PID, not pattern](kill-by-pid-not-pattern.md) — pkill -f matches the calling shell; pgrep -fa, then kill the PIDs
- [Bench CubeOrange](bench-cubeorange.md) — its Zephyr bootloader replaced with stock via the BMP on 2026-09-25; by-id path; MP_FIRMWARE_PORT; flash only on Buzz's go
