# Project memory index

One line per memory. Files live in `.claude/memory/` in this repository so they are version
controlled; the per-project memory directory holds symlinks to them.

**CRITICAL:**
- [NOT IN THE C#, NOT IN SCOPE](not-in-the-csharp-not-in-scope.md) — never invent a feature Mission Planner does not have; ask instead
- [Port from the C# source](port-from-the-csharp-source.md) — references/missionplanner is the spec; read the .cs, never recall it
- [No callers, no port](no-callers-no-port.md) — a .cs file or function nobody calls is recorded out of scope, never ported (Buzz, 2026-09-25)
- [Match the original layout](match-the-original-layout.md) — Buzz is the oracle on look and feel; default to MP's arrangement, read it from the .resx
- [Mode buttons kept](mode-buttons-kept.md) — the flight screen's own mode buttons panel stays over the C#'s CMB_modes drop-down (Buzz, 2026-09-26)
- [Windows unsafe rulings](windows-unsafe-rulings.md) — unsafe per file for Windows APIs: win32.rs yes, camera capture yes (one file), joystick not now (Buzz, 2026-09-27)
- [No dialogs for avoidable errors](no-dialogs-for-avoidable-errors.md) — an error the main window can show as state/connectivity/colour never gets a message box (Buzz, 2026-09-25)
- [Mute on language](mute-on-language.md) — English when `language` is empty; no i18n questions or reports until told otherwise (Buzz, 2026-09-25)
- [Work the matrix in priority order](work-the-matrix-in-priority-order.md) — every "high" row of NOT_DONE_YET_MATRIX.md to 100% before any "med" row (Buzz, 2026-09-26); within med: Standard/Advanced Params, DroneCAN, MAVFtp, Sik Radio first (Buzz, 2026-10-02)

- [NEVER save the VM](never-save-the-vm.md) — no savestate/pause/poweroff/live snapshot of tiny10 without Buzz's word: "SAVE = things IMMEDIATELY stop working, do not do" (2026-09-26)
- [One build at a time](one-build-at-a-time.md) — one cargo build at a time under `flock <scratchpad>/build.lock`; with the VM up, debug at 4 jobs and NO release build (a release build beside the VM OOM-killed gnome-shell and the VM on 2026-09-26); with agents on the lock the coordinator goes first through mine.sh (2026-10-02)
- [Scratch target dirs fill the disk](scratch-target-dirs-fill-the-disk.md) — /tmp is the root disk, 96% full without me; target-solo hit 65 GB; prune planner-* incremental caches, old test binaries and finished agents' target dirs; df before big runs (2026-09-26)
- [No foreground waiting](no-foreground-waiting.md) — background long operations, poll cheaply, never block the session
- [GUI runs stay short](gui-runs-stay-short.md) — windows live ~5s, pinned to DP-1-3; never debug by re-running the GUI
- [GUI tests take the mouse](gui-tests-take-the-mouse.md) — on the desktop, scripts drive the real pointer: only on Buzz's word, stop means now; `tools/gui-headless.sh` runs them on Xvfb :99 with lavapipe and touches nothing of his
- [Verify edits applied](verify-edits-applied.md) — a replacement that matches nothing looks like success
- [Never stop after a commit](never-stop-after-a-commit.md) — chain to the next task; a commit is not an exit condition
- [Verify before committing](verify-before-committing.md) — clippy is its own command, read it, then commit
- [Autotests mandatory](autotests-mandatory.md) — every change ships with a test that fails if it stops working; test the path the product takes
- [No new subagents](delegate-to-opus-subagents.md) — NONE new since 2026-10-02 (Buzz: "when each of these current subagents is finished it job, dont create new ones"); the six running then finish and are merged by me
- [Worktree agents share the target dir](worktree-agents-share-the-target-dir.md) — touch a crate's lib.rs before an integration build while agents run
- [Drop-downs escape the page](dropdowns-escape-the-page.md) — deferred+anchored, thirty rows, wheel-scrolled; the page clips anything else
- [Kill by PID, not pattern](kill-by-pid-not-pattern.md) — pkill -f matches the calling shell; pgrep -fa, then kill the PIDs
- [Bench CubeOrange](bench-cubeorange.md) — stock bootloader since 2026-09-25; by-id path, COM4 in the VM; its ADS-B heartbeat (1:0) and the same-firmware question; the host's MR-VMU is arduzeph's, not ours; flash only on Buzz's go
- [Passwords remembered, plain](passwords-remembered-plain.md) — InputBox password answers are kept like any other; plain text boxes are fine for now
- [Never edit a running script](never-edit-a-running-script.md) — bash reads scripts incrementally; a runner edited mid-suite breaks the test running at that moment
- [VM work shows on its console](vm-work-shows-on-its-console.md) — every VM job through tools/win10/vm-run.sh into the visible "Claude at work" window, never bare SSH; Buzz watches (2026-09-26)
- [Windows VM tiny10](windows-vm-tiny10.md) — ssh -p 2222 user@localhost (PowerShell), share /home/buzz/vmshare = S:, host is 10.0.2.2 from the guest; snapshot before installs
