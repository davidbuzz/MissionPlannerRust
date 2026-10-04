# Project memory index

One line per memory. Files live in `.claude/memory/` in this repository so they are version
controlled; the per-project memory directory holds symlinks to them.

**CRITICAL:**
- [NOT IN THE C#, NOT IN SCOPE](not-in-the-csharp-not-in-scope.md) — never invent a feature Mission Planner does not have; ask instead
- [Port from the C# source](port-from-the-csharp-source.md) — https://github.com/ArduPilot/MissionPlanner is the spec, the clone `MP_SRC` names (never its path in the repo); read the .cs, never recall it
- [No callers, no port](no-callers-no-port.md) — a .cs file or function nobody calls is recorded out of scope, never ported (Buzz, 2026-09-25)
- [Match the original layout](match-the-original-layout.md) — Buzz is the oracle on look and feel; default to MP's arrangement, read it from the .resx
- [Mode buttons kept](mode-buttons-kept.md) — the flight screen's own mode buttons panel stays over the C#'s CMB_modes drop-down (Buzz, 2026-09-26)
- [Windows unsafe rulings](windows-unsafe-rulings.md) — unsafe per file for Windows APIs: win32.rs yes, camera capture yes (one file), joystick not now (Buzz, 2026-09-27)
- [Link kept alive, questions once](link-kept-alive-no-dialogs.md) — a connected link is kept and reconnected every second with no dialog (D23); the take-off altitude is asked once a session (D24) (Buzz, 2026-10-03)
- [No dialogs for avoidable errors](no-dialogs-for-avoidable-errors.md) — an error the main window can show as state/connectivity/colour never gets a message box (Buzz, 2026-09-25)
- [Mute on language](mute-on-language.md) — English when `language` is empty; no i18n questions or reports until told otherwise (Buzz, 2026-09-25)
- [Work the matrix in priority order](work-the-matrix-in-priority-order.md) — every "high" row of NOT_DONE_YET_MATRIX.md to 100% before any "med" row (Buzz, 2026-09-26); within med: Standard/Advanced Params, DroneCAN, MAVFtp, Sik Radio first (Buzz, 2026-10-02); a row at 100 moves out the same day, to DELIVERABLES.md's "Finished since the audit" list or PLAN.md §12 (Buzz, 2026-10-03)
- [C# tree reads are measurement](csharp-tree-reads-are-measurement.md) — Rust never opens a path under the clone; tree reads only as measurement of the port, only via MP_SRC, passing without it (Buzz, 2026-10-03)
- [Licensing decisions](licensing-decisions.md) — GPL-3.0-only; the header on every .rs (xtask::licence::HEADER, tested); David "Buzz" Bussenschutt / MissionPlannerRust; no product photos, no CPOL code; `cargo xtask licences` after a dependency change (Buzz, 2026-10-03)

- [Acceptable is not do-it-now](acceptable-is-not-do-it-now.md) — "X is acceptable" is information, not an instruction; never interrupt or destroy on it without asking, never claim agreement he didn't give (2026-10-04)
- [10A is for backups](10a-is-for-backups.md) — do NOT USE /media/buzz/10A for anything, ever, not even a read or a listing; VM disks go on / (nvme1n1p2); an interrupted command may have partly run, check before saying it didn't (Buzz, 2026-10-03)
- [headless-planner is internal](headless-planner-is-internal.md) — not part of the application; releases and packages ship `planner` only (Buzz, 2026-10-04)
- [NEVER save the VM](never-save-the-vm.md) — no savestate/pause/poweroff/live snapshot of tiny10 without Buzz's word: "SAVE = things IMMEDIATELY stop working, do not do" (2026-09-26)
- [One build at a time](one-build-at-a-time.md) — one cargo build at a time under `flock <scratchpad>/build.lock`; with the VM up, debug at 4 jobs and NO release build (a release build beside the VM OOM-killed gnome-shell and the VM on 2026-09-26); with agents on the lock the coordinator goes first through mine.sh (2026-10-02)
- [Scratch target dirs fill the disk](scratch-target-dirs-fill-the-disk.md) — /tmp is the root disk, 96% full without me; target-solo hit 65 GB; prune planner-* incremental caches, old test binaries and finished agents' target dirs; df before big runs (2026-09-26)
- [Autonomous means no prompts](autonomous-means-no-prompts.md) — before Buzz leaves me to work alone, the commands the work needs must not prompt: one prompt halted the night of 2026-10-03/04
- [No foreground waiting](no-foreground-waiting.md) — background long operations, poll cheaply, never block the session
- [GUI runs stay short](gui-runs-stay-short.md) — windows live ~5s, pinned to DP-1-3; never debug by re-running the GUI
- [GUI tests take the mouse](gui-tests-take-the-mouse.md) — on the desktop, scripts drive the real pointer: only on Buzz's word, stop means now; `tools/gui-headless.sh` runs them on Xvfb :99 with lavapipe and touches nothing of his
- [Verify edits applied](verify-edits-applied.md) — a replacement that matches nothing looks like success
- [Never stop after a commit](never-stop-after-a-commit.md) — chain to the next task; a commit is not an exit condition
- [rustfmt follows children](rustfmt-follows-children.md) — rustfmt on main.rs/lib.rs rewrites every module they declare and the tree is not fmt-clean: format leaf files, revert the rest (2026-10-03)
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
- [tridge-mac](tridge-mac.md) — the borrowed Mac for the macOS build: ssh tridge-mac, ~/MissionPlannerRust, the mpr-*.sh scripts, LZMA_API_STATIC=1, no window over SSH (2026-10-03)
- [Windows VM tiny10](windows-vm-tiny10.md) — ssh -p 2222 user@localhost (PowerShell), share /home/buzz/vmshare = S:, host is 10.0.2.2 from the guest; snapshot before installs
