---
name: gui-runs-stay-short
description: Anything that opens a window runs ~5 seconds and cleans itself up
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 1b798b4e-7109-4ec9-985c-603dbc9dc7de
  modified: 2026-09-23T00:00:00.000Z
---

Anything that puts a window on Buzz's desktop — screenshots, GPU probes, manual GUI checks — runs
for about **5 seconds** and then exits. He has killed two of mine: one repainting at 1 ms under
`MP_BENCH` (which flickers and pegs a core), and one benchmark that ran for minutes.

**Why:** these run on his actual desktop, not a headless CI box. A window that lingers, flickers or
steals focus is an interruption, and a long benchmark is indistinguishable from a hang.

**Background agents must never open windows.** A workflow scoping the GPU tile path had its
investigation agent run a windowed probe over and over while Buzz was working — five windows in a
row, none of them launched in the foreground, so killing them did not stop the next. Any task
delegated to a subagent or workflow is told explicitly: no GUI, no windowed probes, read the source
and reason instead. If a measurement genuinely needs a window, take it once, in the foreground,
where it can be seen and stopped.

**One 5-second run is normally enough.** Repeated runs to refine a number are not worth the
interruption; take the measurement once and record it.

**Never iterate on a GUI problem by re-running the GUI.** On 2026-09-23 a screenshot came out
wrong and the debugging was: run, look, adjust, run again — about eight windows in a few minutes.
Buzz interrupted with "the app is popping" and "its all over the place". The failure was treating
each run as cheap because each one was short; it is the *count* that interrupts, not the duration.
When a GUI run produces the wrong result, stop running it. Diagnose from the source, the probe
file and the one capture already taken, make every fix at once, and spend the next run confirming
rather than exploring. If that is genuinely impossible, say so and ask before opening more windows.

**Pin the window to one monitor, and put the mouse there first.** Buzz has three (eDP-1
2560x1600 +1244+1440, DP-1-1 2560x1440 +2560+0, DP-1-3 2560x1440 +0+0) in one 5120x3040 X screen.
He chose **DP-1-3, at +0+0**; `tools/screenshot.sh` defaults to it and `SHOT_AT=X,Y` overrides.

**The window follows the mouse pointer, and the scripts move the pointer.** Buzz worked this out —
"is app following mouse curson?" — after the window had appeared on a different screen three runs
running. mutter places a new window on the monitor containing the pointer (`center-new-windows` is
`false`), and `xdotool mousemove` moves the *real* cursor because there is no other kind. So each
run dragged his cursor to wherever it last clicked, and the next run's window was created there.
Two fixes, both in `screenshot.sh`: move the pointer to the target corner **before launching the
binary**, so the window is created in the right place rather than created elsewhere and dragged;
and record the pointer's position up front and restore it in the `trap`, so a run does not leave
somebody's cursor on another screen. `windowmove` afterwards is the backstop, not the mechanism.

**Never target a window this script did not launch.** `tools/screenshot.sh` had a fallback that
matched `--name "Mission Planner"` by title alone, for a window manager that does not set
`_NET_WM_PID`. Buzz runs the **real Mission Planner**, which has exactly that title. The fallback
found it, and the script screenshotted it, clicked at (378,132) inside it and typed "flight.bin"
into it — while it was on SETUP > Install Firmware with a flight controller plugged in. Synthetic
input into somebody else's ground station is not a screenshot bug; it is a way to flash a board by
accident. The window must be owned by the pid the script started, verified, and a run that cannot
find one **fails** rather than reaching for whatever else answers to the name.

Two related traps in the same file, both of which produced a screenshot of the wrong thing while
reporting success: the capture is an `x11grab` of the *screen* at the window's coordinates, so the
window must be fully on one monitor and on top — check it, do not assume it; and
`xdotool search --pid` returns every window a client owns including unmapped transients, so it
needs `--onlyvisible --name` or it hands back an id that is gone a moment later.

**Do not run `crates/mp-gui/tests/layout.rs` as part of routine verification.** It is `#[ignore]`d
because it opens four windows in sequence. Buzz interrupted a run of it that was tacked onto an
ordinary `cargo test --workspace` after a commit that had not touched layout. Plain
`cargo test --workspace` skips it, which is the point of the ignore; run it deliberately, and only
when panel geometry actually changed.

**How to apply:** `tools/screenshot.sh` defaults to 5 s visible. Size probe runs in frames that
come to roughly 5 s (~300 at 60 fps), not in minutes. Never leave `MP_BENCH` set for an ordinary
screenshot. Always clean up on exit — trap EXIT/INT/TERM and pre-kill leftovers — so a killed run
cannot leave a window behind. See [[no-foreground-waiting]].
