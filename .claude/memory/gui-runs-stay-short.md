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

**Pin the window to one monitor.** Buzz has three (eDP-1 2560x1600 +1244+1440, DP-1-1 2560x1440
+2560+0, DP-1-3 2560x1440 +0+0) in one 5120x3040 X screen. Left to the window manager the window
landed somewhere different almost every run, on top of whatever he was doing. He chose **DP-1-3,
at +0+0**, which `tools/screenshot.sh` now defaults to and `SHOT_AT=X,Y` overrides. Note the
window manager pulls a window to the active monitor on `windowactivate`, so activate first and
move afterwards, then verify the position and move again — a single `windowmove` does not stick.

**Do not run `crates/mp-gui/tests/layout.rs` as part of routine verification.** It is `#[ignore]`d
because it opens four windows in sequence. Buzz interrupted a run of it that was tacked onto an
ordinary `cargo test --workspace` after a commit that had not touched layout. Plain
`cargo test --workspace` skips it, which is the point of the ignore; run it deliberately, and only
when panel geometry actually changed.

**How to apply:** `tools/screenshot.sh` defaults to 5 s visible. Size probe runs in frames that
come to roughly 5 s (~300 at 60 fps), not in minutes. Never leave `MP_BENCH` set for an ordinary
screenshot. Always clean up on exit — trap EXIT/INT/TERM and pre-kill leftovers — so a killed run
cannot leave a window behind. See [[no-foreground-waiting]].
