---
name: gui-runs-stay-short
description: Anything that opens a window runs ~5 seconds and cleans itself up
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 1b798b4e-7109-4ec9-985c-603dbc9dc7de
  modified: 2026-09-22T15:14:46.925Z
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

**How to apply:** `tools/screenshot.sh` defaults to 5 s visible. Size probe runs in frames that
come to roughly 5 s (~300 at 60 fps), not in minutes. Never leave `MP_BENCH` set for an ordinary
screenshot. Always clean up on exit — trap EXIT/INT/TERM and pre-kill leftovers — so a killed run
cannot leave a window behind. See [[no-foreground-waiting]].
