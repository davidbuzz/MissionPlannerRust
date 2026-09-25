---
name: gui-tests-take-the-mouse
description: GUI test scripts drive the real pointer and keyboard, so they interfere with Buzz's own mouse while they run; run them only when he says so, and prefer headless checks
metadata:
  type: feedback
---

**"GUI tests interfere with the engineer's mouse movement while they are in-flight."** Buzz,
2026-09-24, after asking for a running suite to be stopped.

`tools/gui-test.sh` and `tools/gui-suite.sh` drive the application with `xdotool`: every `click`,
`hover`, `scroll`, `doubleclick` and `type` moves the one pointer and types into the one keyboard
focus the desktop has. A window pinned to DP-1-3 is out of his sight, but the pointer is not his
while a script runs - it jumps to the test window mid-motion, and his own click can land in the
test or the test's click in his work. A 27-script suite is forty minutes of that.

**Why it matters beyond annoyance:** a stray click of his lands in a script's window and the
script fails for a reason no log explains; a stray click of the script's lands in his editor.
And the reverse of [[gui-runs-stay-short]]'s lesson holds: a suite that is running when he sits
down is a suite he will kill, and its results are lost either way.

**How to apply:**

- **Do not start a GUI script or suite while Buzz is at the machine unless he has just said to**
  (a "go" that names the scripts). An earlier "go" does not carry to a later batch; a suite he
  asked for and then stopped is stopped.
- **Prefer the headless route first**: `headless-planner` verbs, `cargo test -p <crate>` in a second target
  directory, the facts and logs a previous run left. A GUI script is the last check, not the
  first probe.
- **When a suite is authorised, say how long it will take and that the pointer is not his
  until it ends**, so he can choose the moment; offer to run it when he steps away.
- **Stop means now**: kill by PID, wrapper first so nothing else spawns ([[kill-by-pid-not-pattern]]),
  confirm no `planner` or `xdotool` remains, and report what ran and what did not.
- A one-script diagnostic run is still a run of the pointer: ask, or wait for the go.

See [[gui-runs-stay-short]] for the window's lifetime and the quiet-machine rule, and
[[no-foreground-waiting]] for how a suite runs in the background once it is allowed.

**2026-09-25, Buzz, twice in one hour: "stop UI testing" - "its too slow, steals the mouse cursor,
and is annoying me. i'll let you run it when i leave."** A suite of 29 scripts is 20 to 30 minutes
of a desk that cannot be used. The go for GUI runs is not a standing permission: it holds for the
moment it is given, and a suite runs only when Buzz says he has left the machine. Stop means now:
kill the runner, the driver and the window by PID (pgrep with a bracketed pattern, e.g.
`gui-suite.s[h]`, so the pattern does not match the killing shell), and confirm with
`xdotool search --name`.

**2026-09-25, later:** Buzz re-enabled UI testing ("please re-enable the UI testing that was suspended earlier"). The go is again a standing one until he says stop; a suite runs on DP-1-3 with SITL from `tools/sitl/start-sitl.sh`, and the machine is kept free of builds while it runs.

**SITL for a suite, 2026-09-25 evening:** start it as its own background Bash task
(`tools/sitl/start-sitl.sh`, `run_in_background`), never as `bash -c "start-sitl.sh && suite"`:
ArduPilot's SITL exits when its parent dies (`SITL_State.cpp` checks `getppid()` every loop), so
the wrapper now waits on it and must stay alive; two suite runs failed every script with
"Connection refused" before this was found. The suite then runs in a separate background task.

**2026-09-26, a way out:** `tools/gui-headless.sh` runs the scripts on Xvfb `:99` with lavapipe
(`VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/lvp_icd.json`, `SHOT_AT=0,0`): xdotool drives that display,
the real pointer and Buzz's windows are never touched, and the Quick page's three scripts passed there
while the desktop was covered by the session-failed screen. Slower than the GPU (budgets recorded from
the slower run); the storm gate still needs the GPU. Prefer it for every run Buzz has not asked to
watch. See [[one-build-at-a-time]] for the screen's cause.
