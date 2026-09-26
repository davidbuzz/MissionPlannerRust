---
name: vm-work-shows-on-its-console
description: Buzz watches the Windows VM's work on its desktop - every job goes through tools/win10/vm-run.sh into the "Claude at work" PowerShell window, never as a bare SSH command (2026-09-26)
metadata:
  type: feedback
---

Buzz asked (2026-09-26 11:20): "can u make the interactions on the win vm so that they are
visible on its console, ie so i can watch the work progressing?" - and, seeing the window,
"great".

**How:** `tools/win10/vm-run.sh <label> [job.ps1 | stdin]` copies the job to `C:\setup\queue`
and the VM's console window (`console-runner.ps1`, the ClaudeConsole task: at logon, in his
session, elevated, no time limit) shows the job's text and then its output as it runs; the
script streams the output back and exits with the job's code (125 = the window is not running -
nobody logged on). A job ends `exit $LASTEXITCODE` when its last native command's code matters.
Installed by `install-console.ps1` over SSH; after editing `console-runner.ps1`, copy it to
`C:\setup` (with a UTF-8 BOM - PS 5.1 reads BOM-less scripts as ANSI) and restart the window
(`Stop-Process` its `runner.pid`, `Start-ScheduledTask ClaudeConsole`).

Each job runs in a PowerShell of its own (`powershell -File`) inside that window, so nothing a
job sets (an environment variable, an `Add-Type`) leaks into the next. The VM's screen is
1600x1200 since 2026-09-26 12:15 (`VBoxManage controlvm tiny10 setvideomodehint 1600 1200 32`,
Buzz's yes): at 1077x774 the planner's pages ran below the window and clicks hit the taskbar.
`tools/win10/sitl-launch.ps1` is the Windows GUI job pattern - planner started with MP_FACTS and
MP_PROBE, `-NoNewWindow`, window at (0,0) and made topmost before each user32 click.

GUI scripts run in the VM through `tools/win10/gui-suite.ps1` / `gui-test.ps1` (the .gui language
on user32; the VM's own Cygwin SITL for SITL scripts). **Budgets are doubled on Windows** (Buzz,
2026-09-26: "double the timeout budget on windows") - `-BudgetScale 2` is the runner's default.

**Why:** SSH sessions run in session 0 where nothing is visible; he wants to see what I am doing
to his VM as it happens.

**How to apply:** builds, runs, installs, git in the VM - all through `vm-run.sh`. Bare SSH only
for the console's own plumbing (copying its scripts, restarting it) or when it is down. Buzz
also keeps his own admin PowerShell box on the VM for commands I hand him to paste (the ones SSH
cannot do); the console window is separate from it. See [[windows-vm-tiny10]].

**A job that never ends (2026-09-26):** the console waits for EOF on a job's output, and every
process started down the chain inherits that pipe (Start-Process with redirection passes all
inheritable handles). The planner's SIMULATION page started a SITL during `sitl-launch.gui` that
outlived the script, so the whole-suite job wrote its results and never "finished" - later jobs
queued behind it. Find the orphan (`Get-CimInstance Win32_Process`, its ParentProcessId and
command line) and stop it by PID; `gui-test.ps1` now ends anything started from a script's
scratch directory. A job's output reaches the laptop only at its end when piped through `tail`.

