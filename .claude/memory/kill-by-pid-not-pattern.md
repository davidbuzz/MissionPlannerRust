---
name: kill-by-pid-not-pattern
description: pkill -f with a pattern kills the shell that typed it too; list with pgrep -fa, then kill the PIDs
metadata:
  type: feedback
---

`pkill -f "<pattern>"` matches every command line containing the pattern, and the Bash tool's
own command line contains it - so the shell that ran the pkill dies with exit 144 before any
later step runs. It happened three times on 2026-09-24: a soak restart, and twice while stopping
a stale GUI verification, once killing the replacement job as well because its log name shared
the pattern (`storm-verif[y]` matched `storm-verify2`).

**Why:** the harness runs each command through `bash -c "<the whole text>"`, so the text is
itself a process's command line.

**How to apply:** `pgrep -fa <pattern>` first, read the list, then `kill <pid> <pid>`. If a
pattern must be used, bracket a character (`frame_pars[e]`) *and* make sure no later command in
the same shell, and no other job's name, contains the literal. See [[no-foreground-waiting]].

**A worktree's lock names this session's own pid.** `git worktree remove` on a subagent's tree says
"locked, lock reason: claude agent <id> (pid N)"; N is the Claude Code session process, which runs
its subagents in-process, and every Bash tool shell (this one, the background chains, the soak
watchers) is its child. Killing "the agent process and its children" by that pid killed this
session's own background work on 2026-09-24 (a release build and a suite, exit 144) - the session
itself ignored the signal. Remove a locked agent worktree with
`git worktree remove --force --force <path>` and `git branch -D <branch>`; never signal the pid the
lock names.

**2026-09-25:** three of my kill loops died with exit 144 because `pgrep -f "<pattern>"` inside
`$( )` matched the shell running the loop - its own command line carries the pattern - and the
loop killed itself before reaching its targets; one such loop also killed the SITL I was
diagnosing, which then looked like a SITL crash. Write the pattern so it cannot match its own
text: a bracketed last letter, `pgrep -f "sitl/arducopte[r]"`, matches the process and not the
shell whose command line contains `arducopte[r]`.

**The bracket trick has a limit (2026-09-25):** `pgrep -f 'sleep 30[0]'` still matched the calling
shell when the same tool call had *started* `sleep 300` — the literal text was in that shell's
command line too (exit 144 again). When start and kill share a command, keep the pid from `$!`
and kill that; the pattern is only safe against the shell's own text when the target's literal
command line is not written anywhere in the command that runs the pgrep.

**Again 2026-09-26:** `pgrep -f "tools/sitl/start-sitl.s[h]"` in a kill loop matched the same
command's *later* text (`setsid tools/sitl/start-sitl.sh` to start the next SITL) - the bracket
does not help when the literal appears elsewhere in the same command. Now the SITL is started and
stopped through scratchpad helpers that record the PIDs: `sitl-up.sh` (start-sitl.sh detached,
waits for "sitl ready", writes `sitl.pids`) and `sitl-down.sh` (kills exactly those).


**Wait loops too (2026-10-02):** `while pgrep -f "diag2.sh"; do sleep 3; done` inside a helper
script never ended, because the script was launched from a Bash tool call whose text *wrote* the
helper with a heredoc - so the wrapper shell's command line carried `diag2.sh` for as long as the
helper ran. A loop that waits on a pattern is the same trap as a kill by one. Wait on a PID
(`while kill -0 $pid`), a flag file the job touches when done, or a line in its output file;
and write helper scripts in a call of their own, not the call that runs them.

**2026-10-03, again:** `for p in $(pgrep -f "try-cpu.sh host"); do kill $p; done` killed my own
shell (exit 144), because that very command line contains "try-cpu.sh host". Taking PIDs from
`pgrep -f` is no protection when the pattern is in the calling command: bracket it
(`pgrep -f "try-cpu.sh hos[t]"`), or list with `pgrep -fa`, read the PIDs, and kill them in a
second call by number.

**2026-10-05, again.** Buzz asked for tiny10 to be stopped with escalating force. My background
script found the VM's process with `pgrep -f "VirtualBoxVM.*tiny10"`. The script's own command
line had that text in it, so the PID it found was the script itself. After `VBoxManage controlvm
tiny10 poweroff` had already turned the VM off, the final SIGKILL step killed my own script. No
harm came of it, by luck.
- Find a VM's process from its own record: `VBoxManage showvminfo tiny10 --machinereadable`
  gives the session's PID.
- Or use `pgrep -x VirtualBoxVM` and check each PID's `/proc/PID/cmdline`.
- Never pass `pgrep -f` a pattern the calling script's own text contains.
