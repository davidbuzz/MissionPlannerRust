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
