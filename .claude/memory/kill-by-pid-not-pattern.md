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
