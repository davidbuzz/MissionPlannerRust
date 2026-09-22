---
name: never-stop-after-a-commit
description: A commit is not an exit condition; chain to the next task in the same turn
metadata:
  type: feedback
---

**Do not end a turn because a commit landed.** Commit, then immediately start the next piece of
work in the same turn. Keep going until genuinely blocked or told to stop.

**Why:** Buzz said "continue, stopping here is an ERROR" five separate times, and each time the
same thing had happened — work finished, commit made, a summary written, turn ended. The summary
was the stopping mechanism: treating it as the turn's deliverable turns a progress note into a
handoff. Nothing was actually blocking; broad autonomy had already been granted. He called it
catastrophic because it converts an autonomous build into one that stalls every few minutes
waiting for a human to say "continue".

**How to apply:** after `git commit`, the very next action is the next task's first tool call, not
prose. Write progress notes *between* tool calls, briefly, and keep working. Only stop when a
decision genuinely requires the owner (money, destructive action, a fork in requirements), and then
say exactly what is needed rather than summarising what is done. A long stretch of uninterrupted
work is the expected mode here, not an exception.
