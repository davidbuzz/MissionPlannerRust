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

**This memory existed and was violated anyway**, on 2026-09-23, after the accelerometer
calibration commit. Writing the rule down was not enough, so here is the mechanism that defeats it:

**The closing summary is the trap, and its quality is what makes it dangerous.** A well-made
summary — tables, findings, judgment calls, "Next: ..." — reads like a deliverable, and a
deliverable feels like an ending. The better it reads, the more final it feels. The sentence
beginning "Next:" is the tell: it describes work instead of performing it. Every single violation
has had that shape.

**The rule is mechanical, not intentional.** After `git commit` succeeds:

1. The next tool call is the next task's first step. Not a summary. Not a recap table.
2. Never write a paragraph beginning "Next:", "Next up:", or "Then:". If the next step is known
   well enough to name it, it is known well enough to start it.
3. A turn ends only on: a genuine blocker needing the owner's decision, or an explicit stop.
   Finishing a feature is neither.

**How to apply:** progress notes go *between* tool calls, one or two sentences, then straight back
to work. Report what a run showed and keep moving. If the urge to summarise appears, that is the
signal that a commit just landed and the next task should already be underway. Only stop when a
decision genuinely requires the owner (money, destructive action, a fork in requirements), and then
say exactly what is needed rather than summarising what is done. Long uninterrupted stretches are
the expected mode, not an exception.
