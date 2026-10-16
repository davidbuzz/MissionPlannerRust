---
name: restarts-kill-background-work
description: A session restart (context compaction, a model switch, the process exiting) kills every background shell and agent; agents resume from their transcripts by SendMessage, shells must be restarted, and both must leave their work on disk as they go
metadata:
  type: feedback
---

When the Claude Code process restarts - after compaction, a model switch, a crash - every
background shell (builds, test chains, crawls) and every subagent is stopped with no completion
record; the notification arrives in the next session as "didn't finish before the previous session
ended". On 2026-10-16 this happened twice in one morning: a build chain, a five-hour crawl, four
ledger-audit agents and two code agents died each time, and the agents had written nothing yet
(one had an empty JSON list), so their hours were lost.

**Why:** the agents kept their results for the end; the shells' lock helper left its
`coordinator-wants-lock` flag behind (its EXIT trap never ran), and a git `index.lock` was left in
a scratch clone.

**How to apply:**
- Agents: `SendMessage` to the agent's id resumes it with its transcript ("the session restarted;
  continue the brief at <path> from where you left off") - no new spawn, no lost context. Every
  brief tells the agent to write its output file incrementally (rewrite after every ten rows) and
  code agents to edit as they go, so a stop costs minutes.
- Shells: check the task's output file for partial results, then start the chain again; remove the
  stale `coordinator-wants-lock` flag and any `.git/index.lock` first (after `pgrep -fa git` shows
  no git running there).
- Before a long chain, keep the chain's script small and its results in files, never only in the
  tool output. Related: [[no-foreground-waiting]], [[one-build-at-a-time]],
  [[delegate-to-opus-subagents]].
