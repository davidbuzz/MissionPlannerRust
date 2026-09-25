---
name: delegate-to-opus-subagents
description: Buzz authorises delegating coding tasks to at most SIX Opus subagents at a time (2026-09-25 afternoon; the order was three, six, none, three, six); they never open windows, never commit, and get disjoint files
metadata:
  type: feedback
---

**"pls delegate coding tasks to at most 3 Opus 5 subagents"** — Buzz, 2026-09-23 14:08 UTC (the transcript's time; the question "are you capable of delegating coding tasks to Opus 5.1 as a sub agent?" came a minute before). Standing
authorisation, with a cap of three concurrent - **raised to six** on 2026-09-23 16:05 UTC (02:05 on the 24th, local; commit b8f5d57): "pls delegate coding tasks to at most 6 Opus 5 subagents". **The stop below came after the raise**, not before it: the order is three → six → none.

**Why:** the queue in PLAN.md §13.2 has independent items, and one context working them in series
is the bottleneck; agents with disjoint files finish that many items in the time of one.

**How to apply:**

- `Agent` tool, `model: "opus"`, `subagent_type: general-purpose`, background; at most six at
  once. Each prompt carries the rules the agents do not otherwise see: not-in-the-C# is not in
  scope, read the `.cs` first, autotests mandatory, workspace lints, comment style.
- **Disjoint file sets, stated explicitly in each prompt.** Concurrent agents on one file is a
  merge nobody asked for.
- **Agents never open a window** — no `mpr-gui`, `gui-test.sh`, `screenshot.sh`; the coordinator
  runs GUI tests, one short run each. See [[gui-runs-stay-short]].
- **Agents never `git add` or `git commit`.** The coordinator integrates, runs clippy and the
  full test suite, reads both, and commits. See [[verify-before-committing]].
- Agents test with `cargo test -p <crate>`, not `--workspace` — the target directory is shared
  and cargo serialises on its lock, which is expected; nobody kills another cargo.
- The coordinator keeps the integration work (wiring an API into the GUI, docs, memory,
  commits) and the verification; the agent's report is data, not a verdict.

**Stopped by Buzz, 2026-09-24 ~03:20 UTC:** "pls dont start any more sub agents, i want to
allow these to finish, but run no more." The four then running (log browser remainder, Install
Firmware Legacy/Ateryx, the Geo Reference form, terrain in the planner) finish and are
integrated; launch none after them until he says otherwise.

**2026-09-25 (~13:00 local): "pls allow at most 3 Opus subagents to work on this."** The stop of
2026-09-24 is lifted and the cap is three, not six. Order now: three → six → none → three. Each
agent works in its own worktree (`isolation: worktree`), on rows with disjoint files, reads the C#
before writing, ships tests and scripts (scripts written, never run), and reports what it changed;
I verify, merge and commit.

**2026-09-25, a correction:** when Buzz stops one agent mid-run (as he did the first row 70 agent
at 13:40), that is not a ban on subagents. I read the harness's "only launch a new agent if the
user explicitly asks" as one and did the next five rows by hand; Buzz asked "why aren't we using
subagents?" Keep three running whenever there is disjoint work; a stopped agent means review its
worktree and start a fresh one on what is left.

**2026-09-25, later:** Buzz raised the cap to **six** open subagents at once ("allow the use of 6 open subagents"). Keep six running whenever there is disjoint work.
