---
name: delegate-to-opus-subagents
description: NONE new since 2026-10-02 ~21:50 local - Buzz: "when each of these current subagents is finished it job, dont create new ones"; the six running then finish and are merged (the order was three, six, none, three, six, none, two, none, six, none)
metadata:
  type: feedback
---

**STOP (current), 2026-10-02 ~21:50 local: "when each of these current subagents is finished it
job, dont create new ones."** - Buzz. The six running then (DroneCAN/UAVCAN page, MAVFtp
remainder, Sik Radio, Warning Manager, Spectrogram + Support Proxy, Proximity + MAVLink Signing)
finish, their worktrees are reviewed, applied to main as patches, verified and committed by the
coordinator; after them, no Agent tool for coding, reviews or searches until he says otherwise.
Order so far: three → six → none → three → six → none → two → none → six → none (this).

**SIX (2026-10-02 morning, stopped the same day): "pls allow use of up-to 6 Opus subagents for writing the code"** - Buzz,
2026-10-02. Lifts the stop of 2026-09-27. Agents write code; the coordinator still
integrates, verifies and commits. All the "How to apply" rules below hold. With six at once the
target directory is shared (`<scratchpad>/cargo-agent.sh`: the build lock, 3 jobs, the memory
cgroup) - six private target directories would fill the disk (26 GB each on 2026-09-27).

**STOP (2026-09-27, lifted 2026-10-02): "pls stop using subagents. when the current agents are done working, use their
work but dont make more."** - Buzz, 2026-09-27. No Agent tool for coding, reviews or searches;
the two agents running then (Legacy Force Bootloader, MAVLink Inspector) were let finish and their
work reviewed and merged. Work items go through this session alone. Ask before starting one again.

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
- **Agents never open a window** — no `planner`, `gui-test.sh`, `screenshot.sh`; the coordinator
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

**Merging an agent's worktree (learned 2026-09-25, seven merges in one afternoon):**
`git -C <worktree> diff --binary -- . ':!Cargo.lock' > patch` plus the `??` files copied by hand;
`git apply --3way` needs a CLEAN INDEX (`git add -A` anything of mine first, or it says "does not
match index" and applies nothing); never take an agent's `Cargo.lock` - drop it and let cargo
re-resolve offline; two agents adding the same dependency leave a duplicate key in `Cargo.toml`
(check `grep -n "^serde_json" crates/mp-gui/Cargo.toml` before building); after a rename sweep,
sweep the new files too and grep for `CARGO_BIN_EXE_<old>` (an underscore before the name defeats
`\b`); one workspace test run per merge, in the background, then commit, then remove the
worktree and its branch (`git worktree remove --force`, `git branch -D`). Six agents on one
`target-agents` dir make every GUI rebuild minutes long and they overwrite each other's builds -
they touch `lib.rs` to recover; a per-agent target dir would cost disk but save the thrash.

**2026-09-25 17:35, Buzz: "pls dont make any new subagents, and as the current ones finish their
tasks, dont make more."** The cap is now zero for new launches until he says otherwise; the six
running at that moment finish and are merged, and then the work is mine alone. Order so far:
three → six → none → three → six → none (this).

**2026-09-26 (~00:20 local), Buzz: "pls use as many as 2 opus sub agents moving forward."** The
stop of 2026-09-25 17:35 is lifted and the cap is **two**. Order so far: three → six → none →
three → six → none → two (this). Same rules: worktree each, disjoint files, the C# read first,
tests shipped, no window, no commit; a per-agent target directory
(`CARGO_TARGET_DIR=<scratchpad>/target-agentN`) so two builds do not thrash one another. A
read-only reviewer of a pending diff counts as one of the two.
