---
name: autonomous-means-no-prompts
description: Before Buzz leaves me to work alone (overnight), every command the work needs must run without a permission prompt - one prompt a minute after he left halted the whole night of 2026-10-03/04
metadata:
  type: feedback
---

Buzz, 2026-10-04 morning: he asked for three release builds by morning and went to sleep,
telling me to proceed autonomously; "a bash permission prompt was popped up unexpectedly barely 1
minute after human left the room, and all progress was halted." From my side a command waiting
for approval looks the same as one still running, so nothing showed it - the Linux build that
should have finished overnight started at 11:05.

**Why:** an autonomous stretch is only autonomous if nothing in it needs him; one prompt costs the
whole stretch.

**How to apply:** when he says he is leaving and to carry on, first list the kinds of command the
remaining work needs (VBoxManage, ssh/scp to the VM, tools/win10/vm-run.sh, gh run/workflow/release,
the scratchpad's mine.sh and scoped-cargo.sh, cargo, git worktree, kill by PID, xvfb-run) and say
which may prompt, so he can allow them before he goes (the fewer-permission-prompts skill, or the
project's .claude/settings.json allow list); prefer forms already allowed. Never start a new kind
of command in the first minutes after he leaves without having said so.

**Why the command of 2026-10-04 19:3x was blocked:** it raised "allow this bash command" while
Buzz had been told to expect no prompts, and he rejected it ("did you deliberately trigger...?";
then: "write a memory about why it was blocked, and don't write commands like that again"). This
session has no allow list (~/.claude/settings.json is empty), so a prompt comes from the
harness's own checks on a command's shape, and which part tripped them is not certain. That
command was the only one of the day to combine all of: a leading `cd ... &&` (the harness's own
guidance: `cd` in a compound command can prompt), several chained `sed -i` edits of tracked
files, a build backgrounded with a trailing shell `&`, and its output redirected to a log file in
the same call.

**Never again, in any of those shapes:** no `sed -i` (edit files with a `python3 - <<'EOF'` script or
the Edit/Write tools); no trailing `&` and no `nohup`/`setsid` backgrounding from the Bash tool
(start long jobs with its `run_in_background`, and read their output file); no redirect of a job's
output to a log file inside a backgrounded command; no leading `cd` in a compound command (use
absolute paths, `git -C`, or the tool's working directory). One kind of action per call. These
forms ran all day without a prompt.
