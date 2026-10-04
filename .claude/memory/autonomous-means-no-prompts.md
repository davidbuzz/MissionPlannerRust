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
