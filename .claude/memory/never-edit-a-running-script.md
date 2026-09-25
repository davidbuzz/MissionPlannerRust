---
name: never-edit-a-running-script
description: Editing tools/gui-test.sh (or any bash script) while a suite runs corrupts the instance running it - bash reads the file incrementally
metadata:
  type: feedback
---

Do not edit `tools/gui-test.sh`, `tools/gui-suite.sh` or any bash script while a run of it is
in progress. Bash reads a script incrementally, by file offset, so an instance already running
picks up the edited bytes mid-way and fails with a syntax error at a line that is fine on disk.

**Why:** on 2026-09-25 a trap fix to gui-test.sh during suite-25m broke the one test running at
that moment (plan-modify-alt: "syntax error near unexpected token" at line 759) after every
expectation in it had passed; the tests started after the edit ran the new file cleanly.

**How to apply:** queue runner edits until the suite's end line, or copy the runner to the
scratchpad and edit the copy, then move it into place between runs. A syntax error reported from
a runner line that reads correctly on disk is this, not a bug in the script. See
[[gui-runs-stay-short]] and [[verify-edits-applied]].
