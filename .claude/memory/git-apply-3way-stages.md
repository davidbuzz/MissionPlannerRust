---
name: git-apply-3way-stages
description: "`git apply --3way` stages what it applies, so the next commit of other paths sweeps an agent's patch in: apply without --3way, or `git reset -q` the index before staging a commit by paths"
metadata:
  type: feedback
---

`git apply --3way <agent patch>` writes the files AND stages them. On 2026-10-16 the crawler's
display fix was committed with `git add tools/...` while a MAVSerial patch sat staged from a
`--3way` apply, and the commit carried 1,300 lines of someone else's work under the wrong message;
the next two commits were likewise mixed, and all three had to be rebuilt (`git reset --soft
HEAD~3`, `git reset`, then staged by paths).

**Why:** a commit takes the whole index, not the paths named in the last `git add`.

**How to apply:** apply agents' patches with plain `git apply` (unstaged), or `git reset -q` right
after a `--3way` apply; before every commit, read `git diff --cached --stat` and check it lists
only that item's files; with several items pending, commit each by staging its paths (and the
ledger's hunks by row, `stage-hunks.py`) from a clean index. Related: [[verify-before-committing]],
[[delegate-to-opus-subagents]].
