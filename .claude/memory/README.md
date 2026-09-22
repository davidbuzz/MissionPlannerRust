# Project memory

Durable notes about how this project is worked on: preferences, constraints and decisions that are
not derivable from the code or the commit history.

They live here, in the repository, so they are version controlled and reviewable like everything
else. Claude Code's per-project memory directory
(`~/.claude/projects/-home-buzz-MissionPlannerRust/memory/`) holds symlinks pointing here, so
recall still works while the content stays under git.

Each file is one fact, with frontmatter naming it and summarising it for recall. `MEMORY.md` is the
index that gets loaded at session start.
