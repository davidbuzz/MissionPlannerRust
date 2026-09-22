---
name: verify-edits-applied
description: A string replacement that matches nothing looks exactly like success; always verify
metadata:
  type: feedback
---

When editing files with a scripted string replacement, **assert that the replacement matched**.
A `str.replace` that finds nothing returns the original string and exits zero, so the edit looks
like it worked, the build still passes, and the change is simply absent.

**Why:** this has happened three times in this project, always the same way — `cargo fmt` had
reformatted the target between writing the pattern and running it, so the exact text no longer
existed. Once it left the map's stats line describing a synthetic scene while claiming to describe
a real flight, which is a worse failure than a compile error because it produces confident wrong
output.

**How to apply:** prefer the **Edit tool**, which fails loudly when `old_string` is absent. When
scripting, assert every replacement changed something (`assert s2 != s`) and assert the *count* when
making several, because a loop that `break`s early silently skips the rest — that variant cost a
commit too. Be especially careful immediately after `cargo fmt`, which is when the text most often
stops matching.

**And verify before committing, not after.** Three commits went out with lints because the check ran
in the same command chain as the commit, so a failed edit still reached git. Run clippy and the
tests, look at the numbers, *then* commit.
