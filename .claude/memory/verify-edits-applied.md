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

**How to apply:** after a replace, assert the string changed (`assert s2 != s`), or replace by line
range located at runtime rather than by literal text. Prefer the Edit tool, which fails loudly when
`old_string` is absent. Be especially careful immediately after running `cargo fmt`.
