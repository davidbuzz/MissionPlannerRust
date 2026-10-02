---
name: rustfmt-follows-children
description: rustfmt on main.rs or lib.rs reformats every module they declare; the tree is not fmt-clean, so format leaf files only or revert the children
metadata:
  type: project
---

`rustfmt file.rs` formats that file **and every out-of-line module it declares** (`mod foo;`),
recursively. Running it on `crates/mp-gui/src/main.rs` or `crates/mp-link/src/lib.rs` rewrites
dozens of files, and this tree is not rustfmt-clean (other sessions committed unformatted code),
so the working tree fills with formatting-only changes that are not the item's.

**Why:** on 2026-10-03 a rustfmt of main.rs and lib.rs touched 22 unrelated files (plan.rs and
scripts_tab.rs alone 440 lines) that had to be reverted and the item's edits re-applied by hand.

**How to apply:** run rustfmt only on files that declare no modules (a new module, a test file);
for main.rs/lib.rs, copy the file aside, run rustfmt, keep the result for the file itself and
`git checkout --` every other file it changed - or hand-format the hunk in the shape of its
neighbours. `rustfmt --check` on a parent file fails for the children's sake, not yours. See
[[verify-before-committing]] and [[verify-edits-applied]].
