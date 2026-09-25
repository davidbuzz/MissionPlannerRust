---
name: passwords-remembered-plain
description: Buzz (2026-09-25) wants InputBox password answers remembered like any other, and does not mind them showing in plain text boxes for now
metadata:
  type: feedback
---

Buzz, 2026-09-25, answering the remembered-answers agent: "i want to remember passwords, and at
least initially, i dont care if they are *visible* in regular text boxes."

**Why:** the C#'s `InputBox` skips saving an answer when its password flag is set; Buzz would
rather have the convenience of a remembered password than the secrecy, while the app is his
bench tool.

**How to apply:** `remember_answer` (`crates/mp-gui/src/config/optional.rs`) keeps every OK
answer under its `InputBox<title><prompt>` key, password boxes included - a written divergence
recorded in PLAN §12 D18. A password box may be a plain text box; do not add masking or skip the
save unless Buzz asks. Related: [[no-dialogs-for-avoidable-errors]].
