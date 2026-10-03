---
name: mute-on-language
description: Buzz (2026-09-25) does not care about translations - the culture is English when config.xml's `language` is empty, and I keep quiet about i18n/language matters until told otherwise
metadata:
  type: feedback
---

Buzz, 2026-09-25, answering the row 76 agent's questions: "With language absent, for now, yes
assume english. i dont care about translations, keep mute about language stuff till told
otherwise."

**Why:** i18n is a P1 deliverable on paper (Deliverable 17) but not what Buzz is testing now; questions about
cultures, fallbacks and translation counts are noise to him.

**How to apply:** when config.xml's `language` is empty the culture is English, not the system's
(a written divergence from `L10N.cs:19-25` in `crates/mp-gui/src/i18n.rs`). Do not raise
translation questions, do not list per-culture counts in reports, do not propose language work;
keep the Fluent plumbing working (its tests and `codegen-resx --check` stay green) and say nothing
about it unless asked. Related: [[not-in-the-csharp-not-in-scope]].
