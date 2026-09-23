---
name: autotests-mandatory
description: Every change ships with an automated test that fails if the behaviour stops working; "verified by hand" is not verification
metadata:
  type: feedback
---

**"autotests mandatory"** — Buzz, 2026-09-24, while the `/goal` prompt was being drafted. Two
words, and they apply to every change, not to deliverables in aggregate.

**Why:** the expensive bugs in this project have all sat in the gap between "the code exists" and
"the code has been run against the thing it is for", and several had tests that passed — because
the tests exercised a different path from the one the product takes. The offline tile store had
no worker thread and served nothing from disk, while every offline test pre-loaded the tile by
hand. Flight recordings went to `~/Documents/Mission Planner/logs`, a directory the real Mission
Planner on Linux never reads, and nothing asked where the real one looks. A test that proves the
convenient path proves nothing about the product.

**How to apply:**

- No change is finished, and nothing is committed, without a test that runs under
  `cargo test --workspace` or `tools/gui-test.sh` and fails if the behaviour stops working.
- Unit tests for the logic; an integration test against real files where a file format is
  involved; a `.gui` test through `tools/gui-test.sh` where a user would click; a differential
  test where the C# is a usable oracle (headless mono, files it wrote, this machine's real
  Mission Planner data under `~/.local/share/Mission Planner`).
- Test the path the product takes, not the shortcut that is easy to call from a test. If a
  fixture is only sometimes available (a real cache, a real device), the test skips with a
  printed reason — it never passes vacuously.
- DELIVERABLES.md's test policy ("if it is not a program that fails, it is not a test") is the
  per-deliverable form of this; this memory is the per-commit form. See
  [[verify-before-committing]] for running and reading the result before the commit.
