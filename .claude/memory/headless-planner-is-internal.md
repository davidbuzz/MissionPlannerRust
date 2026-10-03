---
name: headless-planner-is-internal
description: headless-planner is NOT part of the application - an internal testing tool only; releases, packages and installers ship `planner` alone (Buzz, 2026-10-04)
metadata:
  type: project
---

Buzz, 2026-10-04: "i consider it NOT part of the application, its only for internal testing."
`headless-planner` (crates/mp-cli) is the command-line twin used by tests, scripts and CI; no
release archive, `.deb`, installer or release note includes it, and no release step builds it.

**Why:** the product is the planner; a second binary in a release is something users would take
for a supported tool.

**How to apply:** `tools/package.sh`, `.github/workflows/release.yml`,
`tools/win10/release-build.ps1` and the macOS packaging build and ship `planner` only; the
tests, the GUI suite and CI's own jobs keep using headless-planner as before. Related:
[[tridge-mac]].
