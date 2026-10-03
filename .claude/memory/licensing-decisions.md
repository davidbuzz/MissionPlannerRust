---
name: licensing-decisions
description: Buzz's licensing decisions of 2026-10-03 before the repository goes public - GPL-3.0-only, the owner's header on every .rs (xtask::licence::HEADER, enforced by xtask/tests/licences.rs), copyright name David "Buzz" Bussenschutt, product name MissionPlannerRust, the product photographs out, AGauge rewritten from behaviour, THIRD_PARTY_LICENSES kept by `cargo xtask licences`
metadata:
  type: project
---

Buzz's rulings of 2026-10-03, when the repository was being readied to go public:

1. **GPL-3.0-only.** The port is a derivative of Mission Planner, whose COPYING.txt is the GPL
   version 3 with no later-version grant, so the work cannot offer one either. Never write
   "GPL-3.0-or-later" of this work or of Mission Planner (ArduPilot and Zed's application crates
   are or-later; that is theirs).
2. **The header.** Every tracked `.rs` file opens with `xtask::licence::HEADER` and a blank line:
   `Copyright (C) 2026 David "Buzz" Bussenschutt`, "part of MissionPlannerRust, a Rust
   implementation derived from Mission Planner (Copyright (C) 2010-2024 Michael Oborne and
   contributors)", the GNU notice for version 3 alone, `SPDX-License-Identifier: GPL-3.0-only`.
   The generators emit it; `xtask/tests/licences.rs` fails on a file without it.
3. **NOTICE** is the GPL §5(a) statement of what was changed; **THIRD_PARTY_LICENSES** records
   every third party whose code, data or behaviour the tree carries (through Mission Planner's
   tree or directly), and its crate table is written by `cargo xtask licences` from
   `cargo metadata` - the test fails when it is stale.
4. **Not carried:** Mission Planner's six product photographs (the makers' pictures; the pages
   draw their named boxes), and anything under the Code Project Open License - AGauge.cs (the
   dials in gauge.rs are written from behaviour), Matrix.cs, Kalman3D.cs, NetDFULib.

**Why:** a public GPL repository must state its licence truthfully, mark itself as changed,
and carry the notices of what it includes.

**How to apply:** a new `.rs` file gets the header first (copy it from any file); after adding
or updating a dependency run `cargo xtask licences`; keep `deny.toml`'s allow list to what the
dependencies need; cite the Designers, never AGauge.cs's code, for the dials. See
[[port-from-the-csharp-source]] and [[not-in-the-csharp-not-in-scope]].
