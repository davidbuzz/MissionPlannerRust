---
name: csharp-tree-reads-are-measurement
description: Buzz's ruling (2026-10-03) - Rust code never opens a path under the C# clone; reading the tree is allowed only as measurement of the port (completion, and layout/text/data/shipped-copy fidelity, all throw-away at 100%), only through MP_SRC, and a test must pass doing nothing without it
metadata:
  type: feedback
---

Buzz, 2026-10-03, after the local clone's path had been written into 51 code sites: the clone "is
NOT part of the codebase, and should NOT be referred to by anything, anywhere"; "any rust code that
opens any file with it anywhere in the path is broken code, AND NEEDS TO BE FIXED"; then, on the
tests that compare the port with the C#: "references to .cs files (or anything else in the original
MP tree) that are purely for the purpose of measuring the completion of the port are acceptable,
as they are essentially throw-away code once the port is at 100% ... and need to all be resolved
via MP_SRC, so that when it's not defined, they gracefully do nothing, successfully"; and the
layout and text fidelity checks "are implicitly completion measurement" by the same logic.

**Why:** the repository must stand alone for the public: no private path, no test that silently
depends on a tree a reader does not have. The oracle checks are scaffolding for the port, not the
product.

**How to apply:** a test or command that needs the C# tree asks `xtask::upstream::tree()` or
mp-gui's `config_coverage::source::csharp_root()` / `csharp(path)` (or `std::env::var_os("MP_SRC")`
in a crate without them), and returns - passing, with a line saying MP_SRC is not set - when it
gets `None`; it never `expect`s the tree. A command that cannot work without the tree fails with
`upstream::absent()`. Never write the clone's location into any file of the repository; on this
machine it is set for cargo in `~/.cargo/config.toml` (`[env]`). PLAN.md §12 D26 is the ruling.
See [[port-from-the-csharp-source]] and [[no-callers-no-port]].
