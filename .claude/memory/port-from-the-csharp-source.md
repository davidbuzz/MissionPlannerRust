---
name: port-from-the-csharp-source
description: The C# tree - a clone of https://github.com/ArduPilot/MissionPlanner (efb0801) that MP_SRC names, outside the repository - is the spec; read the .cs file before implementing
metadata:
  node_type: memory
  type: project
  originSessionId: 1b798b4e-7109-4ec9-985c-603dbc9dc7de
  modified: 2026-09-23T00:00:00.000Z
---

**This project is a reimplementation of Mission Planner's C# code, and that code is on this
machine.** Buzz keeps a full read-only clone of https://github.com/ArduPilot/MissionPlanner (commit
efb0801) outside the codebase, and the environment variable `MP_SRC` names it: set for cargo in
`~/.cargo/config.toml`'s `[env]` on this machine, exported by the chain scripts for the tools
(`MP_SRC=$(grep MP_SRC ~/.cargo/config.toml | cut -d'"' -f2)` in a shell). Nothing in the
repository says where the clone is - Buzz, 2026-10-03: the clone's directory is not part of the
codebase and must not be referred to by anything, anywhere - so code that needs the tree reads
`MP_SRC` and, without it, skips or says what to set (`xtask::upstream`, mp-gui's
`config_coverage::source::csharp_root`). zed's source sits beside it the same way, for reading
gpui. Both are easy to forget exist: neither shows in `git status`.

Before writing any feature that Mission Planner already has, **find and read the `.cs` file**:

```sh
find "$MP_SRC" -name "ParamFile.cs"
grep -rn "SaveParamFile" --include=*.cs "$MP_SRC"
```

**Why:** on 2026-09-23 I implemented `.param` file save/load having concluded the C# source was
not on this machine, and reconstructed `ExtLibs/Utilities/ParamFile.cs` from memory. Reading the
actual file afterwards, nearly every detail was wrong:

- The skip-list has **16** entries, not the 7 I remembered. The nine I missed — `GND_TEMP`,
  `BARO1_GND_PRESS`, `BARO2_GND_PRESS`, `BARO3_GND_PRESS`, `BARO_GND_TEMP`, `CMD_INDEX`,
  `LOG_LASTFILE`, `FORMAT_VERSION` — are the same class of thing as the seven I had.
- It is applied on **load**, not on save. `SaveParamFile` writes whatever it is handed.
- Numbers are written as `value.ToString(InvariantCulture)` — shortest representation. Mission
  Planner writes `ACRO_RP_EXPO,0.3`, not `0.300000`.

I then committed a message asserting the output was "byte-for-byte the shape the C# application
produces", and built a test fixture to match my invention rather than the real format. Buzz had to
say "we are reimplementing the .cs code missionPlanner" to stop it.

**How to apply:** when a feature has a Mission Planner counterpart, the `.cs` file is the
specification and reading it is the first step, not a check afterwards. Cite the path and line in
the code comment, as `ExtLibs/Utilities/ParamFile.cs:50` — a citation that was never opened is
worse than none, because it reads as evidence. Where the Rust deliberately differs from the C#,
say so explicitly and say why; a silent divergence looks like a bug to the next reader. If a
protocol or format detail is being recalled rather than read — bootloader command bytes, file
layouts, magic numbers — that is the signal to go and open the file. See
[[verify-edits-applied]].
