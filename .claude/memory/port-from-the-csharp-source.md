---
name: port-from-the-csharp-source
description: The C# tree at references/missionplanner is the spec; read the .cs file before implementing
metadata:
  node_type: memory
  type: project
  originSessionId: 1b798b4e-7109-4ec9-985c-603dbc9dc7de
  modified: 2026-09-23T00:00:00.000Z
---

**This project is a reimplementation of Mission Planner's C# code, and that code is here.**
`references/missionplanner` is a full read-only clone of the upstream tree (the directory was
named `referneces` until 2026-09-25, when Buzz renamed it). It is gitignored, so it does not appear
in `git status` and is easy to forget exists. `references/zed` is the gpui tree, the same way.

Before writing any feature that Mission Planner already has, **find and read the `.cs` file**:

```sh
find references/missionplanner -name "ParamFile.cs"
grep -rn "SaveParamFile" --include=*.cs references/missionplanner/
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
