---
name: not-in-the-csharp-not-in-scope
description: CRITICAL — if Mission Planner's C# does not do it, it is not in scope. Never invent features.
metadata:
  node_type: memory
  type: feedback
  originSessionId: 1b798b4e-7109-4ec9-985c-603dbc9dc7de
  modified: 2026-09-23T00:00:00.000Z
---

# CRITICAL — READ BEFORE BUILDING ANYTHING

**"ITS NOT IN SCOPE IF ITS NOT PART OF MISSION PLANNER C#."** Buzz, 2026-09-23, after I added an
ASCII terminal plot to `mpr` for dataflash logs.

Mission Planner plots logs with ZedGraph in `Log/LogBrowse.cs` — a WinForms chart. There is no
terminal plotting anywhere in the 1.2M lines, and D14's definition of done is explicitly a GPU
chart. I built the ASCII renderer because it was the shape I could verify from a terminal without
opening a window, which is a convenience for *me* and not a thing the product does.

**Why this is the rule and not a preference:** the goal is a reimplementation that accounts for
100% of Mission Planner's behaviour. Every invented feature is work that does not close a ledger
row, has no oracle to be checked against, and has to be maintained forever by someone who will
wonder what it was for. It also makes the completeness claim unfalsifiable — "3,678 of 3,678" means
nothing if the Rust side has grown things the C# never had.

**How to apply:** before building anything, find it in `references/missionplanner`. If it is not
there, it is not in scope — say so and do the thing that is, rather than building the adjacent
thing that was easier. Two legitimate exceptions, and neither is a feature:

- **Test and development scaffolding** that never ships to a user: `tools/screenshot.sh`,
  `MP_SMOKE`, `mp-fuzz-checks`, the corpus analysis in `mp-script`. These exist to verify the port.
- **A deliberate divergence from a C# behaviour**, where the C# is wrong — the KML writer dropping
  the last flight segment, the `frame_parse` fuzz gap. Those are changes to a ported behaviour, not
  new features, and each one is written down where it happens with the reason.

If something seems worth having and is not in the C#, it is a question for Buzz, not a decision to
make while implementing something else. **Ask; do not build it and mention it afterwards.**

Buzz asked for this to be recorded as a critical, essential memory, in capitals, having had to say
it three times in five minutes. Treat a plan item as a *question about where the behaviour lives in
the C#* before it is a question about how to write it in Rust. See [[port-from-the-csharp-source]] — that memory is about
reading the `.cs` before writing; this one is about not writing when there is no `.cs` to read.

**The other direction (2026-09-25):** being in the C# is necessary, not sufficient. Buzz keeps an
explicit out-of-scope list in `PLAN.md` §12 D13 (PX4Flow, Bluetooth Setup, the Antenna Tracker,
Ateryx, ESP8266, CubeID, Terminal, REPL, LogAnalyzer, OSD Video, Altitude Angel, Swarm, Follow
Me, Moving Base, the example plugins…), and rulings on the rest of §12. Check it before queuing a
page or a row; a ruled-out feature's ledger row is `Dropped` with the ruling as its reason, never
`Missing`. When something looks low-engagement and is not on the list, suggest it to Buzz with its
size in lines; do not skip it silently.
