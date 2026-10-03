---
name: no-callers-no-port
description: Buzz's rule (2026-09-25) - a .cs file or function with no callers anywhere in the C# tree is not ported; it goes in an "out of scope because of X" record instead
metadata:
  type: feedback
---

Buzz, 2026-09-25, after the D14 agent found that `Exocortex.DSP` and `fft3.cs` have no callers and
the real FFT is `FFT2`: "any/all .cs files or even .cs functions for which you can not find *any*
callers or users of it are to be written to a 'out of scope because of x' record, and not ported."

**Why:** the C# tree carries dead code (whole libraries under `ExtLibs/`, orphaned helpers); porting
it is work that ships nothing, and a DoD written against a dead library (D14's `Exocortex.DSP`) was
simply wrong until somebody looked for callers.

**How to apply:** before porting a `.cs` file or a function, grep the C# tree for its callers
(`grep -rn "ClassName\|methodName" "$MP_SRC" --include=*.cs`, minus its own file and
tests). None found → do not port; write it into the out-of-scope record with the reason "no callers
in the C# tree as of the reference commit" - the ledger (`ledger/ledger.csv`) row's status, and
PLAN.md §12 D13's list where it is a page or a library. If a DoD names it, correct the DoD to name
what the C# actually runs. Tell the agents this in every brief. Related:
[[not-in-the-csharp-not-in-scope]], [[port-from-the-csharp-source]].
