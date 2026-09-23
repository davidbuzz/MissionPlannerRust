---
name: match-the-original-layout
description: Buzz is the final oracle on look and feel; default to Mission Planner's layout unless he says otherwise
metadata:
  node_type: memory
  type: feedback
  originSessionId: 1b798b4e-7109-4ec9-985c-603dbc9dc7de
  modified: 2026-09-23T00:00:00.000Z
---

**"i, the human am the final oracle on look/feel/style/implementation, but for most things, the
closer it feels to an original MP layout, the better. where i choose to vary it, i say so here."**
— Buzz, 2026-09-23.

Two halves and both matter:

**Default to Mission Planner's layout.** Not "re-lay it out and see if anyone objects". If a screen
exists in the C#, find where its controls sit before deciding where ours go — which panel holds
what, what is on by default, what order things appear in. The tuning graph is the worked example:
it was put at the bottom of a sidebar because that was convenient, and the C# has it in
`splitContainer1.Panel1`, collapsed until `CB_tuning` uncollapses it, above everything else. The
faithful placement was also the better one.

**The layout is in the `.resx`, not in a screenshot.** `FlightData.resx` alone carries 845 geometry
entries and 834 control-tree entries — `>>ctrl.Parent`, `>>ctrl.ZOrder`, `Location`, `Size`,
`Anchor`, `Dock` for every control. Exact, diffable, already in the tree. Read that rather than
guessing or asking for a picture.

**Buzz decides, and he will say so.** A divergence is his call, not one to make while implementing
something else and mention afterwards. When something cannot be matched — gpui has no
`splitContainer`, no `ToolStrip`, no `DataGridView` — say which MP control it stands in for and
why it differs, rather than quietly inventing an arrangement.

**This softens PLAN.md §1.2.** That section lists "bug-for-bug WinForms pixel fidelity" as a
decided non-goal and says "screens are re-laid out, not traced". *Pixel* fidelity is still a
non-goal — MP ships `<dpiAware>false</dpiAware>` and its own pixels are not stable across machines
— but "re-laid out" was too strong and has been corrected in the plan. Close in feel, by default.

## Ratified divergences — keep this list, do not "fix" these back

Buzz said he would say where he varies it. This is where those go. Anything here is **settled**:
do not move it toward the original, and do not raise it again.

| Area | Ruling | Date |
|---|---|---|
| **Colour palette** | **Keep the dark theme. Do not implement Mission Planner's scheme.** His words: *"i have not currently complained about the fact that the color palette isnt bright GREEN AND BROWN, which is how the original is, because those colors in the original are nasty. do NOT implement the original color scheme (burnt frog)"*. This covers the chrome — panels, text, borders, buttons. It does **not** cover the artificial horizon, where blue sky over brown ground is the universal convention for an attitude indicator rather than a Mission Planner style choice. | 2026-09-23 |

See [[not-in-the-csharp-not-in-scope]] for the companion rule about features; this one is about
arrangement.
