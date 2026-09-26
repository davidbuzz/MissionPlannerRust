---
name: mode-buttons-kept
description: The flight screen's mode buttons panel is the port's own, not the C#'s CMB_modes/Set Mode - Buzz chose to keep it (2026-09-26); do not remove it under the not-in-the-C# rule
metadata:
  type: project
---

Below the Actions grid the flight screen has one button per flight mode (`fly.rs`
`mode_controls`); Mission Planner's Actions tab instead picks the mode with the `CMB_modes`
drop-down and `BUT_setmode` in the grid, whose cells the port leaves empty. Asked on 2026-09-26
(fly-actionsgrid ran 12 px past its column once a 4.8 SITL's mode list arrived), Buzz chose
**keep the mode buttons**, over restoring the C#'s drop-down or having both.

**Why:** in his words, "i dont like the usability of a drop-down, because in all circumstances,
its a 3-mouse-click experience, and a row of buttons is a single-mouse-click experience. better
feel." His ruling on look and feel ([[match-the-original-layout]] makes him the oracle); many GUI
scripts pick modes by clicking these buttons too (`click Stabilize`, `click Guided`).

**How to apply:** leave the panel; the Actions page may scroll, and fly-actionsgrid no longer
checks `fly.page.overflow`. See [[not-in-the-csharp-not-in-scope]] - this is a recorded exception.
Where the C# has a drop-down for a short, often-used choice, one-click buttons are the kind of
change he may want - but ask first, as here; the default stays the C#'s layout.
