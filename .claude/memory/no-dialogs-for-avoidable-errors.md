---
name: no-dialogs-for-avoidable-errors
description: Buzz's ruling (2026-09-25) - never show a dialog box for an error that the main window can show as state, connectivity or a colour change; the C#'s MessageBox is not the spec for those
metadata:
  type: feedback
---

Never display a dialog box for an error that can instead be shown as state, connectivity or a
colour change in the main application display. The first case: the C#'s "lost communication with
the board." / "comms timeout" boxes in `Utilities/Firmware.cs:702, 710` - the port went away during
a flash. Buzz: "a totally unnecessary dialog box, pls hide it so it *never* shows up again and is
permanently disabled in the code. the connectivity info is *always* avail top-right in the main
screen."

**Why:** the link's state is always visible at the top right of the window, and a modal that
repeats it only interrupts the operator; MP's habit of boxing every exception is one of the things
this port is allowed to leave behind (a written divergence, not an invention: the information is
still shown, just not as a box).

**How to apply:** when porting a `CustomMessageBox.Show` / `MessageBox.Show` that reports an error
(not a question, not a confirmation), ask whether the window already shows - or could show - that
condition as state: a status line, a connection indicator, a colour. If it can, put the words there
and write the divergence at the site, citing this ruling. Pin it with a test that the path shows no
box (`person.shown.is_empty()` in the firmware bench). Questions that need an answer, and
confirmations before something irreversible, keep their boxes. Related: [[match-the-original-layout]]
(Buzz is the oracle on look and feel), [[bench-cubeorange]].

Buzz, 2026-09-25, on the show-again boxes: questions, confirmations and warnings "aren't *errors*, so are acceptable as dialog boxes" - `MessageShowAgain`'s warnings with their "Show me again?" tick and the armed-only "Refresh Params" question stay boxes (PLAN §12 D17).
