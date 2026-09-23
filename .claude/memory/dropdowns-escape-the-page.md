---
name: dropdowns-escape-the-page
description: A drop-down list drawn inside a page is clipped by the page's scroll container and its rows past the thirtieth are unreachable; draw it deferred+anchored, window it, and scroll it with the runner's `scroll` directive
metadata:
  type: project
---

A combo box's list on a setup/config page must be `gpui::deferred(gpui::anchored().child(list))`
inside a zero-size `div().absolute().left(x).top(y)`: the page body is a scroll container and
clips an `.absolute()` child that runs past its edge, while the probe still reports the clipped
rows' positions, so the runner clicks where nothing is drawn. The list shows `LIST_ROWS_SHOWN`
(30) rows from `Combo::top_index`, opened on the selected row and moved `WHEEL_ROWS` (3) a notch
by `on_scroll_wheel` (`config/servo_output.rs::dropdown`, shared by the serial and ESC pages).

**Why:** the fetched `apm.pdef.xml` lists a `SERVOn_FUNCTION`'s 128 values and a
`SERIALn_PROTOCOL`'s 48 alphabetically, so RCPassThru is row 96 and MAVLink2 row 30; the agent's
scripts clicked them at y=1941 and y=862 and failed (2026-09-24).

**How to apply:** in a script, `click <combo>`, `scroll <combo>-list down N`, assert
`config.<page>.list.top`, then click the row. Sixty notches down clamps at the end (forty once lost some under load) and two up
gives a known top whatever the rows-per-notch; `expect` pins the exact value once measured.
Reuse `servo_output::dropdown` for any new page's combo rather than writing a list.
