# Click everything: a plan (the owner's request, 2026-10-14)

> "click all the things, and report all the stuff that doesn't work ... a GUI test tool that
> literally clicks every single button in the entire app, esp all the not-immediately-visible
> ones" - the owner, 2026-10-14.

The goal is a list of every control a user can click that gives no visible response, made by a
program rather than by hand, and the systemic cause behind many of them fixed first.

## Part 1 - the status line on every screen

**The cause.** Many buttons answer only through `file_status`, the status line. It is drawn in one
place: `map_status`, the grey line under the map, so only FLIGHT DATA and FLIGHT PLAN ever show it,
prefixed to the map's statistics and cut off at the window's width. On SETUP, CONFIG, PARAMS, LOGS,
SIMULATION, HELP, EXPERIMENTAL and PLUGINS a button whose answer is a status message looks dead
(found by the first click audit, 2026-10-14). The GUI scripts read the `status` fact, not the
screen, so none of them failed.

**The change - the owner's design (2026-10-14).**
> "move/relocate/new 'map_status' that is OUTSIDE AND ABOVE the map/s, and inside the top nav
> area, ie be4low the tab buttons and below the connection info, next-to the maps ... on "fly" tab,
> there is a "tuning" block at teh top above the map, i want it below the map instead ... plan and
> simulation tabs dont need any fix ... all tabs can now write info here and have it be seen by
> the user."

- A status row, the header's last line: across the window under the tabs and the link's state,
  above every screen's body, of a fixed height so nothing moves when a message comes or goes.
- At its left, `file_status` when there is one (`status-bar`): an "Error: ..." in the alert
  colour, anything else in the text colour, one line, cut to the room left with the whole text in
  a tooltip. This is the owner's no-dialog rule kept properly: an error is shown as the main
  window's state.
- At its right, on FLIGHT DATA and FLIGHT PLAN, `map_status` (`map-status`): the shared map's
  statistics, moved up out of the strip along the map's foot, held to half the row. SIMULATION's
  map is its own and never had the strip.
- FLIGHT DATA's tuning graph under its map, where the C# has it over (`splitContainer1`), so the
  map is directly under the row.
- A fact, `status.drawn`, records what the line drew in the frame (not what was set).
- A second, `status.serial`, counts the messages set, so the same words set again - which change
  nothing on screen - are told from none (2aaff89).

(A first version drew a bar along the window's foot: at 1280 by 800 it cut EXPERIMENTAL's table
off. A second put the message in the header's second line, after the link's state, until the
owner's design.)

**Tests.** A unit test for the line's colour; `tests/gui/status-bar.gui` sets a status on four
screens and expects `status.drawn` and the `status-bar` control on screen, and on FLIGHT DATA and
FLIGHT PLAN the map under the row, the statistics at its right and the tuning graph under the map;
it sets EXPERIMENTAL's DEM answer twice and expects `status.serial` to go 1, 2.

## Part 2 - the crawler, `tools/gui-crawl/`

A Python program over the planner's own probe (`MP_PROBE`: every named control's rectangle) and
facts (`MP_FACTS`), on virtual displays (Xvfb, lavapipe), so nothing touches the owner's desktop.

**Reach.** It starts from every screen and every page (SETUP's groups opened), in two states:
disconnected, and connected to a copter SITL. On each page it clicks every named control, and:
- when a click makes new controls appear - a page, a drop-down, a dialog, a menu, a sub-form - it
  explores those too, up to three levels deep, then resets: Escape, or a relaunch and a replay of
  the path when Escape does not restore the page;
- it right-clicks the maps and the grids, to open their context menus, and explores those;
- it scrolls a scrolling list to bring a row it could not see into view, then clicks it;
- it presses every keyboard shortcut the HELP text lists, on the screen it belongs to;
- a page that opens with a box over it (the Full Parameter List's warning) is answered first.

**Observation.** Each click is judged on what a user could see, and on what changed behind it:
- the window captured before and after, the changed pixels counted outside the clicked control and
  inside it separately (a focus border is not a response), a blinking caret and the screen's own
  animation (taken from an idle baseline) left out;
- the facts compared, the telemetry that drifts on its own left out;
- the status line's text, and its count (`status.serial`) for an answer that repeats the message
  already showing; a URL or folder handed to the desktop (fake `xdg-open` and `gio`);
  controls that appeared or vanished; the screen changing;
- whether the control is drawn disabled (dim text on the panel colour), which is not a fault.

**Verdicts.** `visible`, `changed-itself`, `status-only`, `repeated-status`, `internal-only`,
`nothing`, `disabled`; a text field's `typed` or `untypable`; and `denied`, `sampled-out`,
`unreachable`, `not-a-control`. Every `nothing`, `internal-only`, `status-only` and `untypable` is
clicked again in a fresh window through its recipe, with a long wait, before it is reported; a
`repeated-status` is listed apart from the faults.

**Safety.** A deny list is never clicked: reboot, bootloader or firmware flashing, DFU, install,
erase, format, exit, quit, the parameter resets, disconnect, and every button that opens a port of
its own - the output windows' Connect, the MAVLink Mirror's Write and Go, DroneCAN's connect, SiK
Radio, the serial passthrough, the video. HELP's update buttons are clicked: they only say updates
are not implemented (df43b93). SITL is disposable; no real hardware is connected; one SITL per
connected worker, on instances 7 and 8 (tcp 5830 and 5840), and `run.sh` stops unless the listener
on each port is the simulator it started - `tools/sitl/run-sitl.sh` refuses a port already in use.
The planners run offline with a made-up firmware device (`MP_FIRMWARE_DEVICE`), so no firmware page
opens a real serial port.

**Output**, under `tools/gui-crawl/out/`:
- `report.md` - every verdict counted, the controls that repeated the message already showing,
  then the faults, grouped by screen and page, each with the way there;
- `results-<worker>.jsonl` and `verify-<worker>.jsonl` - every click, one row each, and the
  verification pass's;
- `recipe.txt` - the whole crawl as one replayable list of actions;
- `dead/<state>-<screen>-<control>.gui` - for each fault, a `.gui` script that reproduces it with
  `tools/gui-headless.sh`, to test its fix.

**Running it.** `tools/gui-crawl/run.sh` starts the displays and SITLs and four workers (two
disconnected, two connected), and stops everything by PID at the end. Expected: a few thousand
clicks, two to three hours.

**Its own test.** `tools/gui-crawl/selftest.sh` checks the guards first - the deny list, and
`run-sitl.sh` refusing a port something holds - then crawls HELP alone and checks the verdicts it
already knows (Show Console visible; Changelog hands a URL to the desktop; the welcome text is not
reported as a control).

## Part 3 - after the crawl

The report goes to the owner, with the faults in priority order. Fixing them is separate work;
each fix comes with its `dead/*.gui` script turned into a passing test.

## Order of work

1. Part 1: the status bar, its tests, a commit (2b33c13; the count, 2aaff89).
2. Part 2: the crawler, its self-test, a commit (40c7425).
3. The full crawl on the planner as built at that commit; the report to the owner.
