# gui-crawl - click everything

Every named control on every screen and page of the planner clicked, and whatever each click
reveals (drop-downs, dialogs, menus, sub-forms) explored too, three levels deep; the maps
right-clicked; the HELP text's shortcuts pressed; rows scrolled into sight. Each click is judged
on what a user could see. The plan and its reasons: `docs/click-everything.md`.

    tools/gui-crawl/run.sh             the whole crawl, three to four hours: out/report.md
    tools/gui-crawl/selftest.sh        its guards; HELP alone, its known verdicts checked; a
                                       control reached as verify.py reaches it; a dimmed
                                       control told from a live one

* `crawl.py` - the crawl, one worker; `--screens`, `--pages`, `--display`, `-- tcp:...`. A box a
  page opens with (`BOX_OKS`: the Full Parameter List's warning, FailSafe's "Ensure your props
  are not on", EXPERIMENTAL's message box) is answered on arriving at the page, after every step
  and before every click - a box that comes up late, after a click's settle, would take the next.
* `verify.py` - each fault clicked again alone, with a long wait.
* `report.py` - `out/report.md`, `out/recipe.txt` (every click as `.gui` lines) and
  `out/dead/*.gui` (a script per fault, for `tools/gui-headless.sh`). A control the source gives
  no click is `not-a-control`; a text field whose typing showed is no fault for its click.
* `observe.py` - the verdict: `visible`, `changed-itself`, `status-only`, `repeated-status` (the
  message already showing set again, counted by the `status.serial` fact), `internal-only`,
  `nothing`, `disabled` (dim text on the panel's colour, told by its pixels); a text field's
  `typed` or `untypable`; and `denied`, `sampled-out`, `unreachable`, `not-a-control`. A change
  wholly inside the control - a check box's tick - is `changed-itself` from 24 pixels.
* `driver.py` - the planner on Xvfb with lavapipe, its probe (`MP_PROBE`) and facts (`MP_FACTS`),
  `xwd` (each pixel's channels by the header's byte order and masks) and `xdotool`; offline, a
  map source configured and nothing fetched.

Runs on virtual displays only (:90 to :94); never on the desktop. Needs Xvfb, xdotool, xwd,
python3 with numpy, the debug planner (`cargo build -p mp-gui --bin planner`) and, for the
connected half, `tools/sitl`.

It reaches nothing outside its own planners. The connected half drives SITL instances 7 and 8
(tcp 5830 and 5840), and `run.sh` stops unless the listener on each port is the simulator it
started: the owner runs his own SITL (5760, from the SIMULATION tab) and planner (UDP 14550)
while it works. The deny list (`crawl.py`'s `DENY`) holds every button that opens a port of its
own - the output windows' Connect, the MAVLink Mirror's Write and Go, DroneCAN's connect, SiK
Radio, the serial passthrough, the video - and the planners run offline with a made-up firmware
device, so no firmware page opens a real serial port. `selftest.sh` checks these guards first.

The first full run and what its faults were: `docs/click-report.md` (2026-10-15). What it cannot
do yet, each found there: answer a chain of boxes (Auth Keys' Add asks three questions before
Use can be reached), step a number up before down, pick another radio button before clicking the
checked one, drag a map, press F2 from another screen, and leave the SITL's parameters as it
found them (its click on Frame Type's Tri dimmed the frame types for the rest of that crawl).
