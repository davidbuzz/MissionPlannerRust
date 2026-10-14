# gui-crawl - click everything

Every named control on every screen and page of the planner clicked, and whatever each click
reveals (drop-downs, dialogs, menus, sub-forms) explored too, three levels deep; the maps
right-clicked; the HELP text's shortcuts pressed; rows scrolled into sight. Each click is judged
on what a user could see. The plan and its reasons: `docs/click-everything.md`.

    tools/gui-crawl/run.sh             the whole crawl, two to three hours: out/report.md
    tools/gui-crawl/selftest.sh        HELP alone, its known verdicts checked

* `crawl.py` - the crawl, one worker; `--screens`, `--pages`, `--display`, `-- tcp:...`.
* `verify.py` - each fault clicked again alone, with a long wait.
* `report.py` - `out/report.md`, `out/recipe.txt` (every click as `.gui` lines) and
  `out/dead/*.gui` (a script per fault, for `tools/gui-headless.sh`).
* `observe.py` - the verdict: `visible`, `changed-itself`, `status-only`, `internal-only`,
  `nothing`, `disabled`; a text field's `typed` or `untypable`; and `denied`, `sampled-out`,
  `unreachable`, `not-a-control`.
* `driver.py` - the planner on Xvfb with lavapipe, its probe (`MP_PROBE`) and facts (`MP_FACTS`),
  `xwd` and `xdotool`.

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
