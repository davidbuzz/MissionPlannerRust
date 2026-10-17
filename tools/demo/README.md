# demo - the user demo's one command

The owner's demo (2026-10-17): a 5-10 minute autonomous, fully visible run of the Linux planner.

    tools/demo/run.sh             five minutes on this display with dist/planner; no arguments
    tools/demo/selftest.sh        the guards, without a window

`run.sh` starts `dist/planner --demo` on the display: the planner opens disconnected and its
Welcome-Demo-Sitl plugin runs with its drawn pointer - SIMULATION > Multirotor, PLAN, Zoom To
Vehicle, Set Home Here, four waypoints, a survey, Write, FLY > Actions, force arm, TakeOff and
Auto, then it unticks itself on PLUGINS (crates/mp-plugin-host/plugins/examples/welcomedemositl.rs).
When its pointer goes, `tour.py` drives the same window with the real pointer for the minutes
left: the click-everything crawl's recipe (`tools/gui-crawl/out/recipe.txt`) replayed screen by
screen, only the blocks the crawl saw happen, gliding to each control and dwelling after each
click so a watcher can follow. The planner stays open at the end, the copter flying.

It takes the mouse: on the desktop only when its owner says so. A rehearsal runs the same command
on an Xvfb display (`tools/gui-headless.sh` makes :99 with lavapipe) with `DISPLAY=:99`, and with
`XDG_DATA_HOME`, `MP_CONFIG_XML` and `MP_SETTINGS` pointed at a scratch directory so it leaves the
machine's settings, tiles and plugin choices alone.

What the tour never clicks: the crawl's deny list (`crawl.py`'s `DENY`: every button that opens a
port or device) and the demo's own (`tour.py`'s `DEMO_DENY`): the simulation and plugins tabs, the
flight modes and actions, arming, fly-to, set WP, mission and fence writes, parameter writes and
refreshes, the connection controls, Ctrl+T, prefetch, and anything that clears the mission. On
SETUP, CONFIG and the other pages only what the crawl's disconnected workers saw happen is played
(their blocks are labelled `disc-*`), so no parameter is written to the flying copter; FLY and
PLAN play the connected crawl's visible clicks less the deny. The demo's SITL is the SIMULATION
tab's, tcp 5760, as Mission Planner's is; `run.sh` first stops an earlier demo's planner and any
ArduPilot SITL left on that port, by pid (the owner's word of 2026-10-17), and stops itself if
something that is not a SITL holds it.
