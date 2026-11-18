# demo - the user demo

The owner's demo (2026-10-17): an autonomous, fully visible run of the Linux planner. Since
2026-11-18 it is one editable list, `owner.gui`, and the planner plays it by itself.

    planner --demo                the demo: the list played with a drawn pointer
    tools/demo/run.sh             the same with dist/planner, after making the way clear for it

`planner --demo` opens disconnected and plays `tools/demo/owner.gui`, a step a line, from the
top: SIMULATION > Multirotor, PLAN, Zoom To Vehicle, Set Home Here, four waypoints, a survey,
Write, FLY > Actions, force arm, TakeOff and Auto - what the Welcome-Demo-Sitl plugin did while
its steps were fixed in its source - and then whatever the list goes on to. The pointer is the
planner's own, drawn over the window; the real mouse is not taken. The planner stays open at the
end, the copter flying.

The list is read from where the planner is started, so run it from the repository:
`./target/debug/planner --demo`. Started elsewhere, it plays the copy that was built in.

`run.sh` adds only this: it stops an earlier demo's planner and any ArduPilot SITL left on tcp
5760, by pid (the owner's word of 2026-10-17), stops itself if something that is not a SITL holds
that port, starts `dist/planner --demo`, and says how the list ended. A rehearsal runs it on an
Xvfb display (`tools/gui-headless.sh` makes :99 with lavapipe) with `DISPLAY=:99`, and with
`XDG_DATA_HOME`, `MP_CONFIG_XML` and `MP_SETTINGS` pointed at a scratch directory so it leaves the
machine's settings, tiles and plugin choices alone.

## The list

`owner.gui` says its own line forms at its head: `click`, `screen`, `key`, `type`, `wait`,
`answer`, `pause`. `crates/mp-gui/src/demo_script.rs` reads and plays them.

A `click` whose control does not come within five seconds, or a `screen` that does not show, is
passed over and the demo goes on: a list recorded in one window is played in another (the fact
`demo.skipped` holds those lines' numbers). A `wait` that does not come - a vehicle that does not
arm - stops the demo where it is: the planner is left as it stands, the status line says which
line and why, and the fact `demo.script` says the same.

## Your own clicks

`planner --record` prints every click you make as a line of the list on the command line -
`screen plan`, `click plan-zoomicon`, `click map@0.40x0.40` (where in the map, two places),
`click map@0.50x0.50:right` for the right button; `--demo` prints them too. Paste the lines you
want into `owner.gui`, anywhere. Typing is not recorded: add a `type <text>` line by hand where
you typed.

## What changed on 2026-11-18

Before, `--demo` ran the Welcome-Demo-Sitl plugin, and `tour.py` then drove the window with the
real pointer for the minutes left: your `owner.gui` lines, and after them the click-everything
crawl's recipe, screen by screen. The owner's word: "i would like this editable-list-of-clicks to
be the entire demo, and for it to start-with whats in the demo right now". So the plugin's steps
are the list's first lines, the crawl's tour is no longer part of the demo, and `tour.py` and its
self-test are gone. The plugin itself is the browser build's welcome, as before.
