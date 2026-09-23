# The next twenty

Ordered by what unblocks the most, then by what an operator hits most often. Each item says what
"done" means, because a list of nouns is not a plan.

Status is updated as items land. The gap audit that produced part of this list runs against
`DELIVERABLES.md`, the vendored C# reference, and the test suite.

| # | Item | Why it is next | Deliverable | Status |
|---|------|----------------|-------------|--------|
| 1 | Text input widget | gpui has none, and its absence has shaped three screens already: no parameter search, no file names, no typed altitudes | D6 | done |
| 2 | Parameter search | 1408 parameters behind a prefix list; typing `WPNAV` is how people actually find one | D12 | done |
| 3 | Mission file name entry | save/load uses one fixed path, so a second mission overwrites the first | D11 | done |
| 4 | Settings that persist | link URL, tile provider, window size and screen are retyped every launch | D17 | done |
| 5 | Vehicle selector | the link tracks every vehicle on the wire and the UI always shows the first | D10 | done |
| 6 | ADS-B and other vehicles on the map | a GCS that cannot show nearby traffic is missing the one thing that prevents a collision | D15 | done |
| 7 | Flight path and mission export to KML | the common way to hand a flight to someone who does not have a GCS | D11 | done |
| 8 | Live tuning graph | watching a value against time is how tuning is done; the Flight Data screen has no plot | D10 | done |
| 9 | Log plotting | the same plot, over a dataflash log, which is how a flight is reviewed | D14 | done |
| 10 | Terrain-relative altitudes | a mission flown at 50 m over a hill is a mission into a hill | D11 | |
| 11 | Satellite imagery provider | planning over a paddock needs imagery, not a street map | D8 | |
| 12 | Mission Planner tile cache compatibility | D8 asks for it explicitly, and it lets an existing cache be reused offline | D8 | |
| 13 | Waypoint altitude and command from the map | right-click a waypoint to change it without crossing to the sidebar | D11 | |
| 14 | Geofence upload verification | the fence is written and never read back to confirm what the vehicle holds | D11 | |
| 15 | Fuzz targets actually built and run | they exist, have never been compiled, and D2's DoD requires 24 h clean | D19 | |
| 16 | Windows build verified in CI | cross-compilation is checked; the D3D11 path has never been exercised | D7 | |
| 17 | Joystick input | flying from a ground station without a transmitter, which D15 names | D15 | |
| 18 | Firmware flashing | the last thing in Initial Setup with no counterpart here | D13 | |
| 19 | Python scripting host | D16, and the owner's stated interest in extensions that need no compiler | D16 | |
| 20 | Packaging and installers | a build nobody can install is a build nobody uses | D20 | |

## Notes

**1 is first because it is load-bearing.** Three separate decisions were made to work around the
absence of a text input: parameters are browsed by prefix rather than searched, missions save to a
fixed path, and altitudes are stepped rather than typed. Each was the right call at the time and
each stops being right once typing is possible.

**10 and 11 are the two that change what can be flown**, rather than how comfortably. Everything
above them is reach; those two are range.

**15 and 16 are verification debt.** They do not add a feature and they are the two places where
this project currently claims more than it has tested.
