# Click everything: the first full crawl's report (2026-10-15)

The owner's request of 2026-10-14: "click all the things, and report all the stuff that doesnt
work". The plan is [click-everything.md](click-everything.md); this is its Part 3, the report.

## In short

`tools/gui-crawl` clicked every control of every screen and page of the planner, as built at
0c7f15e, in four states: two with no vehicle, two connected to an ArduCopter SITL of the crawl's
own. 8,770 clicks; 4,367 answered visibly, 175 changed only themselves (a tick), 146 took
typing; the rest were rows of long lists sampled out (3,106), out of reach (383), not controls by
the source (259) or denied as opening a port of their own (42).

292 clicks gave no visible answer. Each was clicked again, alone, in a fresh window: 257 still
gave none - 185 controls once the states are merged. Three of those, on the Full Parameter List,
were the crawler's own fault (its opening warning left over the screen, 13d6a27) and work. The
other 182 were each read against the Rust and the C# it ports:

| What it was | Rows | |
|---|---|---|
| Not a control: a label, picture, gauge, chart or page background | 47 | no change |
| Dimmed on purpose: drawn disabled in that state, so it takes no click | 45 | **question 1** |
| Silent in the C# too: the handler returns without a word in that state | 33 | **fixed**: the status line says why, quietly |
| Missing what the C# shows | 1 | **fixed** |
| Dead | 10 | **9 fixed**; 1 not ported |
| The crawler's own doing | 44 | the crawler fixed where it could |
| Not in the C# | 2 | **question 2** |

Found on the way, and fixed: the compass page's Fitness list pressed the button under it, and a
debug build's cut-off strip lay over DroneCAN's filter list. And four real gaps, ported since
(4f538d7, below).

## Fixed (55bb4be)

### Dead

- **CONFIG > Planner > Joystick Setup, the eight Settings buttons**: each opened its settings form
  and the form was never drawn - nor the page's message boxes, Import's question or its file
  dialogs. SETUP's joystick page drew them; the joystick window over CONFIG did not
  (`config/planner.rs`; `tests/gui/config-planner-joystick.gui`, new).
- **CONFIG > Planner > Browse** (the log folder): on a desktop with no folder dialog to show (no
  XDG portal) the press did nothing; it now says why on the status line.

### Missing what the C# shows

- **EXPERIMENTAL > Log Index, a sorted column's header**: the sort arrow was appended to the
  title and cut off with it in a 60-pixel column; the C# draws it beside the text. Drawn beside
  it now (`log_index.rs`).

### Silent in the C# too - now said on the status line, quietly

The port's rule for a press the C# answers with nothing (`fly::Refusal::Quiet`, the owner's bug
of 2026-10-14): say why, so a press that did nothing is not taken for one that did.

- FLIGHT DATA > Telemetry Logs: Play with no log loaded - "no log loaded".
- SETUP and CONFIG > Joystick: Detect with no joystick (fourteen rows) - "no joystick".
- CONFIG > Planner: Stop with no video running - "no video capture to stop".
- SETUP > Compass: Remove Missing with none missing - "no compass is missing".
- SETUP > OSD: Enable Telemetry where the vehicle has none of SR0_, SR1_ or SR3_'s rates.
- SETUP > RTK/GPS Inject: Messages Seen with none seen; Delete on the new base-position row.
- EXPERIMENTAL and SETUP > Advanced > Auth Keys: Disable Signing and Use with no vehicle - "no
  vehicle".
- EXPERIMENTAL > CoT: Clear on an empty box.
- EXPERIMENTAL > Log Index: Default or a folder with no logs in it.
- EXPERIMENTAL and LOGS > Geo Ref Images: Pre-process with no log or no folder of pictures; OK on
  a folder box naming nothing that is there.

Some presses did something and showed nothing; these now say what they did (**question 3**):
Auth Keys' and the Warning Manager's Save ("saved ... to" the file), Disable Signing with a vehicle,
EXPERIMENTAL's ReSort All logs ("N logs sorted"), and Force Accel Cal and Force Compass Cal
(whether the vehicle accepted).

### Found on the way

- **SETUP > Compass, the Fitness drop-down**: its list was drawn inside the page, so the Large
  Vehicle MagCal button lay under its second row - choosing "8" also opened MagCal's box. The list
  is drawn over the page now, as every other drop-down is.
- **SETUP > DroneCAN > Filter, in a debug build**: the red cut-off strip listed the filter's ninety
  boxes, which scroll in their list as the C#'s do, and lay over the list's rows. They may scroll
  now (`layout_guard.rs`).

## Decided by the owner (2026-10-16)

1. **Dimmed controls** (45 rows, listed below): a click on a dimmed control says why on the status
   line, quietly, as the Advanced page's withheld buttons do - done 2026-10-16 (`ui::Live`: the
   reason a control is dimmed for travels with it, its click says it and does nothing else; each
   reason is the C#'s own condition for `Enabled = false`, cited at the site). Beta Updates, listed
   among the 45, is live instead: the updater is ported and reads `beta_updates` (the ledger audit).
2. **FLIGHT DATA's tuning graph, its Clear button**: removed (bc2b1cf); the graph is emptied as
   playback starts, as the C#'s is.
3. **Confirmations of a silent success** (above): kept.
4. **CONFIG > Planner > Browse**: the planner's own chooser, as every other file dialog is - done
   2026-10-16 (`file_list::Chooser::folders`, a folder mode: `..` and folders, a folder clicked
   opened into the box, OK the folder the box names; `config-planner-browse.gui`).
5. **SETUP > Frame Type**: the class or type the vehicle has, clicked, says "already Quad" or
   "already Plus" (a665094).

## The second crawl (2026-10-16)

The crawler mended (its BGR read, its dimmed-control test, the boxes it answers on arrival, a dead
display not taken for a live one) and run again over the same binary (989565e): 8,791 clicks, 94
with no visible answer on 58 controls, from 257 on the first run - the crawler's own misreadings
gone. The 58, read one by one:

- **24 not controls**: the Advanced page's thirteen labels, Auth Keys' grid and label, DroneCAN's
  grid, Geo Ref's map, the telemetry log's file name, the raw sensor's bar, SIMULATION's map, the
  parameter tree, Parachute's picture, the Mount page's wiki link (handed to the desktop, which the
  crawl only logs), Proximity's picture.
- **9 dimmed on purpose**, saying why since the commit that closes this row: CONFIG > Planner's
  GDI+, Password, MAVLink debug, Temp, ADSB and Analytics boxes (and Beta Updates, live now), the
  page strip's left arrow, Geo Ref's seconds.
- **3 fixed since the binary**: the tuning graph's Clear (removed, bc2b1cf), Frame Type's Plus
  ("already Plus", a665094), the Planner page's Browse (the chooser).
- **19 the crawler's own doing**: the chooser's filter type already chosen (ten clicks), the HUD
  prompt's OK and Geo Ref's prompt field and checked radio, the proxy port and the SITL heading at
  their minimum, Frame Type's types and picture after its own Tri, and seven EXPERIMENTAL buttons
  reached behind Clear Custom Maps' "Removed 0 images" box, which the crawler did not answer -
  `experimental-message-ok` is in `BOX_OKS` now, and the self-test holds it.
- **3 answered as designed**: Geo Ref's Do It (status only), Clear Track and the SITL speed
  (internal state).

Twenty-one repeated status messages (Motor Test's, the joystick page's Detects, the spectrogram's
bounds, the update check, Force Mag Cal, the intervals' Set All) light the line a moment since
fbb5356. Nothing new for the port.

## The real gaps, ported since (4f538d7)

The four the triage found, each ported from the C# the same day:

- **LOGS > DataFlash > Download DataFlash Log Via Mavlink**: its four greyed buttons work. Create
  KML and .bin to .log run the DataFlash page's own conversions with this window's questions and
  Output lines (`LogDownloadMavLink.cs:469-514, 577-600`); First Person KML writes the C#'s
  first-person tour as a KMZ (`:525-575`, `LogOutput.cs:244-368`, a writer of its own in
  `mp-kml`); Clear Logs asks "Are you sure?" and sends `LOG_ERASE` twice, a request the link has
  now (`:450-467`, `MAVLinkInterface.cs:6269-6279`). Tested against a vehicle of the script's own
  holding two logs, so no SITL is erased (`logs-downloader-tools.gui`, `logs-downloader-clear.gui`).
- **FLIGHT DATA > HUD**: a click on the pre-arm text opens Prearm Checks, `PrearmStatus.cs`
  ported - it asks the vehicle to run its checks on opening and every five seconds while they
  fail, shows "Ready to Arm" or the vehicle's PreArm lines, and closes when the vehicle arms
  (`HUD.cs:1227-1231`, `FlightData.cs:3439-3447`; `fly-prearm.gui`). Where its zone overlaps
  EKF's, a click opens both, as the C#'s does.
- **FLIGHT DATA > Telemetry Logs, Play** (and Space): playback starting clears the tuning graph
  (`FlightData.cs:558-596`; `fly-playback-tuning.gui`).
- **SETUP > DroneCAN > Filter**: each box keeps its own tick, an untick removes one copy of its id
  from the list, ALL only its own (`ConfigDroneCAN.cs:899-962`; `config-dronecan-filter.gui`).

## The crawler's own faults

Fixed: it never told a dimmed control from a dead one (its screenshot read BGR as RGB); a box a
page opens with was left over a control the screen shows at once (13d6a27); its planners had no
map imagery, so the map type buttons did nothing; the parameter metadata's loading counted as a
click's effect; a text field's click counted as a fault where typing into it worked (0ad02f6);
FailSafe's "Ensure your props are not on" box took every click on its page; a check box's tick
fell under its threshold; and page backgrounds were clicked as controls (95315c9).

Still to teach it: answer a chain of boxes (Auth Keys' Add asks for a name, a passphrase and a
strength before Use is reachable); step a number up before down; pick another radio button before
clicking the checked one; drag a map rather than click it; press F2 from another screen; and leave
the SITL's frame class alone - its click on Tri dimmed the frame types for the rest of the crawl.

## The lists

**Dimmed on purpose** (question 1): CONFIG > Planner: Vario, Theme Custom, GDI+, Password, Beta
updates, MAVLink debug, Language, OSD colour, Theme, ADSB, Analytics, Temp; Full Parameter List:
Apply (until a comparison finds a change); EXPERIMENTAL: CoT's baud rate, Log Index's Delete, Geo
Ref's GeoTag Images, Estimate Offset and its seconds, MAVLink Inspector's Graph It, Warning
Manager's colour box; FLIGHT DATA > Actions without a vehicle: Abort Land, Change Speed, Force
Arm; the page strip's left arrow; FLIGHT PLAN: the fence's and the survey area's Clear and Undo
with nothing drawn; SETUP: ADSB's Save, Battery Monitor 2's pins, the Y6B frame type under a
quad, Airspeed's pin, Compass's Accept and Cancel before Start, the Mount page without
CAM_TRIGG_TYPE; SIMULATION: the swarm buttons (Multilink is not ported).

**Not controls**: labels (the Advanced page's thirteen, Auth Keys', the Log Index's statistics,
Proximity's caption, Accelerometer's text), pictures (Battery Monitor, Airspeed, OSD, Optical Flow,
Parachute, Geo Ref), the HUD clicked away from its zones, the raw sensor's output bars, the
DroneCAN and Auth Keys grids with nothing in them, Geo Ref's map with no marker, the telemetry
log's file name, Proximity's picture, and the background of seven pages, five of them pages ruled
out of the port.

**The crawler's own doing**: FailSafe's twelve (its opening box), Frame Type's four (the crawl's own
Tri), text fields already holding the keyboard (nine), DroneCAN's filter ticks (under the strip
above), RTK's NTRIP v1 tick, the map type buttons (no imagery), Auth Keys' Use (behind Add's
boxes), Geo Ref's checked radio button, Support Proxy's port at its minimum, MAVFtp's root node
(listed already), F2 on FLIGHT DATA, and SIMULATION's map (it acts on a drag).
