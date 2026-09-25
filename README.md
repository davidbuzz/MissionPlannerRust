# Mission Planner, in Rust

A complete, file-by-file reimplementation of [ArduPilot Mission
Planner](https://github.com/ArduPilot/MissionPlanner) — 3,678 C# files, 1,208,836 lines, ~93
projects — as a Rust application that is **fast**, **multi-platform** and **GPU-accelerated**.

- **What we are building**: [DELIVERABLES.md](DELIVERABLES.md) — 20 deliverables, each with a
  falsifiable definition of done, the C# paths it replaces, and a numeric target.
- **Progress screenshots**: [docs/progress/](docs/progress/)

## Status

Early, but flyable behind SITL and a real autopilot. The protocol and telemetry spine is solid; the
UI covers flying, planning and the first of the setup screens.

Measured on this tree: **23 crates, 252,700 hand-written Rust LOC** (plus 91,634 generated; `.rs` files
under `crates/`, tests included), **2,746 tests** green on `cargo test --workspace` (40 ignored:
they need SITL, a window, or the network), **143 GUI scripts** under `tests/gui/`, across 218 commits.
Linux only, so far: the repository has no remote, and the three-OS CI matrix has never run.

| Working today | |
|---|---|
| MAVLink v1/v2 codec | zero-copy parse, allocation-free encode, v2 signing |
| Generated dialect | 349 messages, 206 enums, generated from the upstream XML |
| Transports | serial, TCP, UDP, a UDP client, websocket and NTRIP (the last three held to the C# classes run under mono), file replay, in-memory test doubles; port enumeration by `CommsSerialPort.GetPortNames`'s rules, held to per-OS fixtures; faults and a real pty unplug rehearsed in tests |
| Link engine | I/O thread, multi-vehicle routing (50 systems in one test), stream requests; parameter sets, reads, commands and `COMMAND_INT`s, set-current, single mission items, the home-position ask and mission transfer with Mission Planner's own retry counts and waits, proved by counting sends under dropped, delayed and duplicated frames; every set and command the screens and `headless-planner` send goes through them |
| Vehicle state | lock-free snapshot bus, packet-loss tracking; all 550 of `CurrentState`'s members accounted for (471 held with the C#'s rules, 48 derived, 30 plumbing, 1 dropped), the last 55 matched per packet to the C#'s own `UpdateCurrentSettings` under mono |
| Parameters | fetched as Mission Planner fetches them: `@PARAM/param.pck?withdefaults=1` over MAVFTP first, the `PARAM_REQUEST_LIST` stream with gap recovery when that will not do; started on its own once a vehicle is heard and nothing is held; typed values, 1,408 from SITL |
| `.param` files | save, load and compare against a vehicle, honouring the C# skip-list |
| Parameter docs | fetched for the connected firmware as Mission Planner fetches them (`apm.pdef.xml`, versioned or weekly), read before the bundled table: 1,407 of a SITL's 1,408 documented instead of 798 |
| Missions | upload and download, `.waypoints` files, 129-file corpus |
| Survey grids | `Grid.CreateGrid`, `CreateCorridor` and `CreateRotary` transliterated over a port of ProjNet's UTM and the C#'s Clipper, bit-identical to the C# on 284 golden cases the real code generated under mono; the Survey (Grid) dialog is `GridUI.cs` whole, its Accept held to GridUI's own code under mono over 40 cases (5,656 Accept calls) bit for bit |
| Logs | `.tlog` read and write; ArduPilot `.BIN` dataflash parsing; the log browser with `LogBrowse.cs`'s two axes, data grid, map, double-click cursor, mode/error/message overlays, Show Params, the preselected graph sets, the five routes, point values, zoom and pan and the grid's export menu, opening a 1 GB log to its first plot in about 0.6 s; `.BIN → .log`, KML+GPX and `.mat` conversions byte-identical to `BinaryLog`, `LogOutput` and `MatLab` run under mono |
| Flight recording | every connection recorded to a `.tlog`, both directions, into the `logs` directory of this application's own data directory |
| Data directory | a directory of its own, `~/.local/share/MissionPlannerRust` on Linux (`$XDG_DATA_HOME/MissionPlannerRust`, never `~/MissionPlannerRust`) and `Documents\MissionPlannerRust` on Windows, so neither application writes over the other's files; the C#'s is found by `Settings.cs`'s rules, mono quirks included; the first start with that directory missing or empty imports the C#'s `config.xml`, `poi.txt`, `cameras.xml` and its other user files once, leaving them as they were; `config.xml` is read for the last link, map type, log directory, planner home and quick views, and written whole on the C#'s events (start-up, the screen buttons, Connect, the close box), byte for byte what the C# writes for the same keys |
| Health | EKF variances and vibration with ArduPilot's own thresholds, clipping counts |
| Calibration | accelerometer, compass, radio and motor test as Mission Planner's own pages, `ConfigHWCompass2`, `ConfigRadioInput` and `ConfigMotorTest` ported whole, in its SETUP list |
| Joystick | axes to `RC_CHANNELS_OVERRIDE` from a thread that blocks on the device and sends on change — 0.1 ms p99 stick-to-link on a fake device — with a release-on-disconnect failsafe (Linux) |
| Firmware | `.apj` parsing, the px4 bootloader protocol, `BoardDetect.cs`'s board detection and `APFirmware.cs`'s catalogue with the Install Firmware page, proven against a mock, a pty and a manifest excerpt; `UploadPX4`'s reboot into the bootloader, port scan and upload wired to real ports and proven against the mock on a bench of pretend ports, and on 2026-09-25 against the bench CubeOrange: ArduCopter 4.7.1 stable flashed from the Install Firmware page, "Upload Done" (PLAN §13.6 row 79). Port failures during a flash go on the status line, never in a box (the owner's ruling) |
| Scripting | the `Script.cs` host API, and a measurement of what the 19 shipped scripts need |
| KML export | a flown path coloured by flight mode, and a mission, for Google Earth |
| Tuning graph | eleven telemetry fields plotted live, min/max reduced so a spike cannot hide |
| Geodesy | typed units, Web Mercator, slippy-map tile arithmetic; pixel, inverse, distance, bearing, `newpos` and UTM match the C# under mono bit for bit over 676 points |
| HUD | all 24 elements `HUD.cs` paints, from a pure scene builder with a coverage table: horizon and ladder, heading tape with target and course marks, cross-track and turn rate, speed and altitude scrollers, VSI, mode and waypoint, link, battery, GPS, ARMED/DISARMED/SAFE/FAILSAFE, the message line, Vibe and EKF with the C#'s thresholds, Ready/Not Ready to Arm, custom items, flight-path vector and AOA scale |
| Maps | GPU tile rendering; Mission Planner's default `GoogleSatelliteMap` and six of its providers with its URL schemes and version checks, proved against its own `GMap.NET.Core.dll`; overlays; the on-disk cache is Mission Planner's own, so a cache filled by either application is read by both, and it is served by a thread that never waits on the network, with GMap.NET's five fetch threads behind it, so a cached view is on screen at start-up whatever the network is doing |
| CLI | `headless-planner watch \| record \| fly \| params \| param set\|save\|load\|diff \| mission \| survey \| log [bintolog\|dflogtokml\|matlab\|loganalysis] \| logs \| ftp \| fields \| kml \| firmware info\|detect\|list \| terrain \| georef \| ports` |
| GUI | fly, plan, setup, config, params and log screens on gpui; the flight screen's lower-left is Mission Planner's fourteen-page tab control and SETUP/CONFIG are its backstage lists, every entry in the C#'s order under the C#'s conditions; `MainV2`'s port box, baud box and CONNECT/DISCONNECT at the top right, with each network kind's questions and the still-moving check (AUTO's port scan not ported) |
| Porting ledger | `ledger/ledger.csv`, one row per C# file with its tier and state - 63 past `ready` with their evidence and omissions, 76,375 C# lines; `cargo xtask ledger check` fails on anything unaccounted for |
| Flight screen coverage | every one of `FlightData`'s 136 wired actions listed with what stands in for it here — 96 done, 19 missing — in `docs/coverage/flightdata.md`, kept current by a test; the lower-left is Mission Planner's own fourteen-page tab control with its Quick view, its tlog playback, its DataFlash Logs page and log downloader, and its Actions page (Set WP, Restart/Resume Mission, Change Alt/Speed/Loiter Radius, Fly To Coords, Abort Landing, Do Action, Jump To Tag) sends what the C# sends, proved against SITL by a script each; the DataFlash page's conversions run on a thread against the golden files; the HUD's right-click menu has Russian HUD, Ground Color, User Items, Swap With Map, Show icons and Battery Cell Voltage; Set Home/EKF Origin, the camera and gimbal commands, the Transponder page and the speed dial are there too |
| Configuration coverage | every one of the 61 `Config*.cs` panels listed in Mission Planner's SETUP and CONFIG order with what stands in for it here — 27 done, 17 partial, 3 missing, 2 plumbing, 12 dropped at the owner's ruling — in `docs/coverage/configuration.md`, held to the C# by tests; Flight Modes and FailSafe are ported from their `Config*.cs` and proved against SITL |
| Planner coverage and menu | every one of `FlightPlanner`'s 121 wired actions listed the same way — 103 done, 6 missing — in `docs/coverage/flightplanner.md`; the map's right-click menu is Mission Planner's, in its order, with 22 entries working, home is its Home Location boxes written first and drawn as its green pin, the panel's radius and altitude boxes set the C#'s parameters after Write, all proved by a GUI script each |

**Not yet**: any run on Windows or macOS - the repository has no remote, so the three-OS CI matrix has never
executed, and the two columns in `DELIVERABLES.md` say so; the log browser's field descriptions
(`LogMessages.xml.xz`); a joystick latency histogram from a real device (none is attached to this machine);
i18n on the screens (the `.ftl` files exist under `assets/i18n/`, generated from the `.resx` with a zero-loss report;
no screen reads them yet); packaging. `PLAN.md` §13.6 is the queue, re-prioritised on 2026-09-24, and says what
*done* means for each.

## Verification

The port is checked against the original rather than against our reading of it. **The C# source is
in the tree** at `references/missionplanner` and is the specification — a behaviour is ported by
reading the `.cs` file, not by recalling what it probably does. The C# implementation also runs
headless under mono (`tools/csharp-reference/`), and its output is the reference:

- **35,750 frames** of a real ArduPilot flight decode identically to Mission Planner's own
  `MAVLink.dll` — msgid, sequence, ids, payload length, CRC and raw bytes.
- **24,626 field values** across 3,000 messages match field-by-field *by name*, via reflection
  over the C# structs. A field at the right offset with the wrong name round-trips perfectly and
  is still wrong everywhere it is displayed.
- **349 of 349** generated `CRC_EXTRA`, `min_len` and `len` values match the table inside the
  shipped assembly.
- On a second corpus we decode **85 frames the C# parser drops**, with none missed — its reader
  loses sync after a corrupt frame. Every extra frame passes CRC with the correct per-message
  seed.
- **0 heap allocations per packet** from bytes to a published vehicle state, proved by a
  counting allocator: 211,638 frame decodes and 70,546 frames through the real link thread,
  recording on (`crates/*/tests/no_alloc*.rs`). The three message types that allocate by design
  — a parameter arriving, the vehicle speaking, a command acknowledged — are named in the test
  with a bound each, and the test fails if a fourth appears.
- A map tile the real Mission Planner wrote on this machine — in its own
  `gmapcache/TileDBv3/en/<provider>/<z>/<y>/<x>.jpg` layout — reads back byte for byte and
  decodes (`crates/mp-tiles/tests/tilecache.rs`; the test says so and skips where no such cache
  exists).
- **Eleven harnesses** under `tools/csharp-reference/` run the C#'s own code under mono - the
  survey grids and `GridUI`, the four log conversions, projection, `CurrentState`, the UDP client,
  websocket and NTRIP transports, MAVFTP, SRTM, geo-referencing, the planner's handlers - each
  regenerable by its `regen-*.sh`, with the goldens under `testdata/`.

Run it yourself: `tools/csharp-reference/regen.sh` regenerates the corpora, `cargo test` compares.

Beyond the differential corpus: five `cargo-fuzz` targets with committed seed corpora (34 million
executions clean at the last short run, `fuzz/README.md` has the numbers; a 24-hour soak of
`frame_parse` and `message_decode` ends 2026-09-24 16:01Z); a bounded pass over the same properties
on stable in every `cargo test --workspace`; and a smoke test that opens a window and paints,
written into the CI workflow for Linux, Windows and macOS and run on Linux here — the only thing
that exercises a graphics backend rather than merely compiling it. The workflow itself has never
run: there is no remote.

The UI is driven and **checked**, not photographed. `tools/gui-test.sh` runs a script of clicks
and keystrokes against the real binary and asserts on what the application says it believes:

```sh
tools/gui-test.sh tests/gui/waypoint-click.gui -- tcp:127.0.0.1:5760
```
```
  ok   mission.items = 0
clicking 'map@0.45x0.40' (button 1) at window-relative 945,511
  ok   mission.items = 3
```

The facts come from the application itself (`MP_FACTS`, see `crates/mp-gui/src/facts.rs`), so a
feature that quietly stops working fails the run rather than producing a screenshot somebody has
to look at. An expectation naming a fact that no longer exists is an error, not a pass.

## Build and run

```sh
cargo build --workspace
cargo test --workspace

headless-planner watch tcp:127.0.0.1:5760     # ArduPilot SITL
headless-planner watch udp:14550              # bind and wait for a vehicle
headless-planner watch file:flight.tlog       # replay a recording
headless-planner record udp:14550 flight.tlog
headless-planner param save tcp:127.0.0.1:5760 backup.param
headless-planner param diff backup.param proposed.param
headless-planner kml flight.tlog flight.kml
planner                          # the graphical front end
```

The GUI records every flight without being asked, into its own data directory's `logs` —
`~/.local/share/MissionPlannerRust/logs` on Linux (`$XDG_DATA_HOME` when set, and never
`~/MissionPlannerRust`), `Documents\MissionPlannerRust\logs` on Windows - a directory of its own
rather than the C#'s `Mission Planner`, so a user running both loses nothing to the other (see
`crates/mp-settings`). On the first start that finds that directory
missing or empty, Mission Planner's `config.xml`, `poi.txt`, `cameras.xml`, `checklist.xml`,
`warnings.xml`, `UserAlerts.json`, `authkeys.xml`, `logo.png`, `logo.txt` and `History` are
copied into it once and left untouched where they were; the tile cache, terrain, logs and
parameter metadata are not copied, and are fetched or recorded again. Map tiles go to the same
place's `gmapcache`. `MP_NO_RECORD` turns recording off.

Requires a recent stable Rust (see `rust-toolchain.toml`).

## Layout

```
crates/
  mp-mavlink           wire format: framing, checksums, signing
  mp-mavlink-dialects  generated message types (do not edit)
  mp-transport         serial, TCP, UDP, UDP client, websocket, NTRIP, replay, test doubles
  mp-vehicle           decoded state, the snapshot bus, EKF and vibration health
  mp-link              the live link: I/O thread, routing, commands, mission transfer, recording
  mp-params            parameter values and metadata, the downloaded table, .param files
  mp-calibration       accelerometer, compass, radio and motor-test calibration
  mp-ftp               files off the vehicle: dataflash log download and MAVFTP, from MAVFtp.cs
  mp-mission           missions, fences, rally points, survey grids
  mp-log               .tlog reading, dataflash parsing
  mp-tiles             map tile fetching, decoding and caching
  mp-input             joystick and gamepad, mapped to RC channels
  mp-firmware          .apj files and the px4 bootloader protocol
  mp-script            the scripting host API and corpus analysis
  mp-kml               missions and flight paths as KML
  mp-chart             time series for the tuning graph and log plots
  mp-units             typed units and geodesy
  mp-settings          where Mission Planner keeps things on disk, ported from Settings.cs
  mp-terrain           srtm.cs: SRTM tiles, the download queue, getAltitude, proved against the C# DLL
  mp-georef            georefimage.cs: photos matched to a log by time, CAM or TRIG, every output byte for byte to the C#
  mp-fuzz-checks       the fuzz properties, so they compile on stable too
  mp-cli               `headless-planner`
  mp-gui               `planner`, built on gpui
xtask/                 codegen and repository invariants
assets/i18n/           the .ftl per culture, generated from Mission Planner's .resx, with the key map and the zero-loss report
fuzz/                  libfuzzer targets and their committed seed corpora
tests/gui/              click-and-assert UI tests, run by tools/gui-test.sh
tools/csharp-reference headless C# reference for differential testing
testdata/              golden corpora
references/            read-only upstream sources (git-excluded)
    missionplanner/    the C# original — the specification for every ported behaviour
    zed/               gpui
```

## Licence

GPL-3.0-or-later, inherited from Mission Planner. Inbound dependencies are restricted to
GPLv3-compatible licences, enforced by `cargo-deny`.
