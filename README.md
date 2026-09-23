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

Measured on this tree: **17 crates, 40,661 hand-written Rust LOC** (plus 91,634 generated),
**608 tests** green on `cargo test --workspace`, across 88 commits.

| Working today | |
|---|---|
| MAVLink v1/v2 codec | zero-copy parse, allocation-free encode, v2 signing |
| Generated dialect | 349 messages, 206 enums, generated from the upstream XML |
| Transports | serial, TCP, UDP, file replay, in-memory test doubles |
| Link engine | I/O thread, multi-vehicle routing, stream requests, commands |
| Vehicle state | lock-free snapshot bus, packet-loss tracking |
| Parameters | full download with gap recovery, typed values, 1,408 from SITL |
| `.param` files | save, load and compare against a vehicle, honouring the C# skip-list |
| Missions | upload and download, `.waypoints` files, 129-file corpus |
| Logs | `.tlog` read and write; ArduPilot `.BIN` dataflash parsing |
| Flight recording | every connection recorded to a `.tlog`, both directions, shown on screen |
| Health | EKF variances and vibration with ArduPilot's own thresholds, clipping counts |
| Calibration | accelerometer, compass, radio, motor test |
| Joystick | axes to `RC_CHANNELS_OVERRIDE` with a release-on-disconnect failsafe (Linux) |
| Firmware | `.apj` parsing and the px4 bootloader protocol, proven against a mock; nothing flashed yet |
| Scripting | the `Script.cs` host API, and a measurement of what the 19 shipped scripts need |
| KML export | a flown path coloured by flight mode, and a mission, for Google Earth |
| Tuning graph | eleven telemetry fields plotted live, min/max reduced so a spike cannot hide |
| Geodesy | typed units, Web Mercator, slippy-map tile arithmetic |
| Maps | GPU tile rendering, flight path, mission and fence overlays |
| CLI | `mpr watch \| record \| fly \| params \| param \| mission \| survey \| log \| logs \| kml \| firmware \| ports` |
| GUI | fly, plan, setup and params screens on gpui |

**Not yet**: log plotting, waypoint editing on the map, terrain-relative altitudes, satellite
imagery, i18n, packaging. `PLAN.md` §13.2 is the queue, and says what *done* means for each.

## Verification

The port is checked against the original rather than against our reading of it. **The C# source is
in the tree** at `referneces/missionplanner` and is the specification — a behaviour is ported by
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

Run it yourself: `tools/csharp-reference/regen.sh` regenerates the corpora, `cargo test` compares.

Beyond the differential corpus: five `cargo-fuzz` targets with committed seed corpora (34 million
executions clean at the last run, `fuzz/README.md` has the numbers); a bounded pass over the same
properties on stable in every `cargo test --workspace`; and a smoke test that opens a window and
paints on Linux, Windows and macOS in CI — the only thing that exercises a graphics backend rather
than merely compiling it.

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

mpr watch tcp:127.0.0.1:5760     # ArduPilot SITL
mpr watch udp:14550              # bind and wait for a vehicle
mpr watch file:flight.tlog       # replay a recording
mpr record udp:14550 flight.tlog
mpr param save tcp:127.0.0.1:5760 backup.param
mpr param diff backup.param proposed.param
mpr kml flight.tlog flight.kml
mpr-gui                          # the graphical front end
```

The GUI records every flight to `Documents/Mission Planner/logs` without being asked — the same
directory the C# application uses, so a flight recorded by either is found by both. `MP_NO_RECORD`
turns it off.

Requires a recent stable Rust (see `rust-toolchain.toml`).

## Layout

```
crates/
  mp-mavlink           wire format: framing, checksums, signing
  mp-mavlink-dialects  generated message types (do not edit)
  mp-transport         serial, TCP, UDP, replay, test doubles
  mp-vehicle           decoded state, the snapshot bus, EKF and vibration health
  mp-link              the live link: I/O thread, routing, commands, .param files
  mp-mission           missions, fences, rally points, survey grids
  mp-log               .tlog reading and writing, dataflash parsing
  mp-tiles             map tile fetching, decoding and caching
  mp-input             joystick and gamepad, mapped to RC channels
  mp-firmware          .apj files and the px4 bootloader protocol
  mp-script            the scripting host API and corpus analysis
  mp-kml               missions and flight paths as KML
  mp-chart             time series for the tuning graph and log plots
  mp-units             typed units and geodesy
  mp-fuzz-checks       the fuzz properties, so they compile on stable too
  mp-cli               `mpr`
  mp-gui               `mpr-gui`, built on gpui
xtask/                 codegen and repository invariants
fuzz/                  libfuzzer targets and their committed seed corpora
tests/gui/              click-and-assert UI tests, run by tools/gui-test.sh
tools/csharp-reference headless C# reference for differential testing
testdata/              golden corpora
referneces/            read-only upstream sources (git-excluded)
    missionplanner/    the C# original — the specification for every ported behaviour
    zed/               gpui
```

## Licence

GPL-3.0-or-later, inherited from Mission Planner. Inbound dependencies are restricted to
GPLv3-compatible licences, enforced by `cargo-deny`.
