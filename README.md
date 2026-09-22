# Mission Planner, in Rust

A complete, file-by-file reimplementation of [ArduPilot Mission
Planner](https://github.com/ArduPilot/MissionPlanner) — 3,678 C# files, 1,208,836 lines, ~93
projects — as a Rust application that is **fast**, **multi-platform** and **GPU-accelerated**.

- **What we are building**: [DELIVERABLES.md](DELIVERABLES.md) — 20 deliverables, each with a
  falsifiable definition of done, the C# paths it replaces, and a numeric target.
- **Progress screenshots**: [docs/progress/](docs/progress/)

## Status

Early. The protocol and telemetry spine works end to end against a real autopilot; the UI is just
beginning.

| Working today | |
|---|---|
| MAVLink v1/v2 codec | zero-copy parse, allocation-free encode, v2 signing |
| Generated dialect | 349 messages, 206 enums, generated from the upstream XML |
| Transports | serial, TCP, UDP, file replay, in-memory test doubles |
| Link engine | I/O thread, multi-vehicle routing, stream requests, commands |
| Vehicle state | lock-free snapshot bus, packet-loss tracking |
| Parameters | full download with gap recovery, typed values, 1,408 from SITL |
| Missions | upload and download, `.waypoints` files, 129-file corpus |
| Logs | `.tlog` read and write; ArduPilot `.BIN` dataflash parsing |
| Geodesy | typed units, Web Mercator, slippy-map tile arithmetic |
| CLI | `mpr watch \| record \| fly \| params \| mission \| ports` |
| GUI | live telemetry, flight path map, primary flight display |

## Verification

The port is checked against the original rather than against our reading of it. The C#
implementation runs headless under mono (`tools/csharp-reference/`), and its output is the
reference:

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

## Build and run

```sh
cargo build --workspace
cargo test --workspace

mpr watch tcp:127.0.0.1:5760     # ArduPilot SITL
mpr watch udp:14550              # bind and wait for a vehicle
mpr watch file:flight.tlog       # replay a recording
mpr record udp:14550 flight.tlog
mpr-gui                          # the graphical front end
```

Requires a recent stable Rust (see `rust-toolchain.toml`).

## Layout

```
crates/
  mp-mavlink           wire format: framing, checksums, signing
  mp-mavlink-dialects  generated message types (do not edit)
  mp-transport         serial, TCP, UDP, replay, test doubles
  mp-vehicle           decoded state and the snapshot bus
  mp-link              the live link: I/O thread, routing, commands
  mp-log               .tlog reading and writing
  mp-units             typed units and geodesy
  mp-cli               `mpr`
  mp-gui               `mpr-gui`, built on gpui
xtask/                 codegen and repository invariants
tools/csharp-reference headless C# reference for differential testing
testdata/              golden corpora
referneces/            read-only upstream sources (git-excluded)
```

## Licence

GPL-3.0-or-later, inherited from Mission Planner. Inbound dependencies are restricted to
GPLv3-compatible licences, enforced by `cargo-deny`.
