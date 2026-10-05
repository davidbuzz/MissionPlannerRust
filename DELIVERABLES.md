# Mission Planner → Rust: 21 Core Deliverables

**Goal:** a file-complete, code-complete reimplementation of [ArduPilot Mission Planner](https://github.com/ArduPilot/MissionPlanner)
(C# / .NET Framework 4.7.2 / WinForms — 3,678 `.cs` files, 1,208,836 LOC, ~93 `.csproj`) in Rust, that is
**extremely fast**, **multi-platform** (Windows / Linux / macOS), and **GPU-accelerated** end to end.

**UI/runtime stack:** Zed's ecosystem — `gpui` from the zed tree, pinned at `62e5991` (Apache-2.0;
wgpu on Linux and web, Direct3D 11 on Windows, Metal on macOS), with the platform crates `gpui_linux` /
`gpui_windows` / `gpui_macos` selected in `crates/mp-gui/Cargo.toml` (and `gpui_web` as the wasm option).
The crates.io 0.2.2 release was left on 2026-09-23 (commit `feaa408`): an older snapshot on the blade
renderer with no web backend. Source: <https://github.com/zed-industries/zed>.

**Licence:** the port is a derivative of GPLv3 Mission Planner → the workspace ships **GPL-3.0-only** (its
`COPYING.txt` grants no later version). `gpui` (Apache-2.0) is inbound-compatible; every crate's licence is in
`THIRD_PARTY_LICENSES` and held to `deny.toml` by `xtask/tests/licences.rs`; NOTICE is the §5(a) statement.

## Summary

| # | Layer | Deliverable | Priority | Linux | Windows | macOS | Testing |
|---|---|---|---|---|---|---|---|
| [D1](#d1-workspace-crate-graph-and-build-system) | 0 | Cargo workspace and crate graph | P0 | In progress (58% completed est) | Built and run (VM, 2026-09-26) | Not started | Unit |
| [D2](#d2-mavlink-protocol-crate) | 0 | MAVLink protocol codec crate | P0 | In progress (90% completed est) | Not started | Not started | Differential vs C# |
| [D3](#d3-transport-layer) | 0 | Serial, TCP, UDP, BLE transports | P0 | In progress (75% completed est) | Spiked (VM, 2026-09-26: the bench board listed by SetupAPI) | Not started | Differential vs C# |
| [D4](#d4-link-engine-the-mavlinkinterface-equivalent) | 0 | Link engine, protocol machines | P0 | In progress (85% completed est) | Not started | Not started | Differential vs C# |
| [D5](#d5-vehicle-state-model--telemetry-bus) | 0 | Vehicle state snapshot bus | P0 | In progress (90% completed est) | Not started | Not started | Differential vs C# |
| [D6](#d6-ui-kit-on-gpui) | 1 | gpui widget kit | P0 | In progress (35% completed est) | Suite run (VM, 2026-09-27) | Not started | Unit + layout |
| [D7](#d7-gpu-render-core) | 1 | Shared wgpu render core | P0 | In progress (40% completed est) | Ran (VM, 2026-09-26, Direct3D 11) | Not started | Unit + Linux paint smoke |
| [D8](#d8-map-engine) | 2 | GPU slippy map engine | P0 | In progress (65% completed est) | Suite run (VM, 2026-09-27) | Not started | Differential vs C# |
| [D9](#d9-hud--primary-flight-display) | 2 | GPU HUD with video | P0 | In progress (80% completed est) | Suite run (VM, 2026-09-27) | Not started | Unit + SITL |
| [D10](#d10-flight-data-screen) | 2 | Flight Data operations screen | P0 | In progress (90% completed est) | Suite run (VM, 2026-09-27) | Not started | Unit + SITL + hardware |
| [D11](#d11-flight-planner-screen) | 2 | Mission and survey planner | P0 | In progress (90% completed est) | Suite run (VM, 2026-09-27) | Not started | Differential vs C# |
| [D12](#d12-configuration--tuning-screens) | 2 | Parameter config and tuning | P1 | In progress (85% completed est) | Suite run (VM, 2026-09-27) | Not started | Unit + SITL |
| [D13](#d13-initial-setup-calibration-and-firmware) | 2 | Setup, calibration, firmware flashing | P1 | In progress (70% completed est) | Suite run; the bench board flashed (VM, 2026-09-27) | Not started | Unit + SITL + hardware |
| [D14](#d14-log-engine-and-analysis) | 2 | Dataflash log parsing, plots | P1 | In progress (80% completed est) | Suite run (VM, 2026-09-27) | Not started | Differential vs C# |
| [D15](#d15-can-peripherals-and-outboard-features) | 2 | DroneCAN, peripherals, video, joystick | P2 | In progress (70% completed est) | Suite run (VM, 2026-09-27) | Not started | Unit |
| [D16](#d16-extension-and-scripting-system) | 2 | Python scripting, WASM extensions | P2 | In progress (80% completed est) | Suite run (VM, 2026-09-27) | Not started | Unit |
| [D17](#d17-localization-settings-and-data-compatibility) | 2 | i18n, settings, data compatibility | P1 | In progress (60% completed est) | Suite run (VM, 2026-09-27) | Not started | Differential vs C# |
| [D18](#d18-translation-factory-and-porting-ledger) | 3 | Translation factory, file ledger | P0 (ledger) / P1 (factory) | In progress (33% completed est) | Not started | Not started | Unit |
| [D19](#d19-verification-suite) | 3 | Differential, SITL, fuzz verification | P0 | In progress (75% completed est) | Suite run (VM, 2026-09-27) | Not started | Differential vs C# + fuzz |
| [D20](#d20-release-packaging-and-operations) | 3 | Installers, updates, crash reporting | P1 | In progress (65% completed est) | Not started | Not started | Unit |
| [D21](#d21-native-in-process-plugin-host) | 2 | Native in-process plugin host | P3 | Not started (0% completed est) | Not started | Not started | Not started |

**Layer** 0 = foundation (protocol/transport/state) · 1 = rendering and UI foundation · 2 = the application · 3 = the machine that builds the machine.
**Priority** P0 = nothing ships without it · P1 = required for feature parity · P2 = required for 100% completeness, sequenced last · P3 = the last thing of all, after P2.
**Linux / Windows / macOS** the implementation status on each operating system, one column each: Not started → Spiked → In progress → Feature complete → Done. The Linux figure is an estimate of how much of the row's DoD is met, judged from its "Today" paragraph against its DoD and `Tests:` lines, revised at each documentation commit. A deliverable is brought up on Linux first and the other two columns say *Not started* until its `Tests:` artefacts have been run there; a CI matrix for all three exists in `.github/workflows/ci.yml` but has not run yet (no remote), so the Windows column records only what ran by hand in the owner's Windows 10 VM (`win10_vm_setup.md`: built and run 2026-09-26, the whole GUI suite 2026-09-27, the bench board flashed) and nothing is claimed for macOS.
**Testing** Not started → Unit → Differential vs C# → Gated in CI → HIL signed off. *Gated in CI* is claimed
for nothing: the workflow exists and has never run.

**Re-audited 2026-10-03** against the tree at 344c34d (the owner's ask): every row's Linux estimate re-scored from its Today paragraph against its DoD, each Today and Tests line brought to what the tree holds - 28 crates, 422,325 hand-written lines, 4,033 tests, 202 GUI scripts, the ledger's 196 rows past `ready` - and the Windows column set to what ran in the VM. **Audited 2026-09-25** against the tree after the day's merges: each `Today:` and `Tests:` claim, its verdict and its evidence are in `docs/audit-2026-09-25.md`; the Linux estimates were not re-scored. **Revised 2026-09-24**, an audit of every row against the tree. The Linux figures are re-scored clause by
clause against each row's DoD and `Tests:` line, and the `Tests:` lines now name the artefacts that exist
under the names they have, with *not yet* for the rest. Two things changed in the priorities:

- **macOS has never run anything, and Windows ran once** (2026-09-26, in the owner's Windows 10 VM: the
  planner built in 31 minutes and ran against the laptop's SITL - a heartbeat, 1,408 parameters, the flight
  screen through Direct3D 11; `win10_vm_setup.md`). The repository has no remote, so the three-OS matrix in
  `.github/workflows/ci.yml` has never executed, and the paint smoke it describes for Direct3D 11 and Metal
  (`docs/adr/0002`) is a workflow, not a result. Creating the remote is the cheapest single act that moves
  those two columns, and it is the owner's.
- **D18 is split.** The ledger - its rows, states, `check`, and evidence - stays P0 as the definition of
  done. The factory machinery around it - the dispatcher, the per-file agent contract, the DSDL, `.resx` and
  screen-spec generators - is sequenced after D12's pages, at P1: the row queue in PLAN.md §13 with the C#
  oracles and the coverage ledgers has been doing the factory's job, and 251,000 hand-written lines have
  arrived that way. The order of everything else is PLAN.md §13.6.

## Test policy (applies to all 21 deliverables)

**No deliverable is done without a coded test suite.** Every deliverable below carries a `Tests:` line
naming the actual test artefacts that must exist and run in CI. Intent, manual checklists and "we verified
it by hand" do not count; if it is not a program that fails, it is not a test.

| Rule | Requirement |
|---|---|
| Location | `crates/<crate>/src/**` unit tests, `crates/<crate>/tests/**` integration tests, `crates/<crate>/benches/**` criterion benches, `fuzz/fuzz_targets/**` for every parser, `xtask/tests/**` for repo-wide invariants |
| Corpus | Shared golden data in `testdata/` (real tlogs, dataflash logs, `.param`/`.waypoints` files, tile fixtures, recorded attitude sequences), versioned with git-lfs, referenced by path not copied |
| Differential | Wherever the C# original defines the behaviour, the test compares against it — either live via the headless C# harness or against checked-in golden output generated from it |
| No untested code | `cargo llvm-cov` gate per crate; a PR that lowers a crate's line coverage below its floor fails CI |
| No sleeping tests | Deterministic time and executors (gpui's test executor, simulated clocks); no wall-clock sleeps, no flaky retries |
| Perf is a test | Every numeric target in this document has a criterion bench that fails the build on regression beyond its threshold |
| Panics are failures | Telemetry, state and render paths are tested under `#![deny(clippy::unwrap_used)]` plus fuzzing that asserts no panic on arbitrary input |
| Platform matrix | Unit + integration suites run on Windows, Linux and macOS; GPU suites run headless (lavapipe/WARP) plus at least one real-GPU runner |


---

Each deliverable below is a shippable artefact with a falsifiable definition of done (DoD).
D1–D5 are the spine; nothing above them is real until they are. D18–D20 are the machine that makes the
other 18 achievable at 1.2M-LOC scale.

---

## Layer 0 — Foundation

### D1. Workspace, crate graph and build system
Cargo workspace mapping ~93 `.csproj` onto a layered Rust crate graph (protocol → transport → domain →
logs/geo → render → ui-kit → screens → app → extensions → tools), with the reference trees kept read-only.
- **DoD:** `cargo build --workspace` green on Linux/Windows/macOS in CI; clippy + `deny.toml` + rustfmt
  enforced; release profile (LTO=fat, codegen-units=1, PGO hook) and a fast dev profile documented;
  cold `cargo check` under 90 s and incremental under 5 s on the dev box; MSRV and edition pinned;
  GPLv3 + third-party attribution (`about.toml`/`cargo-deny`) generated automatically.
- **Replaces:** `MissionPlanner.sln`, `MissionPlanner.csproj`, `MissionPlannerLib.csproj`, `build*.bat`.
- **Today:** 28 crates and `xtask` build as one workspace; `xtask/tests/graph.rs` holds §5.1's layering
  over `cargo metadata`; clippy, rustfmt and `deny.toml` are enforced; the release profile is LTO fat
  with one codegen unit and the dev profile optimises dependencies; MSRV 1.95.0 and edition 2024 are
  pinned; `cargo xtask` is an alias in `.cargo/config.toml`. **Not yet:** the three-OS CI matrix has
  never run (no remote); no PGO hook; the build budgets measured 2026-10-03 by `tools/build-budget.sh` (16 jobs, the dependencies fetched, nothing else building): cold `cargo check --workspace` 237 s - **over the 90 s asked for**, the dependencies' own checking being most of it (the 2.6 GB check directory is theirs); incremental 4.6 s with a leaf of mp-gui touched and 5.1 s with mp-units (L0) touched, at the 5 s line; attribution
  is NOTICE and THIRD_PARTY_LICENSES (2026-10-03, the owner's licensing decisions: GPL-3.0-only, his header on every `.rs`, the product photographs and the CPOL AGauge code out), the crate table written by `cargo xtask licences` from `cargo metadata`; `xtask/tests/licences.rs` holds every dependency's licence to `deny.toml`'s list as cargo-deny reads an SPDX expression, and `cargo-deny` itself runs from the workflow.
- **Tests:** `xtask/tests/graph.rs` (exists: the layer rules, no upward edge, no cycle, each rule proven able to fail); `xtask/tests/licences.rs` (exists: the header on every tracked Rust file, GPL-3.0-only in both manifests and no Rust file claiming or-later, every dependency admitted by `deny.toml`, the SPDX reader, the crate table current, NOTICE and THIRD_PARTY_LICENSES naming what is carried); the build budget is `tools/build-budget.sh`, not a test - a cold check of the workspace inside `cargo test` would be four minutes under the target directory's lock, so the figures are measured by the script and recorded here; no proc-macro crate exists, so no `trybuild`; the CI matrix builds all three OSes on every PR once there is a remote for it to run on.

### D2. MAVLink protocol crate
Generated message set (common + ardupilotmega + all dialects Mission Planner ships), MAVLink v1/v2,
signing, zero-copy frame parse/serialize.
- **DoD:** generated from the same XML the C# side uses, checked in as generated output + regenerable by
  `cargo xtask codegen`; round-trip property tests (proptest) and a `cargo-fuzz` target with 24 h clean;
  decodes a 1 GB tlog at **> 1 M messages/s single-threaded**, zero heap allocations per packet
  (verified by an allocation-counting test).
- **Replaces:** `ExtLibs/Mavlink` (40,718 LOC, machine-generated).
- **Today:** the allocation claim is tested. `crates/mp-mavlink/tests/no_alloc.rs` installs a
  counting allocator and replays every frame of every recorded flight through the framing and
  the typed decoder: 211,638 decodes, zero allocations, the same in release. The 24-hour soaks of
  `frame_parse` and `message_decode` ran 2026-09-23 16:01Z to 2026-09-24 16:01Z and ended clean:
  30.7 billion and 4.1 billion executions, no crash, no timeout, no out-of-memory; `frame_parse`
  stayed at its 90 edges all day, `message_decode` grew from 13,419 to 13,782 (`fuzz/README.md`).
  D2's 24-hour clause is met.
  `benches/decode.rs` measures the frame decoder over a five-message mix as ArduPilot sends it:
  **9.5 M frames/s** whole-buffer and in 64- and 256-byte chunks, 10.2 M in 1,024-byte chunks
  (criterion, release, 2026-09-24, with the fuzz soak on two other cores) - nine times the
  target for the framing and CRC, on one core. The typed decode of every message is
  `tests/no_alloc.rs`'s 211,638 and is not timed separately.
- **Tests:** `crates/mp-mavlink/tests/roundtrip.rs` (proptest encode→decode identity over every generated message type); the golden decode is `tests/differential_tlog.rs` (35,750 frames against `MAVLink.dll` under mono) with `crates/mp-mavlink-dialects/tests/differential_fields.rs` (24,626 field values by name) and `reference_table.rs` (349 `CRC_EXTRA`/`min_len`/`len` against the shipped assembly); `tests/signing.rs`; the truncation cases live in `tests/robustness.rs` and `tests/decoder.rs`; `fuzz/fuzz_targets/frame_parse.rs` and `message_decode.rs` (24 h soak clean, `fuzz/README.md`); `crates/mp-fuzz-checks/tests/bounded.rs` runs every fuzz property on stable in `cargo test --workspace`; `tests/no_alloc.rs`; `benches/decode.rs` (9.5 M frames/s, recorded above).

### D3. Transport layer
`serial | TCP | UDP | BLE | NTRIP | websocket | file-replay`, device enumeration and hotplug on all three
OSes, plus bootloader/flashing transports (px4uploader, DFU, ADB).
- **DoD:** trait-based transport with a deterministic in-memory/replay implementation; serial enumeration
  returns identical device lists to the C# app on the same hardware (Windows COM, Linux `/dev/serial/by-id`,
  macOS `cu.*`); reconnect and surprise-unplug covered by tests; NTRIP/RTCM injection verified against a
  live caster; **≤ 1 ms** added latency over raw OS read.
- **Replaces:** `ExtLibs/Comms` (8,249), `ExtLibs/Ntrip`, `ExtLibs/Zeroconf`, `ExtLibs/px4uploader`,
  `ExtLibs/NetDFULib`, `ExtLibs/WinUSBNet`, `ExtLibs/SharpAdbClient`, `Radio/`, `SikRadio/`.
- **Today:** serial, TCP, UDP, file replay and the in-memory mock, behind one trait. Port
  enumeration is `CommsSerialPort.GetPortNames` ported rule for rule (`crates/mp-transport/src/enumerate.rs`)
  and held to Linux, macOS and Windows fixtures in `tests/enumerate.rs`; `tests/faults.rs` runs real
  frames through every fault the DoD names and re-checks each delivered checksum; `tests/hotplug.rs`
  unplugs a real `SerialTransport` over a pty and reopens it. The UDP client
  (`CommsUDPSerialConnect.cs`), the websocket client (`CommsWebSocket.cs`: RFC 6455 framing and
  the socket.io conversation, in-crate) and NTRIP (`CommsNTRIP.cs`: the v1/v2 requests, Basic
  auth, the SOURCETABLE checks, the GGA sentence sent every 30 s, the reconnect limit) are ported
  as `udpcl:`, `ws://` and `ntrip://` links and held to the three C# classes compiled straight from
  the reference tree and run against peers on 127.0.0.1 (`tools/csharp-reference/MpComms.cs`,
  `testdata/comms/golden/`: the exact request bytes, 13 GGA sentences, the whole websocket
  conversation, the UDP client's reads, writes and counts) - PLAN.md §13.4 row 43. Serial on real hardware:
  a Cube Orange on `/dev/ttyACM0` (`27B1:0004`, listed by `headless-planner ports` under its by-id name) gave
  341 frames in 8 s with no CRC error on 2026-09-24. **Not yet:**
  BLE, TLS (`wss://`, NTRIP over https), DFU and ADB (the px4 bootloader runs over the serial
  transport, D13); BLE is not in the C# and is ruled out (NOT_DONE_YET_MATRIX.md); the ≤ 1 ms latency is measured (below). Windows port names are SetupAPI's hardware id and bus-reported name as `Win32DeviceMgmt` reads them (`crates/mp-transport/src/win32.rs`, 2026-09-26: the bench CubeOrange listed and detected in the VM as board id 140).
- **Tests:** the per-transport loopbacks are `crates/mp-transport/tests/transports.rs`, `integration_replay.rs`, `udp_client.rs` and `websocket.rs`, with `csharp_goldens.rs` holding the UDP client, websocket and NTRIP bytes to the C# classes run under mono; `tests/faults.rs` (drop, duplicate, reorder, partial write, mid-frame disconnect over real frames); `tests/enumerate.rs` over Linux, macOS and Windows fixtures; `tests/hotplug.rs` (a real pty unplugged and reopened); `tests/ntrip.rs` against an in-process caster; `tests/description.rs`; `benches/latency.rs` (a one-byte poke answered by 64 bytes, two thousand round trips through the OS's own socket or port and then through the transport: over TCP on loopback the transport adds nothing measurable at p99 - raw p99 50.6 µs, transport 44.6 µs - and over a pseudo-terminal 116 ns - raw p99 52.4 µs, transport 52.5 µs - against D3's 1 ms; release, 2026-09-24, gated).

### D4. Link engine (the `MAVLinkInterface` equivalent)
Per-link packet pump, routing/forwarding, and the high-level protocol state machines: parameters,
mission/rally/fence up- and download, MAVFTP, log download, command_long/ack, requests and retries.
- **DoD:** every protocol state machine is an explicitly-tested state machine (not ad-hoc retry loops);
  multi-vehicle `sysid/compid` routing with N ≥ 50 simultaneous vehicles; full param download from a real
  ArduPilot SITL matches the C# app's result set exactly; packet loss/timeout behaviour covered by a
  fault-injection replay harness.
- **Replaces:** `ExtLibs/ArduPilot/Mavlink/*` (MAVLinkInterface, MAVState, MAVList).
- **Today:** the machines are explicit and tested under fault. `crates/mp-link/src/timeouts.rs` is the
  C#'s retry table (`MAVLinkInterface.cs:1748-4380`) in one struct; `requests.rs` retries parameter
  sets, reads, commands and set-current as `setParamAsync`/`GetParamAsync`/`doCommandAsync`/
  `setWPCurrentAsync` do, `param_download.rs` is `getParamListAsync`'s whole-list-then-holes recovery,
  and `mission_transfer.rs` handles every `MAV_MISSION_RESULT` as `mav_mission.cs` does. `tests/retries.rs`
  counts every send on the wire under timeouts, reordering, duplicates and seeded bad links (44 tests);
  `tests/routing.rs` runs 50 systems and 56 components through one link. The GUI's and the CLI's
  sets and commands go through those requests, with the C#'s message texts on the status line
  (PLAN.md §13.4 row 11). MAVFTP is `MAVFtp.cs` whole (row 36): the burst read with its gap filling, list, upload, remove, rename, CRC32, the C#'s retry table, as a state machine in `mp-ftp` the link drives, with `headless-planner ftp`; the C#'s own `MAVFtp` under mono gives the same request bytes and the same `param.pck` from SITL. On a real Cube Orange over USB
  (2026-09-24): 929 parameters downloaded and reported complete, `@SYS` listed over MAVFTP, six
  logs listed. `doCommandInt`, `setWP` for one item and `getHomePosition` are requests too
  (PLAN.md §13.6 row 74): a `COMMAND_INT` waits for its ack with `doCommandAsync`'s three retries
  and none of its `IN_PROGRESS` patience, a press's `MISSION_ITEM` waits 450 ms ten more times
  for the ack or the request for the next item, and `GET_HOME_POSITION` asks again three times
  700 ms apart for a `HOME_POSITION` - so Change Alt, ArduPlane's guided target, Format SD, the
  scripting commands, Set Home Here and the camera's `COMMAND_INT`s no longer go out once and
  raw. Parameters are fetched as `getParamListMavftp` fetches them (PLAN.md §13.6 row 81):
  `@PARAM/param.pck?withdefaults=1` over MAVFTP first, the stream as the fallback, started once a
  vehicle is heard with nothing held (`tests/param_fetch.rs`, through the real link thread). The
  reboot goes out twice and unwaited, as `doCommand` sends it (row 86). **Not yet:** the
  log-download machine as a request; `uploadPartial`, which only the Dowding plugin calls in the C#.
- **Finished since the audit** (moved from NOT_DONE_YET_MATRIX.md on 2026-10-03, which holds only unfinished work from that day):
  - **crates/mp-link/src/lib.rs** (high): A connected link must stay connected (owner's bug report 2026-10-03): after a vehicle or SITL reboot, a dropped TCP connection or a serial device that vanished, the link is to be kept, reconnected aggressively and shown connected throughout, with no dialog box during the reconnection - Mission Planner's own main link does not reconnect (only its mirror and NMEA TcpSerial streams do, CommsTCPSerial.cs doAutoReconnect), so this is the owner's ruling, PLAN.md section 12 D23; done 2026-10-03: the link thread keeps its vehicles, parameters and messages and opens its transport again by URL every second (reopen_transport, RECONNECT_INTERVAL) until it is back or DISCONNECT is pressed - a replay and a listening link end as before - the status line saying 'reconnecting, try N', nothing asked; a TCP connect is capped at five seconds so the thread stays stoppable; crates/mp-link/tests/reconnect.rs (the vehicle kept across the loss, the refusals counted, a close not waiting for the next try) and link-reconnect.gui against tests/gui/dropping-vehicle.py passed headless
- **Tests:** the machines are exercised by `crates/mp-link/tests/retries.rs` (44 tests: every send counted under timeouts, reordering, duplicates and seeded bad links) and `tests/link.rs` against an in-memory vehicle, with the SITL halves behind `--ignored` in `params_sitl.rs`, `mission_sitl.rs`, `fence_sitl.rs`, `commands_sitl.rs` and `logs_sitl.rs`; MAVFTP is `crates/mp-ftp/tests/mavftp.rs` and `csharp.rs` (768 names, the CRC vectors and 13 payloads against `MAVFtp.cs` under mono); `tests/routing.rs` (50 systems, 56 components through one link); `tests/param_fetch.rs` (MAVFTP first, the stream when the file is refused); `tests/no_alloc_ingest.rs`, `replay_clock.rs`, `current_state.rs`, `telemetry_storm.rs`, `traffic.rs`, `packet_in.rs`, `compassmot.rs`, and `resume_sitl.rs` behind `--ignored`. Not yet: a log-download machine as a request, and `params_sitl.rs` does not diff the downloaded set against the C# application's dump.

### D5. Vehicle state model + telemetry bus
The `CurrentState` equivalent: decoded, UI-facing vehicle state, published as lock-free immutable
snapshots so the renderer never blocks on the I/O thread.
- **DoD:** single-writer/multi-reader snapshot (arc-swap / triple-buffer) with **zero locks on the render
  path**; no allocation in the ingest→state path; every C# `CurrentState` field accounted for, with a
  checked-in field-coverage report; unit-typed geodesy (no bare `f64` lat/lon) throughout.
- **Replaces:** `ExtLibs/ArduPilot/CurrentState.cs`, the C# event/timer/`Invoke` marshalling model.
- **Today:** `crates/mp-vehicle/tests/no_alloc_ingest.rs` and `crates/mp-link/tests/no_alloc_ingest.rs`
  prove zero allocations per packet from a transport read to a published state, over 70,546 real
  frames through the real link thread with recording on. `PARAM_VALUE`, `STATUSTEXT` and
  `COMMAND_ACK` allocate by design and are listed with a bound each. `Transport::description()`
  returns `&str` and the publish path allocates nothing (71,500 allocations over 35,750 publishes
  before; the tests demand zero now). The field-coverage report exists: `crates/mp-vehicle/src/coverage.rs` accounts for all
  550 public members of `CurrentState` (**471 done, 48 derived, 0 missing, 30 plumbing, 1 dropped**),
  matched to the C# file by a test, rendered to `docs/coverage/currentstate.md`; the porting that
  took it from 219 done added SYS_STATUS, GPS, battery, radio, wind, terrain, rangefinder, servo,
  high-latency and the onboard subsystems with the C#'s rules and quirks kept, and the last 55
  (the clock, the once-a-second counts, the battery integration, HIL channels, custom fields,
  gimbal/tracker/base, stream rates, K-index, speedup) are held per packet to the C#'s own
  `UpdateCurrentSettings` under mono over three tlogs (`tools/csharp-reference/MpState.cs`,
  PLAN.md §13.4 row 38) - the differential the `Tests:` line asks for, for those fields. Their
  callers in the link and the GUI are row 39.
- **Tests:** the field coverage is `crates/mp-vehicle/src/coverage.rs`'s tests, matched to `CurrentState.cs` by name, type, display text, group and order, rendered to `docs/coverage/currentstate.md` and failing when stale; the state timeline against the C# is `tests/current_state_oracle.rs` (58 fields per packet over three tlogs from `MpState.cs` under mono) with `current_state_replay.rs`, `current_state_clock.rs`, `current_state_statics.rs`, `replay_state.rs`, `onboard.rs`, `modes.rs`, `nav.rs`; `tests/no_alloc_ingest.rs` in both `mp-vehicle` and `mp-link`; `benches/snapshot.rs` (publish 153 ns and load 25 ns at steady state; the gate is a writer at 1 kHz under eight readers loading without pause for a second: publish p99 26.7 µs against §8.2's 200 µs, load p99 899 ns against 20 µs, 0 allocations in 1,000 publishes; release, 2026-09-24). Not yet: a `loom` model of the publish path.

---

## Layer 1 — Rendering and UI foundation

### D6. UI kit on gpui
The widget vocabulary 135 screens need: dense forms, labelled fields, combo/spin/slider/toggle,
data grids (the `ObjectListView` replacement), trees, tabs, docking/panels, modals, toasts, status bars,
theming (`*.mpsystheme` import) and keymaps/actions.
- **DoD:** every widget has a gallery example and a snapshot test; dark/light themes; HiDPI + multi-monitor
  + IME verified on all three OSes; a 100,000-row virtualised grid scrolls at **120 fps**; the kit is behind
  our own thin façade so a gpui API break is a one-crate fix.
- **Replaces:** `ExtLibs/ObjectListView` (42,184), `ExtLibs/BSE.Windows.Forms` (10,323),
  `ExtLibs/CsAssortedWidgets` (10,742), `ExtLibs/Controls` (17,531), `Controls/` chrome, `ThemeManager.cs`.
- **Today:** the widgets exist inside `mp-gui`, made as the screens needed them: a single-line text
  field, drop-down lists, check boxes, buttons, the fourteen-page tab control, the backstage list,
  modal prompts and questions as the C#'s `InputBox` and `MessageBox` put them, a virtual grid over
  the log's index (`logbrowse/grid.rs`), the status line and a scroll strip; `crates/mp-gui/tests/layout.rs`
  holds every screen inside the window. **Not yet:** no `mp-ui` crate or facade - `mp-gui` is 249,000 lines in one crate and names gpui's platform crates directly, §5.1's one pinned exception;
  no widget gallery or snapshot tests; no themes or `*.mpsystheme` import (the palette is the
  owner's dark one, PLAN.md §1.2); the numeric up-down (`MavlinkNumericUpDown`, typed or stepped by its arrows) lives in `config/servo_output.rs`, shared by the pages rather than a kit; the text field (`textfield.rs`) has a caret, a selection by keyboard, click, drag and double-click, and the clipboard chords, with a multi-line `text_area` for the log browser's txt_info and User Params' Modify box, but no undo, right-click menu, sideways scrolling or blinking caret, and fourteen pages still draw their own type-at-the-end boxes (PLAN.md §13.6 row 87); HiDPI,
  multi-monitor and IME unverified; the 100,000-row grid at 120 fps unmeasured.
- **Tests:** `crates/mp-gui/tests/layout.rs` (every screen inside the window at 1600×1200 and at a small size; `--ignored`, they need a window) and the 202 `tests/gui/*.gui` scripts through `tools/gui-test.sh` (every one has run: the last full passes were Linux 2026-09-26 and the Windows VM 2026-09-27, the 25 written since each passed headless at their commit), which assert the application's own facts; the widgets' unit tests are inline in `mp-gui`. Not yet: `tests/snapshots/**` with golden images, `tests/interaction.rs` on gpui's test executor, `tests/grid.rs` and `benches/grid_scroll.rs`, `tests/theme_import.rs`, `tests/hidpi.rs`.

### D7. GPU render core
Shared `wgpu` layer under everything visual: device/queue sharing with gpui (or offscreen render-to-texture
if gpui refuses to share — **decision gate, see PLAN.md**), render graph, instanced markers, GPU line
tessellation, glyph atlas labels, offscreen targets, frame pacing, and a software/remote-desktop fallback.
- **DoD:** a custom wgpu viewport composites correctly inside a gpui window on Windows, Linux (X11 +
  Wayland) and macOS; documented per-frame budget with a live profiler overlay; degrades gracefully under
  RDP/VNC and on llvmpipe; headless rendering works in CI for snapshot tests.
- **Replaces:** GDI+/`System.Drawing`, `OpenTK`/`GLControl`, `SkiaSharp`, `ExtLibs/MissionPlanner.Drawing`
  (17,602), `ExtLibs/SvgNet`, `ExtLibs/LibTessDotNet`.
- **Today:** `MP_SMOKE=1` makes the real binary exit 0 once it has painted three frames, and
  `.github/workflows/ci.yml` runs it on Linux (xvfb + llvmpipe), Windows (Direct3D 11, WARP) and
  macOS - a workflow that has never run, since the repository has no remote; on this machine the
  Linux path paints every day. `docs/adr/0001` settled the gate the other way from the DoD's
  wording: the map lives inside gpui through `canvas()` with no custom wgpu pass, at 2.7 ms for a
  decimated 100,000-point track, so `mp-render` was never made and every visual - tiles, tracks,
  markers, the HUD, the charts - is gpui primitives. See `docs/adr/0002` for what the Windows
  smoke does and does not prove. **Not yet:** a profiler overlay and per-frame budget, headless
  rendering for snapshot tests, the degraded-target (RDP/VNC) check, and any run at all on macOS; Windows has run in the owner's VM (2026-09-26, Direct3D 11; the GUI suite there 2026-09-27), never in CI.
- **Tests:** the paint smoke (`MP_SMOKE=1`, in the workflow for all three OSes, run on Linux here). Not yet: `tests/headless.rs` golden renders on lavapipe and WARP, `tests/shaders.rs` (no shader of ours exists; everything is gpui's), `tests/viewport_composite.rs` (the ADR's measurement was a one-off spike, not a regression test), `benches/frame.rs`, `tests/fallback.rs`.

---

## Layer 2 — The application

### D8. Map engine
Slippy map: tile providers + on-disk cache + async decode, projections, raster and vector overlays,
markers, tracks, polygons, geofences, survey grids, and full editing interaction (drag, snap, rubber-band).
- **DoD:** pans/zooms at **120 fps with a 1 M-point track + 10 k markers** visible; tile cache compatible
  with (or migratable from) the existing Mission Planner cache; offline mode; identical provider list to
  the C# app; projection round-trips verified against ProjNet/GDAL to **< 1 mm**.
- **Replaces:** `ExtLibs/GMap.NET.*` (33,899), `ExtLibs/Maps` (7,992), `ExtLibs/ProjNet` (10,212),
  `ExtLibs/GeoUtility` (8,006), `ExtLibs/GDAL`, `ExtLibs/GeoidHeightsDotNet`.
- **Today:** GPU tile rendering with Mission Planner's default provider (`GoogleSatelliteMap`) and
  six of its providers in its order and names - Google road/satellite/terrain, Bing road/satellite/
  hybrid, OpenStreetMap - with the C#'s URL schemes, version checks and `Referer`s, proved against
  the shipped `GMap.NET.Core.dll` under mono (`crates/mp-tiles/tests/providers.rs`); and the on-disk
  cache in Mission Planner's own layout — `gmapcache/TileDBv3/en/<Name>/<z>/<y>/<x>.jpg`, ported from
  `ExtLibs/Maps/MyImageCache.cs` and proved against a tile the C# application wrote
  (`crates/mp-tiles/tests/tilecache.rs`); offline mode serves the cache. **Not yet:** the `redb`
  index; the other 59 of the C#'s 66 providers (hybrids need a two-layer map, some need keys or other projections; the seventh here is `Custom`, the cache-only provider Inject Custom Map fills, 2026-10-03); two providers that are not Mission Planner's (OpenTopoMap, Esri World
  Imagery) remain, last in the list, pending the owner's call; editing beyond click-to-add and
  drag. **Projection proved:**
  `crates/mp-units/tests/projection.rs` holds Web Mercator, `GetDistance`, `GetBearing`, `newpos`
  and (in `mp_mission::utm`) `utmpos` to what the C# itself returns under mono over 676 points
  (`testdata/projection/`), bit for bit where GMap exposes the value and to the identical whole
  pixel at zooms 1-30 where it does not; the round trip is < 1 mm (worst 5.8 nm). Four geodesy
  divergences from the C# were found by it and fixed. `benches/pan_zoom.rs` measures the
  following frame at p99 5.42 ms with a 1 M-point track and 10 k markers, inside the 120 fps
  budget on the CPU side, and gates it; the GPU half is not measured. **Start-up from a cache:**
  the store reads the disk on a thread that never waits on the network and hands what the cache
  lacks to a pool of five fetch threads, GMap.NET's `GThreadPoolSize` (`Core.cs:62`); one thread
  did both before, so one uncached tile at the first view held every cached tile behind it for
  the fetch's timeout and a full cache took five to ten seconds to appear (owner's report,
  2026-09-24). `crates/mp-tiles/tests/startup.rs` holds a cached tile to under 500 ms with every
  fetch thread hung; `tests/gui/tiles-startup.gui` holds the screen to it with a hole in the cache
  and a proxy that never answers.
- **Finished since the audit** (moved from NOT_DONE_YET_MATRIX.md on 2026-10-03, which holds only unfinished work from that day):
  - **crates/mp-gui/src/mapview.rs** (high, found 2026-10-05 in the GUI suite, fixed the same day): hovering a map marker whose tooltip runs over several lines - a CAMERA_FEEDBACK photo's "Photo / Alt / No / Roll" - ended the planner, gpui panicking on a newline in a line it shapes; the report it left in the shared data folder then put "Send Error" over every later script that used that folder. The tooltip is drawn as GMapRoundedToolTip draws it: the lines one under another, each centred, the box as wide as the widest. Tests: a_tooltips_lines_are_split_where_drawstring_breaks_them; fly-camera.gui then fly-changealt.gui on one SITL, the sequence that crashed, with no crash report.
  - **crates/mp-gui/src/mapview.rs** (high): With several vehicles the picker above the flight map re-fits the map to everything drawn, and a track or line to latitude 0, longitude 0 - a vehicle without a fix - zooms it out to half the planet (owner's bug report 2026-10-03); Mission Planner draws no route point and no line at 0,0 (FlightData.cs's `cs.lat != 0 && cs.lng != 0` guards), so 0,0 is to be left out of the render, the fit and every line; done 2026-10-03: mapview.rs is_fixed - latitude and longitude both non-zero, the C#'s test - applied to every layer (the vehicle's marker and route, the mission, home, rally, fences, the fence return, the polygon, traffic, the guided and tracker markers, the KML shapes, the photos) and to Zoom to Mission, so the fit frames only what is drawn; a vehicle without a fix loses its marker and keeps its route (map.vehicle.drawn); unit tests over every layer and the fit, map-zero-position.gui against tests/gui/zero-vehicles.py passed headless. Left for the owner's word: the picker still refits to everything drawn where Mission Planner keeps the zoom and pans (updateMapPosition)
  - **crates/mp-gui/src/mapview.rs** (high): The flight map drew the vehicle as a red arrow (owner's bug report 2026-10-02: "looks like a mouse cursor, not like a quadcopter or plane or rover"); now Mission Planner's marker for the vehicle's type - the quad's motors and arms with its sysid, the plane's coloured outline, the rover, boat, heli, sub and single copter icons turned to the heading, the tracker's icon - with the heading, course, nav bearing and target lines and the Planner page's GMapMarkerBase settings, as Common.getMAVMarker and ExtLibs/Maps/GMapMarker*.cs have them; fly-map-marker.gui passed headless against the SITL copter (a quad, sysid 1, heading, course and target lines)
- **Tests:** `crates/mp-units/tests/projection.rs` (676 points against the C# under mono, < 1 mm round trip); `crates/mp-tiles/tests/tilecache.rs` (hit, miss, corrupt-entry recovery, and a tile the C# application wrote), `offline.rs`, `providers.rs` and `versions.rs` (against the shipped `GMap.NET.Core.dll`), `urlcache.rs`, `wire.rs` (what leaves the machine for a Bing map), `startup.rs` (a cached tile in under 500 ms with every fetch thread stuck; its offline case raced once under load, PLAN.md §13.6 row 85); `crates/mp-units/benches/pan_zoom.rs` (the following frame at p99 5.42 ms with a 1 M-point track and 10 k markers, CPU side); the map's interaction through the `tests/gui/plan-*.gui` scripts, `cache-compat.gui`, `map-provider.gui` and `tiles-startup.gui`. Not yet: `tests/overlays.rs` golden images, `tests/editing.rs` on the test executor (rubber-band and snap are not ported), the GPU half of the frame budget.

### D9. HUD / primary flight display
GPU artificial horizon, tapes, compass, gauges, warnings, with live video underlay and OSD-style overlays.
- **DoD:** pixel-comparable to the C# HUD (side-by-side review signed off), **< 16 ms packet-to-pixel**
  at the 99th percentile, runs at 120 fps while using < 3 % CPU; video underlay with hardware decode.
- **Replaces:** `Controls/HUD*.cs`, `Controls/` PFD widgets.
- **Today:** all 24 elements of `doPaint()` are ported and drawing, the flight-path vector and
  AOA scale from `AOA_SSA` and `AOA_CRIT` since PLAN.md §13.4 row 41 (`hud::Status::Blocked` is
  constructed by nothing now).
  `crates/mp-gui/src/hud.rs` builds a pure scene from the vehicle state - the
  geometry of `HUD.cs doPaint()` with its constants (`Height / 30` font, `Height / 65` per
  degree of pitch, a `Height / 14` heading tape, `Width / 10` scrollers) - and paints it on a gpui
  canvas. All 24 elements draw, in a coverage table a test holds to the code; `tests/gui/hud.gui`
  asserts the drawn set against the real application on SITL; the pictures of `displayicons`
  (`HUD.cs:2861-2899, 3150-3301`) as drawn stand-ins, the cell-voltage, Bat2 and GPS2 lines, and
  every readout in the display units the C#'s `CurrentState` getters multiply by, with the unit
  names (PLAN.md §13.4 row 41). Golden frames since PLAN.md §13.6 row 75: the scene drawn
  headless by a software rasteriser (`hud/raster.rs`: 4 x 4 coverage samples, gpui's fill and
  stroke rules, a fixed bitmap font) and held to 18 images in `testdata/hud/` - three six-frame
  sheets of `autotest.tlog` played through the flight screen's own `hud::live_inputs` (hardest
  roll, hardest pitch, arming with ARMED up and gone) and ten hard cases (level, banked, pitch
  ±90°, inverted, NaN attitude, NaN readouts, a lost fix as text and as its picture, no vehicle,
  and since then an infinite heading, speed and altitude and the camera frame with the instruments
  on and off);
  `HUD_UPDATE_GOLDENS=1` redraws them. Packet-to-pixel is measured under `MP_STORM`: the storm's
  link stamps each packet's arrival into the snapshot (`VehicleState::packet_in`, off on every
  product link) and `storm.rs` times it to the frame's present, as `storm.latency.*`; measured 2026-09-26 on the
  release build with nothing else building: p50 9.8 ms, p99 14.7 ms, max 18.4 ms, 2 of 569 over
  16 ms, once the link's snapshot cadence went from 20 ms to 5 ms (with 20 ms: p99 25.3 ms). The video underlay is a V4L2
  camera's frame (`crates/mp-video`, PLAN.md §13.6 rows 83-84), drawn under the scene with the sky
  and ground left out as `HUD.cs:1986-2013` does, or alone when Enable HUD Overlay is off; decoded
  on the CPU (MJPEG through `image`, YUYV), not in hardware, Linux only, `config-video.gui` unrun.
- **Finished since the audit** (moved from NOT_DONE_YET_MATRIX.md on 2026-10-03, which holds only unfinished work from that day):
  - **crates/mp-gui/src/fly.rs** (low): Record HUD to AVI and Stop Record, ported 2026-10-03: the HUD menu's Video entries over ExtLibs/Utilities/AviWriter.cs in crates/mp-video/src/avi.rs - the notice, <log folder>/<yyyy-MM-dd HH-mm-ss>.avi, the display's frame every 40 ms as the software rasteriser's JPEG at quality 50 with the headers rewritten each time, "Recording" and "Start Recording" on the entry, the index at the close; fly-hud-avi.gui passed headless
  - **crates/mp-gui/src/hud** (low): HUD latency figure owed a measured run: measured 2026-09-26 on the release build with nothing else building - packet-to-pixel p50 9.8 ms, p99 14.7 ms, max 18.4 ms, 2 of 569 over 16 ms at the link's 5 ms snapshot cadence (PLAN.md row 75, DELIVERABLES.md's table, tests/gui/storm.gui's header); the row was not moved when the run was made. A re-run on today's build is a desktop run (the GPU display, on the owner's word), and its result goes in the same three places
- **Tests:** the coverage table in `crates/mp-gui/src/hud.rs`, held to the code by its unit tests; the golden frames in `crates/mp-gui/src/hud/golden.rs` (every case against its golden, no golden without a case, and proofs that a one-pixel move, a changed digit and a changed colour each fail the comparison); the latency clock in `crates/mp-gui/src/storm.rs` and the stamp in `crates/mp-link/tests/packet_in.rs`; `tests/gui/hud.gui`, `hud-health.gui`, `hud-units.gui`, `hud-icons.gui` and `hud-cells.gui` on SITL, and `tests/gui/storm.gui` for `storm.latency.p99 < 16`; `crates/mp-video`'s unit tests over a fake device and the `camera_overlay_on`/`_off` goldens. Not yet: `tests/video_underlay.rs` against a real device, hardware decode.

### D10. Flight Data screen
The live operations screen: HUD + map + quick view + tuning graph + actions + messages + status tabs,
servo/RC, and the vehicle action buttons.
- **DoD:** every tab, button and action of the C# `GCSViews/FlightData` present and behaviourally verified
  against SITL; layout persists; no UI stall > 8 ms during a 200 Hz telemetry storm.
- **Replaces:** `GCSViews/FlightData*` and its dependents.
- **Today:** the coverage list exists and is honest: `crates/mp-gui/src/coverage.rs` has one row
  per event wiring in `FlightData.Designer.cs` (136), naming the control, its text, its handler
  and what this application has for it. At its first count 27 were on the flight screen (arm/disarm, modes, take-off,
  fly-to-here, auto-pan, the map, the tuning graph, the joystick, the HUD's health indicators),
  2 are elsewhere (tlog replay as a link URL, `headless-planner kml`), 19 are WinForms plumbing, 1 is dropped
  (undock, in a single window), and 87 were missing - the transponder, gimbal and camera, video,
  scripts, tlog playback controls, POIs, set-home/EKF-origin, change alt/speed/loiter, set WP,
  quick-view field choice, HUD menu items, log conversions. `docs/coverage/flightdata.md` is the
  rendered list, and a test fails when it is stale or when a claimed id leaves the source.
  Since then the Actions tab is ported in its own 5×5 arrangement - Set WP, Restart and Resume
  Mission, Change Alt/Speed/Loiter Radius, Fly To Coords, Fly To Here Alt, Abort Landing, Set
  Home Alt, Do Action with the C#'s 19 entries - each sending what `FlightData.cs` sends and proved by a `tests/gui/fly-*.gui` script (Resume Mission's asserts the C#'s own outcome on ArduCopter, which refuses a take-off once airborne, row 67), inside the C#'s fourteen-page tab control under the HUD; the Quick view with its field chooser, the Telemetry Logs page's paced playback, POIs, the DataFlash Logs page with the log downloader, and the EKF and Vibration windows behind the HUD's texts (rows 18, 22); then the DataFlash page's four conversions (Convert .Bin to .Log, Create KML + gpx, Create Matlab File, Auto Analysis: `FlightData.cs:1082-1098, 1135-1202, 1311-1378`) each on its own thread against the golden files, the HUD's right-click menu (`FlightData.Designer.cs:458-566`: Russian HUD, Ground Color, User Items with the "Hud Header" prompt, Swap With Map; Battery Cell Voltage, Show icons and the Video entries dimmed with their reasons), and Jump To Tag on the map menu (`FlightData.cs:6504-6531`, sent as `DO_JUMP_TAG`) (row 28); then Set Home Here and Set EKF Origin Here with the ground height from `mp-terrain`, Point Camera Here/Coords and Trigger Camera, Clear Track, Message, Set Mount, the POI files, Customize and MultiLine, Set View Count (its grid resized since 2026-09-26: `setQuickViewRowsCols` whole, `fly-quick-viewcount.gui`), Battery Cell Count, the speed dial and its double-click, the Transponder page whole and the gimbal's bars (row 42), and the Geo Reference Images form over `mp-georef` (row 54): **96 done, 1 elsewhere, 19 missing, 18 plumbing, 2 dropped** (undock and the HUD double-click's pop-out window, in a single window; the rest video, scripts and windows of their own); since 2026-09-26, with the video sources, Set View Count's grid and the gimbal's video, **109 done, 1 elsewhere, 6 missing, 18 plumbing, 2 dropped**; since 2026-10-03, with Record Hud to AVI and Stop Record, **111 done, 1 elsewhere, 4 missing, 18 plumbing, 2 dropped**, and with the RAW Sensor window **112 done, 1 elsewhere, 3 missing, 18 plumbing, 2 dropped**, and with Camera Overlap and the photo markers **113 done, 1 elsewhere, 2 missing, 18 plumbing, 2 dropped**. The gimbal's video (an Opus agent's, reviewed and merged): the map menu's Gimbal Video - Full Sized, Mini, Pop Out - and the Payload tab's Video Control, `GimbalVideoControl` whole in `crates/mp-gui/src/gimbal_video.rs` (`FlightData.cs:6534-6712`, `Controls/GimbalVideoControl.cs`, its settings form and the Video Stream form) over `mp-link`'s ports of `CameraProtocol.cs` and `GimbalManagerProtocol.cs` (`camera.rs`, `gimbal_manager.rs`: made on a component's first heartbeat, started two seconds on, told every packet, asked for their messages with the streams): the stream through GStreamer, the keys held for rates and pressed for picture, record, lock, retract, centre, point down and point home, the clicks that point, mark a region of interest or track, the mouse marker on the map, the mini video and mini map at 30% of the panel; drawn in the main window where the C# opens windows; `fly-gimbal-video.gui`, `fly-gimbal-video-control.gui`, `fly-gimbal.gui`. The storm number is measured on the frame: at 200 Hz through the real link, the release build's p99 is 4.9 ms with no frame over 8 ms in 590 (`tests/gui/storm.gui`, on a quiet machine, 2026-09-24; the debug build does not meet it, and the two runs of 06:36Z that day, taken with two release builds compiling, gave p99 9 and 11 ms with 35 and 49 stalls - the gate holds on a quiet machine in release and must be re-run on one). Re-run on 2026-09-26 in release with no build beside it: p50 2.6 ms, p99 3.9 ms, max 5.6 ms, 0 stalls in 570 frames at 199 Hz.
  `MainV2`'s connection controls (2026-09-25, PLAN.md §13.6 row 80) sit at the top right of every screen: the port box
  filled as `CMB_serialport_Click` fills it, the baud box dimmed for the network kinds and while connected, CONNECT/
  DISCONNECT with the still-moving check, each network kind's `InputBox` questions and the settings saved after,
  proved by `tests/gui/main-connect.gui` against SITL over TCP in both suites; AUTO's port scan is not ported.
  The SIMULATION screen is `GCSViews/SITL.cs` whole (row 78, `crates/mp-gui/src/sitl/`): the models, the
  version box, the command line, and on Linux the manifest's native SITL build fetched and started,
  the WASM probe otherwise; its launchers are tested against stubs, `tests/gui/sitl.gui` has run
  (2026-09-25), and a real launch is recorded on Linux (2026-09-26, `tests/gui/sitl-launch.gui`: the
  copter picture fetches the manifest's build and starts it, FLIGHT DATA connects, the heartbeat,
  the parameters, a mode changed and back) and on Windows (2026-09-26 in the VM, `tools/win10/sitl-launch.ps1`: the Cygwin build fetched and started, 34 s); a macOS launch is still owed.
- **Finished since the audit** (moved from NOT_DONE_YET_MATRIX.md on 2026-10-03, which holds only unfinished work from that day):
  - **crates/mp-gui/src/cmd_keys.rs, cmd_keys/** (high, the owner's bug of 2026-10-04 "Mission Planner's other keyboard shortcuts do nothing", finished 2026-10-05): every key of MainV2.ProcessCmdKey (MainV2.cs:4067-4208) and FlightData.ProcessCmdKey (GCSViews/FlightData.cs:865-943). Done 2026-10-04: F2, F3, F4, F12, F5, Ctrl+P, Ctrl+T, Ctrl+Y, Ctrl+F, the flight screen's Ctrl+1 to Ctrl+0. Done 2026-10-05 (the coding subagent's, merged and verified): Ctrl+X the map cache (Controls/GMAPCache.cs), Ctrl+J DevOps (Controls/DevopsUI.cs, with MAVLinkInterface.device_op), Ctrl+W propagation settings (Controls/PropagationSettings.cs; the overlay they configure is not ported), Ctrl+Z the camera test (ExtLibs/ArduPilot/Camera.cs), Ctrl+G and Ctrl+L's windows over every screen, and log playback's Space and the keypad's - and + (the C#'s steps). Tests: each form's unit tests; keys-mapcache.gui, keys-devops.gui, keys-propagation.gui, keys-camera-test.gui (its own fake vehicle, tests/gui/camera-vehicle.py), keys-nmea-spectrogram.gui, fly-playback-keys.gui - all passed headless 2026-10-05.
  - **crates/mp-gui/src/mapview.rs, main.rs** (med, the owner's bug of 2026-10-05, finished the same day): the map's "following" button zoomed right out and stopped following at the first zoom - "following" was the automatic fit of the whole path. Now Mission Planner's Auto Pan (CHK_autopan, ticked by default and kept as that setting; GCSViews/FlightData.cs:1929-1933, 2732-2733, 4242-4253, 5573-5596): while ticked, the flight screen centres the vehicle every three seconds at the zoom the pilot has, when it is more than 0.0001 degrees off centre; the first fix at the default zoom 3 goes to 17; a zoom or a pan does not untick it; the button toggles it and is the flight screen's alone, as the planning map has no Auto Pan. Tests: mapview.rs (a test per rule), tests/gui/fly-map-follow.gui (map.following)
  - **crates/mp-gui/src/fly.rs** (high): The take-off altitude box 'Enter Alt' / 'Enter Takeoff Alt' came up at every TakeOff, for every vehicle (owner's bug report 2026-10-03); the first answer of a session is to be remembered and the question not asked again while the application runs - PLAN.md section 12 D24; done 2026-10-03: fly::takeoff_press - the first press asks, offering the saved takeoff_alt or 5, every later press takes off at the session's answer with no box, on any vehicle; the answer is still saved for the next session; fly-takeoff.gui's second TakeOff asks nothing and sends the take-off again, headless against SITL
  - **crates/mp-gui/src/main.rs** (high): The flight map's column is sometimes laid out 438 px too wide (found 2026-10-03 by the new layout guard, three runs of fly-takeoff.gui of five: the map at x 417 is 1622 px wide inside a column the window cuts at 1184 - `map[417,171 1622x719 in 416,170 1184x748]` - from two seconds after the window to the end, the right 438 px of the map and its follow button off the window and the vehicle drawn off centre; two runs of the same steps laid it out at 1174). Not reproduced on demand: the probe captures of the same steps against SITL show 1174. Cause not yet known; the first frames lay the body out at 1600x1200 at (8, 76) before the header is in, so a size kept from them is the first suspect. Found 2026-10-03: the width varied with the text (1622, 1615) and the probe runs that showed 1174 had shorter status lines - the gimbal video panel's wrapper around the map pane (gimbal_video.rs map_place, 2026-09-26) had no zero minimum, so the strip under the map, a nowrap line main.rs's map_status already truncates with min_w(0), handed its full width up as the wrapper's minimum; min_w(0) on the wrapper, and fly-takeoff.gui's end check - the layout guard - passes headless against SITL
  - **crates/mp-gui/src/fly.rs** (high): The flight map's right-click menu was absent (owner's bug report 2026-10-02): the port had made a right click Fly To Here and put contextMenuStripMap's other entries in a row under the Actions grid; the menu is back on the map with its thirteen entries in the Designer's order and its three drop-downs (Add Poi, Set Home Here, Gimbal Video) opening under the pointer, each entry acting where the menu was opened; the nine fly scripts that used the entries open the menu first and passed headless
  - **crates/mp-gui/src/fly.rs** (low): RAW_Sensor window from the Actions grid, ported 2026-10-03 in crates/mp-gui/src/raw_sensor.rs: Raw Sensor View shows the form over the flight screen - the Raw Sensor page's roll, pitch and yaw dials (gauge.rs's Dial, generalised from the speed dial), the six check boxes, Save CSV, the update-rate combo writing ratesensors and asking for RAW_SENSORS, the ten-second chart; the Radio page's eight inputs and eight outputs; "Please connect first" on the status line without a link; fly-rawsensor.gui (the recorded flight) and fly-rawsensor-nolink.gui passed headless
  - **crates/mp-gui/src/fly.rs** (low): Camera overlap toggle: CAMERA_FEEDBACK photo markers on map, ported 2026-10-03: the link keeps MAV.camerapoints (mp-link camera_points.rs) and timesincelastshot; the flight map draws each shot as GMapMarkerPhoto - the red or green camera icon, the footprint from ImageProjection.calc (its tilted-camera branch now in mp-georef) for the last four and under the pointer, the tooltip - and, with Camera Overlap checked, GMapMarkerOverlapCount's cells and legend (crates/mp-gui/src/camera_photos.rs, mapview.rs); fly-cameraoverlap.gui passed headless on testdata/georef/camera.tlog
- **Tests:** `crates/mp-gui/src/coverage.rs` (the crate is a binary, so its tests are inline) lists every `FlightData` wiring and fails when the report is stale or a claimed id leaves the source; the 44 `tests/gui/fly-*.gui` scripts and `crates/mp-link/tests/commands_sitl.rs` drive arm, disarm, modes, take-off, guided and the Actions page against SITL; `tests/gui/settings-persist.gui` for what survives a restart; `tests/gui/storm.gui` with `crates/mp-link/tests/telemetry_storm.rs` for the 200 Hz budget. Not yet: per-tab snapshots, a layout-persistence test beyond `config.xml`'s keys, and the storm as a criterion bench.

### D11. Flight Planner screen
Waypoint/mission editing, survey grid generation, fences and rally points, terrain and altitude handling,
KML/DXF/shapefile import-export, geotagging hand-off.
- **DoD:** missions produced by the Rust planner are **byte-identical** to the C# planner for a corpus of
  saved `.waypoints` files and grid parameter sets; all survey-grid algorithms numerically verified;
  reads and writes the existing file formats unchanged.
- **Replaces:** `GCSViews/FlightPlanner*`, `Grid/` (3,750), `ExtLibs/SimpleGrid`, `ExtLibs/Gridv2`,
  `ExtLibs/SharpKml`, `ExtLibs/KMLib`, `ExtLibs/netDxf` (65,129), `GeoRef/`, `NoFly/`.
- **Today:** the survey grid is a transliteration of `Grid.CreateGrid` (`crates/mp-mission/src/grid.rs`)
  over a port of ProjNet's transverse Mercator (`utm.rs`, with `utmpos`'s signed-zone convention),
  and it is proved against the C# itself: `tools/csharp-reference/MpGrid.cs` runs the real
  `Grid.CreateGrid` under mono, `regen-grid.sh` regenerates 180 golden cases over 40 polygons
  (rectangles, L/T/U/comb, concave fields, slivers, zone and equator crossings, the antimeridian,
  every start position, lead-in, overshoot, trigger spacing), and `tests/grid_vectors.rs` matches
  every case **bit for bit**. The corridor and rotary generators are ported the same way, with the
  C#'s Clipper inside the rotary (`clipper.rs`), and match on 41 + 63 cases; the Survey (Grid) dialog itself is `GridUI.cs` (`crates/mp-mission/src/gridui.rs` with `System.Decimal` and .NET's number formats in `dotnet.rs`, the camera list from the shipped `camerasBuiltin.xml`, the form from `GridUI.resx` in `crates/mp-gui/src/survey_ui.rs`), and the Survey dialog's oracle - deleted 2026-10-03, being GridUI.cs's own code re-hosted; its goldens stand - ran GridUI's own code statement for statement under mono over 40 cases and 5,656 Accept calls that `tests/gridui_vectors.rs` matches bit for bit - every control, Stats label and emitted item (PLAN.md §13.4 row 29); Gridv2 is not ported;
  a Windows .NET 4.7.2 oracle run (PLAN.md R5) is still owed. The screen's actions are counted:
  `crates/mp-gui/src/planner_coverage.rs` lists all 121 event wirings of `FlightPlanner.Designer.cs`
  (at its first count 40 done, 69 missing, 12 plumbing; now 103 done, 6 missing, 12 plumbing,
  `docs/coverage/flightplanner.md`), and the map's right-click menu is the C#'s in its order, with
  63 entries ported from their handlers (and 8 on the polygon icon's menu), each held to its ledger
  row by `planner_coverage.rs`'s tests and driven by the `tests/gui/plan-*.gui` scripts. Home is the C#'s: the Home Location boxes, not a row,
  written first on Write and Save, kept apart on read, drawn as GMap's green "H" pin; the
  panel's WP Radius, Loiter Radius, Default Alt, frame and Spline boxes with the C#'s typing
  rules and the parameters set after Write; Geo-Fence's return location, file load and save,
  and Clear (PLAN.md §13.4 rows 12, 13); the zoom entries, the Zoom To geocoder prompt, the zoom icon, box and bar, the WP and loiter radius circles scaled as `GMapMarkerRect` scales them, and marker hover with its tooltip (row 30); the rally points (set, upload as the C# uploads them, download by the mission protocol, clear, save and load), the polygon files (`.poly`, `.shp`, offset, area) and the WP and spline circles, each held to the C#'s own handlers under mono (`tools/csharp-reference/PlannerOracle.cs`, row 50); Verify Height, the terrain-frame rules, Set Home Here at the ground's height, the Lat box's prompt and the Elevation Graph over `mp-terrain` (row 35); File Load/Save's Load and Append, Load KML File and Load SHP File and Map Tool's KML Overlay - KML read as SharpKml's parser and `processKML` read it, the shapefile's table through a dBase reader, KMZ through the log crate's zip - with the C#'s two questions and its "Bad KML File :" and "Error opening File" (row 56); Auto WP's Create Circle Survey and Text (the string's glyph outlines as waypoints, the font read with `ttf-parser`), Enter UTM Coord through GeoUtility's own transform, Tracker Home and its marker, the POI entries on the planner, and the polygon icon with its menu - Fence Inclusion and Exclusion shown while the fence is drawn, exclusion polygons uploaded after the inclusion (row 57); Write Fast as a burst mode of the mission transfer, the Alt Mode question and the row and zero-altitude checks both Write buttons run first, the Alt Warn box, the MAVFTP box with `missionpck`'s `@MISSION/mission.dat` written and read over MAVFTP, the Grid box painting the UTM grid, the GEO/UTM/MGRS pointer read-out through GeoUtility's forward transform, ˅, Switch Docking with `FP_docking`, and Prefetch and Prefetch WP Path over `mp_tiles::prefetch` with GMap's menu and form (row 69). Geo-Fence Upload and Download are live since 2026-09-26: the C#'s `FENCE_POINT` protocol to a vehicle without MISSION_FENCE (bit 16384; `tests/gui/legacy-vehicle.py`), and Geo-Fence and Rally hidden over one that has it, as `FlightPlanner.cs:2680-2691` hides them; Inject Custom Map and View KML live since 2026-10-03, Rotate Map blocked on the renderer and GDAL Opacity on the owner's call (**107 done, 2 missing**). Terrain: `srtm.cs`
  is ported whole into `mp-terrain` (tile names, `.hgt` 1"/3" reading, the interpolation and void
  rule, `.asc` grids, the download queue with the C#'s servers and ocean rule, the cache sweep) and
  held to `MissionPlanner.Utilities.dll`'s own `srtm.getAltitude` under mono over 1,259 lookups bit
  for bit (`tools/csharp-reference/SrtmOracle.cs`, PLAN.md §13.4 row 34); the screens call it since
  row 35: the planner's Verify Height, home at the ground's height and the Elevation Graph, and
  the flight screen's Set Home Here and Set EKF Origin Here (row 42).
- **Finished since the audit** (moved from NOT_DONE_YET_MATRIX.md on 2026-10-03, which holds only unfinished work from that day):
  - **crates/mp-gui/src/plan.rs** (med, 2026-10-05): the Home Location link zooms to home as the C#'s does - `label4_LinkClicked` sets home from the vehicle, then calls `zoomToHomeToolStripMenuItem_Click` (GCSViews/FlightPlanner.cs:4283-4300, 8388-8401). Test: plan-panel-params.gui (the map at 17 or nearer after the link).
  - **crates/mp-gui/src/plan.rs** (high): The planner's Mission box scrolled Read WPs, Write WPs and the file buttons off the screen (owner's bug report 2026-10-03), and a self-test is wanted that proves, from the measured geometry, that the important controls of each screen are never hidden - inside the window and inside the box that holds them - on every GUI run; done 2026-10-03: the probe judges every named control's clipping from gpui's content mask cut to the window, crates/mp-gui/src/layout_guard.rs names the controls that matter on each screen and publishes layout.hidden, every GUI run ends by demanding 0 (tools/gui-test.sh, `allow-hidden` to opt out), tests/layout.rs asserts it screen by screen and in Bottom docking; the Mission box's buttons come straight after the pointer read-out as panel4 then panel5 flow in the C#, and the bottom strip is as high as its tallest panel (up to half the window) instead of a fixed 240 px; plan-mission-visible.gui and plan-docking.gui passed headless
  - **crates/mp-gui/src/glyph_text.rs** (high): Plan > Text said "Error" on Windows: the font looked up with fontconfig, which Windows has not got; now the installed font's file or GDI+'s Microsoft Sans Serif; plan-text.gui passed in the VM 2026-09-26
  - **crates/mp-gui/src/plan.rs** (med): GeoFence upload and download menu items: merged with the review's fixes (MISSION_FENCE bit 16384, Geo-Fence and Rally hidden over such a vehicle, return marker, link drop); its five scripts passed headless 2026-09-26 against tests/gui/legacy-vehicle.py and the SITL
  - **crates/mp-gui/src/plan.rs** (low): Inject custom map button, ported 2026-10-03 (crates/mp-gui/src/inject_map.rs): the folder asked for, every jpg/jpeg/png under it at Z<zoom>/<y>/<x> saved again as JPEG into the tile cache under the Custom provider (now a TileSource in mp-tiles, cache-only) on a thread with the bar and the Cancel text, then the map type to Custom and the 'Injecting Custom Map Results' box; plan-inject-map.gui passed headless
  - **crates/mp-gui/src/plan.rs** (low): KML link (lnk_kml), ported 2026-10-03: View KML beside the Grid box opens http://127.0.0.1:56781/network.kml with the desktop's opener, 'Failed to open url ...' on the status line when it cannot; the server behind the address is Utilities/httpserver.cs's row; plan-kml-link.gui passed headless through the opener's harness door (MP_OPEN_WITH_SHELL_LOG)
- **Tests:** `crates/mp-mission/tests/waypoints.rs` (129 corpus files round-tripped, the five Mission Planner wrote byte for byte); `grid_vectors.rs`, `corridor_vectors.rs`, `rotary_vectors.rs` and `gridui_vectors.rs` bit for bit against the C# under mono, `survey.rs`, `robustness.rs`; KML in `crates/mp-kml/tests/` and the planner's KML and SHP loads in `mp-gui`'s tests; terrain is `crates/mp-terrain/tests/oracle.rs` (1,259 lookups against the C# DLL), `queue.rs`, `live.rs`; missions, fences and rally points to SITL and back are `crates/mp-link/tests/mission_sitl.rs` and `fence_sitl.rs` with `tests/gui/plan-rally-legacy.gui` (against `tests/gui/legacy-vehicle.py`: Mission Planner hides Rally Points over a vehicle with MISSION_FENCE, as the SITL is); the 60 `tests/gui/plan-*.gui` scripts. Not yet: DXF (netDxf is not ported), `Gridv2`, the Windows .NET 4.7.2 oracle run.

### D12. Configuration & tuning screens
The full parameter system — tree/list/advanced editors driven by parameter metadata — plus every
`Config*.cs` panel (flight modes, failsafes, battery, compass, ESC, gimbal, ADSB, EKF, …).
- **DoD:** parameter metadata generated from `ParameterMetaDataBackup.xml` / `ParameterFactMetaData.xml` /
  `apm.pdef.xml` at build time; every C# config panel enumerated with a checked-in coverage ledger at 100 %;
  param save/restore round-trips against SITL; `.param` files interoperate with the C# app.
- **Replaces:** `GCSViews/ConfigurationView/*` (the bulk of 67,553 LOC in `GCSViews/`).
- **Today:** parameters fetched as `getParamListMavftp` fetches them (2026-09-25, PLAN.md §13.6 row 81): `@PARAM/param.pck?withdefaults=1` read over MAVFTP and unpacked (`mp_params::parampck`, the C#'s `parampck.cs`), the classic stream with gap recovery as the fallback, and the fetch started on its own once a vehicle is heard with nothing held, as `MAVLinkInterface.Open` starts it; proved through the real link thread against a vehicle serving the file and one without it. Before that: full parameter download with gap recovery (1,408 from SITL), a searchable browser, and
  `.param` save/load/compare in both the GUI and `headless-planner param save|load|diff`. The load-time skip-list
  is ported from `ExtLibs/Utilities/ParamFile.cs:50-76` (all 16 entries, on the load side as the C#
  has it), and numbers are written through a `G15` formatter matching
  `double.ToString(InvariantCulture)` — shortest representation, scientific below 1e-4, which is
  where gyro offsets live. Parameter documentation is fetched for the firmware actually flying,
  as the C# fetches it (`mp_params::pdef`: the version from the banner, the versioned or
  unversioned `apm.pdef.xml` into the C#'s directory, read before the bundled table) - on this
  SITL that takes documented parameters from 798 of 1,408 to 1,407. The panel ledger exists:
  `crates/mp-gui/src/config_coverage.rs` lists all 61 `Config*.cs` panels in the C#'s two menus'
  order with their titles - at its first count **0 done, 7 partial, 48 missing, 2 plumbing, 4 dropped**, 569 wirings -
  held to `InitialSetup.cs`/`SoftwareConfig.cs` by tests and rendered to
  `docs/coverage/configuration.md` (PLAN.md §13.4 row 14); Flight Modes and FailSafe are ported
  from their `Config*.cs` (rows 15, 16), each proved by a script that changes a parameter on
  SITL through the retrying set and reads it back. The SETUP and CONFIG screens are the C#'s
  backstage views: every `AddBackstageViewPage` call of `InitialSetup.cs` and `SoftwareConfig.cs`
  is a list entry with its conditions, headings open and close, the last page is remembered, and
  pages not yet ported say so under their C# title (row 17). Since then Frame Type, Battery Monitor, Install Firmware, Radio Calibration, Motor Test, both compass pages, and the next three Mandatory Hardware pages - Servo Output (`ConfigRadioOutput.cs`: 16 or 32 rows, the bar from `SERVO_OUTPUT_RAW`, reversed/function/min/trim/max written as the `Mavlink*` controls write them, the 300 ms timer started only when not already running, `:154-160`), ESC Calibration (`ConfigESCCalibration.cs:28-45`: `ESC_CALIBRATION` = 3, the button disabled on success, the `MOT_PWM_*`/`MOT_SPIN_*` boxes) and Serial Ports (`ConfigSerial.cs`: one row per `SERIALn_BAUD`, speed/protocol/options with the Set Bitmask window, `SerialOptionRules.json`'s rules applied on a protocol change, `:382-430`; port names from `@SYS/uarts.txt` over MAVFTP since row 37) - each proved by a script writing then restoring a parameter on SITL (rows 20-27, 32); Accel Calibration whole, the older Frame Type and Secure (row 44); ADSB, Camera Gimbal, Battery Monitor 2, Range Finder, Optical Flow and Airspeed (row 45); and the CONFIG list's Planner page (`ConfigPlanner.cs`, 63 controls and 64 wirings: the display units through `mp_vehicle::units`, the telemetry rate combos sending `REQUEST_DATA_STREAM`, the speech prompt chains, map follow/no-fly, load-on-connect, the map access mode, the joystick window; the video, theme, language and layout controls dimmed with their reasons; row 33, its keys in `config.xml` as `Settings.Instance` keys) ; and SETUP's six small pages - HW ID, OSD, CAN GPS Order, Compass/Motor Calib, Initial Tune Parameter with `ParamCompare`, Parachute (row 70, 2026-09-25, their scripts written for SITL and owed a run) ; Standard and Advanced Params over the display view's Layout setting, the MAVFtp page and Heli Setup (row 71's second part, 2026-09-25) (**31 done, 14 partial, 2 missing, 2 plumbing, 12 dropped** - PX4Flow, Bluetooth Setup, the Antenna Tracker, Ateryx Zero Sensors, ESP8266 Setup, CubeID Update, Terminal and the Script REPL dropped at the owner's ruling, PLAN §12 D13; GeoFence, the rover's Basic Tuning and User Params, row 71; Install Firmware Legacy and the manifest page up to the point of touching a board, and Ateryx Pids, row 52; RTK/GPS Inject with its RTCM3 parser and `GPS_RTCM_DATA` injection, row 48; the Serial Ports page names its rows from `@SYS/uarts.txt` over MAVFTP, row 37; Basic Tuning is `ConfigArduplane.cs` whole and Advanced drawn with its thirteen windows named, row 47; Extended Tuning is `ConfigArducopter.cs` whole with its 128 wirings, row 46).
  The Full Parameter List gained the defaults `param.pck` carries (the Default column, None Default), Reset to Default, Load Presaved, Commit Params, the Modified filter, Refresh Table and the collapse (rows 81-82); FFT Setup is `ConfigFFT.cs` whole.
  Since then the copter's Basic Tuning (`ConfigSimplePids`, row 92), DroneCAN/UAVCAN over the `mp-dronecan` crate (15 of 15 wirings), Onboard OSD over `ExtLibs/OSDConfigurator` (2026-10-03), Sik Radio over `mp-sikradio`, the MAVFtp page, Standard and Advanced Params and the Advanced page's thirteen windows: **41 done, 6 partial, 0 missing, 2 plumbing, 12 dropped** (`docs/coverage/configuration.md`, 2026-10-03). **Still owed:** the 6 partial pages, each with its remainder named in the report - Install Firmware (the bench bootloader rewrite on the owner's go), Install Firmware Legacy (Ctrl+Q/Ctrl+P and the device-change probe, recorded omissions), RTK/GPS Inject (DroneCAN over SLCAN), DroneCAN (Compare Params and Reset to Default await the owner's ruling), Advanced (Follow Me, Moving Base and Anon Log ruled out), Planner (the video, theme, language and layout controls held by rulings) - and Joystick's Windows reader; the parameter
  metadata is fetched at run time and bundled as generated Rust rather than generated at build
  time from `ParameterMetaDataBackup.xml`; the `.param` fixture is written from a reading of the
  C# source, not captured from a run of it, and mono's float formatting can diverge from .NET
  4.7.2 (PLAN.md R5) - though §13.4 row 23 found mono formatting `BinaryLog`'s floats as .NET
  does - so settling it needs the Windows runner.
- **Finished since the audit** (moved from NOT_DONE_YET_MATRIX.md on 2026-10-03, which holds only unfinished work from that day):
  - **crates/mp-gui/src/config/advanced.rs** (med): Advanced page: every window ported or ruled out - Follow Me, Moving Base and Anon Log are ruled out of scope (the last 2026-10-02: beta and not interesting); FFT, the MAVLink Inspector, the Warning Manager, Proximity, Mavlink Signing, Spectrogram, Support Proxy, Mavlink Mirror, NMEA and Param gen are done; Param gen 2026-10-03 (ParameterMetaDataParser over every location's Parameters.cpp and its groups' files, ParameterMetaData.xml written as XElement.Save writes it, held to a fixture; config-paramgen.gui passed headless), Mavlink Mirror and NMEA 2026-10-02 (the grid of mirrors kept in serialpasslist, each started by Go; the NMEA sentences at the rate with the EGM96 geoid ported into mp-terrain and held to the C# under Mono; config-mirror.gui and config-nmea.gui passed headless), Spectrogram and Support Proxy 2026-10-02 (GenerateImage over a log's IMU and ISBH samples, the fixture's numbers checked against pymavlink; the link mirrored to a support server over TCP or UDP, each packet re-encoded; config-spectrogram.gui and config-supportproxy.gui passed headless), Proximity and Mavlink Signing 2026-10-02 (the readings drawn as Temp_Paint draws them; authkeys.xml as Crypto.cs encrypts it, read and written byte for byte against files Mono wrote, the link signing its frames and checking the vehicle's; config-proximity.gui and config-signing.gui passed headless), the Warning Manager 2026-10-02 (the rules, the 250 ms engine on the frame, warnings.xml read and written as the C# writes it, the HUD's message and the quick views' colouring; config-warnings.gui passed headless), the Inspector 2026-09-27 (its tree, rates, GCS traffic and Graph It over a packet subscription on the link; config-mavlink-inspector.gui passed headless against the SITL)
  - **crates/mp-gui/src/config/friendly_params.rs** (med): Standard Params page: the DropDown combo typed into (the exact-matching row, or none and the C#'s NullReferenceException text), Ctrl+S as Write Params, Refresh Params; config-standard-params.gui passed headless
  - **crates/mp-gui/src/config/friendly_params.rs** (med): Advanced Params page: the track bar's thumb dragged and its keys (LargeChange and SmallChange 10), Ctrl+S; config-advanced-params.gui passed headless with the runner's new drag verb
  - **crates/mp-gui/src/config/mavftp.rs** (med): MAVFtp page: 16 of 16 wirings - the tree's and list's own selection, keys, delayed label edit, header drag and dividers from comctl32, Upload of several files, the context menu from the keyboard; Mount as Drive fails on the status line (no Dokan here); config-mavftp.gui passed headless
  - **crates/mp-gui/src/config/adsb.rs** (low): ADSB page: its five wirings, the bitmask narrowed to the parameter's type, Ctrl+S; config-adsb.gui passed headless
  - **crates/mp-gui/src/config/user_params.rs** (low): User Params: the whole page since 2026-09-26 (the list or the 22 RC option names, the combos writing their parameters, Modify's InputBox with the C#'s Cancel quirk kept); the row was not moved when the page was finished; the coverage entry is Done since 2026-10-03
- **Tests:** `crates/mp-params/tests/param_meta.rs` (the bundled table and the fetched `apm.pdef.xml`) and `param_file_compat.rs` (`.param` files byte for byte); `crates/mp-gui/src/config_coverage.rs`'s tests hold the 61-panel ledger to `InitialSetup.cs` and `SoftwareConfig.cs`; the 60 `tests/gui/config-*.gui` scripts each change a parameter on SITL and put it back (all run in the Linux suite of 2026-09-26 and the Windows suite of 2026-09-27, the newer ones headless at their commits), `crates/mp-params`'s `parampck` tests, and `params-retry.gui` proves a set survives a dropped send. Not yet: `tests/metadata_codegen.rs` (the metadata is not generated at build time), a `param_roundtrip.rs` over every parameter type and edge value, per-panel snapshots.

### D13. Initial setup, calibration and firmware
Wizards and calibration routines (accel, compass/mag-cal, radio, ESC, frame, sensors) and the firmware
path: board detect, firmware catalogue, upload via px4/DFU/serial bootloaders.
- **DoD:** calibration maths (ellipsoid fit, `MagCalib`, accel cal) reproduces C# results to
  **1e-6 relative**, proven by golden-vector tests; board detection matches `MissionPlannerTests`
  `DetectBoardTest` cases; a real board flashes successfully on all three OSes.
- **Replaces:** `GCSViews/InitialSetup/*`, `MagCalib.cs`, `ExtLibs/ArduPilot` firmware code (23,564).
- **Today:** accelerometer, compass, radio and motor test are pages of the C#'s setup list;
  accelerometer (`ConfigAccelerometerCalibration.cs` whole: the six-position conversation with
  "Click when Done" answering each `ACCELCAL_VEHICLE_POS`, Level, Simple; PLAN.md §13.4 row 44),
  compass, radio and motor test are their `Config*.cs` whole - the priority table and onboard
  calibration of `ConfigHWCompass2`, `ConfigRadioInput`'s calibration conversation writing
  MIN/MAX/TRIM, `ConfigMotorTest`'s buttons from the frame's motor layout - each proved by a script
  against SITL (PLAN.md §13.4 rows 25-27), and exercised by hand on a physical MR-VMU-RT1176 earlier (unproven: no test or recorded run). `mp-firmware` ports
  the `.apj` container and the px4 bootloader protocol from `ExtLibs/px4uploader/`, with
  `tests/firmware_upload.rs` driving a complete upload against a strict in-process mock that
  asserts every byte, and `tests/flash_px4.rs` driving `UploadPX4`'s whole sequence - the reboot
  into the bootloader, the thirty-second port scan, the same-firmware question, the upload and
  its words - over a bench of pretend ports (row 79). The Install Firmware page now takes that
  path to this machine's real ports, and **has flashed a real board**: on 2026-09-25 (13:52-13:55,
  on the owner's explicit go) `tests/gui/setup-firmware-flash-bench.gui` wrote ArduCopter 4.7.1
  stable to the bench CubeOrange through the page - reboot into the bootloader, scan, CRC compare,
  erase, program, verify, reboot, "Upload Done" - and the board came back running it (PLAN.md §13.6
  row 79 has every attempt, including the experimental bootloader that had to be replaced through
  the owner's debugger first). Flashed from Linux (2026-09-25) and from the Windows VM (2026-09-27, the board on COM4, "Upload Done" in 44.7 s, after the reboot learned to pass over the board's ADS-B heartbeat); macOS remains. A port failing during
  the flash is a status line, never a message box (the owner's ruling of the same day, a written
  divergence from `Firmware.cs:702, 710`). The CLI still offers `headless-planner firmware info` and
  `headless-planner firmware detect` and `firmware list`, and nothing that writes. Board detection is
  `Utilities/BoardDetect.cs` ported rule for rule (`crates/mp-firmware/src/detect.rs`), its probes
  proved against the px4 mock over a pty; all 16 `DetectBoardTest` calls are fixtures, and five of
  them fail against the C# itself, which the fixture records. A real Cube Orange running ArduPilot
  (2026-09-24) presents `27B1:0004`, ArduPilot's application-mode id, which `BoardDetect.cs` does
  not know either - the C# identifies a Cube by its bootloader id after a reboot, or by a WMI
  name on Windows - so `headless-planner firmware detect` says what the C# would do next: ask, then probe for
  an STK500 bootloader. The firmware catalogue is
  `APFirmware.cs` ported - the manifest, its mirror-then-ardupilot.org order, the board and
  release selection - proved on a 240-record excerpt of the real manifest, and the Install
  Firmware page showed what the C# would flash with its Upload button disabled (PLAN.md §13.4
  row 21) until row 79 wired the upload to real ports. Install Firmware Legacy (`ConfigFirmware.cs` over `firmware2.xml`) and the manifest page's remainder are ported, and since 2026-10-03 the legacy page's flows go on to a px4-family board through `upload_px4` as the manifest page's do, the other boards' uploads stopping as not ported, each stop named in `mp_firmware::flow::Stop` (row 52); Bootloader Update sends `MAV_CMD_FLASH_BOOTLOADER` after its
  two questions (row 71), and the bench CubeOrange answered `MAV_RESULT_UNSUPPORTED`. `MagCalib.cs`'s fit - the offboard
  sphere and ellipsoid behind the older compass page's Live and Log Calibration, the one
  calibration whose maths the DoD's 1e-6 clause is about - is ported as
  `mp_calibration::magcalib` (2026-09-25): a telemetry log's `RAW_IMU` less `SENSOR_OFFSETS`
  through the C#'s duplicate filter, its throttle gate, count check and outlier cut, or a
  dataflash log's `MAG` lines less their offsets, fitted as `LeastSq`/`doLSQ` fit them, and the
  boxes `SaveOffsets` shows; `headless-planner magcal <log> [--ellipsoid] [--min-throttle N]` is `ProcessLog`.
  alglib's Levenberg-Marquardt is replaced by the `levenberg-marquardt` crate on alglib's own
  central-difference Jacobian (`diffstep` 0.1, `epsx` 0, `maxits` 100 kept), so the fit is held
  to PLAN.md §7.2's class D rather than 1e-6: the C# cannot run here to give its residual, and
  alglib's path cannot be bit-matched. Live Calibration's dialog (its spheres, drawn flat, the
  coverage test and prompts) and the offsets written to the vehicle through
  `PREFLIGHT_SET_SENSOR_OFFSETS` are ported (2026-09-26, `mp_calibration::live_magcal`,
  `crates/mp-gui/src/config/live_magcal.rs`), proved by unit tests and one run of the loop
  through the real link against a scripted autopilot; its GUI script needs an ArduPlane
  3.7.1-4.0 SITL, which is not bundled, so it is written and unrun.
- **Finished since the audit** (moved from NOT_DONE_YET_MATRIX.md on 2026-10-03, which holds only unfinished work from that day):
  - **crates/mp-transport/src/win32.rs** (high): Windows board detection: the port list takes SetupAPI's hardware id and bus-reported name as Win32DeviceMgmt reads them - Install Firmware found no board on Windows (owner's bug report 2026-09-26); committed 9a51796, the bench CubeOrange detected in the VM as board id 140
  - **crates/mp-gui/src/config/firmware.rs** (med): Install Firmware manifest page: Ctrl+Q, FirmwareSelection's pickers, the bootloader probe on a port's arrival, Force Bootloader, Bootloader Update, the stale "flashing is not enabled" texts gone (the owner's Windows report) - merged 2026-09-26 with the review's fixes (no serial port opened while MP_FIRMWARE_DEVICE names the device; the found board forgotten with the SETUP screen; a port back as another device an arrival; no probe around a flash left running; a lost link said). config-firmware, config-firmware-legacy and config-firmware-blupdate passed in the VM too. The legacy page's Force Bootloader is the manifest page's, one handler for both (merged 2026-09-27, its "flashing is not enabled" text gone; config-firmware-legacy passed headless)
  - **Windows bench** (med): The bench CubeOrange passed through to the VM (done 2026-09-26) and detected there (board id 140); flashed from Windows on the owner's go 2026-09-27 - ArduCopter 4.7.1 stable, "Upload Done" in 44.7 s, after the same-firmware question was answered and the reboot taught to pass over the board's ADS-B heartbeat (PLAN §13.6 row 79)
  - **crates/mp-gui/src/config/sikradio.rs** (med): Sik Radio page: 17 of 17 wirings over the new mp-sikradio crate (the AT/RT session, settings, SiK and RFD900x bootloader uploads, IHex, XModem), every control held to Sikradio.resx; config-sikradio.gui passed headless against the scripted RFD900+ stand-in
  - **crates/mp-gui/src/config/onboard_osd.rs** (med): Onboard OSD page (ConfigOSD over ExtLibs/OSDConfigurator), ported 2026-10-03: every OSD* parameter a setting, a tab a screen of items drawn with the Clarity font on the 30x16 or 60x22 canvas and dragged a cell at a time, the option rows (bool, spin, dropdown, bitwise, text), Copy and Paste Layout, Show Names, Decrease, HD Layout, Clear All, Write customization under its progress dialog, Auto write on leaving, Discard and Refresh with their questions, the OSD5/6 slot names fetched over OSD_PARAM_SHOW_CONFIG and the slots dialog writing OSD_PARAM_CONFIG; config-onboard-osd.gui passed headless against tests/gui/osd-vehicle.py (the SITL copter lists OSD_TYPE alone); 20 unit tests; the C#'s boxes on the status line by the owner's rule
  - **crates/mp-gui/src/config/firmware_legacy.rs** (low): Install Firmware Legacy: all 20 wirings since 2026-09-26, and since 2026-10-03 UploadFlash too - the page's flows go on to a px4-family board through mp_firmware::flow::upload_px4 as the manifest page's do (not while MP_FIRMWARE_DEVICE stands in for the machine; the other boards' uploads stop as not ported), the dimmed Upload row gone; recorded omissions: Ctrl+Q/Ctrl+P (no control of the page takes the keyboard) and Instance_DeviceChanged's probe; config-firmware-legacy.gui passed headless
  - **crates/mp-gui/src/config/optical_flow.rs** (low): Optical Flow: both its wirings and the whole page since 2026-09-26 (the legacy FLOW_ENABLE page or the new-style one, the sensor picture, the rover's height override); the one divergence - a yaw below -179 kept rather than written back - is at its site; the row was not moved when the page was finished; the coverage entry is Done since 2026-10-03
  - **crates/mp-gui/src/config** (low): Mandatory and Optional Hardware heading panels' text: both sentences at label1's place since 2026-09-26 (setup.rs heading_page, ConfigMandatory.resx and ConfigOptional.resx); the row was not moved then
- **Tests:** `crates/mp-firmware/tests/board_detect.rs` (all 16 `DetectBoardTest` calls, the five that fail against the C# recorded), `firmware_upload.rs` (every byte against a strict px4 mock), `flash_px4.rs` (`UploadPX4`'s whole sequence over a bench of pretend ports), `manifest.rs` and `legacy.rs` (the catalogues on fixtures); the calibration pages' unit tests inline in `mp-gui` and `mp-calibration`, and `tests/gui/config-accel.gui`, `config-compass.gui`, `config-radio.gui`, `config-motortest.gui`, `config-firmware.gui` and `config-firmware-legacy.gui` on SITL, `config-compassmot.gui` (unrun) with `crates/mp-link/tests/compassmot.rs`, and `setup-firmware-flash-bench.gui` on the bench board; `crates/mp-calibration/tests/magcal_vectors.rs` (class D on the sample sets `testdata`'s logs give, `testdata/magcal`, against an independent sphere fit, and synthetic offsets and scale recovered to 1%) and `crates/mp-cli/tests/magcal_verb.rs`. Not yet: `tests/accelcal_vectors.rs` (accelerometer calibration is the vehicle's), a DFU mock, and a real-board flash on macOS (Linux 2026-09-25 and the Windows VM 2026-09-27, `tests/gui/setup-firmware-flash-bench.gui`).

### D14. Log engine and analysis
Dataflash (`.bin`/`.log`) and tlog parsing, log download, graphing, LogAnalyzer rules, DSP/FFT, exports
(`.mat`, CSV, KML), EXIF geotagging.
- **DoD:** memory-mapped columnar parse of a **1 GB dataflash log in < 2 s**, then scrub a **10 M-point**
  multi-series plot at **120 fps** with GPU line rendering + LOD; parsed field values match the C# parser
  exactly across a corpus of real logs; FFT output matches `FFT2` (`ExtLibs/Utilities/fft.cs`, the one Mission Planner runs; `Exocortex.DSP` and `fft3.cs` have no callers) within float tolerance.
- **Replaces:** `Log/` (9,971), `LogAnalyzer/`, `graphs/`, `ExtLibs/ZedGraph` (52,265),
  `ExtLibs/Exocortex.DSP`, the used subset of `ExtLibs/alglibnet` (251,616 — audit what is actually called),
  `ExtLibs/MetaDataExtractorCSharp240d` (17,800), `ExtLibs/ICSharpCode.SharpZipLib` + `zlib.net` + `7zip`.
- **Today:** `.tlog` read and write, dataflash `.BIN` parsing, log download from a vehicle,
  automatic recording of every flight — both directions of the link, named in local time as Mission
  Planner names them, into the logs directory `Settings.GetDefaultLogDir` names (ported in
  `mp-settings`; not `Documents/` on Linux, as an earlier version of this line said) — and KML
  export of a flown path, coloured by flight mode. `mp-chart` holds the min/max reduction the plot
  target needs and drives the live tuning graph and the log browser, which plots any field a
  `.BIN` declares on two axes - left click for the left, one axis per unit, right click for the
  shared right axis, as `Log/LogBrowse.cs` does - with the units and multipliers the log's own
  `FMTU`/`UNIT`/`MULT` messages declare (`mp_log::plot::units`; the C# has the same code and a
  guard that keeps it from ever running, recorded at the site). The data grid is `dataGridView1`
  as a virtual grid over `mp_log::index` (nine bytes per record, sixteen rows decoded at a time,
  a per-type filter), with Graph Left/Right acting on the selected cell and refusing what
  `graphit_clickprocess` refuses; `mp_log::track` draws the log's first GPS route and its logged
  mission on a map beside the chart; the double-click cursor with its map pin and grid row, and
  the strip's Map/Time/Data Table/Mode/Errors/MSG/Events boxes drawing what the C# draws
  (PLAN.md §13.4 row 7); then the rest of `LogBrowse.cs`, ledgered in `docs/coverage/logbrowse.md`
  (37 designer wirings: 29 done, 0 missing, 6 plumbing, 2 dropped): Show Params, the preselected
  graph sets from the shipped `graphs/*.xml` evaluated without IronPython, the GPS/GPS2/GPSB/POS/CMD
  routes with CAM markers, ZedGraph's point values, zoom and pan, the grid's export menu, Ctrl+G
  and the field modifier; and D14's parse budget met - `mp_log::logfile::LogFile` opens a 1.07 GB
  log to its first plot in about 0.6 s (326.8 s before), gated by `benches/parse_1gb.rs` (row 51). The DataFlash Logs page's four conversions are ported and held to
  Mission Planner's own code under mono: `.BIN → .log` byte-identical, KML+GPX and `.mat`
  identical but for a namespace order and a hash-table order, Auto Analysis with ArduPilot's
  LogAnalyzer checks ported from the runner's Python and run in-process, held to Python 2.7 over
  the same logs (row 23); the page's buttons call them (row 28). Geo Reference Images' logic is `mp-georef`
  (`georefimage.cs` and `GeoRefImageBase.cs`: the three matching modes, every output file and the
  EXIF geotags byte for byte to the real classes under mono over a SITL flight with camera
  messages, `tools/csharp-reference/GeorefOracle.cs`, row 49) and its form is inside this window (row 54).
  **The FFT** is `mp_log::fft` on `rustfft`: nothing in Mission Planner calls `Exocortex.DSP` (nor
  `fft3.cs`, the alglib sliding DFT) - the FFT window `Controls/fftui.cs` and `Spectrogram.cs` both
  use `FFT2` in `ExtLibs/Utilities/fft.cs`, so that is the spec: a periodic Hann window with gain
  `4/N` (a sine of amplitude A on a bin reads A), the unnormalised forward DFT, `N/2` magnitudes,
  dB as `20/ln10 * ln(m + double.Epsilon)`, `FreqTable`'s whole-hertz `int` bins, the window's
  sample-rate estimate (`Math.Round(1000/timedelta, 1)` over its exponential average) and the
  "Run all imus" average (every whole slice but the last, each over the slice count, bins below
  Start Freq zero). `headless-planner log fft <log> <MSG.Field> [size] [--mag] [--start hz]` prints the graph's
  title and the highest peaks as its tooltip reads them (`"{0} hz/{1} rpm"`). The window is
  `Controls/fftui.cs` as a modal over SETUP (`config/fftui.rs`), opened from the FFT Setup page
  (`config/fft.rs`, `ConfigFFT.cs`) as in the C#, and from the Advanced page's FFT button (row 90). The Spectrogram window is `Spectrogram.cs`'s `GenerateImage` over a log's IMU and ISBH samples (2026-10-02, its numbers checked against pymavlink), and the DataFlash page's Auto Analysis runs ArduPilot's LogAnalyzer checks ported from the Python 2 source in-process on every platform (`crates/mp-log/src/analysis`, 2026-10-03, held to Python 2.7 over the same text). **The 10 M-point scrub** is measured and met:
  `mp_chart::Series` keeps an index - runs of non-decreasing time and a min/max pyramid with each
  block's first time - so `extent`, `auto_range` and `reduce` read O(width x log32(n/width))
  things instead of every sample, the same columns, lows and highs as the scan to the bit
  (`reduce_scan`, kept as the reference); `Positions::line_at_time` searches a running-latest
  time index instead of walking. Eight series of 10 M samples, 1,000 cursor steps: whole log p50
  1.89 ms, p99 3.41 ms a step (1 M: 1.30 / 1.87 ms); zoomed to a minute p50 0.71 ms, p99 1.08 ms;
  reads a frame 10 M/1 M 1.35 (116 a column), time 1.46; the frame before was 1,270 ms at 10 M.
  `logbrowse/view.rs`'s `nearest_point` (Show Point Values) measures only the samples within the
  pointer's reach through a per-curve time order, held equal to the old walk over 900 pointers
  (row 86); the field descriptions come from `LogMessages.xml.xz`, downloaded and parsed as
  `LogMetaData.cs` does (`logbrowse/metadata.rs`, row 86). Not yet: a memory-mapped parse - `LogFile` reads
  and indexes instead, `unsafe` being forbidden in `mp-log`, and meets the budget without it, at
  a peak of 1.62 GB for the 1.07 GB log.
- **Finished since the audit** (moved from NOT_DONE_YET_MATRIX.md on 2026-10-03, which holds only unfinished work from that day):
  - **crates/mp-gui/src/plotline.rs** (high): Logs > PLOT drew a string of dots (owner's bug report 2026-09-26): every ZedGraph curve - the log browser's, the tuning graph's, the FFT screen's - as a line through its points, clipped, the FFT's diamonds; log-browse.gui passed headless (ATT.Roll one line of 182 points)
  - **crates/mp-log/src/analysis** (med): Log analysis (loganalysis): ArduPilot's LogAnalyzer, the Python 2 source Mission Planner ships as runner.exe (LogAnalyzer/py2exe), ported 2026-10-03 and run in-process on every platform - DataflashLog's reading of the .log text, the seventeen checks line for line with Python 2's arithmetic and wording, TestSuite's XML written beside the log and read back as Utilities/LogAnalyzer.cs reads it; held to Python 2.7 running the runner's own source over the same text (tools/loganalyzer-golden.sh; the dict and set orders the Python leaves to its hash tables are the divergences, written at their sites); the DataFlash Logs page's Auto Analysis and `headless-planner log loganalysis` use it; fly-conversions.gui exercises the button
- **Tests:** `crates/mp-log/tests/dataflash.rs`, `tlog.rs`, `logfile.rs`, `convert.rs` (`.BIN → .log` byte-identical to `BinaryLog` under mono), `matlab.rs`, `analysis.rs`, `robustness.rs`, with `crates/mp-kml/tests/dflog.rs` and `real_flight.rs` for KML+GPX, `crates/mp-cli/tests/log_verbs.rs`, and `crates/mp-georef/tests/oracle.rs` (14 cases against the C# classes), `photos.rs`, `roundtrip.rs`, `edge.rs`, `behaviour.rs`; `fuzz/fuzz_targets/tlog_reader.rs` with the bounded pass in `mp-fuzz-checks`; `benches/parse_1gb.rs` (0.6 s to first plot on a 1.07 GB log); the log browser's `logbrowse/coverage.rs` tests and the 8 `tests/gui/log-*.gui` scripts; `crates/mp-log/tests/fft.rs` (synthetic sines recovered at their bin to 1e-6 relative, A/2 either side, a DC level at 2c, dB, the average's slices and start frequency, and `rustfft` against a line-for-line transcription of `FFT2.run` to 1e-12 of the peak) with `fft`'s unit tests and `log_verbs.rs`'s `headless-planner log fft` case; `crates/mp-chart/tests/lod.rs` (the indexed extent, range and reduction equal to the scan at 1-2,560 columns over rolling, repeating, restarting and random series, proptest, and the reads a frame bounded as the length grows 16 times) and `overlay.rs`'s `line_at_time_is_the_walk`; `crates/mp-chart/benches/scrub_10m.rs` (the gate: p99 <= 8.33 ms at 10 M, reads 10 M/1 M <= 1.5 and <= 800 a column, time ratio <= 2.5, and the reduction at 10 M equal to the scan at 240 and 1,920 columns). Not yet: a dataflash fuzz target.

### D15. CAN, peripherals and outboard features
DroneCAN/UAVCAN (node list, param edit, firmware update), OSD configurator, antenna tracker, SiK radio
config, joystick input, swarm control, warnings engine, web APIs, ADS-B / Altitude Angel.
- **DoD:** DSDL-generated DroneCAN types regenerable from source; joystick input path **< 5 ms** end to end
  on all three OSes; antenna tracker and swarm verified against SITL; every feature in this bucket either
  shipped or explicitly listed as dropped with a rationale in PLAN.md.
- **Replaces:** `ExtLibs/DroneCAN` (26,733) + `UAVCANFlasher`, `ExtLibs/OSDConfigurator`, `Antenna/`,
  `Joystick/`, `Swarm/` (6,365), `Warnings/`, `ExtLibs/WebAPIs` (24,978), `ExtLibs/AltitudeAngelWings`,
  `ExtLibs/NMEA2000`, `ExtLibs/solo`, `ExtLibs/Onvif`, video stack (`DirectShowLib` 37,629, `LibVLC.NET`,
  `AviFile`, `WebCamService`).
- **Today:** ADS-B traffic on the map, and joystick input on Linux — `/dev/input/js*` read without
  `unsafe` on a thread that blocks on the device, mapped to `RC_CHANNELS_OVERRIDE` with expo,
  reversal and a release-on-disconnect failsafe, and sent on change from a second thread through
  a `LinkSender` handle: p99 0.152 ms stick-to-link for an isolated movement on an in-process
  fake device (`crates/mp-input/tests/latency.rs`), a 20 ms floor between sends so a stirred
  gamepad cannot flood a radio (a stick stirred at 1 kHz puts 50 frames/s on the wire, each at
  most 20 ms stale), and Mission Planner's 50 ms resend ceiling. The 5 ms target is therefore met
  for a movement, not for a continuous stir, which no floor could meet without flooding. **Still owed:** the histogram from a
  real device (`tests/real_device.rs`, ignored until one is attached); a deadzone; a per-link
  send budget. Since then RTK/GPS Inject (`ConfigSerialInjectGPS.cs`, `rtcm3.cs`, the `GPS_RTCM_DATA`
  injection; PLAN.md §13.4 row 48) and the NTRIP transport (row 43), the base-station half of this
  bucket. Video capture exists on Linux: `crates/mp-video` over V4L2, the Planner page's Video
  Device, Video Format, Start and Stop, the frame under the HUD (PLAN.md §13.6 rows 83-84); and
  since 2026-09-26 the HUD menu's GStreamer, HereLink and MJPEG sources with GStreamer Stop (row
  93), the pipeline text run in the installed runtime's launcher and its frames read back; the gimbal's video (`GimbalVideoControl`) over the camera and gimbal-manager protocols; on Windows, Media Foundation's capture devices, MJPEG and YUY2 types and an asynchronous Source Reader (`mediafoundation.rs`, the crate's one `unsafe` file by the owner's ruling of 2026-09-27; its tests pass in the VM, a real camera there still owed); Record HUD to AVI over `AviWriter.cs` (2026-10-03). **Since 2026-10-02, the rest of the bucket:** DroneCAN/UAVCAN is the `mp-dronecan` crate and its page, 15 of 15 wirings - node 127, the transfer layer, SLCAN, MAVLinkCAN and multicast, the parameter window, firmware updates, passthroughs, Filter, Stats, the Inspector with Graph It and its Subscriber decoding 27 types field by field - against stand-ins, never a real bus; Compare Params and Reset to Default await the owner's ruling. Sik Radio is the `mp-sikradio` crate and its page, 17 of 17 wirings - the AT/RT session, the settings, SiK and RFD900x bootloader uploads, IHex, XModem - against a scripted RFD900+. The OSD configurator is the Onboard OSD page over `ExtLibs/OSDConfigurator` (2026-10-03). The warnings engine (`Warnings/`) is `warnings.rs`: the rules, the 250 ms pass on the frame, `warnings.xml` as the C# writes it, the HUD's message and the quick views' colouring; what it would speak is kept as a fact, since speech is not ported. Joystick Setup is the page whole (Linux; the Windows reader `JoystickWindows.cs` is owed, so Windows lists no joystick). Dropped at the owner's ruling (PLAN.md §12 D13): the antenna tracker (`Antenna/`, `TrackerUI`), swarm (`Swarm/`), Altitude Angel Wings, NMEA2000, Bluetooth and ESP8266 Setup; dropped under D16 and D13 together: 66 of `ExtLibs/WebAPIs`' 67 files - the dead clients and Dowding's (`Program.cs`, the library's console program, is the one left `ready`) - and the HIL library's `XPlane.cs` (its only user; `Hil.cs`, `Aircraft.cs`, `QuadCopter.cs` and `Wind.cs` are still `ready` though nothing but it reached them). **Still owed:** speech (`Utilities/Speech.cs` and its callers in `MainV2`, `CurrentState`, `MAVLinkInterface`, the warning engine and the scripts), the Windows joystick reader, a real camera on Windows, `ExtLibs/solo` and `ExtLibs/Onvif` (two rows still `ready`; 3DR Solo is out by D9), and DSDL generation - the 27 decoded types are hand-ported, PLAN.md §4.2's `dsdlgen` is not written.
- **Tests:** `crates/mp-input/tests/latency.rs` (p99 0.152 ms on a fake device) and `real_device.rs` (`--ignored` until a joystick is attached); `crates/mp-link/tests/traffic.rs` for ADS-B; the RTCM parser's and injection's unit tests with `tests/gui/config-rtk.gui` against a caster the script starts; `crates/mp-video`'s unit tests over `testing::FakeSource`, and `tests/gui/config-video.gui` (unrun). `crates/mp-dronecan`'s and `crates/mp-sikradio`'s tests are inline; `tests/gui/config-dronecan.gui` and `config-dronecan-inspector.gui`, `config-sikradio.gui` (against the scripted RFD900+), `config-warnings.gui`, `config-joystick.gui`, `config-onboard-osd.gui` (against `tests/gui/osd-vehicle.py`), `fly-gimbal-video*.gui`, `fly-hud-avi.gui`; `mp-video`'s Media Foundation tests pass in the VM; `fuzz/fuzz_targets/dronecan_slcan.rs`. Not yet: `dsdl_roundtrip.rs` (no DSDL generator), `node_sim.rs` and a real CAN bus, `video_pipeline.rs` against a real stream, and `feature_ledger.rs`, which is the one that would make this bucket's omissions visible; `tracker.rs` and `swarm.rs` fall with D13's ruling.

### D16. Extension and scripting system
Three tiers, because Mission Planner already ships two mechanisms and users touch both:
**(1) embedded Python** via `rustpython-vm` (pure Rust, no system Python, no compile step) as the
direct replacement for IronPython, with `pyo3`/CPython behind an opt-in feature for users who need
numpy or pymavlink; **(2) a sandboxed WASM extension host** modelled on zed's `extension` /
`extension_host` / `extension_api` for distributable extensions; **(3)** full-trust native plugins,
which are [D21](#d21-native-in-process-plugin-host) and come last.
Python is not a preference, it is compatibility: the 19 scripts in `testdata/scripts/` ship with
Mission Planner today and are already Python. A Rust-native scripting language would break every
one of them.
- **DoD:** a sample extension builds, loads, adds a panel, subscribes to telemetry and sends commands;
  extensions are sandboxed and cannot crash the app; a migration guide plus at least one real C# plugin and
  one IronPython script reimplemented as proof; API versioned and documented.
- **Replaces:** `Plugin/`, `Plugins/`, `plugins/` (13,289 total), `Script.cs` + `Scripts/` + IronPython.
- **Today:** the extension half is built (2026-09-26, PLAN.md §13.6 row 96): `crates/mp-plugin-host` hosts `*.wasm` plugins from `plugins/` beside the executable on wasmtime's component model, the C#'s `Plugin` lifecycle and `PluginHost` surface as a WIT world, each plugin on its thread at its `loopratehz` under fuel; the four shipped plugins and seven examples are ported to it and driven through the host in tests, and the planner draws a plugin's described form and its menu entries. Not reachable from a plugin, listed at the site: packet subscription, sockets, the main window's members. It was decided (PLAN.md §12 D22) and tried first: `experiments/wasm-plugin-host` hosts a WebAssembly plugin on the C#'s `Plugin` lifecycle under wasmtime, with the write-up recommending the component model for the real host (row 95); the interpreter is decided too - RustPython, §12 D20, row 94. `mp-script` implements the `Script.cs` host API with the C#'s semantics - including
  `GetParam` returning 0.0 for a missing parameter, `ChangeMode` always returning true, `WaitFor`
  substring-matching messages that arrived before the call, channels capped at 8 and an override
  sent twice 20 ms apart - and measures what the corpus needs. **The measurement changes the
  estimate:** 15 of the 19 scripts reach .NET types directly through IronPython's assembly loading
  which no Rust engine can provide by itself; only 4 stay inside the scope bindings, and 11 call into `MAV`, which is where the compatibility shim had to start - and did (below). The interpreter is wired (2026-09-25):
  RustPython runs the scripts on the Scripts tab with `Script` and `cs`, the 19 shipped scripts are
  moved to Python 3 (`testdata/scripts/CHANGES.md`) and each ends in tests as recorded - two run,
  fourteen stopped at `import clr`, wipe.py at `MAV`. Since 2026-09-26 scripts are handed `MAV`, `MainV2`, the screens, `Ports` and `Joystick` through the shim (`crates/mp-script/src/clr/`): getWP and setWPTotal as the link's requests, a dropped link or Abort ending a waiting script, `MAV.wps` and `MAV.rallypoints` kept in the link (2026-10-03); of the fourteen `import clr` scripts ten reach their end or loop as under IronPython and four stop on the C#'s own signatures - `setWPCurrent(1)` where the only form takes three arguments, a two-argument `SubscribeToPacketType` - IronPython's TypeErrors in the shim's words (ruled 2026-10-03: an overload the C# has not got is not ours to add); `stock_scripts.rs` records every verdict.
- **Finished since the audit** (moved from NOT_DONE_YET_MATRIX.md on 2026-10-03, which holds only unfinished work from that day):
  - **crates/mp-gui/src/plugins_ui.rs, build.rs, fly.rs; plugins/examples/opendroneid.rs** (high, the owner's bug of 2026-10-03 and what followed, finished 2026-10-05): a plain start loads the plugins Mission Planner ships - the planner carries the ten ports, built for WebAssembly by mp-gui's build script, a plugins folder's file replacing a built-in of its name (Plugin/PluginLoader.cs:203-311); payloadconfig left out, as its empty table never loads in the C#. Open Drone ID's form is its flight screen page, `tabDroneID` "Drone ID", as the C#'s plugin inserts it (plugin.wit's flight-data-tab-add: `TabListOriginal.Add`, `tabControlactions.TabPages.Insert(5, tab)`), shown where the saved tabs name it and listed by Customize - FlightData's Activate applies the saved tabs after the plugins are loaded, so as in the C# the page shows once ticked there; no plugin form lies over the window any more (it covered FLIGHT PLAN's Write, found by the Welcome-Demo-Sitl). By the owner's word (2026-10-05) Open Drone ID is off until a user turns it on: off while `DisabledPlugins` has never been saved, and Save keeps an empty list where the C# removes it, so a plugin ticked on stays on. Tests: the host's (built-ins, a file replacing one, the disable list; opendroneid's page and question), mp-gui's (the ten carried, the page's place by the setting, the default and the empty list through config.xml), plugins-shipped.gui, plugins.gui, plugin-manager.gui and each plugin's script, plugin-opendroneid-page.gui (the page, no form over the window), plugin-opendroneid-once.gui (asked once across a restart), check/plugins_check.js in a page.
  - **experiments/web-experiment (plugins in a page), crates/mp-plugin-host/plugins/examples/welcomedemositl.rs** (high, the owner's, finished 2026-10-05): plugins run in a web page on wasmtime's Pulley interpreter - one plugin host for desktop and browser (src/web.rs, the one file of `unsafe` by the owner's ruling), the ten built-ins compiled to Pulley bytecode by mp-gui's build script; check/plugins_check.js holds the page to the desktop's plugins-shipped.gui. And the owner's Welcome-Demo-Sitl, not in the C#, built into the browser build only and enabled there: a drawn pointer (demo_pointer.rs, through plugin.wit's demo calls; real clicks, left and right, dispatched through the window) answers any message box first, clicks SIMULATION > Multirotor, PLAN, Zoom To Vehicle on its zoom icon, Set Home Here on the map's right-click menu at the copter, four waypoints in a square sized to the map, Write - a refusal's message box answered and the demo stopped, saying why - FLY > Actions, force arm, TakeOff and its prompt's OK (armed again if the copter disarmed before the climb), Auto; then unticks itself on PLUGINS, saves, goes back to FLY, its pointer goes, and it asks for no more loops. The owner's run of 2026-10-05 found the first version leaning on the planning map being zoomed in and home known already: one waypoint at zoom 3, Write refused for want of home, everything after it behind the box; the steps above are the fix, with a map too far out for distinct waypoints a stop rather than a pile of clicks. `planner.html?demo=0` starts without it (every other browser check). Tests: crates/mp-plugin-host/tests/welcome_demo.rs (the order, the square, the waits, the rearm, a message box first, a refused Write, a map too far out, the pointer gone at the end), demo_pointer.rs's (the pointer's end), mp-os's page query, check/demo_check.js (the whole demo in a page: home and four waypoints on FLIGHT PLAN, the mission on the vehicle, armed in Auto, the demo on the disabled list, the pointer gone).
  - **crates/mp-gui/src/plugin_manager.rs, cmd_keys.rs** (high, the owner's bug of 2026-10-04, finished the same day): Ctrl+P opens the plugin manager, Plugin/PluginUI.cs - the plugins in a grid (name, author, version, file, Enabled; green, dark orange when loaded and disabled, dark red "Not loaded", a built-in plugin counting as a file in the plugins folder), Save && Close writing DisabledPlugins for the next start with the restart warning, Show Errors. By the owner's word the same day: a PLUGINS tab between LOGS and HELP, which Ctrl+P shows; an Exercise column whose Try runs each working plugin's map-menu entries; buttons sized to their text. The key reached nothing at first - gpui gives a key with nothing focused to the window's top alone - so the main window holds the keyboard whenever no control does. Tests: plugin_manager's and cmd_keys' unit tests, plugin-manager.gui, and plugin-<name>.gui for each of the seven working plugins; seen working on tridge's Mac.
  - **crates/mp-script** (med): Scripts handed MAV, MainV2, screens, Ports, Joystick: merged 2026-09-26 with the 19 review findings fixed (getWP and setWPTotal as the C#'s link requests, a dropped link and Abort end a waiting script, the named vehicle throughout) and a second review's four; fly-scripts-clr.gui passed headless, the SITL test on a settled SITL; MAV.wps and MAV.rallypoints kept in the link since 2026-10-03 (mp-link's mission_points, beside fencepoints): a script's setWPTotal empties the list named and each setWP the vehicle takes fills it - by the message that ended it, the float path's MISSION_REQUEST_INT branch filing wps whatever the list, as the C# does - as do the stream's items, RALLY_POINTs and this link's own uploads; the flight map draws them while no plan is being edited and Set WP counts them; fly-scripts-clr.gui passed headless with the five in MAV.wps and four drawn, plan-rally-legacy.gui with the rally list filled, read back and cleared
  - **crates/mp-script** (med): Fourteen shipped scripts that import clr run as they do under Mission Planner's IronPython: 10 reach their end or loop (cubeorange, example4, example5, rc - heli, rc, wipe to the end; example1, example3, example8 loop, example6 waits for its next datagram), and the four that stop do so on the C#'s own signatures - TAKEOFF and PARACHUTE LANDING APPROACH call setWPCurrent(1) where the only form takes sysid, compid and index (MAVLinkInterface.cs:2452), example2 and example10 call SubscribeToPacketType with two arguments where it takes four or five (:5567-5568) - IronPython's TypeErrors, which the shim gives in its words (crates/mp-script/tests/stock_scripts.rs records every verdict; ruled 2026-10-03: nothing in the C# would run them, and an overload it has not got is not ours to add)
- **Tests:** `crates/mp-script/tests/stock_scripts.rs` runs every `testdata/scripts/*.py` through the corpus scan and under the engine against a simulated vehicle, asserting the file count and each script's recorded verdict; `engine.rs`'s tests cover the objects, the console, the abort and the thread; `tests/gui/fly-scripts.gui` and `fly-scripts-clr.gui` the tab; `crates/mp-plugin-host/tests/{host,examples,real_plugins}.rs` the WebAssembly host over the four shipped plugins and seven examples, and `tests/gui/plugins.gui`. Not yet: `script_kill.rs` (the kill switch), `sample_extension.rs` (a wasm extension built, loaded, adding a panel and sending a command), `sandbox.rs`, `api_compat.rs`, `scripting.rs` (the reimplemented IronPython examples with identical effects).

### D17. Localization, settings and data compatibility
All UI strings through Fluent, every existing culture migrated, Crowdin flow preserved; settings storage;
and strict backward compatibility with the C# app's user data.
- **DoD:** automated `.resx` → `.ftl` conversion with a zero-string-loss report for every culture present in
  the repo; missing-translation lint in CI; the Rust app **reads and writes the existing** `config.xml`,
  `.waypoints`, `.param`, `.tlog`, mission/fence/rally files and map cache without conversion; a user can
  run both apps side by side, each in its own data directory - the Rust app's is
  `~/.local/share/MissionPlannerRust` on Linux, which imports the C#'s user files once on its first
  start (owner's ruling, PLAN.md §12 D11).
- **Replaces:** `L10N.cs`, `ExtLibs/Strings`, the per-culture `.resx` sprawl, `crowdin.bat`, settings code.
- **Today:** `mp-settings` ports the data-directory rules from `ExtLibs/Utilities/Settings.cs`,
  including the mono quirk that puts a Linux installation under `~/.local/share` rather than
  `~/Documents`, which is how it finds the C#'s `Mission Planner` directory - only ever read. This
  application's own is `~/.local/share/MissionPlannerRust` on Linux (`$XDG_DATA_HOME` when set;
  never `~/MissionPlannerRust`, where the C#'s old-approach rule would look) and
  `Documents\MissionPlannerRust` on Windows (PLAN.md §12 D11, §13.6 row 77). On the first start that finds its
  own directory missing or empty, `mp_settings::migrate` copies the C#'s `config.xml`, `poi.txt`,
  `cameras.xml`, `checklist.xml`, `warnings.xml`, `UserAlerts.json`, `authkeys.xml`, `logo.png`,
  `logo.txt` and `History` across by name, once, recording them in a marker file and leaving the
  C#'s copies as they were; the tile cache, terrain, logs and parameter metadata are left behind
  to be re-created. The GUI publishes what a start imported as `config.imported`
  (`tests/gui/config-import.gui`). `mp_settings::Config` reads and writes `config.xml` exactly as
  `Settings.Load`/`Save` do - keys sorted case-insensitively, `/` spelled `____`, a UTF-8 BOM,
  no final newline - and a test renders this machine's real file back byte for byte. The GUI
  reads the recording directory, the last link and the map type from it. The GUI holds the
  whole file as `Settings.Instance` and writes it whole on the C#'s events - start-up, the
  FLIGHT DATA and FLIGHT PLAN buttons, Connect, the close box (`MainV2.cs:1107, 1309-1323,
  1846, 2171`) - with the planner's home and panel boxes put in when the screen is deactivated
  (`FlightPlanner.cs:340-344, 2572-2612`), the quick views when chosen (`FlightData.cs:2482`),
  the map type and altitude frame when changed, the link when opened; keys sort as mono's
  en-US sort does and a key holding `/` is dropped as `Settings.Save` drops it. What the C#'s
  own `MissionPlanner.Utilities.dll` wrote under mono for the same keys
  (`tests/fixtures/config-saved.xml`, harness `SettingsOracle.cs`) is matched byte for byte,
  and `tests/gui/settings-persist.gui` restarts the application through the close box and
  finds every value back. Its own choices (window, recording, map) still live in its own file.
  **i18n's first half** (PLAN.md §13.6 row 76): `cargo xtask codegen-resx` turns `Strings.resx`
  and `FlightData.resx` and their sibling cultures into `assets/i18n/<culture>/<stem>.ftl` - 185
  and 135 language keys, ten and eighteen cultures - with `keymap.toml`, the `.resx` name → Fluent
  id of every key ever generated, immutable so Crowdin's memory and the committed `.ftl` stay
  keyed (R14), and `report.md`, the zero-loss report: every language entry of every culture file
  is a translation or a listed orphan (two in Arabic), and four translations whose placeholders
  differ from the English are named. Each message formats back through a real `FluentBundle` to
  the exact .NET string, placeholders filled. **The second half** (row 76, 2026-09-25):
  `crates/mp-gui/src/i18n.rs` embeds the 32 `.ftl` files, takes the culture from config.xml's
  `language` as `L10N.GetConfigLang` does (English when empty, the owner's ruling), falls back
  through the culture's parents to English, and the flight screen's 46 tab and button texts go
  through `fl!()`, English unchanged; a lint fails when English lacks a key a screen asks for,
  and per-culture tests hold each culture's gaps to `report.md`'s "Screens through Fluent" table.
  **Not yet:** every other screen's strings, and the Planner page's language box - held at the owner's word (English when `language` is empty; NOT_DONE_YET_MATRIX.md's D17 row).
- **Tests:** `crates/mp-settings`'s unit tests over `tests/fixtures/` (this machine's real `config.xml` rendered back byte for byte, and `config-saved.xml` from `SettingsOracle.cs` under mono) and `migrate.rs`'s over scratch directories (each artefact copied with the C#'s copy byte-identical afterwards, a second start importing nothing, a non-empty directory left alone, no C# directory), `tests/gui/settings-persist.gui` (a restart through the close box) and `config-planner.gui` (forty keys read back); the data formats are proved in their own crates - `.waypoints` in `mp-mission`, `.param` in `mp-params`, `.tlog` in `mp-mavlink` and `mp-log`, the map cache in `mp-tiles`. `xtask/tests/resx.rs` is `resx_conversion.rs` and `placeholders.rs` together: every message of every culture's `.ftl` formatted back through a `FluentBundle` to its `.resx` value (2,900-odd), nothing lost, the ids immutable, the committed assets current, and the placeholder mismatches counted in the report. `crates/mp-gui/src/i18n.rs`'s tests (the embedded files equal to `assets/i18n`, the fallback chain, the English lint, one test per culture). Not yet: `pseudolocale.rs`, and a single `data_compat.rs` over every format at once.

---

## Layer 3 — The machine that builds the machine

### D18. Translation factory and porting ledger
The industrial pipeline that converts 1.2M LOC: codegen (MAVLink XML, DSDL, param metadata, `.resx`,
WinForms `Designer.cs` → screen specs), work-unit definition, dependency-ordered waves, an agent-driven
per-file porting harness, and a machine-readable ledger tracking every one of the 3,678 source files.
- **DoD:** `ledger/ledger.csv` enumerates every C# file with the PLAN.md §6.2 columns and a state
  from `ready → claimed → ported → tested → verified → reviewed → done`, plus `deferred` and
  `dropped`; `cargo xtask ledger check` fails CI if a file is unaccounted for, a `done` row lacks
  evidence, or a `dropped` row lacks the owner's reason; `cargo xtask ledger status` prints progress
  in retired C# lines; one full wave executed end to end to prove the throughput rate; the
  porting-agent contract (prompt + test + differential check + review gate) documented and versioned.
- **Today:** the ledger exists and passes its own check: 3,678 rows, one per `.cs` file, tier from a
  classifier that names its evidence per vendored root, sha256 for staleness; 2,618 rows `ready`, 187 `tested`, 9 `ported` and 864 `dropped` (2026-10-03) - 117,472 of 1,208,836 C# lines past `ready`: `CurrentState.cs`,
  `MAVLinkInterface.cs`, `FlightData.cs`, `FlightPlanner.cs`, `LogBrowse.cs`, `HUD.cs`, `Grid.cs`
  and `GridUI.cs`, `clipper.cs`, the log conversions, `srtm.cs`, `MAVFtp.cs`, the Comms classes,
  `Settings.cs`, `georefimage.cs`, `BoardDetect.cs`, the manifests, the map providers and
  projection, and every `Config*.cs` page ported whole (PLAN.md §13.6 row 72). The 864 `dropped` rows, 149,229 lines: 461 are the files nothing calls, each with one of four reasons re-derived from the C# tree by `xtask/tests/dead_csharp.rs` (row 89), 354 are the owner's rulings (PLAN.md §12 D13), 44 are separate programs, 4 declare nothing, and one is GeoidHeightsDotNet's console program. Existing Rust work
  is credited only when re-entered with evidence (PLAN.md §5.2); `done` needs the review gate,
  which nothing has passed. `init` is deterministic and `refresh` keeps hand-edited columns.
  `target_crate` and `evidence` are filled on all 196 rows past `ready` and the class columns on 193 of them; `unit_id` and `deps` are empty. Generators: `mavlink`, `param_meta`, `modes` and `resx`
  (`.resx` → `.ftl`, PLAN.md §13.6 row 76) in `xtask/src/codegen/`. Not started: DSDL, screen
  specs, `xtask next`, the contract dry run.
- **Tests:** `xtask/tests/ledger.rs` (22 tests: the schema on a fixture tree and the real ledger, every one of the 3,678 files exactly once with a valid tier, disposition and state, evidence on a `done` row, `init` byte-deterministic, `refresh` preserving hand-edited columns) `dead_csharp.rs` (every `dropped` reason re-derived from the C# tree, skipping without it) and `graph.rs`. `xtask/tests/codegen.rs` (2026-10-03: the committed `modes.rs`, `param_meta_copter.rs` and the `all` dialect each regenerated in memory, formatted as the xtask formats them and compared byte for byte, the first differing line named; skipped without the reference tree; `resx` has `--check` and `resx.rs`). Not yet: a dry run of the porting-agent contract.

### D19. Verification suite
Proof that the Rust app behaves like the C# original before anyone flies behind it.
- **DoD:** differential harness running the C# reference headless (Mono/.NET on Linux) against the Rust
  implementation over a golden corpus of tlogs, dataflash logs and param dumps, diffing decoded output;
  fuzzing on all parsers; proptest round-trips; numeric-equivalence tests with stated tolerances;
  ArduPilot SITL integration tests in CI driving scripted missions; UI snapshot tests on headless GPU;
  criterion perf gates that **fail the build on regression**; a documented hardware-in-the-loop checklist
  signed off before each release.
- **Today:** the differential corpus against `MAVLink.dll` is a `cargo test` (the workflow that
  would run it in CI has never run - no remote); eleven oracle harnesses under
  `tools/csharp-reference/` run the C#'s own code under mono - the MAVLink dump, the grids, GridUI,
  the log conversions, projection, `CurrentState`, the three transports, MAVFTP, SRTM,
  geo-referencing, the planner's handlers - with goldens under `testdata/`, eight `regen*.sh`
  scripts, and `MpFtp.cs` and `PlannerOracle.cs` built and run as their headers say; thirteen `cargo-fuzz` targets build and run clean with committed seed corpora - the first five (34.3 M executions at their last short pass) and, since 2026-10-03, eight for the readers written since the first soak (the .bin reader and converter, the log analyzer, zip, fence/rally/polygon, DBF, Intel HEX, the firmware catalogues, SLCAN into DroneCAN), each run 45 s and seeded from the test data; `dbf_table` found a reader allocating for the header's record count, fixed with the input as a test - and the 24-hour soaks of `frame_parse` and `message_decode` ended clean 2026-09-24 16:01Z; the
  same properties run bounded on stable in `cargo test --workspace`, so a target cannot rot
  uncompiled; SITL integration tests run behind `--ignored`; 202 GUI scripts assert the application's own facts through `tools/gui-test.sh` (every one has run; README.md carries the count passing at its latest full run, and the whole suite ran in the Windows VM on 2026-09-27 at 16895c7: 148 pass, 22 pass over budget, 4 fail - three budgets since retimed, one lost wheel notch since spaced - 3 skip by design, of 177), and `tools/gui-headless.sh` runs them on a virtual X display
  with Mesa's lavapipe drawing the application, so a run needs no desktop and no pointer (since
  2026-09-26; the storm gate still wants the GPU). The mutation self-test is not written, no
  UI snapshot of a screen exists (the HUD's goldens are a software rasteriser's, not gpui's), and the perf gates are six benches whose thresholds hold in release on a
  quiet machine, not in CI. The layout guard (2026-10-03, the Mission box row under D11): every GUI run ends by demanding `layout.hidden` 0 - no important control of the screen clipped by the window or by its box - and `crates/mp-gui/tests/layout.rs` asserts it screen by screen.
- **Finished since the audit** (moved from NOT_DONE_YET_MATRIX.md on 2026-10-03, which holds only unfinished work from that day):
  - **crates/mp-video/src/gstreamer.rs** (med): On Windows the HUD's and the gimbal's GStreamer pipelines gave no frames in the suite (fly-gstreamer, fly-gimbal-video): not the planner - GStreamer's first-run plugin scan outlasts a script's wait in the VM and a launcher stopped mid-scan writes no registry. The Windows runner builds the registry once, before the scripts; both passed from an empty registry 2026-09-27
- **Tests:** this deliverable *is* the test infrastructure, and today it is proven by the oracles' own regeneration (`tools/csharp-reference/regen*.sh` reproduce every golden byte for byte) and by `crates/mp-fuzz-checks/tests/bounded.rs`. Not yet: `tests/harness_selftest.rs`, the mutation test that plants a bug - an off-by-one in a parser, a swapped lat/lon, a wrong unit, a dropped retry, a 2 ms frame-budget regression - and asserts the differential harness, the fuzzers, the SITL suite and the perf gates each **fail**; a harness that cannot detect a planted bug is not a harness. Also not yet: golden-corpus integrity checks, C#-reference reproducibility in CI, flake tracking.

### D20. Release, packaging and operations
Shipping the thing: signed installers per OS, auto-update, crash reporting, telemetry opt-in, docs and the
migration guide for existing Mission Planner users.
- **DoD:** one command produces signed artefacts for Windows (MSI/EXE), macOS (notarised `.dmg`) and Linux
  (AppImage + `.deb`); auto-update channel with rollback; symbolicated crash reports; first-run migration
  imports existing Mission Planner settings and caches; user-facing docs and a "what changed" guide published;
  **cold start < 500 ms**, installer < 150 MB.
- **Replaces:** `Updater/`, `ExtLibs/Installer`, `wix/`, `Msi/`, `MAC/`, `MissionPlanner.sh`, `build*.bat`.
- **Today:** `tools/package.sh` builds a stripped release binary and reports what a machine needs to
  run it, and says itself it is not a package. The update is ported: `mp-update` is
  `Utilities/Update.cs` (the version check, the MD5-listed download of changed files as `.new`)
  and `Updater/Program.cs` (`headless-planner update-apply`, the swap and restart), driven from
  the HELP screen (`GCSViews/Help.cs`, ported) and `MainV2`'s once-a-day check; the channel's
  addresses are settings of `app.config`'s names, empty until one is published for this program.
  Crash reporting is `Program.cs`'s `handleException` as a panic hook: the report (the C#'s
  `postData` fields, the backtrace symbolicated by the release profile's debug info) written under
  the data directory's `crash-reports/`, the next start asking "Report this Error???" and posting
  it with the user's message to `CrashReportUrl` (`crash.rs`). `tools/package.sh deb` writes the
  Debian package - both binaries, the desktop entry and icon, the licence, `Depends` read off what
  the planner links - which `crates/mp-cli/tests/package_smoke.rs` installs into a clean
  `ubuntu:noble` container, runs and removes. `tools/cold-start.sh` measures the start: 200 ms to
  the third painted frame, 465 ms start to exit, medians of 20 warm runs of the release build on
  lavapipe (2026-10-03), under the 500 ms asked for. `docs/migration-guide.md` is the "what changed"
  guide for Mission Planner's users. Unsigned; the AppImage, the MSI and the `.dmg` are not
  started.
- **Finished since the audit** (moved from NOT_DONE_YET_MATRIX.md on 2026-10-03, which holds only unfinished work from that day):
  - **dist/** (high): Linux release app rebuilt at today's main (owner's ask): built 2026-09-26 14:25 at 962cd28, thin LTO and no debuginfo to fit beside the VM (fat LTO needs the VM off)
- **Tests:** `crates/mp-cli/tests/package_smoke.rs` (the `.deb` installed into a clean `ubuntu:noble` container, both binaries run, removed, nothing left); `tests/gui/help.gui` against `tests/gui/update-server.py` (the check, the question, the download) and `crash-report.gui` (a forced panic's report and the next start's question); `mp-update`'s and `crash.rs`'s unit tests; `tools/cold-start.sh` (200 ms to the third frame, 465 ms start to exit, 2026-10-03). Not yet: `package_smoke` for Windows and macOS (no MSI or `.dmg` exists), `tests/update.rs` between two signed builds (nothing is signed), `tests/migration.rs` against a real Mission Planner data directory (the import's tests in `mp-settings` run on scratch directories), and a `benches/cold_start.rs` gate (the script measures, nothing fails the build).

---

### D21. Native in-process plugin host

Functional equivalence to `PluginLoader.cs`: loading arbitrary third-party code into the process
with full trust, the way Mission Planner does today. Sequenced **after everything else** - D16's
sandboxed WASM extensions are the safe default and cover most needs; this covers the rest, because
100% of Mission Planner includes the part that can shoot you in the foot.
- **Scope:** native `cdylib` plugins (`.so`/`.dll`/`.dylib`) loaded via `libloading` behind a
  versioned C ABI, with a host vtable for telemetry, sending, and registering panels and menu
  actions; plus load-time compilation of loose Rust source when a toolchain is present, which is
  the direct analogue of Roslyn compiling loose `.cs`.
- **Explicitly not promised:** loading existing *C# plugin assemblies*. That needs a CLR. Existing
  plugins are rewritten against this API or D16's; a migration guide ships with it.
- **DoD:** an out-of-tree sample plugin loads, registers a panel and a menu action, reads telemetry
  and sends a command; an ABI-version mismatch is refused with a clear message rather than a crash;
  a panicking plugin is contained at the boundary and named in the resulting report; `--safe-mode`
  loads nothing; loading is per-plugin opt-in with consent recorded on disk.
- **Today:** the native host is not built. The owner's ruling (PLAN.md §12 D22, 2026-09-25) tried WebAssembly first: `experiments/wasm-plugin-host` (row 95: 65 ms to load, under a microsecond a call) gave the verdict, and `crates/mp-plugin-host` - the name this row planned for the native host, taken by the WebAssembly one - is the real host since 2026-09-26 (D16, row 96): wasmtime's component model, the C#'s `Plugin` lifecycle and `PluginHost` surface as a WIT world, `*.wasm` beside the executable, the four shipped plugins and seven examples ported to it. This native `cdylib` host is not needed unless a plugin the owner wants cannot be carried by the WebAssembly one; packet subscription, sockets and the main window's members are what it cannot carry today, listed at the site.
- **Tests:** none exist. Planned: `crates/mp-plugin-host/tests/load.rs` builds the sample plugin in CI and loads it;
  `tests/abi.rs` asserts a deliberately mismatched ABI version is refused; `tests/panic.rs` asserts
  a plugin that panics in each callback does not terminate the host and is reported by name;
  `tests/safe_mode.rs` asserts nothing loads; `tests/source_plugin.rs` compiles and loads a loose
  `.rs` plugin, and is skipped-with-a-message rather than silently passing when no toolchain exists.
- **Replaces:** `Plugin/PluginLoader.cs`, `Plugin/Plugin.cs`, the full-trust half of `Plugins/`.

## Cross-cutting acceptance gates

Every deliverable above must also satisfy:

| Gate | Requirement |
|---|---|
| Platform parity | One gate per operating system, judged separately: a deliverable is *Done on Linux*, *Done on Windows* or *Done on macOS* when its `Tests:` artefacts pass on that OS, and the summary table carries the three columns. Bring-up is on Linux first, so *Done* on Linux with *Not started* elsewhere is the expected state during bring-up and is not a defect. The deliverable as a whole is done only when all three columns say so |
| Performance | Meets its stated numeric budget, measured by a checked-in benchmark, gated in CI |
| No-panic path | Telemetry ingest, state update and render paths contain no `unwrap`/`expect`/panic |
| Coded test suite | The `Tests:` artefacts listed for that deliverable exist, run in CI on every PR, and fail loudly — no deliverable reaches *Feature complete* without them |
| Differential proof | Behaviour compared against the C# original on real data where a reference exists |
| Ledger | Every C# source file it replaces is marked in the D18 ledger |
| Licence hygiene | GPLv3 compliance, upstream attribution recorded, `cargo-deny` clean |

## Numeric targets (the "extreme performance" contract)

| Metric | Target | Measured (2026-09-24) |
|---|---|---|
| Cold start to connected UI | < 500 ms | **200 ms** to the third painted frame (median of 20 warm runs of the release build under `MP_SMOKE`, max 601), 465 ms process start to exit (max 1,230), on Xvfb with lavapipe, 2026-10-03 (`tools/cold-start.sh`; PLAN.md section 8.2) |
| Idle CPU (connected, 10 Hz telemetry) | < 1 % of one core | unmeasured |
| Packet-to-pixel latency (p99) | < 16 ms | **14.7 ms** (p50 9.8, max 18.4; 2 of 569 over) on the release build under a 200 Hz storm, 2026-09-26, with the link's 5 ms snapshot cadence; the frame alone is p99 3.8 ms |
| Stick input to packet on the wire (p99) | < 5 ms | 0.152 ms on an in-process fake device; no real device attached |
| MAVLink decode throughput | > 1 M msg/s/core, 0 allocations per packet | 9.5 M frames/s framing and CRC on one core (`benches/decode.rs`, release, 2026-09-24); 0 allocations proven over 211,638 typed decodes |
| 1 GB dataflash log open | < 2 s to first plot | 0.61-0.62 s to first plot on a 1.07 GB log (`benches/parse_1gb.rs`, quiet machine) |
| Log plot scrub, 10 M points | 120 fps | p99 3.41 ms a cursor step over eight 10 M-sample series, CPU side (`benches/scrub_10m.rs`, 2026-09-25); GPU unmeasured |
| Map pan/zoom, 1 M-point track + 10 k markers | 120 fps | p99 5.42 ms for the following frame, CPU side (`benches/pan_zoom.rs`); GPU unmeasured |
| Resident memory, 1 vehicle + map + 1 GB log open | < 1 GB | 1.62 GB peak while opening the 1.07 GB log - **over the target**; the rest unmeasured |
| Concurrent vehicles | ≥ 50 without frame drops | 50 systems and 56 components through one link at 2.8 µs a frame (`tests/routing.rs`), without a frame drawn |

---

*Companion document: `PLAN.md` (roadmap, crate graph, technology decisions, risk register). These 21
deliverables are the "what"; PLAN.md is the "in what order, and how we prove it".*
