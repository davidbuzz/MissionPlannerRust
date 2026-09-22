# Mission Planner → Rust: 20 Core Deliverables

**Goal:** a file-complete, code-complete reimplementation of [ArduPilot Mission Planner](referneces/missionplanner)
(C# / .NET Framework 4.7.2 / WinForms — 3,678 `.cs` files, 1,208,836 LOC, ~93 `.csproj`) in Rust, that is
**extremely fast**, **multi-platform** (Windows / Linux / macOS), and **GPU-accelerated** end to end.

**UI/runtime stack:** Zed's ecosystem — `gpui` 0.2.2 (Apache-2.0, published, wgpu-backed) + `wgpu` 29,
with platform backends `gpui_linux` / `gpui_windows` / `gpui_macos` (and `gpui_web` as the wasm option).
Reference clone: [referneces/zed](referneces/zed).

**Licence:** the port is a derivative of GPLv3 Mission Planner → the workspace ships **GPLv3**.
`gpui` (Apache-2.0) is inbound-compatible. Per-crate licences from `zed` must be checked individually.

## Summary

| # | Layer | Deliverable | Priority | Implementation | Testing |
|---|---|---|---|---|---|
| [D1](#d1-workspace-crate-graph-and-build-system) | 0 | Cargo workspace and crate graph | P0 | In progress | Unit |
| [D2](#d2-mavlink-protocol-crate) | 0 | MAVLink protocol codec crate | P0 | In progress | Differential vs C# |
| [D3](#d3-transport-layer) | 0 | Serial, TCP, UDP, BLE transports | P0 | In progress | Unit |
| [D4](#d4-link-engine-the-mavlinkinterface-equivalent) | 0 | Link engine, protocol machines | P0 | In progress | Differential vs C# |
| [D5](#d5-vehicle-state-model--telemetry-bus) | 0 | Vehicle state snapshot bus | P0 | In progress | Differential vs C# |
| [D6](#d6-ui-kit-on-gpui) | 1 | gpui widget kit | P0 | In progress | Unit |
| [D7](#d7-gpu-render-core) | 1 | Shared wgpu render core | P0 | Spiked | Unit |
| [D8](#d8-map-engine) | 2 | GPU slippy map engine | P0 | Spiked | Unit |
| [D9](#d9-hud--primary-flight-display) | 2 | GPU HUD with video | P0 | In progress | Unit |
| [D10](#d10-flight-data-screen) | 2 | Flight Data operations screen | P0 | Not started | Not started |
| [D11](#d11-flight-planner-screen) | 2 | Mission and survey planner | P0 | In progress | Differential vs C# |
| [D12](#d12-configuration--tuning-screens) | 2 | Parameter config and tuning | P1 | In progress | Unit |
| [D13](#d13-initial-setup-calibration-and-firmware) | 2 | Setup, calibration, firmware flashing | P1 | Not started | Not started |
| [D14](#d14-log-engine-and-analysis) | 2 | Dataflash log parsing, plots | P1 | In progress | Unit |
| [D15](#d15-can-peripherals-and-outboard-features) | 2 | DroneCAN, peripherals, video, joystick | P2 | Not started | Not started |
| [D16](#d16-extension-and-scripting-system) | 2 | Python scripting, WASM extensions | P2 | Not started | Not started |
| [D17](#d17-localization-settings-and-data-compatibility) | 2 | i18n, settings, data compatibility | P1 | Not started | Not started |
| [D18](#d18-translation-factory-and-porting-ledger) | 3 | Translation factory, file ledger | P0 | In progress | Unit |
| [D19](#d19-verification-suite) | 3 | Differential, SITL, fuzz verification | P0 | In progress | Differential vs C# |
| [D20](#d20-release-packaging-and-operations) | 3 | Installers, updates, crash reporting | P1 | Not started | Not started |
| [D21](#d21-native-in-process-plugin-host) | 2 | Native in-process plugin host | P3 | Not started | Not started |

**Layer** 0 = foundation (protocol/transport/state) · 1 = rendering and UI foundation · 2 = the application · 3 = the machine that builds the machine.
**Priority** P0 = nothing ships without it · P1 = required for feature parity · P2 = required for 100% completeness, sequenced last · P3 = the last thing of all, after P2.
**Implementation** Not started → Spiked → In progress → Feature complete → Done.
**Testing** Not started → Unit → Differential vs C# → Gated in CI → HIL signed off.

## Test policy (applies to all 20 deliverables)

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
other 17 achievable at 1.2M-LOC scale.

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
- **Tests:** `xtask/tests/graph.rs` parses `cargo metadata` and asserts the layer rules (no crate depends upward, no cycles, no UI crate in the telemetry path); `xtask/tests/licences.rs` wraps `cargo-deny` and fails on a non-GPLv3-compatible dependency; `xtask/tests/build_budget.rs` asserts cold `cargo check` and incremental rebuild stay under budget; `trybuild` UI tests for every proc-macro crate; CI matrix builds all three OSes on every PR.

### D2. MAVLink protocol crate
Generated message set (common + ardupilotmega + all dialects Mission Planner ships), MAVLink v1/v2,
signing, zero-copy frame parse/serialize.
- **DoD:** generated from the same XML the C# side uses, checked in as generated output + regenerable by
  `cargo xtask codegen`; round-trip property tests (proptest) and a `cargo-fuzz` target with 24 h clean;
  decodes a 1 GB tlog at **> 1 M messages/s single-threaded**, zero heap allocations per packet
  (verified by an allocation-counting test).
- **Replaces:** `ExtLibs/Mavlink` (40,718 LOC, machine-generated).
- **Tests:** `crates/mavlink/tests/roundtrip.rs` proptest encode→decode identity over **every** generated message type; `tests/golden_decode.rs` diffs decoded fields against C#-produced golden JSON for a corpus of real tlogs; `tests/signing.rs` for MAVLink2 signature accept/reject vectors; `tests/truncation.rs` for v2 zero-trimming edge cases; `fuzz/fuzz_targets/parse_frame.rs` (24 h clean required before D2 is done); `tests/no_alloc.rs` uses a counting global allocator to assert zero allocations per packet; `benches/decode.rs` gates the >1 M msg/s target.

### D3. Transport layer
`serial | TCP | UDP | BLE | NTRIP | websocket | file-replay`, device enumeration and hotplug on all three
OSes, plus bootloader/flashing transports (px4uploader, DFU, ADB).
- **DoD:** trait-based transport with a deterministic in-memory/replay implementation; serial enumeration
  returns identical device lists to the C# app on the same hardware (Windows COM, Linux `/dev/serial/by-id`,
  macOS `cu.*`); reconnect and surprise-unplug covered by tests; NTRIP/RTCM injection verified against a
  live caster; **≤ 1 ms** added latency over raw OS read.
- **Replaces:** `ExtLibs/Comms` (8,249), `ExtLibs/Ntrip`, `ExtLibs/Zeroconf`, `ExtLibs/px4uploader`,
  `ExtLibs/NetDFULib`, `ExtLibs/WinUSBNet`, `ExtLibs/SharpAdbClient`, `Radio/`, `SikRadio/`.
- **Tests:** `crates/transport/tests/loopback.rs` per transport (serial via a PTY pair / com0com, TCP, UDP, websocket, file-replay); `tests/faults.rs` fault-injection over a mock transport (drop, duplicate, reorder, partial write, mid-frame disconnect); `tests/enumerate.rs` parses checked-in per-OS device fixtures (Windows registry dumps, Linux udev/sysfs trees, macOS IOKit dumps) and asserts the device list; `tests/hotplug.rs` simulated surprise-unplug and reconnect; `tests/ntrip.rs` against an in-process mock caster; `benches/latency.rs` gates the ≤1 ms overhead target.

### D4. Link engine (the `MAVLinkInterface` equivalent)
Per-link packet pump, routing/forwarding, and the high-level protocol state machines: parameters,
mission/rally/fence up- and download, MAVFTP, log download, command_long/ack, requests and retries.
- **DoD:** every protocol state machine is an explicitly-tested state machine (not ad-hoc retry loops);
  multi-vehicle `sysid/compid` routing with N ≥ 50 simultaneous vehicles; full param download from a real
  ArduPilot SITL matches the C# app's result set exactly; packet loss/timeout behaviour covered by a
  fault-injection replay harness.
- **Replaces:** `ExtLibs/ArduPilot/Mavlink/*` (MAVLinkInterface, MAVState, MAVList).
- **Tests:** one test module per protocol state machine — `tests/params.rs`, `tests/mission.rs`, `tests/fence_rally.rs`, `tests/ftp.rs`, `tests/log_download.rs` — each driven by recorded packet traces plus a scripted peer; `tests/retries.rs` injects timeouts, out-of-order acks and partial transfers and asserts convergence or a clean error; `tests/routing.rs` drives 50 simultaneous sysid/compid vehicles through one link; `tests/sitl_params.rs` (feature `sitl`) downloads the full param set from ArduPilot SITL and diffs it against the C# app's dump.

### D5. Vehicle state model + telemetry bus
The `CurrentState` equivalent: decoded, UI-facing vehicle state, published as lock-free immutable
snapshots so the renderer never blocks on the I/O thread.
- **DoD:** single-writer/multi-reader snapshot (arc-swap / triple-buffer) with **zero locks on the render
  path**; no allocation in the ingest→state path; every C# `CurrentState` field accounted for, with a
  checked-in field-coverage report; unit-typed geodesy (no bare `f64` lat/lon) throughout.
- **Replaces:** `ExtLibs/ArduPilot/CurrentState.cs`, the C# event/timer/`Invoke` marshalling model.
- **Tests:** `tests/field_coverage.rs` reads the D18 ledger and fails if any C# `CurrentState` field lacks a Rust counterpart; `tests/decode_to_state.rs` replays golden tlogs and diffs the resulting state timeline against C# output; `tests/concurrency.rs` stress-tests the snapshot bus (writer at 1 kHz, 8 readers) asserting no torn reads and no reader stall, with a `loom` model of the publish path; `tests/no_alloc_ingest.rs` allocation counter over the ingest→state path; `benches/snapshot.rs` gates publish and read latency.

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
- **Tests:** `crates/ui/tests/snapshots/**` headless render snapshots for every widget in light and dark themes (perceptual diff with a tolerance, golden images in `testdata/ui/`); `tests/interaction.rs` keyboard navigation, focus order, tab stops and action dispatch via gpui's test executor; `tests/grid.rs` virtualised 100 k-row grid correctness (scroll, sort, select, resize) plus `benches/grid_scroll.rs` gating 120 fps; `tests/theme_import.rs` loads real `*.mpsystheme` fixtures and asserts the resolved palette; `tests/hidpi.rs` layout at 1x/1.5x/2x scale.

### D7. GPU render core
Shared `wgpu` layer under everything visual: device/queue sharing with gpui (or offscreen render-to-texture
if gpui refuses to share — **decision gate, see PLAN.md**), render graph, instanced markers, GPU line
tessellation, glyph atlas labels, offscreen targets, frame pacing, and a software/remote-desktop fallback.
- **DoD:** a custom wgpu viewport composites correctly inside a gpui window on Windows, Linux (X11 +
  Wayland) and macOS; documented per-frame budget with a live profiler overlay; degrades gracefully under
  RDP/VNC and on llvmpipe; headless rendering works in CI for snapshot tests.
- **Replaces:** GDI+/`System.Drawing`, `OpenTK`/`GLControl`, `SkiaSharp`, `ExtLibs/MissionPlanner.Drawing`
  (17,602), `ExtLibs/SvgNet`, `ExtLibs/LibTessDotNet`.
- **Tests:** `crates/render/tests/headless.rs` renders every primitive on lavapipe (Linux), WARP (Windows) and the macOS software path, diffing against golden PNGs; `tests/shaders.rs` compiles every WGSL shader for all backends and asserts pipeline creation; `tests/viewport_composite.rs` proves a custom wgpu viewport composites correctly inside a gpui window (this is the spike that gates the whole GPU goal — it becomes a permanent regression test); `benches/frame.rs` per-layer frame budget; `tests/fallback.rs` forces the software path and asserts correct output.

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
- **Tests:** `tests/projection.rs` round-trips a fixture grid of coordinates against ProjNet/GDAL reference values asserting <1 mm error; `tests/tilecache.rs` cache hit/miss/evict/corrupt-entry recovery and compatibility with the existing Mission Planner cache layout; `tests/overlays.rs` golden-image renders of tracks, polygons, fences and marker clusters; `tests/editing.rs` drag/snap/rubber-band interaction via the test executor; `tests/offline.rs` asserts full function with the network disabled; `benches/pan_zoom.rs` gates 120 fps with a 1 M-point track + 10 k markers.

### D9. HUD / primary flight display
GPU artificial horizon, tapes, compass, gauges, warnings, with live video underlay and OSD-style overlays.
- **DoD:** pixel-comparable to the C# HUD (side-by-side review signed off), **< 16 ms packet-to-pixel**
  at the 99th percentile, runs at 120 fps while using < 3 % CPU; video underlay with hardware decode.
- **Replaces:** `Controls/HUD*.cs`, `Controls/` PFD widgets.
- **Tests:** `tests/hud_golden.rs` renders recorded attitude/telemetry sequences and perceptually diffs every frame against golden images, including the degenerate cases (gimbal-lock attitudes, NaN/absent fields, GPS loss, failsafe banners); `tests/latency.rs` timestamps packet-in to frame-presented and gates the <16 ms p99 target; `tests/video_underlay.rs` decodes a fixture stream and asserts composition order and hardware-decode fallback.

### D10. Flight Data screen
The live operations screen: HUD + map + quick view + tuning graph + actions + messages + status tabs,
servo/RC, and the vehicle action buttons.
- **DoD:** every tab, button and action of the C# `GCSViews/FlightData` present and behaviourally verified
  against SITL; layout persists; no UI stall > 8 ms during a 200 Hz telemetry storm.
- **Replaces:** `GCSViews/FlightData*` and its dependents.
- **Tests:** `tests/action_coverage.rs` enumerates every C# `FlightData` control and action from the D18 ledger and fails on anything unimplemented; `tests/sitl_ops.rs` drives arm/disarm/mode-change/RTL/guided-goto against SITL and asserts resulting vehicle state; per-tab UI snapshots; `tests/layout_persist.rs` save/restore of the screen layout; `benches/telemetry_storm.rs` pumps 200 Hz telemetry and asserts no frame exceeds 8 ms.

### D11. Flight Planner screen
Waypoint/mission editing, survey grid generation, fences and rally points, terrain and altitude handling,
KML/DXF/shapefile import-export, geotagging hand-off.
- **DoD:** missions produced by the Rust planner are **byte-identical** to the C# planner for a corpus of
  saved `.waypoints` files and grid parameter sets; all survey-grid algorithms numerically verified;
  reads and writes the existing file formats unchanged.
- **Replaces:** `GCSViews/FlightPlanner*`, `Grid/` (3,750), `ExtLibs/SimpleGrid`, `ExtLibs/Gridv2`,
  `ExtLibs/SharpKml`, `ExtLibs/KMLib`, `ExtLibs/netDxf` (65,129), `GeoRef/`, `NoFly/`.
- **Tests:** `tests/mission_bytes.rs` loads a corpus of real `.waypoints`/`.mission` files, round-trips them and asserts **byte identity**; `tests/grid_vectors.rs` survey-grid generation against golden outputs from the C# `Grid`/`Gridv2` for a matrix of polygon/angle/overlap/terrain inputs; `tests/kml_dxf.rs` import→export round-trip against fixture files; `tests/terrain.rs` altitude-following maths against golden vectors; `tests/sitl_upload.rs` uploads missions, fences and rally points to SITL and reads them back.

### D12. Configuration & tuning screens
The full parameter system — tree/list/advanced editors driven by parameter metadata — plus every
`Config*.cs` panel (flight modes, failsafes, battery, compass, ESC, gimbal, ADSB, EKF, …).
- **DoD:** parameter metadata generated from `ParameterMetaDataBackup.xml` / `ParameterFactMetaData.xml` /
  `apm.pdef.xml` at build time; every C# config panel enumerated with a checked-in coverage ledger at 100 %;
  param save/restore round-trips against SITL; `.param` files interoperate with the C# app.
- **Replaces:** `GCSViews/ConfigurationView/*` (the bulk of 67,553 LOC in `GCSViews/`).
- **Tests:** `tests/metadata_codegen.rs` asserts the generated parameter metadata matches the source XML and compiles; `tests/panel_coverage.rs` fails if any C# `Config*.cs` panel is missing from the Rust implementation (ledger-driven); `tests/param_roundtrip.rs` writes and re-reads every parameter type against SITL including bitmask/enum/float edge values; per-panel UI snapshots; `tests/param_file_compat.rs` reads and writes `.param` files produced by the C# app byte-for-byte.

### D13. Initial setup, calibration and firmware
Wizards and calibration routines (accel, compass/mag-cal, radio, ESC, frame, sensors) and the firmware
path: board detect, firmware catalogue, upload via px4/DFU/serial bootloaders.
- **DoD:** calibration maths (ellipsoid fit, `MagCalib`, accel cal) reproduces C# results to
  **1e-6 relative**, proven by golden-vector tests; board detection matches `MissionPlannerTests`
  `DetectBoardTest` cases; a real board flashes successfully on all three OSes.
- **Replaces:** `GCSViews/InitialSetup/*`, `MagCalib.cs`, `ExtLibs/ArduPilot` firmware code (23,564).
- **Tests:** `tests/magcal_vectors.rs` and `tests/accelcal_vectors.rs` assert 1e-6 relative agreement with golden outputs captured from the C# `MagCalib`/calibration code over recorded sensor datasets, including ill-conditioned inputs; `tests/board_detect.rs` ports the existing `MissionPlannerTests` `DetectBoardTest` cases plus USB descriptor fixtures for every supported board; `tests/firmware_upload.rs` runs against an in-process mock px4/DFU bootloader asserting the exact byte protocol and checksum behaviour; `tests/firmware_catalogue.rs` parses real firmware manifests.

### D14. Log engine and analysis
Dataflash (`.bin`/`.log`) and tlog parsing, log download, graphing, LogAnalyzer rules, DSP/FFT, exports
(`.mat`, CSV, KML), EXIF geotagging.
- **DoD:** memory-mapped columnar parse of a **1 GB dataflash log in < 2 s**, then scrub a **10 M-point**
  multi-series plot at **120 fps** with GPU line rendering + LOD; parsed field values match the C# parser
  exactly across a corpus of real logs; FFT output matches `Exocortex.DSP` within float tolerance.
- **Replaces:** `Log/` (9,971), `LogAnalyzer/`, `graphs/`, `ExtLibs/ZedGraph` (52,265),
  `ExtLibs/Exocortex.DSP`, the used subset of `ExtLibs/alglibnet` (251,616 — audit what is actually called),
  `ExtLibs/MetaDataExtractorCSharp240d` (17,800), `ExtLibs/ICSharpCode.SharpZipLib` + `zlib.net` + `7zip`.
- **Tests:** `tests/parser_diff.rs` parses a corpus of real dataflash and tlog files and diffs every decoded field against the C# parser's output; `fuzz/fuzz_targets/dataflash.rs` and `tlog.rs` asserting no panic and no unbounded allocation on corrupt logs (truncated, bit-flipped, wrong-endian, fabricated FMT messages); `tests/fft.rs` compares against `Exocortex.DSP` golden spectra; `tests/exports.rs` `.mat`/CSV/KML round-trips; `benches/parse_1gb.rs` gates <2 s to first plot and `benches/scrub_10m.rs` gates 120 fps scrubbing.

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
- **Tests:** `tests/dsdl_roundtrip.rs` proptest over every generated DroneCAN type; `tests/node_sim.rs` drives a simulated CAN node through enumerate/param-edit/firmware-update; `tests/joystick.rs` uses a virtual HID device fixture to assert mapping, expo/deadzone maths and <5 ms end-to-end latency; `tests/tracker.rs` and `tests/swarm.rs` against SITL; `tests/video_pipeline.rs` smoke-tests each capture/decode backend per OS; `tests/feature_ledger.rs` fails if a feature in this bucket is neither implemented nor explicitly marked dropped.

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
- **Tests:** `tests/stock_scripts.rs` runs every `testdata/scripts/*.py` against a simulated
  vehicle and asserts each either completes or fails with a recorded, reviewed reason - the file
  count is asserted too, so a script silently disappearing from the corpus fails;
  `tests/script_kill.rs` asserts an infinite loop is terminated by the kill switch within a bounded
  time; `tests/sample_extension.rs` builds the sample extension to wasm in CI, loads it, and asserts it can add a panel, subscribe to telemetry and send a command; `tests/sandbox.rs` asserts a malicious or panicking extension cannot crash, block or read outside its sandbox (infinite loop, OOM, filesystem escape, host-call abuse); `tests/api_compat.rs` loads extensions built against older API versions; `tests/scripting.rs` runs a fixture script corpus including the reimplemented IronPython examples and asserts identical effects.

### D17. Localization, settings and data compatibility
All UI strings through Fluent, every existing culture migrated, Crowdin flow preserved; settings storage;
and strict backward compatibility with the C# app's user data.
- **DoD:** automated `.resx` → `.ftl` conversion with a zero-string-loss report for every culture present in
  the repo; missing-translation lint in CI; the Rust app **reads and writes the existing** `config.xml`,
  `.waypoints`, `.param`, `.tlog`, mission/fence/rally files and map cache without conversion; a user can
  run both apps against the same data directory.
- **Replaces:** `L10N.cs`, `ExtLibs/Strings`, the per-culture `.resx` sprawl, `crowdin.bat`, settings code.
- **Tests:** `tests/resx_conversion.rs` asserts zero string loss for every culture present in the C# repo and fails on any English key without a Rust counterpart; `tests/placeholders.rs` asserts argument arity and type agreement between every translation and its English source; `tests/pseudolocale.rs` renders screens in a pseudo-locale to catch truncation and hard-coded strings; `tests/data_compat.rs` reads real `config.xml`, `.waypoints`, `.param`, `.tlog` and map-cache fixtures produced by the C# app, writes them back, and asserts byte equality — the both-apps-same-data-directory guarantee.

---

## Layer 3 — The machine that builds the machine

### D18. Translation factory and porting ledger
The industrial pipeline that converts 1.2M LOC: codegen (MAVLink XML, DSDL, param metadata, `.resx`,
WinForms `Designer.cs` → screen specs), work-unit definition, dependency-ordered waves, an agent-driven
per-file porting harness, and a machine-readable ledger tracking every one of the 3,678 source files.
- **DoD:** `ledger.toml`/`.json` enumerates every C# file with status
  (`ported | codegen | replaced-by-crate | dropped | n/a`) and its Rust destination; `cargo xtask coverage`
  prints % complete and fails CI if a file is unaccounted for; one full wave executed end to end to prove
  the throughput rate; the porting-agent contract (prompt + test + differential check + review gate)
  documented and versioned.
- **Tests:** `xtask/tests/codegen.rs` regenerates every generated artefact (MAVLink, DSDL, param metadata, `.resx`→`.ftl`, screen specs) and fails if the checked-in output differs or does not compile; `xtask/tests/ledger.rs` validates the ledger schema, asserts every one of the 3,678 C# files appears exactly once with a valid status, and that every `ported` entry names an existing Rust file with tests; `xtask/tests/coverage.rs` self-tests the `cargo xtask coverage` report against a fixture tree; a dry-run test of the porting-agent contract on a known file.

### D19. Verification suite
Proof that the Rust app behaves like the C# original before anyone flies behind it.
- **DoD:** differential harness running the C# reference headless (Mono/.NET on Linux) against the Rust
  implementation over a golden corpus of tlogs, dataflash logs and param dumps, diffing decoded output;
  fuzzing on all parsers; proptest round-trips; numeric-equivalence tests with stated tolerances;
  ArduPilot SITL integration tests in CI driving scripted missions; UI snapshot tests on headless GPU;
  criterion perf gates that **fail the build on regression**; a documented hardware-in-the-loop checklist
  signed off before each release.
- **Tests:** this deliverable *is* the test infrastructure, so it is proven by **mutation testing**: `tests/harness_selftest.rs` injects known regressions (off-by-one in a parser, a swapped lat/lon, a wrong unit conversion, a dropped retry, a 2 ms frame-budget regression) and asserts the differential harness, the fuzzers, the SITL suite and the perf gates each **fail**. A harness that cannot detect a planted bug is not a harness. Also covers: golden-corpus integrity checks, C#-reference-harness reproducibility, and CI flake tracking with a zero-tolerance quarantine policy.

### D20. Release, packaging and operations
Shipping the thing: signed installers per OS, auto-update, crash reporting, telemetry opt-in, docs and the
migration guide for existing Mission Planner users.
- **DoD:** one command produces signed artefacts for Windows (MSI/EXE), macOS (notarised `.dmg`) and Linux
  (AppImage + `.deb`); auto-update channel with rollback; symbolicated crash reports; first-run migration
  imports existing Mission Planner settings and caches; user-facing docs and a "what changed" guide published;
  **cold start < 500 ms**, installer < 150 MB.
- **Replaces:** `Updater/`, `ExtLibs/Installer`, `wix/`, `Msi/`, `MAC/`, `MissionPlanner.sh`, `build*.bat`.
- **Tests:** `tests/package_smoke.rs` per OS installs the built artefact in a clean container/VM, launches it headless, connects to SITL, and uninstalls, asserting no leftover files; `tests/update.rs` exercises update and rollback between two signed builds; `tests/crash_report.rs` forces a crash and asserts a symbolicated report; `tests/migration.rs` runs first-run migration against a real Mission Planner data directory fixture and asserts settings, map cache and mission files are imported intact; `benches/cold_start.rs` gates the <500 ms target.

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
- **Tests:** `crates/mp-plugin-host/tests/load.rs` builds the sample plugin in CI and loads it;
  `tests/abi.rs` asserts a deliberately mismatched ABI version is refused; `tests/panic.rs` asserts
  a plugin that panics in each callback does not terminate the host and is reported by name;
  `tests/safe_mode.rs` asserts nothing loads; `tests/source_plugin.rs` compiles and loads a loose
  `.rs` plugin, and is skipped-with-a-message rather than silently passing when no toolchain exists.
- **Replaces:** `Plugin/PluginLoader.cs`, `Plugin/Plugin.cs`, the full-trust half of `Plugins/`.

## Cross-cutting acceptance gates

Every deliverable above must also satisfy:

| Gate | Requirement |
|---|---|
| Platform parity | Works on Windows, Linux and macOS; no deliverable is "done" on one OS |
| Performance | Meets its stated numeric budget, measured by a checked-in benchmark, gated in CI |
| No-panic path | Telemetry ingest, state update and render paths contain no `unwrap`/`expect`/panic |
| Coded test suite | The `Tests:` artefacts listed for that deliverable exist, run in CI on every PR, and fail loudly — no deliverable reaches *Feature complete* without them |
| Differential proof | Behaviour compared against the C# original on real data where a reference exists |
| Ledger | Every C# source file it replaces is marked in the D18 ledger |
| Licence hygiene | GPLv3 compliance, upstream attribution recorded, `cargo-deny` clean |

## Numeric targets (the "extreme performance" contract)

| Metric | Target |
|---|---|
| Cold start to connected UI | < 500 ms |
| Idle CPU (connected, 10 Hz telemetry) | < 1 % of one core |
| Packet-to-pixel latency (p99) | < 16 ms |
| Stick input to packet on the wire (p99) | < 5 ms |
| MAVLink decode throughput | > 1 M msg/s/core, 0 allocations per packet |
| 1 GB dataflash log open | < 2 s to first plot |
| Log plot scrub, 10 M points | 120 fps |
| Map pan/zoom, 1 M-point track + 10 k markers | 120 fps |
| Resident memory, 1 vehicle + map + 1 GB log open | < 1 GB |
| Concurrent vehicles | ≥ 50 without frame drops |

---

*Companion document: `PLAN.md` (roadmap, crate graph, technology decisions, risk register). These 20
deliverables are the "what"; PLAN.md is the "in what order, and how we prove it".*
