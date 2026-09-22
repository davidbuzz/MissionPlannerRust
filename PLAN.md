# MissionPlannerRust — Engineering Plan

**Status:** v1, 2026-09-23. Supersedes `DELIVERABLES.md` (kept as the deliverable catalogue; this
document is the plan).
**Upstream pin:** ArduPilot Mission Planner `efb080190de0bf091f9aab982c8848a124de7588` (2026-09-17).
**Reference trees are READ-ONLY:** `referneces/missionplanner`, `referneces/zed`.

Everything in this document that is a number was measured on this box in this session, or is marked
`UNVERIFIED`. Where two prior analyses disagreed, the disagreement is resolved by an experiment and
the experiment is named. Where it cannot be resolved yet, there is a decision gate with a date.

---

## 1. What we are building

A ground control station for ArduPilot vehicles, in Rust, that accounts for 100% of Mission
Planner's behaviour, runs on Linux/Windows/macOS, and is fast enough that the performance claim is a
table of measured numbers rather than an adjective.

### 1.1 The four goals, restated so they are falsifiable

| # | Goal | Falsifiable form |
|---|---|---|
| G1 | File-complete port | `cargo xtask ledger check` exits 0 with every one of 3,678 `.cs` files in a terminal state (`done` / `dropped(reason)`), each `done` row carrying its evidence artifact |
| G2 | Extreme performance | Every number in §8 met on the reference hardware, gated in CI against absolute floors, published as a C#-vs-Rust comparison table |
| G3 | Multi-platform | One `cargo build --release` produces a working binary on x86_64 Linux, Windows and macOS; every release gate runs on all three |
| G4 | GPU-accelerated visuals | Map, HUD, charts and overlays composited on the GPU; frame budgets in §8 met at 2560×1440; draw-call counts asserted in the headless harness |

### 1.2 Non-goals — decided, not deferred

| Non-goal | Evidence | Consequence |
|---|---|---|
| Android and iOS | No gpui Cargo.toml mentions `android`; `gpui_apple` is `cfg(target_os = "macos")` | The shipping Play Store app (`com.michaeloborne.MissionPlanner`, built by `.github/workflows/android.yml`) is **lost**. 39,234 LOC of real mobile app. Needs owner sign-off — see §12 D1 |
| Browser GCS | Serial, raw TCP and UDP cannot exist in a browser | `gpui_web` exists; we keep the pure-compute crates wasm-clean in CI but promise nothing |
| Bug-for-bug WinForms pixel fidelity | 2,766 resx + 805 designer `Location` entries vs 284 `Dock`; MP ships `<dpiAware>false</dpiAware>` | Screens are re-laid out, not traced. Layout debt tracked per screen |
| ~~In-process arbitrary-code plugins~~ | **Overruled by the owner, 2026-09-23: "when I said 100% of MissionPlanner, I didn't mean 99%."** Moved to Phase 12, after everything else | See §10.4 |
| Reproducing MP's unsigned MD5 updater | `Utilities/Update.cs` trusts an MD5 list fetched over the network | Replaced by ed25519-signed artifacts |

### 1.3 The fidelity ↔ performance tension, resolved

These goals conflict at identifiable sites and the conflict must be settled by a rule, not by
case-by-case argument.

**Rule: transliterate the arithmetic, redesign the plumbing.**

Every measured speed-up available comes from replacing an *architecture*, not from translating code
faster:

| MP does | Cost | We do |
|---|---|---|
| `BaseStream.Read(buffer, count, 1)` — one byte per syscall, with `await Task.Delay(1)` spin-polls (`MAVLinkInterface.cs:4730,4750`) | ~3 heap allocs + 36 `DateTime` reads + ~20 locked dict lookups per packet | 64 KiB reads into `BytesMut`, `memchr` resync, zero-alloc `RawFrame` |
| 549 mutable fields under `lock(this)`, read unlocked by data binding (`CurrentState.cs`) | torn reads, 10 Hz `ResetBindings` reflection storm | `#[repr(C)] Copy` snapshot published via `ArcSwap`, coalesced at display rate |
| boxed `object[]` + whole-row `ToString` + `double.Parse` per graphed sample (`LogBrowse.cs:1537`) | a seek and ~2N allocs per f64 | columnar store + min/max LOD pyramid; query cost O(pixels) |
| per-marker GDI+ `DrawImage` on the UI thread (`GMapControl.cs:595`) | frame-bound at a few hundred markers | one instanced draw |

Conversely, where fidelity wins it wins **on purpose and it is recorded**:

- `PointLatLngAlt.GetDistance` is a hand-inlined spherical haversine (literal
  `0.017453292519943295`, R = 6371000, **not** WGS84). We port it literally. A "better" geodesy crate
  shifts every distance in the app by up to 0.5%.
- `CompassCalibrator.cs` is all-`f32` with an explicit operation order. We gate it **bit-exact** and
  forbid `mul_add` — i.e. we explicitly forbid optimising 904 LOC.
- `utmpos` encodes hemisphere as the *sign* of the UTM zone (oracle returns `zone=-56` for
  Brisbane). Preserved.

**Mechanism.** Every ledger row carries two columns:

- `fidelity_class` ∈ {`bit-exact`, `tolerance-bounded`, `behaviour-equivalent`, `redesigned`}
- `perf_class` ∈ {`hot`, `warm`, `cold`}

"Faithful to the C#" is **not** an accepted defence for anything marked `hot`.

**And G1 is redefined accordingly.** 422 non-vendored, non-designer files totalling 208,951 LOC
reference `System.Windows.Forms` (measured). There is no faithful Rust rendering of
`Control.Invalidate()` or `BindingSource.ResetBindings()`; a faithful port of the 10 Hz reflective
binding storm would be *slower* in Rust than in C#. So G1 means **behaviour coverage**: each ledger
row records "accounted for by `<rust module(s)>`", not "translated to". A ledger reading 3,678/3,678
proves the inventory is closed — it does not claim line-for-line translation, and never did.

---

## 2. Verdict on the Zed / gpui ecosystem

### 2.1 ADOPT — with three corrections to the premise and explicit abandon conditions

**New evidence generated in this session, which settles a point the prior analyses disputed:**

```
$ cargo check -p mp-gui     # gpui 0.2.2 from crates.io
    Finished `dev` profile in 2m 23s                          EXIT=0
$ cargo build -p mp-gui
    target/debug/mpr-gui   612,709,416 bytes                  EXIT=0
$ DISPLAY=:0 ./target/debug/mpr-gui                           # ran 25s, no error, window opened
```

gpui **builds, links and runs on this box today** — Ubuntu 24.04, X11, hybrid Intel UHD + NVIDIA
Quadro T2000. A prior probe reported a link failure (`-lxkbcommon-x11`); that probe was against the
git/wgpu variant with different features. The published crate works. `Cargo.lock` has 747 packages.

**Correction 1 — "the same ecosystem as Zed" is three renderers, not one.**

| Platform | Backend | Shader language | Crate |
|---|---|---|---|
| Linux, FreeBSD, web | wgpu 29.0.4 | WGSL | `gpui_wgpu` |
| Windows | Direct3D 11 | HLSL | `gpui_windows/src/directx_renderer.rs` (2,064 LOC) |
| macOS / iOS | Metal | MSL | `gpui_apple/src/metal_renderer.rs` (1,633 LOC) |

Verified: `grep -c wgpu crates/gpui_windows/Cargo.toml` → `0`. Only `gpui_web` and `gpui_linux`
depend on `gpui_wgpu`. Any plan that sizes a GPU patch by measuring `wgpu_renderer.rs` has sized
one third of the work.

**Correction 2 — crates.io gpui is the *old* renderer, and the new one is unpublishable.**
`gpui 0.2.2` resolves `blade-graphics 0.7.1` (7 `blade` entries in our lock, zero `wgpu`). The wgpu
renderer lives only in the zed monorepo, in crates that declare bare `{ path = ... }` deps and
inherit `publish = false`. Using it means a git pin to a 245-crate monorepo with no semver — and it
drags the toolchain from our 1.95.0 to zed's pinned 1.98.1.

**Correction 3 — there is no custom-GPU escape hatch.** `Primitive` is a closed 8-variant enum
(`scene.rs:222`); `paint_surface` is `#[cfg(target_os = "macos")]` CVPixelBuffer-only;
`elements/canvas.rs` (95 LOC) is a paint callback, not a GPU canvas; `AtlasKey` takes CPU bytes.
`wgpu_renderer.rs:1545` is literally `PrimitiveBatch::Surfaces(_surfaces) => {}` — an empty arm —
while the surfaces pipeline, bind-group layout and `vs_surface`/`fs_surface` WGSL all exist unhooked.

### 2.2 Why adopt anyway

Because ~85% of the GPU story needs **no fork at all**:

- **Map tiles** — `paint_image` keys the atlas on `RenderImageParams { image_id, frame_index }`, so a
  tile with a stable id derived from `(provider, z, x, y)` uploads **once** and stays GPU-resident.
- **HUD, tracks, geofences, chart series, markers** — quads, lyon-tessellated paths, atlased glyphs.

Only **live video** and the **3D terrain view** need something gpui does not have. Both are Phase 6
features. So the fork decision is not a gate on starting; it is a gate on two features.

### 2.3 Sourcing decision

**Pin `gpui = "0.2.2"` from crates.io now.** Rationale: it works (verified above); it keeps MSRV at
1.95.0; it is a published crate with a checksum; and the blade-vs-wgpu distinction is invisible to
everything we build through `mp-render` and `mp-ui`. Treat the git/wgpu pin as an **upgrade gated on
the Phase 0 spike**, not as a starting position.

### 2.4 Abandon conditions — pre-registered, written before the spike runs

We leave gpui if **any** of these fires:

| # | Condition | Measured how |
|---|---|---|
| A1 | The Windows D3D11 backend cannot hold p99 ≤ 8.33 ms at 2560×1440 with tiles + HUD + 6 overlays | Run the spike on a real Windows box |
| A2 | gpui refuses to **start** on a target we must support (RDP session, VM with basic display adapter, 2015-era integrated GPU) | `directx_devices.rs` hard-fails with `"Required feature StructuredBuffer is not supported"` and has **no WARP/GL/software fallback**. Test by RDP'ing into a Windows box and launching |
| A3 | The external-texture patch exceeds ~600 LOC on one backend, or cannot be done without touching all three | Prototype on `gpui_wgpu` first (the arm is already stubbed), then cost HLSL and MSL |
| A4 | A gpui rev bump costs more than ~1 engineer-week per quarter | Measured after two bumps |

**Fallbacks, in order:** `egui` + `wgpu` (`egui_wgpu::CallbackTrait` gives `prepare`/`paint` with a
raw `wgpu::RenderPass`, and `eframe` ships a `glow` OpenGL backend — an answer to A2 that gpui has
no equivalent for); then `iced 0.14` (`iced_wgpu::Primitive` gives `prepare`/`draw`/`render` with our
own `CommandEncoder` — technically the best custom-GPU story of the three, but it has **no
accessibility at all** and no virtualized table).

**The insurance policy is architectural, not contractual:** a CI check over `cargo metadata` fails
the build if any crate below layer L6 names `gpui`. That bounds a pivot to two crates instead of the
project, and it is why §10 front-loads UI-free work.

---

## 3. Technology decisions

| Area | Choice | Rejected | Conf. | What would change our mind |
|---|---|---|---|---|
| UI framework | `gpui 0.2.2` (crates.io) behind an `mp-ui` facade | `gpui` git pin (unpublishable deps, MSRV 1.98.1); `gpui-component 0.6.6` (depends on `gpui-pre 0.3.6`, a third-party snapshot of an *older* zed — 683-package lock with **no** `gpui`, no wgpu) | med | Any of A1–A4 in §2.4 |
| GPU | Whatever backend gpui gives per platform; our own `wgpu` only behind `mp-render` | Owning the swapchain from day one | med | The spike shows tiles/HUD cannot hit budget through gpui primitives |
| Widget kit | Write `mp-ui` (~40 components) | Vendor zed `crates/ui` (28,832 LOC, GPL-3.0-or-later — legally fine, but editor-shaped and `publish=false`) | high | If `mp-ui` exceeds ~25k LOC, revisit vendoring |
| Text input | Write our own single-line editor | `ui_input::InputField` — a 272-LOC shim over `crates/editor` (181,570 LOC) | high | IME cannot be made correct on all three OSes in < 2k LOC |
| MAVLink codec | Our own generator (`xtask/src/codegen/`, 759 LOC, already emits lenient newtype enums) | `mavlink-bindgen 0.18.0` as primary — strict enums **drop the whole message** on `fix_type=200`; no `units`; no field reflection | high | Upstream lands units + lenient enums + a field table |
| MAVLink dialect source | MP's own 20 XMLs, pinned | The crate's bundled upstream (common.xml 230 msgs vs MP's 204; MP-only `RELAY_STATUS`; `offspec.xml` msgid 26900 absent) | high | Nothing |
| DroneCAN | Own v0 DSDL parser + emitter, gated on all 147 `DATA_TYPE_SIGNATURE` values in `canard_dsdlc/messages.cs` | `dronecan 0.1.0` (590 LOC, no DSDL); `canadensis` (Cyphal v1, wire-incompatible) | med | Signature CRC resists → run the vendored Python generator once, commit JSON |
| CRC | `crc 3.4.0` `Table<16>` CRC_16_MCRF4XX | `mavlink-core`'s bytewise (92.4 ns vs 20.2 ns per 40 B frame, bit-identical `0x63dd`) | high | — |
| Async runtime | `tokio` for I/O; dedicated OS threads per link; gpui's executor for UI | One runtime for everything (`gpui_tokio` hard-codes 2 workers) | high | — |
| Snapshot publication | `arc-swap 1.9` (measured: load 15.4 ns, store 118 ns on 704 B) | `RwLock` (UI holds it across render → priority inversion); `triple_buffer` (SPSC only; ≥5 readers per vehicle) | high | Profiling shows the HUD path needs 2.25 ns |
| Serial | `serialport 4.10.1` + `tokio-serial 5.5.0` | MP's vendored Mono `System.IO.Ports` (already `<Compile Remove>`d) | high | MPL-2.0 is GPLv3-compatible; confirmed |
| Numerics | `nalgebra 0.35` + `rustfft 6.4.1` + `levenberg-marquardt 0.15` | `alglibnet` (251,616 LOC, **exactly 3 call sites** outside its own tree: `MagCalib.cs`, `fft3.cs`, `Utils.cs`) | high | — |
| Polygon ops | `geo 0.33` Buffer/BooleanOps + `i_overlay 9.0` | Two vendored Clipper copies (4,930 + 4,815 LOC) | med | Scaled-integer vs f64 shifts grid output → pin scale, golden-test |
| Projections | **Unresolved — see §12 D4.** Either `proj 0.31` (native libproj) or drop Shapefile + arbitrary-CRS GeoTIFF | "~200 LOC UTM kernel" — cannot serve `DotSpatial.Projections`' EPSG/proj4/ESRI-WKT reprojection used by `FlightPlanner.cs:3533` (.shp import) and `GeoTiff.cs:232` | low | Audit which CRSs real user GeoTIFFs actually use |
| Video | `gstreamer 0.25` | `LibVLC.NET` (`vlc-rs` dead since 2018); `DirectShowLib` (37,629 LOC, Windows-only) | med | LGPL/GPL shipping obligations of the ffmpeg alternative |
| Charting | Write `mp-chart` on gpui primitives + LOD pyramid | `ZedGraph` (52,265 LOC); `plotters` (static backend, no interaction model) | high | — |
| Scripting | `rhai 1.26` (operation limits + per-call timeouts — these scripts fly aircraft) | IronPython compat via `rustpython-vm` (incomplete stdlib) or `pyo3` (forfeits single-static-binary) | med | Owner requires verbatim user-script compatibility (§12 D6) |
| Plugins | `wasmtime 48` component host, capability-gated per MAVLink msgid | In-process dylib | high | — |
| i18n | `fluent 0.17` + `i18n-embed-fl 0.10` (compile-checked `fl!()`) | `.resx` (of 30,916 English entries only ~2,634 are language) | high | — |
| Expression eval | Own compiled evaluator over column handles | `evalexpr 13.1.0` — **AGPL-3.0-only**, must never enter | high | — |
| Licence | GPL-3.0-or-later | Anything else (`COPYING.txt` is verbatim GPLv3; the port is a derivative work) | high | Nothing. Note: consuming Apache-2.0 gpui permanently forecloses GPLv2 |

---

## 4. Inventory and disposition

Computed this session by classifying **every** `.cs` file
(`scratchpad/part.sh`); the five buckets sum **exactly** to the 1,208,836 total.

| Tier | Disposition | Files | LOC | % |
|---|---|---:|---:|---:|
| **T0** | Vendored third-party → **delete**, replace with crates | 1,838 | 731,732 | 60.5% |
| **T1** | Machine-generated → **regenerate** from IDL already in-tree | 407 | 133,227 | 11.0% |
| **T2** | WinForms designer output → **extract** to a screen spec | 201 | 59,237 | 4.9% |
| **T3** | Mission-Planner-original → **hand-port** | 1,034 | 245,406 | 20.3% |
| **T4** | Mobile (Xamarin app, non-generated) → **drop** pending owner ruling | 198 | 39,234 | 3.2% |
| | **TOTAL** | **3,678** | **1,208,836** | **100%** |

> **The denominator has a hole.** `.gitmodules` declares `ExtLibs/mono` →
> `https://github.com/meee1/mono.git` (shallow). The directory is **empty**, so nothing under it is
> in the 3,678 / 1,208,836 figures — yet `crowdin.bat` passes `-xr!mono` when zipping `*.resx`, so it
> contains files, and both CI workflows check out with `submodules: true`. **Phase 0 must fetch and
> measure it before a ledger row is written**, or every "3,678 of 3,678" claim certifies the wrong
> denominator.

### 4.1 T0 — the deletions that matter (top 20 by LOC)

| Component | LOC | Replacement | Verified dead? |
|---|---:|---|---|
| `ExtLibs/alglibnet` | 251,616 | `nalgebra` + `rustfft` + `levenberg-marquardt` | 3 external call sites |
| `ExtLibs/netDxf` | 65,129 | `dxf 0.6.1` | serves a 53-LOC reader + 1 debug write |
| `ExtLibs/ZedGraph` | 52,265 | `mp-chart` (greenfield) | `FilteredPointList` exists, **zero** uses |
| `ExtLibs/ObjectListView` ×4 | 42,184 | gpui `uniform_list` | not in `.sln`; sole consumer `Log/LogIndex.cs` |
| `ExtLibs/DirectShowLib` | 37,629 | `gstreamer` / `nokhwa` | ~95% DVB/DVD code never touched |
| `ExtLibs/GMap.NET.*` ×3 | 33,899 | `mp-map` (greenfield) | — |
| `ExtLibs/ICSharpCode.SharpZipLib` | 27,249 | `zip` + `flate2` | 6 call sites |
| `ExtLibs/BaseClasses` | 24,298 | — | DirectShow COM base classes, **zero** vehicle-domain consumers |
| `ExtLibs/MetaDataExtractor` | 17,800 | `kamadak-exif` + `little_exif` | check maker-note tags first |
| `ExtLibs/MissionPlanner.Drawing`+ | ~21,202 | gpui | pure `LIB`-build scaffolding |
| `ExtLibs/SvgNet` | 13,317 | gpui `svg()` | `GL2.cs` (1,445) dead behind `#if GL`, undefined |
| `ExtLibs/SharpKml` + `KMLib` | 13,924 | `kml 0.14` | — |
| `ExtLibs/CsAssortedWidgets` | 10,742 | — | **not in `.sln`, zero consumers** |
| `ExtLibs/BSE.Windows.Forms` | 10,323 | `mp-ui` | 2 consumers |
| `ExtLibs/ProjNet` | 10,212 | see §12 D4 | `Transform()` has **zero** callers |
| `ExtLibs/GeoUtility` | 8,006 | `mgrs` (or hand-port `MGRS.cs`, 624 LOC) | `MapService.cs` (2,263) is a dead 2nd tile downloader |
| `ExtLibs/SharpAdbClient` | 7,512 | — | only external ref is a `LIB` shim stub |
| `ExtLibs/NMEA2000` | 6,756 | — | in `.sln`, **not** in `MissionPlanner.csproj` |
| `ExtLibs/zlib.net` | 5,637 | `flate2` | csproj-referenced, **zero** call sites |
| `ExtLibs/7zip` | 4,545 | `sevenz-rust2` | csproj-referenced, **zero** call sites |

`ExtLibs/NetDFULib` (1,824) additionally P/Invokes ST's non-redistributable `STDFU.dll` under a
**CPOL** licence that is **not GPL-compatible** — a compliance removal, not just a cleanup. Same for
`ExtLibs/Utilities/Matrix.cs` and `Kalman3D.cs` (CPOL, Philip R. Braica).

### 4.2 T1 — generated, and what regenerates it

| Source | Generated C# | Generator | Gate |
|---|---:|---|---|
| 20 MAVLink XMLs (11,934 lines, ArduPilot fork + `offspec.xml`) | `Mavlink.cs` 37,563 | `xtask codegen mavlink` (in-tree, 759 LOC) | **349** rows of `testdata/mavlink/csharp_message_infos.csv` must match on `(id,name,crc_extra,min_len,len)`. Note: 350 `message_info(` entries exist, one has a null name and is skipped by the dumper — a literal "350" CI gate fails on day one |
| 147 `.uavcan` DSDL | `out/src` 23,374 (381 files) | `xtask dsdlgen` (**to write**) | all 147 `DATA_TYPE_SIGNATURE` + `default_dtid` must equal `canard_dsdlc/messages.cs` |
| Android resources | 65,305 | — | dropped with T4 |
| `Resources.Designer.cs` / `Settings.Designer.cs` | 6,415 | `xtask assetgen` | — |

### 4.3 T2 — screens, and the non-obvious fact that makes them tractable

**For the 80 `Localizable` forms the layout is not in the C# at all.** `FlightData.Designer.cs` makes
221 `resources.ApplyResources()` calls; `FlightData.resx` carries 834 `>>` entries giving
`>>ctrl.Name`, `>>ctrl.Type` (assembly-qualified), `>>ctrl.Parent`, `>>ctrl.ZOrder`, plus
`Location`/`Size`/`Anchor`/`Dock`/`TabIndex`. Repo-wide: 3,493 `>>Name`, 3,127 `>>Parent`, 3,127
`>>ZOrder`, 2,911 `ApplyResources` calls.

So the extractor is **two cooperating readers**: `quick-xml` over the `.resx` for the control tree
and localizable geometry; `tree-sitter-c-sharp` over `Designer.cs` for the 104 non-localizable forms,
the never-localized properties (`BackColor` 849, `Enabled` 589, `Image` 75) and the 1,539 event
wirings. Output is **RON**, not gpui — so "did we read WinForms correctly" and "is the gpui output
good" stay separately debuggable.

Control vocabulary is finite: **3,931 instantiations of only 103 distinct types**. Top: Label 1,195,
MyButton 411, MavlinkNumericUpDown 230, CheckBox 189, TextBox 186, NumericUpDown 167,
ToolStripMenuItem 166, GroupBox 163, ComboBox 162. A ~40-component kit covers >95%.

### 4.4 T3 — the real work, and its shape

**1,034 files / 245,406 LOC.** Size distribution (this is the number that drives everything in §6
and §10):

| Bucket | Files | LOC | Share of T3 |
|---|---:|---:|---:|
| < 200 LOC | ~760 | ~42k | 17% |
| 200–600 | ~210 | ~75k | 31% |
| 600–1,500 | ~64 | ~55k | 22% |
| **> 1,500** | **~22** | **~73k** | **30%** |

The 5 largest: `FlightPlanner.cs` 8,576 · `MAVLinkInterface.cs` 6,906 · `FlightData.cs` 6,756 ·
`CurrentState.cs` 5,016 · `MainV2.cs` 4,826.

> **Refuted claim, fixed here.** Prior drafts said "split the 8 giant files by `#region`".
> Measured: `grep -c '#region'` returns **0** for all five files above. They are flat partial classes
> with 162–216 methods and no section markers. The mechanical pre-processing step the dispatcher
> depends on **does not exist**. See §6.5 for the replacement (budgeted human decomposition).

### 4.5 Honest bottom-up Rust estimate

| Input | LOC |
|---|---:|
| T3 raw | 245,406 |
| less droppable inside T3 (installer, updater, `wix`, `resedit`, `temp.cs` 2,657, `solo` 406, `Arduino` 954, `Mock`, `TestPlugin`, `tlogThumbnailHandler` 593, `DriverCleanup` 359, mobile shims) | −~25,000 |
| **real port surface** | **~220,000** |
| of which UI-coupled rewrite | ~125,000 |
| of which non-UI logic | ~95,000 |

| Output | Rust src LOC |
|---|---:|
| non-UI logic @ 0.75× | ~71,000 |
| UI-coupled @ 0.45× | ~56,000 |
| greenfield with no C# analogue (`mp-ui`, `mp-chart`, `mp-map`, `mp-render`, 3D, plugin host, `xtask`) | 70,000–85,000 |
| **subtotal src** | **197,000–212,000** |
| tests/benches/fuzz at the ratio measured in our own existing crates (`mp-mavlink` 2,150 total with ~45% test; `mp-transport` 1,449 with ~32%; the contract in §6.4 demands *more* than exists) | **+35–45%** |
| **TOTAL hand-written Rust** | **~270,000–300,000** |
| plus generated (MAVLink ~41k already + DSDL ~20k + screens/params) | **90,000–130,000** |

**Call it 290k hand-written ± 40k.** Prior drafts' 243k–259k figures did not include tests.

---

## 5. Target workspace, crate graph and conventions

### 5.1 Layers (the L6 rule is load-bearing)

```
L0  mp-units  mp-math  mp-time  mp-bus  mp-settings
L1  mp-mavlink  mp-mavlink-dialects  mp-dronecan  mp-gnss  mp-adsb  mp-cot
L2  mp-transport  mp-transport-ble  mp-platform{,-linux,-windows,-macos}
L3  mp-link  mp-vehicle  mp-params  mp-mission  mp-ftp  mp-firmware
    mp-calibration  mp-joystick  mp-swarm  mp-antenna  mp-hil
L4  mp-log-dataflash  mp-log-tlog  mp-log-ulog  mp-log-analysis  mp-logstore
L5  mp-geo  mp-terrain  mp-survey  mp-georef  mp-kml  mp-nofly
--------------------------- gpui boundary -----------------------------
L6  mp-render  mp-map  mp-hud  mp-chart  mp-video  mp-terrain3d  mp-icons
L7  mp-ui  mp-theme  mp-l10n
L8  mp-screen-*  (one crate per GCSView; never depend on each other)
L9  mp-app                        L10  mission-planner (bin, <200 LOC)
L11 mp-plugin-api  mp-plugin-host  mp-script       L12 xtask  mp-codegen
```

**Mechanically enforced by CI over `cargo metadata`:**

1. No crate below **L6** may depend on `gpui`. *(This is the pivot insurance. ~110–130k Rust LOC
   stays framework-agnostic.)*
2. No crate below **L3** may depend on `mp-link`.
3. No **L8** screen crate may depend on another L8 screen crate — they talk through `mp-app`.
4. Only **`mp-render`** may name `gpui` or `wgpu` *internals* (if the fork lands, it lives here and
   nowhere else).

> **Ordering correction.** A prior draft placed `mp-app` *after* the screens. 187 files reference
> `MainV2` (4,370 references, 3,837 of them `MainV2.comPort`), clustered in exactly the screen layer:
> 61 in `GCSViews`, 40 in `Controls`. The app-state root, screen router, `Link` global, settings store
> and the modal API must be **frozen before the first screen agent is dispatched**. The modal API
> alone has 954 `CustomMessageBox.Show` + 170 `InputBox.Show` call sites.

### 5.2 Current state — re-baselined honestly

Measured this session: **5 commits, 9 crates, 48,775 Rust LOC** of which **40,547 is the generated
dialect** → **~8,228 hand-written**, plus 1,100 LOC of `xtask`.

| Crate | LOC | Note |
|---|---:|---|
| `mp-mavlink-dialects` | 40,952 | 40,547 generated |
| `mp-mavlink` | 2,150 | ~45% tests |
| `mp-transport` | 1,449 | serial/tcp/udp/replay |
| `mp-vehicle` | 1,044 | snapshot bus |
| `mp-link` | 883 | uncommitted changes |
| `xtask` | 1,100 | mavlink codegen only |
| `mp-log`, `mp-gui`, `mp-cli`, `mp-units` | 1,197 | `mp-gui`/`mp-log` **untracked** |

What exists is *vertical-slice* work (commit messages say so). What does **not** exist: the ledger,
the dependency graph, the dispatcher, the DSDL/resx/screenspec/paramgen generators, the corpus, the
GPU spike. And `.github/workflows/ci.yml`'s "differential" job neither installs mono nor references a
Mission Planner distribution — it runs `cargo test --test differential_tlog` against checked-in CSVs.
**Phase 0 is ~12% complete, not ~40%.** Every existing file is re-entered into the ledger *under
contract*, not marked done.

### 5.3 Conventions

**Errors.** `thiserror` per crate, one enum, no cross-crate re-export chains. `anyhow` only in
`mission-planner/src/main.rs`, `xtask`, tests and benches. `mp-link` and `mp-transport` additionally
expose a `#[non_exhaustive] LinkError` separating `Transient` (retry) from `Fatal` (drop link) —
because `MAVLinkInterface.cs` throwing on timeout is the behaviour we most want *not* to reproduce.

**Units.** `mp-units` newtypes. **Altitude carries its frame in the type** (`AltAmsl`, `AltAgl`,
`AltRel`, `AltTerrain`) — `PointLatLngAlt`'s single untyped `Alt` field is a known MP bug class.
Bare `f64` latitude banned by lint. `Radians` internally, `Degrees` only at protocol and UI edges.
`Color`/`Tag` do **not** live on the geodetic type (carrying `System.Drawing.Color` on the universal
coordinate is why `ExtLibs/Utilities` cannot build without `System.Drawing`).

**No-panic telemetry path** — scoped per-crate, not workspace-wide (the current blanket
`unwrap_used = "deny"` will be neutralised by `#[allow]` spam in UI crates):

```rust
#![deny(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing,
        clippy::panic, clippy::arithmetic_side_effects)]
```
applied to `mp-mavlink`, `mp-mavlink-dialects`, `mp-transport`, `mp-link`, `mp-vehicle`, `mp-bus`,
`mp-log-dataflash`, plus a test asserting no `core::panicking` symbol is reachable. A GCS that
panics mid-flight is worse than one showing a stale value.

**`unsafe`:** `forbid` everywhere except `mp-render`, `mp-platform-*`, `mp-video`, each with a
module-level justification and a `SAFETY:` comment per site.

**Features:** `default = []` on every library crate; only the bin turns features on; no
`default-features = true` on gpui internally.

**MSRV:** 1.95.0 while on crates.io gpui. Moves to **1.98.1 only if** the git/wgpu pin is adopted
(zed HEAD pins 1.98.1; `rustup` already has 1.98 installed here).

### 5.4 Build-time strategy — day-1, before any agent runs

Measured on this box: `mold`, `ld.lld`, `lld` are **all absent from PATH**. `target/` is already
2.5 GB at 9 crates; `cargo check -p mp-gui` cold was 2m23s over 747 packages.

| Fix | Why |
|---|---|
| `apt install mold` + `-C link-arg=-fuse-ld=mold` | highest-leverage single change |
| `lto = "fat"` → `"thin"` | zed uses thin + `codegen-units=1` over the same closure |
| replace `[profile.dev.package."*"] opt-level = 2` with zed's ~15 targeted entries (`quote`, `syn`, `proc-macro2`, `taffy`, `resvg`, `wasmtime`, `tree-sitter`) | optimising 747 deps is the dominant cold-build cost |
| **add the missing `[profile.dev.build-override]`** | zed's own comment: *"without this cargo will compile ~400 crates twice"* |
| add `[profile.dev]` `split-debuginfo="unpacked"`, `incremental=true`, `codegen-units=16` | — |
| `cargo-hakari 0.9.39` workspace-hack | feature unification churn dominates incremental rebuilds at 60 crates |
| shared `CARGO_TARGET_DIR` or `sccache` across agent worktrees | 8–15 GB per worktree × 12 agents vs 401 GB free |

Rejected: `rustc_codegen_cranelift` — no component installed, nightly-only, conflicts with a pinned
stable toolchain.

---

## 6. The translation factory

### 6.1 Codegen pipelines (all output **checked in**; never `build.rs`)

| `xtask` cmd | Input | Output | Gate |
|---|---|---|---|
| `mavgen` | `codegen/mavlink/*.xml` | dialect + `FieldInfo` sidecar | 349-row CSV; `git diff --exit-code` |
| `dsdlgen` | 147 `.uavcan` | ~20k Rust | 147 signatures vs `messages.cs` |
| `paramgen` | `ParameterMetaDataBackup.xml` (4,148,833 B; 11,474 DisplayName, 5,885 Range, 3,973 Units, 3,056 Values) + `ParameterFactMetaData.xml` | `ParamMeta` + derived `ParamUiDescriptor` | measure static-array vs `postcard` blob on **cold build time**, pick by the number |
| `resx2ftl` | 1,263 `.resx` | `assets/i18n/**/*.ftl` | committed immutable `keymap.toml`; ~2,634 language keys out of 30,916 entries |
| `screenspec` | `.resx` + `Designer.cs` | 201 RON specs | round-trips every `>>Parent`/`>>ZOrder` |
| `themegen`/`assetgen`/`datagen` | `*.mpsystheme`, `Resources/` (216 files), `mavcmd.xml`, `dataflashlog.xml`, `APMotorLayout.json`, `SerialOptionRules.json` | statics + assets | `mavcmd`/`dataflashlog` labels route through the **FTL key space** — they are translatable |
| `extract-resx-images` | 55 real binary entries (not the 480 boilerplate `Bitmap1`/`Icon1` headers) | PNG/ICO | one-shot; mono never a build dep |

**Critical:** `codegen/` holds **copies** of the inputs, not symlinks — `referneces/` is gitignored,
so a symlink makes the repo unbuildable for anyone else. `codegen/PROVENANCE.md` pins the upstream
SHA.

### 6.2 The ledger — the definition of "done"

`ledger/ledger.csv`, one row per `.cs` file, **3,678 rows** (plus whatever `ExtLibs/mono` adds):

```
path, loc, namespace, csproj, tier, disposition, target_crate, unit_id, size,
fidelity_class, perf_class, verification_class, deps, state, owner, attempts,
evidence, sha256, omissions, notes
```

- `state` ∈ `blocked → ready → claimed → ported → tested → verified → reviewed → done`, plus
  `deferred` / `dropped`.
- A unit becomes `ready` only when every dep is `done`. **That is the scheduler.**
- `sha256` of the C# source is the staleness detector: upstream moves → re-hash → changed rows flip
  back to `ready` with a rebase flag.
- `cargo xtask ledger check` fails CI on: any unclassified file; any `done` row missing evidence;
  any `dropped` row missing an owner-ratified reason; any `done` row whose Rust file lacks its
  provenance header.
- Progress is reported in **retired-C#-LOC**, never files-done:
  `T0 731,732 retired / T1 133,227 generated / T2 59,237 extracted / T3 41,220 done of 245,406`.

### 6.3 `verification_class` — because the oracle does not reach everywhere

| Class | LOC (approx) | Contract |
|---|---:|---|
| `oracle` | ~162k of T0+T3 under `ExtLibs/` | green `xtask diff` at the declared tolerance class |
| `golden` | ~10k | one-shot capture from the C#, committed |
| `spec+review` | **~209k WinForms-coupled** | written behaviour spec + structural snapshot + 2-person review. **No claim of differential equivalence** |

> **Refuted claim, fixed here.** "The oracle *is* the specification" is true for ExtLibs and false
> overall. 422 non-vendored non-designer files totalling **208,951 LOC** reference
> `System.Windows.Forms` (measured) and cannot be driven headless. Progress is reported against the
> three classes **separately**.

### 6.4 The per-file agent contract — five artifacts or the unit does not advance

1. **The port**, headed
   `//! Ported from <path> @ efb0801… (GPL-3.0-or-later)` plus a §5(a) change notice, with
   `// C#: <path>:<line>` on non-obvious transliterations.
   *(GPLv3 §5(a) obligation is real and cannot be inherited: grepping every non-vendored logic
   directory for "GNU General Public"/"GPL" returns **exactly one file** — `AP_GeodesicGrid.cs`. The
   port must originate attribution.)*
2. **Unit tests** under `crates/<c>/tests/`, `cargo-llvm-cov` floor 80%.
3. **Verification** appropriate to `verification_class` (above).
4. **A fuzz target** if the unit parses untrusted bytes. Non-negotiable — `BinaryLog.cs:38-51`
   reinterprets `byte[]` as `short[]` through a `[StructLayout(LayoutKind.Explicit)]` union.
5. **A criterion bench** if `perf_class = hot`.

Plus a **mandatory non-empty `omissions` list for any unit > 200 LOC.** An agent reporting
"not ported: nothing" on a 900-LOC file is almost always wrong, and silent omission is the dominant
failure mode of agent porting. Machine-enforced: a ledger patch with an empty `omissions` field on a
large unit is rejected.

Plus a **behaviour-difference entry**. Empty is allowed; absent is not.

### 6.5 Decomposition — the step that does not exist yet

Because there are **zero `#region` markers** in the giant files, `xtask next` cannot emit them.

| Bucket | Files | LOC | Treatment |
|---|---:|---:|---|
| > 1,500 LOC | ~22 | ~73k | **Owner-led design session**, 0.5–1 day each, producing a written module plan (target crate, sub-unit list, shared-type contract) committed to the ledger *before* dispatch |
| 600–1,500 | ~64 | ~55k | 1-hour lightweight version of the same |
| < 600 | ~948 | ~117k | Dispatchable directly |

**Budget: 3–5 weeks of owner time.** This was the single largest unbudgeted item in every prior
draft.

### 6.6 Dispatch and gates

`cargo xtask next --agents N` is a **pure function of the ledger**: N ready units in topological
order with a stable path-sort tiebreak, emitting the *full* payload (filled prompt, dep table
`csharp_symbol → rust_path`, stub list, oracle command, golden dir, tolerance class, banned actions).
Two runs over the same ledger produce byte-identical output. No agent picks its own work.

One git worktree per agent, one branch per unit (`port/<unit>`), one PR per unit. Never a shared
worktree. Per-crate in-flight cap: **1 L/XL + 3 S/M** — `Cargo.toml` and `lib.rs` are where
concurrent agents actually collide.

**Stub-crate strategy** so main stays green across 1,000+ incremental units: every target crate is
created empty with each not-yet-ported public item present as a signature with `todo!()`;
`clippy::todo` stays at **warn**; `xtask todo-census` fails the build **only** when a `todo!()`
survives inside a unit whose state is `done`. Anything reachable from `main` is behind
`#[cfg(feature = "unported")]`.

| Gate | Who | What |
|---|---|---|
| **G1** | machine | `fmt --check`, `clippy -D warnings`, `cargo nextest` (per-test process isolation), `xtask diff`, llvm-cov floor, `cargo-mutants ≤10%` surviving on bit-exact/residual crates, `cargo deny`, `hakari verify` |
| **G2** | a **different model instance**, given *only* the C# source and the Rust diff (no access to the porter's reasoning, or it inherits the porter's blind spots) | fixed rubric: semantic equivalence; **error-path parity** (did the port swallow a `catch` the original used for control flow? MP does this constantly); integer width/overflow (C# `int` wraps only in `unchecked`, Rust debug-panics); float formatting and `CultureInfo`; **"what is missing"** against a checklist of the C# methods |
| **G3** | human | mandatory for every L/XL unit, everything flight-critical (arming, mode change, mission upload, geofence, failsafe, RTL, firmware flashing), and the **first unit of every new crate** (it sets the idioms the next 50 copy). 1-in-10 sampled for S/M |

### 6.7 Review bandwidth is the binding constraint — and it must be staffed

~22 XL + ~64 L units + ~40 flight-critical + ~60 first-of-crate + ~105 sampled ≈ **~290 units
requiring genuine human review.** At 1.5–2.5 h each including one rework cycle: **435–725 hours**.
One reviewer at 20 productive review-hours/week = **22–36 weeks of pure, single-threaded review** —
before owner decisions, HIL rig time, or the 22 decomposition sessions.

**Agent count is irrelevant above ~8.** Every phase in §10 therefore carries a
`G3 review-hours` figure, and the calendar is derived from review throughput, not from agent-waves.

---

## 7. Verification

### 7.1 The oracle — proven, and with a named gap

**Proven working:** `msbuild -t:Restore,Build -p:Configuration=Release
ExtLibs/Utilities/MissionPlanner.Utilities.csproj` under Mono 6.12.0.200 exits 0 and produces 18
DLLs (alglibnet, ProjNET, GeoUtility, GMap.NET.Core, SharpKml, netDxf, MetaDataExtractor,
MissionPlanner.Comms, MAVLink, …). A probe returned `utm zone=-56`, round-trip error 3e-15 deg, and
`Grid.CreateGrid` producing waypoints headless. `mcs -target:library ExtLibs/Mavlink/*.cs` builds in
0.63 s.

**Named gap, settled by experiment:** `MissionPlannerLib.csproj` **does not build on Linux** —
399 errors; five `ProjectReference`s require `Microsoft.NET.Sdk.WindowsDesktop` (NETSDK1107), which
mono's msbuild does not have. The fallback also fails: `MagCalib.cs` itself does
`using System.Windows.Forms;` and `using MissionPlanner.Controls;`. Of the root-level files, only
`L10N.cs` and `Script.cs` are WinForms-clean.

**Fix — budgeted, not deferred.** Both of:
- **(a)** a ~300-LOC stub `System.Windows.Forms` + `MissionPlanner.Controls` facade assembly,
  compiled first and referenced by harness csprojs. Cheapest experiment to settle it: write the stub
  with the ~12 types `MagCalib.cs` touches and re-run `mcs MagCalib.cs` against the already-built
  `MissionPlanner.Utilities.dll`. **One afternoon.**
- **(c)** a **Windows CI runner with real .NET Framework 4.7.2** for anything culture-, float-format-
  or `DateTime`-sensitive. Mono 6.12 is EOL-adjacent and diverges from .NET 4.7.2 in exactly those
  areas; an oracle that is itself wrong produces a port that faithfully reproduces mono's bug.

Oracle verbs: `tlog-dump`, `dflog-dump`, `param-dump`, `mission-dump`, `fence-dump`, `rally-dump`,
`grid`, `corridor`, `rotary`, `geodesy`, `magcal`, `compasscal`, `parampck`, `mavftp-crc32`,
`dsdl-signature`. Persistent long-lived NDJSON mode for differential fuzzing (a per-case mono spawn
is ~100 ms and makes the target useless). Built **from pinned source**, cached as a CI artifact keyed
on the MP SHA — not from a downloaded Windows binary distribution as today's `regen.sh` does.

### 7.2 Four numeric tolerance classes, declared per ledger row

| Class | Rule | Applies to |
|---|---|---|
| **A** bit-exact | a single differing byte fails | MAVLink frames, CRC/CRC_EXTRA, signing hashes, tlog framing, dataflash record decode, param wire coercion, mission int32 1e7 lat/lng, **every file format we write** |
| **B** ≤4 ULP or 1e-12 relative, ≤1e-6 m positional | validated over a 1e6-pair global sweep | geodesy, UTM (incl. the negative-zone convention) |
| **C** structural + invariants | identical point *count*, then ordering, then ≤1e-7 deg, **plus** invariants that hold even when C# and Rust tie-break differently (every polygon interior point within `spacing/2` of a lane; lane separation within 1%; no lane exits by more than `overshoot`) | survey grid |
| **D** residual | `residual_rust ≤ residual_csharp × 1.001`, offsets within 1% of fitted radius, identical convergence classification | `MagCalib` (alglib `minlmcreatev`, numerical Jacobian, diffstep 0.1 — cannot be bit-matched) |

**Carve-out inside D:** `CompassCalibrator.cs` is all-`f32` with explicit operation order, so it is
gated **bit-exact** — and `mul_add` is forbidden (Rust does not auto-contract to FMA; introducing it
silently breaks equality).

### 7.3 Corpus, fuzzing, SITL

**Corpus** (git-lfs, with a <50 MB in-repo smoke subset so a new contributor can test offline):
200–500 real `.tlog`; 100+ `.bin` across Copter/Plane/Rover/Sub spanning several releases (so FMT
changes are covered); 50+ `.param`; 50+ `.waypoints`/`.fen`/`.ral`; plus **adversarial** — truncated
final frame, torn packets, interleaved v1/v2, signed packets, multi-sysid swarm, >4 GB, and one
containing msgid 26900. *Decide the anonymisation policy: these contain real GPS tracks of real
people.*

**Fuzz** (`cargo-fuzz 0.13.2`, corpora seeded from every frame sliced out of every tlog):
`fuzz_frame_decode` (any `Ok` must re-encode to identical bytes), `fuzz_dflog`, `fuzz_param_file`,
`fuzz_mission_file`, `fuzz_kml`, `fuzz_dxf`, plus a **nightly** differential target through the
persistent oracle. Nightly rotation across libFuzzer / honggfuzz / AFL — they find different bugs.

**Property tests** (`proptest 1.11`): round-trip over all 349 messages; MAVLink2 truncation; signing
tamper-detection; and most valuably **protocol state machines** modelling MP's hand-rolled retry
loops against adversarial links (drop, duplicate, reorder, delay past timeout, out-of-order
`PARAM_VALUE` index, every `MAV_MISSION_RESULT`), asserting termination, no deadlock, and retry
counts equal to the C# constants (`setParamAsync` 3, `GetParam` 3, `setWPCurrent` 5, `doCommand` 3).

**SITL:** fetch `SITL_x86_64_linux_gnu` from `firmware.ardupilot.org/manifest.json.gz` exactly as
`GCSViews/SITL.cs:316` does (so no ArduPilot toolchain in CI), cache by URL+etag, spawn
`-M<model> -O<home> -s10 --serial0 tcp:0 --wipe --defaults`, connect TCP 5760, RC on UDP 5501 — but
replace MP's `await Task.Delay(2000)` sleep-and-hope with a bounded poll for the first HEARTBEAT, or
the suite is flaky on loaded runners. Scenarios: connect, full param download via **both** paths,
set/verify, mission upload+download bit-identical, fence+rally, arm/disarm, mode change, auto mission
to completion, RTL, geofence breach, link loss + reconnect, 5-vehicle swarm on 5760/5770/5780/5790/5800.

### 7.4 UI verification — two tiers, because pixels are not portable

- **Primary: structural snapshots.** Serialise the gpui `Scene`/element tree to text and diff with
  `insta 1.48`. Deterministic, GPU-independent, runs on all three OS jobs, catches layout
  regressions.
- **Secondary: pixel snapshots**, GPU-pinned, run **only** on the lavapipe CI job
  (`/usr/share/vulkan/icd.d/lvp_icd.json` is already installed here), compared **perceptually**
  (`image-compare` SSIM or `dssim-core`), **never** exact-equality. Zed's own `MATCH_THRESHOLD = 0.99`
  exact-pixel count is tuned for one Metal machine and will not survive a driver change. A flaky
  visual gate that engineers learn to ignore is worse than no gate.
- Assert `GpuSpecs.is_software_emulated == true` on the lavapipe job and `false` on GPU jobs, so a
  misconfigured runner fails loudly instead of baselining against the wrong rasteriser.
- A scripted UI walk as `#[gpui::test(iterations = 100, seeds = [...])]` using the deterministic
  `TestDispatcher` — replaying interleavings a human could never reproduce.
- **UNVERIFIED / to build:** `WgpuHeadlessRenderer` (~250 LOC mirroring `MetalHeadlessRenderer`'s
  3-method trait, plus a `WgpuRenderer::new_offscreen` that skips `create_surface_unsafe`). Today
  `current_headless_renderer()` returns `Some` only on macOS. This one patch unlocks visual
  regression *and* GPU benchmarking on Linux CI.

### 7.5 Perf gates and HIL

**Perf:** `criterion 0.8` + `critcmp 0.1.8` at a 5% regression gate vs merge-base, **plus absolute
floors** (so slow drift over 200 PRs cannot sneak through), **plus `iai-callgrind 0.16`**
instruction-count benchmarks for parsers where wall-clock noise on shared runners defeats thresholds.

**HIL — procurement starts in Phase 0, not at release.** ≥3 autopilots (Cube Orange+, Pixhawk 6X,
one cheap F4) on a USB hub, a SiK pair, one BLE link. Automated: detect board by VID/PID; flash via
**all three** paths (px4uploader, DFU, DroneCAN); download full params; upload a 200-waypoint mission
and read it back **bit-identical**; run compass and accel calibration with props off; arm on the
bench; verify failsafe; pull a dataflash log over MAVFTP and diff it against the same log pulled by
C# Mission Planner. Flashing can brick hardware and has **no SITL equivalent** — this is the one
thing more simulation cannot replace.

**Release gate:** a manual flight test by a qualified pilot on a cheap airframe, on each OS, before
any build is tagged flight-ready.

---

## 8. Performance architecture and targets

### 8.1 Architecture

**Ingest, three tiers.** One dedicated OS thread per link (`spawn_dedicated`), blocking 64 KiB reads
into a `BytesMut`, `memchr` resync on 0xFD/0xFE, CRC + signature validated in place, emitting a
zero-alloc `RawFrame { buf: Bytes, rx: Instant, rx_wall: SystemTime, link: LinkId }`. Typed decode
**only** for msgids a subscriber wants, via a msgid-indexed table. `VehicleId` resolved **once** per
frame via `slotmap` + `dashmap`.

Measured budget: CRC 20 ns + typed deser 14 ns + ~15 ns bookkeeping ≈ **50 ns/packet**. At 7,000
pkt/s (20 vehicles @ 10 Hz) that is 0.035% of one core; at 127,000 pkt/s (254 vehicles @ 500 msg/s)
it is ~0.64%.

**Publication.** `#[derive(Clone, Copy)] #[repr(C)] TelemetrySnapshot` (~700 B, POD, no String/Vec)
published via `ArcSwap` (measured: load 15.4 ns, store 118 ns), **coalesced on a monotonic deadline
at `min(display_hz, 120 Hz)`** — never per packet. Every widget in a frame sees a coherent snapshot,
which MP structurally cannot do.

**Logs.** `memmap2` + rayon chunked parallel scan resyncing on `0xA3 0x95`, prefix-summed per-chunk
counts, second parallel pass writing into **disjoint exactly-sized column slices** — no locks, no
reallocation. Columns at native width per `(type, instance, field)`. Then the **min/max mip
pyramid**: level *k* = min/max pairs over 2^k buckets, ~2n f32 total, one O(n) rayon pass. Query
picks the level where one element ≈ one pixel column → ≤2×viewport-width segments **regardless of
series length**.

> This is **architecture, not optimisation**. gpui CPU-tessellates Paths with lyon every frame into a
> full-viewport MSAA intermediate; `PathRasterizationVertex` is 112 bytes. A 100k-segment polyline is
> ~67 MB/frame. Without the pyramid, a 10M-point series cannot be drawn at all.
>
> Two traps designed in: dataflash timestamps are **not** evenly spaced, so buckets carry
> `(t_min, t_max)`; and the pyramid is per `(type, instance, field)` or every GPS[0]/GPS[1] and
> IMU[0..2] graph is wrong. Use **min/max, not LTTB** — min/max never hides a one-sample spike (a
> clipped accel, a glitched GPS). *(`tsdownsample` does not exist on crates.io — verified.)*

**Writes.** tlog goes through a bounded channel to a dedicated thread with a 1 MiB `BufWriter` and
`write_vectored`, drop-oldest on full. MP does `BitConverter.GetBytes` + `packet.ToArray()` under
`lock(logfile)` inline on the link thread.

**Outbound.** Per-link token bucket (`baud/10` B/s) + priority queue: P0 RC override, P1 commands,
P2 params, P3 mission, P4 MAVFTP bulk. This is what deletes `giveComport` — the global flag that
stalls *all* telemetry during a parameter download.

### 8.2 Targets — every one gated in CI with an absolute floor

| Metric | Target | MP today (derived) | Measured how |
|---|---|---|---|
| Startup → first frame | ≤300 ms cold, ≤120 ms warm | 3–10 s (splash + plugin load + IronPython + WinForms) | instrumented `main`, 20-run median |
| Idle, disconnected | **0 frames rendered**, <0.1% of a core | polls at 1 ms | 5-second CI assertion |
| Idle, 1 vehicle @10 Hz, HUD visible | ≤2% of one core | — | `perf stat` over 60 s |
| Packet → snapshot published | p99 ≤200 µs | — | hdrhistogram in the loopback rig |
| Packet → pixel | p50 ≤1 vsync + 1 ms (~9 ms @60 Hz), p99 ≤2 vsync | p50 ~60 ms, p99 ~150 ms (1 ms poll + 100 ms binding tick + 30 ms HUD self-throttle at `HUD.cs:1284`) | stamped synthetic ATTITUDE read in the HUD element's render |
| Stick → syscall out | p50 ≤2 ms, p99 ≤5 ms | p50 ~65 ms (Thread.Sleep(50) + Task.Delay(40) + 15.6 ms Windows timer) | 50 Hz sampled histogram |
| Map frame, 2560×1440, 135 tiles + 6 overlays + 1M-pt track | p99 ≤8.33 ms | — | `wgpu-profiler` per pass |
| HUD frame, ~200 primitives | p99 ≤2 ms | hard-capped 33 fps | same |
| Chart, 8 series × 10M samples | p99 ≤8.33 ms, draw calls **independent of sample count** | undecimated `PointPairList` → GDI+ | draw-call counter in headless harness |
| tlog parse | ≥300 MB/s; 1 GB replays ≥1000× realtime headless | byte-at-a-time `ReadByte()` | `xtask replay --speed max` |
| dataflash parse | ≥300 MB/s; 1 GB opens ≤2 s cold | 3 parallel `List<long>` indexes = 24 B/record; ~530 MB steady on a 1 GB log | same |
| Steady-state allocations | **0 per packet** | ~8–12 | `divan` counting allocator |
| Memory: idle / 1 vehicle / 256 vehicles | ≤80 MB / ≤250 MB / ≤400 MB RSS | — | 8 h soak, fail if slope >1 MB/h |

Per-vehicle memory budget 256 KB: snapshot ~0.7 KB + params ~1,200 × 40 B + sparse last-frame cache
~60 × 280 B + trail ring 4,096 × 24 B.

---

## 9. GPU visual architecture

Six domains. Each non-chrome domain renders into its own texture, composited as one layer, dirty-
flagged and cached so panning the map does not re-render the HUD. All passes recorded into **one
`CommandEncoder` per frame**, submitted during element prepaint — same-queue ordering guarantees our
writes land before gpui's draw, so **no fences**.

**The colour contract, written once, enforced in every layer shader.** gpui deliberately selects a
**non-sRGB** surface format (`wgpu_renderer.rs:366` prefers `Bgra8Unorm`), blends in sRGB-*encoded*
space, and applies DirectWrite-style text gamma (default 1.8). So layer textures must be sRGB-encoded
`Unorm` (**never** a `*_SRGB` view) with `linear_to_srgb` applied as the **last** operation, or
composited colours silently mismatch the chrome. Copy gpui's helpers verbatim from
`shaders.wgsl:209-238` so there is one definition.

### 9.1 Map (replaces GMap.NET 33,899 LOC + `ExtLibs/Maps` 7,992)

- **Tiles:** ONE 256×256×N 2D **texture array** with an LRU layer allocator — *not* gpui's sprite
  atlas, which defaults to 1024×1024 pages and would split ~135 visible tiles across ~9 pages with
  re-upload on every pan. All visible tiles in **one instanced draw**. Parent-tile fallback as a
  second instanced pass with a src-rect.
- **Precision:** Web Mercator in **f64** on CPU, converted to **camera-relative f32** before upload.
  At z20 world pixel coords reach 2^28 and raw f32 loses sub-pixel precision — exactly where RTK and
  survey users care. Designed in, never retrofitted.
- **Disk cache:** byte-compatible with
  `gmapcache/TileDBv3/<lang>/<provider>/<z>/<y>/<x>.jpg` (operators carry multi-GB offline caches
  into the field; migration is a non-starter) **plus** a `redb` sidecar index so existence checks are
  one B-tree probe instead of a syscall storm. Add TTL/etag revalidation, which GMap lacks.
- **Providers:** the 71 `*Provider*.cs` classes + MP's 24 collapse to ~200 lines of TOML descriptors
  plus ~5 trait impls for the ones with real logic (Bing quadkey, Google session scrape, WMS/WMTS
  GetCapabilities, GDAL raster). **Audit the dead endpoints** — CloudMade is shut down; the
  Lithuania/Latvia/Czech services moved.
- **Vectors:** polylines as instanced screen-space expanded quads (endpoints in a storage buffer,
  miter/round joins in the vertex shader — no CPU tessellation on the hot path, device-pixel-exact
  width, O(1) appends). Polygons tessellated **once** with lyon and cached.
- **Markers:** the 36 types re-authored as **tessellated vector geometry**, not rotated bitmaps —
  which also sidesteps the verified asymmetry that `PolychromeSprite` has **no transformation field**
  while `MonochromeSprite` does, and gives resolution independence plus per-vehicle tint for swarms.
- **Track LOD:** Douglas-Peucker pyramid built once at load, level *k* at 2^k screen px, selecting
  ~0.5 px tolerance. This is what turns MP's hardcoded **200-point** live-track cap
  (`FlightData.cs:3785`) into 1,000,000 points at 120 fps.
- **New capabilities MP lacks:** continuous fractional zoom and free rotation.

### 9.2 HUD (replaces `HUD.cs`, 3,856 LOC)

~2,400 LOC of it is a 2D graphics abstraction — dual OpenGL-immediate-mode and GDI+/Skia backends, a
hand-rolled per-character GL texture atlas (`GL.GenTextures` **per glyph**, `HUD.cs:3452-3597`),
stencil-buffer polygon fill — and **evaporates**, because gpui owns the atlas and the GPU. Only the
~1,380 LOC of instrument geometry in `doPaint()` (1954–3333) is ported: horizon, pitch ladder clipped
to ±40°, roll arc, flight-path vector, heading tape with target bug, xtrack bar, rate-of-turn, both
scrollers, VSI, mode/wp text, AOA/SSA, dual battery with per-cell volts, dual GPS fix/HDOP, custom
user items, armed banner, pluggable prearm/EKF/vibe icon slots.

Delete the 30 ms self-throttle at `HUD.cs:1284` that hard-caps it at 33 fps. Keep all HUD paths at
**one paint order** so they collapse into a single intermediate MSAA pass. Design it to render to an
**arbitrary offscreen target** from the first line — OSDVideo and the web stream both grab
`hud1.objBitmap`, and retrofitting that later is painful.

### 9.3 Charts (replaces ZedGraph, 52,265 LOC)

Port the *semantics* of `Scale.cs`/`LogScale.cs`/`DateScale.cs` — tick-step selection, nice-number
rounding, minor tics, auto-range with grace — which is the genuinely subtle part worth reading
ZedGraph for, and nothing else. Series drawn as ≤2×pixel-width min/max bars from the pyramid;
decoration as quads, paths and text. Cache built `Path` objects in view state keyed by
`(data_generation, viewport, zoom)`.

**Do not reproduce `zg1_ZoomEvent`** (`LogBrowse.cs:2906`), which on *every* zoom and pan clears
`GraphObjList` and re-enumerates the entire log five times. Extract MODE/ERR/EV/MSG/PARM **once** at
load into small side tables; zoom becomes a pure viewport transform.

### 9.4 Video, 3D, propagation — the three that need the fork (or a workaround)

| Feature | Unforked path | Forked path |
|---|---|---|
| Live video | NV12 → `RenderImage` CPU round-trip; ~8.3 MB readback + 8.3 MB upload per 1080p frame (~500 MB/s @60 Hz) → cap at 30 Hz | Y (`R8Unorm`) + UV (`Rg8Unorm`) planes straight into gpui's **existing, unhooked** `fs_surface` YUV sampler (`shaders.wgsl:1313-1358`) — zero CPU colour conversion |
| 3D terrain (`OpenGLtest2.cs`, 1,766 LOC, allocates a fresh `tileInfo` + VBO **per waypoint marker per frame**) | render offscreen, composite via `RenderImage`, 30 Hz | geometry clipmap with GPU vertex displacement from an `R16Uint` height texture array sharing the map's tile pyramid — zero CPU vertex work |
| RF propagation (`Propagation.cs` ray-marches `srtm.getAltitude` per sample on a worker thread) | rayon CPU fallback | WGSL compute pass, radial LOS march per output texel into a storage texture, colour-map in the fragment shader — three orders of magnitude |

Keep the pipeline strings from `CameraProtocol.cs:37-95` **verbatim** (battle-tested config per
`VIDEO_STREAM_TYPE`), changing only the tail from
`videoconvert ! video/x-raw,format=BGRA ! appsink` to NV12. Assert in tests that no `videoconvert`
element survives.

---

## 10. Phased roadmap

**Shape: capability-first spine, factory mechanics grafted in, risk spikes run in parallel rather
than serially in front.** Every phase ends at something a pilot can do — the only honest progress
metric, and the property that survives cancellation. The risk work (GPU spike, kill criteria,
toolchain) is a *parallel week-1-to-3 track*, because there is no reason for transport and protocol
work to wait behind it, and no reason for the first window to wait behind 250k LOC of headless code.

**Staffing assumption, stated so the calendar is derivable:** 1 owner (decisions, G3 review,
decomposition), 1–3 engineers, an agent fleet of 8–14 concurrent units. Calendars below assume
**2 engineers + owner**. With 1 owner alone, multiply by ~1.8 (review is single-threaded).

| Ph | Name | Weeks | G3 hrs | Exit criteria (falsifiable) | Kill / pivot |
|---|---|---:|---:|---|---|
| **0** | Ground truth & spikes | 3–4 | 40 | Ledger 3,678/3,678 classified incl. `ExtLibs/mono`; toolchain + profiles + mold fixed; oracle builds **from source** in CI and answers 15 verbs; WinForms-stub experiment answered yes/no; all 4 generators emit and re-emit byte-identically; **GPU spike ADR written with numbers from Linux AND Windows AND a degraded target** | A1–A4 (§2.4) → pivot to egui; if the WinForms stub fails, root-level code moves to `spec+review` and MagCalib gets golden files from Windows |
| **1** | FLY (thin slice) | 10–14 | 120 | **M1** below | If M1 clause 2 or 3 fails on gpui, the framework decision reopens; everything L0–L5 stands |
| **2** | Retire the vendored mass | 3–4 | 30 | 731,732 LOC (T0) in terminal state with a named replacement crate + licence verdict each; `cargo deny` green; **measured fleet rate published** and Phases 3–8 re-forecast in writing | G1 first-pass <40% or mean rework >2 loops → re-cut unit granularity before scaling |
| **3** | PLAN (missions, fences, survey, map editor) | 8–10 | 90 | 200-wp mission round-trips bit-identical on real hardware over a 10%-loss link; `xtask diff grid\|corridor\|rotary` green at class C on ≥30 real polygons; survey flown in SITL | Grid golden tests cannot be made to pass → escalate to owner: bug-compat or corrected |
| **4** | REVIEW (logs, charts, LOD) | 8–10 | 90 | 2 GB `.bin` opens ≤4 s cold; 10M×8 scrub p99 ≤8.33 ms with work provably O(width); `dflog-dump` class-A across 100+ `.bin`; all 659 `<expression>` elements in `graphs/*.xml` match; 18 LogAnalyzer checks native | If the pyramid cannot hold 8 series, drop the chart target to 60 fps before changing framework |
| **5** | CONFIGURE (params) | 11–14 | 100 | ~12–16 panels schema-driven with **zero** hand-written layout; the other ~48 hand-ported and each recorded in the ledger as `schema` / `schema+logic` / `hand`; ConfigRawParams filters 2,500 params per keystroke at p99 ≤8.33 ms | — |
| **6** | BRING UP (calibration, firmware, radios, SITL) | 8–10 | 110 | **HIL rig green 20 consecutive runs, zero bricks**; CompassCalibrator bit-exact on 3 MAG sets incl. `get_completion_mask()`; MagCalib class-D on ≥50 logs; SiK + RFD900x configured and flashed on real hardware | px4uploader byte-trace mismatch → stop, do not flash |
| **7** | SEE (video, gimbal, OSD, 3D) | 7–9 | 70 | 1080p60 under HUD at full rate with **no `videoconvert`** — or, if the fork was deferred, 30 Hz with the measured CPU/PCIe cost in an ADR; 3D terrain 50 km² at 60 fps with zero per-frame CPU mesh alloc | Fork proves untenable → ship video at 30 Hz via `RenderImage`, defer 3D |
| **8** | BUS + FLEET (DroneCAN, RTK, swarm, joystick) | 10–13 | 100 | Real CAN bus ≥3 nodes enumerates + firmware-updates over SLCAN **and** SocketCAN; SLCAN mode-switch 50× without losing telemetry; RTK FIXED on 2 vehicles; stick→syscall p99 ≤5 ms on Linux **and** Windows; 20-vehicle SITL swarm at 120 fps | gilrs cannot expose raw HID axes → hidapi fallback (+~800 LOC) |
| **9** | SHIP (i18n, packaging, update, crash) | 6–8 | 50 | 18 locales incl. RTL + CJK on 3 OSes; `cargo dist` emits MSI/dmg/deb/rpm/AppImage from one tag; **ed25519-signed** update installs and rejects a tampered artifact; minidump on all 3 OSes | Apple/EV identity not obtained → ship unsigned with a named regression |
| **10** | EXTEND (plugins, scripting) | 5–6 | 50 | A malicious extension (infinite `loop()`, unauthorised `mavlink:send:COMMAND_LONG`, out-of-namespace file read) is **preempted or refused with no dropped frame**; 19 stock scripts run under rhai with a working kill switch | UI-capability decision (§12 D7) forced by FaceMap/Dowding |
| **12** | NATIVE PLUGINS (`PluginLoader` parity) | 4–6 | 40 | A native plugin built out-of-tree loads, registers a panel and a menu action, reads telemetry and sends a command; an ABI-mismatched plugin is refused by version, not by crashing; a panicking plugin does not take the GCS with it; `--safe-mode` loads none; loose-source plugins compile and load when a toolchain is present | Owner may stop after the sandboxed host (Phase 10) if full-trust loading is judged not worth the failure modes |
| **11** | CLOSE (the long tail + certification) | 8–12 | 80 | **3,678/3,678 in terminal state**, every `dropped` with an owner-ratified reason; full-corpus differential green 7 consecutive nightlies; mutants ≤10% on class-A/D; exhaustive 2^32 f32 param-rounding sweep clean; pilot flight test on 3 OSes | — |

**Totals: 87–114 weeks with 2 engineers + owner ≈ 20–26 months** to full closure; **~35–47 months
with a single owner** (review-bound). `~930 G3 review-hours` is the number that dominates.

### 10.1 M1 — "FIRST FLIGHT", the first falsifiable milestone

**Target: end of Phase 1, week 14–18** (not week 10 — that estimate was off by ~3×; gpui ships no
text input, no table and no chart, and clause 2 alone is multi-week after the framework works).

Demonstrated live on **all three** desktop OSes from one `cargo build --release`:

1. Connect to SITL over TCP **and** a physical Cube Orange+ over USB, auto-detected, <2 s to first HEARTBEAT.
2. Live HUD from gpui Path/Quad/glyph primitives, **120 fps, frame CPU <2 ms at 2560×1440**, per-pass numbers from `wgpu-profiler`.
3. Map with ≥3 providers, GPU-resident texture array, live marker, and a **1,000,000-point** trail via the LOD pyramid, panning at 120 fps.
4. Full ~1,200-param download in <3 s via the MAVFTP `param.pck` fast path, with the `PARAM_REQUEST_LIST` fallback proven by forcing the fast path to fail; displayed in a virtualized table filtering per keystroke without a dropped frame.
5. Writes a legacy-format `.tlog` that **opens in real Mission Planner**, and replays a 100 MB third-party tlog at ≥300 MB/s.
6. **0 heap allocations per packet** (counting-allocator test) and **0 frames rendered** when disconnected (5 s CI assertion).
7. Every on-screen string through `fl!()`; every altitude carries a frame in its type.

**If clause 2 or 3 fails, the Phase 0 spike was wrong and the framework decision reopens** — that is
the point of putting them in M1.

### 10.2 M0 — "LIT PIXEL", week 8–10, Linux only

Inserted because a 14-week gap with nothing on screen is the largest cancellation risk in the
programme. Connect to SITL over TCP; decode HEARTBEAT / ATTITUDE / GLOBAL_POSITION_INT; draw a
static-geometry HUD and one map tile layer in a gpui window at 60 fps; write a tlog real Mission
Planner opens. Falsifiable, kills the gpui unknown, reachable.

### 10.3 The MVP cut line — what ships if the programme stops

Define and gate this explicitly, because "stops at 60% with nothing shippable" is the classic failure
profile of a rewrite against a working incumbent:

> **v1 = connect (serial/TCP/UDP) + telemetry & HUD + moving map with tiles and unbounded track +
> full parameter table with load/save + mission upload/download/edit + dataflash & tlog review.**

That is ~50k C# ported + ~55k Rust greenfield ≈ **90–110k hand-written Rust**, reachable in
**10–14 months with 2 engineers**, on Linux and Windows. It is a product. Phases 6–11 are **scope**,
gated on v1 shipping — not obligation.

---

### 10.4 Phase 12 — in-process arbitrary-code plugins, and what "equivalent" can mean

An earlier draft of this plan listed this as a decided non-goal. The owner overruled it: the goal
is 100% of Mission Planner, and `PluginLoader.cs` is part of Mission Planner. It goes last, after
everything else, because it is the one capability whose *absence* costs nothing to a pilot and
whose *presence* can cost everything to a flight.

**What the C# does.** `PluginLoader` scans a directory, Roslyn-compiles loose `.cs` files at
startup, `Assembly.Load`s prebuilt DLLs, instantiates anything deriving from `Plugin`, and calls
`Init`/`Loaded`/`Loop`/`Exit` in-process with full trust and direct access to `MainV2` and
`CurrentState`. There is no sandbox and no capability model: a plugin is the application.

**What Rust can do, in descending order of fidelity.**

| Mechanism | Fidelity to `PluginLoader` | Notes |
|---|---|---|
| Native `cdylib` via `libloading` | High | A versioned C ABI entry point, a host vtable for telemetry/send/UI registration. Truly in-process, truly arbitrary, exactly the same trust model |
| Loose `.rs` compiled at load time | High | The literal analogue of Roslyn compiling loose `.cs`: invoke the toolchain, cache by content hash, `dlopen` the result. Requires a toolchain on the machine, which Roslyn did not |
| Sandboxed WASM extension (Phase 10) | Medium | Safe and portable, but cannot do the arbitrary in-process things a `PluginLoader` plugin can, so it is a complement rather than a replacement |
| Loading existing C# plugin assemblies | **Impossible without a CLR** | Stated plainly rather than promised. Existing plugins must be rewritten against the native or WASM API; a migration guide is part of the deliverable |

**The failure modes are the deliverable.** Full trust in-process means a plugin can corrupt vehicle
state, block the render thread, or crash the GCS mid-flight. The phase is therefore judged on its
guard rails as much as its loader: an ABI version check that refuses rather than crashes,
`catch_unwind` at every boundary crossing, a `--safe-mode` flag that loads nothing, per-plugin
opt-in recorded on disk, and crash reports that name every loaded plugin. A GCS that silently dies
because a third-party plugin dereferenced a null pointer is worse than one that never loaded it.

## 11. Risk register

| # | Risk | L | I | Early warning | Mitigation |
|---|---|---|---|---|---|
| R1 | gpui cannot meet the Windows/degraded-target bar (D3D11 hard-fails with *"Required feature StructuredBuffer is not supported"* and has **no WARP/GL/software fallback**) | med | **fatal** | RDP or VM smoke test refuses to open a window | A2 in §2.4 → egui+wgpu (`eframe` ships a `glow` backend). L6 rule bounds the cost to 2 crates |
| R2 | Review bandwidth, not agents, is the throughput ceiling (~930 G3 hours) | **high** | high | Ledger `claimed` count grows while `reviewed` flat for 2 waves | Staff 3–4 reviewers, or cut to the §10.3 MVP. Publish per-phase G3 hours |
| R3 | Silent omission: an agent ports 80% of a file, reports done, green build, missing feature found months later | **high** | high | `omissions` fields trending empty on large units | Mandatory non-empty omissions >200 LOC (machine-enforced) + G2 method-checklist by a different model |
| R4 | 22 XL files (73k LOC, no `#region`) stall the dispatcher | **high** | high | `xtask next` emits nothing for a crate for a full wave | 22 budgeted owner decomposition sessions in §6.5, **before** dispatch |
| R5 | Mono oracle diverges from .NET 4.7.2 on float formatting / CultureInfo / DateTime | med | high | Class-A diff fails only on culture-sensitive verbs | Windows CI runner with real 4.7.2 for those units; golden files committed |
| R6 | Vendored replacement does not fit (i_overlay vs Clipper scaled-integer semantics; `dxf` MLINE; `mgrs` band letters; proj4rs CRS coverage) | med | med | API-fit spike fails in Phase 0 | Named fallback per crate; worst case hand-port (`MGRS.cs` is 624 self-contained LOC) |
| R7 | The 209k LOC of WinForms-coupled code has no oracle and drifts silently | **high** | high | `spec+review` bucket grows with no spec written | Separate `verification_class` reporting; structural snapshots; 2-person review |
| R8 | gpui pre-1.0 churn / fork maintenance across 3 shader languages | med | high | A rev bump costs >1 engineer-week | Exact pin; `mp-ui` facade; A4 tripwire; upstream the surfaces patch |
| R9 | Firmware flashing bricks hardware (no SITL equivalent) | med | **fatal** | px4uploader byte-trace mismatch | HIL rig from Phase 0; byte-level trace comparison before any real flash; 2-person review |
| R10 | Cancellation during the long UI stretch | med | **fatal** | No demo for >6 weeks | M0 at week 8–10; capability-named phases; MVP cut line |
| R11 | Long tail silently abandoned (ConfigHWBT, Ateryx, NMEA2000, Dowding, 12 stray locales, 32 example plugins) | **high** | med | Ledger rows stuck in `ready` with no owner | Phase 11 exists solely for this; a `dropped` row needs an owner-ratified reason and is as auditable as `done` |
| R12 | Losing the shipping Android product without a decision | med | high | Nobody raises it | §12 D1 forces the ruling before Phase 2 |
| R13 | Build/CI cost (8–15 GB target per worktree; 3-OS matrix over a 747+ crate closure) | med | med | PR CI >25 min | mold + hakari + shared target + sccache, day 1 |
| R14 | Crowdin continuity broken (a decade of community translation across 18 locales) | low | high | Key renames after the first regeneration | Immutable committed `keymap.toml`; TMX export first; one-time pre-translation upload; **normalise the 12 stray locale codes before upload** |
| R15 | External-party lead times on the critical path (Apple identity, EV cert, Crowdin owner, HIL hardware, pilot) | med | med | Phase 9 arrives with no signing identity | Start procurement in Phase 0; 2–8 weeks calendar that agents cannot compress |
| R16 | `ExtLibs/mono` submodule expands the denominator after the ledger is sealed | low | med | — | Fetch and measure in Phase 0 week 1 |

---

## 12. Open decisions for the owner

| # | Decision | Recommendation | Cost of deciding wrong |
|---|---|---|---|
| **D1** | **Drop Android/iOS?** 39,234 LOC of real app; `android.yml` signs an AAB and uploads to Play Store `com.michaeloborne.MissionPlanner`; `mac.yml` builds Xamarin.iOS. gpui has no mobile backend. | **Drop, explicitly and on the record.** Also drops the entire `#if LIB` / `ZZZLibShims` apparatus (~3.4k LOC) that MP's own CLAUDE.md calls the #1 trap. | Wrong-drop: lose a shipping product and its users. Wrong-keep: write a gpui mobile backend from zero, or carry a second UI stack. Decide **before Phase 2**. |
| **D2** | **Sourcing: crates.io gpui 0.2.2 (blade) or git-pinned wgpu?** | **crates.io now** (verified working here), git pin only if the spike proves the fork is needed. | Wrong: a git pin to a 245-crate monorepo with no semver, MSRV forced to 1.98.1, and the platform crates are structurally unpublishable. |
| **D3** | **Degraded targets: must we support RDP / VM / basic display adapter?** | Enumerate them with users, then make it an A2 kill criterion. | If yes and gpui cannot start, this is a framework-level finding, not tuning — and it is the strongest argument for egui. Decide in **week 2**. |
| **D4** | **Projections: keep Shapefile import + arbitrary-CRS GeoTIFF?** `DotSpatial.Projections` supplies EPSG/proj4/ESRI-WKT reprojection to `FlightPlanner.cs:3533` and `GeoTiff.cs:232`. | Audit which CRSs real user GeoTIFFs use; if narrow, drop and restrict to WGS84/UTM with a named feature loss. | Keeping it = `proj 0.31` + libproj as a native dep on all three platforms. Dropping it = two silent feature losses. |
| **D5** | **`httpserver.cs` control routes:** `/command_long`, `/rcoverride`, `/guided?`, `POST /guide`, `/websocket/raw` let any local HTTP client **arm and fly the aircraft with no authentication**. | Keep the read routes (`/hud.jpg`, `/map.jpg`, `*.kml`, `/mav/`), require auth on the control routes, bind to loopback by default. | Shipping them unchanged directly contradicts the plugin-sandbox pitch. Removing them breaks documented third-party integrations. |
| **D6** | **Scripting: rhai only, or Python compat?** 19 stock scripts, unknown number in the wild. | **rhai** — with the operation limits and kill switch IronPython never had. These scripts send RC overrides to flying aircraft. | Python compat via `pyo3` forfeits the single-static-binary property and complicates signing/notarization on all three OSes. |
| **D7** | **Do extensions get custom UI?** `IHudIconRenderer` (a plugin drawing into the HUD **every paint cycle**) and `IDynamicParameterControl` do not fit "declarative overlays + declarative menus". | A declarative UI-description capability, plus a native trusted tier for first-party features. | Decide **before the WIT is frozen** — retrofitting a versioned ABI is far more expensive later. |
| **D8** | **Bug-for-bug or corrected?** Per case: MP's NTRIP GGA checksum defect (a transient zero re-seeds the XOR); the MAVLink2 replay window MP computes then discards; LogAnalyzer thresholds tuned for pre-2016 firmware; .NET banker's rounding in param display. | Correct them, each as a logged divergence. | Fixing the replay window means some existing signed setups start rejecting packets. Every one needs an explicit ruling. |
| **D9** | **Legacy hardware:** APM1/APM2 STK500 (954), 3DR Solo (406, hardcoded root creds in source), Parrot Bebop/Disco (shells out to a bundled `adb.exe`), VRBrain. | Drop all; ~2k LOC and several Windows-only paths. | Small user cohorts with no upgrade path. |
| **D10** | **Windows shell integration:** 4 ProgIds (`.tlog`, `.dfbin`, `.log`), the COM thumbnailer, a **separate `driver.msi`**, the **.appx Store channel**, and the `signtool /n "michael oborne"` identity. | Keep file associations (+ Linux `.desktop`/MIME, macOS UTI); audit whether the 7.4 MB of drivers is still needed on Win10+; decide the Store channel. | "Double-click a .tlog opens Mission Planner" is behaviour every existing user has. |
| **D11** | **User-data directory name and migration.** `Settings.AppConfigName = "Mission Planner"` → `~/Documents/Mission Planner/`, plus ~20 files (`authkeys.xml`, `cameras.xml`, `checklist.xml`, `warnings.xml`, `UserAlerts.json`, `poi.txt`, `History/`, OEM `logo.png`/`logo.txt`…). | New directory name (must **not** collide, or a user running both loses data) + a one-shot importer per artefact. | Silent settings/state loss is the most visible possible regression. |
| **D12** | **Staffing.** | 2–3 engineers + 1 owner, or cut to the §10.3 MVP. | With one owner, review alone is 35–47 months for full closure. This is the single biggest schedule lever. |

---

## 13. First two weeks — do this now, in this order

**Week 1 — unblock, then measure.**

| # | Action | Done when | Owner |
|---:|---|---|---|
| 1 | `git submodule update --init` on a scratch copy of `referneces/missionplanner`; measure `ExtLibs/mono` | LOC and file count known; either rows added or one owner-ratified `dropped` row written | eng |
| 2 | Install `mold`; wire `-C link-arg=-fuse-ld=mold`; set `lto="thin"`; replace `[profile.dev.package."*"]` with zed's ~15 targeted entries; **add `[profile.dev.build-override]`**; add `[profile.dev]` incremental/codegen-units/split-debuginfo; add `cargo-hakari` | cold + warm workspace build times recorded in `docs/baselines.md`, ≥2× better | eng |
| 3 | **GPU SPIKE A — Linux.** 135 tiles via `paint_image` with a stable `ImageId` per `(provider,z,x,y)` + ~200 HUD-equivalent paths/glyphs, at 2560×1440. Record p50/p99 frame CPU and per-pass GPU on the Quadro T2000 **and** under lavapipe (`VK_DRIVER_FILES=/usr/share/vulkan/icd.d/lvp_icd.json`) | numbers in `docs/adr/0002-gpu.md` | eng |
| 4 | **GPU SPIKE B — Windows/degraded.** RDP into a Windows box; build and launch a gpui app (zed's own examples will do). Does a window open? Then the same tile+HUD benchmark on D3D11 | A1 and A2 answered yes/no **in writing** | owner+eng |
| 5 | **Oracle from source.** Replace `tools/csharp-reference/regen.sh`'s `MP_DIST` dependency with `msbuild -t:Restore,Build` over `MissionPlanner.Utilities.csproj` (proven: 18 DLLs, exit 0). Cache as a CI artifact keyed on the MP SHA. Fix `ci.yml`'s "differential" job to actually run it | `mp-oracle geodesy` reproduces `zone=-56` from a clean container in <10 s | eng |
| 6 | **WinForms-stub experiment** (the one that settles the root-level oracle gap): write a ~300-LOC stub `System.Windows.Forms` + `MissionPlanner.Controls` facade with the ~12 types `MagCalib.cs` touches; `mcs MagCalib.cs` against the built `MissionPlanner.Utilities.dll` | yes/no in writing. If no → MagCalib and all root-level code move to `verification_class = spec+review` + Windows golden files | eng |
| 7 | Bump `rust-toolchain.toml` and `rust-version` **only if** D2 chooses the git pin; otherwise stay at 1.95.0 and record why | ADR-0003 | owner |

**Week 2 — governance, then the first real unit.**

| # | Action | Done when | Owner |
|---:|---|---|---|
| 8 | **Seed the ledger.** All 3,678 rows (+ mono) from the five-tier partition, with `tier`, `disposition`, `target_crate`, `size`, `fidelity_class`, `perf_class`, `verification_class`, `sha256`. `cargo xtask ledger check` exits 0 | LOC sums **exactly** to 1,208,836 + mono; zero unclassified | eng |
| 9 | **Build the dependency graph.** `petgraph` topo-sort over the 360 `ProjectReference` edges across 129 csproj, then a file-level `using`/type graph via `tree-sitter-c-sharp` with SCC collapse | `deps` column populated; `xtask next --agents 8` emits byte-identical output twice | eng |
| 10 | **Re-enter the 5 existing commits under contract.** 9 crates, ~8.2k hand-written Rust — none has an omissions list, a fuzz target, a G2 review or a live oracle gate | every existing file has a ledger row in `ported` (not `done`) | eng |
| 11 | **Spike `tree-sitter-c-sharp 0.23.5` against the `tree-sitter 0.27` runtime** (ABI pair, unverified). Then run `screenspec` on `FlightData`, `FlightPlanner` and `ConfigArducopter` | 3 RON specs that round-trip every `>>Parent`/`>>ZOrder`; ABI verdict recorded | eng |
| 12 | **`dsdlgen` signature gate.** Parse all 147 `.uavcan`, emit, and compare every `DATA_TYPE_SIGNATURE` + `default_dtid` against `canard_dsdlc/messages.cs` | 147/147 match, or the Python-dump fallback is committed | eng |
| 13 | **Decomposition session #1:** `MAVLinkInterface.cs` (6,906 LOC, 0 regions). Produce a written module plan: target crates, sub-unit list, shared-type contract | committed to `ledger/decomp/MAVLinkInterface.md`; 6–10 child units created | **owner** |
| 14 | **Dispatch one real unit end-to-end** through the full contract: `xtask next` → agent → port with provenance header → tests → `xtask diff` green → fuzz target → G1 → G2 (different model) → G3 → merge → ledger flips to `done` **by machine** | the loop closes with nobody touching the ledger by hand | owner+eng |
| 15 | Start HIL hardware procurement and the Apple Developer / EV cert applications | POs raised — these have 2–8 week lead times that agents cannot compress | **owner** |
| 16 | Owner rulings on **D1 (mobile)**, **D3 (degraded targets)**, **D12 (staffing)** | ADRs written | **owner** |

**Action 14 is the milestone.** It proves the factory closes its first loop on a real file. Every one
of the remaining ~1,030 T3 units is then the same loop with different inputs, and project status
becomes a number the ledger prints rather than an opinion anyone holds. **If that loop cannot be
closed in week 2, the factory design is wrong and must be fixed before another agent is dispatched.**

---

## Appendix A — claims this plan refutes

Recorded so they are not re-asserted.

| Claim | Status | Evidence |
|---|---|---|
| "gpui does not link on this box" | **FALSE** for crates.io gpui | `cargo build -p mp-gui` exits 0; `target/debug/mpr-gui` links; runs 25 s under X11 |
| "Split the 8 giant files by `#region`" | **IMPOSSIBLE** | `grep -c '#region'` = **0** for FlightPlanner/FlightData/MAVLinkInterface/CurrentState/MainV2 |
| "MissionPlannerLib gives an oracle for root-level code" | **FALSE** | 399 errors; 5 `ProjectReference`s need `Microsoft.NET.Sdk.WindowsDesktop`; `MagCalib.cs` itself uses WinForms |
| "~31 switchboards = ~20k LOC of mechanically portable code" | **WRONG by ~2.5×** | 61 panels = 21,253 logic + 23,092 designer; only **8** have ≥10 `.setup()` calls, ~12 have ≥7; 354 setup calls total |
| "The oracle is the specification" | **TRUE for ExtLibs, FALSE overall** | 422 non-vendored non-designer files / **208,951 LOC** reference `System.Windows.Forms` |
| "ExtLibs/Xamarin is sample code outside the shipping product" | **FALSE** | `android.yml` signs an AAB and uploads to Play Store; 39,234 LOC non-generated |
| "2,763 lines of `graphs/*.xml`" | **WRONG** | 2,235 lines / **659** `<expression>` elements; the extra 528 is `dataflashlog.xml`, which has zero expressions |
| "350 messages" as a CI gate | **OFF BY ONE** | 350 `message_info(` entries, one with a null name skipped by the dumper → **349** CSV rows |
| "The workspace is ~40% through Phase 0" | **WRONG both ways** | 5 commits / 9 crates / 48,775 Rust LOC (40,547 generated) — but it is *vertical-slice* work; Phase 0 is ~12% |
| "`Primitive` has an escape hatch" | **FALSE** | closed 8-variant enum; `paint_surface` macOS-only; `wgpu_renderer.rs:1545` is `PrimitiveBatch::Surfaces(_surfaces) => {}` |
| "`tsdownsample` crate" | **DOES NOT EXIST** | not on crates.io |
| "`evalexpr` for the expression engine" | **LICENCE BLOCKER** | 13.1.0 is AGPL-3.0-only |

