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
| Bug-for-bug WinForms **pixel** fidelity | MP ships `<dpiAware>false</dpiAware>`, so its own pixels are not stable across machines | **Layout fidelity is not dropped with it — see below.** Pixel-exact tracing is out; matching the arrangement is the default |
| ~~In-process arbitrary-code plugins~~ | **Overruled by the owner, 2026-09-23: "when I said 100% of MissionPlanner, I didn't mean 99%."** Moved to Phase 12, after everything else | See §10.4 |
| Reproducing MP's unsigned MD5 updater | `Utilities/Update.cs` trusts an MD5 list fetched over the network | Replaced by ed25519-signed artifacts |

> **Corrected by the owner, 2026-09-23.** An earlier version of this row said "screens are
> re-laid out, not traced", and that went too far. The ruling: *"i, the human am the final oracle
> on look/feel/style/implementation, but for most things, the closer it feels to an original MP
> layout, the better. where i choose to vary it, i say so here."*
>
> So the default is to match Mission Planner's arrangement — which panel holds what, what is on by
> default, what order things appear in — and a divergence is the owner's call rather than an
> implementer's convenience. What stays out of scope is *pixel* tracing, for the reason in the row
> above.
>
> This costs less than it sounds, because **the layout is not in the C# and does not need to be
> inferred from screenshots**: `FlightData.resx` alone carries 845 geometry entries and 834
> control-tree entries (`>>ctrl.Parent`, `>>ctrl.ZOrder`, `Location`, `Size`, `Anchor`, `Dock`).
> §4.3 and §6.1's `screenspec` generator already read it. Reading the resx is both more faithful
> and cheaper than guessing.
>
> Worked example, from this session: the live tuning graph was placed at the bottom of a sidebar
> because that was convenient. `GCSViews/FlightData.cs:1902` has it in `splitContainer1.Panel1`,
> collapsed until `CB_tuning` uncollapses it, above everything else. The faithful placement was
> also the better one — at the bottom of a column that already scrolls, a plot nobody can see
> without scrolling to it is a plot nobody watches.
>
> **Ratified divergences** live in `.claude/memory/match-the-original-layout.md` and are settled,
> not open questions. The first: **the colour palette stays dark and does not become Mission
> Planner's.** The owner's words — *"those colors in the original are nasty. do NOT implement the
> original color scheme (burnt frog)"*. Chrome only; the artificial horizon keeps blue over brown,
> which is the universal convention for an attitude indicator rather than an MP style choice.

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
| Polygon ops | ~~`geo 0.33` Buffer/BooleanOps + `i_overlay 9.0`~~ **Clipper 6.4.2 transliterated** (`crates/mp-mission/src/clipper.rs`, 2026-09-24) | Two vendored Clipper copies (4,930 + 4,815 LOC) | med | The fallback fired: the rotary grid's output is bit-for-bit the C#'s only with the C#'s own offset, union, and mono's unstable `List.Sort` inside it (§13.4 item 8). The port covers what the grids reach - mitred closed paths, union, both fills, the tree output - and names the branches it leaves out. Everything else that needs polygon ops still goes through the same module |
| Projections | **Unresolved — see §12 D4.** Either `proj 0.31` (native libproj) or drop Shapefile + arbitrary-CRS GeoTIFF | "~200 LOC UTM kernel" — cannot serve `DotSpatial.Projections`' EPSG/proj4/ESRI-WKT reprojection used by `FlightPlanner.cs:3533` (.shp import) and `GeoTiff.cs:232` | low | Audit which CRSs real user GeoTIFFs actually use |
| Video | `gstreamer 0.25` | `LibVLC.NET` (`vlc-rs` dead since 2018); `DirectShowLib` (37,629 LOC, Windows-only) | med | LGPL/GPL shipping obligations of the ffmpeg alternative |
| Charting | Write `mp-chart` on gpui primitives + LOD pyramid | `ZedGraph` (52,265 LOC); `plotters` (static backend, no interaction model) | high | — |
| Scripting | **`rustpython-vm 0.5`** — the 19 shipped scripts are already Python, so a Rust-native language would break every one of them; `pyo3` behind an opt-in feature for numpy/pymavlink users | `rhai 1.26` (fast and easily sandboxed, but forces a rewrite of user scripts that work today); `pyo3` as default (forfeits the single static binary) | med | Any stock script that will not run unmodified is recorded per script, not hand-waved (§12 D6) |
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

> **Superseded by the ledger, 2026-09-24.** `ledger/ledger.csv` classifies the same 3,678 files with
> a rule per vendored root that names its evidence (a licence header, a `LICENSE`, csproj metadata
> naming a third party, or §4.1) rather than a directory list: **T0 1,623 / 703,375 · T1 473 /
> 162,686 · T2 186 / 55,872 · T3 1,020 / 246,248 · T4 376 / 40,655**. The moves: swagger-generated
> `WebAPIs/*/src` to T1; `Core`, `Transitions`, `LEDBulb`, `Half`, `Tools.cs` and `FastJSON` to T3
> for lack of any third-party evidence in the tree (an owner can send them back with one line each);
> `uno`, `wasm`, `System.Drawing.android` and `UsbSerialForAndroid` to T4; and a few third-party
> files that had been hiding in T3 to T0. The table above is kept as the first measurement;
> `cargo xtask ledger status` is the current one.

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
| `ExtLibs/MetaDataExtractorCSharp240d` | 17,800 | `kamadak-exif` + `little_exif` | check maker-note tags first |
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
| 147 `.uavcan` DSDL | `out/include` 205 files + `out/src` 176 files, 23,374 LOC | `xtask dsdlgen` (**to write**) | all 147 `DATA_TYPE_SIGNATURE` + `default_dtid` must equal `canard_dsdlc/messages.cs` |
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
| 200–599 | ~210 | ~75k | 31% |
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

Measured on this tree: **82 commits, 13 crates, 36,603 hand-written Rust LOC** plus **91,634
generated**, and **531 tests** green on `cargo test --workspace`.

| Crate | Hand-written | Generated | Note |
|---|---:|---:|---|
| `mp-gui` | 11,514 | — | fly / plan / setup / params on gpui |
| `mp-link` | 2,903 | — | link engine: I/O thread, routing, snapshot publication, commands, mission transfer, `.tlog` recording |
| `mp-params` | 1,645 | 46,786 | parameter values, the downloaded table, metadata, `.param` files |
| `mp-calibration` | 838 | — | accelerometer, level, barometer, compass, radio, motor test |
| `mp-ftp` | 316 | — | dataflash log download; no MAVFTP yet |
| `mp-mission` | 2,621 | — | missions, fences, rally, survey grids |
| `mp-vehicle` | 1,612 | 190 | snapshot bus, RC channels, EKF/vibration health |
| `mp-mavlink` | 2,346 | — | ~45% tests |
| `mp-tiles` | 2,030 | — | fetch, decode, cache |
| `xtask` | 1,748 | — | mavlink codegen |
| `mp-transport` | 1,631 | — | serial/tcp/udp/replay |
| `mp-cli` | 1,520 | — | `mpr` |
| `mp-log` | 1,330 | — | tlog reading, dataflash, plot extraction |
| `mp-input` | 980 | — | joystick → RC channels |
| `mp-units` | 907 | — | typed units and geodesy |
| `mp-mavlink-dialects` | 488 | 44,658 | |
| `mp-fuzz-checks` | 369 | — | the fuzz properties, shared with the stable harness |

The `mp-link`, `mp-params`, `mp-calibration`, `mp-ftp`, `mp-vehicle` and `mp-log` rows were
re-measured after the split (§13.2 item 9) as `wc -l` of `src/`, tests excluded, with
`src/generated/` in the second column; the other rows are the earlier baseline's.

What exists is still *vertical-slice* work, now a fairly wide slice: the telemetry spine, the map,
mission planning, parameter configuration including `.param` interop, calibration, flight recording,
and the first joystick path. What does **not** exist: the ledger, the dependency graph, the
dispatcher, the DSDL/resx/screenspec/paramgen generators, the full corpus, the 3D and video paths,
i18n, packaging. The crate graph now has §5.1's layering for the crates that exist, and CI holds it
there: `xtask/tests/graph.rs` places every crate in a layer and fails on a UI framework below L6,
`mp-link` below L3, a dependency pointing up a layer, or a cycle. `mp-link` is the link engine;
parameters, calibration and log download are `mp-params`, `mp-calibration` and `mp-ftp`, and the
mission transfer machine stays in the link because `MAVLinkInterface` is where Mission Planner has
it. The six crates §5.1 does not name are placed in the test with a reason each, and `mp-gui` still
names gpui's platform crates, which §5.1 reserves for an `mp-render` that does not exist yet —
pinned in the test as rule 4's only exception.

`.github/workflows/ci.yml`'s "differential" job still neither installs mono nor references a Mission
Planner distribution — it runs `cargo test --test differential_tlog` against checked-in CSVs. **Phase
0 remains incomplete**: every existing file is re-entered into the ledger *under contract*, not
marked done.

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

> **As built (2026-09-24):** `cargo xtask ledger check` enforces the non-empty `omissions` from size
> M upwards - 200 lines and more, which is this section's threshold rather than "L/XL". The
> behaviour-difference entry and §7.2's tolerance class have no column yet; `fidelity_class` takes
> §1.3's four names. Adding the two columns is a schema change the checker will have to learn.

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
| **C** structural + invariants | identical point *count*, then ordering, then ≤1e-7 deg, **plus** invariants that hold even when C# and Rust tie-break differently: lanes one spacing apart to 1%, every part of a lane line inside the polygon is flown, and the polygon reaches less than one spacing past the outermost lanes. *(Corrected 2026-09-24: the earlier "every interior point within `spacing/2` of a lane" is false for Mission Planner's own output - the strip between an edge and the outermost lane reaches 0.9 of a spacing, and a sliver's tip sits three spacings from any lane. The restated invariants are checked against all 180 C# goldens.)* | survey grid |
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
| **10** | EXTEND (plugins, scripting) | 5–6 | 50 | A malicious extension (infinite `loop()`, unauthorised `mavlink:send:COMMAND_LONG`, out-of-namespace file read) is **preempted or refused with no dropped frame**; **all 19 stock `Scripts/*.py` run unmodified** under the embedded Python engine with a working kill switch | UI-capability decision (§12 D7) forced by FaceMap/Dowding |
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

### 10.4 Extension model — three tiers, because Mission Planner already has two

An earlier draft listed in-process arbitrary-code plugins as a decided non-goal. The owner
overruled it: the goal is 100% of Mission Planner, and `PluginLoader.cs` is part of Mission
Planner. The owner then asked whether plugins might use a dynamic language so nothing needs
compiling — which is not merely convenient, it is what Mission Planner already does.

**What exists upstream.** Two separate mechanisms, not one:

| Upstream | Mechanism | Users write |
|---|---|---|
| `Script.cs` + `Scripts/` | IronPython, interpreted in-process, no compile step | **19 shipped `.py` scripts**, API of `GetParam`, `ChangeParam`, `ChangeMode`, `WaitFor`, `SendRC`, `Sleep`, `runScript`, `mavlink_connection`, `recv_match` |
| `PluginLoader.cs` + `Plugins/` | Roslyn-compiles loose `.cs`, `Assembly.Load`s DLLs, full trust | Compiled C# plugins |

So the port needs both, and the scripting half is the one users touch.

**Tier 1 — embedded Python (Phase 10, the default).** Directly replaces IronPython. No compiler,
no toolchain, edit-and-run.

| Engine | Verdict |
|---|---|
| `rustpython-vm` 0.5.0 (MIT) | **Default.** Pure Rust, no system Python, builds for every target we ship including wasm, and sandboxable because we own the host bindings. Slower than CPython and its stdlib has gaps |
| `pyo3` 0.29 (MIT/Apache) | **Opt-in feature** for users who need numpy, scipy or pymavlink. Costs a CPython runtime on the machine and complicates packaging, so it is not the default |

The compatibility test is concrete: the 19 stock scripts must run unmodified, and that corpus is
checked in. IronPython is a Python 2.7/3.4-era dialect, so where a stock script does not run, the
divergence is recorded per script rather than hand-waved.

> **Measured, and it changes this section's premise.** `crates/mp-script` scans the corpus and
> `cargo test -p mp-script` prints the result: **15 of the 19 reach into .NET directly** —
> `MissionPlanner.MainV2.instance`, `System.Threading.Thread`, `clr.AddReference`. `Script.cs:40-44`
> loads every loaded assembly into the script namespace, so those are CLR types reached through
> Python syntax. **No Rust engine can provide them**, rustpython or otherwise, because there is no
> CLR behind it.
>
> Only **4** — `debugenv.py`, `example1.py`, `rc.py`, `wipe.py` — stay inside the scope bindings
> and could run on a Python engine alone.
>
> So the engine choice decides far less than this section assumed. Python is still right, because
> the scripts are Python and the four that are portable stay portable — but "the 19 run unmodified"
> is not a thing an engine choice buys. It needs a compatibility shim answering to the names the
> scripts already use (`MissionPlanner.MainV2`, `MAV.doARM`, `cs.alt`), which is a different and
> larger piece of work, and D16's estimate has to carry it. The eleven scripts that call `MAV.*`
> say where it starts.

**PHP was considered and rejected.** Not from taste: the Rust ecosystem has `php` 0.1.0 bindings
and `phprs` 0.1.x, a from-scratch VM — both far too immature to put under a ground control
station — and no Mission Planner user has a PHP script to preserve. Python is the compatible
choice precisely because the existing scripts are already Python.

**Tier 2 — sandboxed WASM extensions (Phase 10).** For extensions that need real speed or a
language other than Python, with a capability model. Portable, safe, and the right default for
anything distributed to other people.

**Tier 3 — native in-process plugins (Phase 12, last).** `PluginLoader` parity: native `cdylib`
loaded through `libloading` behind a versioned C ABI with a host vtable, plus load-time
compilation of loose Rust source — the literal analogue of Roslyn compiling loose `.cs`. Loading
existing *C# assemblies* remains impossible without a CLR; that is stated rather than promised,
and a migration guide ships instead.

**The failure modes are the deliverable.** Tier 3 means a plugin can corrupt vehicle state, block
the render thread, or crash the GCS mid-flight. The phase is judged on its guard rails as much as
its loader: an ABI version check that refuses rather than crashes, `catch_unwind` at every
boundary, a `--safe-mode` flag that loads nothing, per-plugin opt-in recorded on disk, and crash
reports naming every loaded plugin. A GCS that dies because a third-party plugin dereferenced null
is worse than one that never loaded it. Tiers 1 and 2 exist so that most users never need tier 3.

### 10.5 Measured: the bundled parameter metadata does not match shipping firmware

Recorded here because it was measured on this box, against a live vehicle, and it changes how D12
must be built.

`ParameterMetaDataBackup.xml` ships with Mission Planner and is the source of every parameter
description, range, enumeration and bitmask — and of the flight mode tables. Against ArduPilot
4.6.0-beta1 SITL:

| Measure | Result |
|---|---|
| Parameters the vehicle reports | 1,408 |
| With no bundled documentation | **581 (41%)** |
| Outside their documented range | 9, including `FENCE_TOTAL` and `EK3_ABIAS_P_NSE` |
| Renamed since the metadata | `ARMING_CHECK` → `ARMING_SKIPCHK`, `WPNAV_SPEED` → `WP_SPD` |

**Consequence.** A configuration screen driven by the bundled file alone shows nothing useful for
two parameters in five, and shows *wrong* ranges for a handful — which is worse, because a range
that disagrees with the firmware will reject a value the vehicle would have accepted, or accept one
it will not.

**Corrected here, because the first version of this row was wrong and it mattered.** The successor
to `ARMING_CHECK` is `ARMING_SKIPCHK`, not `ARMING_OPTIONS`. Both names exist on 4.7 — a parameter
download from this SITL returns `ARMING_OPTIONS`, `ARMING_SKIPCHK`, `ARMING_RUDDER`,
`ARMING_ACCTHRESH`, `ARMING_MAGTHRESH`, `ARMING_MIS_ITEMS` and `ARMING_NEED_LOC`, and no
`ARMING_CHECK` — so writing to `ARMING_OPTIONS` believing it disables checks silently does nothing
of the kind. **The sense is also inverted**: `ARMING_CHECK` was a bitmask of checks to *run*,
`ARMING_SKIPCHK` is a bitmask of checks to *skip*, so "all checks off" went from `0` to `-1`. A
force-arm written against the old name and the old sense is a force-arm that does not disarm
anything, and the vehicle that then arms does so for an unrelated reason — which is exactly what
happened here before it was traced. `crates/mp-gui/src/telemetry.rs` probes for one name and falls
back to the other.

**What D12 has to do instead.** Fetch metadata matching the firmware version the vehicle reports,
as Mission Planner itself does at runtime, and fall back to the bundled copy only when offline.
The bundled file stays as the offline default and as the codegen input for flight modes, which are
stable enough to compile in. Parameters are not.

**Cheapest experiment to size the fetch path:** take the `AUTOPILOT_VERSION` message the vehicle
already sends, resolve it to an ArduPilot release, and check whether that release's `apm.pdef.xml`
covers the 581 currently missing. That is a day's work and it decides whether D12 needs a metadata
cache with its own versioning or just a download.

> **Done, 2026-09-24, and the answer is "just a download".** The C# does not use
> `AUTOPILOT_VERSION` for this: it asks for the banner (`DO_SEND_BANNER`) and reads the version
> out of the `STATUSTEXT` that names the vehicle, then fetches
> `autotest.ardupilot.org/Parameters/versioned/<Vehicle>/stable-<version>/apm.pdef.xml` into the
> data directory as `<Vehicle><version>.apm.pdef.xml`, or the unversioned file, refreshed weekly.
> `mp_params::pdef` ports that. On this SITL's 1,408 parameters the bundled table documents
> **827** and the fetched `ArduCopter.apm.pdef.xml` documents **1,407** - the 581 become 1. The
> bundled table stays as the fallback and as the codegen input for flight modes, as this section
> said it should.

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
| **D6** | ~~**Scripting: rhai only, or Python compat?**~~ **SETTLED** — see §3 and §10.4. `rustpython-vm` is the default engine, `pyo3` an opt-in feature. rhai was rejected because it forces a rewrite of 19 scripts that work today. | What remains open is narrower: **which of the 19 stock scripts run unmodified**, recorded per script rather than in aggregate. IronPython is a 2.7/3.4-era dialect; rustpython is not. | A script that silently behaves differently is worse than one that refuses to run — these send RC overrides to flying aircraft. Each divergence gets a row, not a hand-wave. |
| **D7** | **Do extensions get custom UI?** `IHudIconRenderer` (a plugin drawing into the HUD **every paint cycle**) and `IDynamicParameterControl` do not fit "declarative overlays + declarative menus". | A declarative UI-description capability, plus a native trusted tier for first-party features. | Decide **before the WIT is frozen** — retrofitting a versioned ABI is far more expensive later. |
| **D8** | **Bug-for-bug or corrected?** Per case: MP's NTRIP GGA checksum defect (a transient zero re-seeds the XOR); the MAVLink2 replay window MP computes then discards; LogAnalyzer thresholds tuned for pre-2016 firmware; .NET banker's rounding in param display. | Correct them, each as a logged divergence. | Fixing the replay window means some existing signed setups start rejecting packets. Every one needs an explicit ruling. |
| **D9** | **Legacy hardware:** APM1/APM2 STK500 (954), 3DR Solo (406, hardcoded root creds in source), Parrot Bebop/Disco (shells out to a bundled `adb.exe`), VRBrain. | Drop all; ~2k LOC and several Windows-only paths. | Small user cohorts with no upgrade path. |
| **D10** | **Windows shell integration:** 4 ProgIds (`.tlog`, `.dfbin`, `.log`), the COM thumbnailer, a **separate `driver.msi`**, the **.appx Store channel**, and the `signtool /n "michael oborne"` identity. | Keep file associations (+ Linux `.desktop`/MIME, macOS UTI); audit whether the 7.4 MB of drivers is still needed on Win10+; decide the Store channel. | "Double-click a .tlog opens Mission Planner" is behaviour every existing user has. |
| **D11** | **User-data directory name and migration.** `Settings.AppConfigName = "Mission Planner"` → `~/Documents/Mission Planner/`, plus ~20 files (`authkeys.xml`, `cameras.xml`, `checklist.xml`, `warnings.xml`, `UserAlerts.json`, `poi.txt`, `History/`, OEM `logo.png`/`logo.txt`…). | New directory name (must **not** collide, or a user running both loses data) + a one-shot importer per artefact. | Silent settings/state loss is the most visible possible regression. |
| **D12** | **Staffing.** | 2–3 engineers + 1 owner, or cut to the §10.3 MVP. | With one owner, review alone is 35–47 months for full closure. This is the single biggest schedule lever. |

---

## 13. The queue

### 13.1 The twenty, and what they cost

Superseded the original two-week plan. Eighteen are closed, two are partly closed and say so. Each
row says what *done* means, because a list of nouns is not a plan.

| # | Item | Why it is next | Deliverable | Done when | Status |
|---:|---|---|---|---|---|
| 1 | Text field widget | gpui ships no input element, and three screens have already been shaped around its absence | D6 | typing works in a focused field; backspace does not split a codepoint; chords are not typed | done |
| 2 | Parameter search | 1,408 parameters behind a prefix list; typing `WPNAV` is how people actually find one | D12 | a name fragment filters the list across every group | done |
| 3 | Mission file names | save and load use one fixed path, so a second mission overwrites the first | D11 | a typed name round-trips a mission to and from disk | done |
| 4 | Settings that persist | link URL, tile provider, window size and screen are retyped every launch | D17 | they survive a restart and a corrupt settings file does not stop startup | done |
| 5 | Vehicle selector | the link tracks every vehicle on the wire and the UI always shows the first | D10 | a second vehicle is selectable and the map and HUD follow the selection | done |
| 6 | ADS-B and other vehicles on the map | a ground station that cannot show nearby traffic is missing the thing that prevents a collision | D15 | `ADSB_VEHICLE` is drawn, aged out, and distinguishable from the flown aircraft | done |
| 7 | KML export of a mission and a flown path | the usual way to hand a flight to someone without a ground station | D11 | the file opens in Google Earth with the track and waypoints in the right places | done |
| 8 | Live tuning graph | watching a value against time is how tuning is done, and the flight screen has no plot | D10 | a chosen field plots at telemetry rate without dropping the frame budget | done, not yet screenshotted |
| 9 | Log plotting | the same plot over a dataflash log, which is how a flight is reviewed | D14 | a field from a downloaded `.BIN` plots against time | |
| 10 | Terrain-relative altitudes | a mission flown at 50 m over a hill is a mission into a hill | D11 | `MAV_FRAME_GLOBAL_TERRAIN_ALT` round-trips and the planner says which frame an item uses | |
| 11 | Satellite imagery provider | planning over a paddock needs imagery, not a street map | D8 | a second provider is selectable and its attribution is shown | |
| 12 | Mission Planner tile cache compatibility | D8 asks for it, and it lets an existing cache be reused offline | D8 | tiles written by the C# application are read without a network | |
| 13 | Record a `.tlog` for every flight | the link can record and the GUI never turns it on, so every flight flown behind this application is unreviewable | D14 | recording starts on connect, both directions are captured, and the screen says it is on | done |
| 14 | Save, load and compare `.param` files | how an operator backs up a build, clones an airframe, or works out what a suggested change actually changed | D12 | a set round-trips against a file the C# application wrote, honouring its skip-list | done |
| 15 | EKF and vibration monitors | the two readouts that explain a vehicle that will not arm, flies badly, or climbs on its own | D10 | variance and vibration are shown, with clipping counts | done |
| 16 | Fuzz targets built and run | they exist, have never been compiled, and D2's DoD requires 24 h clean on `frame_parse` | D19 | the targets build and CI runs a bounded fuzz pass | done |
| 17 | Windows build verified | cross-compilation is checked; the Direct3D 11 path has never been exercised | D7 | a Windows build opens a window and paints, recorded in an ADR | done |
| 18 | Joystick input | flying from a ground station without a transmitter, which D15 names | D15 | axes map to `RC_CHANNELS_OVERRIDE` with a failsafe on disconnect | done |
| 19 | Firmware flashing | the last item in Initial Setup with no counterpart here | D13 | a `.apj` is written to a board over the bootloader and verified | protocol done, no board flashed |
| 20 | Python scripting host | D16, and the owner's stated interest in extensions that need no compiler | D16 | a script can read telemetry and drive a command, sandboxed | host API done, engine not wired |

**What they cost, recorded because the estimate was wrong in an instructive direction.** Six
items, all "small". Each one turned up a defect in something already believed finished, and the
defects were worth more than the features:

| Item | What it was supposed to add | What it actually found |
|---|---|---|
| 13 | turn recording on | the link recorded **inbound frames only**, so every command the ground station ever sent was absent from every log |
| 14 | `.param` files | the C# skip-list has **16 entries and applies on load**, not 7 on save; and its number format is shortest-representation, not fixed decimals. All three were reconstructed from memory instead of read from `ExtLibs/Utilities/ParamFile.cs`, which is in this repository. Corrected: the five extra names include the whole `BARO*_GND_*` family, which had been loading a previous day's ground pressure onto vehicles |
| 18 | flying from a gamepad | the failsafe measured **stick movement**, but `/dev/input/js*` is edge-triggered, so a held stick looked identical to an unplugged one and control was handed to a transmitter that this feature assumes is absent. Found by reviewing the code before committing it, not by running it |
| 15 | show two numbers | `VIBRATION` and `EKF_STATUS_REPORT` were arriving at 4 Hz and being discarded |
| 16 | run the fuzzers | `frame_parse` reaches **90** coverage edges and stops. It never decodes a message. The 351 per-message decoders — where a length field is trusted — had never been fuzzed by anything. `message_decode` reaches 13,473 |
| 17 | check Windows | nothing in CI had ever run a graphics backend on **any** platform. `open_window` failing printed an error and exited 0 |

The pattern: the expensive bugs were all in the gap between "the code exists" and "the code has
been run against the thing it is for". That is an argument for §6.4's five-artifact contract being
enforced on work done *before* the factory starts, not only on units dispatched through it.

### 13.2 The next ten, in order

Four carried over from §13.1 (9–12, never started), and six that the last stretch showed are owed.
Ordered by what an operator hits first, then by what unblocks the most.

| # | Item | Why it is next | Deliverable | Done when | Status |
|---:|---|---|---|---|---|
| 1 | Log plotting from a `.BIN` | the other half of the tuning graph: watching a value live is how a problem is noticed, plotting it afterwards is how it is diagnosed. `mp-chart` already holds the reduction | D14 | a field from a dataflash log plots against time, with the field chosen from what the log actually contains | done, with both axes: a left click graphs on the left, one axis per unit; a right click on the right, one shared axis (`Log/LogBrowse.cs:3079-3128`). The data grid and the map beside the chart are still owed |
| 2 | Waypoint editing on the map | a mission planner that cannot drag a waypoint is not a mission planner. Displaced twice already | D11 | a waypoint drags to a new position, a click adds one, and the change survives an upload and a read-back | done |
| 3 | Terrain-relative altitudes | a mission flown at 50 m over a hill is a mission into a hill | D11 | `MAV_FRAME_GLOBAL_TERRAIN_ALT` round-trips, and the planner says which frame every item uses | done |
| 4 | Fence and rally read-back | upload works and read-back does not, so a fence cannot be checked against what the vehicle actually holds | D11 | a fence and a rally set download and compare against the file that produced them | done |
| 5 | Satellite imagery | planning over a paddock needs imagery, not a street map | D8 | a second provider is selectable and its attribution is shown | done |
| 6 | Mission Planner tile cache | D8 asks for it, and operators carry multi-GB offline caches into the field | D8 | tiles written by the C# application are read with the network off | done |
| 7 | Stick-to-wire under 5 ms | D15 sets p99 ≤5 ms and the joystick path polls at 50 ms, missing it by an order of magnitude. It also runs on gpui's foreground executor, so a slow frame suspends the thing flying the aircraft | D15 | a dedicated thread blocks on the device and sends on change; a histogram over a real device shows p99 ≤5 ms | done on a fake device: an isolated movement reaches the link at p99 0.152 ms; a stick stirred at 1 kHz is capped at 50 frames/s by a 20 ms floor and is at most 20 ms stale (p99 19.9 ms). The real-device histogram is owed - no joystick on this machine; `cargo test -p mp-input --test real_device -- --ignored --nocapture` runs it, and because a hand on a stick is a continuous stream its bound is floor + 5 ms, not 5 |
| 8 | Zero-allocation proof on the ingest path | D2's DoD says zero heap allocations per packet "verified by an allocation-counting test". No such test exists, so the claim is untested | D2 | a counting global allocator asserts zero allocations across a replayed tlog's ingest→state path | done |
| 9 | Split `mp-link` | §5.1's layering is the pivot insurance and `mp-link` currently violates it: 6,728 LOC carrying params, missions, calibration, log download and `.param` files. A CI rule cannot enforce a graph the code does not have | D1 | `mp-params`, `mp-calibration` and `mp-ftp` exist; `xtask/tests/graph.rs` asserts the layer rules and passes | done: `mp-params` (with the parameter metadata, from `mp-vehicle`), `mp-calibration` (with radio calibration's `RcRange`, from `mp-vehicle`) and `mp-ftp` (log download; no MAVFTP code exists yet) exist, and `graph.rs` holds §5.1's four rules plus no upward edge and no cycle, each rule proven able to fail. The mission transfer machine stays in `mp-link`: the C# has it in `MAVLinkInterface`, and in `mp-mission` it would pull `mp-vehicle` and the dialect into `mp-kml`. The upward edge the plan did not name was **recording**: the link wrote `.tlog`s through `mp-log`, which is L4, so the writer moved into the link, where `MAVLinkInterface.SaveToTlog` has it too (`MAVLinkInterface.cs:1467`). Rule 4 is not yet true: `mp-gui` names gpui's platform crates, pinned as the one exception until `mp-render` exists |
| 10 | The porting ledger | G1 is "3,678 files in a terminal state" and there is no ledger to hold them. Nothing above can be called *done* in the sense this plan defines | D18 | `ledger/ledger.csv` has a row per `.cs` file and `cargo xtask ledger check` exits 0 | done: 3,678 rows, every one `ready`; `check` and `status` exist, with 22 tests and a CI step. Still empty: `target_crate`, `unit_id`, `deps` and the class columns; `ExtLibs/mono` is still an unfetched submodule |

**1 to 6 are what a pilot notices.** Everything in §13.1 made the application more trustworthy;
these make it more capable. 2 is the one that is embarrassing to still owe.

**7 and 8 are numbers this document claims and has not met.** They are listed with the rest rather
than in a corner, because a performance target nobody is scheduled to meet is a wish.

**9 and 10 are the factory.** They buy nothing an operator can see and everything the remaining
1.2M LOC depends on: without the graph, a framework pivot costs the project instead of two crates,
and without the ledger there is no definition of finished.

**What 6, 7 and 8 found**, in the manner of §13.1's table, because the pattern held: one small
item, three defects in work believed finished.

| Item | What it was supposed to add | What it actually found |
|---|---|---|
| 10 | a ledger | `cargo xtask` was not a command in this repository - no `.cargo/config.toml` alias existed, and CI called `cargo run -p xtask --` instead - so every `cargo xtask …` in this document was aspirational. Fixed. The tier counts in §4 came from a scratch script with no evidence rules; the classifier in `xtask/src/ledger/classify.rs` carries a reason per vendored root and moves 215 files between tiers as a result (the table under §4 has both sets). §4.2's rows do not sum to its T1 total, its DroneCAN row miscounts (`out/include` 205 + `out/src` 176, not 381 in `out/src`), it lists Android resources as `regenerate` while §4 drops them, and §4.4's buckets overlapped at 600. §6.4's behaviour-difference entry and §7.2's tolerance class have no column in §6.2 |
| 7 | a faster send path | `mp_input::reader`: a thread blocks in `read(2)` on the device, a second hands each change to the link, bursts coalesce to the newest position, 0.109 ms p99 on a fake device against 5 ms. Three things about the ported behaviour: **Mission Planner on Linux never notices an unplug** — `IsJoystickValid` is `fs != null` (`JoystickLinux.cs:263`), so it keeps sending the last stick position after the cable is out; ours releases, deliberately stricter. **The 200 ms stick timeout could never fire** — the old GUI loop fed the failsafe and checked it with the same timestamp, so the only real backstop was the vehicle's own RC override timeout; kept as it was, now named. And **sending on every change would flood a radio**: the C# sends at most every 50 ms and in practice every ~80 ms (`MainV2.cs:2249, 2356, 2446`), a gamepad reports at up to 1 kHz, and `mp-link`'s outbound queue has no back-pressure. So the reader has a 20 ms floor between sends (§8.2 measures at 50 Hz) under Mission Planner's 50 ms ceiling; the per-link budget §8.1 describes (a token bucket) is still not built |
| 8 | a test for a claim | The claim was true: 211,638 frame decodes and 70,546 frames through the real link thread allocate nothing per packet, in debug and release alike (`crates/mp-mavlink/tests/no_alloc.rs`, `crates/mp-vehicle/tests/no_alloc_ingest.rs`, `crates/mp-link/tests/no_alloc_ingest.rs`). Three message types allocate by design and are named in the test with a bound each: `PARAM_VALUE` (stored by name, as `MAVLinkInterface.cs:5770`), `STATUSTEXT` and `COMMAND_ACK` (the operator's message log). What was *not* true: `Transport::description()` is called on every snapshot publish and returns a fresh `String` - two allocations per publish, a hundred a second at 50 Hz, on a path the plan calls allocation-free; owed, and it needs the `Transport` trait to change. Also: §8.1 describes decoding only subscribed msgids, a `slotmap`/`dashmap` registry, a 120 Hz publish cap and `memchr` resync; the link decodes every frame, uses a `BTreeMap`, publishes at a fixed 50 Hz and resyncs a byte at a time. None of it allocates, and the numbers are met without it - so §8.1 describes a design, not the code |
| 6 | read the C# cache | The cache layout was invented — `<id>/<z>/<x>/<y>.<ext>`, "the one every slippy-map tool uses" — with no C# behind it. `ExtLibs/Maps/MyImageCache.cs:72-74` writes `gmapcache/TileDBv3/en/<Name>/<z>/<y>/<x>.jpg`: **always `.jpg` whatever the bytes**, y before x, the provider's C# `Name`, and it reads nothing else. Replaced wholesale, proved against a tile the real application wrote on this machine (`crates/mp-tiles/tests/tilecache.rs`) |
| | | **The offline store never read the disk.** `TileStore::offline` started no worker thread, and the worker is what reads the cache; every offline test passed because every offline test pre-loaded the tile by hand. With `MP_OFFLINE` the map drew a graticule over a full cache. Now it runs the worker with a policy that never fetches, and a test asks for a tile the way the map does |
| | | **Recordings were going where the C# never looks.** `Settings.GetUserDataDirectory` asks .NET for `MyDocuments`, and under mono that is `$HOME` — mono printed `/home/buzz` when asked — so a Linux installation lives in `~/.local/share/Mission Planner/`, where this machine's real one keeps its logs, parameter metadata and cache. Item 13 recorded to `~/Documents/Mission Planner/logs`, reconstructed from the enum's name. `mp-settings` now ports the rules; both the cache and the recordings use them |

---

### 13.3 The ten after

Nine of §13.2 are done and one is in flight, so the next ten are drawn, as the rule says, from
DELIVERABLES.md in P0 → P1 → P2 order, flying and planning first. Each row says what *done* means.

| # | Item | Why it is next | Deliverable | Done when | Status |
|---:|---|---|---|---|---|
| 1 | Split `mp-link` | carried from §13.2 item 9: the L0–L12 layering is the pivot insurance and the code does not have it | D1 | `mp-params`, `mp-calibration` and `mp-ftp` exist, `mp-link` is the link engine only, and `xtask/tests/graph.rs` asserts the §5.1 layer rules over `cargo metadata` and passes | done - see §13.2 item 9: `mp-link` 2,903 lines from 5,213, 20 graph tests, 767 tests on the branch and 863 after the merge |
| 2 | LogBrowse's data grid and the map beside the chart | the two halves of `Log/LogBrowse.cs` still missing, and the "Graph Left/Right" buttons act on the grid's selected cell | D14 | the grid shows the log's records with a selectable cell, `BUT_Graphit`/`BUT_Graphit_R` graph that cell (`graphit_clickprocess`), and the flown path from the log is drawn on a map beside the chart | done: `dataGridView1` as a virtual grid over a 9-byte-per-record index (16 rows decoded at a time, the type filter of `ColumnHeaderMouseClick`, headers following the current row as `RowEnter` does), Graph Left/Right/Clear Graph in the C# order with `graphit_clickprocess`'s six refusals in its words, and `myGMAP1` beside `zg1` with the first GPS route and the logged mission. Not yet: the chart cursor and its map marker, the strip's check boxes, GPS2/POS routes. The fixture log has no GPS fix, so the map assertion runs on `dataflash_damaged.bin` |
| 3 | Survey grid differential against the C# | D11's DoD is byte-identical missions and numerically verified grids, and nothing compares a grid to `Grid.CreateGrid` yet | D11 | a test runs the mono oracle's `grid` verb over ≥ 30 real polygons and holds §7.2 class C: identical point count, then order, then ≤ 1e-7°, plus the invariants where tie-breaks differ | done, and better than asked: `Grid.CreateGrid` runs headless under mono (`tools/csharp-reference/MpGrid.cs`, `regen-grid.sh`, no stubs needed) and **all 180 cases over 40 polygons match bit for bit** (`crates/mp-mission/tests/grid_vectors.rs`, goldens in `testdata/grid/`); the tie-break allow-list is empty. The old Rust grid was not a port - a flat tangent plane instead of UTM, bearing 0 as east-west, lanes from the wrong edge, overshoot at both ends - and was replaced by a transliteration of `Grid.cs` with ProjNet's transverse Mercator in `utm.rs`. Corridor, rotary and Gridv2 are not ported |
| 4 | HUD instrument parity | flying is what the application is for, and `HUD.cs doPaint()` (1954–3333) has elements this HUD does not draw | D9 | every element of `doPaint()` is listed in a coverage test with its status, and the missing ones a pilot uses first - pitch ladder clipped to ±40°, heading tape with the target bug, xtrack bar, VSI, dual battery, GPS fix/HDOP, armed banner - are drawn and asserted through facts | done for 18 of `doPaint()`'s 24 elements (`crates/mp-gui/src/hud.rs` `ELEMENTS`, held to the code by a test): the ladder, heading tape and bugs, xtrack and rate of turn, both scrollers, VSI, mode and waypoint, link info, battery, GPS, ARMED/DISARMED/SAFE, FAILSAFE, the message line. Missing, listed: flight-path vector, AOA, custom items, the vibe/EKF/pre-arm indicators. The C# shows the GPS fix only, not HDOP, so neither does this |
| 5 | Protocol state machines under fault | D4 says every state machine is an explicitly tested machine, and the retry loops are only exercised against a well-behaved SITL | D4 | `tests/retries.rs` injects timeouts, out-of-order `PARAM_VALUE`, every `MAV_MISSION_RESULT` and partial transfers and asserts convergence with the C# retry counts (`setParamAsync` 3, `GetParam` 3, `setWPCurrent` 5, `doCommand` 3); `tests/routing.rs` drives 50 vehicles through one link | done: `crates/mp-link/src/timeouts.rs` holds every C# count and wait in one table (`setParamAsync` 3×700 ms, `GetParamAsync` 3×700, `doCommandAsync` 3×2 s with arming at 10 s and calibration at 1×25 s, `setWPCurrent` 5×2 s, `getWPCount` 6×700, `getWP` 5×2.5 s, `setWPTotal` 3×700, `setWP` 10×450, `getParamList` 2 whole-list retries under 75 % with 10 reads a round; `MAVLinkInterface.cs:1748-4380`); `requests.rs` gives the link `set_param`, `read_param`, `command` and `set_current_waypoint` with those retries, `param_download.rs` replaces the ad-hoc loop, and `mission_transfer.rs` handles every `MAV_MISSION_RESULT` as `mav_mission.cs:101-151` does, in its words. `tests/retries.rs` (40 tests, 2,152 lines) counts the sends on the wire for each scenario and seeded bad links (15 % drop, delay, duplicate); `tests/routing.rs` runs 56 components on 50 systems through one link at 2.8 µs a frame with no bleed. **Found:** parameter sets and commands had no retries at all; the old gap recovery chased holes in any table, so every parameter echo outside a download started ten reads every 1.5 s forever, and a hole never answered blocked the rest. Nine deliberate divergences are recorded at the tests (`DIVERGENCE:`), each where the C# resends the wrong item, ignores a result or doubles traffic on duplicates. **Owed:** the GUI still sends sets and commands raw through `link.send` and gets none of this until it moves to the new calls (§13.4 row 11) |
| 6 | Transport faults and enumeration | D3's DoD; a surprise unplug is the field failure nobody rehearses | D3 | `tests/faults.rs` (drop, duplicate, reorder, partial write, mid-frame disconnect) and `tests/enumerate.rs` over checked-in per-OS device fixtures pass | done: `crates/mp-transport/tests/faults.rs` runs 300-500 real frames through the mock under every fault (dropped and duplicated bytes, repeated and reordered frames, a frame split at every byte, a disconnect mid-frame, all at once) and re-checks every delivered checksum itself; `tests/hotplug.rs` pulls a real `SerialTransport` over a pty behind a by-id-style link and reopens it after the "replug"; `tests/enumerate.rs` holds `CommsSerialPort.GetPortNames` (`crates/mp-transport/src/enumerate.rs`, the seven unix globs, mono's `ttyS*` list, the Bluetooth `COM` fix, trim-then-dedup, directory order and no sort) to Linux, macOS and Windows fixtures. **Found:** the mock returned `Ok(0)` forever after an unplug where the `serialport` crate reports `BrokenPipe`; the list is now the C#'s, `/dev/tty`, `usbmon*` and `ttyS0-31` included, because Mission Planner under mono lists them. Not ported: WMI friendly names on Windows, BLE/WinUSB/Xamarin port sources |
| 7 | FlightData action coverage | D10's DoD is every tab, button and action of `GCSViews/FlightData`, and nobody has counted what is missing | D10 | `tests/action_coverage.rs` enumerates the controls and actions from `FlightData.resx`/`Designer.cs` and prints the unimplemented list; the list is the deliverable, and it shrinks in later commits | done: `crates/mp-gui/src/coverage.rs` lists all 136 event wirings of `FlightData.Designer.cs` - **27 done** on the flight screen, 2 elsewhere, **87 missing**, 19 plumbing, 1 dropped - rendered to `docs/coverage/flightdata.md`. Tests parse the Designer when the C# tree is present, check every claimed id against the source, and fail when the report is stale |
| 8 | Projection proof and the pan/zoom budget | D8 asks for < 1 mm against ProjNet and 120 fps with a 1 M-point track; both are claimed, neither is gated | D8 | `tests/projection.rs` round-trips a fixture grid against oracle values at < 1 mm; `benches/pan_zoom.rs` exists and its number is recorded | done, against the C# itself rather than ProjNet's documentation: `tools/csharp-reference/MpProjection.cs` runs GMap's `MercatorProjection`, `PointLatLngAlt.GetDistance/GetBearing/newpos` and `utmpos` under mono over 676 points (`testdata/projection/`), and `crates/mp-units/tests/projection.rs` holds the port to them - the pixel identical at zooms 1-30 (3,380 readouts), the inverse, distance, bearing and `newpos` **bit for bit**, our own round trip < 1e-8° and < 1 mm (worst 4.3e-14°, 5.8 nm); `mp_mission::utm` matches all 676 `utmpos` goldens bit for bit too. **Found and fixed:** `distance_to` was on the wrong radius (up to 27.6 m out), `bearing_to` disagreed with the C# on degenerate pairs (identical points 180 vs 0), `offset` travelled on 6,371,008.8 m where `newpos` uses 6,378,100 m (up to 22 km on long legs), and the Mercator inverse differed in the last bits. `crates/mp-units/benches/pan_zoom.rs` gates the following frame at p99 < 8.33 ms in release: p50 2.63 ms, p99 5.42 ms on a 1 M-point track, 10 k markers and 169 tiles, of which the auto-fit scan over every track point is nearly all - the first thing to cache. GMap's own round trip is 7.8 cm at zoom 20 because it returns whole pixels, which is why the map keeps `f64` positions |
| 9 | Parameter metadata for the firmware actually flying | §10.5 measured 41% of a 4.6 vehicle's parameters undocumented by the bundled file; the C# fetches per version | D12 | `AUTOPILOT_VERSION` resolves to a release, its `apm.pdef.xml` is fetched and cached, and the undocumented count on this SITL is recorded before and after | done, by the C#'s route rather than the row's: Mission Planner takes the version from the `STATUSTEXT` banner it asks for with `DO_SEND_BANNER`, not from `AUTOPILOT_VERSION` (`MAVLinkInterface.cs:930, 1822-1830`), and so does `mp_params::pdef` - the versioned `Copter<x.y.z>.apm.pdef.xml` for a release, else the vehicle's weekly-refreshed unversioned file, in the C#'s directory with the C#'s names. **Measured on this SITL: the bundled table documents 827 of 1,408 parameters; the fetched file documents 1,407** (`crates/mp-params/src/pdef.rs`, the test prints both). The GUI reads the fetched file first and the bundle second, and publishes `params.metadata.*` facts |
| 10 | Read Mission Planner's `config.xml` | D17: both applications against one data directory means the shared settings too | D17 | our settings read the keys both applications use (last link, map type, log directory) from a `config.xml` the real application wrote, without conversion, with that file as the fixture | done: `mp_settings::Config` reads and writes the C#'s `config.xml` (`Settings.Load`/`Save`) and renders it back byte for byte - proved on this machine's real file, 83 keys, BOM, case-insensitive key order and `____` for `/` included. The GUI takes the recording directory from `logdirectory`, and the last link (`comport`, `<comport>_BAUD`, `TCP_*`/`UDP_*`) and map type from it when its own settings say nothing |

### 13.4 The ten after that

§13.3 is done bar item 5, which is in flight, so the queue is refilled again from DELIVERABLES.md
P0 → P1 → P2, flying first, then planning and missions, then setup. Every row names a C# file it
is ported from; nothing is drawn from anywhere else.

| # | Item | Why it is next | Deliverable | Done when | Status |
|---:|---|---|---|---|---|
| 1 | The flight screen's actions | 87 of `FlightData`'s 136 wirings are missing (§13.3 item 7), and the ones a pilot reaches for are among them: Set WP, Change Alt/Speed/Loiter Radius, Restart/Resume Mission, Set Home Here, Set EKF Origin Here, Fly To Coords, Fly To Here Alt, Abort Landing, Do Action, Clear Track | D10 | each ported from its `FlightData.cs` handler with the C#'s message and values, in the Actions tab's own arrangement (`FlightData.resx`), its coverage row flipped to done, and a `tests/gui/*.gui` script asserting it against SITL through facts | done for 12 of the 15: Set WP with the C#'s waypoint list (`MISSION_SET_CURRENT`), Restart Mission, Change Alt (a `current`=3 `MISSION_ITEM`), Change Speed (`DO_CHANGE_SPEED`, no `multiplierspeed`, as the C#), Set Loiter Rad (`PARAM_SET` to `LOITER_RAD`/`WP_LOITER_RAD`), Resume Mission (the C#'s whole sequence, one step a frame, its attempt limits), Fly To Coords and Fly To Here Alt (`SET_POSITION_TARGET_GLOBAL_INT` mask `0xFDF8`, the plane path), Abort Landing (`DO_GO_AROUND`), Set Home Alt, Do Action with the C#'s 19-entry list and its "Are you sure" prompts - in the Actions tab's 5×5 grid from `FlightData.resx:1371`. The lower-left is now the C#'s `tabControlactions` (`FlightData.Designer.cs:595-608`): fourteen pages in the Designer's order with the `.resx` texts, Quick selected at start, one row of headers with arrows as multiline is off (`FlightData.cs:429`), the actions panel and grid on Actions where `BUT_ARM` and `CMB_modes` live, the tuning graph above the map where `CB_tuning` puts it; only one page shows, which is what keeps the column inside a 1200 px window - the grid had pushed it 417 px past the bottom, `layout.rs` caught it, and `fly.page.overflow` is 0 on every page. The questions are modal dialogs as the C#'s are. Coverage **40 done, 75 missing**. Twelve `tests/gui/fly-*.gui` scripts, `fly-resumemis` flying SITL. **Found:** the runner's typed text can be overtaken by its next click when ibus sits between xdotool and gpui (keys round-trip through the input method, clicks do not), so scripts settle after typing and check `fly.prompt.value`. Not ported: Set Home Here and Set EKF Origin Here (need `srtm.getAltitude`), Clear Track (one line in `mapview.rs`, owed). Each send is once, not retried (§13.4 row 11). **Found:** `sensors.rs` read the motor-outputs bit as `0x4000` where `MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS` is `0x8000` (the HUD's safety indicator; fixed with §13.4 item 4), and Force Arm sent the force-disarm magic (fixed with item 2's commit) |
| 2 | The planner's actions, counted and then done | D11's DoD is every action of `FlightPlanner`, and nobody has counted: `FlightPlanner.Designer.cs` wires 121 events | D11 | a coverage table like §13.3 item 7's (`planner_coverage.rs`, `docs/coverage/flightplanner.md`, held to the Designer by a test), then the map's right-click items in the C#'s order - insert/delete WP, loiter, jump, RTL/land/takeoff, ROI, clear, reverse, zoom to, the polygon items - each with a `.gui` script | done: `crates/mp-gui/src/planner_coverage.rs` lists all 121 wirings of `FlightPlanner.Designer.cs` with the `.resx` text - **40 done, 69 missing, 12 plumbing** - held to the Designer, the `.resx` and `contextMenuStrip1.Items.AddRange`'s order by tests, rendered to `docs/coverage/flightplanner.md`, counts published as `coverage.flightplanner.*`. The map's right-click menu is the C#'s, in its order, with 22 entries ported from their `FlightPlanner.cs` handlers (Delete WP, Insert WP/Spline, At Current Position, Loiter forever/time/circles, Jump start/WP, RTL, Land, Takeoff, DO_SET_ROI, Clear Mission, Draw/Clear Polygon, From Current Waypoints, Measure Distance, Reverse WPs, Modify Alt, Load/Save WP File), each with the C#'s prompt text and offered value, the rest present but dimmed. Mouse buttons are now the C#'s: left places, right opens the menu. **Found:** `is_navigation` was a range guess (16-95, 5000-5006, 5100); it is now the 45 commands `Mavlink.cs` marks `[hasLocation]`, checked against that file - `DO_SET_ROI`'s coordinates went to the vehicle unscaled before. Missing, biggest groups: the planning panel's boxes (WP radius, default alt, loiter radius, home fields), Map Tool, polygon icon menu, Geo-Fence, Rally, Auto WP. Not ported: zoom-to (the map has no API for it), Set Home Here (needs the home boxes and terrain), Load and Append, the old fence/rally messages, shapefiles/KML/POI/UTM/GDAL |
| 3 | The HUD's last six | §13.3 item 4 left flight-path vector, AOA, custom items and the vibe/EKF/pre-arm indicators listed as missing | D9 | `hud::missing()` is empty, or every remaining row says why the C# element cannot be drawn without a feature we do not have | done: all six ported from `doPaint()` (`ExtLibs/Controls/HUD.cs:2232-2245, 2804-2843, 3029-3077, 3148-3301`) with the C#'s positions, thresholds (Vibe red > 60 / orange > 30, EKF red > 0.8 / orange > 0.5) and texts, `ekfstatus` computed as `CurrentState.cs:2649-2741` does, pre-arm from the `PREARM_CHECK` bit; ten unit tests and `tests/gui/hud-health.gui`. Four draw live; the flight-path vector and AOA scale draw when given values but `mp_vehicle` had no `AOA_SSA`/`AOA_CRIT` - item 4 has since added `AOA_SSA` and `SYS_STATUS.load`, so wiring those two is what remains (`Status::Blocked` names them). The custom-items list has no editor (the C#'s `hud1_useritem_` settings are not ported) |
| 4 | `CurrentState` field coverage | D5's DoD: every C# `CurrentState` field accounted for, with a checked-in report; `Transport::description()` allocation still owed | D5 | a test enumerates `CurrentState.cs`'s properties and prints which `mp_vehicle` field stands for each; the two `description()` allocations are gone from the publish path | done, both halves. `Transport::description()` returns `&str` now, each transport keeping its text and rewriting it only when what it describes changes (serial on `set_baud`, UDP when the peer changes, in a buffer sized at bind), so the publish path allocates nothing: 71,500 description allocations over 35,750 publishes before, 0 after, and both `no_alloc_ingest` tests now demand zero with only `PARAM_VALUE`, `COMMAND_ACK` and `STATUSTEXT` allowed; `tests/description.rs` pins every transport's text unchanged. For the coverage: `crates/mp-vehicle/src/coverage.rs` has one row per public member of `CurrentState` - **550 rows: 416 done, 48 derived, 55 missing, 30 plumbing, 1 dropped** (219 done before the porting) - matched against the C# file by name, type, display text, group and order, rendered to `docs/coverage/currentstate.md`. Ported on the way: SYS_STATUS's load, errors and safety, the C#'s invalid-position and unknown-HDOP rules, GPS velocities and accuracy, GPS2, all nine batteries with the C#'s filter, RADIO to every vehicle, WIND, TERRAIN, RANGEFINDER, SERVO_OUTPUT_RAW, HIGH_LATENCY/2, and `onboard.rs` (IMUs, ESCs, PID tuning, mount and gimbal, EKF status, AOA_SSA, RPM, ...), plus `units.rs` (`MainV2.ChangeUnits`). 34 constructed-message tests and a replay over both tlogs. **Found:** the safety indicator read `0x4000` (XY position control) for `MOTOR_OUTPUTS` (`0x8000`); now taken from the dialect. Missing: the NAMED_VALUE_FLOAT slots, HIL, the packet-clock fields (time in air, distance travelled) which need a clock from `mp-link`. Not done: `Transport::description()` (mp-link) |
| 5 | Mission Planner's map providers | D8: the provider list is not the C#'s - two of three are not Mission Planner's and its default `GoogleSatelliteMap` is absent | D8 | the C#'s providers from `GMapProviders`/`MyImageCache`, the default the C# defaults to, cache names the C#'s so both applications read one cache; the non-C# providers stay only if the owner says so | done for the default and six providers: `GoogleSatelliteMap` is the default as `FlightPlanner.cs:7282` has it, with `GoogleMap`, `GoogleTerrainMap`, `BingMap`, `BingSatelliteMap`, `BingHybridMap` and `OpenStreetMap` in the C#'s order and names (`GMapProviders.cs`, `Strings.resx`), the Google URL scheme with `GetServerNum`, the `Galileo` secret and the encrypted host decrypted by `Stuff.cs`'s method, `TryCorrectVersion` with the C#'s regexes and its `UrlCache` (`gmap.rs`, `versions.rs`, `urlcache.rs`), Bing's quadkey URL and `tilegeneration` check, the C#'s `Referer` on every request. 84 URLs and the list compared against the shipped `GMap.NET.Core.dll` run under mono (`tests/fixtures/GMapOracle.cs`); a real GoogleSatelliteMap tile from this machine's cache read back; a localhost proxy proves the version check precedes the first Bing fetch. Not ported: the Bing session key (another project's key), two-layer hybrids, GCJ-02, WMS, keyed providers, `NoMap`; 60 of the C#'s 66 remain, most needing nothing new. OpenTopoMap and Esri stay, last, pending the owner's word |
| 6 | The telemetry storm on the UI | D10: no UI stall > 8 ms during a 200 Hz telemetry storm is claimed for the link, never for the frame | D10 | a bench or probe run pumps 200 Hz through the real link into the GUI and records the frame's p99 | measured: `MP_STORM=200` puts `mp_link::testing::Storm` on the loopback - ATTITUDE, GLOBAL_POSITION_INT, VFR_HUD and HEARTBEAT 200 times a second each through the real link thread and snapshot bus, a circle flown so every readout changes - and the frame is timed from the top of render to the last paint, less the harness's own facts and probe writes (`crates/mp-gui/src/storm.rs`, facts `frame.p50/p99/max/stalls`, `storm.rate`); `tests/gui/storm.gui` asserts the budget. **Debug build, 2026-09-24: p99 26 ms, 127 stalls in 480 frames at 200 Hz. Release build (`MP_GUI_BIN=target/release/mpr-gui`), same day: p99 15 ms, 38 stalls in 550 frames - over the 8 ms budget**, measured while a debug build ran on other cores, to be repeated quiet; either way D10's number is not met yet, and the per-frame costs below are the work. Fixed on the way: the map's auto-fit scanned every track point each frame (PLAN §13.3 row 8's finding) and now keeps the track's rectangle as it grows, bit-identical to the scan, 2.88 ms → 28 ns over a million points in release. Found, being fixed under row 24: `Telemetry::view()` copies the whole parameter table every frame (1.0 ms with 1,408 parameters, debug) and the parameter screen's documentation fallback scans the table for any name with a digit (71-82 ms a frame, debug). `telemetry_storm.rs` now takes the fastest of three loads per sample, so a descheduled thread cannot fail it. Noted meanwhile: `crates/mp-link/tests/telemetry_storm.rs` asserts the worst single snapshot load against a frame budget, and one such sample failed with the machine at load 30 (six agents building) and passed alone a minute later - a descheduled thread, not the code. When this row is done, that test should measure the load path in a way a preemption cannot fail |
| 7 | LogBrowse's chart cursor and its map marker | owed since §13.2: the C# moves a marker on the map as the cursor crosses the chart | D14 | `zg1_MouseMove`'s behaviour, with the strip's check boxes and the GPS2/POS routes | done, and the row's premise was wrong: `zg1_MouseMoveEvent` (`LogBrowse.cs:3574-3586`) only debounces, and ZedGraph's point readout is off by default and never turned on, so the C# does nothing on hover. Its chart action is a **double-click** (3278-3297, 3464-3523): a dashed cursor line at the pointer's line, or on a time axis the first GPS/GPS2/POS record at or after that time (`DFLog.GetLineNoFromTime`), the pink pin on the map at `GetGPSFromRow`'s record (3329-3420, the 1000-line jump kept), the map centred on it, and the grid at that row once Data Table has shown. The strip's check boxes in the `.resx` order - Map, Time, Data Table, Mode, Errors, MSG, Events, with Map and Data Table off at start - each drawing what the C# draws: mode and MSG labels along the bottom, errors and events in red from the top, minute marks, the 71-colour mode bands. One deliberate divergence, written at the site: the C# draws the time-axis cursor at x = line number, which lands in 1900 off the chart (its own `//TODO - time fails`); ours draws it at the record's time. `mp_log::overlay` supplies positions with line numbers; the series now share one time origin, which had misaligned curves. `tests/gui/log-cursor.gui` and a `doubleclick` directive in `gui-test.sh`. Not ported: the point-value tooltip (a context-menu option, off by default), the strip's Show Params and preselect, remembering the boxes between sessions |
| 8 | Corridor and rotary grids | D11: `Grid.cs:46-310` is not ported, only `CreateGrid` | D11 | both transliterated and held to the mono oracle as `CreateGrid` is | done, bit for bit: `CreateCorridor` (`Grid.cs:55-188`) and `CreateRotary` (`:196-310`) transliterated into `corridor.rs` and `rotary.rs`, with `clipper.rs` - the C#'s Clipper 6.4.2 offset and union, and mono's introsort, because the rotary's insets are what the library returns and nothing else matches in the last bit - held to the mono oracle over **41 corridor cases (14,442 points), 63 rotary cases and 25 raw offset cases**, every double identical, allow-lists empty (`tests/corridor_vectors.rs`, `tests/rotary_vectors.rs`; `MpGrid.cs` gained `corridor`, `rotary` and `offset` verbs, `regen-grid.sh` reproduces all 309 goldens byte for byte). The C#'s oddities kept and named: a two-point corridor gives nothing, parallel legs fly to `utmpos.Zero` on the equator, triggers at whole metres. Not reached by any golden: the introsort's heapsort fallback and three Clipper branches, named in the file. No `mpr survey` variant (the CLI has no tests to extend); the API is `create_corridor`/`create_rotary` |
| 9 | Board detection | D13: `DetectBoardTest` cases exist in `MissionPlannerTests` and nothing runs them | D13 | `BoardDetect.cs` ported and the test cases pass over USB descriptor fixtures | done, with a finding about the C# tests: `crates/mp-firmware/src/detect.rs` is `Utilities/BoardDetect.cs:18-673` - the 29-board enum, the four new-style bootloader ids and the old-style names first match wins, the 19 WMI rules in the C#'s if/else order, the bootloader board-id path (flash 2,080,768 + rev ≥ 5 + board 9 = `px4v3`), the STK500v1/v2 probes, and the yes/no questions that are all the C# does under mono - behind a `DetectHost` trait so the probes run against the px4 mock and a pty. **All 16 `DetectBoardTest` calls are in `testdata/boards/detect_board_tests.json`, and five of them fail against `BoardDetect.cs` itself** (Test3/4/6/8 hit the replug prompt and can never yield the `chbootloader` they expect; Test5 expects `chbootloader` where the code returns `fmuv5`); five read the test machine's live WMI table; the fixture records what each asserts beside what the code returns, and the test pins that split. 31 device-list and 30 WMI cases built from the C#'s own constants cover every id. `mpr firmware detect <port>` reads the USB ids and opens nothing; this machine's MR-VMU-RT1176 (27B1:0004) is not in the C# table and says so. No board was touched |
| 10 | The `frame_parse` 24 h soak | D19, owed since §13.1: the fuzz target that proved itself empty has never been run long since it was fixed | D19 | a 24 h run recorded in `fuzz/README.md` with its edge count, clean or with the crash fixed | running since 2026-09-24 02:03 UTC, `frame_parse` and `message_decode` both, one core each |
| 11 | The GUI's sets and commands through the retrying requests | §13.3 item 5 gave `mp-link` the C#'s retries, and the screens still call `link.send` raw, so a lost `PARAM_SET` or `COMMAND_LONG` is lost | D4, D10, D12 | every `link.send` of a `ParamSet`, `CommandLong` or `MissionSetCurrent` in `mp-gui` and `mp-cli` goes through `Link::set_param`/`command`/`set_current_waypoint`, and a `.gui` script proves a set survives a dropped first send on the mock | done: every set and command the C# waits on is a request - Set WP, Restart, Change Speed, Set Loiter Rad, Abort Landing, Do Action with the `DIGICAM_CONTROL` fallback the C# sends on a refused trigger (`MAVLinkInterface.cs:4557-4569`), Resume Mission step by step, arm/disarm/takeoff, force arm, every parameter write through a `ParamWrites` queue one at a time as the C# writes them (unlisted names read first), the calibration sends, `mpr param set/load` and `mpr fly` - each ending with the C#'s message-box text on the status line ("Error: The Command failed to execute", "Set X Failed", "N parameters successfully saved."...). Left raw because the C# does not wait: accel start and position, reboot, mode changes and the safety switch (`:4631-4641`), guided targets, the banner, RC override. Still raw though the C# waits: Format SD and the scripting commands (`doCommandInt`), Change Alt and the plane guided target (`setWP`), which `mp-link` has no request for yet. Proved on the loopback with a scripted vehicle dropping the first send (mp-gui 329 tests, mp-cli 6), `tests/gui/params-retry.gui` on SITL. **Found:** `Link::request` lost sight of a request for the length of the link thread's pick-up - drained from the queue, not yet in the table - and a caller reading `None` as "forgotten" abandoned a write the vehicle then accepted; the pick-up and the reader now hold the table's lock across the queue, with a test that polls from the moment of queueing. Also noted, unfixed: `mp_calibration::cancel_compass` sends param3=0 where the C# sends 0,0,1 (`ConfigHWCompass2.cs:345`); the reboot button sends once where the C# sends twice |
| 12 | Home is item 0 | found by §13.4 item 2: a mission drawn on an empty map has no home record, so its first waypoint becomes item 0, which the vehicle and `.waypoints` files treat as home; the C# always writes home first (`FlightPlanner.cs` `WPtoList`/`getWPs`, from `TXT_homelat/lng/alt` or the vehicle's `HOME_POSITION`) | D11 | a mission written from the planner has the home record at 0 as the C# writes it, `waypoint-click.gui` and `altitude-frame.gui` updated to say so, and a `.waypoints` round trip through the corpus still byte-identical | done, as the C# has it: home is not a grid row but the three Home Location boxes (`FlightPlanner.cs:7001-7050, 7195-7218`), filled from the vehicle's `HOME_POSITION`, else the planned home from `config.xml`; Write refuses "Your home location is invalid" without one (:658-671) and otherwise sends home at 0 as a GLOBAL-frame WAYPOINT (:6198-6227); Save writes record 0 or the blank home line (:6108-6160); reading keeps item 0 apart and asks "Reset Home to loaded coords" when it differs (:5640-5670). The first click on an empty map is waypoint 1. `write_planned` round-trips all 129 corpus files, the five Mission Planner wrote byte for byte. `tests/gui/plan-home.gui` plus seven scripts updated; coverage **44 done, 65 missing**. **Found on integration:** the planner adopted a vehicle mission as soon as the transfer's item list was non-empty, and since §13.3 item 5 that list grows item by item, so a five-item read came back as one row; the view now says whether the download is complete and the plan waits for it (`fly-setwp.gui` caught it). Left: the "H" home marker on the map (row 13), the home boxes saved back to `config.xml`, the vehicle's home altitude from `HOME_POSITION` (one line once the field is used), "read N items" still counting home |
| 13 | The home marker and Clear Track on the map | row 12 took home out of the mission, and the C# draws an "H" at the boxes' home (`FlightPlanner.cs`, the `home` marker on `MainMap`); `BUT_clear_track_Click` needs one line in `mapview.rs` | D8, D10, D11 | the map draws the C#'s home marker at the planner's home and the flight screen's home, and Clear Track clears the flown path, each proved by a script | done: the green GMap pin with "H" past zoom 16 (`WPOverlay.cs:44-50, 385-398`, `GMarkerGoogle.cs:111-127`), from the boxes on the planner and from `HOME_POSITION` on the flight screen once the mission is read (`FlightData.cs:3808-3845`); `clear_track()` on the map, its button owed to the flight batch. With it, the planning panel's boxes - WP Radius, Loiter Radius, Default Alt, the frame, Spline, a dimmed MAVFTP - at the head of the table with the C#'s typing rules (`:6984-7106`), taking the vehicle's values on show (`:6695-6750`), and the parameters set after Write one at a time as "Setting params" does (`:6296-6317`); and four Geo-Fence entries (Set Return Location, Load, Save, Clear with its three parameter writes, `:2112-2155, 4345-4412, 5959-6021, 6663-6670`). **Upload and Download stay dimmed:** the C# uploads with `FENCE_POINT` (`:3691-3900`), which ArduPilot removed in 4.8, so a faithful port cannot work against this SITL, and Download's mission-protocol branch needs the vehicle's capabilities, which nothing asks for yet; uploading through the mission protocol from this menu goes beyond the C# - owner's call. Coverage 55 done, 54 missing |
| 14 | The configuration panels counted | D12's DoD is a coverage ledger of every `Config*.cs` panel at 100 %, and nobody had counted | D12 | `config_coverage.rs` lists every panel in `GCSViews/ConfigurationView/` with the C#'s two menus' order and titles, held by tests, rendered to `docs/coverage/configuration.md` | done: **61 panels - 0 done, 7 partial, 48 missing, 2 plumbing, 4 dropped - 569 Designer wirings**, in the SETUP list's order (`InitialSetup.cs:162-351`, 47 calls) and the CONFIG list's (`SoftwareConfig.cs:156-257`, 20 calls), titles from `InitialSetup.resx`/`Strings.resx`, each row citing the call that adds it; the four pages outside `ConfigurationView/` (Sik Radio, Joystick, Antenna Tracker, MAVFtp) in a side table. Partial: Param Loading, Accel, both Compass, Radio, Motor Test, the Full Parameter List. Dropped because the C# never shows them: `ConfigTradHeli`, `ConfigHWCAN` (commented out at `InitialSetup.cs:186, 275`), `ConfigSecure`, `ConfigPlannerAdv`. Biggest missing: Extended Tuning (128 wirings), Planner (64), `ConfigArduplane` (47), RTK/GPS Inject, Install Firmware, Frame Type, DroneCAN, Battery Monitor |
| 15 | Flight Modes | the first D12 panel in the SETUP list's order after the calibrations (`InitialSetup.cs:226-229`) | D12 | `ConfigFlightModes.cs` ported - the six combos from the firmware's mode list, the PWM bands 1230/1360/1490/1620/1749, the lit current row, Simple/Super Simple bits, Save through the retrying set - with a script proving a save reaches SITL | done: `config/flight_modes.rs` - the mode list per firmware from the parameter documentation (`Common.cs:88-176`, plane's INITIALISING appended), the six combos loaded as `Activate` loads them (`ConfigFlightModes.cs:38-232`, stopping at the first missing value), the 100 ms readout with the channel from `FLTMODE_CH`/`MODE_CH` and `readSwitch`'s 1230/1360/1490/1620/1749 bands lighting the row (:250-352), Simple/Super Simple enabled by mode name (:436-467) and packed as an integer mask, Save writing each parameter in turn through the retrying `set_param` and stopping where the C# throws (:354-413), the `.resx` table at 110/110/110/140/88 px; 18 unit tests, `tests/gui/config-flightmodes.gui` changing FLTMODE2 and SIMPLE on SITL and restoring them. **Found on integration:** packing the switches by summing floats gave -0.0 for none, since that is `Sum for f64`'s identity, and the fact read "-0". Reached from a Mandatory Hardware panel on the setup screen rather than the C#'s list position, pending row 17 |
| 16 | FailSafe | the panel after Flight Modes in the same list | D12 | `ConfigFailSafe.cs` ported - the channel bars, the throttle/battery/GCS failsafe controls writing their parameters on change - with a script | done: `config/failsafe.rs` - every control at its `.resx` position in the 688×482 page, the eight Radio IN and Servo OUT bars, the mode label red when the mode changes with the throttle below `FS_THR_VALUE` (:173-192), each control writing its parameter as it changes as `MavlinkComboBox`/`MavlinkCheckBox`/`MavlinkNumericUpDown` do (on change, after 300 ms for numbers), the names chosen per firmware (`BATT_FS_LOW_ACT` else `FS_BATT_ENABLE`; `LOW_VOLT`/`FS_BATT_VOLTAGE`/`BATT_LOW_VOLT`; the plane set), the props warning on open (:91-92); 32 unit tests, `tests/gui/config-failsafe.gui` changing `FS_THR_ENABLE` on SITL and restoring it. Numbers step by arrows only (gpui has no numeric up-down). The two panels share one Mandatory Hardware strip in the C#'s order; the ledger counts **0 done, 9 partial, 46 missing** |
| 17 | The setup screen as `InitialSetup` | the C# shows one page at a time beside a list (`InitialSetup.cs`, `BackstageView`); ours stacks panels, and every new panel lands below the fold | D12, D13 | the setup screen is the C#'s list with the C#'s order and one page shown, the calibration panels and the config pages on it, proved by a script that selects each and reads `setup.page` | done: SETUP is the backstage view - `SETUP_LIST` holds all 47 `AddBackstageViewPage` calls of `InitialSetup.cs:162-351` and `CONFIG_LIST` the 20 of `SoftwareConfig.cs:156-257`, one line each with the C#'s conditions as code, checked as text against the `if`s around each call; headings open and close, an entry deactivates the old page and activates the new (`BackstageView.cs:428-523`), the last page is remembered per screen (`InitialSetup.cs:397-403`, `SoftwareConfig.cs:293-315`), the list rebuilds on link and vehicle changes; CONFIG is a tab beside SETUP as `MainV2`'s buttons are. Accel, compass, radio, motor test, joystick, Flight Modes and FailSafe are pages; Loading (`ConfigParamLoading`) and the two heading pages are ported; the rest say "not ported" under their C# title. `tests/gui/setup-list.gui` clicks every entry and heading; a layout test holds both screens inside 1600×1200. **Left, row 18:** four old panels have no C# page (logs, estimator, ground-pressure calibration, identity) and sit in the empty page area until moved to where the C# shows their content |
| 18 | The four homeless setup panels | row 17 found them: the C# shows logs on the flight screen's DataFlash Logs tab (`BUT_DFMavlink`), ground-pressure calibration on Actions (`Preflight_Calibration`), the estimator's content in the HUD's EKF and vibe windows (`HUD.cs:1207-1231`), and nothing for the identity panel | D10, D12 | each panel is on the page the C# has its content on, or gone, with the coverage rows saying so and the scripts that used them updated | |
| 19 | Frame Type | the second Mandatory Hardware entry (`InitialSetup.cs:189`), copter only | D12 | `ConfigFrameClassType.cs` ported - the class buttons and type rows from `Common.ValidList`, each click writing `FRAME_CLASS` then `FRAME_TYPE` - with a script changing the type on SITL | done: eight classes and six types with `ValidList`'s 29 pairs (`Common.cs:15-85`), `SetFrameParam`'s two writes in order (`:281-293`), the page disabled without both parameters, the frame pictures as named boxes since the C#'s PNGs are not shipped; 26 unit tests, `tests/gui/config-frametype.gui` switching Plus to X and back |
| 20 | Battery Monitor | the first Optional Hardware entry | D12 | `ConfigBatteryMonitoring.cs` ported - the Monitor/Sensor/HW Ver combos, the divider and amps-per-volt arithmetic, the presets - with a script changing capacity on SITL | done: `config/battery_monitor.rs` - `Activate`'s reading of the boxes newest name first (`ConfigBatteryMonitoring.cs:18-176`), the sensor recognised from the boxes' text against the C#'s constants, the board from the pins; the Monitor combo's pin writes and enable patterns (:205-266), the nine sensor presets in single precision (:356-462), the eleven boards' pin pairs (:483-563), the four boxes writing on leave with the measured-voltage and measured-current arithmetic (:274-354, 621-653), the once-a-second readouts, the speech alert questions for the session; 26 unit tests, `tests/gui/config-battery.gui` writing BATT_CAPACITY 3400 on SITL and restoring 3300. Not the photo, nor typing into the two combos. Ledger 1 done, 10 partial, 44 missing |
| 21 | The firmware catalogue | D13's firmware path without flashing: the manifest, the choice per board and vehicle, the page | D13 | `APFirmware.cs`'s manifest fetched, cached and parsed, the selection functions tested on a fixture, the Install Firmware page with Upload disabled, `mpr firmware list` | done, with one correction to the row: **the C# keeps no manifest cache file** - `APFirmware.GetList` holds the manifest in a static for the process (`APFirmware.cs:103-138`) and `MainV2.BGFirmwareCheck` warms it once a day recording `fw_check` in `config.xml` (`MainV2.cs:3923-3938`) - so none was invented; `MP_FIRMWARE_MANIFEST` names a file for tests. `crates/mp-firmware/src/manifest.rs`: `FirmwareInfo` one field for one, read as Newtonsoft reads it, the mirror first and ardupilot.org as the fallback (`ConfigFirmwareManifest.cs:67`, `APFirmware.cs:276`), CubePilot's peripheral manifest appended (`:131-182`), `GetRelease`/`GetBoardID`/`GetOptions`/`LookForPort` (`:186-298`; `ConfigFirmwareManifest.cs:193-245`) with `FirmwareSelection`'s platform pre-select; 18 tests on a 240-record excerpt of the real manifest (`testdata/firmware/`) and one on the whole 97,251-record file when named. The page: `ConfigFirmwareManifest.Designer.cs`'s 946×375 layout with the vehicle pictures as named boxes, `Activate`'s labels in its order, Beta, a vehicle's click to the chosen file, `ConfigFirmwareDisabled`'s text when connected, **Upload disabled** - nothing flashes in this build; `mpr firmware list`. `tests/gui/config-firmware.gui` runs offline against the fixture. Ledger 1 done, 12 partial, 42 missing |
| 22 | The flight screen's second batch | D10's next rows a pilot reaches for: quick view fields, tlog playback, POIs, and row 18's panels moved | D10 | quick view with the C#'s field chooser, the Telemetry Logs page's playback controls over the replay transport, POIs with the `.poi` file, the logs panel on DataFlash Logs, the estimator behind the HUD's Vibe/EKF, identity dropped, each with a script | in flight |
| 23 | The dataflash conversions | `but_bintolog`, `but_dflogtokml`, `BUT_matlab`, `BUT_loganalysis` on the DataFlash Logs page | D14 | `.BIN → .log` byte-identical to `BinaryLog.ConvertBin` under mono on the two fixtures, KML+GPX and `.mat` proved against the harness or element by element, Auto Analysis as the C# runs it, `mpr log <verb>` | in flight |
| 24 | Per-frame parameter costs | row 6 found them: the view copies the parameter table every frame, and the documentation lookup scans for numbered names | D10, D12 | the view rebuilds its parameter list only when the table's generation changes, the lookup is indexed with the C#'s matching rule kept, both proved by tests and the figures recorded | in flight |
| 25 | Radio Calibration complete | the ledger's partial row: no `RCn_TRIM`, no Reverse, no Spektrum bind, a threshold and a summary the C# does not have | D13 | `ConfigRadioInput.cs` whole: the 16 bars, the calibration conversation writing MIN/MAX/TRIM forced, Reverse per firmware, the DSM binds, with a script | done: `config/radio.rs` - the bars as `HorizontalProgressBar2` draws them, `Activate`'s `RCMAP_*` binding and " (rcN)" labels (`:42-177`), `Calibrate Radio`'s six steps (`:197-406`) with the channel-1 gate, trims constrained into range and `RCn_MIN/MAX/TRIM` written forced, Reverse to `RCn_REV` or `RCn_REVERSED` with the `SWITCH_ENABLE` write first (`:419-469`), DSM2/DSMX/DSM8 as `START_RX_PAIR` (`:471-508`). **Found:** the old `RcRange` invented a 50 µs travel threshold, skipped absent channels, started the minimum at 65535, wrote no trims and made up its summary; rewritten to the C#'s arithmetic. SITL's inputs are static, so the script proves the conversation and that nothing is written, as the C# would write nothing |
| 26 | Motor Test complete | the ledger's partial row: no Test all, no sequence, no duration, limits the C# does not have | D13 | `ConfigMotorTest.cs` whole: one button per motor from the frame's layout, `DO_MOTOR_TEST` with the C#'s parameters, Test all, Sequence, Stop, Spin Arm/Min, with a script | done: `config/motor_test.rs` and `mp_calibration::motor_layouts` (the 75 layouts of the C#'s `APMotorLayout.json`), `get_motormax`'s rules (`:111-233`), `testMotor`'s seven parameters (`:305-327`), Test all / Sequence / Stop (`:263-288`), Set Motor Spin Arm/Min (`:341-396`). **Found:** the old panel invented a 15 % throttle cap, a fixed 2 s, a props warning and an armed lock-out the C# does not have; gone |
| 27 | Compass complete | both compass rows partial: no priority table, no Use/Learn/Fitness, no Accept, no Large Vehicle MagCal | D13 | `ConfigHWCompass2.cs` whole and `ConfigHWCompass.cs` where 4.0 firmware gets it, with a script | in integration: the priority table decoding device ids as `Device.cs` does, Up/Down and Remove Missing writing `COMPASS_PRIO*_ID` (`:184-262, 519-532`), Use/Learn/Fitness, Start/Accept/Cancel with the C#'s parameters, the 100 ms timer's bars and report text (`:358-464`), Large Vehicle MagCal (`:476-498`), the reboot question; the 4.0 page but for Live Calibration. **Found and fixed:** `cancel_compass` sent param3=0 where the C# sends 0,0,1. **Divergence, at the site:** the C# shows "Reboot failed" when the reboot succeeded (`:166-169`); ours shows it when it did not |

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
| "The C# source is not on this machine" | **FALSE, and it cost real rework** | `referneces/missionplanner` is a full read-only clone, gitignored so it never appears in `git status`. `ExtLibs/Utilities/ParamFile.cs` was reconstructed from memory and got the skip-list length, the side it applies to, and the number format all wrong |
| "`ARMING_CHECK` → `ARMING_OPTIONS`" | **WRONG NAME, AND THE SENSE IS INVERTED** | 4.7 SITL reports `ARMING_SKIPCHK` and `ARMING_OPTIONS` as separate parameters and no `ARMING_CHECK`. `ARMING_CHECK` was checks-to-run; `ARMING_SKIPCHK` is checks-to-skip, so "all off" went from `0` to `-1` |
| "`frame_parse` fuzzes the MAVLink parser" | **TRUE BUT NEARLY EMPTY** | it reaches 90 coverage edges and never calls a message decoder. Seeding the corpus with 145 real frames changed nothing, which is what proved it. `message_decode` reaches 13,473 |
| "Compiling on `windows-latest` proves the Windows build works" | **FALSE** | nothing in CI ran a graphics backend on any platform. D3D11 device creation, swap-chain and shader compilation had never executed |
| "An event-driven joystick read tells you the device is alive" | **FALSE, and it is a flight-safety bug** | `/dev/input/js*` is edge-triggered: a held stick emits nothing. A failsafe fed by event arrival releases control to the transmitter after 200 ms of a pilot holding a position — on a feature whose premise is that there is no transmitter |
| "The 19 stock scripts are Python, so a Python engine keeps them working" | **TRUE of the language, FALSE of the scripts** | 15 of 19 reach .NET types directly through IronPython's assembly loading. 4 are portable. Measured by `cargo test -p mp-script` |
| "`Script.cs` is the scripting API to port" | **IT IS A QUARTER OF IT** | 11 of 19 scripts call `MAV.*` (`MAVLinkInterface`), 6 call `cs.*` (`CurrentState`); only 1 uses `Script.*` alone and 5 need no host at all |

