# A WebAssembly plugin host, as an experiment

PLAN.md §13.6 row 95, under the owner's ruling §12 D22 (2026-09-25): before anything native,
try whether Mission Planner's plugins can live in WebAssembly under wasmtime.

## What the C# does

`Plugin/PluginLoader.cs` loads every `.dll` in the plugins folder and compiles every `.cs`
there at run time; each plugin is a class deriving `MissionPlanner.Plugin.Plugin`
(`Plugin/Plugin.cs:15-42`) with `Name`, `Version`, `Author`, `Init`, `Loaded`, `Loop`, `Exit`
and `loopratehz`, and a `Host` of type `PluginHost` (`Plugin/Plugin.cs:44-180`) that opens the
whole application to it: `cs` (the vehicle's state), `comPort` (the link), `config`, the flight
and planning screens' map menus, the map controls themselves, `AddWPtoList`, `InsertWP`,
`GetWPs`. `MainV2`'s plugin thread calls `Loop` when `NextRun` has passed and the rate is above
zero, and moves `NextRun` on by `1000 / loopratehz` ms (`MainV2.cs:2524-2540`). The tree ships
about twenty example plugins and four real ones: OpenDroneID, TerrainMaker, Dowding and
AnonymizeBinlog.

Neither a .NET assembly nor run-time C# compilation can exist in the Rust port, so the mechanism
is ours to choose; the plugins' needs are the C#'s.

## What this is

Two crates in the workspace:

- `plugin/` - `fencedist`, built for `wasm32-unknown-unknown`: the C#'s `Plugin` lifecycle as
  exports, and what `plugins/example3-fencedist.cs` and `plugins/example2-menu.cs` do with
  `Host` - `Loaded` adds "Draw Fence Dist" to the flight screen's map menu, the click shows the
  example's message box and measures the distance from the vehicle (`Host.cs`) to the fence
  (`MAV.fencepoints`), `Loop` at 2 Hz keeps the distance on the status line. The host is reached
  through eight imports in the `env` module; strings cross as a pointer and a length into the
  plugin's linear memory. No WASI: the plugin has no files, clock or sockets of its own.
- `host/` - `wasm-plugin-host`: `Plugin::load` is `PluginLoader.Load` (compile, instantiate, bind
  the imports, read `Name`/`Version`/`Author`), `init`/`loaded`/`exit` the lifecycle,
  `tick` the plugin thread's `NextRun` rule, `Host` the part of `PluginHost` the plugin reaches:
  `cs`, a map menu, `CustomMessageBox.Show`, the status line, the fence points. Every call into
  the plugin gets a fresh tank of fuel.

```sh
rustup target add wasm32-unknown-unknown
experiments/wasm-plugin-host/run.sh      # builds the plugin, runs the host's tests, prints the measurements
```

`cargo test --workspace` runs the host's tests too (they build the plugin when it is missing, and
say `SKIP` on a machine without the wasm target); the plugin crate is a member so it type-checks
natively, its imports stubbed off wasm32.

## What the tests prove (`host/tests/plugin.rs`)

- a plugin loads, says who it is, `Loaded` adds its menu entry, `Exit` logs;
- `Loop` reads `cs` and the fence through the host and puts the distance on the status line;
  the host's state changes between calls are seen; no fence is the example's 99999, no position
  NaN;
- `Loop` runs at `loopratehz`, not every tick - `MainV2.cs:2524-2540`'s rule;
- the menu click shows the example's message box and measures; another plugin's entry is not
  this plugin's;
- **a plugin that panics is an error the host reports** ("the plugin panicked"), and the host and
  the plugin's state go on - the C# takes the whole application down;
- **a plugin that never returns is stopped by its fuel** within a bounded time, and the host goes
  on - the C# hangs its plugin thread;
- a module without the exports is refused at load, with a reason.

## Measurements

On this machine (2026-09-25, `run.sh`, release host, the plugin at `opt-level = "s"`):

| Measure | Value |
|---|---|
| Plugin module | 244,138 bytes (most of it `std`'s formatting; `no_std` would be a tenth) |
| Load: compile with Cranelift and instantiate | 73 ms cold, 62 to 66 ms warm |
| Instance memory | 1,114,112 bytes (17 pages: `std`'s stack and heap) |
| `Loop` call: two `cs` reads, eight fence reads, a status line | 613 ns |
| A bare export call | 238 ns |
| A runaway plugin stopped by fuel | well under a second |

The C#'s plugin thread runs `Loop` at `loopratehz`, a few hertz at most, so a call cost under a
microsecond is nothing; the load cost is per plugin at start-up, and wasmtime's precompiled
`.cwasm` form would bring it to about a millisecond if it mattered.

## What the examples need of a host, against this surface

| Need | Examples | Here |
|---|---|---|
| `cs` reads | fencedist, hudonoff, latencytracker, modechange | `host_cs_get` by name |
| a map menu entry and its click | menu, fencedist, mapicondesc, watchbutton | `host_menu_add`, `plugin_menu_click` |
| `CustomMessageBox.Show`, `InputBox.Show` | menu, and most | `host_message`; no `InputBox` yet |
| `MAV.fencepoints` | fencedist | `host_fence_count`, `host_fence_lat/lng` |
| `comPort` sends and parameter writes | modechange, canrtcm, forwarding, multiforward, herelink | not yet: the link's `doCommand`/`setParam` as imports |
| the mission grid: `AddWPtoList`, `InsertWP`, `GetWPs` | menu (Fix mission top/bottom), Shortcuts | not yet |
| map markers and overlays | fencedist (`GMapMarkerFill`), mapicondesc, multiplepositions | not yet: a marker surface |
| `config` | persistentsimple, payloadconfig | not yet: settings get/set |
| the HUD on/off, the fonts, the menus removed | hudonoff, fontsize, menuremove | not yet: window-level knobs |
| a Windows Forms panel of its own | payloadconfig, switch, TerrainMaker, OpenDroneID | the open question: a plugin cannot draw gpui; a host-drawn form described by the plugin, or no UI |
| files and sockets | canlogfile, trace, forwarding, AnonymizeBinlog | WASI, or host-lent handles |

## What this decides

**A WebAssembly plugin host works, and is better than the C#'s where it matters.** The `Plugin`
lifecycle maps one to one onto exports; `PluginHost`'s members map onto imports; the plugin
thread's rate rule is the host's; and two things the C# cannot do come for free - a plugin that
panics is an error line, not a dead application, and a plugin that never returns is stopped.
The costs are small: 65 ms to load, a microsecond a call, a megabyte an instance.

What a real host still has to decide, none of it a blocker:

1. **The API's shape.** Eight hand-written imports with pointer-and-length strings were enough
   here; `PluginHost` has some twenty members and the examples reach into the mission grid,
   the link, the settings and the map. The component model with a WIT interface gives typed
   strings, records and lists across the boundary and bindings for plugin authors in Rust, C,
   Go or Python; that is the form to build the real host in.
2. **Plugin UI.** TerrainMaker, OpenDroneID, payloadconfig and switch draw Windows Forms panels
   of their own. A WASM plugin cannot draw gpui; the host would draw a form the plugin
   describes (labels, boxes, buttons, a grid) - the C#'s own `PluginUI` is a small form - or
   those four are ported as built-in features and plugins get no UI beyond menus, messages and
   input boxes.
3. **Files and sockets.** AnonymizeBinlog, canlogfile and the forwarding examples open files
   and UDP sockets. wasmtime's WASI support gives them a preopened directory and sockets the
   host chooses; nothing else is reachable, which is the point.
4. **Distribution.** One `.wasm` for every OS, no per-OS builds, no unsafe ABI: the native
   dynamic-library option (Deliverable 21) is not needed.

Recommendation: build the host in `mp-gui` on this model, with WIT, and port the four real
plugins as its first plugins; keep the examples as the API's test suite. That is a row for
PLAN.md when the owner says so.
