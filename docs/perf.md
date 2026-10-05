# Rendering performance

The owner's request of 2026-10-05: assess the planner's rendering (gpui and its own drawing), measure first, then take the easy speedups, each with a before and an after. The matrix row is NOT_DONE_YET_MATRIX.md's "crates/mp-gui (rendering)"; the targets are PLAN.md §8.2's.

## How it is measured

`MP_FRAMES=1` makes the planner time its own frames, per screen and per visit (`crates/mp-gui/src/frametimes.rs`), and publish them as facts (`frames.<screen>.*`):

- **render** - the top of `render` to the root element built: the planner reading its state and building the element tree. Split further into laps (`lap.<name>`: `view`, `state`, `pages`, `fly`, `plugins`, `plan`, `feeds`, `body`, `root`).
- **paint** - from there to the last element painted: gpui's layout (taffy), prepaint and paint, with the planner's own paint code inside it, which reports itself (`spent.hud`, `spent.map`).
- **present** - gpui drawing the scene and handing it to the platform. On lavapipe, the software Vulkan the headless runs use, this is the CPU rasterising; on a GPU it is small.
- **gap** - frame start to frame start; **fps** - frames a second while the screen shows; **first** - the first frame on arriving, which builds the page.

The harness's own work in a frame (the facts, the probe) is left out. Two GUI scripts drive the moments the owner named:

```sh
MP_GUI_BIN=<a release planner> MP_FACTS_KEEP=idle.facts tools/gui-headless.sh perf-screens
tools/sitl/start-sitl.sh copter &   # then:
MP_GUI_BIN=<a release planner> MP_FACTS_KEEP=connected.facts tools/gui-headless.sh perf-connected
```

`perf-screens.gui` shows every screen in turn with no vehicle; `perf-connected.gui` does the same with a SITL copter's telemetry, and pans and zooms the flight and planning maps and scrolls the parameter tree. The release binary for measuring was built with thin LTO (`CARGO_PROFILE_RELEASE_LTO=thin CARGO_PROFILE_RELEASE_CODEGEN_UNITS=16 cargo build --release -p mp-gui`); the shipped release's fat LTO takes an hour and changes the code little.

Where gpui's own time goes was found with callgrind, collecting inside `gpui::window::Window::draw` only (so not lavapipe's rasterising): `perf` needs `perf_event_paranoid` below 4, which this machine does not have.

## Before (2026-10-05, release build, lavapipe on Xvfb, 1600x1200)

Medians in milliseconds.

Idle, no vehicle:

| screen | frame | render | paint | present | fps | first frame |
|---|---|---|---|---|---|---|
| fly | 30.3 | 0.28 | 2.6 | 27.5 | 9 | 23.1 |
| plan | 34.9 | 0.40 | 9.5 | 24.7 | 10 | 72.3 |
| simulation | 28.2 | 0.31 | 1.4 | 26.2 | 10 | 35.4 |
| config | 15.3 | 0.35 | 2.2 | 12.4 | 10 | 15.9 |
| help | 12.2 | 0.23 | 1.1 | 10.8 | 9 | 13.7 |
| params | 7.7 | 0.23 | 1.5 | 6.0 | 10 | 10.9 |
| plugins | 5.9 | 0.20 | 0.7 | 5.0 | 10 | 7.1 |
| logs | 5.8 | 0.19 | 0.8 | 4.9 | 10 | 6.6 |
| setup | 5.4 | 0.19 | 0.6 | 4.7 | 9 | 7.6 |

Connected to a SITL copter (the flight map panned and zoomed, the planning map panned, the parameter tree scrolled):

| screen | frame | render | paint | of it, HUD | of it, map | present | fps | first frame |
|---|---|---|---|---|---|---|---|---|
| fly | 103.7 | 1.23 | 17.1 | 0.56 | 0.06 | 84.1 | 8 | 30.0 |
| plan | 67.3 | 1.11 | 13.4 | - | 0.18 | 51.9 | 10 | 108.0 |
| simulation | 26.0 | 0.68 | 1.5 | - | 0.08 | 23.8 | 9 | 32.8 |
| params | 15.1 | 0.91 | 5.5 | - | - | 8.2 | 12 | 21.2 |
| config | 13.3 | 0.61 | 1.0 | - | - | 11.6 | 10 | 18.4 |

The flight screen connected, by the page showing under the HUD (paint): Quick 19.2, Messages 23.9, Gauges 25.2, PreFlight 28.3, **Actions 67.1**.

## What it says

- **The planner's own work is small.** Render - reading the state and building the tree - is 0.2 to 1.2 ms on every screen. The HUD's own drawing is 0.6 ms and the map's under 0.2 ms.
- **gpui's layout is the planner-side cost.** Inside `Window::draw` on the connected flight screen, 77% is taffy's layout pass and a quarter of all of it is measuring text there - shaping, and resolving each text's font again (`resolve_font`, `font_id`: 15-19%). The cost follows the number of elements and text runs laid out each frame, and taffy measures text more than once in a flex layout.
- **The 100 ms REFRESH is Mission Planner's own rate.** FlightData's binding update runs at 10 Hz (`GCSViews/FlightData.cs:5421-5424`, "run at 10 hz"), and the HUD's data comes through it; HUD.cs's 30 ms is only a cap on its repaint. PLAN.md §8.2 asks more of the port: 0 frames idle and disconnected (the planner draws 10 a second), packet to pixel within a frame and a millisecond (the fixed tick gives ~50 ms), ≤2% of a core with a vehicle at 10 Hz (the connected flight screen's ~18 ms a frame is ~18% at 10 frames a second). A faster repaint is only worth having once a frame is cheap.

## The easy wins

Each with its before and after, as they are taken.
