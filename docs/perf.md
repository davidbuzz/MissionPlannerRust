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

### 1. The flight screen's message list builds only the rows in view (2026-10-05)

The list keeps up to 200 of the vehicle's messages and built every one each frame - a row of three
text runs, the message itself truncated to the column, which means shaping all of it - inside a
scrolling box that showed ten. Now it is a `uniform_list` and builds the rows in its box
(`fly::messages_panel`; the facts `fly.messages.count` and `fly.messages.drawn`;
`tests/gui/fly-messages.gui`).

| connected flight screen, Quick page (perf-connected.gui) | before | after |
|---|---|---|
| paint, median | 17.1 ms | 5.5 ms |
| frame, median (lavapipe's present included) | 103.7 ms | 86.3 ms |

With 16 to 22 messages held. Before, the cost grew with every message up to 200; after, it is the
box's ten rows whatever the list holds.

## Looked at, not easy wins (2026-10-05)

Found by leaving one part of a page out at a time and measuring the rest (each the median of a
visit's paint, release build, lavapipe):

- **The flight screen's Actions page: 44.1 ms of paint against Quick's 5.4.** Without its arm and
  disarm row 45.1, without the mode list 38.6, without the C#'s action grid 12.5: the grid is ~32
  ms. Its twenty-odd buttons wrap their labels in a fifth of the column, as the C#'s
  `TableLayoutPanel` does, and gpui keeps a text's size only for the one wrap width it was last
  asked about - so each label is wrapped anew for every width taffy tries in a frame, and its font
  resolved again each time (gpui's `TextLayout::layout`). Tried: the grid as rows of equal columns
  in flex - 66.0 ms, worse; the buttons' and mode chips' labels unwrapped - 44.1, no change. Not
  taken.
- **FLIGHT PLAN: 9.6 ms of paint idle.** Without the waypoint grid and its editor 5.3, without the
  sidebar 4.8: the cost is spread over many controls, none at fault alone.

What would take these down is not a local change. gpui lays out the whole window every frame
because the planner is one view; a page that changes rarely - the Actions page, PLAN's sidebar -
could be a view of its own drawn with gpui's view caching (`AnyView::cached`), which reuses last
frame's layout and paint until that view is told it changed. That is how Zed keeps its large
windows cheap; here it means moving each such page's state behind its own entity - a refactor, not
an easy win.


## Packet to pixel and the repaint policy (2026-10-05)

The owner's question: is repainting at 30 Hz rather than Mission Planner's 10 Hz worth it, or would
it draw old data more often; should the screen repaint on new data, with a floor of once a second;
and can a tethered vehicle streaming at 100 Hz be drawn at 100 Hz. With `MP_FRAMES` the link now
stamps each vehicle state with its newest message's arrival, and frametimes publishes, per screen:
`fresh` and `stale` (frames that drew a state with a message the frame before had not, and frames
that drew the same again), `latency` (a fresh frame's packet to pixel: the newest message's arrival
to the frame presented), `wait` (the part of it before the frame began - the link's publish, then
the wait for a repaint) and `age` (how old the newest message on screen is at each frame).
`MP_REPAINT` chooses the policy (`crates/mp-gui/src/repaint.rs`): `tick:<ms>`, or `data[:<ms>]` -
repaint when a vehicle's published state has taken a message, looked for every 2 ms, and at least
every second (or `<ms>`).

Release build, lavapipe on Xvfb, 1600x1200, the flight screen held still for 20 s; the SITL copter
with every stream rate set to 4 or 50 Hz (config.xml's `CMB_rate*`), and `MP_STORM`'s in-memory
vehicle at 100 and 200 Hz (whose own repaint is every 16 ms). Milliseconds, p50 / p99.

| run | policy | fps | fresh / stale | wait | render + paint | present |
|---|---|---|---|---|---|---|
| idle | 100 ms (today) | 9 | - | - | 5.1 | 51 |
| idle | data | **1** | - | - | 4.2 | 45 |
| SITL 4 Hz | 100 ms | 6 | 100 / **40** | 78 / 177 | 8.7 | 123 |
| SITL 4 Hz | 16 ms | 6 | 74 / 35 | 70 / 204 | 9.1 | 125 |
| SITL 4 Hz | data | 4 | 90 / **0** | **16** / 216 | 8.9 | 136 |
| SITL 50 Hz | 100 ms | 5 | 110 / 0 | 10 / 48 | 9.6 | 135 |
| SITL 50 Hz | data | 3 | 80 / 0 | 10 / 69 | 9.8 | 140 |
| storm 100 Hz | data | 6 | 130 / 0 | 8.7 / 15 | 6.9 | 132 |
| storm 200 Hz | data | 6 | 130 / 0 | 5.3 / 12 | 6.9 | 128 |

What it says:

- **Present dominates here, and is lavapipe's.** Drawing a 1600x1200 frame in software takes
  110-170 ms, so no policy draws more than about 6 frames a second on this machine, and the
  latency facts' absolute numbers (150-230 ms) are mostly that. On a GPU present is a few
  milliseconds; the planner's own share of a connected frame, render and paint, is ~7-9 ms at the
  median and ~12-14 ms at the 99th percentile (storm.rs's `frame.p99_us`: 12.7-13.9 ms).
- **A faster timer draws old data.** At the SITL's 4 Hz, 29% of the 10 Hz timer's frames showed
  nothing new, and the 16 ms timer's as many: the timer only shortens the wait until the next
  frame, and redraws the same state in between.
- **Repainting on new data** drew no stale frame, cut the wait from a 4 Hz packet to its frame from
  78 ms to 16 ms at the median, and drew one frame a second with nothing arriving - the floor
  the owner asked for - where the timer draws ten.
- **Before any policy, the link held data back**: a message arriving within the 5 ms publish
  interval of the last publish waited for the link's next read to return - the next packet, or
  the 100 ms read timeout - ~104 ms when it came alone; fixed (`crates/mp-link/tests/publish_latency.rs`).
- **100 Hz**: the link and the snapshot bus keep up at 100 and 200 Hz (the storm arrived at 99 and
  199 Hz; every frame fresh, the wait 5-9 ms). Drawing at 100 Hz needs each frame's render, paint
  and present under 10 ms and a display of 100 Hz or more: the planner's part is under that at the
  median and over it at the 99th percentile on this machine, so a GPU machine would draw most of a
  100 Hz stream's states and skip a few. To be measured on the owner's machine with a GPU.

### On the owner's machine with a GPU (2026-10-05)

The same release build on the owner's Linux desktop, the window on a 60 Hz monitor (all three
of his run at 60 Hz), drawn by the laptop's Intel UHD graphics through Mesa's Vulkan (its NVIDIA
Quadro's driver not loaded), 25 s each, with `MP_FRAMES`'s readout in the window's corner - the
last second's frames a second, CPU and present times, packet to pixel and the share of fresh
frames (`frametimes::readout`, the owner's wish to see the achieved rate while testing).
Milliseconds, p50 / p99.

| run | policy | fps | fresh / stale | packet to pixel | wait | whole frame | present |
|---|---|---|---|---|---|---|---|
| SITL 4 Hz | 100 ms (today) | 10 | 96 / **144** | 49 / **164** | 39 | 5.1 / 18.4 | 1.5 / 13.2 |
| SITL 4 Hz | data | 5 | 120 / 0 | **16 / 36** | 5.6 | 5.4 / 12.6 | 1.6 / 5.8 |
| storm 100 Hz | data | **57** | 1430 / 0 | 21 / 30 | 7.9 | 7.0 / 11.1 | 2.3 / 6.0 |
| storm 200 Hz | data | **55** | 1370 / 0 | 19 / 28 | 5.8 | 7.3 / 13.6 | 2.5 / 7.9 |

- On a GPU a whole frame - the planner, gpui and present - is 5-7 ms at the median; storm.rs's
  render-and-paint 99th percentile is 6.5 ms. lavapipe's 110-170 ms present was all of the
  difference.
- A 100 or 200 Hz stream is drawn at the display's rate, every frame fresh, packet to pixel
  ~20 ms - most of it the wait for the display's next refresh. On a 100 Hz display most frames
  would fit the 10 ms budget and the slowest 1% (11-14 ms) would not.
- Today's 10 Hz timer at a 4 Hz stream: 60% of its frames redraw what the frame before showed, and
  the newest data on screen is 164 ms old at the 99th percentile; repainting on new data draws half
  the frames, none stale, at 16 / 36 ms.

Left before `data` could be the default: what changes without a vehicle's message - map tiles
arriving, the demo's pointer, a progress bar - asks for its own frames rather than leaning on the
10 Hz timer (found by the GUI suite run with `MP_REPAINT=data`); and the owner's choice.
