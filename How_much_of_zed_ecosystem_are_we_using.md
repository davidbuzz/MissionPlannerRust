# How much of the Zed ecosystem are we using?

As of 2026-09-26 (main at `a72f272`). "Zed" here is the Zed code editor
([zed-industries/zed](https://github.com/zed-industries/zed)), a read-only clone of it kept outside this repository,
and one of the models this port was built on: DELIVERABLES.md sets the goal - Mission Planner
reimplemented "extremely fast, multi-platform, GPU-accelerated end to end" - and names the stack
"Zed's ecosystem". The question: what do we take from Zed **explicitly** - named in our manifests
or our code - as against what we took as inspiration.

## The short answer

**One framework, named explicitly: `gpui`**, Zed's GPU UI framework, pinned to Zed's monorepo at
revision `62e5991`, together with its **three platform backend crates** (`gpui_linux`,
`gpui_windows`, `gpui_macos`) and **one patch** that points `calloop` at Zed's fork. That is five
names: the four gpui crates are Apache-2.0, and the `calloop` fork keeps calloop's MIT licence.

Everything else from Zed in our build — **16 more crates from the Zed monorepo** and **5 crates
from Zed's forks of other projects** — arrives only because `gpui` depends on it. Our code names
none of them.

**Nothing from Zed's application layer is used**: not its widget kit (`ui`), editor, workspace,
theme, settings, pickers, actions or extension host. Those appear here only as inspiration, listed
in §4.

| | Crates | Named by us | Licence |
|---|---:|---|---|
| gpui and its backends | 4 | yes — `crates/mp-gui/Cargo.toml` | Apache-2.0 |
| Zed's `calloop` fork (patch) | 1 | yes — root `Cargo.toml` `[patch.crates-io]` | MIT (upstream's) |
| Other Zed-monorepo crates gpui pulls in | 16 | no | Apache-2.0 |
| Crates from Zed's forks gpui pulls in | 5 | no | upstreams' own |
| Zed application crates (`ui`, `editor`, ...) | 0 | — | (GPL-3.0; none in the build) |

Of 1,024 packages in `Cargo.lock`, 26 come from Zed's repositories: 20 from the monorepo, 6 from its forks.

## 1. Named explicitly

### `gpui` — the whole GUI

- **Where:** `crates/mp-gui/Cargo.toml:24`,
  `gpui = { git = "https://github.com/zed-industries/zed", rev = "62e5991dd0f0c8a3af8d5e7e9c4652490d468db8" }`.
  Git-pinned since `feaa408` (2026-09-23). Before that it was the crates.io release `0.2.2`.
- **Why the tree and not crates.io** (the manifest's own comment, and PLAN.md §2.3): the published
  0.2.2 is an older snapshot on the `blade` renderer with no web backend. The tree renders with
  wgpu on Linux and the web, Direct3D 11 on Windows and Metal on macOS, "and staying on the tree is
  also how Zed itself consumes it". The cost: a git pin to a 245-crate monorepo with no semver.
- **How much of our code:** 81 of `mp-gui`'s source files use it, plus one example
  (`examples/tile_atlas_probe.rs`). `mp-gui` is about 193,000 lines. No other crate in the
  workspace touches gpui: link, telemetry, parameters, missions, logs, firmware and scripting are
  all our own and UI-free.
- **What we use of it:**

  | Area | gpui items (with how often the code names them, where counted) |
  |---|---|
  | Elements and layout | `div`, the `Styled`/`prelude` builder (flex, sizes, colours), `canvas` (16), `deferred` (41) and `anchored` (41) for every drop-down, menu and dialog, `uniform_list` (once: the parameter rows), `Stateful`, `AnyElement`, `IntoElement` |
  | Window, app and state | `Application`/`App` (29), `Window` (41), `Context`, `FocusHandle` (34), `cx.spawn`, `Task`, `WindowOptions`/`TitlebarOptions`, `open_window`; `cx.new` only 9 times — one root view (the `MissionPlanner` window) plus tooltip views |
  | Input | `ClickEvent` (34), `KeyDownEvent` (28), `MouseDownEvent`/`MouseUpEvent`/`MouseMoveEvent`, `ScrollWheelEvent`, `MouseButton`, `Keystroke`, `Modifiers`, `ScrollHandle` |
  | Geometry and style | `point` (52), `px`, `Pixels`, `relative`, `size`, `Bounds`, `Edges`, `Corners`, `Hsla`, `rgb`/`rgba`, `FontWeight`, `BorderStyle`, `CursorStyle`, `ContentMask` |
  | Custom painting | `window.paint_path` (30 sites, lyon-tessellated paths: map tracks, fences, HUD), `paint_quad` (23), `paint_image` (7, map tiles and video frames through `RenderImage`), `PathBuilder`, `fill`, `quad`, text shaping (`text_system`, `shape_line`, `TextRun`) |
  | Text | `SharedString` (19) |

- **What we deliberately do not use of gpui:** entities as an architecture (the app is one root
  view; every screen is a function returning elements), gpui's action and key-binding system
  (`actions!`, `on_action`, `KeyBinding`: none — keys are handled in `on_key_down`), the `img()` and
  `svg()` elements (pictures are painted), animations, and gpui's test harness (`TestAppContext`,
  `#[gpui::test]`: none — GUI tests drive the real window through the facts and probe files, with
  xdotool on Linux and user32 on Windows).

### `gpui_linux`, `gpui_windows`, `gpui_macos` — choosing the platform

- **Where:** `crates/mp-gui/Cargo.toml:97-103`, one per target, at the same revision.
- **Used in:** `crates/mp-gui/src/platform.rs` (51 lines), which starts the application on the
  right backend: `gpui_linux::current_platform`, `gpui_windows::WindowsPlatform::new`,
  `gpui_macos::MacPlatform::new`.
- **Why named directly:** it reimplements Zed's `gpui_platform::current_platform` (about 20 lines),
  because consumed as a git dependency `gpui_platform`'s inherited `gpui.workspace = true` resolves
  against crates.io, where an unrelated crate called `gpui_platform` exists at a higher version.

### `calloop` — Zed's fork, by patch

- **Where:** root `Cargo.toml:101`, `[patch.crates-io] calloop = { git = "https://github.com/zed-industries/calloop" }`,
  beside `async-task` pinned to the smol-rs revision Zed pins.
- **Why:** gpui's own workspace patches these two crates, and a git dependency does not carry its
  consumer's patch table. Without them gpui would build against a different executor and Linux
  event loop than the ones Zed tests against.

### Not a dependency, but named: the reference clone

That clone (121 MB, 245 crates, at the same `62e5991`) is a read-only checkout, outside the repository, for reading
gpui's source. It is never built. The plan's findings about gpui's renderers (PLAN.md §2.1) come
from reading it.

## 2. Pulled in by gpui, never named by us

From the Zed monorepo, all at `62e5991`, all Apache-2.0 (`perf` lives in Zed's `tooling/`):

`gpui_apple`, `gpui_wgpu` (the backends' shared pieces), `gpui_macros`, `gpui_shared_string`,
`gpui_util`, `collections`, `sum_tree`, `refineable`, `derive_refineable`, `http_client`,
`scheduler`, `perf`, `util_macros`, `zlog`, `ztracing`, `ztracing_macro`.

From Zed's forks of other projects, each under its upstream's licence:

- `zed-font-kit` — Zed's `font-kit` fork (font loading)
- `zed-scap` — Zed's `scap` fork (screen capture)
- `zed-xim`, `xim-ctext`, `xim-parser` — Zed's `xim-rs` fork (X input methods on Linux)

Our code's many `collections::` paths are the standard library's `std::collections`, not Zed's
`collections` crate.

## 3. Not used: Zed's application layer

- **`ui`, Zed's widget kit** — not vendored. PLAN.md's decision table weighed it (28,832 lines,
  GPL-3.0-or-later, editor-shaped, `publish = false`) against writing our own `mp-ui` of about 40
  components, and chose our own. `mp-ui` itself does not exist yet (NOT_DONE_YET_MATRIX.md,
  "mp-ui, the widget facade crate"). Today the widgets are `mp-gui`'s own: `ui.rs` (246 lines of
  buttons, panels and fields), `textfield.rs` (1,525 lines, the text box with its keys and
  selection), and the drop-downs, menus, dialogs and scrolling written on `deferred`/`anchored`.
- **`editor`, `workspace`, `theme`, `settings`, `picker`, `menu`, `zed_actions`** — none in
  `Cargo.lock`.
- **Zed's extension system** (`extension`, `extension_host`, `extension_api`) — not used.
  `mp-plugin-host` runs plugins on wasmtime's component model with a WIT world of our own
  (`wit/plugin.wit`), the same technology Zed's host uses, with none of Zed's code.

## 4. Inspiration: what Zed shaped without its code in our build

Zed is where the port's premise comes from: a desktop application whose whole UI is drawn on the
GPU, fast enough to feel instant, one codebase on Linux, Windows and macOS. DELIVERABLES.md's goal
line ("extremely fast, multi-platform, GPU-accelerated end to end") and its stack line ("Zed's
ecosystem") are that premise. Beyond the framework itself, this is what the port took from Zed, and
what it looked at in Zed and chose against.

**Taken:**

| From Zed | Where it shows |
|---|---|
| The premise: a GPU-drawn native UI, fast, the same code on three desktops | DELIVERABLES.md goal and stack lines; PLAN.md §2 "Verdict on the Zed / gpui ecosystem" (ADOPT) |
| Consuming gpui from the Zed tree - "how Zed itself consumes it" - not the crates.io snapshot | `mp-gui/Cargo.toml` comment; PLAN.md §2.3 |
| Zed's `[patch]` table (async-task, calloop), so the executor and event loop are the ones Zed tests | root `Cargo.toml` |
| Platform selection the way Zed's `gpui_platform::current_platform` does it | `platform.rs` (reimplemented) |
| Drawing our own scrollbars, as Zed does - gpui has no scrollbar geometry | `ui.rs:180` |
| Map, HUD, tracks and fences drawn with gpui's own primitives and atlas-cached images, as Zed draws its UI, rather than owning a GPU surface | ADR 0001 (map viewport in gpui); PLAN.md §2.2 |
| A sandboxed WASM extension host "modelled on zed's `extension` / `extension_host` / `extension_api`" | DELIVERABLES.md Deliverable 16; built as `mp-plugin-host` on wasmtime's component model, our own WIT world, none of Zed's code |

**Looked at in Zed, and not taken:**

| Zed's | Decision | Why (PLAN.md) |
|---|---|---|
| `crates/ui`, its widget kit (28,832 lines) | write our own `mp-ui` | GPL-3.0-or-later, editor-shaped, `publish = false`; revisit if ours passes ~25k lines |
| `ui_input::InputField` for text boxes - a 272-line shim over `crates/editor` (181,570 lines) | write our own single-line editor (`textfield.rs`) | far too much editor for a text box; revisit only if IME cannot be made right on three OSes in under 2k lines |
| Its release profile: thin LTO, ~15 targeted `opt-level` entries, `[profile.dev.build-override]` | proposed in §5.4, **not adopted** | release is still fat LTO with one codegen unit; every dependency at `opt-level = 2` in dev |
| Its toolchain pin, 1.98.1 | not taken | MSRV stays 1.95.0 |
| Its screenshot test, `MATCH_THRESHOLD = 0.99` of exact pixels | rejected | "tuned for one Metal machine and will not survive a driver change"; the plan wants perceptual comparison instead - and today GUI tests assert on published facts, with no image comparison |

`gpui-component` (a third-party kit built on a copy of an older Zed, `gpui-pre 0.3.6`, with no wgpu)
was also rejected; it is not Zed's own.

## 5. What it costs us, and what would change it

- **One revision to move:** every Zed crate in the build is at `62e5991`; updating gpui means moving
  that pin for all 20 monorepo crates at once, with no semver to say what broke.
- **The renderers are Zed's to change:** gpui has no custom GPU escape hatch (a closed primitive
  enum; `paint_surface` is macOS-only), so live video and 3D terrain go through gpui's image and
  path primitives (PLAN.md §2.1, correction 3).
- **Licences:** every crate from Zed's monorepo in the build is Apache-2.0, and Zed's forks keep
  their projects' own permissive licences (calloop MIT, and so on); all are inbound-compatible with
  the port's GPLv3. Zed's application crates (`ui`, `editor`, ...) are GPL-3.0-or-later, which the
  port could take too (PLAN.md: "legally fine") - they were declined for being editor-shaped, not
  for their licence.

---

*Not the Zed editor:* "ZedGraph" in `docs/coverage/logbrowse.md` and `docs/coverage/flightdata.md`
(`ZedGraphControl`, `ZedGraphTimer_Tick`) is the C# chart library Mission Planner is built on; it
is unrelated. Every other mention of Zed in the repository - PLAN.md, DELIVERABLES.md, README.md,
ADR 0001, the manifests and the code - is the editor.
