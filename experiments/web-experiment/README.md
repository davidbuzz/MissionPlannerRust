# MissionPlannerRust in a web browser (experiment, local only)

The whole planner, `crates/mp-gui`, built for `wasm32-unknown-unknown` and run in a web page
through gpui's own web backend (`gpui_web`, at the planner's gpui revision): every screen, the
map with its tiles, and a link to a vehicle. The vehicle can be ArduPilot's WebAssembly SITL
running in the same page.

## Try it

```sh
# 1. Build the planner for the web (nightly; std rebuilt with atomics; emsdk's clang for ring):
experiments/web-experiment/tools/planner-wasm.sh build --release
wasm-bindgen --target web --out-dir experiments/web-experiment/www/pkg-planner \
    target/web/wasm32-unknown-unknown/release/planner.wasm
# 2. Serve the page with the cross-origin isolation headers threads need:
python3 experiments/web-experiment/www/serve.py 8080
# 3. Open http://127.0.0.1:8080/planner.html?vehicle=copter (or plane, rover, heli)
#    Port box: TCP, CONNECT, accept 127.0.0.1 and 5760 - the SITL starts in the page.
```

`www/index.html` is the smaller first step: the planner's HUD alone (`src/lib.rs`, with hud.rs
linked in unchanged), fed by the SITL in the page (`?link=sitl`) or a WebSocket
(`?link=ws://...`, with `tools/ws_relay.py` in front of a TCP port).

## Tests

```sh
NODE_PATH=<node_modules with playwright> node check/planner_check.js   # the whole planner, connected
NODE_PATH=<node_modules with playwright> node check/check.js           # the HUD page
```

Both run headless Chromium. Without a GPU it has no WebGPU adapter, so gpui falls back to WebGL2
on SwiftShader.

## What it took

Most of it is mechanical and done by scripts, so it can be redone on a newer tree. Each one is a
no-op on the desktop: the replacement is std's own item there.

| Script | What a web page lacks | Replacement (std's own on the desktop) |
|---|---|---|
| `tools/port_clock.py` | std's clock panics | `web_time::{Instant, SystemTime}` |
| `tools/port_threads.py` | std cannot spawn threads | `wasm_thread`, Web Workers on shared memory |
| `tools/port_os.py` | `temp_dir`, `process::id` and `split_paths` panic | `mp_os::*` |
| `tools/port_locks.py` | the main thread may never wait (`Atomics.wait` throws) | `.os_lock()`: spins on the main thread only |

By hand:
- `mp-os::http`: a synchronous XHR from a worker, for tiles, terrain and catalogues.
- `mp-transport/src/page.rs`: the link through the page, `www/link.js` on the other side.
- Plugins (wasmtime) are not built for a page, and a stand-in carries the same names.
- RustPython loses `host_env`.
- A font is bundled.

## Not done

- `std::sync::mpsc::Receiver::recv_timeout` still reads std's clock. It is used by scripts' abort
  watch, firmware upload and the link mirror.
- `RwLock` and blocking `recv`/`join` on the page's main thread are not swept. Only the paths
  exercised so far are proven: startup, every top screen, the map, connect, the HUD.
- Files: std::fs fails in a page, so settings, logs and parameter files are not kept between
  visits. The browser's own storage (OPFS) would be the place.
- Networking beyond the page: the owner's route is Tailscale's Go client built for wasm, with
  DERP relays over WebSockets and a user-space TCP/UDP stack. `page.rs` hands the page a URL, which
  is where it would go.
- Serial: WebSerial.
