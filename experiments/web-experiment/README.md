# MissionPlannerRust in a web browser

The whole planner, `crates/mp-gui`, built for `wasm32-unknown-unknown` and run in a web page
through gpui's own web backend (`gpui_web`, at the planner's gpui revision): every screen, the
map with its tiles, and a link to a vehicle - ArduPilot's WebAssembly SITL running in the same page,
or anything on a Tailscale tailnet: a vehicle, its companion computer, a SITL on another machine.

## Try it

```sh
# 1. Build the planner for the web (nightly; std rebuilt with atomics; emsdk's clang for ring):
experiments/web-experiment/tools/planner-wasm.sh build --release
wasm-bindgen --target web --out-dir experiments/web-experiment/www/pkg-planner \
    target/web/wasm32-unknown-unknown/release/planner.wasm
# 2. Serve the page with the cross-origin isolation headers threads need:
python3 experiments/web-experiment/www/serve.py 8080
# 3. Open http://127.0.0.1:8080/planner.html
#    SIMULATION, then click Multirotor (or Plane, Rover, Helicopter): "try local wasm" is ticked,
#    the SITL starts in the page and the planner connects to it, as on macOS.
```

At a first visit the Welcome-Demo-Sitl plugin (the owner's, 2026-10-05; not in Mission Planner)
shows the planner at work: a drawn pointer clicks SIMULATION and Multirotor, PLAN, Zoom To
Vehicle on its zoom icon, Set Home Here on the map's right-click menu at the copter, four
waypoints around it, Write, FLY and Actions, force arm, TakeOff and Auto, so the copter flies the
mission; then it unticks itself on PLUGINS and saves, goes back to FLY, and its pointer goes.
Where the planner refuses something - Write's message box, say - it answers the box and stops,
saying why on the status line. It is a
plugin like the others, built into the browser build only, and the PLUGINS tab turns it off.
Until the page keeps its settings, every visit runs it again. `planner.html?demo=0` starts
without it, as every check but `demo_check.js` does.

## A vehicle on a tailnet

A web page has no sockets, so the page carries a Tailscale node of its own (`www/tailscale.js`):
Tailscale's own Go client built for the browser (`tailscale/main.go`, from tailscale.com's
`cmd/tsconnect`, v1.104.0), kept built in `www/tailscale/` (8 MB, gzipped) and loaded the first time
a link needs it. It joins the tailnet through Tailscale's coordination server and carries its
traffic over Tailscale's DERP relays on WebSockets; TCP and UDP to any tailnet address work.

To connect: the port box's TCP (or UDPCl), CONNECT, and a tailnet address - `100.x.y.z` or the
machine's MagicDNS name - and port. `127.0.0.1` and `localhost` stay the SITL in the page; anything
else goes over the tailnet. The first time, the page shows "sign in to Tailscale": sign in (a new
tab) and approve the device, and the link goes on. The node's keys are kept in the browser's
localStorage, so the page stays on the tailnet across reloads.

Page options: `?tscontrol=<url>` for a coordination server other than Tailscale's (Headscale),
`?tsauthkey=<key>` to join with an auth key, `?tshostname=<name>` for the node's name (default
`mpr-<words>`), and `?tsderphttp=1` for a test tailnet whose DERP relay has no TLS.

### Go: a development dependency of the tailnet networking only

Go is not needed to build, test or run the planner - in the browser or anywhere else. It is needed
only to change the browser's Tailscale networking: `tailscale/build.sh` rebuilds
`www/tailscale/tailscale.wasm.gz` from `tailscale/main.go` (with the Go version `tailscale/go.mod`
names), after a change to that file or to the Tailscale version; the result is committed, as
`tools/sitl/wasm` keeps ArduPilot's builds. `check/tailnet_e2e.sh` also uses Go, to build the
Headscale and tailscaled its test tailnet runs.

`www/index.html` is the smaller first step: the planner's HUD alone (`src/lib.rs`, with hud.rs
linked in unchanged), fed by the SITL in the page (`?link=sitl`) or a WebSocket
(`?link=ws://...`, with `tools/ws_relay.py` in front of a TCP port).

## Tests

```sh
NODE_PATH=<node_modules with playwright> node check/sim_check.js       # SIMULATION: click, start, connect
NODE_PATH=<node_modules with playwright> node check/planner_check.js   # connect through the port box
NODE_PATH=<node_modules with playwright> node check/tour_check.js      # every screen, connected, no error
NODE_PATH=<node_modules with playwright> node check/plugins_check.js   # the built-in plugins, as on the desktop
NODE_PATH=<node_modules with playwright> node check/demo_check.js      # the Welcome-Demo-Sitl, start to finish
NODE_PATH=<node_modules with playwright> node check/check.js           # the HUD page
NODE_PATH=<node_modules with playwright> check/tailnet_e2e.sh          # over a tailnet (needs Go)
```

`tailnet_e2e.sh` stands up a tailnet of its own - Headscale with its embedded DERP, behind
`check/cors_proxy.py` (which adds the `access-control-allow-origin: *` Tailscale's own control
server sends), and a userspace tailscaled named sitl-box serving a SITL on its 5760 - then connects
the planner in a page to sitl-box over it (`tailscale_check.js`), and checks the sign-in a pilot
without a key meets (`tailscale_login_check.js`). All of them run headless Chromium. The planner's facts - what `MP_FACTS` writes on the desktop for
the GUI scripts - are readable in the page with `?facts=1` (`globalThis.mpFacts()`). Without a GPU, headless Chromium has no WebGPU adapter, so gpui falls back to WebGL2
on SwiftShader.

## What it took

Most of it is mechanical and done by scripts, so it can be redone on a newer tree. Each one is a
no-op on the desktop: the replacement is std's own item there.

| Script | What a web page lacks | Replacement (std's own on the desktop) |
|---|---|---|
| `tools/port_clock.py` | std's clock panics | `web_time::{Instant, SystemTime}` |
| `tools/port_threads.py` | std cannot spawn threads | `wasm_thread`, Web Workers on shared memory |
| `tools/port_os.py` | `temp_dir`, `process::id` and `split_paths` panic | `mp_os::*` |
| `tools/port_locks.py` | the main thread may never wait (`Atomics.wait` throws); `recv_timeout` reads std's clock | `.os_lock()`, `.os_recv_timeout()`: they spin on the main thread only |

By hand:
- `mp-os::http`: a synchronous XHR from a worker, for tiles, terrain and catalogues.
- `mp-transport/src/page.rs`: the link through the page, `www/link.js` on the other side.
- Plugins: the same plugin host, on wasmtime's Pulley interpreter in a page. The planner's build
  script compiles the built-in plugins to Pulley bytecode for a wasm32 build
  (mp-plugin-host's `precompile-web-plugins`), and `crates/mp-plugin-host/src/web.rs`, the one
  file of unsafe by the owner's ruling, gives wasmtime the platform functions it needs and loads
  the bytecode.
- RustPython loses `host_env`.
- A font is bundled.

## Not done

- `RwLock` and blocking `recv`/`join` on the page's main thread are not swept. The paths exercised
  so far are proven: startup, every top screen with and without a vehicle, the map, connect, the
  HUD, and the full parameter download.
- Files: std::fs fails in a page, so settings, logs and parameter files are not kept between
  visits. The browser's own storage (OPFS) would be the place.
- The in-page SITL's eeprom.bin: its parameters do not survive a reload (the desktop's bridge
  keeps them in the vehicle's sitl folder). The same storage would keep them.
- UDP listening (`udp:0.0.0.0:14550`), where a vehicle sends first: the page's Tailscale node
  dials out (TCP, UDPCl) but does not listen yet.
- Serial: WebSerial.
- Settings are not kept, so the built-in Drone ID plugin asks its start-up question at every visit
  (the storage row above would end it).
