# ArduPilot SITL as WebAssembly, built locally

Two vehicles built by the owner on 2026-09-25 from ArduPilot's new `wasm` board, which ArduPilot
does not publish yet (its firmware manifest lists no WebAssembly SITL, which is what
`crates/mp-gui/src/sitl/wasm.rs` probes for). Kept here so the planner's SITL screen can be
pointed at them without a toolchain (PLAN.md §12 D14 and D21, §13.6 row 78).

| | |
|---|---|
| Files | `arducopter.js` + `arducopter.wasm` (3.6 MB), `arduplane.js` + `arduplane.wasm` (3.6 MB) |
| ArduPilot | `ArduPilot-4.6.0-beta1-8776-g9f648ccabc`, commit `9f648ccabcf25a421192dc5b57491ce981506872` of https://github.com/ardupilot/ardupilot, built from a working tree carrying two uncommitted files of the owner's RP2350 work |
| Toolchain | Emscripten 6.0.8 from emsdk (`Tools/environment_install/install-wasm-prereqs-ubuntu.sh`); Ubuntu's `emscripten` 3.1.6 package cannot link the board (its `wasm-ld` rejects waf's `-Bstatic`/`-Bdynamic` markers) |
| Built with | `./waf configure --board wasm && ./waf build --target bin/arducopter` (and `bin/arduplane`) |
| Licence | GPL-3.0-or-later, ArduPilot's; these are build artefacts of ArduPilot, not part of this project, and the source is at the commit above |

## What they are

Each `.js` is an ES6 module of Emscripten's JS glue (`-sMODULARIZE`, `-sEXPORT_ES6`, pthreads
with `-sPROXY_TO_PTHREAD` and a pool of four workers, growable memory) that loads the `.wasm`
beside it. There is no TCP port: SERIAL0 is exposed as exports on the module, which a host calls
over the module's heap (`libraries/AP_HAL_SITL/UARTDriver.cpp:1333-1355`):

```
ardupilot_serial_write(serial_num, buf, len) -> written
ardupilot_serial_read(serial_num, buf, max_len) -> read
ardupilot_serial_read_available(serial_num) -> bytes
ardupilot_malloc(size) -> pointer
```

So they run under Node or a browser, not under wasmtime, and a bridge that reads and writes
those exports and serves tcp:127.0.0.1:5760 is what the planner would connect to.

## Checking them

`wasm_plane_smoke_test.mjs` is ArduPilot's own check, copied from `Tools/autotest/` at the same
commit (GPL-3.0-or-later, ArduPilot's); `wasm_copter_smoke_test.mjs` is the same with
`--model quad`. Each loads its module, starts the vehicle with `--serial0 wasm`, and waits
fifteen seconds for MAVLink on SERIAL0:

```sh
node tools/sitl/wasm/wasm_plane_smoke_test.mjs tools/sitl/wasm/arduplane.js
node tools/sitl/wasm/wasm_copter_smoke_test.mjs tools/sitl/wasm/arducopter.js
```

Both passed here on 2026-09-25 under Node 18.19 ("Received MAVLink data from ... WebAssembly
SITL"). `package.json` beside the modules declares them ES modules, which Node 18 needs to load
a `.js` that uses `import.meta`; Node 22 and later work it out from the syntax.
