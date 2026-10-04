# ArduPilot SITL as WebAssembly, built locally

Four vehicles built by the owner from ArduPilot's new `wasm` board - copter and plane on
2026-09-25, rover and heli on 2026-10-04 - which ArduPilot
does not publish yet (its firmware manifest lists no WebAssembly SITL, which is what
`crates/mp-gui/src/sitl/wasm.rs` probes for). Kept here so the planner's SITL screen can be
pointed at them without a toolchain (PLAN.md §12 D14 and D21, §13.6 row 78).

| | |
|---|---|
| Files | `arducopter.js` + `arducopter.wasm` (3.6 MB), `arduplane.js` + `arduplane.wasm` (3.6 MB), `ardurover.js` + `ardurover.wasm` (3.4 MB), `arducopter-heli.js` + `arducopter-heli.wasm` (3.6 MB, ArduPilot's heli target) |
| ArduPilot | Unmodified ArduPilot master (the owner, 2026-10-04): `ArduPilot-4.6.0-beta1-8776-g9f648ccabc`, commit `9f648ccabcf25a421192dc5b57491ce981506872` of https://github.com/ardupilot/ardupilot - the corresponding source for all four. Built in `~/ardupilot_rp2350_v6_buzz`, whose working tree on 2026-09-25 also held two of the owner's RP2350 files, which the `wasm` board does not build: the copter rebuilt there on 2026-10-04 from the clean tree is byte for byte the one built then |
| Toolchain | Emscripten 6.0.8 from emsdk (`Tools/environment_install/install-wasm-prereqs-ubuntu.sh`); Ubuntu's `emscripten` 3.1.6 package cannot link the board (its `wasm-ld` rejects waf's `-Bstatic`/`-Bdynamic` markers) |
| Built with | `./waf configure --board wasm && ./waf build --target bin/arducopter` (and `bin/arduplane`, `bin/ardurover`, `bin/arduheli`, whose output is `arducopter-heli`) |
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

So they run under Node or a browser, not under wasmtime. `bridge.mjs` (this project's, GPL-3.0-only)
is the bridge: it loads a module, starts its vehicle with the arguments it is given, and serves
SERIAL0 on tcp:127.0.0.1:<port>. The SIMULATION screen's "try local wasm" box (the owner's,
2026-10-04; ticked by default on macOS) starts it for the picture clicked - plane, rover, copter or
heli - with Mission Planner's command line translated (`--serial0 wasm`, no `--defaults` file:
each module loads its vehicle's defaults from its own ROMFS), and the planner connects to 5760 as
it does to a native SITL:

```sh
node tools/sitl/wasm/bridge.mjs tools/sitl/wasm/arducopter.js 5760 -Mquad -O-35.36,149.16,584,353 -s1 --serial0 wasm --serial1 none --serial2 none
```

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
SITL"). The rover (`--model rover`) and the heli (`--model heli`) sent MAVLink on SERIAL0 the same
way on 2026-10-04; the copter through the bridge is a test (`sitl::launcher`'s
`the_local_copter_speaks_mavlink_on_the_bridges_port`) and `tests/gui/sitl-local-wasm.gui`. `package.json` beside the modules declares them ES modules, which Node 18 needs to load
a `.js` that uses `import.meta`; Node 22 and later work it out from the syntax.
