# ArduPilot SITL, bundled

Prebuilt ArduPilot software-in-the-loop binaries, so the integration tests and a developer's
first five minutes need no ArduPilot checkout or build toolchain.

| | |
|---|---|
| Version | `ArduPilot-4.6.0-beta1-7926-g36b207e558` |
| Commit | `36b207e5583065464caedb791ef88b32b43b19d7` (2026-09-08) |
| Built for | Linux x86-64, dynamically linked |
| Upstream | https://github.com/ArduPilot/ardupilot |
| Licence | GPL-3.0-or-later, same as this repository |

These are **build artifacts of ArduPilot**, not part of this project. ArduPilot is GPLv3 and its
complete source is at the URL and commit above, which is what the licence requires when
redistributing a binary.

## Use

```sh
tools/sitl/run-sitl.sh copter      # listens on tcp:127.0.0.1:5760
tools/sitl/run-sitl.sh copter -I7  # instance 7: tcp:127.0.0.1:5830 (5760 + 10 N)
tools/sitl/run-sitl.sh plane
tools/sitl/start-sitl.sh copter    # the same, in the background, proved to be streaming first
tools/sitl/make-damaged-log.py     # testdata/dataflash_damaged.bin, flown on the copter (pymavlink)

headless-planner watch tcp:127.0.0.1:5760
cargo test -p mp-link -- --ignored   # the live-vehicle tests
```

## Why a binary is committed

The alternative is asking every contributor, and every CI job, to clone ArduPilot (a large repo)
and build it (several minutes) before a single integration test can run. A 5.8 MB binary that
pins an exact, reproducible vehicle firmware is the cheaper trade, and it makes
"does the port actually talk to an autopilot?" a question anyone can answer in one command.

If the binary is ever inconvenient - a different architecture, a newer ArduPilot, a licence audit
that prefers source-only - `run-sitl.sh` takes `SITL_BINARY` to point at your own build, and
nothing else in the repository depends on these files.

## Starting it for a test run

`start-sitl.sh` starts `run-sitl.sh` in the background, connects a throwaway client and requires a
MAVLink frame within ten seconds, restarting the simulator up to three times when none comes: on
2026-09-25 one start in about a dozen stopped after "Smoothing reset at 0.001" and never sent a
frame. SITL accepts one client at a time on 5760 and the next one the moment the last leaves. To
stop it, kill by pid - `pgrep -f "sitl/arducopte[r]"` - never with a pattern the killing shell's own
command line would match.

A port something already listens on is refused: `run-sitl.sh` exits 3 rather than start a
simulator there, as SITL itself only logs its failed bind and runs on - on 2026-10-14 a click
audit's planners, whose SITL could not bind 5760, drove the owner's own SITL (started from the
SIMULATION tab) instead. `start-sitl.sh` hands its simulator over only when the listener on 5760
is the pid it started, and `tools/gui-crawl/run.sh` checks its two (instances 7 and 8) the same way.

## WebAssembly builds

`wasm/` holds the owner's local builds of ArduPilot's `wasm` board (copter and plane on
2026-09-25, rover and heli on 2026-10-04, Emscripten 6.0.8) with their provenance in
`wasm/README.md`. They are Emscripten ES6 modules that expose SERIAL0 as exports rather than a TCP
port, so they run under Node or a browser and need a bridge to tcp:5760 before the planner can
connect to them: `wasm/bridge.mjs`, which the SIMULATION screen's "try local wasm" starts.
