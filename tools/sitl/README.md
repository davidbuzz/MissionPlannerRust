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
tools/sitl/run-sitl.sh plane

mpr watch tcp:127.0.0.1:5760
cargo test -p mp-link -- --ignored   # the live-vehicle test
```

## Why a binary is committed

The alternative is asking every contributor, and every CI job, to clone ArduPilot (a large repo)
and build it (several minutes) before a single integration test can run. A 5.8 MB binary that
pins an exact, reproducible vehicle firmware is the cheaper trade, and it makes
"does the port actually talk to an autopilot?" a question anyone can answer in one command.

If the binary is ever inconvenient - a different architecture, a newer ArduPilot, a licence audit
that prefers source-only - `run-sitl.sh` takes `SITL_BINARY` to point at your own build, and
nothing else in the repository depends on these files.
