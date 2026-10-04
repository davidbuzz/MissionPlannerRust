# Building MissionPlannerRust on macOS

How the planner is built, tested and made into a release binary on a Mac, as it was first done on
2026-10-03. Every command and number here comes from that run; where something was not tried, it
says so.

## The reference machine

| | |
|---|---|
| Hardware | Apple Silicon (arm64), 10 CPU cores, 16 GB of memory |
| macOS | 26.3 (build 25D125) |
| Compiler | Xcode 26.2 at `/Applications/Xcode.app`, Apple clang 17.0.0 |
| Rust | rustup 1.29.1; the toolchain the repository pins, 1.95.0 (`rust-toolchain.toml`) |
| Also present | Homebrew, with `pkg-config`; no `cmake` (the build did not ask for one) |
| Disk | 41 GB free at the start of the release build; the first debug build was made with 11 GB free |

The build was driven over SSH from a Linux machine. The Mac was somebody else's desktop, with its
owner logged in at the console, which matters for one step below (running the window).

## 1. Prerequisites

A C compiler and linker, which on macOS come from Xcode or its Command Line Tools:

```sh
xcode-select --install        # the Command Line Tools; a full Xcode works too (26.2 was used)
```

The Command Line Tools alone are the usual minimum for Rust on macOS and should be enough; the
reference machine had the whole Xcode, so "enough" is not something this run proved.

Rust, through rustup, which installs the pinned toolchain the first time `cargo` runs in the
repository, with the `rustfmt`, `clippy` and `rust-src` components and the two cross-check targets
`rust-toolchain.toml` names:

```sh
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source ~/.cargo/env
```

Nothing from Homebrew is known to be needed. `pkg-config` was present on the reference machine
and the build may or may not have consulted it; a machine without it has not been tried.

## 2. Clone

```sh
git clone https://github.com/davidbuzz/MissionPlannerRust ~/MissionPlannerRust
cd ~/MissionPlannerRust
```

The C# source of Mission Planner is not needed: the tests that measure the port's completion
against it read `MP_SRC`, and when that variable is not set they skip, successfully. Leave it unset
on a Mac unless you have a clone of https://github.com/ArduPilot/MissionPlanner to point it at.

## 3. The debug build

```sh
CARGO_INCREMENTAL=0 CARGO_PROFILE_DEV_DEBUG=0 cargo build -p mp-gui --bin planner
```

The two variables are for a small disk: no incremental caches and no debug info, which together
are most of a Cargo target directory. Measured, from a cold cache:

| | |
|---|---|
| Crates compiled | 649 |
| Wall time | 6 min 42 s |
| `target/debug/planner` | 128 MB |
| `target/debug/` afterwards | 3.9 GB (5.1 GB once the test binaries were built too) |

A plain `cargo build -p mp-gui --bin planner` works as well and is what the CI job runs; it writes
debug info and incremental state and needs several times the disk. The whole workspace
(`cargo build --workspace`) builds the command-line tool `headless-planner` and every library and
example as well.

## 4. The tests

```sh
CARGO_INCREMENTAL=0 CARGO_PROFILE_DEV_DEBUG=0 cargo test --workspace --no-fail-fast
```

This runs 177 test binaries. Three kinds of test are not exercised on a Mac, by design:

- **Completion measurements** against Mission Planner's C# skip while `MP_SRC` is unset.
- **Vehicle tests** (`--ignored`, in `mp-link`) need an ArduPilot SITL on `tcp:127.0.0.1:5760`.
  The simulator binaries bundled under `tools/sitl/` are Linux x86-64 executables and do not run
  on macOS; `tools/sitl/run-sitl.sh` takes `SITL_BINARY=<path>` for an ArduPilot `arducopter`
  built locally (`./waf configure --board sitl && ./waf copter` in an ArduPilot checkout, which
  builds on macOS). This was not tried on the reference machine.
- **GUI scripts** under `tests/gui/` run through `tools/gui-headless.sh`, which is Xvfb and Mesa's
  lavapipe: Linux only.

The first run, at commit d2e634c, had eleven failures in eight binaries, all macOS-specific and all
fixed in the commit that adds this file:

- Nine were golden comparisons that asked for Mission Planner's doubles **to the bit**: the UTM
  projection (two tests in `mp-mission`), the corridor and rotary grids, and three Web Mercator and
  distance tests in `mp-units`. Apple's libm returns the last bit of a sine, cosine or arctangent
  differently from glibc's in places, and the goldens were written by Mission Planner's own code
  under mono on Linux. The differences seen were one ulp in a UTM coordinate, eight in a Web
  Mercator inverse (`atan` and `exp` composed) and 4e-19 in a latitude near the equator: a
  nanometre at most. Those tests now hold to `mp_units::golden_match` - equality on Linux, sixteen
  ulps or 1e-14 elsewhere - and they still report where identity holds. Two triangle cases of the survey grid decide a lane against the polygon in
  that last bit and land it 60 m along, as `Grid.cs` itself would on that libm; off Linux they are
  held to the grid invariants (`TIE_BREAKS` in `crates/mp-mission/tests/grid_vectors.rs`).
- Two were pseudo-terminal harnesses in `mp-transport`, which macOS refuses: the pty slave's second
  open by path fails with ENOTTY ("Not a typewriter"). They are Linux-only now, as `mp-firmware`'s
  already were; the serial path they exercise is held by the Linux run.
- One was the UDP transport golden, whose two-read split of 140 bytes is Linux's FIONREAD
  semantics (the next datagram); macOS counts every queued byte, as Windows does, and the transport
  already follows the platform, so the test now does too.

With those fixes applied, the eight binaries passed on this Mac (three runs on 2026-10-03).

Two timing tests are worth knowing about. The joystick reader's (`mp-input`) pass on this Mac and
failed on GitHub's hosted macOS runner, which woke the reader's thread up to 90 ms late; they now
collect a run of frames until the reader falls silent and scale their slack to the machine's
measured wake-up lateness. The snapshot cadence test (`mp-link`, `publish_cadence`) failed one run
in three here: macOS sleeps 16.7 ms as about 22 ms, so a 20 ms cadence repeated fewer snapshots
than the test's fixed floor expected; the floor now follows the frame the run measured, and here it
measures a frame longer than that cadence and checks only that the default cadence repeats none
(five runs of five passed).

## 5. Running it

`headless-planner`, the internal testing tool, is a command-line program and runs anywhere a
shell does (`headless-planner watch file:testdata/mavlink/autotest.tlog` replays a recording); it
is not part of the application and is never released. The `planner` window needs the
logged-in desktop session: launched over SSH as a user who is not the one at the console, it
opened no window and the smoke run timed out. Launch it from a Terminal in your own session:

```sh
./target/debug/planner
MP_SMOKE=1 MP_NO_TILES=1 MP_NO_RECORD=1 ./target/debug/planner   # open, paint, exit: what CI checks
```

CI's `test (macos-latest)` job does exactly that smoke step after the tests, on a hosted runner
with a desktop session of its own.

Settings, recorded flights and map tiles go to the planner's own data directory, never to Mission
Planner's.

## 6. The release binary

The release is the planner alone: headless-planner is an internal testing tool, not part of the
application (the owner, 2026-10-04).

```sh
LZMA_API_STATIC=1 cargo build --release -p mp-gui --bin planner
```

The variable is for a Mac that has Homebrew's `xz`. RustPython's `lzma` module (in the planner
through `mp-script`) is built on `liblzma-sys`, which takes the system's liblzma through
`pkg-config` when it finds one and otherwise compiles xz from its own source into the binary. The
first release build here, made without the variable, linked `/opt/homebrew/opt/xz/lib/liblzma.5.dylib`,
a library another Mac does not have at that path; with it, the re-link (6 min 07 s from the cached
dependencies) linked no Homebrew library at all - `otool -L` shows only macOS's frameworks and
`/usr/lib` - and the stripped planner grew by 118 KB, which is xz.

The release profile is `Cargo.toml`'s: fat link-time optimisation, one codegen unit, `debug = 1`
for symbolicated crash reports. It is the slow, memory-hungry build; measured on the reference
machine, from the debug build's dependency cache (the release profile shares nothing with it, so
every crate is compiled again):

| | |
|---|---|
| Wall time | 9 min 45 s (585 s; 1,504 s of CPU across the ten cores) |
| Peak memory (`/usr/bin/time -l`, maximum resident set) | 8.05 GB |
| `target/release/planner` | 66.5 MB (with the profile's debug info) |
| `target/release/` afterwards | 3.7 GB |

`tools/package.sh` is the Linux release script (it strips with GNU `strip`, reads `ldd` and builds
a Debian package), so on macOS the last step is done by hand:

```sh
mkdir -p dist
cp target/release/planner dist/
strip dist/planner
otool -L dist/planner        # what it links; anything under /opt/homebrew is a mistake
```

Stripped, the planner is 57.0 MB (73.9 MB since the ten shipped plugins are built into it,
2026-10-04). `file` reports `Mach-O 64-bit executable arm64`.
Built as above, the planner links macOS's own frameworks and libraries and nothing else: AppKit,
Metal, OpenGL, QuartzCore, CoreGraphics, CoreVideo, ColorSync, CoreLocation, UserNotifications,
Security, SystemConfiguration, IOKit, Carbon, ApplicationServices, CoreServices, Foundation and
CoreFoundation, with `libSystem`, `libobjc`, `libiconv` and `libffi` from `/usr/lib`.

CI's release workflow (`.github/workflows/release.yml`, run by hand) makes the same build on
GitHub's Apple Silicon runner for both architectures - `--target aarch64-apple-darwin` and
`--target x86_64-apple-darwin` - and joins them with `lipo -create` into one universal planner
that runs on every Mac, Intel ones included, which a build on an Apple Silicon Mac alone does not.

That is the release binary. There is no `.dmg`, no bundle (`Planner.app`) and no code signing or
notarisation yet; `README.md` lists them under what is not started. An unsigned binary downloaded
from elsewhere will be held by Gatekeeper until it is allowed in System Settings, which is a reason
to build it on the machine that runs it until signing exists.

## 7. Cross-checks the Mac also has

`rust-toolchain.toml` installs the `x86_64-pc-windows-gnu` and `wasm32-unknown-unknown` targets
with the toolchain, so `cargo check --target wasm32-unknown-unknown -p mp-mavlink` works here too.
A Windows *build* from a Mac would need a MinGW linker and was not attempted; the Windows binary is
built from Linux (see `README.md`) or on Windows itself.
