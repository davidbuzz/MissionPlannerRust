# Fuzz targets

Deep fuzzing for every parser that touches input from outside the process: MAVLink frames, the
streaming decoder, waypoint files and telemetry logs.

**These targets are not built by `cargo build --workspace`** — the `fuzz` directory is excluded
from the workspace because libfuzzer needs a nightly toolchain, and an ordinary build must not
require one.

```sh
rustup toolchain install nightly
cargo install cargo-fuzz
cargo +nightly fuzz run frame_parse
cargo +nightly fuzz run frame_stream
cargo +nightly fuzz run waypoint_file
cargo +nightly fuzz run tlog_reader
```

**Status: written, not yet run.** There is no nightly toolchain on the development machine these
were written on, so they have never been compiled. Treat them as unverified until someone runs
them and records the result here. D2's definition of done requires 24 hours clean on
`frame_parse`.

The same invariants are checked on stable, on every `cargo test`, by the `robustness.rs` test in
each crate: seeded generators rather than coverage-guided mutation, so they are weaker, but they
run everywhere and every time. Those are the tests that currently hold the line.
