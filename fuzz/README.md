# Fuzz targets

Deep fuzzing for every parser that touches input from outside the process: MAVLink frames, the
streaming decoder, waypoint files and telemetry logs.

**These targets are not built by `cargo build --workspace`** — the `fuzz` directory is excluded
from the workspace because libfuzzer needs a nightly toolchain, and an ordinary build must not
require one.

```sh
rustup toolchain install nightly
cargo install cargo-fuzz
# tools/fuzz.sh points the target at the committed seeds and creates the corpus directory
# libFuzzer refuses to start without.
tools/fuzz.sh frame_parse
tools/fuzz.sh frame_stream
tools/fuzz.sh waypoint_file
tools/fuzz.sh tlog_reader
tools/fuzz.sh message_decode

# Any libFuzzer argument passes through:
tools/fuzz.sh message_decode -max_total_time=3600
```

## Where the properties live

In `crates/mp-fuzz-checks`, not here. Each file under `fuzz_targets/` is a one-line libfuzzer
entry point calling a function in that crate.

This is deliberate. The targets spent their whole existence uncompiled — the fuzz directory is
outside the workspace, so nothing built them, and a target that does not build is a target that is
not running. With the properties in a workspace crate, `cargo test --workspace` compiles them and
runs a bounded pass over them on every test run, and a target cannot drift from what is checked.

The bounded pass (`crates/mp-fuzz-checks/tests/bounded.rs`) is not a substitute for a real run. It
has no coverage feedback, so it will not find what libFuzzer finds. It is what keeps the targets
honest between runs. The same invariants are also checked by the `robustness.rs` test in
`mp-mavlink`, `mp-mission` and `mp-log`, against seeded generators.

## What the first run actually showed

`frame_parse` did 23 million executions in 46 seconds, reached 90 coverage edges in the first
second, and then found nothing at all for the remaining 45. The obvious diagnosis was a missing
corpus: random mutation essentially never produces a frame whose CRC passes, so a fuzzer with
nothing to start from should be stuck at the checksum.

That diagnosis was wrong, and testing it is what found the real problem. Seeding the corpus with
real frames from a recorded flight — 145 of them, two per message id — and running again gave
exactly the same 90 edges. The corpus was never the limit. **`frame_parse` does not decode
messages at all.** It calls `parse`, which validates the header, the length and the checksum, and
then the frame accessors. Ninety edges is the whole of the framing layer, and there was nothing
further to reach because the target never went there.

So the per-message decoders — 351 of them, each reading a payload at fixed offsets, each reachable
from the network by anything that can put a message id on the wire — had never been fuzzed by
anything. `message_decode` is the target that covers them, and zero-extension is the specific
thing it is looking for: MAVLink v2 trims trailing zero bytes, so a decoder must treat a short
payload as zero-padded rather than indexing into it. A decoder that indexes straight into the
slice works perfectly against every real sender and panics on the first truncated frame.

The corpus is still committed and still worth having — it is how `message_decode` reaches valid
message ids at all. `tools/fuzz-seed.py` builds it from any recorded `.tlog`, and the committed
one comes from the `testdata/mavlink` fixtures, so it can be rebuilt from a clean checkout:

```sh
for t in frame_parse message_decode frame_stream; do
  for f in testdata/mavlink/*.tlog; do tools/fuzz-seed.py "$f" "fuzz/seeds/$t"; done
done
```

Two frames per message id, which covers every decoder without committing a corpus nobody will
read. Re-seed it whenever a dialect gains messages; a decoder with no seed is a decoder the
fuzzer will have to guess an id for.

Seeds live in `fuzz/seeds/`, which is committed; `fuzz/corpus/`, where a run writes what it finds,
is not. A 60-second `message_decode` run ends with about 2,600 corpus entries, and running the
fuzzer should not leave two and a half thousand untracked files behind. It costs nothing: a run
from the 159 seeds alone reached 13,473 edges, slightly *more* than the same run starting from the
full discovered corpus.

The seeder validates checksums, which is not fussiness. It resynchronises on magic bytes rather
than walking a record structure — the committed fixtures open with `\n\nInit ArduCopter V4.8.0`,
so a strict walker reports a file with no frames in it — and a resynchronising scan cannot tell a
frame start from a 0xFD inside a payload. Over the two fixtures it finds 1,091 candidates and
rejects 932 of them. A seed that is not a frame teaches the mutator nothing.

## Status

Run on 2026-09-23, on an x86-64 Linux box, nightly + cargo-fuzz 0.13.2. All five targets build and
all five ran clean.

| target | executions | time | exec/s | edges |
|---|---:|---:|---:|---:|
| `frame_parse` | 23,161,376 | 46 s | 503,508 | 90 |
| `frame_stream` | 1,050,887 | 46 s | 22,845 | — |
| `waypoint_file` | 2,284,611 | 46 s | 49,665 | — |
| `tlog_reader` | 4,033,655 | 46 s | 87,688 | — |
| `message_decode` | 3,783,735 | 61 s | 62,028 | 13,419 |

The two edge counts are the point. `message_decode` reaches 150 times more of the codebase than
`frame_parse` does, which is the measure of how much was going unfuzzed before it existed.

**The 24-hour soak Deliverable 2's definition of done asks for was run 2026-09-23 16:01Z to 2026-09-24
16:01Z**, `frame_parse` and `message_decode` at once, one core each, `-max_total_time=86400`,
the fuzz soak sharing the machine with a day's builds and GUI runs. Both clean: no crash, no
timeout, no out-of-memory, `fuzz/artifacts/` empty at the end.

| target | executions | time | exec/s | edges | features | corpus |
|---|---:|---:|---:|---:|---:|---:|
| `frame_parse` | 30,729,462,219 | 86,401 s | 355,660 | 90 | 113 | 20 units, 680 B |
| `message_decode` | 4,141,970,579 | 86,401 s | 47,938 | 13,782 | 15,912 | 2,608 units, 65 KB |

`frame_parse` found nothing new after its first minute - 90 edges is the whole framing path, and
no input reaches more - so its day was thirty billion confirmations rather than a search.
`message_decode` grew its corpus from the seeds to 2,608 units and its edges from 13,419 to 13,782
over the day, the per-message decoders being where new paths still turned up. The short runs
above remain the regression check.

CI runs a bounded pass of all five on every push (the `fuzz` job in `.github/workflows/ci.yml`),
and uploads any crashing input as an artifact: a crash whose input is gone is a crash nobody can
reproduce.

## The readers added since, run 2026-10-03

Eight targets for what the first soak did not reach - the parsers written after it - each 45 s on
the same box, nightly + cargo-fuzz 0.13.2, from the seeds in `fuzz/seeds/<target>` (copies of the
test data: `edge.bin` and the head of `dataflash.bin`, the synthetic and example logs, `route.kmz`
and an SRTM tile, the fence, rally and polygon files, the three `.dbf`, a hand-written HEX, the
peripheral manifest, a legacy list and a GitHub listing, SLCAN lines).

| target | executions | exec/s | edges | peak RSS |
|---|---:|---:|---:|---:|
| `dataflash_bin` | 4,890 | 106 | 1,226 | 598 MB |
| `loganalyzer_text` | 10,414 | 226 | 2,965 | 489 MB |
| `zip_archive` | 876 | 19 | 1,303 | 330 MB |
| `fence_file` | 480,612 | 10,448 | 502 | 523 MB |
| `dbf_table` | 2,329 | - | - | out of memory, see below |
| `intel_hex` | 1,002,892 | 21,802 | 212 | 76 MB |
| `catalogue_text` | 1,700,748 | 36,972 | 2,871 | 556 MB |
| `dronecan_slcan` | 690,594 | 15,012 | 753 | 570 MB |

`dataflash_bin` and `zip_archive` are slow by nature - each execution converts a whole log to
text, or inflates and deflates an archive - and `loganalyzer_text` runs seventeen checks over
every input; the seconds are spent in the code under test, which is where they should be.

**`dbf_table` found something in its first two seconds.** An 86-byte table whose header claimed
989 million records (libFuzzer's `oom-df8062b3...`, kept as the unit test) made `read` allocate for the count
before reading a record: 23 GB for a file of 86 bytes. The reader now sizes its allocation by what
the bytes can hold and fails at the first missing record (`crates/mp-mission/src/dbf.rs`, with the
input as a unit test); rerun for 60 s, 1,478,379 executions, clean.

CI runs each of the eight a bounded slice beside the five above.
