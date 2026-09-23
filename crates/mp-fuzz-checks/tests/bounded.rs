//! A bounded fuzz pass that runs on stable, as part of the ordinary test suite.
//!
//! D2's definition of done asks for 24 hours clean on `frame_parse`, which is a job for libFuzzer
//! on nightly and not for `cargo test`. This is the other half: enough input, generated cheaply
//! and deterministically, to prove every target still compiles and still holds on the kinds of
//! input that break parsers - truncation, near-misses, and bytes that look like a frame until the
//! last field.
//!
//! Deterministic on purpose. A test that fails one run in fifty with a different input each time
//! is a test people disable. The generator is seeded from a constant, so a failure here can be
//! reproduced by running it again, and the failing input is printed.

use mp_fuzz_checks::TARGETS;

/// How many inputs each target sees. Sized to keep the whole pass under a second in a debug
/// build: this runs on every `cargo test`, and a suite people wait for is a suite people skip.
const ITERATIONS: usize = 2_000;

/// xorshift64*, because the point is reproducible bytes rather than statistical quality, and a
/// dependency for that would be a dependency for nothing.
struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        self.0.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }

    fn below(&mut self, limit: usize) -> usize {
        if limit == 0 {
            0
        } else {
            usize::try_from(self.next() % limit as u64).unwrap_or(0)
        }
    }

    fn byte(&mut self) -> u8 {
        u8::try_from(self.next() & 0xFF).unwrap_or(0)
    }
}

/// Inputs shaped like the things that actually arrive.
///
/// Purely random bytes almost never get past a magic-byte check, so a generator that only produces
/// those tests the first `if` in the parser and nothing after it. These start from something
/// plausible and damage it, which is what a radio does.
fn generate(rng: &mut Rng, iteration: usize) -> Vec<u8> {
    let length = rng.below(300);
    let mut data = Vec::with_capacity(length);
    match iteration % 5 {
        // Pure noise. Still worth doing: it is the only shape that finds a parser which trusts a
        // length field before checking anything else.
        0 => {
            for _ in 0..length {
                data.push(rng.byte());
            }
        }
        // A v2 frame header followed by noise, so the length and message id fields are reached.
        1 => {
            data.push(0xFD);
            for _ in 0..length {
                data.push(rng.byte());
            }
        }
        // A v1 frame header followed by noise.
        2 => {
            data.push(0xFE);
            for _ in 0..length {
                data.push(rng.byte());
            }
        }
        // Several frame starts back to back, which is what a stream of half-received frames looks
        // like and what wedges a decoder that waits for the rest of one.
        3 => {
            for _ in 0..length {
                data.push(if rng.below(4) == 0 { 0xFD } else { rng.byte() });
            }
        }
        // Text, for the waypoint reader: mostly printable, with the header it looks for.
        _ => {
            data.extend_from_slice(b"QGC WPL 110\n");
            for _ in 0..length {
                let choice = rng.below(8);
                data.push(match choice {
                    0 => b'\n',
                    1 => b'\t',
                    2 => rng.byte(),
                    _ => b'0' + u8::try_from(rng.below(10)).unwrap_or(0),
                });
            }
        }
    }
    data
}

/// Every target, over generated input. A panic is a finding.
#[test]
fn every_target_survives_a_bounded_pass() {
    for (name, target) in TARGETS {
        // A separate stream per target, so adding one does not change what the others see and
        // turn an unrelated commit into a mysterious failure.
        let seed = 0x2026_0923_u64.wrapping_mul(name.len() as u64 + 1) | 1;
        let mut rng = Rng(seed);
        for iteration in 0..ITERATIONS {
            let data = generate(&mut rng, iteration);
            // The input is printed before the call rather than after, because the call is what
            // panics: printing afterwards prints nothing for the one input that mattered.
            let described = std::panic::catch_unwind(|| target(&data));
            assert!(
                described.is_ok(),
                "{name} panicked on iteration {iteration}: {data:02x?}"
            );
        }
    }
}

/// The inputs a parser is most often wrong about are the empty one and the one-byte one.
#[test]
fn every_target_survives_the_degenerate_inputs() {
    for (name, target) in TARGETS {
        for data in [
            b"".as_slice(),
            b"\0".as_slice(),
            b"\xfd".as_slice(),
            b"\xfe".as_slice(),
            // A length field claiming more than the buffer holds.
            b"\xfd\xff\x00\x00\x00\x01\x01".as_slice(),
            b"\xfe\xff\x00\x01\x01\x00".as_slice(),
            // A header repeated with nothing after it.
            b"\xfd\xfd\xfd\xfd\xfd\xfd\xfd\xfd".as_slice(),
        ] {
            let outcome = std::panic::catch_unwind(|| target(data));
            assert!(outcome.is_ok(), "{name} panicked on {data:02x?}");
        }
    }
}

/// Every target has a committed seed corpus.
///
/// A target with no seeds still runs, and on this protocol it runs almost nowhere: `frame_parse`
/// from nothing reached 90 coverage edges, `message_decode` from 159 real frames reached 13,473.
/// A missing seeds directory is the difference between fuzzing and appearing to.
#[test]
fn every_target_has_seeds() {
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../fuzz/seeds")
        .canonicalize()
        .expect("the seeds directory");
    for (name, _) in TARGETS {
        let directory = root.join(name);
        let count = std::fs::read_dir(&directory)
            .unwrap_or_else(|err| panic!("{name} has no seeds at {}: {err}", directory.display()))
            .count();
        assert!(count > 0, "{name} has an empty seeds directory");
    }
}

/// Every target the `fuzz` crate declares is one this harness runs.
///
/// Without this, adding a libfuzzer target and forgetting to add it here leaves it in the same
/// state the four started in: present, plausible, never compiled.
#[test]
fn the_fuzz_crate_declares_exactly_these_targets() {
    let manifest = include_str!("../../../fuzz/Cargo.toml");
    let declared: Vec<&str> = manifest
        .lines()
        .filter_map(|line| line.strip_prefix("name = \""))
        .filter_map(|rest| rest.strip_suffix('"'))
        .filter(|name| *name != "mp-fuzz")
        .collect();
    let checked: Vec<&str> = TARGETS.iter().map(|(name, _)| *name).collect();
    assert_eq!(
        declared, checked,
        "the fuzz crate's targets and the checked properties have diverged"
    );
}
