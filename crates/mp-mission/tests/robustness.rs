//! Deterministic robustness testing for the mission file reader.
//!
//! Mission files come from other tools, other people and other decades. The reader must reject or
//! accept, never panic, and anything it accepts must survive a write-read round trip - otherwise
//! the file a user saves is not the one they loaded.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]
// The generator narrows u64 to smaller types deliberately; it is producing test input, not
// interpreting vehicle data, so truncation is the intent rather than a hazard.
#![allow(clippy::cast_possible_truncation)]

use mp_mission::{read_waypoints, write_waypoints};

struct Rng(u64);

impl Rng {
    const fn new(seed: u64) -> Self {
        Self(seed)
    }

    fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }

    fn below(&mut self, limit: usize) -> usize {
        if limit == 0 {
            0
        } else {
            (self.next_u64() % limit as u64) as usize
        }
    }

    fn pick<'a>(&mut self, options: &[&'a str]) -> &'a str {
        options[self.below(options.len())]
    }
}

#[test]
fn hostile_waypoint_files_are_rejected_or_accepted_but_never_panic() {
    let mut rng = Rng::new(0xF00D_0001);

    // Tokens chosen to hit the edges: empty fields, absurd magnitudes, NaN spellings, negative
    // sequence numbers, unicode, and the separators the format allows.
    const TOKENS: &[&str] = &[
        "0",
        "1",
        "-1",
        "99",
        "65535",
        "70000",
        "-99999",
        "3.14159",
        "1e308",
        "-1e308",
        "NaN",
        "inf",
        "-inf",
        "",
        " ",
        "\t",
        ",",
        "abc",
        "16",
        "0.00000001",
        "9999999999999999999",
        "-35.363262",
        "149.165237",
        "٣",
        "🛩",
        "0x10",
    ];
    const SEPARATORS: &[&str] = &["\t", " ", ",", "  ", "\t\t"];

    for _ in 0..20_000 {
        let mut text = String::from("QGC WPL 110\n");
        for _ in 0..rng.below(8) {
            let fields = rng.below(15);
            for f in 0..fields {
                if f > 0 {
                    text.push_str(rng.pick(SEPARATORS));
                }
                text.push_str(rng.pick(TOKENS));
            }
            text.push('\n');
        }

        // Whatever comes back must be self-consistent.
        if let Ok(items) = read_waypoints(&text) {
            let written = write_waypoints(&items);
            let reread = read_waypoints(&written).expect("our own output must parse");
            assert_eq!(
                items.len(),
                reread.len(),
                "count changed on round trip for:\n{text}"
            );
            assert_eq!(
                written,
                write_waypoints(&reread),
                "writing is not idempotent for:\n{text}"
            );
        }
    }
}

#[test]
fn arbitrary_bytes_never_panic_the_reader() {
    let mut rng = Rng::new(0xF00D_0002);
    for _ in 0..20_000 {
        let len = rng.below(300);
        let mut bytes = Vec::with_capacity(len + 12);
        // Half the corpus carries a valid header, so the record parser is actually reached rather
        // than everything failing at the first line.
        if rng.below(2) == 0 {
            bytes.extend_from_slice(b"QGC WPL 110\n");
        }
        for _ in 0..len {
            bytes.push((rng.next_u64() >> 24) as u8);
        }
        if let Ok(text) = std::str::from_utf8(&bytes) {
            let _ = read_waypoints(text);
        }
    }
}

#[test]
fn a_line_of_only_separators_does_not_produce_an_item() {
    // Degenerate but real: files edited by hand end up with stray tabs on a line.
    for line in ["\t\t\t\t\t\t\t\t\t\t\t", "   ", ",,,,,,,,,,,", "\t ,\t ,\t"] {
        let text = format!("QGC WPL 110\n{line}\n");
        let items = read_waypoints(&text).expect("parses");
        assert!(items.is_empty(), "{line:?} produced {} items", items.len());
    }
}
