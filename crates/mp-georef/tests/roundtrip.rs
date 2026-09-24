//! `ToString("R")` - what the positions' XML beside the log is written with - held to 5,000
//! numbers the oracle formatted under mono (`testdata/georef/golden/roundtrip.txt`): log-shaped
//! latitudes and altitudes, random doubles of every magnitude, and floats.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

mod common;

use mp_georef::numfmt::{double_roundtrip, single_roundtrip};

#[test]
fn round_trip_formats_match_mono() {
    let text = std::fs::read_to_string(common::data().join("golden/roundtrip.txt")).unwrap();
    let mut failures = Vec::new();
    let mut count = 0;
    for line in text.lines() {
        let parts: Vec<&str> = line.split(' ').collect();
        let (got, want) = if parts[0] == "F" {
            let bits = u32::from_str_radix(parts[1], 16).unwrap();
            (single_roundtrip(f32::from_bits(bits)), parts[2])
        } else {
            let bits = u64::from_str_radix(parts[0], 16).unwrap();
            (double_roundtrip(f64::from_bits(bits)), parts[1])
        };
        count += 1;
        if got != want {
            failures.push(format!("{line}: got {got}"));
        }
    }
    assert_eq!(count, 5000);
    assert!(
        failures.is_empty(),
        "{} of 5000 differ:\n{}",
        failures.len(),
        failures.join("\n")
    );
}
