//! The generated message table must match the table inside the shipping C# assembly.
//!
//! This runs on any clone: it compares checked-in generated code against checked-in reference
//! data, with no XML, no mono and no reference tree required. If someone regenerates the dialect
//! from a drifted definition set, this fails.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use std::collections::BTreeMap;

#[test]
fn generated_table_matches_the_shipped_csharp_table() {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../testdata/mavlink/binary_message_infos.csv"
    );
    let csv = std::fs::read_to_string(path).expect("reference table");

    let mut reference = BTreeMap::new();
    for line in csv.lines().skip(1) {
        let f: Vec<&str> = line.split(',').collect();
        let [id, name, crc, min_len, len] = f[..] else {
            continue;
        };
        reference.insert(
            id.parse::<u32>().unwrap(),
            (
                name.to_owned(),
                crc.parse::<u8>().unwrap(),
                min_len.parse::<u8>().unwrap(),
                len.parse::<u8>().unwrap(),
            ),
        );
    }
    assert!(reference.len() > 300, "reference table looks truncated");

    let mut checked = 0;
    let mut mismatches = Vec::new();
    for info in mp_mavlink_dialects::MESSAGES {
        let Some((name, crc, min_len, len)) = reference.get(&info.id) else {
            continue; // present in the XML but not in this build of the C# assembly
        };
        checked += 1;
        if info.name != name
            || info.crc_extra != *crc
            || info.min_len != *min_len
            || info.len != *len
        {
            mismatches.push(format!(
                "id {}: generated {} crc={} min={} len={} vs reference {name} crc={crc} min={min_len} len={len}",
                info.id, info.name, info.crc_extra, info.min_len, info.len
            ));
        }
    }

    assert!(
        mismatches.is_empty(),
        "generated table drifted:\n{}",
        mismatches.join("\n")
    );
    assert!(
        checked > 300,
        "expected to check the whole table, checked {checked}"
    );
}
