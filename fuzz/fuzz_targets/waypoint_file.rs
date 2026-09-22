//! Fuzzes the waypoint file reader.
//!
//! Mission files come from other tools, other people and other decades. The reader must reject or
//! accept, never panic - and anything it accepts must survive a write-read round trip, or the file
//! a user saves will not match the one they loaded.

#![no_main]

use libfuzzer_sys::fuzz_target;
use mp_mission::{read_waypoints, write_waypoints};

fuzz_target!(|data: &[u8]| {
    let Ok(text) = std::str::from_utf8(data) else { return };
    let Ok(items) = read_waypoints(text) else { return };

    let written = write_waypoints(&items);
    let reread = read_waypoints(&written).expect("our own output must parse");
    assert_eq!(items.len(), reread.len(), "item count changed on round trip");

    // Writing must be a fixed point: the second write cannot differ from the first.
    assert_eq!(written, write_waypoints(&reread), "writing is not idempotent");
});
