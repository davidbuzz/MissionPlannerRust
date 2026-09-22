//! Fuzzes single-frame parsing.
//!
//! The property: `parse` must never panic, never loop, and never report consuming more bytes than
//! it was given. Every byte on a telemetry link is attacker-influenced in the sense that matters -
//! radio noise, a misconfigured peer, a corrupted log - and a parser that panics takes the ground
//! station down mid-flight.

#![no_main]

use libfuzzer_sys::fuzz_target;
use mp_mavlink::parse;
use mp_mavlink_dialects::all::DIALECT;

fuzz_target!(|data: &[u8]| {
    if let Ok((frame, used)) = parse(data, &DIALECT) {
        assert!(used <= data.len(), "claimed to consume {used} of {} bytes", data.len());
        assert_eq!(frame.raw.len(), used, "raw slice must match the consumed length");
        assert!(frame.payload.len() <= 255);

        // Zero-extension must never read out of bounds, whatever the payload length claimed.
        let mut buffer = [0u8; 255];
        frame.payload_into(&mut buffer);
        let _ = frame.payload_byte(254);
        let _ = frame.signable_bytes();
    }
});
