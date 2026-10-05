// Copyright (C) 2026 David "Buzz" Bussenschutt
//
// This file is part of MissionPlannerRust, a Rust implementation derived from
// Mission Planner (Copyright (C) 2010-2024 Michael Oborne and contributors,
// https://github.com/ArduPilot/MissionPlanner); NOTICE records the changes.
//
// MissionPlannerRust is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by the
// Free Software Foundation, version 3 of the License.
//
// MissionPlannerRust is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
// or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
// more details.
//
// You should have received a copy of the GNU General Public License along with
// MissionPlannerRust. If not, see <https://www.gnu.org/licenses/>.
//
// SPDX-License-Identifier: GPL-3.0-only

//! The three small codecs the network transports need, written here rather than pulled in.
//!
//! A websocket handshake proves itself with `base64(SHA-1(key + GUID))` (RFC 6455 section 4.2.2),
//! a client frame is masked with four unpredictable bytes (section 5.3), and NTRIP's
//! `Authorization: Basic` header is base64. Each is a few dozen lines, and the crate stays free of
//! dependencies for them.

use std::collections::hash_map::RandomState;
use std::hash::{BuildHasher, Hasher};
use std::sync::atomic::{AtomicU64, Ordering};
use web_time::{SystemTime, UNIX_EPOCH};

const BASE64: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

/// Standard base64 with padding, as .NET's `Convert.ToBase64String`.
pub(crate) fn base64(data: &[u8]) -> String {
    let mut out = String::with_capacity(data.len().div_ceil(3) * 4);
    for chunk in data.chunks(3) {
        let byte = |i: usize| u32::from(chunk.get(i).copied().unwrap_or(0));
        let group = (byte(0) << 16) | (byte(1) << 8) | byte(2);
        for (i, shift) in [18u32, 12, 6, 0].into_iter().enumerate() {
            if i <= chunk.len() {
                let sextet = (group >> shift) & 63;
                let symbol = BASE64.get(sextet as usize).copied().unwrap_or(b'=');
                out.push(char::from(symbol));
            } else {
                out.push('=');
            }
        }
    }
    out
}

/// SHA-1 (FIPS 180-4), which RFC 6455 names for `Sec-WebSocket-Accept`. Not for anything that
/// needs collision resistance; the handshake does not.
#[allow(clippy::indexing_slicing)] // `w` is [u32; 80] and every index below is inside 0..80
pub(crate) fn sha1(data: &[u8]) -> [u8; 20] {
    let mut h: [u32; 5] = [
        0x6745_2301,
        0xEFCD_AB89,
        0x98BA_DCFE,
        0x1032_5476,
        0xC3D2_E1F0,
    ];
    let bits = (data.len() as u64).wrapping_mul(8);
    let mut message = Vec::with_capacity(data.len() + 72);
    message.extend_from_slice(data);
    message.push(0x80);
    while message.len() % 64 != 56 {
        message.push(0);
    }
    message.extend_from_slice(&bits.to_be_bytes());

    for block in message.chunks_exact(64) {
        let mut w = [0u32; 80];
        for (slot, word) in w.iter_mut().zip(block.chunks_exact(4)) {
            *slot = word
                .iter()
                .fold(0u32, |acc, &byte| (acc << 8) | u32::from(byte));
        }
        for t in 16..80 {
            w[t] = (w[t - 3] ^ w[t - 8] ^ w[t - 14] ^ w[t - 16]).rotate_left(1);
        }
        let [mut a, mut b, mut c, mut d, mut e] = h;
        for (t, &word) in w.iter().enumerate() {
            let (f, k) = match t {
                0..=19 => ((b & c) | (!b & d), 0x5A82_7999),
                20..=39 => (b ^ c ^ d, 0x6ED9_EBA1),
                40..=59 => ((b & c) | (b & d) | (c & d), 0x8F1B_BCDC),
                _ => (b ^ c ^ d, 0xCA62_C1D6),
            };
            let next = a
                .rotate_left(5)
                .wrapping_add(f)
                .wrapping_add(e)
                .wrapping_add(k)
                .wrapping_add(word);
            e = d;
            d = c;
            c = b.rotate_left(30);
            b = a;
            a = next;
        }
        for (state, add) in h.iter_mut().zip([a, b, c, d, e]) {
            *state = state.wrapping_add(add);
        }
    }

    let mut out = [0u8; 20];
    for (bytes, word) in out.chunks_exact_mut(4).zip(h) {
        bytes.copy_from_slice(&word.to_be_bytes());
    }
    out
}

/// Sixty-four bits nobody on the network can predict.
///
/// Each `RandomState` is keyed from the operating system's randomness (once per thread, then
/// stepped), so SipHash over a counter and the clock under that secret key is unpredictable to a
/// peer. That is what RFC 6455 asks of a masking key and a `Sec-WebSocket-Key` - the mask exists to
/// stop a client being steered into writing chosen bytes on the wire, not to keep a secret - and it
/// needs no dependency.
pub(crate) fn random_u64() -> u64 {
    static COUNTER: AtomicU64 = AtomicU64::new(0);
    let mut hasher = RandomState::new().build_hasher();
    hasher.write_u64(COUNTER.fetch_add(1, Ordering::Relaxed));
    let nanos = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |elapsed| elapsed.as_nanos());
    hasher.write_u128(nanos);
    hasher.finish()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn hex(bytes: &[u8]) -> String {
        bytes.iter().map(|b| format!("{b:02x}")).collect()
    }

    #[test]
    fn base64_matches_rfc_4648s_vectors() {
        // RFC 4648 section 10.
        for (input, expected) in [
            ("", ""),
            ("f", "Zg=="),
            ("fo", "Zm8="),
            ("foo", "Zm9v"),
            ("foob", "Zm9vYg=="),
            ("fooba", "Zm9vYmE="),
            ("foobar", "Zm9vYmFy"),
        ] {
            assert_eq!(base64(input.as_bytes()), expected, "{input:?}");
        }
        assert_eq!(base64(&[0xFB, 0xFF, 0xBF]), "+/+/");
    }

    #[test]
    fn sha1_matches_fips_180s_vectors() {
        assert_eq!(
            hex(&sha1(b"abc")),
            "a9993e364706816aba3e25717850c26c9cd0d89d"
        );
        assert_eq!(hex(&sha1(b"")), "da39a3ee5e6b4b0d3255bfef95601890afd80709");
        assert_eq!(
            hex(&sha1(
                b"abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq"
            )),
            "84983e441c3bd26ebaae4aa1f95129e5e54670f1"
        );
        // A message that pads into a second block, and one of a million bytes.
        assert_eq!(
            hex(&sha1(&[b'a'; 64])),
            "0098ba824b5c16427bd7a1122a5a442a25ec644d"
        );
        assert_eq!(
            hex(&sha1(&vec![b'a'; 1_000_000])),
            "34aa973cd4c4daa4f61eeb2bdbad27316534016f"
        );
    }

    #[test]
    fn the_handshake_answer_is_rfc_6455s_worked_example() {
        // RFC 6455 section 1.3.
        let key = "dGhlIHNhbXBsZSBub25jZQ==";
        let accept = base64(&sha1(
            format!("{key}258EAFA5-E914-47DA-95CA-C5AB0DC85B11").as_bytes(),
        ));
        assert_eq!(accept, "s3pPLMBiTxaQ9kYGzzhZRbK+xOo=");
    }

    #[test]
    fn random_numbers_differ_from_call_to_call() {
        let drawn: std::collections::BTreeSet<u64> = (0..64).map(|_| random_u64()).collect();
        assert_eq!(drawn.len(), 64);
    }
}
