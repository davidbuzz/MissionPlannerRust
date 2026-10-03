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

//! MAVLink v2 signing.
//!
//! A signed frame appends link id (1 byte), a 48-bit timestamp and a 48-bit truncated
//! SHA-256 over `secret_key || frame-through-checksum || link_id || timestamp`.

use sha2::{Digest, Sha256};

use crate::frame::{Frame, SIGNATURE_LEN};

/// Unix seconds at the MAVLink signing epoch, 2015-01-01T00:00:00Z.
pub const SIGNING_EPOCH_UNIX_SECS: u64 = 1_420_070_400;

/// A 32-byte shared secret.
#[derive(Clone)]
pub struct SigningKey([u8; 32]);

impl SigningKey {
    /// Wraps raw key bytes.
    #[must_use]
    pub const fn new(bytes: [u8; 32]) -> Self {
        Self(bytes)
    }

    /// Derives a key from a passphrase, as Mission Planner's signing dialog does.
    #[must_use]
    pub fn from_passphrase(passphrase: &str) -> Self {
        let digest = Sha256::digest(passphrase.as_bytes());
        let mut key = [0u8; 32];
        key.copy_from_slice(&digest);
        Self(key)
    }

    /// Raw key bytes.
    #[must_use]
    pub const fn as_bytes(&self) -> &[u8; 32] {
        &self.0
    }
}

impl core::fmt::Debug for SigningKey {
    /// Never prints key material.
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str("SigningKey(<redacted>)")
    }
}

/// Converts Unix microseconds to the signing timestamp unit (10 µs since the 2015 epoch).
#[must_use]
pub const fn timestamp_from_unix_micros(unix_micros: u64) -> u64 {
    let epoch_micros = SIGNING_EPOCH_UNIX_SECS * 1_000_000;
    if unix_micros <= epoch_micros {
        0
    } else {
        (unix_micros - epoch_micros) / 10
    }
}

/// Builds the 13-byte signature block for a frame's signable bytes.
///
/// `signable` is the frame from the length byte through the checksum, i.e.
/// [`Frame::signable_bytes`].
#[must_use]
pub fn sign(key: &SigningKey, link_id: u8, timestamp: u64, signable: &[u8]) -> [u8; SIGNATURE_LEN] {
    let ts = timestamp.to_le_bytes();
    let ts6 = [ts[0], ts[1], ts[2], ts[3], ts[4], ts[5]];

    let mut hasher = Sha256::new();
    hasher.update(key.as_bytes());
    hasher.update(signable);
    hasher.update([link_id]);
    hasher.update(ts6);
    let digest = hasher.finalize();

    let mut out = [0u8; SIGNATURE_LEN];
    out[0] = link_id;
    out[1..7].copy_from_slice(&ts6);
    out[7..13].copy_from_slice(digest.get(..6).unwrap_or(&[0; 6]));
    out
}

/// Verifies a parsed frame's signature in constant time with respect to the digest bytes.
#[must_use]
pub fn verify(key: &SigningKey, frame: &Frame<'_>) -> bool {
    let Some(sig) = frame.signature else {
        return false;
    };
    if sig.len() != SIGNATURE_LEN {
        return false;
    }
    let Some(link_id) = sig.first().copied() else {
        return false;
    };
    let Some(ts_bytes) = sig.get(1..7) else {
        return false;
    };
    let mut ts = [0u8; 8];
    if let Some(dst) = ts.get_mut(..6) {
        dst.copy_from_slice(ts_bytes);
    }
    let timestamp = u64::from_le_bytes(ts);

    let expected = sign(key, link_id, timestamp, frame.signable_bytes());
    let Some(actual_tail) = sig.get(7..13) else {
        return false;
    };
    let Some(expected_tail) = expected.get(7..13) else {
        return false;
    };

    let mut diff = 0u8;
    for (a, b) in actual_tail.iter().zip(expected_tail) {
        diff |= a ^ b;
    }
    diff == 0
}

/// Extracts `(link_id, timestamp)` from a signature block.
#[must_use]
pub fn signature_meta(sig: &[u8]) -> Option<(u8, u64)> {
    let link_id = sig.first().copied()?;
    let ts_bytes = sig.get(1..7)?;
    let mut ts = [0u8; 8];
    ts.get_mut(..6)?.copy_from_slice(ts_bytes);
    Some((link_id, u64::from_le_bytes(ts)))
}
