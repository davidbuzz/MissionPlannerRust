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

//! Frame layout, parsing and encoding for MAVLink v1 and v2.
//!
//! Replaces `ExtLibs/Mavlink/MavlinkParse.cs`. Where the C# version allocates a `byte[]` per
//! packet and throws on malformed input, this version borrows the caller's buffer and returns
//! errors the decoder can resynchronise from.

use crate::crc;
use crate::dialect::Dialect;

/// Start-of-frame byte for MAVLink v1.
pub const STX_V1: u8 = 0xFE;
/// Start-of-frame byte for MAVLink v2.
pub const STX_V2: u8 = 0xFD;
/// Incompatibility flag marking a signed v2 frame. A parser that does not understand a set
/// incompatibility flag must drop the frame, per the MAVLink specification.
pub const INCOMPAT_FLAG_SIGNED: u8 = 0x01;
/// Maximum payload length the wire format can express.
pub const MAX_PAYLOAD_LEN: usize = 255;
/// Length of a v2 signature block: link id (1) + timestamp (6) + truncated HMAC (6).
pub const SIGNATURE_LEN: usize = 13;
/// v1 header: STX, len, seq, sysid, compid, msgid.
pub const V1_HEADER_LEN: usize = 6;
/// v2 header: STX, len, incompat, compat, seq, sysid, compid, msgid[3].
pub const V2_HEADER_LEN: usize = 10;
/// Trailing CRC length.
pub const CHECKSUM_LEN: usize = 2;
/// Largest frame the wire format can express (v2, full payload, signed).
pub const MAX_FRAME_LEN: usize = V2_HEADER_LEN + MAX_PAYLOAD_LEN + CHECKSUM_LEN + SIGNATURE_LEN;

/// Which framing a frame uses.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum MavVersion {
    /// Legacy 6-byte header, 8-bit message ids.
    V1,
    /// 10-byte header, 24-bit message ids, optional signing.
    V2,
}

impl MavVersion {
    /// Header length for this framing.
    #[must_use]
    pub const fn header_len(self) -> usize {
        match self {
            Self::V1 => V1_HEADER_LEN,
            Self::V2 => V2_HEADER_LEN,
        }
    }
}

/// A parsed frame borrowing the input buffer. Copying a `Frame` copies only the descriptor.
#[derive(Debug, Clone, Copy)]
pub struct Frame<'a> {
    /// Framing version.
    pub version: MavVersion,
    /// v2 incompatibility flags (always 0 for v1).
    pub incompat_flags: u8,
    /// v2 compatibility flags (always 0 for v1).
    pub compat_flags: u8,
    /// Per-link sequence number, used for loss detection.
    pub seq: u8,
    /// Sending system id.
    pub sysid: u8,
    /// Sending component id.
    pub compid: u8,
    /// Message id (24-bit on v2, 8-bit on v1).
    pub msgid: u32,
    /// Payload bytes exactly as they appeared on the wire, still truncated for v2.
    pub payload: &'a [u8],
    /// Checksum as transmitted (already verified by [`parse`]).
    pub checksum: u16,
    /// Signature block for signed v2 frames.
    pub signature: Option<&'a [u8]>,
    /// The complete frame, STX through signature.
    pub raw: &'a [u8],
}

impl Frame<'_> {
    /// Whether this frame carries a signature block.
    #[must_use]
    pub const fn is_signed(&self) -> bool {
        self.signature.is_some()
    }

    /// Reads a payload field, zero-extending past a v2-truncated payload.
    ///
    /// MAVLink2 strips trailing zero bytes, so a receiver must treat missing tail bytes as zero.
    /// Getting this wrong is the classic source of "the field reads 0 sometimes" bugs.
    #[must_use]
    pub fn payload_byte(&self, index: usize) -> u8 {
        self.payload.get(index).copied().unwrap_or(0)
    }

    /// Copies the payload into `out`, zero-extending to `out.len()`.
    pub fn payload_into(&self, out: &mut [u8]) {
        let n = self.payload.len().min(out.len());
        if let (Some(dst), Some(src)) = (out.get_mut(..n), self.payload.get(..n)) {
            dst.copy_from_slice(src);
        }
        if let Some(tail) = out.get_mut(n..) {
            tail.fill(0);
        }
    }

    /// The bytes a signature is computed over: the frame from `len` through the checksum.
    #[must_use]
    pub fn signable_bytes(&self) -> &[u8] {
        let end = self.version.header_len() + self.payload.len() + CHECKSUM_LEN;
        self.raw.get(..end).unwrap_or(self.raw)
    }
}

/// Why a byte sequence is not a frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum ParseError {
    /// Not enough bytes yet; `needed` more would let the parser make progress.
    #[error("incomplete frame, need {needed} more byte(s)")]
    Incomplete {
        /// Additional bytes required before parsing can continue.
        needed: usize,
    },
    /// First byte is not a known start-of-frame marker.
    #[error("invalid start byte {0:#04x}")]
    BadStx(u8),
    /// The dialect does not know this message, so its CRC seed is unavailable.
    #[error("unknown message id {msgid}")]
    UnknownMessage {
        /// The unrecognised message id.
        msgid: u32,
    },
    /// Checksum mismatch: corruption, wrong dialect, or a false-positive STX during resync.
    #[error("checksum mismatch: frame says {expected:#06x}, computed {actual:#06x}")]
    Crc {
        /// Checksum carried by the frame.
        expected: u16,
        /// Checksum computed over the received bytes.
        actual: u16,
    },
    /// An incompatibility flag we do not implement is set; the spec requires dropping the frame.
    #[error("unsupported incompatibility flags {0:#04x}")]
    UnsupportedIncompatFlags(u8),
}

/// Why a frame could not be encoded.
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum EncodeError {
    /// Output buffer too small.
    #[error("output buffer too small: need {needed} bytes, have {have}")]
    BufferTooSmall {
        /// Bytes required.
        needed: usize,
        /// Bytes available.
        have: usize,
    },
    /// Payload exceeds the 255-byte wire limit.
    #[error("payload of {0} bytes exceeds the 255 byte limit")]
    PayloadTooLong(usize),
    /// MAVLink v1 cannot express message ids above 255.
    #[error("message id {0} does not fit in a v1 frame")]
    MsgIdTooLargeForV1(u32),
}

/// Parses one frame from the front of `input`.
///
/// Returns the frame and the number of bytes consumed. The frame borrows `input`, so no payload
/// copy occurs. On [`ParseError::Incomplete`] the caller should read more bytes and retry with a
/// longer slice; on any other error the caller should skip one byte and resynchronise.
#[allow(clippy::indexing_slicing)] // every index below is guarded by an explicit length check
pub fn parse<'a, D>(input: &'a [u8], dialect: &D) -> Result<(Frame<'a>, usize), ParseError>
where
    D: Dialect + ?Sized,
{
    let stx = *input.first().ok_or(ParseError::Incomplete { needed: 1 })?;
    let version = match stx {
        STX_V1 => MavVersion::V1,
        STX_V2 => MavVersion::V2,
        other => return Err(ParseError::BadStx(other)),
    };

    let header_len = version.header_len();
    if input.len() < header_len {
        return Err(ParseError::Incomplete {
            needed: header_len - input.len(),
        });
    }

    let payload_len = input[1] as usize;
    let (incompat_flags, compat_flags, seq, sysid, compid, msgid) = match version {
        MavVersion::V1 => (0, 0, input[2], input[3], input[4], u32::from(input[5])),
        MavVersion::V2 => (
            input[2],
            input[3],
            input[4],
            input[5],
            input[6],
            u32::from_le_bytes([input[7], input[8], input[9], 0]),
        ),
    };

    let signed = incompat_flags & INCOMPAT_FLAG_SIGNED != 0;
    let unknown_flags = incompat_flags & !INCOMPAT_FLAG_SIGNED;
    let sig_len = if signed { SIGNATURE_LEN } else { 0 };
    let total = header_len + payload_len + CHECKSUM_LEN + sig_len;
    if input.len() < total {
        return Err(ParseError::Incomplete {
            needed: total - input.len(),
        });
    }

    // Length is known, so the whole frame can be skipped even though we refuse to interpret it.
    if unknown_flags != 0 {
        return Err(ParseError::UnsupportedIncompatFlags(unknown_flags));
    }

    let payload_end = header_len + payload_len;
    let payload = &input[header_len..payload_end];
    let checksum = u16::from_le_bytes([input[payload_end], input[payload_end + 1]]);

    let crc_extra = dialect
        .crc_extra(msgid)
        .ok_or(ParseError::UnknownMessage { msgid })?;
    let actual = crc::checksum(&input[1..payload_end], crc_extra);
    if actual != checksum {
        return Err(ParseError::Crc {
            expected: checksum,
            actual,
        });
    }

    let signature = if signed {
        Some(&input[payload_end + CHECKSUM_LEN..total])
    } else {
        None
    };

    Ok((
        Frame {
            version,
            incompat_flags,
            compat_flags,
            seq,
            sysid,
            compid,
            msgid,
            payload,
            checksum,
            signature,
            raw: &input[..total],
        },
        total,
    ))
}

/// Trims trailing zero bytes the way MAVLink v2 requires, always keeping at least one byte.
///
/// Matches the reference C implementation's `_mav_trim_payload`.
#[must_use]
pub fn trim_payload(payload: &[u8]) -> &[u8] {
    let mut len = payload.len();
    while len > 1 && payload.get(len - 1) == Some(&0) {
        len -= 1;
    }
    payload.get(..len).unwrap_or(payload)
}

/// Encodes a MAVLink v2 frame into `out`, returning its length. Does not allocate.
// The argument list mirrors the wire header one-for-one, which is clearer here than a builder.
// D4's link engine will wrap this in a typed sender that carries seq/sysid/compid itself.
#[allow(clippy::too_many_arguments)]
#[allow(clippy::indexing_slicing)] // guarded by the BufferTooSmall check below
pub fn encode_v2(
    out: &mut [u8],
    seq: u8,
    sysid: u8,
    compid: u8,
    msgid: u32,
    payload: &[u8],
    crc_extra: u8,
    compat_flags: u8,
) -> Result<usize, EncodeError> {
    if payload.len() > MAX_PAYLOAD_LEN {
        return Err(EncodeError::PayloadTooLong(payload.len()));
    }
    let trimmed = trim_payload(payload);
    let total = V2_HEADER_LEN + trimmed.len() + CHECKSUM_LEN;
    if out.len() < total {
        return Err(EncodeError::BufferTooSmall {
            needed: total,
            have: out.len(),
        });
    }

    let len =
        u8::try_from(trimmed.len()).map_err(|_| EncodeError::PayloadTooLong(trimmed.len()))?;
    let id = msgid.to_le_bytes();
    out[0] = STX_V2;
    out[1] = len;
    out[2] = 0; // incompat flags; signing is applied by a later pass
    out[3] = compat_flags;
    out[4] = seq;
    out[5] = sysid;
    out[6] = compid;
    out[7] = id[0];
    out[8] = id[1];
    out[9] = id[2];
    out[V2_HEADER_LEN..V2_HEADER_LEN + trimmed.len()].copy_from_slice(trimmed);

    let payload_end = V2_HEADER_LEN + trimmed.len();
    let ck = crc::checksum(&out[1..payload_end], crc_extra);
    out[payload_end..payload_end + CHECKSUM_LEN].copy_from_slice(&ck.to_le_bytes());
    Ok(total)
}

/// Encodes a MAVLink v1 frame into `out`, returning its length. Does not allocate.
#[allow(clippy::indexing_slicing)] // guarded by the BufferTooSmall check below
pub fn encode_v1(
    out: &mut [u8],
    seq: u8,
    sysid: u8,
    compid: u8,
    msgid: u32,
    payload: &[u8],
    crc_extra: u8,
) -> Result<usize, EncodeError> {
    if payload.len() > MAX_PAYLOAD_LEN {
        return Err(EncodeError::PayloadTooLong(payload.len()));
    }
    if msgid > u32::from(u8::MAX) {
        return Err(EncodeError::MsgIdTooLargeForV1(msgid));
    }
    let total = V1_HEADER_LEN + payload.len() + CHECKSUM_LEN;
    if out.len() < total {
        return Err(EncodeError::BufferTooSmall {
            needed: total,
            have: out.len(),
        });
    }

    let len =
        u8::try_from(payload.len()).map_err(|_| EncodeError::PayloadTooLong(payload.len()))?;
    out[0] = STX_V1;
    out[1] = len;
    out[2] = seq;
    out[3] = sysid;
    out[4] = compid;
    out[5] = u8::try_from(msgid).map_err(|_| EncodeError::MsgIdTooLargeForV1(msgid))?;
    out[V1_HEADER_LEN..V1_HEADER_LEN + payload.len()].copy_from_slice(payload);

    let payload_end = V1_HEADER_LEN + payload.len();
    let ck = crc::checksum(&out[1..payload_end], crc_extra);
    out[payload_end..payload_end + CHECKSUM_LEN].copy_from_slice(&ck.to_le_bytes());
    Ok(total)
}
