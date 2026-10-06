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
//! Replaces `ExtLibs/Mavlink/MavlinkParse.cs` and `MavlinkHeader.cs`. Where the C# version
//! allocates a `byte[]` per packet and throws on malformed input, this version borrows the caller's
//! buffer and returns errors the decoder can resynchronise from.
//!
//! A v2 header is as long as its incompatibility flags make it: `SYSID32` widens the sender's
//! system id to four bytes, `TARGET32` adds the target system, four bytes, after the message id
//! (tridge's 32-bit system ids, Mission Planner e6454ccdd). `// C#: ExtLibs/Mavlink/MavlinkHeader.cs`

use crate::crc;
use crate::dialect::Dialect;

/// Start-of-frame byte for MAVLink v1.
pub const STX_V1: u8 = 0xFE;
/// Start-of-frame byte for MAVLink v2.
pub const STX_V2: u8 = 0xFD;
/// Incompatibility flag marking a signed v2 frame. A parser that does not understand a set
/// incompatibility flag must drop the frame, per the MAVLink specification.
pub const INCOMPAT_FLAG_SIGNED: u8 = 0x01;
/// Incompatibility flag marking a v2 frame whose sender's system id is four bytes wide.
/// `// C#: ExtLibs/Mavlink/Mavlink.cs:401`
pub const INCOMPAT_FLAG_SYSID32: u8 = 0x02;
/// Incompatibility flag marking a v2 frame that carries its target system, four bytes, in the
/// header after the message id. `// C#: ExtLibs/Mavlink/Mavlink.cs:402`
pub const INCOMPAT_FLAG_TARGET32: u8 = 0x04;
/// The incompatibility flags this parser implements; a frame with any other is dropped whole.
/// `// C#: ExtLibs/Mavlink/MavlinkHeader.cs:5`
pub const SUPPORTED_INCOMPAT_FLAGS: u8 =
    INCOMPAT_FLAG_SIGNED | INCOMPAT_FLAG_SYSID32 | INCOMPAT_FLAG_TARGET32;
/// Maximum payload length the wire format can express.
pub const MAX_PAYLOAD_LEN: usize = 255;
/// Length of a v2 signature block: link id (1) + timestamp (6) + truncated HMAC (6).
pub const SIGNATURE_LEN: usize = 13;
/// v1 header: STX, len, seq, sysid, compid, msgid.
pub const V1_HEADER_LEN: usize = 6;
/// v2 header without the wide fields: STX, len, incompat, compat, seq, sysid, compid, msgid[3].
pub const V2_HEADER_LEN: usize = 10;
/// v2 header at its longest: `SYSID32`'s three more bytes and `TARGET32`'s four.
pub const V2_MAX_HEADER_LEN: usize = V2_HEADER_LEN + 3 + 4;
/// Trailing CRC length.
pub const CHECKSUM_LEN: usize = 2;
/// Largest frame the wire format can express (v2, both wide fields, full payload, signed).
pub const MAX_FRAME_LEN: usize = V2_MAX_HEADER_LEN + MAX_PAYLOAD_LEN + CHECKSUM_LEN + SIGNATURE_LEN;

/// A v2 frame's header length and message id, wherever its flags put them: the id after a
/// four-byte sender with `SYSID32`. `None` for what is not the start of a v2 frame.
/// `// C#: ExtLibs/Mavlink/MAVLinkMessage.cs:217-230`
#[must_use]
pub fn v2_layout(frame: &[u8]) -> Option<(usize, u32)> {
    if frame.first() != Some(&STX_V2) {
        return None;
    }
    let flags = *frame.get(2)?;
    let at = if flags & INCOMPAT_FLAG_SYSID32 != 0 {
        10
    } else {
        7
    };
    let id = frame.get(at..at + 3)?;
    let msgid = u32::from_le_bytes([*id.first()?, *id.get(1)?, *id.get(2)?, 0]);
    Some((v2_header_len(flags), msgid))
}

/// A v2 header's length for its incompatibility flags. `// C#: ExtLibs/Mavlink/MavlinkHeader.cs:7-12`
#[must_use]
pub const fn v2_header_len(incompat_flags: u8) -> usize {
    V2_HEADER_LEN
        + if incompat_flags & INCOMPAT_FLAG_SYSID32 != 0 {
            3
        } else {
            0
        }
        + if incompat_flags & INCOMPAT_FLAG_TARGET32 != 0 {
            4
        } else {
            0
        }
}

/// Which framing a frame uses.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum MavVersion {
    /// Legacy 6-byte header, 8-bit message ids.
    V1,
    /// 10-byte header (more with `SYSID32` or `TARGET32`), 24-bit message ids, optional signing.
    V2,
}

impl MavVersion {
    /// Header length for this framing, without v2's wide fields.
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
    /// Sending system id: 32-bit with `SYSID32`, else the header's one byte.
    pub sysid: u32,
    /// Sending component id.
    pub compid: u8,
    /// Message id (24-bit on v2, 8-bit on v1).
    pub msgid: u32,
    /// The header's target system, with `TARGET32`; the payload's `target_system` byte otherwise.
    pub target_system: Option<u32>,
    /// The header's length, its wide fields included.
    pub header_len: usize,
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
        let end = self.header_len + self.payload.len() + CHECKSUM_LEN;
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
    /// An incompatibility flag we do not implement is set on a frame whose checksum passed; the
    /// spec requires dropping it. Its length is known all the same, and the frame is skipped
    /// whole, as the C#'s `ReadPacket` consumes it: a frame inside its payload is not one.
    /// `// C#: ExtLibs/Mavlink/MavlinkParse.cs:205-218`
    #[error("unsupported incompatibility flags {flags:#04x}")]
    UnsupportedIncompatFlags {
        /// The flags not implemented.
        flags: u8,
        /// The whole frame's length, to skip.
        len: usize,
    },
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

    // A v2 header's length is in its flags. `// C#: ExtLibs/Mavlink/MavlinkParse.cs:189-198`
    let header_len = match version {
        MavVersion::V1 => V1_HEADER_LEN,
        MavVersion::V2 => match input.get(2) {
            Some(&flags) => v2_header_len(flags),
            None => V2_HEADER_LEN,
        },
    };
    if input.len() < header_len {
        return Err(ParseError::Incomplete {
            needed: header_len - input.len(),
        });
    }

    let payload_len = input[1] as usize;
    let read_u32 =
        |at: usize| u32::from_le_bytes([input[at], input[at + 1], input[at + 2], input[at + 3]]);
    // `processBuffer`'s reading. `// C#: ExtLibs/Mavlink/MAVLinkMessage.cs:189-245`
    let (incompat_flags, compat_flags, seq, sysid, compid, msgid, target_system) = match version {
        MavVersion::V1 => (
            0,
            0,
            input[2],
            u32::from(input[3]),
            input[4],
            u32::from(input[5]),
            None,
        ),
        MavVersion::V2 => {
            let flags = input[2];
            let (sysid, at) = if flags & INCOMPAT_FLAG_SYSID32 != 0 {
                (read_u32(5), 9)
            } else {
                (u32::from(input[5]), 6)
            };
            let msgid = u32::from_le_bytes([input[at + 1], input[at + 2], input[at + 3], 0]);
            let target = (flags & INCOMPAT_FLAG_TARGET32 != 0).then(|| read_u32(at + 4));
            (flags, input[3], input[4], sysid, input[at], msgid, target)
        }
    };

    let signed = incompat_flags & INCOMPAT_FLAG_SIGNED != 0;
    let unknown_flags = incompat_flags & !SUPPORTED_INCOMPAT_FLAGS;
    let sig_len = if signed { SIGNATURE_LEN } else { 0 };
    let total = header_len + payload_len + CHECKSUM_LEN + sig_len;
    if input.len() < total {
        return Err(ParseError::Incomplete {
            needed: total - input.len(),
        });
    }

    let payload_end = header_len + payload_len;
    let payload = &input[header_len..payload_end];
    let checksum = u16::from_le_bytes([input[payload_end], input[payload_end + 1]]);

    let crc_extra = dialect
        .crc_extra(msgid)
        .ok_or(ParseError::UnknownMessage { msgid })?;
    // Over the header, its wide fields included, and the payload. `// C#: MavlinkParse.cs:222`
    let actual = crc::checksum(&input[1..payload_end], crc_extra);
    if actual != checksum {
        return Err(ParseError::Crc {
            expected: checksum,
            actual,
        });
    }

    // A flag this parser does not implement: the frame is dropped whole, as the C# drops it
    // (`MavlinkParse.cs:217-218`) - a frame in its payload is not one. **Divergence:** only once
    // its checksum, read where the flags it knows put it, has passed. The C# consumes any
    // candidate's claimed length; this parser resynchronises a byte at a time past a checksum
    // failure, and noise shaped like a header has an unknown flag 31 times in 32 - skipped whole,
    // it would take the frames after it with it.
    if unknown_flags != 0 {
        return Err(ParseError::UnsupportedIncompatFlags {
            flags: unknown_flags,
            len: total,
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
            target_system,
            header_len,
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

/// Encodes a MAVLink v2 frame into `out`, returning its length. Does not allocate. A system id
/// over 255 is written four bytes wide, with `SYSID32`.
// The argument list mirrors the wire header one-for-one, which is clearer here than a builder.
// Deliverable 4's link engine will wrap this in a typed sender that carries seq/sysid/compid itself.
#[allow(clippy::too_many_arguments)]
pub fn encode_v2(
    out: &mut [u8],
    seq: u8,
    sysid: u32,
    compid: u8,
    msgid: u32,
    payload: &[u8],
    crc_extra: u8,
    compat_flags: u8,
) -> Result<usize, EncodeError> {
    encode_v2_targeted(
        out,
        seq,
        sysid,
        compid,
        msgid,
        payload,
        crc_extra,
        compat_flags,
        None,
    )
}

/// [`encode_v2`] with a target system: one over 255 goes in the header, four bytes after the
/// message id, with `TARGET32`; one that fits is the payload's own `target_system` byte, as is
/// 255 for a wider one (`SetPayloadTarget`), the caller's to write. Source and target widths are
/// independent. `// C#: ExtLibs/Mavlink/MavlinkParse.cs:287-388; MavlinkHeader.cs:45-82`
#[allow(clippy::too_many_arguments)]
#[allow(clippy::indexing_slicing)] // guarded by the BufferTooSmall check below
pub fn encode_v2_targeted(
    out: &mut [u8],
    seq: u8,
    sysid: u32,
    compid: u8,
    msgid: u32,
    payload: &[u8],
    crc_extra: u8,
    compat_flags: u8,
    target_system: Option<u32>,
) -> Result<usize, EncodeError> {
    if payload.len() > MAX_PAYLOAD_LEN {
        return Err(EncodeError::PayloadTooLong(payload.len()));
    }
    let target = target_system.filter(|&target| target > 255);
    let flags = if sysid > 255 {
        INCOMPAT_FLAG_SYSID32
    } else {
        0
    } | if target.is_some() {
        INCOMPAT_FLAG_TARGET32
    } else {
        0
    };
    let header_len = v2_header_len(flags);
    let trimmed = trim_payload(payload);
    let total = header_len + trimmed.len() + CHECKSUM_LEN;
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
    out[2] = flags; // the wide fields' flags; signing is applied by a later pass
    out[3] = compat_flags;
    out[4] = seq;
    let mut at = 5;
    if flags & INCOMPAT_FLAG_SYSID32 != 0 {
        out[at..at + 4].copy_from_slice(&sysid.to_le_bytes());
        at += 4;
    } else {
        // Under 256: one byte.
        #[allow(clippy::cast_possible_truncation)]
        let narrow = sysid as u8;
        out[at] = narrow;
        at += 1;
    }
    out[at] = compid;
    out[at + 1..at + 4].copy_from_slice(&id[..3]);
    at += 4;
    if let Some(target) = target {
        out[at..at + 4].copy_from_slice(&target.to_le_bytes());
    }
    out[header_len..header_len + trimmed.len()].copy_from_slice(trimmed);

    let payload_end = header_len + trimmed.len();
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
