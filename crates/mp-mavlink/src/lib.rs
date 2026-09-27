//! MAVLink v1/v2 frame codec.
//!
//! This crate owns the wire format only: framing, checksums and signing. Message *contents*
//! live in the generated dialect crate, so that this hot path stays small, auditable and
//! free of generated code.
//!
//! Design constraints, from `DELIVERABLES.md` D2:
//!
//! * **Zero-copy parse.** [`frame::parse`] borrows the caller's buffer; payloads are never copied.
//! * **Allocation-free.** Nothing in this crate allocates. [`decoder::FrameDecoder`] owns a
//!   fixed-size buffer, so a link cannot grow memory under load or attack.
//! * **No panics.** Every slice access is length-checked first; the fuzz targets assert this.
//!
//! The C# original this replaces is `ExtLibs/Mavlink/MavlinkParse.cs` and `MavlinkCRC.cs`.

pub mod crc;
pub mod decoder;
pub mod dialect;
pub mod field;
pub mod frame;
pub mod message;
pub mod payload;
pub mod signing;

pub use decoder::{DecodeStats, FrameDecoder};
pub use dialect::{Dialect, MessageInfo, StaticDialect};
pub use field::{FieldInfo, FieldValue};
pub use frame::{
    EncodeError, Frame, INCOMPAT_FLAG_SIGNED, MAX_FRAME_LEN, MAX_PAYLOAD_LEN, MavVersion,
    ParseError, SIGNATURE_LEN, STX_V1, STX_V2, encode_v1, encode_v2, parse, trim_payload,
};
pub use message::Message;
pub use signing::{SigningKey, sign, verify};
