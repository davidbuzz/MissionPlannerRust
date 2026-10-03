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

//! `@PARAM/param.pck`, the parameter file ArduPilot serves over MAVFTP: `parampck.unpack`, ported
//! from `ExtLibs/ArduPilot/parampck.cs` (GPL-3.0-only), and a `pack` for the tests and the
//! benches, which the C# has not got.
//!
//! The packed format, as the C# quotes it from ArduPilot's `AP_Filesystem_Param`:
//!
//! ```text
//! file header:
//!   uint16_t magic = 0x671b (0x671c when defaults are included)
//!   uint16_t num_params
//!   uint16_t total_params
//! per-parameter:
//!   uint8_t type:4;         // AP_Param type NONE=0, INT8=1, INT16=2, INT32=3, FLOAT=4
//!   uint8_t flags:4;        // bit 0: includes default value for this param
//!   uint8_t common_len:4;   // number of name bytes in common with previous entry, 0..15
//!   uint8_t name_len:4;     // non-common length of param name -1 (0..15)
//!   uint8_t name[name_len]; // name
//!   uint8_t data[];         // value, length given by variable type, data length doubled if
//!                           // default is included
//! ```
//!
//! Leading zero bytes before an entry are padding - ArduPilot pads so that a value never crosses
//! a read packet boundary - and are skipped. Mission Planner asks for the file as
//! `@PARAM/param.pck?withdefaults=1` (`MAVLinkInterface.cs:1877`), and ArduPilot answers with the
//! defaults where it has them, in the `0x671c` format, or the plain file where it has not.

use crate::{ParamType, ParamValue};

/// `parampck.magic`: the header's first two bytes of a file without defaults.
/// `// C#: ExtLibs/ArduPilot/parampck.cs:16`
pub const MAGIC: u16 = 0x671b;

/// `parampck.magic_with_defaults`: the header of a file whose entries may carry defaults.
/// `// C#: ExtLibs/ArduPilot/parampck.cs:17`
pub const MAGIC_WITH_DEFAULTS: u16 = 0x671c;

/// The header's three `uint16_t`s.
pub const HEADER_LEN: usize = 6;

/// One parameter as the file holds it.
#[derive(Debug, Clone, PartialEq)]
pub struct PackedParam {
    /// The name, the common prefix restored.
    pub name: String,
    /// The value, carrying the AP_Param type as its storage type - as the classic download holds
    /// ArduPilot's parameters ([`ParamValue::from_ardupilot`]), so the two paths fill one table
    /// alike.
    pub value: ParamValue,
    /// The default, when the file carries one for this parameter (`withdefaults=1` and the
    /// vehicle knows it); the value itself when the file carries defaults but flags this entry as
    /// being at its default, as `parampck.unpack` sets `default_value` then.
    pub default: Option<f64>,
}

/// Where `unpack` returns null. The C# says nothing about which; these say.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum PckError {
    /// Fewer than six bytes.
    #[error("param.pck is {0} bytes, shorter than its header")]
    Short(usize),
    /// Neither magic.
    #[error("param.pck magic {0:#06x} is neither 0x671b nor 0x671c")]
    BadMagic(u16),
    /// An entry's type is not INT8, INT16, INT32 or FLOAT.
    #[error("param.pck entry {index} has type {kind}, which is not a parameter type")]
    BadType {
        /// Which entry, counting from 0.
        index: usize,
        /// The type nibble.
        kind: u8,
    },
    /// An entry runs past the end of the data.
    #[error("param.pck entry {0} runs past the end of the file")]
    Truncated(usize),
    /// `common_len` reaches past the previous name.
    #[error("param.pck entry {0} shares more of the previous name than there is")]
    BadCommon(usize),
    /// The header's `num_params` is not the number of entries, or exceeds `total_params`.
    #[error("param.pck header says {header} of {total} parameters, the file holds {found}")]
    BadCount {
        /// `num_params`.
        header: u16,
        /// `total_params`.
        total: u16,
        /// The entries counted.
        found: usize,
    },
}

/// The AP_Param type nibble as a [`ParamType`], and its width.
/// `// C#: ExtLibs/ArduPilot/parampck.cs:35-41`
const fn ap_type(kind: u8) -> Option<(ParamType, usize)> {
    match kind {
        1 => Some((ParamType::Int8, 1)),
        2 => Some((ParamType::Int16, 2)),
        3 => Some((ParamType::Int32, 4)),
        4 => Some((ParamType::Real32, 4)),
        _ => None,
    }
}

/// The nibble for a type, for `pack`; unsigned types are stored as their signed width, as
/// AP_Param has no unsigned kinds.
const fn ap_nibble(kind: ParamType) -> u8 {
    match kind {
        ParamType::Uint8 | ParamType::Int8 => 1,
        ParamType::Uint16 | ParamType::Int16 => 2,
        ParamType::Uint32 | ParamType::Int32 => 3,
        ParamType::Real32 => 4,
    }
}

/// `decode_value`: the little-endian value of `width` bytes at `at`, as an `f64`.
/// `// C#: ExtLibs/ArduPilot/parampck.cs:135-148`
fn decode_value(kind: ParamType, bytes: &[u8]) -> Option<f64> {
    Some(match kind {
        ParamType::Int8 => f64::from(i8::from_le_bytes([*bytes.first()?])),
        ParamType::Int16 => f64::from(i16::from_le_bytes([*bytes.first()?, *bytes.get(1)?])),
        ParamType::Int32 => f64::from(i32::from_le_bytes([
            *bytes.first()?,
            *bytes.get(1)?,
            *bytes.get(2)?,
            *bytes.get(3)?,
        ])),
        ParamType::Real32 => f64::from(f32::from_le_bytes([
            *bytes.first()?,
            *bytes.get(1)?,
            *bytes.get(2)?,
            *bytes.get(3)?,
        ])),
        ParamType::Uint8 | ParamType::Uint16 | ParamType::Uint32 => return None,
    })
}

/// `parampck.unpack`: the file's parameters, in the file's order.
///
/// # Errors
///
/// Where the C# returns null: a short file, a wrong magic, an unknown type, and a count that does
/// not match; and where it would throw, an entry running off the end.
/// `// C#: ExtLibs/ArduPilot/parampck.cs:43-132`
pub fn unpack(data: &[u8]) -> Result<Vec<PackedParam>, PckError> {
    if data.len() < HEADER_LEN {
        return Err(PckError::Short(data.len()));
    }
    let word = |at: usize| {
        u16::from_le_bytes([
            data.get(at).copied().unwrap_or(0),
            data.get(at + 1).copied().unwrap_or(0),
        ])
    };
    let magic = word(0);
    let num_params = word(2);
    let total_params = word(4);
    let with_defaults = match magic {
        MAGIC => false,
        MAGIC_WITH_DEFAULTS => true,
        other => return Err(PckError::BadMagic(other)),
    };

    let mut at = HEADER_LEN;
    let mut list = Vec::new();
    let mut last_name = String::new();
    loop {
        // Pad bytes: zeros between entries.
        while data.get(at) == Some(&0) {
            at += 1;
        }
        let index = list.len();
        let (Some(&ptype), Some(&plen)) = (data.get(at), data.get(at + 1)) else {
            if at >= data.len() {
                break;
            }
            return Err(PckError::Truncated(index));
        };
        at += 2;
        let flags = (ptype >> 4) & 0x0f;
        let kind_nibble = ptype & 0x0f;
        let Some((kind, width)) = ap_type(kind_nibble) else {
            return Err(PckError::BadType {
                index,
                kind: kind_nibble,
            });
        };
        let name_len = usize::from((plen >> 4) & 0x0f) + 1;
        let common_len = usize::from(plen & 0x0f);
        let Some(common) = last_name.get(..common_len) else {
            return Err(PckError::BadCommon(index));
        };
        let Some(tail) = data.get(at..at + name_len) else {
            return Err(PckError::Truncated(index));
        };
        // `(char)data[i]`: each byte as a character, as the C# builds the name.
        let mut name = common.to_owned();
        name.extend(tail.iter().map(|&byte| char::from(byte)));
        at += name_len;

        let Some(value_bytes) = data.get(at..at + width) else {
            return Err(PckError::Truncated(index));
        };
        let Some(value) = decode_value(kind, value_bytes) else {
            return Err(PckError::BadType {
                index,
                kind: kind_nibble,
            });
        };
        at += width;

        let default = if with_defaults {
            if flags & 1 == 0 {
                Some(value)
            } else {
                let Some(default_bytes) = data.get(at..at + width) else {
                    return Err(PckError::Truncated(index));
                };
                at += width;
                decode_value(kind, default_bytes)
            }
        } else {
            None
        };

        last_name.clone_from(&name);
        list.push(PackedParam {
            name,
            // The classic download's form: ArduPilot's float, with the storage type kept.
            #[allow(clippy::cast_possible_truncation)]
            value: ParamValue::from_ardupilot(value as f32, kind),
            default,
        });
    }

    if list.len() != usize::from(num_params) || list.len() > usize::from(total_params) {
        return Err(PckError::BadCount {
            header: num_params,
            total: total_params,
            found: list.len(),
        });
    }
    Ok(list)
}

/// The inverse of [`unpack`], for tests and benches: the entries in the order given, each name's
/// common prefix with the one before it factored out, `with_defaults` choosing the `0x671c` form
/// (an entry whose default differs from its value carries it; one whose default is `None` or
/// equal is flagged as at its default). `total` is the header's `total_params`, `None` for the
/// number of entries. Not the C#'s: Mission Planner never writes this file.
///
/// A name longer than sixteen characters past the common prefix, or a value of an unsigned type,
/// cannot be packed and is written as the format allows - the name cut to sixteen, the value as
/// its signed width - since the file has no way to say either.
#[must_use]
pub fn pack(entries: &[PackedParam], with_defaults: bool, total: Option<u16>) -> Vec<u8> {
    let mut out = Vec::new();
    let magic = if with_defaults {
        MAGIC_WITH_DEFAULTS
    } else {
        MAGIC
    };
    out.extend_from_slice(&magic.to_le_bytes());
    let count = u16::try_from(entries.len()).unwrap_or(u16::MAX);
    out.extend_from_slice(&count.to_le_bytes());
    out.extend_from_slice(&total.unwrap_or(count).to_le_bytes());
    let mut last_name = String::new();
    for entry in entries {
        let common = last_name
            .bytes()
            .zip(entry.name.bytes())
            .take_while(|(a, b)| a == b)
            .count()
            .min(15)
            .min(entry.name.len().saturating_sub(1));
        let tail = entry.name.as_bytes().get(common..).unwrap_or(&[]);
        let tail = tail.get(..tail.len().min(16)).unwrap_or(tail);
        let kind = entry.value.param_type();
        let nibble = ap_nibble(kind);
        let value = entry.value.as_f64();
        let default_differs = with_defaults && entry.default.is_some_and(|d| d != value);
        let flags = u8::from(default_differs);
        out.push((flags << 4) | nibble);
        let name_len = u8::try_from(tail.len() - 1).unwrap_or(15);
        let common_len = u8::try_from(common).unwrap_or(15);
        out.push((name_len << 4) | common_len);
        out.extend_from_slice(tail);
        out.extend_from_slice(&encode_value(nibble, value));
        if default_differs && let Some(default) = entry.default {
            out.extend_from_slice(&encode_value(nibble, default));
        }
        last_name.clone_from(&entry.name);
    }
    out
}

/// A value's bytes for its AP_Param type nibble.
#[allow(clippy::cast_possible_truncation)] // the float form is the file's
fn encode_value(nibble: u8, value: f64) -> Vec<u8> {
    match nibble {
        1 => vec![(value.clamp(-128.0, 127.0).round() as i8).to_le_bytes()[0]],
        2 => (value.clamp(-32_768.0, 32_767.0).round() as i16)
            .to_le_bytes()
            .to_vec(),
        3 => (value
            .clamp(f64::from(i32::MIN), f64::from(i32::MAX))
            .round() as i32)
            .to_le_bytes()
            .to_vec(),
        _ => (value as f32).to_le_bytes().to_vec(),
    }
}

#[cfg(test)]
mod tests {
    #![allow(
        clippy::unwrap_used,
        clippy::indexing_slicing,
        clippy::cast_possible_truncation
    )]

    use super::*;

    fn entry(name: &str, value: f64, kind: ParamType, default: Option<f64>) -> PackedParam {
        PackedParam {
            name: name.to_owned(),
            value: ParamValue::from_ardupilot(value as f32, kind),
            default,
        }
    }

    /// A file written by hand from the format's description: two entries, the second sharing
    /// "ACRO_" with the first, a pad byte between them, and no defaults.
    #[test]
    fn a_hand_packed_file_unpacks_as_the_format_says() {
        let mut data = vec![0x1b, 0x67, 2, 0, 5, 0];
        // ACRO_BAL_ROLL, float 1.0: type 4, flags 0; name_len 13-1=12, common 0.
        data.extend_from_slice(&[0x04, 0xc0]);
        data.extend_from_slice(b"ACRO_BAL_ROLL");
        data.extend_from_slice(&1.0f32.to_le_bytes());
        // Two pad bytes, as ArduPilot pads before a value that would cross a packet.
        data.extend_from_slice(&[0, 0]);
        // ACRO_RP_EXPO, int16 -30: common 5 ("ACRO_"), tail "RP_EXPO" (7, so name_len 6).
        data.extend_from_slice(&[0x02, 0x65]);
        data.extend_from_slice(b"RP_EXPO");
        data.extend_from_slice(&(-30i16).to_le_bytes());

        let list = unpack(&data).unwrap();
        assert_eq!(list.len(), 2);
        assert_eq!(list[0].name, "ACRO_BAL_ROLL");
        assert_eq!(list[0].value.as_f64(), 1.0);
        assert_eq!(list[0].value.param_type(), ParamType::Real32);
        assert_eq!(list[0].default, None);
        assert_eq!(list[1].name, "ACRO_RP_EXPO");
        assert_eq!(list[1].value.as_f64(), -30.0);
        assert_eq!(list[1].value.param_type(), ParamType::Int16);
    }

    /// `withdefaults=1`: the `0x671c` magic, an entry at its default carries none and reports its
    /// value as the default, an entry off its default carries the default after the value.
    #[test]
    fn defaults_follow_the_value_when_flagged() {
        let mut data = vec![0x1c, 0x67, 2, 0, 2, 0];
        // INS_GYRO_FILTER int8 20, flags 0: at its default.
        data.extend_from_slice(&[0x01, 0xe0]);
        data.extend_from_slice(b"INS_GYRO_FILTER");
        data.push(20);
        // MOT_THST_EXPO float 0.65, flags 1: default 0.5 follows.
        data.extend_from_slice(&[0x14, 0xc0]);
        data.extend_from_slice(b"MOT_THST_EXPO");
        data.extend_from_slice(&0.65f32.to_le_bytes());
        data.extend_from_slice(&0.5f32.to_le_bytes());

        let list = unpack(&data).unwrap();
        assert_eq!(list[0].default, Some(20.0));
        assert!((list[1].value.as_f64() - 0.65).abs() < 1e-6);
        assert_eq!(list[1].default, Some(0.5));
    }

    /// `pack` and `unpack` are inverses over a table like a copter's, defaults and all, the names
    /// sharing prefixes as ArduPilot's do.
    #[test]
    fn pack_then_unpack_is_the_identity() {
        let entries = vec![
            entry("ACRO_BAL_PITCH", 1.0, ParamType::Real32, Some(1.0)),
            entry("ACRO_BAL_ROLL", 1.5, ParamType::Real32, Some(1.0)),
            entry("ACRO_OPTIONS", 0.0, ParamType::Int8, Some(0.0)),
            entry(
                "ATC_ACCEL_P_MAX",
                110_000.0,
                ParamType::Real32,
                Some(110_000.0),
            ),
            entry("BATT_CAPACITY", 3300.0, ParamType::Int32, Some(3300.0)),
            entry("SERVO1_FUNCTION", 33.0, ParamType::Int16, Some(0.0)),
            entry("SIM_SPEEDUP", -1.0, ParamType::Real32, Some(-1.0)),
        ];
        let data = pack(&entries, true, None);
        assert_eq!(&data[..2], &MAGIC_WITH_DEFAULTS.to_le_bytes());
        assert_eq!(unpack(&data).unwrap(), entries);

        let plain: Vec<PackedParam> = entries
            .iter()
            .map(|e| PackedParam {
                default: None,
                ..e.clone()
            })
            .collect();
        let data = pack(&plain, false, Some(1408));
        assert_eq!(&data[..2], &MAGIC.to_le_bytes());
        assert_eq!(u16::from_le_bytes([data[4], data[5]]), 1408);
        assert_eq!(unpack(&data).unwrap(), plain);
    }

    /// Where the C# returns null: short, wrong magic, unknown type, wrong count.
    #[test]
    fn the_nulls_are_errors_that_say_why() {
        assert_eq!(unpack(&[0x1b, 0x67, 0]), Err(PckError::Short(3)));
        assert_eq!(
            unpack(&[0x3d, 0x76, 0, 0, 0, 0]),
            Err(PckError::BadMagic(0x763d))
        );
        let mut bad_type = vec![0x1b, 0x67, 1, 0, 1, 0, 0x05, 0x00, b'A', 0];
        assert_eq!(
            unpack(&bad_type),
            Err(PckError::BadType { index: 0, kind: 5 })
        );
        bad_type[6] = 0x01;
        // Header says two, the file holds one.
        bad_type[2] = 2;
        assert_eq!(
            unpack(&bad_type),
            Err(PckError::BadCount {
                header: 2,
                total: 1,
                found: 1
            })
        );
        // An entry cut short.
        let cut = vec![0x1b, 0x67, 1, 0, 1, 0, 0x04, 0x10, b'A', b'B', 0, 0];
        assert_eq!(unpack(&cut), Err(PckError::Truncated(0)));
        // An empty file with a header is no parameters, as `num_params` 0 says.
        assert_eq!(unpack(&[0x1b, 0x67, 0, 0, 0, 0]), Ok(Vec::new()));
    }
}
