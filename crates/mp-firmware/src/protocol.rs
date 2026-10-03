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

//! The wire protocol: command bytes, response bytes, and framing.
//!
//! `// C#: ExtLibs/px4uploader/Uploader.cs:46-92`

/// Protocol bytes, responses and commands together, as the bootloader defines them.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Code {
    /// Does nothing. Used to flush a confused bootloader.
    Nop = 0x00,
    /// The operation succeeded.
    Ok = 0x10,
    /// The operation was understood and failed.
    Failed = 0x11,
    /// Precedes every response; the bootloader is in sync with us.
    InSync = 0x12,
    /// The operation was not understood. Bootloader revision 3 and later.
    Invalid = 0x13,
    /// The firmware is not for this silicon revision.
    BadSiliconRev = 0x14,

    /// Ends every command.
    Eoc = 0x20,
    /// Asks for a sync response, to find out whether anything is listening.
    GetSync = 0x21,
    /// Reads one of the [`Info`] words.
    GetDevice = 0x22,
    /// Erases the whole program flash.
    ChipErase = 0x23,
    /// Enters verify mode. Bootloader revision 2 only.
    ChipVerify = 0x24,
    /// Writes a block at the program address and advances it.
    ProgMulti = 0x27,
    /// Reads a block back. Bootloader revision 2 only.
    ReadMulti = 0x28,
    /// Asks the bootloader for a CRC over the whole flash. Revision 3 and later.
    GetCrc = 0x29,
    /// Reads a byte of one-time-programmable memory.
    GetOtp = 0x2a,
    /// Reads a word of the unique device id.
    GetSn = 0x2b,
    /// Reads the MCU IDCODE.
    GetChip = 0x2c,
    /// Sets the minimum boot delay.
    SetDelay = 0x2d,
    /// Reads the chip description string.
    GetChipDes = 0x2e,
    /// Leaves the bootloader and starts the firmware.
    Reboot = 0x30,
    /// Bootloader debug hook.
    Debug = 0x31,
    /// Changes the serial baud rate.
    SetBaud = 0x33,

    /// Erases sectors of external flash.
    ExtfErase = 0x34,
    /// Writes a block to external flash and advances the address.
    ExtfProgMulti = 0x35,
    /// Reads a block back from external flash.
    ExtfReadMulti = 0x36,
    /// Asks for a CRC over external flash.
    ExtfGetCrc = 0x37,
}

impl Code {
    /// The byte this code is on the wire.
    #[must_use]
    pub const fn byte(self) -> u8 {
        self as u8
    }
}

/// The words `GET_DEVICE` can return.
///
/// `// C#: ExtLibs/px4uploader/Uploader.cs:79-86`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Info {
    /// Bootloader protocol revision.
    BootloaderRevision = 1,
    /// Board type, which must match the firmware's `board_id`.
    BoardId = 2,
    /// Board revision.
    BoardRevision = 3,
    /// Maximum firmware size in bytes.
    FlashSize = 4,
    /// Vector area.
    VectorArea = 5,
    /// External flash size.
    ExternalFlashSize = 6,
}

impl Info {
    /// The byte this word is selected by.
    #[must_use]
    pub const fn byte(self) -> u8 {
        self as u8
    }
}

/// The oldest bootloader protocol revision this understands.
///
/// `// C#: ExtLibs/px4uploader/Uploader.cs:88`
pub const BL_REV_MIN: u32 = 2;
/// The newest bootloader protocol revision this understands.
pub const BL_REV_MAX: u32 = 20;

/// The most bytes one `PROG_MULTI` may carry.
///
/// 252, not 255. The protocol maximum is 255 and the value must be a multiple of four, so 252 is
/// the largest usable. `// C#: ExtLibs/px4uploader/Uploader.cs:90`
pub const PROG_MULTI_MAX: usize = 252;
/// The most bytes one `READ_MULTI` may carry.
///
/// Also 252: "something overflows with >= 64" says the C#, and the constant it then uses is 252.
/// `// C#: ExtLibs/px4uploader/Uploader.cs:91`
pub const READ_MULTI_MAX: usize = 252;

/// Builds a bare command: the code, then end-of-command.
#[must_use]
pub fn command(code: Code) -> [u8; 2] {
    [code.byte(), Code::Eoc.byte()]
}

/// Builds a `GET_DEVICE` for one info word.
#[must_use]
pub fn get_info(info: Info) -> [u8; 3] {
    [Code::GetDevice.byte(), info.byte(), Code::Eoc.byte()]
}

/// Builds a `PROG_MULTI` carrying one block.
///
/// The length is a single byte, so a block longer than [`PROG_MULTI_MAX`] cannot be expressed;
/// callers split first. Returns `None` rather than truncating, because a silently short write
/// leaves a gap in the middle of the firmware.
#[must_use]
pub fn program_multi(data: &[u8]) -> Option<Vec<u8>> {
    if data.is_empty() || data.len() > PROG_MULTI_MAX {
        return None;
    }
    let mut out = Vec::with_capacity(data.len() + 3);
    out.push(Code::ProgMulti.byte());
    #[allow(clippy::cast_possible_truncation)] // bounded by PROG_MULTI_MAX above
    out.push(data.len() as u8);
    out.extend_from_slice(data);
    out.push(Code::Eoc.byte());
    Some(out)
}

/// Builds an `EXTF_PROG_MULTI` carrying one block.
#[must_use]
pub fn program_multi_external(data: &[u8]) -> Option<Vec<u8>> {
    let mut out = program_multi(data)?;
    // Same shape, different opcode. Built from the same function so the two cannot drift.
    *out.first_mut()? = Code::ExtfProgMulti.byte();
    Some(out)
}

/// Builds an `EXTF_GET_CRC` over a length of external flash.
///
/// `// C#: ExtLibs/px4uploader/Uploader.cs:765-772` - the length goes out little-endian between
/// the opcode and the end-of-command.
#[must_use]
pub fn external_crc(length: u32) -> [u8; 6] {
    let size = length.to_le_bytes();
    [
        Code::ExtfGetCrc.byte(),
        size[0],
        size[1],
        size[2],
        size[3],
        Code::Eoc.byte(),
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The command bytes are the bootloader's, and a wrong one is a board that does not boot.
    #[test]
    fn the_opcodes_are_the_ones_the_bootloader_defines() {
        // C#: ExtLibs/px4uploader/Uploader.cs:46-77, value for value.
        assert_eq!(Code::Nop.byte(), 0x00);
        assert_eq!(Code::Ok.byte(), 0x10);
        assert_eq!(Code::Failed.byte(), 0x11);
        assert_eq!(Code::InSync.byte(), 0x12);
        assert_eq!(Code::Invalid.byte(), 0x13);
        assert_eq!(Code::BadSiliconRev.byte(), 0x14);
        assert_eq!(Code::Eoc.byte(), 0x20);
        assert_eq!(Code::GetSync.byte(), 0x21);
        assert_eq!(Code::GetDevice.byte(), 0x22);
        assert_eq!(Code::ChipErase.byte(), 0x23);
        assert_eq!(Code::ChipVerify.byte(), 0x24);
        assert_eq!(Code::ProgMulti.byte(), 0x27);
        assert_eq!(Code::ReadMulti.byte(), 0x28);
        assert_eq!(Code::GetCrc.byte(), 0x29);
        assert_eq!(Code::GetOtp.byte(), 0x2a);
        assert_eq!(Code::GetSn.byte(), 0x2b);
        assert_eq!(Code::GetChip.byte(), 0x2c);
        assert_eq!(Code::SetDelay.byte(), 0x2d);
        assert_eq!(Code::GetChipDes.byte(), 0x2e);
        assert_eq!(Code::Reboot.byte(), 0x30);
        assert_eq!(Code::Debug.byte(), 0x31);
        assert_eq!(Code::SetBaud.byte(), 0x33);
        assert_eq!(Code::ExtfErase.byte(), 0x34);
        assert_eq!(Code::ExtfProgMulti.byte(), 0x35);
        assert_eq!(Code::ExtfReadMulti.byte(), 0x36);
        assert_eq!(Code::ExtfGetCrc.byte(), 0x37);
    }

    /// And so are the info selectors.
    #[test]
    fn the_info_words_are_the_ones_the_bootloader_defines() {
        assert_eq!(Info::BootloaderRevision.byte(), 1);
        assert_eq!(Info::BoardId.byte(), 2);
        assert_eq!(Info::BoardRevision.byte(), 3);
        assert_eq!(Info::FlashSize.byte(), 4);
        assert_eq!(Info::VectorArea.byte(), 5);
        assert_eq!(Info::ExternalFlashSize.byte(), 6);
    }

    /// 252, and it matters: the value must be a multiple of four.
    #[test]
    fn the_block_size_is_a_multiple_of_four_below_the_byte_limit() {
        assert_eq!(PROG_MULTI_MAX, 252);
        assert_eq!(READ_MULTI_MAX, 252);
        assert_eq!(PROG_MULTI_MAX % 4, 0);
        const _: () = assert!(PROG_MULTI_MAX < 256, "the length field is one byte");
    }

    /// The frame is opcode, length, payload, end - and nothing else.
    #[test]
    fn a_program_block_is_framed_exactly_as_the_bootloader_expects() {
        let block = program_multi(&[1, 2, 3, 4]).expect("a four-byte block is valid");
        assert_eq!(block, [0x27, 4, 1, 2, 3, 4, 0x20]);

        let external = program_multi_external(&[1, 2, 3, 4]).expect("valid");
        assert_eq!(external, [0x35, 4, 1, 2, 3, 4, 0x20]);
    }

    /// A block that cannot be expressed must be refused, not truncated: a short write leaves a gap
    /// in the middle of the firmware and the board boots into it.
    #[test]
    fn an_oversized_block_is_refused_rather_than_truncated() {
        assert!(program_multi(&[0u8; PROG_MULTI_MAX]).is_some());
        assert!(program_multi(&[0u8; PROG_MULTI_MAX + 1]).is_none());
        assert!(program_multi(&[]).is_none());
    }

    /// The external CRC length is little-endian between the opcode and the terminator.
    #[test]
    fn the_external_crc_carries_its_length_little_endian() {
        assert_eq!(external_crc(0x0403_0201), [0x37, 1, 2, 3, 4, 0x20]);
    }

    /// Bare commands are two bytes, never one.
    #[test]
    fn a_bare_command_is_terminated() {
        assert_eq!(command(Code::GetSync), [0x21, 0x20]);
        assert_eq!(command(Code::ChipErase), [0x23, 0x20]);
        assert_eq!(command(Code::Reboot), [0x30, 0x20]);
        assert_eq!(get_info(Info::BoardId), [0x22, 2, 0x20]);
    }
}
