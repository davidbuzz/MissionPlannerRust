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

//! The px4 bootloader mock, shared by every test that needs the other end of the wire.
//!
//! Moved here unchanged from `firmware_upload.rs` so that board detection (`board_detect.rs`)
//! probes the same strict mock the upload is proven against, rather than a second, laxer one.

// Each test binary uses a different part of the mock.
#![allow(dead_code)]
// The mock indexes into a buffer it has just length-checked, and panicking is what it is for: an
// out-of-range index here is a malformed command, which is the failure the tests exist to catch.
#![allow(clippy::indexing_slicing, clippy::expect_used)]

use mp_firmware::firmware::crc32;
use mp_firmware::protocol::{Code, Info, PROG_MULTI_MAX};
use std::io::{Read, Write};

/// A bootloader that answers, and records exactly what it was asked.
pub(crate) struct MockBootloader {
    /// Everything the uploader has written, in order.
    pub(crate) received: Vec<u8>,
    /// Bytes waiting to be read back.
    pub(crate) outgoing: std::collections::VecDeque<u8>,
    /// What `GET_DEVICE` returns, by selector byte.
    pub(crate) info: std::collections::HashMap<u8, u32>,
    /// The flash, as the mock has been programmed.
    pub(crate) flash: Vec<u8>,
    /// Whether an erase has happened.
    pub(crate) erased: bool,
    /// The CRC to report, or `None` to compute one over what was actually written.
    pub(crate) crc_override: Option<u32>,
    /// How much flash the board claims.
    pub(crate) flash_size: usize,
    /// What `GET_CHIP` returns: the MCU's IDCODE.
    pub(crate) chip: u32,
    /// What `GET_CHIP_DES` returns.
    pub(crate) chip_desc: String,
    /// What `GET_SN` returns, a word per address.
    pub(crate) sn: [u8; 12],
    /// Whether `GET_CHIP` is answered `INSYNC INVALID`, as a bootloader without it answers.
    pub(crate) refuse_chip: bool,
}

impl MockBootloader {
    pub(crate) fn new(board_id: u32, flash_size: usize) -> Self {
        let mut info = std::collections::HashMap::new();
        info.insert(Info::BootloaderRevision.byte(), 5);
        info.insert(Info::BoardId.byte(), board_id);
        info.insert(Info::BoardRevision.byte(), 0);
        info.insert(
            Info::FlashSize.byte(),
            u32::try_from(flash_size).unwrap_or(u32::MAX),
        );
        Self {
            received: Vec::new(),
            outgoing: std::collections::VecDeque::new(),
            info,
            flash: Vec::new(),
            erased: false,
            crc_override: None,
            flash_size,
            // A CubeOrange's STM32H743, as its bootloader describes it.
            chip: 0x1003_6450,
            chip_desc: "STM32H7[4|5]x,rev:V".to_owned(),
            sn: *b"\x00\x1d\x00\x2a\x32\x30\x51\x0b\x35\x38\x39\x38",
            refuse_chip: false,
        }
    }

    pub(crate) fn reply_ok(&mut self) {
        self.outgoing.push_back(Code::InSync.byte());
        self.outgoing.push_back(Code::Ok.byte());
    }

    pub(crate) fn reply_word(&mut self, value: u32) {
        self.outgoing.extend(value.to_le_bytes());
        self.reply_ok();
    }

    /// The CRC the board would report: what was written, padded with erased flash.
    pub(crate) fn flash_crc(&self) -> u32 {
        if let Some(forced) = self.crc_override {
            return forced;
        }
        let mut state = crc32(&self.flash, 0);
        let mut index = self.flash.len();
        while index + 1 < self.flash_size {
            state = crc32(&[0xff; 4], state);
            index += 4;
        }
        state
    }

    /// Consumes one complete command from `received`, answering it.
    ///
    /// Returns false when there is not a whole command yet.
    pub(crate) fn step(&mut self) -> bool {
        let Some(&opcode) = self.received.first() else {
            return false;
        };
        match opcode {
            b if b == Code::GetSync.byte() => {
                if self.received.len() < 2 {
                    return false;
                }
                assert_eq!(
                    self.received[1],
                    Code::Eoc.byte(),
                    "GET_SYNC must be terminated"
                );
                self.received.drain(..2);
                self.reply_ok();
            }
            b if b == Code::GetDevice.byte() => {
                if self.received.len() < 3 {
                    return false;
                }
                let selector = self.received[1];
                assert_eq!(
                    self.received[2],
                    Code::Eoc.byte(),
                    "GET_DEVICE must be terminated"
                );
                self.received.drain(..3);
                let value = *self.info.get(&selector).unwrap_or(&0);
                self.reply_word(value);
            }
            b if b == Code::ChipErase.byte() => {
                if self.received.len() < 2 {
                    return false;
                }
                assert_eq!(self.received[1], Code::Eoc.byte());
                self.received.drain(..2);
                self.erased = true;
                self.flash.clear();
                self.reply_ok();
            }
            b if b == Code::ProgMulti.byte() || b == Code::ExtfProgMulti.byte() => {
                if self.received.len() < 2 {
                    return false;
                }
                let length = usize::from(self.received[1]);
                assert!(
                    length <= PROG_MULTI_MAX,
                    "block of {length} exceeds the protocol maximum"
                );
                assert!(length > 0, "an empty block is not a valid PROG_MULTI");
                if self.received.len() < length + 3 {
                    return false;
                }
                assert_eq!(
                    self.received[length + 2],
                    Code::Eoc.byte(),
                    "PROG_MULTI must be terminated"
                );
                assert!(
                    self.erased || b == Code::ExtfProgMulti.byte(),
                    "programming before erasing"
                );
                let block: Vec<u8> = self.received[2..length + 2].to_vec();
                self.received.drain(..length + 3);
                self.flash.extend_from_slice(&block);
                self.reply_ok();
            }
            b if b == Code::GetCrc.byte() => {
                if self.received.len() < 2 {
                    return false;
                }
                assert_eq!(self.received[1], Code::Eoc.byte());
                self.received.drain(..2);
                let crc = self.flash_crc();
                self.reply_word(crc);
            }
            b if b == Code::ExtfGetCrc.byte() => {
                if self.received.len() < 6 {
                    return false;
                }
                assert_eq!(self.received[5], Code::Eoc.byte());
                let length = u32::from_le_bytes([
                    self.received[1],
                    self.received[2],
                    self.received[3],
                    self.received[4],
                ]);
                self.received.drain(..6);
                let end = usize::try_from(length).unwrap_or(0).min(self.flash.len());
                let crc = crc32(&self.flash[..end], 0);
                self.reply_word(crc);
            }
            b if b == Code::GetChip.byte() => {
                if self.received.len() < 2 {
                    return false;
                }
                assert_eq!(
                    self.received[1],
                    Code::Eoc.byte(),
                    "GET_CHIP must be terminated"
                );
                self.received.drain(..2);
                if self.refuse_chip {
                    self.outgoing.push_back(Code::InSync.byte());
                    self.outgoing.push_back(Code::Invalid.byte());
                } else {
                    let chip = self.chip;
                    self.reply_word(chip);
                }
            }
            b if b == Code::GetChipDes.byte() => {
                if self.received.len() < 2 {
                    return false;
                }
                assert_eq!(
                    self.received[1],
                    Code::Eoc.byte(),
                    "GET_CHIP_DES must be terminated"
                );
                self.received.drain(..2);
                let text = self.chip_desc.clone().into_bytes();
                self.outgoing
                    .extend(u32::try_from(text.len()).unwrap_or(0).to_le_bytes());
                self.outgoing.extend(text);
                self.reply_ok();
            }
            b if b == Code::GetSn.byte() => {
                if self.received.len() < 6 {
                    return false;
                }
                assert_eq!(
                    self.received[5],
                    Code::Eoc.byte(),
                    "GET_SN must be terminated"
                );
                let address = u32::from_le_bytes([
                    self.received[1],
                    self.received[2],
                    self.received[3],
                    self.received[4],
                ]);
                self.received.drain(..6);
                let at = usize::try_from(address).unwrap_or(usize::MAX);
                assert!(
                    at + 4 <= self.sn.len(),
                    "GET_SN past the serial number: {at}"
                );
                let word: Vec<u8> = self.sn[at..at + 4].to_vec();
                self.outgoing.extend(word);
                self.reply_ok();
            }
            b if b == Code::Reboot.byte() => {
                if self.received.len() < 2 {
                    return false;
                }
                assert_eq!(self.received[1], Code::Eoc.byte());
                self.received.drain(..2);
                // No reply: the board is gone the moment it obeys.
            }
            other => {
                panic!("the uploader sent an opcode the bootloader does not define: {other:#04x}")
            }
        }
        true
    }
}

impl Write for MockBootloader {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        self.received.extend_from_slice(buf);
        while self.step() {}
        Ok(buf.len())
    }
    fn flush(&mut self) -> std::io::Result<()> {
        Ok(())
    }
}

impl Read for MockBootloader {
    fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
        if self.outgoing.is_empty() {
            return Err(std::io::Error::new(
                std::io::ErrorKind::TimedOut,
                "the bootloader has nothing to say",
            ));
        }
        let count = buf.len().min(self.outgoing.len());
        for slot in buf.iter_mut().take(count) {
            *slot = self.outgoing.pop_front().unwrap_or(0);
        }
        Ok(count)
    }
}
