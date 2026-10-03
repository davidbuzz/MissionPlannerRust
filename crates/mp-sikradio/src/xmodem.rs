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

//! XModem-CRC, as the RFD900x family's bootloader takes a firmware: `Radio/XModem.cs`.
//!
//! 128-byte blocks numbered from 1, the last padded with `&` (0x26), each answered with an ACK -
//! ten tries a block - then `EOT` and the reboot into the new firmware with `BOOTNEW`.

use std::io;

use crate::{Port, Wire};

/// `SOH`.
const SOH: u8 = 0x01;
/// `EOT`.
const EOT: u8 = 0x04;
/// `ACK`.
const ACK: u8 = 0x06;
/// `C`, the receiver's request for CRC mode, read past while waiting for an ACK.
const C: u8 = 0x43;
/// What a short block is padded with.
const PAD: u8 = 0x26;
/// The tries a block or the `EOT` gets.
const RETRIES: usize = 10;

/// `CRC_calc`: CRC-16/XMODEM.
/// `// C#: Radio/XModem.cs:24-41`
#[must_use]
pub fn crc(data: &[u8]) -> u16 {
    let mut crc: u16 = 0;
    for byte in data {
        crc = crc.rotate_left(8);
        crc ^= u16::from(*byte);
        crc ^= (crc & 0xFF) >> 4;
        crc ^= crc << 12;
        crc ^= (crc & 0xFF) << 5;
    }
    crc
}

/// `SendBlock(byte[] data, ...)`: the block, its number and complement, the data padded to 128,
/// the CRC high byte first; what has come in discarded first.
/// `// C#: Radio/XModem.cs:69-90`
#[must_use]
pub fn block(data: &[u8], number: usize) -> [u8; 133] {
    let mut bits = [PAD; 128];
    for (slot, byte) in bits.iter_mut().zip(data) {
        *slot = *byte;
    }
    let mut packet = [0u8; 133];
    let low = u8::try_from(number % 256).unwrap_or(0);
    packet[0] = SOH;
    packet[1] = low;
    packet[2] = 255 - low;
    packet[3..131].copy_from_slice(&bits);
    let crc = crc(&bits);
    let [high, low] = crc.to_be_bytes();
    packet[131] = high;
    packet[132] = low;
    packet
}

/// Why an upload stopped.
#[derive(Debug)]
pub enum XModemError {
    /// A read that timed out, where the C#'s `ReadByte` throws out of `Upload`.
    Timeout,
    /// The port's own.
    Io(io::Error),
}

impl From<io::Error> for XModemError {
    fn from(error: io::Error) -> Self {
        Self::Io(error)
    }
}

impl std::fmt::Display for XModemError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Timeout => f.write_str("The operation has timed out."),
            Self::Io(error) => write!(f, "{error}"),
        }
    }
}

impl std::error::Error for XModemError {}

/// `ReadByte`, past any `C`.
fn ack<P: Port>(wire: &mut Wire<P>) -> Result<u8, XModemError> {
    loop {
        let byte = wire.read_byte()?.ok_or(XModemError::Timeout)?;
        if byte != C {
            return Ok(byte);
        }
    }
}

/// `Upload`: the file's bytes in blocks, `EOT`, then `BOOTNEW`. False when a block or the `EOT`
/// went unacknowledged ten times; `progress` hears the fraction sent, and 100 at the `EOT` as the
/// C#'s `SendEOT` says it.
/// `// C#: Radio/XModem.cs:92-196`
///
/// # Errors
/// A timeout or the port's error, as the C# throws them.
pub fn upload<P: Port>(
    firmware: &[u8],
    wire: &mut Wire<P>,
    progress: &mut dyn FnMut(f64),
) -> Result<bool, XModemError> {
    wire.set_read_timeout_ms(2000);
    let start = firmware.len().div_ceil(128);
    let mut left = start;
    for (index, data) in firmware.chunks(128).enumerate() {
        let mut sent = false;
        for _ in 0..RETRIES {
            let packet = block(data, index + 1);
            wire.discard_in_buffer()?;
            wire.write(&packet)?;
            if ack(wire)? == ACK {
                sent = true;
                break;
            }
        }
        if !sent {
            return Ok(false);
        }
        left -= 1;
        #[allow(clippy::cast_precision_loss)]
        progress(1.0 - left as f64 / start as f64);
    }
    let mut ended = false;
    for _ in 0..RETRIES {
        wire.write(&[EOT])?;
        progress(100.0);
        if ack(wire)? == ACK {
            ended = true;
            break;
        }
    }
    if !ended {
        return Ok(false);
    }
    // Boot.
    wire.sleep(100);
    wire.write_str("\r\n")?;
    wire.sleep(100);
    wire.write_str("BOOTNEW\r\n")?;
    wire.sleep(100);
    Ok(true)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::radio::{Mode, Radio};

    /// CRC-16/XMODEM's check value.
    #[test]
    fn the_crc_is_xmodems() {
        assert_eq!(crc(b"123456789"), 0x31C3);
        let packet = block(b"AB", 1);
        assert_eq!(&packet[..5], &[SOH, 1, 254, b'A', b'B']);
        assert_eq!(packet[5], PAD);
        assert_eq!(block(&[], 256)[1..3], [0, 255]);
    }

    /// A firmware sent to the bootloader block by block, a NAK resent, and the new one booted.
    #[test]
    fn a_firmware_goes_over_in_blocks() {
        let mut radio = Radio::rfd900p();
        radio.enter(Mode::XModem);
        radio.nak_blocks = 1;
        let mut wire = Wire::new(radio);
        let firmware: Vec<u8> = (0..300u32).map(|i| (i % 251) as u8).collect();
        let mut progress = Vec::new();
        assert!(upload(&firmware, &mut wire, &mut |p| progress.push(p)).unwrap());
        let received = &wire.port().received;
        assert_eq!(received.len(), 384, "three blocks");
        assert_eq!(&received[..300], firmware.as_slice());
        assert!(received[300..].iter().all(|b| *b == PAD));
        assert_eq!(progress.last(), Some(&100.0));
        assert!(wire.port().commands().iter().any(|c| c == "x EOT"));
        assert!(String::from_utf8_lossy(&wire.port().written).ends_with("BOOTNEW\r\n"));
    }

    /// No answer at all is the C#'s timeout.
    #[test]
    fn a_silent_bootloader_times_out() {
        let mut wire = Wire::new(Radio::rfd900p());
        let result = upload(&[1, 2, 3], &mut wire, &mut |_| {});
        assert!(matches!(result, Err(XModemError::Timeout)));
    }
}
