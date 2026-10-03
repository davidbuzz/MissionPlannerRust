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

//! SiK telemetry radios: what SETUP's Sik Radio page says to one, and how it reads the answers.
//!
//! The page is `Radio/Sikradio.cs`; the library under it is RFDesign's, compiled into the planner
//! (`MissionPlanner.csproj:1190-1199`): `SikRadio/RFD900.cs` - the session that finds which mode
//! the radio is in and moves it between transparent, AT command and bootloader modes, the
//! settings and their ranges, the modem models and their firmware checks - and `SikRadio/RFDLib/`,
//! its AT command client. The firmware goes over `Radio/Uploader.cs` (the SiK bootloader, from an
//! Intel HEX file, `Radio/IHex.cs`) or `Radio/XModem.cs` (the RFD900x family's bootloader).
//!
//! * [`Port`] is the serial port the C# opens (`MissionPlanner.Radio.ComPort`), with its clock:
//!   every guard time, sleep and read timeout of the C# runs on [`Port::now`] and
//!   [`Port::sleep`], so a test's scripted radio answers the C#'s 1.5-second `+++` guard at once
//!   and still sees the silences it needs. [`Wire`] is that port as the C# calls it -
//!   `ReadByte`, `ReadChar`, `ReadExisting`, `BytesToRead`, `DiscardInBuffer`, `ReadTimeout`.
//! * [`at`] is the AT command layer: `RFDLib`'s client and token waits, and the page's own
//!   `doCommand`.
//! * [`settings`] is `TSetting` and its kin: the `ATI5?` and `ATI5` lines parsed, the options and
//!   ranges, the settings file.
//! * [`session`] is `TSession` and the `RFD900` modem classes ([`modem`]).
//! * [`ihex`], [`uploader`] and [`xmodem`] are the firmware upload.
//! * [`page`] is the wire half of the page's handlers - Load Settings, Save Settings, Reset to
//!   Defaults, Set PPM Fail Safe, the encryption level, Upload Firmware - run over a session and
//!   reporting what the C# puts in `lbl_status`, its progress bar and its boxes.
//!
//! Nothing here opens a port or touches a window: the application gives a [`Port`] and shows what
//! comes back.

pub mod at;
pub mod ihex;
pub mod modem;
pub mod page;
pub mod session;
pub mod settings;
pub mod uploader;
pub mod xmodem;

#[cfg(test)]
mod radio;

use std::collections::VecDeque;
use std::io;
use std::time::Duration;

/// The radio's serial port: `ICommsSerial`, as `ComPort.GetComPortForSiKRadio` opens it - a
/// serial port at the connection box's port and baud rate, or a TCP socket when the link is TCP
/// (`Radio/Sikradio.cs:236-271`) - with the clock the C#'s sleeps and timeouts run on.
pub trait Port: Send {
    /// `Write`: the bytes, all of them.
    ///
    /// # Errors
    /// The port's own, as the C#'s `Write` throws.
    fn write(&mut self, data: &[u8]) -> io::Result<()>;

    /// Waits up to `timeout` for bytes and returns how many it put in `buf`: `Ok(0)` when the
    /// timeout passed with none. A zero timeout takes only what has already arrived.
    ///
    /// # Errors
    /// The port's own.
    fn read(&mut self, buf: &mut [u8], timeout: Duration) -> io::Result<usize>;

    /// `BaudRate = baud`. A TCP socket has no baud rate and keeps none.
    ///
    /// # Errors
    /// The port's own.
    fn set_baud(&mut self, baud: u32) -> io::Result<()>;

    /// `BaudRate`.
    fn baud(&self) -> u32;

    /// `Thread.Sleep`.
    fn sleep(&mut self, duration: Duration);

    /// The time on the port's clock, from any start: `DateTime.Now` and `Stopwatch` as the C#
    /// measures its deadlines.
    fn now(&self) -> Duration;

    /// `IsOpen`.
    fn is_open(&self) -> bool {
        true
    }

    /// `Close`: the bootloader upload closes the port when it fails (`Radio/Uploader.cs:96-101`).
    fn close(&mut self) {}
}

/// The error a closed port gives, as the C#'s `InvalidOperationException` for one.
#[must_use]
pub fn closed() -> io::Error {
    io::Error::new(io::ErrorKind::NotConnected, "The port is closed.")
}

/// The port as the C# calls it: what has arrived and not been read yet, and `ReadTimeout`.
#[derive(Debug)]
pub struct Wire<P> {
    /// The port.
    port: P,
    /// Bytes read from the port and not yet taken.
    pending: VecDeque<u8>,
    /// `ReadTimeout`.
    read_timeout: Duration,
}

/// The longest a `DiscardInBuffer` keeps draining, in reads: the OS buffer, not a stream that
/// never stops.
const DISCARD_READS: usize = 64;

impl<P: Port> Wire<P> {
    /// The port, with `ReadTimeout` at `SerialPort`'s 500 ms until something sets it.
    #[must_use]
    pub fn new(port: P) -> Self {
        Self {
            port,
            pending: VecDeque::new(),
            read_timeout: Duration::from_millis(500),
        }
    }

    /// The port.
    #[must_use]
    pub const fn port(&self) -> &P {
        &self.port
    }

    /// The port, to change.
    pub fn port_mut(&mut self) -> &mut P {
        &mut self.port
    }

    /// The port back.
    #[must_use]
    pub fn into_port(self) -> P {
        self.port
    }

    /// `IsOpen`.
    #[must_use]
    pub fn is_open(&self) -> bool {
        self.port.is_open()
    }

    /// `Close`.
    pub fn close(&mut self) {
        self.pending.clear();
        self.port.close();
    }

    /// `Write(byte[], ...)`.
    ///
    /// # Errors
    /// The port's.
    pub fn write(&mut self, data: &[u8]) -> io::Result<()> {
        if !self.port.is_open() {
            return Err(closed());
        }
        self.port.write(data)
    }

    /// `Write(string)`: the text's bytes - every string the radio code writes is ASCII.
    ///
    /// # Errors
    /// The port's.
    pub fn write_str(&mut self, text: &str) -> io::Result<()> {
        self.write(text.as_bytes())
    }

    /// Reads what the port has within `timeout` into the pending bytes.
    fn fill(&mut self, timeout: Duration) -> io::Result<()> {
        if !self.port.is_open() {
            return Err(closed());
        }
        let mut buf = [0u8; 512];
        let read = self.port.read(&mut buf, timeout)?;
        self.pending.extend(buf.iter().take(read));
        Ok(())
    }

    /// `ReadTimeout`.
    #[must_use]
    pub const fn read_timeout(&self) -> Duration {
        self.read_timeout
    }

    /// `ReadTimeout = timeout`.
    pub const fn set_read_timeout(&mut self, timeout: Duration) {
        self.read_timeout = timeout;
    }

    /// `ReadTimeout = ms`.
    pub const fn set_read_timeout_ms(&mut self, ms: u64) {
        self.read_timeout = Duration::from_millis(ms);
    }

    /// `BytesToRead`.
    ///
    /// # Errors
    /// The port's.
    pub fn bytes_to_read(&mut self) -> io::Result<usize> {
        self.fill(Duration::ZERO)?;
        Ok(self.pending.len())
    }

    /// `ReadByte`: the next byte within `ReadTimeout`, or `None` where the C# throws its
    /// `TimeoutException`.
    ///
    /// # Errors
    /// The port's.
    pub fn read_byte(&mut self) -> io::Result<Option<u8>> {
        self.read_byte_within(self.read_timeout)
    }

    /// The next byte within `timeout`, or `None`.
    ///
    /// # Errors
    /// The port's.
    pub fn read_byte_within(&mut self, timeout: Duration) -> io::Result<Option<u8>> {
        if self.pending.is_empty() {
            self.fill(timeout)?;
        }
        Ok(self.pending.pop_front())
    }

    /// `ReadChar`: the next byte as an ASCII character (`SerialPort`'s default encoding, which
    /// makes a byte over 127 a `?`), or `None` at the timeout.
    ///
    /// # Errors
    /// The port's.
    pub fn read_char(&mut self) -> io::Result<Option<char>> {
        Ok(self.read_byte()?.map(ascii))
    }

    /// `ReadExisting`: whatever has arrived, as ASCII text.
    ///
    /// # Errors
    /// The port's.
    pub fn read_existing(&mut self) -> io::Result<String> {
        self.fill(Duration::ZERO)?;
        Ok(self.pending.drain(..).map(ascii).collect())
    }

    /// `DiscardInBuffer`: what has arrived, dropped.
    ///
    /// # Errors
    /// The port's.
    pub fn discard_in_buffer(&mut self) -> io::Result<()> {
        if !self.port.is_open() {
            return Err(closed());
        }
        self.pending.clear();
        let mut buf = [0u8; 512];
        for _ in 0..DISCARD_READS {
            if self.port.read(&mut buf, Duration::ZERO)? == 0 {
                break;
            }
        }
        Ok(())
    }

    /// `Thread.Sleep(ms)`.
    pub fn sleep(&mut self, ms: u64) {
        self.port.sleep(Duration::from_millis(ms));
    }

    /// The port's clock.
    #[must_use]
    pub fn now(&self) -> Duration {
        self.port.now()
    }

    /// `BaudRate`.
    #[must_use]
    pub fn baud(&self) -> u32 {
        self.port.baud()
    }

    /// `BaudRate = baud`.
    ///
    /// # Errors
    /// The port's.
    pub fn set_baud(&mut self, baud: u32) -> io::Result<()> {
        self.port.set_baud(baud)
    }
}

/// A byte as `Encoding.ASCII` decodes it: itself below 128, `?` above.
#[must_use]
pub fn ascii(byte: u8) -> char {
    if byte.is_ascii() {
        char::from(byte)
    } else {
        '?'
    }
}

/// `int.Parse(text, NumberStyles.Any)` as the radio's answers need it: white space around an
/// optional sign and decimal digits, and a fraction of zeros (`Any` takes a decimal point when
/// what follows it is zero). `None` where the C# throws.
#[must_use]
pub fn parse_int_any(text: &str) -> Option<i64> {
    let text = text.trim();
    let (negative, digits) = match text.strip_prefix('-') {
        Some(rest) => (true, rest),
        None => (false, text.strip_prefix('+').unwrap_or(text)),
    };
    let (whole, fraction) = digits.split_once('.').unwrap_or((digits, ""));
    if whole.is_empty()
        || !whole.bytes().all(|b| b.is_ascii_digit())
        || !fraction.bytes().all(|b| b == b'0')
    {
        return None;
    }
    let value: i64 = whole.parse().ok()?;
    let value = if negative { -value } else { value };
    i32::try_from(value).ok().map(i64::from)
}

/// `int.Parse(text, NumberStyles.AllowHexSpecifier)`: hex digits only, no sign, no space.
#[must_use]
pub fn parse_int_hex(text: &str) -> Option<i64> {
    if text.is_empty() || !text.bytes().all(|b| b.is_ascii_hexdigit()) {
        return None;
    }
    // `AllowHexSpecifier` reads up to eight digits as a two's-complement Int32.
    let value = u32::from_str_radix(text, 16).ok()?;
    Some(i64::from(i32::from_ne_bytes(value.to_ne_bytes())))
}

/// `int.TryParse(text)`: white space around an optional sign and digits.
#[must_use]
pub fn try_parse_int(text: &str) -> Option<i32> {
    let text = text.trim();
    let digits = text
        .strip_prefix('-')
        .or_else(|| text.strip_prefix('+'))
        .unwrap_or(text);
    if digits.is_empty() || !digits.bytes().all(|b| b.is_ascii_digit()) {
        return None;
    }
    text.parse().ok()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The C#'s integer parses, as the radio's answers meet them.
    #[test]
    fn integers_parse_as_the_csharp_parses_them() {
        assert_eq!(parse_int_any(" 145 "), Some(145));
        assert_eq!(parse_int_any("-5"), Some(-5));
        assert_eq!(parse_int_any("12.00"), Some(12));
        assert_eq!(parse_int_any("12.5"), None);
        assert_eq!(parse_int_any("ERROR"), None);
        assert_eq!(parse_int_any(""), None);
        assert_eq!(parse_int_hex("091"), Some(0x91));
        assert_eq!(parse_int_hex("0 91"), None);
        assert_eq!(parse_int_hex("FFFFFFFF"), Some(-1));
        assert_eq!(try_parse_int(" 3 "), Some(3));
        assert_eq!(try_parse_int("3a"), None);
        assert_eq!(try_parse_int("+7"), Some(7));
        assert_eq!(ascii(b'A'), 'A');
        assert_eq!(ascii(0xC8), '?');
    }
}
