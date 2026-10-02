//! The SiK bootloader: `Radio/Uploader.cs` - sync, the device, erase, program and verify an
//! Intel HEX image, reboot - and its board, frequency and protocol codes.
//!
//! The C#'s `LogEvent` is not listened to on the page's path (`RFD900APU.DoFirmwareProgramming`
//! takes only the progress, `SikRadio/RFD900.cs:2012-2029`), so nothing here is logged.

use std::fmt;
use std::io;
use std::time::Duration;

use crate::ihex::IHex;
use crate::{Port, Wire};

/// `Uploader.Code`: the protocol's bytes.
/// `// C#: Radio/Uploader.cs:35-56`
pub mod code {
    /// `OK`.
    pub const OK: u8 = 0x10;
    /// `FAILED`.
    pub const FAILED: u8 = 0x11;
    /// `INSYNC`.
    pub const INSYNC: u8 = 0x12;
    /// `EOC`.
    pub const EOC: u8 = 0x20;
    /// `GET_SYNC`.
    pub const GET_SYNC: u8 = 0x21;
    /// `GET_DEVICE`: returns the device id and frequency bytes.
    pub const GET_DEVICE: u8 = 0x22;
    /// `CHIP_ERASE`.
    pub const CHIP_ERASE: u8 = 0x23;
    /// `LOAD_ADDRESS`.
    pub const LOAD_ADDRESS: u8 = 0x24;
    /// `PROG_FLASH`.
    pub const PROG_FLASH: u8 = 0x25;
    /// `READ_FLASH`.
    pub const READ_FLASH: u8 = 0x26;
    /// `PROG_MULTI`.
    pub const PROG_MULTI: u8 = 0x27;
    /// `READ_MULTI`.
    pub const READ_MULTI: u8 = 0x28;
    /// `REBOOT`.
    pub const REBOOT: u8 = 0x30;
}

/// `Enum.Parse` of a byte enum: a number (any value that fits in a byte, named or not) or one of
/// the names.
fn parse_enum(text: &str, names: &[(u8, &'static str)]) -> Option<u8> {
    let text = text.trim();
    if text
        .strip_prefix(['-', '+'])
        .unwrap_or(text)
        .bytes()
        .all(|b| b.is_ascii_digit())
        && !text.is_empty()
    {
        return text.parse::<i64>().ok().and_then(|v| u8::try_from(v).ok());
    }
    names
        .iter()
        .find(|(_, name)| *name == text)
        .map(|(v, _)| *v)
}

/// `Uploader.Board`: a device id, with the names of those the C# names; `ToString` is the name or
/// the number.
/// `// C#: Radio/Uploader.cs:14-33`
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Board(pub u8);

impl Board {
    /// `DEVICE_ID_RF50`.
    pub const RF50: Self = Self(0x4D);
    /// `DEVICE_ID_HM_TRP`.
    pub const HM_TRP: Self = Self(0x4E);
    /// `DEVICE_ID_RFD900`.
    pub const RFD900: Self = Self(0x42);
    /// `DEVICE_ID_RFD900A`.
    pub const RFD900A: Self = Self(0x43);
    /// `DEVICE_ID_HB1060`.
    pub const HB1060: Self = Self(0x50);
    /// `DEVICE_ID_RFD900U`.
    pub const RFD900U: Self = Self(0x81);
    /// `DEVICE_ID_RFD900P`.
    pub const RFD900P: Self = Self(0x82);
    /// `DEVICE_ID_RFD900X`.
    pub const RFD900X: Self = Self(0x83);
    /// `DEVICE_ID_RFD900X2`.
    pub const RFD900X2: Self = Self(0x84);
    /// `DEVICE_ID_RFD900UX2`.
    pub const RFD900UX2: Self = Self(0x85);
    /// `DEVICE_ID_RFD900UX`.
    pub const RFD900UX: Self = Self(0x88);
    /// `FAILED`.
    pub const FAILED: Self = Self(0x11);

    const NAMES: [(u8, &'static str); 12] = [
        (0x4D, "DEVICE_ID_RF50"),
        (0x4E, "DEVICE_ID_HM_TRP"),
        (0x42, "DEVICE_ID_RFD900"),
        (0x43, "DEVICE_ID_RFD900A"),
        (0x50, "DEVICE_ID_HB1060"),
        (0x81, "DEVICE_ID_RFD900U"),
        (0x82, "DEVICE_ID_RFD900P"),
        (0x83, "DEVICE_ID_RFD900X"),
        (0x84, "DEVICE_ID_RFD900X2"),
        (0x85, "DEVICE_ID_RFD900UX2"),
        (0x88, "DEVICE_ID_RFD900UX"),
        (0x11, "FAILED"),
    ];

    /// The C#'s name for it.
    #[must_use]
    pub fn name(self) -> Option<&'static str> {
        Self::NAMES
            .iter()
            .find(|(v, _)| *v == self.0)
            .map(|(_, n)| *n)
    }

    /// `Enum.IsDefined`.
    #[must_use]
    pub fn is_defined(self) -> bool {
        self.name().is_some()
    }

    /// `Enum.Parse(typeof(Uploader.Board), text)`.
    #[must_use]
    pub fn parse(text: &str) -> Option<Self> {
        parse_enum(text, &Self::NAMES).map(Self)
    }
}

impl fmt::Display for Board {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.name() {
            Some(name) => f.write_str(name),
            None => write!(f, "{}", self.0),
        }
    }
}

/// `Uploader.Frequency`: a band code.
/// `// C#: Radio/Uploader.cs:58-68`
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Frequency(pub u8);

impl Frequency {
    /// `FREQ_NONE`.
    pub const NONE: Self = Self(0xF0);
    /// `FREQ_433`.
    pub const F433: Self = Self(0x43);
    /// `FREQ_470`.
    pub const F470: Self = Self(0x47);
    /// `FREQ_868`.
    pub const F868: Self = Self(0x86);
    /// `FREQ_915`.
    pub const F915: Self = Self(0x91);
    /// `FAILED`.
    pub const FAILED: Self = Self(0x11);

    const NAMES: [(u8, &'static str); 6] = [
        (0xF0, "FREQ_NONE"),
        (0x43, "FREQ_433"),
        (0x47, "FREQ_470"),
        (0x86, "FREQ_868"),
        (0x91, "FREQ_915"),
        (0x11, "FAILED"),
    ];

    /// The C#'s name for it.
    #[must_use]
    pub fn name(self) -> Option<&'static str> {
        Self::NAMES
            .iter()
            .find(|(v, _)| *v == self.0)
            .map(|(_, n)| *n)
    }

    /// `Enum.Parse(typeof(Uploader.Frequency), text)`.
    #[must_use]
    pub fn parse(text: &str) -> Option<Self> {
        parse_enum(text, &Self::NAMES).map(Self)
    }
}

impl fmt::Display for Frequency {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.name() {
            Some(name) => f.write_str(name),
            None => write!(f, "{}", self.0),
        }
    }
}

/// What the upload throws.
#[derive(Debug)]
pub enum UploadError {
    /// "SYNC FAIL": no sync after the tries.
    SyncFail,
    /// "SYNC LOST": a command's `INSYNC OK` missing.
    SyncLost,
    /// "This Firmware requires banking support".
    BankingRequired,
    /// "VERIFY FAIL".
    VerifyFail,
    /// "Timeout": a byte that did not come.
    Timeout,
    /// "bootloader device ID mismatch - device:" and the id.
    DeviceMismatch(Board),
    /// The port's own.
    Io(io::Error),
}

impl fmt::Display for UploadError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::SyncFail => f.write_str("SYNC FAIL"),
            Self::SyncLost => f.write_str("SYNC LOST"),
            Self::BankingRequired => f.write_str("This Firmware requires banking support"),
            Self::VerifyFail => f.write_str("VERIFY FAIL"),
            Self::Timeout => f.write_str("Timeout"),
            Self::DeviceMismatch(id) => write!(f, "bootloader device ID mismatch - device:{id}"),
            Self::Io(error) => write!(f, "{error}"),
        }
    }
}

impl std::error::Error for UploadError {}

impl From<io::Error> for UploadError {
    fn from(error: io::Error) -> Self {
        Self::Io(error)
    }
}

/// `Uploader` over a port.
/// `// C#: Radio/Uploader.cs:8-557`
pub struct Uploader<'a, P> {
    wire: &'a mut Wire<P>,
    progress: &'a mut dyn FnMut(f64),
    banking: bool,
    bytes_processed: usize,
    bytes_to_process: usize,
    id: Board,
    freq: Frequency,
    /// `PROG_MULTI_MAX`: the most bytes in one `PROG_MULTI`.
    pub prog_multi_max: usize,
    /// `READ_MULTI_MAX`: the most bytes one `READ_MULTI` asks for.
    pub read_multi_max: usize,
}

impl<'a, P: Port> Uploader<'a, P> {
    /// An uploader on the port, telling `progress` how far it has got (0 to 1, and past 1 as the
    /// C#'s verify counts each block twice).
    pub fn new(wire: &'a mut Wire<P>, progress: &'a mut dyn FnMut(f64)) -> Self {
        Self {
            wire,
            progress,
            banking: false,
            bytes_processed: 0,
            bytes_to_process: 0,
            id: Board::FAILED,
            freq: Frequency::FAILED,
            prog_multi_max: 32,
            read_multi_max: 255,
        }
    }

    /// The board the bootloader said, once it has.
    #[must_use]
    pub const fn board(&self) -> Board {
        self.id
    }

    /// The band the bootloader said, once it has.
    #[must_use]
    pub const fn frequency(&self) -> Frequency {
        self.freq
    }

    #[allow(clippy::cast_precision_loss)]
    fn report(&mut self) {
        let done = self.bytes_processed as f64 / self.bytes_to_process.max(1) as f64;
        (self.progress)(done);
    }

    /// `upload`: sync, program and verify the image, reboot. A failure closes the port, as the
    /// C#'s `catch` does, before it is thrown on.
    /// `// C#: Radio/Uploader.cs:86-103`
    ///
    /// # Errors
    /// What the C# throws.
    pub fn upload(&mut self, image: &IHex) -> Result<(), UploadError> {
        (self.progress)(0.0);
        let result = self
            .connect_and_sync()
            .and_then(|()| self.upload_and_verify(image))
            .and_then(|()| self.send(&[code::REBOOT]).map_err(UploadError::from));
        if result.is_err() && self.wire.is_open() {
            self.wire.close();
        }
        result
    }

    /// `connect_and_sync`: up to three syncs, then one that must succeed, then the device.
    /// `// C#: Radio/Uploader.cs:105-128`
    ///
    /// # Errors
    /// What the C# throws.
    pub fn connect_and_sync(&mut self) -> Result<(), UploadError> {
        // Must be longer than a full flash erase (about a second).
        self.wire.set_read_timeout_ms(2000);
        for _ in 0..3 {
            if self.cmd_sync()? {
                break;
            }
        }
        if !self.cmd_sync()? {
            return Err(UploadError::SyncFail);
        }
        self.check_device()
    }

    /// `upload_and_verify`.
    /// `// C#: Radio/Uploader.cs:130-178`
    fn upload_and_verify(&mut self, image: &IHex) -> Result<(), UploadError> {
        if image.banking_detected && self.id.0 & 0x80 != 0x80 {
            return Err(UploadError::BankingRequired);
        }
        if self.id.0 & 0x80 == 0x80 {
            self.banking = true;
        }
        // Erase the program area first.
        self.send(&[code::CHIP_ERASE, code::EOC])?;
        // The erase takes about two seconds.
        self.wire.sleep(2000);
        self.get_sync()?;
        // Once to program, once to verify.
        self.bytes_to_process = image.blocks.values().map(Vec::len).sum::<usize>() * 2;
        self.bytes_processed = 0;
        for (address, data) in &image.blocks {
            self.set_address(*address)?;
            self.upload_block_multi(data)?;
        }
        for (address, data) in &image.blocks {
            self.set_address(*address)?;
            self.verify_block_multi(data)?;
            self.bytes_processed += data.len();
            self.report();
        }
        Ok(())
    }

    /// `upload_block_multi`: `PROG_MULTI` in chunks.
    /// `// C#: Radio/Uploader.cs:189-212`
    fn upload_block_multi(&mut self, data: &[u8]) -> Result<(), UploadError> {
        for chunk in data.chunks(self.prog_multi_max) {
            let length = u8::try_from(chunk.len()).unwrap_or(u8::MAX);
            self.send(&[code::PROG_MULTI, length])?;
            self.send(chunk)?;
            self.send(&[code::EOC])?;
            self.get_sync()?;
            self.bytes_processed += chunk.len();
            self.report();
        }
        Ok(())
    }

    /// `verify_block_multi`: `READ_MULTI` in chunks, each byte compared.
    /// `// C#: Radio/Uploader.cs:214-236, 307-324`
    fn verify_block_multi(&mut self, data: &[u8]) -> Result<(), UploadError> {
        for chunk in data.chunks(self.read_multi_max) {
            let length = u8::try_from(chunk.len()).unwrap_or(u8::MAX);
            self.send(&[code::READ_MULTI, length, code::EOC])?;
            for expected in chunk {
                if self.recv()? != *expected {
                    return Err(UploadError::VerifyFail);
                }
            }
            self.get_sync()?;
            self.bytes_processed += chunk.len();
            self.report();
        }
        Ok(())
    }

    /// `cmdSync`: whether the bootloader answers `GET_SYNC`.
    /// `// C#: Radio/Uploader.cs:244-260`
    fn cmd_sync(&mut self) -> Result<bool, UploadError> {
        self.wire.discard_in_buffer()?;
        self.send(&[code::GET_SYNC, code::EOC])?;
        Ok(self.get_sync().is_ok())
    }

    /// `cmdSetAddress`: 24 bits on a banking board, 16 otherwise, low byte first.
    /// `// C#: Radio/Uploader.cs:281-301`
    fn set_address(&mut self, address: u32) -> Result<(), UploadError> {
        let [a0, a1, a2, _] = address.to_le_bytes();
        if self.banking {
            self.send(&[code::LOAD_ADDRESS, a0, a1, a2, code::EOC])?;
        } else {
            self.send(&[code::LOAD_ADDRESS, a0, a1, code::EOC])?;
        }
        self.get_sync()
    }

    /// `checkDevice`: the id and band, the id one the C# names.
    /// `// C#: Radio/Uploader.cs:331-346`
    fn check_device(&mut self) -> Result<(), UploadError> {
        self.send(&[code::GET_DEVICE, code::EOC])?;
        self.id = Board(self.recv()?);
        self.freq = Frequency(self.recv()?);
        if !self.id.is_defined() {
            return Err(UploadError::DeviceMismatch(self.id));
        }
        self.get_sync()
    }

    /// `getDevice`: the id and band of a bootloader not yet synchronised with.
    /// `// C#: Radio/Uploader.cs:348-362`
    ///
    /// # Errors
    /// What the C# throws.
    pub fn get_device(&mut self) -> Result<(Board, Frequency), UploadError> {
        self.send(&[code::EOC])?;
        self.wire.sleep(100);
        self.wire.discard_in_buffer()?;
        self.cmd_sync()?;
        self.send(&[code::GET_DEVICE, code::EOC])?;
        let device = Board(self.recv()?);
        let freq = Frequency(self.recv()?);
        self.get_sync()?;
        Ok((device, freq))
    }

    /// `getSync`: `INSYNC` then `OK`, anything else "SYNC LOST".
    /// `// C#: Radio/Uploader.cs:373-401`
    fn get_sync(&mut self) -> Result<(), UploadError> {
        let sync = self.recv().map_err(|_| UploadError::SyncLost)?;
        if sync != code::INSYNC {
            return Err(UploadError::SyncLost);
        }
        let status = self.recv().map_err(|_| UploadError::SyncLost)?;
        if status != code::OK {
            return Err(UploadError::SyncLost);
        }
        Ok(())
    }

    /// `send`.
    /// `// C#: Radio/Uploader.cs:403-485`
    fn send(&mut self, bytes: &[u8]) -> io::Result<()> {
        self.wire.write(bytes)
    }

    /// `recv`: a byte within `ReadTimeout`, or "Timeout".
    /// `// C#: Radio/Uploader.cs:487-506`
    fn recv(&mut self) -> Result<u8, UploadError> {
        let timeout: Duration = self.wire.read_timeout();
        self.wire
            .read_byte_within(timeout)?
            .ok_or(UploadError::Timeout)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::radio::{Mode, Radio};

    fn bootloader() -> Wire<Radio> {
        let mut radio = Radio::rfd900p();
        radio.enter(Mode::Bootloader);
        let mut wire = Wire::new(radio);
        wire.set_baud(115_200).unwrap();
        wire
    }

    /// An image programmed and read back over the bootloader, with 24-bit addresses on a
    /// banking board, then the reboot.
    #[test]
    fn an_image_is_programmed_verified_and_rebooted() {
        let mut wire = bootloader();
        let image = IHex::parse(
            ":10000000000102030405060708090A0B0C0D0E0F00\n:1000100010111213141516171819101B1C1D1E1F00\n:0300400052464400\n",
        )
        .unwrap();
        let mut progress = Vec::new();
        let mut record = |p: f64| progress.push(p);
        let mut uploader = Uploader::new(&mut wire, &mut record);
        uploader.upload(&image).unwrap();
        assert_eq!(uploader.board(), Board::RFD900P);
        assert_eq!(uploader.frequency(), Frequency::F915);
        let radio = wire.port();
        assert_eq!(radio.mode(), Mode::Transparent, "rebooted");
        assert_eq!(radio.flash.get(&0x40), Some(&b'R'));
        assert_eq!(radio.flash.len(), 35);
        let boot: Vec<&String> = radio
            .commands()
            .iter()
            .filter(|c| c.starts_with("boot"))
            .collect();
        assert_eq!(boot.first().map(|s| s.as_str()), Some("boot 0x21"));
        assert!(boot.iter().any(|c| *c == "boot 0x23"), "erased");
        assert_eq!(boot.last().map(|s| s.as_str()), Some("boot 0x30"));
        // LOAD_ADDRESS with three address bytes: the RFD900+ banks.
        let written = &radio.written;
        let load = written
            .windows(5)
            .any(|w| w == [0x24, 0x40, 0x00, 0x00, 0x20]);
        assert!(load, "a 24-bit address");
        assert_eq!(progress.first(), Some(&0.0));
        assert!(progress.last().unwrap() >= &1.0, "{progress:?}");
    }

    /// A bootloader that stops answering: no sync, and the port closed.
    #[test]
    fn a_silent_bootloader_fails_the_sync_and_closes_the_port() {
        let mut wire = bootloader();
        wire.port_mut().bootloader_silent = true;
        let image = IHex::parse(":0100000001FE\n").unwrap();
        let mut ignore = |_| {};
        let result = Uploader::new(&mut wire, &mut ignore).upload(&image);
        assert!(matches!(result, Err(UploadError::SyncFail)), "{result:?}");
        assert!(!wire.is_open(), "closed");
    }

    /// A banked image on a board without banking is refused before the erase.
    #[test]
    fn a_banked_image_needs_a_banking_board() {
        let mut wire = bootloader();
        wire.port_mut().local.board = 0x43;
        let image = IHex::parse(":020000040001F9\n:0100000001FE\n").unwrap();
        let mut ignore = |_| {};
        let result = Uploader::new(&mut wire, &mut ignore).upload(&image);
        assert!(matches!(result, Err(UploadError::BankingRequired)));
        assert_eq!(
            UploadError::BankingRequired.to_string(),
            "This Firmware requires banking support"
        );
    }

    /// `getDevice` from cold, and the enum texts.
    #[test]
    fn the_device_and_its_names() {
        let mut wire = bootloader();
        let mut ignore = |_| {};
        let (board, freq) = Uploader::new(&mut wire, &mut ignore).get_device().unwrap();
        assert_eq!((board, freq), (Board::RFD900P, Frequency::F915));
        assert_eq!(Board::RFD900P.to_string(), "DEVICE_ID_RFD900P");
        assert_eq!(Board(7).to_string(), "7");
        assert_eq!(Board::parse(" 130\r\n"), Some(Board::RFD900P));
        assert_eq!(Board::parse("DEVICE_ID_HM_TRP"), Some(Board::HM_TRP));
        assert_eq!(Board::parse("300"), None);
        assert_eq!(Board::parse(""), None);
        assert_eq!(Frequency::parse("145"), Some(Frequency::F915));
        assert_eq!(Frequency(0x91).to_string(), "FREQ_915");
    }
}
