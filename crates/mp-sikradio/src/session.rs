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

//! `TSession`: which mode the radio is in and how to move it to another - transparent, AT
//! command, the SiK bootloader or the RFD900x's - the modem object, the settings with their
//! ranges, and the firmware programming of each model.
//! `// C#: SikRadio/RFD900.cs:13-1300, 1795-1826, 1983-2064, 2144-2164, 2182-2196, 2237-2345, 2433-2504, 2636-2655`
//!
//! The session owns the port while it lives; `EndSession` drops the session and keeps the port
//! (`ComPort._Port` outlives `_Session`), which is [`Session::into_wire`].

use std::fmt;
use std::io;
use std::path::Path;

use crate::at::{self, AtClient};
use crate::ihex::IHex;
use crate::modem::{self, Country, Family, Modem};
use crate::page::Report;
use crate::settings::{self, Setting, Settings};
use crate::uploader::{Board, Uploader, code};
use crate::{Port, Wire, xmodem};

/// `TSession.TMode`.
/// `// C#: SikRadio/RFD900.cs:61-69`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    /// `INIT`: not yet known.
    Init,
    /// `TRANSPARENT`.
    Transparent,
    /// `AT_COMMAND`.
    AtCommand,
    /// `BOOTLOADER`: the SiK bootloader (RFD900a, u, +).
    Bootloader,
    /// `BOOTLOADER_X`: the RFD900x family's.
    BootloaderX,
    /// `UNKNOWN`.
    Unknown,
}

impl fmt::Display for Mode {
    /// `TMode.ToString()`, which the page shows as "Mode is ...".
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::Init => "INIT",
            Self::Transparent => "TRANSPARENT",
            Self::AtCommand => "AT_COMMAND",
            Self::Bootloader => "BOOTLOADER",
            Self::BootloaderX => "BOOTLOADER_X",
            Self::Unknown => "UNKNOWN",
        })
    }
}

/// `BOOTLOADERX_BAUD`.
const BOOTLOADER_X_BAUD: u32 = 57_600;
/// `BOOTLOADER_BAUD`.
const BOOTLOADER_BAUD: u32 = 115_200;
/// `ENC_KEY_SETTING_NAME`.
pub const KEY_SETTING: &str = "AESKEY";
/// The most `ATIn10:n` lines asked for. The C# asks until the radio says "eof"; a SiK radio has
/// fewer than sixty registers, and this keeps one that never says it from holding the session.
const ATI10_LINES: usize = 256;

/// The text the C#'s `GetRefFirmwarePath` throws in the planner, which does not embed the
/// reference firmware it reads (`MissionPlanner.csproj:75` removes `SikRadio\**` from the
/// embedded resources): `new StreamReader(null)`.
/// `// C#: SikRadio/RFD900.cs:2391-2412`
pub const NO_REFERENCE_FIRMWARE: &str = "Value cannot be null.\r\nParameter name: stream";

/// The session: the port, the mode the radio was last known to be in, the board the page read,
/// and the modem object once made.
#[derive(Debug)]
pub struct Session<P> {
    wire: Wire<P>,
    mode: Mode,
    /// `Board`: what the page's Load Settings read from `ATI2`.
    pub board: Board,
    modem: Option<Modem>,
    main_firmware_baud: u32,
    client: AtClient,
}

impl<P: Port> Session<P> {
    /// `new TSession(Port, MainFirmwareBaud)`.
    /// `// C#: SikRadio/RFD900.cs:31-45`
    #[must_use]
    pub fn new(wire: Wire<P>, main_firmware_baud: u32) -> Self {
        Self {
            wire,
            mode: Mode::Init,
            board: Board::FAILED,
            modem: None,
            main_firmware_baud,
            client: AtClient::default(),
        }
    }

    /// The port, as `Session.Port`.
    pub fn wire(&mut self) -> &mut Wire<P> {
        &mut self.wire
    }

    /// The port, to look at.
    #[must_use]
    pub const fn wire_ref(&self) -> &Wire<P> {
        &self.wire
    }

    /// The port back, the session ended.
    #[must_use]
    pub fn into_wire(self) -> Wire<P> {
        self.wire
    }

    /// The baud rate the radio's firmware runs at.
    #[must_use]
    pub const fn main_firmware_baud(&self) -> u32 {
        self.main_firmware_baud
    }

    /// The mode as last known, without finding it out.
    #[must_use]
    pub const fn mode(&self) -> Mode {
        self.mode
    }

    /// The modem object, if made.
    #[must_use]
    pub const fn modem(&self) -> Option<Modem> {
        self.modem
    }

    /// `ATCClient.DoQuery`.
    ///
    /// # Errors
    /// The port's.
    pub fn query(&mut self, command: &str, wait_for_terminator: bool) -> io::Result<String> {
        self.client
            .do_query(&mut self.wire, command, wait_for_terminator)
    }

    /// `ATCClient.DoCommand`.
    ///
    /// # Errors
    /// The port's.
    pub fn command(&mut self, command: &str) -> io::Result<bool> {
        self.client.do_command(&mut self.wire, command)
    }

    /// `WaitForToken`.
    fn wait_for(&mut self, token: &str, max_wait_ms: u64) -> io::Result<bool> {
        at::wait_for(&mut self.wire, token, max_wait_ms)
    }

    /// `WriteBootloaderCode`.
    fn write_code(&mut self, code: u8) -> io::Result<()> {
        self.wire.write(&[code])
    }

    /// `IsInBootloaderMode`: at the bootloader's baud rate, `GET_SYNC` answered with `INSYNC`;
    /// the firmware's baud rate put back when not.
    /// `// C#: SikRadio/RFD900.cs:145-168`
    fn is_in_bootloader_mode(&mut self) -> io::Result<bool> {
        self.wire.set_baud(BOOTLOADER_BAUD)?;
        self.wire.sleep(100);
        self.write_code(code::EOC)?;
        self.wire.sleep(100);
        self.wire.discard_in_buffer()?;
        self.write_code(code::GET_SYNC)?;
        self.write_code(code::EOC)?;
        let result = matches!(self.wire.read_byte(), Ok(Some(code::INSYNC)));
        if !result {
            self.wire.set_baud(self.main_firmware_baud)?;
        }
        Ok(result)
    }

    /// `IsInBootloaderXMode`: at 57600, a `U` to set the baud rate, then `CHIPID` answered with
    /// "RFD" within 200 ms; the baud rate put back when not.
    /// `// C#: SikRadio/RFD900.cs:174-196`
    fn is_in_bootloader_x_mode(&mut self) -> io::Result<bool> {
        let previous = self.wire.baud();
        self.wire.set_baud(BOOTLOADER_X_BAUD)?;
        self.wire.sleep(200);
        // A capital U, to sync the baud rate.
        self.wire.write_str("U")?;
        self.wire.sleep(200);
        self.wire.write_str("\r\n")?;
        self.wire.sleep(200);
        self.wire.discard_in_buffer()?;
        self.wire.write_str("CHIPID\r\n")?;
        if self.wait_for("RFD", 200)? {
            // Left at the bootloader's baud rate.
            return Ok(true);
        }
        self.wire.set_baud(previous)?;
        Ok(false)
    }

    /// `TryEscapeFromTransparent`: a second and a half of quiet, `+++`, and "OK" within 1.5 s.
    /// `// C#: SikRadio/RFD900.cs:198-208`
    fn try_escape_from_transparent(&mut self) -> io::Result<bool> {
        self.wire.set_read_timeout_ms(2000);
        self.wire.sleep(1500);
        self.wire.discard_in_buffer()?;
        self.wire.write_str("+++")?;
        self.wait_for("OK\r\n", 1500)
    }

    /// `DetermineMode`: the RFD900x's bootloader; AT command mode already, which an RFD radio
    /// says by answering `ATI` with "RFD"; `+++` twice; the bootloaders again.
    /// `// C#: SikRadio/RFD900.cs:210-242`
    fn determine_mode(&mut self) -> io::Result<Mode> {
        if self.is_in_bootloader_x_mode()? {
            return Ok(Mode::BootloaderX);
        }
        // Already in AT command mode?
        self.wire.discard_in_buffer()?;
        self.wire.write_str("\r\n")?;
        self.wire.sleep(100);
        self.wire.write_str("ATI\r\n")?;
        if self.wait_for("RFD", 400)? {
            return Ok(Mode::AtCommand);
        }
        if self.try_escape_from_transparent()? || self.try_escape_from_transparent()? {
            return Ok(Mode::AtCommand);
        }
        // Neither transparent nor AT command mode: probably a bootloader.
        if self.is_in_bootloader_x_mode()? {
            return Ok(Mode::BootloaderX);
        }
        if self.is_in_bootloader_mode()? {
            return Ok(Mode::Bootloader);
        }
        Ok(Mode::Unknown)
    }

    /// `CheckIfInBootloaderMode`.
    /// `// C#: SikRadio/RFD900.cs:244-254`
    fn check_if_in_bootloader_mode(&mut self) -> io::Result<()> {
        if self.is_in_bootloader_x_mode()? {
            self.mode = Mode::BootloaderX;
        } else if self.is_in_bootloader_mode()? {
            self.mode = Mode::Bootloader;
        }
        Ok(())
    }

    /// `GetMode`: the mode, found out the first time.
    /// `// C#: SikRadio/RFD900.cs:345-352`
    ///
    /// # Errors
    /// The port's.
    pub fn get_mode(&mut self) -> io::Result<Mode> {
        if self.mode == Mode::Init {
            self.mode = self.determine_mode()?;
        }
        Ok(self.mode)
    }

    /// `GetModemObject`: the model `ATI2` names in AT command mode (and DINIO when `ATI` says
    /// it), or the one a bootloader names; `None` when it cannot be had.
    /// `// C#: SikRadio/RFD900.cs:277-343`
    ///
    /// # Errors
    /// The port's, and a bootloader's that does not answer, as the C# throws them.
    pub fn get_modem_object(&mut self) -> io::Result<Option<Modem>> {
        if self.modem.is_none() {
            match self.get_mode()? {
                Mode::Transparent | Mode::AtCommand => {
                    if self.mode == Mode::Transparent
                        && self.put_into_at_command_mode()? != Mode::AtCommand
                    {
                        return Ok(None);
                    }
                    let result = self.query("ATI2", true)?;
                    // `(Uploader.Board)int.Parse(Result)`: an unchecked cast to the byte.
                    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
                    if let Some(code) = crate::try_parse_int(&result) {
                        self.modem = Modem::for_board(Board(code as u8));
                    }
                    if self.modem.is_some() {
                        let ati = self.query("ATI", true)?;
                        if ati.contains("DINIO")
                            && let Some(modem) = self.modem.as_mut()
                        {
                            modem.dinio = true;
                        }
                    }
                }
                Mode::Bootloader => self.modem = self.bootloader_object()?,
                Mode::BootloaderX => self.modem = self.bootloader_x_object()?,
                Mode::Init | Mode::Unknown => {}
            }
        }
        Ok(self.modem)
    }

    /// `RFD900APU.GetObjectForModem`: the SiK bootloader's device id, of the a, u or + only.
    /// `// C#: SikRadio/RFD900.cs:2044-2064`
    fn bootloader_object(&mut self) -> io::Result<Option<Modem>> {
        let mut ignore = |_| {};
        let (board, _) = Uploader::new(&mut self.wire, &mut ignore)
            .get_device()
            .map_err(io::Error::other)?;
        Ok(match board {
            Board::RFD900A | Board::RFD900P | Board::RFD900U => Modem::for_board(board),
            _ => None,
        })
    }

    /// `RFD900xuxRevN.GetObjectForModem`: `CHIPID`'s answer.
    /// `// C#: SikRadio/RFD900.cs:2144-2164`
    fn bootloader_x_object(&mut self) -> io::Result<Option<Modem>> {
        self.wire.write_str("\r\n")?;
        self.wire.sleep(200);
        self.wire.discard_in_buffer()?;
        self.wire.write_str("CHIPID\r\n")?;
        let tokens = [
            "RFD900xSub:1",
            "RFD900uxSub:1",
            "RFD900xSub:2",
            "RFD900uxSub:2",
        ];
        let board = match at::wait_for_any_token(&mut self.wire, &tokens, 200)? {
            Some(0) => Board::RFD900X,
            Some(1) => Board::RFD900UX,
            Some(2) => Board::RFD900X2,
            Some(3) => Board::RFD900UX2,
            _ => return Ok(None),
        };
        Ok(Modem::for_board(board))
    }

    /// `PutIntoBootloaderMode`: from AT command mode, `AT&UPDATE` and the bootloaders looked
    /// for.
    /// `// C#: SikRadio/RFD900.cs:354-376`
    ///
    /// # Errors
    /// The port's.
    pub fn put_into_bootloader_mode(&mut self) -> io::Result<Mode> {
        match self.get_mode()? {
            Mode::Bootloader | Mode::BootloaderX => {}
            _ => {
                if self.put_into_at_command_mode()? == Mode::AtCommand {
                    self.wire.write_str("\r\n")?;
                    self.wire.sleep(100);
                    self.wire.write_str("AT&UPDATE\r\n")?;
                    self.wire.sleep(100);
                    self.check_if_in_bootloader_mode()?;
                }
            }
        }
        self.get_mode()
    }

    /// `PutIntoATCommandMode`: `+++` from transparent mode; a bootloader rebooted into the
    /// firmware, whose mode is then found out again.
    /// `// C#: SikRadio/RFD900.cs:378-403`
    ///
    /// # Errors
    /// The port's.
    pub fn put_into_at_command_mode(&mut self) -> io::Result<Mode> {
        match self.get_mode()? {
            Mode::Transparent => {
                return self.put_into_at_command_mode_assuming_in_transparent_mode();
            }
            Mode::Bootloader => {
                self.wire.set_baud(BOOTLOADER_BAUD)?;
                self.write_code(code::REBOOT)?;
                self.wire.set_baud(self.main_firmware_baud)?;
                self.mode = Mode::Init;
            }
            Mode::BootloaderX => {
                // Boot.
                self.wire.sleep(100);
                self.wire.write_str("\r\n")?;
                self.wire.sleep(100);
                self.wire.write_str("RESET\r\n")?;
                self.wire.sleep(100);
                self.wire.set_baud(self.main_firmware_baud)?;
                self.mode = Mode::Init;
            }
            _ => {}
        }
        self.get_mode()
    }

    /// `PutIntoATCommandModeAssumingInTransparentMode`: a second and a half of quiet, `+++`, "OK"
    /// within three seconds; else the mode found out again.
    /// `// C#: SikRadio/RFD900.cs:409-425`
    ///
    /// # Errors
    /// The port's.
    pub fn put_into_at_command_mode_assuming_in_transparent_mode(&mut self) -> io::Result<Mode> {
        self.wire.set_read_timeout_ms(2000);
        self.wire.sleep(1500);
        self.wire.discard_in_buffer()?;
        self.wire.write_str("+++")?;
        if self.wait_for("OK\r\n", 3000)? {
            self.mode = Mode::AtCommand;
            return Ok(Mode::AtCommand);
        }
        self.mode = Mode::Init;
        self.put_into_at_command_mode()
    }

    /// `PutIntoTransparentMode`: from AT command mode, `ATO`.
    /// `// C#: SikRadio/RFD900.cs:432-464`
    ///
    /// # Errors
    /// The port's.
    pub fn put_into_transparent_mode(&mut self) -> io::Result<Mode> {
        if self.get_mode()? == Mode::Transparent {
            return Ok(Mode::Transparent);
        }
        if self.put_into_at_command_mode()? != Mode::AtCommand {
            return Ok(Mode::Unknown);
        }
        self.wire.write_str("\r\n")?;
        self.wire.sleep(100);
        self.wire.write_str("ATO\r\n")?;
        self.wire.sleep(100);
        // Only logged in the C#.
        self.wait_for("ATO\r\n", 100)?;
        self.mode = Mode::Transparent;
        Ok(Mode::Transparent)
    }

    /// `AssumeMode`: the mode taken as known, the port at its baud rate.
    /// `// C#: SikRadio/RFD900.cs:466-476`
    ///
    /// # Errors
    /// The port's.
    pub fn assume_mode(&mut self, mode: Mode) -> io::Result<()> {
        let baud = if mode == Mode::BootloaderX {
            BOOTLOADER_X_BAUD
        } else {
            self.main_firmware_baud
        };
        self.wire.set_baud(baud)?;
        self.mode = mode;
        Ok(())
    }

    /// `GetEncryptionKey`: `&E?`'s answer when it is hex - an empty answer is - as the `AESKEY`
    /// text setting.
    /// `// C#: SikRadio/RFD900.cs:1091-1116`
    fn encryption_key(&mut self, remote: bool) -> io::Result<Option<Setting>> {
        let temp = self.query(if remote { "RT&E?" } else { "AT&E?" }, true)?;
        if !temp.contains("OK")
            && !temp.contains("ERROR")
            && temp.chars().all(|c| c.is_ascii_hexdigit())
        {
            return Ok(Some(Setting::text("&E", KEY_SETTING, temp)));
        }
        Ok(None)
    }

    /// `UseATI10ToGetATI5QueryResponse`: `ATI10:0`, `ATI10:1`, ... as `ATI5?`'s lines, until a
    /// line without `=` or saying "eof"; `None` at an error or no answer.
    /// `// C#: SikRadio/RFD900.cs:1246-1270`
    fn use_ati10(&mut self, remote: bool) -> io::Result<Option<String>> {
        let prefix = if remote { "RTI10:" } else { "ATI10:" };
        let mut result = String::new();
        for index in 0..ATI10_LINES {
            let line = self.query(&format!("{prefix}{index}"), true)?;
            let lower = line.to_lowercase();
            if lower.contains("error") || line.is_empty() {
                return Ok(None);
            }
            if lower.contains("eof") || !line.contains('=') {
                return Ok(Some(result));
            }
            result.push_str(&line);
            result.push_str("\r\n");
        }
        Ok(Some(result))
    }

    /// `GetSettings(Remote, Board, ATI5Response, Ranges, out UseRanges)`: the ranges asked for -
    /// `ATI10` or `ATI5?` - and [`settings::get_settings`] over them and the `ATI5` answer given,
    /// with the key when the radio gives one.
    /// `// C#: SikRadio/RFD900.cs:1278-1297`
    ///
    /// # Errors
    /// The port's.
    pub fn get_settings(
        &mut self,
        remote: bool,
        board: Board,
        response: &str,
        ranges: Option<&Settings>,
    ) -> io::Result<(Settings, bool)> {
        let query = match self.use_ati10(remote)? {
            Some(lines) if !lines.is_empty() => lines,
            _ => self.client.do_query_with_multi_line_response(
                &mut self.wire,
                if remote { "RTI5?" } else { "ATI5?" },
            )?,
        };
        let (mut result, used) =
            settings::get_settings(&query, board == Board::RFD900X, response, ranges);
        if let Some(key) = self.encryption_key(remote)? {
            result.insert(KEY_SETTING, key);
        }
        Ok((result, used))
    }

    /// `RFD900xuxRevN.GetCountryCode` / `GetRemoteCountryCode`: `AT+Cnn?`'s number, `NONE` when
    /// it is not one.
    /// `// C#: SikRadio/RFD900.cs:2175-2196, 2225-2228`
    fn country(&mut self, modem: Modem, remote: bool) -> io::Result<Option<Country>> {
        let Some(register) = modem.country_register() else {
            return Ok(None);
        };
        let reply = self.query(
            &format!("{}T+C{register}?", if remote { "R" } else { "A" }),
            true,
        )?;
        Ok(Some(
            crate::try_parse_int(&reply).map_or(Country::NONE, Country),
        ))
    }

    /// The page's `GetCountryCodeFromSession`: the country an RFD900x-family modem is locked to,
    /// "--" for any other modem or none.
    /// `// C#: Radio/Sikradio.cs:1445-1460`
    ///
    /// # Errors
    /// The port's.
    pub fn country_text(&mut self, remote: bool) -> io::Result<String> {
        let Some(modem) = self.get_modem_object()? else {
            return Ok("--".to_owned());
        };
        Ok(match self.country(modem, remote)? {
            Some(country) if country.is_locked() => country.to_string(),
            _ => "--".to_owned(),
        })
    }

    /// `RFD900.ProgramFirmware` - `RFD900xux`'s override for the RFD900x and ux - over the file:
    /// true when it is in the radio.
    /// `// C#: SikRadio/RFD900.cs:1795-1824, 2433-2504`
    ///
    /// # Errors
    /// What the C# throws out of it to the page's `catch`: the port's, a file that cannot be
    /// read where the C# reads it outside a `catch`, and the missing reference firmware.
    pub fn program_firmware(
        &mut self,
        modem: Modem,
        path: &Path,
        report: &mut dyn Report,
    ) -> io::Result<bool> {
        if modem.family() == Family::Xux {
            return self.program_xux(modem, path, report);
        }
        self.program(modem, path, report)
    }

    /// `RFD900xux.ProgramFirmware`: a certified file goes on; else, a country-locked modem
    /// takes only certified firmware.
    /// `// C#: SikRadio/RFD900.cs:2433-2504`
    fn program_xux(
        &mut self,
        modem: Modem,
        path: &Path,
        report: &mut dyn Report,
    ) -> io::Result<bool> {
        let bytes = std::fs::read(path)?;
        if modem::is_certified(&bytes) {
            return self.program(modem, path, report);
        }
        report.status("Trying to put into AT command mode.");
        if self.put_into_at_command_mode()? != Mode::AtCommand {
            report.status("Trying to put into bootloader mode.");
            if self.put_into_bootloader_mode()? == Mode::BootloaderX {
                report.status("Programming temporary firmware into modem to check certified.");
                // `GetRefFirmwarePath` reads a resource the planner does not embed, and throws.
                return Err(io::Error::other(NO_REFERENCE_FIRMWARE));
            }
            return Ok(false);
        }
        report.status("Trying to put into AT command mode.");
        if self.put_into_at_command_mode()? != Mode::AtCommand {
            return Ok(false);
        }
        report.status("Checking if modem certified.");
        let locked = self.country(modem, false)?.is_some_and(Country::is_locked);
        if !locked {
            // They can program whatever they want into it.
            return self.program(modem, path, report);
        }
        // Only certified firmware, which this is not.
        report.message("The selected firmware is not certified to run on this modem.  Aborting.");
        Ok(false)
    }

    /// `RFD900.ProgramFirmware`: the file checked, the bootloader entered, the firmware put in.
    /// `// C#: SikRadio/RFD900.cs:1795-1817`
    fn program(&mut self, modem: Modem, path: &Path, report: &mut dyn Report) -> io::Result<bool> {
        if !self.check_firmware(modem, path, report)? {
            report.status("Incorrect firmware selected.");
            self.wire.sleep(2000);
            return Ok(false);
        }
        report.status("Putting into bootloader mode.");
        match self.put_into_bootloader_mode()? {
            Mode::Bootloader | Mode::BootloaderX => {
                report.status("Programming selected firmware into modem.");
                self.do_firmware_programming(modem, path, report)
            }
            _ => {
                report.status("Failed to put into bootloader mode.");
                Ok(false)
            }
        }
    }

    /// `CheckFirmwareIncludingFileNameOK`: a DINIO modem's file named DINIO, and the model's
    /// check - the image holds a search token (a box when not), any file for the Rev2 models.
    /// `// C#: SikRadio/RFD900.cs:1747-1757, 1983-2006, 2334-2345, 2626-2629`
    fn check_firmware(
        &mut self,
        modem: Modem,
        path: &Path,
        report: &mut dyn Report,
    ) -> io::Result<bool> {
        let named = path
            .file_name()
            .is_some_and(|name| name.to_string_lossy().contains("DINIO"));
        if modem.dinio && !named {
            return Ok(false);
        }
        let found = match modem.family() {
            Family::Rev2 => return Ok(true),
            // A file that will not load is false, without the box.
            Family::Apu => match std::fs::read_to_string(path)
                .ok()
                .and_then(|text| IHex::parse(&text).ok())
            {
                Some(image) => modem::search_hex(&image, modem.search_tokens()),
                None => return Ok(false),
            },
            Family::Xux => modem::search_binary(&std::fs::read(path)?, modem.search_tokens()),
        };
        if !found {
            report.message(&modem.wrong_firmware_text());
        }
        Ok(found)
    }

    /// `DoFirmwareProgramming`: the SiK bootloader's upload of the Intel HEX file; or XModem,
    /// asked again "Programming firmware failed.  Try again?" each time it fails.
    /// `// C#: SikRadio/RFD900.cs:2012-2029, 2313-2332`
    fn do_firmware_programming(
        &mut self,
        modem: Modem,
        path: &Path,
        report: &mut dyn Report,
    ) -> io::Result<bool> {
        if modem.family() == Family::Apu {
            let Some(image) = std::fs::read_to_string(path)
                .ok()
                .and_then(|text| IHex::parse(&text).ok())
            else {
                return Ok(false);
            };
            let mut progress = |d: f64| report.progress(d);
            if Uploader::new(&mut self.wire, &mut progress)
                .upload(&image)
                .is_err()
            {
                return Ok(false);
            }
            self.assume_mode(Mode::Transparent)?;
            return Ok(true);
        }
        loop {
            if self.try_programming_once(modem, path, report)? {
                return Ok(true);
            }
            if !report.try_again() {
                return Ok(false);
            }
        }
    }

    /// `TryFirmwareProgrammingOnce`: a Rev2 model at the higher baud rate for it.
    /// `// C#: SikRadio/RFD900.cs:2289-2304, 2636-2655`
    fn try_programming_once(
        &mut self,
        modem: Modem,
        path: &Path,
        report: &mut dyn Report,
    ) -> io::Result<bool> {
        let higher = if modem.family() == Family::Rev2 {
            self.wire.write_str("BAUDHI\r\n")?;
            let ok = self.wait_for("OK", 500)?;
            if ok {
                self.wire.set_baud(1_200_000)?;
            }
            ok
        } else {
            false
        };
        let result = self.do_programming(path, report)?;
        if higher {
            self.wire.write_str("BAUDLO\r\n")?;
            self.wait_for("OK", 500)?;
            self.wire.set_baud(57_600)?;
        }
        Ok(result)
    }

    /// `DoProgramming`: `UPLOAD`, "Ready" and "C", the XModem upload, and the mode to be found
    /// out again.
    /// `// C#: SikRadio/RFD900.cs:2263-2287`
    fn do_programming(&mut self, path: &Path, report: &mut dyn Report) -> io::Result<bool> {
        self.wire.write_str("\r")?;
        self.wire.sleep(100);
        self.wire.discard_in_buffer()?;
        self.wire.write_str("UPLOAD\r")?;
        if !(self.wait_for("Ready\r\n", 5000)? && self.wait_for("C", 5000)?) {
            return Ok(false);
        }
        // `FileMode.OpenOrCreate`: a file not there is an empty one.
        let firmware = std::fs::read(path).unwrap_or_default();
        let mut progress = |d: f64| report.progress(d);
        let Ok(result) = xmodem::upload(&firmware, &mut self.wire, &mut progress) else {
            return Ok(false);
        };
        self.assume_mode(Mode::Init)?;
        Ok(result)
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use crate::page::tests::Heard;
    use crate::radio::{Mode as RadioMode, Radio, Unit};

    pub(crate) fn session(radio: Radio) -> Session<Radio> {
        Session::new(Wire::new(radio), 57_600)
    }

    /// From transparent mode: not the RFD900x's bootloader, no answer to `ATI`, then `+++`.
    #[test]
    fn plus_plus_plus_finds_command_mode() {
        let mut s = session(Radio::rfd900p());
        assert_eq!(s.get_mode().unwrap(), Mode::AtCommand);
        assert_eq!(s.wire().port().mode(), RadioMode::Command);
        assert_eq!(s.wire().baud(), 57_600, "the firmware's baud rate put back");
        assert_eq!(s.put_into_transparent_mode().unwrap(), Mode::Transparent);
        assert_eq!(s.wire().port().mode(), RadioMode::Transparent);
        assert_eq!(
            s.put_into_at_command_mode().unwrap(),
            Mode::AtCommand,
            "+++ from transparent"
        );
    }

    /// An RFD radio already in AT command mode says so to `ATI`.
    #[test]
    fn an_rfd_in_command_mode_answers_ati() {
        let mut radio = Radio::rfd900p();
        radio.enter_command_mode();
        let mut s = session(radio);
        assert_eq!(s.get_mode().unwrap(), Mode::AtCommand);
        assert!(
            !String::from_utf8_lossy(&s.wire().port().written).contains("+++"),
            "no escape needed"
        );
    }

    /// The bootloaders: the RFD900x's at 57600, the SiK one at 115200.
    #[test]
    fn the_bootloaders_are_found() {
        let mut radio = Radio::with(Unit::rfd900x(), None);
        radio.enter(RadioMode::BootloaderX);
        let mut s = session(radio);
        assert_eq!(s.get_mode().unwrap(), Mode::BootloaderX);
        assert_eq!(s.wire().baud(), 57_600);
        assert_eq!(
            s.get_modem_object().unwrap().map(|m| m.board()),
            Some(Board::RFD900X)
        );

        let mut radio = Radio::rfd900p();
        radio.enter(RadioMode::Bootloader);
        let mut s = session(radio);
        assert_eq!(s.get_mode().unwrap(), Mode::Bootloader);
        assert_eq!(s.wire().baud(), 115_200, "left at the bootloader's");
        assert_eq!(
            s.get_modem_object().unwrap().map(|m| m.board()),
            Some(Board::RFD900P)
        );
        assert_eq!(
            s.put_into_at_command_mode().unwrap(),
            Mode::AtCommand,
            "rebooted"
        );
    }

    /// A radio that answers nothing is UNKNOWN, and stays so.
    #[test]
    fn a_silent_radio_is_unknown() {
        let mut radio = Radio::rfd900p();
        radio.serial_baud = 9600;
        let mut s = session(radio);
        assert_eq!(s.get_mode().unwrap(), Mode::Unknown);
        assert_eq!(s.put_into_at_command_mode().unwrap(), Mode::Unknown);
        assert_eq!(s.get_modem_object().unwrap(), None);
        assert_eq!(Mode::Unknown.to_string(), "UNKNOWN");
    }

    /// The modem object from `ATI2`; an HM-TRP has none.
    #[test]
    fn the_modem_object_is_the_boards() {
        let mut s = session(Radio::rfd900p());
        assert_eq!(
            s.get_modem_object().unwrap(),
            Some(Modem::for_board(Board::RFD900P).unwrap())
        );
        let mut s = session(Radio::with(Unit::hm_trp(), None));
        s.mode = Mode::AtCommand;
        s.wire().port_mut().enter_command_mode();
        assert_eq!(s.get_modem_object().unwrap(), None);
        assert_eq!(s.country_text(false).unwrap(), "--");
    }

    /// `ATI5?` when `ATI10` is refused, with the key; and `ATI10` when the radio has it.
    #[test]
    fn settings_come_with_their_ranges_and_the_key() {
        let mut radio = Radio::rfd900p();
        radio.enter_command_mode();
        let mut s = session(radio);
        s.mode = Mode::AtCommand;
        let ati5 = s
            .client
            .do_query_with_multi_line_response(&mut s.wire, "ATI5")
            .unwrap();
        let (settings, used) = s.get_settings(false, Board::RFD900P, &ati5, None).unwrap();
        assert!(!used);
        assert_eq!(
            settings
                .get("AIR_SPEED")
                .unwrap()
                .option_names()
                .unwrap()
                .len(),
            13
        );
        assert_eq!(
            settings.get(KEY_SETTING).unwrap().text_value(),
            Some("00000000000000000000000000000000")
        );
        assert!(s.wire().port().commands().iter().any(|c| c == "ATI5?"));

        s.wire().port_mut().local.ati10 = true;
        let (again, _) = s.get_settings(false, Board::RFD900P, &ati5, None).unwrap();
        assert_eq!(again.get("NETID"), settings.get("NETID"));
        assert!(
            s.wire().port().commands().iter().any(|c| c == "ATI10:19"),
            "to the EOF"
        );
    }

    /// An RFD900+ programmed from an Intel HEX file holding its name, over the SiK bootloader.
    #[test]
    fn an_rfd900p_takes_its_hex() {
        let dir = std::env::temp_dir().join(format!("mp-sikradio-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let file = dir.join("rfd900p.ihx");
        // "RFD900P" at 0x10.
        std::fs::write(&file, ":0700100052464439303050E1\n:0400000001020304F2\n").unwrap();
        let mut s = session(Radio::rfd900p());
        let modem = s.get_modem_object().unwrap().unwrap();
        let mut heard = Heard::default();
        assert!(s.program_firmware(modem, &file, &mut heard).unwrap());
        assert_eq!(
            heard.statuses,
            [
                "Putting into bootloader mode.",
                "Programming selected firmware into modem."
            ]
        );
        assert!(heard.messages.is_empty());
        assert_eq!(s.mode(), Mode::Transparent);
        assert_eq!(s.wire().port().flash.get(&0x10), Some(&b'R'));

        // A file for another model: the box, and "Incorrect firmware selected."
        let wrong = dir.join("rfd900a.ihx");
        std::fs::write(&wrong, ":0700100052464439303041E1\n").unwrap();
        let mut heard = Heard::default();
        assert!(!s.program_firmware(modem, &wrong, &mut heard).unwrap());
        assert_eq!(heard.messages, [modem.wrong_firmware_text()]);
        assert_eq!(heard.statuses, ["Incorrect firmware selected."]);
        std::fs::remove_dir_all(&dir).ok();
    }

    /// An RFD900x not locked to a country takes a firmware holding its name, over XModem.
    #[test]
    fn an_rfd900x_takes_its_bin_over_xmodem() {
        let dir = std::env::temp_dir().join(format!("mp-sikradio-x-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let file = dir.join("rfd900x.bin");
        let mut firmware = vec![0u8; 200];
        firmware[50..57].copy_from_slice(b"RFD900X");
        std::fs::write(&file, &firmware).unwrap();
        let mut s = session(Radio::with(Unit::rfd900x(), None));
        let modem = s.get_modem_object().unwrap().unwrap();
        let mut heard = Heard::default();
        assert!(s.program_firmware(modem, &file, &mut heard).unwrap());
        assert_eq!(
            heard.statuses,
            [
                "Trying to put into AT command mode.",
                "Trying to put into AT command mode.",
                "Checking if modem certified.",
                "Putting into bootloader mode.",
                "Programming selected firmware into modem."
            ]
        );
        assert_eq!(&s.wire().port().received[..200], firmware.as_slice());
        assert_eq!(s.mode(), Mode::Init, "found out again");

        // Locked to a country, an uncertified file is refused with the box.
        let mut s = session(Radio::with(Unit::rfd900x(), None));
        s.wire().port_mut().local.country = Some(3);
        let modem = s.get_modem_object().unwrap().unwrap();
        assert_eq!(s.country_text(false).unwrap(), "US");
        let mut heard = Heard::default();
        assert!(!s.program_firmware(modem, &file, &mut heard).unwrap());
        assert_eq!(
            heard.messages,
            ["The selected firmware is not certified to run on this modem.  Aborting."]
        );
        std::fs::remove_dir_all(&dir).ok();
    }
}
