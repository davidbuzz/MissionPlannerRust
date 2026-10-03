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

//! A SiK radio for the tests: the port, on a clock of its own, and the local radio behind it with
//! the remote radio it reaches over the air.
//!
//! What it does is what the C# expects of one - the parts the page's code depends on:
//!
//! * transparent mode until `+++` arrives after a second of quiet, then "OK" a second later and
//!   AT command mode; bytes at the wrong baud rate are noise;
//! * in AT command mode, every printable character echoed and a carriage return echoed as `\r\n`
//!   and acted on: `ATI`, `ATI2`, `ATI3`, `ATI5`, `ATI5?`, `ATI7`, `ATI10:n`, `AT&E?`, `AT&E=`,
//!   `ATSn=v`, `AT&W`, `AT&F`, `ATZ`, `ATO`, `AT&T`, `AT&R`, `AT+Cnn?`, `ATPO=1`, `ATPI=1`,
//!   `AT&UPDATE`; `RT...` is the same asked of the remote radio, when there is one;
//! * the SiK bootloader at 115200 baud (`GET_SYNC`, `GET_DEVICE`, `CHIP_ERASE`, `LOAD_ADDRESS`,
//!   `PROG_MULTI`, `READ_MULTI`, `REBOOT`), and the RFD900x bootloader at 57600 (`CHIPID`,
//!   `RESET`, `UPLOAD` and an XModem-CRC receive, `BOOTNEW`).

use std::collections::{BTreeMap, VecDeque};
use std::io;
use std::time::Duration;

use crate::Port;

/// How long a radio takes to answer.
const LATENCY: Duration = Duration::from_millis(10);
/// How long the remote radio's answer takes over the air.
const AIR: Duration = Duration::from_millis(80);
/// The quiet `+++` needs on each side.
const GUARD: Duration = Duration::from_millis(1000);

/// One register.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct Param {
    pub(crate) number: u8,
    pub(crate) name: String,
    pub(crate) value: i32,
    pub(crate) default: i32,
    /// What `ATI5?` adds after the name: `(N)[1..115]`.
    pub(crate) range: String,
    /// What `ATI5?` adds after the value: `{...}`.
    pub(crate) options: String,
}

fn param(number: u8, name: &str, value: i32, range: &str, options: &str) -> Param {
    Param {
        number,
        name: name.to_owned(),
        value,
        default: value,
        range: range.to_owned(),
        options: options.to_owned(),
    }
}

/// One radio.
#[derive(Debug, Clone)]
pub(crate) struct Unit {
    pub(crate) ati: String,
    pub(crate) board: u8,
    pub(crate) freq: u8,
    pub(crate) params: Vec<Param>,
    /// What `&W` saved, read back at a reboot.
    pub(crate) eeprom: Vec<i32>,
    /// `&E?`'s answer; `None` is "ERROR".
    pub(crate) key: Option<String>,
    /// Whether `ATI5?` gives the ranges; otherwise it is `ATI5` (the firmware reads `ATI5`).
    pub(crate) ranged: bool,
    /// Whether `ATI10:n` gives the lines; otherwise "ERROR".
    pub(crate) ati10: bool,
    pub(crate) rssi: String,
    /// `+Cnn?`'s answer.
    pub(crate) country: Option<i32>,
    /// A multipoint radio's `[n]` before each answer.
    pub(crate) node: Option<String>,
    pub(crate) ppm_recorded: bool,
    pub(crate) reboots: u32,
    /// Whether `&UPDATE` starts the RFD900x's bootloader rather than the SiK one.
    pub(crate) x_bootloader: bool,
}

impl Unit {
    /// An RFD900+ on RFD's SiK 2.65, 915 MHz.
    pub(crate) fn rfd900p() -> Self {
        let params = vec![
            param(0, "FORMAT", 25, "(R)[0..255]", ""),
            param(
                1,
                "SERIAL_SPEED",
                57,
                "(N)[1..115]",
                "{1200,2400,4800,9600,19200,38400,57600,115200,}",
            ),
            param(
                2,
                "AIR_SPEED",
                64,
                "(N)[2..250]",
                "{2,4,8,16,19,24,32,48,64,96,128,192,250,}",
            ),
            param(3, "NETID", 25, "(N)[0..499]", ""),
            param(4, "TXPOWER", 30, "(N)[0..30]", ""),
            param(5, "ECC", 0, "(N)[0..1]", ""),
            param(
                6,
                "MAVLINK",
                1,
                "(N)[0..2]",
                "{RawData,Mavlink,LowLatency,}",
            ),
            param(7, "OPPRESEND", 0, "(N)[0..1]", ""),
            param(8, "MIN_FREQ", 915_000, "(N)[902000..927000]", ""),
            param(9, "MAX_FREQ", 928_000, "(N)[903000..928000]", ""),
            param(10, "NUM_CHANNELS", 20, "(N)[1..50]", ""),
            param(11, "DUTY_CYCLE", 100, "(N)[10..100]", ""),
            param(12, "LBT_RSSI", 0, "(N)[0..220]", ""),
            param(13, "MANCHESTER", 0, "(N)[0..1]", ""),
            param(14, "RTSCTS", 0, "(N)[0..1]", ""),
            param(15, "MAX_WINDOW", 131, "(N)[20..400]", ""),
            param(16, "ENCRYPTION_LEVEL", 0, "(N)[0..1]", "{Off,128b,}"),
            param(17, "GPI1_1R/CIN", 0, "(N)[0..1]", ""),
            param(18, "GPO1_1R/COUT", 0, "(N)[0..1]", ""),
        ];
        let eeprom = params.iter().map(|p| p.value).collect();
        Self {
            ati: "RFD SiK 2.65 on RFD900P".to_owned(),
            board: 0x82,
            freq: 0x91,
            params,
            eeprom,
            key: Some("00000000000000000000000000000000".to_owned()),
            ranged: true,
            ati10: false,
            rssi: "L/R RSSI: 210/198  L/R noise: 45/40 pkts: 120  txe=0 rxe=0 stx=0 srx=0 ecc=0/0 temp=31 dco=0".to_owned(),
            country: None,
            node: None,
            ppm_recorded: false,
            reboots: 0,
            x_bootloader: false,
        }
    }

    /// An RFD900x on RFD's SiK 3.07: its bootloader is XModem's.
    pub(crate) fn rfd900x() -> Self {
        let mut unit = Self::rfd900p();
        unit.ati = "RFD SiK 3.07 on RFD900X".to_owned();
        unit.board = 0x83;
        unit.x_bootloader = true;
        unit
    }

    /// A 3DR radio on ArduPilot's SiK 1.9: no ranges, no key, the board an HM-TRP.
    pub(crate) fn hm_trp() -> Self {
        let mut unit = Self::rfd900p();
        unit.ati = "SiK 1.9 on HM-TRP".to_owned();
        unit.board = 0x4E;
        unit.params.truncate(16);
        unit.params.iter_mut().for_each(|p| {
            p.range.clear();
            p.options.clear();
        });
        unit.eeprom.truncate(16);
        unit.key = None;
        unit.ranged = false;
        unit
    }

    /// The register called `name`.
    pub(crate) fn value(&self, name: &str) -> Option<i32> {
        self.params.iter().find(|p| p.name == name).map(|p| p.value)
    }

    fn prefixed(&self, line: &str) -> String {
        match &self.node {
            Some(node) => format!("{node}{line}"),
            None => line.to_owned(),
        }
    }

    fn plain_lines(&self) -> String {
        self.params
            .iter()
            .map(|p| self.prefixed(&format!("S{}:{}={}\r\n", p.number, p.name, p.value)))
            .collect()
    }

    fn query_line(&self, p: &Param) -> String {
        self.prefixed(&format!(
            "S{}:{}{}={}{}",
            p.number, p.name, p.range, p.value, p.options
        ))
    }

    /// The answer to `AT` + `rest` (`rest` upper case), and what it does to the radio.
    fn act(&mut self, rest: &str) -> Act {
        let text = |s: String| Act::Say(s);
        match rest {
            "I" => text(self.prefixed(&self.ati)),
            "I2" => text(self.prefixed(&self.board.to_string())),
            "I3" => text(self.prefixed(&self.freq.to_string())),
            "I5" => Act::Lines(self.plain_lines()),
            "I5?" if self.ranged => Act::Lines(
                self.params
                    .iter()
                    .map(|p| format!("{}\r\n", self.query_line(p)))
                    .collect(),
            ),
            "I5?" => Act::Lines(self.plain_lines()),
            "I7" => text(self.prefixed(&self.rssi)),
            "&E?" => text(
                self.key
                    .as_ref()
                    .map_or_else(|| "ERROR".to_owned(), |k| self.prefixed(k)),
            ),
            "&W" => {
                self.eeprom = self.params.iter().map(|p| p.value).collect();
                text("OK".to_owned())
            }
            "&F" => {
                self.params.iter_mut().for_each(|p| p.value = p.default);
                text("OK".to_owned())
            }
            "Z" => {
                for (p, saved) in self.params.iter_mut().zip(&self.eeprom) {
                    p.value = *saved;
                }
                self.reboots += 1;
                Act::Reboot
            }
            "O" => Act::Transparent,
            "&T" => Act::Nothing,
            "&R" => {
                self.ppm_recorded = true;
                text("OK".to_owned())
            }
            "&UPDATE" => Act::Update,
            "PO=1" | "PI=1" => text("OK".to_owned()),
            _ => self.act_more(rest),
        }
    }

    fn act_more(&mut self, rest: &str) -> Act {
        if let Some(index) = rest.strip_prefix("I10:") {
            if !self.ati10 {
                return Act::Say("ERROR".to_owned());
            }
            let line = index
                .parse::<usize>()
                .ok()
                .and_then(|i| self.params.get(i))
                .map_or_else(|| "EOF".to_owned(), |p| self.query_line(p));
            return Act::Say(line);
        }
        if let Some(key) = rest.strip_prefix("&E=") {
            self.key = Some(key.to_owned());
            return Act::Say("OK".to_owned());
        }
        if let Some(query) = rest.strip_prefix("+C")
            && query.ends_with('?')
        {
            return Act::Say(
                self.country
                    .map_or_else(|| "ERROR".to_owned(), |c| c.to_string()),
            );
        }
        if let Some(set) = rest.strip_prefix('S')
            && let Some((number, value)) = set.split_once('=')
            && let (Ok(number), Ok(value)) = (number.parse::<u8>(), value.parse::<i32>())
            && let Some(p) = self.params.iter_mut().find(|p| p.number == number)
        {
            p.value = value;
            return Act::Say("OK".to_owned());
        }
        Act::Say("ERROR".to_owned())
    }
}

/// What a command does.
enum Act {
    Say(String),
    Lines(String),
    Reboot,
    Transparent,
    Update,
    Nothing,
}

/// The local radio's mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Mode {
    Transparent,
    Command,
    Bootloader,
    BootloaderX,
    XModem,
}

/// The port and the radios.
#[derive(Debug)]
pub(crate) struct Radio {
    now: Duration,
    baud: u32,
    open: bool,
    /// Bytes on their way back, each with when it arrives.
    out: VecDeque<(Duration, u8)>,
    /// Everything written to the port.
    pub(crate) written: Vec<u8>,
    /// The AT and RT lines acted on, and the bootloader commands, in order.
    commands: Vec<String>,
    mode: Mode,
    line: String,
    last_rx: Duration,
    /// The radio's baud rate in transparent and AT command modes.
    pub(crate) serial_baud: u32,
    pub(crate) local: Unit,
    pub(crate) remote: Option<Unit>,
    /// The bootloader's input not yet acted on.
    boot_in: Vec<u8>,
    /// The flash, as the bootloader wrote it.
    pub(crate) flash: BTreeMap<u32, u8>,
    address: u32,
    /// What the XModem receive took, block by block.
    pub(crate) received: Vec<u8>,
    xmodem_block: u8,
    /// Blocks to refuse with a NAK before taking them, for a test of the retries.
    pub(crate) nak_blocks: u32,
    /// Whether the bootloader goes quiet, for a test of a lost one.
    pub(crate) bootloader_silent: bool,
}

impl Radio {
    /// An RFD900+ with a remote one, in transparent mode at 57600.
    pub(crate) fn rfd900p() -> Self {
        Self::with(Unit::rfd900p(), Some(Unit::rfd900p()))
    }

    /// The radios given.
    pub(crate) fn with(local: Unit, remote: Option<Unit>) -> Self {
        Self {
            now: Duration::from_secs(100),
            baud: 57_600,
            open: true,
            out: VecDeque::new(),
            written: Vec::new(),
            commands: Vec::new(),
            mode: Mode::Transparent,
            line: String::new(),
            last_rx: Duration::ZERO,
            serial_baud: 57_600,
            local,
            remote,
            boot_in: Vec::new(),
            flash: BTreeMap::new(),
            address: 0,
            received: Vec::new(),
            xmodem_block: 1,
            nak_blocks: 0,
            bootloader_silent: false,
        }
    }

    /// Straight into AT command mode, for a test that starts there.
    pub(crate) fn enter_command_mode(&mut self) {
        self.mode = Mode::Command;
    }

    /// Straight into a bootloader, for a test that starts there.
    pub(crate) fn enter(&mut self, mode: Mode) {
        self.mode = mode;
    }

    pub(crate) const fn mode(&self) -> Mode {
        self.mode
    }

    pub(crate) fn commands(&self) -> &Vec<String> {
        &self.commands
    }

    fn say_at(&mut self, at: Duration, text: &str) {
        let at = self.out.back().map_or(at, |(last, _)| (*last).max(at));
        for byte in text.bytes() {
            self.out.push_back((at, byte));
        }
    }

    fn say(&mut self, text: &str) {
        self.say_at(self.now + LATENCY, text);
    }

    fn say_bytes(&mut self, bytes: &[u8]) {
        let at = self.now + LATENCY;
        let at = self.out.back().map_or(at, |(last, _)| (*last).max(at));
        for byte in bytes {
            self.out.push_back((at, *byte));
        }
    }

    fn command_byte(&mut self, byte: u8) {
        match byte {
            b'\r' => {
                self.say("\r\n");
                let line = std::mem::take(&mut self.line).trim().to_uppercase();
                self.execute(&line);
            }
            b'\n' => {}
            byte if byte.is_ascii_graphic() || byte == b' ' => {
                self.line.push(char::from(byte));
                self.say(&char::from(byte).to_string());
            }
            _ => {}
        }
    }

    fn execute(&mut self, line: &str) {
        if line.is_empty() {
            return;
        }
        self.commands.push(line.to_owned());
        if let Some(rest) = line.strip_prefix("AT") {
            match self.local.act(rest) {
                Act::Say(text) => self.say(&format!("{text}\r\n")),
                Act::Lines(text) => self.say(&text),
                Act::Reboot | Act::Transparent => self.mode = Mode::Transparent,
                Act::Update if self.local.x_bootloader => self.mode = Mode::BootloaderX,
                Act::Update => self.mode = Mode::Bootloader,
                Act::Nothing => {}
            }
        } else if let Some(rest) = line.strip_prefix("RT") {
            let Some(remote) = self.remote.as_mut() else {
                return;
            };
            let at = self.now + AIR;
            match remote.act(rest) {
                Act::Say(text) => self.say_at(at, &format!("{text}\r\n")),
                Act::Lines(text) => self.say_at(at, &text),
                _ => {}
            }
        }
    }

    fn boot(&mut self) {
        use crate::uploader::code::{
            CHIP_ERASE, EOC, GET_DEVICE, GET_SYNC, INSYNC, LOAD_ADDRESS, OK, PROG_MULTI,
            READ_MULTI, REBOOT,
        };
        loop {
            let Some(&command) = self.boot_in.first() else {
                return;
            };
            let banking = self.local.board & 0x80 != 0;
            let need = match command {
                GET_SYNC | GET_DEVICE | CHIP_ERASE => 2,
                LOAD_ADDRESS => {
                    if banking {
                        5
                    } else {
                        4
                    }
                }
                PROG_MULTI => match self.boot_in.get(1) {
                    Some(&len) => 3 + usize::from(len),
                    None => return,
                },
                READ_MULTI => 3,
                REBOOT => 1,
                _ => {
                    self.boot_in.remove(0);
                    continue;
                }
            };
            if self.boot_in.len() < need {
                return;
            }
            let frame: Vec<u8> = self.boot_in.drain(..need).collect();
            if command != REBOOT && frame.last() != Some(&EOC) {
                continue;
            }
            self.commands.push(format!("boot {command:#x}"));
            if self.bootloader_silent {
                continue;
            }
            match command {
                GET_SYNC => self.say_bytes(&[INSYNC, OK]),
                GET_DEVICE => {
                    let reply = [self.local.board, self.local.freq, INSYNC, OK];
                    self.say_bytes(&reply);
                }
                CHIP_ERASE => {
                    self.flash.clear();
                    self.say_bytes(&[INSYNC, OK]);
                }
                LOAD_ADDRESS => {
                    let mut address = u32::from(frame[1]) | (u32::from(frame[2]) << 8);
                    if banking {
                        address |= u32::from(frame[3]) << 16;
                    }
                    self.address = address;
                    self.say_bytes(&[INSYNC, OK]);
                }
                PROG_MULTI => {
                    for byte in &frame[2..frame.len() - 1] {
                        self.flash.insert(self.address, *byte);
                        self.address += 1;
                    }
                    self.say_bytes(&[INSYNC, OK]);
                }
                READ_MULTI => {
                    let mut reply = Vec::new();
                    for _ in 0..frame[1] {
                        reply.push(self.flash.get(&self.address).copied().unwrap_or(0xFF));
                        self.address += 1;
                    }
                    reply.extend([INSYNC, OK]);
                    self.say_bytes(&reply);
                }
                REBOOT => {
                    self.mode = Mode::Transparent;
                    self.local.reboots += 1;
                }
                _ => {}
            }
        }
    }

    fn boot_x_line(&mut self, byte: u8) {
        match byte {
            b'\r' | b'\n' => {
                let line = std::mem::take(&mut self.line);
                let line = line.trim();
                if line.is_empty() {
                    return;
                }
                self.commands.push(format!("x {line}"));
                match line {
                    "CHIPID" => self.say("RFD900xSub:1\r\n"),
                    "RESET" | "BOOTNEW" => self.mode = Mode::Transparent,
                    "UPLOAD" => {
                        self.say("Ready\r\n");
                        self.say("C");
                        self.mode = Mode::XModem;
                        self.xmodem_block = 1;
                        self.boot_in.clear();
                    }
                    _ => {}
                }
            }
            byte => self.line.push(char::from(byte)),
        }
    }

    fn xmodem(&mut self) {
        const SOH: u8 = 0x01;
        const EOT: u8 = 0x04;
        const ACK: u8 = 0x06;
        const NAK: u8 = 0x15;
        loop {
            match self.boot_in.first() {
                None => return,
                Some(&EOT) => {
                    self.boot_in.remove(0);
                    self.commands.push("x EOT".to_owned());
                    self.say_bytes(&[ACK]);
                    self.mode = Mode::BootloaderX;
                    return;
                }
                Some(&SOH) if self.boot_in.len() >= 133 => {
                    let block: Vec<u8> = self.boot_in.drain(..133).collect();
                    let crc = crate::xmodem::crc(&block[3..131]);
                    let good = block[1] == self.xmodem_block
                        && block[2] == 255 - self.xmodem_block
                        && block[131] == (crc >> 8) as u8
                        && block[132] == (crc & 0xFF) as u8;
                    if good && self.nak_blocks == 0 {
                        self.received.extend_from_slice(&block[3..131]);
                        self.xmodem_block = self.xmodem_block.wrapping_add(1);
                        self.say_bytes(&[ACK]);
                    } else {
                        self.nak_blocks = self.nak_blocks.saturating_sub(1);
                        self.say_bytes(&[NAK]);
                    }
                }
                Some(&SOH) => return,
                Some(_) => {
                    self.boot_in.remove(0);
                }
            }
        }
    }
}

impl Port for Radio {
    fn write(&mut self, data: &[u8]) -> io::Result<()> {
        if !self.open {
            return Err(crate::closed());
        }
        self.written.extend_from_slice(data);
        match self.mode {
            Mode::Transparent => {
                if self.baud == self.serial_baud
                    && data == b"+++"
                    && self.now.saturating_sub(self.last_rx) >= GUARD
                {
                    self.mode = Mode::Command;
                    self.line.clear();
                    let at = self.now + GUARD;
                    self.say_at(at, "OK\r\n");
                }
            }
            Mode::Command => {
                if self.baud == self.serial_baud {
                    for &byte in data {
                        self.command_byte(byte);
                    }
                }
            }
            Mode::Bootloader => {
                if self.baud == 115_200 {
                    self.boot_in.extend_from_slice(data);
                    self.boot();
                }
            }
            Mode::BootloaderX => {
                if self.baud == 57_600 {
                    for &byte in data {
                        self.boot_x_line(byte);
                    }
                }
            }
            Mode::XModem => {
                self.boot_in.extend_from_slice(data);
                self.xmodem();
            }
        }
        self.last_rx = self.now;
        Ok(())
    }

    fn read(&mut self, buf: &mut [u8], timeout: Duration) -> io::Result<usize> {
        if !self.open {
            return Err(crate::closed());
        }
        match self.out.front() {
            Some((at, _)) if *at <= self.now + timeout => self.now = self.now.max(*at),
            _ => {
                self.now += timeout;
                return Ok(0);
            }
        }
        let mut count = 0;
        while count < buf.len()
            && let Some((at, byte)) = self.out.front().copied()
            && at <= self.now
        {
            buf[count] = byte;
            count += 1;
            self.out.pop_front();
        }
        Ok(count)
    }

    fn set_baud(&mut self, baud: u32) -> io::Result<()> {
        self.baud = baud;
        Ok(())
    }

    fn baud(&self) -> u32 {
        self.baud
    }

    fn sleep(&mut self, duration: Duration) {
        self.now += duration;
    }

    fn now(&self) -> Duration {
        self.now
    }

    fn is_open(&self) -> bool {
        self.open
    }

    fn close(&mut self) {
        self.open = false;
    }
}
