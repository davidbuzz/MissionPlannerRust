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

//! RTK/GPS Inject: `GCSViews/ConfigurationView/ConfigSerialInjectGPS.cs`, an Optional Hardware
//! entry of Initial Setup's list, shown whether or not a vehicle is connected
//! (`GCSViews/InitialSetup.cs:251`).
//!
//! What it does: a base station's corrections - read from a serial port, a UDP port it listens on,
//! a UDP or TCP host, or an NTRIP caster - go to the vehicle. A thread reads the port, logs every
//! byte to a `.gpsbase` file, finds RTCM 3, Swift (SBP), u-blox (UBX) and NMEA messages in what it
//! reads, and hands each RTCM and SBP message - or, until it has found one, the bytes as read - to
//! the link's `InjectGpsData`, which cuts it into `GPS_RTCM_DATA` fragments, or `GPS_INJECT_DATA`
//! for a vehicle too old for them (`ConfigSerialInjectGPS.cs:615-805, 1136-1151`). A station
//! message, 1005 or 1006, puts the base in `cs.Base` and on the RTCM Base line; a u-blox base's
//! survey in fills the Survey In group; each constellation's messages turn its light green; each
//! observation message sets the signal bars (`:167-252, 807-1113`).
//!
//! What it shows, as its Designer places it in a 754 x 594 page: the port, Connect and the baud
//! rate; "Send NTRIP GGA", "Send NTRIP protocol v1.0" and "Automatically Configure Receiver", with
//! the receiver's kind under it once ticked; Link Status - the rates in and out, the messages seen;
//! RTCM - five lights and the base's position; the map, with the base on it; and in the split
//! container, the automatic configuration's options - for a u-blox the 1.30 box, the survey-in
//! accuracy and time, Restart, Save Current Position, the base positions' grid and the Survey In
//! group; for a Septentrio its fixed position, RTCM amount, interval and constellations; for a
//! Unicore a paragraph - over the signal bars.
//!
//! How it is built: the protocols are modules of their own - [`rtcm3`], [`sbp`], [`nmea`], [`ubx`]
//! and [`septentrio`] (with Unicore) - ported as far as the page uses them; [`worker`] is the
//! page's statics and its thread; then the page object, its handlers, the facts and the drawing.
//! The thread's injection is `mp_link::inject`, and `cs.Base` goes into the vehicle's state
//! through the link's sender.
//!
//! Where it differs from the C#, and why:
//!
//! * the port is opened, and a receiver configured, on the port's thread before its loop, where
//!   the C# waits for them on the UI thread - an NTRIP caster's first line, a u-blox's seven baud
//!   rates; Connect is disabled until the thread has started, as the C# disables it while it
//!   configures a Septentrio or Unicore. The thread closes the port when Connect stops it, where
//!   the C# closes it on the UI thread, which a read in progress would hold up;
//! * the handlers' commands to a receiver - Use, Restart, the Septentrio boxes - run on a thread of
//!   their own beside the read loop, as the C#'s run on the UI thread beside it;
//! * the baud box is enabled for any serial port: the C# enables it for a name containing "com",
//!   which on Linux no serial port has;
//! * Septentrio's wait for an acknowledgement ends at its second whether or not a line arrives;
//! * what the C# lets escape into WinForms' unhandled-exception handler - `float.Parse` under Set
//!   Position, `int.Parse` under Use, a closed port under the constellation boxes and Set
//!   Position, an I/O error configuring a receiver - sends nothing and shows nothing here;
//! * the messages seen are listed by name, where `ConcurrentDictionary` lists them in its hash
//!   order; `GPS_INJECT_DATA` goes to each vehicle in id order, where `MAVlist` has them in the
//!   order heard; all the vehicles "use the same MAVLink version", as this link sends every
//!   message as MAVLink 2;
//! * a bar's label and value are written across it, where the C# writes them up it;
//! * u-blox MON-VER and MON-HW, which the C# prints to the console, are read and dropped.
//!
//! What is not ported: DroneCAN - the C# feeds every byte to `DroneCAN.ReadSLCAN` and injects the
//! `uavcan.equipment.gnss.RTCMStream` it finds, and `ExtLibs/DroneCAN` is not in this application
//! ([`CAN_DIMMED`]); the lines `DoConnect` writes for a CAN adapter on a serial port are written.
//! `CommsSerialPipe`, which `DoConnect` falls back to when .NET's `SerialPort` refuses a port
//! name (`ConfigSerialInjectGPS.cs:414-427`), has no counterpart here and needs none: it is a
//! serial port opened through Win32's `CreateFile("\\.\<name>")` rather than a pipe, and
//! [`mp_transport::SerialTransport`] already opens every name that call opens (the `serialport`
//! crate uses the same call on Windows), so the first attempt cannot fail the way the fallback
//! answers. Of `rtcm3.cs`, the pseudoranges, phases and times of the observation
//! messages and the ephemeris of 1019, which nothing on the page reads.
//!
//! The colours are this application's; the lights keep the C#'s green and red.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::cell::RefCell;
use std::collections::VecDeque;
use std::rc::Rc;
use std::sync::Arc;
use std::time::{Duration, Instant};

use gpui::{AnyElement, Context, FocusHandle, SharedString, Window, div, prelude::*, px, rgb};
use mp_units::LatLon;
use mp_vehicle::LatLngAlt;

use crate::MissionPlanner;
use crate::config::optional::{at, button, group, label, message_box, text_box};
use crate::config::servo_output::{
    Check, CheckState, Combo, Message, check_box, combo_box, dropdown,
};
use crate::mapview::MapViewport;
use crate::settings::Persisted;
use crate::setup::Key;
use crate::telemetry::TelemetryView;
use crate::textfield::{KeyOutcome, TextField};
use crate::ui::{action, theme};

// ---------------------------------------------------------------------------------------------
// The protocols the page reads and writes: `ExtLibs/Utilities/rtcm3.cs`, `sbp.cs`, `nmea.cs`,
// `ubx_m8p.cs`, `Septentrio.cs` and `Unicore.cs`, as far as the page uses them.
// ---------------------------------------------------------------------------------------------

/// A decoder's `IndexOutOfRangeException`: the C#'s parsers index fixed arrays, and a message
/// that runs past one throws out of `Read` into the read loop's `catch`, which drops the rest of
/// the chunk being read.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Overflow;

/// `buffer[at] = data`, or the C#'s `IndexOutOfRangeException`.
fn put(buffer: &mut [u8], at: usize, data: u8) -> Result<(), Overflow> {
    *buffer.get_mut(at).ok_or(Overflow)? = data;
    Ok(())
}

/// `buffer[at]`; past the end - which the page's messages never reach - 0.
fn byte(buffer: &[u8], at: usize) -> u8 {
    buffer.get(at).copied().unwrap_or(0)
}

pub mod rtcm3 {
    //! `ExtLibs/Utilities/rtcm3.cs`: the framing and CRC-24Q exactly, the station position of
    //! 1005 and 1006, `ecef2pos`, and of the observation messages what the page's signal bars
    //! show - each satellite's system, number and L1 signal strength. The pseudoranges, phases and
    //! times the C# also works out, and the ephemeris of 1019 with the week it guesses from it,
    //! are read by nothing on the page and are not ported.

    use super::{Overflow, byte, put};

    /// `RTCM3PREAMB`.
    /// `// C#: ExtLibs/Utilities/rtcm3.cs:15`
    pub const PREAMBLE: u8 = 0xD3;

    /// `packet`'s size.
    /// `// C#: ExtLibs/Utilities/rtcm3.cs:74`
    pub const PACKET: usize = 1024 * 4;

    /// `R2D`: radians to degrees.
    /// `// C#: ExtLibs/Utilities/rtcm3.cs:52`
    pub const R2D: f64 = 180.0 / std::f64::consts::PI;

    /// `RE_WGS84` and `FE_WGS84`.
    /// `// C#: ExtLibs/Utilities/rtcm3.cs:48-49`
    const RE_WGS84: f64 = 6_378_137.0;
    const FE_WGS84: f64 = 1.0 / 298.257_223_563;

    /// `crc24qtab`: the table of the polynomial `0x1864CFB`, not reversed. The C# lists the 256
    /// values; this works them out, and a test holds them to the C#'s list.
    /// `// C#: ExtLibs/Utilities/rtcm3.cs:531-565`
    pub const CRC24Q_TABLE: [u32; 256] = {
        let mut table = [0u32; 256];
        let mut i = 0;
        while i < 256 {
            #[allow(clippy::cast_possible_truncation)] // i < 256
            let mut crc = (i as u32) << 16;
            let mut bit = 0;
            while bit < 8 {
                crc <<= 1;
                if crc & 0x100_0000 != 0 {
                    crc ^= 0x186_4CFB;
                }
                bit += 1;
            }
            #[allow(clippy::indexing_slicing)] // i < 256
            {
                table[i] = crc & 0xFF_FFFF;
            }
            i += 1;
        }
        table
    };

    /// `crc24.crc24q(buf, len, crc)`: CRC-24Q.
    /// `// C#: ExtLibs/Utilities/rtcm3.cs:583-588`
    #[must_use]
    pub fn crc24q(buf: &[u8], mut crc: u32) -> u32 {
        for &b in buf {
            let index = ((crc >> 16) ^ u32::from(b)) & 0xFF;
            crc = ((crc << 8) & 0xFF_FFFF) ^ CRC24Q_TABLE.get(index as usize).copied().unwrap_or(0);
        }
        crc
    }

    /// `getbitu`: `len` bits from bit `pos`, most significant first.
    /// `// C#: ExtLibs/Utilities/rtcm3.cs:319-326`
    #[must_use]
    pub fn getbitu(buff: &[u8], pos: u32, len: u32) -> u32 {
        let mut bits: u32 = 0;
        for i in pos..pos + len {
            let at = (i / 8) as usize;
            bits = (bits << 1).wrapping_add(u32::from((byte(buff, at) >> (7 - i % 8)) & 1));
        }
        bits
    }

    /// `getbits`: `getbitu` with its sign extended.
    /// `// C#: ExtLibs/Utilities/rtcm3.cs:369-375`
    #[must_use]
    #[allow(clippy::cast_possible_wrap)] // two's complement, as the C#'s `(int)` of a uint
    pub fn getbits(buff: &[u8], pos: u32, len: u32) -> i32 {
        let bits = getbitu(buff, pos, len);
        if len == 0 || 32 <= len || bits & (1u32 << (len - 1)) == 0 {
            return bits as i32;
        }
        (bits | (!0u32 << len)) as i32
    }

    /// `getbits_38`: a 38-bit signed number, as a double.
    /// `// C#: ExtLibs/Utilities/rtcm3.cs:364-367`
    #[must_use]
    pub fn getbits_38(buff: &[u8], pos: u32) -> f64 {
        f64::from(getbits(buff, pos, 32)) * 64.0 + f64::from(getbitu(buff, pos + 32, 6))
    }

    /// `ecef2pos`: an earth-centred position to latitude and longitude in radians and height in
    /// metres on WGS84, iterated to 0.1 mm.
    /// `// C#: ExtLibs/Utilities/rtcm3.cs:399-413`
    #[must_use]
    pub fn ecef2pos(r: [f64; 3]) -> [f64; 3] {
        let e2 = FE_WGS84 * (2.0 - FE_WGS84);
        let r2 = r[0] * r[0] + r[1] * r[1];
        let mut z = r[2];
        let mut zk = 0.0;
        let mut v = RE_WGS84;
        while (z - zk).abs() >= 1E-4 {
            zk = z;
            let sinp = z / (r2 + z * z).sqrt();
            v = RE_WGS84 / (1.0 - e2 * sinp * sinp).sqrt();
            z = r[2] + v * e2 * sinp;
        }
        let pi = std::f64::consts::PI;
        [
            if r2 > 1E-12 {
                (z / r2.sqrt()).atan()
            } else if r[2] > 0.0 {
                pi / 2.0
            } else {
                -pi / 2.0
            },
            if r2 > 1E-12 { r[1].atan2(r[0]) } else { 0.0 },
            (r2 + z * z).sqrt() - v,
        ]
    }

    /// `pos2ecef`: the other way. Only the tests use it, to make positions to check `ecef2pos`
    /// against.
    /// `// C#: ExtLibs/Utilities/rtcm3.cs:415-426`
    #[cfg(test)]
    #[must_use]
    pub fn pos2ecef(pos: [f64; 3]) -> [f64; 3] {
        let (sinp, cosp, sinl, cosl) = (pos[0].sin(), pos[0].cos(), pos[1].sin(), pos[1].cos());
        let e2 = FE_WGS84 * (2.0 - FE_WGS84);
        let v = RE_WGS84 / (1.0 - e2 * sinp * sinp).sqrt();
        [
            (v + pos[2]) * cosp * cosl,
            (v + pos[2]) * cosp * sinl,
            (v * (1.0 - e2) + pos[2]) * sinp,
        ]
    }

    /// One satellite of an observation message, as the page's signal bars use it: `ob.sys`,
    /// `ob.prn` and `ob.snr`.
    /// `// C#: ExtLibs/Utilities/rtcm3.cs:1013-1043`
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct Ob {
        /// `G`, `R`, `E`, `B` or `Q`.
        pub sys: char,
        /// The satellite's number.
        pub prn: u8,
        /// The L1 signal's carrier to noise, dBHz, cut to a byte.
        pub snr: u8,
    }

    /// `rtcm3`: the parser, fed a byte at a time.
    #[derive(Debug, Clone)]
    pub struct Rtcm3 {
        /// `step`.
        step: u8,
        /// `packet`.
        packet: Vec<u8>,
        /// `payloadlen`: the frame's length field, and once a frame is whole, that plus the three
        /// bytes before it.
        payloadlen: u32,
        /// `msglencount`.
        msglencount: u32,
        /// What `ObsMessage` was raised with for the last message, until taken.
        obs: Option<Vec<Ob>>,
    }

    impl Default for Rtcm3 {
        fn default() -> Self {
            Self {
                step: 0,
                packet: vec![0; PACKET],
                payloadlen: 0,
                msglencount: 0,
                obs: None,
            }
        }
    }

    impl Rtcm3 {
        /// `resetParser`.
        /// `// C#: ExtLibs/Utilities/rtcm3.cs:76-80`
        pub fn reset_parser(&mut self) {
            self.step = 0;
        }

        /// `length`: the whole frame once one has been read, three bytes of header, the
        /// message and three of CRC.
        /// `// C#: ExtLibs/Utilities/rtcm3.cs:69-72`
        #[must_use]
        pub fn length(&self) -> usize {
            (self.payloadlen + 2 + 1) as usize
        }

        /// `packet`, as far as `length`: the frame last read.
        #[must_use]
        pub fn packet(&self) -> &[u8] {
            self.packet.get(..self.length()).unwrap_or(&self.packet)
        }

        /// The observations the last message raised `ObsMessage` with, taken.
        pub fn take_obs(&mut self) -> Option<Vec<Ob>> {
            self.obs.take()
        }

        /// `Read(data)`: the message number when `data` ends a frame whose CRC-24Q is right,
        /// else -1; an observation message that overruns the C#'s arrays is its exception.
        /// `// C#: ExtLibs/Utilities/rtcm3.cs:82-317`
        pub fn read(&mut self, data: u8) -> Result<i32, Overflow> {
            match self.step {
                1 => {
                    put(&mut self.packet, 1, data)?;
                    self.step += 1;
                }
                2 => {
                    put(&mut self.packet, 2, data)?;
                    self.step += 1;
                    // `rtcmpreamble.Read`: 8 bits of preamble, 6 reserved, 10 of length.
                    self.payloadlen = getbitu(&self.packet, 14, 10);
                    self.msglencount = 0;
                    // "reset on oversize packet" - which ten bits cannot be.
                    if self.payloadlen as usize > self.packet.len() {
                        self.step = 0;
                    }
                }
                3 => {
                    if self.msglencount < self.payloadlen {
                        put(&mut self.packet, (self.msglencount + 3) as usize, data)?;
                        self.msglencount += 1;
                    } else {
                        // `goto case 4`: this byte is the CRC's first.
                        self.step += 1;
                        put(&mut self.packet, (self.payloadlen + 3) as usize, data)?;
                        self.step += 1;
                    }
                }
                4 => {
                    put(&mut self.packet, (self.payloadlen + 3) as usize, data)?;
                    self.step += 1;
                }
                5 => {
                    put(&mut self.packet, (self.payloadlen + 3 + 1) as usize, data)?;
                    self.step += 1;
                }
                6 => {
                    put(&mut self.packet, (self.payloadlen + 3 + 2) as usize, data)?;
                    self.payloadlen += 3;
                    let framed = self
                        .packet
                        .get(..self.payloadlen as usize)
                        .unwrap_or_default();
                    let crc = crc24q(framed, 0);
                    let crcpacket = getbitu(&self.packet, self.payloadlen * 8, 24);
                    self.step = 0;
                    if crc == crcpacket {
                        // `rtcmheader.Read`: the message number.
                        let messageno = getbitu(&self.packet, 24, 12);
                        self.obs = observations(&self.packet, messageno)?;
                        #[allow(clippy::cast_possible_wrap)] // twelve bits
                        return Ok(messageno as i32);
                    }
                }
                // `default: case 0:`
                _ => {
                    if data == PREAMBLE {
                        self.step = 1;
                        put(&mut self.packet, 0, data)?;
                    }
                }
            }
            Ok(-1)
        }
    }

    /// The observation messages `Read` decodes and raises `ObsMessage` for, by number.
    /// `// C#: ExtLibs/Utilities/rtcm3.cs:145-261`
    fn observations(packet: &[u8], messageno: u32) -> Result<Option<Vec<Ob>>, Overflow> {
        Ok(Some(match messageno {
            1002 => legacy(packet, Legacy::Gps1002),
            1004 => legacy(packet, Legacy::Gps1004),
            1012 => legacy(packet, Legacy::Glonass1012),
            1074 => msm(packet, false, 'G')?,
            1077 => msm(packet, true, 'G')?,
            1084 => msm(packet, false, 'R')?,
            1087 => msm(packet, true, 'R')?,
            1094 => msm(packet, false, 'E')?,
            1097 => msm(packet, true, 'E')?,
            1114 => msm(packet, false, 'Q')?,
            1117 => msm(packet, true, 'Q')?,
            1124 => msm(packet, false, 'B')?,
            1127 => msm(packet, true, 'B')?,
            _ => return Ok(None),
        }))
    }

    /// The three legacy observation messages.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    enum Legacy {
        /// `type1002`: GPS L1.
        Gps1002,
        /// `type1004`: GPS L1 and L2.
        Gps1004,
        /// `type1012`: GLONASS L1 and L2.
        Glonass1012,
    }

    /// `type1002.Read`, `type1004.Read` and `type1012.Read`: each satellite's number and L1
    /// signal strength, a satellite whose L1 phase is the invalid `0xFFF80000` left out, sorted
    /// by number.
    /// `// C#: ExtLibs/Utilities/rtcm3.cs:596-665, 758-843, 898-989`
    fn legacy(packet: &[u8], kind: Legacy) -> Vec<Ob> {
        let glonass = kind == Legacy::Glonass1012;
        // The header: 12 bits of message number, 12 of station, the epoch (27 bits for GLONASS,
        // 30 for GPS), the synchronous flag, then the number of satellites.
        let epoch = if glonass { 27 } else { 30 };
        let nsat = getbitu(packet, 24 + 12 + 12 + epoch + 1, 5);
        // Then the smoothing indicator and interval: 64 bits of header for GPS, 61 for GLONASS.
        let mut i: u32 = if glonass { 24 + 61 } else { 24 + 64 };
        let mut obs = Vec::new();
        for _ in 0..nsat {
            #[allow(clippy::cast_possible_truncation)] // six bits
            let prn = getbitu(packet, i, 6) as u8;
            i += 6 + 1; // prn, code1
            if glonass {
                i += 5; // fcn
            }
            i += if glonass { 25 } else { 24 }; // pr1
            let ppr1 = getbits(packet, i, 20);
            i += 20 + 7; // ppr1, lock1
            i += if glonass { 7 } else { 8 }; // amb
            let cnr1 = getbitu(packet, i, 8);
            i += 8;
            if kind != Legacy::Gps1002 {
                i += 2 + 14 + 20 + 7 + 8; // code2, pr21, ppr2, lock2, cnr2
            }
            // `(uint) ob.raw.ppr1 != 0xFFF80000`: -524288, the invalid phase.
            if ppr1 != -524_288 {
                obs.push(Ob {
                    sys: if glonass { 'R' } else { 'G' },
                    prn,
                    // `(byte) (ob.raw.cnr1 * 0.25)`
                    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
                    snr: (f64::from(cnr1) * 0.25) as u8,
                });
            }
        }
        obs.sort_by_key(|ob| ob.prn);
        obs
    }

    /// `type1074.Read` (MSM4) and `type1077.Read` (MSM7), and the classes that relabel them
    /// for GLONASS, Galileo, QZSS and BeiDou: each satellite in the mask, with the strength of
    /// its last L1 signal, sorted by number. A mask of more than 64 cells overruns the C#'s
    /// `cellmask`.
    /// `// C#: ExtLibs/Utilities/rtcm3.cs:1045-1562`
    fn msm(packet: &[u8], seven: bool, sys: char) -> Result<Vec<Ob>, Overflow> {
        // Message number, station, epoch, multiple message, IODS, reserved, clock steering,
        // external clock, smoothing, smoothing interval.
        let mut i: u32 = 24 + 12 + 12 + 30 + 1 + 3 + 7 + 2 + 2 + 1 + 3;
        let mut sats = Vec::new();
        for j in 1..=64u8 {
            if getbitu(packet, i, 1) > 0 {
                sats.push(j);
            }
            i += 1;
        }
        let mut sigs = Vec::new();
        for j in 1..=32u32 {
            if getbitu(packet, i, 1) > 0 {
                sigs.push(j);
            }
            i += 1;
        }
        let mut cellmask = [0u8; 64];
        let mut ncell: u32 = 0;
        for j in 0..sats.len() * sigs.len() {
            #[allow(clippy::cast_possible_truncation)] // one bit
            let bit = getbitu(packet, i, 1) as u8;
            *cellmask.get_mut(j).ok_or(Overflow)? = bit;
            i += 1;
            if bit > 0 {
                ncell += 1;
            }
        }
        #[allow(clippy::cast_possible_truncation)] // at most 64
        let nsat = sats.len() as u32;
        // The satellite data: whole milliseconds, MSM7's extended information, the fraction,
        // MSM7's rough phase-range rate.
        i += nsat * if seven { 8 + 4 + 10 + 14 } else { 8 + 10 };
        // The signal data before the strength: pseudorange, phase range, lock time, half cycle.
        i += ncell
            * if seven {
                20 + 24 + 10 + 1
            } else {
                15 + 22 + 4 + 1
            };
        let mut cnr = [0.0f64; 64];
        for j in 0..ncell as usize {
            let value = if seven {
                f64::from(getbitu(packet, i, 10)) * 0.0625
            } else {
                f64::from(getbitu(packet, i, 6))
            };
            *cnr.get_mut(j).ok_or(Overflow)? = value;
            i += if seven { 10 } else { 6 };
        }
        let mut obs = Vec::with_capacity(sats.len());
        let mut sig = 0usize;
        let mut used = 0usize;
        for &prn in &sats {
            let mut ob = Ob { sys, prn, snr: 0 };
            for &signal in &sigs {
                if cellmask.get(sig) == Some(&1) {
                    // L1: signals 1 to 13 - but not 10, in MSM4.
                    if (1..=13).contains(&signal) && (seven || signal != 10) {
                        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
                        let snr = cnr.get(used).copied().unwrap_or(0.0) as u8;
                        ob.snr = snr;
                    }
                    used += 1;
                }
                sig += 1;
            }
            obs.push(ob);
        }
        obs.sort_by_key(|ob| ob.prn);
        Ok(obs)
    }

    /// `type1005.Read` and `type1006.Read`'s `ecefposition`: the antenna reference point,
    /// metres, from the frame of a 1005 or 1006.
    /// `// C#: ExtLibs/Utilities/rtcm3.cs:1579-1612, 1665-1700`
    #[must_use]
    pub fn station_ecef(packet: &[u8]) -> [f64; 3] {
        // Message number, station, ITRF year, the four indicators.
        let mut i = 24 + 12 + 12 + 6 + 4;
        let rr0 = getbits_38(packet, i);
        // The oscillator indicator and a reserved bit.
        i += 38 + 2;
        let rr1 = getbits_38(packet, i);
        // The quarter cycle indicator.
        i += 38 + 2;
        let rr2 = getbits_38(packet, i);
        [rr0 * 0.0001, rr1 * 0.0001, rr2 * 0.0001]
    }
}

pub mod sbp {
    //! `ExtLibs/Utilities/sbp.cs`: Swift Navigation's binary protocol, found in the stream so
    //! that it is injected message by message.

    /// `Crc16Ccitt`'s table: the polynomial 4129 (0x1021), from zero.
    /// `// C#: ExtLibs/Utilities/sbp.cs:130-185`
    const TABLE: [u16; 256] = {
        let mut table = [0u16; 256];
        let mut i = 0;
        while i < 256 {
            let mut temp: u16 = 0;
            #[allow(clippy::cast_possible_truncation)] // i < 256
            let mut a: u16 = (i as u16) << 8;
            let mut j = 0;
            while j < 8 {
                if (temp ^ a) & 0x8000 != 0 {
                    temp = (temp << 1) ^ 4129;
                } else {
                    temp <<= 1;
                }
                a <<= 1;
                j += 1;
            }
            #[allow(clippy::indexing_slicing)] // i < 256
            {
                table[i] = temp;
            }
            i += 1;
        }
        table
    };

    /// `Crc16Ccitt.Accumulate`.
    /// `// C#: ExtLibs/Utilities/sbp.cs:136-141`
    fn accumulate(data: u8, crc: u16) -> u16 {
        let index = ((crc >> 8) ^ u16::from(data)) & 0xFF;
        (crc << 8) ^ TABLE.get(index as usize).copied().unwrap_or(0)
    }

    /// `sbp`: the parser.
    #[derive(Debug, Clone, Default)]
    pub struct Sbp {
        /// `state`.
        state: u8,
        /// `msg.msg_type`.
        msg_type: u16,
        /// `msg.length`: the payload's.
        length: u8,
        /// `msg.buffer`.
        buffer: Vec<u8>,
        /// `lengthcount`.
        lengthcount: usize,
        /// `msg.crc`.
        crc: u16,
        /// `crcpacket`.
        crcpacket: u16,
    }

    impl Sbp {
        /// `resetParser`.
        pub fn reset_parser(&mut self) {
            self.state = 0;
        }

        /// `length`: the payload and its eight bytes of framing.
        #[must_use]
        pub fn length(&self) -> usize {
            usize::from(self.length) + 8
        }

        /// `packet`.
        #[must_use]
        pub fn packet(&self) -> &[u8] {
            &self.buffer
        }

        fn set(&mut self, at: usize, data: u8) {
            if let Some(slot) = self.buffer.get_mut(at) {
                *slot = data;
            }
        }

        /// `read(data)`: the message type when `data` ends a message whose CRC is right, else
        /// -1.
        /// `// C#: ExtLibs/Utilities/sbp.cs:45-126`
        pub fn read(&mut self, data: u8) -> i32 {
            match self.state {
                1 => {
                    self.msg_type = u16::from(data);
                    self.set(1, data);
                    self.crcpacket = accumulate(data, self.crcpacket);
                    self.state += 1;
                }
                2 => {
                    self.msg_type = self.msg_type.wrapping_add(u16::from(data) << 8);
                    self.set(2, data);
                    self.crcpacket = accumulate(data, self.crcpacket);
                    self.state += 1;
                }
                3 | 4 => {
                    // `msg.sender`, which nothing reads.
                    self.set(usize::from(self.state), data);
                    self.crcpacket = accumulate(data, self.crcpacket);
                    self.state += 1;
                }
                5 => {
                    self.length = data;
                    self.set(5, data);
                    self.crcpacket = accumulate(data, self.crcpacket);
                    // `Array.Resize(ref msg.buffer, 8 + data)`
                    self.buffer.resize(8 + usize::from(data), 0);
                    self.lengthcount = 0;
                    self.state += 1;
                }
                6 if self.lengthcount != usize::from(self.length) => {
                    self.set(6 + self.lengthcount, data);
                    self.crcpacket = accumulate(data, self.crcpacket);
                    self.lengthcount += 1;
                }
                // `goto case 7` from a full payload.
                6 | 7 => {
                    self.state = 8;
                    self.crc = u16::from(data);
                    self.set(6 + self.lengthcount, data);
                }
                8 => {
                    self.crc = self.crc.wrapping_add(u16::from(data) << 8);
                    self.set(7 + self.lengthcount, data);
                    self.state = 0;
                    if self.crc == self.crcpacket {
                        return i32::from(self.msg_type);
                    }
                }
                // `default: case 0:` - a new `piksimsg`, its buffer 4096 bytes.
                _ => {
                    if data == 0x55 {
                        self.state = 1;
                        self.msg_type = 0;
                        self.length = 0;
                        self.buffer = vec![0; 4096];
                        self.set(0, data);
                        self.crcpacket = 0;
                    }
                }
            }
            -1
        }
    }
}

pub mod nmea {
    //! `ExtLibs/Utilities/nmea.cs`: NMEA sentences, found in the stream only to be counted.

    /// `buffer`'s size.
    const BUFFER: usize = 1024;

    /// `nmea`: the parser.
    #[derive(Debug, Clone)]
    pub struct Nmea {
        /// `step`.
        step: u8,
        /// `buffer`.
        buffer: [u8; BUFFER],
        /// `msglencount`.
        msglencount: usize,
    }

    impl Default for Nmea {
        fn default() -> Self {
            Self {
                step: 0,
                buffer: [0; BUFFER],
                msglencount: 0,
            }
        }
    }

    /// `GetChecksum`: the exclusive-or of the characters after the `$` up to the first `*`, as
    /// two upper-case hex digits.
    /// `// C#: ExtLibs/Utilities/nmea.cs:91-122`
    #[must_use]
    pub fn checksum(sentence: &str) -> String {
        let mut sum: u32 = 0;
        for c in sentence.chars() {
            match c {
                '$' => {}
                '*' => break,
                // `Convert.ToByte(Character)`, of text that is ASCII here.
                _ => sum ^= u32::from(c),
            }
        }
        format!("{sum:02X}")
    }

    impl Nmea {
        /// `resetParser`.
        pub fn reset_parser(&mut self) {
            self.step = 0;
        }

        /// `Read(data)`: 1 when `data` ends a sentence whose checksum is right, else -1.
        ///
        /// As the C# does, a sentence found leaves the parser where it is: the next one is read
        /// onto the end of it, fails its checksum, and puts the parser back to looking for a `$` -
        /// so of a run of sentences every other one is found, unless another parser finding a
        /// message resets this one between them.
        /// `// C#: ExtLibs/Utilities/nmea.cs:43-88`
        pub fn read(&mut self, data: u8) -> i32 {
            match self.step {
                1 => {
                    if data == b'G' {
                        self.buffer[1] = data;
                        self.step += 1;
                    } else {
                        self.step = 0;
                    }
                }
                2 => {
                    if self.msglencount > 1000 {
                        self.step = 0;
                    }
                    if let Some(slot) = self.buffer.get_mut(self.msglencount + 2) {
                        *slot = data;
                    }
                    self.msglencount += 1;
                    if data == b'\n' {
                        // `ASCIIEncoding.ASCII.GetString`: past 127 is `?`.
                        let line: String = self
                            .buffer
                            .get(..self.msglencount + 2)
                            .unwrap_or_default()
                            .iter()
                            .map(|&b| if b.is_ascii() { char::from(b) } else { '?' })
                            .collect();
                        let last = line.trim().split([',', '*']).next_back().unwrap_or("");
                        if last == checksum(&line) {
                            return 1;
                        }
                        self.step = 0;
                    }
                }
                _ => {
                    if data == b'$' {
                        self.step = 1;
                        self.msglencount = 0;
                        self.buffer[0] = data;
                    }
                }
            }
            -1
        }
    }
}

pub mod ubx {
    //! `ExtLibs/Utilities/ubx_m8p.cs`: u-blox's binary protocol - the parser, the messages the
    //! page reads from an M8P or F9P base, and the configuration it writes to one.

    use std::io;
    use std::time::Duration;

    use super::{Overflow, byte, put};

    /// `buffer`'s size.
    const BUFFER: usize = 1024 * 8;

    /// Where a message's payload starts in `packet`: `ByteArrayToStructure<T>(6)`.
    const PAYLOAD: usize = 6;

    /// `Ubx`: the parser.
    #[derive(Debug, Clone)]
    pub struct Ubx {
        /// `step`.
        step: u8,
        /// `buffer`.
        buffer: Vec<u8>,
        /// `payloadlen`.
        payloadlen: usize,
        /// `msglencount`.
        msglencount: usize,
    }

    impl Default for Ubx {
        fn default() -> Self {
            Self {
                step: 0,
                buffer: vec![0; BUFFER],
                payloadlen: 0,
                msglencount: 0,
            }
        }
    }

    /// `ubx_checksum`: the Fletcher sums of `packet[offset..size]`.
    /// `// C#: ExtLibs/Utilities/ubx_m8p.cs:131-148`
    #[must_use]
    pub fn checksum(packet: &[u8], size: usize, offset: usize) -> [u8; 2] {
        let (mut a, mut b) = (0u32, 0u32);
        for i in offset..size {
            a = a.wrapping_add(u32::from(byte(packet, i)));
            b = b.wrapping_add(a);
        }
        #[allow(clippy::cast_possible_truncation)] // masked
        [(a & 0xFF) as u8, (b & 0xFF) as u8]
    }

    /// `generate(cl, subclass, payload)`: a whole message.
    /// `// C#: ExtLibs/Utilities/ubx_m8p.cs:150-168`
    #[must_use]
    pub fn generate(class: u8, subclass: u8, payload: &[u8]) -> Vec<u8> {
        let len = payload.len();
        let mut data = Vec::with_capacity(len + 8);
        #[allow(clippy::cast_possible_truncation)] // masked
        data.extend_from_slice(&[
            0xb5,
            0x62,
            class,
            subclass,
            (len & 0xff) as u8,
            ((len >> 8) & 0xff) as u8,
        ]);
        data.extend_from_slice(payload);
        let sum = checksum(&data, len + 6, 2);
        data.extend_from_slice(&sum);
        data
    }

    impl Ubx {
        /// `resetParser`.
        pub fn reset_parser(&mut self) {
            self.step = 0;
        }

        /// `@class`.
        #[must_use]
        pub fn class(&self) -> u8 {
            byte(&self.buffer, 2)
        }

        /// `subclass`.
        #[must_use]
        pub fn subclass(&self) -> u8 {
            byte(&self.buffer, 3)
        }

        /// `packet`.
        #[must_use]
        pub fn packet(&self) -> &[u8] {
            &self.buffer
        }

        /// `Read(data)`: `class << 8 | subclass` when `data` ends a message whose checksum is
        /// right, else -1; a message longer than the buffer is the C#'s exception.
        ///
        /// A message with no payload leaves the C#'s parser waiting at `step` 6 for bytes it
        /// never counts, until another parser finding a message resets it; so does this.
        /// `// C#: ExtLibs/Utilities/ubx_m8p.cs:51-129`
        pub fn read(&mut self, data: u8) -> Result<i32, Overflow> {
            match self.step {
                1 => {
                    if data == 0x62 {
                        put(&mut self.buffer, 1, data)?;
                        self.step += 1;
                    } else {
                        self.step = 0;
                    }
                }
                2 | 3 => {
                    put(&mut self.buffer, usize::from(self.step), data)?;
                    self.step += 1;
                }
                4 => {
                    put(&mut self.buffer, 4, data)?;
                    self.payloadlen = usize::from(data);
                    self.step += 1;
                }
                5 => {
                    put(&mut self.buffer, 5, data)?;
                    self.step += 1;
                    self.payloadlen += usize::from(data) << 8;
                    self.msglencount = 0;
                    if self.payloadlen > self.buffer.len() {
                        self.step = 0;
                    }
                }
                6 => {
                    if self.msglencount < self.payloadlen {
                        put(&mut self.buffer, self.msglencount + 6, data)?;
                        self.msglencount += 1;
                        if self.msglencount == self.payloadlen {
                            self.step += 1;
                        }
                    }
                }
                7 => {
                    put(&mut self.buffer, self.msglencount + 6, data)?;
                    self.step += 1;
                }
                8 => {
                    put(&mut self.buffer, self.msglencount + 6 + 1, data)?;
                    let sum = checksum(&self.buffer, self.payloadlen + 6, 2);
                    self.step = 0;
                    if sum == [byte(&self.buffer, self.msglencount + 6), data] {
                        return Ok((i32::from(self.class()) << 8) + i32::from(self.subclass()));
                    }
                }
                _ => {
                    if data == 0xb5 {
                        self.step = 1;
                        put(&mut self.buffer, 0, data)?;
                    }
                }
            }
            Ok(-1)
        }
    }

    fn u32_at(packet: &[u8], at: usize) -> u32 {
        u32::from_le_bytes([
            byte(packet, at),
            byte(packet, at + 1),
            byte(packet, at + 2),
            byte(packet, at + 3),
        ])
    }

    fn i32_at(packet: &[u8], at: usize) -> i32 {
        i32::from_le_bytes(u32_at(packet, at).to_le_bytes())
    }

    fn i8_at(packet: &[u8], at: usize) -> i8 {
        i8::from_le_bytes([byte(packet, at)])
    }

    /// `ubx_nav_svin`, as the page reads it.
    /// `// C#: ExtLibs/Utilities/ubx_m8p.cs:239-272`
    #[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
    pub struct NavSvin {
        /// `dur`: seconds surveyed.
        pub dur: u32,
        /// `meanX`, `meanY`, `meanZ`: cm.
        pub mean: [i32; 3],
        /// `meanXHP`, `meanYHP`, `meanZHP`: 0.1 mm.
        pub mean_hp: [i8; 3],
        /// `meanAcc`: 0.1 mm.
        pub mean_acc: u32,
        /// `obs`.
        pub obs: u32,
        /// `valid`.
        pub valid: u8,
        /// `active`.
        pub active: u8,
    }

    impl NavSvin {
        /// `ByteArrayToStructure<ubx_nav_svin>(6)`.
        #[must_use]
        pub fn of(packet: &[u8]) -> Self {
            let p = PAYLOAD;
            Self {
                dur: u32_at(packet, p + 8),
                mean: [
                    i32_at(packet, p + 12),
                    i32_at(packet, p + 16),
                    i32_at(packet, p + 20),
                ],
                mean_hp: [
                    i8_at(packet, p + 24),
                    i8_at(packet, p + 25),
                    i8_at(packet, p + 26),
                ],
                mean_acc: u32_at(packet, p + 28),
                obs: u32_at(packet, p + 32),
                valid: byte(packet, p + 36),
                active: byte(packet, p + 37),
            }
        }

        /// `getECEF`: metres.
        /// `// C#: ExtLibs/Utilities/ubx_m8p.cs:264-271`
        #[must_use]
        pub fn ecef(&self) -> [f64; 3] {
            [0, 1, 2].map(|i| {
                f64::from(self.mean.get(i).copied().unwrap_or(0)) / 100.0
                    + f64::from(self.mean_hp.get(i).copied().unwrap_or(0)) * 0.0001
            })
        }
    }

    /// Of `ubx_nav_pvt`, what the page reads: the fix, the flags and the position.
    /// `// C#: ExtLibs/Utilities/ubx_m8p.cs:208-237`
    #[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
    pub struct NavPvt {
        /// `fix_type`.
        pub fix_type: u8,
        /// `flags`.
        pub flags: u8,
        /// `lon`, 1e-7 degrees.
        pub lon: i32,
        /// `lat`, 1e-7 degrees.
        pub lat: i32,
        /// `height`, mm.
        pub height: i32,
    }

    impl NavPvt {
        /// `ByteArrayToStructure<ubx_nav_pvt>(6)`.
        #[must_use]
        pub fn of(packet: &[u8]) -> Self {
            let p = PAYLOAD;
            Self {
                fix_type: byte(packet, p + 20),
                flags: byte(packet, p + 21),
                lon: i32_at(packet, p + 24),
                lat: i32_at(packet, p + 28),
                height: i32_at(packet, p + 32),
            }
        }
    }

    /// `ubx_cfg_tmode3`: forty bytes.
    /// `// C#: ExtLibs/Utilities/ubx_m8p.cs:317-438`
    #[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
    pub struct Tmode3 {
        /// `flags`: the mode, and 256 for a position in latitude and longitude.
        pub flags: u16,
        /// `ecefXorLat`, `ecefYorLon`, `ecefZorAlt`.
        pub position: [i32; 3],
        /// `ecefXOrLatHP`, `ecefYOrLonHP`, `ecefZOrAltHP`.
        pub position_hp: [i8; 3],
        /// `fixedPosAcc`, 0.1 mm.
        pub fixed_pos_acc: u32,
        /// `svinMinDur`, seconds.
        pub svin_min_dur: u32,
        /// `svinAccLimit`, 0.1 mm.
        pub svin_acc_limit: u32,
    }

    impl Tmode3 {
        /// `ubx_cfg_tmode3(lat, lng, alt, acc = 0.001)`: a fixed position - earth-centred
        /// centimetres when the "latitude" is past 90, else degrees and centimetres, each with
        /// its high-precision part.
        /// `// C#: ExtLibs/Utilities/ubx_m8p.cs:320-350`
        #[must_use]
        #[allow(clippy::cast_possible_truncation)] // `(int)` and `(sbyte)` casts, as the C#'s
        pub fn fixed(lat: f64, lng: f64, alt: f64) -> Self {
            let acc = 0.001;
            let (flags, scale) = if lat.abs() > 90.0 {
                (2, 100.0)
            } else {
                (256 + 2, 1e7)
            };
            let x = (lat * scale) as i32;
            let y = (lng * scale) as i32;
            let z = (alt * 100.0) as i32;
            Self {
                flags,
                position: [x, y, z],
                position_hp: [
                    ((lat * scale - f64::from(x)) * 100.0) as i8,
                    ((lng * scale - f64::from(y)) * 100.0) as i8,
                    ((alt * 100.0 - f64::from(z)) * 100.0) as i8,
                ],
                #[allow(clippy::cast_sign_loss)]
                fixed_pos_acc: (acc * 1000.0) as u32,
                svin_min_dur: 60,
                svin_acc_limit: 2000,
            }
        }

        /// `ubx_cfg_tmode3(DurationS, AccLimit)`: survey in.
        /// `// C#: ExtLibs/Utilities/ubx_m8p.cs:352-368`
        #[must_use]
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        pub fn survey_in(duration: u32, acc_limit: f64) -> Self {
            Self {
                flags: 1,
                svin_min_dur: duration,
                svin_acc_limit: (acc_limit * 10000.0) as u32,
                ..Self::default()
            }
        }

        /// `ByteArrayToStructure<ubx_cfg_tmode3>(6)`.
        #[must_use]
        pub fn of(packet: &[u8]) -> Self {
            let p = PAYLOAD;
            Self {
                flags: u16::from_le_bytes([byte(packet, p + 2), byte(packet, p + 3)]),
                position: [
                    i32_at(packet, p + 4),
                    i32_at(packet, p + 8),
                    i32_at(packet, p + 12),
                ],
                position_hp: [
                    i8_at(packet, p + 16),
                    i8_at(packet, p + 17),
                    i8_at(packet, p + 18),
                ],
                fixed_pos_acc: u32_at(packet, p + 20),
                svin_min_dur: u32_at(packet, p + 24),
                svin_acc_limit: u32_at(packet, p + 28),
            }
        }

        /// `(byte[]) tmode3`: `StructureToByteArray`, packed.
        #[must_use]
        pub fn bytes(&self) -> [u8; 40] {
            let mut out = [0u8; 40];
            let mut at = 2;
            let mut write = |bytes: &[u8]| {
                for &b in bytes {
                    if let Some(slot) = out.get_mut(at) {
                        *slot = b;
                    }
                    at += 1;
                }
            };
            write(&self.flags.to_le_bytes());
            for value in self.position {
                write(&value.to_le_bytes());
            }
            for value in self.position_hp {
                write(&value.to_le_bytes());
            }
            write(&[0]);
            write(&self.fixed_pos_acc.to_le_bytes());
            write(&self.svin_min_dur.to_le_bytes());
            write(&self.svin_acc_limit.to_le_bytes());
            out
        }

        /// `(modeflags) flags`, as `ToString` names it: the enum's name, or the number for a
        /// value it has no name for.
        /// `// C#: ExtLibs/Utilities/ubx_m8p.cs:387-394`
        #[must_use]
        pub fn mode_name(&self) -> String {
            match self.flags {
                0 => "Disabled".to_owned(),
                1 => "SurveyIn".to_owned(),
                2 => "FixedECEF".to_owned(),
                256 => "LLA".to_owned(),
                258 => "FixedLLA".to_owned(),
                other => other.to_string(),
            }
        }

        /// `getPointLatLngAlt`: the fixed position - its earth-centred metres as they stand for
        /// `FixedECEF`, as the C# puts them into a `PointLatLngAlt`, or degrees and metres for
        /// `FixedLLA` - or nothing.
        /// `// C#: ExtLibs/Utilities/ubx_m8p.cs:413-437`
        #[must_use]
        pub fn point(&self) -> Option<[f64; 3]> {
            let [x, y, z] = self.position.map(f64::from);
            let [xh, yh, zh] = self.position_hp.map(f64::from);
            match self.flags {
                2 => Some([
                    x / 100.0 + xh * 0.0001,
                    y / 100.0 + yh * 0.0001,
                    z / 100.0 + zh * 0.0001,
                ]),
                258 => Some([
                    x / 1e7 + xh / 1e9,
                    y / 1e7 + yh / 1e9,
                    z / 100.0 + zh * 0.0001,
                ]),
                _ => None,
            }
        }
    }

    /// What `ubx_m8p`'s configuration writes to: the port, whose baud rate it may change, and
    /// the clock it sleeps on.
    pub trait Receiver {
        /// `port.Write(packet, 0, packet.Length)`.
        fn write(&mut self, bytes: &[u8]) -> io::Result<()>;
        /// `port.BaudRate`.
        fn baud(&self) -> u32;
        /// `port.BaudRate = baud`: a serial port's; other ports have none, and ignore it.
        fn set_baud(&mut self, baud: u32);
        /// `Thread.Sleep`.
        fn sleep(&mut self, time: Duration);
    }

    /// `turnon_off(port, clas, subclass, every_xsamples)`: CFG-MSG, the message's rate on the
    /// current port.
    /// `// C#: ExtLibs/Utilities/ubx_m8p.cs:620-629`
    pub fn turnon_off(
        port: &mut dyn Receiver,
        class: u8,
        subclass: u8,
        every: u8,
    ) -> io::Result<()> {
        port.write(&generate(
            0x6,
            0x1,
            &[class, subclass, 0, every, 0, every, 0, 0],
        ))?;
        port.sleep(Duration::from_millis(10));
        Ok(())
    }

    /// `poll_msg(port, clas, subclass)`: a message with no payload, which asks for one.
    /// `// C#: ExtLibs/Utilities/ubx_m8p.cs:631-640`
    pub fn poll_msg(port: &mut dyn Receiver, class: u8, subclass: u8) -> io::Result<()> {
        port.write(&generate(class, subclass, &[]))?;
        port.sleep(Duration::from_millis(10));
        Ok(())
    }

    /// `SetupM8P(port, m8p_130plus)`: UART1 to 460800 at each baud rate the receiver may be at,
    /// USB, 1 Hz, stationary, the NMEA off, the survey-in, position and RTCM messages on - MSM7,
    /// or MSM4 for firmware 1.30 and the F9P - the raw data and hardware status on.
    /// `// C#: ExtLibs/Utilities/ubx_m8p.cs:440-572`
    pub fn setup_m8p(port: &mut dyn Receiver, m8p_130plus: bool) -> io::Result<()> {
        let ms = Duration::from_millis;
        for baud in [port.baud(), 9600, 38400, 57600, 115_200, 230_400, 460_800] {
            port.set_baud(baud);
            port.sleep(ms(50));
            // "U = bit 01010101 - often used for autobaud"
            port.write(b"UU")?;
            // "port config - 460800 - uart1"
            port.write(&generate(
                0x6,
                0x00,
                &[
                    0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0x08, 0x07, 0x00, 0x23,
                    0x00, 0x23, 0x00, 0x00, 0x00, 0x00, 0x00,
                ],
            ))?;
            port.sleep(ms(100));
        }
        // "port config - usb"
        port.write(&generate(
            0x6,
            0x00,
            &[
                0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x23, 0x00,
                0x23, 0x00, 0x00, 0x00, 0x00, 0x00,
            ],
        ))?;
        port.sleep(ms(300));
        port.set_baud(460_800);
        // "set rate to 1hz"
        port.write(&generate(0x6, 0x8, &[0xE8, 0x03, 0x01, 0x00, 0x01, 0x00]))?;
        port.sleep(ms(200));
        // "set navmode to stationary"
        port.write(&generate(
            0x6,
            0x24,
            &[
                0xFF, 0xFF, 0x02, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x0F, 0x00,
                0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x23, 0x10, 0x27,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            ],
        ))?;
        port.sleep(ms(200));
        // "turn off all nmea"
        for a in 0..=0xfu8 {
            if a == 0xb || a == 0xc || a == 0xe {
                continue;
            }
            turnon_off(port, 0xf0, a, 0)?;
        }
        // mon-ver
        poll_msg(port, 0xa, 0x4)?;
        // survey in, pvt, 1005 every 5 s
        turnon_off(port, 0x01, 0x3b, 1)?;
        turnon_off(port, 0x01, 0x07, 1)?;
        turnon_off(port, 0xf5, 0x05, 5)?;
        let (rate1, rate2) = if m8p_130plus { (0, 1) } else { (1, 0) };
        // 1074/1077, 1084/1087, 1094/1097, 1124/1127
        for (four, seven) in [(0x4a, 0x4d), (0x54, 0x57), (0x5e, 0x61), (0x7c, 0x7f)] {
            turnon_off(port, 0xf5, four, rate2)?;
            turnon_off(port, 0xf5, seven, rate1)?;
        }
        // 4072 off, 1230 every 5 s, NAV-VELNED, RXM-RAWX/RAW, RXM-SFRBX/SFRB, MON-HW
        turnon_off(port, 0xf5, 0xFE, 0)?;
        turnon_off(port, 0xf5, 0xE6, 5)?;
        turnon_off(port, 0x01, 0x12, 1)?;
        turnon_off(port, 0x02, 0x15, 1)?;
        turnon_off(port, 0x02, 0x10, 1)?;
        turnon_off(port, 0x02, 0x13, 2)?;
        turnon_off(port, 0x02, 0x11, 2)?;
        turnon_off(port, 0x0a, 0x09, 2)?;
        port.sleep(ms(100));
        Ok(())
    }

    /// The base position `SetupBasePos` is given: `PointLatLngAlt.Zero` asks for a survey in.
    #[derive(Debug, Clone, Copy, Default, PartialEq)]
    pub struct Position {
        /// `Lat`, or the ECEF X.
        pub lat: f64,
        /// `Lng`, or the ECEF Y.
        pub lng: f64,
        /// `Alt`, or the ECEF Z.
        pub alt: f64,
        /// Whether it equals `PointLatLngAlt.Zero`: (0, 0, 0) with no `Tag`.
        pub zero: bool,
    }

    /// `SetupBasePos(port, basepos, surveyindur, surveyinacc, disable)`: TMODE3 off, saved and the
    /// receiver restarted; or a survey in of at least `surveyindur` seconds (60 for 0) to
    /// `surveyinacc` metres (2 for 0); or the fixed position.
    /// `// C#: ExtLibs/Utilities/ubx_m8p.cs:574-618`
    pub fn setup_base_pos(
        port: &mut dyn Receiver,
        basepos: Position,
        surveyindur: i32,
        surveyinacc: f64,
        disable: bool,
    ) -> io::Result<()> {
        let ms = Duration::from_millis;
        port.sleep(ms(100));
        port.sleep(ms(100));
        let surveyindur = if surveyindur == 0 { 60 } else { surveyindur };
        let surveyinacc = if surveyinacc == 0.0 { 2.0 } else { surveyinacc };
        if disable {
            port.write(&generate(0x6, 0x71, &[0u8; 40]))?;
            // "save - bbr"
            port.write(&generate(
                0x06,
                0x09,
                &[0, 0, 0, 0, 0xff, 0xff, 0, 0, 0, 0, 0, 0, 0x01],
            ))?;
            port.sleep(ms(1000));
            // "reboot"
            port.write(&generate(0x06, 0x04, &[0x14, 0xff, 2, 0]))?;
            port.sleep(ms(3000));
            return Ok(());
        }
        let tmode = if basepos.zero {
            // `(uint) surveyindur`
            #[allow(clippy::cast_sign_loss)]
            Tmode3::survey_in(surveyindur as u32, surveyinacc)
        } else {
            Tmode3::fixed(basepos.lat, basepos.lng, basepos.alt)
        };
        port.write(&generate(0x6, 0x71, &tmode.bytes()))
    }
}

pub mod septentrio {
    //! `ExtLibs/Utilities/Septentrio.cs` and `Unicore.cs`: the base receivers configured with
    //! text commands. Septentrio's echo each command; Unicore's are not waited for.

    use std::io;
    use std::time::Duration;

    use super::ubx::Receiver;

    /// `DefaultBaudrate`.
    /// `// C#: ExtLibs/Utilities/Septentrio.cs:233`
    pub const DEFAULT_BAUDRATE: u32 = 115_200;

    /// `AckTimeout`, ms.
    /// `// C#: ExtLibs/Utilities/Septentrio.cs:228`
    pub const ACK_TIMEOUT: Duration = Duration::from_millis(1000);

    /// A receiver that answers: [`Receiver`], and the lines it sends back.
    pub trait Answering: Receiver {
        /// The next line from the port, `StreamReader.ReadLine`'s, or `None` when nothing whole
        /// came within `wait`.
        fn read_line(&mut self, wait: Duration) -> Option<String>;
    }

    /// How a Septentrio command went.
    #[derive(Debug, Clone, PartialEq, Eq)]
    pub enum Failure {
        /// `FailedAckException`: no echo within [`ACK_TIMEOUT`].
        FailedAck,
        /// `FormatException`, with its message.
        Format(String),
        /// `InvalidOperationException`, with its message.
        InvalidOperation(String),
        /// The port failed: the C#'s `IOException`.
        Io(String),
    }

    impl From<io::Error> for Failure {
        fn from(error: io::Error) -> Self {
            Self::Io(error.to_string())
        }
    }

    /// `SendAck`: the command, then lines until one holds it (less its newline); none within
    /// [`ACK_TIMEOUT`] is a failed acknowledgement.
    ///
    /// The C# checks the time only after each line it reads, so a receiver that says nothing
    /// leaves it waiting on `ReadLineAsync`; here the wait ends at the timeout either way.
    /// `// C#: ExtLibs/Utilities/Septentrio.cs:193-220`
    pub fn send_ack(port: &mut dyn Answering, command: &str) -> Result<(), Failure> {
        port.write(command.as_bytes())?;
        let wanted = command.get(..command.len().saturating_sub(1)).unwrap_or("");
        let started = std::time::Instant::now();
        while let Some(left) = ACK_TIMEOUT.checked_sub(started.elapsed()) {
            match port.read_line(left) {
                Some(line) if line.contains(wanted) => return Ok(()),
                Some(_) => {}
                None => break,
            }
        }
        Err(Failure::FailedAck)
    }

    /// `Single.ToString(format)` for a fixed number of decimals, as the .NET Framework does it: a
    /// `float` is first rounded to its seven significant digits, then written.
    #[must_use]
    pub fn single_fixed(value: f32, decimals: usize) -> String {
        let seven: f64 = format!("{value:.6e}").parse().unwrap_or(f64::from(value));
        format!("{seven:.decimals$}")
    }

    /// `ConfigureBaseReceiver`: 115200, the baud rate found and set, static, and RTCM 3 out.
    /// `// C#: ExtLibs/Utilities/Septentrio.cs:78-90`
    pub fn configure_base_receiver(port: &mut dyn Answering) -> Result<(), Failure> {
        port.set_baud(DEFAULT_BAUDRATE);
        configure_baud(port)?;
        send_ack(port, "setPVTMode,Static,All,Auto\n")?;
        send_ack(port, "setDataInOut,USB1+USB2+COM1+COM2+COM3,Auto,RTCMv3\n")
    }

    /// `ConfigureBaud`: at each rate the receiver may be at, its ports asked for 115200, until
    /// one acknowledges.
    /// `// C#: ExtLibs/Utilities/Septentrio.cs:119-143`
    fn configure_baud(port: &mut dyn Answering) -> Result<(), Failure> {
        let bauds = [
            port.baud(),
            1200,
            2400,
            4800,
            9600,
            19200,
            38400,
            57600,
            115_200,
            230_400,
            460_800,
        ];
        let command =
            format!("setCOMSettings,COM1+COM2+COM3,baud{DEFAULT_BAUDRATE},bits8,No,bit1,none\n");
        let mut acknowledged = false;
        for baud in bauds {
            port.set_baud(baud);
            if send_ack(port, &command).is_ok() {
                acknowledged = true;
                break;
            }
        }
        if !acknowledged {
            return Err(Failure::FailedAck);
        }
        port.set_baud(DEFAULT_BAUDRATE);
        Ok(())
    }

    /// `SetBasePosition`: the fixed position, then static at it.
    /// `// C#: ExtLibs/Utilities/Septentrio.cs:96-102`
    pub fn set_base_position(
        port: &mut dyn Answering,
        latitude: f32,
        longitude: f32,
        altitude: f32,
    ) -> Result<(), Failure> {
        send_ack(
            port,
            &format!(
                "setStaticPosGeodetic,Geodetic1,{},{},{},WGS84\n",
                single_fixed(latitude, 9),
                single_fixed(longitude, 9),
                single_fixed(altitude, 4)
            ),
        )?;
        send_ack(port, "setPVTMode,Static,,Geodetic1\n")
    }

    /// `SetAutoBasePosition`.
    /// `// C#: ExtLibs/Utilities/Septentrio.cs:108-113`
    pub fn set_auto_base_position(port: &mut dyn Answering) -> Result<(), Failure> {
        send_ack(port, "setPVTMode,Static,,auto\n")
    }

    /// `RTCMSignals`: GPS, GLONASS, BeiDou, Galileo.
    /// `// C#: ExtLibs/Utilities/Septentrio.cs:44-71`
    #[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
    pub struct Signals {
        /// `Gps`.
        pub gps: bool,
        /// `Glonass`.
        pub glonass: bool,
        /// `Beidou`.
        pub beidou: bool,
        /// `Galileo`.
        pub galileo: bool,
    }

    /// `SetEnabledRTCM`: the station messages and the MSM level of each constellation - 3 for
    /// Lite, 4 for Basic, 7 for anything else.
    /// `// C#: ExtLibs/Utilities/Septentrio.cs:149-178`
    pub fn set_enabled_rtcm(
        port: &mut dyn Answering,
        level: u8,
        signals: Signals,
    ) -> Result<(), Failure> {
        let level = match level {
            3 | 4 => level,
            _ => 7,
        };
        let mut messages = "RTCM1006+RTCM1033+RTCM1230".to_owned();
        if signals.gps {
            messages += &format!("+RTCM107{level}");
        }
        if signals.glonass {
            messages += &format!("+RTCM108{level}");
        }
        if signals.galileo {
            messages += &format!("+RTCM109{level}");
        }
        if signals.beidou {
            messages += &format!("+RTCM112{level}");
        }
        send_ack(
            port,
            &format!("setRTCMv3Output,COM1+COM2+COM3+USB1+USB2,{messages}\n"),
        )
    }

    /// `SetRTCMInterval`.
    /// `// C#: ExtLibs/Utilities/Septentrio.cs:184-187`
    pub fn set_rtcm_interval(port: &mut dyn Answering, interval: f32) -> Result<(), Failure> {
        send_ack(
            port,
            &format!(
                "setRTCMv3Interval,MSM3+MSM4+MSM7+RTCM1005|6+RTCM1033+RTCM1230,{}\n",
                single_fixed(interval, 1)
            ),
        )
    }

    /// Unicore's `ConfigureBaseReceiver`: 115200, then GGA and a 60 s survey in, the station and
    /// MSM4 messages on COM3, and saved - written, not waited for.
    /// `// C#: ExtLibs/Utilities/Unicore.cs:17-46`
    pub fn configure_unicore(port: &mut dyn Receiver) -> io::Result<()> {
        port.set_baud(115_200);
        for command in [
            "GPGGA COM3 1\r\n",
            "mode base time 60 2 2.5\r\n",
            "rtcm1006 com3 1\r\n",
            "rtcm1033 com3 1\r\n",
            "rtcm1074 com3 1\r\n",
            "rtcm1124 com3 1\r\n",
            "rtcm1084 com3 1\r\n",
            "rtcm1094 com3 1\r\n",
            "saveconfig\r\n",
        ] {
            port.write(command.as_bytes())?;
        }
        Ok(())
    }
}

// ---------------------------------------------------------------------------------------------
// The page's statics and its read loop: `comPort`, `t12`, `mainloop`, `sendData`,
// `ExtractBasePos`, `ProcessUBXMessage`.
// ---------------------------------------------------------------------------------------------

pub use worker::{Command, Event, Handler, Kind, OpenSpec, SeptentrioInputs, Setup, Shared, Svin};

pub mod worker {
    //! The thread `DoConnect` starts (`t12`, "injectgps") and what it shares with the page.
    //!
    //! In the C# the page's statics - the port, the parsers, the counts - are read and written by
    //! the page on the UI thread and by `mainloop` on its own; the page's controls are changed from
    //! the thread through `BeginInvoke`. Here the thread owns what it reads and writes, the page
    //! reads the counts through [`Shared`], and what the thread would have invoked on the page
    //! arrives as [`Event`]s the page takes once a frame.
    //!
    //! The C# opens the port, and configures an attached receiver, on the UI thread, which waits
    //! for them - an NTRIP caster's first line, a u-blox's seven baud rates. Here the thread does
    //! both before its loop, and the page carries on drawing meanwhile, its Connect button disabled
    //! as the C# disables it for the Septentrio and Unicore configurations.

    use std::collections::BTreeMap;
    use std::io::{self, Write as _};
    use std::path::PathBuf;
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::sync::{Arc, Mutex, MutexGuard};
    use std::time::{Duration, Instant};

    use mp_link::LinkSender;
    use mp_transport::{NtripOptions, Transport};
    use mp_vehicle::{LatLngAlt, VehicleId};

    use super::septentrio::{self, Answering, Failure, Signals};
    use super::ubx::{self, Receiver};
    use super::{nmea::Nmea, rtcm3, rtcm3::Rtcm3, sbp::Sbp, ubx::Ubx};

    /// A lock that a panicking holder did not poison for everyone else.
    fn lock<T>(mutex: &Mutex<T>) -> MutexGuard<'_, T> {
        mutex
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
    }

    /// The kinds of port `CMB_serialport` offers: the serial ports, then its four others.
    /// `// C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.cs:89-93, 334-372`
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum Kind {
        /// A serial port: `SerialPort`.
        Serial,
        /// "UDP Host": `UdpSerial`, listening.
        UdpHost,
        /// "UDP Client": `UdpSerialConnect`.
        UdpClient,
        /// "TCP Client": `TcpSerial`.
        TcpClient,
        /// "NTRIP": `CommsNTRIP`.
        Ntrip,
    }

    impl Kind {
        /// The kind `CMB_serialport.Text` names.
        #[must_use]
        pub fn of(text: &str) -> Self {
            match text {
                "NTRIP" => Self::Ntrip,
                "TCP Client" => Self::TcpClient,
                "UDP Host" => Self::UdpHost,
                "UDP Client" => Self::UdpClient,
                _ => Self::Serial,
            }
        }

        /// Whether the read loop leaves its reconnecting to the port itself: `comPort is
        /// CommsNTRIP || comPort is UdpSerialConnect || comPort is UdpSerial`.
        /// `// C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.cs:672`
        const fn reconnects_itself(self) -> bool {
            matches!(self, Self::Ntrip | Self::UdpClient | Self::UdpHost)
        }
    }

    /// What `DoConnect` opens: the port and what its `Open` asked for.
    #[derive(Debug, Clone)]
    pub struct OpenSpec {
        /// The kind of port.
        pub kind: Kind,
        /// `comPort.PortName`, for a serial port.
        pub name: String,
        /// `comPort.BaudRate`.
        pub baud: u32,
        /// The host `Open`'s question was answered with.
        pub host: String,
        /// The port it was answered with, as typed.
        pub port: String,
        /// The caster's URL, for NTRIP.
        pub url: String,
        /// `lat`, `lng`, `alt` and `ntrip_v1` of the `CommsNTRIP`.
        pub ntrip: NtripOptions,
        /// `UdpSerial.Open` returns without opening when its question is cancelled, and
        /// `DoConnect` carries on regardless.
        /// `// C#: ExtLibs/Comms/CommsUdpSerial.cs:114-115`
        pub cancelled: bool,
    }

    /// The Septentrio panel's boxes, as a handler reads them.
    #[derive(Debug, Clone, Default)]
    pub struct SeptentrioInputs {
        /// `chk_septentriofixedposition.Checked`.
        pub fixed: bool,
        /// The three position boxes' text.
        pub position: [String; 3],
        /// `cmb_septentriortcmamount.Text`.
        pub amount: String,
        /// `input_septentriortcminterval.Text`.
        pub interval: String,
        /// The four constellation boxes.
        pub signals: Signals,
    }

    /// What `DoConnect` configures once the port is open, from the page's boxes.
    #[derive(Debug, Clone, Default)]
    pub struct Setup {
        /// `chk_autoconfig.Checked`, and `comboBoxConfigType.Text`.
        pub autoconfig: Option<String>,
        /// `chk_m8p_130p.Checked`.
        pub m8p_130p: bool,
        /// The page's `basepos`.
        pub basepos: ubx::Position,
        /// The Septentrio panel.
        pub septentrio: SeptentrioInputs,
    }

    /// Which handler a command to the receiver came from: each catches differently.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum Handler {
        /// `DoConnect`'s `ConfigureSeptentrioReceiver`.
        Connect,
        /// `chk_septentriofixedposition_Click`.
        FixedPosition,
        /// `button_septentriosetposition_Click`.
        SetPosition,
        /// `cmb_septentriortcmamount_SelectedIndexChanged`.
        Amount,
        /// `button_septentriortcminterval_Click`.
        Interval,
        /// `chk_septentrioconstellation_Click`.
        Constellation,
    }

    /// What the page asks of the receiver while the port is open.
    #[derive(Debug, Clone)]
    pub enum Command {
        /// `dg_basepos_CellContentClick`'s Use: `SetupBasePos` and a TMODE3 poll.
        UseBase {
            /// The position.
            basepos: ubx::Position,
            /// `int.Parse(txt_surveyinDur.Text)`.
            duration: i32,
            /// `double.Parse(txt_surveyinAcc.Text)`.
            accuracy: f64,
        },
        /// `but_restartsvin_Click`: TMODE3 off, `SetupM8P`, then the survey in.
        Restart {
            /// `chk_m8p_130p.Checked`.
            m8p_130p: bool,
            /// The two boxes' text, parsed inside the C#'s `try`.
            duration: String,
            /// The accuracy box.
            accuracy: String,
        },
        /// `UpdateSeptentrioBasePosition`.
        SeptentrioPosition(Handler, SeptentrioInputs),
        /// `button_septentriosetposition_Click`'s `SetBasePosition`.
        SeptentrioSetPosition([f32; 3], [String; 3]),
        /// `UpdateSeptentrioRTCMSettings`.
        SeptentrioRtcm(Handler, SeptentrioInputs),
    }

    /// `updateSVINLabel`'s arguments, and the statics it reads.
    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct Svin {
        /// `valid`.
        pub valid: bool,
        /// `active`.
        pub active: bool,
        /// `dur`.
        pub dur: u32,
        /// `obs`.
        pub obs: u32,
        /// `acc`, metres.
        pub acc: f64,
        /// `ubxsvin.getECEF()`.
        pub ecef: [f64; 3],
        /// `ubxmode`.
        pub mode: ubx::Tmode3,
    }

    /// What the thread would have done to the page through `BeginInvoke`, and `DoConnect`'s
    /// steps after the port opened.
    #[derive(Debug, Clone)]
    pub enum Event {
        /// The port is open and configured, and the loop has started: `DoConnect`'s end.
        Started,
        /// `DoConnect` returned early with a message box; `open` says whether the port is left
        /// open, as a failed u-blox configuration leaves it.
        Failed {
            /// The box's text.
            message: String,
            /// `comPort.IsOpen` after.
            open: bool,
        },
        /// A message box.
        Message(String),
        /// `CMB_baudrate.Text = ...`.
        Baud(u32),
        /// `Settings.Instance[key] = value`.
        Setting(&'static str, String),
        /// `updateSVINLabel(...)`.
        Svin(Svin),
        /// `Rtcm3_ObsMessage`.
        Obs(Vec<rtcm3::Ob>),
        /// `seenRTCM(seenmsg)`.
        Seen(i32),
        /// A handler's command has ended.
        Done(Handler, Result<(), Failure>),
    }

    /// Where `sendData` sends: `MainV2.Comports`, which here is the one link, and its vehicles;
    /// and `MainV2.comPort.MAV`, whose `cs.Base` the base station's position goes to.
    #[derive(Debug, Default)]
    pub struct Route {
        /// The link, while there is one with a vehicle on it.
        pub sender: Option<LinkSender>,
        /// `port.MAVlist`: every vehicle heard on it.
        pub vehicles: Vec<VehicleId>,
        /// The vehicle being shown.
        pub selected: Option<VehicleId>,
    }

    /// The page's counters and last base, statics in the C#.
    #[derive(Debug, Default)]
    pub struct Counts {
        /// `msgseen`.
        pub msgseen: BTreeMap<String, i32>,
        /// `bytes`.
        pub bytes: u64,
        /// `bps`.
        pub bps: u64,
        /// `bpsusefull`.
        pub bpsusefull: u64,
        /// `status_line3`.
        pub status_line3: Option<String>,
        /// Bytes handed to `InjectGpsData` with a vehicle to take them.
        pub injected: u64,
        /// The MAVLink messages that made.
        pub injected_messages: u64,
        /// `cs.Base` of `MainV2.comPort.MAV` while no vehicle is connected: the blank `MAVState`
        /// the C# writes it into then.
        pub base: Option<LatLngAlt>,
        /// Bytes written to the `.gpsbase` file.
        pub logged: u64,
        /// The `.gpsbase` file.
        pub log: Option<PathBuf>,
    }

    /// The statics `ProcessUBXMessage` keeps.
    #[derive(Debug, Default)]
    struct UbxState {
        /// `pollTMODE`: `None` is `DateTime.MinValue`.
        poll_tmode: Option<Instant>,
        /// `ubxmode`.
        mode: ubx::Tmode3,
        /// `ubxsvin`.
        svin: ubx::NavSvin,
    }

    /// The parsers: statics, so a frame cut short by a disconnect is finished by the next
    /// connection's bytes.
    #[derive(Debug, Default)]
    struct Parsers {
        rtcm3: Rtcm3,
        sbp: Sbp,
        ubx: Ubx,
        nmea: Nmea,
        ubx_state: UbxState,
    }

    /// Everything the page and its thread share, for the life of the application.
    #[derive(Debug)]
    pub struct Shared {
        /// `rtcm_msg`: `GPS_RTCM_DATA` rather than `GPS_INJECT_DATA`.
        pub rtcm_msg: AtomicBool,
        /// The counters.
        pub counts: Mutex<Counts>,
        /// Where to send.
        pub route: Mutex<Route>,
        /// What the thread has done to the page, oldest first.
        pub events: Mutex<Vec<Event>>,
        /// `basedata.Flush()`, asked for by the page's timer.
        pub flush: AtomicBool,
        /// The parsers and the u-blox statics.
        parsers: Mutex<Parsers>,
    }

    impl Default for Shared {
        fn default() -> Self {
            Self {
                // `private static bool rtcm_msg = true;`
                // `// C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.cs:52`
                rtcm_msg: AtomicBool::new(true),
                counts: Mutex::new(Counts::default()),
                route: Mutex::new(Route::default()),
                events: Mutex::new(Vec::new()),
                flush: AtomicBool::new(false),
                parsers: Mutex::new(Parsers::default()),
            }
        }
    }

    impl Shared {
        /// The counters.
        pub fn counts(&self) -> MutexGuard<'_, Counts> {
            lock(&self.counts)
        }

        /// Where to send.
        pub fn route(&self) -> MutexGuard<'_, Route> {
            lock(&self.route)
        }

        /// Takes what the thread has done since the last call.
        pub fn take_events(&self) -> Vec<Event> {
            std::mem::take(&mut *lock(&self.events))
        }

        fn push(&self, event: Event) {
            lock(&self.events).push(event);
        }

        /// `sendData(data, length)`: to every vehicle on the link, or to the first alone when
        /// `GPS_RTCM_DATA` goes out and all of them speak the same MAVLink - which, as this link
        /// sends every message as MAVLink 2, they do.
        /// `// C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.cs:1115-1151`
        pub fn send_data(&self, data: &[u8]) {
            let rtcm_msg = self.rtcm_msg.load(Ordering::Acquire);
            let (sent, messages) = {
                let route = lock(&self.route);
                let Some(sender) = route.sender.as_ref() else {
                    return;
                };
                // `GetCanJustSendOnce`: `rtcm_msg && GetAreAllUsingSameMavlinkVersion`.
                let once = rtcm_msg;
                let mut messages = 0;
                let mut sent = false;
                for &vehicle in &route.vehicles {
                    messages += sender.inject_gps_data(vehicle, data, rtcm_msg);
                    sent = true;
                    if once {
                        break;
                    }
                }
                (sent, messages)
            };
            if sent {
                let mut counts = self.counts();
                counts.injected += data.len() as u64;
                counts.injected_messages += messages as u64;
            }
        }

        /// `MainV2.comPort.MAV.cs.Base = position`.
        fn set_base(&self, position: LatLngAlt) {
            let route = lock(&self.route);
            match (route.sender.as_ref(), route.selected) {
                (Some(sender), Some(selected)) => sender.set_base(selected, position),
                _ => self.counts().base = Some(position),
            }
        }
    }

    /// The open port, shared by the read loop and the handlers' commands, as the C#'s `comPort`
    /// is by `mainloop` and the UI thread.
    pub enum Port {
        /// A serial port, kept as one so its baud rate can be changed.
        Serial(mp_transport::SerialTransport),
        /// Any other.
        Other(Box<dyn Transport>),
    }

    impl std::fmt::Debug for Port {
        fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
            match self {
                Self::Serial(serial) => serial.fmt(f),
                Self::Other(other) => f.write_str(other.description()),
            }
        }
    }

    impl Port {
        fn io(&mut self) -> &mut dyn Transport {
            match self {
                Self::Serial(serial) => serial,
                Self::Other(other) => other.as_mut(),
            }
        }

        fn is_open(&mut self) -> bool {
            self.io().is_open()
        }
    }

    /// `ICommsSerial.Open()` for the spec: the port, or the text of the C#'s exception.
    fn open_transport(spec: &OpenSpec) -> Result<Port, String> {
        let parse_port = |text: &str| {
            text.trim().parse::<u16>().map_err(|_| {
                format!(
                    "System.FormatException: Input string was not in a correct format. ({text})"
                )
            })
        };
        let other = Port::Other;
        match spec.kind {
            Kind::Serial => mp_transport::SerialTransport::open(&spec.name, spec.baud)
                .map(Port::Serial)
                .map_err(|e| e.to_string()),
            Kind::TcpClient => {
                mp_transport::TcpTransport::connect(&spec.host, parse_port(&spec.port)?)
                    .map(|t| other(Box::new(t)))
                    .map_err(|e| e.to_string())
            }
            Kind::UdpHost => mp_transport::UdpTransport::bind("0.0.0.0", parse_port(&spec.port)?)
                .map(|t| other(Box::new(t)))
                .map_err(|e| e.to_string()),
            Kind::UdpClient => {
                mp_transport::UdpClientTransport::open(&spec.host, parse_port(&spec.port)?)
                    .map(|t| other(Box::new(t)))
                    .map_err(|e| e.to_string())
            }
            Kind::Ntrip => mp_transport::NtripTransport::open(&spec.url, spec.ntrip)
                .map(|t| other(Box::new(t)))
                .map_err(|e| e.to_string()),
        }
    }

    /// The port as `ubx_m8p` and `Septentrio` use it: locked for each write and read, so the read
    /// loop carries on between them, and with the baud rate last set.
    struct Handle {
        port: Arc<Mutex<Port>>,
        /// `comPort.BaudRate`.
        baud: u32,
        /// Bytes read after the last whole line.
        pending: Vec<u8>,
    }

    impl Receiver for Handle {
        fn write(&mut self, bytes: &[u8]) -> io::Result<()> {
            lock(&self.port).io().write_all(bytes)
        }

        fn baud(&self) -> u32 {
            self.baud
        }

        fn set_baud(&mut self, baud: u32) {
            self.baud = baud;
            if let Port::Serial(serial) = &mut *lock(&self.port) {
                let _ = serial.set_baud(baud);
            }
        }

        fn sleep(&mut self, time: Duration) {
            std::thread::sleep(time);
        }
    }

    impl Answering for Handle {
        fn read_line(&mut self, wait: Duration) -> Option<String> {
            let until = Instant::now() + wait;
            loop {
                // `ReadLine`: a line ends at `\n`, `\r` or `\r\n`.
                if let Some(end) = self.pending.iter().position(|&b| b == b'\n' || b == b'\r') {
                    let line = String::from_utf8_lossy(self.pending.get(..end).unwrap_or_default())
                        .into_owned();
                    let crlf = self.pending.get(end) == Some(&b'\r')
                        && self.pending.get(end + 1) == Some(&b'\n');
                    self.pending.drain(..end + if crlf { 2 } else { 1 });
                    return Some(line);
                }
                let left = until.checked_duration_since(Instant::now())?;
                let mut buf = [0u8; 256];
                let n = {
                    let mut port = lock(&self.port);
                    let io = port.io();
                    let _ = io.set_read_timeout(
                        left.min(Duration::from_millis(20))
                            .max(Duration::from_millis(1)),
                    );
                    io.read(&mut buf).unwrap_or(0)
                };
                self.pending
                    .extend_from_slice(buf.get(..n).unwrap_or_default());
            }
        }
    }

    /// One `DoConnect`'s port and thread, as the page holds them.
    #[derive(Debug)]
    pub struct Worker {
        /// `threadrun`, for this thread.
        pub run: Arc<AtomicBool>,
        /// `comPort.IsOpen`.
        pub open: Arc<AtomicBool>,
        /// Whether the loop has started.
        pub running: Arc<AtomicBool>,
        /// The port, once open.
        port: Arc<Mutex<Option<Arc<Mutex<Port>>>>>,
        /// The shared statics.
        shared: Arc<Shared>,
        /// The thread.
        thread: Option<std::thread::JoinHandle<()>>,
    }

    impl Worker {
        /// `DoConnect` from the port's creation on: opens it on a thread of its own, configures
        /// the receiver, and runs `mainloop`.
        #[must_use]
        pub fn start(
            spec: OpenSpec,
            setup: Setup,
            shared: Arc<Shared>,
            log_dir: Option<PathBuf>,
        ) -> Self {
            Self::start_on(spec, setup, shared, log_dir, None)
        }

        /// [`Worker::start`] on a port already open - a test's in-memory one - rather than the
        /// one `spec` names.
        #[must_use]
        pub fn start_on(
            spec: OpenSpec,
            setup: Setup,
            shared: Arc<Shared>,
            log_dir: Option<PathBuf>,
            preopened: Option<Port>,
        ) -> Self {
            let run = Arc::new(AtomicBool::new(true));
            let open = Arc::new(AtomicBool::new(false));
            let running = Arc::new(AtomicBool::new(false));
            let port = Arc::new(Mutex::new(None));
            let thread = {
                let (run, open, running, port, shared) = (
                    Arc::clone(&run),
                    Arc::clone(&open),
                    Arc::clone(&running),
                    Arc::clone(&port),
                    Arc::clone(&shared),
                );
                std::thread::Builder::new()
                    .name("injectgps".to_owned())
                    .spawn(move || {
                        let flags = Flags {
                            run: &run,
                            open: &open,
                            running: &running,
                            slot: &port,
                        };
                        connect(&spec, &setup, &shared, &flags, preopened, log_dir);
                    })
                    .ok()
            };
            Self {
                run,
                open,
                running,
                port,
                shared,
                thread,
            }
        }

        /// `comPort.IsOpen`.
        #[must_use]
        pub fn is_open(&self) -> bool {
            self.open.load(Ordering::Acquire)
        }

        /// Whether `mainloop` is running.
        #[must_use]
        pub fn is_running(&self) -> bool {
            self.running.load(Ordering::Acquire)
        }

        /// `threadrun = false; comPort.Close();`: the port is closed from here on, and the thread,
        /// at its next pass, closes it and the `.gpsbase` file.
        ///
        /// The C# closes the port on the UI thread there and then. Here the thread closes it: a
        /// port can be held for a while by a read in progress - an NTRIP caster being reconnected
        /// to waits for its first line - and the page must not wait with it.
        pub fn stop(&mut self) {
            self.run.store(false, Ordering::Release);
            self.open.store(false, Ordering::Release);
            lock(&self.port).take();
            // Not joined, for the same reason: a thread configuring a receiver sleeps for seconds.
            self.thread.take();
        }

        /// A handler's command, run on a thread of its own against the open port, as the C#
        /// runs it on the UI thread beside the read loop. With no port open, nothing is sent.
        pub fn command(&self, command: Command) {
            let Some(port) = lock(&self.port).clone() else {
                return;
            };
            let shared = Arc::clone(&self.shared);
            let _ = std::thread::Builder::new()
                .name("injectgps-command".to_owned())
                .spawn(move || run_command(command, port, &shared));
        }
    }

    /// `double.Parse` and `int.Parse`: the C#'s `FormatException` text when the text is not a
    /// number.
    fn format_error() -> String {
        "System.FormatException: Input string was not in a correct format.".to_owned()
    }

    fn run_command(command: Command, port: Arc<Mutex<Port>>, shared: &Shared) {
        let mut handle = Handle {
            port,
            baud: 0,
            pending: Vec::new(),
        };
        match command {
            // C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.cs:1320-1327 - no `try`.
            Command::UseBase {
                basepos,
                duration,
                accuracy,
            } => {
                let _ = ubx::setup_base_pos(&mut handle, basepos, duration, accuracy, false)
                    .and_then(|()| ubx::poll_msg(&mut handle, 0x06, 0x71));
            }
            // C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.cs:1403-1421
            Command::Restart {
                m8p_130p,
                duration,
                accuracy,
            } => {
                let zero = ubx::Position {
                    zero: true,
                    ..ubx::Position::default()
                };
                let result = ubx::setup_base_pos(&mut handle, zero, 0, 0.0, true)
                    .map_err(|e| e.to_string())
                    .and_then(|()| ubx::setup_m8p(&mut handle, m8p_130p).map_err(|e| e.to_string()))
                    .and_then(|()| {
                        let duration =
                            duration.trim().parse::<i32>().map_err(|_| format_error())?;
                        let accuracy =
                            accuracy.trim().parse::<f64>().map_err(|_| format_error())?;
                        ubx::setup_base_pos(&mut handle, zero, duration, accuracy, false)
                            .map_err(|e| e.to_string())
                    });
                if let Err(why) = result {
                    shared.push(Event::Message(format!("Error configuring\n{why}")));
                }
            }
            Command::SeptentrioPosition(handler, inputs) => {
                let result = update_septentrio_base_position(&mut handle, &inputs, shared);
                shared.push(Event::Done(handler, result));
            }
            // C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.cs:1478-1491
            Command::SeptentrioSetPosition([lat, lng, alt], texts) => {
                let result = septentrio::set_base_position(&mut handle, lat, lng, alt);
                if result.is_ok() {
                    let [lat, lng, alt] = texts;
                    shared.push(Event::Setting(
                        "SerialInjectGPS_SeptentrioFixedAtitude",
                        lat,
                    ));
                    shared.push(Event::Setting(
                        "SerialInjectGPS_SeptentrioFixedLongitude",
                        lng,
                    ));
                    shared.push(Event::Setting(
                        "SerialInjectGPS_SeptentrioFixedAltitude",
                        alt,
                    ));
                }
                shared.push(Event::Done(Handler::SetPosition, result));
            }
            Command::SeptentrioRtcm(handler, inputs) => {
                let result = update_septentrio_rtcm_settings(&mut handle, &inputs, shared);
                shared.push(Event::Done(handler, result));
            }
        }
    }

    /// `float.Parse` of the three position boxes.
    fn position_of(inputs: &SeptentrioInputs) -> Result<[f32; 3], Failure> {
        let parse = |text: &String| {
            text.trim().parse::<f32>().map_err(|_| {
                Failure::Format("Input string was not in a correct format.".to_owned())
            })
        };
        let [lat, lng, alt] = &inputs.position;
        Ok([parse(lat)?, parse(lng)?, parse(alt)?])
    }

    /// `UpdateSeptentrioBasePosition`: the fixed position in range, or automatic; and the box
    /// saved.
    /// `// C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.cs:1499-1518`
    fn update_septentrio_base_position(
        port: &mut dyn Answering,
        inputs: &SeptentrioInputs,
        shared: &Shared,
    ) -> Result<(), Failure> {
        if inputs.fixed {
            let [latitude, longitude, altitude] = position_of(inputs)?;
            if !((-90.0..=90.0).contains(&latitude)
                && (-180.0..=180.0).contains(&longitude)
                && (-1000.0..=30000.0).contains(&altitude))
            {
                return Err(Failure::InvalidOperation(
                    "Operation is not valid due to the current state of the object.".to_owned(),
                ));
            }
            septentrio::set_base_position(port, latitude, longitude, altitude)?;
        } else {
            septentrio::set_auto_base_position(port)?;
        }
        shared.push(Event::Setting(
            "SerialInjectGPS_SeptentrioFixedPosition",
            bool_text(inputs.fixed),
        ));
        Ok(())
    }

    /// `bool.ToString()`.
    #[must_use]
    pub fn bool_text(value: bool) -> String {
        if value { "True" } else { "False" }.to_owned()
    }

    /// `UpdateSeptentrioRTCMSettings`: the constellations at the level chosen, the four boxes
    /// saved, then the interval - a number from 0.1 to 600 - set and saved.
    /// `// C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.cs:1546-1595`
    fn update_septentrio_rtcm_settings(
        port: &mut dyn Answering,
        inputs: &SeptentrioInputs,
        shared: &Shared,
    ) -> Result<(), Failure> {
        let level = match inputs.amount.as_str() {
            "Lite" => 3,
            "Full" => 7,
            _ => 4,
        };
        septentrio::set_enabled_rtcm(port, level, inputs.signals)?;
        let signals = inputs.signals;
        for (key, on) in [
            ("SerialInjectGPS_SeptentrioGPS", signals.gps),
            ("SerialInjectGPS_SeptentrioGLONASS", signals.glonass),
            ("SerialInjectGPS_SeptentrioGalileo", signals.galileo),
            ("SerialInjectGPS_SeptentrioBeiDou", signals.beidou),
        ] {
            shared.push(Event::Setting(key, bool_text(on)));
        }
        let interval =
            inputs.interval.trim().parse::<f32>().map_err(|_| {
                Failure::Format("The RTCM interval isn't a valid number".to_owned())
            })?;
        if !(0.1..=600.0).contains(&interval) {
            return Err(Failure::InvalidOperation(
                "The RTCM interval should be between 0,1 and 600".to_owned(),
            ));
        }
        septentrio::set_rtcm_interval(port, interval)?;
        shared.push(Event::Setting(
            "SerialInjectGPS_SeptentrioRTCMInterval",
            inputs.interval.clone(),
        ));
        Ok(())
    }

    /// The thread's flags and the page's handle on its port.
    struct Flags<'a> {
        run: &'a AtomicBool,
        open: &'a AtomicBool,
        running: &'a AtomicBool,
        slot: &'a Mutex<Option<Arc<Mutex<Port>>>>,
    }

    /// `DoConnect` from the port's creation, then `mainloop`.
    /// `// C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.cs:384-523`
    fn connect(
        spec: &OpenSpec,
        setup: &Setup,
        shared: &Arc<Shared>,
        flags: &Flags<'_>,
        preopened: Option<Port>,
        log_dir: Option<PathBuf>,
    ) {
        let Flags {
            run,
            open,
            running,
            slot,
        } = *flags;
        let port = if spec.cancelled {
            None
        } else if let Some(port) = preopened {
            Some(Arc::new(Mutex::new(port)))
        } else {
            match open_port(spec) {
                Ok(port) => Some(Arc::new(Mutex::new(port))),
                Err(why) => {
                    // C#: ConfigSerialInjectGPS.cs:447-452
                    shared.push(Event::Failed {
                        message: format!(
                            "Error Connecting\nif using com0com please rename the ports to COM??\n{why}"
                        ),
                        open: false,
                    });
                    return;
                }
            }
        };
        if let Some(port) = &port {
            open.store(true, Ordering::Release);
            *lock(slot) = Some(Arc::clone(port));
        }
        let mut basedata = open_basedata(log_dir, shared);

        let mut handle = port.as_ref().map(|port| Handle {
            port: Arc::clone(port),
            baud: spec.baud,
            pending: Vec::new(),
        });
        match (setup.autoconfig.as_deref(), handle.as_mut()) {
            // C#: ConfigSerialInjectGPS.cs:455-482
            (Some("UBlox M8P/F9P"), Some(handle)) => {
                if let Err(why) = ubx::setup_m8p(handle, setup.m8p_130p) {
                    shared.push(Event::Failed {
                        message: format!("Error configuring\n{why}"),
                        open: true,
                    });
                    // The port stays open, with no loop reading it, until Connect closes it.
                    while run.load(Ordering::Acquire) {
                        std::thread::sleep(Duration::from_millis(50));
                    }
                    open.store(false, Ordering::Release);
                    lock(&handle.port).io().close();
                    return;
                }
                if !setup.basepos.zero {
                    let _ = ubx::setup_base_pos(handle, setup.basepos, 0, 0.0, true);
                    let _ = ubx::setup_base_pos(handle, setup.basepos, 0, 0.0, false);
                }
                shared.push(Event::Baud(460_800));
            }
            // C#: ConfigSerialInjectGPS.cs:483-495, 553-582
            (Some("Septentrio"), Some(handle)) => {
                let result = septentrio::configure_base_receiver(handle)
                    .and_then(|()| {
                        update_septentrio_base_position(handle, &setup.septentrio, shared)
                    })
                    .and_then(|()| {
                        update_septentrio_rtcm_settings(handle, &setup.septentrio, shared)
                    });
                shared.push(Event::Done(Handler::Connect, result));
                shared.push(Event::Baud(septentrio::DEFAULT_BAUDRATE));
            }
            // C#: ConfigSerialInjectGPS.cs:496-509, 525-548
            (Some("Unicore UM982"), Some(handle)) => {
                let _ = septentrio::configure_unicore(handle);
            }
            _ => {}
        }
        drop(handle);

        running.store(true, Ordering::Release);
        {
            // C#: ConfigSerialInjectGPS.cs:519-520
            let mut counts = shared.counts();
            counts.msgseen.clear();
            counts.bytes = 0;
        }
        shared.push(Event::Started);
        mainloop(spec, shared, run, open, port.as_ref(), &mut basedata);
        running.store(false, Ordering::Release);
        open.store(false, Ordering::Release);
        if let Some(port) = &port {
            lock(port).io().close();
        }
        if let Some(file) = basedata.as_mut() {
            let _ = file.flush();
        }
    }

    /// `comPort.Open()`, and for a serial port the three lines a CAN adapter wants.
    /// `// C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.cs:399-431`
    fn open_port(spec: &OpenSpec) -> Result<Port, String> {
        let mut port = open_transport(spec)?;
        if spec.kind == Kind::Serial {
            // "this is for a CAN adapter"
            let io = port.io();
            io.write_all(b"\r\r\r").map_err(|e| e.to_string())?;
            std::thread::sleep(Duration::from_millis(50));
            io.write_all(b"S8\r").map_err(|e| e.to_string())?;
            std::thread::sleep(Duration::from_millis(50));
            io.write_all(b"O\r").map_err(|e| e.to_string())?;
        }
        Ok(port)
    }

    /// `basedata`: `yyyy-MM-dd HH-mm-ss.gpsbase` in the log directory, a new file; a box if it
    /// cannot be made, and the connection carries on without it.
    /// `// C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.cs:434-445`
    fn open_basedata(
        log_dir: Option<PathBuf>,
        shared: &Shared,
    ) -> Option<io::BufWriter<std::fs::File>> {
        let dir = log_dir?;
        let path = dir.join(format!(
            "{}.gpsbase",
            chrono::Local::now().format("%Y-%m-%d %H-%M-%S")
        ));
        match std::fs::OpenOptions::new()
            .read(true)
            .write(true)
            .create_new(true)
            .open(&path)
        {
            Ok(file) => {
                let mut counts = shared.counts();
                counts.log = Some(path);
                counts.logged = 0;
                // `BufferedStream`'s default buffer.
                Some(io::BufWriter::with_capacity(4096, file))
            }
            Err(why) => {
                shared.push(Event::Message(format!(
                    "Error creating file to save base data into {why}"
                )));
                None
            }
        }
    }

    /// Ten seconds with nothing read, or the port closed: reconnect, unless the port reconnects
    /// itself.
    /// `// C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.cs:661`
    const RECONNECT_TIMEOUT: Duration = Duration::from_secs(10);

    /// `mainloop`: read, log, detect, inject, count.
    /// `// C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.cs:615-805`
    fn mainloop(
        spec: &OpenSpec,
        shared: &Shared,
        run: &AtomicBool,
        open: &AtomicBool,
        port: Option<&Arc<Mutex<Port>>>,
        basedata: &mut Option<io::BufWriter<std::fs::File>>,
    ) {
        let mut lastrecv = Instant::now();
        let (mut isrtcm, mut issbp) = (false, false);
        // `iscan`: only DroneCAN's RTCM stream sets it, and DroneCAN is not ported (see the
        // module's notes).
        let iscan = false;
        while run.load(Ordering::Acquire) {
            // "reconnect logic - 10 seconds with no data, or comport is closed"
            if let Some(port) = port {
                let is_open = lock(port).is_open();
                open.store(is_open && run.load(Ordering::Acquire), Ordering::Release);
                if lastrecv.elapsed() > RECONNECT_TIMEOUT || !is_open {
                    if !spec.kind.reconnects_itself() {
                        // "Reconnecting": `comPort.Close(); comPort.Open();` - the port opened
                        // again as it was, without `DoConnect`'s lines for a CAN adapter.
                        lock(port).io().close();
                        let reopened = open_transport(spec).map(|fresh| {
                            if run.load(Ordering::Acquire) {
                                *lock(port) = fresh;
                            }
                        });
                        if reopened.is_err() {
                            // "Failed to reconnect": ten seconds' sleep.
                            let until = Instant::now() + Duration::from_secs(10);
                            while Instant::now() < until && run.load(Ordering::Acquire) {
                                std::thread::sleep(Duration::from_millis(50));
                            }
                        }
                    }
                    lastrecv = Instant::now();
                }
            }
            if shared.flush.swap(false, Ordering::AcqRel)
                && let Some(file) = basedata.as_mut()
                && file.flush().is_err()
            {
                *basedata = None;
            }
            // "limit to 110 byte packets", or 180 for the new message.
            let size = if shared.rtcm_msg.load(Ordering::Acquire) {
                180
            } else {
                110
            };
            let mut buffer = [0u8; 180];
            let Some(port) = port else {
                std::thread::sleep(Duration::from_millis(10));
                continue;
            };
            // `while (comPort.BytesToRead > 0)`: read until a read finds nothing.
            loop {
                let read = {
                    let mut held = lock(port);
                    let io = held.io();
                    let _ = io.set_read_timeout(Duration::from_millis(10));
                    io.read(buffer.get_mut(..size).unwrap_or_default())
                };
                let n = match read {
                    Ok(0) | Err(_) => break,
                    Ok(n) => n,
                };
                lastrecv = Instant::now();
                let chunk = buffer.get(..n).unwrap_or_default();
                {
                    let mut counts = shared.counts();
                    counts.bytes += n as u64;
                    counts.bps += n as u64;
                }
                if let Some(file) = basedata.as_mut()
                    && file.write_all(chunk).is_ok()
                {
                    shared.counts().logged += n as u64;
                }
                // "if this is raw data transport of unknown packet types"
                if !(isrtcm || issbp || iscan) {
                    shared.send_data(chunk);
                }
                if process(chunk, shared, port, &mut isrtcm, &mut issbp, iscan).is_err() {
                    // The C#'s `catch`: the rest of the chunk, and of this pass, dropped.
                    break;
                }
            }
            std::thread::sleep(Duration::from_millis(10));
        }
    }

    /// The loop over a chunk's bytes: each parser in turn, a message found by one resetting the
    /// others.
    /// `// C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.cs:725-795`
    fn process(
        chunk: &[u8],
        shared: &Shared,
        port: &Arc<Mutex<Port>>,
        isrtcm: &mut bool,
        issbp: &mut bool,
        iscan: bool,
    ) -> Result<(), super::Overflow> {
        let mut parsers = lock(&shared.parsers);
        let parsers = &mut *parsers;
        let seen = |shared: &Shared, name: String| {
            *shared.counts().msgseen.entry(name).or_insert(0) += 1;
        };
        for &b in chunk {
            // "rtcm and not can"
            if !iscan {
                let found = parsers.rtcm3.read(b)?;
                if found > 0 {
                    parsers.sbp.reset_parser();
                    parsers.ubx.reset_parser();
                    parsers.nmea.reset_parser();
                    *isrtcm = true;
                    shared.send_data(parsers.rtcm3.packet());
                    shared.counts().bpsusefull += parsers.rtcm3.length() as u64;
                    seen(shared, format!("Rtcm{found}"));
                    if let Some(obs) = parsers.rtcm3.take_obs()
                        && !obs.is_empty()
                    {
                        shared.push(Event::Obs(obs));
                    }
                    extract_base_pos(found, parsers.rtcm3.packet(), shared);
                    shared.push(Event::Seen(found));
                }
            }
            // sbp
            let found = parsers.sbp.read(b);
            if found > 0 {
                parsers.rtcm3.reset_parser();
                parsers.ubx.reset_parser();
                parsers.nmea.reset_parser();
                *issbp = true;
                let length = parsers.sbp.length();
                shared.send_data(parsers.sbp.packet().get(..length).unwrap_or_default());
                shared.counts().bpsusefull += length as u64;
                seen(shared, format!("Sbp{found:04X}"));
            }
            // ubx
            let found = parsers.ubx.read(b)?;
            if found > 0 {
                parsers.rtcm3.reset_parser();
                parsers.sbp.reset_parser();
                parsers.nmea.reset_parser();
                process_ubx_message(&parsers.ubx, &mut parsers.ubx_state, shared, port);
                seen(shared, format!("Ubx{found:04X}"));
            }
            // nmea
            if parsers.nmea.read(b) > 0 {
                parsers.rtcm3.reset_parser();
                parsers.sbp.reset_parser();
                parsers.ubx.reset_parser();
                seen(shared, "NMEA".to_owned());
            }
            // can_rtcm: DroneCAN's SLCAN reader is not ported.
        }
        Ok(())
    }

    /// `ExtractBasePos`: a 1005 or 1006's station, to latitude, longitude and height - into
    /// `cs.Base` and the RTCM Base line.
    /// `// C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.cs:1062-1113`
    fn extract_base_pos(seen: i32, packet: &[u8], shared: &Shared) {
        if seen != 1005 && seen != 1006 {
            return;
        }
        let llh = rtcm3::ecef2pos(rtcm3::station_ecef(packet));
        let position = LatLngAlt {
            lat: llh[0] * rtcm3::R2D,
            lng: llh[1] * rtcm3::R2D,
            alt: llh[2],
        };
        shared.set_base(position);
        shared.counts().status_line3 = Some(format!(
            "{} {} {} - {}",
            mp_params::param_file::invariant_double(position.lat),
            mp_params::param_file::invariant_double(position.lng),
            mp_params::param_file::invariant_double(position.alt),
            chrono::Local::now().format("%H:%M:%S")
        ));
    }

    /// Every 30 s, TMODE3 and MON-VER are asked for.
    /// `// C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.cs:971-977`
    const POLL_TMODE: Duration = Duration::from_secs(30);

    /// `ProcessUBXMessage`: the survey in, the position, TMODE3; anything unasked-for turned off.
    /// `// C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.cs:877-983`
    fn process_ubx_message(
        parser: &Ubx,
        state: &mut UbxState,
        shared: &Shared,
        port: &Arc<Mutex<Port>>,
    ) {
        let mut handle = Handle {
            port: Arc::clone(port),
            baud: 0,
            pending: Vec::new(),
        };
        let packet = parser.packet();
        match (parser.class(), parser.subclass()) {
            // NAV-SVIN: "survey in"
            (0x1, 0x3b) => {
                let svin = ubx::NavSvin::of(packet);
                state.svin = svin;
                shared.push(Event::Svin(Svin {
                    valid: svin.valid == 1,
                    active: svin.active == 1,
                    dur: svin.dur,
                    obs: svin.obs,
                    acc: f64::from(svin.mean_acc) / 10000.0,
                    ecef: svin.ecef(),
                    mode: state.mode,
                }));
            }
            // NAV-PVT: a 3D fix, the fix OK flag set.
            (0x1, 0x7) => {
                let pvt = ubx::NavPvt::of(packet);
                if pvt.fix_type >= 0x3 && pvt.flags & 1 > 0 {
                    shared.set_base(LatLngAlt {
                        lat: f64::from(pvt.lat) / 1e7,
                        lng: f64::from(pvt.lon) / 1e7,
                        alt: f64::from(pvt.height) / 1000.0,
                    });
                }
            }
            // ACK and NAK, MON-VER, MON-HW, NAV-VELNED: logged, or kept for nothing on the page.
            (0x5, 0x1 | 0x0) | (0xa, 0x4 | 0x9) | (0x1, 0x12) => {}
            // RTCM, RXM
            (0xf5 | 0x02, _) => {}
            // CFG-TMODE3
            (0x06, 0x71) => state.mode = ubx::Tmode3::of(packet),
            (class, subclass) => {
                let _ = ubx::turnon_off(&mut handle, class, subclass, 0);
            }
        }
        if state.poll_tmode.is_none_or(|next| next < Instant::now()) {
            let _ = ubx::poll_msg(&mut handle, 0x06, 0x71);
            state.poll_tmode = Some(Instant::now() + POLL_TMODE);
            let _ = ubx::poll_msg(&mut handle, 0x0a, 0x4);
        }
    }

    /// Feeds bytes through the parsers as the read loop does, 180 at a time, what the parsers
    /// write back going to `port`: for the tests, which read a recorded stream this way.
    #[cfg(test)]
    pub fn feed(shared: &Shared, bytes: &[u8], port: Port) -> (bool, bool) {
        let port = Arc::new(Mutex::new(port));
        let (mut isrtcm, mut issbp) = (false, false);
        for chunk in bytes.chunks(180) {
            if !(isrtcm || issbp) {
                shared.send_data(chunk);
            }
            let _ = process(chunk, shared, &port, &mut isrtcm, &mut issbp, false);
        }
        (isrtcm, issbp)
    }
}

// ---------------------------------------------------------------------------------------------
// The page: `ConfigSerialInjectGPS`'s controls, as its Designer and `.resx` place them.
// ---------------------------------------------------------------------------------------------

/// The page's title in Initial Setup's list.
/// `// C#: GCSViews/InitialSetup.cs:251`
pub const TITLE: &str = "RTK/GPS Inject";

/// `$this.Size`.
/// `// C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.resx ($this.Size)`
pub const PAGE_SIZE: (f32, f32) = (754.0, 594.0);

/// A control's `Location` and `Size`.
pub type Place = (f32, f32, f32, f32);

/// A control of the page: its Designer name, the control it is in, its place there, and its
/// `Text`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Control {
    /// The Designer's name.
    pub name: &'static str,
    /// The control it is added to; "" for the page.
    pub parent: &'static str,
    /// `Location` and `Size`.
    pub place: Place,
    /// `Text`, "" for none.
    pub text: &'static str,
}

const fn control(
    name: &'static str,
    parent: &'static str,
    place: Place,
    text: &'static str,
) -> Control {
    Control {
        name,
        parent,
        place,
        text,
    }
}

/// `splitContainer1.Panel2`, whose place depends on whether `Panel1` is collapsed.
const PANEL2: &str = "splitContainer1.Panel2";

/// Every control with a place, as the `.resx` places it.
/// `// C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.Designer.cs:29-783; ConfigSerialInjectGPS.resx`
#[rustfmt::skip]
pub const CONTROLS: &[Control] = &[
    control("CMB_serialport", "", (3.0, 3.0, 121.0, 21.0), ""),
    control("BUT_connect", "", (130.0, 3.0, 75.0, 23.0), "Connect"),
    control("CMB_baudrate", "", (3.0, 30.0, 121.0, 21.0), ""),
    control("chk_rtcmmsg", "", (3.0, 57.0, 106.0, 17.0), "Inject MSG Type"),
    control("chk_sendgga", "", (3.0, 80.0, 182.0, 17.0), "Send NTRIP GGA? (VRS/Smart)"),
    control("check_sendntripv1", "", (3.0, 106.0, 161.0, 17.0), "Send NTRIP protocol v1.0 ?"),
    control("chk_autoconfig", "", (3.0, 132.0, 182.0, 17.0), "Automatically Configure Receiver"),
    control("comboBoxConfigType", "", (3.0, 155.0, 202.0, 21.0), ""),
    control("groupBox3", "", (212.0, 3.0, 539.0, 64.0), "Link Status"),
    control("label3", "groupBox3", (6.0, 13.0, 76.0, 13.0), "Input data rate"),
    control("lbl_status1", "groupBox3", (93.0, 13.0, 51.0, 13.0), "9999 bps"),
    control("label4", "groupBox3", (165.0, 13.0, 84.0, 13.0), "Output data rate"),
    control("lbl_status2", "groupBox3", (255.0, 13.0, 13.0, 13.0), "2"),
    control("label6", "groupBox3", (6.0, 31.0, 83.0, 13.0), "Messages Seen"),
    control("labelmsgseen", "groupBox3", (93.0, 30.0, 446.0, 31.0), "rtcm0000\nrtcm0000"),
    control("groupBox2", "", (212.0, 73.0, 348.0, 60.0), "RTCM"),
    control("label11", "groupBox2", (6.0, 20.0, 31.0, 13.0), "Base"),
    control("labelbase", "groupBox2", (43.0, 16.0, 20.0, 20.0), ""),
    control("label12", "groupBox2", (69.0, 20.0, 26.0, 13.0), "Gps"),
    control("labelgps", "groupBox2", (101.0, 16.0, 20.0, 20.0), ""),
    control("label13", "groupBox2", (127.0, 20.0, 45.0, 13.0), "Glonass"),
    control("labelglonass", "groupBox2", (178.0, 16.0, 20.0, 20.0), ""),
    control("label15", "groupBox2", (204.0, 20.0, 40.0, 13.0), "Beidou"),
    control("label14BDS", "groupBox2", (250.0, 16.0, 20.0, 20.0), ""),
    control("label16", "groupBox2", (278.0, 20.0, 39.0, 13.0), "Galileo"),
    control("labelGall", "groupBox2", (324.0, 16.0, 20.0, 20.0), ""),
    control("label5", "groupBox2", (6.0, 40.0, 68.0, 13.0), "RTCM Base "),
    control("lbl_status3", "groupBox2", (93.0, 40.0, 241.0, 13.0), "-00.000000000 000.000000000 0000.000000000"),
    control("myGMAP1", "", (562.0, 67.0, 183.0, 92.0), ""),
    control("splitContainer1", "", (3.0, 182.0, 748.0, 409.0), ""),
    control("groupBox_autoconfig", "splitContainer1", (0.0, 0.0, 748.0, 185.0), "Automatic Config Options"),
    control("panel_ubloxoptions", "groupBox_autoconfig", (3.0, 16.0, 742.0, 166.0), ""),
    control("panel2", "panel_ubloxoptions", (0.0, 0.0, 543.0, 166.0), ""),
    control("chk_m8p_130p", "panel2", (3.0, 3.0, 113.0, 17.0), "M8P fw 130+/F9P"),
    control("label1", "panel2", (3.0, 29.0, 85.0, 13.0), "SurveyIn Acc(m)"),
    control("txt_surveyinAcc", "panel2", (94.0, 25.0, 37.0, 20.0), "2.00"),
    control("label2", "panel2", (137.0, 29.0, 41.0, 13.0), "Time(s)"),
    control("txt_surveyinDur", "panel2", (184.0, 25.0, 37.0, 20.0), "60"),
    control("but_restartsvin", "panel2", (227.0, 22.0, 75.0, 23.0), "Restart"),
    control("but_save_basepos", "panel2", (429.0, 22.0, 75.0, 23.0), "Save Current Position"),
    control("dg_basepos", "panel2", (3.0, 51.0, 537.0, 112.0), ""),
    control("groupBox1", "panel_ubloxoptions", (546.0, 0.0, 196.0, 166.0), "Survey In"),
    control("lbl_svin", "groupBox1", (6.0, 16.0, 184.0, 13.0), "Survey In"),
    control("label7", "groupBox1", (6.0, 29.0, 184.0, 13.0), "Survey In"),
    control("label8", "groupBox1", (6.0, 42.0, 184.0, 13.0), "Survey In"),
    control("label9", "groupBox1", (6.0, 55.0, 184.0, 13.0), "Survey In"),
    control("label10", "groupBox1", (6.0, 68.0, 184.0, 13.0), "Survey In"),
    control("panel_septentrio", "groupBox_autoconfig", (3.0, 16.0, 742.0, 166.0), ""),
    control("label14", "panel_septentrio", (97.0, 0.0, 87.0, 13.0), "Atitude (WGS84)"),
    control("label17", "panel_septentrio", (203.0, 0.0, 101.0, 13.0), "Longitude (WGS84)"),
    control("label18", "panel_septentrio", (309.0, 0.0, 59.0, 13.0), "Altitude (m)"),
    control("chk_septentriofixedposition", "panel_septentrio", (0.0, 18.0, 91.0, 17.0), "Fixed Position"),
    control("input_septentriofixedatitude", "panel_septentrio", (100.0, 16.0, 100.0, 20.0), ""),
    control("input_septentriofixedlongitude", "panel_septentrio", (206.0, 16.0, 100.0, 20.0), ""),
    control("input_septentriofixedaltitude", "panel_septentrio", (312.0, 16.0, 100.0, 20.0), ""),
    control("button_septentriosetposition", "panel_septentrio", (418.0, 16.0, 75.0, 20.0), "Set Position"),
    control("label19", "panel_septentrio", (0.0, 54.0, 123.0, 13.0), "RTCM Message Amount"),
    control("cmb_septentriortcmamount", "panel_septentrio", (140.0, 51.0, 166.0, 21.0), ""),
    control("label20", "panel_septentrio", (0.0, 90.0, 136.0, 13.0), "RTCM Message Interval (s)"),
    control("input_septentriortcminterval", "panel_septentrio", (140.0, 87.0, 166.0, 20.0), ""),
    control("button_septentriortcminterval", "panel_septentrio", (312.0, 87.0, 75.0, 20.0), "Set Interval"),
    control("label21", "panel_septentrio", (0.0, 124.0, 135.0, 13.0), "RTCM Constellation Usage"),
    control("chk_septentriogps", "panel_septentrio", (141.0, 123.0, 48.0, 17.0), "GPS"),
    control("chk_septentrioglonass", "panel_septentrio", (195.0, 123.0, 77.0, 17.0), "GLONASS"),
    control("chk_septentriobeidou", "panel_septentrio", (279.0, 123.0, 61.0, 17.0), "BeiDou"),
    control("chk_septentriogalileo", "panel_septentrio", (347.0, 123.0, 58.0, 17.0), "Galileo"),
    control("panel_um982", "groupBox_autoconfig", (3.0, 16.0, 742.0, 166.0), ""),
    control("label22", "panel_um982", (28.0, 22.0, 415.0, 65.0), UM982_TEXT),
    control("panel1", PANEL2, (3.0, 4.0, 740.0, 186.0), ""),
];

/// `label22.Text`, the Unicore panel's one label.
/// `// C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.resx (label22.Text)`
pub const UM982_TEXT: &str = "At connect the receiver  will start a \u{201c}survey-in\u{201d} to determine the GPS precision location.\nThe survey-in is finished when the RTCM Base location is no longer changing.\nBased on the location and signal it could take a couple minutes\n\nOnce the survey is completed, you are ready to go.\n";

/// `splitContainer1.SplitterDistance`, and the splitter's own width.
/// `// C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.resx (splitContainer1.SplitterDistance)`
const SPLITTER_DISTANCE: f32 = 185.0;
const SPLITTER_WIDTH: f32 = 4.0;

/// `panel1.MaximumSize`'s height: with `Panel1` collapsed its anchors would stretch it to 375.
/// `// C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.resx (panel1.MaximumSize)`
const PANEL1_MAX_HEIGHT: f32 = 200.0;

/// A control by name.
#[must_use]
pub fn control_named(name: &str) -> Option<&'static Control> {
    CONTROLS.iter().find(|control| control.name == name)
}

/// Where a control is on the page, its parents' places added in: with `Panel1` collapsed,
/// `Panel2` is the whole split container and `panel1`, anchored on all four sides, grows to its
/// maximum height.
#[must_use]
pub fn absolute(name: &str, collapsed: bool) -> Place {
    let Some(control) = control_named(name) else {
        return (0.0, 0.0, 0.0, 0.0);
    };
    let (mut x, mut y, width, mut height) = control.place;
    if name == "panel1" && collapsed {
        height = PANEL1_MAX_HEIGHT;
    }
    let mut parent = control.parent;
    while !parent.is_empty() {
        if parent == PANEL2 {
            if !collapsed {
                y += SPLITTER_DISTANCE + SPLITTER_WIDTH;
            }
            parent = "splitContainer1";
            continue;
        }
        let Some(up) = control_named(parent) else {
            break;
        };
        x += up.place.0;
        y += up.place.1;
        parent = up.parent;
    }
    (x, y, width, height)
}

/// A control's `Text`.
fn text_of(name: &str) -> &'static str {
    control_named(name).map_or("", |control| control.text)
}

/// `CMB_serialport`'s items after the serial ports.
/// `// C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.cs:90-93`
pub const PORT_EXTRAS: [&str; 4] = ["UDP Host", "UDP Client", "TCP Client", "NTRIP"];

/// `CMB_baudrate`'s items.
/// `// C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.resx (CMB_baudrate.Items to Items12)`
pub const BAUDS: [&str; 13] = [
    "2400", "4800", "9600", "14400", "19200", "28800", "38400", "57600", "115200", "230400",
    "460800", "500000", "1000000",
];

/// `comboBoxConfigType`'s items.
/// `// C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.resx (comboBoxConfigType.Items to Items2)`
pub const CONFIG_TYPES: [&str; 3] = ["UBlox M8P/F9P", "Septentrio", "Unicore UM982"];

/// `cmb_septentriortcmamount`'s items.
/// `// C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.resx (cmb_septentriortcmamount.Items to Items2)`
pub const AMOUNTS: [&str; 3] = ["Lite", "Basic", "Full"];

/// `dg_basepos`'s columns: name, `HeaderText`, `Width` (100 where the `.resx` sets none).
/// `// C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.Designer.cs:698-743; ConfigSerialInjectGPS.resx`
pub const COLUMNS: [(&str, &str, f32); 6] = [
    ("Lat", "Lat/ECEFX", 100.0),
    ("Long", "Long/ECEFY", 100.0),
    ("Alt", "Alt/ECEFZ", 77.0),
    ("BaseName1", "Name", 100.0),
    ("Use", "Use", 40.0),
    ("Delete", "Delete", 40.0),
];

/// A `DataGridView`'s row header, column header and row heights.
const GRID_ROW_HEADER: f32 = 41.0;
const GRID_HEADER: f32 = 23.0;
const GRID_ROW: f32 = 22.0;

/// The four indicator lights of `groupBox2`, and what the facts call them.
pub const LIGHTS: [(&str, &str); 5] = [
    ("labelbase", "base"),
    ("labelgps", "gps"),
    ("labelglonass", "glonass"),
    ("label14BDS", "beidou"),
    ("labelGall", "galileo"),
];

/// `Strings.Connect` and `Strings.Stop`.
/// `// C#: ExtLibs/Strings/Strings.resx:450-455`
pub const CONNECT: &str = "Connect";
/// The button's text while connected.
pub const STOP: &str = "Stop";

/// `Strings.InvalidPortName` and `Strings.InvalidBaudRate`.
/// `// C#: ExtLibs/Strings/Strings.resx:177-178, 193-194`
pub const INVALID_PORT_NAME: &str = "Invalid PortName";
/// A baud rate that does not parse.
pub const INVALID_BAUD_RATE: &str = "Invalid BaudRate";

/// What `Open`'s cancelled question throws, as `ex.ToString()` begins.
/// `// C#: ExtLibs/Comms/CommsNTRIP.cs:108-110; CommsTCPSerial.cs:125-129; CommsUDPSerialConnect.cs:142-146`
pub const CANCELED: &str = "System.Exception: Canceled by request";

/// `DoConnect`'s box for a port that did not open.
/// `// C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.cs:449-450`
pub fn error_connecting(why: &str) -> String {
    format!("Error Connecting\nif using com0com please rename the ports to COM??\n{why}")
}

/// `closed port`: what a handler reading `comPort.BaseStream` with the port closed throws.
const PORT_CLOSED: &str = "The port is closed.";

/// Why DroneCAN's part of the read loop is not here.
pub const CAN_DIMMED: &str = "DroneCAN over SLCAN (ExtLibs/DroneCAN: ReadSLCAN and the \
                              uavcan.equipment.gnss.RTCMStream it carries) is not ported";

/// `ExpireType.Set(label, seconds)` for the lights: the base's messages for 20 s, the rest 5.
/// `// C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.cs:807-875`
#[must_use]
pub fn light_for(seen: i32) -> Option<(usize, u64)> {
    Some(match seen {
        1001..=1004 | 1071..=1077 => (1, 5),
        1005 | 1006 | 4072 => (0, 20),
        1009..=1012 | 1081..=1087 => (2, 5),
        1091..=1097 => (4, 5),
        1121..=1127 => (3, 5),
        _ => return None,
    })
}

/// One of `groupBox2`'s lights: green once its messages are seen, red again once
/// `ExpireType` says they have stopped.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Light {
    /// `BackColor == Color.Green`.
    pub green: bool,
    /// When `ExpireType.HasExpired` turns true; never set is expired.
    expires: Option<Instant>,
}

/// A base position, as `PointLatLngAlt` holds one with its `Tag`.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct BasePos {
    /// `Lat`.
    pub lat: f64,
    /// `Lng`.
    pub lng: f64,
    /// `Alt`.
    pub alt: f64,
    /// `Tag`: the name.
    pub tag: String,
}

impl BasePos {
    /// `== PointLatLngAlt.Zero`: (0, 0, 0), no tag.
    #[must_use]
    pub fn is_zero(&self) -> bool {
        self.lat == 0.0 && self.lng == 0.0 && self.alt == 0.0 && self.tag.is_empty()
    }

    /// As `SetupBasePos` is given it.
    #[must_use]
    pub fn position(&self) -> ubx::Position {
        ubx::Position {
            lat: self.lat,
            lng: self.lng,
            alt: self.alt,
            zero: self.is_zero(),
        }
    }
}

/// `double.ToString(CultureInfo.InvariantCulture)`.
fn invariant(value: f64) -> String {
    mp_params::param_file::invariant_double(value)
}

/// `double.Parse`, forgiving of the white space .NET forgives.
fn parse_double(text: &str) -> Option<f64> {
    text.trim().parse::<f64>().ok()
}

/// `loadBasePOS`: `base_pos` is `lat,lng,alt,name`; anything else is `Zero`.
/// `// C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.cs:1234-1258`
#[must_use]
pub fn load_base_pos(setting: Option<&str>) -> BasePos {
    let Some(setting) = setting else {
        return BasePos::default();
    };
    let parts: Vec<&str> = setting.split(',').collect();
    let number = |at: usize| parts.get(at).and_then(|text| parse_double(text));
    match (number(0), number(1), number(2), parts.get(3)) {
        (Some(lat), Some(lng), Some(alt), Some(tag)) => BasePos {
            lat,
            lng,
            alt,
            tag: (*tag).to_owned(),
        },
        _ => BasePos::default(),
    }
}

/// `String.Format("{0},{1},{2},{3}", ...)` of a base position, as `base_pos` holds it.
/// `// C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.cs:1273-1274`
#[must_use]
pub fn base_pos_setting(lat: f64, lng: f64, alt: f64, name: &str) -> String {
    format!(
        "{},{},{},{name}",
        invariant(lat),
        invariant(lng),
        invariant(alt)
    )
}

/// `baseposlist.xml`: where it is.
/// `// C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.cs:67-68`
fn basepos_list_file() -> Option<std::path::PathBuf> {
    mp_settings::user_data_directory().map(|dir| dir.join("baseposlist.xml"))
}

/// Escapes text for an element, as `XmlWriter` does.
fn xml_escape(text: &str) -> String {
    text.replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
}

/// `XmlSerializer(typeof(List<PointLatLngAlt>))`'s document for the list: each point's public
/// read-write properties in declaration order, a double as `XmlConvert` writes it (the shortest
/// text that reads back), an empty string and `Color` - which has no settable property - as
/// empty elements.
/// `// C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.cs:284-294; ExtLibs/Utilities/PointLatLngAlt.cs:21-29`
#[must_use]
pub fn basepos_list_xml(list: &[BasePos]) -> String {
    let mut out = String::from(
        "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n<ArrayOfPointLatLngAlt xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xmlns:xsd=\"http://www.w3.org/2001/XMLSchema\">",
    );
    for point in list {
        out.push_str("\n  <PointLatLngAlt>");
        out.push_str(&format!("\n    <Lat>{}</Lat>", point.lat));
        out.push_str(&format!("\n    <Lng>{}</Lng>", point.lng));
        out.push_str(&format!("\n    <Alt>{}</Alt>", point.alt));
        if point.tag.is_empty() {
            out.push_str("\n    <Tag />");
        } else {
            out.push_str(&format!("\n    <Tag>{}</Tag>", xml_escape(&point.tag)));
        }
        out.push_str("\n    <Tag2 />\n    <color />\n  </PointLatLngAlt>");
    }
    out.push_str("\n</ArrayOfPointLatLngAlt>");
    out
}

/// The list back from its document: each `PointLatLngAlt`'s `Lat`, `Lng`, `Alt` and `Tag`.
/// `// C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.cs:259-282`
#[must_use]
pub fn basepos_list_from(xml: &str) -> Option<Vec<BasePos>> {
    let unescape = |text: &str| {
        text.replace("&lt;", "<")
            .replace("&gt;", ">")
            .replace("&amp;", "&")
    };
    let field = |item: &str, name: &str| -> Option<String> {
        let open = format!("<{name}>");
        let start = item.find(&open)? + open.len();
        let end = item.get(start..)?.find(&format!("</{name}>"))? + start;
        item.get(start..end).map(unescape)
    };
    if !xml.contains("<ArrayOfPointLatLngAlt") {
        return None;
    }
    let mut list = Vec::new();
    for item in xml.split("<PointLatLngAlt>").skip(1) {
        let number = |name: &str| field(item, name).and_then(|text| parse_double(&text));
        list.push(BasePos {
            lat: number("Lat").unwrap_or(0.0),
            lng: number("Lng").unwrap_or(0.0),
            alt: number("Alt").unwrap_or(0.0),
            tag: field(item, "Tag").unwrap_or_default(),
        });
    }
    Some(list)
}

/// One `VerticalProgressBar2` of `panel1`: a satellite's signal strength.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Bar {
    /// `Label`: the system's letter, then the satellite's number once it has had one.
    pub label: String,
    /// `Value`.
    pub value: i32,
    /// `Location.X` and `Width`, as the last message for its system left them.
    pub x: i32,
    /// `Width`.
    pub width: i32,
}

/// `VerticalProgressBar2`'s `Minimum`, `Maximum` and `minline`.
/// `// C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.cs:240-243`
const BAR_RANGE: (i32, i32) = (25, 55);
const BAR_MINLINE: i32 = 40;

/// `Rtcm3_ObsMessage`: the bars of the message's system made up to one per satellite, zeroed,
/// and set - each at its place after the systems before it, in the order GPS, GLONASS, BeiDou,
/// Galileo, QZSS, each as wide as the panel shared among every bar there is.
/// `// C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.cs:167-252`
pub fn obs_message(bars: &mut Vec<Bar>, obs: &[rtcm3::Ob], panel_width: i32) {
    let Some(first) = obs.first() else {
        return;
    };
    let sys = first.sys;
    let of = |bar: &Bar, sys: char| bar.label.starts_with(sys);
    while bars.iter().filter(|bar| of(bar, sys)).count() < obs.len() {
        bars.push(Bar {
            label: sys.to_string(),
            value: 0,
            x: 0,
            width: 0,
        });
    }
    for bar in bars.iter_mut().filter(|bar| of(bar, sys)) {
        bar.value = 0;
    }
    #[allow(clippy::cast_possible_truncation, clippy::cast_possible_wrap)]
    let total = bars.len() as i32;
    let width = panel_width / total.max(1);
    #[allow(clippy::cast_possible_truncation, clippy::cast_possible_wrap)]
    let count = |sys: char| bars.iter().filter(|bar| of(bar, sys)).count() as i32;
    let start = match sys {
        'R' => count('G'),
        'B' => count('G') + count('R'),
        'E' => count('G') + count('R') + count('B'),
        'Q' => count('G') + count('R') + count('B') + count('E'),
        _ => 0,
    };
    for (cnt, (bar, ob)) in bars
        .iter_mut()
        .filter(|bar| of(bar, sys))
        .zip(obs)
        .enumerate()
    {
        bar.value = i32::from(ob.snr);
        bar.label = format!("{}{}", ob.sys, ob.prn);
        #[allow(clippy::cast_possible_truncation, clippy::cast_possible_wrap)]
        let cnt = cnt as i32;
        bar.x = width * (start + cnt);
        bar.width = width;
    }
}

/// The survey-in group's labels: `lbl_svin` and `label7` to `label10`.
#[derive(Debug, Clone, PartialEq)]
pub struct SvinLabels {
    /// Each one's `Visible`.
    pub visible: [bool; 5],
    /// Each one's `Text`.
    pub text: [String; 5],
    /// `lbl_svin.BackColor`: green, red, or the default.
    pub valid: Option<bool>,
}

impl Default for SvinLabels {
    /// As the `.resx` leaves them: all hidden, all "Survey In".
    fn default() -> Self {
        Self {
            visible: [false; 5],
            text: std::array::from_fn(|_| "Survey In".to_owned()),
            valid: None,
        }
    }
}

/// `double` in string concatenation: its `ToString()`.
fn concat(value: f64) -> String {
    invariant(value)
}

impl SvinLabels {
    /// `updateSVINLabel(valid, active, dur, obs, acc)`: with no base position chosen, the survey
    /// in's state and then its position; with one, the receiver's TMODE3.
    /// `// C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.cs:991-1060`
    pub fn update(&mut self, basepos_zero: bool, svin: &Svin) {
        let [svin_label, l7, l8, l9, l10] = &mut self.text;
        if basepos_zero {
            self.visible = [true; 5];
            *svin_label = if svin.valid {
                "Postion is valid"
            } else {
                "Position is invalid"
            }
            .to_owned();
            self.valid = Some(svin.valid);
            if svin.valid {
                let llh = rtcm3::ecef2pos(svin.ecef);
                *l7 = format!("Lat/X: {}", concat(llh[0] * rtcm3::R2D));
                *l8 = format!("Lng/Y: {}", concat(llh[1] * rtcm3::R2D));
                *l9 = format!("Alt/Z: {}", concat(llh[2]));
            } else {
                *l7 = if svin.active {
                    "In Progress"
                } else {
                    "Complete"
                }
                .to_owned();
                *l8 = format!("Duration: {}", svin.dur);
                *l9 = format!("Observations: {}", svin.obs);
            }
            *l10 = format!("Current Acc: {}", concat(svin.acc));
        } else {
            self.visible = [true, false, false, false, false];
            *svin_label = format!("Using {}", svin.mode.mode_name());
            self.valid = Some(true);
            if let Some([x, y, z]) = svin.mode.point() {
                *l7 = format!("Lat/X: {}", concat(x));
                *l8 = format!("Lng/Y: {}", concat(y));
                *l9 = format!("Alt/Z: {}", concat(z));
                self.visible = [true, true, true, true, false];
            }
        }
    }
}

/// Which text box is being typed into.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Editing {
    /// `txt_surveyinAcc`.
    SurveyAcc,
    /// `txt_surveyinDur`.
    SurveyDur,
    /// One of the Septentrio position boxes.
    Position(usize),
    /// `input_septentriortcminterval`.
    Interval,
    /// A cell of `dg_basepos`: its row - the grid's length for the new row - and column.
    Cell(usize, usize),
}

/// Which combo box's list is down.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Dropdown {
    /// `CMB_serialport`.
    Port,
    /// `CMB_baudrate`.
    Baud,
    /// `comboBoxConfigType`.
    ConfigType,
    /// `cmb_septentriortcmamount`.
    Amount,
}

/// A question `Open` or a handler asks: `InputBox.Show(title, prompt, ref value)`.
#[derive(Debug)]
pub struct Prompt {
    /// The caption.
    pub title: &'static str,
    /// The question.
    pub prompt: &'static str,
    /// The answer box.
    pub field: TextField,
    /// What the answer is for.
    pub purpose: Purpose,
}

/// What a question's answer is for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Purpose {
    /// `CommsNTRIP.Open`'s URL.
    NtripUrl,
    /// `TcpSerial.Open`'s host, then its port.
    TcpHost,
    /// The port.
    TcpPort,
    /// `UdpSerial.Open`'s local port.
    UdpPort,
    /// `UdpSerialConnect.Open`'s host, then its port.
    UdpClientHost,
    /// The port.
    UdpClientPort,
    /// `but_save_basepos_Click`'s name for the position.
    Location,
}

/// The page object: its controls' state, as the constructor and the handlers leave it.
pub struct Page {
    /// The screen the page object belongs to.
    made_for: Key,
    /// `CMB_serialport`.
    pub ports: Combo,
    /// `CMB_baudrate`; `Enabled` is `ports`' serial-ness.
    pub baud: Combo,
    /// `chk_rtcmmsg.Checked`.
    pub rtcmmsg: bool,
    /// `chk_sendgga.Checked` and `Enabled`.
    pub sendgga: (bool, bool),
    /// `check_sendntripv1.Checked`.
    pub ntrip_v1: bool,
    /// `chk_autoconfig.Checked`: shows `Panel1` and `comboBoxConfigType`.
    pub autoconfig: bool,
    /// `comboBoxConfigType`.
    pub config_type: Combo,
    /// `panel_ubloxoptions`, `panel_septentrio` and `panel_um982`'s `Visible`.
    pub panels: [bool; 3],
    /// `chk_m8p_130p.Checked`.
    pub m8p_130p: bool,
    /// `txt_surveyinAcc`.
    pub survey_acc: TextField,
    /// `txt_surveyinDur`.
    pub survey_dur: TextField,
    /// `chk_septentriofixedposition.Checked`.
    pub fixed_position: bool,
    /// The three position boxes.
    pub position: [TextField; 3],
    /// `button_septentriosetposition.Enabled`.
    pub set_position_enabled: bool,
    /// `cmb_septentriortcmamount`.
    pub amount: Combo,
    /// `input_septentriortcminterval`.
    pub interval: TextField,
    /// The four constellation boxes.
    pub signals: septentrio::Signals,
    /// `lbl_status1`, `lbl_status2`, `lbl_status3` and `labelmsgseen`.
    pub status: [String; 4],
    /// The five lights.
    pub lights: [Light; 5],
    /// The survey-in group.
    pub svin: SvinLabels,
    /// `basepos`.
    pub basepos: BasePos,
    /// `baseposList`.
    pub list: Vec<BasePos>,
    /// `dg_basepos`'s rows: the four text cells of each.
    pub grid: Vec<[String; 4]>,
    /// `panel1`'s bars.
    pub bars: Vec<Bar>,
    /// `BUT_connect.Text` and `Enabled`.
    pub connect: (&'static str, bool),
    /// The box being typed into.
    pub editing: Option<Editing>,
    /// The text a grid cell held when its edit began.
    edit_was: String,
    /// The list that is down.
    pub dropdown: Option<Dropdown>,
    /// When `timer1` last ticked.
    last_tick: Option<Instant>,
    /// `myGMAP1`.
    map: Rc<RefCell<MapViewport>>,
    /// Its one marker, `base`'s.
    pub marker: Option<LatLon>,
    /// `basepostlistfile`.
    list_file: Option<std::path::PathBuf>,
}

/// A combo box of fixed items, keyed by index.
fn combo_of(items: &[&str]) -> Combo {
    let mut combo = Combo::default();
    #[allow(clippy::cast_possible_wrap)]
    {
        combo.options = items
            .iter()
            .enumerate()
            .map(|(index, item)| (index as i64, (*item).to_owned()))
            .collect();
    }
    combo.selected = None;
    combo.enabled = true;
    combo
}

/// `ComboBox.Text = text` on a drop-down list: the item that says it, or nothing. Whether the
/// selection changed, when `SelectedIndexChanged` is raised.
fn select_text(combo: &mut Combo, text: &str) -> bool {
    let key = combo
        .options
        .iter()
        .find(|(_, item)| item == text)
        .map(|(key, _)| *key);
    match key {
        Some(key) => combo.select(key),
        None => false,
    }
}

/// A text box holding `text`.
fn field(text: &str) -> TextField {
    let mut field = TextField::new("");
    field.set(text);
    field
}

/// `bool.Parse` of a saved check box.
fn saved_bool(persisted: &Persisted, key: &str) -> Option<bool> {
    let text = persisted.get(key)?.trim();
    if text.eq_ignore_ascii_case("true") {
        Some(true)
    } else if text.eq_ignore_ascii_case("false") {
        Some(false)
    } else {
        None
    }
}

/// `String.Format("{0,10} bps", n)`.
fn bps_text(bps: u64, sent: bool) -> String {
    format!("{bps:>10} bps{}", if sent { " sent" } else { "" })
}

/// The whole application's part of the page: its statics and its page object.
pub struct RtkInject {
    /// What the page and its thread share.
    pub shared: Arc<Shared>,
    /// The port and thread `DoConnect` started.
    pub worker: Option<worker::Worker>,
    /// The page object, while its screen shows.
    pub page: Option<Page>,
    /// Whether the page is showing.
    active: bool,
    /// `DoConnect` waiting on `Open`'s questions.
    pending: Option<worker::OpenSpec>,
    /// The question showing.
    pub prompt: Option<Prompt>,
    /// Message boxes, the first showing.
    pub messages: VecDeque<Message>,
    /// `basepostlistfile`: `baseposlist.xml` in the user data directory.
    pub list_file: Option<std::path::PathBuf>,
}

impl Default for RtkInject {
    fn default() -> Self {
        Self {
            shared: Arc::new(Shared::default()),
            worker: None,
            page: None,
            active: false,
            pending: None,
            prompt: None,
            messages: VecDeque::new(),
            list_file: basepos_list_file(),
        }
    }
}

/// A box with no caption: `CustomMessageBox.Show(text)`.
fn plain(text: impl Into<String>) -> Message {
    crate::config::optional::plain(text)
}

impl RtkInject {
    /// Whether the page is showing.
    #[must_use]
    pub const fn is_active(&self) -> bool {
        self.active
    }

    /// `comPort.IsOpen`.
    #[must_use]
    pub fn is_open(&self) -> bool {
        self.worker.as_ref().is_some_and(worker::Worker::is_open)
    }

    /// `threadrun`.
    #[must_use]
    pub fn is_running(&self) -> bool {
        self.worker.as_ref().is_some_and(worker::Worker::is_running)
    }

    /// The constructor: the controls from `Settings.Instance`, each restored as its handler sees
    /// it, the base positions from `baseposlist.xml` and `base_pos`.
    /// `// C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.cs:81-165`
    fn construct(&mut self, key: Key, persisted: &mut Persisted, map_source: Option<&str>) {
        let mut ports = combo_of(&[]);
        #[allow(clippy::cast_possible_wrap)]
        {
            ports.options = mp_transport::list_ports()
                .into_iter()
                .map(|port| port.name)
                .chain(PORT_EXTRAS.iter().map(|&extra| extra.to_owned()))
                .enumerate()
                .map(|(index, name)| (index as i64, name))
                .collect();
        }
        let page = Page {
            made_for: key,
            ports,
            baud: combo_of(&BAUDS),
            rtcmmsg: true,
            sendgga: (true, true),
            ntrip_v1: false,
            autoconfig: false,
            config_type: combo_of(&CONFIG_TYPES),
            // `panel_ubloxoptions` is the one the Designer leaves visible.
            panels: [true, false, false],
            m8p_130p: false,
            survey_acc: field(text_of("txt_surveyinAcc")),
            survey_dur: field(text_of("txt_surveyinDur")),
            fixed_position: false,
            position: [field(""), field(""), field("")],
            set_position_enabled: false,
            amount: combo_of(&AMOUNTS),
            // `input_septentriortcminterval.Value`, 1.0 to one decimal.
            interval: field("1.0"),
            signals: septentrio::Signals {
                gps: true,
                glonass: true,
                beidou: true,
                galileo: true,
            },
            status: [
                text_of("lbl_status1").to_owned(),
                text_of("lbl_status2").to_owned(),
                text_of("lbl_status3").to_owned(),
                text_of("labelmsgseen").to_owned(),
            ],
            lights: [Light::default(); 5],
            svin: SvinLabels::default(),
            basepos: BasePos::default(),
            list: Vec::new(),
            grid: Vec::new(),
            bars: Vec::new(),
            connect: (if self.is_running() { STOP } else { CONNECT }, true),
            editing: None,
            edit_was: String::new(),
            dropdown: None,
            last_tick: None,
            map: Rc::new(RefCell::new(MapViewport::new(0, 0))),
            marker: None,
            list_file: self.list_file.clone(),
        };
        use_imagery(&page.map, map_source);
        self.page = Some(page);

        // "restore last port and baud - its the simple things that make life better"
        if let Some(port) = persisted.get("SerialInjectGPS_port").map(str::to_owned)
            && let Some(page) = self.page.as_mut()
            && select_text(&mut page.ports, &port)
        {
            page.port_changed();
        }
        if let Some(kind) = persisted
            .get("SerialInjectGPS_AutoConfigType")
            .map(str::to_owned)
            && let Some(page) = self.page.as_mut()
            && select_text(&mut page.config_type, &kind)
        {
            page.config_type_changed(persisted);
        }
        let Some(page) = self.page.as_mut() else {
            return;
        };
        for (index, key) in [
            "SerialInjectGPS_SeptentrioFixedAtitude",
            "SerialInjectGPS_SeptentrioFixedLongitude",
            "SerialInjectGPS_SeptentrioFixedAltitude",
        ]
        .into_iter()
        .enumerate()
        {
            if let (Some(text), Some(box_)) = (persisted.get(key), page.position.get_mut(index)) {
                box_.set(text);
            }
        }
        for (key, slot) in [
            ("SerialInjectGPS_SeptentrioGPS", &mut page.signals.gps),
            (
                "SerialInjectGPS_SeptentrioGLONASS",
                &mut page.signals.glonass,
            ),
            (
                "SerialInjectGPS_SeptentrioGalileo",
                &mut page.signals.galileo,
            ),
            ("SerialInjectGPS_SeptentrioBeiDou", &mut page.signals.beidou),
        ] {
            if let Some(value) = saved_bool(persisted, key) {
                *slot = value;
            }
        }
        let baud = persisted
            .get("SerialInjectGPS_baud")
            .unwrap_or("115200")
            .to_owned();
        select_text(&mut page.baud, &baud);
        let level = persisted
            .get("SerialInjectGPS_SeptentrioRTCMLevel")
            .and_then(|text| text.trim().parse::<i64>().ok())
            .unwrap_or(1);
        if page.amount.select(level) {
            // `cmb_septentriortcmamount_SelectedIndexChanged`
            persisted.set("SerialInjectGPS_SeptentrioRTCMLevel", level.to_string());
        }
        if let Some(text) = persisted.get("SerialInjectGPS_SeptentrioRTCMInterval") {
            page.interval.set(text);
        }
        if let Some(text) = persisted.get("SerialInjectGPS_SIAcc") {
            page.survey_acc.set(text);
        }
        if let Some(text) = persisted.get("SerialInjectGPS_SITime") {
            page.survey_dur.set(text);
        }
        if saved_bool(persisted, "SerialInjectGPS_autoconfig") == Some(true) {
            page.autoconfig_changed(true, persisted);
        }
        if let Some(fixed) = saved_bool(persisted, "SerialInjectGPS_SeptentrioFixedPosition") {
            page.fixed_position = fixed;
        }
        if let Some(value) = saved_bool(persisted, "SerialInjectGPS_m8p_130p") {
            page.m8p_130p = value;
        }
        // `loadBasePosList()`: a file that does not parse is the C#'s "Failed to load Base
        // Position List" box.
        if let Some(path) = page.list_file.clone()
            && let Ok(text) = std::fs::read_to_string(&path)
        {
            match basepos_list_from(&text) {
                Some(list) => page.list = list,
                None => self
                    .messages
                    .push_back(crate::config::optional::error(format!(
                        "Failed to load Base Position List\n{}",
                        path.display()
                    ))),
            }
        }
        page.update_grid();
        page.basepos = load_base_pos(persisted.get("base_pos"));
        // The Septentrio level set above, while a Septentrio port is open.
        let septentrio =
            (page.config_type.text() == "Septentrio").then(|| page.septentrio_inputs());
        if let (Some(inputs), Some(worker)) = (septentrio, self.worker.as_ref())
            && worker.is_open()
        {
            worker.command(Command::SeptentrioRtcm(Handler::Amount, inputs));
        }
        // "restore current static state": `chk_rtcmmsg.Checked = rtcm_msg`, and its handler.
        let rtcm_msg = self
            .shared
            .rtcm_msg
            .load(std::sync::atomic::Ordering::Acquire);
        self.rtcmmsg_changed(rtcm_msg);
    }

    /// `Activate`: the page object made if its screen has none, the map on the flight map's
    /// imagery at zoom 16, and the timer started.
    /// `// C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.cs:1214-1222`
    pub fn activate(&mut self, key: Key, persisted: &mut Persisted, map_source: Option<&str>) {
        if self.page.as_ref().is_none_or(|page| page.made_for != key) {
            self.construct(key, persisted, map_source);
        }
        self.active = true;
        if let Some(page) = self.page.as_mut() {
            use_imagery(&page.map, map_source);
            page.last_tick = Some(Instant::now());
            page.dropdown = None;
        }
    }

    /// `Deactivate`: the timer stopped.
    /// `// C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.cs:1224-1227`
    pub fn deactivate(&mut self) {
        self.active = false;
        if let Some(page) = self.page.as_mut() {
            page.dropdown = None;
        }
    }

    /// `BUT_connect_Click`: with the port open, closed; else `DoConnect`.
    /// `// C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.cs:301-324`
    pub fn connect_click(&mut self, persisted: &mut Persisted, planned_home: (f64, f64, f64)) {
        let open = self.is_open();
        // `threadrun = false`, whichever way this goes.
        if let Some(mut worker) = self.worker.take() {
            worker.stop();
        }
        if open {
            if let Some(page) = self.page.as_mut() {
                page.connect.0 = CONNECT;
                page.sendgga.1 = true;
            }
            return;
        }
        self.do_connect(persisted, planned_home);
    }

    /// `DoConnect` up to `comPort.Open()`: the port made for `CMB_serialport`'s choice and its
    /// baud rate read; then `Open`'s questions.
    /// `// C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.cs:326-392`
    fn do_connect(&mut self, persisted: &mut Persisted, planned_home: (f64, f64, f64)) {
        self.shared.counts().status_line3 = None;
        let Some(page) = self.page.as_mut() else {
            return;
        };
        let port_text = page.ports.text().to_owned();
        let kind = Kind::of(&port_text);
        let mut spec = OpenSpec {
            kind,
            name: String::new(),
            baud: 0,
            host: String::new(),
            port: String::new(),
            url: String::new(),
            ntrip: mp_transport::NtripOptions::default(),
            cancelled: false,
        };
        match kind {
            Kind::Ntrip => {
                page.baud.select(0);
                if page.sendgga.0 {
                    let (lat, lng, alt) = planned_home;
                    spec.ntrip.lat = lat;
                    spec.ntrip.lng = lng;
                    spec.ntrip.alt = alt;
                }
                spec.ntrip.ntrip_v1 = page.ntrip_v1;
                page.sendgga.1 = false;
                if page.autoconfig {
                    page.autoconfig_changed(false, persisted);
                }
            }
            Kind::TcpClient | Kind::UdpHost | Kind::UdpClient => {
                page.baud.select(0);
            }
            Kind::Serial => {
                // `comPort.PortName = ""` throws.
                if port_text.is_empty() {
                    self.messages.push_back(plain(INVALID_PORT_NAME));
                    return;
                }
                spec.name.clone_from(&port_text);
            }
        }
        persisted.set("SerialInjectGPS_port", port_text);
        persisted.set("SerialInjectGPS_baud", page.baud.text());
        let Ok(baud) = page.baud.text().trim().parse::<u32>() else {
            self.messages.push_back(plain(INVALID_BAUD_RATE));
            return;
        };
        spec.baud = baud;
        // `comPort.Open()`, and its questions.
        // `// C#: ExtLibs/Comms/CommsNTRIP.cs:96-114; CommsTCPSerial.cs:101-143;
        // CommsUdpSerial.cs:100-118; CommsUDPSerialConnect.cs:123-149`
        let setting = |key: &str, default: &str| persisted.get(key).unwrap_or(default).to_owned();
        self.prompt = match kind {
            Kind::Ntrip => Some(prompt(
                "remote host",
                "Enter url (eg http://user:pass@host:port/mount)",
                &setting("NTRIP_url", ""),
                Purpose::NtripUrl,
            )),
            Kind::TcpClient => Some(prompt(
                "remote host",
                "Enter host name/ip (ensure remote end is already started)",
                &setting("TCP_host", "127.0.0.1"),
                Purpose::TcpHost,
            )),
            Kind::UdpHost => Some(prompt(
                "Listern Port",
                "Enter Local port (ensure remote end is already sending)",
                &setting("UDP_port", "14550"),
                Purpose::UdpPort,
            )),
            Kind::UdpClient => Some(prompt(
                "remote host",
                "Enter host name/ip (ensure remote end is already started)",
                &setting("UDP_host", "127.0.0.1"),
                Purpose::UdpClientHost,
            )),
            Kind::Serial => None,
        };
        if self.prompt.is_some() {
            self.pending = Some(spec);
        } else {
            self.start(spec, persisted);
        }
    }

    /// A question answered: OK with its text, which `InputBox` keeps in `persisted` as it keeps
    /// every titled answer before the handler looks at it, or Cancel, which keeps nothing.
    /// `// C#: Program.cs:564-566; ExtLibs/Controls/InputBox.cs:73-84, 178-184`
    pub fn answer(&mut self, ok: bool, persisted: &mut Persisted, cs_base: LatLngAlt) {
        let Some(prompt) = self.prompt.take() else {
            return;
        };
        let text = prompt.field.value().to_owned();
        if ok {
            super::optional::remember_answer(persisted, prompt.title, prompt.prompt, &text);
        }
        if prompt.purpose == Purpose::Location {
            if ok && let Some(page) = self.page.as_mut() {
                page.save_base_pos(&text, cs_base, persisted);
            }
            return;
        }
        let Some(mut spec) = self.pending.take() else {
            return;
        };
        if !ok {
            // `UdpSerial.Open` returns; the others throw "Canceled by request" into `DoConnect`'s
            // catch.
            if prompt.purpose == Purpose::UdpPort {
                spec.cancelled = true;
                self.start(spec, persisted);
            } else {
                self.messages.push_back(plain(error_connecting(CANCELED)));
            }
            return;
        }
        let next = match prompt.purpose {
            Purpose::NtripUrl => {
                persisted.set("NTRIP_url", text.clone());
                spec.url = text;
                None
            }
            Purpose::TcpHost => {
                spec.host = text;
                Some(prompt_of(
                    "remote Port",
                    "Enter remote port",
                    persisted.get("TCP_port").unwrap_or("5760"),
                    Purpose::TcpPort,
                ))
            }
            Purpose::TcpPort => {
                persisted.set("TCP_port", text.clone());
                persisted.set("TCP_host", spec.host.clone());
                spec.port = text;
                None
            }
            Purpose::UdpPort => {
                persisted.set("UDP_port", text.clone());
                spec.port = text;
                None
            }
            Purpose::UdpClientHost => {
                spec.host = text;
                Some(prompt_of(
                    "remote Port",
                    "Enter remote port",
                    persisted.get("UDP_port").unwrap_or("14550"),
                    Purpose::UdpClientPort,
                ))
            }
            Purpose::UdpClientPort => {
                persisted.set("UDP_port", text.clone());
                persisted.set("UDP_host", spec.host.clone());
                spec.port = text;
                None
            }
            Purpose::Location => None,
        };
        if next.is_some() {
            self.prompt = next;
            self.pending = Some(spec);
        } else {
            self.start(spec, persisted);
        }
    }

    /// The rest of `DoConnect` on the port's own thread; the button disabled until it has
    /// started, as the C# disables it while a receiver is configured.
    fn start(&mut self, spec: OpenSpec, persisted: &Persisted) {
        let Some(page) = self.page.as_mut() else {
            return;
        };
        let setup = Setup {
            autoconfig: page
                .autoconfig
                .then(|| page.config_type.text().to_owned())
                .filter(|kind| !kind.is_empty()),
            m8p_130p: page.m8p_130p,
            basepos: page.basepos.position(),
            septentrio: page.septentrio_inputs(),
        };
        page.connect.1 = false;
        self.worker = Some(worker::Worker::start(
            spec,
            setup,
            Arc::clone(&self.shared),
            log_directory(persisted),
        ));
    }

    /// What the thread has done since the last frame, done to the page.
    fn take_events(&mut self, persisted: &mut Persisted) {
        for event in self.shared.take_events() {
            match event {
                Event::Started => {
                    if let Some(page) = self.page.as_mut() {
                        page.connect = (STOP, true);
                        page.invalidate(Instant::now());
                        page.bars.clear();
                    }
                }
                Event::Failed { message, open } => {
                    if let Some(page) = self.page.as_mut() {
                        page.connect.1 = true;
                    }
                    if !open {
                        self.worker = None;
                    }
                    self.messages.push_back(plain(message));
                }
                Event::Message(text) => self.messages.push_back(plain(text)),
                Event::Baud(baud) => {
                    if let Some(page) = self.page.as_mut() {
                        select_text(&mut page.baud, &baud.to_string());
                    }
                }
                Event::Setting(key, value) => persisted.set(key, value),
                Event::Svin(svin) => {
                    if let Some(page) = self.page.as_mut() {
                        let zero = page.basepos.is_zero();
                        page.svin.update(zero, &svin);
                    }
                }
                Event::Obs(obs) => {
                    if let Some(page) = self.page.as_mut() {
                        let width = absolute("panel1", !page.autoconfig).2;
                        #[allow(clippy::cast_possible_truncation)]
                        obs_message(&mut page.bars, &obs, width as i32);
                    }
                }
                Event::Seen(seen) => {
                    if let Some(page) = self.page.as_mut()
                        && let Some((light, seconds)) = light_for(seen)
                        && let Some(light) = page.lights.get_mut(light)
                    {
                        light.green = true;
                        light.expires = Some(Instant::now() + Duration::from_secs(seconds));
                    }
                }
                Event::Done(handler, result) => {
                    if let Some(page) = self.page.as_mut() {
                        page.connect.1 = true;
                    }
                    if let Some(text) = done_message(handler, &result) {
                        self.messages.push_back(plain(text));
                    }
                }
            }
        }
    }

    /// `timer1_Tick`: the rates, the messages seen and the base line; the rates reset; the
    /// lights that have lapsed red; the map's marker; the base data flushed.
    /// `// C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.cs:1161-1212`
    fn timer_tick(&mut self, now: Instant, cs_base: LatLngAlt) {
        let Some(page) = self.page.as_mut() else {
            return;
        };
        let mut counts = self.shared.counts();
        let seen: String = counts
            .msgseen
            .iter()
            .map(|(key, count)| format!("{key}={count} "))
            .collect();
        page.status = [
            bps_text(counts.bps, false),
            bps_text(counts.bpsusefull, true),
            counts.status_line3.clone().unwrap_or_default(),
            seen,
        ];
        counts.bps = 0;
        counts.bpsusefull = 0;
        drop(counts);
        page.invalidate(now);
        // `MainV2.comPort.MAV.cs.Base != PointLatLng.Empty`: the marker put there, or moved, and
        // the map zoomed to it.
        if (cs_base.lat != 0.0 || cs_base.lng != 0.0)
            && let Ok(base) = LatLon::new(cs_base.lat, cs_base.lng)
            && page.marker != Some(base)
        {
            page.marker = Some(base);
            let mut map = page.map.borrow_mut();
            map.centre_on(base);
            map.zoom_to_fit(&[base]);
        }
        self.shared
            .flush
            .store(true, std::sync::atomic::Ordering::Release);
    }
}

/// `CustomMessageBox.Show` of a handler's catch, for how its command went: each handler's own
/// message for a failed acknowledgement, the exception's message where the handler shows it,
/// and nothing where it catches silently or not at all.
/// `// C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.cs:533-547, 565-579, 1469-1475,
/// 1486-1490, 1531-1537, 1603-1616, 1621-1624`
#[must_use]
pub fn done_message(handler: Handler, result: &Result<(), septentrio::Failure>) -> Option<String> {
    use septentrio::Failure;
    let Err(failure) = result else {
        return None;
    };
    let fixed = "Configuration of fixed position on Septentrio receiver failed.";
    Some(match (handler, failure) {
        (Handler::Connect, Failure::FailedAck) => {
            "Automatic configuration of Septentrio receiver failed.".to_owned()
        }
        (Handler::Connect, Failure::InvalidOperation(_) | Failure::Format(_)) => {
            "Septentrio fixed base position is invalid.".to_owned()
        }
        (Handler::FixedPosition | Handler::SetPosition | Handler::Amount, Failure::FailedAck) => {
            fixed.to_owned()
        }
        (Handler::Interval, Failure::FailedAck) => {
            "Configuration of RTCM interval on Septentrio receiver failed.".to_owned()
        }
        (Handler::Interval, Failure::Format(text) | Failure::InvalidOperation(text)) => {
            text.clone()
        }
        _ => return None,
    })
}

/// `InputBox` for `Open`'s questions and the location's name.
fn prompt(title: &'static str, question: &'static str, value: &str, purpose: Purpose) -> Prompt {
    prompt_of(title, question, value, purpose)
}

fn prompt_of(title: &'static str, question: &'static str, value: &str, purpose: Purpose) -> Prompt {
    Prompt {
        title,
        prompt: question,
        field: field(value),
        purpose,
    }
}

/// `Settings.Instance.LogDir`: the `logdirectory` key, or the default under the user data
/// directory, made if it is missing.
/// `// C#: ExtLibs/Utilities/Settings.cs:127-160`
fn log_directory(persisted: &Persisted) -> Option<std::path::PathBuf> {
    persisted.config().log_directory().or_else(|| {
        let dir = mp_settings::default_log_directory()?;
        let _ = std::fs::create_dir_all(&dir);
        Some(dir)
    })
}

/// `myGMAP1.MapProvider = FlightData.mymap.MapProvider`: a tile store of its own for the flight
/// map's source, replaced only when that changes.
/// `// C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.cs:1216`
fn use_imagery(map: &Rc<RefCell<MapViewport>>, source: Option<&str>) {
    let Some(source) = source.and_then(mp_tiles::source::source_by_id) else {
        return;
    };
    if map.borrow().source_id() == Some(source.id) {
        return;
    }
    let cache = mp_tiles::cache::TileCache::new(mp_tiles::cache::TileCache::default_root());
    let store = if std::env::var("MP_OFFLINE").is_ok() {
        mp_tiles::TileStore::offline(source, cache)
    } else {
        mp_tiles::TileStore::new(source, cache)
    };
    map.borrow_mut().set_tiles(Arc::new(store));
}

impl Page {
    /// `CMB_serialport_SelectedIndexChanged`: the baud rate enabled for a serial port.
    ///
    /// The C# enables it when the port's name contains "com", which names a serial port on
    /// Windows only: on Linux its serial ports are `/dev/tty*`, and the test would leave their
    /// baud rate unsettable. Here it is enabled for anything but the four network choices.
    /// `// C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.cs:1153-1159`
    fn port_changed(&mut self) {
        self.baud.enabled = Kind::of(self.ports.text()) == Kind::Serial;
    }

    /// `chk_autoconfig_CheckedChanged`.
    /// `// C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.cs:1282-1288`
    fn autoconfig_changed(&mut self, checked: bool, persisted: &mut Persisted) {
        self.autoconfig = checked;
        persisted.set(
            "SerialInjectGPS_autoconfig",
            worker::bool_text(self.autoconfig),
        );
    }

    /// `comboBoxConfigType_SelectedIndexChanged`: saved, and its panel shown.
    /// `// C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.cs:1441-1455`
    fn config_type_changed(&mut self, persisted: &mut Persisted) {
        let text = self.config_type.text().to_owned();
        self.panels = [
            text == "UBlox M8P/F9P",
            text == "Septentrio",
            text == "Unicore UM982",
        ];
        persisted.set("SerialInjectGPS_AutoConfigType", text);
    }

    /// `invalidateRTCMStatus`: each light whose time has run out, red.
    /// `// C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.cs:584-596`
    fn invalidate(&mut self, now: Instant) {
        for light in &mut self.lights {
            if light.expires.is_none_or(|at| at < now) {
                light.green = false;
            }
        }
    }

    /// The Septentrio panel's boxes, as a handler reads them.
    fn septentrio_inputs(&self) -> SeptentrioInputs {
        SeptentrioInputs {
            fixed: self.fixed_position,
            position: self.position.each_ref().map(|box_| box_.value().to_owned()),
            amount: self.amount.text().to_owned(),
            interval: self.interval.value().to_owned(),
            signals: self.signals,
        }
    }

    /// `updateBasePosDG`: the grid rebuilt from the list and the list saved - unless the list is
    /// empty, when neither happens.
    /// `// C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.cs:1290-1306`
    fn update_grid(&mut self) {
        if self.list.is_empty() {
            return;
        }
        self.grid = self
            .list
            .iter()
            .map(|point| {
                [
                    invariant(point.lat),
                    invariant(point.lng),
                    invariant(point.alt),
                    point.tag.clone(),
                ]
            })
            .collect();
        self.save_list();
    }

    /// `saveBasePosList`.
    fn save_list(&self) {
        if let Some(path) = self.list_file.as_ref() {
            let _ = std::fs::write(path, basepos_list_xml(&self.list));
        }
    }

    /// `but_save_basepos_Click` after its question: `base_pos` saved as `cs.Base` and the name,
    /// and the point added to the list.
    /// `// C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.cs:1260-1280`
    fn save_base_pos(&mut self, location: &str, base: LatLngAlt, persisted: &mut Persisted) {
        persisted.set(
            "base_pos",
            base_pos_setting(base.lat, base.lng, base.alt, location),
        );
        self.list.push(BasePos {
            lat: base.lat,
            lng: base.lng,
            alt: base.alt,
            tag: location.to_owned(),
        });
        self.update_grid();
    }

    /// `dg_basepos_CellEndEdit`: the list made long enough for the row and the cell's value put
    /// into it - a number that does not parse changes nothing and saves nothing.
    /// `// C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.cs:1335-1368`
    pub fn cell_end_edit(&mut self, row: usize, column: usize) {
        while self.list.len() <= row {
            self.list.push(BasePos::default());
        }
        let value = self
            .grid
            .get(row)
            .and_then(|cells| cells.get(column))
            .cloned()
            .unwrap_or_default();
        let Some(point) = self.list.get_mut(row) else {
            return;
        };
        match column {
            0..=2 => {
                let Some(number) = parse_double(&value) else {
                    return;
                };
                match column {
                    0 => point.lat = number,
                    1 => point.lng = number,
                    _ => point.alt = number,
                }
            }
            3 => point.tag = value,
            _ => return,
        }
        self.save_list();
    }

    /// `dg_basepos_RowsRemoved`.
    /// `// C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.cs:1370-1379`
    fn rows_removed(&mut self, row: usize) {
        if self.list.is_empty() {
            return;
        }
        if row < self.list.len() {
            self.list.remove(row);
        }
        self.save_list();
    }
}

impl RtkInject {
    /// `dg_basepos_CellContentClick`: Use makes the row the base position - and, with the port
    /// open, the receiver's; Delete removes the row.
    /// `// C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.cs:1308-1333`
    pub fn cell_click(&mut self, row: usize, column: usize, persisted: &mut Persisted) {
        let open = self.is_open();
        let Some(page) = self.page.as_mut() else {
            return;
        };
        match column {
            4 => {
                // A cell of the new row has no value: `null.ToInvariantString()` is null, and
                // `String.Format` writes it as nothing.
                let cells = page.grid.get(row).cloned().unwrap_or_default();
                let [lat, lng, alt, name] = &cells;
                persisted.set("base_pos", format!("{lat},{lng},{alt},{name}"));
                page.basepos = load_base_pos(persisted.get("base_pos"));
                if open {
                    // `int.Parse` and `double.Parse` of the boxes, outside any `try`: a box that
                    // does not parse throws in the C#, and here nothing is sent.
                    let duration = page.survey_dur.value().trim().parse::<i32>();
                    let accuracy = parse_double(page.survey_acc.value());
                    if let (Ok(duration), Some(accuracy), Some(worker)) =
                        (duration, accuracy, self.worker.as_ref())
                    {
                        worker.command(Command::UseBase {
                            basepos: page.basepos.position(),
                            duration,
                            accuracy,
                        });
                    }
                }
            }
            // The new row cannot be removed: `RemoveAt` throws for it.
            5 if row < page.grid.len() => {
                page.grid.remove(row);
                page.rows_removed(row);
            }
            _ => {}
        }
    }

    /// `but_restartsvin_Click`: no base position, the lights checked, the survey-in labels
    /// cleared, the messages seen forgotten; with the port open, the receiver surveying in
    /// afresh.
    /// `// C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.cs:1396-1422`
    pub fn restart_click(&mut self) {
        let open = self.is_open();
        let Some(page) = self.page.as_mut() else {
            return;
        };
        page.basepos = BasePos::default();
        page.invalidate(Instant::now());
        page.svin.update(
            true,
            &Svin {
                valid: false,
                active: false,
                dur: 0,
                obs: 0,
                acc: 0.0,
                ecef: [0.0; 3],
                mode: ubx::Tmode3::default(),
            },
        );
        self.shared.counts().msgseen.clear();
        if open && let Some(worker) = self.worker.as_ref() {
            worker.command(Command::Restart {
                m8p_130p: page.m8p_130p,
                duration: page.survey_dur.value().to_owned(),
                accuracy: page.survey_acc.value().to_owned(),
            });
        }
    }

    /// `but_save_basepos_Click`: the name asked for. `cs.Base` is never null - an unset base is
    /// (0, 0, 0) - so its "No valid base position" box is never shown.
    /// `// C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.cs:1260-1280`
    pub fn save_click(&mut self) {
        self.prompt = Some(prompt(
            "Enter Location",
            "Enter a friendly name for this location.",
            "",
            Purpose::Location,
        ));
    }

    /// The Septentrio handlers: each command with its handler's own conditions.
    pub fn septentrio(&mut self, handler: Handler) {
        let open = self.is_open();
        let Some(page) = self.page.as_mut() else {
            return;
        };
        let inputs = page.septentrio_inputs();
        let command = match handler {
            // C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.cs:1458-1476
            Handler::FixedPosition => {
                page.set_position_enabled = page.fixed_position;
                open.then_some(Command::SeptentrioPosition(handler, inputs))
            }
            // C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.cs:1478-1491 - `float.Parse`
            // of each box, uncaught in the C#; a box that does not parse sends nothing here.
            Handler::SetPosition => {
                let parsed: Vec<f32> = inputs
                    .position
                    .iter()
                    .filter_map(|text| text.trim().parse::<f32>().ok())
                    .collect();
                match (open, parsed.as_slice()) {
                    (true, &[lat, lng, alt]) => Some(Command::SeptentrioSetPosition(
                        [lat, lng, alt],
                        inputs.position.clone(),
                    )),
                    _ => None,
                }
            }
            // C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.cs:1520-1538
            Handler::Amount => (open && page.config_type.text() == "Septentrio")
                .then_some(Command::SeptentrioRtcm(handler, inputs)),
            // C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.cs:1597-1617 - no `IsOpen`:
            // the closed port throws into the handler's `InvalidOperationException` catch.
            Handler::Interval => {
                if !open {
                    self.messages.push_back(plain(PORT_CLOSED));
                    return;
                }
                Some(Command::SeptentrioRtcm(handler, inputs))
            }
            // C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.cs:1621-1624 - no `IsOpen` and
            // no `catch`: the closed port's exception is unhandled in the C#; nothing here.
            Handler::Constellation => open.then_some(Command::SeptentrioRtcm(handler, inputs)),
            Handler::Connect => None,
        };
        if let (Some(command), Some(worker)) = (command, self.worker.as_ref()) {
            worker.command(command);
        }
    }

    /// `chk_rtcmmsg_CheckedChanged`: which message injects.
    /// `// C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.cs:1229-1232`
    pub fn rtcmmsg_changed(&mut self, checked: bool) {
        if let Some(page) = self.page.as_mut() {
            page.rtcmmsg = checked;
        }
        self.shared
            .rtcm_msg
            .store(checked, std::sync::atomic::Ordering::Release);
    }

    /// `labelmsgseen_Click`: the messages seen forgotten.
    /// `// C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.cs:1430-1433`
    pub fn msgseen_click(&mut self) {
        self.shared.counts().msgseen.clear();
    }

    /// A row chosen from one of the lists: its `SelectedIndexChanged`, when the selection moved.
    pub fn choose(&mut self, which: Dropdown, key: i64, persisted: &mut Persisted) {
        let Some(page) = self.page.as_mut() else {
            return;
        };
        page.dropdown = None;
        match which {
            Dropdown::Port => {
                if page.ports.select(key) {
                    page.port_changed();
                }
            }
            Dropdown::Baud => {
                page.baud.select(key);
            }
            Dropdown::ConfigType => {
                if page.config_type.select(key) {
                    page.config_type_changed(persisted);
                }
            }
            Dropdown::Amount => {
                if page.amount.select(key) {
                    persisted.set("SerialInjectGPS_SeptentrioRTCMLevel", key.to_string());
                    self.septentrio(Handler::Amount);
                }
            }
        }
    }

    /// A keystroke in the box being typed into: the survey-in boxes' `TextChanged` saves them;
    /// a grid cell's edit ends on Enter, and Escape puts its text back first.
    pub fn key(&mut self, event: &gpui::KeyDownEvent, persisted: &mut Persisted) -> bool {
        let Some(page) = self.page.as_mut() else {
            return false;
        };
        let Some(editing) = page.editing else {
            return false;
        };
        match editing {
            Editing::SurveyAcc | Editing::SurveyDur => {
                let (field, key) = if editing == Editing::SurveyAcc {
                    (&mut page.survey_acc, "SerialInjectGPS_SIAcc")
                } else {
                    (&mut page.survey_dur, "SerialInjectGPS_SITime")
                };
                if field.key(event) == KeyOutcome::Changed {
                    // `txt_surveyinAcc_TextChanged`, `txt_surveyinDur_TextChanged`
                    // `// C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.cs:1386-1394`
                    persisted.set(key, field.value());
                }
                true
            }
            Editing::Position(index) => page
                .position
                .get_mut(index)
                .is_some_and(|field| field.key(event) != KeyOutcome::Ignored),
            Editing::Interval => page.interval.key(event) != KeyOutcome::Ignored,
            Editing::Cell(row, column) => {
                if row >= page.grid.len() {
                    // `DefaultValuesNeeded`: the new row takes its Use and Delete, and becomes
                    // a row.
                    page.grid.push(Default::default());
                }
                let Some(cell) = page
                    .grid
                    .get_mut(row)
                    .and_then(|cells| cells.get_mut(column))
                else {
                    return false;
                };
                let mut editor = field(cell);
                match editor.key(event) {
                    KeyOutcome::Changed => *cell = editor.value().to_owned(),
                    KeyOutcome::Submitted => {
                        page.editing = None;
                        page.cell_end_edit(row, column);
                    }
                    KeyOutcome::Cancelled => {
                        cell.clone_from(&page.edit_was);
                        page.editing = None;
                        page.cell_end_edit(row, column);
                    }
                    KeyOutcome::Ignored => return false,
                }
                true
            }
        }
    }
}

/// Facts a UI test asserts on.
pub fn record_facts(rtk: &RtkInject, view: &TelemetryView, persisted: &Persisted) {
    use crate::facts::record;
    let none = || "none".to_owned();
    record("config.rtk.title", TITLE);
    record("config.rtk.active", rtk.is_active());
    record("config.rtk.open", rtk.is_open());
    record("config.rtk.running", rtk.is_running());
    record(
        "config.rtk.prompt",
        rtk.prompt.as_ref().map_or("none", |prompt| prompt.title),
    );
    record(
        "config.rtk.prompt.value",
        rtk.prompt
            .as_ref()
            .map_or_else(none, |prompt| prompt.field.value().to_owned()),
    );
    record(
        "config.rtk.message",
        rtk.messages
            .front()
            .map_or_else(none, |message| message.text.replace('\n', " ")),
    );
    let counts = rtk.shared.counts();
    record(
        "config.rtk.messages",
        counts.msgseen.values().map(|&n| i64::from(n)).sum::<i64>(),
    );
    record(
        "config.rtk.types",
        counts.msgseen.keys().cloned().collect::<Vec<_>>().join(","),
    );
    record("config.rtk.injected", counts.injected);
    record("config.rtk.injected.messages", counts.injected_messages);
    record("config.rtk.logged", counts.logged);
    record(
        "config.rtk.log",
        counts
            .log
            .as_ref()
            .map_or_else(none, |path| path.display().to_string()),
    );
    let mirror = counts.base;
    drop(counts);
    let base = cs_base(view, mirror);
    record(
        "config.rtk.cs.base",
        if base == LatLngAlt::ZERO {
            none()
        } else {
            format!("{},{},{}", base.lat, base.lng, base.alt)
        },
    );
    record(
        "config.rtk.setting.port",
        persisted.get("SerialInjectGPS_port").unwrap_or("none"),
    );
    record(
        "config.rtk.setting.url",
        persisted.get("NTRIP_url").unwrap_or("none"),
    );
    record("config.rtk.dimmed", CAN_DIMMED);
    let Some(page) = rtk.page.as_ref() else {
        record("config.rtk.page", false);
        return;
    };
    record("config.rtk.page", true);
    record("config.rtk.port", page.ports.text());
    record(
        "config.rtk.ports",
        page.ports
            .options
            .iter()
            .map(|(_, name)| name.as_str())
            .collect::<Vec<_>>()
            .join(","),
    );
    record("config.rtk.baud", page.baud.text());
    record("config.rtk.baud.enabled", page.baud.enabled);
    record("config.rtk.button", page.connect.0);
    record("config.rtk.button.enabled", page.connect.1);
    record("config.rtk.sendgga", page.sendgga.0);
    record("config.rtk.sendgga.enabled", page.sendgga.1);
    record("config.rtk.autoconfig", page.autoconfig);
    record("config.rtk.configtype", page.config_type.text());
    let [bps, sent, base_line, seen] = &page.status;
    record("config.rtk.bps", bps.trim());
    record("config.rtk.bps.sent", sent.trim());
    record(
        "config.rtk.base",
        if base_line.is_empty() {
            none()
        } else {
            base_line.clone()
        },
    );
    record("config.rtk.msgseen", seen.trim());
    record(
        "config.rtk.lights",
        LIGHTS
            .iter()
            .zip(page.lights)
            .map(|((_, name), light)| {
                format!("{name}={}", if light.green { "Green" } else { "Red" })
            })
            .collect::<Vec<_>>()
            .join(","),
    );
    record("config.rtk.bars", page.bars.len());
    record(
        "config.rtk.bars.labels",
        page.bars
            .iter()
            .map(|bar| bar.label.as_str())
            .collect::<Vec<_>>()
            .join(","),
    );
    record(
        "config.rtk.svin",
        if page.svin.visible[0] {
            page.svin.text[0].clone()
        } else {
            "hidden".to_owned()
        },
    );
    record(
        "config.rtk.basepos",
        if page.basepos.is_zero() {
            "zero".to_owned()
        } else {
            base_pos_setting(
                page.basepos.lat,
                page.basepos.lng,
                page.basepos.alt,
                &page.basepos.tag,
            )
        },
    );
    record("config.rtk.grid.rows", page.grid.len());
}

/// `MainV2.comPort.MAV.cs.Base`: the vehicle's, or with none the blank `MAVState`'s.
#[must_use]
pub fn cs_base(view: &TelemetryView, mirror: Option<LatLngAlt>) -> LatLngAlt {
    match view.state.as_deref() {
        Some(state) => state.base,
        None => mirror.unwrap_or(LatLngAlt::ZERO),
    }
}

// ---------------------------------------------------------------------------------------------
// Drawing the page, and its part in the application.
// ---------------------------------------------------------------------------------------------

/// The keyboard focus of the page's text boxes and of its question's answer.
pub struct Focus {
    /// The text box or grid cell being typed into.
    pub text: FocusHandle,
    /// The `InputBox`'s answer.
    pub prompt: FocusHandle,
}

impl Focus {
    /// New handles.
    pub fn new(cx: &mut Context<MissionPlanner>) -> Self {
        Self {
            text: cx.focus_handle(),
            prompt: cx.focus_handle(),
        }
    }
}

/// `Color.Green` and `Color.Red`, the lights' `BackColor`s.
const GREEN: u32 = 0x00_80_00;
const RED: u32 = 0xff_00_00;

/// A check box of the page.
fn check(checked: bool, enabled: bool) -> Check {
    let mut check = Check::default();
    check.state = if checked {
        CheckState::Checked
    } else {
        CheckState::Unchecked
    };
    check.enabled = enabled;
    check
}

/// A group box's frame and caption.
fn frame(place: Place, caption: &'static str) -> AnyElement {
    group(place, caption, true).into_any_element()
}

/// A label of the page at its place.
fn text_at(place: Place, text: impl Into<SharedString>) -> AnyElement {
    let (x, y, _, _) = place;
    label(x, y, text, true).into_any_element()
}

/// The page, as the Designer lays it out, over the page area.
/// `// C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.Designer.cs:29-783`
#[allow(clippy::too_many_lines)]
pub fn page(
    rtk: &RtkInject,
    focus: &Focus,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let Some(page) = rtk.page.as_ref().filter(|_| rtk.is_active()) else {
        return div().into_any_element();
    };
    let collapsed = !page.autoconfig;
    let p = |name: &str| absolute(name, collapsed);
    let typing = focus.text.is_focused(window);
    let mut body = crate::probe::measured("rtk-page", div())
        .relative()
        .w(px(PAGE_SIZE.0))
        .h(px(PAGE_SIZE.1));

    // The port, its button and its speed.
    body = body
        .child(combo_box(
            "rtk-CMB_serialport".to_owned(),
            &page.ports,
            p("CMB_serialport"),
            |this| this.rtk_toggle(Dropdown::Port),
            cx,
        ))
        .child(button(
            "rtk-BUT_connect",
            page.connect.0,
            p("BUT_connect"),
            page.connect.1,
            |this, _window, _cx| {
                let home = this.plan.planned_home_location();
                this.rtk_inject
                    .connect_click(&mut this.persisted, (home.lat, home.lng, home.alt));
            },
            cx,
        ))
        .child(combo_box(
            "rtk-CMB_baudrate".to_owned(),
            &page.baud,
            p("CMB_baudrate"),
            |this| this.rtk_toggle(Dropdown::Baud),
            cx,
        ));
    // `chk_rtcmmsg` is `Visible = False`: its handler runs only from the constructor.
    let (x, y, _, _) = p("chk_sendgga");
    body = body.child(check_box(
        "rtk-chk_sendgga".to_owned(),
        &check(page.sendgga.0, page.sendgga.1),
        text_of("chk_sendgga"),
        (x, y),
        |this| {
            if let Some(page) = this.rtk_inject.page.as_mut() {
                page.sendgga.0 = !page.sendgga.0;
            }
        },
        cx,
    ));
    let (x, y, _, _) = p("check_sendntripv1");
    body = body.child(check_box(
        "rtk-check_sendntripv1".to_owned(),
        &check(page.ntrip_v1, true),
        text_of("check_sendntripv1"),
        (x, y),
        |this| {
            if let Some(page) = this.rtk_inject.page.as_mut() {
                page.ntrip_v1 = !page.ntrip_v1;
            }
        },
        cx,
    ));
    let (x, y, _, _) = p("chk_autoconfig");
    body = body.child(check_box(
        "rtk-chk_autoconfig".to_owned(),
        &check(page.autoconfig, true),
        text_of("chk_autoconfig"),
        (x, y),
        |this| {
            if let Some(page) = this.rtk_inject.page.as_mut() {
                page.autoconfig_changed(!page.autoconfig, &mut this.persisted);
            }
        },
        cx,
    ));
    if page.autoconfig {
        body = body.child(combo_box(
            "rtk-comboBoxConfigType".to_owned(),
            &page.config_type,
            p("comboBoxConfigType"),
            |this| this.rtk_toggle(Dropdown::ConfigType),
            cx,
        ));
    }

    // Link Status.
    body = body.child(frame(p("groupBox3"), text_of("groupBox3")));
    for name in ["label3", "label4", "label6"] {
        body = body.child(text_at(p(name), text_of(name)));
    }
    let [bps, sent, base_line, seen] = &page.status;
    body = body
        .child(text_at(p("lbl_status1"), bps.clone()))
        .child(text_at(p("lbl_status2"), sent.clone()));
    let (x, y, width, height) = p("labelmsgseen");
    body = body.child(
        crate::probe::measured("rtk-labelmsgseen", at(x, y, width, height))
            .id("rtk-labelmsgseen")
            .overflow_hidden()
            .text_xs()
            .text_color(rgb(theme::TEXT))
            .cursor_pointer()
            .child(seen.clone())
            .on_click(cx.listener(|this, _event, _window, cx| {
                this.rtk_inject.msgseen_click();
                cx.notify();
            })),
    );

    // RTCM.
    body = body.child(frame(p("groupBox2"), text_of("groupBox2")));
    for name in [
        "label11", "label12", "label13", "label15", "label16", "label5",
    ] {
        body = body.child(text_at(p(name), text_of(name)));
    }
    for ((name, _), light) in LIGHTS.iter().zip(page.lights) {
        let (x, y, width, height) = p(name);
        body = body.child(
            crate::probe::measured(format!("rtk-{name}"), at(x, y, width, height))
                .bg(rgb(if light.green { GREEN } else { RED })),
        );
    }
    body = body.child(text_at(p("lbl_status3"), base_line.clone()));

    // `myGMAP1`, with the base's yellow dot.
    let (x, y, width, height) = p("myGMAP1");
    body = body.child(
        crate::probe::measured("rtk-myGMAP1", at(x, y, width, height))
            .overflow_hidden()
            .border_1()
            .border_color(rgb(theme::BORDER))
            .child(crate::mapview::map_element(Rc::clone(&page.map)))
            .child(marker_element(Rc::clone(&page.map), page.marker)),
    );

    // `splitContainer1`: the automatic configuration's options, when shown, over the bars.
    if page.autoconfig {
        body = body.child(frame(
            p("groupBox_autoconfig"),
            text_of("groupBox_autoconfig"),
        ));
        if page.panels[0] {
            body = ublox_panel(body, page, &p, typing, focus, cx);
        }
        if page.panels[1] {
            body = septentrio_panel(body, page, &p, typing, focus, cx);
        }
        if page.panels[2] {
            let (x, y, width, height) = p("label22");
            body = body.child(
                at(x, y, width, height)
                    .flex()
                    .flex_col()
                    .text_xs()
                    .text_color(rgb(theme::TEXT))
                    .children(UM982_TEXT.lines().map(|line| div().child(line.to_owned()))),
            );
        }
    }
    body = body.child(bars_element(page, p("panel1")));

    // A list that is down.
    if let Some(which) = page.dropdown {
        let (name, combo) = match which {
            Dropdown::Port => ("CMB_serialport", &page.ports),
            Dropdown::Baud => ("CMB_baudrate", &page.baud),
            Dropdown::ConfigType => ("comboBoxConfigType", &page.config_type),
            Dropdown::Amount => ("cmb_septentriortcmamount", &page.amount),
        };
        let (x, y, width, height) = p(name);
        body = body.child(dropdown(
            &format!("rtk-{name}"),
            combo,
            (x, y + height, width),
            move |this, key| {
                this.rtk_inject.choose(which, key, &mut this.persisted);
            },
            move |this, lines| {
                if let Some(page) = this.rtk_inject.page.as_mut() {
                    let combo = match which {
                        Dropdown::Port => &mut page.ports,
                        Dropdown::Baud => &mut page.baud,
                        Dropdown::ConfigType => &mut page.config_type,
                        Dropdown::Amount => &mut page.amount,
                    };
                    combo.scroll_list(lines);
                }
            },
            cx,
        ));
    }
    div().flex().flex_col().child(body).into_any_element()
}

/// A text box of the page, typed into while it has the focus.
#[allow(clippy::too_many_arguments)]
fn page_text_box(
    id: &'static str,
    field: &TextField,
    editing: Editing,
    page: &Page,
    typing: bool,
    focus: &Focus,
    place: Place,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    text_box(
        id,
        field.value(),
        Some(&focus.text),
        typing && page.editing == Some(editing),
        true,
        place,
        move |this| {
            if let Some(page) = this.rtk_inject.page.as_mut() {
                // A grid cell being edited is left, which ends its edit.
                if let Some(Editing::Cell(row, column)) = page.editing {
                    page.cell_end_edit(row, column);
                }
                page.editing = Some(editing);
            }
        },
        |this, event| this.rtk_inject.key(event, &mut this.persisted),
        cx,
    )
}

/// `panel_ubloxoptions`: `panel2` - the 1.30 box, the survey-in boxes, Restart, Save Current
/// Position and the base positions - and the Survey In group.
/// `// C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.Designer.cs:361-447`
fn ublox_panel(
    mut body: gpui::Div,
    page: &Page,
    p: &dyn Fn(&str) -> Place,
    typing: bool,
    focus: &Focus,
    cx: &mut Context<MissionPlanner>,
) -> gpui::Div {
    let (x, y, _, _) = p("chk_m8p_130p");
    body = body.child(check_box(
        "rtk-chk_m8p_130p".to_owned(),
        &check(page.m8p_130p, true),
        text_of("chk_m8p_130p"),
        (x, y),
        |this| {
            if let Some(page) = this.rtk_inject.page.as_mut() {
                page.m8p_130p = !page.m8p_130p;
                // `chk_m8p_130p_CheckedChanged`
                // `// C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.cs:1381-1384`
                this.persisted
                    .set("SerialInjectGPS_m8p_130p", worker::bool_text(page.m8p_130p));
            }
        },
        cx,
    ));
    for name in ["label1", "label2"] {
        body = body.child(text_at(p(name), text_of(name)));
    }
    body = body
        .child(page_text_box(
            "rtk-txt_surveyinAcc",
            &page.survey_acc,
            Editing::SurveyAcc,
            page,
            typing,
            focus,
            p("txt_surveyinAcc"),
            cx,
        ))
        .child(page_text_box(
            "rtk-txt_surveyinDur",
            &page.survey_dur,
            Editing::SurveyDur,
            page,
            typing,
            focus,
            p("txt_surveyinDur"),
            cx,
        ))
        .child(button(
            "rtk-but_restartsvin",
            text_of("but_restartsvin"),
            p("but_restartsvin"),
            true,
            |this, _window, _cx| this.rtk_inject.restart_click(),
            cx,
        ))
        .child(button(
            "rtk-but_save_basepos",
            text_of("but_save_basepos"),
            p("but_save_basepos"),
            true,
            |this, _window, _cx| this.rtk_inject.save_click(),
            cx,
        ))
        .child(grid_element(page, p("dg_basepos"), typing, focus, cx));

    body = body.child(frame(p("groupBox1"), text_of("groupBox1")));
    for (index, name) in ["lbl_svin", "label7", "label8", "label9", "label10"]
        .into_iter()
        .enumerate()
    {
        if !page.svin.visible.get(index).copied().unwrap_or(false) {
            continue;
        }
        let (x, y, width, height) = p(name);
        let mut cell = crate::probe::measured(format!("rtk-{name}"), at(x, y, width, height))
            .text_xs()
            .overflow_hidden()
            .whitespace_nowrap()
            .text_color(rgb(theme::TEXT))
            .child(page.svin.text.get(index).cloned().unwrap_or_default());
        if index == 0
            && let Some(valid) = page.svin.valid
        {
            cell = cell.bg(rgb(if valid { GREEN } else { RED }));
        }
        body = body.child(cell);
    }
    body
}

/// `panel_septentrio`: the fixed position, the RTCM amount and interval, the constellations.
/// `// C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.Designer.cs:204-359`
fn septentrio_panel(
    mut body: gpui::Div,
    page: &Page,
    p: &dyn Fn(&str) -> Place,
    typing: bool,
    focus: &Focus,
    cx: &mut Context<MissionPlanner>,
) -> gpui::Div {
    for name in [
        "label14", "label17", "label18", "label19", "label20", "label21",
    ] {
        body = body.child(text_at(p(name), text_of(name)));
    }
    let (x, y, _, _) = p("chk_septentriofixedposition");
    body = body.child(check_box(
        "rtk-chk_septentriofixedposition".to_owned(),
        &check(page.fixed_position, true),
        text_of("chk_septentriofixedposition"),
        (x, y),
        |this| {
            if let Some(page) = this.rtk_inject.page.as_mut() {
                page.fixed_position = !page.fixed_position;
            }
            this.rtk_inject.septentrio(Handler::FixedPosition);
        },
        cx,
    ));
    for (index, (id, name)) in [
        (
            "rtk-input_septentriofixedatitude",
            "input_septentriofixedatitude",
        ),
        (
            "rtk-input_septentriofixedlongitude",
            "input_septentriofixedlongitude",
        ),
        (
            "rtk-input_septentriofixedaltitude",
            "input_septentriofixedaltitude",
        ),
    ]
    .into_iter()
    .enumerate()
    {
        if let Some(field) = page.position.get(index) {
            body = body.child(page_text_box(
                id,
                field,
                Editing::Position(index),
                page,
                typing,
                focus,
                p(name),
                cx,
            ));
        }
    }
    body = body
        .child(button(
            "rtk-button_septentriosetposition",
            text_of("button_septentriosetposition"),
            p("button_septentriosetposition"),
            page.set_position_enabled,
            |this, _window, _cx| this.rtk_inject.septentrio(Handler::SetPosition),
            cx,
        ))
        .child(combo_box(
            "rtk-cmb_septentriortcmamount".to_owned(),
            &page.amount,
            p("cmb_septentriortcmamount"),
            |this| this.rtk_toggle(Dropdown::Amount),
            cx,
        ))
        .child(page_text_box(
            "rtk-input_septentriortcminterval",
            &page.interval,
            Editing::Interval,
            page,
            typing,
            focus,
            p("input_septentriortcminterval"),
            cx,
        ))
        .child(button(
            "rtk-button_septentriortcminterval",
            text_of("button_septentriortcminterval"),
            p("button_septentriortcminterval"),
            true,
            |this, _window, _cx| this.rtk_inject.septentrio(Handler::Interval),
            cx,
        ));
    let signals = page.signals;
    for (name, on, pick) in [
        (
            "chk_septentriogps",
            signals.gps,
            (|s: &mut septentrio::Signals| s.gps = !s.gps) as fn(&mut septentrio::Signals),
        ),
        ("chk_septentrioglonass", signals.glonass, |s| {
            s.glonass = !s.glonass;
        }),
        ("chk_septentriobeidou", signals.beidou, |s| {
            s.beidou = !s.beidou
        }),
        ("chk_septentriogalileo", signals.galileo, |s| {
            s.galileo = !s.galileo;
        }),
    ] {
        let (x, y, _, _) = p(name);
        body = body.child(check_box(
            format!("rtk-{name}"),
            &check(on, true),
            text_of(name),
            (x, y),
            move |this| {
                if let Some(page) = this.rtk_inject.page.as_mut() {
                    pick(&mut page.signals);
                }
                this.rtk_inject.septentrio(Handler::Constellation);
            },
            cx,
        ));
    }
    body
}

/// `dg_basepos`: the column headers, a row per base position and the new row, each text cell
/// typed into when clicked, Use and Delete buttons.
/// `// C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.Designer.cs:694-743`
fn grid_element(
    page: &Page,
    (x, y, width, height): Place,
    typing: bool,
    focus: &Focus,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let mut header = div().flex().h(px(GRID_HEADER)).child(
        div()
            .w(px(GRID_ROW_HEADER))
            .h_full()
            .border_1()
            .border_color(rgb(theme::BORDER)),
    );
    for (_, text, column_width) in COLUMNS {
        header = header.child(
            div()
                .w(px(column_width))
                .h_full()
                .px_1()
                .border_1()
                .border_color(rgb(theme::BORDER))
                .text_xs()
                .text_color(rgb(theme::DIM))
                .child(text),
        );
    }
    let mut rows = div()
        .id("rtk-dg_basepos-rows")
        .flex()
        .flex_col()
        .overflow_y_scroll()
        .h(px(height - GRID_HEADER - 2.0));
    let new_row: [String; 4] = Default::default();
    for (row, cells) in page
        .grid
        .iter()
        .chain(std::iter::once(&new_row))
        .enumerate()
    {
        let mut line = div().flex().flex_shrink_0().h(px(GRID_ROW)).child(
            div()
                .w(px(GRID_ROW_HEADER))
                .h_full()
                .border_1()
                .border_color(rgb(theme::BORDER))
                .text_xs()
                .text_color(rgb(theme::DIM))
                .child(if row == page.grid.len() { "*" } else { "" }),
        );
        for (column, (_, _, column_width)) in COLUMNS.into_iter().enumerate() {
            let id = SharedString::from(format!("rtk-dg_basepos-{row}-{column}"));
            let cell = crate::probe::measured(id.to_string(), div())
                .id(id)
                .w(px(column_width))
                .h_full()
                .px_1()
                .border_1()
                .border_color(rgb(theme::BORDER))
                .overflow_hidden()
                .whitespace_nowrap()
                .text_xs();
            let cell = if column >= 4 {
                cell.bg(rgb(theme::ACTION))
                    .text_color(rgb(theme::TEXT))
                    .cursor_pointer()
                    .child(if column == 4 { "Use" } else { "Delete" })
                    .on_click(cx.listener(move |this, _event, _window, cx| {
                        this.rtk_inject.cell_click(row, column, &mut this.persisted);
                        cx.notify();
                    }))
            } else if typing && page.editing == Some(Editing::Cell(row, column)) {
                cell.track_focus(&focus.text)
                    .key_context("TextField")
                    .border_color(rgb(theme::ACCENT))
                    .text_color(rgb(theme::TEXT))
                    .child(cells.get(column).cloned().unwrap_or_default())
                    .on_key_down(
                        cx.listener(|this, event: &gpui::KeyDownEvent, _window, cx| {
                            if this.rtk_inject.key(event, &mut this.persisted) {
                                cx.notify();
                            }
                        }),
                    )
            } else {
                let handle = focus.text.clone();
                cell.text_color(rgb(theme::TEXT))
                    .cursor_text()
                    .child(cells.get(column).cloned().unwrap_or_default())
                    .on_click(cx.listener(move |this, _event, window, cx| {
                        this.rtk_begin_cell(row, column);
                        handle.focus(window, cx);
                        cx.notify();
                    }))
            };
            line = line.child(cell);
        }
        rows = rows.child(line);
    }
    crate::probe::measured("rtk-dg_basepos", at(x, y, width, height))
        .overflow_hidden()
        .border_1()
        .border_color(rgb(theme::BORDER))
        .bg(rgb(theme::BG))
        .child(header)
        .child(rows)
        .into_any_element()
}

/// `panel1`: a `VerticalProgressBar2` per satellite, 25 to 55 dBHz, the red line at 40, each at
/// the place its last message left it. The C# writes each bar's label and value up its bar;
/// here they are written across it, the label over the value.
/// `// C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.cs:189-247; ExtLibs/Controls/HorizontalProgressBar2.cs:168-230`
fn bars_element(page: &Page, (x, y, width, height): Place) -> AnyElement {
    let (minimum, maximum) = BAR_RANGE;
    let bar_height = height - 30.0;
    #[allow(clippy::cast_precision_loss)]
    let fraction = |value: i32| {
        // `HorizontalProgressBar2.Value`: kept above the minimum, and at most the maximum.
        let value = value.clamp(minimum + 1, maximum);
        (value - minimum) as f32 / (maximum - minimum) as f32
    };
    let mut panel = crate::probe::measured("rtk-panel1", at(x, y, width, height));
    for bar in &page.bars {
        #[allow(clippy::cast_precision_loss)]
        let (left, bar_width) = (bar.x as f32, bar.width as f32);
        panel = panel.child(
            at(left, 0.0, bar_width, bar_height)
                .border_1()
                .border_color(rgb(theme::BORDER))
                .bg(rgb(theme::PANEL))
                .child(
                    div()
                        .absolute()
                        .left_0()
                        .right_0()
                        .bottom_0()
                        .h(gpui::relative(fraction(bar.value)))
                        .bg(rgb(0x40_57_04)),
                )
                .child(
                    div()
                        .absolute()
                        .left_0()
                        .right_0()
                        .top(gpui::relative(fraction(BAR_MINLINE).mul_add(-1.0, 1.0)))
                        .h(px(2.0))
                        .bg(rgb(RED)),
                )
                .child(
                    div()
                        .absolute()
                        .left_0()
                        .right_0()
                        .top_1()
                        .flex()
                        .flex_col()
                        .items_center()
                        .text_size(px(8.0))
                        .text_color(rgb(theme::TEXT))
                        .child(bar.label.clone())
                        .child(bar.value.to_string()),
                ),
        );
    }
    panel.into_any_element()
}

/// `GMarkerGoogleType.yellow_dot`: the base's marker, painted over the map where the map put
/// the base in the same frame.
fn marker_element(map: Rc<RefCell<MapViewport>>, marker: Option<LatLon>) -> impl IntoElement {
    gpui::canvas(
        |_bounds, _window, _cx| (),
        move |_bounds, (), window, _cx| {
            let Some(place) = marker else {
                return;
            };
            let Some((x, y)) = map.borrow().screen_of(place) else {
                return;
            };
            window.paint_quad(gpui::quad(
                gpui::Bounds {
                    origin: gpui::point(px(x - 6.0), px(y - 18.0)),
                    size: gpui::size(px(12.0), px(12.0)),
                },
                gpui::Corners::all(px(6.0)),
                rgb(0xff_d7_00),
                gpui::Edges::all(px(1.0)),
                rgb(0x00_00_00),
                gpui::BorderStyle::Solid,
            ));
        },
    )
    .absolute()
    .size_full()
}

/// The question or message box showing, over the whole window.
pub fn overlay(
    rtk: &RtkInject,
    focus: &Focus,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    if let Some(message) = rtk.messages.front() {
        return Some(message_box(
            "rtk-message",
            "rtk-message-ok",
            message,
            window,
            |this| {
                this.rtk_inject.messages.pop_front();
            },
            cx,
        ));
    }
    let prompt = rtk.prompt.as_ref()?;
    let focused = focus.prompt.is_focused(window);
    let buttons = vec![
        action(
            "rtk-prompt-ok",
            "OK",
            theme::ACCENT,
            true,
            cx.listener(|this, _event: &(), _window, cx| {
                this.rtk_answer(true);
                cx.notify();
            }),
        ),
        action(
            "rtk-prompt-cancel",
            "Cancel",
            theme::DIM,
            true,
            cx.listener(|this, _event: &(), _window, cx| {
                this.rtk_answer(false);
                cx.notify();
            }),
        ),
    ];
    let dialog = crate::probe::measured("rtk-prompt", div())
        .flex()
        .flex_col()
        .gap_2()
        .w(px(340.0))
        .p_3()
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::ACCENT))
        .rounded_md()
        .child(
            div()
                .text_xs()
                .text_color(rgb(theme::DIM))
                .child(prompt.title),
        )
        .child(
            div()
                .text_sm()
                .text_color(rgb(theme::TEXT))
                .child(prompt.prompt),
        )
        .child(crate::textfield::text_field(
            "rtk-prompt-value",
            &prompt.field,
            &focus.prompt,
            focused,
            px(310.0),
            cx.listener(|this, event: &gpui::KeyDownEvent, _window, cx| {
                let outcome = this
                    .rtk_inject
                    .prompt
                    .as_mut()
                    .map(|prompt| prompt.field.key(event));
                match outcome {
                    Some(KeyOutcome::Submitted) => this.rtk_answer(true),
                    Some(KeyOutcome::Cancelled) => this.rtk_answer(false),
                    _ => {}
                }
                cx.notify();
            }),
        ))
        .child(div().flex().justify_end().gap_2().children(buttons));
    let size = window.viewport_size();
    Some(
        gpui::deferred(
            gpui::anchored()
                .position(gpui::point(px(0.0), px(0.0)))
                .child(
                    div()
                        .id("rtk-prompt-backdrop")
                        .w(size.width)
                        .h(size.height)
                        .flex()
                        .items_center()
                        .justify_center()
                        .occlude()
                        .child(dialog),
                ),
        )
        .with_priority(2)
        .into_any_element(),
    )
}

// ---- RTK/GPS Inject ----
impl MissionPlanner {
    /// `Activate`, when the page is chosen.
    pub(crate) fn rtk_inject_activate(&mut self) {
        let key = Key::of(&self.telemetry.view());
        let source = self.map.borrow().source_id();
        self.rtk_inject.activate(key, &mut self.persisted, source);
    }

    /// `Deactivate`, when another is.
    pub(crate) fn rtk_inject_deactivate(&mut self) {
        self.rtk_inject.deactivate();
    }

    /// A combo box's list dropped down, or back up.
    fn rtk_toggle(&mut self, which: Dropdown) {
        if let Some(page) = self.rtk_inject.page.as_mut() {
            page.dropdown = if page.dropdown == Some(which) {
                None
            } else {
                let combo = match which {
                    Dropdown::Port => &mut page.ports,
                    Dropdown::Baud => &mut page.baud,
                    Dropdown::ConfigType => &mut page.config_type,
                    Dropdown::Amount => &mut page.amount,
                };
                combo.open_list();
                Some(which)
            };
        }
    }

    /// A grid cell clicked: its edit begins, a cell being edited before it ended first.
    fn rtk_begin_cell(&mut self, row: usize, column: usize) {
        if let Some(page) = self.rtk_inject.page.as_mut() {
            if let Some(Editing::Cell(was_row, was_column)) = page.editing {
                page.cell_end_edit(was_row, was_column);
            }
            page.edit_was = page
                .grid
                .get(row)
                .and_then(|cells| cells.get(column))
                .cloned()
                .unwrap_or_default();
            page.editing = Some(Editing::Cell(row, column));
        }
    }

    /// The question answered.
    fn rtk_answer(&mut self, ok: bool) {
        let view = self.telemetry.view();
        let mirror = self.rtk_inject.shared.counts().base;
        let base = cs_base(&view, mirror);
        self.rtk_inject.answer(ok, &mut self.persisted, base);
    }

    /// Once a frame: the page object disposed with its screen, the link and vehicles the thread
    /// sends to, what the thread did, the timer, and the question's focus.
    pub(crate) fn rtk_inject_tick(
        &mut self,
        view: &TelemetryView,
        window: &mut Window,
        cx: &mut Context<Self>,
    ) {
        let on_setup = self.screen == crate::Screen::Setup;
        let rtk = &mut self.rtk_inject;
        if !rtk.active
            && rtk
                .page
                .as_ref()
                .is_some_and(|page| !on_setup || page.made_for != Key::of(view))
        {
            rtk.page = None;
        }
        {
            let handle = self.telemetry.send_handle();
            let mut route = rtk.shared.route();
            route.selected = handle.as_ref().map(|(_, id)| *id);
            route.sender = handle.map(|(sender, _)| sender);
            route.vehicles = self.telemetry.vehicles();
        }
        rtk.take_events(&mut self.persisted);
        let now = Instant::now();
        let due = rtk.active
            && rtk.page.as_ref().is_some_and(|page| {
                page.last_tick
                    .is_none_or(|at| now.duration_since(at) >= TIMER)
            });
        if due {
            let mirror = rtk.shared.counts().base;
            rtk.timer_tick(now, cs_base(view, mirror));
            if let Some(page) = rtk.page.as_mut() {
                page.last_tick = Some(now);
            }
        }
        // A cell whose box has lost the focus has its edit ended, as leaving it does.
        if let Some(page) = rtk.page.as_mut()
            && let Some(Editing::Cell(row, column)) = page.editing
            && !self.rtk_focus.text.is_focused(window)
        {
            page.editing = None;
            page.cell_end_edit(row, column);
        }
        if rtk.prompt.is_some()
            && rtk.messages.is_empty()
            && !self.rtk_focus.prompt.is_focused(window)
        {
            self.rtk_focus.prompt.focus(window, cx);
        }
    }
}
// ---- end RTK/GPS Inject ----

/// `timer1.Interval`.
/// `// C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.Designer.cs:162`
const TIMER: Duration = Duration::from_millis(1000);

#[cfg(test)]
#[allow(clippy::cast_possible_truncation, clippy::cast_possible_wrap)]
mod tests {
    use std::collections::BTreeMap;
    use std::time::{Duration, Instant};

    use mp_transport::Transport as _;
    use mp_transport::testing::Loopback;
    use mp_vehicle::VehicleId;

    use super::*;

    /// A minute of a base station's RTCM, made and checked as `testdata/rtcm/generate.py` says.
    const STREAM: &[u8] = include_bytes!("../../../../testdata/rtcm/base-station.rtcm3");

    /// The messages rtcm-rs 0.11.0 - a decoder of its own - reads from the stream, by number.
    const COUNTS: [(i32, usize); 12] = [
        (1004, 1),
        (1005, 6),
        (1006, 1),
        (1012, 1),
        (1033, 1),
        (1074, 59),
        (1077, 1),
        (1084, 59),
        (1087, 1),
        (1094, 60),
        (1124, 60),
        (1230, 6),
    ];

    /// The generator's satellites and their L1 strengths.
    const GPS: [(u8, u8); 8] = [
        (2, 41),
        (5, 44),
        (10, 47),
        (12, 38),
        (15, 50),
        (18, 43),
        (24, 46),
        (29, 39),
    ];
    const GLONASS: [(u8, u8); 6] = [(1, 40), (7, 45), (8, 42), (14, 37), (22, 48), (23, 44)];
    const GALILEO: [(u8, u8); 5] = [(3, 43), (11, 46), (19, 39), (27, 49), (30, 41)];
    const BEIDOU: [(u8, u8); 6] = [(6, 36), (9, 42), (13, 45), (21, 40), (28, 47), (33, 44)];

    /// SITL's home, which the stream's station is.
    const HOME: (f64, f64, f64) = (-35.363261, 149.165230, 584.0);

    fn until(what: &str, mut check: impl FnMut() -> bool) {
        let deadline = Instant::now() + Duration::from_secs(10);
        while !check() {
            assert!(Instant::now() < deadline, "timed out waiting for {what}");
            std::thread::sleep(Duration::from_millis(2));
        }
    }

    /// Every frame of a stream, as the parser reads them: number, bytes, what it raised.
    fn frames(stream: &[u8]) -> Vec<(i32, Vec<u8>, Option<Vec<rtcm3::Ob>>)> {
        let mut parser = rtcm3::Rtcm3::default();
        let mut out = Vec::new();
        for &b in stream {
            let n = parser.read(b).expect("no overflow");
            if n > 0 {
                out.push((n, parser.packet().to_vec(), parser.take_obs()));
            }
        }
        out
    }

    fn obs_of(list: &[(u8, u8)], sys: char) -> Vec<rtcm3::Ob> {
        list.iter()
            .map(|&(prn, snr)| rtcm3::Ob { sys, prn, snr })
            .collect()
    }

    /// Bits written most significant first, as the RTCM layouts are.
    #[derive(Default)]
    struct Bits(Vec<bool>);

    impl Bits {
        fn u(&mut self, value: u64, width: u32) {
            for i in (0..width).rev() {
                self.0.push(i < 64 && (value >> i) & 1 == 1);
            }
        }

        fn frame(&self) -> Vec<u8> {
            let mut payload = vec![0u8; self.0.len().div_ceil(8)];
            for (i, &bit) in self.0.iter().enumerate() {
                if bit {
                    payload[i / 8] |= 0x80 >> (i % 8);
                }
            }
            let mut out = vec![0xD3, (payload.len() >> 8) as u8, payload.len() as u8];
            out.extend_from_slice(&payload);
            let crc = rtcm3::crc24q(&out, 0);
            out.extend_from_slice(&[(crc >> 16) as u8, (crc >> 8) as u8, crc as u8]);
            out
        }
    }

    // ---- rtcm3.cs ----

    /// The table worked out from the polynomial is the C#'s list, value for value.
    /// `// C#: ExtLibs/Utilities/rtcm3.cs:531-565`
    #[test]
    fn the_crc24q_table_is_the_csharps() {
        let Some(source) = crate::config_coverage::source::csharp("ExtLibs/Utilities/rtcm3.cs")
        else {
            eprintln!("skipped: the C# tree is not checked out");
            return;
        };
        let start = source.find("crc24qtab =").expect("the table");
        let body = &source[start..];
        let body = &body[..body.find("};").expect("its end")];
        let values: Vec<u32> = body
            .split(|c: char| c == ',' || c == '{' || c.is_whitespace())
            .filter_map(|token| token.strip_prefix("0x"))
            .map(|hex| u32::from_str_radix(hex, 16).expect("hex"))
            .collect();
        assert_eq!(values.as_slice(), rtcm3::CRC24Q_TABLE.as_slice());
    }

    /// A frame from another implementation's tests - Zephyr's RTK decoder, a 1230 with its
    /// CRC - is read after junk, whole; the same frame with a CRC bit flipped is not.
    #[test]
    fn a_frame_from_another_implementation_is_read_and_a_bad_crc_is_not() {
        // modules/zephyr/tests/subsys/gnss/rtk/rtcm3/src/main.c, test_frame_is_detected
        let frame = [0xD3, 0x00, 0x04, 0x4C, 0xE0, 0x00, 0x80, 0xED, 0xED, 0xD6];
        assert_eq!(rtcm3::crc24q(&frame[..7], 0), 0x00ED_EDD6);
        let mut parser = rtcm3::Rtcm3::default();
        let read: Vec<i32> = [0xFF, 0xD3, 0x13]
            .iter()
            .chain(&frame)
            .map(|&b| parser.read(b).expect("no overflow"))
            .collect();
        // The junk D3 starts a frame of 0x3D3 bytes, which swallows the real one: a frame is
        // only found after junk that does not look like the start of one.
        assert!(read.iter().all(|&n| n == -1), "{read:?}");
        let mut parser = rtcm3::Rtcm3::default();
        let read: Vec<i32> = [0xFF, 0xFF]
            .iter()
            .chain(&frame)
            .map(|&b| parser.read(b).expect("no overflow"))
            .collect();
        assert_eq!(read.last(), Some(&1230));
        assert!(read[..read.len() - 1].iter().all(|&n| n == -1));
        assert_eq!(parser.length(), 10);
        assert_eq!(parser.packet(), &frame);

        let mut bad = frame;
        bad[9] ^= 1;
        let mut parser = rtcm3::Rtcm3::default();
        assert!(bad.iter().all(|&b| parser.read(b) == Ok(-1)));
    }

    /// Every byte of the recorded stream is in a frame the parser reads, and the messages it
    /// counts are the independent decoder's.
    #[test]
    fn every_frame_of_the_recorded_stream_is_read_and_counted() {
        let frames = frames(STREAM);
        let mut counts: BTreeMap<i32, usize> = BTreeMap::new();
        let mut framed = Vec::new();
        for (n, bytes, _) in &frames {
            *counts.entry(*n).or_default() += 1;
            framed.extend_from_slice(bytes);
        }
        assert_eq!(counts, COUNTS.into_iter().collect());
        assert_eq!(framed, STREAM);
    }

    /// The station of 1005 and 1006: its ECEF metres, and `ecef2pos` of them SITL's home.
    #[test]
    fn the_station_is_sitls_home() {
        let frames = frames(STREAM);
        for number in [1005, 1006] {
            let (_, bytes, _) = frames.iter().find(|(n, ..)| *n == number).expect("found");
            let ecef = rtcm3::station_ecef(bytes);
            let expected = [-4_471_571.438_4, 2_669_270.76, -3_671_144.536_8];
            for (got, want) in ecef.iter().zip(expected) {
                assert!((got - want).abs() < 1e-6, "{ecef:?}");
            }
            let llh = rtcm3::ecef2pos(ecef);
            assert!((llh[0] * rtcm3::R2D - HOME.0).abs() < 1e-8, "{llh:?}");
            assert!((llh[1] * rtcm3::R2D - HOME.1).abs() < 1e-8, "{llh:?}");
            assert!((llh[2] - HOME.2).abs() < 1e-3, "{llh:?}");
        }
        // And back: `pos2ecef` of a position, through `ecef2pos`, is the position.
        let pos = [-0.617_2, 2.603_4, 1234.5];
        let back = rtcm3::ecef2pos(rtcm3::pos2ecef(pos));
        for (got, want) in back.iter().zip(pos) {
            assert!((got - want).abs() < 1e-6, "{back:?}");
        }
        // The pole, where the C# has its own case.
        assert_eq!(rtcm3::ecef2pos([0.0, 0.0, 6_356_752.0])[1], 0.0);
    }

    /// What each observation message raises: its satellites and their L1 strengths - MSM4's
    /// whole dBHz, MSM7's sixteenths cut to a byte, the legacy messages' quarters - by number.
    #[test]
    fn the_observations_are_the_generators() {
        let frames = frames(STREAM);
        let first = |number: i32| {
            frames
                .iter()
                .find(|(n, ..)| *n == number)
                .and_then(|(_, _, obs)| obs.clone())
                .expect("observations")
        };
        assert_eq!(first(1074), obs_of(&GPS, 'G'));
        assert_eq!(first(1077), obs_of(&GPS, 'G'));
        // GLONASS's L2 is signal 8, which is in the 1 to 13 the C# takes for L1 whatever the
        // constellation: the last of the two is the one kept, the L2's, 3 dBHz below.
        let glonass_l2: Vec<(u8, u8)> = GLONASS.iter().map(|&(prn, cnr)| (prn, cnr - 3)).collect();
        assert_eq!(first(1084), obs_of(&glonass_l2, 'R'));
        assert_eq!(first(1087), obs_of(&glonass_l2, 'R'));
        assert_eq!(first(1094), obs_of(&GALILEO, 'E'));
        assert_eq!(first(1124), obs_of(&BEIDOU, 'B'));
        assert_eq!(first(1004), obs_of(&GPS[..5], 'G'));
        assert_eq!(first(1012), obs_of(&GLONASS[..4], 'R'));
        // The station messages raise nothing.
        assert!(
            frames
                .iter()
                .filter(|(n, ..)| *n == 1005)
                .all(|(_, _, obs)| obs.is_none())
        );
    }

    /// An MSM whose satellite and signal masks make more than 64 cells runs past the C#'s
    /// `cellmask` - the exception the read loop's `catch` swallows - and is not counted.
    #[test]
    fn an_msm_of_more_than_64_cells_is_the_csharps_exception() {
        let mut bits = Bits::default();
        bits.u(1074, 12);
        bits.u(7, 12);
        bits.u(0, 30 + 1 + 3 + 7 + 2 + 2 + 1 + 3);
        // 22 satellites, 3 signals: 66 cells.
        bits.u((1u64 << 22) - 1, 64);
        bits.u(0b111, 32);
        bits.u(0, 66 + 22 * 18);
        let frame = bits.frame();
        let mut parser = rtcm3::Rtcm3::default();
        let (last, rest) = frame.split_last().expect("bytes");
        assert!(rest.iter().all(|&b| parser.read(b) == Ok(-1)));
        assert_eq!(parser.read(*last), Err(Overflow));
        // 21 satellites of 3 signals, 63 cells, is read.
        let mut bits = Bits::default();
        bits.u(1074, 12);
        bits.u(7, 12);
        bits.u(0, 49);
        bits.u((1u64 << 21) - 1, 64);
        bits.u(0b111, 32);
        bits.u((1u64 << 63) - 1, 63);
        bits.u(0, 21 * 18 + 63 * 48);
        let mut parser = rtcm3::Rtcm3::default();
        let read: Vec<i32> = bits
            .frame()
            .iter()
            .map(|&b| parser.read(b).expect("no overflow"))
            .collect();
        assert_eq!(read.last(), Some(&1074));
        assert_eq!(parser.take_obs().map(|obs| obs.len()), Some(21));
    }

    // ---- the read loop ----

    /// The read loop over the stream: every message counted as `msgseen` names them, the whole
    /// of each frame the useful bytes, the station's position in `cs.Base` - the blank
    /// `MAVState`'s with no vehicle - and on the RTCM Base line, a light for each message and
    /// the bars for each observation message.
    #[test]
    fn the_read_loop_counts_the_stream_and_finds_the_base() {
        let shared = Shared::default();
        let (isrtcm, issbp) = worker::feed(
            &shared,
            STREAM,
            worker::Port::Other(Box::new(Loopback::pair().0)),
        );
        assert!(isrtcm && !issbp);
        let counts = shared.counts();
        let expected: BTreeMap<String, i32> = COUNTS
            .iter()
            .map(|&(n, count)| (format!("Rtcm{n}"), count as i32))
            .collect();
        assert_eq!(counts.msgseen, expected);
        assert_eq!(counts.bpsusefull, STREAM.len() as u64);
        let base = counts.base.expect("the base");
        assert!((base.lat - HOME.0).abs() < 1e-8 && (base.lng - HOME.1).abs() < 1e-8);
        // `String.Format("{0} {1} {2} - {3}", ...)`: each number at fifteen digits, then the time.
        let line = counts.status_line3.clone().expect("the base line");
        let (numbers, time) = line.split_once(" - ").expect("the time");
        let numbers: Vec<f64> = numbers
            .split(' ')
            .map(|n| n.parse().expect("a number"))
            .collect();
        assert!((numbers[0] - HOME.0).abs() < 1e-8, "{line}");
        assert!((numbers[1] - HOME.1).abs() < 1e-8, "{line}");
        assert!((numbers[2] - HOME.2).abs() < 1e-3, "{line}");
        assert_eq!(
            numbers[1].to_string().len(),
            "149.165229999677".len(),
            "{line}"
        );
        assert_eq!(time.len(), "HH:mm:ss".len(), "{line}");
        drop(counts);
        let events = shared.take_events();
        let seen = events
            .iter()
            .filter(|event| matches!(event, Event::Seen(_)))
            .count();
        assert_eq!(seen, 256);
        let obs = events
            .iter()
            .filter(|event| matches!(event, Event::Obs(_)))
            .count();
        assert_eq!(obs, 59 + 59 + 60 + 60 + 4);
    }

    /// The stream through a real link to a vehicle: the first 180 bytes raw, as the C# sends
    /// them before it has found any RTCM, then each frame - each as `InjectGpsData` cuts it
    /// into `GPS_RTCM_DATA` fragments, one sequence number a message; and the station in the
    /// vehicle's `cs.Base`.
    #[test]
    fn the_stream_is_injected_through_the_link_as_the_csharp_fragments_it() {
        use mp_mavlink::FrameDecoder;
        use mp_mavlink_dialects::all::{DIALECT, MavMessage};

        let (mut vehicle, gcs) = Loopback::pair();
        let config = mp_link::LinkConfig {
            send_heartbeat: false,
            stream_rate_hz: 0,
            ..mp_link::LinkConfig::default()
        };
        let link = mp_link::Link::from_transport(Box::new(gcs), config);
        vehicle
            .write_all(&mp_link::testing::heartbeat(0))
            .expect("heartbeat");
        let id = VehicleId::new(1, 1);
        until("the vehicle", || link.vehicles().contains(&id));

        let shared = Shared::default();
        {
            let mut route = shared.route();
            route.sender = Some(link.sender());
            route.vehicles = link.vehicles();
            route.selected = Some(id);
        }
        worker::feed(
            &shared,
            STREAM,
            worker::Port::Other(Box::new(Loopback::pair().0)),
        );

        let mut expected: Vec<Vec<u8>> = vec![STREAM[..180].to_vec()];
        expected.extend(frames(STREAM).into_iter().map(|(_, bytes, _)| bytes));
        let fragments_expected: usize = expected.iter().map(|m| m.len() / 180 + 1).sum();

        let mut decoder = FrameDecoder::new();
        let mut fragments: Vec<(u8, u8, Vec<u8>)> = Vec::new();
        let mut buf = [0u8; 4096];
        let deadline = Instant::now() + Duration::from_secs(10);
        while fragments.len() < fragments_expected && Instant::now() < deadline {
            let n = vehicle.read(&mut buf).unwrap_or(0);
            decoder.push_and_drain(&buf[..n], &DIALECT, |frame| {
                if let Some(MavMessage::GpsRtcmData(m)) =
                    MavMessage::decode(frame.msgid, frame.payload)
                {
                    fragments.push((m.flags, m.len, m.data[..usize::from(m.len)].to_vec()));
                }
            });
            if n == 0 {
                std::thread::sleep(Duration::from_millis(1));
            }
        }
        // Reassembled: a message starts at fragment 0, and all its fragments carry its sequence
        // number, one more than the last message's.
        let mut messages: Vec<Vec<u8>> = Vec::new();
        let mut last_seq: Option<u8> = None;
        for (flags, _, data) in &fragments {
            let (fragment, seq) = ((flags >> 1) & 0x3, flags >> 3);
            if fragment == 0 {
                if let Some(last) = last_seq {
                    assert_eq!(seq, (last + 1) & 0x1f, "sequence");
                }
                last_seq = Some(seq);
                messages.push(Vec::new());
            } else {
                assert_eq!(Some(seq), last_seq);
            }
            messages
                .last_mut()
                .expect("a message")
                .extend_from_slice(data);
        }
        assert_eq!(messages.len(), expected.len());
        assert_eq!(messages, expected);
        let counts = shared.counts();
        assert_eq!(
            counts.injected,
            expected.iter().map(|m| m.len() as u64).sum::<u64>()
        );
        assert_eq!(counts.injected_messages, fragments_expected as u64);
        drop(counts);
        until("the base in the vehicle's state", || {
            link.vehicle(id)
                .is_some_and(|handle| (handle.load().base.lat - HOME.0).abs() < 1e-8)
        });
    }

    /// A connection's thread on a port already open: `DoConnect`'s end reported, what it reads
    /// logged to the `.gpsbase` file whole and counted, until it is stopped.
    #[test]
    fn a_connection_reads_logs_and_counts_until_stopped() {
        let (mut base, port) = Loopback::pair();
        let shared = Arc::new(Shared::default());
        let dir = std::env::temp_dir().join(format!(
            "headless-planner-rtk-{}-{}",
            std::process::id(),
            line!()
        ));
        std::fs::create_dir_all(&dir).expect("the log directory");
        let spec = OpenSpec {
            kind: Kind::TcpClient,
            name: String::new(),
            baud: 115_200,
            host: "127.0.0.1".to_owned(),
            port: "1".to_owned(),
            url: String::new(),
            ntrip: mp_transport::NtripOptions::default(),
            cancelled: false,
        };
        let mut worker = worker::Worker::start_on(
            spec,
            Setup::default(),
            Arc::clone(&shared),
            Some(dir.clone()),
            Some(worker::Port::Other(Box::new(port))),
        );
        until("the loop", || worker.is_running());
        assert!(worker.is_open());
        base.write_all(STREAM).expect("the stream");
        until("every message", || {
            shared.counts().msgseen.values().sum::<i32>() == 256
        });
        assert!(matches!(shared.take_events().first(), Some(Event::Started)));
        shared
            .flush
            .store(true, std::sync::atomic::Ordering::Release);
        let log = shared.counts().log.clone().expect("the .gpsbase file");
        assert_eq!(log.extension().and_then(|e| e.to_str()), Some("gpsbase"));
        until("the log flushed", || {
            std::fs::read(&log).is_ok_and(|bytes| bytes == STREAM)
        });
        assert_eq!(shared.counts().logged, STREAM.len() as u64);
        worker.stop();
        assert!(!worker.is_open());
        until("the loop to end", || !worker.is_running());
        let _ = std::fs::remove_dir_all(&dir);
    }

    // ---- sbp.cs, nmea.cs ----

    /// `Crc16Ccitt` from zeros is CRC-16/XMODEM, whose check value for "123456789" is 0x31C3;
    /// an SBP message with its CRC is found, its type returned, its bytes its packet.
    #[test]
    fn an_sbp_message_is_found_by_its_crc() {
        let mut parser = sbp::Sbp::default();
        let mut message = vec![0x55, 0x02, 0x02, 0x34, 0x12, 0x03, 0xAA, 0xBB, 0xCC];
        // The CRC over everything after the preamble, as `read` accumulates it.
        let crc = crc16(&message[1..]);
        assert_eq!(crc16(b"123456789"), 0x31C3);
        message.extend_from_slice(&crc.to_le_bytes());
        let read: Vec<i32> = message.iter().map(|&b| parser.read(b)).collect();
        assert_eq!(read.last(), Some(&0x0202));
        assert!(read[..read.len() - 1].iter().all(|&n| n == -1));
        assert_eq!(parser.length(), 11);
        assert_eq!(parser.packet(), message.as_slice());
        message[6] ^= 1;
        let mut parser = sbp::Sbp::default();
        assert!(message.iter().all(|&b| parser.read(b) == -1));
    }

    /// CRC-16/XMODEM bit by bit.
    fn crc16(bytes: &[u8]) -> u16 {
        let mut crc: u16 = 0;
        for &b in bytes {
            crc ^= u16::from(b) << 8;
            for _ in 0..8 {
                crc = if crc & 0x8000 != 0 {
                    (crc << 1) ^ 0x1021
                } else {
                    crc << 1
                };
            }
        }
        crc
    }

    /// A sentence with its checksum is found at its line end - and, as the C# leaves its
    /// parser, the next is read onto the end of it and checked against the first one's
    /// checksum: a different sentence is missed and puts the parser back, so the one after it
    /// is found.
    #[test]
    fn nmea_sentences_are_found_every_other_one() {
        // The NMEA 0183 standard's GGA example, whose checksum is 47.
        let sentence = b"$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
        assert_eq!(
            nmea::checksum("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47"),
            "47"
        );
        let body = "$GPGLL,4916.45,N,12311.12,W,225444,A,";
        let other = format!("{body}*{}\r\n", nmea::checksum(body));
        let mut parser = nmea::Nmea::default();
        let found: Vec<i32> = [&sentence[..], other.as_bytes(), &sentence[..]]
            .iter()
            .map(|bytes| bytes.iter().map(|&b| parser.read(b)).max().unwrap_or(-1))
            .collect();
        assert_eq!(found, [1, -1, 1]);
        // A reset between them, as another parser's message makes, finds both.
        let mut parser = nmea::Nmea::default();
        for _ in 0..2 {
            assert_eq!(sentence.iter().map(|&b| parser.read(b)).max(), Some(1));
            parser.reset_parser();
        }
        let mut bad = sentence.to_vec();
        bad[10] = b'9';
        let mut parser = nmea::Nmea::default();
        assert!(bad.iter().all(|&b| parser.read(b) == -1));
    }

    // ---- ubx_m8p.cs ----

    /// A receiver that records what it is sent, the baud rates it is set to and the sleeps.
    #[derive(Default)]
    struct Recording {
        writes: Vec<Vec<u8>>,
        bauds: Vec<u32>,
        baud: u32,
        slept: Duration,
        /// Lines to answer with; with none, each command is echoed.
        silent: bool,
        lines: std::collections::VecDeque<String>,
    }

    impl ubx::Receiver for Recording {
        fn write(&mut self, bytes: &[u8]) -> std::io::Result<()> {
            self.writes.push(bytes.to_vec());
            if !self.silent {
                let text = String::from_utf8_lossy(bytes);
                self.lines.push_back("garbage".to_owned());
                self.lines
                    .push_back(format!("$R: {}", text.trim_end_matches(['\r', '\n'])));
            }
            Ok(())
        }
        fn baud(&self) -> u32 {
            self.baud
        }
        fn set_baud(&mut self, baud: u32) {
            self.baud = baud;
            self.bauds.push(baud);
        }
        fn sleep(&mut self, time: Duration) {
            self.slept += time;
        }
    }

    impl septentrio::Answering for Recording {
        fn read_line(&mut self, _wait: Duration) -> Option<String> {
            self.lines.pop_front()
        }
    }

    /// `generate`'s Fletcher checksum: the CFG-MSG that turns GGA off is, as u-blox's own
    /// manuals print it, B5 62 06 01 08 00 F0 00 00 00 00 00 00 00 FF 23; and the parser
    /// finds it, as class 6, id 1.
    #[test]
    fn a_ubx_message_is_generated_and_found() {
        let message = ubx::generate(0x06, 0x01, &[0xF0, 0x00, 0, 0, 0, 0, 0, 0]);
        assert_eq!(
            message,
            [
                0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0xFF, 0x23
            ]
        );
        let mut parser = ubx::Ubx::default();
        let read: Vec<i32> = message
            .iter()
            .map(|&b| parser.read(b).expect("no overflow"))
            .collect();
        assert_eq!(read.last(), Some(&0x0601));
        assert_eq!((parser.class(), parser.subclass()), (6, 1));
        // A poll - no payload - leaves the C#'s parser waiting at step 6, so the next message
        // is missed; a reset, which another parser's message makes, frees it.
        let poll = ubx::generate(0x0a, 0x04, &[]);
        let mut parser = ubx::Ubx::default();
        let found = |parser: &mut ubx::Ubx, bytes: &[u8]| {
            bytes
                .iter()
                .map(|&b| parser.read(b).expect("no overflow"))
                .max()
        };
        assert_eq!(found(&mut parser, &poll), Some(-1));
        assert_eq!(found(&mut parser, &message), Some(-1));
        parser.reset_parser();
        assert_eq!(found(&mut parser, &message), Some(0x0601));
    }

    /// TMODE3 as the C#'s structure packs it, worked by hand from its constructor: a fixed
    /// position in degrees, one in ECEF, the survey in, and the disabled one.
    /// `// C#: ExtLibs/Utilities/ubx_m8p.cs:317-411`
    #[test]
    fn tmode3_is_packed_as_the_csharp_packs_it() {
        let hex = |bytes: &[u8]| bytes.iter().map(|b| format!("{b:02x}")).collect::<String>();
        let fixed = ubx::Tmode3::fixed(-35.363_261_23, 149.165_234_56, 584.123);
        assert_eq!(
            (fixed.flags, fixed.position, fixed.position_hp),
            (258, [-353_632_612, 1_491_652_345, 58412], [-30, 59, 30])
        );
        assert_eq!(
            hex(&fixed.bytes()),
            "000002019cfeebeaf9cee8582ce40000e23b1e00010000003c000000d00700000000000000000000"
        );
        let ecef = ubx::Tmode3::fixed(-4_471_571.438_4, 2_669_270.76, -3_671_144.536_8);
        assert_eq!(
            hex(&ecef.bytes()),
            "0000020069ec58e5e3fbe80f2b471eeaac63bc00010000003c000000d00700000000000000000000"
        );
        assert_eq!(ecef.mode_name(), "FixedECEF");
        let [x, _, _] = ecef.point().expect("a point");
        assert!((x - -4_471_571.438_4).abs() < 1e-3, "{x}");
        assert_eq!(fixed.mode_name(), "FixedLLA");
        let survey = ubx::Tmode3::survey_in(60, 2.0);
        assert_eq!(
            hex(&survey.bytes()),
            "0000010000000000000000000000000000000000000000003c000000204e00000000000000000000"
        );
        assert_eq!(survey.point(), None);
        assert_eq!(ubx::Tmode3::default().mode_name(), "Disabled");
        // Read back from a CFG-TMODE3 message, as `ProcessUBXMessage` reads it.
        let message = ubx::generate(0x06, 0x71, &fixed.bytes());
        assert_eq!(ubx::Tmode3::of(&message), fixed);
    }

    /// `SetupBasePos`: disabling writes TMODE3 off, saves and restarts; a zero position asks
    /// for a survey in of 60 s to 2 m when given 0 and 0; a position fixes it.
    #[test]
    fn setup_base_pos_writes_what_the_csharp_writes() {
        let zero = ubx::Position {
            zero: true,
            ..ubx::Position::default()
        };
        let mut port = Recording::default();
        ubx::setup_base_pos(&mut port, zero, 0, 0.0, true).expect("written");
        assert_eq!(
            port.writes,
            [
                ubx::generate(0x06, 0x71, &[0; 40]),
                ubx::generate(0x06, 0x09, &[0, 0, 0, 0, 0xff, 0xff, 0, 0, 0, 0, 0, 0, 1]),
                ubx::generate(0x06, 0x04, &[0x14, 0xff, 2, 0]),
            ]
        );
        assert_eq!(port.slept, Duration::from_millis(4200));
        let mut port = Recording::default();
        ubx::setup_base_pos(&mut port, zero, 0, 0.0, false).expect("written");
        assert_eq!(
            port.writes,
            [ubx::generate(
                0x06,
                0x71,
                &ubx::Tmode3::survey_in(60, 2.0).bytes()
            )]
        );
        let mut port = Recording::default();
        let here = ubx::Position {
            lat: -35.363_261_23,
            lng: 149.165_234_56,
            alt: 584.123,
            zero: false,
        };
        ubx::setup_base_pos(&mut port, here, 120, 0.5, false).expect("written");
        assert_eq!(
            port.writes,
            [ubx::generate(
                0x06,
                0x71,
                &ubx::Tmode3::fixed(here.lat, here.lng, here.alt).bytes()
            )]
        );
    }

    /// `SetupM8P`: UART1 to 460800 at the port's rate and six others, then 460800; fifty
    /// messages; MSM7 before firmware 1.30, MSM4 with it.
    #[test]
    fn setup_m8p_writes_what_the_csharp_writes() {
        for (m8p_130p, msm4, msm7) in [(false, 0, 1), (true, 1, 0)] {
            let mut port = Recording {
                baud: 57_600,
                ..Recording::default()
            };
            ubx::setup_m8p(&mut port, m8p_130p).expect("written");
            assert_eq!(
                port.bauds,
                [
                    57_600, 9600, 38400, 57_600, 115_200, 230_400, 460_800, 460_800
                ]
            );
            assert_eq!(port.writes.len(), 50);
            assert_eq!(port.writes[0], b"UU");
            let rate = |class: u8, id: u8| {
                port.writes
                    .iter()
                    .find(|w| w.get(2..8) == Some(&[0x06, 0x01, 0x08, 0x00, class, id]))
                    .map(|w| w[9])
            };
            assert_eq!(rate(0xf5, 0x4a), Some(msm4), "1074");
            assert_eq!(rate(0xf5, 0x4d), Some(msm7), "1077");
            assert_eq!(rate(0xf5, 0x05), Some(5), "1005");
            assert_eq!(rate(0xf0, 0x00), Some(0), "GGA off");
            assert_eq!(rate(0xf0, 0x0b), None, "0x0b is left alone");
        }
    }

    // ---- Septentrio.cs, Unicore.cs ----

    /// Septentrio's commands, each waited for until it is echoed: configured at 115200, the
    /// RTCM chosen and its interval; a receiver that says nothing fails its acknowledgement at
    /// every baud rate.
    #[test]
    fn septentrio_commands_are_the_csharps_and_wait_for_their_echo() {
        let mut port = Recording::default();
        septentrio::configure_base_receiver(&mut port).expect("acknowledged");
        let sent: Vec<String> = port
            .writes
            .iter()
            .map(|w| String::from_utf8_lossy(w).into_owned())
            .collect();
        assert_eq!(
            sent,
            [
                "setCOMSettings,COM1+COM2+COM3,baud115200,bits8,No,bit1,none\n",
                "setPVTMode,Static,All,Auto\n",
                "setDataInOut,USB1+USB2+COM1+COM2+COM3,Auto,RTCMv3\n",
            ]
        );
        assert_eq!(port.bauds, [115_200, 115_200, 115_200]);

        let mut port = Recording::default();
        let all = septentrio::Signals {
            gps: true,
            glonass: true,
            beidou: true,
            galileo: true,
        };
        septentrio::set_enabled_rtcm(&mut port, 4, all).expect("acknowledged");
        septentrio::set_rtcm_interval(&mut port, 1.0).expect("acknowledged");
        septentrio::set_base_position(&mut port, -35.363_261, 149.165_23, 584.0)
            .expect("acknowledged");
        let sent: Vec<String> = port
            .writes
            .iter()
            .map(|w| String::from_utf8_lossy(w).into_owned())
            .collect();
        assert_eq!(
            sent,
            [
                "setRTCMv3Output,COM1+COM2+COM3+USB1+USB2,RTCM1006+RTCM1033+RTCM1230+RTCM1074+RTCM1084+RTCM1094+RTCM1124\n",
                "setRTCMv3Interval,MSM3+MSM4+MSM7+RTCM1005|6+RTCM1033+RTCM1230,1.0\n",
                "setStaticPosGeodetic,Geodetic1,-35.363260000,149.165200000,584.0000,WGS84\n",
                "setPVTMode,Static,,Geodetic1\n",
            ]
        );

        let mut silent = Recording {
            silent: true,
            ..Recording::default()
        };
        assert_eq!(
            septentrio::configure_base_receiver(&mut silent),
            Err(septentrio::Failure::FailedAck)
        );
        assert_eq!(silent.writes.len(), 11, "one try at each baud rate");
    }

    /// Unicore's commands, written at 115200 and not waited for.
    #[test]
    fn unicore_commands_are_the_csharps() {
        let mut port = Recording {
            silent: true,
            ..Recording::default()
        };
        septentrio::configure_unicore(&mut port).expect("written");
        assert_eq!(port.bauds, [115_200]);
        assert_eq!(port.writes.len(), 9);
        assert_eq!(port.writes[1], b"mode base time 60 2 2.5\r\n");
        assert_eq!(port.writes[8], b"saveconfig\r\n");
    }

    /// Each handler's box for how its command went.
    #[test]
    fn each_handler_says_what_its_catch_says() {
        use septentrio::Failure;
        let ack = Err(Failure::FailedAck);
        assert_eq!(
            done_message(Handler::Connect, &ack).as_deref(),
            Some("Automatic configuration of Septentrio receiver failed.")
        );
        assert_eq!(
            done_message(Handler::Connect, &Err(Failure::Format(String::new()))).as_deref(),
            Some("Septentrio fixed base position is invalid.")
        );
        assert_eq!(
            done_message(Handler::Amount, &ack).as_deref(),
            Some("Configuration of fixed position on Septentrio receiver failed.")
        );
        assert_eq!(
            done_message(Handler::Interval, &ack).as_deref(),
            Some("Configuration of RTCM interval on Septentrio receiver failed.")
        );
        let range = Err(Failure::InvalidOperation(
            "The RTCM interval should be between 0,1 and 600".to_owned(),
        ));
        assert_eq!(
            done_message(Handler::Interval, &range).as_deref(),
            Some("The RTCM interval should be between 0,1 and 600")
        );
        assert_eq!(done_message(Handler::FixedPosition, &range), None);
        assert_eq!(done_message(Handler::Constellation, &ack), None);
        assert_eq!(done_message(Handler::Interval, &Ok(())), None);
        // The texts are the C#'s.
        if let Some(source) = crate::config_coverage::source::csharp(
            "GCSViews/ConfigurationView/ConfigSerialInjectGPS.cs",
        ) {
            for handler in [Handler::Connect, Handler::Amount, Handler::Interval] {
                let text = done_message(handler, &ack).expect("a box");
                assert!(source.contains(&format!("\"{text}\"")), "{text}");
            }
            assert!(source.contains("\"The RTCM interval should be between 0,1 and 600\""));
        }
    }

    // ---- the page ----

    /// The `.resx`'s places and texts, the Designer's parents, the lists' items and the grid's
    /// columns.
    #[test]
    fn the_controls_are_the_resx() {
        let (Some(resx), Some(designer)) = (
            crate::config_coverage::source::csharp(
                "GCSViews/ConfigurationView/ConfigSerialInjectGPS.resx",
            ),
            crate::config_coverage::source::csharp(
                "GCSViews/ConfigurationView/ConfigSerialInjectGPS.Designer.cs",
            ),
        ) else {
            eprintln!("skipped: the C# tree is not checked out");
            return;
        };
        let values = crate::config_coverage::source::resx(&resx);
        let get = |key: String| values.get(&key).map(|v| v.replace("\r\n", "\n"));
        let pair = |a: f32, b: f32| format!("{a}, {b}");
        for control in CONTROLS {
            let (x, y, w, h) = control.place;
            assert_eq!(
                get(format!("{}.Location", control.name)),
                Some(pair(x, y)),
                "{}",
                control.name
            );
            assert_eq!(
                get(format!("{}.Size", control.name)),
                Some(pair(w, h)),
                "{}",
                control.name
            );
            if !control.text.is_empty() {
                assert_eq!(
                    get(format!("{}.Text", control.name)).as_deref(),
                    Some(control.text),
                    "{}",
                    control.name
                );
            }
            let parent = match control.parent {
                "" => "this".to_owned(),
                PANEL2 => "this.splitContainer1.Panel2".to_owned(),
                "splitContainer1" => "this.splitContainer1.Panel1".to_owned(),
                other => format!("this.{other}"),
            };
            assert!(
                designer.contains(&format!("{parent}.Controls.Add(this.{});", control.name)),
                "{} in {parent}",
                control.name
            );
        }
        let items = |name: &str, count: usize| -> Vec<String> {
            (0..count)
                .map(|i| {
                    let suffix = if i == 0 { String::new() } else { i.to_string() };
                    get(format!("{name}.Items{suffix}")).expect("an item")
                })
                .collect()
        };
        assert_eq!(items("CMB_baudrate", 13), BAUDS);
        assert_eq!(items("comboBoxConfigType", 3), CONFIG_TYPES);
        assert_eq!(items("cmb_septentriortcmamount", 3), AMOUNTS);
        for (name, header, width) in COLUMNS {
            assert_eq!(get(format!("{name}.HeaderText")).as_deref(), Some(header));
            match get(format!("{name}.Width")) {
                Some(set) => assert_eq!(set, format!("{width}")),
                None => assert_eq!(width, 100.0, "{name}: the default width"),
            }
        }
        assert_eq!(
            get("$this.Size".to_owned()),
            Some(pair(PAGE_SIZE.0, PAGE_SIZE.1))
        );
        assert_eq!(
            get("chk_rtcmmsg.Visible".to_owned()).as_deref(),
            Some("False")
        );
        assert_eq!(
            get("comboBoxConfigType.Visible".to_owned()).as_deref(),
            Some("False")
        );
        assert_eq!(
            get("button_septentriosetposition.Enabled".to_owned()).as_deref(),
            Some("False")
        );
        // The strings the page shows are the C#'s.
        let source = crate::config_coverage::source::csharp(
            "GCSViews/ConfigurationView/ConfigSerialInjectGPS.cs",
        )
        .expect("the page's source");
        for extra in PORT_EXTRAS {
            assert!(source.contains(&format!("CMB_serialport.Items.Add(\"{extra}\");")));
        }
        assert!(source.contains("\"Enter a friendly name for this location.\""));
        assert!(source.contains(
            "\"Error Connecting\\nif using com0com please rename the ports to COM??\\n\""
        ));
    }

    /// Where the controls land: a group's labels offset by the group, the survey-in group by
    /// its panel and the split container, the bars under the options when they show.
    #[test]
    fn places_are_their_parents_added_up() {
        assert_eq!(absolute("labelbase", true), (255.0, 89.0, 20.0, 20.0));
        assert_eq!(absolute("lbl_svin", false), (558.0, 214.0, 184.0, 13.0));
        assert_eq!(absolute("panel1", true), (6.0, 186.0, 740.0, 200.0));
        assert_eq!(absolute("panel1", false), (6.0, 375.0, 740.0, 186.0));
    }

    /// Every one of the Designer's 24 wirings, and what stands for it here.
    const WIRINGS: [(&str, &str); 24] = [
        ("CMB_serialport_SelectedIndexChanged", "fn port_changed"),
        ("timer1_Tick", "fn timer_tick"),
        ("chk_rtcmmsg_CheckedChanged", "fn rtcmmsg_changed"),
        ("chk_autoconfig_CheckedChanged", "fn autoconfig_changed"),
        (
            "chk_septentrioconstellation_Click",
            "Handler::Constellation",
        ),
        (
            "chk_septentrioconstellation_Click",
            "Handler::Constellation",
        ),
        (
            "chk_septentrioconstellation_Click",
            "Handler::Constellation",
        ),
        (
            "chk_septentrioconstellation_Click",
            "Handler::Constellation",
        ),
        (
            "cmb_septentriortcmamount_SelectedIndexChanged",
            "Handler::Amount",
        ),
        (
            "chk_septentriofixedposition_Click",
            "Handler::FixedPosition",
        ),
        (
            "chk_m8p_130p_CheckedChanged",
            "\"SerialInjectGPS_m8p_130p\"",
        ),
        ("txt_surveyinAcc_TextChanged", "\"SerialInjectGPS_SIAcc\""),
        ("txt_surveyinDur_TextChanged", "\"SerialInjectGPS_SITime\""),
        ("labelmsgseen_Click", "fn msgseen_click"),
        (
            "comboBoxConfigType_SelectedIndexChanged",
            "fn config_type_changed",
        ),
        ("button_septentriortcminterval_Click", "Handler::Interval"),
        ("button_septentriosetposition_Click", "Handler::SetPosition"),
        ("but_restartsvin_Click", "fn restart_click"),
        ("but_save_basepos_Click", "fn save_click"),
        ("BUT_connect_Click", "fn connect_click"),
        ("dg_basepos_CellContentClick", "fn cell_click"),
        ("dg_basepos_CellEndEdit", "fn cell_end_edit"),
        ("dg_basepos_DefaultValuesNeeded", "`DefaultValuesNeeded`"),
        ("dg_basepos_RowsRemoved", "fn rows_removed"),
    ];

    #[test]
    fn every_wiring_is_handled() {
        let own = include_str!("rtk_inject.rs");
        for (handler, ours) in WIRINGS {
            assert!(own.contains(ours), "{handler}: {ours}");
        }
        let Some(designer) = crate::config_coverage::source::csharp(
            "GCSViews/ConfigurationView/ConfigSerialInjectGPS.Designer.cs",
        ) else {
            eprintln!("skipped: the C# tree is not checked out");
            return;
        };
        let mut wired: Vec<&str> = designer
            .lines()
            .filter(|line| line.contains(" += new "))
            .filter_map(|line| line.split("(this.").nth(1)?.split(')').next())
            .collect();
        let mut ours: Vec<&str> = WIRINGS.iter().map(|(handler, _)| *handler).collect();
        wired.sort_unstable();
        ours.sort_unstable();
        assert_eq!(wired, ours);
    }

    /// Every fact the script asserts on is recorded here, and every control it clicks is drawn.
    #[test]
    fn the_gui_script_names_facts_and_controls_this_page_has() {
        let script = include_str!("../../../../tests/gui/config-rtk.gui");
        let source = include_str!("rtk_inject.rs");
        let (mut facts, mut clicks) = (0, 0);
        for line in script.lines() {
            let line = line.split('#').next().unwrap_or("");
            let mut words = line.split_whitespace();
            match (words.next(), words.next()) {
                (Some("expect"), Some(key)) if key.starts_with("config.rtk.") => {
                    assert!(source.contains(&format!("\"{key}\"")), "{key}");
                    facts += 1;
                }
                (Some("click"), Some(id)) if id.starts_with("rtk-") => {
                    assert!(source.contains(&format!("\"{id}\"")), "{id}");
                    clicks += 1;
                }
                _ => {}
            }
        }
        assert!(facts >= 12, "{facts} facts");
        assert!(clicks >= 3, "{clicks} clicks");
        // The caster it starts serves the recorded stream.
        assert!(script.contains("testdata/rtcm/base-station.rtcm3"));
    }

    /// `seenRTCM`'s cases: the message numbers each light answers to, and for how long, are
    /// the C#'s switch.
    #[test]
    fn each_light_answers_to_the_csharps_messages() {
        let Some(source) = crate::config_coverage::source::csharp(
            "GCSViews/ConfigurationView/ConfigSerialInjectGPS.cs",
        ) else {
            eprintln!("skipped: the C# tree is not checked out");
            return;
        };
        let start = source
            .find("private static void seenRTCM")
            .expect("seenRTCM");
        let body = &source[start..start + source[start..].find("ProcessUBXMessage").expect("end")];
        let mut cases: Vec<i32> = Vec::new();
        for line in body.lines() {
            let line = line.trim();
            if let Some(number) = line
                .strip_prefix("case ")
                .and_then(|rest| rest.split(':').next())
            {
                cases.push(number.trim().parse().expect("a number"));
                continue;
            }
            for ((name, _), light) in LIGHTS.iter().zip(0..) {
                if line.starts_with(&format!("ExpireType.Set(Instance.{name}, ")) {
                    let seconds: u64 = line
                        .trim_start_matches(&format!("ExpireType.Set(Instance.{name}, "))
                        .trim_end_matches(");")
                        .parse()
                        .expect("seconds");
                    for case in cases.drain(..) {
                        assert_eq!(light_for(case), Some((light, seconds)), "{case}");
                    }
                }
            }
        }
        assert_eq!(light_for(1033), None);
        assert_eq!(light_for(1230), None);
    }

    fn view() -> TelemetryView {
        TelemetryView::disconnected("")
    }

    fn scratch(tag: &str) -> std::path::PathBuf {
        let dir =
            std::env::temp_dir().join(format!("headless-planner-rtk-{}-{tag}", std::process::id()));
        std::fs::create_dir_all(&dir).expect("scratch");
        dir
    }

    /// A page object on a dictionary and list file of the test's own.
    fn page_on(persisted: &mut Persisted, list: std::path::PathBuf) -> RtkInject {
        let mut rtk = RtkInject {
            list_file: Some(list),
            ..RtkInject::default()
        };
        rtk.activate(Key::of(&view()), persisted, None);
        rtk
    }

    /// The constructor restores each control from `Settings.Instance` as its handler would,
    /// and the base positions from their file and `base_pos`.
    #[test]
    fn the_constructor_restores_the_saved_choices() {
        let dir = scratch("restore");
        let list = dir.join("baseposlist.xml");
        std::fs::write(
            &list,
            basepos_list_xml(&[BasePos {
                lat: -35.1,
                lng: 149.2,
                alt: 580.5,
                tag: "field <A>".to_owned(),
            }]),
        )
        .expect("the list");
        let mut persisted = Persisted::at(None);
        for (key, value) in [
            ("SerialInjectGPS_port", "NTRIP"),
            ("SerialInjectGPS_baud", "57600"),
            ("SerialInjectGPS_autoconfig", "True"),
            ("SerialInjectGPS_AutoConfigType", "Septentrio"),
            ("SerialInjectGPS_SeptentrioGLONASS", "False"),
            ("SerialInjectGPS_SeptentrioRTCMLevel", "2"),
            ("SerialInjectGPS_SIAcc", "1.5"),
            ("base_pos", "-35.25,149.5,600,home"),
        ] {
            persisted.set(key, value);
        }
        let rtk = page_on(&mut persisted, list.clone());
        let page = rtk.page.as_ref().expect("a page");
        assert_eq!(page.ports.text(), "NTRIP");
        assert!(!page.baud.enabled, "a network port has no baud rate");
        assert_eq!(page.baud.text(), "57600");
        assert!(page.autoconfig);
        assert_eq!(page.panels, [false, true, false]);
        assert!(!page.signals.glonass && page.signals.gps);
        assert_eq!(page.amount.text(), "Full");
        assert_eq!(page.survey_acc.value(), "1.5");
        assert_eq!(page.survey_dur.value(), "60");
        assert_eq!(
            page.grid,
            [["-35.1", "149.2", "580.5", "field <A>"].map(str::to_owned)]
        );
        assert_eq!(
            page.basepos,
            BasePos {
                lat: -35.25,
                lng: 149.5,
                alt: 600.0,
                tag: "home".to_owned(),
            }
        );
        assert_eq!(page.connect, (CONNECT, true));
        assert_eq!(
            persisted.get("SerialInjectGPS_SeptentrioRTCMLevel"),
            Some("2")
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// Connect on NTRIP asks for the caster's URL, the saved one in the box; cancelled, it is
    /// `DoConnect`'s box with the exception's text. On TCP it asks for the host and then the
    /// port. With no port chosen, and with no baud rate, the C#'s boxes.
    #[test]
    fn connect_asks_open_s_questions() {
        let dir = scratch("connect");
        let mut persisted = Persisted::at(None);
        persisted.set("SerialInjectGPS_port", "NTRIP");
        persisted.set("NTRIP_url", "ntrip://user:pass@127.0.0.1:2101/MOUNT");
        persisted.set("logdirectory", dir.display().to_string());
        let mut rtk = page_on(&mut persisted, dir.join("list.xml"));
        rtk.connect_click(&mut persisted, (-35.0, 149.0, 500.0));
        let prompt = rtk.prompt.as_ref().expect("the question");
        assert_eq!(prompt.title, "remote host");
        assert_eq!(
            prompt.field.value(),
            "ntrip://user:pass@127.0.0.1:2101/MOUNT"
        );
        assert_eq!(rtk.pending.as_ref().map(|spec| spec.ntrip.lat), Some(-35.0));
        let page = rtk.page.as_ref().expect("a page");
        assert_eq!(page.baud.text(), "2400", "`CMB_baudrate.SelectedIndex = 0`");
        assert!(!page.sendgga.1);
        assert_eq!(persisted.get("SerialInjectGPS_baud"), Some("2400"));
        rtk.answer(false, &mut persisted, LatLngAlt::ZERO);
        assert_eq!(
            rtk.messages.pop_front().map(|m| m.text),
            Some(error_connecting(CANCELED))
        );
        assert!(rtk.worker.is_none());
        // `InputBox` keeps an answer on OK only (InputBox.cs:178-184).
        assert_eq!(
            persisted.get("InputBoxremotehostEnterurleghttpuserpasshostportmount"),
            None
        );

        // TCP: host, then port; the port's cancel is the same box.
        if let Some(page) = rtk.page.as_mut() {
            select_text(&mut page.ports, "TCP Client");
        }
        rtk.connect_click(&mut persisted, (0.0, 0.0, 0.0));
        assert_eq!(
            rtk.prompt.as_ref().map(|p| p.field.value()),
            Some("127.0.0.1")
        );
        rtk.answer(true, &mut persisted, LatLngAlt::ZERO);
        // The host's OK kept, as `InputBox` keeps every titled answer, before the port is asked.
        assert_eq!(
            persisted.get("InputBoxremotehostEnterhostnameipensureremoteendisalreadystarted"),
            Some("127.0.0.1")
        );
        assert_eq!(rtk.prompt.as_ref().map(|p| p.title), Some("remote Port"));
        assert_eq!(rtk.prompt.as_ref().map(|p| p.field.value()), Some("5760"));
        rtk.answer(false, &mut persisted, LatLngAlt::ZERO);
        assert_eq!(
            rtk.messages.pop_front().map(|m| m.text),
            Some(error_connecting(CANCELED))
        );
        assert_eq!(
            persisted.get("TCP_host"),
            None,
            "saved only once both are answered"
        );
        assert_eq!(persisted.get("InputBoxremotePortEnterremoteport"), None);

        // UDP Host's cancel opens nothing and carries on: the thread runs with no port.
        if let Some(page) = rtk.page.as_mut() {
            select_text(&mut page.ports, "UDP Host");
        }
        rtk.connect_click(&mut persisted, (0.0, 0.0, 0.0));
        assert_eq!(rtk.prompt.as_ref().map(|p| p.title), Some("Listern Port"));
        rtk.answer(false, &mut persisted, LatLngAlt::ZERO);
        until("the loop", || rtk.is_running());
        assert!(!rtk.is_open());
        rtk.take_events(&mut persisted);
        assert_eq!(
            rtk.page.as_ref().map(|page| page.connect),
            Some((STOP, true))
        );
        // Not open, so Connect asks again - and first ends the thread.
        rtk.connect_click(&mut persisted, (0.0, 0.0, 0.0));
        assert!(rtk.prompt.is_some());
        assert!(rtk.worker.is_none());

        // No port chosen: `PortName = ""` throws; no baud rate: `int.Parse` does.
        rtk.prompt = None;
        if let Some(page) = rtk.page.as_mut() {
            page.ports.selected = None;
        }
        rtk.connect_click(&mut persisted, (0.0, 0.0, 0.0));
        assert_eq!(
            rtk.messages.pop_front().map(|m| m.text),
            Some(INVALID_PORT_NAME.to_owned())
        );
        if let Some(page) = rtk.page.as_mut() {
            page.ports.options.push((99, "/dev/ttyRTK".to_owned()));
            page.ports.select(99);
            page.baud.selected = None;
        }
        rtk.connect_click(&mut persisted, (0.0, 0.0, 0.0));
        assert_eq!(
            rtk.messages.pop_front().map(|m| m.text),
            Some(INVALID_BAUD_RATE.to_owned())
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// Every question the page asks keeps its OK's answer under `InputBox`'s key for it: the
    /// caption and question with all but letters and digits taken out. The facts publish each.
    /// `// C#: Program.cs:564-566; ExtLibs/Controls/InputBox.cs:73-84, 178-184`
    #[test]
    fn each_question_keeps_its_ok_answer_under_the_input_box_key() {
        let questions = [
            (
                "remote host",
                "Enter url (eg http://user:pass@host:port/mount)",
                "InputBoxremotehostEnterurleghttpuserpasshostportmount",
            ),
            (
                "remote host",
                "Enter host name/ip (ensure remote end is already started)",
                "InputBoxremotehostEnterhostnameipensureremoteendisalreadystarted",
            ),
            (
                "remote Port",
                "Enter remote port",
                "InputBoxremotePortEnterremoteport",
            ),
            (
                "Listern Port",
                "Enter Local port (ensure remote end is already sending)",
                "InputBoxListernPortEnterLocalportensureremoteendisalreadysending",
            ),
            (
                "Enter Location",
                "Enter a friendly name for this location.",
                "InputBoxEnterLocationEnterafriendlynameforthislocation",
            ),
        ];
        let source = include_str!("rtk_inject.rs");
        for (title, question, key) in questions {
            assert!(source.contains(&format!("{question:?}")), "{question}");
            assert_eq!(crate::config::optional::answers_key(title, question), key);
            assert!(crate::settings::PUBLISHED.contains(&key), "{key}");
        }
        let dir = scratch("answers");
        let mut persisted = Persisted::at(None);
        let mut rtk = page_on(&mut persisted, dir.join("list.xml"));
        rtk.save_click();
        rtk.answer(false, &mut persisted, LatLngAlt::ZERO);
        assert_eq!(persisted.get(questions[4].2), None, "Cancel keeps nothing");
        rtk.save_click();
        if let Some(prompt) = rtk.prompt.as_mut() {
            prompt.field.set("a, b");
        }
        rtk.answer(true, &mut persisted, LatLngAlt::ZERO);
        assert_eq!(persisted.get(questions[4].2), Some("a%2C+b"));
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// The grid: Save Current Position asks for a name and adds `cs.Base`; Use makes a row the
    /// base position; an edited cell goes into the list, a number that does not parse does
    /// not; Delete removes a row; and the list is saved as the C#'s `XmlSerializer` writes it.
    #[test]
    fn base_positions_are_saved_used_edited_and_deleted() {
        let dir = scratch("grid");
        let list = dir.join("baseposlist.xml");
        let mut persisted = Persisted::at(None);
        let mut rtk = page_on(&mut persisted, list.clone());
        rtk.save_click();
        assert_eq!(rtk.prompt.as_ref().map(|p| p.title), Some("Enter Location"));
        if let Some(prompt) = rtk.prompt.as_mut() {
            prompt.field.set("roof");
        }
        let base = LatLngAlt {
            lat: -35.363_261,
            lng: 149.165_23,
            alt: 584.0,
        };
        rtk.answer(true, &mut persisted, base);
        assert_eq!(
            persisted.get("base_pos"),
            Some("-35.363261,149.16523,584,roof")
        );
        // The name kept as `InputBox` keeps every titled answer.
        // C#: GCSViews/ConfigurationView/ConfigSerialInjectGPS.cs:1269; InputBox.cs:178-184
        assert_eq!(
            persisted.get("InputBoxEnterLocationEnterafriendlynameforthislocation"),
            Some("roof")
        );
        let saved = std::fs::read_to_string(&list).expect("saved");
        assert_eq!(
            saved,
            "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n<ArrayOfPointLatLngAlt xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xmlns:xsd=\"http://www.w3.org/2001/XMLSchema\">\n  <PointLatLngAlt>\n    <Lat>-35.363261</Lat>\n    <Lng>149.16523</Lng>\n    <Alt>584</Alt>\n    <Tag>roof</Tag>\n    <Tag2 />\n    <color />\n  </PointLatLngAlt>\n</ArrayOfPointLatLngAlt>"
        );
        assert_eq!(basepos_list_from(&saved).map(|l| l.len()), Some(1));

        // An edit of the new row makes a row; a latitude that does not parse changes nothing.
        let page = rtk.page.as_mut().expect("a page");
        page.grid.push([
            "-35.5".to_owned(),
            "x".to_owned(),
            String::new(),
            "b".to_owned(),
        ]);
        page.cell_end_edit(1, 0);
        page.cell_end_edit(1, 1);
        page.cell_end_edit(1, 3);
        assert_eq!(page.list.len(), 2);
        assert_eq!((page.list[1].lat, page.list[1].lng), (-35.5, 0.0));
        assert_eq!(page.list[1].tag, "b");

        rtk.cell_click(1, 4, &mut persisted);
        assert_eq!(persisted.get("base_pos"), Some("-35.5,x,,b"));
        assert!(
            rtk.page.as_ref().is_some_and(|page| page.basepos.is_zero()),
            "a base position that does not parse is Zero"
        );
        rtk.cell_click(0, 4, &mut persisted);
        assert_eq!(
            rtk.page.as_ref().map(|page| page.basepos.tag.clone()),
            Some("roof".to_owned())
        );

        rtk.cell_click(0, 5, &mut persisted);
        let page = rtk.page.as_ref().expect("a page");
        assert_eq!(page.grid.len(), 1);
        assert_eq!(page.list.len(), 1);
        assert_eq!(page.list[0].tag, "b");
        // The new row cannot be deleted.
        rtk.cell_click(1, 5, &mut persisted);
        assert_eq!(rtk.page.as_ref().map(|page| page.grid.len()), Some(1));
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// `base_pos` read back: four fields, the last the name; anything else Zero.
    #[test]
    fn base_pos_reads_back_as_written() {
        let written = base_pos_setting(-35.1, 149.2, 580.25, "x");
        assert_eq!(written, "-35.1,149.2,580.25,x");
        assert_eq!(
            load_base_pos(Some(&written)),
            BasePos {
                lat: -35.1,
                lng: 149.2,
                alt: 580.25,
                tag: "x".to_owned()
            }
        );
        assert!(load_base_pos(Some("1,2,3")).is_zero());
        assert!(load_base_pos(None).is_zero());
        assert!(
            !load_base_pos(Some("0,0,0,t")).is_zero(),
            "a tag is not Zero"
        );
    }

    /// The bars: a system's made up to one per satellite and placed after the systems before
    /// it, all as wide as the panel shared among every bar; a message with fewer satellites
    /// leaves the extra bar where it was, at 0.
    #[test]
    fn the_bars_are_laid_out_as_the_csharp_lays_them() {
        let mut bars = Vec::new();
        obs_message(&mut bars, &obs_of(&GLONASS[..2], 'R'), 740);
        assert_eq!(
            bars.iter()
                .map(|b| (b.label.as_str(), b.x, b.width))
                .collect::<Vec<_>>(),
            [("R1", 0, 370), ("R7", 370, 370)]
        );
        obs_message(&mut bars, &obs_of(&GPS[..3], 'G'), 740);
        let placed: Vec<(&str, i32, i32, i32)> = bars
            .iter()
            .map(|b| (b.label.as_str(), b.x, b.width, b.value))
            .collect();
        // GPS first; GLONASS's bars stay where they were until its next message.
        assert_eq!(
            placed,
            [
                ("R1", 0, 370, 40),
                ("R7", 370, 370, 45),
                ("G2", 0, 148, 41),
                ("G5", 148, 148, 44),
                ("G10", 296, 148, 47)
            ]
        );
        obs_message(&mut bars, &obs_of(&GPS[..1], 'G'), 740);
        assert_eq!(bars[3].value, 0, "the bar left over is zeroed");
        assert_eq!(bars[3].label, "G5");
    }

    /// `updateSVINLabel`: surveying, surveyed, and with a base position chosen.
    #[test]
    fn the_survey_in_labels_are_the_csharps() {
        let mut labels = SvinLabels::default();
        let svin = Svin {
            valid: false,
            active: true,
            dur: 42,
            obs: 40,
            acc: 1.25,
            ecef: [0.0; 3],
            mode: ubx::Tmode3::default(),
        };
        labels.update(true, &svin);
        assert_eq!(
            labels.text,
            [
                "Position is invalid",
                "In Progress",
                "Duration: 42",
                "Observations: 40",
                "Current Acc: 1.25"
            ]
            .map(str::to_owned)
        );
        assert_eq!(labels.valid, Some(false));
        let ecef = rtcm3::pos2ecef([HOME.0.to_radians(), HOME.1.to_radians(), HOME.2]);
        labels.update(
            true,
            &Svin {
                valid: true,
                ecef,
                ..svin
            },
        );
        assert!(
            labels.text[1].starts_with("Lat/X: -35.36326"),
            "{}",
            labels.text[1]
        );
        assert_eq!(labels.valid, Some(true));
        labels.update(
            false,
            &Svin {
                mode: ubx::Tmode3::fixed(-35.5, 149.5, 600.0),
                ..svin
            },
        );
        assert_eq!(labels.text[0], "Using FixedLLA");
        assert_eq!(labels.text[1], "Lat/X: -35.5");
        assert_eq!(labels.visible, [true, true, true, true, false]);
    }

    /// `ProcessUBXMessage`: a 3D fix's position is `cs.Base`; a message nobody asked for is
    /// turned off on the port; TMODE3 and MON-VER are polled.
    #[test]
    fn ubx_messages_set_the_base_and_turn_off_the_rest() {
        let (mut receiver, port) = Loopback::pair();
        let shared = Shared::default();
        let mut pvt = [0u8; 92];
        pvt[20] = 3; // 3D fix
        pvt[21] = 1; // fix OK
        pvt[24..28].copy_from_slice(&1_491_652_300i32.to_le_bytes());
        pvt[28..32].copy_from_slice(&(-353_632_610i32).to_le_bytes());
        pvt[32..36].copy_from_slice(&584_000i32.to_le_bytes());
        let mut stream = ubx::generate(0x01, 0x07, &pvt);
        stream.extend(ubx::generate(0x01, 0x35, &[0; 8]));
        worker::feed(&shared, &stream, worker::Port::Other(Box::new(port)));
        let base = shared.counts().base.expect("the base");
        assert!((base.lat - -35.363_261).abs() < 1e-9 && (base.alt - 584.0).abs() < 1e-9);
        assert_eq!(
            shared.counts().msgseen.keys().cloned().collect::<Vec<_>>(),
            ["Ubx0107", "Ubx0135"]
        );
        let mut written = vec![0u8; 256];
        let n = receiver.read(&mut written).expect("written");
        let mut expected = ubx::generate(0x06, 0x71, &[]);
        expected.extend(ubx::generate(0x0a, 0x04, &[]));
        expected.extend(ubx::generate(0x06, 0x01, &[0x01, 0x35, 0, 0, 0, 0, 0, 0]));
        assert_eq!(&written[..n], expected.as_slice());
    }

    /// `timer1_Tick`: the rates as `String.Format("{0,10} bps", ...)` writes them and reset, the
    /// messages seen as `key=count ` each, the base line; a light whose time has run out red.
    #[test]
    fn the_timer_writes_the_rates_and_resets_them() {
        assert_eq!(bps_text(9876, false), "      9876 bps");
        assert_eq!(bps_text(0, true), "         0 bps sent");
        let dir = scratch("timer");
        let mut persisted = Persisted::at(None);
        let mut rtk = page_on(&mut persisted, dir.join("list.xml"));
        {
            let mut counts = rtk.shared.counts();
            counts.bps = 1200;
            counts.bpsusefull = 1100;
            counts.msgseen.insert("Rtcm1074".to_owned(), 3);
            counts.msgseen.insert("Rtcm1005".to_owned(), 1);
            counts.status_line3 = Some("-35.1 149.2 580 - 10:00:00".to_owned());
        }
        let now = Instant::now();
        if let Some(page) = rtk.page.as_mut() {
            page.lights[1] = Light {
                green: true,
                expires: Some(now + Duration::from_secs(5)),
            };
            page.lights[2] = Light {
                green: true,
                expires: Some(now - Duration::from_secs(1)),
            };
        }
        let base = LatLngAlt {
            lat: -35.1,
            lng: 149.2,
            alt: 580.0,
        };
        rtk.timer_tick(now, base);
        let page = rtk.page.as_ref().expect("a page");
        assert_eq!(
            page.status,
            [
                "      1200 bps",
                "      1100 bps sent",
                "-35.1 149.2 580 - 10:00:00",
                "Rtcm1005=1 Rtcm1074=3 "
            ]
            .map(str::to_owned)
        );
        assert_eq!(
            page.lights.map(|light| light.green),
            [false, true, false, false, false]
        );
        assert_eq!(page.marker.map(|m| m.latitude()), Some(-35.1));
        let counts = rtk.shared.counts();
        assert_eq!((counts.bps, counts.bpsusefull), (0, 0));
        assert!(rtk.shared.flush.load(std::sync::atomic::Ordering::Acquire));
        drop(counts);
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// The facts the script reads, on a page with nothing connected.
    #[test]
    fn the_facts_say_what_the_page_shows() {
        let dir = scratch("facts");
        let mut persisted = Persisted::at(None);
        let rtk = page_on(&mut persisted, dir.join("list.xml"));
        crate::facts::record("config.rtk.button", "unset");
        record_facts(&rtk, &view(), &persisted);
        let _ = std::fs::remove_dir_all(&dir);
        assert!(rtk.is_active());
        assert_eq!(TITLE, "RTK/GPS Inject");
        assert_eq!(
            crate::config_coverage::listing(crate::config_coverage::Screen::Setup, 251)
                .map(|(listed, _)| listed.title),
            Some(TITLE)
        );
    }
}
