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

//! The NMEA output: `Controls/SerialOutputNMEA.cs`, the Advanced page's NMEA button (`new
//! SerialOutputNMEA().Show()`, `ConfigAdvanced.cs:47-50`). A port, a baud and an update rate,
//! and Connect: a thread writes the vehicle's position as NMEA sentences to the port at the
//! rate.
//!
//! What it does:
//!
//! * the constructor lists the ports and the four network kinds, shows the rate as `updaterate +
//!   "hz"` (5 to start), and reads Stop when the thread is running - the stream, the rate and
//!   the thread are the class's, not the form's, so closing and opening the form changes nothing
//!   (`:20-33`, `:284-286`);
//! * Connect (`BUT_connect_Click`, `:35-110`): with the stream open, the thread is stopped, the
//!   stream closed and the button reads Connect; else the stream the port names is made - a TCP
//!   host listening on 14551 with no `Open`, a TCP client under `ConfigRef`
//!   `SerialOutputNMEATCP`, a UDP host or client, or the serial port - `Strings.InvalidPortName`
//!   when that throws, the baud `int.Parse`d (`Strings.InvalidBaudRate`), the stream opened
//!   ("Error Connecting"), the thread started and the button reads Stop;
//! * the thread (`mainloop`, `:112-242`) writes, each `updaterate`th of a second, with the
//!   vehicle's position as degrees and decimal minutes (`(int)lat + (lat - (int)lat) * .6f`):
//!   `GGA` with the fix, the satellites, the HDOP, the altitude above sea level and the geoid's
//!   undulation ([`mp_terrain::geoid`]); `GLL`; `HDG` with the yaw; `VTG` with the course, the
//!   yaw and the ground speed in knots and km/h; `RMC` with the local date; `HOM` every 20th
//!   round when `HomeLoc` is set - which nothing in the C# does, so never; `RPY` with the roll,
//!   pitch and yaw; each with its checksum, the XOR of the characters after `$`, then `\r\n`;
//!   then a sleep of the rest of the period, 4 s at most; a write that throws is skipped;
//! * the rate combo (`CMB_updaterate_SelectedIndexChanged`, `:271-282`): `float.Parse` of the
//!   text less `hz`, `Strings.InvalidUpdateRate` when that fails;
//! * `FormClosing` does nothing: the thread goes on.
//!
//! Where this is not the C# (each at its site):
//!
//! * the C#'s boxes - `InvalidPortName`, `InvalidBaudRate`, `InvalidUpdateRate`, "Error
//!   Connecting" - go on the status line (the owner's ruling of 2026-09-25); the transports'
//!   questions keep their boxes;
//! * the form is drawn over SETUP, modal; a TCP host takes one client (the module's notes in
//!   `serial_output.rs`);
//! * the thread reads a copy of the vehicle's state taken each frame, where the C# reads
//!   `MainV2.comPort.MAV.cs` live; a sentence written is counted, for the harness.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};
use web_time::{Duration, Instant};

use gpui::{AnyElement, Context, FocusHandle, Window, div, prelude::*, px, rgb};
use mp_mission::dotnet::{format_f64, general_f32, general_f64};
use mp_transport::Transport;
use mp_vehicle::VehicleState;

use super::optional::{button, input_box};
use super::serial_output::{
    BAUDS, CONNECT, ERROR_CONNECTING, INVALID_PORT_NAME, Kind, Opening, Questions, STOP, baud_of,
    combo, open, poll, port_items,
};
use crate::MissionPlanner;
use crate::settings::Persisted;
use crate::ui::theme;

/// `this.Text`. `// C#: Controls/SerialOutputNMEA.resx`
pub const FORM_TEXT: &str = "SerialOutput NMEA";
/// `ClientSize`.
pub const CLIENT: (f32, f32) = (228.0, 75.0);
/// `CMB_serialport`, `BUT_connect`, `CMB_baudrate`, `CMB_updaterate`: their places.
const PORT_AT: (f32, f32, f32, f32) = (13.0, 13.0, 121.0, 21.0);
const CONNECT_AT: (f32, f32, f32, f32) = (140.0, 13.0, 75.0, 23.0);
const BAUD_AT: (f32, f32, f32, f32) = (13.0, 40.0, 121.0, 21.0);
const RATE_AT: (f32, f32, f32, f32) = (141.0, 40.0, 75.0, 21.0);
/// `CMB_updaterate.Items`.
pub const RATES: [&str; 6] = ["10hz", "5hz", "2hz", "1hz", "0.5hz", "0.25hz"];
/// `updaterate`'s start.
pub const RATE: f64 = 5.0;
/// The TCP and UDP hosts' port in the names. `// C#: Controls/SerialOutputNMEA.cs:27, 29`
pub const HOST_PORT: u16 = 14551;
/// `ConfigRef` of the TCP client. `// C#: Controls/SerialOutputNMEA.cs:63`
pub const CONFIG_REF: &str = "_SerialOutputNMEATCP";
/// `Strings.InvalidUpdateRate`.
pub const INVALID_UPDATE_RATE: &str = "Invalid Update Rate";
/// The sleep's ceiling. `// C#: Controls/SerialOutputNMEA.cs:233`
const SLEEP_MOST: Duration = Duration::from_millis(4000);

/// What the sentences read of the vehicle, as `cs` has it.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Snapshot {
    /// `cs.lat`, `cs.lng`, degrees.
    pub lat: f64,
    pub lng: f64,
    /// `cs.gpsstatus`.
    pub gps_status: u8,
    /// `cs.satcount`.
    pub satcount: u8,
    /// `cs.gpshdop`.
    pub hdop: f32,
    /// `cs.altasl / CurrentState.multiplieralt`: metres above sea level.
    pub alt_asl: f32,
    /// `cs.yaw`, degrees 0 to 360.
    pub yaw: f32,
    /// `cs.groundcourse`, degrees.
    pub ground_course: f32,
    /// `cs.groundspeed`, m/s.
    pub ground_speed: f32,
    /// `cs.roll`, `cs.pitch`, degrees.
    pub roll: f32,
    pub pitch: f32,
}

impl Snapshot {
    /// The vehicle's state as `cs` holds it: the yaw brought to 0 to 360 as `CurrentState.yaw`
    /// is.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:274-284`
    #[must_use]
    #[allow(clippy::cast_possible_truncation)] // the C#'s floats
    pub fn of(state: &VehicleState) -> Self {
        let (lat, lng) = state
            .position
            .map_or((0.0, 0.0), |at| (at.latitude(), at.longitude()));
        let mut yaw = state.attitude.yaw.to_degrees().0 as f32;
        if yaw < 0.0 {
            yaw += 360.0;
        }
        Self {
            lat,
            lng,
            gps_status: state.gps.fix_type,
            satcount: state.gps.satellites_visible,
            hdop: state.gps.hdop,
            alt_asl: state.altitude_msl.0 as f32,
            yaw,
            ground_course: state.gps.course,
            ground_speed: state.ground_speed.0 as f32,
            roll: state.attitude.roll.to_degrees().0 as f32,
            pitch: state.attitude.pitch.to_degrees().0 as f32,
        }
    }
}

/// `GetChecksum`: the XOR of the characters after `$`, as two hex digits.
/// `// C#: Controls/SerialOutputNMEA.cs:248-269`
#[must_use]
pub fn checksum(sentence: &str) -> String {
    let mut sum: u32 = 0;
    for c in sentence.chars() {
        match c {
            '$' => {}
            '*' => continue,
            other => {
                let byte = u32::from(other) & 0xff;
                sum = if sum == 0 { byte } else { sum ^ byte };
            }
        }
    }
    format!("{sum:02X}")
}

/// `(int)lat + (lat - (int)lat) * .6f`: degrees to the degrees-and-minutes number the sentences
/// write, with the C#'s `float` 0.6.
#[must_use]
#[allow(clippy::cast_possible_truncation)] // `(int)`
pub fn degrees_minutes(value: f64) -> f64 {
    let whole = f64::from(value as i32);
    whole + (value - whole) * f64::from(0.6_f32)
}

/// The sentences one round writes, in order, each with its checksum and `\r` (`WriteLine` adds
/// the `\n`): `GGA`, `GLL`, `HDG`, `VTG`, `RMC`, `HOM` every 20th round with a home, `RPY`.
/// `// C#: Controls/SerialOutputNMEA.cs:124-230`
#[must_use]
pub fn sentences(
    cs: &Snapshot,
    utc: chrono::DateTime<chrono::Utc>,
    local: chrono::NaiveDate,
    counter: u32,
    home: Option<(f64, f64, f64)>,
) -> Vec<String> {
    let lat = degrees_minutes(cs.lat);
    let lng = degrees_minutes(cs.lng);
    let ns = if cs.lat < 0.0 { "S" } else { "N" };
    let ew = if cs.lng < 0.0 { "W" } else { "E" };
    let time = utc.format("%H%M%S%.3f").to_string();
    let finish = |line: String| format!("{line}*{}\r", checksum(&line));
    let mut out = Vec::new();
    // GGA: the fix is 1 from a 3D fix up; the undulation to a tenth of a metre; the last
    // field empty.
    out.push(finish(format!(
        "$GPGGA,{time},{},{ns},{},{ew},{},{},{},{},M,{},M,,",
        format_f64((lat * 100.0).abs(), "0000.00000"),
        format_f64((lng * 100.0).abs(), "00000.00000"),
        u8::from(cs.gps_status >= 3),
        cs.satcount,
        general_f32(cs.hdop),
        general_f64(f64::from(cs.alt_asl)),
        format_f64(mp_terrain::geoid::undulation(cs.lat, cs.lng), "0.0"),
    )));
    out.push(finish(format!(
        "$GPGLL,{},{ns},{},{ew},{time},A,A",
        format_f64((lat * 100.0).abs(), "0000.00"),
        format_f64((lng * 100.0).abs(), "00000.00"),
    )));
    out.push(finish(format!(
        "$GPHDG,{},0,E,0,E",
        format_f64(f64::from(cs.yaw), "0.0")
    )));
    out.push(finish(format!(
        "$GPVTG,{},{},{},{}",
        format_f64(f64::from(cs.ground_course), "000"),
        format_f64(f64::from(cs.yaw), "000"),
        format_f64(f64::from(cs.ground_speed) * 1.943_844, "00.0"),
        format_f64(f64::from(cs.ground_speed) * 3.6, "00.0"),
    )));
    out.push(finish(format!(
        "$GPRMC,{time},A,{},{ns},{},{ew},{},{},{},0,E,A",
        format_f64((lat * 100.0).abs(), "0.00000"),
        format_f64((lng * 100.0).abs(), "0.00000"),
        format_f64(f64::from(cs.ground_speed) * 1.943_844, "0.0"),
        format_f64(f64::from(cs.ground_course), "0.0"),
        local.format("%d%m%y"),
    )));
    if counter.is_multiple_of(20)
        && let Some((hlat, hlng, halt)) = home
        && hlat != 0.0
        && hlng != 0.0
    {
        out.push(finish(format!(
            "$GPHOM,{time},{},{},{},{},{},M,",
            format_f64((hlat * 100.0).abs(), "0.00000"),
            if hlat < 0.0 { "S" } else { "N" },
            format_f64((hlng * 100.0).abs(), "0.00000"),
            if hlng < 0.0 { "W" } else { "E" },
            general_f64(halt),
        )));
    }
    out.push(finish(format!(
        "$GPRPY,{},{},{},",
        format_f64(f64::from(cs.roll), "0.00000"),
        format_f64(f64::from(cs.pitch), "0.00000"),
        format_f64(f64::from(cs.yaw), "0.00000"),
    )));
    out
}

/// What the thread shares with the window.
struct Shared {
    /// `threadrun`.
    running: AtomicBool,
    /// `updaterate`.
    rate: Mutex<f64>,
    /// The vehicle, as the window last copied it.
    snapshot: Mutex<Snapshot>,
    /// Sentences written, and the last one.
    written: AtomicUsize,
    last: Mutex<String>,
}

/// `mainloop` on its thread, until `threadrun` is false.
/// `// C#: Controls/SerialOutputNMEA.cs:112-242`
fn main_loop(mut stream: Box<dyn Transport>, shared: &Shared) {
    let mut counter: u32 = 0;
    while shared.running.load(Ordering::Acquire) {
        if !stream.is_open() {
            wasm_thread::sleep(Duration::from_millis(10));
            continue;
        }
        let started = Instant::now();
        let cs = shared.snapshot.lock().map(|s| *s).unwrap_or_default();
        let lines = sentences(
            &cs,
            chrono::Utc::now(),
            chrono::Local::now().date_naive(),
            counter,
            None,
        );
        for line in lines {
            // `WriteLine`: the line and `\n`; a throw ends the round, caught.
            if stream.write_all(format!("{line}\n").as_bytes()).is_err() {
                break;
            }
            shared.written.fetch_add(1, Ordering::Relaxed);
            if let Ok(mut last) = shared.last.lock() {
                *last = line;
            }
        }
        let rate = shared.rate.lock().map(|r| *r).unwrap_or(RATE);
        let period = Duration::from_secs_f64((1000.0 / rate.max(0.001)).abs() / 1000.0);
        let elapsed = started.elapsed();
        let sleep_for = period.saturating_sub(elapsed).min(SLEEP_MOST);
        wasm_thread::sleep(sleep_for);
        counter = counter.wrapping_add(1);
    }
    stream.close();
}

/// The output, which outlives its form: the stream, the thread and the rate.
#[derive(Default)]
pub struct NmeaOutput {
    /// The form, while it is open.
    pub window: Option<Form>,
    /// How many times it has been opened.
    pub opened: usize,
    /// The thread's share, while a thread was started.
    shared: Option<Arc<Shared>>,
    /// A stream being opened, and the kind and baud it was asked for.
    opening: Option<Opening>,
    /// A thread's handle, to let go.
    thread: Option<wasm_thread::JoinHandle<()>>,
    /// `updaterate`: the class's, kept across forms.
    rate: Option<f64>,
    /// What the C# would have boxed, for the status line.
    status: Option<String>,
}

/// The form's controls.
#[derive(Debug)]
pub struct Form {
    /// `CMB_serialport`: its items and text, and whether its list is down.
    pub ports: Vec<String>,
    pub port: String,
    pub port_open: bool,
    /// `CMB_baudrate`.
    pub baud: String,
    pub baud_open: bool,
    /// `CMB_updaterate`.
    pub rate: String,
    pub rate_open: bool,
    /// The questions a Connect is asking.
    pub questions: Questions,
    /// The kind and baud the questions are for.
    pending: Option<(Kind, u32)>,
}

impl std::fmt::Debug for NmeaOutput {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("NmeaOutput")
            .field("window", &self.window)
            .field("running", &self.running())
            .finish_non_exhaustive()
    }
}

impl NmeaOutput {
    /// `threadrun`.
    #[must_use]
    pub fn running(&self) -> bool {
        self.shared
            .as_ref()
            .is_some_and(|shared| shared.running.load(Ordering::Acquire))
    }

    /// `updaterate`.
    #[must_use]
    pub fn rate(&self) -> f64 {
        self.rate.unwrap_or(RATE)
    }

    /// Sentences written since the thread started.
    #[must_use]
    pub fn written(&self) -> usize {
        self.shared
            .as_ref()
            .map_or(0, |shared| shared.written.load(Ordering::Relaxed))
    }

    /// The last sentence written.
    #[must_use]
    pub fn last(&self) -> String {
        self.shared
            .as_ref()
            .and_then(|shared| shared.last.lock().ok().map(|l| l.clone()))
            .unwrap_or_default()
    }

    /// `BUT_connect.Text`.
    #[must_use]
    pub fn button(&self) -> &'static str {
        if self.running() || self.opening.is_some() {
            STOP
        } else {
            CONNECT
        }
    }

    /// `new SerialOutputNMEA().Show()`: the ports and kinds listed, the first baud, the rate as
    /// the class has it, Stop while the thread runs.
    /// `// C#: Controls/SerialOutputNMEA.cs:20-33`
    pub fn show(&mut self) {
        self.opened += 1;
        let ports = port_items(HOST_PORT);
        self.window = Some(Form {
            port: ports.first().cloned().unwrap_or_default(),
            ports,
            port_open: false,
            baud: BAUDS[0].to_owned(),
            baud_open: false,
            rate: format!("{}hz", general_f64(self.rate())),
            rate_open: false,
            questions: Questions::default(),
            pending: None,
        });
    }

    /// The form's close box: `FormClosing` does nothing; the thread goes on.
    pub fn close(&mut self) {
        self.window = None;
    }

    /// `CMB_serialport`'s list, `CMB_baudrate`'s, `CMB_updaterate`'s: opened or closed.
    pub fn toggle_list(&mut self, which: Combo) {
        if let Some(form) = self.window.as_mut() {
            let (port, baud, rate) = (form.port_open, form.baud_open, form.rate_open);
            form.port_open = false;
            form.baud_open = false;
            form.rate_open = false;
            match which {
                Combo::Port => form.port_open = !port,
                Combo::Baud => form.baud_open = !baud,
                Combo::Rate => form.rate_open = !rate,
            }
        }
    }

    /// An item chosen in a list; the rate's `SelectedIndexChanged` parses it.
    /// `// C#: Controls/SerialOutputNMEA.cs:271-282`
    pub fn choose(&mut self, which: Combo, index: usize) {
        let Some(form) = self.window.as_mut() else {
            return;
        };
        match which {
            Combo::Port => {
                if let Some(item) = form.ports.get(index) {
                    form.port = item.clone();
                }
                form.port_open = false;
            }
            Combo::Baud => {
                if let Some(item) = BAUDS.get(index) {
                    form.baud = (*item).to_owned();
                }
                form.baud_open = false;
            }
            Combo::Rate => {
                if let Some(item) = RATES.get(index) {
                    form.rate = (*item).to_owned();
                }
                form.rate_open = false;
                let text = form.rate.replace("hz", "");
                match text.trim().parse::<f32>() {
                    Ok(rate) => {
                        self.rate = Some(f64::from(rate));
                        if let Some(shared) = self.shared.as_ref()
                            && let Ok(mut held) = shared.rate.lock()
                        {
                            *held = f64::from(rate);
                        }
                    }
                    Err(_) => self.status = Some(INVALID_UPDATE_RATE.to_owned()),
                }
            }
        }
    }

    /// `BUT_connect_Click`: Stop ends the thread and closes the stream; Connect makes the stream
    /// the port names, asking the kind's questions first.
    /// `// C#: Controls/SerialOutputNMEA.cs:35-110`
    pub fn press_connect(&mut self, persisted: &Persisted) {
        if self.opening.is_some() {
            return;
        }
        if self.running() {
            if let Some(shared) = self.shared.as_ref() {
                shared.running.store(false, Ordering::Release);
            }
            if let Some(thread) = self.thread.take() {
                let deadline = Instant::now() + Duration::from_millis(500);
                while !thread.is_finished() && Instant::now() < deadline {
                    wasm_thread::sleep(Duration::from_millis(1));
                }
            }
            return;
        }
        let Some(form) = self.window.as_mut() else {
            return;
        };
        let port = form.port.trim().to_owned();
        if port.is_empty() {
            // `new SerialPort()` with no name: `Strings.InvalidPortName`.
            self.status = Some(INVALID_PORT_NAME.to_owned());
            return;
        }
        let kind = Kind::of(&port);
        let baud = match baud_of(&form.baud) {
            Ok(baud) => baud,
            Err(words) => {
                self.status = Some(words);
                return;
            }
        };
        // `ConfigRef = "SerialOutputNMEATCP"` is the TCP client's alone; the UDP kinds ask under
        // the plain keys.
        let config_ref = if kind == Kind::TcpClient {
            CONFIG_REF
        } else {
            ""
        };
        form.questions = Questions {
            pending: kind.questions(config_ref),
            ..Questions::default()
        };
        form.pending = Some((kind, baud));
        if !form.questions.ask_next(persisted) {
            self.start_opening();
        }
    }

    /// A question answered: the next asked, or the stream opened with the answers.
    pub fn answered(&mut self, ok: bool, persisted: &mut Persisted) {
        let Some(form) = self.window.as_mut() else {
            return;
        };
        if !form.questions.answered(ok, persisted) {
            form.pending = None;
            return;
        }
        if !form.questions.ask_next(persisted) {
            self.start_opening();
        }
    }

    /// The stream opened on its thread, with the answers.
    fn start_opening(&mut self) {
        let Some(form) = self.window.as_mut() else {
            return;
        };
        let Some((kind, baud)) = form.pending.take() else {
            return;
        };
        let answers = std::mem::take(&mut form.questions.answers);
        self.opening = Some(open(kind, HOST_PORT, baud, answers));
    }

    /// Once a frame: the vehicle copied for the thread; a stream opened makes the thread, or
    /// "Error Connecting" on the status line. What goes on the status line is returned.
    pub fn tick(&mut self, state: Option<&VehicleState>) -> Option<String> {
        if let Some(shared) = self.shared.as_ref()
            && let Some(state) = state
            && let Ok(mut held) = shared.snapshot.lock()
        {
            *held = Snapshot::of(state);
        }
        if let Some(opening) = self.opening.as_ref()
            && let Some(opened) = poll(opening)
        {
            self.opening = None;
            match opened {
                Ok(stream) => self.start_thread(stream),
                Err(error) => self.status = Some(format!("{ERROR_CONNECTING}: {error}")),
            }
        }
        self.status.take()
    }

    /// `t12.Start()`: the thread with the stream and the rate.
    fn start_thread(&mut self, stream: Box<dyn Transport>) {
        let shared = Arc::new(Shared {
            running: AtomicBool::new(true),
            rate: Mutex::new(self.rate()),
            snapshot: Mutex::new(Snapshot::default()),
            written: AtomicUsize::new(0),
            last: Mutex::new(String::new()),
        });
        let for_thread = Arc::clone(&shared);
        match wasm_thread::Builder::new()
            .name("Nmea output".to_owned())
            .spawn(move || main_loop(stream, &for_thread))
        {
            Ok(handle) => {
                self.thread = Some(handle);
                self.shared = Some(shared);
            }
            Err(error) => self.status = Some(format!("{ERROR_CONNECTING}: {error}")),
        }
    }

    /// Waits for a stream being opened, as a test does.
    #[cfg(test)]
    fn finish_opening(&mut self) -> Option<String> {
        let mut status = None;
        for _ in 0..2000 {
            if let Some(words) = self.tick(None) {
                status = Some(words);
            }
            if self.opening.is_none() {
                break;
            }
            wasm_thread::sleep(Duration::from_millis(5));
        }
        status
    }
}

/// The form's three combos.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Combo {
    Port,
    Baud,
    Rate,
}

/// The facts, under `config.nmea.`.
pub fn record_facts(holder: &NmeaOutput) {
    use crate::facts::record;
    record("config.nmea.window", holder.window.is_some());
    record("config.nmea.opened", holder.opened);
    record("config.nmea.running", holder.running());
    record("config.nmea.rate", general_f64(holder.rate()));
    record("config.nmea.button", holder.button());
    record("config.nmea.written", holder.written());
    record("config.nmea.last", holder.last());
    let Some(form) = holder.window.as_ref() else {
        return;
    };
    record("config.nmea.port", form.port.clone());
    record("config.nmea.baud", form.baud.clone());
    record("config.nmea.ratetext", form.rate.clone());
    record(
        "config.nmea.prompt",
        form.questions
            .asking
            .as_ref()
            .map_or("none", |asking| asking.question.title),
    );
}

fn access(this: &mut MissionPlanner) -> &mut NmeaOutput {
    &mut this.extra.nmea_output
}

/// The form over SETUP: the port, Connect, the baud and the rate, and a question over it.
/// `// C#: Controls/SerialOutputNMEA.resx`
pub fn overlay(
    holder: &NmeaOutput,
    prompt: &FocusHandle,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    let form = holder.window.as_ref()?;
    let size = window.viewport_size();
    let idle = holder.opening.is_none();
    let client = div()
        .relative()
        .w(px(CLIENT.0))
        .h(px(CLIENT.1))
        .child(button(
            "nmea-BUT_connect",
            holder.button(),
            CONNECT_AT,
            idle,
            |this, window, cx| {
                window.blur(cx);
                this.extra.nmea_output.press_connect(&this.persisted);
            },
            cx,
        ))
        .child(combo(
            "nmea-CMB_updaterate",
            &form.rate,
            &RATES.map(str::to_owned),
            form.rate_open,
            idle,
            RATE_AT,
            |this| access(this).toggle_list(Combo::Rate),
            |this, index| access(this).choose(Combo::Rate, index),
            cx,
        ))
        .child(combo(
            "nmea-CMB_baudrate",
            &form.baud,
            &BAUDS.map(str::to_owned),
            form.baud_open,
            idle,
            BAUD_AT,
            |this| access(this).toggle_list(Combo::Baud),
            |this, index| access(this).choose(Combo::Baud, index),
            cx,
        ))
        .child(combo(
            "nmea-CMB_serialport",
            &form.port,
            &form.ports,
            form.port_open,
            idle,
            PORT_AT,
            |this| access(this).toggle_list(Combo::Port),
            |this, index| access(this).choose(Combo::Port, index),
            cx,
        ));
    let caption = div()
        .flex()
        .items_center()
        .justify_between()
        .px_2()
        .py_1()
        .border_b_1()
        .border_color(rgb(theme::BORDER))
        .child(div().text_xs().text_color(rgb(theme::DIM)).child(FORM_TEXT))
        .child(crate::ui::action(
            "nmea-close",
            "X",
            theme::TEXT,
            true,
            cx.listener(|this, _event: &(), _window, cx| {
                access(this).close();
                cx.notify();
            }),
        ));
    let body = crate::probe::measured("nmea", div())
        .flex()
        .flex_col()
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::ACCENT))
        .rounded_md()
        .child(caption)
        .child(client);
    let over = div()
        .id("nmea-backdrop")
        .w(size.width)
        .h(size.height)
        .flex()
        .items_center()
        .justify_center()
        .occlude()
        .child(body);
    let mut layers = div().child(
        gpui::deferred(
            gpui::anchored()
                .position(gpui::point(px(0.0), px(0.0)))
                .child(over),
        )
        .with_priority(1),
    );
    if let Some(asking) = form.questions.asking.as_ref() {
        layers = layers.child(input_box(
            "nmea-prompt-box",
            &asking.input,
            prompt,
            window,
            |this, event| {
                let outcome = access(this)
                    .window
                    .as_mut()
                    .and_then(|form| form.questions.asking.as_mut())
                    .map(|asking| asking.input.field.key(event));
                match outcome {
                    Some(crate::textfield::KeyOutcome::Submitted) => {
                        finish_question(this, true);
                        true
                    }
                    Some(crate::textfield::KeyOutcome::Cancelled) => {
                        finish_question(this, false);
                        true
                    }
                    Some(crate::textfield::KeyOutcome::Changed) => true,
                    _ => false,
                }
            },
            |this| finish_question(this, true),
            |this| finish_question(this, false),
            cx,
        ));
    }
    Some(layers.into_any_element())
}

fn finish_question(this: &mut MissionPlanner, ok: bool) {
    this.extra.nmea_output.answered(ok, &mut this.persisted);
}

#[cfg(test)]
mod tests {
    use super::super::serial_output::INVALID_BAUD_RATE;
    use super::*;
    use chrono::TimeZone as _;

    fn canberra() -> Snapshot {
        Snapshot {
            lat: -35.363_261,
            lng: 149.165_23,
            gps_status: 3,
            satcount: 10,
            hdop: 1.2,
            alt_asl: 584.0,
            yaw: 123.4,
            ground_course: 90.0,
            ground_speed: 10.0,
            roll: 1.5,
            pitch: -2.25,
        }
    }

    /// `GetChecksum`: the XOR after `$`, two hex digits.
    #[test]
    fn the_checksum_is_the_csharps() {
        assert_eq!(checksum("$GPGLL,3537.86,S,14909.91,E,120000.000,A,A"), "46");
        assert_eq!(checksum("$GPHDG,123.4,0,E,0,E"), "5A");
    }

    /// The sentences of a round, each as the C# formats it: the degrees-and-minutes number with
    /// the float 0.6, the GGA's fix, satellites, HDOP, altitude and undulation, the VTG's knots
    /// and km/h, the RMC's local date, the RPY's five places; no HOM, as nothing sets HomeLoc.
    /// `// C#: Controls/SerialOutputNMEA.cs:124-230`
    #[test]
    fn the_sentences_are_the_csharps() {
        let utc = chrono::Utc
            .with_ymd_and_hms(2026, 10, 2, 12, 34, 56)
            .single()
            .expect("a time");
        let local = chrono::NaiveDate::from_ymd_opt(2026, 10, 2).expect("a date");
        let lines = sentences(&canberra(), utc, local, 0, None);
        assert_eq!(lines.len(), 6);
        assert!(
            lines[0].starts_with(
                "$GPGGA,123456.000,3521.79566,S,14909.91380,E,1,10,1.2,584,M,19.4,M,,*"
            ),
            "{}",
            lines[0]
        );
        assert!(
            lines[1].starts_with("$GPGLL,3521.80,S,14909.91,E,123456.000,A,A*"),
            "{}",
            lines[1]
        );
        assert!(
            lines[2].starts_with("$GPHDG,123.4,0,E,0,E*"),
            "{}",
            lines[2]
        );
        assert!(
            lines[3].starts_with("$GPVTG,090,123,19.4,36.0*"),
            "{}",
            lines[3]
        );
        assert!(
            lines[4].starts_with(
                "$GPRMC,123456.000,A,3521.79566,S,14909.91380,E,19.4,90.0,021026,0,E,A*"
            ),
            "{}",
            lines[4]
        );
        assert!(
            lines[5].starts_with("$GPRPY,1.50000,-2.25000,123.40000,*"),
            "{}",
            lines[5]
        );
        for line in &lines {
            assert!(line.ends_with('\r'));
            let body = &line[..line.len() - 4];
            assert!(
                line[line.len() - 3..line.len() - 1] == checksum(body),
                "{line}"
            );
        }
        // A home set, on a 20th round: HOM between RMC and RPY.
        let with_home = sentences(&canberra(), utc, local, 20, Some((-35.0, 149.0, 584.0)));
        assert_eq!(with_home.len(), 7);
        assert!(
            with_home[5].starts_with("$GPHOM,123456.000,3500.00000,S,14900.00000,E,584,M,*"),
            "{}",
            with_home[5]
        );
        let off_round = sentences(&canberra(), utc, local, 21, Some((-35.0, 149.0, 584.0)));
        assert_eq!(off_round.len(), 6);
        // The 0.6 is a float: the C#'s number, not 0.6 exactly.
        assert_eq!(degrees_minutes(-35.5), -35.0 + -0.5 * f64::from(0.6_f32));
    }

    /// The form: Connect over a UDP client to nothing starts the thread, which writes rounds at
    /// the rate; the rate changed takes; Stop ends it; the class keeps the rate and the thread
    /// across forms.
    /// `// C#: Controls/SerialOutputNMEA.cs:20-110, 271-286`
    #[test]
    fn connect_runs_the_thread_at_the_rate_and_stop_ends_it() {
        let mut persisted = Persisted::at(None);
        let mut output = NmeaOutput::default();
        output.show();
        assert_eq!(output.button(), CONNECT);
        assert_eq!(
            output.window.as_ref().map(|f| f.rate.clone()),
            Some("5hz".to_owned())
        );
        // A rate that does not parse: the box's words.
        output.window.as_mut().expect("form").rate = "fast".to_owned();
        output.choose(Combo::Rate, 0);
        assert_eq!(output.rate(), 10.0);
        if let Some(form) = output.window.as_mut() {
            let index = form
                .ports
                .iter()
                .position(|p| p == "UDP Client")
                .expect("UDP Client");
            output.choose(Combo::Port, index);
        }
        output.press_connect(&persisted);
        // The two questions: host, then port, their defaults.
        assert_eq!(
            output
                .window
                .as_ref()
                .and_then(|f| f.questions.asking.as_ref())
                .map(|a| a.question.title),
            Some("remote host")
        );
        output.answered(true, &mut persisted);
        assert_eq!(
            output
                .window
                .as_ref()
                .and_then(|f| f.questions.asking.as_ref())
                .map(|a| a.question.title),
            Some("remote Port")
        );
        output.answered(true, &mut persisted);
        assert_eq!(output.button(), STOP);
        assert_eq!(output.finish_opening(), None);
        let state = VehicleState::default();
        for _ in 0..200 {
            output.tick(Some(&state));
            if output.written() >= 12 {
                break;
            }
            wasm_thread::sleep(Duration::from_millis(5));
        }
        assert!(output.running());
        assert!(output.written() >= 12, "{}", output.written());
        assert!(output.last().starts_with("$GPRPY,"), "{}", output.last());
        // The form closed and opened again: Stop, the thread on.
        output.close();
        output.show();
        assert_eq!(output.button(), STOP);
        assert_eq!(
            output.window.as_ref().map(|f| f.rate.clone()),
            Some("10hz".to_owned())
        );
        output.press_connect(&persisted);
        assert!(!output.running());
        assert_eq!(output.button(), CONNECT);
        assert_eq!(persisted.get("UDP_host"), Some("127.0.0.1"));
    }

    /// An invalid baud and an empty port name are the C#'s boxes, on the status line.
    #[test]
    fn bad_baud_and_port_are_the_strings() {
        let persisted = Persisted::at(None);
        let mut output = NmeaOutput::default();
        output.show();
        output.window.as_mut().expect("form").baud = "fast".to_owned();
        output.press_connect(&persisted);
        assert_eq!(output.tick(None), Some(INVALID_BAUD_RATE.to_owned()));
        output.window.as_mut().expect("form").baud = "9600".to_owned();
        output.window.as_mut().expect("form").port = String::new();
        output.press_connect(&persisted);
        assert_eq!(output.tick(None), Some(INVALID_PORT_NAME.to_owned()));
    }

    /// The GUI script names only facts and controls this window has.
    #[test]
    fn the_gui_script_names_facts_and_controls_this_window_has() {
        let script = include_str!("../../../../tests/gui/config-nmea.gui");
        let source = include_str!("nmea_output.rs");
        let mut facts = 0;
        for line in script.lines() {
            let line = line.split('#').next().unwrap_or("");
            let mut words = line.split_whitespace();
            match (words.next(), words.next()) {
                (Some("expect"), Some(key)) if key.starts_with("config.nmea.") => {
                    assert!(
                        source.contains(&format!("\"{key}\"")),
                        "{key} is not recorded"
                    );
                    facts += 1;
                }
                (Some("click"), Some(id)) if id.starts_with("nmea-") => {
                    let fixed = id.split('@').next().unwrap_or(id);
                    let drawn = source.contains(&format!("\"{fixed}\""))
                        || fixed.starts_with("nmea-CMB_")
                        || fixed.starts_with("nmea-prompt-");
                    assert!(drawn, "{id} is not drawn");
                }
                _ => {}
            }
        }
        assert!(facts >= 6, "{facts} facts");
    }
}
