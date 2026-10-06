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

//! Serial port transport and enumeration.
//!
//! Replaces `ExtLibs/Comms/CommsSerialPort.cs`. Which ports are listed, and in what order, is the
//! C#'s rule set in [`crate::enumerate`]; this module only feeds it what the OS shows. Enumeration
//! carries USB VID/PID because board detection (Deliverable 13) identifies autopilots by them, exactly as
//! `BoardDetect.cs` does today.

#[cfg(not(target_family = "wasm"))]
use mp_os::fs::FsExt as _;
use std::io;
#[cfg(not(target_family = "wasm"))]
use std::io::{Read, Write};
use std::time::Duration;

#[cfg(not(target_family = "wasm"))]
use crate::DEFAULT_READ_TIMEOUT;
#[cfg(not(target_family = "wasm"))]
use crate::enumerate;
pub use crate::enumerate::PortInfo;
use crate::{OpenError, Transport};

/// Lists serial ports currently present, as Mission Planner would list them on this machine.
///
/// The names and their order are `SerialPort.GetPortNames()`'s: on Linux and macOS the `/dev`
/// globs and Mono's own list, stable `/dev/serial/by-id` names first; on Windows the port names
/// the OS reports, with the C#'s repairs applied. Each is then annotated with the USB ids the
/// `serialport` crate finds for the device it names. Nothing is opened.
///
/// Returns an empty list rather than an error when enumeration is unsupported: a headless CI box
/// with no serial hardware is a normal environment, not a failure.
#[must_use]
pub fn list_ports() -> Vec<PortInfo> {
    #[cfg(target_family = "wasm")]
    {
        crate::page::serial_ports()
    }
    #[cfg(not(target_family = "wasm"))]
    {
        let known = usb_ports();
        let names = port_names(&known);
        enumerate::with_usb_metadata(names, device_node, &known)
    }
}

/// What the `serialport` crate knows about each device node, for its USB ids - on Windows with
/// Windows' own record of each port over it, the hardware id, description and bus-reported name
/// Mission Planner reads (`win32.rs`). Its own list is not the one shown: its order is its own,
/// and it has no by-id names.
#[cfg(not(target_family = "wasm"))]
fn usb_ports() -> Vec<PortInfo> {
    let ports = crate_ports();
    #[cfg(windows)]
    let ports = crate::enumerate::with_windows_devices(ports, &crate::win32::com_ports());
    ports
}

/// The `serialport` crate's list, as the crate reports it.
#[cfg(not(target_family = "wasm"))]
fn crate_ports() -> Vec<PortInfo> {
    let Ok(ports) = serialport::available_ports() else {
        return Vec::new();
    };
    ports
        .into_iter()
        .map(|p| match p.port_type {
            serialport::SerialPortType::UsbPort(usb) => PortInfo {
                name: p.port_name,
                vid: Some(usb.vid),
                pid: Some(usb.pid),
                serial_number: usb.serial_number,
                manufacturer: usb.manufacturer,
                product: usb.product,
                hardware_id: None,
                description: None,
            },
            _ => PortInfo::bare(p.port_name),
        })
        .collect()
}

#[cfg(unix)]
fn port_names(_known: &[PortInfo]) -> Vec<String> {
    let listing = dev_listing();
    let entries: Vec<&str> = listing.iter().map(String::as_str).collect();
    let runtime = enumerate::mono_port_names(&entries);
    enumerate::get_port_names(&entries, &runtime)
}

#[cfg(all(not(unix), not(target_family = "wasm")))]
fn port_names(known: &[PortInfo]) -> Vec<String> {
    // .NET reads the SERIALCOMM registry key here; the serialport crate asks SetupAPI. The two
    // normally name the same COM ports, and a driver that registers with only one of them would
    // make the lists differ. The crate's list stands in, in its order, with the C#'s trimming and
    // Bluetooth repair applied to it.
    let runtime: Vec<&str> = known.iter().map(|port| port.name.as_str()).collect();
    enumerate::get_port_names(&[], &runtime)
}

/// The two directories the C# globs, as the listing [`enumerate`] expects: absolute paths in
/// directory order, directories with a trailing `/`.
#[cfg(unix)]
fn dev_listing() -> Vec<String> {
    let mut listing = Vec::new();
    for directory in ["/dev/", "/dev/serial/by-id/"] {
        // The C# wraps each glob in its own try/catch: a directory that cannot be read lists
        // nothing, and the others still count.
        let Ok(read) = mp_os::fs::read_dir(directory) else {
            continue;
        };
        for entry in read.flatten() {
            let path = format!("{directory}{}", entry.file_name().to_string_lossy());
            // stat() rather than the entry's own type, because GetFiles decides by what a symlink
            // points at; a dangling link fails the stat and counts as a file, as it does there.
            let is_dir = mp_os::fs::metadata(&path).is_ok_and(|meta| meta.os_is_dir());
            listing.push(if is_dir { format!("{path}/") } else { path });
        }
    }
    listing
}

/// The device node a listed name stands for: a by-id symlink resolves to its `/dev/ttyACM0`.
/// realpath() reads links and never opens the device.
#[cfg(unix)]
fn device_node(name: &str) -> Option<String> {
    mp_os::fs::canonicalize(name)
        .ok()
        .map(|path| path.to_string_lossy().into_owned())
}

/// On Windows a port name is already the device. canonicalize() would open it to find out, and
/// opening a COM port can reset the board on the other end.
#[cfg(all(not(unix), not(target_family = "wasm")))]
fn device_node(_name: &str) -> Option<String> {
    None
}

/// An open serial port.
#[cfg(not(target_family = "wasm"))]
pub struct SerialTransport {
    port: Box<dyn serialport::SerialPort>,
    name: String,
    baud: u32,
    /// `serial:<path>:<baud>`, kept ready for [`Transport::description`].
    description: String,
    open: bool,
}

/// What a serial port is called: its path and the rate it runs at.
fn describe(name: &str, baud: u32) -> String {
    format!("serial:{name}:{baud}")
}

#[cfg(not(target_family = "wasm"))]
impl std::fmt::Debug for SerialTransport {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SerialTransport")
            .field("name", &self.name)
            .field("baud", &self.baud)
            .field("open", &self.open)
            .finish()
    }
}

#[cfg(not(target_family = "wasm"))]
impl SerialTransport {
    /// Opens a port at the given baud rate.
    pub fn open(path: &str, baud: u32) -> Result<Self, OpenError> {
        // C#: ExtLibs/Comms/CommsSerialPort.cs:472-474 - a device path that is not there fails
        // before the driver is asked, with the message Mission Planner shows. An unplugged board's
        // by-id link is gone or dangling, and both land here.
        if path.starts_with('/') && !std::path::Path::new(path).os_exists() {
            return Err(OpenError::io(
                format!("opening {path} at {baud} baud"),
                io::Error::new(io::ErrorKind::NotFound, "No such device"),
            ));
        }
        let port = serialport::new(path, baud)
            .timeout(DEFAULT_READ_TIMEOUT)
            .open()
            .map_err(|e| {
                OpenError::io(
                    format!("opening {path} at {baud} baud"),
                    io::Error::other(e),
                )
            })?;
        Ok(Self {
            port,
            name: path.to_owned(),
            baud,
            description: describe(path, baud),
            open: true,
        })
    }

    /// Toggles DTR, which is how ArduPilot boards are pushed into the bootloader.
    pub fn set_dtr(&mut self, level: bool) -> io::Result<()> {
        self.port
            .write_data_terminal_ready(level)
            .map_err(io::Error::other)
    }

    /// Changes baud rate on an open port, used by the SiK radio configurator.
    pub fn set_baud(&mut self, baud: u32) -> io::Result<()> {
        self.port.set_baud_rate(baud).map_err(io::Error::other)?;
        self.baud = baud;
        self.description = describe(&self.name, baud);
        Ok(())
    }
}

#[cfg(not(target_family = "wasm"))]
impl Transport for SerialTransport {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        match self.port.read(buf) {
            Ok(n) => Ok(n),
            Err(e) if e.kind() == io::ErrorKind::TimedOut => Ok(0),
            Err(e) if e.kind() == io::ErrorKind::WouldBlock => Ok(0),
            Err(e) => {
                // A vanished USB device reports a variety of errors per platform; all of them
                // mean the same thing to us.
                self.open = false;
                Err(e)
            }
        }
    }

    fn write_all(&mut self, buf: &[u8]) -> io::Result<()> {
        self.port.write_all(buf).inspect_err(|_| {
            self.open = false;
        })
    }

    fn description(&self) -> &str {
        &self.description
    }

    fn is_open(&self) -> bool {
        self.open
    }

    fn set_read_timeout(&mut self, timeout: Duration) -> io::Result<()> {
        self.port.set_timeout(timeout).map_err(io::Error::other)
    }

    fn close(&mut self) {
        self.open = false;
    }
}

/// An open serial port in a web page: one the browser has let the page use (WebSerial), opened
/// and carried by the page as its links are (web/www/serial.js, link.js) - for SiK radios and the
/// firmware tools, which open their ports themselves. WebSerial changes a port's rate only by
/// opening it again, so a baud change is the port reopened at the new rate; DTR is the port's
/// `setSignals`. Each goes to the page in order with the bytes around it.
#[cfg(target_family = "wasm")]
pub struct SerialTransport {
    link: crate::page::PageTransport,
    name: String,
    baud: u32,
    /// `serial:<name>:<baud>`, kept ready for [`Transport::description`].
    description: String,
}

#[cfg(target_family = "wasm")]
impl std::fmt::Debug for SerialTransport {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SerialTransport")
            .field("name", &self.name)
            .field("baud", &self.baud)
            .field("open", &self.link.is_open())
            .finish()
    }
}

#[cfg(target_family = "wasm")]
impl SerialTransport {
    /// Asks the page to open the port it calls `path` at `baud`.
    ///
    /// # Errors
    /// None today: the page answers by sending bytes or not.
    pub fn open(path: &str, baud: u32) -> Result<Self, OpenError> {
        Ok(Self {
            link: crate::page::PageTransport::open(&describe(path, baud))?,
            name: path.to_owned(),
            baud,
            description: describe(path, baud),
        })
    }

    /// DTR, which is how ArduPilot boards are pushed into the bootloader.
    ///
    /// # Errors
    /// None: the page sets it in its turn.
    pub fn set_dtr(&mut self, level: bool) -> io::Result<()> {
        self.link
            .control(&format!("serial-dtr\n{}", u8::from(level)));
        Ok(())
    }

    /// The port at another rate, used by the SiK radio configurator: reopened by the page.
    ///
    /// # Errors
    /// None: the page reopens it in its turn.
    pub fn set_baud(&mut self, baud: u32) -> io::Result<()> {
        self.link.control(&format!("serial-baud\n{baud}"));
        self.baud = baud;
        self.description = describe(&self.name, baud);
        Ok(())
    }
}

#[cfg(target_family = "wasm")]
impl Transport for SerialTransport {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        self.link.read(buf)
    }

    fn write_all(&mut self, buf: &[u8]) -> io::Result<()> {
        self.link.write_all(buf)
    }

    fn description(&self) -> &str {
        &self.description
    }

    fn is_open(&self) -> bool {
        self.link.is_open()
    }

    fn set_read_timeout(&mut self, timeout: Duration) -> io::Result<()> {
        self.link.set_read_timeout(timeout)
    }

    fn close(&mut self) {
        self.link.close();
    }
}
