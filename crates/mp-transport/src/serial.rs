//! Serial port transport and enumeration.
//!
//! Replaces `ExtLibs/Comms/CommsSerialPort.cs`. Enumeration carries USB VID/PID because board
//! detection (D13) identifies autopilots by them, exactly as `BoardDetect.cs` does today.

use std::io::{self, Read, Write};
use std::time::Duration;

use crate::{DEFAULT_READ_TIMEOUT, OpenError, Transport};

/// A discovered serial port.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PortInfo {
    /// Device path (`/dev/ttyACM0`) or Windows port name (`COM3`).
    pub name: String,
    /// USB vendor id, when the port is a USB device.
    pub vid: Option<u16>,
    /// USB product id, when the port is a USB device.
    pub pid: Option<u16>,
    /// USB serial number, used to tell two identical boards apart.
    pub serial_number: Option<String>,
    /// Manufacturer string.
    pub manufacturer: Option<String>,
    /// Product string.
    pub product: Option<String>,
}

impl PortInfo {
    /// A one-line label for the connection dropdown.
    #[must_use]
    pub fn label(&self) -> String {
        match (&self.product, self.vid, self.pid) {
            (Some(product), Some(vid), Some(pid)) => {
                format!("{} - {product} ({vid:04x}:{pid:04x})", self.name)
            }
            (Some(product), _, _) => format!("{} - {product}", self.name),
            _ => self.name.clone(),
        }
    }
}

/// Lists serial ports currently present.
///
/// Returns an empty list rather than an error when enumeration is unsupported: a headless CI box
/// with no serial hardware is a normal environment, not a failure.
#[must_use]
pub fn list_ports() -> Vec<PortInfo> {
    let Ok(ports) = serialport::available_ports() else {
        return Vec::new();
    };
    let mut out: Vec<PortInfo> = ports
        .into_iter()
        .map(|p| {
            let (vid, pid, serial_number, manufacturer, product) = match p.port_type {
                serialport::SerialPortType::UsbPort(usb) => (
                    Some(usb.vid),
                    Some(usb.pid),
                    usb.serial_number,
                    usb.manufacturer,
                    usb.product,
                ),
                _ => (None, None, None, None, None),
            };
            PortInfo {
                name: p.port_name,
                vid,
                pid,
                serial_number,
                manufacturer,
                product,
            }
        })
        .collect();
    // Stable ordering so the UI list does not reshuffle between refreshes.
    out.sort_by(|a, b| a.name.cmp(&b.name));
    out
}

/// An open serial port.
pub struct SerialTransport {
    port: Box<dyn serialport::SerialPort>,
    name: String,
    baud: u32,
    open: bool,
}

impl std::fmt::Debug for SerialTransport {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SerialTransport")
            .field("name", &self.name)
            .field("baud", &self.baud)
            .field("open", &self.open)
            .finish()
    }
}

impl SerialTransport {
    /// Opens a port at the given baud rate.
    pub fn open(path: &str, baud: u32) -> Result<Self, OpenError> {
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
        Ok(())
    }
}

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

    fn description(&self) -> String {
        format!("serial:{}:{}", self.name, self.baud)
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
