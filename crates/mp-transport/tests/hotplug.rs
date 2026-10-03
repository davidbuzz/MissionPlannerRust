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

//! Surprise unplug and reconnect.
//!
//! Mission Planner learns that a port has gone from the exception its next read throws, which
//! ends the read loop (`ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:4762-4767`); it reconnects
//! by opening a new port under the same name; and a device path that is not there fails before
//! the driver is asked, with "No such device" (`ExtLibs/Comms/CommsSerialPort.cs:502-504`). These
//! tests pin the same three things here: on the mock, on a real serial port over a pseudo-terminal
//! whose master side is pulled away, and on TCP.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use std::io::{self, Write};
use std::net::TcpListener;
use std::sync::mpsc;
use std::thread;
use std::time::{Duration, Instant};

use mp_transport::testing::Loopback;
use mp_transport::{OpenError, Transport};

/// Reads until `want` bytes have arrived or `limit` passes, treating `Ok(0)` as an idle tick.
fn read_exactly(link: &mut dyn Transport, want: usize, limit: Duration) -> Vec<u8> {
    let deadline = Instant::now() + limit;
    let mut got = Vec::new();
    let mut buf = [0u8; 256];
    while got.len() < want && Instant::now() < deadline {
        let n = link.read(&mut buf).unwrap();
        got.extend_from_slice(&buf[..n]);
    }
    got
}

#[test]
fn a_surprise_unplug_is_reported_on_the_next_read_as_an_error_not_zero_bytes_forever() {
    let (mut vehicle, gcs) = Loopback::pair();
    let plug = gcs.plug();
    // The reader owns its end, as the link's I/O thread does; the cable is pulled from outside.
    let mut gcs: Box<dyn Transport> = Box::new(gcs);
    let (have_it, got_it) = mpsc::channel();

    let reader = thread::spawn(move || {
        let mut buf = [0u8; 64];
        let mut received = Vec::new();
        loop {
            match gcs.read(&mut buf) {
                // Idle: the mock returns at once, so do not spin a core while waiting.
                Ok(0) => thread::sleep(Duration::from_millis(1)),
                Ok(n) => {
                    received.extend_from_slice(&buf[..n]);
                    if received.len() == b"heartbeat".len() {
                        have_it.send(()).unwrap();
                    }
                }
                Err(error) => {
                    let open = gcs.is_open();
                    let again = gcs.read(&mut buf).map_err(|e| e.kind());
                    return (received, error.kind(), open, again);
                }
            }
        }
    });

    vehicle.write_all(b"heartbeat").unwrap();
    got_it.recv_timeout(Duration::from_secs(10)).unwrap();
    plug.pull();

    let (received, kind, open, again) = reader.join().unwrap();
    assert_eq!(received, b"heartbeat");
    assert_eq!(kind, io::ErrorKind::BrokenPipe);
    assert!(!open, "the end knows it is closed once a read has failed");
    assert_eq!(
        again,
        Err(io::ErrorKind::BrokenPipe),
        "still failing, not Ok(0)"
    );

    // The far end finds out on its next write.
    let error = vehicle.write_all(b"x").unwrap_err();
    assert_eq!(error.kind(), io::ErrorKind::BrokenPipe);
    assert!(!vehicle.is_open());
}

#[test]
fn an_unplug_loses_whatever_was_still_in_flight() {
    // A hang-up discards the driver's buffer: bytes written before the unplug but not yet read
    // never arrive, and the next read is the error.
    let (mut vehicle, mut gcs) = Loopback::pair();
    vehicle.write_all(b"never read").unwrap();
    assert_eq!(gcs.pending(), 10);
    gcs.plug().pull();
    let mut buf = [0u8; 64];
    let error = gcs.read(&mut buf).unwrap_err();
    assert_eq!(error.kind(), io::ErrorKind::BrokenPipe);
}

#[cfg(feature = "serial")]
#[test]
fn opening_a_serial_path_that_does_not_exist_fails_naming_the_path() {
    // url.rs shows this parses without looking at the disk; here it is opened.
    let path = "/dev/serial/by-id/usb-nothing-here-if-mavlink";
    match mp_transport::open(path) {
        Err(OpenError::Io { context, source }) => {
            assert!(context.contains(path), "{context}");
            assert_eq!(source.kind(), io::ErrorKind::NotFound);
            // C#: ExtLibs/Comms/CommsSerialPort.cs:504
            assert_eq!(source.to_string(), "No such device");
        }
        Err(other) => panic!("expected an I/O error naming the path, got {other}"),
        Ok(_) => panic!("{path} opened"),
    }
}

#[cfg(not(feature = "serial"))]
#[test]
fn opening_a_serial_path_without_serial_support_says_so() {
    let result = mp_transport::open("/dev/serial/by-id/usb-nothing-here-if-mavlink");
    assert!(matches!(result, Err(OpenError::Unsupported("serial"))));
}

#[test]
fn a_tcp_peer_that_goes_away_is_end_of_stream_and_the_same_url_reconnects() {
    // SITL restarting: the connection vanishes under the GCS and comes back on the same port.
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let url = format!("tcp:127.0.0.1:{}", listener.local_addr().unwrap().port());
    let server = thread::spawn(move || {
        for greeting in [b"first".as_slice(), b"second"] {
            let (mut stream, _) = listener.accept().unwrap();
            stream.write_all(greeting).unwrap();
        }
    });

    let mut link = mp_transport::open(&url).unwrap();
    link.set_read_timeout(Duration::from_millis(50)).unwrap();
    assert_eq!(
        read_exactly(link.as_mut(), 5, Duration::from_secs(10)),
        b"first"
    );

    // TCP says goodbye rather than failing: a peer's close is Ok(0) with is_open() false, which
    // the Transport contract defines as end of stream. The link engine stops on either.
    let deadline = Instant::now() + Duration::from_secs(10);
    let mut buf = [0u8; 16];
    while link.is_open() && Instant::now() < deadline {
        assert_eq!(link.read(&mut buf).unwrap(), 0);
    }
    assert!(!link.is_open(), "the peer's close must be noticed");

    let mut link = mp_transport::open(&url).unwrap();
    link.set_read_timeout(Duration::from_millis(50)).unwrap();
    assert_eq!(
        read_exactly(link.as_mut(), 6, Duration::from_secs(10)),
        b"second"
    );
    server.join().unwrap();
}

/// A real serial port: the slave side of a pseudo-terminal, opened through `SerialTransport` by a
/// stable, by-id-shaped symlink. Dropping the master is the board being unplugged; removing and
/// re-pointing the link is what udev does as it goes and comes back.
// Linux only: macOS's pseudo-terminals refuse the slave's second open by path with ENOTTY ("Not a
// typewriter"; the owner's Mac and the hosted runner, 2026-10-03), so the port never opens and the
// test fails for the harness's sake, not the transport's, whose serial path the Linux run holds.
#[cfg(all(target_os = "linux", feature = "serial"))]
mod pty {
    use std::io::Write;
    use std::path::{Path, PathBuf};
    use std::time::Duration;

    use serialport::{SerialPort, TTYPort};

    use super::read_exactly;
    use mp_transport::OpenError;

    /// A new pseudo-terminal: its master, and the path of its slave with nothing holding it open.
    fn board() -> Option<(TTYPort, String)> {
        let (master, slave) = match TTYPort::pair() {
            Ok(pair) => pair,
            Err(e) => {
                println!("no pseudo-terminals here ({e}); skipped");
                return None;
            }
        };
        let path = slave.name()?;
        drop(slave);
        Some((master, path))
    }

    fn plug_in(link: &Path, device: &str) {
        let _ = std::fs::remove_file(link);
        std::os::unix::fs::symlink(device, link).unwrap();
    }

    #[test]
    fn a_serial_port_that_vanishes_fails_its_next_read_and_the_same_url_reopens_when_it_returns() {
        let dir = PathBuf::from(env!("CARGO_TARGET_TMPDIR"))
            .join(format!("hotplug-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let link = dir.join("usb-ArduPilot_Test_0123456789ABCDEF-if00");
        let url = format!("serial:{}:115200", link.display());

        let Some((mut master, device)) = board() else {
            return;
        };
        plug_in(&link, &device);
        let mut port = mp_transport::open(&url).unwrap();
        master.write_all(b"hello").unwrap();
        assert_eq!(
            read_exactly(port.as_mut(), 5, Duration::from_secs(10)),
            b"hello"
        );

        // Unplugged: the very next read is an error, and so is the one after.
        drop(master);
        let mut buf = [0u8; 16];
        let error = port
            .read(&mut buf)
            .expect_err("a vanished port must fail its read");
        assert!(
            !matches!(
                error.kind(),
                std::io::ErrorKind::TimedOut | std::io::ErrorKind::WouldBlock
            ),
            "{error:?} would read as a timeout"
        );
        // poll() reports the hang-up and the serialport crate maps it to BrokenPipe.
        #[cfg(target_os = "linux")]
        assert_eq!(error.kind(), std::io::ErrorKind::BrokenPipe, "{error:?}");
        assert!(!port.is_open());
        assert!(port.read(&mut buf).is_err(), "still failing, not Ok(0)");
        drop(port);

        // While it is gone the stable name does not open, and the error says which name.
        std::fs::remove_file(&link).unwrap();
        match mp_transport::open(&url) {
            Err(OpenError::Io { context, source }) => {
                assert!(context.contains(&link.display().to_string()), "{context}");
                assert_eq!(source.to_string(), "No such device");
            }
            Err(other) => panic!("expected an I/O error naming the path, got {other}"),
            Ok(_) => panic!("opened a device that is not there"),
        }

        // Plugged back in, perhaps as a different node: the same URL opens it.
        let Some((mut master, device)) = board() else {
            return;
        };
        plug_in(&link, &device);
        let mut port = mp_transport::open(&url).unwrap();
        master.write_all(b"again").unwrap();
        assert_eq!(
            read_exactly(port.as_mut(), 5, Duration::from_secs(10)),
            b"again"
        );

        drop(port);
        let _ = std::fs::remove_dir_all(&dir);
    }
}
