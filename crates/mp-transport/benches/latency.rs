//! What a transport adds to the OS's own read: D3's `benches/latency.rs`.
//!
//! D3's DoD is ≤ 1 ms added latency over a raw OS read. Measured as a round trip: this end
//! sends one byte, the far end answers with a 64-byte frame the moment it sees it, and the
//! time from the send to the whole answer's arrival is taken twice - once reading the far end's
//! answer through the operating system's own socket or port, once through `mp_transport`'s
//! [`Transport`] for the same link - two thousand times each. The transport's cost is the
//! difference of the two 99th percentiles, and the gate is that difference under a millisecond.
//!
//! Two links: TCP over the loopback interface, and a serial port that is a pseudo-terminal - the
//! one real serial device a test can make. The serial baseline reads the slave through the
//! `serialport` crate's `TTYPort`, which is what the port I/O itself is here (the crate opens
//! the device raw; a `std::fs::File` on a pseudo-terminal reads it in canonical mode, a line at a
//! time, and would measure the terminal discipline rather than the read). The pseudo-terminal
//! part is skipped, saying so, on a machine without one.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]
#![allow(missing_docs)] // criterion_group! expands to an undocumented pub fn

use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream};
use std::time::{Duration, Instant};

use criterion::{Criterion, criterion_group};
use mp_transport::Transport;

/// D3: added latency over a raw OS read, at the 99th percentile.
const BUDGET: Duration = Duration::from_millis(1);
const ROUNDS: usize = 2000;
const ANSWER: usize = 64;

/// A link this end can send on and receive from, whatever it is underneath.
trait Duplex {
    fn send(&mut self, bytes: &[u8]);
    fn recv(&mut self, buf: &mut [u8]) -> usize;
}

impl Duplex for TcpStream {
    fn send(&mut self, bytes: &[u8]) {
        self.write_all(bytes).unwrap();
    }
    fn recv(&mut self, buf: &mut [u8]) -> usize {
        self.read(buf).unwrap()
    }
}

impl Duplex for Box<dyn Transport> {
    fn send(&mut self, bytes: &[u8]) {
        self.write_all(bytes).unwrap();
    }
    fn recv(&mut self, buf: &mut [u8]) -> usize {
        self.read(buf).unwrap()
    }
}

/// One byte out, sixty-four back: the round trip, timed from the send to the last byte.
fn round_trip(link: &mut dyn Duplex) -> Duration {
    let mut buf = [0u8; ANSWER];
    let started = Instant::now();
    link.send(&[1]);
    let mut got = 0;
    while got < ANSWER {
        got += link.recv(&mut buf[got..]);
    }
    started.elapsed()
}

fn p99(times: &mut [Duration]) -> Duration {
    times.sort_unstable();
    times[times.len() * 99 / 100]
}

fn p50(times: &mut [Duration]) -> Duration {
    times.sort_unstable();
    times[times.len() / 2]
}

/// The far end of a TCP link: answers every byte with the frame, for as many connections as
/// come.
fn tcp_far_end() -> u16 {
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let port = listener.local_addr().unwrap().port();
    std::thread::spawn(move || {
        for stream in listener.incoming() {
            let Ok(mut stream) = stream else { break };
            stream.set_nodelay(true).ok();
            let mut poke = [0u8; 1];
            while stream.read(&mut poke).map(|n| n > 0).unwrap_or(false) {
                if stream.write_all(&[0x5A; ANSWER]).is_err() {
                    break;
                }
            }
        }
    });
    port
}

/// The round trips over TCP: through the OS's socket, then through the transport.
fn tcp_rounds(port: u16, rounds: usize) -> (Vec<Duration>, Vec<Duration>) {
    let mut raw = TcpStream::connect(("127.0.0.1", port)).unwrap();
    raw.set_nodelay(true).unwrap();
    let raw_times: Vec<Duration> = (0..rounds).map(|_| round_trip(&mut raw)).collect();
    drop(raw);

    let mut transport = mp_transport::open(&format!("tcp:127.0.0.1:{port}")).unwrap();
    let transport_times: Vec<Duration> = (0..rounds).map(|_| round_trip(&mut transport)).collect();
    (raw_times, transport_times)
}

#[cfg(all(unix, feature = "serial"))]
mod pty {
    use super::*;
    use serialport::{SerialPort, TTYPort};

    /// A pseudo-terminal whose master answers every byte with the frame: the slave's path, and
    /// the slave itself, which the caller holds for the length of the measurement - closing the
    /// last descriptor on the slave hangs the master up, and a hung-up master's thread ends and
    /// takes the terminal with it, so the second opener would find no such device. `None` where
    /// there are no pseudo-terminals.
    pub(super) fn far_end() -> Option<(String, TTYPort)> {
        let (mut master, slave) = match TTYPort::pair() {
            Ok(pair) => pair,
            Err(e) => {
                println!("latency: no pseudo-terminals here ({e}); serial skipped");
                return None;
            }
        };
        let path = slave.name()?;
        master.set_timeout(Duration::from_secs(30)).ok();
        std::thread::spawn(move || {
            let mut poke = [0u8; 1];
            while master.read(&mut poke).map(|n| n > 0).unwrap_or(false) {
                if master.write_all(&[0x5A; ANSWER]).is_err() {
                    break;
                }
            }
        });
        Some((path, slave))
    }

    impl Duplex for Box<dyn SerialPort> {
        fn send(&mut self, bytes: &[u8]) {
            self.write_all(bytes).unwrap();
        }
        fn recv(&mut self, buf: &mut [u8]) -> usize {
            self.read(buf).unwrap()
        }
    }

    /// The round trips over the pseudo-terminal: through the crate's port, then the transport.
    pub(super) fn rounds(path: &str, rounds: usize) -> (Vec<Duration>, Vec<Duration>) {
        let mut raw: Box<dyn SerialPort> = serialport::new(path, 115_200)
            .timeout(Duration::from_secs(5))
            .open()
            .unwrap();
        let raw_times: Vec<Duration> = (0..rounds).map(|_| round_trip(&mut raw)).collect();
        drop(raw);

        let mut transport = mp_transport::open(&format!("serial:{path}:115200")).unwrap();
        let transport_times: Vec<Duration> =
            (0..rounds).map(|_| round_trip(&mut transport)).collect();
        (raw_times, transport_times)
    }
}

fn latency(c: &mut Criterion) {
    let port = tcp_far_end();
    let mut group = c.benchmark_group("tcp_round_trip");
    group.bench_function("raw", |b| {
        let mut raw = TcpStream::connect(("127.0.0.1", port)).unwrap();
        raw.set_nodelay(true).unwrap();
        b.iter(|| round_trip(&mut raw));
    });
    group.bench_function("transport", |b| {
        let mut transport = mp_transport::open(&format!("tcp:127.0.0.1:{port}")).unwrap();
        b.iter(|| round_trip(&mut transport));
    });
    group.finish();
}

fn report(link: &str, (mut raw, mut through): (Vec<Duration>, Vec<Duration>)) {
    let (raw_p50, raw_p99) = (p50(&mut raw), p99(&mut raw));
    let (t_p50, t_p99) = (p50(&mut through), p99(&mut through));
    let added = t_p99.saturating_sub(raw_p99);
    println!(
        "latency gate, {link}: raw p50 {raw_p50:?} p99 {raw_p99:?}; transport p50 {t_p50:?} p99 \
         {t_p99:?}; added at p99 {added:?} (budget {BUDGET:?}) over {ROUNDS} round trips"
    );
    assert!(
        added <= BUDGET,
        "{link}: the transport adds {added:?} at p99 over the raw read, more than D3's {BUDGET:?}"
    );
}

/// D3's floor: the transport's added latency at the 99th percentile, TCP and serial.
fn gate() {
    if cfg!(debug_assertions) {
        println!("latency gate: skipped in an unoptimised build");
        return;
    }
    let port = tcp_far_end();
    report("tcp", tcp_rounds(port, ROUNDS));
    #[cfg(all(unix, feature = "serial"))]
    if let Some((path, _keep_alive)) = pty::far_end() {
        report("serial (pty)", pty::rounds(&path, ROUNDS));
    }
}

criterion_group!(benches, latency);

fn main() {
    benches();
    Criterion::default().configure_from_args().final_summary();
    gate();
}
