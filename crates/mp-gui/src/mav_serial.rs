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

//! EXPERIMENTAL's MAVSerial pass, "create a exclusive passthrough to the gps (port 500)"
//! (temp.resx's label15): the vehicle's first GPS port carried in `SERIAL_CONTROL` messages over
//! the link, and served to one TCP client at a time on port 500 - a GPS tool on another machine
//! talking to the GPS through the autopilot.
//!
//! * [`SerialPort`]: `MAVLinkSerialPort`, the proxy port. Made, it hears the vehicle's
//!   `SERIAL_CONTROL`s ([`reply_of`]) and starts its loop; `Open` sends a request with the
//!   timeout, 10 ms, and the baud rate - 0, no change, as nothing here sets it - and waits a
//!   second; the loop asks for data every `timeout` ms while the port is open, and a reply with
//!   data asks again at once; what replies carry goes into a 4096-byte circular buffer that
//!   overflows as the C#'s does ([`Ring`]); `Read` waits up to `timeout` for data or fails,
//!   "MAVLinkSerialPort Timeout on read"; `Write` sends the bytes with timeout 0, 70 a message
//!   (mp-link's `serial_control`, the C#'s `SendSerialControl`), 10 ms before each; `Close` sends
//!   the close flag, the port handed back to its driver.
//! * [`Pass`]: `but_mavserialport_Click`'s listener and `DoAcceptTcpClientCallback` - the port on
//!   `SERIAL_CONTROL_DEV.GPS1` and a listener on 0.0.0.0:500; a client accepted (a second while
//!   one is connected turned away, as the C#'s callback returns from it), the port opened for the
//!   first, and the bytes carried both ways until it goes or the port closes.
//! * [`press`]: the button - pressed with none running, the port made for the vehicle shown and
//!   the listener started; pressed again, all of it closed. What the C# shows in a box - no link
//!   (`Strings.PleaseConnect`), no vehicle (`Strings.No_valid_heartbeats_read_from_port`), port
//!   500 refused (below 1024 without root on Linux) - is said on the status line as an error, the
//!   owner's rule for errors the window can show; listening and closed are said quietly (the
//!   owner's rule of 2026-10-14: a press that answers nothing looks broken). The fact
//!   `experimental.mavserial` names where it is.
//!
//! Where it is not the C#, each for a reason:
//!
//! * `getHeartBeat` - a ground station's heartbeat sent, and up to 2.2 s read for the vehicle's -
//!   is the link's vehicle, heard: the planner's link reads every packet itself, and a vehicle it
//!   has heard is what that read would find.
//! * The C#'s subscription outlives `Close` (its unsubscribe is commented out), and a reply still
//!   on its way asks again with `EXCLUSIVE`, taking the GPS back from its driver after the port
//!   was handed back; here the subscription ends with the port, and nothing is sent after the
//!   close flag - nor after `Open`'s second, which the C#'s ends with `open = true` even when the
//!   port was closed during it.
//! * A press after a close starts again: the C#'s `comport` is never set back to null, so every
//!   press after its first only closes it again. A press whose listener is refused leaves nothing
//!   behind - the port made has sent nothing - where the C#'s keeps its `comport`, and its next
//!   press closes rather than tries again.
//! * `client.Connected` is a read of nothing (the client gone) or a failed write, as .NET's
//!   socket learns it; a second client is closed, where the C#'s is accepted and never answered
//!   (its callback returns before `EndAcceptTcpClient`).
//!
//! `// C#: temp.cs:54-58, 310-408; temp.resx (label15); ExtLibs/ArduPilot/Mavlink/MAVLinkSerialPort.cs:15-316; ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:2965-3016; ExtLibs/Utilities/CircularBuffer.cs:87-169`

use mp_os::Lock as _;
use std::io::{ErrorKind, Read as _, Write as _};
use std::net::{Ipv4Addr, SocketAddr, SocketAddrV4, TcpListener, TcpStream};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, Weak};
use web_time::{Duration, Instant};

use mp_link::commands::{SERIAL_CONTROL_DEV_GPS1, serial_control};
use mp_link::inspector::{Packet, PacketSubscription};
use mp_mavlink_dialects::all::{MavMessage, SerialControl};
use mp_vehicle::VehicleId;

use crate::MissionPlanner;
use crate::fly::error_box;

/// The button's text, which its quiet answers begin with.
/// `// C#: temp.resx (but_mavserialport.Text)`
const NAME: &str = "MAVSerial pass";
/// `Strings.PleaseConnect`: the constructor's exception without a link.
/// `// C#: ExtLibs/Strings/Strings.resx:205-207; ExtLibs/ArduPilot/Mavlink/MAVLinkSerialPort.cs:58-61`
const PLEASE_CONNECT: &str = "Please connect first";
/// `Strings.No_valid_heartbeats_read_from_port`: its exception with no vehicle.
/// `// C#: ExtLibs/Strings/Strings.resx:627-629; ExtLibs/ArduPilot/Mavlink/MAVLinkSerialPort.cs:63-66`
const NO_HEARTBEATS: &str = "No valid heartbeats read from port";
/// `Read`'s exception when nothing has come within `timeout`.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkSerialPort.cs:164`
const READ_TIMEOUT: &str = "MAVLinkSerialPort Timeout on read";

/// `timeout`, in milliseconds: how long `Read` waits, how often the loop asks, and the timeout
/// every request carries. `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkSerialPort.cs:25`
const TIMEOUT_MS: u16 = 10;
/// `new CircularBuffer<byte>(4096)`. `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkSerialPort.cs:29`
const BUFFER_CAPACITY: usize = 4096;
/// `mainloop`'s sleep between `GetData`s.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkSerialPort.cs:87`
const LOOP_SLEEP: Duration = Duration::from_millis(5);
/// `Read`'s sleep while it waits. `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkSerialPort.cs:162`
const READ_SLEEP: Duration = Duration::from_millis(1);
/// `Open`'s wait before the port is open.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkSerialPort.cs:202`
const OPEN_WAIT: Duration = Duration::from_secs(1);
/// How often `Open`'s wait looks for a `Close`, which ends it.
const OPEN_POLL: Duration = Duration::from_millis(10);
/// `SendSerialControl`'s "dont flood the port": the sleep before each message of data.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:3000-3001`
const DONT_FLOOD: Duration = Duration::from_millis(10);

/// `new TcpListener(IPAddress.Any, 500)`. `// C#: temp.cs:337`
const LISTEN: SocketAddr = SocketAddr::V4(SocketAddrV4::new(Ipv4Addr::UNSPECIFIED, 500));
/// The bridge's reads, either way: `new byte[4096]`. `// C#: temp.cs:375, 389`
const BRIDGE_BUFFER: usize = 4096;
/// The bridge's sleep between rounds. `// C#: temp.cs:401`
const BRIDGE_SLEEP: Duration = Duration::from_millis(1);
/// How often the listener looks for a client: the C#'s `BeginAcceptTcpClient` is told of one,
/// here a listener that does not block is asked.
const ACCEPT_POLL: Duration = Duration::from_millis(5);

/// The fact naming where the passthrough is: "listening on 0.0.0.0:500", "serving <client>",
/// "closed", or "refused: <why>".
const FACT: &str = "experimental.mavserial";

/// `experimental.mavserial` set to `what`.
fn note(what: impl std::fmt::Display) {
    crate::facts::record(FACT, what);
}

// ---------------------------------------------------------------------------------------------
// The port.
// ---------------------------------------------------------------------------------------------

/// What the port sends on: the link, from whichever thread - its loop's, the bridge's, or the
/// link's own as a reply is heard.
pub(crate) trait SerialLink: Send + Sync {
    /// `generatePacket`: `message` queued for the vehicle; false once the link has stopped.
    fn send(&self, message: &MavMessage) -> bool;
}

impl SerialLink for mp_link::LinkSender {
    fn send(&self, message: &MavMessage) -> bool {
        Self::send(self, message)
    }
}

/// `CircularBuffer<byte>` as `MAVLinkSerialPort` uses it, `AllowOverflow` on: `Put` writes on
/// round the ring over the oldest bytes and the size stops at the capacity without the head
/// moving, so after an overflow `Get` starts where the oldest byte was put - in the middle of the
/// newest - as the C#'s does.
/// `// C#: ExtLibs/Utilities/CircularBuffer.cs:87-169; ExtLibs/ArduPilot/Mavlink/MAVLinkSerialPort.cs:130-134`
#[derive(Debug)]
struct Ring {
    /// `buffer`, `capacity` long.
    buffer: Vec<u8>,
    /// `head`: where `Get` reads next.
    head: usize,
    /// `tail`: where `Put` writes next.
    tail: usize,
    /// `size`: how many it holds.
    size: usize,
}

impl Ring {
    /// `new CircularBuffer<byte>(capacity)`.
    fn new(capacity: usize) -> Self {
        Self {
            buffer: vec![0; capacity],
            head: 0,
            tail: 0,
            size: 0,
        }
    }

    /// `Put(src, 0, src.Length)` with `AllowOverflow`.
    /// `// C#: ExtLibs/Utilities/CircularBuffer.cs:99-116`
    fn put(&mut self, src: &[u8]) {
        let capacity = self.buffer.len();
        for &byte in src {
            if self.tail == capacity {
                self.tail = 0;
            }
            if let Some(slot) = self.buffer.get_mut(self.tail) {
                *slot = byte;
            }
            self.tail += 1;
        }
        self.size = (self.size + src.len()).min(capacity);
    }

    /// `Get(dst, 0, dst.Length)`: as many bytes as it holds, `dst`'s length at most; how many.
    /// `// C#: ExtLibs/Utilities/CircularBuffer.cs:154-169`
    fn get(&mut self, dst: &mut [u8]) -> usize {
        let capacity = self.buffer.len();
        let count = dst.len().min(self.size);
        for slot in dst.iter_mut().take(count) {
            if self.head == capacity {
                self.head = 0;
            }
            *slot = self.buffer.get(self.head).copied().unwrap_or(0);
            self.head += 1;
        }
        self.size -= count;
        count
    }

    /// `Size`.
    const fn size(&self) -> usize {
        self.size
    }
}

/// Whether `GetData` asks again at `now`, the last asking at `last` (`None`, `DateTime.MinValue`,
/// never): `lastgetdata.AddMilliseconds(timeout) < DateTime.Now`.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkSerialPort.cs:146`
fn due(last: Option<Instant>, now: Instant) -> bool {
    last.is_none_or(|last| last + Duration::from_millis(u64::from(TIMEOUT_MS)) < now)
}

/// `MAVLinkSerialPort`: a serial port of the vehicle's, carried in `SERIAL_CONTROL`s.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkSerialPort.cs:15-316`
pub(crate) struct SerialPort {
    /// `mavint`: what its messages go out on.
    link: Box<dyn SerialLink>,
    /// `port`: the `SERIAL_CONTROL_DEV`.
    device: u8,
    /// `buffer`: what the vehicle's replies carried, not yet read.
    buffer: Mutex<Ring>,
    /// `open`.
    open: AtomicBool,
    /// `lastgetdata`: when `GetData` last asked.
    last_get_data: Mutex<Option<Instant>>,
    /// Set by `Close`, as its `bgdata.Abort()`: the loop ended, `Open`'s wait with it, and
    /// nothing sent after the close flag.
    ended: AtomicBool,
    /// Held while a message is sent, so none follows the close flag.
    sending: Mutex<()>,
    /// `Open`'s wait: [`OPEN_WAIT`], shorter in a test.
    open_wait: Duration,
}

impl SerialPort {
    /// `new MAVLinkSerialPort(mavint, port)` past its refusals ([`refusal`]): the port on
    /// `device`, and its loop started. What the vehicle says reaches it by the caller's
    /// subscription, [`SerialPort::received`].
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkSerialPort.cs:53-78`
    pub(crate) fn new(link: Box<dyn SerialLink>, device: u8, open_wait: Duration) -> Arc<Self> {
        let port = Arc::new(Self {
            link,
            device,
            buffer: Mutex::new(Ring::new(BUFFER_CAPACITY)),
            open: AtomicBool::new(false),
            last_get_data: Mutex::new(None),
            ended: AtomicBool::new(false),
            sending: Mutex::new(()),
            open_wait,
        });
        let looping = Arc::downgrade(&port);
        let spawned = wasm_thread::Builder::new()
            .name("MAVLinkSerialPort".to_owned())
            .spawn(move || main_loop(&looping));
        if let Err(error) = spawned {
            log::debug!("MAVLinkSerialPort: {error}");
        }
        port
    }

    /// `SendSerialControl(port, timeoutms, data, baudrate, close)`: mp-link's messages sent, 10 ms
    /// before each that carries data; none but the close flag once the port is closed.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:2965-3016`
    fn send_serial_control(&self, timeout: u16, data: &[u8], baudrate: u32, close: bool) {
        for message in serial_control(self.device, timeout, data, baudrate, close) {
            if !data.is_empty() {
                wasm_thread::sleep(DONT_FLOOD);
            }
            let Ok(_sending) = self.sending.os_lock() else {
                return;
            };
            if !close && self.ended.load(Ordering::Acquire) {
                return;
            }
            self.link.send(&message);
        }
    }

    /// `GetData(now)`: a request for what the port has read - `SendSerialControl(port, timeout,
    /// null)`, `EXCLUSIVE | RESPOND | MULTI` - when `timeout` ms have passed since the last and
    /// the port is open, or `now`.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkSerialPort.cs:142-151`
    fn get_data(&self, now: bool) {
        let Ok(mut last) = self.last_get_data.os_lock() else {
            return;
        };
        if (due(*last, Instant::now()) && self.open.load(Ordering::Acquire)) || now {
            self.send_serial_control(TIMEOUT_MS, &[], 0, false);
            *last = Some(Instant::now());
        }
    }

    /// `ReceviedPacket`: a `SERIAL_CONTROL` from the vehicle. One with no bytes is passed over;
    /// one with bytes has them put in the buffer, overflowing as it may, and asks again at once
    /// ("we just received data, so do a request again"). A `count` past the 70 bytes of `data`
    /// puts the 70, where the C#'s `Put` throws part way.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkSerialPort.cs:108-140`
    pub(crate) fn received(&self, reply: &SerialControl) {
        if reply.count == 0 {
            return;
        }
        log::debug!("MAVLinkSerialPort data count {}", reply.count);
        let bytes = reply
            .data
            .get(..usize::from(reply.count))
            .unwrap_or(&reply.data);
        if let Ok(mut buffer) = self.buffer.os_lock() {
            buffer.put(bytes);
        }
        self.get_data(true);
    }

    /// `buffer.Size`.
    fn buffered(&self) -> usize {
        self.buffer.os_lock().map_or(0, |buffer| buffer.size())
    }

    /// `Read(readto, offset, length)`: what the buffer holds, `into`'s length at most, once it
    /// holds something - asking meanwhile - or [`READ_TIMEOUT`] after `timeout` ms.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkSerialPort.cs:153-171`
    pub(crate) fn read(&self, into: &mut [u8]) -> Result<usize, &'static str> {
        let deadline = Instant::now() + Duration::from_millis(u64::from(TIMEOUT_MS));
        while self.buffered() == 0 {
            self.get_data(false);
            wasm_thread::sleep(READ_SLEEP);
            if Instant::now() > deadline {
                return Err(READ_TIMEOUT);
            }
        }
        Ok(self
            .buffer
            .os_lock()
            .map_or(0, |mut buffer| buffer.get(into)))
    }

    /// `Write(write, offset, length)`: the bytes sent with timeout 0.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkSerialPort.cs:173-180`
    pub(crate) fn write(&self, data: &[u8]) {
        self.send_serial_control(0, data, 0, false);
    }

    /// `BytesToRead`: a request when one is due, then what the buffer holds.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkSerialPort.cs:269-279`
    pub(crate) fn bytes_to_read(&self) -> usize {
        self.get_data(false);
        self.buffered()
    }

    /// `Open()`: the request with the timeout and `baud` - which nothing here sets, so 0, no
    /// change - then `Open`'s wait, and open; a `Close` during the wait leaves it closed.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkSerialPort.cs:198-204`
    pub(crate) fn open(&self) {
        log::info!("MAVLinkSerialPort Open");
        self.send_serial_control(TIMEOUT_MS, &[], 0, false);
        let until = Instant::now() + self.open_wait;
        loop {
            if self.ended.load(Ordering::Acquire) {
                return;
            }
            let now = Instant::now();
            if now >= until {
                break;
            }
            wasm_thread::sleep((until - now).min(OPEN_POLL));
        }
        self.open.store(true, Ordering::Release);
    }

    /// `IsOpen`. `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkSerialPort.cs:292-295`
    pub(crate) fn is_open(&self) -> bool {
        self.open.load(Ordering::Acquire)
    }

    /// `Close()`: closed, its loop ended, and the close flag sent - flags 0, the port handed back
    /// to its driver. Once.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkSerialPort.cs:182-191`
    pub(crate) fn close(&self) {
        self.open.store(false, Ordering::Release);
        if self.ended.swap(true, Ordering::AcqRel) {
            return;
        }
        log::info!("MAVLinkSerialPort Close");
        self.send_serial_control(0, &[], 0, true);
    }
}

/// `mainloop`: `GetData()` every 5 ms - which asks only while the port is open - until `Close`,
/// or until the port is let go, as its finalizer aborts the thread.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkSerialPort.cs:80-102`
fn main_loop(port: &Weak<SerialPort>) {
    loop {
        let Some(port) = port.upgrade() else {
            return;
        };
        if port.ended.load(Ordering::Acquire) {
            return;
        }
        port.get_data(false);
        drop(port);
        wasm_thread::sleep(LOOP_SLEEP);
    }
}

/// What `SubscribeToPacketType(SERIAL_CONTROL, ReceviedPacket, sysidcurrent, compidcurrent, true)`
/// hands the port: a `SERIAL_CONTROL` the link read from `vehicle` - not one it sent.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkSerialPort.cs:68-72`
fn reply_of(packet: &Packet, vehicle: VehicleId) -> Option<&SerialControl> {
    if packet.sent || VehicleId::new(packet.sysid, packet.compid) != vehicle {
        return None;
    }
    match &packet.message {
        MavMessage::SerialControl(reply) => Some(reply),
        _ => None,
    }
}

// ---------------------------------------------------------------------------------------------
// The bridge.
// ---------------------------------------------------------------------------------------------

/// `but_mavserialport_Click`'s passthrough while it runs: `comport`, `listener` and `client`.
/// `// C#: temp.cs:54-58`
pub(crate) struct Pass {
    /// `comport`.
    port: Arc<SerialPort>,
    /// Where `listener` listens.
    address: SocketAddr,
    /// Set as it is closed: `listener.Stop()` and `client.Close()`.
    stop: Arc<AtomicBool>,
    /// The listener's thread, which ends with it, and its client's with it.
    accepting: Option<wasm_thread::JoinHandle<()>>,
    /// The subscription that hands the port the vehicle's replies, ended with it.
    _heard: Option<PacketSubscription>,
}

impl Pass {
    /// `listener = new TcpListener(address); listener.Start();
    /// listener.BeginAcceptTcpClient(...)` for `port`: the listener's thread started.
    ///
    /// # Errors
    /// The listener refused - the address held, or one below 1024 without root on Linux - or its
    /// thread not started.
    /// `// C#: temp.cs:331-341`
    pub(crate) fn start(
        port: Arc<SerialPort>,
        address: SocketAddr,
        heard: Option<PacketSubscription>,
    ) -> std::io::Result<Self> {
        let listener = TcpListener::bind(address)?;
        listener.set_nonblocking(true)?;
        let address = listener.local_addr()?;
        let stop = Arc::new(AtomicBool::new(false));
        let accepting = {
            let (port, stop) = (Arc::clone(&port), Arc::clone(&stop));
            wasm_thread::Builder::new()
                .name("mavserial-listener".to_owned())
                .spawn(move || accept(&listener, &port, &stop, address))?
        };
        note(format!("listening on {address}"));
        Ok(Self {
            port,
            address,
            stop,
            accepting: Some(accepting),
            _heard: heard,
        })
    }

    /// Where it listens.
    pub(crate) const fn address(&self) -> SocketAddr {
        self.address
    }

    /// The second press: `comport.Close()`, `client.Close()`, `listener.Stop()` - the port handed
    /// back, and the client and the listener closed before it returns.
    /// `// C#: temp.cs:312-324`
    pub(crate) fn close(mut self) {
        self.shut();
        note("closed");
    }

    /// The closing, once.
    fn shut(&mut self) {
        let Some(accepting) = self.accepting.take() else {
            return;
        };
        self.port.close();
        self.stop.store(true, Ordering::Release);
        let _ = accepting.join();
    }
}

impl Drop for Pass {
    fn drop(&mut self) {
        self.shut();
    }
}

/// `DoAcceptTcpClientCallback`, a loop where the C#'s is a callback: each client accepted - one
/// while another is connected turned away (`if (client != null && client.Connected) return;`),
/// else served on a thread of its own ([`serve`]) until it goes. Ends once `stop` is set, the
/// listener closed with it, after its client's thread.
/// `// C#: temp.cs:349-408`
fn accept(
    listener: &TcpListener,
    port: &Arc<SerialPort>,
    stop: &Arc<AtomicBool>,
    address: SocketAddr,
) {
    let connected = Arc::new(AtomicBool::new(false));
    let mut serving: Option<wasm_thread::JoinHandle<()>> = None;
    while !stop.load(Ordering::Acquire) {
        match listener.accept() {
            Ok((stream, peer)) => {
                if connected.swap(true, Ordering::AcqRel) {
                    log::info!("{NAME}: {peer} turned away, a client is connected");
                    drop(stream);
                    continue;
                }
                if let Some(done) = serving.take() {
                    let _ = done.join();
                }
                note(format!("serving {peer}"));
                let (port, stop) = (Arc::clone(port), Arc::clone(stop));
                let gone = Arc::clone(&connected);
                let spawned = wasm_thread::Builder::new()
                    .name("mavserial-client".to_owned())
                    .spawn(move || {
                        serve(stream, &port, &stop);
                        if !stop.load(Ordering::Acquire) {
                            note(format!("listening on {address}"));
                        }
                        gone.store(false, Ordering::Release);
                    });
                match spawned {
                    Ok(handle) => serving = Some(handle),
                    Err(error) => {
                        log::debug!("{NAME}: {error}");
                        connected.store(false, Ordering::Release);
                    }
                }
            }
            Err(error) => {
                if error.kind() != ErrorKind::WouldBlock {
                    log::debug!("{NAME}: {error}");
                }
                wasm_thread::sleep(ACCEPT_POLL);
            }
        }
    }
    if let Some(done) = serving.take() {
        let _ = done.join();
    }
}

/// One client served: the port opened if it is not (`if (!comport.IsOpen) comport.Open()`), then,
/// while the client is there and the port open, what the client sent written to the port and
/// what the port read written to the client, a millisecond's sleep between rounds.
/// `// C#: temp.cs:362-403`
fn serve(mut stream: TcpStream, port: &SerialPort, stop: &AtomicBool) {
    if stream.set_nonblocking(true).is_err() {
        return;
    }
    if !port.is_open() {
        port.open();
    }
    let mut data = vec![0u8; BRIDGE_BUFFER];
    while !stop.load(Ordering::Acquire) && port.is_open() {
        // `while (stream.DataAvailable)`: what the client sent, to the port - until it stops, or
        // the port is closed under a client that never does.
        loop {
            if stop.load(Ordering::Acquire) || !port.is_open() {
                return;
            }
            match stream.read(&mut data) {
                Ok(0) => return,
                Ok(len) => port.write(data.get(..len).unwrap_or_default()),
                Err(error) if error.kind() == ErrorKind::WouldBlock => break,
                Err(error) if error.kind() == ErrorKind::Interrupted => {}
                Err(_) => return,
            }
        }
        // `while (comport.BytesToRead > 0)`: what the vehicle sent, to the client; a read that
        // times out passed over, as the C#'s `catch` passes it.
        while port.bytes_to_read() > 0 {
            if let Ok(len) = port.read(&mut data)
                && write_all(&mut stream, data.get(..len).unwrap_or_default(), stop).is_err()
            {
                return;
            }
        }
        wasm_thread::sleep(BRIDGE_SLEEP);
    }
}

/// `stream.Write(data, 0, len)` on a socket that does not block: every byte written, a
/// millisecond's wait while its buffer is full; an error, or `stop`, ends it.
fn write_all(stream: &mut TcpStream, mut data: &[u8], stop: &AtomicBool) -> std::io::Result<()> {
    while !data.is_empty() {
        if stop.load(Ordering::Acquire) {
            return Err(ErrorKind::Interrupted.into());
        }
        match stream.write(data) {
            Ok(0) => return Err(ErrorKind::WriteZero.into()),
            Ok(written) => data = data.get(written..).unwrap_or_default(),
            Err(error) if error.kind() == ErrorKind::WouldBlock => wasm_thread::sleep(BRIDGE_SLEEP),
            Err(error) if error.kind() == ErrorKind::Interrupted => {}
            Err(error) => return Err(error),
        }
    }
    Ok(())
}

// ---------------------------------------------------------------------------------------------
// The button.
// ---------------------------------------------------------------------------------------------

/// The passthrough while it runs: the C#'s statics, which outlive the form.
/// `// C#: temp.cs:54-58`
static PASS: Mutex<Option<Pass>> = Mutex::new(None);

/// Why `new MAVLinkSerialPort(...)` throws: no link open (`!mavint.BaseStream.IsOpen`), else no
/// vehicle heard on it (`mavint.getHeartBeat().Length == 0`); otherwise the vehicle.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkSerialPort.cs:58-66`
fn refusal<T>(open: bool, vehicle: Option<T>) -> Result<T, &'static str> {
    if !open {
        return Err(PLEASE_CONNECT);
    }
    vehicle.ok_or(NO_HEARTBEATS)
}

/// What the status line says of a listener refused at `address`: the C#'s exception, in its box.
fn refused(address: SocketAddr, error: &std::io::Error) -> String {
    format!("{address}: {error}")
}

/// `comport = new MAVLinkSerialPort(MainV2.comPort, GPS1)`, its subscription to the vehicle
/// shown, then the listener on 0.0.0.0:500; or why not.
/// `// C#: temp.cs:327-341`
fn start(telemetry: &crate::telemetry::Telemetry) -> Result<Pass, String> {
    let open = telemetry.is_open() && !telemetry.view().reconnecting;
    let (sender, vehicle) = refusal(open, telemetry.send_handle())?;
    let port = SerialPort::new(Box::new(sender), SERIAL_CONTROL_DEV_GPS1, OPEN_WAIT);
    let hearing = Arc::clone(&port);
    let heard = telemetry.on_packet(move |packet| {
        if let Some(reply) = reply_of(packet, vehicle) {
            hearing.received(reply);
        }
    });
    Pass::start(port, LISTEN, heard).map_err(|error| refused(LISTEN, &error))
}

/// `but_mavserialport_Click`: with the passthrough running, closed - the port handed back, the
/// client and the listener closed; else started. What the C# shows in a box is said on the
/// status line as an error; listening and closed, quietly.
/// `// C#: temp.cs:310-347`
pub(crate) fn press(this: &mut MissionPlanner) {
    let Ok(mut pass) = PASS.os_lock() else {
        return;
    };
    if let Some(running) = pass.take() {
        running.close();
        this.file_status = Some(format!("{NAME}: closed"));
        return;
    }
    match start(&this.telemetry) {
        Ok(running) => {
            this.file_status = Some(format!("{NAME}: listening on {}", running.address()));
            *pass = Some(running);
        }
        Err(why) => {
            log::info!("{NAME}: {why}");
            note(format!("refused: {why}"));
            this.file_status = Some(error_box(why));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::experimental::{Act, Tool};
    use mp_link::commands::{
        SERIAL_CONTROL_FLAG_EXCLUSIVE, SERIAL_CONTROL_FLAG_MULTI, SERIAL_CONTROL_FLAG_RESPOND,
    };

    /// A link that keeps every `SERIAL_CONTROL` sent on it.
    #[derive(Default)]
    struct Fake(Mutex<Vec<SerialControl>>);

    impl Fake {
        fn sent(&self) -> Vec<SerialControl> {
            self.0.os_lock().unwrap().clone()
        }
    }

    impl SerialLink for Arc<Fake> {
        fn send(&self, message: &MavMessage) -> bool {
            if let MavMessage::SerialControl(control) = message {
                self.0.os_lock().unwrap().push(*control);
            }
            true
        }
    }

    /// A port on GPS1 over a fake link, its `Open` waiting 20 ms.
    fn port() -> (Arc<Fake>, Arc<SerialPort>) {
        let fake = Arc::new(Fake::default());
        let port = SerialPort::new(
            Box::new(Arc::clone(&fake)),
            SERIAL_CONTROL_DEV_GPS1,
            Duration::from_millis(20),
        );
        (fake, port)
    }

    /// The vehicle's `SERIAL_CONTROL` carrying `bytes`: `REPLY`, as ArduPilot answers.
    fn reply(bytes: &[u8]) -> SerialControl {
        let mut data = [0u8; 70];
        data[..bytes.len()].copy_from_slice(bytes);
        SerialControl {
            baudrate: 0,
            timeout: 0,
            device: SERIAL_CONTROL_DEV_GPS1,
            flags: 1,
            count: u8::try_from(bytes.len()).unwrap(),
            data,
        }
    }

    /// A request as `GetData` and `Open` send it: `EXCLUSIVE | RESPOND | MULTI`, the timeout, no
    /// bytes, no baud rate.
    fn is_request(control: &SerialControl) -> bool {
        let request =
            SERIAL_CONTROL_FLAG_EXCLUSIVE | SERIAL_CONTROL_FLAG_RESPOND | SERIAL_CONTROL_FLAG_MULTI;
        control.flags == request
            && control.timeout == TIMEOUT_MS
            && control.count == 0
            && control.baudrate == 0
            && control.device == SERIAL_CONTROL_DEV_GPS1
    }

    /// Whether `test` comes true within two seconds.
    fn within(test: impl Fn() -> bool) -> bool {
        let deadline = Instant::now() + Duration::from_secs(2);
        while Instant::now() < deadline {
            if test() {
                return true;
            }
            wasm_thread::sleep(Duration::from_millis(2));
        }
        test()
    }

    /// The button works here, and says what the C# says: `Strings.PleaseConnect` without a link,
    /// `Strings.No_valid_heartbeats_read_from_port` with one and no vehicle, `Read`'s timeout,
    /// and the listener's address, `IPAddress.Any` and 500.
    /// `// C#: temp.cs:329, 337; MAVLinkSerialPort.cs:58-66, 164; ExtLibs/Strings/Strings.resx:205-207, 627-629`
    #[test]
    fn the_button_is_live_and_says_what_the_csharp_says() {
        assert_eq!(
            crate::experimental::tool("but_mavserialport"),
            Tool::Act(Act::MavSerialPass)
        );
        assert_eq!(PLEASE_CONNECT, "Please connect first");
        assert_eq!(NO_HEARTBEATS, "No valid heartbeats read from port");
        assert_eq!(READ_TIMEOUT, "MAVLinkSerialPort Timeout on read");
        assert_eq!(LISTEN.to_string(), "0.0.0.0:500");
        assert_eq!(refusal(false, None::<u8>), Err(PLEASE_CONNECT));
        assert_eq!(refusal(false, Some(1)), Err(PLEASE_CONNECT));
        assert_eq!(refusal(true, None::<u8>), Err(NO_HEARTBEATS));
        assert_eq!(refusal(true, Some(1)), Ok(1));
        assert_eq!(error_box(PLEASE_CONNECT), "Error: Please connect first");
    }

    /// The buffer's `Put` and `Get` as the C#'s: in order, a `Get` taking what it holds at most;
    /// and overflowing as the C#'s overflows, the size held at the capacity and the head left
    /// where it was - a ring of four given 1, 2, 3 and then 4, 5, 6 gives 5, 6, 3, 4, and one
    /// given ten bytes at once gives 9, 10, 7, 8.
    /// `// C#: ExtLibs/Utilities/CircularBuffer.cs:99-169`
    #[test]
    fn the_buffer_puts_gets_and_overflows_as_the_csharps() {
        let mut ring = Ring::new(4);
        ring.put(&[1, 2, 3]);
        let mut two = [0u8; 2];
        assert_eq!(ring.get(&mut two), 2);
        assert_eq!(two, [1, 2]);
        ring.put(&[4, 5, 6]);
        assert_eq!(ring.size(), 4);
        let mut eight = [0u8; 8];
        assert_eq!(ring.get(&mut eight), 4);
        assert_eq!(&eight[..4], &[3, 4, 5, 6]);
        assert_eq!(ring.get(&mut eight), 0);

        let mut ring = Ring::new(4);
        ring.put(&[1, 2, 3]);
        ring.put(&[4, 5, 6]);
        assert_eq!(ring.size(), 4);
        let mut four = [0u8; 4];
        assert_eq!(ring.get(&mut four), 4);
        assert_eq!(four, [5, 6, 3, 4]);

        let mut ring = Ring::new(4);
        ring.put(&[1, 2, 3, 4, 5, 6, 7, 8, 9, 10]);
        assert_eq!(ring.get(&mut four), 4);
        assert_eq!(four, [9, 10, 7, 8]);

        assert_eq!(Ring::new(BUFFER_CAPACITY).buffer.len(), 4096);
    }

    /// `GetData` asks again once more than `timeout`, 10 ms, has passed since it last asked, and
    /// at once the first time (`DateTime.MinValue`).
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkSerialPort.cs:142-151`
    #[test]
    fn get_data_asks_once_its_timeout_has_passed() {
        let then = Instant::now();
        assert!(due(None, then));
        assert!(!due(Some(then), then));
        assert!(!due(Some(then), then + Duration::from_millis(10)));
        assert!(due(Some(then), then + Duration::from_millis(11)));
    }

    /// What the port sends, as the C# sends it: nothing while closed (its loop asks only while
    /// open); `Open`'s request, then the loop's every 10 ms; `Write`'s bytes with timeout 0,
    /// `EXCLUSIVE | RESPOND`, 10 ms before each message; and `Close`'s flags 0, after which
    /// nothing - the loop ended, a write dropped.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkSerialPort.cs:80-93, 144-151, 173-204; MAVLinkInterface.cs:2965-3016`
    #[test]
    fn the_port_sends_what_the_csharp_sends() {
        let (fake, port) = port();
        wasm_thread::sleep(Duration::from_millis(30));
        assert!(fake.sent().is_empty(), "the loop asks nothing while closed");
        assert!(!port.is_open());

        // A write before the port is open goes out all the same, as the C#'s does.
        let start = Instant::now();
        let data: Vec<u8> = (0..75).collect();
        port.write(&data);
        assert!(
            start.elapsed() >= Duration::from_millis(20),
            "10 ms before each message"
        );
        let sent = fake.sent();
        assert_eq!(sent.len(), 2);
        assert_eq!((sent[0].flags, sent[0].count, sent[0].timeout), (4, 70, 0));
        assert_eq!((sent[1].flags, sent[1].count, sent[1].timeout), (6, 5, 0));
        assert_eq!(&sent[0].data[..], &data[..70]);
        assert_eq!(&sent[1].data[..5], &data[70..]);

        port.open();
        assert!(port.is_open());
        let sent = fake.sent();
        assert!(is_request(&sent[2]), "Open's request: {:?}", sent[2]);
        assert!(
            within(|| fake.sent().len() >= 6 && fake.sent()[2..].iter().all(is_request)),
            "the loop asks while open"
        );

        port.close();
        assert!(!port.is_open());
        let closed = fake.sent();
        let last = closed.last().unwrap();
        assert_eq!(
            (last.flags, last.timeout, last.count, last.baudrate),
            (0, 0, 0, 0)
        );
        port.write(b"after");
        port.close();
        wasm_thread::sleep(Duration::from_millis(30));
        assert_eq!(
            fake.sent().len(),
            closed.len(),
            "nothing after the close flag"
        );
    }

    /// The vehicle's replies: one with no bytes passed over; one with bytes buffered, and a
    /// request sent at once, open or not; `Read` takes what is there, `into`'s length at most,
    /// and with nothing there waits `timeout` and fails with the C#'s words.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkSerialPort.cs:108-171, 269-279`
    #[test]
    fn a_reply_is_buffered_and_read() {
        let (fake, port) = port();
        port.received(&reply(&[]));
        assert_eq!(port.bytes_to_read(), 0);
        assert!(fake.sent().is_empty());

        port.received(&reply(b"hello"));
        let sent = fake.sent();
        assert_eq!(sent.len(), 1);
        assert!(is_request(&sent[0]), "asked again at once: {:?}", sent[0]);
        assert_eq!(port.bytes_to_read(), 5);
        let mut three = [0u8; 3];
        assert_eq!(port.read(&mut three), Ok(3));
        assert_eq!(&three, b"hel");
        assert_eq!(port.read(&mut three), Ok(2));
        assert_eq!(&three[..2], b"lo");

        let start = Instant::now();
        assert_eq!(port.read(&mut three), Err(READ_TIMEOUT));
        assert!(start.elapsed() >= Duration::from_millis(10));
    }

    /// The subscription's filter: a `SERIAL_CONTROL` read from the vehicle shown, not one the
    /// link sent, nor one from another vehicle, nor another message.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkSerialPort.cs:71-72`
    #[test]
    fn the_port_hears_its_vehicles_serial_control() {
        let vehicle = VehicleId::new(1, 1);
        let packet = |sysid: u32, sent: bool, message: MavMessage| Packet {
            sysid,
            compid: 1,
            msgid: message.id(),
            message,
            length: 0,
            rxtime: mp_vehicle::DateTime::now(),
            at: Instant::now(),
            sent,
        };
        let answer = reply(b"$GP");
        let heard = packet(1, false, MavMessage::SerialControl(answer));
        assert_eq!(reply_of(&heard, vehicle), Some(&answer));
        assert_eq!(
            reply_of(&packet(1, true, MavMessage::SerialControl(answer)), vehicle),
            None
        );
        assert_eq!(
            reply_of(
                &packet(2, false, MavMessage::SerialControl(answer)),
                vehicle
            ),
            None
        );
        let heartbeat = MavMessage::decode(0, &[0; 9]).unwrap();
        assert_eq!(reply_of(&packet(1, false, heartbeat), vehicle), None);
    }

    /// The bridge, over a fake link and a listener on this machine: the first client opens the
    /// port, its bytes go to the vehicle and the vehicle's come back to it; a second while it is
    /// connected is closed; pressed again, the close flag is sent and the client and the listener
    /// are closed.
    /// `// C#: temp.cs:312-324, 349-408`
    #[test]
    fn the_bridge_carries_both_ways_one_client_at_a_time() {
        let (fake, port) = port();
        let pass = Pass::start(Arc::clone(&port), "127.0.0.1:0".parse().unwrap(), None).unwrap();
        let address = pass.address();
        assert_ne!(address.port(), 0);

        let mut client = TcpStream::connect(address).unwrap();
        client
            .set_read_timeout(Some(Duration::from_secs(2)))
            .unwrap();
        assert!(within(|| port.is_open()), "the first client opens the port");
        assert!(is_request(&fake.sent()[0]), "Open's request");

        client.write_all(b"$PUBX,00*33\r\n").unwrap();
        assert!(
            within(|| fake.sent().iter().any(|control| control.count > 0)),
            "the client's bytes sent"
        );
        let written: Vec<SerialControl> = fake
            .sent()
            .into_iter()
            .filter(|control| control.count > 0)
            .collect();
        assert_eq!(written.len(), 1);
        assert_eq!(
            (written[0].flags, written[0].timeout, written[0].count),
            (6, 0, 13)
        );
        assert_eq!(&written[0].data[..13], b"$PUBX,00*33\r\n");

        port.received(&reply(b"\xb5\x62ubx"));
        let mut answer = [0u8; 5];
        client.read_exact(&mut answer).unwrap();
        assert_eq!(&answer, b"\xb5\x62ubx");

        let mut second = TcpStream::connect(address).unwrap();
        second
            .set_read_timeout(Some(Duration::from_secs(2)))
            .unwrap();
        let mut byte = [0u8; 1];
        match second.read(&mut byte) {
            Ok(0) => {}
            Err(error) if error.kind() == ErrorKind::ConnectionReset => {}
            other => panic!("the second client was not turned away: {other:?}"),
        }

        pass.close();
        assert!(!port.is_open());
        let closed = fake.sent();
        assert_eq!(closed.last().map(|control| control.flags), Some(0));
        match client.read(&mut byte) {
            Ok(0) => {}
            Err(error) if error.kind() == ErrorKind::ConnectionReset => {}
            other => panic!("the client was not closed: {other:?}"),
        }
        assert!(TcpStream::connect(address).is_err(), "the listener closed");
        wasm_thread::sleep(Duration::from_millis(30));
        assert_eq!(
            fake.sent().len(),
            closed.len(),
            "nothing after the close flag"
        );
    }

    /// A listener refused - here its address held - is said with the address, and the port made
    /// for it has sent nothing.
    /// `// C#: temp.cs:337-346`
    #[test]
    fn a_refused_listener_says_why_and_sends_nothing() {
        let holder = TcpListener::bind("127.0.0.1:0").unwrap();
        let held = holder.local_addr().unwrap();
        let (fake, port) = port();
        let why = Pass::start(port, held, None)
            .map(|_| ())
            .map_err(|error| refused(held, &error))
            .unwrap_err();
        assert!(why.starts_with(&format!("{held}: ")), "{why}");
        wasm_thread::sleep(Duration::from_millis(30));
        assert!(fake.sent().is_empty());
    }
}
