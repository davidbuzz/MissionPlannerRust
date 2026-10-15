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

//! The built-in Tailscale node itself, with the `node` feature (the owner's feature of
//! 2026-10-14, not in the C#): tailscale-rs's `Device` on a tokio runtime of the node's own
//! thread, its answers turned into [`State`], and the planner's blocking calls carried to that
//! runtime and back.
//!
//! From blocking to async and back: a call spawns its work on the node's runtime and waits on a
//! channel for the answer, never past its deadline - and the work's own timeout, on the runtime,
//! ends it there at the same moment. A planner thread never drives the runtime itself: tokio's
//! `Handle::block_on` panics inside a runtime, and when a timer outlives the runtime's shutdown,
//! and a stop may come in the middle of a call. A socket's reads come from a task that owns its
//! receiving end, so a read that times out abandons nothing (tailscale-rs's receive takes the
//! bytes out of its socket before it answers, so a receive cancelled midway would lose them); its
//! writes go straight to the network stack's own blocking calls, which wait on channels, never on
//! a runtime. The runtime's threads never wait for a planner thread, so neither can hold up the
//! other for longer than a call's timeout.

use std::future::Future;
use std::io;
use std::net::{IpAddr, SocketAddr};
use std::path::Path;
use std::sync::mpsc::{self, Receiver, RecvTimeoutError};
use std::sync::{Arc, Condvar, Mutex, MutexGuard, PoisonError};
use std::time::Duration;
use std::{fmt, str::FromStr};

use mp_os::{Lock as _, RecvTimeout as _};
use tokio::runtime::Handle;
use tokio::sync::watch;
use web_time::Instant;

use crate::{Error, KEY_FILE, NO_KEY_FOLDER, Settings, State, state_after, wait_verdict};

/// How long a stop gives the device, then its runtime, to close down before they are dropped.
const SHUTDOWN: Duration = Duration::from_secs(2);

/// How often the node asks its device how it stands while it is not running...
const POLL_STARTING: Duration = Duration::from_secs(1);

/// ...and once it is.
const POLL_RUNNING: Duration = Duration::from_secs(5);

/// How much longer than its work's own timeout a call waits for the answer: past that the
/// runtime has gone, and the work with it.
const GRACE: Duration = Duration::from_millis(500);

/// The runtime's worker threads: the device's actors and the sockets' readers are light work.
const WORKERS: usize = 2;

/// What the node tells the control server it is (`Hostinfo.App`).
const CLIENT_NAME: &str = "MissionPlannerRust";

/// What a socket says once the node it came through has stopped.
const STOPPED: &str = "the built-in Tailscale node stopped";

/// The built-in Tailscale node: a handle, cloned freely. It runs from [`Node::start`] until
/// [`Node::stop`]; dropping the handles does not stop it.
#[derive(Clone)]
pub struct Node {
    inner: Arc<Inner>,
}

/// What the node's thread and its callers share.
struct Inner {
    shared: Mutex<Shared>,
    /// Woken at every change of `shared`.
    changed: Condvar,
}

/// The node as its thread last left it.
#[derive(Default)]
struct Shared {
    state: State,
    /// The device, while it is up and the node not stopping.
    device: Option<Arc<tailscale::Device>>,
    /// The runtime, while it runs.
    runtime: Option<Handle>,
    /// Tells the node's thread to stop, by being dropped: taken by the first stop.
    stop: Option<watch::Sender<bool>>,
    /// Asked to stop: from then on the node's thread has no say on the state but its last.
    stopping: bool,
}

impl Inner {
    fn lock(&self) -> MutexGuard<'_, Shared> {
        self.shared.os_lock().unwrap_or_else(PoisonError::into_inner)
    }

    /// The node's thread's word on how it stands, taken unless the node is stopping.
    fn set_state(&self, state: State) {
        let mut shared = self.lock();
        if !shared.stopping && shared.state != state {
            shared.state = state;
            self.changed.notify_all();
        }
    }

    /// Anything else of the shared state changed.
    fn update(&self, change: impl FnOnce(&mut Shared)) {
        change(&mut self.lock());
        self.changed.notify_all();
    }

    /// The node's thread has ended: failed and why, or stopped. A node asked to stop is stopped
    /// whatever its thread last met.
    fn finish(&self, failed: Option<String>) {
        self.update(|shared| {
            shared.device = None;
            shared.runtime = None;
            shared.stop = None;
            shared.state = match failed {
                Some(why) if !shared.stopping => State::Failed { why },
                _ => State::Stopped,
            };
        });
    }

    /// The device and its runtime once the node runs, waited for until `deadline` at the latest;
    /// why not, if it does not run by then or will not run at all.
    fn running(&self, deadline: Instant) -> Result<(Arc<tailscale::Device>, Handle), Error> {
        let mut shared = self.lock();
        loop {
            let time_up = Instant::now() >= deadline;
            match wait_verdict(&shared.state, time_up) {
                Some(Ok(())) => {
                    if let (Some(device), Some(runtime)) = (&shared.device, &shared.runtime) {
                        return Ok((Arc::clone(device), runtime.clone()));
                    }
                    // Running in name only: stopping. Its end is a moment away.
                    if time_up {
                        return Err(Error::NotRunning(State::Stopped.to_string()));
                    }
                }
                Some(Err(err)) => return Err(err),
                None => {}
            }
            let left = deadline.saturating_duration_since(Instant::now());
            shared = self
                .changed
                .wait_timeout(shared, left)
                .map_or_else(|poisoned| poisoned.into_inner().0, |(guard, _)| guard);
        }
    }
}

impl Node {
    /// Starts the node on a thread of its own, at once: it reads its keys from [`KEY_FILE`] in
    /// `settings.state_dir` (making them the first time), reaches the control server, and runs
    /// until [`Node::stop`]. [`Node::state`] says how far it has got.
    #[must_use]
    pub fn start(settings: Settings) -> Self {
        let (stop, stop_asked) = watch::channel(false);
        let inner = Arc::new(Inner {
            shared: Mutex::new(Shared {
                state: State::Starting,
                stop: Some(stop),
                ..Shared::default()
            }),
            changed: Condvar::new(),
        });
        if settings.state_dir.as_os_str().is_empty() {
            inner.finish(Some(NO_KEY_FOLDER.to_owned()));
            return Self { inner };
        }
        let thread = Arc::clone(&inner);
        let started = wasm_thread::Builder::new()
            .name("mp-tailnet".to_owned())
            .spawn(move || run(&thread, &settings, stop_asked));
        if let Err(err) = started {
            inner.finish(Some(format!("could not start its thread: {err}")));
        }
        Self { inner }
    }

    /// How the node stands now. Never waits.
    #[must_use]
    pub fn state(&self) -> State {
        self.inner.lock().state.clone()
    }

    /// Whether it is on the tailnet now.
    #[must_use]
    pub fn is_running(&self) -> bool {
        self.inner.lock().state.is_running()
    }

    /// Stops the node: at once for its callers, whose calls from now on fail, and within a few
    /// seconds on its thread, which closes the device and then the runtime - and with it every
    /// socket that came through it.
    pub fn stop(&self) {
        self.inner.update(|shared| {
            shared.stopping = true;
            shared.stop = None;
            shared.device = None;
            shared.state = State::Stopped;
        });
    }

    /// Resolves a host as Go's tsnet does when it dials one: an IP address is itself; a MagicDNS
    /// name - a node's full name, or the short name of a node in this node's own tailnet, in any
    /// case, a trailing dot or none - is that node's address, IPv4 first. Waits no longer than
    /// `timeout` for the node to run and its network map to answer.
    /// `// Go: net/tsdial/tsdial.go:401-415; ipn/ipnlocal/resolve.go:60-77, 112-144` (in the
    /// vendored tailscale-rs's `Device::resolve`)
    ///
    /// # Errors
    /// [`Error::Unresolved`] for a name the tailnet does not have; the node's own state when it
    /// does not run in time; [`Error::Timeout`].
    pub fn resolve(&self, host: &str, timeout: Duration) -> Result<IpAddr, Error> {
        // Go: net/tsdial/tsdial.go:406-408 - an address needs no node.
        if let Ok(ip) = host.parse::<IpAddr>() {
            return Ok(ip);
        }
        let deadline = Instant::now() + timeout;
        let (device, runtime) = self.inner.running(deadline)?;
        let host = host.to_owned();
        call(&runtime, deadline, async move {
            device
                .resolve(&host)
                .await
                .map_err(|err| tailscale_error(err, &host))
        })
    }

    /// A TCP connection to a host's port over the tailnet, the host resolved as
    /// [`Node::resolve`] resolves it. Waits no longer than `timeout` in all for the node to run,
    /// the name to resolve and the connection to open.
    ///
    /// # Errors
    /// As [`Node::resolve`]; tailscale-rs's own when the connection fails.
    pub fn connect_tcp(
        &self,
        host: &str,
        port: u16,
        timeout: Duration,
    ) -> Result<TcpStream, Error> {
        let deadline = Instant::now() + timeout;
        let (device, runtime) = self.inner.running(deadline)?;
        let host = host.to_owned();
        let stream = call(&runtime, deadline, async move {
            device
                .tcp_connect_host(&host, port)
                .await
                .map_err(|err| tailscale_error(err, &host))
        })?;
        Ok(TcpStream::new(stream, &runtime))
    }

    /// A UDP socket to send to a host's port over the tailnet and to read what comes back, bound
    /// to an ephemeral port of the node's own, the host resolved as [`Node::resolve`] resolves
    /// it. Waits no longer than `timeout` in all.
    ///
    /// # Errors
    /// As [`Node::resolve`]; tailscale-rs's own when no port can be bound.
    pub fn bind_udp(&self, host: &str, port: u16, timeout: Duration) -> Result<UdpSocket, Error> {
        let deadline = Instant::now() + timeout;
        let (device, runtime) = self.inner.running(deadline)?;
        let host = host.to_owned();
        let (socket, remote) = call(&runtime, deadline, async move {
            device
                .udp_bind_for_host(&host, port)
                .await
                .map_err(|err| tailscale_error(err, &host))
        })?;
        Ok(UdpSocket::new(socket, remote, &runtime))
    }
}

impl fmt::Debug for Node {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Node")
            .field("state", &self.state())
            .finish_non_exhaustive()
    }
}

/// The node's thread: its runtime, the device on it until a stop, and the runtime's end.
fn run(inner: &Inner, settings: &Settings, stop: watch::Receiver<bool>) {
    // tailscale-rs makes its TLS configurations from rustls's process default, which rustls picks
    // for itself only when one provider is built in; a planner with the node has two - ureq's
    // ring, which ureq names wherever it uses it, and tailscale-rs's aws-lc-rs - and rustls's
    // `ClientConfig::builder` would panic. So the node names the one tailscale-rs is built for,
    // once a process: a second start finds it named.
    let _ = rustls::crypto::aws_lc_rs::default_provider().install_default();
    let runtime = match tokio::runtime::Builder::new_multi_thread()
        .worker_threads(WORKERS)
        .thread_name("mp-tailnet-runtime")
        .enable_all()
        .build()
    {
        Ok(runtime) => runtime,
        Err(err) => {
            inner.finish(Some(format!("could not start its runtime: {err}")));
            return;
        }
    };
    inner.update(|shared| shared.runtime = Some(runtime.handle().clone()));
    let failed = runtime.block_on(drive(inner, settings, stop));
    // No call starts on the runtime from here; those under way end with it.
    inner.update(|shared| {
        shared.device = None;
        shared.runtime = None;
    });
    runtime.shutdown_timeout(SHUTDOWN);
    inner.finish(failed);
}

/// The device's life on the runtime: made, asked how it stands until a stop, and shut down.
/// `Some` and why when it fails; `None` when it was stopped.
async fn drive(
    inner: &Inner,
    settings: &Settings,
    mut stop: watch::Receiver<bool>,
) -> Option<String> {
    let config = tokio::select! {
        () = stop_asked(&mut stop) => return None,
        config = node_config(settings) => config,
    };
    let config = match config {
        Ok(config) => config,
        Err(why) => return Some(why),
    };
    let device = tokio::select! {
        () = stop_asked(&mut stop) => return None,
        device = tailscale::Device::new(&config, None) => device,
    };
    let device = match device {
        Ok(device) => Arc::new(device),
        Err(err) => return Some(format!("could not start: {err}")),
    };
    inner.update(|shared| {
        if !shared.stopping {
            shared.device = Some(Arc::clone(&device));
        }
    });
    loop {
        let pause = tokio::select! {
            () = stop_asked(&mut stop) => break,
            pause = poll(inner, &device) => pause,
        };
        tokio::select! {
            () = stop_asked(&mut stop) => break,
            () = tokio::time::sleep(pause) => {}
        }
    }
    inner.update(|shared| shared.device = None);
    // Gracefully, unless a call still holds the device: then the runtime's end ends it.
    if let Ok(device) = Arc::try_unwrap(device) {
        device.shutdown(Some(SHUTDOWN)).await;
    }
    None
}

/// Until a stop is asked: the value set, or the sender gone.
async fn stop_asked(stop: &mut watch::Receiver<bool>) {
    loop {
        let asked = *stop.borrow_and_update();
        if asked || stop.changed().await.is_err() {
            return;
        }
    }
}

/// How the node stands, asked of its device, and how long until it is asked again.
async fn poll(inner: &Inner, device: &tailscale::Device) -> Duration {
    let state = match device.is_authorized().await {
        Ok(tailscale::AuthState::NotAuthorized(url)) => state_after(Some(url.as_str()), None),
        Ok(tailscale::AuthState::Authorized) => {
            // Signed in: starting until the network map names this node, unless it already has.
            let running = inner.lock().state.is_running();
            if !running {
                inner.set_state(state_after(None, None));
            }
            match device.self_node().await {
                Ok(node) => {
                    let name = node.fqdn(false);
                    let addresses = &node.tailnet_address;
                    state_after(
                        None,
                        Some((name.as_str(), addresses.ipv4.addr(), addresses.ipv6.addr())),
                    )
                }
                Err(err) => State::Failed {
                    why: err.to_string(),
                },
            }
        }
        Err(err) => State::Failed {
            why: err.to_string(),
        },
    };
    let pause = if state.is_running() {
        POLL_RUNNING
    } else {
        POLL_STARTING
    };
    inner.set_state(state);
    pause
}

/// tailscale-rs's configuration from the settings: its keys from [`KEY_FILE`] in the state
/// directory, made there the first time and readable by this user alone; the node's name; the
/// control server.
async fn node_config(settings: &Settings) -> Result<tailscale::Config, String> {
    mp_os::fs::create_dir_all(&settings.state_dir)
        .map_err(|err| format!("{}: {err}", settings.state_dir.display()))?;
    let key_file = settings.state_dir.join(KEY_FILE);
    let mut config = tailscale::Config::default_with_key_file(&key_file)
        .await
        .map_err(|err| format!("{}: {err}", key_file.display()))?;
    keep_private(&key_file);
    config.client_name = Some(CLIENT_NAME.to_owned());
    config.requested_hostname = Some(settings.hostname.clone());
    if let Some(url) = &settings.control_url {
        config.control_server_url = control_url(url)?;
    }
    Ok(config)
}

/// A control server's address, as tailscale-rs takes it.
fn control_url<T: FromStr>(text: &str) -> Result<T, String>
where
    T::Err: fmt::Display,
{
    text.parse()
        .map_err(|err: T::Err| format!("the control server {text}: {err}"))
}

/// The key file readable and writable by this user alone: tailscale-rs writes it with the
/// default permissions, and the keys in it are the node's identity on the tailnet. Best effort -
/// a node whose key file cannot be narrowed runs as tailscale-rs's own would.
#[cfg(unix)]
fn keep_private(file: &Path) {
    use std::os::unix::fs::PermissionsExt as _;
    let _ = mp_os::fs::set_permissions(file, mp_os::fs::Permissions::from_mode(0o600));
}

/// Windows' own permissions on the user's folder are the key file's.
#[cfg(not(unix))]
fn keep_private(_file: &Path) {}

/// tailscale-rs's error as this crate's: a name it could not resolve, a timeout, or its own words.
fn tailscale_error(err: tailscale::Error, host: &str) -> Error {
    match err {
        tailscale::Error::Unresolved => Error::Unresolved(host.to_owned()),
        tailscale::Error::Timeout => Error::Timeout,
        other => Error::Tailscale(other.to_string()),
    }
}

/// `work` run on the node's runtime, its answer waited for here until `deadline` - and the work's
/// own timeout ends it on the runtime then too. A runtime gone meanwhile is a stopped node.
fn call<T, F>(runtime: &Handle, deadline: Instant, work: F) -> Result<T, Error>
where
    T: Send + 'static,
    F: Future<Output = Result<T, Error>> + Send + 'static,
{
    let left = deadline.saturating_duration_since(Instant::now());
    let (answer, answered) = mpsc::sync_channel(1);
    drop(runtime.spawn(async move {
        let result = tokio::time::timeout(left, work)
            .await
            .unwrap_or(Err(Error::Timeout));
        let _ = answer.send(result);
    }));
    match answered.os_recv_timeout(left + GRACE) {
        Ok(result) => result,
        Err(RecvTimeoutError::Timeout) => Err(Error::Timeout),
        Err(RecvTimeoutError::Disconnected) => Err(Error::NotRunning(State::Stopped.to_string())),
    }
}

/// What a TCP connection's reader hands over.
enum Incoming {
    /// Bytes, in order.
    Data(bytes::Bytes),
    /// The far end closed its side.
    End,
    /// The connection failed.
    Failed(io::Error),
}

/// A TCP connection over the tailnet, blocking as `std::net::TcpStream` is: a read waits no
/// longer than the read timeout ([`TcpStream::set_read_timeout`]) and then fails `TimedOut`; a
/// read of 0 is the far end's close; a write waits until the network stack has taken the bytes.
pub struct TcpStream {
    stream: Arc<tailscale::netstack::TcpStream>,
    incoming: Receiver<Incoming>,
    /// The task that receives into `incoming`, ended with the connection.
    reader: tokio::task::JoinHandle<()>,
    /// What the last chunk received had left after the last read.
    pending: bytes::Bytes,
    read_timeout: Option<Duration>,
    /// Nothing more will arrive: the far end closed, or the connection failed or was shut.
    ended: bool,
}

impl TcpStream {
    /// The connection, with its reader started on the node's runtime.
    fn new(stream: tailscale::netstack::TcpStream, runtime: &Handle) -> Self {
        let stream = Arc::new(stream);
        let (sender, incoming) = mpsc::channel();
        let receiving = Arc::clone(&stream);
        let reader = runtime.spawn(async move {
            loop {
                let next = match receiving.recv_bytes().await {
                    Ok(bytes) if bytes.is_empty() => Incoming::End,
                    Ok(bytes) => Incoming::Data(bytes),
                    Err(err) => Incoming::Failed(err.into()),
                };
                let last = !matches!(next, Incoming::Data(_));
                if sender.send(next).is_err() || last {
                    break;
                }
            }
        });
        Self {
            stream,
            incoming,
            reader,
            pending: bytes::Bytes::new(),
            read_timeout: None,
            ended: false,
        }
    }

    /// The local end: this node's tailnet address and an ephemeral port.
    #[must_use]
    pub fn local_addr(&self) -> SocketAddr {
        self.stream.local_addr()
    }

    /// The far end.
    #[must_use]
    pub fn peer_addr(&self) -> SocketAddr {
        self.stream.remote_addr()
    }

    /// How long a read waits for bytes before it fails `TimedOut`; `None`, for ever.
    pub fn set_read_timeout(&mut self, timeout: Option<Duration>) {
        self.read_timeout = timeout;
    }

    /// Closes it: nothing more is read, and the network stack closes the connection once its
    /// reader has let go.
    pub fn shutdown(&mut self) {
        self.reader.abort();
        self.ended = true;
        self.pending = bytes::Bytes::new();
    }
}

impl io::Read for TcpStream {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        if buf.is_empty() {
            return Ok(0);
        }
        if self.pending.is_empty() {
            if self.ended {
                return Ok(0);
            }
            let next = match self.read_timeout {
                Some(timeout) => self.incoming.os_recv_timeout(timeout),
                None => self
                    .incoming
                    .recv()
                    .map_err(|_| RecvTimeoutError::Disconnected),
            };
            match next {
                Ok(Incoming::Data(bytes)) => self.pending = bytes,
                Ok(Incoming::End) => {
                    self.ended = true;
                    return Ok(0);
                }
                Ok(Incoming::Failed(err)) => {
                    self.ended = true;
                    return Err(err);
                }
                Err(RecvTimeoutError::Timeout) => return Err(io::ErrorKind::TimedOut.into()),
                Err(RecvTimeoutError::Disconnected) => {
                    self.ended = true;
                    return Err(io::Error::new(io::ErrorKind::NotConnected, STOPPED));
                }
            }
        }
        let n = self.pending.len().min(buf.len());
        let chunk = self.pending.split_to(n);
        if let Some(into) = buf.get_mut(..n) {
            into.copy_from_slice(&chunk);
        }
        Ok(n)
    }
}

impl io::Write for TcpStream {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        if buf.is_empty() {
            return Ok(0);
        }
        self.stream.send_blocking(buf).map_err(io::Error::from)
    }

    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

impl Drop for TcpStream {
    fn drop(&mut self) {
        self.reader.abort();
    }
}

impl fmt::Debug for TcpStream {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("TcpStream")
            .field("local", &self.local_addr())
            .field("peer", &self.peer_addr())
            .field("ended", &self.ended)
            .finish_non_exhaustive()
    }
}

/// A UDP socket on the tailnet with the host it sends to, blocking as `std::net::UdpSocket` is: a
/// read waits no longer than the read timeout ([`UdpSocket::set_read_timeout`]) and then fails
/// `TimedOut`.
pub struct UdpSocket {
    socket: Arc<tailscale::netstack::UdpSocket>,
    /// Where [`UdpSocket::send`] sends.
    remote: SocketAddr,
    incoming: Receiver<io::Result<(SocketAddr, bytes::Bytes)>>,
    /// The task that receives into `incoming`, ended with the socket.
    reader: tokio::task::JoinHandle<()>,
    read_timeout: Option<Duration>,
    /// Nothing more will arrive: the socket failed, or its node stopped.
    ended: bool,
}

impl UdpSocket {
    /// The socket, with its reader started on the node's runtime.
    fn new(socket: tailscale::netstack::UdpSocket, remote: SocketAddr, runtime: &Handle) -> Self {
        let socket = Arc::new(socket);
        let (sender, incoming) = mpsc::channel();
        let receiving = Arc::clone(&socket);
        let reader = runtime.spawn(async move {
            loop {
                let next = receiving.recv_from_bytes().await.map_err(io::Error::from);
                let failed = next.is_err();
                if sender.send(next).is_err() || failed {
                    break;
                }
            }
        });
        Self {
            socket,
            remote,
            incoming,
            reader,
            read_timeout: None,
            ended: false,
        }
    }

    /// The local end: this node's tailnet address and an ephemeral port.
    #[must_use]
    pub fn local_addr(&self) -> SocketAddr {
        self.socket.local_addr()
    }

    /// Where [`UdpSocket::send`] sends: the host's tailnet address and port.
    #[must_use]
    pub const fn remote(&self) -> SocketAddr {
        self.remote
    }

    /// How long a read waits for a datagram before it fails `TimedOut`; `None`, for ever.
    pub fn set_read_timeout(&mut self, timeout: Option<Duration>) {
        self.read_timeout = timeout;
    }

    /// One datagram to the remote end.
    ///
    /// # Errors
    /// The network stack's, when it will not take the datagram.
    pub fn send(&self, data: &[u8]) -> io::Result<()> {
        self.socket
            .send_to_blocking(self.remote, data)
            .map_err(io::Error::from)
    }

    /// The next datagram, from anyone, into `into` (cleared first), and who sent it.
    ///
    /// # Errors
    /// `TimedOut` when none came within the read timeout; the network stack's when the socket
    /// failed, and `NotConnected` once its node has stopped - after either, [`UdpSocket::is_ended`].
    pub fn recv_into(&mut self, into: &mut Vec<u8>) -> io::Result<SocketAddr> {
        if self.ended {
            return Err(io::Error::new(io::ErrorKind::NotConnected, STOPPED));
        }
        let next = match self.read_timeout {
            Some(timeout) => self.incoming.os_recv_timeout(timeout),
            None => self
                .incoming
                .recv()
                .map_err(|_| RecvTimeoutError::Disconnected),
        };
        match next {
            Ok(Ok((from, datagram))) => {
                into.clear();
                into.extend_from_slice(&datagram);
                Ok(from)
            }
            Ok(Err(err)) => {
                self.ended = true;
                Err(err)
            }
            Err(RecvTimeoutError::Timeout) => Err(io::ErrorKind::TimedOut.into()),
            Err(RecvTimeoutError::Disconnected) => {
                self.ended = true;
                Err(io::Error::new(io::ErrorKind::NotConnected, STOPPED))
            }
        }
    }

    /// Whether nothing more can arrive: the socket failed, or its node stopped.
    #[must_use]
    pub const fn is_ended(&self) -> bool {
        self.ended
    }
}

impl Drop for UdpSocket {
    fn drop(&mut self) {
        self.reader.abort();
    }
}

impl fmt::Debug for UdpSocket {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("UdpSocket")
            .field("local", &self.local_addr())
            .field("remote", &self.remote)
            .field("ended", &self.ended)
            .finish_non_exhaustive()
    }
}

#[cfg(test)]
mod tests {
    use std::path::PathBuf;

    use super::*;

    /// A node given no folder for its keys fails as it starts, saying why, without a thread or a
    /// network; a call on it gives up at once, with how it stands; an address still resolves to
    /// itself; and a stop leaves it stopped.
    #[test]
    fn a_node_with_nowhere_for_its_keys_fails_at_once() {
        let node = Node::start(Settings::new(PathBuf::new()));
        assert_eq!(node.state(), State::Failed { why: NO_KEY_FOLDER.to_owned() });
        assert!(!node.is_running());
        let started = Instant::now();
        let failed = node.connect_tcp("white", 5760, Duration::from_secs(30));
        assert!(
            matches!(&failed, Err(Error::NotRunning(why)) if why.contains(NO_KEY_FOLDER)),
            "{failed:?}"
        );
        assert!(started.elapsed() < Duration::from_secs(5), "it did not wait");
        assert_eq!(
            node.resolve("fd7a:115c:a1e0::1", Duration::from_secs(1)),
            Ok("fd7a:115c:a1e0::1".parse::<IpAddr>().expect("an address"))
        );
        node.stop();
        assert_eq!(node.state(), State::Stopped);
    }

    /// The bridge from a planner thread to the runtime: an answer comes back; work that outlasts
    /// its deadline gives up then, a timeout, rather than holding the caller; and a call made on a
    /// runtime that has gone says the node stopped, at once, rather than waiting or panicking.
    #[test]
    fn a_call_answers_gives_up_at_its_deadline_and_outlives_its_runtime() {
        let runtime = tokio::runtime::Builder::new_multi_thread()
            .worker_threads(1)
            .enable_all()
            .build()
            .expect("a runtime");
        let handle = runtime.handle().clone();
        let soon = || Instant::now() + Duration::from_secs(5);
        assert_eq!(call(&handle, soon(), async { Ok(7) }), Ok(7));
        assert_eq!(
            call(&handle, soon(), async { Err::<(), _>(Error::Unresolved("white".to_owned())) }),
            Err(Error::Unresolved("white".to_owned()))
        );
        let started = Instant::now();
        let slow = call(&handle, Instant::now() + Duration::from_millis(200), async {
            tokio::time::sleep(Duration::from_secs(30)).await;
            Ok(())
        });
        assert_eq!(slow, Err(Error::Timeout));
        assert!(started.elapsed() < Duration::from_secs(5), "it gave up at its deadline");
        runtime.shutdown_timeout(Duration::from_secs(1));
        let started = Instant::now();
        assert_eq!(
            call(&handle, soon(), async { Ok(1) }),
            Err(Error::NotRunning(State::Stopped.to_string()))
        );
        assert!(started.elapsed() < Duration::from_secs(5), "it did not wait for a runtime gone");
    }
}
