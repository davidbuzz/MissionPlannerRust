//! The built-in HTTP server, `Utilities/httpserver.cs`: `MainV2` starts it with the window on
//! port 56781 ("motion jpg stream-network kml"), and it answers, one thread a client, as the C#
//! answers - the request's first line matched by substring, in the C#'s order:
//!
//! * `/websocket/server`: the handshake, then `MAV.cs` and `MAV.wps` as JSON text frames five
//!   times a second until the client's close frame;
//! * `/websocket/raw` (or `/` with `Upgrade: websocket`): the handshake, then every MAVLink frame
//!   the link reads as a binary frame, and every frame the client sends written to the link;
//! * `/georefnetwork.kml`: what the Geo Reference form last handed the server;
//! * `/location.kml`: a model placemark a vehicle (`block_plane_0.dae`), the view on the first;
//! * `/network.kml`: the two network links Google Earth follows;
//! * `/wps.kml`: the planner's points, a placemark each and the track in the air and on the ground;
//! * `/block_plane_0.dae` and `/hud.html`: files beside the program;
//! * `/hud.jpg`, `/map.jpg`, `/both.jpg`: a multipart JPEG stream at five a second;
//! * `/guided?lat=&lng=&alt=` and `POST /guide` (`{"lat":..,"lon":..,"alt":..}`): `setGuidedModeWP`,
//!   from a loopback client that no other site's page sent;
//! * `/command_long`, `/rcoverride`, `/get_mission`: 404;
//! * `/mavlink/`: the last ATTITUDE, VFR_HUD, NAV_CONTROLLER_OUTPUT, GPS_RAW_INT, HEARTBEAT,
//!   GPS_STATUS, STATUSTEXT and SYS_STATUS as JSON, with `META_LINKQUALITY` - Mavelous's feed;
//! * `/mav/`: Mavelous's files from `mavelous_web` beside the program, with `Last-Modified`;
//! * `.jpg`: a georeferenced photo from the form's folder, fitted to 640 by 480;
//! * `/`: a page of links; anything else 404.
//!
//! Divergences, each at its site: `map.jpg` and `both.jpg` answer 404 - the map is drawn by the
//! GPU and this application has no way to read it back, where the C# draws its control to a
//! bitmap; `hud.jpg` is the HUD's software rasteriser's frame, the camera's picture not in it;
//! a raw frame forwarded to a websocket is the message encoded again (the link hands its
//! subscribers the decoded message, not the bytes), and a frame received from one goes to the
//! link as a message, which addresses it as it addresses its own; `MAV.cs` is the numeric
//! properties this application holds (`quick::current_state_json`); the files are looked for
//! beside the program (`help::install_dir`), and a file that is not there is a 404 where the C#'s
//! exception closes the connection; `MP_HTTP_PORT` is the harness's door - another port, or
//! `off`. The C#'s `SYS_STATUS` key carries the `META_LINKQUALITY` object - it assigns both from
//! one expression - and so does this.
//! `// C#: Utilities/httpserver.cs; MainV2.cs:3224-3236, 2108`

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::collections::{BTreeMap, HashMap};
use std::io::{Read, Write};
use std::net::{IpAddr, TcpListener, TcpStream};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex, mpsc};
use std::time::{Duration, Instant};

use base64::Engine as _;
use mp_mavlink_dialects::all::{DIALECT, MavMessage};
use mp_mission::MissionItem;

use crate::MissionPlanner;
use crate::telemetry::TelemetryView;

/// `new TcpListener(IPAddress.Any, 56781)`.
pub const PORT: u16 = 56781;
/// The harness's door: another port, or `off`.
pub const PORT_ENV: &str = "MP_HTTP_PORT";
/// `MaxConcurrentConnections`.
const MAX_CONCURRENT: usize = 200;
/// `MaxConnectionsPerIp`.
const MAX_PER_IP: usize = 20;
/// `stream.ReadTimeout = 5000`.
const READ_TIMEOUT: Duration = Duration::from_secs(5);
/// The streams' and the websocket's `Thread.Sleep(200)`.
const PERIOD: Duration = Duration::from_millis(200);
/// `ComputeWebSocketHandshakeSecurityHash09`'s magic.
const WS_MAGIC: &str = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
/// The listener's failure, `log.Error`'s words.
const LISTEN_FAILED: &str = "Exception starting listener. Possible multiple instances of planner?";
/// What `map.jpg` and `both.jpg` say here.
const NO_MAP_IMAGE: &str =
    "map.jpg is not ported: the map is drawn by the GPU and cannot be read back here";

/// `/network.kml`, as the C# has it written out.
pub const NETWORK_KML: &str = r#"<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2" xmlns:gx="http://www.google.com/kml/ext/2.2" xmlns:kml="http://www.opengis.net/kml/2.2" xmlns:atom="http://www.w3.org/2005/Atom">
    <Folder>
        <name> Network Links </name>
        <open> 1 </open>
        <NetworkLink>
            <name> View Centered Placemark</name>
            <open> 1 </open>
            <refreshVisibility> 0 </refreshVisibility>
            <flyToView> 1 </flyToView>
            <Link>
                <href> http://127.0.0.1:56781/location.kml</href>
                <refreshMode> onInterval </refreshMode>
                <refreshInterval> 1 </refreshInterval>
                <viewRefreshTime> 1 </viewRefreshTime>
            </Link>
        </NetworkLink>
        <NetworkLink>
            <name> View Centered Placemark</name>
            <open> 1 </open>
            <refreshVisibility> 0 </refreshVisibility>
            <flyToView> 0 </flyToView>
            <Link>
                <href> http://127.0.0.1:56781/wps.kml</href>
            </Link>
        </NetworkLink>
    </Folder>
</kml>"#;

/// `m3u/GeoRefnetworklink.kml`, the file the Geo Reference form's Location Kml opens: a network
/// link to `/georefnetwork.kml`, refreshed every twenty seconds. Mission Planner ships it beside
/// the program; written under the data directory here when it is not beside this one.
/// `// C#: m3u/GeoRefnetworklink.kml; GeoRef/georefimage.cs:321-325`
pub const GEOREF_NETWORK_LINK_KML: &str = r#"<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2" xmlns:gx="http://www.google.com/kml/ext/2.2" xmlns:kml="http://www.opengis.net/kml/2.2" xmlns:atom="http://www.w3.org/2005/Atom">
<Folder>
	<name>Network Links</name>
	<open>1</open>
	<NetworkLink>
		<name>View Centered Placemark</name>
		<open>1</open>
		<refreshVisibility>0</refreshVisibility>
		<flyToView>0</flyToView>
		<Link>
			<href>http://127.0.0.1:56781/georefnetwork.kml</href>
			<refreshMode>onInterval</refreshMode>
			<refreshInterval>20</refreshInterval>
			<viewRefreshTime>20</viewRefreshTime>
		</Link>
	</NetworkLink>
</Folder>
</kml>
"#;

/// `/`'s page.
const INDEX_HTML: &str = "
                <a href=/mav/>Mavelous</a>
<a href=/mavlink/>Mavelous traffic</a>
<a href=/hud.jpg>Hud image</a>
<a href=/map.jpg>Map image </a>
<a href=/both.jpg>Map & hud image</a>
<a href=/hud.html>hud html5</a>
<a href=/network.kml>network kml</a>
<a href=/georefnetwork.kml>georef kml</a>
";

/// One vehicle for `location.kml`: `MAV.cs`'s position and attitude.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Located {
    /// `cs.lat`.
    pub lat: f64,
    /// `cs.lng`.
    pub lng: f64,
    /// `cs.altasl`.
    pub altasl: f64,
    /// `cs.yaw`.
    pub yaw: f64,
    /// `cs.roll`.
    pub roll: f64,
    /// `cs.pitch`.
    pub pitch: f64,
}

/// One of `FlightPlanner.instance.pointlist` for `wps.kml`.
#[derive(Debug, Clone, PartialEq)]
pub struct PlanPoint {
    /// `Tag`: "Home" for home, the row number for an item.
    pub tag: String,
    /// `Lat`.
    pub lat: f64,
    /// `Lng`.
    pub lng: f64,
    /// `Alt`.
    pub alt: f64,
}

/// What the window hands the server once a frame.
#[derive(Debug, Clone, Default)]
pub struct Snapshot {
    /// `JsonConvert.SerializeObject(MAV.cs)`.
    pub cs_json: String,
    /// `JsonConvert.SerializeObject(MAV.wps)`.
    pub wps_json: String,
    /// Every vehicle on every link, `MainV2.Comports`' `MAVlist`s.
    pub vehicles: Vec<Located>,
    /// `MainV2.comPort.MAV.cs`, the `LookAt`.
    pub primary: Option<Located>,
    /// The planner's points.
    pub plan_points: Vec<PlanPoint>,
    /// `/mavlink/`'s JSON.
    pub mavlink_json: String,
    /// The HUD's latest frame, while a stream wants one.
    pub hud_jpeg: Option<Arc<Vec<u8>>>,
}

/// What a client asked the window to do.
#[derive(Debug, Clone, PartialEq)]
pub enum Command {
    /// `setGuidedModeWP(gwp)`.
    Guided {
        /// `lat`.
        lat: f64,
        /// `lng`.
        lng: f64,
        /// `alt`.
        alt: f32,
    },
    /// A frame from a raw websocket, for the link; boxed, a message being far larger than the
    /// guided point.
    Send(Box<MavMessage>),
}

/// Between the server's threads and the window.
#[derive(Default)]
pub struct Shared {
    snapshot: Mutex<Snapshot>,
    commands: Mutex<Vec<Command>>,
    /// `myhud.streamjpgenable`: a stream client is on.
    hud_wanted: AtomicBool,
    /// `run`.
    run: AtomicBool,
    /// Requests answered.
    requests: AtomicU64,
    /// The last request's first line.
    last_url: Mutex<String>,
    /// The port listened on, once the listener is up.
    listening: Mutex<Option<u16>>,
    /// Why it is not, if it is not.
    failure: Mutex<Option<String>>,
    /// `georefkml` and `georefimagepath`.
    georef: Mutex<(String, String)>,
    /// The raw websockets' queues.
    raw_clients: Mutex<Vec<mpsc::Sender<Vec<u8>>>>,
    /// How many, so the link thread takes no lock when there are none.
    raw_count: AtomicUsize,
    /// `activeConnections`.
    active: AtomicUsize,
    /// `perIpConnections`.
    per_ip: Mutex<HashMap<IpAddr, usize>>,
}

/// A lock taken whatever the other side did.
fn lock<T>(mutex: &Mutex<T>) -> std::sync::MutexGuard<'_, T> {
    mutex
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner)
}

impl Shared {
    /// Whether any raw websocket is on.
    pub fn has_raw_clients(&self) -> bool {
        self.raw_count.load(Ordering::Acquire) > 0
    }

    /// A frame the link read, to every raw websocket; a client that has gone is forgotten.
    pub fn forward_raw(&self, frame: &[u8]) {
        let mut clients = lock(&self.raw_clients);
        clients.retain(|client| client.send(frame.to_vec()).is_ok());
        self.raw_count.store(clients.len(), Ordering::Release);
    }

    fn add_raw_client(&self) -> mpsc::Receiver<Vec<u8>> {
        let (sender, receiver) = mpsc::channel();
        let mut clients = lock(&self.raw_clients);
        clients.push(sender);
        self.raw_count.store(clients.len(), Ordering::Release);
        receiver
    }

    fn running(&self) -> bool {
        self.run.load(Ordering::Acquire)
    }
}

/// The server: its listener thread and what it shares with the window.
pub struct Server {
    shared: Arc<Shared>,
}

impl Server {
    /// `new httpserver().listernforclients` on its thread: the listener on the port, or
    /// [`PORT_ENV`]'s; `off` leaves the server down.
    #[must_use]
    pub fn start(files: PathBuf) -> Self {
        let shared = Arc::new(Shared::default());
        let port = match std::env::var(PORT_ENV) {
            Ok(door) if door.eq_ignore_ascii_case("off") => None,
            Ok(door) => Some(door.trim().parse().unwrap_or(PORT)),
            Err(_) => Some(PORT),
        };
        if let Some(port) = port {
            shared.run.store(true, Ordering::Release);
            let worker = Arc::clone(&shared);
            let dir = files.clone();
            let _ = std::thread::Builder::new()
                .name("motion jpg stream-network kml".to_owned())
                .spawn(move || listen(&worker, port, &dir));
        }
        Self { shared }
    }

    /// Whether the listener is meant to be up.
    #[must_use]
    pub fn running(&self) -> bool {
        self.shared.running()
    }

    /// What the threads share.
    #[must_use]
    pub fn shared(&self) -> &Arc<Shared> {
        &self.shared
    }

    /// The window's snapshot for the clients.
    pub fn publish(&self, snapshot: Snapshot) {
        *lock(&self.shared.snapshot) = snapshot;
    }

    /// `myhud.streamjpgenable`.
    #[must_use]
    pub fn hud_wanted(&self) -> bool {
        self.shared.hud_wanted.load(Ordering::Acquire)
    }

    /// What the clients asked for since the last frame.
    #[must_use]
    pub fn take_commands(&self) -> Vec<Command> {
        std::mem::take(&mut *lock(&self.shared.commands))
    }

    /// `httpserver.georefkml` and `georefimagepath`, the Geo Reference form's.
    pub fn set_georef(&self, kml: String, image_dir: String) {
        *lock(&self.shared.georef) = (kml, image_dir);
    }

    /// `httpserver.Stop()`.
    pub fn stop(&self) {
        self.shared.run.store(false, Ordering::Release);
    }

    /// What a script can see: the port, the requests, the last one and the clients.
    pub fn record_facts(&self) {
        crate::facts::record(
            "http.listening",
            lock(&self.shared.listening).map_or_else(|| "none".to_owned(), |port| port.to_string()),
        );
        crate::facts::record(
            "http.requests",
            self.shared.requests.load(Ordering::Acquire),
        );
        let last = lock(&self.shared.last_url).clone();
        crate::facts::record(
            "http.last",
            if last.is_empty() {
                "none".to_owned()
            } else {
                last
            },
        );
        crate::facts::record("http.clients", self.shared.active.load(Ordering::Acquire));
        crate::facts::record(
            "http.failure",
            lock(&self.shared.failure)
                .clone()
                .unwrap_or_else(|| "none".to_owned()),
        );
    }
}

/// `listernforclients`: the listener, then a thread a client, within the caps.
fn listen(shared: &Arc<Shared>, port: u16, files: &Path) {
    let listener = match TcpListener::bind(("0.0.0.0", port)) {
        Ok(listener) => listener,
        Err(err) => {
            *lock(&shared.failure) = Some(format!("{LISTEN_FAILED} ({err})"));
            return;
        }
    };
    if listener.set_nonblocking(true).is_err() {
        *lock(&shared.failure) = Some(LISTEN_FAILED.to_owned());
        return;
    }
    *lock(&shared.listening) = Some(port);
    while shared.running() {
        match listener.accept() {
            Ok((stream, peer)) => {
                let ip = peer.ip();
                if !take_slot(shared, ip) {
                    continue;
                }
                let _ = stream.set_nonblocking(false);
                let _ = stream.set_read_timeout(Some(READ_TIMEOUT));
                let worker = Arc::clone(shared);
                let dir = files.to_path_buf();
                let spawned = std::thread::Builder::new()
                    .name("http client".to_owned())
                    .spawn(move || {
                        let mut stream = stream;
                        serve(&mut stream, &worker, &dir);
                        release_slot(&worker, ip);
                    });
                if spawned.is_err() {
                    release_slot(shared, ip);
                }
            }
            Err(err) if err.kind() == std::io::ErrorKind::WouldBlock => {
                std::thread::sleep(Duration::from_millis(50));
            }
            Err(_) => std::thread::sleep(Duration::from_millis(50)),
        }
    }
    *lock(&shared.listening) = None;
}

/// `DoAcceptTcpClientCallback`'s caps: a connection is admitted within [`MAX_CONCURRENT`] and
/// [`MAX_PER_IP`], else dropped.
fn take_slot(shared: &Shared, ip: IpAddr) -> bool {
    if shared.active.load(Ordering::Acquire) >= MAX_CONCURRENT {
        return false;
    }
    let mut per_ip = lock(&shared.per_ip);
    let count = per_ip.entry(ip).or_insert(0);
    if *count >= MAX_PER_IP {
        return false;
    }
    *count += 1;
    shared.active.fetch_add(1, Ordering::AcqRel);
    true
}

/// `ReleaseConnectionSlot`.
fn release_slot(shared: &Shared, ip: IpAddr) {
    shared.active.fetch_sub(1, Ordering::AcqRel);
    let mut per_ip = lock(&shared.per_ip);
    if let Some(count) = per_ip.get_mut(&ip) {
        *count = count.saturating_sub(1);
        if *count == 0 {
            per_ip.remove(&ip);
        }
    }
}

/// A client's connection, as `ProcessClient` uses it: bytes both ways, `client.Available`, and
/// who is at the far end.
pub trait Connection: Read + Write {
    /// `client.Available > 0`.
    fn has_input(&mut self) -> bool;
    /// `client.Client.RemoteEndPoint`'s address.
    fn peer(&self) -> Option<IpAddr>;
    /// `client.Connected`.
    fn connected(&self) -> bool {
        true
    }
}

impl Connection for TcpStream {
    fn has_input(&mut self) -> bool {
        let mut byte = [0u8; 1];
        let _ = self.set_nonblocking(true);
        let available = matches!(self.peek(&mut byte), Ok(1..));
        let _ = self.set_nonblocking(false);
        available
    }

    fn peer(&self) -> Option<IpAddr> {
        self.peer_addr().ok().map(|address| address.ip())
    }

    fn connected(&self) -> bool {
        // A peer that has closed answers a peek with 0 bytes.
        let mut byte = [0u8; 1];
        let _ = self.set_nonblocking(true);
        let closed = matches!(self.peek(&mut byte), Ok(0));
        let _ = self.set_nonblocking(false);
        !closed
    }
}

/// `ProcessClient`: each request on the connection answered in turn - the C#'s `goto again` for
/// the routes that keep the connection - until one closes it or the client goes.
pub fn serve<C: Connection>(stream: &mut C, shared: &Shared, files: &Path) {
    // `myhud.streamjpgenable = true`: a frame is kept ready for whatever this client asks.
    shared.hud_wanted.store(true, Ordering::Release);
    while shared.running() {
        let Some((head, url)) = read_request(stream) else {
            break;
        };
        shared.requests.fetch_add(1, Ordering::AcqRel);
        *lock(&shared.last_url) = url.clone();
        if !route(stream, &head, &url, shared, files) {
            break;
        }
    }
    // `streamjpgenable = false` once the stream's client has gone.
    if shared.active.load(Ordering::Acquire) <= 1 {
        shared.hud_wanted.store(false, Ordering::Release);
    }
}

/// The head of one request, `request = new byte[1024 * 4]` read once, and its first line less
/// the `\r`: `None` when nothing came or the head has no line end.
fn read_request<C: Read>(stream: &mut C) -> Option<(String, String)> {
    let mut request = [0u8; 4096];
    let len = stream.read(&mut request).ok()?;
    if len == 0 {
        return None;
    }
    let head: String = request
        .get(..len)?
        .iter()
        .map(|byte| char::from(*byte))
        .collect();
    let index = head.find('\n')?;
    let url = head.get(..index.saturating_sub(1))?.to_owned();
    Some((head, url))
}

/// One request answered. True when the connection is kept for the next (`goto again`).
fn route<C: Connection>(
    stream: &mut C,
    head: &str,
    url: &str,
    shared: &Shared,
    files: &Path,
) -> bool {
    let lower = url.to_ascii_lowercase();
    if url.contains(" /websocket/server") {
        websocket_server(stream, head, shared);
        false
    } else if url.contains(" /websocket/raw")
        || url.contains(" / ") && head.contains("Upgrade: websocket")
    {
        websocket_raw(stream, head, shared);
        false
    } else if url.contains(" /georefnetwork.kml") {
        let kml = lock(&shared.georef).0.clone();
        let header = format!(
            "HTTP/1.1 200 OK\r\nServer: here\r\nKeep-Alive: timeout=15, max=100\r\nConnection: Keep-Alive\r\nCache-Control: no-cache\r\nContent-Type: application/vnd.google-earth.kml+xml\r\nX-Pad: avoid browser bug\r\nContent-Length: {}\r\n\r\n",
            kml.len()
        );
        write_all(stream, &[header.as_bytes(), kml.as_bytes()])
    } else if url.contains(" /location.kml") {
        let snapshot = lock(&shared.snapshot);
        let kml = location_kml(&snapshot.vehicles, snapshot.primary.as_ref());
        drop(snapshot);
        write_all(stream, &[kml_header(kml.len()).as_bytes(), kml.as_bytes()])
    } else if url.contains(" /network.kml") {
        write_all(
            stream,
            &[
                kml_header(NETWORK_KML.len()).as_bytes(),
                NETWORK_KML.as_bytes(),
            ],
        )
    } else if url.contains(" /wps.kml") {
        let kml = wps_kml(&lock(&shared.snapshot).plan_points);
        write_all(stream, &[kml_header(kml.len()).as_bytes(), kml.as_bytes()])
    } else if url.contains(" /block_plane_0.dae") {
        match std::fs::read(files.join("block_plane_0.dae")) {
            Ok(bytes) => {
                let _ = write_all(
                    stream,
                    &[
                        b"HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\n\r\n",
                        &bytes,
                    ],
                );
            }
            Err(_) => not_found(stream, "text/plain"),
        }
        false
    } else if url.contains(" /hud.html") {
        match std::fs::read(files.join("hud.html")) {
            Ok(bytes) => {
                let header = format!(
                    "HTTP/1.1 200 OK\r\nConnection: close\r\nContent-Type: text/html\r\nContent-Length: {}\r\n\r\n",
                    bytes.len()
                );
                let _ = write_all(stream, &[header.as_bytes(), &bytes]);
            }
            Err(_) => not_found(stream, "text/html"),
        }
        false
    } else if lower.contains(" /hud.jpg")
        || lower.contains(" /map.jpg")
        || lower.contains(" /both.jpg")
    {
        if lower.contains("hud.jpg") {
            jpeg_stream(stream, shared);
        } else {
            let header = format!(
                "HTTP/1.1 404 not found\r\nContent-Type: text/plain\r\nContent-Length: {}\r\n\r\n",
                NO_MAP_IMAGE.len()
            );
            let _ = write_all(stream, &[header.as_bytes(), NO_MAP_IMAGE.as_bytes()]);
        }
        false
    } else if url.contains(" /guided?") {
        guided(stream, head, shared, guided_query(url));
        false
    } else if lower.contains("post /guide") {
        guided(stream, head, shared, guided_body(head));
        false
    } else if lower.contains(" /command_long")
        || lower.contains(" /rcoverride")
        || lower.contains(" /get_mission")
    {
        not_found(stream, "image/jpg");
        false
    } else if lower.contains(" /mavlink/") {
        let json = lock(&shared.snapshot).mavlink_json.clone();
        let header = format!(
            "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: {}\r\n\r\n",
            json.len()
        );
        write_all(stream, &[header.as_bytes(), json.as_bytes()])
    } else if lower.contains(" /mav/") {
        static_file(stream, head, url, &files.join("mavelous_web"))
    } else if lower.contains(".jpg") {
        let dir = lock(&shared.georef).1.clone();
        photo(stream, url, &dir)
    } else if lower.contains(" / ") {
        let _ = write_all(
            stream,
            &[
                b"HTTP/1.1 200 OK\r\nConnection: close\r\nContent-Type: text/html\r\n\r\n",
                INDEX_HTML.as_bytes(),
            ],
        );
        false
    } else {
        not_found(stream, "text/plain");
        false
    }
}

/// Every piece written and the stream flushed; false when the client has gone.
fn write_all<C: Write>(stream: &mut C, pieces: &[&[u8]]) -> bool {
    for piece in pieces {
        if stream.write_all(piece).is_err() {
            return false;
        }
    }
    stream.flush().is_ok()
}

/// "HTTP/1.1 404 not found" with the content type the C#'s branch names.
fn not_found<C: Write>(stream: &mut C, content_type: &str) {
    let header = format!("HTTP/1.1 404 not found\r\nContent-Type: {content_type}\r\n\r\n");
    let _ = write_all(stream, &[header.as_bytes()]);
}

/// The KML routes' header.
fn kml_header(length: usize) -> String {
    format!(
        "HTTP/1.1 200 OK\r\nContent-Type: application/vnd.google-earth.kml+xml\r\nContent-Length: {length}\r\n\r\n"
    )
}

// ---------------------------------------------------------------------------------------------
// WebSockets.
// ---------------------------------------------------------------------------------------------

/// `ComputeWebSocketHandshakeSecurityHash09`: SHA-1 of the key and the magic, base64.
#[must_use]
pub fn websocket_accept(key: &str) -> String {
    let digest = sha1_smol::Sha1::from(format!("{key}{WS_MAGIC}")).digest();
    base64::engine::general_purpose::STANDARD.encode(digest.bytes())
}

/// The key as the C# cuts it from the head: nineteen characters past `Sec-WebSocket-Key:`, to
/// the line's end.
fn websocket_key(head: &str) -> Option<&str> {
    let start = head.find("Sec-WebSocket-Key:")? + 19;
    let rest = head.get(start..)?;
    let end = rest.find('\r').or_else(|| rest.find('\n'))?;
    rest.get(..end)
}

/// One frame to the client, length as the C# writes it: one byte to 125, else `126` and two.
#[must_use]
pub fn websocket_frame(opcode: u8, payload: &[u8]) -> Vec<u8> {
    let mut packet = Vec::with_capacity(payload.len() + 4);
    packet.push(opcode);
    if payload.len() <= 125 {
        #[allow(clippy::cast_possible_truncation)] // at most 125
        packet.push(payload.len() as u8);
    } else {
        packet.push(126);
        #[allow(clippy::cast_possible_truncation)] // the C#'s two bytes
        packet.push((payload.len() >> 8) as u8);
        #[allow(clippy::cast_possible_truncation)]
        packet.push((payload.len() & 0xff) as u8);
    }
    packet.extend_from_slice(payload);
    packet
}

/// One frame from the client: its opcode and payload unmasked, `None` at the stream's end.
fn websocket_read<C: Read>(stream: &mut C) -> Option<(u8, Vec<u8>)> {
    let mut byte = [0u8; 1];
    stream.read_exact(&mut byte).ok()?;
    let opcode = byte[0];
    stream.read_exact(&mut byte).ok()?;
    let lenw = byte[0];
    let mut length = usize::from(lenw & 0x7f);
    if length == 126 {
        let mut two = [0u8; 2];
        stream.read_exact(&mut two).ok()?;
        length = usize::from(u16::from_be_bytes(two));
    } else if length == 127 {
        let mut eight = [0u8; 8];
        stream.read_exact(&mut eight).ok()?;
        length = usize::try_from(u64::from_be_bytes(eight)).ok()?;
    }
    let mut mask = [0u8; 4];
    if lenw & 0x80 != 0 {
        stream.read_exact(&mut mask).ok()?;
    }
    let mut payload = vec![0u8; length.min(1024 * 1024)];
    stream.read_exact(&mut payload).ok()?;
    for (index, value) in payload.iter_mut().enumerate() {
        *value ^= mask.get(index % 4).copied().unwrap_or(0);
    }
    Some((opcode, payload))
}

/// `/websocket/server`: the handshake, then `cs` and `wps` as text frames every 200 ms, until a
/// close frame's first byte (`0x88`) comes.
fn websocket_server<C: Connection>(stream: &mut C, head: &str, shared: &Shared) {
    let accept = websocket_key(head)
        .map(websocket_accept)
        .unwrap_or_default();
    let handshake = format!(
        "HTTP/1.1 101 WebSocket Protocol Handshake\r\nUpgrade: WebSocket\r\nConnection: Upgrade\r\nWebSocket-Location: ws://localhost:56781/websocket/server\r\nSec-WebSocket-Accept: {accept}\r\nServer: Mission Planner\r\n\r\n"
    );
    if !write_all(stream, &[handshake.as_bytes()]) {
        return;
    }
    while shared.running() && stream.connected() {
        while stream.has_input() {
            let mut byte = [0u8; 1];
            if stream.read_exact(&mut byte).is_err() || byte[0] == 0x88 {
                return;
            }
        }
        let (cs, wps) = {
            let snapshot = lock(&shared.snapshot);
            (snapshot.cs_json.clone(), snapshot.wps_json.clone())
        };
        for text in [cs, wps] {
            if !write_all(stream, &[&websocket_frame(0x81, text.as_bytes())]) {
                return;
            }
        }
        std::thread::sleep(PERIOD);
    }
}

/// `/websocket/raw`: the handshake, then every frame the link reads as a binary frame, and
/// every frame the client sends to the link, until a close frame.
fn websocket_raw<C: Connection>(stream: &mut C, head: &str, shared: &Shared) {
    let accept = websocket_key(head)
        .map(websocket_accept)
        .unwrap_or_default();
    let protocol = if head.contains("Sec-WebSocket-Protocol:") {
        "Sec-WebSocket-Protocol: binary\r\n"
    } else {
        ""
    };
    let handshake = format!(
        "HTTP/1.1 101 WebSocket Protocol Handshake\r\nUpgrade: WebSocket\r\nConnection: upgrade\r\nDate: {}\r\nSec-WebSocket-Accept: {accept}\r\n{protocol}Server: Mission Planner\r\n\r\n",
        chrono::Utc::now().format("%a, %d %b %Y %H:%M:%S GMT")
    );
    if !write_all(stream, &[handshake.as_bytes()]) {
        return;
    }
    let frames = shared.add_raw_client();
    while shared.running() && stream.connected() {
        while let Ok(frame) = frames.try_recv() {
            if !write_all(stream, &[&websocket_frame(0x82, &frame)]) {
                return;
            }
        }
        while stream.has_input() {
            let Some((opcode, payload)) = websocket_read(stream) else {
                return;
            };
            if opcode == 0x88 {
                return;
            }
            if let Ok((frame, _)) = mp_mavlink::parse(&payload, &DIALECT)
                && let Some(message) = MavMessage::decode(frame.msgid, frame.payload)
            {
                lock(&shared.commands).push(Command::Send(Box::new(message)));
            }
        }
        std::thread::sleep(PERIOD);
    }
}

/// A frame the link read, as the bytes a raw websocket is given: the message encoded again
/// under its sender's ids.
#[must_use]
pub fn reencode(sysid: u8, compid: u8, message: &MavMessage) -> Option<Vec<u8>> {
    let mut payload = [0u8; 255];
    let len = message.encode(&mut payload);
    let mut out = [0u8; 300];
    let n = mp_mavlink::encode_v2(
        &mut out,
        0,
        sysid,
        compid,
        message.id(),
        payload.get(..len)?,
        message.crc_extra(),
        0,
    )
    .ok()?;
    out.get(..n).map(<[u8]>::to_vec)
}

// ---------------------------------------------------------------------------------------------
// KML and JSON.
// ---------------------------------------------------------------------------------------------

/// `double.ToString()`.
fn number(value: f64) -> String {
    if value.is_finite() {
        value.to_string()
    } else {
        "NaN".to_owned()
    }
}

/// `/location.kml`: a `Placemark` a vehicle - "P/Q <altasl>", a `Model` of `block_plane_0.dae`
/// at its position (an altitude below 0 is 0.01), heading its yaw, roll and tilt the negatives,
/// scale 2 - and the `LookAt` on the first vehicle when it has a position.
/// `// C#: Utilities/httpserver.cs:559-636`
#[must_use]
pub fn location_kml(vehicles: &[Located], primary: Option<&Located>) -> String {
    let mut out = String::from(
        "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n<kml xmlns=\"http://www.opengis.net/kml/2.2\"><Document>",
    );
    if let Some(cs) = primary
        && cs.lat != 0.0
        && cs.lng != 0.0
    {
        out.push_str(&format!(
            "<LookAt><longitude>{}</longitude><latitude>{}</latitude><altitude>{}</altitude><heading>{}</heading><tilt>80</tilt><range>50</range><altitudeMode>absolute</altitudeMode></LookAt>",
            number(cs.lng),
            number(cs.lat),
            number(cs.altasl),
            number(cs.yaw)
        ));
    }
    for cs in vehicles {
        let altitude = if cs.altasl < 0.0 { 0.01 } else { cs.altasl };
        out.push_str(&format!(
            "<Placemark><name>P/Q {}</name><visibility>1</visibility><Model><altitudeMode>absolute</altitudeMode><Location><longitude>{}</longitude><latitude>{}</latitude><altitude>{}</altitude></Location><Orientation><heading>{}</heading><tilt>{}</tilt><roll>{}</roll></Orientation><Scale><x>2</x><y>2</y><z>2</z></Scale><Link><href>block_plane_0.dae</href></Link></Model></Placemark>",
            number(cs.altasl),
            number(cs.lng),
            number(cs.lat),
            number(altitude),
            number(cs.yaw),
            number(-cs.pitch),
            number(-cs.roll)
        ));
    }
    out.push_str("</Document></kml>");
    out
}

/// `/wps.kml`: a `Placemark` a point, "WP <tag> Alt: <alt>", then the track twice - "WPs" in
/// the air in yellow, four wide, and "onground" clamped to the ground, half as strong - each
/// with its style inline, both styles under the one id the C# gives them.
/// `// C#: Utilities/httpserver.cs:682-786`
#[must_use]
pub fn wps_kml(points: &[PlanPoint]) -> String {
    let mut out = String::from(
        "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n<kml xmlns=\"http://www.opengis.net/kml/2.2\"><Document>",
    );
    for point in points {
        out.push_str(&format!(
            "<Placemark><name>WP {} Alt: {}</name><Point><altitudeMode>absolute</altitudeMode><coordinates>{},{},{}</coordinates></Point></Placemark>",
            point.tag,
            number(point.alt),
            number(point.lng),
            number(point.lat),
            number(point.alt)
        ));
    }
    let coordinates: Vec<String> = points
        .iter()
        .map(|point| {
            format!(
                "{},{},{}",
                number(point.lng),
                number(point.lat),
                number(point.alt)
            )
        })
        .collect();
    let coordinates = coordinates.join(" ");
    // `HexStringToColor("ff00ffff")`: a=ff, b=00, g=ff, r=ff - KML's aabbggrr is `ff00ffff`.
    out.push_str(&format!(
        "<Placemark><name>WPs</name><Style id=\"yellowLineGreenPoly\"><LineStyle><color>ff00ffff</color><width>4</width></LineStyle></Style><LineString><extrude>0</extrude><tessellate>1</tessellate><altitudeMode>absolute</altitudeMode><coordinates>{coordinates}</coordinates></LineString></Placemark>"
    ));
    out.push_str(&format!(
        "<Placemark><name>onground</name><Style id=\"yellowLineGreenPoly\"><LineStyle><color>7f00ffff</color><width>4</width></LineStyle></Style><LineString><extrude>0</extrude><tessellate>1</tessellate><altitudeMode>clampToGround</altitudeMode><coordinates>{coordinates}</coordinates></LineString></Placemark>"
    ));
    out.push_str("</Document></kml>");
    out
}

/// `JsonConvert.SerializeObject(MAV.wps)`: the items by sequence number, each a `Locationwp` -
/// `frame`, `Tag` (null), `id`, `p1` to `p4`, `lat`, `lng`, `alt`.
/// `// C#: ExtLibs/Utilities/locationwp.cs:200-208`
#[must_use]
pub fn wps_json(items: &[MissionItem]) -> String {
    let mut out = String::from("{");
    for (index, item) in items.iter().enumerate() {
        if index > 0 {
            out.push(',');
        }
        out.push_str(&format!(
            "\"{}\":{{\"frame\":{},\"Tag\":null,\"id\":{},\"p1\":{},\"p2\":{},\"p3\":{},\"p4\":{},\"lat\":{},\"lng\":{},\"alt\":{}}}",
            item.seq,
            item.frame,
            item.command,
            number(item.param1),
            number(item.param2),
            number(item.param3),
            number(item.param4),
            number(item.x),
            number(item.y),
            number(item.z)
        ));
    }
    out.push('}');
    out
}

/// The last of each message `/mavlink/` reports, as the link read them.
#[derive(Debug, Default)]
pub struct LastMessages {
    by_id: BTreeMap<u32, MavMessage>,
}

impl LastMessages {
    /// `MAV.getPacketLast` for the eight: anything else is not kept.
    pub fn remember(&mut self, msgid: u32, message: &MavMessage) {
        if matches!(
            message,
            MavMessage::Attitude(_)
                | MavMessage::VfrHud(_)
                | MavMessage::NavControllerOutput(_)
                | MavMessage::GpsRawInt(_)
                | MavMessage::Heartbeat(_)
                | MavMessage::GpsStatus(_)
                | MavMessage::Statustext(_)
                | MavMessage::SysStatus(_)
        ) {
            self.by_id.insert(msgid, *message);
        }
    }

    /// `/mavlink/`'s JSON: each message held as `{"msg": ..., "index": 1, "time_usec": 0}` under
    /// its name, then `META_LINKQUALITY` - and `SYS_STATUS` the same object, as the C# assigns
    /// both from one expression.
    /// `// C#: Utilities/httpserver.cs:1037-1133`
    #[must_use]
    pub fn json(&self, master_in: i64, master_out: i64, packet_loss: f64, index: i64) -> String {
        let mut parts: Vec<String> = Vec::new();
        for message in self.by_id.values() {
            if let Some((name, msg)) = message_json(message) {
                parts.push(format!(
                    "\"{name}\":{{\"msg\":{msg},\"index\":1,\"time_usec\":0}}"
                ));
            }
        }
        let meta = format!(
            "{{\"msg\":{{\"master_in\":{master_in},\"mav_loss\":0,\"mavpackettype\":\"META_LINKQUALITY\",\"master_out\":{master_out},\"packet_loss\":{}}},\"index\":{index},\"time_usec\":0}}",
            number(packet_loss)
        );
        parts.retain(|part| !part.starts_with("\"SYS_STATUS\""));
        parts.push(format!("\"SYS_STATUS\":{meta}"));
        parts.push(format!("\"META_LINKQUALITY\":{meta}"));
        format!("{{{}}}", parts.join(","))
    }
}

/// A JSON array of bytes.
fn bytes_json(bytes: &[u8]) -> String {
    let parts: Vec<String> = bytes.iter().map(ToString::to_string).collect();
    format!("[{}]", parts.join(","))
}

/// A JSON string of the message's text, to its first NUL.
fn text_json(text: &[u8]) -> String {
    let end = text
        .iter()
        .position(|byte| *byte == 0)
        .unwrap_or(text.len());
    let text: String = text
        .get(..end)
        .unwrap_or(&[])
        .iter()
        .map(|byte| char::from(*byte))
        .collect();
    serde_json::to_string(&text).unwrap_or_else(|_| "\"\"".to_owned())
}

/// One message as `JsonConvert.SerializeObject` writes its struct: the fields by name.
fn message_json(message: &MavMessage) -> Option<(&'static str, String)> {
    let n = |value: f32| number(f64::from(value));
    Some(match message {
        MavMessage::Attitude(m) => (
            "ATTITUDE",
            format!(
                "{{\"time_boot_ms\":{},\"roll\":{},\"pitch\":{},\"yaw\":{},\"rollspeed\":{},\"pitchspeed\":{},\"yawspeed\":{}}}",
                m.time_boot_ms,
                n(m.roll),
                n(m.pitch),
                n(m.yaw),
                n(m.rollspeed),
                n(m.pitchspeed),
                n(m.yawspeed)
            ),
        ),
        MavMessage::VfrHud(m) => (
            "VFR_HUD",
            format!(
                "{{\"airspeed\":{},\"groundspeed\":{},\"alt\":{},\"climb\":{},\"heading\":{},\"throttle\":{}}}",
                n(m.airspeed),
                n(m.groundspeed),
                n(m.alt),
                n(m.climb),
                m.heading,
                m.throttle
            ),
        ),
        MavMessage::NavControllerOutput(m) => (
            "NAV_CONTROLLER_OUTPUT",
            format!(
                "{{\"nav_roll\":{},\"nav_pitch\":{},\"alt_error\":{},\"aspd_error\":{},\"xtrack_error\":{},\"nav_bearing\":{},\"target_bearing\":{},\"wp_dist\":{}}}",
                n(m.nav_roll),
                n(m.nav_pitch),
                n(m.alt_error),
                n(m.aspd_error),
                n(m.xtrack_error),
                m.nav_bearing,
                m.target_bearing,
                m.wp_dist
            ),
        ),
        MavMessage::GpsRawInt(m) => (
            "GPS_RAW_INT",
            format!(
                "{{\"time_usec\":{},\"lat\":{},\"lon\":{},\"alt\":{},\"eph\":{},\"epv\":{},\"vel\":{},\"cog\":{},\"fix_type\":{},\"satellites_visible\":{},\"alt_ellipsoid\":{},\"h_acc\":{},\"v_acc\":{},\"vel_acc\":{},\"hdg_acc\":{},\"yaw\":{}}}",
                m.time_usec,
                m.lat,
                m.lon,
                m.alt,
                m.eph,
                m.epv,
                m.vel,
                m.cog,
                m.fix_type,
                m.satellites_visible,
                m.alt_ellipsoid,
                m.h_acc,
                m.v_acc,
                m.vel_acc,
                m.hdg_acc,
                m.yaw
            ),
        ),
        MavMessage::Heartbeat(m) => (
            "HEARTBEAT",
            format!(
                "{{\"custom_mode\":{},\"type\":{},\"autopilot\":{},\"base_mode\":{},\"system_status\":{},\"mavlink_version\":{}}}",
                m.custom_mode,
                m.r#type,
                m.autopilot,
                m.base_mode,
                m.system_status,
                m.mavlink_version
            ),
        ),
        MavMessage::GpsStatus(m) => (
            "GPS_STATUS",
            format!(
                "{{\"satellites_visible\":{},\"satellite_prn\":{},\"satellite_used\":{},\"satellite_elevation\":{},\"satellite_azimuth\":{},\"satellite_snr\":{}}}",
                m.satellites_visible,
                bytes_json(&m.satellite_prn),
                bytes_json(&m.satellite_used),
                bytes_json(&m.satellite_elevation),
                bytes_json(&m.satellite_azimuth),
                bytes_json(&m.satellite_snr)
            ),
        ),
        MavMessage::Statustext(m) => (
            "STATUSTEXT",
            format!(
                "{{\"severity\":{},\"text\":{},\"id\":{},\"chunk_seq\":{}}}",
                m.severity,
                text_json(&m.text),
                m.id,
                m.chunk_seq
            ),
        ),
        MavMessage::SysStatus(m) => (
            "SYS_STATUS",
            format!(
                "{{\"onboard_control_sensors_present\":{},\"onboard_control_sensors_enabled\":{},\"onboard_control_sensors_health\":{},\"load\":{},\"voltage_battery\":{},\"current_battery\":{},\"drop_rate_comm\":{},\"errors_comm\":{},\"errors_count1\":{},\"errors_count2\":{},\"errors_count3\":{},\"errors_count4\":{},\"battery_remaining\":{}}}",
                m.onboard_control_sensors_present,
                m.onboard_control_sensors_enabled,
                m.onboard_control_sensors_health,
                m.load,
                m.voltage_battery,
                m.current_battery,
                m.drop_rate_comm,
                m.errors_comm,
                m.errors_count1,
                m.errors_count2,
                m.errors_count3,
                m.errors_count4,
                m.battery_remaining
            ),
        ),
        _ => return None,
    })
}

// ---------------------------------------------------------------------------------------------
// Streams, commands, files.
// ---------------------------------------------------------------------------------------------

/// `/hud.jpg`: `multipart/x-mixed-replace`, a JPEG every 200 ms while the client stays.
fn jpeg_stream<C: Connection>(stream: &mut C, shared: &Shared) {
    shared.hud_wanted.store(true, Ordering::Release);
    if !write_all(
        stream,
        &[b"HTTP/1.1 200 OK\r\nContent-Type: multipart/x-mixed-replace;boundary=PLANNER\r\n\r\n--PLANNER\r\n"],
    ) {
        return;
    }
    while shared.running() && stream.connected() {
        std::thread::sleep(PERIOD);
        let Some(data) = lock(&shared.snapshot).hud_jpeg.clone() else {
            continue;
        };
        let header = format!(
            "Content-Type: image/jpeg\r\nContent-Length: {}\r\n\r\n",
            data.len()
        );
        if !write_all(stream, &[header.as_bytes(), &data, b"\r\n--PLANNER\r\n"]) {
            return;
        }
    }
}

/// `GetHeaderValue`: a header's value, without case, trimmed.
fn header_value(head: &str, name: &str) -> Option<String> {
    head.lines().find_map(|line| {
        let (key, value) = line.split_once(':')?;
        key.trim()
            .eq_ignore_ascii_case(name)
            .then(|| value.trim().to_owned())
    })
}

/// `IsLoopbackOrigin`: an absolute URL whose host is `localhost` or a loopback address.
fn is_loopback_origin(value: &str) -> bool {
    let Some((_, rest)) = value.split_once("://") else {
        return false;
    };
    let host = rest
        .split(['/', '?', '#'])
        .next()
        .unwrap_or("")
        .rsplit('@')
        .next()
        .unwrap_or("");
    let host = host
        .strip_prefix('[')
        .and_then(|h| h.split(']').next())
        .unwrap_or_else(|| host.split(':').next().unwrap_or(host));
    host.eq_ignore_ascii_case("localhost")
        || host
            .parse::<IpAddr>()
            .is_ok_and(|address| address.is_loopback())
}

/// `IsCrossSiteRequest`: an `Origin` or `Referer` that is not a loopback origin.
#[must_use]
pub fn is_cross_site(head: &str) -> bool {
    for name in ["Origin", "Referer"] {
        if let Some(value) = header_value(head, name)
            && !value.is_empty()
            && !is_loopback_origin(&value)
        {
            return true;
        }
    }
    false
}

/// `/guided?lat=..&lng=..&alt=..`'s numbers.
fn guided_query(url: &str) -> Option<(f64, f64, f32)> {
    let pattern =
        regex::Regex::new(r"(?i)lat=([\-\.0-9]+)&lng=([\-\.0-9]+)&alt=([\.0-9]+)").ok()?;
    let captures = pattern.captures(url)?;
    Some((
        captures.get(1)?.as_str().parse().ok()?,
        captures.get(2)?.as_str().parse().ok()?,
        captures.get(3)?.as_str().parse().ok()?,
    ))
}

/// `POST /guide`'s body, `{"lat":..,"lon":..,"alt":..}` as the regex finds it in the head.
fn guided_body(head: &str) -> Option<(f64, f64, f32)> {
    let pattern =
        regex::Regex::new(r#"(?i)lat":([\-\.0-9]+),"lon":([\-\.0-9]+),"alt":([\.0-9]+)"#).ok()?;
    let captures = pattern.captures(head)?;
    Some((
        captures.get(1)?.as_str().parse().ok()?,
        captures.get(2)?.as_str().parse().ok()?,
        captures.get(3)?.as_str().parse().ok()?,
    ))
}

/// The guided routes: 403 to anyone but a loopback client whose request no other site's page
/// sent; else `setGuidedModeWP` asked of the window and "Sent Guide Mode Wp", or "Failed Guide
/// Mode Wp" when the numbers are not there.
/// `// C#: Utilities/httpserver.cs:884-982`
fn guided<C: Connection>(
    stream: &mut C,
    head: &str,
    shared: &Shared,
    point: Option<(f64, f64, f32)>,
) {
    let loopback = stream.peer().is_some_and(|ip| ip.is_loopback());
    if !loopback || is_cross_site(head) {
        let _ = write_all(stream, &[b"HTTP/1.1 403 Forbidden\r\n\r\nForbidden"]);
        return;
    }
    match point {
        Some((lat, lng, alt)) => {
            lock(&shared.commands).push(Command::Guided { lat, lng, alt });
            let _ = write_all(stream, &[b"HTTP/1.1 200 OK\r\n\r\nSent Guide Mode Wp"]);
        }
        None => {
            let _ = write_all(stream, &[b"HTTP/1.1 200 OK\r\n\r\nFailed Guide Mode Wp"]);
        }
    }
}

/// The request line's path, `([^\s]+)\s(.+)\sHTTP/1`.
fn request_path(url: &str) -> Option<&str> {
    let mut parts = url.splitn(3, ' ');
    let _method = parts.next()?;
    let path = parts.next()?;
    parts.next().filter(|rest| rest.starts_with("HTTP/1"))?;
    Some(path)
}

/// `/mav/`: Mavelous's files from `mavelous_web`, a path kept inside it (403 otherwise),
/// `index.html` for the bare path, the content type by extension, `Last-Modified` and a 304 for
/// an `If-Modified-Since` that matches it, 404 for a file that is not there; the connection kept.
/// `// C#: Utilities/httpserver.cs:1148-1242`
fn static_file<C: Connection>(stream: &mut C, head: &str, url: &str, root: &Path) -> bool {
    let Some(path) = request_path(url) else {
        return not_found_keep(stream);
    };
    let decoded = percent_decode(path);
    let mut file = decoded.replace("/mav/", "");
    if file.is_empty() || file == "/" {
        file = "index.html".to_owned();
    }
    let full = root.join(file.trim_start_matches('/'));
    let inside = match (root.canonicalize(), full.canonicalize()) {
        (Ok(base), Ok(full)) => full == base || full.starts_with(&base),
        _ => false,
    };
    if !inside {
        if full.exists() {
            let _ = write_all(
                stream,
                &[b"HTTP/1.1 403 Forbidden\r\nContent-Length: 0\r\n\r\n"],
            );
            return true;
        }
        return not_found_keep(stream);
    }
    let Ok(bytes) = std::fs::read(&full) else {
        return not_found_keep(stream);
    };
    let content_type = if file.contains(".htm") {
        "text/html"
    } else if file.contains(".js") {
        "application/x-javascript"
    } else if file.contains(".css") {
        "text/css"
    } else {
        "text/plain"
    };
    let modified = std::fs::metadata(&full)
        .and_then(|meta| meta.modified())
        .ok()
        .map(|time| {
            chrono::DateTime::<chrono::Utc>::from(time)
                .format("%a, %d %b %Y %H:%M:%S GMT")
                .to_string()
        })
        .unwrap_or_default();
    if let Some(since) = header_value(head, "If-Modified-Since")
        && since.eq_ignore_ascii_case(&modified)
    {
        return write_all(
            stream,
            &[b"HTTP/1.1 304 not modified\r\nConnection: Keep-Alive\r\n\r\n"],
        );
    }
    let header = format!(
        "HTTP/1.1 200 OK\r\nContent-Type: {content_type}\r\nConnection: keep-alive\r\nLast-Modified: {modified}\r\nContent-Length: {}\r\n\r\n",
        bytes.len()
    );
    write_all(stream, &[header.as_bytes(), &bytes])
}

/// The 404 that keeps the connection, `Content -Type` as the C# misspells it.
fn not_found_keep<C: Write>(stream: &mut C) -> bool {
    write_all(
        stream,
        &[b"HTTP/1.1 404 not found\r\nConnection: Keep-Alive\r\nContent-Length: 0\r\nContent -Type: text/plain\r\n\r\n"],
    )
}

/// `WebUtility.UrlDecode`.
fn percent_decode(text: &str) -> String {
    let bytes = text.as_bytes();
    let mut out = Vec::with_capacity(bytes.len());
    let mut index = 0;
    while index < bytes.len() {
        let byte = bytes.get(index).copied().unwrap_or(0);
        if byte == b'%'
            && let Some(hex) = bytes.get(index + 1..index + 3)
            && let Ok(text) = std::str::from_utf8(hex)
            && let Ok(value) = u8::from_str_radix(text, 16)
        {
            out.push(value);
            index += 3;
        } else if byte == b'+' {
            out.push(b' ');
            index += 1;
        } else {
            out.push(byte);
            index += 1;
        }
    }
    String::from_utf8_lossy(&out).into_owned()
}

/// `.jpg`: the photo at `georefimagepath + path`, fitted to 640 by 480 (`ResizeImage`, the
/// aspect kept), as JPEG; 404 when it cannot be read, where the C#'s exception closes the
/// connection.
/// `// C#: Utilities/httpserver.cs:1244-1293, 1392-1419`
fn photo<C: Connection>(stream: &mut C, url: &str, dir: &str) -> bool {
    let Some(path) = request_path(url) else {
        not_found(stream, "image/jpg");
        return false;
    };
    let file = format!("{dir}{path}");
    let Ok(image) = image::open(&file) else {
        not_found(stream, "image/jpg");
        return false;
    };
    let (width, height) = (image.width(), image.height());
    #[allow(clippy::cast_precision_loss)] // picture sizes
    let percent = (640.0 / width as f32).min(480.0 / height as f32);
    #[allow(
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss,
        clippy::cast_precision_loss
    )]
    let (new_width, new_height) = (
        ((width as f32 * percent) as u32).max(1),
        ((height as f32 * percent) as u32).max(1),
    );
    let resized = image.resize_exact(
        new_width,
        new_height,
        image::imageops::FilterType::CatmullRom,
    );
    let rgb = resized.to_rgb8();
    let mut jpeg = Vec::new();
    if image::codecs::jpeg::JpegEncoder::new_with_quality(&mut jpeg, 75)
        .encode(
            rgb.as_raw(),
            rgb.width(),
            rgb.height(),
            image::ExtendedColorType::Rgb8,
        )
        .is_err()
    {
        not_found(stream, "image/jpg");
        return false;
    }
    let header = format!(
        "HTTP/1.1 200 OK\r\nServer: here\r\nKeep-Alive: timeout=15, max=100\r\nConnection: Keep-Alive\r\nContent-Type: image/jpg\r\nX-Pad: avoid browser bug\r\nContent-Length: {}\r\n\r\n",
        jpeg.len()
    );
    write_all(stream, &[header.as_bytes(), &jpeg])
}

// ---------------------------------------------------------------------------------------------
// The window's side.
// ---------------------------------------------------------------------------------------------

/// What the window keeps for the server: the server, the link's packet tap and the last of the
/// eight messages, and when the HUD's frame was last made.
pub struct Host {
    /// The server.
    pub server: Server,
    subscription: Option<mp_link::inspector::PacketSubscription>,
    last: Arc<Mutex<LastMessages>>,
    next_hud: Instant,
    hud_jpeg: Option<Arc<Vec<u8>>>,
    /// `packetindex`.
    index: i64,
    /// The Geo Reference form's KML last handed over, so the hand-over is once a change.
    georef_handed: Option<String>,
}

impl Host {
    /// The server started, as `MainV2` starts it with the window.
    #[must_use]
    pub fn start() -> Self {
        Self {
            server: Server::start(crate::help::install_dir()),
            subscription: None,
            last: Arc::new(Mutex::new(LastMessages::default())),
            next_hud: Instant::now(),
            hud_jpeg: None,
            index: 0,
            georef_handed: None,
        }
    }
}

impl MissionPlanner {
    /// Once a frame: the clients' commands done, the link's packets tapped for the raw websockets
    /// and `/mavlink/`, the HUD's frame made while a stream wants one, and the snapshot
    /// published.
    pub(crate) fn http_tick(&mut self, view: &TelemetryView) {
        if !self.http.server.running() {
            return;
        }
        // `OnPacketReceived`: the tap follows the link.
        let stale = self
            .http
            .subscription
            .as_ref()
            .is_none_or(|subscription| !self.telemetry.carries(subscription));
        if stale {
            let shared = Arc::clone(self.http.server.shared());
            let last = Arc::clone(&self.http.last);
            self.http.subscription = self.telemetry.on_packet(move |packet| {
                if packet.sent {
                    return;
                }
                lock(&last).remember(packet.msgid, &packet.message);
                if shared.has_raw_clients()
                    && let Some(bytes) = reencode(packet.sysid, packet.compid, &packet.message)
                {
                    shared.forward_raw(&bytes);
                }
            });
        }
        for command in self.http.server.take_commands() {
            match command {
                // `MainV2.comPort.setGuidedModeWP(gwp)`: `Locationwp`'s frame, 0.
                Command::Guided { lat, lng, alt } => {
                    self.fly_press(
                        &crate::telemetry::Report::default(),
                        |actions, target, view| {
                            crate::fly::guided_sends(actions, target, view, (lat, lng, alt), 0)
                        },
                    );
                }
                Command::Send(message) => {
                    self.telemetry.send(&message);
                }
            }
        }
        let now = Instant::now();
        if self.http.server.hud_wanted() && now >= self.http.next_hud {
            self.http.next_hud = now + PERIOD;
            let (width, height) =
                self.fly_data
                    .hud_bounds
                    .get()
                    .map_or((398.0, 258.0), |laid_out| {
                        (
                            f32::from(laid_out.size.width),
                            f32::from(laid_out.size.height),
                        )
                    });
            let scene = crate::fly::hud_scene(
                &self.hud,
                self.fly_data.hud_settings.ground_colours(),
                width,
                height,
            );
            #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // sizes in pixels
            let (w, h) = (width.max(1.0) as u32, height.max(1.0) as u32);
            self.http.hud_jpeg = crate::hud::frame_jpeg(&scene, w, h).map(Arc::new);
        }
        let default = mp_vehicle::VehicleState::default();
        let state = view.state.as_deref().unwrap_or(&default);
        let located = |state: &mp_vehicle::VehicleState| Located {
            lat: state.position.map_or(0.0, |p| p.latitude()),
            lng: state.position.map_or(0.0, |p| p.longitude()),
            altasl: state.altitude_msl.0,
            yaw: state.attitude.yaw.to_degrees().0,
            roll: state.attitude.roll.to_degrees().0,
            pitch: state.attitude.pitch.to_degrees().0,
        };
        let vehicles: Vec<Located> = view.state.as_deref().map(located).into_iter().collect();
        let mut plan_points = Vec::new();
        if let Some(home) = self.plan.home() {
            plan_points.push(PlanPoint {
                tag: "Home".to_owned(),
                lat: home.lat,
                lng: home.lng,
                alt: home.alt,
            });
        }
        for item in self.plan.items() {
            if item.x != 0.0 || item.y != 0.0 {
                plan_points.push(PlanPoint {
                    tag: item.seq.to_string(),
                    lat: item.x,
                    lng: item.y,
                    alt: item.z,
                });
            }
        }
        // `httpGeoRefKML`: the Geo Reference form's KML and photo folder, when they change.
        if let Some((kml, dir)) = self.georef.http_kml()
            && self.http.georef_handed.as_deref() != Some(kml)
        {
            self.http.georef_handed = Some(kml.to_owned());
            self.http.server.set_georef(kml.to_owned(), dir);
        }
        self.http.index += 1;
        let mavlink_json = lock(&self.http.last).json(
            i64::try_from(state.link.received).unwrap_or(i64::MAX),
            i64::try_from(view.frames).unwrap_or(i64::MAX),
            100.0 - f64::from(state.link.quality_percent()),
            self.http.index,
        );
        self.http.server.publish(Snapshot {
            cs_json: crate::quick::current_state_json(state),
            wps_json: wps_json(&view.wps),
            vehicles,
            primary: view.state.as_deref().map(located),
            plan_points,
            mavlink_json,
            hud_jpeg: self.http.hud_jpeg.clone(),
        });
    }
}

#[cfg(test)]
mod tests {
    use std::io::Cursor;

    use super::*;

    /// A client's connection in memory: what it sent, and what it is answered.
    struct Duplex {
        input: Cursor<Vec<u8>>,
        output: Vec<u8>,
        peer: IpAddr,
    }

    impl Duplex {
        fn new(request: &str, peer: IpAddr) -> Self {
            Self {
                input: Cursor::new(request.as_bytes().to_vec()),
                output: Vec::new(),
                peer,
            }
        }

        fn answer(&self) -> String {
            String::from_utf8_lossy(&self.output).into_owned()
        }
    }

    impl Read for Duplex {
        fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
            self.input.read(buf)
        }
    }

    impl Write for Duplex {
        fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
            self.output.extend_from_slice(buf);
            Ok(buf.len())
        }

        fn flush(&mut self) -> std::io::Result<()> {
            Ok(())
        }
    }

    impl Connection for Duplex {
        fn has_input(&mut self) -> bool {
            usize::try_from(self.input.position()).is_ok_and(|at| at < self.input.get_ref().len())
        }

        fn peer(&self) -> Option<IpAddr> {
            Some(self.peer)
        }

        fn connected(&self) -> bool {
            false
        }
    }

    fn shared() -> Shared {
        let shared = Shared::default();
        shared.run.store(true, Ordering::Release);
        shared
    }

    fn get(path: &str, peer: IpAddr, shared: &Shared, files: &Path) -> String {
        let mut client = Duplex::new(&format!("GET {path} HTTP/1.1\r\nHost: x\r\n\r\n"), peer);
        serve(&mut client, shared, files);
        client.answer()
    }

    const LOOPBACK: IpAddr = IpAddr::V4(std::net::Ipv4Addr::LOCALHOST);
    const ELSEWHERE: IpAddr = IpAddr::V4(std::net::Ipv4Addr::new(10, 0, 0, 7));

    /// The handshake's hash is RFC 6455's example.
    #[test]
    fn the_websocket_accept_is_the_rfcs() {
        assert_eq!(
            websocket_accept("dGhlIHNhbXBsZSBub25jZQ=="),
            "s3pPLMBiTxaQ9kYGzzhZRbK+xOo="
        );
        assert_eq!(websocket_frame(0x81, b"ab"), vec![0x81, 2, b'a', b'b']);
        let long = vec![b'x'; 300];
        let frame = websocket_frame(0x82, &long);
        assert_eq!(&frame[..4], &[0x82, 126, 1, 44]);
        assert_eq!(frame.len(), 304);
    }

    /// The index, the network link and an unknown path.
    #[test]
    fn the_index_the_network_kml_and_a_404() {
        let shared = shared();
        let dir = std::env::temp_dir();
        let index = get("/", LOOPBACK, &shared, &dir);
        assert!(
            index.starts_with("HTTP/1.1 200 OK\r\nConnection: close\r\nContent-Type: text/html")
        );
        assert!(index.contains("<a href=/network.kml>network kml</a>"));
        let kml = get("/network.kml", LOOPBACK, &shared, &dir);
        assert!(kml.contains("Content-Type: application/vnd.google-earth.kml+xml"));
        assert!(kml.contains("http://127.0.0.1:56781/location.kml"));
        let missing = get("/nothing", LOOPBACK, &shared, &dir);
        assert!(missing.starts_with("HTTP/1.1 404 not found\r\nContent-Type: text/plain"));
        assert_eq!(shared.requests.load(Ordering::Acquire), 3);
        assert_eq!(*lock(&shared.last_url), "GET /nothing HTTP/1.1");
    }

    /// `location.kml`: the view on the first vehicle and a model a vehicle, a negative altitude
    /// lifted to 0.01.
    #[test]
    fn location_kml_models_each_vehicle() {
        let one = Located {
            lat: -35.3,
            lng: 149.1,
            altasl: -2.0,
            yaw: 90.0,
            roll: 5.0,
            pitch: -3.0,
        };
        let kml = location_kml(&[one], Some(&one));
        assert!(kml.contains("<LookAt><longitude>149.1</longitude><latitude>-35.3</latitude><altitude>-2</altitude><heading>90</heading><tilt>80</tilt>"));
        assert!(kml.contains("<name>P/Q -2</name>"));
        assert!(kml.contains("<altitude>0.01</altitude>"));
        assert!(kml.contains("<heading>90</heading><tilt>3</tilt><roll>-5</roll>"));
        assert!(kml.contains("<href>block_plane_0.dae</href>"));
        let none = location_kml(&[], None);
        assert!(!none.contains("LookAt"));
    }

    /// `wps.kml`: a placemark a point and the two tracks under the one style id.
    #[test]
    fn wps_kml_has_the_points_and_both_tracks() {
        let points = vec![
            PlanPoint {
                tag: "Home".to_owned(),
                lat: -35.0,
                lng: 149.0,
                alt: 580.0,
            },
            PlanPoint {
                tag: "1".to_owned(),
                lat: -35.001,
                lng: 149.001,
                alt: 600.0,
            },
        ];
        let kml = wps_kml(&points);
        assert!(kml.contains("<name>WP Home Alt: 580</name>"));
        assert!(kml.contains("<name>WP 1 Alt: 600</name>"));
        assert_eq!(kml.matches("yellowLineGreenPoly").count(), 2);
        assert!(kml.contains("<color>ff00ffff</color>"));
        assert!(kml.contains("<color>7f00ffff</color>"));
        assert!(kml.contains("<altitudeMode>clampToGround</altitudeMode>"));
        assert!(kml.contains("149,-35,580 149.001,-35.001,600"));
    }

    /// The guided routes: a loopback client's numbers become a command; another host, or a
    /// page from another site, is forbidden; no numbers is "Failed".
    #[test]
    fn guided_takes_loopback_requests_only() {
        let shared = shared();
        let dir = std::env::temp_dir();
        let ok = get("/guided?lat=-34&lng=117.8&alt=30", LOOPBACK, &shared, &dir);
        assert!(ok.ends_with("Sent Guide Mode Wp"));
        assert_eq!(
            lock(&shared.commands).as_slice(),
            &[Command::Guided {
                lat: -34.0,
                lng: 117.8,
                alt: 30.0
            }]
        );
        let far = get("/guided?lat=-34&lng=117.8&alt=30", ELSEWHERE, &shared, &dir);
        assert!(far.starts_with("HTTP/1.1 403 Forbidden"));
        let mut cross = Duplex::new(
            "GET /guided?lat=-34&lng=117.8&alt=30 HTTP/1.1\r\nOrigin: http://evil.example\r\n\r\n",
            LOOPBACK,
        );
        serve(&mut cross, &shared, &dir);
        assert!(cross.answer().starts_with("HTTP/1.1 403 Forbidden"));
        let bad = get("/guided?lat=x", LOOPBACK, &shared, &dir);
        assert!(bad.ends_with("Failed Guide Mode Wp"));
        let mut post = Duplex::new(
            "POST /guide HTTP/1.1\r\nHost: x\r\nReferer: http://localhost:56781/mav/\r\n\r\n{\"lat\":-35.5,\"lon\":149.2,\"alt\":40}",
            LOOPBACK,
        );
        serve(&mut post, &shared, &dir);
        assert!(post.answer().ends_with("Sent Guide Mode Wp"));
        assert_eq!(lock(&shared.commands).len(), 2);
    }

    /// `/mavlink/`: the last messages by name, `SYS_STATUS` carrying `META_LINKQUALITY` as the
    /// C# assigns it, and the three 404 routes.
    #[test]
    fn mavlink_json_is_the_csharps() {
        let mut last = LastMessages::default();
        last.remember(
            0,
            &MavMessage::Heartbeat(mp_mavlink_dialects::all::Heartbeat {
                custom_mode: 4,
                r#type: 2,
                autopilot: 3,
                base_mode: 81,
                system_status: 4,
                mavlink_version: 3,
            }),
        );
        last.remember(
            1,
            &MavMessage::SysStatus(mp_mavlink_dialects::all::SysStatus {
                onboard_control_sensors_present: 1,
                onboard_control_sensors_enabled: 1,
                onboard_control_sensors_health: 1,
                load: 100,
                voltage_battery: 12000,
                current_battery: -1,
                drop_rate_comm: 0,
                errors_comm: 0,
                errors_count1: 0,
                errors_count2: 0,
                errors_count3: 0,
                errors_count4: 0,
                battery_remaining: -1,
            }),
        );
        let json = last.json(11110, 194, 1.5, 7);
        assert!(json.contains("\"HEARTBEAT\":{\"msg\":{\"custom_mode\":4,\"type\":2,\"autopilot\":3,\"base_mode\":81,\"system_status\":4,\"mavlink_version\":3},\"index\":1,\"time_usec\":0}"));
        assert!(json.contains("\"META_LINKQUALITY\":{\"msg\":{\"master_in\":11110,\"mav_loss\":0,\"mavpackettype\":\"META_LINKQUALITY\",\"master_out\":194,\"packet_loss\":1.5},\"index\":7,\"time_usec\":0}"));
        assert!(json.contains("\"SYS_STATUS\":{\"msg\":{\"master_in\":11110"));
        assert!(
            serde_json::from_str::<serde_json::Value>(&json).is_ok(),
            "{json}"
        );
        let shared = shared();
        for path in ["/command_long", "/rcoverride", "/get_mission"] {
            let answer = get(path, LOOPBACK, &shared, &std::env::temp_dir());
            assert!(answer.starts_with("HTTP/1.1 404 not found\r\nContent-Type: image/jpg"));
        }
    }

    /// `/mav/`: a file inside `mavelous_web`, its type, a 304, and a path that climbs out.
    #[test]
    fn mavelous_files_stay_inside_their_folder() {
        let dir = std::env::temp_dir().join(format!("mp-http-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(dir.join("mavelous_web")).unwrap();
        std::fs::write(dir.join("mavelous_web/index.html"), "<html>mavelous</html>").unwrap();
        std::fs::write(dir.join("secret.txt"), "no").unwrap();
        let shared = shared();
        let page = get("/mav/", LOOPBACK, &shared, &dir);
        assert!(page.starts_with("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nConnection: keep-alive\r\nLast-Modified: "));
        assert!(page.ends_with("<html>mavelous</html>"));
        let modified = page
            .lines()
            .find_map(|line| line.strip_prefix("Last-Modified: "))
            .unwrap()
            .to_owned();
        let mut again = Duplex::new(
            &format!("GET /mav/index.html HTTP/1.1\r\nIf-Modified-Since: {modified}\r\n\r\n"),
            LOOPBACK,
        );
        serve(&mut again, &shared, &dir);
        assert!(again.answer().starts_with("HTTP/1.1 304 not modified"));
        let climb = get("/mav/../secret.txt", LOOPBACK, &shared, &dir);
        assert!(
            climb.starts_with("HTTP/1.1 403 Forbidden") || climb.starts_with("HTTP/1.1 404"),
            "{climb}"
        );
        assert!(!climb.contains("no\r"));
        let missing = get("/mav/none.js", LOOPBACK, &shared, &dir);
        assert!(missing.starts_with("HTTP/1.1 404 not found\r\nConnection: Keep-Alive"));
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// `wps` as JSON: `Locationwp`'s fields by sequence number.
    #[test]
    fn wps_json_is_locationwp_by_seq() {
        let item = MissionItem {
            seq: 1,
            frame: 3,
            command: 16,
            param1: 0.0,
            x: -35.0,
            y: 149.0,
            z: 50.0,
            ..MissionItem::default()
        };
        let json = wps_json(&[item]);
        assert_eq!(
            json,
            "{\"1\":{\"frame\":3,\"Tag\":null,\"id\":16,\"p1\":0,\"p2\":0,\"p3\":0,\"p4\":0,\"lat\":-35,\"lng\":149,\"alt\":50}}"
        );
        assert!(serde_json::from_str::<serde_json::Value>(&json).is_ok());
    }

    /// `map.jpg` says what it cannot do; a cross-site check knows a loopback origin.
    #[test]
    fn the_map_stream_is_not_here_and_origins_are_judged() {
        let shared = shared();
        let answer = get("/map.jpg", LOOPBACK, &shared, &std::env::temp_dir());
        assert!(answer.starts_with("HTTP/1.1 404 not found"));
        assert!(answer.contains(NO_MAP_IMAGE));
        assert!(!is_cross_site(
            "GET / HTTP/1.1\r\nOrigin: http://localhost:56781\r\n"
        ));
        assert!(!is_cross_site(
            "GET / HTTP/1.1\r\nReferer: http://127.0.0.1/x\r\n"
        ));
        assert!(is_cross_site(
            "GET / HTTP/1.1\r\nOrigin: http://example.com\r\n"
        ));
        assert!(!is_cross_site("GET / HTTP/1.1\r\n"));
    }
}
