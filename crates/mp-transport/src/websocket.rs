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

//! The websocket client: Mission Planner's "WS" link.
//!
//! Ported from `ExtLibs/Comms/CommsWebSocket.cs` (`WebSocket`), which is .NET's `ClientWebSocket`
//! with a reader beside it. MAVLink goes both ways as binary messages; text messages are only
//! for the socket.io handshake the C# half-speaks (it sends engine.io's `2probe` on every open, and
//! answers `3` with a namespace connect and an upgrade). The client half of RFC 6455 - the opening
//! handshake, masked frames out, unmasked frames in, fragments, ping and close - is written here,
//! doing what `ClientWebSocket` does as recorded under mono in `testdata/comms/golden/ws.txt`.
//!
//! # One thread, not two
//!
//! The C# reads in a background task (`RunReader`) into a buffer that `Read` drains. Here the same
//! reading happens inside [`Transport::read`] when nothing is buffered, which is the thread the
//! link engine already dedicates to the link: the same bytes in the same order, and the reconnect
//! the reader does after a failure happens at the same point in the stream.
//!
//! # Not ported
//!
//! - `wss://`: it needs TLS, which needs a TLS crate. Opening a `wss://` URL is
//!   [`OpenError::Unsupported`].
//! - What `ClientWebSocket` does on its own that the C# never asks for: an unsolicited pong every
//!   30 seconds (`ClientWebSocketOptions.KeepAliveInterval`), and a close frame with status 1002
//!   before it gives up on a server that broke the protocol. A MAVLink link is never idle for
//!   30 seconds - the GCS heartbeat alone is once a second - and after a protocol error the C#
//!   reconnects either way, which this does.

use std::io::{self, Read, Write};
use std::net::TcpStream;
use std::time::Duration;

use crate::codec::{base64, random_u64, sha1};
use crate::dotnet::Uri;
use crate::socket::is_timeout;
use crate::{DEFAULT_READ_TIMEOUT, OpenError, Transport};

/// RFC 6455 section 1.3's GUID, appended to the key to make the accept value.
const ACCEPT_GUID: &str = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";

/// How long the C#'s reader waits before reopening after a failure.
/// `// C#: ExtLibs/Comms/CommsWebSocket.cs:182`
pub const RECONNECT_DELAY: Duration = Duration::from_millis(100);

/// The C#'s receive buffer: a text message is looked at this much at a time.
/// `// C#: ExtLibs/Comms/CommsWebSocket.cs:135`
const RECEIVE_CHUNK: usize = 1024 * 8;

const OP_CONTINUATION: u8 = 0x0;
const OP_TEXT: u8 = 0x1;
const OP_BINARY: u8 = 0x2;
const OP_CLOSE: u8 = 0x8;
const OP_PING: u8 = 0x9;
const OP_PONG: u8 = 0xA;

/// Mission Planner's websocket link.
#[derive(Debug)]
pub struct WebSocketTransport {
    /// `_url`: what is (re)opened, which socket.io's session id is appended to.
    url: String,
    stream: Option<TcpStream>,
    /// `client.State == Open`.
    open: bool,
    /// `autoReconnect`: cleared by `Close`.
    auto_reconnect: bool,
    /// `socketio`: set by engine.io's open packet; writes then carry engine.io's message prefix.
    socketio: bool,
    /// A write failed. `ClientWebSocket` aborts on a failed send, which fails the reader's pending
    /// receive; here the next read takes that path.
    broken: bool,
    /// Bytes from the server not yet made into frames.
    incoming: Vec<u8>,
    /// The frame being received, once its header is in.
    frame: Option<FrameHead>,
    /// The kind of the message in progress, for its continuation frames.
    message: Option<u8>,
    /// A control frame's payload, or the text received since the last look at it.
    control: Vec<u8>,
    text: Vec<u8>,
    /// `readMemoryStream` from `readCursor` on: binary payload not yet read.
    received: Vec<u8>,
    cursor: usize,
    scratch: Vec<u8>,
    /// The frame being sent, kept so sending allocates only when a frame is longer than any
    /// before it.
    outgoing: Vec<u8>,
    timeout: Duration,
    description: String,
}

#[derive(Debug, Clone, Copy)]
struct FrameHead {
    fin: bool,
    opcode: u8,
    remaining: u64,
}

/// What a frame's arrival asks of the transport.
enum Event {
    Nothing,
    /// A protocol violation: `ClientWebSocket` throws from the receive, and the reader reconnects.
    Fail,
    /// The server's close frame.
    Closed,
}

impl WebSocketTransport {
    /// `Open()`: connects to `url` (`ws://host[:port]/path`), shakes hands, and sends `2probe`.
    ///
    /// The C# takes the URL from an input box that defaults to `WS_url` in config.xml, and saves it
    /// back; here the link URL is the URL. Unlike the C#, whose `Open(string)` is `async void` so
    /// that a failed connect never reaches its caller (the reader then finds the socket not open
    /// and stops), a failure to connect is returned.
    /// `// C#: ExtLibs/Comms/CommsWebSocket.cs:93-123, 192-204`
    pub fn open(url: &str) -> Result<Self, OpenError> {
        let target = Uri::parse(url)
            .map_err(|e| OpenError::io(format!("parsing {url}"), io::Error::other(e)))?;
        let description = format!(
            "{}://{}:{}{}",
            target.scheme,
            target.host,
            target.port.unwrap_or_default(),
            target.path_and_query
        );
        let mut transport = Self {
            url: url.to_owned(),
            stream: None,
            open: false,
            auto_reconnect: true,
            socketio: false,
            broken: false,
            incoming: Vec::new(),
            frame: None,
            message: None,
            control: Vec::new(),
            text: Vec::new(),
            received: Vec::new(),
            cursor: 0,
            scratch: vec![0; RECEIVE_CHUNK],
            outgoing: Vec::new(),
            timeout: DEFAULT_READ_TIMEOUT,
            description,
        };
        transport.connect().map_err(|e| match e {
            ConnectError::Tls => OpenError::Unsupported("TLS (wss://)"),
            ConnectError::Io(e) => OpenError::io(format!("opening {url}"), e),
        })?;
        Ok(transport)
    }

    /// The URL a reconnect opens, with socket.io's `&sid=` once the server has named a session.
    #[must_use]
    pub fn url(&self) -> &str {
        &self.url
    }

    /// Whether engine.io's open packet has been seen, so writes are prefixed as its messages.
    #[must_use]
    pub const fn is_socketio(&self) -> bool {
        self.socketio
    }

    /// `Open(string url)`: a new connection, the handshake, and engine.io's probe.
    /// `// C#: ExtLibs/Comms/CommsWebSocket.cs:192-204`
    fn connect(&mut self) -> Result<(), ConnectError> {
        self.drop_connection();
        let target = Uri::parse(&self.url).map_err(|e| ConnectError::Io(io::Error::other(e)))?;
        if target.scheme == "wss" {
            return Err(ConnectError::Tls);
        }
        if target.scheme != "ws" {
            // ClientWebSocket.ConnectAsync takes nothing else.
            return Err(ConnectError::Io(io::Error::new(
                io::ErrorKind::InvalidInput,
                format!(
                    "only ws:// and wss:// URLs are websockets, not {}",
                    self.url
                ),
            )));
        }
        let port = target.port.unwrap_or(80);
        let host = target
            .host
            .trim_start_matches('[')
            .trim_end_matches(']')
            .to_owned();
        let mut stream = TcpStream::connect((host.as_str(), port))?;
        stream.set_nodelay(true)?;

        // The opening handshake, header for header as ClientWebSocket writes it (ws.txt): the
        // host with its port always, and nothing from the URL's user info.
        let mut nonce = [0u8; 16];
        for (chunk, random) in nonce.chunks_exact_mut(8).zip([random_u64(), random_u64()]) {
            chunk.copy_from_slice(&random.to_le_bytes());
        }
        let key = base64(&nonce);
        let request = format!(
            "GET {} HTTP/1.1\r\nHost: {}:{port}\r\nConnection: Upgrade\r\nUpgrade: websocket\r\n\
             Sec-WebSocket-Version: 13\r\nSec-WebSocket-Key: {key}\r\n\r\n",
            target.path_and_query, target.host
        );
        stream.write_all(request.as_bytes())?;

        // The response, read to its blank line with no time limit, as ConnectAsync is called with
        // CancellationToken.None; what follows it is already the frame stream.
        stream.set_read_timeout(None)?;
        let mut response = Vec::new();
        let head_end = loop {
            if let Some(at) = find(&response, b"\r\n\r\n") {
                break at + 4;
            }
            let n = stream.read(&mut self.scratch)?;
            if n == 0 {
                return Err(ConnectError::Io(io::Error::new(
                    io::ErrorKind::UnexpectedEof,
                    "the server closed the connection during the websocket handshake",
                )));
            }
            response.extend_from_slice(self.scratch.get(..n).unwrap_or_default());
        };
        let head = String::from_utf8_lossy(response.get(..head_end).unwrap_or_default());
        check_handshake(&head, &key).map_err(ConnectError::Io)?;
        self.incoming.clear();
        self.incoming
            .extend_from_slice(response.get(head_end..).unwrap_or_default());
        self.frame = None;
        self.message = None;
        self.control.clear();
        self.text.clear();
        stream.set_read_timeout(Some(self.timeout))?;
        self.stream = Some(stream);
        self.open = true;
        self.broken = false;

        // engine.io's probe, sent on every open whatever the server is.
        // C#: CommsWebSocket.cs:198-203
        self.send_frame(OP_TEXT, &[], b"2probe");
        Ok(())
    }

    fn drop_connection(&mut self) {
        if let Some(stream) = self.stream.take() {
            let _ = stream.shutdown(std::net::Shutdown::Both);
        }
        self.open = false;
    }

    /// Sends one frame of `prefix` then `payload`, final and masked; a failure marks the
    /// connection broken and is not reported, as the C# logs and swallows it.
    /// `// C#: ExtLibs/Comms/CommsWebSocket.cs:281-303`
    fn send_frame(&mut self, opcode: u8, prefix: &[u8], payload: &[u8]) {
        let Some(stream) = self.stream.as_mut() else {
            return;
        };
        let mask = random_u64().to_le_bytes();
        let frame = &mut self.outgoing;
        frame.clear();
        frame.push(0x80 | opcode);
        match prefix.len() + payload.len() {
            len @ 0..=125 => frame.push(0x80 | u8::try_from(len).unwrap_or(125)),
            len @ 126..=0xFFFF => {
                frame.push(0x80 | 126);
                frame.extend_from_slice(&u16::try_from(len).unwrap_or(u16::MAX).to_be_bytes());
            }
            len => {
                frame.push(0x80 | 127);
                frame.extend_from_slice(&(len as u64).to_be_bytes());
            }
        }
        let key = mask.get(..4).unwrap_or(&[0; 4]);
        frame.extend_from_slice(key);
        frame.extend(
            prefix
                .iter()
                .chain(payload)
                .zip(key.iter().cycle())
                .map(|(byte, mask)| byte ^ mask),
        );
        if stream.write_all(frame).is_err() {
            self.broken = true;
        }
    }

    /// The catch in `RunReader`: after a failed receive, wait a moment and open again.
    ///
    /// If the reopen fails, the C#'s `async void Open` throws where nobody catches it and the new
    /// client is never open, so the reader's loop ends; here the link is simply closed.
    /// `// C#: ExtLibs/Comms/CommsWebSocket.cs:176-185`
    fn receive_failed(&mut self) {
        self.drop_connection();
        if self.auto_reconnect {
            std::thread::sleep(RECONNECT_DELAY);
            if self.connect().is_err() {
                self.drop_connection();
            }
        }
    }

    /// One pass of `RunReader`'s loop: receive what is there, up to the read timeout.
    /// `// C#: ExtLibs/Comms/CommsWebSocket.cs:129-187`
    fn pump(&mut self) {
        if self.broken {
            self.receive_failed();
            return;
        }
        let Some(stream) = self.stream.as_mut() else {
            self.open = false;
            return;
        };
        match stream.read(&mut self.scratch) {
            // The connection ended without a close frame: ClientWebSocket throws
            // ("closed without completing the close handshake"), and the reader reconnects.
            Ok(0) => self.receive_failed(),
            Ok(n) => {
                self.incoming
                    .extend_from_slice(self.scratch.get(..n).unwrap_or_default());
                match self.parse() {
                    Event::Nothing => {}
                    Event::Fail => self.receive_failed(),
                    Event::Closed => self.open = false,
                }
            }
            Err(e) if is_timeout(&e) => {}
            Err(_) => self.receive_failed(),
        }
    }

    /// Makes frames of what has come in, and acts on each.
    fn parse(&mut self) -> Event {
        loop {
            let Some(mut head) = self.frame else {
                match parse_head(&self.incoming) {
                    Head::Incomplete => return Event::Nothing,
                    Head::Invalid => return Event::Fail,
                    Head::Complete(head, used) => {
                        self.incoming.drain(..used);
                        match self.start_frame(head) {
                            Some(event) => return event,
                            None => {
                                self.frame = Some(head);
                                continue;
                            }
                        }
                    }
                }
            };

            let available = u64::try_from(self.incoming.len()).unwrap_or(u64::MAX);
            let take = usize::try_from(head.remaining.min(available)).unwrap_or(0);
            head.remaining -= take as u64;
            self.frame = Some(head);
            match self.kind(head) {
                OP_BINARY => self
                    .received
                    .extend_from_slice(self.incoming.get(..take).unwrap_or_default()),
                OP_TEXT => {
                    let mut at = 0;
                    while at < take {
                        let end = take.min(at + RECEIVE_CHUNK - self.text.len());
                        self.text
                            .extend_from_slice(self.incoming.get(at..end).unwrap_or_default());
                        at = end;
                        if self.text.len() == RECEIVE_CHUNK {
                            self.on_text();
                        }
                    }
                }
                _ => self
                    .control
                    .extend_from_slice(self.incoming.get(..take).unwrap_or_default()),
            }
            self.incoming.drain(..take);
            if head.remaining > 0 {
                return Event::Nothing;
            }
            self.frame = None;
            match self.end_frame(head) {
                Event::Nothing => {}
                event => return event,
            }
        }
    }

    /// The kind of message a frame carries: its own opcode, or its message's for a continuation.
    fn kind(&self, head: FrameHead) -> u8 {
        if head.opcode == OP_CONTINUATION {
            self.message.unwrap_or(OP_BINARY)
        } else {
            head.opcode
        }
    }

    /// Checks a frame's header against the message in progress. `None` to go on receiving it.
    fn start_frame(&mut self, head: FrameHead) -> Option<Event> {
        match head.opcode {
            OP_CONTINUATION if self.message.is_none() => Some(Event::Fail),
            OP_TEXT | OP_BINARY if self.message.is_some() => Some(Event::Fail),
            OP_TEXT | OP_BINARY => {
                self.message = Some(head.opcode);
                None
            }
            OP_CLOSE | OP_PING | OP_PONG => {
                self.control.clear();
                None
            }
            _ => None,
        }
    }

    /// Acts on a frame that has all arrived.
    fn end_frame(&mut self, head: FrameHead) -> Event {
        match self.kind(head) {
            // Each receive of text is looked at on its own (C#: CommsWebSocket.cs:146-170), and a
            // receive ends where a frame does.
            OP_TEXT if !self.text.is_empty() => self.on_text(),
            // Answered inside ClientWebSocket with the same payload (ws.txt: "server ping hi").
            OP_PING => {
                let payload = std::mem::take(&mut self.control);
                self.send_frame(OP_PONG, &[], &payload);
                self.control = payload;
            }
            // C#: CommsWebSocket.cs:171-174 and 131 - "unknown websocket data", and the state is
            // CloseReceived, so the reader stops. ClientWebSocket does not answer the close frame
            // (ws.txt: "server close 1000	none") and the C# never reconnects after it.
            OP_CLOSE => return Event::Closed,
            _ => {}
        }
        if head.fin && head.opcode != OP_PING && head.opcode != OP_PONG && head.opcode != OP_CLOSE {
            self.message = None;
        }
        Event::Nothing
    }

    /// Looks at the text received, and empties it.
    fn on_text(&mut self) {
        let mut text = std::mem::take(&mut self.text);
        self.look_at_text(&text);
        text.clear();
        self.text = text;
    }

    /// A text message, as the C#'s reader reads it.
    /// `// C#: ExtLibs/Comms/CommsWebSocket.cs:146-170`
    fn look_at_text(&mut self, text: &[u8]) {
        // C# indexes the first character, so an empty text message throws there and the reader
        // reconnects. Reconnecting because a server sent an empty message is a bug; it is
        // ignored instead.
        let Some(&first) = text.first() else {
            return;
        };
        if first == b'0' {
            // engine.io's open packet names the session: `"sid":"([^"]+)"`.
            if let Some(sid) = session_id(text) {
                self.url.push_str("&sid=");
                self.url.push_str(&sid);
                self.socketio = true;
            }
        } else if first == b'3' {
            // The answer to the probe: join the MAVControl namespace, and upgrade.
            self.send_frame(OP_TEXT, &[], b"40/MAVControl,");
            self.send_frame(OP_TEXT, &[], b"5");
        }
    }

    fn take(&mut self, buf: &mut [u8]) -> usize {
        let waiting = self.received.get(self.cursor..).unwrap_or_default();
        let n = waiting.len().min(buf.len());
        if let (Some(to), Some(from)) = (buf.get_mut(..n), waiting.get(..n)) {
            to.copy_from_slice(from);
        }
        self.cursor += n;
        // C#: CommsWebSocket.cs:218-222 - all read, so the buffer starts again.
        if self.cursor == self.received.len() {
            self.received.clear();
            self.cursor = 0;
        }
        n
    }
}

impl Transport for WebSocketTransport {
    /// `Read`: what the reader has received, receiving first if there is nothing.
    /// `// C#: ExtLibs/Comms/CommsWebSocket.cs:206-231`
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        if buf.is_empty() {
            return Ok(0);
        }
        if self.cursor == self.received.len() && (self.open || self.broken) {
            self.pump();
        }
        Ok(self.take(buf))
    }

    /// `Write`: one binary message; after engine.io's open packet, prefixed with its message type
    /// `4`. A failure is logged and swallowed by the C#, and not reported here.
    /// `// C#: ExtLibs/Comms/CommsWebSocket.cs:281-303`
    fn write_all(&mut self, buf: &[u8]) -> io::Result<()> {
        let prefix: &[u8] = if self.socketio { &[4] } else { &[] };
        self.send_frame(OP_BINARY, prefix, buf);
        Ok(())
    }

    fn description(&self) -> &str {
        &self.description
    }

    /// `IsOpen`: `client.State == WebSocketState.Open`.
    /// `// C#: ExtLibs/Comms/CommsWebSocket.cs:76-89`
    fn is_open(&self) -> bool {
        self.open
    }

    fn set_read_timeout(&mut self, timeout: Duration) -> io::Result<()> {
        self.timeout = timeout;
        match self.stream.as_ref() {
            Some(stream) => stream.set_read_timeout(Some(timeout)),
            None => Ok(()),
        }
    }

    /// `Close`: no more reconnecting, and the socket dropped without a close frame, as
    /// `ClientWebSocket.Dispose` aborts it.
    /// `// C#: ExtLibs/Comms/CommsWebSocket.cs:349-361`
    fn close(&mut self) {
        self.auto_reconnect = false;
        self.broken = false;
        self.drop_connection();
    }
}

enum ConnectError {
    Tls,
    Io(io::Error),
}

impl From<io::Error> for ConnectError {
    fn from(e: io::Error) -> Self {
        Self::Io(e)
    }
}

enum Head {
    Incomplete,
    Invalid,
    Complete(FrameHead, usize),
}

/// A frame header from the server, checked as `ClientWebSocket` checks it: no reserved bits (no
/// extension was negotiated), no mask (a server never masks), a known opcode, and a control frame
/// final and no longer than 125 bytes.
fn parse_head(bytes: &[u8]) -> Head {
    let (Some(&first), Some(&second)) = (bytes.first(), bytes.get(1)) else {
        return Head::Incomplete;
    };
    let fin = first & 0x80 != 0;
    let opcode = first & 0x0F;
    if first & 0x70 != 0 || second & 0x80 != 0 {
        return Head::Invalid;
    }
    let control = opcode & 0x08 != 0;
    if !matches!(
        opcode,
        OP_CONTINUATION | OP_TEXT | OP_BINARY | OP_CLOSE | OP_PING | OP_PONG
    ) {
        return Head::Invalid;
    }
    let (length, used) = match second & 0x7F {
        126 => match bytes.get(2..4) {
            Some(&[high, low]) => (u64::from(u16::from_be_bytes([high, low])), 4),
            _ => return Head::Incomplete,
        },
        127 => match bytes.get(2..10) {
            Some(eight) => (
                eight
                    .iter()
                    .fold(0u64, |acc, &byte| (acc << 8) | u64::from(byte)),
                10,
            ),
            None => return Head::Incomplete,
        },
        short => (u64::from(short), 2),
    };
    if control && (!fin || length > 125) {
        return Head::Invalid;
    }
    Head::Complete(
        FrameHead {
            fin,
            opcode,
            remaining: length,
        },
        used,
    )
}

/// Checks the server's answer to the opening handshake: `101`, `Upgrade: websocket`,
/// `Connection: Upgrade`, and the accept value the key calls for (RFC 6455 section 4.1).
fn check_handshake(head: &str, key: &str) -> io::Result<()> {
    let mut lines = head.split("\r\n");
    let status = lines.next().unwrap_or_default();
    let code = status.split(' ').nth(1).unwrap_or_default();
    let refuse = |why: String| io::Error::new(io::ErrorKind::ConnectionRefused, why);
    if code != "101" {
        return Err(refuse(format!(
            "the server answered the websocket handshake with {status:?}, not 101"
        )));
    }
    let header = |name: &str| {
        head.split("\r\n").skip(1).find_map(|line| {
            let (field, value) = line.split_once(':')?;
            field
                .trim()
                .eq_ignore_ascii_case(name)
                .then(|| value.trim().to_owned())
        })
    };
    let upgrade = header("Upgrade").unwrap_or_default();
    let connection = header("Connection").unwrap_or_default();
    if !upgrade.eq_ignore_ascii_case("websocket")
        || !connection
            .split(',')
            .any(|token| token.trim().eq_ignore_ascii_case("upgrade"))
    {
        return Err(refuse(
            "the server's handshake answer is not a websocket upgrade".to_owned(),
        ));
    }
    let expected = base64(&sha1(format!("{key}{ACCEPT_GUID}").as_bytes()));
    if header("Sec-WebSocket-Accept").as_deref() != Some(expected.as_str()) {
        return Err(refuse(
            "the server's Sec-WebSocket-Accept does not answer our key".to_owned(),
        ));
    }
    Ok(())
}

/// `Regex(@"""sid"":""([^""]+)""")` over the message decoded as ASCII, where a byte past ASCII is
/// a `?`.
fn session_id(text: &[u8]) -> Option<String> {
    let at = find(text, b"\"sid\":\"")? + 7;
    let rest = text.get(at..)?;
    let end = rest.iter().position(|&byte| byte == b'"')?;
    if end == 0 {
        return None;
    }
    Some(
        rest.get(..end)?
            .iter()
            .map(|&byte| {
                if byte.is_ascii() {
                    char::from(byte)
                } else {
                    '?'
                }
            })
            .collect(),
    )
}

fn find(haystack: &[u8], needle: &[u8]) -> Option<usize> {
    haystack
        .windows(needle.len())
        .position(|window| window == needle)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_session_id_is_what_the_csharps_regex_captures() {
        assert_eq!(
            session_id(br#"0{"sid":"abc","upgrades":[]}"#).as_deref(),
            Some("abc")
        );
        assert_eq!(session_id(br#"0{"sid":"","x":1}"#), None);
        assert_eq!(session_id(b"0{}"), None);
        assert_eq!(
            session_id(b"0{\"sid\":\"a\xC3\xA9\"}").as_deref(),
            Some("a??")
        );
    }

    #[test]
    fn a_server_frame_header_is_checked_as_client_websocket_checks_it() {
        assert!(matches!(
            parse_head(&[0x82, 0x04]),
            Head::Complete(
                FrameHead {
                    fin: true,
                    opcode: OP_BINARY,
                    remaining: 4
                },
                2
            )
        ));
        assert!(matches!(
            parse_head(&[0x82, 126, 0x01, 0x00]),
            Head::Complete(FrameHead { remaining: 256, .. }, 4)
        ));
        assert!(matches!(
            parse_head(&[0x82, 127, 0, 0, 0, 0, 0, 1, 0, 0]),
            Head::Complete(
                FrameHead {
                    remaining: 65_536,
                    ..
                },
                10
            )
        ));
        assert!(matches!(parse_head(&[0x82]), Head::Incomplete));
        assert!(matches!(parse_head(&[0x82, 126, 0x01]), Head::Incomplete));
        // Masked, reserved bits, unknown opcodes, a fragmented or long control frame.
        assert!(matches!(parse_head(&[0x82, 0x84]), Head::Invalid));
        assert!(matches!(parse_head(&[0xC2, 0x00]), Head::Invalid));
        assert!(matches!(parse_head(&[0x83, 0x00]), Head::Invalid));
        assert!(matches!(parse_head(&[0x09, 0x00]), Head::Invalid));
        assert!(matches!(parse_head(&[0x89, 126, 0, 126]), Head::Invalid));
    }

    #[test]
    fn a_handshake_answer_must_carry_the_right_accept() {
        let key = "dGhlIHNhbXBsZSBub25jZQ==";
        let good = "HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n\
                    Sec-WebSocket-Accept: s3pPLMBiTxaQ9kYGzzhZRbK+xOo=\r\n\r\n";
        assert!(check_handshake(good, key).is_ok());
        assert!(check_handshake(&good.replace("s3pP", "XXXX"), key).is_err());
        assert!(check_handshake(&good.replace("101", "200"), key).is_err());
        assert!(check_handshake(&good.replace("Upgrade: websocket", "Upgrade: h2c"), key).is_err());
    }
}
