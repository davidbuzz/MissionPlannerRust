//! The NTRIP client: RTK corrections from a caster.
//!
//! Ported from `ExtLibs/Comms/CommsNTRIP.cs` (`CommsNTRIP`). It asks a caster for a mount point
//! with an HTTP `GET` (NTRIP 2 by default, NTRIP 1 on request), sends the caster a GGA sentence of
//! the position it was given so a virtual reference station can be made for it, and then the
//! connection is a byte stream of RTCM - which [`Transport::read`] gives out. Injecting those
//! bytes into the vehicle as `GPS_RTCM_DATA` is the RTK page's job, not this one's.
//!
//! What goes on the wire - the URL's escaping, the request, the `Authorization` header, the GGA
//! sentence and its cadence - matches what the C# sent under mono, recorded in
//! `testdata/comms/golden/` by `tools/csharp-reference/regen-comms.sh`.
//!
//! # TLS
//!
//! The C# wraps the connection in TLS when the port is 443 or the URL is `https://`. Not ported:
//! it needs a TLS crate. Such a URL is [`OpenError::Unsupported`].

use std::io::{self, Read, Write};
use std::net::TcpStream;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use crate::codec::base64;
use crate::dotnet::{Uri, format_fixed, unescape_data_string};
use crate::socket::is_timeout;
use crate::{DEFAULT_READ_TIMEOUT, OpenError, Transport};

/// How often the position goes back to the caster.
/// `// C#: ExtLibs/Comms/CommsNTRIP.cs:411`
pub const GGA_INTERVAL: Duration = Duration::from_secs(30);

/// How many times a dropped connection is made again, over the transport's whole life.
/// `// C#: ExtLibs/Comms/CommsNTRIP.cs:34`
pub const RECONNECTS: u32 = 3;

/// What `CommsNTRIP` is told besides the URL: its public fields.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct NtripOptions {
    /// Latitude sent to the caster, degrees. With `lng`, 0 is no position, and no GGA is sent.
    pub lat: f64,
    /// Longitude sent to the caster, degrees.
    pub lng: f64,
    /// Altitude sent to the caster, metres.
    pub alt: f64,
    /// Ask in NTRIP 1's form (`HTTP/1.0`, no `Host` or `Ntrip-Version`) rather than NTRIP 2's.
    pub ntrip_v1: bool,
}

/// The wall clock, which the C# reads as `DateTime.Now` for both the sentence's time and its
/// 30-second gate. Replaceable, so a test can move it.
pub type Clock = Box<dyn FnMut() -> SystemTime + Send>;

/// Where to connect and what to ask, from the URL.
#[derive(Debug, Clone)]
struct Target {
    host: String,
    port: Option<u16>,
    scheme: String,
    path_and_query: String,
    /// The `Authorization` header line, or nothing when the URL has no user info.
    auth: String,
}

/// Mission Planner's NTRIP client.
pub struct NtripTransport {
    target: Target,
    ntrip_v1: bool,
    lat: f64,
    lng: f64,
    alt: f64,
    stream: Option<TcpStream>,
    /// `client.Client.Connected`.
    open: bool,
    /// `retrys`.
    retries: u32,
    /// `_lastnmea`: none is `DateTime.MinValue`.
    last_nmea: Option<SystemTime>,
    clock: Clock,
    timeout: Duration,
    description: String,
}

impl std::fmt::Debug for NtripTransport {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("NtripTransport")
            .field("description", &self.description)
            .field("open", &self.open)
            .field("retries", &self.retries)
            .finish_non_exhaustive()
    }
}

impl NtripTransport {
    /// `Open(url)`: connects to the caster and asks for the mount point.
    ///
    /// `url` is what the C#'s input box takes and saves as `NTRIP_url`:
    /// `ntrip://user:pass@host:port/mount`, or the same with `http://`. The request goes out, the
    /// caster's first line must say `200` and must not be a source table, and then - if a position
    /// is set - the first GGA sentence.
    /// `// C#: ExtLibs/Comms/CommsNTRIP.cs:137-155, 321-406`
    pub fn open(url: &str, options: NtripOptions) -> Result<Self, OpenError> {
        Self::open_with_clock(url, options, Box::new(SystemTime::now))
    }

    /// [`NtripTransport::open`] with the clock the GGA sentence's time and cadence are read from.
    pub fn open_with_clock(
        url: &str,
        options: NtripOptions,
        clock: Clock,
    ) -> Result<Self, OpenError> {
        let target = target(url)?;
        if target.port == Some(443) || target.scheme == "https" {
            // C#: CommsNTRIP.cs:343-348
            return Err(OpenError::Unsupported("TLS (NTRIP over https or port 443)"));
        }
        let description = format!(
            "ntrip:{}:{}{}",
            target.host,
            target
                .port
                .map_or_else(|| "-1".to_owned(), |p| p.to_string()),
            target.path_and_query
        );
        let mut transport = Self {
            target,
            ntrip_v1: options.ntrip_v1,
            lat: options.lat,
            lng: options.lng,
            alt: options.alt,
            stream: None,
            open: false,
            retries: RECONNECTS,
            last_nmea: None,
            clock,
            timeout: DEFAULT_READ_TIMEOUT,
            description,
        };
        transport
            .connect()
            .map_err(|e| OpenError::io(format!("opening {}", transport.description), e))?;
        Ok(transport)
    }

    /// Sets the position the GGA sentence reports: `lat`, `lng` and `alt`, which the C# exposes as
    /// fields the RTK page sets from the planned home.
    /// `// C#: ExtLibs/Comms/CommsNTRIP.cs:22, 28-29; GCSViews/ConfigurationView/ConfigSerialInjectGPS.cs:339-344`
    pub const fn set_position(&mut self, lat: f64, lng: f64, alt: f64) {
        self.lat = lat;
        self.lng = lng;
        self.alt = alt;
    }

    /// Reconnects left: [`RECONNECTS`], less one for each reconnect that succeeded.
    #[must_use]
    pub const fn reconnects_left(&self) -> u32 {
        self.retries
    }

    /// `doConnect`: the connection, the request, the caster's answer, and the first GGA.
    /// `// C#: ExtLibs/Comms/CommsNTRIP.cs:321-406`
    fn connect(&mut self) -> io::Result<()> {
        let port = self.target.port.ok_or_else(|| {
            // C#: `new TcpClient(host, -1)` throws for a scheme with no default port.
            io::Error::new(
                io::ErrorKind::InvalidInput,
                "Specified argument was out of the range of valid values.\nParameter name: port",
            )
        })?;
        let host = self
            .target
            .host
            .trim_start_matches('[')
            .trim_end_matches(']')
            .to_owned();
        // C#: CommsNTRIP.cs:334. The keep-alive the C# asks for next (10 hours idle, then every
        // 3 s) is a Windows-only IOControl that "fails under mono" and is caught; the standard
        // library has no call for it, so it is not made.
        let mut stream = TcpStream::connect((host.as_str(), port))?;

        // C#: CommsNTRIP.cs:355-376
        let request = if self.ntrip_v1 {
            format!(
                "GET {} HTTP/1.0\r\nUser-Agent: NTRIP MissionPlanner/1.0\r\n{}Connection: close\r\n\r\n",
                self.target.path_and_query, self.target.auth
            )
        } else {
            format!(
                "GET {} HTTP/1.1\r\nHost: {}:{port}\r\nNtrip-Version: Ntrip/2.0\r\n\
                 User-Agent: NTRIP MissionPlanner/1.0\r\n{}Connection: close\r\n\r\n",
                self.target.path_and_query, self.target.host, self.target.auth
            )
        };
        stream.write_all(request.as_bytes())?;

        // The caster's first line, waited for as long as it takes: the C#'s socket has no
        // receive timeout.
        stream.set_read_timeout(None)?;
        let line = read_line(&mut stream)?;

        // C#: CommsNTRIP.cs:382-389
        if !line.contains("200") {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!("Bad ntrip Response\n\n{line}"),
            ));
        }
        // C#: CommsNTRIP.cs:391-400. The C# logs the table (`sr.ReadToEnd()`) and has no other
        // use for it; this reads it to the caster's close, giving up after one read timeout of
        // silence rather than waiting forever, and drops it.
        if line.contains("SOURCETABLE") {
            stream.set_read_timeout(Some(self.timeout))?;
            let mut table = Vec::new();
            let _ = stream.read_to_end(&mut table);
            return Err(io::Error::new(
                io::ErrorKind::NotFound,
                format!("Got SOURCETABLE - Bad ntrip mount point\n\n{line}"),
            ));
        }

        stream.set_read_timeout(Some(self.timeout))?;
        self.stream = Some(stream);
        self.open = true;

        // C#: CommsNTRIP.cs:402-405 - "vrs may take up to 60+ seconds to respond"
        self.send_nmea()?;
        self.verify_connected()
    }

    /// `VerifyConnected`: on a closed link, reconnects - at most [`RECONNECTS`] times - and says it
    /// is closed even when the reconnect worked. The next call finds it open.
    ///
    /// A reconnect that fails is an error of its own and does not use up a retry, so a caster that
    /// is down is tried again on every read.
    /// `// C#: ExtLibs/Comms/CommsNTRIP.cs:460-483`
    fn verify_connected(&mut self) -> io::Result<()> {
        if self.open {
            return Ok(());
        }
        self.drop_connection();
        if self.retries > 0 {
            self.connect()?;
            self.retries -= 1;
        }
        Err(io::Error::new(
            io::ErrorKind::NotConnected,
            "The ntrip is closed",
        ))
    }

    fn drop_connection(&mut self) {
        if let Some(stream) = self.stream.take() {
            let _ = stream.shutdown(std::net::Shutdown::Both);
        }
        self.open = false;
    }

    /// `SendNMEA`: every 30 seconds, while there is a position, the GGA sentence to the caster.
    /// `// C#: ExtLibs/Comms/CommsNTRIP.cs:408-431`
    fn send_nmea(&mut self) -> io::Result<()> {
        if self.lat == 0.0 && self.lng == 0.0 {
            return Ok(());
        }
        let now = (self.clock)();
        // C#: `_lastnmea.AddSeconds(30) < DateTime.Now`, from `DateTime.MinValue`.
        let due = self
            .last_nmea
            .is_none_or(|last| last.checked_add(GGA_INTERVAL).is_some_and(|at| at < now));
        if !due {
            return Ok(());
        }
        let sentence = gga_sentence(self.lat, self.lng, self.alt, now);
        // C#: WriteLine adds "\r\n".
        self.write_line(&sentence)?;
        self.last_nmea = Some(now);
        Ok(())
    }

    /// `WriteLine` and `Write`: checked for a live connection first, and a failed write only
    /// marks the connection closed.
    /// `// C#: ExtLibs/Comms/CommsNTRIP.cs:209-233`
    fn write_line(&mut self, line: &str) -> io::Result<()> {
        let mut bytes = line.as_bytes().to_vec();
        bytes.extend_from_slice(b"\r\n");
        self.write_all(&bytes)
    }
}

impl Transport for NtripTransport {
    /// `Read`: checks the connection (reconnecting if it dropped), sends the GGA if it is due,
    /// then reads RTCM.
    ///
    /// The C# learns of a dropped connection only when a write fails (`Socket.Connected` does not
    /// notice the caster hanging up), so with no position to send it never reconnects, and with
    /// one it takes two GGA writes - up to a minute - to notice (golden/ntrip-reconnect.txt).
    /// Here the end of the stream marks it closed, so the next read does the reconnect
    /// `VerifyConnected` exists for.
    /// `// C#: ExtLibs/Comms/CommsNTRIP.cs:157-173`
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        self.verify_connected()?;
        self.send_nmea()?;
        if buf.is_empty() {
            return Ok(0);
        }
        let Some(stream) = self.stream.as_mut() else {
            self.open = false;
            return Ok(0);
        };
        match stream.read(buf) {
            // Diverges from the C#, which reads 0 here for ever with `IsOpen` still true (see
            // above): the caster has hung up, and saying so is what lets the reconnect happen.
            Ok(0) => {
                self.open = false;
                Ok(0)
            }
            Ok(n) => Ok(n),
            Err(e) if is_timeout(&e) => Ok(0),
            Err(_) => {
                self.open = false;
                // C#: CommsNTRIP.cs:169-172
                Err(io::Error::new(
                    io::ErrorKind::ConnectionAborted,
                    "ntrip Socket Closed",
                ))
            }
        }
    }

    /// `Write`: to the caster, after checking the connection; a failure is swallowed, and only
    /// marks the connection closed, as it makes `Socket.Connected` false in the C#.
    /// `// C#: ExtLibs/Comms/CommsNTRIP.cs:223-233`
    fn write_all(&mut self, buf: &[u8]) -> io::Result<()> {
        self.verify_connected()?;
        if let Some(stream) = self.stream.as_mut()
            && stream.write_all(buf).is_err()
        {
            self.open = false;
        }
        Ok(())
    }

    fn description(&self) -> &str {
        &self.description
    }

    /// `IsOpen`: `client.Client.Connected`.
    /// `// C#: ExtLibs/Comms/CommsNTRIP.cs:79-92`
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

    /// `Close`: the connection dropped. As in the C#, a read after it reconnects, while there are
    /// reconnects left.
    /// `// C#: ExtLibs/Comms/CommsNTRIP.cs:279-302`
    fn close(&mut self) {
        self.drop_connection();
    }
}

/// `Open(url)`'s preparation of the URL: `PercentEncode` all of it, escape the first `@` of
/// several, make `ntrip://` `http://`, and read it as a `System.Uri`; then `doConnect`'s
/// `Authorization` line from its user info.
/// `// C#: ExtLibs/Comms/CommsNTRIP.cs:137-155, 323-329`
fn target(url: &str) -> Result<Target, OpenError> {
    let ats = url.chars().filter(|&c| c == '@').count();
    let mut prepared = percent_encode(url);
    if ats > 1 {
        prepared = prepared.replacen('@', "%40", 1);
    }
    // String.Replace: every occurrence, and only in lower case.
    let prepared = prepared.replace("ntrip://", "http://");
    let uri = Uri::parse(&prepared)
        .map_err(|e| OpenError::io(format!("parsing {url}"), io::Error::other(e)))?;

    let auth = if uri.user_info.is_empty() {
        String::new()
    } else {
        // `new ASCIIEncoding().GetBytes(...)`: past ASCII is a `?`.
        let credentials: Vec<u8> = unescape_data_string(&uri.user_info)
            .chars()
            .map(|c| u8::try_from(c).ok().filter(u8::is_ascii).unwrap_or(b'?'))
            .collect();
        format!("Authorization: Basic {}\r\n", base64(&credentials))
    };
    Ok(Target {
        host: uri.host,
        port: uri.port,
        scheme: uri.scheme,
        path_and_query: uri.path_and_query,
        auth,
    })
}

/// Checks an NTRIP URL as [`NtripTransport::open`] would read it, without connecting.
pub(crate) fn check_url(url: &str) -> Result<(), String> {
    target(url).map(|_| ()).map_err(|e| match e {
        OpenError::Io { source, .. } => source.to_string(),
        other => other.to_string(),
    })
}

/// `CommsNTRIP.PercentEncode`: every character but ASCII letters, digits and `-._~@/:` becomes
/// `%XX` of its low byte - `(byte)c` of a UTF-16 unit, so `é` is `%E9`.
/// `// C#: ExtLibs/Comms/CommsNTRIP.cs:117-135`
#[must_use]
pub fn percent_encode(value: &str) -> String {
    let mut out = String::with_capacity(value.len());
    for unit in value.encode_utf16() {
        let kept = u8::try_from(unit).ok().filter(|&c| {
            c.is_ascii_alphanumeric() || matches!(c, b'-' | b'.' | b'_' | b'~' | b'@' | b'/' | b':')
        });
        match kept {
            Some(c) => out.push(char::from(c)),
            None => {
                let [_, low] = unit.to_be_bytes();
                out.push_str(&format!("%{low:02X}"));
            }
        }
    }
    out
}

/// The GGA sentence `SendNMEA` makes of a position at a moment, checksum included and line end
/// not: `$GPGGA,hhmmss.ss,ddmm.mm,N,dddmm.mm,E,1,10,1,alt,M,0,M,0.0,0*CS`.
///
/// As the C# builds it: degrees and minutes by `(int)lat + (lat - (int)lat) * .6f` - a
/// single-precision 0.6, widened - times 100; two decimals of a minute, rounded as .NET rounds;
/// fix quality 1, 10 satellites, HDOP 1, the altitude, a geoid separation of 0, and an empty-ish
/// differential age of `0.0` from station `0`. The time is UTC with its hundredths truncated.
/// `// C#: ExtLibs/Comms/CommsNTRIP.cs:408-431`
#[must_use]
pub fn gga_sentence(lat: f64, lng: f64, alt: f64, now: SystemTime) -> String {
    let widened = f64::from(0.6_f32);
    let latdms = lat.trunc() + (lat - lat.trunc()) * widened;
    let lngdms = lng.trunc() + (lng - lng.trunc()) * widened;

    let since_epoch = now.duration_since(UNIX_EPOCH).unwrap_or_default();
    let of_day = since_epoch.as_secs() % 86_400;
    let hundredths = since_epoch.subsec_millis() / 10;
    let time = format!(
        "{:02}{:02}{:02}.{hundredths:02}",
        of_day / 3600,
        of_day / 60 % 60,
        of_day % 60
    );

    let line = format!(
        "$GPGGA,{time},{},{},{},{},1,10,1,{},M,0,M,0.0,0",
        format_fixed((latdms * 100.0).abs(), 4, 2),
        if lat < 0.0 { "S" } else { "N" },
        format_fixed((lngdms * 100.0).abs(), 5, 2),
        if lng < 0.0 { "W" } else { "E" },
        format_fixed(alt, 1, 2),
    );
    let checksum = nmea_checksum(&line);
    format!("{line}*{checksum}")
}

/// `GetChecksum`: the exclusive-or of every character after the `$` and before any `*`, as two
/// upper-case hex digits.
/// `// C#: ExtLibs/Comms/CommsNTRIP.cs:434-458`
#[must_use]
pub fn nmea_checksum(sentence: &str) -> String {
    // The C# skips a `*` and carries on (`continue` in a switch), rather than stopping at it; a
    // sentence without its checksum yet has none, so the two agree.
    let sum = sentence
        .bytes()
        .filter(|&byte| byte != b'$' && byte != b'*')
        .fold(0u8, |sum, byte| sum ^ byte);
    format!("{sum:02X}")
}

/// `new StreamReader(st).ReadLine()`: the caster's first line.
///
/// A `StreamReader` fills a 1024-byte buffer from the socket and finds the line in it; whatever
/// else that read brought - an NTRIP 2 caster's headers, and any RTCM that arrived with them - stays
/// in the reader's buffer, and the C# never reads the reader again: from here on it reads the
/// socket itself. So what arrived with the line is dropped, exactly as here, and which bytes that
/// is depends on how the network split them, exactly as there. A `\r` ends a line as `\r\n` or
/// `\n` does; when it is the last byte read, the reader reads again to see whether `\n` follows.
/// `// C#: ExtLibs/Comms/CommsNTRIP.cs:353, 378`
fn read_line(stream: &mut TcpStream) -> io::Result<String> {
    // StreamReader.DefaultBufferSize
    let mut chunk = [0u8; 1024];
    let mut got = Vec::new();
    loop {
        let end = got.iter().position(|&byte| byte == b'\n' || byte == b'\r');
        let complete = match end {
            // A `\r` with nothing read after it yet.
            Some(at) if got.get(at) == Some(&b'\r') && at + 1 == got.len() => false,
            Some(_) => true,
            None => false,
        };
        if let (true, Some(at)) = (complete, end) {
            return Ok(String::from_utf8_lossy(got.get(..at).unwrap_or_default()).into_owned());
        }
        let n = stream.read(&mut chunk)?;
        if n == 0 {
            if let Some(at) = end {
                return Ok(String::from_utf8_lossy(got.get(..at).unwrap_or_default()).into_owned());
            }
            // ReadLine gives null, and the C# fails dereferencing it.
            return Err(io::Error::new(
                io::ErrorKind::UnexpectedEof,
                "Bad ntrip Response\n\nthe caster closed the connection without answering",
            ));
        }
        got.extend_from_slice(chunk.get(..n).unwrap_or_default());
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn percent_encoding_keeps_what_the_csharp_keeps() {
        assert_eq!(
            percent_encode("ntrip://user:p!ss@host:2101/Mount Point"),
            "ntrip://user:p%21ss@host:2101/Mount%20Point"
        );
        assert_eq!(percent_encode("café"), "caf%E9");
        assert_eq!(percent_encode("100%"), "100%25");
        assert_eq!(percent_encode("a?b=c&d#e"), "a%3Fb%3Dc%26d%23e");
        // A character past Latin-1 is its low byte: U+20AC is `%AC`.
        assert_eq!(percent_encode("\u{20AC}"), "%AC");
    }

    #[test]
    fn the_checksum_is_the_xor_between_dollar_and_star() {
        // The NMEA 0183 standard's own GGA example, whose checksum is 47.
        assert_eq!(
            nmea_checksum("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,"),
            "47"
        );
        assert_eq!(nmea_checksum("$A"), "41");
        assert_eq!(nmea_checksum("$AA"), "00");
    }

    #[test]
    fn a_url_without_user_info_sends_no_authorization() {
        let bare = target("ntrip://caster.example:2101/MOUNT").map_err(|e| e.to_string());
        assert_eq!(bare.as_ref().map(|t| t.auth.as_str()), Ok(""));
        let with = target("ntrip://user:pass@caster.example:2101/MOUNT").map_err(|e| e.to_string());
        assert_eq!(
            with.as_ref().map(|t| t.auth.as_str()),
            Ok("Authorization: Basic dXNlcjpwYXNz\r\n")
        );
        // Non-ASCII credentials go out as `?`, after `PercentEncode` has made them `%XX` that
        // are not UTF-8 and so stay escaped: `ü` is sent as `%FC`.
        let odd = target("ntrip://m\u{FC}:x@caster.example:2101/M").map_err(|e| e.to_string());
        assert_eq!(
            odd.as_ref().map(|t| t.auth.as_str()),
            Ok(format!("Authorization: Basic {}\r\n", base64(b"m%FC:x")).as_str())
        );
    }
}
