//! `CaptureMJPEG`: the HUD's MJPEG source, a multipart stream of JPEGs read over HTTP on a
//! thread of its own, each one the HUD's picture as it arrives.
//!
//! The C# (`ExtLibs/Utilities/CaptureMJPEG.cs`) asks for the URL with `HttpWebRequest`, finds
//! where the parts start, and reads part after part until told to stop, connecting again
//! whenever the stream ends or fails. Here the request is a plain HTTP/1.0 GET over a
//! `TcpStream` - the answer to one is never chunked - so an `https://` URL is not read; an
//! MJPEG server on a camera or a companion computer serves plain HTTP.

use std::io::{BufReader, Write as _};
use std::net::{Shutdown, TcpStream, ToSocketAddrs};
use std::sync::{Arc, Mutex, PoisonError};
use std::thread::JoinHandle;
use std::time::Duration;

use crate::{Feed, Frame, VideoError, convert, multipart};

/// `CaptureMJPEG.URL`'s first value, and Set MJPEG source's answer when `mjpeg_url` is not set.
/// `// C#: ExtLibs/Utilities/CaptureMJPEG.cs:20; GCSViews/FlightData.cs:4894-4896`
pub const DEFAULT_URL: &str = "http://127.0.0.1:56781/map.jpg";

/// How long a read waits: `ReadTimeout = 10000`.
/// `// C#: ExtLibs/Utilities/CaptureMJPEG.cs:127-128`
const READ_TIMEOUT: Duration = Duration::from_secs(10);

/// How long a connection is tried for.
const CONNECT_TIMEOUT: Duration = Duration::from_secs(10);

/// The pause before connecting again. The C# goes straight back to `start:`, which against a
/// refused connection spins a core; the pause keeps the retry and not the spin.
const RETRY_PAUSE: Duration = Duration::from_millis(200);

/// A running MJPEG capture: `CaptureMJPEG.runAsync()` until `Stop()`.
#[derive(Debug)]
pub struct CaptureMjpeg {
    feed: Arc<Feed>,
    socket: Arc<Mutex<Option<TcpStream>>>,
    thread: Option<JoinHandle<()>>,
    url: String,
}

impl CaptureMjpeg {
    /// `CaptureMJPEG.URL = url; CaptureMJPEG.runAsync()`: the "mjpg stream reader" thread
    /// started, connecting and reading until stopped.
    /// `// C#: ExtLibs/Utilities/CaptureMJPEG.cs:31-46`
    ///
    /// # Errors
    ///
    /// Only when the thread cannot be made.
    pub fn start(url: &str) -> Result<Self, VideoError> {
        let feed = Arc::new(Feed::default());
        let socket: Arc<Mutex<Option<TcpStream>>> = Arc::new(Mutex::new(None));
        let thread = {
            let feed = Arc::clone(&feed);
            let socket = Arc::clone(&socket);
            let url = url.to_owned();
            std::thread::Builder::new()
                .name("mjpg stream reader".to_owned())
                .spawn(move || get_url(&url, &feed, &socket))
                .map_err(|why| VideoError::Device(why.to_string()))?
        };
        Ok(Self {
            feed,
            socket,
            thread: Some(thread),
            url: url.to_owned(),
        })
    }

    /// `CaptureMJPEG.Stop()`: `running = false`, and the connection shut so a read waiting on
    /// it returns now rather than at its timeout. It does not wait for the thread, as the C#'s
    /// does not; the thread ends by itself.
    /// `// C#: ExtLibs/Utilities/CaptureMJPEG.cs:48-51`
    pub fn stop(&mut self) {
        self.feed.tell_stop();
        if let Some(socket) = self
            .socket
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .as_ref()
        {
            let _ = socket.shutdown(Shutdown::Both);
        }
        self.feed.clear();
        self.thread = None;
    }

    /// The latest frame, if one has arrived since the last connection.
    #[must_use]
    pub fn latest(&self) -> Option<Arc<Frame>> {
        self.feed.latest()
    }

    /// How many frames have arrived.
    #[must_use]
    pub fn frames(&self) -> u64 {
        self.feed.frames()
    }

    /// The last failure: the C# logs it.
    #[must_use]
    pub fn error(&self) -> Option<String> {
        self.feed.error()
    }

    /// Whether the thread is still going.
    #[must_use]
    pub fn is_running(&self) -> bool {
        self.thread
            .as_ref()
            .is_some_and(|thread| !thread.is_finished())
    }

    /// The URL being read.
    #[must_use]
    pub fn url(&self) -> &str {
        &self.url
    }
}

impl Drop for CaptureMjpeg {
    fn drop(&mut self) {
        self.stop();
    }
}

/// `getUrl`: connect, find the parts, read them until told to stop or the stream fails, clear
/// the picture, and go back to the start while still running.
/// `// C#: ExtLibs/Utilities/CaptureMJPEG.cs:82-218`
fn get_url(url: &str, feed: &Feed, socket: &Mutex<Option<TcpStream>>) {
    let mut sequence = 0u64;
    while !feed.stopping() {
        match connect(url) {
            Ok((stream, mut reader)) => {
                *socket.lock().unwrap_or_else(PoisonError::into_inner) = Some(stream);
                // Stopped while connecting: the shutdown missed this socket.
                if feed.stopping() {
                    break;
                }
                while !feed.stopping() {
                    match multipart::read_part(&mut reader) {
                        // A picture that will not decode is passed over: `catch { }`.
                        Ok(data) => {
                            if let Ok(frame) = convert::picture_to_frame(&data, sequence) {
                                sequence += 1;
                                feed.show(frame);
                            }
                        }
                        Err(why) => {
                            feed.note(why.to_string());
                            break;
                        }
                    }
                }
                // "clear last image"
                feed.clear();
            }
            Err(why) => feed.note(why),
        }
        if !feed.stopping() {
            std::thread::sleep(RETRY_PAUSE);
        }
    }
}

/// The request and its answer's headers, up to where the first part starts: the connection,
/// and a reader over it.
/// `// C#: ExtLibs/Utilities/CaptureMJPEG.cs:90-129`
fn connect(url: &str) -> Result<(TcpStream, BufReader<TcpStream>), String> {
    let (host, port, path) = parse_url(url)?;
    let address = (host.as_str(), port)
        .to_socket_addrs()
        .map_err(|why| why.to_string())?
        .next()
        .ok_or_else(|| format!("{host}: no address"))?;
    let mut stream =
        TcpStream::connect_timeout(&address, CONNECT_TIMEOUT).map_err(|why| why.to_string())?;
    stream
        .set_read_timeout(Some(READ_TIMEOUT))
        .map_err(|why| why.to_string())?;
    let request = format!("GET {path} HTTP/1.0\r\nHost: {host}:{port}\r\n\r\n");
    stream
        .write_all(request.as_bytes())
        .map_err(|why| why.to_string())?;
    let mut reader = BufReader::new(stream.try_clone().map_err(|why| why.to_string())?);
    let status = multipart::read_line(&mut reader).map_err(|why| why.to_string())?;
    // `GetResponse` throws for an answer that is not a success.
    let code = status.split_whitespace().nth(1).unwrap_or_default();
    if !code.starts_with('2') {
        return Err(format!("the server answered {status}"));
    }
    let headers = multipart::read_headers(&mut reader).map_err(|why| why.to_string())?;
    let content_type = headers
        .iter()
        .find(|(name, _)| name.eq_ignore_ascii_case("Content-Type"))
        .map(|(_, value)| value.clone())
        .ok_or("the answer has no Content-Type")?;
    if !content_type.contains("boundary=") {
        multipart::skip_to_boundary(&mut reader).map_err(|why| why.to_string())?;
    }
    Ok((stream, reader))
}

/// `http://host[:port]/path` in its three parts; port 80 when none is given.
fn parse_url(url: &str) -> Result<(String, u16, String), String> {
    let rest = url
        .trim()
        .strip_prefix("http://")
        .ok_or_else(|| format!("{url}: only an http:// URL is read"))?;
    let (authority, path) = rest
        .find('/')
        .map_or((rest, "/"), |slash| rest.split_at(slash));
    let (host, port) = match authority.rsplit_once(':') {
        Some((host, port)) => (
            host,
            port.parse::<u16>()
                .map_err(|_| format!("{url}: the port is not a number"))?,
        ),
        None => (authority, 80),
    };
    if host.is_empty() {
        return Err(format!("{url}: no host"));
    }
    Ok((host.to_owned(), port, path.to_owned()))
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

    use std::io::Read as _;
    use std::net::TcpListener;
    use std::time::Instant;

    use super::*;

    #[test]
    fn a_url_is_split_into_host_port_and_path() {
        assert_eq!(
            parse_url(DEFAULT_URL).unwrap(),
            ("127.0.0.1".to_owned(), 56781, "/map.jpg".to_owned())
        );
        assert_eq!(
            parse_url("http://cam").unwrap(),
            ("cam".to_owned(), 80, "/".to_owned())
        );
        assert!(parse_url("https://cam/x").is_err());
        assert!(parse_url("http://cam:port/x").is_err());
    }

    /// A JPEG of one colour.
    fn jpeg(width: u32, height: u32, rgb: [u8; 3]) -> Vec<u8> {
        let picture = image::RgbImage::from_pixel(width, height, image::Rgb(rgb));
        let mut bytes = std::io::Cursor::new(Vec::new());
        picture
            .write_to(&mut bytes, image::ImageFormat::Jpeg)
            .unwrap();
        bytes.into_inner()
    }

    /// Serves `frames` JPEGs to each of `connections` clients, as an MJPEG server does, then
    /// holds the connection open for `hold` and closes it.
    fn serve(
        connections: usize,
        frames: usize,
        hold: Duration,
    ) -> (u16, std::thread::JoinHandle<Vec<String>>) {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let port = listener.local_addr().unwrap().port();
        let server = std::thread::spawn(move || {
            let mut requests = Vec::new();
            for _ in 0..connections {
                let (mut client, _) = listener.accept().unwrap();
                let mut request = [0u8; 512];
                let read = client.read(&mut request).unwrap();
                requests.push(String::from_utf8_lossy(&request[..read]).into_owned());
                client
                    .write_all(
                        b"HTTP/1.0 200 OK\r\nContent-Type: multipart/x-mixed-replace;boundary=frame\r\n\r\n",
                    )
                    .unwrap();
                for _ in 0..frames {
                    let picture = jpeg(32, 16, [255, 0, 0]);
                    let head = format!(
                        "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: {}\r\n\r\n",
                        picture.len()
                    );
                    client.write_all(head.as_bytes()).unwrap();
                    client.write_all(&picture).unwrap();
                    client.write_all(b"\r\n").unwrap();
                }
                std::thread::sleep(hold);
            }
            requests
        });
        (port, server)
    }

    /// Frames arrive as RGBA at the JPEG's size; the stream's end clears the picture and the
    /// capture connects again, as the C#'s `goto start` does.
    #[test]
    fn frames_arrive_and_the_capture_connects_again_when_the_stream_ends() {
        let (port, server) = serve(2, 3, Duration::ZERO);
        let mut capture = CaptureMjpeg::start(&format!("http://127.0.0.1:{port}/video")).unwrap();
        let deadline = Instant::now() + Duration::from_secs(10);
        while capture.frames() < 6 {
            assert!(Instant::now() < deadline, "{capture:?}");
            std::thread::sleep(Duration::from_millis(5));
        }
        let requests = server.join().unwrap();
        assert_eq!(requests.len(), 2);
        assert!(
            requests[0].starts_with("GET /video HTTP/1.0\r\n"),
            "{}",
            requests[0]
        );
        assert!(capture.is_running());
        capture.stop();
        assert!(capture.latest().is_none());
    }

    /// The pixels are the JPEG's, near enough for a lossy picture.
    #[test]
    fn a_frame_is_the_pictures_colour() {
        let (port, _server) = serve(1, 1, Duration::from_secs(5));
        let capture = CaptureMjpeg::start(&format!("http://127.0.0.1:{port}/")).unwrap();
        let deadline = Instant::now() + Duration::from_secs(10);
        let frame = loop {
            if let Some(frame) = capture.latest() {
                break frame;
            }
            assert!(Instant::now() < deadline, "{capture:?}");
            std::thread::sleep(Duration::from_millis(2));
        };
        assert_eq!((frame.width, frame.height), (32, 16));
        let pixel = &frame.rgba[..4];
        assert!(
            pixel[0] > 240 && pixel[1] < 15 && pixel[2] < 15,
            "{pixel:?}"
        );
        assert_eq!(pixel[3], 255);
    }

    /// Nothing listening: the failure is kept, the thread tries again until stopped.
    #[test]
    fn a_refused_connection_is_noted_and_retried() {
        let port = {
            let listener = TcpListener::bind("127.0.0.1:0").unwrap();
            listener.local_addr().unwrap().port()
        };
        let mut capture = CaptureMjpeg::start(&format!("http://127.0.0.1:{port}/")).unwrap();
        let deadline = Instant::now() + Duration::from_secs(10);
        while capture.error().is_none() {
            assert!(Instant::now() < deadline);
            std::thread::sleep(Duration::from_millis(5));
        }
        assert!(capture.is_running());
        assert_eq!(capture.frames(), 0);
        capture.stop();
        assert!(!capture.is_running());
    }
}
