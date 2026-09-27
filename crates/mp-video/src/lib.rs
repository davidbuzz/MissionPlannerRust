//! Video capture for the HUD: what Mission Planner's `WebCamService.Capture` and the Planner
//! page's video controls do, on the platforms this application runs on.
//!
//! The C# (`ExtLibs/WebCamService/Capture.cs`; `GCSViews/ConfigurationView/ConfigPlanner.cs:
//! 261-370, 741-750`) lists the DirectShow video input devices by name, lists a device's capture
//! pin's stream capabilities as `GCSBitmapInfo`s - "`{Width} x {Height} {fps:0.00} fps
//! {Standard}`" - and Start builds a capture graph whose sample grabber hands each frame to the
//! HUD, which draws it under everything else (`ExtLibs/Controls/HUD.cs:1988-1996`). Stop
//! disposes the graph. Under mono the C# does none of it (`MainV2.MONO`).
//!
//! Here the [`Source`] trait is that list-and-open, the [`Capture`] is the running graph - a
//! thread that reads frames from a [`Stream`], turns them into RGBA and keeps the latest for the
//! HUD to draw - and the sources are:
//!
//! * [`v4l2::V4l2Source`] on Linux: the capture nodes under `/dev/video*` that say
//!   `V4L2_CAP_VIDEO_CAPTURE` (a webcam's metadata node does not), named from `VIDIOC_QUERYCAP`'s
//!   card name with the node after it ([`node_name`]: two nodes of one webcam share the card);
//!   a device's formats and frame sizes as the Video Format list, in MJPEG and YUYV,
//!   the two this crate decodes; frames from a memory-mapped stream.
//! * `mediafoundation::MediaFoundationSource` on Windows: Media Foundation's video capture
//!   devices by friendly name, as DirectShow's list names them; their native media types in
//!   MJPEG and YUY2 as the Video Format list; frames from an asynchronous Source Reader. The
//!   crate's one `unsafe` file, by the owner's ruling of 2026-09-27.
//! * [`testing::FakeSource`], scripted devices, formats and frames, for the tests and the
//!   screen's headless checks.
//!
//! The HUD menu's other sources hand the HUD frames the same way, each on a thread of its own
//! with the latest frame kept: [`gstreamer::GStreamer`] (Set GStreamer Source and HereLink
//! Video, `ExtLibs/Utilities/GStreamer.cs`) and [`mjpeg::CaptureMjpeg`] (Set MJPEG source,
//! `ExtLibs/Utilities/CaptureMJPEG.cs`).
//!
//! macOS has no source yet. The label a format gets is the C#'s, with the fourcc where
//! DirectShow had its analog video standard (`Standard`), which V4L2 and Media Foundation have
//! no counterpart for.

// No `unsafe` but in the Windows source's FFI, which allows it for its file alone.
#![deny(unsafe_code)]

use std::fmt;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex, PoisonError};
use std::thread::JoinHandle;

pub mod convert;
pub mod gstreamer;
#[cfg(windows)]
pub mod mediafoundation;
pub mod mjpeg;
pub mod multipart;
pub mod testing;
#[cfg(target_os = "linux")]
pub mod v4l2;
pub mod zip;

/// A video input device, as the Video Device list names it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Device {
    /// The node, `/dev/video0`: the device's identity, which opening it uses.
    pub path: PathBuf,
    /// What the Video Device list shows: DirectShow's `DsDevice.Name`; on Linux V4L2's card
    /// name with the node after it, [`node_name`]: "Integrated_Webcam_HD: Integrate
    /// (/dev/video0)".
    pub name: String,
}

/// A V4L2 node's name in the Video Device list: the card name `VIDIOC_QUERYCAP` gives, then the
/// node's path in brackets - "Integrated_Webcam_HD: Integrate (/dev/video0)".
///
/// Divergence, the owner's ruling of 2026-09-25 (PLAN.md §13.6 row 84): the C# lists
/// DirectShow's bare `DsDevice.Name` (`ExtLibs/WebCamService/Capture.cs:244-258`), but a UVC
/// webcam with a second sensor registers two capture nodes that share one card name, and the
/// list must tell them apart, so the node is added.
#[must_use]
pub fn node_name(card: &str, path: &std::path::Path) -> String {
    format!("{card} ({})", path.display())
}

/// The pixel formats this crate decodes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PixelFormat {
    /// Motion JPEG, one JPEG per frame.
    Mjpeg,
    /// Packed 4:2:2 YUV, two pixels in four bytes.
    Yuyv,
}

impl PixelFormat {
    /// The V4L2 fourcc.
    #[must_use]
    pub const fn fourcc(self) -> [u8; 4] {
        match self {
            Self::Mjpeg => *b"MJPG",
            Self::Yuyv => *b"YUYV",
        }
    }

    /// From a fourcc, for the two this crate decodes.
    #[must_use]
    pub fn from_fourcc(fourcc: [u8; 4]) -> Option<Self> {
        match &fourcc {
            b"MJPG" => Some(Self::Mjpeg),
            b"YUYV" => Some(Self::Yuyv),
            _ => None,
        }
    }
}

impl fmt::Display for PixelFormat {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let fourcc = self.fourcc();
        f.write_str(std::str::from_utf8(&fourcc).unwrap_or("????"))
    }
}

/// One entry of the Video Format list: `GCSBitmapInfo`.
/// `// C#: GCSViews/ConfigurationView/ConfigPlanner.cs:968-989`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Mode {
    /// The pixel format.
    pub format: PixelFormat,
    /// `Width`.
    pub width: u32,
    /// `Height`.
    pub height: u32,
    /// The frame interval Start asks for, seconds per frame as a fraction: V4L2's fastest for
    /// the format and size. The C# labels `MaxFrameInterval` (100 ns units), the slowest of the
    /// pin's range, and starts at the media type's own rate; see [`Mode::label`].
    pub interval: (u32, u32),
}

impl Mode {
    /// Frames per second: `10000000.0 / Fps` in the C#, the interval's reciprocal here.
    #[must_use]
    pub fn fps(&self) -> f64 {
        let (numerator, denominator) = self.interval;
        if numerator == 0 {
            return 0.0;
        }
        f64::from(denominator) / f64::from(numerator)
    }

    /// `GCSBitmapInfo.ToString`: `Width + " x " + Height + " {0:0.00} fps " + Standard`.
    /// Two divergences: the fourcc stands where DirectShow's analog video standard stood (V4L2
    /// has no counterpart for a webcam), and the rate is the one Start asks for rather than the
    /// C#'s `MaxFrameInterval`, which is the slowest the pin allows and not the rate it runs at.
    /// `// C#: GCSViews/ConfigurationView/ConfigPlanner.cs:985-988, 347-348`
    #[must_use]
    pub fn label(&self) -> String {
        format!(
            "{} x {} {:.2} fps {}",
            self.width,
            self.height,
            self.fps(),
            self.format
        )
    }
}

/// One frame, ready to draw.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Frame {
    /// Pixels across.
    pub width: u32,
    /// Pixels down.
    pub height: u32,
    /// RGBA, row-major, `width * height * 4` bytes.
    pub rgba: Vec<u8>,
    /// The driver's frame sequence number.
    pub sequence: u64,
}

impl Frame {
    /// The pixels as BGRA, the order a gpui `RenderImage` holds them in. Here rather than in the
    /// screen so it runs at this crate's optimisation level.
    #[must_use]
    pub fn to_bgra(&self) -> Vec<u8> {
        let mut bgra = self.rgba.clone();
        for pixel in bgra.chunks_exact_mut(4) {
            pixel.swap(0, 2);
        }
        bgra
    }
}

/// Where the capture fails. The text is the exception's `Message`: the screen puts the C#'s
/// "Camera Fail: " before it.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum VideoError {
    /// The device could not be opened or configured; the text is the driver's.
    #[error("{0}")]
    Device(String),
    /// A read failed.
    #[error("read failed: {0}")]
    Read(String),
    /// No frame came within the read's timeout; the reading thread reads again.
    #[error("no frame within the read timeout")]
    TimedOut,
    /// A frame's bytes were not the format the mode promised.
    #[error("bad frame: {0}")]
    BadFrame(String),
    /// The format is one this crate does not decode.
    #[error("unsupported pixel format {0}")]
    Unsupported(String),
    /// The GStreamer runtime would not run: the C#'s words for its `DllNotFoundException`.
    /// `// C#: ExtLibs/Utilities/GStreamer.cs:1206-1210`
    #[error("The file was not found at {launch}\nPlease verify permissions {why}")]
    NotFound {
        /// `GstLaunch`: the launcher's path.
        launch: String,
        /// Why it would not run, as the system says.
        why: String,
    },
}

/// A source of frames once a device is open: the capture graph's sample grabber.
pub trait Stream: Send {
    /// The size the device settled on, which a driver may make other than the size asked for;
    /// a YUYV frame is decoded at this size.
    fn size(&self) -> (u32, u32);
    /// The next frame's raw bytes and its sequence number, blocking until the driver has one
    /// or the stream's timeout ([`VideoError::TimedOut`]).
    fn next_raw(&mut self) -> Result<(Vec<u8>, u64), VideoError>;
}

/// Where devices come from: DirectShow in the C#, V4L2 here, a script in the tests.
pub trait Source: Send + Sync {
    /// `getDevices`: the video input devices, in the platform's order.
    fn devices(&self) -> Vec<Device>;
    /// A device's stream capabilities, as the Video Format list shows them.
    ///
    /// # Errors
    ///
    /// When the device will not open or say.
    fn modes(&self, device: &Device) -> Result<Vec<Mode>, VideoError>;
    /// Opens the device in a mode and starts streaming.
    ///
    /// # Errors
    ///
    /// When the device will not open, take the format or start.
    fn open(&self, device: &Device, mode: &Mode) -> Result<Box<dyn Stream>, VideoError>;
}

/// A running capture: `MainV2.cam`. Frames are read and decoded on a thread of their own; the
/// screen takes the latest each time it draws. Dropping it stops the thread, as `Dispose`
/// stops the graph.
/// `// C#: ExtLibs/WebCamService/Capture.cs:220-231; ConfigPlanner.cs:261-294`
pub struct Capture {
    latest: Arc<Mutex<Option<Arc<Frame>>>>,
    error: Arc<Mutex<Option<String>>>,
    frames: Arc<AtomicU64>,
    stop: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
    mode: Mode,
    device: Device,
}

impl fmt::Debug for Capture {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Capture")
            .field("device", &self.device)
            .field("mode", &self.mode)
            .field("frames", &self.frames())
            .finish_non_exhaustive()
    }
}

impl Capture {
    /// `new Capture(device, media)` then `Start()`: the device opened in the mode and the reading
    /// thread started.
    ///
    /// # Errors
    ///
    /// What the C#'s `catch` shows after "Camera Fail: ": the device would not open, take the
    /// format or start.
    pub fn start(source: &dyn Source, device: &Device, mode: &Mode) -> Result<Self, VideoError> {
        let mut stream = source.open(device, mode)?;
        let latest: Arc<Mutex<Option<Arc<Frame>>>> = Arc::new(Mutex::new(None));
        let error: Arc<Mutex<Option<String>>> = Arc::new(Mutex::new(None));
        let frames = Arc::new(AtomicU64::new(0));
        let stop = Arc::new(AtomicBool::new(false));
        let thread = {
            let latest = Arc::clone(&latest);
            let error = Arc::clone(&error);
            let frames = Arc::clone(&frames);
            let stop = Arc::clone(&stop);
            // A YUYV frame is laid out at the size the driver settled on, not the size asked.
            let (width, height) = stream.size();
            let decoded = Mode {
                width,
                height,
                ..*mode
            };
            let note = move |why: &VideoError| {
                *error.lock().unwrap_or_else(PoisonError::into_inner) = Some(why.to_string());
            };
            std::thread::Builder::new()
                .name("video-capture".to_owned())
                .spawn(move || {
                    while !stop.load(Ordering::Acquire) {
                        let decoded = stream
                            .next_raw()
                            .and_then(|(raw, sequence)| convert::decode(&decoded, &raw, sequence));
                        match decoded {
                            Ok(frame) => {
                                *latest.lock().unwrap_or_else(PoisonError::into_inner) =
                                    Some(Arc::new(frame));
                                frames.fetch_add(1, Ordering::Release);
                            }
                            // Nothing yet: look at `stop` and read again.
                            Err(VideoError::TimedOut) => {}
                            // A frame that will not decode is dropped and noted; the C#'s
                            // sample grabber would have thrown into the graph.
                            Err(why @ VideoError::BadFrame(_)) => note(&why),
                            // A dead device: stop rather than spin.
                            Err(why) => {
                                note(&why);
                                break;
                            }
                        }
                    }
                })
                .map_err(|why| VideoError::Device(why.to_string()))?
        };
        Ok(Self {
            latest,
            error,
            frames,
            stop,
            thread: Some(thread),
            mode: *mode,
            device: device.clone(),
        })
    }

    /// The latest frame decoded, if one has arrived.
    #[must_use]
    pub fn latest(&self) -> Option<Arc<Frame>> {
        self.latest
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .clone()
    }

    /// How many frames have been decoded.
    #[must_use]
    pub fn frames(&self) -> u64 {
        self.frames.load(Ordering::Acquire)
    }

    /// The last read or decode failure, if any.
    #[must_use]
    pub fn error(&self) -> Option<String> {
        self.error
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .clone()
    }

    /// Whether the reading thread is still going.
    #[must_use]
    pub fn is_running(&self) -> bool {
        self.thread.as_ref().is_some_and(|t| !t.is_finished())
    }

    /// The device being captured.
    #[must_use]
    pub const fn device(&self) -> &Device {
        &self.device
    }

    /// The mode it was opened in.
    #[must_use]
    pub const fn mode(&self) -> &Mode {
        &self.mode
    }
}

impl Drop for Capture {
    /// `Dispose`: the thread told to stop and joined.
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Release);
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
    }
}

/// What a stream's reading thread hands the screen - the C#'s `OnNewImage` event, kept as the
/// latest frame - and the flag that tells the thread to stop. [`gstreamer::GStreamer`] and
/// [`mjpeg::CaptureMjpeg`] each own one, shared with their thread.
#[derive(Debug, Default)]
pub(crate) struct Feed {
    latest: Mutex<Option<Arc<Frame>>>,
    frames: AtomicU64,
    error: Mutex<Option<String>>,
    stop: AtomicBool,
}

impl Feed {
    /// A new frame: `_onNewImage?.Invoke(null, image)`.
    pub(crate) fn show(&self, frame: Frame) {
        *self.latest.lock().unwrap_or_else(PoisonError::into_inner) = Some(Arc::new(frame));
        self.frames.fetch_add(1, Ordering::Release);
    }

    /// No picture: `_onNewImage?.Invoke(null, null)`, which clears the HUD's.
    pub(crate) fn clear(&self) {
        *self.latest.lock().unwrap_or_else(PoisonError::into_inner) = None;
    }

    /// What went wrong, kept for the screen: the C# logs it.
    pub(crate) fn note(&self, why: impl Into<String>) {
        *self.error.lock().unwrap_or_else(PoisonError::into_inner) = Some(why.into());
    }

    /// Whether the thread has been told to stop.
    pub(crate) fn stopping(&self) -> bool {
        self.stop.load(Ordering::Acquire)
    }

    /// Tells the thread to stop.
    pub(crate) fn tell_stop(&self) {
        self.stop.store(true, Ordering::Release);
    }

    /// The latest frame.
    pub(crate) fn latest(&self) -> Option<Arc<Frame>> {
        self.latest
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .clone()
    }

    /// How many frames have arrived.
    pub(crate) fn frames(&self) -> u64 {
        self.frames.load(Ordering::Acquire)
    }

    /// The last failure noted.
    pub(crate) fn error(&self) -> Option<String> {
        self.error
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .clone()
    }
}

/// The platform's source: V4L2 on Linux, Media Foundation on Windows, nothing elsewhere yet.
#[must_use]
pub fn platform_source() -> Option<Arc<dyn Source>> {
    #[cfg(target_os = "linux")]
    {
        Some(Arc::new(v4l2::V4l2Source))
    }
    #[cfg(windows)]
    {
        Some(Arc::new(mediafoundation::MediaFoundationSource))
    }
    #[cfg(not(any(target_os = "linux", windows)))]
    {
        None
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

    use super::testing::FakeSource;
    use super::*;

    #[test]
    fn a_mode_reads_as_the_csharps_bitmap_info() {
        let mode = Mode {
            format: PixelFormat::Mjpeg,
            width: 1280,
            height: 720,
            interval: (1, 30),
        };
        assert_eq!(mode.label(), "1280 x 720 30.00 fps MJPG");
        let odd = Mode {
            format: PixelFormat::Yuyv,
            width: 640,
            height: 480,
            interval: (1001, 30_000),
        };
        assert_eq!(odd.label(), "640 x 480 29.97 fps YUYV");
        assert_eq!(
            Mode {
                interval: (0, 30),
                ..odd
            }
            .fps(),
            0.0
        );
    }

    #[test]
    fn the_two_decodable_fourccs_round_trip() {
        assert_eq!(PixelFormat::from_fourcc(*b"MJPG"), Some(PixelFormat::Mjpeg));
        assert_eq!(PixelFormat::from_fourcc(*b"YUYV"), Some(PixelFormat::Yuyv));
        assert_eq!(PixelFormat::from_fourcc(*b"H264"), None);
        assert_eq!(PixelFormat::Mjpeg.to_string(), "MJPG");
    }

    /// Start reads and decodes on its thread; the screen sees the latest; Drop stops it.
    #[test]
    fn a_capture_keeps_the_latest_frame_and_stops_when_dropped() {
        let source = FakeSource::webcam();
        let device = source.devices()[0].clone();
        let mode = source.modes(&device).unwrap()[1];
        assert_eq!(mode.format, PixelFormat::Yuyv);
        let capture = Capture::start(&source, &device, &mode).unwrap();
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(5);
        while capture.frames() < 3 {
            assert!(
                std::time::Instant::now() < deadline,
                "no frames: {capture:?}"
            );
            std::thread::sleep(std::time::Duration::from_millis(2));
        }
        let frame = capture.latest().expect("a frame");
        assert_eq!((frame.width, frame.height), (mode.width, mode.height));
        assert_eq!(frame.rgba.len(), (mode.width * mode.height * 4) as usize);
        assert!(frame.sequence >= 2);
        assert_eq!(capture.error(), None);
        assert!(capture.is_running());
        drop(capture);
        assert!(source.streams_stopped() >= 1);
    }

    /// A device that will not open is the C#'s "Camera Fail", with the driver's words.
    #[test]
    fn a_device_that_will_not_open_fails_to_start() {
        let source = FakeSource::webcam().refusing_open("busy");
        let device = source.devices()[0].clone();
        let mode = source.modes(&device).unwrap()[0];
        let failed = Capture::start(&source, &device, &mode).err().unwrap();
        assert_eq!(failed.to_string(), "busy");
        assert_eq!(source.streams_stopped(), 0);
    }

    /// A timeout is not a failure and a corrupt frame is dropped: the thread goes on reading.
    #[test]
    fn timeouts_and_corrupt_frames_do_not_stop_the_capture() {
        let source = FakeSource::webcam().stalling_every(3).corrupting_every(4);
        let device = source.devices()[0].clone();
        let mode = source.modes(&device).unwrap()[1];
        let capture = Capture::start(&source, &device, &mode).unwrap();
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(5);
        while capture.frames() < 10 {
            assert!(std::time::Instant::now() < deadline, "{capture:?}");
            std::thread::sleep(std::time::Duration::from_millis(2));
        }
        assert!(capture.is_running());
        assert_eq!(
            capture.error().as_deref(),
            Some("bad frame: the driver flagged the buffer as corrupt")
        );
    }

    /// A driver that settles on another size: YUYV is decoded at the size it sends.
    #[test]
    fn a_yuyv_frame_is_decoded_at_the_size_the_driver_settled_on() {
        let source = FakeSource::webcam().settling_on(160, 120);
        let device = source.devices()[0].clone();
        let mode = source.modes(&device).unwrap()[1];
        let capture = Capture::start(&source, &device, &mode).unwrap();
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(5);
        while capture.latest().is_none() {
            assert!(std::time::Instant::now() < deadline, "{capture:?}");
            std::thread::sleep(std::time::Duration::from_millis(2));
        }
        let frame = capture.latest().unwrap();
        assert_eq!((frame.width, frame.height), (160, 120));
        assert_eq!(capture.error(), None);
    }

    /// gpui's images are BGRA.
    #[test]
    fn a_frame_swaps_to_bgra() {
        let frame = Frame {
            width: 2,
            height: 1,
            rgba: vec![1, 2, 3, 4, 5, 6, 7, 8],
            sequence: 0,
        };
        assert_eq!(frame.to_bgra(), [3, 2, 1, 4, 7, 6, 5, 8]);
    }

    /// A stream that dies ends the thread with the reason kept; the last good frame stays.
    #[test]
    fn a_dead_stream_stops_the_thread_and_keeps_the_reason() {
        let source = FakeSource::webcam().dying_after(2);
        let device = source.devices()[0].clone();
        let mode = source.modes(&device).unwrap()[1];
        let capture = Capture::start(&source, &device, &mode).unwrap();
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(5);
        while capture.is_running() {
            assert!(std::time::Instant::now() < deadline);
            std::thread::sleep(std::time::Duration::from_millis(2));
        }
        assert_eq!(capture.frames(), 2);
        assert!(capture.latest().is_some());
        assert_eq!(
            capture.error().as_deref(),
            Some("read failed: the device went away")
        );
    }
}
