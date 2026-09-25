//! The Linux source: V4L2 through the `v4l` crate's ioctls. What DirectShow is to the C#
//! (`WebCamService/Capture.cs:244-258`; `ConfigPlanner.cs:296-360`).

use std::path::Path;
use std::time::Duration;

use v4l::buffer::Type;
use v4l::io::traits::CaptureStream;
use v4l::video::Capture as _;
use v4l::{Format, FourCC};

use crate::{Device, Mode, PixelFormat, Source, Stream, VideoError};

/// How many buffers the memory-mapped stream queues: enough that a slow frame does not drop the
/// next, few enough that the latest frame is recent.
const BUFFERS: u32 = 4;

/// The V4L2 source.
#[derive(Debug, Clone, Copy, Default)]
pub struct V4l2Source;

/// Whether a node is a video capture device rather than a metadata or output node: a UVC webcam
/// registers one of each, and only the first can stream pictures.
fn is_capture(caps: &v4l::Capabilities) -> bool {
    use v4l::capability::Flags;
    caps.capabilities.contains(Flags::VIDEO_CAPTURE) && caps.capabilities.contains(Flags::STREAMING)
}

/// One node's device entry, if it captures video, named by its card and its node: two capture
/// nodes of one webcam share the card. The C# lists DirectShow's bare name; the node is added
/// by the owner's ruling (a written divergence, [`crate::node_name`]). The path stays the
/// device's identity.
/// `// C#: ExtLibs/WebCamService/Capture.cs:244-258`
fn device_of(path: &Path) -> Option<Device> {
    let dev = v4l::Device::with_path(path).ok()?;
    let caps = dev.query_caps().ok()?;
    is_capture(&caps).then(|| Device {
        path: path.to_path_buf(),
        name: crate::node_name(&caps.card, path),
    })
}

impl Source for V4l2Source {
    /// `getDevices`: the capture nodes, in `/dev/video*` order, by card name and node.
    /// `// C#: ExtLibs/WebCamService/Capture.cs:244-258`
    fn devices(&self) -> Vec<Device> {
        let mut nodes = v4l::context::enum_devices();
        nodes.sort_by_key(v4l::context::Node::index);
        nodes
            .iter()
            .filter_map(|node| device_of(node.path()))
            .collect()
    }

    /// The capture pin's stream capabilities, `GetStreamCaps(i)` for each `i`: one entry per
    /// format and discrete frame size, in the driver's order, as DirectShow gives a UVC camera's
    /// list. Only MJPEG and YUYV are listed - the two this crate decodes - and a stepwise size
    /// range (which a UVC camera does not have) is not. Each entry's rate is its fastest.
    /// `// C#: GCSViews/ConfigurationView/ConfigPlanner.cs:338-350`
    fn modes(&self, device: &Device) -> Result<Vec<Mode>, VideoError> {
        let dev = v4l::Device::with_path(&device.path)
            .map_err(|why| VideoError::Device(why.to_string()))?;
        let mut modes = Vec::new();
        for description in dev
            .enum_formats()
            .map_err(|why| VideoError::Device(why.to_string()))?
        {
            let Some(format) = PixelFormat::from_fourcc(description.fourcc.repr) else {
                continue;
            };
            let sizes = dev
                .enum_framesizes(description.fourcc)
                .map_err(|why| VideoError::Device(why.to_string()))?;
            for size in &sizes {
                let v4l::framesize::FrameSizeEnum::Discrete(discrete) = &size.size else {
                    continue;
                };
                let (width, height) = (discrete.width, discrete.height);
                let intervals = dev
                    .enum_frameintervals(description.fourcc, width, height)
                    .unwrap_or_default();
                let fastest = intervals
                    .iter()
                    .filter_map(|interval| match &interval.interval {
                        v4l::frameinterval::FrameIntervalEnum::Discrete(fraction)
                            if fraction.numerator > 0 && fraction.denominator > 0 =>
                        {
                            Some((fraction.numerator, fraction.denominator))
                        }
                        _ => None,
                    })
                    .min_by(|a, b| {
                        (u64::from(a.0) * u64::from(b.1)).cmp(&(u64::from(b.0) * u64::from(a.1)))
                    });
                modes.push(Mode {
                    format,
                    width,
                    height,
                    // A driver that will not say: the size without a rate, as 0 fps.
                    interval: fastest.unwrap_or((0, 1)),
                });
            }
        }
        Ok(modes)
    }

    /// `new Capture(index, media)` and `Start()`: the format set, the rate asked for, the
    /// buffers mapped; streaming begins on the first read.
    /// `// C#: ExtLibs/WebCamService/Capture.cs:77-111, 220-231`
    fn open(&self, device: &Device, mode: &Mode) -> Result<Box<dyn Stream>, VideoError> {
        let dev = v4l::Device::with_path(&device.path)
            .map_err(|why| VideoError::Device(why.to_string()))?;
        let wanted = Format::new(mode.width, mode.height, FourCC::new(&mode.format.fourcc()));
        let got = dev
            .set_format(&wanted)
            .map_err(|why| VideoError::Device(why.to_string()))?;
        if got.fourcc.repr != mode.format.fourcc() {
            return Err(VideoError::Unsupported(
                got.fourcc.str().unwrap_or("????").to_owned(),
            ));
        }
        if mode.interval.0 > 0 {
            let params = v4l::video::capture::Parameters::new(v4l::Fraction::new(
                mode.interval.0,
                mode.interval.1,
            ));
            // A driver that refuses the rate still streams at its own; not an error.
            let _ = dev.set_params(&params);
        }
        let mut stream = v4l::io::mmap::Stream::with_buffers(&dev, Type::VideoCapture, BUFFERS)
            .map_err(|why| VideoError::Device(why.to_string()))?;
        // A camera that stops sending must not hold the reading thread in `poll` for ever:
        // Stop joins that thread.
        stream.set_timeout(READ_TIMEOUT);
        Ok(Box::new(V4l2Stream {
            stream,
            format: mode.format,
            width: got.width,
            height: got.height,
            stride: got.stride,
        }))
    }
}

/// How long a read waits for the driver before it reports [`VideoError::TimedOut`] and the
/// reading thread looks again at whether it has been told to stop.
const READ_TIMEOUT: Duration = Duration::from_millis(500);

/// A memory-mapped capture stream. It holds the device's handle itself (an `Arc`), so the
/// `v4l::Device` it was made from can go.
struct V4l2Stream {
    stream: v4l::io::mmap::Stream<'static>,
    format: PixelFormat,
    /// The size the driver settled on, which need not be the size asked for.
    width: u32,
    height: u32,
    /// Bytes per row as the driver lays them out; YUYV rows may be padded past `width * 2`.
    stride: u32,
}

impl Stream for V4l2Stream {
    fn size(&self) -> (u32, u32) {
        (self.width, self.height)
    }

    fn next_raw(&mut self) -> Result<(Vec<u8>, u64), VideoError> {
        let (buffer, meta) = self.stream.next().map_err(|why| {
            if why.kind() == std::io::ErrorKind::TimedOut {
                VideoError::TimedOut
            } else {
                VideoError::Read(why.to_string())
            }
        })?;
        if meta.flags.contains(v4l::buffer::Flags::ERROR) {
            return Err(VideoError::BadFrame(
                "the driver flagged the buffer as corrupt".to_owned(),
            ));
        }
        let used = usize::try_from(meta.bytesused).unwrap_or(buffer.len());
        let bytes = buffer.get(..used).unwrap_or(buffer);
        let raw = match self.format {
            PixelFormat::Yuyv => unpad_rows(bytes, self.width, self.height, self.stride),
            PixelFormat::Mjpeg => bytes.to_vec(),
        };
        Ok((raw, u64::from(meta.sequence)))
    }
}

/// Packed rows of `width * 2` bytes out of rows `stride` bytes apart; the bytes as they are
/// when the rows are not padded.
fn unpad_rows(bytes: &[u8], width: u32, height: u32, stride: u32) -> Vec<u8> {
    let row = (width as usize) * 2;
    let stride = stride as usize;
    if stride <= row {
        return bytes.to_vec();
    }
    bytes
        .chunks(stride)
        .take(height as usize)
        .flat_map(|line| line.get(..row).unwrap_or(line))
        .copied()
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The machine's capture nodes, whatever they are: none is fine, but each one listed must
    /// have a path that exists and a name ending in that path that no other node has, and its
    /// modes must be decodable formats only.
    #[test]
    fn the_machines_capture_nodes_are_named_and_decodable() {
        let source = V4l2Source;
        let devices = source.devices();
        for device in &devices {
            assert!(device.path.exists(), "{device:?}");
            assert!(
                device
                    .name
                    .ends_with(&format!(" ({})", device.path.display())),
                "{device:?}"
            );
            let same = devices.iter().filter(|other| other.name == device.name);
            assert_eq!(same.count(), 1, "{device:?}");
            if let Ok(modes) = source.modes(device) {
                for mode in modes {
                    assert!(mode.width > 0 && mode.height > 0, "{mode:?}");
                    assert!(!mode.label().is_empty());
                }
            }
        }
    }

    /// Only a node that both captures video and streams counts; a metadata node does not.
    #[test]
    fn a_metadata_node_is_not_a_device() {
        use v4l::capability::Flags;
        let capture = v4l::Capabilities {
            driver: "uvcvideo".to_owned(),
            card: "cam".to_owned(),
            bus: "usb".to_owned(),
            version: (6, 8, 0),
            capabilities: Flags::VIDEO_CAPTURE | Flags::STREAMING,
        };
        assert!(is_capture(&capture));
        let meta = v4l::Capabilities {
            capabilities: Flags::META_CAPTURE | Flags::STREAMING,
            ..capture
        };
        assert!(!is_capture(&meta));
    }

    /// Two capture nodes of one webcam share a card name; the list tells them apart by the
    /// node, in brackets after the card.
    #[test]
    fn two_nodes_with_one_card_are_told_apart() {
        let card = "Integrated_Webcam_HD: Integrate";
        let first = crate::node_name(card, Path::new("/dev/video0"));
        let second = crate::node_name(card, Path::new("/dev/video2"));
        assert_eq!(first, "Integrated_Webcam_HD: Integrate (/dev/video0)");
        assert_eq!(second, "Integrated_Webcam_HD: Integrate (/dev/video2)");
    }

    /// Padded YUYV rows are packed; unpadded ones pass as they are.
    #[test]
    fn padded_rows_are_packed() {
        let padded = [1, 2, 3, 4, 0, 0, 5, 6, 7, 8, 0, 0];
        assert_eq!(unpad_rows(&padded, 2, 2, 6), [1, 2, 3, 4, 5, 6, 7, 8]);
        assert_eq!(unpad_rows(&padded, 3, 2, 6), padded);
    }

    /// This machine's webcam, for real: the capture nodes by name, the first one's modes, and
    /// ten frames decoded from it in its first mode. It takes the camera, so it runs only on
    /// request: `cargo test -p mp-video -- --ignored --nocapture`.
    #[test]
    #[ignore = "opens the machine's webcam"]
    #[allow(clippy::unwrap_used, clippy::expect_used)]
    fn the_webcam_lists_and_streams() {
        let source = V4l2Source;
        let devices = source.devices();
        for device in &devices {
            eprintln!("device {} {:?}", device.path.display(), device.name);
        }
        let first = devices.first().expect("a capture node");
        let modes = source.modes(first).unwrap();
        for mode in &modes {
            eprintln!("mode {}", mode.label());
        }
        let mode = modes.first().expect("a decodable mode");
        let started = std::time::Instant::now();
        let capture = crate::Capture::start(&source, first, mode).unwrap();
        while capture.frames() < 10 {
            assert!(
                started.elapsed() < std::time::Duration::from_secs(10),
                "{capture:?} {:?}",
                capture.error()
            );
            std::thread::sleep(std::time::Duration::from_millis(5));
        }
        let first_frames = started.elapsed();
        let counted = capture.frames();
        std::thread::sleep(std::time::Duration::from_secs(2));
        let rate = (capture.frames() - counted) as f64 / 2.0;
        let frame = capture.latest().unwrap();
        eprintln!(
            "{} frames of {} x {} in {first_frames:?}, then {rate:.1} fps; last error {:?}",
            capture.frames(),
            frame.width,
            frame.height,
            capture.error()
        );
        assert_eq!(
            frame.rgba.len(),
            frame.width as usize * frame.height as usize * 4
        );
        drop(capture);
    }
}
