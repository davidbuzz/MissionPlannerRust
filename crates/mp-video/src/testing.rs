//! A scripted [`Source`] for the tests and the screen's headless checks: devices, modes and
//! frames as a test lays them out, and a count of the streams stopped.

use std::path::PathBuf;
use std::sync::Arc;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::time::Duration;

use crate::{Device, Mode, PixelFormat, Source, Stream, VideoError};

/// The scripted source.
#[derive(Debug, Clone)]
pub struct FakeSource {
    devices: Vec<(Device, Vec<Mode>)>,
    refuse_open: Option<String>,
    refuse_modes: Option<String>,
    die_after: Option<u64>,
    stall_every: Option<u64>,
    corrupt_every: Option<u64>,
    settle_on: Option<(u32, u32)>,
    stopped: Arc<AtomicUsize>,
    /// How long a frame takes to "arrive".
    frame_time: Duration,
}

impl FakeSource {
    /// No devices at all.
    #[must_use]
    pub fn empty() -> Self {
        Self {
            devices: Vec::new(),
            refuse_open: None,
            refuse_modes: None,
            die_after: None,
            stall_every: None,
            corrupt_every: None,
            settle_on: None,
            stopped: Arc::new(AtomicUsize::new(0)),
            frame_time: Duration::from_millis(1),
        }
    }

    /// One webcam like this machine's: MJPEG 1280x720 at 30, YUYV 640x480 at 30 and 320x240 at
    /// 15.
    #[must_use]
    pub fn webcam() -> Self {
        Self::empty().with_device(
            "/dev/video0",
            "Integrated_Webcam_HD: Integrate",
            &[
                Mode {
                    format: PixelFormat::Mjpeg,
                    width: 1280,
                    height: 720,
                    interval: (1, 30),
                },
                Mode {
                    format: PixelFormat::Yuyv,
                    width: 640,
                    height: 480,
                    interval: (1, 30),
                },
                Mode {
                    format: PixelFormat::Yuyv,
                    width: 320,
                    height: 240,
                    interval: (1, 15),
                },
            ],
        )
    }

    /// Another device.
    #[must_use]
    pub fn with_device(mut self, path: &str, name: &str, modes: &[Mode]) -> Self {
        self.devices.push((
            Device {
                path: PathBuf::from(path),
                name: name.to_owned(),
            },
            modes.to_vec(),
        ));
        self
    }

    /// Every open fails with this reason.
    #[must_use]
    pub fn refusing_open(mut self, why: &str) -> Self {
        self.refuse_open = Some(why.to_owned());
        self
    }

    /// Every device refuses to list its modes, with this reason.
    #[must_use]
    pub fn refusing_modes(mut self, why: &str) -> Self {
        self.refuse_modes = Some(why.to_owned());
        self
    }

    /// The stream dies after this many frames.
    #[must_use]
    pub const fn dying_after(mut self, frames: u64) -> Self {
        self.die_after = Some(frames);
        self
    }

    /// Every `n`th read times out instead of giving a frame.
    #[must_use]
    pub const fn stalling_every(mut self, n: u64) -> Self {
        self.stall_every = Some(n);
        self
    }

    /// Every `n`th frame comes flagged corrupt.
    #[must_use]
    pub const fn corrupting_every(mut self, n: u64) -> Self {
        self.corrupt_every = Some(n);
        self
    }

    /// The driver settles on this size whatever size is asked for.
    #[must_use]
    pub const fn settling_on(mut self, width: u32, height: u32) -> Self {
        self.settle_on = Some((width, height));
        self
    }

    /// How many streams have been dropped.
    #[must_use]
    pub fn streams_stopped(&self) -> usize {
        self.stopped.load(Ordering::Acquire)
    }
}

impl Source for FakeSource {
    fn devices(&self) -> Vec<Device> {
        self.devices
            .iter()
            .map(|(device, _)| device.clone())
            .collect()
    }

    fn modes(&self, device: &Device) -> Result<Vec<Mode>, VideoError> {
        if let Some(why) = &self.refuse_modes {
            return Err(VideoError::Device(why.clone()));
        }
        self.devices
            .iter()
            .find(|(held, _)| held == device)
            .map(|(_, modes)| modes.clone())
            .ok_or_else(|| VideoError::Device(format!("{} is not a device", device.path.display())))
    }

    fn open(&self, device: &Device, mode: &Mode) -> Result<Box<dyn Stream>, VideoError> {
        if let Some(why) = &self.refuse_open {
            return Err(VideoError::Device(why.clone()));
        }
        let modes = self
            .devices
            .iter()
            .find(|(held, _)| held == device)
            .map(|(_, modes)| modes.clone())
            .unwrap_or_default();
        if !modes.contains(mode) {
            return Err(VideoError::Device(format!(
                "{} has no mode {}",
                device.name,
                mode.label()
            )));
        }
        let (width, height) = self.settle_on.unwrap_or((mode.width, mode.height));
        Ok(Box::new(FakeStream {
            mode: Mode {
                width,
                height,
                ..*mode
            },
            sequence: 0,
            reads: 0,
            die_after: self.die_after,
            stall_every: self.stall_every,
            corrupt_every: self.corrupt_every,
            stopped: Arc::clone(&self.stopped),
            frame_time: self.frame_time,
        }))
    }
}

/// Frames of a flat colour that changes with the sequence number, so a test can tell them apart.
struct FakeStream {
    mode: Mode,
    sequence: u64,
    reads: u64,
    die_after: Option<u64>,
    stall_every: Option<u64>,
    corrupt_every: Option<u64>,
    stopped: Arc<AtomicUsize>,
    frame_time: Duration,
}

/// Whether `count`, counted from one, is a multiple of `n`.
fn every(count: u64, n: Option<u64>) -> bool {
    n.is_some_and(|n| n > 0 && count.is_multiple_of(n))
}

impl Stream for FakeStream {
    fn size(&self) -> (u32, u32) {
        (self.mode.width, self.mode.height)
    }

    fn next_raw(&mut self) -> Result<(Vec<u8>, u64), VideoError> {
        if self.die_after.is_some_and(|after| self.sequence >= after) {
            return Err(VideoError::Read("the device went away".to_owned()));
        }
        std::thread::sleep(self.frame_time);
        self.reads += 1;
        if every(self.reads, self.stall_every) {
            return Err(VideoError::TimedOut);
        }
        let sequence = self.sequence;
        self.sequence += 1;
        if every(self.sequence, self.corrupt_every) {
            return Err(VideoError::BadFrame(
                "the driver flagged the buffer as corrupt".to_owned(),
            ));
        }
        let raw = match self.mode.format {
            PixelFormat::Yuyv => {
                let pixels = (self.mode.width as usize) * (self.mode.height as usize);
                #[allow(clippy::cast_possible_truncation)] // below 216
                let y = 16 + (sequence % 200) as u8;
                let mut raw = Vec::with_capacity(pixels * 2);
                for _ in 0..pixels / 2 {
                    raw.extend_from_slice(&[y, 128, y, 128]);
                }
                raw
            }
            PixelFormat::Mjpeg => {
                let mut picture = image::RgbImage::new(self.mode.width, self.mode.height);
                #[allow(clippy::cast_possible_truncation)] // below 256
                let shade = (sequence % 256) as u8;
                for pixel in picture.pixels_mut() {
                    *pixel = image::Rgb([shade, shade, shade]);
                }
                let mut jpeg = Vec::new();
                image::DynamicImage::ImageRgb8(picture)
                    .write_to(
                        &mut std::io::Cursor::new(&mut jpeg),
                        image::ImageFormat::Jpeg,
                    )
                    .map_err(|why| VideoError::Read(why.to_string()))?;
                jpeg
            }
        };
        Ok((raw, sequence))
    }
}

impl Drop for FakeStream {
    fn drop(&mut self) {
        self.stopped.fetch_add(1, Ordering::Release);
    }
}
