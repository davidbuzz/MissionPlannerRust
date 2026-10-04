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

//! The Windows source: Media Foundation's video capture devices, where the C# uses DirectShow's
//! (`ExtLibs/WebCamService/Capture.cs:244-258, 276-523`; `ConfigPlanner.cs:296-360`).
//!
//! Both APIs sit on the same kernel streaming devices, so the lists match: a device is
//! `MFEnumDeviceSources`' entry of the video capture type, named by its friendly name - the
//! `FriendlyName` that DirectShow's `DsDevice.Name` reads - with its symbolic link as its
//! identity; the Video Format list is the device's native media types in MJPEG and YUY2 (the
//! two this crate decodes; YUY2 is V4L2's YUYV byte for byte), one entry a format and size at its
//! fastest rate, as the V4L2 source lists them. A frame is a sample from a Source Reader set to
//! that media type, read asynchronously so that a camera that stops sending still lets the
//! reading thread look at whether it has been told to stop.
//!
//! Divergence: Media Foundation rather than DirectShow, whose Rust bindings would be the graph
//! builder, the sample grabber (gone from current SDKs) and a COM callback for each frame; the
//! owner's ruling of 2026-09-27 ("Camera capture on Windows ... Yes, unsafe in one file").
//!
//! FFI, so `unsafe`, which the workspace denies elsewhere: allowed for this file alone by that
//! ruling, each block with what makes it sound. Media Foundation's objects are free-threaded, so
//! the stream is read on the capture thread though it was opened on another.
#![allow(unsafe_code)]

use mp_os::Lock as _;
use mp_os::RecvTimeout as _;
use std::path::PathBuf;
use std::sync::Mutex;
use std::sync::mpsc::{self, Receiver, RecvTimeoutError, Sender};
use std::time::Duration;

use windows::Win32::Media::MediaFoundation::{
    IMFActivate, IMFAttributes, IMFMediaEvent, IMFMediaSource, IMFMediaType, IMFSample,
    IMFSourceReader, IMFSourceReaderCallback, IMFSourceReaderCallback_Impl,
    MF_DEVSOURCE_ATTRIBUTE_FRIENDLY_NAME, MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE,
    MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID,
    MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_SYMBOLIC_LINK, MF_E_NO_MORE_TYPES, MF_MT_FRAME_RATE,
    MF_MT_FRAME_SIZE, MF_MT_SUBTYPE, MF_SOURCE_READER_ALL_STREAMS, MF_SOURCE_READER_ASYNC_CALLBACK,
    MF_SOURCE_READER_FIRST_VIDEO_STREAM, MF_SOURCE_READERF_ENDOFSTREAM, MF_SOURCE_READERF_ERROR,
    MF_VERSION, MFCreateAttributes, MFCreateDeviceSource, MFCreateSourceReaderFromMediaSource,
    MFEnumDeviceSources, MFSTARTUP_FULL, MFShutdown, MFStartup, MFVideoFormat_MJPG,
    MFVideoFormat_YUY2,
};
use windows::Win32::System::Com::{
    COINIT_MULTITHREADED, CoInitializeEx, CoTaskMemFree, CoUninitialize,
};
use windows::core::{GUID, HRESULT, HSTRING, PWSTR, Ref, implement};

use crate::{Device, Mode, PixelFormat, Source, Stream, VideoError};

/// How long a read waits for a frame before it reports [`VideoError::TimedOut`] and the reading
/// thread looks again at whether it has been told to stop: the V4L2 source's.
const READ_TIMEOUT: Duration = Duration::from_millis(500);

/// The Source Reader's first video stream, as the `u32` its methods take.
#[allow(clippy::cast_sign_loss)]
const FIRST_VIDEO: u32 = MF_SOURCE_READER_FIRST_VIDEO_STREAM.0 as u32;
/// All the Source Reader's streams.
#[allow(clippy::cast_sign_loss)]
const ALL_STREAMS: u32 = MF_SOURCE_READER_ALL_STREAMS.0 as u32;

/// The Media Foundation source.
#[derive(Debug, Clone, Copy, Default)]
pub struct MediaFoundationSource;

/// COM on the calling thread for as long as this lives: joined to the multithreaded apartment,
/// unless the thread is already in the single-threaded one - gpui's window thread is, and Media
/// Foundation works from it as well - and then left as it was. Made and dropped within one
/// function, so on one thread.
struct Com {
    joined: bool,
}

impl Com {
    fn enter() -> Self {
        // SAFETY: no reserved pointer. `S_OK` and `S_FALSE` (already in the apartment) are each
        // balanced by `CoUninitialize` in `drop`; `RPC_E_CHANGED_MODE` (in the other one) is not.
        let result = unsafe { CoInitializeEx(None, COINIT_MULTITHREADED) };
        Self {
            joined: result.is_ok(),
        }
    }
}

impl Drop for Com {
    fn drop(&mut self) {
        if self.joined {
            // SAFETY: balances this value's own successful `CoInitializeEx`, on its thread.
            unsafe { CoUninitialize() };
        }
    }
}

/// Media Foundation started for as long as this lives; `MFStartup` counts its callers.
struct Platform;

impl Platform {
    fn start() -> Result<Self, VideoError> {
        // SAFETY: the version this SDK was built for; balanced by `MFShutdown` in `drop`.
        unsafe { MFStartup(MF_VERSION, MFSTARTUP_FULL) }.map_err(device_error)?;
        Ok(Self)
    }
}

impl Drop for Platform {
    fn drop(&mut self) {
        // SAFETY: balances this value's own `MFStartup`.
        let _ = unsafe { MFShutdown() };
    }
}

fn device_error(why: windows::core::Error) -> VideoError {
    VideoError::Device(why.message())
}

/// An attribute store asking for video capture devices, with `more` slots besides.
fn video_capture_attributes(more: u32) -> Result<IMFAttributes, VideoError> {
    let mut attributes = None;
    // SAFETY: an out pointer to a local `Option`.
    unsafe { MFCreateAttributes(&raw mut attributes, 1 + more) }.map_err(device_error)?;
    let attributes = attributes.ok_or_else(|| VideoError::Device("no attribute store".into()))?;
    // SAFETY: two GUIDs from the SDK, by reference.
    unsafe {
        attributes.SetGUID(
            &MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE,
            &MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID,
        )
    }
    .map_err(device_error)?;
    Ok(attributes)
}

/// A string attribute of a device, the memory Media Foundation allocated for it freed.
fn string_of(activate: &IMFActivate, key: &GUID) -> Option<String> {
    let mut text = PWSTR::null();
    let mut length = 0u32;
    // SAFETY: out pointers to locals; the string is the callee's CoTaskMem allocation.
    unsafe { activate.GetAllocatedString(key, &raw mut text, &raw mut length) }.ok()?;
    // SAFETY: a NUL-terminated wide string the call just returned.
    let value = unsafe { text.to_string() }.ok();
    // SAFETY: freed once, as `GetAllocatedString` documents.
    unsafe { CoTaskMemFree(Some(text.0.cast_const().cast())) };
    value
}

/// The video capture devices' activation objects, in Media Foundation's order.
fn activations() -> Result<Vec<IMFActivate>, VideoError> {
    let attributes = video_capture_attributes(0)?;
    let mut array: *mut Option<IMFActivate> = std::ptr::null_mut();
    let mut count = 0u32;
    // SAFETY: out pointers to locals; the array is the callee's CoTaskMem allocation.
    unsafe { MFEnumDeviceSources(&attributes, &raw mut array, &raw mut count) }
        .map_err(device_error)?;
    if array.is_null() {
        return Ok(Vec::new());
    }
    // SAFETY: `count` entries at `array`, each a reference the caller now owns: each is moved out
    // (so released when the `Vec` drops) before the array itself is freed.
    let taken = unsafe { std::slice::from_raw_parts_mut(array, count as usize) }
        .iter_mut()
        .filter_map(Option::take)
        .collect();
    // SAFETY: the array `MFEnumDeviceSources` allocated, its entries moved out, freed once.
    unsafe { CoTaskMemFree(Some(array.cast_const().cast())) };
    Ok(taken)
}

/// One device's entry: its friendly name, `DsDevice.Name`, and its symbolic link, which opening
/// it uses.
fn device_of(activate: &IMFActivate) -> Option<Device> {
    let name = string_of(activate, &MF_DEVSOURCE_ATTRIBUTE_FRIENDLY_NAME)?;
    let link = string_of(
        activate,
        &MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_SYMBOLIC_LINK,
    )?;
    Some(Device {
        path: PathBuf::from(link),
        name,
    })
}

/// The device a symbolic link names, opened as a media source.
fn media_source(device: &Device) -> Result<IMFMediaSource, VideoError> {
    let attributes = video_capture_attributes(1)?;
    let link = HSTRING::from(device.path.as_os_str());
    // SAFETY: a GUID from the SDK and a NUL-terminated string that outlives the call.
    unsafe {
        attributes.SetString(
            &MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_SYMBOLIC_LINK,
            &link,
        )
    }
    .map_err(device_error)?;
    // SAFETY: the attribute store just made. A link to no device fails here.
    unsafe { MFCreateDeviceSource(&attributes) }.map_err(|why| {
        // `new Capture(iDeviceNum, media)` for a device that is not there.
        // `// C#: ExtLibs/WebCamService/Capture.cs:84-87`
        VideoError::Device(format!(
            "No video capture devices found at that index! ({})",
            why.message()
        ))
    })
}

/// A media type's pixel format, size and rate, as far as this crate decodes it: the subtype
/// GUID, `MF_MT_FRAME_SIZE` (width high, height low) and `MF_MT_FRAME_RATE` (frames per second
/// as numerator high, denominator low).
fn mode_of(subtype: GUID, size: u64, rate: Option<u64>) -> Option<Mode> {
    let format = if subtype == MFVideoFormat_MJPG {
        PixelFormat::Mjpeg
    } else if subtype == MFVideoFormat_YUY2 {
        PixelFormat::Yuyv
    } else {
        return None;
    };
    let (width, height) = halves(size);
    if width == 0 || height == 0 {
        return None;
    }
    // Frames a second, numerator over denominator, is a frame interval of denominator over
    // numerator; a type that states no rate is the size without one, as 0 fps.
    let interval = match rate.map(halves) {
        Some((frames, seconds)) if frames > 0 && seconds > 0 => (seconds, frames),
        _ => (0, 1),
    };
    Some(Mode {
        format,
        width,
        height,
        interval,
    })
}

/// A packed `UINT64` attribute's two halves, high then low.
fn halves(packed: u64) -> (u32, u32) {
    let high = u32::try_from(packed >> 32).unwrap_or(u32::MAX);
    let low = u32::try_from(packed & u64::from(u32::MAX)).unwrap_or(u32::MAX);
    (high, low)
}

/// Whether `a`'s interval is shorter than `b`'s: the faster rate.
fn faster(a: (u32, u32), b: (u32, u32)) -> bool {
    if b.0 == 0 {
        return a.0 > 0;
    }
    a.0 > 0 && u64::from(a.0) * u64::from(b.1) < u64::from(b.0) * u64::from(a.1)
}

/// The Video Format list from the native media types in the device's order: one entry for each
/// format and size, where the first type of it stood, at the fastest rate any type of it has -
/// Media Foundation lists each rate a camera offers as a type of its own, DirectShow and V4L2
/// one entry with a range.
fn fold_modes(types: impl IntoIterator<Item = Mode>) -> Vec<Mode> {
    let mut modes: Vec<Mode> = Vec::new();
    for mode in types {
        match modes.iter_mut().find(|seen| {
            seen.format == mode.format && seen.width == mode.width && seen.height == mode.height
        }) {
            Some(seen) if faster(mode.interval, seen.interval) => seen.interval = mode.interval,
            Some(_) => {}
            None => modes.push(mode),
        }
    }
    modes
}

/// The reader's native media types for its first video stream, each with the mode it is.
fn native_types(reader: &IMFSourceReader) -> Result<Vec<(IMFMediaType, Mode)>, VideoError> {
    let mut types = Vec::new();
    for index in 0.. {
        // SAFETY: a stream constant and an index; past the last type the call says so.
        let media_type = match unsafe { reader.GetNativeMediaType(FIRST_VIDEO, index) } {
            Ok(media_type) => media_type,
            Err(why) if why.code() == MF_E_NO_MORE_TYPES => break,
            Err(why) => return Err(device_error(why)),
        };
        // SAFETY: three attribute reads by GUID from the SDK.
        let (subtype, size, rate) = unsafe {
            (
                media_type.GetGUID(&MF_MT_SUBTYPE),
                media_type.GetUINT64(&MF_MT_FRAME_SIZE),
                media_type.GetUINT64(&MF_MT_FRAME_RATE).ok(),
            )
        };
        let (Ok(subtype), Ok(size)) = (subtype, size) else {
            continue;
        };
        if let Some(mode) = mode_of(subtype, size, rate) {
            types.push((media_type, mode));
        }
    }
    Ok(types)
}

/// A synchronous Source Reader on a device, for listing its types. Released, it shuts the device
/// down (the reader's default), so the reader that streams lists its own.
fn listing_reader(source: &IMFMediaSource) -> Result<IMFSourceReader, VideoError> {
    // SAFETY: the media source just opened; no attributes.
    unsafe { MFCreateSourceReaderFromMediaSource(source, None) }.map_err(device_error)
}

/// Shuts a media source down, which lets the camera go; a source not shut down keeps it.
fn shut(source: &IMFMediaSource) {
    // SAFETY: a media source this file opened, shut once.
    let _ = unsafe { source.Shutdown() };
}

impl Source for MediaFoundationSource {
    /// `getDevices`: the video capture devices by friendly name, in Media Foundation's order.
    /// `// C#: ExtLibs/WebCamService/Capture.cs:244-258`
    fn devices(&self) -> Vec<Device> {
        let _com = Com::enter();
        let Ok(_platform) = Platform::start() else {
            return Vec::new();
        };
        activations()
            .unwrap_or_default()
            .iter()
            .filter_map(device_of)
            .collect()
    }

    /// The device's stream capabilities, `GetStreamCaps(i)` for each `i`: its native media types
    /// in MJPEG and YUY2, one entry a format and size, at its fastest rate.
    /// `// C#: GCSViews/ConfigurationView/ConfigPlanner.cs:338-350`
    fn modes(&self, device: &Device) -> Result<Vec<Mode>, VideoError> {
        let _com = Com::enter();
        let _platform = Platform::start()?;
        let source = media_source(device)?;
        let listed = listing_reader(&source).and_then(|reader| native_types(&reader));
        shut(&source);
        Ok(fold_modes(listed?.into_iter().map(|(_, mode)| mode)))
    }

    /// `new Capture(index, media)` and `Start()`: the device opened, the native type of the
    /// mode's format and size at the mode's rate (or the nearest the device has) set on its
    /// first video stream, and the first frame asked for.
    /// `// C#: ExtLibs/WebCamService/Capture.cs:77-111, 276-523`
    fn open(&self, device: &Device, mode: &Mode) -> Result<Box<dyn Stream>, VideoError> {
        let _com = Com::enter();
        let platform = Platform::start()?;
        let source = media_source(device)?;
        match open_reader(&source, mode) {
            Ok(Opened {
                reader,
                arrivals,
                size,
            }) => Ok(Box::new(MediaFoundationStream {
                reader,
                source,
                arrivals,
                pending: false,
                size,
                sequence: 0,
                _platform: platform,
            })),
            Err(why) => {
                shut(&source);
                Err(why)
            }
        }
    }
}

/// The native type for a mode: its format and size, at its rate if the device has it, else the
/// fastest of them.
fn pick(types: Vec<(IMFMediaType, Mode)>, mode: &Mode) -> Option<(IMFMediaType, Mode)> {
    let mut best: Option<(IMFMediaType, Mode)> = None;
    for (media_type, found) in types {
        if found.format != mode.format || found.width != mode.width || found.height != mode.height {
            continue;
        }
        if found.interval == mode.interval {
            return Some((media_type, found));
        }
        if best
            .as_ref()
            .is_none_or(|(_, kept)| faster(found.interval, kept.interval))
        {
            best = Some((media_type, found));
        }
    }
    best
}

/// A reader set to a mode, what its callback sends, and the size it settled on.
struct Opened {
    reader: IMFSourceReader,
    arrivals: Receiver<Arrival>,
    size: (u32, u32),
}

/// An asynchronous Source Reader on the device, set to the mode.
fn open_reader(source: &IMFMediaSource, mode: &Mode) -> Result<Opened, VideoError> {
    let (sender, arrivals) = mpsc::channel();
    let callback: IMFSourceReaderCallback = Arrivals {
        sender: Mutex::new(sender),
    }
    .into();
    let mut attributes = None;
    // SAFETY: an out pointer to a local `Option`.
    unsafe { MFCreateAttributes(&raw mut attributes, 1) }.map_err(device_error)?;
    let attributes = attributes.ok_or_else(|| VideoError::Device("no attribute store".into()))?;
    // SAFETY: the callback object, which the reader keeps a reference to.
    unsafe { attributes.SetUnknown(&MF_SOURCE_READER_ASYNC_CALLBACK, &callback) }
        .map_err(device_error)?;
    // SAFETY: the media source and the attribute store just made.
    let reader = unsafe { MFCreateSourceReaderFromMediaSource(source, &attributes) }
        .map_err(device_error)?;
    let (media_type, found) =
        pick(native_types(&reader)?, mode).ok_or_else(|| VideoError::Unsupported(mode.label()))?;
    // SAFETY: stream constants and a native type of this device's.
    unsafe {
        reader
            .SetStreamSelection(ALL_STREAMS, false)
            .and_then(|()| reader.SetStreamSelection(FIRST_VIDEO, true))
            .and_then(|()| reader.SetCurrentMediaType(FIRST_VIDEO, None, &media_type))
    }
    .map_err(device_error)?;
    Ok(Opened {
        reader,
        arrivals,
        size: (found.width, found.height),
    })
}

/// What the reader's callback hands the reading thread: a frame's bytes, or why there is none.
enum Arrival {
    /// A sample's bytes, contiguous.
    Frame(Vec<u8>),
    /// A read that finished with no sample: a gap in the stream.
    Nothing,
    /// The stream failed or ended.
    Failed(VideoError),
}

/// `IMFSourceReaderCallback`: each finished read's sample copied out and sent to the reading
/// thread, so no COM object crosses the channel.
#[implement(IMFSourceReaderCallback)]
struct Arrivals {
    sender: Mutex<Sender<Arrival>>,
}

impl Arrivals {
    fn send(&self, arrival: Arrival) {
        // A stream already dropped has no receiver; the arrival is not wanted.
        let _ = self
            .sender
            .os_lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .send(arrival);
    }
}

/// A sample's bytes: its buffers made one and copied.
fn bytes_of(sample: &IMFSample) -> windows::core::Result<Vec<u8>> {
    // SAFETY: a sample the reader handed the callback.
    let buffer = unsafe { sample.ConvertToContiguousBuffer() }?;
    let mut data: *mut u8 = std::ptr::null_mut();
    let mut length = 0u32;
    // SAFETY: out pointers to locals; unlocked below before the buffer goes.
    unsafe { buffer.Lock(&raw mut data, None, Some(&raw mut length)) }?;
    let bytes = if data.is_null() {
        Vec::new()
    } else {
        // SAFETY: `length` valid bytes at `data` while the buffer is locked.
        unsafe { std::slice::from_raw_parts(data, length as usize) }.to_vec()
    };
    // SAFETY: the lock taken above.
    unsafe { buffer.Unlock() }?;
    Ok(bytes)
}

impl IMFSourceReaderCallback_Impl for Arrivals_Impl {
    fn OnReadSample(
        &self,
        hrstatus: HRESULT,
        _dwstreamindex: u32,
        dwstreamflags: u32,
        _lltimestamp: i64,
        psample: Ref<IMFSample>,
    ) -> windows::core::Result<()> {
        #[allow(clippy::cast_sign_loss)]
        let failed = dwstreamflags & (MF_SOURCE_READERF_ERROR.0 as u32) != 0;
        #[allow(clippy::cast_sign_loss)]
        let ended = dwstreamflags & (MF_SOURCE_READERF_ENDOFSTREAM.0 as u32) != 0;
        let arrival = if hrstatus.is_err() {
            Arrival::Failed(VideoError::Read(
                windows::core::Error::from_hresult(hrstatus).message(),
            ))
        } else if failed {
            Arrival::Failed(VideoError::Read("the device reported an error".to_owned()))
        } else if ended {
            Arrival::Failed(VideoError::Read("the stream ended".to_owned()))
        } else if let Some(sample) = psample.as_ref() {
            match bytes_of(sample) {
                Ok(bytes) => Arrival::Frame(bytes),
                Err(why) => Arrival::Failed(VideoError::BadFrame(why.message())),
            }
        } else {
            Arrival::Nothing
        };
        self.send(arrival);
        Ok(())
    }

    fn OnFlush(&self, _dwstreamindex: u32) -> windows::core::Result<()> {
        Ok(())
    }

    fn OnEvent(
        &self,
        _dwstreamindex: u32,
        _pevent: Ref<IMFMediaEvent>,
    ) -> windows::core::Result<()> {
        Ok(())
    }
}

/// A device streaming through an asynchronous Source Reader: one read asked for at a time, its
/// sample waited for up to [`READ_TIMEOUT`].
struct MediaFoundationStream {
    reader: IMFSourceReader,
    source: IMFMediaSource,
    arrivals: Receiver<Arrival>,
    /// Whether a read has been asked for and has not finished.
    pending: bool,
    size: (u32, u32),
    /// Frames read: Media Foundation's samples carry a time, not a number.
    sequence: u64,
    /// Last, so Media Foundation stays started until the reader and source have gone.
    _platform: Platform,
}

// SAFETY: Media Foundation's objects are free-threaded - the Source Reader and a device's media
// source are called from any thread, and its asynchronous callbacks come on its own work-queue
// threads - so the stream may be opened on one thread and read on the capture thread. Only one
// thread holds it at a time.
unsafe impl Send for MediaFoundationStream {}

impl Stream for MediaFoundationStream {
    fn size(&self) -> (u32, u32) {
        self.size
    }

    fn next_raw(&mut self) -> Result<(Vec<u8>, u64), VideoError> {
        if !self.pending {
            // SAFETY: the stream's first video stream, no flags, and no out pointers: in
            // asynchronous mode the sample comes to the callback.
            unsafe {
                self.reader
                    .ReadSample(FIRST_VIDEO, 0, None, None, None, None)
            }
            .map_err(|why| VideoError::Read(why.message()))?;
            self.pending = true;
        }
        match self.arrivals.os_recv_timeout(READ_TIMEOUT) {
            Ok(arrival) => {
                self.pending = false;
                match arrival {
                    Arrival::Frame(bytes) => {
                        self.sequence += 1;
                        Ok((bytes, self.sequence))
                    }
                    Arrival::Nothing => Err(VideoError::TimedOut),
                    Arrival::Failed(why) => Err(why),
                }
            }
            Err(RecvTimeoutError::Timeout) => Err(VideoError::TimedOut),
            Err(RecvTimeoutError::Disconnected) => {
                Err(VideoError::Read("the reader's callback went".to_owned()))
            }
        }
    }
}

impl Drop for MediaFoundationStream {
    /// `Dispose`: the device shut down, which lets the camera go.
    fn drop(&mut self) {
        shut(&self.source);
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

    use super::*;

    fn packed(high: u32, low: u32) -> u64 {
        (u64::from(high) << 32) | u64::from(low)
    }

    /// A native type reads as the mode it is: MJPEG and YUY2 kept, the size from the high and
    /// low halves, frames a second turned into the frame interval; anything else left out.
    #[test]
    fn a_native_type_reads_as_its_mode() {
        let mjpeg = mode_of(MFVideoFormat_MJPG, packed(1280, 720), Some(packed(30, 1))).unwrap();
        assert_eq!(
            mjpeg,
            Mode {
                format: PixelFormat::Mjpeg,
                width: 1280,
                height: 720,
                interval: (1, 30),
            }
        );
        assert_eq!(mjpeg.label(), "1280 x 720 30.00 fps MJPG");
        let ntsc = mode_of(
            MFVideoFormat_YUY2,
            packed(640, 480),
            Some(packed(30000, 1001)),
        )
        .unwrap();
        assert_eq!(ntsc.format, PixelFormat::Yuyv);
        assert_eq!(ntsc.interval, (1001, 30000));
        assert_eq!(ntsc.label(), "640 x 480 29.97 fps YUYV");
        let unrated = mode_of(MFVideoFormat_YUY2, packed(320, 240), None).unwrap();
        assert_eq!(unrated.interval, (0, 1));
        let nv12 = GUID::from_u128(0x3231_564e_0000_0010_8000_00aa_0038_9b71);
        assert_eq!(mode_of(nv12, packed(640, 480), Some(packed(30, 1))), None);
        assert_eq!(mode_of(MFVideoFormat_MJPG, packed(0, 480), None), None);
    }

    /// Media Foundation's one type a rate becomes one entry a format and size, where its first
    /// type stood, at its fastest rate.
    #[test]
    fn one_entry_a_format_and_size_at_its_fastest() {
        let mode = |format, width, height, fps: u32| Mode {
            format,
            width,
            height,
            interval: (1, fps),
        };
        let listed = fold_modes([
            mode(PixelFormat::Yuyv, 640, 480, 15),
            mode(PixelFormat::Mjpeg, 1280, 720, 30),
            mode(PixelFormat::Yuyv, 640, 480, 30),
            mode(PixelFormat::Mjpeg, 1280, 720, 5),
            mode(PixelFormat::Mjpeg, 640, 480, 30),
            Mode {
                interval: (0, 1),
                ..mode(PixelFormat::Yuyv, 320, 240, 1)
            },
            mode(PixelFormat::Yuyv, 320, 240, 10),
        ]);
        let labels: Vec<String> = listed.iter().map(Mode::label).collect();
        assert_eq!(
            labels,
            [
                "640 x 480 30.00 fps YUYV",
                "1280 x 720 30.00 fps MJPG",
                "640 x 480 30.00 fps MJPG",
                "320 x 240 10.00 fps YUYV",
            ]
        );
    }

    /// A native type made as a camera lists it.
    fn native(format: GUID, width: u32, height: u32, fps: u32) -> (IMFMediaType, Mode) {
        use windows::Win32::Media::MediaFoundation::MFCreateMediaType;
        // SAFETY: a new media type and three attribute writes by GUID from the SDK.
        let media_type = unsafe {
            let media_type = MFCreateMediaType().unwrap();
            media_type.SetGUID(&MF_MT_SUBTYPE, &format).unwrap();
            media_type
                .SetUINT64(&MF_MT_FRAME_SIZE, packed(width, height))
                .unwrap();
            media_type
                .SetUINT64(&MF_MT_FRAME_RATE, packed(fps, 1))
                .unwrap();
            media_type
        };
        let mode = mode_of(format, packed(width, height), Some(packed(fps, 1))).unwrap();
        (media_type, mode)
    }

    /// Start takes the native type of the mode's format and size at the mode's rate; a rate the
    /// device does not have gives the fastest of that format and size; another format or size
    /// is never taken.
    #[test]
    fn start_takes_the_type_of_the_mode() {
        let _platform = Platform::start().unwrap();
        let types = || {
            vec![
                native(MFVideoFormat_YUY2, 640, 480, 30),
                native(MFVideoFormat_MJPG, 640, 480, 15),
                native(MFVideoFormat_MJPG, 640, 480, 30),
                native(MFVideoFormat_MJPG, 640, 480, 5),
                native(MFVideoFormat_MJPG, 1280, 720, 60),
            ]
        };
        let asked = |fps| Mode {
            format: PixelFormat::Mjpeg,
            width: 640,
            height: 480,
            interval: (1, fps),
        };
        let rate = |(media_type, _): (IMFMediaType, Mode)| {
            // SAFETY: an attribute read by GUID from the SDK.
            halves(unsafe { media_type.GetUINT64(&MF_MT_FRAME_RATE) }.unwrap())
        };
        assert_eq!(pick(types(), &asked(15)).map(rate), Some((15, 1)));
        assert_eq!(pick(types(), &asked(5)).map(rate), Some((5, 1)));
        assert_eq!(pick(types(), &asked(25)).map(rate), Some((30, 1)));
        let (_, found) = pick(types(), &asked(25)).unwrap();
        assert_eq!(found.interval, (1, 30));
        let elsewhere = Mode {
            width: 320,
            height: 240,
            ..asked(30)
        };
        assert!(pick(types(), &elsewhere).is_none());
    }

    /// The reader's callback, fed as Media Foundation feeds it: a sample's bytes go to the
    /// reading thread whole; a failed read, a device error and the stream's end go as failures;
    /// a read with no sample goes as nothing.
    #[test]
    fn the_callback_hands_on_what_each_read_brought() {
        use windows::Win32::Foundation::{E_FAIL, S_OK};
        use windows::Win32::Media::MediaFoundation::{MFCreateMemoryBuffer, MFCreateSample};
        let _platform = Platform::start().unwrap();
        let (sender, arrivals) = mpsc::channel();
        let callback: IMFSourceReaderCallback = Arrivals {
            sender: Mutex::new(sender),
        }
        .into();
        let jpeg = [0xFF_u8, 0xD8, 0xFF, 0xE0, 1, 2, 3, 0xFF, 0xD9];
        // SAFETY: a new sample holding one new buffer, its bytes written while it is locked.
        let sample = unsafe {
            let buffer = MFCreateMemoryBuffer(64).unwrap();
            let mut data: *mut u8 = std::ptr::null_mut();
            buffer.Lock(&raw mut data, None, None).unwrap();
            std::ptr::copy_nonoverlapping(jpeg.as_ptr(), data, jpeg.len());
            buffer.Unlock().unwrap();
            buffer.SetCurrentLength(9).unwrap();
            let sample = MFCreateSample().unwrap();
            sample.AddBuffer(&buffer).unwrap();
            sample
        };
        #[allow(clippy::cast_sign_loss)]
        let (error, ended) = (
            MF_SOURCE_READERF_ERROR.0 as u32,
            MF_SOURCE_READERF_ENDOFSTREAM.0 as u32,
        );
        // SAFETY: the callback called as the reader calls it, with a sample or none.
        unsafe {
            callback.OnReadSample(S_OK, 0, 0, 0, &sample).unwrap();
            callback.OnReadSample(S_OK, 0, 0, 0, None).unwrap();
            callback.OnReadSample(E_FAIL, 0, 0, 0, None).unwrap();
            callback.OnReadSample(S_OK, 0, error, 0, None).unwrap();
            callback.OnReadSample(S_OK, 0, ended, 0, None).unwrap();
        }
        let got: Vec<Arrival> = arrivals.try_iter().collect();
        assert_eq!(got.len(), 5);
        assert!(matches!(&got[0], Arrival::Frame(bytes) if bytes == &jpeg));
        assert!(matches!(&got[1], Arrival::Nothing));
        assert!(matches!(&got[2], Arrival::Failed(VideoError::Read(why)) if !why.is_empty()));
        assert!(
            matches!(&got[3], Arrival::Failed(VideoError::Read(why)) if why == "the device reported an error")
        );
        assert!(
            matches!(&got[4], Arrival::Failed(VideoError::Read(why)) if why == "the stream ended")
        );
    }

    /// This machine's cameras, whatever they are - the tiny10 VM has none, which must list as
    /// none rather than fail - each with a name, a link and decodable modes.
    #[test]
    fn the_machines_cameras_are_named_and_decodable() {
        let source = MediaFoundationSource;
        for device in source.devices() {
            assert!(!device.name.is_empty(), "{device:?}");
            assert!(!device.path.as_os_str().is_empty(), "{device:?}");
            if let Ok(modes) = source.modes(&device) {
                for mode in modes {
                    assert!(mode.width > 0 && mode.height > 0, "{mode:?}");
                }
            }
        }
    }

    /// A device that is not there fails to list or open with the C#'s words, not a panic.
    #[test]
    fn a_device_that_is_not_there_says_so() {
        let source = MediaFoundationSource;
        let gone = Device {
            path: PathBuf::from(
                r"\\?\usb#vid_0000&pid_0000&mi_00#0#{e5323777-f976-4f5b-9b55-b94699c46e44}\global",
            ),
            name: "gone".to_owned(),
        };
        let listed = source.modes(&gone).unwrap_err();
        assert!(
            listed
                .to_string()
                .starts_with("No video capture devices found at that index!"),
            "{listed}"
        );
        let mode = Mode {
            format: PixelFormat::Mjpeg,
            width: 640,
            height: 480,
            interval: (1, 30),
        };
        let opened = source.open(&gone, &mode).err().unwrap();
        assert!(
            opened
                .to_string()
                .starts_with("No video capture devices found at that index!"),
            "{opened}"
        );
    }

    /// This machine's webcam, for real: listed, its modes, and ten frames decoded in its first
    /// mode. It takes the camera, so it runs only on request:
    /// `cargo test -p mp-video -- --ignored --nocapture`.
    #[test]
    #[ignore = "opens the machine's webcam"]
    fn the_webcam_lists_and_streams() {
        let source = MediaFoundationSource;
        let devices = source.devices();
        for device in &devices {
            eprintln!("device {:?} {}", device.name, device.path.display());
        }
        let first = devices.first().expect("a camera");
        let modes = source.modes(first).unwrap();
        for mode in &modes {
            eprintln!("mode {}", mode.label());
        }
        let mode = modes.first().expect("a decodable mode");
        let started = web_time::Instant::now();
        let capture = crate::Capture::start(&source, first, mode).unwrap();
        while capture.frames() < 10 {
            assert!(
                started.elapsed() < Duration::from_secs(10),
                "{capture:?} {:?}",
                capture.error()
            );
            wasm_thread::sleep(Duration::from_millis(5));
        }
        let frame = capture.latest().unwrap();
        eprintln!(
            "{} frames of {} x {} in {:?}; last error {:?}",
            capture.frames(),
            frame.width,
            frame.height,
            started.elapsed(),
            capture.error()
        );
        assert_eq!(
            frame.rgba.len(),
            frame.width as usize * frame.height as usize * 4
        );
    }
}
