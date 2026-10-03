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

//! Raw frames to RGBA: the sample grabber's job in the C# (`Capture.cs`'s `BufferCB` hands the
//! HUD a 32-bit bitmap), done here for the two formats a webcam offers over V4L2.

use crate::{Frame, Mode, PixelFormat, VideoError};

/// One raw frame in `mode`'s format to RGBA.
///
/// # Errors
///
/// A JPEG that will not decode, or a YUYV buffer of the wrong length.
pub fn decode(mode: &Mode, raw: &[u8], sequence: u64) -> Result<Frame, VideoError> {
    match mode.format {
        PixelFormat::Mjpeg => mjpeg_to_frame(raw, sequence),
        PixelFormat::Yuyv => yuyv_to_frame(mode.width, mode.height, raw, sequence),
    }
}

/// A JPEG frame, whatever size the encoder made it (a camera can send a smaller image than the
/// mode asked for at the start of a stream).
///
/// # Errors
///
/// What the JPEG decoder says.
pub fn mjpeg_to_frame(raw: &[u8], sequence: u64) -> Result<Frame, VideoError> {
    let image = image::load_from_memory_with_format(raw, image::ImageFormat::Jpeg)
        .map_err(|why| VideoError::BadFrame(why.to_string()))?;
    let rgba = image.to_rgba8();
    Ok(Frame {
        width: rgba.width(),
        height: rgba.height(),
        rgba: rgba.into_raw(),
        sequence,
    })
}

/// A picture of a stream's part - a JPEG from an MJPEG server, a PNG from the GStreamer
/// pipeline - to RGBA, its format told by its first bytes: `new Bitmap(new MemoryStream(buf1))`.
/// `// C#: ExtLibs/Utilities/CaptureMJPEG.cs:168-171`
///
/// # Errors
///
/// What the decoder says.
pub fn picture_to_frame(raw: &[u8], sequence: u64) -> Result<Frame, VideoError> {
    let image =
        image::load_from_memory(raw).map_err(|why| VideoError::BadFrame(why.to_string()))?;
    let rgba = image.to_rgba8();
    Ok(Frame {
        width: rgba.width(),
        height: rgba.height(),
        rgba: rgba.into_raw(),
        sequence,
    })
}

/// Packed YUYV (4:2:2) to RGBA: each four bytes are two pixels sharing one U and one V, BT.601
/// limited range, as V4L2's `V4L2_PIX_FMT_YUYV` is defined.
///
/// # Errors
///
/// A buffer shorter than `width * height * 2`.
pub fn yuyv_to_frame(
    width: u32,
    height: u32,
    raw: &[u8],
    sequence: u64,
) -> Result<Frame, VideoError> {
    let pixels = (width as usize) * (height as usize);
    let needed = pixels * 2;
    if raw.len() < needed {
        return Err(VideoError::BadFrame(format!(
            "YUYV frame of {} bytes, {needed} needed for {width} x {height}",
            raw.len()
        )));
    }
    let mut rgba = Vec::with_capacity(pixels * 4);
    for quad in raw.chunks_exact(4).take(pixels / 2) {
        if let &[y0, u, y1, v] = quad {
            push_pixel(&mut rgba, y0, u, v);
            push_pixel(&mut rgba, y1, u, v);
        }
    }
    Ok(Frame {
        width,
        height,
        rgba,
        sequence,
    })
}

/// One YUV sample to RGBA, BT.601 limited range in fixed point.
#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // clamped to 0..=255 first
fn push_pixel(out: &mut Vec<u8>, y: u8, u: u8, v: u8) {
    let c = i32::from(y) - 16;
    let d = i32::from(u) - 128;
    let e = i32::from(v) - 128;
    let r = (298 * c + 409 * e + 128) >> 8;
    let g = (298 * c - 100 * d - 208 * e + 128) >> 8;
    let b = (298 * c + 516 * d + 128) >> 8;
    out.push(r.clamp(0, 255) as u8);
    out.push(g.clamp(0, 255) as u8);
    out.push(b.clamp(0, 255) as u8);
    out.push(255);
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::indexing_slicing)]

    use super::*;

    /// The standard's own worked values: black, white, and a saturated red.
    #[test]
    fn yuyv_decodes_the_bt601_reference_colours() {
        // Two black pixels, then two white, then two red (Y 81, U 90, V 240).
        let raw = [16, 128, 16, 128, 235, 128, 235, 128, 81, 90, 81, 240];
        let frame = yuyv_to_frame(6, 1, &raw, 7).unwrap();
        assert_eq!(frame.sequence, 7);
        assert_eq!(&frame.rgba[0..4], &[0, 0, 0, 255]);
        assert_eq!(&frame.rgba[8..12], &[255, 255, 255, 255]);
        let red = &frame.rgba[16..20];
        assert!(red[0] >= 250 && red[1] <= 5 && red[2] <= 5, "{red:?}");
    }

    #[test]
    fn a_short_yuyv_buffer_is_refused_with_the_sizes() {
        let error = yuyv_to_frame(4, 2, &[0; 10], 0).err().unwrap();
        assert_eq!(
            error.to_string(),
            "bad frame: YUYV frame of 10 bytes, 16 needed for 4 x 2"
        );
    }

    /// A JPEG the `image` crate wrote comes back the same size, near enough the same colour.
    #[test]
    fn mjpeg_round_trips_through_the_jpeg_encoder() {
        let mut picture = image::RgbImage::new(16, 8);
        for pixel in picture.pixels_mut() {
            *pixel = image::Rgb([200, 30, 30]);
        }
        let mut jpeg = Vec::new();
        image::DynamicImage::ImageRgb8(picture)
            .write_to(
                &mut std::io::Cursor::new(&mut jpeg),
                image::ImageFormat::Jpeg,
            )
            .unwrap();
        let frame = mjpeg_to_frame(&jpeg, 3).unwrap();
        assert_eq!((frame.width, frame.height), (16, 8));
        assert_eq!(frame.rgba.len(), 16 * 8 * 4);
        let [r, g, b, a] = [frame.rgba[0], frame.rgba[1], frame.rgba[2], frame.rgba[3]];
        assert!(r > 180 && g < 60 && b < 60 && a == 255, "{r} {g} {b} {a}");
        assert!(mjpeg_to_frame(b"not a jpeg", 0).is_err());
    }
}
