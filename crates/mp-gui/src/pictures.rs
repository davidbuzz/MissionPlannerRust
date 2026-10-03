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

//! Mission Planner's pictures: the `Properties.Resources` images the ported pages' `PictureBox`es,
//! `ImageLabel`s, Frame Class buttons and grid image columns show, drawn in their boxes the way
//! WinForms lays them out; the SITL page's vehicle pictures, which live in `SITL.resx` alone; and
//! the HUD's `HUDT` icons.
//!
//! The files are the C# tree's `Resources/*.png` (both trees are GPL-3.0-only), copied to
//! `assets/images/<resource>.<ext>` under the resource's name - the `Properties.Resources` property
//! the Designer assigns, so `global::MissionPlanner.Properties.Resources.frames_h` is
//! `assets/images/frames_h.png` although `Properties/Resources.resx` points it at
//! `Resources/frames-h.png`. The HUD's are `ExtLibs/Controls/Resources/*.png` under their `HUDT`
//! property's name (`HUDT._2dfix_wide` is `2dfix_wide.png`, carried as `_2dfix_wide.png`). An
//! image that is only a form's `.resx` base64, as SITL's are, is carried under that form and key
//! (`SITL.pictureBoxplane.ImageNormal.png`). Only the pictures a ported page shows are carried:
//! `SITES`, the tests' table, is every assignment, each held to its Designer line, its `.resx`
//! layout and the file's bytes, and `hud::Icon::ALL` is every picture `HUD.cs` draws. The six
//! product photographs the Designers also show - a Cube, a power module, a MinimOSD, an airspeed
//! sensor, a sonar and an optical flow sensor, the tree's `Resources/*.jpg` - are not carried:
//! they are the makers' pictures, not Mission Planner's to license, so their boxes draw their names
//! as they did before the images came (the owner's ruling, 2026-10-03; `PHOTOGRAPHS`, under test,
//! names them).
//!
//! Each image is embedded with `include_bytes!`, decoded once with the `image` crate on first use
//! and kept; a box that scales it gets a copy resampled to its size in device pixels, also kept,
//! so the GPU draws it one texel to a pixel rather than minifying a 1042-pixel image into 150
//! without mipmaps. A resource that is not embedded, or does not decode, falls back to the box
//! with its name in it that the pages drew before.
//!
//! [`placements`] is WinForms' arithmetic, from the .NET reference source (not in the C# tree):
//! `ControlPaint.CalculateBackgroundImageRectangle` for a `BackgroundImage` and its
//! `BackgroundImageLayout`, `PictureBox.ImageRectangleFromSizeMode` for an `Image` and its
//! `SizeMode`.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::collections::HashMap;
use std::sync::{Arc, Mutex, OnceLock};

use gpui::{
    AnyElement, Bounds, Corners, Div, Pixels, RenderImage, Window, canvas, div, point, prelude::*,
    px, rgb, size,
};
use image::RgbaImage;

use crate::ui::theme;

/// A box: `Location` and `Size`, as the `.resx` gives them.
pub type Place = (f32, f32, f32, f32);

/// How an image sits in its box.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Layout {
    /// `ImageLayout.None` and `PictureBoxSizeMode.Normal`: at the top left corner, unscaled,
    /// clipped by the box.
    None,
    /// `ImageLayout.Tile`, the `BackgroundImageLayout` WinForms defaults to: unscaled copies
    /// from the top left corner, clipped by the box.
    #[cfg_attr(not(test), allow(dead_code))]
    Tile,
    /// `ImageLayout.Center`: unscaled, centred in each direction the box is larger in, at the
    /// left or top edge in one it is not. Also a `DataGridViewImageColumn`'s cell, whose
    /// `ImageLayout` defaults to `Normal`: "centered, original size" - in a cell at least as large
    /// as the image, which Compass's 40 by 22 cells are for their 20 by 20 arrows.
    Center,
    /// `ImageLayout.Stretch` and `PictureBoxSizeMode.StretchImage`: the whole box. Also what
    /// `new Bitmap(image, width, height)` is, drawn in a box of that size.
    Stretch,
    /// `ImageLayout.Zoom`, for a `BackgroundImage`: the aspect kept, filling the direction that
    /// limits it, the other length rounded and centred.
    Zoom,
    /// `PictureBoxSizeMode.Zoom`, for an `Image`: the aspect kept, both lengths truncated, and
    /// centred both ways.
    ZoomImage,
}

/// One `Properties.Resources` image and its bytes.
#[derive(Debug, Clone, Copy)]
pub struct Embedded {
    /// The resource's name: the `Properties.Resources` property the Designer assigns.
    pub resource: &'static str,
    /// Its file in `assets/images`.
    #[cfg_attr(not(test), allow(dead_code))]
    pub file: &'static str,
    /// The file.
    pub bytes: &'static [u8],
}

/// An [`Embedded`] for `assets/images/<file>`.
macro_rules! embedded {
    ($resource:literal, $file:literal) => {
        Embedded {
            resource: $resource,
            file: $file,
            bytes: include_bytes!(concat!("../../../assets/images/", $file)),
        }
    };
}

/// Every image carried, by resource name.
/// `// C#: Properties/Resources.resx` (the file each resource names) and `Resources.Designer.cs`
/// (the property each resource is); `ExtLibs/Controls/HUDT.resx` and `HUDT.Designer.cs` for the
/// HUD's; `GCSViews/SITL.resx` for SITL's.
pub const IMAGES: [Embedded; 63] = [
    embedded!("APM_airframes_001", "APM_airframes_001.png"),
    embedded!("APM_airframes_08", "APM_airframes_08.png"),
    embedded!("Antenna_Tracker_01", "Antenna_Tracker_01.png"),
    
    embedded!("FW_icons_2013_logos_03", "FW_icons_2013_logos_03.png"),
    embedded!("FW_icons_2013_logos_04", "FW_icons_2013_logos_04.png"),
    embedded!("FW_icons_2013_logos_06", "FW_icons_2013_logos_06.png"),
    embedded!("FW_icons_2013_logos_07", "FW_icons_2013_logos_07.png"),
    embedded!("FW_icons_2013_logos_08", "FW_icons_2013_logos_08.png"),
    embedded!("FW_icons_2013_logos_09", "FW_icons_2013_logos_09.png"),
    embedded!("FW_icons_2013_logos_10", "FW_icons_2013_logos_10.png"),
    embedded!("FW_icons_2013_logos_12", "FW_icons_2013_logos_12.png"),
    embedded!("FW_icons_2013_logos_13", "FW_icons_2013_logos_13.png"),
    
    embedded!("Parachute", "Parachute.png"),
    embedded!(
        "SITL.pictureBoxheli.ImageNormal",
        "SITL.pictureBoxheli.ImageNormal.png"
    ),
    embedded!(
        "SITL.pictureBoxheli.ImageOver",
        "SITL.pictureBoxheli.ImageOver.png"
    ),
    embedded!(
        "SITL.pictureBoxplane.ImageNormal",
        "SITL.pictureBoxplane.ImageNormal.png"
    ),
    embedded!(
        "SITL.pictureBoxplane.ImageOver",
        "SITL.pictureBoxplane.ImageOver.png"
    ),
    embedded!(
        "SITL.pictureBoxquad.ImageNormal",
        "SITL.pictureBoxquad.ImageNormal.png"
    ),
    embedded!(
        "SITL.pictureBoxquad.ImageOver",
        "SITL.pictureBoxquad.ImageOver.png"
    ),
    embedded!(
        "SITL.pictureBoxrover.ImageNormal",
        "SITL.pictureBoxrover.ImageNormal.png"
    ),
    embedded!(
        "SITL.pictureBoxrover.ImageOver",
        "SITL.pictureBoxrover.ImageOver.png"
    ),
    embedded!("Shutter", "Shutter.png"),
    embedded!("_2dfix_wide", "_2dfix_wide.png"),
    embedded!("_3ddgps_wide", "_3ddgps_wide.png"),
    embedded!("_3dfix_wide", "_3dfix_wide.png"),
    
    embedded!("batt_1", "batt_1.png"),
    embedded!("batt_2", "batt_2.png"),
    embedded!("batt_3", "batt_3.png"),
    embedded!("batt_4", "batt_4.png"),
    embedded!("batt_red", "batt_red.png"),
    embedded!("batt_yellow", "batt_yellow.png"),
    embedded!("cameraGimalPitch1", "cameraGimalPitch1.png"),
    embedded!("cameraGimalRoll1", "cameraGimalRoll1.png"),
    embedded!("cameraGimalYaw", "cameraGimalYaw.png"),
    embedded!("boat", "boat.png"),
    embedded!("down", "down.png"),
    embedded!("ekf_green", "ekf_green.png"),
    embedded!("ekf_red", "ekf_red.png"),
    embedded!("ekf_yellow", "ekf_yellow.png"),
    embedded!("frames_h", "frames_h.png"),
    embedded!("frames_plus", "frames_plus.png"),
    embedded!("frames_x", "frames_x.png"),
    embedded!("heli", "heli.png"),
    embedded!("new_3DR_04", "new_3DR_04.png"),
    embedded!("nofix_wide", "nofix_wide.png"),
    embedded!("nogps_wide", "nogps_wide.png"),
    
    
    embedded!("prearm_green", "prearm_green.png"),
    embedded!("prearm_red", "prearm_red.png"),
    // `Resources.quadicon`, the Proximity window's copter (`config/proximity.rs`).
    embedded!("quadicon", "quadicon.png"),
    embedded!("redsinglecopter2", "redsinglecopter2.png"),
    // `MissionPlanner.Maps.Resources.rover`, the flight map's rover marker: `car.png` in
    // `ExtLibs/Maps/Resources.resx`, carried under the resource's name as the rest are.
    embedded!("rover", "rover.png"),
    embedded!("rover_11", "rover_11.png"),
    embedded!("rtkfixed_wide", "rtkfixed_wide.png"),
    embedded!("rtkfloat_wide", "rtkfloat_wide.png"),
    
    embedded!("sub", "sub.png"),
    embedded!("unknown", "unknown.png"),
    embedded!("up", "up.png"),
    embedded!("vibe_green", "vibe_green.png"),
    embedded!("vibe_red", "vibe_red.png"),
    embedded!("vibe_yellow", "vibe_yellow.png"),
    embedded!("x8", "x8.png"),
    embedded!("y6a", "y6a.png"),
    embedded!("y6b", "y6b.png"),
];

/// Where a picture's image comes from in the C#.
#[cfg(test)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Source {
    /// `this.<control>.<property> = global::MissionPlanner.Properties.Resources.<resource>;` in
    /// the Designer.
    Resources,
    /// `<control>.<property>` in the page's own `.resx`, as base64 - the same bytes as the
    /// resource this port draws in its place.
    Resx,
}

/// One image the C# assigns to a control of a ported page.
#[cfg(test)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Site {
    /// The Designer, under `GCSViews/ConfigurationView/`, or its path in the C# tree where it
    /// is elsewhere (`GCSViews/SITL.Designer.cs`).
    pub designer: &'static str,
    /// The control.
    pub control: &'static str,
    /// `Image` or `BackgroundImage`.
    pub property: &'static str,
    /// The resource drawn.
    pub resource: &'static str,
    /// Where the Designer takes it from.
    pub from: Source,
    /// Its layout: the `.resx`'s `BackgroundImageLayout` or `SizeMode` (`Normal` where it gives
    /// none), `ImageLabel`'s inner `PictureBox`'s `Zoom`, the Frame Class buttons' 60 by 60
    /// `Bitmap`, or a grid image column's centred cell.
    pub layout: Layout,
}

/// A [`Site`] whose image is a `Properties.Resources` assignment.
#[cfg(test)]
const fn site(
    designer: &'static str,
    control: &'static str,
    property: &'static str,
    resource: &'static str,
    layout: Layout,
) -> Site {
    Site {
        designer,
        control,
        property,
        resource,
        from: Source::Resources,
        layout,
    }
}

/// The Designers of the ported pages.
#[cfg(test)]
const OSD: &str = "ConfigHWOSD.Designer.cs";
/// Camera Gimbal.
#[cfg(test)]
const MOUNT: &str = "ConfigMount.designer.cs";
/// Frame Type before 3.5.
#[cfg(test)]
const FRAME_LEGACY: &str = "ConfigFrameType.Designer.cs";
/// Frame Type.
#[cfg(test)]
const FRAME: &str = "ConfigFrameClassType.Designer.cs";
/// Install Firmware.
#[cfg(test)]
const FIRMWARE: &str = "ConfigFirmwareManifest.Designer.cs";
/// Install Firmware Legacy.
#[cfg(test)]
const FIRMWARE_LEGACY: &str = "ConfigFirmware.Designer.cs";
/// Compass.
#[cfg(test)]
const COMPASS: &str = "ConfigHWCompass2.Designer.cs";
/// The SITL page, which is not under `ConfigurationView`.
#[cfg(test)]
pub const SITL: &str = "GCSViews/SITL.Designer.cs";

/// A [`Site`] whose image is base64 in its page's `.resx`, under `<control>.<property>`, and
/// carried as `<resource>`.
#[cfg(test)]
const fn resx_site(
    designer: &'static str,
    control: &'static str,
    property: &'static str,
    resource: &'static str,
    layout: Layout,
) -> Site {
    Site {
        designer,
        control,
        property,
        resource,
        from: Source::Resx,
        layout,
    }
}

/// Every image the ported pages' Designers assign, in each Designer's order.
/// `// C#: GCSViews/ConfigurationView/*.Designer.cs, *.resx; GCSViews/SITL.Designer.cs:140-174,
/// SITL.resx; ConfigHWCompass2.Designer.cs:523, 531`
#[cfg(test)]
#[rustfmt::skip]
pub const SITES: [Site; 63] = [
    site(OSD, "pictureBox5", "BackgroundImage", "MinimOSD", Layout::Zoom),
    site(MOUNT, "pictureBox1", "BackgroundImage", "cameraGimalPitch1", Layout::Zoom),
    site(MOUNT, "pictureBox2", "BackgroundImage", "cameraGimalRoll1", Layout::Zoom),
    site(MOUNT, "pictureBox3", "BackgroundImage", "cameraGimalYaw", Layout::Zoom),
    site(MOUNT, "pictureBox4", "BackgroundImage", "Shutter", Layout::Zoom),
    site("ConfigHWAirspeed.Designer.cs", "pictureBox4", "BackgroundImage", "airspeed", Layout::Zoom),
    site("ConfigHWRangeFinder.Designer.cs", "pictureBox3", "BackgroundImage", "sonar", Layout::Zoom),
    resx_site("ConfigBatteryMonitoring.Designer.cs", "pictureBox5", "BackgroundImage", "BR_APMPWRDEAN_2", Layout::Zoom),
    site("ConfigBatteryMonitoring2.Designer.cs", "pictureBox5", "BackgroundImage", "BR_APMPWRDEAN_2", Layout::Zoom),
    site("ConfigHWOptFlow.Designer.cs", "pictureBox2", "BackgroundImage", "opticalflow", Layout::Zoom),
    site("ConfigHWParachute.Designer.cs", "pictureBox3", "BackgroundImage", "sonar", Layout::Zoom),
    site("ConfigHWParachute.Designer.cs", "pictureBox3", "Image", "Parachute", Layout::ZoomImage),
    site(FRAME_LEGACY, "pictureBoxPlus", "Image", "frames_plus", Layout::ZoomImage),
    site(FRAME_LEGACY, "pictureBoxX", "Image", "frames_x", Layout::ZoomImage),
    site(FRAME_LEGACY, "pictureBoxV", "Image", "new_3DR_04", Layout::ZoomImage),
    site(FRAME_LEGACY, "pictureBoxH", "Image", "frames_h", Layout::ZoomImage),
    site(FRAME_LEGACY, "pictureBoxY", "Image", "y6b", Layout::ZoomImage),
    site(FRAME, "pictureBoxPlus", "Image", "frames_plus", Layout::ZoomImage),
    site(FRAME, "pictureBoxX", "Image", "frames_x", Layout::ZoomImage),
    site(FRAME, "pictureBoxV", "Image", "new_3DR_04", Layout::ZoomImage),
    site(FRAME, "pictureBoxH", "Image", "frames_h", Layout::ZoomImage),
    site(FRAME, "pictureBoxY", "Image", "y6b", Layout::ZoomImage),
    site(FRAME, "radioButtonOctaQuad", "Image", "FW_icons_2013_logos_06", Layout::Stretch),
    site(FRAME, "radioButtonTri", "Image", "FW_icons_2013_logos_08", Layout::Stretch),
    site(FRAME, "radioButtonHeli", "Image", "FW_icons_2013_logos_13", Layout::Stretch),
    site(FRAME, "radioButtonY6", "Image", "FW_icons_2013_logos_07", Layout::Stretch),
    site(FRAME, "radioButtonOcta", "Image", "FW_icons_2013_logos_12", Layout::Stretch),
    site(FRAME, "radioButtonHexa", "Image", "FW_icons_2013_logos_09", Layout::Stretch),
    site(FRAME, "radioButtonQuad", "Image", "FW_icons_2013_logos_03", Layout::Stretch),
    site(FIRMWARE, "imageLabel1", "Image", "pixhawk2cube", Layout::ZoomImage),
    site(FIRMWARE, "pictureBoxSub", "Image", "sub", Layout::ZoomImage),
    site(FIRMWARE, "pictureAntennaTracker", "Image", "Antenna_Tracker_01", Layout::ZoomImage),
    site(FIRMWARE, "pictureBoxRover", "Image", "rover_11", Layout::ZoomImage),
    site(FIRMWARE, "pictureBoxOctaQuad", "Image", "x8", Layout::ZoomImage),
    site(FIRMWARE, "pictureBoxOcta", "Image", "FW_icons_2013_logos_12", Layout::ZoomImage),
    site(FIRMWARE, "pictureBoxHeli", "Image", "APM_airframes_08", Layout::ZoomImage),
    site(FIRMWARE, "pictureBoxY6", "Image", "y6a", Layout::ZoomImage),
    site(FIRMWARE, "pictureBoxTri", "Image", "FW_icons_2013_logos_08", Layout::ZoomImage),
    site(FIRMWARE, "pictureBoxHexa", "Image", "FW_icons_2013_logos_10", Layout::ZoomImage),
    site(FIRMWARE, "pictureBoxQuad", "Image", "FW_icons_2013_logos_04", Layout::ZoomImage),
    site(FIRMWARE, "pictureBoxPlane", "Image", "APM_airframes_001", Layout::ZoomImage),
    site(FIRMWARE_LEGACY, "pictureBoxAPM", "Image", "APM_airframes_001", Layout::ZoomImage),
    site(FIRMWARE_LEGACY, "pictureBoxQuad", "Image", "FW_icons_2013_logos_04", Layout::ZoomImage),
    site(FIRMWARE_LEGACY, "pictureBoxHexa", "Image", "FW_icons_2013_logos_10", Layout::ZoomImage),
    site(FIRMWARE_LEGACY, "pictureBoxTri", "Image", "FW_icons_2013_logos_08", Layout::ZoomImage),
    site(FIRMWARE_LEGACY, "pictureBoxY6", "Image", "y6a", Layout::ZoomImage),
    site(FIRMWARE_LEGACY, "pictureBoxHeli", "Image", "APM_airframes_08", Layout::ZoomImage),
    site(FIRMWARE_LEGACY, "pictureBoxOcta", "Image", "FW_icons_2013_logos_12", Layout::ZoomImage),
    site(FIRMWARE_LEGACY, "pictureBoxOctaQuad", "Image", "x8", Layout::ZoomImage),
    site(FIRMWARE_LEGACY, "pictureBoxRover", "Image", "rover_11", Layout::ZoomImage),
    site(FIRMWARE_LEGACY, "pictureAntennaTracker", "Image", "Antenna_Tracker_01", Layout::ZoomImage),
    site(FIRMWARE_LEGACY, "pictureBoxSub", "Image", "sub", Layout::ZoomImage),
    site(FIRMWARE_LEGACY, "imageLabel1", "Image", "pixhawk2cube", Layout::ZoomImage),
    site(COMPASS, "Up", "Image", "up", Layout::Center),
    site(COMPASS, "Down", "Image", "down", Layout::Center),
    resx_site(SITL, "pictureBoxheli", "ImageNormal", "SITL.pictureBoxheli.ImageNormal", Layout::None),
    resx_site(SITL, "pictureBoxheli", "ImageOver", "SITL.pictureBoxheli.ImageOver", Layout::None),
    resx_site(SITL, "pictureBoxquad", "ImageNormal", "SITL.pictureBoxquad.ImageNormal", Layout::None),
    resx_site(SITL, "pictureBoxquad", "ImageOver", "SITL.pictureBoxquad.ImageOver", Layout::None),
    resx_site(SITL, "pictureBoxrover", "ImageNormal", "SITL.pictureBoxrover.ImageNormal", Layout::None),
    resx_site(SITL, "pictureBoxrover", "ImageOver", "SITL.pictureBoxrover.ImageOver", Layout::None),
    resx_site(SITL, "pictureBoxplane", "ImageNormal", "SITL.pictureBoxplane.ImageNormal", Layout::None),
    resx_site(SITL, "pictureBoxplane", "ImageOver", "SITL.pictureBoxplane.ImageOver", Layout::None),
];

/// The product photographs the Designers assign that are not carried - the Cube on both Install
/// Firmware pages, the power module on both Battery Monitors, the MinimOSD, the airspeed sensor,
/// the sonar (Range Finder and Parachute) and the optical flow sensor: the makers' pictures, not
/// Mission Planner's to license, left out on the owner's ruling of 2026-10-03. Their `SITES` rows
/// stay as the record of what the Designers show; their boxes draw their names.
#[cfg(test)]
pub const PHOTOGRAPHS: [&str; 6] = [
    "BR_APMPWRDEAN_2",
    "MinimOSD",
    "airspeed",
    "opticalflow",
    "pixhawk2cube",
    "sonar",
];

/// The bytes of a resource, if it is carried.
#[must_use]
pub fn bytes(resource: &str) -> Option<&'static [u8]> {
    IMAGES
        .iter()
        .find(|image| image.resource == resource)
        .map(|image| image.bytes)
}

/// A resource's pixels, straight RGBA, decoded on first use and kept; `None` when it is not
/// carried or does not decode, and then never tried again.
fn decoded(resource: &'static str) -> Option<Arc<RgbaImage>> {
    type Decoded = HashMap<&'static str, Option<Arc<RgbaImage>>>;
    static DECODED: OnceLock<Mutex<Decoded>> = OnceLock::new();
    let mut cache = DECODED.get_or_init(Mutex::default).lock().ok()?;
    cache
        .entry(resource)
        .or_insert_with(|| {
            let bytes = bytes(resource)?;
            image::load_from_memory(bytes)
                .ok()
                .map(|image| Arc::new(image.into_rgba8()))
        })
        .clone()
}

/// Premultiplies or unpremultiplies one channel by an alpha, rounded.
fn scale_channel(channel: u8, numerator: u16, denominator: u16) -> u8 {
    if denominator == 0 {
        return 0;
    }
    let value = (u16::from(channel) * numerator + denominator / 2) / denominator;
    u8::try_from(value).unwrap_or(u8::MAX)
}

/// A resource's pixels, straight RGBA, as [`decoded`] keeps them; for the HUD's rasteriser.
#[must_use]
pub fn pixels(resource: &'static str) -> Option<Arc<RgbaImage>> {
    decoded(resource)
}

/// `source` resampled to `width` by `height`, in the BGRA gpui draws.
fn render_image(source: &RgbaImage, width: u32, height: u32) -> Arc<RenderImage> {
    let mut pixels = resampled(source, width, height);
    // gpui's frames are BGRA in an `RgbaImage`, as `mapview.rs`'s tiles are.
    for pixel in pixels.pixels_mut() {
        pixel.0.swap(0, 2);
    }
    Arc::new(RenderImage::new(vec![image::Frame::new(pixels)]))
}

/// `source` resampled to `width` by `height`, straight RGBA: the copy [`paint`] hands the GPU,
/// and the one the HUD's rasteriser draws. Resampled with its alpha premultiplied, so a
/// transparent pixel's colour does not bleed into the edge beside it.
#[must_use]
pub fn resampled(source: &RgbaImage, width: u32, height: u32) -> RgbaImage {
    if source.width() == width && source.height() == height {
        source.clone()
    } else {
        let mut premultiplied = source.clone();
        for pixel in premultiplied.pixels_mut() {
            let [r, g, b, a] = pixel.0;
            let alpha = u16::from(a);
            pixel.0 = [
                scale_channel(r, alpha, 255),
                scale_channel(g, alpha, 255),
                scale_channel(b, alpha, 255),
                a,
            ];
        }
        let mut resized = image::imageops::resize(
            &premultiplied,
            width,
            height,
            image::imageops::FilterType::Triangle,
        );
        for pixel in resized.pixels_mut() {
            let [r, g, b, a] = pixel.0;
            let alpha = u16::from(a);
            pixel.0 = [
                scale_channel(r, 255, alpha),
                scale_channel(g, 255, alpha),
                scale_channel(b, 255, alpha),
                a,
            ];
        }
        resized
    }
}

/// `image` turned `degrees` clockwise about `pivot` - a point in its pixels - into a square whose
/// centre is the pivot and which holds the image at any angle: what `RotateTransform(heading)`
/// then `DrawImage` at the pivot's negative draws, for a map marker's icon. Each pixel of the
/// square is the image sampled bilinearly, alpha premultiplied, where that pixel was before the
/// turn, and nothing where that is outside the image. Straight RGBA; the side comes back too.
#[must_use]
#[allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss
)]
pub fn turned(image: &RgbaImage, pivot: (f32, f32), degrees: f32) -> (RgbaImage, u32) {
    let (w, h) = (image.width() as f32, image.height() as f32);
    let reach = [(0.0, 0.0), (w, 0.0), (0.0, h), (w, h)]
        .iter()
        .map(|(x, y)| (x - pivot.0).hypot(y - pivot.1))
        .fold(0.0_f32, f32::max);
    let side = (reach * 2.0).ceil() as u32 + 2;
    let (sin, cos) = degrees.to_radians().sin_cos();
    let centre = side as f32 / 2.0;
    let mut out = RgbaImage::new(side, side);
    for (dx, dy, pixel) in out.enumerate_pixels_mut() {
        let u = dx as f32 + 0.5 - centre;
        let v = dy as f32 + 0.5 - centre;
        // The turn undone: where this pixel of the square was in the image. A positive angle
        // turns clockwise with y downward, as GDI+'s does.
        let sx = v.mul_add(sin, u * cos) + pivot.0 - 0.5;
        let sy = v.mul_add(cos, -u * sin) + pivot.1 - 0.5;
        *pixel = image::Rgba(sample(image, sx, sy));
    }
    (out, side)
}

/// The image at (`x`, `y`), between its pixels: the four around the point weighed by distance,
/// each colour weighed by its alpha as well, so a clear pixel lends no colour to its neighbour;
/// clear outside the image.
#[allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss
)]
fn sample(image: &RgbaImage, x: f32, y: f32) -> [u8; 4] {
    let (x0, y0) = (x.floor(), y.floor());
    let (fx, fy) = (x - x0, y - y0);
    let mut sum = [0.0_f32; 4];
    for (ix, wx) in [(x0, 1.0 - fx), (x0 + 1.0, fx)] {
        for (iy, wy) in [(y0, 1.0 - fy), (y0 + 1.0, fy)] {
            let weight = wx * wy;
            if weight <= 0.0
                || ix < 0.0
                || iy < 0.0
                || ix >= image.width() as f32
                || iy >= image.height() as f32
            {
                continue;
            }
            let [r, g, b, a] = image.get_pixel(ix as u32, iy as u32).0;
            let alpha = f32::from(a) / 255.0 * weight;
            sum[0] += f32::from(r) * alpha;
            sum[1] += f32::from(g) * alpha;
            sum[2] += f32::from(b) * alpha;
            sum[3] += f32::from(a) * weight;
        }
    }
    if sum[3] <= 0.0 {
        return [0; 4];
    }
    let alpha = sum[3] / 255.0;
    let channel = |value: f32| (value / alpha).round().clamp(0.0, 255.0) as u8;
    [
        channel(sum[0]),
        channel(sum[1]),
        channel(sum[2]),
        sum[3].round().clamp(0.0, 255.0) as u8,
    ]
}

/// A resource drawn `width` by `height` logical pixels with its point `pivot` on a marker's
/// point and the whole turned `degrees` clockwise about it: the image to paint, and the side of
/// the square it fills, in logical pixels. Made for each whole degree on first use and kept, so
/// gpui's atlas keeps each; `None` when the resource is not carried or does not decode.
#[must_use]
#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
pub fn rotated(
    resource: &'static str,
    (width, height): (f32, f32),
    pivot: (f32, f32),
    degrees: f32,
    scale: f32,
) -> Option<(Arc<RenderImage>, f32)> {
    type Key = (&'static str, u32, u32, i32);
    type Turned = HashMap<Key, (Arc<RenderImage>, u32)>;
    static ROTATED: OnceLock<Mutex<Turned>> = OnceLock::new();
    let turn = (degrees.rem_euclid(360.0).round() as i32) % 360;
    let (device_width, device_height) = (device(width * scale), device(height * scale));
    let source = decoded(resource)?;
    let mut cache = ROTATED.get_or_init(Mutex::default).lock().ok()?;
    let (render, side) = cache
        .entry((resource, device_width, device_height, turn))
        .or_insert_with(|| {
            let base = resampled(&source, device_width, device_height);
            let (mut pixels, side) = turned(&base, (pivot.0 * scale, pivot.1 * scale), turn as f32);
            // gpui's frames are BGRA in an `RgbaImage`, as `render_image` makes them.
            for pixel in pixels.pixels_mut() {
                pixel.0.swap(0, 2);
            }
            (
                Arc::new(RenderImage::new(vec![image::Frame::new(pixels)])),
                side,
            )
        })
        .clone();
    #[allow(clippy::cast_precision_loss)] // a square's side in pixels
    let logical = side as f32 / scale.max(f32::EPSILON);
    Some((render, logical))
}

/// A resource at a size in device pixels, made on first use and kept, so its `ImageId` - which
/// is what gpui's sprite atlas keys on - is the same every frame.
fn sized(resource: &'static str, width: u32, height: u32) -> Option<Arc<RenderImage>> {
    type Key = (&'static str, u32, u32);
    static SIZED: OnceLock<Mutex<HashMap<Key, Arc<RenderImage>>>> = OnceLock::new();
    let source = decoded(resource)?;
    let mut cache = SIZED.get_or_init(Mutex::default).lock().ok()?;
    Some(Arc::clone(
        cache
            .entry((resource, width, height))
            .or_insert_with(|| render_image(&source, width, height)),
    ))
}

/// A length in whole device pixels, at least one.
#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // rounded, and clamped
fn device(length: f32) -> u32 {
    length.round().clamp(1.0, 16_384.0) as u32
}

/// Where WinForms draws an image of `image` size in a box of `area` size, relative to the box's
/// top left corner: `(x, y, width, height)`, one rectangle for each copy - several for
/// [`Layout::Tile`]. The box clips them. Whole pixels, as WinForms' integer `Rectangle`s are.
///
/// `ImageLayout` is `ControlPaint.CalculateBackgroundImageRectangle`; `PictureBoxSizeMode.Zoom`
/// is `PictureBox.ImageRectangleFromSizeMode`, which truncates where the other rounds.
#[must_use]
pub fn placements(layout: Layout, area: (f32, f32), image: (f32, f32)) -> Vec<Place> {
    let (width, height) = (area.0.trunc(), area.1.trunc());
    let (image_width, image_height) = image;
    if image_width <= 0.0 || image_height <= 0.0 || width <= 0.0 || height <= 0.0 {
        return Vec::new();
    }
    match layout {
        Layout::None => vec![(0.0, 0.0, image_width, image_height)],
        Layout::Tile => {
            let mut copies = Vec::new();
            let mut y = 0.0;
            while y < height {
                let mut x = 0.0;
                while x < width {
                    copies.push((x, y, image_width, image_height));
                    x += image_width;
                }
                y += image_height;
            }
            copies
        }
        Layout::Center => {
            let x = if width > image_width {
                ((width - image_width) / 2.0).trunc()
            } else {
                0.0
            };
            let y = if height > image_height {
                ((height - image_height) / 2.0).trunc()
            } else {
                0.0
            };
            vec![(x, y, image_width, image_height)]
        }
        Layout::Stretch => vec![(0.0, 0.0, width, height)],
        Layout::Zoom => {
            let x_ratio = width / image_width;
            let y_ratio = height / image_height;
            if x_ratio < y_ratio {
                let zoomed = (image_height * x_ratio + 0.5).trunc();
                vec![(0.0, ((height - zoomed) / 2.0).trunc(), width, zoomed)]
            } else {
                let zoomed = (image_width * y_ratio + 0.5).trunc();
                vec![(((width - zoomed) / 2.0).trunc(), 0.0, zoomed, height)]
            }
        }
        Layout::ZoomImage => {
            let ratio = (width / image_width).min(height / image_height);
            let zoomed_width = (image_width * ratio).trunc();
            let zoomed_height = (image_height * ratio).trunc();
            vec![(
                ((width - zoomed_width) / 2.0).trunc(),
                ((height - zoomed_height) / 2.0).trunc(),
                zoomed_width,
                zoomed_height,
            )]
        }
    }
}

/// Paints a resource into `bounds` by its layout, clipped to them.
#[allow(clippy::cast_precision_loss)] // an image's side is a few thousand pixels at most
fn paint(
    resource: &'static str,
    source: &RgbaImage,
    layout: Layout,
    bounds: Bounds<Pixels>,
    window: &mut Window,
) {
    let scale = window.scale_factor();
    let area = (f32::from(bounds.size.width), f32::from(bounds.size.height));
    let image = (source.width() as f32, source.height() as f32);
    for (x, y, width, height) in placements(layout, area, image) {
        let Some(render) = sized(resource, device(width * scale), device(height * scale)) else {
            return;
        };
        let target = Bounds {
            origin: point(bounds.origin.x + px(x), bounds.origin.y + px(y)),
            size: size(px(width), px(height)),
        };
        let _ = window.paint_image(bounds, target, Corners::default(), render, 0, false);
    }
}

/// Paints a resource stretched to `target`, as GDI+'s `DrawImage(image, x, y, width, height)`
/// draws it, clipped to `clip`; `false`, painting nothing, when it is not carried or does not
/// decode. For a picture drawn in a canvas rather than in a box of its own: the HUD's.
pub fn paint_stretched(
    resource: &'static str,
    clip: Bounds<Pixels>,
    target: Bounds<Pixels>,
    window: &mut Window,
) -> bool {
    if decoded(resource).is_none() {
        return false;
    }
    let scale = window.scale_factor();
    let (width, height) = (f32::from(target.size.width), f32::from(target.size.height));
    if let Some(render) = sized(resource, device(width * scale), device(height * scale)) {
        let _ = window.paint_image(clip, target, Corners::default(), render, 0, false);
    }
    true
}

/// A resource drawn by its layout over the whole of its parent, which must have a size; `None`
/// when the resource is not carried or does not decode.
#[must_use]
pub fn image(resource: &'static str, layout: Layout) -> Option<AnyElement> {
    let source = decoded(resource)?;
    Some(
        canvas(
            |_bounds, _window, _cx| (),
            move |bounds, (), window, _cx| paint(resource, &source, layout, bounds, window),
        )
        .size_full()
        .into_any_element(),
    )
}

/// An `ImageLabel`'s `PictureBox`, docked to fill what its label leaves in a flex column: the
/// resource zoomed, or the name where it is not carried. The fact `config.<page>.picture.<id>`
/// names the resource.
/// `// C#: ExtLibs/Controls/ImageLabel.Designer.cs:37-45`
#[must_use]
pub fn image_label(
    page: &str,
    id: &str,
    resource: &'static str,
    name: &'static str,
    colour: u32,
) -> AnyElement {
    let drawn = image(resource, Layout::ZoomImage);
    record(page, id, drawn.as_ref().map(|_| resource));
    let fill = div().absolute().inset_0();
    let fill = match drawn {
        Some(element) => fill.child(element),
        None => fill
            .flex()
            .items_center()
            .justify_center()
            .text_lg()
            .text_color(rgb(colour))
            .child(name),
    };
    div().flex_1().relative().child(fill).into_any_element()
}

/// Records `config.<page>.picture.<id>` = the resource drawn there, or `none` for a named box.
pub fn record(page: &str, id: &str, drawn: Option<&str>) {
    if crate::facts::enabled() {
        crate::facts::record(
            format!("config.{page}.picture.{id}"),
            drawn.unwrap_or("none"),
        );
    }
}

/// What a page drew before it carried the image, and still draws when a resource is missing:
/// the box with a name in it.
#[must_use]
pub fn named(body: Div, name: impl Into<gpui::SharedString>) -> Div {
    body.flex()
        .items_center()
        .justify_center()
        .border_1()
        .border_color(rgb(theme::BORDER))
        .rounded_sm()
        .text_xs()
        .text_color(rgb(theme::DIM))
        .child(name.into())
}

/// An absolutely placed box.
fn at((x, y, width, height): Place) -> Div {
    div()
        .absolute()
        .left(px(x))
        .top(px(y))
        .w(px(width))
        .h(px(height))
}

/// A `PictureBox` at its place showing one resource, its `BackgroundImage` or its `Image`, by
/// the layout the `.resx` gives it; the fact `config.<page>.picture.<id>` names the resource.
#[must_use]
pub fn picture(
    page: &str,
    id: &'static str,
    place: Place,
    resource: &'static str,
    layout: Layout,
) -> AnyElement {
    let body = crate::probe::measured(id, at(place));
    let element = image(resource, layout);
    record(page, id, element.as_ref().map(|_| resource));
    match element {
        Some(element) => body.child(element).into_any_element(),
        None => named(body, id).into_any_element(),
    }
}

/// A `PictureBox` with both a `BackgroundImage` and an `Image`: WinForms paints the background
/// first and the image over it. The fact names the image, and `.background` the background.
#[must_use]
pub fn layered(
    page: &str,
    id: &'static str,
    place: Place,
    (background, background_layout): (&'static str, Layout),
    (front, front_layout): (&'static str, Layout),
) -> AnyElement {
    let back = image(background, background_layout);
    let over = image(front, front_layout);
    record(
        page,
        &format!("{id}.background"),
        back.as_ref().map(|_| background),
    );
    record(page, id, over.as_ref().map(|_| front));
    if back.is_none() && over.is_none() {
        return named(crate::probe::measured(id, at(place)), id).into_any_element();
    }
    let layer = |element: AnyElement| div().absolute().size_full().child(element);
    crate::probe::measured(id, at(place))
        .children(back.map(layer))
        .children(over.map(layer))
        .into_any_element()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config_coverage::source::csharp;

    /// An image turned about a pivot: a 4 by 4 with its one red pixel top right, turned 90
    /// degrees clockwise about its centre, has the pixel bottom right; the square holds the
    /// image at any angle; an image turned 0 is itself, centred.
    #[test]
    fn an_image_turns_clockwise_about_its_pivot() {
        let mut image = RgbaImage::new(4, 4);
        for pixel in image.pixels_mut() {
            *pixel = image::Rgba([0, 0, 255, 255]);
        }
        image.put_pixel(3, 0, image::Rgba([255, 0, 0, 255]));
        let (same, side) = turned(&image, (2.0, 2.0), 0.0);
        assert!(side >= 6 && side % 2 == 0, "{side}");
        let offset = (side - 4) / 2;
        assert_eq!(same.get_pixel(offset + 3, offset).0, [255, 0, 0, 255]);
        assert_eq!(same.get_pixel(offset, offset).0, [0, 0, 255, 255]);
        assert_eq!(same.get_pixel(0, 0).0, [0, 0, 0, 0], "clear outside");
        let (quarter, _) = turned(&image, (2.0, 2.0), 90.0);
        assert_eq!(
            quarter.get_pixel(offset + 3, offset + 3).0,
            [255, 0, 0, 255]
        );
        assert_eq!(quarter.get_pixel(offset + 3, offset).0, [0, 0, 255, 255]);
        // The marker icons the flight map turns are carried.
        for resource in ["rover", "boat", "heli", "sub", "redsinglecopter2"] {
            let (render, side) =
                rotated(resource, (20.0, 20.0), (10.0, 10.0), 45.0, 1.0).expect(resource);
            assert!(side >= 28.0, "{resource}: {side}");
            assert_eq!(render.size(0).width.0, render.size(0).height.0);
        }
        assert!(rotated("no_such_resource", (20.0, 20.0), (10.0, 10.0), 0.0, 1.0).is_none());
    }

    /// Every carried image decodes, from the file its name says, and is carried once.
    #[test]
    fn every_embedded_image_decodes() {
        let mut total = 0;
        for embedded in IMAGES {
            let image = image::load_from_memory(embedded.bytes)
                .unwrap_or_else(|error| panic!("{} does not decode: {error}", embedded.file));
            assert!(image.width() > 0 && image.height() > 0, "{}", embedded.file);
            assert!(
                embedded
                    .file
                    .starts_with(&format!("{}.", embedded.resource)),
                "{} is not named for {}",
                embedded.file,
                embedded.resource
            );
            assert!(
                decoded(embedded.resource).is_some(),
                "{}",
                embedded.resource
            );
            total += embedded.bytes.len();
        }
        let mut names: Vec<&str> = IMAGES.iter().map(|image| image.resource).collect();
        names.sort_unstable();
        names.dedup();
        assert_eq!(names.len(), IMAGES.len(), "a resource carried twice");
        assert_eq!(total, 686_349, "the images embedded, in bytes");
        assert!(decoded("no_such_resource").is_none());
    }

    /// `assets/images` holds exactly the carried images: nothing unused is shipped beside them.
    #[test]
    fn assets_images_holds_what_is_embedded() {
        let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../assets/images");
        let mut files: Vec<String> = std::fs::read_dir(&dir)
            .map(|entries| {
                entries
                    .filter_map(Result::ok)
                    .map(|entry| entry.file_name().to_string_lossy().into_owned())
                    .collect()
            })
            .unwrap_or_default();
        files.sort();
        let mut embedded: Vec<String> = IMAGES.iter().map(|image| image.file.to_owned()).collect();
        embedded.sort();
        assert_eq!(files, embedded);
    }

        /// Every site's resource is carried, or is one of the photographs left out; every carried
    /// image is some site's: nothing the Designers do not assign; and no photograph is carried.
    #[test]
    fn every_site_is_carried_and_every_image_used() {
        for site in SITES {
            assert!(
                bytes(site.resource).is_some() || PHOTOGRAPHS.contains(&site.resource),
                "{}'s {} is neither carried nor a photograph left out",
                site.control,
                site.resource
            );
        }
        for photograph in PHOTOGRAPHS {
            assert!(
                bytes(photograph).is_none(),
                "{photograph} is a product photograph and is carried"
            );
            assert!(
                SITES.iter().any(|site| site.resource == photograph),
                "{photograph} is no Designer's"
            );
        }

        for icon in crate::hud::Icon::ALL {
            assert!(bytes(icon.resource()).is_some(), "{icon:?} is not carried");
        }
        for kind in crate::mapview::MarkerKind::ALL {
            if let Some((resource, _)) = kind.icon() {
                assert!(bytes(resource).is_some(), "{kind:?} is not carried");
            }
        }
        for embedded in IMAGES {
            assert!(
                SITES.iter().any(|site| site.resource == embedded.resource)
                    || crate::hud::Icon::ALL
                        .iter()
                        .any(|icon| icon.resource() == embedded.resource)
                    || crate::mapview::MarkerKind::ALL
                        .iter()
                        .any(|kind| kind.icon().is_some_and(|(r, _)| r == embedded.resource))
                    || embedded.resource == crate::config::proximity::QUAD_ICON,
                "{} is carried but no site, HUD picture, map marker or the proximity window shows it",
                embedded.resource
            );
        }
    }

    /// SITL's pictures are its Designer's: each vehicle's `ImageNormal` and `ImageOver`, drawn at
    /// `SizeMode.Normal` as its sites have them; and the `.resx`'s `Image`, which the Designer's
    /// `selected = false` replaces with `ImageNormal` before the page is shown, is the same bytes.
    /// `// C#: GCSViews/SITL.Designer.cs:137-179; ExtLibs/Controls/PictureBoxMouseOver.cs:17, 37-50`
    #[test]
    fn sitls_pictures_are_its_designers() {
        use crate::sitl::model::Vehicle;
        let find = |control: &str, property: &str| {
            SITES
                .iter()
                .find(|site| {
                    site.designer == SITL && site.control == control && site.property == property
                })
                .map(|site| (site.resource, site.layout))
        };
        for vehicle in Vehicle::ALL {
            let control = vehicle.control();
            assert_eq!(
                find(control, "ImageNormal"),
                Some((vehicle.image(false), crate::sitl::view::PICTURE_LAYOUT)),
                "{control}"
            );
            assert_eq!(
                find(control, "ImageOver"),
                Some((vehicle.image(true), crate::sitl::view::PICTURE_LAYOUT)),
                "{control}"
            );
        }
        let Some(resx) = csharp("GCSViews/SITL.resx") else {
            eprintln!("skipped: the C# tree is not checked out here");
            return;
        };
        let values = crate::config_coverage::source::resx(&resx);
        for vehicle in Vehicle::ALL {
            let control = vehicle.control();
            let squeeze = |key: String| -> String {
                values
                    .get(&key)
                    .map(|text| text.split_whitespace().collect())
                    .unwrap_or_default()
            };
            assert_eq!(
                squeeze(format!("{control}.Image")),
                squeeze(format!("{control}.ImageNormal")),
                "{control}"
            );
        }
    }

    /// Heli Setup's Designer assigns no picture, so the page draws none: the gauge background
    /// and the `HS1`..`HS4` bars are the older `ConfigTradHeli`'s, which Initial Setup no longer
    /// lists. `// C#: GCSViews/ConfigurationView/ConfigTradHeli.Designer.cs:923;
    /// GCSViews/InitialSetup.cs:186-187`
    #[test]
    fn heli_setups_designer_has_no_picture() {
        let Some(designer) = csharp("GCSViews/ConfigurationView/ConfigTradHeli4.Designer.cs")
        else {
            eprintln!("skipped: the C# tree is not checked out here");
            return;
        };
        for line in designer.lines() {
            assert!(
                !line.contains("Image =") && !line.contains("Properties.Resources."),
                "ConfigTradHeli4.Designer.cs: {line}"
            );
        }
        let older =
            csharp("GCSViews/ConfigurationView/ConfigTradHeli.Designer.cs").unwrap_or_default();
        assert!(older.contains(
            "this.Gservoloc.BackgroundImage = global::MissionPlanner.Properties.Resources.Gaugebg;"
        ));
    }

    /// Every picture `HUD.cs` draws is an [`crate::hud::Icon`], carried as the `HUDT` resource
    /// it names: its file through `HUDT.Designer.cs` and `HUDT.resx`, byte for byte.
    /// `// C#: ExtLibs/Controls/HUD.cs:2861-2893, 2931-3008, 3173-3295`
    #[test]
    fn the_huds_pictures_are_hudts() {
        let (Some(hud), Some(hudt), Some(hudt_resx)) = (
            csharp("ExtLibs/Controls/HUD.cs"),
            csharp("ExtLibs/Controls/HUDT.Designer.cs"),
            csharp("ExtLibs/Controls/HUDT.resx"),
        ) else {
            eprintln!("skipped: the C# tree is not checked out here");
            return;
        };
        let Some(root) = crate::config_coverage::source::csharp_root() else {
            return;
        };
        let resources: Vec<&str> = crate::hud::Icon::ALL
            .iter()
            .map(|icon| icon.resource())
            .collect();
        // Every `HUDT.<name>` HUD.cs hands `DrawImage` or keeps as an `icon`.
        let mut drawn: Vec<&str> = Vec::new();
        for (index, _) in hud.match_indices("HUDT.") {
            let name: &str = hud
                .get(index + "HUDT.".len()..)
                .and_then(|rest| {
                    let end = rest
                        .find(|c: char| !(c.is_ascii_alphanumeric() || c == '_'))
                        .unwrap_or(rest.len());
                    rest.get(..end)
                })
                .unwrap_or_default();
            let line_start = hud.get(..index).and_then(|before| before.rfind('\n'));
            let line = hud
                .get(line_start.map_or(0, |at| at + 1)..)
                .and_then(|rest| rest.lines().next())
                .unwrap_or_default();
            if (line.contains("DrawImage(") || line.contains("icon = "))
                && !line.trim_start().starts_with("//")
                && !drawn.contains(&name)
            {
                drawn.push(name);
            }
        }
        drawn.sort_unstable();
        let mut ours = resources.clone();
        ours.sort_unstable();
        assert_eq!(drawn, ours, "HUD.cs's pictures");
        for resource in resources {
            let file = resource_file(&hudt, &hudt_resx, resource)
                .unwrap_or_else(|| panic!("{resource} is not in HUDT.resx"));
            let on_disk = std::fs::read(root.join("ExtLibs/Controls").join(&file))
                .unwrap_or_else(|_| panic!("{file} is not in the C# tree"));
            assert!(
                bytes(resource) == Some(on_disk.as_slice()),
                "the carried {resource} is not {file}"
            );
        }
    }

    /// The pictures drawn with `picture` or `layered` in the pages, as their source has them:
    /// every string literal followed by a `Layout`, with that layout.
    fn call_sites() -> Vec<(String, String, String)> {
        let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("src/config");
        let mut found = Vec::new();
        let mut files: Vec<_> = std::fs::read_dir(&dir)
            .map(|entries| entries.filter_map(Result::ok).map(|e| e.path()).collect())
            .unwrap_or_default();
        files.sort();
        for path in files {
            let Ok(source) = std::fs::read_to_string(&path) else {
                continue;
            };
            let file = path
                .file_name()
                .map(|name| name.to_string_lossy().into_owned())
                .unwrap_or_default();
            for (index, _) in source.match_indices("Layout::") {
                let before = source.get(..index).unwrap_or_default();
                let before = before.strip_suffix("crate::pictures::").unwrap_or(before);
                let Some(before) = before.trim_end().strip_suffix("\",") else {
                    continue;
                };
                let Some(open) = before.rfind('"') else {
                    continue;
                };
                let literal = before.get(open + 1..).unwrap_or_default().to_owned();
                let layout: String = source
                    .get(index + "Layout::".len()..)
                    .unwrap_or_default()
                    .chars()
                    .take_while(char::is_ascii_alphanumeric)
                    .collect();
                found.push((file.clone(), literal, layout));
            }
        }
        found
    }

        /// Every resource a page names at a `picture` call is carried, or is a photograph left out,
    /// and is the one its Designer assigns with the layout its `.resx` gives. The list is the call
    /// sites, file by file.
    #[test]
    fn every_call_site_resource_is_carried() {
        let sites = call_sites();
        let listed: Vec<(&str, &str, &str)> = sites
            .iter()
            .map(|(file, resource, layout)| (file.as_str(), resource.as_str(), layout.as_str()))
            .collect();
        assert_eq!(
            listed,
            [
                ("airspeed.rs", "airspeed", "Zoom"),
                ("battery_monitor.rs", "BR_APMPWRDEAN_2", "Zoom"),
                ("battery_monitor2.rs", "BR_APMPWRDEAN_2", "Zoom"),
                ("compass.rs", "up", "Center"),
                ("compass.rs", "down", "Center"),
                ("mount.rs", "cameraGimalPitch1", "Zoom"),
                ("mount.rs", "cameraGimalRoll1", "Zoom"),
                ("mount.rs", "cameraGimalYaw", "Zoom"),
                ("mount.rs", "Shutter", "Zoom"),
                ("optical_flow.rs", "opticalflow", "Zoom"),
                ("osd.rs", "MinimOSD", "Zoom"),
                ("parachute.rs", "sonar", "Zoom"),
                ("parachute.rs", "Parachute", "ZoomImage"),
                ("rangefinder.rs", "sonar", "Zoom"),
            ]
        );
                for (file, resource, layout) in &sites {
            assert!(
                bytes(resource).is_some() || PHOTOGRAPHS.contains(&resource.as_str()),
                "{file}: {resource} is neither carried nor a photograph left out"
            );
            assert!(
                SITES.iter().any(
                    |site| site.resource == resource && format!("{:?}", site.layout) == *layout
                ),
                "{file}: {resource} with {layout} is no Designer's"
            );
        }
    }

    /// The tables the Frame Type and Install Firmware pages draw from name the Designers'
    /// resources for their controls.
    #[test]
    fn the_page_tables_name_the_designers_resources() {
        let find = |designer: &str, control: &str| {
            SITES
                .iter()
                .find(|site| site.designer == designer && site.control == control)
                .map(|site| (site.resource, site.layout))
        };
        for button in crate::config::frame_type::CLASSES {
            let control = format!("radioButton{}", button.text);
            let expected = find(FRAME, &control).map(|(resource, _)| resource);
            // The Undefined button, `radioButtonUndef`, has no image.
            assert_eq!(button.image, expected, "{control}");
        }
        for option in crate::config::frame_type::TYPES {
            let control = format!("pictureBox{}", option.name);
            assert_eq!(
                option.image,
                find(FRAME, &control).map(|(r, _)| r),
                "{control}"
            );
        }
        for row in crate::config::frame_type_legacy::ROWS {
            let control = format!("pictureBox{}", row.name);
            assert_eq!(
                row.image,
                find(FRAME_LEGACY, &control).map(|(r, _)| r),
                "{control}"
            );
        }
        for picture in crate::config::firmware::PICTURES {
            assert_eq!(
                find(FIRMWARE, picture.control),
                Some((picture.image, Layout::ZoomImage)),
                "{}",
                picture.control
            );
        }
        assert_eq!(
            find(FIRMWARE, "imageLabel1"),
            Some((crate::config::firmware::CUBE_IMAGE, Layout::ZoomImage))
        );
        for picture in crate::config::firmware_legacy::PICTURES {
            assert_eq!(
                find(FIRMWARE_LEGACY, picture.control),
                Some((picture.image, Layout::ZoomImage)),
                "{}",
                picture.control
            );
        }
        assert_eq!(
            find(FIRMWARE_LEGACY, "imageLabel1"),
            Some((
                crate::config::firmware_legacy::CUBE_IMAGE,
                Layout::ZoomImage
            ))
        );
    }

    /// WinForms' rectangles, on a few sizes.
    #[test]
    fn the_layout_arithmetic_is_winforms() {
        // MinimOSD, 404 x 183, in OSD's 75 x 75 box: the width limits it; 183 * 75/404 = 33.97,
        // rounded to 34, centred at (75 - 34) / 2 = 20.
        assert_eq!(
            placements(Layout::Zoom, (75.0, 75.0), (404.0, 183.0)),
            [(0.0, 20.0, 75.0, 34.0)]
        );
        // The sonar, 156 x 195, in Parachute's 75 x 69: the height limits it; 156 * 69/195 =
        // 55.2, 55, centred at 10.
        assert_eq!(
            placements(Layout::Zoom, (75.0, 69.0), (156.0, 195.0)),
            [(10.0, 0.0, 55.0, 69.0)]
        );
        // The parachute, 65 x 80, as an `Image`: ratio 69/80, 56.06 truncated to 56, centred.
        assert_eq!(
            placements(Layout::ZoomImage, (75.0, 69.0), (65.0, 80.0)),
            [(9.0, 0.0, 56.0, 69.0)]
        );
        // frames_x, 1000 x 175, in its 406 x 78 box: 406/1000 is the smaller ratio, so 406
        // wide and 175 * 0.406 = 71.05, 71 high, 3 down.
        assert_eq!(
            placements(Layout::ZoomImage, (406.0, 78.0), (1000.0, 175.0)),
            [(0.0, 3.0, 406.0, 71.0)]
        );
        // A background rounds where an `Image` truncates: 177 * 0.406 = 71.86 is 72, not 71.
        assert_eq!(
            placements(Layout::Zoom, (406.0, 78.0), (1000.0, 177.0)),
            [(0.0, 3.0, 406.0, 72.0)]
        );
        assert_eq!(
            placements(Layout::ZoomImage, (406.0, 78.0), (1000.0, 177.0)),
            [(0.0, 3.0, 406.0, 71.0)]
        );
        // Stretch fills, whatever the image.
        assert_eq!(
            placements(Layout::Stretch, (60.0, 60.0), (300.0, 300.0)),
            [(0.0, 0.0, 60.0, 60.0)]
        );
        // Center: centred where the box is larger, at the edge where it is not.
        assert_eq!(
            placements(Layout::Center, (75.0, 75.0), (65.0, 80.0)),
            [(5.0, 0.0, 65.0, 80.0)]
        );
        assert_eq!(
            placements(Layout::None, (75.0, 75.0), (65.0, 80.0)),
            [(0.0, 0.0, 65.0, 80.0)]
        );
        // Tile: copies from the corner until the box is covered.
        assert_eq!(
            placements(Layout::Tile, (100.0, 50.0), (40.0, 30.0)),
            [
                (0.0, 0.0, 40.0, 30.0),
                (40.0, 0.0, 40.0, 30.0),
                (80.0, 0.0, 40.0, 30.0),
                (0.0, 30.0, 40.0, 30.0),
                (40.0, 30.0, 40.0, 30.0),
                (80.0, 30.0, 40.0, 30.0),
            ]
        );
        // Nothing to draw in an empty box, or of an empty image.
        assert!(placements(Layout::Zoom, (0.0, 75.0), (65.0, 80.0)).is_empty());
        assert!(placements(Layout::Tile, (75.0, 75.0), (0.0, 80.0)).is_empty());
    }

    /// A resized copy keeps an opaque colour and does not darken a transparent edge.
    #[test]
    fn resampling_keeps_colours() {
        let mut source = RgbaImage::new(4, 4);
        for (x, _, pixel) in source.enumerate_pixels_mut() {
            // Left half opaque red, right half transparent black.
            pixel.0 = if x < 2 {
                [255, 0, 0, 255]
            } else {
                [0, 0, 0, 0]
            };
        }
        let image = render_image(&source, 2, 2);
        let frame = image.as_bytes(0).unwrap_or_default();
        // BGRA: the left column stays red wherever it has any alpha.
        let first = frame.get(..4).unwrap_or_default();
        assert_eq!(first.get(2), Some(&255), "red kept: {first:?}");
        assert_eq!(first.first(), Some(&0), "no blue: {first:?}");
    }

    /// `Properties.Resources.<name>`'s file, through `Resources.Designer.cs` (the name it gets)
    /// and `Resources.resx` (the file that name is).
    fn resource_file(designer: &str, resx: &str, property: &str) -> Option<String> {
        let marker = format!("Bitmap {property} {{");
        let after = designer.split_once(&marker)?.1;
        let key = after.split_once("GetObject(\"")?.1.split_once('"')?.0;
        let entry = resx.split_once(&format!("<data name=\"{key}\""))?.1;
        let value = entry.split_once("<value>")?.1.split_once(';')?.0;
        Some(value.trim_start_matches("..\\").replace('\\', "/"))
    }

    /// Every site is its Designer's line, with the layout its `.resx` gives and the bytes its
    /// resource's file holds; and every `Properties.Resources` image each Designer assigns is a
    /// site. Skipped when the C# tree is not checked out.
    #[test]
    fn the_sites_are_the_designers() {
        let (Some(resources), Some(resources_resx)) = (
            csharp("Properties/Resources.Designer.cs"),
            csharp("Properties/Resources.resx"),
        ) else {
            eprintln!("skipped: the C# tree is not checked out here");
            return;
        };
        let Some(root) = crate::config_coverage::source::csharp_root() else {
            return;
        };
        let view = "GCSViews/ConfigurationView";
        // A Designer's path in the tree: under `ConfigurationView` unless it names its own.
        let path = |designer: &str| {
            if designer.contains('/') {
                designer.to_owned()
            } else {
                format!("{view}/{designer}")
            }
        };
        let image_label = csharp("ExtLibs/Controls/ImageLabel.Designer.cs").unwrap_or_default();
        let frame_class = csharp(&format!("{view}/ConfigFrameClassType.cs")).unwrap_or_default();
        for site in SITES {
            let designer = csharp(&path(site.designer))
                .unwrap_or_else(|| panic!("{} is not in the C# tree", site.designer));
            let full = path(site.designer);
            let stem = full.split_once('.').map_or(full.as_str(), |(stem, _)| stem);
            let resx = csharp(&format!("{stem}.resx")).unwrap_or_default();
            let values = crate::config_coverage::source::resx(&resx);
            let file_bytes = match site.from {
                Source::Resources => {
                    let line = format!(
                        "this.{}.{} = global::MissionPlanner.Properties.Resources.{};",
                        site.control, site.property, site.resource
                    );
                    assert!(designer.contains(&line), "{}: no `{line}`", site.designer);
                    let file = resource_file(&resources, &resources_resx, site.resource)
                        .unwrap_or_else(|| panic!("{} is not in Resources.resx", site.resource));
                    std::fs::read(root.join(&file))
                        .unwrap_or_else(|_| panic!("{file} is not in the C# tree"))
                }
                Source::Resx => {
                    use base64::Engine as _;
                    let key = format!("{}.{}", site.control, site.property);
                    // Assigned in the Designer from the `.resx`, or by `ApplyResources`.
                    let line = format!(
                        "this.{key} = ((System.Drawing.Image)(resources.GetObject(\"{key}\")));"
                    );
                    assert!(
                        designer.contains(&line)
                            || designer.contains(&format!(
                                "resources.ApplyResources(this.{}, \"{}\");",
                                site.control, site.control
                            )),
                        "{}: neither `{line}` nor ApplyResources",
                        site.designer
                    );
                    let text: String = values
                        .get(&key)
                        .unwrap_or_else(|| panic!("{stem}.resx has no {key}"))
                        .split_whitespace()
                        .collect();
                    base64::engine::general_purpose::STANDARD
                        .decode(text)
                        .unwrap_or_else(|_| panic!("{stem}.resx's {key} is not base64"))
                }
            };
                        if PHOTOGRAPHS.contains(&site.resource) {
                assert!(
                    bytes(site.resource).is_none(),
                    "{}.{}: the photograph {} is carried",
                    site.control,
                    site.property,
                    site.resource
                );
            } else {
                assert!(
                    bytes(site.resource) == Some(file_bytes.as_slice()),
                    "{}.{}: the carried {} is not the C#'s file",
                    site.control,
                    site.property,
                    site.resource
                );
            }


            // The layout.
            let layout = if site.control.starts_with("radioButton") {
                let resize = format!("{0}.Image = new Bitmap({0}.Image, 60, 60);", site.control);
                assert!(frame_class.contains(&resize), "no `{resize}`");
                Layout::Stretch
            } else if site.designer == FIRMWARE || site.designer == FIRMWARE_LEGACY {
                assert!(designer.contains(&format!(
                    "this.{} = new MissionPlanner.Controls.ImageLabel();",
                    site.control
                )));
                assert!(image_label.contains(
                    "this.PictureBox.SizeMode = System.Windows.Forms.PictureBoxSizeMode.Zoom;"
                ));
                Layout::ZoomImage
            } else if designer.contains(&format!(
                "this.{} = new System.Windows.Forms.DataGridViewImageColumn();",
                site.control
            )) {
                // `DataGridViewImageColumn.ImageLayout` left at `Normal`: centred, unscaled.
                assert!(
                    !designer.contains(&format!("this.{}.ImageLayout", site.control)),
                    "{}: an ImageLayout",
                    site.control
                );
                Layout::Center
            } else if site.property == "BackgroundImage" {
                match values
                    .get(&format!("{}.BackgroundImageLayout", site.control))
                    .map(|value| value.trim())
                {
                    Some("Zoom") => Layout::Zoom,
                    Some("Stretch") => Layout::Stretch,
                    Some("Center") => Layout::Center,
                    Some("None") => Layout::None,
                    _ => Layout::Tile,
                }
            } else {
                match values
                    .get(&format!("{}.SizeMode", site.control))
                    .map(|value| value.trim())
                {
                    Some("Zoom") => Layout::ZoomImage,
                    Some("StretchImage") => Layout::Stretch,
                    Some("CenterImage") => Layout::Center,
                    // `PictureBoxSizeMode.Normal`, the default, which a `.resx` leaves out.
                    None | Some("Normal") => Layout::None,
                    other => panic!("{}: SizeMode {other:?} is not drawn", site.control),
                }
            };
            assert_eq!(layout, site.layout, "{}.{}", site.control, site.property);
        }

        // Every image each of these Designers assigns is a site.
        let mut designers: Vec<&str> = SITES.iter().map(|site| site.designer).collect();
        designers.dedup();
        for name in designers {
            let designer = csharp(&path(name)).unwrap_or_default();
            for line in designer.lines() {
                let Some(assignment) = line.trim().strip_prefix("this.") else {
                    continue;
                };
                // `Properties.Resources.<name>`, or the page's `.resx` by its key.
                if let Some((left, resource)) =
                    assignment.split_once(" = global::MissionPlanner.Properties.Resources.")
                {
                    let resource = resource.trim_end_matches(';');
                    let Some((control, property)) = left.split_once('.') else {
                        continue;
                    };
                    assert!(
                        SITES.iter().any(|site| site.designer == name
                            && site.control == control
                            && site.property == property
                            && site.resource == resource),
                        "{name}: {control}.{property} = {resource} is not a site"
                    );
                } else if let Some((left, _)) =
                    assignment.split_once(" = ((System.Drawing.Image)(resources.GetObject(")
                {
                    let Some((control, property)) = left.split_once('.') else {
                        continue;
                    };
                    assert!(
                        SITES.iter().any(|site| site.designer == name
                            && site.control == control
                            && site.property == property
                            && site.from == Source::Resx),
                        "{name}: {control}.{property} from the .resx is not a site"
                    );
                }
            }
        }
    }
}
