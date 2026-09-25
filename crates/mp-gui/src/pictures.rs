//! Mission Planner's pictures: the `Properties.Resources` images the ported pages' `PictureBox`es,
//! `ImageLabel`s and Frame Class buttons show, drawn in their boxes the way WinForms lays them out.
//!
//! The files are the C# tree's `Resources/*.png|*.jpg` (both trees are GPL-3.0), copied to
//! `assets/images/<resource>.<ext>` under the resource's name - the `Properties.Resources` property
//! the Designer assigns, so `global::MissionPlanner.Properties.Resources.frames_h` is
//! `assets/images/frames_h.png` although `Properties/Resources.resx` points it at
//! `Resources/frames-h.png`. Only the pictures a ported page shows are carried: `SITES`, the
//! tests' table, is every assignment, each held to its Designer line, its `.resx` layout and the
//! file's bytes.
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
    #[cfg_attr(not(test), allow(dead_code))]
    None,
    /// `ImageLayout.Tile`, the `BackgroundImageLayout` WinForms defaults to: unscaled copies
    /// from the top left corner, clipped by the box.
    #[cfg_attr(not(test), allow(dead_code))]
    Tile,
    /// `ImageLayout.Center`: unscaled, centred in each direction the box is larger in, at the
    /// left or top edge in one it is not.
    #[cfg_attr(not(test), allow(dead_code))]
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
/// (the property each resource is).
pub const IMAGES: [Embedded; 32] = [
    embedded!("APM_airframes_001", "APM_airframes_001.png"),
    embedded!("APM_airframes_08", "APM_airframes_08.png"),
    embedded!("Antenna_Tracker_01", "Antenna_Tracker_01.png"),
    embedded!("BR_APMPWRDEAN_2", "BR_APMPWRDEAN_2.jpg"),
    embedded!("FW_icons_2013_logos_03", "FW_icons_2013_logos_03.png"),
    embedded!("FW_icons_2013_logos_04", "FW_icons_2013_logos_04.png"),
    embedded!("FW_icons_2013_logos_06", "FW_icons_2013_logos_06.png"),
    embedded!("FW_icons_2013_logos_07", "FW_icons_2013_logos_07.png"),
    embedded!("FW_icons_2013_logos_08", "FW_icons_2013_logos_08.png"),
    embedded!("FW_icons_2013_logos_09", "FW_icons_2013_logos_09.png"),
    embedded!("FW_icons_2013_logos_10", "FW_icons_2013_logos_10.png"),
    embedded!("FW_icons_2013_logos_12", "FW_icons_2013_logos_12.png"),
    embedded!("FW_icons_2013_logos_13", "FW_icons_2013_logos_13.png"),
    embedded!("MinimOSD", "MinimOSD.jpg"),
    embedded!("Parachute", "Parachute.png"),
    embedded!("Shutter", "Shutter.png"),
    embedded!("airspeed", "airspeed.jpg"),
    embedded!("cameraGimalPitch1", "cameraGimalPitch1.png"),
    embedded!("cameraGimalRoll1", "cameraGimalRoll1.png"),
    embedded!("cameraGimalYaw", "cameraGimalYaw.png"),
    embedded!("frames_h", "frames_h.png"),
    embedded!("frames_plus", "frames_plus.png"),
    embedded!("frames_x", "frames_x.png"),
    embedded!("new_3DR_04", "new_3DR_04.png"),
    embedded!("opticalflow", "opticalflow.jpg"),
    embedded!("pixhawk2cube", "pixhawk2cube.jpg"),
    embedded!("rover_11", "rover_11.png"),
    embedded!("sonar", "sonar.jpg"),
    embedded!("sub", "sub.png"),
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
    /// The Designer, under `GCSViews/ConfigurationView/`.
    pub designer: &'static str,
    /// The control.
    pub control: &'static str,
    /// `Image` or `BackgroundImage`.
    pub property: &'static str,
    /// The resource drawn.
    pub resource: &'static str,
    /// Where the Designer takes it from.
    pub from: Source,
    /// Its layout: the `.resx`'s `BackgroundImageLayout` or `SizeMode`, `ImageLabel`'s inner
    /// `PictureBox`'s `Zoom`, or the Frame Class buttons' 60 by 60 `Bitmap`.
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

/// Every image the ported pages' Designers assign, in each Designer's order.
/// `// C#: GCSViews/ConfigurationView/*.Designer.cs, *.resx`
#[cfg(test)]
#[rustfmt::skip]
pub const SITES: [Site; 53] = [
    site(OSD, "pictureBox5", "BackgroundImage", "MinimOSD", Layout::Zoom),
    site(MOUNT, "pictureBox1", "BackgroundImage", "cameraGimalPitch1", Layout::Zoom),
    site(MOUNT, "pictureBox2", "BackgroundImage", "cameraGimalRoll1", Layout::Zoom),
    site(MOUNT, "pictureBox3", "BackgroundImage", "cameraGimalYaw", Layout::Zoom),
    site(MOUNT, "pictureBox4", "BackgroundImage", "Shutter", Layout::Zoom),
    site("ConfigHWAirspeed.Designer.cs", "pictureBox4", "BackgroundImage", "airspeed", Layout::Zoom),
    site("ConfigHWRangeFinder.Designer.cs", "pictureBox3", "BackgroundImage", "sonar", Layout::Zoom),
    Site { designer: "ConfigBatteryMonitoring.Designer.cs", control: "pictureBox5", property: "BackgroundImage", resource: "BR_APMPWRDEAN_2", from: Source::Resx, layout: Layout::Zoom },
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

/// `source` resampled to `width` by `height`, in the BGRA gpui draws. Resampled with its alpha
/// premultiplied, so a transparent pixel's colour does not bleed into the edge beside it.
fn render_image(source: &RgbaImage, width: u32, height: u32) -> Arc<RenderImage> {
    let mut pixels = if source.width() == width && source.height() == height {
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
    };
    // gpui's frames are BGRA in an `RgbaImage`, as `mapview.rs`'s tiles are.
    for pixel in pixels.pixels_mut() {
        pixel.0.swap(0, 2);
    }
    Arc::new(RenderImage::new(vec![image::Frame::new(pixels)]))
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
        assert_eq!(total, 617_345, "the images embedded, in bytes");
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

    /// Every site's resource is carried, and every carried image is some site's: nothing the
    /// Designers do not assign.
    #[test]
    fn every_site_is_carried_and_every_image_used() {
        for site in SITES {
            assert!(
                bytes(site.resource).is_some(),
                "{}'s {} is not carried",
                site.control,
                site.resource
            );
        }
        for embedded in IMAGES {
            assert!(
                SITES.iter().any(|site| site.resource == embedded.resource),
                "{} is carried but no site shows it",
                embedded.resource
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

    /// Every resource a page names at a `picture` call is carried, and is the one its Designer
    /// assigns with the layout its `.resx` gives. The list is the call sites, file by file.
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
                bytes(resource).is_some(),
                "{file}: {resource} is not carried"
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
        let root = crate::config_coverage::source::csharp_root();
        let view = "GCSViews/ConfigurationView";
        let image_label = csharp("ExtLibs/Controls/ImageLabel.Designer.cs").unwrap_or_default();
        let frame_class = csharp(&format!("{view}/ConfigFrameClassType.cs")).unwrap_or_default();
        for site in SITES {
            let designer = csharp(&format!("{view}/{}", site.designer))
                .unwrap_or_else(|| panic!("{} is not in the C# tree", site.designer));
            let stem = site
                .designer
                .split_once('.')
                .map_or(site.designer, |(stem, _)| stem);
            let resx = csharp(&format!("{view}/{stem}.resx")).unwrap_or_default();
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
            assert!(
                bytes(site.resource) == Some(file_bytes.as_slice()),
                "{}.{}: the carried {} is not the C#'s file",
                site.control,
                site.property,
                site.resource
            );

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
                    other => panic!("{}: SizeMode {other:?} is not drawn", site.control),
                }
            };
            assert_eq!(layout, site.layout, "{}.{}", site.control, site.property);
        }

        // Every image each of these Designers assigns is a site.
        let mut designers: Vec<&str> = SITES.iter().map(|site| site.designer).collect();
        designers.dedup();
        for name in designers {
            let designer = csharp(&format!("{view}/{name}")).unwrap_or_default();
            for line in designer.lines() {
                let Some((left, resource)) = line.trim().strip_prefix("this.").and_then(|line| {
                    line.split_once(" = global::MissionPlanner.Properties.Resources.")
                }) else {
                    continue;
                };
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
            }
        }
    }
}
