//! A software rasteriser for a HUD [`Scene`], and the perceptual comparison of two frames.
//!
//! The display is painted by gpui on the GPU, which needs a window; a golden frame has to be
//! drawn without one, the same on every run. So this draws the scene's own items - the fills,
//! strokes, labels and pictures [`super::scene`] produces from the C#'s `doPaint()` geometry - into
//! an RGB buffer with nothing but arithmetic:
//!
//! * **Coverage** is sampled 4 x 4 per pixel, at the sample centres, so an edge that crosses a
//!   pixel blends it by the share of samples inside. A float that differs in its last bit on
//!   another machine moves at most one sample, a sixteenth of a pixel's colour, which
//!   [`CHANNEL_TOLERANCE`] absorbs.
//! * **Fills** are even-odd, as gpui's `PathBuilder::fill` (lyon's default) fills.
//! * **Strokes** are gpui's `PathBuilder::stroke` defaults: butt ends, mitred corners with a
//!   miter limit of 4 and bevelled past it, an open polyline. Every segment and corner of one
//!   stroke is drawn as one shape, so a translucent stroke is not darker where its segments
//!   overlap.
//! * **Text** is a fixed 5 x 8 bitmap font ([`glyph`]), scaled to the label's size. It is not
//!   the font gpui shapes with - no font file is read, so the frame does not depend on what the
//!   machine has installed - but every character has its own shape, so a changed number is a
//!   changed picture.
//! * **Pictures** are drawn as [`super::paint`] draws them: their stand-ins, [`icon_items`].
//! * **A vertex that is not finite** - an attitude of NaN - leaves its shape undrawn. In a
//!   release build gpui's tessellator refuses a fill with a NaN corner (lyon's
//!   `PositionIsNaN`) and [`super::paint`] draws nothing for it; a debug build stops sooner, in
//!   lyon's path builder, which asserts every point is finite.
//!
//! [`compare`] is the perceptual difference: a pixel differs when a channel is more than
//! [`CHANNEL_TOLERANCE`] away, and two frames match while no more than [`PIXELS_TOLERATED`]
//! pixels differ. A moved line, a changed digit or a changed colour moves far more pixels than
//! that; `golden.rs` proves each.

use super::{Align, Item, Scene, icon_items};

/// Samples per pixel along each axis.
const SUB: i64 = 4;

/// Samples per pixel.
const SAMPLES: u8 = 16;

/// A channel may differ by this much and the pixel still match: one sample of sixteen flipping
/// on a black-white edge moves a channel by 16.
pub(super) const CHANNEL_TOLERANCE: u8 = 20;

/// Two frames match while no more pixels than this differ.
pub(super) const PIXELS_TOLERATED: usize = 4;

/// gpui's stroke miter limit: lyon's `StrokeOptions::DEFAULT_MITER_LIMIT`.
const MITER_LIMIT: f32 = 4.0;

/// An RGB image, row by row, three bytes a pixel.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(super) struct Image {
    /// Pixels across.
    pub width: u32,
    /// Pixels down.
    pub height: u32,
    /// `width * height` pixels, red, green and blue.
    pub pixels: Vec<u8>,
}

impl Image {
    /// An image of one colour, `0xRRGGBB`.
    pub fn new(width: u32, height: u32, colour: u32) -> Self {
        let [_, r, g, b] = colour.to_be_bytes();
        let count = usize::try_from(u64::from(width) * u64::from(height)).unwrap_or(0);
        Self {
            width,
            height,
            pixels: [r, g, b].repeat(count),
        }
    }

    /// The pixel at `(x, y)`, if it is inside.
    pub fn pixel(&self, x: u32, y: u32) -> Option<[u8; 3]> {
        let at = self.offset(x, y)?;
        let rgb = self.pixels.get(at..at + 3)?;
        Some([*rgb.first()?, *rgb.get(1)?, *rgb.get(2)?])
    }

    /// Where the pixel at `(x, y)` starts in [`Image::pixels`].
    fn offset(&self, x: u32, y: u32) -> Option<usize> {
        (x < self.width && y < self.height)
            .then(|| {
                usize::try_from((u64::from(y) * u64::from(self.width) + u64::from(x)) * 3).ok()
            })
            .flatten()
    }

    /// Blends `colour` over the pixel at `(x, y)` by `alpha`, from 0 to 1.
    fn blend(&mut self, x: u32, y: u32, colour: [u8; 3], alpha: f32) {
        let Some(at) = self.offset(x, y) else {
            return;
        };
        if let Some(rgb) = self.pixels.get_mut(at..at + 3) {
            for (channel, source) in rgb.iter_mut().zip(colour) {
                let (dst, src) = (f32::from(*channel), f32::from(source));
                // Within 0..=255 by construction: a mix of two bytes.
                #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
                let mixed = alpha.mul_add(src - dst, dst).round().clamp(0.0, 255.0) as u8;
                *channel = mixed;
            }
        }
    }
}

/// Draws a scene into a `width` x `height` image. What is behind the display is black, so a
/// shape that is not drawn - the sky of a NaN attitude - shows as a hole rather than as sky.
pub(super) fn render(scene: &Scene, width: u32, height: u32) -> Image {
    let mut image = Image::new(width, height, 0x00_00_00);
    for item in &scene.items {
        draw(&mut image, item);
    }
    image
}

/// Draws one item.
fn draw(image: &mut Image, item: &Item) {
    match item {
        Item::Fill {
            points,
            colour,
            alpha,
        } => {
            if points.len() >= 3 {
                fill(image, std::slice::from_ref(points), *colour, *alpha);
            }
        }
        Item::Stroke {
            points,
            width,
            colour,
            alpha,
        } => fill(image, &stroke_outline(points, *width), *colour, *alpha),
        Item::Label {
            text,
            at,
            size,
            colour,
            align,
        } => fill(
            image,
            &label_outline(text, *at, *size, *align),
            *colour,
            1.0,
        ),
        Item::Icon { icon, rect } => {
            for part in icon_items(*icon, *rect) {
                draw(image, &part);
            }
        }
    }
}

/// Fills the union of `shapes`, each even-odd, with `colour` at `alpha`, blended once however
/// many of them cover a sample. A shape with a vertex that is not finite is left out.
// Sample and pixel indices are small, non-negative and inside the image: the casts are checked
// by the clamps before them.
#[allow(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_precision_loss
)]
fn fill(image: &mut Image, shapes: &[Vec<(f32, f32)>], colour: u32, alpha: f32) {
    let shapes: Vec<&Vec<(f32, f32)>> = shapes
        .iter()
        .filter(|shape| {
            shape.len() >= 3 && shape.iter().all(|(x, y)| x.is_finite() && y.is_finite())
        })
        .collect();
    let Some((min, max)) = bounds(&shapes) else {
        return;
    };
    let (width, height) = (i64::from(image.width), i64::from(image.height));
    let (left, top) = ((min.0.floor() as i64).max(0), (min.1.floor() as i64).max(0));
    let (right, bottom) = (
        (max.0.ceil() as i64).min(width),
        (max.1.ceil() as i64).min(height),
    );
    if left >= right || top >= bottom {
        return;
    }
    let span = (right - left) as usize;
    let mut coverage = vec![0u8; span * (bottom - top) as usize];
    let mut intervals: Vec<(f32, f32)> = Vec::new();
    let mut crossings: Vec<f32> = Vec::new();
    for sub_row in top * SUB..bottom * SUB {
        let y = (sub_row as f32 + 0.5) / SUB as f32;
        intervals.clear();
        for shape in &shapes {
            crossings.clear();
            let edges = shape.iter().zip(shape.iter().cycle().skip(1));
            for (&(x0, y0), &(x1, y1)) in edges {
                if (y0 <= y && y < y1) || (y1 <= y && y < y0) {
                    crossings.push((y - y0).mul_add((x1 - x0) / (y1 - y0), x0));
                }
            }
            crossings.sort_by(f32::total_cmp);
            intervals.extend(
                crossings
                    .chunks_exact(2)
                    .filter_map(|pair| Some((*pair.first()?, *pair.get(1)?))),
            );
        }
        intervals.sort_by(|a, b| a.0.total_cmp(&b.0));
        let row = ((sub_row / SUB - top) as usize) * span;
        let mut merged: Option<(f32, f32)> = None;
        for &(start, end) in intervals
            .iter()
            .chain(std::iter::once(&(f32::MAX, f32::MAX)))
        {
            match merged {
                Some((from, to)) if start <= to => merged = Some((from, to.max(end))),
                _ => {
                    if let Some((from, to)) = merged.replace((start, end)) {
                        // The samples whose centres lie in [from, to).
                        let first = ((from * SUB as f32 - 0.5).ceil() as i64).max(left * SUB);
                        let last = ((to * SUB as f32 - 0.5).ceil() as i64).min(right * SUB);
                        let mut sample = first;
                        while sample < last {
                            let column = sample / SUB;
                            let next = ((column + 1) * SUB).min(last);
                            if let Some(count) = coverage.get_mut(row + (column - left) as usize) {
                                *count += (next - sample) as u8;
                            }
                            sample = next;
                        }
                    }
                }
            }
        }
    }
    let [_, r, g, b] = colour.to_be_bytes();
    for (index, count) in coverage.iter().enumerate() {
        if *count > 0 {
            let (x, y) = (left as usize + index % span, top as usize + index / span);
            let share = alpha * f32::from(*count) / f32::from(SAMPLES);
            image.blend(x as u32, y as u32, [r, g, b], share);
        }
    }
}

/// The corners of the box around every shape's vertices.
fn bounds(shapes: &[&Vec<(f32, f32)>]) -> Option<((f32, f32), (f32, f32))> {
    let mut points = shapes.iter().flat_map(|shape| shape.iter());
    let first = *points.next()?;
    Some(points.fold((first, first), |(min, max), &(x, y)| {
        ((min.0.min(x), min.1.min(y)), (max.0.max(x), max.1.max(y)))
    }))
}

/// The shapes a polyline stroked `width` wide covers: a rectangle per segment, butt at the ends,
/// and a mitre - or past the limit a bevel - at each corner, as lyon strokes an open path.
fn stroke_outline(points: &[(f32, f32)], width: f32) -> Vec<Vec<(f32, f32)>> {
    let half = width / 2.0;
    // Repeated points make no segment and no corner.
    let mut path: Vec<(f32, f32)> = Vec::with_capacity(points.len());
    for &point in points {
        if path.last() != Some(&point) {
            path.push(point);
        }
    }
    let directions: Vec<(f32, f32)> = path
        .windows(2)
        .filter_map(|pair| {
            let (a, b) = (*pair.first()?, *pair.get(1)?);
            let length = (b.0 - a.0).hypot(b.1 - a.1);
            Some(((b.0 - a.0) / length, (b.1 - a.1) / length))
        })
        .collect();
    let at = |p: (f32, f32), n: (f32, f32), distance: f32| {
        (n.0.mul_add(distance, p.0), n.1.mul_add(distance, p.1))
    };
    let normal = |d: (f32, f32)| (-d.1, d.0);
    let mut shapes = Vec::new();
    for (pair, &d) in path.windows(2).zip(&directions) {
        let (Some(&a), Some(&b)) = (pair.first(), pair.get(1)) else {
            continue;
        };
        let n = normal(d);
        shapes.push(vec![
            at(a, n, half),
            at(b, n, half),
            at(b, n, -half),
            at(a, n, -half),
        ]);
    }
    for (corner, turn) in path.iter().skip(1).zip(directions.windows(2)) {
        let (Some(&d1), Some(&d2)) = (turn.first(), turn.get(1)) else {
            continue;
        };
        let cross = d1.0.mul_add(d2.1, -(d1.1 * d2.0));
        if cross.abs() < 1e-6 && d1.0.mul_add(d2.0, d1.1 * d2.1) > 0.0 {
            continue;
        }
        // The outside of the turn, where the two rectangles leave a gap.
        let side = if cross > 0.0 { -1.0 } else { 1.0 };
        let (n1, n2) = (normal(d1), normal(d2));
        let (o1, o2) = (at(*corner, n1, side * half), at(*corner, n2, side * half));
        let sum = (n1.0 + n2.0, n1.1 + n2.1);
        let length = sum.0.hypot(sum.1);
        let mitre = (length > 1e-6)
            .then(|| (side * sum.0 / length, side * sum.1 / length))
            .and_then(|m| {
                let cosine = m.0.mul_add(side * n1.0, m.1 * side * n1.1);
                (cosine > 0.0 && 1.0 / cosine <= MITER_LIMIT).then(|| at(*corner, m, half / cosine))
            });
        match mitre {
            Some(tip) => shapes.push(vec![*corner, o1, tip, o2]),
            None => shapes.push(vec![*corner, o1, o2]),
        }
    }
    shapes
}

/// The font's cell, as a share of the label's size: a glyph is 5 cells wide and 8 tall, and
/// advances 6.
const CELL: f32 = 0.1;

/// Where the glyph's top row sits below the label's anchor, in cells: gpui draws a line
/// `size * 1.2` tall with the text's top at the anchor.
const TOP_CELLS: f32 = 1.5;

/// The rectangles a label's glyphs cover.
pub(super) fn label_outline(
    text: &str,
    at: (f32, f32),
    size: f32,
    align: Align,
) -> Vec<Vec<(f32, f32)>> {
    let cell = size * CELL;
    let count = text.chars().count();
    #[allow(clippy::cast_precision_loss)] // a label is a few characters
    let width = (count as f32).mul_add(6.0 * cell, -cell).max(0.0);
    let left = match align {
        Align::Left => at.0,
        Align::Centre => at.0 - width / 2.0,
    };
    let top = cell.mul_add(TOP_CELLS, at.1);
    let mut shapes = Vec::new();
    let mut x = left;
    for character in text.chars() {
        let mut column_x = x;
        for column in glyph(character) {
            // Each run of set bits down a column is one rectangle.
            let mut row = 0u8;
            while row < 8 {
                if column & (1 << row) == 0 {
                    row += 1;
                    continue;
                }
                let start = row;
                while row < 8 && column & (1 << row) != 0 {
                    row += 1;
                }
                let (y0, y1) = (
                    f32::from(start).mul_add(cell, top),
                    f32::from(row).mul_add(cell, top),
                );
                let x1 = column_x + cell;
                shapes.push(vec![(column_x, y0), (x1, y0), (x1, y1), (column_x, y1)]);
            }
            column_x += cell;
        }
        x = 6.0f32.mul_add(cell, x);
    }
    shapes
}

/// What a character without a glyph draws: a solid box, as a font's missing glyph is.
const MISSING: [u8; 5] = [0x7F; 5];

/// The classic 5 x 8 LCD font for printable ASCII, a column a byte, bit 0 at the top.
const FONT: [[u8; 5]; 95] = [
    [0x00, 0x00, 0x00, 0x00, 0x00], // space
    [0x00, 0x00, 0x5F, 0x00, 0x00], // !
    [0x00, 0x07, 0x00, 0x07, 0x00], // "
    [0x14, 0x7F, 0x14, 0x7F, 0x14], // #
    [0x24, 0x2A, 0x7F, 0x2A, 0x12], // $
    [0x23, 0x13, 0x08, 0x64, 0x62], // %
    [0x36, 0x49, 0x56, 0x20, 0x50], // &
    [0x00, 0x08, 0x07, 0x03, 0x00], // '
    [0x00, 0x1C, 0x22, 0x41, 0x00], // (
    [0x00, 0x41, 0x22, 0x1C, 0x00], // )
    [0x2A, 0x1C, 0x7F, 0x1C, 0x2A], // *
    [0x08, 0x08, 0x3E, 0x08, 0x08], // +
    [0x00, 0x80, 0x70, 0x30, 0x00], // ,
    [0x08, 0x08, 0x08, 0x08, 0x08], // -
    [0x00, 0x00, 0x60, 0x60, 0x00], // .
    [0x20, 0x10, 0x08, 0x04, 0x02], // /
    [0x3E, 0x51, 0x49, 0x45, 0x3E], // 0
    [0x00, 0x42, 0x7F, 0x40, 0x00], // 1
    [0x72, 0x49, 0x49, 0x49, 0x46], // 2
    [0x21, 0x41, 0x49, 0x4D, 0x33], // 3
    [0x18, 0x14, 0x12, 0x7F, 0x10], // 4
    [0x27, 0x45, 0x45, 0x45, 0x39], // 5
    [0x3C, 0x4A, 0x49, 0x49, 0x31], // 6
    [0x41, 0x21, 0x11, 0x09, 0x07], // 7
    [0x36, 0x49, 0x49, 0x49, 0x36], // 8
    [0x46, 0x49, 0x49, 0x29, 0x1E], // 9
    [0x00, 0x00, 0x14, 0x00, 0x00], // :
    [0x00, 0x40, 0x34, 0x00, 0x00], // ;
    [0x00, 0x08, 0x14, 0x22, 0x41], // <
    [0x14, 0x14, 0x14, 0x14, 0x14], // =
    [0x00, 0x41, 0x22, 0x14, 0x08], // >
    [0x02, 0x01, 0x59, 0x09, 0x06], // ?
    [0x3E, 0x41, 0x5D, 0x59, 0x4E], // @
    [0x7C, 0x12, 0x11, 0x12, 0x7C], // A
    [0x7F, 0x49, 0x49, 0x49, 0x36], // B
    [0x3E, 0x41, 0x41, 0x41, 0x22], // C
    [0x7F, 0x41, 0x41, 0x41, 0x3E], // D
    [0x7F, 0x49, 0x49, 0x49, 0x41], // E
    [0x7F, 0x09, 0x09, 0x09, 0x01], // F
    [0x3E, 0x41, 0x41, 0x51, 0x73], // G
    [0x7F, 0x08, 0x08, 0x08, 0x7F], // H
    [0x00, 0x41, 0x7F, 0x41, 0x00], // I
    [0x20, 0x40, 0x41, 0x3F, 0x01], // J
    [0x7F, 0x08, 0x14, 0x22, 0x41], // K
    [0x7F, 0x40, 0x40, 0x40, 0x40], // L
    [0x7F, 0x02, 0x1C, 0x02, 0x7F], // M
    [0x7F, 0x04, 0x08, 0x10, 0x7F], // N
    [0x3E, 0x41, 0x41, 0x41, 0x3E], // O
    [0x7F, 0x09, 0x09, 0x09, 0x06], // P
    [0x3E, 0x41, 0x51, 0x21, 0x5E], // Q
    [0x7F, 0x09, 0x19, 0x29, 0x46], // R
    [0x26, 0x49, 0x49, 0x49, 0x32], // S
    [0x03, 0x01, 0x7F, 0x01, 0x03], // T
    [0x3F, 0x40, 0x40, 0x40, 0x3F], // U
    [0x1F, 0x20, 0x40, 0x20, 0x1F], // V
    [0x3F, 0x40, 0x38, 0x40, 0x3F], // W
    [0x63, 0x14, 0x08, 0x14, 0x63], // X
    [0x03, 0x04, 0x78, 0x04, 0x03], // Y
    [0x61, 0x59, 0x49, 0x4D, 0x43], // Z
    [0x00, 0x7F, 0x41, 0x41, 0x41], // [
    [0x02, 0x04, 0x08, 0x10, 0x20], // backslash
    [0x00, 0x41, 0x41, 0x41, 0x7F], // ]
    [0x04, 0x02, 0x01, 0x02, 0x04], // ^
    [0x40, 0x40, 0x40, 0x40, 0x40], // _
    [0x00, 0x03, 0x07, 0x08, 0x00], // `
    [0x20, 0x54, 0x54, 0x78, 0x40], // a
    [0x7F, 0x28, 0x44, 0x44, 0x38], // b
    [0x38, 0x44, 0x44, 0x44, 0x28], // c
    [0x38, 0x44, 0x44, 0x28, 0x7F], // d
    [0x38, 0x54, 0x54, 0x54, 0x18], // e
    [0x00, 0x08, 0x7E, 0x09, 0x02], // f
    [0x18, 0xA4, 0xA4, 0x9C, 0x78], // g
    [0x7F, 0x08, 0x04, 0x04, 0x78], // h
    [0x00, 0x44, 0x7D, 0x40, 0x00], // i
    [0x20, 0x40, 0x40, 0x3D, 0x00], // j
    [0x7F, 0x10, 0x28, 0x44, 0x00], // k
    [0x00, 0x41, 0x7F, 0x40, 0x00], // l
    [0x7C, 0x04, 0x78, 0x04, 0x78], // m
    [0x7C, 0x08, 0x04, 0x04, 0x78], // n
    [0x38, 0x44, 0x44, 0x44, 0x38], // o
    [0xFC, 0x18, 0x24, 0x24, 0x18], // p
    [0x18, 0x24, 0x24, 0x18, 0xFC], // q
    [0x7C, 0x08, 0x04, 0x04, 0x08], // r
    [0x48, 0x54, 0x54, 0x54, 0x24], // s
    [0x04, 0x04, 0x3F, 0x44, 0x24], // t
    [0x3C, 0x40, 0x40, 0x20, 0x7C], // u
    [0x1C, 0x20, 0x40, 0x20, 0x1C], // v
    [0x3C, 0x40, 0x30, 0x40, 0x3C], // w
    [0x44, 0x28, 0x10, 0x28, 0x44], // x
    [0x4C, 0x90, 0x90, 0x90, 0x7C], // y
    [0x44, 0x64, 0x54, 0x4C, 0x44], // z
    [0x00, 0x08, 0x36, 0x41, 0x00], // {
    [0x00, 0x00, 0x77, 0x00, 0x00], // |
    [0x00, 0x41, 0x36, 0x08, 0x00], // }
    [0x02, 0x01, 0x02, 0x04, 0x02], // ~
];

/// The degree sign, which is not ASCII.
const DEGREE: [u8; 5] = [0x00, 0x06, 0x09, 0x09, 0x06];

/// A character's columns.
fn glyph(character: char) -> [u8; 5] {
    if character == '\u{b0}' {
        return DEGREE;
    }
    u32::from(character)
        .checked_sub(0x20)
        .and_then(|index| FONT.get(usize::try_from(index).ok()?))
        .copied()
        .unwrap_or(MISSING)
}

/// How two frames differ.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct Difference {
    /// Pixels with a channel more than [`CHANNEL_TOLERANCE`] away.
    pub differing: usize,
    /// The largest difference in any channel.
    pub worst: u8,
    /// The box around the differing pixels: left, top, right, bottom, inclusive.
    pub region: Option<(u32, u32, u32, u32)>,
}

impl Difference {
    /// Whether the frames match: few enough pixels differ.
    pub const fn matches(&self) -> bool {
        self.differing <= PIXELS_TOLERATED
    }
}

/// How `actual` differs from `expected`, or why they cannot be compared.
///
/// # Errors
///
/// The images are not the same size.
pub(super) fn compare(expected: &Image, actual: &Image) -> Result<Difference, String> {
    if (expected.width, expected.height) != (actual.width, actual.height) {
        return Err(format!(
            "{}x{} against {}x{}",
            actual.width, actual.height, expected.width, expected.height
        ));
    }
    let mut difference = Difference {
        differing: 0,
        worst: 0,
        region: None,
    };
    let pixels = expected
        .pixels
        .chunks_exact(3)
        .zip(actual.pixels.chunks_exact(3));
    for (index, (want, got)) in pixels.enumerate() {
        let worst = want
            .iter()
            .zip(got)
            .map(|(a, b)| a.abs_diff(*b))
            .max()
            .unwrap_or(0);
        difference.worst = difference.worst.max(worst);
        if worst > CHANNEL_TOLERANCE {
            difference.differing += 1;
            let index = u32::try_from(index).unwrap_or(u32::MAX);
            let (x, y) = (index % expected.width, index / expected.width);
            difference.region = Some(match difference.region {
                None => (x, y, x, y),
                Some((l, t, r, b)) => (l.min(x), t.min(y), r.max(x), b.max(y)),
            });
        }
    }
    Ok(difference)
}

/// A picture of the difference, for a person: the expected frame dimmed, with every differing
/// pixel in magenta.
pub(super) fn difference_image(expected: &Image, actual: &Image) -> Image {
    let mut out = expected.clone();
    for (index, pixel) in out.pixels.chunks_exact_mut(3).enumerate() {
        let got = actual.pixels.get(index * 3..index * 3 + 3);
        let differs = got.is_none_or(|got| {
            pixel
                .iter()
                .zip(got)
                .any(|(a, b)| a.abs_diff(*b) > CHANNEL_TOLERANCE)
        });
        if differs {
            pixel.copy_from_slice(&[0xff, 0x00, 0xff]);
        } else {
            for channel in pixel.iter_mut() {
                *channel /= 3;
            }
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    const RED: u32 = 0xff_00_00;

    fn fill_scene(points: Vec<(f32, f32)>, alpha: f32) -> Scene {
        Scene {
            items: vec![Item::Fill {
                points,
                colour: RED,
                alpha,
            }],
            ..Scene::default()
        }
    }

    /// A square on pixel edges covers exactly its pixels, and nothing next to them.
    #[test]
    fn a_square_on_pixel_edges_fills_exactly_its_pixels() {
        let image = render(
            &fill_scene(vec![(2.0, 3.0), (6.0, 3.0), (6.0, 7.0), (2.0, 7.0)], 1.0),
            10,
            10,
        );
        for y in 0..10 {
            for x in 0..10 {
                let inside = (2..6).contains(&x) && (3..7).contains(&y);
                let want = if inside { [0xff, 0, 0] } else { [0, 0, 0] };
                assert_eq!(image.pixel(x, y), Some(want), "({x}, {y})");
            }
        }
    }

    /// An edge through the middle of a pixel covers half its samples: half the colour.
    #[test]
    fn an_edge_through_a_pixel_blends_it_by_its_coverage() {
        let image = render(
            &fill_scene(vec![(0.0, 0.0), (2.5, 0.0), (2.5, 1.0), (0.0, 1.0)], 1.0),
            4,
            1,
        );
        assert_eq!(image.pixel(1, 0), Some([0xff, 0, 0]));
        assert_eq!(image.pixel(2, 0), Some([0x80, 0, 0]));
        assert_eq!(image.pixel(3, 0), Some([0, 0, 0]));
    }

    /// A translucent stroke is one shape: its corner is no darker than its sides.
    #[test]
    fn a_translucent_strokes_corner_is_blended_once() {
        let scene = Scene {
            items: vec![Item::Stroke {
                points: vec![(2.0, 2.0), (12.0, 2.0), (12.0, 12.0)],
                width: 3.0,
                colour: RED,
                alpha: 0.5,
            }],
            ..Scene::default()
        };
        let image = render(&scene, 16, 16);
        let reds: Vec<u8> = image.pixels.chunks_exact(3).map(|p| p[0]).collect();
        assert!(reds.iter().all(|red| *red <= 0x80), "{reds:?}");
        // The mitre fills the outside of the corner: (12, 1) is inside it and inside neither
        // segment.
        assert_eq!(image.pixel(12, 1), Some([0x80, 0, 0]));
        // Butt ends: nothing before the first point.
        assert_eq!(image.pixel(0, 2), Some([0, 0, 0]));
    }

    /// A shape with a vertex that is not a number is not drawn, as a failed tessellation is not.
    #[test]
    fn a_shape_with_a_nan_vertex_is_not_drawn() {
        let image = render(
            &fill_scene(
                vec![(0.0, 0.0), (f32::NAN, 0.0), (4.0, 4.0), (0.0, 4.0)],
                1.0,
            ),
            4,
            4,
        );
        assert!(image.pixels.iter().all(|channel| *channel == 0));
    }

    /// Every character has a shape of its own, so a changed character is a changed picture.
    #[test]
    fn every_glyph_is_different_and_only_the_space_is_empty() {
        let mut all: Vec<(char, [u8; 5])> = (0x20u8..0x7f)
            .map(|byte| (char::from(byte), glyph(char::from(byte))))
            .collect();
        all.push(('\u{b0}', glyph('\u{b0}')));
        all.push(('\u{2192}', glyph('\u{2192}')));
        for (index, (a, shape)) in all.iter().enumerate() {
            assert_eq!(*a == ' ', *shape == [0; 5], "{a:?}");
            for (b, other) in all.iter().skip(index + 1) {
                assert_ne!(shape, other, "{a:?} and {b:?} look the same");
            }
        }
    }

    /// A centred label is centred on its anchor; a left one starts there.
    #[test]
    fn labels_are_placed_by_their_alignment() {
        let extent = |align| {
            let shapes = label_outline("88", (50.0, 10.0), 10.0, align);
            let xs = shapes.iter().flatten().map(|(x, _)| *x);
            let (min, max) = xs.fold((f32::MAX, f32::MIN), |(lo, hi), x| (lo.min(x), hi.max(x)));
            (min, max)
        };
        let (left, right) = extent(Align::Centre);
        assert!(
            (50.0 - left - (right - 50.0)).abs() < 1e-4,
            "{left} {right}"
        );
        assert!((extent(Align::Left).0 - 50.0).abs() < 1e-4);
    }

    /// The comparison: the same image matches; a few pixels a little off match; one pixel far
    /// off is counted; more than the tolerated count does not match; sizes must agree.
    #[test]
    fn the_comparison_counts_pixels_past_the_channel_tolerance() {
        let base = Image::new(8, 8, 0x40_40_40);
        let same = compare(&base, &base.clone()).expect("same size");
        assert_eq!((same.differing, same.worst, same.region), (0, 0, None));
        let mut nudged = base.clone();
        for channel in &mut nudged.pixels {
            *channel += CHANNEL_TOLERANCE;
        }
        let nudged = compare(&base, &nudged).expect("same size");
        assert!(nudged.matches() && nudged.differing == 0);
        let mut changed = base.clone();
        for x in 0..=u32::try_from(PIXELS_TOLERATED).expect("small") {
            changed.blend(x, 3, [0xff, 0xff, 0xff], 1.0);
        }
        let changed = compare(&base, &changed).expect("same size");
        assert_eq!(changed.differing, PIXELS_TOLERATED + 1);
        assert!(!changed.matches());
        assert_eq!(changed.region, Some((0, 3, 4, 3)));
        assert!(compare(&base, &Image::new(8, 9, 0)).is_err());
    }
}
