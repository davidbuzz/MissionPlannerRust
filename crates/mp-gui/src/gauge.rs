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

//! The round dials: the Gauges page's speed dial, `Gspeed`, with two needles, airspeed and
//! ground speed, over a scale from 0 to its maximum - 60 until a double click asks for another -
//! and the RAW Sensor window's roll, pitch and yaw dials, each with one needle.
//!
//! Ported from the behaviour of `ExtLibs/Controls/AGauge.cs` alone, none of its code: Mission
//! Planner draws these dials with `AGauge` (A.J. Bauer, 2007, with Michael Oborne's changes), a
//! control under the Code Project Open License, which the GPL does not admit. The port of
//! 2026-09-25 transliterated its painting, and the owner had it rewritten (2026-10-03). What is
//! here is written from what the control shows and from the two pages' Designers, which are
//! Mission Planner's own: a dial drawn in a 150-pixel square and scaled to its control, with the coloured
//! bands of its ranges, the arc its scale sits on, a line and a number at every major step, the
//! minor lines between them with the middle one longer when their count is odd, a caption, and a
//! shaded needle on a cap for each value, pointing where the value falls between the scale's start
//! and its end. The face image the Designer's `BackgroundImage` gives the control is not drawn,
//! under the palette ruling, and the scale and needles are drawn on the page's own background -
//! in white where the Designer has them black, which the dark background would swallow.
//! `// C#: GCSViews/FlightData.Designer.cs:1464-1607, Controls/RAW_Sensor.Designer.cs:405-547,
//! 609-750, 752-893`
//!
//! Galt, Gheading and Gvspeed, the Gauges page's other dials, are not ported.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use crate::hud::{Align, Item, Scene};

/// The square every dial is set out in before it is scaled to its control: the Designers give
/// each dial's `Center` as (75, 75) and its radii within 75.
pub const BASE_SIZE: f32 = 150.0;

/// The font the gauge inherits, Microsoft Sans Serif 8.25pt, in pixels at 96 dpi.
const FONT: f32 = 11.0;

/// White, the scale's colour. `// C#: GCSViews/FlightData.Designer.cs:1581-1595`
const WHITE: u32 = 0xff_ff_ff;
/// `Color.Gray`.
const GRAY: u32 = 0x80_80_80;
/// `Color.DimGray`, the RAW Sensor needles' cap.
const DIM_GRAY: u32 = 0x69_69_69;
/// `Color.LightGreen`.
const LIGHT_GREEN: u32 = 0x90_ee_90;
/// `Color.LightSteelBlue`.
const LIGHT_STEEL_BLUE: u32 = 0xb0_c4_de;

/// `NeedlesColor1`: which colour a needle is shaded in.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NeedleColour {
    /// `Gray`.
    Gray,
    /// `Red`.
    Red,
}

impl NeedleColour {
    /// The needle's two shades, lit and shadowed, and the colour of the ring round its cap.
    const fn shades(self) -> (u32, u32, u32) {
        match self {
            Self::Gray => (0xc8_c8_c8, 0x70_70_70, GRAY),
            Self::Red => (0xff_50_50, 0xb0_00_00, 0xff_00_00),
        }
    }
}

/// One needle: its value, its length, its width, its shading and its cap's colour.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Needle {
    /// `Value0` to `Value3`, as bound.
    pub value: f32,
    /// `NeedlesRadius`: the tip's distance from the centre.
    pub radius: f32,
    /// `NeedlesWidth`: half the needle's width at the centre is twice this, and the cap's radius
    /// three times it.
    pub width: f32,
    /// `NeedlesColor1`.
    pub colour: NeedleColour,
    /// `NeedlesColor2`, the cap's.
    pub cap: u32,
}

/// `RangesEnabled[i]`: a coloured band between two radii from one value of the scale to another,
/// drawn under the scale when its end is past its start.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Band {
    /// `RangesStartValue`.
    pub from: f32,
    /// `RangesEndValue`.
    pub to: f32,
    /// `RangesInnerRadius`.
    pub inner: f32,
    /// `RangesOuterRadius`.
    pub outer: f32,
    /// `RangesColor`.
    pub colour: u32,
}

/// One dial's settings, as its Designer gives them: the scale's span and arc, the arc under it,
/// its lines and their colours, its numbers, its caption and its bands.
#[derive(Debug, Clone, PartialEq)]
pub struct Dial {
    /// `MinValue`.
    pub min: f32,
    /// `MaxValue`.
    pub max: f32,
    /// `BaseArcStart`, degrees clockwise from east.
    pub arc_start: f32,
    /// `BaseArcSweep`.
    pub arc_sweep: f32,
    /// `BaseArcRadius`, `BaseArcWidth` and `BaseArcColor`: the arc the scale sits on, or `None`
    /// where it is not drawn.
    pub base_arc: Option<(f32, f32, u32)>,
    /// `ScaleLinesMajorStepValue`.
    pub major_step: f32,
    /// `ScaleLinesMinorNumOf`.
    pub minor_count: u8,
    /// `ScaleLinesMajorInnerRadius`, `OuterRadius`, `Width`.
    pub major: (f32, f32, f32),
    /// `ScaleLinesMinorInnerRadius`, `OuterRadius`, `Width`.
    pub minor: (f32, f32, f32),
    /// `ScaleLinesInterInnerRadius`, `OuterRadius`, `Width`: the middle minor line.
    pub inter: (f32, f32, f32),
    /// `ScaleLinesMajorColor` (and the inter line's), `ScaleLinesMinorColor`, `ScaleNumbersColor`.
    pub colours: (u32, u32, u32),
    /// `ScaleNumbersRadius`.
    pub numbers_radius: f32,
    /// `CapsText[0]` at `CapsPosition[0]`; the other caps are empty.
    pub cap: (&'static str, (f32, f32)),
    /// The ranges enabled.
    pub bands: &'static [Band],
}

/// The speed dial's settings, as the Designer gives them. `// C#: GCSViews/FlightData.Designer.cs:1464-1607`
#[derive(Debug, Clone, PartialEq)]
pub struct SpeedGauge {
    /// `MaxValue`: 60 in the Designer, and whatever a double click has set since.
    pub max: f32,
}

impl Default for SpeedGauge {
    fn default() -> Self {
        Self { max: 60.0 }
    }
}

/// The speed dial's `MinValue`.
const SPEED_MIN: f32 = 0.0;

impl SpeedGauge {
    /// The two needles the Designer enables: airspeed, gray and two wide, and ground speed, red
    /// and one wide, both 50 long with white caps. Each value is held to the scale, as the control
    /// holds the values it is given. `// C#: GCSViews/FlightData.Designer.cs:1464-1607`
    #[must_use]
    pub fn needles(&self, airspeed: f32, groundspeed: f32) -> [Needle; 2] {
        let held = |value: f32| value.max(SPEED_MIN).min(self.max);
        [
            Needle {
                value: held(airspeed),
                radius: 50.0,
                width: 2.0,
                colour: NeedleColour::Gray,
                cap: WHITE,
            },
            Needle {
                value: held(groundspeed),
                radius: 50.0,
                width: 1.0,
                colour: NeedleColour::Red,
                cap: WHITE,
            },
        ]
    }

    /// `MaxValue = float.Parse(max)`: the maximum changed where the text is a number above the
    /// minimum, and left as it was otherwise, as the control leaves a maximum that is not above
    /// its minimum; text that is not a number throws in the C#, which has no `catch` here, and is
    /// .NET's message. `// C#: GCSViews/FlightData.cs:3153-3161`
    pub fn set_max(&mut self, text: &str) -> Result<(), &'static str> {
        let value =
            crate::fly::dotnet_float(text).ok_or("Input string was not in a correct format.")?;
        if value > SPEED_MIN {
            self.max = value;
        }
        Ok(())
    }

    /// The dial as the Designer sets it: 0 to the maximum over 270 degrees from the lower left,
    /// a line and a number every 10 with nine between, "Speed" under the centre, no ranges; its
    /// base arc (radius 70) is not drawn, as it has not been.
    /// `// C#: GCSViews/FlightData.Designer.cs:1464-1607`
    #[must_use]
    pub const fn dial(&self) -> Dial {
        Dial {
            min: SPEED_MIN,
            max: self.max,
            arc_start: 135.0,
            arc_sweep: 270.0,
            base_arc: None,
            major_step: 10.0,
            minor_count: 9,
            major: (50.0, 60.0, 2.0),
            minor: (55.0, 60.0, 1.0),
            inter: (52.0, 60.0, 1.0),
            colours: (WHITE, WHITE, WHITE),
            numbers_radius: 42.0,
            cap: ("Speed", (58.0, 85.0)),
            bands: &[],
        }
    }

    /// The dial drawn `size` pixels square: the scale, the caption and the needles.
    #[must_use]
    pub fn scene(&self, size: f32, needles: &[Needle]) -> Scene {
        self.dial().scene(size, needles)
    }
}

/// The RAW Sensor dials' bands for roll and pitch: LightSteelBlue from -90 to 90, LightGreen
/// beyond it either way, 50 to 60 out. `// C#: Controls/RAW_Sensor.Designer.cs:689-725, 832-868`
const LEVEL_BANDS: [Band; 3] = [
    Band {
        from: -90.0,
        to: 90.0,
        inner: 50.0,
        outer: 60.0,
        colour: LIGHT_STEEL_BLUE,
    },
    Band {
        from: 90.0,
        to: 180.0,
        inner: 50.0,
        outer: 60.0,
        colour: LIGHT_GREEN,
    },
    Band {
        from: -180.0,
        to: -90.0,
        inner: 50.0,
        outer: 60.0,
        colour: LIGHT_GREEN,
    },
];

/// The yaw dial's one band: LightGreen the whole way round.
/// `// C#: Controls/RAW_Sensor.Designer.cs:481-521`
const YAW_BANDS: [Band; 1] = [Band {
    from: 0.0,
    to: 360.0,
    inner: 50.0,
    outer: 60.0,
    colour: LIGHT_GREEN,
}];

/// What the three RAW Sensor dials share: a gray base arc of 50, the major lines 50 to 60 and
/// two wide, the minor 50 to 55, the inter line the Designer's 60 to 50 - the same line - the
/// numbers at 38, the caption at (10, 10); the black lines white under the palette ruling.
/// `// C#: Controls/RAW_Sensor.Designer.cs:405-547`
const fn raw_dial(
    min: f32,
    max: f32,
    arc_start: f32,
    major_step: f32,
    minor_count: u8,
    cap: &'static str,
    bands: &'static [Band],
) -> Dial {
    Dial {
        min,
        max,
        arc_start,
        arc_sweep: 360.0,
        base_arc: Some((50.0, 2.0, GRAY)),
        major_step,
        minor_count,
        major: (50.0, 60.0, 2.0),
        minor: (50.0, 55.0, 1.0),
        inter: (50.0, 60.0, 1.0),
        colours: (WHITE, GRAY, WHITE),
        numbers_radius: 38.0,
        cap: (cap, (10.0, 10.0)),
        bands,
    }
}

/// `Groll`: -180 to 179 the whole way round from the bottom, a line every 30 with five between,
/// "Roll". `// C#: Controls/RAW_Sensor.Designer.cs:752-893`
#[must_use]
pub const fn roll_dial() -> Dial {
    raw_dial(-180.0, 179.0, 90.0, 30.0, 5, "Roll", &LEVEL_BANDS)
}

/// `Gpitch`: -90 to 89 the whole way round from the bottom, a line every 20 with nine between,
/// "Pitch". `// C#: Controls/RAW_Sensor.Designer.cs:609-750`
#[must_use]
pub const fn pitch_dial() -> Dial {
    raw_dial(-90.0, 89.0, 90.0, 20.0, 9, "Pitch", &LEVEL_BANDS)
}

/// `aGauge1`: 0 to 359 the whole way round from the top, a line every 45 with two between,
/// "Yaw". `// C#: Controls/RAW_Sensor.Designer.cs:405-547`
#[must_use]
pub const fn yaw_dial() -> Dial {
    raw_dial(0.0, 359.0, 270.0, 45.0, 2, "Yaw", &YAW_BANDS)
}

/// `Center`, in the base square: every dial here has it at (75, 75).
const CENTRE: (f32, f32) = (75.0, 75.0);

/// A point of the drawn dial: `radius` from the centre towards `degrees`, clockwise from east,
/// scaled to the control.
fn at(scale: f32, radius: f32, degrees: f32) -> (f32, f32) {
    let (sin, cos) = degrees.to_radians().sin_cos();
    (
        (CENTRE.0 + radius * cos) * scale,
        (CENTRE.1 + radius * sin) * scale,
    )
}

/// An arc as a polyline: a vertex every three degrees or so, and both ends exactly.
fn arc(scale: f32, radius: f32, from: f32, sweep: f32) -> Vec<(f32, f32)> {
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // a count of degrees
    let steps = ((sweep.abs() / 3.0).ceil() as usize).max(1);
    #[allow(clippy::cast_precision_loss)] // small counts
    (0..=steps)
        .map(|step| at(scale, radius, from + sweep * step as f32 / steps as f32))
        .collect()
}

/// A circle as a polygon.
fn circle((x, y): (f32, f32), radius: f32) -> Vec<(f32, f32)> {
    (0..36)
        .map(|step| {
            #[allow(clippy::cast_precision_loss)]
            let angle = step as f32 * std::f32::consts::TAU / 36.0;
            (x + radius * angle.cos(), y + radius * angle.sin())
        })
        .collect()
}

/// A scale number as `float.ToString()` writes it: "0", "10", "12.5".
fn number_text(value: f32) -> String {
    value.to_string()
}

/// How far past the scale's end a line's value may fall through rounding and still be drawn.
const SLACK: f32 = 1e-3;

impl Dial {
    /// A value held to the scale, as the control holds the values it is given.
    #[must_use]
    pub fn held(&self, value: f32) -> f32 {
        value.max(self.min).min(self.max)
    }

    /// The RAW Sensor dials' one needle, `NeedlesEnabled[0]`: gray, 50 long, two wide, over a
    /// DimGray cap. `// C#: Controls/RAW_Sensor.Designer.cs:443-478`
    #[must_use]
    pub fn raw_needle(&self, value: f32) -> Needle {
        Needle {
            value: self.held(value),
            radius: 50.0,
            width: 2.0,
            colour: NeedleColour::Gray,
            cap: DIM_GRAY,
        }
    }

    /// Where a value of the scale points, in degrees clockwise from east: the scale's minimum at
    /// the arc's start, its maximum at the arc's end, and every value between in proportion.
    fn angle_of(&self, value: f32) -> f32 {
        self.arc_start + (value - self.min) * self.arc_sweep / (self.max - self.min)
    }

    /// The dial drawn `size` pixels square, bottom to top: the bands, the base arc, the scale
    /// with its numbers, the caption and the needles.
    #[must_use]
    pub fn scene(&self, size: f32, needles: &[Needle]) -> Scene {
        let scale = size / BASE_SIZE;
        let mut scene = Scene::default();
        for band in self.bands {
            self.band(&mut scene, scale, band);
        }
        if let Some((radius, width, colour)) = self.base_arc {
            scene.items.push(Item::Stroke {
                points: arc(scale, radius, self.arc_start, self.arc_sweep),
                width: width * scale,
                colour,
                alpha: 1.0,
            });
        }
        self.scale_lines(&mut scene, scale);
        let (caption, (x, y)) = self.cap;
        scene.items.push(Item::Label {
            text: caption.to_owned(),
            at: (x * scale, y * scale),
            size: FONT * scale,
            colour: self.colours.2,
            align: Align::Left,
        });
        for needle in needles {
            self.needle(&mut scene, scale, needle);
        }
        scene
    }

    /// A band: the ring between its radii from where its first value points to where its last
    /// does, filled in its colour; nothing when its last value is not past its first.
    fn band(&self, scene: &mut Scene, scale: f32, band: &Band) {
        if band.to <= band.from {
            return;
        }
        let from = self.angle_of(band.from);
        let sweep = self.angle_of(band.to) - from;
        let mut outline = arc(scale, band.outer, from, sweep);
        outline.extend(arc(scale, band.inner, from + sweep, -sweep));
        scene.items.push(Item::Fill {
            points: outline,
            colour: band.colour,
            alpha: 1.0,
        });
    }

    /// A radial line between two radii where `value` points.
    fn line(
        &self,
        scene: &mut Scene,
        scale: f32,
        (inner, outer, width): (f32, f32, f32),
        colour: u32,
        value: f32,
    ) {
        let degrees = self.angle_of(value);
        scene.items.push(Item::Stroke {
            points: vec![at(scale, inner, degrees), at(scale, outer, degrees)],
            width: width * scale,
            colour,
            alpha: 1.0,
        });
    }

    /// The scale: a major line and its number at the minimum and every major step after it, and
    /// after each major line the minor lines, spaced evenly to the next major step, the middle one
    /// the longer inter line when their count is odd. A scale whose span is no whole number of
    /// steps - roll's 359 in steps of 30 - ends with the minor lines past its last major, as far
    /// as they fall within it.
    fn scale_lines(&self, scene: &mut Scene, scale: f32) {
        let (major_colour, minor_colour, numbers_colour) = self.colours;
        let between = f32::from(self.minor_count) + 1.0;
        let middle = (self.minor_count % 2 == 1).then_some(self.minor_count.div_ceil(2));
        let mut value = self.min;
        while value <= self.max + SLACK {
            self.line(scene, scale, self.major, major_colour, value);
            let (x, y) = at(scale, self.numbers_radius, self.angle_of(value));
            scene.items.push(Item::Label {
                text: number_text(value),
                at: (x, y - FONT * scale / 2.0),
                size: FONT * scale,
                colour: numbers_colour,
                align: Align::Centre,
            });
            for step in 1..=self.minor_count {
                let minor = value + f32::from(step) * self.major_step / between;
                if minor > self.max + SLACK {
                    break;
                }
                if middle == Some(step) {
                    self.line(scene, scale, self.inter, major_colour, minor);
                } else {
                    self.line(scene, scale, self.minor, minor_colour, minor);
                }
            }
            value += self.major_step;
        }
    }

    /// A needle: a two-tone pointer from a short tail through the centre to its tip at its
    /// length, lit on one side and shadowed on the other, with a cap over the centre - a disc in
    /// the cap's colour ringed in the needle's.
    fn needle(&self, scene: &mut Scene, scale: f32, needle: &Needle) {
        let degrees = self.angle_of(needle.value);
        let tip = at(scale, needle.radius, degrees);
        let tail = at(scale, needle.radius / 10.0, degrees + 180.0);
        let shoulder = needle.width * 2.0;
        let left = at(scale, shoulder, degrees - 90.0);
        let right = at(scale, shoulder, degrees + 90.0);
        let (lit, shadow, ring) = needle.colour.shades();
        for (side, colour) in [(left, lit), (right, shadow)] {
            scene.items.push(Item::Fill {
                points: vec![tip, side, tail],
                colour,
                alpha: 1.0,
            });
        }
        let centre = at(scale, 0.0, 0.0);
        let cap = circle(centre, needle.width * 3.0 * scale);
        scene.items.push(Item::Fill {
            points: cap.clone(),
            colour: needle.cap,
            alpha: 1.0,
        });
        let mut outline = cap;
        if let Some(&first) = outline.first() {
            outline.push(first);
        }
        scene.items.push(Item::Stroke {
            points: outline,
            width: 1.0,
            colour: ring,
            alpha: 1.0,
        });
    }
}

/// `tabPage1_Resize`: a page narrower than its height's half, or wider than nearly twice it, puts
/// the dials in a row a third of its width each where it is under 500 wide - speed, altitude,
/// heading, the vertical speed hidden - and a quarter where it is wider, the vertical speed
/// first; a squarer page puts them two by two, half its shorter side each. The speed dial's size
/// and place: (left, top, side).
/// `// C#: GCSViews/FlightData.cs:5331-5392`
#[must_use]
pub fn speed_place(width: f32, height: f32) -> (f32, f32, f32) {
    let ratio = width / height.max(1.0);
    if ratio > 0.5 && ratio < 1.9 {
        let side = (height.min(width) / 2.0).floor();
        return (side, 0.0, side);
    }
    if width < 500.0 {
        ((0.0), 0.0, (width / 3.0).floor())
    } else {
        let side = (width / 4.0).floor();
        (side, 0.0, side)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn labels(scene: &Scene) -> Vec<&str> {
        scene
            .items
            .iter()
            .filter_map(|item| match item {
                Item::Label { text, .. } => Some(text.as_str()),
                _ => None,
            })
            .collect()
    }

        /// The strokes that are radial lines - two points - as (length, width), whole pixels.
    fn lines(scene: &Scene) -> Vec<(u32, u32)> {
        scene
            .items
            .iter()
            .filter_map(|item| match item {
                Item::Stroke { points, width, .. } if points.len() == 2 => {
                    let (dx, dy) = (points[1].0 - points[0].0, points[1].1 - points[0].1);
                    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
                    Some((dx.hypot(dy).round() as u32, width.round() as u32))
                }
                _ => None,
            })
            .collect()
    }

    fn fills(scene: &Scene) -> usize {
        scene
            .items
            .iter()
            .filter(|item| matches!(item, Item::Fill { .. }))
            .count()
    }

        /// The first needle's tip: the first point of the first three-cornered fill (a band's outline
    /// and a cap have many more corners).
    fn first_tip(scene: &Scene) -> (f32, f32) {
        scene
            .items
            .iter()
            .find_map(|item| match item {
                Item::Fill { points, .. } if points.len() == 3 => points.first().copied(),
                _ => None,
            })
            .expect("a needle")
    }

    /// The Designer's dial: 0 to 60 in tens, and the caption.
    #[test]
    fn the_dial_is_numbered_to_its_maximum() {
        let gauge = SpeedGauge::default();
        let scene = gauge.scene(150.0, &[]);
        assert_eq!(
            labels(&scene),
            ["0", "10", "20", "30", "40", "50", "60", "Speed"]
        );
        // Seven major lines, nine minor after each but the last.
        let strokes = scene
            .items
            .iter()
            .filter(|item| matches!(item, Item::Stroke { .. }))
            .count();
        assert_eq!(strokes, 7 + 6 * 9);
        // 0 at the arc's start, 135 degrees clockwise from east: down and to the left.
        let Some(Item::Label { at, .. }) = scene
            .items
            .iter()
            .find(|item| matches!(item, Item::Label { text, .. } if text == "0"))
        else {
            panic!("no zero");
        };
        assert!(at.0 < 75.0 && at.1 > 75.0, "{at:?}");
    }

    /// A new maximum renumbers the dial; one that is not above the minimum is ignored, and text
    /// that is not a number is .NET's error.
    #[test]
    fn a_double_click_sets_the_maximum() {
        let mut gauge = SpeedGauge::default();
        assert_eq!(gauge.set_max("25"), Ok(()));
        assert!((gauge.max - 25.0).abs() < f32::EPSILON);
        assert_eq!(labels(&gauge.scene(150.0, &[])), ["0", "10", "20", "Speed"]);
        assert_eq!(gauge.set_max("0"), Ok(()));
        assert!((gauge.max - 25.0).abs() < f32::EPSILON);
        assert!(gauge.set_max("fast").is_err());
        assert!((gauge.max - 25.0).abs() < f32::EPSILON);
    }

    /// The needles are held to the scale, as the control holds its values.
    #[test]
    fn the_needles_stay_on_the_scale() {
        let gauge = SpeedGauge::default();
        let [air, ground] = gauge.needles(-3.0, 80.0);
        assert!((air.value - 0.0).abs() < f32::EPSILON);
        assert!((ground.value - 60.0).abs() < f32::EPSILON);
        assert_eq!(
            (air.colour, ground.colour),
            (NeedleColour::Gray, NeedleColour::Red)
        );
        // Drawn: two shaded halves and a cap each, and a ring round each cap.
        let scene = gauge.scene(130.0, &[air, ground]);
        assert_eq!(fills(&scene), 2 * 3);
        let rings = scene
            .items
            .iter()
            .filter(|item| matches!(item, Item::Stroke { points, .. } if points.len() > 2))
            .count();
        assert_eq!(rings, 2);
    }

    /// A needle points where its value falls on the scale: the yaw dial's 0 is straight up, its
    /// 90 to the right; the speed dial's maximum is at the arc's end, down and to the right.
    #[test]
    fn a_needle_points_at_its_value() {
        let yaw = yaw_dial();
        let up = first_tip(&yaw.scene(150.0, &[yaw.raw_needle(0.0)]));
        assert!((up.0 - 75.0).abs() < 0.01 && (up.1 - 25.0).abs() < 0.01, "{up:?}");
        let right = first_tip(&yaw.scene(150.0, &[yaw.raw_needle(90.0)]));
        assert!(
            (right.0 - 125.0).abs() < 0.5 && (right.1 - 75.0).abs() < 0.5,
            "{right:?}"
        );
        let speed = SpeedGauge::default();
        let [_, ground] = speed.needles(0.0, 60.0);
        let end = first_tip(&speed.scene(150.0, &[ground]));
        assert!(end.0 > 100.0 && end.1 > 100.0, "{end:?}");
        // At twice the size, twice as far out.
        let far = first_tip(&yaw.scene(300.0, &[yaw.raw_needle(90.0)]));
        assert!((far.0 - 250.0).abs() < 1.0 && (far.1 - 150.0).abs() < 1.0, "{far:?}");
    }

    /// The RAW Sensor dials' bands come first, under everything, one fill each where the end is
    /// past the start, then the base arc.
    #[test]
    fn the_bands_lie_under_the_scale() {
        let scene = roll_dial().scene(150.0, &[]);
        assert!(
            matches!(&scene.items[..3], [Item::Fill { .. }, Item::Fill { .. }, Item::Fill { .. }]),
            "three bands first"
        );
        assert!(
            matches!(&scene.items[3], Item::Stroke { points, colour, .. } if points.len() > 100 && *colour == GRAY),
            "then the gray base arc"
        );
        assert_eq!(fills(&yaw_dial().scene(150.0, &[])), 1);
        // A band whose end is not past its start is not drawn.
        let mut dial = yaw_dial();
        dial.bands = &[Band {
            from: 90.0,
            to: 90.0,
            inner: 50.0,
            outer: 60.0,
            colour: LIGHT_GREEN,
        }];
        assert_eq!(fills(&dial.scene(150.0, &[])), 0);
    }

    /// The pitch dial's nine minor lines between majors: the middle one is the longer inter line,
    /// 50 to 60 like a major but one wide; the others 50 to 55.
    #[test]
    fn the_middle_minor_line_is_the_longer_inter_line() {
                let drawn = lines(&pitch_dial().scene(150.0, &[]));
        let majors = drawn.iter().filter(|&&line| line == (10, 2)).count();
        let inters = drawn.iter().filter(|&&line| line == (10, 1)).count();
        let minors = drawn.iter().filter(|&&line| line == (5, 1)).count();
        // -90 to 89 in twenties: nine majors, nine inter lines and 72 minor lines, the last
        // major's nine minors running to 88.
        assert_eq!((majors, inters, minors), (9, 9, 72));
                assert_eq!(drawn.len(), 9 + 9 + 72);
    }

    /// A scale that is no whole number of steps runs its minor lines on past the last major:
    /// roll's -180 to 179 in thirties has twelve majors and the five minors after 150 as well.
    #[test]
    fn the_scale_runs_to_its_end() {
        let scene = roll_dial().scene(150.0, &[]);
        let numbers: Vec<&str> = labels(&scene);
        assert_eq!(numbers.len(), 12 + 1, "{numbers:?}");
        assert_eq!(numbers[0], "-180");
        assert_eq!(numbers[11], "150");
        assert_eq!(numbers[12], "Roll");
                let drawn = lines(&scene);
        assert_eq!(drawn.len(), 12 + 12 * 5);
        // The yaw dial: eight majors, two minors each, the last pair past 315.
        let yaw = lines(&yaw_dial().scene(150.0, &[]));
        assert_eq!(yaw.len(), 8 + 8 * 2);
    }

    /// `tabPage1_Resize`'s places: this column's page is tall and under 500 wide, so the speed
    /// dial is a third of its width, first in the row.
    #[test]
    fn the_speed_dial_is_placed_as_the_page_resize_places_it() {
        assert_eq!(speed_place(392.0, 800.0), (0.0, 0.0, 130.0));
        assert_eq!(speed_place(800.0, 200.0), (200.0, 0.0, 200.0));
        assert_eq!(speed_place(300.0, 300.0), (150.0, 0.0, 150.0));
    }
}
