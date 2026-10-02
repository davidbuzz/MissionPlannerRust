//! The `AGauge` dials: the Gauges page's speed dial, `Gspeed`, with two needles, airspeed and
//! ground speed, over a scale from 0 to its maximum - 60 until a double click asks for another -
//! and the RAW Sensor window's roll, pitch and yaw dials, each with one needle.
//!
//! `AGauge` draws in a 150-pixel square scaled to the control: the coloured bands of its ranges,
//! the base arc, a major line and a number every step, the minor lines between with the middle
//! one longer when there is one, the caption, and each needle as three shaded triangles about
//! the centre over a cap. The C# draws its dial face from a `BackgroundImage` in the `.resx`;
//! the face is not drawn here, under the palette ruling, and the scale and needles are drawn on
//! the page's own background - in white where the Designer has them black, which the dark
//! background would swallow.
//! `// C#: ExtLibs/Controls/AGauge.cs:1499-1987, GCSViews/FlightData.Designer.cs:1464-1607,
//! Controls/RAW_Sensor.Designer.cs:405-547, 609-750, 752-893`
//!
//! Galt, Gheading and Gvspeed, the Gauges page's other dials, are not ported.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use crate::hud::{Align, Item, Scene};

/// `basesize`: the square every `AGauge` draws in before it is scaled to its size.
/// `// C#: ExtLibs/Controls/AGauge.cs:2010`
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

/// `AGauge.NeedleColorEnum`, the needles' shading.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NeedleColour {
    /// `Gray`.
    Gray,
    /// `Red`.
    Red,
}

/// One needle: its value, its length, its width, its shading and its cap's colour.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Needle {
    /// `Value0` to `Value3`, as bound.
    pub value: f32,
    /// `NeedlesRadius`.
    pub radius: f32,
    /// `NeedlesWidth`.
    pub width: f32,
    /// `NeedlesColor1`.
    pub colour: NeedleColour,
    /// `NeedlesColor2`.
    pub cap: u32,
}

/// `RangesEnabled[i]`: a coloured band between two radii from one value of the scale to another,
/// drawn under the scale when its end is past its start.
/// `// C#: ExtLibs/Controls/AGauge.cs:1588-1614`
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
    /// and one wide, both 50 long with white caps. Each value is held to the scale, as the
    /// `Value` setters hold it. `// C#: ExtLibs/Controls/AGauge.cs:240-250`
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
    /// minimum, and left as it was otherwise; text that is not a number throws in the C#, which
    /// has no `catch` here, and is .NET's message. `// C#: GCSViews/FlightData.cs:3140-3148,
    /// ExtLibs/Controls/AGauge.cs:446-462`
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
    /// `// C#: ExtLibs/Controls/AGauge.cs:1499-1987`
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

impl Dial {
    /// A value held to the scale, as the `Value` setters hold it.
    /// `// C#: ExtLibs/Controls/AGauge.cs:240-250`
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

    /// Where a value of the scale points, in degrees clockwise from east.
    fn angle_of(&self, value: f32) -> f32 {
        self.arc_start + (value - self.min) * self.arc_sweep / (self.max - self.min)
    }

    /// The dial drawn `size` pixels square: the bands, the base arc, the scale, the caption and
    /// the needles. `// C#: ExtLibs/Controls/AGauge.cs:1499-1987`
    #[must_use]
    pub fn scene(&self, size: f32, needles: &[Needle]) -> Scene {
        let scale = size / BASE_SIZE;
        let mut scene = Scene::default();
        let span = self.max - self.min;
        let at = |radius: f32, degrees: f32| {
            let radians = degrees.to_radians();
            (
                (CENTRE.0 + radius * radians.cos()) * scale,
                (CENTRE.1 + radius * radians.sin()) * scale,
            )
        };
        // An arc as points, one every five degrees and the end.
        let arc = |radius: f32, from: f32, sweep: f32| -> Vec<(f32, f32)> {
            let sweep = sweep.min(360.0);
            #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // whole steps
            let steps = ((sweep / 5.0).ceil() as usize).max(1);
            #[allow(clippy::cast_precision_loss)] // small counts
            (0..=steps)
                .map(|step| at(radius, from + sweep * step as f32 / steps as f32))
                .collect()
        };
        // `AddPie` outer, `AddPie` inner reversed, `FillPie` clipped to the two: the band.
        for band in self.bands {
            if band.to <= band.from {
                continue;
            }
            let from = self.angle_of(band.from);
            let sweep = (band.to - band.from) * self.arc_sweep / span;
            let mut points = arc(band.outer, from, sweep);
            let mut inner = arc(band.inner, from, sweep);
            inner.reverse();
            points.append(&mut inner);
            scene.items.push(Item::Fill {
                points,
                colour: band.colour,
                alpha: 1.0,
            });
        }
        // `DrawArc` with the base arc's pen.
        if let Some((radius, width, colour)) = self.base_arc {
            scene.items.push(Item::Stroke {
                points: arc(radius, self.arc_start, self.arc_sweep),
                width: width * scale,
                colour,
                alpha: 1.0,
            });
        }
        let line = |scene: &mut Scene,
                    (inner, outer, width): (f32, f32, f32),
                    colour: u32,
                    degrees: f32| {
            scene.items.push(Item::Stroke {
                points: vec![at(inner, degrees), at(outer, degrees)],
                width: width * scale,
                colour,
                alpha: 1.0,
            });
        };
        let (major_colour, minor_colour, numbers_colour) = self.colours;
        // `(m_MaxValue - m_MinValue) / m_ScaleLinesMajorStepValue * (MinorNumOf + 1)` minor
        // steps across the sweep.
        let minor_step =
            self.arc_sweep / ((span / self.major_step) * (f32::from(self.minor_count) + 1.0));
        let mut count = 0.0f32;
        while count <= span {
            let major = self.arc_start + count * self.arc_sweep / span;
            line(&mut scene, self.major, major_colour, major);
            if count < span {
                for minor in 1..=self.minor_count {
                    let degrees = major + f32::from(minor) * minor_step;
                    // With an odd count, the middle one is the longer "inter" line.
                    let middle = self.minor_count % 2 == 1 && self.minor_count / 2 + 1 == minor;
                    if middle {
                        line(&mut scene, self.inter, major_colour, degrees);
                    } else {
                        line(&mut scene, self.minor, minor_colour, degrees);
                    }
                }
            }
            // The number, centred on its point at the numbers' radius.
            let (x, y) = at(self.numbers_radius, major);
            scene.items.push(Item::Label {
                text: number_text(self.min + count),
                at: (x, y - FONT * scale / 2.0),
                size: FONT * scale,
                colour: numbers_colour,
                align: Align::Centre,
            });
            count += self.major_step;
        }
        scene.items.push(Item::Label {
            text: self.cap.0.to_owned(),
            at: (self.cap.1.0 * scale, self.cap.1.1 * scale),
            size: FONT * scale,
            colour: numbers_colour,
            align: Align::Left,
        });
        for needle in needles {
            self.needle(&mut scene, needle, scale);
        }
        scene
    }

    /// A needle of type 0: its cap, three shaded triangles about the centre, and two lines in
    /// the cap's colour. `// C#: ExtLibs/Controls/AGauge.cs:1818-1946`
    fn needle(&self, scene: &mut Scene, needle: &Needle, scale: f32) {
        #[allow(clippy::cast_possible_truncation)] // `(Int32)(...) % 360`
        let brush_angle = (self.angle_of(needle.value) as i32 % 360) as f32;
        let angle = f64::from(brush_angle).to_radians();
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let subcol = (((brush_angle + 225.0) % 180.0) * 100.0 / 180.0) as i32;
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let subcol2 = (((brush_angle + 135.0) % 180.0) * 100.0 / 180.0) as i32;
        let grey = |level: i32| {
            let level = u32::try_from(level.clamp(0, 255)).unwrap_or(0);
            (level << 16) | (level << 8) | level
        };
        let rgb = |r: i32, g: i32, b: i32| {
            let byte = |v: i32| u32::try_from(v.clamp(0, 255)).unwrap_or(0);
            (byte(r) << 16) | (byte(g) << 8) | byte(b)
        };
        let (mut brush1, mut brush2, brush3, mut brush4, outline) = match needle.colour {
            NeedleColour::Gray => (
                grey(80 + subcol),
                grey(180 - subcol),
                grey(((80 + subcol2) + 255) % 255),
                grey(((180 - subcol2) + 255) % 255),
                0x80_80_80,
            ),
            NeedleColour::Red => (
                rgb(145 + subcol, subcol, subcol),
                rgb(245 - subcol, 100 - subcol, 100 - subcol),
                rgb(145 + subcol2, subcol2, subcol2),
                rgb(245 - subcol2, 100 - subcol2, 100 - subcol2),
                0xff_00_00,
            ),
        };
        if (((brush_angle + 225.0) % 360.0) / 180.0).floor() == 0.0 {
            std::mem::swap(&mut brush1, &mut brush2);
        }
        if (((brush_angle + 135.0) % 360.0) / 180.0).floor() == 0.0 {
            brush4 = brush3;
        }
        let radius = f64::from(needle.radius);
        let width = f64::from(needle.width);
        let (cx, cy) = (f64::from(CENTRE.0), f64::from(CENTRE.1));
        let point = |along: f64, across: f64, turn: f64| {
            #[allow(clippy::cast_possible_truncation)] // the C#'s `(Single)`
            let point = (
                ((cx + along * angle.cos() + across * (angle + turn).cos()) as f32) * scale,
                ((cy + along * angle.sin() + across * (angle + turn).sin()) as f32) * scale,
            );
            point
        };
        let half = std::f64::consts::FRAC_PI_2;
        // `m_NeedleRadius / 20` and `/ 5`: integer divisions in the C#, the radius an `int`.
        #[allow(clippy::cast_possible_truncation)]
        let whole = needle.radius as i32;
        let (tail, fifth) = (f64::from(whole / 20), f64::from(whole / 5));
        let tip = point(radius, 0.0, 0.0);
        let back = point(-tail, 0.0, 0.0);
        let side_a = point(-fifth, width * 2.0, half);
        let side_b = point(-fifth, width * 2.0, -half);
        let fill = |scene: &mut Scene, points: Vec<(f32, f32)>, colour: u32| {
            scene.items.push(Item::Fill {
                points,
                colour,
                alpha: 1.0,
            });
        };
        // The cap under the needle: `FillEllipse` in the cap's colour, then its outline.
        #[allow(clippy::cast_possible_truncation)] // the centre, 75
        let centre = (cx as f32 * scale, cy as f32 * scale);
        let cap = circle(centre, needle.width * 3.0 * scale);
        fill(scene, cap.clone(), needle.cap);
        let mut ring = cap;
        if let Some(&first) = ring.first() {
            ring.push(first);
        }
        scene.items.push(Item::Stroke {
            points: ring,
            width: 1.0,
            colour: outline,
            alpha: 1.0,
        });
        fill(scene, vec![tip, back, side_a], brush1);
        fill(scene, vec![tip, back, side_b], brush2);
        let inner = point(-(tail - 1.0), 0.0, 0.0);
        fill(scene, vec![inner, side_a, side_b], brush4);
        for end in [back, tip] {
            scene.items.push(Item::Stroke {
                points: vec![centre, end],
                width: 1.0,
                colour: needle.cap,
                alpha: 1.0,
            });
        }
    }
}

/// `Center`, in the base square: every dial here has it at (75, 75).
const CENTRE: (f32, f32) = (75.0, 75.0);

/// A scale number as `float.ToString()` writes it: "0", "10", "12.5".
fn number_text(value: f32) -> String {
    value.to_string()
}

/// A circle as a polygon.
fn circle((x, y): (f32, f32), radius: f32) -> Vec<(f32, f32)> {
    (0..24)
        .map(|step| {
            #[allow(clippy::cast_precision_loss)]
            let angle = step as f32 * std::f32::consts::TAU / 24.0;
            (x + radius * angle.cos(), y + radius * angle.sin())
        })
        .collect()
}

/// `tabPage1_Resize`: a page narrower than its height's half, or wider than nearly twice it, puts
/// the dials in a row a third of its width each where it is under 500 wide - speed, altitude,
/// heading, the vertical speed hidden - and a quarter where it is wider, the vertical speed
/// first; a squarer page puts them two by two, half its shorter side each. The speed dial's size
/// and place: (left, top, side).
/// `// C#: GCSViews/FlightData.cs:5217-5278`
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

    /// The needles are held to the scale, as the C#'s value setters hold them.
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
        // Drawn: three triangles, a cap and its ring, and two lines each.
        let scene = gauge.scene(130.0, &[air, ground]);
        let fills = scene
            .items
            .iter()
            .filter(|item| matches!(item, Item::Fill { .. }))
            .count();
        assert_eq!(fills, 2 * 4);
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
