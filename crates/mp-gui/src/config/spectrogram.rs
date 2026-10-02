//! The spectrogram window: `Controls/SpectrogramUI.cs`, which the Advanced page's Spectrogram
//! opens (`ConfigAdvanced.BUT_spect_Click`, `GCSViews/ConfigurationView/ConfigAdvanced.cs:119-122`:
//! `new SpectrogramUI().Show()`), and the images it draws, `Spectrogram.GenerateImage`
//! (`ExtLibs/Utilities/Spectrogram.cs:22-329`).
//!
//! What it shows, as the Designer lays out its 800 x 450 form (`SpectrogramUI.Designer.cs:30-188`;
//! the `.resx` holds nothing): along the top Load Log, the sensor's combo box (ACC1 to ACC5, GYR1
//! to GYR5, "ACC1" in it), Min (-80) and Max (-20), and Update; under them one ZedGraph control
//! holding three graphs, "X", "Y" and "Z", each "T" across and "Frequency" up
//! (`SpectrogramUI.cs:25-48`), laid out two by two as `MasterPane.DoLayout` lays out three
//! (`ExtLibs/ZedGraph/ZedGraph/MasterPane.cs:937-1007`, `SquareColPreferred`), ten pixels apart
//! and with no margin (`ZedGraphControl.cs:523-526`).
//!
//! What it does:
//!
//! * Load Log asks for a log (`*.log;*.bin`) and reads it, drawing nothing (`:50-64`);
//! * a sensor chosen from the list (`cmb_sensor_SelectedIndexChanged`), Min or Max changed
//!   (`num_min_ValueChanged`, `num_max_ValueChanged`) and Update (`but_redraw_Click`) each draw
//!   (`:71-149`): the box's text names the message - the box is a `DropDown` combo, so it may be
//!   typed into, IMU say - and its AccX, AccY and AccZ, or GyrX, GyrY and GyrZ for a name with GYR
//!   in it, are made images by [`generate`] and drawn into X, Y and Z: each image stretched over
//!   its graph from the first transform's time to the last one's, in seconds, and from 0 to the
//!   highest frequency, the frequency axis stepped every 20 with its grid shown over the image;
//! * [`generate`] (`Spectrogram.cs:22-248`), for a log with `ISBH` - the IMU's batch samples - or
//!   for one without:
//!   - with: the samples of the `ISBD` batches whose header names the sensor - instance 0, 1 or 2
//!     for a name with a 1, a 2, or neither in it, the accelerometer for a name with ACC in it
//!     and the gyro otherwise - each its batch's `TimeUS`, divided by the header's `mul`; the
//!     frequencies of the header's `smp_rate`; windows of `N` that do not overlap;
//!   - without: every record of the message (`GetEnumeratorType`, so ACC1 is ACC1 and ACC
//!     instance 0), its time `TimeUS` - `SampleUS` where its first record has no `TimeUS` - and
//!     the field, a field it lacks read as 0 (`Convert.ToDouble(null)`); windows of `N` a quarter
//!     window apart (the C#'s comment says 50 % overlap), or apart by a window where there are more
//!     than 2048 windows' worth; the rate estimated from the windows' spans, an average that starts
//!     at zero and moves a hundredth of the way each window, as `(int)(1e6 * N / span)`;
//!   - each window, `N = 1 << 10` samples, is `FFT2.rin` - a Hann window, decibels - and its `N /
//!     2` bins become a column of the image, the lowest bin at the bottom, each coloured by
//!     [`colour`]: the decibels mapped from Min..Max onto 0..255 and constrained, and the hue
//!     `(255 - that) / 255` of an HSL colour of saturation and lightness 0.5 - grey at or below
//!     Min, red at or above Max. The image is as wide as `N`-sample windows fit the samples,
//!     times four where they are a quarter window apart, so its last three columns are empty.
//!
//! Where this is not the C#, each written at its site:
//!
//! * the form is drawn over SETUP, modal, where the C#'s is a free form (`Show`, not
//!   `ShowDialog`); a second click opens a fresh one. It does not resize (`SpectrogramUI_Resize`
//!   is empty anyway);
//! * Load Log's dialog is a typed path ([`crate::config::firmware::PathBox`]); the reading and
//!   the drawing run off the UI thread, the controls dimmed meanwhile, where the C# holds the form
//!   until they are done;
//! * what the C# throws - a draw with no log loaded, a message the log has none of, too few
//!   samples for one window - goes on the status line, as the owner ruled (2026-09-25); the C#
//!   shows its unhandled-exception box. The graphs keep what they showed, as the C#'s do, since
//!   they are cleared only after all three images are made;
//! * with `ISBH` the C# caches the samples by log and sensor but not by axis, so after the first
//!   draw its Y and Z show X's samples; here each shows its own (`Spectrogram.cs:40, 90-91, 103`).
//!   The cache, ten minutes long, is otherwise a saving of time, and the log is read again here;
//! * the header's `smp_rate` and `mul` are taken as the log holds them, where the C# parses them
//!   back from their text;
//! * the image is drawn by the GPU, stretched as GDI+ stretches it; the axes are ZedGraph's
//!   scales ([`crate::plan::elevation::Scale`]) without its measured margins, the Y title written
//!   level, and a frequency label that would overlap the one below it left out;
//! * an image wider than [`MAX_TEXTURE_COLUMNS`] is drawn from every so-many column, which at the
//!   graph's few hundred pixels shows the same;
//! * the colours of the controls are this application's.
//!
//! The Ctrl+L key of the main window opens this form too (`MainV2.cs:4138-4144`); that key is the
//! main window's, not ported here. Of `ImageSharpExtensions` (`SpectrogramUI.cs:152-178`),
//! `ToBitmap` hands the image to ZedGraph, which a texture does here, and `ToImageSharpImage` has
//! no callers (`ledger/dead-functions.csv`): not ported.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::cell::RefCell;
use std::path::Path;
#[cfg(test)]
use std::path::PathBuf;
use std::rc::Rc;
use std::sync::Arc;
use std::sync::mpsc::{Receiver, TryRecvError};

use gpui::{
    AnyElement, Context, Corners, FocusHandle, KeyDownEvent, RenderImage, SharedString, Window,
    canvas, div, prelude::*, px, rgb,
};
use mp_log::dataflash::{LogMessage, Value};
use mp_log::fft;
use mp_log::logfile::LogFile;

use super::fftui::{LOG_FILTER, OUT_OF_RANGE, records};
use super::firmware::{BoxIds, PathBox, path_box};
use super::motor_test::NumericUpDown;
use super::optional::{at, button, label};
use super::servo_output::{Combo, dropdown};
use crate::MissionPlanner;
use crate::plan::elevation::Scale;
use crate::textfield::{KeyOutcome, TextField};
use crate::ui::theme;

// ---------------------------------------------------------------------------------------------
// The Designer's words and places.
// ---------------------------------------------------------------------------------------------

/// The form's caption, `this.Text`. `// C#: Controls/SpectrogramUI.Designer.cs:181`
pub const FORM_TEXT: &str = "SpectrogramUI";

/// `ClientSize`. `// C#: Controls/SpectrogramUI.Designer.cs:171`
pub const CLIENT: (f32, f32) = (800.0, 450.0);

/// `but_loadlog`. `// C#: Controls/SpectrogramUI.Designer.cs:47-53`
pub const LOAD_LOG: &str = "Load Log";
/// Its place.
pub const LOAD_AT: (f32, f32, f32, f32) = (15.0, 6.0, 75.0, 23.0);

/// `cmb_sensor`: its place. `// C#: Controls/SpectrogramUI.Designer.cs:57-74`
pub const SENSOR_AT: (f32, f32, f32, f32) = (96.0, 6.0, 93.0, 21.0);
/// `cmb_sensor.Items`.
pub const SENSORS: [&str; 10] = [
    "ACC1", "ACC2", "ACC3", "ACC4", "ACC5", "GYR1", "GYR2", "GYR3", "GYR4", "GYR5",
];
/// `cmb_sensor.Text`, which selects the first item.
pub const SENSOR_TEXT: &str = "ACC1";

/// `label1.Text` and its place. `// C#: Controls/SpectrogramUI.Designer.cs:141-146`
pub const MIN_TEXT: &str = "Min";
/// Its place.
const MIN_LABEL_AT: (f32, f32) = (208.0, 11.0);
/// `label2.Text`. `// C#: Controls/SpectrogramUI.Designer.cs:150-155`
pub const MAX_TEXT: &str = "Max";
/// Its place.
const MAX_LABEL_AT: (f32, f32) = (307.0, 11.0);

/// `num_min`: -80, from -1000 to 1000. `// C#: Controls/SpectrogramUI.Designer.cs:95-114`
pub const MIN_AT: (f32, f32, f32, f32) = (238.0, 7.0, 63.0, 20.0);
/// `num_min.Value`.
pub const MIN_DEFAULT: f64 = -80.0;
/// `num_max`: -20, from -1000 to 1000. `// C#: Controls/SpectrogramUI.Designer.cs:118-137`
pub const MAX_AT: (f32, f32, f32, f32) = (340.0, 7.0, 63.0, 20.0);
/// `num_max.Value`.
pub const MAX_DEFAULT: f64 = -20.0;
/// Both boxes' `Minimum` and `Maximum`.
pub const NUMBER_BOUNDS: (f64, f64) = (-1000.0, 1000.0);

/// `but_redraw`. `// C#: Controls/SpectrogramUI.Designer.cs:159-165`
pub const UPDATE: &str = "Update";
/// Its place.
pub const UPDATE_AT: (f32, f32, f32, f32) = (409.0, 6.0, 75.0, 23.0);

/// `zedGraphControl1`. `// C#: Controls/SpectrogramUI.Designer.cs:78-91`
pub const GRAPH_AT: (f32, f32, f32, f32) = (12.0, 35.0, 776.0, 403.0);

/// The three panes' titles. `// C#: Controls/SpectrogramUI.cs:29, 34, 39`
pub const PANE_TITLES: [&str; 3] = ["X", "Y", "Z"];
/// Each pane's X title. `// C#: Controls/SpectrogramUI.cs:30, 35, 40`
pub const X_TITLE: &str = "T";
/// Each pane's Y title. `// C#: Controls/SpectrogramUI.cs:31, 36, 41`
pub const Y_TITLE: &str = "Frequency";
/// `YAxis.Scale.MajorStep = 20`, the grid shown. `// C#: Controls/SpectrogramUI.cs:120-125`
pub const Y_STEP: f64 = 20.0;

/// `MasterPane.Default.InnerPaneGap`. `// C#: ExtLibs/ZedGraph/ZedGraph/MasterPane.cs:144`
pub const PANE_GAP: f32 = 10.0;

/// The fields of a name with GYR in it, and of any other.
/// `// C#: Controls/SpectrogramUI.cs:73-78`
pub const GYR_FIELDS: [&str; 3] = ["GyrX", "GyrY", "GyrZ"];
/// The others'.
pub const ACC_FIELDS: [&str; 3] = ["AccX", "AccY", "AccZ"];

/// `int bins = 10`. `// C#: ExtLibs/Utilities/Spectrogram.cs:116, 172`
pub const BINS: u32 = 10;
/// `int N = 1 << bins`: the samples of one transform.
pub const N: usize = 1 << BINS;

/// What `file` being null throws when a draw comes before Load Log: .NET's
/// `NullReferenceException`.
pub const NO_LOG: &str = "Object reference not set to an instance of an object.";

/// `Enumerable.Min` of no elements: `InvalidOperationException`.
pub const NO_ELEMENTS: &str = "Sequence contains no elements";

/// The widest image drawn whole; a wider one is drawn from every so-many column.
pub const MAX_TEXTURE_COLUMNS: usize = 2048;

// ---------------------------------------------------------------------------------------------
// The colours.
// ---------------------------------------------------------------------------------------------

/// `MathHelper.mapConstrained`, with .NET's `Math.Max` and `Math.Min`, which keep a NaN.
/// `// C#: ExtLibs/Utilities/Math.cs:32-38`
#[must_use]
pub fn map_constrained(x: f64, in_min: f64, in_max: f64, out_min: f64, out_max: f64) -> f64 {
    let output = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    if output.is_nan() {
        return output;
    }
    output.max(out_min).min(out_max)
}

/// `Convert.ToByte(x * 255)`: rounded to even.
#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // 0..=255 by construction
fn to_byte(fraction: f64) -> u8 {
    (fraction * 255.0).round_ties_even().clamp(0.0, 255.0) as u8
}

/// `Spectrogram.HSL2RGB`: hue, saturation and lightness, each 0 to 1, as red, green and blue -
/// grey, the lightness, where the hue's sextant is none of the six (a hue of 1).
/// `// C#: ExtLibs/Utilities/Spectrogram.cs:257-320`
#[must_use]
pub fn hsl_to_rgb(h: f64, sl: f64, l: f64) -> [u8; 3] {
    let (mut r, mut g, mut b) = (l, l, l);
    let v = if l <= 0.5 {
        l * (1.0 + sl)
    } else {
        l + sl - l * sl
    };
    if v > 0.0 {
        let m = l + l - v;
        let sv = (v - m) / v;
        let h = h * 6.0;
        #[allow(clippy::cast_possible_truncation)] // `(int) h`, a hue of 0..1 times six
        let sextant = h as i32;
        let fract = h - f64::from(sextant);
        let vsf = v * sv * fract;
        let mid1 = m + vsf;
        let mid2 = v - vsf;
        match sextant {
            0 => (r, g, b) = (v, mid1, m),
            1 => (r, g, b) = (mid2, v, m),
            2 => (r, g, b) = (m, v, mid1),
            3 => (r, g, b) = (m, mid2, v),
            4 => (r, g, b) = (mid1, m, v),
            5 => (r, g, b) = (v, m, mid2),
            _ => {}
        }
    }
    [to_byte(r), to_byte(g), to_byte(b)]
}

/// `Spectrogram.GetColor`: decibels from `min` to `max` mapped onto 0 to 255 and constrained,
/// and `GetRainbowColor((byte)(255 - that))` - the hue `i / 255` at saturation and lightness 0.5.
/// `// C#: ExtLibs/Utilities/Spectrogram.cs:250-255, 322-329`
#[must_use]
pub fn colour(value: f64, min: i32, max: i32) -> [u8; 3] {
    let scale = map_constrained(value, f64::from(min), f64::from(max), 0.0, 255.0);
    // `(byte)`: truncated, a NaN 0, as `as` makes it.
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let i = (255.0 - scale) as u8;
    hsl_to_rgb(f64::from(i) / 255.0, 0.5, 0.5)
}

/// The colour of a bin at or above Max: hue 0.
pub const HOT: [u8; 3] = [191, 64, 64];
/// The colour of a bin at or below Min: hue 1, no sextant, the lightness.
pub const COLD: [u8; 3] = [128, 128, 128];

/// C#'s `(int)` of a double, as .NET Framework on x86 and x64 converts: truncated, and a value
/// out of range or NaN `int.MinValue`.
#[must_use]
pub fn dotnet_int(value: f64) -> i32 {
    if value.is_nan() || value >= 2_147_483_648.0 || value <= -2_147_483_649.0 {
        return i32::MIN;
    }
    #[allow(clippy::cast_possible_truncation)] // in range, checked above
    let truncated = value.trunc() as i32;
    truncated
}

// ---------------------------------------------------------------------------------------------
// The images.
// ---------------------------------------------------------------------------------------------

/// An `Image<Rgba32>`: each pixel a colour, or nothing where no column was drawn (transparent).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Image {
    /// Columns: one a window.
    pub width: usize,
    /// Rows: one a bin, the highest at the top.
    pub height: usize,
    /// Row by row from the top.
    pixels: Vec<Option<[u8; 3]>>,
}

impl Image {
    /// `new Image<Rgba32>(width, height)`, which refuses an empty side.
    fn new(width: usize, height: usize) -> Result<Self, String> {
        if width == 0 || height == 0 {
            return Err(format!(
                "an image of {width} x {height}: too few samples for one {N}-sample window"
            ));
        }
        Ok(Self {
            width,
            height,
            pixels: vec![None; width * height],
        })
    }

    /// The pixel at a column and row, `None` where nothing was drawn or past the edge.
    #[must_use]
    pub fn pixel(&self, x: usize, y: usize) -> Option<[u8; 3]> {
        if x >= self.width {
            return None;
        }
        self.pixels.get(y * self.width + x).copied().flatten()
    }

    /// `img[x, y] = colour`.
    fn set(&mut self, x: usize, y: usize, colour: [u8; 3]) {
        if x < self.width
            && let Some(pixel) = self.pixels.get_mut(y * self.width + x)
        {
            *pixel = Some(colour);
        }
    }

    /// How many pixels are `colour`.
    #[must_use]
    pub fn count(&self, colour: [u8; 3]) -> usize {
        self.pixels
            .iter()
            .filter(|pixel| **pixel == Some(colour))
            .count()
    }

    /// The pixels as the GPU takes them - BGRA, nothing transparent - from every so-many column
    /// when wider than [`MAX_TEXTURE_COLUMNS`].
    #[must_use]
    pub fn texture(&self) -> image::RgbaImage {
        let step = self.width.div_ceil(MAX_TEXTURE_COLUMNS).max(1);
        let width = self.width.div_ceil(step);
        let mut out = image::RgbaImage::new(
            u32::try_from(width).unwrap_or(1),
            u32::try_from(self.height).unwrap_or(1),
        );
        for (y, row) in (0..self.height).zip(0_u32..) {
            for (x, column) in (0..width).zip(0_u32..) {
                let bgra = self
                    .pixel(x * step, y)
                    .map_or([0, 0, 0, 0], |[r, g, b]| [b, g, r, 255]);
                if let Some(pixel) = out.get_pixel_mut_checked(column, row) {
                    pixel.0 = bgra;
                }
            }
        }
        out
    }
}

/// One `GenerateImage`'s results: the image, `freqtout` and `allfftdata`.
#[derive(Debug, Clone, PartialEq)]
pub struct Generated {
    /// The field it is of.
    pub field: &'static str,
    /// The image.
    pub image: Image,
    /// `freqtout`: each bin's frequency, `FFT2.FreqTable`.
    pub freqs: Vec<f64>,
    /// `allfftdata`: each window's first time, in microseconds, and its spectrum in decibels.
    pub columns: Vec<(f64, Vec<f64>)>,
}

/// What a draw leaves in the three graphs.
#[derive(Debug, Clone, PartialEq)]
pub struct Drawn {
    /// What the box said: the message.
    pub sensor: String,
    /// X, Y and Z.
    pub panes: Vec<Generated>,
    /// `allfftdata.Min(a => a.timeus) / 1000000.0`: where the image starts, in seconds.
    pub mintime: f64,
    /// `allfftdata.Max(...)`: where it ends.
    pub maxtime: f64,
    /// `freqt.Max()`: the top of the frequency axis.
    pub freqmax: f64,
}

/// A record's field as `Convert.ToDouble(item.GetRaw(field))`: a field the message lacks is
/// `GetRaw`'s null, which converts to 0.
fn convert(message: &LogMessage, field: &str) -> Result<f64, String> {
    match message.field(field) {
        None => Ok(0.0),
        Some(Value::Text(text)) => mp_log::netfmt::parse_double(text).ok_or_else(|| {
            format!(
                "{}.{field} {text:?}: {}",
                message.name,
                super::fftui::NOT_A_NUMBER
            )
        }),
        Some(value) => value
            .as_f64()
            .ok_or_else(|| format!("{}.{field}: not a number", message.name)),
    }
}

/// A field as `item.items[cb.dflog.FindMessageOffset(msgtype, name)]` parsed: one the message
/// lacks is the index -1, which throws.
fn number(message: &LogMessage, field: &str) -> Result<f64, String> {
    message
        .field(field)
        .and_then(Value::as_f64)
        .ok_or_else(|| OUT_OF_RANGE.to_owned())
}

/// `int.Parse` of a field's text: a whole number.
#[allow(clippy::cast_possible_truncation)] // a byte or a ushort field
fn whole(message: &LogMessage, field: &str) -> Result<i64, String> {
    number(message, field).map(|value| value as i64)
}

/// What [`transform`] makes: the image, each window's first time and spectrum, and the average
/// of the windows' spans.
type Transformed = (Image, Vec<(f64, Vec<f64>)>, f64);

/// The windows' loop, `foreach (var fftdata in data.Windowed(N, divisor))`: each full window of
/// `N` samples, `N / divisor` apart, transformed and drawn as a column of an image `count` wide
/// and `height` high, its first time kept with its spectrum. Also the average of the windows'
/// spans the message path estimates its rate from.
/// `// C#: ExtLibs/Utilities/Spectrogram.cs:132-164, 201-240; ExtLibs/Utilities/Extensions.cs:204-237`
#[allow(clippy::too_many_arguments)]
fn transform(
    total: usize,
    sample: impl Fn(usize) -> Result<(f64, f64), String>,
    count: usize,
    divisor: usize,
    height: usize,
    min: i32,
    max: i32,
) -> Result<Transformed, String> {
    let mut image = Image::new(count, height)?;
    let mut columns = Vec::new();
    let mut timedelta = 0.0_f64;
    let step = (N / divisor).max(1);
    let mut start = 0;
    let mut done = 0;
    while start + N <= total {
        let mut first = f64::MAX;
        let mut last = f64::MIN;
        let mut data = Vec::with_capacity(N);
        for index in start..start + N {
            let (time, value) = sample(index)?;
            first = first.min(time);
            last = last.max(time);
            data.push(value);
        }
        timedelta = timedelta * 0.99 + (last - first) * 0.01;
        let spectrum = fft::rin(&data, true).ok_or_else(|| format!("{N} samples"))?;
        for (i, bin) in spectrum.iter().take(height).enumerate() {
            image.set(done, height - 1 - i, colour(*bin, min, max));
        }
        columns.push((first, spectrum));
        done += 1;
        start += step;
    }
    Ok((image, columns, timedelta))
}

/// `GenerateImage` for a log with `ISBH`: the batch samples of the sensor the name picks, one
/// image for each axis.
/// `// C#: ExtLibs/Utilities/Spectrogram.cs:29-169`
fn batches(
    log: &LogFile,
    kind: &str,
    fields: [&'static str; 3],
    min: i32,
    max: i32,
) -> Result<Vec<Generated>, String> {
    let sensorno: i64 = if kind.contains('1') {
        0
    } else if kind.contains('2') {
        1
    } else {
        2
    };
    let sensor: i64 = if kind.contains("ACC") { 0 } else { 1 };
    let (mut ns, mut type1, mut instance) = (-1_i64, -1_i64, -1_i64);
    let mut multiplier = -1.0_f64;
    let mut sample_rate = -1.0_f64;
    let mut axes: [Vec<(f64, f64)>; 3] = Default::default();
    for (item, _) in records(log, &["ISBH", "ISBD"]) {
        if item.name == "ISBH" {
            ns = whole(&item, "N")?;
            type1 = whole(&item, "type")?;
            instance = whole(&item, "instance")?;
            if instance != sensorno || type1 != sensor {
                continue;
            }
            sample_rate = number(&item, "smp_rate")?;
            multiplier = number(&item, "mul")?;
        } else if item.name.starts_with("ISBD") {
            if ns != whole(&item, "N")? || instance != sensorno || type1 != sensor {
                continue;
            }
            let time = number(&item, "TimeUS")?;
            // `item.GetRaw(field.ToLower().Substring(field.Length - 1))`: x, y or z.
            for (axis, data) in ["x", "y", "z"].into_iter().zip(axes.iter_mut()) {
                let Some(Value::Samples(shorts)) = item.field(axis) else {
                    return Err(NO_LOG.to_owned());
                };
                data.extend(
                    shorts
                        .iter()
                        .map(|short| (time, f64::from(*short) / multiplier)),
                );
            }
        }
    }
    let freqs = fft::freq_table(
        i32::try_from(N).unwrap_or(i32::MAX),
        dotnet_int(sample_rate),
    );
    let mut panes = Vec::new();
    for (field, data) in fields.into_iter().zip(axes.iter()) {
        // "batch sampling is non continuous": windows a window apart.
        let count = data.len() / N;
        let (image, columns, _) = transform(
            data.len(),
            |index| {
                data.get(index)
                    .copied()
                    .ok_or_else(|| OUT_OF_RANGE.to_owned())
            },
            count,
            1,
            freqs.len(),
            min,
            max,
        )?;
        panes.push(Generated {
            field,
            image,
            freqs: freqs.clone(),
            columns,
        });
    }
    Ok(panes)
}

/// `GenerateImage` for a log without `ISBH`: the message's records, one image for each field.
/// `// C#: ExtLibs/Utilities/Spectrogram.cs:171-247`
fn messages(
    log: &LogFile,
    kind: &str,
    fields: [&'static str; 3],
    min: i32,
    max: i32,
) -> Result<Vec<Generated>, String> {
    let items: Vec<LogMessage> = records(log, &[kind])
        .into_iter()
        .map(|(message, _)| message)
        .collect();
    // `acc1data[0]`, which throws for a message the log has none of.
    let first = items.first().ok_or_else(|| OUT_OF_RANGE.to_owned())?;
    let timeus = if first.field("TimeUS").is_some() {
        "TimeUS"
    } else {
        "SampleUS"
    };
    let total = items.len();
    let mut count = total / N;
    // "50% overlap"
    let divisor = if count > 2048 { 1 } else { 4 };
    count *= divisor;
    let mut panes = Vec::new();
    for field in fields {
        let (image, columns, timedelta) = transform(
            total,
            |index| {
                let item = items.get(index).ok_or_else(|| OUT_OF_RANGE.to_owned())?;
                Ok((convert(item, timeus)?, convert(item, field)?))
            },
            count,
            divisor,
            N / 2,
            min,
            max,
        )?;
        // "1s / (1/ (N/delta))"
        #[allow(clippy::cast_precision_loss)] // 1024
        let sample_rate = dotnet_int(1_000_000.0 / (1.0 / (N as f64 / timedelta)));
        panes.push(Generated {
            field,
            image,
            freqs: fft::freq_table(i32::try_from(N).unwrap_or(i32::MAX), sample_rate),
            columns,
        });
    }
    Ok(panes)
}

/// `cmb_sensor_SelectedIndexChanged`'s work: the three images of the message `kind` names,
/// drawn between `min` and `max`, and where they go in the graphs.
/// `// C#: Controls/SpectrogramUI.cs:71-134; ExtLibs/Utilities/Spectrogram.cs:22-248`
pub fn generate(log: &LogFile, kind: &str, min: i32, max: i32) -> Result<Drawn, String> {
    let fields = if kind.contains("GYR") {
        GYR_FIELDS
    } else {
        ACC_FIELDS
    };
    let panes = if log.index().seen().iter().any(|name| name == "ISBH") {
        batches(log, kind, fields, min, max)?
    } else {
        messages(log, kind, fields, min, max)?
    };
    // `allfftdata` and `freqt` are the last call's, Z's.
    let last = panes.last().ok_or_else(|| NO_ELEMENTS.to_owned())?;
    let times = last.columns.iter().map(|(time, _)| *time);
    let mintime = times
        .clone()
        .reduce(f64::min)
        .ok_or_else(|| NO_ELEMENTS.to_owned())?
        / 1_000_000.0;
    let maxtime = times
        .reduce(f64::max)
        .ok_or_else(|| NO_ELEMENTS.to_owned())?
        / 1_000_000.0;
    let freqmax = last
        .freqs
        .iter()
        .copied()
        .reduce(f64::max)
        .ok_or_else(|| NO_ELEMENTS.to_owned())?;
    Ok(Drawn {
        sensor: kind.to_owned(),
        panes,
        mintime,
        maxtime,
        freqmax,
    })
}

// ---------------------------------------------------------------------------------------------
// The axes.
// ---------------------------------------------------------------------------------------------

/// The frequency axis: 0 to `freqt.Max()`, a major step of 20.
/// `// C#: Controls/SpectrogramUI.cs:101-102, 120-125`
#[must_use]
pub fn frequency_scale(freqmax: f64) -> Scale {
    let mut scale = Scale::pick(0.0, freqmax, Some((0.0, freqmax)));
    scale.major_step = Y_STEP;
    #[allow(clippy::cast_possible_truncation)] // a power of ten
    let decimals = scale.mag - Y_STEP.log10().floor() as i32;
    scale.decimals = usize::try_from(decimals.max(0)).unwrap_or(0);
    scale
}

/// The time axis: `mintime` to `maxtime`, the step ZedGraph picks.
/// `// C#: Controls/SpectrogramUI.cs:99-100`
#[must_use]
pub fn time_scale(mintime: f64, maxtime: f64) -> Scale {
    Scale::pick(mintime, maxtime, Some((mintime, maxtime)))
}

/// The three panes' places in the graph control: `SquareColPreferred` for three is two rows of
/// two, the gap between them, no margin round them.
/// `// C#: ExtLibs/ZedGraph/ZedGraph/MasterPane.cs:937-1007, 1015-1078`
#[must_use]
pub fn pane_rects() -> [(f32, f32, f32, f32); 3] {
    let (_, _, width, height) = GRAPH_AT;
    let pane_width = (width - PANE_GAP) / 2.0;
    let pane_height = (height - PANE_GAP) / 2.0;
    [
        (0.0, 0.0, pane_width, pane_height),
        (pane_width + PANE_GAP, 0.0, pane_width, pane_height),
        (0.0, pane_height + PANE_GAP, pane_width, pane_height),
    ]
}

/// The chart's place in its pane: room for the title above, the frequency labels at the left,
/// the time labels and title below.
const CHART_INSET: (f32, f32, f32, f32) = (44.0, 22.0, 12.0, 30.0);

/// The gap a frequency label needs from the one below it.
const LABEL_SPACING: f32 = 11.0;

// ---------------------------------------------------------------------------------------------
// The form.
// ---------------------------------------------------------------------------------------------

/// A log read by Load Log: `file`.
#[derive(Debug, Clone)]
pub struct Loaded {
    /// Its file's name.
    pub name: String,
    /// The log.
    pub log: Arc<LogFile>,
}

/// What the thread did.
#[derive(Debug)]
pub enum Done {
    /// Load Log's `new DFLogBuffer(File.OpenRead(...))`.
    Loaded(Loaded),
    /// A draw's three images.
    Drawn(Drawn),
}

/// What the thread is doing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Task {
    /// Reading a log.
    Load,
    /// Drawing.
    Draw,
}

impl Task {
    /// What a fact calls it.
    #[must_use]
    pub const fn key(self) -> &'static str {
        match self {
            Self::Load => "load",
            Self::Draw => "draw",
        }
    }
}

/// Which box has the keyboard.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Edit {
    /// `cmb_sensor`'s text.
    Sensor,
    /// `num_min`.
    Min,
    /// `num_max`.
    Max,
}

impl Edit {
    /// The control id.
    #[must_use]
    pub const fn id(self) -> &'static str {
        match self {
            Self::Sensor => "spect-cmb_sensor",
            Self::Min => "spect-num_min",
            Self::Max => "spect-num_max",
        }
    }
}

/// The ids of Load Log's dialog; the question and message ids are unused.
const PATH_IDS: BoxIds = BoxIds {
    question: "spect-question",
    yes: "spect-question-yes",
    no: "spect-question-no",
    message: "spect-message",
    ok: "spect-message-ok",
    path: "spect-open",
    path_value: "spect-open-value",
    path_ok: "spect-open-ok",
    path_cancel: "spect-open-cancel",
};

/// The textures of what was drawn, made once a draw lands so each keeps its id; and those a
/// later draw replaced, for the next paint to take out of the GPU's atlas, which keeps an image
/// until told to drop it.
#[derive(Clone, Default)]
struct Textures {
    /// X's, Y's and Z's.
    shown: Vec<Arc<RenderImage>>,
    /// Replaced, not yet dropped.
    retired: Rc<RefCell<Vec<Arc<RenderImage>>>>,
}

impl std::fmt::Debug for Textures {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Textures")
            .field("shown", &self.shown.len())
            .field("retired", &self.retired.borrow().len())
            .finish()
    }
}

impl Textures {
    /// Every texture, shown and retired, taken: for the form's close box to drop.
    fn take_all(&mut self) -> Vec<Arc<RenderImage>> {
        let mut all = std::mem::take(&mut self.shown);
        all.append(&mut self.retired.borrow_mut());
        all
    }
}

/// The `SpectrogramUI` form.
#[derive(Debug)]
pub struct Spectrogram {
    /// `file`: the log Load Log read.
    log: Option<Loaded>,
    /// `cmb_sensor.Text`.
    pub sensor: TextField,
    /// `cmb_sensor.Items` and `SelectedIndex`, and the dropped list's place.
    pub list: Combo,
    /// Whether the list is dropped down.
    pub list_open: bool,
    /// `num_min`.
    pub min: NumericUpDown,
    /// `num_max`.
    pub max: NumericUpDown,
    /// The values `ValueChanged` was last raised at, Min's and Max's.
    values: (f64, f64),
    /// The box being typed into.
    pub editing: Option<Edit>,
    /// Load Log's dialog, while it is up.
    asking: Option<PathBox>,
    /// What the thread is doing.
    running: Option<(Task, Receiver<Result<Done, String>>)>,
    /// How the last thing done ended, for the facts.
    pub last: Option<String>,
    /// What the graphs show.
    drawn: Option<Drawn>,
    /// Its textures.
    textures: Textures,
    /// What the C# would have thrown, for the status line.
    error: Option<String>,
}

impl Default for Spectrogram {
    fn default() -> Self {
        Self::new()
    }
}

impl Spectrogram {
    /// `new SpectrogramUI()`: the Designer's controls, the three panes empty, no log.
    /// `// C#: Controls/SpectrogramUI.cs:25-48; SpectrogramUI.Designer.cs:30-188`
    #[must_use]
    pub fn new() -> Self {
        let mut sensor = TextField::new("");
        sensor.set(SENSOR_TEXT);
        let options: Vec<(i64, String)> = (0_i64..)
            .zip(SENSORS)
            .map(|(index, name)| (index, name.to_owned()))
            .collect();
        // `Text = "ACC1"` finds the item and selects it, before the handler is wired.
        let list = Combo {
            param: String::new(),
            options,
            selected: Some(0),
            enabled: true,
            top_index: 0,
        };
        let (low, high) = NUMBER_BOUNDS;
        Self {
            log: None,
            sensor,
            list,
            list_open: false,
            min: NumericUpDown::new((MIN_DEFAULT, low, high)),
            max: NumericUpDown::new((MAX_DEFAULT, low, high)),
            values: (MIN_DEFAULT, MAX_DEFAULT),
            editing: None,
            asking: None,
            running: None,
            last: None,
            drawn: None,
            textures: Textures::default(),
            error: None,
        }
    }

    /// The log's file name, if one is loaded.
    #[must_use]
    pub fn log_name(&self) -> Option<&str> {
        self.log.as_ref().map(|loaded| loaded.name.as_str())
    }

    /// What the graphs show.
    #[must_use]
    pub const fn drawn(&self) -> Option<&Drawn> {
        self.drawn.as_ref()
    }

    /// Whether the thread is reading or drawing: the controls are dimmed meanwhile.
    #[must_use]
    pub fn running(&self) -> Option<Task> {
        self.running.as_ref().map(|(task, _)| *task)
    }

    /// The dialog's text, while it is up.
    #[must_use]
    pub fn prompt(&self) -> Option<&str> {
        self.asking.as_ref().map(|path| path.field.value())
    }

    /// What the C# would have thrown since last asked, as a test asks.
    #[cfg(test)]
    pub fn take_error(&mut self) -> Option<String> {
        self.error.take()
    }

    /// `but_loadlog_Click`: the `OpenFileDialog`.
    /// `// C#: Controls/SpectrogramUI.cs:50-56`
    pub fn press_load(&mut self) {
        if self.running.is_some() || self.asking.is_some() {
            return;
        }
        self.leave();
        self.list_open = false;
        self.asking = Some(PathBox::new("", LOG_FILTER));
    }

    /// A key for the dialog: Enter is OK, Escape Cancel.
    pub fn prompt_key(&mut self, event: &KeyDownEvent) -> bool {
        let Some(path) = self.asking.as_mut() else {
            return false;
        };
        match path.field.key(event) {
            KeyOutcome::Changed => true,
            KeyOutcome::Submitted => {
                self.prompt_done(true);
                true
            }
            KeyOutcome::Cancelled => {
                self.prompt_done(false);
                true
            }
            KeyOutcome::Ignored => false,
        }
    }

    /// Types into the dialog, as a test does.
    #[cfg(test)]
    pub fn type_prompt(&mut self, text: &str) {
        if let Some(path) = self.asking.as_mut() {
            path.field.set(text);
        }
    }

    /// The dialog closed: on OK a file that exists is read - `file = new DFLogBuffer(...)` -
    /// and nothing drawn; anything else does nothing, as `if (!File.Exists(ofd.FileName))
    /// return;` does.
    /// `// C#: Controls/SpectrogramUI.cs:57-63`
    pub fn prompt_done(&mut self, ok: bool) {
        let Some(path) = self.asking.take() else {
            return;
        };
        let Some(file) = path.chosen().filter(|_| ok) else {
            return;
        };
        self.start(Task::Load, move || load(&file).map(Done::Loaded));
    }

    /// The list's arrow: dropped down, or up again.
    pub fn toggle_list(&mut self) {
        if self.running.is_some() {
            return;
        }
        self.leave();
        self.list_open = !self.list_open;
        if self.list_open {
            self.list.open_list();
        }
    }

    /// A row of the list chosen: the text its item's, and - when that moved the selection -
    /// `cmb_sensor_SelectedIndexChanged`, a draw.
    /// `// C#: Controls/SpectrogramUI.Designer.cs:74; SpectrogramUI.cs:71-134`
    pub fn choose(&mut self, index: i64) {
        self.list_open = false;
        let Some(text) = self
            .list
            .options
            .iter()
            .find(|(key, _)| *key == index)
            .map(|(_, text)| text.clone())
        else {
            return;
        };
        self.sensor.set(text);
        if self.list.select(index) {
            self.draw();
        }
    }

    /// A key in the box being typed into. Typing into the combo's text leaves no item selected,
    /// as a Win32 combo box's edit does with its list up, and raises nothing; Enter, the arrow
    /// keys and leaving a number box raise `ValueChanged` when its value moved.
    pub fn key(&mut self, event: &KeyDownEvent) -> bool {
        match self.editing {
            Some(Edit::Sensor) => match self.sensor.key(event) {
                KeyOutcome::Changed => {
                    self.list.selected = None;
                    true
                }
                KeyOutcome::Submitted | KeyOutcome::Cancelled | KeyOutcome::Ignored => false,
            },
            Some(which @ (Edit::Min | Edit::Max)) => {
                let commits = matches!(event.keystroke.key.as_str(), "up" | "down" | "enter");
                let handled = self.number_mut(which).key(event);
                if commits {
                    self.value_changed();
                }
                handled
            }
            None => false,
        }
    }

    /// Types into the combo's text, as a test does.
    #[cfg(test)]
    pub fn type_sensor(&mut self, text: &str) {
        self.sensor.set(text);
        self.list.selected = None;
    }

    /// A box clicked into.
    pub fn begin(&mut self, which: Edit) {
        if self.editing != Some(which) {
            self.leave();
        }
        self.list_open = false;
        self.editing = Some(which);
    }

    /// The keyboard left the box: a number's text validated, and `ValueChanged` raised when it
    /// moved.
    pub fn leave(&mut self) {
        match self.editing.take() {
            Some(which @ (Edit::Min | Edit::Max)) => {
                self.number_mut(which).commit();
                self.value_changed();
            }
            Some(Edit::Sensor) | None => {}
        }
    }

    /// A number box.
    pub fn number_mut(&mut self, which: Edit) -> &mut NumericUpDown {
        match which {
            Edit::Max => &mut self.max,
            Edit::Min | Edit::Sensor => &mut self.min,
        }
    }

    /// An arrow of a number box: `UpButton` or `DownButton`, then `ValueChanged` if it moved.
    pub fn step(&mut self, which: Edit, delta: f64) {
        if self.running.is_some() {
            return;
        }
        self.number_mut(which).step(delta);
        self.value_changed();
    }

    /// `num_min_ValueChanged` and `num_max_ValueChanged`, each a draw, for the box whose
    /// value moved since it was last raised.
    /// `// C#: Controls/SpectrogramUI.cs:136-144`
    fn value_changed(&mut self) {
        let now = (self.min.value(), self.max.value());
        #[allow(clippy::float_cmp)] // a box's value, as `Value` compares it
        let moved = [now.0 != self.values.0, now.1 != self.values.1];
        self.values = now;
        for _ in moved.into_iter().filter(|moved| *moved) {
            self.draw();
        }
    }

    /// `but_redraw_Click`, and each handler's call of `cmb_sensor_SelectedIndexChanged(null,
    /// null)`: the three images of the message the box names, made from `file` - which throws
    /// with no log loaded - between `(int)num_min.Value` and `(int)num_max.Value`.
    /// `// C#: Controls/SpectrogramUI.cs:71-134, 146-149`
    pub fn draw(&mut self) {
        // `Value` validates text being typed, raising `ValueChanged` on its way: the draw being
        // started now is the one it would have started.
        self.min.commit();
        self.max.commit();
        self.values = (self.min.value(), self.max.value());
        let Some(loaded) = self.log.clone() else {
            self.last = Some(NO_LOG.to_owned());
            self.error = Some(NO_LOG.to_owned());
            return;
        };
        if self.running.is_some() {
            return;
        }
        let kind = self.sensor.value().to_owned();
        #[allow(clippy::cast_possible_truncation)] // `(int)` of a box of -1000..1000
        let (min, max) = (self.min.value() as i32, self.max.value() as i32);
        self.start(Task::Draw, move || {
            generate(&loaded.log, &kind, min, max).map(Done::Drawn)
        });
    }

    /// Runs `work` on a thread of its own.
    fn start(&mut self, task: Task, work: impl FnOnce() -> Result<Done, String> + Send + 'static) {
        let (send, receive) = std::sync::mpsc::channel();
        let spawned = std::thread::Builder::new()
            .name("mp-spectrogram".to_owned())
            .spawn(move || {
                // The receiver may be gone with the form; nothing is owed then.
                let _ = send.send(work());
            });
        if spawned.is_ok() {
            self.running = Some((task, receive));
        } else {
            self.last = Some("no thread".to_owned());
        }
    }

    /// Once a frame: what the thread did. The error of what threw is returned, for the status
    /// line, with any the form met since.
    pub fn poll(&mut self) -> Option<String> {
        let mut error = self.error.take();
        if let Some((_, receiver)) = self.running.as_ref() {
            let result = match receiver.try_recv() {
                Ok(result) => Some(result),
                Err(TryRecvError::Empty) => None,
                Err(TryRecvError::Disconnected) => Some(Err(
                    "the spectrogram thread ended without a result".to_owned(),
                )),
            };
            if let Some(result) = result {
                self.running = None;
                match result {
                    Ok(Done::Loaded(loaded)) => {
                        self.log = Some(loaded);
                        self.last = Some("loaded".to_owned());
                    }
                    Ok(Done::Drawn(drawn)) => {
                        let shown = drawn
                            .panes
                            .iter()
                            .map(|pane| {
                                Arc::new(RenderImage::new(vec![image::Frame::new(
                                    pane.image.texture(),
                                )]))
                            })
                            .collect();
                        let replaced = std::mem::replace(&mut self.textures.shown, shown);
                        self.textures.retired.borrow_mut().extend(replaced);
                        self.drawn = Some(drawn);
                        self.last = Some("done".to_owned());
                    }
                    Err(failed) => {
                        self.last = Some(failed.clone());
                        error = Some(failed);
                    }
                }
            }
        }
        error
    }

    /// Waits for the thread, as a test does.
    #[cfg(test)]
    pub fn finish(&mut self) -> Option<String> {
        let mut error = None;
        for _ in 0..3000 {
            if let Some(failed) = self.poll() {
                error = Some(failed);
            }
            if self.running.is_none() {
                break;
            }
            std::thread::sleep(std::time::Duration::from_millis(5));
        }
        error
    }
}

/// `new DFLogBuffer(File.OpenRead(ofd.FileName))`.
fn load(path: &Path) -> Result<Loaded, String> {
    let log = LogFile::open(path).map_err(|error| format!("{}: {error}", path.display()))?;
    Ok(Loaded {
        name: path
            .file_name()
            .map(|name| name.to_string_lossy().into_owned())
            .unwrap_or_default(),
        log: Arc::new(log),
    })
}

/// The window and how often it has been opened, held with the Advanced page.
#[derive(Debug, Default)]
pub struct SpectrogramWindow {
    /// The form, while it is open.
    pub window: Option<Spectrogram>,
    /// How many times it has been opened.
    pub opened: usize,
}

impl SpectrogramWindow {
    /// `new SpectrogramUI().Show()`: a fresh form.
    /// `// C#: GCSViews/ConfigurationView/ConfigAdvanced.cs:119-122`
    pub fn show(&mut self) {
        self.opened += 1;
        self.window = Some(Spectrogram::new());
    }

    /// The form's close box: its textures handed back, for the GPU's atlas to drop.
    pub fn close(&mut self) -> Vec<Arc<RenderImage>> {
        self.window
            .take()
            .map(|mut form| form.textures.take_all())
            .unwrap_or_default()
    }

    /// Once a frame: a box the keyboard has left read, and what the thread did - an error
    /// returned for the status line.
    pub fn tick(&mut self, number_focused: bool, text_focused: bool) -> Option<String> {
        let form = self.window.as_mut()?;
        match form.editing {
            Some(Edit::Min | Edit::Max) if !number_focused => form.leave(),
            Some(Edit::Sensor) if !text_focused => form.leave(),
            _ => {}
        }
        form.poll()
    }
}

/// The keyboard focus of the form's boxes.
pub struct FocusHandles {
    /// Min or Max being typed into.
    pub number: FocusHandle,
    /// The combo's text being typed into.
    pub text: FocusHandle,
    /// Load Log's dialog.
    pub prompt: FocusHandle,
}

impl FocusHandles {
    /// New handles.
    pub fn new(cx: &mut Context<MissionPlanner>) -> Self {
        Self {
            number: cx.focus_handle(),
            text: cx.focus_handle(),
            prompt: cx.focus_handle(),
        }
    }
}

/// Facts a UI test asserts on.
pub fn record_facts(holder: &SpectrogramWindow) {
    use crate::facts::record;
    record("config.spectrogram.window", holder.window.is_some());
    record("config.spectrogram.opened", holder.opened);
    let Some(form) = holder.window.as_ref() else {
        return;
    };
    record("config.spectrogram.log", form.log_name().unwrap_or("none"));
    record("config.spectrogram.sensor", form.sensor.value());
    record(
        "config.spectrogram.sensor.selected",
        form.list.selected.unwrap_or(-1),
    );
    record("config.spectrogram.sensor.list", form.list_open);
    record("config.spectrogram.min", form.min.field.value());
    record("config.spectrogram.max", form.max.field.value());
    record(
        "config.spectrogram.prompt",
        if form.asking.is_some() {
            LOAD_LOG
        } else {
            "none"
        },
    );
    record(
        "config.spectrogram.prompt.text",
        form.prompt().unwrap_or(""),
    );
    record(
        "config.spectrogram.running",
        form.running().map_or("none", Task::key),
    );
    record(
        "config.spectrogram.last",
        form.last.as_deref().unwrap_or("none"),
    );
    record("config.spectrogram.drawn", form.drawn.is_some());
    for (index, title) in PANE_TITLES.iter().enumerate() {
        record(format!("config.spectrogram.pane.{index}.title"), title);
    }
    let Some(drawn) = form.drawn() else {
        return;
    };
    record("config.spectrogram.drawn.sensor", &drawn.sensor);
    record("config.spectrogram.freqmax", drawn.freqmax);
    record("config.spectrogram.mintime", drawn.mintime);
    record("config.spectrogram.maxtime", drawn.maxtime);
    if let Some(pane) = drawn.panes.first() {
        record("config.spectrogram.columns", pane.image.width);
        record("config.spectrogram.rows", pane.image.height);
        record("config.spectrogram.windows", pane.columns.len());
    }
    for (index, pane) in drawn.panes.iter().enumerate() {
        record(format!("config.spectrogram.pane.{index}.field"), pane.field);
        record(
            format!("config.spectrogram.pane.{index}.hot"),
            pane.image.count(HOT),
        );
        record(
            format!("config.spectrogram.pane.{index}.cold"),
            pane.image.count(COLD),
        );
    }
}

// ---------------------------------------------------------------------------------------------
// Drawing.
// ---------------------------------------------------------------------------------------------

/// How the drawing reaches the form.
fn access(this: &mut MissionPlanner) -> Option<&mut Spectrogram> {
    this.extra.spectrogram.window.as_mut()
}

/// A `NumericUpDown` at its place: its text, typed into while it has the focus, and its arrows.
/// Shared with the Support Proxy's port.
#[allow(clippy::too_many_arguments)]
pub fn updown(
    id: &'static str,
    number: &NumericUpDown,
    (x, y, width, height): (f32, f32, f32, f32),
    editing: bool,
    handle: &FocusHandle,
    enabled: bool,
    on_begin: impl Fn(&mut MissionPlanner) + 'static,
    on_key: impl Fn(&mut MissionPlanner, &KeyDownEvent) -> bool + 'static,
    on_step: impl Fn(&mut MissionPlanner, f64) + Clone + 'static,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let focused = editing && handle.is_focused(window);
    let text = crate::probe::measured(id, div())
        .id(id)
        .flex_1()
        .h_full()
        .flex()
        .items_center()
        .px_1()
        .overflow_hidden()
        .text_xs()
        .text_color(rgb(if enabled { theme::TEXT } else { theme::DIM }))
        .child(number.field.value().to_owned())
        .children(focused.then(|| div().w(px(1.0)).h(px(12.0)).bg(rgb(theme::ACCENT))));
    let text = if !enabled {
        text
    } else if editing {
        text.track_focus(handle)
            .key_context("TextField")
            .cursor_text()
            .on_key_down(cx.listener(move |this, event: &KeyDownEvent, _window, cx| {
                if on_key(this, event) {
                    cx.notify();
                }
            }))
    } else {
        let handle = handle.clone();
        text.cursor_text()
            .on_click(cx.listener(move |this, _event, window, cx| {
                on_begin(this);
                handle.focus(window, cx);
                cx.notify();
            }))
    };
    let mut arrows = div()
        .w(px(14.0))
        .h_full()
        .flex()
        .flex_col()
        .border_l_1()
        .border_color(rgb(theme::BORDER));
    for (suffix, glyph, delta) in [("up", "▲", 1.0), ("down", "▼", -1.0)] {
        let arrow_id = format!("{id}-{suffix}");
        let base = crate::probe::measured(arrow_id.clone(), div())
            .id(SharedString::from(arrow_id))
            .flex_1()
            .flex()
            .items_center()
            .justify_center()
            .text_size(px(7.0))
            .child(glyph);
        let on_step = on_step.clone();
        arrows = arrows.child(if enabled {
            base.text_color(rgb(theme::TEXT))
                .cursor_pointer()
                .hover(|style| style.bg(rgb(theme::BORDER)))
                .on_click(cx.listener(move |this, _event, _window, cx| {
                    on_step(this, delta);
                    cx.notify();
                }))
                .into_any_element()
        } else {
            base.text_color(rgb(theme::DIM)).into_any_element()
        });
    }
    at(x, y, width, height)
        .flex()
        .rounded_sm()
        .border_1()
        .border_color(rgb(if focused {
            theme::ACCENT
        } else {
            theme::BORDER
        }))
        .bg(rgb(if enabled { theme::ACTION } else { theme::PANEL }))
        .child(text)
        .child(arrows)
        .into_any_element()
}

/// The combo box: its text, typed into while it has the focus, and the arrow that drops its
/// list down.
fn sensor_box(
    form: &Spectrogram,
    handle: &FocusHandle,
    enabled: bool,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let (x, y, width, height) = SENSOR_AT;
    let editing = form.editing == Some(Edit::Sensor);
    let focused = editing && handle.is_focused(window);
    let id = Edit::Sensor.id();
    let text = crate::probe::measured(id, div())
        .id(id)
        .flex_1()
        .h_full()
        .flex()
        .items_center()
        .px_1()
        .overflow_hidden()
        .whitespace_nowrap()
        .text_xs()
        .text_color(rgb(if enabled { theme::TEXT } else { theme::DIM }))
        .child(form.sensor.value().to_owned())
        .children(focused.then(|| div().w(px(1.0)).h(px(12.0)).bg(rgb(theme::ACCENT))));
    let text = if !enabled {
        text
    } else if editing {
        text.track_focus(handle)
            .key_context("TextField")
            .cursor_text()
            .on_key_down(cx.listener(|this, event: &KeyDownEvent, _window, cx| {
                if access(this).is_some_and(|form| form.key(event)) {
                    cx.notify();
                }
            }))
    } else {
        let handle = handle.clone();
        text.cursor_text()
            .on_click(cx.listener(move |this, _event, window, cx| {
                if let Some(form) = access(this) {
                    form.begin(Edit::Sensor);
                }
                handle.focus(window, cx);
                cx.notify();
            }))
    };
    let arrow_id = "spect-cmb_sensor-arrow";
    let arrow = crate::probe::measured(arrow_id, div())
        .id(arrow_id)
        .w(px(14.0))
        .h_full()
        .flex()
        .items_center()
        .justify_center()
        .border_l_1()
        .border_color(rgb(theme::BORDER))
        .text_size(px(7.0))
        .child("▼");
    let arrow = if enabled {
        arrow
            .text_color(rgb(theme::TEXT))
            .cursor_pointer()
            .hover(|style| style.bg(rgb(theme::BORDER)))
            .on_click(cx.listener(|this, _event, window, cx| {
                window.blur(cx);
                if let Some(form) = access(this) {
                    form.toggle_list();
                }
                cx.notify();
            }))
    } else {
        arrow.text_color(rgb(theme::DIM))
    };
    at(x, y, width, height)
        .flex()
        .rounded_sm()
        .border_1()
        .border_color(rgb(if focused {
            theme::ACCENT
        } else {
            theme::BORDER
        }))
        .bg(rgb(if enabled { theme::ACTION } else { theme::PANEL }))
        .child(text)
        .child(arrow)
        .into_any_element()
}

/// One pane: its title, the chart - the image stretched over it, the frequency grid over that -
/// the labels of both axes, and their titles.
/// `// C#: Controls/SpectrogramUI.cs:96-125`
fn pane(
    index: usize,
    (x, y, width, height): (f32, f32, f32, f32),
    drawn: Option<(&Drawn, Arc<RenderImage>)>,
) -> AnyElement {
    let (left, top, right, bottom) = CHART_INSET;
    let (chart_width, chart_height) = (width - left - right, height - top - bottom);
    let mut chart = crate::probe::measured(format!("spect-pane-{index}-chart"), div())
        .absolute()
        .left(px(left))
        .top(px(top))
        .w(px(chart_width))
        .h(px(chart_height))
        .border_1()
        .border_color(rgb(theme::BORDER))
        .bg(rgb(theme::BG));
    let mut labels = div().absolute().left_0().top_0().size_full();
    if let Some((drawn, texture)) = drawn {
        // `ImageObj(img, mintime, freqt.Max(), tdelta, freqt.Max())`, `ZOrder.F_BehindGrid`:
        // the image fills the chart, whose axes run exactly over it.
        chart = chart.child(
            canvas(
                |_bounds, _window, _cx| {},
                move |bounds, (), window, _cx| {
                    let _ =
                        window.paint_image(bounds, bounds, Corners::default(), texture, 0, false);
                },
            )
            .absolute()
            .size_full(),
        );
        let frequency = frequency_scale(drawn.freqmax);
        let mut last_label: Option<f32> = None;
        for tic in frequency.tics() {
            #[allow(clippy::cast_possible_truncation)] // a fraction of a chart's height
            let at_y = chart_height * (1.0 - frequency.fraction(tic) as f32);
            // `MajorGrid.IsVisible`: over the image.
            chart = chart.child(
                div()
                    .absolute()
                    .left_0()
                    .top(px(at_y))
                    .w_full()
                    .h(px(1.0))
                    .bg(rgb(theme::BORDER)),
            );
            if last_label.is_some_and(|below| below - at_y < LABEL_SPACING) {
                continue;
            }
            last_label = Some(at_y);
            labels = labels.child(
                div()
                    .absolute()
                    .left(px(0.0))
                    .top(px(top + at_y - 6.0))
                    .w(px(left - 3.0))
                    .flex()
                    .justify_end()
                    .text_size(px(8.0))
                    .text_color(rgb(theme::DIM))
                    .child(frequency.label(tic)),
            );
        }
        let time = time_scale(drawn.mintime, drawn.maxtime);
        for tic in time.tics() {
            #[allow(clippy::cast_possible_truncation)] // a fraction of a chart's width
            let at_x = left + chart_width * time.fraction(tic) as f32;
            labels = labels.child(
                div()
                    .absolute()
                    .left(px(at_x - 20.0))
                    .top(px(top + chart_height + 1.0))
                    .w(px(40.0))
                    .flex()
                    .justify_center()
                    .text_size(px(8.0))
                    .text_color(rgb(theme::DIM))
                    .child(time.label(tic)),
            );
        }
    }
    crate::probe::measured(format!("spect-pane-{index}"), at(x, y, width, height))
        .child(
            div()
                .absolute()
                .left_0()
                .top(px(2.0))
                .w_full()
                .flex()
                .justify_center()
                .text_xs()
                .text_color(rgb(theme::TEXT))
                .child(PANE_TITLES.get(index).copied().unwrap_or_default()),
        )
        .child(
            div()
                .absolute()
                .left(px(2.0))
                .top(px(top - 11.0))
                .text_size(px(8.0))
                .text_color(rgb(theme::DIM))
                .child(Y_TITLE),
        )
        .child(chart)
        .child(labels)
        .child(
            div()
                .absolute()
                .left(px(left))
                .top(px(height - bottom + 13.0))
                .w(px(chart_width))
                .flex()
                .justify_center()
                .text_size(px(9.0))
                .text_color(rgb(theme::DIM))
                .child(X_TITLE),
        )
        .into_any_element()
}

/// The form over the window, its caption with a close box, and Load Log's dialog over it while
/// it is up.
/// `// C#: Controls/SpectrogramUI.Designer.cs:30-188`
pub fn overlay(
    holder: &SpectrogramWindow,
    focus: &FocusHandles,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    let form = holder.window.as_ref()?;
    let size = window.viewport_size();
    let idle = form.running.is_none();
    let (gx, gy, gw, gh) = GRAPH_AT;
    let mut graph = crate::probe::measured("spect-graph", at(gx, gy, gw, gh))
        .border_1()
        .border_color(rgb(theme::BORDER));
    for (index, rect) in pane_rects().into_iter().enumerate() {
        let drawn = form
            .drawn
            .as_ref()
            .zip(form.textures.shown.get(index).cloned());
        graph = graph.child(pane(index, rect, drawn));
    }
    // The textures a draw replaced, out of the atlas.
    let retired = Rc::clone(&form.textures.retired);
    graph = graph.child(
        canvas(
            |_bounds, _window, _cx| {},
            move |_bounds, (), window, cx| {
                for image in retired.borrow_mut().drain(..) {
                    cx.drop_image(image, Some(window));
                }
            },
        )
        .absolute()
        .size_0(),
    );
    let prompt = focus.prompt.clone();
    let mut client = div()
        .relative()
        .w(px(CLIENT.0))
        .h(px(CLIENT.1))
        .child(graph)
        .child(button(
            "spect-but_loadlog",
            LOAD_LOG,
            LOAD_AT,
            idle,
            move |this, window, cx| {
                if let Some(form) = access(this) {
                    form.press_load();
                    prompt.focus(window, cx);
                }
            },
            cx,
        ))
        .child(sensor_box(form, &focus.text, idle, window, cx))
        .child(label(MIN_LABEL_AT.0, MIN_LABEL_AT.1, MIN_TEXT, true))
        .child(label(MAX_LABEL_AT.0, MAX_LABEL_AT.1, MAX_TEXT, true));
    for (which, place) in [(Edit::Min, MIN_AT), (Edit::Max, MAX_AT)] {
        let number = match which {
            Edit::Max => &form.max,
            Edit::Min | Edit::Sensor => &form.min,
        };
        client = client.child(updown(
            which.id(),
            number,
            place,
            form.editing == Some(which),
            &focus.number,
            idle,
            move |this| {
                if let Some(form) = access(this) {
                    form.begin(which);
                }
            },
            |this, event| access(this).is_some_and(|form| form.key(event)),
            move |this, delta| {
                if let Some(form) = access(this) {
                    form.step(which, delta);
                }
            },
            window,
            cx,
        ));
    }
    client = client.child(button(
        "spect-but_redraw",
        UPDATE,
        UPDATE_AT,
        idle,
        |this, window, cx| {
            window.blur(cx);
            if let Some(form) = access(this) {
                form.leave();
                form.draw();
            }
        },
        cx,
    ));
    if form.list_open && idle {
        let (x, y, width, height) = SENSOR_AT;
        client = client.child(dropdown(
            Edit::Sensor.id(),
            &form.list,
            (x, y + height, width),
            |this, index| {
                if let Some(form) = access(this) {
                    form.choose(index);
                }
            },
            |this, lines| {
                if let Some(form) = access(this) {
                    form.list.scroll_list(lines);
                }
            },
            cx,
        ));
    }
    let caption = div()
        .flex()
        .items_center()
        .justify_between()
        .px_2()
        .py_1()
        .border_b_1()
        .border_color(rgb(theme::BORDER))
        .child(div().text_xs().text_color(rgb(theme::DIM)).child(FORM_TEXT))
        .child(crate::ui::action(
            "spect-close",
            "X",
            theme::TEXT,
            true,
            cx.listener(|this, _event: &(), window, cx| {
                for image in this.extra.spectrogram.close() {
                    cx.drop_image(image, Some(window));
                }
                cx.notify();
            }),
        ));
    let body = crate::probe::measured("spectrogram", div())
        .flex()
        .flex_col()
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::ACCENT))
        .rounded_md()
        .child(caption)
        .child(client);
    let over = div()
        .id("spectrogram-backdrop")
        .w(size.width)
        .h(size.height)
        .flex()
        .items_center()
        .justify_center()
        .occlude()
        .child(body);
    let mut layers = div().child(
        gpui::deferred(
            gpui::anchored()
                .position(gpui::point(px(0.0), px(0.0)))
                .child(over),
        )
        .with_priority(1),
    );
    if let Some(path) = form.asking.as_ref() {
        layers = layers.child(path_box(
            PATH_IDS,
            path,
            &focus.prompt,
            window,
            |this, event| access(this).is_some_and(|form| form.prompt_key(event)),
            |this, ok| {
                if let Some(form) = access(this) {
                    form.prompt_done(ok);
                }
            },
            cx,
        ));
    }
    Some(layers.into_any_element())
}

/// The file a path names, for a test.
#[cfg(test)]
fn fixture(name: &str) -> PathBuf {
    std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../testdata")
        .join(name)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A dataflash record: the head, the type, the payload.
    fn record(msg_type: u8, payload: &[u8]) -> Vec<u8> {
        let mut out = vec![0xA3, 0x95, msg_type];
        out.extend_from_slice(payload);
        out
    }

    /// Text zero-padded to a width.
    fn fixed(text: &str, width: usize) -> Vec<u8> {
        let mut out = text.as_bytes().to_vec();
        out.resize(width, 0);
        out
    }

    /// An `FMT` record.
    fn fmt(msg_type: u8, length: u8, name: &str, format: &str, columns: &str) -> Vec<u8> {
        let mut payload = vec![msg_type, length];
        payload.extend(fixed(name, 4));
        payload.extend(fixed(format, 16));
        payload.extend(fixed(columns, 64));
        record(0x80, &payload)
    }

    /// A sine of `amplitude` at `hz`, sampled at `rate`, at sample `i`.
    fn sine(amplitude: f64, hz: f64, rate: f64, i: usize) -> f64 {
        #[allow(clippy::cast_precision_loss)]
        let t = i as f64 / rate;
        amplitude * (2.0 * std::f64::consts::PI * hz * t).sin()
    }

    /// `ACC1` and `GYR1` at 1 kHz, `samples` of each: a 100 Hz sine of 2 on AccZ, 250 Hz of 1
    /// on GyrY, nothing on the rest.
    fn acc_gyr_log(samples: usize) -> LogFile {
        let mut bytes = Vec::new();
        bytes.extend(fmt(
            200,
            3 + 8 + 12,
            "ACC1",
            "Qfff",
            "TimeUS,AccX,AccY,AccZ",
        ));
        bytes.extend(fmt(
            201,
            3 + 8 + 12,
            "GYR1",
            "Qfff",
            "TimeUS,GyrX,GyrY,GyrZ",
        ));
        for i in 0..samples {
            let time = 1_000_000_u64 + 1000 * i as u64;
            #[allow(clippy::cast_possible_truncation)]
            let accz = sine(2.0, 100.0, 1000.0, i) as f32;
            #[allow(clippy::cast_possible_truncation)]
            let gyry = sine(1.0, 250.0, 1000.0, i) as f32;
            let mut acc = time.to_le_bytes().to_vec();
            for value in [0.0_f32, 0.0, accz] {
                acc.extend(value.to_le_bytes());
            }
            bytes.extend(record(200, &acc));
            let mut gyr = time.to_le_bytes().to_vec();
            for value in [0.0_f32, gyry, 0.0] {
                gyr.extend(value.to_le_bytes());
            }
            bytes.extend(record(201, &gyr));
        }
        LogFile::from_bytes(bytes)
    }

    /// `ISBH`/`ISBD` batches at 2 kHz, multiplier 100: sensor `(type, instance)` of each header,
    /// 8 x 32 samples a batch - x a 250 Hz sine of 3, y a 500 Hz sine of 1, z nothing.
    fn batch_log(batches: usize, sensors: &[(u8, u8)]) -> LogFile {
        let mut bytes = Vec::new();
        bytes.extend(fmt(
            210,
            3 + 28,
            "ISBH",
            "QHBBHHQf",
            "TimeUS,N,type,instance,mul,smp_cnt,SampleUS,smp_rate",
        ));
        bytes.extend(fmt(
            211,
            3 + 12 + 192,
            "ISBD",
            "QHHaaa",
            "TimeUS,N,seqno,x,y,z",
        ));
        let mut index = 0_usize;
        let mut number = 0_u16;
        for batch in 0..batches {
            for (kind, instance) in sensors {
                let time = 1_000_000_u64 + 100_000 * batch as u64;
                let mut header = time.to_le_bytes().to_vec();
                header.extend(number.to_le_bytes());
                header.extend([*kind, *instance]);
                header.extend(100_u16.to_le_bytes());
                header.extend(256_u16.to_le_bytes());
                header.extend(time.to_le_bytes());
                header.extend(2000.0_f32.to_le_bytes());
                bytes.extend(record(210, &header));
                for seq in 0..8_u16 {
                    let mut data = (time + u64::from(seq) * 10_000).to_le_bytes().to_vec();
                    data.extend(number.to_le_bytes());
                    data.extend(seq.to_le_bytes());
                    let (mut x, mut y) = (Vec::new(), Vec::new());
                    for k in 0..32 {
                        #[allow(clippy::cast_possible_truncation)]
                        let xs = (sine(3.0, 250.0, 2000.0, index + k) * 100.0).round() as i16;
                        #[allow(clippy::cast_possible_truncation)]
                        let ys = (sine(1.0, 500.0, 2000.0, index + k) * 100.0).round() as i16;
                        x.extend(xs.to_le_bytes());
                        y.extend(ys.to_le_bytes());
                    }
                    data.extend(x);
                    data.extend(y);
                    data.extend(vec![0_u8; 64]);
                    bytes.extend(record(211, &data));
                    if (*kind, *instance) == (0, 0) {
                        index += 32;
                    }
                }
                number = number.wrapping_add(1);
            }
        }
        LogFile::from_bytes(bytes)
    }

    /// The row a bin is drawn in.
    fn row_of(bin: usize) -> usize {
        N / 2 - 1 - bin
    }

    /// The Designer's words and places, read from the tree when it is here.
    #[test]
    fn the_text_and_places_are_the_designers() {
        let Some(designer) =
            crate::config_coverage::source::csharp("Controls/SpectrogramUI.Designer.cs")
        else {
            eprintln!("skipped: the C# tree is not checked out");
            return;
        };
        let place = |(x, y, w, h): (f32, f32, f32, f32)| {
            (format!("Point({x}, {y})"), format!("Size({w}, {h})"))
        };
        for (at, text) in [
            (LOAD_AT, LOAD_LOG),
            (UPDATE_AT, UPDATE),
            (SENSOR_AT, SENSOR_TEXT),
        ] {
            let (point, size) = place(at);
            assert!(designer.contains(&point), "{point}");
            assert!(designer.contains(&size), "{size}");
            assert!(designer.contains(&format!("Text = \"{text}\";")), "{text}");
        }
        for at in [MIN_AT, MAX_AT, GRAPH_AT] {
            let (point, size) = place(at);
            assert!(
                designer.contains(&point) && designer.contains(&size),
                "{point}"
            );
        }
        for (text, (x, y)) in [(MIN_TEXT, MIN_LABEL_AT), (MAX_TEXT, MAX_LABEL_AT)] {
            assert!(designer.contains(&format!("Text = \"{text}\";")));
            assert!(designer.contains(&format!("Point({x}, {y})")));
        }
        for sensor in SENSORS {
            assert!(designer.contains(&format!("\"{sensor}\"")), "{sensor}");
        }
        assert!(designer.contains("ClientSize = new System.Drawing.Size(800, 450)"));
        assert!(designer.contains(&format!("this.Text = \"{FORM_TEXT}\";")));
        // -80 and -20: the decimal's sign word, 0x80000000.
        assert_eq!(designer.matches("-2147483648});").count(), 4);
        // The Designer's lines end in CRLF: the decimal's parts, one a line, up to the comma.
        assert!(designer.contains("            80,"));
        assert!(designer.contains("            20,"));
        let cs =
            crate::config_coverage::source::csharp("Controls/SpectrogramUI.cs").unwrap_or_default();
        for title in PANE_TITLES {
            assert!(cs.contains(&format!("Title.Text = \"{title}\";")));
        }
        assert!(cs.contains(&format!("XAxis.Title.Text = \"{X_TITLE}\";")));
        assert!(cs.contains(&format!("YAxis.Title.Text = \"{Y_TITLE}\";")));
        assert!(cs.contains("YAxis.Scale.MajorStep = 20;"));
        assert!(cs.contains(&format!("ofd.Filter = \"{LOG_FILTER}\";")));
        assert!(cs.contains("new string[] {\"AccX\", \"AccY\", \"AccZ\"}"));
        assert!(cs.contains("new string[] {\"GyrX\", \"GyrY\", \"GyrZ\"}"));
        let generator = crate::config_coverage::source::csharp("ExtLibs/Utilities/Spectrogram.cs")
            .unwrap_or_default();
        assert!(generator.contains("var bins = 10;"));
        assert!(generator.contains("if (count > 2048)"));
        assert!(generator.contains("int divisor = 4;"));
    }

    /// Every wiring of the Designer is a handler here.
    #[test]
    fn every_wiring_is_ported() {
        let Some(designer) =
            crate::config_coverage::source::csharp("Controls/SpectrogramUI.Designer.cs")
        else {
            eprintln!("skipped: the C# tree is not checked out");
            return;
        };
        let wired: Vec<&str> = designer
            .lines()
            .filter(|line| line.contains(" += new System.EventHandler(this."))
            .filter_map(|line| line.split("(this.").nth(1))
            .filter_map(|rest| rest.split(')').next())
            .collect();
        assert_eq!(
            wired,
            [
                "but_loadlog_Click",
                "cmb_sensor_SelectedIndexChanged",
                "num_min_ValueChanged",
                "num_max_ValueChanged",
                "but_redraw_Click",
                "SpectrogramUI_Resize",
            ]
        );
        // `SpectrogramUI_Resize` is empty.
        let cs =
            crate::config_coverage::source::csharp("Controls/SpectrogramUI.cs").unwrap_or_default();
        let resize = cs
            .split("SpectrogramUI_Resize(object sender, EventArgs e)")
            .nth(1)
            .and_then(|rest| rest.split('}').next())
            .unwrap_or_default();
        assert_eq!(resize.trim(), "{");
    }

    /// `GetColor`: Max and above red, Min and below grey, the middle the hue 127/255 - and
    /// `HSL2RGB`'s sextants, `Convert.ToByte` rounding to even.
    #[test]
    fn the_colours_are_get_color() {
        assert_eq!(colour(-20.0, -80, -20), HOT);
        assert_eq!(colour(30.0, -80, -20), HOT);
        assert_eq!(colour(-80.0, -80, -20), COLD);
        assert_eq!(colour(-500.0, -80, -20), COLD);
        // -50: 127.5 of 255, `(byte)(255 - 127.5)` 127, the hue 0.498 - sextant 2, fract
        // 0.98824: r = m = 0.25, g = v = 0.75, b = m + v * sv * fract = 0.74412.
        assert_eq!(colour(-50.0, -80, -20), [64, 191, 190]);
        // A NaN is `(byte)NaN`, 0: red.
        assert_eq!(colour(f64::NAN, -80, -20), HOT);
        // Min equal to Max: 0/0 for the value itself, NaN, red; ±inf constrained either side.
        assert_eq!(colour(-20.0, -20, -20), HOT);
        assert_eq!(colour(-19.0, -20, -20), HOT);
        assert_eq!(colour(-21.0, -20, -20), COLD);
        // A sextant each: 0 red-to-yellow, 1, 2, 3, 4, 5 magenta.
        assert_eq!(hsl_to_rgb(0.0, 0.5, 0.5), [191, 64, 64]);
        assert_eq!(hsl_to_rgb(1.0 / 6.0, 0.5, 0.5), [191, 191, 64]);
        assert_eq!(hsl_to_rgb(2.0 / 6.0, 0.5, 0.5), [64, 191, 64]);
        assert_eq!(hsl_to_rgb(3.0 / 6.0, 0.5, 0.5), [64, 191, 191]);
        assert_eq!(hsl_to_rgb(4.0 / 6.0, 0.5, 0.5), [64, 64, 191]);
        assert_eq!(hsl_to_rgb(5.0 / 6.0, 0.5, 0.5), [191, 64, 191]);
        assert_eq!(hsl_to_rgb(1.0, 0.5, 0.5), COLD);
        assert_eq!(map_constrained(5.0, 0.0, 10.0, 0.0, 255.0), 127.5);
        assert!(map_constrained(f64::NAN, 0.0, 10.0, 0.0, 255.0).is_nan());
        assert_eq!(dotnet_int(8172.6), 8172);
        assert_eq!(dotnet_int(-1.9), -1);
        assert_eq!(dotnet_int(f64::INFINITY), i32::MIN);
        assert_eq!(dotnet_int(f64::NAN), i32::MIN);
    }

    /// The message path on 4096 ACC1 records at 1 kHz: four windows' worth, a quarter window
    /// apart, so 16 columns of which 13 are drawn; the rate the windows' spans give, (int)(1e6 *
    /// 1024 / 125296) = 8172, and the axis to bin 511 of it, 4078; AccZ's 100 Hz in bin 102 -
    /// red, at 6 dB over Max - and X and Y, all zeros, grey.
    #[test]
    fn the_message_path_draws_a_quarter_window_apart() {
        let log = acc_gyr_log(4096);
        let drawn = generate(&log, "ACC1", -80, -20).expect("drawn");
        assert_eq!(drawn.sensor, "ACC1");
        let fields: Vec<&str> = drawn.panes.iter().map(|pane| pane.field).collect();
        assert_eq!(fields, ACC_FIELDS);
        let z = drawn.panes.get(2).expect("Z");
        assert_eq!((z.image.width, z.image.height), (16, 512));
        assert_eq!(z.columns.len(), 13);
        assert_eq!(z.freqs.len(), 512);
        assert_eq!(z.freqs.get(1), Some(&7.0), "1 * 8172 / 1024, in int");
        assert_eq!(drawn.freqmax, 4078.0);
        assert_eq!(drawn.mintime, 1.0);
        assert_eq!(drawn.maxtime, 1.0 + 3072.0 / 1000.0);
        // The windows' first times, a quarter window apart.
        let starts: Vec<f64> = z.columns.iter().take(3).map(|(time, _)| *time).collect();
        assert_eq!(starts, [1_000_000.0, 1_256_000.0, 1_512_000.0]);
        // 100 Hz at 1 kHz: bin 102.4, the hottest bin 102 of every column.
        for column in 0..13 {
            assert_eq!(z.image.pixel(column, row_of(102)), Some(HOT), "{column}");
            assert_eq!(z.image.pixel(column, row_of(400)), Some(COLD), "{column}");
        }
        for column in 13..16 {
            assert_eq!(z.image.pixel(column, 0), None, "never drawn: transparent");
        }
        for pane in drawn.panes.iter().take(2) {
            assert_eq!(pane.image.count(COLD), 13 * 512, "{}", pane.field);
        }
        // The colours follow Max: at +10 the 6 dB bin is no longer red.
        let cooler = generate(&log, "ACC1", -80, 10).expect("drawn");
        let z = cooler.panes.get(2).expect("Z");
        assert_ne!(z.image.pixel(0, row_of(102)), Some(HOT));
        assert_eq!(z.image.count(HOT), 0);
    }

    /// A name with GYR in it draws GyrX, GyrY and GyrZ: GYR1's 250 Hz in Y, bin 256.
    #[test]
    fn gyr_draws_the_gyro() {
        let log = acc_gyr_log(2048);
        let drawn = generate(&log, "GYR1", -80, -20).expect("drawn");
        let fields: Vec<&str> = drawn.panes.iter().map(|pane| pane.field).collect();
        assert_eq!(fields, GYR_FIELDS);
        // Two windows' worth, four a window: 8 columns, 5 drawn.
        let y = drawn.panes.get(1).expect("Y");
        assert_eq!((y.image.width, y.columns.len()), (8, 5));
        assert_eq!(y.image.pixel(0, row_of(256)), Some(HOT));
        assert_eq!(
            drawn.panes.first().map(|pane| pane.image.count(HOT)),
            Some(0)
        );
    }

    /// A message the log has none of is the C#'s `acc1data[0]`; fewer samples than a window an
    /// image of no width; a field the message lacks is read as 0 - all grey, no error.
    #[test]
    fn what_the_c_sharp_throws_is_an_error() {
        let log = acc_gyr_log(1000);
        assert_eq!(
            generate(&log, "ACC2", -80, -20),
            Err(OUT_OF_RANGE.to_owned())
        );
        assert!(generate(&log, "ACC1", -80, -20).is_err_and(|error| error.contains("image of 0")));
        // BARO has no AccX, AccY or AccZ: each read as 0, all grey - one window of 1100.
        let mut bytes = fmt(202, 3 + 8 + 4, "BARO", "Qf", "TimeUS,Alt");
        for i in 0..1100_u64 {
            let mut baro = (1_000_000 + 1000 * i).to_le_bytes().to_vec();
            baro.extend(100.0_f32.to_le_bytes());
            bytes.extend(record(202, &baro));
        }
        let drawn = generate(&LogFile::from_bytes(bytes), "BARO", -80, -20).expect("drawn");
        let colds: Vec<usize> = drawn
            .panes
            .iter()
            .map(|pane| pane.image.count(COLD))
            .collect();
        assert_eq!(colds, [512, 512, 512]);
        assert_eq!(drawn.panes.first().map(|pane| pane.image.width), Some(4));
    }

    /// With `ISBH` the batches of the sensor the name picks: ACC1 is accelerometer 0, its X
    /// the 250 Hz sine at the header's 2000 Hz, bin 128, and its Y the 500 Hz one, bin 256 -
    /// each pane its own axis, where the C#'s cache gives Y and Z X's - the windows a window apart,
    /// the frequencies to bin 511 of 2000 Hz, 998. GYR2 picks gyro 1, which this log has none of.
    #[test]
    fn the_batch_path_reads_the_sensor_the_name_picks() {
        // 16 batches of 256: 4096 samples, 4 windows.
        let log = batch_log(16, &[(0, 0), (1, 1)]);
        let drawn = generate(&log, "ACC1", -80, -20).expect("drawn");
        let x = drawn.panes.first().expect("X");
        assert_eq!((x.image.width, x.columns.len()), (4, 4));
        assert_eq!(drawn.freqmax, 998.0);
        assert_eq!(x.freqs.get(1), Some(&1.0), "1 * 2000 / 1024");
        assert_eq!(x.image.pixel(0, row_of(128)), Some(HOT));
        let y = drawn.panes.get(1).expect("Y");
        assert_eq!(y.image.pixel(0, row_of(256)), Some(HOT));
        assert_ne!(y.image.pixel(0, row_of(128)), Some(HOT), "Y is not X");
        let z = drawn.panes.get(2).expect("Z");
        assert_eq!(z.image.count(COLD), 4 * 512);
        // Each window's time is its first batch's TimeUS: windows of 1024 a window apart.
        let starts: Vec<f64> = x.columns.iter().map(|(time, _)| *time).collect();
        assert_eq!(starts, [1_000_000.0, 1_400_000.0, 1_800_000.0, 2_200_000.0]);
        assert_eq!(drawn.mintime, 1.0);
        assert_eq!(drawn.maxtime, 2.2);
        // Gyro 1 batches exist, with the accelerometer's data in them: "GYR2" reads them.
        let gyro = generate(&log, "GYR2", -80, -20).expect("drawn");
        assert_eq!(gyro.panes.first().map(|pane| pane.columns.len()), Some(4));
        // Instance 2: none, the rate -1 and no samples, an image of no width.
        assert!(generate(&log, "ACC3", -80, -20).is_err());
    }

    /// The fixture: no `ACC1`, so the list's ACC1 throws; IMU typed draws its three IMUs read
    /// as one series - 1365 records, one window's worth, four columns of which two are drawn -
    /// at the rate the two windows' spans give. The numbers the GUI script expects.
    #[test]
    fn the_fixture_draws_its_imu() {
        let log = LogFile::open(fixture("dataflash.bin")).expect("testdata/dataflash.bin");
        assert_eq!(
            generate(&log, "ACC1", -80, -20),
            Err(OUT_OF_RANGE.to_owned())
        );
        let drawn = generate(&log, "IMU", -80, -20).expect("drawn");
        let x = drawn.panes.first().expect("X");
        assert_eq!(
            (x.image.width, x.image.height, x.columns.len()),
            (4, 512, 2)
        );
        // The two windows span 3.39 and 3.40 s of TimeUS; their average, 0.99 * 0.01 * the
        // first + 0.01 * the second, is 271446.76 us, so the rate (int)(1e6 * 1024 / that) is 3772
        // and bin 511 is 511 * 3772 / 1024 = 1882 - as pymavlink's reading of the same TimeUS
        // computes it.
        assert_eq!(drawn.freqmax, 1882.0);
        assert_eq!(x.freqs.get(1), Some(&3.0));
        assert_eq!((drawn.mintime, drawn.maxtime), (52.351_946, 55.752_249));
        let counts: Vec<usize> = drawn
            .panes
            .iter()
            .flat_map(|pane| [pane.image.count(HOT), pane.image.count(COLD)])
            .collect();
        assert_eq!(counts, [6, 24, 4, 31, 4, 17]);
        // Max up one: fewer bins at or above it.
        let max_up = generate(&log, "IMU", -80, -19).expect("drawn");
        let hot: Vec<usize> = max_up
            .panes
            .iter()
            .map(|pane| pane.image.count(HOT))
            .collect();
        assert_eq!(hot, [4, 4, 4]);
    }

    /// The texture: BGRA, nothing transparent, a wide image drawn from every so-many column.
    #[test]
    fn the_texture_is_bgra_and_capped() {
        let mut image = Image::new(3, 2).expect("an image");
        image.set(0, 0, [10, 20, 30]);
        let texture = image.texture();
        assert_eq!((texture.width(), texture.height()), (3, 2));
        assert_eq!(texture.get_pixel(0, 0).0, [30, 20, 10, 255]);
        assert_eq!(texture.get_pixel(1, 0).0, [0, 0, 0, 0]);
        let mut wide = Image::new(5000, 1).expect("an image");
        wide.set(3, 0, [1, 2, 3]);
        wide.set(4998, 0, [4, 5, 6]);
        let texture = wide.texture();
        // Every third column of 5000: 1667.
        assert_eq!(texture.width(), 1667);
        assert_eq!(texture.get_pixel(1, 0).0, [3, 2, 1, 255]);
        assert_eq!(texture.get_pixel(1666, 0).0, [6, 5, 4, 255]);
        assert!(Image::new(0, 512).is_err());
    }

    /// The axes: frequency from 0 every 20 to `freqt.Max()`, time ZedGraph's pick; three panes
    /// two by two, ten pixels apart.
    #[test]
    fn the_axes_and_panes_are_the_master_panes() {
        let frequency = frequency_scale(998.0);
        assert_eq!(frequency.tics().len(), 50);
        assert_eq!(frequency.tics().get(1), Some(&20.0));
        assert_eq!(frequency.label(980.0), "980");
        let time = time_scale(1.0, 2.2);
        assert!(time.tics().first().is_some_and(|tic| *tic >= 1.0));
        let [x, y, z] = pane_rects();
        assert_eq!(x, (0.0, 0.0, 383.0, 196.5));
        assert_eq!(y, (393.0, 0.0, 383.0, 196.5));
        assert_eq!(z, (0.0, 206.5, 383.0, 196.5));
    }

    /// The form as the Designer makes it, then Load Log: a path that is not a file does nothing,
    /// the fixture is read and nothing drawn.
    #[test]
    fn load_log_reads_and_draws_nothing() {
        let mut form = Spectrogram::new();
        assert_eq!(form.sensor.value(), "ACC1");
        assert_eq!(form.list.selected, Some(0));
        assert_eq!(
            (form.min.field.value(), form.max.field.value()),
            ("-80", "-20")
        );
        assert!(form.log_name().is_none() && form.drawn().is_none());
        form.press_load();
        assert_eq!(form.prompt(), Some(""));
        form.type_prompt("/no/such/file.bin");
        form.prompt_done(true);
        assert!(form.prompt().is_none() && form.running().is_none());
        form.press_load();
        form.type_prompt(&fixture("dataflash.bin").display().to_string());
        form.prompt_done(true);
        assert_eq!(form.running(), Some(Task::Load));
        assert_eq!(form.finish(), None);
        assert_eq!(form.log_name(), Some("dataflash.bin"));
        assert_eq!(form.last.as_deref(), Some("loaded"));
        assert!(form.drawn().is_none(), "Load Log draws nothing");
    }

    /// A draw before Load Log is the C#'s null `file`: on the status line, nothing drawn - from
    /// Update, from a value changed, and from a sensor chosen.
    #[test]
    fn a_draw_with_no_log_is_an_error() {
        let mut holder = SpectrogramWindow::default();
        holder.show();
        let form = holder.window.as_mut().expect("open");
        form.draw();
        assert_eq!(holder.tick(false, false), Some(NO_LOG.to_owned()));
        let form = holder.window.as_mut().expect("open");
        form.step(Edit::Max, 1.0);
        assert_eq!(form.max.field.value(), "-19");
        assert_eq!(form.take_error(), Some(NO_LOG.to_owned()));
        form.choose(3);
        assert_eq!(form.sensor.value(), "ACC4");
        assert_eq!(form.take_error(), Some(NO_LOG.to_owned()));
        assert!(form.drawn().is_none());
        // The close box: nothing drawn, no textures to drop.
        assert!(holder.close().is_empty());
        assert!(holder.window.is_none() && holder.tick(false, false).is_none());
    }

    /// The handlers' draws: a row chosen that moves the selection draws, the same row again
    /// does not; typed text leaves no row selected and draws nothing until Update; a value
    /// changed by its arrow draws; the controls wait for the thread.
    #[test]
    fn each_handler_draws_as_the_c_sharp_does() {
        let mut form = Spectrogram::new();
        form.log = Some(Loaded {
            name: "sines.bin".to_owned(),
            log: Arc::new(acc_gyr_log(2048)),
        });
        form.choose(0);
        assert_eq!(form.running(), None, "ACC1 is already selected");
        form.choose(5);
        assert_eq!(form.sensor.value(), "GYR1");
        assert_eq!(form.running(), Some(Task::Draw));
        // While it runs, the arrows and the list wait.
        form.step(Edit::Min, 1.0);
        assert_eq!(form.min.field.value(), "-80");
        assert_eq!(form.finish(), None);
        assert_eq!(
            form.drawn().map(|drawn| drawn.sensor.as_str()),
            Some("GYR1")
        );
        assert_eq!(form.textures.shown.len(), 3);
        assert!(form.textures.retired.borrow().is_empty());
        // Typed: no row selected, nothing drawn.
        form.begin(Edit::Sensor);
        form.type_sensor("ACC1");
        assert_eq!(form.list.selected, None);
        assert_eq!(form.running(), None);
        // Choosing ACC1 now moves the selection from none: a draw.
        form.choose(0);
        assert_eq!(form.running(), Some(Task::Draw));
        assert_eq!(form.finish(), None);
        assert_eq!(
            form.drawn().map(|drawn| drawn.sensor.as_str()),
            Some("ACC1")
        );
        // GYR1's textures replaced, waiting for the next paint to drop them from the atlas.
        assert_eq!(form.textures.shown.len(), 3);
        assert_eq!(form.textures.retired.borrow().len(), 3);
        // Min's down arrow: -81, a draw at -81.
        form.step(Edit::Min, -1.0);
        assert_eq!(form.min.field.value(), "-81");
        assert_eq!(form.running(), Some(Task::Draw));
        assert_eq!(form.finish(), None);
        // Update draws whatever the box says.
        form.type_sensor("GYR1");
        form.draw();
        assert_eq!(form.finish(), None);
        assert_eq!(
            form.drawn().map(|drawn| drawn.sensor.as_str()),
            Some("GYR1")
        );
        // A message the log lacks: on the status line, the graphs as they were.
        form.type_sensor("ACC2");
        form.draw();
        assert_eq!(form.finish(), Some(OUT_OF_RANGE.to_owned()));
        assert_eq!(
            form.drawn().map(|drawn| drawn.sensor.as_str()),
            Some("GYR1")
        );
    }

    /// Every fact the GUI script asserts on is recorded here, and every control it clicks is
    /// drawn here or by the dialog and list this form uses.
    #[test]
    fn the_gui_script_names_facts_and_controls_this_window_has() {
        let script = include_str!("../../../../tests/gui/config-spectrogram.gui");
        let source = include_str!("spectrogram.rs");
        let mut facts = 0;
        for line in script.lines() {
            let line = line.split('#').next().unwrap_or("");
            let mut words = line.split_whitespace();
            match (words.next(), words.next()) {
                (Some("expect"), Some(key)) if key.starts_with("config.spectrogram.") => {
                    let pane = key
                        .strip_prefix("config.spectrogram.pane.")
                        .and_then(|rest| rest.split_once('.'))
                        .map(|(_, what)| format!("config.spectrogram.pane.{{index}}.{what}"));
                    let recorded = source.contains(&format!("\"{key}\""))
                        || pane.is_some_and(|pane| source.contains(&format!("\"{pane}\"")));
                    assert!(recorded, "{key} is not recorded");
                    facts += 1;
                }
                (Some("click"), Some(id)) if id.starts_with("spect-") => {
                    let row = id.strip_prefix("spect-cmb_sensor-").is_some_and(|row| {
                        row.parse::<usize>().is_ok_and(|row| row < SENSORS.len())
                    });
                    let arrow = id.ends_with("-up") || id.ends_with("-down");
                    let drawn = source.contains(&format!("\"{id}\""))
                        || row
                        || (arrow
                            && source.contains(&format!(
                                "\"{}\"",
                                id.trim_end_matches("-up").trim_end_matches("-down")
                            )));
                    assert!(drawn, "{id} is not drawn");
                }
                _ => {}
            }
        }
        assert!(facts > 15, "{facts} facts");
    }
}
