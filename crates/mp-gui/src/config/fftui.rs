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

//! The FFT window: `Controls/fftui.cs`, which SETUP's FFT Setup page opens (`ConfigFFT.but_fft_Click`,
//! `GCSViews/ConfigurationView/ConfigFFT.cs:159-162`) and the Advanced page's FFT button opens too
//! (`ConfigAdvanced.cs:114-117`, still dimmed there).
//!
//! What it does: five buttons each ask for a file, read it, and draw spectra into a flow panel of
//! graphs - "Freq Hz" across, "Amplitude" up, each point's tooltip `"{0} hz/{1} rpm"`. The Bins
//! box is the FFT size's power of two (10: 1,024 samples), Start Freq zeroes the bins below it
//! (5 Hz), and Magnitude plots magnitudes where the default is decibels. The buttons, as the
//! Designer lays them out right to left (`fftui.Designer.cs:139-192`):
//!
//! * **IMU Batch Sample** (`but_ISBH_Click`, `fftui.cs:789-971`): the `ISBH` headers and `ISBD`
//!   batches `INS_LOG_BAT_MASK` logs - a series per sensor, `type * 6 + instance`, at the header's
//!   `smp_rate`, each sample `x / mul`; the panel cleared, one graph per sensor with data.
//! * **Run all imus - IMU1-3 MSG** (`but_fftimu13_Click`, `:602-769`): the `IMU`, `IMU2` and `IMU3`
//!   messages, a GYR and an ACC series per message type.
//! * **Run all imus - ACC GYR MSG** (`BUT_accgyrall_Click`, `:357-552`): `ACC1`-`ACC4` and
//!   `GYR1`-`GYR4`, or the instanced `ACC` and `GYR` of a newer log.
//! * **Run Log - imu1 ACC1 GYR1 MSG** (`acc1gyr1myButton1_Click`, `:170-353`): the first IMU's
//!   six axes, one graph each.
//! * **Run 16bit Mono Wav** (`BUT_runwav_Click`, `:21-131`): a sound file's samples, the rate
//!   asked for in an `InputBox`, drawn into the Designer's own graph.
//!
//! The sample rate of the log buttons is estimated from the samples' `TimeUS`, an exponential
//! average of the gaps ([`fft::SampleClock`]); the spectrum is `FFT2.rin` over `N`-sample slices
//! averaged with the last slice skipped ([`fft::average`]). All of that is `mp_log::fft`'s; this
//! module gathers the series as each button gathers them and lays the graphs out.
//!
//! Where this is not the C#, each written at its site:
//!
//! * the window is a panel over SETUP, modal, where the C#'s is a free form (`Show`, not
//!   `ShowDialog`); FFT on the page opens a fresh one, as `new fftui()` does;
//! * the file dialogs are a typed path ([`crate::config::firmware::PathBox`]), and the handler runs
//!   off the UI thread, the buttons dimmed while it does;
//! * an exception the C# would throw - a log that will not open, a field the log lacks, a sensor
//!   number past its array, a Bins value `1 << bins` cannot be - goes on the status line, as the
//!   owner ruled (2026-09-25); the C# shows its unhandled-exception box;
//! * a Bins value over 20 (a million samples) is refused rather than allocated;
//! * the form does not resize, so `fftui_Resize`'s rescale never runs; the graphs are the size
//!   `SetScale` gives them in the Designer's 785 x 489 panel;
//! * a curve the C# draws `Color.Black` is drawn in the theme's text colour, which is what shows
//!   on this application's dark graph.
//!
//! Nothing in `fftui.cs` is dead: the five buttons, the check box and both boxes are wired in the
//! Designer, and every member has a user. `zedGraphControl1_MouseMoveEvent` only debounces the
//! tooltip and changes nothing a user sees. Of `FFT2.datastate` (`ExtLibs/Utilities/fft.cs:36-49`)
//! the `avgx`, `avgy` and `avgz` arrays have no users (PLAN §12 D16): not ported.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::collections::BTreeMap;
use std::path::PathBuf;
use std::rc::Rc;
use std::sync::mpsc::{Receiver, TryRecvError};

use gpui::{
    AnyElement, Bounds, Context, FocusHandle, KeyDownEvent, Pixels, SharedString, Window, canvas,
    div, prelude::*, px, rgb,
};
use mp_log::dataflash::{LogMessage, Value};
use mp_log::fft;
use mp_log::logfile::LogFile;

use super::firmware::{BoxIds, PathBox, path_box};
use super::motor_test::NumericUpDown;
use super::optional::{InputBox, at, button, input_box, label};
use crate::MissionPlanner;
use crate::textfield::KeyOutcome;
use crate::ui::theme;

// ---------------------------------------------------------------------------------------------
// The Designer's words and places.
// ---------------------------------------------------------------------------------------------

/// The form's caption, `this.Text`.
/// `// C#: Controls/fftui.Designer.cs:209`
pub const FORM_TEXT: &str = "fftui";

/// `ClientSize`. `// C#: Controls/fftui.Designer.cs:196`
pub const CLIENT: (f32, f32) = (809.0, 542.0);

/// `tableLayoutPanel1`, the flow panel the graphs go in, scrolling.
/// `// C#: Controls/fftui.Designer.cs:75-79`
pub const PANEL_AT: (f32, f32, f32, f32) = (12.0, 12.0, 785.0, 489.0);

/// `label1.Text`. `// C#: Controls/fftui.Designer.cs:98-102`
pub const BINS: &str = "Bins";
/// `label2.Text`. `// C#: Controls/fftui.Designer.cs:108-112`
pub const START_FREQ: &str = "Start Freq";
/// `chk_mag.Text`. `// C#: Controls/fftui.Designer.cs:131-135`
pub const MAGNITUDE: &str = "Magnitude";

/// `NUM_bins`: `Value` 10, and a `NumericUpDown`'s default bounds, 0 to 100.
/// `// C#: Controls/fftui.Designer.cs:84-92`
const BINS_AT: (f32, f32, f32, f32) = (45.0, 507.0, 42.0, 20.0);
/// `NUM_startfreq`: `Value` 5, 0 to 100. `// C#: Controls/fftui.Designer.cs:117-125`
const START_AT: (f32, f32, f32, f32) = (152.0, 507.0, 42.0, 20.0);
/// `chk_mag`. `// C#: Controls/fftui.Designer.cs:131-133`
const MAG_AT: (f32, f32) = (200.0, 510.0);

/// A `NumericUpDown`'s default `Maximum`, which neither box changes.
const NUMERIC_MAX: f64 = 100.0;

/// The largest Bins this port transforms: `1 << 20`, a million samples. The C# allocates
/// whatever `1 << bins` asks for, and runs out of memory long before `NUM_bins` runs out of
/// range.
pub const MAX_BINS: i32 = 20;

/// `SetScale`'s cell: a third of the panel's width and half its height, less 30, in `int`.
/// `// C#: Controls/fftui.cs:583-586`
pub const GRAPH_SIZE: (f32, f32) = (251.0, 229.0);

/// A control's `Margin` in a `FlowLayoutPanel`: three pixels each side.
const FLOW_MARGIN: f32 = 3.0;

/// `"Freq Hz"`, every spectrum's x title. `// C#: Controls/fftui.cs:85, 333, 528, 745, 949`
pub const X_TITLE: &str = "Freq Hz";
/// `"Amplitude"`. `// C#: Controls/fftui.cs:86, 334, 529, 746, 950`
pub const Y_TITLE: &str = "Amplitude";

/// A new `ZedGraphControl`'s pane: its title and axis titles, `ZedGraphLocale.resx`'s
/// `title_def`, `x_title_def` and `y_title_def`.
/// `// C#: ExtLibs/ZedGraph/ZedGraph/ZedGraphControl.cs:527-533`
pub const BLANK_TITLE: &str = "Title";
/// `x_title_def`.
pub const BLANK_X: &str = "X Axis";
/// `y_title_def`.
pub const BLANK_Y: &str = "Y Axis";

/// The log buttons' `OpenFileDialog.Filter`. `// C#: Controls/fftui.cs:176, 363, 608, 795`
pub const LOG_FILTER: &str = "*.log;*.bin|*.log;*.bin;*.BIN;*.LOG";
/// Run 16bit Mono Wav's. `// C#: Controls/fftui.cs:27`
pub const WAV_FILTER: &str = "*.wav|*.wav";

/// Run 16bit Mono Wav's `InputBox`: its caption, its question (the C#'s, first letter missing),
/// and the rate it starts at.
/// `// C#: Controls/fftui.cs:45-46`
pub const RATE_TITLE: &str = "fft sample rate";
/// The question.
pub const RATE_PROMPT: &str = "nter source file sample rate";
/// `int hz = 8000`.
pub const RATE_DEFAULT: i32 = 8000;

/// `int.Parse`'s `FormatException`, what `InputBox.Show(..., ref int)` throws for text that is
/// not a whole number. `// C#: ExtLibs/Controls/InputBox.cs:21-27`
pub const NOT_A_NUMBER: &str = "Input string was not in a correct format.";

/// .NET's `IndexOutOfRangeException` text: what the handlers throw for a sensor number past
/// their six (or twelve) series.
pub const OUT_OF_RANGE: &str = "Index was outside the bounds of the array.";

/// The six colours `acc1gyr1myButton1_Click` and `BUT_runwav_Click` take their curves' from, in
/// order: Red, Green, Black, Violet, Blue, Orange. Black is the theme's text colour here.
/// `// C#: Controls/fftui.cs:42-43, 211-212`
const COLOURS_ACC1: [u32; 6] = [
    0xff_0000,
    0x00_8000,
    theme::TEXT,
    0xee_82ee,
    0x00_00ff,
    0xff_a500,
];
/// The run-all buttons' x, y and z: Red, Green, Blue. `// C#: Controls/fftui.cs:376-377, 522-524`
const COLOURS_XYZ: [u32; 3] = [0xff_0000, 0x00_8000, 0x00_00ff];

// ---------------------------------------------------------------------------------------------
// The buttons.
// ---------------------------------------------------------------------------------------------

/// The five buttons.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Run {
    /// `but_ISBH`, "IMU Batch Sample".
    Isbh,
    /// `but_fftimu13`, "Run all imus - IMU1-3 MSG".
    Imu13,
    /// `BUT_accgyrall`, "Run all imus - ACC GYR MSG".
    AccGyrAll,
    /// `but_accgyr1`, "Run Log - imu1 ACC1 GYR1 MSG".
    AccGyr1,
    /// `BUT_runwav`, "Run 16bit Mono Wav".
    Wav,
}

impl Run {
    /// The five, left to right as the Designer places them.
    pub const ALL: [Self; 5] = [
        Self::Isbh,
        Self::Imu13,
        Self::AccGyrAll,
        Self::AccGyr1,
        Self::Wav,
    ];

    /// The control id.
    #[must_use]
    pub const fn id(self) -> &'static str {
        match self {
            Self::Isbh => "fft-but_ISBH",
            Self::Imu13 => "fft-but_fftimu13",
            Self::AccGyrAll => "fft-BUT_accgyrall",
            Self::AccGyr1 => "fft-but_accgyr1",
            Self::Wav => "fft-BUT_runwav",
        }
    }

    /// The text. `// C#: Controls/fftui.Designer.cs:146, 157, 168, 179, 190`
    #[must_use]
    pub const fn text(self) -> &'static str {
        match self {
            Self::Isbh => "IMU Batch Sample",
            Self::Imu13 => "Run all imus - IMU1-3 MSG",
            Self::AccGyrAll => "Run all imus - ACC GYR MSG",
            Self::AccGyr1 => "Run Log - imu1 ACC1 GYR1 MSG",
            Self::Wav => "Run 16bit Mono Wav",
        }
    }

    /// `Location` and `Size`. `// C#: Controls/fftui.Designer.cs:142-144, 153-155, 164-166,
    /// 175-177, 186-188`
    #[must_use]
    pub const fn at(self) -> (f32, f32, f32, f32) {
        match self {
            Self::Isbh => (362.0, 508.0, 75.0, 32.0),
            Self::Imu13 => (443.0, 508.0, 75.0, 32.0),
            Self::AccGyrAll => (524.0, 508.0, 87.0, 32.0),
            Self::AccGyr1 => (617.0, 507.0, 99.0, 33.0),
            Self::Wav => (722.0, 507.0, 75.0, 33.0),
        }
    }

    /// Its file dialog's filter.
    #[must_use]
    pub const fn filter(self) -> &'static str {
        match self {
            Self::Wav => WAV_FILTER,
            _ => LOG_FILTER,
        }
    }

    /// What a fact calls it.
    #[must_use]
    pub const fn key(self) -> &'static str {
        match self {
            Self::Isbh => "isbh",
            Self::Imu13 => "imu13",
            Self::AccGyrAll => "accgyrall",
            Self::AccGyr1 => "accgyr1",
            Self::Wav => "wav",
        }
    }
}

// ---------------------------------------------------------------------------------------------
// The graphs.
// ---------------------------------------------------------------------------------------------

/// One `LineItem`.
#[derive(Debug, Clone, PartialEq)]
pub struct Curve {
    /// Its label, which the legend shows.
    pub name: String,
    /// Its colour.
    pub colour: u32,
    /// `(x, y)`: frequency and amplitude.
    pub points: Vec<(f64, f64)>,
    /// `SymbolType.Diamond` rather than `None`.
    pub symbols: bool,
    /// `IsY2Axis`.
    pub y2: bool,
}

impl Curve {
    /// A curve without symbols on the left axis, of `freqs` against `values`.
    fn line(name: String, colour: u32, freqs: &[f64], values: &[f64]) -> Self {
        Self {
            name,
            colour,
            points: freqs.iter().copied().zip(values.iter().copied()).collect(),
            symbols: false,
            y2: false,
        }
    }

    /// The curve as a series `mp_chart` reduces: x as the series' `at`.
    fn series(&self) -> mp_chart::Series {
        let mut series = mp_chart::Series::new(self.name.clone(), self.points.len().max(1));
        for (x, y) in &self.points {
            series.push(*x, *y);
        }
        series
    }
}

/// One `ZedGraphControl`'s pane.
#[derive(Debug, Clone, PartialEq)]
pub struct Graph {
    /// `GraphPane.Title.Text`.
    pub title: String,
    /// `XAxis.Title.Text`.
    pub x_title: &'static str,
    /// `YAxis.Title.Text`.
    pub y_title: &'static str,
    /// `CurveList`.
    pub curves: Vec<Curve>,
    /// `Legend.IsVisible`.
    pub legend: bool,
    /// `XAxis.Scale.Max`, where a handler sets it: half the sample rate.
    pub x_max: Option<f64>,
    /// `YAxis.Scale.Max`, where `SetScale` sets it.
    pub y_max: Option<f64>,
    /// `Y2Axis.IsVisible`.
    pub y2: bool,
}

impl Graph {
    /// `NewZedGraph()`, or the Designer's `zedGraphControl1`, before anything is drawn in it.
    #[must_use]
    pub fn blank() -> Self {
        Self {
            title: BLANK_TITLE.to_owned(),
            x_title: BLANK_X,
            y_title: BLANK_Y,
            curves: Vec::new(),
            legend: true,
            x_max: None,
            y_max: None,
            y2: false,
        }
    }

    /// The x range the pane shows: from the lowest x, to `x_max` where it is set.
    #[must_use]
    pub fn x_range(&self) -> Option<(f64, f64)> {
        let mut low = f64::INFINITY;
        let mut high = f64::NEG_INFINITY;
        for curve in &self.curves {
            for (x, _) in &curve.points {
                low = low.min(*x);
                high = high.max(*x);
            }
        }
        if !low.is_finite() {
            return None;
        }
        let high = self.x_max.unwrap_or(high);
        (high > low).then_some((low, high))
    }

    /// The y range an axis shows: its curves' range with `mp_chart`'s padding, the top at
    /// `YAxis.Scale.Max` where `SetScale` set it.
    #[must_use]
    pub fn y_range(&self, y2: bool) -> Option<mp_chart::Range> {
        let (from, to) = self.x_range()?;
        let series: Vec<mp_chart::Series> = self
            .curves
            .iter()
            .filter(|curve| curve.y2 == y2)
            .map(Curve::series)
            .collect();
        let borrowed: Vec<&mp_chart::Series> = series.iter().collect();
        let mut range = mp_chart::auto_range(&borrowed, from, to)?;
        if !y2 && let Some(max) = self.y_max {
            range.high = max;
            if range.low >= range.high {
                range.low = range.high - 1.0;
            }
        }
        Some(range)
    }

    /// `YAxis.Scale.Max` after `AxisChange`: the top of the axis the data gives it.
    fn scale_max(&self) -> f64 {
        self.y_max
            .or_else(|| self.y_range(false).map(|range| range.high))
            .unwrap_or(0.0)
    }
}

/// `SetScale`: every graph titled with GYR given the highest y maximum of them, and every one
/// titled with ACC the highest of those; the maxima start at zero.
/// `// C#: Controls/fftui.cs:554-595`
pub fn set_scale(graphs: &mut [&mut Graph]) {
    let mut max_gyr: f64 = 0.0;
    let mut max_acc: f64 = 0.0;
    for graph in graphs.iter() {
        if graph.title.contains("GYR") {
            max_gyr = max_gyr.max(graph.scale_max());
        } else if graph.title.contains("ACC") {
            max_acc = max_acc.max(graph.scale_max());
        }
    }
    for graph in graphs.iter_mut() {
        if graph.title.contains("GYR") {
            graph.y_max = Some(max_gyr);
        } else if graph.title.contains("ACC") {
            graph.y_max = Some(max_acc);
        }
    }
}

// ---------------------------------------------------------------------------------------------
// The handlers.
// ---------------------------------------------------------------------------------------------

/// What the handlers read from the window's controls.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Settings {
    /// `(int)NUM_bins.Value`.
    pub bins: i32,
    /// `(double)NUM_startfreq.Value`.
    pub start_freq: f64,
    /// `indB`: `!chk_mag.Checked`.
    pub in_db: bool,
}

impl Default for Settings {
    /// The Designer's: 10, 5, Magnitude unticked.
    fn default() -> Self {
        Self {
            bins: 10,
            start_freq: fft::DEFAULT_START_FREQ,
            in_db: true,
        }
    }
}

/// `int N = 1 << bins`: C#'s shift takes the count's low five bits. A size under two, the
/// negative `1 << 31`, and one over [`MAX_BINS`] are refused.
/// `// C#: Controls/fftui.cs:185-187`
pub fn size(bins: i32) -> Result<usize, String> {
    let shift = bins & 31;
    if !(1..=MAX_BINS).contains(&shift) {
        return Err(format!(
            "Bins {bins}: 1 << {bins} samples is not an FFT size this window takes (1 to {MAX_BINS})"
        ));
    }
    Ok(1_usize << shift)
}

/// What a run leaves in the panel.
#[derive(Debug, Clone, PartialEq)]
pub enum Outcome {
    /// Graphs added after those already there: the run-all and imu1 buttons' six.
    Append(Vec<Graph>),
    /// The panel cleared first, then these: IMU Batch Sample's.
    Replace(Vec<Graph>),
    /// Run 16bit Mono Wav: the Designer's graph moved to the end of the panel, with this drawn
    /// in it when a whole window of samples was read.
    Wav(Option<Graph>),
}

/// `GetEnumeratorType(types)`'s wanted instances: each name, and for a name ending in a digit
/// the name without it and the instance one less (`ACC1` asks for `ACC` instance 0 as well).
/// An empty instance is every instance.
/// `// C#: ExtLibs/Utilities/DFLogBuffer.cs:700-733`
#[must_use]
pub fn wanted(types: &[&str]) -> BTreeMap<String, Vec<String>> {
    let mut instances: BTreeMap<String, Vec<String>> = BTreeMap::new();
    for name in types {
        // `(\w+)(\[([0-9]+)\])?`: a name without brackets, every instance.
        instances
            .entry((*name).to_owned())
            .or_default()
            .push(String::new());
        // `(\w+)([0-9]+)$`: `\w+` greedy, so one trailing digit.
        let mut chars = name.chars();
        if let Some(last) = chars.next_back()
            && let Some(digit) = last.to_digit(10)
            && !chars.as_str().is_empty()
        {
            let instance = i64::from(digit) - 1;
            instances
                .entry(chars.as_str().to_owned())
                .or_default()
                .push(instance.to_string());
        }
    }
    instances
}

/// A decoded value as `raw[i].ToString()` writes it.
fn value_text(value: &Value) -> String {
    match value {
        Value::Int(v) => v.to_string(),
        Value::Uint(v) => v.to_string(),
        Value::Float(v) => mp_log::netfmt::double(*v),
        Value::Text(text) => text.clone(),
        Value::Bytes(_) | Value::Samples(_) => String::new(),
    }
}

/// The records `GetEnumeratorType(types)` gives, in log order, each with its `instance`: the
/// value of the field its `FMTU` marks, or `""` for a message with none.
/// `// C#: ExtLibs/Utilities/DFLogBuffer.cs:700-770; ExtLibs/Utilities/DFLog.cs:53-77`
pub fn records(log: &LogFile, types: &[&str]) -> Vec<(LogMessage, String)> {
    let wanted = wanted(types);
    let labels = log.instance_fields();
    let names: Vec<&str> = wanted.keys().map(String::as_str).collect();
    log.messages(&names)
        .filter_map(|(_, message)| {
            let instance = labels
                .get(&message.name)
                .and_then(|label| message.field(label))
                .map(value_text)
                .unwrap_or_default();
            let asked = wanted.get(&message.name)?;
            (asked.iter().any(String::is_empty) || asked.contains(&instance))
                .then_some((message, instance))
        })
        .collect()
}

/// A field as a number: `double.Parse(item.items[offset])` or `Convert.ToDouble(item.raw[...])`.
/// A field the message lacks is `FindMessageOffset`'s -1 and an index the C# throws on.
fn number(message: &LogMessage, field: &str) -> Result<f64, String> {
    message
        .field(field)
        .and_then(Value::as_f64)
        .ok_or_else(|| format!("{}: no numeric {field} field", message.name))
}

/// `FFT2.datastate`: a sensor's samples and its clock. `avgx`, `avgy` and `avgz` have no users.
/// `// C#: ExtLibs/Utilities/fft.cs:36-49`
#[derive(Debug, Clone, Default)]
pub struct DataState {
    /// `type`: the graph's name for the sensor, `null` until a record names it.
    pub kind: Option<String>,
    /// `timedelta` and `lasttime`.
    pub clock: fft::SampleClock,
    /// `sample_rate`: IMU Batch Sample's, from the header.
    pub sample_rate: f64,
    /// `datax`.
    pub x: Vec<f64>,
    /// `datay`.
    pub y: Vec<f64>,
    /// `dataz`.
    pub z: Vec<f64>,
}

impl DataState {
    /// `timedelta = timedelta * 0.99 + (time - lasttime) * 0.01` where the time moved, and
    /// `lasttime = time`, without the run-all buttons' refusal of a time gone back:
    /// `but_fftimu13_Click`'s clock. `// C#: Controls/fftui.cs:660-664`
    fn tick_unguarded(&mut self, time_ms: f64) {
        #[allow(clippy::float_cmp)] // the C#'s `time != lasttime`, exactly
        if time_ms != self.clock.lasttime {
            self.clock.timedelta =
                self.clock.timedelta * 0.99 + (time_ms - self.clock.lasttime) * 0.01;
        }
        self.clock.lasttime = time_ms;
    }

    /// Adds one sample of each axis.
    fn push(&mut self, (x, y, z): (f64, f64, f64)) {
        self.x.push(x);
        self.y.push(y);
        self.z.push(z);
    }
}

/// The graph a run-all button draws for a sensor with more than `N` samples: `type + " x"`,
/// `" y"`, `" z"` in red, green and blue, the legend on, the x axis to half the rate.
/// `// C#: Controls/fftui.cs:480-548, 697-765, 897-967`
fn sensor_graph(
    state: &DataState,
    n: usize,
    sample_rate: f64,
    file_name: &str,
    settings: Settings,
) -> Option<Graph> {
    let kind = state.kind.clone().unwrap_or_default();
    let mut curves = Vec::new();
    for (axis, (data, colour)) in ["x", "y", "z"]
        .into_iter()
        .zip([&state.x, &state.y, &state.z].into_iter().zip(COLOURS_XYZ))
    {
        let spectrum = fft::average(data, n, sample_rate, settings.start_freq, settings.in_db)?;
        curves.push(Curve::line(
            format!("{kind} {axis}"),
            colour,
            &spectrum.freqs,
            &spectrum.amplitudes,
        ));
    }
    Some(Graph {
        title: fft::title(&kind, file_name, sample_rate),
        x_title: X_TITLE,
        y_title: Y_TITLE,
        curves,
        legend: true,
        x_max: Some(sample_rate / 2.0),
        y_max: None,
        y2: false,
    })
}

/// The run-all buttons' six controls: a graph per sensor with data in order, then blank ones
/// to six, `SetScale` over the six.
/// `// C#: Controls/fftui.cs:378-381, 478-550`
fn six(states: &[DataState], n: usize, file_name: &str, settings: Settings) -> Vec<Graph> {
    let mut graphs: Vec<Graph> = states
        .iter()
        .filter(|state| state.x.len() > n)
        .filter_map(|state| {
            let rate = state.clock.sample_rate();
            sensor_graph(state, n, rate, file_name, settings)
        })
        .collect();
    graphs.resize_with(6.max(graphs.len()), Graph::blank);
    let mut all: Vec<&mut Graph> = graphs.iter_mut().collect();
    set_scale(&mut all);
    graphs
}

/// A state by sensor number, or the `IndexOutOfRangeException` the C#'s array throws.
fn state_at(states: &mut [DataState], sensor: i64) -> Result<&mut DataState, String> {
    usize::try_from(sensor)
        .ok()
        .and_then(|index| states.get_mut(index))
        .ok_or_else(|| OUT_OF_RANGE.to_owned())
}

/// `int.Parse(s)` of a record's instance or of its name's digit.
fn parse_int(text: &str) -> Result<i64, String> {
    text.parse()
        .map_err(|_| format!("{text:?}: {NOT_A_NUMBER}"))
}

/// Run all imus - ACC GYR MSG: `ACC1`-`ACC4`, `GYR1`-`GYR4` (and a newer log's instanced
/// `ACC`/`GYR`), a gyro in series 0-2 and an accelerometer in 3-5, by the instance or the name's
/// digit; each series' clock refuses a time gone back.
/// `// C#: Controls/fftui.cs:357-552`
pub fn accgyrall(log: &LogFile, file_name: &str, settings: Settings) -> Result<Outcome, String> {
    let n = size(settings.bins)?;
    let mut states = vec![DataState::default(); 6];
    let types = [
        "ACC1", "GYR1", "ACC2", "GYR2", "ACC3", "GYR3", "ACC4", "GYR4",
    ];
    for (message, instance) in records(log, &types) {
        let (base, fields) = if message.name.starts_with("ACC") {
            (3, ["AccX", "AccY", "AccZ"])
        } else if message.name.starts_with("GYR") {
            (0, ["GyrX", "GyrY", "GyrZ"])
        } else {
            continue;
        };
        let sensor = if instance.is_empty() {
            parse_int(message.name.get(3..).unwrap_or_default())? - 1 + base
        } else {
            parse_int(&instance)? + base
        };
        let state = state_at(&mut states, sensor)?;
        state.kind = Some(message.name.clone());
        let time = number(&message, "TimeUS")? / 1000.0;
        if !state.clock.accept(time) {
            continue;
        }
        state.push((
            number(&message, fields[0])?,
            number(&message, fields[1])?,
            number(&message, fields[2])?,
        ));
    }
    Ok(Outcome::Append(six(&states, n, file_name, settings)))
}

/// Run all imus - IMU1-3 MSG: `IMU`, `IMU2`, `IMU3` by message name - a newer log's instanced
/// `IMU` is every IMU under the one name, all in series 0, as the C# has it - the gyro in series
/// 0-2 as `"IMU GYR"`, the accelerometer in 3-5 as `"IMU ACC"`; the clock takes every time.
/// `// C#: Controls/fftui.cs:602-769`
pub fn imu13(log: &LogFile, file_name: &str, settings: Settings) -> Result<Outcome, String> {
    let n = size(settings.bins)?;
    let mut states = vec![DataState::default(); 6];
    for (message, _) in records(log, &["IMU", "IMU2", "IMU3"]) {
        let sensor = match message.name.as_str() {
            "IMU2" => 1,
            "IMU3" => 2,
            _ => 0,
        };
        let time = number(&message, "TimeUS")? / 1000.0;
        let acc = (
            number(&message, "AccX")?,
            number(&message, "AccY")?,
            number(&message, "AccZ")?,
        );
        let gyr = (
            number(&message, "GyrX")?,
            number(&message, "GyrY")?,
            number(&message, "GyrZ")?,
        );
        let accel = state_at(&mut states, sensor + 3)?;
        accel.kind = Some(format!("{} ACC", message.name));
        accel.tick_unguarded(time);
        accel.push(acc);
        let gyro = state_at(&mut states, sensor)?;
        gyro.kind = Some(format!("{} GYR", message.name));
        gyro.tick_unguarded(time);
        gyro.push(gyr);
    }
    Ok(Outcome::Append(six(&states, n, file_name, settings)))
}

/// IMU Batch Sample: each `ISBH` names the sensor (`type * 6 + instance`), the batch number
/// `N`, the rate and the multiplier; each `ISBD` of that batch number adds its 32 samples a
/// axis, divided by the multiplier. The panel is cleared, and a graph drawn for each sensor with
/// more than `N` samples, at the header's rate.
/// `// C#: Controls/fftui.cs:789-971`
pub fn isbh(log: &LogFile, file_name: &str, settings: Settings) -> Result<Outcome, String> {
    let n = size(settings.bins)?;
    let mut states = vec![DataState::default(); 12];
    let mut batch: f64 = 0.0;
    let mut sensor: i64 = 0;
    let mut multiplier: f64 = -1.0;
    for (message, _) in records(log, &["ISBH", "ISBD"]) {
        if message.name.starts_with("ISBH") {
            batch = number(&message, "N")?;
            let kind = number(&message, "type")?;
            let instance = number(&message, "instance")?;
            #[allow(clippy::cast_possible_truncation)] // `int.Parse` of a byte field
            let (kind, instance) = (kind as i64, instance as i64);
            sensor = kind * 6 + instance;
            let rate = number(&message, "smp_rate")?;
            multiplier = number(&message, "mul")?;
            let state = state_at(&mut states, sensor)?;
            state.sample_rate = rate;
            if kind == 0 {
                state.kind = Some(format!("ACC{instance}"));
            }
            if kind == 1 {
                state.kind = Some(format!("GYR{instance}"));
            }
        } else if message.name.starts_with("ISBD") {
            let Ok(state) = state_at(&mut states, sensor) else {
                continue;
            };
            #[allow(clippy::float_cmp)] // two whole numbers from the log
            if number(&message, "N")? != batch {
                continue;
            }
            let time = number(&message, "TimeUS")? / 1000.0;
            if !state.clock.accept(time) {
                continue;
            }
            for (field, data) in [
                ("x", &mut state.x),
                ("y", &mut state.y),
                ("z", &mut state.z),
            ] {
                let Some(Value::Samples(samples)) = message.field(field) else {
                    return Err(format!("ISBD: no {field} samples"));
                };
                data.extend(samples.iter().map(|sample| f64::from(*sample) / multiplier));
            }
        }
    }
    let mut graphs: Vec<Graph> = states
        .iter()
        .filter(|state| state.x.len() > n)
        .filter_map(|state| sensor_graph(state, n, state.sample_rate, file_name, settings))
        .collect();
    let mut all: Vec<&mut Graph> = graphs.iter_mut().collect();
    set_scale(&mut all);
    Ok(Outcome::Replace(graphs))
}

/// Run Log - imu1 ACC1 GYR1 MSG: the first IMU's `ACC1`/`GYR1` (or `ACC[0]`/`GYR[0]`) into six
/// `N`-sample buffers, each full set transformed and added in at `1 / (N / 2)`; the rate from the
/// accelerometer's clock, which averages every gap - a time gone back and the gaps while its
/// buffer waits for the gyro's included - and moves on only with a sample kept. Six graphs,
/// one curve each, no legend, the bins below Start Freq zeroed.
/// `// C#: Controls/fftui.cs:170-353`
pub fn accgyr1(log: &LogFile, file_name: &str, settings: Settings) -> Result<Outcome, String> {
    let n = size(settings.bins)?;
    let half = n / 2;
    // datainGX, GY, GZ, AX, AY, AZ.
    let mut data = vec![vec![0.0; n]; 6];
    let mut avg = vec![vec![0.0; half]; 6];
    let heads = [
        "GYR1-GyrX",
        "GYR1-GyrY",
        "GYR1-GyrZ",
        "ACC1-AccX",
        "ACC1-AccY",
        "ACC1-AccZ",
    ];
    let (mut count_a, mut count_g) = (0_usize, 0_usize);
    let (mut lasttime, mut timedelta) = (0.0_f64, 0.0_f64);
    let mut accels = 0_usize;
    #[allow(clippy::cast_precision_loss)] // an FFT size
    let weight = 1.0 / (n as f64 / 2.0);
    for (message, instance) in records(log, &["ACC1", "GYR1"]) {
        let (first, fields, count) =
            if message.name == "ACC1" || message.name == "ACC" && instance == "0" {
                let time = number(&message, "TimeUS")? / 1000.0;
                timedelta = timedelta * 0.99 + (time - lasttime) * 0.01;
                accels += 1;
                // "we missed gyro data"
                if count_a >= n {
                    continue;
                }
                lasttime = time;
                (3, ["AccX", "AccY", "AccZ"], &mut count_a)
            } else if message.name == "GYR1" || message.name == "GYR" && instance == "0" {
                number(&message, "TimeUS")?;
                // "we missed accel data"
                if count_g >= n {
                    continue;
                }
                (0, ["GyrX", "GyrY", "GyrZ"], &mut count_g)
            } else {
                continue;
            };
        for (axis, field) in fields.iter().enumerate() {
            let value = number(&message, field)?;
            if let Some(slot) = data
                .get_mut(first + axis)
                .and_then(|buffer| buffer.get_mut(*count))
            {
                *slot = value;
            }
        }
        *count += 1;
        if count_a >= n && count_g >= n {
            for (buffer, sum) in data.iter().zip(avg.iter_mut()) {
                let answer = fft::rin(buffer, settings.in_db).unwrap_or_default();
                for (total, bin) in sum.iter_mut().zip(&answer) {
                    *total += bin * weight;
                }
            }
            count_a = 0;
            count_g = 0;
        }
    }
    if accels == 0 {
        return Err(format!(
            "{file_name}: no ACC1 or ACC[0] messages to time the samples by"
        ));
    }
    let sample_rate = fft::round_1(1000.0 / timedelta);
    #[allow(clippy::cast_possible_truncation)] // `(int)samplerate`
    let freqs = fft::freq_table(i32::try_from(n).unwrap_or(i32::MAX), sample_rate as i32);
    // "0 out all data befor cutoff"
    for sum in &mut avg {
        for (total, freq) in sum.iter_mut().zip(&freqs) {
            if *freq < settings.start_freq {
                *total = 0.0;
                continue;
            }
            break;
        }
    }
    let mut graphs: Vec<Graph> = heads
        .iter()
        .zip(avg.iter().zip(COLOURS_ACC1))
        .map(|(head, (sum, colour))| Graph {
            title: fft::title(head, file_name, sample_rate),
            x_title: X_TITLE,
            y_title: Y_TITLE,
            curves: vec![Curve::line((*head).to_owned(), colour, &freqs, sum)],
            legend: false,
            x_max: None,
            y_max: None,
            y2: false,
        })
        .collect();
    let mut all: Vec<&mut Graph> = graphs.iter_mut().collect();
    set_scale(&mut all);
    Ok(Outcome::Append(graphs))
}

/// Run 16bit Mono Wav: the file's bytes two at a time as little-endian 16-bit samples, from
/// the first byte - the header read as samples too - in windows of `N`; each window's
/// spectrum drawn as "FFT", red with diamonds, and folded into a running mean drawn as "Avg",
/// green on the right-hand axis. After each window the file steps back `N / 2` *bytes* - a
/// quarter of the window, where the C#'s comment says half - and a short last window is never
/// transformed. Only the first `1 << (bins / 2)` bins are candidates for zeroing below Start
/// Freq, as C# reads `1 << bins / 2`.
/// `// C#: Controls/fftui.cs:21-131`
pub fn wav(bytes: &[u8], hz: i32, settings: Settings) -> Result<Outcome, String> {
    let n = size(settings.bins)?;
    let half = n / 2;
    let mut avg = vec![0.0; n];
    let mut buffer = vec![0.0; n];
    let (mut filled, mut samples, mut position) = (0_usize, 0_u32, 0_usize);
    let mut drawn = None;
    let freqs = fft::freq_table(i32::try_from(n).unwrap_or(i32::MAX), hz);
    let cutoff = 1_usize << ((settings.bins / 2) & 31);
    while position < bytes.len() {
        let low = bytes.get(position).copied().unwrap_or(0);
        let high = bytes.get(position + 1).copied().unwrap_or(0);
        position += if position + 1 < bytes.len() { 2 } else { 1 };
        if let Some(slot) = buffer.get_mut(filled) {
            *slot = f64::from(i16::from_le_bytes([low, high]));
        }
        filled += 1;
        if filled < n {
            continue;
        }
        samples += 1;
        let answer = fft::rin(&buffer, settings.in_db).unwrap_or_default();
        let latest = f64::from(samples);
        for (mean, bin) in avg.iter_mut().zip(&answer).take(half) {
            *mean = *mean * (1.0 - 1.0 / latest) + bin * (1.0 / latest);
        }
        for (mean, freq) in avg.iter_mut().zip(&freqs).take(cutoff) {
            if *freq < settings.start_freq {
                *mean = 0.0;
                continue;
            }
            break;
        }
        let mut fft_curve = Curve::line("FFT".to_owned(), COLOURS_ACC1[0], &freqs, &answer);
        fft_curve.symbols = true;
        let mut avg_curve = Curve::line("Avg".to_owned(), COLOURS_ACC1[1], &freqs, &avg);
        avg_curve.y2 = true;
        drawn = Some(Graph {
            title: format!("FFT - {hz}"),
            x_title: X_TITLE,
            y_title: Y_TITLE,
            curves: vec![fft_curve, avg_curve],
            legend: true,
            x_max: None,
            y_max: None,
            y2: true,
        });
        // "50% overlap": `st.Seek(-(1 << bins) / 2, SeekOrigin.Current)`, in bytes.
        position = position.saturating_sub(half);
        filled = 0;
        buffer = vec![0.0; n];
    }
    Ok(Outcome::Wav(drawn))
}

/// What a run needs, sent to the thread that does it.
#[derive(Debug, Clone, PartialEq)]
pub struct Job {
    /// Which button.
    pub run: Run,
    /// The file chosen.
    pub path: PathBuf,
    /// The boxes' values when the button was pressed.
    pub settings: Settings,
    /// Run 16bit Mono Wav's rate.
    pub hz: i32,
}

/// Runs a button's handler on its file: `File.OpenRead`, then the handler.
pub fn perform(job: &Job) -> Result<Outcome, String> {
    let file_name = job
        .path
        .file_name()
        .map(|name| name.to_string_lossy().into_owned())
        .unwrap_or_default();
    let opened = |error: std::io::Error| format!("{}: {error}", job.path.display());
    if job.run == Run::Wav {
        let bytes = mp_os::fs::read(&job.path).map_err(opened)?;
        return wav(&bytes, job.hz, job.settings);
    }
    let log = LogFile::open(&job.path).map_err(opened)?;
    match job.run {
        Run::Isbh => isbh(&log, &file_name, job.settings),
        Run::Imu13 => imu13(&log, &file_name, job.settings),
        Run::AccGyrAll => accgyrall(&log, &file_name, job.settings),
        Run::AccGyr1 | Run::Wav => accgyr1(&log, &file_name, job.settings),
    }
}

// ---------------------------------------------------------------------------------------------
// The window.
// ---------------------------------------------------------------------------------------------

/// A control of the flow panel: the Designer's `zedGraphControl1`, or one a handler made.
#[derive(Debug, Clone, PartialEq)]
enum Slot {
    /// `zedGraphControl1`, which lives as long as the form.
    Designer,
    /// A `NewZedGraph()`.
    Made(Graph),
}

/// Which box a key goes to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NumberBox {
    /// `NUM_bins`.
    Bins,
    /// `NUM_startfreq`.
    StartFreq,
}

impl NumberBox {
    /// The control id.
    #[must_use]
    pub const fn id(self) -> &'static str {
        match self {
            Self::Bins => "fft-NUM_bins",
            Self::StartFreq => "fft-NUM_startfreq",
        }
    }
}

/// A run's file dialog or its rate question.
#[derive(Debug)]
enum Asking {
    /// The `OpenFileDialog`, typed.
    Path(Run, PathBox),
    /// Run 16bit Mono Wav's `InputBox`, after its file.
    Rate(PathBuf, InputBox),
}

/// The `fftui` form.
#[derive(Debug)]
pub struct FftUi {
    /// `NUM_bins`.
    pub bins: NumericUpDown,
    /// `NUM_startfreq`.
    pub start_freq: NumericUpDown,
    /// `chk_mag.Checked`.
    pub magnitude: bool,
    /// `zedGraphControl1`'s pane.
    designer: Graph,
    /// Whether `zedGraphControl1` has been sized: the Designer makes it 779 x 0.
    designer_shown: bool,
    /// `tableLayoutPanel1.Controls`, in order.
    panel: Vec<Slot>,
    /// The box being typed into.
    pub editing: Option<NumberBox>,
    /// The file dialog or the question, while it is up.
    asking: Option<Asking>,
    /// The rate question as its OK closed it, until the holder keeps the answer in
    /// `Settings.Instance`.
    answered: Option<InputBox>,
    /// The run the thread is doing.
    running: Option<(Run, Receiver<Result<Outcome, String>>)>,
    /// The last run and how it ended, for the facts.
    pub last: Option<(Run, String)>,
    /// The pointer over a graph: which, and where as fractions across and down its plot.
    hover: Option<(usize, f64, f64)>,
    /// Where each graph's plot was laid out, for the pointer.
    bounds: Rc<std::cell::RefCell<Vec<Option<Bounds<Pixels>>>>>,
}

impl Default for FftUi {
    fn default() -> Self {
        Self::new()
    }
}

impl FftUi {
    /// `new fftui()`: the Designer's controls, the panel holding only `zedGraphControl1`.
    /// `// C#: Controls/fftui.cs:15-19; Controls/fftui.Designer.cs:29-217`
    #[must_use]
    pub fn new() -> Self {
        let defaults = Settings::default();
        Self {
            bins: NumericUpDown::new((f64::from(defaults.bins), 0.0, NUMERIC_MAX)),
            start_freq: NumericUpDown::new((defaults.start_freq, 0.0, NUMERIC_MAX)),
            magnitude: false,
            designer: Graph::blank(),
            designer_shown: false,
            panel: vec![Slot::Designer],
            editing: None,
            asking: None,
            answered: None,
            running: None,
            last: None,
            hover: None,
            bounds: Rc::default(),
        }
    }

    /// The boxes' values as a handler reads them: `(int)NUM_bins.Value`, validated first.
    pub fn settings(&mut self) -> Settings {
        #[allow(clippy::cast_possible_truncation)] // a box of 0 to 100
        let bins = self.bins.commit() as i32;
        Settings {
            bins,
            start_freq: self.start_freq.commit(),
            in_db: !self.magnitude,
        }
    }

    /// The graphs the panel shows, in order: `zedGraphControl1` only once it has been sized.
    pub fn graphs(&self) -> impl Iterator<Item = &Graph> {
        self.panel.iter().filter_map(|slot| match slot {
            Slot::Designer => self.designer_shown.then_some(&self.designer),
            Slot::Made(graph) => Some(graph),
        })
    }

    /// Whether a handler is running.
    #[must_use]
    pub const fn is_running(&self) -> bool {
        self.running.is_some()
    }

    /// The file dialog's caption and path, while it is up.
    #[must_use]
    pub fn path_prompt(&self) -> Option<(&'static str, &str)> {
        match &self.asking {
            Some(Asking::Path(run, path)) => Some((run.text(), path.field.value())),
            _ => None,
        }
    }

    /// The rate question's text, while it is up.
    #[must_use]
    pub fn rate_prompt(&self) -> Option<&str> {
        match &self.asking {
            Some(Asking::Rate(_, input)) => Some(input.field.value()),
            _ => None,
        }
    }

    /// `chk_mag_CheckedChanged`: `indB = !chk_mag.Checked`.
    /// `// C#: Controls/fftui.cs:973-976`
    pub fn toggle_magnitude(&mut self) {
        self.magnitude = !self.magnitude;
    }

    /// A button: its `OpenFileDialog`.
    pub fn press(&mut self, run: Run) {
        if self.running.is_some() || self.asking.is_some() {
            return;
        }
        self.leave();
        self.asking = Some(Asking::Path(run, PathBox::new("", run.filter())));
    }

    /// A key for the file dialog or the question: Enter is OK, Escape Cancel. What a closing
    /// box asks for is returned: a job to start.
    pub fn prompt_key(&mut self, event: &KeyDownEvent) -> (bool, Result<Option<Job>, String>) {
        let outcome = match &mut self.asking {
            Some(Asking::Path(_, path)) => path.field.key(event),
            Some(Asking::Rate(_, input)) => input.field.key(event),
            None => return (false, Ok(None)),
        };
        match outcome {
            KeyOutcome::Changed => (true, Ok(None)),
            KeyOutcome::Submitted => (true, self.prompt_done(true)),
            KeyOutcome::Cancelled => (true, self.prompt_done(false)),
            KeyOutcome::Ignored => (false, Ok(None)),
        }
    }

    /// Types into the file dialog or the question, as a test does.
    #[cfg(test)]
    pub fn type_prompt(&mut self, text: &str) {
        match &mut self.asking {
            Some(Asking::Path(_, path)) => path.field.set(text),
            Some(Asking::Rate(_, input)) => input.field.set(text),
            None => {}
        }
    }

    /// The dialog or the question closed. OK on a file that exists runs the handler - for Run
    /// 16bit Mono Wav, after its rate question; anything else does nothing, as
    /// `if (!File.Exists(ofd.FileName)) return;` does. The question's Cancel keeps 8000 and
    /// runs: `InputBox.Show`'s result is not looked at. Text that is not a whole number is the
    /// `FormatException` the C# throws, returned as the error.
    /// `// C#: Controls/fftui.cs:25-32, 45-46, 176-181`
    pub fn prompt_done(&mut self, ok: bool) -> Result<Option<Job>, String> {
        match self.asking.take() {
            Some(Asking::Path(run, path)) => {
                let Some(file) = path.chosen().filter(|_| ok) else {
                    return Ok(None);
                };
                if run == Run::Wav {
                    self.asking = Some(Asking::Rate(
                        file,
                        InputBox::new(RATE_TITLE, RATE_PROMPT, &RATE_DEFAULT.to_string()),
                    ));
                    return Ok(None);
                }
                Ok(Some(Job {
                    run,
                    path: file,
                    settings: self.settings(),
                    hz: RATE_DEFAULT,
                }))
            }
            Some(Asking::Rate(file, input)) => {
                let answer = if ok {
                    let answer = input.field.value().trim().to_owned();
                    // `InputBox` keeps the text before the `ref int` overload parses it.
                    self.answered = Some(input);
                    answer
                } else {
                    RATE_DEFAULT.to_string()
                };
                let hz: i32 = answer.parse().map_err(|_| NOT_A_NUMBER.to_owned())?;
                Ok(Some(Job {
                    run: Run::Wav,
                    path: file,
                    settings: self.settings(),
                    hz,
                }))
            }
            None => Ok(None),
        }
    }

    /// The rate question as its OK closed it, once: the answer `InputBox` keeps in
    /// `Settings.Instance` under `InputBoxfftsampleratentersourcefilesamplerate`, which the
    /// window writes - kept before `int.Parse` looks at it, so text that is no number is kept
    /// too.
    /// `// C#: Controls/fftui.cs:45-46; ExtLibs/Controls/InputBox.cs:21-27, 73-84, 178-184`
    pub fn take_answered(&mut self) -> Option<InputBox> {
        self.answered.take()
    }

    /// Starts a job on a thread of its own.
    pub fn start(&mut self, job: Job) {
        let (send, receive) = std::sync::mpsc::channel();
        let run = job.run;
        let spawned = wasm_thread::Builder::new()
            .name("mp-fft".to_owned())
            .spawn(move || {
                // The receiver may be gone with the window; nothing is owed then.
                let _ = send.send(perform(&job));
            });
        if spawned.is_ok() {
            self.running = Some((run, receive));
        } else {
            self.last = Some((run, "no thread".to_owned()));
        }
    }

    /// Once a frame: a finished run's graphs into the panel. The error of a run that threw is
    /// returned, for the status line.
    pub fn poll(&mut self) -> Option<String> {
        let (run, receiver) = self.running.as_ref()?;
        let run = *run;
        let result = match receiver.try_recv() {
            Ok(result) => result,
            Err(TryRecvError::Empty) => return None,
            Err(TryRecvError::Disconnected) => {
                Err("the FFT thread ended without a result".to_owned())
            }
        };
        self.running = None;
        match result {
            Ok(outcome) => {
                self.apply(outcome);
                self.last = Some((run, "done".to_owned()));
                None
            }
            Err(error) => {
                self.last = Some((run, error.clone()));
                Some(error)
            }
        }
    }

    /// What a run leaves in the panel.
    pub fn apply(&mut self, outcome: Outcome) {
        self.hover = None;
        match outcome {
            Outcome::Append(graphs) => self.panel.extend(graphs.into_iter().map(Slot::Made)),
            // `tableLayoutPanel1.Controls.Clear()`: `zedGraphControl1` goes with the rest.
            Outcome::Replace(graphs) => {
                self.panel = graphs.into_iter().map(Slot::Made).collect();
            }
            // `tableLayoutPanel1.Controls.Add(zedGraphControl1)`: a control already there is sent
            // to the back, the end of a flow panel; `SetScale` sizes it.
            Outcome::Wav(graph) => {
                if let Some(graph) = graph {
                    self.designer = graph;
                }
                self.panel.retain(|slot| *slot != Slot::Designer);
                self.panel.push(Slot::Designer);
                self.designer_shown = true;
            }
        }
    }

    /// A box clicked into.
    pub fn begin(&mut self, which: NumberBox) {
        if self.editing != Some(which) {
            self.leave();
        }
        self.editing = Some(which);
    }

    /// The focus left the box: `ValidateEditText`.
    pub fn leave(&mut self) {
        match self.editing.take() {
            Some(NumberBox::Bins) => {
                self.bins.commit();
            }
            Some(NumberBox::StartFreq) => {
                self.start_freq.commit();
            }
            None => {}
        }
    }

    /// The box a key or an arrow acts on.
    pub fn number_mut(&mut self, which: NumberBox) -> &mut NumericUpDown {
        match which {
            NumberBox::Bins => &mut self.bins,
            NumberBox::StartFreq => &mut self.start_freq,
        }
    }

    /// A key in the box being typed into.
    pub fn number_key(&mut self, event: &KeyDownEvent) -> bool {
        match self.editing {
            Some(which) => self.number_mut(which).key(event),
            None => false,
        }
    }

    /// The pointer moved over a graph's plot, or left it.
    pub fn hover(&mut self, at: Option<(usize, f64, f64)>) -> bool {
        let changed = self.hover != at;
        self.hover = at;
        changed
    }

    /// The tooltip for the point nearest the pointer: `zedGraphControl_PointValueEvent`'s
    /// `"{0} hz/{1} rpm"`, within ZedGraph's seven pixels of a point.
    /// `// C#: Controls/fftui.cs:597-600`
    #[must_use]
    pub fn tooltip(&self) -> Option<String> {
        let (index, across, down) = self.hover?;
        let graph = self.graphs().nth(index)?;
        let x = graph.x_range()?;
        let series: Vec<(mp_chart::Series, mp_chart::Range)> = graph
            .curves
            .iter()
            .filter_map(|curve| Some((curve.series(), graph.y_range(curve.y2)?)))
            .collect();
        // A spectrum's bins are in frequency order, so the point search's order is the series'
        // own (`TimeOrder::of` finds nothing to sort).
        let orders: Vec<crate::logbrowse::view::TimeOrder> = series
            .iter()
            .map(|(series, _)| crate::logbrowse::view::TimeOrder::of(series))
            .collect();
        let curves: Vec<crate::logbrowse::view::Curve<'_>> = series
            .iter()
            .zip(orders.iter())
            .map(|((series, range), order)| crate::logbrowse::view::Curve {
                series,
                order,
                range: *range,
            })
            .collect();
        let (_, point) =
            crate::logbrowse::view::nearest_point(&curves, x, (across, down), plot_size())?;
        Some(fft::point_value(point.at))
    }

    /// Where a window position falls on graph `index`'s plot, as it was last laid out.
    fn plot_point(&self, index: usize, x: f32, y: f32) -> Option<(usize, f64, f64)> {
        let bounds = (*self.bounds.borrow().get(index)?)?;
        let (width, height) = (f32::from(bounds.size.width), f32::from(bounds.size.height));
        let across = (x - f32::from(bounds.origin.x)) / width;
        let down = (y - f32::from(bounds.origin.y)) / height;
        ((0.0..=1.0).contains(&across) && (0.0..=1.0).contains(&down) && width > 0.0)
            .then(|| (index, f64::from(across), f64::from(down)))
    }
}

/// A plot's size inside its graph: the graph less its title, axis titles and legend.
const fn plot_size() -> (f32, f32) {
    (
        GRAPH_SIZE.0 - PLOT_LEFT - 8.0,
        GRAPH_SIZE.1 - PLOT_TOP - PLOT_BOTTOM,
    )
}

/// The plot's place in a graph: room for the title above, the y title at the left, the x title
/// and the legend below.
const PLOT_LEFT: f32 = 26.0;
/// Above the plot.
const PLOT_TOP: f32 = 30.0;
/// Below it.
const PLOT_BOTTOM: f32 = 44.0;

/// Facts a UI test asserts on.
pub fn record_facts(window: Option<&FftUi>) {
    use crate::facts::record;
    record("config.fft.window", window.is_some());
    let Some(ui) = window else {
        return;
    };
    record("config.fft.window.bins", ui.bins.field.value());
    record("config.fft.window.startfreq", ui.start_freq.field.value());
    record("config.fft.window.magnitude", ui.magnitude);
    record(
        "config.fft.window.running",
        ui.running.as_ref().map_or("none", |(run, _)| run.key()),
    );
    record(
        "config.fft.window.last",
        ui.last.as_ref().map_or_else(
            || "none".to_owned(),
            |(run, how)| format!("{} {how}", run.key()),
        ),
    );
    record(
        "config.fft.window.prompt",
        ui.path_prompt().map_or("none", |(title, _)| title),
    );
    record(
        "config.fft.window.prompt.text",
        ui.path_prompt().map_or("", |(_, text)| text),
    );
    record("config.fft.window.rate", ui.rate_prompt().unwrap_or("none"));
    let graphs: Vec<&Graph> = ui.graphs().collect();
    record("config.fft.window.graphs", graphs.len());
    for (index, graph) in graphs.iter().enumerate() {
        record(
            format!("config.fft.window.graph.{index}.title"),
            &graph.title,
        );
        record(
            format!("config.fft.window.graph.{index}.curves"),
            graph
                .curves
                .iter()
                .map(|curve| curve.name.as_str())
                .collect::<Vec<_>>()
                .join(","),
        );
        if let Some(curve) = graph.curves.first() {
            let spectrum = fft::Spectrum {
                sample_rate: 0.0,
                n: curve.points.len() * 2,
                slices: 0,
                freqs: curve.points.iter().map(|(x, _)| *x).collect(),
                amplitudes: curve.points.iter().map(|(_, y)| *y).collect(),
            };
            record(
                format!("config.fft.window.graph.{index}.peak"),
                fft::peaks(&spectrum, ui.start_freq.value(), 1)
                    .first()
                    .map_or_else(|| "none".to_owned(), |(hz, _)| fft::point_value(*hz)),
            );
        }
    }
    record(
        "config.fft.window.tooltip",
        ui.tooltip().unwrap_or_else(|| "none".to_owned()),
    );
}

// ---------------------------------------------------------------------------------------------
// Drawing.
// ---------------------------------------------------------------------------------------------

/// How the window's drawing reaches its form.
pub type Access = fn(&mut MissionPlanner) -> Option<&mut FftUi>;

/// The ids of the file dialog; the question and message ids are unused.
const PATH_IDS: BoxIds = BoxIds {
    question: "fft-question",
    yes: "fft-question-yes",
    no: "fft-question-no",
    message: "fft-message",
    ok: "fft-message-ok",
    path: "fft-open",
    path_value: "fft-open-value",
    path_ok: "fft-open-ok",
    path_cancel: "fft-open-cancel",
};

/// A `NumericUpDown` at its place: the text, typed into while it has the focus, and the arrows.
#[allow(clippy::too_many_arguments)]
fn number_box(
    which: NumberBox,
    number: &NumericUpDown,
    editing: bool,
    handle: &FocusHandle,
    enabled: bool,
    access: Access,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let (x, y, width, height) = match which {
        NumberBox::Bins => BINS_AT,
        NumberBox::StartFreq => START_AT,
    };
    let focused = editing && handle.is_focused(window);
    let text = crate::probe::measured(which.id(), div())
        .id(which.id())
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
                if access(this).is_some_and(|ui| ui.number_key(event)) {
                    cx.notify();
                }
            }))
    } else {
        let handle = handle.clone();
        text.cursor_text()
            .on_click(cx.listener(move |this, _event, window, cx| {
                if let Some(ui) = access(this) {
                    ui.begin(which);
                }
                handle.focus(window, cx);
                cx.notify();
            }))
    };
    let arrow = |suffix: &'static str,
                 glyph: &'static str,
                 delta: f64,
                 cx: &mut Context<MissionPlanner>| {
        let id = format!("{}-{suffix}", which.id());
        let base = crate::probe::measured(id.clone(), div())
            .id(SharedString::from(id))
            .flex_1()
            .flex()
            .items_center()
            .justify_center()
            .text_size(px(7.0))
            .child(glyph);
        if enabled {
            base.text_color(rgb(theme::TEXT))
                .cursor_pointer()
                .hover(|style| style.bg(rgb(theme::BORDER)))
                .on_click(cx.listener(move |this, _event, _window, cx| {
                    if let Some(ui) = access(this) {
                        ui.number_mut(which).step(delta);
                    }
                    cx.notify();
                }))
                .into_any_element()
        } else {
            base.text_color(rgb(theme::DIM)).into_any_element()
        }
    };
    let arrows = div()
        .w(px(14.0))
        .h_full()
        .flex()
        .flex_col()
        .border_l_1()
        .border_color(rgb(theme::BORDER))
        .child(arrow("up", "▲", 1.0, cx))
        .child(arrow("down", "▼", -1.0, cx));
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

/// Each curve of a graph as a `LineItem` draws it: the line through its points, reduced to
/// `columns` - one a pixel of the plot - with a hollow diamond at each where it is the FFT's
/// `SymbolType.Diamond`. It was a bar a column, and a curve with fewer points than the plot has
/// pixels a row of dots.
/// `// C#: Controls/fftui.cs:81, 110, 327, 522-524`
fn graph_lines(graph: &Graph, columns: usize) -> Vec<crate::plotline::Line> {
    let Some((from, to)) = graph.x_range() else {
        return Vec::new();
    };
    graph
        .curves
        .iter()
        .filter_map(|curve| {
            let range = graph.y_range(curve.y2)?;
            Some(crate::plotline::Line {
                points: crate::plotline::curve(&curve.series(), range, from, to, columns),
                colour: rgb(curve.colour).into(),
                diamonds: curve.symbols,
            })
        })
        .filter(|line| !line.points.is_empty())
        .collect()
}

/// One graph: its title, the plot of its curves as lines, the axis titles, the legend when it
/// shows, and the tooltip under the pointer.
fn graph_element(
    graph: &Graph,
    index: usize,
    tooltip: Option<&str>,
    bounds: &Rc<std::cell::RefCell<Vec<Option<Bounds<Pixels>>>>>,
    access: Access,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let (width, height) = GRAPH_SIZE;
    let (plot_width, plot_height) = plot_size();
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // a plot's width
    let columns = plot_width.max(1.0) as usize;
    let cell = Rc::clone(bounds);
    let mut plot = div()
        .id(SharedString::from(format!("fft-graph-{index}-plot")))
        .absolute()
        .left(px(PLOT_LEFT))
        .top(px(PLOT_TOP))
        .w(px(plot_width))
        .h(px(plot_height))
        .border_1()
        .border_color(rgb(theme::BORDER))
        .bg(rgb(theme::BG))
        .child(
            canvas(
                move |laid_out, _window, _cx| {
                    let mut all = cell.borrow_mut();
                    if all.len() <= index {
                        all.resize(index + 1, None);
                    }
                    if let Some(slot) = all.get_mut(index) {
                        *slot = Some(laid_out);
                    }
                },
                |_bounds, (), _window, _cx| {},
            )
            .absolute()
            .size_full(),
        )
        .on_mouse_move(
            cx.listener(move |this, event: &gpui::MouseMoveEvent, _window, cx| {
                let (x, y) = (f32::from(event.position.x), f32::from(event.position.y));
                if let Some(ui) = access(this) {
                    let at = ui.plot_point(index, x, y);
                    if ui.hover(at) {
                        cx.notify();
                    }
                }
            }),
        );
    plot = plot.child(crate::plotline::element(graph_lines(graph, columns)));
    if let Some(text) = tooltip {
        plot = plot.child(
            crate::probe::measured(format!("fft-graph-{index}-tooltip"), div())
                .absolute()
                .left(px(4.0))
                .top(px(4.0))
                .px_1()
                .border_1()
                .border_color(rgb(theme::BORDER))
                .bg(rgb(theme::PANEL))
                .text_xs()
                .text_color(rgb(theme::TEXT))
                .child(text.to_owned()),
        );
    }
    let mut legend = div()
        .absolute()
        .left(px(PLOT_LEFT))
        .top(px(PLOT_TOP + plot_height + 16.0))
        .w(px(plot_width))
        .flex()
        .flex_wrap()
        .gap_2();
    if graph.legend {
        for curve in &graph.curves {
            legend = legend.child(
                div()
                    .flex()
                    .items_center()
                    .gap_1()
                    .child(div().w(px(10.0)).h(px(2.0)).bg(rgb(curve.colour)))
                    .child(
                        div()
                            .text_size(px(9.0))
                            .text_color(rgb(theme::TEXT))
                            .child(curve.name.clone()),
                    ),
            );
        }
    }
    crate::probe::measured(format!("fft-graph-{index}"), div())
        .relative()
        .flex_shrink_0()
        .w(px(width))
        .h(px(height))
        .m(px(FLOW_MARGIN))
        .border_1()
        .border_color(rgb(theme::BORDER))
        .bg(rgb(theme::PANEL))
        .child(
            div()
                .absolute()
                .left(px(4.0))
                .top(px(2.0))
                .w(px(width - 8.0))
                .h(px(PLOT_TOP - 2.0))
                .overflow_hidden()
                .text_size(px(10.0))
                .text_color(rgb(theme::TEXT))
                .child(graph.title.clone()),
        )
        .child(plot)
        .child(
            div()
                .absolute()
                .left(px(2.0))
                .top(px(PLOT_TOP))
                .w(px(PLOT_LEFT - 4.0))
                .text_size(px(8.0))
                .text_color(rgb(theme::DIM))
                .child(graph.y_title),
        )
        .child(
            div()
                .absolute()
                .left(px(PLOT_LEFT))
                .top(px(PLOT_TOP + plot_height + 2.0))
                .w(px(plot_width))
                .flex()
                .justify_center()
                .text_size(px(9.0))
                .text_color(rgb(theme::DIM))
                .child(graph.x_title),
        )
        .child(legend)
        .into_any_element()
}

/// The form over the window: its caption with a close box, the flow panel of graphs, the boxes,
/// Magnitude and the five buttons at the Designer's places.
/// `// C#: Controls/fftui.Designer.cs:29-217`
#[allow(clippy::too_many_arguments)]
pub fn dialog(
    ui: &FftUi,
    access: Access,
    close: fn(&mut MissionPlanner),
    number: &FocusHandle,
    prompt: &FocusHandle,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let size = window.viewport_size();
    let idle = !ui.is_running();
    let tooltip = ui.tooltip();
    let hovered = ui.hover.map(|(index, _, _)| index);
    let (px_, py_, pw, ph) = PANEL_AT;
    let mut flow = crate::probe::measured("fft-panel", div())
        .id("fft-panel")
        .absolute()
        .left(px(px_))
        .top(px(py_))
        .w(px(pw))
        .h(px(ph))
        .flex()
        .flex_wrap()
        .content_start()
        .overflow_y_scroll()
        .border_1()
        .border_color(rgb(theme::BORDER))
        .on_hover(cx.listener(move |this, hovered: &bool, _window, cx| {
            if !hovered
                && let Some(ui) = access(this)
                && ui.hover(None)
            {
                cx.notify();
            }
        }));
    for (index, graph) in ui.graphs().enumerate() {
        let tip = (hovered == Some(index))
            .then_some(tooltip.as_deref())
            .flatten();
        flow = flow.child(graph_element(graph, index, tip, &ui.bounds, access, cx));
    }
    let mut client = div()
        .relative()
        .w(px(CLIENT.0))
        .h(px(CLIENT.1))
        .child(flow)
        .child(label(12.0, 510.0, BINS, true))
        .child(number_box(
            NumberBox::Bins,
            &ui.bins,
            ui.editing == Some(NumberBox::Bins),
            number,
            idle,
            access,
            window,
            cx,
        ))
        .child(label(93.0, 510.0, START_FREQ, true))
        .child(number_box(
            NumberBox::StartFreq,
            &ui.start_freq,
            ui.editing == Some(NumberBox::StartFreq),
            number,
            idle,
            access,
            window,
            cx,
        ));
    let mut mag = crate::config::servo_output::Check::default();
    mag.enabled = idle;
    mag.state = if ui.magnitude {
        crate::config::failsafe::CheckState::Checked
    } else {
        crate::config::failsafe::CheckState::Unchecked
    };
    client = client.child(crate::config::servo_output::check_box(
        "fft-chk_mag".to_owned(),
        &mag,
        MAGNITUDE,
        MAG_AT,
        move |this| {
            if let Some(ui) = access(this) {
                ui.toggle_magnitude();
            }
        },
        cx,
    ));
    for run in Run::ALL {
        let prompt = prompt.clone();
        client = client.child(button(
            run.id(),
            run.text(),
            run.at(),
            idle,
            move |this, window, cx| {
                if let Some(ui) = access(this) {
                    ui.press(run);
                    prompt.focus(window, cx);
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
            "fft-close",
            "X",
            theme::TEXT,
            true,
            cx.listener(move |this, _event: &(), _window, cx| {
                close(this);
                cx.notify();
            }),
        ));
    let form = crate::probe::measured("fftui", div())
        .flex()
        .flex_col()
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::ACCENT))
        .rounded_md()
        .child(caption)
        .child(client);
    let over = div()
        .id("fftui-backdrop")
        .w(size.width)
        .h(size.height)
        .flex()
        .items_center()
        .justify_center()
        .occlude()
        .child(form);
    gpui::deferred(
        gpui::anchored()
            .position(gpui::point(px(0.0), px(0.0)))
            .child(over),
    )
    .with_priority(1)
    .into_any_element()
}

/// The file dialog or the rate question over the form, while one is up.
pub fn prompt_box(
    ui: &FftUi,
    access: Access,
    prompt: &FocusHandle,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    match &ui.asking {
        Some(Asking::Path(_, path)) => Some(path_box(
            PATH_IDS,
            path,
            prompt,
            window,
            move |this, event| {
                let (handled, result) =
                    access(this).map_or((false, Ok(None)), |ui| ui.prompt_key(event));
                after_prompt(this, result);
                handled
            },
            move |this, ok| finish_prompt(this, access, ok),
            cx,
        )),
        Some(Asking::Rate(_, input)) => Some(input_box(
            "fft-rate-box",
            input,
            prompt,
            window,
            move |this, event| {
                let (handled, result) =
                    access(this).map_or((false, Ok(None)), |ui| ui.prompt_key(event));
                keep_rate_answer(this, access);
                after_prompt(this, result);
                handled
            },
            move |this| finish_prompt(this, access, true),
            move |this| finish_prompt(this, access, false),
            cx,
        )),
        None => None,
    }
}

/// The dialog or the question closed by a button.
fn finish_prompt(this: &mut MissionPlanner, access: Access, ok: bool) {
    let Some(ui) = access(this) else {
        return;
    };
    let result = ui.prompt_done(ok);
    keep_rate_answer(this, access);
    after_prompt(this, result);
}

/// The rate question's answer kept as `InputBox` keeps it, after a key or a button that may have
/// closed it with OK: the window object holds no settings, the application does.
/// `// C#: Controls/fftui.cs:46; ExtLibs/Controls/InputBox.cs:178-184`
fn keep_rate_answer(this: &mut MissionPlanner, access: Access) {
    if let Some(input) = access(this).and_then(FftUi::take_answered) {
        input.remember(&mut this.persisted);
    }
}

/// What a closed dialog asked for: a job started, or the error the C# throws on the status line.
fn after_prompt(this: &mut MissionPlanner, result: Result<Option<Job>, String>) {
    match result {
        Ok(Some(job)) => start_job(this, job),
        Ok(None) => {}
        Err(error) => this.file_status = Some(error),
    }
}

/// A job started, and the status line told.
fn start_job(this: &mut MissionPlanner, job: Job) {
    let words = format!("FFT: {} - {}", job.run.text(), job.path.display());
    if let Some(ui) = this.extra.fft.window.as_mut() {
        ui.start(job);
        this.file_status = Some(words);
    }
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

    /// A log of old-style `ACC1` and `GYR1` at 1 kHz: a 100 Hz sine on AccZ and 50 Hz on GyrX.
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
            let gyrx = sine(1.0, 50.0, 1000.0, i) as f32;
            let mut acc = time.to_le_bytes().to_vec();
            for value in [0.0_f32, 0.0, accz] {
                acc.extend(value.to_le_bytes());
            }
            bytes.extend(record(200, &acc));
            let mut gyr = time.to_le_bytes().to_vec();
            for value in [gyrx, 0.0, 0.0] {
                gyr.extend(value.to_le_bytes());
            }
            bytes.extend(record(201, &gyr));
        }
        LogFile::from_bytes(bytes)
    }

    /// An `ISBH`/`ISBD` log: one accelerometer batch stream at 2 kHz, a 250 Hz sine on z,
    /// multiplier 100.
    fn batch_log(batches: usize) -> LogFile {
        let mut bytes = Vec::new();
        // ISBH: QHBBHHQf = 8+2+1+1+2+2+8+4.
        bytes.extend(fmt(
            210,
            3 + 28,
            "ISBH",
            "QHBBHHQf",
            "TimeUS,N,type,instance,mul,smp_cnt,SampleUS,smp_rate",
        ));
        // ISBD: QHHaaa = 8+2+2+64*3.
        bytes.extend(fmt(
            211,
            3 + 12 + 192,
            "ISBD",
            "QHHaaa",
            "TimeUS,N,seqno,x,y,z",
        ));
        let mut index = 0_usize;
        for batch in 0..batches {
            let time = 1_000_000_u64 + 100_000 * batch as u64;
            let mut header = time.to_le_bytes().to_vec();
            header.extend(u16::try_from(batch).unwrap_or(0).to_le_bytes());
            header.extend([0_u8, 0]);
            header.extend(100_u16.to_le_bytes());
            header.extend(256_u16.to_le_bytes());
            header.extend(time.to_le_bytes());
            header.extend(2000.0_f32.to_le_bytes());
            bytes.extend(record(210, &header));
            for seq in 0..8_u16 {
                let mut data = (time + u64::from(seq) * 10_000).to_le_bytes().to_vec();
                data.extend(u16::try_from(batch).unwrap_or(0).to_le_bytes());
                data.extend(seq.to_le_bytes());
                let mut z = Vec::new();
                for _ in 0..32 {
                    #[allow(clippy::cast_possible_truncation)]
                    let value = (sine(3.0, 250.0, 2000.0, index) * 100.0).round() as i16;
                    z.extend(value.to_le_bytes());
                    index += 1;
                }
                data.extend(vec![0_u8; 64]);
                data.extend(vec![0_u8; 64]);
                data.extend(z);
                bytes.extend(record(211, &data));
            }
        }
        LogFile::from_bytes(bytes)
    }

    fn graphs(outcome: Outcome) -> Vec<Graph> {
        match outcome {
            Outcome::Append(graphs) | Outcome::Replace(graphs) => graphs,
            Outcome::Wav(graph) => graph.into_iter().collect(),
        }
    }

    /// The highest bin of a curve at or above a frequency.
    fn peak(curve: &Curve, from: f64) -> Option<f64> {
        let spectrum = fft::Spectrum {
            sample_rate: 0.0,
            n: curve.points.len() * 2,
            slices: 0,
            freqs: curve.points.iter().map(|(x, _)| *x).collect(),
            amplitudes: curve.points.iter().map(|(_, y)| *y).collect(),
        };
        fft::peaks(&spectrum, from, 1).first().map(|(hz, _)| *hz)
    }

    /// The Designer's words and places, read from the tree when it is here.
    #[test]
    fn the_text_is_the_designers() {
        let Some(designer) = crate::config_coverage::source::csharp("Controls/fftui.Designer.cs")
        else {
            eprintln!("skipped: MP_SRC does not name a clone of https://github.com/ArduPilot/MissionPlanner");
            return;
        };
        for run in Run::ALL {
            assert!(
                designer.contains(&format!(".Text = \"{}\";", run.text())),
                "{}",
                run.text()
            );
            let (x, y, width, height) = run.at();
            assert!(designer.contains(&format!("Point({x}, {y})")));
            assert!(designer.contains(&format!("Size({width}, {height})")));
        }
        for text in [BINS, START_FREQ, MAGNITUDE, FORM_TEXT] {
            assert!(designer.contains(&format!("Text = \"{text}\";")), "{text}");
        }
        assert!(designer.contains("ClientSize = new System.Drawing.Size(809, 542)"));
        assert!(designer.contains("Location = new System.Drawing.Point(12, 12)"));
        assert!(designer.contains("Size = new System.Drawing.Size(785, 489)"));
        let cs = crate::config_coverage::source::csharp("Controls/fftui.cs").unwrap_or_default();
        assert!(cs.contains(&format!("\"{RATE_TITLE}\", \"{RATE_PROMPT}\"")));
        assert!(cs.contains("\"{0} hz/{1} rpm\""));
        assert!(cs.contains(&format!("ofd.Filter = \"{LOG_FILTER}\"")));
        assert!(cs.contains(&format!("ofd.Filter = \"{WAV_FILTER}\"")));
    }

    /// A new form: 10 and 5 in the boxes, decibels, only the Designer's graph - unsized, so
    /// nothing shows.
    #[test]
    fn a_new_form_has_the_designers_values() {
        let mut ui = FftUi::new();
        assert_eq!(ui.bins.field.value(), "10");
        assert_eq!(ui.start_freq.field.value(), "5");
        assert_eq!(ui.settings(), Settings::default());
        assert_eq!(ui.graphs().count(), 0);
        ui.toggle_magnitude();
        assert!(!ui.settings().in_db);
    }

    /// `1 << bins` as C# shifts, and what is refused.
    #[test]
    fn the_size_is_c_sharps_shift() {
        assert_eq!(size(10), Ok(1024));
        assert_eq!(size(1), Ok(2));
        assert_eq!(size(33), Ok(2), "the count's low five bits");
        assert!(size(0).is_err());
        assert!(size(31).is_err());
        assert!(size(21).is_err());
    }

    /// `ACC1` asks for `ACC` instance 0 as well; `IMU` asks for every instance.
    #[test]
    fn the_wanted_types_are_get_enumerator_types() {
        let wanted = wanted(&["ACC1", "GYR1"]);
        assert_eq!(wanted.get("ACC1"), Some(&vec![String::new()]));
        assert_eq!(wanted.get("ACC"), Some(&vec!["0".to_owned()]));
        assert_eq!(wanted.get("GYR"), Some(&vec!["0".to_owned()]));
        let wanted = super::wanted(&["IMU", "IMU2", "IMU3"]);
        assert_eq!(
            wanted.get("IMU"),
            Some(&vec![String::new(), "1".to_owned(), "2".to_owned()])
        );
    }

    /// The fixture's `IMU` records are three IMUs under one name; the IMU1-3 button reads them
    /// all into series 0 and estimates the rate from the times, which move every third record.
    #[test]
    fn imu13_on_the_fixture_estimates_the_rate_and_averages() {
        let log = LogFile::open(fixture("dataflash.bin"))
            .unwrap_or_else(|_| LogFile::from_bytes(Vec::new()));
        assert!(!log.is_empty(), "testdata/dataflash.bin");
        let settings = Settings {
            bins: 8,
            ..Settings::default()
        };
        let outcome = imu13(&log, "dataflash.bin", settings);
        let graphs = graphs(outcome.unwrap_or(Outcome::Append(Vec::new())));
        assert_eq!(graphs.len(), 6, "the six controls, blanks included");
        let titles: Vec<&str> = graphs.iter().map(|graph| graph.title.as_str()).collect();
        // 1365 records, times 40 ms apart three at a time, from 52.35 s: the average of the gaps
        // starts at the first time and decays by 0.99 a step over 454 steps.
        let mut clock = DataState::default();
        let mut times = Vec::new();
        for (message, _) in records(&log, &["IMU"]) {
            let time = message
                .field("TimeUS")
                .and_then(Value::as_f64)
                .unwrap_or(0.0)
                / 1000.0;
            clock.tick_unguarded(time);
            times.push(time);
        }
        assert_eq!(times.len(), 1365);
        let rate = clock.clock.sample_rate();
        assert!((rate - 22.2).abs() < 1e-9, "{rate}");
        let expected = fft::title("IMU GYR", "dataflash.bin", rate);
        assert_eq!(titles.first(), Some(&expected.as_str()));
        assert_eq!(
            titles.get(1),
            Some(&fft::title("IMU ACC", "dataflash.bin", rate).as_str())
        );
        assert_eq!(titles.get(2..), Some(&[BLANK_TITLE; 4][..]));
        let gyro = graphs.first().map(|graph| &graph.curves);
        let names: Vec<&str> = gyro
            .into_iter()
            .flatten()
            .map(|curve| curve.name.as_str())
            .collect();
        assert_eq!(names, ["IMU GYR x", "IMU GYR y", "IMU GYR z"]);
        // 1365 samples, 256 a slice: five slices, four averaged, 128 bins to half the rate.
        let first = graphs.first().and_then(|graph| graph.curves.first());
        assert_eq!(first.map(|curve| curve.points.len()), Some(128));
        assert_eq!(
            graphs.first().and_then(|graph| graph.x_max),
            Some(rate / 2.0)
        );
        // Bins below 5 Hz are left at zero; the rest are decibels, well below zero.
        let points = first.map(|curve| curve.points.clone()).unwrap_or_default();
        assert!(
            points
                .iter()
                .filter(|(hz, _)| *hz < 5.0)
                .all(|(_, y)| *y == 0.0)
        );
        assert!(
            points
                .iter()
                .filter(|(hz, _)| *hz >= 5.0)
                .all(|(_, y)| *y < 0.0)
        );
        // The peaks: the three IMUs read in turn as one series put their differences at a third
        // of the rate, 7.4 Hz, 7 in whole hertz - on every axis but the gyro's z, whose biases
        // differ least.
        let peaks: Vec<Option<f64>> = graphs
            .iter()
            .flat_map(|graph| graph.curves.iter().map(|curve| peak(curve, 5.0)))
            .collect();
        assert_eq!(
            peaks,
            [
                Some(7.0),
                Some(7.0),
                Some(10.0),
                Some(7.0),
                Some(7.0),
                Some(7.0)
            ]
        );
        // SetScale: the GYR graph's top is the highest GYR top, zero at least.
        assert!(
            graphs
                .first()
                .and_then(|graph| graph.y_max)
                .is_some_and(|max| max >= 0.0)
        );
    }

    /// The same log has no `ACC`/`GYR` messages: six blank graphs; and no `ACC1` for the imu1
    /// button, which the status line says.
    #[test]
    fn a_log_without_the_messages_draws_blanks_or_says_so() {
        let log = LogFile::open(fixture("dataflash.bin"))
            .unwrap_or_else(|_| LogFile::from_bytes(Vec::new()));
        let outcome = accgyrall(&log, "dataflash.bin", Settings::default());
        let graphs = graphs(outcome.unwrap_or(Outcome::Append(Vec::new())));
        assert_eq!(graphs.len(), 6);
        assert!(
            graphs
                .iter()
                .all(|graph| graph.title == BLANK_TITLE && graph.curves.is_empty())
        );
        let refused = accgyr1(&log, "dataflash.bin", Settings::default());
        assert!(refused.is_err_and(|error| error.contains("ACC1")));
        let isbh = isbh(&log, "dataflash.bin", Settings::default());
        assert_eq!(isbh, Ok(Outcome::Replace(Vec::new())));
    }

    /// A 1 kHz ACC1/GYR1 log: the run-all button finds the gyro's 50 Hz and the accelerometer's
    /// 100 Hz, the rate estimated at 1000, the x axis to 500.
    #[test]
    fn accgyrall_finds_the_sines() {
        let log = acc_gyr_log(4096);
        let settings = Settings::default();
        let graphs =
            graphs(accgyrall(&log, "sines.bin", settings).unwrap_or(Outcome::Append(Vec::new())));
        let titles: Vec<&str> = graphs.iter().map(|graph| graph.title.as_str()).collect();
        assert_eq!(
            titles.get(..2),
            Some(
                &[
                    "FFT GYR1 - sines.bin - 1000hz input",
                    "FFT ACC1 - sines.bin - 1000hz input"
                ][..]
            )
        );
        let gyro_x = graphs.first().and_then(|graph| graph.curves.first());
        assert_eq!(gyro_x.map(|curve| curve.name.as_str()), Some("GYR1 x"));
        assert_eq!(gyro_x.and_then(|curve| peak(curve, 5.0)), Some(49.0));
        let acc_z = graphs.get(1).and_then(|graph| graph.curves.get(2));
        assert_eq!(acc_z.map(|curve| curve.name.as_str()), Some("ACC1 z"));
        // Bin 102 of 1024 at 1000 Hz: 99.6, whole hertz truncated to 99.
        assert_eq!(acc_z.and_then(|curve| peak(curve, 5.0)), Some(99.0));
        assert_eq!(graphs.first().and_then(|graph| graph.x_max), Some(500.0));
        assert_eq!(fft::point_value(99.0), "99 hz/5940 rpm");
    }

    /// The imu1 button on the same log: six graphs, one curve each, no legend, averaged over
    /// every full buffer at `2 / N`.
    #[test]
    fn accgyr1_draws_six_axes() {
        let log = acc_gyr_log(3000);
        let settings = Settings {
            in_db: false,
            ..Settings::default()
        };
        let graphs =
            graphs(accgyr1(&log, "sines.bin", settings).unwrap_or(Outcome::Append(Vec::new())));
        let names: Vec<String> = graphs
            .iter()
            .flat_map(|graph| graph.curves.iter().map(|curve| curve.name.clone()))
            .collect();
        assert_eq!(
            names,
            [
                "GYR1-GyrX",
                "GYR1-GyrY",
                "GYR1-GyrZ",
                "ACC1-AccX",
                "ACC1-AccY",
                "ACC1-AccZ"
            ]
        );
        assert!(graphs.iter().all(|graph| !graph.legend));
        assert!(
            graphs
                .first()
                .is_some_and(|graph| graph.title == "FFT GYR1-GyrX - sines.bin - 1000hz input")
        );
        let acc_z = graphs.get(5).and_then(|graph| graph.curves.first());
        assert_eq!(acc_z.and_then(|curve| peak(curve, 5.0)), Some(99.0));
        // Two full buffers of 1024: each spectrum's peak about 2 (a sine of amplitude 2 read by
        // the window's gain, spread over two bins), added in at 2/1024 twice.
        let top = acc_z
            .map(|curve| curve.points.iter().map(|(_, y)| *y).fold(0.0, f64::max))
            .unwrap_or(0.0);
        assert!(top > 0.002 && top < 0.008, "{top}");
    }

    /// IMU Batch Sample on a synthetic batch log: one graph, ACC0 at the header's 2000 Hz, the
    /// 250 Hz sine found, the x axis to 1000.
    #[test]
    fn isbh_reads_the_batches() {
        // 4 batches of 8 x 32: 1024 samples, four slices of 256, three averaged.
        let log = batch_log(4);
        let settings = Settings {
            bins: 8,
            ..Settings::default()
        };
        let graphs =
            graphs(isbh(&log, "batch.bin", settings).unwrap_or(Outcome::Append(Vec::new())));
        assert_eq!(graphs.len(), 1);
        let graph = graphs.first();
        assert_eq!(
            graph.map(|graph| graph.title.as_str()),
            Some("FFT ACC0 - batch.bin - 2000hz input")
        );
        assert_eq!(graph.and_then(|graph| graph.x_max), Some(1000.0));
        let z = graph.and_then(|graph| graph.curves.get(2));
        assert_eq!(z.map(|curve| curve.name.as_str()), Some("ACC0 z"));
        assert_eq!(z.and_then(|curve| peak(curve, 5.0)), Some(250.0));
    }

    /// A header naming a sensor past the twelve is the C#'s index exception, on the status line.
    #[test]
    fn a_sensor_past_the_array_is_an_error() {
        let mut states = vec![DataState::default(); 6];
        assert!(state_at(&mut states, 6).is_err_and(|error| error == OUT_OF_RANGE));
        assert!(state_at(&mut states, -1).is_err());
        assert!(state_at(&mut states, 5).is_ok());
    }

    /// The wav button: the header read as samples, windows stepping back N/2 bytes, the last
    /// window's FFT and the running mean on the right-hand axis.
    #[test]
    fn wav_reads_sixteen_bit_samples_with_the_c_sharps_overlap() {
        // 8000 Hz, a 1000 Hz tone, 2048 samples.
        let mut bytes = Vec::new();
        for i in 0..2048 {
            #[allow(clippy::cast_possible_truncation)]
            let value = sine(10_000.0, 1000.0, 8000.0, i) as i16;
            bytes.extend(value.to_le_bytes());
        }
        let settings = Settings {
            bins: 8,
            ..Settings::default()
        };
        let Ok(Outcome::Wav(Some(graph))) = wav(&bytes, 8000, settings) else {
            panic!("a window was read");
        };
        assert_eq!(graph.title, "FFT - 8000");
        assert!(graph.y2);
        let names: Vec<&str> = graph
            .curves
            .iter()
            .map(|curve| curve.name.as_str())
            .collect();
        assert_eq!(names, ["FFT", "Avg"]);
        assert!(
            graph
                .curves
                .first()
                .is_some_and(|curve| curve.symbols && !curve.y2)
        );
        assert!(graph.curves.get(1).is_some_and(|curve| curve.y2));
        assert_eq!(
            graph.curves.first().and_then(|curve| peak(curve, 5.0)),
            Some(1000.0)
        );
        // 4096 bytes, 512 a window, stepping 384: windows at 0, 384, ... while a whole one fits.
        let short = wav(bytes.get(..100).unwrap_or_default(), 8000, settings);
        assert_eq!(short, Ok(Outcome::Wav(None)));
    }

    /// The Designer's graph shows only once the wav button sizes it, and moves to the end of
    /// the panel; IMU Batch Sample's clear removes it.
    #[test]
    fn the_panel_keeps_the_c_sharps_order() {
        let mut ui = FftUi::new();
        ui.apply(Outcome::Append(vec![Graph::blank(); 6]));
        assert_eq!(ui.graphs().count(), 6);
        let mut wav_graph = Graph::blank();
        wav_graph.title = "FFT - 8000".to_owned();
        ui.apply(Outcome::Wav(Some(wav_graph)));
        assert_eq!(ui.graphs().count(), 7);
        assert_eq!(
            ui.graphs().last().map(|graph| graph.title.as_str()),
            Some("FFT - 8000")
        );
        ui.apply(Outcome::Append(vec![Graph::blank(); 6]));
        assert_eq!(
            ui.graphs().nth(6).map(|graph| graph.title.as_str()),
            Some("FFT - 8000")
        );
        ui.apply(Outcome::Wav(None));
        assert_eq!(
            ui.graphs().last().map(|graph| graph.title.as_str()),
            Some("FFT - 8000")
        );
        ui.apply(Outcome::Replace(Vec::new()));
        assert_eq!(ui.graphs().count(), 0);
    }

    /// `SetScale`: the GYR graphs share their highest top, the ACC ones theirs, from zero.
    #[test]
    fn set_scale_shares_the_tops() {
        let make = |title: &str, top: f64| Graph {
            title: title.to_owned(),
            curves: vec![Curve::line("c".to_owned(), 0, &[0.0, 1.0], &[-50.0, top])],
            ..Graph::blank()
        };
        let mut graphs = [
            make("FFT GYR1", -10.0),
            make("FFT GYR2", -20.0),
            make("FFT ACC1", 3.0),
        ];
        let mut all: Vec<&mut Graph> = graphs.iter_mut().collect();
        set_scale(&mut all);
        let [gyr1, gyr2, acc1] = &graphs;
        assert_eq!(gyr1.y_max, Some(0.0), "the maximum starts at zero");
        assert_eq!(gyr2.y_max, Some(0.0));
        assert!(acc1.y_max.is_some_and(|max| max > 3.0));
    }

    /// The file dialog: a path that is not a file does nothing; the wav button asks its rate
    /// after the file, Cancel keeping 8000 and text that is no number refused.
    #[test]
    fn the_dialogs_run_as_the_c_sharp_does() {
        let mut ui = FftUi::new();
        ui.press(Run::Imu13);
        assert_eq!(ui.path_prompt(), Some((Run::Imu13.text(), "")));
        ui.type_prompt("/no/such/file.bin");
        assert_eq!(ui.prompt_done(true), Ok(None));
        assert!(ui.path_prompt().is_none());

        let file = fixture("dataflash.bin");
        ui.press(Run::Imu13);
        ui.type_prompt(&file.display().to_string());
        let job = ui.prompt_done(true).ok().flatten();
        assert_eq!(job.as_ref().map(|job| job.run), Some(Run::Imu13));
        assert_eq!(job.map(|job| job.settings), Some(Settings::default()));

        ui.press(Run::Wav);
        ui.type_prompt(&file.display().to_string());
        assert_eq!(ui.prompt_done(true), Ok(None));
        assert_eq!(ui.rate_prompt(), Some("8000"));
        let job = ui.prompt_done(false).ok().flatten();
        assert_eq!(job.map(|job| job.hz), Some(RATE_DEFAULT));

        ui.press(Run::Wav);
        ui.type_prompt(&file.display().to_string());
        let _ = ui.prompt_done(true);
        ui.type_prompt("fast");
        assert_eq!(ui.prompt_done(true), Err(NOT_A_NUMBER.to_owned()));
    }

    /// The rate question's OK keeps its text as `InputBox` keeps every titled answer - before
    /// `int.Parse`, so a word is kept too - and Cancel keeps nothing.
    /// `// C#: Controls/fftui.cs:45-46; ExtLibs/Controls/InputBox.cs:21-27, 73-84, 178-184`
    #[test]
    fn the_rate_ok_keeps_the_answer_under_the_input_box_key() {
        let key = crate::config::optional::answers_key(RATE_TITLE, RATE_PROMPT);
        assert_eq!(key, "InputBoxfftsampleratentersourcefilesamplerate");
        assert!(crate::settings::PUBLISHED.contains(&key.as_str()));
        let file = fixture("dataflash.bin");
        let mut ui = FftUi::new();
        let ask = |ui: &mut FftUi| {
            ui.press(Run::Wav);
            ui.type_prompt(&file.display().to_string());
            assert_eq!(ui.prompt_done(true), Ok(None));
            assert!(
                ui.take_answered().is_none(),
                "the file dialog keeps nothing"
            );
        };
        ask(&mut ui);
        let _ = ui.prompt_done(false);
        assert!(ui.take_answered().is_none(), "Cancel keeps nothing");
        ask(&mut ui);
        ui.type_prompt("fast");
        let _ = ui.prompt_done(true);
        let mut settings = crate::settings::Persisted::at(None);
        ui.take_answered()
            .expect("OK's box")
            .remember(&mut settings);
        assert!(ui.take_answered().is_none(), "kept once");
        assert_eq!(settings.get(&key), Some("fast"));
    }

    /// A job on the thread: the fixture's IMUs through the button's handler, and a file that
    /// will not open reported.
    #[test]
    fn a_job_runs_on_its_thread_and_lands_in_the_panel() {
        let mut ui = FftUi::new();
        ui.start(Job {
            run: Run::Imu13,
            path: fixture("dataflash.bin"),
            settings: Settings {
                bins: 8,
                ..Settings::default()
            },
            hz: RATE_DEFAULT,
        });
        assert!(ui.is_running());
        let mut error = None;
        for _ in 0..600 {
            error = ui.poll();
            if !ui.is_running() {
                break;
            }
            wasm_thread::sleep(std::time::Duration::from_millis(10));
        }
        assert_eq!(error, None);
        assert_eq!(ui.graphs().count(), 6);
        assert_eq!(ui.last.as_ref().map(|(_, how)| how.as_str()), Some("done"));

        let failed = perform(&Job {
            run: Run::Isbh,
            path: PathBuf::from("/no/such/log.bin"),
            settings: Settings::default(),
            hz: RATE_DEFAULT,
        });
        assert!(failed.is_err_and(|error| error.starts_with("/no/such/log.bin")));
    }

    /// Each curve is the line through its points - the FFT's with a diamond at each, as
    /// `SymbolType.Diamond` - not a bar a column, which drew a curve with fewer points than the
    /// plot has pixels as a row of dots.
    #[test]
    fn a_graph_draws_each_curve_as_a_line() {
        let freqs: Vec<f64> = (0..50).map(f64::from).collect();
        let values: Vec<f64> = freqs.iter().map(|freq| (freq / 5.0).sin()).collect();
        let mut fft = Curve::line("FFT".to_owned(), 0xff_0000, &freqs, &values);
        fft.symbols = true;
        let average = Curve::line("Avg".to_owned(), 0x00_ff00, &freqs, &values);
        let graph = Graph {
            curves: vec![fft, average],
            ..Graph::blank()
        };
        let lines = graph_lines(&graph, 600);
        assert_eq!(lines.len(), 2);
        assert!(lines[0].diamonds && !lines[1].diamonds);
        assert!(lines.iter().all(|line| line.points.len() == 50));
        assert!(graph_lines(&Graph::blank(), 600).is_empty());
    }
}
