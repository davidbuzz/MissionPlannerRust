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

//! Live Calibration: `MagCalib.DoGUIMagCalib`'s window, `ProgressReporterSphere`, and the loop of
//! `prd_DoWork` that fills it (`MagCalib.cs:136-790`), behind the older Compass page's "Live
//! Calibration" button (`ConfigHWCompass.cs:257-261`).
//!
//! The page (`compass.rs`) shows `Strings.MagCalibMsg`, then opens this window and runs
//! `prd_DoWork`'s writes as one of its jobs: learning off, each compass's offsets zeroed and its
//! ellipsoid made a sphere. Then the loop, once a frame: the stream rates set as the C# sets them
//! (sensors 2, attitude and position 0, `ALL` stopped and `RAW_SENSORS` asked for at 50), each
//! compass's newest reading through the filter, a sphere fit a second past 100 samples with its
//! error in the text, the newest point on each sphere, and the coverage test's aim points on the
//! first; until Done, or until auto accept finds the error under 0.2 with fifteen of the twenty
//! points hit. Then the rates put back and asked for, the checks - the sign of each axis, "run the
//! calibration anyway?" while a point is still wanted, the outliers, ten samples - and the fits,
//! and the window closes, or says the error and waits for Close. The page then saves each answer
//! (`SaveOffsets`, `SaveOffsets2`, `SaveOffsets3`, a box each) and runs `Activate()`. The maths and
//! the words are `mp_calibration::live_magcal`'s and `mp_calibration::magcalib`'s.
//!
//! The window is the `.resx`'s: `ProgressReporterDialogue`'s label, bar, warning picture, Close
//! and Details at their places in `ProgressReporterDialogue.resx`, and the three spheres, the
//! instructions, the two check boxes and Done at theirs in `ProgressReporterSphere.resx`, in its
//! 825 x 446 client area. The colours are this application's, but for the spheres', which are
//! `Sphere.cs`'s.
//!
//! Where this differs from the C#, and why:
//!
//! * **The spheres are drawn flat.** `Sphere` is an OpenTK `GLControl`; this port has no OpenGL.
//!   Each sphere is projected here as `OnPaint` projects it - the same 45-degree perspective from
//!   an eye three times the display's extent away along (1, 1, 1), turned 5 degrees about z each
//!   time a point is added while "Rotate with each data point" is ticked, looking at the origin
//!   with z up, the far plane at 5000 - and painted on a 2D canvas: the six axes, the points in
//!   `OnPaint`'s colours, the aim points white and the newest point red, in its order.
//! * **Samples are read from the vehicle's state, once a frame.** The C# subscribes to every
//!   `RAW_IMU`, `SCALED_IMU2` and `SCALED_IMU3` packet; this application's screens read the link
//!   through its state snapshots, which keep each message's latest magnetometer reading
//!   (`VehicleState::imu`). Each new snapshot's three readings go through the same filter, so a
//!   reading the vehicle sent between two frames is missed and one a snapshot repeats is counted
//!   as a repeat - which the filter's three-per-bucket limit already caps. The first snapshot after
//!   the loop starts is taken as the moment of subscribing and not sampled.
//! * **The fit runs in the frame**, a second counted from the loop's start rather than the wall
//!   clock's `DateTime.Now.Second`, where the C#'s runs on the dialogue's worker thread.
//! * **No beep or speech** at the loop's end: `Console.Beep` and the speech engine are not ported.
//! * The "Details..." box shows the exception's words without a stack trace.
//! * The window's label follows the text every 200 ms, as `timer1` copies it; the bar's marquee
//!   is a block sweeping the bar.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::sync::Arc;
use web_time::{Duration, Instant};

use gpui::{
    AnyElement, Bounds, ContentMask, Context, Hsla, PathBuilder, Pixels, Point, Window, canvas,
    div, point, prelude::*, px, rgb, size,
};
use mp_calibration::CalibrationError;
use mp_calibration::live_magcal::{
    ACCEPT_ERROR, BAD_RAW_VALUES, DRAW_AFTER, Extremes, FIT_AFTER, HIT_TARGET, Live, Source,
    coverage,
};
use mp_calibration::magcalib::Sample;
use mp_vehicle::{StreamRates, VehicleState};

use super::compass::{Autopilot, CheckState, at, button, check_box};
use crate::MissionPlanner;
use crate::telemetry::TelemetryView;
use crate::ui::theme;

/// The window's caption, `ProgressReporterDialogue`'s `Text`.
/// `// C#: ExtLibs/Controls/ProgressReporterDialogue.resx ($this.Text)`
pub const FORM_TEXT: &str = "Progress";
/// The caption once the work has failed. `// C#: ExtLibs/Controls/ProgressReporterDialogue.cs:217`
pub const ERROR_TEXT: &str = "Error";
/// `$this.ClientSize`. `// C#: ExtLibs/Controls/ProgressReporterSphere.resx ($this.ClientSize)`
const CLIENT: (f32, f32) = (825.0, 446.0);
/// `btnCancel.Text`, as `DoGUIMagCalib` sets it. `// C#: MagCalib.cs:154`
pub const DONE: &str = "Done";
/// `btnClose.Text`. `// C#: ExtLibs/Controls/ProgressReporterDialogue.resx (btnClose.Text)`
pub const CLOSE: &str = "Close";
/// `linkLabel1.Text`. `// C#: ExtLibs/Controls/ProgressReporterDialogue.resx (linkLabel1.Text)`
pub const DETAILS: &str = "Details...";
/// What Done writes into the label. `// C#: ExtLibs/Controls/ProgressReporterDialogue.cs:245`
pub const CANCELLING: &str = "Cancelling...";
/// `label1.Text`. `// C#: ExtLibs/Controls/ProgressReporterSphere.resx (label1.Text)`
pub const INSTRUCTIONS: &str = "Aim for the White dots.\nPlease point the autopilot north, and \
rotate around\nthe pitch axis until level.\nthen\nTurn the autopilot 90 degrees, and rotate \
around the\nroll axis until level.\nThis method should hit every white dot.";
/// `CHK_rotate.Text`. `// C#: ExtLibs/Controls/ProgressReporterSphere.resx (CHK_rotate.Text)`
pub const ROTATE: &str = "Rotate with each data point";
/// `chk_auto.Text`. `// C#: ExtLibs/Controls/ProgressReporterSphere.resx (chk_auto.Text)`
pub const AUTO_ACCEPT: &str = "Use Auto Accept";

/// `timer1.Interval`. `// C#: ExtLibs/Controls/ProgressReporterDialogue.designer.cs:84-87`
const TIMER: Duration = Duration::from_millis(200);
/// `ShowDone`'s pause before it closes the window.
/// `// C#: ExtLibs/Controls/ProgressReporterDialogue.cs:190-205`
const CLOSE_DELAY: Duration = Duration::from_millis(100);

/// `MAV_DATA_STREAM.ALL`. `// C#: ExtLibs/Mavlink/Mavlink.cs:3520-3545`
const STREAM_ALL: u8 = 0;
/// `MAV_DATA_STREAM.RAW_SENSORS`.
const STREAM_RAW_SENSORS: u8 = 1;
/// `MAV_DATA_STREAM.RC_CHANNELS`.
const STREAM_RC_CHANNELS: u8 = 3;
/// `MAV_DATA_STREAM.POSITION`.
const STREAM_POSITION: u8 = 6;
/// `MAV_DATA_STREAM.EXTRA1`.
const STREAM_EXTRA1: u8 = 10;
/// `MAV_DATA_STREAM.EXTRA2`.
const STREAM_EXTRA2: u8 = 11;
/// `MAV_DATA_STREAM.EXTRA3`.
const STREAM_EXTRA3: u8 = 12;
/// The rate the loop asks `RAW_SENSORS` for. `// C#: MagCalib.cs:454, 524`
const RAW_SENSORS_HZ: i32 = 50;

/// `Sphere.cs`'s `deg2rad`. `// C#: ExtLibs/Controls/Sphere.cs:14-15`
const DEG2RAD: f64 = 1.0 / (180.0 / std::f64::consts::PI);

/// Where the work is.
#[derive(Debug, Clone, PartialEq)]
pub enum Phase {
    /// `prd_DoWork`'s writes, before the loop.
    Starting,
    /// The loop.
    Sampling,
    /// "run the calibration anyway?" is showing.
    Asking,
    /// `ShowDoneWithError`: the message, and the exception's for Details when one was thrown.
    Failed {
        /// What the label says.
        message: String,
        /// The exception's message, for "Details...".
        details: Option<String>,
    },
    /// `ShowDone`, or a cancel acknowledged: the window closes `delay` after the next frame.
    Closing {
        /// How long after.
        delay: Duration,
        /// When, once a frame has set it.
        at: Option<Instant>,
    },
}

impl Phase {
    /// The word a fact carries.
    #[must_use]
    pub const fn key(&self) -> &'static str {
        match self {
            Self::Starting => "starting",
            Self::Sampling => "sampling",
            Self::Asking => "asking",
            Self::Failed { .. } => "failed",
            Self::Closing { .. } => "closing",
        }
    }
}

/// One `Sphere` control: its points, aim points and centre, and the eye it is seen from.
/// `// C#: ExtLibs/Controls/Sphere.cs:11-80`
#[derive(Debug, Clone, PartialEq)]
pub struct Sphere {
    /// `points`.
    pub points: Vec<Sample>,
    /// `aimpoints`.
    pub aims: Vec<[f64; 3]>,
    /// `CenterPoint`: the running fit's offsets, added to every point drawn.
    pub centre: [f64; 3],
    /// `minx`, `miny`, `minz`, from zero.
    min: [f32; 3],
    /// `maxx`, `maxy`, `maxz`, from zero.
    max: [f32; 3],
    /// `eye`'s direction.
    eye: [f64; 3],
    /// `rotatewithdata`.
    pub rotate: bool,
}

impl Default for Sphere {
    fn default() -> Self {
        Self {
            points: Vec::new(),
            aims: Vec::new(),
            centre: [0.0; 3],
            min: [0.0; 3],
            max: [0.0; 3],
            eye: [1.0, 1.0, 1.0],
            rotate: true,
        }
    }
}

/// An axis drawn: its two ends in pixels, and its colour.
pub type Segment = ((f32, f32), (f32, f32), u32);

/// A sphere projected into its square: the axes, the points with their colours, the aim points
/// and the newest point, in pixels from the square's corner.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Projected {
    /// Each axis from the origin: its two ends and its colour.
    pub axes: Vec<Segment>,
    /// The points and their colours.
    pub points: Vec<(f32, f32, u32)>,
    /// The aim points.
    pub aims: Vec<(f32, f32)>,
    /// The newest point.
    pub last: Option<(f32, f32)>,
}

impl Sphere {
    /// `Clear`: the points go; the extremes stay. `// C#: ExtLibs/Controls/Sphere.cs:32-38`
    pub fn clear(&mut self) {
        self.points.clear();
    }

    /// `AddPoint`, and the repaint it causes: the extremes widened, the point kept, and the view
    /// turned 5 degrees about z while `rotatewithdata`. `// C#: ExtLibs/Controls/Sphere.cs:61-80,
    /// 125-130, 150-157`
    pub fn add(&mut self, point: Sample) {
        for ((value, min), max) in point.iter().zip(&mut self.min).zip(&mut self.max) {
            *min = min.min(*value);
            *max = max.max(*value);
        }
        self.points.push(point);
        if self.rotate {
            let (sin, cos) = (5.0 * DEG2RAD).sin_cos();
            let [x, y, z] = self.eye;
            self.eye = [x * cos - y * sin, x * sin + y * cos, z];
        }
    }

    /// `OnPaint`'s projection into a `side`-pixel square. `// C#: ExtLibs/Controls/Sphere.cs:115-253`
    #[must_use]
    pub fn project(&self, side: f32) -> Projected {
        let [minx, miny, minz] = self.min;
        let [maxx, maxy, maxz] = self.max;
        let mut max = f64::from(
            ((maxx - minx) / 2.0)
                .max((maxy - miny) / 2.0)
                .max((maxz - minz) / 2.0),
        );
        if max < 300.0 {
            max = 400.0;
        }
        max *= 1.3;
        let eyedist = max * 3.0;
        let length = |v: [f64; 3]| (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
        let unit = |v: [f64; 3]| {
            let l = length(v);
            [v[0] / l, v[1] / l, v[2] / l]
        };
        let cross = |a: [f64; 3], b: [f64; 3]| {
            [
                a[1] * b[2] - a[2] * b[1],
                a[2] * b[0] - a[0] * b[2],
                a[0] * b[1] - a[1] * b[0],
            ]
        };
        let dot = |a: [f64; 3], b: [f64; 3]| a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
        let direction = if length(self.eye).is_finite() && length(self.eye) > 0.0 {
            unit(self.eye)
        } else {
            unit([1.0, 1.0, 1.0])
        };
        let eye = direction.map(|v| v * eyedist);
        // `Matrix4.LookAt(eye, 0, up z)`: z back towards the eye, x to the right, y up.
        let back = unit(eye);
        let right = unit(cross([0.0, 0.0, 1.0], back));
        let up = cross(back, right);
        // `CreatePerspectiveFieldOfView(45 degrees, 1, 0.00001, 5000)`.
        let focal = 1.0 / (22.5 * DEG2RAD).tan();
        let (near, far) = (0.000_01, 5000.0);
        let side64 = f64::from(side);
        let project = |p: [f64; 3]| -> Option<(f32, f32)> {
            let v = [p[0] - eye[0], p[1] - eye[1], p[2] - eye[2]];
            let depth = -dot(v, back);
            if !(near..=far).contains(&depth) {
                return None;
            }
            let x = focal * dot(v, right) / depth;
            let y = focal * dot(v, up) / depth;
            #[allow(clippy::cast_possible_truncation)] // pixels
            Some((
                ((x + 1.0) / 2.0 * side64) as f32,
                ((1.0 - y) / 2.0 * side64) as f32,
            ))
        };
        let centred = |p: Sample| {
            [
                f64::from(p[0]) + self.centre[0],
                f64::from(p[1]) + self.centre[1],
                f64::from(p[2]) + self.centre[2],
            ]
        };
        let mut out = Projected::default();
        let origin = project([0.0; 3]);
        for (end, colour) in [
            ([0.0, 0.0, max], 0x00_00_ff),
            ([0.0, max, 0.0], 0x00_ff_00),
            ([max, 0.0, 0.0], 0xff_00_00),
            ([0.0, 0.0, -max], 0xff_ff_00),
            ([0.0, -max, 0.0], 0xff_00_ff),
            ([-max, 0.0, 0.0], 0x00_ff_ff),
        ] {
            if let (Some(from), Some(to)) = (origin, project(end)) {
                out.axes.push((from, to, colour));
            }
        }
        let range = [maxx - minx, maxy - miny, maxz - minz];
        for item in &self.points {
            let colour = (channel(item[0], range[0]) << 16)
                | (channel(item[1], range[1]) << 8)
                | channel(item[2], range[2]);
            if let Some((x, y)) = project(centred(*item)) {
                out.points.push((x, y, colour));
            }
        }
        for aim in &self.aims {
            let p = [
                aim[0] + self.centre[0],
                aim[1] + self.centre[1],
                aim[2] + self.centre[2],
            ];
            out.aims.extend(project(p));
        }
        out.last = self.points.last().and_then(|p| project(centred(*p)));
        out
    }
}

/// A point's colour on one axis: `(int)Math.Abs(value / range * 254) & 0xff`, where a value the
/// cast cannot hold - infinite, not a number, past an `int` - is `int.MinValue` on .NET, and so 0.
/// `// C#: ExtLibs/Controls/Sphere.cs:209-215`
fn channel(value: f32, range: f32) -> u32 {
    let v = (value / range * 254.0).abs();
    if !v.is_finite() || v >= 2_147_483_648.0 {
        return 0;
    }
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // in range, and positive
    let whole = v as u32;
    whole & 0xff
}

/// `prd_DoWork`'s locals for one run of the loop. `// C#: MagCalib.cs:382-472`
#[derive(Debug, Clone)]
struct Pass {
    have2: bool,
    have3: bool,
    extremes: Extremes,
    /// The rates to put back.
    backup: StreamRates,
    started: Instant,
    /// `lastlsq`, `lastlsq2`, `lastlsq3`: the second each fit last ran in.
    last_fit: [Option<u64>; 3],
    /// `lastcount`.
    last_count: usize,
    /// `centre`.
    centre: [f64; 3],
    /// `extramsg`.
    extramsg: String,
    /// `pointshit`, the last coverage test's.
    hits: usize,
    /// The snapshot last sampled.
    last_state: Option<Arc<VehicleState>>,
    /// Whether a snapshot has been seen since the loop began.
    primed: bool,
}

/// The window: `ProgressReporterSphere`, with `ProgressReporterDialogue`'s state.
#[derive(Debug, Clone)]
struct Form {
    phase: Phase,
    /// `doWorkArgs.CancelRequested`: Done pressed, not yet seen by the loop.
    cancel_requested: bool,
    /// `btnCancel.Visible`.
    done_visible: bool,
    /// `_status`, what the work last said.
    status: String,
    /// `lblProgressMessage.Text`.
    label: String,
    /// When `timer1` next copies the status to the label.
    next_timer: Option<Instant>,
    /// `sphere1` to `3`.
    spheres: [Sphere; 3],
    /// `CHK_rotate`.
    rotate: bool,
    /// `chk_auto`, `autoaccept`.
    auto_accept: bool,
    /// `progressBar1` at 100, as `ShowDone` leaves it.
    full: bool,
    pass: Option<Pass>,
}

impl Form {
    fn new() -> Self {
        Self {
            phase: Phase::Starting,
            cancel_requested: false,
            done_visible: true,
            status: String::new(),
            label: String::new(),
            next_timer: None,
            spheres: Default::default(),
            rotate: true,
            auto_accept: true,
            full: false,
            pass: None,
        }
    }

    /// `ShowDoneWithError`: the message in the label, Done hidden, Close shown.
    /// `// C#: ExtLibs/Controls/ProgressReporterDialogue.cs:207-235`
    fn fail(&mut self, message: String, details: Option<String>) {
        self.label.clone_from(&message);
        self.done_visible = false;
        self.phase = Phase::Failed { message, details };
    }
}

/// Live Calibration: `MagCalib`'s state, which lives as long as the application, and its window
/// while it shows.
#[derive(Debug, Clone, Default)]
pub struct LiveMagCal {
    live: Live,
    form: Option<Form>,
    /// The window closed after a run: the answers are to be saved.
    closed: bool,
}

impl LiveMagCal {
    /// `DoGUIMagCalib`'s resets, before its box. `// C#: MagCalib.cs:138-146`
    pub fn begin(&mut self) {
        self.live.begin();
    }

    /// The window opens: `new ProgressReporterSphere()`, Done's text set, the work begun.
    /// `// C#: MagCalib.cs:151-159`
    pub fn open(&mut self) {
        self.form = Some(Form::new());
        self.closed = false;
    }

    /// Whether the window is showing.
    #[must_use]
    pub const fn is_open(&self) -> bool {
        self.form.is_some()
    }

    /// `MagCalib`'s statics.
    #[cfg(test)]
    #[must_use]
    pub const fn live(&self) -> &Live {
        &self.live
    }

    /// Where the work is, `None` with the window closed.
    #[must_use]
    pub fn phase(&self) -> Option<&Phase> {
        self.form.as_ref().map(|form| &form.phase)
    }

    /// What the window's label says.
    #[cfg(test)]
    #[must_use]
    pub fn label(&self) -> Option<&str> {
        self.form.as_ref().map(|form| form.label.as_str())
    }

    /// The window's three spheres.
    #[cfg(test)]
    #[must_use]
    pub fn spheres(&self) -> Option<&[Sphere; 3]> {
        self.form.as_ref().map(|form| &form.spheres)
    }

    /// The loop begins, after `prd_DoWork`'s writes: the rates backed up and set, `ALL` stopped
    /// and `RAW_SENSORS` asked for at 50, the spheres cleared.
    /// `// C#: MagCalib.cs:437-472`
    pub fn start<A: Autopilot>(&mut self, autopilot: &mut A, have2: bool, have3: bool, now: Instant) {
        let Some(form) = self.form.as_mut() else {
            return;
        };
        if form.phase != Phase::Starting {
            return;
        }
        let backup = autopilot.rates();
        autopilot.set_rates(StreamRates {
            sensors: 2,
            attitude: 0,
            position: 0,
            ..backup
        });
        autopilot.request_stream(STREAM_ALL, 0);
        autopilot.request_stream(STREAM_RAW_SENSORS, RAW_SENSORS_HZ);
        for sphere in &mut form.spheres {
            sphere.clear();
        }
        form.pass = Some(Pass {
            have2,
            have3,
            extremes: Extremes::default(),
            backup,
            started: now,
            last_fit: [None; 3],
            last_count: 0,
            centre: [0.0; 3],
            extramsg: String::new(),
            hits: 0,
            last_state: None,
            primed: false,
        });
        form.phase = Phase::Sampling;
    }

    /// An exception out of `prd_DoWork`: "There was an unexpected error (...)", with Details.
    /// `// C#: ExtLibs/Controls/ProgressReporterDialogue.cs:120-133, 209`
    pub fn fail_exception(&mut self, message: &str) {
        if let Some(form) = self.form.as_mut() {
            form.fail(
                format!("There was an unexpected error ({message})"),
                Some(message.to_owned()),
            );
        }
    }

    /// The exception's words, while the window says one.
    #[must_use]
    pub fn details(&self) -> Option<&str> {
        match self.phase()? {
            Phase::Failed { details, .. } => details.as_deref(),
            _ => None,
        }
    }

    /// Done, `btnCancel_Click`: hidden, "Cancelling...", and the cancel asked for.
    /// `// C#: ExtLibs/Controls/ProgressReporterDialogue.cs:237-250`
    pub fn click_done(&mut self) {
        let Some(form) = self.form.as_mut() else {
            return;
        };
        if !form.done_visible || !matches!(form.phase, Phase::Starting | Phase::Sampling) {
            return;
        }
        form.done_visible = false;
        CANCELLING.clone_into(&mut form.label);
        form.cancel_requested = true;
    }

    /// Close, after an error: the window goes. `// C#: ExtLibs/Controls/ProgressReporterDialogue.cs:253-257`
    pub fn click_close(&mut self) {
        if matches!(self.phase(), Some(Phase::Failed { .. })) {
            self.form = None;
            self.closed = true;
        }
    }

    /// "Rotate with each data point": the first two spheres only, as the handler sets them.
    /// `// C#: ExtLibs/Controls/ProgressReporterSphere.cs:111-115`
    pub fn toggle_rotate(&mut self) {
        if let Some(form) = self.form.as_mut() {
            form.rotate = !form.rotate;
            let [first, second, _] = &mut form.spheres;
            first.rotate = form.rotate;
            second.rotate = form.rotate;
        }
    }

    /// "Use Auto Accept". `// C#: ExtLibs/Controls/ProgressReporterSphere.cs:117-120`
    pub fn toggle_auto(&mut self) {
        if let Some(form) = self.form.as_mut() {
            form.auto_accept = !form.auto_accept;
        }
    }

    /// The answer to "run the calibration anyway?": No forgets every answer and closes the
    /// window, as a cancel acknowledged does; Yes goes on to the fit.
    /// `// C#: MagCalib.cs:737-785; ExtLibs/Controls/ProgressReporterDialogue.cs:157-163`
    pub fn missing_answered(&mut self, yes: bool, parameters: &[(String, f64)]) {
        let Self { live, form, .. } = self;
        let Some(form) = form.as_mut() else {
            return;
        };
        if form.phase != Phase::Asking {
            return;
        }
        if yes {
            finish(live, form, parameters);
        } else {
            live.forget_all();
            form.phase = Phase::Closing {
                delay: Duration::ZERO,
                at: None,
            };
        }
    }

    /// Once, after the window has closed from a run: the answers `SaveOffsets` is given.
    pub fn take_closed(&mut self) -> Option<[Option<Vec<f64>>; 3]> {
        if !self.closed {
            return None;
        }
        self.closed = false;
        Some(self.live.answers.clone())
    }

    /// Once a frame: `timer1`, the window's closing, and one pass of the loop. True when the loop
    /// has ended wanting "run the calibration anyway?" asked.
    /// `// C#: MagCalib.cs:470-700; ExtLibs/Controls/ProgressReporterDialogue.cs:303-330`
    pub fn tick<A: Autopilot>(
        &mut self,
        autopilot: &mut A,
        view: &TelemetryView,
        now: Instant,
    ) -> bool {
        let Self { live, form, closed } = self;
        let Some(open) = form.as_mut() else {
            return false;
        };
        if matches!(open.phase, Phase::Starting | Phase::Sampling)
            && open.next_timer.is_none_or(|next| now >= next)
        {
            open.label.clone_from(&open.status);
            open.next_timer = Some(now + TIMER);
        }
        match &mut open.phase {
            Phase::Closing { delay, at } => {
                let at = *at.get_or_insert(now + *delay);
                if now >= at {
                    *form = None;
                    *closed = true;
                }
                return false;
            }
            Phase::Sampling => {}
            _ => return false,
        }
        if let Some(pass) = open.pass.as_mut() {
            sample(live, pass, view);
        }
        iterate(live, open, autopilot, &view.parameters, now)
    }
}

/// The subscriptions: each compass's reading from a snapshot not seen before.
/// `// C#: MagCalib.cs:189-292, 457-461`
fn sample(live: &mut Live, pass: &mut Pass, view: &TelemetryView) {
    let Some(state) = view.state.as_ref() else {
        return;
    };
    if pass
        .last_state
        .as_ref()
        .is_some_and(|last| Arc::ptr_eq(last, state))
    {
        return;
    }
    pass.last_state = Some(Arc::clone(state));
    if !std::mem::replace(&mut pass.primed, true) {
        return;
    }
    let sources = [Source::RawImu, Source::ScaledImu2, Source::ScaledImu3];
    for (source, imu) in sources.into_iter().zip(state.imu.iter()) {
        live.sample(source, counts(imu.mag));
    }
}

/// A reading back in the `short`s its message carried.
fn counts(mag: [f32; 3]) -> [i16; 3] {
    #[allow(clippy::cast_possible_truncation)] // each came from an i16
    mag.map(|value| value.round() as i16)
}

/// One pass of `prd_DoWork`'s loop. True when it ended wanting the question asked.
/// `// C#: MagCalib.cs:470-700`
fn iterate<A: Autopilot>(
    live: &mut Live,
    form: &mut Form,
    autopilot: &mut A,
    parameters: &[(String, f64)],
    now: Instant,
) -> bool {
    let Some(pass) = form.pass.as_mut() else {
        return false;
    };
    // `UpdateProgressAndStatus` is ignored while a cancel is asked and not acknowledged.
    if !form.cancel_requested {
        form.status = live.status(pass.have2, pass.have3, &pass.extramsg);
    }
    if form.cancel_requested {
        form.cancel_requested = false;
        return end_loop(live, form, autopilot, parameters);
    }
    let Some(last) = live.samples[0].last() else {
        return false;
    };
    pass.extremes.take(last);
    let second = now.saturating_duration_since(pass.started).as_secs();
    let lens = live.samples.each_ref().map(|samples| samples.len());
    let newest = live.samples.each_ref().map(|samples| samples.last());
    for (index, ((len, last_fit), sphere)) in lens
        .iter()
        .zip(&mut pass.last_fit)
        .zip(&mut form.spheres)
        .enumerate()
    {
        if *len > FIT_AFTER && *last_fit != Some(second) {
            if index == 0 {
                autopilot.request_stream(STREAM_RAW_SENSORS, RAW_SENSORS_HZ);
            }
            *last_fit = Some(second);
            if let Some(centre) = live.running_fit(index) {
                if index == 0 {
                    pass.centre = centre;
                }
                sphere.centre = centre;
            }
        }
    }
    let [count1, count2, count3] = lens;
    if pass.last_count == count1 {
        return false;
    }
    pass.last_count = count1;
    let [sphere1, sphere2, sphere3] = &mut form.spheres;
    sphere1.add(last);
    sphere1.aims.clear();
    for ((count, reading), sphere) in [count2, count3]
        .into_iter()
        .zip(newest.into_iter().skip(1))
        .zip([sphere2, sphere3])
    {
        if count > DRAW_AFTER
            && let Some(reading) = reading
        {
            sphere.add(reading);
            sphere.aims.clear();
        }
    }
    let found = coverage(live.samples[0].data(), pass.centre);
    pass.hits = found.hits;
    form.spheres[0].aims = found.aims;
    pass.extramsg = found.message;
    if live.errors[0] < ACCEPT_ERROR && pass.hits > HIT_TARGET && form.auto_accept {
        pass.extramsg.clear();
        return end_loop(live, form, autopilot, parameters);
    }
    false
}

/// After the loop: the rates put back and asked for again, then the sign check, the question
/// while a point is still wanted, or the fit. True when the question is to be asked.
/// `// C#: MagCalib.cs:695-749`
fn end_loop<A: Autopilot>(
    live: &mut Live,
    form: &mut Form,
    autopilot: &mut A,
    parameters: &[(String, f64)],
) -> bool {
    let Some(pass) = form.pass.as_ref() else {
        return false;
    };
    let backup = pass.backup;
    autopilot.set_rates(backup);
    for (stream, hz) in [
        (STREAM_RAW_SENSORS, backup.sensors),
        (STREAM_POSITION, backup.position),
        (STREAM_EXTRA1, backup.attitude),
        (STREAM_EXTRA2, backup.attitude),
        (STREAM_EXTRA3, backup.sensors),
        (STREAM_RAW_SENSORS, backup.sensors),
        (STREAM_RC_CHANNELS, backup.rc),
    ] {
        autopilot.request_stream(stream, hz);
    }
    if pass.extremes.bad() {
        live.forget_bad();
        form.fail(BAD_RAW_VALUES.to_owned(), None);
        return false;
    }
    if !pass.extramsg.is_empty() {
        form.phase = Phase::Asking;
        return true;
    }
    finish(live, form, parameters);
    false
}

/// The outliers, the count and the fits - the ellipsoid when the vehicle has `COMPASS_DIA_X` -
/// then `timer1` once more and `ShowDone`, or the error. `// C#: MagCalib.cs:751-785;
/// ExtLibs/Controls/ProgressReporterDialogue.cs:139-176`
fn finish(live: &mut Live, form: &mut Form, parameters: &[(String, f64)]) {
    let (have2, have3) = form
        .pass
        .as_ref()
        .map_or((false, false), |pass| (pass.have2, pass.have3));
    let ellipsoid = parameters.iter().any(|(name, _)| name == "COMPASS_DIA_X");
    form.label.clone_from(&form.status);
    match live.finish(have2, have3, ellipsoid) {
        Ok(()) => {
            form.done_visible = false;
            form.full = true;
            form.phase = Phase::Closing {
                delay: CLOSE_DELAY,
                at: None,
            };
        }
        // `doWorkArgs.ErrorMessage`: no exception, no Details.
        Err(error @ CalibrationError::NotEnoughData { .. }) => form.fail(error.to_string(), None),
        Err(error) => {
            let message = error.to_string();
            form.fail(
                format!("There was an unexpected error ({message})"),
                Some(message),
            );
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Facts.
// ---------------------------------------------------------------------------------------------

/// Facts a UI test asserts on: whether the window shows and where its work is, each compass's
/// samples, error and answer, the label, the coverage test's hits and the spheres' points.
pub fn record_facts(live: &LiveMagCal) {
    use crate::facts::record;
    let key = |name: &str| format!("config.compass.livecal.{name}");
    record(key("open"), live.is_open());
    record(key("phase"), live.phase().map_or("closed", Phase::key));
    for (index, ((samples, error), answer)) in live
        .live
        .samples
        .iter()
        .zip(live.live.errors)
        .zip(&live.live.answers)
        .enumerate()
    {
        let n = index + 1;
        record(key(&format!("samples{n}")), samples.len());
        record(key(&format!("error{n}")), error);
        record(
            key(&format!("offsets{n}")),
            answer.as_ref().map_or_else(
                || "none".to_owned(),
                |ofs| {
                    ofs.iter()
                        .take(3)
                        .map(|value| format!("{value:.0}"))
                        .collect::<Vec<_>>()
                        .join(" ")
                },
            ),
        );
    }
    let form = live.form.as_ref();
    record(
        key("status"),
        form.map_or_else(String::new, |form| form.label.replace('\n', " ")),
    );
    record(
        key("hits"),
        form.and_then(|form| form.pass.as_ref())
            .map_or(0, |pass| pass.hits),
    );
    for index in 0..3 {
        record(
            key(&format!("points{}", index + 1)),
            form.and_then(|form| form.spheres.get(index))
                .map_or(0, |sphere| sphere.points.len()),
        );
    }
    record(
        key("aims"),
        form.and_then(|form| form.spheres.first())
            .map_or(0, |sphere| sphere.aims.len()),
    );
    record(key("done"), form.is_some_and(|form| form.done_visible));
}

// ---------------------------------------------------------------------------------------------
// Drawing.
// ---------------------------------------------------------------------------------------------

/// `Sphere`'s size on the form. `// C#: ExtLibs/Controls/ProgressReporterSphere.resx (sphere1.Size)`
const SPHERE_SIDE: f32 = 263.0;

/// A sphere's canvas: black, the axes, the points, the aim points, the newest point.
/// `// C#: ExtLibs/Controls/Sphere.cs:115-253`
fn sphere_canvas(id: &'static str, sphere: &Sphere) -> AnyElement {
    let projected = sphere.project(SPHERE_SIDE);
    let body = canvas(
        |_bounds, _window, _cx| (),
        move |bounds, (), window, _cx| {
            window.paint_quad(gpui::fill(bounds, rgb(0x00_00_00)));
            window.with_content_mask(Some(ContentMask { bounds }), |window| {
                paint_sphere(&projected, bounds.origin, window);
            });
        },
    )
    .size_full();
    crate::probe::measured(id, div())
        .size(px(SPHERE_SIDE))
        .child(body)
        .into_any_element()
}

/// Paints what [`Sphere::project`] made, from `origin`.
fn paint_sphere(projected: &Projected, origin: Point<Pixels>, window: &mut Window) {
    let at = |(x, y): (f32, f32)| point(origin.x + px(x), origin.y + px(y));
    for (from, to, colour) in &projected.axes {
        let mut builder = PathBuilder::stroke(px(1.0));
        builder.move_to(at(*from));
        builder.line_to(at(*to));
        if let Ok(path) = builder.build() {
            window.paint_path(path, Hsla::from(rgb(*colour)));
        }
    }
    let square = |window: &mut Window, (x, y): (f32, f32), side: f32, colour: u32| {
        window.paint_quad(gpui::fill(
            Bounds {
                origin: at((x - side / 2.0, y - side / 2.0)),
                size: size(px(side), px(side)),
            },
            rgb(colour),
        ));
    };
    // `GL.PointSize(8)` for the points and the aim points, 12 for the newest.
    for (x, y, colour) in &projected.points {
        square(window, (*x, *y), 8.0, *colour);
    }
    for aim in &projected.aims {
        square(window, *aim, 8.0, 0xff_ff_ff);
    }
    if let Some(last) = projected.last {
        square(window, last, 12.0, 0xff_00_00);
    }
}

/// `progressBar1`: full after `ShowDone`, otherwise the marquee the work's `-1` asks for.
/// `// C#: ExtLibs/Controls/ProgressReporterDialogue.cs:303-330`
fn progress_bar(full: bool) -> AnyElement {
    let bar = at(11.0, 90.0, 277.0, 13.0)
        .border_1()
        .border_color(rgb(theme::BORDER))
        .bg(rgb(theme::BG));
    let fill = if full {
        div().h_full().w_full().bg(rgb(theme::OK))
    } else {
        let millis = web_time::SystemTime::now()
            .duration_since(web_time::UNIX_EPOCH)
            .map_or(0, |since| since.as_millis() % 2000);
        #[allow(clippy::cast_precision_loss)] // under 2000
        let left = millis as f32 / 2000.0 * (277.0 - 60.0);
        div()
            .absolute()
            .top_0()
            .left(px(left))
            .h_full()
            .w(px(60.0))
            .bg(rgb(theme::OK))
    };
    bar.child(fill).into_any_element()
}

/// The window, while it shows: the caption, then the client area as the `.resx` places it.
pub fn window(
    live: &LiveMagCal,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    let form = live.form.as_ref()?;
    let failed = matches!(form.phase, Phase::Failed { .. });
    let text = |x: f32, y: f32, width: f32, height: f32, words: &str| {
        at(x, y, width, height)
            .flex()
            .flex_col()
            .justify_center()
            .text_xs()
            .text_color(rgb(theme::TEXT))
            .children(words.split('\n').map(|line| line.to_owned()))
    };
    let mut client = div()
        .relative()
        .w(px(CLIENT.0))
        .h(px(CLIENT.1))
        // `lblProgressMessage`, `MiddleLeft`; moved to 65 beside the warning on an error.
        .child(crate::probe::measured(
            "magcal-status",
            text(if failed { 65.0 } else { 13.0 }, 13.0, 275.0, 74.0, &form.label),
        ))
        // `label1`, `AutoSize`, its lines as the `.resx` breaks them.
        .child(
            at(282.0, 8.0, 254.0, 104.0)
                .flex()
                .flex_col()
                .text_xs()
                .text_color(rgb(theme::TEXT))
                .children(INSTRUCTIONS.split('\n').map(str::to_owned)),
        );
    if failed {
        // `imgWarning`, `iconWarning48`.
        client = client.child(
            at(13.0, 22.0, 48.0, 48.0)
                .flex()
                .items_center()
                .justify_center()
                .text_3xl()
                .text_color(rgb(theme::WARN))
                .child("\u{26a0}"),
        );
    } else {
        client = client.child(progress_bar(form.full));
    }
    for ((sphere, x), id) in form
        .spheres
        .iter()
        .zip([11.0, 280.0, 549.0])
        .zip(["magcal-sphere1", "magcal-sphere2", "magcal-sphere3"])
    {
        client = client.child(at(x, 141.0, SPHERE_SIDE, SPHERE_SIDE).child(sphere_canvas(id, sphere)));
    }
    let checked = |on: bool| {
        if on {
            CheckState::Checked
        } else {
            CheckState::Unchecked
        }
    };
    client = client
        .child(check_box(
            (13.0, 411.0, 157.0, 17.0),
            "magcal-rotate".into(),
            ROTATE,
            (checked(form.rotate), true),
            cx.listener(|this, _event, _window, cx| {
                this.compass.live_mut().toggle_rotate();
                cx.notify();
            }),
        ))
        .child(check_box(
            (176.0, 410.0, 107.0, 17.0),
            "magcal-auto".into(),
            AUTO_ACCEPT,
            (checked(form.auto_accept), true),
            cx.listener(|this, _event, _window, cx| {
                this.compass.live_mut().toggle_auto();
                cx.notify();
            }),
        ));
    if form.done_visible {
        client = client.child(button(
            (732.0, 411.0, 75.0),
            "magcal-done",
            DONE,
            true,
            cx.listener(|this, _event: &(), _window, cx| {
                this.compass.live_mut().click_done();
                cx.notify();
            }),
        ));
    }
    if failed {
        client = client.child(button(
            (213.0, 109.0, 75.0),
            "magcal-close",
            CLOSE,
            true,
            cx.listener(|this, _event: &(), _window, cx| {
                this.compass.live_mut().click_close();
                cx.notify();
            }),
        ));
        if live.details().is_some() {
            client = client.child(
                div().absolute().left(px(240.0)).top(px(90.0)).child(
                    crate::probe::measured("magcal-details", div())
                        .id("magcal-details")
                        .text_xs()
                        .text_color(rgb(theme::ACCENT))
                        .underline()
                        .cursor_pointer()
                        .child(DETAILS)
                        .on_click(cx.listener(|this, _event, _window, cx| {
                            this.compass.show_live_details();
                            cx.notify();
                        })),
                ),
            );
        }
    }
    let caption = div()
        .px_2()
        .py_1()
        .border_b_1()
        .border_color(rgb(theme::BORDER))
        .text_xs()
        .text_color(rgb(theme::DIM))
        .child(if failed { ERROR_TEXT } else { FORM_TEXT });
    let body = crate::probe::measured("magcal-form", div())
        .flex()
        .flex_col()
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::ACCENT))
        .rounded_md()
        .child(caption)
        .child(client);
    let size = window.viewport_size();
    Some(
        gpui::deferred(
            gpui::anchored()
                .position(point(px(0.0), px(0.0)))
                .child(
                    div()
                        .id("magcal-backdrop")
                        .w(size.width)
                        .h(size.height)
                        .flex()
                        .items_center()
                        .justify_center()
                        .occlude()
                        .child(body),
                ),
        )
        // Under the page's boxes, which "run the calibration anyway?" is one of.
        .with_priority(1)
        .into_any_element(),
    )
}
