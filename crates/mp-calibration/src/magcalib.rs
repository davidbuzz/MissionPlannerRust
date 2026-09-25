//! Mission Planner's own compass fit, `MagCalib.cs`: offsets (and an ellipsoid) least-squares
//! fitted to magnetometer samples on the ground station, rather than by the vehicle's onboard
//! calibrator.
//!
//! The C# file has two users. **Log calibration** (`ProcessLog`, behind the hidden Temp screen's
//! `BUT_magfit2`, "mag calb log", `temp.cs:410-413`) reads a log's samples - a
//! telemetry log's `RAW_IMU` with `SENSOR_OFFSETS` taken back off ([`TlogSamples`], `getOffsets`),
//! or a dataflash log's `MAG` lines with their `OfsX/Y/Z` taken back off ([`DataflashSamples`],
//! `getOffsetsLog`) - fits them ([`fit_tlog`], [`fit_dataflash`]) and shows the offsets in a box
//! ([`manual_message`] when it cannot write them, [`saved_message`] when it did).
//! **Live calibration** (`DoGUIMagCalib`, the "Live Calibration" button) gathers the same samples
//! from the link while the operator turns the vehicle, draws them on three spheres and runs the
//! same fit. This module is the maths and the words both share; the pages come later (PLAN.md
//! §13.4 row 44 records Live Calibration as the compass page's one missing wiring):
//!
//! - Log calibration reads the chosen file the way `headless-planner magcal` does -
//!   [`TlogSamples`] or [`DataflashSamples`], then [`fit_tlog`] or [`fit_dataflash`] - and then
//!   writes [`offset_params`] through the link and shows [`saved_message`], or shows
//!   [`manual_message`] when not connected. No compass page calls it: the older page's
//!   `BUT_MagCalibrationLog_Click` (`ConfigHWCompass.cs:362-373`, asking "Min Throttle") is wired
//!   to no button by its Designer - dead C#, recorded rather than ported (PLAN.md §12 D16) - and
//!   the Temp screen is not ported.
//! - "Live Calibration" will feed `RAW_IMU`/`SCALED_IMU2`/`SCALED_IMU3` through [`SampleFilter`],
//!   run [`least_sq`] each second past 100 samples for the sphere centres and [`live_error`] for
//!   its status text, then [`remove_outliers`] and [`least_sq`] with the ellipsoid when the vehicle
//!   has `COMPASS_DIA_X`. Its sphere-coverage test (`:625-693`), `GetColour` and the "Bad compass
//!   raw values" sign check (`:728-735`) are the page's and are not here.
//!
//! # The fit, and what differs from alglib
//!
//! The C# fits with alglib's `minlmcreatev` - Levenberg-Marquardt on a numerical Jacobian - with
//! `diffstep` 0.1, `epsx` 0 and `maxits` 100 (`doLSQ`, `MagCalib.cs:1194-1204`). alglib is not
//! ported; the `levenberg-marquardt` crate (a MINPACK `lmder` translation) does the minimising,
//! held to PLAN.md §7.2's class D rather than to bit equality:
//!
//! - **The Jacobian** is alglib's own central difference, computed here the way `minlmiteration`
//!   computes it (`ExtLibs/alglibnet/optimization.cs:44104-44188`, ALGLIB 3.14): each parameter
//!   moved by `diffstep` × its scale (1, as `minlmsetscale` is never called) either way from the
//!   base point, and the column `(f(x+h) - f(x-h)) / (x+h - (x-h))` written as alglib writes it.
//!   alglib recomputes it only every `2n` iterations and makes rank-one secant updates between
//!   (`minlmsetacctype(state, 1)`, `:43615-43642`); MINPACK recomputes it on every accepted step.
//! - **`epsx` 0** means no step is ever small enough to stop on (`:46366-46398` - a step of
//!   exactly zero is). Here `xtol`, `ftol` and `gtol` are all 0, so the crate stops only when
//!   rounding leaves nothing to gain (its `NoImprovementPossible`), which is alglib's completion
//!   code 7, "stopping conditions are too stringent, further improvement is impossible".
//! - **`maxits` 100** counts accepted steps (`repiterationscount`, `:44563-44567`). MINPACK
//!   computes the Jacobian once per accepted step, so the 101st request is refused and the crate
//!   stops there, on the last accepted point - alglib's code 5. The crate's own cap, `patience`
//!   × (n + 1) residual evaluations, is given `patience` = `maxits`: a backstop reached first only
//!   if the steps averaged more than n rejected trials each, where alglib has no such cap at all.
//! - **The damping** differs: alglib's λ starts at 0.001 × the largest diagonal of 2JᵀJ and
//!   moves by ×2ν/×0.33 (`:44331-44342`, `:45854-45900`); MINPACK's trust region by its own
//!   rules. The two walk different paths to the same minimum, which is why class D compares
//!   residuals, not bits.
//! - **What alglib hands back** is `state.x` (`minlmresultsbuf`, `:45513-45530`), the array the
//!   last `fvec` call received - and `sphere_ellipsoid_error` writes the normalised diagonals back
//!   into its argument (`MagCalib.cs:1446-1453`). So the C#'s ellipsoid answer always has
//!   diagonals of length √3; [`do_lsq`] evaluates the model once more at the crate's answer, as the
//!   C#'s last callback does, so this one has too.
//!
//! `mul_add` appears nowhere: the Rust does not contract to FMA, and neither is it asked to.

use std::cell::Cell;
use std::collections::HashMap;

use levenberg_marquardt::{LeastSquaresProblem, LevenbergMarquardt, TerminationReason};
use mp_mavlink_dialects::all::MavMessage;
use nalgebra::storage::Owned;
use nalgebra::{DMatrix, DVector, Dyn};

use crate::CalibrationError;

/// One magnetometer sample, the C#'s `Tuple<float, float, float>`.
pub type Sample = [f32; 3];

/// The bucket size of the duplicate filter: a sample's key is each axis divided by this, as an
/// integer. `// C#: MagCalib.cs:1019` (`getOffsets`), `:174` (live)
pub const FILTER_DIV: i32 = 20;

/// A bucket takes its first this many samples; later ones are dropped as near-duplicates.
/// `// C#: MagCalib.cs:1044` (`getOffsets`), `:204`, `:241`, `:281` (live)
pub const FILTER_LIMIT: u32 = 3;

/// Fewer samples than this is "Log does not contain enough data".
/// `// C#: MagCalib.cs:1070` (`getOffsets`), `:762` (live)
pub const MIN_SAMPLES: usize = 10;

/// The box shown when there are fewer than [`MIN_SAMPLES`].
/// `// C#: MagCalib.cs:1072, 764`
pub const NOT_ENOUGH_DATA: &str = "Log does not contain enough data";

/// The farthest `1/OUTLIER_DIVISOR` of the samples from the origin are dropped before the fit.
/// `// C#: MagCalib.cs:1090` (`getOffsets`), `:810` (`RemoveOutliers`)
pub const OUTLIER_DIVISOR: usize = 16;

/// alglib's differentiation step. `// C#: MagCalib.cs:1203`
pub const DIFF_STEP: f64 = 0.1;

/// alglib's step-size stopping rule: 0, never. `// C#: MagCalib.cs:1197`
pub const EPSX: f64 = 0.0;

/// alglib's iteration limit. `// C#: MagCalib.cs:1198`
pub const MAX_ITS: usize = 100;

/// The result boxes' title. `// C#: MagCalib.cs:1317, 1323`
pub const TITLE: &str = "New Mag Offsets";

/// alglib's `terminationtype` 2: the step was no longer than `epsx`.
pub const TERMINATION_STEP: i32 = 2;
/// alglib's `terminationtype` 5: `maxits` steps were taken.
pub const TERMINATION_MAXITS: i32 = 5;
/// alglib's `terminationtype` 7: no further improvement is possible.
pub const TERMINATION_STRINGENT: i32 = 7;
/// alglib's `terminationtype` -8: NaN or infinity met.
pub const TERMINATION_NAN: i32 = -8;

/// Which residual function a fit uses.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Model {
    /// `sphere_error`: four parameters, the offsets and the radius.
    /// `// C#: MagCalib.cs:1482-1498`
    Sphere,
    /// `sphere_ellipsoid_error`: six (offsets, diagonals) or nine (and off-diagonals), fitted to
    /// the radius `rad`. The offsets are parameters but are not used - the C#'s
    /// `new Vector3(0, 0, 0);//(p1[0], p1[1], p1[2])` - so the ellipsoid is centred on the
    /// origin, whatever the sphere fit found, and the offsets come back as they went in.
    /// `// C#: MagCalib.cs:1441-1480`
    SphereEllipsoid,
}

/// One `doLSQ` run: where it started, where it stopped and how.
#[derive(Debug, Clone, PartialEq)]
pub struct Stage {
    /// The residual function.
    pub model: Model,
    /// The starting parameters.
    pub start: Vec<f64>,
    /// The answer, as `minlmresults` returns it.
    pub x: Vec<f64>,
    /// alglib's objective at the start: the sum of the squared residuals.
    pub start_residual: f64,
    /// alglib's objective at the answer.
    pub residual: f64,
    /// The residual vector at the answer, `state.fi`.
    pub fi: Vec<f64>,
    /// alglib's `terminationtype` this run corresponds to (see [`TERMINATION_MAXITS`] and the
    /// others).
    pub termination: i32,
    /// Why the crate stopped, in its words, for the log `doLSQ` writes.
    pub reason: String,
    /// Accepted steps, `rep.iterationscount`.
    pub iterations: usize,
    /// Residual evaluations the crate made, not counting the Jacobian's.
    pub evaluations: usize,
}

/// `LeastSq`: the sphere fit, and optionally the ellipsoid after it.
#[derive(Debug, Clone, PartialEq)]
pub struct LeastSq {
    /// The answer: offsets and radius (4 values), or offsets, diagonals and off-diagonals (9).
    pub x: Vec<f64>,
    /// `avg_samples`: the samples' mean distance from the origin, which the C# keeps in the
    /// static `rad` for the ellipsoid's residuals. Here it is carried rather than shared.
    pub rad: f64,
    /// Each `doLSQ` in order: the sphere, then the six- and nine-parameter ellipsoid.
    pub stages: Vec<Stage>,
}

/// `calcRadius`: the samples' mean distance from the origin. `// C#: MagCalib.cs:1141-1152`
#[must_use]
pub fn calc_radius(data: &[Sample]) -> f64 {
    let mut avg_samples = 0.0;
    for item in data {
        let (x, y, z) = (f64::from(item[0]), f64::from(item[1]), f64::from(item[2]));
        avg_samples += (x * x + y * y + z * z).sqrt();
    }
    avg_samples / data.len() as f64
}

/// `LeastSq`: the sphere from the origin with the mean radius, then - with `ellipsoid` - the
/// ellipsoid from the sphere's offsets with unit diagonals, then again with off-diagonals.
///
/// # Errors
///
/// [`CalibrationError::NoSamples`] for an empty set, where alglib's `minlmcreatev` throws.
/// `// C#: MagCalib.cs:1159-1192`
pub fn least_sq(data: &[Sample], ellipsoid: bool) -> Result<LeastSq, CalibrationError> {
    if data.is_empty() {
        // `calcRadius` divides by zero, and `minlmcreatev` asserts M >= 1 and a finite X.
        return Err(CalibrationError::NoSamples);
    }
    let avg_samples = calc_radius(data);
    let sphere = do_lsq(
        data,
        Model::Sphere,
        &[0.0, 0.0, 0.0, avg_samples],
        avg_samples,
    );
    let rad = avg_samples; // `rad = avg_samples;//x[3];` (:1172)
    let mut x = sphere.x.clone();
    x.truncate(4);
    let mut stages = vec![sphere];
    if ellipsoid {
        let [x0, x1, x2] = first_three(&x);
        let six = do_lsq(
            data,
            Model::SphereEllipsoid,
            &[x0, x1, x2, 1.0, 1.0, 1.0],
            rad,
        );
        let mut start = six.x.clone();
        start.extend([0.0, 0.0, 0.0]);
        stages.push(six);
        let nine = do_lsq(data, Model::SphereEllipsoid, &start, rad);
        x.clone_from(&nine.x);
        stages.push(nine);
    }
    Ok(LeastSq { x, rad, stages })
}

/// `doLSQ`: one Levenberg-Marquardt run of `model` from `start`, `rad` the radius the ellipsoid
/// is fitted to. The module's documentation says how this differs from alglib.
/// `// C#: MagCalib.cs:1194-1262`
#[must_use]
pub fn do_lsq(data: &[Sample], model: Model, start: &[f64], rad: f64) -> Stage {
    let mut start_x = start.to_vec();
    let start_fi = evaluate(model, &mut start_x, data, rad);
    let problem = Problem {
        data,
        model,
        rad,
        x: DVector::from_column_slice(start),
        jacobians: Cell::new(0),
    };
    let solver = LevenbergMarquardt::new()
        .with_ftol(EPSX)
        .with_xtol(EPSX)
        .with_gtol(0.0)
        .with_patience(MAX_ITS);
    let (problem, report) = solver.minimize(problem);
    let accepted = problem.jacobians.get().saturating_sub(1);
    let termination = termination_type(&report.termination, accepted);
    // The C#'s answer is `state.x` after the last `fvec`, which normalised the diagonals in place.
    let mut x = problem.x.as_slice().to_vec();
    let fi = evaluate(model, &mut x, data, rad);
    Stage {
        model,
        start: start.to_vec(),
        start_residual: sum_of_squares(&start_fi),
        residual: sum_of_squares(&fi),
        x,
        fi,
        termination,
        reason: format!("{:?}", report.termination),
        iterations: accepted.min(MAX_ITS),
        evaluations: report.number_of_evaluations,
    }
}

/// The error the live calibration shows for each compass: the square root of the absolute sum of
/// the residuals - their sum, not their squares' - rounded to two places, half to even as
/// `Math.Round` rounds. `// C#: MagCalib.cs:1227-1261`
#[must_use]
pub fn live_error(fi: &[f64]) -> f64 {
    let mut error = 0.0;
    for item in fi {
        error += item;
    }
    let root = f64::abs(error).sqrt();
    (root * 100.0).round_ties_even() / 100.0
}

/// alglib's objective: the sum of the squared residuals.
#[must_use]
pub fn sum_of_squares(fi: &[f64]) -> f64 {
    let mut sum = 0.0;
    for value in fi {
        sum += value * value;
    }
    sum
}

/// The residuals of `model` at `p`, as the C#'s callbacks compute them - including
/// `sphere_ellipsoid_error`'s writing the normalised diagonals back into `p`.
/// `// C#: MagCalib.cs:1441-1498`
#[must_use]
pub fn evaluate(model: Model, p: &mut [f64], data: &[Sample], rad: f64) -> Vec<f64> {
    match model {
        Model::Sphere => sphere_error(p, data),
        Model::SphereEllipsoid => sphere_ellipsoid_error(p, data, rad),
    }
}

/// `sphere_error`. `// C#: MagCalib.cs:1482-1498`
fn sphere_error(xi: &[f64], data: &[Sample]) -> Vec<f64> {
    let [xofs, yofs, zofs] = first_three(xi);
    let r = xi.get(3).copied().unwrap_or(0.0);
    data.iter()
        .map(|d| {
            let (x, y, z) = (f64::from(d[0]), f64::from(d[1]), f64::from(d[2]));
            let (dx, dy, dz) = (x + xofs, y + yofs, z + zofs);
            r - (dx * dx + dy * dy + dz * dz).sqrt()
        })
        .collect()
}

/// `sphere_ellipsoid_error`: the diagonals normalised to length √3 and written back into `p1`,
/// the off-diagonals as they are, the offsets unused. `// C#: MagCalib.cs:1441-1468`
fn sphere_ellipsoid_error(p1: &mut [f64], data: &[Sample], rad: f64) -> Vec<f64> {
    let offsets = [0.0, 0.0, 0.0]; // `new Vector3(0, 0, 0);//(p1[0], p1[1], p1[2]);` (:1444)
    let mut diagonals = [1.0, 1.0, 1.0];
    let mut offdiagonals = [0.0, 0.0, 0.0];
    if p1.len() >= 6
        && let Some([a, b, c]) = p1.get_mut(3..6)
    {
        // `diagonals.normalized() * Math.Sqrt(3)`: each divided by the length, then multiplied.
        let length = (*a * *a + *b * *b + *c * *c).sqrt();
        let sqrt3 = 3.0_f64.sqrt();
        diagonals = [
            *a / length * sqrt3,
            *b / length * sqrt3,
            *c / length * sqrt3,
        ];
        [*a, *b, *c] = diagonals;
    }
    if p1.len() >= 8
        && let Some([a, b, c]) = p1.get(6..9)
    {
        offdiagonals = [*a, *b, *c];
    }
    data.iter()
        .map(|d| {
            let mag = [f64::from(d[0]), f64::from(d[1]), f64::from(d[2])];
            rad - radius(mag, offsets, diagonals, offdiagonals)
        })
        .collect()
}

/// `radius`: the length of the sample, offset, through the symmetric matrix of diagonals and
/// off-diagonals, `Matrix3 * Vector3` row by row. `// C#: MagCalib.cs:1470-1479`,
/// `ExtLibs/Utilities/Matrix3.cs:185-190`, `Vector3.cs:254-257`
fn radius(mag: [f64; 3], offsets: [f64; 3], diagonals: [f64; 3], offdiagonals: [f64; 3]) -> f64 {
    let v = [
        mag[0] + offsets[0],
        mag[1] + offsets[1],
        mag[2] + offsets[2],
    ];
    let [dx, dy, dz] = diagonals;
    let [ox, oy, oz] = offdiagonals;
    let a = [dx, ox, oy];
    let b = [ox, dy, oz];
    let c = [oy, oz, dz];
    let x = a[0] * v[0] + a[1] * v[1] + a[2] * v[2];
    let y = b[0] * v[0] + b[1] * v[1] + b[2] * v[2];
    let z = c[0] * v[0] + c[1] * v[1] + c[2] * v[2];
    (x * x + y * y + z * z).sqrt()
}

/// The first three values, zero past the end.
fn first_three(x: &[f64]) -> [f64; 3] {
    let at = |i: usize| x.get(i).copied().unwrap_or(0.0);
    [at(0), at(1), at(2)]
}

/// alglib's completion code for where the crate stopped. `accepted` is how many steps were
/// accepted, which tells the `maxits` refusal from any other.
fn termination_type(reason: &TerminationReason, accepted: usize) -> i32 {
    match reason {
        TerminationReason::User("jacobian") if accepted >= MAX_ITS => TERMINATION_MAXITS,
        TerminationReason::LostPatience => TERMINATION_MAXITS,
        // A zero residual, a zero gradient or a zero trust region: alglib's step comes out zero,
        // which `epsx` 0 accepts (`optimization.cs:46366-46398`).
        TerminationReason::ResidualsZero
        | TerminationReason::Orthogonal
        | TerminationReason::Converged { xtol: true, .. } => TERMINATION_STEP,
        TerminationReason::Converged { .. } | TerminationReason::NoImprovementPossible(_) => {
            TERMINATION_STRINGENT
        }
        // Not finite, or a residual refused; the others cannot happen with n >= 4 and m >= 1.
        TerminationReason::Numerical(_)
        | TerminationReason::User(_)
        | TerminationReason::NoParameters
        | TerminationReason::NoResiduals
        | TerminationReason::WrongDimensions(_) => TERMINATION_NAN,
    }
}

/// The least-squares problem the crate minimises: the model's residuals, and alglib's
/// finite-difference Jacobian of them.
struct Problem<'a> {
    data: &'a [Sample],
    model: Model,
    rad: f64,
    x: DVector<f64>,
    /// How many Jacobians have been asked for: one at the start, then one per accepted step.
    jacobians: Cell<usize>,
}

impl Problem<'_> {
    fn residuals_at(&self, x: &[f64]) -> Vec<f64> {
        let mut p = x.to_vec();
        evaluate(self.model, &mut p, self.data, self.rad)
    }
}

impl LeastSquaresProblem<f64, Dyn, Dyn> for Problem<'_> {
    type ResidualStorage = Owned<f64, Dyn>;
    type JacobianStorage = Owned<f64, Dyn, Dyn>;
    type ParameterStorage = Owned<f64, Dyn>;

    fn set_params(&mut self, x: &DVector<f64>) {
        self.x.copy_from(x);
    }

    fn params(&self) -> DVector<f64> {
        self.x.clone()
    }

    fn residuals(&self) -> Option<DVector<f64>> {
        Some(DVector::from_vec(self.residuals_at(self.x.as_slice())))
    }

    /// `minlmiteration`'s finite differences (`optimization.cs:44104-44188`); refused once
    /// `maxits` steps have been accepted, which stops the crate on the last of them.
    fn jacobian(&self) -> Option<DMatrix<f64>> {
        let asked = self.jacobians.get();
        self.jacobians.set(asked + 1);
        if asked >= MAX_ITS {
            return None;
        }
        let base = self.x.as_slice();
        let n = base.len();
        let m = self.data.len();
        let mut j = DMatrix::zeros(m, n);
        for k in 0..n {
            let mut x = base.to_vec();
            let xk = base.get(k).copied().unwrap_or(0.0);
            // State.S[k] is 1: `minlmsetscale` is never called.
            let xm1 = xk - DIFF_STEP;
            if let Some(slot) = x.get_mut(k) {
                *slot = xm1;
            }
            let fm1 = self.residuals_at(&x);
            let xp1 = xk + DIFF_STEP;
            if let Some(slot) = x.get_mut(k) {
                *slot = xp1;
            }
            let fp1 = self.residuals_at(&x);
            let v = xp1 - xm1;
            if v != 0.0 {
                let v = 1.0 / v;
                for (i, (fp, fm)) in fp1.iter().zip(fm1.iter()).enumerate() {
                    if let Some(cell) = j.get_mut((i, k)) {
                        *cell = v * fp - v * fm;
                    }
                }
            }
        }
        Some(j)
    }
}

/// The near-duplicate filter: a sample is kept while its bucket - each axis divided by
/// [`FILTER_DIV`] as an integer, truncating - has had no more than [`FILTER_LIMIT`].
/// `// C#: MagCalib.cs:1036-1050` (`getOffsets`), `:196-212, 233-249, 273-289` (live)
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct SampleFilter {
    counts: HashMap<[i32; 3], u32>,
}

impl SampleFilter {
    /// A filter that has seen nothing.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Counts a raw reading into its bucket, and says whether to keep it.
    pub fn admit(&mut self, x: i16, y: i16, z: i16) -> bool {
        let item = [
            i32::from(x) / FILTER_DIV,
            i32::from(y) / FILTER_DIV,
            i32::from(z) / FILTER_DIV,
        ];
        let count = self.counts.entry(item).or_insert(0);
        *count += 1;
        *count <= FILTER_LIMIT
    }
}

/// `getOffsets`' pass over a telemetry log: `VFR_HUD` switches the samples on and off by the
/// throttle, `SENSOR_OFFSETS` is the offset each later sample has taken off again, and `RAW_IMU`'s
/// magnetometer is the sample, through the [`SampleFilter`]. Feed it every message of the log in
/// order. `// C#: MagCalib.cs:940-1059`
#[derive(Debug, Clone, PartialEq)]
pub struct TlogSamples {
    throttle_threshold: i32,
    use_data: bool,
    offset: [f32; 3],
    filter: SampleFilter,
    min: [f32; 3],
    max: [f32; 3],
    /// The samples, filtered, in log order.
    pub data: Vec<Sample>,
    /// Every sample taken, before the filter - what the C# draws into its `.dxf`.
    pub vertexes: usize,
}

impl TlogSamples {
    /// Ready to read a log, using only samples while the throttle is at least
    /// `throttle_threshold` percent (all of them when it is 0 or less).
    /// `// C#: MagCalib.cs:963-967`
    #[must_use]
    pub fn new(throttle_threshold: i32) -> Self {
        Self {
            throttle_threshold,
            use_data: throttle_threshold <= 0,
            offset: [0.0; 3],
            filter: SampleFilter::new(),
            min: [0.0; 3],
            max: [0.0; 3],
            data: Vec::new(),
            vertexes: 0,
        }
    }

    /// One message of the log. `// C#: MagCalib.cs:988-1057`
    pub fn message(&mut self, message: &MavMessage) {
        match message {
            MavMessage::VfrHud(hud) => {
                self.use_data = i32::from(hud.throttle) >= self.throttle_threshold;
            }
            MavMessage::SensorOffsets(offsets) => {
                self.offset = [
                    f32::from(offsets.mag_ofs_x),
                    f32::from(offsets.mag_ofs_y),
                    f32::from(offsets.mag_ofs_z),
                ];
            }
            MavMessage::RawImu(imu) if self.use_data => {
                // `short - float`, in float.
                let sample = [
                    f32::from(imu.xmag) - self.offset[0],
                    f32::from(imu.ymag) - self.offset[1],
                    f32::from(imu.zmag) - self.offset[2],
                ];
                self.vertexes += 1;
                for ((value, min), max) in sample.iter().zip(&mut self.min).zip(&mut self.max) {
                    set_min_or_max(*value, min, max);
                }
                if self.filter.admit(imu.xmag, imu.ymag, imu.zmag) {
                    self.data.push(sample);
                }
            }
            _ => {}
        }
    }

    /// The last `SENSOR_OFFSETS`' magnetometer offsets, the C#'s "Current offset".
    /// `// C#: MagCalib.cs:1063`
    #[must_use]
    pub const fn offset(&self) -> [f32; 3] {
        self.offset
    }

    /// The "old method": minus the middle of each axis's range, from zero-started extremes.
    /// `// C#: MagCalib.cs:1092`
    #[must_use]
    pub fn old_method(&self) -> [f32; 3] {
        let mut middle = [0.0; 3];
        for ((out, max), min) in middle.iter_mut().zip(self.max).zip(self.min) {
            // Adding zero makes a negative zero positive, as .NET writes it.
            *out = -(max + min) / 2.0 + 0.0;
        }
        middle
    }
}

/// `setMinorMax`. `// C#: MagCalib.cs:1431-1437`
fn set_min_or_max(value: f32, min: &mut f32, max: &mut f32) {
    if value > *max {
        *max = value;
    }
    if value < *min {
        *min = value;
    }
}

/// `getOffsetsLog`'s pass over a dataflash log: each `MAG`, `MAG2` and `MAG3` line with both a
/// `MagX` and an `OfsX` column is a sample of compass 1, 2 or 3 with its offsets taken back off,
/// and those offsets are kept as the compass's "old ofs". Every `MAG` line goes to compass 1,
/// whatever its instance field says, as in the C#. `// C#: MagCalib.cs:833-899`
#[derive(Debug, Clone, Default, PartialEq)]
pub struct DataflashSamples {
    /// The samples of compass 1, 2 and 3, in log order.
    pub data: [Vec<Sample>; 3],
    /// The offsets each compass's last line carried, `ofsDoubles`.
    pub old_offsets: [[f64; 3]; 3],
}

impl DataflashSamples {
    /// Nothing read yet.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// One line of type `msgtype` with its `MagX/Y/Z` and `OfsX/Y/Z`, parsed as `float.Parse`
    /// parses them. Other types are ignored. `// C#: MagCalib.cs:845-897`
    pub fn line(&mut self, msgtype: &str, mag: [f32; 3], ofs: [f32; 3]) {
        let compass = match msgtype {
            "MAG" => 0,
            "MAG2" => 1,
            "MAG3" => 2,
            _ => return,
        };
        let sample = [mag[0] - ofs[0], mag[1] - ofs[1], mag[2] - ofs[2]];
        if let Some(data) = self.data.get_mut(compass) {
            data.push(sample);
        }
        if let Some(old) = self.old_offsets.get_mut(compass) {
            *old = [f64::from(ofs[0]), f64::from(ofs[1]), f64::from(ofs[2])];
        }
    }
}

/// What a log's fit found for compass 1.
#[derive(Debug, Clone, PartialEq)]
pub struct LogFit {
    /// How many samples were fitted, after the outliers went.
    pub samples: usize,
    /// `LeastSq(data)`: the sphere.
    pub sphere: LeastSq,
    /// `LeastSq(data, true)`: the ellipsoid.
    pub ellipsoid: LeastSq,
    /// The telemetry path's third fit, the nine-parameter ellipsoid again from the offsets with
    /// unit diagonals (`:1102`). `None` on the dataflash path.
    pub refit: Option<Stage>,
    /// The answer `ProcessLog` hands to `SaveOffsets`: the last fit's first three values.
    pub offsets: [f64; 3],
}

/// `getOffsets` after its pass over the log: at least [`MIN_SAMPLES`], the farthest sixteenth
/// dropped, the sphere, the ellipsoid, the nine-parameter ellipsoid again, and the offsets of the
/// last. The `.dxf` it writes is not (see the ledger).
///
/// # Errors
///
/// [`CalibrationError::NotEnoughData`] under [`MIN_SAMPLES`], where the C# shows
/// [`NOT_ENOUGH_DATA`] and throws. `// C#: MagCalib.cs:1070-1117`
pub fn fit_tlog(mut data: Vec<Sample>) -> Result<LogFit, CalibrationError> {
    if data.len() < MIN_SAMPLES {
        return Err(CalibrationError::NotEnoughData {
            samples: data.len(),
        });
    }
    let capacity = list_capacity(data.len());
    remove_outliers(&mut data, capacity);
    let sphere = least_sq(&data, false)?;
    let ellipsoid = least_sq(&data, true)?;
    let [x0, x1, x2] = first_three(&ellipsoid.x);
    let refit = do_lsq(
        &data,
        Model::SphereEllipsoid,
        &[x0, x1, x2, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0],
        ellipsoid.rad,
    );
    let offsets = first_three(&refit.x);
    Ok(LogFit {
        samples: data.len(),
        sphere,
        ellipsoid,
        refit: Some(refit),
        offsets,
    })
}

/// `getOffsetsLog` for one compass: the sphere and the ellipsoid on every sample, with no count
/// check, no filter and no outliers dropped, and the ellipsoid's offsets as the answer.
///
/// # Errors
///
/// [`CalibrationError::NoSamples`] for a compass with no lines, where alglib throws and
/// `ProcessLog` swallows it. `// C#: MagCalib.cs:901-932`
pub fn fit_dataflash(data: &[Sample]) -> Result<LogFit, CalibrationError> {
    let sphere = least_sq(data, false)?;
    let ellipsoid = least_sq(data, true)?;
    let offsets = first_three(&ellipsoid.x);
    Ok(LogFit {
        samples: data.len(),
        sphere,
        ellipsoid,
        refit: None,
        offsets,
    })
}

/// `RemoveOutliers`, and `getOffsets`' copy of it: sorted by distance from the origin with the
/// C#'s unstable sort, and the last `count / 16` removed. `capacity` is the list's backing array,
/// which sets the sort's depth limit (see [`list_capacity`]).
/// `// C#: MagCalib.cs:793-811, 1074-1090`
pub fn remove_outliers(data: &mut Vec<Sample>, capacity: usize) {
    cs_sort(data, capacity, |d1, d2| {
        // `d1.Item1 * d1.Item1 + ...` is float arithmetic; only the square root is double.
        let ans1 = f64::from(d1[0] * d1[0] + d1[1] * d1[1] + d1[2] * d1[2]).sqrt();
        let ans2 = f64::from(d2[0] * d2[0] + d2[1] * d2[1] + d2[2] * d2[2]).sqrt();
        if ans1 > ans2 {
            1
        } else if ans1 < ans2 {
            -1
        } else {
            0
        }
    });
    let remove = data.len() / OUTLIER_DIVISOR;
    data.truncate(data.len() - remove);
}

/// The backing-array length of a `List<T>` filled one `Add` at a time: 4, then doubling.
#[must_use]
pub fn list_capacity(count: usize) -> usize {
    if count == 0 {
        0
    } else {
        count.next_power_of_two().max(4)
    }
}

/// `ToString("0")` on a double: rounded half away from zero, never "-0", as .NET Framework writes
/// it (the same rule as `mp-gui`'s compass page).
fn whole(value: f64) -> String {
    let rounded = value.round();
    if rounded == 0.0 {
        "0".to_owned()
    } else {
        format!("{rounded:.0}")
    }
}

fn three(ofs: &[f64]) -> String {
    let [x, y, z] = first_three(ofs);
    format!("{} {} {}", whole(x), whole(y), whole(z))
}

/// The box `SaveOffsets` shows when it wrote the offsets, for compass 1, 2 or 3.
/// `// C#: MagCalib.cs:1315-1317, 1370-1372, 1413-1415`
#[must_use]
pub fn saved_message(compass: u8, ofs: &[f64]) -> String {
    format!(
        "New offsets for compass #{compass} are {}\nThese have been saved for you.",
        three(ofs)
    )
}

/// The box `SaveOffsets` shows when it cannot write them - not connected, or the vehicle has no
/// `COMPASS_OFS_X` - for compass 1, 2 or 3; the third words it differently.
/// `// C#: MagCalib.cs:1321-1323, 1376-1378, 1419-1421`
#[must_use]
pub fn manual_message(compass: u8, ofs: &[f64]) -> String {
    let head = if compass == 3 {
        "New compass3 offsets are".to_owned()
    } else {
        format!("New offsets for compass #{compass} are")
    };
    format!(
        "{head} {}\n\nPlease write these down for manual entry",
        three(ofs)
    )
}

/// The box when writing failed. `// C#: MagCalib.cs:1311, 1366, 1409`
#[must_use]
pub fn failed_message(compass: u8) -> String {
    format!("Setting new offsets for compass #{compass} failed")
}

/// The name part of compass `n`'s parameters: `""`, `"2"`, `"3"`.
fn suffix(compass: u8) -> &'static str {
    match compass {
        2 => "2",
        3 => "3",
        _ => "",
    }
}

/// The parameters `SaveOffsets` writes the offsets to - after `COMPASS_LEARN` 0, and for compasses
/// 1 and 2 only when `MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS` is refused - and which it checks for to
/// decide whether it can write at all (`COMPASS_OFS_X`). `// C#: MagCalib.cs:1273-1295, 1329-1350,
/// 1384-1393`
#[must_use]
pub fn offset_params(compass: u8) -> [String; 3] {
    let s = suffix(compass);
    ["X", "Y", "Z"].map(|axis| format!("COMPASS_OFS{s}_{axis}"))
}

/// The ellipsoid's parameters `SaveOffsets` writes from values 3-8 of a nine-value answer, when
/// the vehicle has the first: `COMPASS_DIA_X/Y/Z`, then `COMPASS_ODI_X/Y/Z`.
/// `// C#: MagCalib.cs:1297-1306, 1352-1361, 1395-1404`
#[must_use]
pub fn ellipsoid_params(compass: u8) -> [String; 6] {
    let s = suffix(compass);
    [
        format!("COMPASS_DIA{s}_X"),
        format!("COMPASS_DIA{s}_Y"),
        format!("COMPASS_DIA{s}_Z"),
        format!("COMPASS_ODI{s}_X"),
        format!("COMPASS_ODI{s}_Y"),
        format!("COMPASS_ODI{s}_Z"),
    ]
}

/// `List<T>.Sort(Comparison<T>)` as .NET Framework runs it, `ArraySortHelper<T>.IntrospectiveSort`:
/// unstable, so which of two equally distant samples is dropped as an outlier is part of the
/// result. Repeated from `mp-mission`'s `clipper.rs` (`cs_sort`, checked against mono there)
/// rather than shared: one private sort is not worth a dependency between the two crates.
fn cs_sort<T: Copy>(keys: &mut [T], capacity: usize, compare: impl Fn(&T, &T) -> i32) {
    if keys.len() < 2 {
        return;
    }
    let depth_limit = 2 * floor_log2(capacity);
    let mut sorter = IntroSort {
        keys,
        compare: &compare,
    };
    sorter.intro_sort(0, sorter.keys.len() - 1, depth_limit);
}

/// `IntrospectiveSortUtilities.FloorLog2`: one more than the floor of the logarithm, as written.
fn floor_log2(mut n: usize) -> usize {
    let mut result = 0;
    while n >= 1 {
        result += 1;
        n /= 2;
    }
    result
}

/// `ArraySortHelper<T>`'s introsort over one slice.
struct IntroSort<'a, T, F> {
    keys: &'a mut [T],
    compare: &'a F,
}

#[allow(clippy::indexing_slicing)] // indices stay within lo..=hi, inside the slice, as in the C#
impl<T: Copy, F: Fn(&T, &T) -> i32> IntroSort<'_, T, F> {
    /// `IntrospectiveSortUtilities.IntrosortSizeThreshold`.
    const SIZE_THRESHOLD: usize = 16;

    fn cmp(&self, a: usize, b: usize) -> i32 {
        (self.compare)(&self.keys[a], &self.keys[b])
    }

    fn swap_if_greater(&mut self, a: usize, b: usize) {
        if a != b && self.cmp(a, b) > 0 {
            self.keys.swap(a, b);
        }
    }

    fn swap(&mut self, i: usize, j: usize) {
        if i != j {
            self.keys.swap(i, j);
        }
    }

    fn intro_sort(&mut self, lo: usize, mut hi: usize, mut depth_limit: usize) {
        while hi > lo {
            let partition_size = hi - lo + 1;
            if partition_size <= Self::SIZE_THRESHOLD {
                if partition_size == 2 {
                    self.swap_if_greater(lo, hi);
                    return;
                }
                if partition_size == 3 {
                    self.swap_if_greater(lo, hi - 1);
                    self.swap_if_greater(lo, hi);
                    self.swap_if_greater(hi - 1, hi);
                    return;
                }
                self.insertion_sort(lo, hi);
                return;
            }
            if depth_limit == 0 {
                self.heapsort(lo, hi);
                return;
            }
            depth_limit -= 1;
            let p = self.pick_pivot_and_partition(lo, hi);
            self.intro_sort(p + 1, hi, depth_limit);
            hi = p - 1;
        }
    }

    fn pick_pivot_and_partition(&mut self, lo: usize, hi: usize) -> usize {
        let middle = lo + ((hi - lo) >> 1);
        self.swap_if_greater(lo, middle);
        self.swap_if_greater(lo, hi);
        self.swap_if_greater(middle, hi);
        let pivot = self.keys[middle];
        self.swap(middle, hi - 1);
        let (mut left, mut right) = (lo, hi - 1);
        while left < right {
            loop {
                left += 1;
                if (self.compare)(&self.keys[left], &pivot) >= 0 {
                    break;
                }
            }
            loop {
                right -= 1;
                if (self.compare)(&pivot, &self.keys[right]) >= 0 {
                    break;
                }
            }
            if left >= right {
                break;
            }
            self.swap(left, right);
        }
        self.swap(left, hi - 1);
        left
    }

    fn heapsort(&mut self, lo: usize, hi: usize) {
        let n = hi - lo + 1;
        for i in (1..=n / 2).rev() {
            self.down_heap(i, n, lo);
        }
        for i in (2..=n).rev() {
            self.swap(lo, lo + i - 1);
            self.down_heap(1, i - 1, lo);
        }
    }

    fn down_heap(&mut self, mut i: usize, n: usize, lo: usize) {
        let d = self.keys[lo + i - 1];
        while i <= n / 2 {
            let mut child = 2 * i;
            if child < n && self.cmp(lo + child - 1, lo + child) < 0 {
                child += 1;
            }
            if (self.compare)(&d, &self.keys[lo + child - 1]) >= 0 {
                break;
            }
            self.keys[lo + i - 1] = self.keys[lo + child - 1];
            i = child;
        }
        self.keys[lo + i - 1] = d;
    }

    fn insertion_sort(&mut self, lo: usize, hi: usize) {
        for i in lo..hi {
            let mut j = i;
            let t = self.keys[i + 1];
            loop {
                if (self.compare)(&t, &self.keys[j]) >= 0 {
                    self.keys[j + 1] = t;
                    break;
                }
                self.keys[j + 1] = self.keys[j];
                if j == lo {
                    self.keys[j] = t;
                    break;
                }
                j -= 1;
            }
        }
    }
}

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::indexing_slicing,
    clippy::float_cmp,
    clippy::needless_range_loop
)]
mod tests {
    use super::*;

    /// Points spread evenly over a sphere (a Fibonacci lattice), `centre + radius * u`.
    fn sphere_points(count: usize, centre: [f64; 3], radius: f64) -> Vec<Sample> {
        let golden = std::f64::consts::PI * (3.0 - 5.0_f64.sqrt());
        (0..count)
            .map(|i| {
                let y = 1.0 - 2.0 * (i as f64 + 0.5) / count as f64;
                let r = (1.0 - y * y).sqrt();
                let theta = golden * i as f64;
                let u = [r * theta.cos(), y, r * theta.sin()];
                #[allow(clippy::cast_possible_truncation)]
                let s = |k: usize| (centre[k] + radius * u[k]) as f32;
                [s(0), s(1), s(2)]
            })
            .collect()
    }

    #[test]
    fn the_filter_keeps_three_per_bucket_and_truncates_towards_zero() {
        let mut filter = SampleFilter::new();
        // -19 / 20 and 19 / 20 are both 0 in C#: the same bucket.
        assert!(filter.admit(-19, 0, 0));
        assert!(filter.admit(19, 0, 0));
        assert!(filter.admit(0, 5, -5));
        assert!(!filter.admit(1, 1, 1));
        assert!(filter.admit(20, 0, 0));
        assert!(filter.admit(-20, 0, 0));
    }

    #[test]
    fn the_tlog_pass_takes_the_offsets_off_and_obeys_the_throttle() {
        use mp_mavlink_dialects::all::{RawImu, SensorOffsets, VfrHud};
        let imu = |x: i16| {
            MavMessage::RawImu(RawImu {
                time_usec: 0,
                xacc: 0,
                yacc: 0,
                zacc: 0,
                xgyro: 0,
                ygyro: 0,
                zgyro: 0,
                xmag: x,
                ymag: 100,
                zmag: -300,
                id: 0,
                temperature: 0,
            })
        };
        let hud = |throttle: u16| {
            MavMessage::VfrHud(VfrHud {
                airspeed: 0.0,
                groundspeed: 0.0,
                alt: 0.0,
                climb: 0.0,
                heading: 0,
                throttle,
            })
        };
        let mut pass = TlogSamples::new(30);
        pass.message(&imu(10)); // before any VFR_HUD, off: the threshold is above 0
        pass.message(&hud(30));
        pass.message(&MavMessage::SensorOffsets(SensorOffsets {
            mag_declination: 0.0,
            raw_press: 0,
            raw_temp: 0,
            gyro_cal_x: 0.0,
            gyro_cal_y: 0.0,
            gyro_cal_z: 0.0,
            accel_cal_x: 0.0,
            accel_cal_y: 0.0,
            accel_cal_z: 0.0,
            mag_ofs_x: 5,
            mag_ofs_y: -7,
            mag_ofs_z: 0,
        }));
        pass.message(&imu(200));
        pass.message(&hud(29));
        pass.message(&imu(400));
        assert_eq!(pass.data, vec![[195.0, 107.0, -300.0]]);
        assert_eq!(pass.offset(), [5.0, -7.0, 0.0]);
        assert_eq!(pass.vertexes, 1);
        // Extremes start at zero: -(195 + 0) / 2, -(107 + 0) / 2, -(0 - 300) / 2.
        assert_eq!(pass.old_method(), [-97.5, -53.5, 150.0]);
    }

    #[test]
    fn every_mag_line_is_compass_one_and_mag2_is_two() {
        let mut pass = DataflashSamples::new();
        pass.line("MAG", [100.0, 50.0, -400.0], [10.0, -5.0, 0.0]);
        pass.line("MAG2", [1.0, 2.0, 3.0], [1.0, 1.0, 1.0]);
        pass.line("MAGX", [1.0, 2.0, 3.0], [1.0, 1.0, 1.0]);
        assert_eq!(pass.data[0], vec![[90.0, 55.0, -400.0]]);
        assert_eq!(pass.data[1], vec![[0.0, 1.0, 2.0]]);
        assert!(pass.data[2].is_empty());
        assert_eq!(pass.old_offsets[0], [10.0, -5.0, 0.0]);
    }

    #[test]
    fn nine_samples_are_not_enough_and_ten_are() {
        let points = sphere_points(10, [0.0, 0.0, 0.0], 300.0);
        let few = points[..9].to_vec();
        assert!(matches!(
            fit_tlog(few),
            Err(CalibrationError::NotEnoughData { samples: 9 })
        ));
        assert!(fit_tlog(points).is_ok());
        assert_eq!(
            CalibrationError::NotEnoughData { samples: 3 }.to_string(),
            NOT_ENOUGH_DATA
        );
    }

    #[test]
    fn an_empty_compass_is_an_error_not_a_panic() {
        assert!(matches!(
            fit_dataflash(&[]),
            Err(CalibrationError::NoSamples)
        ));
    }

    #[test]
    fn outliers_are_the_farthest_sixteenth() {
        let mut data: Vec<Sample> = (1..=32u8).rev().map(|i| [f32::from(i), 0.0, 0.0]).collect();
        remove_outliers(&mut data, list_capacity(32));
        assert_eq!(data.len(), 30);
        assert_eq!(data.first(), Some(&[1.0, 0.0, 0.0]));
        assert_eq!(data.last(), Some(&[30.0, 0.0, 0.0]));
        assert_eq!(list_capacity(5), 8);
        assert_eq!(list_capacity(3), 4);
    }

    #[test]
    fn the_sphere_fit_recovers_known_offsets() {
        let centre = [-120.0, 85.0, 230.0];
        let data = sphere_points(400, centre, 400.0);
        let fit = least_sq(&data, false).unwrap();
        assert_eq!(fit.x.len(), 4);
        for k in 0..3 {
            // The offsets are what is added to a sample: minus the centre.
            assert!((fit.x[k] + centre[k]).abs() < 0.01 * 400.0, "{:?}", fit.x);
        }
        assert!((fit.x[3] - 400.0).abs() < 0.01 * 400.0);
        let stage = &fit.stages[0];
        assert!(stage.residual < stage.start_residual);
        assert!(
            [TERMINATION_STEP, TERMINATION_MAXITS, TERMINATION_STRINGENT]
                .contains(&stage.termination),
            "{}",
            stage.reason
        );
    }

    #[test]
    fn the_ellipsoid_leaves_the_offsets_and_normalises_the_diagonals() {
        let data = sphere_points(300, [30.0, -20.0, 10.0], 350.0);
        let fit = least_sq(&data, true).unwrap();
        let sphere = least_sq(&data, false).unwrap();
        assert_eq!(fit.x.len(), 9);
        assert_eq!(fit.x[..3], sphere.x[..3]);
        let length = (fit.x[3] * fit.x[3] + fit.x[4] * fit.x[4] + fit.x[5] * fit.x[5]).sqrt();
        assert!((length - 3.0_f64.sqrt()).abs() < 1e-12);
        assert_eq!(fit.stages.len(), 3);
        // Each stage starts where the last stopped and never ends worse than it started.
        assert!(fit.stages[1].residual <= fit.stages[1].start_residual);
        assert!(fit.stages[2].residual <= fit.stages[2].start_residual);
        assert!((fit.stages[2].start_residual - fit.stages[1].residual).abs() < 1e-6);
    }

    #[test]
    fn the_live_error_sums_the_residuals_not_their_squares() {
        assert_eq!(live_error(&[1.0, -5.0]), 2.0);
        assert_eq!(live_error(&[0.0]), 0.0);
        // sqrt(2) = 1.41421..., to two places.
        assert_eq!(live_error(&[2.0]), 1.41);
    }

    #[test]
    fn the_boxes_say_what_the_csharp_says() {
        let ofs = [12.5, -0.4, -45.5];
        assert_eq!(
            manual_message(1, &ofs),
            "New offsets for compass #1 are 13 0 -46\n\nPlease write these down for manual entry"
        );
        assert_eq!(
            manual_message(3, &ofs),
            "New compass3 offsets are 13 0 -46\n\nPlease write these down for manual entry"
        );
        assert_eq!(
            saved_message(2, &ofs),
            "New offsets for compass #2 are 13 0 -46\nThese have been saved for you."
        );
        assert_eq!(
            failed_message(3),
            "Setting new offsets for compass #3 failed"
        );
        assert_eq!(offset_params(1)[0], "COMPASS_OFS_X");
        assert_eq!(offset_params(2)[2], "COMPASS_OFS2_Z");
        assert_eq!(ellipsoid_params(3)[4], "COMPASS_ODI3_Y");
    }

    #[test]
    fn the_hundred_step_limit_stops_on_the_last_accepted_point() {
        // A Jacobian refused at the limit is the maxits code; any other refusal is not.
        assert_eq!(
            termination_type(&TerminationReason::User("jacobian"), MAX_ITS),
            TERMINATION_MAXITS
        );
        assert_eq!(
            termination_type(&TerminationReason::User("jacobian"), 3),
            TERMINATION_NAN
        );
        assert_eq!(
            termination_type(&TerminationReason::NoImprovementPossible("xtol"), 12),
            TERMINATION_STRINGENT
        );
    }
}
