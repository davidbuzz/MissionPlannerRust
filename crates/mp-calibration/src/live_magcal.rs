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

//! Mission Planner's Live Calibration, `MagCalib.DoGUIMagCalib` (`MagCalib.cs:136-790`), the half
//! of it that is not a window: the samples gathered from `RAW_IMU`, `SCALED_IMU2` and
//! `SCALED_IMU3` through the near-duplicate filter, the running sphere fit and its error, the
//! sphere-coverage test that says where to turn the vehicle next, the status text, the checks
//! after Done, the last fit, and the command `SaveOffsets` tries first. The fit itself is
//! [`crate::magcalib`]'s; the window, its loop and the requests are the Compass page's (`mp-gui`'s
//! `config/live_magcal.rs`).
//!
//! [`Live`] is `MagCalib`'s static state: the three compasses' samples, their errors and the three
//! answers. It is kept as the C# keeps it - for the application's life - because the C# resets
//! only some of it at each run: `DoGUIMagCalib` sets `ans` to null but not `ans2` or `ans3`, so an
//! answer for the second or third compass from an earlier run is saved again after a run that made
//! none (`:138`, `:753-758`, `:765-766`, `:782-791`). That is ported as it is.

use mp_mavlink_dialects::all::MavMessage;
use mp_vehicle::VehicleId;

use crate::CalibrationError;
use crate::command;
use crate::magcalib::{
    MIN_SAMPLES, Sample, SampleFilter, least_sq, live_error, remove_outliers, set_min_or_max,
};

/// `Strings.MagCalibMsg`, the box before the window.
/// `// C#: ExtLibs/Strings/Strings.resx:417-419`
pub const INTRO: &str =
    "Please click ok and move the autopilot around all axises in a circular motion";

/// `Strings.MissingDataPoints`, asked after Done while the coverage test still wants a point.
/// `// C#: ExtLibs/Strings/Strings.resx:420-422`
pub const MISSING_DATA_POINTS: &str =
    "You are missing data points. do you want to run the calibration anyway?";

/// `Strings.RunAnyway`, that question's caption. `// C#: ExtLibs/Strings/Strings.resx:423-425`
pub const RUN_ANYWAY: &str = "run anyway";

/// The error the window shows when an axis never changed sign. `// C#: MagCalib.cs:731`
pub const BAD_RAW_VALUES: &str = "Bad compass raw values. Check for magnetic interferance.";

/// What each compass's error reads before its first fit. `// C#: MagCalib.cs:20-22, 144-146`
pub const START_ERROR: f64 = 99.0;

/// The loop fits a compass once a second when it has more samples than this.
/// `// C#: MagCalib.cs:522, 551, 576`
pub const FIT_AFTER: usize = 100;

/// Compasses 2 and 3 are drawn once they have more samples than this. `// C#: MagCalib.cs:610, 620`
pub const DRAW_AFTER: usize = 30;

/// `hittarget`: the coverage test's points that must be hit, less one. `// C#: MagCalib.cs:437`
pub const HIT_TARGET: usize = 14;

/// Auto accept ends the loop below this error. `// C#: MagCalib.cs:689`
pub const ACCEPT_ERROR: f64 = 0.2;

/// A running fit's offsets are shown only when the first is smaller than this.
/// `// C#: MagCalib.cs:533, 561, 586`
pub const CENTRE_LIMIT: f64 = 999.0;

/// The start of the coverage test's message. `// C#: MagCalib.cs:674`
pub const AIM_FOR: &str = "more data needed Aim For ";

/// `MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS`. `// C#: ExtLibs/Mavlink/Mavlink.cs:1097`
pub const CMD_PREFLIGHT_SET_SENSOR_OFFSETS: u16 = 242;

/// `sensoroffsetsenum.magnetometer`, the first compass.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:6439-6447`
pub const SENSOR_MAGNETOMETER: u8 = 2;

/// `sensoroffsetsenum.second_magnetometer`.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:6439-6447`
pub const SENSOR_SECOND_MAGNETOMETER: u8 = 5;

/// `SetSensorOffsets`: `doCommand(PREFLIGHT_SET_SENSOR_OFFSETS, (int) sensor, x, y, z, 0, 0, 0)`.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:6488-6491`
#[must_use]
pub fn set_sensor_offsets(target: VehicleId, sensor: u8, offsets: [f32; 3]) -> MavMessage {
    let [x, y, z] = offsets;
    command(
        target,
        CMD_PREFLIGHT_SET_SENSOR_OFFSETS,
        [f32::from(sensor), x, y, z, 0.0, 0.0, 0.0],
    )
}

/// One compass's samples: `filtercompassN` and `datacompassN`, and the length of the list's
/// backing array, which `List<T>.Clear` keeps and the outlier sort's depth limit reads.
/// `// C#: MagCalib.cs:174-189`
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Samples {
    filter: SampleFilter,
    data: Vec<Sample>,
    capacity: usize,
}

impl Samples {
    /// `filtercompassN.Clear()` and `datacompassN.Clear()`: the capacity stays.
    /// `// C#: MagCalib.cs:139-144`
    pub fn clear(&mut self) {
        self.filter = SampleFilter::new();
        self.data.clear();
    }

    /// `ReceviedPacket` for `RAW_IMU`, compass 1: nothing for a reading whose x and y are both
    /// zero, then the filter, then kept. Whether it was kept.
    /// `// C#: MagCalib.cs:257-292`
    pub fn raw_imu(&mut self, mag: [i16; 3]) -> bool {
        let [x, y, z] = mag;
        if x == 0 && y == 0 {
            return false;
        }
        if !self.filter.admit(x, y, z) {
            return false;
        }
        self.push([f32::from(x), f32::from(y), f32::from(z)]);
        true
    }

    /// `ReceviedPacket` for `SCALED_IMU2` and `SCALED_IMU3`: the filter first - so a zero reading
    /// still counts in its bucket - then nothing for a reading with any axis zero, then kept.
    /// `// C#: MagCalib.cs:189-255`
    pub fn scaled_imu(&mut self, mag: [i16; 3]) -> bool {
        let [x, y, z] = mag;
        if !self.filter.admit(x, y, z) {
            return false;
        }
        if x == 0 || y == 0 || z == 0 {
            return false;
        }
        self.push([f32::from(x), f32::from(y), f32::from(z)]);
        true
    }

    /// `List<T>.Add`: the backing array 4 long at first, then doubled when full.
    fn push(&mut self, sample: Sample) {
        self.data.push(sample);
        while self.data.len() > self.capacity {
            self.capacity = if self.capacity == 0 {
                4
            } else {
                self.capacity * 2
            };
        }
    }

    /// The samples, in the order they were kept.
    #[must_use]
    pub fn data(&self) -> &[Sample] {
        &self.data
    }

    /// How many.
    #[must_use]
    pub fn len(&self) -> usize {
        self.data.len()
    }

    /// Whether there are none.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.data.is_empty()
    }

    /// The newest, `datacompassN[datacompassN.Count - 1]`.
    #[must_use]
    pub fn last(&self) -> Option<Sample> {
        self.data.last().copied()
    }

    /// `RemoveOutliers(ref datacompassN)`. `// C#: MagCalib.cs:793-811`
    pub fn remove_outliers(&mut self) {
        remove_outliers(&mut self.data, self.capacity);
    }
}

/// Which message a reading came in, and so which compass it is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Source {
    /// `RAW_IMU`: compass 1.
    RawImu,
    /// `SCALED_IMU2`: compass 2.
    ScaledImu2,
    /// `SCALED_IMU3`: compass 3.
    ScaledImu3,
}

/// `MagCalib`'s statics: every compass's samples, its error, and the answers `SaveOffsets`,
/// `SaveOffsets2` and `SaveOffsets3` are given. `// C#: MagCalib.cs:20-25, 172-189`
#[derive(Debug, Clone, PartialEq)]
pub struct Live {
    /// `datacompass1` to `3`, with their filters.
    pub samples: [Samples; 3],
    /// `error`, `error2`, `error3`.
    pub errors: [f64; 3],
    /// `ans`, `ans2`, `ans3`.
    pub answers: [Option<Vec<f64>>; 3],
}

impl Default for Live {
    fn default() -> Self {
        Self {
            samples: Default::default(),
            errors: [START_ERROR; 3],
            answers: [None, None, None],
        }
    }
}

impl Live {
    /// `DoGUIMagCalib`'s first lines: `ans` forgotten - not `ans2` or `ans3` - the samples and
    /// filters cleared and the errors back to 99. `// C#: MagCalib.cs:138-146`
    pub fn begin(&mut self) {
        self.answers[0] = None;
        for samples in &mut self.samples {
            samples.clear();
        }
        self.errors = [START_ERROR; 3];
    }

    /// `ReceviedPacket`: one reading. Whether it was kept.
    pub fn sample(&mut self, source: Source, mag: [i16; 3]) -> bool {
        let [first, second, third] = &mut self.samples;
        match source {
            Source::RawImu => first.raw_imu(mag),
            Source::ScaledImu2 => second.scaled_imu(mag),
            Source::ScaledImu3 => third.scaled_imu(mag),
        }
    }

    /// The loop's fit of compass `index` (0 to 2): `LeastSq(datacompassN, false)`, whose `doLSQ`
    /// sets the compass's error from its residuals, and the offsets when the first of them is
    /// under [`CENTRE_LIMIT`] - what the loop makes the sphere's `CenterPoint`. `None` for no fit,
    /// where the C#'s `catch {}` swallows alglib's throw. `// C#: MagCalib.cs:520-594, 1227-1261`
    pub fn running_fit(&mut self, index: usize) -> Option<[f64; 3]> {
        let data = self.samples.get(index)?.data();
        let fit = least_sq(data, false).ok()?;
        let error = fit.stages.last().map(|stage| live_error(&stage.fi))?;
        if let Some(slot) = self.errors.get_mut(index) {
            *slot = error;
        }
        let x0 = fit.x.first().copied().unwrap_or(0.0);
        (x0.abs() < CENTRE_LIMIT).then(|| {
            let at = |i: usize| fit.x.get(i).copied().unwrap_or(0.0);
            [at(0), at(1), at(2)]
        })
    }

    /// After a "Bad compass raw values": `ans` and `ans2` forgotten, not `ans3`.
    /// `// C#: MagCalib.cs:731-733`
    pub fn forget_bad(&mut self) {
        self.answers[0] = None;
        self.answers[1] = None;
    }

    /// After No to "run anyway": all three forgotten. `// C#: MagCalib.cs:743-745`
    pub fn forget_all(&mut self) {
        self.answers = [None, None, None];
    }

    /// The end of `prd_DoWork`: the outliers dropped from compass 1, and from 2 and 3 when the
    /// vehicle has them and they have samples; fewer than [`MIN_SAMPLES`] left on compass 1 is
    /// "Log does not contain enough data" with `ans` and `ans2` forgotten; otherwise each compass
    /// fitted - the ellipsoid too when `ellipsoid` (the vehicle has `COMPASS_DIA_X`) - and its
    /// answer kept. A compass without samples keeps the answer it had.
    ///
    /// # Errors
    ///
    /// [`CalibrationError::NotEnoughData`], whose words are the C#'s; and
    /// [`CalibrationError::NoSamples`] where alglib would throw, which cannot happen past the
    /// count check. `// C#: MagCalib.cs:751-785`
    pub fn finish(
        &mut self,
        have2: bool,
        have3: bool,
        ellipsoid: bool,
    ) -> Result<(), CalibrationError> {
        let [first, second, third] = &mut self.samples;
        first.remove_outliers();
        if have2 && !second.is_empty() {
            second.remove_outliers();
        }
        if have3 && !third.is_empty() {
            third.remove_outliers();
        }
        let count = first.len();
        if count < MIN_SAMPLES {
            self.forget_bad();
            return Err(CalibrationError::NotEnoughData { samples: count });
        }
        let fits = [(true, 0), (have2, 1), (have3, 2)];
        for (have, index) in fits {
            let Some(samples) = self.samples.get(index) else {
                continue;
            };
            if !have || samples.is_empty() {
                continue;
            }
            let fit = least_sq(samples.data(), ellipsoid)?;
            if let Some(error) = fit.stages.last().map(|stage| live_error(&stage.fi))
                && let Some(slot) = self.errors.get_mut(index)
            {
                *slot = error;
            }
            if let Some(answer) = self.answers.get_mut(index) {
                *answer = Some(fit.x);
            }
        }
        Ok(())
    }

    /// The window's text: the count of compass 1's samples - "Got + ", as the C# writes it - each
    /// error the vehicle has a compass for, and the coverage test's message.
    /// `// C#: MagCalib.cs:475-482`
    #[must_use]
    pub fn status(&self, have2: bool, have3: bool, extramsg: &str) -> String {
        let [error1, error2, error3] = self.errors;
        let mut text = format!(
            "Got + {} samples\nCompass 1 error: {}",
            self.samples.first().map_or(0, Samples::len),
            double_text(error1)
        );
        if have2 {
            text.push_str(&format!("\nCompass 2 error: {}", double_text(error2)));
        }
        if have3 {
            text.push_str(&format!("\nCompass 3 error: {}", double_text(error3)));
        }
        text.push('\n');
        text.push_str(extramsg);
        text
    }
}

/// `double.ToString()` for the errors, which are rounded to two places: the shortest form, as
/// .NET Framework's fifteen digits give for such a number, and its words for the non-finite.
fn double_text(value: f64) -> String {
    if value.is_nan() {
        "NaN".to_owned()
    } else if value.is_infinite() {
        if value > 0.0 {
            "Infinity".to_owned()
        } else {
            "-Infinity".to_owned()
        }
    } else {
        format!("{value}")
    }
}

/// The loop's "old method" extremes of compass 1's newest sample, from zero.
/// `// C#: MagCalib.cs:428-435, 508-510`
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Extremes {
    /// `minx`, `miny`, `minz`.
    pub min: [f32; 3],
    /// `maxx`, `maxy`, `maxz`.
    pub max: [f32; 3],
}

impl Extremes {
    /// `setMinorMax` on each axis.
    pub fn take(&mut self, sample: Sample) {
        for ((value, min), max) in sample.iter().zip(&mut self.min).zip(&mut self.max) {
            set_min_or_max(*value, min, max);
        }
    }

    /// The check after the loop: an axis whose extremes have the same sign. As both start at
    /// zero, a minimum above zero or a maximum below it cannot happen, so it never holds on the
    /// loop's extremes; ported as written. `// C#: MagCalib.cs:728-735`
    #[must_use]
    pub fn bad(&self) -> bool {
        self.min.iter().zip(&self.max).any(|(min, max)| {
            (*min > 0.0 && *max > 0.0) || (*min < 0.0 && *max < 0.0)
        })
    }
}

/// `GetColour`: which of the sphere display's axis colours a direction is nearest, by its pitch
/// from straight up and its yaw. A yaw of exactly 270 names none. `// C#: MagCalib.cs:27-57`
#[must_use]
pub const fn get_colour(pitch: i32, yaw: i32) -> &'static str {
    if pitch == 0 {
        return "DarkBlue";
    }
    if pitch == 180 {
        return "Yellow";
    }
    if pitch < 90 {
        if yaw < 90 || yaw > 270 {
            return "DarkBlue-Red";
        }
        if yaw < 180 {
            return "DarkBlue-Blue";
        }
        if yaw < 270 {
            return "DarkBlue-Pink";
        }
    } else {
        if yaw < 90 || yaw > 270 {
            return "Yellow-Green";
        }
        if yaw < 180 {
            return "Yellow-Blue";
        }
        if yaw < 270 {
            return "Yellow-Pink";
        }
    }
    ""
}

/// What the coverage test found.
#[derive(Debug, Clone, PartialEq)]
pub struct Coverage {
    /// `pointshit`: of the twenty points on the sphere, how many have a sample near them.
    pub hits: usize,
    /// The points with none, in raw sample space: what `sphere1.AimFor` is given.
    pub aims: Vec<[f64; 3]>,
    /// `displayresult`: "more data needed Aim For" and the last missed point's colour, or empty.
    pub message: String,
}

/// `MathHelper.rad2deg`. `// C#: ExtLibs/Utilities/Math.cs:10`
const RAD2DEG: f64 = 180.0 / std::f64::consts::PI;

/// The coverage test: the mean radius of compass 1's samples about the running centre (summed in
/// single precision, as the C# sums it), then twenty points on a sphere of that radius - four
/// rings, 30 to 210 degrees from the top, five points round each, the first and last the same -
/// each hit when a sample lies within a third of the radius of it. `centre` is the running fit's
/// offsets, which are added to a sample. `// C#: MagCalib.cs:625-686`
#[must_use]
pub fn coverage(data: &[Sample], centre: [f64; 3]) -> Coverage {
    let add = |point: Sample, offset: [f64; 3]| {
        [
            f64::from(point[0]) + offset[0],
            f64::from(point[1]) + offset[1],
            f64::from(point[2]) + offset[2],
        ]
    };
    let length = |v: [f64; 3]| (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    let mut radius: f32 = 0.0;
    for point in data {
        #[allow(clippy::cast_possible_truncation)] // `(float)(point + centre).length()`
        let r = length(add(*point, centre)) as f32;
        radius += r;
    }
    #[allow(clippy::cast_precision_loss)] // `radius /= datacompass1.Count`, float by int
    let count = data.len() as f32;
    radius /= count;

    let factor = 3;
    let factor2 = 4;
    let max_distance = radius / 3.0;
    let mut hits = 0;
    let mut aims = Vec::new();
    let mut message = String::new();
    for j in 0..=factor {
        let theta = (std::f64::consts::PI * (f64::from(j) + 0.5)) / f64::from(factor);
        for i in 0..=factor2 {
            let phi = (2.0 * std::f64::consts::PI * f64::from(i)) / f64::from(factor2);
            let r = f64::from(radius);
            #[allow(clippy::cast_possible_truncation)] // `new Vector3((float) ..., ...)`
            let surface = [
                (theta.sin() * phi.cos() * r) as f32,
                (theta.sin() * phi.sin() * r) as f32,
                (theta.cos() * r) as f32,
            ];
            let point_sphere = [
                f64::from(surface[0]) - centre[0],
                f64::from(surface[1]) - centre[1],
                f64::from(surface[2]) - centre[2],
            ];
            let found = data.iter().any(|point| {
                let d = length([
                    point_sphere[0] - f64::from(point[0]),
                    point_sphere[1] - f64::from(point[1]),
                    point_sphere[2] - f64::from(point[2]),
                ]);
                d < f64::from(max_distance)
            });
            if found {
                hits += 1;
            } else {
                #[allow(clippy::cast_possible_truncation)] // `(int)(theta * MathHelper.rad2deg)`
                let (pitch, yaw) = ((theta * RAD2DEG) as i32, (phi * RAD2DEG) as i32);
                message = format!("{AIM_FOR}{}", get_colour(pitch, yaw));
                aims.push(point_sphere);
            }
        }
    }
    Coverage {
        hits,
        aims,
        message,
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::indexing_slicing, clippy::float_cmp)]
mod tests {
    use super::*;

    /// Points spread evenly over a sphere (a Fibonacci lattice) around `centre`, as a
    /// magnetometer's raw readings are when the vehicle is turned through every attitude, rounded
    /// to the integers `RAW_IMU` carries.
    fn readings(count: usize, centre: [f64; 3], radius: f64) -> Vec<[i16; 3]> {
        let golden = std::f64::consts::PI * (3.0 - 5.0_f64.sqrt());
        (0..count)
            .map(|i| {
                #[allow(clippy::cast_precision_loss)]
                let (i, n) = (i as f64, count as f64);
                let y = 1.0 - 2.0 * (i + 0.5) / n;
                let r = (1.0 - y * y).sqrt();
                let theta = golden * i;
                let u = [r * theta.cos(), y, r * theta.sin()];
                #[allow(clippy::cast_possible_truncation)]
                let s = |k: usize| (centre[k] + radius * u[k]).round() as i16;
                [s(0), s(1), s(2)]
            })
            .collect()
    }

    #[test]
    fn raw_imu_skips_a_zero_reading_before_the_filter_and_scaled_after() {
        let mut live = Live::default();
        assert!(!live.sample(Source::RawImu, [0, 0, 500]));
        // Not counted: three more in the same bucket are all kept.
        assert!(live.sample(Source::RawImu, [1, 1, 500]));
        assert!(live.sample(Source::RawImu, [2, 2, 500]));
        assert!(live.sample(Source::RawImu, [3, 3, 500]));
        assert!(!live.sample(Source::RawImu, [4, 4, 500]));
        assert_eq!(live.samples[0].len(), 3);
        // SCALED_IMU2: a zero counts in its bucket, so only two more of that bucket are kept.
        assert!(!live.sample(Source::ScaledImu2, [5, 0, 500]));
        assert!(live.sample(Source::ScaledImu2, [5, 1, 500]));
        assert!(live.sample(Source::ScaledImu2, [5, 2, 500]));
        assert!(!live.sample(Source::ScaledImu2, [5, 3, 500]));
        assert_eq!(live.samples[1].len(), 2);
        assert!(live.samples[2].is_empty());
    }

    #[test]
    fn begin_forgets_the_first_answer_but_not_the_others() {
        let mut live = Live {
            answers: [Some(vec![1.0]), Some(vec![2.0]), Some(vec![3.0])],
            errors: [0.1, 0.2, 0.3],
            ..Live::default()
        };
        live.sample(Source::RawImu, [100, 100, 100]);
        live.begin();
        assert_eq!(live.answers, [None, Some(vec![2.0]), Some(vec![3.0])]);
        assert_eq!(live.errors, [START_ERROR; 3]);
        assert!(live.samples[0].is_empty());
        // The filter was cleared too: the same reading is kept again.
        assert!(live.sample(Source::RawImu, [100, 100, 100]));
        live.forget_bad();
        assert_eq!(live.answers, [None, None, Some(vec![3.0])]);
        live.forget_all();
        assert_eq!(live.answers, [None, None, None]);
    }

    /// Readings on a sphere about a known point, through the filter: the running fit and the
    /// last fit both find the offsets - minus the centre - and the error falls from 99.
    #[test]
    fn the_samples_on_a_sphere_give_back_its_offsets() {
        let centre = [-120.0, 85.0, 230.0];
        let mut live = Live::default();
        let mut kept = 0;
        for reading in readings(600, centre, 450.0) {
            kept += usize::from(live.sample(Source::RawImu, reading));
        }
        for reading in readings(300, [40.0, -60.0, 10.0], 380.0) {
            live.sample(Source::ScaledImu2, reading);
        }
        assert_eq!(live.samples[0].len(), kept);
        assert!(kept > FIT_AFTER, "{kept}");
        let running = live.running_fit(0).unwrap();
        for k in 0..3 {
            assert!((running[k] + centre[k]).abs() < 2.0, "{running:?}");
        }
        assert!(live.errors[0] < START_ERROR);

        live.finish(true, false, true).unwrap();
        let first = live.answers[0].clone().unwrap();
        assert_eq!(first.len(), 9, "the ellipsoid's nine");
        for k in 0..3 {
            assert!((first[k] + centre[k]).abs() < 2.0, "{first:?}");
        }
        let second = live.answers[1].clone().unwrap();
        assert!((second[0] - -40.0).abs() < 2.0, "{second:?}");
        assert!((second[1] - 60.0).abs() < 2.0, "{second:?}");
        assert!(live.answers[2].is_none());
    }

    #[test]
    fn without_the_ellipsoid_the_answer_is_the_sphere() {
        let mut live = Live::default();
        for reading in readings(200, [10.0, 20.0, 30.0], 400.0) {
            live.sample(Source::RawImu, reading);
        }
        live.finish(false, false, false).unwrap();
        assert_eq!(live.answers[0].as_ref().map(Vec::len), Some(4));
    }

    #[test]
    fn under_ten_samples_is_not_enough_and_forgets_two_answers() {
        let mut live = Live::default();
        live.answers[2] = Some(vec![7.0; 4]);
        for reading in readings(9, [0.0; 3], 400.0) {
            live.sample(Source::RawImu, reading);
        }
        let error = live.finish(true, true, false).unwrap_err();
        assert_eq!(error.to_string(), "Log does not contain enough data");
        assert_eq!(live.answers, [None, None, Some(vec![7.0; 4])]);
    }

    #[test]
    fn the_status_is_the_csharps_text() {
        let mut live = Live::default();
        live.sample(Source::RawImu, [100, 100, 100]);
        assert_eq!(
            live.status(false, false, ""),
            "Got + 1 samples\nCompass 1 error: 99\n"
        );
        live.errors = [0.15, 1.41, f64::NAN];
        assert_eq!(
            live.status(true, true, "more data needed Aim For Yellow"),
            "Got + 1 samples\nCompass 1 error: 0.15\nCompass 2 error: 1.41\n\
             Compass 3 error: NaN\nmore data needed Aim For Yellow"
        );
    }

    #[test]
    fn the_colours_follow_get_colour() {
        assert_eq!(get_colour(0, 45), "DarkBlue");
        assert_eq!(get_colour(180, 45), "Yellow");
        assert_eq!(get_colour(30, 0), "DarkBlue-Red");
        assert_eq!(get_colour(30, 360), "DarkBlue-Red");
        assert_eq!(get_colour(30, 90), "DarkBlue-Blue");
        assert_eq!(get_colour(30, 180), "DarkBlue-Pink");
        assert_eq!(get_colour(30, 270), "");
        assert_eq!(get_colour(150, 0), "Yellow-Green");
        assert_eq!(get_colour(210, 90), "Yellow-Blue");
        assert_eq!(get_colour(89, 269), "DarkBlue-Pink");
    }

    /// A full sphere hits all twenty points; one hemisphere misses the others, and the message
    /// names the last point missed.
    #[test]
    fn the_coverage_test_counts_the_twenty_points() {
        let centre = [-50.0, 30.0, 100.0];
        let offsets = [50.0, -30.0, -100.0];
        let whole: Vec<Sample> = readings(800, centre, 400.0)
            .into_iter()
            .map(|r| r.map(f32::from))
            .collect();
        let full = coverage(&whole, offsets);
        assert_eq!(full.hits, 20);
        assert!(full.aims.is_empty());
        assert_eq!(full.message, "");

        // The top only (z above the centre): the 150- and 210-degree rings are missed.
        let top: Vec<Sample> = whole.iter().copied().filter(|p| p[2] > 180.0).collect();
        let half = coverage(&top, offsets);
        assert!(half.hits < 20 && half.hits >= 5, "{}", half.hits);
        assert_eq!(half.aims.len(), 20 - half.hits);
        assert!(half.message.starts_with(AIM_FOR), "{}", half.message);
        // Each aim point is in raw space: the centre plus a point on the sphere.
        let aim = half.aims[0];
        let r = ((aim[0] - centre[0]).powi(2) + (aim[1] - centre[1]).powi(2)
            + (aim[2] - centre[2]).powi(2))
        .sqrt();
        assert!((r - 400.0).abs() < 60.0, "{r}");
    }

    #[test]
    fn the_extremes_start_at_zero_so_the_sign_check_never_holds() {
        let mut extremes = Extremes::default();
        extremes.take([200.0, -50.0, 300.0]);
        extremes.take([250.0, -80.0, 310.0]);
        assert_eq!(extremes.min, [0.0, -80.0, 0.0]);
        assert_eq!(extremes.max, [250.0, 0.0, 310.0]);
        assert!(!extremes.bad());
        let odd = Extremes {
            min: [1.0, -5.0, -5.0],
            max: [3.0, 5.0, 5.0],
        };
        assert!(odd.bad());
    }

    #[test]
    fn set_sensor_offsets_is_the_csharps_command() {
        let MavMessage::CommandLong(long) =
            set_sensor_offsets(VehicleId::new(1, 1), SENSOR_SECOND_MAGNETOMETER, [1.5, -2.0, 3.0])
        else {
            panic!("not a COMMAND_LONG");
        };
        assert_eq!(long.command, 242);
        assert_eq!(
            [long.param1, long.param2, long.param3, long.param4, long.param5],
            [5.0, 1.5, -2.0, 3.0, 0.0]
        );
        assert_eq!((long.target_system, long.target_component), (1, 1));
    }

    #[test]
    fn a_cleared_list_keeps_its_capacity() {
        let mut samples = Samples::default();
        for i in 0..5 {
            samples.raw_imu([100 * (i + 1), 100, 100]);
        }
        assert_eq!(samples.capacity, 8);
        samples.clear();
        samples.raw_imu([100, 100, 100]);
        assert_eq!(samples.capacity, 8);
    }
}
