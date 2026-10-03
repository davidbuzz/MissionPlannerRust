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

//! What `CurrentState` works out over time, against its clock ([`crate::clock`]): the vertical
//! speed, the time in air, the distance travelled and what the battery says about it, the
//! simulator's speed-up and the low-airspeed warning - and the getters derived from them.
//!
//! Two kinds of update drive it, as in the C#. A message sets a property whose setter does
//! arithmetic against `datetime` - `alt`, `current` - which [`VehicleState::apply`] does as it
//! goes. And `UpdateCurrentSettings`, which the C# calls on every vehicle after each read from the
//! link (`MainV2.cs:3058-3069`) and after each packet of a log (`Log/MavlinkLog.cs:141-144`),
//! counts the seconds: [`VehicleState::update_current_settings`], which
//! [`crate::VehicleRegistry::update_current_settings`] calls on every vehicle.
//!
//! # Units
//!
//! As everywhere in this crate, what is held is SI. **Divergence:** the C# holds two of these in
//! the user's units instead: `distTraveled` adds each second's distance times `multiplierdist`
//! (`CurrentState.cs:4612-4613`), and `verticalspeed` differentiates the `alt` getter, which
//! applies `multiplieralt`, and then multiplies by `multiplierspeed` again when read
//! (`CurrentState.cs:1043-1051`). Both are the same numbers as here in metric units; in feet the
//! C#'s are in feet, and its vertical speed in feet per second times the speed multiplier.

use mp_mavlink_dialects::all::MavSysStatusSensor;
use mp_units::{LatLon, MetresPerSecond};

use crate::clock::DateTime;
use crate::state::VehicleState;

/// `MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE`, the airspeed sensor, which must be enabled and
/// healthy for the low-airspeed warning. `// C#: ExtLibs/Mavlink/Mavlink.cs:2867`
const DIFFERENTIAL_PRESSURE: u32 =
    MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE.0;

/// `-0.01f` as the `current` setter compares it with a double: the wire's -1, "no sensor", after
/// `/ 100.0f`. `// C#: ExtLibs/ArduPilot/CurrentState.cs:1403`
const NO_CURRENT_SENSOR: f64 = -0.01_f32 as f64;

impl VehicleState {
    /// `alt`: the altitude above home less the user's [`VehicleState::alt_offset_home`], metres.
    ///
    /// The single-precision value the C# holds (`_alt`), which `GLOBAL_POSITION_INT` and the
    /// high-latency messages set; [`VehicleState::altitude_relative`] is the same altitude in
    /// double precision, without the offset.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:327`
    #[must_use]
    pub fn alt(&self) -> f32 {
        self.alt_backing - self.alt_offset_home
    }

    /// The `alt` setter: the new altitude, and from it the vertical speed - and the climb rate,
    /// until a `VFR_HUD` has given a better one - once at least 0.2 s have passed on the clock
    /// and the altitude has changed, or at once if the clock has gone backwards.
    ///
    /// The vertical speed is filtered, four tenths of the old and six of the new, and reset to 0
    /// if that makes it infinite. The first update after start divides by the time since
    /// `DateTime.MinValue`, two thousand years, and so moves it by next to nothing.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:328-346, 1050`
    pub(crate) fn set_alt(&mut self, value: f32) {
        self.alt_backing = value;
        let alt = self.alt();
        let elapsed = self.datetime.seconds_since(self.last_alt);
        // C#: CurrentState.cs:333, `a && b || c`
        #[allow(clippy::float_cmp)] // the C#'s exact comparison
        let changed = self.old_alt != alt;
        if (elapsed >= 0.2 && changed) || self.last_alt > self.datetime {
            let rate = (alt - self.old_alt) / cast_f32(elapsed);
            // C#: CurrentState.cs:335-339
            if !self.got_vfr {
                self.climb_rate = MetresPerSecond(f64::from(rate));
            }
            // C#: CurrentState.cs:340-342, through the setter at 1050
            self.vertical_speed_backing = self.vertical_speed_backing * 0.4 + rate * 0.6;
            if self.vertical_speed_backing.is_infinite() {
                self.vertical_speed_backing = 0.0;
            }
            self.last_alt = self.datetime;
            self.old_alt = alt;
        }
    }

    /// `verticalspeed`, metres per second, positive up: the altitude's rate of change, filtered;
    /// 0 rather than NaN.
    ///
    /// **Divergence:** the C#'s getter also writes the 0 back when it finds NaN, so the filter
    /// restarts from it; a snapshot is read-only, so here the filter would keep a NaN - `0.4 *
    /// NaN` being NaN. It cannot arise from the wire: the setter divides by at least 0.2 s, or by
    /// a negative time, never by zero, and a MAVLink altitude is an integer.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:1043-1051`
    #[must_use]
    pub fn vertical_speed(&self) -> f32 {
        if self.vertical_speed_backing.is_nan() {
            0.0
        } else {
            self.vertical_speed_backing
        }
    }

    /// The `current` setter on the first battery: the current, and the capacity it has used,
    /// integrated over the clock since the last reading.
    ///
    /// The first reading only starts the clock. The wire's "no sensor" makes the current 0 and
    /// leaves the clock where it was, so the next real reading is integrated over the gap.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:1396-1415`
    pub(crate) fn set_current(&mut self, value: f64) {
        // C#: CurrentState.cs:1401
        if self.last_current == DateTime::MIN {
            self.last_current = self.datetime;
        }
        // C#: CurrentState.cs:1402-1407
        #[allow(clippy::float_cmp)] // the C#'s exact comparison
        let no_sensor = value == NO_CURRENT_SENSOR;
        if no_sensor {
            self.battery.current = 0.0;
            return;
        }
        // C#: CurrentState.cs:1409-1411
        self.battery_used_mah += value * 1000.0 * self.datetime.hours_since(self.last_current);
        self.battery.current = cast_f32(value);
        self.last_current = self.datetime;
    }

    /// `UpdateCurrentSettings`' once-a-second work, on the first call in each new second of
    /// [`VehicleState::datetime`]: the distance travelled while armed on a 3D fix, and a second of
    /// time in air while armed with the throttle over 12% or the ground speed over 3 m/s.
    ///
    /// `link_closed` is the C#'s `BaseStream != null && !BaseStream.IsOpen && !logreadmode`: a
    /// link that is shut and not replaying, on which the distance restarts from 0 - at the first
    /// second that would add to it. The link thread and a replay pass false.
    ///
    /// The C#'s 50 ms rate limit around this (`CurrentState.cs:4584`) is its callers' cadence,
    /// and the rest of the method belongs to the link - the stream requests, which read
    /// [`VehicleState::rates`] - or is its own `dowindcalc` estimate, which is not ported (see
    /// [`VehicleState::wind_speed`]).
    ///
    /// The seconds are counted by the clock's seconds field, not by elapsed time: a gap of a
    /// minute to the second counts one. **Divergence:** the C# starts its counter at the wall
    /// clock when the state is made (`CurrentState.cs:128`), which in a replay is unrelated to
    /// the log's time and decides at random whether the first second counts;
    /// [`crate::VehicleRegistry::apply_at`] starts it at the first packet's time, which on a live
    /// link is the same instant.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:4580-4627`
    pub fn update_current_settings(&mut self, link_closed: bool) {
        // C#: CurrentState.cs:4602-4604
        if self.datetime.second() == self.last_second_counter.second() {
            return;
        }
        self.last_second_counter = self.datetime;

        // C#: CurrentState.cs:4606-4619. `lastpos` and `lat`, `lng` must each be non-zero; the
        // position is `None` for the C#'s (0, 0).
        let nonzero = |p: &LatLon| p.latitude() != 0.0 && p.longitude() != 0.0;
        if let (Some(last), Some(here)) = (self.last_pos, self.position)
            && nonzero(&last)
            && self.armed
            && nonzero(&here)
            && self.gps.fix_type >= 3
        {
            if link_closed {
                self.dist_traveled = 0.0;
            }
            self.dist_traveled += cast_f32(last.distance_to(here).0);
        }
        self.last_pos = self.position;

        // C#: CurrentState.cs:4621-4626. `ch3percent`, from VFR_HUD. **Divergence:** before the
        // first one the C# works a percentage out of servo 3 and the SERVO3_* parameters instead
        // (CurrentState.cs:971-1003), which are not held here, so this reads 0 until then.
        if (f32::from(self.throttle_percent) > 12.0 || self.ground_speed.0 > 3.0) && self.armed {
            self.time_in_air += 1.0;
            self.time_since_arm_in_air += 1.0;
        }
    }

    /// `timeInAirMinSec`: the time in air as minutes, and seconds in hundredths - 61 s is 1.01.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:1195-1197`
    #[must_use]
    pub fn time_in_air_min_sec(&self) -> f32 {
        let minutes = truncate_i32(self.time_in_air / 60.0);
        i32_f32(minutes) + self.time_in_air % 60.0 / 100.0
    }

    /// `battery_mahperkm`: capacity used per kilometre travelled. Infinite before any distance
    /// with some capacity used, and NaN with neither, as the C#'s unguarded division is.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:1471-1474`
    #[must_use]
    pub fn battery_mah_per_km(&self) -> f64 {
        self.battery_used_mah / f64::from(self.dist_traveled / 1000.0)
    }

    /// `battery_kmleft`: kilometres left at [`VehicleState::battery_mah_per_km`], from the
    /// capacity used and the percentage remaining. Unguarded like it: infinite at 100% remaining
    /// with some capacity used and some distance travelled.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:1476-1480`
    #[must_use]
    pub fn battery_km_left(&self) -> f64 {
        let remaining = f32::from(self.battery.remaining_percent);
        (f64::from(100.0 / (100.0 - remaining)) * self.battery_used_mah - self.battery_used_mah)
            / self.battery_mah_per_km()
    }

    /// `DistFromMovingBase`: metres from [`VehicleState::base`] to the vehicle on the C#'s flat
    /// projection, 111319.5 m a degree with longitude scaled by the cosine of the base's latitude;
    /// 0 without a position.
    ///
    /// Not 0 without a base: the C# checks for a null base, which its property never returns - an
    /// unset base is (0, 0) - so until one is set this is the distance from 0° 0°.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:1786-1805`
    #[must_use]
    pub fn dist_from_moving_base(&self) -> f32 {
        let Some(here) = self.position else {
            return 0.0;
        };
        let base = self.base;
        // C#: CurrentState.cs:1796-1803
        let rads = base.lat.abs() * 0.017_453_292_5;
        let scale_long_down = rads.cos();
        let dstlat = (base.lat - here.latitude()).abs() * 111_319.5;
        let dstlon = (base.lng - here.longitude()).abs() * 111_319.5 * scale_long_down;
        cast_f32((dstlat * dstlat + dstlon * dstlon).sqrt())
    }

    /// `gimballat`: the latitude the camera is pointed at, in single precision; 0 until the
    /// flight screen has projected one ([`VehicleState::gimbal_point`]).
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:2024-2032`
    #[must_use]
    pub fn gimbal_lat(&self) -> f32 {
        self.gimbal_point.map_or(0.0, |point| cast_f32(point.lat))
    }

    /// `gimballng`, likewise. `// C#: ExtLibs/ArduPilot/CurrentState.cs:2034-2042`
    #[must_use]
    pub fn gimbal_lng(&self) -> f32 {
        self.gimbal_point.map_or(0.0, |point| cast_f32(point.lng))
    }

    /// `timesincelastshot` as the flight screen works it out from the camera's feedback, seconds:
    /// the last shot's time less the one before's, or `double.MinValue` taken from the only one
    /// (which is `f64::MAX`); `None` with no shots, which leaves the field as it was.
    ///
    /// `camera_times_usec` is `CAMERA_FEEDBACK.time_usec` of each shot in `MAV.camerapoints`'
    /// order (`MAVLinkInterface.cs:5736-5745`); whoever keeps that list sets
    /// [`VehicleState::time_since_last_shot`] from this each time the map redraws.
    /// `// C#: GCSViews/FlightData.cs:4021-4038`
    #[must_use]
    #[allow(clippy::cast_precision_loss)] // `mark.time_usec / 1000.0`, a ulong in a double
    pub fn shot_interval(camera_times_usec: impl IntoIterator<Item = u64>) -> Option<f64> {
        let mut interval = None;
        let mut old_time = f64::MIN;
        for time in camera_times_usec {
            let seconds = (time as f64 / 1000.0) / 1000.0;
            interval = Some(seconds - old_time);
            old_time = seconds;
        }
        interval
    }

    /// Gives the low-airspeed warning the vehicle's `AIRSPEED_MIN` parameter, or failing that its
    /// `ARSPD_FBW_MIN` - the C# reads `parent.param` for them itself. The owner of the parameter
    /// table calls this when either is fetched or changes; the warning takes the value up the
    /// next time `VFR_HUD` checks, at most every five seconds in the air, as the C# does.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:3858-3880`
    pub fn set_airspeed_min_params(
        &mut self,
        airspeed_min: Option<f64>,
        arspd_fbw_min: Option<f64>,
    ) {
        self.airspeed_min_param = airspeed_min.or(arspd_fbw_min).map(cast_f32);
    }

    /// `lowairspeed`, as `VFR_HUD` sets it: armed, in the air since arming, with an airspeed
    /// sensor enabled and healthy, below the minimum airspeed parameter.
    ///
    /// The parameter is re-read from [`VehicleState::set_airspeed_min_params`] at most every five
    /// seconds and only in the air, and a vehicle without either parameter keeps whatever minimum
    /// was last read - 0 at first, which never warns. **Divergence:** the C# times the five
    /// seconds on the wall clock; this times them on [`VehicleState::datetime`], which on a live
    /// link is the wall clock and in a replay is the flight's.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:3858-3888`
    pub(crate) fn check_low_airspeed(&mut self, airspeed: f32) {
        if self.time_since_arm_in_air > 0.0
            && self.datetime.seconds_since(self.last_airspeed_min_check) > 5.0
        {
            if let Some(minimum) = self.airspeed_min_param {
                self.cached_airspeed_min = minimum;
            }
            self.last_airspeed_min_check = self.datetime;
        }
        let sensors = &self.sensors;
        self.low_airspeed = self.armed
            && self.time_since_arm_in_air > 0.0
            && self.cached_airspeed_min > 0.0
            && sensors.enabled & DIFFERENTIAL_PRESSURE != 0
            && sensors.health & DIFFERENTIAL_PRESSURE != 0
            && airspeed < self.cached_airspeed_min;
    }

    /// `speedup`, from `RAW_IMU`'s clock: how fast the vehicle's time runs against ours, a
    /// simulator's speed-up, filtered 95% old to 5% new.
    ///
    /// Only readings that move the IMU clock forward by less than ten seconds count, and the
    /// first one the C# takes is measured from its `imutime` of 0 - so it starts counting only if
    /// it first hears the vehicle within ten seconds of boot, and never again after the vehicle
    /// reboots, since the IMU clock then goes backwards from what it last took. The first reading
    /// it does take is measured against `DateTime.MinValue` and adds next to nothing.
    ///
    /// **Divergence:** the C# measures against `DateTime.Now`; this measures against
    /// [`VehicleState::datetime`], which on a live link is `DateTime.Now` as the packet was read
    /// (`MAVLinkInterface.cs:4721`), and in a replay is the time the packet was recorded - where
    /// the C#'s figure is how fast the file is being read rather than anything about the vehicle.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:3699-3713`
    #[allow(clippy::cast_precision_loss)] // `imu.time_usec * 1.0e-6`, a ulong in a double
    pub(crate) fn update_speedup(&mut self, time_usec: u64) {
        let time = time_usec as f64 * 1.0e-6;
        let delta_wall = self.datetime.seconds_since(self.last_imu_time);
        let delta_imu = time - self.imu_time;
        if delta_imu > 0.0 && delta_imu < 10.0 {
            self.speedup = cast_f32(f64::from(self.speedup) * 0.95 + delta_imu / delta_wall * 0.05);
            self.imu_time = time;
            self.last_imu_time = self.datetime;
        }
    }
}

/// A double narrowed to the single precision a C# `float` holds.
#[allow(clippy::cast_possible_truncation)]
const fn cast_f32(value: f64) -> f32 {
    value as f32
}

/// An `int` in a C# `float` expression.
#[allow(clippy::cast_precision_loss)]
const fn i32_f32(value: i32) -> f32 {
    value as f32
}

/// C#'s `(int)` of a `float`: toward zero; saturating here where the C# is unspecified.
#[allow(clippy::cast_possible_truncation)]
const fn truncate_i32(value: f32) -> i32 {
    value as i32
}
