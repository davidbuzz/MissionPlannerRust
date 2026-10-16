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

//! `Spline2`: ArduCopter's spline waypoint controller as Mission Planner carries it, which
//! `WPOverlay.RegenerateWPRoute` steps along to draw the curve through spline waypoints.
//!
//! Ported from `ExtLibs/Utilities/Spline2.cs` (GPL-3.0-only): the position vector conversions,
//! `set_wp_origin_and_destination`, `set_spline_destination` and the spline solution and its
//! stepping, each with the C#'s mix of `float` and `double` - the controller's times, speeds and
//! lengths are `float`, its vectors `Vector3`'s `double`s, and a location comes back through
//! `int` 1e-7 degrees multiplied by a `float`, so a sampled point has a `float`'s precision. The
//! one method nothing calls, `update_spline`, is not ported (ledger/dead-functions.csv), nor is
//! the heading `advance_spline_target_along_track` works out and nothing reads (`_yaw`).

use std::ops::{Add, Mul, Neg, Sub};

/// `MathHelper.rad2deg`. `// C#: ExtLibs/Utilities/Math.cs:10`
const RAD2DEG: f64 = 180.0 / std::f64::consts::PI;

/// `MathHelper.deg2rad`: `1.0 / rad2deg`. `// C#: ExtLibs/Utilities/Math.cs:11`
const DEG2RAD: f64 = 1.0 / RAD2DEG;

/// `LATLON_TO_CM = 1.113195f`: centimetres in 1e-7 of a degree of latitude, a `float` the vector
/// arithmetic reads as a `double`. `// C#: ExtLibs/Utilities/Spline2.cs:101`
const LATLON_TO_CM: f64 = 1.113_195_f32 as f64;

/// `_wp_accel_cms`'s default, 1 m/s/s. `// C#: ExtLibs/Utilities/Spline2.cs:56`
pub const DEFAULT_ACCEL_CMS: f32 = 100.0;

/// `_wp_speed_cms`'s default, 6 m/s. `// C#: ExtLibs/Utilities/Spline2.cs:58`
pub const DEFAULT_SPEED_CMS: f32 = 600.0;

/// `Vector3`, the `double` one: what the controller's positions and velocities are, in
/// centimetres from home - north, east and up.
/// `// C#: ExtLibs/Utilities/Vector3.cs:19-32, 196-255`
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Vector3 {
    /// `x`: north, centimetres.
    pub x: f64,
    /// `y`: east, centimetres.
    pub y: f64,
    /// `z`: up, centimetres.
    pub z: f64,
}

impl Vector3 {
    /// `new Vector3(x, y, z)`.
    #[must_use]
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    /// `length()`: `Sqrt(x * x + y * y + z * z)`.
    #[must_use]
    pub fn length(self) -> f64 {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }
}

impl Add for Vector3 {
    type Output = Self;

    fn add(self, v: Self) -> Self {
        Self::new(self.x + v.x, self.y + v.y, self.z + v.z)
    }
}

impl Sub for Vector3 {
    type Output = Self;

    fn sub(self, v: Self) -> Self {
        Self::new(self.x - v.x, self.y - v.y, self.z - v.z)
    }
}

impl Neg for Vector3 {
    type Output = Self;

    fn neg(self) -> Self {
        Self::new(-self.x, -self.y, -self.z)
    }
}

/// `operator *(Vector3<T> self, double v)`: a `float` operand is widened first, as C# widens it.
impl Mul<f64> for Vector3 {
    type Output = Self;

    fn mul(self, v: f64) -> Self {
        Self::new(self.x * v, self.y * v, self.z * v)
    }
}

/// `spline_segment_end_type`: how the vehicle leaves a spline segment's destination.
/// `// C#: ExtLibs/Utilities/Spline2.cs:14-19`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SegmentEnd {
    /// `SEGMENT_END_STOP`: it stops there.
    Stop,
    /// `SEGMENT_END_STRAIGHT`: a straight segment follows.
    Straight,
    /// `SEGMENT_END_SPLINE`: another spline segment follows.
    Spline,
}

/// `SegmentType`: whether the active segment is straight or a spline.
/// `// C#: ExtLibs/Utilities/Spline2.cs:22-26`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SegmentType {
    Straight,
    Spline,
}

/// `Spline2`: one controller, its origin the home it was made with.
/// `// C#: ExtLibs/Utilities/Spline2.cs:8-408`
#[derive(Debug, Clone)]
pub struct Spline2 {
    /// `target_pos`: where the last step put the intermediate target.
    pub target_pos: Vector3,
    /// `target_vel`.
    target_vel: Vector3,
    /// `_flags.reached_destination`.
    reached_destination: bool,
    /// `_flags.fast_waypoint`.
    fast_waypoint: bool,
    /// `_flags.segment_type`.
    segment_type: SegmentType,
    /// `_spline_time`, a `float`.
    spline_time: f32,
    /// `_spline_origin_vel`.
    spline_origin_vel: Vector3,
    /// `_spline_destination_vel`.
    spline_destination_vel: Vector3,
    /// `_hermite_spline_solution`.
    hermite: [Vector3; 4],
    /// `_spline_vel_scaler`, a `float`.
    spline_vel_scaler: f32,
    /// `_slow_down_dist`, a `float`.
    slow_down_dist: f32,
    /// `_wp_accel_cms`, static in the C#.
    wp_accel_cms: f32,
    /// `_wp_speed_cms`, static in the C#.
    wp_speed_cms: f32,
    /// `_origin`.
    origin: Vector3,
    /// `_destination`.
    destination: Vector3,
    /// `scaleLongDown`: a `float` cosine kept in a `double`.
    scale_long_down: f64,
    /// `scaleLongUp`.
    scale_long_up: f64,
    /// `homeLocation`'s latitude and longitude.
    home: (f64, f64),
    /// `_track_length`.
    track_length: f64,
}

impl Spline2 {
    /// `new Spline2(homeLocation)`, with `_wp_accel_cms` and `_wp_speed_cms` as given - their
    /// defaults [`DEFAULT_ACCEL_CMS`] and [`DEFAULT_SPEED_CMS`] until `processToScreen` has read a
    /// vehicle's WPNAV_ACCEL and WPNAV_SPEED (`GCSViews/FlightPlanner.cs:5565-5586`).
    /// `// C#: ExtLibs/Utilities/Spline2.cs:105-108`
    #[must_use]
    pub const fn new(home_lat: f64, home_lng: f64, accel_cms: f32, speed_cms: f32) -> Self {
        let zero = Vector3::new(0.0, 0.0, 0.0);
        Self {
            target_pos: zero,
            target_vel: zero,
            reached_destination: false,
            fast_waypoint: false,
            segment_type: SegmentType::Straight,
            spline_time: 0.0,
            spline_origin_vel: zero,
            spline_destination_vel: zero,
            hermite: [zero; 4],
            spline_vel_scaler: 0.0,
            slow_down_dist: 0.0,
            wp_accel_cms: accel_cms,
            wp_speed_cms: speed_cms,
            origin: zero,
            destination: zero,
            scale_long_down: 0.0,
            scale_long_up: 0.0,
            home: (home_lat, home_lng),
            track_length: 0.0,
        }
    }

    /// `longitude_scale(loc)`: `(float) Math.Cos(loc.Lat * deg2rad)`.
    /// `// C#: ExtLibs/Utilities/Spline2.cs:110-115`
    #[allow(clippy::cast_possible_truncation)] // the C#'s `(float)`
    fn longitude_scale(lat: f64) -> f32 {
        (lat * DEG2RAD).cos() as f32
    }

    /// `pv_location_to_vector(loc)`: a location as centimetres north, east and up of home, the
    /// east scaled by the cosine of home's latitude. Sets `scaleLongDown` and `scaleLongUp`.
    /// `// C#: ExtLibs/Utilities/Spline2.cs:68-78`
    pub fn pv_location_to_vector(&mut self, lat: f64, lng: f64, alt: f64) -> Vector3 {
        let (home_lat, home_lng) = self.home;
        self.scale_long_down = f64::from(Self::longitude_scale(home_lat));
        // `1.0f/scaleLongDown`: a `float` over a `double` is a `double` division.
        self.scale_long_up = 1.0 / self.scale_long_down;
        Vector3::new(
            (lat * 1.0e7 - home_lat * 1.0e7) * LATLON_TO_CM,
            (lng * 1.0e7 - home_lng * 1.0e7) * LATLON_TO_CM * self.scale_long_down,
            alt * 100.0,
        )
    }

    /// `pv_vector_to_location(loc)`: back to a location - the latitude and longitude each an `int`
    /// of 1e-7 degrees times `1.0e-7f`, so a `float`, and the altitude `z / 100`.
    /// `// C#: ExtLibs/Utilities/Spline2.cs:80-83`
    #[must_use]
    pub fn pv_vector_to_location(&self, v: Vector3) -> (f64, f64, f64) {
        // `int * float` is a `float` multiplication in C#.
        #[allow(clippy::cast_precision_loss)] // the C#'s int-to-float conversion
        let lat = self.pv_get_lat(v) as f32 * 1.0e-7_f32;
        #[allow(clippy::cast_precision_loss)] // the C#'s int-to-float conversion
        let lng = self.pv_get_lon(v) as f32 * 1.0e-7_f32;
        (f64::from(lat), f64::from(lng), v.z / 100.0)
    }

    /// `pv_get_lat`: `(int32_t) (home.Lat * 1.0e7f) + (int32_t) (pos_vec.x / LATLON_TO_CM)`, the
    /// sum unchecked as C#'s is.
    /// `// C#: ExtLibs/Utilities/Spline2.cs:86-91`
    #[allow(clippy::cast_possible_truncation)] // the C#'s `(int32_t)`
    fn pv_get_lat(&self, v: Vector3) -> i32 {
        let home = (self.home.0 * 1.0e7) as i32;
        home.wrapping_add((v.x / LATLON_TO_CM) as i32)
    }

    /// `pv_get_lon`: `(int32_t) (home.Lng * 1.0e7f) + (int32_t) (pos_vec.y / LATLON_TO_CM *
    /// scaleLongUp)`.
    /// `// C#: ExtLibs/Utilities/Spline2.cs:94-99`
    #[allow(clippy::cast_possible_truncation)] // the C#'s `(int32_t)`
    fn pv_get_lon(&self, v: Vector3) -> i32 {
        let home = (self.home.1 * 1.0e7) as i32;
        home.wrapping_add((v.y / LATLON_TO_CM * self.scale_long_up) as i32)
    }

    /// `_flags.reached_destination`.
    #[must_use]
    pub const fn reached_destination(&self) -> bool {
        self.reached_destination
    }

    /// `_flags.reached_destination = ...`, which `RegenerateWPRoute` sets once the straight
    /// segment before the splines is given, so the first spline starts from its end.
    pub const fn set_reached_destination(&mut self, reached: bool) {
        self.reached_destination = reached;
    }

    /// `_track_length`, which `set_wp_origin_and_destination` measures.
    #[must_use]
    pub const fn track_length(&self) -> f64 {
        self.track_length
    }

    /// `set_wp_origin_and_destination(origin, destination)`: a straight segment.
    /// `// C#: ExtLibs/Utilities/Spline2.cs:123-135`
    pub fn set_wp_origin_and_destination(&mut self, origin: Vector3, destination: Vector3) {
        self.origin = origin;
        self.destination = destination;
        let pos_delta = self.destination - self.origin;
        self.track_length = pos_delta.length();
        self.reached_destination = false;
        self.fast_waypoint = false;
        self.segment_type = SegmentType::Straight;
    }

    /// `set_spline_destination(destination, stopped_at_start, seg_end_type, next_destination)`:
    /// the next spline segment, from the last destination once it has been reached.
    /// `// C#: ExtLibs/Utilities/Spline2.cs:139-162`
    pub fn set_spline_destination(
        &mut self,
        destination: Vector3,
        stopped_at_start: bool,
        seg_end_type: SegmentEnd,
        next_destination: Vector3,
    ) {
        let origin = if self.reached_destination {
            self.destination
        } else {
            // "my edit"
            self.origin
        };
        self.set_spline_origin_and_destination(
            origin,
            destination,
            stopped_at_start,
            seg_end_type,
            next_destination,
        );
    }

    /// `set_spline_origin_and_destination`: the velocities at either end and the Hermite solution
    /// between them, scaled down where the two velocities together are more than four times the
    /// distance.
    /// `// C#: ExtLibs/Utilities/Spline2.cs:166-274`
    #[allow(clippy::cast_possible_truncation)] // the C#'s `(float)` lengths
    fn set_spline_origin_and_destination(
        &mut self,
        origin: Vector3,
        destination: Vector3,
        stopped_at_start: bool,
        seg_end_type: SegmentEnd,
        next_destination: Vector3,
    ) {
        let prev_segment_exists = self.reached_destination;
        // `0.1f`, widened.
        let tenth = f64::from(0.1_f32);
        if stopped_at_start || !prev_segment_exists {
            self.spline_origin_vel = (destination - origin) * tenth;
            self.spline_time = 0.0;
            self.spline_vel_scaler = 0.0;
        } else if self.segment_type == SegmentType::Straight {
            self.spline_origin_vel = self.destination - self.origin;
            self.spline_time = 0.0;
            self.spline_vel_scaler = 0.0;
        } else {
            self.spline_origin_vel = self.spline_destination_vel;
            if self.spline_time > 1.0 && self.spline_time < 1.1_f32 {
                self.spline_time -= 1.0;
            } else {
                self.spline_time = 0.0;
            }
            self.spline_vel_scaler = 0.0;
        }

        match seg_end_type {
            SegmentEnd::Stop => {
                self.spline_destination_vel = (destination - origin) * tenth;
                self.fast_waypoint = false;
            }
            SegmentEnd::Straight => {
                self.spline_destination_vel = next_destination - destination;
                self.fast_waypoint = true;
            }
            SegmentEnd::Spline => {
                self.spline_destination_vel = next_destination - origin;
                self.fast_waypoint = true;
            }
        }

        // "code below ensures we don't get too much overshoot when the next segment is short"
        let vel_len = (self.spline_origin_vel + self.spline_destination_vel).length() as f32;
        let pos_len = (destination - origin).length() as f32 * 4.0;
        if vel_len > pos_len {
            let vel_scaling = f64::from(pos_len / vel_len);
            let (origin_vel, destination_vel) = (
                self.spline_origin_vel * vel_scaling,
                self.spline_destination_vel * vel_scaling,
            );
            self.update_spline_solution(origin, destination, origin_vel, destination_vel);
        } else {
            let (origin_vel, destination_vel) =
                (self.spline_origin_vel, self.spline_destination_vel);
            self.update_spline_solution(origin, destination, origin_vel, destination_vel);
        }

        self.origin = origin;
        self.destination = destination;
        self.calc_slow_down_distance(self.wp_speed_cms, self.wp_accel_cms);
        self.reached_destination = false;
        self.segment_type = SegmentType::Spline;
    }

    /// `calc_slow_down_distance(speed_cms, accel_cmss)`: `speed^2 / (4 accel)`, 0 with no
    /// acceleration.
    /// `// C#: ExtLibs/Utilities/Spline2.cs:277-288`
    fn calc_slow_down_distance(&mut self, speed_cms: f32, accel_cmss: f32) {
        if accel_cmss <= 0.0 {
            self.slow_down_dist = 0.0;
            return;
        }
        self.slow_down_dist = speed_cms * speed_cms / (4.0 * accel_cmss);
    }

    /// `update_spline_solution`: the cubic's coefficients, in the C#'s order of operations.
    /// `// C#: ExtLibs/Utilities/Spline2.cs:326-332`
    fn update_spline_solution(
        &mut self,
        origin: Vector3,
        dest: Vector3,
        origin_vel: Vector3,
        dest_vel: Vector3,
    ) {
        self.hermite = [
            origin,
            origin_vel,
            -origin * 3.0 - origin_vel * 2.0 + dest * 3.0 - dest_vel,
            origin * 2.0 + origin_vel - dest * 2.0 + dest_vel,
        ];
    }

    /// `advance_spline_target_along_track(dt)`: the target where the spline's time is, then the
    /// time moved on by the speed - which grows by the acceleration each step to the speed limit,
    /// or slows within the stopping distance of a destination it stops at - over the spline's own
    /// speed there; the destination is reached once the time is 1.
    /// `// C#: ExtLibs/Utilities/Spline2.cs:334-384`
    #[allow(clippy::cast_possible_truncation)] // the C#'s `(float)` casts
    pub fn advance_spline_target_along_track(&mut self, dt: f32) {
        if self.reached_destination {
            return;
        }
        let (position, velocity) = self.calc_spline_pos_vel(self.spline_time);
        self.target_pos = position;
        self.target_vel = velocity;

        let spline_dist_to_wp = (self.destination - self.target_pos).length() as f32;
        if !self.fast_waypoint && spline_dist_to_wp < self.slow_down_dist {
            self.spline_vel_scaler =
                f64::from(spline_dist_to_wp * 2.0 * self.wp_accel_cms).sqrt() as f32;
        } else if self.spline_vel_scaler < self.wp_speed_cms {
            self.spline_vel_scaler += self.wp_accel_cms * dt;
        }
        if self.spline_vel_scaler > self.wp_speed_cms {
            self.spline_vel_scaler = self.wp_speed_cms;
        }

        let spline_time_scale =
            (f64::from(self.spline_vel_scaler) / self.target_vel.length()) as f32;
        self.spline_time += spline_time_scale * dt;
        // "we will reach the next waypoint in the next step so set reached_destination flag"
        if self.spline_time >= 1.0 {
            self.reached_destination = true;
        }
    }

    /// `calc_spline_pos_vel(spline_time)`: the cubic and its derivative at a time, the time's
    /// square and cube in `float`.
    /// `// C#: ExtLibs/Utilities/Spline2.cs:392-405`
    fn calc_spline_pos_vel(&self, spline_time: f32) -> (Vector3, Vector3) {
        let spline_time_sqrd = spline_time * spline_time;
        let spline_time_cubed = spline_time_sqrd * spline_time;
        let [h0, h1, h2, h3] = self.hermite;
        let (t, t2, t3) = (
            f64::from(spline_time),
            f64::from(spline_time_sqrd),
            f64::from(spline_time_cubed),
        );
        let position = h0 + h1 * t + h2 * t2 + h3 * t3;
        let velocity = h1 + h2 * 2.0 * t + h3 * 3.0 * t2;
        (position, velocity)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Home in Canberra, as the planner's tests have it.
    const HOME: (f64, f64) = (-35.363_262_1, 149.165_237_4);

    fn spline() -> Spline2 {
        Spline2::new(HOME.0, HOME.1, DEFAULT_ACCEL_CMS, DEFAULT_SPEED_CMS)
    }

    /// A location goes to centimetres from home and back: north by `LATLON_TO_CM` a 1e-7 degree,
    /// east by that and home's cosine, up by 100 a metre; back through whole 1e-7 degrees and a
    /// `float`.
    #[test]
    #[allow(clippy::cast_possible_truncation)] // the C#'s `(float)` and `(int)` casts
    fn a_location_goes_to_centimetres_from_home_and_back() {
        let mut sp = spline();
        let v = sp.pv_location_to_vector(HOME.0 + 0.001, HOME.1 + 0.002, 50.0);
        let l = f64::from(1.113_195_f32);
        let cos = f64::from((HOME.0 * (1.0 / (180.0 / std::f64::consts::PI))).cos() as f32);
        assert!((v.x - 10_000.0 * l).abs() < 1e-6, "{}", v.x);
        assert!((v.y - 20_000.0 * l * cos).abs() < 1e-6, "{}", v.y);
        assert!((v.z - 5_000.0).abs() < 1e-9);
        let (lat, lng, alt) = sp.pv_vector_to_location(v);
        // Whole 1e-7 degrees, then a float: within a float's step at these magnitudes.
        let lat_float = f64::from((HOME.0 + 0.001) as f32);
        assert!((lat - lat_float).abs() < 1e-5, "{lat}");
        assert!((lng - (HOME.1 + 0.002)).abs() < 2e-5, "{lng}");
        assert!((alt - 50.0).abs() < 1e-9);
        // The latitude is exactly an int of 1e-7 degrees times 1e-7f, as a float.
        #[allow(clippy::cast_possible_truncation)]
        let ints = (HOME.0 * 1e7) as i32 + (v.x / l) as i32;
        #[allow(clippy::cast_precision_loss)]
        let expected = f64::from(ints as f32 * 1.0e-7_f32);
        assert_eq!(lat, expected);
    }

    /// The vector arithmetic is componentwise and `-a * b` is `(-a) * b`, as in the C#.
    #[test]
    fn vectors_add_subtract_negate_and_scale() {
        let a = Vector3::new(1.0, 2.0, 3.0);
        let b = Vector3::new(0.5, -1.0, 4.0);
        assert_eq!(a + b, Vector3::new(1.5, 1.0, 7.0));
        assert_eq!(a - b, Vector3::new(0.5, 3.0, -1.0));
        assert_eq!(-a * 3.0, Vector3::new(-3.0, -6.0, -9.0));
        assert!((Vector3::new(3.0, 4.0, 12.0).length() - 13.0).abs() < 1e-12);
    }

    /// The Hermite cubic `p(t) = (2t³ - 3t² + 1) P0 + (t³ - 2t² + t) V0 + (-2t³ + 3t²) P1 +
    /// (t³ - t²) V1`, written apart from the C#'s coefficient form, to hold the port's solution to.
    fn hermite(p0: Vector3, v0: Vector3, p1: Vector3, v1: Vector3, t: f64) -> Vector3 {
        let (t2, t3) = (t * t, t * t * t);
        p0 * (2.0 * t3 - 3.0 * t2 + 1.0)
            + v0 * (t3 - 2.0 * t2 + t)
            + p1 * (-2.0 * t3 + 3.0 * t2)
            + v1 * (t3 - t2)
    }

    /// The first spline segment after a straight one: its origin velocity is the straight
    /// segment's direction, its destination velocity the next segment's, scaled down where the two
    /// are more than four times the distance; each step is a point on the Hermite cubic at the
    /// time the step began, and moves the time on by a speed of 100, 200, ... 600 cm/s over the
    /// cubic's own speed there, until the time is 1.
    #[test]
    #[allow(clippy::cast_possible_truncation)] // the C#'s `(float)` lengths
    fn a_spline_segment_is_sampled_on_the_hermite_cubic_as_its_speed_grows() {
        let mut sp = spline();
        let before = sp.pv_location_to_vector(HOME.0, HOME.1, 100.0);
        let start = sp.pv_location_to_vector(HOME.0 + 0.0005, HOME.1, 100.0);
        let end = sp.pv_location_to_vector(HOME.0 + 0.0005, HOME.1 + 0.0008, 100.0);
        let next = sp.pv_location_to_vector(HOME.0 + 0.0010, HOME.1 + 0.0008, 100.0);
        sp.set_wp_origin_and_destination(before, start);
        sp.set_reached_destination(true);
        sp.set_spline_destination(end, false, SegmentEnd::Straight, next);

        // The velocities, before the overshoot scaling.
        let v0 = start - before;
        let v1 = next - end;
        let vel_len = (v0 + v1).length() as f32;
        let pos_len = (end - start).length() as f32 * 4.0;
        let scale = if vel_len > pos_len {
            f64::from(pos_len / vel_len)
        } else {
            1.0
        };
        let (v0, v1) = (v0 * scale, v1 * scale);

        let mut speed: f32 = 0.0;
        let mut steps = 0;
        while !sp.reached_destination() {
            let time = f64::from(sp.spline_time);
            sp.advance_spline_target_along_track(1.0);
            let expected = hermite(start, v0, end, v1, time);
            let got = sp.target_pos;
            assert!(
                (got - expected).length() < 1e-3,
                "step {steps}: {got:?} against {expected:?}"
            );
            // The cubic's speed there, from its derivative taken apart.
            let h = 1e-6;
            let ahead = hermite(start, v0, end, v1, time + h);
            let behind = hermite(start, v0, end, v1, time - h);
            let cubic_speed = (ahead - behind).length() / (2.0 * h);
            speed = (speed + 100.0).min(600.0);
            let step = f64::from(sp.spline_time) - time;
            let wanted = f64::from(speed) / cubic_speed;
            assert!(
                (step - wanted).abs() <= wanted * 1e-4,
                "step {steps}: the time moved {step}, not {wanted}"
            );
            steps += 1;
            assert!(steps < 10_000, "the segment never ends");
        }
        assert!(sp.spline_time >= 1.0);
        // About 75 m of curve at up to 6 m a step: a dozen or two.
        assert!((10..40).contains(&steps), "{steps} steps");
    }

    /// A spline along a straight line stays on it: from a straight approach due north through a
    /// spline point to a straight leg due north, every sample keeps home's longitude.
    #[test]
    fn a_spline_through_a_straight_line_stays_on_it() {
        let mut sp = spline();
        let before = sp.pv_location_to_vector(HOME.0, HOME.1, 50.0);
        let start = sp.pv_location_to_vector(HOME.0 + 0.0003, HOME.1, 50.0);
        let end = sp.pv_location_to_vector(HOME.0 + 0.0006, HOME.1, 50.0);
        let next = sp.pv_location_to_vector(HOME.0 + 0.0009, HOME.1, 50.0);
        sp.set_wp_origin_and_destination(before, start);
        sp.set_reached_destination(true);
        sp.set_spline_destination(end, false, SegmentEnd::Straight, next);
        let mut samples = Vec::new();
        while !sp.reached_destination() {
            sp.advance_spline_target_along_track(1.0);
            samples.push(sp.pv_vector_to_location(sp.target_pos));
        }
        assert!(samples.len() > 3);
        let (first_lat, _, _) = samples[0];
        // The first sample is the segment's start, as `pv_vector_to_location` gives it back.
        assert!((first_lat - (HOME.0 + 0.0003)).abs() < 1e-5);
        let home_lng = sp.pv_vector_to_location(Vector3::default()).1;
        for (lat, lng, alt) in &samples {
            assert!((lng - home_lng).abs() < 1e-9, "{lat},{lng} left the line");
            assert!((alt - 50.0).abs() < 1e-6);
        }
        // Northward all the way.
        for pair in samples.windows(2) {
            assert!(pair[1].0 >= pair[0].0, "{pair:?}");
        }
    }

    /// The next segment of a run of splines starts where the last ended, with its velocity, and
    /// keeps the time it overran by when that is under 0.1.
    #[test]
    fn the_next_spline_segment_starts_from_the_last_destination() {
        let mut sp = spline();
        let a = sp.pv_location_to_vector(HOME.0, HOME.1, 0.0);
        let b = sp.pv_location_to_vector(HOME.0 + 0.0004, HOME.1, 0.0);
        let c = sp.pv_location_to_vector(HOME.0 + 0.0004, HOME.1 + 0.0005, 0.0);
        let d = sp.pv_location_to_vector(HOME.0 + 0.0008, HOME.1 + 0.0005, 0.0);
        sp.set_wp_origin_and_destination(a, b);
        sp.set_reached_destination(true);
        sp.set_spline_destination(c, false, SegmentEnd::Spline, d);
        while !sp.reached_destination() {
            sp.advance_spline_target_along_track(1.0);
        }
        let ended = sp.spline_time;
        sp.set_spline_destination(d, false, SegmentEnd::Straight, a);
        assert_eq!(sp.origin, c);
        assert_eq!(sp.destination, d);
        if ended > 1.0 && ended < 1.1_f32 {
            assert_eq!(sp.spline_time, ended - 1.0);
        } else {
            assert_eq!(sp.spline_time, 0.0);
        }
        // The first step is where the time left it: at the start when it is 0.
        sp.advance_spline_target_along_track(1.0);
        if sp.spline_time == 0.0 {
            assert!((sp.target_pos - c).length() < 1e-6);
        }
    }

    /// With no acceleration the stopping distance is 0; otherwise `speed² / (4 accel)`.
    #[test]
    fn the_slow_down_distance_is_the_c_sharps() {
        let mut sp = spline();
        sp.calc_slow_down_distance(600.0, 100.0);
        assert!((sp.slow_down_dist - 900.0).abs() < f32::EPSILON);
        sp.calc_slow_down_distance(600.0, 0.0);
        assert_eq!(sp.slow_down_dist, 0.0);
    }
}
