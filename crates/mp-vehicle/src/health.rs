//! The two readouts that explain a vehicle nobody can explain.
//!
//! A vehicle that will not arm, drifts in a hover, climbs when told to hold, or flips on takeoff
//! is almost always one of two things: the estimator does not believe its own answer, or the
//! frame is shaking hard enough that the accelerometers cannot be read. Both are transmitted
//! continuously and neither is visible anywhere on a normal flight display, which is why the
//! usual diagnosis is a forum post and a week.
//!
//! So both get a number, a threshold, and a verdict. The verdict matters more than the number:
//! "0.62" means nothing to a pilot standing in a paddock, and "position estimate is marginal"
//! means something to everybody.

/// How bad a reading is.
///
/// Three levels rather than a continuous colour, because the decision a pilot makes is also three:
/// fly, look into it, do not fly.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default)]
pub enum Severity {
    /// Nothing to do.
    #[default]
    Good,
    /// Worth understanding before flying.
    Warning,
    /// Do not fly.
    Bad,
}

impl Severity {
    /// The worse of two.
    #[must_use]
    pub fn max(self, other: Self) -> Self {
        if other > self { other } else { self }
    }

    /// A word for the verdict.
    #[must_use]
    pub const fn label(self) -> &'static str {
        match self {
            Self::Good => "ok",
            Self::Warning => "marginal",
            Self::Bad => "bad",
        }
    }
}

/// Variance above which the estimator is considered marginal.
///
/// ArduPilot's own EKF failsafe triggers at `FS_EKF_THRESH`, which defaults to 0.8, and Mission
/// Planner's EKF window turns a bar red at the same figure. 0.5 is the amber it uses below that -
/// not a threshold the firmware acts on, but the point past which a variance is drifting rather
/// than sitting where a healthy estimator sits, which is under about 0.3.
pub const VARIANCE_BAD: f32 = 0.8;
/// Variance above which a reading is worth understanding.
pub const VARIANCE_WARNING: f32 = 0.5;

/// Vibration above which the accelerometers are at risk of being unusable, in m/s².
///
/// ArduPilot's guidance: below 30 is fine, 30 to 60 is a grey area that may cause problems, above
/// 60 will. These are the same numbers the wiki gives pilots, deliberately - a ground station that
/// invents its own thresholds teaches a pilot a scale nobody else uses.
pub const VIBRATION_BAD: f32 = 60.0;
/// Vibration worth investigating, in m/s².
pub const VIBRATION_WARNING: f32 = 30.0;

/// The estimator's own account of how well it is doing.
///
/// Every variance is normalised by the firmware so that 1.0 is the innovation the filter considers
/// its limit; they are not in any physical unit and are not comparable to each other except by
/// that scale.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct EkfStatus {
    /// Velocity estimate variance.
    pub velocity_variance: f32,
    /// Horizontal position estimate variance.
    pub position_horizontal_variance: f32,
    /// Vertical position estimate variance.
    pub position_vertical_variance: f32,
    /// Compass estimate variance.
    pub compass_variance: f32,
    /// Terrain altitude estimate variance.
    pub terrain_altitude_variance: f32,
    /// `EKF_STATUS_FLAGS` bitfield.
    pub flags: u16,
    /// Whether an `EKF_STATUS_REPORT` has ever arrived.
    ///
    /// Distinguishes "the estimator reports zero variance", which is what a stationary vehicle on
    /// the bench reports, from "nothing has been heard" - which look identical in the numbers and
    /// mean opposite things.
    pub seen: bool,
}

/// One named variance.
pub type Variance = (&'static str, f32);

impl EkfStatus {
    /// The five variances, named as a pilot would name them.
    ///
    /// Not as the wire names them. `pos_horiz_variance` is a field name; "position (horizontal)"
    /// is what it is.
    #[must_use]
    pub const fn variances(&self) -> [Variance; 5] {
        [
            ("velocity", self.velocity_variance),
            ("position (horizontal)", self.position_horizontal_variance),
            ("position (vertical)", self.position_vertical_variance),
            ("compass", self.compass_variance),
            ("terrain", self.terrain_altitude_variance),
        ]
    }

    /// The largest variance, and which one it is.
    ///
    /// The terrain variance is excluded unless the vehicle actually has a terrain source: without
    /// a rangefinder it sits at a value that means nothing, and letting it drive the verdict makes
    /// every vehicle look unhealthy.
    #[must_use]
    pub fn worst(&self) -> Variance {
        let mut worst: Variance = ("velocity", self.velocity_variance);
        for candidate in [
            ("position (horizontal)", self.position_horizontal_variance),
            ("position (vertical)", self.position_vertical_variance),
            ("compass", self.compass_variance),
        ] {
            if candidate.1 > worst.1 {
                worst = candidate;
            }
        }
        worst
    }

    /// The verdict on the estimator.
    #[must_use]
    pub fn severity(&self) -> Severity {
        if !self.seen {
            return Severity::Good;
        }
        // An uninitialised estimator is not a marginal one, whatever its variances say. It has
        // never produced an answer, so the numbers beside it are the absence of an answer rather
        // than a poor one.
        if self.flags & FLAG_UNINITIALIZED != 0 {
            return Severity::Bad;
        }
        let worst = self.worst().1;
        if worst >= VARIANCE_BAD {
            Severity::Bad
        } else if worst >= VARIANCE_WARNING {
            Severity::Warning
        } else {
            Severity::Good
        }
    }

    /// Whether the GPS is being rejected as faulty.
    ///
    /// Its own question rather than part of the verdict: a glitching GPS on a vehicle flying
    /// indoors on optical flow is expected, and on one relying on it for position it is the whole
    /// problem.
    #[must_use]
    pub const fn gps_glitching(&self) -> bool {
        self.flags & FLAG_GPS_GLITCHING != 0
    }

    /// Whether the estimator has never produced a healthy answer.
    #[must_use]
    pub const fn uninitialised(&self) -> bool {
        self.flags & FLAG_UNINITIALIZED != 0
    }

    /// Whether the estimator has given up on knowing where it is.
    ///
    /// Constant position mode is what the filter falls back to with no position source at all. A
    /// vehicle in it will hold attitude and nothing else.
    #[must_use]
    pub const fn constant_position_mode(&self) -> bool {
        self.flags & FLAG_CONST_POS_MODE != 0
    }

    /// The estimates the filter says are good, named for a person.
    #[must_use]
    pub fn healthy_estimates(&self) -> Vec<&'static str> {
        NAMED_FLAGS
            .iter()
            .filter(|flag| self.flags & flag.bit != 0)
            .map(|flag| flag.name)
            .collect()
    }

    /// The estimates the filter does not consider good, and whose absence is worth saying.
    ///
    /// The useful half of the flags. A vehicle that will not switch to Loiter has one of these
    /// missing, and which one says why.
    ///
    /// Only the ones a pilot can act on. Height above ground needs a rangefinder most vehicles do
    /// not have, and the two predicted-position flags are the filter's forecast of what the real
    /// ones already say - listing either would put a permanent amber line on a healthy aircraft,
    /// and a warning that is always on is a warning nobody reads.
    #[must_use]
    pub fn unhealthy_estimates(&self) -> Vec<&'static str> {
        NAMED_FLAGS
            .iter()
            .filter(|flag| flag.essential && self.flags & flag.bit == 0)
            .map(|flag| flag.name)
            .collect()
    }

    /// Every estimate the filter does not consider good, essential or not.
    ///
    /// For a detail view that shows the lot rather than the ones worth acting on.
    #[must_use]
    pub fn all_unhealthy_estimates(&self) -> Vec<&'static str> {
        NAMED_FLAGS
            .iter()
            .filter(|flag| self.flags & flag.bit == 0)
            .map(|flag| flag.name)
            .collect()
    }
}

/// `EKF_UNINITIALIZED`.
const FLAG_UNINITIALIZED: u16 = 1024;
/// `EKF_GPS_GLITCHING`.
const FLAG_GPS_GLITCHING: u16 = 32_768;
/// `EKF_CONST_POS_MODE`.
const FLAG_CONST_POS_MODE: u16 = 128;

/// The "this estimate is good" flags, in the order they matter to a pilot.
///
/// `EKF_CONST_POS_MODE`, `EKF_GPS_GLITCHING` and `EKF_UNINITIALIZED` are deliberately absent:
/// those three are set when something is *wrong*, so listing them among the healthy estimates
/// would invert their meaning.
const NAMED_FLAGS: &[Flag] = &[
    Flag::essential(1, "attitude"),
    Flag::essential(2, "horizontal velocity"),
    Flag::essential(4, "vertical velocity"),
    Flag::essential(8, "position (relative)"),
    Flag::essential(16, "position (absolute)"),
    Flag::essential(32, "altitude"),
    // Needs a rangefinder or a terrain database. Most vehicles have neither, so reporting its
    // absence as a problem paints an amber line on every healthy aircraft forever - which teaches
    // the pilot to ignore that line, including on the day it means something.
    Flag::optional(64, "height above ground"),
    // The filter's forecast of whether it will be able to hold position, used by the arming
    // checks. It is set alongside the real thing on a healthy vehicle and tells a pilot nothing
    // the real thing has not already told them.
    Flag::optional(256, "predicted position (relative)"),
    Flag::optional(512, "predicted position (absolute)"),
];

/// One `EKF_STATUS_FLAGS` bit.
struct Flag {
    bit: u16,
    name: &'static str,
    /// Whether its absence is worth telling a pilot about.
    essential: bool,
}

impl Flag {
    const fn essential(bit: u16, name: &'static str) -> Self {
        Self {
            bit,
            name,
            essential: true,
        }
    }

    const fn optional(bit: u16, name: &'static str) -> Self {
        Self {
            bit,
            name,
            essential: false,
        }
    }
}

/// How hard the frame is shaking, and whether it has overwhelmed an accelerometer.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Vibration {
    /// Vibration on the X axis, in m/s².
    pub x: f32,
    /// Vibration on the Y axis, in m/s².
    pub y: f32,
    /// Vibration on the Z axis, in m/s².
    pub z: f32,
    /// Clipping events per accelerometer, since boot.
    pub clipping: [u32; 3],
    /// Whether a `VIBRATION` message has ever arrived.
    pub seen: bool,
}

impl Vibration {
    /// The three axes, named.
    #[must_use]
    pub const fn axes(&self) -> [(&'static str, f32); 3] {
        [("x", self.x), ("y", self.y), ("z", self.z)]
    }

    /// The worst axis, and which one it is.
    ///
    /// Z is usually the worst on a multirotor and usually the one that matters, but saying which
    /// axis rather than assuming is what distinguishes a propeller balance problem from a frame
    /// resonance.
    #[must_use]
    pub fn worst(&self) -> (&'static str, f32) {
        let mut worst = ("x", self.x);
        if self.y > worst.1 {
            worst = ("y", self.y);
        }
        if self.z > worst.1 {
            worst = ("z", self.z);
        }
        worst
    }

    /// Clipping events across all accelerometers.
    #[must_use]
    pub fn total_clipping(&self) -> u32 {
        self.clipping.iter().copied().fold(0, u32::saturating_add)
    }

    /// The verdict on the frame.
    ///
    /// Any clipping at all is at least a warning, whatever the vibration levels say. Clipping is
    /// an accelerometer that has been driven past its range: the samples during it are not merely
    /// noisy, they are wrong, and the filter has been fed them.
    #[must_use]
    pub fn severity(&self) -> Severity {
        if !self.seen {
            return Severity::Good;
        }
        let worst = self.worst().1;
        let from_level = if worst >= VIBRATION_BAD {
            Severity::Bad
        } else if worst >= VIBRATION_WARNING {
            Severity::Warning
        } else {
            Severity::Good
        };
        if self.total_clipping() > 0 {
            from_level.max(Severity::Warning)
        } else {
            from_level
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Nothing heard is not the same as everything at zero, and must not read as a healthy vehicle
    /// on a screen that has never received a message.
    #[test]
    fn an_unheard_estimator_is_not_reported_as_anything() {
        let ekf = EkfStatus::default();
        assert!(!ekf.seen);
        assert_eq!(ekf.severity(), Severity::Good);
        assert_eq!(Vibration::default().severity(), Severity::Good);
    }

    /// The thresholds are ArduPilot's own, so the verdict matches what the firmware will do.
    #[test]
    fn the_verdict_follows_the_firmwares_own_threshold() {
        let at = |variance| EkfStatus {
            velocity_variance: variance,
            seen: true,
            ..EkfStatus::default()
        };
        assert_eq!(at(0.2).severity(), Severity::Good);
        assert_eq!(at(0.49).severity(), Severity::Good);
        assert_eq!(at(0.5).severity(), Severity::Warning);
        assert_eq!(at(0.79).severity(), Severity::Warning);
        // 0.8 is FS_EKF_THRESH's default: the firmware acts here, so we must not still say "ok".
        assert_eq!(at(0.8).severity(), Severity::Bad);
    }

    /// A vehicle with no rangefinder reports a terrain variance that means nothing.
    #[test]
    fn terrain_variance_does_not_drive_the_verdict() {
        let ekf = EkfStatus {
            terrain_altitude_variance: 1.0,
            seen: true,
            ..EkfStatus::default()
        };
        assert_eq!(ekf.severity(), Severity::Good);
        // It is still shown, because a vehicle that does have one wants to see it.
        assert_eq!(ekf.variances()[4], ("terrain", 1.0));
    }

    /// An estimator that has never worked is not a marginal one.
    #[test]
    fn an_uninitialised_estimator_is_bad_whatever_its_variances_say() {
        let ekf = EkfStatus {
            flags: FLAG_UNINITIALIZED,
            seen: true,
            ..EkfStatus::default()
        };
        assert_eq!(ekf.severity(), Severity::Bad);
        assert!(ekf.uninitialised());
    }

    /// Which axis is shaking is the difference between a propeller and a frame.
    #[test]
    fn the_worst_axis_is_named_rather_than_assumed() {
        let vibration = Vibration {
            x: 12.0,
            y: 41.0,
            z: 9.0,
            seen: true,
            ..Vibration::default()
        };
        assert_eq!(vibration.worst(), ("y", 41.0));
        assert_eq!(vibration.severity(), Severity::Warning);
    }

    /// ArduPilot's published numbers, so a pilot reads the same scale everywhere.
    #[test]
    fn vibration_uses_the_thresholds_the_wiki_gives_pilots() {
        let at = |level| Vibration {
            z: level,
            seen: true,
            ..Vibration::default()
        };
        assert_eq!(at(15.0).severity(), Severity::Good);
        assert_eq!(at(30.0).severity(), Severity::Warning);
        assert_eq!(at(60.0).severity(), Severity::Bad);
    }

    /// Clipping is an accelerometer driven past its range: the samples are wrong, not noisy.
    #[test]
    fn any_clipping_at_all_is_worth_saying_even_when_levels_look_fine() {
        let vibration = Vibration {
            x: 1.0,
            y: 1.0,
            z: 1.0,
            clipping: [0, 3, 0],
            seen: true,
        };
        assert_eq!(vibration.total_clipping(), 3);
        assert_eq!(vibration.severity(), Severity::Warning);
    }

    /// Clipping must not downgrade a verdict that is already worse than a warning.
    #[test]
    fn clipping_never_makes_a_bad_reading_look_better() {
        let vibration = Vibration {
            z: 90.0,
            clipping: [1, 0, 0],
            seen: true,
            ..Vibration::default()
        };
        assert_eq!(vibration.severity(), Severity::Bad);
    }

    /// The flags say which estimate is missing, which is why a mode change was refused.
    #[test]
    fn the_flags_name_what_the_filter_does_not_have() {
        // Attitude and both velocities good; no position at all.
        let ekf = EkfStatus {
            flags: 1 | 2 | 4,
            seen: true,
            ..EkfStatus::default()
        };
        assert_eq!(
            ekf.healthy_estimates(),
            ["attitude", "horizontal velocity", "vertical velocity"]
        );
        assert!(ekf.unhealthy_estimates().contains(&"position (absolute)"));
        assert!(!ekf.unhealthy_estimates().contains(&"attitude"));
    }

    /// A vehicle with no rangefinder is not an unhealthy vehicle, and must not wear an amber line
    /// forever for the lack of one.
    #[test]
    fn an_estimate_the_vehicle_cannot_make_is_not_reported_as_a_problem() {
        // Everything essential good; AGL and both predicted flags clear, as on a vehicle with no
        // rangefinder.
        let ekf = EkfStatus {
            flags: 1 | 2 | 4 | 8 | 16 | 32,
            seen: true,
            ..EkfStatus::default()
        };
        assert!(
            ekf.unhealthy_estimates().is_empty(),
            "{:?}",
            ekf.unhealthy_estimates()
        );
        // Still reachable for a detail view that wants the lot.
        assert!(
            ekf.all_unhealthy_estimates()
                .contains(&"height above ground")
        );
    }

    /// The fault flags must never be listed among the healthy estimates.
    #[test]
    fn a_fault_flag_is_not_reported_as_a_good_estimate() {
        let ekf = EkfStatus {
            flags: FLAG_GPS_GLITCHING | FLAG_CONST_POS_MODE | FLAG_UNINITIALIZED,
            seen: true,
            ..EkfStatus::default()
        };
        assert!(ekf.healthy_estimates().is_empty());
        assert!(ekf.gps_glitching());
        assert!(ekf.constant_position_mode());
    }

    /// A saturating total, so a counter that has run away cannot panic a debug build.
    #[test]
    fn a_runaway_clipping_counter_does_not_overflow() {
        let vibration = Vibration {
            clipping: [u32::MAX, u32::MAX, 1],
            seen: true,
            ..Vibration::default()
        };
        assert_eq!(vibration.total_clipping(), u32::MAX);
    }
}
