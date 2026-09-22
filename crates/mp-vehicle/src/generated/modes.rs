//! Flight mode names, generated from Mission Planner's parameter metadata.
//!
//! DO NOT EDIT. Regenerate with `cargo xtask codegen modes`.
//!
//! Source: `referneces/missionplanner/ParameterMetaDataBackup.xml`, the same file the C#
//! application reads at runtime. Mode numbers are vehicle-specific: mode 4 is Guided on a
//! copter and ACRO on a plane, so a lookup without the vehicle type is not just imprecise,
//! it is wrong.

/// A vehicle family, which determines how a custom mode number is read.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum VehicleFamily {
    /// Copter.
    Copter,
    /// Plane.
    Plane,
    /// Rover.
    Rover,
}

impl VehicleFamily {
    /// The family a `MAV_TYPE` belongs to, or `None` if it is not one we know.
    ///
    /// Returning `None` rather than guessing matters: showing a copter's mode
    /// names for an unrecognised airframe is worse than showing the raw number.
    #[must_use]
    pub const fn from_mav_type(mav_type: u8) -> Option<Self> {
        Some(match mav_type {
            2 | 3 | 4 | 13 | 14 | 15 | 29 => Self::Copter,
            1 | 16 | 17 | 18 | 19 | 20 | 21 | 22 | 23 | 24 | 25 => Self::Plane,
            10 | 11 => Self::Rover,
            _ => return None,
        })
    }

    /// The name of a custom mode for this family.
    #[must_use]
    pub const fn mode_name(self, custom_mode: u32) -> Option<&'static str> {
        Some(match self {
            Self::Copter => match custom_mode {
                0 => "Stabilize",
                1 => "Acro",
                2 => "AltHold",
                3 => "Auto",
                4 => "Guided",
                5 => "Loiter",
                6 => "RTL",
                7 => "Circle",
                9 => "Land",
                11 => "Drift",
                13 => "Sport",
                14 => "Flip",
                15 => "AutoTune",
                16 => "PosHold",
                17 => "Brake",
                18 => "Throw",
                19 => "Avoid_ADSB",
                20 => "Guided_NoGPS",
                21 => "Smart_RTL",
                22 => "FlowHold",
                23 => "Follow",
                24 => "ZigZag",
                25 => "SystemID",
                26 => "Heli_Autorotate",
                27 => "Auto RTL",
                _ => return None,
            },
            Self::Plane => match custom_mode {
                0 => "Manual",
                1 => "CIRCLE",
                2 => "STABILIZE",
                3 => "TRAINING",
                4 => "ACRO",
                5 => "FBWA",
                6 => "FBWB",
                7 => "CRUISE",
                8 => "AUTOTUNE",
                10 => "Auto",
                11 => "RTL",
                12 => "Loiter",
                13 => "TAKEOFF",
                14 => "AVOID_ADSB",
                15 => "Guided",
                17 => "QSTABILIZE",
                18 => "QHOVER",
                19 => "QLOITER",
                20 => "QLAND",
                21 => "QRTL",
                22 => "QAUTOTUNE",
                23 => "QACRO",
                24 => "THERMAL",
                25 => "Loiter to QLand",
                _ => return None,
            },
            Self::Rover => match custom_mode {
                0 => "Manual",
                1 => "Acro",
                3 => "Steering",
                4 => "Hold",
                5 => "Loiter",
                6 => "Follow",
                7 => "Simple",
                10 => "Auto",
                11 => "RTL",
                12 => "SmartRTL",
                15 => "Guided",
                _ => return None,
            },
        })
    }

    /// Every mode this family offers, as (number, name), ordered by number.
    #[must_use]
    pub const fn modes(self) -> &'static [(u32, &'static str)] {
        match self {
            Self::Copter => &[
                (0, "Stabilize"),
                (1, "Acro"),
                (2, "AltHold"),
                (3, "Auto"),
                (4, "Guided"),
                (5, "Loiter"),
                (6, "RTL"),
                (7, "Circle"),
                (9, "Land"),
                (11, "Drift"),
                (13, "Sport"),
                (14, "Flip"),
                (15, "AutoTune"),
                (16, "PosHold"),
                (17, "Brake"),
                (18, "Throw"),
                (19, "Avoid_ADSB"),
                (20, "Guided_NoGPS"),
                (21, "Smart_RTL"),
                (22, "FlowHold"),
                (23, "Follow"),
                (24, "ZigZag"),
                (25, "SystemID"),
                (26, "Heli_Autorotate"),
                (27, "Auto RTL"),
            ],
            Self::Plane => &[
                (0, "Manual"),
                (1, "CIRCLE"),
                (2, "STABILIZE"),
                (3, "TRAINING"),
                (4, "ACRO"),
                (5, "FBWA"),
                (6, "FBWB"),
                (7, "CRUISE"),
                (8, "AUTOTUNE"),
                (10, "Auto"),
                (11, "RTL"),
                (12, "Loiter"),
                (13, "TAKEOFF"),
                (14, "AVOID_ADSB"),
                (15, "Guided"),
                (17, "QSTABILIZE"),
                (18, "QHOVER"),
                (19, "QLOITER"),
                (20, "QLAND"),
                (21, "QRTL"),
                (22, "QAUTOTUNE"),
                (23, "QACRO"),
                (24, "THERMAL"),
                (25, "Loiter to QLand"),
            ],
            Self::Rover => &[
                (0, "Manual"),
                (1, "Acro"),
                (3, "Steering"),
                (4, "Hold"),
                (5, "Loiter"),
                (6, "Follow"),
                (7, "Simple"),
                (10, "Auto"),
                (11, "RTL"),
                (12, "SmartRTL"),
                (15, "Guided"),
            ],
        }
    }
}

/// The mode name for a vehicle type and custom mode number.
#[must_use]
pub fn flight_mode_name(mav_type: u8, custom_mode: u32) -> Option<&'static str> {
    VehicleFamily::from_mav_type(mav_type)?.mode_name(custom_mode)
}
