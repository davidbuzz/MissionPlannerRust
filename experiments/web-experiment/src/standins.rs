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
// SPDX-License-Identifier: GPL-3.0-only

//! What hud.rs and pictures.rs reach for in the rest of mp-gui, which the experiment does not
//! build: each module here stands where its mp-gui namesake does, so `crate::fly::...` and the
//! rest resolve unchanged. Functions are copied verbatim from the files named; the rest is the
//! least that compiles.

/// `crate::config::battery_monitor`: `float_text`, copied from
/// crates/mp-gui/src/config/battery_monitor.rs:237-319.
pub mod config {
    pub mod battery_monitor {
        /// `float.ToString()` as .NET Framework writes it: `G` at seven significant digits, fixed-point
        /// for a power of ten from -5 (exclusive) to 7 (exclusive) and `1.234568E+07` otherwise, trailing
        /// zeros dropped, and a negative zero written "0".
        ///
        /// The seventh digit is rounded half away from zero on the float's exact decimal expansion, as
        /// the CRT's `_ecvt` behind `Number.FormatSingle` rounds; a float lands exactly on a half at the
        /// eighth digit only for a few short binary fractions, and none of this page's numbers.
        #[must_use]
        pub fn float_text(value: f32) -> String {
            if value.is_nan() {
                return "NaN".to_owned();
            }
            if value.is_infinite() {
                return if value > 0.0 { "Infinity" } else { "-Infinity" }.to_owned();
            }
            if value == 0.0 {
                return "0".to_owned();
            }
            // Forty digits hold a float's whole expansion for any number this page shows.
            let exact = format!("{:.40e}", f64::from(value).abs());
            let Some((mantissa, exponent)) = exact.split_once('e') else {
                return "0".to_owned();
            };
            let mut exponent: i32 = exponent.parse().unwrap_or(0);
            let digits: Vec<u8> = mantissa
                .bytes()
                .filter(u8::is_ascii_digit)
                .map(|digit| digit - b'0')
                .collect();
            let mut kept: Vec<u8> = digits.iter().copied().take(7).collect();
            if digits.get(7).is_some_and(|digit| *digit >= 5) {
                let mut carry = true;
                for digit in kept.iter_mut().rev() {
                    if *digit == 9 {
                        *digit = 0;
                    } else {
                        *digit += 1;
                        carry = false;
                        break;
                    }
                }
                if carry {
                    kept.insert(0, 1);
                    kept.pop();
                    exponent += 1;
                }
            }
            while kept.len() > 1 && kept.last() == Some(&0) {
                kept.pop();
            }
            let text: String = kept.iter().map(|digit| char::from(b'0' + digit)).collect();
            let sign = if value < 0.0 { "-" } else { "" };
            if exponent > -5 && exponent < 7 {
                let point = usize::try_from(exponent + 1).unwrap_or(0);
                if exponent >= 0 {
                    let (whole, fraction) = if text.len() > point {
                        (
                            text.get(..point).unwrap_or(""),
                            text.get(point..).unwrap_or(""),
                        )
                    } else {
                        (text.as_str(), "")
                    };
                    let zeros = "0".repeat(point.saturating_sub(text.len()));
                    if fraction.is_empty() {
                        format!("{sign}{whole}{zeros}")
                    } else {
                        format!("{sign}{whole}.{fraction}")
                    }
                } else {
                    let zeros = "0".repeat(usize::try_from(-exponent - 1).unwrap_or(0));
                    format!("{sign}0.{zeros}{text}")
                }
            } else {
                let (first, rest) = text.split_at(1);
                let point = if rest.is_empty() { "" } else { "." };
                let exponent_sign = if exponent < 0 { '-' } else { '+' };
                format!(
                    "{sign}{first}{point}{rest}E{exponent_sign}{:02}",
                    exponent.unsigned_abs()
                )
            }
        }
    }
}

/// `crate::fly`: `displayed_altitude` and its multiplier, copied from crates/mp-gui/src/fly.rs.
pub mod fly {
    /// `CurrentState.multiplieralt` (crates/mp-gui/src/fly.rs:653-657).
    pub const MULTIPLIER_ALT: f32 = 1.0;

    /// crates/mp-gui/src/fly.rs:1634-1636.
    pub fn displayed_altitude(relative: f64, offset: f32) -> f64 {
        (relative - f64::from(offset)) * f64::from(MULTIPLIER_ALT)
    }
}

/// `crate::quick`: the display-unit multipliers, copied from crates/mp-gui/src/quick.rs:739-797.
pub mod quick {
    use mp_vehicle::units::DisplayUnits;

    /// Which of `CurrentState`'s multipliers a property's value is shown through.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum Multiplier {
        /// `multiplierdist`.
        Dist,
        /// `multiplieralt`.
        Alt,
        /// `multiplierspeed`.
        Speed,
    }

    /// The held properties the C# shows in the user's units, with the multiplier each one's getter
    /// applies and the getter's line. The getter decides, not the text: `alt_error` says "(dist)"
    /// and is multiplied as an altitude, `altasl2` says "(dist)" and is not multiplied at all.
    ///
    /// `wind_vel` is the one multiplied where it is set rather than where it is read: the `WIND`
    /// handler stores `wind.speed * multiplierspeed` (`CurrentState.cs:2858`). The C#'s other
    /// source, `HIGH_LATENCY`, stores its speed unmultiplied (`:2540`); the vehicle's state does not
    /// say which message set it, and this multiplies it as a vehicle sending `WIND` - ArduPilot on
    /// a normal link - has it.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs`
    pub const IN_DISPLAY_UNITS: &[(&str, Multiplier, u32)] = &[
        ("alt", Multiplier::Alt, 327),
        ("altasl", Multiplier::Alt, 354),
        ("airspeed", Multiplier::Speed, 496),
        ("groundspeed", Multiplier::Speed, 529),
        ("wp_dist", Multiplier::Dist, 1093),
        ("alt_error", Multiplier::Alt, 1102),
        ("climbrate", Multiplier::Speed, 1154),
        ("verticalspeed", Multiplier::Speed, 1048),
        ("DistFromMovingBase", Multiplier::Dist, 1781),
        ("wind_vel", Multiplier::Speed, 2858),
        ("DistToHome", Multiplier::Dist, 1781),
        ("sonarrange", Multiplier::Alt, 1861),
        ("ter_curalt", Multiplier::Alt, 2055),
        ("ter_alt", Multiplier::Alt, 2063),
    ];

    /// The multiplier a property is shown through, if any.
    #[must_use]
    pub fn multiplier(name: &str) -> Option<Multiplier> {
        IN_DISPLAY_UNITS
            .iter()
            .find(|(field, _, _)| *field == name)
            .map(|(_, multiplier, _)| *multiplier)
    }

    /// A property's SI value as its getter returns it: through `toDistDisplayUnit`,
    /// `toAltDisplayUnit` or `toSpeedDisplayUnit` where it has a multiplier, as it is otherwise.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:4360-4373`
    #[must_use]
    pub fn to_display(name: &str, value: f64, units: &DisplayUnits) -> f64 {
        match multiplier(name) {
            Some(Multiplier::Dist) => units.to_dist(value),
            Some(Multiplier::Alt) => units.to_alt(value),
            Some(Multiplier::Speed) => units.to_speed(value),
            None => value,
        }
    }
}

/// `crate::ui::theme`: the two colours pictures.rs uses (crates/mp-gui/src/ui.rs).
pub mod ui {
    pub mod theme {
        pub const BORDER: u32 = 0x3a4048;
        pub const DIM: u32 = 0x8b949e;
    }
}

/// `crate::facts`: off. The planner writes facts for its GUI scripts; the page has none.
pub mod facts {
    pub fn enabled() -> bool {
        false
    }

    pub fn record(_key: impl Into<String>, _value: impl std::fmt::Display) {}
}

/// `crate::probe`: off. The planner's layout probe measures elements for the layout guard.
pub mod probe {
    pub fn measured(_name: impl Into<String>, element: gpui::Div) -> gpui::Div {
        element
    }
}
