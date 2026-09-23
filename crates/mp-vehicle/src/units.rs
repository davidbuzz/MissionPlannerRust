//! The user's display units: what `CurrentState`'s static multipliers do.
//!
//! The C# keeps three multipliers and three unit names as statics on `CurrentState`
//! (`multiplierdist`, `multiplieralt`, `multiplierspeed`; `DistanceUnit`, `AltUnit`,
//! `SpeedUnit`), sets them in `MainV2.ChangeUnits` from the `distunits`, `altunits` and
//! `speedunits` settings, and applies them in each property's getter - `alt` returns
//! `(_alt - altoffsethome) * multiplieralt`. The state in this crate stays SI; a display holds a
//! [`DisplayUnits`] and converts with it, which is the same arithmetic in a different place.
//! `// C#: ExtLibs/ArduPilot/CurrentState.cs:23-38, 4360-4383, 4549-4553; MainV2.cs:4247-4330`

/// A distance or altitude unit: the C#'s `distances` and `altitudes` enums, which have the same
/// two members. `// C#: ExtLibs/Utilities/distances.cs:3-7, altitudes.cs:3-7`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Distance {
    /// `Meters`.
    Meters,
    /// `Feet`.
    Feet,
}

impl Distance {
    /// The setting's value as `Enum.Parse` reads it: the member's name, exactly.
    #[must_use]
    pub fn parse(name: &str) -> Option<Self> {
        match name {
            "Meters" => Some(Self::Meters),
            "Feet" => Some(Self::Feet),
            _ => None,
        }
    }
}

/// A speed unit: the C#'s `speeds` enum. `// C#: ExtLibs/Utilities/speeds.cs:3-10`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Speed {
    /// `meters_per_second`.
    MetersPerSecond,
    /// `fps`: feet per second.
    Fps,
    /// `kph`.
    Kph,
    /// `mph`.
    Mph,
    /// `knots`.
    Knots,
}

impl Speed {
    /// The setting's value as `Enum.Parse` reads it: the member's name, exactly.
    #[must_use]
    pub fn parse(name: &str) -> Option<Self> {
        match name {
            "meters_per_second" => Some(Self::MetersPerSecond),
            "fps" => Some(Self::Fps),
            "kph" => Some(Self::Kph),
            "mph" => Some(Self::Mph),
            "knots" => Some(Self::Knots),
            _ => None,
        }
    }
}

/// Feet in a metre: the C#'s `3.2808399f`, spelled as the shortest literal that rounds to the
/// same single. `// C#: MainV2.cs:4262, 4284, 4305`
const FEET: f32 = 3.280_84;

/// The multipliers and unit names a display converts SI values with.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DisplayUnits {
    /// Metres to the display's distance unit (`multiplierdist`).
    pub dist: f32,
    /// The distance unit's name (`DistanceUnit`).
    pub dist_unit: &'static str,
    /// Metres to the display's altitude unit (`multiplieralt`).
    pub alt: f32,
    /// The altitude unit's name (`AltUnit`).
    pub alt_unit: &'static str,
    /// Metres per second to the display's speed unit (`multiplierspeed`).
    pub speed: f32,
    /// The speed unit's name (`SpeedUnit`).
    pub speed_unit: &'static str,
}

impl Default for DisplayUnits {
    /// The statics before `ChangeUnits` first runs: multipliers of 1 and no unit names.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:27-38`
    fn default() -> Self {
        Self {
            dist: 1.0,
            dist_unit: "",
            alt: 1.0,
            alt_unit: "",
            speed: 1.0,
            speed_unit: "",
        }
    }
}

impl DisplayUnits {
    /// `MainV2.ChangeUnits`: each setting, or metres and metres per second where there is none.
    ///
    /// A setting that is not a member name makes `Enum.Parse` throw, and the C# catches that
    /// around the whole method - so the units after it are left as they were. That is kept:
    /// parsing stops at the first bad setting.
    /// `// C#: MainV2.cs:4247-4330`
    #[must_use]
    pub fn change_units(
        self,
        distunits: Option<&str>,
        altunits: Option<&str>,
        speedunits: Option<&str>,
    ) -> Self {
        let mut units = self;
        // C#: MainV2.cs:4252-4270
        let Some(distance) = distunits.map_or(Some(Distance::Meters), Distance::parse) else {
            return units;
        };
        (units.dist, units.dist_unit) = distance_multiplier(distance);
        // C#: MainV2.cs:4273-4292. Parsed as `altitudes`, which has the same members.
        let Some(altitude) = altunits.map_or(Some(Distance::Meters), Distance::parse) else {
            return units;
        };
        (units.alt, units.alt_unit) = distance_multiplier(altitude);
        // C#: MainV2.cs:4295-4325
        let Some(speed) = speedunits.map_or(Some(Speed::MetersPerSecond), Speed::parse) else {
            return units;
        };
        (units.speed, units.speed_unit) = match speed {
            Speed::MetersPerSecond => (1.0, "m/s"),
            Speed::Fps => (FEET, "fps"),
            Speed::Kph => (3.6, "kph"),
            Speed::Mph => (2.236_936_3, "mph"),
            // The C#'s `1.94384449f`, written as the shortest literal that rounds to the same
            // single: `1.9438445` would be the next one up.
            Speed::Knots => (1.943_844_4, "kts"),
        };
        units
    }

    /// `toDistDisplayUnit`: metres in the display's distance unit.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:4360-4363`
    #[must_use]
    pub fn to_dist(&self, metres: f64) -> f64 {
        metres * f64::from(self.dist)
    }

    /// `toAltDisplayUnit`: metres in the display's altitude unit.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:4365-4368`
    #[must_use]
    pub fn to_alt(&self, metres: f64) -> f64 {
        metres * f64::from(self.alt)
    }

    /// `toSpeedDisplayUnit`: metres per second in the display's speed unit.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:4370-4373`
    #[must_use]
    pub fn to_speed(&self, metres_per_second: f64) -> f64 {
        metres_per_second * f64::from(self.speed)
    }

    /// `fromDistDisplayUnit`: a distance the user typed, in metres.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:4375-4378`
    #[must_use]
    pub fn from_dist(&self, value: f64) -> f64 {
        value / f64::from(self.dist)
    }

    /// `fromSpeedDisplayUnit`: a speed the user typed, in metres per second.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:4380-4383`
    #[must_use]
    pub fn from_speed(&self, value: f64) -> f64 {
        value / f64::from(self.speed)
    }

    /// A field's display text with its unit filled in, as `GetNameandUnit` does it: the first of
    /// `(dist)`, `(speed)` and `(alt)` that the text contains, in that order, is replaced.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:4549-4553`
    #[must_use]
    pub fn name_and_unit(&self, display: &str) -> String {
        if display.contains("(dist)") {
            display.replace("(dist)", &format!("({})", self.dist_unit))
        } else if display.contains("(speed)") {
            display.replace("(speed)", &format!("({})", self.speed_unit))
        } else if display.contains("(alt)") {
            display.replace("(alt)", &format!("({})", self.alt_unit))
        } else {
            display.to_owned()
        }
    }
}

/// A distance unit's multiplier and name. `// C#: MainV2.cs:4255-4263`
const fn distance_multiplier(unit: Distance) -> (f32, &'static str) {
    match unit {
        Distance::Meters => (1.0, "m"),
        Distance::Feet => (FEET, "ft"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn no_settings_is_metres_and_metres_per_second() {
        let units = DisplayUnits::default().change_units(None, None, None);
        assert_eq!(
            (units.dist, units.dist_unit, units.alt, units.alt_unit),
            (1.0, "m", 1.0, "m")
        );
        assert_eq!((units.speed, units.speed_unit), (1.0, "m/s"));
    }

    /// A C# `float` literal, parsed from the text the C# writes, so the comparison does not
    /// depend on how the literal is spelled here.
    fn csharp(literal: &str) -> f32 {
        literal.parse().expect("a float literal")
    }

    #[test]
    fn the_multipliers_are_the_csharps_single_precision_literals() {
        // C#: MainV2.cs:4262, 4284, 4305, 4309, 4313, 4317
        let units =
            DisplayUnits::default().change_units(Some("Feet"), Some("Meters"), Some("knots"));
        assert_eq!((units.dist, units.dist_unit), (csharp("3.2808399"), "ft"));
        assert_eq!((units.alt, units.alt_unit), (1.0, "m"));
        assert_eq!(
            (units.speed, units.speed_unit),
            (csharp("1.94384449"), "kts")
        );
        for (name, multiplier, unit) in [
            ("meters_per_second", "1", "m/s"),
            ("fps", "3.2808399", "fps"),
            ("kph", "3.6", "kph"),
            ("mph", "2.23693629", "mph"),
            ("knots", "1.94384449", "kts"),
        ] {
            let units = DisplayUnits::default().change_units(None, None, Some(name));
            assert_eq!(
                (units.speed, units.speed_unit),
                (csharp(multiplier), unit),
                "{name}"
            );
        }
        let feet = DisplayUnits::default().change_units(None, Some("Feet"), None);
        assert_eq!((feet.alt, feet.alt_unit), (csharp("3.2808399"), "ft"));
    }

    #[test]
    fn a_bad_setting_stops_the_rest_as_the_csharps_catch_does() {
        let before = DisplayUnits::default();
        let units = before.change_units(Some("Feet"), Some("furlongs"), Some("knots"));
        assert_eq!(units.dist_unit, "ft", "set before the bad one");
        assert_eq!(units.alt_unit, "", "the bad one is left as it was");
        assert_eq!(units.speed_unit, "", "and so is everything after it");
        assert_eq!(Speed::parse("Knots"), None, "Enum.Parse is case-sensitive");
    }

    #[test]
    fn conversions_go_both_ways() {
        let units = DisplayUnits::default().change_units(Some("Feet"), Some("Feet"), Some("kph"));
        // toDistDisplayUnit is `input * multiplierdist`: a double times the widened single.
        assert_eq!(units.to_dist(100.0), 100.0 * f64::from(csharp("3.2808399")));
        assert_eq!(units.to_alt(10.0), 10.0 * f64::from(csharp("3.2808399")));
        assert_eq!(units.to_speed(10.0), 10.0 * f64::from(csharp("3.6")));
        assert!((units.from_dist(units.to_dist(123.0)) - 123.0).abs() < 1e-9);
        assert!((units.from_speed(units.to_speed(7.0)) - 7.0).abs() < 1e-9);
    }

    #[test]
    fn the_display_text_gets_its_unit() {
        // C#: ExtLibs/ArduPilot/CurrentState.cs:4549-4553, over the texts the C# declares.
        let units = DisplayUnits::default().change_units(Some("Feet"), Some("Meters"), Some("mph"));
        assert_eq!(units.name_and_unit("Dist to WP (dist)"), "Dist to WP (ft)");
        assert_eq!(units.name_and_unit("AirSpeed (speed)"), "AirSpeed (mph)");
        assert_eq!(units.name_and_unit("Altitude (alt)"), "Altitude (m)");
        assert_eq!(units.name_and_unit("Gps HDOP"), "Gps HDOP");
    }
}
