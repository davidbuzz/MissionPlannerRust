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

//! The planner's pointer read-out, `coords1`: `ExtLibs/Controls/Coords.cs` @ efb0801
//! (GPL-3.0-only), a `Coords` control with `Vertical = true` in `panel4` at the top of the
//! action panel (`FlightPlanner.Designer.cs:406`; `FlightPlanner.resx`: `coords1` at (0, 0),
//! 127 by 55).
//!
//! A combo box offers GEO, UTM and MGRS (`CMB_coordsystem`, GEO to begin with), and the control
//! paints the pointer's position in that system beside it, one number a line, the terrain height
//! under them in the display unit, and where that height came from (`AltSource`) in a line of its
//! own. `SetMouseDisplay` feeds it on every mouse move over the map
//! (`GCSViews/FlightPlanner.cs:2778-2797`), the height from `srtm.getAltitude`.
//!
//! What is here is the text the control paints; the drawing is `plan::coords_panel`. The numbers
//! are formatted as .NET formats them: `0.0000000` for degrees, `0.00` for the height, and a
//! double's own `ToString()` for UTM's metres.

use mp_mission::dotnet::format_f64;
use mp_mission::geoutility::{utm_to_mgrs, wgs84_to_utm};

/// `Coords.CoordsSystems`, in the combo's order.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub(crate) enum CoordSystem {
    /// Latitude and longitude.
    #[default]
    Geo,
    /// Zone and band, easting, northing.
    Utm,
    /// The military grid reference.
    Mgrs,
}

impl CoordSystem {
    /// The combo's entries in its order.
    pub(crate) const ALL: [Self; 3] = [Self::Geo, Self::Utm, Self::Mgrs];

    /// The combo's text for the entry, which is also `coords1.System`.
    #[must_use]
    pub(crate) const fn name(self) -> &'static str {
        match self {
            Self::Geo => "GEO",
            Self::Utm => "UTM",
            Self::Mgrs => "MGRS",
        }
    }

    /// The probe id of the entry's button.
    #[must_use]
    pub(crate) const fn id(self) -> &'static str {
        match self {
            Self::Geo => "plan-coords-geo",
            Self::Utm => "plan-coords-utm",
            Self::Mgrs => "plan-coords-mgrs",
        }
    }
}

/// What the control holds: `System`, `Lat`, `Lng`, `Alt`, `AltUnit` and `AltSource`.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Coords {
    /// The combo's choice.
    pub system: CoordSystem,
    /// `Lat`, degrees.
    pub lat: f64,
    /// `Lng`, degrees.
    pub lng: f64,
    /// `Alt`, already in the display unit (`altdata.alt * CurrentState.multiplieralt`).
    pub alt: f64,
    /// `AltUnit`, `CurrentState.AltUnit`.
    pub alt_unit: &'static str,
    /// `AltSource`: "SRTM", "ASCII", "Ocean", "Invalid" or "".
    pub alt_source: &'static str,
}

impl Default for Coords {
    fn default() -> Self {
        Self {
            system: CoordSystem::Geo,
            lat: 0.0,
            lng: 0.0,
            alt: 0.0,
            alt_unit: "m",
            alt_source: "",
        }
    }
}

impl Coords {
    /// `OnPaint` with `Vertical`: the lines drawn beside the combo, top to bottom. Empty where
    /// the C# draws nothing - UTM and MGRS outside 80 S to 84 N or at the antimeridian, where
    /// `OnPaint` returns before drawing, and an MGRS GeoUtility refuses (the `catch {}`).
    /// `// C#: ExtLibs/Controls/Coords.cs:100-170`
    #[must_use]
    pub(crate) fn lines(&self) -> Vec<String> {
        let alt = format!("{}{}", format_f64(self.alt, "0.00"), self.alt_unit);
        match self.system {
            CoordSystem::Geo => vec![
                format_f64(self.lat, "0.0000000"),
                format_f64(self.lng, "0.0000000"),
                alt,
            ],
            CoordSystem::Utm | CoordSystem::Mgrs => {
                if self.lat > 84.0 || self.lat < -80.0 || self.lng >= 180.0 || self.lng <= -180.0 {
                    return Vec::new();
                }
                let Ok(utm) = wgs84_to_utm(self.lat, self.lng) else {
                    return Vec::new();
                };
                if self.system == CoordSystem::Utm {
                    // `utm.ToString().Split(' ')`: zone and band, easting, northing.
                    let mut lines: Vec<String> =
                        utm.text().split(' ').map(ToOwned::to_owned).collect();
                    lines.push(alt);
                    lines
                } else {
                    let Ok(mgrs) = utm_to_mgrs(&utm) else {
                        return Vec::new();
                    };
                    vec![mgrs.text(), alt]
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn at_cmac(system: CoordSystem) -> Coords {
        Coords {
            system,
            lat: -35.363_262_1,
            lng: 149.165_237_4,
            alt: 584.0,
            alt_unit: "m",
            alt_source: "SRTM",
        }
    }

    #[test]
    fn geo_is_seven_decimals_each_then_the_height_in_its_unit() {
        assert_eq!(
            at_cmac(CoordSystem::Geo).lines(),
            vec!["-35.3632621", "149.1652374", "584.00m"]
        );
        let zero = Coords::default();
        assert_eq!(zero.lines(), vec!["0.0000000", "0.0000000", "0.00m"]);
    }

    #[test]
    fn utm_is_the_zone_and_band_then_easting_northing_and_height() {
        let lines = at_cmac(CoordSystem::Utm).lines();
        assert_eq!(lines.len(), 4);
        assert_eq!(lines[0], "55H");
        // CMAC is about 696.7 km east and 6,084.6 km north in zone 55.
        let east: f64 = lines[1].parse().expect("easting");
        let north: f64 = lines[2].parse().expect("northing");
        assert!((696_000.0..697_500.0).contains(&east), "{east}");
        assert!((6_084_000.0..6_085_500.0).contains(&north), "{north}");
        assert_eq!(lines[3], "584.00m");
    }

    #[test]
    fn mgrs_is_one_reference_then_the_height() {
        let lines = at_cmac(CoordSystem::Mgrs).lines();
        assert_eq!(lines.len(), 2);
        assert!(lines[0].starts_with("55HFA"), "{}", lines[0]);
        assert_eq!(lines[1], "584.00m");
    }

    #[test]
    fn utm_and_mgrs_draw_nothing_beyond_the_bands_as_on_paint_returns() {
        let mut polar = at_cmac(CoordSystem::Utm);
        polar.lat = 85.0;
        assert!(polar.lines().is_empty());
        polar.system = CoordSystem::Mgrs;
        assert!(polar.lines().is_empty());
        polar.system = CoordSystem::Geo;
        assert_eq!(polar.lines().len(), 3);
    }

    #[test]
    fn the_combo_names_and_ids_are_in_the_designers_order() {
        assert_eq!(
            CoordSystem::ALL.map(CoordSystem::name),
            ["GEO", "UTM", "MGRS"]
        );
        assert_eq!(CoordSystem::default(), CoordSystem::Geo);
    }
}
