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

//! The `Coords` control, `ExtLibs/Controls/Coords.cs` @ efb0801 (GPL-3.0-only), of which Mission
//! Planner has two:
//!
//! - the planner's pointer read-out, `coords1` with `Vertical = true` in `panel4` at the top of
//!   the action panel (`FlightPlanner.Designer.cs:406`; `FlightPlanner.resx`: `coords1` at
//!   (0, 0), 127 by 55), which `SetMouseDisplay` feeds on every mouse move over the map
//!   (`GCSViews/FlightPlanner.cs:2778-2797`), the height from `srtm.getAltitude`;
//! - the flight screen's, `coords1` with `Vertical = false` in `panel1` under the flight map
//!   (`FlightData.resx`: at (7, 2), 247 by 21, in `tableMap`'s 25-pixel second row), bound to the
//!   vehicle's `lat`, `lng` and `alt` through `bindingSource1`
//!   (`GCSViews/FlightData.Designer.cs:2842-2854`), its `AltUnit` set from `CurrentState.AltUnit`
//!   (`GCSViews/FlightData.cs:447, 4510`).
//!
//! A combo box offers GEO, UTM and MGRS (`CMB_coordsystem`, GEO to begin with), and the control
//! paints the position in that system beside it and the height in the display unit: vertical,
//! one number a line, with where the height came from (`AltSource`) in a line of its own under
//! the combo; horizontal, one line with three spaces before the height and no `AltSource`.
//!
//! What is here is the text the control paints; the drawing is `plan::coords_panel` and
//! `fly::coords_strip`. The numbers are formatted as .NET formats them: `0.0000000` for degrees,
//! `0.00` for the height, and a double's own `ToString()` for UTM's metres.

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

    /// The probe id of the entry's button under the flight map.
    #[must_use]
    pub(crate) const fn fly_id(self) -> &'static str {
        match self {
            Self::Geo => "fly-coords-geo",
            Self::Utm => "fly-coords-utm",
            Self::Mgrs => "fly-coords-mgrs",
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
        self.position().map_or_else(Vec::new, |mut lines| {
            lines.push(self.height());
            lines
        })
    }

    /// `OnPaint` without `Vertical`: the one line drawn beside the combo - GEO's latitude, a
    /// space and the longitude, UTM's `ToString()` or MGRS's - then three spaces and the height.
    /// Empty where the C# draws nothing, as [`Self::lines`].
    /// `// C#: ExtLibs/Controls/Coords.cs:108-111, 130-133, 153-156`
    #[must_use]
    pub(crate) fn line(&self) -> String {
        self.position().map_or_else(String::new, |parts| {
            format!("{}   {}", parts.join(" "), self.height())
        })
    }

    /// `Alt.ToString("0.00") + AltUnit`.
    fn height(&self) -> String {
        format!("{}{}", format_f64(self.alt, "0.00"), self.alt_unit)
    }

    /// The position in the chosen system, in the parts the vertical layout puts a line each:
    /// GEO's two numbers, `utm.ToString().Split(' ')` - zone and band, easting, northing - or the
    /// one MGRS reference at `Precision = 5`. The horizontal layout joins them with a space, which
    /// for UTM is `utm.ToString()` again. `None` where `OnPaint` draws nothing.
    /// `// C#: ExtLibs/Controls/Coords.cs:100-159`
    fn position(&self) -> Option<Vec<String>> {
        match self.system {
            CoordSystem::Geo => Some(vec![
                format_f64(self.lat, "0.0000000"),
                format_f64(self.lng, "0.0000000"),
            ]),
            CoordSystem::Utm | CoordSystem::Mgrs => {
                if self.lat > 84.0 || self.lat < -80.0 || self.lng >= 180.0 || self.lng <= -180.0 {
                    return None;
                }
                let utm = wgs84_to_utm(self.lat, self.lng).ok()?;
                if self.system == CoordSystem::Utm {
                    Some(utm.text().split(' ').map(ToOwned::to_owned).collect())
                } else {
                    Some(vec![utm_to_mgrs(&utm).ok()?.text()])
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
        assert_eq!(
            CoordSystem::ALL.map(CoordSystem::fly_id),
            ["fly-coords-geo", "fly-coords-utm", "fly-coords-mgrs"]
        );
    }

    /// `Vertical = false`, the flight screen's: the position on one line - GEO's two numbers a
    /// space apart, UTM's `ToString()`, the MGRS reference - then three spaces and the height,
    /// and no `AltSource`.
    #[test]
    fn horizontal_is_one_line_with_three_spaces_before_the_height() {
        let geo = at_cmac(CoordSystem::Geo);
        assert_eq!(geo.line(), "-35.3632621 149.1652374   584.00m");
        assert!(!geo.line().contains("SRTM"));
        let utm = at_cmac(CoordSystem::Utm);
        assert_eq!(
            utm.line(),
            format!("{}   584.00m", utm.lines()[..3].join(" "))
        );
        let mgrs = at_cmac(CoordSystem::Mgrs);
        assert_eq!(mgrs.line(), format!("{}   584.00m", mgrs.lines()[0]));
    }

    /// On the equator at zone 31's meridian the transform's numbers are round: 500 km east and 0
    /// north, which `UTM.ToString()` writes as `500000` and `0`; in MGRS square EA, its digits all
    /// zeros and so all dropped.
    #[test]
    fn on_the_equator_at_a_zones_meridian_the_numbers_are_round() {
        let mut coords = Coords {
            system: CoordSystem::Utm,
            lat: 0.0,
            lng: 3.0,
            ..Coords::default()
        };
        assert_eq!(coords.line(), "31N 500000 0   0.00m");
        assert_eq!(coords.lines(), vec!["31N", "500000", "0", "0.00m"]);
        coords.system = CoordSystem::Mgrs;
        assert_eq!(coords.line(), "31NEA   0.00m");
        coords.system = CoordSystem::Geo;
        assert_eq!(coords.line(), "0.0000000 3.0000000   0.00m");
    }

    #[test]
    fn horizontal_draws_nothing_where_on_paint_returns() {
        let mut polar = at_cmac(CoordSystem::Utm);
        polar.lat = 85.0;
        assert_eq!(polar.line(), "");
        polar.system = CoordSystem::Mgrs;
        assert_eq!(polar.line(), "");
        let mut antimeridian = at_cmac(CoordSystem::Utm);
        antimeridian.lng = 180.0;
        assert_eq!(antimeridian.line(), "");
    }
}
