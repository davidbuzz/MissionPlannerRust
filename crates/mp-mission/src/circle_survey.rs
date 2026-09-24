//! Create Circle Survey: `CircleSurveyMission.createGrid`, ported from
//! `Utilities/CircleSurveyMission.cs` @ efb0801 (GPL-3.0-or-later).
//!
//! Six `InputBox`es (start altitude 10, end altitude 20, separation 2, radius 5, photos 50, start
//! heading 0), then a `DO_SET_ROI` at the centre and, for each altitude from the start to the end
//! by the separation, a ring of `WAYPOINT`s with a two-second delay at headings from the start to
//! the start plus 360 by `360 / photos` - an integer division - each followed by a
//! `DO_DIGICAM_CONTROL` whose latitude cell is 1 and longitude 0 (`AddCommand(DO_DIGICAM_CONTROL,
//! 0, 0, 0, 0, 0, 1, 0)`, x then y). Each point is `PointLatLngAlt.newpos(heading, radius)`.
//!
//! This module gives the `AddCommand` calls in order; the screen runs them through `FillCommand`
//! and `setfromMap`, which decide the row's command and altitude as the C# does.

use std::fmt;

/// `MAV_CMD_DO_SET_ROI`.
pub const DO_SET_ROI: u16 = 201;
/// `MAV_CMD_DO_DIGICAM_CONTROL`.
pub const DO_DIGICAM_CONTROL: u16 = 203;

/// The six answers, as `createGrid` holds them.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Answers {
    /// "startalt", metres.
    pub start_alt: i32,
    /// "endalt".
    pub end_alt: i32,
    /// "seperation" (the C#'s spelling), metres between rings.
    pub separation: i32,
    /// "radius", metres.
    pub radius: i32,
    /// "photos" a ring.
    pub photos: i32,
    /// "start heading", degrees.
    pub start_heading: i32,
}

impl Default for Answers {
    /// What the six boxes offer.
    fn default() -> Self {
        Self {
            start_alt: 10,
            end_alt: 20,
            separation: 2,
            radius: 5,
            photos: 50,
            start_heading: 0,
        }
    }
}

/// The prompts, in order, with each answer's text for the box.
/// `// C#: Utilities/CircleSurveyMission.cs:17-22`
pub const PROMPTS: [&str; 6] = [
    "startalt",
    "endalt",
    "seperation",
    "radius",
    "photos",
    "start heading",
];

/// One `AddCommand(cmd, p1, p2, p3, p4, x, y, z)` call.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Command {
    /// The command.
    pub command: u16,
    /// `p1` to `p4`.
    pub params: [f64; 4],
    /// `x`, the longitude for a positioned command.
    pub x: f64,
    /// `y`, the latitude.
    pub y: f64,
    /// `z`, the altitude.
    pub z: f64,
}

/// Where the C# would not come back.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CircleSurveyError {
    /// `360 / photos` with no photos: `DivideByZeroException`.
    NoPhotos,
    /// A heading step of 0 (more than 360 photos) or a separation of 0 or less with an end above
    /// the start: the C#'s loop never ends. Refused here rather than hung.
    NeverEnds,
}

impl fmt::Display for CircleSurveyError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NoPhotos => {
                f.write_str("System.DivideByZeroException: Attempted to divide by zero.")
            }
            Self::NeverEnds => f.write_str(
                "the survey would never end: a heading step of 0, or a separation of 0 or less",
            ),
        }
    }
}

impl std::error::Error for CircleSurveyError {}

/// `PointLatLngAlt.newpos(bearing, distance)`: the point `distance` metres away on `bearing`
/// degrees, on a sphere of 6,378,100 m.
/// `// C#: ExtLibs/Utilities/PointLatLngAlt.cs:313-335`
#[must_use]
pub fn newpos(lat: f64, lng: f64, bearing: f64, distance: f64) -> (f64, f64) {
    let radius_of_earth = 6_378_100.0;
    let lat1 = lat.to_radians();
    let lon1 = lng.to_radians();
    let brng = bearing.to_radians();
    let dr = distance / radius_of_earth;
    let lat2 = (lat1.sin() * dr.cos() + lat1.cos() * dr.sin() * brng.cos()).asin();
    let lon2 =
        lon1 + (brng.sin() * dr.sin() * lat1.cos()).atan2(dr.cos() - lat1.sin() * lat2.sin());
    (lat2.to_degrees(), lon2.to_degrees())
}

/// `createGrid(centerPoint)` once the boxes are answered: the `AddCommand` calls in order.
///
/// # Errors
///
/// Where the C# divides by zero or loops without end.
/// `// C#: Utilities/CircleSurveyMission.cs:24-42`
pub fn create_grid(
    centre_lat: f64,
    centre_lng: f64,
    centre_alt: f64,
    answers: Answers,
) -> Result<Vec<Command>, CircleSurveyError> {
    if answers.photos == 0 {
        return Err(CircleSurveyError::NoPhotos);
    }
    let step = 360 / answers.photos;
    if step <= 0 || (answers.separation <= 0 && answers.start_alt <= answers.end_alt) {
        return Err(CircleSurveyError::NeverEnds);
    }
    let mut out = vec![Command {
        command: DO_SET_ROI,
        params: [0.0; 4],
        x: centre_lng,
        y: centre_lat,
        z: centre_alt,
    }];
    let mut alt = answers.start_alt;
    while alt <= answers.end_alt {
        let mut heading = answers.start_heading;
        while heading <= answers.start_heading + 360 {
            let (lat, lng) = newpos(
                centre_lat,
                centre_lng,
                f64::from(heading),
                f64::from(answers.radius),
            );
            out.push(Command {
                command: crate::commands::WAYPOINT,
                params: [2.0, 0.0, 0.0, 0.0],
                x: lng,
                y: lat,
                z: f64::from(alt),
            });
            out.push(Command {
                command: DO_DIGICAM_CONTROL,
                params: [0.0; 4],
                x: 0.0,
                y: 1.0,
                z: 0.0,
            });
            heading += step;
        }
        alt += answers.separation;
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_defaults_make_a_roi_and_six_rings_of_fifty_two_photos() {
        let rows = create_grid(-35.363, 149.165, 0.0, Answers::default()).expect("a survey");
        // 360 / 50 = 7: headings 0, 7, ..., 357 - 52 of them; altitudes 10, 12, ..., 20 - six.
        assert_eq!(rows.len(), 1 + 6 * 52 * 2);
        assert_eq!(rows[0].command, DO_SET_ROI);
        assert_eq!((rows[0].x, rows[0].y, rows[0].z), (149.165, -35.363, 0.0));
        assert_eq!(rows[1].command, crate::commands::WAYPOINT);
        assert_eq!(rows[1].params, [2.0, 0.0, 0.0, 0.0]);
        assert_eq!(rows[1].z, 10.0);
        // Heading 0 from the centre: due north by 5 m.
        assert!((rows[1].y - -35.363).abs() > 4e-5 && (rows[1].y - -35.363).abs() < 5e-5);
        assert!((rows[1].x - 149.165).abs() < 1e-9);
        assert_eq!(rows[2].command, DO_DIGICAM_CONTROL);
        assert_eq!((rows[2].x, rows[2].y, rows[2].z), (0.0, 1.0, 0.0));
        // The last ring is at 20 m.
        assert_eq!(rows[rows.len() - 2].z, 20.0);
    }

    #[test]
    fn four_photos_close_the_ring_on_the_start_heading() {
        let answers = Answers {
            start_alt: 5,
            end_alt: 5,
            photos: 4,
            start_heading: 90,
            ..Answers::default()
        };
        let rows = create_grid(0.0, 0.0, 3.0, answers).expect("a survey");
        // 90, 180, 270, 360, 450: five, the last where the first was.
        assert_eq!(rows.len(), 1 + 5 * 2);
        assert_eq!(rows[0].z, 3.0);
        assert!((rows[1].x - rows[9].x).abs() < 1e-9);
        assert!((rows[1].y - rows[9].y).abs() < 1e-9);
    }

    #[test]
    fn what_the_c_sharp_cannot_finish_is_refused() {
        let none = Answers {
            photos: 0,
            ..Answers::default()
        };
        assert_eq!(
            create_grid(0.0, 0.0, 0.0, none),
            Err(CircleSurveyError::NoPhotos)
        );
        let too_many = Answers {
            photos: 400,
            ..Answers::default()
        };
        assert_eq!(
            create_grid(0.0, 0.0, 0.0, too_many),
            Err(CircleSurveyError::NeverEnds)
        );
        let flat = Answers {
            separation: 0,
            ..Answers::default()
        };
        assert_eq!(
            create_grid(0.0, 0.0, 0.0, flat),
            Err(CircleSurveyError::NeverEnds)
        );
        // A start above the end with no separation ends at once: only the ROI.
        let upside_down = Answers {
            start_alt: 30,
            end_alt: 20,
            separation: 0,
            ..Answers::default()
        };
        assert_eq!(
            create_grid(0.0, 0.0, 0.0, upside_down).map(|rows| rows.len()),
            Ok(1)
        );
    }

    #[test]
    fn newpos_is_the_c_sharps_sphere() {
        // 1,000 m due east at the equator: 1000 / 6378100 radians of longitude.
        let (lat, lng) = newpos(0.0, 0.0, 90.0, 1000.0);
        assert!(lat.abs() < 1e-12);
        assert!((lng - (1000.0 / 6_378_100.0_f64).to_degrees()).abs() < 1e-12);
    }
}
