//! Mission Planner's `Commands` grid, and the home that is kept apart from it.
//!
//! The C# keeps home out of the grid. `Commands` holds waypoint 1 onwards, its row headers read
//! `(a + 1)` (`updateRowNumbers`, `GCSViews/FlightPlanner.cs:7316`), and home lives in the three
//! Home Location boxes, `TXT_homelat`, `TXT_homelng` and `TXT_homealt`. Every handler that
//! writes a mission puts home back in front - to the vehicle (`saveWPs`, `:6196-6227`) and to a
//! `.waypoints` file (`savewaypoints`, `:6108-6122`) - and every handler that reads one takes
//! item 0 off again and offers it to the boxes (`processToScreen`, `:5634-5671`).
//!
//! So the grid can never hand its first row to the vehicle as home: home is not a row, and a
//! mission drawn on an empty map starts at waypoint 1 whatever the boxes hold.
//!
//! The rows here are held the same way - a list with no home in it, numbered from 1 by
//! [`number_rows`] - and [`upload_list`] and [`split_home`] are the two crossings between that and
//! the list the vehicle and the file carry, where home is item 0.

use mp_units::LatLon;

use crate::commands::WAYPOINT;
use crate::item::{MAV_FRAME_GLOBAL, MissionItem};

/// Home as the Home Location boxes hold it once parsed, or as `cs.PlannedHomeLocation` and
/// `cs.HomeLocation` hold it: latitude, longitude, and altitude above mean sea level.
///
/// All three zero is the C#'s `new PointLatLngAlt()`, "no home".
#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub struct Home {
    /// Latitude in degrees.
    pub lat: f64,
    /// Longitude in degrees.
    pub lng: f64,
    /// Altitude in metres above mean sea level.
    pub alt: f64,
}

impl Home {
    /// The boxes as `BUT_write_Click` reads them before a write: each one parsed, or no home at
    /// all if any of them throws - which is "Your home location is invalid".
    ///
    /// `double.Parse` and `float.Parse` take surrounding white space, a sign, a decimal point and
    /// an exponent, as Rust's parse does. Two differences, both in what is refused: the C#'s
    /// culture-dependent thousands separator (`"1,000"`), and the words it spells infinity and
    /// not-a-number in - a home at either is refused here rather than written into a file.
    /// `// C#: GCSViews/FlightPlanner.cs:658-671`
    #[must_use]
    pub fn parse(lat: &str, lng: &str, alt: &str) -> Option<Self> {
        Some(Self {
            lat: parse_number(lat)?,
            lng: parse_number(lng)?,
            alt: parse_number(alt)?,
        })
    }

    /// Whether this is a home at all: `Lat != 0 && Lng != 0`, the test `updateHomeText` puts to
    /// the vehicle's home and the planned one before taking either.
    /// `// C#: GCSViews/FlightPlanner.cs:7198, 7208-7209`
    #[must_use]
    pub fn is_set(&self) -> bool {
        self.lat != 0.0 && self.lng != 0.0
    }

    /// The item `saveWPs` puts at 0: a `WAYPOINT` in the `GLOBAL` frame at home, altitude above
    /// sea level, every parameter zero.
    /// `// C#: GCSViews/FlightPlanner.cs:6198-6206`
    #[must_use]
    pub fn item(self) -> MissionItem {
        MissionItem {
            seq: 0,
            current: 0,
            frame: MAV_FRAME_GLOBAL,
            command: WAYPOINT,
            x: self.lat,
            y: self.lng,
            z: self.alt,
            ..MissionItem::default()
        }
    }
}

/// A number as `double.Parse` reads one, for the purposes of [`Home::parse`].
#[must_use]
pub fn parse_number(text: &str) -> Option<f64> {
    text.trim()
        .parse::<f64>()
        .ok()
        .filter(|number| number.is_finite())
}

/// The list `saveWPs` hands the upload: home at 0 when there is one to insert, then every row,
/// numbered from zero, each sent with `current` 0 and `autocontinue` 1.
///
/// `commandlist.Insert(0, home)` only for a mission on an ArduPilot autopilot; any other
/// autopilot is sent the rows alone, so its first waypoint is item 0. `mav_mission.upload` sends
/// every item through `setWPAsync(..., current: 0, autocontinue: 1)`, whatever the grid had.
/// `// C#: GCSViews/FlightPlanner.cs:6219-6227; ExtLibs/ArduPilot/mav_mission.cs:101`
#[must_use]
pub fn upload_list(home: Option<Home>, rows: &[MissionItem]) -> Vec<MissionItem> {
    let mut items: Vec<MissionItem> = home
        .map(Home::item)
        .into_iter()
        .chain(rows.iter().copied())
        .collect();
    for item in &mut items {
        item.current = 0;
        item.autocontinue = 1;
    }
    renumber(&mut items);
    items
}

/// A list read from a vehicle or a file, taken apart as `processToScreen` takes it: item 0 is
/// home and leaves the grid (`Commands.Rows.Remove(Commands.Rows[0]); // remove home row`), and
/// the rows after it are the grid, numbered from 1.
///
/// The grid stops at the first row after home whose command is 0 or 255 - "0 and not home", and
/// "bad record - never loaded any WP's". A blank home, the item `ReadWaypointFile` inserts when a
/// file does not start at 0, is command 0 and is home, so it does not stop anything.
/// `// C#: GCSViews/FlightPlanner.cs:5518-5525, 5634-5671`
#[must_use]
pub fn split_home(items: &[MissionItem]) -> (Option<MissionItem>, Vec<MissionItem>) {
    let mut items = items.iter();
    let home = items.next().copied();
    let mut rows: Vec<MissionItem> = items
        .take_while(|item| item.command != 0 && item.command != 255)
        .copied()
        .collect();
    number_rows(&mut rows);
    (home, rows)
}

/// Where "Insert WP after wp#" puts the new row, as a row index, or `None` if the C# would refuse
/// the number.
///
/// `Commands.Rows.Insert(int.Parse(wpno), 1)`: the typed number is a grid row index, which is also
/// the number of the waypoint the new row follows, because row `n` is waypoint `n + 1` and home is
/// 0. `int.Parse` takes surrounding white space and a sign and nothing fractional; `Rows.Insert`
/// takes `0` to `Rows.Count` and throws otherwise, which the handler reports as "Invalid insert
/// position".
/// `// C#: GCSViews/FlightPlanner.cs:4072-4083`
#[must_use]
pub fn insert_index(rows: usize, wpno: &str) -> Option<usize> {
    let row: i64 = wpno.trim().parse().ok()?;
    let row = usize::try_from(row).ok()?;
    (row <= rows).then_some(row)
}

/// The number "Insert WP after wp#" offers: `(selectedrow + 1).ToString("0")`.
///
/// `selectedrow` is the grid row last added or entered, so the offer is the waypoint number of
/// that row, and accepting it inserts straight after it. With nothing selected the C#'s field
/// holds the row it last added, which is the last one; `None` stands for that here.
/// `// C#: GCSViews/FlightPlanner.cs:4072`
#[must_use]
pub fn insert_offer(rows: usize, selected: Option<usize>) -> usize {
    selected.map_or(rows, |row| (row + 1).min(rows))
}

/// The positions of the rows whose command is `WAYPOINT`, in order: what Polygon > From Current
/// Waypoints makes the survey polygon from. Other commands - a loiter, a spline, a land - are
/// passed over, as the C#'s `== MAV_CMD.WAYPOINT.ToString()` passes them. Home is not a row, so
/// it is never a corner.
///
/// `// C#: GCSViews/FlightPlanner.cs:3619-3631`
#[must_use]
pub fn waypoint_positions(rows: &[MissionItem]) -> Vec<LatLon> {
    rows.iter()
        .filter(|item| item.command == WAYPOINT)
        .filter_map(|item| item.position().ok().flatten())
        .collect()
}

/// Numbers the grid's rows as its headers read, from 1: row `a` is waypoint `a + 1`, because home
/// is 0.
/// `// C#: GCSViews/FlightPlanner.cs:7307-7325`
pub fn number_rows(rows: &mut [MissionItem]) {
    for (index, item) in rows.iter_mut().enumerate() {
        item.seq = u16::try_from(index + 1).unwrap_or(u16::MAX);
    }
}

/// Numbers a list the vehicle or a file carries 0 to n, so the sequence has no gaps; a vehicle
/// refuses a mission with one.
pub fn renumber(items: &mut [MissionItem]) {
    for (index, item) in items.iter_mut().enumerate() {
        item.seq = u16::try_from(index).unwrap_or(u16::MAX);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::commands::{LOITER_TIME, loiter_time, waypoint};

    fn at(latitude: f64, longitude: f64) -> LatLon {
        LatLon::new(latitude, longitude).expect("valid")
    }

    /// Three waypoints, as the grid holds them.
    fn rows() -> Vec<MissionItem> {
        let mut rows = vec![
            waypoint(at(-35.1, 149.1), 100.0, 3),
            waypoint(at(-35.2, 149.2), 100.0, 3),
            waypoint(at(-35.3, 149.3), 100.0, 3),
        ];
        number_rows(&mut rows);
        rows
    }

    fn canberra() -> Home {
        Home {
            lat: -35.363_262,
            lng: 149.165_237,
            alt: 584.0,
        }
    }

    /// The grid numbers its rows from 1, because home is 0.
    #[test]
    fn rows_are_numbered_from_one() {
        let seqs: Vec<u16> = rows().iter().map(|item| item.seq).collect();
        assert_eq!(seqs, vec![1, 2, 3]);
    }

    /// What the vehicle is sent: home first, from the boxes, as a GLOBAL waypoint; the rows after
    /// it, numbered on from 1, every item current 0 and autocontinue 1.
    #[test]
    fn the_upload_has_home_at_zero_and_the_rows_after_it() {
        let mut grid = rows();
        grid[1].current = 1;
        grid[1].autocontinue = 0;
        let sent = upload_list(Some(canberra()), &grid);
        assert_eq!(sent.len(), 4);
        let home = sent[0];
        assert_eq!((home.seq, home.command, home.frame), (0, 16, 0));
        assert_eq!((home.x, home.y, home.z), (-35.363_262, 149.165_237, 584.0));
        assert_eq!(
            (home.param1, home.param2, home.param3, home.param4),
            (0.0, 0.0, 0.0, 0.0)
        );
        assert!((sent[1].x - -35.1).abs() < 1e-12, "the first row is item 1");
        let seqs: Vec<u16> = sent.iter().map(|item| item.seq).collect();
        assert_eq!(seqs, vec![0, 1, 2, 3]);
        assert!(sent.iter().all(|item| item.current == 0));
        assert!(sent.iter().all(|item| item.autocontinue == 1));
    }

    /// Any autopilot but ArduPilot is sent the rows alone, and its first waypoint is item 0.
    #[test]
    fn without_home_the_rows_are_numbered_from_zero() {
        let sent = upload_list(None, &rows());
        assert_eq!(sent.len(), 3);
        assert!((sent[0].x - -35.1).abs() < 1e-12);
        let seqs: Vec<u16> = sent.iter().map(|item| item.seq).collect();
        assert_eq!(seqs, vec![0, 1, 2]);
    }

    /// Reading takes item 0 off as home and numbers what follows from 1.
    #[test]
    fn a_read_list_loses_home_to_the_boxes() {
        let sent = upload_list(Some(canberra()), &rows());
        let (home, grid) = split_home(&sent);
        let home = home.expect("item 0");
        assert!((home.z - 584.0).abs() < 1e-12);
        assert_eq!(grid, rows());
    }

    /// A row with command 0 or 255 ends the grid; home may be either and still is home.
    #[test]
    fn a_command_zero_or_255_after_home_ends_the_grid() {
        for stop in [0, 255] {
            let mut items = vec![MissionItem {
                command: 0,
                ..MissionItem::default()
            }];
            items.extend(rows());
            items.insert(
                3,
                MissionItem {
                    command: stop,
                    ..MissionItem::default()
                },
            );
            renumber(&mut items);
            let (home, grid) = split_home(&items);
            assert_eq!(home.map(|item| item.command), Some(0), "a blank home");
            assert_eq!(grid.len(), 2, "command {stop} stopped the grid");
        }
    }

    /// Nothing read is nothing: no home and no rows.
    #[test]
    fn an_empty_list_has_no_home() {
        assert_eq!(split_home(&[]), (None, Vec::new()));
    }

    /// The boxes parse as `double.Parse` does, and any one that does not is no home.
    #[test]
    fn the_boxes_parse_or_there_is_no_home() {
        assert_eq!(
            Home::parse(" -35.363262", "+149.165237 ", "584"),
            Some(canberra())
        );
        assert_eq!(
            Home::parse("1e1", ".5", "5."),
            Some(Home {
                lat: 10.0,
                lng: 0.5,
                alt: 5.0
            })
        );
        for (lat, lng, alt) in [
            ("", "149", "0"),
            ("-35", "", "0"),
            ("-35", "149", ""),
            ("-35", "149", "high"),
            ("NaN", "149", "0"),
            ("-35", "inf", "0"),
        ] {
            assert_eq!(Home::parse(lat, lng, alt), None, "{lat:?} {lng:?} {alt:?}");
        }
    }

    /// A home is set when neither coordinate is zero, as `updateHomeText` asks.
    #[test]
    fn a_home_needs_both_coordinates() {
        assert!(canberra().is_set());
        assert!(!Home::default().is_set());
        assert!(
            !Home {
                lat: -35.0,
                ..Home::default()
            }
            .is_set()
        );
    }

    /// The typed number is the waypoint the new row follows; home is 0, so 0 puts it first.
    #[test]
    fn insert_after_a_waypoint_number_lands_straight_after_it() {
        // Three rows: 0 to 3 are all valid.
        assert_eq!(insert_index(3, "0"), Some(0), "after home");
        assert_eq!(insert_index(3, "2"), Some(2), "after waypoint 2");
        assert_eq!(insert_index(3, "3"), Some(3), "after the last one");
        // int.Parse's white space and sign.
        assert_eq!(insert_index(3, " 2 "), Some(2));
        assert_eq!(insert_index(3, "+2"), Some(2));
        // An empty grid takes 0 only.
        assert_eq!(insert_index(0, "0"), Some(0));
        assert_eq!(insert_index(0, "1"), None);
    }

    /// Everything `Rows.Insert` or `int.Parse` would throw on is refused.
    #[test]
    fn a_number_the_c_sharp_would_throw_on_is_refused() {
        for wpno in ["4", "-1", "", "two", "2.0", "1e1"] {
            assert_eq!(insert_index(3, wpno), None, "{wpno:?} should be refused");
        }
    }

    /// The offer is "after the selected row", or after the last row with nothing selected.
    #[test]
    fn the_offered_number_inserts_after_the_selected_row() {
        // Row 1 (waypoint 2) selected offers 2, which inserts at row 2.
        assert_eq!(insert_offer(3, Some(1)), 2);
        assert_eq!(
            insert_index(3, &insert_offer(3, Some(1)).to_string()),
            Some(2)
        );
        // Nothing selected: after the last.
        assert_eq!(insert_offer(3, None), 3);
        // An empty grid offers 0, the only number it takes.
        assert_eq!(insert_offer(0, None), 0);
    }

    /// Only WAYPOINT rows become polygon corners.
    #[test]
    fn only_waypoint_rows_make_the_polygon() {
        let mut grid = rows();
        grid.insert(1, loiter_time(at(-35.9, 149.9), 50.0, 3, 5.0));
        number_rows(&mut grid);
        assert_eq!(grid[1].command, LOITER_TIME);
        let corners = waypoint_positions(&grid);
        assert_eq!(
            corners,
            vec![at(-35.1, 149.1), at(-35.2, 149.2), at(-35.3, 149.3)]
        );
    }

    /// Renumbering a wire list leaves no gap and no repeat.
    #[test]
    fn renumbering_counts_from_zero() {
        let mut items = upload_list(Some(canberra()), &rows());
        items.remove(1);
        renumber(&mut items);
        let seqs: Vec<u16> = items.iter().map(|item| item.seq).collect();
        assert_eq!(seqs, vec![0, 1, 2]);
    }
}
