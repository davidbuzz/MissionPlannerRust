//! Mission Planner's `Commands` grid, over a mission held in wire order.
//!
//! The C# keeps home out of the grid: `Commands` holds waypoint 1 onwards, home lives in three
//! text boxes, and every handler that numbers a row adds or subtracts one for it - "home is 0"
//! (`GCSViews/FlightPlanner.cs:3130`). A mission held here is the list the vehicle and the
//! `.waypoints` file carry, where home is item 0. So an edit the C# makes to `Commands.Rows` is an
//! edit to the items after home, and `first_row` says where those start: 1 when item 0 is home, 0
//! when the list has no home record at all (a plan drawn from nothing, where the first item the
//! operator placed is the first row).
//!
//! Sequence numbers are not touched here; [`renumber`] is for the caller to run after an edit.

use mp_units::LatLon;

use crate::commands::WAYPOINT;
use crate::item::MissionItem;

/// Where "Insert WP after wp#" puts the new row, as a list index, or `None` if the C# would
/// refuse the number.
///
/// `Commands.Rows.Insert(int.Parse(wpno), 1)`: the typed number is a grid row index, which is also
/// the number of the waypoint the new row follows, because row `n` is waypoint `n + 1`. `int.Parse`
/// takes surrounding white space and a sign and nothing fractional; `Rows.Insert` takes `0` to
/// `Rows.Count` and throws otherwise, which the handler reports as "Invalid insert position".
/// `// C#: GCSViews/FlightPlanner.cs:4072-4083`
#[must_use]
pub fn insert_index(len: usize, first_row: usize, wpno: &str) -> Option<usize> {
    let row: i64 = wpno.trim().parse().ok()?;
    let row = usize::try_from(row).ok()?;
    let rows = len.saturating_sub(first_row);
    (row <= rows).then_some(first_row + row)
}

/// The number "Insert WP after wp#" offers: `(selectedrow + 1).ToString("0")`.
///
/// `selectedrow` is the grid row last added or entered, so the offer is the waypoint number of
/// that row, and accepting it inserts straight after it. With nothing selected the C#'s field
/// holds the row it last added, which is the last one; `None` stands for that here.
/// `// C#: GCSViews/FlightPlanner.cs:4072`
#[must_use]
pub fn insert_offer(len: usize, first_row: usize, selected: Option<usize>) -> usize {
    let rows = len.saturating_sub(first_row);
    // Home selected is "after home", 0 - a number the grid, which cannot select home, never
    // offers but always takes.
    selected.map_or(rows, |index| {
        (index + 1).saturating_sub(first_row).min(rows)
    })
}

/// Reverse WPs: every row taken from the end and added again, so the grid is reversed and home,
/// which is not in it, stays where it is.
///
/// `// C#: GCSViews/FlightPlanner.cs:5840-5858`
pub fn reverse(items: &mut [MissionItem], first_row: usize) {
    if let Some(rows) = items.get_mut(first_row..) {
        rows.reverse();
    }
}

/// Clear Mission: `Commands.Rows.Clear()`, which leaves home, since home is not a row.
///
/// `// C#: GCSViews/FlightPlanner.cs:2064-2082`
pub fn clear(items: &mut Vec<MissionItem>, first_row: usize) {
    items.truncate(first_row);
}

/// The positions of the rows whose command is `WAYPOINT`, in order: what Polygon > From Current
/// Waypoints makes the survey polygon from. Other commands - a loiter, a spline, a land - are
/// passed over, as the C#'s `== MAV_CMD.WAYPOINT.ToString()` passes them.
///
/// `// C#: GCSViews/FlightPlanner.cs:3619-3631`
#[must_use]
pub fn waypoint_positions(items: &[MissionItem], first_row: usize) -> Vec<LatLon> {
    items
        .get(first_row..)
        .unwrap_or_default()
        .iter()
        .filter(|item| item.command == WAYPOINT)
        .filter_map(|item| item.position().ok().flatten())
        .collect()
}

/// Numbers the items 0 to n, so the sequence has no gaps; a vehicle refuses a mission with one.
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

    /// Home and three waypoints, numbered as they would arrive from a vehicle.
    fn with_home() -> Vec<MissionItem> {
        let mut items = vec![
            MissionItem {
                frame: 0,
                ..waypoint(at(-35.0, 149.0), 580.0, 0)
            },
            waypoint(at(-35.1, 149.1), 100.0, 3),
            waypoint(at(-35.2, 149.2), 100.0, 3),
            waypoint(at(-35.3, 149.3), 100.0, 3),
        ];
        renumber(&mut items);
        items
    }

    /// The typed number is the waypoint the new row follows; home is 0, so 0 puts it first.
    #[test]
    fn insert_after_a_waypoint_number_lands_straight_after_it() {
        // Home plus three waypoints: rows 0..=2, so 0 to 3 are all valid.
        assert_eq!(insert_index(4, 1, "0"), Some(1), "after home");
        assert_eq!(insert_index(4, 1, "2"), Some(3), "after waypoint 2");
        assert_eq!(
            insert_index(4, 1, "3"),
            Some(4),
            "after the last one, at the end"
        );
        // int.Parse's white space and sign.
        assert_eq!(insert_index(4, 1, " 2 "), Some(3));
        assert_eq!(insert_index(4, 1, "+2"), Some(3));
    }

    /// Everything `Rows.Insert` or `int.Parse` would throw on is refused.
    #[test]
    fn a_number_the_c_sharp_would_throw_on_is_refused() {
        for wpno in ["4", "-1", "", "two", "2.0", "1e1"] {
            assert_eq!(insert_index(4, 1, wpno), None, "{wpno:?} should be refused");
        }
    }

    /// Without a home record the first item is the first row, and an empty plan takes row 0.
    #[test]
    fn a_plan_without_home_numbers_its_rows_from_the_first_item() {
        assert_eq!(insert_index(0, 0, "0"), Some(0));
        assert_eq!(insert_index(0, 0, "1"), None);
        assert_eq!(insert_index(3, 0, "1"), Some(1));
        assert_eq!(insert_index(3, 0, "3"), Some(3));
        assert_eq!(insert_index(3, 0, "4"), None);
    }

    /// The offer is "after the selected row", or after the last row with nothing selected.
    #[test]
    fn the_offered_number_inserts_after_the_selected_row() {
        // Home plus three: waypoint 2 (index 2) selected offers 2, which inserts at index 3.
        assert_eq!(insert_offer(4, 1, Some(2)), 2);
        assert_eq!(
            insert_index(4, 1, &insert_offer(4, 1, Some(2)).to_string()),
            Some(3)
        );
        // Nothing selected: after the last.
        assert_eq!(insert_offer(4, 1, None), 3);
        // Home itself selected: after home, which is 0 and inserts at index 1.
        assert_eq!(insert_offer(4, 1, Some(0)), 0);
        assert_eq!(insert_index(4, 1, "0"), Some(1));
        // No home: index 0 selected offers 1, which inserts at index 1.
        assert_eq!(insert_offer(3, 0, Some(0)), 1);
        assert_eq!(insert_index(3, 0, "1"), Some(1));
        // An empty grid offers 0, the only number it takes.
        assert_eq!(insert_offer(0, 0, None), 0);
        assert_eq!(insert_offer(1, 1, None), 0);
    }

    /// Reversing leaves home first and turns the rows round.
    #[test]
    fn reversing_turns_the_rows_round_and_leaves_home_first() {
        let mut items = with_home();
        reverse(&mut items, 1);
        let latitudes: Vec<f64> = items.iter().map(|item| item.x).collect();
        assert_eq!(latitudes, vec![-35.0, -35.3, -35.2, -35.1]);
        // Without home, everything is a row.
        let mut items = with_home();
        reverse(&mut items, 0);
        assert!((items[0].x - -35.3).abs() < f64::EPSILON);
        assert!((items[3].x - -35.0).abs() < f64::EPSILON);
    }

    /// Reversing a list shorter than its first row does nothing and does not panic.
    #[test]
    fn reversing_nothing_is_nothing() {
        let mut items: Vec<MissionItem> = Vec::new();
        reverse(&mut items, 1);
        assert!(items.is_empty());
    }

    /// Clear Mission empties the grid; home, not being a row, survives it.
    #[test]
    fn clearing_the_mission_keeps_home() {
        let mut items = with_home();
        clear(&mut items, 1);
        assert_eq!(items.len(), 1);
        assert!((items[0].z - 580.0).abs() < f64::EPSILON);
        let mut items = with_home();
        clear(&mut items, 0);
        assert!(items.is_empty());
    }

    /// Only WAYPOINT rows become polygon corners, and home is not one of them.
    #[test]
    fn only_waypoint_rows_make_the_polygon() {
        let mut items = with_home();
        items.insert(2, loiter_time(at(-35.9, 149.9), 50.0, 3, 5.0));
        renumber(&mut items);
        assert_eq!(items[2].command, LOITER_TIME);
        let corners = waypoint_positions(&items, 1);
        assert_eq!(
            corners,
            vec![at(-35.1, 149.1), at(-35.2, 149.2), at(-35.3, 149.3)]
        );
    }

    /// Renumbering leaves no gap and no repeat.
    #[test]
    fn renumbering_counts_from_zero() {
        let mut items = with_home();
        items.remove(1);
        renumber(&mut items);
        let seqs: Vec<u16> = items.iter().map(|item| item.seq).collect();
        assert_eq!(seqs, vec![0, 1, 2]);
    }
}
