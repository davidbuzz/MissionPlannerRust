//! The QGC WPL 110 file format.
//!
//! This is the `.waypoints` / `.txt` mission format Mission Planner, QGroundControl, MAVProxy and
//! ArduPilot's own test suite all read and write. Users have years of these files, so reading them
//! is not a feature, it is a prerequisite (`DELIVERABLES.md` D17).
//!
//! # Layout
//!
//! A header line, then one tab-separated record per item:
//!
//! ```text
//! QGC WPL 110
//! seq  current  frame  command  p1  p2  p3  p4  x(lat)  y(lon)  z(alt)  autocontinue
//! ```
//!
//! # The quirks, all reproduced from `ExtLibs/Utilities/MissionFile.cs` and `FlightPlanner.cs`
//!
//! * Fields may be separated by tabs, spaces **or** commas, with runs collapsed. Files in the wild
//!   use all three.
//! * Lines beginning with `#` are comments.
//! * A line with fewer than ten fields is skipped rather than failing the load. Trailing blank
//!   lines are common and must not break a file.
//! * If the first record is not sequence 0, a blank home item is inserted, because everything
//!   downstream assumes index 0 is home.
//! * Command id 99 is rewritten to 0. This is a legacy Mission Planner marker, and a file
//!   containing it loads differently if the rule is not applied.
//! * Mission Planner writes latitude and longitude with **eight** decimal places on normal items
//!   but **seven** on the home item, and altitude with six throughout. Reproduced exactly, because
//!   a diff against a file the C# app wrote should be empty.

use crate::item::MissionItem;

/// The only header this format has ever had.
pub const HEADER: &str = "QGC WPL 110";

/// Legacy marker Mission Planner writes for "no command"; it loads as 0.
const LEGACY_EMPTY_COMMAND: u16 = 99;

/// Fields in a complete record.
const FIELD_COUNT: usize = 12;

/// Why a waypoint file could not be read.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum WaypointFileError {
    /// The file does not begin with a QGC WPL header.
    #[error("not a waypoint file: expected a line containing \"QGC WPL\", found {found:?}")]
    BadHeader {
        /// The first line as found, truncated.
        found: String,
    },
    /// A record could not be parsed.
    #[error("line {line}: {reason}")]
    BadRecord {
        /// One-based line number, so it matches what an editor shows.
        line: usize,
        /// What went wrong.
        reason: String,
    },
}

/// Parses a waypoint file.
pub fn read_waypoints(text: &str) -> Result<Vec<MissionItem>, WaypointFileError> {
    let mut lines = text.lines().enumerate();

    let Some((_, header)) = lines.next() else {
        return Err(WaypointFileError::BadHeader {
            found: String::new(),
        });
    };
    if !header.contains("QGC WPL") {
        return Err(WaypointFileError::BadHeader {
            found: header.chars().take(40).collect(),
        });
    }

    let mut items: Vec<MissionItem> = Vec::new();
    for (index, line) in lines {
        let line_number = index + 1;
        let trimmed = line.trim();
        if trimmed.is_empty() || trimmed.starts_with('#') {
            continue;
        }

        let fields: Vec<&str> = trimmed
            .split(['\t', ' ', ','])
            .filter(|f| !f.is_empty())
            .collect();
        // Short lines are skipped, not rejected: the reference implementation does the same, and
        // truncated trailing lines are common in files people have edited by hand.
        if fields.len() < FIELD_COUNT - 2 {
            continue;
        }

        let parse_u16 = |i: usize| -> Result<u16, WaypointFileError> {
            field(&fields, i, line_number)?.parse::<f64>().map_or_else(
                |e| Err(bad(line_number, format!("field {i}: {e}"))),
                |v| Ok(saturating_u16(v)),
            )
        };
        let parse_u8 = |i: usize| -> Result<u8, WaypointFileError> {
            field(&fields, i, line_number)?.parse::<f64>().map_or_else(
                |e| Err(bad(line_number, format!("field {i}: {e}"))),
                |v| Ok(saturating_u8(v)),
            )
        };
        let parse_f64 = |i: usize| -> Result<f64, WaypointFileError> {
            field(&fields, i, line_number)?
                .parse::<f64>()
                .map_err(|e| bad(line_number, format!("field {i}: {e}")))
        };

        let seq = parse_u16(0)?;
        // A file whose first record is not home gets a blank home inserted, because every screen
        // and every upload path indexes from home at zero.
        if items.is_empty() && seq != 0 {
            items.push(MissionItem {
                seq: 0,
                ..MissionItem::default()
            });
        }

        let mut command = parse_u16(3)?;
        if command == LEGACY_EMPTY_COMMAND {
            command = 0;
        }

        items.push(MissionItem {
            seq,
            current: parse_u8(1)?,
            frame: parse_u8(2)?,
            command,
            param1: parse_f64(4)?,
            param2: parse_f64(5)?,
            param3: parse_f64(6)?,
            param4: parse_f64(7)?,
            x: parse_f64(8)?,
            y: parse_f64(9)?,
            z: parse_f64(10)?,
            autocontinue: saturating_u8(
                fields
                    .get(11)
                    .and_then(|f| f.parse::<f64>().ok())
                    .unwrap_or(1.0),
            ),
        });
    }

    Ok(items)
}

/// Narrows to `u16`, saturating. A corrupt field must not wrap: command 70000 becoming 4464 is a
/// different, valid command, and a mission that quietly changes meaning is worse than one that
/// refuses to load.
#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
fn saturating_u16(value: f64) -> u16 {
    if value.is_nan() {
        0
    } else {
        value.clamp(0.0, f64::from(u16::MAX)).round() as u16
    }
}

/// Narrows to `u8`, saturating, for the same reason.
#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
fn saturating_u8(value: f64) -> u8 {
    if value.is_nan() {
        0
    } else {
        value.clamp(0.0, f64::from(u8::MAX)).round() as u8
    }
}

fn field<'a>(fields: &[&'a str], index: usize, line: usize) -> Result<&'a str, WaypointFileError> {
    fields
        .get(index)
        .copied()
        .ok_or_else(|| bad(line, format!("missing field {index}")))
}

fn bad(line: usize, reason: String) -> WaypointFileError {
    WaypointFileError::BadRecord { line, reason }
}

/// Writes a waypoint file in Mission Planner's exact format.
///
/// Lines end with `\n`. The reference implementation uses `StreamWriter.WriteLine`, which emits
/// `\r\n` on Windows; readers everywhere trim whitespace, and a mission file that differs only by
/// line ending between platforms would make every cross-platform diff noisy.
#[must_use]
pub fn write_waypoints(items: &[MissionItem]) -> String {
    let mut out = String::with_capacity(items.len() * 96 + HEADER.len() + 1);
    out.push_str(HEADER);
    out.push('\n');

    for item in items {
        // The home item is written with seven decimal places of latitude and longitude, every
        // other item with eight. That is not a typo: the two are written by different code paths
        // in FlightPlanner.cs, and reproducing it keeps a diff against a C#-written file empty.
        let (coord_decimals, param_decimals) = if item.is_home() { (7, 0) } else { (8, 8) };

        out.push_str(&format!(
            "{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{:.6}\t{}\n",
            item.seq,
            item.current,
            item.frame,
            item.command,
            decimals(item.param1, param_decimals),
            decimals(item.param2, param_decimals),
            decimals(item.param3, param_decimals),
            decimals(item.param4, param_decimals),
            decimals(item.x, coord_decimals),
            decimals(item.y, coord_decimals),
            item.z,
            item.autocontinue,
        ));
    }
    out
}

/// Formats a number with a fixed number of decimals, or as a bare integer when asked for none.
fn decimals(value: f64, places: usize) -> String {
    if places == 0 {
        // The home row writes its four parameters as plain `0`, not `0.00000000`.
        #[allow(clippy::cast_possible_truncation)]
        let whole = value.clamp(f64::from(i32::MIN), f64::from(i32::MAX)) as i64;
        format!("{whole}")
    } else {
        format!("{value:.places$}")
    }
}
