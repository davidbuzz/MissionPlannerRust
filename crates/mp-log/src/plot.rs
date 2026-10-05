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

//! Pulling a plottable series out of a dataflash log.
//!
//! The other half of the tuning graph: watching a value live is how a problem is noticed, and
//! plotting it afterwards is how it is diagnosed. `mp_chart` already holds the reduction and the
//! auto-ranging, so this is the extraction - which fields a log contains, and the samples for one
//! of them against time.
//!
//! **The field list comes from the log, not from a table.** ArduPilot's message set changes
//! between releases and every log declares its own formats in `FMT` messages, so a hard-coded
//! list of "the fields worth plotting" is a list that is wrong for some logs and silently short
//! for others. Reading what is actually there costs one pass and cannot go stale.

use crate::dataflash::{DataflashReader, Value};

/// The field every timestamped dataflash message carries, in microseconds since boot.
///
/// Not every message has one - `FMT`, `PARM` and `FILE` do not - and those are simply not
/// plottable against time, which is the right answer rather than an error.
const TIME_FIELD: &str = "TimeUS";

/// One field that can be plotted.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub struct PlottableField {
    /// The message it belongs to, e.g. `ATT`.
    pub message: String,
    /// Which instance, for a message that has several - `IMU[0]`, `IMU[1]`, `IMU[2]`.
    ///
    /// `None` for a message type that declares no instance field.
    pub instance: Option<i64>,
    /// The field label, e.g. `Roll`.
    pub field: String,
    /// How many samples the log holds.
    pub samples: usize,
}

impl std::fmt::Display for PlottableField {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self.instance {
            Some(instance) => write!(f, "{}[{instance}].{}", self.message, self.field),
            None => write!(f, "{}.{}", self.message, self.field),
        }
    }
}

/// Which message types carry several instances, and which field says which.
///
/// **Without this, a log with three IMUs plots all three as one trace.** ArduPilot writes one
/// `VIBE` record per IMU, one `MAG` per compass and one `XKF*` per EKF core, distinguished only by
/// an instance field - so a series built by message name alone interleaves three different sensors
/// and the result looks like noise that is not there. PLAN.md §8.1 names this as a designed-in
/// trap; this is where it is avoided.
///
/// The instance field is not guessable from the name: it is `I` on `MAG`, `IMU` on `VIBE`,
/// `Instance` on `BAT`, `C` on the EKF cores, `chan` on `MAV` and `NodeId` on `CAND`. ArduPilot
/// declares it properly, in `FMTU`: a `#` in the `UnitIds` string marks the field, positionally.
/// `// C#: ExtLibs/Utilities/DFLogBuffer.cs:230-250`
#[must_use]
pub(crate) fn instance_fields(data: &[u8]) -> std::collections::BTreeMap<String, String> {
    let mut by_type: std::collections::BTreeMap<i64, usize> = std::collections::BTreeMap::new();
    let mut reader = DataflashReader::new(data);
    while let Some(message) = reader.next_message() {
        if message.name != "FMTU" {
            continue;
        }
        let (Some(format_type), Some(units)) = (
            message.field("FmtType").and_then(Value::as_f64),
            message.field("UnitIds").and_then(Value::as_text),
        ) else {
            continue;
        };
        if let Some(index) = units.trim().find('#') {
            #[allow(clippy::cast_possible_truncation)] // a format type is a byte
            by_type.insert(format_type as i64, index);
        }
    }

    instance_labels(&by_type, reader.formats())
}

/// The instance field of each format, by name: the position each `FMTU` marks, looked up in the
/// whole log's formats.
///
/// FMTU names the format by number; the label it points at comes from that format's own
/// declaration, which the reader has collected by the time the walk is done.
pub(crate) fn instance_labels(
    by_type: &std::collections::BTreeMap<i64, usize>,
    formats: &std::collections::BTreeMap<u8, crate::dataflash::MessageFormat>,
) -> std::collections::BTreeMap<String, String> {
    let mut out = std::collections::BTreeMap::new();
    for (format_type, index) in by_type {
        if let Ok(key) = u8::try_from(*format_type)
            && let Some(format) = formats.get(&key)
            && let Some(label) = format.labels.get(*index)
        {
            out.insert(format.name.clone(), label.clone());
        }
    }
    out
}

/// Everything in a log that can be plotted against time.
///
/// Sorted by message then field, so the list is stable between runs - an inventory that reorders
/// itself is one nobody can diff.
#[must_use]
pub fn plottable(data: &[u8]) -> Vec<PlottableField> {
    let instances = instance_fields(data);
    let mut counts: std::collections::BTreeMap<(String, Option<i64>, String), usize> =
        std::collections::BTreeMap::new();

    let mut reader = DataflashReader::new(data);
    while let Some(message) = reader.next_message() {
        // A message with no timestamp cannot be placed on a time axis. `FMT` and `FILE` are the
        // common cases and neither is something anybody plots.
        if message.field(TIME_FIELD).is_none() {
            continue;
        }
        let instance_label = instances.get(&message.name);
        #[allow(clippy::cast_possible_truncation)] // an instance number is a small integer
        let instance = instance_label
            .and_then(|label| message.field(label))
            .and_then(Value::as_f64)
            .map(|value| value as i64);

        for (label, value) in &message.fields {
            // The instance field is the selector, not a series. Plotting "which IMU is this"
            // against time is a staircase nobody wants.
            if label == TIME_FIELD || Some(label) == instance_label || value.as_f64().is_none() {
                continue;
            }
            *counts
                .entry((message.name.clone(), instance, label.clone()))
                .or_default() += 1;
        }
    }
    counts
        .into_iter()
        .map(|((message, instance, field), samples)| PlottableField {
            message,
            instance,
            field,
            samples,
        })
        .collect()
}

/// One extracted sample.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point {
    /// Seconds since the first timestamped message in the log: see [`seconds_since`].
    pub seconds: f64,
    /// The record it came from, counted from the start of the log with the format declarations:
    /// `DFItem.lineno`, which is what `LogBrowse` plots against when its Time box is unticked.
    /// `// C#: Log/LogBrowse.cs:1509, 1551-1561`
    pub line: usize,
    /// The value.
    pub value: f64,
}

/// The `TimeUS` of the first message in a log that has one: where its time axis starts.
///
/// One origin for the whole log, so that two fields plotted together, and the labels drawn over
/// them, line up in time as they do in the C#, which plots every one against the same clock.
#[must_use]
pub fn time_origin(data: &[u8]) -> Option<f64> {
    let mut reader = DataflashReader::new(data);
    while let Some(message) = reader.next_message() {
        if let Some(time) = message.field(TIME_FIELD).and_then(Value::as_f64) {
            return Some(time);
        }
    }
    None
}

/// Where a `TimeUS` sits on the time axis: seconds after the log's origin.
///
/// A log can contain a time that goes backwards, across a reboot within one file. Clamped rather
/// than dropped: the samples are real and a negative x would put them off the plot.
#[must_use]
pub fn seconds_since(origin: f64, time_us: f64) -> f64 {
    ((time_us - origin) / 1_000_000.0).max(0.0)
}

/// Pulls one field's samples out of a log.
///
/// Time is seconds since the **first timestamped message**, not since boot. A log downloaded from
/// a vehicle that has been powered for an hour starts at `TimeUS` = 3.6e9, and an axis labelled
/// from there tells a reader nothing they wanted to know.
#[must_use]
pub fn extract(data: &[u8], message_name: &str, field_name: &str) -> Vec<Point> {
    extract_instance(data, message_name, None, field_name)
}

/// Pulls one instance's samples out of a log.
///
/// `instance` of `None` takes every record of the message, which is right for a type that has no
/// instances and wrong for one that does - see [`plottable`]. The caller knows which, because the
/// inventory told it.
///
/// Every record is counted for its line number, and only the named message's are decoded.
#[must_use]
pub fn extract_instance(
    data: &[u8],
    message_name: &str,
    instance: Option<i64>,
    field_name: &str,
) -> Vec<Point> {
    let instance_label = instance
        .is_some()
        .then(|| instance_fields(data).get(message_name).cloned())
        .flatten();
    let Some(origin) = time_origin(data) else {
        return Vec::new();
    };
    let mut points = Vec::new();
    let mut reader = DataflashReader::new(data);
    let mut line = 0usize;
    while let Some(record) = reader.next_record() {
        let this_line = line;
        line += 1;
        let named = reader
            .formats()
            .get(&record.msg_type)
            .is_some_and(|format| format.name == message_name);
        if !named || record.msg_type == crate::dataflash::FMT_TYPE {
            continue;
        }
        let Some(message) = data
            .get(record.offset..)
            .and_then(|bytes| crate::dataflash::decode_record(reader.formats(), bytes))
        else {
            continue;
        };
        if let Some(label) = instance_label.as_deref() {
            #[allow(clippy::cast_possible_truncation)] // instance numbers are small
            let found = message
                .field(label)
                .and_then(Value::as_f64)
                .map(|value| value as i64);
            if found != instance {
                continue;
            }
        }
        let (Some(time), Some(value)) = (
            message.field(TIME_FIELD).and_then(Value::as_f64),
            message.field(field_name).and_then(Value::as_f64),
        ) else {
            continue;
        };
        if value.is_finite() {
            points.push(Point {
                seconds: seconds_since(origin, time),
                line: this_line,
                value,
            });
        }
    }
    points
}

/// The unit and scale a field is declared with.
#[derive(Debug, Clone, PartialEq)]
pub struct FieldUnit {
    /// The unit's label as the log declares it - `deg`, `m/s`, `V` - or empty when the field
    /// declares none, or the log predates `UNIT` messages.
    pub unit: String,
    /// What to multiply a stored value by to put it in that unit. 1 when there is nothing to do.
    ///
    /// ArduPilot writes `0` in a `MULT` record to mean "no multiplier", and the C# treats both 0
    /// and 1 as "leave the value alone" (`GraphItem_AddCurve` scales only when the multiplier is
    /// neither), so 0 is folded into 1 here rather than handed to a caller who would multiply by
    /// it and get a flat line.
    pub multiplier: f64,
}

impl Default for FieldUnit {
    fn default() -> Self {
        Self {
            unit: String::new(),
            multiplier: 1.0,
        }
    }
}

/// Every field's unit, as the log declares it: the C#'s `UnitMultiList`.
#[derive(Debug, Default, Clone, PartialEq)]
pub struct UnitTable {
    by_field: std::collections::BTreeMap<(String, String), FieldUnit>,
}

impl UnitTable {
    /// The unit of one field, or the unitless default: the C#'s `GetUnit`.
    /// `// C#: ExtLibs/Utilities/DFLogBuffer.cs:821-829`
    #[must_use]
    pub fn get(&self, message: &str, field: &str) -> FieldUnit {
        self.by_field
            .get(&(message.to_owned(), field.to_owned()))
            .cloned()
            .unwrap_or_default()
    }

    /// How many fields have a declared unit.
    #[must_use]
    pub fn len(&self) -> usize {
        self.by_field.len()
    }

    /// Whether the log declared any units at all.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.by_field.is_empty()
    }
}

/// Reads the units a log declares for its fields.
///
/// Three message types carry them: `FMTU` gives each format a string of unit ids and a string of
/// multiplier ids, one character per field; `UNIT` maps a unit id to its label; `MULT` maps a
/// multiplier id to a number. A field whose stored value is already scaled by its format
/// character - `c`/`C`/`e`/`E` are hundredths and `L` is a coordinate in 1e-7 degrees, all of
/// which [`crate::dataflash::FieldType::decode`] has already divided out - gets a multiplier of 1
/// whatever `MULT` says, as the C# does, or the value would be scaled twice.
/// `// C#: ExtLibs/Utilities/DFLogBuffer.cs:230-292, 503-545`
///
/// **Deliberate divergence.** In the C#, the loops that read `UNIT` and `MULT` are guarded by
/// `if (Unit.Count > 0)` and `if (Mult.Count > 0)` over dictionaries nothing ever seeds, so the
/// shipping application resolves every unit to `""` and its per-unit axes never separate. The
/// code around the guards is unambiguous about what was meant - a left axis per unit - and that
/// is what is done here. The owner can rule the other way; the site is this comment.
#[must_use]
pub fn units(data: &[u8]) -> UnitTable {
    use std::collections::BTreeMap;

    let mut fmtu: BTreeMap<u8, (String, String)> = BTreeMap::new();
    let mut unit_labels: BTreeMap<char, String> = BTreeMap::new();
    let mut multipliers: BTreeMap<char, f64> = BTreeMap::new();

    let mut reader = DataflashReader::new(data);
    while let Some(message) = reader.next_message() {
        match message.name.as_str() {
            "FMTU" => {
                if let (Some(format_type), Some(unit_ids), Some(mult_ids)) = (
                    message.field("FmtType").and_then(Value::as_f64),
                    message.field("UnitIds").and_then(Value::as_text),
                    message.field("MultIds").and_then(Value::as_text),
                ) {
                    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
                    // a format type is a byte
                    let key = format_type as u8;
                    fmtu.insert(
                        key,
                        (unit_ids.trim().to_owned(), mult_ids.trim().to_owned()),
                    );
                }
            }
            "UNIT" => {
                if let (Some(id), Some(label)) = (
                    message.field("Id").and_then(Value::as_f64),
                    message.field("Label").and_then(text_of),
                ) && let Some(id) = unit_id(id)
                {
                    unit_labels.insert(id, label);
                }
            }
            "MULT" => {
                if let (Some(id), Some(mult)) = (
                    message.field("Id").and_then(Value::as_f64),
                    message.field("Mult").and_then(Value::as_f64),
                ) && let Some(id) = unit_id(id)
                {
                    multipliers.insert(id, mult);
                }
            }
            _ => {}
        }
    }

    unit_table(&fmtu, &unit_labels, &multipliers, reader.formats())
}

/// The table `BuildUnitMultiList` makes from what `FMTU`, `UNIT` and `MULT` said last of each
/// format and id, and the whole log's formats.
/// `// C#: ExtLibs/Utilities/DFLogBuffer.cs:503-545`
pub(crate) fn unit_table(
    fmtu: &std::collections::BTreeMap<u8, (String, String)>,
    unit_labels: &std::collections::BTreeMap<char, String>,
    multipliers: &std::collections::BTreeMap<char, f64>,
    formats: &std::collections::BTreeMap<u8, crate::dataflash::MessageFormat>,
) -> UnitTable {
    let mut table = UnitTable::default();
    for (key, format) in formats {
        let Some((unit_ids, mult_ids)) = fmtu.get(key) else {
            continue;
        };
        // A format whose type string and label list disagree is skipped whole, as the C# skips
        // it: with the columns out of step there is no saying which unit belongs to which field.
        if format.format.len() != format.labels.len() {
            continue;
        }
        for (index, (label, type_char)) in
            format.labels.iter().zip(format.format.chars()).enumerate()
        {
            let unit = unit_ids
                .chars()
                .nth(index)
                .and_then(|id| unit_labels.get(&id))
                .cloned()
                .unwrap_or_default();
            let multiplier = if matches!(type_char, 'c' | 'C' | 'e' | 'E' | 'L') {
                1.0
            } else {
                mult_ids
                    .chars()
                    .nth(index)
                    .and_then(|id| multipliers.get(&id))
                    .copied()
                    .filter(|value| value.is_finite() && *value != 0.0)
                    .unwrap_or(1.0)
            };
            table.by_field.insert(
                (format.name.clone(), label.clone()),
                FieldUnit { unit, multiplier },
            );
        }
    }
    table
}

/// Text from a field that may be stored as `Z`, sixty-four raw bytes.
///
/// ArduPilot uses `Z` both for text - `UNIT.Label` is `QbZ` - and for the contents of an embedded
/// file, so the decoder hands it back as bytes and cannot know which. Here it is text: NUL
/// terminated, trimmed, as the firmware writes a label. The first version of this read the field
/// with `as_text` and got nothing, and every unit in every log was `""` - which is exactly what
/// the C# ends up with by a different route, so the test that caught it was the one against a
/// hand-built log with known labels, not the one against the real file.
pub(crate) fn text_of(value: &Value) -> Option<String> {
    match value {
        Value::Text(text) => Some(text.trim().to_owned()),
        Value::Bytes(bytes) => {
            let end = bytes.iter().position(|b| *b == 0).unwrap_or(bytes.len());
            Some(String::from_utf8_lossy(bytes.get(..end)?).trim().to_owned())
        }
        _ => None,
    }
}

/// A `UNIT`/`MULT` id as the log stores it - an `int8` holding a character.
pub(crate) fn unit_id(value: f64) -> Option<char> {
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    // the field is a byte, checked to be one before use
    let byte = if value >= 0.0 && value <= f64::from(u8::MAX) {
        value as u8
    } else {
        return None;
    };
    char::from_u32(u32::from(byte))
}

/// Splits a `MSG.FIELD` selector.
///
/// Returns `None` for anything without exactly one dot, rather than guessing - `ATT.Roll.Extra` is
/// a typo, and picking one interpretation of it produces an empty plot with no explanation.
#[must_use]
pub fn parse_selector(selector: &str) -> Option<(&str, &str)> {
    let mut parts = selector.split('.');
    let message = parts.next()?;
    let field = parts.next()?;
    if parts.next().is_some() || message.is_empty() || field.is_empty() {
        return None;
    }
    Some((message, field))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixture() -> Vec<u8> {
        let path =
            std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testdata/dataflash.bin");
        mp_os::fs::read(&path).unwrap_or_else(|err| panic!("reading {}: {err}", path.display()))
    }

    /// The inventory comes from the log's own FMT messages, so it must find real ArduPilot fields.
    #[test]
    fn a_real_log_reports_the_fields_it_contains() {
        let fields = plottable(&fixture());
        assert!(
            fields.len() > 50,
            "a real log should offer plenty to plot, found {}",
            fields.len()
        );

        // Attitude and vibration are in every ArduPilot log and are what somebody plots first.
        // VIBE carries an instance - one record per IMU - so it is named with one.
        let names: Vec<String> = fields.iter().map(ToString::to_string).collect();
        for expected in ["ATT.Roll", "ATT.Pitch", "VIBE[0].VibeX"] {
            assert!(
                names.iter().any(|name| name == expected),
                "{expected} should be plottable; found {} fields",
                names.len()
            );
        }
    }

    /// A log with three IMUs has three VIBE traces, not one with three sensors interleaved.
    ///
    /// The bug this prevents is quiet: a single VIBE.VibeZ series built by message name alone
    /// alternates between IMU 0, 1 and 2 sample by sample, and the result looks like vibration
    /// that is not there. PLAN.md §8.1 names it as a designed-in trap.
    #[test]
    fn a_message_with_instances_is_split_by_instance() {
        let fields = plottable(&fixture());
        let vibe: Vec<&PlottableField> = fields
            .iter()
            .filter(|field| field.message == "VIBE" && field.field == "VibeZ")
            .collect();
        assert!(
            vibe.len() > 1,
            "this log has several IMUs, so VIBE.VibeZ should appear once per instance; got {}",
            vibe.len()
        );
        assert!(
            vibe.iter().all(|field| field.instance.is_some()),
            "every VIBE series should name its IMU"
        );
        // And the instances are distinct.
        let mut seen: Vec<Option<i64>> = vibe.iter().map(|field| field.instance).collect();
        seen.sort();
        let count = seen.len();
        seen.dedup();
        assert_eq!(seen.len(), count, "the same instance was listed twice");
    }

    /// The instance field itself is a selector, not a series.
    #[test]
    fn the_instance_field_is_not_offered_as_a_series() {
        let fields = plottable(&fixture());
        assert!(
            !fields
                .iter()
                .any(|field| field.message == "VIBE" && field.field == "IMU"),
            "plotting which IMU a record came from is a staircase nobody wants"
        );
    }

    /// Splitting has to actually separate the samples, not just relabel them.
    #[test]
    fn extracting_one_instance_leaves_the_others_out() {
        let all: Vec<&PlottableField> = Vec::new();
        let _ = all;
        let first = extract_instance(&fixture(), "VIBE", Some(0), "VibeZ");
        let second = extract_instance(&fixture(), "VIBE", Some(1), "VibeZ");
        let merged = extract(&fixture(), "VIBE", "VibeZ");

        assert!(!first.is_empty(), "IMU 0 should have samples");
        assert!(!second.is_empty(), "IMU 1 should have samples");
        assert!(
            merged.len() > first.len(),
            "the unsplit series should hold every instance: {} vs {}",
            merged.len(),
            first.len()
        );
        // Two different sensors do not report identical values sample for sample.
        assert_ne!(
            first.iter().map(|p| p.value).collect::<Vec<_>>(),
            second.iter().map(|p| p.value).collect::<Vec<_>>(),
            "the two instances returned the same samples, so the split did nothing"
        );
    }

    /// `TimeUS` is the axis, not a series - offering it to plot against itself is noise.
    #[test]
    fn the_timestamp_is_not_offered_as_a_field() {
        let fields = plottable(&fixture());
        assert!(
            !fields.iter().any(|field| field.field == TIME_FIELD),
            "TimeUS is the axis"
        );
    }

    /// A message with no timestamp cannot go on a time axis.
    ///
    /// `FMT` declares formats and `FILE` carries embedded file bytes; neither has a `TimeUS`.
    /// `PARM` was on this list on the assumption that a parameter write is untimed, and the log
    /// says otherwise - its format is `TimeUS,Name,Value`, so plotting a parameter against time is
    /// both possible and something people do. The log is the authority, not the assumption.
    #[test]
    fn untimed_messages_are_not_offered() {
        let fields = plottable(&fixture());
        for untimed in ["FMT", "FILE"] {
            assert!(
                !fields.iter().any(|field| field.message == untimed),
                "{untimed} has no timestamp and cannot be plotted against time"
            );
        }
        assert!(
            fields.iter().any(|field| field.message == "PARM"),
            "PARM carries a TimeUS, so its values are plottable"
        );
    }

    /// The extraction has to produce a series that actually goes somewhere.
    #[test]
    fn a_field_extracts_to_a_series_that_advances_in_time() {
        let points = extract(&fixture(), "ATT", "Roll");
        assert!(points.len() > 100, "found only {} samples", points.len());

        // Time starts near the log's first timestamp - the first ATT is logged within seconds of
        // it - and never goes backwards; lines climb with it.
        assert!(
            (0.0..5.0).contains(&points[0].seconds),
            "the first sample is {}s into the log",
            points[0].seconds
        );
        for pair in points.windows(2) {
            assert!(
                pair[1].seconds >= pair[0].seconds,
                "time went backwards: {} then {}",
                pair[0].seconds,
                pair[1].seconds
            );
            assert!(pair[1].line > pair[0].line, "lines are in log order");
        }
        assert!(
            points.last().is_some_and(|p| p.seconds > 0.0),
            "the series should span some time"
        );
    }

    /// Two fields share one clock, as every curve on the C#'s chart does.
    ///
    /// Each series used to start its own clock at its own first sample, which put a `GPS` sample
    /// and the `ATT` sample logged beside it at different places on the same axis - and a mode
    /// change drawn over both at a third.
    #[test]
    fn every_series_is_timed_from_the_logs_first_timestamp() {
        let data = fixture();
        let origin = time_origin(&data).expect("the fixture has timestamps");
        let attitude = extract(&data, "ATT", "Roll");
        let performance = extract(&data, "PM", "Load");
        // The fixture logs its first PM record nearly eight seconds after its first ATT, so a
        // clock of each series' own would put both at 0.
        assert!(
            performance[0].seconds > attitude[0].seconds + 7.0,
            "PM at {}s, ATT at {}s",
            performance[0].seconds,
            attitude[0].seconds
        );
        // And each is where its own TimeUS says, from the one origin.
        let mut reader = DataflashReader::new(&data);
        let mut first = None;
        while let Some(message) = reader.next_message() {
            if message.name == "PM" {
                first = message.field(TIME_FIELD).and_then(Value::as_f64);
                break;
            }
        }
        let expected = seconds_since(origin, first.expect("a PM record"));
        assert!((performance[0].seconds - expected).abs() < 1e-9);
    }

    /// A sample's line is its record's place in the log, format declarations counted: the row the
    /// grid shows it on, and the x the C# plots it at with Time unticked.
    #[test]
    fn a_sample_carries_the_line_its_record_is_on() {
        let data = fixture();
        let index = crate::index::RecordIndex::build(&data);
        let points = extract(&data, "ATT", "Roll");
        let attitude = index
            .format_named("ATT")
            .map(|format| format.msg_type)
            .expect("ATT is declared");
        for point in points.iter().take(20) {
            assert_eq!(index.msg_type(point.line), Some(attitude), "{point:?}");
        }
        let rows = index.rows_named("ATT");
        assert_eq!(
            points
                .iter()
                .map(|point| u32::try_from(point.line).unwrap_or(u32::MAX))
                .collect::<Vec<_>>(),
            rows,
            "every ATT record is a sample, on its own row"
        );
    }

    /// Roll in a real log is degrees and stays inside what an aircraft can do.
    #[test]
    fn the_extracted_values_are_the_ones_in_the_log() {
        let points = extract(&fixture(), "ATT", "Roll");
        let extreme = points
            .iter()
            .map(|point| point.value.abs())
            .fold(0.0_f64, f64::max);
        assert!(
            extreme < 180.0,
            "roll of {extreme} degrees is not a roll, the field is being misread"
        );
        assert!(points.iter().all(|point| point.value.is_finite()));
    }

    /// Asking for something that is not there is an empty series, not a panic.
    #[test]
    fn an_absent_field_extracts_to_nothing() {
        assert!(extract(&fixture(), "ATT", "NotAField").is_empty());
        assert!(extract(&fixture(), "NOSUCHMSG", "Roll").is_empty());
        assert!(extract(&[], "ATT", "Roll").is_empty());
    }

    /// A selector with the wrong shape is refused rather than guessed at.
    #[test]
    fn a_malformed_selector_is_refused() {
        assert_eq!(parse_selector("ATT.Roll"), Some(("ATT", "Roll")));
        assert_eq!(parse_selector("ATT"), None);
        assert_eq!(parse_selector("ATT.Roll.Extra"), None);
        assert_eq!(parse_selector(".Roll"), None);
        assert_eq!(parse_selector("ATT."), None);
        assert_eq!(parse_selector(""), None);
    }

    /// One dataflash record: the two head bytes, the type, the payload.
    fn record(msg_type: u8, payload: &[u8]) -> Vec<u8> {
        let mut out = vec![
            crate::dataflash::HEAD_BYTE1,
            crate::dataflash::HEAD_BYTE2,
            msg_type,
        ];
        out.extend_from_slice(payload);
        out
    }

    /// A fixed-width text field, zero padded as the firmware writes them.
    fn fixed(text: &str, width: usize) -> Vec<u8> {
        let mut out = text.as_bytes().to_vec();
        out.resize(width, 0);
        out
    }

    /// An `FMT` record declaring a message type: `BBnNZ` = Type, Length, Name, Format, Columns.
    fn fmt(msg_type: u8, length: u8, name: &str, format: &str, columns: &str) -> Vec<u8> {
        let mut payload = vec![msg_type, length];
        payload.extend(fixed(name, 4));
        payload.extend(fixed(format, 16));
        payload.extend(fixed(columns, 64));
        record(crate::dataflash::FMT_TYPE, &payload)
    }

    const FMTU: u8 = 129;
    const UNIT: u8 = 130;
    const MULT: u8 = 131;
    const TEST: u8 = 132;

    /// A log that declares units the way ArduPilot does, small enough to reason about.
    ///
    /// `TEST` has five fields. `A` is a float in degrees with multiplier `0` (the id for "1");
    /// `B` is a hundredths integer (`c`) in metres whose `MULT` says 0.01 - which must be ignored,
    /// because the decoder already divided by 100; `C` is a float in m/s with a real 0.1
    /// multiplier; `D` has the `-` multiplier, which the log stores as 0 and means "none".
    fn log_with_units() -> Vec<u8> {
        let mut log = Vec::new();
        log.extend(fmt(
            crate::dataflash::FMT_TYPE,
            89,
            "FMT",
            "BBnNZ",
            "Type,Length,Name,Format,Columns",
        ));
        log.extend(fmt(
            FMTU,
            44,
            "FMTU",
            "QBNN",
            "TimeUS,FmtType,UnitIds,MultIds",
        ));
        log.extend(fmt(UNIT, 76, "UNIT", "QbZ", "TimeUS,Id,Label"));
        log.extend(fmt(MULT, 20, "MULT", "Qbd", "TimeUS,Id,Mult"));
        log.extend(fmt(TEST, 25, "TEST", "Qfcff", "TimeUS,A,B,C,D"));

        let mut fmtu = 0u64.to_le_bytes().to_vec();
        fmtu.push(TEST);
        fmtu.extend(fixed("sdmnn", 16));
        fmtu.extend(fixed("F0BA-", 16));
        log.extend(record(FMTU, &fmtu));

        for (id, label) in [('s', "s"), ('d', "deg"), ('m', "m"), ('n', "m/s")] {
            let mut unit = 0u64.to_le_bytes().to_vec();
            unit.push(id as u8);
            unit.extend(fixed(label, 64));
            log.extend(record(UNIT, &unit));
        }
        for (id, mult) in [
            ('F', 1e-6_f64),
            ('0', 1.0),
            ('B', 0.01),
            ('A', 0.1),
            ('-', 0.0),
        ] {
            let mut record_bytes = 0u64.to_le_bytes().to_vec();
            record_bytes.push(id as u8);
            record_bytes.extend(mult.to_le_bytes());
            log.extend(record(MULT, &record_bytes));
        }
        log
    }

    /// The unit and multiplier of each field come from FMTU, UNIT and MULT together.
    #[test]
    fn units_are_read_from_the_logs_fmtu_unit_and_mult_messages() {
        let table = units(&log_with_units());
        assert_eq!(
            table.len(),
            5,
            "every TEST field has a row, TimeUS included"
        );
        assert_eq!(
            table.get("TEST", "A"),
            FieldUnit {
                unit: "deg".to_owned(),
                multiplier: 1.0
            }
        );
        assert_eq!(
            table.get("TEST", "C"),
            FieldUnit {
                unit: "m/s".to_owned(),
                multiplier: 0.1
            }
        );
        assert_eq!(table.get("TEST", "TimeUS").unit, "s");
        assert!((table.get("TEST", "TimeUS").multiplier - 1e-6).abs() < 1e-12);
    }

    /// A field the decoder already scales is not scaled again.
    ///
    /// `c` is stored as hundredths and read back divided by 100, so a MULT of 0.01 on top of
    /// that would shrink every value a hundredfold. The C# forces these to 1 for the same
    /// reason. C#: ExtLibs/Utilities/DFLogBuffer.cs:528-534
    #[test]
    fn a_format_the_decoder_already_scales_keeps_a_multiplier_of_one() {
        let table = units(&log_with_units());
        assert_eq!(
            table.get("TEST", "B"),
            FieldUnit {
                unit: "m".to_owned(),
                multiplier: 1.0
            }
        );
    }

    /// The `-` multiplier is stored as 0 and means "none", not "multiply by nothing".
    #[test]
    fn a_zero_multiplier_means_no_multiplier() {
        let table = units(&log_with_units());
        assert_eq!(table.get("TEST", "D").multiplier, 1.0);
        assert_eq!(table.get("TEST", "D").unit, "m/s");
    }

    /// A field the log says nothing about is unitless and unscaled, as `GetUnit` answers.
    #[test]
    fn an_unknown_field_is_unitless_and_unscaled() {
        let table = units(&log_with_units());
        assert_eq!(table.get("TEST", "Nope"), FieldUnit::default());
        assert_eq!(table.get("NOPE", "A"), FieldUnit::default());
        assert!(units(&[]).is_empty());
        assert_eq!(units(&[]).get("ATT", "Roll"), FieldUnit::default());
    }

    /// A format whose type string and column list disagree is skipped whole.
    ///
    /// With the columns out of step there is no saying which unit belongs to which field, and
    /// guessing would label a field with its neighbour's unit.
    #[test]
    fn a_format_whose_columns_disagree_with_its_types_gets_no_units() {
        let mut log = log_with_units();
        // Three type characters, two labels.
        log.extend(fmt(133, 19, "ODD", "Qff", "TimeUS,A"));
        let mut fmtu = 0u64.to_le_bytes().to_vec();
        fmtu.push(133);
        fmtu.extend(fixed("sdd", 16));
        fmtu.extend(fixed("F00", 16));
        log.extend(record(FMTU, &fmtu));

        let table = units(&log);
        assert_eq!(table.get("ODD", "A"), FieldUnit::default());
        assert_eq!(
            table.get("TEST", "A").unit,
            "deg",
            "the good format is unaffected"
        );
    }

    /// A real log declares its units, and attitude is in degrees.
    #[test]
    fn a_real_log_declares_units_for_attitude() {
        let table = units(&fixture());
        assert!(
            !table.is_empty(),
            "a modern ArduPilot log carries UNIT messages"
        );
        let roll = table.get("ATT", "Roll");
        assert_eq!(roll.unit, "deg", "{roll:?}");
        assert_eq!(roll.multiplier, 1.0, "{roll:?}");
        // A coordinate is stored in 1e-7 degrees under format `L`, which the decoder already
        // scales, so whatever MULT says the multiplier is 1.
        let lat = table.get("GPS", "Lat");
        assert_eq!(lat.multiplier, 1.0, "{lat:?}");
        assert!(lat.unit.starts_with("deg"), "{lat:?}");
    }
}
