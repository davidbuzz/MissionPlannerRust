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
fn instance_fields(data: &[u8]) -> std::collections::BTreeMap<String, String> {
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

    // FMTU names the format by number; the label it points at comes from that format's own
    // declaration, which the reader has collected by the time the walk is done.
    let mut out = std::collections::BTreeMap::new();
    for (format_type, index) in by_type {
        if let Ok(key) = u8::try_from(format_type)
            && let Some(format) = reader.formats().get(&key)
            && let Some(label) = format.labels.get(index)
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
    /// Seconds since the first timestamped message in the log.
    pub seconds: f64,
    /// The value.
    pub value: f64,
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
    let mut points = Vec::new();
    let mut origin: Option<f64> = None;
    let mut reader = DataflashReader::new(data);
    while let Some(message) = reader.next_message() {
        if message.name != message_name {
            continue;
        }
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
        let start = *origin.get_or_insert(time);
        // A log can contain a time that goes backwards, across a reboot within one file. Clamped
        // rather than dropped: the samples are real and a negative x would put them off the plot.
        let seconds = ((time - start) / 1_000_000.0).max(0.0);
        if value.is_finite() {
            points.push(Point { seconds, value });
        }
    }
    points
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
        std::fs::read(&path).unwrap_or_else(|err| panic!("reading {}: {err}", path.display()))
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

        // Time starts at zero and never goes backwards.
        assert!(
            points[0].seconds.abs() < f64::EPSILON,
            "the axis starts at zero"
        );
        for pair in points.windows(2) {
            assert!(
                pair[1].seconds >= pair[0].seconds,
                "time went backwards: {} then {}",
                pair[0].seconds,
                pair[1].seconds
            );
        }
        assert!(
            points.last().is_some_and(|p| p.seconds > 0.0),
            "the series should span some time"
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
}
