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

//! The parameters a dataflash log carries, as the log browser's Show Params lists them.
//!
//! A vehicle writes a `PARM` record for every parameter when logging starts, and another each
//! time one changes. `chk_params_CheckedChanged` reads them all into a parameter list - one entry
//! per name, holding the last value logged, at the place the name was first seen - and shows it
//! with the Full Parameter List's grid, `ConfigRawParams`, which sorts it by name with its natural
//! comparer. This is that list and that order, and the text the grid writes a value in; the grid
//! is the screen's.
//! `// C#: Log/LogBrowse.cs:3805-3841; GCSViews/ConfigurationView/ConfigRawParams.cs:563-676, 746-857`

use crate::dataflash::{LogMessage, Value};

/// One parameter as the log last logged it.
#[derive(Debug, Clone, PartialEq)]
pub struct LogParam {
    /// `Name`.
    pub name: String,
    /// `Value`, the last one logged.
    pub value: f64,
    /// `Default`, which newer firmware logs beside the value, when this record has one.
    pub default: Option<f64>,
}

impl LogParam {
    /// The value as the grid writes it: `MAVLinkParam.ToString()` of a `REAL32`, which is the
    /// value as a `float` in .NET's general format - `0.1`, `3.141593`.
    /// `// C#: ExtLibs/Mavlink/MAVLinkParam.cs:218-223`
    #[must_use]
    pub fn value_text(&self) -> String {
        #[allow(clippy::cast_possible_truncation)] // `(float)this`: it is a REAL32
        crate::netfmt::single(self.value as f32)
    }

    /// The default as the grid writes it: `default_value_to_string()`, `NaN` when there is none.
    /// `// C#: ExtLibs/Mavlink/MAVLinkParam.cs:226-233; ConfigRawParams.cs:599-603`
    #[must_use]
    pub fn default_text(&self) -> String {
        #[allow(clippy::cast_possible_truncation)] // `(float)default_value`
        self.default.map_or_else(
            || "NaN".to_owned(),
            |default| crate::netfmt::single(default as f32),
        )
    }
}

/// The parameters in a log's `PARM` records, as `chk_params_CheckedChanged` collects them.
///
/// Each record's `Value` is `double.Parse`d, and its `Default` read if the `PARM` format declares
/// one; a name already in the list has its entry replaced where it stands, since the records are
/// in time order and the later one is the value that changed. (`MAVLinkParamList.Add` replaces by
/// name, so the unconditional `Add` after the loop's replacement changes nothing.) A record with
/// no `Name` or no numeric `Value` is skipped where the C# would throw out of the whole handler;
/// a real log has none.
/// `// C#: Log/LogBrowse.cs:3812-3838; ExtLibs/Mavlink/pymavlink/generator/CS/MAVLinkParamList.cs:32-50, 87-90`
#[must_use]
pub fn from_messages<'a>(messages: impl IntoIterator<Item = &'a LogMessage>) -> Vec<LogParam> {
    let mut list: Vec<LogParam> = Vec::new();
    for message in messages {
        if message.name != "PARM" {
            continue;
        }
        let Some(name) = message.field("Name").and_then(Value::as_text) else {
            continue;
        };
        let Some(value) = message.field("Value").and_then(Value::as_f64) else {
            continue;
        };
        let default = message.field("Default").and_then(Value::as_f64);
        let param = LogParam {
            name: name.to_owned(),
            value,
            default,
        };
        match list.iter_mut().find(|seen| seen.name == param.name) {
            Some(seen) => *seen = param,
            None => list.push(param),
        }
    }
    list
}

/// Reads the parameters out of a log's bytes: every `PARM` record, in log order.
#[must_use]
pub fn read(data: &[u8]) -> Vec<LogParam> {
    let mut reader = crate::dataflash::DataflashReader::new(data);
    let mut parms = Vec::new();
    while let Some(message) = reader.next_message() {
        if message.name == "PARM" {
            parms.push(message);
        }
    }
    from_messages(&parms)
}

/// Whether the grid shows its Default column: any parameter logged a default.
/// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:598-599, 653`
#[must_use]
pub fn has_defaults(params: &[LogParam]) -> bool {
    params.iter().any(|param| param.default.is_some())
}

/// Sorts a list as the grid sorts its Command column: `NaturalStringComparer`, ascending.
///
/// Favourites, which the comparer puts first, are the user's settings and there are none in a
/// log; the order is the names'.
/// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:674-676, 833-857`
pub fn sort(params: &mut [LogParam]) {
    params.sort_by(|a, b| natural_compare(&a.name, &b.name));
}

/// `NaturalStringComparer.NaturalCompare`: runs of digits compared as numbers, leading zeroes
/// skipped and the longer number the bigger; everything else a character at a time, upper-cased.
/// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:757-829`
#[must_use]
pub fn natural_compare(x: &str, y: &str) -> std::cmp::Ordering {
    use std::cmp::Ordering;
    let x: Vec<char> = x.chars().collect();
    let y: Vec<char> = y.chars().collect();
    let (mut ix, mut iy) = (0usize, 0usize);
    loop {
        let (Some(&cx), Some(&cy)) = (x.get(ix), y.get(iy)) else {
            return match (ix == x.len(), iy == y.len()) {
                (true, true) => Ordering::Equal,
                (true, false) => Ordering::Less,
                _ => Ordering::Greater,
            };
        };
        if cx.is_ascii_digit() && cy.is_ascii_digit() {
            while x.get(ix) == Some(&'0') {
                ix += 1;
            }
            while y.get(iy) == Some(&'0') {
                iy += 1;
            }
            let end_x = ix + x.iter().skip(ix).take_while(|c| c.is_ascii_digit()).count();
            let end_y = iy + y.iter().skip(iy).take_while(|c| c.is_ascii_digit()).count();
            let (length_x, length_y) = (end_x - ix, end_y - iy);
            if length_x != length_y {
                return length_x.cmp(&length_y);
            }
            while ix < end_x {
                let (a, b) = (x.get(ix), y.get(iy));
                if a != b {
                    return a.cmp(&b);
                }
                ix += 1;
                iy += 1;
            }
        } else {
            let order = cx.to_ascii_uppercase().cmp(&cy.to_ascii_uppercase());
            if order != Ordering::Equal {
                return order;
            }
            ix += 1;
            iy += 1;
        }
    }
}

/// What the Options column holds for a parameter: `(range + "\n" + options.Replace(",", "\n"))
/// .Trim()`, where the range is the documentation's `low high` and the options its `code:name,`
/// pairs, each ending in a comma.
/// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:612-621;
/// ExtLibs/Utilities/ParameterMetaDataRepositoryAPMpdef.cs:236-246`
#[must_use]
pub fn options_text(range: Option<(f64, f64)>, values: &[(i64, &str)]) -> String {
    let range = range.map_or_else(String::new, |(low, high)| {
        format!(
            "{} {}",
            crate::netfmt::double(low),
            crate::netfmt::double(high)
        )
    });
    let options: String = values
        .iter()
        .map(|(code, name)| format!("{code}:{name}\n"))
        .collect();
    format!("{range}\n{options}").trim().to_owned()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn parm(name: &str, value: f64, default: Option<f64>) -> LogMessage {
        let mut fields = vec![
            ("TimeUS".to_owned(), Value::Uint(1)),
            ("Name".to_owned(), Value::Text(name.to_owned())),
            ("Value".to_owned(), Value::Float(value)),
        ];
        if let Some(default) = default {
            fields.push(("Default".to_owned(), Value::Float(default)));
        }
        LogMessage {
            name: "PARM".to_owned(),
            fields,
        }
    }

    /// A name logged twice keeps its first place and its last value.
    #[test]
    fn a_changed_parameter_keeps_its_place_and_takes_the_new_value() {
        let messages = [
            parm("B_ONE", 1.0, None),
            parm("A_TWO", 2.0, None),
            parm("B_ONE", 3.0, None),
        ];
        let list = from_messages(&messages);
        let names: Vec<(&str, f64)> = list
            .iter()
            .map(|param| (param.name.as_str(), param.value))
            .collect();
        assert_eq!(names, vec![("B_ONE", 3.0), ("A_TWO", 2.0)]);
        assert!(!has_defaults(&list));
    }

    /// The values are written as floats in .NET's general format, and a missing default as NaN.
    #[test]
    fn values_are_written_as_the_grid_writes_them() {
        // Seven significant digits: a float's general format.
        let list = from_messages(&[parm("X", 0.1, Some(1.234_567_89))]);
        assert_eq!(list[0].value_text(), "0.1");
        assert_eq!(list[0].default_text(), "1.234568");
        assert!(has_defaults(&list));
        let plain = from_messages(&[parm("Y", 1500.0, None)]);
        assert_eq!(plain[0].value_text(), "1500");
        assert_eq!(plain[0].default_text(), "NaN");
    }

    /// The natural order: numbers as numbers, letters without case.
    #[test]
    fn the_names_sort_naturally() {
        let mut list = from_messages(&[
            parm("SERVO10_MIN", 0.0, None),
            parm("SERVO2_MIN", 0.0, None),
            parm("servo1_min", 0.0, None),
            parm("ATC_RAT", 0.0, None),
            parm("SERVO02_MAX", 0.0, None),
        ]);
        sort(&mut list);
        let names: Vec<&str> = list.iter().map(|param| param.name.as_str()).collect();
        assert_eq!(
            names,
            vec![
                "ATC_RAT",
                "servo1_min",
                "SERVO02_MAX",
                "SERVO2_MIN",
                "SERVO10_MIN"
            ]
        );
    }

    /// The Options column: the range, then each option on a line of its own.
    #[test]
    fn the_options_column_is_the_range_then_the_values() {
        assert_eq!(
            options_text(Some((0.0, 100.0)), &[(0, "Disabled"), (1, "Enabled")]),
            "0 100\n0:Disabled\n1:Enabled"
        );
        assert_eq!(options_text(None, &[(1, "On")]), "1:On");
        assert_eq!(options_text(Some((0.5, 2.0)), &[]), "0.5 2");
        assert_eq!(options_text(None, &[]), "");
    }

    /// The healthy fixture logs its parameters once each: the list is as long as the `.param`
    /// file Mission Planner wrote from it.
    #[test]
    fn the_fixtures_parameters_are_the_ones_mission_planner_saved() {
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testdata");
        let data = mp_os::fs::read(root.join("dataflash.bin")).expect("fixture");
        let list = read(&data);
        let saved = mp_os::fs::read_to_string(root.join("dataflash/golden/kml/dataflash.bin.param"))
            .expect("golden .param");
        let lines: Vec<&str> = saved
            .lines()
            .filter(|line| !line.trim().is_empty())
            .collect();
        assert_eq!(list.len(), lines.len(), "one entry per saved parameter");
        for line in lines {
            let (name, _) = line.split_once(',').expect("NAME,value");
            assert!(
                list.iter().any(|param| param.name == name),
                "{name} is in the saved file but not in the list"
            );
        }
    }
}
