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

//! `TestSuite`: the checks run over a log, and `outputXML`, the file `runner.exe -x` writes and
//! Mission Planner reads. `// LogAnalyzer.py:56-215`

use super::checks::{CHECKS, Outcome, Status};
use super::logdata::DataflashLog;
use super::pyval::{py_repr, repr_float, timedelta};

/// One check's outcome, with the check's name.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct Ran {
    /// `Test.name`.
    pub(crate) name: &'static str,
    /// `Test.result`.
    pub(crate) outcome: Outcome,
}

/// `TestSuite.run(logdata, verbose=False)`: `GPS2` standing in for a missing `GPS`
/// ("*cough*"), then every enabled check in turn; one that raises is an UNKNOWN whose message
/// is the exception's text. `// LogAnalyzer.py:76-92`
pub(crate) fn run(log: &mut DataflashLog) -> Vec<Ran> {
    if !log.channels.contains_key("GPS")
        && let Some(gps2) = log.channels.get("GPS2").cloned()
    {
        log.channels.insert("GPS", gps2);
    }
    CHECKS
        .iter()
        .map(|(name, check)| Ran {
            name,
            outcome: match check(log) {
                Ok(outcome) => outcome,
                Err(raised) => Outcome::new(Status::Unknown, raised.text),
            },
        })
        .collect()
}

/// `xml.sax.saxutils.escape`.
fn escape(text: &str) -> String {
    text.replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
}

/// `TestSuite.outputXML(xmlFile)`: the header, the parameters (each value's `repr`) and a
/// `result` a check - FAIL and WARN with the `data` placeholder, NA without a message.
///
/// # Errors
///
/// A log that never named a known vehicle: `escape(None)` raises after the `duration` line,
/// and the file holds what was written up to there, which is returned. The parameters come in
/// the order the log declared them, not the Python dict's. `// LogAnalyzer.py:138-215`
pub(crate) fn output_xml(log: &DataflashLog, ran: &[Ran]) -> Result<String, String> {
    let mut xml = String::new();
    xml.push_str("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<loganalysis>\n<header>\n");
    xml.push_str(&format!("  <logfile>{}</logfile>\n", escape(&log.filename)));
    xml.push_str(&format!(
        "  <sizekb>{}</sizekb>\n",
        escape(&repr_float(log.filesize_kb))
    ));
    xml.push_str(&format!("  <sizelines>{}</sizelines>\n", log.line_count));
    xml.push_str(&format!(
        "  <duration>{}</duration>\n",
        escape(&timedelta(log.duration_secs))
    ));
    let Some(vehicle) = log.vehicle_type else {
        return Err(xml);
    };
    xml.push_str(&format!(
        "  <vehicletype>{}</vehicletype>\n",
        escape(vehicle.name())
    ));
    if let Some(copter_type) = log.copter_type() {
        xml.push_str(&format!(
            "  <coptertype>{}</coptertype>\n",
            escape(copter_type)
        ));
    }
    xml.push_str(&format!(
        "  <firmwareversion>{}</firmwareversion>\n",
        escape(&log.firmware_version)
    ));
    xml.push_str(&format!(
        "  <firmwarehash>{}</firmwarehash>\n",
        escape(&log.firmware_hash)
    ));
    xml.push_str(&format!(
        "  <hardwaretype>{}</hardwaretype>\n",
        escape(&log.hardware_type)
    ));
    xml.push_str(&format!("  <freemem>{}</freemem>\n", log.free_ram));
    xml.push_str(&format!(
        "  <skippedlines>{}</skippedlines>\n",
        log.skipped_lines
    ));
    xml.push_str("</header>\n<params>\n");
    for (name, value) in log.parameters.iter() {
        xml.push_str(&format!(
            "  <param name=\"{name}\" value=\"{}\" />\n",
            escape(&py_repr(value))
        ));
    }
    xml.push_str("</params>\n<results>\n");
    for Ran { name, outcome } in ran {
        xml.push_str("  <result>\n");
        xml.push_str(&format!("    <name>{}</name>\n", escape(name)));
        xml.push_str(&format!("    <status>{}</status>\n", outcome.status.text()));
        if outcome.status != Status::Na {
            xml.push_str(&format!(
                "    <message>{}</message>\n",
                escape(&outcome.message)
            ));
        }
        if matches!(outcome.status, Status::Fail | Status::Warn) {
            xml.push_str("    <data>(test data will be embedded here at some point)</data>\n");
        }
        xml.push_str("  </result>\n");
    }
    xml.push_str("</results>\n</loganalysis>\n");
    Ok(xml)
}

#[cfg(test)]
mod tests {
    use super::*;

    const HEAD: &str = "FMT, 128, 89, FMT, BBnNZ, Type,Length,Name,Format,Columns\n\
                        FMT, 32, 35, PARM, QNff, TimeUS,Name,Value,Default\n\
                        FMT, 10, 10, MSG, QZ, TimeUS,Message\n\
                        FMT, 5, 5, GPS2, QBf, TimeUS,NSats,HDop\n\
                        FMT, 6, 6, MOT, Qffff, TimeUS,Mot1,Mot2,Mot3\n";

    #[test]
    fn gps2_stands_in_for_gps_and_a_raise_is_unknown() {
        let text = format!("{HEAD}MSG, 1, ArduCopter V4.5.7 (1a2b3c4d)\nGPS2, 2, 9, 1.2\n");
        let mut log = DataflashLog::read(&text, "t.log", true).unwrap();
        let ran = run(&mut log);
        assert_eq!(ran.len(), 17);
        let names: Vec<&str> = ran.iter().map(|r| r.name).collect();
        assert_eq!(names.first(), Some(&"Autotune"));
        assert_eq!(names.last(), Some(&"Vibration"));
        let gps = ran.iter().find(|r| r.name == "GPS").unwrap();
        assert_eq!(gps.outcome, Outcome::new(Status::Good, ""));
        let dupe = ran.iter().find(|r| r.name == "Dupe Log Data").unwrap();
        assert_eq!(
            dupe.outcome,
            Outcome::new(Status::Unknown, "No ATT log data")
        );
        let xml = output_xml(&log, &ran).unwrap();
        assert!(xml.starts_with("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<loganalysis>\n<header>\n  <logfile>t.log</logfile>\n"));
        assert!(xml.contains("  <vehicletype>ArduCopter</vehicletype>\n  <coptertype>quad</coptertype>\n  <firmwareversion>V4.5.7</firmwareversion>\n  <firmwarehash>1a2b3c4d</firmwarehash>\n  <hardwaretype></hardwaretype>\n  <freemem>0</freemem>\n  <skippedlines>0</skippedlines>\n</header>\n"));
        assert!(xml.contains("  <result>\n    <name>Compass</name>\n    <status>FAIL</status>\n    <message>'COMPASS_OFS_X' not found</message>\n    <data>(test data will be embedded here at some point)</data>\n  </result>\n"));
        assert!(xml.contains("    <name>Dupe Log Data</name>\n    <status>UNKNOWN</status>\n    <message>No ATT log data</message>\n  </result>\n"));
        assert!(xml.ends_with("</results>\n</loganalysis>\n"));
    }

    #[test]
    fn na_has_no_message_and_params_are_reprs() {
        let text = format!(
            "{HEAD}MSG, 1, ArduPlane V4.5.7 (1a2b3c4d)\nPARM, 2, A<B, 1.5, 0\nPARM, 2, TUNE_HIGH, 10000, 0\n"
        );
        let mut log = DataflashLog::read(&text, "a&b.log", true).unwrap();
        let ran = run(&mut log);
        let xml = output_xml(&log, &ran).unwrap();
        assert!(xml.contains("<logfile>a&amp;b.log</logfile>"));
        assert!(xml.contains("  <param name=\"A<B\" value=\"1.5\" />\n  <param name=\"TUNE_HIGH\" value=\"10000.0\" />\n"));
        assert!(xml.contains("    <name>Autotune</name>\n    <status>NA</status>\n  </result>\n"));
        assert!(!xml.contains("<coptertype>"));
    }

    #[test]
    fn no_vehicle_is_a_file_cut_at_the_duration() {
        let text = format!("{HEAD}MSG, 1, ArduSub V4.5.7 (1a2b3c4d)\n");
        let mut log = DataflashLog::read(&text, "t.log", true).unwrap();
        let ran = run(&mut log);
        let partial = output_xml(&log, &ran).unwrap_err();
        assert!(
            partial.ends_with("  <duration>0:00:00</duration>\n"),
            "{partial}"
        );
        assert!(!partial.contains("vehicletype"));
    }
}
