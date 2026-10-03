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

//! Auto Analysis: ArduPilot's `LogAnalyzer`, which Mission Planner runs as `runner.exe`, ported
//! and run in-process.
//!
//! `BUT_loganalysis_Click` (`GCSViews/FlightData.cs:1311-1385`) converts a `.bin` to a temporary
//! `.log` with `BinaryLog.ConvertBin`, then `LogAnalyzer.CheckLogFile` (`Utilities/LogAnalyzer.cs:
//! 18-115`) downloads `LogAnalyzer64.zip` from firmware.ardupilot.org into
//! `<data dir>/LogAnalyzer/`, extracts it and runs `runner.exe -x "<log>.xml" -s "<log>"` there -
//! ArduPilot's Python `LogAnalyzer` packaged by py2exe, a Windows program - and
//! `LogAnalyzer.Results` reads the XML it wrote, which `Controls.LogAnalyzer` shows as text.
//!
//! The runner's source is in the C# tree (`LogAnalyzer/py2exe`: `DataflashLog.py`,
//! `LogAnalyzer.py`, `VehicleType.py` and `tests/*.py`) and is ported under this module:
//! [`logdata`] reads the `.log` as the Python does, [`checks`] are its seventeen tests, [`suite`]
//! runs them and writes the XML `outputXML` writes, `<log>.xml` beside the log as `-x` names it.
//! So the analysis runs on every platform with nothing fetched: the download, the zip and the
//! process are what this port leaves out of `CheckLogFile`, and the "Failed to download
//! LogAnalyzer" and "Failed to start LogAnalyzer" boxes with them. The XML is still written and
//! read back with the C#'s own reading ([`results`]), quirks included, and shown as its text
//! ([`report`]).
//!
//! Held to the Python itself: `tests/analysis.rs` runs the port over the checked-in logs against
//! the XML Python 2.7 wrote for the same text (`testdata/dataflash/golden/loganalysis`, made by
//! `tools/loganalyzer-golden.sh`), and `report` to the C#'s reading of the analyzer's example
//! output, byte for byte.
//! `// C#: GCSViews/FlightData.cs:1311-1385; Utilities/LogAnalyzer.cs; Controls/LogAnalyzer.cs;
//! LogAnalyzer/py2exe`

mod checks;
mod logdata;
mod pyval;
mod suite;

use std::path::{Path, PathBuf};

use crate::convert::{ModeName, convert_bin_file};

/// `Environment.NewLine`, which the report's header lines end with.
const NEWLINE: &str = if cfg!(windows) { "\r\n" } else { "\n" };

/// Why the analysis produced nothing to show. Each is one of the C#'s message boxes.
#[derive(Debug, thiserror::Error)]
pub enum AnalysisError {
    /// "File access issue: ..." - the `.bin` could not be converted.
    #[error("File access issue: {0}")]
    FileAccess(std::io::Error),
    /// "Bad input file": the analyzer wrote no XML - the log would not open or read (the Python
    /// raises out of `DataflashLog.read`), or the XML could not be written.
    #[error("Bad input file")]
    BadInputFile,
    /// "Failed to load analyzer results" - the XML did not read.
    #[error("Failed to load analyzer results\n{0}")]
    Results(String),
}

/// `LogAnalyzer.analysis`: the header the analyzer writes, and its tests.
/// `// C#: Utilities/LogAnalyzer.cs:218-232`
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Analysis {
    /// `logfile`.
    pub logfile: Option<String>,
    /// `sizekb`.
    pub sizekb: Option<String>,
    /// `sizelines`.
    pub sizelines: Option<String>,
    /// `duration`.
    pub duration: Option<String>,
    /// `vehicletype`.
    pub vehicletype: Option<String>,
    /// `firmwareversion`.
    pub firmwareversion: Option<String>,
    /// `firmwarehash`.
    pub firmwarehash: Option<String>,
    /// `hardwaretype`.
    pub hardwaretype: Option<String>,
    /// `freemem`.
    pub freemem: Option<String>,
    /// `skippedlines`.
    pub skippedlines: Option<String>,
    /// `results`.
    pub results: Vec<TestResult>,
}

/// `LogAnalyzer.result`: one test. `// C#: Utilities/LogAnalyzer.cs:234-240`
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct TestResult {
    /// `name`.
    pub name: Option<String>,
    /// `status`.
    pub status: Option<String>,
    /// `message`.
    pub message: Option<String>,
    /// `data`.
    pub data: Option<String>,
}

/// Where the analyzer writes its XML: `<log>.xml`, the `-x` argument `CheckLogFile` passes.
/// `// C#: Utilities/LogAnalyzer.cs:80-84`
#[must_use]
pub fn xml_path_for(log: &Path) -> PathBuf {
    let mut xml = log.as_os_str().to_owned();
    xml.push(".xml");
    PathBuf::from(xml)
}

/// `runner.exe -x <xml> -s <log>`: reads `log` as `DataflashLog` reads it - `-s`, lines that will
/// not read skipped - runs the checks and writes the analyzer's XML to `xml`, whose text is
/// returned. A log that never names a known vehicle is written as far as the Python gets before
/// it raises, the header cut at its duration.
///
/// # Errors
///
/// [`AnalysisError::BadInputFile`] when the log does not open or read, or the XML cannot be
/// written: the runner then leaves no XML.
pub fn run_analyzer(log: &Path, xml: &Path) -> Result<String, AnalysisError> {
    let bytes = std::fs::read(log).map_err(|_| AnalysisError::BadInputFile)?;
    // The Python reads bytes; a log that is not UTF-8 is read with its bad bytes replaced, which
    // counts those bytes differently in the size.
    let text = String::from_utf8_lossy(&bytes);
    let written = analyse_text(&text, &log.display().to_string())
        .map_err(|_| AnalysisError::BadInputFile)?;
    std::fs::write(xml, &written).map_err(|_| AnalysisError::BadInputFile)?;
    Ok(written)
}

/// The analyzer over a log's text, named `filename` in its XML: what `runner.exe` writes, or
/// the Python's exception out of `DataflashLog.read`. The whole of the analysis without a file,
/// which is what the fuzz target feeds.
///
/// # Errors
///
/// The text would not read as a log: `str(e)` of the exception.
pub fn analyse_text(text: &str, filename: &str) -> Result<String, String> {
    let mut data = logdata::DataflashLog::read(text, filename, true).map_err(|e| e.text)?;
    let ran = suite::run(&mut data);
    Ok(match suite::output_xml(&data, &ran) {
        Ok(xml) | Err(xml) => xml,
    })
}

/// Auto Analysis (`BUT_loganalysis`) for one file: a `.bin` is converted to a temporary `.log`
/// first, the analyzer is run on it ([`run_analyzer`]) with its XML beside it, and what it wrote
/// is read ([`results`]); the temporary `.log` is deleted afterwards, its `.xml` is not.
///
/// # Errors
///
/// Each of the button's message boxes: see [`AnalysisError`].
/// `// C#: GCSViews/FlightData.cs:1311-1385`
pub fn analyse(log: &Path, mode_name: ModeName<'_>) -> Result<Analysis, AnalysisError> {
    analyse_to(log, None, mode_name)
}

/// [`analyse`], with the XML written to `xml` when one is named instead of beside the log.
///
/// # Errors
///
/// As [`analyse`].
pub fn analyse_to(
    log: &Path,
    xml: Option<&Path>,
    mode_name: ModeName<'_>,
) -> Result<Analysis, AnalysisError> {
    let is_bin = log.to_string_lossy().to_lowercase().ends_with(".bin");
    let converted = if is_bin {
        // `Path.GetTempFileName() + ".log"`.
        let temp = std::env::temp_dir().join(format!(
            "tmp{}{}.tmp.log",
            std::process::id(),
            crate::now_micros()
        ));
        convert_bin_file(log, &temp, mode_name).map_err(AnalysisError::FileAccess)?;
        Some(temp)
    } else {
        None
    };
    let file = converted.as_deref().unwrap_or(log);
    let xml = xml.map_or_else(|| xml_path_for(file), Path::to_path_buf);
    let outcome =
        run_analyzer(file, &xml).and_then(|text| results(&text).map_err(AnalysisError::Results));
    if let Some(temp) = converted {
        let _ = std::fs::remove_file(temp);
    }
    outcome
}

/// A node of the document as an `XmlReader` steps through it.
#[derive(Debug, Clone, PartialEq, Eq)]
enum Node {
    /// An element's start, and the index of its end.
    Start { name: String, end: usize },
    /// Text or CDATA with something other than white space in it.
    Text(String),
    /// Text of white space only: textual to `ReadString`, skipped by `MoveToContent`.
    Whitespace(String),
    /// A comment or processing instruction.
    Other,
    /// An element's end.
    End,
}

fn flatten(node: roxmltree::Node<'_, '_>, out: &mut Vec<Node>) {
    for child in node.children() {
        if child.is_element() {
            let at = out.len();
            out.push(Node::Start {
                name: child.tag_name().name().to_owned(),
                end: 0,
            });
            flatten(child, out);
            let end = out.len();
            out.push(Node::End);
            if let Some(Node::Start { end: slot, .. }) = out.get_mut(at) {
                *slot = end;
            }
        } else if let Some(text) = child.text().filter(|_| child.is_text()) {
            if text.chars().all(|c| matches!(c, ' ' | '\t' | '\r' | '\n')) {
                out.push(Node::Whitespace(text.to_owned()));
            } else {
                out.push(Node::Text(text.to_owned()));
            }
        } else {
            out.push(Node::Other);
        }
    }
}

/// `ReadToFollowing(name)`: the next start of an element called `name` from `from` on.
fn following(nodes: &[Node], from: usize, name: &str) -> Option<(usize, usize)> {
    nodes
        .iter()
        .enumerate()
        .skip(from)
        .find_map(|(at, node)| match node {
            Node::Start { name: tag, end } if tag == name => Some((at, *end)),
            _ => None,
        })
}

/// `ReadString` on the element starting at `at`: the text and white space up to its first other
/// node, and the index of that node, where the reader is left.
fn read_string(nodes: &[Node], at: usize) -> (String, usize) {
    let mut text = String::new();
    let mut next = at + 1;
    while let Some(Node::Text(part) | Node::Whitespace(part)) = nodes.get(next) {
        text.push_str(part);
        next += 1;
    }
    (text, next)
}

/// Steps through the subtree `start..=end` as `while (subtree.Read()) { MoveToElement();
/// if (IsStartElement()) ... }` does, handing each element start it lands on to `visit`, which
/// returns where it left the reader.
fn walk(
    nodes: &[Node],
    start: usize,
    end: usize,
    visit: &mut dyn FnMut(&str, usize) -> Result<usize, String>,
) -> Result<(), String> {
    let mut at = start;
    while at <= end {
        // IsStartElement's MoveToContent: past white space, comments and instructions.
        while at <= end && matches!(nodes.get(at), Some(Node::Whitespace(_) | Node::Other)) {
            at += 1;
        }
        if at > end {
            break;
        }
        if let Some(Node::Start { name, .. }) = nodes.get(at) {
            at = visit(&name.to_lowercase(), at)?;
        }
        at += 1;
    }
    Ok(())
}

/// `LogAnalyzer.Results`: the header and the tests out of the analyzer's XML, as the C# reads
/// them - the first `header` element, then the first `results` element after it, and again while
/// there are more. A test is kept when the next `result` starts, so **the last test is never
/// shown**, and a `name`, `status`, `message` or `data` before any `result` is an error.
///
/// # Errors
///
/// The XML does not parse, or a test's field comes before its `result`.
/// `// C#: Utilities/LogAnalyzer.cs:117-216`
pub fn results(xml: &str) -> Result<Analysis, String> {
    let document = roxmltree::Document::parse(xml).map_err(|e| e.to_string())?;
    let mut nodes = Vec::new();
    flatten(document.root(), &mut nodes);
    let mut answer = Analysis::default();
    let mut at = 0usize;
    while at < nodes.len() {
        match following(&nodes, at, "header") {
            Some((start, end)) => {
                walk(&nodes, start, end, &mut |name, at| {
                    let slot = match name {
                        "logfile" => &mut answer.logfile,
                        "sizekb" => &mut answer.sizekb,
                        "sizelines" => &mut answer.sizelines,
                        "duration" => &mut answer.duration,
                        "vehicletype" => &mut answer.vehicletype,
                        "firmwareversion" => &mut answer.firmwareversion,
                        "firmwarehash" => &mut answer.firmwarehash,
                        "hardwaretype" => &mut answer.hardwaretype,
                        "freemem" => &mut answer.freemem,
                        "skippedlines" => &mut answer.skippedlines,
                        _ => return Ok(at),
                    };
                    let (text, next) = read_string(&nodes, at);
                    *slot = Some(text);
                    Ok(next)
                })?;
                at = end + 1;
            }
            None => at = nodes.len(),
        }
        // "params - later"
        match following(&nodes, at, "results") {
            Some((start, end)) => {
                let mut current: Option<TestResult> = None;
                walk(&nodes, start, end, &mut |name, at| {
                    let field = match name {
                        "result" => {
                            if let Some(done) = current.take()
                                && done.name.as_deref() != Some("")
                            {
                                answer.results.push(done);
                            }
                            current = Some(TestResult::default());
                            return Ok(at);
                        }
                        "name" | "status" | "message" | "data" => name,
                        _ => return Ok(at),
                    };
                    let Some(result) = current.as_mut() else {
                        return Err(format!(
                            "System.NullReferenceException: <{field}> before any <result>"
                        ));
                    };
                    let (text, next) = read_string(&nodes, at);
                    let slot = match field {
                        "name" => &mut result.name,
                        "status" => &mut result.status,
                        "message" => &mut result.message,
                        _ => &mut result.data,
                    };
                    *slot = Some(text);
                    Ok(next)
                })?;
                at = end + 1;
            }
            None => at = nodes.len(),
        }
    }
    Ok(answer)
}

/// The text `Controls.LogAnalyzer` shows: the header, a line each, then `Test: name = status -
/// message` per test. The header's lines end with the platform's newline and the tests' with
/// `\r\n`, as the C# writes them.
/// `// C#: Controls/LogAnalyzer.cs:8-32`
#[must_use]
pub fn report(analysis: &Analysis) -> String {
    let field = |value: &Option<String>| value.clone().unwrap_or_default();
    let mut text = format!(
        "Log File {}\nSize (kb) {}\nNo of lines {}\nDuration {}\nVehicletype {}\n\
         Firmware Version {}\nFirmware Hash {}\nHardware Type {}\nFree Mem {}\nSkipped Lines {}\n",
        field(&analysis.logfile),
        field(&analysis.sizekb),
        field(&analysis.sizelines),
        field(&analysis.duration),
        field(&analysis.vehicletype),
        field(&analysis.firmwareversion),
        field(&analysis.firmwarehash),
        field(&analysis.hardwaretype),
        field(&analysis.freemem),
        field(&analysis.skippedlines),
    )
    .replace('\n', NEWLINE);
    for result in &analysis.results {
        text.push_str(&format!(
            "Test: {} = {} - {}\r\n",
            field(&result.name),
            field(&result.status),
            field(&result.message)
        ));
    }
    text
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_last_test_is_dropped_and_a_nameless_one_kept() {
        let xml = "<a><header><LogFile>x.log</LogFile><freemem/></header>\
                   <results><result><status>GOOD</status></result>\
                   <result><name>B</name></result><result><name>C</name></result></results></a>";
        let analysis = results(xml).unwrap();
        assert_eq!(analysis.logfile.as_deref(), Some("x.log"));
        assert_eq!(analysis.freemem.as_deref(), Some(""));
        let names: Vec<_> = analysis.results.iter().map(|r| r.name.clone()).collect();
        assert_eq!(names, [None, Some("B".to_owned())]);
    }

    #[test]
    fn results_are_only_read_after_a_header() {
        let xml = "<a><results><result><name>A</name></result><result/></results></a>";
        assert!(results(xml).unwrap().results.is_empty());
        let xml = "<a><header/><results><name>A</name></results></a>";
        assert!(results(xml).is_err());
    }

    #[test]
    fn the_xml_goes_beside_the_log_and_a_missing_log_is_a_bad_input_file() {
        assert_eq!(
            xml_path_for(Path::new("/logs/a b.log")),
            PathBuf::from("/logs/a b.log.xml")
        );
        let missing =
            std::env::temp_dir().join(format!("mp-log-missing-{}.log", std::process::id()));
        let outcome = run_analyzer(&missing, &xml_path_for(&missing));
        assert!(matches!(outcome, Err(AnalysisError::BadInputFile)));
        assert!(!xml_path_for(&missing).exists());
    }

    /// A log that never names a vehicle: the XML is cut at its duration, as the Python's crash
    /// leaves it, and the C#'s reading of it fails - "Failed to load analyzer results".
    #[test]
    fn an_unknown_vehicle_is_a_truncated_xml_that_fails_to_load() {
        let dir = std::env::temp_dir().join(format!("mp-log-novehicle-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let log = dir.join("sub.log");
        std::fs::write(
            &log,
            "FMT, 128, 89, FMT, BBnNZ, Type,Length,Name,Format,Columns\n\
             FMT, 10, 10, MSG, QZ, TimeUS,Message\nMSG, 1, ArduSub V4.5.7 (1a2b3c4d)\n",
        )
        .unwrap();
        let outcome = analyse(&log, &crate::convert::no_mode_names);
        assert!(
            matches!(outcome, Err(AnalysisError::Results(_))),
            "{outcome:?}"
        );
        let written = std::fs::read_to_string(xml_path_for(&log)).unwrap();
        assert!(
            written.ends_with("<duration>0:00:00</duration>\n"),
            "{written}"
        );
        std::fs::remove_dir_all(&dir).unwrap();
    }
}
