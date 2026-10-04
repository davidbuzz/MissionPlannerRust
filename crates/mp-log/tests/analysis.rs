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

//! "Auto Analysis": ArduPilot's LogAnalyzer, ported into `mp_log::analysis`, held to the Python
//! itself - Python 2.7 running the runner's own source over the same text, its XML kept under
//! `testdata/dataflash/golden/loganalysis` (made by `tools/loganalyzer-golden.sh`) - and the
//! report to the C#'s own reading of the analyzer's example output (`LogAnalyzer.Results` and
//! `Controls.LogAnalyzer`'s text, run under mono by `tools/csharp-reference/regen-log.sh`), byte
//! for byte.
//!
//! Two things in the Python's output follow its hash tables, which differ between the Windows
//! runner and the Linux build that made the goldens: the order of the `<param>` elements, and the
//! order of the lines of the NaNs check and the words of the Event/Failsafe check. The comparison
//! sorts those; everything else is exact.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use std::path::{Path, PathBuf};

use mp_log::analysis::{
    Analysis, analyse, analyse_to, report, results, run_analyzer, xml_path_for,
};
use mp_log::convert::flight_mode_name;

fn testdata(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../testdata")
        .join(name)
}

fn scratch(test: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("mp-log-analysis-{test}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

fn example() -> Analysis {
    let xml = std::fs::read_to_string(testdata("dataflash/example_output.xml")).unwrap();
    results(&xml).unwrap()
}

#[test]
fn the_report_is_the_csharps_text() {
    let golden =
        std::fs::read_to_string(testdata("dataflash/golden/loganalysis/example_output.txt"))
            .unwrap();
    // The ten header lines end with Environment.NewLine, which was "\n" where the golden was
    // written and is "\r\n" on Windows. The tests' lines end "\r\n" everywhere, and the "\n" inside
    // the Compass test's message comes from the XML, the same everywhere: only the header changes.
    let expected = if cfg!(windows) {
        let header_end = golden.match_indices('\n').nth(9).map(|(at, _)| at + 1).unwrap();
        golden[..header_end].replace('\n', "\r\n") + &golden[header_end..]
    } else {
        golden
    };
    assert_eq!(report(&example()), expected);
}

/// Vibration is the last test in the file, and `Results` never adds the last one.
#[test]
fn the_last_test_is_never_shown() {
    let analysis = example();
    assert_eq!(analysis.results.len(), 11);
    assert_eq!(
        analysis.results.last().unwrap().name.as_deref(),
        Some("VCC")
    );
    assert_eq!(analysis.vehicletype.as_deref(), Some("ArduCopter"));
}

/// The analyzer's XML, taken apart for comparing: the header's elements but the log's name, the
/// parameters sorted, and a `(name, status, message, data)` a result, the two hash-ordered
/// messages sorted.
#[derive(Debug, PartialEq, Eq)]
struct Shape {
    header: Vec<(String, String)>,
    params: Vec<(String, String)>,
    results: Vec<(String, String, Option<String>, Option<String>)>,
}

fn shape(xml: &str) -> Shape {
    let document = roxmltree::Document::parse(xml).unwrap();
    let root = document.root_element();
    let text = |node: roxmltree::Node| node.text().unwrap_or("").to_owned();
    let header = root
        .children()
        .find(|n| n.has_tag_name("header"))
        .unwrap()
        .children()
        .filter(|n| n.is_element() && !n.has_tag_name("logfile"))
        .map(|n| (n.tag_name().name().to_owned(), text(n)))
        .collect();
    let mut params: Vec<(String, String)> = root
        .children()
        .find(|n| n.has_tag_name("params"))
        .unwrap()
        .children()
        .filter(|n| n.has_tag_name("param"))
        .map(|n| {
            (
                n.attribute("name").unwrap().to_owned(),
                n.attribute("value").unwrap().to_owned(),
            )
        })
        .collect();
    params.sort();
    let results = root
        .children()
        .find(|n| n.has_tag_name("results"))
        .unwrap()
        .children()
        .filter(|n| n.has_tag_name("result"))
        .map(|result| {
            let field = |name: &str| result.children().find(|n| n.has_tag_name(name)).map(text);
            let name = field("name").unwrap();
            let message = field("message").map(|message| match name.as_str() {
                "NaNs" => {
                    let mut lines: Vec<&str> = message.split('\n').collect();
                    lines.sort_unstable();
                    lines.join("\n")
                }
                "Event/Failsafe" => {
                    let mut words: Vec<&str> = message.split(' ').collect();
                    words.sort_unstable();
                    words.join(" ")
                }
                _ => message,
            });
            (name, field("status").unwrap(), message, field("data"))
        })
        .collect();
    Shape {
        header,
        params,
        results,
    }
}

fn golden(name: &str) -> String {
    std::fs::read_to_string(testdata(&format!(
        "dataflash/golden/loganalysis/{name}.xml"
    )))
    .unwrap()
}

/// ArduPilot's own example log (`Tools/LogAnalyzer/examples/robert_lefebvre_octo_PM.log`, a 2013
/// APM 2 octo): the XML the port writes against the XML Python 2.7 wrote for it, and the
/// `logfile` element the path the runner was given.
#[test]
fn the_example_log_analyses_as_the_python_does() {
    let dir = scratch("robert");
    let log = testdata("dataflash/loganalyzer/robert_lefebvre_octo_PM.log");
    let xml = dir.join("robert.xml");
    let written = run_analyzer(&log, &xml).unwrap();
    assert_eq!(written, std::fs::read_to_string(&xml).unwrap());
    assert_eq!(shape(&written), shape(&golden("robert_lefebvre_octo_PM")));
    assert!(written.contains(&format!("<logfile>{}</logfile>", log.display())));
    assert!(written.contains("<sizekb>302.7548828125</sizekb>"));
    assert!(written.contains("<coptertype>octo</coptertype>"));
    std::fs::remove_dir_all(&dir).unwrap();
}

/// The checked-in `.bin` logs, converted by the port and analysed, against the XML Python 2.7
/// wrote for the port's own conversion of them; and the text log, as it is.
#[test]
fn the_checked_in_logs_analyse_as_the_python_does() {
    let dir = scratch("checked-in");
    for (log, name) in [
        ("dataflash.bin", "dataflash"),
        ("dataflash/edge.bin", "edge"),
    ] {
        let xml = dir.join(format!("{name}.xml"));
        let analysis = analyse_to(&testdata(log), Some(&xml), &flight_mode_name).unwrap();
        let written = std::fs::read_to_string(&xml).unwrap();
        assert_eq!(shape(&written), shape(&golden(name)), "{name}");
        // The temporary .log the .bin was converted to is named in the XML and deleted after.
        let temp = analysis.logfile.unwrap();
        assert!(temp.ends_with(".tmp.log"), "{temp}");
        assert!(!Path::new(&temp).exists());
    }
    let xml = dir.join("synthetic.xml");
    let written = run_analyzer(&testdata("dataflash/synthetic.log"), &xml).unwrap();
    assert_eq!(shape(&written), shape(&golden("synthetic")));
    std::fs::remove_dir_all(&dir).unwrap();
}

/// `analyse` on a `.bin`: the XML is left beside the temporary `.log`, as `CheckLogFile` leaves
/// it, and the report shows the C#'s reading of it - the last check dropped.
#[test]
fn analyse_leaves_the_xml_beside_the_temporary_log() {
    let analysis = analyse(&testdata("dataflash/edge.bin"), &flight_mode_name).unwrap();
    let temp = PathBuf::from(analysis.logfile.clone().unwrap());
    let xml = xml_path_for(&temp);
    assert!(xml.exists(), "{}", xml.display());
    std::fs::remove_file(&xml).unwrap();
    assert_eq!(analysis.vehicletype.as_deref(), Some("ArduPlane"));
    assert_eq!(analysis.firmwareversion.as_deref(), Some("V4.5.7"));
    let names: Vec<&str> = analysis
        .results
        .iter()
        .map(|r| r.name.as_deref().unwrap())
        .collect();
    // A plane: the copter-only checks are NA, which the XML names without a message;
    // Vibration, the last, is never shown.
    assert_eq!(
        names,
        [
            "Autotune",
            "Brownout",
            "Compass",
            "Dupe Log Data",
            "Empty",
            "Event/Failsafe",
            "GPS",
            "IMU Mismatch",
            "Motor Balance",
            "NaNs",
            "OpticalFlow",
            "Parameters",
            "PM",
            "Pitch/Roll",
            "Thrust",
            "VCC"
        ]
    );
    let text = report(&analysis);
    assert!(text.contains("Test: Autotune = NA - \r\n"), "{text}");
    assert!(
        text.contains("Test: NaNs = FAIL - Found NaN in EDGE.f\n\r\n"),
        "{text}"
    );
}

/// A corpus of the owner's own logs, each `<name>.log` beside the `<name>.xml` Python 2.7 wrote
/// for it: `MP_LOGANALYZER_CORPUS=<dir> cargo test -p mp-log --test analysis -- --ignored`.
#[test]
#[ignore = "needs MP_LOGANALYZER_CORPUS"]
fn the_corpus_analyses_as_the_python_does() {
    let corpus =
        PathBuf::from(std::env::var("MP_LOGANALYZER_CORPUS").expect("MP_LOGANALYZER_CORPUS"));
    let dir = scratch("corpus");
    let mut compared = 0;
    for entry in std::fs::read_dir(&corpus).unwrap() {
        let log = entry.unwrap().path();
        if log.extension().is_none_or(|e| e != "log") {
            continue;
        }
        let expected = log.with_extension("xml");
        if !expected.exists() {
            continue;
        }
        let name = log.file_name().unwrap().to_string_lossy().into_owned();
        let written = run_analyzer(&log, &dir.join(format!("{name}.xml"))).unwrap();
        assert_eq!(
            shape(&written),
            shape(&std::fs::read_to_string(&expected).unwrap()),
            "{name}"
        );
        compared += 1;
    }
    assert!(compared > 0, "nothing to compare in {}", corpus.display());
    std::fs::remove_dir_all(&dir).unwrap();
}
