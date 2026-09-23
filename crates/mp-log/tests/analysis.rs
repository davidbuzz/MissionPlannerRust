//! "Auto Analysis": the analyzer is ArduPilot's Python tool, run as `runner.exe`, and only its
//! invocation and its report are ported. The report is held to the C#'s own reading of the
//! analyzer's example output - `LogAnalyzer.Results` and `Controls.LogAnalyzer`'s text, run under
//! mono by `tools/csharp-reference/regen-log.sh` - byte for byte; the invocation is run against a
//! stand-in runner that writes that same XML where it is told to.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use std::path::{Path, PathBuf};

use mp_log::analysis::{Analysis, analyse, analyzer_url, check_log_file, report, results};
use mp_log::convert::flight_mode_name;

fn testdata(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../testdata")
        .join(name)
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
    // The header's lines end with Environment.NewLine, the tests' with "\r\n"; the golden was
    // written where the newline is "\n".
    let ours = report(&example());
    let ours = if cfg!(windows) {
        ours.replace("\r\n", "\n").replace("\n", "\r\n")
    } else {
        ours
    };
    assert_eq!(ours, golden);
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

/// A `.bin` is converted to a temporary `.log`, the runner is fetched and run on it with the C#'s
/// arguments, and what it writes is read back. The stand-in runner is a shell script, so this runs
/// where a script can be a program.
#[cfg(unix)]
#[test]
fn the_analyzer_is_run_on_a_converted_log() {
    use std::os::unix::fs::PermissionsExt;

    let dir = std::env::temp_dir().join(format!("mp-log-analysis-{}", std::process::id()));
    let analyzer = dir.join("LogAnalyzer");
    std::fs::create_dir_all(&analyzer).unwrap();
    let runner = analyzer.join("runner.exe");
    // `-x <xml> -s <log>`: copy the example output to the XML path, and record the arguments.
    std::fs::write(
        &runner,
        format!(
            "#!/bin/sh\nprintf '%s\\n' \"$@\" > \"$PWD/args\"\ncp '{}' \"$2\"\n",
            testdata("dataflash/example_output.xml").display()
        ),
    )
    .unwrap();
    std::fs::set_permissions(&runner, std::fs::Permissions::from_mode(0o755)).unwrap();

    let log = dir.join("flight.BIN");
    std::fs::copy(testdata("dataflash.bin"), &log).unwrap();
    let mut fetched = Vec::new();
    // The download fails, and the runner from before is used.
    let mut fetch = |url: &str, zip: &Path| {
        fetched.push((url.to_owned(), zip.to_owned()));
        false
    };
    let analysis = analyse(&log, &analyzer, &mut fetch, &flight_mode_name).unwrap();
    assert_eq!(analysis, example());
    assert_eq!(
        fetched,
        [(analyzer_url().to_owned(), analyzer.join("LogAnalyzer.zip"))]
    );

    let args = std::fs::read_to_string(analyzer.join("args")).unwrap();
    let args: Vec<&str> = args.lines().collect();
    assert_eq!(args.len(), 4);
    assert_eq!(args[0], "-x");
    assert_eq!(args[2], "-s");
    assert!(
        args[3].ends_with(".log"),
        "the .bin was converted: {}",
        args[3]
    );
    assert_eq!(args[1], format!("{}.xml", args[3]));
    assert!(
        !Path::new(args[3]).exists(),
        "the temporary .log is deleted"
    );
    std::fs::remove_file(args[1]).unwrap();
    std::fs::remove_dir_all(&dir).unwrap();
}

/// No download and no runner from before: the button's "Failed to download LogAnalyzer".
#[test]
fn no_runner_is_a_failed_download() {
    let dir = std::env::temp_dir().join(format!("mp-log-analysis-none-{}", std::process::id()));
    let outcome = check_log_file(Path::new("x.log"), &dir, &mut |_, _| false);
    assert!(matches!(
        outcome,
        Err(mp_log::analysis::AnalysisError::Download)
    ));
    std::fs::remove_dir_all(&dir).unwrap();
}
