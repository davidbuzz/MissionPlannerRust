//! Auto Analysis: Mission Planner does not analyse a log itself. It runs ArduPilot's Python
//! `LogAnalyzer`, packaged by py2exe as `runner.exe`, and shows what that writes.
//!
//! `BUT_loganalysis_Click` (`GCSViews/FlightData.cs:1311-1385`) converts a `.bin` to a temporary
//! `.log` with `BinaryLog.ConvertBin`, then `LogAnalyzer.CheckLogFile` (`Utilities/LogAnalyzer.cs:
//! 18-115`) downloads `LogAnalyzer64.zip` (or `LogAnalyzer.zip` on a 32-bit system) from
//! firmware.ardupilot.org into `<data dir>/LogAnalyzer/` - every time, falling back to the copy
//! already there when the download fails - extracts it, and runs
//! `runner.exe -x "<log>.xml" -s "<log>"` there, waiting for it to finish. `LogAnalyzer.Results`
//! reads the XML it wrote and `Controls.LogAnalyzer` shows it as text.
//!
//! So this ports exactly that and no more: the download's URL and destination (the fetch itself is
//! the caller's, as it owns the network), the extraction, the command line, the reading of the
//! XML - quirks included - and the report text. The checks are the Python tool's; none of them is
//! reimplemented here. `runner.exe` is a Windows program: on any other system it does not start,
//! which is also what Mission Planner does there, and the error says so.
//!
//! `tests/analysis.rs` holds [`results`] and [`report`] to the C#'s own reading of the analyzer's
//! example output, byte for byte.
//! `// C#: GCSViews/FlightData.cs:1311-1385; Utilities/LogAnalyzer.cs; Controls/LogAnalyzer.cs`

use std::path::{Path, PathBuf};

use crate::convert::{ModeName, convert_bin_file};

/// What `CheckLogFile` downloads on a 64-bit system. `// C#: Utilities/LogAnalyzer.cs:42-44`
pub const ANALYZER_URL_64: &str =
    "https://firmware.ardupilot.org/Tools/MissionPlanner/LogAnalyzer/LogAnalyzer64.zip";
/// What `CheckLogFile` downloads on a 32-bit system. `// C#: Utilities/LogAnalyzer.cs:48-50`
pub const ANALYZER_URL_32: &str =
    "https://firmware.ardupilot.org/Tools/MissionPlanner/LogAnalyzer/LogAnalyzer.zip";

/// `Environment.NewLine`, which the report's header lines end with.
const NEWLINE: &str = if cfg!(windows) { "\r\n" } else { "\n" };

/// Why the analysis produced nothing to show. Each is one of the C#'s message boxes.
#[derive(Debug, thiserror::Error)]
pub enum AnalysisError {
    /// "File access issue: ..." - the `.bin` could not be converted.
    #[error("File access issue: {0}")]
    FileAccess(std::io::Error),
    /// "Failed to download LogAnalyzer": no download, and no runner from before.
    #[error("Failed to download LogAnalyzer")]
    Download,
    /// "Failed to start LogAnalyzer": the runner would not run - as on any system but Windows.
    #[error("Failed to start LogAnalyzer: {0}")]
    Start(std::io::Error),
    /// "Bad input file": the runner wrote no XML.
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

/// The URL `CheckLogFile` fetches: by the bitness of the system, `Is64BitOperatingSystem`.
#[must_use]
pub const fn analyzer_url() -> &'static str {
    if cfg!(target_pointer_width = "64") {
        ANALYZER_URL_64
    } else {
        ANALYZER_URL_32
    }
}

/// Where the analyzer lives: `<data dir>/LogAnalyzer/`. `// C#: Utilities/LogAnalyzer.cs:26-27`
#[must_use]
pub fn analyzer_dir(data_dir: &Path) -> PathBuf {
    data_dir.join("LogAnalyzer")
}

/// The command `CheckLogFile` runs: `runner.exe -x "<log>.xml" -s "<log>"`, in the analyzer's
/// directory, its output collected. `// C#: Utilities/LogAnalyzer.cs:80-91`
#[must_use]
pub fn runner_command(dir: &Path, log: &Path) -> std::process::Command {
    let mut xml = log.as_os_str().to_owned();
    xml.push(".xml");
    let mut command = std::process::Command::new(dir.join("runner.exe"));
    command
        .arg("-x")
        .arg(xml)
        .arg("-s")
        .arg(log)
        .current_dir(dir)
        .stdout(std::process::Stdio::piped())
        .stderr(std::process::Stdio::piped());
    command
}

/// `CheckLogFile`: makes sure the analyzer is there - `fetch(url, zip)` downloads it, returning
/// whether it did, and a fresh download is extracted over the old - runs it on `log` and returns
/// where it was told to write its XML, `<log>.xml`, whether or not it did.
///
/// # Errors
///
/// [`AnalysisError::Download`] when there is neither a download nor a runner from before, and
/// [`AnalysisError::Start`] when the runner will not run.
/// `// C#: Utilities/LogAnalyzer.cs:18-115`
pub fn check_log_file(
    log: &Path,
    dir: &Path,
    fetch: &mut dyn FnMut(&str, &Path) -> bool,
) -> Result<PathBuf, AnalysisError> {
    let runner = dir.join("runner.exe");
    let zip = dir.join("LogAnalyzer.zip");
    std::fs::create_dir_all(dir).map_err(|_| AnalysisError::Download)?;
    if fetch(analyzer_url(), &zip) {
        let data = std::fs::read(&zip).map_err(|_| AnalysisError::Download)?;
        crate::zip::extract(&data, dir).map_err(|_| AnalysisError::Download)?;
    } else if !runner.exists() {
        return Err(AnalysisError::Download);
    }
    if !runner.exists() {
        return Err(AnalysisError::Download);
    }
    // "until we are done": the output is only logged.
    runner_command(dir, log)
        .output()
        .map_err(AnalysisError::Start)?;
    let mut xml = log.as_os_str().to_owned();
    xml.push(".xml");
    Ok(PathBuf::from(xml))
}

/// Auto Analysis (`BUT_loganalysis`) for one file: a `.bin` is converted to a temporary `.log`
/// first, the analyzer is run on it ([`check_log_file`]), and what it wrote is read
/// ([`results`]); the temporary `.log` is deleted afterwards, its `.xml` is not.
///
/// # Errors
///
/// Each of the button's message boxes: see [`AnalysisError`].
/// `// C#: GCSViews/FlightData.cs:1311-1385`
pub fn analyse(
    log: &Path,
    dir: &Path,
    fetch: &mut dyn FnMut(&str, &Path) -> bool,
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
    let outcome = check_log_file(file, dir, fetch).and_then(|xml| {
        let text = std::fs::read_to_string(&xml).map_err(|_| AnalysisError::BadInputFile)?;
        results(&text).map_err(AnalysisError::Results)
    });
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
    fn the_command_line_is_the_csharps() {
        let command = runner_command(Path::new("/data/LogAnalyzer"), Path::new("/logs/a b.log"));
        assert_eq!(
            command.get_program(),
            Path::new("/data/LogAnalyzer/runner.exe")
        );
        let args: Vec<_> = command.get_args().collect();
        assert_eq!(args, ["-x", "/logs/a b.log.xml", "-s", "/logs/a b.log"]);
        assert_eq!(
            command.get_current_dir(),
            Some(Path::new("/data/LogAnalyzer"))
        );
    }
}
