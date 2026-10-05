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

//! Crash reports: `Program.cs`'s `handleException`, which every unhandled exception reaches
//! (`AppDomain.UnhandledException`, `Application.ThreadException`) - "An error has occurred ...
//! Report this Error???" with Yes and No, Yes asking for a message in an `InputBox` and posting
//! the report (the OS, the versions, the exception, its stack, its site, its data, the message
//! and the threads' stacks), and "Could not send report! Typically due to lack of internet
//! connection." when that fails.
//!
//! A Rust panic ends the gpui process, and there is no window left to ask from; so the hook
//! writes the report under the data directory's `crash-reports/` as the panic happens (the
//! backtrace symbolicated by the release profile's debug info), and the next start asks the C#'s
//! question about the first report not yet asked about, moving it to `crash-reports/seen/` once
//! answered. Where a report is posted is the setting `CrashReportUrl`: the C#'s address is
//! Mission Planner's own mail script (`http://vps.oborne.me/mail.php`), not this program's to
//! post to; without one, Yes keeps the report and the status line says where. The question shows
//! the report's first lines where the C# shows `ex.ToString()` whole. `Tracking.AddException`
//! (Google Analytics, Mission Planner's account) is not ported. The C#'s special cases - "The
//! port is closed", `MissingMethodException`, `FileNotFoundException` and the rest - are .NET's
//! exceptions, which a panic never is.
//! `// C#: Program.cs:80, 190, 677-691, 717-869; ExtLibs/Utilities/Download.cs:306-315`

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use mp_os::fs::FsExt as _;
use std::backtrace::Backtrace;
use std::panic::PanicHookInfo;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::sync::mpsc::{Receiver, TryRecvError, channel};

use gpui::{AnyElement, Context, KeyDownEvent, Window};
use mp_firmware::flow::Buttons;
use mp_firmware::manifest::Fetch;

use crate::MissionPlanner;
use crate::config::firmware::{BoxIds, question_box};
use crate::config::optional::{InputBox, input_box};
use crate::textfield::KeyOutcome;

/// Under the data directory: the reports, and the ones already asked about.
pub const REPORTS_DIR: &str = "crash-reports";
pub const SEEN_DIR: &str = "seen";
/// Where a report is posted: a setting, empty until the owner has somewhere.
pub const URL_KEY: &str = "CrashReportUrl";
/// The question's caption and words. `// C#: Program.cs:802-803`
pub const CAPTION: &str = "Send Error";
pub const HEAD: &str = "An error has occurred";
pub const TAIL: &str = "Report this Error???";
/// The message's `InputBox`. `// C#: Program.cs:815-816`
pub const MESSAGE_TITLE: &str = "Message";
pub const MESSAGE_PROMPT: &str = "Please enter a message about this error if you can.";
/// When the post fails. `// C#: Program.cs:866`
pub const COULD_NOT_SEND: &str =
    "Could not send report! Typically due to lack of internet connection.";
/// How many of the report's lines the question shows.
const SHOWN_LINES: usize = 8;

/// The most of each line the question shows: a page's report has the browser's whole stack on
/// one line, which, shown whole, made the box taller than the window and put its Yes and No out
/// of reach (the owner's browsers, 2026-10-05). The report itself keeps every line whole.
const SHOWN_LINE_CHARS: usize = 200;

/// The ids of the question.
const BOXES: BoxIds = BoxIds {
    question: "crash-question",
    yes: "crash-question-yes",
    no: "crash-question-no",
    message: "crash-box",
    ok: "crash-box-ok",
    path: "crash-path",
    path_value: "crash-path-value",
    path_ok: "crash-path-ok",
    path_cancel: "crash-path-cancel",
};

/// `Environment.OSVersion.VersionString`, as near as the standard library knows it.
fn os_version() -> String {
    format!("{} {}", std::env::consts::OS, std::env::consts::ARCH)
}

/// `'&'` and `'='` to spaces, as the C# clears the exception, the stack and the message of them
/// before posting. `// C#: Program.cs:855-858`
fn cleared(text: &str) -> String {
    text.replace(['&', '='], " ")
}

/// A report's body, as `handleException` composes `postData` up to the message: the OS and the
/// versions, the exception, its stack, its site and its data. `// C#: Program.cs:853-858`
#[must_use]
pub fn compose(message: &str, location: &str, thread: &str, stack: &str) -> String {
    let version = env!("CARGO_PKG_VERSION");
    format!(
        "message={} {version} {version}\nException panicked at {location}: {}\nStack: {}\nTargetSite {location} {thread}\ndata \n",
        os_version(),
        cleared(message),
        cleared(stack.trim_end())
    )
}

/// The report of a panic, as the hook sees it.
#[must_use]
pub fn report_text(info: &PanicHookInfo<'_>, backtrace: &Backtrace) -> String {
    let payload = info.payload();
    let message = payload
        .downcast_ref::<&str>()
        .map(|s| (*s).to_owned())
        .or_else(|| payload.downcast_ref::<String>().cloned())
        .unwrap_or_else(|| "Box<dyn Any>".to_owned());
    let location = info.location().map_or_else(
        || "unknown".to_owned(),
        |at| format!("{}:{}:{}", at.file(), at.line(), at.column()),
    );
    let thread = wasm_thread::current()
        .name()
        .unwrap_or("<unnamed>")
        .to_owned();
    compose(&message, &location, &thread, &backtrace.to_string())
}

/// `postData` whole: the report, the user's message, and the threads' stacks as JSON - here the
/// one thread the report has, its frames. `// C#: Program.cs:859-860`
#[must_use]
pub fn post_body(report: &str, message: &str) -> String {
    let thread = report
        .lines()
        .find_map(|line| line.strip_prefix("TargetSite "))
        .and_then(|rest| rest.split(' ').nth(1))
        .unwrap_or("main");
    let frames: Vec<&str> = report
        .lines()
        .skip_while(|line| !line.starts_with("Stack: "))
        .take_while(|line| !line.starts_with("TargetSite "))
        .map(|line| line.strip_prefix("Stack: ").unwrap_or(line).trim())
        .filter(|line| !line.is_empty())
        .collect();
    let process_info = serde_json::to_string_pretty(&serde_json::json!({ thread: frames }))
        .unwrap_or_else(|_| "{}".to_owned());
    format!("{report}message {}\n\n{process_info}", cleared(message))
}

/// The reports directory under `data_dir`.
fn reports_dir(data_dir: &Path) -> PathBuf {
    data_dir.join(REPORTS_DIR)
}

/// A report written as `crash-reports/<time>.txt`, the time to the second and a counter when
/// two fall in one.
///
/// # Errors
///
/// The directory or the file could not be written.
pub fn write_report(data_dir: &Path, text: &str) -> std::io::Result<PathBuf> {
    let dir = reports_dir(data_dir);
    mp_os::fs::create_dir_all(&dir)?;
    let stamp = chrono::Local::now().format("%Y-%m-%dT%H-%M-%S").to_string();
    let mut path = dir.join(format!("{stamp}.txt"));
    let mut counter = 1;
    while path.os_exists() {
        path = dir.join(format!("{stamp}-{counter}.txt"));
        counter += 1;
    }
    mp_os::fs::write(&path, text)?;
    Ok(path)
}

/// The reports not yet asked about, oldest first: by the stamp in the name, then by the counter
/// two reports of one second get - `<stamp>.txt` before `<stamp>-1.txt`, which a plain sort of the
/// names would put the other way round (`-` sorts before `.`).
#[must_use]
pub fn pending(data_dir: &Path) -> Vec<PathBuf> {
    let Ok(entries) = mp_os::fs::read_dir(reports_dir(data_dir)) else {
        return Vec::new();
    };
    let mut reports: Vec<PathBuf> = entries
        .flatten()
        .map(|e| e.path())
        .filter(|p| p.os_is_file() && p.extension().is_some_and(|e| e == "txt"))
        .collect();
    reports.sort_by_cached_key(|path| written_order(path));
    reports
}

/// Where a report's name puts it: its stamp, then its counter (none is 0), then the name itself
/// for a file the application did not write.
fn written_order(path: &Path) -> (String, u32, String) {
    let stem = path
        .file_stem()
        .map(|stem| stem.to_string_lossy().into_owned())
        .unwrap_or_default();
    let (stamp, counter) = match stem.split_at_checked(STAMP_LEN) {
        Some((stamp, rest)) if stamp.len() == STAMP_LEN => (
            stamp.to_owned(),
            rest.strip_prefix('-')
                .and_then(|counter| counter.parse().ok())
                .unwrap_or(0),
        ),
        _ => (stem.clone(), 0),
    };
    (stamp, counter, stem)
}

/// The length of a report's stamp, `%Y-%m-%dT%H-%M-%S`.
const STAMP_LEN: usize = "2026-10-03T12-22-26".len();

/// A report asked about: moved to `crash-reports/seen/`.
fn mark_seen(path: &Path) {
    let Some(dir) = path.parent() else {
        return;
    };
    let seen = dir.join(SEEN_DIR);
    if mp_os::fs::create_dir_all(&seen).is_ok()
        && let Some(name) = path.file_name()
    {
        let _ = mp_os::fs::rename(path, seen.join(name));
    }
}

/// The hook: the default's words on stderr, then the report written for the next start.
/// `// C#: Program.cs:80, 190`
pub fn install_hook(data_dir: Option<PathBuf>) {
    let default = std::panic::take_hook();
    std::panic::set_hook(Box::new(move |info| {
        default(info);
        let text = report_text(info, &Backtrace::force_capture());
        if let Some(dir) = &data_dir {
            match write_report(dir, &text) {
                Ok(path) => eprintln!("planner: crash report written to {}", path.display()),
                Err(why) => eprintln!("planner: crash report not written: {why}"),
            }
        }
    }));
}

/// What the conversation about a report is at.
pub enum Flow {
    /// Nothing to ask.
    Idle,
    /// "An error has occurred ... Report this Error???"
    Question {
        /// The report.
        path: PathBuf,
        /// Its text.
        report: String,
    },
    /// "Please enter a message about this error if you can."
    Message {
        path: PathBuf,
        report: String,
        input: InputBox,
    },
    /// The post, on its thread.
    Posting {
        path: PathBuf,
        receiver: Receiver<Result<String, String>>,
    },
}

/// The reports the last runs left, and the question about them.
pub struct Crash {
    flow: Flow,
    /// The reports still to ask about, after the one in hand.
    queue: Vec<PathBuf>,
    /// The network.
    fetch: Arc<dyn Fetch + Send + Sync>,
    /// Reports posted this run.
    posted: usize,
    /// What the last report came to, for the facts.
    last: String,
}

impl Crash {
    /// The reports under the data directory, the first already asked about.
    #[must_use]
    pub fn new(data_dir: Option<&Path>) -> Self {
        let reports = data_dir.map(pending).unwrap_or_default();
        Self::with(Arc::new(mp_firmware::manifest::Http), reports)
    }

    /// Over another network and these reports: the tests'.
    #[must_use]
    pub fn with(fetch: Arc<dyn Fetch + Send + Sync>, reports: Vec<PathBuf>) -> Self {
        let mut this = Self {
            flow: Flow::Idle,
            queue: reports,
            fetch,
            posted: 0,
            last: "none".to_owned(),
        };
        this.next();
        this
    }

    /// The next report's question, or nothing.
    fn next(&mut self) {
        self.flow = Flow::Idle;
        while !self.queue.is_empty() {
            let path = self.queue.remove(0);
            if let Ok(report) = mp_os::fs::read_to_string(&path) {
                self.flow = Flow::Question { path, report };
                return;
            }
        }
    }

    /// The question's text: "An error has occurred", the report's first lines, "Report this
    /// Error???". `// C#: Program.cs:802`
    #[must_use]
    pub fn question_text(&self) -> Option<String> {
        let Flow::Question { report, .. } = &self.flow else {
            return None;
        };
        let shown: Vec<String> = report
            .lines()
            .skip(1)
            .take(SHOWN_LINES)
            .map(|line| match line.char_indices().nth(SHOWN_LINE_CHARS) {
                Some((cut, _)) => format!("{}...", &line[..cut]),
                None => line.to_owned(),
            })
            .collect();
        Some(format!("{HEAD}\n{}\n\n{TAIL}", shown.join("\n")))
    }

    /// Yes asks for the message; No is the end of it. `// C#: Program.cs:804-817`
    pub fn answer(&mut self, yes: bool) {
        let Flow::Question { path, report } = std::mem::replace(&mut self.flow, Flow::Idle) else {
            return;
        };
        if yes {
            self.flow = Flow::Message {
                path,
                report,
                input: InputBox::new(MESSAGE_TITLE, MESSAGE_PROMPT, ""),
            };
        } else {
            self.last = "dismissed".to_owned();
            mark_seen(&path);
            self.next();
        }
    }

    /// The message box, while it asks.
    #[must_use]
    pub fn input(&self) -> Option<&InputBox> {
        match &self.flow {
            Flow::Message { input, .. } => Some(input),
            _ => None,
        }
    }

    /// A key in the message box.
    pub fn message_key(&mut self, event: &KeyDownEvent) -> KeyOutcome {
        match &mut self.flow {
            Flow::Message { input, .. } => input.field.key(event),
            _ => KeyOutcome::Ignored,
        }
    }

    /// The message given (or the box cancelled - the C# posts either way, the message then
    /// empty): the report posted to `url`, or kept where it is when there is no `url`. Returns
    /// what the status line says, if anything. `// C#: Program.cs:811-867`
    pub fn message_done(&mut self, url: Option<&str>) -> Option<String> {
        let Flow::Message {
            path,
            report,
            input,
        } = std::mem::replace(&mut self.flow, Flow::Idle)
        else {
            return None;
        };
        let body = post_body(&report, input.field.value());
        let url = url.unwrap_or("").trim().to_owned();
        if url.is_empty() {
            self.last = format!(
                "Crash report kept at {}; no {URL_KEY} is set",
                path.display()
            );
            mark_seen(&path);
            self.next();
            return Some(self.last.clone());
        }
        let (sender, receiver) = channel();
        let fetch = Arc::clone(&self.fetch);
        let spawned = wasm_thread::Builder::new()
            .name("mp-crash-report".to_owned())
            .spawn(move || {
                let _ = sender.send(fetch.post(&url, &body));
            });
        if spawned.is_err() {
            self.last = COULD_NOT_SEND.to_owned();
            mark_seen(&path);
            self.next();
            return Some(self.last.clone());
        }
        self.flow = Flow::Posting { path, receiver };
        None
    }

    /// The post's end, once a frame: nothing said on success, as the C# says nothing; "Could not
    /// send report!" otherwise.
    pub fn tick(&mut self) -> Option<String> {
        let Flow::Posting { path, receiver } = &self.flow else {
            return None;
        };
        let outcome = match receiver.try_recv() {
            Ok(outcome) => outcome,
            Err(TryRecvError::Empty) => {
                crate::repaint::in_flight();
                return None;
            }
            Err(TryRecvError::Disconnected) => Err("the post stopped without an answer".to_owned()),
        };
        let path = path.clone();
        mark_seen(&path);
        let status = match outcome {
            Ok(_) => {
                self.posted += 1;
                self.last = "report sent".to_owned();
                None
            }
            Err(_) => {
                self.last = COULD_NOT_SEND.to_owned();
                Some(self.last.clone())
            }
        };
        self.next();
        status
    }

    /// The facts a UI test asserts on.
    pub fn record_facts(&self) {
        use crate::facts::record;
        record(
            "crash.flow",
            match &self.flow {
                Flow::Idle => "idle",
                Flow::Question { .. } => "question",
                Flow::Message { .. } => "message",
                Flow::Posting { .. } => "posting",
            },
        );
        record(
            "crash.text",
            self.question_text().unwrap_or_else(|| "none".to_owned()),
        );
        record("crash.pending", self.queue.len());
        record("crash.posted", self.posted);
        record("crash.last", &self.last);
    }
}

impl MissionPlanner {
    /// Once a frame: the post's end on the status line.
    pub(crate) fn crash_tick(&mut self) {
        if let Some(status) = self.crash.tick() {
            self.file_status = Some(status);
        }
    }

    /// The message box's OK or Cancel: the report posted where `CrashReportUrl` says.
    pub(crate) fn crash_message_done(&mut self) {
        let url = self.persisted.get(URL_KEY).map(str::to_owned);
        if let Some(status) = self.crash.message_done(url.as_deref()) {
            self.file_status = Some(status);
        }
    }
}

/// The question or the message box, over whatever screen is showing.
pub fn overlay(
    this: &MissionPlanner,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    if let Some(text) = this.crash.question_text() {
        return Some(question_box(
            BOXES,
            CAPTION,
            &text,
            Buttons::YesNo,
            window,
            |this, yes| this.crash.answer(yes),
            cx,
        ));
    }
    let input = this.crash.input()?;
    Some(input_box(
        "crash-message",
        input,
        &this.crash_focus,
        window,
        |this, event| match this.crash.message_key(event) {
            KeyOutcome::Submitted | KeyOutcome::Cancelled => {
                this.crash_message_done();
                true
            }
            KeyOutcome::Changed => true,
            KeyOutcome::Ignored => false,
        },
        MissionPlanner::crash_message_done,
        MissionPlanner::crash_message_done,
        cx,
    ))
}

/// The facts a UI test asserts on.
pub fn record_facts(crash: &Crash) {
    crash.record_facts();
}

#[cfg(test)]
mod tests {
    use mp_os::fs::FsExt as _;
    use mp_os::Lock as _;
    use super::*;
    use std::sync::Mutex;

    struct Sink {
        posted: Mutex<Vec<(String, String)>>,
        fail: bool,
    }

    impl Fetch for Sink {
        fn get(&self, _url: &str) -> Result<Vec<u8>, String> {
            Err("not here".to_owned())
        }

        fn post(&self, url: &str, data: &str) -> Result<String, String> {
            if self.fail {
                return Err("no route".to_owned());
            }
            self.posted
                .os_lock()
                .expect("the sink")
                .push((url.to_owned(), data.to_owned()));
            Ok("ok".to_owned())
        }
    }

    fn scratch(test: &str) -> PathBuf {
        let dir = mp_os::temp_dir().join(format!("mp-gui-crash-{test}-{}", mp_os::process_id()));
        let _ = mp_os::fs::remove_dir_all(&dir);
        mp_os::fs::create_dir_all(&dir).expect("a scratch directory");
        dir
    }

    fn wait(crash: &mut Crash) -> Option<String> {
        for _ in 0..2000 {
            let status = crash.tick();
            if !matches!(crash.flow, Flow::Posting { .. }) {
                return status;
            }
            wasm_thread::sleep(std::time::Duration::from_millis(5));
        }
        None
    }

    #[test]
    fn the_report_is_the_csharps_post_data() {
        let report = compose(
            "boom & bang = 1",
            "crates/x.rs:10:5",
            "main",
            "   0: a::b\n             at /src/a.rs:1:2\n   1: c::d\n",
        );
        assert!(
            report.starts_with(&format!("message={} ", os_version())),
            "{report}"
        );
        assert!(
            report.contains("\nException panicked at crates/x.rs:10:5: boom   bang   1\n"),
            "{report}"
        );
        assert!(
            report.contains("\nStack:    0: a::b\n             at /src/a.rs:1:2\n   1: c::d\n"),
            "{report}"
        );
        assert!(
            report.ends_with("\nTargetSite crates/x.rs:10:5 main\ndata \n"),
            "{report}"
        );
        let body = post_body(&report, "it broke when = pressed");
        assert!(
            body.contains("\ndata \nmessage it broke when   pressed\n\n{"),
            "{body}"
        );
        assert!(
            body.contains(
                "\"main\": [\n    \"0: a::b\",\n    \"at /src/a.rs:1:2\",\n    \"1: c::d\"\n  ]"
            ),
            "{body}"
        );
    }

        /// Two reports of one second: the counter orders them, not the names' characters (`-` sorts
    /// before `.`, so a plain sort would list `<stamp>-1.txt` before `<stamp>.txt`).
    /// A page's report has the browser's whole stack on one line; the question shows the start
    /// of it, so its box fits the window and Yes and No can be reached (the owner's browsers,
    /// 2026-10-05: the box ran off the bottom of the page).
    #[test]
    fn a_long_line_is_cut_short_in_the_question() {
        let dir = scratch("long-line");
        let stack = "at planner.wasm.f | ".repeat(500);
        write_report(
            &dir,
            &format!("message=page\nException RuntimeError: unreachable\nStack: {stack}\n"),
        )
        .expect("written");
        let crash = Crash::new(Some(dir.as_path()));
        let text = crash.question_text().expect("asked");
        assert!(text.contains("Exception RuntimeError: unreachable"));
        let longest = text.lines().map(|line| line.chars().count()).max().unwrap_or(0);
        assert!(longest <= SHOWN_LINE_CHARS + 3, "a line of {longest} characters");
        assert!(text.contains("Stack: at planner.wasm.f"));
    }

    #[test]
    fn reports_of_one_second_list_in_the_order_written() {
        let dir = scratch("order");
        let reports = reports_dir(&dir);
        mp_os::fs::create_dir_all(&reports).expect("dir");
        let names = [
            "2026-10-03T12-22-26-1.txt",
            "2026-10-03T12-22-27.txt",
            "2026-10-03T12-22-26-2.txt",
            "2026-10-03T12-22-26.txt",
            "2026-10-03T12-22-26-10.txt",
        ];
        for name in names {
            mp_os::fs::write(reports.join(name), "x").expect("written");
        }
        let listed: Vec<String> = pending(&dir)
            .iter()
            .map(|path| path.file_name().unwrap_or_default().to_string_lossy().into_owned())
            .collect();
        assert_eq!(
            listed,
            [
                "2026-10-03T12-22-26.txt",
                "2026-10-03T12-22-26-1.txt",
                "2026-10-03T12-22-26-2.txt",
                "2026-10-03T12-22-26-10.txt",
                "2026-10-03T12-22-27.txt",
            ]
        );
    }

    #[test]
    fn reports_are_written_listed_and_asked_about_in_turn() {
        let dir = scratch("flow");
        let first =
            write_report(&dir, &compose("first", "a.rs:1:1", "main", "frames")).expect("written");
        let second =
            write_report(&dir, &compose("second", "b.rs:2:2", "main", "frames")).expect("written");
        assert_ne!(first, second);
        assert_eq!(pending(&dir), vec![first.clone(), second.clone()]);
        let sink = Arc::new(Sink {
            posted: Mutex::new(Vec::new()),
            fail: false,
        });
        let mut crash = Crash::with(sink.clone(), pending(&dir));
        let text = crash.question_text().expect("a question");
        assert!(
            text.starts_with("An error has occurred\nException panicked at a.rs:1:1: first\n"),
            "{text}"
        );
        assert!(text.ends_with("\n\nReport this Error???"), "{text}");
        // Yes, a message, and the post; the report is then seen.
        crash.answer(true);
        assert!(crash.input().is_some());
        if let Flow::Message { input, .. } = &mut crash.flow {
            input.field.set("my note");
        }
        assert!(crash.message_done(Some("https://x/mail.php")).is_none());
        assert!(wait(&mut crash).is_none());
        assert_eq!(crash.posted, 1);
        let posted = sink.posted.os_lock().expect("the sink");
        assert_eq!(posted.len(), 1);
        assert_eq!(posted[0].0, "https://x/mail.php");
        assert!(posted[0].1.contains("message my note\n"), "{}", posted[0].1);
        drop(posted);
        assert!(!first.os_exists());
        assert!(
            dir.join(REPORTS_DIR)
                .join(SEEN_DIR)
                .join(first.file_name().expect("name"))
                .exists()
        );
        // The second is asked about next; No puts it away.
        assert!(crash.question_text().is_some_and(|t| t.contains("second")));
        crash.answer(false);
        assert!(crash.question_text().is_none());
        assert_eq!(crash.last, "dismissed");
        assert!(pending(&dir).is_empty());
        mp_os::fs::remove_dir_all(&dir).expect("cleanup");
    }

    #[test]
    fn no_destination_keeps_the_report_and_a_failed_post_says_so() {
        let dir = scratch("kept");
        let report =
            write_report(&dir, &compose("kept", "k.rs:1:1", "main", "frames")).expect("written");
        let sink = Arc::new(Sink {
            posted: Mutex::new(Vec::new()),
            fail: true,
        });
        let mut crash = Crash::with(sink.clone(), pending(&dir));
        crash.answer(true);
        let status = crash.message_done(None).expect("a status");
        assert_eq!(
            status,
            format!(
                "Crash report kept at {}; no CrashReportUrl is set",
                report.display()
            )
        );
        assert!(!report.os_exists(), "moved to seen");

        let again =
            write_report(&dir, &compose("again", "k.rs:2:2", "main", "frames")).expect("written");
        let mut crash = Crash::with(sink, pending(&dir));
        crash.answer(true);
        assert!(crash.message_done(Some("https://x/mail.php")).is_none());
        let status = wait(&mut crash).expect("a status");
        assert_eq!(status, COULD_NOT_SEND);
        assert!(!again.os_exists());
        mp_os::fs::remove_dir_all(&dir).expect("cleanup");
    }
}
