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

//! What the libraries under the planner report through the `log` crate, on standard error.
//!
//! gpui says what it could not do - no Metal device, a font family whose every face it skipped -
//! only through `log`, and with no logger installed those lines went nowhere: on a Mac without
//! Metal the planner exited 1 with nothing said, and on the owner's borrowed Mac (macOS 26,
//! 2026-10-04) it drew no text at all, with nothing said either. This prints them, a line each,
//! warnings and errors by default; `MP_LOG` (`error`, `warn`, `info`, `debug`, `trace`, `off`)
//! chooses another level, as the planner's other `MP_*` switches do. It is not Mission
//! Planner's log4net file, which this port does not keep: a line on stderr for what would
//! otherwise be lost.

use log::{LevelFilter, Log, Metadata, Record};

/// The logger: prints every record at or above the level `install` set.
struct StderrLog;

static LOGGER: StderrLog = StderrLog;

impl Log for StderrLog {
    fn enabled(&self, metadata: &Metadata) -> bool {
        metadata.level() <= log::max_level()
    }

    fn log(&self, record: &Record) {
        if self.enabled(record.metadata()) {
            eprintln!("{}", line(record));
        }
    }

    fn flush(&self) {}
}

/// One record as printed: `planner: WARN gpui_macos::text_system: ...`.
fn line(record: &Record) -> String {
    format!(
        "planner: {} {}: {}",
        record.level(),
        record.target(),
        record.args()
    )
}

/// The level `MP_LOG` names, warnings when it is unset or names none.
fn level(setting: Option<&str>) -> LevelFilter {
    setting
        .and_then(|s| s.trim().parse().ok())
        .unwrap_or(LevelFilter::Warn)
}

/// Installs the logger at the level `MP_LOG` names. A second call, or a logger already installed,
/// changes nothing.
pub(crate) fn install() {
    let setting = std::env::var("MP_LOG").ok();
    if log::set_logger(&LOGGER).is_ok() {
        log::set_max_level(level(setting.as_deref()));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn warnings_unless_mp_log_names_a_level() {
        assert_eq!(level(None), LevelFilter::Warn);
        assert_eq!(level(Some("")), LevelFilter::Warn);
        assert_eq!(level(Some("loud")), LevelFilter::Warn);
        assert_eq!(level(Some("debug")), LevelFilter::Debug);
        assert_eq!(level(Some(" ERROR ")), LevelFilter::Error);
        assert_eq!(level(Some("off")), LevelFilter::Off);
    }

    #[test]
    fn a_record_is_one_line_naming_its_level_and_source() {
        let text = format!("font {:?} has no PostScript name", "X");
        let printed = line(
            &Record::builder()
                .level(log::Level::Warn)
                .target("gpui_macos::text_system")
                .args(format_args!("{text}"))
                .build(),
        );
        assert_eq!(
            printed,
            "planner: WARN gpui_macos::text_system: font \"X\" has no PostScript name"
        );
    }

    /// The path a gpui warning takes: `install`, then the global logger takes a warning and not
    /// what is below it. (Skipped where `MP_LOG` is set in the environment running the tests.)
    #[test]
    fn installed_it_takes_warnings_and_not_what_is_below_them() {
        if std::env::var_os("MP_LOG").is_some() {
            eprintln!("skipped: MP_LOG is set; unset it to run this test");
            return;
        }
        install();
        let logger = log::logger();
        let at = |level| Metadata::builder().level(level).target("gpui").build();
        assert!(logger.enabled(&at(log::Level::Error)));
        assert!(logger.enabled(&at(log::Level::Warn)));
        assert!(!logger.enabled(&at(log::Level::Info)));
    }
}
