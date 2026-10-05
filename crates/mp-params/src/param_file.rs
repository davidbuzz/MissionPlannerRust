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

//! Reading, writing and comparing `.param` files.
//!
//! This is how an operator backs up a build before changing it, clones a setup onto a second
//! airframe, and works out what a suggested change actually changed. Mission Planner writes these
//! and so do MAVProxy, QGroundControl and every forum post offering a tune, so the format has to
//! be read as tolerantly as all of them write it and written as plainly as all of them read it.
//!
//! The one thing not to get clever about is failure. A file with three unreadable lines in a
//! thousand is a file with nine hundred and ninety-seven good parameters, and refusing the whole
//! of it helps nobody; but a load that silently skipped three lines and said nothing is worse,
//! because the operator believes the airframe matches the file. So nothing is rejected outright
//! and nothing is discarded quietly: every line that did not become a parameter is reported.

use crate::round_to_significant_digits;
use std::collections::BTreeMap;

/// Significant digits used when comparing, matching Mission Planner's comparison behaviour.
///
/// A parameter round-trips through a 32-bit float on the wire and through decimal text in the
/// file, and neither is exact. Comparing the raw doubles reports every parameter on the vehicle
/// as different from the file it was just written from, which makes the comparison useless at
/// precisely the moment it is wanted.
const COMPARE_DIGITS: i32 = 7;

/// Parameters Mission Planner drops when it **reads** a file, and why.
///
/// Verbatim from `ExtLibs/Utilities/ParamFile.cs:50-76`, in its order. Every one is state rather
/// than configuration - a count the vehicle maintains, a sensor reading it takes at boot, a
/// bookkeeping value. Writing them into an airframe is at best meaningless and at worst harmful:
/// `ARSPD_OFFSET`, `GND_ABS_PRESS`, `GND_TEMP` and the `BARO*_GND_*` family are calibration
/// readings taken on the day, and loading yesterday's onto a vehicle sitting at a different
/// pressure gives it a wrong idea of its own altitude.
///
/// `WP_TOTAL`, `CMD_TOTAL` and `FENCE_TOTAL` are worse than meaningless: they say how many mission
/// items the vehicle has, and setting one without writing the items claims a mission that is not
/// there.
///
/// **On the load side, not the save side.** `SaveParamFile` writes whatever it is handed - a file
/// Mission Planner saved *contains* these - and `loadParamFile` is where they are skipped. An
/// earlier version of this had the list on save and seven entries long, reconstructed from memory
/// while believing the C# source was not on this machine. It is, at
/// https://github.com/ArduPilot/MissionPlanner, and reading it corrected the length, the side and the number
/// format all at once.
pub const NOT_LOADED: &[&str] = &[
    "SYSID_SW_MREV",
    "WP_TOTAL",
    "CMD_TOTAL",
    "FENCE_TOTAL",
    "SYS_NUM_RESETS",
    "ARSPD_OFFSET",
    "GND_ABS_PRESS",
    "GND_TEMP",
    "BARO1_GND_PRESS",
    "BARO2_GND_PRESS",
    "BARO3_GND_PRESS",
    "BARO_GND_TEMP",
    "CMD_INDEX",
    "LOG_LASTFILE",
    "FORMAT_VERSION",
];

/// The statistics group, which the C# list does not cover because it predates it.
///
/// `STAT_RUNTIME`, `STAT_FLTTIME`, `STAT_FLTCNT`, `STAT_BOOTCNT`, `STAT_DISTFLWN` and
/// `STAT_RESET` are counters ArduPilot accumulates over the life of the airframe. Loading a
/// backup that contains them winds the vehicle's own history back to whenever the backup was
/// taken - seen for real while testing this: a file saved five minutes earlier proposed
/// `STAT_RUNTIME 1301 -> 1022`, which is a ground station telling an aircraft it has flown less
/// than it has.
///
/// A deliberate departure from the reference implementation rather than an oversight in reading
/// it. The reasoning behind the C# list is "state the vehicle maintains, not configuration the
/// operator chose", and these are squarely that; the list is simply older than the parameters.
const NOT_LOADED_PREFIXES: &[&str] = &["STAT_"];

/// Whether a parameter read from a file is one to keep.
///
/// The `loadParamFile` filter. Mission Planner drops these while reading, so a file that contains
/// them loads without them.
#[must_use]
pub fn is_loaded(name: &str) -> bool {
    !NOT_LOADED.contains(&name)
        && !NOT_LOADED_PREFIXES
            .iter()
            .any(|prefix| name.starts_with(prefix))
}

/// A line that could not be read as a parameter.
///
/// Carried rather than logged, so the caller can show the operator which lines of their file were
/// ignored. A file half of which failed to parse usually means the wrong file was chosen.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Rejected {
    /// The line number, counting from one, as an editor would show it.
    pub line: usize,
    /// The line itself, so the operator can see what was wrong with it.
    pub text: String,
    /// What was wrong.
    pub reason: RejectReason,
}

/// Why a line was not read as a parameter.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RejectReason {
    /// A name with no value after it.
    NoValue,
    /// A value that is not a number.
    NotANumber,
    /// A name longer than the 16 bytes MAVLink allows, which no vehicle could accept.
    NameTooLong,
    /// The same name appeared earlier in the file with a different value.
    Duplicate,
}

impl std::fmt::Display for RejectReason {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Self::NoValue => "no value",
            Self::NotANumber => "value is not a number",
            Self::NameTooLong => "name is longer than 16 characters",
            Self::Duplicate => "name appears twice with different values",
        })
    }
}

/// A parsed `.param` file.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct ParamFile {
    /// Sorted, because that is the order Mission Planner writes and the order a human diffs.
    values: BTreeMap<String, f64>,
    /// Lines that did not become parameters, in file order.
    rejected: Vec<Rejected>,
}

impl ParamFile {
    /// An empty file.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Reads a file's text.
    ///
    /// Accepts every separator the ecosystem uses - Mission Planner writes commas, MAVProxy and
    /// ArduPilot's own defaults files write whitespace, and some tools write tabs - because a
    /// parameter file is something an operator is handed rather than something they generate, and
    /// refusing one over its separator is refusing to do the job.
    ///
    /// `#` starts a comment, wherever it appears. Mission Planner writes a `#NOTE:` header and
    /// hand-edited files carry trailing notes about what a line is for.
    #[must_use]
    pub fn parse(text: &str) -> Self {
        let mut file = Self::new();
        for (index, raw) in text.lines().enumerate() {
            let line = index + 1;
            let body = raw.split('#').next().unwrap_or("").trim();
            if body.is_empty() {
                continue;
            }
            let mut fields = body.split([',', ' ', '\t']).filter(|f| !f.is_empty());
            let Some(name) = fields.next() else {
                continue;
            };
            let name = name.trim().to_uppercase();
            let Some(value) = fields.next() else {
                file.reject(line, raw, RejectReason::NoValue);
                continue;
            };
            if name.len() > 16 {
                file.reject(line, raw, RejectReason::NameTooLong);
                continue;
            }
            // Mission Planner writes `1.000000`; ArduPilot's defaults files write `1`; a
            // hand-edited file may hold `1e-3`. Rust's float parser takes all three.
            let Ok(value) = value.trim().parse::<f64>() else {
                file.reject(line, raw, RejectReason::NotANumber);
                continue;
            };
            if !value.is_finite() {
                file.reject(line, raw, RejectReason::NotANumber);
                continue;
            }
            // Dropped here, on read, because that is where Mission Planner drops them
            // (`loadParamFile`). Not a rejection: the line was perfectly well formed and the file
            // is not wrong for containing it - a file Mission Planner saved always will.
            if !is_loaded(&name) {
                continue;
            }
            // A repeat with the same value is a harmless duplicate and the last one wins, which is
            // what every other tool does. A repeat with a *different* value is a file that
            // disagrees with itself, and the operator should be told rather than served whichever
            // line happened to come last.
            match file.values.insert(name, value) {
                Some(previous) if !same(previous, value) => {
                    file.reject(line, raw, RejectReason::Duplicate);
                }
                _ => {}
            }
        }
        file
    }

    fn reject(&mut self, line: usize, text: &str, reason: RejectReason) {
        self.rejected.push(Rejected {
            line,
            text: text.trim().to_owned(),
            reason,
        });
    }

    /// Builds one from name/value pairs.
    ///
    /// Everything given, nothing filtered - `SaveParamFile` writes whatever it is handed, and a
    /// `.param` file Mission Planner saved contains `WP_TOTAL` and the rest. They are dropped when
    /// the file is read back, by both implementations, which is where the protection actually is.
    pub fn from_values<I, S>(values: I) -> Self
    where
        I: IntoIterator<Item = (S, f64)>,
        S: Into<String>,
    {
        let mut file = Self::new();
        for (name, value) in values {
            file.values.insert(name.into(), value);
        }
        file
    }

    /// The parameters, sorted by name.
    pub fn iter(&self) -> impl Iterator<Item = (&str, f64)> {
        self.values
            .iter()
            .map(|(name, value)| (name.as_str(), *value))
    }

    /// Looks one up.
    #[must_use]
    pub fn get(&self, name: &str) -> Option<f64> {
        self.values.get(name).copied()
    }

    /// Adds or replaces one, whether or not it is on the skip-list.
    pub fn insert(&mut self, name: impl Into<String>, value: f64) {
        self.values.insert(name.into(), value);
    }

    /// How many parameters were read.
    #[must_use]
    pub fn len(&self) -> usize {
        self.values.len()
    }

    /// Whether nothing was read.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.values.is_empty()
    }

    /// Lines that did not become parameters, in file order.
    #[must_use]
    pub fn rejected(&self) -> &[Rejected] {
        &self.rejected
    }

    /// Renders the file, in the form Mission Planner writes.
    ///
    /// `NAME,VALUE`, sorted by name, each value as its **shortest representation** - `1` not
    /// `1.000000`, `0.3` not `0.300000`. That is what `SaveParamFile` does:
    /// `value.ToString(CultureInfo.InvariantCulture)`, whose two branches are both the same
    /// expression. An earlier version of this wrote six decimal places and a commit message
    /// asserted it was byte-for-byte what the C# produces; it was not, and the fixture had been
    /// written to match the invention rather than the original.
    ///
    /// Rust's `{}` for `f64` prints the shortest string that round-trips, which agrees with .NET's
    /// invariant `ToString` on every value a parameter can hold - these arrive as `f32` widened to
    /// `f64` and rounded to seven significant digits, well inside the range where the two agree.
    #[must_use]
    pub fn render(&self) -> String {
        let mut out = String::new();
        for (name, value) in &self.values {
            out.push_str(name);
            out.push(',');
            out.push_str(&invariant_double(*value));
            out.push('\n');
        }
        out
    }

    /// Writes the file, replacing atomically.
    ///
    /// A parameter backup is written at the moment an operator is about to change something, and
    /// a half-written backup is worse than none: it looks like a backup. Written to a temporary
    /// name beside the target and renamed over it, so the file either has the old contents or the
    /// new ones.
    ///
    /// # Errors
    /// If the directory cannot be written to, or the rename fails.
    pub fn save(&self, path: &std::path::Path) -> std::io::Result<()> {
        let temporary = path.with_extension("param.tmp");
        mp_os::fs::write(&temporary, self.render())?;
        mp_os::fs::rename(&temporary, path)
    }

    /// Reads a file from disk.
    ///
    /// # Errors
    /// If the file cannot be read. A file that parses badly is not an error - see `rejected`.
    pub fn load(path: &std::path::Path) -> std::io::Result<Self> {
        Ok(Self::parse(&mp_os::fs::read_to_string(path)?))
    }

    /// Compares this file against another, as "what would change if the other were loaded".
    ///
    /// The receiver is the *current* state and the argument is the *proposed* one, so a
    /// `Difference::Changed` reads "this parameter is currently `from` and would become `to`".
    /// Getting that direction backwards is the easy mistake and the one that matters, because the
    /// whole point of comparing is deciding whether to apply.
    #[must_use]
    pub fn compare(&self, proposed: &Self) -> Vec<Difference> {
        let mut differences = Vec::new();
        for (name, current) in &self.values {
            // A parameter that would never be written is not a difference worth reporting. A
            // comparison answers "what would change if I loaded this", and a file saved before
            // the statistics group was skipped still carries six counters that can only ever be
            // noise in that answer.
            if !is_loaded(name) {
                continue;
            }
            match proposed.values.get(name) {
                Some(&new) if !same(*current, new) => differences.push(Difference {
                    name: name.clone(),
                    kind: Change::Changed {
                        from: *current,
                        to: new,
                    },
                }),
                Some(_) => {}
                None => differences.push(Difference {
                    name: name.clone(),
                    kind: Change::Missing { from: *current },
                }),
            }
        }
        for (name, &new) in &proposed.values {
            if !is_loaded(name) {
                continue;
            }
            if !self.values.contains_key(name) {
                differences.push(Difference {
                    name: name.clone(),
                    kind: Change::Added { to: new },
                });
            }
        }
        differences.sort_by(|a, b| a.name.cmp(&b.name));
        differences
    }
}

/// Formats a double the way .NET's `double.ToString(CultureInfo.InvariantCulture)` does.
///
/// `SaveParamFile` writes exactly that, so this is what byte-identity with a Mission-Planner-saved
/// file depends on. Rust's `{}` is close but not the same: .NET Framework's parameterless
/// `ToString` is `G15`, which rounds to fifteen significant digits and **switches to scientific
/// notation when the exponent is -5 or smaller**. Rust never switches. The values where they
/// disagree are not exotic - `INS_GYROFFS_Z` at 8.9e-5 is written `8.9E-05` by the C# and
/// `0.000089` by `{}`, and gyro and accelerometer offsets live in exactly that range.
///
/// Implemented from the documented `G15` rules rather than captured from a running .NET, because
/// this machine has only mono and `PLAN.md` R5 records that mono diverges from .NET 4.7.2 on
/// precisely float formatting. The ground truth needs the Windows runner §7.1 already budgets;
/// until then this is a careful reading of the specification, and it is labelled as one.
#[must_use]
pub fn invariant_double(value: f64) -> String {
    if value == 0.0 {
        // Covers -0.0 too, which .NET prints as "0".
        return "0".to_owned();
    }
    if !value.is_finite() {
        return if value.is_nan() {
            "NaN".to_owned()
        } else if value > 0.0 {
            "Infinity".to_owned()
        } else {
            "-Infinity".to_owned()
        };
    }

    // Fifteen significant digits, then the trailing zeros G format removes.
    let rounded: f64 = format!("{value:.*e}", G_DIGITS - 1)
        .parse()
        .unwrap_or(value);
    #[allow(clippy::cast_possible_truncation)] // a f64 exponent fits an i32 many times over
    let exponent = rounded.abs().log10().floor() as i32;

    // "If the exponent is greater than -5 and less than the precision specifier, fixed-point
    // notation is used; otherwise scientific." -5 itself is therefore scientific.
    if exponent > -5 && exponent < G_DIGITS_I32 {
        let places = usize::try_from((G_DIGITS_I32 - 1 - exponent).max(0)).unwrap_or(0);
        let text = format!("{rounded:.places$}");
        return trim_trailing_zeros(&text);
    }

    let mantissa = trim_trailing_zeros(&format!("{:.*}", G_DIGITS - 1, rounded / powi10(exponent)));
    // .NET writes the exponent with a sign and at least two digits: E-05, E+16.
    let sign = if exponent < 0 { '-' } else { '+' };
    format!("{mantissa}E{sign}{:02}", exponent.abs())
}

/// Significant digits in .NET's parameterless `double.ToString`.
const G_DIGITS: usize = 15;
/// The same, as an `i32`, for the exponent comparisons.
const G_DIGITS_I32: i32 = 15;

/// Ten raised to a signed power, without `powi` on a negative exponent losing precision.
fn powi10(exponent: i32) -> f64 {
    if exponent >= 0 {
        10f64.powi(exponent)
    } else {
        1.0 / 10f64.powi(-exponent)
    }
}

/// Removes the trailing zeros `G` format does not print, and a trailing point with them.
fn trim_trailing_zeros(text: &str) -> String {
    if !text.contains('.') {
        return text.to_owned();
    }
    text.trim_end_matches('0').trim_end_matches('.').to_owned()
}

/// Whether two values are the same parameter value, at the precision a parameter survives.
fn same(a: f64, b: f64) -> bool {
    round_to_significant_digits(a, COMPARE_DIGITS) == round_to_significant_digits(b, COMPARE_DIGITS)
}

/// One parameter that differs between two sets.
#[derive(Debug, Clone, PartialEq)]
pub struct Difference {
    /// The parameter's name.
    pub name: String,
    /// How it differs.
    pub kind: Change,
}

/// How a parameter differs between two sets.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Change {
    /// Present in both, with different values.
    Changed {
        /// The current value.
        from: f64,
        /// The proposed value.
        to: f64,
    },
    /// Present only in the proposed set.
    ///
    /// Usually a different firmware version rather than a mistake: a parameter that exists in the
    /// file and not on the vehicle cannot be written, and saying so is more useful than failing
    /// the whole load.
    Added {
        /// The proposed value.
        to: f64,
    },
    /// Present only in the current set.
    Missing {
        /// The current value.
        from: f64,
    },
}

impl std::fmt::Display for Difference {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self.kind {
            Change::Changed { from, to } => {
                write!(f, "{:<16} {from} -> {to}", self.name)
            }
            Change::Added { to } => write!(f, "{:<16} (absent) -> {to}", self.name),
            Change::Missing { from } => write!(f, "{:<16} {from} -> (absent)", self.name),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The separator is whatever the tool that wrote the file felt like using.
    #[test]
    fn every_separator_the_ecosystem_uses_is_read() {
        let file = ParamFile::parse(
            "ACRO_RP_RATE,360.0\n\
             ACRO_Y_RATE 202.5\n\
             ATC_ANG_PIT_P\t4.5\n\
             ATC_ANG_RLL_P   4.500000\n",
        );
        assert_eq!(file.len(), 4);
        assert_eq!(file.get("ACRO_RP_RATE"), Some(360.0));
        assert_eq!(file.get("ACRO_Y_RATE"), Some(202.5));
        assert_eq!(file.get("ATC_ANG_PIT_P"), Some(4.5));
        assert_eq!(file.get("ATC_ANG_RLL_P"), Some(4.5));
        assert!(file.rejected().is_empty());
    }

    /// Mission Planner's own header, and hand-written notes, are comments.
    #[test]
    fn comments_and_blank_lines_are_not_parameters() {
        let file = ParamFile::parse(
            "#NOTE: this file was written by a ground station\n\
             \n\
             ATC_ANG_PIT_P,4.5  # raised after the first hover\n\
             # ATC_ANG_RLL_P,4.5\n",
        );
        assert_eq!(file.len(), 1);
        assert_eq!(file.get("ATC_ANG_PIT_P"), Some(4.5));
        assert!(file.rejected().is_empty(), "{:?}", file.rejected());
    }

    /// A bad line loses that line, not the file - but it is reported rather than swallowed.
    #[test]
    fn unreadable_lines_are_reported_and_the_rest_is_kept() {
        let file = ParamFile::parse(
            "ATC_ANG_PIT_P,4.5\n\
             ATC_ANG_RLL_P,banana\n\
             ATC_ANG_YAW_P\n\
             A_NAME_THAT_IS_FAR_TOO_LONG,1\n\
             ATC_RAT_PIT_P,0.135\n",
        );
        assert_eq!(file.len(), 2);
        assert_eq!(file.rejected().len(), 3);
        assert_eq!(file.rejected()[0].line, 2);
        assert_eq!(file.rejected()[0].reason, RejectReason::NotANumber);
        assert_eq!(file.rejected()[1].line, 3);
        assert_eq!(file.rejected()[1].reason, RejectReason::NoValue);
        assert_eq!(file.rejected()[2].line, 4);
        assert_eq!(file.rejected()[2].reason, RejectReason::NameTooLong);
    }

    /// A file that disagrees with itself says so.
    #[test]
    fn a_name_repeated_with_a_different_value_is_reported() {
        let file = ParamFile::parse("ATC_ANG_PIT_P,4.5\nATC_ANG_PIT_P,4.5\nATC_ANG_PIT_P,9.0\n");
        assert_eq!(file.get("ATC_ANG_PIT_P"), Some(9.0));
        assert_eq!(file.rejected().len(), 1);
        assert_eq!(file.rejected()[0].line, 3);
        assert_eq!(file.rejected()[0].reason, RejectReason::Duplicate);
    }

    /// Loading drops the vehicle's own bookkeeping, as Mission Planner's `loadParamFile` does.
    #[test]
    fn state_parameters_are_dropped_on_load() {
        // Every name on the C# list, in a file that also holds one real parameter. A file Mission
        // Planner saved looks exactly like this - `SaveParamFile` writes them - and both
        // implementations drop them on the way back in.
        let mut text = String::from("ATC_ANG_PIT_P,4.5\n");
        for name in NOT_LOADED {
            text.push_str(&format!("{name},1\n"));
        }
        let file = ParamFile::parse(&text);

        assert_eq!(file.len(), 1);
        assert_eq!(file.get("ATC_ANG_PIT_P"), Some(4.5));
        for skipped in NOT_LOADED {
            assert_eq!(file.get(skipped), None, "{skipped}");
        }
        // Skipping is not rejecting: the lines were well formed.
        assert!(file.rejected().is_empty(), "{:?}", file.rejected());
    }

    /// The list is the C# one, entry for entry.
    ///
    /// Asserted by length and contents rather than trusted, because the first version of this had
    /// seven of the sixteen and nothing noticed.
    #[test]
    fn the_skip_list_matches_the_c_sharp_source() {
        // ExtLibs/Utilities/ParamFile.cs:50-76, in order.
        const FROM_THE_CSHARP: &[&str] = &[
            "SYSID_SW_MREV",
            "WP_TOTAL",
            "CMD_TOTAL",
            "FENCE_TOTAL",
            "SYS_NUM_RESETS",
            "ARSPD_OFFSET",
            "GND_ABS_PRESS",
            "GND_TEMP",
            "BARO1_GND_PRESS",
            "BARO2_GND_PRESS",
            "BARO3_GND_PRESS",
            "BARO_GND_TEMP",
            "CMD_INDEX",
            "LOG_LASTFILE",
            "FORMAT_VERSION",
        ];
        assert_eq!(NOT_LOADED, FROM_THE_CSHARP);
    }

    /// A counter the vehicle accumulates is not configuration, and loading one rewinds its
    /// history. Found on a live vehicle: a file five minutes old proposed winding `STAT_RUNTIME`
    /// back by nearly five minutes.
    #[test]
    fn the_vehicles_own_statistics_are_dropped_on_load() {
        let file = ParamFile::parse(
            "ATC_ANG_PIT_P,4.5\n\
             STAT_RUNTIME,1301\n\
             STAT_BOOTCNT,47\n\
             STAT_FLTTIME,9000\n\
             STAT_FLTCNT,112\n\
             STAT_DISTFLWN,41000\n\
             STAT_RESET,1\n",
        );
        assert_eq!(file.len(), 1);
        assert_eq!(file.get("ATC_ANG_PIT_P"), Some(4.5));
    }

    /// The prefix rule must not catch a parameter that merely starts with the same letters.
    #[test]
    fn a_name_that_only_looks_like_a_statistic_is_kept() {
        let file = ParamFile::parse("STATE_OF_MIND,1\nSTAT,2\n");
        assert_eq!(file.len(), 2);
    }

    /// Saving keeps everything, because that is what `SaveParamFile` does.
    ///
    /// The filtering is on the load side. A file that omitted these would not be the file Mission
    /// Planner writes, and the point of the format is that both programs read each other's.
    #[test]
    fn saving_writes_everything_it_is_given() {
        let file = ParamFile::from_values([
            ("ATC_ANG_PIT_P", 4.5),
            ("WP_TOTAL", 12.0),
            ("STAT_RUNTIME", 1301.0),
        ]);
        assert_eq!(file.len(), 3);
        assert!(file.render().contains("WP_TOTAL,12"));
    }

    /// What is written can be read back as the same thing. This is the whole promise of a backup.
    #[test]
    fn a_saved_file_reads_back_identically() {
        let original = ParamFile::from_values([
            ("ATC_ANG_PIT_P", 4.5),
            ("ATC_RAT_PIT_P", 0.135),
            ("BATT_CAPACITY", 5200.0),
            ("COMPASS_OFS_X", -31.25),
            ("INS_GYROFFS_X", 0.001_234),
        ]);
        let reread = ParamFile::parse(&original.render());
        assert!(reread.rejected().is_empty());
        assert_eq!(original.len(), reread.len());
        for (name, value) in original.iter() {
            let back = reread
                .get(name)
                .expect("every name survives the round trip");
            assert!(same(value, back), "{name}: {value} != {back}");
        }
    }

    /// The number format is what `SaveParamFile` writes, including where it goes scientific.
    ///
    /// The boundary is the part worth pinning: .NET's `G15` switches to scientific at an exponent
    /// of -5, so 1e-4 is fixed and 1e-5 is not. An `{}` printer never switches, which is how an
    /// earlier version came to claim byte-identity it did not have.
    #[test]
    fn numbers_are_written_the_way_dotnet_writes_them() {
        // Whole numbers lose their point entirely - "1", not "1.0" and not "1.000000".
        assert_eq!(invariant_double(1.0), "1");
        assert_eq!(invariant_double(0.0), "0");
        assert_eq!(invariant_double(-0.0), "0");
        assert_eq!(invariant_double(360.0), "360");
        assert_eq!(invariant_double(110_000.0), "110000");
        // Fractions keep only the digits they need.
        assert_eq!(invariant_double(0.3), "0.3");
        assert_eq!(invariant_double(4.5), "4.5");
        assert_eq!(invariant_double(202.5), "202.5");
        assert_eq!(invariant_double(0.135), "0.135");
        assert_eq!(invariant_double(-31.25), "-31.25");
        assert_eq!(invariant_double(0.0036), "0.0036");
        // Just inside the fixed-point range.
        assert_eq!(invariant_double(0.0001), "0.0001");
        assert_eq!(invariant_double(0.000_567), "0.000567");
        // And just outside it, where the C# goes scientific and `{}` does not.
        assert_eq!(invariant_double(0.000_089), "8.9E-05");
        assert_eq!(invariant_double(0.000_01), "1E-05");
        assert_eq!(invariant_double(-0.000_089), "-8.9E-05");
    }

    /// Whatever the format, it has to read back as the same number.
    #[test]
    fn the_written_form_round_trips_through_the_parser() {
        for value in [
            0.0, 1.0, 0.3, -31.25, 0.000_089, 0.000_1, 1e-7, 123_456.75, -0.000_567, 110_000.0,
        ] {
            let text = invariant_double(value);
            let back: f64 = text
                .parse()
                .unwrap_or_else(|err| panic!("{text} did not parse: {err}"));
            assert!(
                same(value, back),
                "{value} wrote as {text} and read back as {back}"
            );
        }
    }

    /// A file Mission Planner wrote, read as it wrote it.
    #[test]
    fn the_c_sharp_applications_own_output_is_read() {
        // Format taken from a file saved by Mission Planner: a NOTE header, comma separated,
        // six decimal places, sorted.
        let file = ParamFile::parse(
            "#NOTE: Strip the \"#\" from the beginning of a line to activate it\n\
             ACRO_BAL_PITCH,1.000000\n\
             ACRO_BAL_ROLL,1.000000\n\
             ACRO_OPTIONS,0.000000\n\
             ACRO_RP_EXPO,0.300000\n",
        );
        assert_eq!(file.len(), 4);
        assert_eq!(file.get("ACRO_RP_EXPO"), Some(0.3));
        assert!(file.rejected().is_empty());
    }

    /// The direction of a comparison is the thing worth getting right.
    #[test]
    fn a_comparison_reads_as_what_would_change() {
        let current = ParamFile::from_values([
            ("ATC_ANG_PIT_P", 4.5),
            ("ATC_RAT_PIT_P", 0.135),
            ("ONLY_ON_VEHICLE", 1.0),
        ]);
        let proposed = ParamFile::from_values([
            ("ATC_ANG_PIT_P", 9.0),
            ("ATC_RAT_PIT_P", 0.135),
            ("ONLY_IN_FILE", 2.0),
        ]);
        let differences = current.compare(&proposed);
        assert_eq!(differences.len(), 3);
        assert_eq!(differences[0].name, "ATC_ANG_PIT_P");
        assert_eq!(differences[0].kind, Change::Changed { from: 4.5, to: 9.0 });
        assert_eq!(differences[1].name, "ONLY_IN_FILE");
        assert_eq!(differences[1].kind, Change::Added { to: 2.0 });
        assert_eq!(differences[2].name, "ONLY_ON_VEHICLE");
        assert_eq!(differences[2].kind, Change::Missing { from: 1.0 });
    }

    /// Float noise is not a change. Without this every comparison reports every parameter.
    #[test]
    fn a_value_that_survived_a_float_is_not_a_difference() {
        let current = ParamFile::from_values([("ATC_RAT_PIT_P", 0.135)]);
        // What 0.135 becomes after a trip through an f32, which is what the wire carries.
        let proposed = ParamFile::from_values([("ATC_RAT_PIT_P", f64::from(0.135_f32))]);
        assert!(
            current.compare(&proposed).is_empty(),
            "{:?}",
            current.compare(&proposed)
        );
    }

    /// A file written before the statistics group was skipped must not fill a comparison with
    /// counters that could never be applied.
    #[test]
    fn statistics_in_an_older_file_are_not_reported_as_differences() {
        let current = ParamFile::from_values([("ATC_ANG_PIT_P", 4.5)]);
        // parse rather than from_values, because that is how an older file arrives: unfiltered.
        let proposed = ParamFile::parse("ATC_ANG_PIT_P,4.5\nSTAT_RUNTIME,1022\nWP_TOTAL,12\n");
        assert!(
            current.compare(&proposed).is_empty(),
            "{:?}",
            current.compare(&proposed)
        );
    }

    /// A genuinely small change is still a change.
    #[test]
    fn a_real_difference_smaller_than_the_noise_floor_is_still_reported() {
        let current = ParamFile::from_values([("ATC_RAT_PIT_P", 0.135)]);
        let proposed = ParamFile::from_values([("ATC_RAT_PIT_P", 0.136)]);
        assert_eq!(current.compare(&proposed).len(), 1);
    }

    /// An empty file is a file with no parameters, not a parse failure.
    #[test]
    fn an_empty_file_is_empty_rather_than_broken() {
        let file = ParamFile::parse("");
        assert!(file.is_empty());
        assert!(file.rejected().is_empty());
        assert!(file.render().is_empty());
    }

    /// Lower-case names appear in hand-edited files and on forum posts.
    #[test]
    fn names_are_read_case_insensitively_and_stored_upper() {
        let file = ParamFile::parse("atc_ang_pit_p,4.5\n");
        assert_eq!(file.get("ATC_ANG_PIT_P"), Some(4.5));
    }

    /// Windows line endings, because half the files in circulation were written on Windows.
    #[test]
    fn carriage_returns_do_not_become_part_of_a_value() {
        let file = ParamFile::parse("ATC_ANG_PIT_P,4.5\r\nATC_RAT_PIT_P,0.135\r\n");
        assert_eq!(file.len(), 2);
        assert_eq!(file.get("ATC_RAT_PIT_P"), Some(0.135));
        assert!(file.rejected().is_empty(), "{:?}", file.rejected());
    }
}
