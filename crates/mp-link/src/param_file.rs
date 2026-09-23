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

use crate::params::round_to_significant_digits;
use std::collections::BTreeMap;

/// Significant digits used when comparing, matching Mission Planner's comparison behaviour.
///
/// A parameter round-trips through a 32-bit float on the wire and through decimal text in the
/// file, and neither is exact. Comparing the raw doubles reports every parameter on the vehicle
/// as different from the file it was just written from, which makes the comparison useless at
/// precisely the moment it is wanted.
const COMPARE_DIGITS: i32 = 7;

/// Parameters Mission Planner omits when it saves, and why.
///
/// From `ExtLibs/Utilities/ParamFile.cs`. Every one is state rather than configuration - a count
/// the vehicle maintains, a sensor offset it measures at boot, a statistic it accumulates.
/// Writing them into another airframe is at best meaningless and at worst harmful: `ARSPD_OFFSET`
/// and `GND_ABS_PRESS` are calibration readings taken on the day, and loading yesterday's onto a
/// vehicle sitting at a different pressure gives it a wrong idea of its own altitude.
///
/// `WP_TOTAL`, `CMD_TOTAL` and `FENCE_TOTAL` are worse than meaningless: they say how many mission
/// items the vehicle has, and setting them without writing the items claims a mission that is not
/// there.
pub const NOT_SAVED: &[&str] = &[
    "SYSID_SW_MREV",
    "WP_TOTAL",
    "CMD_TOTAL",
    "FENCE_TOTAL",
    "SYS_NUM_RESETS",
    "ARSPD_OFFSET",
    "GND_ABS_PRESS",
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
const NOT_SAVED_PREFIXES: &[&str] = &["STAT_"];

/// Whether a parameter is one to write into a saved file.
#[must_use]
pub fn is_saved(name: &str) -> bool {
    !NOT_SAVED.contains(&name)
        && !NOT_SAVED_PREFIXES
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

    /// Builds one from name/value pairs, dropping the parameters Mission Planner does not save.
    pub fn from_values<I, S>(values: I) -> Self
    where
        I: IntoIterator<Item = (S, f64)>,
        S: Into<String>,
    {
        let mut file = Self::new();
        for (name, value) in values {
            let name = name.into();
            if is_saved(&name) {
                file.values.insert(name, value);
            }
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
    /// `NAME,VALUE` with six decimal places, sorted by name. Six places because that is what the
    /// C# application writes and what makes two saved files diff cleanly against each other; a
    /// shortest-representation printer would write `1` here and `1.0` there depending on how the
    /// value arrived, and turn an unchanged parameter into a diff line.
    #[must_use]
    pub fn render(&self) -> String {
        let mut out = String::new();
        for (name, value) in &self.values {
            out.push_str(&format!("{name},{value:.6}\n"));
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
        std::fs::write(&temporary, self.render())?;
        std::fs::rename(&temporary, path)
    }

    /// Reads a file from disk.
    ///
    /// # Errors
    /// If the file cannot be read. A file that parses badly is not an error - see `rejected`.
    pub fn load(path: &std::path::Path) -> std::io::Result<Self> {
        Ok(Self::parse(&std::fs::read_to_string(path)?))
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
            if !is_saved(name) {
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
            if !is_saved(name) {
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

    /// Saving drops the vehicle's own bookkeeping, as Mission Planner does.
    #[test]
    fn state_parameters_are_not_saved() {
        let file = ParamFile::from_values([
            ("ATC_ANG_PIT_P", 4.5),
            ("WP_TOTAL", 12.0),
            ("GND_ABS_PRESS", 101_325.0),
            ("SYS_NUM_RESETS", 7.0),
            ("ARSPD_OFFSET", 1.234),
            ("CMD_TOTAL", 12.0),
            ("FENCE_TOTAL", 5.0),
            ("SYSID_SW_MREV", 120.0),
        ]);
        assert_eq!(file.len(), 1);
        assert_eq!(file.get("ATC_ANG_PIT_P"), Some(4.5));
        for skipped in NOT_SAVED {
            assert_eq!(file.get(skipped), None, "{skipped}");
        }
    }

    /// A counter the vehicle accumulates is not configuration, and loading one rewinds its
    /// history. Found on a live vehicle: a file five minutes old proposed winding `STAT_RUNTIME`
    /// back by nearly five minutes.
    #[test]
    fn the_vehicles_own_statistics_are_not_saved() {
        let file = ParamFile::from_values([
            ("ATC_ANG_PIT_P", 4.5),
            ("STAT_RUNTIME", 1301.0),
            ("STAT_BOOTCNT", 47.0),
            ("STAT_FLTTIME", 9_000.0),
            ("STAT_FLTCNT", 112.0),
            ("STAT_DISTFLWN", 41_000.0),
            ("STAT_RESET", 1.0),
        ]);
        assert_eq!(file.len(), 1);
        assert_eq!(file.get("ATC_ANG_PIT_P"), Some(4.5));
    }

    /// The prefix rule must not catch a parameter that merely starts with the same letters.
    #[test]
    fn a_name_that_only_looks_like_a_statistic_is_saved() {
        let file = ParamFile::from_values([("STATE_OF_MIND", 1.0), ("STAT", 2.0)]);
        assert_eq!(file.len(), 2);
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
