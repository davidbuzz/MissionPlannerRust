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

//! The porting ledger (PLAN.md §6.2): one row per C# file, and the commands that keep it honest.
//!
//! `ledger/ledger.csv` is the definition of "done" for goal G1. Some columns are *measured* from
//! the C# tree and rewritten by every refresh (`path`, `loc`, `namespace`, `csproj`, `tier`,
//! `size`, `sha256`); the rest are *hand-kept* by the people and agents doing the work, and a
//! refresh never touches them, except to send a row whose source changed upstream back to
//! `ready` with an `upstream-changed` note (the §6.2 staleness rule).
//!
//! Column conventions `check` relies on:
//! - `loc` is the file's newline count (`wc -l`), which is what PLAN.md §4's totals sum.
//! - `size` is the §4.4 bucket of `loc`: S < 200, M 200–599, L 600–1,500, XL > 1,500.
//! - `csproj` is the nearest `.csproj` in the file's directory or an ancestor; where one
//!   directory holds several, the first by name (`MissionPlanner.csproj`, not
//!   `MissionPlannerLib.csproj`, at the root).
//! - `target_crate` is a crate directory name (`mp-link`) or a repo-relative `.rs` path.
//! - `evidence` is `;`-separated repo-relative paths; for a `hand-port` row one of the `.rs`
//!   files it names is the port and carries the §6.4 `Ported from <path>` header.

mod classify;
/// Public for `tests/dead_csharp.rs`, which reads the ledger's D16 rows with this dialect.
pub mod csv;

use std::collections::{BTreeMap, BTreeSet};
use std::fmt::Write as _;
use std::path::{Path, PathBuf};

use anyhow::{Context, Result, bail};
use sha2::{Digest, Sha256};

pub use classify::{Tier, classify};

/// The columns of `ledger/ledger.csv`, in the order PLAN.md §6.2 lists them.
pub const COLUMNS: [&str; 20] = [
    "path",
    "loc",
    "namespace",
    "csproj",
    "tier",
    "disposition",
    "target_crate",
    "unit_id",
    "size",
    "fidelity_class",
    "perf_class",
    "verification_class",
    "deps",
    "state",
    "owner",
    "attempts",
    "evidence",
    "sha256",
    "omissions",
    "notes",
];

/// PLAN.md §6.2's states: the progression, then the two exits from it.
const STATES: [&str; 10] = [
    "blocked", "ready", "claimed", "ported", "tested", "verified", "reviewed", "done", "deferred",
    "dropped",
];

/// Past `ready`: someone has started, so a change to the C# invalidates what they have.
const STARTED: [&str; 6] = [
    "claimed", "ported", "tested", "verified", "reviewed", "done",
];

/// At or past `ported`: Rust exists, so the §6.4 omissions list is owed.
const PORTED: [&str; 5] = ["ported", "tested", "verified", "reviewed", "done"];

const DISPOSITIONS: [&str; 5] = [
    "delete",
    "regenerate",
    "extract",
    "hand-port",
    "drop-pending-owner",
];

/// PLAN.md §1.3.
const FIDELITY_CLASSES: [&str; 4] = [
    "bit-exact",
    "tolerance-bounded",
    "behaviour-equivalent",
    "redesigned",
];

/// PLAN.md §1.3.
const PERF_CLASSES: [&str; 3] = ["hot", "warm", "cold"];

/// PLAN.md §6.3.
const VERIFICATION_CLASSES: [&str; 3] = ["oracle", "golden", "spec+review"];

/// One ledger row. Every field is kept as text so that a malformed value is a problem `check`
/// reports, not a load failure that hides every other problem.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Row {
    /// Path relative to the C# root, `/`-separated.
    pub path: String,
    /// Newline count.
    pub loc: String,
    /// The first `namespace` the file declares.
    pub namespace: String,
    /// The nearest project file.
    pub csproj: String,
    /// T0–T4.
    pub tier: String,
    /// What happens to the file.
    pub disposition: String,
    /// Where the Rust lands.
    pub target_crate: String,
    /// The work unit this file belongs to.
    pub unit_id: String,
    /// S, M, L or XL.
    pub size: String,
    /// PLAN.md §1.3.
    pub fidelity_class: String,
    /// PLAN.md §1.3.
    pub perf_class: String,
    /// PLAN.md §6.3.
    pub verification_class: String,
    /// Units that must be `done` first.
    pub deps: String,
    /// Where the unit is in PLAN.md §6.2's progression.
    pub state: String,
    /// Who holds it.
    pub owner: String,
    /// How many times it has been dispatched.
    pub attempts: String,
    /// The artifacts that prove it.
    pub evidence: String,
    /// Hash of the C# the row was last measured against.
    pub sha256: String,
    /// What the port deliberately leaves out.
    pub omissions: String,
    /// Free text; the owner's reason for a `dropped` row lives here.
    pub notes: String,
}

impl Row {
    fn fields(&self) -> [&str; 20] {
        [
            &self.path,
            &self.loc,
            &self.namespace,
            &self.csproj,
            &self.tier,
            &self.disposition,
            &self.target_crate,
            &self.unit_id,
            &self.size,
            &self.fidelity_class,
            &self.perf_class,
            &self.verification_class,
            &self.deps,
            &self.state,
            &self.owner,
            &self.attempts,
            &self.evidence,
            &self.sha256,
            &self.omissions,
            &self.notes,
        ]
    }

    fn from_fields(fields: Vec<String>) -> Option<Row> {
        let [
            path,
            loc,
            namespace,
            csproj,
            tier,
            disposition,
            target_crate,
            unit_id,
            size,
            fidelity_class,
            perf_class,
            verification_class,
            deps,
            state,
            owner,
            attempts,
            evidence,
            sha256,
            omissions,
            notes,
        ]: [String; 20] = fields.try_into().ok()?;
        Some(Row {
            path,
            loc,
            namespace,
            csproj,
            tier,
            disposition,
            target_crate,
            unit_id,
            size,
            fidelity_class,
            perf_class,
            verification_class,
            deps,
            state,
            owner,
            attempts,
            evidence,
            sha256,
            omissions,
            notes,
        })
    }

    fn fresh(m: &Measured) -> Row {
        let mut row = Row {
            path: m.path.clone(),
            tier: m.tier.as_str().to_owned(),
            disposition: m.tier.disposition().to_owned(),
            // Nothing is done until it is done under contract (PLAN.md §5.2): existing Rust is
            // not credited by the refresh, only by a person moving the row.
            state: "ready".to_owned(),
            ..Row::default()
        };
        row.measure(m);
        row
    }

    fn measure(&mut self, m: &Measured) {
        self.loc = m.loc.to_string();
        self.namespace.clone_from(&m.namespace);
        self.csproj.clone_from(&m.csproj);
        self.size = size_bucket(m.loc).to_owned();
        self.sha256.clone_from(&m.sha256);
    }

    /// Whether anyone has written anything into this row that a refresh did not.
    fn has_hand_data(&self) -> bool {
        let hand = [
            &self.target_crate,
            &self.unit_id,
            &self.fidelity_class,
            &self.perf_class,
            &self.verification_class,
            &self.deps,
            &self.owner,
            &self.attempts,
            &self.evidence,
            &self.omissions,
            &self.notes,
        ];
        let default_disposition = Tier::parse(&self.tier).map(Tier::disposition);
        hand.iter().any(|f| !f.is_empty())
            || !matches!(self.state.as_str(), "ready" | "blocked")
            || default_disposition != Some(self.disposition.as_str())
    }
}

/// What the C# tree says about one file.
struct Measured {
    path: String,
    loc: usize,
    namespace: String,
    csproj: String,
    tier: Tier,
    sha256: String,
}

/// `cargo xtask ledger <init|refresh|check|status> [--tree DIR] [--ledger FILE] [--repo DIR]`.
pub fn run(args: &[String], repo: &Path) -> Result<()> {
    let Some((command, flags)) = args.split_first() else {
        usage();
        std::process::exit(2);
    };
    // `repo_root()` is `xtask/..`; resolve it so every path this prints is one a person can read.
    let repo = repo.canonicalize().unwrap_or_else(|_| repo.to_path_buf());
            // The C# tree `MP_SRC` names; without it (unset, or naming no directory), `check` reads the
    // CSV alone and `refresh` says what to set - the stand-in path is that sentence.
    let mut tree = crate::upstream::tree()
        .unwrap_or_else(|| PathBuf::from(format!("({})", crate::upstream::absent())));
    let mut ledger = repo.join("ledger/ledger.csv");
    let mut repo = repo;
    let mut flags = flags.iter();
    while let Some(flag) = flags.next() {
        let slot = match flag.as_str() {
            "--tree" => &mut tree,
            "--ledger" => &mut ledger,
            "--repo" => &mut repo,
            other => bail!("unknown option for `ledger {command}`: {other}"),
        };
        *slot = PathBuf::from(
            flags
                .next()
                .with_context(|| format!("{flag} needs a path"))?,
        );
    }
    match command.as_str() {
        // One command under two names: a first run creates the file, every later run merges
        // into it, so there is no way to overwrite hand-kept columns by picking the wrong verb.
        "init" | "refresh" => refresh(&tree, &ledger),
        "check" => check(&tree, &ledger, &repo),
        "status" => status(&ledger),
        other => {
            eprintln!("unknown ledger command: {other}\n");
            usage();
            std::process::exit(2);
        }
    }
}

fn usage() {
    println!(
        "cargo xtask ledger <command> [--tree DIR] [--ledger FILE] [--repo DIR]\n\n\
         commands:\n  \
         init | refresh  measure every .cs file into ledger/ledger.csv, keeping hand-kept columns\n  \
         check           fail, naming each offender, if the ledger is incomplete or inconsistent\n  \
         status          progress in retired C# lines per tier (PLAN.md §6.2)"
    );
}

/// Creates or updates the ledger from the C# tree.
fn refresh(tree: &Path, ledger: &Path) -> Result<()> {
    if !tree.is_dir() {
                bail!(
            "C# tree not found at {}\n{}",
            tree.display(),
            crate::upstream::absent()
        );
    }
    let measured = scan(tree)?;

    let mut old: BTreeMap<String, Row> = BTreeMap::new();
    if ledger.exists() {
        let Ledger { rows, problems } = read(ledger)?;
        // Rewriting a file we could not fully read would drop whatever we failed to parse.
        if let Some(problem) = problems.first() {
            bail!(
                "{}: {problem}; fix it by hand before refreshing",
                ledger.display()
            );
        }
        for (_, row) in rows {
            let path = row.path.clone();
            if old.insert(path.clone(), row).is_some() {
                bail!("two rows for {path}; merge them by hand before refreshing");
            }
        }
    }

    let (mut added, mut flagged) = (0usize, 0usize);
    let mut rows = Vec::with_capacity(measured.len());
    for m in &measured {
        let row = match old.remove(&m.path) {
            Some(mut row) => {
                if update(&mut row, m) {
                    flagged += 1;
                }
                row
            }
            None => {
                added += 1;
                Row::fresh(m)
            }
        };
        rows.push(row);
    }

    // A row whose file vanished upstream is deleted only if it holds nothing but measurements;
    // otherwise it stays, and `check` names it until a person decides where its work went.
    let mut kept = 0usize;
    let removed = old.len();
    for row in old.into_values() {
        if row.has_hand_data() {
            println!(
                "kept {}: the file is gone but the row carries work",
                row.path
            );
            kept += 1;
            rows.push(row);
        }
    }
    rows.sort_by(|a, b| a.path.cmp(&b.path));
    write(ledger, &rows)?;

    println!(
        "wrote {}: {} rows ({} new, {} flagged upstream-changed, {} removed, {} kept without a \
         file)",
        ledger.display(),
        thousands(rows.len()),
        thousands(added),
        thousands(flagged),
        thousands(removed - kept),
        thousands(kept),
    );
    Ok(())
}

/// Refreshes one existing row's measured columns. Returns whether the row was sent back to
/// `ready` because its source changed under work in progress.
fn update(row: &mut Row, m: &Measured) -> bool {
    let old_tier = Tier::parse(&row.tier);
    if old_tier != Some(m.tier) {
        // A disposition someone chose survives a reclassification; the default follows the tier.
        if old_tier.map(Tier::disposition) == Some(row.disposition.as_str())
            || row.disposition.is_empty()
        {
            row.disposition = m.tier.disposition().to_owned();
        }
        row.tier = m.tier.as_str().to_owned();
    }
    let mut flagged = false;
    if row.sha256 != m.sha256 && STARTED.contains(&row.state.as_str()) {
        let note = format!("upstream-changed: was {}", row.state);
        row.notes = if row.notes.is_empty() {
            note
        } else {
            format!("{}; {note}", row.notes)
        };
        row.state = "ready".to_owned();
        flagged = true;
    }
    row.measure(m);
    flagged
}

/// Validates the ledger, and against the C# tree when it is present.
fn check(tree: &Path, ledger: &Path, repo: &Path) -> Result<()> {
    let Ledger { rows, mut problems } = read(ledger)?;
    check_rows(&rows, repo, &mut problems);

    let tree_present = tree.is_dir();
    let mut files = 0usize;
    if tree_present {
        let measured = scan(tree)?;
        files = measured.len();
        check_tree(&rows, &measured, &mut problems);
    }

    for problem in &problems {
        println!("{problem}");
    }
    if !problems.is_empty() {
        bail!(
            "{} problem(s) in {}",
            thousands(problems.len()),
            ledger.display()
        );
    }
    if tree_present {
        println!(
            "ledger ok: {} rows, {} .cs files in {}",
            thousands(rows.len()),
            thousands(files),
            tree.display()
        );
    } else {
        println!(
            "ledger ok: {} rows. C# tree not found at {}, so only the CSV's internal \
             consistency was checked",
            thousands(rows.len()),
            tree.display()
        );
    }
    Ok(())
}

/// The checks that need nothing but the CSV and the Rust repository.
fn check_rows(rows: &[(usize, Row)], repo: &Path, problems: &mut Vec<String>) {
    let mut seen = BTreeSet::new();
    let mut previous: Option<&str> = None;
    let mut unsorted = 0usize;
    for (line, row) in rows {
        let mut bad = |what: String| problems.push(format!("{}: {what}", describe(*line, row)));
        let path = row.path.as_str();

        if path.is_empty() {
            bad("empty path".to_owned());
        }
        if !seen.insert(path) {
            bad("duplicate row for this path".to_owned());
        }
        if previous.is_some_and(|p| p > path) {
            // One report, not one per line, when a spreadsheet re-sorts the whole file.
            if unsorted == 0 {
                bad("rows are not sorted by path (a refresh sorts them)".to_owned());
            }
            unsorted += 1;
        }
        previous = Some(path);

        let tier = Tier::parse(&row.tier);
        if tier.is_none() {
            bad(format!("invalid tier `{}` (T0–T4)", row.tier));
        }
        if !DISPOSITIONS.contains(&row.disposition.as_str()) {
            bad(format!(
                "invalid disposition `{}` ({})",
                row.disposition,
                DISPOSITIONS.join(", ")
            ));
        }
        let state = row.state.as_str();
        if !STATES.contains(&state) {
            bad(format!("invalid state `{state}` ({})", STATES.join(", ")));
        }
        for (column, value, allowed) in [
            ("fidelity_class", &row.fidelity_class, &FIDELITY_CLASSES[..]),
            ("perf_class", &row.perf_class, &PERF_CLASSES[..]),
            (
                "verification_class",
                &row.verification_class,
                &VERIFICATION_CLASSES[..],
            ),
        ] {
            if !value.is_empty() && !allowed.contains(&value.as_str()) {
                bad(format!(
                    "invalid {column} `{value}` ({})",
                    allowed.join(", ")
                ));
            }
        }
        match row.loc.parse::<usize>() {
            Ok(loc) if row.size != size_bucket(loc) => bad(format!(
                "size `{}` does not match loc {loc} (expected {})",
                row.size,
                size_bucket(loc)
            )),
            Ok(_) => {}
            Err(_) => bad(format!("loc `{}` is not a line count", row.loc)),
        }
        if row.sha256.len() != 64
            || !row
                .sha256
                .bytes()
                .all(|b| b.is_ascii_digit() || (b'a'..=b'f').contains(&b))
        {
            bad(format!(
                "sha256 `{}` is not a lowercase hex digest",
                row.sha256
            ));
        }
        if !row.attempts.is_empty() && row.attempts.parse::<u32>().is_err() {
            bad(format!("attempts `{}` is not a count", row.attempts));
        }

        if state == "done" {
            check_done(row, repo, &mut bad);
        }
        if state == "dropped" && row.notes.trim().is_empty() {
            bad("dropped without the owner's reason in notes".to_owned());
        }
        // PLAN.md §6.4: "not ported: nothing" on a large file is almost always wrong.
        if PORTED.contains(&state) && row.size != "S" && row.omissions.trim().is_empty() {
            bad(format!(
                "{state} with size {} but no omissions (PLAN.md §6.4: owed from 200 lines up)",
                row.size
            ));
        }
    }
}

/// A `done` row must point at evidence that exists, and a hand-port must name its port.
fn check_done(row: &Row, repo: &Path, bad: &mut impl FnMut(String)) {
    let evidence: Vec<&str> = split_list(&row.evidence).collect();
    if evidence.is_empty() {
        bad("done without evidence".to_owned());
    }
    for entry in &evidence {
        if Path::new(entry).is_absolute() || !repo.join(entry).exists() {
            bad(format!(
                "evidence `{entry}` does not exist in the repository"
            ));
        }
    }
    let target = row.target_crate.as_str();
    let target_is_file = target.contains('/') || target.ends_with(".rs");
    if !target.is_empty() {
        let exists = if target_is_file {
            repo.join(target).is_file()
        } else {
            // `xtask` sits at the root; every other crate under `crates/`.
            repo.join("crates")
                .join(target)
                .join("Cargo.toml")
                .is_file()
                || repo.join(target).join("Cargo.toml").is_file()
        };
        if !exists {
            bad(format!("target_crate `{target}` does not exist"));
        }
    }
    if row.disposition == "hand-port" {
        let mut rust = evidence.iter().copied().filter(|e| e.ends_with(".rs"));
        let ported = rust.any(|file| has_provenance(&repo.join(file), &row.path))
            || (target_is_file && has_provenance(&repo.join(target), &row.path));
        if !ported {
            bad(format!(
                "done, but no Rust file in evidence carries the header `Ported from {}` \
                 (PLAN.md §6.4)",
                row.path
            ));
        }
    }
}

fn has_provenance(file: &Path, path: &str) -> bool {
    std::fs::read_to_string(file).is_ok_and(|text| {
        text.lines()
            .any(|l| l.contains("Ported from") && l.contains(path))
    })
}

/// The checks that compare the ledger with the C# tree.
fn check_tree(rows: &[(usize, Row)], measured: &[Measured], problems: &mut Vec<String>) {
    let mut by_path: BTreeMap<&str, &Row> = BTreeMap::new();
    for (_, row) in rows {
        by_path.entry(row.path.as_str()).or_insert(row);
    }
    for m in measured {
        let Some(row) = by_path.get(m.path.as_str()) else {
            problems.push(format!(
                "{}: no row; every .cs file needs one (cargo xtask ledger refresh)",
                m.path
            ));
            continue;
        };
        if row.sha256 != m.sha256 {
            problems.push(format!(
                "{}: sha256 is stale, the file changed upstream (cargo xtask ledger refresh)",
                m.path
            ));
        } else if row.loc != m.loc.to_string() {
            problems.push(format!(
                "{}: loc {} does not match the file's {}",
                m.path, row.loc, m.loc
            ));
        }
        if row.tier != m.tier.as_str() {
            problems.push(format!(
                "{}: tier {} but the classifier says {} (cargo xtask ledger refresh)",
                m.path,
                row.tier,
                m.tier.as_str()
            ));
        }
    }
    let files: BTreeSet<&str> = measured.iter().map(|m| m.path.as_str()).collect();
    for (line, row) in rows {
        if !files.contains(row.path.as_str()) {
            problems.push(format!(
                "{}: no such file in the C# tree",
                describe(*line, row)
            ));
        }
    }
}

/// Prints progress in retired C# lines, PLAN.md §6.2's headline first.
fn status(ledger: &Path) -> Result<()> {
    let Ledger { rows, problems } = read(ledger)?;
    if let Some(problem) = problems.first() {
        bail!(
            "{}: {problem}; run `cargo xtask ledger check`",
            ledger.display()
        );
    }

    #[derive(Clone, Copy, Default)]
    struct Sum {
        files: usize,
        loc: usize,
        done: usize,
        dropped: usize,
    }
    let mut sums: BTreeMap<Tier, Sum> = BTreeMap::new();
    let mut states: BTreeMap<&str, usize> = BTreeMap::new();
    for (_, row) in &rows {
        *states.entry(row.state.as_str()).or_default() += 1;
        let Some(tier) = Tier::parse(&row.tier) else {
            continue;
        };
        let loc = row.loc.parse::<usize>().unwrap_or(0);
        let sum = sums.entry(tier).or_default();
        sum.files += 1;
        sum.loc += loc;
        match row.state.as_str() {
            "done" => sum.done += loc,
            "dropped" => sum.dropped += loc,
            _ => {}
        }
    }
    let retired = |t: Tier| sums.get(&t).map_or(0, |s| s.done + s.dropped);
    let total = |t: Tier| sums.get(&t).map_or(0, |s| s.loc);

    // Both terminal states retire C#: G1 counts `dropped(reason)` alongside `done`.
    println!(
        "T0 {} retired / T1 {} generated / T2 {} extracted / T3 {} done of {}",
        thousands(retired(Tier::T0)),
        thousands(retired(Tier::T1)),
        thousands(retired(Tier::T2)),
        thousands(retired(Tier::T3)),
        thousands(total(Tier::T3)),
    );
    println!();
    println!(
        "{:<4} {:<18} {:>6} {:>10} {:>10} {:>10} {:>6}",
        "tier", "disposition", "files", "loc", "done", "dropped", "%"
    );
    let mut all = Sum::default();
    for tier in Tier::ALL {
        let s = sums.get(&tier).copied().unwrap_or_default();
        println!(
            "{:<4} {:<18} {:>6} {:>10} {:>10} {:>10} {:>6}",
            tier.as_str(),
            tier.disposition(),
            thousands(s.files),
            thousands(s.loc),
            thousands(s.done),
            thousands(s.dropped),
            percent(s.done + s.dropped, s.loc),
        );
        all.files += s.files;
        all.loc += s.loc;
        all.done += s.done;
        all.dropped += s.dropped;
    }
    println!(
        "{:<23} {:>6} {:>10} {:>10} {:>10} {:>6}",
        "total",
        thousands(all.files),
        thousands(all.loc),
        thousands(all.done),
        thousands(all.dropped),
        percent(all.done + all.dropped, all.loc),
    );
    let mut line = String::from("states:");
    for state in STATES {
        if let Some(n) = states.get(state) {
            let _ = write!(line, " {state} {}", thousands(*n));
        }
    }
    println!("{line}");
    Ok(())
}

/// A parsed ledger.
struct Ledger {
    /// Each row with the line it starts on, so problems point where a person would look.
    rows: Vec<(usize, Row)>,
    /// Records that could not be read as rows.
    problems: Vec<String>,
}

/// Reads the ledger into rows (with the line each starts on) and the record-level problems
/// found on the way. A wrong header is fatal: without it no column can be trusted.
fn read(ledger: &Path) -> Result<Ledger> {
    let text =
        std::fs::read_to_string(ledger).with_context(|| format!("reading {}", ledger.display()))?;
    let mut records = csv::parse(&text)
        .with_context(|| format!("parsing {}", ledger.display()))?
        .into_iter();
    let header = records.next().map(|r| r.fields).unwrap_or_default();
    if header != COLUMNS {
        bail!(
            "{}: the header must be exactly `{}`",
            ledger.display(),
            COLUMNS.join(",")
        );
    }
    let mut rows = Vec::new();
    let mut problems = Vec::new();
    for record in records {
        let count = record.fields.len();
        match Row::from_fields(record.fields) {
            Some(row) => rows.push((record.line, row)),
            None => problems.push(format!(
                "line {}: {count} fields, expected {}",
                record.line,
                COLUMNS.len()
            )),
        }
    }
    Ok(Ledger { rows, problems })
}

/// Writes the ledger through a temporary file, so an interrupted refresh cannot leave half a
/// ledger behind.
fn write(ledger: &Path, rows: &[Row]) -> Result<()> {
    let mut out = String::new();
    csv::write_record(&mut out, COLUMNS);
    for row in rows {
        csv::write_record(&mut out, row.fields());
    }
    if let Some(dir) = ledger.parent() {
        std::fs::create_dir_all(dir).with_context(|| format!("creating {}", dir.display()))?;
    }
    let partial = ledger.with_extension("csv.partial");
    std::fs::write(&partial, out).with_context(|| format!("writing {}", partial.display()))?;
    std::fs::rename(&partial, ledger).with_context(|| format!("replacing {}", ledger.display()))?;
    Ok(())
}

/// Measures every `.cs` file under `tree`, sorted by path.
fn scan(tree: &Path) -> Result<Vec<Measured>> {
    let mut sources = Vec::new();
    let mut projects = BTreeMap::new();
    walk(tree, "", &mut sources, &mut projects)?;
    sources.sort();
    sources
        .into_iter()
        .map(|path| {
            let bytes = std::fs::read(tree.join(&path))
                .with_context(|| format!("reading {}", tree.join(&path).display()))?;
            Ok(Measured {
                loc: bytes.iter().filter(|&&b| b == b'\n').count(),
                namespace: first_namespace(&String::from_utf8_lossy(&bytes)),
                csproj: nearest_csproj(&path, &projects),
                tier: classify(&path, &bytes),
                sha256: format!("{:x}", Sha256::digest(&bytes)),
                path,
            })
        })
        .collect()
}

/// Collects `.cs` paths and, per directory, the first `.csproj` by name.
fn walk(
    dir: &Path,
    rel: &str,
    sources: &mut Vec<String>,
    projects: &mut BTreeMap<String, String>,
) -> Result<()> {
    let entries = std::fs::read_dir(dir).with_context(|| format!("listing {}", dir.display()))?;
    for entry in entries {
        let entry = entry.with_context(|| format!("listing {}", dir.display()))?;
        let name = entry.file_name().to_string_lossy().into_owned();
        let child = if rel.is_empty() {
            name.clone()
        } else {
            format!("{rel}/{name}")
        };
        // Symlinks are not followed: the reference tree has none, and following one could loop
        // or count a file twice.
        let kind = entry.file_type()?;
        if kind.is_dir() {
            if name != ".git" {
                walk(&entry.path(), &child, sources, projects)?;
            }
        } else if kind.is_file() {
            // `find -name '*.cs'`, the measurement PLAN.md §4 was made with, is case-sensitive.
            if name.ends_with(".cs") {
                sources.push(child);
            } else if name.ends_with(".csproj") {
                let first = projects
                    .entry(rel.to_owned())
                    .or_insert_with(|| name.clone());
                if name < *first {
                    *first = name;
                }
            }
        }
    }
    Ok(())
}

fn nearest_csproj(path: &str, projects: &BTreeMap<String, String>) -> String {
    let mut dir = parent(path);
    loop {
        if let Some(name) = projects.get(dir) {
            return if dir.is_empty() {
                name.clone()
            } else {
                format!("{dir}/{name}")
            };
        }
        if dir.is_empty() {
            return String::new();
        }
        dir = parent(dir);
    }
}

fn parent(path: &str) -> &str {
    path.rsplit_once('/').map_or("", |(dir, _)| dir)
}

/// The first `namespace` declaration, block-scoped or file-scoped, outside block comments.
fn first_namespace(text: &str) -> String {
    let mut in_comment = false;
    for line in text.lines() {
        let line = line.trim_start_matches('\u{feff}').trim();
        if in_comment {
            in_comment = !line.contains("*/");
            continue;
        }
        if line.starts_with("/*") {
            in_comment = !line.contains("*/");
            continue;
        }
        let Some(rest) = line.strip_prefix("namespace") else {
            continue;
        };
        if !rest.starts_with(char::is_whitespace) {
            continue;
        }
        let name: String = rest
            .trim_start()
            .chars()
            .take_while(|c| !c.is_whitespace() && !matches!(c, '{' | ';' | '/'))
            .collect();
        if !name.is_empty() {
            return name;
        }
    }
    String::new()
}

/// PLAN.md §4.4's buckets.
fn size_bucket(loc: usize) -> &'static str {
    match loc {
        0..200 => "S",
        200..600 => "M",
        600..=1500 => "L",
        _ => "XL",
    }
}

fn split_list(list: &str) -> impl Iterator<Item = &str> {
    list.split(';').map(str::trim).filter(|e| !e.is_empty())
}

fn describe(line: usize, row: &Row) -> String {
    if row.path.is_empty() {
        format!("line {line}")
    } else {
        row.path.clone()
    }
}

fn thousands(n: usize) -> String {
    let digits = n.to_string();
    let mut out = String::with_capacity(digits.len() + digits.len() / 3);
    for (i, c) in digits.chars().enumerate() {
        if i > 0 && (digits.len() - i).is_multiple_of(3) {
            out.push(',');
        }
        out.push(c);
    }
    out
}

fn percent(part: usize, whole: usize) -> String {
    if whole == 0 {
        return "-".to_owned();
    }
    // Integer arithmetic, so the report cannot differ between machines in the last digit.
    let tenths = part * 1000 / whole;
    format!("{}.{}%", tenths / 10, tenths % 10)
}
