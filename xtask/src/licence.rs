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

//! The licence of the work and the record of what it carries (DELIVERABLES.md D1): the header
//! every tracked Rust file opens with, the identifier the manifests declare, the crates the
//! workspace is built from - the table `cargo xtask licences` keeps in `THIRD_PARTY_LICENSES` from
//! `cargo metadata` - and whether each crate's licence is one deny.toml admits, its SPDX
//! expression read as cargo-deny reads it.
//!
//! The work is GPL-3.0-only. Mission Planner's COPYING.txt is the GNU General Public License
//! version 3 and nothing in its tree grants a later version, so a derivative of it cannot either;
//! the owner's decision of 2026-10-03, with the header's form and names.

use std::collections::HashSet;
use std::path::{Path, PathBuf};
use std::process::Command;

use anyhow::{Context, Result, anyhow, bail};

/// The header every tracked `.rs` file opens with, followed by a blank line: the owner's form -
/// his copyright, the product's name, the derivation with Mission Planner's copyright, the GNU
/// notice for version 3 alone, and the SPDX identifier.
pub const HEADER: &str = "\
// Copyright (C) 2026 David \"Buzz\" Bussenschutt\n\
//\n\
// This file is part of MissionPlannerRust, a Rust implementation derived from\n\
// Mission Planner (Copyright (C) 2010-2024 Michael Oborne and contributors,\n\
// https://github.com/ArduPilot/MissionPlanner); NOTICE records the changes.\n\
//\n\
// MissionPlannerRust is free software: you can redistribute it and/or modify\n\
// it under the terms of the GNU General Public License as published by the\n\
// Free Software Foundation, version 3 of the License.\n\
//\n\
// MissionPlannerRust is distributed in the hope that it will be useful, but\n\
// WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY\n\
// or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for\n\
// more details.\n\
//\n\
// You should have received a copy of the GNU General Public License along with\n\
// MissionPlannerRust. If not, see <https://www.gnu.org/licenses/>.\n\
//\n\
// SPDX-License-Identifier: GPL-3.0-only\n";

/// The SPDX identifier of the work, as every manifest declares it.
pub const LICENCE: &str = "GPL-3.0-only";

/// The record of third-party material, at the repository's root.
pub const THIRD_PARTY: &str = "THIRD_PARTY_LICENSES";

/// The line that opens the crate table in [`THIRD_PARTY`].
pub const TABLE_BEGIN: &str =
    "<!-- BEGIN crates: written by `cargo xtask licences` from `cargo metadata`; not edited by hand -->";

/// The line that closes it.
pub const TABLE_END: &str = "<!-- END crates -->";

/// Every `.rs` file git tracks under `root`, as git lists them.
pub fn rust_files(root: &Path) -> Result<Vec<PathBuf>> {
    let output = Command::new("git")
        .args(["ls-files", "-z", "--", "*.rs"])
        .current_dir(root)
        .output()
        .context("running git ls-files")?;
    if !output.status.success() {
        bail!(
            "git ls-files failed: {}",
            String::from_utf8_lossy(&output.stderr)
        );
    }
    Ok(output
        .stdout
        .split(|&byte| byte == 0)
        .filter(|path| !path.is_empty())
        .map(|path| root.join(String::from_utf8_lossy(path).as_ref()))
        .collect())
}

/// Whether a source text opens with [`HEADER`] and the blank line after it. A header anywhere
/// else - under a doc comment, say - is not the file's header.
#[must_use]
pub fn has_header(text: &str) -> bool {
    text.strip_prefix(HEADER)
        .is_some_and(|rest| rest.starts_with('\n'))
}

/// The tracked `.rs` files that do not open with the header.
pub fn without_header(root: &Path) -> Result<Vec<PathBuf>> {
    let mut missing = Vec::new();
    for file in rust_files(root)? {
        let text = std::fs::read_to_string(&file)
            .with_context(|| format!("reading {}", file.display()))?;
        if !has_header(&text) {
            missing.push(file);
        }
    }
    Ok(missing)
}

/// One crate the workspace is built from.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub struct Dependency {
    /// Its name.
    pub name: String,
    /// Its version.
    pub version: String,
    /// Its licence: the manifest's SPDX expression, or [`CLARIFIED`]'s reading of the licence
    /// file a manifest names instead, or `file:<path>` where neither says.
    pub licence: String,
    /// Where it comes from: `crates.io`, a git URL and commit, or `path`.
    pub source: String,
}

/// Crates whose manifest names a licence file and no SPDX expression, and what the file is.
pub const CLARIFIED: [(&str, &str); 4] = [
    // LICENSE: the Mersenne Twister's three-clause BSD terms, Matsumoto and Nishimura 1997-2002.
    ("mt19937", "BSD-3-Clause"),
    // LICENSE: the Python Software Foundation licence.
    ("rustpython-doc", "PSF-2.0"),
    // Lib/PSF-LICENSE: the Python Software Foundation licence, for the standard library it carries.
    ("rustpython-pylib", "PSF-2.0"),
    // LICENSE: the two-clause BSD licence.
    ("syn-ext", "BSD-2-Clause"),
];

/// Every crate the workspace depends on, for every target, from `cargo metadata` over the lock
/// file; the workspace's own crates left out; sorted by name and version.
pub fn dependencies(root: &Path) -> Result<Vec<Dependency>> {
    let output = Command::new("cargo")
        .args(["metadata", "--format-version", "1", "--locked"])
        .current_dir(root)
        .output()
        .context("running cargo metadata")?;
    if !output.status.success() {
        bail!(
            "cargo metadata failed: {}",
            String::from_utf8_lossy(&output.stderr)
        );
    }
    let metadata: serde_json::Value =
        serde_json::from_slice(&output.stdout).context("reading cargo metadata's JSON")?;
    let members: HashSet<&str> = metadata
        .get("workspace_members")
        .and_then(serde_json::Value::as_array)
        .map(|members| {
            members
                .iter()
                .filter_map(serde_json::Value::as_str)
                .collect()
        })
        .unwrap_or_default();
    let mut dependencies = Vec::new();
    for package in metadata
        .get("packages")
        .and_then(serde_json::Value::as_array)
        .into_iter()
        .flatten()
    {
        if members.contains(text(package, "id").unwrap_or_default()) {
            continue;
        }
        let name = text(package, "name").unwrap_or_default().to_owned();
        let licence = match text(package, "license") {
            Some(expression) if !expression.is_empty() => expression.to_owned(),
            _ => CLARIFIED
                .iter()
                .find(|(crate_name, _)| *crate_name == name)
                .map_or_else(
                    || format!("file:{}", text(package, "license_file").unwrap_or("(none)")),
                    |(_, licence)| (*licence).to_owned(),
                ),
        };
        dependencies.push(Dependency {
            name,
            version: text(package, "version").unwrap_or_default().to_owned(),
            licence,
            source: source_label(text(package, "source")),
        });
    }
    dependencies.sort();
    Ok(dependencies)
}

/// A string field of a JSON object, if it has one.
fn text<'value>(value: &'value serde_json::Value, key: &str) -> Option<&'value str> {
    value.get(key).and_then(serde_json::Value::as_str)
}

/// A package's source as the table shows it.
fn source_label(source: Option<&str>) -> String {
    let Some(source) = source else {
        return "path".to_owned();
    };
    if source.starts_with("registry+https://github.com/rust-lang/crates.io-index") {
        return "crates.io".to_owned();
    }
    if let Some(git) = source.strip_prefix("git+") {
        let (locator, commit) = git.split_once('#').unwrap_or((git, ""));
        let url = locator.split_once('?').map_or(locator, |(url, _)| url);
        let commit = commit.get(..7).unwrap_or(commit);
        return if commit.is_empty() {
            format!("git {url}")
        } else {
            format!("git {url} @ {commit}")
        };
    }
    source.to_owned()
}

/// The crates as a Markdown table.
#[must_use]
pub fn table(dependencies: &[Dependency]) -> String {
    let mut out = String::from("| Crate | Version | Licence | Source |\n|---|---|---|---|\n");
    for dependency in dependencies {
        out.push_str(&format!(
            "| {} | {} | {} | {} |\n",
            dependency.name, dependency.version, dependency.licence, dependency.source
        ));
    }
    out
}

/// The crate table [`THIRD_PARTY`] should hold today.
fn fresh_table(dependencies: &[Dependency]) -> String {
    format!(
        "{TABLE_BEGIN}\n{} crates, every target's, from `cargo metadata` over Cargo.lock; each \
         licence is one `deny.toml` admits (`xtask/tests/licences.rs` holds both).\n\n{}{TABLE_END}",
        dependencies.len(),
        table(dependencies)
    )
}

/// Brings the crate table in [`THIRD_PARTY`] up to date with `cargo metadata`, between its two
/// marker lines, unless `check_only`; `true` when it already was.
pub fn update_third_party(root: &Path, check_only: bool) -> Result<bool> {
    let path = root.join(THIRD_PARTY);
    let text =
        std::fs::read_to_string(&path).with_context(|| format!("reading {}", path.display()))?;
    let begin = text
        .find(TABLE_BEGIN)
        .ok_or_else(|| anyhow!("{THIRD_PARTY} has no `{TABLE_BEGIN}` line"))?;
    let end = text
        .find(TABLE_END)
        .ok_or_else(|| anyhow!("{THIRD_PARTY} has no `{TABLE_END}` line"))?;
    if end < begin {
        bail!("{THIRD_PARTY}'s crate table ends before it begins");
    }
    let end = end + TABLE_END.len();
    let fresh = fresh_table(&dependencies(root)?);
    if text[begin..end] == fresh {
        return Ok(true);
    }
    if !check_only {
        let mut out = String::with_capacity(text.len() + fresh.len());
        out.push_str(&text[..begin]);
        out.push_str(&fresh);
        out.push_str(&text[end..]);
        std::fs::write(&path, out).with_context(|| format!("writing {}", path.display()))?;
    }
    Ok(false)
}

/// The licences `deny.toml` admits: the strings of its `allow = [ ... ]` list.
pub fn deny_allow_list(root: &Path) -> Result<Vec<String>> {
    let path = root.join("deny.toml");
    let text =
        std::fs::read_to_string(&path).with_context(|| format!("reading {}", path.display()))?;
    let start = text
        .find("allow = [")
        .ok_or_else(|| anyhow!("deny.toml has no `allow = [` list"))?;
    let list = &text[start..];
    let close = list
        .find(']')
        .ok_or_else(|| anyhow!("deny.toml's allow list does not close"))?;
    Ok(list[..close]
        .split('"')
        .skip(1)
        .step_by(2)
        .map(str::to_owned)
        .collect())
}

/// Whether an SPDX licence expression is satisfied by the licences in `allow`, read as cargo-deny
/// reads it: `OR` offers a choice, `AND` needs both, parentheses group, `WITH` binds an exception
/// to its licence (and the pair must be allowed as a pair), and the old `/` means `OR`.
#[must_use]
pub fn allowed(expression: &str, allow: &[String]) -> bool {
    let spaced = expression
        .replace('/', " OR ")
        .replace('(', " ( ")
        .replace(')', " ) ");
    let tokens: Vec<&str> = spaced.split_whitespace().collect();
    if tokens.is_empty() {
        return false;
    }
        let mut parser = Parser {
        tokens: &tokens,
        at: 0,
        allow,
        malformed: false,
    };
    let satisfied = parser.any_of();
    satisfied && !parser.malformed && parser.at == tokens.len()
}

/// A recursive-descent reader of one SPDX expression.
struct Parser<'a> {
    tokens: &'a [&'a str],
    at: usize,
    allow: &'a [String],
    /// Set where a licence was expected and the text ended or gave an operator instead: such
    /// an expression admits nothing, whatever its other terms say.
    malformed: bool,
}


impl<'a> Parser<'a> {
    fn peek(&self) -> Option<&'a str> {
        self.tokens.get(self.at).copied()
    }

    fn take(&mut self) -> Option<&'a str> {
        let token = self.peek();
        if token.is_some() {
            self.at += 1;
        }
        token
    }

    /// `a OR b OR ...`: one allowed alternative is enough.
    fn any_of(&mut self) -> bool {
        let mut satisfied = self.all_of();
        while self.peek() == Some("OR") {
            self.at += 1;
            let next = self.all_of();
            satisfied = satisfied || next;
        }
        satisfied
    }

    /// `a AND b AND ...`: every part must be allowed.
    fn all_of(&mut self) -> bool {
        let mut satisfied = self.term();
        while self.peek() == Some("AND") {
            self.at += 1;
            let next = self.term();
            satisfied = satisfied && next;
        }
        satisfied
    }

    /// A parenthesised expression, or a licence with its optional `WITH` exception.
    fn term(&mut self) -> bool {
        match self.take() {
                        Some("(") => {
                let inner = self.any_of();
                if self.take() != Some(")") {
                    self.malformed = true;
                }
                inner
            }
            Some(licence) if !matches!(licence, ")" | "AND" | "OR" | "WITH") => {
                let mut term = licence.to_owned();
                if self.peek() == Some("WITH") {
                    self.at += 1;
                                        match self.take() {
                        Some(exception) if !matches!(exception, "(" | ")" | "AND" | "OR") => {
                            term = format!("{term} WITH {exception}");
                        }
                        _ => {
                            self.malformed = true;
                            return false;
                        }
                    }
                }
                self.allow.contains(&term)
            }
            _ => {
                self.malformed = true;
                false
            }
        }
    }
}
