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

//! `.resx` → Fluent `.ftl`: DELIVERABLES.md Deliverable 17, PLAN.md §6.1's `resx2ftl`, §13.6 row 76.
//!
//! Mission Planner's strings live in `.resx` files: `ExtLibs/Strings/Strings.resx` for the texts
//! the code shows (`Strings.ErrorNoResponse`), and one `Designer.cs`-side `.resx` per screen for
//! every control's `Text`, `ToolTipText`, `HeaderText` and combo `Items`, beside that control's
//! `Size`, `Location` and the rest of its layout. A culture's translation is a sibling file,
//! `Strings.zh-Hans.resx`, holding only the entries it translates; .NET falls back to the base
//! file for the rest. `L10N.cs` picks the culture from `config.xml`'s `language`.
//!
//! This turns each base file and its siblings into one `.ftl` per culture under `assets/i18n/`,
//! and writes two things beside them:
//!
//! - `keymap.toml`, the `.resx` name → Fluent id of every key ever generated. It is read before
//!   generating and an id that would change is an error, not a rename: Crowdin's translation
//!   memory and every committed `.ftl` are keyed on these ids (PLAN.md R14).
//! - `report.md`, the zero-loss report Deliverable 17 asks for: per culture, how many entries the file had,
//!   how many were language, how many translate a base key, how many are orphans - translations
//!   of a name the base file no longer has, kept in the `.ftl` under an `# orphan` comment rather
//!   than dropped - and how many base keys the culture leaves untranslated. Nothing a translator
//!   wrote is lost: every string entry of every culture file is in its `.ftl`, and the report
//!   says where.
//!
//! **What is a language key.** In `Strings.resx` every string entry. In a screen's `.resx` a name
//! ending in `.Text`, `.ToolTipText`, `.HeaderText` or `.Items`/`.ItemsN` - the four suffixes
//! that carry words across the whole tree (18,338, 169, 524 and 2,018 entries respectively);
//! `.Size`, `.Location`, `.Font` and the `>>` metadata are layout, and stay where they are.
//! Entries with a `type` or `mimetype` are not strings at all.
//!
//! **What an id is.** `Strings.resx` names are Fluent ids as they are (`ErrorNoResponse`); a
//! screen's are the file's stem and the name with its dots made dashes (`flightdata-BUT_ARM-Text`),
//! which keeps the control's name and property readable in the `.ftl` and unique.
//!
//! **What a value becomes.** Exactly the .NET string, so that a message shown through Fluent is
//! the message the C# showed: `{0}` and `{0:F2}` become `{ $arg0 }` (the format is lost, and the
//! report says so), any other brace is a `{"{"}` literal, leading and trailing spaces on a line
//! and lines that begin with `.`, `[`, `*` or `}` go into string literals Fluent will not trim or
//! misread, a line break is a continuation line, and an empty line or value is `{""}`. `\r\n`
//! becomes `\n`. Every generated file is parsed back before it is written, and a value that does
//! not survive the round trip is an error here, not a surprise on screen.
//!
//! **What a screen asks for.** [`SCREENS`] names the application's screens whose words go through
//! Fluent, by their source files; every `fl!("<id>")` in them is a key that screen asks for
//! ([`keys_in`]). The report ends with each screen's table: per culture, how many of its keys the
//! culture's own files lack - English lacking one is a bug `mp-gui`'s lint fails on, any other
//! culture lacking one is shown in English, as it is in the C#. `mp-gui`'s tests hold their
//! count to this table, so neither can drift from the other.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt::Write as _;
use std::path::{Path, PathBuf};

use anyhow::{Context, Result, bail};

/// The base files converted, relative to the reference tree, with the stem their `.ftl` files
/// take. `Strings.resx` is the application's texts; `FlightData.resx` the first screen.
pub const BASES: &[(&str, &str)] = &[
    ("ExtLibs/Strings/Strings.resx", "strings"),
    ("GCSViews/FlightData.resx", "flightdata"),
];

/// The culture the base file is written in.
pub const BASE_CULTURE: &str = "en";

/// A screen whose words go through Fluent: its name, and its source files relative to the
/// repository, which `mp-gui`'s `i18n::SCREEN_SOURCES` names too.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Screen {
    /// The name the report gives it.
    pub name: &'static str,
    /// Its source files.
    pub sources: &'static [&'static str],
}

/// The screens through Fluent so far: the flight screen, whose tab pages and buttons read their
/// words from `FlightData.resx` (PLAN.md §13.6 row 76).
pub const SCREENS: &[Screen] = &[Screen {
    name: "Flight Data",
    sources: &[
        "crates/mp-gui/src/fly.rs",
        "crates/mp-gui/src/payload.rs",
        "crates/mp-gui/src/transponder.rs",
    ],
}];

/// The keys one screen asks for.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct ScreenKeys {
    /// The screen's name.
    pub name: String,
    /// Its source files, as [`Screen::sources`] names them.
    pub sources: Vec<String>,
    /// Every key its sources ask for with `fl!`, in the order found, once each.
    pub keys: Vec<String>,
}

/// The keys a source file asks for with `fl!("<id>"...)`, in order, once each - however the call
/// is wrapped, and with or without a path in front of the macro.
#[must_use]
pub fn keys_in(source: &str) -> Vec<String> {
    let mut found: Vec<String> = Vec::new();
    let mut rest = source;
    while let Some(at) = rest.find("fl!(") {
        rest = rest.get(at + "fl!(".len()..).unwrap_or_default();
        let Some(quoted) = rest.trim_start().strip_prefix('"') else {
            continue;
        };
        if let Some((key, _)) = quoted.split_once('"')
            && !found.iter().any(|k| k == key)
        {
            found.push(key.to_owned());
        }
    }
    found
}

/// The keys each of [`SCREENS`] asks for, read from the repository at `repo`.
pub fn screen_keys(repo: &Path) -> Result<Vec<ScreenKeys>> {
    let mut screens = Vec::new();
    for screen in SCREENS {
        let mut keys: Vec<String> = Vec::new();
        for source in screen.sources {
            let path = repo.join(source);
            let text = std::fs::read_to_string(&path)
                .with_context(|| format!("reading {}", path.display()))?;
            for key in keys_in(&text) {
                if !keys.contains(&key) {
                    keys.push(key);
                }
            }
        }
        screens.push(ScreenKeys {
            name: screen.name.to_owned(),
            sources: screen.sources.iter().map(|&s| s.to_owned()).collect(),
            keys,
        });
    }
    Ok(screens)
}

/// The message ids a generated `.ftl` defines.
fn message_ids(text: &str) -> BTreeSet<String> {
    let resource = match fluent_syntax::parser::parse(text) {
        Ok(resource) | Err((resource, _)) => resource,
    };
    resource
        .body
        .iter()
        .filter_map(|entry| match entry {
            fluent_syntax::ast::Entry::Message(message) => Some(message.id.name.to_owned()),
            _ => None,
        })
        .collect()
}

/// The report's last section: for each screen, per culture, how many of the keys it asks for the
/// culture's own `.ftl` files lack. `files` are the generated files, keyed `<culture>/<stem>.ftl`.
#[must_use]
pub fn screens_section(files: &BTreeMap<PathBuf, String>, screens: &[ScreenKeys]) -> String {
    let mut by_culture: BTreeMap<String, BTreeSet<String>> = BTreeMap::new();
    for (relative, text) in files {
        if relative.extension().and_then(|e| e.to_str()) != Some("ftl") {
            continue;
        }
        let Some(culture) = relative
            .parent()
            .and_then(|p| p.to_str())
            .filter(|c| !c.is_empty())
        else {
            continue;
        };
        by_culture
            .entry(culture.to_owned())
            .or_default()
            .extend(message_ids(text));
    }
    let mut cultures: Vec<&String> = by_culture.keys().collect();
    // English first, as the base tables have it; the rest in order.
    cultures.sort_by_key(|c| (c.as_str() != BASE_CULTURE, c.as_str()));

    let mut out = String::from(
        "## Screens through Fluent\n\n\
         The keys each screen asks for with `fl!` in its sources, and per culture how many of them \
         the culture's own `.ftl` files lack. English lacking one is a bug - `mp-gui`'s lint fails \
         on it and the screen would show the key's name in brackets; any other culture lacking one \
         shows it in English, as the C# does. At run time a culture also falls back through its \
         parents before English (`zh-TW` through `zh-Hant`); these counts are each culture's own \
         files. `mp-gui`'s per-culture tests hold their counts to this table.\n\n",
    );
    for screen in screens {
        let _ = writeln!(
            out,
            "### {}\n\n{} keys, from {}.\n",
            screen.name,
            screen.keys.len(),
            screen
                .sources
                .iter()
                .map(|s| format!("`{s}`"))
                .collect::<Vec<_>>()
                .join(", ")
        );
        out.push_str("| culture | lacks | translated |\n|---|---:|---:|\n");
        let mut english_lacks = Vec::new();
        for culture in &cultures {
            let have = by_culture.get(*culture);
            let lacking: Vec<&String> = screen
                .keys
                .iter()
                .filter(|key| !have.is_some_and(|ids| ids.contains(*key)))
                .collect();
            if culture.as_str() == BASE_CULTURE {
                english_lacks.clone_from(&lacking);
            }
            let _ = writeln!(
                out,
                "| {culture} | {} | {} |",
                lacking.len(),
                screen.keys.len() - lacking.len()
            );
        }
        out.push('\n');
        if !english_lacks.is_empty() {
            let _ = writeln!(
                out,
                "English lacks: {}.\n",
                english_lacks
                    .iter()
                    .map(|k| format!("`{k}`"))
                    .collect::<Vec<_>>()
                    .join(", ")
            );
        }
    }
    out
}

/// One string entry of a `.resx`: a `<data name=... xml:space="preserve">` with no `type` or
/// `mimetype`, and its `<value>`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Entry {
    /// The `name` attribute.
    pub name: String,
    /// The value's text, entities decoded, `\r\n` made `\n`.
    pub value: String,
}

/// The string entries of a `.resx`, in file order.
pub fn string_entries(xml: &str) -> Result<Vec<Entry>> {
    let doc = roxmltree::Document::parse(xml).context("parsing .resx")?;
    let mut entries = Vec::new();
    for data in doc
        .root_element()
        .children()
        .filter(|n| n.has_tag_name("data"))
    {
        if data.attribute("type").is_some() || data.attribute("mimetype").is_some() {
            continue;
        }
        let Some(name) = data.attribute("name") else {
            continue;
        };
        let value = data
            .children()
            .find(|c| c.has_tag_name("value"))
            .and_then(|v| v.text())
            .unwrap_or_default()
            .replace("\r\n", "\n");
        entries.push(Entry {
            name: name.to_owned(),
            value,
        });
    }
    Ok(entries)
}

/// Whether a string entry carries language: every one in `Strings.resx`; in a screen's file the
/// four suffixes above, and never the `>>` metadata.
#[must_use]
pub fn is_language(strings_file: bool, name: &str) -> bool {
    if name.starts_with(">>") {
        return false;
    }
    if strings_file {
        return true;
    }
    let Some((_, property)) = name.rsplit_once('.') else {
        return false;
    };
    matches!(property, "Text" | "ToolTipText" | "HeaderText")
        || (property.starts_with("Items")
            && property["Items".len()..]
                .chars()
                .all(|c| c.is_ascii_digit()))
}

/// The Fluent id of an entry: the name itself for `Strings.resx`, else `<stem>-<name>` with dots
/// made dashes; any character Fluent's `Identifier` cannot hold becomes `_`, and a leading digit
/// gets a `k` in front.
#[must_use]
pub fn ftl_id(stem: &str, strings_file: bool, name: &str) -> String {
    let raw = if strings_file {
        name.to_owned()
    } else {
        format!("{stem}-{}", name.replace('.', "-"))
    };
    let mut id: String = raw
        .chars()
        .map(|c| {
            if c.is_ascii_alphanumeric() || c == '_' || c == '-' {
                c
            } else {
                '_'
            }
        })
        .collect();
    if !id.chars().next().is_some_and(|c| c.is_ascii_alphabetic()) {
        id.insert(0, 'k');
    }
    id
}

/// The .NET placeholder indices a value uses - `{0}`, `{1:F2}`, `{2,-10}` - sorted, once each.
#[must_use]
pub fn placeholders(value: &str) -> Vec<u32> {
    let mut found = BTreeSet::new();
    let bytes = value.as_bytes();
    let mut i = 0;
    while i < bytes.len() {
        if bytes.get(i) == Some(&b'{')
            && let Some((index, end)) = placeholder_at(value, i)
        {
            found.insert(index);
            i = end;
            continue;
        }
        i += 1;
    }
    found.into_iter().collect()
}

/// A .NET placeholder starting at byte `at`: its index and the byte after its `}`.
fn placeholder_at(value: &str, at: usize) -> Option<(u32, usize)> {
    let rest = &value[at + 1..];
    let digits: String = rest.chars().take_while(char::is_ascii_digit).collect();
    if digits.is_empty() {
        return None;
    }
    let after = &rest[digits.len()..];
    let close = if after.starts_with(':') || after.starts_with(',') {
        after.find('}')?
    } else if after.starts_with('}') {
        0
    } else {
        return None;
    };
    let index = digits.parse().ok()?;
    Some((index, at + 1 + digits.len() + close + 1))
}

/// A Fluent string literal holding `text` exactly.
fn literal(text: &str) -> String {
    let mut out = String::from("{\"");
    for c in text.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\u000A"),
            '\r' => out.push_str("\\u000D"),
            c => out.push(c),
        }
    }
    out.push_str("\"}");
    out
}

/// One line of a value as Fluent will read it back unchanged: the placeholders as variables,
/// other braces as literals, the edges' spaces and a special first character in literals.
fn ftl_line(line: &str) -> String {
    let stripped = line.trim_start_matches(' ');
    let leading = &line[..line.len() - stripped.len()];
    let body = stripped.trim_end_matches(' ');
    let trailing = &stripped[body.len()..];

    let mut out = String::new();
    if !leading.is_empty() {
        out.push_str(&literal(leading));
    }
    let mut chars = body.char_indices().peekable();
    let mut first = true;
    while let Some((i, c)) = chars.next() {
        let special_start = first && matches!(c, '.' | '[' | '*' | '}');
        first = false;
        if c == '{' {
            if let Some((index, end)) = placeholder_at(body, i) {
                let _ = write!(out, "{{ $arg{index} }}");
                while chars.peek().is_some_and(|&(j, _)| j < end) {
                    chars.next();
                }
                continue;
            }
            out.push_str("{\"{\"}");
        } else if c == '}' || special_start {
            out.push_str(&literal(&c.to_string()));
        } else {
            out.push(c);
        }
    }
    if !trailing.is_empty() {
        out.push_str(&literal(trailing));
    }
    if out.is_empty() {
        out.push_str("{\"\"}");
    }
    out
}

/// A value as the right-hand side of `id =`: one line, or the first line and indented
/// continuation lines.
#[must_use]
pub fn ftl_value(value: &str) -> String {
    let value = value.replace("\r\n", "\n").replace('\r', "\n");
    let mut lines = value.split('\n').map(ftl_line);
    let mut out = lines.next().unwrap_or_default();
    for line in lines {
        out.push_str("\n    ");
        out.push_str(&line);
    }
    out
}

/// The `.resx` name → Fluent id of every key ever generated, keyed as `<base path>:<name>`.
#[derive(Debug, Default, Clone, PartialEq, Eq)]
pub struct Keymap {
    keys: BTreeMap<String, String>,
}

impl Keymap {
    /// Reads a `keymap.toml` written by [`Keymap::to_toml`]: `[keys]`, then one
    /// `"<base>:<name>" = "<id>"` a line.
    pub fn parse(text: &str) -> Result<Self> {
        let mut keys = BTreeMap::new();
        for (number, line) in text.lines().enumerate() {
            let line = line.trim();
            if line.is_empty() || line.starts_with('#') || line == "[keys]" {
                continue;
            }
            let Some((key, id)) = line.split_once(" = ") else {
                bail!("keymap.toml line {}: not `\"key\" = \"id\"`", number + 1);
            };
            let unquote = |s: &str| {
                s.strip_prefix('"')
                    .and_then(|s| s.strip_suffix('"'))
                    .map(str::to_owned)
            };
            let (Some(key), Some(id)) = (unquote(key), unquote(id)) else {
                bail!("keymap.toml line {}: not `\"key\" = \"id\"`", number + 1);
            };
            keys.insert(key, id);
        }
        Ok(Self { keys })
    }

    /// The file: sorted, one key a line, and nothing else.
    #[must_use]
    pub fn to_toml(&self) -> String {
        let mut out = String::from(
            "# The .resx name -> Fluent id of every key `cargo xtask codegen-resx` has generated.\n\
             # Immutable: an id that would change is an error (PLAN.md R14). Append-only.\n\n\
             [keys]\n",
        );
        for (key, id) in &self.keys {
            let _ = writeln!(out, "\"{key}\" = \"{id}\"");
        }
        out
    }

    /// Records `id` for `key`, or refuses if the key already has another id.
    pub fn assign(&mut self, key: &str, id: &str) -> Result<()> {
        match self.keys.get(key) {
            Some(held) if held != id => bail!(
                "keymap.toml holds `{key}` as `{held}` and this generation would make it `{id}`: \
                 ids are immutable, since every .ftl and Crowdin's memory are keyed on them"
            ),
            Some(_) => Ok(()),
            None => {
                self.keys.insert(key.to_owned(), id.to_owned());
                Ok(())
            }
        }
    }

    /// How many keys it holds.
    #[must_use]
    pub fn len(&self) -> usize {
        self.keys.len()
    }

    /// Whether it holds none.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.keys.is_empty()
    }
}

/// A culture's file beside a base file: `Strings.zh-Hans.resx` beside `Strings.resx`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Culture {
    /// The code between the stem and `.resx`, as the file spells it.
    pub code: String,
    /// The file.
    pub path: PathBuf,
}

/// The culture files beside a base file, sorted by code.
pub fn cultures_of(base: &Path) -> Result<Vec<Culture>> {
    let stem = base
        .file_stem()
        .and_then(|s| s.to_str())
        .context("base file name")?;
    let dir = base.parent().context("base directory")?;
    let mut found = Vec::new();
    for entry in std::fs::read_dir(dir).with_context(|| format!("reading {}", dir.display()))? {
        let path = entry?.path();
        let Some(name) = path.file_name().and_then(|n| n.to_str()) else {
            continue;
        };
        if let Some(code) = name
            .strip_prefix(stem)
            .and_then(|rest| rest.strip_prefix('.'))
            .and_then(|rest| rest.strip_suffix(".resx"))
            && !code.is_empty()
        {
            found.push(Culture {
                code: code.to_owned(),
                path,
            });
        }
    }
    found.sort_by(|a, b| a.code.cmp(&b.code));
    Ok(found)
}

/// What one culture's conversion found, for the report.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct CultureReport {
    /// The code.
    pub code: String,
    /// String entries in the file.
    pub entries: usize,
    /// Of those, language keys.
    pub language: usize,
    /// Of those, translations of a key the base file has.
    pub translated: usize,
    /// Language entries whose name the base file does not have, kept under `# orphan`.
    pub orphans: Vec<String>,
    /// Translations whose placeholders differ from the base's: `(name, base, translation)`.
    pub placeholder_mismatches: Vec<(String, Vec<u32>, Vec<u32>)>,
    /// Base keys this culture does not translate.
    pub missing: usize,
}

/// What one base file's conversion produced.
#[derive(Debug, Clone, Default)]
pub struct BaseOutput {
    /// The base file, relative to the reference tree.
    pub source: String,
    /// The stem the `.ftl` files take.
    pub stem: String,
    /// Language keys in the base file.
    pub keys: usize,
    /// Base values whose `.NET` placeholders carried a format (`{0:F2}`) that the `.ftl` cannot.
    pub formats_dropped: Vec<String>,
    /// Each culture's report, the base's own first.
    pub cultures: Vec<CultureReport>,
}

/// Everything a run produces: the files, relative to `assets/i18n/`, and what happened.
#[derive(Debug, Clone, Default)]
pub struct Output {
    /// `<culture>/<stem>.ftl` → text, plus `keymap.toml` and `report.md`.
    pub files: BTreeMap<PathBuf, String>,
    /// Per base file.
    pub bases: Vec<BaseOutput>,
}

/// Converts every base in [`BASES`] under `tree` with the keymap given, which it extends, and
/// reports what each of `screens` asks for.
pub fn convert(tree: &Path, mut keymap: Keymap, screens: &[ScreenKeys]) -> Result<Output> {
    let mut output = Output::default();
    for &(relative, stem) in BASES {
        let base_path = tree.join(relative);
        let base_xml = std::fs::read_to_string(&base_path)
            .with_context(|| format!("reading {}", base_path.display()))?;
        let strings_file = stem == "strings";
        let base: Vec<Entry> = string_entries(&base_xml)?
            .into_iter()
            .filter(|e| is_language(strings_file, &e.name))
            .collect();
        let mut ids: BTreeMap<&str, String> = BTreeMap::new();
        for entry in &base {
            let id = ftl_id(stem, strings_file, &entry.name);
            keymap.assign(&format!("{relative}:{}", entry.name), &id)?;
            ids.insert(&entry.name, id);
        }
        let base_placeholders: BTreeMap<&str, Vec<u32>> = base
            .iter()
            .map(|e| (e.name.as_str(), placeholders(&e.value)))
            .collect();
        let formats_dropped: Vec<String> = base
            .iter()
            .filter(|e| has_placeholder_format(&e.value))
            .map(|e| e.name.clone())
            .collect();

        let mut base_out = BaseOutput {
            source: relative.to_owned(),
            stem: stem.to_owned(),
            keys: base.len(),
            formats_dropped,
            cultures: Vec::new(),
        };

        // The base culture's file: every key.
        let mut text = header(relative, base.len(), base.len(), 0);
        for entry in &base {
            let Some(id) = ids.get(entry.name.as_str()) else {
                continue;
            };
            let _ = writeln!(text, "{id} = {}", ftl_value(&entry.value));
        }
        check_ftl(&text, relative)?;
        output.files.insert(
            PathBuf::from(BASE_CULTURE).join(format!("{stem}.ftl")),
            text,
        );
        base_out.cultures.push(CultureReport {
            code: BASE_CULTURE.to_owned(),
            entries: base.len(),
            language: base.len(),
            translated: base.len(),
            ..Default::default()
        });

        for culture in cultures_of(&base_path)? {
            let xml = std::fs::read_to_string(&culture.path)
                .with_context(|| format!("reading {}", culture.path.display()))?;
            let entries = string_entries(&xml)?;
            let language: Vec<&Entry> = entries
                .iter()
                .filter(|e| is_language(strings_file, &e.name))
                .collect();
            let mut report = CultureReport {
                code: culture.code.clone(),
                entries: entries.len(),
                language: language.len(),
                ..Default::default()
            };
            let source = format!(
                "{}/{}",
                Path::new(relative)
                    .parent()
                    .map(|p| p.display().to_string())
                    .unwrap_or_default(),
                culture
                    .path
                    .file_name()
                    .and_then(|n| n.to_str())
                    .unwrap_or_default()
            );
            let mut body = String::new();
            let mut translated: BTreeSet<&str> = BTreeSet::new();
            for entry in &language {
                match ids.get(entry.name.as_str()) {
                    Some(id) => {
                        translated.insert(&entry.name);
                        let theirs = placeholders(&entry.value);
                        let ours = base_placeholders
                            .get(entry.name.as_str())
                            .cloned()
                            .unwrap_or_default();
                        if theirs != ours {
                            report
                                .placeholder_mismatches
                                .push((entry.name.clone(), ours, theirs));
                        }
                        let _ = writeln!(body, "{id} = {}", ftl_value(&entry.value));
                    }
                    None => {
                        // Kept, and said so: a translator wrote it, and the base file lost the
                        // control or renamed it. Its id follows the rule, so a base entry of
                        // that name would meet it again.
                        let id = ftl_id(stem, strings_file, &entry.name);
                        keymap.assign(&format!("{relative}:{}", entry.name), &id)?;
                        report.orphans.push(entry.name.clone());
                        let _ = writeln!(
                            body,
                            "# orphan: no entry `{}` in {relative}\n{id} = {}",
                            entry.name,
                            ftl_value(&entry.value)
                        );
                    }
                }
            }
            report.translated = translated.len();
            report.missing = base.len() - translated.len();
            let mut text = header(&source, base.len(), report.translated, report.orphans.len());
            text.push_str(&body);
            check_ftl(&text, &source)?;
            output.files.insert(
                PathBuf::from(&culture.code).join(format!("{stem}.ftl")),
                text,
            );
            base_out.cultures.push(report);
        }
        output.bases.push(base_out);
    }
    output
        .files
        .insert(PathBuf::from("keymap.toml"), keymap.to_toml());
    let mut report = report(&output.bases, keymap.len());
    if !screens.is_empty() {
        report.push_str(&screens_section(&output.files, screens));
    }
    output.files.insert(PathBuf::from("report.md"), report);
    Ok(output)
}

/// Whether a value has a placeholder with a format or alignment, `{0:F2}` or `{0,-8}`, which the
/// `.ftl` keeps as a bare `{ $arg0 }`.
fn has_placeholder_format(value: &str) -> bool {
    let bytes = value.as_bytes();
    (0..bytes.len()).any(|i| {
        bytes.get(i) == Some(&b'{')
            && placeholder_at(value, i).is_some_and(|(_, end)| {
                value
                    .get(i..end)
                    .is_some_and(|inside| inside.contains(':') || inside.contains(','))
            })
    })
}

fn header(source: &str, keys: usize, translated: usize, orphans: usize) -> String {
    let mut text = format!(
        "# Generated by `cargo xtask codegen-resx` from {source}; do not edit here - edit the \
         .resx, or the translation in Crowdin, and regenerate.\n"
    );
    if orphans == 0 {
        let _ = writeln!(text, "# {translated} of {keys} keys.\n");
    } else {
        let _ = writeln!(
            text,
            "# {translated} of {keys} keys, and {orphans} orphan(s) the base file no longer has.\n"
        );
    }
    text
}

/// Parses a generated file back, so a value Fluent would misread never reaches the tree.
fn check_ftl(text: &str, source: &str) -> Result<()> {
    if let Err((_, errors)) = fluent_syntax::parser::parse(text) {
        let first = errors
            .first()
            .map(|e| format!("{:?} at {:?}", e.kind, e.pos));
        bail!(
            "the .ftl generated from {source} does not parse: {}",
            first.unwrap_or_default()
        );
    }
    Ok(())
}

/// `report.md`: the zero-loss report.
fn report(bases: &[BaseOutput], keys: usize) -> String {
    let mut out = String::from(
        "# .resx → .ftl: the zero-loss report\n\n\
         Generated by `cargo xtask codegen-resx` (DELIVERABLES.md Deliverable 17, PLAN.md §6.1); a test fails \
         when this file is stale. *Entries* are a culture file's string entries, *language* the \
         ones that carry words (every one in `Strings.resx`; `.Text`, `.ToolTipText`, `.HeaderText` \
         and `.Items` in a screen's file), *translated* the ones that translate a key the base \
         file has, *orphans* the ones whose name the base file no longer has - kept in the `.ftl` \
         under an `# orphan` comment, not dropped - and *missing* the base keys the culture does \
         not translate, which fall back to English as they do in the C#. **Loss is zero by \
         construction:** every language entry of every culture file is either a translation or a \
         listed orphan.\n\n",
    );
    let _ = writeln!(out, "Keys in `keymap.toml`: {keys}.\n");
    for base in bases {
        let _ = writeln!(
            out,
            "## `{}` → `<culture>/{}.ftl`\n\n{} language keys in the base file.\n",
            base.source, base.stem, base.keys
        );
        out.push_str("| culture | entries | language | translated | orphans | placeholder mismatches | missing |\n|---|---:|---:|---:|---:|---:|---:|\n");
        for c in &base.cultures {
            let _ = writeln!(
                out,
                "| {} | {} | {} | {} | {} | {} | {} |",
                c.code,
                c.entries,
                c.language,
                c.translated,
                c.orphans.len(),
                c.placeholder_mismatches.len(),
                c.missing
            );
        }
        out.push('\n');
        if !base.formats_dropped.is_empty() {
            let _ = writeln!(
                out,
                "Base values with a placeholder format or alignment the `.ftl` keeps as a bare \
                 variable ({}): {}.\n",
                base.formats_dropped.len(),
                base.formats_dropped.join(", ")
            );
        }
        for c in &base.cultures {
            if !c.orphans.is_empty() {
                let _ = writeln!(out, "Orphans in {}: {}.\n", c.code, c.orphans.join(", "));
            }
            for (name, ours, theirs) in &c.placeholder_mismatches {
                let _ = writeln!(
                    out,
                    "Placeholder mismatch in {}: `{name}` uses {theirs:?} where the base uses {ours:?}.\n",
                    c.code
                );
            }
        }
    }
    out
}
