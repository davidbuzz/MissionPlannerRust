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

//! Generates parameter metadata from Mission Planner's XML.
//!
//! A parameter download gives 1,408 names and numbers. What makes a configuration screen usable is
//! everything around them: what the parameter is called in English, what it does, its units, its
//! legal range, whether it is an enumeration or a bitmask, and whether an ordinary user should be
//! touching it at all.
//!
//! Mission Planner reads that from `ParameterMetaDataBackup.xml`, which is generated from
//! ArduPilot's source comments. We generate a Rust table from the same file, so the descriptions a
//! pilot reads match the firmware they are flying - every vehicle section of it, as the C# reads
//! the connected vehicle's section and no other.

use std::collections::{BTreeMap, HashMap};
use std::path::Path;

use anyhow::{Context, Result, bail};

/// What we keep about a parameter.
#[derive(Debug, Default, Clone, PartialEq)]
pub struct ParamMeta {
    /// Parameter name.
    pub name: String,
    /// Short label for a form field.
    pub display_name: String,
    /// Full description.
    pub description: String,
    /// Units as written in the metadata, e.g. `cm/s`.
    pub units: String,
    /// Inclusive minimum and maximum, when the metadata gives a range.
    pub range: Option<(f64, f64)>,
    /// The `<Range>` text as it stands.
    pub range_text: String,
    /// Suggested step for a spinner.
    pub increment: Option<f64>,
    /// Named values, when the parameter is an enumeration.
    pub values: Vec<(i64, String)>,
    /// Named bits, when it is a bitmask.
    pub bitmask: Vec<(u32, String)>,
    /// `Standard` or `Advanced`; advanced parameters are hidden by default.
    pub user_level: String,
    /// Whether changing it requires a reboot.
    pub reboot_required: bool,
    /// `<ReadOnly>True</ReadOnly>`: the C#'s `ParameterMetaDataRepositoryAPM` fallback answers
    /// `GetParameterMetaData(name, ReadOnly, ...)` from this file for a name the fetched
    /// `apm.pdef.xml` lacks.
    pub read_only: bool,
}

/// Reads one tag's contents.
fn tag<'a>(xml: &'a str, name: &str) -> Option<&'a str> {
    let open = format!("<{name}>");
    let close = format!("</{name}>");
    let start = xml.find(&open)? + open.len();
    let end = xml[start..].find(&close)? + start;
    xml.get(start..end)
}

/// Collapses whitespace and escapes what would break a Rust string literal.
fn clean(text: &str) -> String {
    text.split_whitespace()
        .collect::<Vec<_>>()
        .join(" ")
        .replace('\\', "\\\\")
        .replace('"', "\\\"")
}

/// Parses `0:All,1:Barometer` style lists.
fn pairs(text: &str) -> Vec<(i64, String)> {
    text.split(',')
        .filter_map(|entry| {
            let (number, label) = entry.split_once(':')?;
            let number = number.trim().parse::<i64>().ok()?;
            let label = clean(label);
            (!label.is_empty()).then_some((number, label))
        })
        .collect()
}

/// Extracts every parameter in a vehicle section.
fn parameters_in(section: &str) -> BTreeMap<String, ParamMeta> {
    let mut out = BTreeMap::new();
    let mut rest = section;

    // Parameter entries are `<NAME> ... </NAME>` where NAME is upper case with underscores.
    while let Some(open_at) = rest.find('<') {
        let Some(close_at) = rest[open_at..].find('>').map(|i| i + open_at) else {
            break;
        };
        let name = &rest[open_at + 1..close_at];

        let looks_like_a_parameter = !name.is_empty()
            && name
                .chars()
                .all(|c| c.is_ascii_uppercase() || c.is_ascii_digit() || c == '_')
            && name.chars().any(|c| c.is_ascii_uppercase());
        if !looks_like_a_parameter {
            rest = &rest[close_at + 1..];
            continue;
        }

        let closing = format!("</{name}>");
        let Some(end_at) = rest[close_at..].find(&closing).map(|i| i + close_at) else {
            rest = &rest[close_at + 1..];
            continue;
        };
        let body = &rest[close_at + 1..end_at];

        let range = tag(body, "Range").and_then(|r| {
            let mut parts = r.split_whitespace();
            let low = parts.next()?.parse::<f64>().ok()?;
            let high = parts.next()?.parse::<f64>().ok()?;
            Some((low, high))
        });

        out.insert(
            name.to_owned(),
            ParamMeta {
                name: name.to_owned(),
                display_name: tag(body, "DisplayName").map(clean).unwrap_or_default(),
                description: tag(body, "Description").map(clean).unwrap_or_default(),
                units: tag(body, "Units").map(clean).unwrap_or_default(),
                range,
                range_text: tag(body, "Range").map(clean).unwrap_or_default(),
                increment: tag(body, "Increment").and_then(|i| i.trim().parse().ok()),
                values: tag(body, "Values").map(pairs).unwrap_or_default(),
                bitmask: tag(body, "Bitmask")
                    .map(pairs)
                    .unwrap_or_default()
                    .into_iter()
                    .filter_map(|(bit, label)| u32::try_from(bit).ok().map(|b| (b, label)))
                    .collect(),
                user_level: tag(body, "User").map(clean).unwrap_or_default(),
                reboot_required: tag(body, "RebootRequired")
                    .is_some_and(|v| v.trim().eq_ignore_ascii_case("true")),
                read_only: tag(body, "ReadOnly")
                    .is_some_and(|v| v.trim().eq_ignore_ascii_case("true")),
            },
        );

        rest = &rest[end_at + closing.len()..];
    }
    out
}

/// The vehicle sections of the file, in its order: the elements under `<Params>`, each named as
/// the C#'s `Firmwares` names a vehicle (`ArduCopter2`, `ArduPlane`, ...), which is what
/// `ParameterMetaDataRepositoryAPM.GetParameterMetaData` asks `Elements(vechileType)` for.
/// A name given twice is refused: the C# would read the two in turn, the table keys by name.
/// `// C#: ExtLibs/Utilities/ParameterMetaDataRepositoryAPM.cs:80`
pub fn section_names(xml: &str) -> Result<Vec<String>> {
    let document = roxmltree::Document::parse(xml.trim_start_matches('\u{feff}'))
        .context("parsing the parameter metadata")?;
    let root = document.root_element();
    if root.tag_name().name() != "Params" {
        bail!("the metadata's root is <{}>, not <Params>", root.tag_name().name());
    }
    let mut names: Vec<String> = Vec::new();
    for section in root.children().filter(roxmltree::Node::is_element) {
        let name = section.tag_name().name().to_owned();
        if names.contains(&name) {
            bail!("<{name}> is in the metadata twice");
        }
        names.push(name);
    }
    if names.is_empty() {
        bail!("no vehicle sections in the metadata");
    }
    Ok(names)
}

/// One parameter as the generated table writes it.
fn entry_source(meta: &ParamMeta) -> String {
    let mut out = String::with_capacity(512);
    out.push_str("    ParamMeta {\n");
    out.push_str(&format!("        name: \"{}\",\n", meta.name));
    out.push_str(&format!(
        "        display_name: \"{}\",\n",
        meta.display_name
    ));
    out.push_str(&format!("        description: \"{}\",\n", meta.description));
    out.push_str(&format!("        units: \"{}\",\n", meta.units));
    match meta.range {
        Some((low, high)) => {
            out.push_str(&format!("        range: Some(({low:?}, {high:?})),\n"));
        }
        None => out.push_str("        range: None,\n"),
    }
    out.push_str(&format!("        range_text: \"{}\",\n", meta.range_text));
    match meta.increment {
        Some(step) => out.push_str(&format!("        increment: Some({step:?}),\n")),
        None => out.push_str("        increment: None,\n"),
    }
    out.push_str("        values: &[");
    for (number, label) in &meta.values {
        out.push_str(&format!("({number}, \"{label}\"), "));
    }
    out.push_str("],\n        bitmask: &[");
    for (bit, label) in &meta.bitmask {
        out.push_str(&format!("({bit}, \"{label}\"), "));
    }
    out.push_str("],\n");
    let level = match meta.user_level.as_str() {
        "Standard" => "UserLevel::Standard",
        "Advanced" => "UserLevel::Advanced",
        _ => "UserLevel::Unspecified",
    };
    out.push_str(&format!("        user_level: {level},\n"));
    out.push_str(&format!(
        "        reboot_required: {},\n",
        meta.reboot_required
    ));
    out.push_str("    },\n");
    out
}

/// Generates the metadata module for every vehicle section of the file.
///
/// The C# reads the section of the vehicle it is asked for and no other
/// (`// C#: ExtLibs/Utilities/ParameterMetaDataRepositoryAPM.cs:70-104`), so every section is
/// kept, by its name. Most parameters are documented alike in several sections - 11,474 in the
/// five sections of the file at 5dbb2b0, about 4,330 of them distinct - so the table holds each
/// distinct entry once, in `ENTRIES`, sorted by name (a name documented differently in two
/// sections, `FLTMODE1` on a copter and a plane, is two entries, in the order the sections give
/// them), and each section lists its parameters as places in it, in name order, for the same
/// binary search the one-section table had.
pub fn generate(metadata_path: &Path) -> Result<String> {
    let xml = std::fs::read_to_string(metadata_path)
        .with_context(|| format!("reading {}", metadata_path.display()))?;

    let mut sections: Vec<(String, BTreeMap<String, ParamMeta>)> = Vec::new();
    for name in section_names(&xml)? {
        let section =
            tag(&xml, &name).with_context(|| format!("no <{name}> section in the metadata"))?;
        let params = parameters_in(section);
        if params.len() < 100 {
            bail!(
                "only {} parameters found in {name}; the parser is wrong",
                params.len()
            );
        }
        sections.push((name, params));
    }

    // Each name's ways of being documented, in the order the sections give them.
    let mut variants: BTreeMap<&str, Vec<String>> = BTreeMap::new();
    for (_, params) in &sections {
        for meta in params.values() {
            let source = entry_source(meta);
            let seen = variants.entry(meta.name.as_str()).or_default();
            if !seen.contains(&source) {
                seen.push(source);
            }
        }
    }
    // Their places: by name, then in that order. An entry's text names it, so no two names
    // share one.
    let mut entries: Vec<&str> = Vec::new();
    let mut place: HashMap<&str, usize> = HashMap::new();
    for sources in variants.values() {
        for source in sources {
            place.insert(source.as_str(), entries.len());
            entries.push(source.as_str());
        }
    }
    if u16::try_from(entries.len()).is_err() {
        bail!(
            "{} distinct entries do not fit the table's u16 places",
            entries.len()
        );
    }
    let total: usize = sections.iter().map(|(_, params)| params.len()).sum();

    let mut out = String::with_capacity(entries.len() * 560 + total * 8);
    out.push_str(crate::licence::HEADER);
    out.push('\n');
    out.push_str(&format!(
        "//! Parameter metadata for every vehicle section of Mission Planner's XML, generated from it.\n\
         //!\n\
         //! DO NOT EDIT. Regenerate with `cargo xtask codegen-param-meta`.\n\
         //!\n\
         //! Source: https://github.com/ArduPilot/MissionPlanner/blob/5dbb2b0/ParameterMetaDataBackup.xml, which ArduPilot\n\
         //! generates from its own source comments. {} sections, {total} parameters, {} distinct entries.\n\n\
         // These are ranges and defaults copied from firmware documentation, not computed\n\
         // constants: a parameter whose range happens to be -3.142 to 3.142 is expressing\n\
         // radians, not approximating pi badly.\n\
         #![allow(clippy::approx_constant)]\n\n\
         use super::{{ParamMeta, Section, UserLevel}};\n\n",
        sections.len(),
        entries.len()
    ));

    out.push_str(
        "/// Every distinct entry, sorted by name: a parameter documented alike in several sections \
         is held once, one documented differently once for each way.\n",
    );
    out.push_str("pub(super) static ENTRIES: &[ParamMeta] = &[\n");
    for source in &entries {
        out.push_str(source);
    }
    out.push_str("];\n\n");

    out.push_str(
        "/// The vehicle sections, in the file's order: each one's parameters as places in \
         [`ENTRIES`], in name order, and the names it marks `<ReadOnly>True</ReadOnly>`, sorted - \
         the C#'s fallback for a name the fetched documentation lacks.\n",
    );
    out.push_str("pub(super) static SECTIONS: &[Section] = &[\n");
    for (name, params) in &sections {
        out.push_str("    Section {\n");
        out.push_str(&format!("        name: \"{name}\",\n"));
        out.push_str("        entries: &[");
        for meta in params.values() {
            let source = entry_source(meta);
            let at = place
                .get(source.as_str())
                .with_context(|| format!("{name}'s {} has no place", meta.name))?;
            out.push_str(&format!("{at}, "));
        }
        out.push_str("],\n        read_only: &[");
        for meta in params.values().filter(|meta| meta.read_only) {
            out.push_str(&format!("\"{}\", ", meta.name));
        }
        out.push_str("],\n    },\n");
    }
    out.push_str("];\n");

    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A file of two sections of a hundred parameters each, `P050` documented differently in
    /// the second, and a parameter only the first has.
    fn two_sections() -> String {
        let section = |name: &str, extra: &str, p050: &str| {
            let mut body = String::new();
            for n in 0..100 {
                let description = if n == 50 { p050 } else { "alike" };
                body.push_str(&format!(
                    "<P{n:03}><DisplayName>P{n:03}</DisplayName><Description>{description}</Description></P{n:03}>"
                ));
            }
            body.push_str(extra);
            format!("<{name}>{body}</{name}>")
        };
        format!(
            "\u{feff}<?xml version=\"1.0\" encoding=\"utf-8\"?>\n<Params>{}{}</Params>",
            section(
                "A",
                "<ONLY_A><DisplayName>a</DisplayName><ReadOnly>True</ReadOnly></ONLY_A>",
                "first"
            ),
            section("B", "", "second")
        )
    }

    /// The sections are the elements under `<Params>`, in order, after a byte order mark; a
    /// name given twice, another root, or no section at all is refused.
    #[test]
    fn the_sections_are_the_elements_under_params() {
        assert_eq!(section_names(&two_sections()).unwrap(), ["A", "B"]);
        assert!(section_names("<Params><A><X/></A><A><Y/></A></Params>").is_err());
        assert!(section_names("<Other><A/></Other>").is_err());
        assert!(section_names("<Params></Params>").is_err());
    }

    /// Every section is generated, by name; an entry documented alike in both is held once, the
    /// one documented differently twice, the first section's first; each section's places are
    /// in name order and its ReadOnly names its own.
    #[test]
    fn every_section_is_generated_and_alike_entries_are_held_once() {
        let path = std::env::temp_dir().join(format!(
            "xtask-param-meta-{}.xml",
            std::process::id()
        ));
        std::fs::write(&path, two_sections()).unwrap();
        let source = generate(&path).unwrap();
        std::fs::remove_file(&path).unwrap();

        assert!(source.contains("2 sections, 201 parameters, 102 distinct entries"));
        // ONLY_A, then P000-P049, P050 twice, P051-P099.
        assert_eq!(source.matches("    ParamMeta {\n").count(), 102);
        let first = source.find("description: \"first\"").unwrap();
        let second = source.find("description: \"second\"").unwrap();
        assert!(first < second, "the first section's P050 first");
        let list = |places: Vec<usize>| {
            let mut text = String::from("entries: &[");
            for place in places {
                text.push_str(&format!("{place}, "));
            }
            text.push_str("],");
            text
        };
        let a: Vec<usize> = std::iter::once(0).chain(1..=51).chain(53..=101).collect();
        let b: Vec<usize> = (1..=50).chain(std::iter::once(52)).chain(53..=101).collect();
        assert!(source.contains(&format!(
            "        name: \"A\",\n        {}\n        read_only: &[\"ONLY_A\", ],",
            list(a)
        )));
        assert!(source.contains(&format!(
            "        name: \"B\",\n        {}\n        read_only: &[],",
            list(b)
        )));
    }
}
