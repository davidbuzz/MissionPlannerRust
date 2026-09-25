//! Generates parameter metadata from Mission Planner's XML.
//!
//! A parameter download gives 1,408 names and numbers. What makes a configuration screen usable is
//! everything around them: what the parameter is called in English, what it does, its units, its
//! legal range, whether it is an enumeration or a bitmask, and whether an ordinary user should be
//! touching it at all.
//!
//! Mission Planner reads that from `ParameterMetaDataBackup.xml`, which is generated from
//! ArduPilot's source comments. We generate a Rust table from the same file, so the descriptions a
//! pilot reads match the firmware they are flying.

use std::collections::BTreeMap;
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

/// Generates the metadata module for one vehicle section.
pub fn generate(metadata_path: &Path, section_name: &str) -> Result<String> {
    let xml = std::fs::read_to_string(metadata_path)
        .with_context(|| format!("reading {}", metadata_path.display()))?;
    let section = tag(&xml, section_name)
        .with_context(|| format!("no <{section_name}> section in the metadata"))?;

    let params = parameters_in(section);
    if params.len() < 100 {
        bail!(
            "only {} parameters found in {section_name}; the parser is wrong",
            params.len()
        );
    }

    let mut out = String::with_capacity(params.len() * 400);
    out.push_str(&format!(
        "//! Parameter metadata for {section_name}, generated from Mission Planner's XML.\n\
         //!\n\
         //! DO NOT EDIT. Regenerate with `cargo xtask codegen-param-meta`.\n\
         //!\n\
         //! Source: `references/missionplanner/ParameterMetaDataBackup.xml`, which ArduPilot\n\
         //! generates from its own source comments. {} parameters.\n\n\
         // These are ranges and defaults copied from firmware documentation, not computed\n\
         // constants: a parameter whose range happens to be -3.142 to 3.142 is expressing\n\
         // radians, not approximating pi badly.\n\
         #![allow(clippy::approx_constant)]\n\n\
         use super::{{ParamMeta, UserLevel}};\n\n",
        params.len()
    ));

    out.push_str("/// Every documented parameter, sorted by name so lookup is a binary search.\n");
    out.push_str("pub static PARAMETERS: &[ParamMeta] = &[\n");
    for meta in params.values() {
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
    }
    out.push_str("];\n");

    // The names marked ReadOnly, sorted as the table is, for the same binary search.
    out.push_str(
        "\n/// The parameters the file marks `<ReadOnly>True</ReadOnly>`, sorted by name: the C#'s \
         fallback for a name the fetched documentation lacks.\n",
    );
    out.push_str("pub static READ_ONLY: &[&str] = &[\n");
    for meta in params.values().filter(|meta| meta.read_only) {
        out.push_str(&format!("    \"{}\",\n", meta.name));
    }
    out.push_str("];\n");

    Ok(out)
}
