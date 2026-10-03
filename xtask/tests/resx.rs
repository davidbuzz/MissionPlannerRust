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

//! `.resx` → `.ftl` (DELIVERABLES.md D17, PLAN.md §13.6 row 76): every generated message formats
//! back to the .NET string it came from, nothing a translator wrote is lost, and the ids never
//! move.
//!
//! The round trip is the point. A `.ftl` that parses is not a `.ftl` that says what the C# said:
//! Fluent trims edge spaces, dedents continuation lines, reads a line's first `.` or `[` as
//! syntax, and inserts isolation marks around placeables unless told not to. So each message is
//! put through a real `FluentBundle` with the placeholders filled, and the result compared with
//! the `.resx` value with the same placeholders filled - for every language entry of every
//! culture file of every base, against the reference tree when it is here, and over the awkward
//! cases by hand when it is not.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use std::path::{Path, PathBuf};

use fluent_bundle::{FluentArgs, FluentBundle, FluentResource};
use unic_langid::LanguageIdentifier;
use xtask::codegen::resx::{
    BASES, Keymap, SCREENS, ScreenKeys, convert, cultures_of, ftl_id, ftl_value, is_language,
    keys_in, placeholders, screen_keys, screens_section, string_entries,
};

fn repo() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("..")
}

fn tree() -> Option<PathBuf> {
    let tree = xtask::upstream::tree();
    if tree.is_none() {
        println!("{}; the tree tests are skipped", xtask::upstream::absent());
    }
    tree
}

/// The .NET string with `{n}`, `{n:format}` and `{n,align}` replaced by what the bundle is given
/// for `$argn`.
fn dotnet_filled(value: &str) -> String {
    let value = value.replace("\r\n", "\n");
    let mut out = String::new();
    let mut rest = value.as_str();
    while let Some(open) = rest.find('{') {
        out.push_str(&rest[..open]);
        let after = &rest[open + 1..];
        let digits: String = after.chars().take_while(char::is_ascii_digit).collect();
        let tail = &after[digits.len()..];
        let close = if digits.is_empty() {
            None
        } else if let Some(stripped) = tail.strip_prefix('}') {
            Some((stripped, 0))
        } else if tail.starts_with(':') || tail.starts_with(',') {
            tail.find('}').map(|c| (&tail[c + 1..], c + 1))
        } else {
            None
        };
        match close {
            Some((remaining, _)) => {
                out.push_str(&format!("<{digits}>"));
                rest = remaining;
            }
            None => {
                out.push('{');
                rest = after;
            }
        }
    }
    out.push_str(rest);
    out
}

/// Formats every message of a `.ftl` and returns `(id, text)`, with `$arg0..$arg9` as `<0>..<9>`.
fn formatted(culture: &str, ftl: &str) -> Vec<(String, String)> {
    let lang: LanguageIdentifier = culture
        .parse()
        .unwrap_or_else(|e| panic!("{culture}: {e:?}"));
    let resource = FluentResource::try_new(ftl.to_owned())
        .unwrap_or_else(|(_, errors)| panic!("{culture}: {errors:?}"));
    let mut bundle = FluentBundle::new(vec![lang]);
    // The isolation marks (FSI/PDI) Fluent puts round placeables are for bidirectional text in
    // a browser; the C# strings have none, and neither may these.
    bundle.set_use_isolating(false);
    bundle.add_resource(resource).expect("no duplicate ids");
    let mut args = FluentArgs::new();
    for n in 0..10 {
        args.set(format!("arg{n}"), format!("<{n}>"));
    }
    let ids: Vec<String> = ftl
        .lines()
        .filter(|l| !l.starts_with('#') && !l.starts_with(' ') && l.contains(" = "))
        .map(|l| l.split(" = ").next().unwrap().to_owned())
        .collect();
    ids.into_iter()
        .map(|id| {
            let message = bundle
                .get_message(&id)
                .unwrap_or_else(|| panic!("{culture}: no {id}"));
            let pattern = message
                .value()
                .unwrap_or_else(|| panic!("{culture}: {id} has no value"));
            let mut errors = Vec::new();
            let text = bundle
                .format_pattern(pattern, Some(&args), &mut errors)
                .into_owned();
            assert!(errors.is_empty(), "{culture}: {id}: {errors:?}");
            (id, text)
        })
        .collect()
}

#[test]
fn the_awkward_values_survive_the_round_trip() {
    // Each is a shape a real value has: the C#'s trailing newline, a leading newline, edge
    // spaces, a bare brace, a placeholder with a format, a line starting with `[` or `.`, an
    // empty line in the middle, an empty value, a quote and a backslash.
    let cases = [
        "The Command failed to execute\n",
        "\nPlease upgrade",
        "Getting ",
        "  two leading",
        "Epprom format changed ({0) vs {1})",
        "Upload succeeded, but verify failed: exp {0} got {1} at ",
        "Timeout in {0:F1} s, {1,-8}|",
        "Update Found\n\nWould you like to update now? [link;",
        "[first]\n.second\n*third\n}fourth",
        "",
        "a \"quoted\" \\ backslash",
        "trailing space \nthen\r\nwindows",
        "{ $notavar } and {{double}}",
    ];
    let mut ftl = String::new();
    for (n, case) in cases.iter().enumerate() {
        ftl.push_str(&format!("case{n} = {}\n", ftl_value(case)));
    }
    let got = formatted("en", &ftl);
    for (n, case) in cases.iter().enumerate() {
        assert_eq!(got[n].1, dotnet_filled(case), "case {n}: {case:?}\n{ftl}");
    }
}

#[test]
fn language_keys_ids_and_placeholders_follow_the_rules() {
    assert!(is_language(true, "ErrorNoResponse"));
    assert!(!is_language(true, ">>$this.Name"));
    assert!(is_language(false, "BUT_ARM.Text"));
    assert!(is_language(false, "CMB_modes.Items"));
    assert!(is_language(false, "CMB_modes.Items12"));
    assert!(is_language(false, "col.HeaderText"));
    assert!(is_language(false, "but.ToolTipText"));
    assert!(!is_language(false, "BUT_ARM.Size"));
    assert!(!is_language(false, "BUT_ARM.ItemsX"));
    assert!(!is_language(false, ">>BUT_ARM.Name"));
    assert_eq!(
        ftl_id("flightdata", false, "BUT_ARM.Text"),
        "flightdata-BUT_ARM-Text"
    );
    assert_eq!(ftl_id("strings", true, "ERROR"), "ERROR");
    assert_eq!(ftl_id("x", true, "1st thing"), "k1st_thing");
    assert_eq!(
        placeholders("exp {0} got {1} at {0:F2} {2,-4}"),
        vec![0, 1, 2]
    );
    assert_eq!(placeholders("({0) vs {1})"), vec![1]);
    assert_eq!(placeholders("{ $x }"), Vec::<u32>::new());
}

#[test]
fn the_keymap_never_moves_an_id() {
    let mut keymap = Keymap::parse("[keys]\n\"a.resx:Name.Text\" = \"a-Name-Text\"\n").unwrap();
    keymap.assign("a.resx:Name.Text", "a-Name-Text").unwrap();
    assert!(keymap.assign("a.resx:Name.Text", "other").is_err());
    keymap.assign("a.resx:Other.Text", "a-Other-Text").unwrap();
    let text = keymap.to_toml();
    assert_eq!(Keymap::parse(&text).unwrap(), keymap);
    assert_eq!(keymap.len(), 2);
}

#[test]
fn every_message_of_every_culture_formats_back_to_its_resx_value() {
    let Some(tree) = tree() else { return };
    let output = convert(&tree, Keymap::default(), &[]).unwrap();
    let mut checked = 0;
    for &(relative, stem) in BASES {
        let strings_file = stem == "strings";
        let base_path = tree.join(relative);
        let mut sources = vec![("en".to_owned(), base_path.clone())];
        sources.extend(
            cultures_of(&base_path)
                .unwrap()
                .into_iter()
                .map(|c| (c.code, c.path)),
        );
        for (culture, path) in sources {
            let entries = string_entries(&std::fs::read_to_string(&path).unwrap()).unwrap();
            let ftl = &output.files[&PathBuf::from(&culture).join(format!("{stem}.ftl"))];
            let got = formatted(&culture, ftl);
            for entry in entries
                .iter()
                .filter(|e| is_language(strings_file, &e.name))
            {
                let id = ftl_id(stem, strings_file, &entry.name);
                let (_, text) = got
                    .iter()
                    .find(|(found, _)| *found == id)
                    .unwrap_or_else(|| panic!("{culture}/{stem}: {id} missing"));
                assert_eq!(
                    *text,
                    dotnet_filled(&entry.value),
                    "{culture}/{stem}: {} ({id})",
                    entry.name
                );
                checked += 1;
            }
        }
    }
    println!("{checked} messages formatted back to their .resx values");
    assert!(checked > 2000, "{checked}");
}

#[test]
fn nothing_a_translator_wrote_is_lost_and_the_counts_are_the_upstream_pins() {
    let Some(tree) = tree() else { return };
    let output = convert(&tree, Keymap::default(), &[]).unwrap();
    for base in &output.bases {
        for culture in &base.cultures {
            assert_eq!(
                culture.language,
                culture.translated + culture.orphans.len(),
                "{}/{}: a language entry is neither a translation nor a listed orphan",
                base.source,
                culture.code
            );
            assert_eq!(culture.missing, base.keys - culture.translated);
        }
    }
    // Mission Planner at efb0801: what the two base files hold.
    assert_eq!(output.bases[0].keys, 185, "Strings.resx language keys");
    assert_eq!(output.bases[1].keys, 135, "FlightData.resx language keys");
    assert_eq!(
        output.bases[0].cultures.len(),
        11,
        "Strings: en and ten cultures"
    );
    assert_eq!(
        output.bases[1].cultures.len(),
        19,
        "FlightData: en and eighteen cultures"
    );
}

#[test]
fn the_committed_assets_are_what_the_generator_writes() {
    let Some(tree) = tree() else { return };
    let dir = repo().join("assets/i18n");
    let keymap = Keymap::parse(&std::fs::read_to_string(dir.join("keymap.toml")).unwrap()).unwrap();
    let screens = screen_keys(&repo()).unwrap();
    let output = convert(&tree, keymap, &screens).unwrap();
    for (relative, text) in &output.files {
        let committed = std::fs::read_to_string(dir.join(relative)).unwrap_or_else(|e| {
            panic!(
                "{}: {e}; run `cargo xtask codegen-resx`",
                relative.display()
            )
        });
        assert_eq!(
            committed,
            *text,
            "{} is stale; run `cargo xtask codegen-resx`",
            relative.display()
        );
    }
}

#[test]
fn a_screens_keys_are_every_fl_call_in_its_sources() {
    let source = "a(fl!(\"one\")); b(crate::i18n::fl!(\n    \"two\",\n)); fl!(\"one\"); \
                  fl!(\"three\", arg0 = x); fl!(not_a_literal)";
    assert_eq!(keys_in(source), ["one", "two", "three"]);

    // The flight screen, as it is in the tree: its tab pages and buttons.
    let screens = screen_keys(&repo()).unwrap();
    assert_eq!(screens.len(), SCREENS.len());
    let flight = &screens[0];
    assert!(flight.keys.len() >= 40, "{:?}", flight.keys);
    assert!(flight.keys.iter().all(|k| k.starts_with("flightdata-")));
    assert!(
        flight
            .keys
            .contains(&"flightdata-tabActions-Text".to_owned())
    );
}

#[test]
fn the_screens_section_counts_what_each_culture_lacks() {
    let files: std::collections::BTreeMap<PathBuf, String> = [
        ("en/s.ftl", "a = A\nb = B\nc = C\n"),
        ("de-DE/s.ftl", "a = Ah\n"),
        ("fr/s.ftl", "a = Ah\nb = Beh\n# orphan\nz = Z\n"),
        ("keymap.toml", "a = b\n"),
    ]
    .into_iter()
    .map(|(p, t)| (PathBuf::from(p), t.to_owned()))
    .collect();
    let screens = [ScreenKeys {
        name: "Test".to_owned(),
        sources: vec!["x.rs".to_owned()],
        keys: vec!["a".to_owned(), "b".to_owned(), "d".to_owned()],
    }];
    let section = screens_section(&files, &screens);
    assert!(
        section.starts_with("## Screens through Fluent\n"),
        "{section}"
    );
    assert!(
        section.contains("### Test\n\n3 keys, from `x.rs`.\n"),
        "{section}"
    );
    // English first, then the rest in order.
    let rows: Vec<&str> = section
        .lines()
        .filter(|l| l.starts_with("| ") && !l.starts_with("| culture"))
        .collect();
    assert_eq!(
        rows,
        ["| en | 1 | 2 |", "| de-DE | 2 | 1 |", "| fr | 1 | 2 |"]
    );
    assert!(section.contains("English lacks: `d`."), "{section}");
}
