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

//! The screens' words through Fluent: DELIVERABLES.md Deliverable 17, PLAN.md §13.6 row 76.
//!
//! Mission Planner keeps its words in `.resx` files, a base file in English and a sibling per
//! culture holding what that culture translates; .NET's `ResourceManager` looks a name up in the
//! UI culture's file, then in that culture's parents', then in the base file.
//! `cargo xtask codegen-resx` turns those files into `assets/i18n/<culture>/<stem>.ftl`, and this
//! module reads them back the way .NET reads the `.resx`:
//!
//! - **What the binary carries.** Every `.ftl` under `assets/i18n`, embedded with `include_str!`
//!   ([`FILES`]), so a build needs nothing beside it at run time and a culture cannot go missing
//!   from an installation. A test holds the table to the directory.
//! - **Which culture.** `L10N.GetConfigLang`: config.xml's `language` ([`SETTING`]) when it is
//!   there, else English - the owner's ruling of 2026-09-25, where the C# takes the system's UI
//!   culture (kept in [`system_culture`] for when that changes: `LC_ALL`, `LC_MESSAGES`, `LANG`).
//!   Read once, at start-up, as `MainV2`'s constructor reads it; nothing here changes it while the
//!   application runs (the Planner page's language box is not ported).
//! - **The fallback chain.** The culture, then its parents - `de-DE`, `de`; `az-Latn-AZ`,
//!   `az-Latn`, `az`; and .NET's own `zh-TW` → `zh-Hant`, `zh-CN` → `zh-Hans` - each one that has
//!   files here, matched without regard to case as .NET matches culture names, then English.
//!   A culture with no files at all is English, as it is in the C#; [`Catalog::note`] says so,
//!   for a status line at most - never a box; today it is only the `i18n.note` fact.
//! - **A lookup.** [`fl!`]`("flightdata-BUT_ARM-Text")` is the text of the first culture in the
//!   chain that has the key; `fl!("ErrorNoResponse", arg0 = count)` fills `{ $arg0 }`, which is
//!   what `codegen-resx` made of .NET's `{0}`. A key no culture in the chain has - English
//!   included - is shown as its name in brackets, `[flightdata-Nope-Text]`, and remembered
//!   ([`Catalog::missed`]); `i18n.missing` publishes the count for a UI test, and the lint below
//!   fails the build's tests before it can happen on screen.
//!
//! **The lint.** [`SCREEN_SOURCES`] are the flight screen's files; a test walks every key they ask
//! for with `fl!` and fails when English lacks one, and one test per culture reports how many of
//! them that culture lacks. The C#'s cultures are incomplete too, so those only report - and hold
//! the count to the table `codegen-resx` writes into `assets/i18n/report.md`, so the report
//! cannot go stale either.
//!
//! Bidirectional isolation marks are off: Fluent would otherwise wrap every placeable in U+2068
//! and U+2069, which .NET's `string.Format` never did, and the English text would change.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::collections::{BTreeMap, HashMap};
use std::sync::{Mutex, OnceLock, PoisonError};

use fluent_bundle::concurrent::FluentBundle;
use fluent_bundle::{FluentArgs, FluentResource};
use unic_langid::LanguageIdentifier;

/// The config.xml key the culture is read from. `// C#: L10N.cs:19-25; MainV2.cs:697-700`
pub const SETTING: &str = "language";

/// The culture the base `.resx` files are written in, and the end of every chain.
pub const BASE: &str = "en";

/// `(culture, stem, text)` of `assets/i18n/<culture>/<stem>.ftl`, the text embedded.
macro_rules! ftl {
    ($culture:literal, $stem:literal) => {
        (
            $culture,
            $stem,
            include_str!(concat!(
                "../../../assets/i18n/",
                $culture,
                "/",
                $stem,
                ".ftl"
            )),
        )
    };
}

/// Every `.ftl` the binary carries: `(culture, stem, text)`, as `assets/i18n/<culture>/<stem>.ftl`.
pub const FILES: &[(&str, &str, &str)] = &[
    ftl!("ar", "flightdata"),
    ftl!("az-Latn-AZ", "flightdata"),
    ftl!("az-Latn-AZ", "strings"),
    ftl!("de-DE", "flightdata"),
    ftl!("de-DE", "strings"),
    ftl!("en", "flightdata"),
    ftl!("en", "strings"),
    ftl!("es-ES", "flightdata"),
    ftl!("fr", "flightdata"),
    ftl!("id-ID", "flightdata"),
    ftl!("id-ID", "strings"),
    ftl!("it-IT", "flightdata"),
    ftl!("ja-JP", "flightdata"),
    ftl!("ja-JP", "strings"),
    ftl!("ko-KR", "flightdata"),
    ftl!("ko-KR", "strings"),
    ftl!("pl", "flightdata"),
    ftl!("pt", "flightdata"),
    ftl!("pt", "strings"),
    ftl!("ru-KZ", "flightdata"),
    ftl!("ru-KZ", "strings"),
    ftl!("ru-RU", "flightdata"),
    ftl!("tr", "flightdata"),
    ftl!("tr", "strings"),
    ftl!("uk", "flightdata"),
    ftl!("uk", "strings"),
    ftl!("zh-Hans", "flightdata"),
    ftl!("zh-Hans", "strings"),
    ftl!("zh-Hant", "flightdata"),
    ftl!("zh-TW", "flightdata"),
];

/// The flight screen's source files, relative to this crate's `src`: the ones whose `fl!` keys
/// the lint walks, and `codegen-resx`'s report counts per culture (its `SCREENS` names the same).
/// The lint's, so the tests'.
#[cfg(test)]
pub const SCREEN_SOURCES: &[&str] = &["fly.rs", "payload.rs", "transponder.rs"];

/// The cultures [`FILES`] has, sorted, once each.
#[must_use]
pub fn cultures() -> Vec<&'static str> {
    let mut found: Vec<&'static str> = FILES.iter().map(|&(culture, _, _)| culture).collect();
    found.sort_unstable();
    found.dedup();
    found
}

/// The system's UI culture as .NET finds it on Linux: the first of `LC_ALL`, `LC_MESSAGES` and
/// `LANG` that is set, made a culture name - or the invariant culture, `""`. Not consulted while
/// the owner's ruling stands (see [`configured`]); kept, and tested, for when it does not.
#[allow(dead_code)] // kept for the day the ruling changes
fn system_culture() -> String {
    ["LC_ALL", "LC_MESSAGES", "LANG"]
        .iter()
        .find_map(|name| std::env::var(name).ok().filter(|value| !value.is_empty()))
        .map_or_else(String::new, |value| posix_culture(&value))
}

/// A POSIX locale as a culture name: `de_DE.UTF-8@euro` is `de-DE`; `C` and `POSIX` are the
/// invariant culture, `""`.
#[must_use]
pub fn posix_culture(locale: &str) -> String {
    let name = locale.split(['.', '@']).next().unwrap_or_default().trim();
    if name.eq_ignore_ascii_case("C") || name.eq_ignore_ascii_case("POSIX") {
        return String::new();
    }
    name.replace('_', "-")
}

/// `L10N.GetConfigLang`: the culture config.xml's `language` names, or the system's UI culture
/// when it names none. An empty value is none: `MainV2` changes the language only for a value
/// that is not empty, and the screens are in the UI culture.
/// `// C#: L10N.cs:19-25; MainV2.cs:697-700`
#[must_use]
pub fn configured(language: Option<&str>) -> String {
    // The C# takes the system's UI culture when `language` is empty (`L10N.cs:19-25`); the
    // owner ruled (2026-09-25) that an empty key is English here. `system_culture` stays for
    // the day that changes.
    language
        .map(str::trim)
        .filter(|name| !name.is_empty())
        .map_or_else(|| BASE.to_owned(), str::to_owned)
}

/// The cultures a lookup tries, most specific first, each one that has files here, and English
/// last: `ResourceManager`'s walk up `CultureInfo.Parent` to the base file.
#[must_use]
pub fn chain(culture: &str) -> Vec<&'static str> {
    let available = cultures();
    let mut names = vec![culture.to_owned()];
    let mut rest = culture;
    while let Some((parent, _)) = rest.rsplit_once('-') {
        // .NET's parents for the Chinese regions are the scripts, not the bare `zh`.
        let lower = rest.to_ascii_lowercase();
        if matches!(lower.as_str(), "zh-tw" | "zh-hk" | "zh-mo") {
            names.push("zh-Hant".to_owned());
        } else if matches!(lower.as_str(), "zh-cn" | "zh-sg") {
            names.push("zh-Hans".to_owned());
        }
        names.push(parent.to_owned());
        rest = parent;
    }
    names.push(BASE.to_owned());
    let mut found: Vec<&'static str> = Vec::new();
    for name in names {
        if let Some(&culture) = available
            .iter()
            .find(|culture| culture.eq_ignore_ascii_case(&name))
            && !found.contains(&culture)
        {
            found.push(culture);
        }
    }
    found
}

/// The ids of the messages a generated `.ftl` defines: every line that starts an entry, which in
/// `codegen-resx`'s files is an identifier at the start of the line followed by ` = `.
fn ids(ftl: &str) -> impl Iterator<Item = &str> {
    ftl.lines().filter_map(|line| {
        let (id, _) = line.split_once(" = ")?;
        (id.starts_with(|c: char| c.is_ascii_alphabetic())
            && id
                .chars()
                .all(|c| c.is_ascii_alphanumeric() || c == '_' || c == '-'))
        .then_some(id)
    })
}

/// One culture's files as a bundle.
struct Culture {
    /// The culture's directory name.
    name: &'static str,
    /// Its messages.
    bundle: FluentBundle<FluentResource>,
}

impl Culture {
    fn new(name: &'static str) -> Self {
        let lang: LanguageIdentifier = name.parse().unwrap_or_default();
        let mut bundle = FluentBundle::new_concurrent(vec![lang]);
        bundle.set_use_isolating(false);
        for &(_, _, text) in FILES.iter().filter(|&&(culture, _, _)| culture == name) {
            // A file that does not parse cleanly still gives the entries it could read (the
            // generator parses every file back before writing it, so this is not expected);
            // an id defined twice keeps its first.
            let resource = FluentResource::try_new(text.to_owned())
                .unwrap_or_else(|(resource, _errors)| resource);
            let _ = bundle.add_resource(resource);
        }
        Self { name, bundle }
    }

    /// The message's text with the arguments given, if the culture has it.
    fn format(&self, id: &str, args: Option<&FluentArgs>) -> Option<String> {
        let pattern = self.bundle.get_message(id)?.value()?;
        let mut errors = Vec::new();
        Some(
            self.bundle
                .format_pattern(pattern, args, &mut errors)
                .into_owned(),
        )
    }
}

/// The texts of one culture and its fallbacks.
pub struct Catalog {
    /// The culture configured, as it was given.
    culture: String,
    /// The cultures tried, most specific first, English last.
    chain: Vec<Culture>,
    /// Every message with no arguments, formatted once through the chain.
    plain: HashMap<String, String>,
    /// Keys no culture in the chain has, and the bracketed name shown for each.
    missed: Mutex<BTreeMap<String, &'static str>>,
}

impl std::fmt::Debug for Catalog {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Catalog")
            .field("culture", &self.culture)
            .field("chain", &self.chain_names())
            .field("messages", &self.plain.len())
            .finish_non_exhaustive()
    }
}

impl Catalog {
    /// The catalog for a culture name as config.xml spells it (`de-DE`, `Fr`, `zh-TW`), with its
    /// fallbacks down to English.
    #[must_use]
    pub fn new(culture: &str) -> Self {
        let chain: Vec<Culture> = chain(culture).into_iter().map(Culture::new).collect();
        let mut plain = HashMap::new();
        for culture in &chain {
            for &(_, _, text) in FILES.iter().filter(|&&(name, _, _)| name == culture.name) {
                for id in ids(text) {
                    if plain.contains_key(id) {
                        continue;
                    }
                    if let Some(text) = culture.format(id, None) {
                        plain.insert(id.to_owned(), text);
                    }
                }
            }
        }
        Self {
            culture: culture.to_owned(),
            chain,
            plain,
            missed: Mutex::new(BTreeMap::new()),
        }
    }

    /// The culture configured.
    #[must_use]
    pub fn culture(&self) -> &str {
        &self.culture
    }

    /// The cultures tried, most specific first.
    #[must_use]
    pub fn chain_names(&self) -> Vec<&'static str> {
        self.chain.iter().map(|culture| culture.name).collect()
    }

    /// Whether a culture in the chain other than English has the key: whether it is translated.
    #[cfg(test)]
    #[must_use]
    pub fn translates(&self, id: &str) -> bool {
        self.chain
            .iter()
            .filter(|culture| culture.name != BASE)
            .any(|culture| culture.bundle.has_message(id))
    }

    /// Whether one culture's own files have the key, fallbacks aside: what the lint counts.
    #[cfg(test)]
    #[must_use]
    pub fn culture_has(culture: &str, id: &str) -> bool {
        FILES
            .iter()
            .filter(|&&(name, _, _)| name == culture)
            .any(|&(_, _, text)| ids(text).any(|found| found == id))
    }

    /// A key's text, from the first culture in the chain that has it; the key's name in brackets,
    /// remembered, when none has.
    #[must_use]
    pub fn text(&self, id: &str) -> &str {
        match self.plain.get(id) {
            Some(text) => text,
            None => self.miss(id),
        }
    }

    /// A key's text with its `$argN` filled, `{0}` in the C#'s `string.Format`. No word the
    /// flight screen reads through Fluent yet has a placeholder, so only the tests call it.
    #[cfg_attr(not(test), allow(dead_code))]
    #[must_use]
    pub fn format(&self, id: &str, args: &[(&str, &str)]) -> String {
        let mut fluent = FluentArgs::with_capacity(args.len());
        for &(name, value) in args {
            fluent.set(name, value.to_owned());
        }
        self.chain
            .iter()
            .find_map(|culture| culture.format(id, Some(&fluent)))
            .unwrap_or_else(|| self.miss(id).to_owned())
    }

    /// The bracketed name of a key nobody has, remembered once.
    fn miss(&self, id: &str) -> &'static str {
        let mut missed = self.missed.lock().unwrap_or_else(PoisonError::into_inner);
        missed
            .entry(id.to_owned())
            // One small string per distinct key the source asks for and no file has: bounded by
            // the program text, and the lint keeps it at zero.
            .or_insert_with(|| Box::leak(format!("[{id}]").into_boxed_str()))
    }

    /// The keys asked for that no culture in the chain has, English included.
    #[must_use]
    pub fn missed(&self) -> Vec<String> {
        self.missed
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .keys()
            .cloned()
            .collect()
    }

    /// What a status line may say about the culture: that a culture other than English was asked
    /// for and nothing here translates it, so the screens are in English.
    #[must_use]
    pub fn note(&self) -> Option<String> {
        let english = self.culture.is_empty()
            || self.culture.eq_ignore_ascii_case(BASE)
            || self
                .culture
                .to_ascii_lowercase()
                .starts_with(&format!("{BASE}-"));
        (!english && self.chain.len() == 1).then(|| {
            format!(
                "language {}: no translation here, the screens are in English",
                self.culture
            )
        })
    }
}

/// The catalog the screens read: English until [`init`] names another.
static CURRENT: OnceLock<Catalog> = OnceLock::new();

/// Sets the application's culture from config.xml's `language`, once, at start-up, before the
/// first screen asks for a word. `L10N`'s static constructor, and `MainV2`'s `changelanguage` at
/// start-up. A second call changes nothing and returns `false`.
/// `// C#: L10N.cs:12-25; MainV2.cs:697-700`
pub fn init(language: Option<&str>) -> bool {
    let mut set = false;
    let _ = CURRENT.get_or_init(|| {
        set = true;
        Catalog::new(&configured(language))
    });
    set
}

/// The catalog the screens read.
pub fn current() -> &'static Catalog {
    CURRENT.get_or_init(|| Catalog::new(BASE))
}

/// A key's text in the application's culture. [`fl!`] with no arguments.
#[must_use]
pub fn text(id: &str) -> &'static str {
    current().text(id)
}

/// A key's text in the application's culture with its `$argN` filled. [`fl!`] with arguments,
/// which no ported screen uses yet.
#[cfg_attr(not(test), allow(dead_code))]
#[must_use]
pub fn format(id: &str, args: &[(&str, &str)]) -> String {
    current().format(id, args)
}

/// Publishes the culture, the chain, the keys missed and the note (`none` without one), for a
/// UI test.
pub fn record_facts() {
    let catalog = current();
    crate::facts::record("i18n.culture", catalog.culture());
    crate::facts::record("i18n.chain", catalog.chain_names().join(","));
    crate::facts::record("i18n.missing", catalog.missed().len());
    crate::facts::record(
        "i18n.note",
        catalog.note().unwrap_or_else(|| "none".to_owned()),
    );
}

/// A screen's word through Fluent: `fl!("flightdata-BUT_ARM-Text")` is a `&'static str`,
/// `fl!("ErrorNoResponse", arg0 = count)` a `String` with `{ $arg0 }` filled. The key is a
/// literal, so the lint can find every one the screen asks for.
macro_rules! fl {
    ($id:literal) => {
        $crate::i18n::text($id)
    };
    ($id:literal, $($name:ident = $value:expr),+ $(,)?) => {
        $crate::i18n::format($id, &[$((stringify!($name), &*$value.to_string())),+])
    };
}
pub(crate) use fl;

/// The keys a source file asks for with `fl!`, in order, once each: the lint's scan.
#[cfg(test)]
#[must_use]
pub fn keys_in(source: &str) -> Vec<&str> {
    let mut found: Vec<&str> = Vec::new();
    let mut rest = source;
    while let Some(at) = rest.find("fl!(") {
        rest = rest.get(at + "fl!(".len()..).unwrap_or_default();
        let Some(quoted) = rest.trim_start().strip_prefix('"') else {
            continue;
        };
        if let Some((key, _)) = quoted.split_once('"')
            && !found.contains(&key)
        {
            found.push(key);
        }
    }
    found
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The flight screen's sources, as the lint reads them.
    const SOURCES: [(&str, &str); 3] = [
        ("fly.rs", include_str!("fly.rs")),
        ("payload.rs", include_str!("payload.rs")),
        ("transponder.rs", include_str!("transponder.rs")),
    ];

    /// Every key the flight screen asks for, once each.
    fn screen_keys() -> Vec<&'static str> {
        let mut keys: Vec<&'static str> = Vec::new();
        for (_, source) in SOURCES {
            for key in keys_in(source) {
                if !keys.contains(&key) {
                    keys.push(key);
                }
            }
        }
        keys
    }

    fn assets() -> std::path::PathBuf {
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../assets/i18n")
    }

    #[test]
    fn the_lint_reads_the_files_it_says_it_reads() {
        let named: Vec<&str> = SOURCES.iter().map(|&(name, _)| name).collect();
        assert_eq!(named, SCREEN_SOURCES);
    }

    #[test]
    fn the_embedded_files_are_every_file_in_assets() {
        let mut on_disk = Vec::new();
        for entry in std::fs::read_dir(assets()).expect("assets/i18n") {
            let path = entry.expect("entry").path();
            if !path.is_dir() {
                continue;
            }
            let culture = path.file_name().and_then(|n| n.to_str()).expect("name");
            for file in std::fs::read_dir(&path).expect("culture dir") {
                let file = file.expect("file").path();
                if let Some(stem) = file
                    .file_name()
                    .and_then(|n| n.to_str())
                    .and_then(|n| n.strip_suffix(".ftl"))
                {
                    on_disk.push((culture.to_owned(), stem.to_owned()));
                }
            }
        }
        on_disk.sort();
        let embedded: Vec<(String, String)> = FILES
            .iter()
            .map(|&(culture, stem, _)| (culture.to_owned(), stem.to_owned()))
            .collect();
        assert_eq!(embedded, on_disk, "FILES must list assets/i18n, sorted");
        for &(culture, stem, text) in FILES {
            let path = assets().join(culture).join(format!("{stem}.ftl"));
            assert_eq!(
                std::fs::read_to_string(&path).expect("read"),
                text,
                "{}",
                path.display()
            );
        }
    }

    #[test]
    fn every_embedded_file_parses_cleanly() {
        for &(culture, stem, text) in FILES {
            assert!(
                FluentResource::try_new(text.to_owned()).is_ok(),
                "{culture}/{stem}.ftl"
            );
        }
    }

    /// The lint: every key the flight screen asks for is in English, so none can show as its
    /// name in brackets.
    #[test]
    fn english_has_every_key_the_flight_screen_asks_for() {
        let keys = screen_keys();
        assert!(keys.len() >= 40, "{} keys: the scan lost them", keys.len());
        let english = Catalog::new(BASE);
        let lacking: Vec<&str> = keys
            .iter()
            .copied()
            .filter(|key| !Catalog::culture_has(BASE, key))
            .collect();
        assert!(lacking.is_empty(), "English lacks {lacking:?}");
        for key in &keys {
            assert!(!english.text(key).starts_with('['), "{key}");
        }
        assert!(english.missed().is_empty());
    }

    /// The flight screen's words in English are the `.resx`'s, unchanged: a few pinned.
    #[test]
    fn english_is_the_resx_text_exactly() {
        let english = Catalog::new(BASE);
        assert_eq!(english.text("flightdata-tabActions-Text"), "Actions");
        assert_eq!(english.text("flightdata-tabServo-Text"), "Servo/Relay");
        assert_eq!(
            english.text("flightdata-BUT_DFMavlink-Text"),
            "Download DataFlash Log Via Mavlink"
        );
        assert_eq!(
            english.text("flightdata-but_bintolog-Text"),
            "Convert .Bin to .Log"
        );
        assert_eq!(english.text("flightdata-BUT_speed1_10-Text"), "0.1");
        assert_eq!(english.text("flightdata-lbl_logpercent-Text"), "0.00 %");
    }

    /// What the screens show in a test run: English, since nothing calls `init`.
    #[test]
    fn the_screens_read_english_until_told_otherwise() {
        assert_eq!(fl!("flightdata-tabQuick-Text"), "Quick");
        assert_eq!(text("flightdata-BUT_ARM-Text"), "Arm/ Disarm");
    }

    /// `Strings.ErrorSetValueFailed`, `"Set {0} Failed"`: the placeholder filled, with nothing
    /// around it, through the function and the macro alike.
    #[test]
    fn a_placeholder_is_filled_and_nothing_isolates_it() {
        let english = Catalog::new(BASE);
        assert_eq!(
            english.format("ErrorSetValueFailed", &[("arg0", "RATE")]),
            "Set RATE Failed"
        );
        assert_eq!(
            english.format("Units", &[("arg0", "m"), ("arg1", "/s")]),
            "Units: m/s"
        );
        assert_eq!(fl!("ErrorSetValueFailed", arg0 = 7), "Set 7 Failed");
        assert_eq!(fl!("ErrorNoResponse"), "Error: no response from MAV");
    }

    #[test]
    fn a_key_nobody_has_is_its_name_in_brackets_and_counted() {
        let english = Catalog::new(BASE);
        assert_eq!(
            english.text("flightdata-Nope-Text"),
            "[flightdata-Nope-Text]"
        );
        assert_eq!(
            english.text("flightdata-Nope-Text"),
            "[flightdata-Nope-Text]"
        );
        assert_eq!(english.format("Nope", &[("arg0", "1")]), "[Nope]");
        assert_eq!(english.missed(), ["Nope", "flightdata-Nope-Text"]);
    }

    #[test]
    fn a_culture_falls_back_to_its_parents_then_english() {
        assert_eq!(chain("de-DE"), ["de-DE", "en"]);
        assert_eq!(chain("de-AT"), ["en"]);
        assert_eq!(chain("Fr"), ["fr", "en"]);
        assert_eq!(chain("fr-FR"), ["fr", "en"]);
        assert_eq!(chain("zh-TW"), ["zh-TW", "zh-Hant", "en"]);
        assert_eq!(chain("zh-HK"), ["zh-Hant", "en"]);
        assert_eq!(chain("zh-CN"), ["zh-Hans", "en"]);
        assert_eq!(chain("en-US"), ["en"]);
        assert_eq!(chain(""), ["en"]);
        assert_eq!(chain("az-Latn-AZ"), ["az-Latn-AZ", "en"]);

        // de-DE translates Actions; a key it leaves out is English.
        let german = Catalog::new("de-DE");
        assert!(german.translates("flightdata-tabActions-Text"));
        assert_ne!(german.text("flightdata-tabActions-Text"), "Actions");
        let english = Catalog::new(BASE);
        let untranslated = screen_keys()
            .into_iter()
            .find(|key| !Catalog::culture_has("de-DE", key))
            .expect("de-DE leaves a flight screen key out");
        assert_eq!(german.text(untranslated), english.text(untranslated));
        assert!(german.missed().is_empty());
        assert_eq!(german.note(), None);
    }

    #[test]
    fn the_culture_is_the_setting_or_the_system() {
        assert_eq!(configured(Some("de-DE")), "de-DE");
        assert_eq!(configured(Some(" Fr ")), "Fr");
        // The owner's ruling: an empty or absent key is English, whatever the machine says.
        assert_eq!(configured(None), BASE);
        assert_eq!(configured(Some("  ")), BASE);
        assert_eq!(posix_culture("de_DE.UTF-8"), "de-DE");
        assert_eq!(posix_culture("sr_RS@latin"), "sr-RS");
        assert_eq!(posix_culture("C.UTF-8"), "");
        assert_eq!(posix_culture("POSIX"), "");
        assert_eq!(
            Catalog::new("xx-YY").note().as_deref(),
            Some("language xx-YY: no translation here, the screens are in English")
        );
        assert_eq!(Catalog::new("en-GB").note(), None);
    }

    #[test]
    fn keys_are_found_however_the_call_is_wrapped() {
        let source = "a(fl!(\"one\")); b(crate::i18n::fl!(\n    \"two\",\n)); fl!(\"one\"); \
                      fl!(\"three\", arg0 = x); fl!(not_a_literal)";
        assert_eq!(keys_in(source), ["one", "two", "three"]);
    }

    /// The flight screen's row of `assets/i18n/report.md`'s table for one culture: how many of
    /// the screen's keys the culture's own files lack.
    fn reported(culture: &str) -> Option<usize> {
        let report = std::fs::read_to_string(assets().join("report.md")).expect("report.md");
        let section = report.split("## Screens through Fluent").nth(1)?;
        section.lines().find_map(|line| {
            let cells: Vec<&str> = line.split('|').map(str::trim).collect();
            (cells.get(1) == Some(&culture))
                .then(|| cells.get(2).and_then(|n| n.parse().ok()))
                .flatten()
        })
    }

    /// Reports - never fails on - how many of the flight screen's keys a culture's own files
    /// lack, and holds the count to `report.md`'s, which `codegen-resx` writes.
    fn report_culture(culture: &str) {
        let keys = screen_keys();
        let lacking: Vec<&str> = keys
            .iter()
            .copied()
            .filter(|key| !Catalog::culture_has(culture, key))
            .collect();
        println!(
            "{culture}: lacks {} of the flight screen's {} keys{}",
            lacking.len(),
            keys.len(),
            if lacking.is_empty() {
                String::new()
            } else {
                format!(" ({})", lacking.join(", "))
            }
        );
        // English lacking one is the other test's failure; here the report is only held current.
        assert_eq!(
            reported(culture),
            Some(lacking.len()),
            "assets/i18n/report.md's flight screen row for {culture} is stale; run `cargo xtask \
             codegen-resx`"
        );
    }

    macro_rules! per_culture {
        ($($test:ident => $culture:literal),+ $(,)?) => {
            $(
                #[test]
                fn $test() {
                    report_culture($culture);
                }
            )+

            #[test]
            fn every_culture_has_its_report_test() {
                assert_eq!(cultures(), [$($culture),+]);
            }
        };
    }

    per_culture! {
        lacking_in_ar => "ar",
        lacking_in_az_latn_az => "az-Latn-AZ",
        lacking_in_de_de => "de-DE",
        lacking_in_en => "en",
        lacking_in_es_es => "es-ES",
        lacking_in_fr => "fr",
        lacking_in_id_id => "id-ID",
        lacking_in_it_it => "it-IT",
        lacking_in_ja_jp => "ja-JP",
        lacking_in_ko_kr => "ko-KR",
        lacking_in_pl => "pl",
        lacking_in_pt => "pt",
        lacking_in_ru_kz => "ru-KZ",
        lacking_in_ru_ru => "ru-RU",
        lacking_in_tr => "tr",
        lacking_in_uk => "uk",
        lacking_in_zh_hans => "zh-Hans",
        lacking_in_zh_hant => "zh-Hant",
        lacking_in_zh_tw => "zh-TW",
    }
}
