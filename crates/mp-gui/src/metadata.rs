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

//! Which parameter documentation the screens read.
//!
//! Two sources, in order: the file ArduPilot generates for the firmware actually connected,
//! fetched and cached as Mission Planner fetches it (`mp_params::pdef`), and the bundled table
//! (`mp_params::param_meta`) for anything the file does not have or before it has arrived.
//! PLAN.md §10.5 is why: the bundled snapshot documented 59% of a live vehicle's parameters and
//! got a few ranges wrong, and a range that disagrees with the firmware refuses values the
//! vehicle would accept.
//!
//! The fetched documentation is turned into `&'static ParamMeta`s by leaking it. Deliberate:
//! every screen holds `Option<&'static ParamMeta>`, the table is loaded once per firmware seen
//! in a session, and a session sees one or two firmwares. A few megabytes that live until exit
//! are cheaper than threading lifetimes through three screens for the sake of freeing them at
//! exit anyway.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use mp_os::ReadWrite as _;
use std::collections::BTreeMap;
use std::sync::RwLock;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::mpsc::{Receiver, channel};

use mp_params::ParamMeta;
use mp_params::pdef::{self, Pdef, PdefParam, Version};

/// The loaded documentation, if any.
struct Loaded {
    /// What it is: `Copter4.5.7` or `ArduCopter`, the file's stem.
    source: String,
    table: BTreeMap<String, &'static ParamMeta>,
    // ---- ConfigRawParams remainder ----
    /// `<field name="ReadOnly">`'s text, by name, for the parameters the file gives one:
    /// [`ParamMeta`] has no such field.
    read_only: BTreeMap<String, String>,
    // ---- end ConfigRawParams remainder ----
}

/// The SITL and AP_Periph files' ReadOnly marks, in that order, once fetched: the C#'s
/// `GetParameterMetaData` asks the vehicle's file, then `SITL`'s, then `AP_Periph`'s, each
/// loaded (and fetched) on demand by `CheckLoad`; here both are fetched once the vehicle's
/// documentation is in, so a ReadOnly question finds them ready.
/// `// C#: ExtLibs/Utilities/ParameterMetaDataRepository.cs:42-46; ParameterMetaDataRepositoryAPMpdef.cs:46-152, 196-206`
static FALLBACK_READ_ONLY: RwLock<Vec<(String, BTreeMap<String, String>)>> =
    RwLock::new(Vec::new());

static LOADED: RwLock<Option<Loaded>> = RwLock::new(None);

/// How many times documentation has been installed; see [`generation`].
static GENERATION: AtomicU64 = AtomicU64::new(0);

/// Documentation for one parameter: the fetched file's if it has it, else the bundled table's.
///
/// Mission Planner's rule for both: the parameter named exactly. In the fetched file, the first
/// of that name in the file's order (`// C#: ExtLibs/Utilities/ParameterMetaDataRepositoryAPMpdef.cs:219-231`,
/// kept by [`Pdef::parse`]); in the bundled table, the entry of that name
/// ([`mp_params::param_meta::lookup`]). Both are searches, not scans.
#[must_use]
pub fn lookup(name: &str) -> Option<&'static ParamMeta> {
    let guard = LOADED.os_read().ok();
    lookup_in(
        guard
            .as_ref()
            .and_then(|guard| guard.as_ref())
            .map(|loaded| &loaded.table),
        name,
    )
}

/// [`lookup`] against a given fetched table rather than the installed one.
fn lookup_in(
    fetched: Option<&BTreeMap<String, &'static ParamMeta>>,
    name: &str,
) -> Option<&'static ParamMeta> {
    fetched
        .and_then(|table| table.get(name).copied())
        .or_else(|| mp_params::param_meta::lookup(name))
}

// ---- ConfigRawParams remainder ----
/// `GetParameterMetaData(name, ParameterMetaDataConstants.ReadOnly, firmware)`: the fetched
/// file's `<field name="ReadOnly">` text for the parameter, else the bundled
/// `ParameterMetaDataBackup.xml`'s mark (`ParameterMetaDataRepositoryAPM`, the C#'s last
/// fallback, as "True"), else `None`, the C#'s empty string. The two fallbacks between - the
/// SITL and AP_Periph vehicles' own `apm.pdef.xml` files - are not fetched here.
/// `// C#: ExtLibs/Utilities/ParameterMetaDataRepository.cs:27-67; GCSViews/ConfigurationView/ConfigRawParams.cs:480-483`
#[must_use]
pub fn read_only(name: &str) -> Option<String> {
    let fetched = LOADED.os_read().ok().and_then(|guard| {
        guard
            .as_ref()
            .and_then(|loaded| loaded.read_only.get(name).cloned())
    });
    let fallbacks = FALLBACK_READ_ONLY.os_read().ok();
    read_only_in(
        fetched,
        fallbacks.as_deref().map_or(&[], Vec::as_slice),
        name,
    )
}

/// [`read_only`]'s chain over given tables: the vehicle's answer, else each fallback file's
/// in order, else the bundled file's mark.
fn read_only_in(
    fetched: Option<String>,
    fallbacks: &[(String, BTreeMap<String, String>)],
    name: &str,
) -> Option<String> {
    fetched
        .or_else(|| {
            fallbacks
                .iter()
                .find_map(|(_, marks)| marks.get(name).cloned())
        })
        .or_else(|| mp_params::param_meta::read_only_backup(name).then(|| "True".to_owned()))
}

/// The two fallback files' ReadOnly marks, fetched (the unversioned `SITL` and `AP_Periph`
/// documentation, as `CheckLoad` fetches them) and kept for [`read_only`]. On a thread; a file
/// that cannot be had is left out, as the C# logs and goes on.
fn fetch_fallbacks() {
    wasm_thread::Builder::new()
        .name("mp-metadata-fallbacks".to_owned())
        .spawn(|| {
            let Some(dir) = mp_settings::data_directory() else {
                return;
            };
            let mut found = Vec::new();
            for vehicle in ["SITL", "AP_Periph"] {
                let Ok(path) = pdef::fetch_unversioned(&dir, vehicle) else {
                    continue;
                };
                let Ok(pdef) = Pdef::load(&path) else {
                    continue;
                };
                let marks: BTreeMap<String, String> = pdef
                    .params()
                    .filter_map(|param| Some((param.name.clone(), param.read_only.clone()?)))
                    .collect();
                found.push((vehicle.to_owned(), marks));
            }
            if let Ok(mut guard) = FALLBACK_READ_ONLY.os_write() {
                *guard = found;
            }
        })
        .ok();
}
// ---- end ConfigRawParams remainder ----

/// Which documentation [`lookup`] is answering from: a number that moves each time a fetched
/// file is installed, and only then.
///
/// A screen that keeps what it looked up - the parameter list, fourteen hundred lookups - keeps
/// it while this is unchanged and looks again when it moves, so a file that arrives mid-session
/// is shown on the next frame.
#[must_use]
pub fn generation() -> u64 {
    GENERATION.load(Ordering::Acquire)
}

/// Where the documentation currently comes from: `bundled`, or the fetched file's name.
#[must_use]
pub fn source() -> String {
    LOADED
        .os_read()
        .ok()
        .and_then(|guard| guard.as_ref().map(|loaded| loaded.source.clone()))
        .unwrap_or_else(|| "bundled".to_owned())
}

/// How many parameters the fetched file documents; zero while only the bundled table is in use.
#[must_use]
pub fn documented() -> usize {
    LOADED
        .os_read()
        .ok()
        .and_then(|guard| guard.as_ref().map(|loaded| loaded.table.len()))
        .unwrap_or(0)
}

/// Makes a fetched file the first source. Returns how many parameters it documents.
pub fn install(source: impl Into<String>, pdef: &Pdef) -> usize {
    let table = table_of(pdef);
    let count = table.len();
    if let Ok(mut guard) = LOADED.os_write() {
        *guard = Some(Loaded {
            source: source.into(),
            table,
            // ---- ConfigRawParams remainder ----
            read_only: pdef
                .params()
                .filter_map(|param| Some((param.name.clone(), param.read_only.clone()?)))
                .collect(),
            // ---- end ConfigRawParams remainder ----
        });
    }
    // After the table is in place, never before: a reader that sees the new number must find
    // the new table, or it would keep what it built from the old one as if it were current.
    GENERATION.fetch_add(1, Ordering::Release);
    count
}

/// A fetched file's documentation, by name.
fn table_of(pdef: &Pdef) -> BTreeMap<String, &'static ParamMeta> {
    pdef.params()
        .map(|param| (param.name.clone(), leak(param)))
        .collect()
}

fn leak_str(text: &str) -> &'static str {
    Box::leak(text.to_owned().into_boxed_str())
}

fn leak(param: &PdefParam) -> &'static ParamMeta {
    let values: Vec<(i64, &'static str)> = param
        .values
        .iter()
        .map(|(code, label)| (*code, leak_str(label)))
        .collect();
    let bitmask: Vec<(u32, &'static str)> = param
        .bitmask
        .iter()
        .map(|(bit, label)| (*bit, leak_str(label)))
        .collect();
    Box::leak(Box::new(ParamMeta {
        name: leak_str(&param.name),
        display_name: leak_str(&param.display_name),
        description: leak_str(&param.description),
        units: leak_str(&param.units),
        range: param.range,
        range_text: leak_str(&param.range_text),
        increment: param.increment,
        values: Box::leak(values.into_boxed_slice()),
        bitmask: Box::leak(bitmask.into_boxed_slice()),
        user_level: param.user_level,
        reboot_required: param.reboot_required,
    }))
}

/// The fetch in flight, if any, and what it is for.
///
/// Fetching goes to the network and reads a multi-megabyte file, so it runs on a thread and the
/// screen collects the result on a later frame. One fetch per firmware: the key is the banner
/// text, and a banner already fetched for is not fetched again.
#[derive(Debug, Default)]
pub struct Fetch {
    requested: Option<String>,
    receiver: Option<Receiver<Result<(String, Pdef), String>>>,
    /// The last thing that happened, for the screen.
    pub status: Option<String>,
    /// Whether the SITL and AP_Periph fallback files have been asked for: once a process.
    fallbacks_asked: bool,
}

impl Fetch {
    /// Advances the fetch once a frame: starts one for a banner not yet fetched for, and
    /// installs a result that has arrived.
    ///
    /// The order is the C#'s: the versioned file for the release the banner names, else the
    /// vehicle's unversioned file. Mission Planner fetches the unversioned files for every
    /// vehicle at startup and the versioned ones for every versioned vehicle when a banner
    /// arrives; here only the connected vehicle's, which is the one that is read.
    /// `// C#: ExtLibs/Utilities/ParameterMetaDataRepositoryAPMpdef.cs:38-41, 52-90; MainV2.cs:1721`
    pub fn advance(&mut self, banner: Option<&str>, mav_type: u8) {
        if let Some(receiver) = &self.receiver {
            match receiver.try_recv() {
                Ok(Ok((source, pdef))) => {
                    let count = install(source.clone(), &pdef);
                    self.status = Some(format!("{source}: {count} parameters documented"));
                    self.receiver = None;
                    if !self.fallbacks_asked {
                        self.fallbacks_asked = true;
                        fetch_fallbacks();
                    }
                }
                Ok(Err(why)) => {
                    self.status = Some(why);
                    self.receiver = None;
                }
                Err(std::sync::mpsc::TryRecvError::Empty) => crate::repaint::in_flight(),
                Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                    self.receiver = None;
                }
            }
            return;
        }
        let Some(banner) = banner else {
            return;
        };
        if self.requested.as_deref() == Some(banner) {
            return;
        }
        self.requested = Some(banner.to_owned());
        let Some(version) = Version::from_banner(banner) else {
            self.status = Some(format!("no version in the banner: {banner}"));
            return;
        };
        let versioned = pdef::versioned_vehicle(mav_type).map(str::to_owned);
        let unversioned = pdef::vehicle(mav_type).map(str::to_owned);
        if versioned.is_none() && unversioned.is_none() {
            // The banner can be heard before the heartbeat's type is in the view (the connect is
            // quick now): no names to fetch by yet, so ask again next frame rather than record
            // a fetch that never was (found 2026-09-25: "no documentation for this firmware: ").
            self.requested = None;
            return;
        }
        let (sender, receiver) = channel();
        self.receiver = Some(receiver);
        self.status = Some(format!("fetching documentation for {version}"));
        wasm_thread::Builder::new()
            .name("mp-metadata".to_owned())
            .spawn(move || {
                let _ = sender.send(fetch(versioned.as_deref(), unversioned.as_deref(), version));
            })
            .ok();
    }
}

/// The versioned file if the release has one, else the vehicle's unversioned file.
fn fetch(
    versioned: Option<&str>,
    unversioned: Option<&str>,
    version: Version,
) -> Result<(String, Pdef), String> {
    let Some(dir) = mp_settings::data_directory() else {
        return Err("no data directory to keep documentation in".to_owned());
    };
    let mut failures = Vec::new();
    if let Some(vehicle) = versioned {
        match pdef::fetch_versioned(&dir, vehicle, version) {
            Ok(path) => match Pdef::load(&path) {
                Ok(pdef) => return Ok((format!("{vehicle}{version}"), pdef)),
                Err(err) => failures.push(err.to_string()),
            },
            Err(err) => failures.push(err.to_string()),
        }
    }
    if let Some(vehicle) = unversioned {
        match pdef::fetch_unversioned(&dir, vehicle) {
            Ok(path) => match Pdef::load(&path) {
                Ok(pdef) => return Ok((vehicle.to_owned(), pdef)),
                Err(err) => failures.push(err.to_string()),
            },
            Err(err) => failures.push(err.to_string()),
        }
    }
    Err(format!(
        "no documentation for this firmware: {}",
        failures.join("; ")
    ))
}

/// The banner in a run of messages: the first `STATUSTEXT` that names a vehicle.
///
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1822-1830`
#[must_use]
pub fn banner_in<'a>(messages: impl IntoIterator<Item = &'a str>) -> Option<&'a str> {
    messages.into_iter().find(|text| {
        let lower = text.to_ascii_lowercase();
        lower.contains("copter") || lower.contains("rover") || lower.contains("plane")
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    const SAMPLE: &str = r#"<paramfile><vehicles><parameters name="ArduCopter">
      <param humanName="From the file" name="ArduCopter:ZZ_TEST_ONLY" documentation="fetched" user="Standard">
        <field name="Range">1 9</field>
      </param>
      <param humanName="Overrides the bundled entry" name="ArduCopter:ACRO_RP_EXPO" documentation="fetched" user="Advanced">
        <field name="Range">-1 1</field>
      </param>
    </parameters></vehicles></paramfile>"#;

    /// The fetched file wins where it documents a parameter, and the bundled table fills the
    /// rest - so a firmware newer than the bundle loses nothing the bundle had.
    #[test]
    #[allow(clippy::unwrap_used)]
    fn the_fetched_file_comes_first_and_the_bundle_fills_in() {
        let pdef = Pdef::parse(SAMPLE).unwrap();
        let before = generation();
        let count = install("Copter9.9.9", &pdef);
        assert!(generation() > before, "an install moves the generation");
        assert_eq!(count, 2);
        assert_eq!(source(), "Copter9.9.9");
        assert_eq!(documented(), 2);
        let fetched = lookup("ZZ_TEST_ONLY").expect("from the file");
        assert_eq!(fetched.description, "fetched");
        assert_eq!(fetched.range, Some((1.0, 9.0)));
        let overridden = lookup("ACRO_RP_EXPO").expect("in both");
        assert_eq!(
            overridden.range,
            Some((-1.0, 1.0)),
            "the file's range, not the bundle's"
        );
        let bundled = lookup("BATT_MONITOR").expect("only the bundle has it");
        assert!(!bundled.description.is_empty());
    }

    /// Over the file Mission Planner fetched onto this machine, when it is here, and without it:
    /// every one of SITL's 1,408 names is documented by the entry of exactly its own name - the
    /// file's where the file has it, the bundle's otherwise - or by nothing when neither has it.
    #[test]
    fn every_sitl_name_is_documented_by_its_own_name_with_or_without_the_real_file() {
        let fixture = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata/params/sitl-copter.param");
        let text = mp_os::fs::read_to_string(&fixture).expect("the SITL dump");
        let names: Vec<&str> = text
            .lines()
            .filter_map(|line| line.split(',').next())
            .collect();
        assert_eq!(names.len(), 1408);
        let bundled = |name: &str| {
            mp_params::param_meta::copter::PARAMETERS
                .iter()
                .find(|meta| meta.name == name)
        };
        let same = |left: Option<&'static ParamMeta>, right: Option<&'static ParamMeta>| match (
            left, right,
        ) {
            (Some(left), Some(right)) => std::ptr::eq(left, right),
            (None, None) => true,
            _ => false,
        };

        for name in &names {
            assert!(same(lookup_in(None, name), bundled(name)), "{name}");
        }

        let Some(dir) = mp_settings::data_directory() else {
            eprintln!("skipped the fetched file: no home directory");
            return;
        };
        let path = pdef::unversioned_path(&dir, "ArduCopter");
        let Ok(pdef) = Pdef::load(&path) else {
            eprintln!("skipped the fetched file: no {}", path.display());
            return;
        };
        let table = table_of(&pdef);
        let mut from_file = 0;
        for name in &names {
            let expected = table.get(*name).copied().or_else(|| bundled(name));
            assert!(same(lookup_in(Some(&table), name), expected), "{name}");
            if let Some(found) = lookup_in(Some(&table), name) {
                assert_eq!(found.name, *name);
            }
            from_file += usize::from(table.contains_key(*name));
        }
        eprintln!(
            "{}: {from_file} of {} SITL names from the file, the rest from the bundle",
            path.display(),
            names.len()
        );
        assert!(from_file > 1000);
    }

    #[test]
    fn the_banner_is_the_first_message_naming_a_vehicle() {
        let messages = [
            "ChibiOS: 6a85082c",
            "ArduCopter V4.5.7 (1c0c8d9c)",
            "RCOut: PWM:1-12",
        ];
        assert_eq!(banner_in(messages), Some("ArduCopter V4.5.7 (1c0c8d9c)"));
        assert_eq!(banner_in(["PreArm: Compass not calibrated"]), None);
        assert_eq!(banner_in(["APM:Rover V4.5.0"]), Some("APM:Rover V4.5.0"));
    }

    /// A fetch is started once per banner and never again for the same one.
    #[test]
    fn a_fetch_is_started_once_per_banner() {
        let mut fetch = Fetch::default();
        fetch.advance(None, 2);
        assert!(fetch.receiver.is_none());
        fetch.advance(Some("no version here"), 2);
        assert!(
            fetch.receiver.is_none(),
            "an unparsable banner starts nothing"
        );
        assert!(fetch.status.as_deref().unwrap_or("").contains("no version"));
    }

    /// `GetParameterMetaData(name, ReadOnly, ...)`: the vehicle's file first, then SITL's, then
    /// AP_Periph's, then the bundled file's mark.
    /// `// C#: ExtLibs/Utilities/ParameterMetaDataRepository.cs:42-49`
    #[test]
    fn read_only_falls_back_through_the_files_in_order() {
        let sitl: BTreeMap<String, String> = [("SIM_ONLY".to_owned(), "True".to_owned())]
            .into_iter()
            .collect();
        let periph: BTreeMap<String, String> = [
            ("SIM_ONLY".to_owned(), "False".to_owned()),
            ("PERIPH_ONLY".to_owned(), "True".to_owned()),
        ]
        .into_iter()
        .collect();
        let fallbacks = vec![("SITL".to_owned(), sitl), ("AP_Periph".to_owned(), periph)];
        assert_eq!(
            read_only_in(Some("False".to_owned()), &fallbacks, "SIM_ONLY").as_deref(),
            Some("False"),
            "the vehicle's file answers first"
        );
        assert_eq!(
            read_only_in(None, &fallbacks, "SIM_ONLY").as_deref(),
            Some("True"),
            "SITL's before AP_Periph's"
        );
        assert_eq!(
            read_only_in(None, &fallbacks, "PERIPH_ONLY").as_deref(),
            Some("True")
        );
        assert_eq!(
            read_only_in(None, &fallbacks, "BARO1_DEVID").as_deref(),
            Some("True"),
            "the bundled file's mark last"
        );
        assert_eq!(read_only_in(None, &fallbacks, "ATC_RAT_RLL_P"), None);
        assert_eq!(read_only_in(None, &[], "NOT_A_PARAM"), None);
    }
}
