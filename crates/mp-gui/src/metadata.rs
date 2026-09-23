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

use std::collections::BTreeMap;
use std::sync::RwLock;
use std::sync::mpsc::{Receiver, channel};

use mp_params::ParamMeta;
use mp_params::pdef::{self, Pdef, PdefParam, Version};

/// The loaded documentation, if any.
struct Loaded {
    /// What it is: `Copter4.5.7` or `ArduCopter`, the file's stem.
    source: String,
    table: BTreeMap<String, &'static ParamMeta>,
}

static LOADED: RwLock<Option<Loaded>> = RwLock::new(None);

/// Documentation for one parameter: the fetched file's if it has it, else the bundled table's.
#[must_use]
pub fn lookup(name: &str) -> Option<&'static ParamMeta> {
    let loaded = LOADED.read().ok().and_then(|guard| {
        guard
            .as_ref()
            .and_then(|loaded| loaded.table.get(name).copied())
    });
    loaded.or_else(|| mp_params::param_meta::lookup(name))
}

/// Where the documentation currently comes from: `bundled`, or the fetched file's name.
#[must_use]
pub fn source() -> String {
    LOADED
        .read()
        .ok()
        .and_then(|guard| guard.as_ref().map(|loaded| loaded.source.clone()))
        .unwrap_or_else(|| "bundled".to_owned())
}

/// How many parameters the fetched file documents; zero while only the bundled table is in use.
#[must_use]
pub fn documented() -> usize {
    LOADED
        .read()
        .ok()
        .and_then(|guard| guard.as_ref().map(|loaded| loaded.table.len()))
        .unwrap_or(0)
}

/// Makes a fetched file the first source. Returns how many parameters it documents.
pub fn install(source: impl Into<String>, pdef: &Pdef) -> usize {
    let table: BTreeMap<String, &'static ParamMeta> = pdef
        .params()
        .map(|param| (param.name.clone(), leak(param)))
        .collect();
    let count = table.len();
    if let Ok(mut guard) = LOADED.write() {
        *guard = Some(Loaded {
            source: source.into(),
            table,
        });
    }
    count
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
                }
                Ok(Err(why)) => {
                    self.status = Some(why);
                    self.receiver = None;
                }
                Err(std::sync::mpsc::TryRecvError::Empty) => {}
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
        let (sender, receiver) = channel();
        self.receiver = Some(receiver);
        self.status = Some(format!("fetching documentation for {version}"));
        let versioned = pdef::versioned_vehicle(mav_type).map(str::to_owned);
        let unversioned = pdef::vehicle(mav_type).map(str::to_owned);
        std::thread::Builder::new()
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
        let count = install("Copter9.9.9", &pdef);
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
}
