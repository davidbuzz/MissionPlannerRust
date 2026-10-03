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

//! Parameter metadata for the firmware actually flying: `apm.pdef.xml`.
//!
//! Ported from `ExtLibs/Utilities/ParameterMetaDataRepositoryAPMpdef.cs` @ efb0801
//! (GPL-3.0-only) and `VersionDetection.cs`. The bundled table in [`crate::param_meta`] is
//! a snapshot of one release; PLAN.md §10.5 measured 41% of a live vehicle's parameters missing
//! from it and a handful documented with the wrong range. Mission Planner does not live with
//! that: it downloads the documentation ArduPilot generates for the exact release the vehicle
//! reports, keeps it beside its other data, and reads that first. This module is that download
//! and that file.
//!
//! Two files per vehicle, exactly where the C# keeps them (`Settings.GetDataDirectory()`):
//!
//! - `<Vehicle><major.minor.patch>.apm.pdef.xml` from
//!   `https://autotest.ardupilot.org/Parameters/versioned/<Vehicle>/stable-<version>/apm.pdef.xml`
//!   for the five vehicles that have versioned documentation, fetched once and kept for good;
//! - `<Vehicle>.apm.pdef.xml.gz` from `https://autotest.ardupilot.org/Parameters/<Vehicle>/`,
//!   unpacked beside itself and refreshed when a week old.
//!
//! The version comes from the vehicle's banner - the `STATUSTEXT` that reads
//! `ArduCopter V4.5.7 (1c0c8d9c)` - parsed by the C#'s regular expression, which is stranger
//! than it looks: `4.6.0-rc1` becomes 4.6.0.1 and `4.8.0-dev` becomes 4.8.0.255. Ported as it is.
//! `AUTOPILOT_VERSION.flight_sw_version` carries the same numbers packed, and is decoded here
//! too, for a vehicle that never sends its banner.

use std::collections::BTreeMap;
use std::io::Read;
use std::path::{Path, PathBuf};
use std::time::Duration;

use crate::param_meta::UserLevel;

/// The vehicles with versioned documentation, by the name the URL and the file use.
/// `// C#: ExtLibs/Utilities/ParameterMetaDataRepositoryAPMpdef.cs:29-32`
pub const VERSIONED_VEHICLES: &[&str] = &["Copter", "Plane", "Rover", "Sub", "Tracker"];

/// Every vehicle with unversioned documentation, by the name the URL and the file use.
/// `// C#: ExtLibs/Utilities/ParameterMetaDataRepositoryAPMpdef.cs:25-28`
pub const VEHICLES: &[&str] = &[
    "SITL",
    "AP_Periph",
    "ArduSub",
    "Rover",
    "ArduCopter",
    "ArduPlane",
    "AntennaTracker",
    "Blimp",
    "Heli",
];

/// How long an unversioned file is trusted before it is fetched again.
/// `// C#: ExtLibs/Utilities/ParameterMetaDataRepositoryAPMpdef.cs:125`
pub const REFRESH_AFTER: Duration = Duration::from_secs(7 * 24 * 60 * 60);

/// A firmware version as `System.Version` holds it: up to four numbers.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct Version {
    /// Major.
    pub major: u32,
    /// Minor.
    pub minor: u32,
    /// Patch, if the banner carried one.
    pub build: Option<u32>,
    /// The fourth number the C# derives from a suffix: the `-rcN` number, 255 for a word like
    /// `dev` or `beta`, or a letter's position for a single-letter suffix.
    pub revision: Option<u32>,
}

impl Version {
    /// A plain three-part version.
    #[must_use]
    pub const fn new(major: u32, minor: u32, build: u32) -> Self {
        Self {
            major,
            minor,
            build: Some(build),
            revision: None,
        }
    }

    /// The version as the C# parses it from a banner such as `ArduCopter V4.5.7 (1c0c8d9c)`.
    ///
    /// `VersionDetection.GetVersion`: `major.minor`, then `.patch`, then for a suffix
    /// `-rcN` the number N as a fourth part, for two or more letters (`dev`, `beta`) 255, and
    /// for a single letter its position in the alphabet - the C# subtracts 0x30 from the
    /// character, so `a` gives 49; that arithmetic is kept. `None` where the C# throws.
    /// `// C#: ExtLibs/Utilities/VersionDetection.cs:15-56`
    #[must_use]
    pub fn from_banner(text: &str) -> Option<Self> {
        let bytes = text.as_bytes();
        // Find the first `digits.digits`.
        let mut start = 0;
        loop {
            let digits = |from: usize| {
                let end = bytes.get(from..).map_or(from, |rest| {
                    from + rest.iter().take_while(|b| b.is_ascii_digit()).count()
                });
                (end > from).then_some(end)
            };
            let Some(major_end) = digits(start) else {
                start = bytes.get(start..)?.iter().position(u8::is_ascii_digit)? + start;
                continue;
            };
            if bytes.get(major_end) != Some(&b'.') {
                start = major_end;
                continue;
            }
            let Some(minor_end) = digits(major_end + 1) else {
                start = major_end + 1;
                continue;
            };
            let major = text.get(start..major_end)?.parse().ok()?;
            let minor = text.get(major_end + 1..minor_end)?.parse().ok()?;
            let mut version = Self {
                major,
                minor,
                build: None,
                revision: None,
            };
            // The repeated group: `.patch`, `-rcN`, a word, or a letter, as many as match.
            let mut at = minor_end;
            loop {
                if bytes.get(at) == Some(&b'.')
                    && let Some(end) = digits(at + 1)
                {
                    version.build = text.get(at + 1..end)?.parse().ok();
                    at = end;
                } else if text.get(at..)?.starts_with("-rc")
                    && let Some(end) = digits(at + 3)
                {
                    version.revision = text.get(at + 3..end)?.parse().ok();
                    at = end;
                } else {
                    let letters = bytes.get(at..).map_or(0, |rest| {
                        rest.iter().take_while(|b| b.is_ascii_lowercase()).count()
                    });
                    if letters >= 2 {
                        version.revision = Some(255);
                        at += letters.min(20);
                    } else if letters == 1 {
                        version.revision = Some(u32::from(*bytes.get(at)?) - 0x30);
                        at += 1;
                    } else {
                        break;
                    }
                }
            }
            return Some(version);
        }
    }

    /// The version packed in `AUTOPILOT_VERSION.flight_sw_version`: major, minor, patch, type.
    ///
    /// Not what the C# feeds its metadata fetch - that is the banner - but the same numbers,
    /// for a vehicle that never sent one.
    #[must_use]
    pub const fn from_flight_sw_version(packed: u32) -> Self {
        Self {
            major: (packed >> 24) & 0xff,
            minor: (packed >> 16) & 0xff,
            build: Some((packed >> 8) & 0xff),
            revision: None,
        }
    }
}

impl std::fmt::Display for Version {
    /// As `System.Version.ToString()` writes it: only the parts that are set.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}.{}", self.major, self.minor)?;
        if let Some(build) = self.build {
            write!(f, ".{build}")?;
        }
        if let Some(revision) = self.revision {
            write!(f, ".{revision}")?;
        }
        Ok(())
    }
}

/// The versioned vehicle name for a `MAV_TYPE`, if that vehicle has versioned documentation.
///
/// `// C#: MainV2.cs:1721` feeds every versioned vehicle at once; the file that is then read is
/// the one for the connected vehicle's firmware, which `Firmwares` maps from `MAV_TYPE`.
#[must_use]
pub const fn versioned_vehicle(mav_type: u8) -> Option<&'static str> {
    // MAV_TYPE: 1 fixed wing, 2 quad, 3 coax, 4 heli, 5 antenna tracker, 10 rover, 11 boat,
    // 12 submarine, 13 hexa, 14 octo, 15 tricopter, 19 VTOL..., 29 dodecarotor.
    match mav_type {
        1 | 16 | 19..=25 => Some("Plane"),
        2 | 3 | 4 | 13 | 14 | 15 | 29 => Some("Copter"),
        5 => Some("Tracker"),
        10 | 11 => Some("Rover"),
        12 => Some("Sub"),
        _ => None,
    }
}

/// The unversioned vehicle name for a `MAV_TYPE`.
#[must_use]
pub const fn vehicle(mav_type: u8) -> Option<&'static str> {
    match mav_type {
        1 | 16 | 19..=25 => Some("ArduPlane"),
        4 => Some("Heli"),
        2 | 3 | 13 | 14 | 15 | 29 => Some("ArduCopter"),
        5 => Some("AntennaTracker"),
        10 | 11 => Some("Rover"),
        12 => Some("ArduSub"),
        _ => None,
    }
}

/// Where the versioned file for a vehicle and version lives.
/// `// C#: ExtLibs/Utilities/ParameterMetaDataRepositoryAPMpdef.cs:61`
#[must_use]
pub fn versioned_path(data_directory: &Path, vehicle: &str, version: Version) -> PathBuf {
    data_directory.join(format!("{vehicle}{version}.apm.pdef.xml"))
}

/// The URL the versioned file is fetched from.
/// `// C#: ExtLibs/Utilities/ParameterMetaDataRepositoryAPMpdef.cs:36`
#[must_use]
pub fn versioned_url(vehicle: &str, version: Version) -> String {
    format!(
        "https://autotest.ardupilot.org/Parameters/versioned/{vehicle}/stable-{version}/apm.pdef.xml"
    )
}

/// Where the unversioned file for a vehicle lives, unpacked.
/// `// C#: ExtLibs/Utilities/ParameterMetaDataRepositoryAPMpdef.cs:103`
#[must_use]
pub fn unversioned_path(data_directory: &Path, vehicle: &str) -> PathBuf {
    data_directory.join(format!("{vehicle}.apm.pdef.xml"))
}

/// The URL the unversioned file is fetched from, gzipped.
/// `// C#: ExtLibs/Utilities/ParameterMetaDataRepositoryAPMpdef.cs:34`
#[must_use]
pub fn unversioned_url(vehicle: &str) -> String {
    format!("https://autotest.ardupilot.org/Parameters/{vehicle}/apm.pdef.xml.gz")
}

/// Why a fetch failed.
#[derive(Debug, thiserror::Error)]
pub enum FetchError {
    /// The server did not give us the file.
    #[error("fetching {url}: {reason}")]
    Http {
        /// What was asked for.
        url: String,
        /// What went wrong.
        reason: String,
    },
    /// Writing the file failed.
    #[error("writing {path}: {source}")]
    Io {
        /// The file.
        path: PathBuf,
        /// The underlying error.
        source: std::io::Error,
    },
}

/// Fetches the versioned file for a vehicle, unless it is already there.
///
/// Returns the path either way. A file already on disk is never re-fetched: a release's
/// documentation does not change, which is why the C# checks `File.Exists` and stops.
/// `// C#: ExtLibs/Utilities/ParameterMetaDataRepositoryAPMpdef.cs:52-71`
pub fn fetch_versioned(
    data_directory: &Path,
    vehicle: &str,
    version: Version,
) -> Result<PathBuf, FetchError> {
    let path = versioned_path(data_directory, vehicle, version);
    if path.is_file() {
        return Ok(path);
    }
    let url = versioned_url(vehicle, version);
    let bytes = download(&url)?;
    write_atomically(&path, &bytes)?;
    Ok(path)
}

/// Fetches the unversioned file for a vehicle, if there is none or it is a week old.
///
/// The gzipped download is kept beside the unpacked file, as the C# keeps it.
/// `// C#: ExtLibs/Utilities/ParameterMetaDataRepositoryAPMpdef.cs:93-147`
pub fn fetch_unversioned(data_directory: &Path, vehicle: &str) -> Result<PathBuf, FetchError> {
    let path = unversioned_path(data_directory, vehicle);
    let fresh = std::fs::metadata(&path)
        .and_then(|meta| meta.modified())
        .ok()
        .and_then(|modified| modified.elapsed().ok())
        .is_some_and(|age| age < REFRESH_AFTER);
    if fresh {
        return Ok(path);
    }
    let url = unversioned_url(vehicle);
    let gzipped = download(&url)?;
    let mut unpacked = Vec::new();
    flate2::read::GzDecoder::new(gzipped.as_slice())
        .read_to_end(&mut unpacked)
        .map_err(|source| FetchError::Io {
            path: path.clone(),
            source,
        })?;
    write_atomically(&path.with_extension("xml.gz"), &gzipped)?;
    write_atomically(&path, &unpacked)?;
    Ok(path)
}

fn download(url: &str) -> Result<Vec<u8>, FetchError> {
    let http = |reason: String| FetchError::Http {
        url: url.to_owned(),
        reason,
    };
    let mut response = ureq::get(url)
        .config()
        .timeout_global(Some(Duration::from_secs(60)))
        .build()
        .call()
        .map_err(|err| http(err.to_string()))?;
    let mut bytes = Vec::new();
    response
        .body_mut()
        .as_reader()
        .read_to_end(&mut bytes)
        .map_err(|err| http(err.to_string()))?;
    Ok(bytes)
}

fn write_atomically(path: &Path, bytes: &[u8]) -> Result<(), FetchError> {
    let io = |source| FetchError::Io {
        path: path.to_path_buf(),
        source,
    };
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent).map_err(io)?;
    }
    let temporary = path.with_extension("part");
    std::fs::write(&temporary, bytes).map_err(io)?;
    std::fs::rename(&temporary, path).map_err(io)
}

/// What one `<param>` says, in the shape [`crate::param_meta::ParamMeta`] has.
#[derive(Debug, Clone, PartialEq)]
pub struct PdefParam {
    /// The bare name, without the `Vehicle:` prefix the file puts on vehicle parameters.
    pub name: String,
    /// `humanName`.
    pub display_name: String,
    /// `documentation`.
    pub description: String,
    /// `<field name="Units">`.
    pub units: String,
    /// `<field name="Range">`, as `low high`.
    pub range: Option<(f64, f64)>,
    /// The same field's text as it stands, for the Options cell.
    pub range_text: String,
    /// `<field name="Increment">`.
    pub increment: Option<f64>,
    /// `<values><value code="N">text</value>`.
    pub values: Vec<(i64, String)>,
    /// `<field name="Bitmask">` as `bit:text,bit:text`, or the `<bitmask><bit code>` elements.
    pub bitmask: Vec<(u32, String)>,
    /// `user`.
    pub user_level: UserLevel,
    /// `<field name="RebootRequired">True</field>`.
    pub reboot_required: bool,
    /// `<field name="ReadOnly">`'s text, as the C# reads it before its `bool.Parse`; `None`
    /// without the field.
    /// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:480-486; ExtLibs/Utilities/ParameterMetaDataConstants.cs:28`
    pub read_only: Option<String>,
}

/// A parsed `apm.pdef.xml`.
///
/// Names are matched as the C# matches them: `Vehicle:NAME` or bare `NAME`, first match in
/// document order, vehicle parameters before the libraries.
/// `// C#: ExtLibs/Utilities/ParameterMetaDataRepositoryAPMpdef.cs:219-231`
#[derive(Debug, Clone, Default)]
pub struct Pdef {
    params: BTreeMap<String, PdefParam>,
    vehicle: String,
}

/// Why a file could not be read.
#[derive(Debug, thiserror::Error)]
pub enum PdefError {
    /// The file could not be read.
    #[error("reading {path}: {source}")]
    Io {
        /// The file.
        path: PathBuf,
        /// The underlying error.
        source: std::io::Error,
    },
    /// The file is not a parameter definition file.
    #[error("not an apm.pdef.xml: {0}")]
    Xml(String),
}

impl Pdef {
    /// Reads a file.
    pub fn load(path: &Path) -> Result<Self, PdefError> {
        let text = std::fs::read_to_string(path).map_err(|source| PdefError::Io {
            path: path.to_path_buf(),
            source,
        })?;
        Self::parse(&text)
    }

    /// Parses a file's text.
    pub fn parse(text: &str) -> Result<Self, PdefError> {
        let document =
            roxmltree::Document::parse(text).map_err(|err| PdefError::Xml(err.to_string()))?;
        let root = document.root_element();
        if root.tag_name().name() != "paramfile" {
            return Err(PdefError::Xml(format!(
                "root is <{}>, not <paramfile>",
                root.tag_name().name()
            )));
        }
        let mut pdef = Self::default();
        // <paramfile><vehicles><parameters name=...>...</parameters></vehicles><libraries>...
        for group in root.children().filter(roxmltree::Node::is_element) {
            for parameters in group.children().filter(roxmltree::Node::is_element) {
                let block = parameters.attribute("name").unwrap_or_default();
                if group.tag_name().name() == "vehicles" && pdef.vehicle.is_empty() {
                    pdef.vehicle = block.to_owned();
                }
                for param in parameters
                    .children()
                    .filter(|node| node.is_element() && node.tag_name().name() == "param")
                {
                    let Some(full) = param.attribute("name") else {
                        continue;
                    };
                    let name = full.rsplit_once(':').map_or(full, |(_, bare)| bare);
                    // First match wins, as the C#'s document-order search returns the first.
                    if pdef.params.contains_key(name) {
                        continue;
                    }
                    pdef.params
                        .insert(name.to_owned(), read_param(name, &param));
                }
            }
        }
        Ok(pdef)
    }

    /// One parameter, if the file documents it.
    #[must_use]
    pub fn get(&self, name: &str) -> Option<&PdefParam> {
        self.params.get(name)
    }

    /// How many parameters the file documents.
    #[must_use]
    pub fn len(&self) -> usize {
        self.params.len()
    }

    /// Whether it documents none.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.params.is_empty()
    }

    /// Every documented parameter, by name.
    pub fn params(&self) -> impl Iterator<Item = &PdefParam> {
        self.params.values()
    }

    /// The vehicle the file is for, from its first `<parameters name>`.
    #[must_use]
    pub fn vehicle(&self) -> &str {
        &self.vehicle
    }
}

fn read_param(name: &str, param: &roxmltree::Node<'_, '_>) -> PdefParam {
    let mut out = PdefParam {
        name: name.to_owned(),
        display_name: param.attribute("humanName").unwrap_or_default().to_owned(),
        description: param
            .attribute("documentation")
            .unwrap_or_default()
            .to_owned(),
        units: String::new(),
        range: None,
        range_text: String::new(),
        increment: None,
        values: Vec::new(),
        bitmask: Vec::new(),
        user_level: match param.attribute("user") {
            Some("Standard") => UserLevel::Standard,
            Some("Advanced") => UserLevel::Advanced,
            _ => UserLevel::Unspecified,
        },
        reboot_required: false,
        read_only: None,
    };
    for child in param.children().filter(roxmltree::Node::is_element) {
        let text = child.text().unwrap_or_default().trim();
        match child.tag_name().name() {
            "field" => match child.attribute("name") {
                Some("Units") => out.units = text.to_owned(),
                Some("Range") => {
                    out.range_text = text.to_owned();
                    let mut parts = text.split_whitespace();
                    if let (Some(low), Some(high)) = (parts.next(), parts.next())
                        && let (Ok(low), Ok(high)) = (low.parse(), high.parse())
                    {
                        out.range = Some((low, high));
                    }
                }
                Some("Increment") => out.increment = text.parse().ok(),
                Some("RebootRequired") => out.reboot_required = text.eq_ignore_ascii_case("true"),
                // `xElement.Value`: the element's text as it stands.
                // C#: ExtLibs/Utilities/ParameterMetaDataRepositoryAPMpdef.cs:250-258
                Some("ReadOnly") => {
                    out.read_only = Some(child.text().unwrap_or_default().to_owned());
                }
                Some("Bitmask") if out.bitmask.is_empty() => {
                    out.bitmask = text
                        .split(',')
                        .filter_map(|entry| {
                            let (bit, label) = entry.split_once(':')?;
                            Some((bit.trim().parse().ok()?, label.trim().to_owned()))
                        })
                        .collect();
                }
                _ => {}
            },
            "values" => {
                for value in child
                    .children()
                    .filter(|node| node.is_element() && node.tag_name().name() == "value")
                {
                    if let Some(code) = value.attribute("code").and_then(|code| code.parse().ok()) {
                        out.values
                            .push((code, value.text().unwrap_or_default().trim().to_owned()));
                    }
                }
            }
            "bitmask" => {
                let bits: Vec<(u32, String)> = child
                    .children()
                    .filter(|node| node.is_element() && node.tag_name().name() == "bit")
                    .filter_map(|bit| {
                        Some((
                            bit.attribute("code")?.parse().ok()?,
                            bit.text().unwrap_or_default().trim().to_owned(),
                        ))
                    })
                    .collect();
                if !bits.is_empty() {
                    out.bitmask = bits;
                }
            }
            _ => {}
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    const SAMPLE: &str = r#"<?xml version="1.0" encoding="utf-8"?>
<paramfile>
  <vehicles>
    <parameters name="ArduCopter">
      <param humanName="Throttle filter cutoff" name="ArduCopter:PILOT_THR_FILT" documentation="Throttle filter cutoff (Hz)" user="Advanced">
        <field name="Units">Hz</field>
        <field name="Range">0 10</field>
        <field name="Increment">0.5</field>
      </param>
      <param humanName="Throttle stick behavior" name="ArduCopter:PILOT_THR_BHV" documentation="Bitmask" user="Standard">
        <bitmask>
          <bit code="0">Feedback from mid stick</bit>
          <bit code="2">Disarm on land detection</bit>
        </bitmask>
        <field name="Bitmask">0:Feedback from mid stick,2:Disarm on land detection</field>
      </param>
      <param humanName="Shadowed" name="ArduCopter:ACRO_RP_EXPO" documentation="the vehicle's" user="Standard"></param>
    </parameters>
  </vehicles>
  <libraries>
    <parameters name="ACRO_">
      <param humanName="Acro Roll/Pitch Expo" name="ACRO_RP_EXPO" documentation="the library's" user="Advanced">
        <field name="Range">-0.5 0.95</field>
        <field name="RebootRequired">True</field>
        <field name="ReadOnly">True</field>
        <values>
          <value code="0">Disabled</value>
          <value code="1">Enabled</value>
        </values>
      </param>
    </parameters>
  </libraries>
</paramfile>"#;

    #[test]
    fn a_pdef_reads_fields_values_and_bits() {
        let pdef = Pdef::parse(SAMPLE).expect("parses");
        assert_eq!(pdef.vehicle(), "ArduCopter");
        let filt = pdef.get("PILOT_THR_FILT").expect("documented");
        assert_eq!(filt.display_name, "Throttle filter cutoff");
        assert_eq!(filt.units, "Hz");
        assert_eq!(filt.range, Some((0.0, 10.0)));
        assert_eq!(filt.increment, Some(0.5));
        assert_eq!(filt.user_level, UserLevel::Advanced);
        let bhv = pdef.get("PILOT_THR_BHV").expect("documented");
        assert_eq!(
            bhv.bitmask,
            vec![
                (0, "Feedback from mid stick".to_owned()),
                (2, "Disarm on land detection".to_owned())
            ]
        );
        assert_eq!(pdef.get("NOPE"), None);
        assert_eq!(filt.read_only, None);
    }

    /// `<field name="ReadOnly">`'s text is kept for the Full Parameter List's edit check.
    #[test]
    fn a_read_only_field_is_kept_as_its_text() {
        let pdef = Pdef::parse(SAMPLE).expect("parses");
        let lib = Pdef::parse(&SAMPLE.replace("ArduCopter:ACRO_RP_EXPO", "ArduCopter:ACRO_OTHER"))
            .expect("parses");
        assert_eq!(
            pdef.get("ACRO_RP_EXPO").and_then(|p| p.read_only.clone()),
            None
        );
        assert_eq!(
            lib.get("ACRO_RP_EXPO").and_then(|p| p.read_only.clone()),
            Some("True".to_owned())
        );
    }

    /// The vehicle's block comes before the libraries, and the C# returns the first match.
    #[test]
    fn the_first_match_in_document_order_wins() {
        let pdef = Pdef::parse(SAMPLE).expect("parses");
        let expo = pdef.get("ACRO_RP_EXPO").expect("documented");
        assert_eq!(expo.description, "the vehicle's");
        assert_eq!(pdef.len(), 3);
    }

    #[test]
    fn a_library_parameter_reads_its_values_and_reboot_flag() {
        let mut only_library = SAMPLE.to_owned();
        only_library = only_library.replace(
            r#"<param humanName="Shadowed" name="ArduCopter:ACRO_RP_EXPO" documentation="the vehicle's" user="Standard"></param>"#,
            "",
        );
        let pdef = Pdef::parse(&only_library).expect("parses");
        let expo = pdef.get("ACRO_RP_EXPO").expect("documented");
        assert_eq!(expo.range, Some((-0.5, 0.95)));
        assert!(expo.reboot_required);
        assert_eq!(
            expo.values,
            vec![(0, "Disabled".to_owned()), (1, "Enabled".to_owned())]
        );
    }

    #[test]
    fn a_file_that_is_not_a_pdef_is_refused() {
        assert!(matches!(Pdef::parse("<Config/>"), Err(PdefError::Xml(_))));
        assert!(matches!(Pdef::parse("nope"), Err(PdefError::Xml(_))));
    }

    /// The banner regex, case by case, as VersionDetection.cs:15-56 reads them.
    #[test]
    fn versions_parse_from_banners_as_the_csharp_parses_them() {
        assert_eq!(
            Version::from_banner("ArduCopter V4.5.7 (1c0c8d9c)"),
            Some(Version::new(4, 5, 7))
        );
        assert_eq!(
            Version::from_banner("ArduPlane V4.8.0-dev (ca7ee129)").map(|v| v.to_string()),
            Some("4.8.0".to_owned()),
            "a hyphen stops the regex: the word branch needs letters right after the number"
        );
        assert_eq!(
            Version::from_banner("ArduCopter V4.0.0dev").map(|v| v.to_string()),
            Some("4.0.0.255".to_owned())
        );
        assert_eq!(
            Version::from_banner("ArduCopter V4.6.0-rc1").map(|v| v.to_string()),
            Some("4.6.0.1".to_owned())
        );
        assert_eq!(
            Version::from_banner("ArduCopter V4.6.0-beta2").map(|v| v.to_string()),
            Some("4.6.0".to_owned()),
            "a hyphenated word that is not -rc stops the match, as the regex does"
        );
        assert_eq!(
            Version::from_banner("APM:Copter V3.6.12 (abc)").map(|v| v.to_string()),
            Some("3.6.12".to_owned())
        );
        assert_eq!(
            Version::from_banner("ArduCopter V4.5a").map(|v| v.to_string()),
            Some("4.5.49".to_owned()),
            "the C# turns a single letter into (letter - 0x30): a is 49"
        );
        assert_eq!(Version::from_banner("no version here"), None);
        assert_eq!(Version::from_banner("ChibiOS: 1234abcd"), None);
    }

    #[test]
    fn the_packed_version_unpacks() {
        let packed = (4u32 << 24) | (5 << 16) | (7 << 8) | 255;
        assert_eq!(
            Version::from_flight_sw_version(packed),
            Version::new(4, 5, 7)
        );
    }

    /// The file names and URLs are the C#'s, character for character.
    #[test]
    fn paths_and_urls_match_the_csharp() {
        let dir = Path::new("/data");
        let version = Version::new(4, 4, 4);
        assert_eq!(
            versioned_path(dir, "Copter", version),
            PathBuf::from("/data/Copter4.4.4.apm.pdef.xml")
        );
        assert_eq!(
            versioned_url("Copter", version),
            "https://autotest.ardupilot.org/Parameters/versioned/Copter/stable-4.4.4/apm.pdef.xml"
        );
        assert_eq!(
            unversioned_path(dir, "ArduCopter"),
            PathBuf::from("/data/ArduCopter.apm.pdef.xml")
        );
        assert_eq!(
            unversioned_url("ArduCopter"),
            "https://autotest.ardupilot.org/Parameters/ArduCopter/apm.pdef.xml.gz"
        );
        assert_eq!(versioned_vehicle(2), Some("Copter"));
        assert_eq!(versioned_vehicle(1), Some("Plane"));
        assert_eq!(versioned_vehicle(12), Some("Sub"));
        assert_eq!(versioned_vehicle(6), None, "a GCS is not a vehicle");
        assert_eq!(vehicle(4), Some("Heli"));
        assert!(VERSIONED_VEHICLES.iter().all(|v| !v.starts_with("Ardu")));
    }

    /// The files the real Mission Planner keeps on this machine parse, and the versioned one
    /// documents what the release documents.
    #[test]
    fn the_real_files_on_this_machine_parse() {
        let Some(dir) = mp_settings::data_directory() else {
            eprintln!("skipped: no home directory");
            return;
        };
        let path = unversioned_path(&dir, "ArduCopter");
        if !path.is_file() {
            eprintln!("skipped: no {}", path.display());
            return;
        }
        let pdef = Pdef::load(&path).expect("the real ArduCopter.apm.pdef.xml parses");
        eprintln!("{}: {} parameters documented", path.display(), pdef.len());
        assert!(pdef.len() > 1000);
        let expo = pdef.get("ACRO_RP_EXPO").expect("a library parameter");
        assert!(expo.range.is_some());
    }

    /// PLAN.md §10.5's measurement, made a test: how many of a real vehicle's parameters the
    /// bundled table documents against how many the downloaded file does.
    #[test]
    fn the_downloaded_file_documents_more_of_a_real_vehicle_than_the_bundled_table() {
        let fixture = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata/params/sitl-copter.param");
        let text = std::fs::read_to_string(&fixture).expect("the SITL parameter dump");
        let names: Vec<&str> = text
            .lines()
            .filter(|line| !line.starts_with('#'))
            .filter_map(|line| line.split(',').next())
            .collect();
        assert_eq!(names.len(), 1408, "the dump PLAN.md 10.5 counted");
        let bundled = names
            .iter()
            .filter(|name| crate::param_meta::lookup(name).is_some())
            .count();
        eprintln!("bundled table documents {bundled} of {}", names.len());

        let Some(dir) = mp_settings::data_directory() else {
            return;
        };
        let path = unversioned_path(&dir, "ArduCopter");
        let Ok(pdef) = Pdef::load(&path) else {
            eprintln!("skipped the download comparison: no {}", path.display());
            return;
        };
        let downloaded = names.iter().filter(|name| pdef.get(name).is_some()).count();
        eprintln!(
            "{} documents {downloaded} of {} ({} undocumented)",
            path.display(),
            names.len(),
            names.len() - downloaded
        );
        assert!(downloaded > bundled, "{downloaded} vs {bundled}");
    }
}
