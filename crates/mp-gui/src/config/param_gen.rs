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

//! Param gen: the Advanced page's "Param gen" button (`BUT_paramgen_Click`,
//! `ConfigAdvanced.cs:57-78`) and `ExtLibs/Utilities/ParameterMetaDataParser.cs`, which it
//! runs behind a `ProgressReporterDialogue` saying "Downloading updated data": every
//! `Parameters.cpp` in the application's list of locations is fetched, its `@Param` and `@Group`
//! comments read - each group's `@Path` files fetched and read in turn, with the file an
//! `AP_NESTEDGROUPINFO` names - and `ParameterMetaData.xml` written in the user data directory,
//! a vehicle element each holding a parameter element each with its documentation.
//!
//! What `GetParameterInformation` does (`:56-168`):
//!
//! * the locations are `ParameterLocationsBleeding`, then `ParameterLocations` (`app.config`),
//!   then four of older Copter and Plane branches, split at `;`;
//! * each is read (`ReadDataFromAddress`, `:464-585`): over HTTP with the user agent and a 30 s
//!   timeout, cached for the run, a 404 or 400 remembered as nothing, any other failure tried
//!   three times then nothing; an address not starting with `http` is a file;
//! * the locations are sorted by vehicle (`GetVehicle`, `:170-202`: the lower-cased address
//!   containing `arducopter` is `ArduCopter2`, `arduplane` `ArduPlane`, `rover` `ArduRover`,
//!   `ardusub` `ArduSub`, `tracker` `ArduTracker`, else `none`) and written in that order, a
//!   vehicle element opened whenever the vehicle changes; a location that fetched nothing, or
//!   fewer than 200 characters (a blank template), is skipped;
//! * a location's parameters (`ParseParameterInformation`, `:277-314`) are its `@Param` comment
//!   blocks (`ParseKeyValuePairs`, `:316-434`): from each `@Param` marker to the next, the first
//!   `@Key: value` line names the parameter and the rest its documentation - `@DisplayName`,
//!   `@Description`, `@Range`, `@Values`... - a key given once; a `@Key{Copter,Plane}: value`
//!   line also sets the key when one of the frames names this vehicle (`truename_map`, which
//!   never names the tracker, so its framed lines are ignored, as in the C#); a parameter named
//!   twice in one file keeps its first block; each is written as `<PREFIXNAME>` with the group's
//!   prefix, spaces and brackets as `_`, and a child element per key;
//! * its groups (`ParseGroupInformation`, `:204-275`): the file an `AP_NESTEDGROUPINFO(Class, ..)`
//!   names - `Class` with the file's extension beside it - read for its parameters and groups
//!   first; then each `@Group` block's `@Path` files, resolved against the location, read for
//!   their parameters under the group's prefix and their groups under it, a path that resolves
//!   to the location itself skipped;
//! * the file written is read back, each vehicle's elements sorted by name and the first of each
//!   name kept, and saved again (`XElement.Save`: `<?xml version="1.0" encoding="utf-8"?>`, two
//!   spaces an indent).
//!
//! Where this is not the C# (each at its site):
//!
//! * the locations are read one after another, not three at a time, and sorted stably, where
//!   `List.Sort` is not: within a vehicle the list's order holds, so the bleeding branch's
//!   definition is the one kept of a name both branches have;
//! * the elements are sorted as the culture's comparer sorts (`mavlink_inspector::culture_cmp`),
//!   as `OrderBy` sorts them;
//! * `ParameterMetaDataRepositoryAPMpdef.GetMetaData(true)` and `Reload()` after the write have
//!   nothing to reload here: the screens read the firmware's own `apm.pdef.xml` and the bundled
//!   table (`metadata.rs`), not this file, which is written where the C# writes it for the user;
//! * the harness names its own locations through `MP_PARAM_LOCATIONS` (`;`-separated), a door
//!   the C# does not have, so a run can read files instead of GitHub;
//! * Cancel closes the dialogue and lets the fetch finish unseen (`ForceExit`); a failure - a
//!   name no XML element may carry, a file that will not write - goes on the status line, where
//!   the C#'s dialogue shows the exception (the owner's ruling of 2026-09-25).

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::collections::{BTreeSet, HashMap};
use std::path::{Path, PathBuf};
use std::sync::mpsc::{Receiver, TryRecvError};
use std::time::Instant;

use gpui::{AnyElement, Context, Window};
use mp_terrain::{Http, UreqHttp};

use super::serial_ports::{Bar, ProgressIds, progress_dialog};
use crate::MissionPlanner;

/// `ParameterLocationsBleeding`, `ParameterLocations`, and the four the handler adds, in the
/// order the handler joins them.
/// `// C#: app.config:19-20; GCSViews/ConfigurationView/ConfigAdvanced.cs:63-70`
pub const LOCATIONS: [&str; 16] = [
    "https://raw.oborne.me/ardupilot/ardupilot/master/ArduCopter/Parameters.cpp",
    "https://raw.oborne.me/ardupilot/ardupilot/master/ArduSub/Parameters.cpp",
    "https://raw.oborne.me/ardupilot/ardupilot/master/ArduPlane/Parameters.cpp",
    "https://raw.oborne.me/ardupilot/ardupilot/master/APMrover2/Parameters.cpp",
    "https://raw.oborne.me/ardupilot/ardupilot/master/Rover/Parameters.cpp",
    "https://raw.oborne.me/ardupilot/ardupilot/master/AntennaTracker/Parameters.cpp",
    "https://raw.oborne.me/ardupilot/ardupilot/ArduCopter-stable/ArduCopter/Parameters.cpp",
    "https://raw.oborne.me/ardupilot/ardupilot/ArduSub-stable/ArduSub/Parameters.cpp",
    "https://raw.oborne.me/ardupilot/ardupilot/ArduPlane-stable/ArduPlane/Parameters.cpp",
    "https://raw.oborne.me/ardupilot/ardupilot/APMrover2-stable/APMrover2/Parameters.cpp",
    "https://raw.oborne.me/ardupilot/ardupilot/Rover-stable/Rover/Parameters.cpp",
    "https://raw.oborne.me/ardupilot/ardupilot/master/AntennaTracker/Parameters.cpp",
    "https://raw.githubusercontent.com/ArduPilot/ardupilot/Copter-3.6/ArduCopter/Parameters.cpp",
    "https://raw.githubusercontent.com/ArduPilot/ardupilot/Copter-3.5/ArduCopter/Parameters.cpp",
    "https://raw.githubusercontent.com/ArduPilot/ardupilot/plane3.9/ArduCopter/Parameters.cpp",
    "https://raw.githubusercontent.com/ArduPilot/ardupilot/plane3.8/ArduCopter/Parameters.cpp",
];
/// The harness's own locations, `;`-separated.
pub const LOCATIONS_ENV: &str = "MP_PARAM_LOCATIONS";
/// `ParameterMetaDataXMLFileName`. `// C#: app.config:21; ParameterMetaDataParser.cs:65`
pub const XML_FILE: &str = "ParameterMetaData.xml";
/// The dialogue's words. `// C#: GCSViews/ConfigurationView/ConfigAdvanced.cs:62`
pub const DOWNLOADING: &str = "Downloading updated data";
/// `ParameterMetaDataConstants`.
/// `// C#: ExtLibs/Utilities/ParameterMetaDataConstants.cs`
pub const PARAM_DELIMITER: &str = "@";
pub const PATH_DELIMITER: char = ',';
pub const PARAM: &str = "Param";
pub const GROUP: &str = "Group";
pub const PATH: &str = "Path";
/// A location shorter than this is a blank template. `// C#: ParameterMetaDataParser.cs:99`
pub const BLANK_TEMPLATE: usize = 200;
/// `ReadDataFromAddress`'s attempts. `// C#: ParameterMetaDataParser.cs:467`
pub const ATTEMPTS: u32 = 3;

/// `truename_map`, in its order: a vehicle element's name and the frame word it answers to.
/// `// C#: ExtLibs/Utilities/ParameterMetaDataParser.cs:34-50`
const TRUENAME_MAP: [(&str, &str); 13] = [
    ("ArduCopter2", "Copter"),
    ("ArduRover", "Rover"),
    ("APMrover2", "Rover"),
    ("ArduSub", "Sub"),
    ("ArduCopter", "Copter"),
    ("ArduPlane", "Plane"),
    ("AntennaTracker", "Tracker"),
    ("Copter", "Copter"),
    ("Rover", "Rover"),
    ("Plane", "Plane"),
    ("Sub", "Sub"),
    ("Tracker", "Tracker"),
    ("Blimp", "Blimp"),
];

/// `GetVehicle`: the element a location's parameters go under.
/// `// C#: ExtLibs/Utilities/ParameterMetaDataParser.cs:170-202`
#[must_use]
pub fn vehicle_of(location: &str) -> &'static str {
    let lower = location.to_lowercase();
    if lower.contains("arducopter") {
        "ArduCopter2"
    } else if lower.contains("arduplane") {
        "ArduPlane"
    } else if lower.contains("rover") {
        "ArduRover"
    } else if lower.contains("ardusub") {
        "ArduSub"
    } else if lower.contains("tracker") {
        "ArduTracker"
    } else {
        "none"
    }
}

/// `GetIndexOfMarkers`: every index of `delimiter` in `text`, case aside.
/// `// C#: ExtLibs/Utilities/ParameterMetaDataParser.cs:436-456`
#[must_use]
pub fn markers(text: &str, delimiter: &str) -> Vec<usize> {
    let lower = text.to_lowercase();
    let needle = delimiter.to_lowercase();
    let mut out = Vec::new();
    let mut from = 0;
    while let Some(found) = lower.get(from..).and_then(|rest| rest.find(&needle)) {
        let at = from + found;
        out.push(at);
        from = at + needle.len();
        if from >= lower.len() {
            break;
        }
    }
    out
}

/// `_paramMetaRegex`: `@Key{Frames}: value` at the start of `text`, the value to the line's end.
/// `// C#: ExtLibs/Utilities/ParameterMetaDataParser.cs:22-24`
fn meta_line(text: &str) -> Option<(String, String, String)> {
    let rest = text.strip_prefix(PARAM_DELIMITER)?;
    let key_end = rest
        .find(|c: char| !(c.is_alphanumeric() || c == '_'))
        .unwrap_or(rest.len());
    let key = &rest[..key_end];
    if key.is_empty() {
        return None;
    }
    let mut after = &rest[key_end..];
    let mut frame = String::new();
    if let Some(inner) = after.strip_prefix('{') {
        let close = inner.find('}')?;
        let frames = &inner[..close];
        if frames.contains(':') {
            return None;
        }
        frame = frames.to_owned();
        after = &inner[close + 1..];
    }
    let value = after.strip_prefix(':')?;
    let line_end = value.find('\n').unwrap_or(value.len());
    let value = &value[..line_end];
    if value.is_empty() {
        return None;
    }
    Some((key.to_owned(), frame, value.to_owned()))
}

/// A parameter's or group's documentation: its name and its keys with their values, in the
/// order found.
pub type Block = (String, Vec<(String, String)>);

/// `ParseKeyValuePairs`: the `@Param` or `@Group` blocks of `text` for `vehicle_type`.
/// `// C#: ExtLibs/Utilities/ParameterMetaDataParser.cs:316-434`
#[must_use]
pub fn blocks(text: &str, node_key: &str, vehicle_type: &str) -> Vec<Block> {
    let mut out: Vec<Block> = Vec::new();
    let starts = markers(text, &format!("{PARAM_DELIMITER}{node_key}"));
    for (i, start) in starts.iter().enumerate() {
        let stop = starts.get(i + 1).map_or(text.len(), |next| next + 1);
        let Some(block) = text.get(*start..stop) else {
            continue;
        };
        if block.is_empty() {
            continue;
        }
        let metas = markers(block, PARAM_DELIMITER);
        let (Some(first), Some(second)) = (metas.first(), metas.get(1)) else {
            continue;
        };
        let Some(name_line) = block.get(*first..*second) else {
            continue;
        };
        let Some((key, _, value)) = meta_line(name_line) else {
            continue;
        };
        if key != node_key {
            continue;
        }
        let name = value.trim_matches([' ', '\r', '\n']).to_owned();
        if out.iter().any(|(held, _)| *held == name) {
            // "Duplicate Key": the first block stays.
            continue;
        }
        let mut dict: Vec<(String, String)> = Vec::new();
        for (x, meta_start) in metas.iter().enumerate().skip(1) {
            let meta_stop = metas.get(x + 1).map_or(block.len(), |next| next + 1);
            let Some(meta) = block.get(*meta_start..meta_stop) else {
                continue;
            };
            let Some((meta_key, frames, meta_value)) = meta_line(meta) else {
                continue;
            };
            let meta_key = meta_key.trim_matches(' ').to_owned();
            let meta_value = meta_value.trim_matches(' ').to_owned();
            if !dict.iter().any(|(k, _)| *k == meta_key) {
                dict.push((meta_key.clone(), meta_value.clone()));
            }
            if !frames.is_empty() {
                for frame in frames.split(',') {
                    let word = frame.trim().to_lowercase();
                    // `truename_map.First(a => a.Value.ToLower() == word)`: the first row with
                    // that word, whose key must be this vehicle; no row is "Invalid MetaFrame".
                    let Some((key_name, _)) = TRUENAME_MAP
                        .iter()
                        .find(|(_, frame_word)| frame_word.to_lowercase() == word)
                    else {
                        continue;
                    };
                    if key_name.to_lowercase() == vehicle_type.to_lowercase()
                        && let Some(entry) = dict.iter_mut().find(|(k, _)| *k == meta_key)
                    {
                        entry.1 = meta_value.clone();
                    }
                }
            }
        }
        out.push((name, dict));
    }
    out
}

/// `new Uri(new Uri(location), relative).AbsoluteUri`: a relative path against the location's
/// directory, `..` and `.` segments resolved; an address with a scheme stands.
fn resolve(location: &str, relative: &str) -> String {
    let relative = relative.trim();
    if relative.contains("://") {
        return relative.to_owned();
    }
    let (scheme, rest) = location
        .split_once("://")
        .map_or(("", location), |(s, r)| (s, r));
    let (host, path) = rest.split_once('/').unwrap_or((rest, ""));
    let mut segments: Vec<&str> = path.split('/').collect();
    segments.pop();
    for part in relative.split('/') {
        match part {
            "" | "." => {}
            ".." => {
                segments.pop();
            }
            other => segments.push(other),
        }
    }
    if scheme.is_empty() {
        format!("{host}/{}", segments.join("/"))
    } else {
        format!("{scheme}://{host}/{}", segments.join("/"))
    }
}

/// `AP_NESTEDGROUPINFO\((.+),.+\)`: the class the first such line names.
/// `// C#: ExtLibs/Utilities/ParameterMetaDataConstants.cs; ParameterMetaDataParser.cs:207`
fn nested_group(text: &str) -> Option<String> {
    let at = text.find("AP_NESTEDGROUPINFO(")?;
    let after = &text[at + "AP_NESTEDGROUPINFO(".len()..];
    let line_end = after.find('\n').unwrap_or(after.len());
    let line = &after[..line_end];
    let close = line.rfind(')')?;
    let inside = &line[..close];
    // `(.+),.+`: greedy, so the last comma splits.
    let comma = inside.rfind(',')?;
    let class = &inside[..comma];
    if class.is_empty() || inside[comma + 1..].is_empty() {
        return None;
    }
    Some(class.to_owned())
}

/// `ReadDataFromAddress` over the run: the cache, the 404 remembered as nothing, the three
/// attempts, the file read for an address that is not `http`.
/// `// C#: ExtLibs/Utilities/ParameterMetaDataParser.cs:464-585`
pub struct Reader<'a> {
    http: &'a dyn Http,
    cache: HashMap<String, String>,
    /// The addresses read, in order, for the facts and the tests.
    pub read: Vec<String>,
}

impl<'a> Reader<'a> {
    #[must_use]
    pub fn new(http: &'a dyn Http) -> Self {
        Self {
            http,
            cache: HashMap::new(),
            read: Vec::new(),
        }
    }

    pub fn read(&mut self, address: &str) -> String {
        if let Some(held) = self.cache.get(address) {
            return held.clone();
        }
        self.read.push(address.to_owned());
        let data = if address.to_lowercase().starts_with("http") {
            let mut data = String::new();
            for _ in 0..ATTEMPTS {
                match self.http.get_status(address) {
                    // A 404 or 400 is nothing, cached; another status is the C#'s throw, tried
                    // again, as is no answer.
                    Ok((404 | 400, _)) => break,
                    Ok((status, bytes)) if (200..300).contains(&status) => {
                        data = String::from_utf8_lossy(&bytes).into_owned();
                        break;
                    }
                    Ok(_) | Err(_) => {}
                }
            }
            data
        } else {
            // `address.Replace("file:///", "")`: on Windows that leaves `C:/...`; here the
            // path keeps its root.
            std::fs::read_to_string(address.replace("file://", "")).unwrap_or_default()
        };
        self.cache.insert(address.to_owned(), data.clone());
        data
    }
}

/// A vehicle's elements as they are written: the element's name, and its keys.
pub type Elements = Vec<Block>;

/// `ParseGroupInformation` and `ParseParameterInformation`: a location's parameters into
/// `out`, then the nested group's file and each group's `@Path` files, recursively.
/// `// C#: ExtLibs/Utilities/ParameterMetaDataParser.cs:204-314`
fn parse_location(
    reader: &mut Reader<'_>,
    text: &str,
    location: &str,
    prefix: &str,
    vehicle: &str,
    out: &mut Elements,
    depth: usize,
) {
    if depth > 8 {
        return;
    }
    for (name, dict) in blocks(text, PARAM, vehicle) {
        let prefix = prefix.replace(['(', ')'], "_").replace(' ', "_");
        let element = format!("{prefix}{}", name.replace(' ', "_"));
        out.push((element, dict));
    }
    if let Some(class) = nested_group(text) {
        let current = location.rsplit('/').next().unwrap_or(location);
        let extension = current.rfind('.').map_or("", |at| &current[at..]);
        let new_name = format!("{class}{extension}");
        if current != new_name {
            let new_path = location.replace(current, &new_name);
            let data = reader.read(&new_path);
            parse_params_only(&data, prefix, vehicle, out);
            parse_groups(reader, &data, &new_path, prefix, vehicle, out, depth + 1);
        }
    }
    parse_groups(reader, text, location, prefix, vehicle, out, depth);
}

/// The nested group's own parameters, under the prefix, with no groups of their own read
/// twice.
fn parse_params_only(text: &str, prefix: &str, vehicle: &str, out: &mut Elements) {
    for (name, dict) in blocks(text, PARAM, vehicle) {
        let prefix = prefix.replace(['(', ')'], "_").replace(' ', "_");
        out.push((format!("{prefix}{}", name.replace(' ', "_")), dict));
    }
}

/// Each `@Group` block's `@Path` files.
fn parse_groups(
    reader: &mut Reader<'_>,
    text: &str,
    location: &str,
    prefix: &str,
    vehicle: &str,
    out: &mut Elements,
    depth: usize,
) {
    for (group, dict) in blocks(text, GROUP, vehicle) {
        for (key, value) in &dict {
            if key != PATH {
                continue;
            }
            for separated in value.split(PATH_DELIMITER) {
                let new_path = resolve(location, separated.trim());
                if new_path == location {
                    continue;
                }
                let data = reader.read(&new_path);
                if data.is_empty() {
                    continue;
                }
                let new_prefix = format!("{prefix}{group}");
                parse_location(
                    reader,
                    &data,
                    &new_path,
                    &new_prefix,
                    vehicle,
                    out,
                    depth + 1,
                );
            }
        }
    }
}

/// What a generation came to: the file, and each vehicle's element count as written.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Outcome {
    pub file: PathBuf,
    pub vehicles: Vec<(String, usize)>,
    pub read: usize,
}

/// An XML name: letters, digits, `_`, `-`, `.`, `:` after a letter or `_`, as
/// `WriteStartElement` checks it.
fn valid_name(name: &str) -> bool {
    let mut chars = name.chars();
    match chars.next() {
        Some(c) if c.is_alphabetic() || c == '_' || c == ':' => {}
        _ => return false,
    }
    chars.all(|c| c.is_alphanumeric() || matches!(c, '_' | '-' | '.' | ':'))
}

/// A text node as `XmlWriter` writes it: `&`, `<`, `>` escaped and `\r` as `&#xD;`.
fn escape(text: &str) -> String {
    text.replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('\r', "&#xD;")
}

/// `GetParameterInformation`: the locations read and parsed, the file written, read back,
/// sorted and saved; the C#'s throws as the error.
/// `// C#: ExtLibs/Utilities/ParameterMetaDataParser.cs:56-168`
pub fn generate(locations: &[String], http: &dyn Http, file: &Path) -> Result<Outcome, String> {
    let mut reader = Reader::new(http);
    let mut locations: Vec<String> = locations
        .iter()
        .map(|l| l.trim().to_owned())
        .filter(|l| !l.is_empty())
        .collect();
    // The precache: every base and its groups' files read first.
    for location in &locations {
        let data = reader.read(location);
        let mut unused = Vec::new();
        parse_groups(&mut reader, &data, location, "", "", &mut unused, 0);
    }
    // Stably, where `List.Sort` is not.
    locations.sort_by_key(|location| vehicle_of(&location.to_lowercase()).to_owned());
    let mut tree: Vec<(String, Elements)> = Vec::new();
    for location in &locations {
        let element = vehicle_of(&location.to_lowercase());
        let data = reader.read(location);
        if data.is_empty() || data.len() < BLANK_TEMPLATE {
            continue;
        }
        if tree.last().is_none_or(|(held, _)| held != element) {
            tree.push((element.to_owned(), Vec::new()));
        }
        let Some((_, elements)) = tree.last_mut() else {
            continue;
        };
        parse_location(&mut reader, &data, location, "", element, elements, 0);
    }
    for (vehicle, elements) in &tree {
        if !valid_name(vehicle) {
            return Err(format!("Invalid name character in '{vehicle}'."));
        }
        for (name, dict) in elements {
            if !valid_name(name) {
                return Err(format!("Invalid name character in '{name}'."));
            }
            for (key, _) in dict {
                if !valid_name(key) {
                    return Err(format!("Invalid name character in '{key}'."));
                }
            }
        }
    }
    // The read-back: each vehicle's elements sorted by name, the first of each name kept.
    let mut vehicles = Vec::new();
    let mut text = String::from("<?xml version=\"1.0\" encoding=\"utf-8\"?>\r\n<Params>\r\n");
    for (vehicle, mut elements) in tree {
        elements.sort_by(|a, b| super::mavlink_inspector::culture_cmp(&a.0, &b.0));
        let mut seen = BTreeSet::new();
        elements.retain(|(name, _)| seen.insert(name.clone()));
        vehicles.push((vehicle.clone(), elements.len()));
        text.push_str(&format!("  <{vehicle}>\r\n"));
        for (name, dict) in &elements {
            if dict.is_empty() {
                text.push_str(&format!("    <{name} />\r\n"));
                continue;
            }
            text.push_str(&format!("    <{name}>\r\n"));
            for (key, value) in dict {
                if value.is_empty() {
                    text.push_str(&format!("      <{key}></{key}>\r\n"));
                } else {
                    text.push_str(&format!("      <{key}>{}</{key}>\r\n", escape(value)));
                }
            }
            text.push_str(&format!("    </{name}>\r\n"));
        }
        text.push_str(&format!("  </{vehicle}>\r\n"));
    }
    text.push_str("</Params>");
    if let Some(parent) = file.parent() {
        std::fs::create_dir_all(parent).map_err(|e| e.to_string())?;
    }
    std::fs::write(file, text).map_err(|e| e.to_string())?;
    Ok(Outcome {
        file: file.to_path_buf(),
        vehicles,
        read: reader.read.len(),
    })
}

/// The locations a run reads: the harness's, or the C#'s.
#[must_use]
pub fn locations() -> Vec<String> {
    if let Ok(list) = std::env::var(LOCATIONS_ENV) {
        return list
            .split(';')
            .map(str::trim)
            .filter(|l| !l.is_empty())
            .map(str::to_owned)
            .collect();
    }
    LOCATIONS.iter().map(|l| (*l).to_owned()).collect()
}

/// A run on its thread.
type Running = (Instant, Receiver<Result<Outcome, String>>);

/// Param gen: the run behind its dialogue, and what the last run came to.
#[derive(Default)]
pub struct ParamGen {
    running: Option<Running>,
    /// How many runs were started.
    pub started: usize,
    /// How many ended, and the last one's outcome.
    pub finished: usize,
    pub last: Option<Result<Outcome, String>>,
    /// Cancel pressed: the dialogue closed, the run's outcome dropped.
    pub cancelled: usize,
    status: Option<String>,
}

impl std::fmt::Debug for ParamGen {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ParamGen")
            .field("running", &self.running.is_some())
            .field("started", &self.started)
            .finish_non_exhaustive()
    }
}

impl ParamGen {
    /// Whether the dialogue is up.
    #[must_use]
    pub fn running(&self) -> bool {
        self.running.is_some()
    }

    /// `BUT_paramgen_Click`: the dialogue up and the run started, writing to `data_dir`'s
    /// `ParameterMetaData.xml`; nothing while one runs.
    /// `// C#: GCSViews/ConfigurationView/ConfigAdvanced.cs:57-78`
    pub fn start(&mut self, data_dir: &Path) {
        self.start_with(data_dir, locations());
    }

    /// The run over `list`.
    pub fn start_with(&mut self, data_dir: &Path, list: Vec<String>) {
        if self.running.is_some() {
            return;
        }
        let file = data_dir.join(XML_FILE);
        let (send, receive) = std::sync::mpsc::channel();
        let spawned = std::thread::Builder::new()
            .name("mp-param-gen".to_owned())
            .spawn(move || {
                let http = UreqHttp::new();
                let _ = send.send(generate(&list, &http, &file));
            });
        match spawned {
            Ok(_) => {
                self.started += 1;
                self.running = Some((Instant::now(), receive));
            }
            Err(error) => self.status = Some(error.to_string()),
        }
    }

    /// The dialogue's Cancel: `ForceExit`, the dialogue closed, the run unseen.
    pub fn cancel(&mut self) {
        if self.running.take().is_some() {
            self.cancelled += 1;
        }
    }

    /// Once a frame: the run's outcome, the dialogue closed; an error on the status line.
    pub fn tick(&mut self) -> Option<String> {
        if let Some((_, receiver)) = self.running.as_ref() {
            let done = match receiver.try_recv() {
                Ok(outcome) => Some(outcome),
                Err(TryRecvError::Empty) => None,
                Err(TryRecvError::Disconnected) => {
                    Some(Err("the run ended with no answer".to_owned()))
                }
            };
            if let Some(outcome) = done {
                self.running = None;
                self.finished += 1;
                if let Err(error) = &outcome {
                    self.status = Some(error.clone());
                }
                self.last = Some(outcome);
            }
        }
        self.status.take()
    }

    /// The dialogue's `started`, for its marquee.
    #[must_use]
    pub fn since(&self) -> Option<Instant> {
        self.running.as_ref().map(|(at, _)| *at)
    }

    /// Waits for the run, as a test does.
    #[cfg(test)]
    fn finish(&mut self) -> Option<String> {
        let mut status = None;
        for _ in 0..4000 {
            if let Some(words) = self.tick() {
                status = Some(words);
            }
            if self.running.is_none() {
                break;
            }
            std::thread::sleep(std::time::Duration::from_millis(5));
        }
        status
    }
}

/// The facts, under `config.paramgen.`.
pub fn record_facts(run: &ParamGen) {
    use crate::facts::record;
    record("config.paramgen.running", run.running());
    record("config.paramgen.started", run.started);
    record("config.paramgen.finished", run.finished);
    record("config.paramgen.cancelled", run.cancelled);
    match run.last.as_ref() {
        Some(Ok(outcome)) => {
            record("config.paramgen.last", "ok");
            record("config.paramgen.file", outcome.file.display().to_string());
            record("config.paramgen.read", outcome.read);
            record(
                "config.paramgen.vehicles",
                outcome
                    .vehicles
                    .iter()
                    .map(|(v, n)| format!("{v}:{n}"))
                    .collect::<Vec<_>>()
                    .join(","),
            );
        }
        Some(Err(error)) => record("config.paramgen.last", format!("error: {error}")),
        None => record("config.paramgen.last", "none"),
    }
}

/// The dialogue while the run is on: "Downloading updated data" over a marquee, with Cancel.
pub fn overlay(
    run: &ParamGen,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    let started = run.since()?;
    Some(progress_dialog(
        ProgressIds {
            frame: "paramgen-progress",
            bar: "paramgen-bar",
            cancel: "paramgen-cancel",
            backdrop: "paramgen-backdrop",
        },
        DOWNLOADING,
        Bar {
            marquee: true,
            value: 0,
        },
        started,
        Some(|this: &mut MissionPlanner| {
            this.extra.param_gen.cancel();
        }),
        window,
        cx,
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A fixture: a copter's `Parameters.cpp`, a library its group's `@Path` names, and the
    /// file that library's `AP_NESTEDGROUPINFO` names.
    fn fixture(dir: &Path) {
        std::fs::create_dir_all(dir.join("ArduCopter")).expect("dir");
        std::fs::create_dir_all(dir.join("libraries/AP_Foo")).expect("dir");
        let mut copter = String::from(
            "// @Param: ACRO_RP_P\n// @DisplayName: Acro Roll and Pitch P gain\n// @Description: Converts pilot roll and pitch into a desired rate of rotation in ACRO mode.\n// @Range: 1 10\n// @User: Standard\n// @Values{Copter}: 1:Low,2:High\n// @Values{Plane}: 0:Off\n    GSCALAR(acro_rp_p, \"ACRO_RP_P\", 4.5),\n\n// @Param: ACRO_RP_P\n// @DisplayName: a second block, which the first wins\n\n// @Group: FOO_\n// @Path: ../libraries/AP_Foo/AP_Foo.cpp\n    GOBJECT(foo, \"FOO_\", AP_Foo),\n\n// @Param: Z & <last>\n// @DisplayName: odd name\n",
        );
        while copter.len() < BLANK_TEMPLATE {
            copter.push_str("// padding\n");
        }
        std::fs::write(dir.join("ArduCopter/Parameters.cpp"), copter).expect("write");
        std::fs::write(
            dir.join("libraries/AP_Foo/AP_Foo.cpp"),
            "// @Param: ENABLE\n// @DisplayName: Foo enable\n// @Values: 0:Disabled,1:Enabled\n    AP_GROUPINFO(\"ENABLE\", 0, AP_Foo, _enable, 0),\n    AP_NESTEDGROUPINFO(AP_Foo_Backend, 1),\n",
        )
        .expect("write");
        std::fs::write(
            dir.join("libraries/AP_Foo/AP_Foo_Backend.cpp"),
            "// @Param: RATE\n// @DisplayName: Backend rate\n// @Units: Hz\n",
        )
        .expect("write");
    }

    /// A server with nothing: every address a 404.
    struct NoHttp;
    impl Http for NoHttp {
        fn get(&self, url: &str) -> Result<Vec<u8>, mp_terrain::HttpError> {
            Err(mp_terrain::HttpError(format!("{url}: 404")))
        }
        fn get_status(&self, _url: &str) -> Result<(u16, Vec<u8>), mp_terrain::HttpError> {
            Ok((404, b"404: Not Found".to_vec()))
        }
    }

    /// `GetVehicle` and the markers.
    #[test]
    fn vehicles_and_markers_are_the_csharps() {
        assert_eq!(
            vehicle_of(
                "https://x/master/ArduCopter/Parameters.cpp"
                    .to_lowercase()
                    .as_str()
            ),
            "ArduCopter2"
        );
        assert_eq!(
            vehicle_of("https://x/apmrover2/parameters.cpp"),
            "ArduRover"
        );
        assert_eq!(
            vehicle_of("https://x/antennatracker/parameters.cpp"),
            "ArduTracker"
        );
        assert_eq!(vehicle_of("https://x/blimp/parameters.cpp"), "none");
        assert_eq!(markers("@Param: A\n@param: B\n@X", "@Param"), vec![0, 10]);
        assert_eq!(
            meta_line("@Values{Copter,Plane}: 1:Low\n"),
            Some((
                "Values".to_owned(),
                "Copter,Plane".to_owned(),
                " 1:Low".to_owned()
            ))
        );
        assert_eq!(
            resolve("https://h/a/b/Parameters.cpp", "../libraries/X/X.cpp"),
            "https://h/a/libraries/X/X.cpp"
        );
        assert_eq!(
            resolve("file:///tmp/a/b.cpp", "./c.cpp"),
            "file:///tmp/a/c.cpp"
        );
        assert_eq!(
            nested_group("  AP_NESTEDGROUPINFO(AP_Foo_Backend, 1),\n"),
            Some("AP_Foo_Backend".to_owned())
        );
        assert!(
            valid_name("ACRO_RP_P")
                && valid_name("_x")
                && !valid_name("1A")
                && !valid_name("Z & <last>")
        );
    }

    /// `ParseKeyValuePairs`: the blocks, a key once, the frame's value for this vehicle, the
    /// first of a name repeated.
    #[test]
    fn blocks_are_read_as_the_csharp_reads_them() {
        let text = "// @Param: ACRO_RP_P\n// @DisplayName: Acro\n// @Values{Copter}: 1:Low\n// @Values{Plane}: 0:Off\n// @Param: ACRO_RP_P\n// @DisplayName: again\n";
        let copter = blocks(text, PARAM, "ArduCopter2");
        assert_eq!(copter.len(), 1);
        assert_eq!(copter[0].0, "ACRO_RP_P");
        assert_eq!(
            copter[0].1,
            vec![
                ("DisplayName".to_owned(), "Acro".to_owned()),
                ("Values".to_owned(), "1:Low".to_owned())
            ]
        );
        let plane = blocks(text, PARAM, "ArduPlane");
        assert_eq!(plane[0].1[1], ("Values".to_owned(), "0:Off".to_owned()));
        // The tracker's frame word maps to AntennaTracker, never ArduTracker: the first value.
        let tracker = blocks(
            "// @Param: A\n// @Values: x\n// @Values{Tracker}: y\n",
            PARAM,
            "ArduTracker",
        );
        assert_eq!(tracker[0].1, vec![("Values".to_owned(), "x".to_owned())]);
    }

    /// The whole run over files: the group's path and its nested group read, the elements
    /// prefixed, sorted and written as `XElement.Save` writes them; a name no element may
    /// carry is the C#'s throw.
    #[test]
    fn a_run_writes_the_xml_the_csharp_writes() {
        let dir = std::env::temp_dir().join(format!("mp-paramgen-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        fixture(&dir);
        let location = format!("file://{}/ArduCopter/Parameters.cpp", dir.display());
        let file = dir.join("out").join(XML_FILE);
        let failed = generate(std::slice::from_ref(&location), &NoHttp, &file);
        assert_eq!(
            failed,
            Err("Invalid name character in 'Z_&_<last>'.".to_owned())
        );
        // The odd name out of the fixture: the run writes.
        let path = dir.join("ArduCopter/Parameters.cpp");
        let text = std::fs::read_to_string(&path).expect("read");
        std::fs::write(
            &path,
            text.replace("// @Param: Z & <last>\n// @DisplayName: odd name\n", ""),
        )
        .expect("write");
        let outcome = generate(&[location], &NoHttp, &file).expect("a run");
        assert_eq!(outcome.vehicles, vec![("ArduCopter2".to_owned(), 3)]);
        let written = std::fs::read_to_string(&file).expect("the file");
        let expected = "<?xml version=\"1.0\" encoding=\"utf-8\"?>\r\n<Params>\r\n  <ArduCopter2>\r\n    <ACRO_RP_P>\r\n      <DisplayName>Acro Roll and Pitch P gain</DisplayName>\r\n      <Description>Converts pilot roll and pitch into a desired rate of rotation in ACRO mode.</Description>\r\n      <Range>1 10</Range>\r\n      <User>Standard</User>\r\n      <Values>1:Low,2:High</Values>\r\n    </ACRO_RP_P>\r\n    <FOO_ENABLE>\r\n      <DisplayName>Foo enable</DisplayName>\r\n      <Values>0:Disabled,1:Enabled</Values>\r\n    </FOO_ENABLE>\r\n    <FOO_RATE>\r\n      <DisplayName>Backend rate</DisplayName>\r\n      <Units>Hz</Units>\r\n    </FOO_RATE>\r\n  </ArduCopter2>\r\n</Params>";
        assert_eq!(written, expected);
        // A location fetching nothing (a 404) is skipped; a run with nothing writes an empty
        // Params.
        let outcome = generate(
            &["https://nowhere/ArduPlane/Parameters.cpp".to_owned()],
            &NoHttp,
            &file,
        )
        .expect("a run");
        assert!(outcome.vehicles.is_empty());
        assert_eq!(
            std::fs::read_to_string(&file).expect("the file"),
            "<?xml version=\"1.0\" encoding=\"utf-8\"?>\r\n<Params>\r\n</Params>"
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// The button: the dialogue up, the run on its thread, its outcome the facts'; Cancel
    /// closes the dialogue with the run unseen.
    #[test]
    fn the_button_runs_the_generation_behind_its_dialogue() {
        let dir = std::env::temp_dir().join(format!("mp-paramgen-ui-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        fixture(&dir);
        let path = dir.join("ArduCopter/Parameters.cpp");
        let text = std::fs::read_to_string(&path).expect("read");
        std::fs::write(
            &path,
            text.replace("// @Param: Z & <last>\n// @DisplayName: odd name\n", ""),
        )
        .expect("write");
        let list = vec![format!(
            "file://{}/ArduCopter/Parameters.cpp",
            dir.display()
        )];
        let mut run = ParamGen::default();
        run.start_with(&dir.join("data"), list.clone());
        assert!(run.running());
        assert_eq!(run.started, 1);
        assert_eq!(run.finish(), None);
        assert!(!run.running());
        assert_eq!(run.finished, 1);
        assert!(
            matches!(run.last.as_ref(), Some(Ok(outcome)) if outcome.vehicles == vec![("ArduCopter2".to_owned(), 3)])
        );
        assert!(dir.join("data").join(XML_FILE).is_file());
        run.start_with(&dir.join("data"), list);
        run.cancel();
        assert!(!run.running());
        assert_eq!(run.cancelled, 1);
        // The C#'s list stands when the harness names none.
        assert_eq!(locations().len(), LOCATIONS.len());
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// The GUI script names only facts and controls this window has.
    #[test]
    fn the_gui_script_names_facts_and_controls_this_window_has() {
        let script = include_str!("../../../../tests/gui/config-paramgen.gui");
        let source = include_str!("param_gen.rs");
        let mut facts = 0;
        for line in script.lines() {
            let line = line.split('#').next().unwrap_or("");
            let mut words = line.split_whitespace();
            match (words.next(), words.next()) {
                (Some("expect"), Some(key)) if key.starts_with("config.paramgen.") => {
                    assert!(
                        source.contains(&format!("\"{key}\"")),
                        "{key} is not recorded"
                    );
                    facts += 1;
                }
                (Some("click"), Some(id)) if id.starts_with("paramgen-") => {
                    assert!(source.contains(&format!("\"{id}\"")), "{id} is not drawn");
                }
                _ => {}
            }
        }
        assert!(facts >= 5, "{facts} facts");
    }
}
