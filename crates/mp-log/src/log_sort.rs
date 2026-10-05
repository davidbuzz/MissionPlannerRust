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

//! `LogSort.SortLogs`: logs moved into folders named for what made them - the temp form's Sort
//! TLogs and ReSort All logs.
//!
//! Each log, with every file beside it whose name starts with its own (its `.rlog`, its `.kml`),
//! goes under the destination folder: an empty one is deleted; one of 1024 bytes or fewer goes to
//! `SMALL`; a telemetry log to `[SITL/]<vehicle type>/<system id>/`, the type its heartbeats', and
//! to `BAD` with no heartbeat; a dataflash log to `[SITL/]<type>/<SYSID_THISMAV>/`, or stays where
//! it is. A `BRD_SERIAL_NUM` adds a folder below the system id.
//!
//! As the C# sorts, quirks and all:
//!
//! * a dataflash log's type is read from its `MSG` lines with `Select(...).Count() > 0`, which is
//!   true for any line at all - so any log with a `MSG` line is `GROUND_ROVER` (the last of the
//!   copter, plane and rover tests), and one without is `GENERIC`, which stays where it is;
//! * a telemetry log's `BRD_SERIAL_NUM` is compared as its sixteen-byte name, NULs and all, which
//!   never equals the fourteen letters: a telemetry log never gets the serial number's folder;
//! * a file already of that name where it would go stops the move of that log's files, as
//!   `File.Move` throws there; whatever fails, the log is left and the next sorted.
//!
//! The C# sorts with `Parallel.ForEach`; here one log after another, in the order given.
//! `// C#: ExtLibs/Utilities/LogSort.cs:1-311`

use std::path::{Path, PathBuf};

use mp_mavlink_dialects::all::{DIALECT, MavMessage, MavType};
use mp_os::fs::FsExt as _;

use crate::TlogReader;
use crate::convert::flight_mode_name;
use crate::dflogbuffer::DfLogBuffer;

/// A log this size or smaller is "most likely invalid". `// C#: LogSort.cs:47`
const SMALL_LOG: u64 = 1024;
/// The folders `SortLogs` makes.
const SMALL: &str = "SMALL";
const SITL: &str = "SITL";
const BAD: &str = "BAD";
/// How many heartbeats it reads before it stops. `// C#: LogSort.cs:210`
const HEARTBEATS: usize = 100;
/// How many `MSG` lines it reads. `// C#: LogSort.cs:81`
const MSG_LINES: usize = 100;
/// `MAV_COMP_ID_MISSIONPLANNER`: "no gcs packets". `// C#: LogSort.cs:191-193`
const GCS_COMPONENT: u8 = 190;
/// `MAV_TYPE`s.
const GENERIC: u8 = 0;
const ANTENNA_TRACKER: u8 = 5;
const GCS: u8 = 6;
const GROUND_ROVER: u8 = 10;
/// The messages that mark a simulator. `// C#: LogSort.cs:195`
const SIMSTATE: u32 = 164;
const HIL_CONTROLS: u32 = 91;
const HEARTBEAT: u32 = 0;
const PARAM_VALUE: u32 = 22;

/// `aptype.ToString()`: the C# enum's name, `MAV_TYPE_` dropped, or the number for one it has
/// not got.
fn type_name(mav_type: u8) -> String {
    MavType(u32::from(mav_type))
        .name()
        .and_then(|name| name.strip_prefix("MAV_TYPE_"))
        .map_or_else(|| mav_type.to_string(), str::to_owned)
}

/// `SortLogs(logs, masterdestdir)`: each log sorted under `master`, or - with none - under the
/// first log's own folder, which the C#'s first log sets for all of them.
/// `// C#: LogSort.cs:17-292`
pub fn sort_logs(logs: &[PathBuf], master: Option<&Path>) {
    let master = master.map(Path::to_path_buf).or_else(|| {
        logs.first()
            .and_then(|log| log.parent().map(Path::to_path_buf))
    });
    let Some(master) = master else {
        return;
    };
    for log in logs {
        if let Err(why) = sort_log(log, &master) {
            log::debug!("LogSort {}: {why}", log.display());
        }
    }
}

/// One log. `// C#: LogSort.cs:25-290`
fn sort_log(log: &Path, master: &Path) -> std::io::Result<()> {
    let length = mp_os::fs::metadata(log)?.len();
    // "delete 0 size files"
    if length == 0 {
        let _ = mp_os::fs::remove_file(log);
        return Ok(());
    }
    // "move small logs - most likerly invalid"
    if length <= SMALL_LOG {
        let destination = master.join(SMALL);
        mp_os::fs::create_dir_all(&destination)?;
        return move_using_mask(log, &destination);
    }
    let name = log.to_string_lossy().to_lowercase();
    if name.ends_with(".bin") || name.ends_with(".log") {
        sort_dataflash(log, master)
    } else {
        sort_mavlink(log, master, name.ends_with("tlog"))
    }
}

/// `master/[SITL/]<type>/<sysid>/[<serial>/]`. `// C#: LogSort.cs:124-133, 148-156, 265-279`
fn destination(master: &Path, sitl: bool, mav_type: u8, sysid: i64, serial: i64) -> PathBuf {
    let mut destination = master.to_path_buf();
    if sitl {
        destination.push(SITL);
    }
    destination.push(type_name(mav_type));
    destination.push(sysid.to_string());
    // "add on board serial number parameter if different than default value 0"
    if serial != 0 {
        destination.push(serial.to_string());
    }
    destination
}

/// A dataflash log: its `SYSID_THISMAV` and `BRD_SERIAL_NUM` parameters, its type from its `MSG`
/// lines (see the module's notes), and `SIM` messages marking a simulator.
/// `// C#: LogSort.cs:73-168`
fn sort_dataflash(log: &Path, master: &Path) -> std::io::Result<()> {
    let data = mp_os::fs::read(log)?;
    let mut buffer = DfLogBuffer::new(&data, &flight_mode_name);
    let parameters = buffer.items_of(&["PARM"]);
    // `int.Parse(list.First()["Value"].ToString())`, 0 where it throws.
    let mut parameter = |wanted: &str| -> i64 {
        parameters
            .iter()
            .find(|(_, item)| item.get(&mut buffer.dflog, "Name") == Some(wanted))
            .and_then(|(_, item)| item.get(&mut buffer.dflog, "Value"))
            .and_then(|value| value.trim().parse::<i32>().ok())
            .map_or(0, i64::from)
    };
    let sysid = parameter("SYSID_THISMAV");
    let serial = parameter("BRD_SERIAL_NUM");
    let messages: Vec<_> = buffer
        .items_of(&["MSG"])
        .into_iter()
        .take(MSG_LINES)
        .collect();
    // `msgs.Select(a => a["Message"].ToLower().Contains("copter")).Count() > 0`, and the same for
    // "plane" and "rover": `Count()` runs the selector over every line, so a line with no Message
    // throws before any is set, and otherwise any line at all passes all three.
    let every_line_has_words = messages
        .iter()
        .all(|(_, item)| item.get(&mut buffer.dflog, "Message").is_some());
    let mav_type = if !messages.is_empty() && every_line_has_words {
        GROUND_ROVER
    } else {
        GENERIC
    };
    let sitl = buffer.seen_message_types().iter().any(|seen| seen == "SIM");
    if sitl || (sysid != 0 && mav_type != GENERIC) {
        let destination = destination(master, sitl, mav_type, sysid, serial);
        mp_os::fs::create_dir_all(&destination)?;
        return move_using_mask(log, &destination);
    }
    Ok(())
}

/// What a pass over a telemetry log keeps. `// C#: LogSort.cs:66-71, 183`
#[derive(Debug, Default)]
struct Pass {
    sitl: bool,
    sysid: u8,
    compid: u8,
    mav_type: u8,
    serial: i64,
    /// `hblist`: each heartbeat's system, component and type.
    heartbeats: Vec<(u8, u8, u8)>,
}

impl Pass {
    /// One packet; false once it has read enough. `// C#: LogSort.cs:190-227`
    fn packet(&mut self, sysid: u8, compid: u8, msgid: u32, payload: &[u8]) -> bool {
        if self.heartbeats.len() > HEARTBEATS {
            return false;
        }
        // "no gcs packets"
        if compid == GCS_COMPONENT {
            return true;
        }
        match msgid {
            SIMSTATE | HIL_CONTROLS => self.sitl = true,
            HEARTBEAT => {
                if let Some(MavMessage::Heartbeat(heartbeat)) = MavMessage::decode(msgid, payload) {
                    self.heartbeats.push((sysid, compid, heartbeat.r#type));
                    self.sysid = sysid;
                    self.compid = compid;
                    self.mav_type = heartbeat.r#type;
                }
            }
            PARAM_VALUE => {
                // `Encoding.ASCII.GetString(param_id) == "BRD_SERIAL_NUM"`: all sixteen bytes, NULs
                // included, which the fourteen letters never equal.
                if let Some(MavMessage::ParamValue(value)) = MavMessage::decode(msgid, payload) {
                    let name: String = value.param_id.iter().map(|&b| char::from(b)).collect();
                    if name == "BRD_SERIAL_NUM" {
                        #[allow(clippy::cast_possible_truncation)] // `(int)param_value`
                        let serial = value.param_value as i32;
                        self.serial = i64::from(serial);
                    }
                }
            }
            _ => {
                if self.sysid == 0 {
                    self.sysid = sysid;
                }
                if self.compid == 0 {
                    self.compid = compid;
                }
            }
        }
        true
    }
}

/// A telemetry log, or a raw stream without timestamps (`.rlog`): its packets read until a
/// hundred heartbeats; none is `BAD`; several vehicles, the last heartbeat that is neither an
/// antenna tracker nor a ground station chosen. `// C#: LogSort.cs:174-285`
fn sort_mavlink(log: &Path, master: &Path, timestamps: bool) -> std::io::Result<()> {
    let data = mp_os::fs::read(log)?;
    let mut pass = Pass::default();
    if timestamps {
        let mut reader = TlogReader::new(&data);
        while let Some(record) = reader.next_record(&DIALECT) {
            let Ok((frame, _)) = mp_mavlink::parse(record.frame, &DIALECT) else {
                continue;
            };
            if !pass.packet(frame.sysid, frame.compid, frame.msgid, frame.payload) {
                break;
            }
        }
    } else {
        let mut decoder = mp_mavlink::FrameDecoder::new();
        let mut on_frame = |frame: &mp_mavlink::Frame<'_>| {
            pass.packet(frame.sysid, frame.compid, frame.msgid, frame.payload);
        };
        decoder.push_and_drain(&data, &DIALECT, &mut on_frame);
        decoder.flush(&DIALECT, &mut on_frame);
    }
    if pass.heartbeats.is_empty() {
        let destination = master.join(BAD);
        mp_os::fs::create_dir_all(&destination)?;
        return move_using_mask(log, &destination);
    }
    // "find most appropriate"
    let mut vehicles: Vec<u16> = pass
        .heartbeats
        .iter()
        .map(|(sysid, compid, _)| u16::from(*sysid) * 256 + u16::from(*compid))
        .collect();
    vehicles.sort_unstable();
    vehicles.dedup();
    if vehicles.len() > 1 {
        for &(sysid, compid, mav_type) in &pass.heartbeats {
            if mav_type == ANTENNA_TRACKER || mav_type == GCS {
                continue;
            }
            pass.sysid = sysid;
            pass.compid = compid;
            pass.mav_type = mav_type;
        }
    }
    let destination = destination(
        master,
        pass.sitl,
        pass.mav_type,
        i64::from(pass.sysid),
        pass.serial,
    );
    mp_os::fs::create_dir_all(&destination)?;
    move_using_mask(log, &destination)
}

/// `Directory.GetFiles(folder, "*.<extension>", ...)`: the files of that extension - in any case,
/// as Windows matches it - in `folder`, and with `recursive` in every folder under it, in name
/// order.
///
/// # Errors
///
/// A folder that cannot be read, which `GetFiles` throws for.
pub fn logs_in(folder: &Path, extension: &str, recursive: bool) -> std::io::Result<Vec<PathBuf>> {
    let mut found = Vec::new();
    let mut entries: Vec<PathBuf> = mp_os::fs::read_dir(folder)?
        .filter_map(Result::ok)
        .map(|entry| entry.path())
        .collect();
    entries.sort();
    for path in entries {
        if path.os_is_dir() {
            if recursive {
                found.extend(logs_in(&path, extension, true)?);
            }
        } else if path
            .extension()
            .is_some_and(|ext| ext.to_string_lossy().eq_ignore_ascii_case(extension))
        {
            found.push(path);
        }
    }
    Ok(found)
}

/// The temp form's Sort TLogs once its folder is chosen: `SortLogs(Directory.GetFiles(folder,
/// "*.tlog"))`, the logs sorted under the folder itself. Returns how many logs it was given.
///
/// # Errors
///
/// A folder that cannot be read, which the handler's `catch` swallows.
/// `// C#: temp.cs:224-239`
pub fn sort_folder(folder: &Path) -> std::io::Result<usize> {
    let logs = logs_in(folder, "tlog", false)?;
    sort_logs(&logs, None);
    Ok(logs.len())
}

/// The temp form's ReSort All logs: every `.tlog`, then `.bin`, `.log` and `.rlog`, in the log
/// directory and every folder under it, sorted under the log directory. Returns how many logs.
///
/// # Errors
///
/// A log directory that cannot be read, which `GetFiles` throws for and nothing catches.
/// `// C#: temp.cs:859-869`
pub fn resort_all(log_directory: &Path) -> std::io::Result<usize> {
    let mut count = 0;
    for extension in ["tlog", "bin", "log", "rlog"] {
        let logs = logs_in(log_directory, extension, true)?;
        count += logs.len();
        sort_logs(&logs, Some(log_directory));
    }
    Ok(count)
}

/// `MoveFileUsingMask`: every file in the log's folder whose name starts with the log's own, its
/// extension dropped - as Windows matches the mask, in any case - moved into `destination`, but
/// one already there. A name already taken there stops it, as `File.Move` throws.
/// `// C#: LogSort.cs:294-310`
fn move_using_mask(log: &Path, destination: &Path) -> std::io::Result<()> {
    let folder = log.parent().unwrap_or_else(|| Path::new("."));
    let stem = log
        .file_stem()
        .map(|stem| stem.to_string_lossy().to_lowercase())
        .unwrap_or_default();
    let mut files: Vec<PathBuf> = mp_os::fs::read_dir(folder)?
        .filter_map(Result::ok)
        .map(|entry| entry.path())
        .filter(|path| {
            path.os_is_file()
                && path
                    .file_name()
                    .is_some_and(|name| name.to_string_lossy().to_lowercase().starts_with(&stem))
        })
        .collect();
    files.sort();
    for file in files {
        let Some(name) = file.file_name() else {
            continue;
        };
        let target = destination.join(name);
        if file == target {
            continue;
        }
        if target.os_exists() {
            return Err(std::io::Error::new(
                std::io::ErrorKind::AlreadyExists,
                format!("{} exists", target.display()),
            ));
        }
        log::info!("Move log {} to {}", file.display(), target.display());
        mp_os::fs::rename(&file, &target)?;
    }
    Ok(())
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;

    fn testdata(name: &str) -> PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata")
            .join(name)
    }

    /// A folder of this test's own, empty.
    fn scratch(name: &str) -> PathBuf {
        let folder = mp_os::temp_dir().join(format!("mp-logsort-{}-{name}", std::process::id()));
        let _ = mp_os::fs::remove_dir_all(&folder);
        mp_os::fs::create_dir_all(&folder).unwrap();
        folder
    }

    fn copy(from: &str, to: &Path) {
        mp_os::fs::write(to, mp_os::fs::read(testdata(from)).unwrap()).unwrap();
    }

    /// The C#'s names: `MAV_TYPE_` dropped; a number the enum has not got, as itself.
    #[test]
    fn types_are_named_as_the_csharp_enum() {
        assert_eq!(type_name(2), "QUADROTOR");
        assert_eq!(type_name(10), "GROUND_ROVER");
        assert_eq!(type_name(0), "GENERIC");
        assert_eq!(type_name(250), "250");
    }

    /// Empty logs are deleted, small ones moved to SMALL with the files that share their name, and
    /// a file already there stops the move.
    #[test]
    fn empty_and_small_logs() {
        let folder = scratch("small");
        mp_os::fs::write(folder.join("empty.tlog"), b"").unwrap();
        mp_os::fs::write(folder.join("tiny.tlog"), [0u8; 100]).unwrap();
        mp_os::fs::write(folder.join("tiny.rlog"), b"r").unwrap();
        mp_os::fs::write(folder.join("other.txt"), b"o").unwrap();
        sort_logs(&[folder.join("empty.tlog"), folder.join("tiny.tlog")], None);
        assert!(!folder.join("empty.tlog").exists());
        assert!(folder.join("SMALL").join("tiny.tlog").exists());
        assert!(folder.join("SMALL").join("tiny.rlog").exists());
        assert!(folder.join("other.txt").exists());
        // Again, with a file of the name already there: left where it is.
        mp_os::fs::write(folder.join("tiny.tlog"), [0u8; 100]).unwrap();
        sort_logs(&[folder.join("tiny.tlog")], None);
        assert!(folder.join("tiny.tlog").exists());
        let _ = mp_os::fs::remove_dir_all(&folder);
    }

    /// A telemetry log of a simulated copter goes under SITL, its heartbeats' type and its system
    /// id, with the file that shares its name; one with no heartbeat goes to BAD.
    #[test]
    fn telemetry_logs_go_under_their_vehicle() {
        let folder = scratch("tlog");
        copy("mavlink/autotest.tlog", &folder.join("flight.tlog"));
        mp_os::fs::write(folder.join("flight.kml"), b"k").unwrap();
        mp_os::fs::write(folder.join("noise.tlog"), [7u8; 2000]).unwrap();
        sort_logs(
            &[folder.join("flight.tlog"), folder.join("noise.tlog")],
            Some(&folder),
        );
        let sorted = folder.join(SITL).join("QUADROTOR").join("1");
        assert!(sorted.join("flight.tlog").exists(), "{:?}", walk(&folder));
        assert!(sorted.join("flight.kml").exists());
        assert!(folder.join(BAD).join("noise.tlog").exists());
        let _ = mp_os::fs::remove_dir_all(&folder);
    }

    /// An ArduCopter dataflash log - its `MSG` lines say so - goes under GROUND_ROVER and its
    /// `SYSID_THISMAV`, as the C#'s counting puts any log with a `MSG` line; its `BRD_SERIAL_NUM`
    /// of 0 adds no folder.
    #[test]
    fn dataflash_logs_are_rovers_as_the_csharp_counts() {
        let folder = scratch("bin");
        copy("dataflash.bin", &folder.join("flight.bin"));
        sort_logs(&[folder.join("flight.bin")], Some(&folder));
        assert!(
            folder
                .join("GROUND_ROVER")
                .join("1")
                .join("flight.bin")
                .exists(),
            "{:?}",
            walk(&folder)
        );
        let _ = mp_os::fs::remove_dir_all(&folder);
    }

    /// Sort TLogs takes the folder's `.tlog`s only, and ReSort All logs every kind under the log
    /// directory, the already sorted left where they are.
    #[test]
    fn the_two_callers_find_their_logs() {
        let folder = scratch("callers");
        copy("mavlink/autotest.tlog", &folder.join("a.tlog"));
        copy("dataflash.bin", &folder.join("b.BIN"));
        assert_eq!(sort_folder(&folder).unwrap(), 1);
        assert!(folder.join("b.BIN").exists());
        let sorted = folder.join(SITL).join("QUADROTOR").join("1").join("a.tlog");
        assert!(sorted.exists());
        assert_eq!(resort_all(&folder).unwrap(), 2);
        assert!(sorted.exists());
        assert!(folder.join("GROUND_ROVER").join("1").join("b.BIN").exists());
        assert!(sort_folder(&folder.join("gone")).is_err());
        let _ = mp_os::fs::remove_dir_all(&folder);
    }

    /// Every file under `folder`.
    fn walk(folder: &Path) -> Vec<PathBuf> {
        let mut out = Vec::new();
        for entry in mp_os::fs::read_dir(folder).unwrap().flatten() {
            let path = entry.path();
            if path.is_dir() {
                out.extend(walk(&path));
            } else {
                out.push(path);
            }
        }
        out
    }
}
