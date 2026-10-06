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

//! `DashWare.Create`: a dataflash log as the CSV DashWare (a video telemetry overlay) reads - the
//! temp form's DashWare.
//!
//! The header is `GLOBAL_TimeMS`, then `<message>_<field>` for every field of every format the log
//! names, in the order the log first names each (`logformat.Values`), or only those whose names
//! are in the list; each column is followed by a comma, the last too. Then a line for each line of
//! those types in the log's order, `FMT` lines left out: its `timems` as `ToInvariantString`
//! writes a double, its fields trimmed under its own columns, and empty columns elsewhere, every
//! one comma-ended. Lines end `"\r\n"`, as `StreamWriter.WriteLine` writes on Windows.
//!
//! As the C# writes it, quirks and all: the columns follow the log's formats, not the list's
//! order; an `FMTU` line read as a format (`FMTLine` takes any line starting `FMT`) adds its
//! columns, and a line of a type whose id byte two formats share is written once for each.
//!
//! **Divergence:** `GetEnumeratorType`'s instance rules are not applied: a name the list gives
//! ending in a digit, `GPS2`, also asks the C#'s enumeration for instance 1 of `GPS`, whose lines
//! then throw `KeyNotFoundException` here when `GPS` itself was not asked for; and a name with a
//! character a regular expression's `\w` does not match asks for its leading part only. Here the
//! lines are those of exactly the names asked for.
//! `// C#: ExtLibs/Utilities/DashWare.cs:13-83; ExtLibs/Utilities/DFLogBuffer.cs:756-829`

use std::io::Write;
use std::path::Path;

use crate::dflogbuffer::DfLogBuffer;
use crate::netfmt;

/// The CSV of `log`'s lines of the types in `fmt_list` - every type with none - written to `out`.
/// Returns how many lines it wrote below the header.
///
/// # Errors
///
/// A line whose time column is not a whole number (`long.Parse` throws), or `out` failing.
/// `// C#: ExtLibs/Utilities/DashWare.cs:13-83`
pub fn create(
    log: &[u8],
    fmt_list: Option<&[String]>,
    out: &mut impl Write,
) -> Result<usize, String> {
    write_csv(log, fmt_list, out, WINDOWS_NEW_LINE)
}

/// `Environment.NewLine` on Windows, which `StreamWriter.WriteLine` ends a line with.
const WINDOWS_NEW_LINE: &str = "\r\n";

/// [`create`] with `new_line` ending each line: mono's `"\n"` for the tests, which hold the CSV to
/// what Mission Planner's own code writes under mono - a field's bytes can hold `"\n"` (a `FILE`
/// message's data), so mono's line ends cannot be turned into Windows' afterwards.
fn write_csv(
    log: &[u8],
    fmt_list: Option<&[String]>,
    out: &mut impl Write,
    new_line: &str,
) -> Result<usize, String> {
    let mut logdata = DfLogBuffer::new(log, &crate::convert::flight_mode_name);
    let mut col_list = vec!["GLOBAL_TimeMS".to_owned()];
    let mut col_start: Vec<(String, usize)> = vec![("GLOBAL".to_owned(), 0)];
    for label in logdata.dflog.labels() {
        if fmt_list.is_some_and(|list| !list.contains(&label.name)) {
            continue;
        }
        col_start.push((label.name.clone(), col_list.len()));
        for field in &label.field_names {
            col_list.push(format!("{}_{field}", label.name));
        }
    }
    let error = |e: std::io::Error| e.to_string();
    // header
    let n_cols = col_list.len();
    for item in &col_list {
        write!(out, "{item},").map_err(error)?;
    }
    out.write_all(new_line.as_bytes()).map_err(error)?;
    // lines
    let types: Vec<&str> = col_start.iter().map(|(name, _)| name.as_str()).collect();
    let mut written = 0;
    for (_, dfitem) in logdata.items_of(&types) {
        if dfitem.msgtype() == "FMT" {
            continue;
        }
        let mut sb = netfmt::double(dfitem.timems(&mut logdata.dflog)?);
        sb.push(',');
        let mut idx = 1;
        let start = col_start
            .iter()
            .find(|(name, _)| name == dfitem.msgtype())
            .map_or(0, |&(_, start)| start);
        while idx < start {
            idx += 1;
            sb.push(',');
        }
        for item in dfitem.items.iter().skip(1) {
            sb.push_str(item.as_deref().map(netfmt::trim).unwrap_or_default());
            idx += 1;
            sb.push(',');
        }
        while idx < n_cols {
            idx += 1;
            sb.push(',');
        }
        sb.push_str(new_line);
        out.write_all(sb.as_bytes()).map_err(error)?;
        written += 1;
    }
    Ok(written)
}

/// The temp form's DashWare once it has its log and list: `DashWare.Create(file, file + ".csv",
/// list)`, the CSV written over any file of that name. Returns how many lines it wrote.
///
/// # Errors
///
/// The log unreadable, the CSV unwritable, or [`create`]'s.
/// `// C#: temp.cs:952; ExtLibs/Utilities/DashWare.cs:15-36`
pub fn create_file(filein: &Path, fmt_list: Option<&[String]>) -> Result<usize, String> {
    let log = mp_os::fs::read(filein).map_err(|e| e.to_string())?;
    let mut fileout = filein.as_os_str().to_owned();
    fileout.push(".csv");
    let file = mp_os::fs::File::create(&fileout).map_err(|e| e.to_string())?;
    let mut out = std::io::BufWriter::new(file);
    let written = create(&log, fmt_list, &mut out)?;
    out.flush().map_err(|e| e.to_string())?;
    Ok(written)
}

#[cfg(test)]
mod tests {
    use super::*;

    const LOG: &[u8] = include_bytes!("../../../testdata/dataflash.bin");

    /// testdata/dashware/cases.txt's types, as `but_dashware_Click` splits them.
    fn list(types: &str) -> Vec<String> {
        types
            .split(';')
            .filter(|name| !name.is_empty())
            .map(str::to_owned)
            .collect()
    }

    /// `out` is `golden`, or the first line where they part is the panic.
    fn same(out: &[u8], golden: &[u8], name: &str) {
        let ours = String::from_utf8_lossy(out);
        let theirs = String::from_utf8_lossy(golden);
        for (number, (a, b)) in ours.split('\n').zip(theirs.split('\n')).enumerate() {
            assert_eq!(a, b, "{name}, line {}", number + 1);
        }
        assert_eq!(out.len(), golden.len(), "{name}: the length");
    }

    /// The CSV with mono's line ends, as the goldens hold it.
    fn csv(types: Option<&str>) -> Vec<u8> {
        let list = types.map(list);
        let mut out = Vec::new();
        write_csv(LOG, list.as_deref(), &mut out, "\n").expect("a CSV");
        out
    }

    /// `golden` with Windows' line ends, where no field holds a `"\n"` to be taken for one.
    fn on_windows(golden: &[u8]) -> Vec<u8> {
        assert!(!golden.contains(&b'\r'));
        let mut out = Vec::new();
        for &byte in golden {
            if byte == b'\n' {
                out.push(b'\r');
            }
            out.push(byte);
        }
        out
    }

    /// The box's offered list, and PARM and MSG's text fields with a name the log has not got:
    /// byte for byte what Mission Planner's own `DashWare.Create` writes (regen-dashware.sh).
    #[test]
    fn the_csv_is_the_csharps() {
        same(
            &csv(Some("GPS;ATT;NTUN;CTUN;MODE;BAT")),
            include_bytes!("../../../testdata/dashware/golden/default.csv"),
            "default.csv",
        );
        same(
            &csv(Some("PARM;MSG;NOSUCH")),
            include_bytes!("../../../testdata/dashware/golden/parm-msg.csv"),
            "parm-msg.csv",
        );
    }

    /// The whole log: the C#'s 17 MB, held by its SHA-256 and line count.
    #[test]
    fn the_whole_log_is_the_csharps() {
        use sha2::Digest as _;
        let out = csv(None);
        let golden = include_str!("../../../testdata/dashware/golden/all.sha256");
        let (sha, lines) = golden.trim().split_once(' ').expect("sha and lines");
        let lines: usize = lines.parse().expect("a count");
        assert_eq!(out.iter().filter(|&&b| b == b'\n').count(), lines);
        let digest: String = sha2::Sha256::digest(&out)
            .iter()
            .map(|b| format!("{b:02x}"))
            .collect();
        if digest != sha {
            let ours = std::env::temp_dir().join("mp-dashware-all.csv");
            std::fs::write(&ours, &out).ok();
            panic!(
                "the whole log's CSV is not the C#'s (regen-dashware.sh); ours is at {}",
                ours.display()
            );
        }
    }

    /// The file beside the log, `<log>.csv`, over an older one, its lines ended as on Windows.
    #[test]
    fn create_file_writes_beside_the_log() {
        let dir = std::env::temp_dir().join(format!("mp-dashware-{}", std::process::id()));
        std::fs::create_dir_all(&dir).expect("a folder");
        let log = dir.join("flight.bin");
        std::fs::write(&log, LOG).expect("the log");
        std::fs::write(dir.join("flight.bin.csv"), vec![b'x'; 200_000]).expect("an old CSV");
        let list = list("GPS;ATT;NTUN;CTUN;MODE;BAT");
        assert_eq!(create_file(&log, Some(&list)), Ok(456));
        let written = std::fs::read(dir.join("flight.bin.csv")).expect("the CSV");
        same(
            &written,
            &on_windows(include_bytes!("../../../testdata/dashware/golden/default.csv")),
            "flight.bin.csv",
        );
        std::fs::remove_dir_all(&dir).ok();
    }
}
