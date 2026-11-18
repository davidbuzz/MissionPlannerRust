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

//! Unpacking a release's archive into the staging folder: the `.tar.gz` of the Linux and macOS
//! downloads, the `.zip` of the Windows one, as `release.yml` makes them - one folder named for
//! the release holding the planner, its licence files and `sitl-wasm`. The folder's name is
//! dropped, so what is staged lies as what is installed does. Only files and folders are taken,
//! and none that would land outside the staging folder. updates.md, section 4.6, step 3.

use std::path::{Component, Path, PathBuf};

/// Where an archive's entry goes under the staging folder: its path without the release's own
/// folder, `None` for that folder itself and for the notes macOS's tar adds (`._name`).
///
/// # Errors
///
/// A path that leaves the staging folder: absolute, or with `..` in it.
fn place(name: &str) -> Result<Option<PathBuf>, String> {
    let mut parts = Vec::new();
    for component in Path::new(name).components() {
        match component {
            Component::CurDir => {}
            Component::Normal(part) => parts.push(part),
            _ => return Err(format!("the archive holds a path outside itself: {name}")),
        }
    }
    // The first part is the release's own folder; a single name is that folder, or a file
    // beside it, which no release has.
    let Some((_, rest)) = parts.split_first() else {
        return Ok(None);
    };
    if rest.is_empty() {
        return Ok(None);
    }
    let relative: PathBuf = rest.iter().collect();
    let apple_note = relative
        .file_name()
        .is_some_and(|file| file.to_string_lossy().starts_with("._"));
    Ok((!apple_note).then_some(relative))
}

/// One file written under the staging folder, its folders made first.
fn write(into: &Path, relative: &Path, data: &mut dyn std::io::Read, mode: Option<u32>) -> Result<(), String> {
    let target = into.join(relative);
    let failed = |why: std::io::Error| format!("{}: {why}", target.display());
    if let Some(parent) = target.parent() {
        mp_os::fs::create_dir_all(parent).map_err(failed)?;
    }
    let mut file = mp_os::fs::File::create(&target).map_err(failed)?;
    std::io::copy(data, &mut file).map_err(failed)?;
    file.sync_all().map_err(failed)?;
    #[cfg(unix)]
    if let Some(mode) = mode {
        use std::os::unix::fs::PermissionsExt as _;
        mp_os::fs::set_permissions(&target, mp_os::fs::Permissions::from_mode(mode & 0o777)).map_err(failed)?;
    }
    #[cfg(not(unix))]
    let _ = mode;
    Ok(())
}

/// The archive unpacked into `into`: the files written, each relative to it.
///
/// # Errors
///
/// An archive of neither kind, a damaged one, an entry that is neither a file nor a folder (a
/// link, a device), a path that leaves `into`, a file that cannot be written, or an archive with
/// no file in it.
pub fn unpack(archive: &Path, into: &Path) -> Result<Vec<PathBuf>, String> {
    let name = archive.to_string_lossy().to_lowercase();
    let name = name.trim_end_matches(".part");
    let files = if name.ends_with(".tar.gz") || name.ends_with(".tgz") {
        unpack_tar_gz(archive, into)?
    } else if name.ends_with(".zip") {
        unpack_zip(archive, into)?
    } else {
        return Err(format!("not an archive this planner unpacks: {}", archive.display()));
    };
    if files.is_empty() {
        return Err(format!("the archive holds no files: {}", archive.display()));
    }
    Ok(files)
}

fn unpack_tar_gz(archive: &Path, into: &Path) -> Result<Vec<PathBuf>, String> {
    let damaged = |why: std::io::Error| format!("{}: {why}", archive.display());
    let file = mp_os::fs::File::open(archive).map_err(damaged)?;
    let mut tar = tar::Archive::new(flate2::read::GzDecoder::new(std::io::BufReader::new(file)));
    let mut files = Vec::new();
    for entry in tar.entries().map_err(damaged)? {
        let mut entry = entry.map_err(damaged)?;
        let name = entry.path().map_err(damaged)?.to_string_lossy().into_owned();
        let kind = entry.header().entry_type();
        if kind == tar::EntryType::Directory {
            place(&name)?;
            continue;
        }
        if !matches!(kind, tar::EntryType::Regular | tar::EntryType::Continuous) {
            return Err(format!("the archive holds something that is not a file: {name}"));
        }
        let Some(relative) = place(&name)? else {
            continue;
        };
        let mode = entry.header().mode().ok();
        write(into, &relative, &mut entry, mode)?;
        files.push(relative);
    }
    Ok(files)
}

fn unpack_zip(archive: &Path, into: &Path) -> Result<Vec<PathBuf>, String> {
    let bytes = mp_os::fs::read(archive).map_err(|why| format!("{}: {why}", archive.display()))?;
    let entries = mp_log::zip::read(&bytes).map_err(|why| format!("{}: {why}", archive.display()))?;
    let mut files = Vec::new();
    for entry in entries {
        // Windows' own archiver writes its separators as it finds them.
        let name = entry.name.replace('\\', "/");
        if name.ends_with('/') {
            place(&name)?;
            continue;
        }
        let Some(relative) = place(&name)? else {
            continue;
        };
        write(into, &relative, &mut entry.data.as_slice(), None)?;
        files.push(relative);
    }
    Ok(files)
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use crate::download::tests::scratch;
    use mp_os::fs::FsExt as _;

    /// A `.tar.gz` of those entries: a path, its bytes and its mode.
    pub(crate) fn tar_gz(entries: &[(&str, &[u8], u32)]) -> Vec<u8> {
        let mut builder =
            tar::Builder::new(flate2::write::GzEncoder::new(Vec::new(), flate2::Compression::fast()));
        for (path, bytes, mode) in entries {
            let mut header = tar::Header::new_gnu();
            header.set_size(bytes.len() as u64);
            header.set_mode(*mode);
            header.set_cksum();
            builder.append_data(&mut header, path, *bytes).unwrap();
        }
        builder.into_inner().unwrap().finish().unwrap()
    }

    #[test]
    fn a_release_archive_is_staged_without_its_own_folder() {
        let dir = scratch("unpack");
        let archive = dir.join("missionplanner-rust_0_0_2_linux-x86_64.tar.gz.part");
        mp_os::fs::write(
            &archive,
            tar_gz(&[
                ("missionplanner-rust_0_0_2_linux-x86_64/planner", b"the planner", 0o755),
                ("missionplanner-rust_0_0_2_linux-x86_64/LICENSE", b"GPL", 0o644),
                ("missionplanner-rust_0_0_2_linux-x86_64/sitl-wasm/arducopter.wasm", b"wasm", 0o644),
                ("missionplanner-rust_0_0_2_linux-x86_64/._planner", b"a note of macOS's tar", 0o644),
            ]),
        )
        .unwrap();
        let staged = dir.join("staged");
        let mut files = unpack(&archive, &staged).unwrap();
        files.sort();
        assert_eq!(
            files,
            [PathBuf::from("LICENSE"), PathBuf::from("planner"), PathBuf::from("sitl-wasm/arducopter.wasm")]
        );
        assert_eq!(mp_os::fs::read(staged.join("planner")).unwrap(), b"the planner");
        assert_eq!(mp_os::fs::read(staged.join("sitl-wasm/arducopter.wasm")).unwrap(), b"wasm");
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt as _;
            let mode = |name: &str| mp_os::fs::metadata(staged.join(name)).unwrap().permissions().mode() & 0o777;
            assert_eq!((mode("planner"), mode("LICENSE")), (0o755, 0o644));
        }
        let _ = mp_os::fs::remove_dir_all(&dir);
    }

    #[test]
    fn the_windows_zip_is_staged_the_same_way() {
        let dir = scratch("unpack-zip");
        let archive = dir.join("missionplanner-rust_0_0_2_windows-x86_64.zip");
        let entry = |name: &str, data: &[u8]| mp_log::zip::Entry { name: name.to_owned(), data: data.to_vec() };
        let bytes = mp_log::zip::write(
            &[
                entry("missionplanner-rust_0_0_2_windows-x86_64/", b""),
                entry("missionplanner-rust_0_0_2_windows-x86_64\\planner.exe", b"MZ"),
                entry("missionplanner-rust_0_0_2_windows-x86_64/sitl-wasm/bridge.mjs", b"js"),
            ],
            mp_log::zip::DosTime { year: 2026, month: 11, day: 18, hour: 0, minute: 0, second: 0 },
        )
        .unwrap();
        mp_os::fs::write(&archive, bytes).unwrap();
        let staged = dir.join("staged");
        let mut files = unpack(&archive, &staged).unwrap();
        files.sort();
        assert_eq!(files, [PathBuf::from("planner.exe"), PathBuf::from("sitl-wasm/bridge.mjs")]);
        assert_eq!(mp_os::fs::read(staged.join("planner.exe")).unwrap(), b"MZ");
        let _ = mp_os::fs::remove_dir_all(&dir);
    }

    #[test]
    fn an_archive_that_would_write_outside_the_staging_folder_is_refused() {
        let dir = scratch("unpack-escape");
        let staged = dir.join("staged");
        for path in ["release/../../escaped", "release/sub/../../../escaped"] {
            let archive = dir.join("bad.tar.gz");
            // tar's own builder refuses such a path, so the header is written by hand.
            let mut header = tar::Header::new_gnu();
            header.as_old_mut().name[..path.len()].copy_from_slice(path.as_bytes());
            header.set_size(4);
            header.set_mode(0o644);
            header.set_cksum();
            let mut builder =
                tar::Builder::new(flate2::write::GzEncoder::new(Vec::new(), flate2::Compression::fast()));
            builder.append(&header, &b"evil"[..]).unwrap();
            mp_os::fs::write(&archive, builder.into_inner().unwrap().finish().unwrap()).unwrap();
            let error = unpack(&archive, &staged).unwrap_err();
            assert!(error.starts_with("the archive holds a path outside itself"), "{error}");
        }
        assert!(!dir.join("escaped").os_exists());
        assert!(!dir.parent().unwrap().join("escaped").os_exists());

        // A link is not a file; nothing but files and folders is taken.
        let archive = dir.join("link.tar.gz");
        let mut header = tar::Header::new_gnu();
        header.set_entry_type(tar::EntryType::Symlink);
        header.set_size(0);
        header.set_cksum();
        let mut builder =
            tar::Builder::new(flate2::write::GzEncoder::new(Vec::new(), flate2::Compression::fast()));
        builder.append_link(&mut header, "release/planner", "/bin/sh").unwrap();
        mp_os::fs::write(&archive, builder.into_inner().unwrap().finish().unwrap()).unwrap();
        assert_eq!(
            unpack(&archive, &staged).unwrap_err(),
            "the archive holds something that is not a file: release/planner"
        );

        // Neither kind of archive, and an archive of nothing.
        assert!(unpack(&dir.join("planner.deb"), &staged).unwrap_err().starts_with("not an archive"));
        let empty = dir.join("empty.tar.gz");
        mp_os::fs::write(&empty, tar_gz(&[])).unwrap();
        assert!(unpack(&empty, &staged).unwrap_err().starts_with("the archive holds no files"));
        let _ = mp_os::fs::remove_dir_all(&dir);
    }
}
