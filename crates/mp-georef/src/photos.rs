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

//! The photos of a folder: `Directory.GetFiles(dir, "*.jpg")` and `"*.tif"`, each one's shutter
//! time (`getPhotoTime`), and the order `compareFileByPhotoTime` sorts them into.

use std::cmp::Ordering;
use std::collections::HashMap;
use std::path::{Path, PathBuf};

use mp_log::netfmt;

use crate::exif_read::{self, TagValue};
use crate::time::DateTime;

/// `PHOTO_FILES_FILTER`'s patterns, in the order the C# asks for them.
/// `// C#: ExtLibs/Utilities/GeoRefImageBase.cs:36, 659-664, 798-799`
pub const PHOTO_EXTENSIONS: [&str; 2] = ["jpg", "tif"];

/// `Path.DirectorySeparatorChar`.
pub const SEPARATOR: char = std::path::MAIN_SEPARATOR;

/// `Directory.GetFiles(dir, "*.jpg")` then `Directory.GetFiles(dir, "*.tif")`: the files of the
/// folder itself (not its subfolders) with each extension, as `dir` + separator + name.
///
/// On Windows the pattern matches without regard to case, and that is what is done here on every
/// system - a camera's `IMG_0001.JPG` is a photo. (Mono on Linux matches case-sensitively, and the
/// Windows quirk that `*.jpg` also finds `x.jpgx` through its 8.3 alias is not reproduced.) Within
/// one pattern the order is the file system's; every caller sorts.
///
/// # Errors
///
/// Where the folder cannot be listed.
/// `// C#: ExtLibs/Utilities/GeoRefImageBase.cs:659-664, 798-799`
pub fn list_photos(dir: &str) -> std::io::Result<Vec<String>> {
    let mut out = Vec::new();
    let mut names: Vec<String> = Vec::new();
    for entry in std::fs::read_dir(dir)? {
        let entry = entry?;
        if entry.file_type()?.is_file() {
            names.push(entry.file_name().to_string_lossy().into_owned());
        }
    }
    names.sort();
    for ext in PHOTO_EXTENSIONS {
        for name in &names {
            let matches = Path::new(name)
                .extension()
                .is_some_and(|e| e.to_string_lossy().eq_ignore_ascii_case(ext));
            if matches {
                out.push(combine(dir, name));
            }
        }
    }
    Ok(out)
}

/// `Path.Combine(dir, name)`: one separator between them.
#[must_use]
pub fn combine(dir: &str, name: &str) -> String {
    if dir.is_empty() || dir.ends_with(['/', '\\']) {
        format!("{dir}{name}")
    } else {
        format!("{dir}{SEPARATOR}{name}")
    }
}

/// `Path.GetFileName`.
#[must_use]
pub fn file_name(path: &str) -> String {
    path.rsplit(['/', '\\']).next().unwrap_or(path).to_owned()
}

/// `Path.GetFileNameWithoutExtension`: the name less its last `.` and what follows.
#[must_use]
pub fn file_stem(path: &str) -> String {
    let name = file_name(path);
    match name.rfind('.') {
        Some(at) => name.get(..at).unwrap_or_default().to_owned(),
        None => name,
    }
}

/// `Path.GetExtension`: the last `.` and what follows, or nothing.
#[must_use]
pub fn extension(path: &str) -> String {
    let name = file_name(path);
    match name.rfind('.') {
        Some(at) if at + 1 < name.len() => name.get(at..).unwrap_or_default().to_owned(),
        _ => String::new(),
    }
}

/// `filedatecache` and `getPhotoTime`.
#[derive(Debug, Clone, Default)]
pub struct PhotoTimes {
    /// `filedatecache`: only the times found are kept, so a photo that gave none is read again.
    cache: HashMap<String, DateTime>,
    /// What `DateTime.Today` is for a date no format parses.
    pub today: DateTime,
}

impl PhotoTimes {
    /// An empty cache, dating unparseable photo dates `today`.
    #[must_use]
    pub fn new(today: DateTime) -> Self {
        Self {
            cache: HashMap::new(),
            today,
        }
    }

    /// Forgets every cached time.
    pub fn clear(&mut self) {
        self.cache.clear();
    }

    /// `getPhotoTime`: the photo's `DateTimeOriginal`, else `DateTimeDigitized`, from a `.jpg` by
    /// the JPEG reader and a `.tif` by the TIFF reader; `DateTime.MinValue` when the file cannot be
    /// read, has neither tag, or the tag is not text.
    /// `// C#: ExtLibs/Utilities/GeoRefImageBase.cs:54-134`
    pub fn get(&mut self, path: &str) -> DateTime {
        if let Some(&time) = self.cache.get(path) {
            return time;
        }
        let lower = path.to_lowercase();
        let read = if lower.ends_with(".jpg") {
            std::fs::read(path)
                .ok()
                .map(|bytes| exif_read::read_jpeg(&bytes))
        } else if lower.ends_with(".tif") {
            std::fs::read(path)
                .ok()
                .map(|bytes| exif_read::read_tiff(&bytes))
        } else {
            // No reader: `foreach` over a null `Metadata` throws, and the outer catch answers.
            None
        };
        let Some(Ok(metadata)) = read else {
            return DateTime::MIN;
        };
        match metadata.date_tag() {
            Some(TagValue::Text(text)) => {
                let time = exif_read::get_date(text, self.today);
                self.cache.insert(path.to_owned(), time);
                time
            }
            // GetDate of anything but text throws, caught by getPhotoTime's outer catch.
            _ => DateTime::MIN,
        }
    }

    /// `compareFileByPhotoTime`: by time, then by the paths in the current culture's order.
    /// `// C#: ExtLibs/Utilities/GeoRefImageBase.cs:731-737`
    pub fn compare(&mut self, x: &str, y: &str) -> Ordering {
        let answer = self.get(x).cmp(&self.get(y));
        if answer == Ordering::Equal {
            netfmt::culture_compare(x, y)
        } else {
            answer
        }
    }

    /// Reads every photo's time, then sorts them by [`PhotoTimes::compare`].
    pub fn sort(&mut self, files: &mut [String]) {
        for file in files.iter() {
            self.get(file);
        }
        files.sort_by(|x, y| {
            let answer = self
                .cache
                .get(x)
                .copied()
                .unwrap_or(DateTime::MIN)
                .cmp(&self.cache.get(y).copied().unwrap_or(DateTime::MIN));
            if answer == Ordering::Equal {
                netfmt::culture_compare(x, y)
            } else {
                answer
            }
        });
    }
}

/// The folder a photo's geotagged copy goes in: `<root>/geotagged`.
/// `// C#: ExtLibs/Utilities/GeoRefImageBase.cs:1213`
#[must_use]
pub fn geotag_folder(root: &str) -> PathBuf {
    PathBuf::from(format!("{root}{SEPARATOR}geotagged"))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn path_parts_are_the_csharp_ones() {
        assert_eq!(file_name("a/b/IMG_0001.jpg"), "IMG_0001.jpg");
        assert_eq!(file_stem("a/b/IMG_0001.jpg"), "IMG_0001");
        assert_eq!(extension("a/b/IMG_0001.jpg"), ".jpg");
        assert_eq!(file_stem("a/b/noext"), "noext");
        assert_eq!(extension("a/b/trailing."), "");
        assert_eq!(combine("dir/", "x.jpg"), "dir/x.jpg");
    }
}
