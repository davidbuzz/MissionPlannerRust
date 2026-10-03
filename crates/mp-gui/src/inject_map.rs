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

//! Inject Custom Map: `BUT_InjectCustomMap_Click` on the planning screen's `panel3`. The button
//! asks for a folder (`FolderBrowserDialog`); every `.jpg`, `.jpeg` and `.png` under it whose path
//! reads `Z<zoom>/<y>/<x>.<ext>` (`\Z*([0-9]+)\([0-9]+)\([0-9]+)\.`, the Windows separators; either
//! separator here, the path being this system's) is read, saved again as JPEG and put in the tile
//! cache under the `Custom` provider, the bar stepping once a file and the button reading
//! "Cancel" meanwhile - a click on it stops the run between two files. At the end the map type
//! goes to Custom (the memory cache cleared and both maps reloaded, which a new store is here)
//! and a box, "Injecting Custom Map Results", says how many tiles each zoom got and how many in
//! all. An exception anywhere in the loop - a file that is not an image, a folder that is not
//! there - ends the run and goes to the console; here it ends the run and goes on the status line,
//! the results box following as it does in the C#.
//!
//! The run is a thread, as `Application.DoEvents()` between files keeps the C#'s window alive.
//! `Directory.GetFiles` returns files in the file system's order; each kind is sorted here.
//! `// C#: GCSViews/FlightPlanner.cs:8416-8527, GCSViews/FlightPlanner.resx (BUT_InjectCustomMap,
//! progressBarInjectCustomMap); ExtLibs/Maps/Custom.cs`

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};

use mp_tiles::TileCache;
use mp_units::TileId;

/// `BUT_InjectCustomMap.Text`.
pub const BUTTON_TEXT: &str = "Inject Custom Map";
/// `Strings.Cancel`, the button's text while a run is on.
pub const CANCEL: &str = "Cancel";
/// The results box's caption.
pub const RESULTS_TITLE: &str = "Injecting Custom Map Results";
/// `FolderBrowserDialog`'s own caption, the dialog having no description set.
pub const FOLDER_TITLE: &str = "Browse For Folder";
/// The provider the tiles go under: `Custom.Instance.DbId`, whose cache directory is its `Name`.
/// `// C#: ExtLibs/Maps/Custom.cs:45`
pub const PROVIDER: &str = "Custom";
/// `ImageFormat.Jpeg` with GDI+'s default quality.
const JPEG_QUALITY: u8 = 75;

/// `Directory.GetFiles(folder, "*.jpg", AllDirectories)`, then `*.jpeg`, then `*.png`, in that
/// order, each kind sorted by path. A folder that cannot be read is no files, as the exception
/// the C# catches leaves it.
#[must_use]
pub fn scan(folder: &Path) -> Vec<PathBuf> {
    let mut found: [Vec<PathBuf>; 3] = [Vec::new(), Vec::new(), Vec::new()];
    let mut pending = vec![folder.to_path_buf()];
    while let Some(dir) = pending.pop() {
        let Ok(entries) = std::fs::read_dir(&dir) else {
            continue;
        };
        for entry in entries.flatten() {
            let path = entry.path();
            if path.is_dir() {
                pending.push(path);
                continue;
            }
            let Some(extension) = path.extension().and_then(|e| e.to_str()) else {
                continue;
            };
            let slot = match extension.to_ascii_lowercase().as_str() {
                "jpg" => 0,
                "jpeg" => 1,
                "png" => 2,
                _ => continue,
            };
            if let Some(list) = found.get_mut(slot) {
                list.push(path);
            }
        }
    }
    found.iter_mut().for_each(|list| list.sort());
    found.into_iter().flatten().collect()
}

/// `\Z*([0-9]+)\([0-9]+)\([0-9]+)\.` on the path: the zoom, then `GPoint(X: group 3, Y: group 2)`,
/// the directory under the zoom's being the row and the file name the column. Either separator.
///
/// `// C#: GCSViews/FlightPlanner.cs:8456-8463`
#[must_use]
pub fn tile_of(path: &Path) -> Option<TileId> {
    let text = path.to_string_lossy();
    let pattern = regex::Regex::new(r"[\\/]Z*([0-9]+)[\\/]([0-9]+)[\\/]([0-9]+)\.").ok()?;
    let captures = pattern.captures(&text)?;
    let number = |index: usize| captures.get(index)?.as_str().parse::<u32>().ok();
    let zoom = u8::try_from(number(1)?).ok()?;
    Some(TileId {
        z: zoom,
        y: number(2)?,
        x: number(3)?,
    })
}

/// The run's shared state: the thread's side and the screen's.
#[derive(Debug, Default)]
struct Shared {
    /// `progressBarInjectCustomMap.Value`.
    done: AtomicUsize,
    /// `tilesCount`, by zoom.
    counts: Mutex<BTreeMap<u8, usize>>,
    /// `stopInjectCustomMap`.
    cancel: AtomicBool,
    finished: AtomicBool,
    /// The exception's text, when the loop ended on one.
    failure: Mutex<Option<String>>,
}

/// One run of the button.
#[derive(Debug)]
pub struct Injection {
    shared: Arc<Shared>,
    /// `progressBarInjectCustomMap.Maximum`: the files and one more.
    maximum: usize,
    handle: Option<std::thread::JoinHandle<()>>,
}

impl Injection {
    /// The run started over `files` into `cache`: each file read, saved as JPEG and put under
    /// the Custom provider at the tile its path names, a file whose path names none skipped; the
    /// first file that is not an image ends the run, as the C#'s exception does.
    /// `// C#: GCSViews/FlightPlanner.cs:8446-8482`
    #[must_use]
    pub fn start(files: Vec<PathBuf>, cache: TileCache) -> Self {
        let shared = Arc::new(Shared::default());
        let worker = Arc::clone(&shared);
        let maximum = files.len() + 1;
        let handle = std::thread::Builder::new()
            .name("inject-custom-map".to_owned())
            .spawn(move || {
                for file in &files {
                    if worker.cancel.load(Ordering::Acquire) {
                        break;
                    }
                    let Some(tile) = tile_of(file) else {
                        continue;
                    };
                    match reencode(file) {
                        Ok(jpeg) => {
                            // `PutImageToCache`: the C# does not look at its answer.
                            let _ = cache.write(PROVIDER, tile, &jpeg);
                        }
                        Err(why) => {
                            if let Ok(mut failure) = worker.failure.lock() {
                                *failure = Some(format!("{}: {why}", file.display()));
                            }
                            break;
                        }
                    }
                    // `if (Value < Maximum) Value++`.
                    if worker.done.load(Ordering::Acquire) < maximum {
                        worker.done.fetch_add(1, Ordering::AcqRel);
                    }
                    if let Ok(mut counts) = worker.counts.lock() {
                        *counts.entry(tile.z).or_insert(0) += 1;
                    }
                }
                worker.finished.store(true, Ordering::Release);
            })
            .ok();
        Self {
            shared,
            maximum,
            handle,
        }
    }

    /// `progressBarInjectCustomMap.Value` and `Maximum`.
    #[must_use]
    pub fn progress(&self) -> (usize, usize) {
        (self.shared.done.load(Ordering::Acquire), self.maximum)
    }

    /// The button clicked while it reads "Cancel": `stopInjectCustomMap = true`.
    pub fn cancel(&self) {
        self.shared.cancel.store(true, Ordering::Release);
    }

    /// Whether the run has ended, stopped or finished.
    #[must_use]
    pub fn finished(&self) -> bool {
        self.shared.finished.load(Ordering::Acquire)
    }

    /// The exception that ended the loop, if one did.
    #[must_use]
    pub fn failure(&self) -> Option<String> {
        self.shared
            .failure
            .lock()
            .ok()
            .and_then(|held| held.clone())
    }

    /// The results box's text: "Number of tiles loaded per zoom : ", a line a zoom in order, then
    /// the total - "tile" for one, "tiles" past one and for none.
    /// `// C#: GCSViews/FlightPlanner.cs:8514-8526`
    #[must_use]
    pub fn results(&self) -> String {
        let counts = self
            .shared
            .counts
            .lock()
            .map(|held| held.clone())
            .unwrap_or_default();
        let mut results = String::new();
        let mut count = 0;
        for (zoom, tiles) in &counts {
            results.push_str(&format!("\nZoom {zoom} : {tiles}"));
            count += tiles;
        }
        results.push_str(&format!(
            "\n\n{count} tile{} loaded !",
            if count > 1 { "s" } else { "" }
        ));
        format!("Number of tiles loaded per zoom : \n{results}")
    }

    /// Lets the thread go when the run is over.
    pub fn join(&mut self) {
        if let Some(handle) = self.handle.take() {
            let _ = handle.join();
        }
    }
}

/// `Image.FromFile(file)` then `Img.Save(tile, ImageFormat.Jpeg)`: the file decoded and written
/// again as JPEG.
fn reencode(file: &Path) -> Result<Vec<u8>, String> {
    let image = image::open(file).map_err(|why| why.to_string())?;
    let rgb = image.to_rgb8();
    let mut out = Vec::new();
    image::codecs::jpeg::JpegEncoder::new_with_quality(&mut out, JPEG_QUALITY)
        .encode(
            rgb.as_raw(),
            rgb.width(),
            rgb.height(),
            image::ExtendedColorType::Rgb8,
        )
        .map_err(|why| why.to_string())?;
    Ok(out)
}

/// What a script can see: the button's text, the bar's value and maximum, and the last run's
/// state.
pub fn record_facts(job: Option<&Injection>) {
    crate::facts::record(
        "plan.inject.button",
        if job.is_some() { CANCEL } else { BUTTON_TEXT },
    );
    let (value, maximum) = job.map_or((0, 0), Injection::progress);
    crate::facts::record("plan.inject.value", value);
    crate::facts::record("plan.inject.maximum", maximum);
}

#[cfg(test)]
mod tests {
    use super::*;

    fn png(path: &Path, colour: [u8; 3]) {
        std::fs::create_dir_all(path.parent().unwrap()).unwrap();
        let image = image::RgbImage::from_pixel(8, 8, image::Rgb(colour));
        image.save(path).unwrap();
    }

    /// The path's zoom, row and column, with either separator; a path with no such run is no
    /// tile.
    #[test]
    fn the_tile_is_read_from_the_path() {
        let tile = tile_of(Path::new("/maps/Z15/18000/30001.jpg")).unwrap();
        assert_eq!((tile.z, tile.y, tile.x), (15, 18000, 30001));
        let tile = tile_of(Path::new(r"C:\maps\15\7\9.png")).unwrap();
        assert_eq!((tile.z, tile.y, tile.x), (15, 7, 9));
        assert!(tile_of(Path::new("/maps/photo.jpg")).is_none());
        assert!(
            tile_of(Path::new("/maps/Z300/1/2.jpg")).is_none(),
            "no such zoom"
        );
    }

    /// A folder's images in the C#'s order - jpg, jpeg, png - each kind sorted; nothing from a
    /// folder that is not there.
    #[test]
    fn the_scan_lists_the_three_kinds_in_order() {
        let dir = std::env::temp_dir().join(format!("mp-inject-scan-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        png(&dir.join("Z15/1/2.png"), [1, 2, 3]);
        png(&dir.join("Z15/1/3.png"), [1, 2, 3]);
        std::fs::write(dir.join("Z15/1/1.jpg"), b"x").unwrap();
        std::fs::create_dir_all(dir.join("Z16/1")).unwrap();
        std::fs::write(dir.join("Z16/1/1.jpeg"), b"x").unwrap();
        std::fs::write(dir.join("notes.txt"), b"x").unwrap();
        // Named with "/" whatever the platform's separator, so the order is what is compared: on
        // Windows the scan's paths carry "\\", as Directory.GetFiles's do (the hosted runner,
        // 2026-10-04).
        let names: Vec<String> = scan(&dir)
            .iter()
            .map(|path| {
                path.strip_prefix(&dir)
                    .unwrap()
                    .components()
                    .map(|part| part.as_os_str().to_string_lossy().into_owned())
                    .collect::<Vec<_>>()
                    .join("/")
            })
            .collect();
        assert_eq!(
            names,
            ["Z15/1/1.jpg", "Z16/1/1.jpeg", "Z15/1/2.png", "Z15/1/3.png"]
        );
        assert!(scan(&dir.join("missing")).is_empty());
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// A run over two tiles and a stray image: the two in the cache as JPEG under Custom, the
    /// stray skipped, the bar at two of four, the results box counting the zoom's two.
    #[test]
    fn the_run_writes_the_tiles_and_counts_them() {
        let dir = std::env::temp_dir().join(format!("mp-inject-run-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        png(&dir.join("tiles/Z15/18000/30000.png"), [10, 20, 30]);
        png(&dir.join("tiles/Z15/18000/30001.png"), [40, 50, 60]);
        png(&dir.join("tiles/stray.png"), [1, 1, 1]);
        let cache = TileCache::new(dir.join("gmapcache"));
        let mut job = Injection::start(scan(&dir.join("tiles")), cache.clone());
        job.join();
        assert!(job.finished());
        assert_eq!(job.progress(), (2, 4));
        assert_eq!(
            job.results(),
            "Number of tiles loaded per zoom : \n\nZoom 15 : 2\n\n2 tiles loaded !"
        );
        let written = cache.read(
            PROVIDER,
            TileId {
                z: 15,
                x: 30001,
                y: 18000,
            },
        );
        assert!(written.is_some(), "the tile is in the cache");
        assert!(job.failure().is_none());
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// A file that is not an image ends the run where the C#'s exception does; one tile counts.
    #[test]
    fn a_file_that_is_not_an_image_ends_the_run() {
        let dir = std::env::temp_dir().join(format!("mp-inject-bad-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        png(&dir.join("Z15/1/1.png"), [10, 20, 30]);
        std::fs::create_dir_all(dir.join("Z15/1")).unwrap();
        std::fs::write(dir.join("Z15/1/2.png"), b"not a picture").unwrap();
        png(&dir.join("Z15/1/3.png"), [10, 20, 30]);
        let cache = TileCache::new(dir.join("gmapcache"));
        let mut job = Injection::start(scan(&dir), cache);
        job.join();
        assert!(job.failure().is_some());
        assert_eq!(job.progress().0, 1);
        assert_eq!(
            job.results(),
            "Number of tiles loaded per zoom : \n\nZoom 15 : 1\n\n1 tile loaded !"
        );
        let _ = std::fs::remove_dir_all(&dir);
    }
}
