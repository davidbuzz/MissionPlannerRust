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

//! `GStreamer`: a pipeline played into the HUD's background - the HUD menu's Set GStreamer
//! Source and HereLink Video - and the runtime found, or on Windows downloaded, for it.
//!
//! The C# (`ExtLibs/Utilities/GStreamer.cs`) loads the GStreamer runtime's libraries and plays
//! the pipeline in its own process through the C API: `gst_parse_launch` on the text,
//! `gst_bin_get_by_name(pipeline, "outsink")` for the `appsink` the text ends in, then a loop
//! pulling each sample and handing it to the HUD as a BGRA bitmap until it is told to stop or the
//! pipeline ends.
//!
//! Here the same pipeline text is played by the runtime's own launcher, `gst-launch-1.0`, as a
//! child process, and the frames come back over its standard output. The reason is the build:
//! the C API through `gstreamer-rs` needs GStreamer's development files (`pkg-config
//! gstreamer-1.0`) wherever this is compiled, and binding the libraries by hand needs `unsafe`,
//! which this crate forbids; the launcher needs only the runtime the C# needs too. So the text
//! the user gives is the C#'s, and [`launch_pipeline`] makes one change to it: the `appsink
//! name=outsink` the C# pulls from becomes `videoconvert ! video/x-raw,format=RGBA ! pngenc
//! compression-level=0 ! multipartmux ! fdsink fd=1` - each frame an uncompressed PNG, which
//! says its own width and height, in a multipart stream read as the MJPEG source's is
//! ([`crate::multipart`]). Without an `appsink name=outsink` the pipeline plays as it is and
//! hands the HUD nothing, as the C#'s waits for its end.

use std::ffi::OsString;
use std::io::{BufRead as _, BufReader};
use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};
use std::sync::{Arc, Mutex, PoisonError};
use wasm_thread::JoinHandle;

use crate::{Feed, Frame, VideoError, convert, multipart};

/// Set GStreamer Source's pipeline when `gstreamer_url` is not set.
/// `// C#: GCSViews/FlightData.cs:4815-4817`
pub const DEFAULT_PIPELINE: &str = "videotestsrc ! video/x-raw, width=1280, height=720, framerate=30/1 ! videoconvert ! video/x-raw,format=BGRA ! appsink name=outsink";

/// Set GStreamer Source's question: `InputBox.Show("GStreamer url", ...)`.
/// `// C#: GCSViews/FlightData.cs:4819-4821`
pub const PIPELINE_TITLE: &str = "GStreamer url";

/// What that question says.
/// `// C#: GCSViews/FlightData.cs:4820`
pub const PIPELINE_TEXT: &str = "Enter the source pipeline\nEnsure the final payload is ! videoconvert ! video/x-raw,format=BGRA ! appsink name=outsink";

/// HereLink Video's address when `herelinkip` is not set.
/// `// C#: GCSViews/FlightData.cs:3157`
pub const HERELINK_IP: &str = "192.168.43.1";

/// HereLink Video's question: `InputBox.Show("herelink ip", "Enter herelink ip address", ...)`.
/// `// C#: GCSViews/FlightData.cs:3162`
pub const HERELINK_TITLE: &str = "herelink ip";

/// What that question says.
pub const HERELINK_TEXT: &str = "Enter herelink ip address";

/// The launcher's file name: what `GstLaunch` names here.
#[cfg(windows)]
pub const LAUNCHER: &str = "gst-launch-1.0.exe";
/// The launcher's file name: what `GstLaunch` names here.
#[cfg(not(windows))]
pub const LAUNCHER: &str = "gst-launch-1.0";

/// The boundary between the frames the launcher writes.
const BOUNDARY: &str = "mpframe";

/// HereLink Video's pipeline: the air unit's RTSP stream at `ip`, decoded to BGRx.
/// `// C#: GCSViews/FlightData.cs:3166-3168`
#[must_use]
pub fn herelink_pipeline(ip: &str) -> String {
    format!(
        "rtspsrc location=rtsp://{ip}:8554/fpv_stream latency=1 udp-reconnect=1 timeout=0 do-retransmission=false ! application/x-rtp ! rtph264depay ! h264parse ! queue ! avdec_h264 ! queue max-size-buffers=1 leaky=2 ! videoconvert ! video/x-raw,format=BGRx ! appsink name=outsink"
    )
}

/// The pipeline split at its links (`!`), a `!` inside quotes left alone.
fn links(pipeline: &str) -> Vec<&str> {
    let mut parts = Vec::new();
    let mut quoted = false;
    let mut start = 0;
    for (at, character) in pipeline.char_indices() {
        match character {
            '"' => quoted = !quoted,
            '!' if !quoted => {
                parts.push(pipeline.get(start..at).unwrap_or_default());
                start = at + 1;
            }
            _ => {}
        }
    }
    parts.push(pipeline.get(start..).unwrap_or_default());
    parts
}

/// Whether an element's words are the C#'s `appsink name=outsink`.
fn is_outsink(words: &[&str]) -> bool {
    words.first() == Some(&"appsink")
        && words
            .iter()
            .any(|word| matches!(*word, "name=outsink" | "name=\"outsink\""))
}

/// The pipeline the launcher plays: the user's, its `appsink name=outsink` - the element the
/// C# pulls samples from (`gst_bin_get_by_name(pipeline, "outsink")`) - replaced by the RGBA
/// PNGs on standard output the reading thread takes frames from. The `appsink`'s `sync` is kept
/// (it syncs to the clock unless told not to, and so does the sink here); its other properties,
/// `drop`, `max-buffers` and `emit-signals`, are about pulling samples, which nothing does here.
/// A word after the element's properties, a branch's `t.`, stays after the new sink.
/// `// C#: ExtLibs/Utilities/GStreamer.cs:1233-1264`
#[must_use]
pub fn launch_pipeline(pipeline: &str) -> String {
    let mut out = Vec::new();
    for link in links(pipeline) {
        let words: Vec<&str> = link.split_whitespace().collect();
        if !is_outsink(&words) {
            out.push(link.trim().to_owned());
            continue;
        }
        let properties = words
            .iter()
            .skip(1)
            .take_while(|word| word.contains('='))
            .count();
        let sync = words
            .iter()
            .skip(1)
            .take(properties)
            .find_map(|word| word.strip_prefix("sync="))
            .unwrap_or("true");
        let rest = words.get(1 + properties..).unwrap_or_default().join(" ");
        let mut sink = format!(
            "videoconvert ! video/x-raw,format=RGBA ! pngenc compression-level=0 ! multipartmux boundary={BOUNDARY} ! fdsink fd=1 sync={sync}"
        );
        if !rest.is_empty() {
            sink.push(' ');
            sink.push_str(&rest);
        }
        out.push(sink);
    }
    out.join(" ! ")
}

/// The pipeline as the launcher's arguments: split at white space outside double quotes, the
/// quotes taken off, as a shell hands `gst-launch-1.0` a pipeline typed after it. The launcher
/// escapes each argument before it parses them (`gst_parse_launchv`), so the whole text as one
/// argument would be one element's name; `gst_parse_launch`, which the C# calls, takes the
/// text whole.
#[must_use]
pub fn launch_args(pipeline: &str) -> Vec<String> {
    let mut args = Vec::new();
    let mut word = String::new();
    let mut quoted = false;
    let mut in_word = false;
    for character in pipeline.chars() {
        match character {
            '"' => {
                quoted = !quoted;
                in_word = true;
            }
            space if space.is_whitespace() && !quoted => {
                if in_word {
                    args.push(std::mem::take(&mut word));
                    in_word = false;
                }
            }
            other => {
                word.push(other);
                in_word = true;
            }
        }
    }
    if in_word {
        args.push(word);
    }
    args
}

/// Where `LookForGstreamer` looks, in its order, for the launcher: this platform's program
/// directories first (`PATH`: on Linux the launcher is `/usr/bin/gst-launch-1.0`, not beside the
/// library the C# looks for), then the C#'s own list - the two Linux library directories, the
/// program's directory, the data directory - and on Windows each fixed drive's `gstreamer`,
/// `Program Files\gstreamer` and `Program Files (x86)\gstreamer`. The C#'s
/// `CustomUserDataDirectory` and SITL's `BundledPath` are not set by anything here.
/// `// C#: ExtLibs/Utilities/GStreamer.cs:1417-1446`
#[must_use]
pub fn search_dirs(
    path: Option<&OsString>,
    program_dir: Option<&Path>,
    data_dir: Option<&Path>,
    drives: &[PathBuf],
) -> (Vec<PathBuf>, Vec<PathBuf>) {
    let on_path = path.map_or_else(Vec::new, |path| mp_os::split_paths(path).collect());
    let mut dirs = vec![
        PathBuf::from("/usr/lib/x86_64-linux-gnu"),
        PathBuf::from("/usr/lib/arm-linux-gnueabihf"),
    ];
    dirs.extend(program_dir.map(Path::to_path_buf));
    dirs.extend(data_dir.map(Path::to_path_buf));
    for drive in drives {
        dirs.push(drive.join("gstreamer"));
        dirs.push(drive.join("Program Files").join("gstreamer"));
        dirs.push(drive.join("Program Files (x86)").join("gstreamer"));
    }
    (on_path, dirs)
}

/// The fixed drives' roots on Windows (`DriveInfo.GetDrives()`, those that are ready): the
/// letters whose root is there. None elsewhere.
#[must_use]
pub fn drives() -> Vec<PathBuf> {
    if !cfg!(windows) {
        return Vec::new();
    }
    (b'C'..=b'Z')
        .map(|letter| PathBuf::from(format!("{}:\\", char::from(letter))))
        .filter(|root| root.is_dir())
        .collect()
}

/// Whether a runtime found at `path` is the process's architecture: on Windows the C# takes a
/// 64-bit runtime (`x86_64` in its path) for a 64-bit process and the other for a 32-bit one.
/// `// C#: ExtLibs/Utilities/GStreamer.cs:1456-1460`
fn fits(path: &Path, windows: bool, is64: bool) -> bool {
    if !windows {
        return true;
    }
    let lower = path.to_string_lossy().to_lowercase();
    lower.contains("_64") == is64
}

/// The launcher in `dir` or any directory under it: `Directory.GetFiles(dir, "*.*",
/// AllDirectories)`, a directory's own files before its subdirectories'.
fn find_under(dir: &Path, windows: bool, is64: bool) -> Option<PathBuf> {
    let entries = std::fs::read_dir(dir).ok()?;
    let mut subdirs = Vec::new();
    let mut found = None;
    for entry in entries.flatten() {
        let Ok(kind) = entry.file_type() else {
            continue;
        };
        let path = entry.path();
        if kind.is_dir() {
            subdirs.push(path);
        } else if found.is_none()
            && entry
                .file_name()
                .to_string_lossy()
                .eq_ignore_ascii_case(LAUNCHER)
            && fits(&path, windows, is64)
        {
            found = Some(path);
        }
    }
    if found.is_some() {
        return found;
    }
    subdirs.sort();
    subdirs
        .iter()
        .find_map(|subdir| find_under(subdir, windows, is64))
}

/// `LookForGstreamer`: the launcher's path, or None - the C#'s `""` - when there is none. A
/// program directory is looked in, not under; the C#'s directories are searched all the way
/// down.
/// `// C#: ExtLibs/Utilities/GStreamer.cs:1417-1485`
#[must_use]
pub fn look_for_gstreamer(on_path: &[PathBuf], dirs: &[PathBuf]) -> Option<PathBuf> {
    let windows = cfg!(windows);
    let is64 = cfg!(target_pointer_width = "64");
    on_path
        .iter()
        .map(|dir| dir.join(LAUNCHER))
        .find(|launcher| launcher.is_file())
        .or_else(|| {
            dirs.iter()
                .filter(|dir| dir.is_dir())
                .find_map(|dir| find_under(dir, windows, is64))
        })
}

/// `GstLaunchExists`: whether `gstlaunchexe` names a file.
/// `// C#: ExtLibs/Utilities/GStreamer.cs:1407-1415`
#[must_use]
pub fn gst_launch_exists(gst_launch: &str) -> bool {
    !gst_launch.is_empty() && Path::new(gst_launch).is_file()
}

/// `SetGSTPath`'s environment, for a runtime whose launcher is `<root>\bin\<launcher>`: its
/// `bin` and `lib` first on `PATH`, `GSTREAMER_ROOT`, `GSTREAMER_1_0_ROOT_X86_64`,
/// `GST_PLUGIN_PATH` and `GST_DEBUG_DUMP_DOT_DIR`. The C# sets them on its own process before
/// loading the libraries; here they are the launcher's. Only on Windows, where the C#'s `;`
/// separates `PATH`: a Linux runtime is where the system put it.
/// `// C#: ExtLibs/Utilities/GStreamer.cs:1364-1399`
#[must_use]
pub fn runtime_environment(
    launcher: &Path,
    path: Option<&OsString>,
    temp: &Path,
) -> Vec<(String, String)> {
    let Some(root) = launcher.parent().and_then(Path::parent) else {
        return Vec::new();
    };
    let root_text = root.display().to_string();
    let mut env = Vec::new();
    let path = path
        .map(|path| path.to_string_lossy().into_owned())
        .unwrap_or_default();
    if !path.contains(&root_text) {
        env.push((
            "PATH".to_owned(),
            format!(
                "{};{};{path}",
                root.join("bin").display(),
                root.join("lib").display()
            ),
        ));
    }
    env.push(("GSTREAMER_ROOT".to_owned(), root_text.clone()));
    env.push(("GSTREAMER_1_0_ROOT_X86_64".to_owned(), root_text));
    env.push((
        "GST_PLUGIN_PATH".to_owned(),
        root.join("lib").display().to_string(),
    ));
    env.push((
        "GST_DEBUG_DUMP_DOT_DIR".to_owned(),
        temp.display().to_string(),
    ));
    env
}

/// A pipeline playing: the launcher, and the thread reading its frames.
#[derive(Debug)]
struct Playing {
    feed: Arc<Feed>,
    child: Arc<Mutex<Option<Child>>>,
    thread: Option<JoinHandle<()>>,
    pipeline: String,
}

/// `FlightData.hudGStreamer`: at most one pipeline playing into the HUD.
#[derive(Debug, Default)]
pub struct GStreamer {
    playing: Option<Playing>,
}

impl GStreamer {
    /// `Start(stringpipeline)`: whatever plays stopped, then the launcher at `gst_launch` run
    /// on the pipeline, and a thread reading its frames until it ends or is stopped.
    /// `// C#: ExtLibs/Utilities/GStreamer.cs:1183-1192, 1194-1349`
    ///
    /// # Errors
    ///
    /// The launcher would not run: the C#'s "The file was not found at ..." for a runtime whose
    /// libraries will not load ([`VideoError::NotFound`]), or the thread would not start.
    pub fn start(&mut self, gst_launch: &Path, pipeline: &str) -> Result<(), VideoError> {
        self.stop();
        let mut command = Command::new(gst_launch);
        command
            .arg("-q")
            .args(launch_args(&launch_pipeline(pipeline)))
            .stdin(Stdio::null())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped());
        if cfg!(windows) {
            let path = std::env::var_os("PATH");
            for (name, value) in runtime_environment(gst_launch, path.as_ref(), &mp_os::temp_dir())
            {
                command.env(name, value);
            }
        }
        #[cfg(windows)]
        {
            use std::os::windows::process::CommandExt as _;
            // CREATE_NO_WINDOW: the launcher is a console program, and the C#'s pipeline had
            // no window of its own.
            command.creation_flags(0x0800_0000);
        }
        let mut child = command.spawn().map_err(|why| VideoError::NotFound {
            launch: gst_launch.display().to_string(),
            why: why.to_string(),
        })?;
        let (Some(stdout), Some(stderr)) = (child.stdout.take(), child.stderr.take()) else {
            let _ = child.kill();
            let _ = child.wait();
            return Err(VideoError::Device(
                "the launcher's output was not piped".to_owned(),
            ));
        };
        let feed = Arc::new(Feed::default());
        let child = Arc::new(Mutex::new(Some(child)));
        let spawned = {
            let feed = Arc::clone(&feed);
            let child = Arc::clone(&child);
            wasm_thread::Builder::new()
                .name("gstreamer".to_owned())
                .spawn(move || play(stdout, stderr, &feed, &child))
        };
        let thread = match spawned {
            Ok(thread) => thread,
            Err(why) => {
                kill(&child);
                return Err(VideoError::Device(why.to_string()));
            }
        };
        self.playing = Some(Playing {
            feed,
            child,
            thread: Some(thread),
            pipeline: pipeline.to_owned(),
        });
        Ok(())
    }

    /// `Stop()`: `threadShouldRun = false` and the thread waited for. The launcher is ended,
    /// which ends its output and so the thread's read; the picture goes with it.
    /// `// C#: ExtLibs/Utilities/GStreamer.cs:1491-1495`
    pub fn stop(&mut self) {
        if let Some(mut playing) = self.playing.take() {
            playing.feed.tell_stop();
            kill(&playing.child);
            if let Some(thread) = playing.thread.take() {
                let _ = thread.join();
            }
        }
    }

    /// The latest frame, while one is showing.
    #[must_use]
    pub fn latest(&self) -> Option<Arc<Frame>> {
        self.playing
            .as_ref()
            .and_then(|playing| playing.feed.latest())
    }

    /// How many frames the pipeline playing has given.
    #[must_use]
    pub fn frames(&self) -> u64 {
        self.playing
            .as_ref()
            .map_or(0, |playing| playing.feed.frames())
    }

    /// What the launcher said went wrong - its first `ERROR:`, or the parse's `WARNING:
    /// erroneous pipeline` - or a frame that would not decode. The C# logs these.
    #[must_use]
    pub fn error(&self) -> Option<String> {
        self.playing
            .as_ref()
            .and_then(|playing| playing.feed.error())
    }

    /// Whether the pipeline is still playing.
    #[must_use]
    pub fn is_running(&self) -> bool {
        self.playing.as_ref().is_some_and(|playing| {
            playing
                .thread
                .as_ref()
                .is_some_and(|thread| !thread.is_finished())
        })
    }

    /// The pipeline started last, as the user gave it.
    #[must_use]
    pub fn pipeline(&self) -> Option<&str> {
        self.playing
            .as_ref()
            .map(|playing| playing.pipeline.as_str())
    }
}

impl Drop for GStreamer {
    /// `~GStreamer()`: `Stop()`.
    fn drop(&mut self) {
        self.stop();
    }
}

/// Ends the launcher, if it has not ended, and reaps it.
fn kill(child: &Mutex<Option<Child>>) {
    if let Some(mut child) = child.lock().unwrap_or_else(PoisonError::into_inner).take() {
        let _ = child.kill();
        let _ = child.wait();
    }
}

/// The thread: frames from the launcher's output until it ends, what it says on its error
/// output kept, and at the end the picture cleared (`_onNewImage?.Invoke(null, null)`).
/// `// C#: ExtLibs/Utilities/GStreamer.cs:1295-1348`
fn play(
    stdout: std::process::ChildStdout,
    stderr: std::process::ChildStderr,
    feed: &Arc<Feed>,
    child: &Mutex<Option<Child>>,
) {
    let said = {
        let feed = Arc::clone(feed);
        wasm_thread::Builder::new()
            .name("gstreamer-stderr".to_owned())
            .spawn(move || {
                let mut noted = false;
                for line in BufReader::new(stderr).lines() {
                    let Ok(line) = line else { break };
                    // `gst_parse_launch`'s error, or the first the pipeline posts.
                    if !noted
                        && (line.starts_with("ERROR:")
                            || line.starts_with("WARNING: erroneous pipeline"))
                    {
                        feed.note(line);
                        noted = true;
                    }
                }
            })
            .ok()
    };
    let mut reader = BufReader::new(stdout);
    let mut sequence = 0u64;
    while !feed.stopping() {
        match multipart::read_part(&mut reader) {
            Ok(data) => match convert::picture_to_frame(&data, sequence) {
                Ok(frame) => {
                    sequence += 1;
                    feed.show(frame);
                }
                Err(why) => feed.note(why.to_string()),
            },
            // The launcher's output ended: the pipeline's end, its error, or Stop.
            Err(VideoError::Read(_)) => break,
            Err(why) => {
                feed.note(why.to_string());
                break;
            }
        }
    }
    feed.clear();
    kill(child);
    if let Some(said) = said {
        let _ = said.join();
    }
}

/// The runtime `DownloadGStreamer` fetches for this process: the zip's URL and its file name
/// in the data directory.
/// `// C#: ExtLibs/Utilities/GStreamer.cs:1502-1514`
#[must_use]
pub const fn runtime_download(is64: bool) -> (&'static str, &'static str) {
    if is64 {
        (
            "https://firmware.ardupilot.org/MissionPlanner/gstreamer/gstreamer-1.0-x86_64-1.14.4.zip",
            "gstreamer-1.0-x86_64-1.14.4.zip",
        )
    } else {
        (
            "https://firmware.ardupilot.org/MissionPlanner/gstreamer/gstreamer-1.0-x86-1.14.4.zip",
            "gstreamer-1.0-x86-1.14.4.zip",
        )
    }
}

/// Whether `DownloadGStreamer` has a runtime for this machine: it returns at once on ARM, and
/// the runtime it fetches is Windows'. The C# fetches that zip on Linux too, when the system's
/// library is not found, and then finds nothing in it that loads; here a Linux or macOS machine
/// is not sent a Windows runtime.
/// `// C#: ExtLibs/Utilities/GStreamer.cs:1499-1500`
#[must_use]
pub const fn can_download() -> bool {
    cfg!(windows) && !cfg!(any(target_arch = "arm", target_arch = "aarch64"))
}

/// `Download.getFilefromNet(url, saveto, status)`: whether the file arrived, saying how it goes.
pub type GetFile<'a> = &'a mut dyn FnMut(&str, &Path, &mut dyn FnMut(i32, &str)) -> bool;

/// `DownloadGStreamer(status)`: the runtime's zip fetched into the data directory
/// (`get_file`, `Download.getFilefromNet`) and extracted there, three tries, saying how it goes
/// through `status` as the C#'s progress dialog shows it.
/// `// C#: ExtLibs/Utilities/GStreamer.cs:1497-1547; Utilities/GStreamerUI.cs:8-23`
pub fn download_gstreamer(
    data_dir: &Path,
    is64: bool,
    get_file: GetFile<'_>,
    inflate: crate::zip::Inflate<'_>,
    status: &mut dyn FnMut(i32, &str),
) {
    let (url, file) = runtime_download(is64);
    let output = data_dir.join(file);
    status(0, "Downloading..");
    for _ in 0..3 {
        if !get_file(url, &output, status) {
            continue;
        }
        let extracted = std::fs::read(&output)
            .map_err(|why| why.to_string())
            .and_then(|archive| {
                status(50, "Extracting..");
                crate::zip::extract(&archive, data_dir, inflate)
            });
        match extracted {
            Ok(_) => {
                status(100, "Done.");
                return;
            }
            Err(why) => {
                status(-1, &format!("Error downloading file {why}"));
                let _ = std::fs::remove_file(&output);
                status(-1, "Retry");
            }
        }
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

    use web_time::{Duration, Instant};

    use super::*;

    /// The sink every frame leaves the launcher through.
    const SINK: &str = "videoconvert ! video/x-raw,format=RGBA ! pngenc compression-level=0 ! multipartmux boundary=mpframe ! fdsink fd=1";

    #[test]
    fn the_default_pipeline_is_the_csharps_with_its_appsink_replaced() {
        assert_eq!(
            launch_pipeline(DEFAULT_PIPELINE),
            format!(
                "videotestsrc ! video/x-raw, width=1280, height=720, framerate=30/1 ! videoconvert ! video/x-raw,format=BGRA ! {SINK} sync=true"
            )
        );
    }

    #[test]
    fn herelinks_pipeline_is_the_csharps() {
        let pipeline = herelink_pipeline(HERELINK_IP);
        assert_eq!(
            pipeline,
            "rtspsrc location=rtsp://192.168.43.1:8554/fpv_stream latency=1 udp-reconnect=1 timeout=0 do-retransmission=false ! application/x-rtp ! rtph264depay ! h264parse ! queue ! avdec_h264 ! queue max-size-buffers=1 leaky=2 ! videoconvert ! video/x-raw,format=BGRx ! appsink name=outsink"
        );
        assert!(
            launch_pipeline(&pipeline)
                .ends_with(&format!("video/x-raw,format=BGRx ! {SINK} sync=true"))
        );
    }

    /// The C#'s UDP H.264 form, as the owner's companion computers send it, with an `appsink`
    /// told not to sync and a quoted caps string holding a `!`.
    #[test]
    fn an_appsinks_sync_and_a_quoted_link_are_kept() {
        let pipeline = "udpsrc port=5600 caps=\"application/x-rtp, encoding-name=(string)H264 !\" ! rtph264depay ! avdec_h264 ! videoconvert ! appsink name=outsink sync=false drop=true";
        assert_eq!(
            launch_pipeline(pipeline),
            format!(
                "udpsrc port=5600 caps=\"application/x-rtp, encoding-name=(string)H264 !\" ! rtph264depay ! avdec_h264 ! videoconvert ! {SINK} sync=false"
            )
        );
    }

    /// Without `appsink name=outsink` the pipeline plays as it is; a branch after the sink
    /// stays after the new one.
    #[test]
    fn a_pipeline_without_the_outsink_is_left_alone() {
        let own = "videotestsrc ! autovideosink";
        assert_eq!(launch_pipeline(own), own);
        let unnamed = "videotestsrc ! appsink";
        assert_eq!(launch_pipeline(unnamed), unnamed);
        let branched =
            "videotestsrc ! tee name=t ! queue ! appsink name=outsink t. ! queue ! fakesink";
        assert_eq!(
            launch_pipeline(branched),
            format!("videotestsrc ! tee name=t ! queue ! {SINK} sync=true t. ! queue ! fakesink")
        );
    }

    #[test]
    fn the_pipeline_is_split_into_arguments_as_a_shell_splits_it() {
        assert_eq!(
            launch_args("udpsrc port=5600 caps=\"application/x-rtp, media=video\" ! fakesink"),
            [
                "udpsrc",
                "port=5600",
                "caps=application/x-rtp, media=video",
                "!",
                "fakesink"
            ]
        );
        assert_eq!(launch_args("  a   b "), ["a", "b"]);
        assert!(launch_args("").is_empty());
    }

    #[test]
    fn the_questions_are_the_csharps() {
        assert_eq!(PIPELINE_TITLE, "GStreamer url");
        assert!(PIPELINE_TEXT.starts_with("Enter the source pipeline\n"));
        assert_eq!(HERELINK_TITLE, "herelink ip");
        assert_eq!(HERELINK_TEXT, "Enter herelink ip address");
    }

    #[test]
    fn the_download_is_the_csharps() {
        assert_eq!(
            runtime_download(true),
            (
                "https://firmware.ardupilot.org/MissionPlanner/gstreamer/gstreamer-1.0-x86_64-1.14.4.zip",
                "gstreamer-1.0-x86_64-1.14.4.zip"
            )
        );
        assert_eq!(runtime_download(false).1, "gstreamer-1.0-x86-1.14.4.zip");
    }

    fn scratch(name: &str) -> PathBuf {
        let dir = mp_os::temp_dir().join(format!("mp-video-gst-{name}-{}", mp_os::process_id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    /// The zip fetched, extracted, "Done."; the launcher is then found under the data directory.
    #[test]
    fn a_downloaded_runtime_is_extracted_and_then_found() {
        let data = scratch("download");
        let launcher = format!("gstreamer/1.0/x86_64/bin/{LAUNCHER}");
        let zip = crate::zip::tests::archive(&[(launcher.as_str(), 0, b"launcher", 8)]);
        let mut said = Vec::new();
        let mut fetched = Vec::new();
        download_gstreamer(
            &data,
            true,
            &mut |url, to, status| {
                fetched.push(url.to_owned());
                status(40, "Downloading.. ETA: 1 Seconds");
                std::fs::write(to, &zip).is_ok()
            },
            &|_, _| Err("nothing is deflated".to_owned()),
            &mut |percent, words| said.push(format!("{percent} {words}")),
        );
        assert_eq!(fetched.len(), 1);
        assert_eq!(
            said,
            [
                "0 Downloading..",
                "40 Downloading.. ETA: 1 Seconds",
                "50 Extracting..",
                "100 Done."
            ]
        );
        assert!(data.join("gstreamer-1.0-x86_64-1.14.4.zip").is_file());
        let (_, dirs) = search_dirs(None, None, Some(&data), &[]);
        assert_eq!(
            look_for_gstreamer(&[], &dirs),
            Some(data.join(&launcher)),
            "{dirs:?}"
        );
        let _ = std::fs::remove_dir_all(&data);
    }

    /// Three tries, each a failure said, the zip deleted after a bad extraction.
    #[test]
    fn a_download_that_fails_is_tried_three_times() {
        let data = scratch("retry");
        let mut tries = 0;
        let mut said = Vec::new();
        download_gstreamer(
            &data,
            true,
            &mut |_, to, _| {
                tries += 1;
                std::fs::write(to, b"not a zip").is_ok()
            },
            &|_, _| Err("unused".to_owned()),
            &mut |percent, words| said.push(format!("{percent} {words}")),
        );
        assert_eq!(tries, 3);
        assert_eq!(said.iter().filter(|line| *line == "-1 Retry").count(), 3);
        assert!(said.contains(&"-1 Error downloading file not a zip archive".to_owned()));
        assert!(!data.join("gstreamer-1.0-x86_64-1.14.4.zip").exists());
        let _ = std::fs::remove_dir_all(&data);
    }

    #[test]
    fn the_launcher_is_looked_for_on_the_path_first() {
        let bin = scratch("path");
        std::fs::write(bin.join(LAUNCHER), b"").unwrap();
        let path = std::env::join_paths([bin.clone()]).unwrap();
        let (on_path, dirs) = search_dirs(Some(&path), None, None, &[]);
        assert_eq!(on_path, std::slice::from_ref(&bin));
        assert_eq!(
            dirs[..2],
            [
                PathBuf::from("/usr/lib/x86_64-linux-gnu"),
                PathBuf::from("/usr/lib/arm-linux-gnueabihf")
            ]
        );
        assert_eq!(look_for_gstreamer(&on_path, &[]), Some(bin.join(LAUNCHER)));
        assert!(gst_launch_exists(&bin.join(LAUNCHER).display().to_string()));
        assert!(!gst_launch_exists(""));
        assert_eq!(look_for_gstreamer(&[], &[bin.join("none")]), None);
        let _ = std::fs::remove_dir_all(&bin);
    }

    #[test]
    fn a_windows_drive_is_looked_in_as_the_csharp_does() {
        let (_, dirs) = search_dirs(None, None, None, &[PathBuf::from("C:\\")]);
        assert_eq!(dirs.len(), 5);
        assert!(dirs[2].ends_with("gstreamer"));
        assert!(dirs[3].to_string_lossy().contains("Program Files"));
        assert!(dirs[4].to_string_lossy().contains("Program Files (x86)"));
        // A 64-bit process takes the x86_64 runtime, a 32-bit one the other.
        assert!(fits(
            Path::new("C:\\gstreamer\\1.0\\x86_64\\bin\\x"),
            true,
            true
        ));
        assert!(!fits(
            Path::new("C:\\gstreamer\\1.0\\x86\\bin\\x"),
            true,
            true
        ));
        assert!(fits(
            Path::new("C:\\gstreamer\\1.0\\x86\\bin\\x"),
            true,
            false
        ));
        assert!(fits(Path::new("/usr/bin/x"), false, true));
    }

    #[test]
    fn the_runtimes_environment_is_set_gst_paths() {
        let launcher = Path::new("/data/gstreamer/1.0/x86_64/bin/gst-launch-1.0.exe");
        let env = runtime_environment(
            launcher,
            Some(&OsString::from("/windows")),
            Path::new("/tmp"),
        );
        let root = "/data/gstreamer/1.0/x86_64";
        // Joined with the platform's separator, as the C#'s Path.Combine joins them: `\` on
        // Windows.
        let (bin, lib) = (
            Path::new(root).join("bin").display().to_string(),
            Path::new(root).join("lib").display().to_string(),
        );
        assert_eq!(
            env,
            [
                ("PATH".to_owned(), format!("{bin};{lib};/windows")),
                ("GSTREAMER_ROOT".to_owned(), root.to_owned()),
                ("GSTREAMER_1_0_ROOT_X86_64".to_owned(), root.to_owned()),
                ("GST_PLUGIN_PATH".to_owned(), lib.clone()),
                ("GST_DEBUG_DUMP_DOT_DIR".to_owned(), "/tmp".to_owned()),
            ]
        );
        // Already on the PATH: left as it is.
        let env = runtime_environment(launcher, Some(&OsString::from(root)), Path::new("/tmp"));
        assert_eq!(env[0].0, "GSTREAMER_ROOT");
    }

    /// A launcher that is not there: the C#'s "The file was not found at".
    #[test]
    fn a_missing_launcher_is_the_csharps_file_not_found() {
        let mut gstreamer = GStreamer::default();
        let missing = Path::new("/nonexistent/gst-launch-1.0");
        let refused = gstreamer.start(missing, DEFAULT_PIPELINE).unwrap_err();
        assert!(
            refused.to_string().starts_with(
                "The file was not found at /nonexistent/gst-launch-1.0\nPlease verify permissions "
            ),
            "{refused}"
        );
        assert!(!gstreamer.is_running());
    }

    /// The installed launcher, if there is one.
    fn installed() -> Option<PathBuf> {
        let path = std::env::var_os("PATH");
        let (on_path, _) = search_dirs(path.as_ref(), None, None, &[]);
        let found = look_for_gstreamer(&on_path, &[]);
        if found.is_none() {
            eprintln!(
                "skipped: no {LAUNCHER} on the PATH - install the GStreamer runtime to run this test"
            );
        }
        found
    }

    fn wait_for(gstreamer: &GStreamer, frames: u64) {
        let deadline = Instant::now() + Duration::from_secs(20);
        while gstreamer.frames() < frames {
            assert!(
                Instant::now() < deadline,
                "{} frames, error {:?}, running {}",
                gstreamer.frames(),
                gstreamer.error(),
                gstreamer.is_running()
            );
            wasm_thread::sleep(Duration::from_millis(5));
        }
    }

    /// The real runtime, headless: a `videotestsrc` pipeline in the C#'s form plays, its frames
    /// arrive at the caps' size, and the red of `pattern=red` is the picture's every pixel;
    /// Stop ends the launcher and the picture.
    #[test]
    fn a_test_pipeline_plays_through_the_installed_runtime() {
        let Some(launcher) = installed() else {
            return;
        };
        let mut gstreamer = GStreamer::default();
        gstreamer
            .start(
                &launcher,
                "videotestsrc pattern=red ! video/x-raw, width=320, height=240, framerate=30/1 ! videoconvert ! video/x-raw,format=BGRA ! appsink name=outsink",
            )
            .unwrap();
        wait_for(&gstreamer, 5);
        let frame = gstreamer.latest().unwrap();
        assert_eq!((frame.width, frame.height), (320, 240));
        assert_eq!(frame.rgba.len(), 320 * 240 * 4);
        assert_eq!(&frame.rgba[..4], &[255, 0, 0, 255]);
        assert_eq!(&frame.rgba[frame.rgba.len() - 4..], &[255, 0, 0, 255]);
        assert!(gstreamer.is_running());
        assert_eq!(gstreamer.error(), None);
        gstreamer.stop();
        assert!(!gstreamer.is_running());
        assert!(gstreamer.latest().is_none());
    }

    /// `pattern=smpte`'s first bar is white-ish grey and its second yellow; a known pixel of
    /// each is where the pattern puts it, and a second Start replaces the first pipeline.
    #[test]
    fn a_second_start_replaces_the_first_and_the_bars_are_where_smpte_puts_them() {
        let Some(launcher) = installed() else {
            return;
        };
        let mut gstreamer = GStreamer::default();
        gstreamer
            .start(
                &launcher,
                "videotestsrc pattern=blue ! video/x-raw,width=8,height=8 ! appsink name=outsink",
            )
            .unwrap();
        wait_for(&gstreamer, 2);
        assert_eq!(&gstreamer.latest().unwrap().rgba[..4], &[0, 0, 255, 255]);
        gstreamer
            .start(&launcher, "videotestsrc pattern=smpte ! video/x-raw,width=140,height=100 ! videoconvert ! appsink name=outsink")
            .unwrap();
        wait_for(&gstreamer, 2);
        let frame = gstreamer.latest().unwrap();
        assert_eq!((frame.width, frame.height), (140, 100));
        let pixel = |x: usize, y: usize| {
            let at = (y * 140 + x) * 4;
            [frame.rgba[at], frame.rgba[at + 1], frame.rgba[at + 2]]
        };
        // The first bar white-grey, the second yellow (their level is the converter's), the
        // last of the top row's seven blue.
        let white = pixel(5, 10);
        let yellow = pixel(25, 10);
        let blue = pixel(135, 10);
        assert!(
            white.iter().all(|c| *c >= 180) && white[0] == white[2],
            "{white:?}"
        );
        assert!(
            yellow[0] >= 180 && yellow[1] >= 180 && yellow[2] < 20,
            "{yellow:?}"
        );
        assert!(blue[0] < 20 && blue[1] < 20 && blue[2] >= 180, "{blue:?}");
    }

    /// A pipeline the launcher will not parse: its words kept, the thread ended, no picture.
    #[test]
    fn a_pipeline_that_will_not_parse_keeps_the_launchers_words() {
        let Some(launcher) = installed() else {
            return;
        };
        let mut gstreamer = GStreamer::default();
        gstreamer
            .start(
                &launcher,
                "videotestsrc ! nosuchelement ! appsink name=outsink",
            )
            .unwrap();
        let deadline = Instant::now() + Duration::from_secs(20);
        while gstreamer.is_running() {
            assert!(Instant::now() < deadline);
            wasm_thread::sleep(Duration::from_millis(5));
        }
        assert_eq!(
            gstreamer.error().as_deref(),
            Some("WARNING: erroneous pipeline: no element \"nosuchelement\"")
        );
        assert_eq!(gstreamer.frames(), 0);
        assert!(gstreamer.latest().is_none());
    }
}
