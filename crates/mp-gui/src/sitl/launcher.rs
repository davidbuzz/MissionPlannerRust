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

//! Getting a simulator and starting it: `CheckandGetSITLImage`, the process half of `StartSITL`
//! and `StartSwarmChain`, and `simulator`, the list of processes killed before the next start.
//!
//! `CheckandGetSITLImage` picks its source by the system it runs on (`SITL.cs:269-456`):
//!
//! - **Windows** (anything not Linux-on-x86 and not ARM): the Cygwin build from
//!   `firmware.ardupilot.org/Tools/MissionPlanner/sitl/`, by release, saved as `<name>.exe` with
//!   the ten Cygwin DLLs beside it - [`Cygwin`].
//! - **Linux on x86 or x64**: the record for `SITL_x86_64_linux_gnu` in ArduPilot's firmware
//!   manifest, saved as `<name>` and made executable; **ARM**: the same for
//!   `SITL_arm_linux_gnueabihf` - [`ManifestSitl`].
//! - Anything else, macOS here, has no build the C# can run (it would fetch the Cygwin `.exe`):
//!   [`NotAvailable`], whose note the page shows - under the owner's ruling D14 (PLAN.md §12),
//!   until ArduPilot publishes a WebAssembly SITL (`wasm.rs`).
//!
//! - **The local WebAssembly build**, the owner's addition (2026-10-04): with SIMULATION's "try
//!   local wasm" box ticked - by default on macOS, where it is the only simulator that runs -
//!   the four pictures start the builds in `tools/sitl/wasm/` under Node, through
//!   `bridge.mjs`, which serves their SERIAL0 on tcp:127.0.0.1:5760 for the usual connect
//!   ([`LocalWasm`]; PLAN.md section 12 D21's bridge, given its word).
//!
//! Where the C# finds no manifest record on Linux it falls through to the Cygwin download, whose
//! `.exe` cannot run there; here that is the same note instead (divergence).
//!
//! The processes are started as `ProcessStartInfo` has them: the command line, the model's
//! directory as the working directory, `PATH` with the SITL directories first and `HOME` the
//! model's directory. The C# sets `PATH` and `HOME` on its own process, so every later child
//! inherits them; here they are the simulator's alone (divergence).

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use mp_os::fs::FsExt as _;
use mp_os::Lock as _;
use std::path::{Path, PathBuf};
use std::process::{Child, Command};
use std::sync::Mutex;
use std::time::Duration;

use mp_firmware::detect::DeviceInfo;
use mp_firmware::manifest::{self, Fetch, Manifest, MavType, ReleaseType};

use super::model::{self, Vehicle};
use super::wasm;

/// `sitlmasterurl`: the development build, and the Cygwin DLLs of every release.
/// `// C#: GCSViews/SITL.cs:34`
pub const SITL_MASTER_URL: &str = "https://firmware.ardupilot.org/Tools/MissionPlanner/sitl/";
/// `sitlbetaurl`. `// C#: GCSViews/SITL.cs:35`
pub const SITL_BETA_URL: &str = "https://firmware.ardupilot.org/Tools/MissionPlanner/sitl/Beta/";
/// `sitlcopterstableurl`, also the helicopter's. `// C#: GCSViews/SITL.cs:37`
pub const SITL_COPTER_STABLE_URL: &str =
    "https://firmware.ardupilot.org/Tools/MissionPlanner/sitl/CopterStable/";
/// `sitlplanestableurl`. `// C#: GCSViews/SITL.cs:38`
pub const SITL_PLANE_STABLE_URL: &str =
    "https://firmware.ardupilot.org/Tools/MissionPlanner/sitl/PlaneStable/";
/// `sitlroverstableurl`. `// C#: GCSViews/SITL.cs:39`
pub const SITL_ROVER_STABLE_URL: &str =
    "https://firmware.ardupilot.org/Tools/MissionPlanner/sitl/RoverStable/";

/// The Cygwin DLLs fetched beside the simulator. `// C#: GCSViews/SITL.cs:429-440`
pub const CYGWIN_DLLS: [&str; 10] = [
    "cygatomic-1.dll",
    "cyggcc_s-1.dll",
    "cyggcc_s-seh-1.dll",
    "cyggomp-1.dll",
    "cygiconv-2.dll",
    "cygintl-8.dll",
    "cygquadmath-0.dll",
    "cygssp-0.dll",
    "cygstdc++-6.dll",
    "cygwin1.dll",
];

/// The manifest platform of Linux on x86 and x64. `// C#: GCSViews/SITL.cs:302-316`
pub const LINUX_X64_PLATFORM: &str = "SITL_x86_64_linux_gnu";
/// The manifest platform of ARM. `// C#: GCSViews/SITL.cs:340-354`
pub const ARM_PLATFORM: &str = "SITL_arm_linux_gnueabihf";

/// `"Failed to download and start sitl\n" + ex`: what a click's `catch` shows.
/// `// C#: GCSViews/SITL.cs:171-174`
pub const FAILED_TO_DOWNLOAD_AND_START: &str = "Failed to download and start sitl";

/// The Cygwin build's folder for a file and a release: master for development, `Beta/`, and the
/// vehicle's stable folder for a stable release - the copter's for the helicopter.
/// `// C#: GCSViews/SITL.cs:395-418`
#[must_use]
pub fn cygwin_base(file: &str, release: ReleaseType) -> &'static str {
    match release {
        ReleaseType::Dev => SITL_MASTER_URL,
        ReleaseType::Beta => SITL_BETA_URL,
        ReleaseType::Official => {
            let lower = file.to_lowercase();
            let mut url = SITL_MASTER_URL;
            if lower.contains("copter") {
                url = SITL_COPTER_STABLE_URL;
            }
            if lower.contains("rover") {
                url = SITL_ROVER_STABLE_URL;
            }
            if lower.contains("plane") {
                url = SITL_PLANE_STABLE_URL;
            }
            if lower.contains("heli") {
                url = SITL_COPTER_STABLE_URL;
            }
            url
        }
    }
}

/// What the Cygwin download fetches, and the name each is saved under in the SITL directory:
/// `new Uri(url, filename)` as `<name>.exe`, then each DLL from the same folder.
/// `// C#: GCSViews/SITL.cs:420-448`
#[must_use]
pub fn cygwin_downloads(file: &str, release: ReleaseType) -> Vec<(String, String)> {
    let base = cygwin_base(file, release);
    let mut downloads = vec![(format!("{base}{file}"), format!("{}.exe", stem(file)))];
    downloads.extend(
        CYGWIN_DLLS
            .iter()
            .map(|dll| (format!("{base}{dll}"), (*dll).to_owned())),
    );
    downloads
}

/// `Path.GetFileNameWithoutExtension`.
fn stem(file: &str) -> &str {
    file.rsplit_once('.').map_or(file, |(stem, _)| stem)
}

/// The vehicle a file is for, by its name as the C# reads it - the last that matches wins.
/// `// C#: GCSViews/SITL.cs:305-313`
#[must_use]
pub fn mav_type_of(file: &str) -> MavType {
    let lower = file.to_lowercase();
    let mut mav_type = MavType::Copter;
    if lower.contains("plane") {
        mav_type = MavType::FixedWing;
    }
    if lower.contains("rover") {
        mav_type = MavType::GroundRover;
    }
    if lower.contains("heli") {
        mav_type = MavType::Helicopter;
    }
    mav_type
}

/// What a process is started with: `ProcessStartInfo` and the variables set before it.
/// `// C#: GCSViews/SITL.cs:652-668`
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Spawn {
    /// `FileName`.
    pub program: PathBuf,
    /// `Arguments`, the whole line.
    pub arguments: String,
    /// `WorkingDirectory`.
    pub working_directory: PathBuf,
    /// `PATH`.
    pub path: String,
    /// `HOME`.
    pub home: PathBuf,
}

/// Where `CheckandGetSITLImage` got to.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Image {
    /// The file to run - which may not be there, for `StartSITL` to find.
    Found(PathBuf),
    /// This desktop has no SITL build to run: the note to show.
    NotAvailable(String),
    /// Something threw: the reason.
    Failed(String),
}

/// A way to get a simulator and start it.
pub trait Launcher: Send + Sync {
    /// What it is, for the page's facts.
    fn name(&self) -> String;

    /// The note the page shows from the start, where this desktop has no simulator at all.
    fn note(&self) -> Option<String> {
        None
    }

    /// `CheckandGetSITLImage(file)`: the simulator for `file` and `release` (`None` is "Skip
    /// Download", which takes what the directory has), fetched into `dir`; `say` is the loading
    /// box's text while it downloads.
    fn image(
        &self,
        file: &str,
        release: Option<ReleaseType>,
        dir: &Path,
        fetch: &dyn Fetch,
        say: &dyn Fn(&str),
    ) -> Image;

    /// `Process.Start(exestart)`, added to `simulator`.
    ///
    /// # Errors
    /// The process could not be started: the reason.
    fn spawn(&self, spawn: &Spawn) -> Result<(), String>;

    /// `simulator.ForEach(a => a.Kill())`: every process started so far.
    fn kill_all(&self);
}

/// `simulator`: the processes started, killed together before the next start - and when the page
/// goes, as its finaliser kills them. `// C#: GCSViews/SITL.cs:55, 90-105`
#[derive(Debug, Default)]
pub struct Processes {
    /// The children.
    children: Mutex<Vec<Child>>,
}

impl Processes {
    /// Starts one and keeps it.
    ///
    /// # Errors
    /// It could not be started.
    pub fn start(&self, spawn: &Spawn) -> Result<(), String> {
        let child = start_process(spawn).map_err(|err| err.to_string())?;
        if let Ok(mut children) = self.children.os_lock() {
            children.push(child);
        }
        Ok(())
    }

    /// Kills every one; one already gone is ignored, as `catch { }` ignores it.
    pub fn kill_all(&self) {
        if let Ok(mut children) = self.children.os_lock() {
            for child in children.iter_mut() {
                let _ = child.kill();
                let _ = child.wait();
            }
            children.clear();
        }
    }
}

impl Drop for Processes {
    fn drop(&mut self) {
        self.kill_all();
    }
}

/// `Process.Start`. On Windows the line goes to the process as it is, in a console of its own -
/// `UseShellExecute` with `ProcessWindowStyle.Minimized`, less the minimising, which `std` cannot
/// ask for. Elsewhere the line is split as mono splits it, and the simulator writes to the
/// application's own console, as `UseShellExecute` leaves it; the C#'s ARM path reads each line
/// and writes it again after "SITL: ", which is not done here.
/// `// C#: GCSViews/SITL.cs:662-711`
fn start_process(spawn: &Spawn) -> std::io::Result<Child> {
    let mut command = Command::new(&spawn.program);
    #[cfg(windows)]
    {
        use std::os::windows::process::CommandExt as _;
        /// `CREATE_NEW_CONSOLE`.
        const CREATE_NEW_CONSOLE: u32 = 0x0000_0010;
        command.raw_arg(&spawn.arguments);
        command.creation_flags(CREATE_NEW_CONSOLE);
    }
    #[cfg(not(windows))]
    {
        command.args(model::split_arguments(&spawn.arguments));
    }
    command
        .current_dir(&spawn.working_directory)
        .env("PATH", &spawn.path)
        .env("HOME", &spawn.home)
        .spawn()
}

/// Windows' source: the Cygwin build from ardupilot.org.
/// `// C#: GCSViews/SITL.cs:377-455`
#[derive(Debug, Default)]
pub struct Cygwin {
    /// `simulator`.
    processes: Processes,
}

impl Launcher for Cygwin {
    fn name(&self) -> String {
        "cygwin".to_owned()
    }

    fn image(
        &self,
        file: &str,
        release: Option<ReleaseType>,
        dir: &Path,
        fetch: &dyn Fetch,
        say: &dyn Fn(&str),
    ) -> Image {
        if let Some(release) = release {
            // "kill old session - so we can overwrite if needed"
            self.kill_all();
            say(model::DOWNLOADING);
            // The C# fetches the simulator and, two at a time, the DLLs, and waits only for the
            // simulator; here they are fetched in turn. A DLL that fails is not looked at.
            for (url, name) in cygwin_downloads(file, release) {
                let _ = mp_firmware::flow::get_file_from_net(
                    fetch,
                    &url,
                    &dir.join(name),
                    &mut |_, _| {},
                );
            }
        }
        Image::Found(dir.join(format!("{}.exe", stem(file))))
    }

    fn spawn(&self, spawn: &Spawn) -> Result<(), String> {
        self.processes.start(spawn)
    }

    fn kill_all(&self) {
        self.processes.kill_all();
    }
}

/// Linux's and ARM's source: the manifest's record for the platform, made executable.
/// `// C#: GCSViews/SITL.cs:302-375`
#[derive(Debug)]
pub struct ManifestSitl {
    /// The manifest platform to take.
    platform: &'static str,
    /// `APFirmware.Manifest`: fetched once, by `GetOptions`'s `GetList()`.
    manifest: Mutex<Option<Manifest>>,
    /// `simulator`.
    processes: Processes,
}

impl ManifestSitl {
    /// One for a platform.
    #[must_use]
    pub fn new(platform: &'static str) -> Self {
        Self {
            platform,
            manifest: Mutex::new(None),
            processes: Processes::default(),
        }
    }
}

impl Launcher for ManifestSitl {
    fn name(&self) -> String {
        format!("manifest {}", self.platform)
    }

    fn image(
        &self,
        file: &str,
        release: Option<ReleaseType>,
        dir: &Path,
        fetch: &dyn Fetch,
        say: &dyn Fn(&str),
    ) -> Image {
        let mav_type = mav_type_of(file);
        let Ok(mut held) = self.manifest.os_lock() else {
            return Image::Failed("the manifest's lock is poisoned".to_owned());
        };
        // `GetOptions` starts with `GetList()`, the default URL.
        let errors = manifest::get_list(&mut held, manifest::MANIFEST_URL, false, fetch);
        let Some(catalogue) = held.as_ref() else {
            // `Manifest.Firmware` on a null manifest: a `NullReferenceException`.
            return Image::Failed(
                errors
                    .first()
                    .map_or_else(|| "no manifest".to_owned(), ToString::to_string),
            );
        };
        let device = DeviceInfo {
            board: Some(String::new()),
            hardwareid: Some(String::new()),
            ..DeviceInfo::default()
        };
        let found = catalogue
            .options(&device, release, Some(mav_type))
            .unwrap_or_default()
            .into_iter()
            .find(|fw| fw.platform.as_deref() == Some(self.platform));
        let Some(fw) = found else {
            let published = wasm::published(catalogue, Some(mav_type), release);
            return Image::NotAvailable(format!(
                "ArduPilot's firmware manifest has no {} SITL for this vehicle and version. {}",
                self.platform,
                wasm::note(&wasm::Probe::from_record(published))
            ));
        };
        let path = dir.join(stem(file));
        if release.is_some() {
            say(model::DOWNLOADING);
            if let Some(url) = fw.url.as_deref() {
                let _ = mp_firmware::flow::get_file_from_net(fetch, url, &path, &mut |_, _| {});
            }
            make_executable(&path);
        }
        Image::Found(path)
    }

    fn spawn(&self, spawn: &Spawn) -> Result<(), String> {
        self.processes.start(spawn)
    }

    fn kill_all(&self) {
        self.processes.kill_all();
    }
}

/// `chmod(path, 0755)`; a failure is logged and ignored in the C#.
/// `// C#: GCSViews/SITL.cs:323-334`
fn make_executable(path: &Path) {
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt as _;
        let _ = mp_os::fs::set_permissions(path, mp_os::fs::Permissions::from_mode(0o755));
    }
    #[cfg(not(unix))]
    {
        let _ = path;
    }
}

/// The note where nothing runs a SITL: the desktop's (the C#'s Cygwin, and the WebAssembly SITL
/// ArduPilot does not publish yet), and in a web page what does run there.
fn not_available_note() -> String {
    if cfg!(target_family = "wasm") {
        PAGE_NOTE.to_owned()
    } else {
        wasm::note(&wasm::Probe::NotYetAsked)
    }
}

/// The browser build's note with "try local wasm" unticked: the page's own SITL is the only one.
pub const PAGE_NOTE: &str = "In a web page the simulator is ArduPilot's WebAssembly SITL, which \
     the page runs itself: tick \"try local wasm\" and click a vehicle.";

/// A desktop with no SITL to run: the owner's ruling D14's note, and nothing started.
#[derive(Debug, Default)]
pub struct NotAvailable;

impl Launcher for NotAvailable {
    fn name(&self) -> String {
        "not available".to_owned()
    }

    fn note(&self) -> Option<String> {
        Some(not_available_note())
    }

    fn image(
        &self,
        _file: &str,
        _release: Option<ReleaseType>,
        _dir: &Path,
        _fetch: &dyn Fetch,
        _say: &dyn Fn(&str),
    ) -> Image {
        Image::NotAvailable(not_available_note())
    }

    fn spawn(&self, _spawn: &Spawn) -> Result<(), String> {
        Err(not_available_note())
    }

    fn kill_all(&self) {}
}

/// The folder holding the local WebAssembly builds and `bridge.mjs`: the first of
/// [`local_wasm_candidates`] that has the bridge.
#[must_use]
pub fn local_wasm_dir() -> Option<PathBuf> {
    local_wasm_candidates(
        std::env::var_os("MP_SITL_WASM").map(PathBuf::from),
        std::env::current_exe().ok(),
    )
    .into_iter()
    .find(|dir| dir.join(BRIDGE).os_is_file())
}

/// Where the WebAssembly builds may be, in order: `MP_SITL_WASM`; `sitl-wasm` beside the
/// executable, as the release archives put it; `share/missionplanner-rust/sitl-wasm` beside its
/// `bin`, as the Debian package installs it; the source tree's `tools/sitl/wasm` this was built from.
#[must_use]
pub fn local_wasm_candidates(named: Option<PathBuf>, exe: Option<PathBuf>) -> Vec<PathBuf> {
    let beside = exe.as_deref().and_then(Path::parent);
    named
        .into_iter()
        .chain(beside.map(|dir| dir.join("sitl-wasm")))
        .chain(
            beside
                .and_then(Path::parent)
                .map(|prefix| prefix.join("share/missionplanner-rust/sitl-wasm")),
        )
        .chain(std::iter::once(
            Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tools/sitl/wasm"),
        ))
        .collect()
}

/// Node: `MP_NODE`, else `node` on `PATH`, else where Homebrew and the installers put it - a
/// program started from the Finder has no shell's `PATH`.
#[must_use]
pub fn find_node() -> Option<PathBuf> {
    if let Some(node) = std::env::var_os("MP_NODE").map(PathBuf::from) {
        return node.os_is_file().then_some(node);
    }
    let name = if cfg!(windows) { "node.exe" } else { "node" };
    let path = std::env::var_os("PATH").unwrap_or_default();
    mp_os::split_paths(&path)
        .chain(
            ["/opt/homebrew/bin", "/usr/local/bin", "/usr/bin"]
                .into_iter()
                .map(PathBuf::from),
        )
        .map(|dir| dir.join(name))
        .find(|candidate| candidate.os_is_file())
}

/// The bridge script in the WebAssembly folder.
pub const BRIDGE: &str = "bridge.mjs";
/// The port the bridge serves, the C#'s SITL port (`model::SITL_LINK`).
pub const LOCAL_WASM_PORT: u16 = 5760;

/// The local WebAssembly module for a picture's file: all four vehicles are built, the heli as
/// ArduPilot's `arducopter-heli` target names it.
#[must_use]
pub fn local_wasm_module(file: &str) -> Option<&'static str> {
    match file {
        "ArduCopter.elf" => Some("arducopter.js"),
        "ArduPlane.elf" => Some("arduplane.js"),
        "ArduRover.elf" => Some("ardurover.js"),
        "ArduHeli.elf" => Some("arducopter-heli.js"),
        _ => None,
    }
}

/// The C#'s command line as the WebAssembly vehicle takes it: SERIAL0 the module's exports
/// (`--serial0 wasm`, the others off, as ArduPilot's own smoke test starts it) where the C# gives
/// `--serial0 tcp:0`, and no `--defaults` file - the module cannot read the disk, and loads its
/// vehicle's defaults from its own ROMFS. The model, home, speed-up, extra line and `--wipe` stay.
#[must_use]
pub fn local_wasm_arguments(line: &str) -> Vec<String> {
    let mut out = Vec::new();
    let mut words = model::split_arguments(line).into_iter();
    while let Some(word) = words.next() {
        if word == "--serial0" || word == "--defaults" {
            let _ = words.next();
            continue;
        }
        out.push(word);
    }
    out.extend(
        ["--serial0", "wasm", "--serial1", "none", "--serial2", "none"].map(str::to_owned),
    );
    out
}

/// A command-line word, quoted when it holds a space, for [`model::split_arguments`] and Windows.
fn quote(word: &str) -> String {
    if word.contains(' ') {
        format!("\"{word}\"")
    } else {
        word.to_owned()
    }
}

/// The owner's "try local wasm" (2026-10-04): the four vehicles' WebAssembly builds in
/// `tools/sitl/wasm/`, started under Node by `bridge.mjs`, which serves their SERIAL0 on
/// tcp:127.0.0.1:5760. Not in the C#.
#[derive(Debug)]
pub struct LocalWasm {
    /// `simulator`: the bridges started.
    processes: Processes,
    /// The port the bridge serves: [`LOCAL_WASM_PORT`], another in a test.
    port: u16,
}

impl Default for LocalWasm {
    fn default() -> Self {
        Self {
            processes: Processes::default(),
            port: LOCAL_WASM_PORT,
        }
    }
}

impl Launcher for LocalWasm {
    fn name(&self) -> String {
        "local wasm".to_owned()
    }

    fn image(
        &self,
        file: &str,
        _release: Option<ReleaseType>,
        _dir: &Path,
        _fetch: &dyn Fetch,
        _say: &dyn Fn(&str),
    ) -> Image {
        let Some(module) = local_wasm_module(file) else {
            return Image::NotAvailable(format!(
                "try local wasm: there is no local WebAssembly build of {file} in tools/sitl/wasm"
            ));
        };
        let Some(dir) = local_wasm_dir() else {
            return Image::NotAvailable(
                "try local wasm: the WebAssembly builds are not here - tools/sitl/wasm, or a \
                 folder named by MP_SITL_WASM"
                    .to_owned(),
            );
        };
        if find_node().is_none() {
            return Image::NotAvailable(
                "try local wasm: Node.js is needed to run the WebAssembly SITL, and none was \
                 found (MP_NODE names one)"
                    .to_owned(),
            );
        }
        Image::Found(dir.join(module))
    }

    /// Node running the bridge over the module `spawn.program` names, with the command line
    /// translated by [`local_wasm_arguments`].
    fn spawn(&self, spawn: &Spawn) -> Result<(), String> {
        let node = find_node().ok_or("Node.js was not found")?;
        let bridge = spawn
            .program
            .parent()
            .map(|dir| dir.join(BRIDGE))
            .ok_or("the module has no folder")?;
        let mut words = vec![
            quote(&bridge.display().to_string()),
            quote(&spawn.program.display().to_string()),
            self.port.to_string(),
        ];
        words.extend(
            local_wasm_arguments(&spawn.arguments)
                .iter()
                .map(|word| quote(word)),
        );
        self.processes.start(&Spawn {
            program: node,
            arguments: words.join(" "),
            ..spawn.clone()
        })
    }

    fn kill_all(&self) {
        self.processes.kill_all();
    }
}

/// The browser build's "try local wasm": the same four WebAssembly builds, started by the page
/// itself in a Web Worker (web/www/link.js) where the desktop starts them
/// under Node, through mp_transport::page. Its SERIAL0 is then what the start's
/// `tcp:127.0.0.1:5760` reaches, as the desktop's bridge serves it there.
#[cfg(target_family = "wasm")]
#[derive(Debug, Default)]
pub struct PageWasm;

#[cfg(target_family = "wasm")]
impl Launcher for PageWasm {
    fn name(&self) -> String {
        "local wasm".to_owned()
    }

    fn image(
        &self,
        file: &str,
        _release: Option<ReleaseType>,
        _dir: &Path,
        _fetch: &dyn Fetch,
        _say: &dyn Fn(&str),
    ) -> Image {
        local_wasm_module(file).map_or_else(
            || {
                Image::NotAvailable(format!(
                    "try local wasm: there is no WebAssembly build of {file}"
                ))
            },
            |module| Image::Found(PathBuf::from(module)),
        )
    }

    /// The page asked to start the module, with the command line [`local_wasm_arguments`]
    /// makes, as the desktop's bridge is given it.
    fn spawn(&self, spawn: &Spawn) -> Result<(), String> {
        mp_transport::page::start_sitl(
            &spawn.program.display().to_string(),
            &local_wasm_arguments(&spawn.arguments),
        );
        Ok(())
    }

    fn kill_all(&self) {
        mp_transport::page::stop_sitl();
    }
}

/// The source `CheckandGetSITLImage` takes on the system this runs on.
/// `// C#: GCSViews/SITL.cs:302-303, 340-341, 377`
#[must_use]
pub fn for_this_desktop() -> Box<dyn Launcher> {
    if cfg!(windows) {
        Box::new(Cygwin::default())
    } else if cfg!(target_os = "linux") && cfg!(any(target_arch = "x86_64", target_arch = "x86")) {
        Box::new(ManifestSitl::new(LINUX_X64_PLATFORM))
    } else if cfg!(target_os = "linux") && cfg!(any(target_arch = "arm", target_arch = "aarch64")) {
        Box::new(ManifestSitl::new(ARM_PLATFORM))
    } else {
        Box::new(NotAvailable)
    }
}

/// A picture's click, as far as it reaches past the page: what `StartSITL` is given.
#[derive(Debug, Clone, PartialEq)]
pub struct Request {
    /// The picture clicked.
    pub vehicle: Vehicle,
    /// `cmb_version`'s release; `None` is "Skip Download".
    pub release: Option<ReleaseType>,
    /// `cmb_model.Text`.
    pub model_text: String,
    /// `BuildHomeLocation` of the marker and `NUM_heading`.
    pub home: String,
    /// `num_simspeed`.
    pub speedup: i64,
    /// `txt_cmdline.Text`.
    pub cmdline: String,
    /// `chk_wipe.Checked`.
    pub wipe: bool,
    /// `sitldirectory`.
    pub dir: PathBuf,
    /// The process's `PATH`, which the simulator's begins with the SITL directories.
    pub path: String,
}

/// How a start ended.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Outcome {
    /// Started, and two seconds given: show FLIGHT DATA and connect to `link`.
    Connect {
        /// Where to connect.
        link: String,
        /// The command line the (first) simulator was given.
        arguments: String,
    },
    /// This desktop has no SITL: the note.
    NotAvailable(String),
    /// A failure the C# boxes; the owner's ruling makes it the status line's.
    Failed(String),
}

/// A picture's click after the page: `CheckandGetSITLImage`, then `StartSITL` - the image found,
/// the old simulators killed, the model and its defaults, the command line, the model's
/// directory, the start, and two seconds.
/// `// C#: GCSViews/SITL.cs:154-238, 604-713`
pub fn start(
    launcher: &dyn Launcher,
    fetch: &dyn Fetch,
    request: &Request,
    say: &dyn Fn(&str),
    wait: &dyn Fn(Duration),
) -> Outcome {
    let image = match launcher.image(
        request.vehicle.file(),
        request.release,
        &request.dir,
        fetch,
        say,
    ) {
        Image::Found(path) => path,
        Image::NotAvailable(note) => return Outcome::NotAvailable(note),
        Image::Failed(reason) => {
            return Outcome::Failed(format!("{FAILED_TO_DOWNLOAD_AND_START}\n{reason}"));
        }
    };
    // A web page has no files: its module is the page's own (PageWasm).
    if !cfg!(target_family = "wasm") && !image.os_is_file() {
        return Outcome::Failed(model::FAILED_TO_DOWNLOAD.to_owned());
    }
    launcher.kill_all();
    let model_name = model::chosen_model(request.vehicle, &request.model_text);
    let config = model::default_config(&model_name, &request.dir, fetch);
    let extra = model::extra_arguments(&config, &request.cmdline, request.wipe);
    let sim_dir = request.dir.join(&model_name);
    let _ = mp_os::fs::create_dir_all(&sim_dir);
    let arguments = model::arguments(&model_name, &request.home, request.speedup, &extra);
    let spawn = Spawn {
        program: image,
        arguments: arguments.clone(),
        working_directory: sim_dir.clone(),
        path: model::simulator_path(&request.dir, &sim_dir, &request.path),
        home: sim_dir,
    };
    if let Err(err) = launcher.spawn(&spawn) {
        return Outcome::Failed(format!("{}\n{err}", model::FAILED_TO_START));
    }
    wait(model::START_WAIT);
    Outcome::Connect {
        link: model::SITL_LINK.to_owned(),
        arguments,
    }
}

/// "Copter Swarm - Single link" (and Ctrl+S) after "how many?".
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ChainRequest {
    /// The answer to "how many?".
    pub how_many: i32,
    /// `cmb_version`'s release.
    pub release: Option<ReleaseType>,
    /// `BuildHomeLocation` of each instance's point, by instance number.
    pub homes: Vec<String>,
    /// `sitldirectory`.
    pub dir: PathBuf,
    /// `Settings.GetUserDataDirectory()`, where `sitl.bat` and `sitl1.sh` grow.
    pub data_dir: PathBuf,
    /// The process's `PATH`.
    pub path: String,
}

/// `StartSwarmChain` after its question: the old simulators killed, the copter found, and one
/// simulator per instance in a directory of its own with its `identity.parm`, each chained to the
/// next; its lines added to `sitl.bat` and `sitl1.sh`; two seconds; then FLIGHT DATA and the one
/// link to the first. Unlike `StartSITL` it does not look for the file first, and the page's
/// speed-up, model, command line and Wipe play no part.
/// `// C#: GCSViews/SITL.cs:993-1125`
pub fn start_chain(
    launcher: &dyn Launcher,
    fetch: &dyn Fetch,
    request: &ChainRequest,
    say: &dyn Fn(&str),
    wait: &dyn Fn(Duration),
) -> Outcome {
    launcher.kill_all();
    let image = match launcher.image(
        Vehicle::Multirotor.file(),
        request.release,
        &request.dir,
        fetch,
        say,
    ) {
        Image::Found(path) => path,
        Image::NotAvailable(note) => return Outcome::NotAvailable(note),
        Image::Failed(reason) => {
            return Outcome::Failed(format!("{FAILED_TO_DOWNLOAD_AND_START}\n{reason}"));
        }
    };
    let model_name = Vehicle::Multirotor.model();
    let config = model::default_config(model_name, &request.dir, fetch);
    let home = |a: i32| {
        usize::try_from(a)
            .ok()
            .and_then(|index| request.homes.get(index))
            .cloned()
            .unwrap_or_default()
    };
    let instances = model::chain(request.how_many, model_name, &config, &image, &home);
    for instance in &instances {
        let sim_dir = request.dir.join(&instance.dir);
        let _ = mp_os::fs::create_dir_all(&sim_dir);
        let _ = mp_os::fs::write(sim_dir.join("identity.parm"), &instance.identity);
        append(&request.data_dir.join("sitl.bat"), &instance.bat);
        append(&request.data_dir.join("sitl1.sh"), &instance.sh);
        let spawn = Spawn {
            program: image.clone(),
            arguments: instance.arguments.clone(),
            working_directory: sim_dir.clone(),
            path: model::simulator_path(&request.dir, &sim_dir, &request.path),
            home: sim_dir,
        };
        if let Err(err) = launcher.spawn(&spawn) {
            return Outcome::Failed(format!("{}\n{err}", model::FAILED_TO_START));
        }
    }
    wait(model::START_WAIT);
    Outcome::Connect {
        link: model::SITL_LINK.to_owned(),
        arguments: instances
            .first()
            .map(|instance| instance.arguments.clone())
            .unwrap_or_default(),
    }
}

/// `File.AppendAllText`.
fn append(path: &Path, text: &str) {
    use std::io::Write as _;
    if let Ok(mut file) = mp_os::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(path)
    {
        let _ = file.write_all(text.as_bytes());
    }
}

#[cfg(test)]
pub mod tests {
    //! The launchers against a stub web and a stub process table.

    use mp_os::fs::FsExt as _;
    use super::*;
    use std::collections::HashMap;
    use std::sync::atomic::{AtomicUsize, Ordering};

    /// The C#'s line as the WebAssembly vehicle takes it: SERIAL0 the exports, the others off,
    /// no `--defaults` file; the model, home, speed-up, extra words and `--wipe` kept.
    #[test]
    fn the_command_line_is_translated_for_the_webassembly_vehicle() {
        let extra = model::extra_arguments("/data/sitl/default_params/copter.parm", "--foo 1", true);
        let line = model::arguments("quad", "-35.36,149.16,584,353", 1, &extra);
        assert_eq!(
            local_wasm_arguments(&line),
            [
                "-Mquad",
                "-O-35.36,149.16,584,353",
                "-s1",
                "--foo",
                "1",
                "--wipe",
                "--serial0",
                "wasm",
                "--serial1",
                "none",
                "--serial2",
                "none",
            ]
        );
        assert_eq!(local_wasm_module("ArduCopter.elf"), Some("arducopter.js"));
        assert_eq!(local_wasm_module("ArduPlane.elf"), Some("arduplane.js"));
        assert_eq!(local_wasm_module("ArduRover.elf"), Some("ardurover.js"));
        assert_eq!(local_wasm_module("ArduHeli.elf"), Some("arducopter-heli.js"));
        assert_eq!(local_wasm_module("ArduSub.elf"), None);
        assert_eq!(quote("/a b/c.js"), "\"/a b/c.js\"");
    }

    /// Where the builds are looked for: the variable, beside the executable (the archives), the
    /// Debian package's share folder, then the source tree.
    #[test]
    fn the_builds_are_looked_for_where_the_releases_put_them() {
        let found = local_wasm_candidates(
            Some(PathBuf::from("/named")),
            Some(PathBuf::from("/usr/bin/planner")),
        );
        assert_eq!(found[0], PathBuf::from("/named"));
        assert_eq!(found[1], PathBuf::from("/usr/bin/sitl-wasm"));
        assert_eq!(
            found[2],
            PathBuf::from("/usr/share/missionplanner-rust/sitl-wasm")
        );
        assert!(found[3].ends_with("tools/sitl/wasm"));
        assert_eq!(local_wasm_candidates(None, None).len(), 1);
    }

    /// A file with no local build: said so, nothing started.
    #[test]
    fn a_vehicle_with_no_local_build_says_so() {
        let wasm = LocalWasm::default();
        let image = wasm.image(
            "ArduSub.elf",
            None,
            Path::new("/nowhere"),
            &StubWeb::default(),
            &|_| {},
        );
        assert!(
            matches!(&image, Image::NotAvailable(note) if note.contains("no local WebAssembly build of ArduSub.elf")),
            "{image:?}"
        );
    }

    /// The path the box takes, end to end: the copter picture's start through [`LocalWasm`] - the
    /// module found, Node running the bridge - and MAVLink from the WebAssembly vehicle on the
    /// bridge's port. Skipped where Node or the builds are not here. (The plane, rover and heli
    /// builds are checked the same way by hand: tools/sitl/wasm/README.md.)
    #[test]
    fn the_local_copter_speaks_mavlink_on_the_bridges_port() {
        if find_node().is_none() || local_wasm_dir().is_none() {
            eprintln!("skipped: needs Node and tools/sitl/wasm (MP_NODE, MP_SITL_WASM)");
            return;
        }
        let port = std::net::TcpListener::bind("127.0.0.1:0")
            .and_then(|listener| listener.local_addr())
            .map(|address| address.port())
            .expect("a free port");
        let wasm = LocalWasm {
            processes: Processes::default(),
            port,
        };
        let dir = scratch("local-wasm");
        let request = Request {
            vehicle: Vehicle::Multirotor,
            release: None,
            model_text: String::new(),
            home: "-35.363262,149.165237,584,353".to_owned(),
            speedup: 1,
            cmdline: String::new(),
            wipe: false,
            dir,
            path: std::env::var("PATH").unwrap_or_default(),
        };
        let outcome = start(&wasm, &StubWeb::default(), &request, &|_| {}, &|_| {});
        assert!(matches!(outcome, Outcome::Connect { .. }), "{outcome:?}");
        let deadline = web_time::Instant::now() + Duration::from_secs(60);
        let mut seen = Vec::new();
        while web_time::Instant::now() < deadline && !seen.contains(&0xFD) {
            if let Ok(mut socket) = std::net::TcpStream::connect(("127.0.0.1", port)) {
                let _ = socket.set_read_timeout(Some(Duration::from_secs(2)));
                let mut buffer = [0u8; 4096];
                while web_time::Instant::now() < deadline {
                    match std::io::Read::read(&mut socket, &mut buffer) {
                        Ok(0) | Err(_) => break,
                        Ok(read) => {
                            seen.extend_from_slice(&buffer[..read]);
                            if seen.contains(&0xFD) {
                                break;
                            }
                        }
                    }
                }
            } else {
                wasm_thread::sleep(Duration::from_millis(200));
            }
        }
        wasm.kill_all();
        assert!(
            seen.contains(&0xFD),
            "no MAVLink 2 from the WebAssembly copter in 60 s ({} bytes)",
            seen.len()
        );
    }

    /// The WebAssembly copter's parameters outlive it, as a native SITL's do (the owner's bug of
    /// 2026-10-04: one written to it was gone after a stop and a start). Written, the copter
    /// stopped as the planner stops it - killed, with no last word - and started again in its
    /// folder, the parameter holds what was written: kept in the folder's `eeprom.bin`, which the
    /// bridge puts back into the module before the vehicle starts and saves whenever it changes
    /// (`tools/sitl/wasm/bridge.mjs`). Skipped where Node or the builds are not here.
    #[test]
    fn the_local_copters_parameters_survive_a_restart() {
        if find_node().is_none() || local_wasm_dir().is_none() {
            eprintln!("skipped: needs Node and tools/sitl/wasm (MP_NODE, MP_SITL_WASM)");
            return;
        }
        // RTL_LOIT_TIME rather than RTL_ALT, which this master has renamed RTL_ALT_M.
        const NAME: &str = "RTL_LOIT_TIME";
        const WRITTEN: f32 = 4_321.0;
        let port = std::net::TcpListener::bind("127.0.0.1:0")
            .and_then(|listener| listener.local_addr())
            .map(|address| address.port())
            .expect("a free port");
        let wasm = LocalWasm {
            processes: Processes::default(),
            port,
        };
        let dir = scratch("local-wasm-eeprom");
        let request = Request {
            vehicle: Vehicle::Multirotor,
            release: None,
            model_text: String::new(),
            home: "-35.363262,149.165237,584,353".to_owned(),
            speedup: 1,
            cmdline: String::new(),
            wipe: false,
            dir: dir.clone(),
            path: std::env::var("PATH").unwrap_or_default(),
        };
        // The parameter as the vehicle reports it, once its list is in; `written` sets it first.
        let session = |written: Option<f32>| -> Option<f32> {
            let outcome = start(&wasm, &StubWeb::default(), &request, &|_| {}, &|_| {});
            assert!(matches!(outcome, Outcome::Connect { .. }), "{outcome:?}");
            // Node takes a moment to listen; the planner's connect retries too.
            let deadline = web_time::Instant::now() + Duration::from_secs(90);
            let link = loop {
                match mp_link::Link::connect(
                    &format!("tcp:127.0.0.1:{port}"),
                    mp_link::LinkConfig {
                        stream_rate_hz: 0,
                        ..mp_link::LinkConfig::default()
                    },
                ) {
                    Ok(link) => break link,
                    Err(err) => {
                        assert!(
                            web_time::Instant::now() < deadline,
                            "the bridge's port never opened: {err:?}"
                        );
                        wasm_thread::sleep(Duration::from_millis(250));
                    }
                }
            };
            let value = |link: &mp_link::Link| {
                let (id, _) = link.primary_vehicle()?;
                link.params(id)?
                    .get(NAME)
                    .map(mp_params::ParamValue::to_param_value_field)
            };
            let mut asked = false;
            let mut set = false;
            let mut last = None;
            while web_time::Instant::now() < deadline {
                if let Some((id, _)) = link.primary_vehicle() {
                    if !asked {
                        asked = link.download_params(id);
                    }
                    last = value(&link);
                    match (written, last) {
                        (Some(want), Some(_)) if !set => {
                            let _ = link.set_param(id, NAME, f64::from(want), true);
                            set = true;
                        }
                        (Some(want), Some(now)) if (now - want).abs() < f32::EPSILON => break,
                        (None, Some(_)) => break,
                        _ => {}
                    }
                }
                wasm_thread::sleep(Duration::from_millis(200));
            }
            // Past the bridge's half-second save, then stopped as the planner stops it.
            wasm_thread::sleep(Duration::from_secs(2));
            drop(link);
            wasm.kill_all();
            last
        };
        let after_writing = session(Some(WRITTEN));
        let after_restart = session(None);
        let saved = mp_os::fs::read_dir(&dir)
            .into_iter()
            .flatten()
            .flatten()
            .any(|entry| entry.path().join("eeprom.bin").os_is_file());
        let _ = mp_os::fs::remove_dir_all(&dir);
        assert_eq!(after_writing, Some(WRITTEN), "the write was not echoed");
        assert!(saved, "no eeprom.bin in the copter's folder");
        assert_eq!(
            after_restart,
            Some(WRITTEN),
            "the parameter did not survive the restart"
        );
    }

    /// A web of fixed files that records what was asked of it.
    #[derive(Debug, Default)]
    pub struct StubWeb {
        /// What each URL serves.
        pub files: HashMap<String, Vec<u8>>,
        /// Every URL asked for, in order.
        pub asked: Mutex<Vec<String>>,
    }

    impl StubWeb {
        /// Serves `body` at `url`.
        pub fn serve(mut self, url: &str, body: &[u8]) -> Self {
            self.files.insert(url.to_owned(), body.to_vec());
            self
        }

        /// The URLs asked for so far.
        pub fn asked(&self) -> Vec<String> {
            self.asked
                .os_lock()
                .map(|asked| asked.clone())
                .unwrap_or_default()
        }
    }

    impl Fetch for StubWeb {
        fn get(&self, url: &str) -> Result<Vec<u8>, String> {
            if let Ok(mut asked) = self.asked.os_lock() {
                asked.push(url.to_owned());
            }
            self.files
                .get(url)
                .cloned()
                .ok_or_else(|| format!("404 {url}"))
        }
    }

    /// A launcher that starts nothing: it hands back a given image and records every spawn.
    #[derive(Debug)]
    pub struct StubLauncher {
        /// What `image` answers.
        pub image: Image,
        /// Every spawn asked for.
        pub spawns: Mutex<Vec<Spawn>>,
        /// How many times everything was killed.
        pub kills: AtomicUsize,
        /// Whether a spawn fails.
        pub refuse: bool,
    }

    impl StubLauncher {
        /// One whose image is `image`.
        pub fn new(image: Image) -> Self {
            Self {
                image,
                spawns: Mutex::new(Vec::new()),
                kills: AtomicUsize::new(0),
                refuse: false,
            }
        }

        /// The spawns so far.
        pub fn spawns(&self) -> Vec<Spawn> {
            self.spawns.os_lock().map(|s| s.clone()).unwrap_or_default()
        }
    }

    impl Launcher for StubLauncher {
        fn name(&self) -> String {
            "stub".to_owned()
        }

        fn image(
            &self,
            _file: &str,
            _release: Option<ReleaseType>,
            _dir: &Path,
            _fetch: &dyn Fetch,
            _say: &dyn Fn(&str),
        ) -> Image {
            self.image.clone()
        }

        fn spawn(&self, spawn: &Spawn) -> Result<(), String> {
            if self.refuse {
                return Err("The system cannot find the file specified".to_owned());
            }
            if let Ok(mut spawns) = self.spawns.os_lock() {
                spawns.push(spawn.clone());
            }
            Ok(())
        }

        fn kill_all(&self) {
            self.kills.fetch_add(1, Ordering::Relaxed);
        }
    }

    /// A directory of its own under the system's temporary one, emptied first.
    pub fn scratch(name: &str) -> PathBuf {
        let dir = mp_os::temp_dir().join(format!("mp-gui-sitl-{}-{name}", mp_os::process_id()));
        let _ = mp_os::fs::remove_dir_all(&dir);
        let _ = mp_os::fs::create_dir_all(&dir);
        dir
    }

    fn request(dir: &Path, vehicle: Vehicle) -> Request {
        Request {
            vehicle,
            release: Some(ReleaseType::Official),
            model_text: String::new(),
            home: "-35.3633515,149.1652412,584,0".to_owned(),
            speedup: 1,
            cmdline: String::new(),
            wipe: false,
            dir: dir.to_path_buf(),
            path: "/usr/bin".to_owned(),
        }
    }

    const VEHICLEINFO: &str = "class VehicleInfo(object):\n    def __init__(self):\n        self.options = {\n    \"ArduCopter\": {\n        \"frames\": {\n            \"+\": {\n                \"default_params_filename\": \"default_params/copter.parm\",\n            },\n            \"hexa\": {\n                \"default_params_filename\": [\"default_params/copter.parm\", \"default_params/copter-hexa.parm\"],\n            },\n        },\n    },\n    \"ArduPlane\": {\n        \"frames\": {\n            \"plane\": {\n                \"default_params_filename\": \"default_params/plane.parm\",\n                \"external\": True,\n            },\n        },\n    },\n}\n";

    fn autotest_web() -> StubWeb {
        StubWeb::default()
            .serve(model::SIM_VEHICLE_URL, b"# no frames here\n")
            .serve(model::VEHICLEINFO_URL, VEHICLEINFO.as_bytes())
            .serve(
                &format!("{}default_params/copter.parm", model::AUTOTEST_URL),
                b"FRAME_CLASS 1\n",
            )
            .serve(
                &format!("{}default_params/copter-hexa.parm", model::AUTOTEST_URL),
                b"FRAME_CLASS 2\n",
            )
            .serve(
                &format!("{}default_params/plane.parm", model::AUTOTEST_URL),
                b"KFF_RDDRMIX 0.5\n",
            )
    }

    /// The Windows launcher's exact URLs for every vehicle and release, and the names they are
    /// saved under.
    #[test]
    fn the_cygwin_urls_are_the_csharps() {
        let copter = cygwin_downloads("ArduCopter.elf", ReleaseType::Official);
        assert_eq!(
            copter.first(),
            Some(&(
                "https://firmware.ardupilot.org/Tools/MissionPlanner/sitl/CopterStable/ArduCopter.elf"
                    .to_owned(),
                "ArduCopter.exe".to_owned()
            ))
        );
        assert_eq!(copter.len(), 11);
        assert_eq!(
            copter.last(),
            Some(&(
                "https://firmware.ardupilot.org/Tools/MissionPlanner/sitl/CopterStable/cygwin1.dll"
                    .to_owned(),
                "cygwin1.dll".to_owned()
            ))
        );
        let cases = [
            (
                "ArduPlane.elf",
                ReleaseType::Official,
                "sitl/PlaneStable/ArduPlane.elf",
            ),
            (
                "ArduRover.elf",
                ReleaseType::Official,
                "sitl/RoverStable/ArduRover.elf",
            ),
            (
                "ArduHeli.elf",
                ReleaseType::Official,
                "sitl/CopterStable/ArduHeli.elf",
            ),
            ("ArduHeli.elf", ReleaseType::Dev, "sitl/ArduHeli.elf"),
            (
                "ArduCopter.elf",
                ReleaseType::Beta,
                "sitl/Beta/ArduCopter.elf",
            ),
            ("ArduPlane.elf", ReleaseType::Dev, "sitl/ArduPlane.elf"),
        ];
        for (file, release, tail) in cases {
            let first = cygwin_downloads(file, release)
                .into_iter()
                .next()
                .map(|(url, _)| url);
            assert_eq!(
                first,
                Some(format!(
                    "https://firmware.ardupilot.org/Tools/MissionPlanner/{tail}"
                )),
                "{file} {release}"
            );
        }
        assert_eq!(mav_type_of("ArduPlane.elf"), MavType::FixedWing);
        assert_eq!(mav_type_of("ArduRover.elf"), MavType::GroundRover);
        assert_eq!(mav_type_of("ArduCopter.elf"), MavType::Copter);
        assert_eq!(mav_type_of("ArduHeli.elf"), MavType::Helicopter);
    }

    /// The Cygwin download against the stub web: every file asked for, the simulator saved as
    /// `.exe`, the old simulators killed first; "Skip Download" asks for nothing.
    #[test]
    fn the_cygwin_launcher_downloads_the_simulator_and_its_dlls() {
        let dir = scratch("cygwin");
        let mut web = StubWeb::default();
        for (url, _) in cygwin_downloads("ArduRover.elf", ReleaseType::Beta) {
            web = web.serve(&url, url.as_bytes());
        }
        let said = Mutex::new(Vec::new());
        let cygwin = Cygwin::default();
        let image = cygwin.image(
            "ArduRover.elf",
            Some(ReleaseType::Beta),
            &dir,
            &web,
            &|text| said.os_lock().expect("lock").push(text.to_owned()),
        );
        assert_eq!(image, Image::Found(dir.join("ArduRover.exe")));
        assert_eq!(web.asked().len(), 11);
        assert_eq!(
            mp_os::fs::read(dir.join("ArduRover.exe")).ok(),
            Some(
                b"https://firmware.ardupilot.org/Tools/MissionPlanner/sitl/Beta/ArduRover.elf"
                    .to_vec()
            )
        );
        assert!(dir.join("cygstdc++-6.dll").os_is_file());
        assert_eq!(*said.os_lock().expect("lock"), [model::DOWNLOADING]);

        let web = StubWeb::default();
        let image = cygwin.image("ArduRover.elf", None, &dir, &web, &|_| {});
        assert_eq!(image, Image::Found(dir.join("ArduRover.exe")));
        assert!(web.asked().is_empty());
        let _ = mp_os::fs::remove_dir_all(dir);
    }

    /// A click through to the connection: the spawn's program, line, directory and variables, and
    /// the link it connects to.
    #[test]
    fn a_start_spawns_the_line_and_connects_to_5760() {
        let dir = scratch("start");
        let exe = dir.join("ArduCopter.exe");
        mp_os::fs::write(&exe, b"MZ").expect("write");
        let launcher = StubLauncher::new(Image::Found(exe.clone()));
        let web = autotest_web();
        let waited = Mutex::new(Vec::new());
        let outcome = start(
            &launcher,
            &web,
            &request(&dir, Vehicle::Multirotor),
            &|_| {},
            &|d| waited.os_lock().expect("lock").push(d),
        );
        let defaults = dir.join("default_params").join("copter.parm");
        let line = format!(
            "-M+ -O-35.3633515,149.1652412,584,0 -s1 --serial0 tcp:0  --defaults \"{}\"  ",
            defaults.display()
        );
        assert_eq!(
            outcome,
            Outcome::Connect {
                link: "tcp:127.0.0.1:5760".to_owned(),
                arguments: line.clone()
            }
        );
        let spawns = launcher.spawns();
        assert_eq!(spawns.len(), 1);
        let spawn = spawns.first().expect("one");
        assert_eq!(spawn.program, exe);
        assert_eq!(spawn.arguments, line);
        assert_eq!(spawn.working_directory, dir.join("+"));
        assert_eq!(spawn.home, dir.join("+"));
        assert_eq!(
            spawn.path,
            format!(
                "{};{};/usr/bin",
                model::with_separator(&dir),
                model::with_separator(&dir.join("+"))
            )
        );
        assert!(dir.join("+").os_is_dir());
        assert_eq!(
            mp_os::fs::read(&defaults).ok(),
            Some(b"FRAME_CLASS 1\n".to_vec())
        );
        assert_eq!(launcher.kills.load(Ordering::Relaxed), 1);
        assert_eq!(*waited.os_lock().expect("lock"), [Duration::from_secs(2)]);
        // sim_vehicle.py first, then vehicleinfo.py, then the file it names.
        assert_eq!(
            web.asked(),
            [
                model::SIM_VEHICLE_URL.to_owned(),
                model::VEHICLEINFO_URL.to_owned(),
                format!("{}default_params/copter.parm", model::AUTOTEST_URL),
            ]
        );
        let _ = mp_os::fs::remove_dir_all(dir);
    }

    /// The model box, the extra line and Wipe; a list of defaults joined into one file.
    #[test]
    fn the_model_box_overrides_the_picture_and_a_list_of_defaults_is_joined() {
        let dir = scratch("hexa");
        let exe = dir.join("ArduCopter.exe");
        mp_os::fs::write(&exe, b"MZ").expect("write");
        let launcher = StubLauncher::new(Image::Found(exe));
        let mut asked = request(&dir, Vehicle::Multirotor);
        asked.model_text = "hexa".to_owned();
        asked.cmdline = "--speedup-extra".to_owned();
        asked.wipe = true;
        asked.speedup = 10;
        let outcome = start(&launcher, &autotest_web(), &asked, &|_| {}, &|_| {});
        let joined = dir.join("hexa-defaults.parm");
        let Outcome::Connect { arguments, .. } = outcome else {
            panic!("{outcome:?}");
        };
        assert_eq!(
            arguments,
            format!(
                "-Mhexa -O-35.3633515,149.1652412,584,0 -s10 --serial0 tcp:0  --defaults \"{}\" --speedup-extra  --wipe ",
                joined.display()
            )
        );
        assert_eq!(
            mp_os::fs::read_to_string(&joined).ok().as_deref(),
            Some("\r\nFRAME_CLASS 1\n\r\nFRAME_CLASS 2\n")
        );
        assert_eq!(
            launcher
                .spawns()
                .first()
                .map(|s| s.working_directory.clone()),
            Some(dir.join("hexa"))
        );
        let _ = mp_os::fs::remove_dir_all(dir);
    }

    /// No defaults to be had anywhere: no `--defaults`.
    #[test]
    fn with_no_defaults_the_line_has_none() {
        let dir = scratch("nodefaults");
        let exe = dir.join("ArduPlane.exe");
        mp_os::fs::write(&exe, b"MZ").expect("write");
        let launcher = StubLauncher::new(Image::Found(exe));
        let outcome = start(
            &launcher,
            &StubWeb::default(),
            &request(&dir, Vehicle::Plane),
            &|_| {},
            &|_| {},
        );
        assert_eq!(
            outcome,
            Outcome::Connect {
                link: model::SITL_LINK.to_owned(),
                arguments: "-Mplane -O-35.3633515,149.1652412,584,0 -s1 --serial0 tcp:0   "
                    .to_owned()
            }
        );
        let _ = mp_os::fs::remove_dir_all(dir);
    }

    /// The C#'s failures, as the status line's words.
    #[test]
    fn a_missing_image_a_refused_start_and_no_sitl_are_outcomes() {
        let dir = scratch("failures");
        let missing = StubLauncher::new(Image::Found(dir.join("ArduCopter.exe")));
        assert_eq!(
            start(
                &missing,
                &StubWeb::default(),
                &request(&dir, Vehicle::Multirotor),
                &|_| {},
                &|_| {}
            ),
            Outcome::Failed("Failed to download the SITL image.".to_owned())
        );
        assert!(missing.spawns().is_empty());

        let exe = dir.join("ArduCopter.exe");
        mp_os::fs::write(&exe, b"MZ").expect("write");
        let mut refusing = StubLauncher::new(Image::Found(exe));
        refusing.refuse = true;
        assert_eq!(
            start(
                &refusing,
                &StubWeb::default(),
                &request(&dir, Vehicle::Multirotor),
                &|_| {},
                &|_| {}
            ),
            Outcome::Failed(
                "Failed to start the simulator\nThe system cannot find the file specified"
                    .to_owned()
            )
        );

        let outcome = start(
            &NotAvailable,
            &StubWeb::default(),
            &request(&dir, Vehicle::Multirotor),
            &|_| {},
            &|_| {},
        );
        let Outcome::NotAvailable(note) = outcome else {
            panic!("{outcome:?}");
        };
        assert!(note.contains("WebAssembly"), "{note}");
        assert!(note.contains("127.0.0.1"), "{note}");
        assert!(
            NotAvailable
                .spawn(&Spawn {
                    program: PathBuf::new(),
                    arguments: String::new(),
                    working_directory: PathBuf::new(),
                    path: String::new(),
                    home: PathBuf::new(),
                })
                .is_err()
        );
        assert_eq!(NotAvailable.note().as_deref(), Some(note.as_str()));
        let _ = mp_os::fs::remove_dir_all(dir);
    }

    /// The chain swarm: one spawn per instance in its own directory with its identity, the
    /// scripts grown, one link.
    #[test]
    fn the_chain_swarm_spawns_each_instance_and_connects_once() {
        let dir = scratch("chain");
        let data = scratch("chain-data");
        let exe = dir.join("ArduCopter.exe");
        let launcher = StubLauncher::new(Image::Found(exe.clone()));
        let outcome = start_chain(
            &launcher,
            &autotest_web(),
            &ChainRequest {
                how_many: 2,
                release: None,
                homes: vec!["A".to_owned(), "B".to_owned()],
                dir: dir.clone(),
                data_dir: data.clone(),
                path: "/bin".to_owned(),
            },
            &|_| {},
            &|_| {},
        );
        let Outcome::Connect { link, .. } = &outcome else {
            panic!("{outcome:?}");
        };
        assert_eq!(link, "tcp:127.0.0.1:5760");
        let spawns = launcher.spawns();
        assert_eq!(spawns.len(), 2);
        let defaults = dir.join("default_params").join("copter.parm");
        assert_eq!(
            spawns.first().map(|s| s.arguments.clone()),
            Some(format!(
                "  --defaults \"{},identity.parm\"  -M+ -s1 --home B --instance 1 --serial0 tcp:0  ",
                defaults.display()
            ))
        );
        assert_eq!(
            spawns.first().map(|s| s.working_directory.clone()),
            Some(dir.join("+2"))
        );
        assert!(spawns.last().is_some_and(|s| s.arguments.contains(
            "--home A --instance 0 --serial0 tcp:0 --serial2 tcpclient:127.0.0.1:5772 "
        )));
        assert!(
            mp_os::fs::read_to_string(dir.join("+1").join("identity.parm"))
                .is_ok_and(|text| text.contains("SYSID_THISMAV=1\r\n"))
        );
        let bat = mp_os::fs::read_to_string(data.join("sitl.bat")).unwrap_or_default();
        assert!(bat.starts_with("mkdir 2\ncd 2\n"), "{bat}");
        assert!(bat.contains("mkdir 1\ncd 1\n"), "{bat}");
        assert!(data.join("sitl1.sh").os_is_file());
        assert_eq!(launcher.kills.load(Ordering::Relaxed), 1);
        let _ = mp_os::fs::remove_dir_all(dir);
        let _ = mp_os::fs::remove_dir_all(data);
    }

    /// The desktop's own choice.
    #[test]
    fn this_desktop_gets_the_csharps_source() {
        let name = for_this_desktop().name();
        if cfg!(windows) {
            assert_eq!(name, "cygwin");
        } else if cfg!(all(target_os = "linux", target_arch = "x86_64")) {
            assert_eq!(name, "manifest SITL_x86_64_linux_gnu");
        } else if cfg!(target_os = "macos") {
            assert_eq!(name, "not available");
        }
    }
}
