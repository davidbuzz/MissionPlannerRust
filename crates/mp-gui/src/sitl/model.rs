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

//! The SITL page's model: its texts and lists from `SITL.resx`, the vehicles behind its four
//! pictures, and the command lines `StartSITL` and `StartSwarmChain` build from the page's
//! choices - and `GetDefaultConfig`, which finds the `--defaults` file a model runs with.
//!
//! Nothing here starts a process or touches the page: `launcher.rs` does the one, `view.rs` the
//! other.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use mp_os::fs::FsExt as _;
use std::path::{Path, PathBuf};

use mp_firmware::manifest::{Fetch, ReleaseType};

// ---- The texts, from SITL.resx ----

/// `groupBox1.Text`, round the map. `// C#: GCSViews/SITL.resx (groupBox1.Text)`
pub const GROUP_HOME: &str = "Home Location - Drag Me";
/// `groupBox2.Text`, round the pictures. `// C#: GCSViews/SITL.resx (groupBox2.Text)`
pub const GROUP_FIRMWARE: &str = "Please select a firmware to run";
/// `groupBox3.Text`. `// C#: GCSViews/SITL.resx (groupBox3.Text)`
pub const GROUP_OPTIONS: &str = "Options";
/// `groupBox4.Text`. `// C#: GCSViews/SITL.resx (groupBox4.Text)`
pub const GROUP_ADVANCED: &str = "Advanced users only";
/// `label1.Text`, beside `NUM_heading`. `// C#: GCSViews/SITL.resx (label1.Text)`
pub const HEADING: &str = "Heading";
/// `label2.Text`, beside `num_simspeed`. `// C#: GCSViews/SITL.resx (label2.Text)`
pub const SIM_SPEED: &str = "Sim Speed";
/// `label7.Text`, beside `cmb_model`. `// C#: GCSViews/SITL.resx (label7.Text)`
pub const MODEL: &str = "Model";
/// `label8.Text`, beside `txt_cmdline`. `// C#: GCSViews/SITL.resx (label8.Text)`
pub const EXTRA_COMMAND_LINE: &str = "Extra command line";
/// `chk_wipe.Text`. `// C#: GCSViews/SITL.resx (chk_wipe.Text)`
pub const WIPE: &str = "Wipe";
/// `but_swarmseq.Text`. `// C#: GCSViews/SITL.resx (but_swarmseq.Text)`
pub const SWARM_SEQ: &str = "Copter Swarm - Single link";
/// `but_swarmlink.Text`. `// C#: GCSViews/SITL.resx (but_swarmlink.Text)`
pub const SWARM_LINK: &str = "Copter Swarm - Multilink";
/// `but_swarmplane.Text`. `// C#: GCSViews/SITL.resx (but_swarmplane.Text)`
pub const SWARM_PLANE: &str = "Plane Swarm - Multilink";
/// `but_swarmrover.Text`. `// C#: GCSViews/SITL.resx (but_swarmrover.Text)`
pub const SWARM_ROVER: &str = "Rover Swarm - Multilink";

/// `Strings.Failed_to_download_the_SITL_image`. `// C#: ExtLibs/Strings/Strings.resx:640-641`
pub const FAILED_TO_DOWNLOAD: &str = "Failed to download the SITL image.";
/// `Strings.Failed_to_connect_to_SITL_instance`. `// C#: ExtLibs/Strings/Strings.resx:606-607`
pub const FAILED_TO_CONNECT: &str = "Failed to connect to SITL instance";
/// `"Failed to start the simulator\n" + ex`'s first line. `// C#: GCSViews/SITL.cs:680, 708`
pub const FAILED_TO_START: &str = "Failed to start the simulator";
/// `Common.LoadingBox("Downloading", "Downloading sitl software")`'s text.
/// `// C#: GCSViews/SITL.cs:422`
pub const DOWNLOADING: &str = "Downloading sitl software";
/// `InputBox.Show("how many?", "how many?", ref max)`: the swarms' question, caption and text.
/// `// C#: GCSViews/SITL.cs:833, 997`
pub const HOW_MANY: &str = "how many?";
/// The swarms' question's answer before it is changed. `// C#: GCSViews/SITL.cs:831, 995`
pub const HOW_MANY_DEFAULT: i32 = 10;

/// `cmb_model.Items`, in order: the models SITL's `-M` takes, the first blank - which leaves the
/// picture's own model in place (`if (cmb_model.Text != "")`).
/// `// C#: GCSViews/SITL.Designer.cs:291-326; GCSViews/SITL.resx (cmb_model.Items..Items33)`
pub const MODELS: [&str; 34] = [
    "",
    "quadplane",
    "xplane",
    "xplane-heli",
    "firefly",
    "+",
    "quad",
    "copter",
    "x",
    "hexa",
    "octa",
    "tri",
    "y6",
    "heli",
    "heli-dual",
    "heli-compound",
    "singlecopter",
    "coaxcopter",
    "rover",
    "crrcsim",
    "jsbsim",
    "flightaxis",
    "gazebo",
    "last_letter",
    "tracker",
    "balloon",
    "plane",
    "calibration",
    "plane-jet",
    "sailboat",
    "motorboat",
    "morse-rover",
    "rover-skid",
    "plane-3d",
];

/// `cmb_version`'s items and the release each downloads; "Skip Download" downloads nothing and
/// runs what the directory already holds.
/// `// C#: GCSViews/SITL.cs:114-125`
pub const VERSIONS: [(&str, Option<ReleaseType>); 4] = [
    ("Latest (Dev)", Some(ReleaseType::Dev)),
    ("Beta", Some(ReleaseType::Beta)),
    ("Stable", Some(ReleaseType::Official)),
    ("Skip Download", None),
];

/// The setting `cmb_version`'s index is kept under. `// C#: GCSViews/SITL.cs:125, 272`
pub const VERSION_SETTING: &str = "sitl_download_version";

/// `cmb_version.SelectedIndex = Settings.Instance.GetInt32("sitl_download_version")`: the index
/// the setting holds - `GetInt32`'s 0 for nothing or a non-number. An index the list has not got
/// throws `ArgumentOutOfRangeException` in the C#'s constructor, which stops the application
/// starting; here it is the first item instead (divergence).
/// `// C#: GCSViews/SITL.cs:125; ExtLibs/Utilities/Settings.cs:201-210`
#[must_use]
pub fn version_index(setting: Option<&str>) -> usize {
    setting
        .and_then(|text| text.trim().parse::<usize>().ok())
        .filter(|index| *index < VERSIONS.len())
        .unwrap_or(0)
}

/// Where the home marker goes when the planner has no home: `Activate`'s point near Canberra.
/// (The field's own initial value, Western Australia, is always replaced by `Activate`.)
/// `// C#: GCSViews/SITL.cs:48, 130-133`
pub const DEFAULT_HOME: (f64, f64) = (-35.363_351_5, 149.165_241_2);
/// `myGMAP1.Zoom` once `Activate` has run. `// C#: GCSViews/SITL.cs:139`
pub const MAP_ZOOM: f64 = 16.0;

/// `NUM_heading.Maximum`; its minimum and value are `NumericUpDown`'s 0.
/// `// C#: GCSViews/SITL.Designer.cs:212-216`
pub const HEADING_MAX: i64 = 360;
/// `num_simspeed.Minimum` and `Value`. `// C#: GCSViews/SITL.Designer.cs:338-348`
pub const SPEED_MIN: i64 = 1;
/// `num_simspeed.Maximum`: `NumericUpDown`'s default, as the Designer sets none.
pub const SPEED_MAX: i64 = 100;

/// Where `StartSITL` connects once the simulator has had two seconds:
/// `new TcpClient("127.0.0.1", 5760)`. `// C#: GCSViews/SITL.cs:713, 724`
pub const SITL_LINK: &str = "tcp:127.0.0.1:5760";
/// `await Task.Delay(2000)` between starting the simulator and connecting to it.
/// `// C#: GCSViews/SITL.cs:713, 1090`
pub const START_WAIT: std::time::Duration = std::time::Duration::from_secs(2);

/// The vehicles of `panel1`'s four pictures, left to right as the `.resx` places them.
/// `// C#: GCSViews/SITL.cs:154-238; GCSViews/SITL.resx (pictureBox*.Location)`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Vehicle {
    /// `pictureBoxplane`.
    Plane,
    /// `pictureBoxrover`.
    Rover,
    /// `pictureBoxquad`.
    Multirotor,
    /// `pictureBoxheli`.
    Helicopter,
}

impl Vehicle {
    /// The four, left to right.
    pub const ALL: [Self; 4] = [Self::Plane, Self::Rover, Self::Multirotor, Self::Helicopter];

    /// The file each click asks `CheckandGetSITLImage` for.
    /// `// C#: GCSViews/SITL.cs:157, 185, 206, 227`
    #[must_use]
    pub const fn file(self) -> &'static str {
        match self {
            Self::Plane => "ArduPlane.elf",
            Self::Rover => "ArduRover.elf",
            Self::Multirotor => "ArduCopter.elf",
            Self::Helicopter => "ArduHeli.elf",
        }
    }

    /// The model each click passes `StartSITL`, which `cmb_model` overrides when not blank.
    /// `// C#: GCSViews/SITL.cs:167, 188, 209, 230`
    #[must_use]
    pub const fn model(self) -> &'static str {
        match self {
            Self::Plane => "plane",
            Self::Rover => "rover",
            Self::Multirotor => "+",
            Self::Helicopter => "heli",
        }
    }

    /// The label under the picture: `label6`, `label5`, `label4`, `label3`.
    /// `// C#: GCSViews/SITL.resx (label3..label6.Text)`
    #[must_use]
    pub const fn label(self) -> &'static str {
        match self {
            Self::Plane => "Plane",
            Self::Rover => "Rover",
            Self::Multirotor => "Multirotor",
            Self::Helicopter => "Helicopter",
        }
    }

    /// The picture's control id, for scripts.
    #[must_use]
    pub const fn id(self) -> &'static str {
        match self {
            Self::Plane => "sitl-picture-plane",
            Self::Rover => "sitl-picture-rover",
            Self::Multirotor => "sitl-picture-quad",
            Self::Helicopter => "sitl-picture-heli",
        }
    }

    /// The `PictureBoxMouseOver`'s name in the Designer.
    /// `// C#: GCSViews/SITL.Designer.cs:40-43`
    #[must_use]
    pub const fn control(self) -> &'static str {
        match self {
            Self::Plane => "pictureBoxplane",
            Self::Rover => "pictureBoxrover",
            Self::Multirotor => "pictureBoxquad",
            Self::Helicopter => "pictureBoxheli",
        }
    }

    /// The picture shown: `ImageOver` under the pointer, `ImageNormal` otherwise - the
    /// `.resx`'s bitmaps, as [`crate::pictures`] carries them. `selected`, the other way to
    /// `ImageOver`, stays false: the Designer's `selected = false` is its only assignment.
    /// `// C#: GCSViews/SITL.Designer.cs:140-143, 151-154, 162-165, 173-176;
    /// ExtLibs/Controls/PictureBoxMouseOver.cs:37-50`
    #[must_use]
    pub const fn image(self, over: bool) -> &'static str {
        match (self, over) {
            (Self::Plane, false) => "SITL.pictureBoxplane.ImageNormal",
            (Self::Plane, true) => "SITL.pictureBoxplane.ImageOver",
            (Self::Rover, false) => "SITL.pictureBoxrover.ImageNormal",
            (Self::Rover, true) => "SITL.pictureBoxrover.ImageOver",
            (Self::Multirotor, false) => "SITL.pictureBoxquad.ImageNormal",
            (Self::Multirotor, true) => "SITL.pictureBoxquad.ImageOver",
            (Self::Helicopter, false) => "SITL.pictureBoxheli.ImageNormal",
            (Self::Helicopter, true) => "SITL.pictureBoxheli.ImageOver",
        }
    }

    /// The picture's `Location` and `Size` in `panel1`.
    /// `// C#: GCSViews/SITL.resx (pictureBox*.Location, pictureBox*.Size)`
    #[must_use]
    pub const fn picture(self) -> (f32, f32, f32, f32) {
        match self {
            Self::Plane => (3.0, 3.0, 131.0, 112.0),
            Self::Rover => (159.0, 3.0, 150.0, 112.0),
            Self::Multirotor => (331.0, 3.0, 125.0, 112.0),
            Self::Helicopter => (477.0, 3.0, 235.0, 112.0),
        }
    }

    /// The label's `Location` in `panel1`. `// C#: GCSViews/SITL.resx (label3..label6.Location)`
    #[must_use]
    pub const fn label_at(self) -> (f32, f32) {
        match self {
            Self::Plane => (48.0, 120.0),
            Self::Rover => (217.0, 120.0),
            Self::Multirotor => (370.0, 120.0),
            Self::Helicopter => (569.0, 120.0),
        }
    }
}

/// `if (cmb_model.Text != "") model = cmb_model.Text;`: the box's text when there is any, else
/// the picture's model. `// C#: GCSViews/SITL.cs:638-640`
#[must_use]
pub fn chosen_model(vehicle: Vehicle, model_text: &str) -> String {
    if model_text.is_empty() {
        vehicle.model().to_owned()
    } else {
        model_text.to_owned()
    }
}

/// `BuildHomeLocation`: `lat,lng,alt,heading`, each as .NET's invariant `ToString` writes it -
/// fifteen significant digits for the doubles - the altitude being SRTM's at the point.
/// `// C#: GCSViews/SITL.cs:240-244`
#[must_use]
pub fn home_location(lat: f64, lng: f64, alt: f64, heading: i64) -> String {
    format!(
        "{},{},{},{heading}",
        mp_log::netfmt::double(lat),
        mp_log::netfmt::double(lng),
        mp_log::netfmt::double(alt),
    )
}

/// What `StartSITL` appends to `extraargs` (which every caller passes as ""): the defaults file
/// when there is one, the extra command line between spaces, and `--wipe`.
/// `// C#: GCSViews/SITL.cs:642-650`
#[must_use]
pub fn extra_arguments(config: &str, cmdline: &str, wipe: bool) -> String {
    let mut extra = String::new();
    if !config.is_empty() {
        extra.push_str(&format!(" --defaults \"{config}\""));
    }
    extra.push(' ');
    extra.push_str(cmdline);
    extra.push(' ');
    if wipe {
        extra.push_str(" --wipe ");
    }
    extra
}

/// `exestart.Arguments`: the simulator's command line.
/// `// C#: GCSViews/SITL.cs:664`
#[must_use]
pub fn arguments(model: &str, home: &str, speedup: i64, extra: &str) -> String {
    format!("-M{model} -O{home} -s{speedup} --serial0 tcp:0 {extra}")
}

/// A path as the C# builds one by concatenation: a directory ends in its separator.
#[must_use]
pub fn with_separator(dir: &Path) -> String {
    let text = dir.display().to_string();
    if text.ends_with(std::path::MAIN_SEPARATOR) {
        text
    } else {
        format!("{text}{}", std::path::MAIN_SEPARATOR)
    }
}

/// The `PATH` the simulator starts with: the SITL directory and the model's, then the process's
/// own, joined by `;` - Windows' separator, which the C# uses on every system (on Linux the first
/// entry of the old `PATH` is lost to it, as in the C#).
/// `// C#: GCSViews/SITL.cs:656-658, 914-917, 1063-1066`
#[must_use]
pub fn simulator_path(sitl_dir: &Path, sim_dir: &Path, path: &str) -> String {
    format!(
        "{};{};{path}",
        with_separator(sitl_dir),
        with_separator(sim_dir)
    )
}

/// Splits a command line into arguments, for the systems where a process is given a list rather
/// than a line: spaces separate, double quotes group and are dropped, `\"` is a quote. What
/// `Process.Start` does with `Arguments` under mono on Linux and macOS, for every line these
/// pages build.
#[must_use]
#[cfg_attr(windows, allow(dead_code))] // Windows is given the line whole
pub fn split_arguments(line: &str) -> Vec<String> {
    let mut arguments = Vec::new();
    let mut current = String::new();
    let mut quoted = false;
    let mut started = false;
    let mut chars = line.chars().peekable();
    while let Some(c) = chars.next() {
        match c {
            '\\' if chars.peek() == Some(&'"') => {
                current.push('"');
                started = true;
                chars.next();
            }
            '"' => {
                quoted = !quoted;
                started = true;
            }
            c if c.is_whitespace() && !quoted => {
                if started {
                    arguments.push(std::mem::take(&mut current));
                    started = false;
                }
            }
            c => {
                current.push(c);
                started = true;
            }
        }
    }
    if started {
        arguments.push(current);
    }
    arguments
}

// ---- The swarm ----

/// `identity.parm`, which gives each swarm instance its own system id. The verbatim string's
/// line ends are the source file's, CRLF.
/// `// C#: GCSViews/SITL.cs:902-912, 1051-1061`
#[must_use]
pub fn identity(sysid: i32) -> String {
    [
        "SERIAL0_PROTOCOL=2".to_owned(),
        "SERIAL1_PROTOCOL=2".to_owned(),
        format!("SYSID_THISMAV={sysid}"),
        format!("MAV_SYSID={sysid}"),
        "SIM_TERRAIN=0".to_owned(),
        "TERRAIN_ENABLE=0".to_owned(),
        "SCHED_LOOP_RATE=50".to_owned(),
        "SIM_RATE_HZ=400".to_owned(),
        "SIM_DRIFT_SPEED=0".to_owned(),
        "SIM_DRIFT_TIME=0".to_owned(),
        String::new(),
    ]
    .join("\r\n")
}

/// One instance of `StartSwarmChain`'s loop.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ChainInstance {
    /// `a`, the instance number SITL is given.
    pub instance: i32,
    /// `extra`: the whole command line.
    pub arguments: String,
    /// `model + (a + 1)`: its directory under the SITL directory.
    pub dir: String,
    /// `identity.parm`'s text.
    pub identity: String,
    /// What is appended to `sitl.bat`.
    pub bat: String,
    /// What is appended to `sitl1.sh`.
    pub sh: String,
}

/// `StartSwarmChain`'s instances for `how_many` copters, last first as the loop runs them: each
/// with its identity, 4 m further along the heading than the one before (`home(a)` gives the
/// `BuildHomeLocation` of `newpos(heading, a * 4)`), and all but the first chained to the next by
/// `--serial2 tcpclient`; and the lines `sitl.bat` and `sitl1.sh` gain for it.
/// `// C#: GCSViews/SITL.cs:1016-1083`
#[must_use]
pub fn chain(
    how_many: i32,
    model: &str,
    config: &str,
    exe: &Path,
    home: &dyn Fn(i32) -> String,
) -> Vec<ChainInstance> {
    let max = how_many.saturating_sub(1);
    let exe_text = exe.display().to_string();
    let exe_name = exe
        .file_name()
        .map(|name| name.to_string_lossy().into_owned())
        .unwrap_or_default();
    let mut instances = Vec::new();
    let mut a = max;
    while a >= 0 {
        let mut extra = " ".to_owned();
        if config.is_empty() {
            extra.push_str(" --defaults \"identity.parm\" ");
        } else {
            extra.push_str(&format!(" --defaults \"{config},identity.parm\" "));
        }
        let serial2 = if a == max {
            String::new()
        } else {
            format!("--serial2 tcpclient:127.0.0.1:{}", 5772 + 10 * a)
        };
        extra.push_str(&format!(
            " -M{model} -s1 --home {} --instance {a} --serial0 tcp:0 {serial2} ",
            home(a)
        ));
        let number = a + 1;
        let bat = format!("mkdir {number}\ncd {number}\n\"{exe_text}\" {extra} &\n");
        let sh = format!(
            "mkdir {number}\ncd {number}\n\"../{}\" {} &\nsleep .3\ncd ..\n",
            exe_name
                .replace("C:", "/mnt/c")
                .replace('\\', "/")
                .replace(".exe", ".elf"),
            extra.replace("C:", "/mnt/c").replace('\\', "/")
        );
        instances.push(ChainInstance {
            instance: a,
            arguments: extra,
            dir: format!("{model}{number}"),
            identity: identity(number),
            bat,
            sh,
        });
        a -= 1;
    }
    instances
}

// ---- GetDefaultConfig ----

/// Where `GetDefaultConfig` looks first. `// C#: GCSViews/SITL.cs:461`
pub const SIM_VEHICLE_URL: &str =
    "https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/autotest/sim_vehicle.py";
/// Where the defaults files are, by the path the lists give. `// C#: GCSViews/SITL.cs:471, 515, 531`
pub const AUTOTEST_URL: &str =
    "https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/autotest/";
/// Where it looks second. `// C#: GCSViews/SITL.cs:480`
pub const VEHICLEINFO_URL: &str =
    "https://firmware.ardupilot.org/Tools/MissionPlanner/vehicleinfo.py";

/// `default_params_regex`, over `sim_vehicle.py`: a frame's name and its first defaults file.
/// `// C#: GCSViews/SITL.cs:30-32`
const DEFAULT_PARAMS_PATTERN: &str =
    r#""([^"]+)"\s*:\s*\{\s*[^\{}]+"default_params_filename"\s*:\s*\[*"([^"]+)"\s*[^\}]*\}"#;

/// `Download.getFilefromNet(url, saveto)`, as the firmware pages have it.
fn download(fetch: &dyn Fetch, url: &str, to: &Path) -> bool {
    mp_firmware::flow::get_file_from_net(fetch, url, to, &mut |_, _| {})
}

/// `sitldirectory + name`: a name the lists give, with its `/`s, under the directory.
fn under(dir: &Path, name: &str) -> PathBuf {
    let mut path = dir.to_path_buf();
    for part in name.split(['/', '\\']).filter(|part| !part.is_empty()) {
        path.push(part);
    }
    path
}

/// `GetDefaultConfig`: the `--defaults` file for a model, "" for none.
///
/// First `sim_vehicle.py`, fetched (or as the directory already holds it), searched with
/// `default_params_regex` for the model's frame; then `vehicleinfo.py`, cut down to JSON by
/// `cleanupJson` and searched vehicle by vehicle for `frames[model].default_params_filename` - one
/// file, fetched, or a list, fetched and joined into one.
///
/// The joined list goes where the C# puts it, `Path.GetTempFileName()`, except that the
/// application writes nowhere but its data directory: here it is `<model>-defaults.parm` in the
/// SITL directory (divergence). Every error the C# catches is "".
/// `// C#: GCSViews/SITL.cs:458-553`
pub fn default_config(model: &str, dir: &Path, fetch: &dyn Fetch) -> String {
    let sim_vehicle = dir.join("sim_vehicle.py");
    if download(fetch, SIM_VEHICLE_URL, &sim_vehicle) || sim_vehicle.os_is_file() {
        let text = mp_os::fs::read_to_string(&sim_vehicle).unwrap_or_default();
        for (frame, file) in sim_vehicle_defaults(&text) {
            // `match.Groups[1].Value.ToLower().Equals(model)`: the frame lowered, the model not.
            if frame.to_lowercase() == model {
                let to = under(dir, &file);
                if download(fetch, &format!("{AUTOTEST_URL}{file}"), &to) || to.os_is_file() {
                    return to.display().to_string();
                }
            }
        }
    }
    let vehicleinfo = dir.join("vehicleinfo.py");
    if download(fetch, VEHICLEINFO_URL, &vehicleinfo) || vehicleinfo.os_is_file() {
        return vehicleinfo_config(model, dir, &vehicleinfo, fetch).unwrap_or_default();
    }
    String::new()
}

/// `default_params_regex.Matches(sim_vehicle.py)`: each frame's name and defaults file.
#[must_use]
pub fn sim_vehicle_defaults(text: &str) -> Vec<(String, String)> {
    let Ok(pattern) = regex::Regex::new(DEFAULT_PARAMS_PATTERN) else {
        return Vec::new();
    };
    pattern
        .captures_iter(text)
        .filter_map(|found| {
            Some((
                found.get(1)?.as_str().to_owned(),
                found.get(2)?.as_str().to_owned(),
            ))
        })
        .collect()
}

/// The `try` of `GetDefaultConfig`'s second half: `None` where the C# throws and catches.
/// `// C#: GCSViews/SITL.cs:483-550`
fn vehicleinfo_config(model: &str, dir: &Path, file: &Path, fetch: &dyn Fetch) -> Option<String> {
    let content = mp_os::fs::read_to_string(file).ok()?;
    let cleaned = cleanup_json(&content);
    mp_os::fs::write(file, &cleaned).ok()?;
    let parsed: serde_json::Value = serde_json::from_str(&lenient_json(&cleaned)).ok()?;
    // `if (obj == null) return "";`
    let serde_json::Value::Object(obj) = parsed else {
        return Some(String::new());
    };
    for (_, fwtype) in obj {
        // `fwtype.Value["frames"]`: indexing a value that is not an object throws.
        let serde_json::Value::Object(vehicle) = fwtype else {
            return None;
        };
        let Some(frames) = vehicle.get("frames").filter(|v| !v.is_null()) else {
            continue;
        };
        let serde_json::Value::Object(frames) = frames else {
            return None;
        };
        let Some(config) = frames.get(model).filter(|v| !v.is_null()) else {
            continue;
        };
        let serde_json::Value::Object(config) = config else {
            return None;
        };
        let configs = config.get("default_params_filename")?;
        let mut data = String::new();
        match configs {
            serde_json::Value::Array(items) => {
                for item in items {
                    let name = token_text(item);
                    let to = under(dir, &name);
                    if let Some(parent) = to.parent() {
                        let _ = mp_os::fs::create_dir_all(parent);
                    }
                    if download(fetch, &format!("{AUTOTEST_URL}{name}"), &to) || to.os_is_file() {
                        data.push_str("\r\n");
                        data.push_str(&mp_os::fs::read_to_string(&to).unwrap_or_default());
                    }
                }
            }
            serde_json::Value::Object(_) => {
                // A `JObject` enumerates `JProperty`s, whose text names no file: nothing is
                // fetched or joined.
            }
            serde_json::Value::Null => return None,
            value => {
                // `configs is JValue`: one file.
                let name = token_text(value);
                let to = under(dir, &name);
                if let Some(parent) = to.parent() {
                    let _ = mp_os::fs::create_dir_all(parent);
                }
                if download(fetch, &format!("{AUTOTEST_URL}{name}"), &to) || to.os_is_file() {
                    return Some(to.display().to_string());
                }
                // Neither fetched nor there: the C# goes on to enumerate the `JValue`, which has
                // no children, and writes an empty file.
            }
        }
        let temp = dir.join(format!("{model}-defaults.parm"));
        mp_os::fs::write(&temp, data).ok()?;
        return Some(temp.display().to_string());
    }
    Some(String::new())
}

/// `JToken.ToString()` of a list item: a string's own text, anything else as JSON writes it.
fn token_text(value: &serde_json::Value) -> String {
    match value {
        serde_json::Value::String(text) => text.clone(),
        serde_json::Value::Bool(true) => "True".to_owned(),
        serde_json::Value::Bool(false) => "False".to_owned(),
        other => other.to_string(),
    }
}

/// `cleanupJson`'s text: the first balanced `{...}`, each `#` to the end of its line removed,
/// and `True` and `False` quoted - Python's dictionary made JSON enough for Newtonsoft.
/// `// C#: GCSViews/SITL.cs:555-570`
#[must_use]
pub fn cleanup_json(content: &str) -> String {
    let matched = brace_match(content, '{', '}');
    // `Regex.Replace(match, @"#.*", "")`: `.` is anything but `\n`.
    let mut uncommented = String::with_capacity(matched.len());
    let mut in_comment = false;
    for c in matched.chars() {
        if c == '\n' {
            in_comment = false;
        } else if c == '#' {
            in_comment = true;
        }
        if !in_comment {
            uncommented.push(c);
        }
    }
    uncommented
        .replace("True", "\"True\"")
        .replace("False", "\"False\"")
}

/// `BraceMatch`: from the first opening brace to the one that closes it, "" when none does.
/// `// C#: GCSViews/SITL.cs:572-602`
#[must_use]
pub fn brace_match(text: &str, open: char, close: char) -> String {
    let mut level = 0i64;
    let mut start = 0usize;
    for (index, c) in text.char_indices() {
        if c == open {
            if level == 0 {
                start = index;
            }
            level += 1;
        }
        if c == close {
            level -= 1;
            if level == 0 {
                return text
                    .get(start..index + c.len_utf8())
                    .unwrap_or_default()
                    .to_owned();
            }
        }
    }
    String::new()
}

/// The leniency Newtonsoft's reader has and `serde_json` has not, for what `cleanupJson` leaves
/// of a Python dictionary: a comma before a closing bracket or brace is dropped.
#[must_use]
pub fn lenient_json(text: &str) -> String {
    let mut out = String::with_capacity(text.len());
    let mut in_string = false;
    let mut escaped = false;
    let mut pending_comma: Option<String> = None;
    for c in text.chars() {
        if in_string {
            out.push(c);
            if escaped {
                escaped = false;
            } else if c == '\\' {
                escaped = true;
            } else if c == '"' {
                in_string = false;
            }
            continue;
        }
        if let Some(held) = pending_comma.as_mut() {
            if c.is_whitespace() {
                held.push(c);
                continue;
            }
            let held = pending_comma.take().unwrap_or_default();
            if c == '}' || c == ']' {
                // The comma is dropped, the whitespace after it kept.
                out.push_str(held.get(1..).unwrap_or_default());
            } else {
                out.push_str(&held);
            }
        }
        match c {
            ',' => pending_comma = Some(",".to_owned()),
            '"' => {
                in_string = true;
                out.push(c);
            }
            _ => out.push(c),
        }
    }
    if let Some(held) = pending_comma {
        out.push_str(&held);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every vehicle with every model the list offers: the line `StartSITL` gives the simulator,
    /// the blank model leaving the picture's own.
    #[test]
    fn the_argument_line_for_every_vehicle_and_model() {
        let home = home_location(-35.363_351_5, 149.165_241_2, 584.0, 0);
        assert_eq!(home, "-35.3633515,149.1652412,584,0");
        for vehicle in Vehicle::ALL {
            for model in MODELS {
                let chosen = chosen_model(vehicle, model);
                let extra = extra_arguments("", "", false);
                let line = arguments(&chosen, &home, 1, &extra);
                let expected_model = if model.is_empty() {
                    vehicle.model()
                } else {
                    model
                };
                assert_eq!(
                    line,
                    format!("-M{expected_model} -O{home} -s1 --serial0 tcp:0   "),
                    "{vehicle:?} {model:?}"
                );
            }
        }
        // The four pictures, as the C#'s handlers name them.
        let models: Vec<&str> = Vehicle::ALL.iter().map(|v| v.model()).collect();
        assert_eq!(models, ["plane", "rover", "+", "heli"]);
        let files: Vec<&str> = Vehicle::ALL.iter().map(|v| v.file()).collect();
        assert_eq!(
            files,
            [
                "ArduPlane.elf",
                "ArduRover.elf",
                "ArduCopter.elf",
                "ArduHeli.elf"
            ]
        );
        assert_eq!(MODELS.len(), 34);
        assert_eq!(MODELS.first(), Some(&""));
        assert_eq!(MODELS.last(), Some(&"plane-3d"));
    }

    /// The defaults file quoted, the extra line between spaces, `--wipe` last, the speed-up.
    #[test]
    fn the_extra_arguments_are_appended_as_start_sitl_appends_them() {
        let extra = extra_arguments("/d/sitl/default_params/copter.parm", "--foo 1", true);
        assert_eq!(
            extra,
            " --defaults \"/d/sitl/default_params/copter.parm\" --foo 1  --wipe "
        );
        assert_eq!(
            arguments("hexa", "1,2,3,90", 5, &extra),
            "-Mhexa -O1,2,3,90 -s5 --serial0 tcp:0  --defaults \"/d/sitl/default_params/copter.parm\" --foo 1  --wipe "
        );
        assert_eq!(
            split_arguments(&arguments("hexa", "1,2,3,90", 5, &extra)),
            [
                "-Mhexa",
                "-O1,2,3,90",
                "-s5",
                "--serial0",
                "tcp:0",
                "--defaults",
                "/d/sitl/default_params/copter.parm",
                "--foo",
                "1",
                "--wipe"
            ]
        );
        assert_eq!(split_arguments("a \"b c\" \\\"d"), ["a", "b c", "\"d"]);
        assert_eq!(split_arguments("\"\""), [""]);
    }

    /// Fifteen significant digits, as .NET Framework writes a double, and the heading an integer.
    #[test]
    fn the_home_is_written_as_dotnet_writes_it() {
        assert_eq!(
            home_location(-35.363_351_512_345_67, 149.1, 0.0, 359),
            "-35.3633515123457,149.1,0,359"
        );
        assert_eq!(
            home_location(0.00001, 1e-5, -12.5, 0),
            "1E-05,1E-05,-12.5,0"
        );
    }

    /// The saved index, else the first; one past the list is the first too.
    #[test]
    fn the_version_comes_back_from_its_setting() {
        assert_eq!(version_index(None), 0);
        assert_eq!(version_index(Some("2")), 2);
        assert_eq!(version_index(Some("3")), 3);
        assert_eq!(version_index(Some("4")), 0);
        assert_eq!(version_index(Some("x")), 0);
        let names: Vec<&str> = VERSIONS.iter().map(|(name, _)| *name).collect();
        assert_eq!(names, ["Latest (Dev)", "Beta", "Stable", "Skip Download"]);
    }

    /// Three instances, the last first; the first-made chains to nothing, the others to the
    /// next port up; identity by instance; the scripts' lines.
    #[test]
    fn the_chain_swarm_builds_what_start_swarm_chain_builds() {
        let exe = Path::new("sitl").join("ArduCopter.exe");
        let homes = |a: i32| format!("H{a}");
        let instances = chain(3, "+", "C:\\sitl\\copter.parm", &exe, &homes);
        assert_eq!(instances.len(), 3);
        let first = instances.first().expect("three");
        assert_eq!(first.instance, 2);
        assert_eq!(
            first.arguments,
            "  --defaults \"C:\\sitl\\copter.parm,identity.parm\"  -M+ -s1 --home H2 --instance 2 --serial0 tcp:0  "
        );
        assert_eq!(first.dir, "+3");
        assert!(first.identity.starts_with(
            "SERIAL0_PROTOCOL=2\r\nSERIAL1_PROTOCOL=2\r\nSYSID_THISMAV=3\r\nMAV_SYSID=3\r\n"
        ));
        assert!(first.identity.ends_with("SIM_DRIFT_TIME=0\r\n"));
        let last = instances.last().expect("three");
        assert_eq!(last.instance, 0);
        assert!(
            last.arguments
                .contains("--instance 0 --serial0 tcp:0 --serial2 tcpclient:127.0.0.1:5772 "),
            "{}",
            last.arguments
        );
        assert_eq!(
            last.bat,
            format!(
                "mkdir 1\ncd 1\n\"{}\" {} &\n",
                exe.display(),
                last.arguments
            )
        );
        let middle = instances.get(1).expect("three");
        assert!(middle.arguments.contains("tcpclient:127.0.0.1:5782 "));
        // `sitl1.sh` runs the `.elf` of the same name, with Windows' paths made WSL's.
        assert!(
            last.sh.starts_with("mkdir 1\ncd 1\n\"../ArduCopter.elf\" "),
            "{}",
            last.sh
        );
        assert!(last.sh.contains("/mnt/c/sitl/copter.parm,identity.parm"));
        assert!(last.sh.ends_with(" &\nsleep .3\ncd ..\n"));
        // No defaults file: identity alone.
        let bare = chain(1, "+", "", &exe, &homes);
        assert_eq!(
            bare.first().map(|i| i.arguments.as_str()),
            Some(
                "  --defaults \"identity.parm\"  -M+ -s1 --home H0 --instance 0 --serial0 tcp:0  "
            )
        );
        // "how many?" answered 0: nothing is started.
        assert!(chain(0, "+", "", &exe, &homes).is_empty());
    }

    /// The regex finds a frame's first defaults file in a `sim_vehicle.py` of the old shape.
    #[test]
    fn sim_vehicle_py_is_searched_with_default_params_regex() {
        let text = r#"
    "quadplane": {
        "waf_target": "bin/arduplane",
        "default_params_filename": ["default_params/quadplane.parm", "x.parm"],
    },
    "+": {
        "waf_target": "bin/arducopter",
        "default_params_filename": "default_params/copter.parm",
    },
"#;
        assert_eq!(
            sim_vehicle_defaults(text),
            [
                (
                    "quadplane".to_owned(),
                    "default_params/quadplane.parm".to_owned()
                ),
                ("+".to_owned(), "default_params/copter.parm".to_owned())
            ]
        );
    }

    /// `cleanupJson` and the lenient read make `vehicleinfo.py` a tree.
    #[test]
    fn vehicleinfo_py_is_cut_down_to_json() {
        let python = "class VehicleInfo(object):\n    def __init__(self):\n        self.options = {\n    \"ArduCopter\": {  # the copter\n        \"frames\": {\n            \"+\": {\n                \"default_params_filename\": \"default_params/copter.parm\",\n                \"external\": True,\n            },\n        },\n    },\n}\n    def x(self): pass\n";
        let cleaned = cleanup_json(python);
        assert!(cleaned.starts_with('{') && cleaned.ends_with('}'));
        assert!(!cleaned.contains('#'));
        assert!(cleaned.contains("\"external\": \"True\""));
        let value: serde_json::Value =
            serde_json::from_str(&lenient_json(&cleaned)).expect("parses");
        assert_eq!(
            value
                .pointer("/ArduCopter/frames/+/default_params_filename")
                .and_then(serde_json::Value::as_str),
            Some("default_params/copter.parm")
        );
        assert_eq!(lenient_json("[1, 2 , ]"), "[1, 2  ]");
        assert_eq!(lenient_json("{\"a,\": \"}\",}"), "{\"a,\": \"}\"}");
        assert_eq!(brace_match("x { a { b } c } d", '{', '}'), "{ a { b } c }");
        assert_eq!(brace_match("{ never", '{', '}'), "");
    }
}
