//! Settings that survive a restart, in two files.
//!
//! [`Persisted`] is Mission Planner's own `config.xml`, held as `Settings.Instance` holds it: read
//! once at start-up, changed in memory by the screens where the C# changes it, and written whole
//! where the C# writes it - `SaveConfig`, on start-up, on the FLIGHT DATA and FLIGHT PLAN buttons
//! and on closing. What the ported screens keep there: the planning screen's home and panel boxes
//! (saved when the screen is left, `config(true)`), the quick view fields (when one is chosen),
//! the map type (when it is changed), the last link (when one is opened, and on every save), and
//! every key of the CONFIG screen's Planner page - the display units, the speech templates, the
//! telemetry rates and the rest - each when its control changes ([`Persisted::set`]).
//!
//! [`Settings`] is this application's own: small and deliberately dumb, a flat list of
//! `key = value` lines for what Mission Planner has no key for, and the link and map this
//! application chose last. A ground station's settings are a handful of strings and numbers, and
//! a serialisation format would be a dependency, a schema and a migration story for something a
//! person can read and fix in a text editor when it goes wrong in a field.
//!
//! Nothing here is allowed to stop the application starting. A settings file that is missing,
//! unreadable, half-written or full of rubbish yields defaults, because the alternative is a
//! ground station that will not open on the day its disk filled up.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use crate::plan::{AltitudeFrame, HomeBox, PanelBox, Plan};
use crate::quick::QuickViews;

/// What is remembered between runs.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Settings {
    /// The link last connected to, offered again next time.
    pub link: Option<String>,
    /// Which tile provider to use.
    pub tile_source: Option<String>,
    /// Window size, as width and height.
    pub window: Option<(u32, u32)>,
    /// Which screen to open on.
    pub screen: Option<String>,
    /// Where missions are read and written.
    pub plan_directory: Option<String>,
    /// The altitude frame new waypoints are created in: relative, absolute or terrain.
    ///
    /// Mission Planner keeps the same choice as `FPaltmode`. Stored as a name rather than the
    /// `MAV_FRAME` number so a hand-edited settings file is readable, and so a number that stops
    /// meaning what it did cannot silently change the frame a mission is planned in.
    pub altitude_frame: Option<String>,
}

impl Settings {
    /// Where the settings live, following each platform's convention.
    ///
    /// Configuration, not cache: losing these loses the operator's choices, so they do not belong
    /// beside the tiles, which are regenerable downloads.
    #[must_use]
    pub fn path() -> PathBuf {
        if let Some(explicit) = std::env::var_os("MP_SETTINGS") {
            return PathBuf::from(explicit);
        }
        let base = std::env::var_os("XDG_CONFIG_HOME")
            .map(PathBuf::from)
            .or_else(|| std::env::var_os("APPDATA").map(PathBuf::from))
            .or_else(|| std::env::var_os("HOME").map(|home| PathBuf::from(home).join(".config")))
            .unwrap_or_else(std::env::temp_dir);
        base.join("mission-planner-rust").join("settings.conf")
    }

    /// Reads the settings, or the defaults if anything at all is wrong.
    #[must_use]
    pub fn load() -> Self {
        let own = std::fs::read_to_string(Self::path())
            .ok()
            .map(|text| Self::parse(&text))
            .unwrap_or_default();
        let theirs = mp_settings::Config::default_path()
            .and_then(|path| mp_settings::Config::load(&path).ok());
        own.with_mission_planner_defaults(theirs.as_ref())
    }

    /// Fills what this file does not say from what Mission Planner's `config.xml` says.
    ///
    /// A pilot who has used Mission Planner on this machine has already chosen a link and a map;
    /// the first run here should open the same link on the same map rather than ask again. Only
    /// the keys both applications mean the same thing by: the link (`comport` and its
    /// companions) and the map provider (`MapType`, matched by the C# provider's name as the C#
    /// matches it, so a provider this application does not have is simply not taken). This
    /// file's own choices win once made, and with neither the map is Mission Planner's default,
    /// which `main.rs` applies where the provider is chosen.
    /// `// C#: ExtLibs/Utilities/Settings.cs:88-125; GCSViews/FlightPlanner.cs:7247-7254`
    #[must_use]
    pub fn with_mission_planner_defaults(mut self, config: Option<&mp_settings::Config>) -> Self {
        let Some(config) = config else {
            return self;
        };
        if self.link.is_none() {
            self.link = config.last_link();
        }
        if self.tile_source.is_none() {
            self.tile_source = config
                .map_type()
                .and_then(mp_tiles::source::source_by_name)
                .map(|source| source.id.to_owned());
        }
        self
    }

    /// Parses the file's contents.
    ///
    /// Unknown keys are ignored rather than rejected, so a settings file written by a newer build
    /// does not stop an older one starting - and a typo costs one setting rather than all of them.
    ///
    /// So is a file an older build wrote: the CONFIG screen's Planner page kept its keys here
    /// (`distunits = Feet`, `speechenable = True`, ...) before they moved to `config.xml`, where
    /// the C# keeps them. Those lines are unknown keys now - read as nothing, and gone from the
    /// file the next time it is written. They are not carried into `config.xml`: Mission Planner
    /// has no such step, and that build read `config.xml` for every key this file did not hold.
    #[must_use]
    pub fn parse(text: &str) -> Self {
        let mut values: BTreeMap<&str, &str> = BTreeMap::new();
        for line in text.lines() {
            let line = line.trim();
            if line.is_empty() || line.starts_with('#') {
                continue;
            }
            if let Some((key, value)) = line.split_once('=') {
                values.insert(key.trim(), value.trim());
            }
        }

        let non_empty = |key: &str| {
            values
                .get(key)
                .map(|value| (*value).to_owned())
                .filter(|value| !value.is_empty())
        };

        Self {
            link: non_empty("link"),
            tile_source: non_empty("tile_source"),
            window: values.get("window").and_then(|value| parse_size(value)),
            screen: non_empty("screen"),
            plan_directory: non_empty("plan_directory"),
            altitude_frame: non_empty("altitude_frame"),
        }
    }

    /// Renders the settings as the file's contents.
    #[must_use]
    pub fn render(&self) -> String {
        let mut out = String::from(
            "# Mission Planner (Rust) settings.\n\
             # One key = value per line. Delete a line to go back to the default.\n\n",
        );
        let mut write = |key: &str, value: &str| {
            out.push_str(key);
            out.push_str(" = ");
            out.push_str(value);
            out.push('\n');
        };
        if let Some(link) = &self.link {
            write("link", link);
        }
        if let Some(source) = &self.tile_source {
            write("tile_source", source);
        }
        if let Some((width, height)) = self.window {
            write("window", &format!("{width}x{height}"));
        }
        if let Some(screen) = &self.screen {
            write("screen", screen);
        }
        if let Some(frame) = &self.altitude_frame {
            write("altitude_frame", frame);
        }
        if let Some(directory) = &self.plan_directory {
            write("plan_directory", directory);
        }
        out
    }

    /// Writes the settings out.
    ///
    /// Through a temporary file and a rename, so a crash mid-write leaves the previous settings
    /// rather than half of the new ones. Failure is reported to the caller and is not fatal:
    /// losing a preference is not worth refusing to run.
    pub fn save(&self) -> std::io::Result<()> {
        let path = Self::path();
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent)?;
        }
        let temporary = path.with_extension("tmp");
        std::fs::write(&temporary, self.render())?;
        std::fs::rename(&temporary, &path)
    }
}

/// Where `SaveConfig` was called from. `// C#: MainV2.cs:1107, 1314, 1322, 1846, 2171`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SaveEvent {
    /// The end of `MainV2`'s constructor, "to test we have write access" - and, here, after the
    /// link given at start-up was opened, which is Connect's save as well.
    Startup,
    /// `MenuFlightData_Click`, after the flight screen is shown.
    FlightData,
    /// `MenuFlightPlanner_Click`, after the planning screen is shown.
    FlightPlanner,
    /// `MainV2_FormClosing`, after the screen showing is deactivated.
    Close,
}

impl SaveEvent {
    /// What the facts call it.
    #[must_use]
    pub const fn label(self) -> &'static str {
        match self {
            Self::Startup => "startup",
            Self::FlightData => "fly",
            Self::FlightPlanner => "plan",
            Self::Close => "close",
        }
    }
}

/// The keys the facts publish as `config.<key>`: every key the screens other than the Planner page
/// write, and the display units, which that page writes too - as does the speech alert that
/// Initial Setup's Battery Monitor writes (`config/battery_monitor.rs`). The Planner page publishes
/// each of its own keys as `config.planner.<key>` (`config/planner.rs`), from this same dictionary.
pub const PUBLISHED: [&str; 27] = [
    "TXT_homelat",
    "TXT_homelng",
    "TXT_homealt",
    "TXT_WPRad",
    "TXT_loiterrad",
    "TXT_DefaultAlt",
    "CMB_altmode",
    "FPaltmode",
    "quickView1",
    "quickView2",
    "quickView3",
    "quickView4",
    "quickView5",
    "quickView6",
    "MapType",
    "comport",
    "TCP_host",
    "TCP_port",
    "UDP_port",
    "distunits",
    "altunits",
    "speedunits",
    // Battery Monitor's "MP Alert on Low Battery".
    // C#: GCSViews/ConfigurationView/ConfigBatteryMonitoring.cs:565-601
    "speechbatteryenabled",
    "speechenable",
    "speechbattery",
    "speechbatteryvolt",
    "speechbatterypercent",
];

/// `CMB_baudrate`'s ninth item, which `MainV2` selects before anything is loaded.
/// `// C#: MainV2.cs:771-775; Controls/ConnectionControl.resx (cmb_Baud.Items8)`
const DEFAULT_BAUD: &str = "115200";

/// The key a quick view's choice is saved under: its `Name`, `quickView1` to `quickView6`.
/// `// C#: GCSViews/FlightData.cs:465, 2482`
fn quick_view_key(index: usize) -> String {
    format!("quickView{}", index + 1)
}

/// Mission Planner's `config.xml`, as `Settings.Instance` holds it.
///
/// The whole file, loaded once; the screens' changes go into it as the C# makes them, and
/// [`Persisted::save_config`] writes all of it, keys this application never touches included, on
/// the C#'s events. A value the C# changes in memory and saves later is changed in memory here and
/// saved later too: a planning screen left for the SETUP screen, which saves nothing, is on disk
/// only after the next save, and nothing is on disk after a kill that the last save did not write.
#[derive(Debug)]
pub struct Persisted {
    /// The file, or `None` with no home directory to find it under.
    path: Option<PathBuf>,
    /// `Settings.config`.
    config: mp_settings::Config,
    /// Why the file was not read, if it exists and could not be. Such a file is not written over.
    unreadable: Option<String>,
    /// `MainV2.comPortName`: the connection box's port, which every save writes as `comport`.
    comport: String,
    /// `CMB_baudrate.Text`, which every save writes as `<comport>_BAUD`.
    baud: String,
    /// The quick views' fields as last seen, so a choice is written when it is made.
    quick_seen: [String; 6],
    /// Saves made this session, the last one's event, and its error if it failed.
    saves: usize,
    last_save: Option<SaveEvent>,
    save_error: Option<String>,
}

impl Persisted {
    /// `Settings.Instance`, from where the C# keeps it or wherever `MP_CONFIG_XML` says.
    #[must_use]
    pub fn load() -> Self {
        Self::at(mp_settings::Config::default_path())
    }

    /// `Settings.Instance` from one file, then what `MainV2`'s start-up takes from it for the
    /// connection box: the port, and the baud rate saved for it.
    ///
    /// A missing or empty file is an empty dictionary, as `Load` leaves it. One that exists and
    /// cannot be read is an empty dictionary too, and is remembered as such: the C# copies it to
    /// `config.xml<time>.failed` and saves over it, but this parser is not the C#'s, and a file
    /// Mission Planner reads that this one refuses is not to be replaced by an empty one.
    /// `// C#: ExtLibs/Utilities/Settings.cs:22-36, 427-505; MainV2.cs:782-808`
    #[must_use]
    pub fn at(path: Option<PathBuf>) -> Self {
        let (config, unreadable) = match path.as_deref().map(read_config) {
            Some(Ok(config)) => (config, None),
            Some(Err(why)) => (mp_settings::Config::default(), Some(why)),
            None => (mp_settings::Config::default(), None),
        };
        let comport = config.get("comport").unwrap_or_default().to_owned();
        let baud = config
            .get(&format!("{comport}_BAUD"))
            .filter(|baud| !baud.is_empty())
            .unwrap_or(DEFAULT_BAUD)
            .to_owned();
        Self {
            path,
            config,
            unreadable,
            comport,
            baud,
            quick_seen: crate::quick::DEFAULTS.map(|(name, _)| name.to_owned()),
            saves: 0,
            last_save: None,
            save_error: None,
        }
    }

    /// The dictionary.
    #[must_use]
    pub const fn config(&self) -> &mp_settings::Config {
        &self.config
    }

    /// `Settings.Instance[key]`: the dictionary's value, or `null`.
    /// `// C#: ExtLibs/Utilities/Settings.cs:49-56`
    #[must_use]
    pub fn get(&self, key: &str) -> Option<&str> {
        self.config.get(key)
    }

    /// `Settings.Instance[key] = value`: in the dictionary at once, and in the file at the next
    /// [`Persisted::save_config`] - a screen's handler writes nothing to disk itself.
    /// `// C#: ExtLibs/Utilities/Settings.cs:58-61`
    pub fn set(&mut self, key: &str, value: impl Into<String>) {
        self.config.set(key, value);
    }

    /// The frame `CMB_altmode` shows when the planning screen loads: `config(false)` puts back the
    /// text `config(true)` saved, if it names one of the three. Without it the box shows its first
    /// item and `FPaltmode` goes only to `currentaltmode`, which no ported code reads - so `None`,
    /// and the caller keeps its default.
    /// `// C#: GCSViews/FlightPlanner.cs:102, 234-239, 2609-2610`
    #[must_use]
    pub fn altitude_frame(&self) -> Option<AltitudeFrame> {
        let text = self.config.get("CMB_altmode")?;
        AltitudeFrame::all()
            .into_iter()
            .find(|frame| frame.combo_text() == text)
    }

    /// `FlightData.Activate`'s loop over the quick views: one whose key is saved shows that
    /// property. Put in the view through the chooser's own calls, as a check box checked would -
    /// the quick page has no other way in.
    /// `// C#: GCSViews/FlightData.cs:462-494`
    pub fn restore_quick_views(&mut self, views: &mut QuickViews) {
        for (index, seen) in self.quick_seen.iter_mut().enumerate() {
            if let Some(name) = self.config.get(&quick_view_key(index)) {
                views.open(index);
                views.choose(name);
                views.close();
            }
            views.field(index).clone_into(seen);
        }
    }

    /// `chk_box_quickview_CheckedChanged`: a view bound to another property has it saved under
    /// the view's name. Called every frame; a field that differs from the one last seen is a
    /// choice made since.
    /// `// C#: GCSViews/FlightData.cs:2475-2482`
    pub fn observe_quick_views(&mut self, views: &QuickViews) {
        for (index, seen) in self.quick_seen.iter_mut().enumerate() {
            let field = views.field(index);
            if seen != field {
                self.config.set(quick_view_key(index), field);
                field.clone_into(seen);
            }
        }
    }

    /// `FlightPlanner.config(true)`, which `Deactivate` runs each time the planning screen is
    /// left: the Home Location boxes, the panel boxes and the altitude frame's box, each as its
    /// text. (`fpminaltwarning` and `fpcoordmouse` go with them, from `TXT_altwarn` and `coords1`,
    /// which are not ported; the file keeps what it holds for them.)
    /// `// C#: GCSViews/FlightPlanner.cs:340-344, 2572-2592`
    pub fn planner_deactivated(&mut self, plan: &Plan, frame: AltitudeFrame) {
        self.config.set("TXT_homelat", plan.home_text(HomeBox::Lat));
        self.config.set("TXT_homelng", plan.home_text(HomeBox::Lng));
        self.config.set("TXT_homealt", plan.home_text(HomeBox::Alt));
        for which in PanelBox::ALL {
            self.config.set(which.config_key(), plan.panel_text(which));
        }
        self.config.set("CMB_altmode", frame.combo_text());
    }

    /// `CMB_altmode_SelectedIndexChanged`: the frame chosen, as the `altmode` number.
    /// `// C#: GCSViews/FlightPlanner.cs:2157-2167`
    pub fn altitude_frame_changed(&mut self, frame: AltitudeFrame) {
        self.config.set("FPaltmode", frame.mav_frame().to_string());
    }

    /// `comboBoxMapType_SelectedValueChanged`: the box's text, which is the provider's `Name`.
    /// `// C#: GCSViews/FlightPlanner.cs:2209-2237`
    pub fn map_type_changed(&mut self, source: &mp_tiles::source::TileSource) {
        self.config.set("MapType", source.cache_name);
    }

    /// A link opened, as Connect opens one from the connection box.
    ///
    /// The box's port becomes `comPortName` and takes back the baud rate saved for it; a serial
    /// link's own baud rate is then the box's. A TCP link's `Open` saves its host and port, and a
    /// UDP link's its port, through `CommsBase.Settings`, which writes to the dictionary. A log
    /// played back and a TCP listener are not ports the box has, and change nothing.
    /// `// C#: MainV2.cs:1962-1984, 1841-1847; ExtLibs/Comms/CommsTCPSerial.cs:138-143;
    /// ExtLibs/Comms/CommsUdpSerial.cs:116-118; Program.cs:661-667`
    pub fn link_opened(&mut self, url: &str) {
        let Ok(link) = url.parse::<mp_transport::LinkUrl>() else {
            return;
        };
        match link {
            mp_transport::LinkUrl::Tcp { host, port } => {
                self.select_port("TCP");
                self.config.set("TCP_port", port.to_string());
                self.config.set("TCP_host", host);
            }
            mp_transport::LinkUrl::Udp { port, .. } => {
                self.select_port("UDP");
                self.config.set("UDP_port", port.to_string());
            }
            mp_transport::LinkUrl::Serial { path, baud } => {
                self.select_port(&path);
                self.baud = baud.to_string();
            }
            // `CommsUDPSerialConnect.Open` saves the host and port it was given, `CommsWebSocket.Open`
            // its URL; NTRIP is not one of the connection box's ports.
            // `// C#: ExtLibs/Comms/CommsUDPSerialConnect.cs:78-79; ExtLibs/Comms/CommsWebSocket.cs:109`
            mp_transport::LinkUrl::UdpClient { host, port } => {
                self.select_port("UDPCl");
                self.config.set("UDP_port", port.to_string());
                self.config.set("UDP_host", host);
            }
            mp_transport::LinkUrl::WebSocket { url } => {
                self.select_port("WS");
                self.config.set("WS_url", url);
            }
            mp_transport::LinkUrl::TcpListen { .. }
            | mp_transport::LinkUrl::File { .. }
            | mp_transport::LinkUrl::Ntrip { .. } => {}
        }
    }

    /// `CMB_serialport_SelectedIndexChanged`: the port chosen, and the baud box restored.
    /// `// C#: MainV2.cs:1962-1984`
    fn select_port(&mut self, port: &str) {
        port.clone_into(&mut self.comport);
        if let Some(saved) = self.config.get(&format!("{}_BAUD", port.replace(' ', "_"))) {
            saved.clone_into(&mut self.baud);
        }
    }

    /// `MainV2.SaveConfig`: the connection box's port and baud rate put back, then the whole file
    /// written. (`APMFirmware`, the vehicle's firmware, is put back with them in the C#; this
    /// application holds no `cs.firmware` to write, so the file keeps what it has.)
    /// `// C#: MainV2.cs:2219-2237; ExtLibs/Utilities/Settings.cs:88-92, 111-125, 507-550`
    pub fn save_config(&mut self, event: SaveEvent) -> Result<(), String> {
        self.config.set("comport", self.comport.clone());
        self.config
            .set(format!("{}_BAUD", self.comport), self.baud.clone());
        self.last_save = Some(event);
        let result = match (&self.path, &self.unreadable) {
            (None, _) => Ok(()),
            (Some(path), Some(why)) => Err(format!(
                "{} was not written: it could not be read at start-up ({why})",
                path.display()
            )),
            (Some(path), None) => self.config.save(path).map_err(|err| err.to_string()),
        };
        match &result {
            Ok(()) => {
                self.saves += 1;
                self.save_error = None;
            }
            Err(why) => self.save_error = Some(why.clone()),
        }
        result
    }

    /// The saves made this session and the last one's event: `config.saves` and `config.saved`.
    #[cfg(test)]
    #[must_use]
    pub const fn saves(&self) -> (usize, Option<SaveEvent>) {
        (self.saves, self.last_save)
    }

    /// Publishes what a UI test asserts on: each key this application writes, as the dictionary
    /// holds it (`none` when absent), the link the file names, and the saves made.
    pub fn record_facts(&self) {
        for key in PUBLISHED {
            crate::facts::record(
                format!("config.{key}"),
                self.config.get(key).unwrap_or("none"),
            );
        }
        crate::facts::record(
            "config.link",
            self.config.last_link().unwrap_or_else(|| "none".to_owned()),
        );
        crate::facts::record("config.saves", self.saves);
        crate::facts::record(
            "config.saved",
            self.last_save.map_or("none", SaveEvent::label),
        );
        crate::facts::record("config.error", self.save_error.as_deref().unwrap_or("none"));
    }
}

/// Reads `config.xml` for [`Persisted::at`]: a missing or empty file is an empty dictionary, and
/// any other failure is the reason.
fn read_config(path: &Path) -> Result<mp_settings::Config, String> {
    let text = match std::fs::read_to_string(path) {
        Ok(text) => text,
        Err(err) if err.kind() == std::io::ErrorKind::NotFound => {
            return Ok(mp_settings::Config::default());
        }
        Err(err) => return Err(err.to_string()),
    };
    if text.trim_start_matches('\u{feff}').trim().is_empty() {
        return Ok(mp_settings::Config::default());
    }
    mp_settings::Config::parse(&text).map_err(|err| err.to_string())
}

/// Parses a `WIDTHxHEIGHT` size.
fn parse_size(value: &str) -> Option<(u32, u32)> {
    let (width, height) = value.split_once(['x', 'X'])?;
    let width: u32 = width.trim().parse().ok()?;
    let height: u32 = height.trim().parse().ok()?;
    // The same floor the command line enforces. A remembered 1x1 window would be unusable and
    // unfixable without editing the file by hand.
    (width >= 640 && height >= 480).then_some((width, height))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A file Mission Planner's `XmlTextWriter` wrote, from `mp-settings`'s fixtures.
    const CSHARP_FILE: &str = include_str!("../../mp-settings/tests/fixtures/config.xml");

    /// A directory of its own for one test, removed afterwards.
    struct Scratch(PathBuf);

    impl Scratch {
        fn new(name: &str) -> Self {
            let path = std::env::temp_dir()
                .join(format!("mp-gui-persisted-{name}-{}", std::process::id()));
            let _ = std::fs::remove_dir_all(&path);
            std::fs::create_dir_all(&path).expect("scratch directory");
            Self(path)
        }

        /// Where `config.xml` goes, under a data directory that does not exist yet - the C#
        /// makes it, and so does a save here.
        fn config(&self) -> PathBuf {
            self.0.join("Mission Planner").join("config.xml")
        }

        fn seed(&self, text: &str) -> PathBuf {
            let path = self.config();
            std::fs::create_dir_all(path.parent().expect("parent")).expect("data directory");
            std::fs::write(&path, text).expect("seed config.xml");
            path
        }
    }

    impl Drop for Scratch {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.0);
        }
    }

    fn reload(path: &Path) -> mp_settings::Config {
        mp_settings::Config::load(path).expect("the saved file reads back")
    }

    #[test]
    fn the_planning_screen_left_is_saved_and_comes_back_after_a_restart() {
        let scratch = Scratch::new("planner");
        let path = scratch.config();
        let mut persisted = Persisted::at(Some(path.clone()));

        // Typed into the boxes, as TXT_home*_TextChanged and the panel's KeyPress take them.
        let mut plan = Plan::default();
        plan.set_home_text(HomeBox::Lat, "-35.25".to_owned());
        plan.set_home_text(HomeBox::Lng, "149.5".to_owned());
        plan.set_home_text(HomeBox::Alt, "600".to_owned());
        plan.set_panel_text(PanelBox::WpRadius, "12");
        plan.set_panel_text(PanelBox::DefaultAlt, "120");

        // Nothing reaches the file until the screen is left and a save follows it.
        persisted.planner_deactivated(&plan, AltitudeFrame::Terrain);
        assert!(!path.exists());
        persisted.save_config(SaveEvent::FlightData).expect("saved");
        let saved = reload(&path);
        assert_eq!(saved.get("TXT_homelat"), Some("-35.25"));
        assert_eq!(saved.get("TXT_homelng"), Some("149.5"));
        assert_eq!(saved.get("TXT_homealt"), Some("600"));
        assert_eq!(saved.get("TXT_WPRad"), Some("12"));
        // Loiter Radius was never touched and is saved as the .resx left it.
        assert_eq!(saved.get("TXT_loiterrad"), Some("45"));
        assert_eq!(saved.get("TXT_DefaultAlt"), Some("120"));
        assert_eq!(saved.get("CMB_altmode"), Some("Terrain"));

        // The next start: MainV2 reads the planned home, FlightPlanner_Load the boxes, and
        // Activate puts the home in its boxes - the path main.rs takes.
        let restarted = Persisted::at(Some(path));
        let mut plan = Plan::default();
        plan.set_planned_home(crate::plan::planned_home_from_config(Some(
            restarted.config(),
        )));
        plan.apply_panel_config(Some(restarted.config()));
        plan.update_home_text(None);
        assert_eq!(plan.home_text(HomeBox::Lat), "-35.25");
        assert_eq!(plan.home_text(HomeBox::Lng), "149.5");
        assert_eq!(plan.home_text(HomeBox::Alt), "600.00");
        assert_eq!(plan.panel_text(PanelBox::WpRadius), "12");
        assert_eq!(plan.panel_text(PanelBox::LoiterRadius), "45");
        assert_eq!(plan.panel_text(PanelBox::DefaultAlt), "120");
        assert_eq!(restarted.altitude_frame(), Some(AltitudeFrame::Terrain));
    }

    #[test]
    fn empty_home_boxes_are_saved_empty() {
        // config(true) writes the boxes' text whatever it is; the .resx leaves them empty.
        let scratch = Scratch::new("empty-home");
        let path = scratch.config();
        let mut persisted = Persisted::at(Some(path.clone()));
        persisted.planner_deactivated(&Plan::default(), AltitudeFrame::Relative);
        persisted.save_config(SaveEvent::Close).expect("saved");
        let text = std::fs::read_to_string(&path).expect("read");
        assert!(text.contains("\n  <TXT_homelat />"), "{text}");
        assert_eq!(reload(&path).get("TXT_homelat"), Some(""));
        // Which MainV2 reads as no home: GetDouble of "" is 0.
        assert_eq!(
            crate::plan::planned_home_from_config(Some(&reload(&path))),
            mp_mission::rows::Home::default()
        );
    }

    #[test]
    fn the_altitude_frame_changed_is_saved_as_its_number() {
        let scratch = Scratch::new("fpaltmode");
        let mut persisted = Persisted::at(Some(scratch.config()));
        persisted.altitude_frame_changed(AltitudeFrame::Absolute);
        assert_eq!(persisted.config().get("FPaltmode"), Some("0"));
        persisted.altitude_frame_changed(AltitudeFrame::Terrain);
        assert_eq!(persisted.config().get("FPaltmode"), Some("10"));
        // FPaltmode alone does not choose the box's frame; CMB_altmode does.
        assert_eq!(persisted.altitude_frame(), None);
    }

    #[test]
    fn a_quick_view_chosen_is_saved_and_comes_back() {
        let scratch = Scratch::new("quick");
        let path = scratch.config();
        let mut persisted = Persisted::at(Some(path.clone()));
        let mut views = QuickViews::default();
        persisted.restore_quick_views(&mut views);
        persisted.observe_quick_views(&views);
        assert_eq!(persisted.config().get("quickView3"), None);

        // Double click the third, check satcount: chk_box_quickview_CheckedChanged.
        views.open(2);
        views.choose("satcount");
        persisted.observe_quick_views(&views);
        assert_eq!(persisted.config().get("quickView3"), Some("satcount"));
        // A view never chosen has no key, as in a real config.xml.
        assert_eq!(persisted.config().get("quickView1"), None);
        persisted
            .save_config(SaveEvent::FlightPlanner)
            .expect("saved");

        // FlightData.Activate after a restart.
        let mut restarted = Persisted::at(Some(path));
        let mut views = QuickViews::default();
        restarted.restore_quick_views(&mut views);
        assert_eq!(views.field(2), "satcount");
        assert_eq!(views.field(0), "alt");
        assert_eq!(views.choosing(), None);
        // Restoring is not a choice: nothing new is written on the next frame.
        let before = restarted.config().clone();
        restarted.observe_quick_views(&views);
        assert_eq!(restarted.config(), &before);
    }

    #[test]
    fn the_map_type_is_saved_as_the_csharp_providers_name() {
        let scratch = Scratch::new("maptype");
        let path = scratch.config();
        let mut persisted = Persisted::at(Some(path.clone()));
        let bing = mp_tiles::source::source_by_id("bing-satellite").expect("ported");
        persisted.map_type_changed(bing);
        persisted.save_config(SaveEvent::Close).expect("saved");
        let saved = reload(&path);
        assert_eq!(saved.get("MapType"), Some("BingSatelliteMap"));
        // And read back as this application reads it.
        let taken = Settings::default().with_mission_planner_defaults(Some(&saved));
        assert_eq!(taken.tile_source.as_deref(), Some("bing-satellite"));
    }

    #[test]
    fn a_link_opened_is_saved_as_the_csharps_connect_saves_it() {
        let scratch = Scratch::new("link");
        let path = scratch.seed(CSHARP_FILE);

        let mut persisted = Persisted::at(Some(path.clone()));
        persisted.link_opened("tcp:10.0.0.5:5762");
        persisted.save_config(SaveEvent::Startup).expect("saved");
        let saved = reload(&path);
        assert_eq!(saved.get("comport"), Some("TCP"));
        assert_eq!(saved.get("TCP_host"), Some("10.0.0.5"));
        assert_eq!(saved.get("TCP_port"), Some("5762"));
        // The baud box took TCP_BAUD back when TCP was chosen, and it is saved again.
        assert_eq!(saved.get("TCP_BAUD"), Some("115200"));
        assert_eq!(saved.last_link().as_deref(), Some("tcp:10.0.0.5:5762"));

        let mut persisted = Persisted::at(Some(path.clone()));
        persisted.link_opened("udp:14551");
        persisted.save_config(SaveEvent::Startup).expect("saved");
        let saved = reload(&path);
        assert_eq!(saved.get("comport"), Some("UDP"));
        assert_eq!(saved.get("UDP_port"), Some("14551"));
        // What the baud box held for TCP, saved for UDP, as the C#'s box keeps its text.
        assert_eq!(saved.get("UDP_BAUD"), Some("115200"));
        assert_eq!(saved.last_link().as_deref(), Some("udp:14551"));

        let mut persisted = Persisted::at(Some(path.clone()));
        persisted.link_opened("serial:COM3:57600");
        persisted.save_config(SaveEvent::Startup).expect("saved");
        assert_eq!(
            reload(&path).last_link().as_deref(),
            Some("serial:COM3:57600")
        );

        // A Linux device's baud rate is not saved: the key has slashes (Settings.cs:524-542).
        let mut persisted = Persisted::at(Some(path.clone()));
        persisted.link_opened("/dev/ttyACM0");
        persisted.save_config(SaveEvent::Startup).expect("saved");
        let saved = reload(&path);
        assert_eq!(saved.get("comport"), Some("/dev/ttyACM0"));
        assert_eq!(saved.last_link().as_deref(), Some("serial:/dev/ttyACM0"));

        // A log played back is not a port: the file says what it said.
        let before = std::fs::read_to_string(&path).expect("read");
        let mut persisted = Persisted::at(Some(path.clone()));
        persisted.link_opened("file:flight.tlog");
        persisted.save_config(SaveEvent::Startup).expect("saved");
        assert_eq!(std::fs::read_to_string(&path).expect("read"), before);
    }

    #[test]
    fn a_save_writes_the_whole_file_back_as_the_csharp_wrote_it() {
        // SaveConfig at start-up, before anything has changed: every key the C# wrote, in its
        // bytes, including the ones no ported screen knows.
        let scratch = Scratch::new("whole");
        let path = scratch.seed(CSHARP_FILE);
        let mut persisted = Persisted::at(Some(path.clone()));
        persisted.save_config(SaveEvent::Startup).expect("saved");
        assert_eq!(std::fs::read_to_string(&path).expect("read"), CSHARP_FILE);
    }

    #[test]
    fn the_real_config_is_written_back_unchanged_when_nothing_changed() {
        // The real file on this machine, copied: Load, the start-up save, byte for byte - comport
        // a device path whose baud key the C# never writes, and 70-odd keys this application
        // does not know.
        let Some(real) = mp_settings::Config::default_path() else {
            eprintln!("skipped: no home directory");
            return;
        };
        let Ok(text) = std::fs::read_to_string(&real) else {
            eprintln!("skipped: no Mission Planner config at {}", real.display());
            return;
        };
        let scratch = Scratch::new("real");
        let path = scratch.seed(&text);
        let mut persisted = Persisted::at(Some(path.clone()));
        persisted.save_config(SaveEvent::Startup).expect("saved");
        assert_eq!(std::fs::read_to_string(&path).expect("read"), text);
        // And a planning screen left as it was loaded changes only what config(true) writes.
        let mut plan = Plan::default();
        plan.set_planned_home(crate::plan::planned_home_from_config(Some(
            persisted.config(),
        )));
        plan.apply_panel_config(Some(persisted.config()));
        plan.update_home_text(None);
        let frame = persisted.altitude_frame().unwrap_or_default();
        persisted.planner_deactivated(&plan, frame);
        persisted.save_config(SaveEvent::FlightData).expect("saved");
        let before = mp_settings::Config::parse(&text).expect("parses");
        let after = reload(&path);
        for key in before.keys() {
            if !key.starts_with("TXT_") && key != "CMB_altmode" {
                assert_eq!(after.get(key), before.get(key), "{key}");
            }
        }
    }

    #[test]
    fn the_display_units_survive_every_save() {
        // Only the Planner page changes them, and it is not shown here; the file is written
        // whole, so what Mission Planner chose is still there for it, and for ChangeUnits.
        let scratch = Scratch::new("units");
        let path = scratch.seed(
            "\u{feff}<?xml version=\"1.0\" encoding=\"utf-8\"?>\n<Config>\n  <altunits>Feet</altunits>\n  <distunits>Feet</distunits>\n  <speedunits>knots</speedunits>\n</Config>",
        );
        let mut persisted = Persisted::at(Some(path.clone()));
        persisted.planner_deactivated(&Plan::default(), AltitudeFrame::Relative);
        persisted.map_type_changed(mp_tiles::source::default_source());
        persisted.save_config(SaveEvent::Close).expect("saved");
        let saved = reload(&path);
        let units = mp_vehicle::units::DisplayUnits::default().change_units(
            saved.get("distunits"),
            saved.get("altunits"),
            saved.get("speedunits"),
        );
        assert_eq!(units.dist_unit, "ft");
        assert_eq!(units.alt_unit, "ft");
        assert_eq!(units.speed_unit, "kts");
    }

    #[test]
    fn a_fresh_installation_saves_what_save_config_puts_back() {
        // No file: comPortName is "" and the baud box holds its ninth item, and SaveConfig writes
        // both - comport as an empty element, the baud under "_BAUD".
        let scratch = Scratch::new("fresh");
        let path = scratch.config();
        let mut persisted = Persisted::at(Some(path.clone()));
        persisted.save_config(SaveEvent::Startup).expect("saved");
        assert_eq!(
            std::fs::read_to_string(&path).expect("read"),
            "\u{feff}<?xml version=\"1.0\" encoding=\"utf-8\"?>\n<Config>\n  <_BAUD>115200</_BAUD>\n  <comport />\n</Config>"
        );
        // Which names no link.
        assert_eq!(reload(&path).last_link(), None);
    }

    /// `tests/gui/settings-persist.gui`, step for step, against the model: the same changes at
    /// the same points, a close, a restart - and what the script expects of `config.<key>` after
    /// the restart is what the model read back. The script runs with a window; this does not.
    #[test]
    fn the_restart_script_expects_what_the_model_saves() {
        let script = include_str!("../../../tests/gui/settings-persist.gui");
        let seeded = "\u{feff}<?xml version=\"1.0\" encoding=\"utf-8\"?>\n<Config>\n  <altunits>Feet</altunits>\n  <distunits>Feet</distunits>\n  <MapType>OpenStreetMap</MapType>\n  <speedunits>knots</speedunits>\n</Config>";
        let setup = script
            .lines()
            .find(|line| line.starts_with("setup printf"))
            .expect("the script seeds its config.xml");
        assert!(
            setup.contains(
                &seeded
                    .replace('\u{feff}', "\\357\\273\\277")
                    .replace('\n', "\\n")
            ),
            "the script's seed and this one differ: {setup}"
        );
        let scratch = Scratch::new("script");
        let path = scratch.seed(seeded);

        // Start on the flight screen.
        let mut persisted = Persisted::at(Some(path.clone()));
        let mut views = QuickViews::default();
        persisted.restore_quick_views(&mut views);
        persisted.save_config(SaveEvent::Startup).expect("saved");
        // The quick view, then FLIGHT PLAN.
        views.open(2);
        views.choose("satcount");
        persisted.observe_quick_views(&views);
        persisted
            .save_config(SaveEvent::FlightPlanner)
            .expect("saved");
        // The planning screen's changes.
        let mut plan = Plan::default();
        plan.set_home_text(HomeBox::Lat, "51.25".to_owned());
        plan.set_home_text(HomeBox::Lng, "7.5".to_owned());
        plan.set_home_text(HomeBox::Alt, "600".to_owned());
        plan.set_panel_text(PanelBox::WpRadius, "12");
        persisted.altitude_frame_changed(AltitudeFrame::Terrain);
        persisted
            .map_type_changed(mp_tiles::source::source_by_id("bing-satellite").expect("ported"));
        // The close box, on the planning screen.
        persisted.planner_deactivated(&plan, AltitudeFrame::Terrain);
        persisted.observe_quick_views(&views);
        persisted.save_config(SaveEvent::Close).expect("saved");

        // The restart.
        let mut restarted = Persisted::at(Some(path));
        let mut views = QuickViews::default();
        restarted.restore_quick_views(&mut views);
        restarted.save_config(SaveEvent::Startup).expect("saved");
        let after = script
            .split("\nrestart\n")
            .nth(1)
            .expect("the script restarts");
        let mut checked = 0;
        for line in after.lines() {
            let Some(rest) = line.strip_prefix("expect config.") else {
                continue;
            };
            let (key, want) = rest.split_once(' ').expect("expect key value");
            let got = match key {
                "saved" => SaveEvent::Startup.label().to_owned(),
                "saves" => "1".to_owned(),
                key => restarted.config().get(key).unwrap_or("none").to_owned(),
            };
            assert_eq!(got, want, "config.{key}");
            checked += 1;
        }
        assert!(checked >= 15, "{checked} config facts checked");

        // What the screens show, from what was read back.
        assert!(after.contains("expect fly.quick.3 satcount"));
        assert_eq!(views.field(2), "satcount");
        let mut plan = Plan::default();
        plan.set_planned_home(crate::plan::planned_home_from_config(Some(
            restarted.config(),
        )));
        plan.apply_panel_config(Some(restarted.config()));
        plan.update_home_text(None);
        let home = HomeBox::ALL
            .iter()
            .map(|which| plan.home_text(*which))
            .collect::<Vec<_>>()
            .join(",");
        assert!(
            after.contains(&format!("expect plan.home {home}")),
            "{home}"
        );
        assert!(after.contains(&format!(
            "expect plan.wprad {}",
            plan.panel_text(PanelBox::WpRadius)
        )));
        assert_eq!(restarted.altitude_frame(), Some(AltitudeFrame::Terrain));
        assert!(after.contains("expect plan.frame terrain"));
    }

    #[test]
    fn a_file_that_cannot_be_read_is_not_written_over() {
        let scratch = Scratch::new("unreadable");
        let path = scratch.seed("<Config><unclosed></Config>");
        let mut persisted = Persisted::at(Some(path.clone()));
        let refused = persisted.save_config(SaveEvent::Startup);
        assert!(refused.is_err(), "{refused:?}");
        assert_eq!(
            std::fs::read_to_string(&path).expect("read"),
            "<Config><unclosed></Config>"
        );
        // An empty one is the empty dictionary Load leaves, and is written.
        let path = scratch.seed("");
        let mut persisted = Persisted::at(Some(path.clone()));
        persisted.save_config(SaveEvent::Startup).expect("saved");
        assert_eq!(reload(&path).get("comport"), Some(""));
    }

    #[test]
    fn mission_planners_config_fills_what_ours_does_not_say() {
        let mut theirs = mp_settings::Config::default();
        theirs.set("comport", "TCP");
        theirs.set("TCP_host", "127.0.0.1");
        theirs.set("TCP_port", "5760");
        theirs.set("MapType", "OpenStreetMap");

        let fresh = Settings::default().with_mission_planner_defaults(Some(&theirs));
        assert_eq!(fresh.link.as_deref(), Some("tcp:127.0.0.1:5760"));
        assert_eq!(fresh.tile_source.as_deref(), Some("osm"));

        // Our own choices, once made, are not overridden.
        let chosen = Settings {
            link: Some("udp:14550".to_owned()),
            tile_source: Some("esri-imagery".to_owned()),
            ..Settings::default()
        }
        .with_mission_planner_defaults(Some(&theirs));
        assert_eq!(chosen.link.as_deref(), Some("udp:14550"));
        assert_eq!(chosen.tile_source.as_deref(), Some("esri-imagery"));

        // Mission Planner's own default, which is what a real config.xml usually holds.
        theirs.set("MapType", "GoogleSatelliteMap");
        let fresh = Settings::default().with_mission_planner_defaults(Some(&theirs));
        assert_eq!(fresh.tile_source.as_deref(), Some("google-satellite"));
        theirs.set("MapType", "BingHybridMap");
        let fresh = Settings::default().with_mission_planner_defaults(Some(&theirs));
        assert_eq!(fresh.tile_source.as_deref(), Some("bing-hybrid"));

        // A provider this application does not have is not taken. GoogleHybridMap is the C#'s,
        // and not ported: it is two layers.
        theirs.set("MapType", "GoogleHybridMap");
        let fresh = Settings::default().with_mission_planner_defaults(Some(&theirs));
        assert_eq!(fresh.tile_source, None);
        assert_eq!(
            Settings::default().with_mission_planner_defaults(None),
            Settings::default()
        );
    }

    /// The real `config.xml` on this machine, when there is one: the map it names is the map
    /// shown, if that provider is ported.
    #[test]
    fn the_map_the_real_mission_planner_last_showed_is_taken() {
        let Some(path) = mp_settings::Config::default_path() else {
            eprintln!("skipped: no home directory");
            return;
        };
        let Ok(config) = mp_settings::Config::load(&path) else {
            eprintln!("skipped: no Mission Planner config at {}", path.display());
            return;
        };
        let Some(map_type) = config.map_type() else {
            eprintln!("skipped: {} names no MapType", path.display());
            return;
        };
        let taken = Settings::default().with_mission_planner_defaults(Some(&config));
        let expected = mp_tiles::source::source_by_name(map_type).map(|source| source.id);
        eprintln!("MapType {map_type} -> {:?}", taken.tile_source);
        assert_eq!(taken.tile_source.as_deref(), expected);
    }

    #[test]
    fn settings_round_trip_through_the_file_format() {
        let settings = Settings {
            link: Some("tcp:127.0.0.1:5760".to_owned()),
            tile_source: Some("osm".to_owned()),
            window: Some((1600, 1200)),
            screen: Some("plan".to_owned()),
            plan_directory: Some("/home/pilot/missions".to_owned()),
            altitude_frame: Some("terrain".to_owned()),
        };
        assert_eq!(Settings::parse(&settings.render()), settings);
    }

    /// A settings file written while the Planner page kept its keys here: everything this file
    /// still keeps loads, the page's lines are read as nothing - its keys are `config.xml`'s now -
    /// and the next write leaves them out.
    #[test]
    fn a_file_with_the_planner_keys_an_older_build_wrote_still_loads() {
        let older = "# Mission Planner (Rust) settings.\n\
                     # One key = value per line. Delete a line to go back to the default.\n\n\
                     link = tcp:127.0.0.1:5760\n\
                     tile_source = osm\n\
                     window = 1600x1200\n\
                     screen = config\n\
                     altitude_frame = terrain\n\
                     plan_directory = /home/pilot/missions\n\
                     CMB_rateattitude = 10\n\
                     distunits = Feet\n\
                     severity = 4\n\
                     speechcustom = Heading to Waypoint {wpn}, altitude is {alt}, Ground speed is {gsp} \n\
                     speechenable = True\n";
        let loaded = Settings::parse(older);
        assert_eq!(
            loaded,
            Settings {
                link: Some("tcp:127.0.0.1:5760".to_owned()),
                tile_source: Some("osm".to_owned()),
                window: Some((1600, 1200)),
                screen: Some("config".to_owned()),
                plan_directory: Some("/home/pilot/missions".to_owned()),
                altitude_frame: Some("terrain".to_owned()),
            }
        );
        let written = loaded.render();
        for key in ["CMB_rateattitude", "distunits", "severity", "speech"] {
            assert!(!written.contains(key), "{key} is written back: {written}");
        }
        assert_eq!(Settings::parse(&written), loaded);
    }

    #[test]
    fn an_empty_file_yields_defaults() {
        assert_eq!(Settings::parse(""), Settings::default());
        assert_eq!(Settings::parse("\n\n   \n"), Settings::default());
    }

    #[test]
    fn rubbish_yields_defaults_rather_than_refusing_to_start() {
        // A ground station that will not open on the day its disk filled up is worse than one
        // that opens having forgotten a preference.
        let settings = Settings::parse("\u{0}\u{1}garbage without any equals signs at all");
        assert_eq!(settings, Settings::default());
    }

    #[test]
    fn unknown_keys_are_ignored_rather_than_rejected() {
        // A file written by a newer build must not stop an older one starting, and a typo should
        // cost one setting rather than all of them.
        let settings = Settings::parse("link = udp:14550\nfuture_thing = 42\nscreen = fly\n");
        assert_eq!(settings.link.as_deref(), Some("udp:14550"));
        assert_eq!(settings.screen.as_deref(), Some("fly"));
    }

    #[test]
    fn comments_and_whitespace_are_tolerated() {
        let settings = Settings::parse("# a comment\n\n  link   =   udp:14550   \n");
        assert_eq!(settings.link.as_deref(), Some("udp:14550"));
    }

    #[test]
    fn an_empty_value_is_the_same_as_not_set() {
        // Otherwise clearing a field in the file would remember an empty string and the
        // application would try to connect to nothing.
        let settings = Settings::parse("link =\nscreen =   \n");
        assert_eq!(settings.link, None);
        assert_eq!(settings.screen, None);
    }

    #[test]
    fn an_unusable_window_size_is_refused() {
        // A remembered 1x1 window would be unusable and unfixable without editing the file.
        assert_eq!(Settings::parse("window = 1x1").window, None);
        assert_eq!(Settings::parse("window = wide").window, None);
        assert_eq!(
            Settings::parse("window = 1600x1200").window,
            Some((1600, 1200))
        );
    }

    #[test]
    fn a_value_containing_an_equals_sign_survives() {
        // A link URL can contain one, and splitting on the last would lose the tail.
        let settings = Settings::parse("link = udp:14550?opt=1\n");
        assert_eq!(settings.link.as_deref(), Some("udp:14550?opt=1"));
    }

    #[test]
    fn the_file_lives_with_configuration_not_with_the_cache() {
        // Losing these loses the operator's choices; tiles are regenerable downloads.
        let path = Settings::path();
        let shown = path.display().to_string();
        assert!(shown.ends_with("settings.conf"), "{shown}");
        assert!(!shown.contains("tiles"), "{shown}");
    }
}
