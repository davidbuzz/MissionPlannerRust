//! Settings that survive a restart.
//!
//! Small and deliberately dumb: a flat list of `key = value` lines. A ground station's settings
//! are a handful of strings and numbers, and a serialisation format would be a dependency, a
//! schema and a migration story for something a person can read and fix in a text editor when it
//! goes wrong in a field.
//!
//! Nothing here is allowed to stop the application starting. A settings file that is missing,
//! unreadable, half-written or full of rubbish yields defaults, because the alternative is a
//! ground station that will not open on the day its disk filled up.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::collections::BTreeMap;
use std::path::PathBuf;

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
