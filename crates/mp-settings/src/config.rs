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

//! Mission Planner's `config.xml`: the settings file, in a format both applications read.
//!
//! Ported from `ExtLibs/Utilities/Settings.cs` `Load()` (427-505) and `Save()` (507-550). The
//! format is as plain as a settings file gets - a `Config` root and one element per key, the
//! element's text being the value - and [`Config::render`] writes it byte for byte as
//! `XmlTextWriter` does under mono, so a file this crate writes is one the C# reads back unchanged
//! and vice versa (DELIVERABLES.md D17) - which is what lets [`crate::migrate`] import the C#'s
//! file into this application's own directory by copying it. Measured, not guessed:
//! `tests/fixtures/config-saved.xml` is what Mission Planner's own `Settings.Save` wrote, under
//! mono, for the keys in `config-saved.txt` (`tests/fixtures/SettingsOracle.cs` drives it), and
//! the tests render the same keys and compare.
//!
//! What the writer does that is easy to get wrong:
//!
//! - The keys are sorted by `OrderBy(a => a)`, the culture comparer: `.` `/` `_` before digits
//!   before letters, letters without regard to case, and only then lower case before upper.
//! - A value that is empty is an empty element, `<key />`.
//! - A key with `/` in it is renamed `____` - and then looked up under the new name, which is not
//!   the name it is held under, so the lookup throws, the `catch` swallows it and the key is not
//!   written at all. Settings set in memory with a slash in their name never reach the file; a
//!   serial port's `/dev/ttyACM0_BAUD` is one. `Load` does not undo the renaming either, so a
//!   `____` element read from the file stays `____` in memory and is written back as it came.
//! - A UTF-8 byte-order mark, two-space indentation, `&`, `<` and `>` escaped and nothing else but
//!   control characters, and no newline after `</Config>`.
//!
//! When the C# saves - its `SaveConfig`, on start-up, on the FLIGHT DATA and FLIGHT PLAN buttons,
//! after Connect and on closing - is the caller's; this is the file.

use std::collections::BTreeMap;
use std::ffi::OsString;
use std::path::{Path, PathBuf};

/// The file's name, `Settings.FileName`.
/// `// C#: ExtLibs/Utilities/Settings.cs:47`
pub const FILE_NAME: &str = "config.xml";

/// The environment variable that puts `config.xml` somewhere else.
///
/// Not the C#'s. The C# writes this file on start-up and on every screen change, and so does
/// this application; a test that did not name a file of its own would rewrite the settings of the
/// Mission Planner installed on the machine running it. `tools/gui-test.sh` points it at a copy.
pub const PATH_VARIABLE: &str = "MP_CONFIG_XML";

/// What `/` becomes in an element name.
/// `// C#: ExtLibs/Utilities/Settings.cs:524-525`
const SLASH: &str = "____";

/// Why a config file could not be read.
#[derive(Debug, thiserror::Error)]
pub enum ConfigError {
    /// The file could not be read.
    #[error("reading {path}: {source}")]
    Io {
        /// The file.
        path: PathBuf,
        /// The underlying error.
        source: std::io::Error,
    },
    /// The file is not the XML the C# writes.
    #[error("config.xml is not well-formed: {0}")]
    Xml(String),
    /// The root element is not `Config`.
    #[error("config.xml's root is <{0}>, not <Config>")]
    NotAConfig(String),
}

/// A loaded `config.xml`: `Settings.config`, the dictionary every `Settings.Instance[key]` reads
/// and writes.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Config {
    entries: BTreeMap<String, String>,
}

impl Config {
    /// Where the C# keeps it: the user data directory plus `config.xml` - or wherever
    /// [`PATH_VARIABLE`] says.
    ///
    /// The C# also migrates a `config.xml` found beside the executable into that directory on
    /// first run (`GetConfigFullPath`); there is no executable-side file here to migrate.
    /// `// C#: ExtLibs/Utilities/Settings.cs:370-411`
    #[must_use]
    pub fn default_path() -> Option<PathBuf> {
        path_from(
            std::env::var_os(PATH_VARIABLE),
            crate::user_data_directory(),
        )
    }

    /// Where the C# application keeps its own `config.xml`: its user data directory, under its
    /// name. [`crate::migrate`] imports it from there once; nothing here writes it.
    /// `// C#: ExtLibs/Utilities/Settings.cs:370-411`
    #[must_use]
    pub fn csharp_path() -> Option<PathBuf> {
        crate::Folders::from_environment()
            .map(|folders| folders.csharp_user_data_directory().join(FILE_NAME))
    }

    /// Reads a file.
    pub fn load(path: &Path) -> Result<Self, ConfigError> {
        let text = std::fs::read_to_string(path).map_err(|source| ConfigError::Io {
            path: path.to_path_buf(),
            source,
        })?;
        Self::parse(&text)
    }

    /// Parses the file's text.
    ///
    /// Every element under the root is a key, named as the element is: the main file's `Load`
    /// keeps `____` as it is (only the `custom.config.xml` defaults are renamed back to `/`). The
    /// root and the XML declaration are skipped, as the C#'s `switch (xmlreader.Name)` skips them.
    /// A malformed entry does not fail the file in the C# - it is caught and ignored - but there is
    /// no such thing as a malformed element once the document parses, so the only failures here
    /// are a document that does not parse and a root that is not `Config`.
    /// `// C#: ExtLibs/Utilities/Settings.cs:468-500`
    pub fn parse(text: &str) -> Result<Self, ConfigError> {
        let text = text.strip_prefix('\u{feff}').unwrap_or(text);
        let document =
            roxmltree::Document::parse(text).map_err(|err| ConfigError::Xml(err.to_string()))?;
        let root = document.root_element();
        if root.tag_name().name() != "Config" {
            return Err(ConfigError::NotAConfig(root.tag_name().name().to_owned()));
        }
        let mut entries = BTreeMap::new();
        for element in root.children().filter(roxmltree::Node::is_element) {
            let key = element.tag_name().name().to_owned();
            let value: String = element.children().filter_map(|node| node.text()).collect();
            entries.insert(key, value);
        }
        Ok(Self { entries })
    }

    /// One setting: `Settings.Instance[key]`.
    /// `// C#: ExtLibs/Utilities/Settings.cs:49-56`
    #[must_use]
    pub fn get(&self, key: &str) -> Option<&str> {
        self.entries.get(key).map(String::as_str)
    }

    /// Sets one setting: `Settings.Instance[key] = value`, in memory until the file is saved.
    /// `// C#: ExtLibs/Utilities/Settings.cs:58-61`
    pub fn set(&mut self, key: impl Into<String>, value: impl Into<String>) {
        self.entries.insert(key.into(), value.into());
    }

    /// How many settings there are.
    #[must_use]
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    /// Whether there are none.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// Every key, in the order `Save` writes them: `config.Keys.OrderBy(a => a)`.
    /// `// C#: ExtLibs/Utilities/Settings.cs:519`
    #[must_use]
    pub fn keys(&self) -> Vec<&str> {
        let mut keys: Vec<&str> = self.entries.keys().map(String::as_str).collect();
        keys.sort_by_cached_key(|key| culture_key(key));
        keys
    }

    /// The file's text, as `Settings.Save` writes it.
    ///
    /// Each key in [`Config::keys`]'s order, `/` spelled `____`; a key the C# refuses (empty, or
    /// holding a space, `-`, `:`, `;`, `@`, `!`, `#`, `$` or `%`) left out, and so is one whose
    /// `____` spelling is not itself a key, because `config[key]` is read with the renamed key.
    /// `// C#: ExtLibs/Utilities/Settings.cs:507-550`
    #[must_use]
    pub fn render(&self) -> String {
        let mut out = String::from("\u{feff}<?xml version=\"1.0\" encoding=\"utf-8\"?>\n<Config>");
        for key in self.keys() {
            let element = key.replace('/', SLASH);
            if element.is_empty()
                || element
                    .chars()
                    .any(|c| matches!(c, ' ' | '-' | ':' | ';' | '@' | '!' | '#' | '$' | '%'))
            {
                continue;
            }
            // `xmlwriter.WriteElementString(key, "" + config[key])`, `key` being the renamed one.
            let Some(value) = self.entries.get(&element) else {
                continue;
            };
            out.push_str("\n  <");
            out.push_str(&element);
            // `WriteElementString` writes no text for "", and `WriteEndElement` then closes the
            // element as an empty one.
            if value.is_empty() {
                out.push_str(" />");
                continue;
            }
            out.push('>');
            push_escaped(&mut out, value);
            out.push_str("</");
            out.push_str(&element);
            out.push('>');
        }
        out.push_str("\n</Config>");
        out
    }

    /// Writes the file: `Settings.Save`, the whole dictionary every time.
    ///
    /// The directory is made if it is missing, as `GetConfigFullPath` makes it. Written through a
    /// temporary file and a rename, so a crash mid-write leaves the old settings rather than half
    /// a file; the bytes that land are [`Config::render`]'s either way.
    /// `// C#: ExtLibs/Utilities/Settings.cs:382-387, 507-550`
    pub fn save(&self, path: &Path) -> Result<(), ConfigError> {
        let io = |source| ConfigError::Io {
            path: path.to_path_buf(),
            source,
        };
        if let Some(directory) = path.parent().filter(|dir| !dir.as_os_str().is_empty()) {
            std::fs::create_dir_all(directory).map_err(io)?;
        }
        let temporary = path.with_extension("xml.tmp");
        std::fs::write(&temporary, self.render()).map_err(io)?;
        std::fs::rename(&temporary, path).map_err(io)
    }

    /// The link the C# would open, as this application's URL, if the file names one.
    ///
    /// `comport` is a device path or one of the words the connection box lists; an empty one is
    /// no link, as `MainV2` leaves the box alone for it. The baud rate is under `<comport>_BAUD`
    /// (which `Save` never writes for a device path with a `/` in it); TCP's host and port are
    /// under `TCP_host` and `TCP_port`, UDP's port under `UDP_port`, with the C#'s defaults.
    /// `UDPCl` is `MainV2`'s `UdpSerialConnect`, which sends to `UDP_host` and `UDP_port`
    /// (defaults 127.0.0.1 and 14550; an empty setting is its default, as `CommsBase.OnSettings`
    /// takes an empty answer); `WS` is its `WebSocket`, whose URL is `WS_url` as typed, and with
    /// none there is nothing to open. `AUTO`, a scan of the serial ports, is not a link.
    /// `// C#: ExtLibs/Utilities/Settings.cs:88-125; MainV2.cs:782-808, 1295-1301, 1481-1488;
    /// ExtLibs/Comms/CommsTCPSerial.cs:35, 114-121; ExtLibs/Comms/CommsUdpSerial.cs:44, 110-112;
    /// ExtLibs/Comms/CommsUDPSerialConnect.cs:31-35, 133-138; ExtLibs/Comms/CommsWebSocket.cs:103;
    /// ExtLibs/Comms/CommsBase.cs:41-55; Program.cs:661-675`
    #[must_use]
    pub fn last_link(&self) -> Option<String> {
        let port = self.get("comport").filter(|port| !port.is_empty())?;
        match port {
            "TCP" => Some(format!(
                "tcp:{}:{}",
                self.get("TCP_host").unwrap_or("127.0.0.1"),
                self.get("TCP_port").unwrap_or("5760")
            )),
            "UDP" => Some(format!("udp:{}", self.get("UDP_port").unwrap_or("14550"))),
            "UDPCl" => {
                let setting = |name: &str, default: &'static str| {
                    self.get(name)
                        .filter(|value| !value.is_empty())
                        .unwrap_or(default)
                };
                Some(format!(
                    "udpcl:{}:{}",
                    setting("UDP_host", "127.0.0.1"),
                    setting("UDP_port", "14550")
                ))
            }
            "WS" => self
                .get("WS_url")
                .filter(|url| !url.is_empty())
                .map(ToOwned::to_owned),
            "AUTO" => None,
            device => Some(match self.get(&format!("{device}_BAUD")) {
                Some(baud) if !baud.is_empty() => format!("serial:{device}:{baud}"),
                _ => format!("serial:{device}"),
            }),
        }
    }

    /// The map provider the C# last showed, by its provider `Name`.
    ///
    /// Written whenever the map type is changed, as the box's text - which is the `Name`. Absent
    /// until the operator first changes it; the C# then shows `GoogleSatelliteMap`, which is
    /// `mp_tiles::source::DEFAULT_PROVIDER` on this side.
    /// `// C#: GCSViews/FlightPlanner.cs:2237, 7247-7295`
    #[must_use]
    pub fn map_type(&self) -> Option<&str> {
        self.get("MapType")
    }

    /// Where the C# records flights, if the user chose somewhere.
    #[must_use]
    pub fn log_directory(&self) -> Option<PathBuf> {
        self.get("logdirectory")
            .filter(|dir| !dir.is_empty())
            .map(PathBuf::from)
    }
}

/// [`Config::default_path`] with its inputs passed in: the variable's value if it names anything,
/// else `config.xml` in the user data directory.
fn path_from(explicit: Option<OsString>, user_data: Option<PathBuf>) -> Option<PathBuf> {
    match explicit {
        Some(path) if !path.is_empty() => Some(PathBuf::from(path)),
        _ => user_data.map(|directory| directory.join(FILE_NAME)),
    }
}

/// The punctuation a key can hold, in the order mono's `en-US` comparer puts it - measured with
/// `OrderBy` over one key per character. All of it sorts before the digits, and the digits before
/// the letters.
const PUNCTUATION: &str = "'- !\"#$%&()*,./:;?@[\\]^_`{|}~+<=>";

/// A key's place in `OrderBy(a => a)` under mono's `en-US` culture, as a value Rust can sort by.
///
/// The comparison has levels. First every character's primary weight, left to right - punctuation
/// in [`PUNCTUATION`]'s order, then digits, then letters with case ignored - with a key that runs
/// out first coming first. Only when those are equal throughout does case count, lower before
/// upper at the first difference: `kindex` before `Kindex`, but `ka1` before `kA2` before `Ka3`.
/// Letters outside ASCII are placed by their code point, which is not the culture's order and is
/// not needed: no key the C# writes has one.
fn culture_key(key: &str) -> (Vec<(u8, u32)>, Vec<bool>, String) {
    let primary = key
        .chars()
        .map(|c| {
            if let Some(at) = PUNCTUATION.find(c) {
                (0, u32::try_from(at).unwrap_or(u32::MAX))
            } else if c.is_ascii_digit() {
                (1, u32::from(c))
            } else if c.is_alphabetic() {
                (2, u32::from(c.to_lowercase().next().unwrap_or(c)))
            } else {
                (3, u32::from(c))
            }
        })
        .collect();
    let case = key.chars().map(char::is_uppercase).collect();
    (primary, case, key.to_owned())
}

/// Text escaping as mono's `XmlTextWriter.WriteString` does it, measured: `&`, `<` and `>` as
/// entities, a control character other than tab, line feed and carriage return as a character
/// reference, and everything else - quotes, `\r\n`, non-ASCII - as it is.
fn push_escaped(out: &mut String, value: &str) {
    for c in value.chars() {
        match c {
            '&' => out.push_str("&amp;"),
            '<' => out.push_str("&lt;"),
            '>' => out.push_str("&gt;"),
            '\t' | '\n' | '\r' => out.push(c),
            control if control.is_control() && u32::from(control) < 0x20 => {
                out.push_str(&format!("&#x{:X};", u32::from(control)));
            }
            other => out.push(other),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const FIXTURE: &str = include_str!("../tests/fixtures/config.xml");

    /// What Mission Planner's own `Settings.Save` wrote, under mono, for [`SAVED_INPUT`].
    const SAVED: &str = include_str!("../tests/fixtures/config-saved.xml");

    /// The `key=value` lines `SettingsOracle.cs` set before saving, values in C# escapes.
    const SAVED_INPUT: &str = include_str!("../tests/fixtures/config-saved.txt");

    /// `Regex.Unescape` for the escapes the input uses: `\n`, `\r`, `\t` and `\xHH`.
    fn unescape(value: &str) -> String {
        let mut out = String::new();
        let mut chars = value.chars();
        while let Some(c) = chars.next() {
            if c != '\\' {
                out.push(c);
                continue;
            }
            match chars.next() {
                Some('n') => out.push('\n'),
                Some('r') => out.push('\r'),
                Some('t') => out.push('\t'),
                Some('x') => {
                    let hex: String = chars.by_ref().take(2).collect();
                    let code = u32::from_str_radix(&hex, 16).expect("two hex digits");
                    out.push(char::from_u32(code).expect("a character"));
                }
                Some(other) => out.push(other),
                None => out.push('\\'),
            }
        }
        out
    }

    /// The dictionary `SettingsOracle.cs` built, built here the same way.
    fn saved_input() -> Config {
        let mut config = Config::default();
        for line in SAVED_INPUT.lines() {
            if line.is_empty() || line.starts_with('#') {
                continue;
            }
            let (key, value) = line.split_once('=').expect("key=value");
            config.set(key, unescape(value));
        }
        config
    }

    #[test]
    fn rendering_is_what_mission_planners_own_save_wrote() {
        // The oracle: MissionPlanner.Utilities.dll's Settings class, given these keys under mono,
        // wrote SAVED. Every rule in the module comment is in it - the sort, `<fpminaltwarning />`,
        // `/dev/ttyACM0_BAUD` left out, the slashed twin of a `____` key writing that key's value
        // a second time, quotes unescaped, a newline and a trailing space kept.
        let config = saved_input();
        assert_eq!(config.render(), SAVED);
    }

    #[test]
    fn what_mission_planner_saved_reads_back_and_saves_again_unchanged() {
        let config = Config::parse(SAVED).expect("the oracle's file parses");
        assert_eq!(config.get("fpminaltwarning"), Some(""));
        assert_eq!(config.get("multi"), Some("line one\nline two"));
        assert_eq!(config.get("sp"), Some(" x "));
        assert_eq!(config.get("uni"), Some("é谷"));
        assert_eq!(
            config.get("update_check"),
            Some("9/20/2026 <beta> & \"more\" 'quoted'")
        );
        assert_eq!(config.get("rawparam____panel1collapsed"), Some("True"));
        assert_eq!(config.get("rawparam/panel1collapsed"), None);
        assert_eq!(config.get("/dev/ttyACM0_BAUD"), None);
        // The twin was written twice with one value, and reads back as one key: the next save
        // writes it once.
        let again = config.render();
        assert_eq!(again.matches("<rawparam____panel1collapsed>").count(), 1);
        assert_eq!(
            again.replacen(
                "\n  <rawparam____panel1collapsed>True</rawparam____panel1collapsed>",
                "",
                1
            ),
            SAVED.replacen(
                "\n  <rawparam____panel1collapsed>True</rawparam____panel1collapsed>",
                "",
                2
            ),
        );
    }

    #[test]
    fn a_file_the_csharp_wrote_reads_back_key_by_key() {
        let config = Config::parse(FIXTURE).expect("the fixture parses");
        assert_eq!(config.get("MapType"), Some("GoogleSatelliteMap"));
        assert_eq!(config.get("comport"), Some("TCP"));
        assert_eq!(config.get("TCP_BAUD"), Some("115200"));
        assert_eq!(
            config.get("logdirectory"),
            Some("/home/pilot/.local/share/Mission Planner/logs")
        );
        // A multi-line value comes back whole, untrimmed.
        let auto = config.get("AutoConnect").expect("AutoConnect");
        assert!(auto.starts_with("[\n"), "{auto:?}");
        assert!(auto.contains("\"Port\": 14550"));
        // Escaped text is unescaped.
        assert_eq!(config.get("update_check"), Some("9/20/2026 <beta> & more"));
        // `Load` keeps the element's name: `____` stays `____`.
        assert_eq!(config.get("rawparam____panel1collapsed"), Some("True"));
        assert_eq!(config.get("rawparam/panel1collapsed"), None);
    }

    #[test]
    fn rendering_reproduces_the_csharp_bytes() {
        // The fixture was written by XmlTextWriter; parse and render must give it back exactly,
        // BOM, order, escaping and missing final newline included.
        let config = Config::parse(FIXTURE).expect("parses");
        assert_eq!(config.render(), FIXTURE);
    }

    #[test]
    fn keys_sort_as_monos_culture_comparer_does() {
        // Measured: `OrderBy(a => a)` under mono's en-US culture over these keys.
        let measured = [
            "FP_docking",
            "FPaltmode",
            "fpcoordmouse",
            "k",
            "K",
            "k.",
            "k.1",
            "k.a",
            "k/",
            "k/a",
            "k_",
            "k____a",
            "k__a",
            "k_1",
            "k_a",
            "k_a_b",
            "k_b",
            "k0",
            "k1",
            "k1a",
            "k9",
            "ka",
            "kA",
            "ka1",
            "kA2",
            "Ka3",
            "kab",
            "kb",
            "kB",
            "UDP_BAUD",
            "UDP_host",
            "UDPCl_BAUD",
        ];
        let mut config = Config::default();
        for key in measured.iter().rev() {
            config.set(*key, "1");
        }
        assert_eq!(config.keys(), measured);
    }

    #[test]
    fn a_key_the_csharp_would_refuse_is_not_written() {
        let mut config = Config::default();
        config.set("good", "1");
        config.set("bad key", "2");
        config.set("bad-key", "3");
        config.set("", "4");
        let rendered = config.render();
        assert!(rendered.contains("<good>1</good>"));
        assert!(!rendered.contains("bad"));
        assert!(!rendered.contains("<>4"));
    }

    #[test]
    fn a_key_with_a_slash_is_lost_unless_its_twin_is_held() {
        // A serial port's baud rate is `<comport>_BAUD`, and a Linux device path has slashes: the
        // C# looks the value up under the renamed key, finds nothing, and writes nothing.
        let mut config = Config::default();
        config.set("comport", "/dev/ttyACM0");
        config.set("/dev/ttyACM0_BAUD", "57600");
        let rendered = config.render();
        assert!(!rendered.contains("57600"), "{rendered}");
        assert!(!rendered.contains("____dev"), "{rendered}");
        // A Windows port name has none, and is written.
        config.set("COM3_BAUD", "57600");
        assert!(config.render().contains("<COM3_BAUD>57600</COM3_BAUD>"));
    }

    #[test]
    fn an_empty_value_is_an_empty_element() {
        let mut config = Config::default();
        config.set("TXT_homelat", "");
        assert!(config.render().ends_with("\n  <TXT_homelat />\n</Config>"));
        let back = Config::parse(&config.render()).expect("parses");
        assert_eq!(back.get("TXT_homelat"), Some(""));
    }

    #[test]
    fn a_control_character_is_a_character_reference() {
        // Measured: "a\x01b" is written `a&#x1;b`, and "a\r\nb" and "a\tb" as they are.
        let mut config = Config::default();
        config.set("ctl", "a\u{1}b");
        config.set("crlf", "a\r\nb");
        config.set("tab", "a\tb");
        let rendered = config.render();
        assert!(rendered.contains("<ctl>a&#x1;b</ctl>"), "{rendered}");
        assert!(rendered.contains("<crlf>a\r\nb</crlf>"), "{rendered}");
        assert!(rendered.contains("<tab>a\tb</tab>"), "{rendered}");
    }

    #[test]
    fn the_root_must_be_config() {
        assert!(matches!(
            Config::parse("<Settings><a>1</a></Settings>"),
            Err(ConfigError::NotAConfig(name)) if name == "Settings"
        ));
        assert!(matches!(Config::parse("not xml"), Err(ConfigError::Xml(_))));
    }

    #[test]
    fn the_last_link_follows_comport_and_its_companions() {
        let mut config = Config::default();
        config.set("comport", "TCP");
        config.set("TCP_host", "10.0.0.5");
        config.set("TCP_port", "5762");
        assert_eq!(config.last_link().as_deref(), Some("tcp:10.0.0.5:5762"));
        config.set("comport", "UDP");
        config.set("UDP_port", "14551");
        assert_eq!(config.last_link().as_deref(), Some("udp:14551"));
        config.set("comport", "COM3");
        config.set("COM3_BAUD", "57600");
        assert_eq!(config.last_link().as_deref(), Some("serial:COM3:57600"));
        // A device path's baud rate is never saved, so what comes back from the file is the port.
        config.set("comport", "/dev/ttyUSB0");
        config.set("/dev/ttyUSB0_BAUD", "57600");
        let back = Config::parse(&config.render()).expect("parses");
        assert_eq!(back.last_link().as_deref(), Some("serial:/dev/ttyUSB0"));
        assert_eq!(Config::default().last_link(), None);
    }

    #[test]
    fn a_comport_this_application_cannot_open_is_no_link() {
        // "" is what SaveConfig writes before anything was chosen, and MainV2 skips it; AUTO is a
        // scan of the serial ports, not a link; WS with no URL saved has nothing to open.
        let mut config = Config::default();
        for port in ["", "AUTO", "WS"] {
            config.set("comport", port);
            config.set("UDP_host", "192.168.2.1");
            assert_eq!(config.last_link(), None, "{port:?}");
        }
        config.set("WS_url", "");
        assert_eq!(config.last_link(), None);
    }

    #[test]
    fn the_udp_client_and_the_websocket_are_links_with_their_settings() {
        // MainV2.cs:1481-1488: UDPCl is a UdpSerialConnect, WS a WebSocket.
        let mut config = Config::default();
        config.set("comport", "UDPCl");
        // The C#'s defaults, with nothing saved (CommsUDPSerialConnect.cs:33, 134)...
        assert_eq!(config.last_link().as_deref(), Some("udpcl:127.0.0.1:14550"));
        // ...and with an empty value saved, which OnSettings takes as none (CommsBase.cs:50-51).
        config.set("UDP_host", "");
        config.set("UDP_port", "");
        assert_eq!(config.last_link().as_deref(), Some("udpcl:127.0.0.1:14550"));
        config.set("UDP_host", "192.168.4.1");
        config.set("UDP_port", "14551");
        assert_eq!(
            config.last_link().as_deref(),
            Some("udpcl:192.168.4.1:14551")
        );

        // WS_url as typed; it round-trips through the file.
        config.set("comport", "WS");
        config.set("WS_url", "ws://192.168.4.1:8080/mavlink");
        let back = Config::parse(&config.render()).expect("parses");
        assert_eq!(
            back.last_link().as_deref(),
            Some("ws://192.168.4.1:8080/mavlink")
        );
    }

    #[test]
    fn the_variable_names_the_file_when_it_names_anything() {
        let data = PathBuf::from("/home/pilot/.local/share/MissionPlannerRust");
        assert_eq!(
            path_from(None, Some(data.clone())),
            Some(data.join("config.xml"))
        );
        assert_eq!(
            path_from(Some(OsString::new()), Some(data.clone())),
            Some(data.join("config.xml"))
        );
        assert_eq!(
            path_from(Some(OsString::from("/tmp/run/config.xml")), Some(data)),
            Some(PathBuf::from("/tmp/run/config.xml"))
        );
        assert_eq!(path_from(None, None), None);
    }

    #[test]
    fn saving_makes_the_directory_and_round_trips() {
        let dir = std::env::temp_dir().join(format!("mp-settings-config-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        // GetConfigFullPath creates the user data directory; so does this.
        let path = dir.join("MissionPlannerRust").join("config.xml");
        let config = Config::parse(FIXTURE).expect("parses");
        config.save(&path).expect("save");
        let back = Config::load(&path).expect("load");
        assert_eq!(back, config);
        assert_eq!(std::fs::read_to_string(&path).expect("read"), FIXTURE);
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// The C#'s real file on this machine, when there is one: it reads, it says what it says, and
    /// it renders back to the same bytes.
    #[test]
    fn the_real_config_on_this_machine_round_trips() {
        let Some(path) = Config::csharp_path() else {
            eprintln!("skipped: no home directory");
            return;
        };
        let Ok(text) = std::fs::read_to_string(&path) else {
            eprintln!("skipped: no Mission Planner config at {}", path.display());
            return;
        };
        let config = Config::parse(&text).expect("the real config.xml parses");
        eprintln!(
            "{}: {} keys, MapType {:?}, link {:?}",
            path.display(),
            config.len(),
            config.map_type(),
            config.last_link()
        );
        assert!(config.len() > 10);
        assert_eq!(
            config.render(),
            text,
            "render must give back the C#'s bytes"
        );
    }

    /// The keys the ported screens write, each set in the real file and rendered: the element
    /// Mission Planner wrote for it is the one written here, a value set to what it already was
    /// changes no byte, and a new value changes that element and nothing else.
    #[test]
    fn each_persisted_key_round_trips_through_the_real_config() {
        let Some(path) = Config::csharp_path() else {
            eprintln!("skipped: no home directory");
            return;
        };
        let Ok(text) = std::fs::read_to_string(&path) else {
            eprintln!("skipped: no Mission Planner config at {}", path.display());
            return;
        };
        let real = Config::parse(&text).expect("the real config.xml parses");
        let keys = [
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
        ];
        let mut seen = 0;
        for key in keys {
            let mut config = real.clone();
            if let Some(value) = real.get(key) {
                seen += 1;
                // The C#'s own line for it is in the file, and is the line rendered for it.
                let line = if value.is_empty() {
                    format!("\n  <{key} />")
                } else {
                    let mut escaped = String::new();
                    push_escaped(&mut escaped, value);
                    format!("\n  <{key}>{escaped}</{key}>")
                };
                assert!(text.contains(&line), "{key}: {line:?} not in the real file");
                config.set(key, value);
                assert_eq!(config.render(), text, "{key} set to itself");
            }
            config.set(key, "changed & <new>");
            let rendered = config.render();
            let back = Config::parse(&rendered).expect("parses");
            assert_eq!(back.get(key), Some("changed & <new>"), "{key}");
            for other in real.keys() {
                if other != key {
                    assert_eq!(back.get(other), real.get(other), "{other} after {key}");
                }
            }
            assert_eq!(
                back.len(),
                real.len() + usize::from(real.get(key).is_none())
            );
        }
        eprintln!(
            "{}: {seen} of {} persisted keys present",
            path.display(),
            keys.len()
        );
    }
}
