//! Mission Planner's `config.xml`: the settings file both applications can share.
//!
//! Ported from `ExtLibs/Utilities/Settings.cs` `Load()` (427-505) and `Save()` (507-540). The
//! format is as plain as a settings file gets - a `Config` root and one element per key, the
//! element's text being the value - with two rules that are easy to get wrong. A key with a `/`
//! in it is written as `____`, because `/` is not a legal element name. And the file is written
//! with the keys sorted, case-insensitively, with a UTF-8 byte-order mark and no newline after
//! the root's closing tag, which is what `XmlTextWriter` produces; [`Config::render`] matches
//! that byte for byte, so a file this crate writes is one the C# reads back unchanged and vice
//! versa - the promise DELIVERABLES.md D17 makes about running both applications on one data
//! directory.
//!
//! Values are kept exactly as stored: the C# reads them with `ReadString`, which does not trim,
//! and some are multi-line JSON.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

/// The file's name, `Settings.FileName`.
/// `// C#: ExtLibs/Utilities/Settings.cs:47`
pub const FILE_NAME: &str = "config.xml";

/// What `/` becomes in an element name.
/// `// C#: ExtLibs/Utilities/Settings.cs:484, 520`
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

/// A loaded `config.xml`.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Config {
    entries: BTreeMap<String, String>,
}

impl Config {
    /// Where the C# keeps it: the user data directory plus `config.xml`.
    ///
    /// The C# also migrates a `config.xml` found beside the executable into that directory on
    /// first run (`GetConfigFullPath`); there is no executable-side file here to migrate.
    /// `// C#: ExtLibs/Utilities/Settings.cs:370-403`
    #[must_use]
    pub fn default_path() -> Option<PathBuf> {
        crate::user_data_directory().map(|directory| directory.join(FILE_NAME))
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
    /// Every element under the root is a key; the root and the XML declaration are skipped, as
    /// the C#'s `switch (xmlreader.Name)` skips them. A malformed entry does not fail the file
    /// in the C# - it is caught and ignored - but there is no such thing as a malformed element
    /// once the document parses, so the only failures here are a document that does not parse
    /// and a root that is not `Config`.
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
            let key = element.tag_name().name().replace(SLASH, "/");
            let value: String = element.children().filter_map(|node| node.text()).collect();
            entries.insert(key, value);
        }
        Ok(Self { entries })
    }

    /// One setting.
    #[must_use]
    pub fn get(&self, key: &str) -> Option<&str> {
        self.entries.get(key).map(String::as_str)
    }

    /// Sets one setting.
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

    /// Every key, sorted as the file is.
    #[must_use]
    pub fn keys(&self) -> Vec<&str> {
        let mut keys: Vec<&str> = self.entries.keys().map(String::as_str).collect();
        keys.sort_by_cached_key(|key| (key.to_lowercase(), (*key).to_owned()));
        keys
    }

    /// The file's text, as `Settings.Save` writes it.
    ///
    /// A UTF-8 byte-order mark, the declaration, `Config`, one indented element per key with the
    /// keys sorted case-insensitively - `OrderBy(a => a)` uses the culture comparer, which is
    /// what puts `kindex` before `MainHeight` in a real file - and no newline after the closing
    /// tag. A key the C# would refuse to write (empty, or containing a space, `-`, `:`, `;`,
    /// `@`, `!`, `#`, `$` or `%`) is left out here too.
    /// `// C#: ExtLibs/Utilities/Settings.cs:507-540`
    #[must_use]
    pub fn render(&self) -> String {
        let mut out = String::from("\u{feff}<?xml version=\"1.0\" encoding=\"utf-8\"?>\n<Config>");
        for key in self.keys() {
            let Some(value) = self.entries.get(key) else {
                continue;
            };
            let element = key.replace('/', SLASH);
            if element.is_empty()
                || element
                    .chars()
                    .any(|c| matches!(c, ' ' | '-' | ':' | ';' | '@' | '!' | '#' | '$' | '%'))
            {
                continue;
            }
            out.push_str("\n  <");
            out.push_str(&element);
            out.push('>');
            push_escaped(&mut out, value);
            out.push_str("</");
            out.push_str(&element);
            out.push('>');
        }
        out.push_str("\n</Config>");
        out
    }

    /// Writes the file, through a temporary file and a rename so a crash mid-write leaves the
    /// old settings rather than half a file.
    pub fn save(&self, path: &Path) -> Result<(), ConfigError> {
        let temporary = path.with_extension("xml.tmp");
        let io = |source| ConfigError::Io {
            path: path.to_path_buf(),
            source,
        };
        std::fs::write(&temporary, self.render()).map_err(io)?;
        std::fs::rename(&temporary, path).map_err(io)
    }

    /// The link the C# would open, as this application's URL, if the file names one.
    ///
    /// `comport` is a device path or one of the words `TCP`, `UDP` and `UDPCl`; the baud rate is
    /// under `<comport>_BAUD`; TCP and UDP hosts and ports are under their own keys.
    /// `// C#: ExtLibs/Utilities/Settings.cs:88-125; MainV2.cs:2211-2227`
    #[must_use]
    pub fn last_link(&self) -> Option<String> {
        let port = self.get("comport")?;
        Some(match port {
            "TCP" => format!(
                "tcp:{}:{}",
                self.get("TCP_host").unwrap_or("127.0.0.1"),
                self.get("TCP_port").unwrap_or("5760")
            ),
            "UDP" => format!("udp:{}", self.get("UDP_port").unwrap_or("14550")),
            "UDPCl" => format!(
                "udp:{}:{}",
                self.get("UDPCl_host").unwrap_or("127.0.0.1"),
                self.get("UDPCl_port").unwrap_or("14550")
            ),
            device => match self.get(&format!("{device}_BAUD")) {
                Some(baud) => format!("serial:{device}:{baud}"),
                None => format!("serial:{device}"),
            },
        })
    }

    /// The map provider the C# last showed, by its provider `Name`.
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

/// XML text escaping as `XmlTextWriter.WriteElementString` does it.
fn push_escaped(out: &mut String, value: &str) {
    for c in value.chars() {
        match c {
            '&' => out.push_str("&amp;"),
            '<' => out.push_str("&lt;"),
            '>' => out.push_str("&gt;"),
            other => out.push(other),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const FIXTURE: &str = include_str!("../tests/fixtures/config.xml");

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
        // A slash in a key survives the ____ spelling.
        assert_eq!(config.get("rawparam/panel1collapsed"), Some("True"));
    }

    #[test]
    fn rendering_reproduces_the_csharp_bytes() {
        // The fixture was written by XmlTextWriter; parse and render must give it back exactly,
        // BOM, order, escaping and missing final newline included.
        let config = Config::parse(FIXTURE).expect("parses");
        assert_eq!(config.render(), FIXTURE);
    }

    #[test]
    fn keys_sort_case_insensitively_as_the_culture_comparer_does() {
        let mut config = Config::default();
        for key in [
            "MainHeight",
            "kindex",
            "AUTO_BAUD",
            "AutoConnect",
            "APMFirmware",
        ] {
            config.set(key, "1");
        }
        assert_eq!(
            config.keys(),
            vec![
                "APMFirmware",
                "AUTO_BAUD",
                "AutoConnect",
                "kindex",
                "MainHeight"
            ]
        );
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
        config.set("comport", "/dev/ttyUSB0");
        config.set("/dev/ttyUSB0_BAUD", "57600");
        assert_eq!(
            config.last_link().as_deref(),
            Some("serial:/dev/ttyUSB0:57600")
        );
        assert_eq!(Config::default().last_link(), None);
    }

    #[test]
    fn saving_and_loading_round_trips() {
        let dir = std::env::temp_dir().join(format!("mp-settings-config-{}", std::process::id()));
        std::fs::create_dir_all(&dir).expect("temp dir");
        let path = dir.join("config.xml");
        let config = Config::parse(FIXTURE).expect("parses");
        config.save(&path).expect("save");
        let back = Config::load(&path).expect("load");
        assert_eq!(back, config);
        assert_eq!(std::fs::read_to_string(&path).expect("read"), FIXTURE);
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// The real file on this machine, when there is one: it reads, it says what it says, and it
    /// renders back to the same bytes.
    #[test]
    fn the_real_config_on_this_machine_round_trips() {
        let Some(path) = Config::default_path() else {
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
}
