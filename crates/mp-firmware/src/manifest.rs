//! The firmware catalogue: ArduPilot's `manifest.json.gz` and the choices made from it.
//!
//! Ported from `ExtLibs/ArduPilot/APFirmware.cs` @ efb0801 (GPL-3.0-or-later), with the parts of
//! `GCSViews/ConfigurationView/ConfigFirmwareManifest.cs` (`LookForPort`) and
//! `test/FirmwareSelection.xaml.cs` that decide which firmware a board gets. Nothing here writes to
//! a board: this module answers "which file", never "put it there".
//!
//! # Where the manifest comes from, and how often
//!
//! `APFirmware.GetList(url, force)` downloads the gzipped manifest, parses it, and appends
//! CubePilot's peripheral manifest to it (`GetListAppend`). The result lives in a static,
//! `APFirmware.Manifest`, for the life of the process: `GetList` returns at once when it is already
//! set, and nothing in Mission Planner passes `force`. **There is no cache file.** Mission Planner
//! does not write the manifest to disk, so it downloads it again - about 1.8 MB - in every process
//! that needs it, and a failed download leaves nothing to fall back on. On this machine
//! `~/.local/share/Mission Planner` holds no manifest, and its `config.xml` holds only
//! `fw_check`, the date `MainV2.BGFirmwareCheck` last warmed that static (`MainV2.cs:3923-3938`).
//! [`get_list`] is that rule: an `Option<Manifest>` the caller keeps for as long as its process
//! lives, filled once.
//!
//! The URL is less obvious than `APFirmware`'s default argument suggests. The Install Firmware
//! page's `Activate` calls `GetList("https://firmware.oborne.me/manifest.json.gz")` first
//! (`ConfigFirmwareManifest.cs:67`) and `GetRelease` then calls `GetList()` with the default,
//! `https://firmware.ardupilot.org/manifest.json.gz` (`APFirmware.cs:103, 276`) - which does
//! nothing when the mirror answered. So the mirror is asked first and ardupilot.org is the
//! fallback: [`PAGE_SOURCES`], run by [`load`]. (The static constructor also queues a `GetList()`
//! of its own on first use, `APFirmware.cs:22-26`, but the page's call is already holding the lock
//! by then, and both find the manifest set.)
//!
//! # What is chosen
//!
//! - [`Manifest::release`] and [`Manifest::release_newest`] (`GetRelease`, `GetReleaseNewest`):
//!   for a release type, the newest version of each vehicle - what the page writes under each
//!   vehicle picture.
//! - [`Manifest::board_ids`] (`GetBoardID`): which board ids a USB device is, from its product
//!   string against each record's platform and bootloader strings, else from its VID/PID.
//! - [`Manifest::look_for_port`] (`LookForPort`): the first device that has board ids, and the
//!   records for those ids, that vehicle and that release. One record is taken without asking;
//!   several open `FirmwareSelection` ([`Selection`]), which pre-selects the device's own
//!   platform; none is "No firmware available for this board!".
//! - [`Manifest::options`] (`GetOptions`): the SITL screen's query, by release, vehicle and
//!   VID/PID.
//!
//! # Test scaffolding
//!
//! [`OVERRIDE_ENV`] names a manifest file to read instead of the network, so a test runs offline
//! against `testdata/firmware/`. It is not a Mission Planner setting, in the way `MP_TILE_CACHE`
//! is not: it exists to verify the port.

use std::io::Read;
use std::path::PathBuf;
use std::time::Duration;

use serde::de::{DeserializeSeed, Error as _, IgnoredAny, MapAccess, SeqAccess, Visitor};
use serde_json::Value;

use crate::detect::DeviceInfo;

/// `APFirmware.GetList`'s default URL.
/// `// C#: ExtLibs/ArduPilot/APFirmware.cs:103`
pub const MANIFEST_URL: &str = "https://firmware.ardupilot.org/manifest.json.gz";

/// The mirror the Install Firmware page and `MainV2`'s daily check ask first.
/// `// C#: GCSViews/ConfigurationView/ConfigFirmwareManifest.cs:67; MainV2.cs:3929`
pub const MIRROR_URL: &str = "https://firmware.oborne.me/manifest.json.gz";

/// CubePilot's peripheral manifest, appended after every successful `GetList`. Not gzipped.
/// `// C#: ExtLibs/ArduPilot/APFirmware.cs:131`
pub const PERIPH_URL: &str =
    "https://raw.githubusercontent.com/CubePilot/periph-manifest/main/manifest.json";

/// The order the Install Firmware page asks for the manifest: `GetList(mirror)` in `Activate`,
/// then `GetList()` inside `GetRelease`.
/// `// C#: GCSViews/ConfigurationView/ConfigFirmwareManifest.cs:67-69; ExtLibs/ArduPilot/APFirmware.cs:276`
pub const PAGE_SOURCES: [&str; 2] = [MIRROR_URL, MANIFEST_URL];

/// Test scaffolding: a manifest file (gzipped, as downloaded) to serve in place of
/// [`MANIFEST_URL`] and [`MIRROR_URL`]. Anything else, the peripheral manifest included, is then
/// not fetched at all.
pub const OVERRIDE_ENV: &str = "MP_FIRMWARE_MANIFEST";

/// How we identify ourselves. Mission Planner sends its product name, version and operating
/// system (`Settings.Instance.UserAgent`, set at `Program.cs:373-374`); this sends ours.
/// `// C#: ExtLibs/ArduPilot/APFirmware.cs:116-117`
pub const USER_AGENT: &str = concat!(
    "MissionPlannerRust/",
    env!("CARGO_PKG_VERSION"),
    " (+https://github.com/davidbuzz/MissionPlannerRust)"
);

/// `HttpClient.Timeout`'s default, which `GetList` leaves alone.
pub const TIMEOUT: Duration = Duration::from_secs(100);

/// The largest download accepted. The real manifest is 1.8 MB gzipped; this is room to grow, and
/// a bound on what a broken server can make us hold.
pub const MAX_DOWNLOAD: u64 = 256 << 20;

/// Why a manifest could not be had.
#[derive(Debug, thiserror::Error)]
pub enum ManifestError {
    /// The server did not give us the file.
    #[error("fetching {url}: {reason}")]
    Http {
        /// What was asked for.
        url: String,
        /// What went wrong.
        reason: String,
    },
    /// The download is not gzip.
    #[error("unpacking the manifest: {0}")]
    Gzip(std::io::Error),
    /// The JSON is malformed, or a field is not what `FirmwareInfo` declares.
    #[error("parsing the manifest: {0}")]
    Json(String),
    /// `GetListAppend` ran with no manifest to append to: a `NullReferenceException` in the C#.
    #[error("no manifest to append {0} to")]
    NothingToAppend(String),
}

/// A version as `System.Version` holds it: two to four numbers, an absent one being -1.
///
/// Ordered as `Version.CompareTo` orders: major, minor, build, revision, with an absent
/// component before any present one.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct Version {
    /// `Major`.
    pub major: i32,
    /// `Minor`.
    pub minor: i32,
    /// `Build`, -1 when absent.
    pub build: i32,
    /// `Revision`, -1 when absent.
    pub revision: i32,
}

impl Version {
    /// `Version.Parse`: two to four components separated by `.`, each a non-negative `Int32`.
    /// Each component is read as `int.Parse` reads it - surrounding white space and a leading
    /// `+` allowed. `None` where the C# throws.
    #[must_use]
    pub fn parse(text: &str) -> Option<Self> {
        let parts: Vec<&str> = text.split('.').collect();
        if !(2..=4).contains(&parts.len()) {
            return None;
        }
        let mut numbers = [-1_i32; 4];
        for (slot, part) in numbers.iter_mut().zip(&parts) {
            let part = part.trim();
            let digits = part.strip_prefix('+').unwrap_or(part);
            if digits.is_empty() || !digits.bytes().all(|b| b.is_ascii_digit()) {
                return None;
            }
            *slot = digits.parse().ok()?;
        }
        let [major, minor, build, revision] = numbers;
        Some(Self {
            major,
            minor,
            build,
            revision,
        })
    }
}

impl std::fmt::Display for Version {
    /// `Version.ToString()`: the components that are present.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}.{}", self.major, self.minor)?;
        if self.build >= 0 {
            write!(f, ".{}", self.build)?;
            if self.revision >= 0 {
                write!(f, ".{}", self.revision)?;
            }
        }
        Ok(())
    }
}

/// `APFirmware.RELEASE_TYPES`, which `generate_manifest.py` maps from its own names.
/// `// C#: ExtLibs/ArduPilot/APFirmware.cs:83-89`
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ReleaseType {
    /// `BETA`: the manifest's "beta".
    Beta,
    /// `DEV`: the manifest's "latest".
    Dev,
    /// `OFFICIAL`: the manifest's "stable".
    Official,
}

impl ReleaseType {
    /// Every value, in the C#'s declaration order.
    pub const ALL: [Self; 3] = [Self::Beta, Self::Dev, Self::Official];

    /// `ToString()`: the name, as the manifest's `mav-firmware-version-type` spells it.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::Beta => "BETA",
            Self::Dev => "DEV",
            Self::Official => "OFFICIAL",
        }
    }

    /// A value by its name, in any case, or by the `generate_manifest.py` name the C#'s comment
    /// maps it from (`beta`, `latest`, `stable`).
    #[must_use]
    pub fn from_name(name: &str) -> Option<Self> {
        let lower = name.to_ascii_lowercase();
        match lower.as_str() {
            "beta" => Some(Self::Beta),
            "dev" | "latest" => Some(Self::Dev),
            "official" | "stable" => Some(Self::Official),
            _ => None,
        }
    }
}

impl std::fmt::Display for ReleaseType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.name())
    }
}

/// `APFirmware.MAV_TYPE`: the vehicles the page offers, by the manifest's `mav-type` names.
/// `// C#: ExtLibs/ArduPilot/APFirmware.cs:91-99`
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum MavType {
    /// `ANTENNA_TRACKER`.
    AntennaTracker,
    /// `Copter` - spelled so in the enum and in the manifest.
    Copter,
    /// `HELICOPTER`.
    Helicopter,
    /// `FIXED_WING`.
    FixedWing,
    /// `GROUND_ROVER`.
    GroundRover,
    /// `SUBMARINE`.
    Submarine,
}

impl MavType {
    /// Every value, in the C#'s declaration order.
    pub const ALL: [Self; 6] = [
        Self::AntennaTracker,
        Self::Copter,
        Self::Helicopter,
        Self::FixedWing,
        Self::GroundRover,
        Self::Submarine,
    ];

    /// `ToString()`: the name, as the manifest's `mav-type` spells it.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::AntennaTracker => "ANTENNA_TRACKER",
            Self::Copter => "Copter",
            Self::Helicopter => "HELICOPTER",
            Self::FixedWing => "FIXED_WING",
            Self::GroundRover => "GROUND_ROVER",
            Self::Submarine => "SUBMARINE",
        }
    }

    /// A value by its name, in any case.
    #[must_use]
    pub fn from_name(name: &str) -> Option<Self> {
        Self::ALL
            .into_iter()
            .find(|mav_type| mav_type.name().eq_ignore_ascii_case(name))
    }
}

impl std::fmt::Display for MavType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.name())
    }
}

/// One record of the manifest: `APFirmware.FirmwareInfo`, field for field.
///
/// A C# string that can be null is an `Option`; the `long`s default to zero when the field is
/// absent or null, as `NullValueHandling.Ignore` leaves them; the arrays default to empty.
/// `// C#: ExtLibs/ArduPilot/APFirmware.cs:27-74`
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct FirmwareInfo {
    /// `board_id`: the id the bootloader reports. Zero for a board with none (Linux, SITL).
    pub board_id: i64,
    /// `mav-type`: a [`MavType`] name, or `CAN_PERIPHERAL`, `Blimp`, ...
    pub mav_type: Option<String>,
    /// `mav-firmware-version-minor`.
    pub mav_firmware_version_minor: i64,
    /// `format`: `apj`, `hex`, `elf`, `ELF`, `abin`, `bin`, `px4`, `zip`.
    pub format: Option<String>,
    /// `url`, a `Uri` in the C#.
    pub url: Option<String>,
    /// `mav-firmware-version-type`: a [`ReleaseType`] name, or `STABLE-4.5.7` and the like.
    pub mav_firmware_version_type: Option<String>,
    /// `mav-firmware-version-patch`.
    pub mav_firmware_version_patch: i64,
    /// `mav-autopilot`.
    pub mav_autopilot: Option<String>,
    /// `vehicletype`: `Copter`, `Plane`, `Rover`, `Sub`, `AntennaTracker`, `AP_Periph`, `Blimp`.
    pub vehicle_type: Option<String>,
    /// `USBID`: `0x2dae/0x1016` and the like.
    pub usbid: Vec<String>,
    /// `platform`: the board's hwdef name.
    pub platform: Option<String>,
    /// `mav-firmware-version`.
    pub mav_firmware_version: Option<Version>,
    /// `bootloader_str`: the USB product strings of the board's bootloader.
    pub bootloader_str: Vec<String>,
    /// `git-sha`.
    pub git_sha: Option<String>,
    /// `mav-firmware-version-major`.
    pub mav_firmware_version_major: i64,
    /// `mav-firmware-version-str`: `V4.7.1`, `V4.8.0-dev`.
    pub mav_firmware_version_str: Option<String>,
    /// `latest`.
    pub latest: i64,
    /// `firmware-name`: only the peripheral manifest has it.
    pub firmware_name: Option<String>,
}

/// The manifest: `APFirmware.ManifestRoot`.
/// `// C#: ExtLibs/ArduPilot/APFirmware.cs:76-81`
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Manifest {
    /// `firmware`, in the file's order - which `GetRelease` groups by.
    pub firmware: Vec<FirmwareInfo>,
    /// `format-version`.
    pub format_version: Option<Version>,
}

/// A field of a record, by `JsonProperty` name: exactly, else ignoring case, as Newtonsoft falls
/// back to.
fn field<'a>(record: &'a serde_json::Map<String, Value>, name: &str) -> Option<&'a Value> {
    record.get(name).or_else(|| {
        record
            .iter()
            .find(|(key, _)| key.eq_ignore_ascii_case(name))
            .map(|(_, value)| value)
    })
}

/// A `string` property: text as it is, a number or boolean as its text, null as null.
fn string_field(
    record: &serde_json::Map<String, Value>,
    name: &str,
) -> Result<Option<String>, String> {
    match field(record, name) {
        None | Some(Value::Null) => Ok(None),
        Some(Value::String(text)) => Ok(Some(text.clone())),
        Some(Value::Number(number)) => Ok(Some(number.to_string())),
        Some(Value::Bool(flag)) => Ok(Some(if *flag { "True" } else { "False" }.to_owned())),
        Some(other) => Err(format!("{name}: expected a string, found {other}")),
    }
}

/// A `long` property: a whole number, or text that is one, as `Convert.ChangeType` reads it.
///
/// Absent is zero. Null is zero where the property has `NullValueHandling.Ignore` and an error
/// otherwise, as Newtonsoft cannot put null in a `long`.
fn long_field(
    record: &serde_json::Map<String, Value>,
    name: &str,
    null_ignored: bool,
) -> Result<i64, String> {
    let bad = |value: &Value| format!("{name}: expected a whole number, found {value}");
    match field(record, name) {
        None => Ok(0),
        Some(Value::Null) if null_ignored => Ok(0),
        Some(Value::Number(number)) => number
            .as_i64()
            .or_else(|| number.as_f64().and_then(whole))
            .ok_or_else(|| bad(&Value::Number(number.clone()))),
        Some(Value::String(text)) => {
            let trimmed = text.trim();
            trimmed
                .strip_prefix('+')
                .unwrap_or(trimmed)
                .parse()
                .map_err(|_| bad(&Value::String(text.clone())))
        }
        Some(other) => Err(bad(other)),
    }
}

/// A JSON number with no fraction, inside the range of a `long`, as that `long`.
#[allow(clippy::cast_possible_truncation)] // checked: whole, and inside the range
fn whole(value: f64) -> Option<i64> {
    (value.fract() == 0.0 && value.abs() < 9.2e18).then_some(value as i64)
}

/// A `string[]` property with `NullValueHandling.Ignore`: absent or null leaves it empty.
///
/// A null element is dropped. The C# keeps it, and `GetBoardID` and `GetOptions` then call
/// `ToLower()` on it and throw; no manifest ArduPilot publishes has one.
fn strings_field(
    record: &serde_json::Map<String, Value>,
    name: &str,
) -> Result<Vec<String>, String> {
    match field(record, name) {
        None | Some(Value::Null) => Ok(Vec::new()),
        Some(Value::Array(items)) => items
            .iter()
            .filter(|item| !item.is_null())
            .map(|item| match item {
                Value::String(text) => Ok(text.clone()),
                Value::Number(number) => Ok(number.to_string()),
                other => Err(format!("{name}: expected strings, found {other}")),
            })
            .collect(),
        Some(other) => Err(format!("{name}: expected an array, found {other}")),
    }
}

/// A `Version` property, through Newtonsoft's `VersionConverter`: a string `Version.Parse`
/// accepts, or null.
fn version_field(
    record: &serde_json::Map<String, Value>,
    name: &str,
) -> Result<Option<Version>, String> {
    version_of(name, field(record, name))
}

/// A `Version` value, for [`version_field`] and the root's `format-version`.
fn version_of(name: &str, value: Option<&Value>) -> Result<Option<Version>, String> {
    match value {
        None | Some(Value::Null) => Ok(None),
        Some(Value::String(text)) => Version::parse(text)
            .map(Some)
            .ok_or_else(|| format!("{name}: {text:?} is not a version")),
        Some(other) => Err(format!("{name}: expected a version string, found {other}")),
    }
}

impl FirmwareInfo {
    /// One record, as `JsonConvert.DeserializeObject` fills it.
    ///
    /// # Errors
    /// Where Newtonsoft throws: a field of the wrong kind, a `long` that is not a whole number,
    /// a version `Version.Parse` refuses. One bad record fails the whole manifest in the C#.
    pub fn from_json(value: &Value) -> Result<Self, String> {
        let Value::Object(record) = value else {
            return Err(format!("expected a record, found {value}"));
        };
        Ok(Self {
            board_id: long_field(record, "board_id", true)?,
            mav_type: string_field(record, "mav-type")?,
            mav_firmware_version_minor: long_field(record, "mav-firmware-version-minor", true)?,
            format: string_field(record, "format")?,
            url: string_field(record, "url")?,
            mav_firmware_version_type: string_field(record, "mav-firmware-version-type")?,
            mav_firmware_version_patch: long_field(record, "mav-firmware-version-patch", true)?,
            mav_autopilot: string_field(record, "mav-autopilot")?,
            vehicle_type: string_field(record, "vehicletype")?,
            usbid: strings_field(record, "USBID")?,
            platform: string_field(record, "platform")?,
            mav_firmware_version: version_field(record, "mav-firmware-version")?,
            bootloader_str: strings_field(record, "bootloader_str")?,
            git_sha: string_field(record, "git-sha")?,
            mav_firmware_version_major: long_field(record, "mav-firmware-version-major", true)?,
            mav_firmware_version_str: string_field(record, "mav-firmware-version-str")?,
            latest: long_field(record, "latest", false)?,
            firmware_name: string_field(record, "firmware-name")?,
        })
    }

    /// Whether this record is for a vehicle and a release, as `LookForPort` and `GetOptions`
    /// compare them: the enums' names against the fields, exactly.
    #[must_use]
    pub fn is(&self, mav_type: MavType, release: ReleaseType) -> bool {
        self.mav_type.as_deref() == Some(mav_type.name())
            && self.mav_firmware_version_type.as_deref() == Some(release.name())
    }
}

/// The root, read record by record so the 77 MB manifest never exists as one JSON tree.
struct RootVisitor;

impl<'de> Visitor<'de> for RootVisitor {
    type Value = Manifest;

    fn expecting(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("a manifest object")
    }

    fn visit_map<A: MapAccess<'de>>(self, mut map: A) -> Result<Manifest, A::Error> {
        let mut firmware = None;
        let mut format_version = None;
        while let Some(key) = map.next_key::<String>()? {
            if key.eq_ignore_ascii_case("firmware") {
                firmware = map.next_value_seed(RecordsSeed)?;
            } else if key.eq_ignore_ascii_case("format-version") {
                let value: Value = map.next_value()?;
                format_version =
                    version_of("format-version", Some(&value)).map_err(A::Error::custom)?;
            } else {
                map.next_value::<IgnoredAny>()?;
            }
        }
        // C#: a manifest with no firmware list leaves `Firmware` null and every query throws.
        // Refused here instead, so it is never mistaken for an empty catalogue.
        let firmware = firmware.ok_or_else(|| A::Error::custom("no firmware list"))?;
        Ok(Manifest {
            firmware,
            format_version,
        })
    }
}

/// The `firmware` array: `None` for null.
struct RecordsSeed;

impl<'de> DeserializeSeed<'de> for RecordsSeed {
    type Value = Option<Vec<FirmwareInfo>>;

    fn deserialize<D: serde::Deserializer<'de>>(
        self,
        deserializer: D,
    ) -> Result<Self::Value, D::Error> {
        deserializer.deserialize_option(RecordsVisitor)
    }
}

struct RecordsVisitor;

impl<'de> Visitor<'de> for RecordsVisitor {
    type Value = Option<Vec<FirmwareInfo>>;

    fn expecting(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("an array of firmware records")
    }

    fn visit_none<E: serde::de::Error>(self) -> Result<Self::Value, E> {
        Ok(None)
    }

    fn visit_some<D: serde::Deserializer<'de>>(
        self,
        deserializer: D,
    ) -> Result<Self::Value, D::Error> {
        deserializer.deserialize_seq(self)
    }

    fn visit_seq<A: SeqAccess<'de>>(self, mut seq: A) -> Result<Self::Value, A::Error> {
        let mut records = Vec::with_capacity(seq.size_hint().unwrap_or(0));
        while let Some(value) = seq.next_element::<Value>()? {
            let index = records.len();
            let record = FirmwareInfo::from_json(&value)
                .map_err(|reason| A::Error::custom(format!("record {index}: {reason}")))?;
            records.push(record);
        }
        Ok(Some(records))
    }
}

impl Manifest {
    /// Parses the manifest's JSON, as `JsonConvert.DeserializeObject<ManifestRoot>` does after
    /// `StreamReader.ReadToEnd` has dropped a byte-order mark.
    ///
    /// # Errors
    /// If it is not JSON, has no firmware list, or a record is one Newtonsoft would refuse.
    pub fn parse(bytes: &[u8]) -> Result<Self, ManifestError> {
        let bytes = bytes.strip_prefix(b"\xEF\xBB\xBF").unwrap_or(bytes);
        let mut deserializer = serde_json::Deserializer::from_slice(bytes);
        let manifest = serde::Deserializer::deserialize_map(&mut deserializer, RootVisitor)
            .map_err(|err| ManifestError::Json(err.to_string()))?;
        deserializer
            .end()
            .map_err(|err| ManifestError::Json(err.to_string()))?;
        Ok(manifest)
    }

    /// Unpacks and parses a download: gzip always for the manifest (`GetList`), gzip only for a
    /// URL ending `.gz` for an appended one (`GetListAppend`).
    /// `// C#: ExtLibs/ArduPilot/APFirmware.cs:119-127, 153-169`
    ///
    /// # Errors
    /// If it does not unpack or does not parse.
    pub fn decode(bytes: &[u8], gzipped: bool) -> Result<Self, ManifestError> {
        if !gzipped {
            return Self::parse(bytes);
        }
        // Multi-member, as .NET's GZipStream reads concatenated members.
        let mut unpacked = Vec::new();
        flate2::read::MultiGzDecoder::new(bytes)
            .read_to_end(&mut unpacked)
            .map_err(ManifestError::Gzip)?;
        Self::parse(&unpacked)
    }

    /// `GetListAppend`'s merge: the other manifest's records after this one's.
    /// `// C#: ExtLibs/ArduPilot/APFirmware.cs:171-173`
    pub fn append(&mut self, other: Self) {
        self.firmware.extend(other.firmware);
    }

    /// `GetBoardID`: the board ids a device is, in the manifest's order, each once.
    ///
    /// First by name: a record matches when its platform is the device's product string, or the
    /// product string is the platform with `-BL` read as `primary` or `secondary` (a CubeRed's
    /// two processors), or one of its bootloader strings is the product string - all ignoring
    /// case. With `boardidcheck`, records without a board id do not count. Failing that, and
    /// only when the hardware id is set, by VID/PID: `VID_xxxx&PID_xxxx&` in the hardware id -
    /// the trailing `&` included, so a Windows id with `&REV_` matches and the `USB\VID_..&PID_..`
    /// that Mission Planner builds on Linux (`Linux.cs:40`), or [`DeviceInfo::from_port`] builds,
    /// does not - against each record's `USBID`s, among records with a board id.
    /// `// C#: ExtLibs/ArduPilot/APFirmware.cs:186-230`
    #[must_use]
    pub fn board_ids(&self, device: &DeviceInfo, boardidcheck: bool) -> Option<Vec<i64>> {
        let board = device.board.as_deref();
        let board_lower = board.map(str::to_lowercase);
        // `device.board?.Replace("-BL", ...).ToLower()`: the replace is case-sensitive.
        let as_secondary = board.map(|b| b.replace("-BL", "secondary").to_lowercase());
        let as_primary = board.map(|b| b.replace("-BL", "primary").to_lowercase());
        // `device.board?.ToLower() + "primary"`: null + "primary" is "primary" in C#.
        let plus_primary = format!("{}primary", board_lower.as_deref().unwrap_or(""));
        let plus_secondary = format!("{}secondary", board_lower.as_deref().unwrap_or(""));

        let by_name = self.firmware.iter().filter(|a| {
            let platform = a.platform.as_deref().map(str::to_lowercase);
            platform == board_lower
                || platform
                    .as_deref()
                    .map(|p| p.replace("primary", "secondary"))
                    == board_lower
                || platform == as_secondary
                || platform == as_primary
                || platform.as_deref() == Some(plus_primary.as_str())
                || platform.as_deref() == Some(plus_secondary.as_str())
                || a.bootloader_str
                    .iter()
                    .any(|b| Some(b.to_lowercase()) == board_lower)
        });
        let ids = distinct(
            by_name
                .filter(|a| !boardidcheck || a.board_id != 0)
                .map(|a| a.board_id),
        );
        if !ids.is_empty() {
            return Some(ids);
        }

        let lookfor = vid_pid(device.hardwareid.as_deref()?)?;
        let ids = distinct(
            self.firmware
                .iter()
                .filter(|a| a.board_id != 0 && has_usbid(a, &lookfor))
                .map(|a| a.board_id),
        );
        (!ids.is_empty()).then_some(ids)
    }

    /// `GetOptions`: the records for a release and a vehicle, narrowed to the device's VID/PID
    /// when its hardware id has one and any record carries it.
    ///
    /// The C# first filters by the device's platform and then discards that ("ignore platform"),
    /// so the board plays no part. `None` for a device with no hardware id, where the C#'s
    /// `Regex.IsMatch(null)` throws.
    /// `// C#: ExtLibs/ArduPilot/APFirmware.cs:232-272`
    #[must_use]
    pub fn options(
        &self,
        device: &DeviceInfo,
        reltype: Option<ReleaseType>,
        mav_type: Option<MavType>,
    ) -> Option<Vec<&FirmwareInfo>> {
        let hardwareid = device.hardwareid.as_deref()?;
        let ans: Vec<&FirmwareInfo> = self
            .firmware
            .iter()
            .filter(|a| {
                reltype.is_none_or(|r| a.mav_firmware_version_type.as_deref() == Some(r.name()))
            })
            .filter(|a| mav_type.is_none_or(|m| a.mav_type.as_deref() == Some(m.name())))
            .collect();
        let Some(lookfor) = vid_pid(hardwareid) else {
            return Some(ans);
        };
        let narrowed: Vec<&FirmwareInfo> = ans
            .iter()
            .copied()
            .filter(|a| has_usbid(a, &lookfor))
            .collect();
        Some(if narrowed.is_empty() { ans } else { narrowed })
    }

    /// `GetRelease`: for a release type, the records of each vehicle's newest version, the
    /// vehicles in the order they first appear and each vehicle's records ordered by format.
    ///
    /// The format order is `OrderBy`'s default comparer, the current culture's: letters by
    /// letter regardless of case, lower case first where two differ only in case - `abin`,
    /// `apj`, `bin`, `elf`, `ELF`, `hex` - and stable.
    /// `// C#: ExtLibs/ArduPilot/APFirmware.cs:274-290`
    #[must_use]
    pub fn release(&self, reltype: ReleaseType) -> Vec<&FirmwareInfo> {
        let mut groups: Vec<(Option<&str>, Vec<&FirmwareInfo>)> = Vec::new();
        for record in self
            .firmware
            .iter()
            .filter(|a| a.mav_firmware_version_type.as_deref() == Some(reltype.name()))
        {
            let key = record.mav_type.as_deref();
            match groups.iter_mut().find(|(group, _)| *group == key) {
                Some((_, members)) => members.push(record),
                None => groups.push((key, vec![record])),
            }
        }
        let mut out = Vec::new();
        for (_, members) in groups {
            // `Max` over a reference type skips nulls; `Option`'s order puts `None` first, so
            // `max` is the newest present version, or `None` when every one is null - and
            // `Version`'s `==` is null-safe, as `Option`'s is.
            let newest = members
                .iter()
                .map(|b| b.mav_firmware_version)
                .max()
                .flatten();
            let mut chosen: Vec<&FirmwareInfo> = members
                .into_iter()
                .filter(|b| b.mav_firmware_version == newest)
                .collect();
            chosen.sort_by(|a, b| culture_order(a.format.as_deref(), b.format.as_deref()));
            out.extend(chosen);
        }
        out
    }

    /// `GetReleaseNewest`: the first record [`Manifest::release`] gives for each vehicle. The
    /// same as `Activate` computes for the page's labels.
    /// `// C#: ExtLibs/ArduPilot/APFirmware.cs:292-298; GCSViews/ConfigurationView/ConfigFirmwareManifest.cs:69-74`
    #[must_use]
    pub fn release_newest(&self, reltype: ReleaseType) -> Vec<&FirmwareInfo> {
        let mut out: Vec<&FirmwareInfo> = Vec::new();
        for record in self.release(reltype) {
            if !out.iter().any(|seen| seen.mav_type == record.mav_type) {
                out.push(record);
            }
        }
        out
    }

    /// The first record of [`Manifest::release_newest`] for a vehicle: what `Activate` writes
    /// under that vehicle's picture. `None` where the C#'s `First` throws.
    /// `// C#: GCSViews/ConfigurationView/ConfigFirmwareManifest.cs:76-92`
    #[must_use]
    pub fn newest(&self, reltype: ReleaseType, mav_type: MavType) -> Option<&FirmwareInfo> {
        self.release_newest(reltype)
            .into_iter()
            .find(|a| a.mav_type.as_deref() == Some(mav_type.name()))
    }

    /// `LookForPort`, up to the download: which device, which board ids, which records.
    ///
    /// The devices are taken in order. For each, the board id is `detected` - the bootloader's
    /// answer, when one was read - or else [`Manifest::board_ids`]; the first device with one
    /// (a detected id must not be zero) is the one. Its records are those with one of its board
    /// ids, the vehicle and the release. `all_options` is the "All Options" link: a blank device
    /// is added at the end, the first device is the one whatever its ids, and every record is a
    /// candidate. `None` is "Failed to detect port to upload to".
    /// `// C#: GCSViews/ConfigurationView/ConfigFirmwareManifest.cs:193-245, 356`
    #[must_use]
    pub fn look_for_port(
        &self,
        devices: &[DeviceInfo],
        detected: Option<i64>,
        mav_type: MavType,
        reltype: ReleaseType,
        all_options: bool,
    ) -> Option<Lookup> {
        let blank = DeviceInfo::default();
        let candidates = devices.iter().chain(all_options.then_some(&blank));
        for device in candidates {
            let devids = if detected.is_none() {
                self.board_ids(device, true)
            } else {
                None
            };
            if !(detected.is_some_and(|id| id != 0) || all_options || devids.is_some()) {
                continue;
            }
            let board_ids = match (devids, detected) {
                (Some(ids), _) => ids,
                (None, Some(id)) => vec![id],
                (None, None) => Vec::new(),
            };
            let items = if all_options {
                self.firmware.clone()
            } else {
                self.firmware
                    .iter()
                    .filter(|a| board_ids.contains(&a.board_id) && a.is(mav_type, reltype))
                    .cloned()
                    .collect()
            };
            return Some(Lookup {
                device: device.clone(),
                board_ids,
                items,
            });
        }
        None
    }
}

/// Each value once, in the order first seen: `Distinct()`.
fn distinct(values: impl Iterator<Item = i64>) -> Vec<i64> {
    let mut out = Vec::new();
    for value in values {
        if !out.contains(&value) {
            out.push(value);
        }
    }
    out
}

/// The `0x<vid>/0x<pid>` a hardware id names, by the C#'s
/// `VID_([0-9a-f]+)&PID_([0-9a-f]+)&` (ignoring case), the digits as written.
/// `// C#: ExtLibs/ArduPilot/APFirmware.cs:214-219, 256-261`
#[must_use]
pub fn vid_pid(hardwareid: &str) -> Option<String> {
    static PATTERN: std::sync::OnceLock<Option<regex::Regex>> = std::sync::OnceLock::new();
    let pattern = PATTERN
        .get_or_init(|| regex::Regex::new("(?i)VID_([0-9a-f]+)&PID_([0-9a-f]+)&").ok())
        .as_ref()?;
    let found = pattern.captures(hardwareid)?;
    Some(format!(
        "0x{}/0x{}",
        found.get(1)?.as_str(),
        found.get(2)?.as_str()
    ))
}

/// Whether one of a record's `USBID`s contains `lookfor`, ignoring case.
fn has_usbid(record: &FirmwareInfo, lookfor: &str) -> bool {
    let lookfor = lookfor.to_lowercase();
    record
        .usbid
        .iter()
        .any(|b| b.to_lowercase().contains(&lookfor))
}

/// `Comparer<string>.Default` for the strings the manifest's formats are made of: `null` first,
/// then letter by letter ignoring case, then lower case before upper at the first difference.
fn culture_order(a: Option<&str>, b: Option<&str>) -> std::cmp::Ordering {
    match (a, b) {
        (None, None) => std::cmp::Ordering::Equal,
        (None, Some(_)) => std::cmp::Ordering::Less,
        (Some(_), None) => std::cmp::Ordering::Greater,
        (Some(a), Some(b)) => a.to_lowercase().cmp(&b.to_lowercase()).then_with(|| {
            a.chars().zip(b.chars()).find(|(x, y)| x != y).map_or(
                std::cmp::Ordering::Equal,
                |(x, _)| {
                    if x.is_lowercase() {
                        std::cmp::Ordering::Less
                    } else {
                        std::cmp::Ordering::Greater
                    }
                },
            )
        }),
    }
}

/// What the label under a vehicle picture reads: vehicle, version, release.
///
/// The version is `mav-firmware-version-str` (`V4.7.1`) when the record has one, else
/// `mav-firmware-version` (`4.7.1`).
/// `// C#: GCSViews/ConfigurationView/ConfigFirmwareManifest.cs:108-113`
#[must_use]
pub fn icon_name(first: &FirmwareInfo) -> String {
    let vehicle = first.vehicle_type.as_deref().unwrap_or("");
    let release = first.mav_firmware_version_type.as_deref().unwrap_or("");
    match first
        .mav_firmware_version_str
        .as_deref()
        .filter(|text| !text.is_empty())
    {
        Some(text) => format!("{vehicle} {text} {release}"),
        None => format!(
            "{vehicle} {} {release}",
            first
                .mav_firmware_version
                .map(|v| v.to_string())
                .unwrap_or_default()
        ),
    }
}

/// What `LookForPort` settled on: the device, its board ids, and the records for them.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Lookup {
    /// The device.
    pub device: DeviceInfo,
    /// Its board ids.
    pub board_ids: Vec<i64>,
    /// `fwitems`: the records for those ids, the vehicle and the release, in manifest order.
    pub items: Vec<FirmwareInfo>,
}

/// What `LookForPort` does with its records.
/// `// C#: GCSViews/ConfigurationView/ConfigFirmwareManifest.cs:241-259`
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Outcome<'a> {
    /// One record: its URL, without asking.
    One(&'a FirmwareInfo),
    /// Several: `FirmwareSelection` opens.
    Choose(Selection<'a>),
    /// None: `Strings.No_firmware_available_for_this_board`.
    NoFirmware,
}

/// `Strings.No_firmware_available_for_this_board`.
/// `// C#: ExtLibs/Strings/Strings.resx:646-648`
pub const NO_FIRMWARE: &str = "No firmware available for this board!";

/// What `LookForPort` says when no device has a board id.
/// `// C#: GCSViews/ConfigurationView/ConfigFirmwareManifest.cs:356`
pub const NO_PORT: &str = "Failed to detect port to upload to (Unknown VID/PID or Board \
                           String)\r\nPlease try Disconnect/Reconnect and upload while on this screen";

impl Lookup {
    /// What happens next.
    #[must_use]
    pub fn outcome(&self) -> Outcome<'_> {
        match self.items.as_slice() {
            [] => Outcome::NoFirmware,
            [one] => Outcome::One(one),
            several => Outcome::Choose(Selection::new(several, &self.device)),
        }
    }

    /// The firmware Mission Planner would download: the single record, or the one
    /// `FirmwareSelection` has selected when it opens. `None` when there is nothing, or when the
    /// operator would have to pick.
    #[must_use]
    pub fn chosen(&self) -> Option<&FirmwareInfo> {
        match self.outcome() {
            Outcome::One(record) => Some(record),
            Outcome::Choose(selection) => selection.selected(),
            Outcome::NoFirmware => None,
        }
    }
}

/// `FirmwareSelection` as it opens for a device: "More than one choice exists. Please filter down
/// to the desired selection."
///
/// Its constructor lists every record, then sets the Platform picker to the device's product
/// string with `-BL` removed. That filters the list when it names one of the records' platforms,
/// and does nothing when it does not (a picker's `SelectedItem` cannot be an item it lacks). The
/// Board ID, Type, USB ID, Bootloader ID and Format pickers are hidden for a device, and Version
/// Type and Version when they offer one choice, as they do here. So the platform is the one
/// filter, and the Firmwares picker holds the URLs that pass it - selected when there is exactly
/// one.
/// `// C#: test/FirmwareSelection.xaml.cs:15-152`
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Selection<'a> {
    /// The Platform picker's choice, when it took one.
    pub platform: Option<String>,
    /// The records the pickers leave.
    pub matching: Vec<&'a FirmwareInfo>,
}

/// More than this many and the Firmwares picker says so instead of listing them.
/// `// C#: test/FirmwareSelection.xaml.cs:108`
pub const SELECTION_LIMIT: usize = 100;

impl<'a> Selection<'a> {
    /// The dialog as it opens for these records and this device.
    #[must_use]
    pub fn new(items: &'a [FirmwareInfo], device: &DeviceInfo) -> Self {
        // The Platform picker's items. `a.Platform.ToString()` throws on a null platform, which
        // `PopulatePicker` catches, leaving the picker empty.
        let platforms: Option<Vec<&str>> = items.iter().map(|a| a.platform.as_deref()).collect();
        let wanted = device
            .board
            .as_deref()
            .map(|board| board.replace("-BL", ""));
        let platform = wanted.filter(|wanted| {
            platforms
                .as_ref()
                .is_some_and(|listed| listed.contains(&wanted.as_str()))
        });
        let matching = items
            .iter()
            .filter(|a| {
                platform
                    .as_deref()
                    .is_none_or(|chosen| a.platform.as_deref() == Some(chosen))
            })
            .collect();
        Self { platform, matching }
    }

    /// The Firmwares picker's items: each URL, or one line saying there are too many or none.
    #[must_use]
    pub fn results(&self) -> Vec<String> {
        let count = self.matching.len();
        if count == 0 {
            vec!["No options to show".to_owned()]
        } else if count < SELECTION_LIMIT {
            self.matching
                .iter()
                .map(|a| a.url.clone().unwrap_or_default())
                .collect()
        } else {
            vec![format!("To many options - apply more filters - {count}")]
        }
    }

    /// The record selected as it opens: the only one, when there is only one.
    #[must_use]
    pub fn selected(&self) -> Option<&'a FirmwareInfo> {
        match self.matching.as_slice() {
            [only] => Some(only),
            _ => None,
        }
    }
}

/// Something that can fetch a URL's bytes: the network, or a file standing in for it.
pub trait Fetch {
    /// The body of a successful GET.
    ///
    /// # Errors
    /// Any failure, as text: the C# logs it and carries on.
    fn get(&self, url: &str) -> Result<Vec<u8>, String>;
}

/// The network, as `HttpClient.GetByteArrayAsync`: a GET that fails on a non-success status.
#[derive(Debug, Clone, Copy, Default)]
pub struct Http;

impl Fetch for Http {
    fn get(&self, url: &str) -> Result<Vec<u8>, String> {
        let mut response = ureq::get(url)
            .header("User-Agent", USER_AGENT)
            .config()
            .timeout_global(Some(TIMEOUT))
            .build()
            .call()
            .map_err(|err| err.to_string())?;
        response
            .body_mut()
            .with_config()
            .limit(MAX_DOWNLOAD)
            .read_to_vec()
            .map_err(|err| err.to_string())
    }
}

/// A manifest file standing in for the network ([`OVERRIDE_ENV`]).
#[derive(Debug, Clone)]
pub struct FromFile {
    /// The gzipped manifest.
    pub path: PathBuf,
}

impl Fetch for FromFile {
    fn get(&self, url: &str) -> Result<Vec<u8>, String> {
        if PAGE_SOURCES.contains(&url) {
            std::fs::read(&self.path).map_err(|err| format!("{}: {err}", self.path.display()))
        } else {
            Err(format!("not fetched: {OVERRIDE_ENV} is set"))
        }
    }
}

/// The network, or the file [`OVERRIDE_ENV`] names.
#[must_use]
pub fn fetcher() -> Box<dyn Fetch + Send + Sync> {
    match std::env::var_os(OVERRIDE_ENV) {
        Some(path) => Box::new(FromFile { path: path.into() }),
        None => Box::new(Http),
    }
}

/// `GetList`: fetches, unpacks and parses the manifest, then appends the peripheral manifest -
/// unless there is a manifest already and `force` is false. A failure leaves `manifest` as it was
/// and is returned for logging, as the C# catches and logs it; a failed append keeps the
/// manifest without it.
/// `// C#: ExtLibs/ArduPilot/APFirmware.cs:103-138`
pub fn get_list(
    manifest: &mut Option<Manifest>,
    url: &str,
    force: bool,
    fetch: &dyn Fetch,
) -> Vec<ManifestError> {
    if !force && manifest.is_some() {
        return Vec::new();
    }
    let fetched = fetch
        .get(url)
        .map_err(|reason| ManifestError::Http {
            url: url.to_owned(),
            reason,
        })
        .and_then(|bytes| Manifest::decode(&bytes, true));
    match fetched {
        Ok(fresh) => {
            *manifest = Some(fresh);
            get_list_append(manifest, PERIPH_URL, fetch)
                .err()
                .into_iter()
                .collect()
        }
        Err(err) => vec![err],
    }
}

/// `GetListAppend`: another manifest's records after the current one's.
/// `// C#: ExtLibs/ArduPilot/APFirmware.cs:140-182`
///
/// # Errors
/// If it cannot be fetched or parsed, or there is no manifest to append to.
pub fn get_list_append(
    manifest: &mut Option<Manifest>,
    url: &str,
    fetch: &dyn Fetch,
) -> Result<(), ManifestError> {
    let bytes = fetch.get(url).map_err(|reason| ManifestError::Http {
        url: url.to_owned(),
        reason,
    })?;
    let other = Manifest::decode(&bytes, url.ends_with(".gz"))?;
    let current = manifest
        .as_mut()
        .ok_or_else(|| ManifestError::NothingToAppend(url.to_owned()))?;
    current.append(other);
    Ok(())
}

/// What the Install Firmware page does to have a manifest: [`get_list`] over [`PAGE_SOURCES`],
/// the mirror and then ardupilot.org, the second doing nothing once the first has worked.
pub fn load(manifest: &mut Option<Manifest>, fetch: &dyn Fetch) -> Vec<ManifestError> {
    PAGE_SOURCES
        .iter()
        .flat_map(|url| get_list(manifest, url, false, fetch))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn versions_parse_and_order_as_system_version_does() {
        let v = |text| Version::parse(text).unwrap_or_else(|| panic!("{text}"));
        assert_eq!(v("4.7.1").to_string(), "4.7.1");
        assert_eq!(v("4.7").to_string(), "4.7");
        assert_eq!(v("1.2.3.4").to_string(), "1.2.3.4");
        assert_eq!(v(" 4 . 7 ").to_string(), "4.7");
        assert!(v("4.7") < v("4.7.0"), "an absent build is below zero");
        assert!(v("4.7.1") > v("4.6.9"));
        assert!(v("4.10.0") > v("4.9.0"), "numbers, not text");
        for bad in ["4", "4.7.1.2.3", "4.x", "4.-1", "", "4..1", "V4.7.1"] {
            assert_eq!(Version::parse(bad), None, "{bad:?}");
        }
    }

    #[test]
    fn release_and_vehicle_names_are_the_csharp_enums() {
        assert_eq!(
            ReleaseType::from_name("official"),
            Some(ReleaseType::Official)
        );
        assert_eq!(
            ReleaseType::from_name("stable"),
            Some(ReleaseType::Official)
        );
        assert_eq!(ReleaseType::from_name("LATEST"), Some(ReleaseType::Dev));
        assert_eq!(ReleaseType::from_name("rc"), None);
        assert_eq!(MavType::from_name("copter"), Some(MavType::Copter));
        assert_eq!(MavType::from_name("FIXED_WING"), Some(MavType::FixedWing));
        assert_eq!(MavType::Copter.name(), "Copter");
        assert_eq!(MavType::AntennaTracker.name(), "ANTENNA_TRACKER");
    }

    #[test]
    fn the_vid_pid_pattern_needs_the_ampersand_after_the_pid() {
        assert_eq!(
            vid_pid(r"USB\VID_26AC&PID_0011&REV_0200").as_deref(),
            Some("0x26AC/0x0011")
        );
        assert_eq!(
            vid_pid(r"usb\vid_2dae&pid_1016&mi_00").as_deref(),
            Some("0x2dae/0x1016")
        );
        assert_eq!(vid_pid(r"USB\VID_26AC&PID_0011"), None);
        assert_eq!(vid_pid(""), None);
    }

    #[test]
    fn formats_order_as_the_current_culture_orders_them() {
        let mut formats = vec!["hex", "ELF", "apj", "elf", "abin", "bin"];
        formats.sort_by(|a, b| culture_order(Some(a), Some(b)));
        assert_eq!(formats, ["abin", "apj", "bin", "elf", "ELF", "hex"]);
        assert_eq!(culture_order(None, Some("abin")), std::cmp::Ordering::Less);
    }

    #[test]
    fn fields_are_read_as_newtonsoft_reads_them() {
        let record = serde_json::json!({
            "board_id": 140,
            "mav-firmware-version-major": "4",
            "mav-firmware-version-minor": 7.0,
            "Platform": "CubeOrange",
            "USBID": null,
            "bootloader_str": ["CubeOrange-BL", null],
            "latest": 1,
            "mav-firmware-version": "4.7.1",
            "image_size": 12
        });
        let info = FirmwareInfo::from_json(&record).expect("parses");
        assert_eq!(info.board_id, 140);
        assert_eq!(info.mav_firmware_version_major, 4);
        assert_eq!(info.mav_firmware_version_minor, 7);
        assert_eq!(
            info.platform.as_deref(),
            Some("CubeOrange"),
            "case-insensitive fallback"
        );
        assert!(info.usbid.is_empty());
        assert_eq!(info.bootloader_str, ["CubeOrange-BL"]);
        assert_eq!(info.latest, 1);
        assert_eq!(info.mav_firmware_version, Version::parse("4.7.1"));

        let absent = FirmwareInfo::from_json(&serde_json::json!({})).expect("empty is fine");
        assert_eq!(absent, FirmwareInfo::default());

        for bad in [
            serde_json::json!({"latest": null}),
            serde_json::json!({"board_id": "one"}),
            serde_json::json!({"mav-firmware-version": "4"}),
            serde_json::json!({"mav-firmware-version": 4.7}),
            serde_json::json!({"USBID": "0x2dae/0x1016"}),
        ] {
            assert!(FirmwareInfo::from_json(&bad).is_err(), "{bad}");
        }
    }

    #[test]
    fn one_bad_record_fails_the_whole_manifest() {
        let text = br#"{"format-version":"1.0.0","firmware":[{"latest":0},{"latest":"x"}]}"#;
        let err = Manifest::parse(text).expect_err("record 1 is bad");
        assert!(err.to_string().contains("record 1"), "{err}");
        let text = b"\xEF\xBB\xBF{\"format-version\":\"1.0.0\",\"firmware\":[]}";
        let manifest = Manifest::parse(text).expect("a BOM is dropped");
        assert_eq!(manifest.format_version, Version::parse("1.0.0"));
        assert!(Manifest::parse(br#"{"format-version":"1.0.0"}"#).is_err());
    }
}
